// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024
 *
 * HEVC/H.265 decoder, using VDEC_HEVC engine.
 * Based on BSP vh265.c and upstream codec_vp9.c pattern.
 */

#include <linux/module.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "dos_regs.h"
#include "hevc_regs.h"
#include "codec_hevc.h"
#include "codec_hevc_bsp.h"
#include "codec_hevc_fw.h"
#include "vdec_helpers.h"
#include "vdec_debugfs.h"
#include "vdec_hevc.h"
#include "codec_hevc_common.h"

/* HEVC register mapping (same HEVC_ASSIST_SCRATCH_* as VP9 but different use) */
#define HEVC_DEC_STATUS_REG	HEVC_ASSIST_SCRATCH_0
#define HEVC_RPM_BUFFER		HEVC_ASSIST_SCRATCH_1
#define HEVC_SHORT_TERM_RPS	HEVC_ASSIST_SCRATCH_2
#define HEVC_VPS_BUFFER		HEVC_ASSIST_SCRATCH_3
#define HEVC_SPS_BUFFER		HEVC_ASSIST_SCRATCH_4
#define HEVC_PPS_BUFFER		HEVC_ASSIST_SCRATCH_5
#define HEVC_SAO_UP		HEVC_ASSIST_SCRATCH_6
#define HEVC_STREAM_SWAP_BUFFER	HEVC_ASSIST_SCRATCH_7
#define HEVC_STREAM_SWAP_BUFFER2 HEVC_ASSIST_SCRATCH_8
#define HEVC_sCALELUT		HEVC_ASSIST_SCRATCH_D
#define HEVC_WAIT_FLAG		HEVC_ASSIST_SCRATCH_E
#define LMEM_DUMP_ADR		HEVC_ASSIST_SCRATCH_F
#define NAL_SEARCH_CTL		HEVC_ASSIST_SCRATCH_I
#define HEVC_DECODE_MODE	HEVC_ASSIST_SCRATCH_J
#define DECODE_STOP_POS		HEVC_ASSIST_SCRATCH_K
#define HEVC_DECODE_COUNT	HEVC_ASSIST_SCRATCH_M
#define HEVC_DECODE_SIZE	HEVC_ASSIST_SCRATCH_N

/* Decode status values from firmware */
#define HEVC_ACTION_DONE			0xff
#define HEVC_CODED_SLICE_SEGMENT_DAT		0x5
#define HEVC_SLICE_SEGMENT_DONE			0x8
#define HEVC_NAL_SEARCH_DONE			0x9  /* Firmware found next NAL header */
#define HEVC_DECPIC_DATA_DONE			0xa
#define HEVC_DECPIC_DATA_ERROR			0xb
#define HEVC_NAL_DECODE_DONE			0xe
#define HEVC_OVER_DECODE			0xf
#define HEVC_DECODE_BUFEMPTY			0x20
#define HEVC_SEARCH_BUFEMPTY			0x22
#define HEVC_DECODE_OVER_SIZE			0x23
#define HEVC_DECODE_BUFEMPTY2			0x24
#define HEVC_FIND_NEXT_PIC_NAL			0x50
#define HEVC_HEAD_PARSER_DONE			0xf0

/* Firmware interrupt request */
#define AMRISC_MAIN_REQ				0x04

/* Driver -> firmware responses */
#define HEVC_ACTION_DEC_CONT			0xfd

/* Decode mode */
#define DECODE_MODE_SINGLE			0

/* Write the MMU map address to the correct register.
 * G12A+ has a dedicated HEVC_ASSIST_MMU_MAP_ADDR register.
 * GXL uses HEVC_ASSIST_SCRATCH_7 (shared with stream swap buffer).
 * BSP: H265_MMU_MAP_BUFFER = HEVC_ASSIST_SCRATCH_7 for pre-G12A.
 */
static inline void codec_hevc_write_mmu_map_addr(struct amvdec_core *core,
						  dma_addr_t paddr)
{
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, HEVC_ASSIST_MMU_MAP_ADDR, paddr);
	else
		amvdec_write_dos(core, HEVC_ASSIST_SCRATCH_7, paddr);
}

/* Returns true if GXL HEVC needs legacy compressed reference mode
 * (without MMU).  Now that MMU is enabled on all platforms, this
 * should always return false -- MMU mode handles FBC internally.
 * Kept for reference; the code paths guarded by this are dead.
 */
static inline bool hevc_needs_compress_ref(struct amvdec_core *core,
					   int is_10bit)
{
	return false;
}

/* HEVC workspace layout - same structure as VP9 */
#define LCU_SIZE		64
#define MAX_REF_PIC_NUM		24

#define MPRED_MV_BUF_SIZE	0x120000

#define IPP_SIZE	0x4000
#define SAO_ABV_SIZE	0x30000
#define SAO_VB_SIZE	0x30000
#define SH_TM_RPS_SIZE	0x800
#define VPS_SIZE	0x800
#define SPS_SIZE	0x800
#define PPS_SIZE	0x2000
#define SAO_UP_SIZE	0x2800
#define SWAP_BUF_SIZE	0x800
#define SWAP_BUF2_SIZE	0x800
#define SCALELUT_SIZE	0x8000
#define DBLK_PARA_SIZE	0x80000
#define DBLK_DATA_SIZE	0x80000
#define DBLK_DATA2_SIZE	0x80000
#define MMU_VBH_SIZE	0x5000
#define MPRED_ABV_SIZE	0x10000
#define MPRED_MV_SIZE	(MPRED_MV_BUF_SIZE * MAX_REF_PIC_NUM)
#define RPM_BUF_SIZE	0x100
#define LMEM_SIZE	0x800

#define IPP_OFFSET       0x00
#define SAO_ABV_OFFSET   (IPP_OFFSET + IPP_SIZE)
#define SAO_VB_OFFSET    (SAO_ABV_OFFSET + SAO_ABV_SIZE)
#define SH_TM_RPS_OFFSET (SAO_VB_OFFSET + SAO_VB_SIZE)
#define VPS_OFFSET       (SH_TM_RPS_OFFSET + SH_TM_RPS_SIZE)
#define SPS_OFFSET       (VPS_OFFSET + VPS_SIZE)
#define PPS_OFFSET       (SPS_OFFSET + SPS_SIZE)
#define SAO_UP_OFFSET    (PPS_OFFSET + PPS_SIZE)
#define SWAP_BUF_OFFSET  (SAO_UP_OFFSET + SAO_UP_SIZE)
#define SWAP_BUF2_OFFSET (SWAP_BUF_OFFSET + SWAP_BUF_SIZE)
#define SCALELUT_OFFSET  (SWAP_BUF2_OFFSET + SWAP_BUF2_SIZE)
#define DBLK_PARA_OFFSET (SCALELUT_OFFSET + SCALELUT_SIZE)
#define DBLK_DATA_OFFSET (DBLK_PARA_OFFSET + DBLK_PARA_SIZE)
#define DBLK_DATA2_OFFSET (DBLK_DATA_OFFSET + DBLK_DATA_SIZE)
#define MMU_VBH_OFFSET   (DBLK_DATA2_OFFSET + DBLK_DATA2_SIZE)
#define MPRED_ABV_OFFSET (MMU_VBH_OFFSET + MMU_VBH_SIZE)
#define MPRED_MV_OFFSET  (MPRED_ABV_OFFSET + MPRED_ABV_SIZE)
#define RPM_OFFSET       (MPRED_MV_OFFSET + MPRED_MV_SIZE)
#define LMEM_OFFSET      (RPM_OFFSET + RPM_BUF_SIZE)

#define SIZE_WORKSPACE	ALIGN(LMEM_OFFSET + LMEM_SIZE, 64 * SZ_1K)

/* RPM parameter structure - from firmware LMEM, must match BSP layout */
#define RPM_BEGIN 0x100
#define RPM_END   0x180

/* MISC_FLAG0 bit definitions (from BSP vh265.c) */
#define PCM_LOOP_FILTER_DISABLED_FLAG_BIT		0
#define PCM_ENABLE_FLAG_BIT				1
#define LOOP_FILER_ACROSS_TILES_ENABLED_FLAG_BIT	2
#define PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT	3
#define DEBLOCKING_FILTER_OVERRIDE_ENABLED_FLAG_BIT	4
#define PPS_DEBLOCKING_FILTER_DISABLED_FLAG_BIT		5
#define DEBLOCKING_FILTER_OVERRIDE_FLAG_BIT		6
#define SLICE_DEBLOCKING_FILTER_DISABLED_FLAG_BIT	7
#define SLICE_SAO_LUMA_FLAG_BIT				8
#define SLICE_SAO_CHROMA_FLAG_BIT			9
#define SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT 10

union hevc_rpm_param {
	struct {
		u16 data[RPM_END - RPM_BEGIN];
	} l;
	struct {
		/* from ucode lmem, do not change this struct */
		u16 CUR_RPS[0x10];
		u16 num_ref_idx_l0_active;
		u16 num_ref_idx_l1_active;
		u16 slice_type;
		u16 slice_temporal_mvp_enable_flag;
		u16 dependent_slice_segment_flag;
		u16 slice_segment_address;
		u16 num_tile_rows_minus1;
		u16 pic_width_in_luma_samples;
		u16 pic_height_in_luma_samples;
		u16 log2_min_coding_block_size_minus3;
		u16 log2_diff_max_min_coding_block_size;
		u16 log2_max_pic_order_cnt_lsb_minus4;
		u16 POClsb;
		u16 collocated_from_l0_flag;
		u16 collocated_ref_idx;
		u16 log2_parallel_merge_level;
		u16 five_minus_max_num_merge_cand;
		u16 sps_num_reorder_pics_0;
		u16 modification_flag;
		u16 tiles_enabled_flag;
		u16 num_tile_columns_minus1;
		u16 num_tile_rows_minus1_2;
		u16 tile_width[12];
		u16 tile_height[8];
		u16 misc_flag0;
		u16 pps_beta_offset_div2;
		u16 pps_tc_offset_div2;
		u16 slice_beta_offset_div2;
		u16 slice_tc_offset_div2;
		u16 pps_cb_qp_offset;
		u16 pps_cr_qp_offset;
		u16 first_slice_segment_in_pic_flag;
		u16 m_temporalId;
		u16 m_nalUnitType;
		u16 vui_num_units_in_tick_hi;
		u16 vui_num_units_in_tick_lo;
		u16 vui_time_scale_hi;
		u16 vui_time_scale_lo;
		u16 bit_depth;
		u16 profile_etc;
		u16 sei_frame_field_info;
		u16 video_signal_type;
		u16 modification_list[0x20];
		u16 conformance_window_flag;
		u16 conf_win_left_offset;
		u16 conf_win_right_offset;
		u16 conf_win_top_offset;
		u16 conf_win_bottom_offset;
		u16 chroma_format_idc;
		u16 color_description;
		u16 aspect_ratio_idc;
		u16 sar_width;
		u16 sar_height;
		u16 sps_max_dec_pic_buffering_minus1_0;
	} p;
};

struct hevc_frame {
	struct list_head list;
	struct vb2_v4l2_buffer *vbuf;
	int index;
	int show;
	int done;
	int output_ready;	/* 1 = decode complete, eligible for POC-ordered output */
	unsigned int width;
	unsigned int height;
	int poc;
};

struct codec_hevc {
	struct mutex lock;
	struct codec_hevc_common common;

	void      *workspace_vaddr;
	dma_addr_t workspace_paddr;

	/* BSP-protocol DMA buffers live in struct codec_hevc_common so
	 * codec_hevc_bsp.c can access them via the shared-header layout
	 * without exposing the full struct codec_hevc.  hevc_bsp path
	 * owns their lifecycle via codec_hevc_bsp_session_start/stop.
	 */

	union hevc_rpm_param rpm_param;

	/* Cached SPS num_reorder_pics for POC-ordered output bumping.
	 * HEVC B-frame streams are encoded in decode order but must be
	 * displayed in POC order. This is the DPB bumping threshold:
	 * when we have more than this many decode-complete frames buffered,
	 * release the lowest-POC one. 0 for IPPP streams (no reorder).
	 * See hevc-display-reorder-plan.md.
	 */
	u16 sps_num_reorder_pics_0;

	int bitdepth_offset;   /* u16 offset of bit_depth in RPM LMEM (firmware-dependent) */
	int is_10bit;

	u32 width, height;

	struct list_head ref_frames_list;
	u32 frames_num;

	u32 lcu_total;

	struct hevc_frame *cur_frame;
	struct hevc_frame *prev_frame;

	int max_ra;

	int new_pic;	/* 1 = first slice of new picture, 0 = continuation */
	int slice_idx;	/* current slice index within picture */

	u32 dec_status; /* saved in hardirq for threaded handler */

	/* ISR rate limiting: count consecutive DONE interrupts with no
	 * SAO progress. If this exceeds a threshold, abort to prevent
	 * the system from becoming unresponsive.
	 */
	u32 done_count;
	u32 last_sao_y_wptr;

	/* POC tracking for MPRED temporal motion vector prediction */
	int curr_poc;
	int iPrevTid0POC;

	/* Per-slice reference picture lists (built from CUR_RPS[]) */
	int ref_poc_list0[16];	/* L0 reference POCs */
	int ref_poc_list1[16];	/* L1 reference POCs */
	int ref_num_l0;		/* number of active L0 references */
	int ref_num_l1;		/* number of active L1 references */
	/* apply_rps intermediate state -- preserved for mpred_trace debugfs.
	 * st_curr0/st_curr1 reflect the RPS classification (past / future
	 * in POC order); num_neg/num_pos their counts.  Diagnostic only.
	 */
	int st_curr0_snap[16];
	int st_curr1_snap[16];
	int num_neg_snap;
	int num_pos_snap;

	unsigned long ref_held;	/* bitmap of capture buffers held as refs */

	/* Deferred ACTION_DONE: after outputting a frame, delay sending
	 * ACTION_DONE to the firmware by 1ms to give userspace a chance
	 * to recycle capture buffers.  Without this, the threaded ISR
	 * processes all frames back-to-back at RT priority, starving
	 * userspace and exhausting capture buffers.
	 */
	struct amvdec_session *sess;
	struct delayed_work action_done_work;
};

/* Find the V4L2 buffer index for a reference picture by POC.
 * Returns the buffer index, or -1 if not found.
 */
static int codec_hevc_find_buf_index(struct codec_hevc *hevc, int poc)
{
	struct hevc_frame *frame;

	list_for_each_entry(frame, &hevc->ref_frames_list, list) {
		if (frame->poc == poc)
			return frame->index;
	}
	return -1;
}

/* Decode CUR_RPS[], build L0/L1 reference lists, and mark ref_held.
 *
 * This combines the BSP's read_ref_pic_set() + set_ref_pic_list() logic.
 * Decodes CUR_RPS[] into StCurr0 (past refs) and StCurr1 (future refs),
 * then builds ordered L0/L1 lists per HEVC spec 8.3.4.
 *
 * Also sets ref_held bitmap to prevent get_new_frame() from recycling
 * capture buffers whose FBC body is still needed as a reference.
 *
 * Must be called after codec_hevc_fetch_rpm() and codec_hevc_compute_poc().
 */
static void codec_hevc_apply_rps(struct codec_hevc *hevc)
{
	union hevc_rpm_param *param = &hevc->rpm_param;
	int cur_poc = hevc->curr_poc;
	unsigned long new_held = 0;
	int st_curr0[16];  /* past reference POCs (negative delta) */
	int st_curr1[16];  /* future reference POCs (positive delta) */
	int num_neg = 0, num_pos = 0;
	int total_num;
	int num_l0_active, num_l1_active;
	u32 slice_type = param->p.slice_type;
	u32 mod_flag = param->p.modification_flag;
	int i;

	/* Step 1: Decode CUR_RPS[] into StCurr0 (past) and StCurr1 (future).
	 * Only entries with the "used by current" bit (bit 13) are included
	 * in the reference lists.  Entries without bit 13 are part of the RPS
	 * but not used as references for this picture.
	 */
	for (i = 0; i < 16; i++) {
		u16 rps = param->p.CUR_RPS[i];
		int delt, poc_tmp;

		if (rps == 0x8000)     /* RPS_END terminator */
			break;

		/* CUR_RPS format -- BSP vh265.c:898 non-LT layout
		 * (SUPPORT_LONG_TERM_RPS NOT defined).  The upstream firmware
		 * we ship emits this layout:
		 *   Bit 14: RPS_USED_BIT (used by current picture)
		 *   Bit 13: RPS_SIGN_BIT (1 = negative/past delta)
		 *   Bits 12..0: absolute delta POC (13-bit)
		 *
		 * Prior to 2026-04-19, the driver used the LT variant
		 * (USED=13, SIGN=12, LT=14) which caused future references to
		 * have bit 13 clear -> skipped as "not used" -> st_curr1
		 * stayed empty -> L1 filled from st_curr0 fallback, giving
		 * every B-slice the wrong collocated POC and ~20 dB PSNR
		 * drift on B-frame content.  See
		 * decoder/hevc/bframe-content-analysis.md "Strike 3".
		 */
		delt = rps & 0x1FFF;   /* bits 12..0: absolute delta POC */
		if (rps & BIT(13))     /* bit 13: sign (1 = past ref) */
			poc_tmp = cur_poc - (0x2000 - delt);
		else
			poc_tmp = cur_poc + delt;

		/* Bit 14: used_by_curr_pic_flag.  Only "used" refs go into
		 * the reference lists.  All RPS entries (used or not) protect
		 * their buffers via ref_held.
		 */
		if (rps & BIT(14)) {
			if (rps & BIT(13))
				st_curr0[num_neg++] = poc_tmp;
			else
				st_curr1[num_pos++] = poc_tmp;
		}

		/* Mark buffer as held regardless of used_by_curr flag */
		{
			int idx = codec_hevc_find_buf_index(hevc, poc_tmp);

			if (idx >= 0)
				set_bit(idx, &new_held);
		}
	}

	hevc->ref_held = new_held;

	/* Snapshot intermediate RPS classification for mpred_trace debugfs.
	 * Done before L0/L1 construction so the trace shows the firmware-
	 * reported past/future split independent of list-building bugs.
	 */
	memcpy(hevc->st_curr0_snap, st_curr0, sizeof(st_curr0));
	memcpy(hevc->st_curr1_snap, st_curr1, sizeof(st_curr1));
	hevc->num_neg_snap = num_neg;
	hevc->num_pos_snap = num_pos;

	/* Step 2: Build L0 and L1 reference picture lists (HEVC spec 8.3.4).
	 * I-slices have no references.  P-slices use L0 only.  B-slices use
	 * both L0 and L1.
	 */
	hevc->ref_num_l0 = 0;
	hevc->ref_num_l1 = 0;
	memset(hevc->ref_poc_list0, 0, sizeof(hevc->ref_poc_list0));
	memset(hevc->ref_poc_list1, 0, sizeof(hevc->ref_poc_list1));

	if (slice_type == 2)  /* I-slice */
		return;

	total_num = num_neg + num_pos;
	if (total_num == 0)
		return;

	num_l0_active = param->p.num_ref_idx_l0_active;
	num_l1_active = (slice_type == 0) ? param->p.num_ref_idx_l1_active : 0;

	if (num_l0_active > 16)
		num_l0_active = 16;
	if (num_l1_active > 16)
		num_l1_active = 16;

	/* L0: StCurr0 (past) first, then StCurr1 (future), cycling */
	if (mod_flag & 1) {
		/* Reference picture list modification for L0 */
		for (i = 0; i < num_l0_active; i++) {
			int mod_idx = param->p.modification_list[i];
			int combined_poc;

			/* modification_list index into {StCurr0, StCurr1} */
			if (mod_idx < num_neg)
				combined_poc = st_curr0[mod_idx];
			else if (mod_idx < total_num)
				combined_poc = st_curr1[mod_idx - num_neg];
			else
				combined_poc = st_curr0[0]; /* fallback */
			hevc->ref_poc_list0[i] = combined_poc;
		}
	} else {
		for (i = 0; i < num_l0_active; i++) {
			int c_idx = i % total_num;

			if (c_idx < num_neg)
				hevc->ref_poc_list0[i] = st_curr0[c_idx];
			else
				hevc->ref_poc_list0[i] = st_curr1[c_idx - num_neg];
		}
	}
	hevc->ref_num_l0 = num_l0_active;

	/* L1 (B-slice only): StCurr1 (future) first, then StCurr0 (past) */
	if (slice_type == 0 && num_l1_active > 0) {
		if (mod_flag & 2) {
			/* Reference picture list modification for L1 */
			int mod_off = (mod_flag & 1) ? num_l0_active : 0;

			for (i = 0; i < num_l1_active; i++) {
				int mod_idx = param->p.modification_list[mod_off + i];
				int combined_poc;

				/* For L1, combined = {StCurr1, StCurr0} */
				if (mod_idx < num_pos)
					combined_poc = st_curr1[mod_idx];
				else if (mod_idx < total_num)
					combined_poc = st_curr0[mod_idx - num_pos];
				else
					combined_poc = st_curr1[0]; /* fallback */
				hevc->ref_poc_list1[i] = combined_poc;
			}
		} else {
			for (i = 0; i < num_l1_active; i++) {
				int c_idx = i % total_num;

				if (c_idx < num_pos)
					hevc->ref_poc_list1[i] = st_curr1[c_idx];
				else
					hevc->ref_poc_list1[i] = st_curr0[c_idx - num_pos];
			}
		}
		hevc->ref_num_l1 = num_l1_active;
	}
}

static void codec_hevc_output_frame(struct amvdec_session *sess,
				    struct hevc_frame *frame);
static void codec_hevc_bump_frames(struct amvdec_session *sess, bool flush);

static void codec_hevc_flush_output(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;
	struct hevc_frame *tmp, *n;

	/*
	 * Only halt firmware during stream teardown (STREAMOFF), not during
	 * normal EOS drain.  Writing 0 to HEVC_DECODE_SIZE stops the firmware
	 * from processing queued frames.  During normal drain, we need the
	 * firmware to keep decoding so all buffered frames are delivered.
	 * During teardown, halt to prevent use-after-free via prev_frame.
	 */
	if (!sess->streamon_out)
		amvdec_write_dos(core, HEVC_DECODE_SIZE, 0);

	/*
	 * Wait for the SAO/deblock double-write pipeline to finish.
	 * The HEVC firmware decodes into FBC compressed buffers, and the
	 * SAO post-processor writes the final NV12/YUV420 pixels via DMA.
	 * If we deliver buffers before SAO completes, userspace gets
	 * partial/stale pixel data (KI-061: last frame ~41 dB PSNR).
	 *
	 * Poll HEVC_SAO_IF_STATUS (idle when all bits clear) and verify
	 * both Y and C write pointers have reached their target addresses.
	 * Same approach as codec_vp9_flush_output().  Timeout after 100ms.
	 */
	{
		int i;
		u32 sao_if, y_wptr, c_wptr, y_start, y_len, c_start, c_len;

		for (i = 0; i < 2000; i++) {
			sao_if = amvdec_read_dos(core, HEVC_SAO_IF_STATUS);
			y_wptr = amvdec_read_dos(core, HEVC_SAO_Y_WPTR);
			c_wptr = amvdec_read_dos(core, HEVC_SAO_C_WPTR);
			y_start = amvdec_read_dos(core,
						  HEVC_SAO_Y_START_ADDR);
			y_len = amvdec_read_dos(core, HEVC_SAO_Y_LENGTH);
			c_start = amvdec_read_dos(core,
						  HEVC_SAO_C_START_ADDR);
			c_len = amvdec_read_dos(core, HEVC_SAO_C_LENGTH);

			if (!sao_if &&
			    y_wptr >= y_start + y_len &&
			    c_wptr >= c_start + c_len)
				break;
			usleep_range(50, 100);
		}
		dev_info(core->dev_dec,
			 "HEVC drain: SAO settled in %d loops "
			 "(if=0x%x Y=%x/%x C=%x/%x)\n",
			 i, sao_if,
			 y_wptr, y_start + y_len,
			 c_wptr, c_start + c_len);
	}

	mutex_lock(&hevc->lock);

	/*
	 * Mark cur_frame ready for output if still pending.  During
	 * normal decoding, frame output is deferred: SLICE_SEGMENT_DONE
	 * marks the PREVIOUS frame output_ready when it sees
	 * slice_segment_address==0.  The last decoded frame stays in
	 * cur_frame with no subsequent picture to trigger its output.
	 * NAL_DECODE_DONE handles this during normal playback, but the
	 * last frame at EOS may never receive a NAL_DECODE_DONE if the
	 * firmware stalls waiting for more data.  Mark it here before
	 * draining the reorder buffer.
	 */
	if (hevc->cur_frame && !hevc->cur_frame->done) {
		hevc->cur_frame->output_ready = 1;
		hevc->cur_frame = NULL;
	}

	/*
	 * Drain the reorder buffer in POC order.  Pick the lowest-POC
	 * not-yet-done frame and output it; repeat until empty.  This
	 * mirrors BSP vh265.c:output_pic_display() with flush_flag=1.
	 * The pre-existing list-iteration-order drain released B-frame
	 * streams in decode order, producing the same wrong output as
	 * the decode-order ISR path (KI-001).
	 */
	for (;;) {
		struct hevc_frame *best = NULL;

		list_for_each_entry(tmp, &hevc->ref_frames_list, list) {
			if (tmp->done)
				continue;
			if (!best || tmp->poc < best->poc)
				best = tmp;
		}
		if (!best)
			break;
		if (best->show)
			amvdec_dst_buf_done(sess, best->vbuf,
					    V4L2_FIELD_NONE);
		else
			v4l2_m2m_buf_queue(sess->m2m_ctx, best->vbuf);
		best->done = 1;
		hevc->frames_num--;
	}

	/* Now release the list entries themselves. */
	list_for_each_entry_safe(tmp, n, &hevc->ref_frames_list, list) {
		list_del(&tmp->list);
		kfree(tmp);
	}
	hevc->cur_frame = NULL;
	hevc->prev_frame = NULL;
	hevc->ref_held = 0;
	mutex_unlock(&hevc->lock);

	/*
	 * The HEVC firmware in SINGLE decode mode has no bitstream-level
	 * EOS mechanism (unlike MPEG1/2 sequence_end_code or H.264 EOS
	 * NAL).  After the last frame, the firmware sits idle polling the
	 * VIFIFO for more data.  esparser_queued_bufs may still be
	 * positive because the firmware consumed the data (BUFEMPTY fired)
	 * but no NAL_DECODE_DONE decremented the counter.
	 *
	 * Force the counter to zero so the post-drain wait in
	 * vdec_drain_work() exits immediately instead of stalling for 5s
	 * (KI-033).
	 */
	if (atomic_read(&sess->esparser_queued_bufs) > 0) {
		dev_dbg(core->dev,
			"HEVC drain: forcing queued count from %d to 0\n",
			atomic_read(&sess->esparser_queued_bufs));
		atomic_set(&sess->esparser_queued_bufs, 0);
	}
}

static u32 codec_hevc_num_pending_bufs(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;

	if (!hevc)
		return 0;

	return hevc->frames_num;
}

static int codec_hevc_alloc_workspace(struct amvdec_core *core,
				      struct codec_hevc *hevc)
{
	hevc->workspace_vaddr = dma_alloc_coherent(core->dev, SIZE_WORKSPACE,
						   &hevc->workspace_paddr,
						   GFP_KERNEL);
	if (!hevc->workspace_vaddr) {
		dev_err(core->dev, "Failed to allocate HEVC Workspace\n");
		return -ENOMEM;
	}

	return 0;
}

static void codec_hevc_setup_workspace(struct amvdec_session *sess,
				       struct codec_hevc *hevc)
{
	struct amvdec_core *core = sess->core;
	dma_addr_t wkaddr = hevc->workspace_paddr;

	amvdec_write_dos(core, HEVCD_IPP_LINEBUFF_BASE, wkaddr + IPP_OFFSET);
	amvdec_write_dos(core, HEVC_RPM_BUFFER, wkaddr + RPM_OFFSET);
	amvdec_write_dos(core, HEVC_SHORT_TERM_RPS, wkaddr + SH_TM_RPS_OFFSET);
	amvdec_write_dos(core, HEVC_VPS_BUFFER, wkaddr + VPS_OFFSET);
	amvdec_write_dos(core, HEVC_SPS_BUFFER, wkaddr + SPS_OFFSET);
	amvdec_write_dos(core, HEVC_PPS_BUFFER, wkaddr + PPS_OFFSET);
	amvdec_write_dos(core, HEVC_SAO_UP, wkaddr + SAO_UP_OFFSET);

	/* SCRATCH_7 (HEVC_STREAM_SWAP_BUFFER):
	 * On pre-G12A with MMU firmware, SCRATCH_7 is repurposed as the
	 * MMU page table map address (H265_MMU_MAP_BUFFER in BSP vh265.c:113).
	 * The MMU firmware reads this at startup, so we must write the
	 * pre-allocated MMU map address here before firmware starts.
	 * On G12A+, a dedicated HEVC_ASSIST_MMU_MAP_ADDR register is used
	 * and SCRATCH_7 holds the swap buffer workspace address.
	 */
	if (codec_hevc_use_mmu(core->platform->revision, sess->pixfmt_cap,
			       hevc->is_10bit) &&
	    core->platform->revision < VDEC_REVISION_G12A)
		amvdec_write_dos(core, HEVC_STREAM_SWAP_BUFFER,
				 hevc->common.mmu_map_paddr);
	else
		amvdec_write_dos(core, HEVC_STREAM_SWAP_BUFFER,
				 wkaddr + SWAP_BUF_OFFSET);
	amvdec_write_dos(core, HEVC_STREAM_SWAP_BUFFER2,
			 wkaddr + SWAP_BUF2_OFFSET);
	amvdec_write_dos(core, HEVC_sCALELUT, wkaddr + SCALELUT_OFFSET);

	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, HEVC_DBLK_CFGE,
				 wkaddr + DBLK_DATA2_OFFSET);

	amvdec_write_dos(core, HEVC_DBLK_CFG4, wkaddr + DBLK_PARA_OFFSET);
	amvdec_write_dos(core, HEVC_DBLK_CFG5, wkaddr + DBLK_DATA_OFFSET);
	amvdec_write_dos(core, LMEM_DUMP_ADR, wkaddr + LMEM_OFFSET);

	/* BSP-protocol: program HEVC_RPM_BUFFER (= HEVC_ASSIST_SCRATCH_1)
	 * with the per-session RPM DMA buffer addr.  Only fires under
	 * BSP_PROTO; upstream firmware doesn't read this register.  This
	 * MUST happen before MPSR=1 so the firmware's first slice handler
	 * has a valid DMA target.  See codec_hevc_bsp.c.
	 */
	/* Dispatch to hevc_bsp setup_workspace hook under BSP_PROTO.
	 * Programs RPM / AUX / MPRED-init / SCRATCH_H registers.
	 * All BSP-protocol workspace programming lives in
	 * codec_hevc_bsp.c; this is the only gate at the
	 * setup_workspace level.
	 */
	if (sess->fw_match &&
	    (sess->fw_match->quirks & AMVDEC_FW_QUIRK_HEVC_BSP_PROTO))
		codec_hevc_bsp_session_setup_workspace(sess, &hevc->common,
				wkaddr + MPRED_MV_OFFSET,
				MPRED_MV_BUF_SIZE,
				wkaddr + MPRED_ABV_OFFSET);

	/* SAO buffer configuration for firmware (BSP config_title_hw) */
	amvdec_write_dos(core, HEVC_ASSIST_SCRATCH_9,
			 SAO_ABV_SIZE / LCU_SIZE);   /* sao_mem_unit */
	amvdec_write_dos(core, HEVC_ASSIST_SCRATCH_A,
			 wkaddr + SAO_ABV_OFFSET);   /* SAO ABV start */
	amvdec_write_dos(core, HEVC_ASSIST_SCRATCH_B,
			 SAO_VB_SIZE / LCU_SIZE);    /* sao_vb_size */
	amvdec_write_dos(core, HEVC_ASSIST_SCRATCH_C,
			 wkaddr + SAO_VB_OFFSET);    /* SAO VB start */

	if (codec_hevc_use_mmu(core->platform->revision, sess->pixfmt_cap,
			       hevc->is_10bit)) {
		amvdec_write_dos(core, HEVC_SAO_MMU_VH0_ADDR,
				 wkaddr + MMU_VBH_OFFSET);
		amvdec_write_dos(core, HEVC_SAO_MMU_VH1_ADDR,
				 wkaddr + MMU_VBH_OFFSET + (MMU_VBH_SIZE / 2));

		/* Write MMU map address.  The MMU map is pre-allocated
		 * in codec_hevc_start() so this address is always valid.
		 * On GXL this overwrites HEVC_ASSIST_SCRATCH_7 (stream
		 * swap buffer) -- the MMU firmware uses SCRATCH_7 for
		 * the MMU map, not the swap buffer.
		 */
		codec_hevc_write_mmu_map_addr(core,
					      hevc->common.mmu_map_paddr);
	}
}

static void codec_hevc_init_decoder(struct amvdec_core *core,
				    struct codec_hevc *hevc)
{
	/* Force clocks on for ALL HEVC submodules (BSP: CONFIG_HEVC_CLK_FORCED_ON).
	 * Without these, the SAO DMA output path is clock-gated off and
	 * produces zero pixel output.
	 */
	amvdec_write_dos_bits(core, HEVC_IQIT_CLK_RST_CTRL, BIT(2));
	amvdec_write_dos_bits(core, HEVC_DBLK_CFG0, BIT(2));
	amvdec_write_dos_bits(core, HEVC_SAO_CTRL1, BIT(2));
	amvdec_write_dos_bits(core, HEVC_MPRED_CTRL1, BIT(24));
	/* Parser/stream/CABAC clock force-on */
	amvdec_write_dos_bits(core, HEVC_STREAM_CONTROL, BIT(15));
	amvdec_write_dos_bits(core, HEVC_SHIFT_CONTROL, BIT(15));
	amvdec_write_dos_bits(core, HEVC_CABAC_CONTROL, BIT(13));
	amvdec_write_dos_bits(core, HEVC_PARSER_CORE_CONTROL, BIT(15));
	amvdec_write_dos_bits(core, HEVC_PARSER_INT_CONTROL, BIT(15));
	/* Parser IF control: force-on clocks for all parser sub-blocks
	 * BIT(5..0) = individual sub-block clock enables
	 */
	amvdec_write_dos_bits(core, HEVC_PARSER_IF_CONTROL,
			      BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0));
	/* IPP dynamic clock gating -- enable ALL gates */
	amvdec_write_dos(core, HEVCD_IPP_DYNCLKGATE_CONFIG, 0xffffffff);

	/* Initialize deblocking filter.
	 * DBLK_CFGB encodes double-write FIFO thresholds (SM1), pipeline
	 * config (G12A), or legacy pipeline mode (GXL/GXBB).
	 * BIT(0) = VP9 format flag -- NOT set here (HEVC mode).
	 */
	if (core->platform->revision >= VDEC_REVISION_SM1) {
		amvdec_write_dos(core, HEVC_DBLK_CFGB,
				 DBLK_CFGB_SM1_DW_FIFO_THRES);
		amvdec_write_dos(core, HEVC_DBLK_CFG3,
				 DBLK_CFG3_SM1_PIPELINE);
	} else if (core->platform->revision >= VDEC_REVISION_G12A) {
		amvdec_write_dos(core, HEVC_DBLK_CFGB,
				 DBLK_CFGB_G12A_INIT);
	} else {
		amvdec_write_dos(core, HEVC_DBLK_CFGB,
				 DBLK_CFGB_PRE_G12A_INIT);
	}
}

/* Deferred ACTION_DONE: signal firmware to process next NAL after a short
 * delay.  This breaks the tight threaded-ISR loop that otherwise processes
 * all frames at RT priority without yielding to userspace.
 */
static void codec_hevc_action_done_work(struct work_struct *work)
{
	struct codec_hevc *hevc = container_of(work, struct codec_hevc,
					       action_done_work.work);
	struct amvdec_session *sess = hevc->sess;
	struct amvdec_core *core = sess->core;

	amvdec_write_dos(core, HEVC_DEC_STATUS_REG, HEVC_ACTION_DONE);
	amvdec_write_dos(core, HEVC_MCPU_INTR_REQ, AMRISC_MAIN_REQ);
}

static int codec_hevc_start(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc;
	u32 val;
	int i;
	int ret;

	hevc = kzalloc(sizeof(*hevc), GFP_KERNEL);
	if (!hevc)
		return -ENOMEM;

	ret = codec_hevc_alloc_workspace(core, hevc);
	if (ret)
		goto free_hevc;

	/* Pre-allocate MMU map before workspace setup.  On GXL, the MMU
	 * map address is written to HEVC_ASSIST_SCRATCH_7 during workspace
	 * setup, replacing the stream swap buffer.  The firmware reads
	 * this register during init, so a valid DMA address must be
	 * present before the firmware starts running.
	 */
	if (codec_hevc_use_mmu(core->platform->revision, sess->pixfmt_cap,
			       0 /* is_10bit not known yet, assume 8-bit */)) {
		/* Must match MMU_MAP_SIZE in codec_hevc_common.c -- this is
		 * the same buffer.  codec_hevc_alloc_mmu_headers() skips its
		 * own allocation when mmu_map_vaddr is already non-NULL, and
		 * codec_hevc_free_mmu_headers() frees with MMU_MAP_SIZE.
		 * Allocating with a smaller size (0x4800) caused KI-107: free
		 * walked extra pages outside the allocation, calling put_page
		 * on adjacent CMA pages and underflowing their refcounts.
		 */
		hevc->common.mmu_map_vaddr = dma_alloc_coherent(core->dev,
				MMU_MAP_SIZE, &hevc->common.mmu_map_paddr,
				GFP_KERNEL);
		if (!hevc->common.mmu_map_vaddr) {
			ret = -ENOMEM;
			goto free_workspace;
		}
	}

	/* Dispatch to hevc_bsp session start (Bucket 3) for BSP-PROTO
	 * firmware -- power VDEC_1 + alloc RPM + alloc AUX.  Upstream
	 * firmware sessions skip this entirely.  All BSP-protocol
	 * orchestration lives in codec_hevc_bsp.c; this is the only
	 * gate at the session_start level.
	 */
	if (sess->fw_match &&
	    (sess->fw_match->quirks & AMVDEC_FW_QUIRK_HEVC_BSP_PROTO)) {
		ret = codec_hevc_bsp_session_start(sess, &hevc->common);
		if (ret)
			goto free_workspace;
	}

	codec_hevc_setup_workspace(sess, hevc);

	/* Reset IQIT to reinit memory */
	amvdec_write_dos(core, DOS_SW_RESET3, BIT(14));
	amvdec_clear_dos_bits(core, HEVC_CABAC_CONTROL, BIT(0));
	amvdec_clear_dos_bits(core, HEVC_PARSER_CORE_CONTROL, BIT(0));

	val = amvdec_read_dos(core, HEVC_STREAM_CONTROL);
	val |= BIT(0); /* stream_fetch_enable */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		val |= (0xf << 25); /* arwlen_axi_max: 16-beat AXI bursts (G12A+) */
	amvdec_write_dos(core, HEVC_STREAM_CONTROL, val);

	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos_bits(core, HEVC_STREAM_FIFO_CTL, BIT(29));

	/* PARSER_INT_CONTROL: configure parser interrupt sources.
	 * BSP: (3<<29) = parser_int_disable_cnt, (2<<26) = parser_int_src_sel,
	 * BIT(24) = startcode_int_en, BIT(22) = nal_search_int_en,
	 * BIT(15) = clk_forceon, BIT(7) = cmd_fifo_int_en,
	 * BIT(4) = cabac_int_en, BIT(0) = parser_int_enable.
	 */
	val = amvdec_read_dos(core, HEVC_PARSER_INT_CONTROL) & 0x03ffffff;
	val |= (3 << 29) | (2 << 26) | BIT(24) | BIT(22) |
	       BIT(15) | BIT(7) | BIT(4) | BIT(0);
	amvdec_write_dos(core, HEVC_PARSER_INT_CONTROL, val);
	amvdec_write_dos_bits(core, HEVC_SHIFT_STATUS, BIT(1) | BIT(0));
	/* SHIFT_CONTROL: BIT(15)=clk_forceon, (3<<6)=sft_valid_wr_position,
	 * (2<<4)=emulate_code_length_minus1, (2<<1)=start_code_length_minus1,
	 * BIT(0)=stream_shift_enable.
	 */
	amvdec_write_dos(core, HEVC_SHIFT_CONTROL,
			 BIT(15) | (3 << 6) | (2 << 4) | (2 << 1) | BIT(0));
	amvdec_write_dos(core, HEVC_CABAC_CONTROL, BIT(13) | BIT(0));
	/* CABAC: BIT(13)=clk_forceon, BIT(0)=cabac_enable */
	amvdec_write_dos(core, HEVC_PARSER_CORE_CONTROL, BIT(15) | BIT(0));
	/* PARSER_CORE: BIT(15)=clk_forceon, BIT(0)=parser_core_enable */
	amvdec_write_dos(core, HEVC_SHIFT_STARTCODE, 0x00000100);
	amvdec_write_dos(core, HEVC_SHIFT_EMULATECODE, 0x00000300);

	amvdec_write_dos(core, HEVC_DEC_STATUS_REG, 0);

	/* Initialize IQIT_SCALELUT memory */
	amvdec_write_dos(core, HEVC_IQIT_SCALELUT_WR_ADDR, 0);
	for (i = 0; i < 1024; i++)
		amvdec_write_dos(core, HEVC_IQIT_SCALELUT_DATA, 0);

	amvdec_write_dos(core, HEVC_DECODE_SIZE, 0);

	amvdec_write_dos(core, HEVC_PARSER_CMD_WRITE, BIT(16));
	for (i = 0; i < ARRAY_SIZE(vdec_hevc_parser_cmd); ++i)
		amvdec_write_dos(core, HEVC_PARSER_CMD_WRITE,
				 vdec_hevc_parser_cmd[i]);

	amvdec_write_dos(core, HEVC_PARSER_CMD_SKIP_0, PARSER_CMD_SKIP_CFG_0);
	amvdec_write_dos(core, HEVC_PARSER_CMD_SKIP_1, PARSER_CMD_SKIP_CFG_1);
	amvdec_write_dos(core, HEVC_PARSER_CMD_SKIP_2, PARSER_CMD_SKIP_CFG_2);
	amvdec_write_dos(core, HEVC_PARSER_IF_CONTROL,
			 BIT(5) | BIT(2) | BIT(0));

	amvdec_write_dos(core, HEVCD_IPP_TOP_CNTL, BIT(0));
	amvdec_write_dos(core, HEVCD_IPP_TOP_CNTL, BIT(1));

	amvdec_write_dos(core, HEVC_WAIT_FLAG, 1);

	/* Clear and enable mailbox interrupt */
	amvdec_write_dos(core, HEVC_ASSIST_MBOX1_CLR_REG, 1);
	amvdec_write_dos(core, HEVC_ASSIST_MBOX1_MASK, 1);
	/* Disable PSCALE */
	amvdec_write_dos(core, HEVC_PSCALE_CTRL, 0);
	/* Let firmware auto-search NAL units.
	 * 0xc = BIT(3)|BIT(2): auto_search_mode + nal_skip_check_enable (BSP).
	 */
	amvdec_write_dos(core, NAL_SEARCH_CTL, 0xc);

	amvdec_write_dos(core, DECODE_STOP_POS, 0);
	amvdec_write_dos(core, HEVC_DECODE_MODE, DECODE_MODE_SINGLE);

	codec_hevc_init_decoder(core, hevc);

	INIT_LIST_HEAD(&hevc->ref_frames_list);
	mutex_init(&hevc->lock);
	hevc->max_ra = INT_MAX;
	hevc->sess = sess;
	INIT_DELAYED_WORK(&hevc->action_done_work, codec_hevc_action_done_work);
	sess->priv = hevc;

	/* Both firmware versions populate bit_depth from SPS RPM 0x56,
	 * but at different LMEM offsets due to different struct layouts:
	 *   36864-byte firmware: LMEM 0x148 (struct offset 0x48)
	 *   16384-byte firmware: LMEM 0x144 (struct offset 0x44)
	 * The BSP firmware expanded tile_width[] from 8 to 12 entries,
	 * shifting all subsequent fields by 4 u16 slots.  The original
	 * upstream struct (matching 16KB firmware) has bit_depth at 0x44:
	 *   tile_width[8]+tile_height[8] then misc..pps_cr_qp_offset,
	 *   first_slice..vui_time_scale_lo, then bit_depth at 0x44.
	 * The BSP struct has tile_width[12], pushing bit_depth to 0x48.
	 */
	if (sess->fw_size == 36864 || sess->fw_size == 40960)
		hevc->bitdepth_offset = 0x48;  /* LMEM 0x148 -- BSP layout (36KB + 40KB swap) */
	else
		hevc->bitdepth_offset = -1;    /* 16KB upstream firmware does not reliably
						 * populate bit_depth in LMEM.  Reading any
						 * offset gets garbage that toggles is_10bit,
						 * causing infinite amvdec_src_change() loops.
						 * Disable bit_depth detection entirely.
						 */

	dev_info(core->dev, "HEVC: firmware %u bytes, bit_depth at RPM offset %#x\n",
		 sess->fw_size, hevc->bitdepth_offset);

	if (hevc->bitdepth_offset < 0)
		dev_warn(core->dev,
			 "HEVC: unknown firmware size %u. "
			 "bit_depth detection disabled -- 10-bit content "
			 "may decode incorrectly.\n", sess->fw_size);

	/* Do NOT inherit sess->width/height here. Starting with width=0
	 * ensures the first frame with valid RPM dimensions triggers
	 * amvdec_src_change() to set up capture buffers. Inheriting
	 * the session default (1280x720) would skip src_change when the
	 * stream matches, causing decode to stall (no capture buffers).
	 *
	 * For KI-023 (firmware restart mid-stream): P-frames with 0x0
	 * RPM dimensions are dropped by the !hevc->width guard in the
	 * ISR until the next I-frame with valid SPS arrives.
	 */

	dev_dbg(core->dev, "HEVC codec started, workspace=%pad size=%x\n",
		 &hevc->workspace_paddr, SIZE_WORKSPACE);

	return 0;

free_workspace:
	/* Undo hevc_bsp session_start if it was done.  session_stop is
	 * a no-op for non-BSP_PROTO sessions.
	 */
	codec_hevc_bsp_session_stop(sess, &hevc->common);
	if (hevc->common.mmu_map_vaddr) {
		/* Must match the alloc size above (MMU_MAP_SIZE).  KI-107
		 * note above explains why size mismatch is a refcount bug.
		 */
		dma_free_coherent(core->dev, MMU_MAP_SIZE,
				  hevc->common.mmu_map_vaddr,
				  hevc->common.mmu_map_paddr);
	}
	dma_free_coherent(core->dev, SIZE_WORKSPACE,
			  hevc->workspace_vaddr,
			  hevc->workspace_paddr);
free_hevc:
	kfree(hevc);
	return ret;
}

static int codec_hevc_stop(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;

	cancel_delayed_work_sync(&hevc->action_done_work);

	/*
	 * By the time we reach here, __vdec_hevc_stop() has already halted
	 * the firmware AND synchronously powered off the HEVC engine
	 * (DOS_SW_RESET3=0xffffffff toggle inside vdec_hevc_power_off).
	 * The IP block holds no DMA refs on our workspace / MMU buffers,
	 * so dma_free_coherent() below is safe.
	 */

	mutex_lock(&hevc->lock);
	if (hevc->workspace_vaddr) {
		dma_free_coherent(core->dev, SIZE_WORKSPACE,
				  hevc->workspace_vaddr,
				  hevc->workspace_paddr);
		hevc->workspace_vaddr = NULL;
	}

	/* hevc_bsp session teardown: free AUX+RPM, release VDEC_1 power.
	 * No-op for upstream-firmware sessions.
	 */
	codec_hevc_bsp_session_stop(sess, &hevc->common);

	codec_hevc_free_fbc_buffers(sess, &hevc->common);
	codec_hevc_free_mmu_headers(sess, &hevc->common);
	mutex_unlock(&hevc->lock);

	return 0;
}

static void codec_hevc_fetch_rpm(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	u16 *rpm_vaddr = hevc->workspace_vaddr + RPM_OFFSET;
	int i, j;
	int count = RPM_END - RPM_BEGIN;

	for (i = 0; i < count; i += 4)
		for (j = 0; j < 4; j++)
			hevc->rpm_param.l.data[i + j] = rpm_vaddr[i + 3 - j];

	/* Debug: dump first 32 raw RPM words to see width/height */
	pr_debug("RPM raw[000]: %04x %04x %04x %04x %04x %04x %04x %04x "
		"%04x %04x %04x %04x %04x %04x %04x %04x\n",
		rpm_vaddr[0], rpm_vaddr[1], rpm_vaddr[2], rpm_vaddr[3],
		rpm_vaddr[4], rpm_vaddr[5], rpm_vaddr[6], rpm_vaddr[7],
		rpm_vaddr[8], rpm_vaddr[9], rpm_vaddr[10], rpm_vaddr[11],
		rpm_vaddr[12], rpm_vaddr[13], rpm_vaddr[14], rpm_vaddr[15]);
	pr_debug("RPM raw[016]: %04x %04x %04x %04x %04x %04x %04x %04x "
		"%04x %04x %04x %04x %04x %04x %04x %04x\n",
		rpm_vaddr[16], rpm_vaddr[17], rpm_vaddr[18], rpm_vaddr[19],
		rpm_vaddr[20], rpm_vaddr[21], rpm_vaddr[22], rpm_vaddr[23],
		rpm_vaddr[24], rpm_vaddr[25], rpm_vaddr[26], rpm_vaddr[27],
		rpm_vaddr[28], rpm_vaddr[29], rpm_vaddr[30], rpm_vaddr[31]);
	pr_debug("RPM parsed[16..31]: l.data = %04x %04x %04x %04x %04x %04x %04x %04x "
		"%04x %04x %04x %04x %04x %04x %04x %04x\n",
		hevc->rpm_param.l.data[16], hevc->rpm_param.l.data[17],
		hevc->rpm_param.l.data[18], hevc->rpm_param.l.data[19],
		hevc->rpm_param.l.data[20], hevc->rpm_param.l.data[21],
		hevc->rpm_param.l.data[22], hevc->rpm_param.l.data[23],
		hevc->rpm_param.l.data[24], hevc->rpm_param.l.data[25],
		hevc->rpm_param.l.data[26], hevc->rpm_param.l.data[27],
		hevc->rpm_param.l.data[28], hevc->rpm_param.l.data[29],
		hevc->rpm_param.l.data[30], hevc->rpm_param.l.data[31]);
}

static int codec_hevc_process_rpm(struct codec_hevc *hevc)
{
	union hevc_rpm_param *param = &hevc->rpm_param;
	u32 pic_width = param->p.pic_width_in_luma_samples;
	u32 pic_height = param->p.pic_height_in_luma_samples;
	int new_is_10bit;
	int reconfig = 0;

	/* Both firmware versions write bit_depth from SPS RPM, but at
	 * different LMEM offsets.  Read from the raw data array using
	 * the firmware-specific offset rather than the named struct field.
	 */
	if (hevc->bitdepth_offset >= 0) {
		u16 raw_bitdepth = param->l.data[hevc->bitdepth_offset];
		int bit_depth_luma = (raw_bitdepth & 0xf) + 8;
		new_is_10bit = (bit_depth_luma > 8) ? 1 : 0;
	} else {
		new_is_10bit = 0;  /* unknown firmware: assume 8-bit */
	}

	/* Detect bit_depth changes mid-stream (e.g. 8-bit -> 10-bit).
	 * Only possible with BSP firmware.
	 * BSP vh265.c:12376: calls init_decode_head_hw() on reconfig_flag.
	 * A bit_depth change requires FBC body size recalculation
	 * (3200 vs 4096 bytes/block), SAO_CTRL5 BIT(9) update, and
	 * potentially a dw_mode switch.  Trigger amvdec_src_change()
	 * to reallocate buffers with correct parameters.
	 */
	if (hevc->bitdepth_offset >= 0 && new_is_10bit != hevc->is_10bit && hevc->width) {
		hevc->is_10bit = new_is_10bit;
		reconfig = 1;
	} else {
		hevc->is_10bit = new_is_10bit;
	}

	/* P/B-frames don't carry SPS -- firmware reports 0x0 after a
	 * restart caused by V4L2 dynamic resolution change (STREAMOFF/
	 * STREAMON CAPTURE reloads firmware, losing SPS context).
	 * Keep existing dimensions; BSP also skips 0x0 (KI-023).
	 */
	if (!pic_width || !pic_height)
		return reconfig;

	if (pic_width != hevc->width || pic_height != hevc->height) {
		hevc->width = pic_width;
		hevc->height = pic_height;
		return 1;
	}

	return reconfig;
}

/* Compute Picture Order Count from LMEM parameters (BSP vh265.c logic).
 * Must be called after codec_hevc_fetch_rpm() and before setup_decode().
 */
static void codec_hevc_compute_poc(struct codec_hevc *hevc)
{
	union hevc_rpm_param *param = &hevc->rpm_param;
	u16 nal_type = param->p.m_nalUnitType;
	u16 temporal_id = param->p.m_temporalId;

	/* IDR pictures reset POC to 0 (BSP NAL_UNIT_CODED_SLICE_IDR = 19,
	 * NAL_UNIT_CODED_SLICE_IDR_N_LP = 20) */
	if (nal_type == 19 || nal_type == 20) {
		hevc->curr_poc = 0;
		if (temporal_id <= 1)
			hevc->iPrevTid0POC = 0;
	} else {
		int iMaxPOClsb = 1 << (param->p.log2_max_pic_order_cnt_lsb_minus4 + 4);
		int iPOClsb = param->p.POClsb;
		int iPrevPOClsb, iPrevPOCmsb, iPOCmsb;

		if (iMaxPOClsb == 0) {
			hevc->curr_poc = 0;
			return;
		}

		iPrevPOClsb = hevc->iPrevTid0POC % iMaxPOClsb;
		iPrevPOCmsb = hevc->iPrevTid0POC - iPrevPOClsb;

		if ((iPOClsb < iPrevPOClsb) &&
		    ((iPrevPOClsb - iPOClsb) >= (iMaxPOClsb / 2)))
			iPOCmsb = iPrevPOCmsb + iMaxPOClsb;
		else if ((iPOClsb > iPrevPOClsb) &&
			 ((iPOClsb - iPrevPOClsb) > (iMaxPOClsb / 2)))
			iPOCmsb = iPrevPOCmsb - iMaxPOClsb;
		else
			iPOCmsb = iPrevPOCmsb;

		/* BLA pictures reset POCmsb (nal_type 16, 17, 18) */
		if (nal_type >= 16 && nal_type <= 18)
			iPOCmsb = 0;

		hevc->curr_poc = iPOCmsb + iPOClsb;
		if (temporal_id <= 1)
			hevc->iPrevTid0POC = hevc->curr_poc;
	}
}

static struct hevc_frame *
codec_hevc_get_new_frame(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	struct v4l2_m2m_buffer *m2m_buf, *m2m_tmp;
	struct vb2_v4l2_buffer *vbuf = NULL;
	struct hevc_frame *new_frame;

	new_frame = kzalloc(sizeof(*new_frame), GFP_KERNEL);
	if (!new_frame)
		return NULL;

	v4l2_m2m_for_each_dst_buf_safe(sess->m2m_ctx, m2m_buf, m2m_tmp) {
		if (!test_bit(m2m_buf->vb.vb2_buf.index, &hevc->ref_held)) {
			vbuf = &m2m_buf->vb;
			v4l2_m2m_dst_buf_remove_by_buf(sess->m2m_ctx, vbuf);
			break;
		}
	}
	if (!vbuf) {
		dev_warn(sess->core->dev,
			 "HEVC: no free capture buffer (all ref_held)\n");
		kfree(new_frame);
		return NULL;
	}

	new_frame->vbuf = vbuf;
	new_frame->index = vbuf->vb2_buf.index;
	new_frame->show = 1;
	new_frame->width = hevc->width;
	new_frame->height = hevc->height;

	list_add_tail(&new_frame->list, &hevc->ref_frames_list);
	hevc->frames_num++;

	dev_dbg(sess->core->dev,
		"HEVC new_frame: idx=%d Y=0x%llx UV=0x%llx\n",
		new_frame->index,
		(u64)vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 0),
		vbuf->vb2_buf.num_planes > 1 ?
		(u64)vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 1) : 0ULL);

	return new_frame;
}

static void codec_hevc_output_frame(struct amvdec_session *sess,
				    struct hevc_frame *frame)
{
	struct codec_hevc *hevc = sess->priv;

	if (!hevc || !frame->show)
		return;

	if (frame->done) {
		/* Already output -- avoid double v4l2_m2m_buf_done which
		 * triggers a vb2 WARN and can crash.  This can happen when
		 * SLICE_SEGMENT_DONE outputs prev_frame and the following
		 * DECPIC_DATA_DONE outputs cur_frame == prev_frame after
		 * session teardown races.
		 */
		return;
	}

	dev_dbg(sess->core->dev,
		"HEVC output: idx=%d poc=%d Y=0x%llx\n",
		frame->index, frame->poc,
		(u64)vb2_dma_contig_plane_dma_addr(&frame->vbuf->vb2_buf, 0));
	amvdec_dst_buf_done(sess, frame->vbuf, V4L2_FIELD_NONE);
	frame->done = 1;
	hevc->frames_num--;
}

/*
 * Display-order bumping (HEVC Annex C DPB behavior, mirrors BSP
 * vh265.c:output_pic()).  Releases frames whose decode is complete in
 * POC order.  For each iteration, picks the lowest-POC frame with
 * output_ready=1 && done=0 and emits it if we have more such frames
 * buffered than sps_num_reorder_pics_0 (or if flushing).
 *
 * Caller must hold hevc->lock.
 *
 * For IPPP streams (sps_num_reorder_pics_0 == 0) this releases every
 * finished frame immediately, preserving the decode-order behavior.
 */
static void codec_hevc_bump_frames(struct amvdec_session *sess, bool flush)
{
	struct codec_hevc *hevc = sess->priv;
	struct hevc_frame *frame, *best;
	int num_pending;

	for (;;) {
		num_pending = 0;
		best = NULL;
		list_for_each_entry(frame, &hevc->ref_frames_list, list) {
			if (!frame->output_ready || frame->done)
				continue;
			num_pending++;
			if (!best || frame->poc < best->poc)
				best = frame;
		}

		if (!best)
			break;

		/* Bumping: stop when we have just enough buffered to
		 * guarantee the next-to-emit POC is really the minimum.
		 * Force-drain on flush (end of stream / error) and when
		 * the DPB is near-full (mirror BSP vh265.c:4540).
		 */
		if (!flush &&
		    num_pending <= hevc->sps_num_reorder_pics_0 &&
		    num_pending < MAX_REF_PIC_NUM - 1)
			break;

		/* codec_hevc_output_frame() already sets frame->done = 1
		 * and decrements frames_num.
		 */
		codec_hevc_output_frame(sess, best);
	}
}

static void codec_hevc_set_sao(struct amvdec_session *sess,
			       struct vb2_buffer *vb,
			       int new_pic)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;
	dma_addr_t buf_y_paddr;
	dma_addr_t buf_u_v_paddr;
	u32 val;
	int use_fbc = codec_hevc_use_fbc(sess->pixfmt_cap, hevc->is_10bit);
	int use_mmu = codec_hevc_use_mmu(core->platform->revision,
					 sess->pixfmt_cap, hevc->is_10bit);

	/* BSP writes sentinel 0xffffffff to SAO_Y_START_ADDR on new_pic.
	 * This may be required to reset SAO DMA internal state for the
	 * new picture before the actual address is written below.
	 */
	if (new_pic)
		amvdec_write_dos(core, HEVC_SAO_Y_START_ADDR, 0xffffffff);

	/* Determine body buffer address (same logic as VP9) */
	if (codec_hevc_use_downsample(sess->pixfmt_cap, hevc->is_10bit))
		buf_y_paddr = hevc->common.fbc_buffer_paddr[vb->index];
	else
		buf_y_paddr = vb2_dma_contig_plane_dma_addr(vb, 0);

	/* FBC compressed body setup.
	 * On G12A+ even 8-bit decode uses FBC internally (dw_mode=1).
	 * The CM_BODY_START_ADDR must point to the FBC buffer.
	 * On GXL with compressed refs, point to per-frame FBC buffer.
	 */
	if (use_fbc) {
		/* Clear FBC_body_uncouple(17) + mem_saving(9) for FBC setup */
		val = amvdec_read_dos(core, HEVC_SAO_CTRL5) & ~0xff0200;
		amvdec_write_dos(core, HEVC_SAO_CTRL5, val);
		amvdec_write_dos(core, HEVC_CM_BODY_START_ADDR, buf_y_paddr);
	} else if (use_mmu) {
		/* G12A+ MMU mode for 8-bit: FBC body goes to allocated buffer */
		amvdec_write_dos(core, HEVC_CM_BODY_START_ADDR,
				 hevc->common.fbc_buffer_paddr[vb->index]);
	} else if (hevc_needs_compress_ref(core, hevc->is_10bit)) {
		/* GXL compressed reference mode: body buffer per frame */
		amvdec_write_dos(core, HEVC_CM_BODY_START_ADDR,
				 hevc->common.fbc_buffer_paddr[vb->index]);
	}

	/* NV12 output addresses.
	 * BSP with dw_mode=0x10 (double-write only, 8-bit NV12):
	 *   SAO_Y_START_ADDR = 0xffffffff sentinel (kept, NOT overwritten)
	 *   SAO_C_START_ADDR = dw_u_v_adr
	 *   SAO_Y_WPTR = dw_y_adr
	 *   SAO_C_WPTR = dw_u_v_adr
	 * The sentinel tells SAO hardware to use WPTR as the write target.
	 */
	if (codec_hevc_is_nv12(sess->pixfmt_cap)) {
		buf_y_paddr = vb2_dma_contig_plane_dma_addr(vb, 0);
		if (vb->num_planes > 1)
			buf_u_v_paddr =
				vb2_dma_contig_plane_dma_addr(vb, 1);
		else
			buf_u_v_paddr =
				buf_y_paddr + amvdec_get_output_size(sess);
		/* BSP: SAO_Y_START = 0xffffffff sentinel for compress+dw modes,
		 * but for dw_mode=0x10 (direct NV12 only), the START address
		 * stays as sentinel. For non-compress mode (#else path in BSP),
		 * START = mc_y_adr. Use sentinel for MMU, direct addr otherwise.
		 */
		amvdec_write_dos(core, HEVC_SAO_Y_START_ADDR,
				 buf_y_paddr);
		amvdec_write_dos(core, HEVC_SAO_C_START_ADDR, buf_u_v_paddr);
		amvdec_write_dos(core, HEVC_SAO_Y_WPTR, buf_y_paddr);
		amvdec_write_dos(core, HEVC_SAO_C_WPTR, buf_u_v_paddr);
		dev_dbg(core->dev,
			"SAO addrs: Y=0x%08llx UV=0x%08llx outsize=0x%x 10bit=%d\n",
			(u64)buf_y_paddr, (u64)buf_u_v_paddr,
			amvdec_get_output_size(sess), hevc->is_10bit);
		dev_dbg(core->dev,
			"SAO CTRL5=0x%x Y_LEN=0x%x C_LEN=0x%x CTRL1=0x%x\n",
			 amvdec_read_dos(core, HEVC_SAO_CTRL5),
			 amvdec_read_dos(core, HEVC_SAO_Y_LENGTH),
			 amvdec_read_dos(core, HEVC_SAO_C_LENGTH),
			 amvdec_read_dos(core, HEVC_SAO_CTRL1));
	}

	/* MMU header address (for address translation via ANC2AXI) */
	if (use_mmu) {
		amvdec_write_dos(core, HEVC_CM_HEADER_START_ADDR,
				 hevc->common.mmu_header_paddr[vb->index]);
		/* BIT(10): use HEVC_CM_HEADER_START_ADDR for MMU.
		 * BIT(9): mem_saving_mode -- 8-bit FBC uses 3200 bytes per
		 * 64x32 block; 10-bit uses full 4096.  BSP only sets BIT(9)
		 * when mem_saving_mode=1 (8-bit).  Setting it for 10-bit
		 * causes body format mismatch: SAO reads compressed refs
		 * as 3200-byte blocks but they were written as 4096-byte,
		 * corrupting the chroma plane (KI-060).
		 */
		amvdec_write_dos_bits(core, HEVC_SAO_CTRL5,
				      SAO_CTRL5_USE_CM_HDR);
		if (!hevc->is_10bit)
			amvdec_write_dos_bits(core, HEVC_SAO_CTRL5,
					      SAO_CTRL5_MEM_SAVING);
		else
			amvdec_clear_dos_bits(core, HEVC_SAO_CTRL5,
					      SAO_CTRL5_MEM_SAVING);
	}

	amvdec_write_dos(core, HEVC_SAO_Y_LENGTH,
			 amvdec_get_output_size(sess));
	amvdec_write_dos(core, HEVC_SAO_C_LENGTH,
			 amvdec_get_output_size(sess) / 2);

	/* DBLK_CFGB (G12A+): output mode control.
	 * BSP dw_mode=1 (FBC+NV12): BIT(8) (compress write) | BIT(9) (double write).
	 * Both are needed on G12A+ even for 8-bit -- the hardware always does
	 * compressed decode internally and double-writes to NV12.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A) {
		amvdec_clear_dos_bits(core, HEVC_DBLK_CFGB,
				      DBLK_CFGB_PIPELINE_MODE | BIT(5) |
				      DBLK_CFGB_COMPRESS_WR |
				      DBLK_CFGB_DOUBLE_WR);
		/* compress write + double write (dw_mode=1) */
		amvdec_write_dos_bits(core, HEVC_DBLK_CFGB,
				      DBLK_CFGB_COMPRESS_WR |
				      DBLK_CFGB_DOUBLE_WR);
		/* dblk pipeline mode for large frames */
		if (sess->width >= 1280)
			amvdec_write_dos_bits(core, HEVC_DBLK_CFGB,
					      DBLK_CFGB_PIPELINE_MODE);

		/* KI-001 Strike 9: SAO_CTRL5 BIT(7) Cb chroma rounding workaround
		 * was historically set unconditionally for G12+.  Comment
		 * claimed "Y and Cr bit-perfect without it; Cb has ~56 dB
		 * error".  But 2026-04-26 retest shows the OPPOSITE pattern
		 * is true on G12B+SM1 with current mainline: V (Cr) is the
		 * worst (~13 dB), Y is bad (~18 dB), U (Cb) is at silicon
		 * floor (~54 dB).  VP9 (works bit-exact on G12+) does NOT
		 * set this bit.  Hypothesis: the workaround was originally
		 * tested against a different test source / firmware era and
		 * actually REGRESSES the V plane on current upstream firmware.
		 * Strike 9: skip the bit on G12+ to test.
		 */
	}

	/* SAO_CTRL1: endianness and mode control.
	 * BSP dw_mode=1: bits[1:0]=0 (both compress and double-write enabled).
	 * BSP dw_mode=0x10: bit[0]=1 (cm_disable, double-write only).
	 *
	 * GXL HEVC 8-bit uses dw_mode=1 (compressed refs + SAO double-write).
	 * Both compress and DW must be enabled (bits[1:0]=0).
	 */
	val = amvdec_read_dos(core, HEVC_SAO_CTRL1) & ~0xffff;
	val |= SAO_CTRL1_CLK_FORCEON;
	val |= SAO_CTRL1_AXI_ALIGN_64;
	val |= SAO_CTRL1_ENDIAN_MASK; /* Little-endian per 2-byte swap */
	/* bits[1:0] = 0 for compressed + double-write mode (dw_mode=1) */
	amvdec_write_dos(core, HEVC_SAO_CTRL1, val);

	/* No downscale for full-res NV12 */
	val = amvdec_read_dos(core, HEVC_SAO_CTRL5) & ~SAO_CTRL5_DW_SCALE_MASK;
	amvdec_write_dos(core, HEVC_SAO_CTRL5, val);

	val = amvdec_read_dos(core, HEVCD_IPP_AXIIF_CONFIG);
	val &= ~(IPP_AXIIF_ENDIAN_MASK | (3 << 8) | IPP_AXIIF_UV_SWAP);
	val |= IPP_AXIIF_ENDIAN_MASK; /* Little-endian per 2-byte swap */
	val |= IPP_AXIIF_ALIGN_64;    /* Linear_LineAlignment = 64 bytes */
	if (codec_hevc_is_nv21(sess->pixfmt_cap))
		val |= IPP_AXIIF_UV_SWAP;
	amvdec_write_dos(core, HEVCD_IPP_AXIIF_CONFIG, val);

	dev_dbg(core->dev,
		"SAO set: idx=%d Y_LEN=0x%x C_LEN=0x%x Y_START=0x%x C_START=0x%x CTRL1=0x%x CTRL5=0x%x BODY=0x%x HDR=0x%x\n",
		vb->index,
		amvdec_read_dos(core, HEVC_SAO_Y_LENGTH),
		amvdec_read_dos(core, HEVC_SAO_C_LENGTH),
		amvdec_read_dos(core, HEVC_SAO_Y_START_ADDR),
		amvdec_read_dos(core, HEVC_SAO_C_START_ADDR),
		amvdec_read_dos(core, HEVC_SAO_CTRL1),
		amvdec_read_dos(core, HEVC_SAO_CTRL5),
		amvdec_read_dos(core, HEVC_CM_BODY_START_ADDR),
		amvdec_read_dos(core, HEVC_CM_HEADER_START_ADDR));
}

/* Configure MPRED (Motion Prediction) hardware for HEVC decode.
 * This is critical for inter-frame prediction (P/B frames).
 * Based on BSP config_mpred_hw() in vh265.c.
 *
 * MPRED CTRL0 register layout (from BSP):
 *   [1:0]   slice_type (0=B, 1=P, 2=I)
 *   [2]     new_pic (first slice of new picture)
 *   [3]     new_tile
 *   [4]     isNextSliceSegment
 *   [5]     TMVPFlag (temporal MV prediction enable)
 *   [6]     LDCFlag (low-delay check)
 *   [7]     ColFromL0Flag
 *   [8]     above_ptr_ctrl
 *   [9]     above_en
 *   [10]    mv_wr_en (MV write enable -- CRITICAL)
 *   [11]    mv_rd_en (MV read enable -- CRITICAL)
 *   [12]    col_isIntra (collocated frame is intra)
 *   [13]    buffer_linear (=1)
 *   [14]    LongTerm_Curr
 *   [15]    LongTerm_Col
 *   [19:16] lcu_size_log2
 *   [23:20] cu_size_log2 (=3, 8x8 min CU)
 *   [26:24] plevel (log2_parallel_merge_level)
 */
static void codec_hevc_set_mpred(struct amvdec_core *core,
				 struct codec_hevc *hevc)
{
	union hevc_rpm_param *param = &hevc->rpm_param;
	u32 log2_min_cb = param->p.log2_min_coding_block_size_minus3 + 3;
	u32 log2_ctb = log2_min_cb + param->p.log2_diff_max_min_coding_block_size;
	u32 lcu_size_log2 = log2_ctb;
	u32 lcu_size = 1 << log2_ctb;
	u32 lcu_width = (hevc->width + lcu_size - 1) / lcu_size;
	u32 lcu_height = (hevc->height + lcu_size - 1) / lcu_size;
	u32 slice_type = param->p.slice_type;
	u32 tmvp_flag = param->p.slice_temporal_mvp_enable_flag;
	u32 col_from_l0 = param->p.collocated_from_l0_flag;
	u32 plevel = param->p.log2_parallel_merge_level;
	u32 max_merge_cand = 5 - param->p.five_minus_max_num_merge_cand;
	u32 mv_wr_start, mv_rd_start, mv_rd_end;
	u32 above_en, mv_wr_en, mv_rd_en, col_isIntra;
	u32 data32;
	int col_poc_val = 0;   /* collocated reference POC */
	int col_buf_idx_snap = -1;  /* snapshot for mpred_trace debugfs */
	bool has_col_ref = false;  /* true if valid collocated ref found */
	int MV_MEM_UNIT;
	int i;

	/* Clamp merge candidates (BSP sanity check) */
	if (max_merge_cand > 5 || max_merge_cand < 1)
		max_merge_cand = 5;

	dev_dbg(core->dev,
		"MPRED enter: slice=%u lcu=%u tmvp=%u merge=%u prev=%p\n",
		slice_type, lcu_size, tmvp_flag, max_merge_cand,
		hevc->prev_frame);

	/* MV write/read enable based on slice type (BSP logic) */
	if (slice_type != 2) {
		/* P or B slice */
		above_en = 1;
		mv_wr_en = 1;
		mv_rd_en = 1;
		col_isIntra = 0;
	} else {
		/* I slice -- still write MVs but don't read */
		above_en = 1;
		mv_wr_en = 1;
		mv_rd_en = 0;
		col_isIntra = 0;
	}

	/* MV buffer addresses -- per-frame MV buffer (BSP: pic->mpred_mv_wr_start_addr) */
	mv_wr_start = hevc->workspace_paddr + MPRED_MV_OFFSET +
		      (hevc->cur_frame->index * MPRED_MV_BUF_SIZE);

	/* Collocated reference: BSP uses collocated_ref_idx to select from
	 * L0 or L1 based on collocated_from_l0_flag and slice_type.
	 * For B-slices: col_from_l0=1 -> use L0, col_from_l0=0 -> use L1
	 * For P-slices: always use L0
	 */
	{
		int col_ref = param->p.collocated_ref_idx;
		int col_buf_idx = -1;
		bool col_found = false;  /* did we resolve col_ref to a POC? */

		if (slice_type == 0) {
			/* B-slice: select L0 or L1 based on col_from_l0 */
			if (col_from_l0) {
				if (col_ref < hevc->ref_num_l0) {
					col_poc_val = hevc->ref_poc_list0[col_ref];
					col_found = true;
				}
			} else {
				if (col_ref < hevc->ref_num_l1) {
					col_poc_val = hevc->ref_poc_list1[col_ref];
					col_found = true;
				}
			}
		} else if (slice_type == 1) {
			/* P-slice: always from L0 */
			if (col_ref < hevc->ref_num_l0) {
				col_poc_val = hevc->ref_poc_list0[col_ref];
				col_found = true;
			}
		}

		if (col_found)
			col_buf_idx = codec_hevc_find_buf_index(hevc, col_poc_val);
		col_buf_idx_snap = col_buf_idx;

		if (col_buf_idx >= 0) {
			mv_rd_start = hevc->workspace_paddr + MPRED_MV_OFFSET +
				      (col_buf_idx * MPRED_MV_BUF_SIZE);
			has_col_ref = true;
		} else if (hevc->prev_frame) {
			/* Fallback to prev_frame if collocated ref not found */
			mv_rd_start = hevc->workspace_paddr + MPRED_MV_OFFSET +
				      (hevc->prev_frame->index * MPRED_MV_BUF_SIZE);
			col_poc_val = hevc->prev_frame->poc;
			has_col_ref = true;
		} else {
			mv_rd_start = mv_wr_start;
			col_isIntra = 1;
		}

		/* Check if collocated frame has same dimensions */
		if (has_col_ref && col_buf_idx >= 0) {
			struct hevc_frame *col_frame;
			bool size_mismatch = false;

			list_for_each_entry(col_frame,
					    &hevc->ref_frames_list, list) {
				if (col_frame->index == col_buf_idx) {
					if (col_frame->width !=
					    hevc->cur_frame->width ||
					    col_frame->height !=
					    hevc->cur_frame->height)
						size_mismatch = true;
					break;
				}
			}
			if (size_mismatch) {
				col_isIntra = 1;
				mv_rd_en = 0;
			}
		}
	}

	/* MV_MEM_UNIT: bytes per LCU for motion vector storage (BSP).
	 * Scales with LCU area: 64x64=0x200(512), 32x32=0x80(128),
	 * 16x16=0x20(32) -- 2 bytes per 4x4 sub-block for MV+refIdx.
	 */
	MV_MEM_UNIT = lcu_size_log2 == 6 ? 0x200 :
		      lcu_size_log2 == 5 ? 0x80 : 0x20;

	mv_rd_end = mv_rd_start + (lcu_width * lcu_height * MV_MEM_UNIT);

	/* CTRL3: MV precision/search range constant (BSP) */
	amvdec_write_dos(core, HEVC_MPRED_CTRL3, MPRED_CTRL3_DEFAULT);

	/* CTRL0: master control register -- THIS WAS MISSING */
	data32 = (slice_type & 3) |
		 ((hevc->new_pic & 1) << 2) | /* new_pic from slice header */
		 (0 << 3) |		/* new_tile = 0 (no tiles) */
		 (0 << 4) |		/* isNextSliceSegment = 0 */
		 ((tmvp_flag & 1) << 5) |
		 (0 << 6) |		/* LDCFlag = 0 (conservative) */
		 ((col_from_l0 & 1) << 7) |
		 (0 << 8) |		/* above_ptr_ctrl = 0 */
		 ((above_en & 1) << 9) |
		 ((mv_wr_en & 1) << 10) |
		 ((mv_rd_en & 1) << 11) |
		 ((col_isIntra & 1) << 12) |
		 (1 << 13) |		/* buffer_linear = 1 */
		 (0 << 14) |		/* LongTerm_Curr = 0 */
		 (0 << 15) |		/* LongTerm_Col = 0 */
		 ((lcu_size_log2 & 0xf) << 16) |
		 (3 << 20) |		/* cu_size_log2 = 3 (8x8) */
		 ((plevel & 0x7) << 24);
	data32 &= ~BIT(28);
	amvdec_write_dos(core, HEVC_MPRED_CTRL0, data32);

	/* Emit per-slice snapshot to debugfs mpred_trace (read via
	 * /sys/kernel/debug/meson-vdec/mpred_trace).  Informational only --
	 * does NOT affect decode.  Used by TODO-HEVC-GXL-BFRAME-CONTENT
	 * and TASK-004 investigations.
	 */
	{
		struct vdec_dbg_mpred_entry e = {
			.timestamp_ns = ktime_get_ns(),
			.codec = 0, /* hevc */
			.slice_type = slice_type,
			.new_pic = hevc->new_pic ? 1 : 0,
			.col_from_l0 = col_from_l0 ? 1 : 0,
			.curr_poc = hevc->curr_poc,
			.ref_num_l0 = min_t(u32, hevc->ref_num_l0,
					    VDEC_DBG_MPRED_MAX_REFS),
			.ref_num_l1 = min_t(u32, hevc->ref_num_l1,
					    VDEC_DBG_MPRED_MAX_REFS),
			.tmvp_flag = tmvp_flag ? 1 : 0,
			.num_neg = min_t(int, hevc->num_neg_snap,
					 VDEC_DBG_MPRED_MAX_REFS),
			.num_pos = min_t(int, hevc->num_pos_snap,
					 VDEC_DBG_MPRED_MAX_REFS),
			.mod_flag = param->p.modification_flag & 0xff,
			.num_ref_idx_l0_active = param->p.num_ref_idx_l0_active,
			.num_ref_idx_l1_active = param->p.num_ref_idx_l1_active,
			.col_poc = has_col_ref ? col_poc_val : -1,
			.col_buf_idx = col_buf_idx_snap,
			.collocated_ref_idx = param->p.collocated_ref_idx,
			.ctrl0 = data32,
			.mv_wr_start = mv_wr_start,
			.mv_rd_start = mv_rd_start,
			.mv_rd_end = mv_rd_end,
		};
		u8 n;

		/* Compute LDCFlag informationally (BSP vh265.c:8042-8058).
		 * Not written to HW -- experimentally falsified 2026-04-19.
		 */
		e.ldc_flag = 0;
		if (slice_type != 2) {
			e.ldc_flag = 1;
			for (n = 0; n < e.ref_num_l0 && e.ldc_flag; n++) {
				if (hevc->ref_poc_list0[n] > hevc->curr_poc)
					e.ldc_flag = 0;
			}
			if (slice_type == 0) {
				for (n = 0; n < e.ref_num_l1 && e.ldc_flag; n++) {
					if (hevc->ref_poc_list1[n] > hevc->curr_poc)
						e.ldc_flag = 0;
				}
			}
		}

		for (n = 0; n < e.ref_num_l0; n++)
			e.ref_poc_list0[n] = hevc->ref_poc_list0[n];
		for (n = 0; n < e.ref_num_l1; n++)
			e.ref_poc_list1[n] = hevc->ref_poc_list1[n];
		for (n = 0; n < e.num_neg; n++)
			e.st_curr0[n] = hevc->st_curr0_snap[n];
		for (n = 0; n < e.num_pos; n++)
			e.st_curr1[n] = hevc->st_curr1_snap[n];
		for (n = 0; n < VDEC_DBG_MPRED_MAX_REFS; n++)
			e.cur_rps[n] = param->p.CUR_RPS[n];

		vdec_dbg_log_mpred(core, &e);
	}

	/* CTRL1: merge candidates and chroma mode (BSP) */
	data32 = max_merge_cand |
		 MPRED_CTRL1_AMVP_MAX |
		 MPRED_CTRL1_AMVP_MAX_MEM |
		 MPRED_CTRL1_NUM_CHROMA |
		 MPRED_CTRL1_DM_CHROMA_IDX |
		 MPRED_CTRL1_CLK_FORCEON;
	amvdec_write_dos(core, HEVC_MPRED_CTRL1, data32);

	/* PIC_SIZE and PIC_SIZE_LCU (BSP) */
	amvdec_write_dos(core, HEVC_MPRED_PIC_SIZE,
			 hevc->width | (hevc->height << 16));
	amvdec_write_dos(core, HEVC_MPRED_PIC_SIZE_LCU,
			 (lcu_width - 1) | ((lcu_height - 1) << 16));

	/* TILE_START and TILE_SIZE_LCU (no tiles = full picture) */
	amvdec_write_dos(core, HEVC_MPRED_TILE_START, 0);
	amvdec_write_dos(core, HEVC_MPRED_TILE_SIZE_LCU,
			 lcu_width | (lcu_height << 16));

	/* REF_NUM: L0 and L1 reference counts (BSP) */
	if (slice_type == 2) {
		/* I-slice: no references */
		amvdec_write_dos(core, HEVC_MPRED_REF_NUM, 0);
		amvdec_write_dos(core, HEVC_MPRED_REF_EN_L0, 0);
		amvdec_write_dos(core, HEVC_MPRED_REF_EN_L1, 0);
	} else if (slice_type == 1) {
		/* P-slice: 1 L0 reference */
		amvdec_write_dos(core, HEVC_MPRED_REF_NUM,
				 param->p.num_ref_idx_l0_active);
		data32 = 0;
		for (i = 0; i < param->p.num_ref_idx_l0_active && i < 16; i++)
			data32 |= BIT(i);
		amvdec_write_dos(core, HEVC_MPRED_REF_EN_L0, data32);
		amvdec_write_dos(core, HEVC_MPRED_REF_EN_L1, 0);
	} else {
		/* B-slice: L0 + L1 references */
		amvdec_write_dos(core, HEVC_MPRED_REF_NUM,
				 param->p.num_ref_idx_l0_active |
				 (param->p.num_ref_idx_l1_active << 8));
		data32 = 0;
		for (i = 0; i < param->p.num_ref_idx_l0_active && i < 16; i++)
			data32 |= BIT(i);
		amvdec_write_dos(core, HEVC_MPRED_REF_EN_L0, data32);
		data32 = 0;
		for (i = 0; i < param->p.num_ref_idx_l1_active && i < 16; i++)
			data32 |= BIT(i);
		amvdec_write_dos(core, HEVC_MPRED_REF_EN_L1, data32);
	}

	/* CUR_POC and COL_POC (BSP) */
	amvdec_write_dos(core, HEVC_MPRED_CUR_POC, hevc->curr_poc);
	if (has_col_ref && !col_isIntra)
		amvdec_write_dos(core, HEVC_MPRED_COL_POC, col_poc_val);

	/* Reference POC lists (BSP config_mpred_hw at vh265.c:5792).
	 * BSP comment: "must follow Ref_POC_xx_L0 -> Ref_POC_xx_L1 in pair
	 * write order!!!"  Write L0[i] then L1[i] for each index.
	 */
	for (i = 0; i < 16; i++) {
		int l0_poc = (i < hevc->ref_num_l0) ?
			     hevc->ref_poc_list0[i] : 0;
		int l1_poc = (i < hevc->ref_num_l1) ?
			     hevc->ref_poc_list1[i] : 0;

		amvdec_write_dos(core, HEVC_MPRED_L0_REF00_POC + i * 4,
				 l0_poc);
		amvdec_write_dos(core, HEVC_MPRED_L1_REF00_POC + i * 4,
				 l1_poc);
	}

	/* COLREF_EN_L0/L1: BSP parity -- BSP vh265.c never writes these
	 * registers anywhere.  The "comment-cited BSP" behavior was wrong.
	 * Previous upstream wrote bit 0 unconditionally for the col-ref
	 * case; this could have been gating or enabling collocated-MV
	 * prediction incorrectly.  Strike 4 (2026-04-19): match BSP by
	 * not writing these at all.  If the HW default doesn't work for
	 * our decode path, a follow-up can revisit with a trace-backed
	 * value (currently unknown what the BSP-supported hardware
	 * expects here).
	 */

	/* MV buffer addresses (BSP) */
	amvdec_write_dos(core, HEVC_MPRED_MV_WR_START_ADDR, mv_wr_start);
	amvdec_write_dos(core, HEVC_MPRED_MV_RD_START_ADDR, mv_rd_start);
	amvdec_write_dos(core, HEVC_MPRED_MV_RD_END_ADDR, mv_rd_end);

	/* Row jump for tiled MV access (no tiles = 0) */
	amvdec_write_dos(core, HEVC_MPRED_MV_WR_ROW_JUMP, 0);
	amvdec_write_dos(core, HEVC_MPRED_MV_RD_ROW_JUMP, 0);

	/* ABV (above) buffer and MV write/read pointers:
	 * Only reset on new picture (BSP: config_mpred_hw new_pic branch).
	 * For continuation slices, pointers continue from where the
	 * previous slice left off.
	 */
	if (hevc->new_pic) {
		amvdec_write_dos(core, HEVC_MPRED_ABV_START_ADDR,
				 hevc->workspace_paddr + MPRED_ABV_OFFSET);
		amvdec_write_dos(core, HEVC_MPRED_MV_WPTR, mv_wr_start);
		amvdec_write_dos(core, HEVC_MPRED_MV_RPTR, mv_rd_start);
	}

	/* CTRL4 BIT(6): enable MPRED use for inter prediction.
	 * Must be set AFTER CTRL0 configures the MPRED engine.
	 */
	amvdec_clear_dos_bits(core, HEVC_MPRED_CTRL4, BIT(6));
	if (slice_type != 2 && has_col_ref && !col_isIntra)
		amvdec_write_dos_bits(core, HEVC_MPRED_CTRL4, BIT(6));

	dev_dbg(core->dev,
		"MPRED: type=%u POC=%d col_POC=%d CTRL0=0x%x mv_wr=0x%x mv_rd=0x%x CTRL4=0x%x\n",
		slice_type, hevc->curr_poc, col_poc_val,
		amvdec_read_dos(core, HEVC_MPRED_CTRL0),
		mv_wr_start, mv_rd_start,
		amvdec_read_dos(core, HEVC_MPRED_CTRL4));
}

static void codec_hevc_set_mcrcc(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	u32 val;

	/* Reset mcrcc */
	amvdec_write_dos(core, HEVCD_MCRCC_CTL1, MCRCC_CTL1_RESET);

	amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, BIT(1));
	val = amvdec_read_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
	val |= (val << 16);
	amvdec_write_dos(core, HEVCD_MCRCC_CTL2, val);
	val = amvdec_read_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR) & 0xffff;
	val |= (val << 16);
	amvdec_write_dos(core, HEVCD_MCRCC_CTL3, val);

	/* Enable mcrcc progressive-mode */
	amvdec_write_dos(core, HEVCD_MCRCC_CTL1, MCRCC_CTL1_ENABLE);
}

/* Allocate compressed reference buffers for GXL HEVC 8-bit.
 * The HEVC firmware on GXL uses FBC internally even for 8-bit content.
 * These buffers hold compressed reference frame data (body+header).
 * SAO double-writes the decoded NV12 output to V4L2 capture buffers.
 */
static int codec_hevc_alloc_compress_bufs(struct amvdec_session *sess,
					  struct codec_hevc_common *comm)
{
	struct device *dev = sess->core->dev;
	struct v4l2_m2m_buffer *buf;
	u32 am21_size = amvdec_am21c_size(sess->width, sess->height,
					  comm->is_10bit);

	if (comm->fbc_buffer_vaddr[0])
		return 0;

	v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
		u32 idx = buf->vb.vb2_buf.index;
		dma_addr_t paddr;
		void *vaddr = dma_alloc_coherent(dev, am21_size, &paddr,
						 GFP_KERNEL);
		if (!vaddr) {
			codec_hevc_free_fbc_buffers(sess, comm);
			return -ENOMEM;
		}

		comm->fbc_buffer_vaddr[idx] = vaddr;
		comm->fbc_buffer_paddr[idx] = paddr;
	}

	return 0;
}

/* Set up ANC2AXI with compressed body addresses for GXL HEVC 8-bit.
 * In compressed reference mode (BSP dw_mode=1), the ANC2AXI table
 * has ONE entry per buffer (body address only), unlike NV12 mode
 * which has TWO entries (Y+UV).
 */
static void codec_hevc_setup_compress_anc2axi(struct amvdec_session *sess,
					       struct codec_hevc_common *comm)
{
	struct amvdec_core *core = sess->core;
	struct vb2_queue *q = v4l2_m2m_get_dst_vq(sess->m2m_ctx);
	int i;

	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR,
			 ANC2AXI_CONF_AUTO_INC);

	for (i = 0; i < vb2_get_num_buffers(q); ++i) {
		if (!vb2_get_buffer(q, i))
			continue;

		/* Write FBC body address (compressed reference frame) */
		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
				 comm->fbc_buffer_paddr[i] >> 5);
	}

	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, 1);
	amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR,
			 (0 << 8) | (0 << 1) | 1);
	for (i = 0; i < 32; ++i)
		amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, 0);
}

/* Set up ANC2AXI with direct Y/UV buffer addresses for 8-bit NV12.
 * This bypasses the MMU path used for FBC/10-bit decode.
 */
static void codec_hevc_setup_buffers_nv12(struct amvdec_session *sess,
					  struct codec_hevc_common *comm)
{
	struct amvdec_core *core = sess->core;
	struct v4l2_m2m_buffer *buf;
	int i;

	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR,
			 ANC2AXI_CONF_AUTO_INC);

	v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
		struct vb2_buffer *vb = &buf->vb.vb2_buf;
		dma_addr_t buf_y_paddr;
		dma_addr_t buf_uv_paddr;

		buf_y_paddr = vb2_dma_contig_plane_dma_addr(vb, 0);
		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
				 buf_y_paddr >> 5);

		if (vb->num_planes > 1)
			buf_uv_paddr =
				vb2_dma_contig_plane_dma_addr(vb, 1);
		else
			buf_uv_paddr =
				buf_y_paddr + amvdec_get_output_size(sess);
		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
				 buf_uv_paddr >> 5);
	}

	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, 1);
	amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1);
	for (i = 0; i < 32; ++i)
		amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, 0);
}

static void codec_hevc_setup_decode(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;
	union hevc_rpm_param *param = &hevc->rpm_param;
	/* Compute actual CTB size from bitstream SPS parameters */
	u32 log2_min_cb = param->p.log2_min_coding_block_size_minus3 + 3;
	u32 log2_ctb = log2_min_cb + param->p.log2_diff_max_min_coding_block_size;
	u32 lcu_size = 1 << log2_ctb;
	u32 lcu_size_log2 = log2_ctb;
	u32 lcu_width = (hevc->width + lcu_size - 1) / lcu_size;
	u32 lcu_height = (hevc->height + lcu_size - 1) / lcu_size;
	u32 misc_flag0 = param->p.misc_flag0;
	u32 slice_deblocking_filter_disabled;
	u32 val, cfg9, cfg1, data32_2;

	hevc->lcu_total = lcu_width * lcu_height;

	/* HEVC decode pipeline configuration:
	 *
	 * G12A+: MMU mode (DECOMP_CTL1=BIT(4)), compressed decode with
	 * SAO double-write to NV12 output.
	 *
	 * GXL 8-bit: The HEVC firmware always uses compressed references
	 * internally. Unlike H.264/MPEG which support 2-plane NV12 refs
	 * via DECOMP_CTL1=BIT(31), the HEVC engine requires FBC even for
	 * 8-bit content. BSP uses dw_mode=1: DECOMP_CTL1=0 (compressed
	 * refs), DECOMP_CTL2=body_size/32, with SAO double-write to NV12.
	 *
	 * The common setup_decode_head sets BIT(31) for non-MMU non-FBC
	 * 8-bit, which is correct for VDEC_1 codecs but wrong for HEVC.
	 * Override it here with compressed mode for pre-G12A HEVC.
	 */
	codec_hevc_setup_decode_head(sess, hevc->is_10bit);

	/* Buffer setup and MMU map fill only needed for first slice of a
	 * new picture.  Continuation slices decode into the same frame.
	 */
	if (hevc->new_pic) {
		/* The HEVC engine always uses FBC compressed references
		 * internally, even for 8-bit content. BSP always sets
		 * mmu_enable=1 for HEVC on all platforms (vh265.c:16456).
		 * Allocate per-frame FBC body buffers unconditionally so
		 * that codec_hevc_fill_mmu_map() and CM_BODY_START_ADDR
		 * point to valid per-frame memory.
		 *
		 * Without this, fbc_buffer_paddr[] is zero and all frames'
		 * MMU page tables map to physical page 0, causing every
		 * frame's compressed reference body to alias to the same
		 * memory. P-frames read partially overwritten reference
		 * data, producing accumulating corruption (~23 dB PSNR
		 * instead of 44+ dB). KI-081 root cause.
		 */
		codec_hevc_alloc_compress_bufs(sess, &hevc->common);

		if (hevc_needs_compress_ref(core, hevc->is_10bit)) {
			u32 width_64 = ALIGN(sess->width, 64) / 64;
			u32 height_32 = ALIGN(sess->height, 32) / 32;
			u32 body_size = (hevc->is_10bit ? 4096 : 3200) *
					width_64 * height_32;
			amvdec_write_dos(core, HEVCD_MPP_DECOMP_CTL1, 0);
			amvdec_write_dos(core, HEVCD_MPP_DECOMP_CTL2,
					 body_size / 32);
			amvdec_write_dos(core, HEVC_CM_BODY_LENGTH,
					 body_size);
			amvdec_write_dos(core, HEVC_CM_HEADER_OFFSET,
					 body_size);
			amvdec_write_dos(core, HEVC_CM_HEADER_LENGTH,
					 amvdec_am21c_head_size(
						 sess->width,
						 sess->height));
			codec_hevc_setup_compress_anc2axi(sess,
							  &hevc->common);
		} else {
			codec_hevc_setup_buffers(sess, &hevc->common,
						 hevc->is_10bit);
		}

		if (codec_hevc_use_mmu(core->platform->revision,
				       sess->pixfmt_cap, hevc->is_10bit)) {
			codec_hevc_write_mmu_map_addr(core,
						hevc->common.mmu_map_paddr);
			codec_hevc_fill_mmu_map(sess, &hevc->common,
					&hevc->cur_frame->vbuf->vb2_buf);
		}
	}

	/* config_mc_buffer: write per-reference buffer indices to the MC
	 * canvas table.  The hardware uses this to resolve L0[i]/L1[i]
	 * reference indices to ANC2AXI entries (physical buffer addresses).
	 *
	 * BSP: config_mc_buffer() at vh265.c:4619 -- writes all L0 and L1
	 * entries from m_aiRefPOCList0/1, looked up via get_pic_by_POC().
	 *
	 * In MMU mode, canvas_id = buffer_index (one ANC2AXI entry per buf).
	 */
	{
		u32 slice_type = param->p.slice_type;
		int i;

		if (slice_type != 2 && hevc->ref_num_l0 > 0) {
			/* L0 references: start at canvas index 0 */
			amvdec_write_dos(core,
					 HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR,
					 (0 << 8) | (0 << 1) | 1);
			for (i = 0; i < hevc->ref_num_l0; i++) {
				int buf_idx = codec_hevc_find_buf_index(
					hevc, hevc->ref_poc_list0[i]);
				u32 canvas_val;

				if (buf_idx < 0) {
					/* Fallback: use prev_frame if available */
					buf_idx = hevc->prev_frame ?
						  hevc->prev_frame->index : 0;
				}
				canvas_val = (buf_idx << 16) |
					     (buf_idx << 8) | buf_idx;
				amvdec_write_dos(core,
					HEVCD_MPP_ANC_CANVAS_DATA_ADDR,
					canvas_val);
			}
			dev_dbg(core->dev,
				"MC config: slice=%u L0=%d L1=%d POC=%d held=0x%lx\n",
				slice_type, hevc->ref_num_l0,
				hevc->ref_num_l1, hevc->curr_poc,
				hevc->ref_held);

			if (slice_type == 0 && hevc->ref_num_l1 > 0) {
				/* B-slice: L1 references at canvas index 16 */
				amvdec_write_dos(core,
					HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR,
					(16 << 8) | (0 << 1) | 1);
				for (i = 0; i < hevc->ref_num_l1; i++) {
					int buf_idx = codec_hevc_find_buf_index(
						hevc,
						hevc->ref_poc_list1[i]);
					u32 canvas_val;

					if (buf_idx < 0) {
						buf_idx = hevc->prev_frame ?
							hevc->prev_frame->index : 0;
					}
					canvas_val = (buf_idx << 16) |
						     (buf_idx << 8) | buf_idx;
					amvdec_write_dos(core,
						HEVCD_MPP_ANC_CANVAS_DATA_ADDR,
						canvas_val);
				}
			}
		}
	}

	codec_hevc_set_sao(sess, &hevc->cur_frame->vbuf->vb2_buf,
			   hevc->new_pic);
	codec_hevc_set_mpred(core, hevc);
	codec_hevc_set_mcrcc(sess);

	/* Update DBLK config */
	amvdec_write_dos(core, HEVC_DBLK_CFG4,
			 hevc->workspace_paddr + DBLK_PARA_OFFSET);
	amvdec_write_dos(core, HEVC_DBLK_CFG5,
			 hevc->workspace_paddr + DBLK_DATA_OFFSET);
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, HEVC_DBLK_CFGE,
				 hevc->workspace_paddr + DBLK_DATA2_OFFSET);

	/* SAO picture size and LCU count (BSP config_sao_hw) */
	amvdec_write_dos(core, HEVC_SAO_PIC_SIZE,
			 hevc->width | (hevc->height << 16));
	amvdec_write_dos(core, HEVC_SAO_PIC_SIZE_LCU,
			 (lcu_width - 1) | ((lcu_height - 1) << 16));

	/* SAO_CTRL0: read-modify-write, only set lcu_size_log2 in [3:0] */
	val = amvdec_read_dos(core, HEVC_SAO_CTRL0);
	val &= ~0xf;
	val |= lcu_size_log2;

	/* SAO_CTRL0 bits [9:8]: loop filter across tiles/slices (BSP) */
	data32_2 = val;
	data32_2 &= ~0x300;
	if (param->p.tiles_enabled_flag)
		data32_2 |= ((misc_flag0 >>
			      LOOP_FILER_ACROSS_TILES_ENABLED_FLAG_BIT) & 1)
			     << 8;

	slice_deblocking_filter_disabled = (misc_flag0 >>
		SLICE_DEBLOCKING_FILTER_DISABLED_FLAG_BIT) & 1;

	if ((misc_flag0 & BIT(PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT))
	    && ((misc_flag0 & BIT(SLICE_SAO_LUMA_FLAG_BIT))
		|| (misc_flag0 & BIT(SLICE_SAO_CHROMA_FLAG_BIT))
		|| !slice_deblocking_filter_disabled)) {
		data32_2 |= ((misc_flag0 >>
			      SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT)
			     & 1) << 9;
	} else {
		data32_2 |= ((misc_flag0 >>
			      PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT)
			     & 1) << 9;
	}
	amvdec_write_dos(core, HEVC_SAO_CTRL0, data32_2);

	/* DBLK_CFG2: picture dimensions (BSP config_sao_hw) */
	amvdec_write_dos(core, HEVC_DBLK_CFG2,
			 hevc->width | (hevc->height << 16));

	/* DBLK_CFG1: LCU size + QP offsets + PCM filter (BSP config_sao_hw) */
	if ((misc_flag0 >> PCM_ENABLE_FLAG_BIT) & 1)
		cfg1 = ((misc_flag0 >> PCM_LOOP_FILTER_DISABLED_FLAG_BIT) & 1)
		       << 3;
	else
		cfg1 = 0;
	cfg1 |= (param->p.pps_cb_qp_offset & 0x1f) << 4;
	cfg1 |= (param->p.pps_cr_qp_offset & 0x1f) << 9;
	cfg1 |= (lcu_size == 64) ? 0 : ((lcu_size == 32) ? 1 : 2);
	if (hevc->width <= 64)
		cfg1 |= BIT(20);
	amvdec_write_dos(core, HEVC_DBLK_CFG1, cfg1);

	/* G12A+: select dblk_main pipeline (BSP config_sao_hw).
	 * STS1+4 (0xd434) = undocumented DBLK pipeline select register.
	 * BIT(28) enables the HEVC dblk_main decode path.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, HEVC_DBLK_STS1 + 4, BIT(28));

	/* DBLK_CFG9: per-slice deblocking params (BSP config_sao_hw) */
	cfg9 = 0;
	if (param->p.tiles_enabled_flag)
		cfg9 |= ((misc_flag0 >>
			  LOOP_FILER_ACROSS_TILES_ENABLED_FLAG_BIT) & 1);

	if ((misc_flag0 & BIT(DEBLOCKING_FILTER_OVERRIDE_ENABLED_FLAG_BIT))
	    && (misc_flag0 & BIT(DEBLOCKING_FILTER_OVERRIDE_FLAG_BIT))) {
		cfg9 |= slice_deblocking_filter_disabled << 2;
		if (!slice_deblocking_filter_disabled) {
			cfg9 |= (param->p.slice_beta_offset_div2 & 0xf) << 3;
			cfg9 |= (param->p.slice_tc_offset_div2 & 0xf) << 7;
		}
	} else {
		cfg9 |= ((misc_flag0 >>
			  PPS_DEBLOCKING_FILTER_DISABLED_FLAG_BIT) & 1) << 2;
		if (!((misc_flag0 >> PPS_DEBLOCKING_FILTER_DISABLED_FLAG_BIT)
		      & 1)) {
			cfg9 |= (param->p.pps_beta_offset_div2 & 0xf) << 3;
			cfg9 |= (param->p.pps_tc_offset_div2 & 0xf) << 7;
		}
	}

	if ((misc_flag0 & BIT(PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT))
	    && ((misc_flag0 & BIT(SLICE_SAO_LUMA_FLAG_BIT))
		|| (misc_flag0 & BIT(SLICE_SAO_CHROMA_FLAG_BIT))
		|| !slice_deblocking_filter_disabled)) {
		cfg9 |= ((misc_flag0 >>
			  SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT)
			 & 1) << 1;
	} else {
		cfg9 |= ((misc_flag0 >>
			  PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED_FLAG_BIT)
			 & 1) << 1;
	}
	amvdec_write_dos(core, HEVC_DBLK_CFG9, cfg9);

	dev_dbg(core->dev,
		"HEVC setup: %ux%u CFGB=0x%x SAO_Y=0x%08x CTRL1=0x%x CTRL0=0x%x PC=0x%x\n",
		hevc->width, hevc->height,
		amvdec_read_dos(core, HEVC_DBLK_CFGB),
		amvdec_read_dos(core, HEVC_SAO_Y_START_ADDR),
		amvdec_read_dos(core, HEVC_SAO_CTRL1),
		amvdec_read_dos(core, HEVC_SAO_CTRL0),
		amvdec_read_dos(core, HEVC_MPC_E));

	/* Signal firmware to decode via INT2 handler (status 5 path).
	 * The INT2 handler calls sao_config which configures internal SAO
	 * hardware registers, then starts decode via CR 0x0114.
	 *
	 * The INT2 handler masks INT1 on entry (MCPU_INTR_MSK = 0xFFFFFFFE).
	 * We must re-enable INT1 from the host so the decode completion
	 * interrupt (INT1 at 0x02aa) can fire during or after decode.
	 */
	/* Signal firmware to decode via INT2 handler.
	 * The INT2 handler at 0x00f8 processes CODED_SLICE_SEGMENT_DAT:
	 *   - Calls sao_config (configures internal SAO/DBLK HW registers)
	 *   - Starts decode engine (CR 0x0114)
	 *   - Enters CTB processing loop
	 *   - When all CTBs done: INT1 fires, calls wait_decode_done
	 *     (flushes SAO/DBLK pipeline for last CTB row)
	 *   - Sends DECPIC_DATA_DONE or NAL_SEARCH_DONE
	 *
	 * We must unmask INT1 (bit 1) so the decode completion handler
	 * can fire during the CTB processing.
	 */
	{
		u32 msk = amvdec_read_dos(core, HEVC_MCPU_INTR_MSK);
		dev_dbg(core->dev,
			"Pre-signal: INTR_MSK=0x%x PC=0x%x\n",
			msk, amvdec_read_dos(core, HEVC_MPC_E));
	}
	amvdec_write_dos(core, HEVC_DEC_STATUS_REG,
			 HEVC_CODED_SLICE_SEGMENT_DAT);
	/* MCPU_INTR_MSK = ~(BIT(1)|BIT(2)): unmask INT1 (decode done)
	 * and INT2 (slice segment). BIT(1)=INT1 mask, BIT(2)=INT2 mask.
	 * Both must be unmasked for the firmware's CTB processing loop.
	 */
	amvdec_write_dos(core, HEVC_MCPU_INTR_MSK, 0xFFFFFFF9);
	amvdec_write_dos(core, HEVC_MCPU_INTR_REQ, AMRISC_MAIN_REQ);

}

static void codec_hevc_resume(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;
	int ret;

	dev_dbg(core->dev, "HEVC resume called, %ux%u\n",
		sess->width, sess->height);

	mutex_lock(&hevc->lock);

	/* Set up canvas/buffer infrastructure for new resolution */
	if (hevc_needs_compress_ref(core, hevc->is_10bit)) {
		codec_hevc_alloc_compress_bufs(sess, &hevc->common);
		codec_hevc_setup_compress_anc2axi(sess, &hevc->common);
	} else {
		ret = codec_hevc_setup_buffers(sess, &hevc->common, hevc->is_10bit);
		if (ret) {
			dev_err(core->dev,
				"HEVC resume: buffer setup failed (%d), aborting\n",
				ret);
			amvdec_write_dos(core, HEVC_DEC_STATUS_REG, HEVC_ACTION_DONE);
			amvdec_write_dos(core, HEVC_MCPU_INTR_REQ, AMRISC_MAIN_REQ);
			mutex_unlock(&hevc->lock);
			return;
		}
	}

	/* Write MMU map address after setup_buffers allocates it */
	if (codec_hevc_use_mmu(core->platform->revision, sess->pixfmt_cap,
			       hevc->is_10bit))
		codec_hevc_write_mmu_map_addr(core,
					      hevc->common.mmu_map_paddr);

	/* Restore DECODE_SIZE before signaling the firmware.
	 * The BUFEMPTY handler clears DECODE_SIZE to 0 to stop ISR
	 * floods. When DECODE_SIZE == 0, the firmware's completion_check
	 * always returns "not done" and DECPIC_DATA_DONE is blocked.
	 * The pre_queue callback normally restores this, but between
	 * src_change and resume, no buffers are queued.
	 */
	amvdec_write_dos(core, HEVC_DECODE_SIZE, 0x7FFFFFFF);

	/* Decode the pending frame that triggered the resolution change. */
	codec_hevc_compute_poc(hevc);
	dev_dbg(core->dev, "HEVC resume: POC=%d, getting frame\n",
		 hevc->curr_poc);

	hevc->new_pic = 1;
	hevc->slice_idx = 0;

	hevc->cur_frame = codec_hevc_get_new_frame(sess);
	if (!hevc->cur_frame) {
		dev_err(core->dev, "No frame available for HEVC resume\n");
		amvdec_write_dos(core, HEVC_DEC_STATUS_REG,
				 HEVC_ACTION_DONE);
		mutex_unlock(&hevc->lock);
		return;
	}
	hevc->cur_frame->poc = hevc->curr_poc;

	codec_hevc_setup_decode(sess);

	mutex_unlock(&hevc->lock);
}

static void codec_hevc_show_frame(struct amvdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	struct hevc_frame *tmp, *n;

	list_for_each_entry_safe(tmp, n, &hevc->ref_frames_list, list) {
		if (tmp->done || !tmp->show)
			continue;

		amvdec_dst_buf_done(sess, tmp->vbuf, V4L2_FIELD_NONE);
		tmp->done = 1;
		hevc->frames_num--;
		list_del(&tmp->list);
		kfree(tmp);
	}
}

static irqreturn_t codec_hevc_threaded_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;
	u32 dec_status;

	if (!hevc)
		return IRQ_HANDLED;

	mutex_lock(&hevc->lock);
	dec_status = hevc->dec_status;

	if (dec_status) {
		dev_dbg(core->dev,
			"HEVC ISR: st=0x%x DS=0x%x PC=0x%x\n",
			dec_status,
			amvdec_read_dos(core, HEVC_DECODE_SIZE),
			amvdec_read_dos(core, HEVC_MPC_E));
	}

	switch (dec_status) {
	case HEVC_SLICE_SEGMENT_DONE:
	case HEVC_HEAD_PARSER_DONE:
		if (dec_status == HEVC_HEAD_PARSER_DONE)
			atomic_inc(&core->debugfs.irq.hevc_head_parser_done);
		dev_dbg(core->dev, "HEVC SLICE/HEAD: st=0x%x\n", dec_status);
		sess->keyframe_found = 1;

		codec_hevc_fetch_rpm(sess);
		codec_hevc_compute_poc(hevc);

		dev_dbg(core->dev,
			"HEVC rpm: %ux%u slice_type=%u POClsb=%u POC=%d seg_addr=%u tmvp=%u bit_depth=0x%x is_10bit=%d\n",
			 hevc->rpm_param.p.pic_width_in_luma_samples,
			 hevc->rpm_param.p.pic_height_in_luma_samples,
			 hevc->rpm_param.p.slice_type,
			 hevc->rpm_param.p.POClsb,
			 hevc->curr_poc,
			 hevc->rpm_param.p.slice_segment_address,
			 hevc->rpm_param.p.slice_temporal_mvp_enable_flag,
			 hevc->bitdepth_offset >= 0 ?
				hevc->rpm_param.l.data[hevc->bitdepth_offset] : 0,
			 hevc->is_10bit);

		if (codec_hevc_process_rpm(hevc)) {
			/* DPB size from SPS: max_dec_pic_buffering + 2.
			 * The extra buffer ensures get_new_frame() always
			 * finds a non-referenced buffer when ref_held
			 * protects reference FBC bodies.
			 */
			u32 dpb = hevc->rpm_param.p.sps_max_dec_pic_buffering_minus1_0 + 2;
			if (dpb < 5)
				dpb = 5;  /* 1 decode + 1 ref + 1 output + 2 margin */
			if (dpb > MAX_REF_PIC_NUM)
				dpb = MAX_REF_PIC_NUM;
			hevc->cur_frame = NULL;
			/* Drop the lock before amvdec_src_change because its
			 * fast-resume path may call codec_hevc_resume() which
			 * also takes hevc->lock -- deadlock if we hold it.
			 */
			mutex_unlock(&hevc->lock);
			amvdec_src_change(sess, hevc->width, hevc->height,
					  dpb);
			/* Don't signal firmware -- leave it paused until
			 * resume() is called after capture buffers are set up.
			 * Lock is already released, return directly.
			 */
			return IRQ_HANDLED;
		}

		if (!hevc->width || !hevc->height) {
			/* Resolution not yet known, ack and wait */
			amvdec_write_dos(core, HEVC_DEC_STATUS_REG,
					 HEVC_ACTION_DONE);
			goto unlock;
		}

		if (hevc->rpm_param.p.slice_segment_address == 0) {
			/* === NEW PICTURE (first slice) === */
			hevc->new_pic = 1;
			hevc->slice_idx = 0;

			/* Cache sps_num_reorder_pics for the bumping decision.
			 * RPM is parsed per-slice; cache on first-slice so the
			 * value is stable for the rest of the picture.
			 */
			hevc->sps_num_reorder_pics_0 =
				hevc->rpm_param.p.sps_num_reorder_pics_0;

			codec_hevc_apply_rps(hevc);

			/* Mark previous picture as decode-complete and bump:
			 * release the lowest-POC frame if we have more buffered
			 * than sps_num_reorder_pics_0 frames (HEVC Annex C DPB
			 * bumping, mirrors BSP vh265.c:output_pic()).
			 */
			if (hevc->cur_frame) {
				hevc->cur_frame->output_ready = 1;
				hevc->prev_frame = hevc->cur_frame;
				codec_hevc_bump_frames(sess, false);
			}

			/* Recycle done frames no longer needed as refs */
			{
				struct hevc_frame *tmp, *n;

				list_for_each_entry_safe(tmp, n,
						&hevc->ref_frames_list,
						list) {
					if (!tmp->done)
						continue;
					if (test_bit(tmp->index,
						     &hevc->ref_held))
						continue;
					list_del(&tmp->list);
					kfree(tmp);
				}
			}

			hevc->cur_frame =
				codec_hevc_get_new_frame(sess);
			if (!hevc->cur_frame) {
				dev_dbg(core->dev,
					"HEVC: no dst buf for POC=%d, deferring\n",
					hevc->curr_poc);
				schedule_delayed_work(
					&hevc->action_done_work,
					msecs_to_jiffies(10));
				goto unlock;
			}
			hevc->cur_frame->poc = hevc->curr_poc;
		} else {
			/* === CONTINUATION SLICE === */
			hevc->new_pic = 0;
			hevc->slice_idx++;

			if (!hevc->cur_frame) {
				dev_dbg(core->dev,
					"HEVC: continuation slice but no cur_frame, skipping\n");
				amvdec_write_dos(core,
						 HEVC_DEC_STATUS_REG,
						 HEVC_ACTION_DONE);
				goto unlock;
			}
		}

		codec_hevc_setup_decode(sess);
		break;

	case HEVC_DECPIC_DATA_DONE:
	case HEVC_FIND_NEXT_PIC_NAL: {
		u32 y_wptr = amvdec_read_dos(core, HEVC_SAO_Y_WPTR);

		hevc->done_count = 0;
		hevc->last_sao_y_wptr = y_wptr;

		dev_dbg(core->dev,
			"HEVC DONE: st=0x%x Y_WPTR=0x%x Y_LEN=0x%x C_LEN=0x%x Y_START=0x%x C_START=0x%x CTRL1=0x%x\n",
			dec_status, y_wptr,
			amvdec_read_dos(core, HEVC_SAO_Y_LENGTH),
			amvdec_read_dos(core, HEVC_SAO_C_LENGTH),
			amvdec_read_dos(core, HEVC_SAO_Y_START_ADDR),
			amvdec_read_dos(core, HEVC_SAO_C_START_ADDR),
			amvdec_read_dos(core, HEVC_SAO_CTRL1));
		/* Dump MMU header and FBC body for first 3 frames */
		if (hevc->cur_frame && hevc->cur_frame->index < 3) {
			int idx = hevc->cur_frame->index;
			u32 *hdr = hevc->common.mmu_header_vaddr[idx];
			u32 *body = hevc->common.fbc_buffer_vaddr[idx];

			if (hdr)
				dev_dbg(core->dev,
					"MMU HDR[%d]: %08x %08x %08x %08x %08x %08x %08x %08x\n",
					idx, hdr[0], hdr[1], hdr[2], hdr[3],
					hdr[4], hdr[5], hdr[6], hdr[7]);
			if (body)
				dev_dbg(core->dev,
					"FBC BODY[%d]: %08x %08x %08x %08x %08x %08x %08x %08x\n",
					idx, body[0], body[1], body[2], body[3],
					body[4], body[5], body[6], body[7]);
		}

		/* Slice decode complete.  For multi-slice pictures the
		 * firmware sends DECPIC_DATA_DONE after each slice, not
		 * after the whole picture.  Do NOT output the frame here
		 * -- frame output happens in SLICE_SEGMENT_DONE (when
		 * seg_addr=0 signals a new picture) or NAL_DECODE_DONE
		 * (end of stream).  Keep cur_frame alive so continuation
		 * slices can decode into it.
		 */
		/* NOTE: Do NOT clear ref_held here. The RPS for the NEXT
		 * frame will clear/set bits appropriately in apply_rps().
		 */

		/* Defer ACTION_DONE by 1ms to break the tight IRQ loop.
		 * Without this delay, the firmware fires the next
		 * SLICE_SEGMENT_DONE immediately, and the threaded ISR
		 * processes frames back-to-back at RT priority, starving
		 * userspace from recycling capture buffers.
		 */
		schedule_delayed_work(&hevc->action_done_work,
				      msecs_to_jiffies(1));
		break;
	}

	case HEVC_NAL_DECODE_DONE: {
		/* NAL_DECODE_DONE (0xe): all bitstream data consumed.
		 * The firmware's completion_check saw consumed >=
		 * DECODE_SIZE and sent this status.
		 *
		 * Output any pending frame.  Then write ACTION_DONE
		 * to break the firmware's spin loop.  Do NOT halt the
		 * firmware -- it returns to its main loop and continues
		 * polling DEC_STATUS_REG.  DECODE_SIZE remains at the
		 * sentinel value (0x7FFFFFFF) set by pre_queue, so the
		 * next completion_check returns 0 (not done) and the
		 * firmware enters the multi-slice path harmlessly.
		 */
		if (hevc->cur_frame) {
			hevc->cur_frame->output_ready = 1;
			hevc->prev_frame = hevc->cur_frame;
			hevc->cur_frame = NULL;
		}
		/* End of stream path: flush-drain the reorder buffer. */
		codec_hevc_bump_frames(sess, true);
		amvdec_write_dos(core, HEVC_DEC_STATUS_REG,
				 HEVC_ACTION_DONE);
		break;
	}

	case HEVC_OVER_DECODE:
		/* Firmware sends 0x0f (OVER_DECODE) and spins waiting for
		 * the driver to clear DEC_STATUS to 0. This happens when
		 * the firmware detects over-decode (decoded past expected
		 * boundary). ACK by writing 0 so firmware can continue.
		 */
		dev_dbg(core->dev, "HEVC OVER_DECODE\n");
		if (hevc->cur_frame) {
			hevc->cur_frame->output_ready = 1;
			hevc->prev_frame = hevc->cur_frame;
			hevc->cur_frame = NULL;
		}
		/* End of stream path: flush-drain the reorder buffer. */
		codec_hevc_bump_frames(sess, true);
		amvdec_write_dos(core, HEVC_DEC_STATUS_REG, 0);
		break;

	case HEVC_DECPIC_DATA_ERROR:
		dev_dbg(core->dev, "HEVC decode error\n");
		amvdec_write_dos(core, HEVC_DEC_STATUS_REG, HEVC_ACTION_DONE);
		break;

	case HEVC_NAL_SEARCH_DONE:
		/* Firmware found next NAL header. ACK and let it continue.
		 * Do NOT fire AMRISC_MAIN_REQ -- firmware polls DEC_STATUS.
		 */
		dev_dbg(core->dev,
			"HEVC NAL_SEARCH: Y_WPTR=0x%08x Y_START=0x%08x SAO_CTRL3=0x%x\n",
			 amvdec_read_dos(core, HEVC_SAO_Y_WPTR),
			 amvdec_read_dos(core, HEVC_SAO_Y_START_ADDR),
			 amvdec_read_dos(core, HEVC_SAO_CTRL3));
		amvdec_write_dos(core, HEVC_DEC_STATUS_REG, HEVC_ACTION_DONE);
		break;

	case HEVC_DECODE_OVER_SIZE:
		/* Firmware sends 0x23 when decode exceeds expected size.
		 * This is a fatal error -- firmware enters infinite loop.
		 * Abort the session.
		 */
		dev_err(core->dev, "HEVC decode over size error\n");
		amvdec_abort(sess);
		break;

	case 0:
		break;

	default:
		dev_dbg(core->dev, "Unexpected HEVC status: %08X\n",
			dec_status);
		amvdec_write_dos(core, HEVC_DEC_STATUS_REG, HEVC_ACTION_DONE);
		break;
	}

unlock:
	mutex_unlock(&hevc->lock);
	return IRQ_HANDLED;
}

static int codec_hevc_pre_queue(struct amvdec_session *sess,
			       struct vb2_buffer *vb)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;

	/* Set DECODE_SIZE to a large sentinel value.  The firmware's
	 * completion_check compares consumed bytes against DECODE_SIZE
	 * and fires NAL_DECODE_DONE (0xe) when consumed >= DECODE_SIZE.
	 * Using a sentinel (0x7FFFFFFF) ensures completion_check never
	 * fires prematurely.
	 *
	 * DECODE_SIZE must be non-zero for two firmware code paths:
	 * 1. DECPIC_DATA_DONE (0xa): gated on DECODE_SIZE != 0
	 * 2. buf_empty_check: skips monitoring when DECODE_SIZE == 0
	 */
	amvdec_write_dos(core, HEVC_DECODE_SIZE, 0x7FFFFFFF);

	return 0;
}

static irqreturn_t codec_hevc_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;
	u32 status;

	if (!hevc)
		return IRQ_HANDLED;

	status = amvdec_read_dos(core, HEVC_DEC_STATUS_REG);

	/* Handle BUFEMPTY in hardirq to avoid DEC_STATUS_REG race.
	 * If the threaded handler writes DEC_CONT for BUFEMPTY, it
	 * can overwrite a pending CODED_SLICE_SEGMENT_DAT before the
	 * firmware's INT2 handler processes it.  By responding here
	 * in the hardirq (before the threaded handler runs for any
	 * subsequent SLICE_SEGMENT_DONE), we ensure the firmware
	 * sees DEC_CONT while still in INT0, and then INT2 fires
	 * with the correct DEC_STATUS_REG value.
	 */
	if (status == HEVC_DECODE_BUFEMPTY ||
	    status == HEVC_SEARCH_BUFEMPTY ||
	    status == HEVC_DECODE_BUFEMPTY2) {
		/* KI-081: Do NOT clear DECODE_SIZE here.
		 * Firmware path 0x02b3 (g12a_hevc_mmu.asm): `beqi r8, 0, +11`
		 * skips DECPIC_DATA_DONE when DECODE_SIZE == 0.  P/B-frames
		 * trigger multiple BUFEMPTY events while the firmware processes
		 * NAL units; clearing DS during decode silently drops the frame.
		 * DECODE_SIZE=0 for teardown is handled in flush_output().
		 */
		amvdec_write_dos(core, HEVC_DEC_STATUS_REG,
				 HEVC_ACTION_DEC_CONT);
		return IRQ_HANDLED;
	}

	hevc->dec_status = status;
	return IRQ_WAKE_THREAD;
}

/*
 * Halt the HEVC firmware at end of drain, matching BSP amhevc_stop().
 * gxl_hevc_mmu.bin / g12a_hevc_mmu.bin sit "idle polling the VIFIFO
 * for more data" after the last NAL is consumed (see
 * codec_hevc_flush_output comment above the counter force-reset).
 * Leaving the firmware in that state wastes CPU cycles and leaves the
 * HEVC engine in an indeterminate idle vs. running state when the
 * next session starts.
 *
 * The counter force-reset remains in codec_hevc_flush_output (see
 * the atomic_set(&esparser_queued_bufs, 0) block there).  Moving it
 * here would add up to 5 s to every drain (vdec_drain_work's
 * wait_event_timeout on the counter) because HEVC firmware does not
 * fire late-EOS ISRs that would decrement it -- the counter stays
 * stuck at its post-BUFEMPTY value.  Leaving the force-reset where
 * it is keeps drain latency bounded.
 *
 * Called from vdec_drain_work() AFTER vdec_wait_inactive() and
 * flush_pending_frames().
 */
static void codec_hevc_drain_finalize(struct amvdec_session *sess)
{
	amvdec_hevc_halt_firmware(sess->core);
	sess->firmware_halted = 1;
}

struct amvdec_codec_ops codec_hevc_ops = {
	.start = codec_hevc_start,
	.stop = codec_hevc_stop,
	.isr = codec_hevc_isr,
	.threaded_isr = codec_hevc_threaded_isr,
	.num_pending_bufs = codec_hevc_num_pending_bufs,
	.drain = codec_hevc_flush_output,
	.drain_finalize = codec_hevc_drain_finalize,
	.resume = codec_hevc_resume,
	.pre_queue = codec_hevc_pre_queue,
};

static int __init codec_hevc_mod_init(void)
{
	return amvdec_register_codec(V4L2_PIX_FMT_HEVC, &codec_hevc_ops);
}

static void __exit codec_hevc_mod_exit(void)
{
	amvdec_unregister_codec(V4L2_PIX_FMT_HEVC, &codec_hevc_ops);
}

module_init(codec_hevc_mod_init);
module_exit(codec_hevc_mod_exit);
MODULE_DESCRIPTION("Meson VDEC HEVC codec");
MODULE_LICENSE("GPL");
