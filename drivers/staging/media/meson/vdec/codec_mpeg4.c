// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024
 *
 * MPEG-4 Part 2 decoder, using VDEC_1 engine.
 * Based on BSP vmpeg4.c/vmpeg4_multi.c and upstream codec_mpeg12.c pattern.
 */

#include <linux/module.h>
#include <linux/soc/amlogic/meson-canvas.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "codec_mpeg4.h"
#include "dos_regs.h"
#include "esparser.h"
#include "hevc_regs.h"
#include "vdec_helpers.h"

#define SIZE_WORKSPACE		SZ_1M

/*
 * The MPEG-4 firmware addresses its workspace via hardcoded internal addresses
 * starting at DCAC_BUFF_START_IP. The MEM_OFFSET_REG tells the firmware the
 * delta between these internal addresses and the actual physical address of the
 * allocated workspace buffer.
 */
#define DCAC_BUFF_START_IP	0x02b00000

/* Firmware register mapping */
#define MP4_PIC_RATIO		AV_SCRATCH_5
#define MP4_ERR_COUNT		AV_SCRATCH_6
#define MP4_PIC_WH		AV_SCRATCH_7
#define MREG_BUFFERIN		AV_SCRATCH_8
#define MREG_BUFFEROUT		AV_SCRATCH_9
#define MP4_NOT_CODED_CNT	AV_SCRATCH_A
#define MP4_OFFSET_REG		AV_SCRATCH_C
#define MEM_OFFSET_REG		AV_SCRATCH_F

/* SM1 multi-instance firmware register aliases */
#define MREG_REF0		AV_SCRATCH_1
#define MREG_REF1		AV_SCRATCH_2

/* Number of context registers saved/restored between frames (SM1) */
#define SM1_CTX_REG_COUNT	10

#define NUM_BUFS		8

/*
 * Picture type codes extracted from BUFFEROUT bits [5:3].
 * These match the VOP coding type values in MPEG-4 Part 2.
 */
#define MPEG4_PIC_TYPE_I	0
#define MPEG4_PIC_TYPE_P	1
#define MPEG4_PIC_TYPE_B	2

/* BUFFEROUT bit fields */
#define MPEG4_BUFFEROUT_PIC_TYPE_SHIFT	3
#define MPEG4_BUFFEROUT_PIC_TYPE_MASK	0x7

/*
 * MDEC_PIC_DC_THRESH value from BSP vmpeg4_multi.c.
 * Configures the DC coefficient threshold for the MDEC block.
 * The value 0x404038aa is used by all BSP MPEG4 decoders.
 */
#define MDEC_PIC_DC_THRESH_MPEG4	0x404038aa

/*
 * BUFFEROUT == 2: firmware timeout / insufficient data notification.
 * BSP names this DEC_RESULT_DONE or DEC_RESULT_AGAIN.  Not a decoded
 * frame -- must be cleared without frame delivery.
 */
#define MPEG4_BUFFEROUT_TIMEOUT		2

/*
 * MPEG1_2_REG codec type values (BSP vmpeg4_multi.c).
 * The multi-instance firmware uses this register to select the bitstream
 * parser: MPEG4 Part 2 vs H.263.  The single-instance gxl_h263.bin
 * firmware also requires the correct type -- it shares the same VLD
 * front-end as the MPEG4 firmware.
 */
#define CODEC_TYPE_MPEG4	1
#define CODEC_TYPE_H263		2

struct codec_mpeg4 {
	void	  *workspace_vaddr;
	dma_addr_t workspace_paddr;
	bool	   ready;

	/*
	 * Pre-allocated canvases for the 8 buffer slots (NV12: Y + UV).
	 * Allocated and configured BEFORE MPSR=1 so firmware caches the
	 * IDs at startup.  Reconfigured in resume() to point at real
	 * vb2 buffer DMA addresses without reallocating IDs.
	 *
	 * Layout: prealloc_canvas[buf_idx][0]=Y, [buf_idx][1]=UV
	 */
	u8	   prealloc_canvas[NUM_BUFS][2];
	bool	   canvases_ready;

	/*
	 * Packed canvas IDs written to AV_SCRATCH_0..3 and AV_SCRATCH_G..J.
	 * Saved here because the firmware overwrites AV_SCRATCH_3 (MP4_RATE)
	 * during decode, making live register reads unreliable.
	 */
	u32	   canvas_packed[NUM_BUFS];

	/*
	 * Reference frame sliding window (BSP update_ref() pattern).
	 *
	 * The MPEG-4 firmware decodes I/P/B frames and delivers them in
	 * decode order, but display order requires a 2-frame delay for
	 * I/P frames.  The BSP driver maintains refs[0] (older ref) and
	 * refs[1] (newer ref).  On each I/P frame:
	 *   refs[0] = refs[1]  (shift)
	 *   refs[1] = current  (store new ref)
	 *   display refs[0]    (output the older reference)
	 * B frames are displayed immediately (they reference both refs).
	 *
	 * ref_buf_idx stores the V4L2 buffer index (0..NUM_BUFS-1), or
	 * -1 if the slot is empty.
	 */
	int	   ref_buf_idx[2];
	int	   ref_count;
	bool	   has_b_frames;

	/*
	 * SM1 multi-instance firmware state.
	 *
	 * The SM1 firmware (sm1_mpeg4_5.bin) is multi-instance: it halts
	 * after each decoded frame and requires the driver to save HW
	 * context, stop the firmware, restore context, and restart for
	 * the next frame.  These fields track the saved register state.
	 */
	u32	   ctx_regs[SM1_CTX_REG_COUNT];
	bool	   ctx_valid;
};

/*
 * Return the MPEG1_2_REG codec type value for the current session.
 * H.263 uses a different bitstream parser than MPEG4 inside the same
 * firmware; the firmware selects the parser based on this register.
 */
static u32 mpeg4_codec_type(struct amvdec_session *sess)
{
	if (sess->fmt_out->pixfmt == V4L2_PIX_FMT_H263)
		return CODEC_TYPE_H263;
	return CODEC_TYPE_MPEG4;
}

/*
 * Find the firmware buffer index from REC_CANVAS_ADDR.
 *
 * The MPEG4 firmware writes decoded frames through a canvas identified
 * by REC_CANVAS_ADDR.  The packed canvas ID must be matched against the
 * values written to AV_SCRATCH_0..3 (buffers 0-3) and AV_SCRATCH_G..J
 * (buffers 4-7) during canvas setup.
 *
 * Returns buffer index (0-7) on success, -1 if no match found.
 */
static int mpeg4_canvas_to_buf_idx(struct amvdec_core *core,
				   struct codec_mpeg4 *mpeg4)
{
	u32 rec_canvas = amvdec_read_dos(core, REC_CANVAS_ADDR);
	int j;

	for (j = 0; j < NUM_BUFS; j++) {
		if (mpeg4->canvas_packed[j] == rec_canvas)
			return j;
	}

	dev_dbg(core->dev,
		"MPEG4: no canvas match for REC_CANVAS_ADDR=0x%06x\n",
		rec_canvas);
	return -1;
}

/*
 * Save the packed canvas IDs that amvdec_set_canvases() wrote to
 * AV_SCRATCH registers.  Must be called right after amvdec_set_canvases()
 * before the firmware has a chance to overwrite any scratch registers.
 */
static void mpeg4_save_canvas_ids(struct amvdec_core *core,
				  struct codec_mpeg4 *mpeg4)
{
	int j;

	for (j = 0; j < 4; j++)
		mpeg4->canvas_packed[j] =
			amvdec_read_dos(core, AV_SCRATCH_0 + j * 4);
	for (j = 0; j < 4; j++)
		mpeg4->canvas_packed[4 + j] =
			amvdec_read_dos(core, AV_SCRATCH_G + j * 4);
}

/*
 * Pre-allocate 16 canvases (NUM_BUFS x 2 NV12 planes), point them at
 * the workspace buffer as a placeholder, pack their IDs into
 * AV_SCRATCH_0..3 / AV_SCRATCH_G..J before MPSR=1, and save the
 * packed values into mpeg4->canvas_packed[].
 *
 * The MPEG-4 firmware caches AV_SCRATCH_0..3,G..J at MPSR=1 startup
 * and ignores driver writes after.  Without this pre-allocation,
 * the resume()-time amvdec_set_canvases() write is a no-op (firmware
 * has already cached the firmware-default 0x070605 values, which
 * never match the driver-saved canvas_packed[]) and every per-frame
 * REC_CANVAS_ADDR=0x070605 fails canvas lookup -> 0 frames delivered.
 *
 * In codec_mpeg4_resume() we reconfigure these same canvas IDs to
 * point at the real capture buffer DMA addresses.  Firmware sees
 * its cached IDs unchanged, but the underlying DMA targets are now
 * the real buffers.
 */
static int mpeg4_prealloc_canvases(struct amvdec_session *sess,
				   struct codec_mpeg4 *mpeg4)
{
	struct amvdec_core *core = sess->core;
	u32 width = ALIGN(sess->width, 64);
	u32 height = ALIGN(sess->height, 32);
	u32 packed;
	int i, j, ret;

	for (i = 0; i < NUM_BUFS; i++) {
		for (j = 0; j < 2; j++) {
			ret = meson_canvas_alloc(core->canvas,
						 &mpeg4->prealloc_canvas[i][j]);
			if (ret) {
				dev_err(core->dev,
					"MPEG4: canvas alloc failed at buf=%d plane=%d\n",
					i, j);
				goto free_canvases;
			}
		}

		/* Y plane (full size) */
		meson_canvas_config(core->canvas, mpeg4->prealloc_canvas[i][0],
				    mpeg4->workspace_paddr, width, height,
				    MESON_CANVAS_WRAP_NONE,
				    MESON_CANVAS_BLKMODE_LINEAR,
				    MESON_CANVAS_ENDIAN_SWAP64);

		/* UV plane (half height) */
		meson_canvas_config(core->canvas, mpeg4->prealloc_canvas[i][1],
				    mpeg4->workspace_paddr, width, height / 2,
				    MESON_CANVAS_WRAP_NONE,
				    MESON_CANVAS_BLKMODE_LINEAR,
				    MESON_CANVAS_ENDIAN_SWAP64);

		/* Pack canvas IDs into firmware-expected u32 layout:
		 * For NV12 the firmware reads canvas IDs as
		 * 0xUUYYYY where YY is Y canvas and UU is UV canvas.
		 * MPEG-4 firmware uses 3-canvas YUV420 layout so we
		 * duplicate UV into the V slot.
		 */
		packed = (mpeg4->prealloc_canvas[i][1] << 16) |
			 (mpeg4->prealloc_canvas[i][1] << 8)  |
			 (mpeg4->prealloc_canvas[i][0]);

		if (i < 4)
			amvdec_write_dos(core, AV_SCRATCH_0 + i * 4, packed);
		else
			amvdec_write_dos(core, AV_SCRATCH_G + (i - 4) * 4,
					 packed);

		mpeg4->canvas_packed[i] = packed;
	}

	mpeg4->canvases_ready = true;
	return 0;

free_canvases:
	while (i >= 0) {
		while (--j >= 0)
			meson_canvas_free(core->canvas,
					  mpeg4->prealloc_canvas[i][j]);
		j = 2;
		i--;
	}
	return ret;
}

static void mpeg4_free_canvases(struct codec_mpeg4 *mpeg4,
				struct amvdec_core *core)
{
	int i, j;

	if (!mpeg4->canvases_ready)
		return;

	for (i = 0; i < NUM_BUFS; i++)
		for (j = 0; j < 2; j++)
			meson_canvas_free(core->canvas,
					  mpeg4->prealloc_canvas[i][j]);

	mpeg4->canvases_ready = false;
}

/*
 * Reconfigure the pre-allocated canvases to point at real capture
 * buffer DMA addresses.  Called from codec_mpeg4_resume() after the
 * V4L2 source-change handshake completes and CAPTURE buffers exist.
 *
 * Canvas IDs themselves are unchanged -- only the DMA target each
 * canvas points at is updated.  Firmware's cached canvas IDs from
 * MPSR=1 startup are still valid; they now reference the right
 * buffers.
 *
 * Handles NV12 contiguous (Y + UV in one plane) and NV12M (separate
 * Y and UV vb2 planes).  Other pixfmts fall back to a warning -- they
 * use the legacy amvdec_set_canvases path which doesn't survive the
 * MPSR=1 cache.
 */
static void mpeg4_reconfig_canvases_for_dst(struct amvdec_session *sess,
					    struct codec_mpeg4 *mpeg4)
{
	struct amvdec_core *core = sess->core;
	struct v4l2_m2m_buffer *buf;
	u32 width = ALIGN(sess->width, 64);
	u32 height = ALIGN(sess->height, 32);
	int i = 0;

	if (!mpeg4->canvases_ready) {
		dev_warn(core->dev,
			 "MPEG4: canvas reconfig called before pre-alloc\n");
		return;
	}

	v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
		struct vb2_buffer *vb = &buf->vb.vb2_buf;
		dma_addr_t y_addr, uv_addr;

		if (i >= NUM_BUFS)
			break;

		y_addr = vb2_dma_contig_plane_dma_addr(vb, 0);

		if (vb->num_planes >= 2)
			uv_addr = vb2_dma_contig_plane_dma_addr(vb, 1);
		else
			uv_addr = y_addr + width * height;

		meson_canvas_config(core->canvas,
				    mpeg4->prealloc_canvas[i][0],
				    y_addr, width, height,
				    MESON_CANVAS_WRAP_NONE,
				    MESON_CANVAS_BLKMODE_LINEAR,
				    MESON_CANVAS_ENDIAN_SWAP64);

		meson_canvas_config(core->canvas,
				    mpeg4->prealloc_canvas[i][1],
				    uv_addr, width, height / 2,
				    MESON_CANVAS_WRAP_NONE,
				    MESON_CANVAS_BLKMODE_LINEAR,
				    MESON_CANVAS_ENDIAN_SWAP64);
		i++;
	}
}

/*
 * sm1_mpeg4_save_context() - Save firmware HW context after a decoded frame.
 *
 * The SM1 multi-instance firmware overwrites these registers during decode.
 * They must be saved after each frame and restored before the next decode
 * to maintain codec state across the stop/restart cycle.
 *
 * BSP reference: vmpeg4_save_hw_context() in vmpeg4_multi.c:707-720.
 */
static void sm1_mpeg4_save_context(struct amvdec_core *core,
				   struct codec_mpeg4 *mpeg4)
{
	mpeg4->ctx_regs[0] = amvdec_read_dos(core, DC_AC_CTRL);
	mpeg4->ctx_regs[1] = amvdec_read_dos(core, IQIDCT_CONTROL);
	mpeg4->ctx_regs[2] = amvdec_read_dos(core, RESYNC_MARKER_LENGTH);
	mpeg4->ctx_regs[3] = amvdec_read_dos(core, RV_AI_MB_COUNT);
	mpeg4->ctx_regs[4] = amvdec_read_dos(core, MPEG1_2_REG);
	mpeg4->ctx_regs[5] = amvdec_read_dos(core, VCOP_CTRL_REG);
	mpeg4->ctx_regs[6] = amvdec_read_dos(core, PIC_HEAD_INFO);
	mpeg4->ctx_regs[7] = amvdec_read_dos(core, SLICE_QP);
	mpeg4->ctx_regs[8] = amvdec_read_dos(core, MB_INFO);
	mpeg4->ctx_regs[9] = amvdec_read_dos(core, MP4_PIC_WH);
	mpeg4->ctx_valid = true;
}

/*
 * sm1_mpeg4_restore_context() - Restore firmware HW context before decode.
 *
 * Writes back the saved register state so the firmware can continue decoding
 * from where it left off.  On first run (ctx_valid == false), writes initial
 * values instead: MPEG1_2_REG = 1 (MPEG4) or 2 (H.263) selects codec type.
 *
 * BSP reference: vmpeg4_multi.c:2618-2628.
 */
static void sm1_mpeg4_restore_context(struct amvdec_core *core,
				      struct codec_mpeg4 *mpeg4,
				      struct amvdec_session *sess)
{
	if (mpeg4->ctx_valid) {
		amvdec_write_dos(core, DC_AC_CTRL, mpeg4->ctx_regs[0]);
		amvdec_write_dos(core, IQIDCT_CONTROL, mpeg4->ctx_regs[1]);
		amvdec_write_dos(core, RESYNC_MARKER_LENGTH,
				 mpeg4->ctx_regs[2]);
		amvdec_write_dos(core, RV_AI_MB_COUNT, mpeg4->ctx_regs[3]);
	}
	amvdec_write_dos(core, MPEG1_2_REG,
			 mpeg4->ctx_valid ? mpeg4->ctx_regs[4] :
			 mpeg4_codec_type(sess));
	amvdec_write_dos(core, VCOP_CTRL_REG, mpeg4->ctx_regs[5]);
	amvdec_write_dos(core, PIC_HEAD_INFO, mpeg4->ctx_regs[6]);
	amvdec_write_dos(core, SLICE_QP, mpeg4->ctx_regs[7]);
	amvdec_write_dos(core, MB_INFO, mpeg4->ctx_regs[8]);
}

/*
 * sm1_mpeg4_setup_refs() - Write reference frame canvas addresses for SM1.
 *
 * The multi-instance firmware requires the driver to set up reference frame
 * registers before each decode.  MREG_REF0/REF1 point to the two most
 * recent I/P frames.  REC_CANVAS_ADDR and ANC2_CANVAS_ADDR point to the
 * output buffer for the current frame.
 *
 * BSP reference: vmpeg4_multi.c:2556-2569.
 */
static void sm1_mpeg4_setup_refs(struct amvdec_core *core,
				 struct codec_mpeg4 *mpeg4,
				 int output_buf_idx)
{
	u32 ref0, ref1, rec;

	/* REF0 = older reference, REF1 = newer reference */
	if (mpeg4->ref_buf_idx[0] == -1) {
		ref0 = (mpeg4->ref_buf_idx[1] == -1) ? 0xffffffff :
			mpeg4->canvas_packed[mpeg4->ref_buf_idx[1]];
	} else {
		ref0 = mpeg4->canvas_packed[mpeg4->ref_buf_idx[0]];
	}
	ref1 = (mpeg4->ref_buf_idx[1] == -1) ? 0xffffffff :
		mpeg4->canvas_packed[mpeg4->ref_buf_idx[1]];

	amvdec_write_dos(core, MREG_REF0, ref0);
	amvdec_write_dos(core, MREG_REF1, ref1);

	if (output_buf_idx >= 0 && output_buf_idx < NUM_BUFS) {
		rec = mpeg4->canvas_packed[output_buf_idx];
	} else {
		rec = 0xffffff;
	}
	amvdec_write_dos(core, REC_CANVAS_ADDR, rec);
	amvdec_write_dos(core, ANC2_CANVAS_ADDR, rec);
}

/*
 * sm1_mpeg4_restart() - Restart the SM1 multi-instance firmware for next frame.
 *
 * This implements the full per-frame restart sequence:
 * 1. Re-setup canvas registers (firmware may have overwritten them)
 * 2. Setup reference frame registers
 * 3. Restore HW context
 * 4. Set MREG_BUFFEROUT = 0x10000 (BSP init value for multi-instance)
 * 5. Set MREG_INPUT with ctx_valid flag
 * 6. Re-enable firmware processor (MPSR = 1)
 *
 * BSP reference: vmpeg4_multi.c:2540-2640 (the "run" function).
 */
static void sm1_mpeg4_restart(struct amvdec_session *sess,
			      int next_buf_idx)
{
	struct amvdec_core *core = sess->core;
	struct codec_mpeg4 *mpeg4 = sess->priv;

	/* Re-setup canvases (firmware may overwrite AV_SCRATCH_3) */
	amvdec_set_canvases(sess,
			    (u32[]){ AV_SCRATCH_0, AV_SCRATCH_G, 0 },
			    (u32[]){ 4, 4, 0 });
	mpeg4_save_canvas_ids(core, mpeg4);

	/* Setup reference frame registers */
	sm1_mpeg4_setup_refs(core, mpeg4, next_buf_idx);

	/* Restore workspace offset */
	amvdec_write_dos(core, MEM_OFFSET_REG,
			 mpeg4->workspace_paddr - DCAC_BUFF_START_IP);

	amvdec_write_dos(core, PSCALE_CTRL, 0);
	amvdec_write_dos(core, MP4_NOT_CODED_CNT, 0);

	/* MREG_BUFFEROUT = 0x10000: multi-instance init value (BSP:2594) */
	amvdec_write_dos(core, MREG_BUFFEROUT, 0x10000);

	/* Clear and enable mailbox interrupt */
	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);
	amvdec_write_dos(core, ASSIST_MBOX1_MASK, 1);

	amvdec_write_dos(core, MDEC_PIC_DC_THRESH, MDEC_PIC_DC_THRESH_MPEG4);

	/* Write pic dimensions: use saved value if context valid */
	amvdec_write_dos(core, MP4_PIC_WH,
			 mpeg4->ctx_valid ? mpeg4->ctx_regs[9] :
			 ((sess->width << 16) | sess->height));

	/* Restore HW context registers */
	sm1_mpeg4_restore_context(core, mpeg4, sess);

	/* MREG_INPUT: stream-based input with ctx_valid flag (BSP:2636) */
	amvdec_write_dos(core, MREG_BUFFERIN, (mpeg4->ctx_valid << 6));

	/* Restart firmware processor */
	amvdec_write_dos(core, MPSR, 1);
}

static const u8 eos_sequence[SZ_1K] = { 0x00, 0x00, 0x01, 0xB1 };

static const u8 *codec_mpeg4_eos_sequence(u32 *len)
{
	*len = ARRAY_SIZE(eos_sequence);
	return eos_sequence;
}

static int codec_mpeg4_can_recycle(struct amvdec_core *core)
{
	return !amvdec_read_dos(core, MREG_BUFFERIN);
}

static void codec_mpeg4_recycle(struct amvdec_core *core, u32 buf_idx)
{
	amvdec_write_dos(core, MREG_BUFFERIN, ~(1 << buf_idx));
}

/*
 * On G12A/G12B/SM1, VDEC_1 and HEVC share a combined power domain.
 * The HEVCD_IPP and HEVCD_MPP blocks must be initialized even for VDEC_1
 * codecs (MPEG4, MPEG12, MJPEG).  Without this, the hardware stalls
 * silently and the firmware never fires the first IRQ.
 *
 * Additionally, the ANC2AXI table must be populated with the physical
 * addresses of the decode buffers so the HEVC post-processing block
 * can perform address translation for VDEC_1 DMA accesses.
 *
 * BSP reference: vmpeg4_hw_ctx_restore() in vmpeg4_multi.c:2519-2586.
 */
static void codec_mpeg4_init_hevc_combine(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct v4l2_m2m_buffer *buf;
	u32 buf_y_paddr;
	u32 buf_uv_paddr;
	u32 width = ALIGN(sess->width, 64);
	u32 output_size = width * ALIGN(sess->height, 32);
	int idx = 0;

	if (core->platform->revision < VDEC_REVISION_G12A)
		return;

	/* Step 1: Reset and enable HEVCD_IPP block */
	amvdec_write_dos(core, HEVCD_IPP_TOP_CNTL, BIT(0));  /* reset */
	amvdec_write_dos(core, HEVCD_IPP_TOP_CNTL, BIT(1));  /* enable */

	/* Step 2: Enable MCR and decompression */
	amvdec_write_dos(core, HEVCD_MPP_VDEC_MCR_CTL, BIT(4) | BIT(0));
	amvdec_write_dos(core, HEVCD_MPP_DECOMP_CTL1, BIT(31));

	/* Step 3: Disable HEVC core (VDEC_1 path only) */
	amvdec_write_dos(core, HEVC_CORE_ENABLE, 0);

	/*
	 * Step 4: Populate ANC2AXI table with buffer physical addresses.
	 * Use auto-increment mode: write Y and UV addresses for each buffer.
	 */
	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR,
			 ANC2AXI_CONF_AUTO_INC);

	v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
		struct vb2_buffer *vb = &buf->vb.vb2_buf;

		if (vb->num_planes > 1) {
			buf_y_paddr = vb2_dma_contig_plane_dma_addr(vb, 0);
			buf_uv_paddr = vb2_dma_contig_plane_dma_addr(vb, 1);
		} else {
			buf_y_paddr = vb2_dma_contig_plane_dma_addr(vb, 0);
			buf_uv_paddr = buf_y_paddr + output_size;
		}

		/* Y plane address (shifted right 5 = divided by 32) */
		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
				 buf_y_paddr >> 5);
		/* UV plane address */
		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
				 buf_uv_paddr >> 5);
		idx++;
	}

	/* Fill remaining slots with last buffer address */
	for (; idx < 16; idx++) {
		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
				 core->scratch_paddr >> 5);
		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
				 core->scratch_paddr >> 5);
	}

	/* Commit table */
	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, 1);

	/* Zero out canvas data -- G12A+ uses AXI table, not canvas */
	amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1);
	for (idx = 0; idx < 32; idx++)
		amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, 0);

	/* Step 5: Configure MCRCC for progressive linear mode */
	amvdec_write_dos(core, HEVCD_MCRCC_CTL1, 0x2);  /* reset MCRCC */
	amvdec_write_dos(core, HEVCD_MCRCC_CTL1, 0xff0); /* progressive mode */

	/* Step 6: Configure IPP AXIIF for linear mode, endian 7 */
	amvdec_write_dos(core, HEVCD_IPP_AXIIF_CONFIG,
			 BIT(3) | 7);  /* linear mode + endian */
}

static int codec_mpeg4_start(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_mpeg4 *mpeg4;
	int ret;

	mpeg4 = kzalloc(sizeof(*mpeg4), GFP_KERNEL);
	if (!mpeg4)
		return -ENOMEM;

	mpeg4->workspace_vaddr = dma_alloc_coherent(core->dev, SIZE_WORKSPACE,
						    &mpeg4->workspace_paddr,
						    GFP_KERNEL);
	if (!mpeg4->workspace_vaddr) {
		dev_err(core->dev, "Failed to request MPEG4 Workspace\n");
		ret = -ENOMEM;
		goto free_mpeg4;
	}

	memset(mpeg4->workspace_vaddr, 0, SIZE_WORKSPACE);

	/*
	 * Full BSP IQIDCT/MC/VLD reset sequence -- match codec_mpeg12_start.
	 *
	 * The original MPEG4 reset was only DOS_SW_RESET0_MDEC, which does
	 * not clear VLD state.  If a prior codec session (e.g. H.264) left
	 * VLD in a non-idle state (VLD_STATUS_CTRL bits 0/5 set), the MPEG4
	 * firmware's main polling loop at 0x06b1 sees VLD busy and exits
	 * without decoding.  Matches KI-005 symptom: VLD_CTRL=0x21 stuck,
	 * FIFO RP frozen, PIC_HEAD=0.
	 *
	 * Mirror codec_mpeg12_start's full reset: MDEC | VLD twice, then
	 * MC, then MDEC_SW_RESET_MDEC_CORE, with readback barriers.
	 */
	amvdec_write_dos(core, DOS_SW_RESET0,
			 DOS_SW_RESET0_MDEC | DOS_SW_RESET0_VLD);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_write_dos(core, DOS_SW_RESET0,
			 DOS_SW_RESET0_MDEC | DOS_SW_RESET0_VLD);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_write_dos(core, DOS_SW_RESET0, DOS_SW_RESET0_MC);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_write_dos(core, MDEC_SW_RESET, MDEC_SW_RESET_MDEC_CORE);
	amvdec_write_dos(core, MDEC_SW_RESET, 0);

	/*
	 * On G12A+, clear IQIDCT_CONTROL to remove stale state from prior
	 * codec sessions (e.g. H.264 sets bit 16). Without this, PSNR
	 * degrades on successive decodes. Do not write on GXL -- the
	 * hardware reset value is correct there.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, IQIDCT_CONTROL, 0);

	amvdec_write_dos(core, POWER_CTL_VLD, POWER_CTL_VLD_ENABLE);

	/*
	 * Initialize PIC_HEAD_INFO to the known good value 0x380
	 * (matches codec_mpeg12_start).  Probe showed PIC_HEAD=0 on
	 * every sample, indicating the firmware's picture header tracking
	 * was starting from an uninitialized state.  MPEG12 explicitly
	 * sets this; MPEG4 previously did not.
	 */
	amvdec_write_dos(core, PIC_HEAD_INFO, PIC_HEAD_INFO_INIT);

	/*
	 * VLD FIFO endianness: do NOT write VLD_MEM_VIFIFO_CONTROL bits 5:3.
	 *
	 * ab395ea6f84b added an unconditional (7 << 3) write, claiming the
	 * VLD endian must match esparser's FETCH_ENDIAN_SWAP64.  868966eda959
	 * gated it to G12A+ after bisecting the GXL breakage.  Testing on
	 * G12B (A311D) 2026-04-21 proved the write also breaks G12B: 0/0/0
	 * IRQs with the write, 7/7/7 without (baseline 98e927c0126d).
	 *
	 * The VLD endian bits default correctly on all tested platforms
	 * (GXL, G12B) without explicit configuration.  The BSP reference
	 * (vmpeg4_multi.c:1164) sets them but in a multi-instance context
	 * with different firmware; our single-instance streaming firmware
	 * does not require it.
	 */

	amvdec_write_dos(core, MEM_OFFSET_REG,
			 mpeg4->workspace_paddr - DCAC_BUFF_START_IP);

	amvdec_write_dos(core, AV_SCRATCH_L, 0);
	amvdec_write_dos(core, AV_SCRATCH_E, 0); /* MP4_SYS_RATE init */
	amvdec_write_dos(core, PSCALE_CTRL, 0);
	amvdec_write_dos(core, MP4_NOT_CODED_CNT, 0);
	amvdec_write_dos(core, MREG_BUFFERIN, 0);

	/*
	 * SM1 multi-instance firmware expects MREG_BUFFEROUT = 0x10000
	 * as its initial value (BSP: vmpeg4_multi.c:2594).  Older
	 * single-instance firmware expects 0.
	 */
	if (core->platform->revision >= VDEC_REVISION_SM1)
		amvdec_write_dos(core, MREG_BUFFEROUT, 0x10000);
	else
		amvdec_write_dos(core, MREG_BUFFEROUT, 0);

	amvdec_write_dos(core, MP4_PIC_WH,
			 (sess->width << 16) | sess->height);
	amvdec_write_dos(core, MDEC_PIC_DC_THRESH, MDEC_PIC_DC_THRESH_MPEG4);

	/*
	 * SM1 requires MDEC_PIC_DC_CTRL bit 31 for proper decoder operation.
	 * SM1-specific firmware variants (mpeg12, xvid) set this bit in their
	 * init code.  Set it explicitly in the driver as well for robustness.
	 *
	 * Note: SM1 now uses sm1_mpeg4_5.bin (multi-instance firmware) which
	 * may set this bit itself.  The driver-side write is kept as a safety
	 * measure -- writing an already-set bit is harmless.
	 */
	if (core->platform->revision == VDEC_REVISION_SM1)
		amvdec_write_dos_bits(core, MDEC_PIC_DC_CTRL,
				      MDEC_PIC_DC_CTRL_OLD_MODE);

	/*
	 * Select codec type: MPEG4 (1) or H.263 (2).
	 *
	 * The MPEG4 firmware uses MPEG1_2_REG to select the bitstream
	 * parser.  On G12A/G12B, H.263 now shares the MPEG4 firmware
	 * (gxl_h263.bin doesn't produce interrupts on G12+ hardware),
	 * so this register is required on all platforms.
	 *
	 * On GXL with the dedicated gxl_h263.bin, the firmware sets this
	 * internally, but writing it explicitly is harmless and ensures
	 * correctness if the firmware changes.
	 *
	 * BSP reference: vmpeg4_multi.c:2624.
	 */
	amvdec_write_dos(core, MPEG1_2_REG, mpeg4_codec_type(sess));

	/*
	 * SM1 multi-instance firmware init: set MREG_INPUT with
	 * ctx_valid=0 for first run.
	 * BSP reference: vmpeg4_multi.c:2636.
	 */
	if (core->platform->revision >= VDEC_REVISION_SM1)
		amvdec_write_dos(core, MREG_BUFFERIN, 0);

	sess->priv = mpeg4;

	/* Pre-allocate 16 canvases (NUM_BUFS x NV12 Y/UV) and write their
	 * packed IDs to AV_SCRATCH_0..3,G..J BEFORE MPSR=1 so firmware
	 * caches valid canvas IDs.  See mpeg4_prealloc_canvases() comment
	 * for rationale.  The legacy amvdec_set_canvases path was a no-op
	 * here (no CAPTURE buffers yet) and lost the firmware cache.
	 *
	 * SM1 takes a different lifecycle (sm1_mpeg4_restart re-allocates
	 * canvases per-frame); its single-shot firmware doesn't need or
	 * tolerate the pre-allocation.  Stick to the legacy path on SM1.
	 */
	if (core->platform->revision < VDEC_REVISION_SM1) {
		ret = mpeg4_prealloc_canvases(sess, mpeg4);
		if (ret)
			goto free_workspace;
	} else {
		ret = amvdec_set_canvases(sess,
					  (u32[]){ AV_SCRATCH_0, AV_SCRATCH_G, 0 },
					  (u32[]){ 4, 4, 0 });
		if (ret)
			goto free_workspace;
		mpeg4_save_canvas_ids(core, mpeg4);
	}

	codec_mpeg4_init_hevc_combine(sess);

	mpeg4->ref_buf_idx[0] = -1;
	mpeg4->ref_buf_idx[1] = -1;
	mpeg4->ref_count = 0;
	mpeg4->ctx_valid = false;
	/* Do NOT set mpeg4->ready=true here.  It must stay false until
	 * codec_mpeg4_resume() configures canvases.  Setting it true in
	 * start() permanently disables the !ready branch of
	 * codec_mpeg4_threaded_isr that emits the src_change V4L2 event,
	 * which gst-launch needs to advance from PREROLLING to PLAYING.
	 * Without src_change, capture queue is never streamed on, no
	 * capture buffers, no frames delivered.  kzalloc already
	 * initialised this field to false.
	 */
	sess->keyframe_found = 1;

	return 0;

free_workspace:
	dma_free_coherent(core->dev, SIZE_WORKSPACE, mpeg4->workspace_vaddr,
			  mpeg4->workspace_paddr);
free_mpeg4:
	kfree(mpeg4);
	return ret;
}

static void codec_mpeg4_resume(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_mpeg4 *mpeg4 = sess->priv;

	if (core->platform->revision < VDEC_REVISION_SM1) {
		/* Reconfigure the pre-allocated canvas IDs (already written
		 * to AV_SCRATCH_0..3,G..J in start() before MPSR=1) to point
		 * at the real CAPTURE buffer DMA addresses.  Firmware's
		 * cached canvas IDs from MPSR=1 startup are still valid;
		 * only the underlying DMA targets change.
		 */
		mpeg4_reconfig_canvases_for_dst(sess, mpeg4);
	} else {
		/* SM1 multi-instance firmware: legacy reallocate-per-resume
		 * path.  sm1_mpeg4_restart re-runs amvdec_set_canvases each
		 * frame so the cached-at-startup contract doesn't apply.
		 */
		amvdec_set_canvases(sess,
				    (u32[]){ AV_SCRATCH_0, AV_SCRATCH_G, 0 },
				    (u32[]){ 4, 4, 0 });
		mpeg4_save_canvas_ids(core, mpeg4);
	}

	codec_mpeg4_init_hevc_combine(sess);

	amvdec_write_dos(core, MP4_PIC_WH,
			 (sess->width << 16) | sess->height);

	mpeg4->ready = true;
}

static int codec_mpeg4_stop(struct amvdec_session *sess)
{
	struct codec_mpeg4 *mpeg4 = sess->priv;
	struct amvdec_core *core = sess->core;

	mpeg4_free_canvases(mpeg4, core);

	if (mpeg4->workspace_vaddr) {
		dma_free_coherent(core->dev, SIZE_WORKSPACE,
				  mpeg4->workspace_vaddr,
				  mpeg4->workspace_paddr);
		mpeg4->workspace_vaddr = NULL;
	}

	/* Note: sess->priv is freed by vdec_stop_streaming() */
	return 0;
}

static irqreturn_t codec_mpeg4_isr(struct amvdec_session *sess)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t codec_mpeg4_threaded_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_mpeg4 *mpeg4 = sess->priv;
	u32 reg;
	u32 pic_wh, width, height;
	int buffer_index;
	int pic_type;
	int display_idx;

	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);

	/*
	 * Before real capture buffers are available, use the first ISR
	 * to trigger a source change event. The firmware writes the
	 * detected resolution to MP4_PIC_WH.
	 */
	if (!mpeg4->ready) {
		pic_wh = amvdec_read_dos(core, MP4_PIC_WH);
		width = pic_wh >> 16;
		height = pic_wh & 0xFFFF;

		dev_dbg(core->dev,
			"MPEG4 first ISR: pic_wh=%08x (%ux%u) status=%d\n",
			pic_wh, width, height, sess->status);

		if (width && height &&
		    sess->status != STATUS_NEEDS_RESUME)
			amvdec_src_change(sess, width, height, NUM_BUFS);

		amvdec_write_dos(core, MREG_BUFFEROUT, 0);
		return IRQ_HANDLED;
	}

	reg = amvdec_read_dos(core, MREG_BUFFEROUT);

	/*
	 * SM1 multi-instance firmware uses 0x10000 as the "no frame"
	 * sentinel instead of 0.  A value <= 0x10000 means no frame.
	 */
	if (core->platform->revision >= VDEC_REVISION_SM1) {
		if (reg <= 0x10000)
			return IRQ_HANDLED;
	} else {
		if (!reg)
			return IRQ_HANDLED;
	}

	/*
	 * BUFFEROUT == 2 signals a firmware timeout / insufficient data
	 * condition (BSP: DEC_RESULT_DONE or DEC_RESULT_AGAIN).  This is
	 * not a decoded frame -- save context on SM1 and clear.
	 */
	if (reg == MPEG4_BUFFEROUT_TIMEOUT) {
		if (core->platform->revision >= VDEC_REVISION_SM1) {
			sm1_mpeg4_save_context(core, mpeg4);
			sm1_mpeg4_restart(sess, -1);
		} else {
			amvdec_write_dos(core, MREG_BUFFEROUT, 0);
		}
		return IRQ_HANDLED;
	}

	/*
	 * The MPEG4 firmware does NOT encode the buffer index in
	 * BUFFEROUT bits [2:0] (unlike MPEG12).  Instead, BUFFEROUT
	 * contains picture type metadata in bits [5:3]:
	 *   0 = I-frame, 1 = P-frame, 2 = B-frame
	 * The actual decoded buffer is identified by REC_CANVAS_ADDR,
	 * which holds the packed canvas ID of the output frame.
	 */
	pic_type = (reg >> MPEG4_BUFFEROUT_PIC_TYPE_SHIFT) &
		   MPEG4_BUFFEROUT_PIC_TYPE_MASK;

	buffer_index = mpeg4_canvas_to_buf_idx(core, mpeg4);
	if (buffer_index < 0) {
		if (core->platform->revision >= VDEC_REVISION_SM1) {
			sm1_mpeg4_save_context(core, mpeg4);
			sm1_mpeg4_restart(sess, -1);
		} else {
			amvdec_write_dos(core, MREG_BUFFEROUT, 0);
		}
		return IRQ_HANDLED;
	}

	/*
	 * SM1 multi-instance: save HW context before updating the
	 * sliding window.  The firmware has halted; register state
	 * must be captured now before we stop the processor.
	 * BSP reference: vmpeg4_multi.c:1549 (save before schedule_work).
	 */
	if (core->platform->revision >= VDEC_REVISION_SM1)
		sm1_mpeg4_save_context(core, mpeg4);

	/*
	 * Reference frame sliding window (BSP update_ref() pattern).
	 *
	 * For streams with B-frames (ASP): I/P frames are delayed by one
	 * in the sliding window so B-frames can be inserted in display
	 * order.  B-frames are displayed immediately.
	 *
	 * For I/P-only streams (SP or ASP without B-frames): decode order
	 * IS display order.  The sliding window is harmful -- at GOP
	 * boundaries the stale ref window conflicts with the new GOP's
	 * reference chain, producing jumbled delivery order with
	 * alternating ~20 dB / ~65 dB PSNR per frame.
	 *
	 * Detection: set has_b_frames on the first B-frame seen.  Until
	 * then, deliver I/P frames directly without the window.
	 */
	if (pic_type != MPEG4_PIC_TYPE_I && pic_type != MPEG4_PIC_TYPE_P) {
		/* B-frame: display immediately, enable sliding window */
		mpeg4->has_b_frames = true;
		amvdec_dst_buf_done_idx(sess, buffer_index,
					(u32)-1, V4L2_FIELD_NONE);
	} else if (!mpeg4->has_b_frames) {
		/* I/P-only stream: deliver directly, no window */
		amvdec_dst_buf_done_idx(sess, buffer_index,
					(u32)-1, V4L2_FIELD_NONE);
	} else {
		/* I/P frame with B-frames present: use sliding window */
		display_idx = mpeg4->ref_buf_idx[0];
		mpeg4->ref_buf_idx[0] = mpeg4->ref_buf_idx[1];
		mpeg4->ref_buf_idx[1] = buffer_index;
		if (mpeg4->ref_count < 2)
			mpeg4->ref_count++;

		if (display_idx >= 0)
			amvdec_dst_buf_done_idx(sess, display_idx,
						(u32)-1, V4L2_FIELD_NONE);
	}

	/*
	 * SM1 multi-instance: restart firmware for next frame.
	 * The firmware has halted after this frame; we must restart it.
	 * Single-instance firmware (GXL/G12A/G12B) continues running.
	 */
	if (core->platform->revision >= VDEC_REVISION_SM1)
		sm1_mpeg4_restart(sess, -1);
	else
		amvdec_write_dos(core, MREG_BUFFEROUT, 0);

	return IRQ_HANDLED;
}

/*
 * mpeg4_drain_deliver() - Apply sliding window logic during drain polling.
 *
 * Same reference frame reordering as the threaded ISR, but used during
 * drain when we are actively polling BUFFEROUT instead of receiving IRQs.
 */
static void mpeg4_drain_deliver(struct amvdec_session *sess,
				struct codec_mpeg4 *mpeg4,
				int buffer_index, int pic_type)
{
	int display_idx;

	/* Mirror the ISR's has_b_frames guard -- see threaded_isr comment */
	if (pic_type != MPEG4_PIC_TYPE_I && pic_type != MPEG4_PIC_TYPE_P) {
		mpeg4->has_b_frames = true;
		amvdec_dst_buf_done_idx(sess, buffer_index,
					(u32)-1, V4L2_FIELD_NONE);
	} else if (!mpeg4->has_b_frames) {
		amvdec_dst_buf_done_idx(sess, buffer_index,
					(u32)-1, V4L2_FIELD_NONE);
	} else {
		display_idx = mpeg4->ref_buf_idx[0];
		mpeg4->ref_buf_idx[0] = mpeg4->ref_buf_idx[1];
		mpeg4->ref_buf_idx[1] = buffer_index;
		if (mpeg4->ref_count < 2)
			mpeg4->ref_count++;

		if (display_idx >= 0)
			amvdec_dst_buf_done_idx(sess, display_idx,
						(u32)-1, V4L2_FIELD_NONE);
	}
}

/*
 * mpeg4_deliver_ref() - Deliver a reference frame, waiting for its capture buffer.
 *
 * At EOS, the sliding window still holds 1-2 reference frames.  Their
 * capture buffers may be in userspace (ffmpeg hasn't re-queued them yet).
 * Unlike the ISR path which defers such frames, this function actively
 * waits for the buffer to become available and delivers it synchronously.
 *
 * This guarantees the frame is in the V4L2 done queue BEFORE
 * vdec_drain_work() sends the LAST sentinel.  The deferral path has a
 * race: the recycle thread and vdec_drain_work compete, and LAST can
 * be sent before the deferred frame is delivered (observed: 29/30).
 */
static void mpeg4_deliver_ref(struct amvdec_session *sess, int buf_idx)
{
	struct vb2_v4l2_buffer *vbuf;
	unsigned long deadline = jiffies + msecs_to_jiffies(2000);

	do {
		vbuf = v4l2_m2m_dst_buf_remove_by_idx(sess->m2m_ctx,
				sess->fw_idx_to_vb2_idx[buf_idx]);
		if (vbuf) {
			amvdec_dst_buf_done(sess, vbuf, V4L2_FIELD_NONE);
			return;
		}
		usleep_range(1000, 2000);
	} while (time_is_after_jiffies(deadline));

	dev_warn(sess->core->dev,
		 "MPEG4 flush_refs: timeout waiting for buf %d\n", buf_idx);
}

/*
 * mpeg4_flush_refs() - Flush remaining reference frames at end-of-stream.
 *
 * After all firmware frames have been collected, the sliding window may
 * still hold 1-2 reference frames that were never displayed.  Deliver
 * them in order: refs[0] first (older), then refs[1] (newer).
 *
 * Each frame is delivered synchronously by waiting for its capture buffer,
 * avoiding the deferred-frame race that causes frame loss at EOS.
 */
static void mpeg4_flush_refs(struct amvdec_session *sess,
			     struct codec_mpeg4 *mpeg4)
{
	if (mpeg4->ref_buf_idx[0] >= 0) {
		mpeg4_deliver_ref(sess, mpeg4->ref_buf_idx[0]);
		mpeg4->ref_buf_idx[0] = -1;
	}
	if (mpeg4->ref_buf_idx[1] >= 0) {
		mpeg4_deliver_ref(sess, mpeg4->ref_buf_idx[1]);
		mpeg4->ref_buf_idx[1] = -1;
	}
	mpeg4->ref_count = 0;
}

/*
 * codec_mpeg4_drain() - Actively drain buffered frames from the firmware.
 *
 * The MPEG-4 firmware holds reference frames internally. At end-of-stream,
 * a visual_object_sequence_end_code (0x000001B1) may trigger the firmware
 * to flush, but it does not reliably deliver all remaining frames via ISR.
 * This function actively polls MREG_BUFFEROUT to collect any frames the
 * firmware has decoded but not yet signaled.
 *
 * After polling, it sends the EOS sequence, polls again, and finally
 * flushes any reference frames remaining in the sliding window.
 */
static void codec_mpeg4_drain(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_mpeg4 *mpeg4 = sess->priv;
	u32 reg;
	int buffer_index;
	int pic_type;
	u32 eos_len;
	const u8 *eos_data;
	int attempts = 0;

	/*
	 * Disable the MBOX1 interrupt during drain to prevent the
	 * threaded ISR from racing with our BUFFEROUT polling.
	 *
	 * Without this, both the ISR and drain loop can read the same
	 * BUFFEROUT value simultaneously, causing the sliding window
	 * to be updated twice for the same frame.  This corrupts the
	 * ref_buf_idx[] state, leading to duplicate frame deliveries
	 * (31/30) or frames arriving after the LAST sentinel (29/30).
	 *
	 * The firmware continues running and decoding while the
	 * interrupt is masked -- it writes BUFFEROUT and spins until
	 * the driver clears it via our poll loop.
	 *
	 * Re-enabled after flush_refs at the end of this function.
	 */
	amvdec_write_dos(core, ASSIST_MBOX1_MASK, 0);

	/*
	 * Pre-EOS active drain: poll BUFFEROUT to pick up any frames
	 * the firmware decoded but the ISR hasn't serviced yet.
	 * Recycle each canvas immediately to avoid starvation.
	 * 200 iterations * 10ms = 2s timeout.
	 */
	while (atomic_read(&sess->esparser_queued_bufs) > 0 &&
	       attempts++ < 200) {
		bool has_frame;

		reg = amvdec_read_dos(core, MREG_BUFFEROUT);

		if (core->platform->revision >= VDEC_REVISION_SM1)
			has_frame = (reg > 0x10000);
		else
			has_frame = (reg != 0);

		if (has_frame) {
			if (reg == MPEG4_BUFFEROUT_TIMEOUT) {
				if (core->platform->revision >= VDEC_REVISION_SM1) {
					sm1_mpeg4_save_context(core, mpeg4);
					sm1_mpeg4_restart(sess, -1);
				} else {
					amvdec_write_dos(core, MREG_BUFFEROUT, 0);
				}
				continue;
			}
			pic_type = (reg >> MPEG4_BUFFEROUT_PIC_TYPE_SHIFT) &
				   MPEG4_BUFFEROUT_PIC_TYPE_MASK;
			buffer_index = mpeg4_canvas_to_buf_idx(core, mpeg4);
			if (buffer_index < 0) {
				if (core->platform->revision >= VDEC_REVISION_SM1) {
					sm1_mpeg4_save_context(core, mpeg4);
					sm1_mpeg4_restart(sess, -1);
				} else {
					amvdec_write_dos(core, MREG_BUFFEROUT, 0);
				}
				continue;
			}
			if (core->platform->revision >= VDEC_REVISION_SM1)
				sm1_mpeg4_save_context(core, mpeg4);
			mpeg4_drain_deliver(sess, mpeg4, buffer_index,
					    pic_type);
			if (core->platform->revision >= VDEC_REVISION_SM1) {
				sm1_mpeg4_restart(sess, -1);
			} else {
				amvdec_write_dos(core, MREG_BUFFEROUT, 0);
			}
			if (codec_mpeg4_can_recycle(core))
				codec_mpeg4_recycle(core, buffer_index);
			continue;
		}
		usleep_range(10000, 11000);
	}

	if (atomic_read(&sess->esparser_queued_bufs) <= 0)
		goto flush_refs;

	/*
	 * Send visual_object_sequence_end_code to trigger firmware flush
	 * of any internally buffered reference frames.
	 */
	eos_data = codec_mpeg4_eos_sequence(&eos_len);
	esparser_queue_eos(core, eos_data, eos_len);
	usleep_range(5000, 6000);

	/*
	 * SM1: restart firmware so it processes the EOS data.
	 */
	if (core->platform->revision >= VDEC_REVISION_SM1)
		sm1_mpeg4_restart(sess, -1);

	/*
	 * Poll BUFFEROUT for EOS-flushed frames.
	 * Max 200 iterations * 10ms = 2s timeout.
	 */
	attempts = 0;
	while (attempts++ < 200) {
		bool has_frame;

		if (atomic_read(&sess->esparser_queued_bufs) <= 0)
			break;

		reg = amvdec_read_dos(core, MREG_BUFFEROUT);

		if (core->platform->revision >= VDEC_REVISION_SM1)
			has_frame = (reg > 0x10000);
		else
			has_frame = (reg != 0);

		if (!has_frame) {
			usleep_range(10000, 11000);
			continue;
		}

		if (reg == MPEG4_BUFFEROUT_TIMEOUT) {
			if (core->platform->revision >= VDEC_REVISION_SM1) {
				sm1_mpeg4_save_context(core, mpeg4);
				sm1_mpeg4_restart(sess, -1);
			} else {
				amvdec_write_dos(core, MREG_BUFFEROUT, 0);
			}
			continue;
		}

		pic_type = (reg >> 3) & 7;
		buffer_index = mpeg4_canvas_to_buf_idx(core, mpeg4);
		if (buffer_index < 0) {
			if (core->platform->revision >= VDEC_REVISION_SM1) {
				sm1_mpeg4_save_context(core, mpeg4);
				sm1_mpeg4_restart(sess, -1);
			} else {
				amvdec_write_dos(core, MREG_BUFFEROUT, 0);
			}
			continue;
		}
		if (core->platform->revision >= VDEC_REVISION_SM1)
			sm1_mpeg4_save_context(core, mpeg4);
		mpeg4_drain_deliver(sess, mpeg4, buffer_index, pic_type);
		if (core->platform->revision >= VDEC_REVISION_SM1)
			sm1_mpeg4_restart(sess, -1);
		else
			amvdec_write_dos(core, MREG_BUFFEROUT, 0);

		if (codec_mpeg4_can_recycle(core))
			codec_mpeg4_recycle(core, buffer_index);
	}

flush_refs:
	/*
	 * Flush remaining reference frames from the sliding window.
	 * At EOS, any frames still held as references must be delivered
	 * to userspace.  This fixes the 2-frame drain loss at EOS.
	 */
	mpeg4_flush_refs(sess, mpeg4);

	/*
	 * The MPEG4 firmware may not flush all frames on EOS (similar to
	 * the MPEG12 firmware limitation). Force the queued counter to
	 * zero so the post-drain wait doesn't stall.
	 */
	if (atomic_read(&sess->esparser_queued_bufs) > 0) {
		dev_dbg(core->dev,
			"MPEG4 drain: forcing queued count from %d to 0\n",
			atomic_read(&sess->esparser_queued_bufs));
		atomic_set(&sess->esparser_queued_bufs, 0);
	}

	/* Re-enable MBOX1 interrupt after drain is complete. */
	amvdec_write_dos(core, ASSIST_MBOX1_MASK, 1);
}

/*
 * Clear VLD_STATUS_CTRL after firmware start.
 *
 * The MPEG4 firmware main loop at AMRISC 0x06b1 does
 *   bandi r2, 1, +5  ->  06b6
 * i.e. "if VLD bit 0 (busy) is set, skip frame processing".  In the
 * V4L2 M2M flow, the esparser writes stream data to the VIFIFO BEFORE
 * MPSR=1, so the VLD hardware starts processing and leaves bit 0 set.
 * The firmware's polling loop then sees VLD busy on every iteration
 * and never enters the frame processing path at 0x06b9.
 *
 * This is the same root cause as KI-AVS (commit 26294cb30cec fixed
 * AVS by NOPing the busy check in the firmware binary; ff3944582428
 * switched to clearing VLD_STATUS_CTRL in post_start instead).  Use
 * the same post_start approach here so no firmware patch is needed.
 *
 * Clear must run AFTER MPSR=1 (firmware running).  post_start is
 * invoked after vdec_1_start has written MPSR=1 and enabled MBOX1.
 */
static void codec_mpeg4_post_start(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	amvdec_write_dos(core, VLD_STATUS_CTRL, 0);
}

struct amvdec_codec_ops codec_mpeg4_ops = {
	.start = codec_mpeg4_start,
	.post_start = codec_mpeg4_post_start,
	.stop = codec_mpeg4_stop,
	.resume = codec_mpeg4_resume,
	.isr = codec_mpeg4_isr,
	.threaded_isr = codec_mpeg4_threaded_isr,
	.can_recycle = codec_mpeg4_can_recycle,
	.recycle = codec_mpeg4_recycle,
	.drain = codec_mpeg4_drain,
};

static int __init codec_mpeg4_mod_init(void)
{
	int ret;

	ret = amvdec_register_codec(V4L2_PIX_FMT_MPEG4, &codec_mpeg4_ops);
	if (ret)
		return ret;

	ret = amvdec_register_codec(V4L2_PIX_FMT_H263, &codec_mpeg4_ops);
	if (ret) {
		amvdec_unregister_codec(V4L2_PIX_FMT_MPEG4, &codec_mpeg4_ops);
		return ret;
	}

	return 0;
}

static void __exit codec_mpeg4_mod_exit(void)
{
	amvdec_unregister_codec(V4L2_PIX_FMT_H263, &codec_mpeg4_ops);
	amvdec_unregister_codec(V4L2_PIX_FMT_MPEG4, &codec_mpeg4_ops);
}

module_init(codec_mpeg4_mod_init);
module_exit(codec_mpeg4_mod_exit);
MODULE_DESCRIPTION("Meson VDEC MPEG-4/H.263 codec");
MODULE_LICENSE("GPL");
