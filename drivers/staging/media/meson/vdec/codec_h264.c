// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#include <linux/module.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include <media/v4l2-ctrls.h>

/*
 * Module parameter: I-frame-only decode mode (overrides V4L2 control).
 * 0 = normal decode (default), 1 = I-frames only.
 * BSP reference: vmh264.c i_only_flag module_param.
 */
static unsigned int i_only_flag;
module_param(i_only_flag, uint, 0644);
MODULE_PARM_DESC(i_only_flag, "H.264 I-frame-only decode: 0=normal, 1=I-only");

/*
 * H.264 MMU mode: power on HEVC SAO pipeline for compressed references.
 * 0 = disabled (default), 1 = enable HEVC SAO register init (G12A+ only).
 * Phase 1: register init only, no decode path change.
 */
static bool h264_mmu_enable;
module_param(h264_mmu_enable, bool, 0644);
MODULE_PARM_DESC(h264_mmu_enable, "H.264 MMU: 0=off (default), 1=HEVC SAO init (G12A+)");

#include "vdec_helpers.h"
#include "vdec_debugfs.h"
#include "dos_regs.h"
#include "hevc_regs.h"
#include "codec_h264.h"
#include "vdec_hevc.h"

#define SIZE_EXT_FW	(20 * SZ_1K)
#define SIZE_WORKSPACE	0x1ee000
#define SIZE_SEI	(8 * SZ_1K)

/*
 * POWER_CTL_VLD bits (VLD = Variable Length Decoder power control).
 * BSP vmh264.c sets (0 << 10) | (1 << 9) | (1 << 6) at init and
 * chunk-feed time.  Bit 10 is documented as disabled (always 0).
 */
#define POWER_CTL_VLD_MEM_GATE		BIT(9)	/* VLD memory clock gate */
#define POWER_CTL_VLD_PSCALE_CLK	BIT(6)	/* PSCALE clock enable */

/*
 * AV_SCRATCH_F error-recovery control bits, passed to firmware.
 * BSP vmh264.c masks with 0xffffffc3 (clears bits [5:2]), then sets:
 *   bit 4: error_recovery_mode (1 = skip-to-IDR on error)
 *   bit 7: ucode IP-only mode flag (set for IP_ONLY_PARAM ucode)
 *   bit 6: IP-only decode enable (BSP conditionally sets/clears)
 */
#define ERR_RECOVERY_MODE		BIT(4)
#define ERR_RECOVERY_IP_ONLY		BIT(7)

/*
 * AV_SCRATCH_F bit 6: firmware I-frame-only decode mode.
 * When set, the ucode skips P/B frame decoding entirely.
 * BSP vmh264.c:10004-10007 (conditionally set for UCODE_IP_ONLY_PARAM).
 */
#define I_ONLY_FW_MODE			BIT(6)

/*
 * MDEC_PIC_DC_THRESH threshold value.  Set identically in BSP for
 * Meson8+ SoCs (vmh264.c:9946,10016).  The four bytes pack DC prediction
 * thresholds for the picture-layer decoder:
 *   [31:24] = 0x40  intra luma threshold
 *   [23:16] = 0x40  intra chroma threshold
 *   [15:8]  = 0x38  inter threshold
 *   [7:0]   = 0xaa  blend/mix threshold
 * These values are hardware tuning constants, not user-configurable.
 */
#define MDEC_PIC_DC_THRESH_DEFAULT	0x404038aa

/*
 * MDEC_PIC_DC_CTRL reset pulse mask.  BSP vmh264.c:9929-9930 writes
 * 0xbf << 24 then immediately clears it, resetting the DC control
 * logic in bits [30:24].  0xBF = all bits in [30:24] except bit 30.
 */
#define MDEC_PIC_DC_CTRL_RESET_MASK	(0xbf << 24)

/*
 * IQIDCT_CONTROL initial value.  BSP vmh264.c:10025-10027 writes 0x200
 * when CSD data is not yet available (GXL+).  This sets bit 9, which
 * enables the default quantization matrix path.
 */
#define IQIDCT_CONTROL_INIT		0x200

/*
 * IQIDCT_CONTROL bit 16: disable DCAC DDR read cache.
 * Required for narrow frames (width <= 256px) to avoid zero-output
 * on G12A/SM1.  BSP vmh264.c:4685-4687.
 */
#define IQIDCT_DCAC_DDR_CACHE_DIS	BIT(16)

/*
 * DCAC_DDR_BYTE64_CTL low nibble value for narrow-frame workaround.
 * BSP vmh264.c:4688 sets low nibble to 0xa when disabling DDR cache
 * for sub-256px-wide streams on G12A+.
 */
#define DCAC_DDR_BYTE64_LOW_MASK	0xf
#define DCAC_DDR_BYTE64_NARROW_VAL	0xa

/*
 * ISR status field: number of decoded frames ready, in bits [15:8].
 */
#define NUM_FRAMES_BIT		8
#define NUM_FRAMES_MASK		GENMASK(7, 0)

/*
 * AV_SCRATCH_2 seq_info field layout (BSP vmh264.c:1153-1155,6588-6590):
 *   bit 15:    frame_mbs_only_flag
 *   bit 13-14: chroma_format_idc (1=4:2:0, 2=4:2:2, 3=4:4:4)
 */
#define SEQ_INFO_CHROMA_FMT_BIT	13
#define SEQ_INFO_CHROMA_FMT_MASK	GENMASK(1, 0)

/*
 * H.264 profile_idc values.  The firmware's 8-bit 4:2:0 pipeline only
 * supports Baseline (66), Main (77), and High (100).  High10 (110),
 * High 4:2:2 (122), and High 4:4:4 Predictive (244) require 10-bit
 * or non-4:2:0 chroma support that the hardware lacks.
 *
 * High10 produces garbage (~19 dB PSNR) because 10-bit coefficients
 * overflow the 8-bit IQIDCT pipeline.  High 4:2:2 and High 4:4:4
 * silently produce 0 frames.
 */
/*
 * H.264 NAL unit types relevant for I-only decode filtering.
 * When i_only is enabled, only IDR (type 5) and non-VCL NALs are fed
 * to the firmware; non-IDR VCL NALs (type 1) are skipped.
 * This makes the firmware decode only IDR I-frames.
 * See ITU-T H.264 Table 7-1.
 */
#define NAL_TYPE_SLICE			1	/* non-IDR coded slice */
#define NAL_TYPE_IDR			5	/* IDR coded slice */

#define H264_PROFILE_HIGH10		110
#define H264_PROFILE_HIGH_422		122
#define H264_PROFILE_HIGH_444_PRED	244

/*
 * AV_SCRATCH_6 crop info layout (BSP vmh264.c:1166-1168):
 *   [31:16] = (left << 8 | right) << 1
 *   [15:0]  = (top << 8 | bottom) << (2 - frame_mbs_only_flag)
 * The upstream driver only uses right (bits [23:16]) and bottom ([7:0]).
 */
#define CROP_BOTTOM_MASK	GENMASK(7, 0)
#define CROP_RIGHT_BIT		16
#define CROP_RIGHT_MASK		GENMASK(7, 0)

/*
 * AV_SCRATCH_0 resume command word layout:
 *   [31:24] = max_reference_size
 *   [23:16] = DPB size (num_dst_bufs)
 *   [15:8]  = max_reference_size - 1
 * BSP vmh264.c:9962-9964.
 */
#define SCRATCH0_MAX_REF_BIT	24
#define SCRATCH0_DPB_SIZE_BIT	16
#define SCRATCH0_REF_MINUS1_BIT	8

/*
 * Offset added by the firmware which must be substracted
 * from the workspace phyaddr
 */
#define WORKSPACE_BUF_OFFSET	0x1000000

/* ISR status */
#define CMD_MASK		GENMASK(7, 0)
#define CMD_SRC_CHANGE		1
#define CMD_FRAMES_READY	2
#define CMD_FATAL_ERROR		6
#define CMD_BAD_WIDTH		7
#define CMD_BAD_HEIGHT		8

#define SEI_DATA_READY	BIT(15)

/* Picture type */
#define PIC_TOP_BOT	5
#define PIC_BOT_TOP	6

/* Size of Motion Vector per macroblock */
#define MB_MV_SIZE	96

/* Frame status data */
#define PIC_STRUCT_BIT	5
#define PIC_STRUCT_MASK	GENMASK(2, 0)
#define BUF_IDX_MASK	GENMASK(4, 0)
#define ERROR_FLAG	BIT(9)
#define OFFSET_BIT	16
#define OFFSET_MASK	GENMASK(15, 0)

/* Bitstream parsed data */
#define MB_TOTAL_BIT	8
#define MB_TOTAL_MASK	GENMASK(15, 0)
#define MB_WIDTH_MASK	GENMASK(7, 0)
#define MAX_REF_BIT	24
#define MAX_REF_MASK	GENMASK(6, 0)
#define AR_IDC_BIT	16
#define AR_IDC_MASK	GENMASK(7, 0)
#define AR_PRESENT_FLAG	BIT(0)
#define AR_EXTEND	0xff

/*
 * Buffer to send to the ESPARSER to signal End Of Stream for H.264.
 * This is a 16x16 encoded picture that will trigger drain firmware-side.
 * There is no known alternative.
 */
static const u8 eos_sequence[SZ_4K] = {
	0x00, 0x00, 0x00, 0x01, 0x06, 0x05, 0xff, 0xe4, 0xdc, 0x45, 0xe9, 0xbd,
	0xe6, 0xd9, 0x48, 0xb7,	0x96, 0x2c, 0xd8, 0x20, 0xd9, 0x23, 0xee, 0xef,
	0x78, 0x32, 0x36, 0x34, 0x20, 0x2d, 0x20, 0x63,	0x6f, 0x72, 0x65, 0x20,
	0x36, 0x37, 0x20, 0x72, 0x31, 0x31, 0x33, 0x30, 0x20, 0x38, 0x34, 0x37,
	0x35, 0x39, 0x37, 0x37, 0x20, 0x2d, 0x20, 0x48, 0x2e, 0x32, 0x36, 0x34,
	0x2f, 0x4d, 0x50, 0x45,	0x47, 0x2d, 0x34, 0x20, 0x41, 0x56, 0x43, 0x20,
	0x63, 0x6f, 0x64, 0x65, 0x63, 0x20, 0x2d, 0x20,	0x43, 0x6f, 0x70, 0x79,
	0x6c, 0x65, 0x66, 0x74, 0x20, 0x32, 0x30, 0x30, 0x33, 0x2d, 0x32, 0x30,
	0x30, 0x39, 0x20, 0x2d, 0x20, 0x68, 0x74, 0x74, 0x70, 0x3a, 0x2f, 0x2f,
	0x77, 0x77, 0x77, 0x2e,	0x76, 0x69, 0x64, 0x65, 0x6f, 0x6c, 0x61, 0x6e,
	0x2e, 0x6f, 0x72, 0x67, 0x2f, 0x78, 0x32, 0x36,	0x34, 0x2e, 0x68, 0x74,
	0x6d, 0x6c, 0x20, 0x2d, 0x20, 0x6f, 0x70, 0x74, 0x69, 0x6f, 0x6e, 0x73,
	0x3a, 0x20, 0x63, 0x61, 0x62, 0x61, 0x63, 0x3d, 0x31, 0x20, 0x72, 0x65,
	0x66, 0x3d, 0x31, 0x20,	0x64, 0x65, 0x62, 0x6c, 0x6f, 0x63, 0x6b, 0x3d,
	0x31, 0x3a, 0x30, 0x3a, 0x30, 0x20, 0x61, 0x6e,	0x61, 0x6c, 0x79, 0x73,
	0x65, 0x3d, 0x30, 0x78, 0x31, 0x3a, 0x30, 0x78, 0x31, 0x31, 0x31, 0x20,
	0x6d, 0x65, 0x3d, 0x68, 0x65, 0x78, 0x20, 0x73, 0x75, 0x62, 0x6d, 0x65,
	0x3d, 0x36, 0x20, 0x70,	0x73, 0x79, 0x5f, 0x72, 0x64, 0x3d, 0x31, 0x2e,
	0x30, 0x3a, 0x30, 0x2e, 0x30, 0x20, 0x6d, 0x69,	0x78, 0x65, 0x64, 0x5f,
	0x72, 0x65, 0x66, 0x3d, 0x30, 0x20, 0x6d, 0x65, 0x5f, 0x72, 0x61, 0x6e,
	0x67, 0x65, 0x3d, 0x31, 0x36, 0x20, 0x63, 0x68, 0x72, 0x6f, 0x6d, 0x61,
	0x5f, 0x6d, 0x65, 0x3d,	0x31, 0x20, 0x74, 0x72, 0x65, 0x6c, 0x6c, 0x69,
	0x73, 0x3d, 0x30, 0x20, 0x38, 0x78, 0x38, 0x64,	0x63, 0x74, 0x3d, 0x30,
	0x20, 0x63, 0x71, 0x6d, 0x3d, 0x30, 0x20, 0x64, 0x65, 0x61, 0x64, 0x7a,
	0x6f, 0x6e, 0x65, 0x3d, 0x32, 0x31, 0x2c, 0x31, 0x31, 0x20, 0x63, 0x68,
	0x72, 0x6f, 0x6d, 0x61,	0x5f, 0x71, 0x70, 0x5f, 0x6f, 0x66, 0x66, 0x73,
	0x65, 0x74, 0x3d, 0x2d, 0x32, 0x20, 0x74, 0x68,	0x72, 0x65, 0x61, 0x64,
	0x73, 0x3d, 0x31, 0x20, 0x6e, 0x72, 0x3d, 0x30, 0x20, 0x64, 0x65, 0x63,
	0x69, 0x6d, 0x61, 0x74, 0x65, 0x3d, 0x31, 0x20, 0x6d, 0x62, 0x61, 0x66,
	0x66, 0x3d, 0x30, 0x20,	0x62, 0x66, 0x72, 0x61, 0x6d, 0x65, 0x73, 0x3d,
	0x30, 0x20, 0x6b, 0x65, 0x79, 0x69, 0x6e, 0x74,	0x3d, 0x32, 0x35, 0x30,
	0x20, 0x6b, 0x65, 0x79, 0x69, 0x6e, 0x74, 0x5f, 0x6d, 0x69, 0x6e, 0x3d,
	0x32, 0x35, 0x20, 0x73, 0x63, 0x65, 0x6e, 0x65, 0x63, 0x75, 0x74, 0x3d,
	0x34, 0x30, 0x20, 0x72,	0x63, 0x3d, 0x61, 0x62, 0x72, 0x20, 0x62, 0x69,
	0x74, 0x72, 0x61, 0x74, 0x65, 0x3d, 0x31, 0x30,	0x20, 0x72, 0x61, 0x74,
	0x65, 0x74, 0x6f, 0x6c, 0x3d, 0x31, 0x2e, 0x30, 0x20, 0x71, 0x63, 0x6f,
	0x6d, 0x70, 0x3d, 0x30, 0x2e, 0x36, 0x30, 0x20, 0x71, 0x70, 0x6d, 0x69,
	0x6e, 0x3d, 0x31, 0x30,	0x20, 0x71, 0x70, 0x6d, 0x61, 0x78, 0x3d, 0x35,
	0x31, 0x20, 0x71, 0x70, 0x73, 0x74, 0x65, 0x70,	0x3d, 0x34, 0x20, 0x69,
	0x70, 0x5f, 0x72, 0x61, 0x74, 0x69, 0x6f, 0x3d, 0x31, 0x2e, 0x34, 0x30,
	0x20, 0x61, 0x71, 0x3d, 0x31, 0x3a, 0x31, 0x2e, 0x30, 0x30, 0x00, 0x80,
	0x00, 0x00, 0x00, 0x01,	0x67, 0x4d, 0x40, 0x0a, 0x9a, 0x74, 0xf4, 0x20,
	0x00, 0x00, 0x03, 0x00, 0x20, 0x00, 0x00, 0x06,	0x51, 0xe2, 0x44, 0xd4,
	0x00, 0x00, 0x00, 0x01, 0x68, 0xee, 0x32, 0xc8, 0x00, 0x00, 0x00, 0x01,
	0x65, 0x88, 0x80, 0x20, 0x00, 0x08, 0x7f, 0xea, 0x6a, 0xe2, 0x99, 0xb6,
	0x57, 0xae, 0x49, 0x30,	0xf5, 0xfe, 0x5e, 0x46, 0x0b, 0x72, 0x44, 0xc4,
	0xe1, 0xfc, 0x62, 0xda, 0xf1, 0xfb, 0xa2, 0xdb,	0xd6, 0xbe, 0x5c, 0xd7,
	0x24, 0xa3, 0xf5, 0xb9, 0x2f, 0x57, 0x16, 0x49, 0x75, 0x47, 0x77, 0x09,
	0x5c, 0xa1, 0xb4, 0xc3, 0x4f, 0x60, 0x2b, 0xb0, 0x0c, 0xc8, 0xd6, 0x66,
	0xba, 0x9b, 0x82, 0x29,	0x33, 0x92, 0x26, 0x99, 0x31, 0x1c, 0x7f, 0x9b,
	0x00, 0x00, 0x01, 0x0ff,
};

static const u8 *codec_h264_eos_sequence(u32 *len)
{
	*len = ARRAY_SIZE(eos_sequence);
	return eos_sequence;
}

struct codec_h264 {
	/* H.264 decoder requires an extended firmware */
	void      *ext_fw_vaddr;
	dma_addr_t ext_fw_paddr;

	/* Buffer for the H.264 Workspace */
	void      *workspace_vaddr;
	dma_addr_t workspace_paddr;

	/* Buffer for the H.264 references MV */
	void      *ref_vaddr;
	dma_addr_t ref_paddr;
	u32	   ref_size;

	/* Buffer for parsed SEI data */
	void      *sei_vaddr;
	dma_addr_t sei_paddr;

	u32 mb_width;
	u32 mb_height;
	u32 max_refs;

	/* Decode stall watchdog (BSP vmh264.c:check_timer_func).
	 * Fires every 50ms; checks MBY_MBX register for macroblock
	 * progress.  If no progress for decode_timeout_ms, aborts.
	 */
	struct timer_list check_timer;
	struct amvdec_session *sess;	/* back-pointer for timer callback */
	unsigned long start_process_time;	/* jiffies when decode started */
	u32 last_mby_mbx;			/* previous MBY_MBX value */
	u32 timeout_count;			/* consecutive stall checks */
	/* I-frame-only decode mode (from V4L2_CID_MESON_VDEC_I_ONLY) */
	bool i_only;
	/* True when vdec_hevc_power_on() succeeded for H.264 MMU mode */
	bool hevc_powered;

	/*
	 * SPS-parsed dimensions, extracted in pre_queue from the raw
	 * bitstream before the firmware sees it.  Used as fallback when
	 * the firmware's 8-bit mb_width field overflows at exactly 4096px
	 * (256 MBs -> wraps to 0).
	 */
	u32 sps_mb_width;	/* pic_width_in_mbs_minus1 + 1 */
	u32 sps_mb_height;	/* (pic_height_in_map_units_minus1 + 1) * 2 if !frame_mbs_only */
	u8  sps_level_idc;	/* level_idc from SPS; 0 if not yet parsed */
};

/*
 * Decode stall watchdog interval (BSP vmh264.c CHECK_INTERVAL).
 * The timer fires this often to sample MBY_MBX for progress.
 */
#define CHECK_INTERVAL		msecs_to_jiffies(50)
#define TIMEOUT_STALL_COUNT	2	/* consecutive stalls before abort */

static void codec_h264_start_watchdog(struct codec_h264 *h264)
{
	h264->start_process_time = jiffies;
	h264->timeout_count = TIMEOUT_STALL_COUNT;
	h264->last_mby_mbx = 0;
}

/*
 * check_timer callback -- mirrors BSP vmh264.c:9675 check_timer_func.
 *
 * Reads MBY_MBX (macroblock decode position register) to detect
 * whether the firmware is making progress.  If the position hasn't
 * changed for decode_timeout_ms, declare a stall and abort.
 *
 * Unlike the BSP which also checks DPB_STATUS_REG to distinguish
 * decode vs. parse stalls, we keep it simple: any stall beyond
 * the timeout triggers abort.  The upstream driver's existing
 * vdec_wait_inactive handles shutdown-path timeouts separately.
 */
/*
 * Check if MBY_MBX indicates the last macroblock of the frame.
 * MBY_MBX format: [15:8] = mb_x (0-indexed), [7:0] = mb_y (0-indexed).
 * Returns true if the firmware has reached the final macroblock but
 * AV_SCRATCH_0 is still 0 (no completion command written).
 *
 * This detects a firmware race where decode completes but the
 * completion signal (CMD_FRAMES_READY) is lost -- same class as
 * the MPEG12 BUFFEROUT race patched in vdec_1.c (KI-091).
 */
static bool codec_h264_frame_done_no_signal(struct codec_h264 *h264,
					    struct amvdec_core *core,
					    u32 mby_mbx)
{
	u32 last_mb_x, last_mb_y, expected;

	if (!h264->mb_width || !h264->mb_height)
		return false;

	last_mb_x = h264->mb_width - 1;
	last_mb_y = h264->mb_height - 1;
	expected = (last_mb_x << 8) | last_mb_y;

	if ((mby_mbx & 0xffff) != expected)
		return false;

	/* Last MB reached -- check if firmware forgot to signal */
	return amvdec_read_dos(core, AV_SCRATCH_0) == 0;
}

static void codec_h264_check_timer(struct timer_list *timer)
{
	struct codec_h264 *h264 = from_timer(h264, timer, check_timer);
	struct amvdec_session *sess = h264->sess;
	struct amvdec_core *core = sess->core;
	u32 timeout_ms = READ_ONCE(sess->decode_timeout_ms);
	u32 mby_mbx;

	if (!timeout_ms || !h264->start_process_time)
		goto rearm;

	/* Check elapsed time since decode started */
	if (jiffies_to_msecs(jiffies - h264->start_process_time) < timeout_ms)
		goto rearm;

	/* Timeout window reached -- check for progress */
	mby_mbx = amvdec_read_dos(core, MBY_MBX);

	if (mby_mbx != h264->last_mby_mbx) {
		/* Progress detected, reset watchdog */
		h264->last_mby_mbx = mby_mbx;
		h264->start_process_time = jiffies;
		h264->timeout_count = TIMEOUT_STALL_COUNT;
		goto rearm;
	}

	/*
	 * Fast recovery for firmware completion race (KI-091):
	 * If MBY_MBX is at the last macroblock and AV_SCRATCH_0 is 0,
	 * the firmware finished decoding but never signaled completion.
	 * Reset the buffer manager immediately instead of waiting for
	 * the full stall timeout.  This converts ~200ms stalls to <50ms.
	 */
	if (codec_h264_frame_done_no_signal(h264, core, mby_mbx)) {
		dev_dbg(core->dev,
			"H.264 completion race: MBY_MBX=0x%04x at last MB (%ux%u), resetting (KI-091)\n",
			mby_mbx & 0xffff, h264->mb_width, h264->mb_height);
		amvdec_write_dos(core, AV_SCRATCH_0, 0);
		codec_h264_start_watchdog(h264);
		goto rearm;
	}

	/* No progress -- decrement stall counter */
	if (h264->timeout_count > 0)
		h264->timeout_count--;

	if (h264->timeout_count == 0) {
		vdec_dbg_capture_stall(core, mby_mbx);
		dev_err(core->dev,
			"H.264 decode stall: MBY_MBX=0x%08x stuck for %ums, aborting\n",
			mby_mbx, timeout_ms);
		/*
		 * error_proc_policy bit 14 (reset_on_timeout):
		 *   1 = trigger buffer manager reset on stall (BSP behaviour)
		 *   0 = abort session (current default)
		 * Reset path: clear AV_SCRATCH_0 to let firmware re-init.
		 */
		if (sess->error_proc_policy & BIT(14)) {
			dev_warn(core->dev,
				 "H.264 decode stall: resetting buffer manager (error_proc_policy bit 14)\n");
			amvdec_write_dos(core, AV_SCRATCH_0, 0);
			h264->timeout_count = TIMEOUT_STALL_COUNT;
			goto rearm;
		}
		amvdec_abort(sess);
		return; /* don't rearm */
	}

rearm:
	mod_timer(&h264->check_timer, jiffies + CHECK_INTERVAL);
}

static int codec_h264_can_recycle(struct amvdec_core *core)
{
	return !amvdec_read_dos(core, AV_SCRATCH_7) ||
	       !amvdec_read_dos(core, AV_SCRATCH_8);
}

static void codec_h264_recycle(struct amvdec_core *core, u32 buf_idx)
{
	/*
	 * Tell the firmware it can recycle this buffer.
	 * AV_SCRATCH_8 serves the same purpose.
	 */
	if (!amvdec_read_dos(core, AV_SCRATCH_7))
		amvdec_write_dos(core, AV_SCRATCH_7, buf_idx + 1);
	else
		amvdec_write_dos(core, AV_SCRATCH_8, buf_idx + 1);
}

/* BSP vmh264.c:1325 -- compressed body buffer size for H264_MMU mode */
static int compute_losless_comp_body_size(int width, int height)
{
	return 3264 * ((width + 63) >> 6) * ((height + 31) >> 5);
}

/* BSP vmh264.c:1346 -- MMU header table size for H264_MMU mode */
static int compute_losless_comp_header_size(int width, int height)
{
	return 128 * ((width + 63) >> 6) * ((height + 63) >> 6);
}

/*
 * codec_h264_hevc_sao_init - Initialize HEVC SAO/IPP/DBLK registers.
 *
 * BSP reference: vmh264.c hevc_mcr_sao_global_hw_init() lines 1660-1801.
 * Phase 1: register init only.  Does NOT set MDEC_PIC_DC_MUX_CTRL bit 31
 * (decode stays on standard canvas path).
 *
 * Called from codec_h264_src_change() after frame dimensions are known.
 */
static void codec_h264_hevc_sao_init(struct amvdec_session *sess,
				     u32 width, u32 height)
{
	struct amvdec_core *core = sess->core;
	u32 data32;
	/* Width rounded to 64-pixel boundary, in 16-pixel LCU units */
	u32 lcu_x_num = ((width + 63) >> 6) << 2;
	u32 lcu_y_num = (height + 15) >> 4;
	u32 mc_buffer_size_u_v = lcu_x_num * lcu_y_num * 16 * 16 / 2;
	u32 mc_buffer_size_u_v_h = (mc_buffer_size_u_v + 0xffff) >> 16;
	int body_size = compute_losless_comp_body_size(width, height);
	int header_size = compute_losless_comp_header_size(width, height);

	/* IPP software reset */
	amvdec_write_dos(core, HEVCD_IPP_TOP_CNTL, 0x1);
	amvdec_write_dos(core, HEVCD_IPP_TOP_CNTL, 0x0);

	/* LCU size = 16 (H.264 macroblocks) */
	amvdec_write_dos(core, HEVCD_IPP_TOP_LCUCONFIG, 16);

	/* Frame dimensions */
	amvdec_write_dos(core, HEVCD_IPP_TOP_FRMCONFIG,
			 (height & 0xffff) << 16 | (width & 0xffff));

	/* 8-bit luma + chroma */
	amvdec_write_dos(core, HEVCD_IPP_BITDEPTH_CONFIG, 0x0);

	/* Lossy decompress path */
	amvdec_write_dos(core, HEVCD_MPP_DECOMP_CTL1, BIT(4));
	amvdec_write_dos(core, HEVCD_MPP_DECOMP_CTL2, 0x0);

	/* AXIIF canvas_mode = 0 (linear, MMU path) */
	data32 = amvdec_read_dos(core, HEVCD_IPP_AXIIF_CONFIG);
	data32 &= ~0x30;
	amvdec_write_dos(core, HEVCD_IPP_AXIIF_CONFIG, data32);

	/* MCR cache thresholds */
	amvdec_write_dos(core, HEVCD_MPP_DECOMP_CTL3,
			 (0x80 << 20) | (0x80 << 10) | 0xff);

	/* Enable MCR + MCRCC */
	amvdec_write_dos(core, HEVCD_MPP_VDEC_MCR_CTL, 0x1 | (0x1 << 4));

	/*
	 * Phase 1: do NOT set MDEC_PIC_DC_MUX_CTRL bit 31.
	 * BSP writes: READ | (1 << 31) to route MDEC through HEVC MCR.
	 * Intentionally skipped -- decode stays on standard canvas path.
	 */

	/* IPP enable */
	amvdec_write_dos(core, HEVCD_IPP_TOP_CNTL, BIT(1));

	/* G12A+ deblock pipeline: CTU=16, frame dimensions */
	amvdec_write_dos(core, HEVC_DBLK_CFG1, 0x2);
	amvdec_write_dos(core, HEVC_DBLK_CFG2,
			 ((height & 0xffff) << 16) | (width & 0xffff));
	amvdec_write_dos(core, HEVC_DBLK_CFGB, 0x40405703);

	/* SAO LCU size log2 = 4 */
	data32 = amvdec_read_dos(core, HEVC_SAO_CTRL0);
	data32 &= ~0xf;
	data32 |= 0x4;
	amvdec_write_dos(core, HEVC_SAO_CTRL0, data32);

	/* SAO picture/tile size */
	amvdec_write_dos(core, HEVC_SAO_PIC_SIZE,
			 (height & 0xffff) << 16 | (width & 0xffff));
	amvdec_write_dos(core, HEVC_SAO_PIC_SIZE_LCU,
			 (lcu_x_num - 1) | ((lcu_y_num - 1) << 16));
	amvdec_write_dos(core, HEVC_SAO_TILE_SIZE_LCU,
			 lcu_x_num | (lcu_y_num << 16));

	/* SAO Y/C buffer length */
	amvdec_write_dos(core, HEVC_SAO_Y_LENGTH,
			 (mc_buffer_size_u_v_h << 16) << 1);
	amvdec_write_dos(core, HEVC_SAO_C_LENGTH,
			 mc_buffer_size_u_v_h << 16);

	/* SAO_CTRL1: endian + canvas_mode for MMU */
	data32 = amvdec_read_dos(core, HEVC_SAO_CTRL1);
	data32 &= ~0x3ff0;
	data32 |= 0xff0; /* big-endian per 64-bit */
	amvdec_write_dos(core, HEVC_SAO_CTRL1, data32);

	/* SAO_CTRL5: DW 1/4 scale + 8-bit SMEM */
	data32 = amvdec_read_dos(core, HEVC_SAO_CTRL5);
	data32 &= ~(0xff << 16);
	data32 |= (0xff << 16);
	data32 |= BIT(9);  /* 8-bit smem mode */
	amvdec_write_dos(core, HEVC_SAO_CTRL5, data32);

	/* CM body/header length */
	amvdec_write_dos(core, HEVC_CM_BODY_LENGTH, body_size);
	amvdec_write_dos(core, HEVC_CM_HEADER_OFFSET, 0);
	amvdec_write_dos(core, HEVC_CM_HEADER_LENGTH, header_size);

	/* SAO_CTRL9: compress mode */
	data32 = amvdec_read_dos(core, HEVC_SAO_CTRL9);
	data32 |= BIT(1);
	amvdec_write_dos(core, HEVC_SAO_CTRL9, data32);

	/* SAO_CTRL5: compress path enable */
	data32 = amvdec_read_dos(core, HEVC_SAO_CTRL5);
	data32 |= BIT(10);
	amvdec_write_dos(core, HEVC_SAO_CTRL5, data32);

	/* SAO_CTRL9: additional compress control */
	data32 = amvdec_read_dos(core, HEVC_SAO_CTRL9);
	data32 |= BIT(7);
	amvdec_write_dos(core, HEVC_SAO_CTRL9, data32);

	dev_dbg(core->dev,
		"H.264 MMU: SAO init %ux%u lcu=%ux%u body=%d hdr=%d\n",
		width, height, lcu_x_num, lcu_y_num, body_size, header_size);
}

static int codec_h264_start(struct amvdec_session *sess)
{
	u32 workspace_offset;
	struct amvdec_core *core = sess->core;
	struct codec_h264 *h264 = sess->priv;

	/* Allocate some memory for the H.264 decoder's state */
	h264->workspace_vaddr =
		dma_alloc_coherent(core->dev, SIZE_WORKSPACE,
				   &h264->workspace_paddr, GFP_KERNEL);
	if (!h264->workspace_vaddr)
		return -ENOMEM;

	/* Allocate some memory for the H.264 SEI dump */
	h264->sei_vaddr = dma_alloc_coherent(core->dev, SIZE_SEI,
					     &h264->sei_paddr, GFP_KERNEL);
	if (!h264->sei_vaddr)
		return -ENOMEM;

	/*
	 * H.264 MMU Phase 1: power on the HEVC SAO pipeline BEFORE any
	 * VDEC_1 register writes.  On SM1, vdec_hevc_power_on() triggers
	 * a combined-power-domain VDEC_1 reset (DOS_SW_RESET0=0xfffffffc)
	 * that clears all AV_SCRATCH registers.  This must happen before
	 * we program those registers below.
	 *
	 * SAO register init happens later in codec_h264_src_change() when
	 * frame dimensions are known.
	 */
	if (h264_mmu_enable &&
	    core->platform->revision >= VDEC_REVISION_G12A) {
		int ret = vdec_hevc_power_on(core);

		if (ret) {
			dev_err(core->dev,
				"H.264 MMU: HEVC power-on failed (%d)\n", ret);
			return ret;
		}
		h264->hevc_powered = true;
	}

	amvdec_write_dos_bits(core, POWER_CTL_VLD,
			      POWER_CTL_VLD_MEM_GATE |
			      POWER_CTL_VLD_PSCALE_CLK);

	/*
	 * Clear parameter registers that may contain stale values from the
	 * engine-level scratch canvas fill (AV_SCRATCH_0..L are pre-filled
	 * with canvas IDs to prevent framebuffer corruption).
	 */
	amvdec_write_dos(core, AV_SCRATCH_0, 0);
	amvdec_write_dos(core, AV_SCRATCH_7, 0);
	amvdec_write_dos(core, AV_SCRATCH_8, 0);
	amvdec_write_dos(core, AV_SCRATCH_D, 0);
	amvdec_write_dos(core, AV_SCRATCH_F, 0);
	amvdec_write_dos(core, AV_SCRATCH_J, 0);

	workspace_offset = h264->workspace_paddr - WORKSPACE_BUF_OFFSET;
	amvdec_write_dos(core, AV_SCRATCH_1, workspace_offset);
	amvdec_write_dos(core, AV_SCRATCH_G, h264->ext_fw_paddr);
	amvdec_write_dos(core, AV_SCRATCH_I, h264->sei_paddr -
					     workspace_offset);

	/* Read I-only from module parameter (global) or V4L2 control (per-session).
	 * Module parameter takes priority when non-zero.
	 * BSP vmh264.c: i_only_flag module_param overrides hw->i_only.
	 */
	if (i_only_flag)
		h264->i_only = true;
	else if (sess->ctrl_i_only)
		h264->i_only = v4l2_ctrl_g_ctrl(sess->ctrl_i_only);

	/* Enable error-recovery (skip to IDR on error) + IP-only flag.
	 * When i_only is set, also enable firmware I-frame-only mode
	 * via AV_SCRATCH_F bit 6 (BSP vmh264.c:10004-10007).
	 */
	amvdec_write_dos(core, AV_SCRATCH_F,
			 ERR_RECOVERY_MODE | ERR_RECOVERY_IP_ONLY |
			 (h264->i_only ? I_ONLY_FW_MODE : 0));

	/* Fast output DPB mode bitmask (BSP vmh264.c fast_output_enable).
	 * Written to AV_SCRATCH_K; 0 = normal DPB reorder.
	 * Value set via V4L2_CID_MESON_VDEC_FAST_OUTPUT control.
	 */
	amvdec_write_dos(core, AV_SCRATCH_K, sess->fast_output_enable);

	amvdec_write_dos(core, MDEC_PIC_DC_THRESH, MDEC_PIC_DC_THRESH_DEFAULT);

	/*
	 * BSP reset pulse for MDEC_PIC_DC_CTRL upper bits [30:24].
	 * Sets 0xBF then immediately clears, resetting the DC
	 * control logic.
	 */
	amvdec_write_dos_bits(core, MDEC_PIC_DC_CTRL, MDEC_PIC_DC_CTRL_RESET_MASK);
	amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL, MDEC_PIC_DC_CTRL_RESET_MASK);

	amvdec_write_dos(core, IQIDCT_CONTROL, IQIDCT_CONTROL_INIT);

	/*
	 * On GXLX (S805X), U-Boot sets DOS_SCRATCH_31 bit 3 to cap H.264
	 * decode at ~1080p via a firmware resolution check at gxl_h264.bin
	 * offset 0x1079: if bit 3 set, firmware enforces
	 * (mb_width-1)+(mb_height-1) <= 188, producing CMD_BAD_WIDTH for
	 * any frame where mb_width+mb_height > 190.
	 *
	 * Do NOT clear this cap: the firmware hangs at MBY_MBX=0 for any
	 * resolution above 1080p regardless of DOS_SCRATCH_31.  The cap
	 * is protective -- it causes a clean CMD_BAD_WIDTH rejection
	 * instead of a firmware deadlock.  Tested 2026-04-05 on S805X:
	 * 1920x1200 and 2560x1440 both produce infinite MBY_MBX=0 stall
	 * when the cap is cleared.
	 */

	/*
	 * On G12A+, clear MMC_CTRL1 bit 3 for non-compressed memory mode.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_clear_dos_bits(core, VDEC_ASSIST_MMC_CTRL1, BIT(3));

	/* Start decode stall watchdog (BSP vmh264.c:12304) */
	h264->sess = sess;
	timer_setup(&h264->check_timer, codec_h264_check_timer, 0);
	codec_h264_start_watchdog(h264);
	mod_timer(&h264->check_timer, jiffies + CHECK_INTERVAL);

	return 0;
}

static int codec_h264_stop(struct amvdec_session *sess)
{
	struct codec_h264 *h264 = sess->priv;
	struct amvdec_core *core = sess->core;

	/* Stop decode stall watchdog before freeing resources */
	del_timer_sync(&h264->check_timer);

	if (h264->ext_fw_vaddr) {
		dma_free_coherent(core->dev, SIZE_EXT_FW,
				  h264->ext_fw_vaddr, h264->ext_fw_paddr);
		h264->ext_fw_vaddr = NULL;
	}

	if (h264->workspace_vaddr) {
		dma_free_coherent(core->dev, SIZE_WORKSPACE,
				  h264->workspace_vaddr, h264->workspace_paddr);
		h264->workspace_vaddr = NULL;
	}

	if (h264->ref_vaddr) {
		dma_free_coherent(core->dev, h264->ref_size,
				  h264->ref_vaddr, h264->ref_paddr);
		h264->ref_vaddr = NULL;
	}

	if (h264->sei_vaddr) {
		dma_free_coherent(core->dev, SIZE_SEI,
				  h264->sei_vaddr, h264->sei_paddr);
		h264->sei_vaddr = NULL;
	}

	if (h264->hevc_powered) {
		vdec_hevc_power_off(core);
		h264->hevc_powered = false;
	}

	return 0;
}

static int codec_h264_load_extended_firmware(struct amvdec_session *sess,
					     const u8 *data, u32 len,
					     u32 mc_size)
{
	struct codec_h264 *h264;
	struct amvdec_core *core = sess->core;

	/*
	 * data points to the full firmware; the extended payload starts
	 * at mc_size (the portion loaded to IMEM by vdec_1_load_firmware).
	 */
	if (len < mc_size + SIZE_EXT_FW)
		return -EINVAL;

	h264 = kzalloc(sizeof(*h264), GFP_KERNEL);
	if (!h264)
		return -ENOMEM;

	h264->ext_fw_vaddr = dma_alloc_coherent(core->dev, SIZE_EXT_FW,
						&h264->ext_fw_paddr,
						GFP_KERNEL);
	if (!h264->ext_fw_vaddr) {
		kfree(h264);
		return -ENOMEM;
	}

	memcpy(h264->ext_fw_vaddr, data + mc_size, SIZE_EXT_FW);

	/*
	 * Patch BSP g12a_h264.bin CMD_FRAMES_READY race on G12A+ (KI-091).
	 *
	 * The frame output loop at AMRISC address 0x19e5-0x19f7 reads
	 * SRAM[0x1c7] (pending frame counter) into r13 and skips frame
	 * output if r13 < 1.  On 12nm AMRISC the SRAM read returns
	 * stale 0, skipping the output loop.  Fix: spin-wait (loop back
	 * to re-read) instead of single-check-and-skip.
	 *
	 * Target instruction at AMRISC address 0x19eb:
	 *   Original: blt r13, r12, +12  (0x0d c3 80 0b at fw byte 0x67AC)
	 *   Patched:  blt r13, r12, -6   (0x0d a3 8f 0b at fw byte 0x67AC)
	 *
	 * The instruction is in the extended firmware area (byte 0x67AC >
	 * MC_SIZE 0x4000).  Offset within ext_fw_vaddr = 0x67AC - 0x4000
	 * = 0x27AC.
	 *
	 * Guard: G12A+ only, firmware name contains "h264" (not "mvc"),
	 * verify the 4 bytes match the unpatched instruction.
	 */
	#define KI091_EXT_OFFSET  0x27AC  /* 0x67AC - MC_SIZE */
	if (core->platform->revision >= VDEC_REVISION_G12A &&
	    strstr(sess->fmt_out->firmware_path, "h264") &&
	    !strstr(sess->fmt_out->firmware_path, "mvc") &&
	    KI091_EXT_OFFSET + 4 <= SIZE_EXT_FW &&
	    ((u8 *)h264->ext_fw_vaddr)[KI091_EXT_OFFSET + 0] == 0x0d &&
	    ((u8 *)h264->ext_fw_vaddr)[KI091_EXT_OFFSET + 1] == 0xc3 &&
	    ((u8 *)h264->ext_fw_vaddr)[KI091_EXT_OFFSET + 2] == 0x80 &&
	    ((u8 *)h264->ext_fw_vaddr)[KI091_EXT_OFFSET + 3] == 0x0b) {
		((u8 *)h264->ext_fw_vaddr)[KI091_EXT_OFFSET + 1] = 0xa3;
		((u8 *)h264->ext_fw_vaddr)[KI091_EXT_OFFSET + 2] = 0x8f;
		dev_info(core->dev,
			 "firmware workaround: patched H.264 CMD_FRAMES_READY race at 0x19eb (G12A+ BSP fw, KI-091)\n");
	}
	#undef KI091_EXT_OFFSET

	sess->priv = h264;

	return 0;
}

static const struct v4l2_fract par_table[] = {
	{ 1, 1 },   { 1, 1 },    { 12, 11 }, { 10, 11 },
	{ 16, 11 }, { 40, 33 },  { 24, 11 }, { 20, 11 },
	{ 32, 11 }, { 80, 33 },  { 18, 11 }, { 15, 11 },
	{ 64, 33 }, { 160, 99 }, { 4, 3 },   { 3, 2 },
	{ 2, 1 }
};

static void codec_h264_set_par(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	u32 seq_info = amvdec_read_dos(core, AV_SCRATCH_2);
	u32 ar_idc = (seq_info >> AR_IDC_BIT) & AR_IDC_MASK;

	if (!(seq_info & AR_PRESENT_FLAG))
		return;

	if (ar_idc == AR_EXTEND) {
		u32 ar_info = amvdec_read_dos(core, AV_SCRATCH_3);

		/* Extended SAR: sar_width in [15:0], sar_height in [31:16] */
		sess->pixelaspect.numerator = ar_info & 0xffff;
		sess->pixelaspect.denominator = (ar_info >> 16) & 0xffff;
		return;
	}

	if (ar_idc >= ARRAY_SIZE(par_table))
		return;

	sess->pixelaspect = par_table[ar_idc];
}

static void codec_h264_resume(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_h264 *h264 = sess->priv;
	u32 mb_width, mb_height, mb_total;

	amvdec_set_canvases(sess,
			    (u32[]){ ANC0_CANVAS_ADDR, 0 },
			    (u32[]){ 32, 0 });

	dev_dbg(core->dev, "max_refs = %u; actual_dpb_size = %u\n",
		h264->max_refs, sess->num_dst_bufs);

	/* Align to a multiple of 4 macroblocks */
	mb_width = ALIGN(h264->mb_width, 4);
	mb_height = ALIGN(h264->mb_height, 4);
	mb_total = mb_width * mb_height;

	/* MV buffer only needs slots for actual reference frames, not display
	 * buffers.  Clamp to the number of capture buffers available -- if the
	 * stream claims 16 refs but only 9 capture buffers were allocated, the
	 * firmware cannot use more references than buffers.
	 */
	h264->ref_size = mb_total * MB_MV_SIZE *
			 min(h264->max_refs, sess->num_dst_bufs);
	h264->ref_vaddr = dma_alloc_coherent(core->dev, h264->ref_size,
					     &h264->ref_paddr, GFP_KERNEL);
	if (!h264->ref_vaddr) {
		dev_err(core->dev,
			"H.264 MV buffer alloc failed: %u bytes (%ux%u, %u refs, %u bufs)\n",
			h264->ref_size, sess->width, sess->height,
			h264->max_refs, sess->num_dst_bufs);
		amvdec_abort(sess);
		return;
	}

	/* Address to store the references' MVs */
	amvdec_write_dos(core, AV_SCRATCH_1, h264->ref_paddr);
	/* End of ref MV */
	amvdec_write_dos(core, AV_SCRATCH_4, h264->ref_paddr + h264->ref_size);

	/*
	 * Disable DCAC DDR read cache for narrow frames (BSP vmh264.c:4684).
	 * Without this, QCIF (176x144) and other sub-256px-wide streams
	 * produce zero decoded frames on G12A/SM1.
	 */
	if (sess->width <= 256) {
		amvdec_write_dos_bits(core, IQIDCT_CONTROL,
				      IQIDCT_DCAC_DDR_CACHE_DIS);
		if (core->platform->revision >= VDEC_REVISION_G12A) {
			u32 val = amvdec_read_dos(core, DCAC_DDR_BYTE64_CTL);

			amvdec_write_dos(core, DCAC_DDR_BYTE64_CTL,
					 (val & ~DCAC_DDR_BYTE64_LOW_MASK) |
					 DCAC_DDR_BYTE64_NARROW_VAL);
		}
	}

	amvdec_write_dos(core, AV_SCRATCH_0,
			 (h264->max_refs << SCRATCH0_MAX_REF_BIT) |
			 (sess->num_dst_bufs << SCRATCH0_DPB_SIZE_BIT) |
			 ((h264->max_refs - 1) << SCRATCH0_REF_MINUS1_BIT));
}

/*
 * codec_h264_pre_queue() - reject unsupported H.264 profiles early.
 *
 * Scan the OUTPUT buffer for an SPS NAL unit (nal_unit_type 7) and extract
 * profile_idc (first byte of the RBSP).  The Amlogic VDEC firmware only
 * supports 8-bit 4:2:0 decode, so High10 (110), High 4:2:2 (122), and
 * High 4:4:4 Predictive (244) must be rejected before the firmware
 * silently produces garbage or zero frames.
 *
 * The check only runs until an SPS is found; once the profile is known,
 * subsequent buffers are passed through without scanning.
 *
 * H.264 NAL start code: 00 00 01 or 00 00 00 01.
 * SPS NAL: nal_unit_type = (nal_header & 0x1F) == 7.
 * profile_idc is the byte immediately after the NAL header byte.
 */
/*
 * Minimal exp-Golomb and SPS dimension parser for 4096px overflow recovery.
 *
 * When the firmware's 8-bit mb_width wraps to 0 at exactly 4096px (256 MBs),
 * we need dimensions from the raw SPS NAL before the firmware sees it.
 * This parser extracts only what we need: profile_idc through
 * pic_width_in_mbs_minus1 and pic_height_in_map_units_minus1.
 *
 * Reference: ITU-T H.264 sec 7.3.2.1.1 (SPS syntax).
 */
struct bitreader {
	const u8 *data;
	u32 size;	/* bytes */
	u32 bit_pos;	/* current bit offset */
};

static inline int br_read_bit(struct bitreader *br)
{
	u32 byte_off, bit_off;

	if (br->bit_pos >= br->size * 8)
		return -1;
	byte_off = br->bit_pos / 8;
	bit_off = 7 - (br->bit_pos % 8);
	br->bit_pos++;
	return (br->data[byte_off] >> bit_off) & 1;
}

static inline int br_read_bits(struct bitreader *br, int n)
{
	int val = 0, bit;

	while (n-- > 0) {
		bit = br_read_bit(br);
		if (bit < 0)
			return -1;
		val = (val << 1) | bit;
	}
	return val;
}

/* Read unsigned exp-Golomb code (H.264 sec 9.1) */
static int br_read_ue(struct bitreader *br)
{
	int leading_zeros = 0;
	int bit;

	while ((bit = br_read_bit(br)) == 0) {
		leading_zeros++;
		if (leading_zeros > 31)
			return -1;
	}
	if (bit < 0)
		return -1;
	if (leading_zeros == 0)
		return 0;
	bit = br_read_bits(br, leading_zeros);
	if (bit < 0)
		return -1;
	return (1 << leading_zeros) - 1 + bit;
}

/* Read signed exp-Golomb code (H.264 sec 9.1.1) -- needed for SPS offsets */
static int br_read_se(struct bitreader *br)
{
	int val = br_read_ue(br);

	if (val < 0)
		return val;
	return (val & 1) ? (val + 1) / 2 : -(val / 2);
}

/*
 * codec_h264_parse_sps_dimensions() - extract dimensions from SPS RBSP.
 *
 * Parses just enough of the SPS to get pic_width_in_mbs_minus1 and
 * pic_height_in_map_units_minus1.  Stores results in h264->sps_mb_width
 * and h264->sps_mb_height (in macroblocks, NOT pixels).
 *
 * @rbsp: pointer to first byte after the NAL header (profile_idc byte)
 * @rbsp_size: remaining bytes
 * @h264: codec context to store results
 *
 * Returns 0 on success, -1 on parse error.
 */
static int codec_h264_parse_sps_dimensions(const u8 *rbsp, u32 rbsp_size,
					   struct codec_h264 *h264)
{
	struct bitreader br = { .data = rbsp, .size = rbsp_size, .bit_pos = 0 };
	int profile_idc, constraint_set_flags, level_idc;
	int seq_parameter_set_id;
	int chroma_format_idc = 1; /* default for non-High profiles */
	int log2_max_frame_num;
	int pic_order_cnt_type;
	int num_ref_frames __attribute__((unused));
	int gaps_in_frame_num_allowed __attribute__((unused));
	int pic_width_in_mbs_minus1;
	int pic_height_in_map_units_minus1;
	int frame_mbs_only_flag;
	int i, val;

	/* profile_idc: u(8) */
	profile_idc = br_read_bits(&br, 8);
	if (profile_idc < 0)
		return -1;
	/* constraint_set0..5_flags + reserved: u(8) */
	constraint_set_flags = br_read_bits(&br, 8);
	(void)constraint_set_flags;
	/* level_idc: u(8) */
	level_idc = br_read_bits(&br, 8);
	h264->sps_level_idc = (u8)level_idc;
	/* seq_parameter_set_id: ue(v) */
	seq_parameter_set_id = br_read_ue(&br);
	(void)seq_parameter_set_id;
	if (seq_parameter_set_id < 0)
		return -1;

	/* High profile and above have extra fields */
	if (profile_idc == 100 || profile_idc == 110 ||
	    profile_idc == 122 || profile_idc == 244 ||
	    profile_idc == 44  || profile_idc == 83  ||
	    profile_idc == 86  || profile_idc == 118 ||
	    profile_idc == 128 || profile_idc == 138 ||
	    profile_idc == 139 || profile_idc == 134 ||
	    profile_idc == 135) {
		int separate_colour_plane_flag;
		int bit_depth_luma_minus8, bit_depth_chroma_minus8;
		int qpprime_y_zero_transform_bypass_flag;
		int seq_scaling_matrix_present_flag;

		chroma_format_idc = br_read_ue(&br);
		if (chroma_format_idc < 0)
			return -1;
		if (chroma_format_idc == 3) {
			separate_colour_plane_flag = br_read_bit(&br);
			(void)separate_colour_plane_flag;
		}
		bit_depth_luma_minus8 = br_read_ue(&br);
		(void)bit_depth_luma_minus8;
		bit_depth_chroma_minus8 = br_read_ue(&br);
		(void)bit_depth_chroma_minus8;
		qpprime_y_zero_transform_bypass_flag = br_read_bit(&br);
		(void)qpprime_y_zero_transform_bypass_flag;
		seq_scaling_matrix_present_flag = br_read_bit(&br);
		if (seq_scaling_matrix_present_flag == 1) {
			int cnt = (chroma_format_idc != 3) ? 8 : 12;
			for (i = 0; i < cnt; i++) {
				int present = br_read_bit(&br);
				if (present == 1) {
					/* Skip scaling list (delta values) */
					int size = (i < 6) ? 16 : 64;
					int last_scale = 8, next_scale = 8;
					int j;
					for (j = 0; j < size; j++) {
						if (next_scale != 0) {
							int delta = br_read_se(&br);
							if (delta == -1 && br.bit_pos >= br.size * 8)
								return -1;
							next_scale = (last_scale + delta + 256) % 256;
						}
						last_scale = (next_scale == 0) ? last_scale : next_scale;
					}
				}
			}
		}
	}

	/* log2_max_frame_num_minus4: ue(v) */
	log2_max_frame_num = br_read_ue(&br);
	(void)log2_max_frame_num;
	if (log2_max_frame_num < 0)
		return -1;

	/* pic_order_cnt_type: ue(v) */
	pic_order_cnt_type = br_read_ue(&br);
	if (pic_order_cnt_type < 0)
		return -1;

	if (pic_order_cnt_type == 0) {
		/* log2_max_pic_order_cnt_lsb_minus4: ue(v) */
		val = br_read_ue(&br);
		if (val < 0)
			return -1;
	} else if (pic_order_cnt_type == 1) {
		int num_ref_frames_in_poc_cycle;

		/* delta_pic_order_always_zero_flag: u(1) */
		br_read_bit(&br);
		/* offset_for_non_ref_pic: se(v) */
		br_read_se(&br);
		/* offset_for_top_to_bottom_field: se(v) */
		br_read_se(&br);
		/* num_ref_frames_in_pic_order_cnt_cycle: ue(v) */
		num_ref_frames_in_poc_cycle = br_read_ue(&br);
		if (num_ref_frames_in_poc_cycle < 0)
			return -1;
		for (i = 0; i < num_ref_frames_in_poc_cycle; i++) {
			val = br_read_se(&br);
			if (val < 0 && br.bit_pos >= br.size * 8)
				return -1;
		}
	}

	/* max_num_ref_frames: ue(v) */
	num_ref_frames = br_read_ue(&br);
	/* gaps_in_frame_num_value_allowed_flag: u(1) */
	gaps_in_frame_num_allowed = br_read_bit(&br);
	/* pic_width_in_mbs_minus1: ue(v) */
	pic_width_in_mbs_minus1 = br_read_ue(&br);
	if (pic_width_in_mbs_minus1 < 0)
		return -1;
	/* pic_height_in_map_units_minus1: ue(v) */
	pic_height_in_map_units_minus1 = br_read_ue(&br);
	if (pic_height_in_map_units_minus1 < 0)
		return -1;
	/* frame_mbs_only_flag: u(1) */
	frame_mbs_only_flag = br_read_bit(&br);
	if (frame_mbs_only_flag < 0)
		return -1;

	h264->sps_mb_width = pic_width_in_mbs_minus1 + 1;
	/* For interlaced, height in MBs = map_units * 2 */
	h264->sps_mb_height = (pic_height_in_map_units_minus1 + 1) *
			      (frame_mbs_only_flag ? 1 : 2);

	return 0;
}

/*
 * codec_h264_slice_sanity_check() - KI-039: reject corrupt slice NALs on G12A+.
 *
 * The G12A/G12B/SM1 H.264 firmware added a DCAC DDR polling loop at
 * instructions 0x0328-0x032a that spins on DCAC_DDR_BYTE64_CTL bits[10:8]
 * until they clear.  Bitflipped NAL data that corrupts the DCAC coefficient
 * region can cause this loop to spin indefinitely, wedging the AMRISC until
 * the decode_timeout watchdog fires and aborts the session.
 *
 * GXL firmware has no such loop; corrupted NALs fail gracefully there.
 *
 * This function is a lightweight sanity gate: it parses just enough of each
 * slice header to detect obviously corrupt NALs (bitflipped fields with
 * impossible values) before submitting them to the firmware.  It is NOT a
 * full RBSP parser and will not catch all corruption -- it only catches the
 * class of bitflip corruption that triggers the DCAC DDR spin loop.
 *
 * Slice header fields checked (ITU-T H.264 sec 7.3.3):
 *   first_mb_in_slice: ue(v)  -- must be <= mb_total (if known)
 *   slice_type:        ue(v)  -- must be <= 9
 *   pic_parameter_set_id: ue(v) -- must be <= 255
 *
 * @data: raw Annex B buffer
 * @size: buffer size in bytes
 * @mb_total: total macroblock count from SPS (0 if not yet parsed)
 *
 * Returns true if all slice NALs pass, false if any field is out of range.
 */
static bool codec_h264_slice_sanity_check(const u8 *data, u32 size,
					  u32 mb_total)
{
	/*
	 * H.264 slice types 0-9 are valid (ITU-T H.264 Table 7-6).
	 * Types 0-4 are normal; 5-9 are the "all slices same type" variants.
	 * Any value > 9 in a real stream is impossible.
	 */
	const u32 SLICE_TYPE_MAX = 9;
	/*
	 * pic_parameter_set_id is ue(v), max value 255 (ITU-T H.264 sec 7.4.3).
	 * A value > 255 cannot reference a valid PPS.
	 */
	const u32 PPS_ID_MAX = 255;
	u32 i;

	for (i = 0; i + 4 < size; i++) {
		u8 nal_type;
		u32 hdr_len;
		struct bitreader br;
		int first_mb, slice_type, pps_id;

		if (data[i] != 0x00 || data[i + 1] != 0x00)
			continue;

		if (data[i + 2] == 0x01) {
			hdr_len = 3;
		} else if (data[i + 2] == 0x00 && i + 5 < size &&
			   data[i + 3] == 0x01) {
			hdr_len = 4;
		} else {
			continue;
		}

		if (i + hdr_len >= size)
			break;

		nal_type = data[i + hdr_len] & 0x1f;

		/* Only check VCL slice NALs (non-IDR=1, IDR=5) */
		if (nal_type != NAL_TYPE_SLICE && nal_type != NAL_TYPE_IDR)
			continue;

		/* RBSP begins immediately after the NAL header byte */
		if (i + hdr_len + 1 >= size)
			continue;

		br.data = &data[i + hdr_len + 1];
		br.size = size - (i + hdr_len + 1);
		br.bit_pos = 0;

		/* first_mb_in_slice: ue(v) */
		first_mb = br_read_ue(&br);
		if (first_mb < 0)
			return false; /* truncated or impossible leading zeros */
		if (mb_total > 0 && (u32)first_mb >= mb_total)
			return false; /* first_mb beyond picture bounds */

		/* slice_type: ue(v) */
		slice_type = br_read_ue(&br);
		if (slice_type < 0 || (u32)slice_type > SLICE_TYPE_MAX)
			return false;

		/* pic_parameter_set_id: ue(v) */
		pps_id = br_read_ue(&br);
		if (pps_id < 0 || (u32)pps_id > PPS_ID_MAX)
			return false;
	}

	return true;
}

/*
 * codec_h264_skip_non_idr() - check if buffer contains only non-IDR slices.
 *
 * When i_only mode is enabled, scan the buffer for NAL start codes.
 * If ALL VCL NALs are non-IDR (type 1), return true to skip this buffer.
 * If any IDR NAL (type 5) is found, return false to process it.
 * Non-VCL NALs (SPS, PPS, SEI) are ignored for this check.
 *
 * Returns true if the buffer should be skipped (no IDR NALs found).
 */
static bool codec_h264_skip_non_idr(const u8 *data, u32 size)
{
	bool has_vcl = false;
	u32 i;

	for (i = 0; i + 4 < size; i++) {
		u8 nal_type;
		u32 hdr_len;

		if (data[i] != 0x00 || data[i + 1] != 0x00)
			continue;

		if (data[i + 2] == 0x01) {
			hdr_len = 3;
		} else if (data[i + 2] == 0x00 && i + 5 < size &&
			   data[i + 3] == 0x01) {
			hdr_len = 4;
		} else {
			continue;
		}

		if (i + hdr_len >= size)
			break;

		nal_type = data[i + hdr_len] & 0x1f;

		if (nal_type == NAL_TYPE_IDR)
			return false; /* IDR found -- don't skip */

		if (nal_type == NAL_TYPE_SLICE)
			has_vcl = true; /* non-IDR slice */
	}

	/* Skip if we found non-IDR VCL NALs but no IDR */
	return has_vcl;
}

static int codec_h264_pre_queue(struct amvdec_session *sess,
				struct vb2_buffer *vb)
{
	struct amvdec_core *core = sess->core;
	u8 *data;
	u32 size;
	u32 i;

	/* I-only decode: skip buffers containing only non-IDR slices.
	 * This makes the firmware process only IDR I-frames.
	 * We must still allow SPS/PPS/SEI through for stream setup.
	 * Use h264->i_only so both the module param and V4L2 ctrl activate
	 * host-side filtering (BSP vmh264.c i_only_flag / hw->i_only).
	 */
	{
		struct codec_h264 *h264_pre = sess->priv;

		if (h264_pre && h264_pre->i_only) {
			u8 *idata = vb2_plane_vaddr(vb, 0);
			u32 isize = vb2_get_plane_payload(vb, 0);

			if (idata && isize && codec_h264_skip_non_idr(idata, isize)) {
				vb2_set_plane_payload(vb, 0, 0);
				return 0;
			}
		}
	}

	data = vb2_plane_vaddr(vb, 0);
	if (!data)
		return 0;

	size = vb2_get_plane_payload(vb, 0);

	/*
	 * SPS profile and dimension scan.
	 *
	 * Only scan until we've processed the first SPS.  After codec_start,
	 * keyframe_found is 0; it gets set to 1 in src_change.  We use it as
	 * a "SPS already checked" flag: once the firmware has parsed an SPS and
	 * triggered src_change, h264->sps_mb_width/height are already populated
	 * from a previous call and there is no need to rescan on every buffer.
	 *
	 * This scan runs BEFORE the KI-039 slice sanity check below so that
	 * h264->sps_mb_width/height are populated when the first buffer contains
	 * both an SPS and slice NALs (the common case for Annex B streams).
	 */
	if (!sess->keyframe_found) {
		/* Scan for Annex B start codes followed by SPS NAL (type 7) */
		for (i = 0; i + 4 < size; i++) {
			u8 nal_type;
			u8 profile_idc;
			u32 hdr_len;

			/* Look for 3-byte (00 00 01) or 4-byte (00 00 00 01) start code */
			if (data[i] != 0x00 || data[i + 1] != 0x00)
				continue;

			if (data[i + 2] == 0x01) {
				hdr_len = 3;
			} else if (data[i + 2] == 0x00 && i + 5 < size &&
				   data[i + 3] == 0x01) {
				hdr_len = 4;
			} else {
				continue;
			}

			if (i + hdr_len + 1 >= size)
				break;

			nal_type = data[i + hdr_len] & 0x1f;
			if (nal_type != 7) /* Not SPS */
				continue;

			/* profile_idc is the byte after the NAL header */
			if (i + hdr_len + 2 >= size)
				break;

			profile_idc = data[i + hdr_len + 1];

			if (profile_idc == H264_PROFILE_HIGH10 ||
			    profile_idc == H264_PROFILE_HIGH_422 ||
			    profile_idc == H264_PROFILE_HIGH_444_PRED) {
				dev_err(core->dev,
					"H.264 profile_idc %u not supported (firmware is 8-bit 4:2:0 only)\n",
					profile_idc);
				amvdec_abort(sess);
				return -EINVAL;
			}

			/* Parse SPS dimensions for 4096px overflow recovery and
			 * KI-039 first_mb_in_slice range check.
			 * The RBSP starts at the profile_idc byte (hdr_len+1).
			 */
			if (i + hdr_len + 1 < size) {
				struct codec_h264 *h264 = sess->priv;

				if (codec_h264_parse_sps_dimensions(
					    &data[i + hdr_len + 1],
					    size - (i + hdr_len + 1),
					    h264) == 0) {
					dev_dbg(core->dev,
						"H.264 SPS parsed: %ux%u MBs (%ux%u px)\n",
						h264->sps_mb_width,
						h264->sps_mb_height,
						h264->sps_mb_width * 16,
						h264->sps_mb_height * 16);

					if (h264->sps_mb_width && h264->sps_mb_height) {
						u32 mb_total = h264->sps_mb_width *
							       h264->sps_mb_height;
						u32 limit = sess->fmt_out->max_mb_total;

						if (limit && mb_total > limit) {
							dev_err(core->dev,
								"H.264 mb_total %u exceeds platform max %u (%ux%u px)\n",
								mb_total, limit,
								h264->sps_mb_width * 16,
								h264->sps_mb_height * 16);
							amvdec_abort(sess);
							return -EINVAL;
						}
					}
				}
			}

			/* Found a supported SPS; fall through to KI-039 check below */
			break;
		}
	}

	/*
	 * KI-039: G12A/G12B/SM1 firmware DCAC DDR spin loop on corrupt slice NALs.
	 *
	 * The G12A firmware (g12a_h264_multi.asm, used on G12B/A311D and SM1)
	 * added a polling loop at instructions 0x0328-0x032a that reads
	 * DCAC_DDR_BYTE64_CTL (host reg 0x3874) bits[10:8] and spins until they
	 * clear.  Bitflipped NAL data that corrupts the DCAC coefficient area
	 * can stall the DMA and hold these bits set indefinitely, causing the
	 * AMRISC to spin until the decode_timeout watchdog aborts the session.
	 *
	 * GXL firmware has no 0x0e1d (DCAC_DDR_BYTE64_CTL) access and handles
	 * corrupted NALs gracefully -- this guard is only needed on G12A+.
	 *
	 * Scan slice headers for clearly-impossible field values that indicate
	 * corruption.  If any slice NAL fails, drop this buffer with an error
	 * state rather than submitting it to firmware.  The session survives.
	 *
	 * The SPS scan above populates h264->sps_mb_width/height first, so the
	 * first_mb_in_slice range check is effective even on the first buffer.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A) {
		struct codec_h264 *h264_sane = sess->priv;
		u32 mb_total_sane = 0;

		if (h264_sane)
			mb_total_sane = h264_sane->sps_mb_width *
					h264_sane->sps_mb_height;

		if (!codec_h264_slice_sanity_check(data, size, mb_total_sane)) {
			dev_warn(core->dev,
				 "H.264 KI-039: dropping corrupt slice NAL (out-of-range header fields); session preserved\n");
			return -EINVAL;
		}
	}

	return 0;
}

static void codec_h264_src_change(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_h264 *h264 = sess->priv;
	u32 parsed_info, mb_total;
	u32 crop_infor, crop_bottom, crop_right;
	u32 frame_width, frame_height;
	u32 coded_width, coded_height;
	u32 seq_info, chroma_format_idc;

	sess->keyframe_found = 1;

	/*
	 * Safety net: check chroma_format_idc from firmware-parsed SPS.
	 * The firmware reports this in AV_SCRATCH_2 bits [14:13].
	 * The hardware only supports 4:2:0 (chroma_format_idc=1).
	 * 4:2:2 and 4:4:4 streams silently produce 0 frames; reject early.
	 * BSP vmh264.c:1153-1157, 6588-6601.
	 */
	seq_info = amvdec_read_dos(core, AV_SCRATCH_2);
	chroma_format_idc = (seq_info >> SEQ_INFO_CHROMA_FMT_BIT) &
			    SEQ_INFO_CHROMA_FMT_MASK;
	if (chroma_format_idc > 1) {
		dev_err(core->dev,
			"H.264 chroma_format_idc %u not supported (firmware is 4:2:0 only)\n",
			chroma_format_idc);
		amvdec_abort(sess);
		return;
	}

	parsed_info = amvdec_read_dos(core, AV_SCRATCH_1);

	/* Total number of 16x16 macroblocks */
	mb_total = (parsed_info >> MB_TOTAL_BIT) & MB_TOTAL_MASK;
	/* Number of macroblocks per line */
	h264->mb_width = parsed_info & MB_WIDTH_MASK;

	/*
	 * Validate macroblock dimensions are consistent.
	 * The firmware's mb_width field is 8 bits (max 255), so streams
	 * wider than 4080px (255*16) overflow, producing a wrong mb_width
	 * and consequently a bogus mb_height (e.g. 5120px -> mb_width=64
	 * instead of 320, giving mb_height=900 and coded_height=14400).
	 * Detect this via mb_width * mb_height != mb_total.
	 */
	/*
	 * Firmware 8-bit overflow recovery: at exactly 4096px (256 MBs),
	 * mb_width wraps to 0.  Try to recover using:
	 *  1) mb_total (if non-zero, 16-bit field can hold up to 65535)
	 *  2) SPS dimensions parsed in pre_queue
	 * If neither is available, abort as before.
	 */
	if (h264->mb_width == 0) {
		if (mb_total > 0 && h264->sps_mb_width > 0) {
			/* Best case: both mb_total and SPS available */
			h264->mb_width = h264->sps_mb_width;
			h264->mb_height = h264->sps_mb_height;
			dev_info(core->dev,
				 "H.264 mb_width=0 overflow: using SPS dims %ux%u MBs (mb_total=%u, parsed_info=0x%08x)\n",
				 h264->mb_width, h264->mb_height,
				 mb_total, parsed_info);
		} else if (mb_total > 0 && h264->sps_mb_height > 0) {
			/* mb_total valid, compute mb_width from mb_total/sps_height */
			h264->mb_width = mb_total / h264->sps_mb_height;
			h264->mb_height = h264->sps_mb_height;
			dev_info(core->dev,
				 "H.264 mb_width=0 overflow: computed mb_width=%u from mb_total=%u/sps_height=%u\n",
				 h264->mb_width, mb_total, h264->sps_mb_height);
		} else if (h264->sps_mb_width > 0) {
			/* mb_total also 0 but SPS parsed */
			h264->mb_width = h264->sps_mb_width;
			h264->mb_height = h264->sps_mb_height;
			mb_total = h264->mb_width * h264->mb_height;
			dev_info(core->dev,
				 "H.264 mb_width=0 + mb_total=0: using SPS dims %ux%u MBs (computed mb_total=%u)\n",
				 h264->mb_width, h264->mb_height, mb_total);
		} else {
			dev_err(core->dev,
				"H.264 mb_width=0 overflow: no SPS fallback available (mb_total=%u, parsed_info=0x%08x)\n",
				mb_total, parsed_info);
			amvdec_abort(sess);
			return;
		}
	} else if (mb_total == 0) {
		/*
		 * CMD_BAD_WIDTH path: firmware writes (mb_width-1) to
		 * AV_SCRATCH_1 in the bad-width notification, not the
		 * normal parsed_info format (mb_total|mb_width).  So
		 * mb_total=0 here means AV_SCRATCH_1 has the wrong
		 * encoding, not that the stream has no macroblocks.
		 * Recover using SPS-parsed dimensions if available.
		 */
		if (h264->sps_mb_width > 0 && h264->sps_mb_height > 0) {
			h264->mb_width = h264->sps_mb_width;
			mb_total = h264->sps_mb_width * h264->sps_mb_height;
			dev_info(core->dev,
				 "H.264 mb_total=0 (bad parsed_info 0x%08x): using SPS dims %ux%u MBs\n",
				 parsed_info,
				 h264->sps_mb_width, h264->sps_mb_height);
		} else {
			dev_err(core->dev,
				"H.264 invalid parsed_info: mb_total=0 mb_width=%u\n",
				h264->mb_width);
			amvdec_abort(sess);
			return;
		}
	}
	/* Number of macroblock lines */
	h264->mb_height = mb_total / h264->mb_width;

	if (h264->mb_width * h264->mb_height != mb_total) {
		dev_err(core->dev,
			"H.264 macroblock overflow: mb_total=%u mb_width=%u (parsed_info=0x%08x). Stream too wide for 8-bit mb_width field\n",
			mb_total, h264->mb_width, parsed_info);
		amvdec_abort(sess);
		return;
	}

	/*
	 * Even when mb_width divides mb_total evenly, the 8-bit field may
	 * have silently wrapped (e.g. 320 & 0xFF = 64, 57600/64 = 900).
	 * Sanity-check the derived coded dimensions against the platform
	 * maximums before amvdec_src_change() gives a confusing error.
	 */
	coded_width = h264->mb_width * 16;
	coded_height = h264->mb_height * 16;
	if ((sess->fmt_out->max_width && coded_width > sess->fmt_out->max_width) ||
	    (sess->fmt_out->max_height && coded_height > sess->fmt_out->max_height)) {
		dev_err(core->dev,
			"H.264 dimensions %ux%u exceed platform max %ux%u (mb_total=%u mb_width=%u); likely 8-bit mb_width overflow\n",
			coded_width, coded_height,
			sess->fmt_out->max_width, sess->fmt_out->max_height,
			mb_total, h264->mb_width);
		amvdec_abort(sess);
		return;
	}

	h264->max_refs = ((parsed_info >> MAX_REF_BIT) & MAX_REF_MASK) + 1;

	/* H.264 spec maximum is 16 reference frames. Clamp firmware-reported
	 * value to prevent unreasonable DPB sizes from malformed streams.
	 */
	if (h264->max_refs > 16) {
		dev_warn(core->dev,
			 "H.264 max_refs %u exceeds spec max 16, clamping\n",
			 h264->max_refs);
		h264->max_refs = 16;
	}

	crop_infor = amvdec_read_dos(core, AV_SCRATCH_6);
	crop_bottom = crop_infor & CROP_BOTTOM_MASK;
	crop_right = (crop_infor >> CROP_RIGHT_BIT) & CROP_RIGHT_MASK;

	/* coded_width and coded_height already computed above for overflow check */

	if (crop_right >= coded_width || crop_bottom >= coded_height) {
		dev_err(core->dev,
			"H.264 crop (%u, %u) exceeds coded size (%ux%u)\n",
			crop_right, crop_bottom,
			coded_width, coded_height);
		amvdec_abort(sess);
		return;
	}

	frame_width = coded_width - crop_right;
	frame_height = coded_height - crop_bottom;

	dev_dbg(core->dev, "frame: %ux%u; crop: %u %u; max_refs: %u\n",
		frame_width, frame_height, crop_right, crop_bottom,
		h264->max_refs);

	codec_h264_set_par(sess);

	/* H.264 MMU Phase 1: init SAO registers with decoded dimensions */
	if (h264->hevc_powered)
		codec_h264_hevc_sao_init(sess, frame_width, frame_height);

	/* SPS parsed = progress; reset watchdog */
	codec_h264_start_watchdog(h264);

	/*
	 * DPB buffer count: max_refs + reorder margin, capped by the H.264
	 * Annex A level limit when the stream declares fewer reference frames
	 * than the level would allow.
	 *
	 * The cap only fires when max_refs > level_max, i.e., the encoder
	 * set max_num_ref_frames=16 in SPS but declared a level that only
	 * permits fewer frames at this resolution.  In that case we use
	 * level_max + margin instead of max_refs + margin, which can save
	 * ~100-140 MB per session at 4K (Level 5.1 permits 5 frames).
	 *
	 * The cap is skipped when level_idc is 0 (not yet parsed) or when
	 * the MB dimensions are unknown (defensive).  No floor is needed:
	 * if level_max >= max_refs the min() has no effect.
	 *
	 * Source: H.264 spec Table A-1 (MaxDpbMbs per level).
	 * BSP reference: vmh264.c get_dec_dpb_size() lines 6056-6146.
	 */
	{
		/* H.264 Annex A Table A-1: MaxDpbMbs per level_idc */
		static const struct { u8 level_idc; u32 max_dpb_mbs; } h264_levels[] = {
			{ 10,    396 }, { 11,    900 }, { 12,   2376 }, { 13,   2376 },
			{ 20,   2376 }, { 21,   4752 }, { 22,   8100 },
			{ 30,   8100 }, { 31,  18000 }, { 32,  20480 },
			{ 40,  32768 }, { 41,  32768 }, { 42,  34816 },
			{ 50, 110400 }, { 51, 184320 }, { 52, 184320 },
			{ 60, 696320 }, { 61, 696320 }, { 62, 696320 },
		};
		u32 dpb = h264->max_refs + sess->reorder_dpb_size_margin;

		dev_dbg(core->dev,
			"H.264 DPB input: level_idc=%u mb=%ux%u max_refs=%u margin=%u dpb=%u\n",
			h264->sps_level_idc, h264->mb_width, h264->mb_height,
			h264->max_refs, sess->reorder_dpb_size_margin, dpb);

		if (h264->sps_level_idc && h264->mb_width && h264->mb_height) {
			u32 pic_mbs = h264->mb_width * h264->mb_height;
			u32 max_dpb_mbs = 0;
			int i;

			for (i = 0; i < ARRAY_SIZE(h264_levels); i++) {
				if (h264_levels[i].level_idc == h264->sps_level_idc) {
					max_dpb_mbs = h264_levels[i].max_dpb_mbs;
					break;
				}
			}

			if (max_dpb_mbs) {
				u32 level_max = max_dpb_mbs / pic_mbs;

				level_max = clamp_val(level_max, 1, 16);
				if (level_max < h264->max_refs) {
					dev_dbg(core->dev,
						"H.264 level %u.%u: DPB cap %u->%u refs (level_max=%u < max_refs=%u)\n",
						h264->sps_level_idc / 10,
						h264->sps_level_idc % 10,
						dpb,
						level_max + sess->reorder_dpb_size_margin,
						level_max, h264->max_refs);
					dpb = level_max + sess->reorder_dpb_size_margin;
				}
			}
		}

		dev_dbg(core->dev,
			"H.264 DPB final: dpb=%u max_buffers=%u -> requesting %u\n",
			dpb, sess->fmt_out->max_buffers,
			min(dpb, (u32)sess->fmt_out->max_buffers));
		amvdec_src_change(sess, frame_width, frame_height,
				  min(dpb, (u32)sess->fmt_out->max_buffers));
	}
}

/*
 * The bitstream offset is split in half in 2 different registers.
 * Fetch its MSB here, which location depends on the frame number.
 *
 * AV_SCRATCH_A..C pack two 16-bit offset MSBs per register:
 *   [31:16] = MSB for odd-numbered frame in the pair
 *   [15:0]  = MSB for even-numbered frame (shifted up to [31:16] on return)
 */
static u32 get_offset_msb(struct amvdec_core *core, int frame_num)
{
	int take_msb = frame_num % 2;
	int reg_offset = (frame_num / 2) * 4;
	u32 offset_msb = amvdec_read_dos(core, AV_SCRATCH_A + reg_offset);

	if (take_msb)
		return offset_msb & 0xffff0000;

	return (offset_msb & 0x0000ffff) << 16;
}

static void codec_h264_frames_ready(struct amvdec_session *sess, u32 status)
{
	struct amvdec_core *core = sess->core;
	int error_count;
	int num_frames;
	int i;

	if (sess->fast_output_enable)
		dev_dbg(core->dev, "H.264 fast output mode: 0x%x\n",
			sess->fast_output_enable);

	error_count = amvdec_read_dos(core, AV_SCRATCH_D);
	num_frames = (status >> NUM_FRAMES_BIT) & NUM_FRAMES_MASK;
	if (error_count) {
		dev_warn(core->dev,
			 "decoder error(s) happened, count %d\n", error_count);
		amvdec_write_dos(core, AV_SCRATCH_D, 0);
	}

	for (i = 0; i < num_frames; i++) {
		u32 frame_status = amvdec_read_dos(core, AV_SCRATCH_1 + i * 4);
		u32 buffer_index = frame_status & BUF_IDX_MASK;
		u32 pic_struct = (frame_status >> PIC_STRUCT_BIT) &
				 PIC_STRUCT_MASK;
		u32 offset = (frame_status >> OFFSET_BIT) & OFFSET_MASK;
		u32 field = V4L2_FIELD_NONE;

		/*
		 * A buffer decode error means it was decoded,
		 * but part of the picture will have artifacts.
		 * Typical reason is a temporarily corrupted bitstream.
		 * error_proc_policy bit 0 (send_error_frame_flag) controls
		 * whether error frames are output to userspace or dropped.
		 * BSP vmh264.c: send_error_frame_flag (bit 0).
		 */
		if (frame_status & ERROR_FLAG) {
			dev_dbg(core->dev, "Buffer %d decode error\n",
				buffer_index);
			if (!(sess->error_proc_policy & BIT(0))) {
				/* Drop error frame -- return DPB slot to firmware
				 * without delivering the buffer to userspace.
				 * Use the same recycle path as normal buffer return.
				 */
				codec_h264_recycle(core, buffer_index);
				continue;
			}
		}

		if (pic_struct == PIC_TOP_BOT)
			field = V4L2_FIELD_INTERLACED_TB;
		else if (pic_struct == PIC_BOT_TOP)
			field = V4L2_FIELD_INTERLACED_BT;

		offset |= get_offset_msb(core, i);
		amvdec_dst_buf_done_idx(sess, buffer_index, offset, field);
	}

	/* Frames delivered = progress; reset watchdog */
	{
		struct codec_h264 *h264 = sess->priv;

		codec_h264_start_watchdog(h264);
	}
}

static irqreturn_t codec_h264_threaded_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	u32 status;
	u32 size;
	u8 cmd;

	/* Session was aborted -- skip all firmware interaction.
	 * The firmware may still fire IRQs until vdec_poweroff() halts it
	 * during STREAMOFF.  Without this check, the ISR would re-enter
	 * codec_h264_src_change() or codec_h264_frames_ready() which access
	 * VB2 buffers that may be in teardown.
	 */
	if (READ_ONCE(sess->aborted))
		return IRQ_HANDLED;

	status = amvdec_read_dos(core, AV_SCRATCH_0);
	cmd = status & CMD_MASK;

	switch (cmd) {
	case CMD_SRC_CHANGE:
		atomic_inc(&core->debugfs.irq.h264_src_change);
		codec_h264_src_change(sess);
		break;
	case CMD_FRAMES_READY:
		atomic_inc(&core->debugfs.irq.h264_frames_ready);
		codec_h264_frames_ready(sess, status);
		break;
	case CMD_FATAL_ERROR:
		dev_err(core->dev, "H.264 decoder fatal error\n");
		goto abort;
	case CMD_BAD_WIDTH: {
		struct codec_h264 *h264_bw = sess->priv;

		size = (amvdec_read_dos(core, AV_SCRATCH_1) + 1) * 16;
		dev_info(core->dev,
			 "H.264 CMD_BAD_WIDTH: firmware reports width=%u (AV_SCRATCH_1=0x%08x)\n",
			 size, amvdec_read_dos(core, AV_SCRATCH_1));
		/*
		 * At 4096px the firmware may fire BAD_WIDTH instead of
		 * SRC_CHANGE.  If we have SPS dimensions, try to proceed
		 * via the normal src_change path.
		 */
		if (h264_bw && h264_bw->sps_mb_width > 0) {
			dev_info(core->dev,
				 "H.264 CMD_BAD_WIDTH: attempting SPS-based recovery (%ux%u px)\n",
				 h264_bw->sps_mb_width * 16,
				 h264_bw->sps_mb_height * 16);
			codec_h264_src_change(sess);
			/*
			 * codec_h264_src_change() triggers async DPB
			 * reallocation just like CMD_SRC_CHANGE.  Prevent
			 * the AV_SCRATCH_0 clear below from telling the
			 * firmware to continue before canvas/DPB is ready.
			 */
			cmd = CMD_SRC_CHANGE;
			break;
		}
		dev_err(core->dev, "Unsupported video width: %u\n", size);
		goto abort;
	}
	case CMD_BAD_HEIGHT:
		size = (amvdec_read_dos(core, AV_SCRATCH_1) + 1) * 16;
		dev_err(core->dev, "Unsupported video height: %u\n", size);
		goto abort;
	case 3: /* Pre-SRC_CHANGE: firmware requests canvas/DPB setup */
	case 0: /* No command / cleared status */
	case 9: /* Firmware-internal: DPB management complete (no action needed) */
		break;
	default:
		dev_info(core->dev, "Unexpected H264 ISR: %08X\n", cmd);
		break;
	}

	if (cmd && cmd != CMD_SRC_CHANGE)
		amvdec_write_dos(core, AV_SCRATCH_0, 0);

	/* Decoder has some SEI data for us ; ignore */
	if (amvdec_read_dos(core, AV_SCRATCH_J) & SEI_DATA_READY)
		amvdec_write_dos(core, AV_SCRATCH_J, 0);

	return IRQ_HANDLED;
abort:
	amvdec_abort(sess);
	return IRQ_HANDLED;
}

static irqreturn_t codec_h264_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);

	return IRQ_WAKE_THREAD;
}

struct amvdec_codec_ops codec_h264_ops = {
	.start = codec_h264_start,
	.stop = codec_h264_stop,
	.load_extended_firmware = codec_h264_load_extended_firmware,
	.pre_queue = codec_h264_pre_queue,
	.isr = codec_h264_isr,
	.threaded_isr = codec_h264_threaded_isr,
	.can_recycle = codec_h264_can_recycle,
	.recycle = codec_h264_recycle,
	.eos_sequence = codec_h264_eos_sequence,
	.resume = codec_h264_resume,
};

static int __init codec_h264_mod_init(void)
{
	return amvdec_register_codec(V4L2_PIX_FMT_H264, &codec_h264_ops);
}

static void __exit codec_h264_mod_exit(void)
{
	amvdec_unregister_codec(V4L2_PIX_FMT_H264, &codec_h264_ops);
}

module_init(codec_h264_mod_init);
module_exit(codec_h264_mod_exit);
MODULE_DESCRIPTION("Meson VDEC H.264 codec");
MODULE_LICENSE("GPL");
