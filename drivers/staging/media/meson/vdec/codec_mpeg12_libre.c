// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Libre Computer
 * Author: Da Xue <da@libre.computer>
 *
 * Stateless MPEG-1/2 decoder using libre_mpeg12.bin firmware.
 *
 * Unlike codec_mpeg12_stateless which synthesizes MPEG-2 headers and feeds
 * them to the stock BSP firmware, this codec writes picture parameters
 * directly to scratch registers and uses a custom firmware that skips
 * all bitstream header parsing.
 *
 * Firmware protocol:
 *   Driver writes params to AV_SCRATCH_G..L, then sets AV_SCRATCH_9 = 1.
 *   Firmware decodes one frame, writes AV_SCRATCH_9 = 2, fires MBOX IRQ.
 *   Firmware loops back to wait for next AV_SCRATCH_9 = 1.
 */

#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-v4l2.h>

#include "codec_mpeg12_libre.h"
#include "dos_regs.h"
#include "vdec_helpers.h"

#define SIZE_WORKSPACE		SZ_128K
#define WORKSPACE_OFFSET	(5 * SZ_1K)

/* Scratch register assignments (must match libre_mpeg12.asm) */
#define LIBRE_REG_CMD		AV_SCRATCH_9	/* command/status */
#define LIBRE_REG_CO_MV		AV_SCRATCH_B	/* workspace DMA addr */
#define LIBRE_REG_OFFSET	AV_SCRATCH_D	/* VIFIFO offset (output) */
#define LIBRE_REG_FRAME_INFO	AV_SCRATCH_E	/* frame info (output) */
#define LIBRE_REG_ERROR		AV_SCRATCH_F	/* error code (output) */
#define LIBRE_REG_PIC_INFO	AV_SCRATCH_G	/* PIC_INFO packed word */
#define LIBRE_REG_FWD_CANVAS	AV_SCRATCH_H	/* forward ref canvas */
#define LIBRE_REG_BWD_CANVAS	AV_SCRATCH_I	/* backward ref canvas */
#define LIBRE_REG_OUT_CANVAS	AV_SCRATCH_J	/* output canvas */
#define LIBRE_REG_WIDTH		AV_SCRATCH_K	/* horizontal_size */
#define LIBRE_REG_HEIGHT	AV_SCRATCH_L	/* vertical_size */

/* AV_SCRATCH_9 protocol values */
#define LIBRE_CMD_IDLE		0
#define LIBRE_CMD_DECODE	1
#define LIBRE_CMD_DONE		2

/* LIBRE_REG_FRAME_INFO bit definitions */
#define LIBRE_FRAME_PROGRESSIVE	BIT(31)

struct codec_mpeg12_libre {
	void	  *workspace_vaddr;
	dma_addr_t workspace_paddr;
};

static u32 build_pic_info(const struct v4l2_ctrl_mpeg2_picture *pic)
{
	u32 info = 0;

	info |= pic->picture_coding_type & 0x7;
	info |= (pic->picture_structure & 0x3) << 3;
	info |= (pic->intra_dc_precision & 0x3) << 5;
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_PROGRESSIVE)
		info |= BIT(7);
	info |= (pic->f_code[0][0] & 0xf) << 8;
	info |= (pic->f_code[0][1] & 0xf) << 12;
	info |= (pic->f_code[1][0] & 0xf) << 16;
	info |= (pic->f_code[1][1] & 0xf) << 20;
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_ALT_SCAN)
		info |= BIT(24);
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_INTRA_VLC)
		info |= BIT(25);
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_Q_SCALE_TYPE)
		info |= BIT(26);
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_CONCEALMENT_MV)
		info |= BIT(27);
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_FRAME_PRED_DCT)
		info |= BIT(28);
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_TOP_FIELD_FIRST)
		info |= BIT(31);

	return info;
}

/*
 * Find the canvas ID for a capture buffer identified by timestamp.
 * Canvas IDs are written to AV_SCRATCH_0..7 by amvdec_set_canvases().
 */
static u32 get_ref_canvas(struct amvdec_session *sess, u64 ts)
{
	struct vb2_queue *cap_q = &sess->m2m_ctx->cap_q_ctx.q;
	struct vb2_buffer *vb;
	int i;

	vb = vb2_find_buffer(cap_q, ts);
	if (!vb)
		return amvdec_read_dos(sess->core, AV_SCRATCH_0);

	/* Map vb2 index to firmware index via fw_idx_to_vb2_idx */
	for (i = 0; i < sess->num_dst_bufs; i++) {
		if (sess->fw_idx_to_vb2_idx[i] == vb->index)
			return amvdec_read_dos(sess->core,
					       AV_SCRATCH_0 + i * 4);
	}

	return amvdec_read_dos(sess->core, AV_SCRATCH_0);
}

/*
 * Get the canvas ID for a capture buffer by its vb2 index.
 */
static u32 get_dst_canvas(struct amvdec_session *sess, u32 vb2_idx)
{
	int i;

	for (i = 0; i < sess->num_dst_bufs; i++) {
		if (sess->fw_idx_to_vb2_idx[i] == vb2_idx)
			return amvdec_read_dos(sess->core,
					       AV_SCRATCH_0 + i * 4);
	}

	return amvdec_read_dos(sess->core, AV_SCRATCH_0);
}

/*
 * pre_queue callback: write per-frame parameters to scratch registers
 * and signal firmware to decode. Called from esparser before writing
 * slice data to VIFIFO.
 */
static int codec_mpeg12_libre_pre_queue(struct amvdec_session *sess,
					struct vb2_buffer *vb)
{
	struct amvdec_core *core = sess->core;
	struct vb2_v4l2_buffer *dst_buf;
	struct v4l2_ctrl *ctrl;
	const struct v4l2_ctrl_mpeg2_picture *pic;
	u32 pic_info, out_canvas;
	u32 fwd_canvas = 0, bwd_canvas = 0;

	/* vb is the OUTPUT (source) buffer. Get the CAPTURE (dest) buffer
	 * from the m2m context for canvas lookup.
	 */
	dst_buf = v4l2_m2m_next_dst_buf(sess->m2m_ctx);
	if (!dst_buf) {
		dev_err(core->dev, "mpeg12 libre: no dst buffer\n");
		return -EINVAL;
	}

	ctrl = v4l2_ctrl_find(&sess->ctrl_handler,
			      V4L2_CID_STATELESS_MPEG2_PICTURE);
	if (!ctrl) {
		dev_err(core->dev, "mpeg12 libre: no picture control\n");
		return -EINVAL;
	}

	pic = ctrl->p_cur.p;
	pic_info = build_pic_info(pic);

	/* Output canvas for this frame's destination buffer */
	out_canvas = get_dst_canvas(sess, dst_buf->vb2_buf.index);

	/* Reference canvases */
	if (pic->picture_coding_type == V4L2_MPEG2_PIC_CODING_TYPE_P ||
	    pic->picture_coding_type == V4L2_MPEG2_PIC_CODING_TYPE_B)
		fwd_canvas = get_ref_canvas(sess, pic->forward_ref_ts);

	if (pic->picture_coding_type == V4L2_MPEG2_PIC_CODING_TYPE_B)
		bwd_canvas = get_ref_canvas(sess, pic->backward_ref_ts);

	/* Write parameters to scratch registers */
	amvdec_write_dos(core, LIBRE_REG_PIC_INFO, pic_info);
	amvdec_write_dos(core, LIBRE_REG_FWD_CANVAS, fwd_canvas);
	amvdec_write_dos(core, LIBRE_REG_BWD_CANVAS, bwd_canvas);
	amvdec_write_dos(core, LIBRE_REG_OUT_CANVAS, out_canvas);
	amvdec_write_dos(core, LIBRE_REG_WIDTH, sess->width);
	amvdec_write_dos(core, LIBRE_REG_HEIGHT, sess->height);
	amvdec_write_dos(core, LIBRE_REG_ERROR, 0);

	/* Signal firmware to start decoding */
	amvdec_write_dos(core, LIBRE_REG_CMD, LIBRE_CMD_DECODE);

	return 0;
}

static int codec_mpeg12_libre_start(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct codec_mpeg12_libre *mpeg12;
	int ret;

	mpeg12 = kzalloc(sizeof(*mpeg12), GFP_KERNEL);
	if (!mpeg12)
		return -ENOMEM;

	mpeg12->workspace_vaddr = dma_alloc_coherent(core->dev, SIZE_WORKSPACE,
						     &mpeg12->workspace_paddr,
						     GFP_KERNEL);
	if (!mpeg12->workspace_vaddr) {
		ret = -ENOMEM;
		goto free_mpeg12;
	}

	/* Full IQIDCT/MC/VLD reset from BSP */
	{
		u32 save_vld = amvdec_read_dos(core, POWER_CTL_VLD);

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

		amvdec_write_dos(core, POWER_CTL_VLD, save_vld);
	}

	amvdec_write_dos(core, POWER_CTL_VLD, POWER_CTL_VLD_ENABLE);
	amvdec_write_dos(core, LIBRE_REG_CO_MV,
			 mpeg12->workspace_paddr + WORKSPACE_OFFSET);

	amvdec_write_dos(core, MPEG1_2_REG, 0);
	amvdec_write_dos(core, PSCALE_CTRL, 0);
	amvdec_write_dos(core, PIC_HEAD_INFO, PIC_HEAD_INFO_INIT);
	amvdec_write_dos(core, M4_CONTROL_REG, 0);
	amvdec_write_dos(core, LIBRE_REG_CMD, LIBRE_CMD_IDLE);
	amvdec_write_dos(core, LIBRE_REG_ERROR, 0);

	/* Set up canvases before firmware starts */
	ret = amvdec_set_canvases(sess, (u32[]){ AV_SCRATCH_0, 0 },
				  (u32[]){ 8, 0 });
	if (ret) {
		dev_err(core->dev, "mpeg12 libre: canvas setup failed: %d\n",
			ret);
		goto free_workspace;
	}

	sess->keyframe_found = 1;
	sess->priv = mpeg12;

	return 0;

free_workspace:
	dma_free_coherent(core->dev, SIZE_WORKSPACE,
			  mpeg12->workspace_vaddr,
			  mpeg12->workspace_paddr);
free_mpeg12:
	kfree(mpeg12);
	return ret;
}

static int codec_mpeg12_libre_stop(struct amvdec_session *sess)
{
	struct codec_mpeg12_libre *mpeg12 = sess->priv;
	struct amvdec_core *core = sess->core;

	/*
	 * codec_ops->stop is called from BOTH vdec.c:613 (codec
	 * unwind) and vdec_1.c:649 (engine unwind, gated on
	 * sess->priv non-NULL). Without NULLing sess->priv after
	 * kfree, the second call double-frees the workspace
	 * dma_free_coherent -> CMA double-free WARN, panics on
	 * hung_task_panic boards. Pattern from codec_h264_mvc.c:1466.
	 */
	if (mpeg12->workspace_vaddr) {
		dma_free_coherent(core->dev, SIZE_WORKSPACE,
				  mpeg12->workspace_vaddr,
				  mpeg12->workspace_paddr);
		mpeg12->workspace_vaddr = NULL;
	}

	kfree(mpeg12);
	sess->priv = NULL;
	return 0;
}

static irqreturn_t codec_mpeg12_libre_isr(struct amvdec_session *sess)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t codec_mpeg12_libre_threaded_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	u32 cmd, error, frame_info, offset;
	u32 buffer_index;
	u32 field = V4L2_FIELD_NONE;
	int i;

	amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);

	cmd = amvdec_read_dos(core, LIBRE_REG_CMD);
	if (cmd != LIBRE_CMD_DONE)
		return IRQ_HANDLED;

	error = amvdec_read_dos(core, LIBRE_REG_ERROR);
	if (error) {
		dev_err(core->dev, "mpeg12 libre: firmware error %u\n", error);
		amvdec_abort(sess);
		return IRQ_HANDLED;
	}

	frame_info = amvdec_read_dos(core, LIBRE_REG_FRAME_INFO);
	offset = amvdec_read_dos(core, LIBRE_REG_OFFSET);

	/*
	 * The firmware writes the output canvas ID in frame_info[23:0].
	 * Map it back to a firmware buffer index by scanning AV_SCRATCH_0..7.
	 */
	buffer_index = 0;
	{
		u32 canvas = frame_info & 0xffffff;

		for (i = 0; i < sess->num_dst_bufs; i++) {
			u32 reg_canvas = amvdec_read_dos(core,
							  AV_SCRATCH_0 + i * 4);
			if ((reg_canvas & 0xffffff) == canvas) {
				buffer_index = i;
				break;
			}
		}
	}

	if (!(frame_info & LIBRE_FRAME_PROGRESSIVE)) {
		u32 pic_struct = (frame_info >> 16) & 0x3;

		if (pic_struct == 1)
			field = V4L2_FIELD_INTERLACED_TB;
		else if (pic_struct == 2)
			field = V4L2_FIELD_INTERLACED_BT;
	}

	dev_dbg(core->dev,
		"mpeg12 libre: done buf=%u offset=%x info=%x\n",
		buffer_index, offset, frame_info);

	amvdec_dst_buf_done_idx(sess, buffer_index, offset, field);

	v4l2_m2m_job_finish(sess->m2m_dev, sess->m2m_ctx);
	return IRQ_HANDLED;
}

static const u8 eos_sequence[SZ_1K] = { 0x00, 0x00, 0x01, 0xB7 };

static const u8 *codec_mpeg12_libre_eos_sequence(u32 *len)
{
	*len = ARRAY_SIZE(eos_sequence);
	return eos_sequence;
}

struct amvdec_codec_ops codec_mpeg12_libre_ops = {
	.start = codec_mpeg12_libre_start,
	.stop = codec_mpeg12_libre_stop,
	.pre_queue = codec_mpeg12_libre_pre_queue,
	.isr = codec_mpeg12_libre_isr,
	.threaded_isr = codec_mpeg12_libre_threaded_isr,
	.eos_sequence = codec_mpeg12_libre_eos_sequence,
};

static int __init codec_mpeg12_libre_mod_init(void)
{
	return amvdec_register_codec(V4L2_PIX_FMT_MPEG2_SLICE,
				     &codec_mpeg12_libre_ops);
}

static void __exit codec_mpeg12_libre_mod_exit(void)
{
	amvdec_unregister_codec(V4L2_PIX_FMT_MPEG2_SLICE,
				&codec_mpeg12_libre_ops);
}

module_init(codec_mpeg12_libre_mod_init);
module_exit(codec_mpeg12_libre_mod_exit);
MODULE_DESCRIPTION("Meson VDEC MPEG-1/2 libre stateless codec");
MODULE_LICENSE("GPL");
