// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#include <linux/gcd.h>
#include <crypto/sha2.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-contig.h>

#include "vdec_helpers.h"
#include "vdec_debugfs.h"
#include "dos_regs.h"
#include "hevc_regs.h"
#include "vdec_hevc.h"
#include "codec_hevc_common.h"

#define NUM_CANVAS_NV12 2
#define NUM_CANVAS_YUV420 3

/*
 * On G12A+, the VDEC has a per-canvas block-mode shadow register that must
 * be written after each meson_canvas_config().  Without this, the decoder
 * reads the wrong block layout and corrupts the Y plane.
 *
 * Bit 11: canvas_blk32_wr (write-enable)
 * Bit 10: canvas_blk32 (0 = linear, 1 = 32x32)
 * Bit  8: canvas_index_wr (write-enable)
 * Bits 7:0: canvas_index
 */
static void canvas_blk32_write(struct amvdec_core *core, u8 canvas_idx,
			       u32 blkmode)
{
	if (core->platform->revision >= VDEC_REVISION_G12A)
		amvdec_write_dos(core, VDEC_ASSIST_CANVAS_BLK32,
				 CANVAS_BLK32_WR | (blkmode << CANVAS_BLK32_BIT) |
				 CANVAS_INDEX_WR | canvas_idx);
}

/*
 * Canvas endianness.  SWAP64 (endian=7) is required on all platforms
 * for correct byte ordering in NV12/YUV420 output.
 * Tested: endian=0 on G12A+ corrupts UV planes (PSNR 7dB).
 *
 * For NV21 (VU byte order), the UV canvas uses endian=6 (SWAP64
 * without SWAP16).  SWAP64 (0x7) does a full 64-bit byte reversal
 * (swap16+swap32+swap64).  Removing SWAP16 (bit 0) leaves adjacent
 * byte pairs unswapped, effectively swapping U and V within each
 * interleaved UV pair.
 */
static u32 canvas_endian(struct amvdec_core *core)
{
	return MESON_CANVAS_ENDIAN_SWAP64;
}

static u32 canvas_endian_uv_swap(struct amvdec_core *core)
{
	return MESON_CANVAS_ENDIAN_SWAP64 ^ MESON_CANVAS_ENDIAN_SWAP16;
}

u32 amvdec_read_dos(struct amvdec_core *core, u32 reg)
{
	return readl_relaxed(core->dos_base + reg);
}
EXPORT_SYMBOL_GPL(amvdec_read_dos);

void amvdec_write_dos(struct amvdec_core *core, u32 reg, u32 val)
{
	writel_relaxed(val, core->dos_base + reg);
}
EXPORT_SYMBOL_GPL(amvdec_write_dos);

void amvdec_write_dos_bits(struct amvdec_core *core, u32 reg, u32 val)
{
	amvdec_write_dos(core, reg, amvdec_read_dos(core, reg) | val);
}
EXPORT_SYMBOL_GPL(amvdec_write_dos_bits);

void amvdec_clear_dos_bits(struct amvdec_core *core, u32 reg, u32 val)
{
	amvdec_write_dos(core, reg, amvdec_read_dos(core, reg) & ~val);
}
EXPORT_SYMBOL_GPL(amvdec_clear_dos_bits);

u32 amvdec_read_parser(struct amvdec_core *core, u32 reg)
{
	return readl_relaxed(core->esparser_base + reg);
}
EXPORT_SYMBOL_GPL(amvdec_read_parser);

void amvdec_write_parser(struct amvdec_core *core, u32 reg, u32 val)
{
	writel_relaxed(val, core->esparser_base + reg);
}
EXPORT_SYMBOL_GPL(amvdec_write_parser);

/*
 * FBC compressed body size per block.
 *
 * 8-bit: 3200 bytes per 64x32 block (mem_saving_mode, SAO_CTRL5 BIT(9)).
 * 10-bit: 4096 bytes per 64x32 block.
 *
 * The HEVC codec init path sets SAO_CTRL5 BIT(9) unconditionally, and
 * codec_hevc_setup_decode_head programs HEVC_CM_BODY_LENGTH with the
 * matching 3200/4096 constant per session.  The allocator must match
 * what the firmware writes or we corrupt the neighbor buffer.
 *
 * Saves ~20% FBC body bytes for 8-bit streams (the common case for
 * VP9 Profile 0 + HEVC Main); 10-bit callers (VP9 Profile 2 -- aborted
 * today per KI-058, HEVC Main10) pass is_10bit=true to get the
 * conservative sizing.
 */
u32 amvdec_am21c_body_size(u32 width, u32 height, bool is_10bit)
{
	u32 width_64 = ALIGN(width, 64) / 64;
	u32 height_32 = ALIGN(height, 32) / 32;
	u32 bytes_per_block = is_10bit ? SZ_4K : 3200;

	return bytes_per_block * width_64 * height_32;
}
EXPORT_SYMBOL_GPL(amvdec_am21c_body_size);

/* 32 bytes per 128x64 block */
u32 amvdec_am21c_head_size(u32 width, u32 height)
{
	u32 width_128 = ALIGN(width, 128) / 128;
	u32 height_64 = ALIGN(height, 64) / 64;

	return 32 * width_128 * height_64;
}
EXPORT_SYMBOL_GPL(amvdec_am21c_head_size);

u32 amvdec_am21c_size(u32 width, u32 height, bool is_10bit)
{
	return ALIGN(amvdec_am21c_body_size(width, height, is_10bit) +
		     amvdec_am21c_head_size(width, height), SZ_64K);
}
EXPORT_SYMBOL_GPL(amvdec_am21c_size);

static int canvas_alloc(struct amvdec_session *sess, u8 *canvas_id)
{
	int ret;

	if (sess->canvas_num >= MAX_CANVAS) {
		dev_err(sess->core->dev, "Reached max number of canvas\n");
		return -ENOMEM;
	}

	ret = meson_canvas_alloc(sess->core->canvas, canvas_id);
	if (ret)
		return ret;

	sess->canvas_alloc[sess->canvas_num++] = *canvas_id;
	return 0;
}

static int set_canvas_yuv420m(struct amvdec_session *sess,
			      struct vb2_buffer *vb, u32 width,
			      u32 height, u32 reg)
{
	struct amvdec_core *core = sess->core;
	u8 canvas_id[NUM_CANVAS_YUV420]; /* Y U V */
	dma_addr_t buf_paddr[NUM_CANVAS_YUV420]; /* Y U V */
	int ret, i;

	for (i = 0; i < NUM_CANVAS_YUV420; ++i) {
		ret = canvas_alloc(sess, &canvas_id[i]);
		if (ret)
			return ret;

		buf_paddr[i] =
		    vb2_dma_contig_plane_dma_addr(vb, i);
	}

	/* Y plane */
	meson_canvas_config(core->canvas, canvas_id[0], buf_paddr[0],
			    width, height, MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    canvas_endian(core));
	canvas_blk32_write(core, canvas_id[0], MESON_CANVAS_BLKMODE_LINEAR);

	/* U plane */
	meson_canvas_config(core->canvas, canvas_id[1], buf_paddr[1],
			    width / 2, height / 2, MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    canvas_endian(core));
	canvas_blk32_write(core, canvas_id[1], MESON_CANVAS_BLKMODE_LINEAR);

	/* V plane */
	meson_canvas_config(core->canvas, canvas_id[2], buf_paddr[2],
			    width / 2, height / 2, MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    canvas_endian(core));
	canvas_blk32_write(core, canvas_id[2], MESON_CANVAS_BLKMODE_LINEAR);

	amvdec_write_dos(core, reg,
			 ((canvas_id[2]) << 16) |
			 ((canvas_id[1]) << 8)  |
			 (canvas_id[0]));

	return 0;
}

static int set_canvas_nv12(struct amvdec_session *sess,
			   struct vb2_buffer *vb, u32 width,
			   u32 height, u32 reg, bool swap_uv)
{
	struct amvdec_core *core = sess->core;
	u8 canvas_id[NUM_CANVAS_NV12]; /* Y U/V */
	dma_addr_t buf_paddr;
	u32 uv_endian = swap_uv ? canvas_endian_uv_swap(core) :
				   canvas_endian(core);
	int ret, i;

	buf_paddr = vb2_dma_contig_plane_dma_addr(vb, 0);

	for (i = 0; i < NUM_CANVAS_NV12; ++i) {
		ret = canvas_alloc(sess, &canvas_id[i]);
		if (ret)
			return ret;
	}

	/* Y plane */
	meson_canvas_config(core->canvas, canvas_id[0], buf_paddr,
			    width, height, MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    canvas_endian(core));
	canvas_blk32_write(core, canvas_id[0], MESON_CANVAS_BLKMODE_LINEAR);

	/* U/V plane at offset width*height */
	meson_canvas_config(core->canvas, canvas_id[1],
			    buf_paddr + (width * height),
			    width, height / 2, MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    uv_endian);
	canvas_blk32_write(core, canvas_id[1], MESON_CANVAS_BLKMODE_LINEAR);

	amvdec_write_dos(core, reg,
			 ((canvas_id[1]) << 16) |
			 ((canvas_id[1]) << 8)  |
			 (canvas_id[0]));

	return 0;
}

static int set_canvas_nv12m(struct amvdec_session *sess,
			    struct vb2_buffer *vb, u32 width,
			    u32 height, u32 reg, bool swap_uv)
{
	struct amvdec_core *core = sess->core;
	u8 canvas_id[NUM_CANVAS_NV12]; /* Y U/V */
	dma_addr_t buf_paddr[NUM_CANVAS_NV12]; /* Y U/V */
	u32 uv_endian = swap_uv ? canvas_endian_uv_swap(core) :
				   canvas_endian(core);
	int ret, i;

	for (i = 0; i < NUM_CANVAS_NV12; ++i) {
		ret = canvas_alloc(sess, &canvas_id[i]);
		if (ret)
			return ret;

		buf_paddr[i] =
		    vb2_dma_contig_plane_dma_addr(vb, i);
	}

	/* Y plane */
	meson_canvas_config(core->canvas, canvas_id[0], buf_paddr[0],
			    width, height, MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    canvas_endian(core));
	canvas_blk32_write(core, canvas_id[0], MESON_CANVAS_BLKMODE_LINEAR);

	/* U/V plane */
	meson_canvas_config(core->canvas, canvas_id[1], buf_paddr[1],
			    width, height / 2, MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_LINEAR,
			    uv_endian);
	canvas_blk32_write(core, canvas_id[1], MESON_CANVAS_BLKMODE_LINEAR);

	amvdec_write_dos(core, reg,
			 ((canvas_id[1]) << 16) |
			 ((canvas_id[1]) << 8)  |
			 (canvas_id[0]));

	return 0;
}

int amvdec_set_canvases(struct amvdec_session *sess,
			u32 reg_base[], u32 reg_num[])
{
	struct v4l2_m2m_buffer *buf;
	u32 pixfmt = sess->pixfmt_cap;
	u32 width = ALIGN(sess->width, 64);
	u32 height = ALIGN(sess->height, 32);
	u32 reg_cur;
	u32 reg_num_cur = 0;
	u32 reg_base_cur = 0;
	int i = 0;
	int ret;

	v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
		if (!reg_base[reg_base_cur])
			return -EINVAL;

		reg_cur = reg_base[reg_base_cur] + reg_num_cur * 4;

		switch (pixfmt) {
		case V4L2_PIX_FMT_NV12:
			ret = set_canvas_nv12(sess, &buf->vb.vb2_buf, width,
					      height, reg_cur, false);
			if (ret)
				return ret;
			break;
		case V4L2_PIX_FMT_NV21:
			ret = set_canvas_nv12(sess, &buf->vb.vb2_buf, width,
					      height, reg_cur, true);
			if (ret)
				return ret;
			break;
		case V4L2_PIX_FMT_NV12M:
			ret = set_canvas_nv12m(sess, &buf->vb.vb2_buf, width,
					       height, reg_cur, false);
			if (ret)
				return ret;
			break;
		case V4L2_PIX_FMT_NV21M:
			ret = set_canvas_nv12m(sess, &buf->vb.vb2_buf, width,
					       height, reg_cur, true);
			if (ret)
				return ret;
			break;
		case V4L2_PIX_FMT_YUV420M:
			ret = set_canvas_yuv420m(sess, &buf->vb.vb2_buf, width,
						 height, reg_cur);
			if (ret)
				return ret;
			break;
		default:
			dev_err(sess->core->dev, "Unsupported pixfmt %08X\n",
				pixfmt);
			return -EINVAL;
		}

		reg_num_cur++;
		if (reg_num_cur >= reg_num[reg_base_cur]) {
			reg_base_cur++;
			reg_num_cur = 0;
		}

		sess->fw_idx_to_vb2_idx[i++] = buf->vb.vb2_buf.index;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(amvdec_set_canvases);

int amvdec_add_ts(struct amvdec_session *sess, u64 ts,
		  struct v4l2_timecode tc, u32 offset, u32 vbuf_flags)
{
	struct amvdec_timestamp *new_ts;
	unsigned long flags;

	new_ts = kzalloc(sizeof(*new_ts), GFP_KERNEL);
	if (!new_ts)
		return -ENOMEM;

	new_ts->ts = ts;
	new_ts->tc = tc;
	new_ts->offset = offset;
	new_ts->flags = vbuf_flags;

	spin_lock_irqsave(&sess->ts_spinlock, flags);
	list_add_tail(&new_ts->list, &sess->timestamps);
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(amvdec_add_ts);

void amvdec_remove_ts(struct amvdec_session *sess, u64 ts)
{
	struct amvdec_timestamp *tmp;
	unsigned long flags;

	spin_lock_irqsave(&sess->ts_spinlock, flags);
	list_for_each_entry(tmp, &sess->timestamps, list) {
		if (tmp->ts == ts) {
			list_del(&tmp->list);
			kfree(tmp);
			goto unlock;
		}
	}
	dev_warn(sess->core->dev_dec,
		 "Couldn't remove buffer with timestamp %llu from list\n", ts);

unlock:
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);
}
EXPORT_SYMBOL_GPL(amvdec_remove_ts);

static void dst_buf_done(struct amvdec_session *sess,
			 struct vb2_v4l2_buffer *vbuf,
			 u32 field, u64 timestamp,
			 struct v4l2_timecode timecode, u32 flags)
{
	struct device *dev = sess->core->dev_dec;
	u32 output_size = amvdec_get_output_size(sess);

	switch (sess->pixfmt_cap) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		vb2_set_plane_payload(&vbuf->vb2_buf, 0,
				      output_size + output_size / 2);
		break;
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV21M:
		vb2_set_plane_payload(&vbuf->vb2_buf, 0, output_size);
		vb2_set_plane_payload(&vbuf->vb2_buf, 1, output_size / 2);
		break;
	case V4L2_PIX_FMT_YUV420M:
		vb2_set_plane_payload(&vbuf->vb2_buf, 0, output_size);
		vb2_set_plane_payload(&vbuf->vb2_buf, 1, output_size / 4);
		vb2_set_plane_payload(&vbuf->vb2_buf, 2, output_size / 4);
		break;
	case V4L2_PIX_FMT_MESON_FBC08_SCATTER:
		vb2_set_plane_payload(&vbuf->vb2_buf, 0,
				      amvdec_am21c_size(sess->width,
							sess->height, false));
		break;
	case V4L2_PIX_FMT_MESON_FBC10_SCATTER:
		vb2_set_plane_payload(&vbuf->vb2_buf, 0,
				      amvdec_am21c_size(sess->width,
							sess->height, true));
		break;
	}

	vbuf->vb2_buf.timestamp = timestamp;
	vbuf->sequence = sess->sequence_cap++;
	vbuf->flags = flags;
	vbuf->timecode = timecode;

	if (sess->status == STATUS_NEEDS_RESUME) {
		/* Mark LAST for drained show frames during a source change */
		vbuf->flags |= V4L2_BUF_FLAG_LAST;
		sess->sequence_cap = 0;
	}

	dev_dbg(dev, "Buffer %u done, ts = %llu, flags = %08X\n",
		vbuf->vb2_buf.index, timestamp, flags);
	vbuf->field = field;
	v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_DONE);

	/* Buffer done probably means the vififo got freed */
	schedule_work(&sess->esparser_queue_work);
}

void amvdec_dst_buf_done(struct amvdec_session *sess,
			 struct vb2_v4l2_buffer *vbuf, u32 field)
{
	struct device *dev = sess->core->dev_dec;
	struct amvdec_timestamp *tmp;
	struct list_head *timestamps = &sess->timestamps;
	struct v4l2_timecode timecode;
	u64 timestamp;
	u32 vbuf_flags;
	unsigned long flags;

	spin_lock_irqsave(&sess->ts_spinlock, flags);
	if (list_empty(timestamps)) {
		dev_err(dev, "Buffer %u done but list is empty\n",
			vbuf->vb2_buf.index);

		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
		spin_unlock_irqrestore(&sess->ts_spinlock, flags);
		return;
	}

	tmp = list_first_entry(timestamps, struct amvdec_timestamp, list);
	timestamp = tmp->ts;
	timecode = tmp->tc;
	vbuf_flags = tmp->flags;
	list_del(&tmp->list);
	kfree(tmp);
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);

	dst_buf_done(sess, vbuf, field, timestamp, timecode, vbuf_flags);
	atomic_dec(&sess->esparser_queued_bufs);
	wake_up(&sess->esparser_queue_done_wq);
}
EXPORT_SYMBOL_GPL(amvdec_dst_buf_done);

void amvdec_dst_buf_done_ts(struct amvdec_session *sess,
			    struct vb2_v4l2_buffer *vbuf, u32 field,
			    u64 timestamp, struct v4l2_timecode timecode,
			    u32 flags)
{
	dst_buf_done(sess, vbuf, field, timestamp, timecode, flags);
}
EXPORT_SYMBOL_GPL(amvdec_dst_buf_done_ts);

void amvdec_dst_buf_done_offset(struct amvdec_session *sess,
				struct vb2_v4l2_buffer *vbuf,
				u32 offset, u32 field, bool allow_drop)
{
	struct device *dev = sess->core->dev_dec;
	struct amvdec_timestamp *match = NULL;
	struct amvdec_timestamp *tmp, *n;
	struct v4l2_timecode timecode = { 0 };
	u64 timestamp = 0;
	u32 vbuf_flags = 0;
	unsigned long flags;

	spin_lock_irqsave(&sess->ts_spinlock, flags);

	/* Look for our vififo offset to get the corresponding timestamp. */
	list_for_each_entry_safe(tmp, n, &sess->timestamps, list) {
		if (tmp->offset > offset) {
			/*
			 * Delete any record that remained unused for 32 match
			 * checks
			 */
			if (tmp->used_count++ >= 32) {
				list_del(&tmp->list);
				kfree(tmp);
			}
			break;
		}

		match = tmp;
	}

	if (!match && sess->wrap_count > 0) {
		/*
		 * VIFIFO wrap-around: esparser tracks offsets as logical
		 * monotonic values (raw_offset + wrap_count * vififo_size),
		 * but MREG_FRAME_OFFSET reports the raw ring-buffer position.
		 * Convert the raw offset to logical and retry the match.
		 */
		u32 logical = offset + sess->wrap_count * sess->vififo_size;

		list_for_each_entry_safe(tmp, n, &sess->timestamps, list) {
			if (tmp->offset > logical)
				break;
			match = tmp;
		}

		if (match)
			dev_dbg(dev,
				"Buffer %u: matched after wrap adjust (%08X -> %08X)\n",
				vbuf->vb2_buf.index, offset, logical);
	}

	if (!match) {
		dev_dbg(dev, "Buffer %u done but can't match offset (%08X)\n",
			vbuf->vb2_buf.index, offset);
		/*
		 * Last resort: fall back to FIFO order (oldest entry).
		 * MREG_FRAME_OFFSET reports the VLD read pointer which can
		 * lag the esparser write pointer stored in the timestamp list,
		 * especially at low bitrates.  The FIFO fallback preserves
		 * approximate PTS ordering when offset matching fails.
		 *
		 * The monotonic clamp below prevents backwards-PTS drops
		 * even if the fallback cascade assigns entries out of order.
		 */
		if (!list_empty(&sess->timestamps)) {
			match = list_first_entry(&sess->timestamps,
						 struct amvdec_timestamp, list);
		} else {
			/*
			 * Timestamp list exhausted.  Synthesize a monotonically
			 * increasing timestamp so userspace does not discard
			 * this frame as out-of-order (PTS=0 drop).
			 */
			dev_dbg(dev,
				"Buffer %u: ts list empty at offset %08X, using synthetic ts (last=%llu)\n",
				vbuf->vb2_buf.index, offset, sess->last_ts_done);
			timestamp = sess->last_ts_done + 1;
		}
	}

	if (match) {
		timestamp = match->ts;
		timecode = match->tc;
		vbuf_flags = match->flags;
		list_del(&match->list);
		kfree(match);
	}
	spin_unlock_irqrestore(&sess->ts_spinlock, flags);

	/*
	 * Clamp to monotonically increasing PTS.  FIFO fallback can assign
	 * an out-of-order entry (e.g. the last frame steals T_{N-1} after
	 * T_N was already delivered), producing a backwards PTS that causes
	 * userspace (ffmpeg) to drop the frame.  Clamping prevents this
	 * without affecting correctly-matched in-order frames.
	 */
	if (timestamp < sess->last_ts_done + 1)
		timestamp = sess->last_ts_done + 1;
	sess->last_ts_done = timestamp;

	dst_buf_done(sess, vbuf, field, timestamp, timecode, vbuf_flags);
	/*
	 * Decrement whether or not offset matched: the firmware produced a
	 * frame, consuming one queued source slot, regardless of whether we
	 * found a matching timestamp.  Without this, an offset mismatch on
	 * the last frame leaves esparser_queued_bufs stuck at 1, causing
	 * drain_work to time out after 5 seconds before signalling EOS.
	 */
	atomic_dec_if_positive(&sess->esparser_queued_bufs);
	wake_up(&sess->esparser_queue_done_wq);
}
EXPORT_SYMBOL_GPL(amvdec_dst_buf_done_offset);

void amvdec_dst_buf_done_idx(struct amvdec_session *sess,
			     u32 buf_idx, u32 offset, u32 field)
{
	struct vb2_v4l2_buffer *vbuf;

	vbuf = v4l2_m2m_dst_buf_remove_by_idx(sess->m2m_ctx,
					      sess->fw_idx_to_vb2_idx[buf_idx]);

	if (!vbuf) {
		/* Buffer is still in userspace -- defer delivery until
		 * it's re-queued.  The recycle thread will retry.
		 *
		 * This path is critical during H.264 drain: the EOS
		 * sequence flushes up to 16 reference frames from the
		 * DPB in a single ISR burst, but capture buffers are
		 * still in userspace being DQBUFed.  Without deferral,
		 * those frames are silently dropped (observed: 16/90
		 * frames delivered for 720p refs=16 bf=16 gop=300).
		 *
		 * Safe during teardown too: vdec_poweroff() halts the
		 * firmware (MPSR=0) before stopping the recycle thread,
		 * so no ISRs fire after halt.  Any frames deferred
		 * before halt are cleaned up by vdec_reset_pending_frames()
		 * in stop_streaming.
		 */
		struct amvdec_pending_frame *pf;

		pf = kmalloc(sizeof(*pf), GFP_ATOMIC);
		if (pf) {
			pf->buf_idx = buf_idx;
			pf->offset = offset;
			pf->field = field;
			spin_lock(&sess->pending_frames_lock);
			list_add_tail(&pf->list, &sess->pending_frames);
			spin_unlock(&sess->pending_frames_lock);
			wake_up_interruptible(&sess->recycle_wq);
			dev_dbg(sess->core->dev_dec,
				"Buffer %u deferred (in userspace)\n",
				buf_idx);
		}
		return;
	}

	if (offset != -1)
		amvdec_dst_buf_done_offset(sess, vbuf, offset, field, true);
	else
		amvdec_dst_buf_done(sess, vbuf, field);
}
EXPORT_SYMBOL_GPL(amvdec_dst_buf_done_idx);

void amvdec_flush_pending_frames(struct amvdec_session *sess)
{
	struct amvdec_pending_frame *pf, *n;
	struct vb2_v4l2_buffer *vbuf;
	LIST_HEAD(local);

	spin_lock(&sess->pending_frames_lock);
	list_splice_init(&sess->pending_frames, &local);
	spin_unlock(&sess->pending_frames_lock);

	list_for_each_entry_safe(pf, n, &local, list) {
		vbuf = v4l2_m2m_dst_buf_remove_by_idx(sess->m2m_ctx,
			sess->fw_idx_to_vb2_idx[pf->buf_idx]);
		if (vbuf) {
			if (pf->offset != -1)
				amvdec_dst_buf_done_offset(sess, vbuf,
					pf->offset, pf->field, true);
			else
				amvdec_dst_buf_done(sess, vbuf, pf->field);
			list_del(&pf->list);
			kfree(pf);
		} else {
			/* Still not available -- put back */
			spin_lock(&sess->pending_frames_lock);
			list_move_tail(&pf->list, &sess->pending_frames);
			spin_unlock(&sess->pending_frames_lock);
		}
	}
}
EXPORT_SYMBOL_GPL(amvdec_flush_pending_frames);

void amvdec_set_par_from_dar(struct amvdec_session *sess,
			     u32 dar_num, u32 dar_den)
{
	u32 div;

	sess->pixelaspect.numerator = sess->height * dar_num;
	sess->pixelaspect.denominator = sess->width * dar_den;
	div = gcd(sess->pixelaspect.numerator, sess->pixelaspect.denominator);
	sess->pixelaspect.numerator /= div;
	sess->pixelaspect.denominator /= div;
}
EXPORT_SYMBOL_GPL(amvdec_set_par_from_dar);

void amvdec_src_change(struct amvdec_session *sess, u32 width,
		       u32 height, u32 dpb_size)
{
	static const struct v4l2_event ev = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION };

	/* Reject bitstream dimensions exceeding platform limits.
	 * Without this, the driver allocates oversized capture buffers
	 * that exhaust CMA and crash the system.
	 */
	if (sess->fmt_out->max_width && width > sess->fmt_out->max_width) {
		dev_err(sess->core->dev,
			"Stream width %u exceeds max %u, aborting\n",
			width, sess->fmt_out->max_width);
		amvdec_abort(sess);
		return;
	}
	if (sess->fmt_out->max_height && height > sess->fmt_out->max_height) {
		dev_err(sess->core->dev,
			"Stream height %u exceeds max %u, aborting\n",
			height, sess->fmt_out->max_height);
		amvdec_abort(sess);
		return;
	}
	if (sess->fmt_out->min_width && width < sess->fmt_out->min_width) {
		dev_err(sess->core->dev,
			"Stream width %u below minimum %u, aborting\n",
			width, sess->fmt_out->min_width);
		amvdec_abort(sess);
		return;
	}

	/*
	 * CMA-budget product clamp for HEVC-engine codecs (HEVC, VP9).
	 * A stream with dimensions within per-dim caps but exceeding the
	 * probe-time CMA budget would call codec_hevc_setup_buffers with
	 * oversized requests, triggering __alloc_pages_noprof WARN + abort
	 * after the BUG print storm.  Abort up front with a clean error.
	 *
	 * PARTIAL: remove once IOMMU / scatter-gather removes the
	 * contiguous-CMA constraint (see cma-opt-sg in vp9/todo.md).
	 */
	/* Scatter-gather FBC body removes the contiguous-CMA budget as
	 * the binding constraint; let the allocation itself report failure
	 * (cleaner error path than a pre-emptive abort).
	 */
	if (sess->core->hevc_engine_cma_max_pixels && !codec_hevc_fbc_sg &&
	    (sess->fmt_out->pixfmt == V4L2_PIX_FMT_VP9 ||
	     sess->fmt_out->pixfmt == V4L2_PIX_FMT_HEVC) &&
	    (u64)width * height > sess->core->hevc_engine_cma_max_pixels) {
		dev_err(sess->core->dev,
			"Stream %ux%u (%llu MP) exceeds HEVC-engine CMA budget %llu MP; aborting\n",
			width, height, ((u64)width * height) >> 20,
			sess->core->hevc_engine_cma_max_pixels >> 20);
		amvdec_abort(sess);
		return;
	}

	v4l2_ctrl_s_ctrl(sess->ctrl_min_buf_capture, dpb_size);

	/* Fast resume: capture is already streaming and resolution unchanged.
	 *
	 * If dpb_size exceeds the allocated buffer count, the firmware can
	 * still decode -- it will manage the DPB with fewer slots, evicting
	 * older references as needed.  Without this path, the driver sets
	 * STATUS_NEEDS_RESUME and sends SRC_CHANGE, but ffmpeg (correctly)
	 * ignores the event since resolution is unchanged, leaving the
	 * decoder permanently stalled with zero output frames.
	 *
	 * Log a warning when clamping so the mismatch is visible in dmesg.
	 */
	if (sess->streamon_cap &&
	    sess->width == width &&
	    sess->height == height) {
		if (dpb_size > sess->num_dst_bufs)
			dev_info(sess->core->dev,
				 "DPB %u exceeds capture bufs %u, resuming with available buffers\n",
				 dpb_size, sess->num_dst_bufs);
		dev_dbg(sess->core->dev,
			"Res. unchanged (%ux%u), fast resume (dpb=%u bufs=%u)\n",
			width, height, dpb_size, sess->num_dst_bufs);
		sess->fmt_out->codec_ops->resume(sess);
		return;
	}

	sess->changed_format = 0;
	sess->width = width;
	sess->height = height;

	vdec_dbg_log_event(sess->core, VDEC_EVT_SRC_CHANGE, 0,
			   (width << 16) | height);

	/*
	 * Race condition fix: if CAPTURE is already streaming and
	 * the allocated buffers are sufficient for the new DPB size,
	 * call resume() directly. This handles the case where ffmpeg
	 * does CAPTURE STREAMON before the firmware fires SRC_CHANGE.
	 * In that scenario, the driver sets STATUS_NEEDS_RESUME and
	 * sends the event, but ffmpeg ignores it because capture is
	 * already streaming. resume() never fires, producing 0 frames.
	 *
	 * When streamon_cap=1 and dpb_size <= num_dst_bufs, the capture
	 * buffers are already allocated and large enough. Skip the event
	 * and resume decoding immediately.
	 */
	if (sess->streamon_cap && dpb_size <= sess->num_dst_bufs) {
		dev_info(sess->core->dev,
			 "Res. changed (%ux%u), cap already streaming, direct resume (dpb=%u bufs=%u)\n",
			 width, height, dpb_size, sess->num_dst_bufs);
		sess->fmt_out->codec_ops->resume(sess);
		WRITE_ONCE(sess->status, STATUS_RUNNING);
		return;
	}

	/*
	 * Capture already streaming but with fewer buffers than the DPB
	 * requires.  This happens when userspace starts CAPTURE with its
	 * default buffer count before the firmware reports the stream's
	 * actual reference frame count.  Sending a SRC_CHANGE event here
	 * is pointless -- userspace ignores it when capture is active --
	 * resulting in zero decoded frames.
	 *
	 * Clamp ctrl_min_buf_capture down to the available buffer count
	 * and resume with reduced DPB.  The firmware will decode correctly
	 * with fewer reference slots, though streams relying on many
	 * references may show quality degradation.  This is strictly
	 * better than producing zero frames.
	 */
	if (sess->streamon_cap && dpb_size > sess->num_dst_bufs) {
		dev_warn(sess->core->dev,
			 "Res. changed (%ux%u), cap already streaming with insufficient bufs (dpb=%u bufs=%u), clamping\n",
			 width, height, dpb_size, sess->num_dst_bufs);
		v4l2_ctrl_s_ctrl(sess->ctrl_min_buf_capture,
				 sess->num_dst_bufs);
		sess->fmt_out->codec_ops->resume(sess);
		WRITE_ONCE(sess->status, STATUS_RUNNING);
		return;
	}

	/*
	 * Set STATUS_NEEDS_RESUME before the log message and event.
	 * vdec_start_streaming() reads sess->status on STREAMON(CAPTURE)
	 * which can arrive within <1ms of the resolution change event.
	 * If status is still STATUS_INIT when STREAMON runs, the resume
	 * path is skipped and the decoder produces zero output frames.
	 */
	atomic_set(&sess->resumed, 0);
	WRITE_ONCE(sess->status, STATUS_NEEDS_RESUME);

	dev_info(sess->core->dev,
		 "Res. changed (%ux%u), DPB %u: cap=%d w=%u/%u h=%u/%u bufs=%u\n",
		 width, height, dpb_size, sess->streamon_cap,
		 sess->width, width, sess->height, height,
		 sess->num_dst_bufs);
	v4l2_event_queue_fh(&sess->fh, &ev);

	/*
	 * If capture is already streaming with enough buffers, resume
	 * directly.  This handles the race where userspace does
	 * CAPTURE STREAMON before firmware fires SRC_CHANGE:
	 * vdec_start_streaming() saw STATUS_INIT (not NEEDS_RESUME)
	 * so it skipped the resume path, and userspace ignores the
	 * SRC_CHANGE event since capture is already active.  Without
	 * this, the decoder produces zero output frames.
	 *
	 * The atomic_cmpxchg on sess->resumed ensures only ONE path
	 * (this or vdec_start_streaming) calls resume(), preventing
	 * double-resume corruption.
	 */
	if (sess->streamon_cap && dpb_size <= sess->num_dst_bufs &&
	    atomic_cmpxchg(&sess->resumed, 0, 1) == 0) {
		dev_info(sess->core->dev,
			 "Cap already streaming, direct resume (dpb=%u bufs=%u)\n",
			 dpb_size, sess->num_dst_bufs);
		sess->fmt_out->codec_ops->resume(sess);
		WRITE_ONCE(sess->status, STATUS_RUNNING);
	}
}
EXPORT_SYMBOL_GPL(amvdec_src_change);

void amvdec_abort(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	vdec_dbg_log_event(core, VDEC_EVT_ABORT, 0, 0);

	dev_info(core->dev, "Aborting decoding session!\n");

	/*
	 * Set aborted flag BEFORE halting firmware so the recycle thread
	 * and ISR handlers see it and skip further buffer access.
	 */
	WRITE_ONCE(sess->aborted, 1);
	smp_wmb();

	/*
	 * Halt the firmware immediately to prevent stray DMA writes
	 * and further ISRs.  For unsupported profiles (e.g. H.264
	 * High10) the firmware may perform DMA with mis-programmed
	 * parameters.  MPSR=0 halts the processor; mask=0 prevents
	 * new IRQs.  The aborted flag above ensures the recycle thread
	 * bails out if it was already awake.
	 */
	if (sess->fmt_out->vdec_ops == &vdec_hevc_ops) {
		amvdec_write_dos(core, HEVC_MPSR, 0);
		amvdec_write_dos(core, HEVC_ASSIST_MBOX1_MASK, 0);
	} else {
		amvdec_write_dos(core, MPSR, 0);
		amvdec_write_dos(core, CPSR, 0);
		amvdec_write_dos(core, ASSIST_MBOX1_MASK, 0);
	}

	/* Clear cur_sess so any in-flight threaded ISR bails out. */
	core->cur_sess = NULL;

	vb2_queue_error(&sess->m2m_ctx->cap_q_ctx.q);
	vb2_queue_error(&sess->m2m_ctx->out_q_ctx.q);
}
EXPORT_SYMBOL_GPL(amvdec_abort);

/*
 * Scratch buffer size: must fit one decoded frame at max VDEC_1 resolution.
 * VDEC_1 codecs support up to 1920x1088 NV12 = ~3.1MB.
 * Use 4MB to cover alignment and block-mode overhead.
 */
#define SCRATCH_SIZE SZ_4M

int amvdec_scratch_init(struct amvdec_core *core)
{
	int ret, i;

	core->scratch_vaddr = dma_alloc_coherent(core->dev, SCRATCH_SIZE,
						 &core->scratch_paddr,
						 GFP_KERNEL);
	if (!core->scratch_vaddr)
		return -ENOMEM;

	for (i = 0; i < 3; i++) {
		ret = meson_canvas_alloc(core->canvas,
					 &core->scratch_canvas[i]);
		if (ret)
			goto free_canvas;
	}

	/*
	 * Configure Y/U/V canvases pointing to the scratch buffer.
	 * Use large stride and height so any firmware resolution fits.
	 * BLKMODE_32x32 matches what VDEC_1 firmware expects.
	 */
	meson_canvas_config(core->canvas, core->scratch_canvas[0],
			    core->scratch_paddr, 1920, 1088,
			    MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_32x32,
			    MESON_CANVAS_ENDIAN_SWAP64);
	meson_canvas_config(core->canvas, core->scratch_canvas[1],
			    core->scratch_paddr, 960, 544,
			    MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_32x32,
			    MESON_CANVAS_ENDIAN_SWAP64);
	meson_canvas_config(core->canvas, core->scratch_canvas[2],
			    core->scratch_paddr, 960, 544,
			    MESON_CANVAS_WRAP_NONE,
			    MESON_CANVAS_BLKMODE_32x32,
			    MESON_CANVAS_ENDIAN_SWAP64);

	return 0;

free_canvas:
	while (i--)
		meson_canvas_free(core->canvas, core->scratch_canvas[i]);
	dma_free_coherent(core->dev, SCRATCH_SIZE,
			  core->scratch_vaddr, core->scratch_paddr);
	core->scratch_vaddr = NULL;
	return ret;
}
EXPORT_SYMBOL_GPL(amvdec_scratch_init);

void amvdec_scratch_free(struct amvdec_core *core)
{
	int i;

	if (!core->scratch_vaddr)
		return;

	for (i = 0; i < 3; i++)
		meson_canvas_free(core->canvas, core->scratch_canvas[i]);

	dma_free_coherent(core->dev, SCRATCH_SIZE,
			  core->scratch_vaddr, core->scratch_paddr);
	core->scratch_vaddr = NULL;
}
EXPORT_SYMBOL_GPL(amvdec_scratch_free);

void amvdec_fill_scratch_canvases(struct amvdec_core *core)
{
	u32 packed;
	int i;

	packed = ((u32)core->scratch_canvas[2] << 16) |
		 ((u32)core->scratch_canvas[1] << 8) |
		 (u32)core->scratch_canvas[0];

	/* Fill AV_SCRATCH_0..L (22 regs) — covers MPEG12, MPEG4, MJPEG */
	for (i = 0; i < 22; i++)
		amvdec_write_dos(core, AV_SCRATCH_0 + i * 4, packed);

	/* Fill ANC0_CANVAS_ADDR + 0..31 — covers H.264 (32 DPB slots) */
	for (i = 0; i < 32; i++)
		amvdec_write_dos(core, ANC0_CANVAS_ADDR + i * 4, packed);
}
EXPORT_SYMBOL_GPL(amvdec_fill_scratch_canvases);

#define MAX_REF_PIC_NUM 24

void amvdec_fill_scratch_hevc(struct amvdec_core *core)
{
	int i;

	/*
	 * GXL+ path: ANC2AXI_TBL_DATA takes address >> 5.
	 * Fill all reference slots with the scratch buffer address.
	 */
	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR,
			 ANC2AXI_CONF_AUTO_INC);
	for (i = 0; i < MAX_REF_PIC_NUM; i++)
		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
				 core->scratch_paddr >> 5);
	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, 1);

	/* Zero out canvas data — GXL+ uses AXI table, not canvas */
	amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1);
	for (i = 0; i < 32; i++)
		amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, 0);
}
EXPORT_SYMBOL_GPL(amvdec_fill_scratch_hevc);

/**
 * amvdec_fw_identify - Compute SHA256 of a firmware buffer and look it up.
 *
 * See vdec_helpers.h for the full return-value contract:
 *   valid pointer  -- matched entry, caller continues
 *   ERR_PTR(-ENOENT) -- table non-empty but hash not found; caller MUST refuse
 *   NULL           -- no table; caller continues (backward compat)
 */
const struct amvdec_fw_version *
amvdec_fw_identify(struct device *dev, const char *fwname,
		   const u8 *data, size_t size,
		   const struct amvdec_fw_version *table, size_t n)
{
	u8 digest[SHA256_DIGEST_SIZE];
	const struct amvdec_fw_version *match = NULL;
	const struct amvdec_fw_version *latest = NULL;
	size_t i;

	sha256(data, size, digest);

	if (!table || !n) {
		dev_info(dev,
			 "meson-vdec fw: %s size=%zu sha256=%02x%02x%02x%02x%02x%02x%02x%02x (no version table)\n",
			 fwname, size,
			 digest[0], digest[1], digest[2], digest[3],
			 digest[4], digest[5], digest[6], digest[7]);
		return NULL;
	}

	for (i = 0; i < n; i++) {
		if (table[i].size == size &&
		    memcmp(table[i].sha256, digest, SHA256_DIGEST_SIZE) == 0) {
			match = &table[i];
			break;
		}
		if (table[i].latest)
			latest = &table[i];
	}

	if (!match) {
		/*
		 * Static-vs-dynamic firmware split:
		 *
		 * BSP / upstream firmware (filename does NOT start with `libre_`,
		 * e.g. `gxl_h264.bin`, `g12a_hevc_mmu.bin`) ships from
		 * linux-firmware or vendor BSP releases.  SHAs are static --
		 * a hash miss means the operator loaded the wrong file or a
		 * fuzzed binary, so refuse to load and force them to verify
		 * the firmware catalog.  Original 2026-04-16 HEVC G12B incident
		 * (wrong firmware silently used, multi-strike investigation).
		 *
		 * libre stateless firmware (filename starts with `libre_`,
		 * e.g. `libre_sm1_h264.bin`, `libre_sm1_vdec1.bin`) is built
		 * fresh from the amlogic codec/decoder/stateless/firmware/
		 * source tree on every M4.x iteration.  SHA changes per build.
		 * Refusing to load every dev iteration would force a kernel
		 * rebuild + reload cycle for every firmware tweak (~15 min)
		 * and gate development on driver-side bookkeeping.  Instead
		 * warn-and-continue with NULL fw_match: caller proceeds with
		 * no quirks (libre dev firmware is the test subject; quirks
		 * are pinned only after a libre version is finalized).
		 *
		 * Per /codec-stateless skill rule 1a (revised 2026-04-23
		 * after this contradiction surfaced) the libre-permissive
		 * carve-out is the documented behavior.
		 */
		if (fwname && strstr(fwname, "libre_")) {
			dev_warn(dev,
				 "meson-vdec fw: %s size=%zu sha256=%02x%02x%02x%02x%02x%02x%02x%02x -- unrecognized libre firmware (dev mode); continuing without quirks; add to fw_versions[] when stable\n",
				 fwname, size,
				 digest[0], digest[1], digest[2], digest[3],
				 digest[4], digest[5], digest[6], digest[7]);
			return NULL;
		}

		dev_err(dev,
			"meson-vdec fw: %s size=%zu sha256=%02x%02x%02x%02x%02x%02x%02x%02x -- unrecognized firmware, please report\n",
			fwname, size,
			digest[0], digest[1], digest[2], digest[3],
			digest[4], digest[5], digest[6], digest[7]);
		dev_err(dev,
			"meson-vdec fw: %s refusing to load unrecognized firmware\n",
			fwname);
		return ERR_PTR(-ENOENT);
	}

	dev_info(dev,
		 "meson-vdec fw: %s label=\"%s\" provenance=\"%s\" size=%zu sha256=%02x%02x%02x%02x%02x%02x%02x%02x quirks=0x%x\n",
		 fwname, match->label, match->provenance, match->size,
		 digest[0], digest[1], digest[2], digest[3],
		 digest[4], digest[5], digest[6], digest[7],
		 match->quirks);

	if (!match->latest && latest) {
		dev_warn(dev,
			 "meson-vdec fw: %s WARNING: firmware is not latest (label=\"%s\", latest=\"%s\"); update linux-firmware for bug fixes\n",
			 fwname, match->label, latest->label);
	}

	return match;
}
EXPORT_SYMBOL_GPL(amvdec_fw_identify);
