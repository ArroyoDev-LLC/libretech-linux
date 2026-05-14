// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/of_irq.h>
#include <linux/kthread.h>
#include <linux/thermal.h>
#include <linux/mm.h>

#include "codec_hevc_common.h"
#include <linux/log2.h>
#include <media/media-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-dma-contig.h>

#include "vdec.h"
#include "vdec_debugfs.h"
#include "esparser.h"
#include "dos_regs.h"
#include "hevc_regs.h"
#include "vdec_helpers.h"
#include "vdec_diag.h"

/*
 * Custom V4L2 control: H.264 I-frame-only decode mode.
 * Values: 0 = normal decode (all frames), 1 = decode I-frames only.
 * When enabled, the firmware skips P/B frame decode via AV_SCRATCH_F bit 6.
 * Useful for fast seeking and thumbnail generation.
 * BSP reference: vmh264.c i_only_flag, AV_SCRATCH_F bit 6 (IP-only ucode mode).
 */
#define V4L2_CID_MESON_VDEC_I_ONLY	(V4L2_CID_USER_BASE + 0x1000)
#include "vdec_1.h"
#include "vdec_hevc.h"
#include "hcodec.h"

/*
 * Custom V4L2 control: decode stall timeout (milliseconds).
 *
 * Mirrors BSP vmh264.c decode_timeout_val.  A host-side timer in the
 * H.264 codec module checks whether the firmware is making macroblock
 * progress.  If no progress for this many ms, the session is aborted.
 * 0 = disabled (no watchdog).
 *
 * BSP reference: vmh264.c:195 (decode_timeout_val = 100),
 * vmh264.c:9675-9770 (check_timer_func).
 */
#define V4L2_CID_USER_MESON_VDEC_BASE		(V4L2_CID_USER_BASE + 0x1000)
#define V4L2_CID_MESON_VDEC_DECODE_TIMEOUT	(V4L2_CID_USER_MESON_VDEC_BASE + 1)
#define V4L2_CID_MESON_VDEC_ERROR_POLICY	(V4L2_CID_USER_MESON_VDEC_BASE + 2)

#define DECODE_TIMEOUT_DEFAULT	200	/* ms -- used by H.264 per-MB watchdog */
#define DECODE_TIMEOUT_MAX	30000	/* ms */
#define WATCHDOG_INTERVAL	msecs_to_jiffies(500)	/* check every 500ms */
/*
 * Framework watchdog timeout: how long without ANY IRQ before declaring
 * a stall. Must be much longer than decode_timeout_ms (which is per-MB
 * for H.264). Per-frame decode times vary widely:
 *   - GXL 4K H.264: ~222ms/frame
 *   - DPB fill: up to 16 frames buffered before first output = ~3.5s
 *   - src_change: firmware pauses for capture buffer allocation
 *
 * 5000ms (5s) is conservative: long enough for DPB fill on the slowest
 * platform (GXL 4K), short enough that a real firmware hang is detected
 * before the user gives up. Separate from decode_timeout_ms to avoid
 * coupling with the H.264 per-MB watchdog.
 */
#define WATCHDOG_TIMEOUT_MS	5000

/*
 * error_proc_policy default: matches BSP vmh264.c:277 (error_proc_policy = 0x7fCff6).
 * 26-bit bitmask controlling host-side error handling behavior.
 */
#define ERROR_PROC_POLICY_DEFAULT	0x007fcff6
#define ERROR_PROC_POLICY_MAX		0x03ffffff

/*
 * Stateless mode: switches the entire driver between stateful per-codec
 * firmware (default) and libre stateless V4L2 Request API firmware.
 *
 * In stateless mode, codecs that have a stateless implementation (currently
 * MPEG-1/2) expose the stateless V4L2 pixel format (MPEG2_SLICE) instead of
 * the stateful one.  Codecs without a stateless implementation remain
 * available as stateful.  As more libre stateless codecs are added (H.264,
 * HEVC, VP9), they will automatically replace their stateful counterparts
 * when this parameter is set.
 *
 * Usage: modprobe meson-vdec stateless=1
 *        meson-vdec.stateless=1  (kernel command line)
 */
static bool stateless_mode;
module_param_named(stateless, stateless_mode, bool, 0444);
MODULE_PARM_DESC(stateless, "0=stateful per-codec API (default), 1=libre stateless V4L2 Request API");

/*
 * Diagnostic hot-reload test bench (Tier 1).  When set, meson_vdec_core
 * exposes /sys/kernel/debug/meson-vdec/diag/vdec1/{firmware,reset,state,
 * peek,poke,mode,cmd}.  Default off for production fleet safety; bringup
 * boards load with `modprobe meson-vdec diag=1`.
 *
 * Design: amlogic/software/codec/decoder/stateless/test-bench-plan.md
 */
module_param_named(diag, vdec_diag_enabled, bool, 0444);
MODULE_PARM_DESC(diag, "0=no diag interface (default), 1=expose debugfs hot-reload test bench");

/* Weak reference to probed core for codec registry busy checks */
static struct amvdec_core *vdec_core_instance;

/*
 * Stateless pixel format check.  Returns true for V4L2 pixel formats that
 * belong to the stateless (V4L2 Request API) codec path.  Used by
 * format_is_visible() to gate formats based on the stateless module param,
 * and by amvdec_is_stateless() to select the stateless decode path at runtime.
 */
static bool is_stateless_pixfmt(u32 pixfmt)
{
	switch (pixfmt) {
	case V4L2_PIX_FMT_MPEG2_SLICE:
	case V4L2_PIX_FMT_H264_SLICE:
	/* Future stateless formats: */
	/* case V4L2_PIX_FMT_HEVC_SLICE: */
	/* case V4L2_PIX_FMT_VP9_FRAME: */
		return true;
	default:
		return false;
	}
}

/*
 * Returns the stateful pixel format that a stateless format replaces.
 * When stateless_mode is active, the stateful equivalent is hidden.
 */
static bool has_stateless_replacement(u32 pixfmt)
{
	switch (pixfmt) {
	case V4L2_PIX_FMT_MPEG1:
	case V4L2_PIX_FMT_MPEG2:
	case V4L2_PIX_FMT_H264:
		return true; /* replaced by MPEG2_SLICE / H264_SLICE in stateless mode */
	default:
		return false;
	}
}

static inline bool amvdec_is_stateless(struct amvdec_session *sess)
{
	return is_stateless_pixfmt(sess->fmt_out->pixfmt);
}

/*
 * Determine if a format entry should be visible to userspace.
 *
 * jpeg_node: true if this query is for the MJPEG-only device node.
 *   - On the MJPEG node: only MJPEG is visible.
 *   - On the main decoder node: MJPEG is hidden (separate node prevents
 *     v4l2-compliance QUERYCAP failure when both JPEG_DECODER and
 *     STATEFUL_DECODER bits would otherwise be set on the same node).
 */
static bool format_is_visible(const struct amvdec_format *fmt, bool jpeg_node)
{
	/* Hide formats whose codec module is not loaded */
	if (!fmt->codec_ops)
		return false;

	if (fmt->pixfmt == V4L2_PIX_FMT_MJPEG)
		return jpeg_node;

	/* Non-MJPEG formats are never visible on the MJPEG device node */
	if (jpeg_node)
		return false;

	/* Stateless formats: only visible in stateless mode */
	if (is_stateless_pixfmt(fmt->pixfmt))
		return stateless_mode;

	/* Stateful formats that have a stateless replacement: hidden in stateless mode */
	if (has_stateless_replacement(fmt->pixfmt))
		return !stateless_mode;

	/* All other formats: always visible (H.264, HEVC, VP9, MPEG4, H.263, etc.) */
	return true;
}

static bool any_stateless_visible(void)
{
	return stateless_mode;
}

struct dummy_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

/* 16 MiB for parsed bitstream swap exchange */
#define SIZE_VIFIFO SZ_16M

static u32 get_output_size(u32 width, u32 height)
{
	/* Width: 64-byte aligned stride for HEVC/VP9 SAO DMA engine
	 * (HEVC_SAO_CTRL1 axi_linealign=2) and VDEC_1 canvas (>=32).
	 *
	 * Height: 32-byte aligned to match BSP canvas configuration.
	 * Over-aligning to 64 causes the canvas height to exceed the
	 * firmware's macroblock-row count (e.g. 512 vs 480 for 640x480),
	 * which misplaces NV12 chroma plane offsets and corrupts rows
	 * 12-15 of every 16-row macroblock in H.264 decode.
	 * BSP H.264 uses mb_height*16 (16-aligned), BSP MPEG12 uses
	 * ALIGN(height, 32).  32 satisfies both.
	 */
	return ALIGN(ALIGN(width, 64) * ALIGN(height, 32), SZ_64K);
}

u32 amvdec_get_output_size(struct amvdec_session *sess)
{
	return get_output_size(sess->width, sess->height);
}
EXPORT_SYMBOL_GPL(amvdec_get_output_size);

static int vdec_codec_needs_recycle(struct amvdec_session *sess)
{
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;

	return codec_ops->can_recycle && codec_ops->recycle;
}

static int vdec_recycle_thread(void *data)
{
	struct amvdec_session *sess = data;
	struct amvdec_core *core = sess->core;
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;
	struct amvdec_buffer *tmp, *n;

	while (!kthread_should_stop()) {
		/*
		 * Event-driven: sleep until vdec_queue_recycle() adds a
		 * buffer to the recycle list (or kthread_stop).  The 5ms
		 * timeout is a safety net -- under normal operation the
		 * wake_up from vdec_queue_recycle fires within microseconds
		 * of ffmpeg's QBUF, eliminating the 500us polling delay
		 * that caused canvas starvation on fast decode streams.
		 */
		wait_event_timeout(sess->recycle_wq,
				   !list_empty(&sess->bufs_recycle) ||
				   kthread_should_stop() ||
				   sess->aborted,
				   msecs_to_jiffies(5));

		/* Skip all work if session is aborted -- accessing VB2
		 * buffers or m2m_ctx after abort races with
		 * vdec_stop_streaming() freeing session resources.
		 */
		if (READ_ONCE(sess->aborted))
			continue;

		/* Deliver any frames that were deferred because capture
		 * buffers were in userspace when the ISR fired.
		 */
		amvdec_flush_pending_frames(sess);

		mutex_lock(&sess->bufs_recycle_lock);
		list_for_each_entry_safe(tmp, n, &sess->bufs_recycle, list) {
			if (!codec_ops->can_recycle(core))
				break;

			codec_ops->recycle(core, tmp->vb->index);
			list_del(&tmp->list);
			kfree(tmp);
		}
		mutex_unlock(&sess->bufs_recycle_lock);
	}

	return 0;
}

/*
 * Framework-level decode watchdog. Monitors last_irq_jiffies to detect
 * firmware stalls on any codec (VP9, HEVC, MPEG-1/2, etc). H.264 has
 * an additional per-MB watchdog (codec_h264.c check_timer) that catches
 * intra-frame stalls faster; this framework watchdog is the safety net
 * for all other codecs that lack per-codec stall detection.
 *
 * Fires every 500ms. If no IRQ has arrived within WATCHDOG_TIMEOUT_MS
 * (5s), aborts the session via amvdec_abort().
 *
 * Suppressed during:
 *   - STATUS != RUNNING (not yet started, or paused for src_change)
 *   - aborted (already aborting, firmware halted)
 *   - firmware_halted (drain_finalize halted firmware; awaiting
 *     STREAMOFF or DEC_CMD_START -- no IRQs are expected)
 *   - draining (should_stop=1, before firmware_halted): the codec's
 *     own drain hook owns this window with bounded timeouts.  Some
 *     codecs (VP9 0xf2 EOS ISR) empirically never fire on current
 *     firmware revisions, so a multi-second no-IRQ window during
 *     drain is normal, not a wedge.  See vp9-001-drain-strategies.md
 *     S1.
 *   - No IRQ ever received (firmware still initializing / DPB filling)
 */
static void vdec_watchdog_cb(struct timer_list *timer)
{
	struct amvdec_session *sess =
		container_of(timer, struct amvdec_session, decode_watchdog);
	u64 last_irq;

	/* Don't fire when not actively decoding */
	if (sess->status != STATUS_RUNNING)
		goto rearm;

	/* Don't fire after abort (firmware already halted) */
	if (READ_ONCE(sess->aborted))
		goto rearm;

	/* Don't fire after drain_finalize halt (no IRQs expected) */
	if (READ_ONCE(sess->firmware_halted))
		goto rearm;

	/* Don't fire during drain (codec hook owns this window).
	 * VP9-001 EOS-drain pattern: no-IRQ windows of seconds are
	 * normal during VP9 drain at 4K+; the codec's queued_bufs
	 * wait + cur_frame->done wait + framework vdec_wait_inactive
	 * detect a real stall faster than this 5s safety net would.
	 * Suppressing here removes the spurious abort that ate
	 * trailing frames at 4K-6K once cma-opt-sg unlocked those
	 * resolutions on small-CMA boards (2026-04-23).
	 */
	if (READ_ONCE(sess->should_stop))
		goto rearm;

	last_irq = sess->last_irq_jiffies;
	if (!last_irq)
		goto rearm; /* no IRQ yet -- firmware still initializing */

	if (jiffies_to_msecs(get_jiffies_64() - last_irq) >= WATCHDOG_TIMEOUT_MS) {
		dev_err(sess->core->dev,
			"decode watchdog: no IRQ for %ums, aborting session\n",
			WATCHDOG_TIMEOUT_MS);
		amvdec_abort(sess);
		return; /* don't rearm */
	}

rearm:
	mod_timer(&sess->decode_watchdog, jiffies + WATCHDOG_INTERVAL);
}

static int vdec_poweron(struct amvdec_session *sess)
{
	int ret;
	struct amvdec_ops *vdec_ops = sess->fmt_out->vdec_ops;

	ret = clk_prepare_enable(sess->core->dos_parser_clk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(sess->core->dos_clk);
	if (ret)
		goto disable_dos_parser;

	ret = vdec_ops->start(sess);
	if (ret)
		goto disable_dos;

	esparser_power_up(sess);

	/* Arm the framework-level decode watchdog */
	mod_timer(&sess->decode_watchdog, jiffies + WATCHDOG_INTERVAL);

	return 0;

disable_dos:
	clk_disable_unprepare(sess->core->dos_clk);
disable_dos_parser:
	clk_disable_unprepare(sess->core->dos_parser_clk);

	return ret;
}

static void vdec_wait_inactive(struct amvdec_session *sess)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(2000);

	/*
	 * Wait until 50ms have elapsed with no IRQ, or 2s total
	 * (whichever comes first).  The 2s cap prevents an infinite
	 * loop when firmware keeps firing IRQs despite MPSR=0 -- the
	 * caller will cut power regardless.
	 */
	while (time_is_after_jiffies64(sess->last_irq_jiffies +
				       msecs_to_jiffies(50))) {
		if (time_after(jiffies, deadline)) {
			dev_warn(sess->core->dev,
				 "vdec_wait_inactive: timeout after 2s, forcing stop\n");
			break;
		}
		msleep(5);
	}
}

/*
 * Asynchronous drain work: waits for the firmware to finish outputting
 * all decoded frames, then signals LAST to the M2M framework.
 *
 * This runs in a workqueue (not the ioctl thread) so that userspace
 * can continue to DQBUF/QBUF capture buffers during drain.  Without
 * this, V4L2_DEC_CMD_STOP would block while the firmware is stalled
 * waiting for capture buffers to be recycled -- a deadlock.
 */
static void vdec_drain_work(struct work_struct *work)
{
	struct amvdec_session *sess =
		container_of(work, struct amvdec_session, drain_work);
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;
	const struct v4l2_event ev = { .type = V4L2_EVENT_EOS };
	struct vb2_v4l2_buffer *last_buf;
	ktime_t t0 = ktime_get();

	vdec_dbg_log_event(sess->core, VDEC_EVT_DRAIN_START, 0,
			   atomic_read(&sess->esparser_queued_bufs));
	dev_dbg(sess->core->dev,
		"drain: entry queued=%d\n",
		atomic_read(&sess->esparser_queued_bufs));

	/* Active drain: flush any frames the ISR may not have delivered.
	 * For MPEG1/2 this polls BUFFEROUT directly; it also sends the
	 * sequence_end_code if the codec needs it.
	 *
	 * NOTE: There used to be a 5-second wait loop BEFORE this drain
	 * call, waiting for esparser_queued_bufs to reach zero.  That was
	 * a bug (KI-033): for codecs where drain() is what delivers the
	 * last frame(s) and decrements the counter, waiting first is
	 * pointless -- the counter can never reach zero without drain.
	 * The wait loop was removed; drain is called immediately and the
	 * post-drain wait below handles the rest.
	 */
	if (codec_ops->drain)
		codec_ops->drain(sess);

	vdec_dbg_log_event(sess->core, VDEC_EVT_DRAIN_CODEC_DRAINED, 0,
			   atomic_read(&sess->esparser_queued_bufs));
	dev_dbg(sess->core->dev,
		"drain: after codec_drain t=%lldus queued=%d\n",
		ktime_us_delta(ktime_get(), t0),
		atomic_read(&sess->esparser_queued_bufs));

	/*
	 * After drain() sent EOS, the firmware may still be processing
	 * the last frames: decoding them into canvases and firing ISRs
	 * asynchronously.  If we send LAST before those ISRs complete,
	 * the decoded frames arrive in the VB2 done queue AFTER the LAST
	 * sentinel and are silently dropped by userspace.
	 *
	 * Wait up to 5s for esparser_queued_bufs to reach zero (all frame
	 * ISRs fired and accounted for) before proceeding to send LAST.
	 * In the common case (frames delivered during drain's EOS poll),
	 * the counter is already zero and this exits immediately.
	 *
	 * 5s is chosen because canvas recycling requires the ffmpeg
	 * DQBUF/QBUF cycle and the recycle_thread: under heavy scheduling
	 * pressure (e.g. right after a long test run) this can take
	 * longer than the 2s that suffices in the steady-state case.
	 *
	 * Also bail early on !streamon_out (same reason as above).
	 */
	wait_event_timeout(sess->esparser_queue_done_wq,
			   atomic_read(&sess->esparser_queued_bufs) <= 0 ||
			   !sess->streamon_out,
			   msecs_to_jiffies(5000));

	vdec_dbg_log_event(sess->core, VDEC_EVT_DRAIN_WAIT_QUEUED, 0,
			   atomic_read(&sess->esparser_queued_bufs));
	dev_dbg(sess->core->dev,
		"drain: after queued_wait t=%lldus queued=%d\n",
		ktime_us_delta(ktime_get(), t0),
		atomic_read(&sess->esparser_queued_bufs));

	/* Final inactivity wait for any trailing IRQs from the drain */
	vdec_wait_inactive(sess);

	vdec_dbg_log_event(sess->core, VDEC_EVT_DRAIN_WAIT_INACTIVE, 0, 0);
	dev_dbg(sess->core->dev,
		"drain: after wait_inactive t=%lldus\n",
		ktime_us_delta(ktime_get(), t0));

	/*
	 * Flush any frames deferred during drain (capture buffers were in
	 * userspace when amvdec_dst_buf_done_idx was called).  This is
	 * critical for codecs with reference frame sliding windows (MPEG4):
	 * flush_refs() may defer the last 1-2 frames, and if we send LAST
	 * before they are delivered, ffmpeg stops and the frames are lost.
	 *
	 * Wait up to 2s for userspace to DQBUF and re-QBUF capture buffers
	 * so the recycle thread can deliver the pending frames.
	 */
	{
		unsigned long pf_timeout = jiffies + msecs_to_jiffies(2000);
		bool pending;

		do {
			amvdec_flush_pending_frames(sess);
			spin_lock(&sess->pending_frames_lock);
			pending = !list_empty(&sess->pending_frames);
			spin_unlock(&sess->pending_frames_lock);
			if (!pending)
				break;
			msleep(5);
		} while (time_is_after_jiffies(pf_timeout));
	}

	vdec_dbg_log_event(sess->core, VDEC_EVT_DRAIN_FLUSH_PENDING, 0, 0);
	dev_dbg(sess->core->dev,
		"drain: after flush_pending t=%lldus\n",
		ktime_us_delta(ktime_get(), t0));

	/*
	 * Per-codec drain_finalize hook: halts the firmware (ending
	 * the HEVC-engine VIFIFO-search spin on gxl_vp9_mmu / gxl_hevc_mmu
	 * that otherwise runs until STREAMOFF) and closes the
	 * esparser_queued_bufs protocol.  Runs AFTER the waits above so
	 * any late post-EOS ISRs have already fired and delivered frames.
	 * Optional -- NULL preserves the legacy "firmware spins to
	 * STREAMOFF" behavior.  See KI-VDEC-ARCH-001.
	 */
	if (codec_ops->drain_finalize) {
		codec_ops->drain_finalize(sess);
		vdec_dbg_log_event(sess->core, VDEC_EVT_DRAIN_FIRMWARE_HALTED, 0,
				   atomic_read(&sess->esparser_queued_bufs));
		dev_dbg(sess->core->dev,
			"drain: after drain_finalize t=%lldus queued=%d\n",
			ktime_us_delta(ktime_get(), t0),
			atomic_read(&sess->esparser_queued_bufs));
	}

	/*
	 * Final check after EVERY drain-path counter closer has run
	 * (wait_event_timeout, flush_pending_frames, drain_finalize).
	 * If the counter is STILL positive here, the input/output
	 * accounting has a real leak for this codec -- worth warning.
	 * Previously this warn fired right after the wait_event_timeout,
	 * which was too early: flush_pending_frames and drain_finalize
	 * still had a chance to deliver or force-close.
	 */
	if (atomic_read(&sess->esparser_queued_bufs) > 0)
		dev_warn(sess->core->dev,
			 "drain: %s, %d frames unaccounted after full drain sequence\n",
			 sess->streamon_out ?
				"post-drain" :
				"output stream stopped",
			 atomic_read(&sess->esparser_queued_bufs));

// MOVED BELOW: 	v4l2_event_queue_fh(&sess->fh, &ev);

	/*
	 * Signal LAST by taking one idle capture buffer and delivering it
	 * as a zero-byte EOS sentinel.  This is the standard V4L2 stateful
	 * decoder EOS mechanism: userspace sees a buffer with LAST flag and
	 * bytesused=0 and stops.
	 *
	 * IMPORTANT: we must not send LAST while decoded frames are still
	 * sitting in the VB2 done queue waiting for userspace to DQBUF them.
	 * If the ready queue is empty it means userspace is still consuming
	 * decoded frames; we must wait for it to recycle at least one buffer
	 * before we can safely use it as the LAST sentinel.  Taking it
	 * immediately via next_buf_last would cause the first re-queued
	 * buffer to be intercepted as LAST before the other done-queue
	 * frames are dequeued, dropping the final frame(s).
	 *
	 * Wait up to 2s for userspace to DQBUF a frame and recycle a
	 * capture buffer back to the ready queue.  In practice this loop
	 * exits on the first iteration because userspace is continuously
	 * DQBUFing while we are draining.
	 */
	{
		unsigned long idle_timeout = jiffies + msecs_to_jiffies(2000);

		do {
			last_buf = v4l2_m2m_dst_buf_remove(sess->m2m_ctx);
			if (last_buf)
				break;
			msleep(5);
		} while (time_is_after_jiffies(idle_timeout));
	}

	if (last_buf) {
		last_buf->vb2_buf.planes[0].bytesused = 0;
		v4l2_m2m_last_buffer_done(sess->m2m_ctx, last_buf);
	} else {
		/* Last resort: userspace has not recycled any buffers in 2s. */
		sess->m2m_ctx->is_draining = true;
		sess->m2m_ctx->next_buf_last = true;
	}

	/* Send EOS event AFTER LAST buffer to prevent gst teardown race */
	v4l2_event_queue_fh(&sess->fh, &ev);

	vdec_dbg_log_event(sess->core, VDEC_EVT_DRAIN_DONE, 0,
			   atomic_read(&sess->esparser_queued_bufs));
	dev_dbg(sess->core->dev,
		"drain: done t=%lldus queued=%d\n",
		ktime_us_delta(ktime_get(), t0),
		atomic_read(&sess->esparser_queued_bufs));
}

static void vdec_poweroff(struct amvdec_session *sess)
{
	struct amvdec_ops *vdec_ops = sess->fmt_out->vdec_ops;
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;

	sess->should_stop = 1;

	/*
	 * Quiesce the firmware BEFORE waiting for inactivity.
	 * Without this, vdec_wait_inactive() can loop forever because
	 * the firmware keeps firing IRQs (updating last_irq_jiffies).
	 *
	 * MPSR=0 halts the AMRISC/HEVC processor.
	 * Mailbox mask=0 disables the interrupt that triggers vdec_isr.
	 *
	 * vdec_ops->stop() will also write these but additionally
	 * handles power gating, isolation, and memory power-down.
	 */
	if (vdec_ops == &vdec_hevc_ops) {
		amvdec_write_dos(sess->core, HEVC_MPSR, 0);
		amvdec_write_dos(sess->core, HEVC_ASSIST_MBOX1_MASK, 0);
	} else {
		amvdec_write_dos(sess->core, MPSR, 0);
		amvdec_write_dos(sess->core, CPSR, 0);
		amvdec_write_dos(sess->core, ASSIST_MBOX1_MASK, 0);
	}

	vdec_wait_inactive(sess);
	if (codec_ops->drain)
		codec_ops->drain(sess);
	if (codec_ops->stop)
		codec_ops->stop(sess);

	vdec_ops->stop(sess);
	clk_disable_unprepare(sess->core->dos_clk);
	clk_disable_unprepare(sess->core->dos_parser_clk);
}

static void
vdec_queue_recycle(struct amvdec_session *sess, struct vb2_buffer *vb)
{
	struct amvdec_buffer *new_buf;

	new_buf = kmalloc(sizeof(*new_buf), GFP_KERNEL);
	if (!new_buf)
		return;
	new_buf->vb = vb;

	mutex_lock(&sess->bufs_recycle_lock);
	list_add_tail(&new_buf->list, &sess->bufs_recycle);
	mutex_unlock(&sess->bufs_recycle_lock);

	/* Wake recycle thread immediately to minimize canvas starvation */
	wake_up(&sess->recycle_wq);
}

static void vdec_m2m_device_run(void *priv)
{
	struct amvdec_session *sess = priv;

	if (amvdec_is_stateless(sess)) {
		struct vb2_v4l2_buffer *src_buf;
		struct vb2_v4l2_buffer *dst_buf;
		struct media_request *src_req;
		struct amvdec_core *core = sess->core;
		int i, fw_idx = -1;
		int ret;

		src_buf = v4l2_m2m_next_src_buf(sess->m2m_ctx);
		dst_buf = v4l2_m2m_next_dst_buf(sess->m2m_ctx);
		src_req = src_buf->vb2_buf.req_obj.req;

		if (src_req) {
			v4l2_ctrl_request_setup(src_req,
						&sess->ctrl_handler);
			/*
			 * Mark the request complete now that we've consumed
			 * its ctrls. This releases POLLPRI on the request fd
			 * so userspace can re-use or free it. Pattern from
			 * cedrus_dec.c:105 -- the request_complete is NOT
			 * "decode completed" (that's v4l2_m2m_buf_done), it's
			 * "ctrls consumed". Without this, POLLPRI only fires
			 * when the buffer is released via DQBUF, which
			 * userspace may wait for via POLLPRI -> deadlock.
			 */
			v4l2_ctrl_request_complete(src_req,
						   &sess->ctrl_handler);
		}

		/*
		 * Deferred poweron: start firmware only when we have
		 * both src and dst buffers ready, so canvases can be set
		 * before MPSR=1, matching the stateful timing.
		 */
		if (sess->status == STATUS_STOPPED) {
			sess->vififo_size = SIZE_VIFIFO;
			sess->vififo_vaddr =
				dma_alloc_coherent(core->dev,
						   sess->vififo_size,
						   &sess->vififo_paddr,
						   GFP_KERNEL);
			if (!sess->vififo_vaddr) {
				dev_err(core->dev, "Failed to alloc VIFIFO\n");
				v4l2_m2m_job_finish(sess->m2m_dev,
						    sess->m2m_ctx);
				return;
			}

			ret = vdec_poweron(sess);
			if (ret) {
				dma_free_coherent(core->dev,
						  sess->vififo_size,
						  sess->vififo_vaddr,
						  sess->vififo_paddr);
				v4l2_m2m_job_finish(sess->m2m_dev,
						    sess->m2m_ctx);
				return;
			}

			sess->status = STATUS_RUNNING;
			core->cur_sess = sess;
		}

		/* Codec-specific per-run buffer setup (stateless codecs) */
		if (sess->fmt_out->codec_ops->setup_buffers)
			sess->fmt_out->codec_ops->setup_buffers(sess);

		/* Find firmware buffer index for this dst buffer */
		for (i = 0; i < sess->num_dst_bufs; i++) {
			if (sess->fw_idx_to_vb2_idx[i] ==
			    dst_buf->vb2_buf.index) {
				fw_idx = i;
				break;
			}
		}

		mutex_lock(&sess->lock);
		esparser_queue(sess, src_buf);
		mutex_unlock(&sess->lock);

		/* Send EOS as separate DMA to push VIFIFO WP past the
		 * VLD read threshold, allowing the firmware to process
		 * the sequence end code and complete the frame. */
		{
			struct amvdec_codec_ops *codec_ops =
				sess->fmt_out->codec_ops;
			u32 eos_len;
			const u8 *eos_data;

			if (codec_ops->eos_sequence) {
				eos_data = codec_ops->eos_sequence(&eos_len);
				esparser_queue_eos(core, eos_data, eos_len);
			}
		}

		/* Tell firmware which dst buffer to use (stateless codecs) */
		if (sess->fmt_out->codec_ops->set_dst_buf)
			sess->fmt_out->codec_ops->set_dst_buf(sess, fw_idx);

		/* ISR calls v4l2_m2m_job_finish when firmware completes */
	} else {
		schedule_work(&sess->esparser_queue_work);
	}
}

static void vdec_m2m_job_abort(void *priv)
{
	struct amvdec_session *sess = priv;

	v4l2_m2m_job_finish(sess->m2m_dev, sess->m2m_ctx);
}

static const struct v4l2_m2m_ops vdec_m2m_ops = {
	.device_run = vdec_m2m_device_run,
	.job_abort = vdec_m2m_job_abort,
};

static void process_num_buffers(struct vb2_queue *q,
				struct amvdec_session *sess,
				unsigned int *num_buffers,
				bool is_reqbufs)
{
	const struct amvdec_format *fmt_out = sess->fmt_out;
	unsigned int q_num_bufs = vb2_get_num_buffers(q);
	unsigned int buffers_total = q_num_bufs + *num_buffers;
	u32 min_buf_capture = sess->ctrl_min_buf_capture ?
		v4l2_ctrl_g_ctrl(sess->ctrl_min_buf_capture) : 1;

	if (q_num_bufs + *num_buffers < min_buf_capture)
		*num_buffers = min_buf_capture - q_num_bufs;
	if (is_reqbufs && buffers_total < fmt_out->min_buffers)
		*num_buffers = fmt_out->min_buffers - q_num_bufs;
	if (buffers_total > fmt_out->max_buffers)
		*num_buffers = fmt_out->max_buffers - q_num_bufs;

	/* We need to program the complete CAPTURE buffer list
	 * in registers during start_streaming, and the firmwares
	 * are free to choose any of them to write frames to. As such,
	 * we need all of them to be queued into the driver.
	 * For stateless, userspace manages buffer flow directly.
	 */
	if (sess->max_alloc_buf_count > 0 &&
	    q_num_bufs + *num_buffers > sess->max_alloc_buf_count)
		*num_buffers = sess->max_alloc_buf_count - q_num_bufs;

	sess->num_dst_bufs = q_num_bufs + *num_buffers;
	if (!amvdec_is_stateless(sess))
		q->min_queued_buffers = max(fmt_out->min_buffers,
					    sess->num_dst_bufs);
}

static int vdec_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
			    unsigned int *num_planes, unsigned int sizes[],
			    struct device *alloc_devs[])
{
	struct amvdec_session *sess = vb2_get_drv_priv(q);
	u32 output_size = amvdec_get_output_size(sess);

	if (*num_planes) {
		switch (q->type) {
		case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
			if (*num_planes != 1 ||
			    sizes[0] < sess->src_buffer_size)
				return -EINVAL;
			break;
		case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
			switch (sess->pixfmt_cap) {
			case V4L2_PIX_FMT_NV12:
			case V4L2_PIX_FMT_NV21:
				if (*num_planes != 1 ||
				    sizes[0] < output_size +
					       output_size / 2)
					return -EINVAL;
				break;
			case V4L2_PIX_FMT_NV12M:
			case V4L2_PIX_FMT_NV21M:
				if (*num_planes != 2 ||
				    sizes[0] < output_size ||
				    sizes[1] < output_size / 2)
					return -EINVAL;
				break;
			case V4L2_PIX_FMT_YUV420M:
				if (*num_planes != 3 ||
				    sizes[0] < output_size ||
				    sizes[1] < output_size / 4 ||
				    sizes[2] < output_size / 4)
					return -EINVAL;
				break;
			case V4L2_PIX_FMT_MESON_FBC08_SCATTER:
			case V4L2_PIX_FMT_MESON_FBC10_SCATTER: {
				bool is_10bit = sess->pixfmt_cap ==
						V4L2_PIX_FMT_MESON_FBC10_SCATTER;
				u32 fbc_size = amvdec_am21c_size(sess->width,
								 sess->height,
								 is_10bit);

				if (*num_planes != 1 || sizes[0] < fbc_size)
					return -EINVAL;
				break;
			}
			default:
				return -EINVAL;
			}

			process_num_buffers(q, sess, num_buffers, false);
			break;
		}

		return 0;
	}

	switch (q->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		/*
		 * Add 512 bytes for the esparser search start code pattern.
		 * esparser_pad_start_code() appends a 4-byte search marker
		 * (00 00 01 ff) plus up to 512 bytes of padding after the
		 * payload.  If the payload fills the entire buffer,
		 * esparser_pad_start_code() truncates the payload to fit.
		 */
		sizes[0] = sess->src_buffer_size + 512;
		*num_planes = 1;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		switch (sess->pixfmt_cap) {
		case V4L2_PIX_FMT_NV12:
		case V4L2_PIX_FMT_NV21:
			sizes[0] = output_size + output_size / 2;
			*num_planes = 1;
			break;
		case V4L2_PIX_FMT_NV12M:
		case V4L2_PIX_FMT_NV21M:
			sizes[0] = output_size;
			sizes[1] = output_size / 2;
			*num_planes = 2;
			break;
		case V4L2_PIX_FMT_YUV420M:
			sizes[0] = output_size;
			sizes[1] = output_size / 4;
			sizes[2] = output_size / 4;
			*num_planes = 3;
			break;
		case V4L2_PIX_FMT_MESON_FBC08_SCATTER:
		case V4L2_PIX_FMT_MESON_FBC10_SCATTER: {
			bool is_10bit = sess->pixfmt_cap ==
					V4L2_PIX_FMT_MESON_FBC10_SCATTER;

			sizes[0] = amvdec_am21c_size(sess->width,
						     sess->height,
						     is_10bit);
			*num_planes = 1;
			break;
		}
		default:
			return -EINVAL;
		}

		process_num_buffers(q, sess, num_buffers, true);
		break;
	default:
		return -EINVAL;
	}

	sess->changed_format = 1;
	return 0;
}

static void vdec_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct amvdec_session *sess = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_m2m_ctx *m2m_ctx = sess->m2m_ctx;

	v4l2_m2m_buf_queue(m2m_ctx, vbuf);

	if (!sess->streamon_out) {
		/* During drain (should_stop=1), STREAMOFF(OUT) has already been
		 * called but drain_work is still running. Allow CAPTURE buffer
		 * recycling so firmware doesn't stall waiting for canvas slots.
		 */
		if (!sess->should_stop ||
		    vb->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
			return;
	}

	if (sess->streamon_cap &&
	    vb->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    vdec_codec_needs_recycle(sess))
		vdec_queue_recycle(sess, vb);

	if (!amvdec_is_stateless(sess) && sess->status >= STATUS_INIT)
		schedule_work(&sess->esparser_queue_work);
}

static int vdec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct amvdec_session *sess = vb2_get_drv_priv(q);
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;
	struct amvdec_core *core = sess->core;
	struct vb2_v4l2_buffer *buf;
	int ret;

	if (core->cur_sess && core->cur_sess != sess) {
		ret = -EBUSY;
		goto bufs_done;
	}

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		sess->streamon_out = 1;
		vdec_dbg_log_event(core, VDEC_EVT_STREAMON_OUT, 0, 0);
	} else {
		sess->streamon_cap = 1;
		vdec_dbg_log_event(core, VDEC_EVT_STREAMON_CAP, 0, 0);
	}

	if (!sess->streamon_out)
		return 0;

	if (READ_ONCE(sess->status) == STATUS_NEEDS_RESUME &&
	    q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (atomic_cmpxchg(&sess->resumed, 0, 1) == 0) {
			codec_ops->resume(sess);
			WRITE_ONCE(sess->status, STATUS_RUNNING);
		}
		return 0;
	}

	/*
	 * Deferred poweron for non-DYN_RESOLUTION codecs (MPEG12, MJPEG,
	 * MPEG4). These codecs set up canvas-to-buffer mappings in their
	 * start() callback, but capture buffers aren't queued until after
	 * OUTPUT STREAMON. We defer the entire poweron (firmware load +
	 * codec start + MPSR) until CAPTURE STREAMON, when buffers exist
	 * and amvdec_set_canvases() can configure real DMA addresses.
	 */
	if (sess->status == STATUS_NEEDS_START &&
	    q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = vdec_poweron(sess);
		if (ret)
			goto vififo_free;

		sess->sequence_cap = 0;
		if (vdec_codec_needs_recycle(sess))
			sess->recycle_thread =
				kthread_run(vdec_recycle_thread, sess,
					    "vdec_recycle");

		sess->status = STATUS_RUNNING;
		schedule_work(&sess->esparser_queue_work);
		return 0;
	}

	if (sess->status == STATUS_RUNNING ||
	    sess->status == STATUS_NEEDS_RESUME ||
	    sess->status == STATUS_INIT ||
	    sess->status == STATUS_NEEDS_START)
		return 0;

	/*
	 * For stateless, VIFIFO is allocated in device_run along with
	 * deferred poweron.
	 */
	if (!amvdec_is_stateless(sess)) {
		sess->vififo_size = SIZE_VIFIFO;
		sess->vififo_vaddr =
			dma_alloc_coherent(sess->core->dev, sess->vififo_size,
					   &sess->vififo_paddr, GFP_KERNEL);
		if (!sess->vififo_vaddr) {
			dev_err(sess->core->dev,
				"Failed to request VIFIFO buffer\n");
			ret = -ENOMEM;
			goto bufs_done;
		}
	}

	sess->should_stop = 0;
	sess->aborted = 0;
	sess->firmware_halted = 0;
	sess->keyframe_found = 0;
	sess->last_offset = 0;
	sess->wrap_count = 0;
	sess->last_ts_done = 0;
	sess->pixelaspect.numerator = 1;
	sess->pixelaspect.denominator = 1;
	atomic_set(&sess->esparser_queued_bufs, 0);
	if (sess->ctrl_min_buf_capture)
		v4l2_ctrl_s_ctrl(sess->ctrl_min_buf_capture, 1);

	/*
	 * For stateless, defer poweron to device_run so firmware starts
	 * with canvases set and data ready (matching stateful timing).
	 * Just initialize session state here.
	 */
	if (amvdec_is_stateless(sess)) {
		sess->sequence_cap = 0;
		sess->sequence_out = 0;
		/* Stay in STATUS_STOPPED; device_run will poweron */
		return 0;
	}

	/*
	 * Non-DYN_RESOLUTION codecs: defer poweron to CAPTURE STREAMON.
	 * Allocate VIFIFO now but don't start the firmware yet.
	 * If capture is already streaming (started before output),
	 * poweron immediately since buffers are available.
	 */
	if (!(sess->fmt_out->flags & V4L2_FMT_FLAG_DYN_RESOLUTION)) {
		sess->sequence_cap = 0;
		sess->sequence_out = 0;
		sess->status = STATUS_NEEDS_START;
		core->cur_sess = sess;

		if (sess->streamon_cap) {
			ret = vdec_poweron(sess);
			if (ret)
				goto vififo_free;
			if (vdec_codec_needs_recycle(sess))
				sess->recycle_thread =
					kthread_run(vdec_recycle_thread, sess,
						    "vdec_recycle");
			sess->status = STATUS_RUNNING;
			schedule_work(&sess->esparser_queue_work);
		}
		return 0;
	}

	ret = vdec_poweron(sess);
	if (ret)
		goto vififo_free;

	sess->sequence_cap = 0;
	sess->sequence_out = 0;
	if (vdec_codec_needs_recycle(sess))
		sess->recycle_thread = kthread_run(vdec_recycle_thread, sess,
						   "vdec_recycle");

	sess->status = STATUS_INIT;
	core->cur_sess = sess;
	schedule_work(&sess->esparser_queue_work);
	return 0;

vififo_free:
	dma_free_coherent(sess->core->dev, sess->vififo_size,
			  sess->vififo_vaddr, sess->vififo_paddr);
bufs_done:
	while ((buf = v4l2_m2m_src_buf_remove(sess->m2m_ctx)))
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_QUEUED);
	while ((buf = v4l2_m2m_dst_buf_remove(sess->m2m_ctx)))
		v4l2_m2m_buf_done(buf, VB2_BUF_STATE_QUEUED);

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		sess->streamon_out = 0;
	else
		sess->streamon_cap = 0;

	return ret;
}

static void vdec_free_canvas(struct amvdec_session *sess)
{
	int i;

	for (i = 0; i < sess->canvas_num; ++i)
		meson_canvas_free(sess->core->canvas, sess->canvas_alloc[i]);

	sess->canvas_num = 0;
}

static void vdec_reset_timestamps(struct amvdec_session *sess)
{
	struct amvdec_timestamp *tmp, *n;

	list_for_each_entry_safe(tmp, n, &sess->timestamps, list) {
		list_del(&tmp->list);
		kfree(tmp);
	}
}

static void vdec_reset_bufs_recycle(struct amvdec_session *sess)
{
	struct amvdec_buffer *tmp, *n;

	list_for_each_entry_safe(tmp, n, &sess->bufs_recycle, list) {
		list_del(&tmp->list);
		kfree(tmp);
	}
}

static void vdec_reset_pending_frames(struct amvdec_session *sess)
{
	struct amvdec_pending_frame *pf, *n;

	spin_lock(&sess->pending_frames_lock);
	list_for_each_entry_safe(pf, n, &sess->pending_frames, list) {
		list_del(&pf->list);
		kfree(pf);
	}
	spin_unlock(&sess->pending_frames_lock);
}

static void vdec_stop_streaming(struct vb2_queue *q)
{
	struct amvdec_session *sess = vb2_get_drv_priv(q);
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;
	struct amvdec_core *core = sess->core;
	struct vb2_v4l2_buffer *buf;

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		vdec_dbg_log_event(core, VDEC_EVT_STREAMOFF_OUT, 0, 0);
	else
		vdec_dbg_log_event(core, VDEC_EVT_STREAMOFF_CAP, 0, 0);

	/*
	 * V4L2 stateful decoder drain sequence:
	 *   V4L2_DEC_CMD_STOP  -> drain_work scheduled (async)
	 *   STREAMOFF(OUT)     -> client done feeding input; decoder must
	 *                         continue decoding buffered frames
	 *   DQBUF(CAPTURE)     -> client collects decoded frames until LAST
	 *   STREAMOFF(CAPTURE) -> client done; tear down decoder
	 *
	 * When STREAMOFF(OUT) arrives while drain_work is still running,
	 * do NOT cancel drain_work and do NOT power off the decoder.
	 * Cancelling would block the calling thread (cancel_work_sync),
	 * which is the same thread that needs to QBUF decoded buffers back
	 * to free canvas slots for the firmware.  The deadlock prevents the
	 * last frame from ever being decoded.
	 *
	 * Instead: just discard any pending source buffers and clear the
	 * output-side flag.  drain_work will complete naturally and send
	 * the LAST sentinel.  The decoder is torn down in the subsequent
	 * STREAMOFF(CAPTURE) call.
	 */
	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE && sess->should_stop && !sess->aborted) {
		while ((buf = v4l2_m2m_src_buf_remove(sess->m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
		sess->streamon_out = 0;
		return;
	}

	/* Disarm the framework-level decode watchdog before teardown */
	del_timer_sync(&sess->decode_watchdog);

	/*
	 * Quiesce firmware BEFORE cancel_work_sync.
	 *
	 * drain_work calls vdec_wait_inactive() which loops while
	 * last_irq_jiffies is recent.  If firmware is still running,
	 * it keeps firing IRQs and drain_work never completes.
	 * cancel_work_sync then blocks forever -> D-state.
	 *
	 * By halting the AMRISC processor (MPSR=0) and masking the
	 * mailbox interrupt here, we guarantee no new IRQs arrive,
	 * so both drain_work and vdec_wait_inactive converge quickly.
	 *
	 * Must handle both VDEC_1 and HEVC register spaces since the
	 * active engine depends on the codec.
	 */
	if (sess->status == STATUS_RUNNING ||
	    sess->status == STATUS_INIT) {
		if (sess->fmt_out->vdec_ops == &vdec_hevc_ops) {
			amvdec_write_dos(core, HEVC_MPSR, 0);
			amvdec_write_dos(core, HEVC_ASSIST_MBOX1_MASK, 0);
		} else {
			amvdec_write_dos(core, MPSR, 0);
			amvdec_write_dos(core, CPSR, 0);
			amvdec_write_dos(core, ASSIST_MBOX1_MASK, 0);
		}
	}

	/*
	 * Wait for any in-flight threaded ISR to complete before
	 * clearing cur_sess and freeing session state.  MPSR=0 +
	 * mask=0 above prevents new IRQs, but a threaded ISR already
	 * dispatched by genirq may still be running.  Without this
	 * barrier, the ISR dereferences sess->priv while stop_streaming
	 * frees it -- use-after-free (KI-100, GXL MVC crash 2026-04-17).
	 */
	synchronize_irq(core->irq);
	core->cur_sess = NULL;

	/*
	 * Signal the session as aborted early so the recycle thread
	 * skips all work (pending frames, buffer recycles) and exits
	 * the wait_event loop immediately.  Without this, the recycle
	 * thread may re-enter wait_event_timeout after kthread_stop
	 * sets the stop flag, racing with the teardown below.
	 */
	WRITE_ONCE(sess->aborted, 1);

	/*
	 * ABBA-deadlock avoidance: esparser_queue_all_src() acquires
	 * sess->lock as its first step, and vdec_stop_streaming runs
	 * with that lock held (it is the vb2 q->lock for both src_vq
	 * and dst_vq -- see m2m_queue_init).  If esparser_queue_work
	 * is already running when cancel_work_sync fires, the worker
	 * blocks on sess->lock while cancel waits for the worker =
	 * deadlock, detected via hung_task_panic.  Drop sess->lock
	 * across the two cancel_work_sync calls to let in-flight
	 * workers finish.
	 *
	 * Safe to drop here: sess->aborted is set, core->cur_sess is
	 * NULL, firmware is quiesced (MPSR=0, MBOX mask=0), and any
	 * late ioctl that races in under the dropped lock will see
	 * sess->aborted and bail out.
	 *
	 * Triggered (2026-04-19) by the DEC_CMD_START path
	 * (commit d8ab724d14fa) which re-schedules
	 * esparser_queue_work post-restart -- a narrow window where
	 * the work is pending/running when the next STREAMOFF fires.
	 */
	mutex_unlock(&sess->lock);
	cancel_work_sync(&sess->esparser_queue_work);
	cancel_work_sync(&sess->drain_work);
	mutex_lock(&sess->lock);

	if (sess->status == STATUS_NEEDS_START) {
		/* Deferred poweron: VIFIFO allocated but never powered on */
		dma_free_coherent(sess->core->dev, sess->vififo_size,
				  sess->vififo_vaddr, sess->vififo_paddr);
		core->cur_sess = NULL;
		sess->status = STATUS_STOPPED;
	} else if (sess->status == STATUS_RUNNING ||
		   sess->status == STATUS_INIT ||
		   (sess->status == STATUS_NEEDS_RESUME &&
		    (!sess->streamon_out || !sess->streamon_cap))) {
		if (vdec_codec_needs_recycle(sess) && sess->recycle_thread) {
			kthread_stop(sess->recycle_thread);
			sess->recycle_thread = NULL;
		}

		vdec_poweroff(sess);
		vdec_free_canvas(sess);
		dma_free_coherent(sess->core->dev, sess->vififo_size,
				  sess->vififo_vaddr, sess->vififo_paddr);
		vdec_reset_timestamps(sess);
		vdec_reset_bufs_recycle(sess);
		vdec_reset_pending_frames(sess);
		kfree(sess->priv);
		sess->priv = NULL;
		core->cur_sess = NULL;
		sess->status = STATUS_STOPPED;
	}

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		while ((buf = v4l2_m2m_src_buf_remove(sess->m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);

		sess->streamon_out = 0;
	} else {
		/*
		 * Stop the recycle thread before returning capture buffers.
		 * The recycle thread dereferences vb2_buffer objects; if we
		 * return them to VB2 first, the thread hits use-after-free.
		 * This must happen unconditionally -- the power-off block
		 * above may be skipped when status == STATUS_NEEDS_RESUME
		 * with both queues streaming, but the recycle thread is
		 * still running and must be stopped before buffer cleanup.
		 */
		if (sess->recycle_thread) {
			kthread_stop(sess->recycle_thread);
			sess->recycle_thread = NULL;
		}

		/* Drain remaining refs if was still running */
		if (sess->status >= STATUS_RUNNING && codec_ops->drain)
			codec_ops->drain(sess);

		while ((buf = v4l2_m2m_dst_buf_remove(sess->m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);

		sess->streamon_cap = 0;
	}
}

static int vdec_vb2_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	vbuf->field = V4L2_FIELD_NONE;
	return 0;
}

static void vdec_vb2_buf_request_complete(struct vb2_buffer *vb)
{
	struct amvdec_session *sess = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_ctrl_request_complete(vb->req_obj.req, &sess->ctrl_handler);
}

static int vdec_vb2_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	vbuf->field = V4L2_FIELD_NONE;
	return 0;
}

static const struct vb2_ops vdec_vb2_ops = {
	.queue_setup = vdec_queue_setup,
	.start_streaming = vdec_start_streaming,
	.stop_streaming = vdec_stop_streaming,
	.buf_queue = vdec_vb2_buf_queue,
	.buf_prepare = vdec_vb2_buf_prepare,
	.buf_request_complete = vdec_vb2_buf_request_complete,
	.buf_out_validate = vdec_vb2_buf_out_validate,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int
vdec_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	struct amvdec_session *sess =
		container_of(file->private_data, struct amvdec_session, fh);

	strscpy(cap->driver, "meson-vdec", sizeof(cap->driver));
	if (sess->jpeg_node)
		strscpy(cap->card, "Amlogic MJPEG Decoder", sizeof(cap->card));
	else
		strscpy(cap->card, "Amlogic Video Decoder", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:meson-vdec", sizeof(cap->bus_info));

	return 0;
}

static const struct amvdec_format *
find_format(const struct amvdec_format *fmts, u32 size, u32 pixfmt,
	    bool jpeg_node)
{
	unsigned int i;

	for (i = 0; i < size; i++) {
		if (fmts[i].pixfmt == pixfmt && format_is_visible(&fmts[i], jpeg_node))
			return &fmts[i];
	}

	return NULL;
}

/* Find the first visible format (for default selection) */
static const struct amvdec_format *
first_visible_format(const struct amvdec_format *fmts, u32 size, bool jpeg_node)
{
	unsigned int i;

	for (i = 0; i < size; i++) {
		if (format_is_visible(&fmts[i], jpeg_node))
			return &fmts[i];
	}

	return &fmts[0];
}

static unsigned int
vdec_supports_pixfmt_cap(const struct amvdec_format *fmt_out, u32 pixfmt_cap)
{
	int i;

	for (i = 0; fmt_out->pixfmts_cap[i]; i++)
		if (fmt_out->pixfmts_cap[i] == pixfmt_cap)
			return 1;

	return 0;
}

/* Forward decl: definition is below, near vdec_enum_framesizes */
static void vdec_cma_clamp_hevc_engine(struct amvdec_core *core,
				       u32 pixfmt, u32 *max_w, u32 *max_h);

static const struct amvdec_format *
vdec_try_fmt_common(struct amvdec_session *sess, u32 size,
		    struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *pfmt = pixmp->plane_fmt;
	const struct amvdec_format *fmts = sess->core->platform->formats;
	const struct amvdec_format *fmt_out = NULL;
	u32 output_size = 0;

	memset(pfmt[0].reserved, 0, sizeof(pfmt[0].reserved));
	memset(pixmp->reserved, 0, sizeof(pixmp->reserved));

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		fmt_out = find_format(fmts, size, pixmp->pixelformat, sess->jpeg_node);
		if (!fmt_out) {
			fmt_out = first_visible_format(fmts, size, sess->jpeg_node);
			pixmp->pixelformat = fmt_out->pixfmt;
		}
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		fmt_out = sess->fmt_out;
		break;
	default:
		return NULL;
	}

	{
		u32 max_w = fmt_out->max_width;
		u32 max_h = fmt_out->max_height;

		vdec_cma_clamp_hevc_engine(sess->core, fmt_out->pixfmt,
					   &max_w, &max_h);
		pixmp->width = clamp(pixmp->width,
				     fmt_out->min_width ? fmt_out->min_width : (u32)256,
				     max_w);
		pixmp->height = clamp(pixmp->height, (u32)144, max_h);
	}
	output_size = get_output_size(pixmp->width, pixmp->height);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (!pfmt[0].sizeimage)
			pfmt[0].sizeimage = sess->src_buffer_size;
		pfmt[0].bytesperline = 0;
		pixmp->num_planes = 1;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fmt_out = sess->fmt_out;
		if (!vdec_supports_pixfmt_cap(fmt_out, pixmp->pixelformat))
			pixmp->pixelformat = fmt_out->pixfmts_cap[0];

		memset(pfmt[1].reserved, 0, sizeof(pfmt[1].reserved));
		/* HEVC/VP9 SAO engine uses 64-byte aligned strides;
		 * VDEC_1 canvas requires >= 32-byte alignment.
		 * Use 64 to satisfy both.
		 */
		if (pixmp->pixelformat == V4L2_PIX_FMT_NV12 ||
		    pixmp->pixelformat == V4L2_PIX_FMT_NV21) {
			pfmt[0].sizeimage = output_size + output_size / 2;
			pfmt[0].bytesperline = ALIGN(pixmp->width, 64);
			pixmp->num_planes = 1;
		} else if (pixmp->pixelformat == V4L2_PIX_FMT_NV12M ||
			   pixmp->pixelformat == V4L2_PIX_FMT_NV21M) {
			pfmt[0].sizeimage = output_size;
			pfmt[0].bytesperline = ALIGN(pixmp->width, 64);

			pfmt[1].sizeimage = output_size / 2;
			pfmt[1].bytesperline = ALIGN(pixmp->width, 64);
			pixmp->num_planes = 2;
		} else if (pixmp->pixelformat == V4L2_PIX_FMT_YUV420M) {
			pfmt[0].sizeimage = output_size;
			pfmt[0].bytesperline = ALIGN(pixmp->width, 64);

			pfmt[1].sizeimage = output_size / 4;
			pfmt[1].bytesperline = ALIGN(pixmp->width, 64) / 2;

			pfmt[2].sizeimage = output_size / 4;
			pfmt[2].bytesperline = ALIGN(pixmp->width, 64) / 2;
			pixmp->num_planes = 3;
		} else if (pixmp->pixelformat == V4L2_PIX_FMT_MESON_FBC08_SCATTER ||
			   pixmp->pixelformat == V4L2_PIX_FMT_MESON_FBC10_SCATTER) {
			/* FBC body is an opaque compressed blob; bytesperline
			 * has no linear meaning.  Size is amvdec_am21c_size,
			 * which encodes resolution + bit depth via block count.
			 */
			bool is_10bit = pixmp->pixelformat ==
					V4L2_PIX_FMT_MESON_FBC10_SCATTER;

			pfmt[0].sizeimage = amvdec_am21c_size(pixmp->width,
							      pixmp->height,
							      is_10bit);
			pfmt[0].bytesperline = 0;
			pixmp->num_planes = 1;
		}
	}

	if (pixmp->field == V4L2_FIELD_ANY)
		pixmp->field = V4L2_FIELD_NONE;

	return fmt_out;
}

static int vdec_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct amvdec_session *sess =
		container_of(file->private_data, struct amvdec_session, fh);

	vdec_try_fmt_common(sess, sess->core->platform->num_formats, f);

	return 0;
}

static int vdec_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct amvdec_session *sess =
		container_of(file->private_data, struct amvdec_session, fh);
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		pixmp->pixelformat = sess->pixfmt_cap;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		pixmp->pixelformat = sess->fmt_out->pixfmt;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pixmp->width = sess->width;
		pixmp->height = sess->height;
		pixmp->colorspace = sess->colorspace;
		pixmp->ycbcr_enc = sess->ycbcr_enc;
		pixmp->quantization = sess->quantization;
		pixmp->xfer_func = sess->xfer_func;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pixmp->width = sess->width;
		pixmp->height = sess->height;
		pixmp->colorspace = sess->colorspace;
		pixmp->ycbcr_enc = sess->ycbcr_enc;
		pixmp->quantization = sess->quantization;
		pixmp->xfer_func = sess->xfer_func;
	}

	vdec_try_fmt_common(sess, sess->core->platform->num_formats, f);

	return 0;
}

static int vdec_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct amvdec_session *sess =
		container_of(file->private_data, struct amvdec_session, fh);
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	u32 num_formats = sess->core->platform->num_formats;
	const struct amvdec_format *fmt_out;
	struct v4l2_pix_format_mplane orig_pixmp;
	struct v4l2_format format;
	u32 pixfmt_out = 0, pixfmt_cap = 0;

	orig_pixmp = *pixmp;

	fmt_out = vdec_try_fmt_common(sess, num_formats, f);
	if (!fmt_out)
		return -EINVAL;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pixfmt_out = pixmp->pixelformat;
		pixfmt_cap = sess->pixfmt_cap;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pixfmt_cap = pixmp->pixelformat;
		pixfmt_out = sess->fmt_out->pixfmt;
	}

	memset(&format, 0, sizeof(format));

	format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	format.fmt.pix_mp.pixelformat = pixfmt_out;
	format.fmt.pix_mp.width = orig_pixmp.width;
	format.fmt.pix_mp.height = orig_pixmp.height;
	vdec_try_fmt_common(sess, num_formats, &format);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		sess->width = format.fmt.pix_mp.width;
		sess->height = format.fmt.pix_mp.height;
		sess->colorspace = pixmp->colorspace;
		sess->ycbcr_enc = pixmp->ycbcr_enc;
		sess->quantization = pixmp->quantization;
		sess->xfer_func = pixmp->xfer_func;
		sess->src_buffer_size = pixmp->plane_fmt[0].sizeimage;
	}

	/* Update fmt_out before capture re-validation so
	 * vdec_try_fmt_common checks pixfmt_cap against the
	 * new codec's supported capture formats, not the old one's.
	 */
	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		sess->fmt_out = fmt_out;
		vdec_dbg_log_event(sess->core, VDEC_EVT_S_FMT_OUT, 0,
				   pixfmt_out);
	}

	memset(&format, 0, sizeof(format));

	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	format.fmt.pix_mp.pixelformat = pixfmt_cap;
	format.fmt.pix_mp.width = orig_pixmp.width;
	format.fmt.pix_mp.height = orig_pixmp.height;
	vdec_try_fmt_common(sess, num_formats, &format);

	sess->width = format.fmt.pix_mp.width;
	sess->height = format.fmt.pix_mp.height;

	/* Always update pixfmt_cap from the re-validated capture format.
	 * When S_FMT(OUTPUT) changes codec, the old pixfmt_cap may not
	 * be supported by the new codec (e.g. NV12M -> YUV420M for MJPEG).
	 */
	sess->pixfmt_cap = format.fmt.pix_mp.pixelformat;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		struct vb2_queue *src_vq;

		src_vq = v4l2_m2m_get_src_vq(sess->m2m_ctx);
		if (amvdec_is_stateless(sess)) {
			src_vq->supports_requests = true;
			src_vq->min_queued_buffers = 0;
		} else {
			src_vq->supports_requests = false;
			src_vq->min_queued_buffers = 1;
		}
	}

	return 0;
}

static int vdec_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct amvdec_session *sess =
		container_of(file->private_data, struct amvdec_session, fh);
	const struct vdec_platform *platform = sess->core->platform;
	const struct amvdec_format *fmt_out;

	memset(f->reserved, 0, sizeof(f->reserved));

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		unsigned int i, visible_idx = 0;

		for (i = 0; i < platform->num_formats; i++) {
			if (!format_is_visible(&platform->formats[i], sess->jpeg_node))
				continue;
			if (visible_idx == f->index) {
				fmt_out = &platform->formats[i];
				f->pixelformat = fmt_out->pixfmt;
				f->flags = fmt_out->flags;
				return 0;
			}
			visible_idx++;
		}
		return -EINVAL;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fmt_out = sess->fmt_out;
		if (f->index >= ARRAY_SIZE(fmt_out->pixfmts_cap) ||
		    !fmt_out->pixfmts_cap[f->index])
			return -EINVAL;

		f->pixelformat = fmt_out->pixfmts_cap[f->index];
	} else {
		return -EINVAL;
	}

	return 0;
}

/*
 * HEVC-engine-formats product cap from probe-time CMA budget.  If
 * (max_w * max_h) exceeds core->hevc_engine_cma_max_pixels, shrink
 * both dimensions proportionally (preserve aspect) so the product
 * fits.  Operates in-place on *max_w, *max_h.  VDEC_1 codecs (H.264,
 * MPEG-1/2, MPEG-4, MJPEG, AVS) are not clamped -- their CMA footprint
 * is dominated by NV12 capture only, no FBC.
 */
static void vdec_cma_clamp_hevc_engine(struct amvdec_core *core,
				       u32 pixfmt, u32 *max_w, u32 *max_h)
{
	u64 budget = core->hevc_engine_cma_max_pixels;
	u64 product;
	u64 scale_1000;

	if (!budget)
		return;
	if (pixfmt != V4L2_PIX_FMT_VP9 && pixfmt != V4L2_PIX_FMT_HEVC)
		return;
	/* Scatter-gather FBC body (codec_hevc_fbc_sg=1) removes the
	 * contiguous-CMA constraint on the largest allocation in the
	 * per-session budget.  When enabled, skip the probe-time clamp
	 * and let HW capability (8K per codec_hevc_fbc_sg/amvdec_am21c_size)
	 * be the limit.  NV12 capture is still contiguous (vb2) and may
	 * still fail alloc for 8K on small-CMA boards; that's a per-open
	 * failure, not a framesize-enumeration cap.
	 */
	if (codec_hevc_fbc_sg)
		return;

	product = (u64)*max_w * *max_h;
	if (product <= budget)
		return;

	/*
	 * Scale both dimensions by sqrt(budget / product), rounded down to
	 * 64 px (HEVC/VP9 SAO alignment).  int_sqrt() returns integer part;
	 * multiply numerator by 1e6 to preserve precision through the sqrt.
	 */
	scale_1000 = int_sqrt(product * 1000000 / budget);
	if (scale_1000 == 0)
		scale_1000 = 1;
	*max_w = round_down((u32)((u64)*max_w * 1000 / scale_1000), 64);
	*max_h = round_down((u32)((u64)*max_h * 1000 / scale_1000), 64);
}

static int vdec_enum_framesizes(struct file *file, void *fh,
				struct v4l2_frmsizeenum *fsize)
{
	struct amvdec_session *sess =
		container_of(file->private_data, struct amvdec_session, fh);
	const struct amvdec_format *formats = sess->core->platform->formats;
	const struct amvdec_format *fmt;
	u32 num_formats = sess->core->platform->num_formats;
	u32 max_w, max_h;

	fmt = find_format(formats, num_formats, fsize->pixel_format, sess->jpeg_node);
	if (!fmt || fsize->index)
		return -EINVAL;

	max_w = fmt->max_width;
	max_h = fmt->max_height;
	vdec_cma_clamp_hevc_engine(sess->core, fmt->pixfmt, &max_w, &max_h);

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;

	fsize->stepwise.min_width = fmt->min_width ? fmt->min_width : 256;
	fsize->stepwise.max_width = max_w;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.min_height = 144;
	fsize->stepwise.max_height = max_h;
	fsize->stepwise.step_height = 1;

	return 0;
}

static int
vdec_decoder_cmd(struct file *file, void *fh, struct v4l2_decoder_cmd *cmd)
{
	struct amvdec_session *sess =
		container_of(file->private_data, struct amvdec_session, fh);
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;
	struct device *dev = sess->core->dev;
	int ret;

	ret = v4l2_m2m_ioctl_try_decoder_cmd(file, fh, cmd);
	if (ret)
		return ret;

	if (cmd->cmd == V4L2_DEC_CMD_START) {
		vdec_dbg_log_event(sess->core, VDEC_EVT_DEC_CMD_START, 0, 0);
		v4l2_m2m_clear_state(sess->m2m_ctx);
		sess->should_stop = 0;
		sess->aborted = 0;

		/*
		 * If drain_finalize halted the firmware (KI-VDEC-ARCH-001),
		 * un-halt it here so the next queued input buffer can resume
		 * decode in the same session without STREAMOFF->STREAMON.
		 * Only HEVC-engine codecs (VP9, HEVC) wire drain_finalize
		 * today, so the restart is HEVC-engine specific.
		 */
		if (sess->firmware_halted) {
			dev_dbg(dev,
				"DEC_CMD_START: restarting firmware after drain_finalize halt\n");
			amvdec_hevc_restart_firmware(sess->core);
			sess->firmware_halted = 0;
		}

		/*
		 * Re-schedule esparser_queue_work so any OUTPUT buffers
		 * still in the m2m rdy_queue (queued by userspace but not
		 * yet consumed by esparser before drain -- e.g. VIFIFO
		 * filled up while firmware was halting) get pushed through
		 * now that firmware is resumed.  Without this, the V4L2
		 * stateful decoder spec contract for DEC_CMD_START "resume
		 * without STREAMOFF" breaks: stuck OUTPUT buffers never
		 * transition to DONE, userspace can never DQBUF or re-QBUF
		 * them, and decode stalls with no visible error.  Stateless
		 * decoders don't use the esparser_queue_work path, so this
		 * is a no-op for them.
		 */
		if (!amvdec_is_stateless(sess) &&
		    READ_ONCE(sess->status) >= STATUS_INIT)
			schedule_work(&sess->esparser_queue_work);
		return 0;
	}

	/* Should not happen */
	if (cmd->cmd != V4L2_DEC_CMD_STOP)
		return -EINVAL;

	dev_dbg(dev, "Received V4L2_DEC_CMD_STOP\n");

	vdec_dbg_log_event(sess->core, VDEC_EVT_DEC_CMD_STOP, 0, 0);
	sess->should_stop = 1;

	if (!(sess->streamon_out & sess->streamon_cap))
		return 0;

	/*
	 * For codecs that use eos_sequence (e.g. H.264), push the EOS
	 * bitstream token into the esparser now so the firmware starts
	 * flushing.  For codecs with an active drain() callback (e.g.
	 * MPEG1/2), drain_work sends EOS itself -- do NOT send it here.
	 *
	 * NEVER call vdec_wait_inactive() or codec_ops->drain() here.
	 * Blocking in this ioctl prevents userspace from running its
	 * DQBUF/QBUF loop, which starves the firmware of canvas buffers.
	 * A stalled firmware never delivers its last frame, causing
	 * permanent 1-frame EOS loss (89/90 frames).  All drain work
	 * happens asynchronously in drain_work below.
	 */
	/*
	 * VP9 on GXL: the EOS sentinel (0x55555555 AMLV) races with
	 * decode -- the firmware's AMLV search between frames finds the
	 * sentinel mid-stream, causing spurious VP9_EOS (0xf2) and frame
	 * loss.  GXL VP9 relies on the flush_output sentinel injection
	 * instead, which runs after all frames are decoded.  G12A+ is
	 * fast enough that the sentinel is consumed after all real frames.
	 *
	 * All other codecs (H.264, MPEG1/2, etc.) need eos_sequence
	 * injected unconditionally -- their sequences are codec-specific
	 * EOS markers, not the VP9 AMLV sentinel, and the firmware drain
	 * depends on them on every platform.
	 */
	if (codec_ops->eos_sequence) {
		bool vp9_gxl_skip =
			sess->fmt_out->pixfmt == V4L2_PIX_FMT_VP9 &&
			sess->core->platform->revision < VDEC_REVISION_G12A;

		if (!vp9_gxl_skip) {
			u32 len;
			const u8 *data = codec_ops->eos_sequence(&len);

			esparser_queue_eos(sess->core, data, len);
			sess->last_irq_jiffies = get_jiffies_64();
		}
	}

	schedule_work(&sess->drain_work);

	return 0;
}

static int vdec_subscribe_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	default:
		return -EINVAL;
	}
}

static int vdec_g_pixelaspect(struct file *file, void *fh, int type,
			      struct v4l2_fract *f)
{
	struct amvdec_session *sess =
		container_of(file->private_data, struct amvdec_session, fh);

	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	*f = sess->pixelaspect;
	return 0;
}

static const struct v4l2_ioctl_ops vdec_ioctl_ops = {
	.vidioc_querycap = vdec_querycap,
	.vidioc_enum_fmt_vid_cap = vdec_enum_fmt,
	.vidioc_enum_fmt_vid_out = vdec_enum_fmt,
	.vidioc_s_fmt_vid_cap_mplane = vdec_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = vdec_s_fmt,
	.vidioc_g_fmt_vid_cap_mplane = vdec_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = vdec_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vdec_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = vdec_try_fmt,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_enum_framesizes = vdec_enum_framesizes,
	.vidioc_subscribe_event = vdec_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_try_decoder_cmd,
	.vidioc_decoder_cmd = vdec_decoder_cmd,
	.vidioc_g_pixelaspect = vdec_g_pixelaspect,
};

static int m2m_queue_init(void *priv, struct vb2_queue *src_vq,
			  struct vb2_queue *dst_vq)
{
	struct amvdec_session *sess = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->ops = &vdec_vb2_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->drv_priv = sess;
	src_vq->buf_struct_size = sizeof(struct dummy_buf);
	src_vq->min_queued_buffers = 1;
	src_vq->dev = sess->core->dev;
	src_vq->lock = &sess->lock;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->ops = &vdec_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->drv_priv = sess;
	dst_vq->buf_struct_size = sizeof(struct dummy_buf);
	dst_vq->min_queued_buffers = 1;
	dst_vq->dev = sess->core->dev;
	dst_vq->lock = &sess->lock;
	return vb2_queue_init(dst_vq);
}


#define V4L2_CID_MESON_VDEC_FAST_OUTPUT (V4L2_CID_USER_BASE + 0x1003)

#define V4L2_CID_MESON_VDEC_MAX_ALLOC_BUFS (V4L2_CID_USER_BASE + 0x1004)
#define V4L2_CID_MESON_VDEC_DPB_MARGIN     (V4L2_CID_USER_BASE + 0x1005)

static int vdec_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct amvdec_session *sess =
		container_of(ctrl->handler, struct amvdec_session, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_MESON_VDEC_DECODE_TIMEOUT:
		sess->decode_timeout_ms = ctrl->val;
		dev_dbg(sess->core->dev, "decode_timeout_ms set to %u\n",
			sess->decode_timeout_ms);
		return 0;
	case V4L2_CID_MESON_VDEC_ERROR_POLICY:
		sess->error_proc_policy = ctrl->val;
		dev_dbg(sess->core->dev, "error_proc_policy set to 0x%x\n",
			sess->error_proc_policy);
		return 0;
	case V4L2_CID_MESON_VDEC_FAST_OUTPUT:
		sess->fast_output_enable = ctrl->val;
		return 0;
	case V4L2_CID_MESON_VDEC_I_ONLY:
		/* stored via ctrl framework, read by codec at start */
		return 0;
	case V4L2_CID_MESON_VDEC_MAX_ALLOC_BUFS:
		sess->max_alloc_buf_count = ctrl->val;
		break;
	case V4L2_CID_MESON_VDEC_DPB_MARGIN:
		sess->reorder_dpb_size_margin = ctrl->val;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct v4l2_ctrl_ops vdec_ctrl_ops = {
	.s_ctrl = vdec_s_ctrl,
};

static const struct v4l2_ctrl_config vdec_decode_timeout_ctrl = {
	.ops = &vdec_ctrl_ops,
	.id = V4L2_CID_MESON_VDEC_DECODE_TIMEOUT,
	.name = "Decode Timeout (ms)",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = DECODE_TIMEOUT_MAX,
	.step = 1,
	.def = DECODE_TIMEOUT_DEFAULT,
};

static const struct v4l2_ctrl_config vdec_error_policy_ctrl = {
	.ops = &vdec_ctrl_ops,
	.id = V4L2_CID_MESON_VDEC_ERROR_POLICY,
	.name = "Error Processing Policy",
	.type = V4L2_CTRL_TYPE_BITMASK,
	.max = ERROR_PROC_POLICY_MAX,
	.def = ERROR_PROC_POLICY_DEFAULT,
};

static const struct v4l2_ctrl_config vdec_ctrl_i_only = {
	.ops	= &vdec_ctrl_ops,
	.id	= V4L2_CID_MESON_VDEC_I_ONLY,
	.name	= "I-Frame Only Decode",
	.type	= V4L2_CTRL_TYPE_BOOLEAN,
	.min	= 0,
	.max	= 1,
	.step	= 1,
	.def	= 0,
};

static int vdec_init_ctrls(struct amvdec_session *sess)
{
	struct v4l2_ctrl_handler *ctrl_handler = &sess->ctrl_handler;
	int ret;

	ret = v4l2_ctrl_handler_init(ctrl_handler, 8);
	if (ret)
		return ret;

	/*
	 * V4L2_CID_MIN_BUFFERS_FOR_CAPTURE is a stateful-decoder control that
	 * informs userspace about the DPB size requirement.  JPEG decoding is
	 * single-frame with no DPB, so the control is not applicable and must
	 * be absent (v4l2-compliance enforces this for JPEG_DECODER nodes).
	 */
	if (!sess->jpeg_node) {
		sess->ctrl_min_buf_capture =
			v4l2_ctrl_new_std(ctrl_handler, NULL,
					  V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
					  1, 32, 1, 1);
	}

	v4l2_ctrl_new_custom(ctrl_handler, &vdec_decode_timeout_ctrl, NULL);
	v4l2_ctrl_new_custom(ctrl_handler, &vdec_error_policy_ctrl, NULL);
	sess->ctrl_i_only =
		v4l2_ctrl_new_custom(ctrl_handler, &vdec_ctrl_i_only, NULL);
	v4l2_ctrl_new_custom(ctrl_handler, &(struct v4l2_ctrl_config){
		.ops = &vdec_ctrl_ops,
		.id = V4L2_CID_MESON_VDEC_MAX_ALLOC_BUFS,
		.name = "Max Allocated Buffers",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0, .max = 64, .step = 1, .def = 0,
	}, NULL);

	v4l2_ctrl_new_custom(ctrl_handler, &(struct v4l2_ctrl_config){
		.ops = &vdec_ctrl_ops,
		.id = V4L2_CID_MESON_VDEC_DPB_MARGIN,
		.name = "DPB Size Margin",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0, .max = 32, .step = 1, .def = 7,
	}, NULL);

	/* Only register stateless controls when stateless codecs are exposed */
	if (any_stateless_visible()) {
		v4l2_ctrl_new_std_compound(ctrl_handler, NULL,
					   V4L2_CID_STATELESS_MPEG2_SEQUENCE,
					   v4l2_ctrl_ptr_create(NULL));
		v4l2_ctrl_new_std_compound(ctrl_handler, NULL,
					   V4L2_CID_STATELESS_MPEG2_PICTURE,
					   v4l2_ctrl_ptr_create(NULL));
		v4l2_ctrl_new_std_compound(ctrl_handler, NULL,
					   V4L2_CID_STATELESS_MPEG2_QUANTISATION,
					   v4l2_ctrl_ptr_create(NULL));

		/* H.264 stateless control set (V4L2 Request API).
		 * Required by codec_h264_libre.c which calls
		 * v4l2_ctrl_find() for SPS/PPS/SCALING_MATRIX/SLICE_PARAMS/
		 * DECODE_PARAMS at decode time. Without these registered,
		 * userspace S_EXT_CTRLS fails with EINVAL and the firmware
		 * never boots (no QBUF-with-request -> no device_run -> no
		 * MBOX1). H264_PRED_WEIGHTS is conditional (weighted_pred
		 * only) -- driver allocates it on demand per session. */
		v4l2_ctrl_new_std_compound(ctrl_handler, NULL,
					   V4L2_CID_STATELESS_H264_SPS,
					   v4l2_ctrl_ptr_create(NULL));
		v4l2_ctrl_new_std_compound(ctrl_handler, NULL,
					   V4L2_CID_STATELESS_H264_PPS,
					   v4l2_ctrl_ptr_create(NULL));
		v4l2_ctrl_new_std_compound(ctrl_handler, NULL,
					   V4L2_CID_STATELESS_H264_SCALING_MATRIX,
					   v4l2_ctrl_ptr_create(NULL));
		v4l2_ctrl_new_std_compound(ctrl_handler, NULL,
					   V4L2_CID_STATELESS_H264_SLICE_PARAMS,
					   v4l2_ctrl_ptr_create(NULL));
		v4l2_ctrl_new_std_compound(ctrl_handler, NULL,
					   V4L2_CID_STATELESS_H264_DECODE_PARAMS,
					   v4l2_ctrl_ptr_create(NULL));
		v4l2_ctrl_new_std_menu(ctrl_handler, NULL,
				       V4L2_CID_STATELESS_H264_DECODE_MODE,
				       V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED,
				       0,
				       V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED);
		v4l2_ctrl_new_std_menu(ctrl_handler, NULL,
				       V4L2_CID_STATELESS_H264_START_CODE,
				       V4L2_STATELESS_H264_START_CODE_NONE,
				       0,
				       V4L2_STATELESS_H264_START_CODE_NONE);
	}

	/* H.264 DPB output mode: bitmask controls frame release policy.
	 * bit[0]: output on IDR, bit[1]: output on POC+1,
	 * bit[2]: output on POC+2 (even-only), bit[3]: IP-only (no reorder)
	 */
	v4l2_ctrl_new_custom(ctrl_handler, &(struct v4l2_ctrl_config){
		.ops = &vdec_ctrl_ops,
		.id = V4L2_CID_MESON_VDEC_FAST_OUTPUT,
		.name = "Fast Output Enable",
		.type = V4L2_CTRL_TYPE_BITMASK,
		.max = 0xf,
		.def = 0,
	}, NULL);

	ret = ctrl_handler->error;
	if (ret) {
		v4l2_ctrl_handler_free(ctrl_handler);
		return ret;
	}

	return 0;
}

static int vdec_open(struct file *file)
{
	struct amvdec_core *core = video_drvdata(file);
	struct device *dev = core->dev;
	const struct amvdec_format *formats = core->platform->formats;
	struct amvdec_session *sess;
	int ret;

	sess = kzalloc(sizeof(*sess), GFP_KERNEL);
	if (!sess)
		return -ENOMEM;

	sess->core = core;
	sess->jpeg_node = (video_devdata(file) == core->vdev_mjpeg);
	sess->m2m_dev = sess->jpeg_node ? core->m2m_mjpeg : core->m2m_dev;

	sess->m2m_ctx = v4l2_m2m_ctx_init(sess->m2m_dev, sess, m2m_queue_init);
	if (IS_ERR(sess->m2m_ctx)) {
		dev_err(dev, "Fail to v4l2_m2m_ctx_init\n");
		ret = PTR_ERR(sess->m2m_ctx);
		goto err_free_sess;
	}

	ret = vdec_init_ctrls(sess);
	if (ret)
		goto err_m2m_ctx_release;

	sess->fmt_out = first_visible_format(formats,
					     core->platform->num_formats,
					     sess->jpeg_node);
	sess->pixfmt_cap = sess->fmt_out->pixfmts_cap[0];
	sess->width = 1280;
	sess->height = 720;
	sess->colorspace = V4L2_COLORSPACE_SRGB;
	sess->ycbcr_enc = V4L2_YCBCR_ENC_601;
	sess->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	sess->xfer_func = V4L2_XFER_FUNC_SRGB;
	sess->pixelaspect.numerator = 1;
	sess->pixelaspect.denominator = 1;
	sess->src_buffer_size = SZ_1M;
	sess->decode_timeout_ms = DECODE_TIMEOUT_DEFAULT;
	sess->error_proc_policy = ERROR_PROC_POLICY_DEFAULT;
	sess->reorder_dpb_size_margin = 7;
	timer_setup(&sess->decode_watchdog, vdec_watchdog_cb, 0);

	INIT_LIST_HEAD(&sess->timestamps);
	INIT_LIST_HEAD(&sess->bufs_recycle);
	INIT_LIST_HEAD(&sess->pending_frames);
	spin_lock_init(&sess->pending_frames_lock);
	init_waitqueue_head(&sess->recycle_wq);
	INIT_WORK(&sess->esparser_queue_work, esparser_queue_all_src);
	init_waitqueue_head(&sess->esparser_queue_done_wq);
	INIT_WORK(&sess->drain_work, vdec_drain_work);
	mutex_init(&sess->lock);
	mutex_init(&sess->bufs_recycle_lock);
	spin_lock_init(&sess->ts_spinlock);

	v4l2_fh_init(&sess->fh, sess->jpeg_node ? core->vdev_mjpeg : core->vdev_dec);
	sess->fh.ctrl_handler = &sess->ctrl_handler;
	v4l2_fh_add(&sess->fh);
	sess->fh.m2m_ctx = sess->m2m_ctx;
	file->private_data = &sess->fh;

	vdec_dbg_log_event(core, VDEC_EVT_OPEN, 0, 0);

	return 0;

err_m2m_ctx_release:
	v4l2_m2m_ctx_release(sess->m2m_ctx);
err_free_sess:
	kfree(sess);
	return ret;
}

static int vdec_close(struct file *file)
{
	struct amvdec_session *sess =
		container_of(file->private_data, struct amvdec_session, fh);

	vdec_dbg_log_event(sess->core, VDEC_EVT_CLOSE, 0, 0);

	/*
	 * Reap recycle thread before v4l2_m2m_ctx_release frees vb2 buffers.
	 *
	 * The OUTPUT-drain early return in vdec_stop_streaming defers full
	 * teardown to a "subsequent STREAMOFF(CAPTURE)".  When CAPTURE was
	 * never streamed on or was already streamed off (typical for
	 * sessions that fail to advance past STATUS_INIT), no subsequent
	 * STREAMOFF arrives.  The recycle thread is still parked in
	 * wait_event_timeout(&sess->recycle_wq, ...) when vdec_close runs
	 * and frees sess via kfree(sess) below -- UAF on sess->recycle_wq.
	 *
	 * stop_streaming itself cannot reap the thread because it runs with
	 * sess->lock (the vb2 q->lock) held, and the thread's
	 * amvdec_flush_pending_frames calls v4l2_m2m_dst_buf_remove_by_idx
	 * which also requires that lock -- kthread_stop there deadlocks.
	 * vdec_close does not hold q->lock, so reaping is safe here.
	 *
	 * Set sess->aborted first so when kthread_stop wakes the thread
	 * out of wait_event_timeout, the iteration body short-circuits
	 * (`if (READ_ONCE(sess->aborted)) continue;`) and the thread heads
	 * straight to the kthread_should_stop check.
	 */
	if (sess->recycle_thread) {
		WRITE_ONCE(sess->aborted, 1);
		kthread_stop(sess->recycle_thread);
		sess->recycle_thread = NULL;
	}

	cancel_work_sync(&sess->esparser_queue_work);
	cancel_work_sync(&sess->drain_work);
	v4l2_m2m_ctx_release(sess->m2m_ctx);
	v4l2_fh_del(&sess->fh);
	v4l2_fh_exit(&sess->fh);

	mutex_destroy(&sess->lock);
	mutex_destroy(&sess->bufs_recycle_lock);

	/*
	 * Clear cur_sess if this session owns it.  vdec_stop_streaming()
	 * normally handles this, but there are paths where cur_sess is
	 * not cleared:
	 *  - The should_stop (drain) path skips cleanup on OUTPUT
	 *    STREAMOFF, deferring to CAPTURE STREAMOFF.  If the session
	 *    closes before CAPTURE STREAMOFF (e.g. ffmpeg exit during
	 *    drain), cur_sess is never cleared.
	 *  - If STREAMON never succeeded (EBUSY from a prior leaked
	 *    session), stop_streaming is never called.
	 *
	 * Without this, cur_sess becomes a dangling pointer after
	 * kfree(sess), causing every subsequent session to get -EBUSY
	 * on VIDIOC_STREAMON until the device node is re-opened or the
	 * module is reloaded.
	 */
	if (sess->core->cur_sess == sess)
		sess->core->cur_sess = NULL;

	kfree(sess);

	return 0;
}

static __poll_t vdec_poll(struct file *file, struct poll_table_struct *wait)
{
	struct amvdec_session *sess =
		container_of(file->private_data, struct amvdec_session, fh);
	__poll_t ret = v4l2_m2m_fop_poll(file, wait);

	/*
	 * Suppress EPOLLERR from the capture queue not being set up yet.
	 * In the stateful decoder protocol, the capture queue is not
	 * streaming until after the SOURCE_CHANGE event. GStreamer checks
	 * EPOLLERR before EPOLLPRI and would abort without seeing the event.
	 *
	 * Suppress EPOLLERR when:
	 * - A SOURCE_CHANGE event is already pending (EPOLLPRI), OR
	 * - Capture is not yet streaming (waiting for SOURCE_CHANGE)
	 */
	if ((ret & EPOLLERR) && !sess->streamon_cap)
		ret &= ~EPOLLERR;

	return ret;
}

static const struct v4l2_file_operations vdec_fops = {
	.owner = THIS_MODULE,
	.open = vdec_open,
	.release = vdec_close,
	.unlocked_ioctl = video_ioctl2,
	.poll = vdec_poll,
	.mmap = v4l2_m2m_fop_mmap,
};

static irqreturn_t vdec_isr(int irq, void *data)
{
	struct amvdec_core *core = data;
	struct amvdec_session *sess = core->cur_sess;

	if (!sess)
		return IRQ_HANDLED;

	sess->last_irq_jiffies = get_jiffies_64();
	atomic_inc(&core->debugfs.irq.total);

	return sess->fmt_out->codec_ops->isr(sess);
}

static irqreturn_t vdec_threaded_isr(int irq, void *data)
{
	struct amvdec_core *core = data;
	struct amvdec_session *sess = core->cur_sess;

	if (!sess)
		return IRQ_HANDLED;

	vdec_dbg_log_isr(sess);

	return sess->fmt_out->codec_ops->threaded_isr(sess);
}

static int vdec_request_validate(struct media_request *req)
{
	struct media_request_object *obj;
	struct amvdec_session *sess = NULL;
	unsigned int count;

	list_for_each_entry(obj, &req->objects, list) {
		if (vb2_request_object_is_buffer(obj)) {
			struct vb2_buffer *vb;

			vb = container_of(obj, struct vb2_buffer, req_obj);
			sess = vb2_get_drv_priv(vb->vb2_queue);
			break;
		}
	}

	if (!sess)
		return -ENOENT;

	count = vb2_request_buffer_cnt(req);
	if (!count)
		return -ENOENT;
	if (count > 1)
		return -EINVAL;

	return vb2_request_validate(req);
}

static const struct media_device_ops vdec_media_ops = {
	.req_validate	= vdec_request_validate,
	.req_queue	= v4l2_m2m_request_queue,
};

static const struct of_device_id vdec_dt_match[] = {
	{ .compatible = "amlogic,gxbb-vdec",
	  .data = &vdec_platform_gxbb },
	{ .compatible = "amlogic,gxm-vdec",
	  .data = &vdec_platform_gxm },
	{ .compatible = "amlogic,gxl-vdec",
	  .data = &vdec_platform_gxl },
	{ .compatible = "amlogic,gxlx-vdec",
	  .data = &vdec_platform_gxlx },
	{ .compatible = "amlogic,g12a-vdec",
	  .data = &vdec_platform_g12a },
	{ .compatible = "amlogic,g12b-vdec",
	  .data = &vdec_platform_g12b },
	{ .compatible = "amlogic,sm1-vdec",
	  .data = &vdec_platform_sm1 },
	{}
};
MODULE_DEVICE_TABLE(of, vdec_dt_match);

/* Thermal cooling device -- throttle VDEC clock on thermal pressure.
 *
 * States map to clock divider ratios from fclk_div2p5 (800 MHz):
 *   state 0 = 800 MHz (/1) -- default, max performance
 *   state 1 = 400 MHz (/2) -- moderate throttle
 *   state 2 = 267 MHz (/3) -- heavy throttle
 *   state 3 = 200 MHz (/4) -- emergency throttle
 *
 * GXL (fclk_div3 parent at 667 MHz) will round down to the nearest
 * achievable rate via the CCF divider -- no special handling needed.
 *
 * No IP pause is required during clock transitions: VDEC muxes lack
 * CLK_SET_RATE_GATE (confirmed 2026-04-10, see h264/todo.md DFS section).
 */

static const unsigned long vdec_cooling_rates[] = {
	800000000,	/* state 0: max (fclk_div2p5 on G12B+/GXL) */
	400000000,	/* state 1 */
	267000000,	/* state 2 */
	200000000,	/* state 3 */
};

#define VDEC_COOLING_MAX_STATE (ARRAY_SIZE(vdec_cooling_rates) - 1)

unsigned long amvdec_clock_rate(struct amvdec_core *core)
{
	unsigned long state = READ_ONCE(core->thermal_state);

	if (state > VDEC_COOLING_MAX_STATE)
		state = VDEC_COOLING_MAX_STATE;
	return vdec_cooling_rates[state];
}

static int vdec_cooling_get_max_state(struct thermal_cooling_device *cdev,
				      unsigned long *state)
{
	*state = VDEC_COOLING_MAX_STATE;
	return 0;
}

static int vdec_cooling_get_cur_state(struct thermal_cooling_device *cdev,
				      unsigned long *state)
{
	struct amvdec_core *core = cdev->devdata;

	*state = READ_ONCE(core->thermal_state);
	return 0;
}

static int vdec_cooling_set_cur_state(struct thermal_cooling_device *cdev,
				      unsigned long state)
{
	struct amvdec_core *core = cdev->devdata;
	unsigned long rate;

	if (state > VDEC_COOLING_MAX_STATE)
		return -EINVAL;

	if (state != READ_ONCE(core->thermal_state)) {
		WRITE_ONCE(core->thermal_state, state);
		rate = vdec_cooling_rates[state];
		dev_info(core->dev, "thermal throttle: state %lu -> %lu MHz\n",
			 state, rate / 1000000);

		/*
		 * Delegate clock updates to each engine's thermal handler.
		 * Each engine owns its clock lifecycle; the thermal callback
		 * calls through the engine API rather than touching clocks
		 * directly. clk_set_rate on a disabled clock is safe -- CCF
		 * caches the rate for the next enable.
		 */
		vdec_1_thermal_update(core, rate);
		vdec_hevc_thermal_update(core, rate);
		hcodec_thermal_update(core, rate);
	}
	return 0;
}

static const struct thermal_cooling_device_ops vdec_cooling_ops = {
	.get_max_state = vdec_cooling_get_max_state,
	.get_cur_state = vdec_cooling_get_cur_state,
	.set_cur_state = vdec_cooling_set_cur_state,
};

static int vdec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct video_device *vdev;
	struct amvdec_core *core;
	const struct of_device_id *of_id;
	int irq;
	int ret;

	core = devm_kzalloc(dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	core->dev = dev;
	platform_set_drvdata(pdev, core);
	vdec_core_instance = core;

	core->dos_base = devm_platform_ioremap_resource_byname(pdev, "dos");
	if (IS_ERR(core->dos_base))
		return PTR_ERR(core->dos_base);

	core->esparser_base = devm_platform_ioremap_resource_byname(pdev, "esparser");
	if (IS_ERR(core->esparser_base))
		return PTR_ERR(core->esparser_base);

	core->regmap_ao =
		syscon_regmap_lookup_by_phandle(dev->of_node,
						"amlogic,ao-sysctrl");
	if (IS_ERR(core->regmap_ao)) {
		dev_err(dev, "Couldn't regmap AO sysctrl\n");
		return PTR_ERR(core->regmap_ao);
	}

	core->canvas = meson_canvas_get(dev);
	if (IS_ERR(core->canvas))
		return PTR_ERR(core->canvas);

	ret = amvdec_scratch_init(core);
	if (ret) {
		dev_err(dev, "Failed to init scratch buffer\n");
		return ret;
	}

	of_id = of_match_node(vdec_dt_match, dev->of_node);
	core->platform = of_id->data;

	/*
	 * Compute HEVC-engine (VP9, HEVC) CMA-feasible resolution cap.
	 * Empirical per-pixel CMA budget at 10 capture buffers on SM1 VP9
	 * (verified 2026-04-20):
	 *   5120x2880 (14.7 MP)  decodes 30/30 bit-exact
	 *   6144x3456 (21.2 MP)  fails dma_alloc_attrs in codec_hevc_setup_buffers
	 *   7680x4320 (33.2 MP)  fails vp9_v4l2m2m open with ENOMEM
	 * Working point ~45 bytes/pixel for 10 bufs + FBC + fixed workspace.
	 *
	 * We cannot read totalcma_pages from a module (not exported), so use
	 * totalram_pages() * 1/4 as a proxy for typical Amlogic CMA sizing
	 * (cma=1G on 4GB boards, cma=256M on 1GB boards = ~25% of RAM).  Apply
	 * 70% headroom factor to leave room for concurrent allocations
	 * (GE2D, camera, mesa, fragmentation).  This over-estimates on boards
	 * with larger-than-default CMA (user passes cma=3G) and under-estimates
	 * on boards with smaller CMA, but errs toward conservative caps.
	 *
	 * PARTIAL: see struct amvdec_core "hevc_engine_cma_max_pixels"
	 * comment.  Remove or raise this cap once IOMMU/scatter-gather
	 * (cma-opt-sg in vp9/todo.md) lands and contiguous-CMA is no
	 * longer the binding constraint.
	 */
	{
		u64 ram_bytes = (u64)totalram_pages() << PAGE_SHIFT;
		u64 cma_estimate = ram_bytes / 4;
		u64 usable = cma_estimate * 70 / 100;

		core->hevc_engine_cma_max_pixels = usable / 45;
		dev_info(dev,
			 "HEVC-engine CMA-feasible max pixels: %llu MP (RAM=%llu MB, CMA estimate=%llu MB at 25%% RAM, 70%% headroom, 45 B/px)\n",
			 core->hevc_engine_cma_max_pixels >> 20,
			 ram_bytes >> 20, cma_estimate >> 20);
	}

	if (core->platform->revision == VDEC_REVISION_G12A ||
	    core->platform->revision == VDEC_REVISION_G12B ||
	    core->platform->revision == VDEC_REVISION_SM1) {
		core->vdec_hevcf_clk = devm_clk_get(dev, "vdec_hevcf");
		if (IS_ERR(core->vdec_hevcf_clk))
			return -EPROBE_DEFER;
	}

	core->dos_parser_clk = devm_clk_get(dev, "dos_parser");
	if (IS_ERR(core->dos_parser_clk))
		return -EPROBE_DEFER;

	core->dos_clk = devm_clk_get(dev, "dos");
	if (IS_ERR(core->dos_clk))
		return -EPROBE_DEFER;

	core->vdec_1_clk = devm_clk_get(dev, "vdec_1");
	if (IS_ERR(core->vdec_1_clk))
		return -EPROBE_DEFER;

	core->vdec_hevc_clk = devm_clk_get(dev, "vdec_hevc");
	if (IS_ERR(core->vdec_hevc_clk))
		return -EPROBE_DEFER;

	/* fclk_div2p5 (800 MHz) for G12A+ clock boost -- optional */
	core->fclk_div2p5 = devm_clk_get_optional(dev, "fclk_div2p5");
	if (IS_ERR(core->fclk_div2p5))
		core->fclk_div2p5 = NULL;

	/* gp0_pll (programmable) for overclock above 800 MHz -- optional */
	core->gp0_pll = devm_clk_get_optional(dev, "gp0_pll");
	if (IS_ERR(core->gp0_pll))
		core->gp0_pll = NULL;

	/* HCODEC clock is optional - encoder won't be available without it */
	core->hcodec_clk = devm_clk_get_optional(dev, "hcodec");
	if (IS_ERR(core->hcodec_clk))
		return PTR_ERR(core->hcodec_clk);
	if (!core->hcodec_clk) {
		/*
		 * Fallback: get HHI syscon regmap for direct HCODEC clock
		 * control when DT doesn't have the hcodec clock entry.
		 */
		core->regmap_hhi = syscon_regmap_lookup_by_compatible(
				"amlogic,meson-gx-hhi-sysctrl");
		if (IS_ERR(core->regmap_hhi))
			core->regmap_hhi = NULL;
	}

	irq = platform_get_irq_byname(pdev, "vdec");
	if (irq < 0)
		return irq;

	ret = devm_request_threaded_irq(core->dev, irq, vdec_isr,
					vdec_threaded_isr, IRQF_ONESHOT,
					"vdec", core);
	if (ret)
		return ret;

	core->irq = irq;

	ret = esparser_init(pdev, core);
	if (ret)
		return ret;

	/* Register as a thermal cooling device so the thermal framework can
	 * throttle VDEC clock on thermal pressure.  Non-fatal if it fails --
	 * decode works at max clock without throttling.
	 */
	core->cooling = thermal_of_cooling_device_register(dev->of_node,
			"meson-vdec", core, &vdec_cooling_ops);
	if (IS_ERR(core->cooling)) {
		dev_warn(dev, "failed to register thermal cooling device: %ld\n",
			 PTR_ERR(core->cooling));
		core->cooling = NULL;
	}

	ret = v4l2_device_register(dev, &core->v4l2_dev);
	if (ret) {
		dev_err(dev, "Couldn't register v4l2 device\n");
		return -ENOMEM;
	}

	core->m2m_dev = v4l2_m2m_init(&vdec_m2m_ops);
	if (IS_ERR(core->m2m_dev)) {
		dev_err(dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(core->m2m_dev);
		core->m2m_dev = NULL;
		goto err_v4l2_unreg;
	}

	core->mdev.dev = dev;
	strscpy(core->mdev.model, "meson-vdec", sizeof(core->mdev.model));
	strscpy(core->mdev.bus_info, "platform:meson-vdec",
		sizeof(core->mdev.bus_info));
	media_device_init(&core->mdev);
	/* Only expose the Request API when a stateless codec is active.
	 * Attaching vdec_media_ops unconditionally causes the V4L2 framework
	 * to advertise V4L2_CAP_REQUEST_API even in pure stateful mode, which
	 * trips v4l2-compliance Request API tests that are irrelevant for
	 * stateful decoders.
	 */
	if (any_stateless_visible())
		core->mdev.ops = &vdec_media_ops;
	core->v4l2_dev.mdev = &core->mdev;

	vdev = video_device_alloc();
	if (!vdev) {
		ret = -ENOMEM;
		goto err_media_cleanup;
	}

	core->vdev_dec = vdev;
	core->dev_dec = dev;
	mutex_init(&core->lock);

	/* Initialize VDEC_1 and HEVC idle autosuspend work */
	vdec_1_power_init(core);
	vdec_hevc_power_init(core);

	strscpy(vdev->name, "meson-video-decoder", sizeof(vdev->name));
	vdev->release = video_device_release;
	vdev->fops = &vdec_fops;
	vdev->ioctl_ops = &vdec_ioctl_ops;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->v4l2_dev = &core->v4l2_dev;
	vdev->lock = &core->lock;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;

	video_set_drvdata(vdev, core);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(dev, "Failed registering video device\n");
		goto err_vdev_release;
	}

	ret = v4l2_m2m_register_media_controller(core->m2m_dev, vdev,
						 MEDIA_ENT_F_PROC_VIDEO_DECODER);
	if (ret) {
		dev_err(dev, "Failed to register media controller\n");
		goto err_video_unreg;
	}

	ret = media_device_register(&core->mdev);
	if (ret) {
		dev_err(dev, "Failed to register media device\n");
		goto err_mc_unreg;
	}

	vdec_debugfs_init(core);

	/* Register MJPEG-only device node (separate from main decoder node to
	 * fix v4l2-compliance: JPEG_DECODER and STATEFUL_DECODER must not both
	 * be set on the same device node).  No media controller needed here
	 * since MJPEG is stateful and does not use the Request API.
	 */
	core->m2m_mjpeg = v4l2_m2m_init(&vdec_m2m_ops);
	if (IS_ERR(core->m2m_mjpeg)) {
		ret = PTR_ERR(core->m2m_mjpeg);
		core->m2m_mjpeg = NULL;
		goto err_media_unreg;
	}

	vdev = video_device_alloc();
	if (!vdev) {
		ret = -ENOMEM;
		goto err_mjpeg_m2m;
	}

	core->vdev_mjpeg = vdev;
	strscpy(vdev->name, "meson-video-decoder-jpeg", sizeof(vdev->name));
	vdev->release = video_device_release;
	vdev->fops = &vdec_fops;
	vdev->ioctl_ops = &vdec_ioctl_ops;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->v4l2_dev = &core->v4l2_dev;
	vdev->lock = &core->lock;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	video_set_drvdata(vdev, core);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(dev, "Failed registering MJPEG video device\n");
		goto err_mjpeg_vdev;
	}

	ret = v4l2_m2m_register_media_controller(core->m2m_mjpeg, vdev,
						  MEDIA_ENT_F_PROC_VIDEO_DECODER);
	if (ret) {
		dev_err(dev, "Failed to register MJPEG media controller: %d\n",
			ret);
		goto err_mjpeg_mc;
	}

	/* Initialize HCODEC encoder if clock is available (DT or direct) */
	if (core->hcodec_clk || core->regmap_hhi) {
		int hcodec_irq;

		hcodec_irq = platform_get_irq_byname_optional(pdev, "hcodec");
		if (hcodec_irq < 0) {
			/*
			 * Fallback: HCODEC uses GIC SPI 45 (Mailbox 2).
			 * Try to get it from the irq domain.
			 */
			struct irq_fwspec fwspec;
			struct device_node *gic;

			gic = of_irq_find_parent(dev->of_node);
			if (gic) {
				fwspec.fwnode = of_node_to_fwnode(gic);
				fwspec.param_count = 3;
				fwspec.param[0] = 0; /* GIC_SPI */
				fwspec.param[1] = 45;
				fwspec.param[2] = IRQ_TYPE_EDGE_RISING;
				hcodec_irq = irq_create_fwspec_mapping(&fwspec);
				of_node_put(gic);
			}
		}
		if (hcodec_irq > 0) {
			ret = devm_request_threaded_irq(dev, hcodec_irq,
							amvenc_isr,
							amvenc_threaded_isr,
							IRQF_ONESHOT,
							"hcodec", core);
			if (ret) {
				dev_warn(dev,
					 "Failed to request HCODEC IRQ: %d\n",
					 ret);
			} else {
				hcodec_power_init(core);
				ret = amvenc_init(core);
				if (ret)
					dev_warn(dev,
						 "Encoder init failed: %d\n",
						 ret);
			}
		}
	}

	vdec_diag_init(core);

	return 0;

err_mjpeg_mc:
	video_unregister_device(core->vdev_mjpeg);
	goto err_mjpeg_m2m;
err_mjpeg_vdev:
	video_device_release(core->vdev_mjpeg);
err_mjpeg_m2m:
	v4l2_m2m_release(core->m2m_mjpeg);
err_media_unreg:
	media_device_unregister(&core->mdev);
err_mc_unreg:
	v4l2_m2m_unregister_media_controller(core->m2m_dev);
err_video_unreg:
	video_unregister_device(core->vdev_dec);
	goto err_media_cleanup;
err_vdev_release:
	video_device_release(vdev);
err_media_cleanup:
	media_device_cleanup(&core->mdev);
	v4l2_m2m_release(core->m2m_dev);
err_v4l2_unreg:
	v4l2_device_unregister(&core->v4l2_dev);
	return ret;
}

static void vdec_remove(struct platform_device *pdev)
{
	struct amvdec_core *core = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&core->hcodec_poweroff_work);
	if (core->hcodec_powered)
		hcodec_poweroff(core);
	cancel_delayed_work_sync(&core->vdec1_poweroff_work);
	if (core->vdec1_powered)
		vdec_1_poweroff(core);
	cancel_delayed_work_sync(&core->hevc_poweroff_work);
	if (core->hevc_powered)
		vdec_hevc_power_off(core);
	vdec_debugfs_remove(core);
	vdec_diag_exit(core);
	vdec_core_instance = NULL;
	if (core->cooling)
		thermal_cooling_device_unregister(core->cooling);
	amvenc_cleanup(core);
	if (core->vdev_mjpeg) {
		v4l2_m2m_unregister_media_controller(core->m2m_mjpeg);
		video_unregister_device(core->vdev_mjpeg);
		v4l2_m2m_release(core->m2m_mjpeg);
	}
	media_device_unregister(&core->mdev);
	v4l2_m2m_unregister_media_controller(core->m2m_dev);
	video_unregister_device(core->vdev_dec);
	media_device_cleanup(&core->mdev);
	v4l2_m2m_release(core->m2m_dev);
	v4l2_device_unregister(&core->v4l2_dev);
	amvdec_scratch_free(core);
}

static struct platform_driver meson_vdec_driver = {
	.probe = vdec_probe,
	.remove_new = vdec_remove,
	.driver = {
		.name = "meson-vdec",
		.of_match_table = vdec_dt_match,
	},
};
module_platform_driver(meson_vdec_driver);

/*
 * Codec registry -- allows per-codec modules to register/unregister their
 * amvdec_codec_ops at module load/unload time.  The core's format tables
 * start with codec_ops = NULL for dynamically-registered codecs; registration
 * fills them in, unregistration NULLs them out.
 *
 * The vdec_dt_match table lists all platforms; each platform's format array
 * is scanned on register/unregister.
 */

static DEFINE_MUTEX(codec_registry_lock);

/* Scan a format array and set/clear codec_ops for matching pixfmt entries */
static int codec_registry_scan(struct amvdec_format *fmts, u32 num,
			       u32 pixfmt, struct amvdec_codec_ops *ops,
			       bool set)
{
	int j, found = 0;

	for (j = 0; j < num; j++) {
		struct amvdec_format *fmt = &fmts[j];

		if (set) {
			if (fmt->pixfmt == pixfmt && !fmt->codec_ops) {
				fmt->codec_ops = ops;
				found++;
			}
		} else {
			if (fmt->pixfmt == pixfmt && fmt->codec_ops == ops) {
				fmt->codec_ops = NULL;
				found++;
			}
		}
	}
	return found;
}

int amvdec_register_codec(u32 pixfmt, struct amvdec_codec_ops *ops)
{
	int i, found = 0;

	mutex_lock(&codec_registry_lock);
	for (i = 0; vdec_dt_match[i].compatible[0]; i++) {
		const struct vdec_platform *platform = vdec_dt_match[i].data;

		found += codec_registry_scan(platform->formats,
					     platform->num_formats,
					     pixfmt, ops, true);
		if (platform->formats_stateless)
			found += codec_registry_scan(platform->formats_stateless,
						     platform->num_formats_stateless,
						     pixfmt, ops, true);
	}
	mutex_unlock(&codec_registry_lock);

	return found ? 0 : -ENOENT;
}
EXPORT_SYMBOL_GPL(amvdec_register_codec);

int amvdec_unregister_codec(u32 pixfmt, struct amvdec_codec_ops *ops)
{
	int i;

	mutex_lock(&codec_registry_lock);

	/* Refuse if a session is actively using this codec */
	if (vdec_core_instance && vdec_core_instance->cur_sess) {
		const struct amvdec_format *active_fmt =
			vdec_core_instance->cur_sess->fmt_out;
		if (active_fmt && active_fmt->codec_ops == ops) {
			mutex_unlock(&codec_registry_lock);
			return -EBUSY;
		}
	}

	for (i = 0; vdec_dt_match[i].compatible[0]; i++) {
		const struct vdec_platform *platform = vdec_dt_match[i].data;

		codec_registry_scan(platform->formats,
				    platform->num_formats,
				    pixfmt, ops, false);
		if (platform->formats_stateless)
			codec_registry_scan(platform->formats_stateless,
					    platform->num_formats_stateless,
					    pixfmt, ops, false);
	}
	mutex_unlock(&codec_registry_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(amvdec_unregister_codec);

MODULE_DESCRIPTION("Meson video decoder driver for GXBB/GXL/GXM/G12/SM1");
MODULE_AUTHOR("Maxime Jourdan <mjourdan@baylibre.com>");
MODULE_LICENSE("GPL");
