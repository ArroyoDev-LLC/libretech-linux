/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#ifndef __MESON_VDEC_CORE_H_
#define __MESON_VDEC_CORE_H_

#include <linux/irqreturn.h>
#include <linux/regmap.h>
#include <linux/list.h>
#include <media/media-device.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/soc/amlogic/meson-canvas.h>

#include "vdec_platform.h"
#include "vdec_debugfs.h"

/**
 * struct amvdec_fw_version - identity + quirks for one known firmware binary
 * @label: human-readable identifier (e.g. "BSP V2603 V2.5")
 * @provenance: upstream commit or BSP release string
 * @size: expected byte size of the firmware
 * @sha256: full 32-byte SHA256 of the firmware
 * @quirks: codec-specific bitmask (each codec defines its own bits)
 * @latest: true on the newest entry for this firmware file
 */
struct amvdec_fw_version {
	const char *label;
	const char *provenance;
	size_t size;
	u8 sha256[32];
	u32 quirks;
	bool latest;
};

/* Custom V4L2 controls for meson-vdec tuning */
#define V4L2_CID_MESON_VDEC_MAX_ALLOC_BUFS  (V4L2_CID_USER_BASE + 0x1004)
#define V4L2_CID_MESON_VDEC_DPB_MARGIN      (V4L2_CID_USER_BASE + 0x1005)

/* 32 buffers in 3-plane YUV420 */
#define MAX_CANVAS (32 * 3)

struct amvdec_buffer {
	struct list_head list;
	struct vb2_buffer *vb;
};

/**
 * struct amvdec_timestamp - stores a src timestamp along with a VIFIFO offset
 *
 * @list: used to make lists out of this struct
 * @tc: timecode from the v4l2 buffer
 * @ts: timestamp from the VB2 buffer
 * @offset: offset in the VIFIFO where the associated packet was written
 * @flags: flags from the v4l2 buffer
 * @used_count: times this timestamp was checked for a match with a dst buffer
 */
struct amvdec_timestamp {
	struct list_head list;
	struct v4l2_timecode tc;
	u64 ts;
	u32 offset;
	u32 flags;
	u32 used_count;
};

struct amvdec_session;

/**
 * struct amvdec_core - device parameters, singleton
 *
 * @dos_base: DOS memory base address
 * @esparser_base: PARSER memory base address
 * @regmap_ao: regmap for the AO bus
 * @dev: core device
 * @dev_dec: decoder device
 * @platform: platform-specific data
 * @canvas: canvas provider reference
 * @dos_parser_clk: DOS_PARSER clock
 * @dos_clk: DOS clock
 * @vdec_1_clk: VDEC_1 clock
 * @vdec_hevc_clk: VDEC_HEVC clock
 * @vdec_hevcf_clk: VDEC_HEVCF clock
 * @esparser_reset: RESET for the PARSER
 * @vdev_dec: video device for the decoder
 * @v4l2_dev: v4l2 device
 * @cur_sess: current decoding session
 * @lock: video device lock
 */
struct amvdec_core {
	void __iomem *dos_base;
	void __iomem *esparser_base;
	struct regmap *regmap_ao;

	struct device *dev;
	struct device *dev_dec;
	const struct vdec_platform *platform;

	struct meson_canvas *canvas;

	/* Scratch buffer for safe decoder output before real buffers exist */
	void *scratch_vaddr;
	dma_addr_t scratch_paddr;
	u8 scratch_canvas[3]; /* Y, U/UV, V canvas IDs */

	struct clk *dos_parser_clk;
	struct clk *dos_clk;
	struct clk *vdec_1_clk;
	struct clk *vdec_hevc_clk;
	struct clk *vdec_hevcf_clk;
	struct clk *fclk_div2p5;	/* 800 MHz fixed divider (G12A+) */
	struct clk *gp0_pll;		/* programmable PLL for overclock (G12A+) */
	atomic_t hevc_power_ref;	/* refcount: HEVC domain users (VDEC_1 + HEVC) */

	struct reset_control *esparser_reset;
	struct reset_control *hevcf_dmc_pipl_reset;

	struct video_device *vdev_dec;
	struct video_device *vdev_mjpeg;	/* MJPEG-only device node */
	struct v4l2_device v4l2_dev;
	struct media_device mdev;
	struct v4l2_m2m_dev *m2m_dev;
	struct v4l2_m2m_dev *m2m_mjpeg;	/* m2m device for MJPEG node */

	struct amvdec_session *cur_sess;
	int irq;			/* vdec IRQ number for synchronize_irq() */
	struct mutex lock;

	/* Diagnostic hot-reload test bench (vdec_diag.c).  NULL when
	 * diag=0; opaque struct vdec_diag * when diag=1. */
	void *diag;

	/* Thermal throttling */
	struct thermal_cooling_device *cooling;
	unsigned long thermal_state;	/* 0=max, higher=slower */

	/* HCODEC encoder */
	struct clk *hcodec_clk;
	struct regmap *regmap_hhi;	/* HHI syscon for direct clock control */
	struct video_device *vdev_enc;
	struct v4l2_m2m_dev *m2m_enc;
	struct mutex enc_lock;		/* encoder video device lock */
	struct mutex hw_lock;		/* serialize HCODEC HW access */
	atomic_t enc_in_use;		/* single-instance enforcement (KI-028) */
	struct completion enc_done;
	struct completion enc_run_done;	/* device_run finished (KI-025) */
	atomic_t enc_aborting;		/* signal device_run to abort (KI-025) */
	u32 enc_status;
	bool enc_need_reset;
	bool hcodec_powered;		/* true while HCODEC domain is on */
	struct delayed_work hcodec_poweroff_work; /* idle autosuspend */
	bool vdec1_powered;		/* true while VDEC_1 domain is on */
	struct delayed_work vdec1_poweroff_work; /* VDEC_1 idle autosuspend */
	bool hevc_powered;		/* true while HEVC domain is on */
	struct delayed_work hevc_poweroff_work; /* HEVC idle autosuspend */

	/* Debugfs diagnostic interface (/sys/kernel/debug/meson-vdec/) */
	struct vdec_debugfs debugfs;

	/*
	 * HEVC-engine (HEVC + VP9) CMA-feasible resolution cap.  Computed
	 * at probe from totalcma_pages.  HEVC-engine formats advertise
	 * max_width/max_height = silicon max (8192x4320 on SM1); actual CMA
	 * determines what board-size fits.  hevc_engine_cma_max_pixels is
	 * the product cap enforced in enum_framesizes + try_fmt.  Zero =
	 * uncapped (fall back to compile-time max).
	 *
	 * PARTIAL RESOLUTION: this clamp exists because VDEC buffers are
	 * backed by contiguous CMA.  Once IOMMU / scatter-gather lands
	 * (cma-opt-sg in vp9/todo.md), FBC bodies can be backed by 4K-page
	 * SG lists via the HEVC MMU and contiguous-CMA is no longer the
	 * binding constraint.  At that point remove this cap or raise it
	 * to the silicon max.
	 */
	u64 hevc_engine_cma_max_pixels;
};

/**
 * struct amvdec_ops - vdec operations
 *
 * @start: mandatory call when the vdec needs to initialize
 * @stop: mandatory call when the vdec needs to stop
 * @conf_esparser: mandatory call to let the vdec configure the ESPARSER
 * @vififo_level: mandatory call to get the current amount of data
 *		  in the VIFIFO
 */
struct amvdec_ops {
	int (*start)(struct amvdec_session *sess);
	int (*stop)(struct amvdec_session *sess);
	void (*conf_esparser)(struct amvdec_session *sess);
	u32 (*vififo_level)(struct amvdec_session *sess);
};

/**
 * struct amvdec_codec_ops - codec operations
 *
 * @start: mandatory call when the codec needs to initialize
 * @post_start: optional call after firmware is running (MPSR=1 + settle)
 * @stop: mandatory call when the codec needs to stop
 * @load_extended_firmware: optional call to load additional firmware bits
 * @num_pending_bufs: optional call to get the number of dst buffers on hold
 * @can_recycle: optional call to know if the codec is ready to recycle
 *		 a dst buffer
 * @recycle: optional call to tell the codec to recycle a dst buffer. Must go
 *	     in pair with @can_recycle
 * @drain: optional call if the codec has a custom way of draining
 * @resume: optional call to resume after a resolution change
 * @eos_sequence: optional call to get an end sequence to send to esparser
 *		  for flush. Mutually exclusive with @drain.
 * @setup_buffers: optional call to set up capture buffers (canvases, etc.)
 *		   before the first frame is decoded. Called once after
 *		   STATUS_RUNNING is entered. Stateless codecs use this.
 * @set_dst_buf: optional call to tell firmware which dst buffer to write to.
 *		 Called per frame for stateless codecs.
 * @isr: mandatory call when the ISR triggers
 * @threaded_isr: mandatory call for the threaded ISR
 * @early_irq_enable: if true, MBOX1 IRQ is enabled BEFORE firmware start
 *   (MPSR=1) rather than after.  Use only for codecs whose firmware does
 *   NOT fire spurious MBOX1 interrupts during init.  MJPEG is safe:
 *   firmware busy-waits for MREG_TO_AMRISC before decoding.  MPEG12 is
 *   NOT safe (fires MCPU_INTR_REQ=4 at addr 0x052f when VIFIFO has data).
 */
struct amvdec_codec_ops {
	int (*start)(struct amvdec_session *sess);
	void (*post_start)(struct amvdec_session *sess);
	int (*stop)(struct amvdec_session *sess);
	int (*load_extended_firmware)(struct amvdec_session *sess,
				      const u8 *data, u32 len, u32 mc_size);
	u32 (*num_pending_bufs)(struct amvdec_session *sess);
	int (*can_recycle)(struct amvdec_core *core);
	void (*recycle)(struct amvdec_core *core, u32 buf_idx);
	void (*drain)(struct amvdec_session *sess);
	/*
	 * Optional.  Called by vdec_drain_work() after all post-drain waits
	 * (esparser counter, vdec_wait_inactive, flush_pending_frames) and
	 * BEFORE the LAST buffer is delivered.  Intended for codecs whose
	 * firmware does not self-halt after the last frame is consumed
	 * (HEVC-engine VIFIFO-search loops on gxl_vp9_mmu / gxl_hevc_mmu).
	 *
	 * A drain_finalize implementation should (a) halt the firmware to
	 * end the post-EOS CPU-waste / spurious-IRQ behavior and (b) close
	 * the esparser_queued_bufs protocol (atomic_set to 0) now that no
	 * more ISRs can decrement it.  After drain_finalize returns, the
	 * firmware must be in a state where the session teardown
	 * (codec_ops->stop at STREAMOFF) is a no-op or idempotent.
	 *
	 * Leaving this NULL preserves the legacy "firmware runs until
	 * STREAMOFF" behavior.  See KI-VDEC-ARCH-001.
	 */
	void (*drain_finalize)(struct amvdec_session *sess);
	void (*resume)(struct amvdec_session *sess);
	const u8 * (*eos_sequence)(u32 *len);
	int (*pre_queue)(struct amvdec_session *sess, struct vb2_buffer *vb);
	int (*setup_buffers)(struct amvdec_session *sess);
	void (*set_dst_buf)(struct amvdec_session *sess, u32 fw_idx);
	irqreturn_t (*isr)(struct amvdec_session *sess);
	irqreturn_t (*threaded_isr)(struct amvdec_session *sess);
	bool early_irq_enable;
};

/**
 * struct amvdec_format - describes one of the OUTPUT (src) format supported
 *
 * @pixfmt: V4L2 pixel format
 * @min_buffers: minimum amount of CAPTURE (dst) buffers
 * @max_buffers: maximum amount of CAPTURE (dst) buffers
 * @min_width: minimum picture width supported (0 = use default 256)
 * @max_width: maximum picture width supported
 * @max_height: maximum picture height supported
 * @flags: enum flags associated with this pixfmt
 * @vdec_ops: the VDEC operations that support this format
 * @codec_ops: the codec operations that support this format
 * @firmware_path: Path to the firmware that supports this format
 * @firmware_tag: Identifies firmware binary for skip-reload optimization.
 *   0 = always reload (stateful, one firmware per codec).
 *   Non-zero = skip IMEM reload if tag matches what is already loaded
 *   (libre stateless multi-codec firmware: one binary per engine).
 * @pixfmts_cap: list of CAPTURE pixel formats available with pixfmt
 */
struct amvdec_format {
	u32 pixfmt;
	u32 min_buffers;
	u32 max_buffers;
	u32 min_width;
	u32 max_width;
	u32 max_height;
	u32 max_mb_total;	/* firmware MB budget limit (0 = no limit); checked in
				 * pre_queue from SPS-parsed dims. Replaces per-dimension
				 * firmware limits where constraint is total MBs not W or H. */
	u32 flags;

	struct amvdec_ops *vdec_ops;
	struct amvdec_codec_ops *codec_ops;

	char *firmware_path;
	u32 firmware_tag;
	u32 pixfmts_cap[8];

	/* Firmware version identification table (per-codec, optional) */
	const struct amvdec_fw_version *fw_versions;
	size_t fw_versions_count;
};

enum amvdec_status {
	STATUS_STOPPED,
	STATUS_NEEDS_START, /* VIFIFO allocated, waiting for capture buffers */
	STATUS_INIT,
	STATUS_RUNNING,
	STATUS_NEEDS_RESUME,
};

/**
 * struct amvdec_session - decoding session parameters
 *
 * @core: reference to the vdec core struct
 * @fh: v4l2 file handle
 * @m2m_dev: v4l2 m2m device
 * @m2m_ctx: v4l2 m2m context
 * @ctrl_handler: V4L2 control handler
 * @ctrl_min_buf_capture: V4L2 control V4L2_CID_MIN_BUFFERS_FOR_CAPTURE
 * @lock: cap & out queues lock
 * @fmt_out: vdec pixel format for the OUTPUT queue
 * @pixfmt_cap: V4L2 pixel format for the CAPTURE queue
 * @src_buffer_size: size in bytes of the OUTPUT buffers' only plane
 * @width: current picture width
 * @height: current picture height
 * @colorspace: current colorspace
 * @ycbcr_enc: current ycbcr_enc
 * @quantization: current quantization
 * @xfer_func: current transfer function
 * @pixelaspect: Pixel Aspect Ratio reported by the decoder
 * @esparser_queued_bufs: number of buffers currently queued into ESPARSER
 * @esparser_queue_work: work struct for the ESPARSER to process src buffers
 * @streamon_cap: stream on flag for capture queue
 * @streamon_out: stream on flag for output queue
 * @sequence_cap: capture sequence counter
 * @sequence_out: output sequence counter
 * @should_stop: flag set if userspace signaled EOS via command
 *		 or empty buffer
 * @firmware_halted: set by codec_ops->drain_finalize after it halts the
 *		 firmware via amvdec_hevc_halt_firmware().  Cleared on
 *		 firmware restart (V4L2_DEC_CMD_START path via
 *		 amvdec_hevc_restart_firmware, or implicitly at fresh
 *		 STREAMON).  See KI-VDEC-ARCH-001.
 * @aborted: session abort flag (unsupported stream); firmware halted
 * @keyframe_found: flag set once a keyframe has been parsed
 * @num_dst_bufs: number of destination buffers
 * @changed_format: the format changed
 * @canvas_alloc: array of all the canvas IDs allocated
 * @canvas_num: number of canvas IDs allocated
 * @vififo_vaddr: virtual address for the VIFIFO
 * @vififo_paddr: physical address for the VIFIFO
 * @vififo_size: size of the VIFIFO dma alloc
 * @bufs_recycle: list of buffers that need to be recycled
 * @bufs_recycle_lock: lock for the bufs_recycle list
 * @recycle_thread: task struct for the recycling thread
 * @timestamps: chronological list of src timestamps
 * @ts_spinlock: spinlock for the timestamps list
 * @last_irq_jiffies: tracks last time the vdec triggered an IRQ
 * @last_offset: tracks last offset of vififo
 * @wrap_count: number of times the vififo wrapped around
 * @fw_idx_to_vb2_idx: firmware buffer index to vb2 buffer index
 * @status: current decoding status
 * @priv: codec private data
 */

struct amvdec_pending_frame {
	struct list_head list;
	u32 buf_idx;
	u32 offset;
	u32 field;
	bool is_error;
};

struct amvdec_session {
	struct amvdec_core *core;

	struct v4l2_fh fh;
	struct v4l2_m2m_dev *m2m_dev;
	struct v4l2_m2m_ctx *m2m_ctx;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *ctrl_min_buf_capture;
	struct v4l2_ctrl *ctrl_i_only;
	struct mutex lock;

	const struct amvdec_format *fmt_out;
	u32 pixfmt_cap;
	u32 src_buffer_size;

	u32 width;
	u32 height;
	u32 colorspace;
	u8 ycbcr_enc;
	u8 quantization;
	u8 xfer_func;

	struct v4l2_fract pixelaspect;

	atomic_t esparser_queued_bufs;
	wait_queue_head_t esparser_queue_done_wq;
	struct work_struct esparser_queue_work;

	unsigned int streamon_cap, streamon_out;
	unsigned int sequence_cap, sequence_out;
	unsigned int should_stop;
	unsigned int firmware_halted;
	unsigned int aborted;
	unsigned int keyframe_found;
	unsigned int num_dst_bufs;
	unsigned int changed_format;

	u8 canvas_alloc[MAX_CANVAS];
	u32 canvas_num;

	void *vififo_vaddr;
	dma_addr_t vififo_paddr;
	u32 vififo_size;

	struct list_head bufs_recycle;
	struct mutex bufs_recycle_lock; /* bufs_recycle list lock */
	struct task_struct *recycle_thread;
	wait_queue_head_t recycle_wq; /* wakes recycle_thread on QBUF */

	/* Frames decoded by firmware but not yet delivered because all
	 * capture buffers were in userspace.  Retried by recycle_thread.
	 */
	struct list_head pending_frames; /* struct amvdec_pending_frame */
	spinlock_t pending_frames_lock;

	struct list_head timestamps;
	spinlock_t ts_spinlock; /* timestamp list lock */

	u64 last_irq_jiffies;
	u64 last_ts_done; /* last timestamp successfully delivered, for fallback */
	u32 last_offset;
	u32 wrap_count;
	u32 fw_idx_to_vb2_idx[32];

	struct work_struct drain_work;

	enum amvdec_status status;
	atomic_t resumed; /* guards against double resume() calls */
	u32 fw_size; /* firmware size in bytes, set by vdec_hevc_load_firmware */

	/* Matched firmware version entry, set by vdec_1_load_firmware */
	const struct amvdec_fw_version *fw_match;
	unsigned int fast_output_enable; /* DPB output mode bitmask (bit3=IP-only) */
	void *priv;

	/*
	 * Decode stall timeout in ms (0 = disabled).
	 * Host-side timer checks firmware macroblock progress.
	 * BSP vmh264.c default: 100.  Exposed via
	 * V4L2_CID_MESON_VDEC_DECODE_TIMEOUT.
	 */
	u32 decode_timeout_ms;

	/*
	 * Framework-level decode watchdog. Fires every 200ms; if
	 * last_irq_jiffies hasn't advanced in decode_timeout_ms, aborts.
	 * Works for ALL codecs (H.264 has an additional per-MB watchdog
	 * in codec_h264.c that catches MBY_MBX stalls within a frame).
	 */
	struct timer_list decode_watchdog;

	/**
	 * @error_proc_policy: 32-bit bitmask controlling error handling.
	 *
	 * BSP vmh264.c default: 0x7fcff6.  Exposed via
	 * V4L2_CID_MESON_VDEC_ERROR_POLICY.
	 */
	u32 error_proc_policy;
	unsigned int max_alloc_buf_count;      /* V4L2 ctrl: 0 = unlimited */
	unsigned int reorder_dpb_size_margin;  /* V4L2 ctrl: default 7 */

	/* True if this session was opened on the MJPEG device node. */
	bool jpeg_node;
};

u32 amvdec_get_output_size(struct amvdec_session *sess);

/**
 * amvdec_register_codec() - Register a codec's ops for a given pixel format.
 *
 * Scans all platform format tables and fills in codec_ops for entries whose
 * pixfmt matches. Called from per-codec module_init().
 *
 * @pixfmt: V4L2 pixel format (e.g. V4L2_PIX_FMT_H264)
 * @ops: codec operations struct
 * Returns 0 on success, -ENOENT if no format entry matches.
 */
int amvdec_register_codec(u32 pixfmt, struct amvdec_codec_ops *ops);

/**
 * amvdec_unregister_codec() - Unregister a codec's ops.
 *
 * NULLs out codec_ops for all format entries matching pixfmt + ops.
 * Refuses if a session is actively using this codec (returns -EBUSY).
 *
 * @pixfmt: V4L2 pixel format
 * @ops: codec operations struct (must match what was registered)
 * Returns 0 on success, -EBUSY if codec is in active use.
 */
int amvdec_unregister_codec(u32 pixfmt, struct amvdec_codec_ops *ops);

/**
 * amvdec_clock_rate() - get the target VDEC clock rate respecting thermal cap
 * @core: the amvdec_core instance
 *
 * Returns the maximum clock rate (Hz) that vdec_1_start / vdec_hevc_start
 * should request.  Normally 800 MHz; reduced when the thermal framework
 * throttles the VDEC cooling device.
 */
unsigned long amvdec_clock_rate(struct amvdec_core *core);

#endif
