/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 Libre Computer
 *
 * Debugfs diagnostic interface for meson-vdec.
 * Provides live register snapshots, IRQ counters, and stall dump
 * at /sys/kernel/debug/meson-vdec/.
 */

#ifndef __VDEC_DEBUGFS_H__
#define __VDEC_DEBUGFS_H__

#include <linux/debugfs.h>
#include <linux/atomic.h>
#include <linux/spinlock.h>
#include <linux/types.h>

struct amvdec_core;
struct amvdec_session;

/* Per-status-code IRQ counters (atomic -- safe from hardirq context) */
struct vdec_dbg_irq_counters {
	atomic_t h264_src_change;
	atomic_t h264_frames_ready;
	atomic_t hevc_head_parser_done;
	atomic_t vp9_head_parser_done;
	atomic_t vp9_eos;
	atomic_t total;
};

/* Last stall dump (captured at watchdog fire time) */
struct vdec_dbg_stall_dump {
	u64 timestamp_ns;
	bool valid;
	u32 scratch[22];	/* AV_SCRATCH_0 .. AV_SCRATCH_L */
	u32 mby_mbx;
	u32 vififo_rp;
	u32 vififo_wp;
	u32 vififo_level;
	u32 mpsr;
	u32 width;
	u32 height;
};

#define VDEC_DBG_SESSION_LOG_SIZE  256
#define VDEC_DBG_ESPARSER_LOG_SIZE 256
#define VDEC_DBG_ISR_LOG_SIZE      256
#define VDEC_DBG_MPRED_LOG_SIZE    256

/* Per-slice MPRED setup snapshot (captured from codec_hevc_set_mpred and
 * codec_vp9_set_mpred).  Enables upstream-vs-BSP diffing for HEVC B-frame
 * decode investigations (TODO-HEVC-GXL-BFRAME-CONTENT) and G12B/SM1 POC=6
 * wedge investigation (TASK-004) without a rebuild per hypothesis.
 *
 * Rationale: HEVC is a spec-bit-exact codec.  When HW output differs from
 * SW reference only on B-slices, the divergence must be in one of
 * (a) CTRL0 bit composition, (b) the ref list L0/L1 contents, or
 * (c) MV buffer addressing.  This struct captures all three at every slice
 * so a session-log read shows the full MPRED setup for comparison.
 */
#define VDEC_DBG_MPRED_MAX_REFS 16
struct vdec_dbg_mpred_entry {
	u64 timestamp_ns;
	u8  codec;			/* 0 = hevc, 1 = vp9 */
	u8  slice_type;			/* 0=B, 1=P, 2=I */
	u8  new_pic;
	u8  col_from_l0;
	s32 curr_poc;
	u8  ref_num_l0;
	u8  ref_num_l1;
	u8  ldc_flag;			/* computed per BSP vh265.c:8042 */
	u8  tmvp_flag;
	u8  num_neg;			/* st_curr0 count from apply_rps */
	u8  num_pos;			/* st_curr1 count from apply_rps */
	u8  mod_flag;			/* rpm_param.p.modification_flag */
	u8  num_ref_idx_l0_active;	/* rpm_param.p.num_ref_idx_l0_active */
	u8  num_ref_idx_l1_active;	/* rpm_param.p.num_ref_idx_l1_active */
	s32 col_poc;			/* resolved collocated POC, -1 if none */
	s16 col_buf_idx;		/* -1 if collocated ref not found */
	u16 collocated_ref_idx;
	u32 ctrl0;			/* value written to HEVC_MPRED_CTRL0 */
	u32 mv_wr_start;
	u32 mv_rd_start;
	u32 mv_rd_end;
	s32 ref_poc_list0[VDEC_DBG_MPRED_MAX_REFS];
	s32 ref_poc_list1[VDEC_DBG_MPRED_MAX_REFS];
	s32 st_curr0[VDEC_DBG_MPRED_MAX_REFS];	/* past refs (POC ordering) */
	s32 st_curr1[VDEC_DBG_MPRED_MAX_REFS];	/* future refs (POC ordering) */
	u16 cur_rps[VDEC_DBG_MPRED_MAX_REFS];	/* raw firmware RPS entries */
};

/* Snapshot request for on-demand register probe.  Read
 * /sys/kernel/debug/meson-vdec/probe_regs at any point to get a live
 * register dump of a curated set (MPRED status / stall, DBLK status,
 * SAO WPTRs, firmware MPC_E + SCRATCH_0..9, VIFIFO, MBY_MBX).
 * Re-runnable; unlike stall_dump which fires only at watchdog, this
 * gives a mid-decode snapshot whenever the reader wants it.
 */

/* Per-ISR snapshot entry (VDEC_1 and HEVC threaded ISR) */
struct vdec_dbg_isr_entry {
	u64 timestamp_ns;
	u32 decode_status;	/* AV_SCRATCH_H -- firmware decode status code */
	u32 bufferout;		/* AV_SCRATCH_9 -- buffer-out index / type */
	u32 scratch_5;		/* AV_SCRATCH_5 -- frame count or codec-specific */
	u32 mpsr;		/* MPSR -- microprocessor status */
	u32 codec_pixfmt;	/* sess->fmt_out->pixfmt for identification */
};

/* Session lifecycle event types */
enum vdec_dbg_event_type {
	VDEC_EVT_OPEN = 1,
	VDEC_EVT_CLOSE,
	VDEC_EVT_S_FMT_OUT,
	VDEC_EVT_STREAMON_OUT,
	VDEC_EVT_STREAMON_CAP,
	VDEC_EVT_STREAMOFF_OUT,
	VDEC_EVT_STREAMOFF_CAP,
	VDEC_EVT_DEC_CMD_STOP,
	VDEC_EVT_DEC_CMD_START,
	VDEC_EVT_SRC_CHANGE,
	VDEC_EVT_DRAIN_START,
	VDEC_EVT_DRAIN_DONE,
	VDEC_EVT_STALL,
	VDEC_EVT_ABORT,
	VDEC_EVT_RESUME,
	VDEC_EVT_IRQ,
	VDEC_EVT_DRAIN_CODEC_DRAINED,
	VDEC_EVT_DRAIN_WAIT_QUEUED,
	VDEC_EVT_DRAIN_WAIT_INACTIVE,
	VDEC_EVT_DRAIN_FLUSH_PENDING,
	VDEC_EVT_DRAIN_FIRMWARE_HALTED,
};

struct vdec_dbg_event {
	u64 timestamp_ns;
	u16 event_type;
	u16 detail;
	u32 value;
};

struct vdec_dbg_esparser_entry {
	u64 timestamp_ns;
	u32 size;
	u8 header[4];  /* first 4 bytes of bitstream */
};

struct vdec_debugfs {
	struct dentry *root;
	struct vdec_dbg_irq_counters irq;
	struct vdec_dbg_stall_dump stall;
	spinlock_t stall_lock;

	/* Session event log (lockless circular buffer) */
	struct vdec_dbg_event session_log[VDEC_DBG_SESSION_LOG_SIZE];
	unsigned int session_log_head;

	/* Esparser feed log (lockless circular buffer) */
	struct vdec_dbg_esparser_entry esparser_log[VDEC_DBG_ESPARSER_LOG_SIZE];
	unsigned int esparser_log_head;

	/* ISR ring buffer (lockless circular buffer, written from threaded ISR) */
	struct vdec_dbg_isr_entry isr_log[VDEC_DBG_ISR_LOG_SIZE];
	unsigned int isr_log_head;

	/* MPRED per-slice setup ring buffer (lockless circular buffer,
	 * written from codec_*_set_mpred() under hevc->lock).
	 */
	struct vdec_dbg_mpred_entry mpred_log[VDEC_DBG_MPRED_LOG_SIZE];
	unsigned int mpred_log_head;
};

#ifdef CONFIG_DEBUG_FS

void vdec_debugfs_init(struct amvdec_core *core);
void vdec_debugfs_remove(struct amvdec_core *core);
void vdec_dbg_capture_stall(struct amvdec_core *core, u32 mby_mbx);
void vdec_dbg_log_event(struct amvdec_core *core, u16 type, u16 detail,
			u32 value);
void vdec_dbg_log_esparser(struct amvdec_core *core, const void *data,
			   u32 size);
void vdec_dbg_log_isr(struct amvdec_session *sess);
void vdec_dbg_log_mpred(struct amvdec_core *core,
			const struct vdec_dbg_mpred_entry *e);

#else

static inline void vdec_debugfs_init(struct amvdec_core *core) {}
static inline void vdec_debugfs_remove(struct amvdec_core *core) {}
static inline void vdec_dbg_capture_stall(struct amvdec_core *core,
					   u32 mby_mbx) {}
static inline void vdec_dbg_log_event(struct amvdec_core *core, u16 type,
				       u16 detail, u32 value) {}
static inline void vdec_dbg_log_esparser(struct amvdec_core *core,
					  const void *data, u32 size) {}
static inline void vdec_dbg_log_isr(struct amvdec_session *sess) {}
static inline void vdec_dbg_log_mpred(struct amvdec_core *core,
				      const struct vdec_dbg_mpred_entry *e) {}

#endif /* CONFIG_DEBUG_FS */

#endif /* __VDEC_DEBUGFS_H__ */
