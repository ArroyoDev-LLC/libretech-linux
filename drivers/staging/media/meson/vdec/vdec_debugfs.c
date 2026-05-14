// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Libre Computer
 *
 * Debugfs diagnostic interface for meson-vdec.
 * Creates /sys/kernel/debug/meson-vdec/ with:
 *   firmware_status  -- live register snapshot (SCRATCH, VIFIFO, MPSR)
 *   irq_counters     -- per-status-code IRQ counts
 *   stall_dump       -- last captured stall state from watchdog
 */

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/ktime.h>

#include "vdec.h"
#include "vdec_debugfs.h"
#include "vdec_helpers.h"
#include "dos_regs.h"
#include "hevc_regs.h"
#include "vdec_1.h"
#include "vdec_hevc.h"

#ifdef CONFIG_DEBUG_FS

/* ---- firmware_status seq_file ---- */

static int firmware_status_show(struct seq_file *s, void *unused)
{
	struct amvdec_core *core = s->private;
	struct amvdec_session *sess;
	bool is_hevc;
	int i;

	sess = core->cur_sess;
	if (!sess) {
		seq_puts(s, "no active session\n");
		return 0;
	}

	if (sess->status < STATUS_INIT) {
		seq_printf(s, "session status: %d (not yet initialized)\n",
			   sess->status);
		return 0;
	}

	/* Detect engine type */
	is_hevc = (sess->fmt_out->vdec_ops == &vdec_hevc_ops);

	seq_printf(s, "engine: %s\n", is_hevc ? "HEVC" : "VDEC_1");
	seq_printf(s, "codec_pixfmt: 0x%08x\n", sess->fmt_out->pixfmt);
	seq_printf(s, "resolution: %ux%u\n", sess->width, sess->height);
	seq_printf(s, "session_status: %d\n", sess->status);
	seq_printf(s, "esparser_queued_bufs: %d\n",
		   atomic_read(&sess->esparser_queued_bufs));
	seq_printf(s, "platform_revision: %d\n",
		   core->platform->revision);
	seq_puts(s, "\n");

	if (is_hevc) {
		/* HEVC engine scratch registers */
		seq_puts(s, "HEVC_ASSIST_SCRATCH:\n");
		for (i = 0; i < 10; i++)
			seq_printf(s, "  SCRATCH_%d: 0x%08x\n", i,
				   amvdec_read_dos(core,
					HEVC_ASSIST_SCRATCH_0 + i * 4));

		seq_printf(s, "HEVC_STREAM_RD_PTR: 0x%08x\n",
			   amvdec_read_dos(core, HEVC_STREAM_RD_PTR));
		seq_printf(s, "HEVC_STREAM_WR_PTR: 0x%08x\n",
			   amvdec_read_dos(core, HEVC_STREAM_WR_PTR));
		seq_printf(s, "HEVC_STREAM_LEVEL: 0x%08x\n",
			   amvdec_read_dos(core, HEVC_STREAM_LEVEL));
		seq_printf(s, "HEVC_MPSR: 0x%08x\n",
			   amvdec_read_dos(core, HEVC_MPSR));
	} else {
		static const char * const scratch_labels[] = {
			"0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
			"A", "B", "C", "D", "E", "F", "G", "H", "I", "J",
			"K", "L"
		};
		/* VDEC_1 engine scratch registers (AV_SCRATCH_0 .. AV_SCRATCH_L) */
		seq_puts(s, "AV_SCRATCH:\n");
		for (i = 0; i < 22; i++)
			seq_printf(s, "  SCRATCH_%s: 0x%08x\n",
				   scratch_labels[i],
				   amvdec_read_dos(core,
					AV_SCRATCH_0 + i * 4));

		seq_printf(s, "MBY_MBX: 0x%08x\n",
			   amvdec_read_dos(core, MBY_MBX));
		seq_printf(s, "VLD_MEM_VIFIFO_RP: 0x%08x\n",
			   amvdec_read_dos(core, VLD_MEM_VIFIFO_RP));
		seq_printf(s, "VLD_MEM_VIFIFO_WP: 0x%08x\n",
			   amvdec_read_dos(core, VLD_MEM_VIFIFO_WP));
		seq_printf(s, "VLD_MEM_VIFIFO_LEVEL: 0x%08x\n",
			   amvdec_read_dos(core, VLD_MEM_VIFIFO_LEVEL));
		seq_printf(s, "VLD_STATUS_CTRL: 0x%08x\n",
			   amvdec_read_dos(core, VLD_STATUS_CTRL));
		seq_printf(s, "MPSR: 0x%08x\n",
			   amvdec_read_dos(core, MPSR));
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(firmware_status);

/* ---- irq_counters seq_file ---- */

static int irq_counters_show(struct seq_file *s, void *unused)
{
	struct amvdec_core *core = s->private;
	struct vdec_dbg_irq_counters *irq = &core->debugfs.irq;

	seq_printf(s, "total: %d\n", atomic_read(&irq->total));
	seq_printf(s, "h264_src_change: %d\n",
		   atomic_read(&irq->h264_src_change));
	seq_printf(s, "h264_frames_ready: %d\n",
		   atomic_read(&irq->h264_frames_ready));
	seq_printf(s, "hevc_head_parser_done: %d\n",
		   atomic_read(&irq->hevc_head_parser_done));
	seq_printf(s, "vp9_head_parser_done: %d\n",
		   atomic_read(&irq->vp9_head_parser_done));
	seq_printf(s, "vp9_eos: %d\n", atomic_read(&irq->vp9_eos));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(irq_counters);

/* ---- stall_dump seq_file ---- */

static int stall_dump_show(struct seq_file *s, void *unused)
{
	static const char * const scratch_labels[] = {
		"0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
		"A", "B", "C", "D", "E", "F", "G", "H", "I", "J",
		"K", "L"
	};
	struct amvdec_core *core = s->private;
	struct vdec_dbg_stall_dump dump;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&core->debugfs.stall_lock, flags);
	dump = core->debugfs.stall;
	spin_unlock_irqrestore(&core->debugfs.stall_lock, flags);

	if (!dump.valid) {
		seq_puts(s, "no stall captured\n");
		return 0;
	}

	seq_printf(s, "timestamp_ns: %llu\n", dump.timestamp_ns);
	seq_printf(s, "resolution: %ux%u\n", dump.width, dump.height);
	seq_printf(s, "MBY_MBX: 0x%08x\n", dump.mby_mbx);
	seq_printf(s, "MPSR: 0x%08x\n", dump.mpsr);
	seq_printf(s, "VIFIFO_RP: 0x%08x\n", dump.vififo_rp);
	seq_printf(s, "VIFIFO_WP: 0x%08x\n", dump.vififo_wp);
	seq_printf(s, "VIFIFO_LEVEL: 0x%08x\n", dump.vififo_level);

	for (i = 0; i < 22; i++)
		seq_printf(s, "SCRATCH_%s: 0x%08x\n",
			   scratch_labels[i], dump.scratch[i]);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(stall_dump);

/* ---- session_log seq_file ---- */

static const char * const vdec_event_names[] = {
	[0]                    = "UNKNOWN",
	[VDEC_EVT_OPEN]        = "OPEN",
	[VDEC_EVT_CLOSE]       = "CLOSE",
	[VDEC_EVT_S_FMT_OUT]   = "S_FMT_OUT",
	[VDEC_EVT_STREAMON_OUT] = "STREAMON_OUT",
	[VDEC_EVT_STREAMON_CAP] = "STREAMON_CAP",
	[VDEC_EVT_STREAMOFF_OUT] = "STREAMOFF_OUT",
	[VDEC_EVT_STREAMOFF_CAP] = "STREAMOFF_CAP",
	[VDEC_EVT_DEC_CMD_STOP] = "DEC_CMD_STOP",
	[VDEC_EVT_DEC_CMD_START] = "DEC_CMD_START",
	[VDEC_EVT_SRC_CHANGE]  = "SRC_CHANGE",
	[VDEC_EVT_DRAIN_START] = "DRAIN_START",
	[VDEC_EVT_DRAIN_DONE]  = "DRAIN_DONE",
	[VDEC_EVT_STALL]       = "STALL",
	[VDEC_EVT_ABORT]       = "ABORT",
	[VDEC_EVT_RESUME]      = "RESUME",
	[VDEC_EVT_IRQ]         = "IRQ",
	[VDEC_EVT_DRAIN_CODEC_DRAINED] = "DRAIN_CODEC_DRAINED",
	[VDEC_EVT_DRAIN_WAIT_QUEUED]   = "DRAIN_WAIT_QUEUED",
	[VDEC_EVT_DRAIN_WAIT_INACTIVE] = "DRAIN_WAIT_INACTIVE",
	[VDEC_EVT_DRAIN_FLUSH_PENDING] = "DRAIN_FLUSH_PENDING",
	[VDEC_EVT_DRAIN_FIRMWARE_HALTED] = "DRAIN_FIRMWARE_HALTED",
};

static int session_log_show(struct seq_file *s, void *unused)
{
	struct amvdec_core *core = s->private;
	unsigned int head = READ_ONCE(core->debugfs.session_log_head);
	unsigned int count = min(head, (unsigned int)VDEC_DBG_SESSION_LOG_SIZE);
	unsigned int start = head >= VDEC_DBG_SESSION_LOG_SIZE ?
			     head % VDEC_DBG_SESSION_LOG_SIZE : 0;
	unsigned int i;

	if (count == 0) {
		seq_puts(s, "no events\n");
		return 0;
	}

	for (i = 0; i < count; i++) {
		unsigned int idx = (start + i) % VDEC_DBG_SESSION_LOG_SIZE;
		const struct vdec_dbg_event *evt = &core->debugfs.session_log[idx];
		const char *name = "UNKNOWN";

		if (evt->event_type < ARRAY_SIZE(vdec_event_names) &&
		    vdec_event_names[evt->event_type])
			name = vdec_event_names[evt->event_type];

		seq_printf(s, "[%llu] %s detail=0x%04x value=0x%08x\n",
			   evt->timestamp_ns, name, evt->detail, evt->value);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(session_log);

/* ---- esparser_log seq_file ---- */

static int esparser_log_show(struct seq_file *s, void *unused)
{
	struct amvdec_core *core = s->private;
	unsigned int head = READ_ONCE(core->debugfs.esparser_log_head);
	unsigned int count = min(head, (unsigned int)VDEC_DBG_ESPARSER_LOG_SIZE);
	unsigned int start = head >= VDEC_DBG_ESPARSER_LOG_SIZE ?
			     head % VDEC_DBG_ESPARSER_LOG_SIZE : 0;
	unsigned int i;

	if (count == 0) {
		seq_puts(s, "no entries\n");
		return 0;
	}

	for (i = 0; i < count; i++) {
		unsigned int idx = (start + i) % VDEC_DBG_ESPARSER_LOG_SIZE;
		const struct vdec_dbg_esparser_entry *ent =
			&core->debugfs.esparser_log[idx];

		seq_printf(s, "[%llu] size=%u hdr=%02x %02x %02x %02x\n",
			   ent->timestamp_ns, ent->size,
			   ent->header[0], ent->header[1],
			   ent->header[2], ent->header[3]);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(esparser_log);

/* ---- isr_log seq_file ---- */

/*
 * Decode status codes reported in AV_SCRATCH_H by VDEC_1 firmware.
 * Values 1-6 are common across H.264 / MPEG-1/2/4 / VC-1.
 */
static const char *isr_decode_status_name(u32 status)
{
	switch (status) {
	case 1: return "PIC_DONE";
	case 2: return "DECODE_BUF_EMPTY";
	case 3: return "SEARCH_BUF_EMPTY";
	case 4: return "SKIP_PIC_DONE";
	case 5: return "INFO";
	case 6: return "ERROR_RESET";
	default: return NULL;
	}
}

/*
 * Expand a V4L2 fourcc into a 4-character printable string.
 * buf must be at least 5 bytes (4 chars + NUL).
 */
static void fourcc_to_str(u32 fourcc, char *buf)
{
	buf[0] = (fourcc >>  0) & 0xff;
	buf[1] = (fourcc >>  8) & 0xff;
	buf[2] = (fourcc >> 16) & 0xff;
	buf[3] = (fourcc >> 24) & 0xff;
	buf[4] = '\0';
	/* Replace non-printable bytes with '?' */
	if (buf[0] < 0x20 || buf[0] > 0x7e) buf[0] = '?';
	if (buf[1] < 0x20 || buf[1] > 0x7e) buf[1] = '?';
	if (buf[2] < 0x20 || buf[2] > 0x7e) buf[2] = '?';
	if (buf[3] < 0x20 || buf[3] > 0x7e) buf[3] = '?';
}

static int isr_log_show(struct seq_file *s, void *unused)
{
	struct amvdec_core *core = s->private;
	unsigned int head = READ_ONCE(core->debugfs.isr_log_head);
	unsigned int count = min(head, (unsigned int)VDEC_DBG_ISR_LOG_SIZE);
	unsigned int start = head >= VDEC_DBG_ISR_LOG_SIZE ?
			     head % VDEC_DBG_ISR_LOG_SIZE : 0;
	u64 base_ns = 0;
	unsigned int i;

	if (count == 0) {
		seq_puts(s, "no ISR entries\n");
		return 0;
	}

	/* Find the timestamp of the oldest entry for relative display */
	base_ns = core->debugfs.isr_log[start].timestamp_ns;

	seq_puts(s, "  idx  rel_ms  decode_status           bufferout  scratch_5  mpsr       codec\n");
	seq_puts(s, "  ---  ------  ----------------------  ---------  ---------  ---------  -----\n");

	for (i = 0; i < count; i++) {
		unsigned int idx = (start + i) % VDEC_DBG_ISR_LOG_SIZE;
		const struct vdec_dbg_isr_entry *ent =
			&core->debugfs.isr_log[idx];
		u64 rel_ms = (ent->timestamp_ns - base_ns) / 1000000;
		const char *status_name;
		char status_str[32];
		char fourcc[5];

		status_name = isr_decode_status_name(ent->decode_status);
		if (status_name)
			snprintf(status_str, sizeof(status_str),
				 "0x%02x (%s)", ent->decode_status, status_name);
		else
			snprintf(status_str, sizeof(status_str),
				 "0x%02x", ent->decode_status);

		fourcc_to_str(ent->codec_pixfmt, fourcc);

		seq_printf(s,
			   "  %3u  %6llu  %-22s  0x%07x  0x%07x  0x%07x  %s\n",
			   i, rel_ms, status_str,
			   ent->bufferout, ent->scratch_5, ent->mpsr,
			   fourcc);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(isr_log);

/* ---- HEVC SAO pipeline status (H.264 MMU Phase 1 verification) ---- */

static int hevc_sao_status_show(struct seq_file *s, void *unused)
{
	struct amvdec_core *core = s->private;
	u32 val;

	seq_puts(s, "# HEVC SAO Pipeline Registers (H.264 MMU Phase 1)\n");
	seq_puts(s, "# Read with h264_mmu_enable=1 during active H.264 session\n\n");

	seq_puts(s, "[IPP]\n");
	seq_printf(s, "HEVCD_IPP_TOP_CNTL:        0x%08x\n",
		   amvdec_read_dos(core, HEVCD_IPP_TOP_CNTL));
	seq_printf(s, "HEVCD_IPP_TOP_LCUCONFIG:   0x%08x  (expect 0x10)\n",
		   amvdec_read_dos(core, HEVCD_IPP_TOP_LCUCONFIG));
	seq_printf(s, "HEVCD_IPP_TOP_FRMCONFIG:   0x%08x  ([31:16]=h [15:0]=w)\n",
		   amvdec_read_dos(core, HEVCD_IPP_TOP_FRMCONFIG));
	seq_printf(s, "HEVCD_IPP_BITDEPTH_CONFIG: 0x%08x  (expect 0x0)\n",
		   amvdec_read_dos(core, HEVCD_IPP_BITDEPTH_CONFIG));
	seq_printf(s, "HEVCD_IPP_AXIIF_CONFIG:    0x%08x\n",
		   amvdec_read_dos(core, HEVCD_IPP_AXIIF_CONFIG));

	seq_puts(s, "\n[DBLK]\n");
	seq_printf(s, "HEVC_DBLK_CFG1:            0x%08x  (expect 0x2)\n",
		   amvdec_read_dos(core, HEVC_DBLK_CFG1));
	seq_printf(s, "HEVC_DBLK_CFG2:            0x%08x  ([31:16]=h [15:0]=w)\n",
		   amvdec_read_dos(core, HEVC_DBLK_CFG2));
	seq_printf(s, "HEVC_DBLK_CFGB:            0x%08x  (expect 0x40405703)\n",
		   amvdec_read_dos(core, HEVC_DBLK_CFGB));

	seq_puts(s, "\n[SAO]\n");
	seq_printf(s, "HEVC_SAO_CTRL0:            0x%08x  ([3:0] expect 0x4)\n",
		   amvdec_read_dos(core, HEVC_SAO_CTRL0));
	seq_printf(s, "HEVC_SAO_CTRL1:            0x%08x\n",
		   amvdec_read_dos(core, HEVC_SAO_CTRL1));
	seq_printf(s, "HEVC_SAO_CTRL5:            0x%08x  (bit9=smem bit10=compress)\n",
		   amvdec_read_dos(core, HEVC_SAO_CTRL5));
	seq_printf(s, "HEVC_SAO_CTRL9:            0x%08x  (bit1=compress bit7=ctrl)\n",
		   amvdec_read_dos(core, HEVC_SAO_CTRL9));
	seq_printf(s, "HEVC_SAO_PIC_SIZE:         0x%08x\n",
		   amvdec_read_dos(core, HEVC_SAO_PIC_SIZE));
	seq_printf(s, "HEVC_SAO_PIC_SIZE_LCU:     0x%08x\n",
		   amvdec_read_dos(core, HEVC_SAO_PIC_SIZE_LCU));
	seq_printf(s, "HEVC_SAO_TILE_SIZE_LCU:    0x%08x\n",
		   amvdec_read_dos(core, HEVC_SAO_TILE_SIZE_LCU));
	seq_printf(s, "HEVC_SAO_Y_LENGTH:         0x%08x\n",
		   amvdec_read_dos(core, HEVC_SAO_Y_LENGTH));
	seq_printf(s, "HEVC_SAO_C_LENGTH:         0x%08x\n",
		   amvdec_read_dos(core, HEVC_SAO_C_LENGTH));

	seq_puts(s, "\n[CM]\n");
	seq_printf(s, "HEVC_CM_BODY_LENGTH:       0x%08x\n",
		   amvdec_read_dos(core, HEVC_CM_BODY_LENGTH));
	seq_printf(s, "HEVC_CM_HEADER_OFFSET:     0x%08x\n",
		   amvdec_read_dos(core, HEVC_CM_HEADER_OFFSET));
	seq_printf(s, "HEVC_CM_HEADER_LENGTH:     0x%08x\n",
		   amvdec_read_dos(core, HEVC_CM_HEADER_LENGTH));

	seq_puts(s, "\n[MDEC]\n");
	val = amvdec_read_dos(core, MDEC_PIC_DC_MUX_CTRL);
	seq_printf(s, "MDEC_PIC_DC_MUX_CTRL:      0x%08x", val);
	if (val & BIT(31))
		seq_puts(s, "  *** bit 31 SET (MDEC->HEVC MCR) ***\n");
	else
		seq_puts(s, "  [OK: bit 31 clear]\n");

	seq_puts(s, "\n[MCR]\n");
	seq_printf(s, "HEVCD_MPP_VDEC_MCR_CTL:    0x%08x  (expect 0x11)\n",
		   amvdec_read_dos(core, HEVCD_MPP_VDEC_MCR_CTL));
	seq_printf(s, "HEVCD_MPP_DECOMP_CTL1:     0x%08x  (expect 0x10)\n",
		   amvdec_read_dos(core, HEVCD_MPP_DECOMP_CTL1));
	seq_printf(s, "HEVCD_MPP_DECOMP_CTL2:     0x%08x\n",
		   amvdec_read_dos(core, HEVCD_MPP_DECOMP_CTL2));
	seq_printf(s, "HEVCD_MPP_DECOMP_CTL3:     0x%08x\n",
		   amvdec_read_dos(core, HEVCD_MPP_DECOMP_CTL3));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(hevc_sao_status);

/* ---- mpred_trace seq_file ---- */

/*
 * Per-slice MPRED snapshot writer.  Called from codec_hevc_set_mpred
 * and codec_vp9_set_mpred under the codec'"'"'s lock.  Appends a single
 * entry to the ring buffer -- non-blocking, one memcpy.
 */
void vdec_dbg_log_mpred(struct amvdec_core *core,
			const struct vdec_dbg_mpred_entry *e)
{
	struct vdec_debugfs *d = &core->debugfs;
	unsigned int head = READ_ONCE(d->mpred_log_head);
	unsigned int idx = head % VDEC_DBG_MPRED_LOG_SIZE;

	d->mpred_log[idx] = *e;
	WRITE_ONCE(d->mpred_log_head, head + 1);
}
EXPORT_SYMBOL_GPL(vdec_dbg_log_mpred);

static const char *mpred_slice_name(u8 slice_type)
{
	switch (slice_type) {
	case 0: return "B";
	case 1: return "P";
	case 2: return "I";
	default: return "?";
	}
}

static void mpred_print_ref_list(struct seq_file *s, const char *label,
				  const s32 *list, u8 count)
{
	u8 i;

	seq_printf(s, "    %s[%u]:", label, count);
	for (i = 0; i < count && i < VDEC_DBG_MPRED_MAX_REFS; i++)
		seq_printf(s, " %d", list[i]);
	seq_puts(s, "\n");
}

static int mpred_trace_show(struct seq_file *s, void *unused)
{
	struct amvdec_core *core = s->private;
	unsigned int head = READ_ONCE(core->debugfs.mpred_log_head);
	unsigned int count = min(head, (unsigned int)VDEC_DBG_MPRED_LOG_SIZE);
	unsigned int start = head >= VDEC_DBG_MPRED_LOG_SIZE ?
			     head % VDEC_DBG_MPRED_LOG_SIZE : 0;
	unsigned int i;

	if (count == 0) {
		seq_puts(s, "no entries\n");
		return 0;
	}

	seq_puts(s, "# MPRED per-slice setup snapshot.\n");
	seq_puts(s, "# Captured at codec_*_set_mpred().  Each slice is one paragraph.\n");
	seq_puts(s, "# Fields:\n");
	seq_puts(s, "#   [ts_ns] codec slice_type=<I/P/B> poc=<curr_poc> new_pic=<0/1>\n");
	seq_puts(s, "#           col_from_l0=<0/1> col_ref_idx=<N> ldc=<0/1> tmvp=<0/1>\n");
	seq_puts(s, "#     L0[n]: <poc...>\n");
	seq_puts(s, "#     L1[n]: <poc...>\n");
	seq_puts(s, "#     col_poc=<poc|-1> col_buf_idx=<idx|-1>\n");
	seq_puts(s, "#     ctrl0=<0x...> mv_wr=<0x...> mv_rd=<0x...> mv_rd_end=<0x...>\n");
	seq_puts(s, "\n");

	for (i = 0; i < count; i++) {
		unsigned int idx = (start + i) % VDEC_DBG_MPRED_LOG_SIZE;
		const struct vdec_dbg_mpred_entry *e = &core->debugfs.mpred_log[idx];

		seq_printf(s,
			   "[%llu] %s slice_type=%s poc=%d new_pic=%u col_from_l0=%u col_ref_idx=%u ldc=%u tmvp=%u\n",
			   e->timestamp_ns,
			   e->codec ? "vp9" : "hevc",
			   mpred_slice_name(e->slice_type),
			   e->curr_poc, e->new_pic, e->col_from_l0,
			   e->collocated_ref_idx, e->ldc_flag, e->tmvp_flag);
		seq_printf(s,
			   "    num_ref_idx_lX_active=%u/%u mod_flag=0x%02x\n",
			   e->num_ref_idx_l0_active, e->num_ref_idx_l1_active,
			   e->mod_flag);
		mpred_print_ref_list(s, "StCurr0", e->st_curr0, e->num_neg);
		mpred_print_ref_list(s, "StCurr1", e->st_curr1, e->num_pos);
		mpred_print_ref_list(s, "L0", e->ref_poc_list0, e->ref_num_l0);
		mpred_print_ref_list(s, "L1", e->ref_poc_list1, e->ref_num_l1);
		seq_printf(s, "    col_poc=%d col_buf_idx=%d\n",
			   e->col_poc, e->col_buf_idx);
		seq_printf(s,
			   "    ctrl0=0x%08x mv_wr=0x%08x mv_rd=0x%08x mv_rd_end=0x%08x\n",
			   e->ctrl0, e->mv_wr_start, e->mv_rd_start,
			   e->mv_rd_end);
		/* Raw CUR_RPS[] (16 u16 entries) -- useful for diffing firmware
		 * RPS parsing vs expected bitstream RPS.  0x8000 = end sentinel.
		 */
		seq_puts(s, "    cur_rps:");
		{
			unsigned int k;

			for (k = 0; k < VDEC_DBG_MPRED_MAX_REFS; k++) {
				if (e->cur_rps[k] == 0x8000) {
					seq_puts(s, " END");
					break;
				}
				seq_printf(s, " 0x%04x", e->cur_rps[k]);
			}
		}
		seq_puts(s, "\n");
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mpred_trace);

/* ---- probe_regs seq_file (on-demand register snapshot) ---- */

/*
 * Live register dump of the curated set used for HEVC B-frame investigation
 * (TODO-HEVC-GXL-BFRAME-CONTENT) and the G12B/SM1 POC=6 wedge (TASK-004).
 * Re-runnable; unlike stall_dump which fires only at watchdog, this reads
 * at cat time so the user can take snapshots mid-decode.
 *
 * All reads are MMIO; no state mutation.  Safe to run while decode is
 * active.  Registers that do not exist on the current engine return 0;
 * this is acceptable because the caller can tell from firmware_status
 * which engine is active.
 */
static int probe_regs_show(struct seq_file *s, void *unused)
{
	struct amvdec_core *core = s->private;
	struct amvdec_session *sess = core->cur_sess;
	bool is_hevc;
	int i;

	if (!sess) {
		seq_puts(s, "no active session\n");
		return 0;
	}

	is_hevc = (sess->fmt_out->vdec_ops == &vdec_hevc_ops);
	seq_printf(s, "ts_ns = %llu\n", ktime_get_ns());
	seq_printf(s, "engine = %s\n", is_hevc ? "HEVC" : "VDEC_1");
	seq_printf(s, "codec_pixfmt = 0x%08x\n", sess->fmt_out->pixfmt);

	/* Firmware identification (TASK-004 audit trail).  fw_match is set
	 * by amvdec_fw_identify() at firmware load.  Future bucket-gated
	 * code paths read fw_match->quirks to dispatch.
	 */
	if (sess->fw_match)
		seq_printf(s, "firmware_id = \"%s\" size=%zu quirks=0x%x\n",
			   sess->fw_match->label,
			   sess->fw_match->size,
			   sess->fw_match->quirks);
	else
		seq_puts(s, "firmware_id = (not identified)\n");

	if (is_hevc) {
		/* MPRED block -- central to B-frame / TASK-004 debugging */
		seq_printf(s, "HEVC_MPRED_CTRL0       = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_MPRED_CTRL0));
		seq_printf(s, "HEVC_MPRED_CTRL1       = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_MPRED_CTRL1));
		seq_printf(s, "HEVC_MPRED_INT_STATUS  = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_MPRED_INT_STATUS));
		seq_printf(s, "HEVC_MPRED_PIC_SIZE    = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_MPRED_PIC_SIZE));
		seq_printf(s, "HEVC_MPRED_CUR_POC     = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_MPRED_CUR_POC));

		/* DBLK block -- stall witness for TASK-004 */
		seq_printf(s, "HEVC_DBLK_STS0         = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_DBLK_STS0));
		seq_printf(s, "HEVC_DBLK_STS1         = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_DBLK_STS1));
		/* STS1+4 (0xd43c) is the G12A+ dblk_main pipeline select
		 * status; read as raw MMIO via amvdec_read_dos.
		 */
		seq_printf(s, "HEVC_DBLK_STS1_G12A    = 0x%08x\n",
			   amvdec_read_dos(core, 0xd43c));

		/* SAO block -- chroma double-write progress */
		seq_printf(s, "HEVC_SAO_Y_WPTR        = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_SAO_Y_WPTR));
		seq_printf(s, "HEVC_SAO_C_WPTR        = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_SAO_C_WPTR));
		seq_printf(s, "HEVC_SAO_Y_START_ADDR  = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_SAO_Y_START_ADDR));
		seq_printf(s, "HEVC_SAO_C_START_ADDR  = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_SAO_C_START_ADDR));

		/* Firmware + bitstream plumbing */
		seq_printf(s, "HEVC_MPSR              = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_MPSR));
		seq_printf(s, "HEVC_CPSR              = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_CPSR));

		/* Firmware program counter and interrupt routing (TASK-004).
		 * MPC_E is the AMRISC instruction pointer (not MPSR which
		 * holds status flags).  Historical firmware idle loop sits
		 * at PC 0x782-0x785; anything else is an active execution
		 * point or a stall.  MCPU_INTR_MSK/REQ and ASSIST_MBOX1_MASK
		 * reveal whether completion IRQs can reach the firmware.
		 */
		seq_printf(s, "HEVC_MPC_E             = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_MPC_E));
		seq_printf(s, "HEVC_MCPU_INTR_MSK     = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_MCPU_INTR_MSK));
		seq_printf(s, "HEVC_MCPU_INTR_REQ     = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_MCPU_INTR_REQ));
		seq_printf(s, "HEVC_ASSIST_MBOX1_MASK = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_ASSIST_MBOX1_MASK));

		/* DOS_SCRATCH_31 (CR 0x3f2f = byte 0xFCBC): per firmware
		 * disasm of sm1_hevc_mmu.bin, BIT(0) is the inner-poll
		 * exit condition at PC 0x7ab-0x7ac.  When firmware idles
		 * at the outer 0x782 loop, DOS_SCRATCH_31 tells us
		 * whether the inner poll would exit immediately (bit0=0)
		 * or spin (bit0=1).  Added for TASK-004 dispatch-path
		 * diagnosis -- see hevc/task004-gxl-vs-sm1-diff-20260420.md
		 * strike-6 candidate #1.
		 */
		seq_printf(s, "DOS_SCRATCH_31         = 0x%08x\n",
			   amvdec_read_dos(core, DOS_SCRATCH_31));

		/* Extra SAO programming registers (TASK-004): verify SAO
		 * output is actually configured.  CTRL1/CTRL5 gate Y and
		 * C double-write; Y_LENGTH/C_LENGTH are the byte counts
		 * HW is supposed to write for the current picture.
		 */
		seq_printf(s, "HEVC_SAO_CTRL0         = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_SAO_CTRL0));
		seq_printf(s, "HEVC_SAO_CTRL1         = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_SAO_CTRL1));
		seq_printf(s, "HEVC_SAO_CTRL5         = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_SAO_CTRL5));
		seq_printf(s, "HEVC_SAO_Y_LENGTH      = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_SAO_Y_LENGTH));
		seq_printf(s, "HEVC_SAO_C_LENGTH      = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_SAO_C_LENGTH));

		/* DBLK output buffer programming: CFG4/CFG5 hold Y/C DBLK
		 * data addresses.  If DBLK is unwired for the current
		 * slice, these may be stale from the previous picture.
		 */
		seq_printf(s, "HEVC_DBLK_CFG4         = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_DBLK_CFG4));
		seq_printf(s, "HEVC_DBLK_CFG5         = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_DBLK_CFG5));

		seq_printf(s, "HEVC_STREAM_START_ADDR = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_STREAM_START_ADDR));
		seq_printf(s, "HEVC_STREAM_END_ADDR   = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_STREAM_END_ADDR));
		seq_printf(s, "HEVC_STREAM_RD_PTR     = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_STREAM_RD_PTR));
		seq_printf(s, "HEVC_STREAM_WR_PTR     = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_STREAM_WR_PTR));
		seq_printf(s, "HEVC_STREAM_LEVEL      = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_STREAM_LEVEL));
		/* Stream-control + parser-int registers -- diagnose parser
		 * configuration state.  TASK-004 SM1 BSP-firmware stuck-at-
		 * RD_PTR=0 vs upstream-firmware advancing test needs to know
		 * whether HEVC_STREAM_CTRL_ENABLE / HEVC_STREAM_CTRL_USE_PARSER /
		 * HEVC_STREAM_FIFO_CTL_FILL_EN bits are configured the same way
		 * for both firmwares.
		 */
		seq_printf(s, "HEVC_STREAM_CONTROL    = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_STREAM_CONTROL));
		seq_printf(s, "HEVC_STREAM_FIFO_CTL   = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_STREAM_FIFO_CTL));
		seq_printf(s, "HEVC_PARSER_INT_CONTROL= 0x%08x\n",
			   amvdec_read_dos(core, HEVC_PARSER_INT_CONTROL));
		/* SHIFT engine + parser status -- TASK-004 upstream wedge:
		 * firmware spins at PC 0x782 polling STREAM_LEVEL.  Need to
		 * see whether SHIFT engine's byte counter / status flags
		 * indicate the parser/CABAC pipeline made progress that
		 * firmware isn't observing, OR whether HEVC_PARSER_INT_STATUS
		 * has unhandled bits the firmware-driver protocol expects ARM
		 * to clear.
		 */
		seq_printf(s, "HEVC_SHIFT_BYTE_COUNT  = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_SHIFT_BYTE_COUNT));
		seq_printf(s, "HEVC_SHIFT_STATUS      = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_SHIFT_STATUS));
		seq_printf(s, "HEVC_SHIFT_CONTROL     = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_SHIFT_CONTROL));
		seq_printf(s, "HEVC_PARSER_INT_STATUS = 0x%08x\n",
			   amvdec_read_dos(core, HEVC_PARSER_INT_STATUS));
		/* HEVCD per-slice engine state: IPP / MPP / MCRCC.  Wedge
		 * happens before slice processing; engines should be idle
		 * (configured by codec_hevc_init_decoder but not active).
		 * Captures here let us diff against working GXL state.
		 */
		seq_printf(s, "HEVCD_IPP_TOP_CNTL     = 0x%08x\n",
			   amvdec_read_dos(core, HEVCD_IPP_TOP_CNTL));
		seq_printf(s, "HEVCD_IPP_AXIIF_CONFIG = 0x%08x\n",
			   amvdec_read_dos(core, HEVCD_IPP_AXIIF_CONFIG));
		seq_printf(s, "HEVCD_MPP_DECOMP_CTL1  = 0x%08x\n",
			   amvdec_read_dos(core, HEVCD_MPP_DECOMP_CTL1));
		seq_printf(s, "HEVCD_MCRCC_CTL1       = 0x%08x\n",
			   amvdec_read_dos(core, HEVCD_MCRCC_CTL1));

		/* Firmware scratch 0..9 -- status reporting channel */
		{
			static const char * const hevc_scratch_labels[] = {
				"0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
				"A", "B", "C", "D", "E", "F", "G", "H", "I", "J",
				"K", "L", "M", "N"
			};
			int n;

			for (n = 0; n < ARRAY_SIZE(hevc_scratch_labels); n++)
				seq_printf(s,
					   "HEVC_ASSIST_SCRATCH_%s  = 0x%08x\n",
					   hevc_scratch_labels[n],
					   amvdec_read_dos(core,
						HEVC_ASSIST_SCRATCH_0 + n * 4));
		}
	} else {
		/* VDEC_1 engine */
		static const char * const scratch_labels[] = {
			"0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
			"A", "B", "C", "D", "E", "F", "G", "H", "I", "J",
			"K", "L"
		};
		seq_printf(s, "MPSR                   = 0x%08x\n",
			   amvdec_read_dos(core, MPSR));
		seq_printf(s, "MBY_MBX                = 0x%08x\n",
			   amvdec_read_dos(core, MBY_MBX));
		seq_printf(s, "VLD_MEM_VIFIFO_RP      = 0x%08x\n",
			   amvdec_read_dos(core, VLD_MEM_VIFIFO_RP));
		seq_printf(s, "VLD_MEM_VIFIFO_WP      = 0x%08x\n",
			   amvdec_read_dos(core, VLD_MEM_VIFIFO_WP));
		seq_printf(s, "VLD_MEM_VIFIFO_LEVEL   = 0x%08x\n",
			   amvdec_read_dos(core, VLD_MEM_VIFIFO_LEVEL));
		for (i = 0; i < 22; i++)
			seq_printf(s, "AV_SCRATCH_%s          = 0x%08x\n",
				   scratch_labels[i],
				   amvdec_read_dos(core,
					AV_SCRATCH_0 + i * 4));
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(probe_regs);

/* ---- init / remove ---- */

void vdec_debugfs_init(struct amvdec_core *core)
{
	struct dentry *root;

	spin_lock_init(&core->debugfs.stall_lock);

	root = debugfs_create_dir("meson-vdec", NULL);
	if (IS_ERR_OR_NULL(root)) {
		dev_warn(core->dev, "debugfs: failed to create meson-vdec dir\n");
		core->debugfs.root = NULL;
		return;
	}

	core->debugfs.root = root;

	debugfs_create_file("firmware_status", 0444, root, core,
			    &firmware_status_fops);
	debugfs_create_file("irq_counters", 0444, root, core,
			    &irq_counters_fops);
	debugfs_create_file("stall_dump", 0444, root, core,
			    &stall_dump_fops);
	debugfs_create_file("session_log", 0444, root, core,
			    &session_log_fops);
	debugfs_create_file("esparser_log", 0444, root, core,
			    &esparser_log_fops);
	debugfs_create_file("isr_log", 0444, root, core,
			    &isr_log_fops);
	debugfs_create_file("hevc_sao_status", 0444, root, core,
			    &hevc_sao_status_fops);
	debugfs_create_file("mpred_trace", 0444, root, core,
			    &mpred_trace_fops);
	debugfs_create_file("probe_regs", 0444, root, core,
			    &probe_regs_fops);

	dev_info(core->dev, "debugfs: /sys/kernel/debug/meson-vdec/ created\n");
}

void vdec_debugfs_remove(struct amvdec_core *core)
{
	debugfs_remove_recursive(core->debugfs.root);
	core->debugfs.root = NULL;
}

/* ---- stall capture (called from codec watchdog timer) ---- */

void vdec_dbg_capture_stall(struct amvdec_core *core, u32 mby_mbx)
{
	struct vdec_dbg_stall_dump *stall = &core->debugfs.stall;
	struct amvdec_session *sess = core->cur_sess;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&core->debugfs.stall_lock, flags);

	stall->timestamp_ns = ktime_get_ns();
	stall->mby_mbx = mby_mbx;
	stall->mpsr = amvdec_read_dos(core, MPSR);
	stall->vififo_rp = amvdec_read_dos(core, VLD_MEM_VIFIFO_RP);
	stall->vififo_wp = amvdec_read_dos(core, VLD_MEM_VIFIFO_WP);
	stall->vififo_level = amvdec_read_dos(core, VLD_MEM_VIFIFO_LEVEL);

	for (i = 0; i < 22; i++)
		stall->scratch[i] = amvdec_read_dos(core,
						     AV_SCRATCH_0 + i * 4);

	if (sess) {
		stall->width = sess->width;
		stall->height = sess->height;
	} else {
		stall->width = 0;
		stall->height = 0;
	}

	stall->valid = true;

	spin_unlock_irqrestore(&core->debugfs.stall_lock, flags);
}
EXPORT_SYMBOL_GPL(vdec_dbg_capture_stall);

/* ---- session event logging ---- */

void vdec_dbg_log_event(struct amvdec_core *core, u16 type, u16 detail,
			u32 value)
{
	unsigned int idx = core->debugfs.session_log_head %
			   VDEC_DBG_SESSION_LOG_SIZE;
	struct vdec_dbg_event *evt = &core->debugfs.session_log[idx];

	evt->timestamp_ns = ktime_get_ns();
	evt->event_type = type;
	evt->detail = detail;
	evt->value = value;

	/* Increment head (not atomic -- acceptable for diagnostic ring) */
	WRITE_ONCE(core->debugfs.session_log_head,
		   core->debugfs.session_log_head + 1);
}
EXPORT_SYMBOL_GPL(vdec_dbg_log_event);

/* ---- esparser feed logging ---- */

void vdec_dbg_log_esparser(struct amvdec_core *core, const void *data,
			   u32 size)
{
	unsigned int idx = core->debugfs.esparser_log_head %
			   VDEC_DBG_ESPARSER_LOG_SIZE;
	struct vdec_dbg_esparser_entry *ent = &core->debugfs.esparser_log[idx];
	u32 copy_len = min_t(u32, size, 4);

	ent->timestamp_ns = ktime_get_ns();
	ent->size = size;
	memset(ent->header, 0, sizeof(ent->header));
	if (data && copy_len > 0)
		memcpy(ent->header, data, copy_len);

	WRITE_ONCE(core->debugfs.esparser_log_head,
		   core->debugfs.esparser_log_head + 1);
}
EXPORT_SYMBOL_GPL(vdec_dbg_log_esparser);

/* ---- ISR ring buffer capture (called from vdec_threaded_isr) ---- */

/*
 * vdec_dbg_log_isr - capture VDEC_1 register state at each threaded ISR.
 *
 * No printk output -- data stays in memory and is read via debugfs
 * isr_log at /sys/kernel/debug/meson-vdec/isr_log.
 * Cost: 4 MMIO reads + ring-buffer write (~500 ns), no lock.
 *
 * SKIPPED for HEVC-engine codecs (HEVC, VP9): the AV_SCRATCH_* and
 * MPSR offsets target the VDEC_1 register block, which is in a
 * separate power domain on SM1. While HEVC/VP9 is the active codec,
 * VDEC_1 is powered down; reading those offsets triggers an AXI bus
 * fault that hangs the SoC.  Originally the call was added with a
 * "instrumentation-only, zero functional change" assumption; the
 * VP9-024 bisect (2026-04-17/18) proved that assumption wrong --
 * MMIO reads have side effects when the target block is unpowered.
 */
void vdec_dbg_log_isr(struct amvdec_session *sess)
{
	struct amvdec_core *core;
	struct vdec_dbg_isr_entry *ent;
	unsigned int idx;
	u32 fmt;

	if (!sess || !sess->core)
		return;

	fmt = sess->fmt_out ? sess->fmt_out->pixfmt : 0;
	if (fmt == V4L2_PIX_FMT_HEVC || fmt == V4L2_PIX_FMT_VP9)
		return;

	core = sess->core;
	idx = core->debugfs.isr_log_head % VDEC_DBG_ISR_LOG_SIZE;
	ent = &core->debugfs.isr_log[idx];

	ent->timestamp_ns  = ktime_get_ns();
	ent->decode_status = amvdec_read_dos(core, AV_SCRATCH_H);
	ent->bufferout     = amvdec_read_dos(core, AV_SCRATCH_9);
	ent->scratch_5     = amvdec_read_dos(core, AV_SCRATCH_5);
	ent->mpsr          = amvdec_read_dos(core, MPSR);
	ent->codec_pixfmt  = fmt;

	WRITE_ONCE(core->debugfs.isr_log_head,
		   core->debugfs.isr_log_head + 1);
}
EXPORT_SYMBOL_GPL(vdec_dbg_log_isr);

#endif /* CONFIG_DEBUG_FS */
