// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2014-2018 Etnaviv Project
 *
 * Per-chip cmdbuf submission entry point for S905D3 NPU
 * (VIPPico v3, customer_id 0x99).
 *
 * Phase A initial split (amlogic/software/npu/CLAUDE.md "Per-chip
 * code-path split"). Bakes in the cumulative R47+R51+R52 work:
 * - R47: no PTA_CONFIG cmdbuf LOAD_STATE
 * - R51: no post-flush SEM+STALL, no pipe_select
 * - R52: no cmdbuf-emitted MMU flush LOAD_STATE; flush done via
 *        direct MMIO write (gpu_write) before the LINK. The BSP
 *        gckMMU model never emits VIVS_MMUv2_CONFIGURATION via
 *        cmdbuf; it uses direct MMIO at probe time and before each
 *        LINK kick. VIPPico v3 silicon hangs FE at CMD_LOAD_STATE
 *        VIVS_MMUv2_CONFIGURATION when it appears in the ring buffer
 *        (confirmed by r91 ring dump: last-fetch 0x08010061 0x04112018
 *        = CMD_LOAD_STATE reg=0x184 in the stalled state).
 *
 * Net 0x99 NPU prelude after R52:
 *   MMIO write VIVS_MMUv2_CONFIGURATION (flush) -> LINK to cmdbuf
 *   (no PTA_CONFIG, no MMU-flush LOAD_STATE in ring, no SEM+STALL,
 *    no pipe_select)
 *
 * The 0x88 customer's full prelude (PTA_CONFIG + MMU flush LOAD_STATE +
 * WAIT + pipe_select) lives in the sibling file etnaviv_buffer_v8_qi.c.
 *
 * R52 was excluded from Phase A because in the shared etnaviv_buffer.c
 * path it interacted with the strike-3 npu_poll_completion-gated re-arm
 * in etnaviv_sched.c:etnaviv_sched_timedout_job, regressing /019.
 * Now that 0x99 logic is isolated to this file, the interaction is gone.
 * Reference: amlogic/software/npu/audit/2026-05-09-tp-only-single-op-stall-localization.md
 */

#include <drm/drm_drv.h>

#include "etnaviv_buffer_internal.h"
#include "etnaviv_cmdbuf.h"
#include "etnaviv_gpu.h"
#include "etnaviv_gem.h"
#include "etnaviv_mmu.h"

#include "common.xml.h"
#include "state.xml.h"
#include "state_blt.xml.h"
#include "state_hi.xml.h"
#include "state_3d.xml.h"
#include "cmdstream.xml.h"

void etnaviv_buffer_queue_v8_pico(struct etnaviv_gpu *gpu, u32 exec_state,
	struct etnaviv_iommu_context *mmu_context, unsigned int event,
	struct etnaviv_cmdbuf *cmdbuf)
{
	struct etnaviv_cmdbuf *buffer = &gpu->buffer;
	unsigned int waitlink_offset = buffer->user_size - 16;
	u32 return_target, return_dwords;
	u32 link_target, link_dwords;
	bool switch_context = gpu->exec_state != exec_state;
	bool switch_mmu_context = gpu->mmu_context != mmu_context;
	unsigned int new_flush_seq = READ_ONCE(mmu_context->flush_seq);
	bool need_flush = switch_mmu_context || gpu->flush_seq != new_flush_seq;

	lockdep_assert_held(&gpu->lock);

	if (drm_debug_enabled(DRM_UT_DRIVER))
		etnaviv_buffer_dump(gpu, buffer, 0, 0x50);

	link_target = etnaviv_cmdbuf_get_va(cmdbuf,
					    &gpu->mmu_context->cmdbuf_mapping);
	link_dwords = cmdbuf->size / 8;

	{
		u32 *cmdbuf_words = cmdbuf->vaddr;
		unsigned int nwords = cmdbuf->size / 4;
		unsigned int w;

		dev_info(gpu->dev, "NPU submit: cmdbuf_va=0x%08x size=%u switch_ctx=%d need_flush=%d flush_seq=%u/%u switch_mmu=%d\n",
			 link_target, cmdbuf->size, switch_context, need_flush,
			 gpu->flush_seq, new_flush_seq, switch_mmu_context);

		/* Scan for NN_INST_ADDR (0x0428) or TP_INST_ADDR (0x042E) */
		for (w = 0; w + 1 < nwords; w += 2) {
			u32 hdr = cmdbuf_words[w];
			u16 off = hdr & VIV_FE_LOAD_STATE_HEADER_OFFSET__MASK;
			if (off == (VIVS_PS_NN_INST_ADDR >> 2) ||
			    off == (VIVS_PS_TP_INST_ADDR >> 2)) {
				dev_info(gpu->dev,
					 "NPU cmdbuf: %s at word %u, addr=0x%08x\n",
					 off == (VIVS_PS_NN_INST_ADDR >> 2) ?
					 "NN_INST" : "TP_INST",
					 w, cmdbuf_words[w+1]);
			}
		}
	}

	/*
	 * Prelude is the minimal kernel-ring needed for c64a 0x99
	 * silicon. Cumulative R47+R51 work removed everything that
	 * hangs FE at LOAD_STATE post-MMU-flush:
	 * - PTA_CONFIG (R47)        -- skipped here
	 * - SEM+STALL FE->PE (R51)  -- skipped here
	 * - etnaviv_cmd_select_pipe (R51) -- skipped here
	 * Per-op cmdbuf state writes (NN_CONFIG, NN_INST_ADDR) emitted
	 * by mesa take a different silicon path that doesn't choke.
	 */
	if (need_flush || switch_context) {
		u32 target, extra_dwords;

		/* link command */
		extra_dwords = 1;

		/*
		 * R52: MMU flush is done via direct MMIO write (gpu_write)
		 * before the LINK, not via cmdbuf LOAD_STATE. VIPPico v3
		 * silicon hangs FE at CMD_LOAD_STATE VIVS_MMUv2_CONFIGURATION
		 * in the ring buffer (audit 2026-05-09-tp-only-single-op-
		 * stall-localization.md, finding 3: last-fetch 0x08010061
		 * 0x04112018 = CMD_LOAD_STATE reg=0x184 = VIVS_MMUv2_CONFIGURATION).
		 * V1 FLUSH_MMU is also done via MMIO for consistency.
		 * No cmdbuf space needed for the flush -- only the LINK.
		 */

		/* No PTA_CONFIG (R47), no MMU-flush LOAD_STATE (R52),
		 * no SEM+STALL (R51), no pipe_select (R51) on 0x99.
		 */

		target = etnaviv_buffer_reserve(gpu, buffer, extra_dwords);

		/*
		 * Switch MMU context if necessary. Must be done after the
		 * link target has been calculated.
		 */
		if (switch_mmu_context) {
			struct etnaviv_iommu_context *old_context = gpu->mmu_context;

			gpu->mmu_context = etnaviv_iommu_context_get(mmu_context);
			etnaviv_iommu_context_put(old_context);
		}

		if (need_flush) {
			/*
			 * R52: flush MMU via direct MMIO, NOT via cmdbuf
			 * LOAD_STATE. VIPPico v3 hangs FE at CMD_LOAD_STATE
			 * VIVS_MMUv2_CONFIGURATION in the ring buffer. The BSP
			 * gckMMU flushes via a direct register write before
			 * issuing the FE LINK kick; mirror that pattern here.
			 *
			 * The gpu_write() happens while gpu->lock is held and
			 * before etnaviv_buffer_replace_wait() kicks the FE,
			 * so the MMIO flush completes before any FE fetch of
			 * the new cmdbuf.
			 */
			if (gpu->mmu_context->global->version == ETNAVIV_IOMMU_V1) {
				gpu_write(gpu, VIVS_GL_FLUSH_MMU,
					  VIVS_GL_FLUSH_MMU_FLUSH_FEMMU |
					  VIVS_GL_FLUSH_MMU_FLUSH_UNK1 |
					  VIVS_GL_FLUSH_MMU_FLUSH_UNK2 |
					  VIVS_GL_FLUSH_MMU_FLUSH_PEMMU |
					  VIVS_GL_FLUSH_MMU_FLUSH_UNK4);
			} else {
				u32 flush = VIVS_MMUv2_CONFIGURATION_MODE_MASK |
					    VIVS_MMUv2_CONFIGURATION_FLUSH_FLUSH;

				if (gpu->sec_mode == ETNA_SEC_NONE ||
				    gpu->identity.nn_core_count > 0)
					flush |= etnaviv_iommuv2_get_mtlb_addr(gpu->mmu_context);

				dev_info(gpu->dev,
					 "NPU MMU flush=0x%08x sec_mode=%d (R52: MMIO not cmdbuf)\n",
					 flush, gpu->sec_mode);

				/* Direct MMIO write -- no cmdbuf LOAD_STATE */
				gpu_write(gpu, VIVS_MMUv2_CONFIGURATION, flush);
			}

			gpu->flush_seq = new_flush_seq;
		}

		if (switch_context) {
			/* R51: skip etnaviv_cmd_select_pipe on 0x99
			 * (pipe_select emits GL_FLUSH_CACHE +
			 * GL_PIPE_SELECT LOAD_STATEs which hang FE on
			 * this silicon). NPU has no 2D/3D pipe to
			 * switch -- the call is dead code on NPU.
			 * Track exec_state without emitting commands.
			 */
			gpu->exec_state = exec_state;
		}

		/* And the link to the submitted buffer */
		link_target = etnaviv_cmdbuf_get_va(cmdbuf,
					&gpu->mmu_context->cmdbuf_mapping);
		CMD_LINK(buffer, link_dwords, link_target);

		/* Update the link target to point to above instructions */
		link_target = target;
		link_dwords = extra_dwords;
	}

	/*
	 * Append a LINK to the submitted command buffer to return to
	 * the ring buffer. For 0x99 NPU: 3 dwords (event + wait +
	 * link). GL_EVENT triggers the ISR; ISR defers to
	 * npu_poll_work which waits for NN/TP idle, then signals.
	 * WAIT+LINK keeps FE alive in the idle loop after the event
	 * fires.
	 */
	return_dwords = 3;

	return_target = etnaviv_buffer_reserve(gpu, buffer, return_dwords);
	CMD_LINK(cmdbuf, return_dwords, return_target);

	dev_info(gpu->dev, "NPU submit: return_target=0x%08x ring_va=0x%08x\n",
		 return_target,
		 etnaviv_cmdbuf_get_va(buffer, &gpu->mmu_context->cmdbuf_mapping));

	/*
	 * VIPPico v3 (S905D3, 0x99): SMALL_BATCH does NOT stall FE.
	 * GL_EVENT fires before NN completes. ISR defers to
	 * npu_poll_work which waits for NN/TP idle, then signals.
	 *
	 * Only defer events for cmdbufs with NN/TP instructions.
	 * init_npu has no NN/TP ops and must complete normally.
	 */
	if (gpu->npu_poll_completion) {
		bool has_nn_tp = false;
		unsigned int w, nw = cmdbuf->size / 4;
		u32 *cw = cmdbuf->vaddr;

		for (w = 0; w + 1 < nw; w += 2) {
			u16 off = cw[w] & VIV_FE_LOAD_STATE_HEADER_OFFSET__MASK;
			if (off == (VIVS_PS_NN_INST_ADDR >> 2) ||
			    off == (VIVS_PS_TP_INST_ADDR >> 2)) {
				has_nn_tp = true;
				break;
			}
		}
		if (has_nn_tp)
			gpu->npu_poll_event = event;
	}
	CMD_LOAD_STATE(buffer, VIVS_GL_EVENT,
		       VIVS_GL_EVENT_EVENT_ID(event) |
		       VIVS_GL_EVENT_FROM_FE);

	/* WAIT+LINK is needed to keep FE alive after GL_EVENT fires.
	 * On 0x99 the npu_poll_work signal is deferred; FE must be in
	 * the ring's idle loop, not executing stale ring buffer content.
	 */
	CMD_WAIT(buffer, gpu->fe_waitcycles);
	CMD_LINK(buffer, 2,
		 etnaviv_cmdbuf_get_va(buffer,
				&gpu->mmu_context->cmdbuf_mapping)
		 + buffer->user_size - 4);

	if (drm_debug_enabled(DRM_UT_DRIVER))
		pr_info("stream link to 0x%08x @ 0x%08x %p\n",
			return_target,
			etnaviv_cmdbuf_get_va(cmdbuf, &gpu->mmu_context->cmdbuf_mapping),
			cmdbuf->vaddr);

	if (drm_debug_enabled(DRM_UT_DRIVER)) {
		print_hex_dump(KERN_INFO, "cmd ", DUMP_PREFIX_OFFSET, 16, 4,
			       cmdbuf->vaddr, cmdbuf->size, 0);

		pr_info("link op: %p\n", buffer->vaddr + waitlink_offset);
		pr_info("addr: 0x%08x\n", link_target);
		pr_info("back: 0x%08x\n", return_target);
		pr_info("event: %d\n", event);
	}

	/*
	 * Kick off the submitted command by replacing the previous
	 * WAIT with a link to the address in the ring buffer.
	 */
	etnaviv_buffer_replace_wait(buffer, waitlink_offset,
				    VIV_FE_LINK_HEADER_OP_LINK |
				    VIV_FE_LINK_HEADER_PREFETCH(link_dwords),
				    link_target);

	if (drm_debug_enabled(DRM_UT_DRIVER))
		etnaviv_buffer_dump(gpu, buffer, 0, 0x50);
}
