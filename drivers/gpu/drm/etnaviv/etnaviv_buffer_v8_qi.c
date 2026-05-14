// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2014-2018 Etnaviv Project
 *
 * Per-chip cmdbuf submission entry point for A311D NPU
 * (VIPNano-QI v8, customer_id 0x88).
 *
 * Phase A initial mechanical split (amlogic/software/npu/CLAUDE.md
 * "Per-chip code-path split"). Body is initially a duplicate of
 * etnaviv_buffer_queue to preserve functional equivalence at the
 * cut-over commit. Subsequent commits simplify this file to
 * straight-line A311D logic by removing the customer_id == 0x99
 * gates that don't apply here.
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

void etnaviv_buffer_queue_v8_qi(struct etnaviv_gpu *gpu, u32 exec_state,
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
	bool has_blt = !!(gpu->identity.minor_features5 &
			  chipMinorFeatures5_BLT_ENGINE);
	bool is_npu = gpu->identity.nn_core_count > 0;

	lockdep_assert_held(&gpu->lock);

	if (drm_debug_enabled(DRM_UT_DRIVER))
		etnaviv_buffer_dump(gpu, buffer, 0, 0x50);

	link_target = etnaviv_cmdbuf_get_va(cmdbuf,
					    &gpu->mmu_context->cmdbuf_mapping);
	link_dwords = cmdbuf->size / 8;

	if (is_npu) {
		u32 *cmdbuf_words = cmdbuf->vaddr;
		unsigned int nwords = cmdbuf->size / 4;

		dev_info(gpu->dev, "NPU submit: cmdbuf_va=0x%08x size=%u switch_ctx=%d need_flush=%d flush_seq=%u/%u switch_mmu=%d\n",
			 link_target, cmdbuf->size, switch_context, need_flush,
			 gpu->flush_seq, new_flush_seq, switch_mmu_context);
		/* Scan for NN_INST_ADDR (0x0428) or TP_INST_ADDR (0x042E) */
		{
			unsigned int w;
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
	}

	/*
	 * If we need maintenance prior to submitting this buffer, we will
	 * need to append a mmu flush load state, followed by a new
	 * link to this buffer - a total of four additional words.
	 */
	if (need_flush || switch_context) {
		u32 target, extra_dwords;

		/* link command */
		extra_dwords = 1;

		/* flush command */
		if (need_flush) {
			if (gpu->mmu_context->global->version == ETNAVIV_IOMMU_V1)
				extra_dwords += 1;
			else
				extra_dwords += 3;
		}

		/* pipe switch commands */
		if (switch_context)
			extra_dwords += 4;

		/* PTA load command -- emitted on A311D 0x88 (this file) when
		 * SEC_KERNEL switch_mmu_context. The 0x99 customer skips it
		 * (separate file etnaviv_buffer_v8_pico.c).
		 */
		if (switch_mmu_context && gpu->sec_mode == ETNA_SEC_KERNEL)
			extra_dwords += 1;

		target = etnaviv_buffer_reserve(gpu, buffer, extra_dwords);
		/*
		 * Switch MMU context if necessary. Must be done after the
		 * link target has been calculated, as the jump forward in the
		 * kernel ring still uses the last active MMU context before
		 * the switch.
		 */
		if (switch_mmu_context) {
			struct etnaviv_iommu_context *old_context = gpu->mmu_context;

			gpu->mmu_context = etnaviv_iommu_context_get(mmu_context);
			etnaviv_iommu_context_put(old_context);
		}

		if (need_flush) {
			/* Add the MMU flush */
			if (gpu->mmu_context->global->version == ETNAVIV_IOMMU_V1) {
				CMD_LOAD_STATE(buffer, VIVS_GL_FLUSH_MMU,
					       VIVS_GL_FLUSH_MMU_FLUSH_FEMMU |
					       VIVS_GL_FLUSH_MMU_FLUSH_UNK1 |
					       VIVS_GL_FLUSH_MMU_FLUSH_UNK2 |
					       VIVS_GL_FLUSH_MMU_FLUSH_PEMMU |
					       VIVS_GL_FLUSH_MMU_FLUSH_UNK4);
			} else {
				u32 flush = VIVS_MMUv2_CONFIGURATION_MODE_MASK |
					    VIVS_MMUv2_CONFIGURATION_FLUSH_FLUSH;

				if (switch_mmu_context &&
				    gpu->sec_mode == ETNA_SEC_KERNEL) {
					unsigned short id =
						etnaviv_iommuv2_get_pta_id(gpu->mmu_context);
					CMD_LOAD_STATE(buffer,
						VIVS_MMUv2_PTA_CONFIG,
						VIVS_MMUv2_PTA_CONFIG_INDEX(id));
				}

				if (gpu->sec_mode == ETNA_SEC_NONE ||
				    gpu->identity.nn_core_count > 0)
					flush |= etnaviv_iommuv2_get_mtlb_addr(gpu->mmu_context);

				if (gpu->identity.nn_core_count > 0)
					dev_info(gpu->dev,
						 "NPU MMU flush=0x%08x sec_mode=%d\n",
						 flush, gpu->sec_mode);

				CMD_LOAD_STATE(buffer, VIVS_MMUv2_CONFIGURATION,
					       flush);
				if (gpu->identity.nn_core_count > 0) {
					/*
					 * A311D 0x88 NPU: no PE for SEM/STALL.
					 * Use WAIT to ensure MMU TLB flush
					 * completes before NN/TP accesses.
					 */
					CMD_WAIT(buffer, 32000);
				} else {
					CMD_SEM(buffer, SYNC_RECIPIENT_FE,
						SYNC_RECIPIENT_PE);
					CMD_STALL(buffer, SYNC_RECIPIENT_FE,
						SYNC_RECIPIENT_PE);
				}
			}

			gpu->flush_seq = new_flush_seq;
		}

		if (switch_context) {
			etnaviv_cmd_select_pipe(gpu, buffer, exec_state);
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
	 * the ring buffer.
	 *
	 * For A311D NPU: 3 dwords (event + wait + link). GL_EVENT
	 * triggers the ISR; ISR signals fence immediately (SMALL_BATCH
	 * stalls FE at NN_INST_ADDR). WAIT+LINK keeps FE alive in the
	 * idle loop after the event fires.
	 */
	if (is_npu)
		return_dwords = 3;
	else
		return_dwords = 7;

	/*
	 * When the BLT engine is present we need 6 more dwords in the return
	 * target: 3 enable/flush/disable + 4 enable/semaphore stall/disable,
	 * but we don't need the normal TS flush state.
	 */
	if (has_blt)
		return_dwords += 6;

	return_target = etnaviv_buffer_reserve(gpu, buffer, return_dwords);
	CMD_LINK(cmdbuf, return_dwords, return_target);

	if (is_npu)
		dev_info(gpu->dev, "NPU submit: return_target=0x%08x ring_va=0x%08x\n",
			 return_target,
			 etnaviv_cmdbuf_get_va(buffer, &gpu->mmu_context->cmdbuf_mapping));

	/*
	 * Append a cache flush, stall, event, wait and link pointing back to
	 * the wait command to the ring buffer.
	 *
	 * For NPU-only A311D: skip the cache flush and PE semaphore/stall
	 * since there is no Pixel Engine. Use FROM_FE for the event instead
	 * of FROM_PE.
	 */
	if (is_npu) {
		/* A311D 0x88 NPU: GL_EVENT FROM_FE; SMALL_BATCH stalls FE
		 * at NN_INST_ADDR, so GL_EVENT fires after NN completes.
		 * ISR signals fence immediately (no npu_poll deferral on 0x88).
		 */
		CMD_LOAD_STATE(buffer, VIVS_GL_EVENT,
			       VIVS_GL_EVENT_EVENT_ID(event) |
			       VIVS_GL_EVENT_FROM_FE);
	} else {
		if (gpu->exec_state == ETNA_PIPE_2D) {
			CMD_LOAD_STATE(buffer, VIVS_GL_FLUSH_CACHE,
					       VIVS_GL_FLUSH_CACHE_PE2D);
		} else {
			CMD_LOAD_STATE(buffer, VIVS_GL_FLUSH_CACHE,
					       VIVS_GL_FLUSH_CACHE_DEPTH |
					       VIVS_GL_FLUSH_CACHE_COLOR |
					       VIVS_GL_FLUSH_CACHE_SHADER_L1);
			if (has_blt) {
				CMD_LOAD_STATE(buffer, VIVS_BLT_ENABLE, 0x1);
				CMD_LOAD_STATE(buffer, VIVS_BLT_SET_COMMAND, 0x1);
				CMD_LOAD_STATE(buffer, VIVS_BLT_ENABLE, 0x0);
			} else {
				CMD_LOAD_STATE(buffer, VIVS_TS_FLUSH_CACHE,
						       VIVS_TS_FLUSH_CACHE_FLUSH);
			}
		}
		CMD_SEM(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_PE);
		CMD_STALL(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_PE);

		if (has_blt) {
			CMD_LOAD_STATE(buffer, VIVS_BLT_ENABLE, 0x1);
			CMD_SEM(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_BLT);
			CMD_STALL(buffer, SYNC_RECIPIENT_FE, SYNC_RECIPIENT_BLT);
			CMD_LOAD_STATE(buffer, VIVS_BLT_ENABLE, 0x0);
		}

		CMD_LOAD_STATE(buffer, VIVS_GL_EVENT,
			       VIVS_GL_EVENT_EVENT_ID(event) |
			       VIVS_GL_EVENT_FROM_PE);
	}

	/* WAIT+LINK keeps FE alive in the ring's idle loop after
	 * GL_EVENT fires (signal happens immediately on 0x88; FE must
	 * not execute stale ring buffer content).
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
