// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017 Etnaviv Project
 */

#include <linux/moduleparam.h>

#include "etnaviv_cmdbuf.h"
#include "etnaviv_drv.h"
#include "etnaviv_dump.h"
#include "etnaviv_gem.h"
#include "etnaviv_gpu.h"
#include "etnaviv_mmu.h"
#include "etnaviv_sched.h"
#include "state.xml.h"
#include "state_hi.xml.h"

static int etnaviv_job_hang_limit = 0;
module_param_named(job_hang_limit, etnaviv_job_hang_limit, int , 0444);
static int etnaviv_hw_jobs_limit = 4;
module_param_named(hw_job_limit, etnaviv_hw_jobs_limit, int , 0444);

static struct dma_fence *etnaviv_sched_run_job(struct drm_sched_job *sched_job)
{
	struct etnaviv_gem_submit *submit = to_etnaviv_submit(sched_job);
	struct dma_fence *fence = NULL;

	if (likely(!sched_job->s_fence->finished.error))
		fence = etnaviv_gpu_submit(submit);
	else
		dev_dbg(submit->gpu->dev, "skipping bad job\n");

	return fence;
}

static enum drm_gpu_sched_stat etnaviv_sched_timedout_job(struct drm_sched_job
							  *sched_job)
{
	struct etnaviv_gem_submit *submit = to_etnaviv_submit(sched_job);
	struct drm_gpu_scheduler *sched = sched_job->sched;
	struct etnaviv_gpu *gpu = submit->gpu;
	u32 dma_addr;
	int change;

	/*
	 * If the GPU managed to complete this jobs fence, the timout is
	 * spurious. Bail out.
	 */
	if (dma_fence_is_signaled(submit->out_fence))
		goto out_no_timeout;

	/*
	 * If the GPU is still making forward progress on the front-end (which
	 * should never loop) we shift out the timeout to give it a chance to
	 * finish the job.
	 */
	dma_addr = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);
	change = dma_addr - gpu->hangcheck_dma_addr;
	if (gpu->state == ETNA_GPU_STATE_RUNNING &&
	    (gpu->completed_fence != gpu->hangcheck_fence ||
	     change < 0 || change > 16)) {
		gpu->hangcheck_dma_addr = dma_addr;
		gpu->hangcheck_fence = gpu->completed_fence;
		goto out_no_timeout;
	}

	/*
	 * VIPPico v3 (S905D3, customer 0x99) deferred-completion re-arm.
	 *
	 * On NPU chips that use npu_poll_completion (GL_EVENT fires from FE
	 * before NN/TP finish; fence signaling is deferred to npu_poll_work
	 * which polls VIVS_HI_IDLE_STATE_NN/_TP and signals), the FE
	 * legitimately parks at WAIT+LINK in the kernel ring buffer for the
	 * entire duration of NN/TP execution. dma_addr stays in the ring's
	 * 8-byte WAIT/LINK loop -- |change| <= 4 -- which the forward-
	 * progress check above classifies as a hang.
	 *
	 * For models where NN/TP execution + npu_poll_work latency exceed the
	 * effective drm_sched timeout (which the nn_core_count=0 race at
	 * sched_init time pins to the non-NPU 500 ms default), this triggers
	 * a watchdog false-positive on /019 (movenetlightning single-op,
	 * ~640 ms total) and similar long-running ops. The recover_hang path
	 * cascades on customer 0x99 into a kernel WARN at
	 * etnaviv_gem_cpu_fini+0x88, then the genpd autosuspend timer drops
	 * the NNA power domain via meson-ee-pwrc, then the in-flight teflon
	 * userspace stuck on a never-signaled fence under hung_task_panic=1
	 * triggers panic=30 reboot.
	 *
	 * The disease is "recover hangs that aren't hangs". Detect the
	 * deferred-pending state precisely:
	 *   1. npu_poll_completion enabled (only customer 0x99 today)
	 *   2. an event is still pending fence signal from npu_poll_work
	 *      (npu_poll_event >= 0; cleared to -1 once poll_work signals)
	 *   3. dma_addr is inside the kernel ring buffer (FE has returned
	 *      from the user cmdbuf to the WAIT+LINK loop; it has executed
	 *      the GL_EVENT and is waiting)
	 *   4. NN and TP idle bits set in VIVS_HI_IDLE_STATE (idle = 1
	 *      convention; bits 17 and 18). Both set means the engines are
	 *      idle -- either work is done (poll_work will signal soon) or
	 *      never started (a different bug). Either way, this is not an
	 *      FE-side hang -- the FE is correctly parked.
	 *
	 * Re-arm the watchdog. The npu_poll_work iteration limit
	 * (100,000 * usleep_range(100,200) = ~20 s upper bound) provides a
	 * fallback: even a fully wedged poll_work signals the fence within
	 * 20 s, after which dma_fence_is_signaled at the top of this function
	 * exits cleanly on the next watchdog tick.
	 *
	 * A311D 0x88 has npu_poll_completion=false; it falls through this
	 * block unchanged and continues to recover_hang on real timeouts as
	 * today. No timing change for either chip; only the recover decision
	 * changes for customer 0x99.
	 */
	if (gpu->npu_poll_completion &&
	    gpu->npu_poll_event >= 0 &&
	    gpu->buffer.vaddr && gpu->buffer.size && gpu->mmu_context) {
		u32 ring_va = etnaviv_cmdbuf_get_va(&gpu->buffer,
				&gpu->mmu_context->cmdbuf_mapping);
		u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

		if (dma_addr >= ring_va &&
		    dma_addr < ring_va + gpu->buffer.size &&
		    (idle & VIVS_HI_IDLE_STATE_NN) &&
		    (idle & VIVS_HI_IDLE_STATE_TP)) {
			dev_dbg(gpu->dev,
				"sched timeout re-armed: NPU deferred-completion pending (dma=0x%08x ring=0x%08x idle=0x%08x event=%d)\n",
				dma_addr, ring_va, idle, gpu->npu_poll_event);
			gpu->hangcheck_dma_addr = dma_addr;
			gpu->hangcheck_fence = gpu->completed_fence;
			goto out_no_timeout;
		}
	}

	/* block scheduler */
	drm_sched_stop(&gpu->sched, sched_job);

	if(sched_job)
		drm_sched_increase_karma(sched_job);

	/* get the GPU back into the init state */
	etnaviv_core_dump(submit);
	etnaviv_gpu_recover_hang(submit);

	drm_sched_resubmit_jobs(&gpu->sched);

	drm_sched_start(&gpu->sched);
	return DRM_GPU_SCHED_STAT_NOMINAL;

out_no_timeout:
	spin_lock(&sched->job_list_lock);
	list_add(&sched_job->list, &sched->pending_list);
	spin_unlock(&sched->job_list_lock);
	return DRM_GPU_SCHED_STAT_NOMINAL;
}

static void etnaviv_sched_free_job(struct drm_sched_job *sched_job)
{
	struct etnaviv_gem_submit *submit = to_etnaviv_submit(sched_job);

	drm_sched_job_cleanup(sched_job);

	etnaviv_submit_put(submit);
}

static const struct drm_sched_backend_ops etnaviv_sched_ops = {
	.run_job = etnaviv_sched_run_job,
	.timedout_job = etnaviv_sched_timedout_job,
	.free_job = etnaviv_sched_free_job,
};

int etnaviv_sched_push_job(struct etnaviv_gem_submit *submit)
{
	struct etnaviv_gpu *gpu = submit->gpu;
	int ret;

	/*
	 * Hold the sched lock across the whole operation to avoid jobs being
	 * pushed out of order with regard to their sched fence seqnos as
	 * allocated in drm_sched_job_arm.
	 */
	mutex_lock(&gpu->sched_lock);

	drm_sched_job_arm(&submit->sched_job);

	submit->out_fence = dma_fence_get(&submit->sched_job.s_fence->finished);
	ret = xa_alloc_cyclic(&gpu->user_fences, &submit->out_fence_id,
			      submit->out_fence, xa_limit_32b,
			      &gpu->next_user_fence, GFP_KERNEL);
	if (ret < 0) {
		drm_sched_job_cleanup(&submit->sched_job);
		goto out_unlock;
	}

	/* the scheduler holds on to the job now */
	kref_get(&submit->refcount);

	drm_sched_entity_push_job(&submit->sched_job);

out_unlock:
	mutex_unlock(&gpu->sched_lock);

	return ret;
}

int etnaviv_sched_init(struct etnaviv_gpu *gpu)
{
	int ret;

	/*
	 * NPU ops can take >500ms per layer (e.g. 224x224 conv on 2-core
	 * VIPPico). Use longer timeout for NPU to avoid false hang detection
	 * during kernel-side NN/TP idle polling + FLUSH_CACHE.
	 */
	/*
	 * NPU poll completion requires strictly sequential execution:
	 * after each NN/TP operation, CPU must poll for idle and issue
	 * FLUSH_CACHE before the next operation can start. Limit to
	 * 1 in-flight job so the ring buffer isn't overwritten.
	 */
	/* r93 probe: bump NPU timeout 10s -> 60s to test if "recover hung GPU"
	 * is a watchdog misfire on legitimate silicon work. mobilenet_v1
	 * inference takes ~280ms; 10s should be plenty but timeouts fire at
	 * ~1s post-submit on every inference. Try 60s. */
	ret = drm_sched_init(&gpu->sched, &etnaviv_sched_ops, NULL,
			     DRM_SCHED_PRIORITY_COUNT,
			     gpu->identity.nn_core_count > 0 ?
				1 : etnaviv_hw_jobs_limit,
			     etnaviv_job_hang_limit,
			     msecs_to_jiffies(gpu->identity.nn_core_count > 0 ?
					      60000 : 500),
			     NULL, NULL,
			     dev_name(gpu->dev), gpu->dev);
	if (ret)
		return ret;

	return 0;
}

void etnaviv_sched_fini(struct etnaviv_gpu *gpu)
{
	drm_sched_fini(&gpu->sched);
}
