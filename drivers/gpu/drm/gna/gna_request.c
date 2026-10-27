// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 2017-2022 Intel Corporation

#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_shmem_helper.h>

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/math.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/sched/signal.h>
#include <linux/slab.h>
#include <linux/timekeeping.h>
#include <linux/uaccess.h>

#include "gna_device.h"
#include "gna_hw.h"
#include "gna_mem.h"
#include "gna_request.h"
#include "gna_score.h"

int gna_validate_score_config(struct gna_compute_cfg *compute_cfg,
			struct gna_device *gna_priv)
{
	size_t buffers_size;

	if (compute_cfg->gna_mode > GNA_MODE_XNN) {
		dev_dbg(gna_dev(gna_priv), "invalid mode: %d\n", compute_cfg->gna_mode);
		return -EINVAL;
	}

	if (compute_cfg->layer_count > gna_priv->info.max_layer_count) {
		dev_dbg(gna_dev(gna_priv), "max layer count exceeded: %u > %u\n",
			compute_cfg->layer_count, gna_priv->info.max_layer_count);
		return -EINVAL;
	}

	if (compute_cfg->buffer_count == 0) {
		dev_dbg(gna_dev(gna_priv), "no buffers\n");
		return -EINVAL;
	}

	buffers_size = sizeof(struct gna_buffer) * compute_cfg->buffer_count;
	if (!access_ok(u64_to_user_ptr(compute_cfg->buffers_ptr), buffers_size))
		return -EACCES;

	return 0;
}

static void gna_request_update_status(struct gna_request *score_request)
{
	struct gna_device *gna_priv = to_gna_device(score_request->drm_f->minor->dev);
	/* The gna_priv's hw_status should be updated first */
	u32 hw_status = gna_priv->hw_status;
	u32 stall_cycles;
	u32 total_cycles;

	/* Technically, the time stamp can be a bit later than
	 * when the hw actually completed scoring. Here we just
	 * do our best in a deferred work, unless we want to
	 * tax isr for a more accurate record.
	 */
	score_request->drv_perf.hw_completed = ktime_get_ns();

	score_request->hw_status = hw_status;

	score_request->status = gna_parse_hw_status(gna_priv, hw_status);

	if (gna_hw_perf_enabled(gna_priv)) {
		if (hw_status & GNA_STS_STATISTICS_VALID) {
			total_cycles = gna_reg_read(gna_priv, GNA_MMIO_PTC);
			stall_cycles = gna_reg_read(gna_priv, GNA_MMIO_PSC);
			score_request->hw_perf.total = total_cycles;
			score_request->hw_perf.stall = stall_cycles;
		} else
			dev_warn(gna_dev(gna_priv), "GNA statistics missing\n");
	}
	if (unlikely(hw_status & GNA_ERROR))
		gna_print_error_status(gna_priv, hw_status);
}

static void gna_request_make_zombie(struct gna_request *score_request)
{
	int i;

	for (i = 0; i < score_request->buffer_count; i++) {
		kvfree((void *)(uintptr_t)score_request->buffer_list[i].gna.patches_ptr);
		drm_gem_object_put(&score_request->buffer_list[i].gem->base.base);
	}
	kvfree(score_request->buffer_list);
	score_request->buffer_list = NULL;
	score_request->buffer_count = 0;
}

/*
 * Drop a request's runtime-PM reference. The device uses autosuspend, so re-arm its timer instead of
 * pm_runtime_put(): that queues the runtime-idle work and wakes a kernel worker on every request, only for the work
 * to find the autosuspend delay not yet expired. Asynchronous, so allowed in the interrupt.
 */
static int gna_pm_put(struct gna_device *gna_priv)
{
	pm_runtime_mark_last_busy(gna_dev(gna_priv));
	return pm_runtime_put_autosuspend(gna_dev(gna_priv));
}

static void gna_request_process(struct work_struct *work)
{
	struct gna_request *score_request;
	struct gna_device *gna_priv;
	unsigned long hw_timeout;
	int ret;

	score_request = container_of(work, struct gna_request, work);
	gna_priv = to_gna_device(score_request->drm_f->minor->dev);

	gna_hw_acquire(gna_priv);        /* after a directly started request, if one is on the hardware */
	score_request->polled = false;   /* a queued request always completes by interrupt */
	score_request->state = ACTIVE;

	score_request->drv_perf.pre_processing = ktime_get_ns();

	ret = pm_runtime_get_sync(gna_dev(gna_priv));
	if (ret < 0 && ret != -EACCES) {
		dev_warn(gna_dev(gna_priv), "pm_runtime_get_sync() failed: %d\n", ret);
		score_request->status = -ENODEV;
		pm_runtime_put_noidle(gna_dev(gna_priv));
		goto tail;
	}

	/* Set busy flag before kicking off HW. The isr will clear it and wake up us. There is
	 * no difference if isr is missed in a timeout situation of the last request. We just
	 * always set it busy and let the wait_event_timeout check the reset.
	 * wq:  X -> true
	 * isr: X -> false
	 */
	gna_priv->dev_busy = true;

	ret = gna_score(score_request);
	if (ret) {
		if (gna_pm_put(gna_priv) < 0)
			dev_warn(gna_dev(gna_priv), "pm_runtime_put_autosuspend() failed: %d\n", ret);
		score_request->status = ret;
		goto tail;
	}

	score_request->drv_perf.processing = ktime_get_ns();

	hw_timeout = gna_priv->recovery_timeout_jiffies;

	hw_timeout = wait_event_timeout(gna_priv->dev_busy_waitq,
			!gna_priv->dev_busy, hw_timeout);

	if (!hw_timeout)
		dev_warn(gna_dev(gna_priv), "hardware timeout occurred\n");

	gna_priv->hw_status = gna_reg_read(gna_priv, GNA_MMIO_STS);

	gna_request_update_status(score_request);

	ret = gna_finish_hw(gna_priv, hw_timeout && score_request->status == 0);
	if (ret < 0 && score_request->status == 0)
		score_request->status = ret; // -ETIMEDOUT

	ret = gna_pm_put(gna_priv);
	if (ret < 0)
		dev_warn(gna_dev(gna_priv), "pm_runtime_put_autosuspend() failed: %d\n", ret);

	/* the page tables stay mapped for the next request (gna_mmu_map_request) */
tail:
	score_request->drv_perf.completion = ktime_get_ns();
	score_request->state = DONE;
	gna_request_make_zombie(score_request);
	gna_hw_release(gna_priv);

	atomic_dec(&gna_priv->enqueued_requests);
	wake_up_interruptible_all(&score_request->waitq);
}

/*
 * Finish a request started from COMPUTE (gna_request_start_direct): from the interrupt -- atomic context, so
 * nothing here sleeps or frees; the request's buffers are released with its last reference, in WAIT -- or from WAIT
 * when the interrupt never came (timed_out).
 */
void gna_request_complete_direct(struct gna_request *score_request, bool timed_out)
{
	struct gna_device *gna_priv = to_gna_device(score_request->drm_f->minor->dev);
	int ret;

	gna_priv->hw_status = gna_reg_read(gna_priv, GNA_MMIO_STS);
	gna_request_update_status(score_request);
	/*
	 * A request run with its interrupt masked (polled) leaves its completion latched: a request that enables the
	 * interrupt would take it at once, before its own work is done, and read "not completed". A following polled
	 * request is unaffected (START clears the completion bit), so the abort that clears the latch waits for the
	 * start that needs it (gna_score); the abort, whenever it runs, clears the mark.
	 */
	if (!timed_out && score_request->status == 0 && score_request->polled)
		gna_priv->completion_latched = true;
	ret = gna_finish_hw(gna_priv, !timed_out && score_request->status == 0);
	if (timed_out)
		score_request->status = -ETIMEDOUT;
	else if (ret < 0 && score_request->status == 0)
		score_request->status = ret;
	gna_pm_put(gna_priv);

	score_request->drv_perf.completion = ktime_get_ns();
	score_request->state = DONE;
	atomic_dec(&gna_priv->enqueued_requests);
	gna_hw_release(gna_priv);
	wake_up_interruptible_all(&score_request->waitq);
}

/* how often a request started with its interrupt masked was completed, and by whom */
static unsigned int polled_reclaim_acquire;    /* by the next user of the hardware */
static unsigned int polled_reclaim_wait;       /* by its WAIT, already finished */
static unsigned int polled_wait_polls;         /* WAITs that found the hardware still running and polled */
static unsigned int polled_sts_done_at_start;  /* STS bit 0 still set right after START: polling would be wrong */
module_param(polled_sts_done_at_start, uint, 0444);
module_param(polled_reclaim_acquire, uint, 0444);
module_param(polled_reclaim_wait, uint, 0444);
module_param(polled_wait_polls, uint, 0444);

/*
 * The hardware is owned by one request at a time (hw_sem). A request started with its completion interrupt masked
 * (GNA_FLAG_SCORE_POLLED) keeps owning it after the hardware has finished, until someone reads the status: complete
 * it here if it has finished. True when a request was completed. Process context.
 */
bool gna_hw_reclaim(struct gna_device *gna_priv)
{
	struct gna_request *req = NULL;
	unsigned long flags;

	spin_lock_irqsave(&gna_priv->direct_lock, flags);
	if (gna_priv->direct_req && gna_priv->direct_req->polled &&
	    (gna_reg_read(gna_priv, GNA_MMIO_STS) & (GNA_STS_SCORE_COMPLETED | GNA_ERROR))) {
		req = gna_priv->direct_req;
		gna_priv->direct_req = NULL;
	}
	spin_unlock_irqrestore(&gna_priv->direct_lock, flags);
	if (!req)
		return false;
	gna_priv->dev_busy = false;
	gna_request_complete_direct(req, false);
	return true;
}

/*
 * Take the hardware. Every path that needs it goes through these, so a finished masked request is always reclaimed
 * by whoever needs the hardware next -- none of them can wait for an interrupt that will not come.
 */
static bool gna_hw_try_acquire(struct gna_device *gna_priv)
{
	if (!down_trylock(&gna_priv->hw_sem))
		return true;
	if (!gna_hw_reclaim(gna_priv))
		return false;
	polled_reclaim_acquire++;
	return !down_trylock(&gna_priv->hw_sem);
}

void gna_hw_acquire(struct gna_device *gna_priv)
{
	while (down_timeout(&gna_priv->hw_sem, msecs_to_jiffies(1)))
		if (gna_hw_reclaim(gna_priv))
			polled_reclaim_acquire++;
}

void gna_hw_release(struct gna_device *gna_priv)
{
	up(&gna_priv->hw_sem);
}

/*
 * WAIT on a request started with its interrupt masked: complete it if the hardware has finished, else poll at a
 * short interval until it has or the timeout passes. Returns as wait_event_interruptible_timeout does: > 0 done,
 * 0 timed out, -ERESTARTSYS on a signal.
 */
long gna_request_wait_polled(struct gna_device *gna_priv, struct gna_request *score_request, u32 timeout_ms)
{
	unsigned long end = jiffies + msecs_to_jiffies(timeout_ms);
	bool polled = false;

	while (READ_ONCE(score_request->state) != DONE) {
		if (gna_hw_reclaim(gna_priv)) {
			if (!polled)
				polled_reclaim_wait++;
			continue;
		}
		if (signal_pending(current))
			return -ERESTARTSYS;
		if (time_after(jiffies, end))
			return 0;
		if (!polled) {
			polled = true;
			polled_wait_polls++;
		}
		usleep_range(50, 100);
	}
	return 1;
}

/*
 * Nothing queued and nothing on the hardware: start the request here, in the submitting thread, and return
 * without waiting -- no workqueue thread is woken, and the interrupt completes it (gna_request_complete_direct).
 * Called with hw_sem held; false (hw_sem given back) when the device could not be resumed, for the workqueue to
 * report the failure as before.
 */
static bool gna_request_start_direct(struct gna_device *gna_priv, struct gna_request *score_request)
{
	unsigned long flags;
	bool owned;
	int ret;

	ret = pm_runtime_get_sync(gna_dev(gna_priv));
	if (ret < 0 && ret != -EACCES) {
		pm_runtime_put_noidle(gna_dev(gna_priv));
		up(&gna_priv->hw_sem);
		return false;
	}

	score_request->polled = !!(score_request->compute_cfg.flags & GNA_FLAG_SCORE_POLLED);
	score_request->state = ACTIVE;
	score_request->drv_perf.pre_processing = ktime_get_ns();
	gna_priv->dev_busy = true;
	spin_lock_irqsave(&gna_priv->direct_lock, flags);
	gna_priv->direct_req = score_request;       /* before the hardware starts: its interrupt completes it */
	spin_unlock_irqrestore(&gna_priv->direct_lock, flags);
	score_request->drv_perf.processing = ktime_get_ns();

	ret = gna_score(score_request);
	/* completion is read from STS bit 0 for a masked request, so it must not still read set from the previous one */
	if (!ret && score_request->polled &&
	    (gna_reg_read(gna_priv, GNA_MMIO_STS) & GNA_STS_SCORE_COMPLETED))
		polled_sts_done_at_start++;
	if (ret) {
		spin_lock_irqsave(&gna_priv->direct_lock, flags);
		owned = gna_priv->direct_req == score_request;
		if (owned)
			gna_priv->direct_req = NULL;
		spin_unlock_irqrestore(&gna_priv->direct_lock, flags);
		if (owned) {
			gna_pm_put(gna_priv);
			score_request->status = ret;
			score_request->drv_perf.completion = ktime_get_ns();
			score_request->state = DONE;
			atomic_dec(&gna_priv->enqueued_requests);
			up(&gna_priv->hw_sem);
			wake_up_interruptible_all(&score_request->waitq);
		}
	}
	return true;
}

static struct gna_request *gna_request_create(struct drm_file *file,
				       struct gna_compute_cfg *compute_cfg)
{

	struct gna_device *gna_priv = to_gna_device(file->minor->dev);
	struct gna_request *score_request;

	score_request = kzalloc(sizeof(*score_request), GFP_KERNEL);
	if (!score_request)
		return NULL;
	kref_init(&score_request->refcount);

	dev_dbg(gna_dev(gna_priv), "labase: %d, lacount: %d\n",
		compute_cfg->layer_base, compute_cfg->layer_count);

	score_request->request_id = atomic_inc_return(&gna_priv->request_count);
	score_request->compute_cfg = *compute_cfg;
	score_request->drm_f = file;
	score_request->state = NEW;
	init_waitqueue_head(&score_request->waitq);
	INIT_WORK(&score_request->work, gna_request_process);
	INIT_LIST_HEAD(&score_request->node);

	return score_request;
}

/*
 * returns true if [inner_offset, inner_size) is embraced by [0, outer_size). False otherwise.
 */
static bool gna_validate_ranges(u64 outer_size, u64 inner_offset, u64 inner_size)
{
	return inner_offset < outer_size &&
		inner_size <= (outer_size - inner_offset);
}

static int gna_validate_patches(struct gna_device *gna_priv, __u64 buffer_size,
				struct gna_memory_patch *patches, u64 count)
{
	u64 idx;

	for (idx = 0; idx < count; ++idx) {
		if (patches[idx].size > 8) {
			dev_dbg(gna_dev(gna_priv), "invalid patch size: %llu\n", patches[idx].size);
			return -EINVAL;
		}

		if (!gna_validate_ranges(buffer_size, patches[idx].offset, patches[idx].size)) {
			dev_dbg(gna_dev(gna_priv),
				"patch out of bounds. buffer size: %llu, patch offset/size:%llu/%llu\n",
				buffer_size, patches[idx].offset, patches[idx].size);
			return -EINVAL;
		}
	}

	return 0;
}

static int gna_buffer_fill_patches(struct gna_buffer *buffer, struct gna_device *gna_priv)
{
	__u64 patches_user = buffer->patches_ptr;
	struct gna_memory_patch *patches;
	/* At this point, the buffer points to a memory region in kernel space where the copied
	 * patches_ptr also lives, but the value of it is still an address from user space. This
	 * function will set patches_ptr to either an address in kernel space or null before it
	 * exits.
	 */
	u64 patch_count;
	int ret;

	buffer->patches_ptr = 0;
	patch_count = buffer->patch_count;
	if (!patch_count)
		return 0;

	patches = kvmalloc_array(patch_count, sizeof(struct gna_memory_patch), GFP_KERNEL);
	if (!patches)
		return -ENOMEM;

	if (copy_from_user(patches, u64_to_user_ptr(patches_user),
				sizeof(struct gna_memory_patch) * patch_count)) {
		ret = -EFAULT;
		goto err_fill_patches;
	}

	ret = gna_validate_patches(gna_priv, buffer->size, patches, patch_count);
	if (ret) {
		dev_dbg(gna_dev(gna_priv), "buffer %p: patches' validation failed\n", buffer);
		goto err_fill_patches;
	}

	buffer->patches_ptr = (uintptr_t)patches;

	return 0;

err_fill_patches:
	kvfree(patches);
	return ret;
}

static int gna_request_fill_buffers(struct gna_request *score_request,
				    struct gna_compute_cfg *compute_cfg)
{
	struct gna_buffer_with_object *buffer_list;
	struct gna_buffer_with_object *buffer;
	struct gna_buffer *cfg_buffers;
	struct drm_gem_object *drmgemo;
	struct gna_device *gna_priv;
	u64 buffers_total_size = 0;
	size_t gem_obj_size;
	u64 buffer_count;
	u32 handle;
	u64 i, j;
	int ret;


	gna_priv = to_gna_device(score_request->drm_f->minor->dev);

	buffer_count = compute_cfg->buffer_count;
	buffer_list = kvmalloc_array(buffer_count, sizeof(*buffer_list), GFP_KERNEL);
	if (!buffer_list)
		return -ENOMEM;

	cfg_buffers = u64_to_user_ptr(compute_cfg->buffers_ptr);
	for (i = 0; i < buffer_count; ++i) {
		if (copy_from_user(&buffer_list[i].gna, cfg_buffers+i,
					sizeof(*buffer_list))) {
			ret = -EFAULT;
			goto err_free_buffers;
		}
		buffer_list[i].gem = NULL;
	}

	for (i = 0; i < buffer_count; i++) {
		buffer = &buffer_list[i];
		handle = buffer->gna.handle;

		if (buffer->gna.offset != 0) {
			dev_dbg(gna_dev(gna_priv), "buffer->offset = %llu for handle %u in score config\n",
				buffer->gna.offset, buffer->gna.handle);
			return -EINVAL;
		}

		for (j = 0; j < i; j++) {
			if (buffer_list[j].gna.handle == handle) {
				dev_dbg(gna_dev(gna_priv),
					"doubled memory id in score config; id:%u\n", handle);
				ret = -EINVAL;
				goto err_zero_patch_user_ptr;
			}
		}

		buffers_total_size +=
			round_up(buffer->gna.size, PAGE_SIZE);
		if (buffers_total_size > gna_priv->info.max_hw_mem) {
			dev_dbg(gna_dev(gna_priv), "buffers' %p total size too big\n", buffer);
			ret = -EINVAL;
			goto err_zero_patch_user_ptr;
		}

		drmgemo = drm_gem_object_lookup(score_request->drm_f, handle);

		if (!drmgemo) {
			dev_dbg(gna_dev(gna_priv), "memory object %u not found\n", handle);
			ret = -EINVAL;
			goto err_zero_patch_user_ptr;
		}

		// we are still in sys call context, but prior request is enqueued.
		// request may slip into queue while some gna_gem_object being deleted
		// border case + not too much harm.
		buffer->gem = to_gna_gem_obj(to_drm_gem_shmem_obj(drmgemo));

		gem_obj_size = drmgemo->size;

		if (!gna_validate_ranges(gem_obj_size, 0, buffer->gna.size)) {
			dev_dbg(gna_dev(gna_priv),
				"buffer out of bounds. mo size: %zu, buffer size:%llu\n",
				gem_obj_size, buffer->gna.size);
			ret = -EINVAL;
			goto err_zero_patch_user_ptr;
		}

		ret = gna_buffer_fill_patches(&buffer->gna, gna_priv);
		if (ret)
			goto err_free_patches;
	}

	score_request->buffer_list = buffer_list;
	score_request->buffer_count = buffer_count;

	return 0;

err_zero_patch_user_ptr:
	/* patches_ptr may still hold an address in userspace.
	 * Don't pass it to kvfree().
	 */
	buffer->gna.patches_ptr = 0;

err_free_patches:
	/* patches_ptr of each processed buffer should be either
	 * null or pointing to an allocated memory block in the
	 * kernel at this point.
	 */
	for (j = 0; j <= i; j++) {
		kvfree((void *)(uintptr_t)buffer_list[j].gna.patches_ptr);
		drm_gem_object_put(&buffer_list[j].gem->base.base);
	}

err_free_buffers:
	kvfree(buffer_list);
	return ret;
}

int gna_enqueue_request(struct gna_compute_cfg *compute_cfg,
			struct drm_file *file, u64 *request_id)
{
	bool is_qos = !!(compute_cfg->flags & GNA_FLAG_SCORE_QOS);
	struct gna_device *gna_priv = to_gna_device(file->minor->dev);
	struct gna_request *score_request;
	u64 pos_in_queue;
	int ret;

	/* a finished masked request still counts as queued until reclaimed: reclaim it first, so this one can start
	 * directly instead of queueing behind it */
	if (gna_hw_reclaim(gna_priv))
		polled_reclaim_acquire++;
	pos_in_queue = atomic_inc_return(&gna_priv->enqueued_requests);
	if (is_qos && pos_in_queue != 1) {
		ret = -EBUSY;
		goto ERR_UNQUEUE_REQUEST;
	}

	score_request = gna_request_create(file, compute_cfg);
	if (!score_request) {
		ret = -ENOMEM;
		goto ERR_UNQUEUE_REQUEST;
	}

	ret = gna_request_fill_buffers(score_request, compute_cfg);
	if (ret) {
		kref_put(&score_request->refcount, gna_request_release);
		goto ERR_UNQUEUE_REQUEST;
	}

	kref_get(&score_request->refcount);
	mutex_lock(&gna_priv->reqlist_lock);
	list_add_tail(&score_request->node, &gna_priv->request_list);
	mutex_unlock(&gna_priv->reqlist_lock);

	if (!(pos_in_queue == 1 && gna_hw_try_acquire(gna_priv) &&
	      gna_request_start_direct(gna_priv, score_request)))
		queue_work(gna_priv->request_wq, &score_request->work);
	kref_put(&score_request->refcount, gna_request_release);

	*request_id = score_request->request_id;

	return 0;

ERR_UNQUEUE_REQUEST:
	atomic_dec(&gna_priv->enqueued_requests);
	return ret;
}

void gna_request_release(struct kref *ref)
{
	struct gna_request *score_request =
		container_of(ref, struct gna_request, refcount);
	gna_request_make_zombie(score_request);
	wake_up_interruptible_all(&score_request->waitq);
	kfree(score_request);
}
