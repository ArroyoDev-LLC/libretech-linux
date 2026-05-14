// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic Meson DRM Writeback Connector
 * Uses GE2D 2D engine to capture primary plane framebuffer content.
 *
 * Copyright (C) 2025 Libre Computer
 */

#include <linux/soc/amlogic/meson-ge2d.h>

#include <drm/drm_atomic.h>
#include <drm/drm_edid.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_writeback.h>

#include "meson_drv.h"
#include "meson_writeback.h"

static const u32 meson_wb_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

static const struct drm_connector_funcs meson_wb_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int meson_wb_atomic_check(struct drm_connector *connector,
				 struct drm_atomic_state *state)
{
	struct drm_connector_state *conn_state =
		drm_atomic_get_new_connector_state(state, connector);
	struct drm_crtc_state *crtc_state;
	struct drm_framebuffer *fb;
	int ret;

	if (!conn_state->writeback_job || !conn_state->writeback_job->fb)
		return 0;

	if (!conn_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_new_crtc_state(state, conn_state->crtc);
	fb = conn_state->writeback_job->fb;

	/* Writeback FB must match display mode dimensions */
	if (fb->width != crtc_state->mode.hdisplay ||
	    fb->height != crtc_state->mode.vdisplay)
		return -EINVAL;

	ret = drm_atomic_helper_check_wb_connector_state(connector, state);
	if (ret < 0)
		return ret;

	return 0;
}

static int meson_wb_get_modes(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;

	return drm_add_modes_noedid(connector, dev->mode_config.max_width,
				    dev->mode_config.max_height);
}

static int meson_wb_prepare_job(struct drm_writeback_connector *wb_connector,
				struct drm_writeback_job *job)
{
	struct drm_gem_dma_object *gem;

	if (!job->fb)
		return 0;

	gem = drm_fb_dma_get_gem_obj(job->fb, 0);
	if (!gem)
		return -EINVAL;

	/* Store DMA address for use in commit */
	job->priv = (void *)(uintptr_t)gem->dma_addr;

	return 0;
}

static void meson_wb_cleanup_job(struct drm_writeback_connector *wb_connector,
				 struct drm_writeback_job *job)
{
	/* Nothing to clean up — we used the DMA address directly */
}

static void meson_wb_atomic_commit(struct drm_connector *connector,
				   struct drm_atomic_state *state)
{
	struct drm_connector_state *conn_state =
		drm_atomic_get_new_connector_state(state, connector);
	struct meson_drm *priv = connector->dev->dev_private;
	struct drm_writeback_connector *wb_conn =
		container_of(connector, struct drm_writeback_connector, base);
	struct drm_framebuffer *fb = conn_state->writeback_job->fb;
	dma_addr_t dst_addr = (dma_addr_t)(uintptr_t)conn_state->writeback_job->priv;
	struct meson_ge2d_job job;
	int ret;

	/* Lazy GE2D acquisition — module may have loaded since init */
	if (!priv->ge2d) {
		priv->ge2d = meson_ge2d_get(priv->dev);
		if (IS_ERR(priv->ge2d)) {
			priv->ge2d = NULL;
			drm_writeback_signal_completion(wb_conn, -ENODEV);
			return;
		}
	}

	/* Source: primary plane's current scanout buffer */
	job.src_addr = priv->viu.osd1_addr;
	job.src_w = fb->width;
	job.src_h = fb->height;
	job.src_stride = priv->viu.osd1_stride;
	job.src_fmt = GE2D_FMT_XRGB8888;

	/* Destination: writeback framebuffer */
	job.dst_addr = dst_addr;
	job.dst_w = fb->width;
	job.dst_h = fb->height;
	job.dst_stride = fb->pitches[0];
	job.dst_fmt = (fb->format->format == DRM_FORMAT_ARGB8888) ?
		       GE2D_FMT_ARGB8888 : GE2D_FMT_XRGB8888;

	drm_writeback_queue_job(wb_conn, conn_state);
	ret = meson_ge2d_blit(priv->ge2d, &job);

	drm_writeback_signal_completion(wb_conn, ret);
}

static const struct drm_connector_helper_funcs meson_wb_conn_helper_funcs = {
	.get_modes = meson_wb_get_modes,
	.prepare_writeback_job = meson_wb_prepare_job,
	.cleanup_writeback_job = meson_wb_cleanup_job,
	.atomic_commit = meson_wb_atomic_commit,
	.atomic_check = meson_wb_atomic_check,
};

int meson_writeback_create(struct meson_drm *priv)
{
	struct drm_writeback_connector *wb;
	int ret;

	/* GE2D acquired lazily on first writeback job — module may load later */
	priv->ge2d = NULL;

	wb = devm_kzalloc(priv->dev, sizeof(*wb), GFP_KERNEL);
	if (!wb)
		return -ENOMEM;

	drm_connector_helper_add(&wb->base, &meson_wb_conn_helper_funcs);

	ret = drm_writeback_connector_init(priv->drm, wb,
					   &meson_wb_connector_funcs,
					   NULL,
					   meson_wb_formats,
					   ARRAY_SIZE(meson_wb_formats),
					   1);
	if (ret)
		return ret;

	/*
	 * The writeback encoder must be cloneable with all other encoders
	 * on the same CRTC, otherwise drm_atomic_helper_check_modeset()
	 * rejects the commit. Set possible_clones on all encoders sharing
	 * this CRTC to include each other, including each encoder's own bit.
	 */
	{
		struct drm_encoder *enc;
		u32 wb_mask = drm_encoder_mask(&wb->encoder);

		wb->encoder.possible_clones = 0;
		drm_for_each_encoder(enc, priv->drm) {
			if (enc->possible_crtcs & wb->encoder.possible_crtcs) {
				wb->encoder.possible_clones |=
					drm_encoder_mask(enc);
				enc->possible_clones |=
					drm_encoder_mask(enc) | wb_mask;
			}
		}
	}

	priv->writeback = wb;

	return 0;
}
