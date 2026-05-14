// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 Maxime Jourdan <mjourdan@baylibre.com>
 */

#include <linux/dma-mapping.h>
#include <linux/export.h>
#include <linux/gfp.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "codec_hevc_common.h"
#include "vdec_helpers.h"
#include "hevc_regs.h"

/* MMU_COMPRESS_HEADER_SIZE + MMU_MAP_SIZE moved to codec_hevc_common.h
 * so codec_hevc.c's pre-alloc path uses the same constants.  MMU map
 * holds one u32 PFN per FBC body page.  Sized for 8K@8-bit
 * (8192x4320 ~= 14K pages); previous 0x4800 (4608 entries, 18 KB)
 * overflowed at 6K (8128 pages) and produced "MMU map overflow"
 * errors during the cma-opt-sg validation.  64 KB covers up to
 * 16384 pages = roughly 9K@8-bit, with headroom for 10-bit framings.
 */

/*
 * Scatter-gather FBC body allocation.  When true, FBC body pages are
 * allocated individually via alloc_page + dma_map_page instead of a
 * single contiguous dma_alloc_coherent per capture buffer.  The HEVC
 * MMU translates via mmu_map so the hardware doesn't require
 * contiguity.  Removes the contiguous-CMA bottleneck that blocks 6K+
 * on fragmented / small-CMA boards (VP9-006).  See cma-opt-sg-plan.md.
 *
 * Default ON post Engagement B cross-platform validation (GXL VP9-006
 * sequential regression + G12B/SM1 5K/6K/8K matrix all pass).  Kept
 * as a module param so an operator can flip back to contiguous via
 * meson_vdec_core.fbc_sg=N for forensics if a regression appears.
 * Engagement C will remove the param + legacy contig path entirely.
 */
bool codec_hevc_fbc_sg = true;
module_param_named(fbc_sg, codec_hevc_fbc_sg, bool, 0644);
MODULE_PARM_DESC(fbc_sg,
	"Use scatter-gather alloc for FBC body (cma-opt-sg; default on)");
EXPORT_SYMBOL_GPL(codec_hevc_fbc_sg);

const u16 vdec_hevc_parser_cmd[] = {
	0x0401,	0x8401,	0x0800,	0x0402,
	0x9002,	0x1423,	0x8CC3,	0x1423,
	0x8804,	0x9825,	0x0800,	0x04FE,
	0x8406,	0x8411,	0x1800,	0x8408,
	0x8409,	0x8C2A,	0x9C2B,	0x1C00,
	0x840F,	0x8407,	0x8000,	0x8408,
	0x2000,	0xA800,	0x8410,	0x04DE,
	0x840C,	0x840D,	0xAC00,	0xA000,
	0x08C0,	0x08E0,	0xA40E,	0xFC00,
	0x7C00
};
EXPORT_SYMBOL_GPL(vdec_hevc_parser_cmd);

/* Configure decode head read mode */
void codec_hevc_setup_decode_head(struct amvdec_session *sess, int is_10bit)
{
	struct amvdec_core *core = sess->core;
	u32 width_64 = ALIGN(sess->width, 64) / 64;
	u32 height_32 = ALIGN(sess->height, 32) / 32;
	/* BSP mem_saving_mode: 8-bit FBC uses 3200 bytes per 64x32 block,
	 * 10-bit uses 4096.  SAO_CTRL5 BIT(9) enables mem_saving_mode for
	 * 8-bit, so BODY_LENGTH must match the compressed block size the
	 * hardware actually writes.  Buffers are allocated at 4096/block
	 * (the maximum), so 8-bit wastes ~20% but is always large enough.
	 */
	u32 body_size = (is_10bit ? 4096 : 3200) * width_64 * height_32;
	u32 head_size = amvdec_am21c_head_size(sess->width, sess->height);
	int use_mmu = codec_hevc_use_mmu(core->platform->revision,
					 sess->pixfmt_cap, is_10bit);

	if (!use_mmu && !codec_hevc_use_fbc(sess->pixfmt_cap, is_10bit)) {
		/* Enable 2-plane reference read mode */
		amvdec_write_dos(core, HEVCD_MPP_DECOMP_CTL1, BIT(31));
		return;
	}

	if (use_mmu) {
		/* MMU mode: compressed decode via page-table translation */
		amvdec_write_dos(core, HEVCD_MPP_DECOMP_CTL1, BIT(4));
		amvdec_write_dos(core, HEVCD_MPP_DECOMP_CTL2, 0);
		/* Enable MMU in SAO (BSP init_decode_head_hw) */
		amvdec_write_dos_bits(core, HEVC_SAO_CTRL9, BIT(0));
	} else {
		amvdec_write_dos(core, HEVCD_MPP_DECOMP_CTL1, 0);
		if (core->platform->revision < VDEC_REVISION_SM1)
			amvdec_write_dos(core, HEVCD_MPP_DECOMP_CTL2,
					 body_size / 32);
	}

	amvdec_write_dos(core, HEVC_CM_BODY_LENGTH, body_size);
	amvdec_write_dos(core, HEVC_CM_HEADER_OFFSET, body_size);
	amvdec_write_dos(core, HEVC_CM_HEADER_LENGTH, head_size);
}
EXPORT_SYMBOL_GPL(codec_hevc_setup_decode_head);

static void codec_hevc_setup_buffers_gxbb(struct amvdec_session *sess,
					  struct codec_hevc_common *comm,
					  int is_10bit)
{
	struct amvdec_core *core = sess->core;
	struct v4l2_m2m_buffer *buf;
	u32 buf_num = v4l2_m2m_num_dst_bufs_ready(sess->m2m_ctx);
	dma_addr_t buf_y_paddr = 0;
	dma_addr_t buf_uv_paddr = 0;
	u32 idx = 0;
	u32 val;
	int i;

	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, 0);

	v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
		struct vb2_buffer *vb = &buf->vb.vb2_buf;

		idx = vb->index;

		if (codec_hevc_use_downsample(sess->pixfmt_cap, is_10bit))
			buf_y_paddr = comm->fbc_buffer_paddr[idx];
		else
			buf_y_paddr = vb2_dma_contig_plane_dma_addr(vb, 0);

		if (codec_hevc_use_fbc(sess->pixfmt_cap, is_10bit)) {
			val = buf_y_paddr | (idx << 8) | 1;
			amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CMD_ADDR,
					 val);
		} else {
			if (vb->num_planes > 1)
				buf_uv_paddr = vb2_dma_contig_plane_dma_addr(vb, 1);
			else
				buf_uv_paddr = buf_y_paddr + amvdec_get_output_size(sess);
			val = buf_y_paddr | ((idx * 2) << 8) | 1;
			amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CMD_ADDR,
					 val);
			val = buf_uv_paddr | ((idx * 2 + 1) << 8) | 1;
			amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CMD_ADDR,
					 val);
		}
	}

	if (codec_hevc_use_fbc(sess->pixfmt_cap, is_10bit))
		val = buf_y_paddr | (idx << 8) | 1;
	else
		val = buf_y_paddr | ((idx * 2) << 8) | 1;

	/* Fill the remaining unused slots with the last buffer's Y addr */
	for (i = buf_num; i < MAX_REF_PIC_NUM; ++i)
		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CMD_ADDR, val);

	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, 1);
	amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1);
	for (i = 0; i < 32; ++i)
		amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, 0);
}

static void codec_hevc_setup_buffers_gxl(struct amvdec_session *sess,
					 struct codec_hevc_common *comm,
					 int is_10bit)
{
	struct amvdec_core *core = sess->core;
	struct vb2_queue *q = v4l2_m2m_get_dst_vq(sess->m2m_ctx);
	u32 revision = core->platform->revision;
	u32 pixfmt_cap = sess->pixfmt_cap;
	int i;

	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR,
			 ANC2AXI_CONF_AUTO_INC);

	/* Iterate over ALL allocated capture buffers, not just queued ones.
	 * The firmware references frames by their ANC2AXI table index,
	 * which must correspond to the vb2 buffer index. If we only fill
	 * entries for queued buffers, reference frames that have been
	 * dequeued to userspace will have wrong ANC2AXI entries, causing
	 * inter-frame reference corruption (P-frames referencing wrong data).
	 */
	for (i = 0; i < vb2_get_num_buffers(q); ++i) {
		struct vb2_buffer *vb = vb2_get_buffer(q, i);
		dma_addr_t buf_y_paddr = 0;
		dma_addr_t buf_uv_paddr = 0;

		if (!vb)
			continue;

		if (codec_hevc_use_mmu(revision, pixfmt_cap, is_10bit))
			buf_y_paddr = comm->mmu_header_paddr[i];
		else if (codec_hevc_use_downsample(pixfmt_cap, is_10bit))
			buf_y_paddr = comm->fbc_buffer_paddr[i];
		else
			buf_y_paddr = vb2_dma_contig_plane_dma_addr(vb, 0);

		amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
				 buf_y_paddr >> 5);

		if (!codec_hevc_use_fbc(pixfmt_cap, is_10bit) &&
		    !codec_hevc_use_mmu(revision, pixfmt_cap, is_10bit)) {
			if (vb->num_planes > 1)
				buf_uv_paddr = vb2_dma_contig_plane_dma_addr(vb, 1);
			else
				buf_uv_paddr = buf_y_paddr + amvdec_get_output_size(sess);
			amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_DATA,
					 buf_uv_paddr >> 5);
		}
	}

	amvdec_write_dos(core, HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR, 1);
	amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR, 1);
	for (i = 0; i < 32; ++i)
		amvdec_write_dos(core, HEVCD_MPP_ANC_CANVAS_DATA_ADDR, 0);
}

/*
 * Free one scatter-gather FBC body buffer: dma_unmap + free each page,
 * then release the bookkeeping arrays.  Idempotent via the NULL checks
 * so it's safe to call on partially-allocated state during unwind.
 */
static void codec_hevc_free_fbc_sg_one(struct device *dev,
				       struct codec_hevc_fbc_sg *sg)
{
	u32 i;

	if (!sg->pages)
		return;

	for (i = 0; i < sg->nb_pages; ++i) {
		if (sg->page_paddr && sg->page_paddr[i])
			dma_unmap_page(dev, sg->page_paddr[i], PAGE_SIZE,
				       DMA_BIDIRECTIONAL);
		if (sg->pages[i])
			__free_page(sg->pages[i]);
	}
	kfree(sg->page_paddr);
	kfree(sg->pages);
	sg->page_paddr = NULL;
	sg->pages = NULL;
	sg->nb_pages = 0;
}

static int codec_hevc_alloc_fbc_sg_one(struct device *dev,
				       struct codec_hevc_fbc_sg *sg,
				       u32 nb_pages)
{
	u32 i;

	sg->pages = kcalloc(nb_pages, sizeof(*sg->pages), GFP_KERNEL);
	if (!sg->pages)
		return -ENOMEM;

	sg->page_paddr = kcalloc(nb_pages, sizeof(*sg->page_paddr),
				 GFP_KERNEL);
	if (!sg->page_paddr) {
		kfree(sg->pages);
		sg->pages = NULL;
		return -ENOMEM;
	}

	for (i = 0; i < nb_pages; ++i) {
		sg->pages[i] = alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (!sg->pages[i])
			goto unwind;

		sg->page_paddr[i] = dma_map_page(dev, sg->pages[i], 0,
						 PAGE_SIZE,
						 DMA_BIDIRECTIONAL);
		if (dma_mapping_error(dev, sg->page_paddr[i])) {
			__free_page(sg->pages[i]);
			sg->pages[i] = NULL;
			sg->page_paddr[i] = 0;
			goto unwind;
		}
	}
	sg->nb_pages = nb_pages;
	return 0;

unwind:
	sg->nb_pages = i;  /* free what we did allocate */
	codec_hevc_free_fbc_sg_one(dev, sg);
	return -ENOMEM;
}

void codec_hevc_free_fbc_buffers(struct amvdec_session *sess,
				 struct codec_hevc_common *comm)
{
	struct device *dev = sess->core->dev;
	u32 am21_size = amvdec_am21c_size(sess->width, sess->height,
					  comm->is_10bit);
	int i;

	/* Free contig path buffers (NULL when sg mode was used). */
	for (i = 0; i < MAX_REF_PIC_NUM; ++i) {
		if (comm->fbc_buffer_vaddr[i]) {
			dma_free_coherent(dev, am21_size,
					  comm->fbc_buffer_vaddr[i],
					  comm->fbc_buffer_paddr[i]);
			comm->fbc_buffer_vaddr[i] = NULL;
		}
	}

	/* Free sg path buffers (NULL when contig mode was used). */
	for (i = 0; i < MAX_REF_PIC_NUM; ++i)
		codec_hevc_free_fbc_sg_one(dev, &comm->fbc_sg[i]);
}
EXPORT_SYMBOL_GPL(codec_hevc_free_fbc_buffers);

static int codec_hevc_alloc_fbc_buffers(struct amvdec_session *sess,
					struct codec_hevc_common *comm)
{
	struct device *dev = sess->core->dev;
	struct v4l2_m2m_buffer *buf;
	u32 am21_size = amvdec_am21c_size(sess->width, sess->height,
					  comm->is_10bit);

	/* FBC-output capture: the vb2 capture buffer IS the FBC body
	 * (sized at queue_setup to amvdec_am21c_size).  Driver doesn't
	 * allocate its own FBC storage; codec_hevc_fill_mmu_map and
	 * codec_vp9_set_sao both pick up the vb2 dma_addr directly
	 * through the non-NV12 branches they already have.
	 */
	if (codec_hevc_is_fbc_output(sess->pixfmt_cap))
		return 0;

	/* Already allocated (setup_buffers is called from both
	 * resume and setup_decode on every frame).  Check either
	 * storage since they're mutually exclusive by param.
	 */
	if (comm->fbc_buffer_vaddr[0] || comm->fbc_sg[0].pages)
		return 0;

	if (codec_hevc_fbc_sg) {
		/* Scatter-gather: per-page alloc_page + dma_map_page.
		 * Eliminates the contiguous-CMA requirement.  See
		 * cma-opt-sg-plan.md.
		 */
		u32 nb_pages = DIV_ROUND_UP(am21_size, PAGE_SIZE);
		int ret;

		v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
			u32 idx = buf->vb.vb2_buf.index;

			ret = codec_hevc_alloc_fbc_sg_one(dev,
							  &comm->fbc_sg[idx],
							  nb_pages);
			if (ret) {
				codec_hevc_free_fbc_buffers(sess, comm);
				return ret;
			}
		}
		return 0;
	}

	/* Contiguous legacy path. */
	v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
		u32 idx = buf->vb.vb2_buf.index;
		dma_addr_t paddr;
		void *vaddr = dma_alloc_coherent(dev, am21_size, &paddr,
						 GFP_KERNEL);
		if (!vaddr) {
			codec_hevc_free_fbc_buffers(sess, comm);
			return -ENOMEM;
		}

		comm->fbc_buffer_vaddr[idx] = vaddr;
		comm->fbc_buffer_paddr[idx] = paddr;
	}

	return 0;
}

void codec_hevc_free_mmu_headers(struct amvdec_session *sess,
				 struct codec_hevc_common *comm)
{
	struct device *dev = sess->core->dev;
	int i;

	for (i = 0; i < MAX_REF_PIC_NUM; ++i) {
		if (comm->mmu_header_vaddr[i]) {
			dma_free_coherent(dev, MMU_COMPRESS_HEADER_SIZE,
					  comm->mmu_header_vaddr[i],
					  comm->mmu_header_paddr[i]);
			comm->mmu_header_vaddr[i] = NULL;
		}
	}

	if (comm->mmu_map_vaddr) {
		dma_free_coherent(dev, MMU_MAP_SIZE,
				  comm->mmu_map_vaddr,
				  comm->mmu_map_paddr);
		comm->mmu_map_vaddr = NULL;
	}
}
EXPORT_SYMBOL_GPL(codec_hevc_free_mmu_headers);

static int codec_hevc_alloc_mmu_headers(struct amvdec_session *sess,
					struct codec_hevc_common *comm)
{
	struct device *dev = sess->core->dev;
	struct v4l2_m2m_buffer *buf;

	/* Already allocated (setup_buffers is called from both
	 * resume and setup_decode on every frame).  Must NOT
	 * reallocate -- the hardware writes FBC translation data
	 * into the per-frame MMU headers during decode.  If we
	 * allocate new headers, the ANC2AXI table points to fresh
	 * (zeroed) headers instead of the ones populated by
	 * previous frame decodes, causing reference corruption.
	 * Check mmu_header_vaddr[0] (not mmu_map_vaddr) because
	 * the MMU map may be pre-allocated in codec_hevc_start()
	 * for GXL before capture buffers exist.
	 */
	if (comm->mmu_header_vaddr[0])
		return 0;

	if (!comm->mmu_map_vaddr) {
		comm->mmu_map_vaddr = dma_alloc_coherent(dev, MMU_MAP_SIZE,
							 &comm->mmu_map_paddr,
							 GFP_KERNEL);
		if (!comm->mmu_map_vaddr)
			return -ENOMEM;
	}

	v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
		u32 idx = buf->vb.vb2_buf.index;
		dma_addr_t paddr;
		void *vaddr = dma_alloc_coherent(dev, MMU_COMPRESS_HEADER_SIZE,
						 &paddr, GFP_KERNEL);
		if (!vaddr) {
			codec_hevc_free_mmu_headers(sess, comm);
			return -ENOMEM;
		}

		comm->mmu_header_vaddr[idx] = vaddr;
		comm->mmu_header_paddr[idx] = paddr;
	}

	return 0;
}

int codec_hevc_setup_buffers(struct amvdec_session *sess,
			     struct codec_hevc_common *comm,
			     int is_10bit)
{
	struct amvdec_core *core = sess->core;
	int ret;

	/* Remember bit depth for free/fill paths (see comm->is_10bit) */
	comm->is_10bit = !!is_10bit;

	if (codec_hevc_use_downsample(sess->pixfmt_cap, is_10bit) ||
	    codec_hevc_use_mmu(core->platform->revision,
			       sess->pixfmt_cap, is_10bit)) {
		ret = codec_hevc_alloc_fbc_buffers(sess, comm);
		if (ret)
			return ret;
	}

	if (codec_hevc_use_mmu(core->platform->revision,
			       sess->pixfmt_cap, is_10bit)) {
		ret = codec_hevc_alloc_mmu_headers(sess, comm);
		if (ret) {
			codec_hevc_free_fbc_buffers(sess, comm);
			return ret;
		}
	}

	if (core->platform->revision == VDEC_REVISION_GXBB)
		codec_hevc_setup_buffers_gxbb(sess, comm, is_10bit);
	else
		codec_hevc_setup_buffers_gxl(sess, comm, is_10bit);

	return 0;
}
EXPORT_SYMBOL_GPL(codec_hevc_setup_buffers);

void codec_hevc_fill_mmu_map(struct amvdec_session *sess,
			     struct codec_hevc_common *comm,
			     struct vb2_buffer *vb)
{
	u32 size = amvdec_am21c_size(sess->width, sess->height,
				     comm->is_10bit);
	u32 nb_pages = size / PAGE_SIZE;
	u32 max_pages = MMU_MAP_SIZE / sizeof(u32);
	u32 *mmu_map = comm->mmu_map_vaddr;
	struct codec_hevc_fbc_sg *sg = &comm->fbc_sg[vb->index];
	u32 first_page;
	u32 i;

	if (!mmu_map) {
		dev_err(sess->core->dev,
			"MMU map not allocated, skipping fill\n");
		return;
	}

	if (nb_pages > max_pages) {
		dev_err(sess->core->dev,
			"MMU map overflow: %u pages > %u max (res %ux%u)\n",
			nb_pages, max_pages, sess->width, sess->height);
		nb_pages = max_pages;
	}

	/*
	 * Three cases:
	 * 1. NV12 + sg allocator: write scattered PFNs from fbc_sg.
	 *    HEVC MMU translates each FBC virtual page through mmu_map
	 *    to the real (non-contiguous) physical page.
	 * 2. NV12 + contig allocator: write sequential PFNs starting at
	 *    fbc_buffer_paddr (legacy behaviour, pre-cma-opt-sg).
	 * 3. Non-NV12: FBC storage is the vb2 output buffer itself
	 *    (multi-plane capture buffer has contiguous DMA addr).
	 *    sg allocator doesn't apply here -- vb2 owns that memory.
	 */
	if (codec_hevc_is_nv12(sess->pixfmt_cap) && sg->pages) {
		u32 sg_pages = min(nb_pages, sg->nb_pages);

		dev_dbg(sess->core->dev,
			"MMU map sg: idx=%d nb_pages=%u sg_nb=%u map_paddr=0x%08llx\n",
			vb->index, nb_pages, sg->nb_pages,
			(u64)comm->mmu_map_paddr);

		for (i = 0; i < sg_pages; ++i)
			mmu_map[i] = sg->page_paddr[i] >> PAGE_SHIFT;
		return;
	}

	if (codec_hevc_is_nv12(sess->pixfmt_cap))
		first_page = comm->fbc_buffer_paddr[vb->index] >> PAGE_SHIFT;
	else
		first_page = vb2_dma_contig_plane_dma_addr(vb, 0) >> PAGE_SHIFT;

	dev_dbg(sess->core->dev,
		"MMU map: idx=%d first_page=0x%x nb_pages=%u fbc_paddr=0x%08llx map_paddr=0x%08llx\n",
		 vb->index, first_page, nb_pages,
		 (u64)comm->fbc_buffer_paddr[vb->index],
		 (u64)comm->mmu_map_paddr);

	for (i = 0; i < nb_pages; ++i)
		mmu_map[i] = first_page + i;
}
EXPORT_SYMBOL_GPL(codec_hevc_fill_mmu_map);
