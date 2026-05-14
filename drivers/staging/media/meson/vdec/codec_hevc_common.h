/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#ifndef __MESON_VDEC_HEVC_COMMON_H_
#define __MESON_VDEC_HEVC_COMMON_H_

#include "vdec.h"

/* Parser command skip configuration (BSP literal values).
 * These configure the parser's NAL unit skip/search behavior.
 * CFG_0: skip count thresholds, CFG_1/2: skip pattern masks.
 */
#define PARSER_CMD_SKIP_CFG_0 0x0000090b
#define PARSER_CMD_SKIP_CFG_1 0x1b14140f
#define PARSER_CMD_SKIP_CFG_2 0x001b1910

#define VDEC_HEVC_PARSER_CMD_LEN 37
extern const u16 vdec_hevc_parser_cmd[VDEC_HEVC_PARSER_CMD_LEN];

/* cma-opt-sg module param; set via /sys/module/meson_vdec_core/parameters/fbc_sg.
 * When true, FBC body allocations go through the scatter-gather path
 * (alloc_page + dma_map_page) instead of dma_alloc_coherent.  The HEVC
 * MMU translates via mmu_map so the contiguous-CMA budget no longer
 * constrains the probe-time max-resolution clamp.
 */
extern bool codec_hevc_fbc_sg;

#define MAX_REF_PIC_NUM	24

/* Per-frame MMU header buffer size (one allocation per reference frame). */
#define MMU_COMPRESS_HEADER_SIZE 0x48000

/* Session-wide MMU map buffer size.  ALL allocators / freers of
 * codec_hevc_common::mmu_map_vaddr MUST use this size.  The pre-alloc
 * in codec_hevc_start (so the firmware can read SCRATCH_7 at init)
 * and the lazy alloc in codec_hevc_alloc_mmu_headers (after
 * SOURCE_CHANGE) write to the SAME pointer; mismatched sizes corrupt
 * adjacent CMA page refcounts (KI-107).
 */
#define MMU_MAP_SIZE 0x10000

/*
 * Scatter-gather FBC body descriptor.  When codec_hevc_fbc_sg module
 * param is enabled, FBC body pages are allocated individually via
 * alloc_page + dma_map_page instead of a single dma_alloc_coherent.
 * The HEVC MMU translates device-visible FBC virtual pages to physical
 * pages via the mmu_map table (codec_hevc_fill_mmu_map writes per-page
 * PFNs).  This removes the contiguous-CMA requirement and unlocks
 * higher resolutions on fragmented / small-CMA boards.
 *
 * Mutually exclusive with fbc_buffer_vaddr/paddr: sg mode leaves
 * those NULL/0, contig mode leaves pages[]/page_paddr[] NULL/0.
 * See cma-opt-sg-plan.md for the full design.
 */
struct codec_hevc_fbc_sg {
	struct page **pages;
	dma_addr_t  *page_paddr;
	u32	     nb_pages;
};

struct codec_hevc_common {
	void      *fbc_buffer_vaddr[MAX_REF_PIC_NUM];
	dma_addr_t fbc_buffer_paddr[MAX_REF_PIC_NUM];

	/* Scatter-gather FBC body allocation (codec_hevc_fbc_sg param
	 * enabled).  Mutually exclusive with fbc_buffer_vaddr[] above.
	 */
	struct codec_hevc_fbc_sg fbc_sg[MAX_REF_PIC_NUM];

	void      *mmu_header_vaddr[MAX_REF_PIC_NUM];
	dma_addr_t mmu_header_paddr[MAX_REF_PIC_NUM];

	void      *mmu_map_vaddr;
	dma_addr_t mmu_map_paddr;

	/*
	 * Mirror of the is_10bit argument to codec_hevc_setup_buffers.
	 * Stored so free_fbc_buffers and fill_mmu_map can compute the
	 * correct FBC body size (3200 B/block 8-bit, 4096 B/block 10-bit)
	 * for dma_free_coherent and MMU page-count without plumbing
	 * is_10bit through every call site.  Set on alloc.
	 */
	bool is_10bit;

	/* BSP-protocol DMA buffers (Bucket 3 / hevc_bsp path).
	 * Allocated only when sess->fw_match->quirks &
	 * AMVDEC_FW_QUIRK_HEVC_BSP_PROTO.  Upstream-firmware sessions
	 * leave these fields NULL/0.  Accessed by codec_hevc_bsp.c via
	 * struct codec_hevc_common * passed in the session hooks.
	 */
	void      *bsp_rpm_vaddr;
	dma_addr_t bsp_rpm_paddr;
	void      *bsp_aux_vaddr;
	dma_addr_t bsp_aux_paddr;
};

/* Returns true if the capture fourcc is one of the AMLOGIC FBC
 * compressed-output formats.  In that mode the vb2 capture buffer
 * holds the FBC body directly (no SAO double-write to NV12).
 */
static inline bool codec_hevc_is_fbc_output(u32 pixfmt)
{
	return pixfmt == V4L2_PIX_FMT_MESON_FBC08_SCATTER ||
	       pixfmt == V4L2_PIX_FMT_MESON_FBC10_SCATTER;
}

/* Returns 1 if the decode pipeline uses framebuffer compression.
 * On G12A+ the HEVC engine always uses the compressed decode path
 * internally; the SAO double-write produces NV12 output alongside.
 * On GXBB/GXL, FBC is only used for 10-bit content.
 * When capture is one of the FBC-output fourccs, FBC is always used
 * regardless of bit depth (that's the whole point of the format).
 */
static inline int codec_hevc_use_fbc(u32 pixfmt, int is_10bit)
{
	return is_10bit || codec_hevc_is_fbc_output(pixfmt);
}

/* Returns 1 if we are decoding 10-bit but outputting 8-bit NV12.
 * FBC-output capture passes the compressed data through without
 * downsampling; the consumer (VPU VD1) or a separate downsample
 * block handles 10->8 if needed.
 */
static inline int codec_hevc_use_downsample(u32 pixfmt, int is_10bit)
{
	return is_10bit && !codec_hevc_is_fbc_output(pixfmt);
}

/* Returns 1 if we are decoding using the IOMMU.
 * The HEVC engine always requires MMU for its compressed decode
 * pipeline on all platforms (GXL through SM1+).  The BSP always
 * sets mmu_enable=1 for HEVC, even on GXL (vh265.c line 16456).
 * VP9 always uses FBC/MMU for reference frames.
 */
static inline int codec_hevc_use_mmu(u32 revision, u32 pixfmt, int is_10bit)
{
	return revision >= VDEC_REVISION_GXL;
}

/* Returns true if pixfmt uses NV12/NV21 layout (Y + interleaved UV/VU) */
static inline bool codec_hevc_is_nv12(u32 pixfmt)
{
	return pixfmt == V4L2_PIX_FMT_NV12M || pixfmt == V4L2_PIX_FMT_NV12 ||
	       pixfmt == V4L2_PIX_FMT_NV21M || pixfmt == V4L2_PIX_FMT_NV21;
}

/* Returns true if pixfmt needs UV swap (NV21 = VU byte order) */
static inline bool codec_hevc_is_nv21(u32 pixfmt)
{
	return pixfmt == V4L2_PIX_FMT_NV21M || pixfmt == V4L2_PIX_FMT_NV21;
}

/* Configure decode head read mode */
void codec_hevc_setup_decode_head(struct amvdec_session *sess, int is_10bit);

void codec_hevc_free_fbc_buffers(struct amvdec_session *sess,
				 struct codec_hevc_common *comm);

void codec_hevc_free_mmu_headers(struct amvdec_session *sess,
				 struct codec_hevc_common *comm);

int codec_hevc_setup_buffers(struct amvdec_session *sess,
			     struct codec_hevc_common *comm,
			     int is_10bit);

void codec_hevc_fill_mmu_map(struct amvdec_session *sess,
			     struct codec_hevc_common *comm,
			     struct vb2_buffer *vb);

#endif
