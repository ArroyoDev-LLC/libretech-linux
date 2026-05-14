/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 */

#ifndef __MESON_VDEC_DOS_REGS_H_
#define __MESON_VDEC_DOS_REGS_H_

/* DOS registers */
#define VDEC_ASSIST_MMC_CTRL1	0x0008
#define VDEC_ASSIST_CANVAS_BLK32 0x0014
	#define CANVAS_BLK32_WR		BIT(11)
	#define CANVAS_BLK32_BIT	10
	#define CANVAS_INDEX_WR		BIT(8)

#define VDEC_ASSIST_AMR1_INT8	0x00b4

#define ASSIST_MBOX1_CLR_REG	0x01d4
#define ASSIST_MBOX1_MASK	0x01d8

#define MPSR			0x0c04
/*
 * MBY_MBX (MB_MOTION_MODE): current macroblock decode position.
 * BSP vmh264.c:652 defines MBY_MBX = MB_MOTION_MODE = 0x0c07.
 * Byte address = 0x0c07 << 2 = 0x301c.
 * Used by the host-side decode stall watchdog to detect progress.
 */
#define MBY_MBX			0x301c
#define MCPU_INTR_MSK		0x0c10
#define CPSR			0x0c84

#define IMEM_DMA_CTRL		0x0d00
#define IMEM_DMA_ADR		0x0d04
#define IMEM_DMA_COUNT		0x0d08
#define LMEM_DMA_CTRL		0x0d40
/* LMEM DMA siblings of LMEM_DMA_CTRL.
 * Required by codec_h264_libre.c pre_queue() LMEM DMA sequence.
 */
#define LMEM_DMA_ADR		0x0d44
#define LMEM_DMA_COUNT		0x0d48
/*
 * WRRSP_LMEM (CR 0x0353 << 2 = 0x0d4c): write-response register for the
 * LMEM_DMA engine on VDEC_1.  Read-only from the host.  Mirrors the
 * pending-write count for outstanding LMEM transactions; non-zero after
 * an LMEM_DMA pulse means the engine has not yet drained.  Used by the
 * vdec_diag.c lmem_dma_test characterization harness to record the
 * post-DMA value for correlating CTRL encodings against engine state.
 * HEVC has its own equivalent at HEVC_WRRSP_LMEM = 0xcd4c.
 */
#define WRRSP_LMEM		0x0d4c

#define MC_STATUS0		0x2424
#define MC_CTRL1		0x242c

#define PSCALE_RST		0x2440
	#define PSCALE_RST_ALL		0x7
#define PSCALE_CTRL		0x2444
	/* PSCALE_CTRL: bit 15 = enable, bit 14 = CBUS BMEM access */
	#define PSCALE_CTRL_EN_BMEM	0xc000
#define PSCALE_BMEM_ADDR	0x247c
#define PSCALE_BMEM_DAT		0x2480

#define DBLK_CTRL		0x2544
#define DBLK_STATUS		0x254c

#define MDEC_SW_RESET		0x2610
#define GCLK_EN			0x260c
#define MDEC_PIC_DC_MUX_CTRL	0x2634
#define MDEC_PIC_DC_CTRL	0x2638
#define MDEC_PIC_DC_STATUS	0x263c
#define ANC0_CANVAS_ADDR	0x2640
#define ANC1_CANVAS_ADDR	0x2644	/* 0x0991 << 2 */
#define ANC2_CANVAS_ADDR	0x2648	/* 0x0992 << 2 */
#define DBKR_CANVAS_ADDR	0x26c0	/* 0x09b0 << 2 */
#define DBKW_CANVAS_ADDR	0x26c4	/* 0x09b1 << 2 */
#define REC_CANVAS_ADDR		0x26c8	/* 0x09b2 << 2 */
#define MDEC_PIC_DC_THRESH	0x26e0

#define MDEC_EXTIF_CFG0		0x2790
#define MDEC_EXTIF_CFG1		0x2794
#define MDEC_EXTIF_CFG2		0x2798

/* Firmware interface registers */
#define AV_SCRATCH_0		0x2700
#define AV_SCRATCH_1		0x2704
#define AV_SCRATCH_2		0x2708
#define AV_SCRATCH_3		0x270c
#define AV_SCRATCH_4		0x2710
#define AV_SCRATCH_5		0x2714
#define AV_SCRATCH_6		0x2718
#define AV_SCRATCH_7		0x271c
#define AV_SCRATCH_8		0x2720
#define AV_SCRATCH_9		0x2724
#define AV_SCRATCH_A		0x2728
#define AV_SCRATCH_B		0x272c
#define AV_SCRATCH_C		0x2730
#define AV_SCRATCH_D		0x2734
#define AV_SCRATCH_E		0x2738
#define AV_SCRATCH_F		0x273c
#define AV_SCRATCH_G		0x2740
#define AV_SCRATCH_H		0x2744
#define AV_SCRATCH_I		0x2748
#define AV_SCRATCH_J		0x274c
#define AV_SCRATCH_K		0x2750
#define AV_SCRATCH_L		0x2754

/*
 * DOS_SCRATCH_31: GXL-family capability register written by U-Boot.
 * Bit 3: H.264 resolution cap for S805X -- firmware enforces
 *   (mb_width-1)+(mb_height-1) <= 188 when set, limiting decode to ~1080p.
 * CPU byte offset = AMRISC CR 0x3f2f * 4 = 0xFCBC.
 * BSP: dos_registers_v1.h DOS_SCRATCH31 = 0x3f2f.
 */
#define DOS_SCRATCH_31		0xFCBC
#define   DOS_SCRATCH_31_H264_1080P_CAP	BIT(3)

#define VLD_STATUS_CTRL		0x3000	/* 0x0c00 << 2 */
#define MPEG1_2_REG		0x3004
#define PIC_HEAD_INFO		0x300c
	/*
	 * PIC_HEAD_INFO init value: bits [9:7] = 0b111.
	 * BSP sets this to skip certain header parsing checks
	 * during firmware initialization.
	 */
	#define PIC_HEAD_INFO_INIT	0x380
#define POWER_CTL_VLD		0x3020
	#define POWER_CTL_VLD_ENABLE	BIT(4)
#define SLICE_QP		0x3028	/* 0x0c0a << 2 */
#define RESYNC_MARKER_LENGTH	0x3038	/* 0x0c0e << 2 */
#define MB_INFO			0x30b0	/* 0x0c2c << 2 */
#define M4_CONTROL_REG		0x30a4

/* Stream Buffer (stbuf) regs */
#define VLD_MEM_VIFIFO_START_PTR	0x3100
#define VLD_MEM_VIFIFO_CURR_PTR	0x3104
#define VLD_MEM_VIFIFO_END_PTR	0x3108
#define VLD_MEM_VIFIFO_CONTROL	0x3110
	#define MEM_FIFO_CNT_BIT	16
	#define MEM_FILL_ON_LEVEL	BIT(10)
	#define MEM_CTRL_EMPTY_EN	BIT(2)
	#define MEM_CTRL_FILL_EN	BIT(1)
#define VLD_MEM_VIFIFO_WP	0x3114
#define VLD_MEM_VIFIFO_RP	0x3118
#define VLD_MEM_VIFIFO_LEVEL	0x311c
#define VLD_MEM_VIFIFO_BUF_CNTL	0x3120
	#define MEM_BUFCTRL_MANUAL	BIT(1)
#define VLD_MEM_VIFIFO_WRAP_COUNT	0x3144

#define VCOP_CTRL_REG		0x3800	/* 0x0e00 << 2 */
#define DC_AC_CTRL		0x3814	/* 0x0e05 << 2 */
#define RV_AI_MB_COUNT		0x3830	/* 0x0e0c << 2 */
#define IQIDCT_CONTROL		0x3838
#define DCAC_DMA_CTRL		0x3848
#define DCAC_DDR_BYTE64_CTL	0x3874

#define DOS_SW_RESET0		0xfc00
	/* DOS_SW_RESET0 bits */
	#define DOS_SW_RESET0_VLD	BIT(4)
	#define DOS_SW_RESET0_MDEC	(BIT(7) | BIT(6))
	#define DOS_SW_RESET0_MC	(BIT(9) | BIT(8))
	#define DOS_SW_RESET0_PSCALE	BIT(10)
	#define DOS_SW_RESET0_VDEC1	(BIT(12) | BIT(11))
#define DOS_GCLK_EN0		0xfc04
	/* DOS_GCLK_EN0: enable all VDEC1 clock gates */
	#define DOS_GCLK_EN0_VDEC1	0x3ff
#define DOS_GEN_CTRL0		0xfc08
	/* DOS_GEN_CTRL0: vififo_vbuf_rp_sel field (bits 2:1) */
	#define DOS_GEN_CTRL0_VDEC1	0
	#define DOS_GEN_CTRL0_HEVC	(3 << 1)
#define DOS_MEM_PD_VDEC		0xfcc0
#define DOS_MEM_PD_HEVC		0xfccc
#define DOS_SW_RESET3		0xfcd0
	/* DOS_SW_RESET3 bits (HEVC domain) */
	#define DOS_SW_RESET3_HEVC_PARTIAL	(BIT(12) | BIT(11))
#define DOS_GCLK_EN3		0xfcd4
#define DOS_VDEC_MCRCC_STALL_CTRL	0xfd00

/* IMEM DMA control bits (shared by VDEC_1 and HEVC IMEM DMA) */
#define IMEM_DMA_CTRL_START	BIT(15)
#define IMEM_DMA_CTRL_ENDIAN_SHIFT	16
/* SWAP64 endian: full 64-bit byte reversal (swap16 | swap32 | swap64) */
#define IMEM_DMA_CTRL_ENDIAN_SWAP64	(7 << IMEM_DMA_CTRL_ENDIAN_SHIFT)
#define IMEM_DMA_CTRL_RUN	(IMEM_DMA_CTRL_START | IMEM_DMA_CTRL_ENDIAN_SWAP64)

/* VDEC_1 engine status bits */
#define MC_STATUS0_BUSY		BIT(0)
#define MC_CTRL1_RESET		(BIT(0) | BIT(3))
#define DBLK_STATUS_BUSY	BIT(0)
#define DBLK_CTRL_RESET		3

/* MDEC_PIC_DC_CTRL bits */
#define MDEC_PIC_DC_CTRL_OLD_MODE	BIT(31)
#define MDEC_PIC_DC_CTRL_NV12		BIT(17)
#define MDEC_PIC_DC_CTRL_CBCR_SWAP	BIT(16)

/* MDEC_SW_RESET bits */
#define MDEC_SW_RESET_MDEC_CORE		BIT(7)

#endif
