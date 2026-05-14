// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Maxime Jourdan <mjourdan@baylibre.com>
 *
 * VDEC_1 is a video decoding block that allows decoding of
 * MPEG 1/2/4, H.263, H.264, MJPEG, VC1
 */

#include <linux/firmware.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/workqueue.h>

#include "vdec_1.h"
#include "vdec_hevc.h"
#include "vdec_helpers.h"
#include "codec_avs.h"
#include "dos_regs.h"

/* AO Registers */
#define AO_RTI_GEN_PWR_SLEEP0	0xe8
	/* SLEEP0 bit definitions (power on/off) */
	#define GEN_PWR_VDEC_1 (BIT(3) | BIT(2))
	#define GEN_PWR_VDEC_1_SM1 (BIT(1))
	#define GEN_PWR_VDEC_HEVC (BIT(7) | BIT(6))
	#define GEN_PWR_VDEC_HEVC_SM1 (BIT(2))
#define AO_RTI_GEN_PWR_ISO0	0xec
	/*
	 * ISO0 bit definitions (isolation control).
	 * NOTE: ISO0 uses DIFFERENT bit positions from SLEEP0 on non-SM1!
	 * HEVC isolation is at bits 10-11 in ISO0 (0xc00) but bits 6-7 in
	 * SLEEP0 (0xc0).  VDEC_1 isolation is at bits 6-7 in ISO0 (0xc0)
	 * but bits 2-3 in SLEEP0 (0x0c).  SM1 uses the same bits for both.
	 */
	#define GEN_PWR_ISO0_HEVC (BIT(11) | BIT(10))       /* 0xc00 */
	#define GEN_PWR_ISO0_VDEC_1 (BIT(7) | BIT(6))       /* 0xc0 */
	#define GEN_PWR_ISO0_HEVC_SM1 (BIT(2))
	#define GEN_PWR_ISO0_VDEC_1_SM1 (BIT(1))

#define MC_SIZE			(4096 * 4)

/*
 * VDEC_1 decoder turbo clock -- DISABLED.
 *
 * The VDEC_1 hardware block cannot operate above 800 MHz.  At 1 GHz
 * via gp0_pll, the H.264 decoder stalls at MBY_MBX=0 on every frame
 * (tested on G12B A311D, 2026-04-20).  The HCODEC encoder block on
 * the same SoC runs fine at 1 GHz, so this is a per-IP timing limit.
 *
 * The module param is retained (read-only) so users see the parameter
 * exists but is locked at 0.  Removing it would break scripts that
 * set all three turbo params in one tee command.
 */
static unsigned int vdec1_turbo;
module_param(vdec1_turbo, uint, 0444);
MODULE_PARM_DESC(vdec1_turbo,
		 "VDEC_1 decoder turbo: DISABLED (HW limit, 800MHz max)");

static const unsigned long vdec1_turbo_rates[] = {
	800000000,	/* 0: 800 MHz max for VDEC_1 decode HW */
};

#define VDEC1_TURBO_MAX (ARRAY_SIZE(vdec1_turbo_rates) - 1)

/*
 * vdec1_target_rate - effective VDEC_1 clock rate
 *
 * Returns turbo_rate when not thermally throttled (state 0), otherwise
 * returns the thermal rate.  Thermal ALWAYS wins over turbo for safety.
 */
static unsigned long vdec1_target_rate(struct amvdec_core *core)
{
	unsigned long thermal_rate = amvdec_clock_rate(core);
	unsigned int level = min_t(unsigned int, vdec1_turbo, VDEC1_TURBO_MAX);
	unsigned long turbo_rate = vdec1_turbo_rates[level];

	if (thermal_rate < 800000000)
		return thermal_rate;
	return turbo_rate;
}

/*
 * VDEC_1 idle autosuspend.
 *
 * Instead of powering off immediately at STREAMOFF, schedule a delayed
 * poweroff.  If another STREAMON arrives before the delay expires, cancel
 * the work and skip the power cycle entirely.
 *
 * Default idle timeout: 100ms. Settable via module param for tuning.
 */
static unsigned int vdec1_idle_ms = 100;
module_param(vdec1_idle_ms, uint, 0644);
MODULE_PARM_DESC(vdec1_idle_ms,
		 "VDEC_1 idle poweroff delay in ms (default=100, 0=immediate)");

static int
vdec_1_load_firmware(struct amvdec_session *sess, const char *fwname)
{
	const struct firmware *fw;
	struct amvdec_core *core = sess->core;
	struct device *dev = core->dev_dec;
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;
	const struct amvdec_fw_version *match;
	static void *mc_addr;
	static dma_addr_t mc_addr_map;
	int ret;
	u32 i = 1000;

	ret = request_firmware(&fw, fwname, dev);
	if (ret < 0)
		return -EINVAL;

	mc_addr = dma_alloc_coherent(core->dev, MC_SIZE,
				     &mc_addr_map, GFP_KERNEL);
	if (!mc_addr) {
		ret = -ENOMEM;
		goto release_firmware;
	}

	/*
	 * Copy the IMEM portion of the firmware (up to MC_SIZE bytes) and
	 * zero-fill any remainder.  Firmware sizes vary:
	 *   - SM1 AVS: 12 KiB (smaller than MC_SIZE) -- zero-padded
	 *   - MPEG12/MPEG4: 16 KiB (exactly MC_SIZE)
	 *   - H.264: 36 KiB (larger than MC_SIZE) -- only first 16 KiB is
	 *     IMEM; the rest is extended firmware handled by
	 *     load_extended_firmware() below
	 */
	memset(mc_addr, 0, MC_SIZE);
	memcpy(mc_addr, fw->data, min_t(size_t, fw->size, MC_SIZE));

	/*
	 * Identify the firmware binary by SHA256 and store the match on the
	 * session.  Must be done AFTER memcpy (we hash fw->data) and BEFORE
	 * any firmware-specific byte patches so that quirk-gated patches can
	 * inspect sess->fw_match.
	 *
	 * amvdec_fw_identify() returns ERR_PTR(-ENOENT) when a non-empty
	 * table was provided but no entry matched -- propagate as load error.
	 * NULL means no table (unwired codec); session continues normally.
	 */
	match = amvdec_fw_identify(dev, fwname,
				   fw->data, fw->size,
				   sess->fmt_out->fw_versions,
				   sess->fmt_out->fw_versions_count);
	if (IS_ERR(match)) {
		ret = PTR_ERR(match);
		goto free_mc;
	}
	sess->fw_match = match;  /* NULL if codec is unwired, or valid entry */

	/*
	 * Patch gxl_mpeg12.bin firmware bug: the sequence_end handler at
	 * address 0x02ef uses "bgti r9, 3" to check if the current canvas
	 * slot needs flushing.  This skips slots 4-7, causing the last
	 * decoded frame to be silently dropped when it lands in those slots.
	 * Fix: change to "bgti r9, 7" (byte 0xBBD: 0x80 -> 0x81).
	 *
	 * Guard: check firmware name, size, and the 4-byte instruction
	 * at the bug site (c9 80 e3 0a at offset 0xBBC) to avoid patching
	 * a fixed or different firmware version. A patched firmware would
	 * have 0x81 at 0xBBD instead of 0x80 and would not match.
	 */
	if (strstr(fwname, "mpeg12") &&
	    fw->size == MC_SIZE &&
	    ((u8 *)mc_addr)[0xBBC] == 0xc9 &&
	    ((u8 *)mc_addr)[0xBBD] == 0x80 &&
	    ((u8 *)mc_addr)[0xBBE] == 0xe3 &&
	    ((u8 *)mc_addr)[0xBBF] == 0x0a) {
		((u8 *)mc_addr)[0xBBD] = 0x81;
		dev_info(dev, "Patched MPEG12 firmware canvas flush bug\n");
	}

	/*
	 * Patch gxl_mpeg12.bin BUFFEROUT race on G12A+: the firmware's
	 * picture-done handler (address 0x038f) reads BUFFEROUT and, if
	 * non-zero, gives up and returns to the main loop without
	 * signaling the current decoded frame.  On GXL (28nm), the ARM
	 * core's IRQ handler clears BUFFEROUT fast enough that the
	 * firmware rarely sees it non-zero.  On G12A/SM1 (12nm), the
	 * AMRISC decodes faster than the ARM can service IRQs, causing
	 * 30-70% frame drops.
	 *
	 * Fix: replace the single-check-and-give-up at 0x0390-0x0393
	 * with a spin-wait loop, matching the firmware's own EOS flush
	 * path at 0x0300-0x0301 which already spin-waits for BUFFEROUT=0.
	 *
	 * Original (0x0390-0x0393):
	 *   beqi  r9, 0, +4     # if BUFFEROUT==0, skip ahead
	 *   li    r7, 2          # set state = 2
	 *   b     -864           # goto main loop (give up)
	 *   sta   r7, [0x7]      # (delay slot) store state
	 *
	 * Patched:
	 *   bneqi r9, 0, -1     # if BUFFEROUT!=0, goto 0x038f (re-read)
	 *   nop                  # (delay slot: safe)
	 *   nop
	 *   nop
	 *
	 * Guard: verify the 4 instruction words at 0x0390-0x0393 match
	 * the known unpatched firmware.  Apply only on G12A+ where the
	 * race is observed.
	 */
	if (strstr(fwname, "mpeg12") &&
	    fw->size == MC_SIZE &&
	    core->platform->revision >= VDEC_REVISION_G12A &&
	    ((u8 *)mc_addr)[0xE40] == 0x09 &&
	    ((u8 *)mc_addr)[0xE41] == 0x00 &&
	    ((u8 *)mc_addr)[0xE42] == 0xa1 &&
	    ((u8 *)mc_addr)[0xE43] == 0x0a &&
	    ((u8 *)mc_addr)[0xE44] == 0x87 &&
	    ((u8 *)mc_addr)[0xE45] == 0x00 &&
	    ((u8 *)mc_addr)[0xE46] == 0x80 &&
	    ((u8 *)mc_addr)[0xE47] == 0x06) {
		/* 0x0390: beqi r9,0,+4 -> bneqi r9,0,-1 */
		((u8 *)mc_addr)[0xE41] = 0xC0;
		((u8 *)mc_addr)[0xE42] = 0x6F;
		/* 0x0391-0x0393: NOP out the give-up path */
		memset((u8 *)mc_addr + 0xE44, 0, 12);
		dev_info(dev,
			 "Patched MPEG12 firmware BUFFEROUT race (G12A+)\n");
	}


	/*
	 * Patch g12a_h264.bin CMD_FRAMES_READY race on G12A+ (KI-091).
	 *
	 * The frame output loop at address 0x19de-0x19f7 reads SRAM[0x1c7]
	 * (pending frame counter) and skips CMD_FRAMES_READY when it reads 0.
	 * On 12nm parts the AMRISC reads stale 0 from SRAM, skipping the
	 * completion signal.  Fix: spin-wait instead of single-check-and-skip.
	 *
	 * Original: blt r13, r12, +12   (0x0b80c30d at byte 0x67AC)
	 * Patched:  blt r13, r12, -6    (0x0b8fa30d at byte 0x67AC)
	 */
	/*
	 * The H.264 patch site is at byte 0x67AC of the original 36-KiB BSP
	 * firmware, but mc_addr is only MC_SIZE = 0x4000 bytes (the IMEM
	 * window).  Reading mc_addr[0x67AC] is an out-of-bounds access that
	 * faults if the surrounding allocation page boundary doesn't include
	 * the addressed offset (observed on SM1 with libre H.264 stateless
	 * 2026-04-25: kernel oops Internal error 0x96000007 -- pgd lookup
	 * fault at 0x67AC offset, vdec_1_start+0x694).
	 *
	 * For the patch to be functional it needs mc_addr sized to the full
	 * firmware (the BSP H.264 firmware is 36 KiB; the libre H.264
	 * stateless firmware is exactly MC_SIZE = 16 KiB).  Until the
	 * allocation is resized, the patch is dead code AND a crash hazard:
	 * skip it here and add a trace if a 36-KiB firmware ever shows up.
	 */
	if (strstr(fwname, "h264") && !strstr(fwname, "mvc") &&
	    fw->size > 0x67B0 &&
	    core->platform->revision >= VDEC_REVISION_G12A) {
		dev_warn(dev,
			 "firmware patch H.264 CMD_FRAMES_READY race at 0x19eb (G12A+, KI-091) NOT applied: mc_addr is MC_SIZE=%u bytes, patch site needs offset 0x67AC; would need a larger mc_addr alloc to reactivate\n",
			 (unsigned int)MC_SIZE);
	}
	amvdec_write_dos(core, MPSR, 0);
	amvdec_write_dos(core, CPSR, 0);

	amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL, MDEC_PIC_DC_CTRL_OLD_MODE);

	amvdec_write_dos(core, IMEM_DMA_ADR, mc_addr_map);
	amvdec_write_dos(core, IMEM_DMA_COUNT, MC_SIZE / 4);
	amvdec_write_dos(core, IMEM_DMA_CTRL, IMEM_DMA_CTRL_RUN);

	while (--i && amvdec_read_dos(core, IMEM_DMA_CTRL) & IMEM_DMA_CTRL_START);

	if (i == 0) {
		dev_err(dev, "Firmware load fail (DMA hang?)\n");
		ret = -EINVAL;
		goto free_mc;
	}

	/*
	 * Patch AVS firmware: NOP the VLD_STATUS_CTRL busy check.
	 *
	 * The AVS firmware main loop at address 0x07dc checks
	 * VLD_STATUS_CTRL bits[1:0] and exits if non-zero (VLD busy).
	 * When the esparser feeds data before firmware start (V4L2 M2M
	 * flow), the VLD has already processed VIFIFO data and bit 0 is
	 * permanently set.  The firmware sees VLD busy and never decodes.
	 *
	 * Fix: NOP the 3-instruction check-and-exit sequence at 0x07dc-07de
	 * so the firmware always falls through to the VIFIFO data check.
	 *
	 * Original (byte 0x1F70): bnandi r3,3,+4; li r2,0; b +36
	 * Patched:                nop; nop; nop
	 */
	if (sess->fw_match &&
	    (sess->fw_match->quirks & AMVDEC_FW_QUIRK_AVS_VLD_BUSY_PATCH)) {
		/*
		 * gxl_avs.bin main loop at AMRISC 0x07dc polls VLD_STATUS_CTRL
		 * bits[1:0] and exits if non-zero.  In the V4L2 M2M flow the
		 * esparser feeds data before firmware start so bit 0 is latched.
		 * NOP the 3-instruction check so the loop falls through.
		 *
		 * The guard is now the firmware hash -- only gxl_avs.bin has the
		 * bug, identified by SHA256.  Preserved byte assertion left for
		 * extra safety in case a hash collision ever lines up.
		 */
		if (((u8 *)mc_addr)[0x1F70] == 0xc3 &&
		    ((u8 *)mc_addr)[0x1F71] == 0x00 &&
		    ((u8 *)mc_addr)[0x1F72] == 0x21 &&
		    ((u8 *)mc_addr)[0x1F73] == 0x0a) {
			memset((u8 *)mc_addr + 0x1F70, 0, 12);
			dev_info(dev,
				 "firmware workaround: patched AVS VLD busy check at 0x07dc (hash-matched gxl_avs.bin)\n");
		} else {
			dev_warn(dev,
				 "AVS VLD busy patch: quirk set but bytes don't match; firmware may already be patched or is a different variant\n");
		}
	}

	/*
	 * Patch MPEG4 firmware: NOP the VLD_STATUS_CTRL busy check in the
	 * main polling loop at AMRISC address 0x06b1.
	 *
	 *   06b0: mfcr r2, VLD_STATUS_CTRL
	 *   06b1: bandi r2, 1, +5        <- branch if bit 0 set -> 06b6
	 *   06b2: blti  r37, 64, +7      <- fall-through to frame process
	 *
	 * On GXL (28nm), the VLD hardware latches "data available" bit 0
	 * when the esparser feeds data before MPSR=1.  The firmware poll
	 * loop sees bit 0 set, branches to 0x06b6 (reset counter, return),
	 * and never enters frame processing.  NOP the check so the loop
	 * always falls through to the counter test at 0x06b2.
	 *
	 * On G12A+ (12nm), this patch is NOT needed and breaks decode:
	 * the VLD busy check at 0x06b1 is part of the firmware's normal
	 * flow control.  G12B MPEG4 was 28/30 frames without this patch
	 * (baseline 98e927c0126d).  Applying it unconditionally caused
	 * 0/0/0 IRQs on G12B (verified 2026-04-20 on 7a31c9b5cc0b).
	 *
	 * Guard: GXL-only + firmware name contains "mpeg4" + 4 guard
	 * bytes at byte offset 0x1AC4 match the known unpatched
	 * instruction 0x0be14042 (LE: 42 40 e1 0b).
	 */
	if (core->platform->revision < VDEC_REVISION_G12A &&
	    strstr(fwname, "mpeg4") && fw->size >= MC_SIZE &&
	    ((u8 *)mc_addr)[0x1AC4] == 0x42 &&
	    ((u8 *)mc_addr)[0x1AC5] == 0x40 &&
	    ((u8 *)mc_addr)[0x1AC6] == 0xe1 &&
	    ((u8 *)mc_addr)[0x1AC7] == 0x0b) {
		memset((u8 *)mc_addr + 0x1AC4, 0, 4);
		dev_info(dev,
			 "Patched MPEG4 firmware VLD busy check (NOP at 0x06b1, GXL only)\n");
	}

	if (codec_ops->load_extended_firmware)
		ret = codec_ops->load_extended_firmware(sess,
							fw->data, fw->size,
							MC_SIZE);

free_mc:
	dma_free_coherent(core->dev, MC_SIZE, mc_addr, mc_addr_map);
release_firmware:
	release_firmware(fw);
	return ret;
}

static int vdec_1_stbuf_power_up(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	amvdec_write_dos(core, VLD_MEM_VIFIFO_CONTROL, 0);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_WRAP_COUNT, 0);
	amvdec_write_dos(core, POWER_CTL_VLD, POWER_CTL_VLD_ENABLE);

	amvdec_write_dos(core, VLD_MEM_VIFIFO_START_PTR, sess->vififo_paddr);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_CURR_PTR, sess->vififo_paddr);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_END_PTR,
			 sess->vififo_paddr + sess->vififo_size - 8);

	amvdec_write_dos_bits(core, VLD_MEM_VIFIFO_CONTROL, 1);
	amvdec_clear_dos_bits(core, VLD_MEM_VIFIFO_CONTROL, 1);

	amvdec_write_dos(core, VLD_MEM_VIFIFO_BUF_CNTL, MEM_BUFCTRL_MANUAL);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_WP, sess->vififo_paddr);

	amvdec_write_dos_bits(core, VLD_MEM_VIFIFO_BUF_CNTL, 1);
	amvdec_clear_dos_bits(core, VLD_MEM_VIFIFO_BUF_CNTL, 1);

	/* FIFO count thresholds: bits 23:16=fill, bits 19:16=empty (0x11)
	 * These are hardware FIFO level thresholds for DMA refill logic.
	 */
	amvdec_write_dos_bits(core, VLD_MEM_VIFIFO_CONTROL,
			      (0x11 << MEM_FIFO_CNT_BIT) | MEM_FILL_ON_LEVEL |
			      MEM_CTRL_FILL_EN | MEM_CTRL_EMPTY_EN);

	return 0;
}

static void vdec_1_conf_esparser(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	/* VDEC_1 specific ESPARSER stuff */
	amvdec_write_dos(core, DOS_GEN_CTRL0, DOS_GEN_CTRL0_VDEC1);
	amvdec_write_dos(core, VLD_MEM_VIFIFO_BUF_CNTL, 1);
	amvdec_clear_dos_bits(core, VLD_MEM_VIFIFO_BUF_CNTL, 1);
}

static u32 vdec_1_vififo_level(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;

	return amvdec_read_dos(core, VLD_MEM_VIFIFO_LEVEL);
}

static void vdec1_clk_enable(struct amvdec_core *core)
{
	unsigned long target = vdec1_target_rate(core);

	/*
	 * If gp0_pll is available and target > 800 MHz, switch the mux
	 * parent to gp0_pll.  GXL has no gp0_pll in its VDEC_1 mux
	 * (only fclk_div3/4/5/7), so this path is skipped on GXL.
	 * The clk_set_parent call targets the MUX (vdec_sel), not the
	 * gate.  Walk: gate -> div -> mux.
	 */
	if (core->gp0_pll && target > 800000000) {
		clk_set_rate(core->gp0_pll, target);
		clk_prepare_enable(core->gp0_pll);

		struct clk *div = clk_get_parent(core->vdec_1_clk);
		struct clk *mux = div ? clk_get_parent(div) : NULL;

		if (mux)
			clk_set_parent(mux, core->gp0_pll);
	}

	clk_set_rate(core->vdec_1_clk, target);
	clk_prepare_enable(core->vdec_1_clk);
}

static int vdec_1_poweron(struct amvdec_core *core)
{
	int ret;

	vdec1_clk_enable(core);

	/*
	 * On G12A+, the MDEC DC cache hardware is inside the HEVC power
	 * domain. VDEC_1 needs HEVC clocks active for correct decode.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A) {
		ret = vdec_hevc_clk_get(core);
		if (ret) {
			clk_disable_unprepare(core->vdec_1_clk);
			return ret;
		}
	}

	/* Power on VDEC_1 (and HEVC on G12A+ for MDEC DC cache) */
	if (core->platform->revision == VDEC_REVISION_SM1)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
				   GEN_PWR_VDEC_1_SM1 |
				   GEN_PWR_VDEC_HEVC_SM1, 0);
	else if (core->platform->revision >= VDEC_REVISION_G12A)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
				   GEN_PWR_VDEC_1 |
				   GEN_PWR_VDEC_HEVC, 0);
	else
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
				   GEN_PWR_VDEC_1, 0);
	usleep_range(10, 20);

	/* Reset VDEC1 */
	amvdec_write_dos(core, DOS_SW_RESET0, 0xfffffffc);
	amvdec_write_dos(core, DOS_SW_RESET0, 0x00000000);

	amvdec_write_dos(core, DOS_GCLK_EN0, DOS_GCLK_EN0_VDEC1);

	/* Enable VDEC Memories */
	amvdec_write_dos(core, DOS_MEM_PD_VDEC, 0);

	/* On G12A+, also power on HEVC memories and enable clock gating */
	if (core->platform->revision >= VDEC_REVISION_G12A) {
		amvdec_write_dos(core, DOS_SW_RESET3, 0xffffffff);
		amvdec_write_dos(core, DOS_SW_RESET3, 0x00000000);
		amvdec_write_dos(core, DOS_GCLK_EN3, 0xffffffff);
		amvdec_write_dos(core, DOS_MEM_PD_HEVC, 0);
	}

	/* Remove isolation */
	if (core->platform->revision == VDEC_REVISION_SM1)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
				   GEN_PWR_ISO0_VDEC_1_SM1 |
				   GEN_PWR_ISO0_HEVC_SM1, 0);
	else if (core->platform->revision >= VDEC_REVISION_G12A)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
				   GEN_PWR_ISO0_HEVC | GEN_PWR_ISO0_VDEC_1, 0);
	else
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
				   GEN_PWR_ISO0_VDEC_1, 0);
	usleep_range(100, 200);

	return 0;
}

void vdec_1_poweroff(struct amvdec_core *core)
{
	/*
	 * Power-off order: clock off BEFORE isolation.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A)
		vdec_hevc_clk_put(core);
	clk_disable_unprepare(core->vdec_1_clk);

	/* Enable vdec1 isolation */
	if (core->platform->revision == VDEC_REVISION_SM1)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
				   GEN_PWR_ISO0_VDEC_1_SM1,
				   GEN_PWR_ISO0_VDEC_1_SM1);
	else
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
				   GEN_PWR_ISO0_VDEC_1, GEN_PWR_ISO0_VDEC_1);

	/* Power off vdec1 memories */
	amvdec_write_dos(core, DOS_MEM_PD_VDEC, 0xffffffff);

	/* Power off vdec1 */
	if (core->platform->revision == VDEC_REVISION_SM1)
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
				   GEN_PWR_VDEC_1_SM1, GEN_PWR_VDEC_1_SM1);
	else
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
				   GEN_PWR_VDEC_1, GEN_PWR_VDEC_1);
}

static void vdec1_poweroff_work_fn(struct work_struct *work)
{
	struct amvdec_core *core = container_of(work, struct amvdec_core,
						vdec1_poweroff_work.work);

	mutex_lock(&core->hw_lock);
	if (core->vdec1_powered) {
		vdec_1_poweroff(core);
		core->vdec1_powered = false;
	}
	mutex_unlock(&core->hw_lock);
}

void vdec_1_power_init(struct amvdec_core *core)
{
	INIT_DELAYED_WORK(&core->vdec1_poweroff_work, vdec1_poweroff_work_fn);
	core->vdec1_powered = false;
}

/*
 * vdec_1_power_get - ensure VDEC_1 is powered on.
 * Cancels any pending idle poweroff and powers on if needed.
 * Self-locking: caller must NOT hold hw_lock.
 */
int vdec_1_power_get(struct amvdec_core *core)
{
	cancel_delayed_work_sync(&core->vdec1_poweroff_work);
	mutex_lock(&core->hw_lock);
	if (!core->vdec1_powered) {
		int ret = vdec_1_poweron(core);

		if (ret) {
			mutex_unlock(&core->hw_lock);
			return ret;
		}
		core->vdec1_powered = true;
	}
	mutex_unlock(&core->hw_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(vdec_1_power_get);

/*
 * vdec_1_power_put - schedule delayed VDEC_1 poweroff.
 * If vdec1_idle_ms is 0, powers off immediately under hw_lock.
 * Otherwise schedules delayed_work (which self-locks).
 * Caller must NOT hold hw_lock.
 */
void vdec_1_power_put(struct amvdec_core *core)
{
	if (vdec1_idle_ms == 0) {
		mutex_lock(&core->hw_lock);
		vdec_1_poweroff(core);
		core->vdec1_powered = false;
		mutex_unlock(&core->hw_lock);
	} else {
		schedule_delayed_work(&core->vdec1_poweroff_work,
				      msecs_to_jiffies(vdec1_idle_ms));
	}
}
EXPORT_SYMBOL_GPL(vdec_1_power_put);

static void __vdec_1_stop(struct amvdec_session *sess)
{
	struct amvdec_core *core = sess->core;
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;
	unsigned long timeout;

	amvdec_write_dos(core, MPSR, 0);
	amvdec_write_dos(core, CPSR, 0);
	amvdec_write_dos(core, ASSIST_MBOX1_MASK, 0);

	/*
	 * Wait for MDEC, DBLK, and MC engines to idle before resetting.
	 * BSP (amvdec_pg_enable) polls each engine's status register and
	 * manually resets on timeout.  Without this, cutting power while
	 * engines are active can latch bad state that persists into the
	 * next power-on, causing ~30% of back-to-back decodes to fail
	 * with 0 output frames.
	 */

	/* Wait for MDEC PIC DC to idle */
	timeout = jiffies + msecs_to_jiffies(10);
	while (amvdec_read_dos(core, MDEC_PIC_DC_STATUS) != 0) {
		if (time_after(jiffies, timeout)) {
			/* Force reset MDEC PIC DC */
			amvdec_write_dos_bits(core, MDEC_PIC_DC_CTRL, BIT(0));
			amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL, BIT(0));
			amvdec_read_dos(core, MDEC_PIC_DC_STATUS);
			break;
		}
		cpu_relax();
	}

	/* Wait for DBLK to idle */
	timeout = jiffies + msecs_to_jiffies(10);
	while (amvdec_read_dos(core, DBLK_STATUS) & DBLK_STATUS_BUSY) {
		if (time_after(jiffies, timeout)) {
			/* Force reset DBLK */
			amvdec_write_dos(core, DBLK_CTRL, DBLK_CTRL_RESET);
			amvdec_write_dos(core, DBLK_CTRL, 0);
			amvdec_read_dos(core, DBLK_STATUS);
			break;
		}
		cpu_relax();
	}

	/* Wait for MC to idle */
	timeout = jiffies + msecs_to_jiffies(10);
	while (amvdec_read_dos(core, MC_STATUS0) & MC_STATUS0_BUSY) {
		if (time_after(jiffies, timeout)) {
			/* Force reset MC */
			amvdec_write_dos_bits(core, MC_CTRL1, MC_CTRL1_RESET);
			amvdec_clear_dos_bits(core, MC_CTRL1, MC_CTRL1_RESET);
			amvdec_read_dos(core, MC_STATUS0);
			break;
		}
		cpu_relax();
	}

	amvdec_write_dos(core, DOS_SW_RESET0, DOS_SW_RESET0_VDEC1);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);

	if (sess->priv)
		codec_ops->stop(sess);

	/* Schedule deferred power-off via autosuspend */
	vdec_1_power_put(core);
}

static int vdec_1_stop(struct amvdec_session *sess)
{
	__vdec_1_stop(sess);
	return 0;
}

static int vdec_1_start(struct amvdec_session *sess)
{
	int ret;
	struct amvdec_core *core = sess->core;
	struct amvdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;

	/*
	 * Power on VDEC_1 domain via autosuspend-aware API.
	 * vdec_1_power_get() cancels any pending idle poweroff and
	 * powers on if needed.  Uses vdec1_target_rate() for the clock
	 * so turbo level is applied at poweron time.
	 */
	ret = vdec_1_power_get(core);
	if (ret)
		return ret;
	/* Reset DOS top registers */
	amvdec_write_dos(core, DOS_VDEC_MCRCC_STALL_CTRL, 0);

	amvdec_write_dos(core, GCLK_EN, DOS_GCLK_EN0_VDEC1);
	amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL, MDEC_PIC_DC_CTRL_OLD_MODE);

	/*
	 * Clear IMEM and MDEC state before firmware load.
	 *
	 * BIT(4) = VLD reset, which clears IMEM contents.  Without this,
	 * firmware from a previous codec session persists in IMEM through
	 * the power cycle (DOS_MEM_PD_VDEC off/on above does not reliably
	 * zero SRAM on all SoCs).  Stale firmware causes codec cross-
	 * contamination: e.g., MPEG12 firmware executing during H.264
	 * decode when sessions run back-to-back without reboot.
	 *
	 * On G12A+, also reset MDEC subsystem (BIT(7)|BIT(6)) to clear
	 * DC prediction pipeline stale state (reduces +42 Y-plane offset).
	 *
	 * This must happen BEFORE firmware load -- the DMA load that
	 * follows immediately overwrites the cleared IMEM.
	 */
	if (core->platform->revision >= VDEC_REVISION_G12A) {
		amvdec_write_dos(core, DOS_SW_RESET0,
				 DOS_SW_RESET0_MDEC | DOS_SW_RESET0_VLD);
		amvdec_write_dos(core, DOS_SW_RESET0, 0);
		amvdec_read_dos(core, DOS_SW_RESET0);
		amvdec_read_dos(core, DOS_SW_RESET0);
		amvdec_read_dos(core, DOS_SW_RESET0);
		amvdec_write_dos(core, MDEC_SW_RESET, MDEC_SW_RESET_MDEC_CORE);
		amvdec_write_dos(core, MDEC_SW_RESET, 0);
	} else {
		/* GXL/GXM: clear VLD/IMEM only (no MDEC reset needed) */
		amvdec_write_dos(core, DOS_SW_RESET0, DOS_SW_RESET0_VLD);
		amvdec_write_dos(core, DOS_SW_RESET0, 0);
		amvdec_read_dos(core, DOS_SW_RESET0);
	}

	/* Disable PSCALE -- MJPEG enables it (PSCALE_CTRL=0xc000) and
	 * leaves it active after stop.  Other codecs (MPEG1/2, MPEG4)
	 * disable it in their start(), but H.264 does not.  Clear it
	 * here so every codec starts with PSCALE disabled regardless
	 * of what the previous codec left behind.
	 */
	amvdec_write_dos(core, PSCALE_CTRL, 0);

	/* Fill canvas registers with scratch buffer before firmware starts */
	amvdec_fill_scratch_canvases(core);

	vdec_1_stbuf_power_up(sess);

	ret = vdec_1_load_firmware(sess, sess->fmt_out->firmware_path);
	if (ret)
		goto stop;

	ret = codec_ops->start(sess);
	if (ret)
		goto stop;

	/* Enable 2-plane output and configure chroma order.
	 * Bit 17: 2-plane enable (NV12/NV21 mode)
	 * Bit 16: cbcr_merge_swap_en (1=NV21/VU, 0=NV12/UV)
	 */
	switch (sess->pixfmt_cap) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV12M:
		amvdec_write_dos_bits(core, MDEC_PIC_DC_CTRL,
				      MDEC_PIC_DC_CTRL_NV12);
		amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL,
				      MDEC_PIC_DC_CTRL_CBCR_SWAP);
		break;
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV21M:
		amvdec_write_dos_bits(core, MDEC_PIC_DC_CTRL,
				      MDEC_PIC_DC_CTRL_NV12 |
				      MDEC_PIC_DC_CTRL_CBCR_SWAP);
		break;
	default:
		amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL,
				      MDEC_PIC_DC_CTRL_NV12 |
				      MDEC_PIC_DC_CTRL_CBCR_SWAP);
		break;
	}

	if (codec_ops->early_irq_enable) {
		/*
		 * Enable MBOX1 BEFORE starting firmware for codecs that decode
		 * fast enough to fire their first-frame IRQ before the host can
		 * enable interrupts via the normal MPSR->CLR->MASK sequence.
		 *
		 * MJPEG firmware: after MPSR=1 the firmware immediately enters
		 * its main loop and can decode frame 0 within ~15-60 us.  With
		 * the normal sequence (MPSR -> usleep -> CLR -> MASK) the first
		 * MBOX1 interrupt is reliably lost, dropping 1 frame per session.
		 *
		 * Safety: codec_mjpeg_start() clears MREG_FROM_AMRISC = 0.
		 * Any stale MBOX1 pending from a prior session fires harmlessly
		 * (ISR sees reg=0 and returns without delivering a frame).
		 * MJPEG firmware does NOT fire spurious init IRQs; it busy-waits
		 * for MREG_TO_AMRISC != 0 before doing any decode.
		 *
		 * MPEG12 must NOT use this path: it fires MCPU_INTR_REQ=4 at
		 * firmware address 0x052f during init when VIFIFO has data,
		 * causing spurious MBOX1 delivery with stale AV_SCRATCH_9.
		 */
		amvdec_write_dos(core, ASSIST_MBOX1_MASK, 1);
		amvdec_write_dos(core, MPSR, 1);
		usleep_range(10, 20);
	} else {
		/* Enable firmware processor */
		amvdec_write_dos(core, MPSR, 1);
		/* Let the firmware settle */
		usleep_range(10, 20);

		/*
		 * Enable IRQ after firmware has initialized.  The firmware's
		 * init sequence (setting up interrupt routing, canvas addresses,
		 * etc.) can generate a spurious MBOX1 interrupt.  If the IRQ is
		 * enabled before the firmware settles, the ISR reads stale
		 * BUFFEROUT/FRAME_OFFSET values and delivers a bogus frame,
		 * consuming a timestamp entry.  This shifts all subsequent
		 * frame-to-timestamp matching, causing ~22dB PSNR drops on
		 * ~9% of MPEG1 decodes (and potentially other VDEC_1 codecs).
		 *
		 * Clear any pending interrupt first, then enable.
		 */
		amvdec_write_dos(core, ASSIST_MBOX1_CLR_REG, 1);
		amvdec_write_dos(core, ASSIST_MBOX1_MASK, 1);
	}

	/* Call codec post_start after firmware is running and IRQ is enabled.
	 * Used by MJPEG: the firmware waits for MREG_TO_AMRISC to be
	 * non-zero before decoding the first frame.  This must happen
	 * after MPSR=1 because the firmware clears MREG_TO_AMRISC during
	 * its init sequence (address 0x0009 in gxl_mjpeg.bin).
	 */
	if (codec_ops->post_start)
		codec_ops->post_start(sess);

	return 0;

stop:
	__vdec_1_stop(sess);
	return ret;
}

void vdec_1_thermal_update(struct amvdec_core *core, unsigned long rate)
{
	/*
	 * Only set VDEC_1's own clock. The HEVC clocks that VDEC_1
	 * uses on G12A+ (for MDEC DC cache) are managed by
	 * vdec_hevc_thermal_update() -- no double-set needed.
	 */
	clk_set_rate(core->vdec_1_clk, rate);
}

struct amvdec_ops vdec_1_ops = {
	.start = vdec_1_start,
	.stop = vdec_1_stop,
	.conf_esparser = vdec_1_conf_esparser,
	.vififo_level = vdec_1_vififo_level,
};
