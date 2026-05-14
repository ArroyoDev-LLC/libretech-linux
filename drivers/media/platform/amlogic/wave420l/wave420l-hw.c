// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Chips&Media Wave420L HEVC encoder - hardware operations
 *
 * Copyright (C) 2025 Libre Computer
 *
 * Platform driver, power/clock/reset management, firmware loading,
 * and Wave4 command protocol for Amlogic SM1/G12B SoCs.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-v4l2.h>

#include "wave420l.h"
#include "codec_wave420l_fw.h"

/* DOS register access (word-addressed) */
static inline u32 dos_read(struct wave420l_dev *dev, u32 reg)
{
	return readl(dev->dos_base + (reg << 2));
}

static inline void dos_write(struct wave420l_dev *dev, u32 reg, u32 val)
{
	writel(val, dev->dos_base + (reg << 2));
}

/* HHI register access (word-addressed, via syscon regmap) */
static inline u32 hhi_read(struct wave420l_dev *dev, u32 reg)
{
	u32 val;

	regmap_read(dev->hhi_map, reg << 2, &val);
	return val;
}

static inline void hhi_write(struct wave420l_dev *dev, u32 reg, u32 val)
{
	regmap_write(dev->hhi_map, reg << 2, val);
}

/* AO register access (byte-addressed, via syscon regmap) */
static inline u32 ao_read(struct wave420l_dev *dev, u32 reg)
{
	u32 val;

	regmap_read(dev->ao_map, reg, &val);
	return val;
}

static inline void ao_write(struct wave420l_dev *dev, u32 reg, u32 val)
{
	regmap_write(dev->ao_map, reg, val);
}

/* ------------------------------------------------------------------ */
/* Clock control                                                      */
/* ------------------------------------------------------------------ */

static void wave420l_clk_enable(struct wave420l_dev *dev)
{
	u32 val;

	if (dev->clk_a) {
		/* SC2+ path: use CCF clocks */
		clk_prepare_enable(dev->clk_a);
		clk_prepare_enable(dev->clk_b);
		clk_prepare_enable(dev->clk_c);
		return;
	}

	/* SM1/G12A/G12B path: direct HHI register programming */

	/* Disable clock gates first */
	val = hhi_read(dev, HHI_WAVE420L_CLK_CNTL);
	val &= ~WAVE420L_CLK_GATE_BITS;
	hhi_write(dev, HHI_WAVE420L_CLK_CNTL, val);

	/* Set clock dividers (667MHz from 2000MHz PLL) */
	hhi_write(dev, HHI_WAVE420L_CLK_CNTL, WAVE420L_CLK_L6);

	/* Re-enable clock gates */
	val = hhi_read(dev, HHI_WAVE420L_CLK_CNTL);
	val |= WAVE420L_CLK_GATE_BITS;
	hhi_write(dev, HHI_WAVE420L_CLK_CNTL, val);

	/* ACLK (second clock register, also 667MHz) */
	val = hhi_read(dev, HHI_WAVE420L_CLK_CNTL2);
	val &= ~BIT(8);
	hhi_write(dev, HHI_WAVE420L_CLK_CNTL2, val);

	hhi_write(dev, HHI_WAVE420L_CLK_CNTL2, (2 << 9) | (0 << 0));

	val = hhi_read(dev, HHI_WAVE420L_CLK_CNTL2);
	val |= BIT(8);
	hhi_write(dev, HHI_WAVE420L_CLK_CNTL2, val);
}

static void wave420l_clk_disable(struct wave420l_dev *dev)
{
	u32 val;

	if (dev->clk_a) {
		clk_disable_unprepare(dev->clk_c);
		clk_disable_unprepare(dev->clk_b);
		clk_disable_unprepare(dev->clk_a);
		return;
	}

	/* Disable clock gates */
	val = hhi_read(dev, HHI_WAVE420L_CLK_CNTL);
	val &= ~WAVE420L_CLK_GATE_BITS;
	hhi_write(dev, HHI_WAVE420L_CLK_CNTL, val);

	val = hhi_read(dev, HHI_WAVE420L_CLK_CNTL2);
	val &= ~BIT(8);
	hhi_write(dev, HHI_WAVE420L_CLK_CNTL2, val);
}

/* ------------------------------------------------------------------ */
/* Power domain control                                               */
/* ------------------------------------------------------------------ */

int wave420l_hw_power_on(struct wave420l_dev *dev)
{
	const struct wave420l_match_data *md = dev->match_data;
	u32 val;
	int ret;

	/* Enable DOS bus clock gate first (required for register access) */
	if (dev->clk_dos) {
		ret = clk_prepare_enable(dev->clk_dos);
		if (ret)
			return ret;
	}

	/* Wake from sleep */
	val = ao_read(dev, AO_RTI_GEN_PWR_SLEEP0);
	val &= ~md->pwr_sleep_bits;
	ao_write(dev, AO_RTI_GEN_PWR_SLEEP0, val);
	udelay(10);

	/* Reset (BSP does reset before clocks) */
	val = dos_read(dev, DOS_SW_RESET4);
	val |= md->reset_bits;
	dos_write(dev, DOS_SW_RESET4, val);
	val &= ~md->reset_bits;
	dos_write(dev, DOS_SW_RESET4, val);

	/* Enable clocks */
	wave420l_clk_enable(dev);

	/* Enable Wave420L idle interrupt */
	dos_write(dev, DOS_WAVE420L_CNTL_STAT, 0x1);

	/* Power on memory blocks */
	dos_write(dev, DOS_MEM_PD_WAVE420L, 0x0);

	/* Remove isolation */
	val = ao_read(dev, AO_RTI_GEN_PWR_ISO0);
	val &= ~md->pwr_iso_bits;
	ao_write(dev, AO_RTI_GEN_PWR_ISO0, val);

	return 0;
}

void wave420l_hw_power_off(struct wave420l_dev *dev)
{
	const struct wave420l_match_data *md = dev->match_data;
	u32 val;

	/* Set isolation */
	val = ao_read(dev, AO_RTI_GEN_PWR_ISO0);
	val |= md->pwr_iso_bits;
	ao_write(dev, AO_RTI_GEN_PWR_ISO0, val);
	udelay(10);

	/* Power down memory */
	dos_write(dev, DOS_MEM_PD_WAVE420L, 0xFFFFFFFF);

	/*
	 * Force firmware reload on next power-on.  The firmware DMA buffer
	 * in system DRAM survives power-down, but the VPU's internal SRAM
	 * state is lost.  init_vpu() re-remaps and restarts the core from
	 * the DMA buffer, but load_firmware() must re-copy the firmware
	 * image from the kernel's request_firmware() cache into the DMA
	 * buffer to ensure a clean state (the VPU may have written runtime
	 * data into the code buffer's stack/data region during operation).
	 */
	dev->fw_loaded = false;

	/* Disable clocks */
	wave420l_clk_disable(dev);

	/* Enter sleep */
	val = ao_read(dev, AO_RTI_GEN_PWR_SLEEP0);
	val |= md->pwr_sleep_bits;
	ao_write(dev, AO_RTI_GEN_PWR_SLEEP0, val);
	udelay(10);

	/* Disable DOS bus clock gate last */
	if (dev->clk_dos)
		clk_disable_unprepare(dev->clk_dos);
}

/* ------------------------------------------------------------------ */
/* Wave4 command protocol                                             */
/* ------------------------------------------------------------------ */

int wave420l_wait_busy(struct wave420l_dev *dev)
{
	int timeout = 5000;  /* 50ms max */

	while (wave420l_read(dev, W4_VPU_BUSY_STATUS) && --timeout > 0)
		udelay(10);

	if (timeout <= 0) {
		dev_err(dev->dev, "VPU busy timeout (INT_REASON=0x%x FAIL=0x%x)\n",
			wave420l_read(dev, W4_VPU_INT_REASON),
			wave420l_read(dev, W4_RET_FAIL_REASON));
		return -ETIMEDOUT;
	}
	return 0;
}

/* Sleepable wait for long-running commands like ENC_PIC */
static int wave420l_wait_busy_long(struct wave420l_dev *dev, int timeout_ms)
{
	int elapsed = 0;

	while (wave420l_read(dev, W4_VPU_BUSY_STATUS) && elapsed < timeout_ms) {
		usleep_range(1000, 2000);  /* 1-2ms sleep */
		elapsed++;
	}

	if (elapsed >= timeout_ms) {
		dev_err(dev->dev, "VPU busy timeout after %dms (INT_REASON=0x%x FAIL=0x%x)\n",
			timeout_ms,
			wave420l_read(dev, W4_VPU_INT_REASON),
			wave420l_read(dev, W4_RET_FAIL_REASON));
		return -ETIMEDOUT;
	}
	return 0;
}

int wave420l_issue_command(struct wave420l_dev *dev, u32 inst_idx, u32 cmd)
{
	wave420l_write(dev, W4_VPU_BUSY_STATUS, 1);
	wave420l_write(dev, W4_RET_SUCCESS, 0);  /* clear for reliable detection */
	wave420l_write(dev, W4_CORE_INDEX, 0);
	/*
	 * W4_INST_INDEX: instanceIndex[15:0] | codecMode[31:16]
	 * HEVC_ENC = 1 (SDK: vpuapifunc.h)
	 */
	wave420l_write(dev, W4_INST_INDEX,
		       (inst_idx & 0xffff) | (W4_CODEC_MODE_HEVC_ENC << 16));
	wave420l_write(dev, W4_COMMAND, cmd);

	/* Trigger host interrupt to VPU (skip for INIT_VPU — core not running yet) */
	if (cmd != W4_CMD_INIT_VPU)
		wave420l_write(dev, W4_VPU_HOST_INT_REQ, 1);

	return 0;
}

/* ------------------------------------------------------------------ */
/* Firmware loading                                                   */
/* ------------------------------------------------------------------ */

int wave420l_hw_load_firmware(struct wave420l_dev *dev)
{
	const struct firmware *fw;
	const struct amvdec_fw_version *match;
	int ret;

	if (dev->fw_loaded)
		return 0;

	ret = request_firmware(&fw, WAVE420L_FW_NAME, dev->dev);
	if (ret) {
		dev_err(dev->dev, "failed to load firmware %s: %d\n",
			WAVE420L_FW_NAME, ret);
		return ret;
	}

	match = amvdec_fw_identify(dev->dev, WAVE420L_FW_NAME,
				   fw->data, fw->size,
				   wave420l_fw_versions, WAVE420L_FW_VERSIONS_COUNT);
	if (IS_ERR(match)) {
		ret = PTR_ERR(match);
		release_firmware(fw);
		return ret;
	}
	/* Encoder has no quirk bits yet; don't persist match. */

	/* Allocate DMA buffer for firmware code (1MB — VPU uses extra space for stack/data) */
	if (!dev->fw_buf.virt) {
		dev->fw_buf.size = WAVE420L_CODE_BUF_SIZE;
		dev->fw_buf.virt = dma_alloc_coherent(dev->dev, dev->fw_buf.size,
						      &dev->fw_buf.dma, GFP_KERNEL);
		if (!dev->fw_buf.virt) {
			release_firmware(fw);
			return -ENOMEM;
		}
	}

	memset(dev->fw_buf.virt, 0, dev->fw_buf.size);
	memcpy(dev->fw_buf.virt, fw->data, fw->size);
	release_firmware(fw);

	dev->fw_loaded = true;
	return 0;
}

int wave420l_hw_init_vpu(struct wave420l_dev *dev)
{
	u32 remap_size, regval;
	u32 code_base = dev->fw_buf.dma;
	u32 code_size = WAVE420L_CODE_BUF_SIZE & ~0xFFF;
	u32 i;
	int ret;

	/* Reset VPU configuration */
	wave420l_write(dev, W4_PO_CONF, 0);

	/* Assert reset on all blocks */
	wave420l_write(dev, W4_VPU_RESET_REQ, 0x7FFFFFF);

	/* Wait for reset to complete */
	ret = readl_poll_timeout(dev->reg_base + W4_VPU_RESET_STATUS,
				 regval, regval == 0, 10, 100000);
	if (ret) {
		dev_err(dev->dev, "VPU reset timeout\n");
		return ret;
	}

	/* Release reset */
	wave420l_write(dev, W4_VPU_RESET_REQ, 0);

	/* Clear command registers */
	for (i = 0x100; i < 0x200; i += 4)
		wave420l_write(dev, i, 0);

	/* Setup memory remap (map 1MB code buffer to VPU address 0) */
	remap_size = (code_size >> 12) & 0x1FF;
	wave420l_write(dev, W4_VPU_REMAP_CTRL,
		       0x80000000 | (0 << 16) | (0 << 12) | BIT(11) | remap_size);
	wave420l_write(dev, W4_VPU_REMAP_VADDR, 0x00000000);
	wave420l_write(dev, W4_VPU_REMAP_PADDR, code_base);

	wave420l_write(dev, W4_ADDR_CODE_BASE, code_base);
	wave420l_write(dev, W4_CODE_SIZE, code_size);
	wave420l_write(dev, W4_CODE_PARAM, 0);

	wave420l_write(dev, W4_TIMEOUT_CNT, 0xFFFFFFFF);
	wave420l_write(dev, W4_HW_OPTION, 0);

	/* Enable interrupts (include WAKEUP for runtime PM resume) */
	regval = BIT(W4_INT_INIT_VPU) | BIT(W4_INT_SET_PARAM) |
		 BIT(W4_INT_FINI_SEQ) | BIT(W4_INT_ENC_PIC) |
		 BIT(W4_INT_SET_FRAMEBUF) | BIT(W4_INT_SLEEP_VPU) |
		 BIT(W4_INT_WAKEUP_VPU) | BIT(W4_INT_CREATE_INSTANCE);
	wave420l_write(dev, W4_VPU_VINT_ENABLE, regval);

	/* Issue INIT_VPU command and start the processor */
	wave420l_issue_command(dev, 0, W4_CMD_INIT_VPU);
	wave420l_write(dev, W4_VPU_REMAP_CORE_START, 1);

	/* Wait for initialization */
	ret = wave420l_wait_busy(dev);
	if (ret) {
		dev_err(dev->dev, "VPU init timeout (BUSY=0x%x FAIL=0x%x)\n",
			wave420l_read(dev, W4_VPU_BUSY_STATUS),
			wave420l_read(dev, W4_RET_FAIL_REASON));
		return ret;
	}

	if (wave420l_read(dev, W4_RET_SUCCESS) == 0) {
		dev_err(dev->dev, "VPU init failed, reason: 0x%x\n",
			wave420l_read(dev, W4_RET_FAIL_REASON));
		return -EIO;
	}

	dev_dbg(dev->dev, "Wave420L firmware initialized, FW ver=0x%x product=0x%x\n",
		 wave420l_read(dev, W4_RET_FW_VERSION),
		 wave420l_read(dev, W4_RET_PRODUCT_VERSION));

	return 0;
}

/*
 * VPU reference counting: power on + init on first instance,
 * power off on last instance release.  Protected by vpu_lock.
 *
 * Note: dev_lock cannot be used here because it is also the V4L2
 * video device lock (vdev->lock) and VB2 queue lock (vq->lock).
 * The V4L2 core holds dev_lock when calling start_streaming, so
 * acquiring it again here would deadlock.
 */
int wave420l_hw_vpu_acquire(struct wave420l_dev *dev)
{
	int ret = 0;

	mutex_lock(&dev->vpu_lock);

	if (dev->vpu_refcount >= WAVE420L_MAX_INSTANCES) {
		dev_warn(dev->dev, "VPU acquire: max instances (%d) reached\n",
			 WAVE420L_MAX_INSTANCES);
		ret = -EBUSY;
		goto out;
	}

	if (dev->vpu_refcount == 0) {
		ret = wave420l_hw_power_on(dev);
		if (ret)
			goto out;

		ret = wave420l_hw_load_firmware(dev);
		if (ret) {
			wave420l_hw_power_off(dev);
			goto out;
		}

		ret = wave420l_hw_init_vpu(dev);
		if (ret) {
			wave420l_hw_power_off(dev);
			goto out;
		}

		dev->vpu_initialized = true;
	}

	dev->vpu_refcount++;
	dev_dbg(dev->dev, "VPU acquire: refcount=%d\n", dev->vpu_refcount);

out:
	mutex_unlock(&dev->vpu_lock);
	return ret;
}

void wave420l_hw_vpu_release(struct wave420l_dev *dev)
{
	mutex_lock(&dev->vpu_lock);

	if (WARN_ON(dev->vpu_refcount <= 0)) {
		mutex_unlock(&dev->vpu_lock);
		return;
	}

	dev->vpu_refcount--;
	dev_dbg(dev->dev, "VPU release: refcount=%d\n", dev->vpu_refcount);

	if (dev->vpu_refcount == 0) {
		wave420l_hw_power_off(dev);
		dev->vpu_initialized = false;
	}

	mutex_unlock(&dev->vpu_lock);
}

/* ------------------------------------------------------------------ */
/* Instance management                                                */
/* ------------------------------------------------------------------ */

int wave420l_hw_create_instance(struct wave420l_instance *inst)
{
	struct wave420l_dev *dev = inst->dev;
	int ret;

	/* Allocate work buffer */
	inst->work_buf.size = WAVE420L_WORK_BUF_SIZE;
	inst->work_buf.virt = dma_alloc_coherent(dev->dev, inst->work_buf.size,
						 &inst->work_buf.dma, GFP_KERNEL);
	if (!inst->work_buf.virt)
		return -ENOMEM;

	/* Allocate temp buffer */
	inst->temp_buf.size = WAVE420L_TEMP_BUF_SIZE;
	inst->temp_buf.virt = dma_alloc_coherent(dev->dev, inst->temp_buf.size,
						 &inst->temp_buf.dma, GFP_KERNEL);
	if (!inst->temp_buf.virt) {
		ret = -ENOMEM;
		goto err_free_work;
	}

	/* Allocate bitstream ring buffer for firmware */
	inst->bs_buf.size = WAVE420L_BS_BUF_SIZE_HW;
	inst->bs_buf.virt = dma_alloc_coherent(dev->dev, inst->bs_buf.size,
					       &inst->bs_buf.dma, GFP_KERNEL);
	if (!inst->bs_buf.virt) {
		ret = -ENOMEM;
		goto err_free_temp;
	}

	/* Configure instance parameters */
	wave420l_write(dev, W4_ADDR_WORK_BASE, inst->work_buf.dma);
	wave420l_write(dev, W4_WORK_SIZE, inst->work_buf.size);
	wave420l_write(dev, W4_WORK_PARAM, 0);

	wave420l_write(dev, W4_ADDR_TEMP_BASE, inst->temp_buf.dma);
	wave420l_write(dev, W4_TEMP_SIZE, inst->temp_buf.size);
	wave420l_write(dev, W4_TEMP_PARAM, 0);

	/* Encoder mode: HEVC = 0 */
	wave420l_write(dev, W4_CMD_ENC_SET_PARAM_ENABLE, 0);

	mutex_lock(&dev->hw_lock);

	wave420l_issue_command(dev, inst->id, W4_CMD_CREATE_INSTANCE);
	ret = wave420l_wait_busy(dev);

	if (!ret && wave420l_read(dev, W4_RET_SUCCESS) == 0) {
		dev_err(dev->dev, "CREATE_INSTANCE failed: 0x%x\n",
			wave420l_read(dev, W4_RET_FAIL_REASON));
		ret = -EIO;
	}

	mutex_unlock(&dev->hw_lock);

	if (ret)
		goto err_free_bs;

	return 0;

err_free_bs:
	dma_free_coherent(dev->dev, inst->bs_buf.size,
			  inst->bs_buf.virt, inst->bs_buf.dma);
err_free_temp:
	dma_free_coherent(dev->dev, inst->temp_buf.size,
			  inst->temp_buf.virt, inst->temp_buf.dma);
err_free_work:
	dma_free_coherent(dev->dev, inst->work_buf.size,
			  inst->work_buf.virt, inst->work_buf.dma);
	return ret;
}

int wave420l_hw_destroy_instance(struct wave420l_instance *inst)
{
	struct wave420l_dev *dev = inst->dev;
	int ret;
	int i;

	mutex_lock(&dev->hw_lock);
	wave420l_issue_command(dev, inst->id, W4_CMD_DESTROY_INSTANCE);
	ret = wave420l_wait_busy(dev);
	mutex_unlock(&dev->hw_lock);

	/* Free reference frame buffers allocated by wave420l_hw_set_fb() */
	if (inst->fb_bufs) {
		for (i = 0; i < inst->fb_count; i++) {
			if (inst->fb_bufs[i].virt)
				dma_free_coherent(dev->dev,
						  inst->fb_bufs[i].size,
						  inst->fb_bufs[i].virt,
						  inst->fb_bufs[i].dma);
		}
		kfree(inst->fb_bufs);
		inst->fb_bufs = NULL;
	}

	if (inst->sub_sam_buf.virt) {
		dma_free_coherent(dev->dev, inst->sub_sam_buf.size,
				  inst->sub_sam_buf.virt, inst->sub_sam_buf.dma);
		inst->sub_sam_buf.virt = NULL;
	}

	if (inst->fbc_c_tbl_buf.virt) {
		dma_free_coherent(dev->dev, inst->fbc_c_tbl_buf.size,
				  inst->fbc_c_tbl_buf.virt,
				  inst->fbc_c_tbl_buf.dma);
		inst->fbc_c_tbl_buf.virt = NULL;
	}

	if (inst->fbc_y_tbl_buf.virt) {
		dma_free_coherent(dev->dev, inst->fbc_y_tbl_buf.size,
				  inst->fbc_y_tbl_buf.virt,
				  inst->fbc_y_tbl_buf.dma);
		inst->fbc_y_tbl_buf.virt = NULL;
	}

	if (inst->mv_buf.virt) {
		dma_free_coherent(dev->dev, inst->mv_buf.size,
				  inst->mv_buf.virt, inst->mv_buf.dma);
		inst->mv_buf.virt = NULL;
	}

	/* Free instance command buffers */
	if (inst->bs_buf.virt) {
		dma_free_coherent(dev->dev, inst->bs_buf.size,
				  inst->bs_buf.virt, inst->bs_buf.dma);
		inst->bs_buf.virt = NULL;
	}

	if (inst->temp_buf.virt) {
		dma_free_coherent(dev->dev, inst->temp_buf.size,
				  inst->temp_buf.virt, inst->temp_buf.dma);
		inst->temp_buf.virt = NULL;
	}

	if (inst->work_buf.virt) {
		dma_free_coherent(dev->dev, inst->work_buf.size,
				  inst->work_buf.virt, inst->work_buf.dma);
		inst->work_buf.virt = NULL;
	}

	return ret;
}

int wave420l_hw_init_seq(struct wave420l_instance *inst)
{
	struct wave420l_dev *dev = inst->dev;
	u32 pic_param, gop_param, rc_param, enc_param, intra_param;
	int ret;

	mutex_lock(&dev->hw_lock);

	/*
	 * Bitstream buffer (required before SET_PARAM, SDK: Wave4VpuEncSetup)
	 * BS_PARAM: lineBufIntEn[6] | sliceIntEnable[5] | ringBufferEnable[4] | endian[3:0]
	 * Endian 0xf = little-endian for ARM (SDK: ~vdi_convert_endian(LE) & 0xf)
	 */
	wave420l_write(dev, W4_BS_START_ADDR, inst->bs_buf.dma);
	wave420l_write(dev, W4_BS_SIZE, inst->bs_buf.size);
	wave420l_write(dev, W4_BS_RD_PTR, inst->bs_buf.dma);
	wave420l_write(dev, W4_BS_WR_PTR, inst->bs_buf.dma);
	wave420l_write(dev, W4_BS_PARAM, 0xf);  /* no ring buffer, LE endian */

	/* Secondary AXI (disabled — no SRAM) */
	wave420l_write(dev, W4_ADDR_SEC_AXI, 0);
	wave420l_write(dev, W4_SEC_AXI_SIZE, 0);
	wave420l_write(dev, W4_USE_SEC_AXI, 0);

	/* Work buffer */
	wave420l_write(dev, W4_ADDR_WORK_BASE, inst->work_buf.dma);
	wave420l_write(dev, W4_WORK_SIZE, inst->work_buf.size);
	wave420l_write(dev, W4_WORK_PARAM, 0);

	/* Temp buffer */
	wave420l_write(dev, W4_ADDR_TEMP_BASE, inst->temp_buf.dma);
	wave420l_write(dev, W4_TEMP_SIZE, inst->temp_buf.size);
	wave420l_write(dev, W4_TEMP_PARAM, 0);

	/* Bitstream buffer again (SDK writes it twice) */
	wave420l_write(dev, W4_BS_START_ADDR, inst->bs_buf.dma);
	wave420l_write(dev, W4_BS_SIZE, inst->bs_buf.size);

	/* SET_PARAM + OPT_COMMON */
	wave420l_write(dev, W4_CMD_ENC_SET_PARAM_OPTION, W4_OPT_COMMON);
	wave420l_write(dev, W4_CMD_ENC_SET_PARAM_ENABLE, W4_ENC_CHANGE_SET_PARAM_ALL);

	/*
	 * SRC_SIZE: height[31:16] | width[15:0]  (SDK order)
	 */
	wave420l_write(dev, W4_CMD_ENC_SEQ_SRC_SIZE,
		       (inst->src_fmt.height << 16) | inst->src_fmt.width);

	/*
	 * PIC_PARAM for WAVE420L (NOT Wave420 — different bit layout):
	 *   profile[2:0] | level[11:3] | tier[12] | internalBitDepth[17:14]
	 *   | chromaFormatIdc[19:18] | losslessEnable[20] | constIntraPredFlag[21]
	 *   | chromaCbQpOffset[26:22] | chromaCrQpOffset[31:27]
	 *
	 * Profile: firmware expects HEVC spec values (Main=1, Main10=2, MainSP=3)
	 * Level: firmware expects general_level_idc (30*level, e.g. Level 4.0=120)
	 */
	{
		/* Convert V4L2 profile enum to firmware profile value */
		u32 fw_profile;
		u32 fw_level;

		switch (inst->profile) {
		case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN:
			fw_profile = 1;
			break;
		case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10:
			fw_profile = 2;
			break;
		case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE:
			fw_profile = 3;
			break;
		default:
			fw_profile = 1;
			break;
		}

		/* Convert V4L2 level enum to HEVC level_idc */
		switch (inst->level) {
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_1:   fw_level = 30;  break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_2:   fw_level = 60;  break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_2_1: fw_level = 63;  break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_3:   fw_level = 90;  break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_3_1: fw_level = 93;  break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_4:   fw_level = 120; break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1: fw_level = 123; break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_5:   fw_level = 150; break;
		case V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1: fw_level = 153; break;
		default:                             fw_level = 120; break;
		}

		/*
		 * BSP uses level=0 (auto) and chromaFormatIdc=0 (firmware default).
		 * Setting level=120 or chromaFormatIdc=1 may cause VCL encoding
		 * to fail with watchdog timeout.
		 */

		/*
		 * internalBitDepth: derive from pixel format.
		 * P010 (16-bit/sample 10-bit LSB) requires Main 10 profile and
		 * internalBitDepth=10.  All other formats use 8-bit.
		 * SDK wave4.c:756: (pHevc->internalBitDepth<<14)
		 */
		{
			u32 internal_bit_depth;

			if (inst->src_fmt.pixelformat == V4L2_PIX_FMT_P010)
				internal_bit_depth = 10;
			else
				internal_bit_depth = 8;

			pic_param = (fw_profile & 0x7);
			pic_param |= (0 & 0x1FF) << 3;    /* level = 0 (auto) */
			pic_param |= (inst->tier & 1) << 12;
			pic_param |= (internal_bit_depth & 0xF) << 14; /* internalBitDepth */
			pic_param |= (0 & 0x3) << 18;	   /* chromaFormatIdc = 0 (BSP default) */
			if (inst->lossless)
				pic_param |= BIT(20);	   /* losslessEnable */
			pic_param |= (inst->chroma_cb_qp_offset & 0x1F) << 22;
			pic_param |= (inst->chroma_cr_qp_offset & 0x1F) << 27;
			/*
			 * useLongTerm: the BSP uses PIC_PARAM[23] for this, but
			 * bits [26:22] are chromaCbQpOffset.  Long-term reference
			 * is instead enabled per-frame via W4_CMD_ENC_LONGTERM_PIC.
			 */
		}

		dev_info(dev->dev, "SET_PARAM: profile=%u level=%u internal_bitdepth=%u cb_qp_off=%d cr_qp_off=%d pic_param=0x%x\n",
			 fw_profile, fw_level,
			 (inst->src_fmt.pixelformat == V4L2_PIX_FMT_P010) ? 10 : 8,
			 inst->chroma_cb_qp_offset, inst->chroma_cr_qp_offset,
			 pic_param);
	}
	wave420l_write(dev, W4_CMD_ENC_SEQ_PIC_PARAM, pic_param);

	/*
	 * GOP_PARAM for WAVE420L:
	 *   gopPresetIdx[7:0] | enTemporalLayerQp[8] | tidQp0[14:9]
	 *   | tidQp1[20:15] | tidQp2[26:21]
	 *
	 * SDK gopPresetIdx mapping (vpuapi.h):
	 *   2 = PRESET_IDX_IPP (consecutive P, no B-frames)
	 *   4 = PRESET_IDX_IBPBP (1 B-frame, gopsize=2)
	 *   5 = PRESET_IDX_IBBBP (3 B-frames, gopsize=4)
	 *   3 = PRESET_IDX_IBBB (consecutive B, gopsize=1)
	 */
	/*
	 * GOP preset index:
	 *   2 = IPP (no B-frames, consecutive P)
	 *   4 = IBPBP (1 B-frame)
	 *   5 = IBBBP (3 B-frames, gopsize=4)
	 */
	if (inst->custom_gop_enable) {
		/*
		 * Custom GOP: gopPresetIdx=0 signals the firmware that a second
		 * SET_PARAM (OPT_CUSTOM_GOP) command will follow with per-picture
		 * GOP entries.  SDK wave4.c:915.
		 */
		gop_param = 0;  /* PRESET_IDX_CUSTOM_GOP */
	} else if (inst->gop_preset) {
		gop_param = inst->gop_preset;
	} else {
		switch (inst->b_frames) {
		case 1:
			gop_param = 4;  /* IBPBP */
			break;
		case 3:
			gop_param = 5;  /* IBBBP */
			break;
		default:
			gop_param = 2;  /* IPP (no B-frames) */
			break;
		}
	}
	/*
	 * Temporal layer QP (UND-TEMPORAL-QP):
	 * GOP_PARAM[8]    = enTemporalLayerQp (enable flag)
	 * GOP_PARAM[14:9] = tidQp0 (signed 6-bit, -32..+31)
	 * GOP_PARAM[20:15]= tidQp1 (signed 6-bit)
	 * GOP_PARAM[26:21]= tidQp2 (signed 6-bit)
	 * Per SDK wave4.c:791 (gopParam.enTemporalLayerQp, tidQp0..2).
	 */
	if (inst->temporal_layer_qp[0] || inst->temporal_layer_qp[1] ||
	    inst->temporal_layer_qp[2]) {
		gop_param |= BIT(8);  /* enTemporalLayerQp */
		gop_param |= ((u32)(inst->temporal_layer_qp[0] & 0x3F) << 9);
		gop_param |= ((u32)(inst->temporal_layer_qp[1] & 0x3F) << 15);
		gop_param |= ((u32)(inst->temporal_layer_qp[2] & 0x3F) << 21);
	}
	wave420l_write(dev, W4_CMD_ENC_SEQ_GOP_PARAM, gop_param);

	/*
	 * Track how many frames the firmware reorder pipeline buffers.
	 * Used by wave420l_hw_flush_instance() to drain the correct number
	 * of B-frames on EOS:
	 *   IPP (preset 2): 0 -- no B-frames
	 *   IBPBP (preset 4): 1 -- 1 B-frame delay
	 *   IBBBP (preset 5): 3 -- 3 B-frame delay
	 */
	switch (inst->b_frames) {
	case 1:  inst->bframe_delay = 1; break;
	case 3:  inst->bframe_delay = 3; break;
	default: inst->bframe_delay = 0; break;
	}

	/*
	 * INTRA_PARAM: decodingRefreshType[2:0] | intraQP[8:3]
	 *   | forcedIdrHeaderEnable[9] | intraPeriod[31:16]
	 *
	 * V4L2 refresh type mapping to SDK decodingRefreshType:
	 *   V4L2_MPEG_VIDEO_HEVC_REFRESH_NONE = 0 -> 0 (non-IDR, non-CRA)
	 *   V4L2_MPEG_VIDEO_HEVC_REFRESH_CRA  = 1 -> 1 (CRA)
	 *   V4L2_MPEG_VIDEO_HEVC_REFRESH_IDR  = 2 -> 2 (IDR)
	 *
	 * intraPeriod: if refresh_period > 0, use it; otherwise use gop_size.
	 */
	{
		u32 intra_period = inst->refresh_period > 0 ?
				   inst->refresh_period : inst->gop_size;

		intra_param = (inst->refresh_type & 0x7);
		intra_param |= (inst->init_qp & 0x3F) << 3;
		/*
		 * bit 9 = forcedIdrHeaderEnable: do NOT set.
		 * Setting this bit disables the firmware's implicit VPS/SPS/PPS
		 * emission via CODEOPT_ENC_HEADER_IMPLICIT (BIT(0) in
		 * ENC_CODE_OPTION).  Leave bit 9 = 0 so the firmware emits
		 * headers at each IDR via the implicit path (KI-092 root cause).
		 */
		intra_param |= (intra_period & 0xFFFF) << 16;
	}
	wave420l_write(dev, W4_CMD_ENC_SEQ_INTRA_PARAM, intra_param);

	wave420l_write(dev, W4_CMD_ENC_SEQ_CONF_WIN_TOP_BOT, 0);
	wave420l_write(dev, W4_CMD_ENC_SEQ_CONF_WIN_LR, 0);
	wave420l_write(dev, W4_CMD_ENC_SEQ_FRAME_RATE,
		       inst->frame_rate ? inst->frame_rate : 30);
	/*
	 * Independent slice: independSliceMode[15:0] | independSliceModeArg[31:16]
	 *   mode 0 = no slicing, mode 1 = N CTUs per slice
	 * Map V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE(0) -> mode=0
	 *     V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_MB(1) -> mode=1, arg=slice_arg
	 * (control name says MB but for HEVC we use CTU count)
	 */
	wave420l_write(dev, W4_CMD_ENC_SEQ_INDEP_SLICE,
		       ((u32)inst->slice_arg << 16) | inst->slice_mode);
	wave420l_write(dev, W4_CMD_ENC_SEQ_DEP_SLICE, 0);
	/*
	 * INTRA_REFRESH (BSP: intraRefreshArg<<16 | intraRefreshMode):
	 *   intraRefreshMode[15:0] = 0=none, 1=row, 2=col, 3=step
	 *   intraRefreshArg[31:16] = CTUs per refresh step
	 */
	wave420l_write(dev, W4_CMD_ENC_SEQ_INTRA_REFRESH,
		       (inst->intra_refresh_num << 16) |
		       (inst->intra_refresh_mode & 0xFFFF));

	/*
	 /*
	 * ENC_PARAM (SDK: wave4.c W4_CMD_ENC_PARAM):
	 *   useRecommendEncParam[2:0] | scalingListEnable[3]
	 *   | cuSizeMode[6:4] | tmvpEnable[7] | wppEnable[8]
	 *   | maxNumMerge[11:9] | dynamicMerge8x8Enable[12]
	 *   | dynamicMerge16x16Enable[13] | dynamicMerge32x32Enable[14]
	 *   | disableDeblk[15] | lfCrossSliceBoundaryEnable[16]
	 *   | betaOffsetDiv2[20:17] | tcOffsetDiv2[24:21]
	 *   | skipIntraTrans[25] | saoEnable[26]
	 *   | intraInInterSliceEnable[27] | intraNxNEnable[28]
	 *
	 * Loop filter mode mapping (V4L2 -> SDK bits):
	 *   Disabled(0):                 disableDeblk=1, lfCross=x
	 *   Enabled(1):                  disableDeblk=0, lfCross=1
	 *   DisabledAtSliceBoundary(2):  disableDeblk=0, lfCross=0
	 *
	 * betaOffsetDiv2/tcOffsetDiv2: signed [-6..+6], stored as (val+6) in 4-bit field.
	 */
	{
		u32 disable_deblk = 0;
		u32 lf_cross = 1;
		u32 beta_reg, tc_reg;

		switch (inst->lf_mode) {
		case V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_DISABLED:
			disable_deblk = 1;
			lf_cross = 0;
			break;
		case V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_ENABLED:
			disable_deblk = 0;
			lf_cross = 1;
			break;
		case V4L2_MPEG_VIDEO_HEVC_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY:
			disable_deblk = 0;
			lf_cross = 0;
			break;
		}

		/* SDK stores raw signed value masked to 4 bits (two's complement) */
		beta_reg = (u32)(inst->lf_beta_offset & 0xF);
		tc_reg = (u32)(inst->lf_tc_offset & 0xF);

		enc_param = 1;  /* useRecommendEncParam = 1 (matches BSP) */
		if (inst->scaling_list_enable)
			enc_param |= BIT(3);  /* scalingListEnable */
		/* cuSizeMode[6:4]: CU size bitmask (default 0x7 = all sizes) */
		enc_param |= ((inst->cu_size_mode ? inst->cu_size_mode : 0x7) << 4);
		enc_param |= (inst->tmv_prediction ? 1 : 0) << 7;  /* tmvpEnable */
		enc_param |= (inst->max_num_merge & 0x7) << 9;     /* maxNumMerge */
		enc_param |= (disable_deblk << 15);
		enc_param |= (lf_cross << 16);
		enc_param |= (beta_reg << 17);
		enc_param |= (tc_reg << 21);
	}
	wave420l_write(dev, W4_CMD_ENC_PARAM, enc_param);

	/*
	 * Layer period (WAVE420L only) -- must be written before
	 * RC_INTRA_MIN_MAX_QP since they share offset 0x0194.
	 * We don't use temporal layers so LAYER_PERIOD=0.
	 */
	wave420l_write(dev, W4_CMD_ENC_SEQ_LAYER_PERIOD, 0);

	/* Intra-frame QP range: intraMinQp[5:0] | intraMaxQp[11:6] */
	wave420l_write(dev, W4_CMD_ENC_RC_INTRA_MIN_MAX_QP,
		       (inst->i_min_qp & 0x3F) |
		       ((inst->i_max_qp & 0x3F) << 6));

	/*
	 * RC_PARAM for WAVE420L:
	 *   rcEnable[0] | cuLevelRCEnable[1] | hvsQPEnable[2]
	 *   | hvsQpScaleEnable[3] | hvsQpScale[6:4] | bitAllocMode[8:7]
	 *   | initBufLevelx8[12:9] | roiEnable[13] | initialRcQp[19:14]
	 *   | initialDelay[31:20]
	 */
	if (inst->rc_enable && inst->bitrate > 0) {
		rc_param = 1;				/* rcEnable */
		rc_param |= (inst->cu_rc_enable << 1);	/* cuLevelRCEnable */
		rc_param |= (1 << 2);			/* hvsQPEnable */
		rc_param |= (1 << 3);			/* hvsQpScaleEnable */
		rc_param |= (2 << 4);			/* hvsQpScale=2 */
		rc_param |= (1 << 9);			/* initBufLevelx8=1 */
		rc_param |= (63 << 14);		/* initialRcQp=63 (auto) */
		rc_param |= ((inst->vbv_size & 0xFFF) << 20); /* initialDelay */
	} else {
		/* CQP mode: enable HVS QP for perceptual quality */
		rc_param = 0;
		rc_param |= (1 << 2);			/* hvsQPEnable */
		rc_param |= (1 << 3);			/* hvsQpScaleEnable */
		rc_param |= (2 << 4);			/* hvsQpScale=2 */
	}
	wave420l_write(dev, W4_CMD_ENC_RC_PARAM, rc_param);

	wave420l_write(dev, W4_CMD_ENC_RC_MIN_MAX_QP,
		       (inst->min_qp & 0x3F) |
		       ((inst->max_qp & 0x3F) << 6) |
		       (10 << 12));

	wave420l_write(dev, W4_CMD_ENC_RC_BIT_RATIO_LAYER_0_3, 0);
	wave420l_write(dev, W4_CMD_ENC_RC_BIT_RATIO_LAYER_4_7, 0);
	/*
	 * NR_PARAM: nrYEnable[0] | nrCbEnable[1] | nrCrEnable[2]
	 *   | nrNoiseEstEnable[3] | nrNoiseSigmaY[11:4]
	 *   | nrNoiseSigmaCb[19:12] | nrNoiseSigmaCr[27:20]
	 * NR_WEIGHT: intra/inter weight per component (SDK defaults: intra=7, inter=4).
	 */
	wave420l_write(dev, W4_CMD_ENC_NR_PARAM,
		       (inst->nr_y_enable ? 1 : 0) |
		       ((inst->nr_cb_enable ? 1 : 0) << 1) |
		       ((inst->nr_cr_enable ? 1 : 0) << 2) |
		       ((inst->nr_noise_est_enable ? 1 : 0) << 3) |
		       ((inst->nr_noise_sigma_y & 0xFF) << 4) |
		       ((inst->nr_noise_sigma_cb & 0xFF) << 12) |
		       ((inst->nr_noise_sigma_cr & 0xFF) << 20));
	/*
	 * NR_WEIGHT: nrIntraWeightY[4:0] | nrIntraWeightCb[9:5]
	 *   | nrIntraWeightCr[14:10] | nrInterWeightY[19:15]
	 *   | nrInterWeightCb[24:20] | nrInterWeightCr[29:25]
	 */
	wave420l_write(dev, W4_CMD_ENC_NR_WEIGHT,
		       (inst->nr_intra_weight_y & 0x1F) |
		       ((inst->nr_intra_weight_cb & 0x1F) << 5) |
		       ((inst->nr_intra_weight_cr & 0x1F) << 10) |
		       ((inst->nr_inter_weight_y & 0x1F) << 15) |
		       ((inst->nr_inter_weight_cb & 0x1F) << 20) |
		       ((inst->nr_inter_weight_cr & 0x1F) << 25));
	/*
	 * NUM_UNITS_IN_TICK and TIME_SCALE convey the frame rate to the
	 * firmware's rate control engine.  Without these, RC cannot compute
	 * the per-frame bit budget.  SDK sets numUnitsInTick=1,
	 * timeScale=framerate.
	 */
	wave420l_write(dev, W4_CMD_ENC_NUM_UNITS_IN_TICK, 1);
	wave420l_write(dev, W4_CMD_ENC_TIME_SCALE, inst->frame_rate);
	wave420l_write(dev, W4_CMD_ENC_NUM_TICKS_POC_DIFF_ONE, 0);
	/*
	 * BSP SDK passes bitrate in bps (not kbps) to transRate/targetRate.
	 *
	 * CBR: transRate == targetRate (constant bitrate).
	 * VBR: transRate = 1.5x targetRate (peak bitrate budget).
	 *      The firmware uses transRate as the VBV buffer rate and
	 *      targetRate as the average target.  Setting transRate > targetRate
	 *      enables VBR mode where the encoder can spend bits freely up to
	 *      the peak.  (per Wave420L BSP SDK vpuapi.h: transRate doc)
	 */
	if (inst->rc_enable && inst->bitrate > 0) {
		u32 trans_rate = inst->bitrate;

		if (inst->bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR)
			trans_rate = inst->bitrate + inst->bitrate / 2;
		wave420l_write(dev, W4_CMD_ENC_RC_TRANS_RATE, trans_rate);
		wave420l_write(dev, W4_CMD_ENC_RC_TARGET_RATE, inst->bitrate);
	} else {
		wave420l_write(dev, W4_CMD_ENC_RC_TRANS_RATE, 0);
		wave420l_write(dev, W4_CMD_ENC_RC_TARGET_RATE, 0);
	}
	/*
	 * ROT_PARAM: rot_en[0] | rot_angle[2:1] | ver_mir[3] | hor_mir[4]
	 * rot_angle: 0=0, 1=90, 2=180, 3=270
	 */
	{
		u32 rot_mir_mode = 0;

		if (inst->rotation != 0 || inst->hflip || inst->vflip) {
			rot_mir_mode |= 1;  /* rot_en */
			switch (inst->rotation) {
			case 90:
				rot_mir_mode |= (1 << 1);
				break;
			case 180:
				rot_mir_mode |= (2 << 1);
				break;
			case 270:
				rot_mir_mode |= (3 << 1);
				break;
			default:
				break;
			}
			if (inst->vflip)
				rot_mir_mode |= (1 << 3);
			if (inst->hflip)
				rot_mir_mode |= (1 << 4);
		}
		wave420l_write(dev, W4_CMD_ENC_ROT_PARAM, rot_mir_mode);
	}

	wave420l_issue_command(dev, inst->id, W4_CMD_INIT_SEQ);
	ret = wave420l_wait_busy(dev);

	if (!ret && wave420l_read(dev, W4_RET_SUCCESS) == 0) {
		dev_err(dev->dev, "INIT_SEQ failed: 0x%x\n",
			wave420l_read(dev, W4_RET_FAIL_REASON));
		ret = -EIO;
	}

	if (!ret) {
		inst->min_fb_count = wave420l_read(dev, W4_RET_ENC_MIN_FB_NUM);
		inst->min_src_count = wave420l_read(dev, W4_RET_ENC_MIN_SRC_BUF_NUM);
		dev_dbg(dev->dev, "INIT_SEQ OK: inst=%d min_fb=%u min_src=%u\n",
			 inst->id, inst->min_fb_count, inst->min_src_count);
	}

	/*
	 * SET_PARAM + OPT_CUSTOM_GOP (UND-CUSTOM-GOP):
	 * Must follow INIT_SEQ when gopPresetIdx=0.  SDK wave4.c:912-970.
	 * The firmware may return updated min_fb/min_src counts after this
	 * command; we take the maximum of both.
	 */
	if (!ret && inst->custom_gop_enable) {
		/*
		 * Built-in IPPP default: gop_size=1, single P-frame entry.
		 * poc_offset=1, refPocL0=-1 (previous frame), temporal_id=0.
		 * This produces identical output to gopPresetIdx=2 and serves
		 * as a regression baseline.  SDK pic param bit layout:
		 *   picType[1:0] | pocOffset<<2 | picQp<<6
		 *   | refPocL0<<14 (5-bit signed &0x1F) | refPocL1<<19
		 *   | temporalId<<24
		 */
		static const struct wave420l_custom_gop_param ippp_default = {
			.gop_size = 1,
			.use_derive_lambda = 1,
			.pics = {
				{
					/* P-frame: poc=1, qp=0 (seq QP), tid=0,
					 * refL0=-1 (previous), refL1=0 (unused) */
					.pic_type   = 1,
					.poc_offset = 1,
					.pic_qp     = 0,
					.temporal_id = 0,
					.ref_poc_l0 = -1,
					.ref_poc_l1 = 0,
				},
			},
		};
		const struct wave420l_custom_gop_param *p =
			(inst->custom_gop.gop_size > 0) ? &inst->custom_gop
							 : &ippp_default;
		int i;
		u32 min_fb, min_src;

		wave420l_write(dev, W4_CMD_ENC_SET_PARAM_OPTION, W4_OPT_CUSTOM_GOP);
		/* Enable all custom GOP parameters */
		wave420l_write(dev, W4_CMD_ENC_SET_CUSTOM_GOP_ENABLE,
			       0xFFFFFFFF);
		/* gop_size[3:0] | useDeriveLambdaWeight[4] */
		wave420l_write(dev, W4_CMD_ENC_CUSTOM_GOP_PARAM,
			       (p->gop_size & 0xF) |
			       ((p->use_derive_lambda ? 1 : 0) << 4));

		for (i = 0; i < p->gop_size; i++) {
			const struct wave420l_custom_gop_pic *pic = &p->pics[i];
			u32 pv;

			pv = (pic->pic_type & 0x3) |
			     ((pic->poc_offset & 0x3F) << 2) |
			     ((pic->pic_qp & 0x3F) << 6) |
			     (((u32)(pic->ref_poc_l0) & 0x1F) << 14) |
			     (((u32)(pic->ref_poc_l1) & 0x1F) << 19) |
			     ((pic->temporal_id & 0x7) << 24);
			wave420l_write(dev,
				       W4_CMD_ENC_CUSTOM_GOP_PIC_PARAM_0 + i * 4,
				       pv);
			/* Lambda: 0 = use firmware-derived lambda */
			wave420l_write(dev,
				       W4_CMD_ENC_CUSTOM_GOP_PIC_LAMBDA_0 + i * 4,
				       0);
		}
		/* Zero out unused entries */
		for (; i < WAVE420L_MAX_CUSTOM_GOP_SIZE; i++) {
			wave420l_write(dev,
				       W4_CMD_ENC_CUSTOM_GOP_PIC_PARAM_0 + i * 4,
				       0);
			wave420l_write(dev,
				       W4_CMD_ENC_CUSTOM_GOP_PIC_LAMBDA_0 + i * 4,
				       0);
		}

		dev_info(dev->dev, "CUSTOM_GOP: gop_size=%u derive_lambda=%u\n",
			 p->gop_size, p->use_derive_lambda);

		wave420l_issue_command(dev, inst->id, W4_CMD_SET_PARAM);
		ret = wave420l_wait_busy(dev);

		if (!ret && wave420l_read(dev, W4_RET_SUCCESS) == 0) {
			dev_err(dev->dev, "SET_PARAM+CUSTOM_GOP failed: 0x%x\n",
				wave420l_read(dev, W4_RET_FAIL_REASON));
			ret = -EIO;
		}

		if (!ret) {
			/* Take the maximum of INIT_SEQ and CUSTOM_GOP buffer counts */
			min_fb = wave420l_read(dev, W4_RET_ENC_MIN_FB_NUM);
			min_src = wave420l_read(dev, W4_RET_ENC_MIN_SRC_BUF_NUM);
			if (min_fb > inst->min_fb_count)
				inst->min_fb_count = min_fb;
			if (min_src > inst->min_src_count)
				inst->min_src_count = min_src;
			dev_dbg(dev->dev,
				"CUSTOM_GOP OK: min_fb=%u min_src=%u\n",
				inst->min_fb_count, inst->min_src_count);
		}
	}

	mutex_unlock(&dev->hw_lock);

	return ret;
}

/*
 * Set VUI (Video Usability Information) parameters via SET_PARAM with OPT_VUI.
 *
 * VUI carries colour space signalling (BT.709, BT.2020), transfer function
 * (SDR, PQ/ST2084), aspect ratio, and video signal type.  The firmware
 * encodes this into the SPS VUI section of the HEVC bitstream.
 *
 * Must be called after INIT_SEQ (OPT_COMMON) and before the first ENC_PIC.
 * The register offsets overlap with the OPT_COMMON set; the firmware
 * distinguishes them by the SET_PARAM_OPTION value.
 *
 * vuiParamFlags bitmask (verified by register probing, offset +3 from
 * HEVC SPS VUI syntax order):
 *   BIT(3): aspect_ratio_info_present_flag
 *   BIT(4): overscan_info_present_flag
 *   BIT(5): video_signal_type_present_flag
 *   BIT(6): colour_description_present_flag
 *   BIT(7): chroma_loc_info_present_flag
 *
 * videoSignal register packing (verified by register probing):
 *   [2:0]   video_format (3 bits, 5=unspecified)
 *   [3]     video_full_range_flag
 *   [11:4]  colour_primaries (8 bits, ITU-T H.273)
 *   [19:12] transfer_characteristics (8 bits, ITU-T H.273)
 *   [27:20] matrix_coefficients (8 bits, ITU-T H.273)
 */
int wave420l_hw_set_vui(struct wave420l_instance *inst)
{
	struct wave420l_dev *dev = inst->dev;
	u32 video_signal;
	u32 vui_flags;
	bool has_colour;
	int ret;

	has_colour = inst->colour_primaries || inst->transfer_characteristics ||
		     inst->matrix_coefficients;

	/* If no VUI parameters are configured, skip the SET_PARAM call */
	if (!has_colour && !inst->video_full_range)
		return 0;

	mutex_lock(&dev->hw_lock);

	/* Required buffer setup before any SET_PARAM command */
	wave420l_write(dev, W4_BS_START_ADDR, inst->bs_buf.dma);
	wave420l_write(dev, W4_BS_SIZE, inst->bs_buf.size);

	wave420l_write(dev, W4_ADDR_WORK_BASE, inst->work_buf.dma);
	wave420l_write(dev, W4_WORK_SIZE, inst->work_buf.size);
	wave420l_write(dev, W4_WORK_PARAM, 0);

	wave420l_write(dev, W4_ADDR_TEMP_BASE, inst->temp_buf.dma);
	wave420l_write(dev, W4_TEMP_SIZE, inst->temp_buf.size);
	wave420l_write(dev, W4_TEMP_PARAM, 0);

	wave420l_write(dev, W4_CMD_ENC_SET_PARAM_OPTION, W4_OPT_VUI);

	/*
	 * vuiParamFlags: bitmask controlling which VUI sub-sections are present.
	 * Follows HEVC SPS VUI flag order:
	 *   BIT(0): aspect_ratio_info_present_flag
	 *   BIT(1): overscan_info_present_flag
	 *   BIT(2): video_signal_type_present_flag
	 *   BIT(3): chroma_loc_info_present_flag
	 *   BIT(4): neutral_chroma_indication_flag
	 *   BIT(5): field_seq_flag
	 *   BIT(6): frame_field_info_present_flag
	 *   BIT(7): default_display_window_flag
	 *   BIT(8): vui_timing_info_present_flag
	 */
	/*
	 * vuiParamFlags: enable video_signal_type and colour_description
	 * in SPS VUI.  BIT(5) = video_signal_type_present_flag,
	 * BIT(6) = colour_description_present_flag.
	 */
	vui_flags = BIT(5) | BIT(6);
	wave420l_write(dev, W4_CMD_ENC_VUI_PARAM_FLAGS, vui_flags);
	wave420l_write(dev, W4_CMD_ENC_VUI_ASPECT_RATIO_IDC, 0);
	wave420l_write(dev, W4_CMD_ENC_VUI_SAR_SIZE, 0);
	wave420l_write(dev, W4_CMD_ENC_VUI_OVERSCAN_APPROPRIATE, 0);

	/*
	 * Pack video signal type register.
	 * Layout derived from the SDK decoder parsing (vpuapifunc.c):
	 *   [7:0]   colour_primaries (8 bits)
	 *   [15:8]  transfer_characteristics (8 bits)
	 *   [23:16] matrix_coefficients (8 bits) -- possibly, or same slot
	 *   [25:24] reserved
	 *   [26]    video_full_range_flag
	 *   [29:27] video_format (3 bits)
	 *   [30]    video_signal_type_present_flag
	 *   [31]    colour_description_present_flag
	 *
	 * Alternatively, the encoder may use a different layout.
	 * Try the decoder-side layout first:
	 *   [15:0]  reserved / colour data
	 *   [23:16] colour_primaries
	 *   [25:24] transfer / matrix (packed elsewhere?)
	 *   [26]    video_full_range_flag
	 *   [29:27] video_format
	 *   [30]    video_signal_type_present_flag
	 *
	 * The encoder SDK writes this as an opaque u32 from config.
	 * Since no sample config sets a non-zero value, we must guess.
	 * Use the most natural HEVC-spec-order packing:
	 *   [2:0]   video_format
	 *   [3]     video_full_range_flag
	 *   [4]     colour_description_present_flag
	 *   [12:5]  colour_primaries
	 *   [20:13] transfer_characteristics
	 *   [28:21] matrix_coefficients
	 *   [29]    video_signal_type_present_flag
	 */
	/*
	 * Pack video signal type register.
	 * vuiParamFlags BIT(5) controls video_signal_type_present_flag.
	 * vuiParamFlags BIT(6) controls colour_description_present_flag.
	 * The videoSignal register layout (verified by register probing):
	 *   [2:0]   video_format (3 bits, 5=unspecified)
	 *   [3]     video_full_range_flag
	 *   [11:4]  colour_primaries (8 bits)
	 *   [19:12] transfer_characteristics (8 bits)
	 *   [27:20] matrix_coefficients (8 bits)
	 */
	video_signal = (inst->video_format & 0x7);
	video_signal |= (inst->video_full_range ? 1 : 0) << 3;
	video_signal |= ((u32)inst->colour_primaries & 0xFF) << 4;
	video_signal |= ((u32)inst->transfer_characteristics & 0xFF) << 12;
	video_signal |= ((u32)inst->matrix_coefficients & 0xFF) << 20;

	wave420l_write(dev, W4_CMD_ENC_VUI_VIDEO_SIGNAL, video_signal);
	wave420l_write(dev, W4_CMD_ENC_VUI_CHROMA_SAMPLE_LOC, 0);
	wave420l_write(dev, W4_CMD_ENC_VUI_DISP_WIN_LEFT_RIGHT, 0);
	wave420l_write(dev, W4_CMD_ENC_VUI_DISP_WIN_TOP_BOT, 0);
	wave420l_write(dev, W4_CMD_ENC_VUI_HRD_RBSP_PARAM_FLAG, 0);

	dev_dbg(dev->dev,
		"SET_PARAM VUI: inst=%d flags=0x%x video_signal=0x%x "
		"(fmt=%u range=%u cp=%u tc=%u mc=%u)\n",
		inst->id, vui_flags, video_signal,
		inst->video_format, inst->video_full_range ? 1 : 0,
		inst->colour_primaries, inst->transfer_characteristics,
		inst->matrix_coefficients);

	wave420l_issue_command(dev, inst->id, W4_CMD_SET_PARAM);
	ret = wave420l_wait_busy(dev);

	if (!ret && wave420l_read(dev, W4_RET_SUCCESS) == 0) {
		dev_err(dev->dev, "SET_PARAM VUI failed: 0x%x\n",
			wave420l_read(dev, W4_RET_FAIL_REASON));
		ret = -EIO;
	}

	if (!ret)
		dev_dbg(dev->dev, "SET_PARAM VUI OK: inst=%d\n", inst->id);

	mutex_unlock(&dev->hw_lock);

	return ret;
}

/*
 * Buffer size macros matching the SDK:
 *   WAVE4_ENC_HEVC_MVCOL_BUF_SIZE(w,h) = ((w+63)/64)*((h+63)/64)*128
 *   WAVE4_FBC_LUMA_TABLE_SIZE(w,h)      = ((h+15)/16)*((w+255)/256)*128
 *   WAVE4_FBC_CHROMA_TABLE_SIZE(w,h)    = ((h+15)/16)*((w/2+255)/256)*128
 *   WAVE4_SUBSAMPLED_ONE_SIZE(w,h)      = (((w/4+15)&~15))*((h/4+7)&~7))
 */
#define W4_MVCOL_SIZE(w, h) \
	(ALIGN((w), 64) / 64 * (ALIGN((h), 64) / 64) * 128)
#define W4_FBC_Y_TBL_SIZE(w, h) \
	(((h) + 15) / 16 * (((w) + 255) / 256) * 128)
#define W4_FBC_C_TBL_SIZE(w, h) \
	(((h) + 15) / 16 * (((w) / 2 + 255) / 256) * 128)
#define W4_SUBSAMP_SIZE(w, h) \
	((((w) / 4 + 15) & ~15) * ((((h) / 4) + 7) & ~7))

int wave420l_hw_set_fb(struct wave420l_instance *inst)
{
	struct wave420l_dev *dev = inst->dev;
	u32 buf_w, buf_h, fb_size;
	u32 sub_sam_size, mv_col_size, fbc_y_tbl_size, fbc_c_tbl_size;
	u32 luma_stride, chroma_stride;
	u32 endian;
	int i, ret;

	buf_w = ALIGN(inst->src_fmt.width, 8);
	buf_h = ALIGN(inst->src_fmt.height, 8);

	/*
	 * Rotation/mirror requires 32-byte aligned dimensions.
	 * 90/270 degree rotation swaps width and height for internal buffers.
	 * (SDK: Wave4VpuEncRegisterFramebuffer)
	 */
	if (inst->rotation == 90 || inst->rotation == 270) {
		buf_w = ALIGN(inst->src_fmt.height, 32);
		buf_h = ALIGN(inst->src_fmt.width, 32);
	} else if (inst->rotation != 0 || inst->hflip || inst->vflip) {
		buf_w = ALIGN(inst->src_fmt.width, 32);
		buf_h = ALIGN(inst->src_fmt.height, 32);
	}

	inst->fb_count = inst->min_fb_count;
	if (inst->fb_count == 0)
		inst->fb_count = 2;

	/*
	 * COMPRESSED_FRAME_MAP: reconstruction buffers use FBC format.
	 * SDK requires this for Wave420L encoder (LINEAR returns NOT_SUPPORTED).
	 * Additional buffers needed: MV col, FBC Y table, FBC C table, sub-sampled.
	 *
	 * FBC strides (8-bit, for W4_FBC_STRIDE register):
	 *   lumaStride = ALIGN32(ALIGN16(width) * 4)
	 *   chromaStride = ALIGN32(ALIGN16(width/2) * 4)
	 *
	 * But frame buffer layout uses the RAW stride (ALIGN32(width)):
	 *   sizeLuma = raw_stride * height
	 *   sizeChroma = ALIGN16(raw_stride/2) * height / 2
	 *   bufCb = bufY + sizeLuma
	 */
	/*
	 * FBC strides: when rotation/mirror is active (not the degenerate
	 * 180+HOR_VER case), the SDK uses ALIGN32(bufWidth) for stride
	 * calculation instead of ALIGN16(width).
	 */
	if ((inst->rotation != 0 || inst->hflip || inst->vflip) &&
	    !(inst->rotation == 180 && inst->hflip && inst->vflip)) {
		luma_stride = ALIGN(ALIGN(buf_w, 32) * 4, 32);
		chroma_stride = ALIGN(ALIGN(buf_w / 2, 16) * 4, 32);
	} else {
		luma_stride = ALIGN(ALIGN(buf_w, 16) * 4, 32);
		chroma_stride = ALIGN(ALIGN(buf_w / 2, 16) * 4, 32);
	}

	/*
	 * Raw stride for buffer layout and COMMON_PIC_INFO.
	 * SDK: CalcLumaSize(COMPRESSED) = stride * height
	 * SDK: CalcChromaSize(COMPRESSED, 420) = ALIGN16(stride/2) * height / 2
	 * SDK: ProductCalculateFrameBufSize = sizeLuma + sizeChroma * 2
	 *
	 * For 4:2:0 with cbcrInterleave (COMPRESSED always uses interleave):
	 *   total = stride * height * 3/2
	 */
	inst->raw_stride = ALIGN(buf_w, 32);
	fb_size = inst->raw_stride * buf_h * 3 / 2;

	mv_col_size = ALIGN(W4_MVCOL_SIZE(buf_w, buf_h), 16);
	fbc_y_tbl_size = ALIGN(W4_FBC_Y_TBL_SIZE(buf_w, buf_h), 16);
	fbc_c_tbl_size = ALIGN(W4_FBC_C_TBL_SIZE(buf_w, buf_h), 16);
	sub_sam_size = W4_SUBSAMP_SIZE(buf_w, buf_h);

	/* Allocate MV col buffer (all frames contiguous) */
	inst->mv_buf.size = ALIGN(mv_col_size * inst->fb_count, PAGE_SIZE) + PAGE_SIZE;
	inst->mv_buf.virt = dma_alloc_coherent(dev->dev, inst->mv_buf.size,
					       &inst->mv_buf.dma, GFP_KERNEL);
	if (!inst->mv_buf.virt)
		return -ENOMEM;

	/* Allocate FBC Y table buffer */
	inst->fbc_y_tbl_buf.size = ALIGN(fbc_y_tbl_size * inst->fb_count, PAGE_SIZE) + PAGE_SIZE;
	inst->fbc_y_tbl_buf.virt = dma_alloc_coherent(dev->dev, inst->fbc_y_tbl_buf.size,
						       &inst->fbc_y_tbl_buf.dma, GFP_KERNEL);
	if (!inst->fbc_y_tbl_buf.virt) {
		ret = -ENOMEM;
		goto err_free_mv;
	}

	/* Allocate FBC C table buffer */
	inst->fbc_c_tbl_buf.size = ALIGN(fbc_c_tbl_size * inst->fb_count, PAGE_SIZE) + PAGE_SIZE;
	inst->fbc_c_tbl_buf.virt = dma_alloc_coherent(dev->dev, inst->fbc_c_tbl_buf.size,
						       &inst->fbc_c_tbl_buf.dma, GFP_KERNEL);
	if (!inst->fbc_c_tbl_buf.virt) {
		ret = -ENOMEM;
		goto err_free_fbc_y;
	}

	/* Allocate sub-sampled buffer */
	inst->sub_sam_buf.size = ALIGN(sub_sam_size * inst->fb_count, PAGE_SIZE) + PAGE_SIZE;
	inst->sub_sam_buf.virt = dma_alloc_coherent(dev->dev, inst->sub_sam_buf.size,
						    &inst->sub_sam_buf.dma, GFP_KERNEL);
	if (!inst->sub_sam_buf.virt) {
		ret = -ENOMEM;
		goto err_free_fbc_c;
	}

	/* Allocate reconstruction frame buffers */
	inst->fb_bufs = kcalloc(inst->fb_count, sizeof(*inst->fb_bufs), GFP_KERNEL);
	if (!inst->fb_bufs) {
		ret = -ENOMEM;
		goto err_free_subsam;
	}

	for (i = 0; i < inst->fb_count; i++) {
		inst->fb_bufs[i].size = fb_size;
		inst->fb_bufs[i].virt = dma_alloc_coherent(dev->dev, fb_size,
							    &inst->fb_bufs[i].dma,
							    GFP_KERNEL);
		if (!inst->fb_bufs[i].virt) {
			ret = -ENOMEM;
			goto err_free_fb_bufs;
		}
	}

	dev_info(dev->dev, "SET_FB: COMPRESSED %u recon bufs, fb=%u, "
		 "mv=%u fbc_y=%u fbc_c=%u subsam=%u raw_stride=%u fbc=%u/%u\n",
		 inst->fb_count, fb_size, mv_col_size, fbc_y_tbl_size,
		 fbc_c_tbl_size, sub_sam_size, inst->raw_stride,
		 luma_stride, chroma_stride);

	mutex_lock(&dev->hw_lock);

	/* Sub-sampled buffer base and per-frame size */
	wave420l_write(dev, W4_ADDR_SUB_SAMPLED_FB_BASE, inst->sub_sam_buf.dma);
	wave420l_write(dev, W4_SUB_SAMPLED_ONE_FB_SIZE, sub_sam_size);

	/* Picture size */
	wave420l_write(dev, W4_PIC_SIZE, (buf_w << 16) | buf_h);

	/* FBC stride: lumaStride[31:16] | chromaStride[15:0] */
	wave420l_write(dev, W4_FBC_STRIDE, (luma_stride << 16) | chroma_stride);

	/*
	 * SDK endian conversion for 128-bit bus:
	 *   vdi_convert_endian(LE) = 0 on little-endian ARM
	 *   bsEndian = (~0) & 0xf = 0xf
	 * But the SDK stores this in SFB_OPTION bits[31:16].
	 */
	endian = 0xf;

	/*
	 * COMMON_PIC_INFO for COMPRESSED_FRAME_MAP:
	 *   bit[29] = nv21 (0)
	 *   bit[28] = LINEAR flag (0 = compressed)
	 *   bit[27:24] = axiID (0)
	 *   bit[23:20] = yuvFormat (0 = 420)
	 *   bit[19:16] = cbcrInterleave (0 for compressed)
	 *   bit[15:0]  = stride * bpp
	 *
	 * SDK: stride = pEncInfo->stride (raw stride, ALIGN32(width))
	 */
	wave420l_write(dev, W4_COMMON_PIC_INFO, inst->raw_stride);

	/*
	 * SFB_OPTION: endian[31:16] | last_batch[4] | first_batch[3]
	 * SDK: regVal = (endian<<16) | (j==q-1)<<4 | ((j==0)<<3)
	 * Single batch: first=1, last=1
	 */
	wave420l_write(dev, W4_SFB_OPTION, (endian << 16) | BIT(4) | BIT(3));

	/*
	 * SET_FB_NUM: (startNo << 8) | endNo
	 * SDK: VpuWriteReg(W4_SET_FB_NUM, (startNo<<8)|endNo)
	 * For single batch of N buffers: startNo=0, endNo=N-1
	 */
	wave420l_write(dev, W4_SET_FB_NUM, (0 << 8) | (inst->fb_count - 1));

	/* Register reconstruction buffer addresses (COMPRESSED_FRAME_MAP) */
	for (i = 0; i < inst->fb_count && i < 8; i++) {
		dma_addr_t base = inst->fb_bufs[i].dma;
		u32 size_luma = inst->raw_stride * buf_h;

		wave420l_write(dev, W4_ADDR_LUMA_BASE0 + (i << 4), base);
		wave420l_write(dev, W4_ADDR_CB_BASE0 + (i << 4), base + size_luma);
		/* FBC Y offset table */
		wave420l_write(dev, W4_ADDR_FBC_Y_OFFSET0 + (i << 4),
			       inst->fbc_y_tbl_buf.dma + i * fbc_y_tbl_size);
		/* FBC C offset table */
		wave420l_write(dev, W4_ADDR_FBC_C_OFFSET0 + (i << 4),
			       inst->fbc_c_tbl_buf.dma + i * fbc_c_tbl_size);
		/* MV col buffer */
		wave420l_write(dev, W4_ADDR_MV_COL0 + (i << 2),
			       inst->mv_buf.dma + i * mv_col_size);
	}

	/* Work buffer must be set before SET_FRAMEBUF */
	wave420l_write(dev, W4_ADDR_WORK_BASE, inst->work_buf.dma);
	wave420l_write(dev, W4_WORK_SIZE, inst->work_buf.size);
	wave420l_write(dev, W4_WORK_PARAM, 0);

	wave420l_issue_command(dev, inst->id, W4_CMD_SET_FB);
	ret = wave420l_wait_busy(dev);

	if (!ret && wave420l_read(dev, W4_RET_SUCCESS) == 0) {
		dev_err(dev->dev, "SET_FB failed: 0x%x\n",
			wave420l_read(dev, W4_RET_FAIL_REASON));
		ret = -EIO;
	}

	mutex_unlock(&dev->hw_lock);

	if (ret)
		goto err_free_fb_bufs;

	return 0;

err_free_fb_bufs:
	for (i = 0; i < inst->fb_count; i++) {
		if (inst->fb_bufs[i].virt)
			dma_free_coherent(dev->dev, inst->fb_bufs[i].size,
					  inst->fb_bufs[i].virt,
					  inst->fb_bufs[i].dma);
	}
	kfree(inst->fb_bufs);
	inst->fb_bufs = NULL;
err_free_subsam:
	dma_free_coherent(dev->dev, inst->sub_sam_buf.size,
			  inst->sub_sam_buf.virt, inst->sub_sam_buf.dma);
	inst->sub_sam_buf.virt = NULL;
err_free_fbc_c:
	dma_free_coherent(dev->dev, inst->fbc_c_tbl_buf.size,
			  inst->fbc_c_tbl_buf.virt, inst->fbc_c_tbl_buf.dma);
	inst->fbc_c_tbl_buf.virt = NULL;
err_free_fbc_y:
	dma_free_coherent(dev->dev, inst->fbc_y_tbl_buf.size,
			  inst->fbc_y_tbl_buf.virt, inst->fbc_y_tbl_buf.dma);
	inst->fbc_y_tbl_buf.virt = NULL;
err_free_mv:
	dma_free_coherent(dev->dev, inst->mv_buf.size,
			  inst->mv_buf.virt, inst->mv_buf.dma);
	inst->mv_buf.virt = NULL;
	return ret;
}

/* ------------------------------------------------------------------ */
/* VPU Sleep/Wake (firmware-level power saving)                       */
/* ------------------------------------------------------------------ */

/*
 * Put VPU firmware into sleep state.  The firmware saves its internal
 * state and halts the processor.  The host must save the stack pointer
 * from W4_RET_CUR_SP and restore it on wakeup.
 *
 * Caller must hold vpu_lock.  No instances may be actively encoding.
 */
int wave420l_hw_sleep_vpu(struct wave420l_dev *dev)
{
	int ret;

	if (!dev->vpu_initialized || dev->vpu_sleeping)
		return 0;

	mutex_lock(&dev->hw_lock);

	/* Wait for any in-flight command to complete */
	ret = wave420l_wait_busy(dev);
	if (ret) {
		dev_err(dev->dev, "SLEEP_VPU: VPU busy before sleep\n");
		mutex_unlock(&dev->hw_lock);
		return ret;
	}

	/* Issue SLEEP_VPU command (instance 0, codec mode doesn't matter) */
	wave420l_issue_command(dev, 0, W4_CMD_SLEEP_VPU);
	ret = wave420l_wait_busy(dev);
	if (ret) {
		dev_err(dev->dev, "SLEEP_VPU timeout\n");
		mutex_unlock(&dev->hw_lock);
		return ret;
	}

	if (wave420l_read(dev, W4_RET_SUCCESS) == 0) {
		dev_err(dev->dev, "SLEEP_VPU failed: 0x%x\n",
			wave420l_read(dev, W4_RET_FAIL_REASON));
		mutex_unlock(&dev->hw_lock);
		return -EIO;
	}

	/* Save stack pointer for restore on wakeup */
	dev->saved_sp = wave420l_read(dev, W4_RET_CUR_SP);
	dev->vpu_sleeping = true;

	dev_dbg(dev->dev, "VPU sleeping, SP=0x%x\n", dev->saved_sp);

	mutex_unlock(&dev->hw_lock);
	return 0;
}

/*
 * Wake VPU from firmware sleep state.  This performs a full VPU
 * re-initialization (reset, remap, INIT_VPU) which restores the
 * firmware's execution context including all instance state.
 *
 * Caller must hold vpu_lock.
 */
int wave420l_hw_wakeup_vpu(struct wave420l_dev *dev)
{
	u32 remap_size, regval;
	u32 code_base, code_size;
	int ret;

	if (!dev->vpu_sleeping)
		return 0;

	mutex_lock(&dev->hw_lock);

	code_base = dev->fw_buf.dma;
	code_size = WAVE420L_CODE_BUF_SIZE & ~0xFFF;

	/* Reset VPU configuration */
	wave420l_write(dev, W4_PO_CONF, 0);

	/* Assert reset on all blocks */
	wave420l_write(dev, W4_VPU_RESET_REQ, 0x7FFFFFF);

	/* Wait for reset to complete */
	ret = readl_poll_timeout(dev->reg_base + W4_VPU_RESET_STATUS,
				 regval, regval == 0, 10, 100000);
	if (ret) {
		dev_err(dev->dev, "WAKEUP_VPU reset timeout\n");
		wave420l_write(dev, W4_VPU_RESET_REQ, 0);
		mutex_unlock(&dev->hw_lock);
		return ret;
	}

	/* Release reset */
	wave420l_write(dev, W4_VPU_RESET_REQ, 0);

	/* Setup memory remap (map 1MB code buffer to VPU address 0) */
	remap_size = (code_size >> 12) & 0x1FF;
	wave420l_write(dev, W4_VPU_REMAP_CTRL,
		       0x80000000 | (0 << 16) | (0 << 12) | BIT(11) | remap_size);
	wave420l_write(dev, W4_VPU_REMAP_VADDR, 0x00000000);
	wave420l_write(dev, W4_VPU_REMAP_PADDR, code_base);

	wave420l_write(dev, W4_ADDR_CODE_BASE, code_base);
	wave420l_write(dev, W4_CODE_SIZE, code_size);
	wave420l_write(dev, W4_CODE_PARAM, 0);

	wave420l_write(dev, W4_TIMEOUT_CNT, 0xFFFFFFFF);
	wave420l_write(dev, W4_HW_OPTION, 0);

	/* Enable interrupts (same as init_vpu) */
	regval = BIT(W4_INT_INIT_VPU) | BIT(W4_INT_SET_PARAM) |
		 BIT(W4_INT_FINI_SEQ) | BIT(W4_INT_ENC_PIC) |
		 BIT(W4_INT_SET_FRAMEBUF) | BIT(W4_INT_SLEEP_VPU) |
		 BIT(W4_INT_WAKEUP_VPU) | BIT(W4_INT_CREATE_INSTANCE);
	wave420l_write(dev, W4_VPU_VINT_ENABLE, regval);

	/* Issue INIT_VPU and start processor -- firmware resumes from sleep */
	wave420l_issue_command(dev, 0, W4_CMD_INIT_VPU);
	wave420l_write(dev, W4_VPU_REMAP_CORE_START, 1);

	ret = wave420l_wait_busy(dev);
	if (ret) {
		dev_err(dev->dev, "WAKEUP_VPU timeout\n");
		mutex_unlock(&dev->hw_lock);
		return ret;
	}

	if (wave420l_read(dev, W4_RET_SUCCESS) == 0) {
		dev_err(dev->dev, "WAKEUP_VPU failed: 0x%x\n",
			wave420l_read(dev, W4_RET_FAIL_REASON));
		mutex_unlock(&dev->hw_lock);
		return -EIO;
	}

	dev->vpu_sleeping = false;
	dev_dbg(dev->dev, "VPU woken up\n");

	mutex_unlock(&dev->hw_lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Flush Instance (force output of buffered frames)                   */
/* ------------------------------------------------------------------ */

/*
 * Write all ENC_PIC flush registers and issue the command.
 * Called with dev->hw_lock held.  Releases hw_lock for the wait,
 * then re-acquires it before returning.
 *
 * Returns 0 on success, -ETIMEDOUT if firmware did not respond.
 * out_bytes is set to 0 when firmware reports no more buffered frames,
 * or to the byte count of the flushed frame when output is present.
 */
static int wave420l_flush_one(struct wave420l_instance *inst, u32 *out_bytes)
{
	struct wave420l_dev *dev = inst->dev;
	int ret;

	/* Set up buffers same as normal ENC_PIC */
	wave420l_write(dev, W4_BS_START_ADDR, inst->bs_buf.dma);
	wave420l_write(dev, W4_BS_SIZE, inst->bs_buf.size);
	wave420l_write(dev, W4_BS_RD_PTR, inst->bs_buf.dma);
	wave420l_write(dev, W4_BS_WR_PTR, inst->bs_buf.dma);
	wave420l_write(dev, W4_BS_PARAM, 0xf);

	/* Secondary AXI (disabled) */
	wave420l_write(dev, W4_ADDR_SEC_AXI, 0);
	wave420l_write(dev, W4_SEC_AXI_SIZE, 0);
	wave420l_write(dev, W4_USE_SEC_AXI, 0);

	/* Work buffer */
	wave420l_write(dev, W4_ADDR_WORK_BASE, inst->work_buf.dma);
	wave420l_write(dev, W4_WORK_SIZE, inst->work_buf.size);
	wave420l_write(dev, W4_WORK_PARAM, 0);

	/* Temp buffer */
	wave420l_write(dev, W4_ADDR_TEMP_BASE, inst->temp_buf.dma);
	wave420l_write(dev, W4_TEMP_SIZE, inst->temp_buf.size);
	wave420l_write(dev, W4_TEMP_PARAM, 0);

	/* Report buffer (not used) */
	wave420l_write(dev, W4_CMD_ENC_ADDR_REPORT_BASE, 0);
	wave420l_write(dev, W4_CMD_ENC_REPORT_SIZE, 0);
	wave420l_write(dev, W4_CMD_ENC_REPORT_PARAM, 0);

	/* Signal end-of-stream: srcPicIdx = 0xFFFFFFFF */
	wave420l_write(dev, W4_CMD_ENC_CODE_OPTION, BIT(0) | BIT(1));
	wave420l_write(dev, W4_CMD_ENC_PIC_PARAM, 0);
	wave420l_write(dev, W4_CMD_ENC_SRC_PIC_IDX, 0xFFFFFFFF);

	/* Zero out remaining ENC_PIC registers */
	wave420l_write(dev, W4_CMD_ENC_SRC_ADDR_Y, 0);
	wave420l_write(dev, W4_CMD_ENC_SRC_ADDR_U, 0);
	wave420l_write(dev, W4_CMD_ENC_SRC_ADDR_V, 0);
	wave420l_write(dev, W4_CMD_ENC_SRC_STRIDE, 0);
	wave420l_write(dev, W4_CMD_ENC_SRC_FORMAT, 0);
	wave420l_write(dev, W4_CMD_ENC_SRC_TIMESTAMP_LOW, 0);
	wave420l_write(dev, W4_CMD_ENC_SRC_TIMESTAMP_HIGH, 0);
	wave420l_write(dev, W4_CMD_ENC_LONGTERM_PIC, 0);
	wave420l_write(dev, W4_CMD_ENC_SUB_FRAME_SYNC_CONFIG, 0);
	wave420l_write(dev, W4_CMD_ENC_CTU_OPT_PARAM, 0);
	wave420l_write(dev, W4_CMD_ENC_ROI_ADDR_CTU_MAP, 0);
	wave420l_write(dev, W4_CMD_ENC_CTU_QP_MAP_ADDR, 0);
	wave420l_write(dev, W4_CMD_ENC_PREFIX_SEI_NAL_ADDR, 0);
	wave420l_write(dev, W4_CMD_ENC_PREFIX_SEI_INFO, 0);
	wave420l_write(dev, W4_CMD_ENC_SUFFIX_SEI_NAL_ADDR, 0);
	wave420l_write(dev, W4_CMD_ENC_SUFFIX_SEI_INFO, 0);

	/* Clear pending interrupts */
	wave420l_write(dev, W4_VPU_VINT_REASON_CLR,
		       wave420l_read(dev, W4_VPU_INT_REASON));
	wave420l_write(dev, W4_VPU_VINT_CLEAR, 1);

	reinit_completion(&inst->irq_done);
	dev->active_inst = inst;

	wave420l_issue_command(dev, inst->id, W4_CMD_ENC_PIC);

	/* Release lock while waiting: other IRQs must not be blocked */
	mutex_unlock(&dev->hw_lock);

	ret = wait_for_completion_timeout(&inst->irq_done,
					  msecs_to_jiffies(8000));

	mutex_lock(&dev->hw_lock);
	dev->active_inst = NULL;

	if (ret == 0) {
		dev_err(dev->dev, "FLUSH ENC_PIC timeout inst=%d\n", inst->id);
		*out_bytes = 0;
		return -ETIMEDOUT;
	}

	if (wave420l_read(dev, W4_RET_SUCCESS) == 0) {
		/* Firmware reports no more buffered frames (expected for IPP) */
		dev_dbg(dev->dev, "FLUSH ENC_PIC: inst=%d no buffered frames "
			"(reason=0x%x)\n", inst->id,
			wave420l_read(dev, W4_RET_FAIL_REASON));
		*out_bytes = 0;
		return 0;
	}

	*out_bytes = wave420l_read(dev, W4_RET_ENC_PIC_BYTE);
	dev_dbg(dev->dev, "FLUSH ENC_PIC OK: inst=%d out_bytes=%u\n",
		inst->id, *out_bytes);
	return 0;
}

/*
 * Flush encoder instance: signals "no more source frames" to firmware
 * by issuing ENC_PIC with srcPicIdx=0xFFFFFFFF (srcEndFlag).  This
 * forces output of any buffered frames in hierarchical GOP with B-frames.
 *
 * For B-frame GOPs (bframe_delay > 0), loops up to bframe_delay+1 times
 * to drain the full reorder pipeline, delivering each flushed frame to a
 * V4L2 capture buffer.  The final flushed frame is marked with
 * V4L2_BUF_FLAG_LAST and an EOS event is queued on the file handle.
 *
 * SDK reference: wave4.c Wave4VpuEncOnePicture() -- when srcEndFlag==1,
 * sets W4_CMD_ENC_SRC_PIC_IDX to 0xFFFFFFFF instead of the source index.
 * The firmware then outputs remaining buffered frames.
 *
 * Called from wave420l_encoder_cmd() after v4l2_m2m_ioctl_encoder_cmd()
 * has placed the M2M context into drain state.
 */
int wave420l_hw_flush_instance(struct wave420l_instance *inst)
{
	static const struct v4l2_event eos_event = { .type = V4L2_EVENT_EOS };
	struct wave420l_dev *dev = inst->dev;
	int i, ret;
	u32 out_bytes;
	/*
	 * max_cycles: bframe_delay flush cycles that may produce output, plus
	 * one terminal cycle where firmware reports "no more buffered frames".
	 */
	int max_cycles = inst->bframe_delay + 1;
	bool eos_signaled = false;

	/* Resume VPU from runtime suspend if sleeping */
	ret = pm_runtime_resume_and_get(dev->dev);
	if (ret < 0 && ret != -ENOSYS) {
		dev_err(dev->dev, "runtime resume failed: %d\n", ret);
		return ret;
	}

	mutex_lock(&dev->hw_lock);

	for (i = 0; i <= max_cycles; i++) {
		ret = wave420l_flush_one(inst, &out_bytes);
		if (ret == -ETIMEDOUT)
			break;

		if (out_bytes == 0)
			break;  /* firmware says no more buffered frames */

		/* Deliver the flushed B-frame to a V4L2 capture buffer */
		{
			struct vb2_v4l2_buffer *dst_buf =
				v4l2_m2m_dst_buf_remove(inst->fh.m2m_ctx);

			if (dst_buf) {
				void *dst = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);

				if (dst && inst->bs_buf.virt &&
				    out_bytes <= vb2_plane_size(&dst_buf->vb2_buf, 0))
					memcpy(dst, inst->bs_buf.virt, out_bytes);
				vb2_set_plane_payload(&dst_buf->vb2_buf, 0, out_bytes);
				dst_buf->sequence = inst->sequence++;
				dst_buf->flags |= V4L2_BUF_FLAG_BFRAME;
				/*
				 * Mark LAST on the final B-frame flush cycle.
				 * i == max_cycles - 1 means this is the last
				 * cycle that can produce output.
				 */
				if (i == max_cycles - 1) {
					dst_buf->flags |= V4L2_BUF_FLAG_LAST;
					v4l2_event_queue_fh(&inst->fh, &eos_event);
					v4l2_m2m_mark_stopped(inst->fh.m2m_ctx);
					eos_signaled = true;
				}
				v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
				dev_dbg(dev->dev,
					"FLUSH: delivered B-frame %u bytes (cycle %d/%d)\n",
					out_bytes, i, max_cycles - 1);
			} else {
				dev_warn(dev->dev,
					 "FLUSH: no dst buf for B-frame output (%u bytes, cycle %d)\n",
					 out_bytes, i);
			}
		}
	}

	/*
	 * If no B-frames were produced (IPP GOP or all cycles exhausted without
	 * output) and EOS hasn't been signaled by the normal drain path yet,
	 * signal it now so userspace doesn't hang waiting for EOS.
	 */
	if (!eos_signaled && inst->bframe_delay == 0) {
		/*
		 * IPP GOP: no B-frames.  The normal device_run path will signal
		 * EOS when the last source buffer is encoded.  Don't signal
		 * here -- let the existing drain completion logic handle it.
		 */
		dev_dbg(dev->dev, "FLUSH: IPP GOP, no B-frames to drain (inst=%d)\n",
			inst->id);
	}

	mutex_unlock(&dev->hw_lock);
	pm_runtime_mark_last_busy(dev->dev);
	pm_runtime_put_autosuspend(dev->dev);
	return (ret == -ETIMEDOUT) ? ret : 0;
}

/* ------------------------------------------------------------------ */
/* Dynamic Parameter Change                                            */
/* ------------------------------------------------------------------ */

/*
 * Issue SET_PARAM with OPT_COMMON to change encoding parameters mid-stream.
 * The enable_option bitmask tells the firmware which registers contain new
 * values.  Must be called with hw_lock NOT held (we take it internally).
 *
 * SDK reference: Wave4VpuEncParaChange() in wave4.c
 */
int wave420l_hw_set_param_change(struct wave420l_instance *inst)
{
	struct wave420l_dev *dev = inst->dev;
	u32 enable = inst->params_changed;
	u32 rc_param;
	int ret;

	if (!enable)
		return 0;

	mutex_lock(&dev->hw_lock);

	/* BS_PARAM (required by SDK before SET_PARAM) */
	wave420l_write(dev, W4_BS_PARAM, 0xf);

	/* Secondary AXI (disabled) */
	wave420l_write(dev, W4_ADDR_SEC_AXI, 0);
	wave420l_write(dev, W4_SEC_AXI_SIZE, 0);
	wave420l_write(dev, W4_USE_SEC_AXI, 0);

	/* Work buffer */
	wave420l_write(dev, W4_ADDR_WORK_BASE, inst->work_buf.dma);
	wave420l_write(dev, W4_WORK_SIZE, inst->work_buf.size);
	wave420l_write(dev, W4_WORK_PARAM, 0);

	/* Temp buffer */
	wave420l_write(dev, W4_ADDR_TEMP_BASE, inst->temp_buf.dma);
	wave420l_write(dev, W4_TEMP_SIZE, inst->temp_buf.size);
	wave420l_write(dev, W4_TEMP_PARAM, 0);

	/* Report buffer (not used) */
	wave420l_write(dev, W4_CMD_ENC_ADDR_REPORT_BASE, 0);
	wave420l_write(dev, W4_CMD_ENC_REPORT_SIZE, 0);
	wave420l_write(dev, W4_CMD_ENC_REPORT_PARAM, 0);

	/*
	 * OPT_CHANGE_PARAM (0x10): mid-stream parameter update.
	 * Must NOT use OPT_COMMON (0x00) here -- that re-initializes the
	 * firmware's sequence state and breaks implicit VPS/SPS/PPS emission
	 * (KI-092).  SDK: Wave4VpuEncParaChange uses OPT_CHANGE_PARAM.
	 */
	wave420l_write(dev, W4_CMD_ENC_SET_PARAM_OPTION, W4_OPT_CHANGE_PARAM);
	wave420l_write(dev, W4_CMD_ENC_SET_PARAM_ENABLE, enable);

	if (enable & W4_ENC_INTRA_PARAM_CHANGE) {
		u32 intra_param = 1;  /* CRA */

		intra_param |= (inst->init_qp & 0x3F) << 3;
		/* bit 9 = forcedIdrHeaderEnable: leave 0 (KI-092) */
		intra_param |= (inst->gop_size & 0xFFFF) << 16;
		wave420l_write(dev, W4_CMD_ENC_SEQ_INTRA_PARAM, intra_param);
	}

	if (enable & W4_ENC_FRAME_RATE_CHANGE)
		wave420l_write(dev, W4_CMD_ENC_SEQ_FRAME_RATE,
			       inst->frame_rate ? inst->frame_rate : 30);

	if (enable & W4_ENC_RC_PARAM_CHANGE) {
		if (inst->rc_enable && inst->bitrate > 0) {
			rc_param = 1;				/* rcEnable */
			rc_param |= (inst->cu_rc_enable << 1);	/* cuLevelRCEnable */
			rc_param |= (1 << 2);			/* hvsQPEnable */
			rc_param |= (1 << 3);			/* hvsQpScaleEnable */
			rc_param |= (2 << 4);			/* hvsQpScale=2 */
			rc_param |= (1 << 9);			/* initBufLevelx8=1 */
			rc_param |= (63 << 14);			/* initialRcQp=63 (auto) */
			rc_param |= ((inst->vbv_size & 0xFFF) << 20); /* initialDelay */
		} else {
			/* CQP mode: enable HVS QP for perceptual quality */
			rc_param = 0;
			rc_param |= (1 << 2);			/* hvsQPEnable */
			rc_param |= (1 << 3);			/* hvsQpScaleEnable */
			rc_param |= (2 << 4);			/* hvsQpScale=2 */
		}
		wave420l_write(dev, W4_CMD_ENC_RC_PARAM, rc_param);
	}

	if (enable & W4_ENC_RC_MIN_MAX_QP_CHANGE)
		wave420l_write(dev, W4_CMD_ENC_RC_MIN_MAX_QP,
			       (inst->min_qp & 0x3F) |
			       ((inst->max_qp & 0x3F) << 6) |
			       (10 << 12));

	if (enable & W4_ENC_RC_TRANS_RATE_CHANGE) {
		u32 trans_rate = 0;

		if (inst->rc_enable && inst->bitrate > 0) {
			trans_rate = inst->bitrate;
			if (inst->bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR)
				trans_rate = inst->bitrate + inst->bitrate / 2;
		}
		wave420l_write(dev, W4_CMD_ENC_RC_TRANS_RATE, trans_rate);
	}

	if (enable & W4_ENC_RC_TARGET_RATE_CHANGE)
		wave420l_write(dev, W4_CMD_ENC_RC_TARGET_RATE,
			       inst->rc_enable ? inst->bitrate : 0);

	dev_info(dev->dev, "SET_PARAM change: enable=0x%x bitrate=%u framerate=%u gop=%u\n",
		 enable, inst->bitrate, inst->frame_rate, inst->gop_size);

	wave420l_issue_command(dev, inst->id, W4_CMD_SET_PARAM);
	ret = wave420l_wait_busy(dev);

	if (!ret && wave420l_read(dev, W4_RET_SUCCESS) == 0) {
		dev_err(dev->dev, "SET_PARAM change failed: 0x%x\n",
			wave420l_read(dev, W4_RET_FAIL_REASON));
		ret = -EIO;
	}

	mutex_unlock(&dev->hw_lock);

	if (!ret)
		inst->params_changed = 0;

	return ret;
}

int wave420l_hw_start_encode(struct wave420l_instance *inst,
			     dma_addr_t src_y, dma_addr_t src_u, dma_addr_t src_v,
			     u32 src_stride,
			     dma_addr_t bs_addr, u32 bs_size,
			     u32 src_idx)
{
	struct wave420l_dev *dev = inst->dev;
	u32 src_chroma_stride;
	u32 src_frame_format;
	int ret;

	/* Resume VPU from runtime suspend if sleeping */
	ret = pm_runtime_resume_and_get(dev->dev);
	if (ret < 0 && ret != -ENOSYS) {
		dev_err(dev->dev, "runtime resume failed: %d\n", ret);
		return ret;
	}

	mutex_lock(&dev->hw_lock);

	/*
	 * Use the instance's own BS buffer (dma_alloc_coherent) for the encode
	 * output, rather than the V4L2 capture buffer directly.  The firmware
	 * needs a buffer it can access via DMA, and we'll copy out afterward.
	 */
	wave420l_write(dev, W4_BS_START_ADDR, inst->bs_buf.dma);
	wave420l_write(dev, W4_BS_SIZE, inst->bs_buf.size);
	wave420l_write(dev, W4_BS_RD_PTR, inst->bs_buf.dma);
	wave420l_write(dev, W4_BS_WR_PTR, inst->bs_buf.dma);
	wave420l_write(dev, W4_BS_PARAM, 0xf);  /* no ring buffer, LE endian */

	/* Secondary AXI (disabled) */
	wave420l_write(dev, W4_ADDR_SEC_AXI, 0);
	wave420l_write(dev, W4_SEC_AXI_SIZE, 0);
	wave420l_write(dev, W4_USE_SEC_AXI, 0);

	/* Work buffer */
	wave420l_write(dev, W4_ADDR_WORK_BASE, inst->work_buf.dma);
	wave420l_write(dev, W4_WORK_SIZE, inst->work_buf.size);
	wave420l_write(dev, W4_WORK_PARAM, 0);

	/* Temp buffer */
	wave420l_write(dev, W4_ADDR_TEMP_BASE, inst->temp_buf.dma);
	wave420l_write(dev, W4_TEMP_SIZE, inst->temp_buf.size);
	wave420l_write(dev, W4_TEMP_PARAM, 0);

	/* Report buffer (not used) */
	wave420l_write(dev, W4_CMD_ENC_ADDR_REPORT_BASE, 0);
	wave420l_write(dev, W4_CMD_ENC_REPORT_SIZE, 0);
	wave420l_write(dev, W4_CMD_ENC_REPORT_PARAM, 0);

	/*
	 * Code option: implicit header + VCL = BIT(0) | BIT(1) = 3
	 * SDK: CODEOPT_ENC_HEADER_IMPLICIT | CODEOPT_ENC_VCL
	 * This emits VPS/SPS/PPS then encodes the video frame in one command.
	 * BIT(9) = encodeVUI: include VUI in SPS when VUI params are set.
	 */
	{
		u32 code_opt = BIT(0) | BIT(1);
		bool has_vui = inst->colour_primaries ||
			       inst->transfer_characteristics ||
			       inst->matrix_coefficients ||
			       inst->video_full_range;

		if (has_vui)
			code_opt |= BIT(9);  /* encodeVUI: embed VUI in SPS */
		wave420l_write(dev, W4_CMD_ENC_CODE_OPTION, code_opt);
	}
	wave420l_write(dev, W4_CMD_ENC_SRC_PIC_IDX, src_idx);

	/* Source buffer addresses */
	wave420l_write(dev, W4_CMD_ENC_SRC_ADDR_Y, src_y);
	wave420l_write(dev, W4_CMD_ENC_SRC_ADDR_U, src_u);
	wave420l_write(dev, W4_CMD_ENC_SRC_ADDR_V, src_v);

	/*
	 * SRC_STRIDE: lumaStride[31:16] | chromaStride[15:0]
	 * For NV12/NV21: chromaStride = lumaStride (interleaved UV)
	 * For I420: chromaStride = lumaStride / 2
	 */
	if (inst->src_fmt.pixelformat == V4L2_PIX_FMT_YUV420 ||
	    inst->src_fmt.pixelformat == V4L2_PIX_FMT_YUV420M)
		src_chroma_stride = src_stride / 2;
	else
		src_chroma_stride = src_stride;
	wave420l_write(dev, W4_CMD_ENC_SRC_STRIDE,
		       (src_stride << 16) | src_chroma_stride);

	/*
	 * SRC_FORMAT: srcFrameFormat[2:0] | srcPixelFormat[5:3] | endian[9:6]
	 * srcFrameFormat bits: packedFormat[2] | cbcrInterleave[1] | nv21[0]
	 * For NV12: cbcrInterleave=1, nv21=0 → srcFrameFormat=2
	 * For NV21: cbcrInterleave=1, nv21=1 → srcFrameFormat=3
	 * For I420: cbcrInterleave=0, nv21=0 → srcFrameFormat=0
	 */
	/*
	 * SRC_FORMAT: srcFrameFormat[2:0] | srcPixelFormat[5:3] | srcEndian[9:6]
	 * srcFrameFormat: packedFormat[2] | cbcrInterleave[1] | nv21[0]
	 * srcPixelFormat: justified[4:3] | formatNo[2:0] (0 = 8-bit left-justified)
	 * srcEndian: 0xf for LE on ARM (SDK: ~vdi_convert_endian(LE) & 0xf)
	 */
	if (inst->src_fmt.pixelformat == V4L2_PIX_FMT_NV12 ||
	    inst->src_fmt.pixelformat == V4L2_PIX_FMT_NV12M ||
	    inst->src_fmt.pixelformat == V4L2_PIX_FMT_P010)
		src_frame_format = 2;  /* cbcrInterleave=1, nv21=0 */
	else if (inst->src_fmt.pixelformat == V4L2_PIX_FMT_NV21 ||
		 inst->src_fmt.pixelformat == V4L2_PIX_FMT_NV21M)
		src_frame_format = 3;  /* cbcrInterleave=1, nv21=1 */
	else
		src_frame_format = 0;  /* planar YUV420 */
	/*
	 * srcPixelFormat: justified[4:3] | formatNo[2:0]
	 *   8-bit:  W4_WTL_LEFT_JUSTIFIED(1)<<2 | W4_WTL_PIXEL_8BIT(0)  = 4 → 0
	 *           (SDK default: justified=LEFT(1), formatNo=8BIT(0), but encodes as 0)
	 *           Actually SDK initialises to RIGHT_JUSTIFIED(0) for 8-bit,
	 *           so srcPixelFormat = 0<<2 | 0 = 0.
	 *   P010:   W4_WTL_LEFT_JUSTIFIED(1)<<2 | W4_WTL_PIXEL_16BIT(1) = 5
	 *           (SDK FORMAT_420_P10_16BIT_LSB → justified=LEFT, formatNo=16BIT)
	 * Source endian = 0xf for LE on ARM (SDK: ~vdi_convert_endian(LE) & 0xf)
	 */
	{
		u32 src_pixel_format;

		if (inst->src_fmt.pixelformat == V4L2_PIX_FMT_P010)
			src_pixel_format = (1 << 2) | 1;  /* LEFT_JUSTIFIED<<2 | PIXEL_16BIT */
		else
			src_pixel_format = 0;              /* RIGHT_JUSTIFIED<<2 | PIXEL_8BIT */

		wave420l_write(dev, W4_CMD_ENC_SRC_FORMAT,
			       src_frame_format | (src_pixel_format << 3) | (0xf << 6));
	}

	/* Remaining ENC_PIC registers (zeroed, but firmware may check them) */
	wave420l_write(dev, W4_CMD_ENC_SRC_TIMESTAMP_LOW, 0);
	wave420l_write(dev, W4_CMD_ENC_SRC_TIMESTAMP_HIGH, 0);
	/*
	 * LONGTERM_PIC (BSP: useCurSrcAsLongtermPic[0] | useLongtermRef[1]):
	 * When long-term ref is enabled at sequence level, enable both
	 * per-frame flags so the firmware manages long-term reference pictures.
	 */
	wave420l_write(dev, W4_CMD_ENC_LONGTERM_PIC,
		       inst->long_term_ref_enable ? 0x3 : 0);
	wave420l_write(dev, W4_CMD_ENC_SUB_FRAME_SYNC_CONFIG, 0);

	/*
	 * CTU_OPT_PARAM: per-CTU control mode selection.
	 *   bit[0]   = roiEnable    -- requires rcEnable=1 (CBR/VBR)
	 *   bit[8:1] = roiDeltaQp   -- signed 8-bit delta applied to all map CTUs
	 *   bit[9]   = ctuQpEnable  -- requires rcEnable=0 (CQP mode)
	 * roiEnable and ctuQpEnable are mutually exclusive.
	 * ROI_ADDR_CTU_MAP:  DMA address of per-CTU importance map (roiEnable)
	 * CTU_QP_MAP_ADDR:   DMA address of per-CTU absolute QP map (ctuQpEnable)
	 */
	{
		u32 ctu_opt = 0;

		if (inst->roi_enable && inst->rc_enable && inst->ctu_map.virt) {
			/* ROI mode: RC must be enabled (CBR/VBR) */
			ctu_opt |= BIT(0);  /* roiEnable */
			ctu_opt |= (((u32)(u8)inst->roi_delta_qp) & 0xFF) << 1;
			wave420l_write(dev, W4_CMD_ENC_ROI_ADDR_CTU_MAP,
				       (u32)inst->ctu_map.dma);
			wave420l_write(dev, W4_CMD_ENC_CTU_QP_MAP_ADDR, 0);
			dev_dbg(dev->dev,
				"ENC_PIC ROI: addr=0x%llx delta_qp=%d\n",
				(u64)inst->ctu_map.dma, inst->roi_delta_qp);
		} else if (inst->ctu_qp_enable && !inst->rc_enable &&
			   inst->ctu_map.virt) {
			/* CTU QP map mode: RC must be disabled (CQP) */
			ctu_opt |= BIT(9);  /* ctuQpEnable */
			wave420l_write(dev, W4_CMD_ENC_ROI_ADDR_CTU_MAP, 0);
			wave420l_write(dev, W4_CMD_ENC_CTU_QP_MAP_ADDR,
				       (u32)inst->ctu_map.dma);
			dev_dbg(dev->dev,
				"ENC_PIC CTU-QP: addr=0x%llx\n",
				(u64)inst->ctu_map.dma);
		} else {
			wave420l_write(dev, W4_CMD_ENC_ROI_ADDR_CTU_MAP, 0);
			wave420l_write(dev, W4_CMD_ENC_CTU_QP_MAP_ADDR, 0);
		}
		wave420l_write(dev, W4_CMD_ENC_CTU_OPT_PARAM, ctu_opt);
	}

	/*
	 * SEI NAL insertion:
	 * PREFIX_SEI_INFO: size[31:16] | encOrder[1] | enable[0]
	 *   encOrder=0: encode with current source picture
	 *   encOrder=1: encode at this command (immediate)
	 * PREFIX_SEI_NAL_ADDR: DMA address of raw SEI NAL payload
	 * The firmware wraps the data in a proper SEI NAL unit with
	 * start code and NAL header.
	 */
	if (inst->sei_enabled && inst->sei_data_size > 0 && inst->sei_buf.dma) {
		wave420l_write(dev, W4_CMD_ENC_PREFIX_SEI_NAL_ADDR,
			       inst->sei_buf.dma);
		wave420l_write(dev, W4_CMD_ENC_PREFIX_SEI_INFO,
			       (inst->sei_data_size << 16) | 1);

		dev_dbg(dev->dev, "ENC_PIC SEI: addr=0x%llx size=%u\n",
			(u64)inst->sei_buf.dma, inst->sei_data_size);
	} else {
		wave420l_write(dev, W4_CMD_ENC_PREFIX_SEI_NAL_ADDR, 0);
		wave420l_write(dev, W4_CMD_ENC_PREFIX_SEI_INFO, 0);
	}
	wave420l_write(dev, W4_CMD_ENC_SUFFIX_SEI_NAL_ADDR, 0);
	wave420l_write(dev, W4_CMD_ENC_SUFFIX_SEI_INFO, 0);

	/*
	 * ENC_PIC_PARAM: per-frame QP and picture type overrides.
	 * ENC_PIC_PARAM[0]     = forcePicSkipEnable
	 * ENC_PIC_PARAM[1]     = forcePicQpEnable
	 * ENC_PIC_PARAM[7:2]   = forcePicQpI
	 * ENC_PIC_PARAM[13:8]  = forcePicQpP
	 * ENC_PIC_PARAM[19:14] = forcePicQpB
	 * ENC_PIC_PARAM[20]    = forcePicTypeEnable
	 * ENC_PIC_PARAM[22:21] = forcePicType (0=I, 1=P, 2=B)
	 * V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME takes precedence (forces IDR/I).
	 * Per SDK wave4.c:Wave4VpuEncStartOneFrame.
	 *
	 * KI-041: In CQP mode (!rc_enable), forcePicQpEnable must be set on
	 * every frame. Without it, firmware ignores the INTRA_PARAM QP for
	 * inter frames and produces identical output regardless of QP setting.
	 */
	{
		u32 enc_pic_param = 0;
		struct v4l2_ctrl *force_key_ctrl =
			v4l2_ctrl_find(&inst->ctrl_handler,
				       V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME);

		/* CQP mode: force QP on every frame via per-frame override */
		if (!inst->rc_enable) {
			u32 i_qp = inst->init_qp & 0x3F;
			u32 p_qp = (inst->p_frame_qp ? inst->p_frame_qp : inst->init_qp) & 0x3F;
			u32 b_qp = (inst->b_frame_qp ? inst->b_frame_qp : inst->init_qp) & 0x3F;

			enc_pic_param |= BIT(1);         /* forcePicQpEnable */
			enc_pic_param |= (i_qp << 2);   /* forcePicQpI */
			enc_pic_param |= (p_qp << 8);   /* forcePicQpP */
			enc_pic_param |= (b_qp << 14);  /* forcePicQpB */
		}

		if (force_key_ctrl && v4l2_ctrl_g_ctrl(force_key_ctrl)) {
			/* FORCE_KEY_FRAME: force I frame (forcePicType=0=I) */
			enc_pic_param |= BIT(20);  /* forcePicTypeEnable, forcePicType=0 (I) */
		} else {
			switch (inst->force_pic_type) {
			case 1:  /* I/IDR */
				enc_pic_param |= BIT(20);  /* forcePicTypeEnable, forcePicType=0 (I) */
				break;
			case 2:  /* P */
				enc_pic_param |= BIT(20) | (1U << 21);  /* forcePicTypeEnable, forcePicType=1 (P) */
				break;
			case 3:  /* B */
				enc_pic_param |= BIT(20) | (2U << 21);  /* forcePicTypeEnable, forcePicType=2 (B) */
				break;
			default:
				break;  /* 0=auto: no force */
			}
		}
		wave420l_write(dev, W4_CMD_ENC_PIC_PARAM, enc_pic_param);
	}

	dev_info(dev->dev, "ENC_PIC: bs=0x%llx/%zu src_y=0x%llx stride=%u/%u src_fmt=0x%x idx=%u\n",
		 (u64)inst->bs_buf.dma, inst->bs_buf.size, (u64)src_y,
		 src_stride, src_chroma_stride,
		 src_frame_format | (0xf << 6), src_idx);

	/* Clear any pending interrupts before issuing command */
	wave420l_write(dev, W4_VPU_VINT_REASON_CLR, wave420l_read(dev, W4_VPU_INT_REASON));
	wave420l_write(dev, W4_VPU_VINT_CLEAR, 1);

	/*
	 * PERF-BENCH async device_run: issue ENC_PIC and return immediately.
	 * The hw_lock is released here so other synchronous commands (e.g.
	 * flush) can acquire it while firmware is running.  The IRQ thread
	 * will read result registers and call wave420l_enc_irq_complete()
	 * to finish the M2M job.
	 *
	 * active_inst is set before the command so the IRQ thread can find
	 * the correct instance.  async_enc_pending distinguishes this async
	 * path from the synchronous flush path (which uses irq_done).
	 */
	dev->active_inst = inst;
	inst->async_enc_pending = true;
	wave420l_issue_command(dev, inst->id, W4_CMD_ENC_PIC);
	mutex_unlock(&dev->hw_lock);

	return 0;
}

int wave420l_hw_get_encode_result(struct wave420l_instance *inst,
				  u32 *bs_size, u32 *pic_type, u32 *src_idx)
{
	struct wave420l_dev *dev = inst->dev;

	if (wave420l_read(dev, W4_RET_SUCCESS) == 0) {
		dev_err(dev->dev, "ENC_PIC failed: inst=%d reason=0x%x\n",
			inst->id, wave420l_read(dev, W4_RET_FAIL_REASON));
		return -EIO;
	}

	*bs_size = wave420l_read(dev, W4_RET_ENC_PIC_BYTE);
	*pic_type = wave420l_read(dev, W4_RET_ENC_PIC_TYPE) & 0xFFFF;
	*src_idx = wave420l_read(dev, W4_RET_ENC_USED_SRC_IDX);

	/* Read per-frame encoding statistics from firmware result registers */
	inst->last_avg_qp = wave420l_read(dev, W4_RET_ENC_PIC_AVG_CU_QP);
	inst->last_num_intra = wave420l_read(dev, W4_RET_ENC_PIC_NUM_INTRA);
	inst->last_num_merge = wave420l_read(dev, W4_RET_ENC_PIC_NUM_MERGE);
	inst->last_num_skip = wave420l_read(dev, W4_RET_ENC_PIC_NUM_SKIP);
	inst->last_num_slices = wave420l_read(dev, W4_RET_ENC_PIC_SLICE_NUM);

	dev_info(dev->dev, "ENC result: bs=%u type=%u src=%u recon_idx=%d "
		 "pic_num=%u avgQP=%u intra=%u merge=%u skip=%u slices=%u\n",
		 *bs_size, *pic_type, *src_idx,
		 (s32)wave420l_read(dev, W4_RET_ENC_PIC_IDX),
		 wave420l_read(dev, W4_RET_ENC_PIC_NUM),
		 inst->last_avg_qp, inst->last_num_intra,
		 inst->last_num_merge, inst->last_num_skip,
		 inst->last_num_slices);

	/* Mark activity for runtime PM autosuspend */
	pm_runtime_mark_last_busy(dev->dev);
	pm_runtime_put_autosuspend(dev->dev);

	return 0;
}

/* ------------------------------------------------------------------ */
/* IRQ handler                                                        */
/* ------------------------------------------------------------------ */

static irqreturn_t wave420l_irq_handler(int irq, void *data)
{
	struct wave420l_dev *dev = data;
	u32 reason;

	reason = wave420l_read(dev, W4_VPU_INT_REASON);
	if (!reason)
		return IRQ_NONE;

	dev_info(dev->dev, "IRQ reason=0x%x\n", reason);

	/* Clear interrupt */
	wave420l_write(dev, W4_VPU_VINT_REASON_CLR, reason);
	wave420l_write(dev, W4_VPU_VINT_CLEAR, 1);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t wave420l_irq_thread(int irq, void *data)
{
	struct wave420l_dev *dev = data;
	struct wave420l_instance *inst;

	list_for_each_entry(inst, &dev->instances, list) {
		if (inst->async_enc_pending) {
			/*
			 * Async device_run path (PERF-BENCH): the encode was
			 * issued without waiting.  Complete the M2M job from
			 * this IRQ thread instead of waking a waiter.
			 */
			inst->async_enc_pending = false;
			dev->active_inst = NULL;
			wave420l_enc_irq_complete(inst);
		} else {
			complete(&inst->irq_done);
		}
	}

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* Runtime PM (firmware-level sleep/wake)                             */
/* ------------------------------------------------------------------ */

static int wave420l_runtime_suspend(struct device *device)
{
	struct wave420l_dev *dev = dev_get_drvdata(device);
	int ret;

	mutex_lock(&dev->vpu_lock);

	if (!dev->vpu_initialized || dev->vpu_refcount == 0) {
		mutex_unlock(&dev->vpu_lock);
		return 0;
	}

	ret = wave420l_hw_sleep_vpu(dev);

	mutex_unlock(&dev->vpu_lock);
	return ret;
}

static int wave420l_runtime_resume(struct device *device)
{
	struct wave420l_dev *dev = dev_get_drvdata(device);
	int ret;

	mutex_lock(&dev->vpu_lock);

	if (!dev->vpu_sleeping) {
		mutex_unlock(&dev->vpu_lock);
		return 0;
	}

	ret = wave420l_hw_wakeup_vpu(dev);

	mutex_unlock(&dev->vpu_lock);
	return ret;
}

static const struct dev_pm_ops wave420l_pm_ops = {
	SET_RUNTIME_PM_OPS(wave420l_runtime_suspend,
			   wave420l_runtime_resume, NULL)
};

/* ------------------------------------------------------------------ */
/* Platform driver                                                    */
/* ------------------------------------------------------------------ */

static const struct wave420l_match_data wave420l_sm1_data = {
	.pwr_sleep_bits = SM1_WAVE420L_PWR_SLEEP_BIT,
	.pwr_iso_bits = SM1_WAVE420L_PWR_ISO_BIT,
	.reset_bits = SM1_WAVE420L_RESET_BITS,
	.dos_base = 0xff620000,
	.use_ccf_clocks = false,
};

static const struct wave420l_match_data wave420l_g12b_data = {
	.pwr_sleep_bits = G12A_WAVE420L_PWR_SLEEP_BITS,
	.pwr_iso_bits = G12A_WAVE420L_PWR_ISO_BITS,
	.reset_bits = SM1_WAVE420L_RESET_BITS,
	.dos_base = 0xff620000,
	.use_ccf_clocks = false,
};

static const struct of_device_id wave420l_dt_match[] = {
	{ .compatible = "amlogic,meson-sm1-wave420l", .data = &wave420l_sm1_data },
	{ .compatible = "amlogic,meson-g12b-wave420l", .data = &wave420l_g12b_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wave420l_dt_match);

static int wave420l_probe(struct platform_device *pdev)
{
	struct wave420l_dev *dev;
	struct resource *res;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->dev = &pdev->dev;
	dev->match_data = of_device_get_match_data(&pdev->dev);
	platform_set_drvdata(pdev, dev);

	mutex_init(&dev->dev_lock);
	mutex_init(&dev->hw_lock);
	mutex_init(&dev->vpu_lock);
	dev->vpu_refcount = 0;
	dev->active_inst = NULL;
	dev->vpu_initialized = false;
	ida_init(&dev->inst_ida);
	INIT_LIST_HEAD(&dev->instances);

	/* Map Wave420L registers */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "wave420l");
	if (!res)
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->reg_base))
		return PTR_ERR(dev->reg_base);

	/* Map DOS registers (shared with vdec, using match_data address) */
	dev->dos_base = devm_ioremap(&pdev->dev,
				     dev->match_data->dos_base, 0x10000);
	if (!dev->dos_base)
		return -ENOMEM;

	/* Get HHI syscon (shared clock control registers) */
	dev->hhi_map = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						       "amlogic,hhi-sysctrl");
	if (IS_ERR(dev->hhi_map)) {
		dev_err(&pdev->dev, "failed to get HHI syscon\n");
		return PTR_ERR(dev->hhi_map);
	}

	/* Get AO syscon (power domain registers) */
	dev->ao_map = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						      "amlogic,ao-sysctrl");
	if (IS_ERR(dev->ao_map)) {
		dev_err(&pdev->dev, "failed to get AO syscon\n");
		return PTR_ERR(dev->ao_map);
	}

	/* DOS bus clock gate (required for register access) */
	dev->clk_dos = devm_clk_get_optional(&pdev->dev, "dos");

	/* Try CCF clocks (SC2+) */
	dev->clk_a = devm_clk_get_optional(&pdev->dev, "wave420_aclk");
	dev->clk_b = devm_clk_get_optional(&pdev->dev, "wave420_bclk");
	dev->clk_c = devm_clk_get_optional(&pdev->dev, "wave420_cclk");

	/* IRQ */
	dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0)
		return dev->irq;

	ret = devm_request_threaded_irq(&pdev->dev, dev->irq,
					wave420l_irq_handler,
					wave420l_irq_thread,
					IRQF_ONESHOT, "wave420l", dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to request IRQ %d: %d\n",
			dev->irq, ret);
		return ret;
	}

	/* V4L2 device */
	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register V4L2 device: %d\n", ret);
		return ret;
	}

	/* Register encoder */
	ret = wave420l_enc_register(dev);
	if (ret) {
		v4l2_device_unregister(&dev->v4l2_dev);
		return ret;
	}

	/* Enable runtime PM with 5 second autosuspend */
	pm_runtime_set_autosuspend_delay(&pdev->dev, 5000);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	dev_info(&pdev->dev, "Wave420L HEVC encoder registered\n");

	return 0;
}

static void wave420l_remove(struct platform_device *pdev)
{
	struct wave420l_dev *dev = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);

	wave420l_enc_unregister(dev);
	v4l2_device_unregister(&dev->v4l2_dev);

	if (dev->fw_buf.virt) {
		dma_free_coherent(dev->dev, dev->fw_buf.size,
				  dev->fw_buf.virt, dev->fw_buf.dma);
	}

	ida_destroy(&dev->inst_ida);
}

static struct platform_driver wave420l_driver = {
	.probe = wave420l_probe,
	.remove = wave420l_remove,
	.driver = {
		.name = "meson-wave420l",
		.of_match_table = wave420l_dt_match,
		.pm = &wave420l_pm_ops,
	},
};
module_platform_driver(wave420l_driver);

MODULE_AUTHOR("Libre Computer");
MODULE_DESCRIPTION("Amlogic Meson Wave420L HEVC Encoder");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(WAVE420L_FW_NAME);
