// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2015-2018 Etnaviv Project
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/dma-fence.h>
#include <linux/dma-mapping.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/thermal.h>

#include "etnaviv_cmdbuf.h"
#include "etnaviv_dump.h"
#include "etnaviv_gpu.h"
#include "etnaviv_gem.h"
#include "etnaviv_mmu.h"
#include "etnaviv_perfmon.h"
#include "etnaviv_sched.h"
#include "common.xml.h"

/* WIP r79: NN drain udelay parameterized for empirical sweep.
 * Default 5000us (current production for A311D). Override via:
 *   echo N > /sys/module/etnaviv/parameters/npu_drain_udelay
 * Audit r79: probe minimum value that still produces correct
 * mobilenet_v1 output. */
static int npu_drain_udelay = 5000;
module_param_named(npu_drain_udelay, npu_drain_udelay, int, 0644);
#include "state.xml.h"
#include "state_hi.xml.h"
#include "cmdstream.xml.h"

static const struct platform_device_id gpu_ids[] = {
	{ .name = "etnaviv-gpu,2d" },
	{ },
};

/*
 * Driver functions:
 */

int etnaviv_gpu_get_param(struct etnaviv_gpu *gpu, u32 param, u64 *value)
{
	struct etnaviv_drm_private *priv = gpu->drm->dev_private;

	switch (param) {
	case ETNAVIV_PARAM_GPU_MODEL:
		*value = gpu->identity.model;
		break;

	case ETNAVIV_PARAM_GPU_REVISION:
		*value = gpu->identity.revision;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_0:
		*value = gpu->identity.features;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_1:
		*value = gpu->identity.minor_features0;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_2:
		*value = gpu->identity.minor_features1;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_3:
		*value = gpu->identity.minor_features2;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_4:
		*value = gpu->identity.minor_features3;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_5:
		*value = gpu->identity.minor_features4;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_6:
		*value = gpu->identity.minor_features5;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_7:
		*value = gpu->identity.minor_features6;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_8:
		*value = gpu->identity.minor_features7;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_9:
		*value = gpu->identity.minor_features8;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_10:
		*value = gpu->identity.minor_features9;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_11:
		*value = gpu->identity.minor_features10;
		break;

	case ETNAVIV_PARAM_GPU_FEATURES_12:
		*value = gpu->identity.minor_features11;
		break;

	case ETNAVIV_PARAM_GPU_STREAM_COUNT:
		*value = gpu->identity.stream_count;
		break;

	case ETNAVIV_PARAM_GPU_REGISTER_MAX:
		*value = gpu->identity.register_max;
		break;

	case ETNAVIV_PARAM_GPU_THREAD_COUNT:
		*value = gpu->identity.thread_count;
		break;

	case ETNAVIV_PARAM_GPU_VERTEX_CACHE_SIZE:
		*value = gpu->identity.vertex_cache_size;
		break;

	case ETNAVIV_PARAM_GPU_SHADER_CORE_COUNT:
		*value = gpu->identity.shader_core_count;
		break;

	case ETNAVIV_PARAM_GPU_PIXEL_PIPES:
		*value = gpu->identity.pixel_pipes;
		break;

	case ETNAVIV_PARAM_GPU_VERTEX_OUTPUT_BUFFER_SIZE:
		*value = gpu->identity.vertex_output_buffer_size;
		break;

	case ETNAVIV_PARAM_GPU_BUFFER_SIZE:
		*value = gpu->identity.buffer_size;
		break;

	case ETNAVIV_PARAM_GPU_INSTRUCTION_COUNT:
		*value = gpu->identity.instruction_count;
		break;

	case ETNAVIV_PARAM_GPU_NUM_CONSTANTS:
		*value = gpu->identity.num_constants;
		break;

	case ETNAVIV_PARAM_GPU_NUM_VARYINGS:
		*value = gpu->identity.varyings_count;
		break;

	case ETNAVIV_PARAM_SOFTPIN_START_ADDR:
		if (priv->mmu_global->version == ETNAVIV_IOMMU_V2)
			*value = ETNAVIV_SOFTPIN_START_ADDRESS;
		else
			*value = ~0ULL;
		break;

	case ETNAVIV_PARAM_GPU_PRODUCT_ID:
		*value = gpu->identity.product_id;
		break;

	case ETNAVIV_PARAM_GPU_CUSTOMER_ID:
		*value = gpu->identity.customer_id;
		break;

	case ETNAVIV_PARAM_GPU_ECO_ID:
		*value = gpu->identity.eco_id;
		break;

	case ETNAVIV_PARAM_GPU_NN_CORE_COUNT:
		*value = gpu->identity.nn_core_count;
		break;

	case ETNAVIV_PARAM_GPU_NN_MAD_PER_CORE:
		*value = gpu->identity.nn_mad_per_core;
		break;

	case ETNAVIV_PARAM_GPU_TP_CORE_COUNT:
		*value = gpu->identity.tp_core_count;
		break;

	case ETNAVIV_PARAM_GPU_ON_CHIP_SRAM_SIZE:
		*value = gpu->identity.on_chip_sram_size;
		break;

	case ETNAVIV_PARAM_GPU_AXI_SRAM_SIZE:
		*value = gpu->identity.axi_sram_size;
		break;

	case ETNAVIV_PARAM_GPU_AXI_SRAM_START:
		/*
		 * Only reports non-zero when the kernel coherent alloc
		 * succeeded AND the chip actually has AXI SRAM. Userspace
		 * treats 0 as "skip OCB_REMAP programming".
		 */
		if (priv->mmu_global && priv->mmu_global->axi_sram_cpu)
			*value = ETNAVIV_AXI_SRAM_VA;
		else
			*value = 0;
		break;

	default:
		DBG("%s: invalid param: %u", dev_name(gpu->dev), param);
		return -EINVAL;
	}

	return 0;
}

static inline bool etnaviv_is_model_rev(struct etnaviv_gpu *gpu, u32 model, u32 revision)
{
	return gpu->identity.model == model &&
	       gpu->identity.revision == revision;
}

#define etnaviv_field(val, field) \
	(((val) & field##__MASK) >> field##__SHIFT)

static void etnaviv_hw_specs(struct etnaviv_gpu *gpu)
{
	if (gpu->identity.minor_features0 &
	    chipMinorFeatures0_MORE_MINOR_FEATURES) {
		u32 specs[4];
		unsigned int streams;

		specs[0] = gpu_read(gpu, VIVS_HI_CHIP_SPECS);
		specs[1] = gpu_read(gpu, VIVS_HI_CHIP_SPECS_2);
		specs[2] = gpu_read(gpu, VIVS_HI_CHIP_SPECS_3);
		specs[3] = gpu_read(gpu, VIVS_HI_CHIP_SPECS_4);

		gpu->identity.stream_count = etnaviv_field(specs[0],
					VIVS_HI_CHIP_SPECS_STREAM_COUNT);
		gpu->identity.register_max = etnaviv_field(specs[0],
					VIVS_HI_CHIP_SPECS_REGISTER_MAX);
		gpu->identity.thread_count = etnaviv_field(specs[0],
					VIVS_HI_CHIP_SPECS_THREAD_COUNT);
		gpu->identity.vertex_cache_size = etnaviv_field(specs[0],
					VIVS_HI_CHIP_SPECS_VERTEX_CACHE_SIZE);
		gpu->identity.shader_core_count = etnaviv_field(specs[0],
					VIVS_HI_CHIP_SPECS_SHADER_CORE_COUNT);
		gpu->identity.pixel_pipes = etnaviv_field(specs[0],
					VIVS_HI_CHIP_SPECS_PIXEL_PIPES);
		gpu->identity.vertex_output_buffer_size =
			etnaviv_field(specs[0],
				VIVS_HI_CHIP_SPECS_VERTEX_OUTPUT_BUFFER_SIZE);

		gpu->identity.buffer_size = etnaviv_field(specs[1],
					VIVS_HI_CHIP_SPECS_2_BUFFER_SIZE);
		gpu->identity.instruction_count = etnaviv_field(specs[1],
					VIVS_HI_CHIP_SPECS_2_INSTRUCTION_COUNT);
		gpu->identity.num_constants = etnaviv_field(specs[1],
					VIVS_HI_CHIP_SPECS_2_NUM_CONSTANTS);

		gpu->identity.varyings_count = etnaviv_field(specs[2],
					VIVS_HI_CHIP_SPECS_3_VARYINGS_COUNT);

		/* This overrides the value from older register if non-zero */
		streams = etnaviv_field(specs[3],
					VIVS_HI_CHIP_SPECS_4_STREAM_COUNT);
		if (streams)
			gpu->identity.stream_count = streams;
	}

	/* Fill in the stream count if not specified */
	if (gpu->identity.stream_count == 0) {
		if (gpu->identity.model >= 0x1000)
			gpu->identity.stream_count = 4;
		else
			gpu->identity.stream_count = 1;
	}

	/* Convert the register max value */
	if (gpu->identity.register_max)
		gpu->identity.register_max = 1 << gpu->identity.register_max;
	else if (gpu->identity.model == chipModel_GC400)
		gpu->identity.register_max = 32;
	else
		gpu->identity.register_max = 64;

	/* Convert thread count */
	if (gpu->identity.thread_count)
		gpu->identity.thread_count = 1 << gpu->identity.thread_count;
	else if (gpu->identity.model == chipModel_GC400)
		gpu->identity.thread_count = 64;
	else if (gpu->identity.model == chipModel_GC500 ||
		 gpu->identity.model == chipModel_GC530)
		gpu->identity.thread_count = 128;
	else
		gpu->identity.thread_count = 256;

	if (gpu->identity.vertex_cache_size == 0)
		gpu->identity.vertex_cache_size = 8;

	if (gpu->identity.shader_core_count == 0) {
		if (gpu->identity.model >= 0x1000)
			gpu->identity.shader_core_count = 2;
		else
			gpu->identity.shader_core_count = 1;
	}

	if (gpu->identity.pixel_pipes == 0)
		gpu->identity.pixel_pipes = 1;

	/* Convert virtex buffer size */
	if (gpu->identity.vertex_output_buffer_size) {
		gpu->identity.vertex_output_buffer_size =
			1 << gpu->identity.vertex_output_buffer_size;
	} else if (gpu->identity.model == chipModel_GC400) {
		if (gpu->identity.revision < 0x4000)
			gpu->identity.vertex_output_buffer_size = 512;
		else if (gpu->identity.revision < 0x4200)
			gpu->identity.vertex_output_buffer_size = 256;
		else
			gpu->identity.vertex_output_buffer_size = 128;
	} else {
		gpu->identity.vertex_output_buffer_size = 512;
	}

	switch (gpu->identity.instruction_count) {
	case 0:
		if (etnaviv_is_model_rev(gpu, 0x2000, 0x5108) ||
		    gpu->identity.model == chipModel_GC880)
			gpu->identity.instruction_count = 512;
		else
			gpu->identity.instruction_count = 256;
		break;

	case 1:
		gpu->identity.instruction_count = 1024;
		break;

	case 2:
		gpu->identity.instruction_count = 2048;
		break;

	default:
		gpu->identity.instruction_count = 256;
		break;
	}

	if (gpu->identity.num_constants == 0)
		gpu->identity.num_constants = 168;

	if (gpu->identity.varyings_count == 0) {
		if (gpu->identity.minor_features1 & chipMinorFeatures1_HALTI0)
			gpu->identity.varyings_count = 12;
		else
			gpu->identity.varyings_count = 8;
	}

	/*
	 * For some cores, two varyings are consumed for position, so the
	 * maximum varying count needs to be reduced by one.
	 */
	if (etnaviv_is_model_rev(gpu, 0x5000, 0x5434) ||
	    etnaviv_is_model_rev(gpu, 0x4000, 0x5222) ||
	    etnaviv_is_model_rev(gpu, 0x4000, 0x5245) ||
	    etnaviv_is_model_rev(gpu, 0x4000, 0x5208) ||
	    etnaviv_is_model_rev(gpu, 0x3000, 0x5435) ||
	    etnaviv_is_model_rev(gpu, 0x2200, 0x5244) ||
	    etnaviv_is_model_rev(gpu, 0x2100, 0x5108) ||
	    etnaviv_is_model_rev(gpu, 0x2000, 0x5108) ||
	    etnaviv_is_model_rev(gpu, 0x1500, 0x5246) ||
	    etnaviv_is_model_rev(gpu, 0x880, 0x5107) ||
	    etnaviv_is_model_rev(gpu, 0x880, 0x5106))
		gpu->identity.varyings_count -= 1;
}

static void etnaviv_hw_identify(struct etnaviv_gpu *gpu)
{
	u32 chipIdentity;

	dev_info(gpu->dev, "hw_identify: reading CHIP_IDENTITY at %pR\n",
		 gpu->mmio);
	chipIdentity = gpu_read(gpu, VIVS_HI_CHIP_IDENTITY);
	dev_info(gpu->dev, "hw_identify: CHIP_IDENTITY=0x%08x\n", chipIdentity);

	/* Special case for older graphic cores. */
	if (etnaviv_field(chipIdentity, VIVS_HI_CHIP_IDENTITY_FAMILY) == 0x01) {
		gpu->identity.model    = chipModel_GC500;
		gpu->identity.revision = etnaviv_field(chipIdentity,
					 VIVS_HI_CHIP_IDENTITY_REVISION);
	} else {
		u32 chipDate = gpu_read(gpu, VIVS_HI_CHIP_DATE);

		gpu->identity.model = gpu_read(gpu, VIVS_HI_CHIP_MODEL);
		gpu->identity.revision = gpu_read(gpu, VIVS_HI_CHIP_REV);
		gpu->identity.customer_id = gpu_read(gpu, VIVS_HI_CHIP_CUSTOMER_ID);

		/*
		 * Reading these two registers on GC600 rev 0x19 result in a
		 * unhandled fault: external abort on non-linefetch
		 */
		if (!etnaviv_is_model_rev(gpu, 0x600, 0x19)) {
			gpu->identity.product_id = gpu_read(gpu, VIVS_HI_CHIP_PRODUCT_ID);
			gpu->identity.eco_id = gpu_read(gpu, VIVS_HI_CHIP_ECO_ID);
		}

		/*
		 * !!!! HACK ALERT !!!!
		 * Because people change device IDs without letting software
		 * know about it - here is the hack to make it all look the
		 * same.  Only for GC400 family.
		 */
		if ((gpu->identity.model & 0xff00) == 0x0400 &&
		    gpu->identity.model != chipModel_GC420) {
			gpu->identity.model = gpu->identity.model & 0x0400;
		}

		/* Another special case */
		if (etnaviv_is_model_rev(gpu, 0x300, 0x2201)) {
			u32 chipTime = gpu_read(gpu, VIVS_HI_CHIP_TIME);

			if (chipDate == 0x20080814 && chipTime == 0x12051100) {
				/*
				 * This IP has an ECO; put the correct
				 * revision in it.
				 */
				gpu->identity.revision = 0x1051;
			}
		}

		/*
		 * NXP likes to call the GPU on the i.MX6QP GC2000+, but in
		 * reality it's just a re-branded GC3000. We can identify this
		 * core by the upper half of the revision register being all 1.
		 * Fix model/rev here, so all other places can refer to this
		 * core by its real identity.
		 */
		if (etnaviv_is_model_rev(gpu, 0x2000, 0xffff5450)) {
			gpu->identity.model = chipModel_GC3000;
			gpu->identity.revision &= 0xffff;
		}

		if (etnaviv_is_model_rev(gpu, 0x1000, 0x5037) && (chipDate == 0x20120617))
			gpu->identity.eco_id = 1;

		if (etnaviv_is_model_rev(gpu, 0x320, 0x5303) && (chipDate == 0x20140511))
			gpu->identity.eco_id = 1;
	}

	dev_info(gpu->dev, "model: GC%x, revision: %x\n",
		 gpu->identity.model, gpu->identity.revision);

	gpu->idle_mask = ~VIVS_HI_IDLE_STATE_AXI_LP;
	/*
	 * If there is a match in the HWDB, we aren't interested in the
	 * remaining register values, as they might be wrong.
	 */
	if (etnaviv_fill_identity_from_hwdb(gpu))
		return;

	gpu->identity.features = gpu_read(gpu, VIVS_HI_CHIP_FEATURE);

	/* Disable fast clear on GC700. */
	if (gpu->identity.model == chipModel_GC700)
		gpu->identity.features &= ~chipFeatures_FAST_CLEAR;

	/* These models/revisions don't have the 2D pipe bit */
	if ((gpu->identity.model == chipModel_GC500 &&
	     gpu->identity.revision <= 2) ||
	    gpu->identity.model == chipModel_GC300)
		gpu->identity.features |= chipFeatures_PIPE_2D;

	if ((gpu->identity.model == chipModel_GC500 &&
	     gpu->identity.revision < 2) ||
	    (gpu->identity.model == chipModel_GC300 &&
	     gpu->identity.revision < 0x2000)) {

		/*
		 * GC500 rev 1.x and GC300 rev < 2.0 doesn't have these
		 * registers.
		 */
		gpu->identity.minor_features0 = 0;
		gpu->identity.minor_features1 = 0;
		gpu->identity.minor_features2 = 0;
		gpu->identity.minor_features3 = 0;
		gpu->identity.minor_features4 = 0;
		gpu->identity.minor_features5 = 0;
	} else
		gpu->identity.minor_features0 =
				gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_0);

	if (gpu->identity.minor_features0 &
	    chipMinorFeatures0_MORE_MINOR_FEATURES) {
		gpu->identity.minor_features1 =
				gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_1);
		gpu->identity.minor_features2 =
				gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_2);
		gpu->identity.minor_features3 =
				gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_3);
		gpu->identity.minor_features4 =
				gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_4);
		gpu->identity.minor_features5 =
				gpu_read(gpu, VIVS_HI_CHIP_MINOR_FEATURE_5);
	}

	/* GC600/300 idle register reports zero bits where modules aren't present */
	if (gpu->identity.model == chipModel_GC600 ||
	    gpu->identity.model == chipModel_GC300)
		gpu->idle_mask = VIVS_HI_IDLE_STATE_TX |
				 VIVS_HI_IDLE_STATE_RA |
				 VIVS_HI_IDLE_STATE_SE |
				 VIVS_HI_IDLE_STATE_PA |
				 VIVS_HI_IDLE_STATE_SH |
				 VIVS_HI_IDLE_STATE_PE |
				 VIVS_HI_IDLE_STATE_DE |
				 VIVS_HI_IDLE_STATE_FE;

	etnaviv_hw_specs(gpu);
}

static void etnaviv_gpu_load_clock(struct etnaviv_gpu *gpu, u32 clock)
{
	gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, clock |
		  VIVS_HI_CLOCK_CONTROL_FSCALE_CMD_LOAD);
	gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, clock);
}

static void etnaviv_gpu_update_clock(struct etnaviv_gpu *gpu)
{
	if (gpu->identity.minor_features2 &
	    chipMinorFeatures2_DYNAMIC_FREQUENCY_SCALING) {
		clk_set_rate(gpu->clk_core,
			     gpu->base_rate_core >> gpu->freq_scale);
		clk_set_rate(gpu->clk_shader,
			     gpu->base_rate_shader >> gpu->freq_scale);
	} else {
		unsigned int fscale = 1 << (6 - gpu->freq_scale);
		u32 clock = gpu_read(gpu, VIVS_HI_CLOCK_CONTROL);

		clock &= ~VIVS_HI_CLOCK_CONTROL_FSCALE_VAL__MASK;
		clock |= VIVS_HI_CLOCK_CONTROL_FSCALE_VAL(fscale);
		etnaviv_gpu_load_clock(gpu, clock);
	}

	/*
	 * Choose number of wait cycles to target a ~30us (1/32768) max latency
	 * until new work is picked up by the FE when it polls in the idle loop.
	 * If the GPU base frequency is unknown use 200 wait cycles.
	 */
	gpu->fe_waitcycles = clamp(gpu->base_rate_core >> (15 - gpu->freq_scale),
				   200UL, 0xffffUL);
}

static int etnaviv_hw_reset(struct etnaviv_gpu *gpu)
{
	u32 control, idle;
	unsigned long timeout;
	bool failed = true;

	/* We hope that the GPU resets in under one second */
	timeout = jiffies + msecs_to_jiffies(1000);

	while (time_is_after_jiffies(timeout)) {
		unsigned int fscale = 1 << (6 - gpu->freq_scale);
		u32 pulse_eater = 0x01590880;

		/* disable clock gating */
		gpu_write_power(gpu, VIVS_PM_POWER_CONTROLS, 0x0);

		/* disable pulse eater */
		pulse_eater |= BIT(17);
		gpu_write_power(gpu, VIVS_PM_PULSE_EATER, pulse_eater);
		pulse_eater |= BIT(0);
		gpu_write_power(gpu, VIVS_PM_PULSE_EATER, pulse_eater);

		/* enable clock */
		control = VIVS_HI_CLOCK_CONTROL_FSCALE_VAL(fscale);
		etnaviv_gpu_load_clock(gpu, control);

		/* isolate the GPU. */
		control |= VIVS_HI_CLOCK_CONTROL_ISOLATE_GPU;
		gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);

		if (gpu->sec_mode == ETNA_SEC_KERNEL) {
			gpu_write(gpu, VIVS_MMUv2_AHB_CONTROL,
			          VIVS_MMUv2_AHB_CONTROL_RESET);
		} else {
			/* set soft reset. */
			control |= VIVS_HI_CLOCK_CONTROL_SOFT_RESET;
			gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);
		}

		/* wait for reset. */
		usleep_range(10, 20);

		/* reset soft reset bit. */
		control &= ~VIVS_HI_CLOCK_CONTROL_SOFT_RESET;
		gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);

		/* reset GPU isolation. */
		control &= ~VIVS_HI_CLOCK_CONTROL_ISOLATE_GPU;
		gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);

		/* read idle register. */
		idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

		/* try resetting again if FE is not idle */
		if ((idle & VIVS_HI_IDLE_STATE_FE) == 0) {
			dev_dbg(gpu->dev, "FE is not idle\n");
			continue;
		}

		/* read reset register. */
		control = gpu_read(gpu, VIVS_HI_CLOCK_CONTROL);

		/* is the GPU idle? */
		if (((control & VIVS_HI_CLOCK_CONTROL_IDLE_3D) == 0) ||
		    ((control & VIVS_HI_CLOCK_CONTROL_IDLE_2D) == 0)) {
			dev_dbg(gpu->dev, "GPU is not idle\n");
			continue;
		}

		/* disable debug registers, as they are not normally needed */
		control |= VIVS_HI_CLOCK_CONTROL_DISABLE_DEBUG_REGISTERS;
		gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, control);

		failed = false;
		break;
	}

	if (failed) {
		idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);
		control = gpu_read(gpu, VIVS_HI_CLOCK_CONTROL);

		dev_err(gpu->dev, "GPU failed to reset: FE %sidle, 3D %sidle, 2D %sidle\n",
			idle & VIVS_HI_IDLE_STATE_FE ? "" : "not ",
			control & VIVS_HI_CLOCK_CONTROL_IDLE_3D ? "" : "not ",
			control & VIVS_HI_CLOCK_CONTROL_IDLE_2D ? "" : "not ");

		return -EBUSY;
	}

	/* We rely on the GPU running, so program the clock */
	etnaviv_gpu_update_clock(gpu);

	gpu->state = ETNA_GPU_STATE_RESET;
	gpu->exec_state = -1;
	if (gpu->mmu_context)
		etnaviv_iommu_context_put(gpu->mmu_context);
	gpu->mmu_context = NULL;

	return 0;
}

static void etnaviv_gpu_enable_mlcg(struct etnaviv_gpu *gpu)
{
	u32 pmc, ppc;

	/* enable clock gating */
	ppc = gpu_read_power(gpu, VIVS_PM_POWER_CONTROLS);
	ppc |= VIVS_PM_POWER_CONTROLS_ENABLE_MODULE_CLOCK_GATING;

	/* Disable stall module clock gating for 4.3.0.1 and 4.3.0.2 revs */
	if (gpu->identity.revision == 0x4301 ||
	    gpu->identity.revision == 0x4302)
		ppc |= VIVS_PM_POWER_CONTROLS_DISABLE_STALL_MODULE_CLOCK_GATING;

	gpu_write_power(gpu, VIVS_PM_POWER_CONTROLS, ppc);

	pmc = gpu_read_power(gpu, VIVS_PM_MODULE_CONTROLS);

	/* Disable PA clock gating for GC400+ without bugfix except for GC420 */
	if (gpu->identity.model >= chipModel_GC400 &&
	    gpu->identity.model != chipModel_GC420 &&
	    !(gpu->identity.minor_features3 & chipMinorFeatures3_BUG_FIXES12))
		pmc |= VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_PA;

	/*
	 * Disable PE clock gating on revs < 5.0.0.0 when HZ is
	 * present without a bug fix.
	 */
	if (gpu->identity.revision < 0x5000 &&
	    gpu->identity.minor_features0 & chipMinorFeatures0_HZ &&
	    !(gpu->identity.minor_features1 &
	      chipMinorFeatures1_DISABLE_PE_GATING))
		pmc |= VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_PE;

	if (gpu->identity.revision < 0x5422)
		pmc |= BIT(15); /* Unknown bit */

	/* Disable TX clock gating on affected core revisions. */
	if (etnaviv_is_model_rev(gpu, 0x4000, 0x5222) ||
	    etnaviv_is_model_rev(gpu, 0x2000, 0x5108) ||
	    etnaviv_is_model_rev(gpu, 0x7000, 0x6202) ||
	    etnaviv_is_model_rev(gpu, 0x7000, 0x6203))
		pmc |= VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_TX;

	/* Disable SE and RA clock gating on affected core revisions. */
	if (etnaviv_is_model_rev(gpu, 0x7000, 0x6202))
		pmc |= VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_SE |
		       VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_RA;

	/* Disable SH_EU clock gating on affected core revisions. */
	if (etnaviv_is_model_rev(gpu, 0x8000, 0x7200) ||
	    etnaviv_is_model_rev(gpu, 0x8000, 0x8002) ||
	    etnaviv_is_model_rev(gpu, 0x9200, 0x6304))
		pmc |= VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_SH_EU;

	pmc |= VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_RA_HZ;
	pmc |= VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_RA_EZ;

	/* Disable NN clock gating on NPU cores to prevent FE register write stalls */
	if (gpu->identity.nn_core_count > 0)
		pmc |= VIVS_PM_MODULE_CONTROLS_DISABLE_MODULE_CLOCK_GATING_NN;

	gpu_write_power(gpu, VIVS_PM_MODULE_CONTROLS, pmc);
}

void etnaviv_gpu_start_fe(struct etnaviv_gpu *gpu, u32 address, u16 prefetch)
{
	gpu_write(gpu, VIVS_FE_COMMAND_ADDRESS, address);
	gpu_write(gpu, VIVS_FE_COMMAND_CONTROL,
		  VIVS_FE_COMMAND_CONTROL_ENABLE |
		  VIVS_FE_COMMAND_CONTROL_PREFETCH(prefetch));

	if (gpu->sec_mode == ETNA_SEC_KERNEL) {
		gpu_write(gpu, VIVS_MMUv2_SEC_COMMAND_CONTROL,
			  VIVS_MMUv2_SEC_COMMAND_CONTROL_ENABLE |
			  VIVS_MMUv2_SEC_COMMAND_CONTROL_PREFETCH(prefetch));
	}
}

/*
 * NN flop reset: run a minimal NN convolution to initialize hardware state.
 * This is required on NPU-only devices (VIPNano/VIPPico) to reset internal
 * flip-flop state. Based on galcore's gckHARDWARE_ResetFlopWithNN.
 *
 * The nn_data buffer layout (GPU VA mapped via sualloc):
 *   0x000: NN instruction (128 bytes)
 *   0x100: NN kernel coefficients (128 bytes)
 *   0x200: NN input data (64 bytes)
 *   0x300: NN output data (64 bytes)
 */

/* PS/PA/GL_API_MODE registers used by NPU paths -- definitions live in state.xml.h */

/* BSP NN instruction data for VIPPico v3 (customerID 0x99, INT8) */
static const u32 nn_flop_ins[32] = {
	0xA0100048, 0x001000C0, 0xC8000000, 0x00080080,
	0x02084001, 0x037FD200, 0xDFF6F000, 0xDFF6E000,
	0x00012004, 0x00000000, 0x00000000, 0x00000800,
	0x00000880, 0x00000000, 0x00000800, 0x00000000,
	0x00020003, 0x01000002, 0x00000000, 0x03FFFFFF,
	0x00000000, 0x03FFFFFF, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000
};

/* BSP NN kernel coefficients for VIPPico v3 (2x2x1 INT8 conv) */
static const u32 nn_flop_ker[32] = {
	0x01020411, 0x00000100, 0x00000000, 0x00000000,
	0x00000000, 0x26540781, 0x00000000, 0x00000069,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x0073FFE0, 0xC0003800, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000
};

static void etnaviv_gpu_nn_flop_reset(struct etnaviv_gpu *gpu)
{
	struct etnaviv_cmdbuf *buffer = &gpu->buffer;
	struct etnaviv_iommu_context *context = gpu->mmu_context;
	struct drm_mm_node *nn_node = &gpu->nn_flop_node;
	void *data_cpu = gpu->nn_data_vaddr;
	dma_addr_t data_paddr = gpu->nn_data_paddr;
	u32 data_va, inst_va, ker_va, in_va, out_va;
	u32 *inst;
	u16 prefetch;
	u32 cmd_va;
	int i, ret;

	if (gpu->identity.nn_core_count == 0 || !data_cpu)
		return;

	/* NN flop reset uses VIPPico v3 (0x99) NNCmd instruction encoding.
	 * A311D (0x88, VIPNano-QI) has different NN instruction format --
	 * running the wrong encoding leaves NN in undefined state, producing
	 * all-zero inference output. Disable until A311D template is added.
	 */
	if (gpu->identity.customer_id != 0x99)
		return;

	/*
	 * Flop reset is a one-time hardware init. The NN engine retains
	 * register state from this reset and must not be re-initialized
	 * on every context switch -- the reserved VAs are stored in the
	 * gpu struct so they persist across client contexts. Use
	 * drm_mm_node_allocated() to detect previous success (handles
	 * partial completion where TP reset failed on a prior call).
	 */
	if (drm_mm_node_allocated(nn_node))
		return;

	/*
	 * Map the NN data page into GPU VA space above the VIP SRAM window.
	 * SRAM_REMAP_ADDRESS=0x400000 makes VAs [0x400000, 0x400000+sram_size)
	 * map to VIP SRAM instead of the MMU. Our data must be above this
	 * window so the NN engine reads/writes DDR via MMU, not SRAM.
	 */
	{
		u32 sram_end = 0x00400000 + gpu->identity.on_chip_sram_size;

		/* Align up to page boundary */
		sram_end = ALIGN(sram_end, PAGE_SIZE);
		if (sram_end < ETNAVIV_SOFTPIN_START_ADDRESS)
			sram_end = ETNAVIV_SOFTPIN_START_ADDRESS;

		mutex_lock(&context->lock);
		ret = drm_mm_insert_node_in_range(&context->mm, nn_node,
						  PAGE_SIZE, 0, 0,
						  sram_end,
						  U64_MAX, DRM_MM_INSERT_LOW);
	}
	mutex_unlock(&context->lock);
	if (ret) {
		dev_err(gpu->dev, "NN flop reset: VA alloc failed: %d\n", ret);
		return;
	}

	data_va = nn_node->start;
	ret = context->global->ops->map(context, data_va, data_paddr,
					PAGE_SIZE,
					ETNAVIV_PROT_READ | ETNAVIV_PROT_WRITE);
	if (ret) {
		dev_err(gpu->dev, "NN flop reset: IOMMU map failed: %d\n", ret);
		goto out_remove;
	}

	inst_va = data_va + 0x000;
	ker_va  = data_va + 0x100;
	in_va   = data_va + 0x200;
	out_va  = data_va + 0x300;

	/* Copy BSP NN instruction and patch addresses */
	inst = data_cpu;
	memcpy(inst, nn_flop_ins, sizeof(nn_flop_ins));
	inst[5] = (ker_va >> 6) & 0x03FFFFFF;
	inst[6] = in_va;
	inst[7] = out_va;

	/* Copy NN kernel coefficients at offset 0x100 */
	memcpy(data_cpu + 0x100, nn_flop_ker, sizeof(nn_flop_ker));

	/* Fill input at offset 0x200: 6 bytes of 0x33 (3x2x1 INT8) */
	memset(data_cpu + 0x200, 0x33, 64);

	/* Pre-fill output at offset 0x300 with 0xAA to detect NN writes */
	memset(data_cpu + 0x300, 0xAA, 64);

	wmb();

	dev_info(gpu->dev,
		 "NN flop reset: inst[0..7]=%08x %08x %08x %08x %08x %08x %08x %08x\n",
		 inst[0], inst[1], inst[2], inst[3],
		 inst[4], inst[5], inst[6], inst[7]);
	dev_info(gpu->dev,
		 "NN flop reset: inst[8..15]=%08x %08x %08x %08x %08x %08x %08x %08x\n",
		 inst[8], inst[9], inst[10], inst[11],
		 inst[12], inst[13], inst[14], inst[15]);
	dev_info(gpu->dev,
		 "NN flop reset: data_paddr=0x%llx data_va=0x%08x\n",
		 (u64)data_paddr, data_va);

	/*
	 * Build the NNCmd in the ring buffer.
	 * SRAM_REMAP_ADDRESS=0x400000: VAs 0-0x3FFFFF → VIP SRAM,
	 * VAs >= 0x400000 → MMU. NN cache uses SRAM, data uses MMU.
	 *
	 * Note: GL registers like SRAM_REMAP and NN_CONFIG cannot be read
	 * via AHB on this chip (sync abort), but can be written via FE
	 * LOAD_STATE commands.
	 */
#define LS(reg, val) do {					\
	cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |	\
		   VIV_FE_LOAD_STATE_HEADER_COUNT(1) |		\
		   VIV_FE_LOAD_STATE_HEADER_OFFSET(		\
			(reg) >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR); \
	cmd[i++] = (val);					\
} while (0)

	{
		u32 *cmd = buffer->vaddr;

		i = 0;
		/*
		 * HI_REORDER fix: set bit 6 of reg 0x0090 for NN chips
		 * without AXI SRAM. Cannot read this register via AHB
		 * (sync abort), so write via FE LOAD_STATE. After reset
		 * the register is 0, so just write 0x40.
		 */
		LS(VIVS_HI_COMPRESSION_FLAGS, 0x40);
		/*
		 * Flush MMU TLB: new IOMMU mapping was just created
		 * for the NN data page. Without this, the GPU's TLB
		 * may not see the mapping and NN reads/writes go to
		 * the safe address.
		 */
		LS(VIVS_MMUv2_CONFIGURATION,
		   VIVS_MMUv2_CONFIGURATION_MODE_MASK |
		   VIVS_MMUv2_CONFIGURATION_FLUSH_FLUSH |
		   etnaviv_iommuv2_get_mtlb_addr(context));
		/* WAIT for TLB flush to complete (no PE for STALL) */
		cmd[i++] = VIV_FE_WAIT_HEADER_OP_WAIT | 32000;
		cmd[i++] = 0; /* pad to 64-bit alignment */
		/*
		 * PA_SYSTEM_MODE and GL_API_MODE must be set before
		 * NN_INST_ADDR — without OPENCL mode the NN engine
		 * ignores NN_INST_ADDR writes entirely.
		 */
		LS(VIVS_PA_SYSTEM_MODE, 0x11);
		LS(VIVS_GL_API_MODE, VIVS_GL_API_MODE_OPENCL);
		LS(VIVS_GL_SRAM_REMAP_ADDRESS, 0x00400000);
		LS(VIVS_GL_SRAM_REMAP_ADDRESS, 0x00400000);
		LS(VIVS_GL_OCB_REMAP_START, 0x0);
		LS(VIVS_GL_OCB_REMAP_END, 0x0);
		LS(VIVS_GL_NN_CONFIG, VIVS_GL_NN_CONFIG_SMALL_BATCH);
		LS(VIVS_GL_UNK03950, 0x0);
		LS(VIVS_GL_SHADER_INDEX, 0x0);
		LS(VIVS_PS_NN_INST_ADDR, (inst_va >> 6) << 6);
		LS(VIVS_PS_UNK10A4, 0x0);
		/*
		 * No FLUSH_CACHE here! SMALL_BATCH doesn't stall the
		 * FE on NN_INST_ADDR on this chip, so FE would race
		 * ahead to FLUSH_CACHE while the NN is still running,
		 * causing the SH/FE to hang permanently. Instead we
		 * END here, poll for NN idle, then issue FLUSH_CACHE
		 * in a separate FE execution.
		 */
		cmd[i++] = VIV_FE_END_HEADER_OP_END;
		cmd[i++] = 0;

		buffer->user_size = i * 4;
	}
#undef LS

	prefetch = buffer->user_size / 8;
	cmd_va = etnaviv_cmdbuf_get_va(buffer,
				       &gpu->mmu_context->cmdbuf_mapping);

	/* Wait for FE to be fully idle before kicking */
	if (etnaviv_gpu_wait_idle(gpu, 200)) {
		dev_warn(gpu->dev,
			 "NN flop reset: FE not idle before kick, idle=0x%08x\n",
			 gpu_read(gpu, VIVS_HI_IDLE_STATE));
	}

	{
		u32 *cmd = buffer->vaddr;
		dev_info(gpu->dev,
			 "NN flop reset: cmd_va=0x%08x data_va=0x%08x idle=0x%08x prefetch=%u\n",
			 cmd_va, data_va, gpu_read(gpu, VIVS_HI_IDLE_STATE), prefetch);
		dev_info(gpu->dev,
			 "NN flop reset: cmd[0..7]=%08x %08x %08x %08x %08x %08x %08x %08x\n",
			 cmd[0], cmd[1], cmd[2], cmd[3],
			 cmd[4], cmd[5], cmd[6], cmd[7]);
	}

	etnaviv_gpu_start_fe(gpu, cmd_va, prefetch);

	/*
	 * Poll for NN idle. SMALL_BATCH doesn't stall the FE on this
	 * chip, so the FE reaches END while the NN is still running.
	 * We must wait for the NN to complete before issuing FLUSH_CACHE.
	 */
	for (i = 0; i < 50000; i++) {
		u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

		if (idle == 0x7FFFFFFF)
			break;
		udelay(10);
	}

	{
		u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);
		u32 dma = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);
		u8 *out = data_cpu + 0x300;

		dev_info(gpu->dev,
			 "NN flop reset: after NN idle=0x%08x dma=0x%08x loops=%d out=[%02x %02x %02x %02x %02x %02x %02x %02x]\n",
			 idle, dma, i, out[0], out[1], out[2], out[3],
			 out[4], out[5], out[6], out[7]);

		if (idle != 0x7FFFFFFF) {
			dev_warn(gpu->dev,
				 "NN flop reset: NN did not complete, mmu_status=0x%08x exc[0..3]=%08x %08x %08x %08x\n",
				 gpu_read(gpu, VIVS_MMUv2_STATUS),
				 gpu_read(gpu, VIVS_MMUv2_EXCEPTION_ADDR(0)),
				 gpu_read(gpu, VIVS_MMUv2_EXCEPTION_ADDR(1)),
				 gpu_read(gpu, VIVS_MMUv2_EXCEPTION_ADDR(2)),
				 gpu_read(gpu, VIVS_MMUv2_EXCEPTION_ADDR(3)));
			goto out_unmap;
		}
	}

	/*
	 * NN completed. Now issue FLUSH_CACHE in a separate FE
	 * execution — safe because SH is idle now.
	 */
	{
		u32 *cmd = buffer->vaddr;

		i = 0;
		cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |
			   VIV_FE_LOAD_STATE_HEADER_COUNT(1) |
			   VIV_FE_LOAD_STATE_HEADER_OFFSET(
				VIVS_GL_FLUSH_CACHE >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR);
		cmd[i++] = 0x00000C23;
		cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |
			   VIV_FE_LOAD_STATE_HEADER_COUNT(1) |
			   VIV_FE_LOAD_STATE_HEADER_OFFSET(
				VIVS_GL_FLUSH_CACHE >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR);
		cmd[i++] = 0x00000C23;
		cmd[i++] = VIV_FE_END_HEADER_OP_END;
		cmd[i++] = 0;

		buffer->user_size = i * 4;
		prefetch = buffer->user_size / 8;
		etnaviv_gpu_start_fe(gpu, cmd_va, prefetch);

		if (etnaviv_gpu_wait_idle(gpu, 200))
			dev_warn(gpu->dev,
				 "NN flop reset: FLUSH_CACHE timeout idle=0x%08x\n",
				 gpu_read(gpu, VIVS_HI_IDLE_STATE));
		else
			dev_info(gpu->dev, "NN flop reset: FLUSH_CACHE OK\n");
	}

	/*
	 * Second NN run: verify we can run another NN operation after
	 * FLUSH_CACHE. Reset output and re-run the same instruction.
	 */
	{
		u8 *out = data_cpu + 0x300;

		memset(data_cpu + 0x300, 0xBB, 64);
		wmb();

		/* Re-build the NNCmd (same as above) */
		i = 0;
#define LS(reg, val) do { \
	cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE | \
		   VIV_FE_LOAD_STATE_HEADER_COUNT(1) | \
		   VIV_FE_LOAD_STATE_HEADER_OFFSET( \
			(reg) >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR); \
	cmd[i++] = (val); \
} while (0)
		{
			u32 *cmd = buffer->vaddr;

			LS(VIVS_PA_SYSTEM_MODE, 0x11);
			LS(VIVS_GL_API_MODE, VIVS_GL_API_MODE_OPENCL);
			LS(VIVS_GL_SRAM_REMAP_ADDRESS, 0x00400000);
			LS(VIVS_GL_SRAM_REMAP_ADDRESS, 0x00400000);
			LS(VIVS_GL_OCB_REMAP_START, 0x0);
			LS(VIVS_GL_OCB_REMAP_END, 0x0);
			LS(VIVS_GL_NN_CONFIG, 0x10);
			LS(VIVS_GL_SHADER_INDEX, 0x0);
			LS(VIVS_PS_NN_INST_ADDR, (inst_va >> 6) << 6);
			LS(VIVS_PS_UNK10A4, 0x0);
			cmd[i++] = VIV_FE_END_HEADER_OP_END;
			cmd[i++] = 0;

			buffer->user_size = i * 4;
			prefetch = buffer->user_size / 8;
			etnaviv_gpu_start_fe(gpu, cmd_va, prefetch);
		}
#undef LS

		for (i = 0; i < 50000; i++) {
			u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);
			if (idle == 0x7FFFFFFF)
				break;
			udelay(10);
		}

		dev_info(gpu->dev,
			 "NN flop reset 2nd: idle=0x%08x loops=%d out=[%02x %02x %02x %02x]\n",
			 gpu_read(gpu, VIVS_HI_IDLE_STATE), i,
			 out[0], out[1], out[2], out[3]);

		/* FLUSH_CACHE after 2nd run */
		{
			u32 *cmd = buffer->vaddr;
			i = 0;
			cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |
				   VIV_FE_LOAD_STATE_HEADER_COUNT(1) |
				   VIV_FE_LOAD_STATE_HEADER_OFFSET(
					VIVS_GL_FLUSH_CACHE >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR);
			cmd[i++] = 0x00000C23;
			cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |
				   VIV_FE_LOAD_STATE_HEADER_COUNT(1) |
				   VIV_FE_LOAD_STATE_HEADER_OFFSET(
					VIVS_GL_FLUSH_CACHE >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR);
			cmd[i++] = 0x00000C23;
			cmd[i++] = VIV_FE_END_HEADER_OP_END;
			cmd[i++] = 0;

			buffer->user_size = i * 4;
			prefetch = buffer->user_size / 8;
			etnaviv_gpu_start_fe(gpu, cmd_va, prefetch);

			if (etnaviv_gpu_wait_idle(gpu, 200))
				dev_warn(gpu->dev,
					 "NN flop reset 2nd: FLUSH_CACHE timeout idle=0x%08x\n",
					 gpu_read(gpu, VIVS_HI_IDLE_STATE));
			else
				dev_info(gpu->dev, "NN flop reset 2nd: FLUSH_CACHE OK\n");
		}
	}

	/*
	 * Keep the NN data page mapped in GPU VA space. The NN/TP engines
	 * retain stale register state referencing the flop reset's VAs.
	 * If we unmap, subsequent mesa TP/NN operations hit MMU faults
	 * at these stale addresses (observed: fault at 0x00480400).
	 * The DMA-coherent page persists for the GPU's lifetime anyway.
	 */
	return;

out_unmap:
	context->global->ops->unmap(context, data_va, PAGE_SIZE);
out_remove:
	mutex_lock(&context->lock);
	drm_mm_remove_node(nn_node);
	mutex_unlock(&context->lock);
}

/*
 * TP flop reset: run a minimal TP transpose to initialize TP hardware state.
 * Required on VIPPico v3 (customerID 0x99) and VIPNano-SI+ (0x88) where
 * TP flip-flops are in undefined state after power-on.
 * Based on galcore's gckHARDWARE_ResetFlopWithTP.
 *
 * Data layout within the nn_data buffer (2 pages):
 *   0x400: TP instructions (8 cores x 128 bytes = 1024 bytes)
 *   0x800: TP input (64 bytes)
 *   0x840: TP output (64 bytes)
 *   0x1000: TP kernel (2048 bytes)
 */

/* TP dimensions: 1x1x2 input, 1x1x2 kernel (64 units), 1x64x1 output */
#define TP_CORE_COUNT		8

/*
 * BSP TP instruction template for VIPPico v3 (customerID 0x99, INT8).
 * 8 cores x 32 dwords each. Words [10], [11], [13] are patched with
 * GPU addresses for input, kernel, and output respectively.
 * Word [12] has the chain bit: 0xC0000002 for cores 0-6, 0x80000002 for core 7.
 * Kernel addresses differ per core (+0x100 each), output addresses differ (+8 each).
 */
static const u32 tp_flop_ins[256] = {
	/* Core 0 */
	0x00000001, 0x00020001, 0x00000001, 0x00000001,
	0x00000000, 0x00000000, 0xA0000E1B, 0x00000000,
	0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62000,
	0xC0000002, 0xDFF6E000, 0x00000000, 0x00000000,
	0x00010001, 0x00000000, 0x00000000, 0x00010001,
	0x00000001, 0x00000000, 0x00010008, 0x00000000,
	0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
	0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
	/* Core 1 */
	0x00000001, 0x00020001, 0x00000001, 0x00000001,
	0x00000000, 0x00000000, 0xA0000E1B, 0x00000000,
	0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62100,
	0xC0000002, 0xDFF6E008, 0x00000000, 0x00000000,
	0x00010001, 0x00000000, 0x00000000, 0x00010001,
	0x00000001, 0x00000000, 0x00010008, 0x00000000,
	0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
	0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
	/* Core 2 */
	0x00000001, 0x00020001, 0x00000001, 0x00000001,
	0x00000000, 0x00000000, 0xA0000E1B, 0x00000000,
	0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62200,
	0xC0000002, 0xDFF6E010, 0x00000000, 0x00000000,
	0x00010001, 0x00000000, 0x00000000, 0x00010001,
	0x00000001, 0x00000000, 0x00010008, 0x00000000,
	0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
	0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
	/* Core 3 */
	0x00000001, 0x00020001, 0x00000001, 0x00000001,
	0x00000000, 0x00000000, 0xA0000E1B, 0x00000000,
	0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62300,
	0xC0000002, 0xDFF6E018, 0x00000000, 0x00000000,
	0x00010001, 0x00000000, 0x00000000, 0x00010001,
	0x00000001, 0x00000000, 0x00010008, 0x00000000,
	0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
	0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
	/* Core 4 */
	0x00000001, 0x00020001, 0x00000001, 0x00000001,
	0x00000000, 0x00000000, 0xA0000E1B, 0x00000000,
	0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62400,
	0xC0000002, 0xDFF6E020, 0x00000000, 0x00000000,
	0x00010001, 0x00000000, 0x00000000, 0x00010001,
	0x00000001, 0x00000000, 0x00010008, 0x00000000,
	0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
	0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
	/* Core 5 */
	0x00000001, 0x00020001, 0x00000001, 0x00000001,
	0x00000000, 0x00000000, 0xA0000E1B, 0x00000000,
	0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62500,
	0xC0000002, 0xDFF6E028, 0x00000000, 0x00000000,
	0x00010001, 0x00000000, 0x00000000, 0x00010001,
	0x00000001, 0x00000000, 0x00010008, 0x00000000,
	0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
	0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
	/* Core 6 */
	0x00000001, 0x00020001, 0x00000001, 0x00000001,
	0x00000000, 0x00000000, 0xA0000E1B, 0x00000000,
	0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62600,
	0xC0000002, 0xDFF6E030, 0x00000000, 0x00000000,
	0x00010001, 0x00000000, 0x00000000, 0x00010001,
	0x00000001, 0x00000000, 0x00010008, 0x00000000,
	0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
	0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
	/* Core 7 (last: chain bit = 0) */
	0x00000001, 0x00020001, 0x00000001, 0x00000001,
	0x00000000, 0x00000000, 0xA0000E1B, 0x00000000,
	0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62700,
	0x80000002, 0xDFF6E038, 0x00000000, 0x00000000,
	0x00010001, 0x00000000, 0x00000000, 0x00010001,
	0x00000001, 0x00000000, 0x00010008, 0x00000000,
	0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
	0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
};

/*
 * BSP TP kernel data for VIPPico v3 (0x99).
 * 512 dwords = 2048 bytes (TPkerLen=0x800).
 * 8 blocks of 64 dwords each. Each block has a header word (0x21),
 * followed by zeros and two sets of {0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F}.
 */
static const u32 tp_flop_ker[512] = {
	/* Block 0 */
	0x00000021, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Block 1 */
	0x00000021, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Block 2 */
	0x00000021, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Block 3 */
	0x00000021, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Block 4 */
	0x00000021, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Block 5 */
	0x00000021, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Block 6 */
	0x00000021, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	/* Block 7 */
	0x00000021, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0xFFFFFFF0, 0xFFFFFFFF, 0x0000000F, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000,
};

static void etnaviv_gpu_tp_flop_reset(struct etnaviv_gpu *gpu)
{
	struct etnaviv_cmdbuf *buffer = &gpu->buffer;
	struct etnaviv_iommu_context *context = gpu->mmu_context;
	struct drm_mm_node *tp_node = &gpu->tp_flop_node;
	void *data_cpu = gpu->nn_data_vaddr;
	dma_addr_t data_paddr = gpu->nn_data_paddr;
	u32 data_va, inst_va, ker_va, in_va, out_va;
	u32 *inst;
	u16 prefetch;
	u32 cmd_va;
	u32 ker_anchor, out_anchor;
	int i, k, ret;

	if (gpu->identity.nn_core_count == 0 || !data_cpu)
		return;

	/* Only for customerID 0x99 (S905D3 VIPPico v3).
	 * A311D (0x88, VIPNano-QI) has 4 TP cores vs 8 in S905D3, with
	 * different TP instruction encoding.  The current tp_flop_ins[]
	 * template is S905D3-specific and crashes A311D by accessing
	 * nonexistent TP cores 4-7.  Disable until A311D template is added.
	 */
	if (gpu->identity.customer_id != 0x99)
		return;

	/*
	 * Flop reset is a one-time hardware init. The TP engine retains
	 * register state referencing the reserved VAs, so the mapping and
	 * its drm_mm node live in the gpu struct across the driver's
	 * lifetime. Use drm_mm_node_allocated() to detect previous success.
	 */
	if (drm_mm_node_allocated(tp_node))
		return;

	/*
	 * Map 2 pages into GPU VA space for TP data.
	 * Page 1: TP instructions (0x400), TP input (0x800), TP output (0x840)
	 * Page 2: TP kernel (0x1000)
	 */
	{
		u32 sram_end = 0x00400000 + gpu->identity.on_chip_sram_size;

		sram_end = ALIGN(sram_end, PAGE_SIZE);
		if (sram_end < ETNAVIV_SOFTPIN_START_ADDRESS)
			sram_end = ETNAVIV_SOFTPIN_START_ADDRESS;

		mutex_lock(&context->lock);
		ret = drm_mm_insert_node_in_range(&context->mm, tp_node,
						  ETNAVIV_FLOP_DATA_SIZE, 0, 0,
						  sram_end,
						  U64_MAX, DRM_MM_INSERT_LOW);
	}
	mutex_unlock(&context->lock);
	if (ret) {
		dev_err(gpu->dev, "TP flop reset: VA alloc failed: %d\n", ret);
		return;
	}

	data_va = tp_node->start;
	/* Map each page individually (IOMMU only supports SZ_4K per call) */
	ret = context->global->ops->map(context, data_va, data_paddr,
					SZ_4K,
					ETNAVIV_PROT_READ | ETNAVIV_PROT_WRITE);
	if (ret) {
		dev_err(gpu->dev, "TP flop reset: IOMMU map page 0 failed: %d\n", ret);
		goto out_remove;
	}
	ret = context->global->ops->map(context, data_va + SZ_4K,
					data_paddr + SZ_4K,
					SZ_4K,
					ETNAVIV_PROT_READ | ETNAVIV_PROT_WRITE);
	if (ret) {
		dev_err(gpu->dev, "TP flop reset: IOMMU map page 1 failed: %d\n", ret);
		goto out_unmap_p0;
	}

	inst_va = data_va + 0x400;
	in_va   = data_va + 0x800;
	out_va  = data_va + 0x840;
	ker_va  = data_va + 0x1000;

	/* Copy TP instructions and patch addresses */
	inst = data_cpu + 0x400;
	memcpy(inst, tp_flop_ins, sizeof(tp_flop_ins));

	/*
	 * Patch per-core addresses in TP instructions.
	 * BSP patches: word[10] = inputAddress (same for all cores)
	 *              word[11] = kernelAddress + (orig[11] - anchor)
	 *              word[13] = outputAddress + (orig[13] - anchor)
	 * Anchors are from the BSP template's core 0 values.
	 */
	ker_anchor = tp_flop_ins[11]; /* 0xDFF62000 */
	out_anchor = tp_flop_ins[13]; /* 0xDFF6E000 */
	for (k = 0; k < TP_CORE_COUNT; k++) {
		u32 orig_ker = tp_flop_ins[k * 32 + 11];
		u32 orig_out = tp_flop_ins[k * 32 + 13];

		inst[k * 32 + 10] = in_va;
		inst[k * 32 + 11] = ker_va + (orig_ker - ker_anchor);
		inst[k * 32 + 13] = out_va + (orig_out - out_anchor);
	}

	/* Copy TP kernel at offset 0x1000 */
	memcpy(data_cpu + 0x1000, tp_flop_ker, sizeof(tp_flop_ker));

	/* Fill TP input at offset 0x800: 2 bytes of 0x33 (1x1x2 INT8) */
	memset(data_cpu + 0x800, 0x33, 64);

	/* Pre-fill TP output at offset 0x840 with 0xCC to detect writes */
	memset(data_cpu + 0x840, 0xCC, 64);

	wmb();

	dev_info(gpu->dev,
		 "TP flop reset: inst_va=0x%08x ker_va=0x%08x in_va=0x%08x out_va=0x%08x\n",
		 inst_va, ker_va, in_va, out_va);

	/*
	 * Build the TPCmd in the ring buffer.
	 * For each core: SRAM_REMAP, OCB_REMAP_START/END, TP_CONFIG=2,
	 * UNK03950=8, SHADER_INDEX=0, TP_INST_ADDR with chain bit.
	 * Last core: UNK03950=0, no chain bit.
	 * Then: PS_UNK10A4=0, END (poll idle, then FLUSH_CACHE separately).
	 */
#define LS(reg, val) do {					\
	cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |	\
		   VIV_FE_LOAD_STATE_HEADER_COUNT(1) |		\
		   VIV_FE_LOAD_STATE_HEADER_OFFSET(		\
			(reg) >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR); \
	cmd[i++] = (val);					\
} while (0)

	{
		u32 *cmd = buffer->vaddr;
		u32 core_inst_va;

		i = 0;

		/*
		 * HI_COMPRESSION_FLAGS and MMU TLB flush, same as NN reset.
		 */
		LS(VIVS_HI_COMPRESSION_FLAGS, 0x40);
		LS(VIVS_MMUv2_CONFIGURATION,
		   VIVS_MMUv2_CONFIGURATION_MODE_MASK |
		   VIVS_MMUv2_CONFIGURATION_FLUSH_FLUSH |
		   etnaviv_iommuv2_get_mtlb_addr(context));
		cmd[i++] = VIV_FE_WAIT_HEADER_OP_WAIT | 32000;
		cmd[i++] = 0;
		LS(VIVS_PA_SYSTEM_MODE, 0x11);
		LS(VIVS_GL_API_MODE, VIVS_GL_API_MODE_OPENCL);

		for (k = 0; k < TP_CORE_COUNT; k++) {
			LS(VIVS_GL_SRAM_REMAP_ADDRESS, 0x00400000);
			LS(VIVS_GL_OCB_REMAP_START, 0x0);
			LS(VIVS_GL_OCB_REMAP_END, 0x0);
			LS(VIVS_GL_TP_CONFIG, 0x2);

			if (k < TP_CORE_COUNT - 1) {
				LS(VIVS_GL_UNK03950, 0x8);
				LS(VIVS_GL_SHADER_INDEX, 0x0);

				/* TP_INST_ADDR with chain bit for cores 0..6 */
				core_inst_va = ((inst_va + k * 128) & 0xFFFFFFC0) | 0x1;
				LS(VIVS_PS_TP_INST_ADDR, core_inst_va);
			} else {
				/* Last core: UNK03950=0, no chain bit */
				LS(VIVS_GL_UNK03950, 0x0);
				LS(VIVS_GL_SHADER_INDEX, 0x0);

				core_inst_va = (inst_va + k * 128) & 0xFFFFFFC0;
				LS(VIVS_PS_TP_INST_ADDR, core_inst_va);
			}
		}

		LS(VIVS_PS_UNK10A4, 0x0);
		cmd[i++] = VIV_FE_END_HEADER_OP_END;
		cmd[i++] = 0;

		buffer->user_size = i * 4;
	}
#undef LS

	prefetch = buffer->user_size / 8;
	cmd_va = etnaviv_cmdbuf_get_va(buffer,
				       &gpu->mmu_context->cmdbuf_mapping);

	if (etnaviv_gpu_wait_idle(gpu, 200)) {
		dev_warn(gpu->dev,
			 "TP flop reset: FE not idle before kick, idle=0x%08x\n",
			 gpu_read(gpu, VIVS_HI_IDLE_STATE));
	}

	dev_info(gpu->dev,
		 "TP flop reset: cmd_va=0x%08x idle=0x%08x prefetch=%u\n",
		 cmd_va, gpu_read(gpu, VIVS_HI_IDLE_STATE), prefetch);

	etnaviv_gpu_start_fe(gpu, cmd_va, prefetch);

	/* Poll for TP idle */
	for (i = 0; i < 50000; i++) {
		u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

		if (idle == 0x7FFFFFFF)
			break;
		udelay(10);
	}

	{
		u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);
		u8 *out = data_cpu + 0x840;

		dev_info(gpu->dev,
			 "TP flop reset: idle=0x%08x loops=%d out=[%02x %02x %02x %02x %02x %02x %02x %02x]\n",
			 idle, i, out[0], out[1], out[2], out[3],
			 out[4], out[5], out[6], out[7]);

		if (idle != 0x7FFFFFFF) {
			dev_warn(gpu->dev,
				 "TP flop reset: TP did not complete, mmu_status=0x%08x exc[0..3]=%08x %08x %08x %08x\n",
				 gpu_read(gpu, VIVS_MMUv2_STATUS),
				 gpu_read(gpu, VIVS_MMUv2_EXCEPTION_ADDR(0)),
				 gpu_read(gpu, VIVS_MMUv2_EXCEPTION_ADDR(1)),
				 gpu_read(gpu, VIVS_MMUv2_EXCEPTION_ADDR(2)),
				 gpu_read(gpu, VIVS_MMUv2_EXCEPTION_ADDR(3)));
			goto out_unmap;
		}
	}

	/* FLUSH_CACHE after TP completion */
	{
		u32 *cmd = buffer->vaddr;

		i = 0;
		cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |
			   VIV_FE_LOAD_STATE_HEADER_COUNT(1) |
			   VIV_FE_LOAD_STATE_HEADER_OFFSET(
				VIVS_GL_FLUSH_CACHE >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR);
		cmd[i++] = 0x00000C23;
		cmd[i++] = VIV_FE_LOAD_STATE_HEADER_OP_LOAD_STATE |
			   VIV_FE_LOAD_STATE_HEADER_COUNT(1) |
			   VIV_FE_LOAD_STATE_HEADER_OFFSET(
				VIVS_GL_FLUSH_CACHE >> VIV_FE_LOAD_STATE_HEADER_OFFSET__SHR);
		cmd[i++] = 0x00000C23;
		cmd[i++] = VIV_FE_END_HEADER_OP_END;
		cmd[i++] = 0;

		buffer->user_size = i * 4;
		prefetch = buffer->user_size / 8;
		etnaviv_gpu_start_fe(gpu, cmd_va, prefetch);

		if (etnaviv_gpu_wait_idle(gpu, 200))
			dev_warn(gpu->dev,
				 "TP flop reset: FLUSH_CACHE timeout idle=0x%08x\n",
				 gpu_read(gpu, VIVS_HI_IDLE_STATE));
		else
			dev_info(gpu->dev, "TP flop reset: FLUSH_CACHE OK\n");
	}

	/*
	 * Keep TP data pages mapped. gpu->tp_flop_node is persistent, so
	 * the drm_mm rbtree has a valid long-lived pointer that will not
	 * be invalidated by stack reuse. The node is detected on the next
	 * call via drm_mm_node_allocated() and the function returns early.
	 */
	return;

out_unmap:
	context->global->ops->unmap(context, data_va + SZ_4K, SZ_4K);
out_unmap_p0:
	context->global->ops->unmap(context, data_va, SZ_4K);
out_remove:
	mutex_lock(&context->lock);
	drm_mm_remove_node(tp_node);
	mutex_unlock(&context->lock);
}

static void etnaviv_gpu_start_fe_idleloop(struct etnaviv_gpu *gpu,
					  struct etnaviv_iommu_context *context)
{
	u16 prefetch;
	u32 address;

	WARN_ON(gpu->state != ETNA_GPU_STATE_INITIALIZED);

	/* setup the MMU */
	etnaviv_iommu_restore(gpu, context);

	/* Run NN + TP flop reset on NPU devices before starting normal FE loop */
	etnaviv_gpu_nn_flop_reset(gpu);
	etnaviv_gpu_tp_flop_reset(gpu);

	/* Start command processor */
	prefetch = etnaviv_buffer_init(gpu);
	address = etnaviv_cmdbuf_get_va(&gpu->buffer,
					&gpu->mmu_context->cmdbuf_mapping);

	etnaviv_gpu_start_fe(gpu, address, prefetch);

	gpu->state = ETNA_GPU_STATE_RUNNING;
}

static void etnaviv_gpu_setup_pulse_eater(struct etnaviv_gpu *gpu)
{
	/*
	 * Base value for VIVS_PM_PULSE_EATER register on models where it
	 * cannot be read, extracted from vivante kernel driver.
	 */
	u32 pulse_eater = 0x01590880;

	if (etnaviv_is_model_rev(gpu, 0x4000, 0x5208) ||
	    etnaviv_is_model_rev(gpu, 0x4000, 0x5222)) {
		pulse_eater |= BIT(23);

	}

	if (etnaviv_is_model_rev(gpu, 0x1000, 0x5039) ||
	    etnaviv_is_model_rev(gpu, 0x1000, 0x5040)) {
		pulse_eater &= ~BIT(16);
		pulse_eater |= BIT(17);
	}

	if ((gpu->identity.revision > 0x5420) &&
	    (gpu->identity.features & chipFeatures_PIPE_3D))
	{
		/* Performance fix: disable internal DFS */
		pulse_eater = gpu_read_power(gpu, VIVS_PM_PULSE_EATER);
		pulse_eater |= BIT(18);
	}

	gpu_write_power(gpu, VIVS_PM_PULSE_EATER, pulse_eater);
}

static void etnaviv_gpu_hw_init(struct etnaviv_gpu *gpu)
{
	WARN_ON(!(gpu->state == ETNA_GPU_STATE_IDENTIFIED ||
		  gpu->state == ETNA_GPU_STATE_RESET));

	if ((etnaviv_is_model_rev(gpu, 0x320, 0x5007) ||
	     etnaviv_is_model_rev(gpu, 0x320, 0x5220)) &&
	    gpu_read(gpu, VIVS_HI_CHIP_TIME) != 0x2062400) {
		u32 mc_memory_debug;

		mc_memory_debug = gpu_read(gpu, VIVS_MC_DEBUG_MEMORY) & ~0xff;

		if (gpu->identity.revision == 0x5007)
			mc_memory_debug |= 0x0c;
		else
			mc_memory_debug |= 0x08;

		gpu_write(gpu, VIVS_MC_DEBUG_MEMORY, mc_memory_debug);
	}

	/* enable module-level clock gating */
	etnaviv_gpu_enable_mlcg(gpu);

	/*
	 * Update GPU AXI cache atttribute to "cacheable, no allocate".
	 * This is necessary to prevent the iMX6 SoC locking up.
	 */
	gpu_write(gpu, VIVS_HI_AXI_CONFIG,
		  VIVS_HI_AXI_CONFIG_AWCACHE(2) |
		  VIVS_HI_AXI_CONFIG_ARCACHE(2));

	/* GC2000 rev 5108 needs a special bus config */
	if (etnaviv_is_model_rev(gpu, 0x2000, 0x5108)) {
		u32 bus_config = gpu_read(gpu, VIVS_MC_BUS_CONFIG);
		bus_config &= ~(VIVS_MC_BUS_CONFIG_FE_BUS_CONFIG__MASK |
				VIVS_MC_BUS_CONFIG_TX_BUS_CONFIG__MASK);
		bus_config |= VIVS_MC_BUS_CONFIG_FE_BUS_CONFIG(1) |
			      VIVS_MC_BUS_CONFIG_TX_BUS_CONFIG(0);
		gpu_write(gpu, VIVS_MC_BUS_CONFIG, bus_config);
	}

	if (gpu->sec_mode == ETNA_SEC_KERNEL) {
		/* Deassert AHB reset (set during etnaviv_hw_reset), then
		 * enable non-secure AHB access. Galcore does:
		 * write 1 (reset), write 0 (deassert), write 2 (nonsec).
		 */
		gpu_write(gpu, VIVS_MMUv2_AHB_CONTROL, 0);
		gpu_write(gpu, VIVS_MMUv2_AHB_CONTROL,
			  VIVS_MMUv2_AHB_CONTROL_NONSEC_ACCESS);
	}

	/* setup the pulse eater */
	etnaviv_gpu_setup_pulse_eater(gpu);

	gpu_write(gpu, VIVS_HI_INTR_ENBL, ~0U);

	gpu->state = ETNA_GPU_STATE_INITIALIZED;
}

static void etnaviv_gpu_npu_poll_work(struct work_struct *work);

int etnaviv_gpu_init(struct etnaviv_gpu *gpu)
{
	struct etnaviv_drm_private *priv = gpu->drm->dev_private;
	dma_addr_t cmdbuf_paddr;
	int ret, i;

	dev_info(gpu->dev, "gpu_init: pm_runtime_get_sync\n");
	ret = pm_runtime_get_sync(gpu->dev);
	if (ret < 0) {
		dev_err(gpu->dev, "Failed to enable GPU power domain\n");
		goto pm_put;
	}
	dev_info(gpu->dev, "gpu_init: pm_runtime done ret=%d\n", ret);

	dev_info(gpu->dev, "gpu_init: hw_identify start\n");
	etnaviv_hw_identify(gpu);
	dev_info(gpu->dev, "gpu_init: hw_identify done model=0x%x rev=0x%x\n",
		 gpu->identity.model, gpu->identity.revision);

	if (gpu->identity.model == 0) {
		dev_err(gpu->dev, "Unknown GPU model\n");
		ret = -ENXIO;
		goto fail;
	}

	if (gpu->identity.nn_core_count > 0)
		dev_warn(gpu->dev, "etnaviv has been instantiated on a NPU, "
                                   "for which the UAPI is still experimental\n");

	/* Exclude VG cores with FE2.0 */
	if (gpu->identity.features & chipFeatures_PIPE_VG &&
	    gpu->identity.features & chipFeatures_FE20) {
		dev_info(gpu->dev, "Ignoring GPU with VG and FE2.0\n");
		ret = -ENXIO;
		goto fail;
	}

	/*
	 * On cores with security features supported, we claim control over the
	 * security states.
	 */
	if ((gpu->identity.minor_features7 & chipMinorFeatures7_BIT_SECURITY) &&
	    (gpu->identity.minor_features10 & chipMinorFeatures10_SECURITY_AHB))
		gpu->sec_mode = ETNA_SEC_KERNEL;

	gpu->state = ETNA_GPU_STATE_IDENTIFIED;

	ret = etnaviv_hw_reset(gpu);
	if (ret) {
		dev_err(gpu->dev, "GPU reset failed\n");
		goto fail;
	}

	ret = etnaviv_iommu_global_init(gpu);
	if (ret)
		goto fail;

	/* Create buffer: */
	ret = etnaviv_cmdbuf_init(priv->cmdbuf_suballoc, &gpu->buffer,
				  PAGE_SIZE);
	if (ret) {
		dev_err(gpu->dev, "could not create command buffer\n");
		goto fail;
	}

	/* Allocate NN/TP flop reset data buffer for NPU devices */
	if (gpu->identity.nn_core_count > 0) {
		gpu->nn_data_vaddr = dma_alloc_wc(gpu->drm->dev,
						  ETNAVIV_FLOP_DATA_SIZE,
						  &gpu->nn_data_paddr,
						  GFP_KERNEL);
		if (!gpu->nn_data_vaddr) {
			dev_err(gpu->dev, "could not alloc NN/TP data buffer\n");
			ret = -ENOMEM;
			goto fail;
		}
	}

	/*
	 * Set the GPU linear window to cover the cmdbuf region, as the GPU
	 * won't be able to start execution otherwise. The alignment to 128M is
	 * chosen arbitrarily but helps in debugging, as the MMU offset
	 * calculations are much more straight forward this way.
	 *
	 * On MC1.0 cores the linear window offset is ignored by the TS engine,
	 * leading to inconsistent memory views. Avoid using the offset on those
	 * cores if possible, otherwise disable the TS feature. MMUv2 doesn't
	 * expose this issue, as all TS accesses are MMU translated, so the
	 * linear window offset won't be used.
	 */
	cmdbuf_paddr = ALIGN_DOWN(etnaviv_cmdbuf_get_pa(&gpu->buffer), SZ_128M);

	if (!(gpu->identity.features & chipFeatures_PIPE_3D) ||
	    (gpu->identity.minor_features0 & chipMinorFeatures0_MC20) ||
	    (gpu->identity.minor_features1 & chipMinorFeatures1_MMU_VERSION)) {
		if (cmdbuf_paddr >= SZ_2G)
			priv->mmu_global->memory_base = SZ_2G;
		else
			priv->mmu_global->memory_base = cmdbuf_paddr;
	} else if (cmdbuf_paddr + SZ_128M >= SZ_2G) {
		dev_info(gpu->dev,
			 "Need to move linear window on MC1.0, disabling TS\n");
		gpu->identity.features &= ~chipFeatures_FAST_CLEAR;
		priv->mmu_global->memory_base = SZ_2G;
	}

	/* Setup event management */
	spin_lock_init(&gpu->event_spinlock);
	init_completion(&gpu->event_free);
	bitmap_zero(gpu->event_bitmap, ETNA_NR_EVENTS);
	for (i = 0; i < ARRAY_SIZE(gpu->event); i++)
		complete(&gpu->event_free);

	/*
	 * NPU completion strategy:
	 *
	 * All NPU chips use GL_EVENT FROM_FE for interrupt delivery.
	 *
	 * VIPNano-QI (A311D, customer_id 0x88): SMALL_BATCH stalls FE
	 * at NN_INST_ADDR until NN completes. GL_EVENT fires at the
	 * right time. ISR signals fence immediately (npu_poll=false).
	 *
	 * VIPPico v3 (S905D3, customer_id 0x99): SMALL_BATCH does NOT
	 * stall FE. GL_EVENT fires before NN completes. ISR defers
	 * fence to npu_poll_work which waits for NN/TP idle (npu_poll=true).
	 */
	if (gpu->identity.nn_core_count > 0) {
		if (gpu->identity.customer_id == 0x99) {
			gpu->npu_poll_completion = true;
			dev_info(gpu->dev,
				 "NPU: GL_EVENT + deferred poll (VIPPico v3)\n");
		} else {
			gpu->npu_poll_completion = false;
			dev_info(gpu->dev,
				 "NPU: GL_EVENT immediate (FE stalls)\n");
		}
		gpu->npu_poll_event = -1;
		INIT_DELAYED_WORK(&gpu->npu_poll_work,
				  etnaviv_gpu_npu_poll_work);
	}

	/* Now program the hardware */
	mutex_lock(&gpu->lock);
	etnaviv_gpu_hw_init(gpu);
	mutex_unlock(&gpu->lock);

	pm_runtime_mark_last_busy(gpu->dev);
	pm_runtime_put_autosuspend(gpu->dev);

	return 0;

fail:
	pm_runtime_mark_last_busy(gpu->dev);
pm_put:
	pm_runtime_put_autosuspend(gpu->dev);

	return ret;
}

#ifdef CONFIG_DEBUG_FS
struct dma_debug {
	u32 address[2];
	u32 state[2];
};

static void verify_dma(struct etnaviv_gpu *gpu, struct dma_debug *debug)
{
	u32 i;

	debug->address[0] = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);
	debug->state[0]   = gpu_read(gpu, VIVS_FE_DMA_DEBUG_STATE);

	for (i = 0; i < 500; i++) {
		debug->address[1] = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);
		debug->state[1]   = gpu_read(gpu, VIVS_FE_DMA_DEBUG_STATE);

		if (debug->address[0] != debug->address[1])
			break;

		if (debug->state[0] != debug->state[1])
			break;
	}
}

int etnaviv_gpu_debugfs(struct etnaviv_gpu *gpu, struct seq_file *m)
{
	struct dma_debug debug;
	u32 dma_lo, dma_hi, axi, idle;
	int ret;

	seq_printf(m, "%s Status:\n", dev_name(gpu->dev));

	ret = pm_runtime_get_sync(gpu->dev);
	if (ret < 0)
		goto pm_put;

	dma_lo = gpu_read(gpu, VIVS_FE_DMA_LOW);
	dma_hi = gpu_read(gpu, VIVS_FE_DMA_HIGH);
	axi = gpu_read(gpu, VIVS_HI_AXI_STATUS);
	idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

	verify_dma(gpu, &debug);

	seq_puts(m, "\tidentity\n");
	seq_printf(m, "\t model: 0x%x\n", gpu->identity.model);
	seq_printf(m, "\t revision: 0x%x\n", gpu->identity.revision);
	seq_printf(m, "\t product_id: 0x%x\n", gpu->identity.product_id);
	seq_printf(m, "\t customer_id: 0x%x\n", gpu->identity.customer_id);
	seq_printf(m, "\t eco_id: 0x%x\n", gpu->identity.eco_id);

	seq_puts(m, "\tfeatures\n");
	seq_printf(m, "\t major_features: 0x%08x\n",
		   gpu->identity.features);
	seq_printf(m, "\t minor_features0: 0x%08x\n",
		   gpu->identity.minor_features0);
	seq_printf(m, "\t minor_features1: 0x%08x\n",
		   gpu->identity.minor_features1);
	seq_printf(m, "\t minor_features2: 0x%08x\n",
		   gpu->identity.minor_features2);
	seq_printf(m, "\t minor_features3: 0x%08x\n",
		   gpu->identity.minor_features3);
	seq_printf(m, "\t minor_features4: 0x%08x\n",
		   gpu->identity.minor_features4);
	seq_printf(m, "\t minor_features5: 0x%08x\n",
		   gpu->identity.minor_features5);
	seq_printf(m, "\t minor_features6: 0x%08x\n",
		   gpu->identity.minor_features6);
	seq_printf(m, "\t minor_features7: 0x%08x\n",
		   gpu->identity.minor_features7);
	seq_printf(m, "\t minor_features8: 0x%08x\n",
		   gpu->identity.minor_features8);
	seq_printf(m, "\t minor_features9: 0x%08x\n",
		   gpu->identity.minor_features9);
	seq_printf(m, "\t minor_features10: 0x%08x\n",
		   gpu->identity.minor_features10);
	seq_printf(m, "\t minor_features11: 0x%08x\n",
		   gpu->identity.minor_features11);

	seq_puts(m, "\tspecs\n");
	seq_printf(m, "\t stream_count:  %d\n",
			gpu->identity.stream_count);
	seq_printf(m, "\t register_max: %d\n",
			gpu->identity.register_max);
	seq_printf(m, "\t thread_count: %d\n",
			gpu->identity.thread_count);
	seq_printf(m, "\t vertex_cache_size: %d\n",
			gpu->identity.vertex_cache_size);
	seq_printf(m, "\t shader_core_count: %d\n",
			gpu->identity.shader_core_count);
	seq_printf(m, "\t nn_core_count: %d\n",
			gpu->identity.nn_core_count);
	seq_printf(m, "\t pixel_pipes: %d\n",
			gpu->identity.pixel_pipes);
	seq_printf(m, "\t vertex_output_buffer_size: %d\n",
			gpu->identity.vertex_output_buffer_size);
	seq_printf(m, "\t buffer_size: %d\n",
			gpu->identity.buffer_size);
	seq_printf(m, "\t instruction_count: %d\n",
			gpu->identity.instruction_count);
	seq_printf(m, "\t num_constants: %d\n",
			gpu->identity.num_constants);
	seq_printf(m, "\t varyings_count: %d\n",
			gpu->identity.varyings_count);

	seq_printf(m, "\taxi: 0x%08x\n", axi);
	seq_printf(m, "\tidle: 0x%08x\n", idle);
	idle |= ~gpu->idle_mask & ~VIVS_HI_IDLE_STATE_AXI_LP;
	if ((idle & VIVS_HI_IDLE_STATE_FE) == 0)
		seq_puts(m, "\t FE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_DE) == 0)
		seq_puts(m, "\t DE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_PE) == 0)
		seq_puts(m, "\t PE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_SH) == 0)
		seq_puts(m, "\t SH is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_PA) == 0)
		seq_puts(m, "\t PA is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_SE) == 0)
		seq_puts(m, "\t SE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_RA) == 0)
		seq_puts(m, "\t RA is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_TX) == 0)
		seq_puts(m, "\t TX is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_VG) == 0)
		seq_puts(m, "\t VG is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_IM) == 0)
		seq_puts(m, "\t IM is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_FP) == 0)
		seq_puts(m, "\t FP is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_TS) == 0)
		seq_puts(m, "\t TS is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_BL) == 0)
		seq_puts(m, "\t BL is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_ASYNCFE) == 0)
		seq_puts(m, "\t ASYNCFE is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_MC) == 0)
		seq_puts(m, "\t MC is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_PPA) == 0)
		seq_puts(m, "\t PPA is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_WD) == 0)
		seq_puts(m, "\t WD is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_NN) == 0)
		seq_puts(m, "\t NN is not idle\n");
	if ((idle & VIVS_HI_IDLE_STATE_TP) == 0)
		seq_puts(m, "\t TP is not idle\n");
	if (idle & VIVS_HI_IDLE_STATE_AXI_LP)
		seq_puts(m, "\t AXI low power mode\n");

	if (gpu->identity.features & chipFeatures_DEBUG_MODE) {
		u32 read0 = gpu_read(gpu, VIVS_MC_DEBUG_READ0);
		u32 read1 = gpu_read(gpu, VIVS_MC_DEBUG_READ1);
		u32 write = gpu_read(gpu, VIVS_MC_DEBUG_WRITE);

		seq_puts(m, "\tMC\n");
		seq_printf(m, "\t read0: 0x%08x\n", read0);
		seq_printf(m, "\t read1: 0x%08x\n", read1);
		seq_printf(m, "\t write: 0x%08x\n", write);
	}

	seq_puts(m, "\tDMA ");

	if (debug.address[0] == debug.address[1] &&
	    debug.state[0] == debug.state[1]) {
		seq_puts(m, "seems to be stuck\n");
	} else if (debug.address[0] == debug.address[1]) {
		seq_puts(m, "address is constant\n");
	} else {
		seq_puts(m, "is running\n");
	}

	seq_printf(m, "\t address 0: 0x%08x\n", debug.address[0]);
	seq_printf(m, "\t address 1: 0x%08x\n", debug.address[1]);
	seq_printf(m, "\t state 0: 0x%08x\n", debug.state[0]);
	seq_printf(m, "\t state 1: 0x%08x\n", debug.state[1]);
	seq_printf(m, "\t last fetch 64 bit word: 0x%08x 0x%08x\n",
		   dma_lo, dma_hi);

	ret = 0;

	pm_runtime_mark_last_busy(gpu->dev);
pm_put:
	pm_runtime_put_autosuspend(gpu->dev);

	return ret;
}
#endif

/* fence object management */
struct etnaviv_fence {
	struct etnaviv_gpu *gpu;
	struct dma_fence base;
};

static inline struct etnaviv_fence *to_etnaviv_fence(struct dma_fence *fence)
{
	return container_of(fence, struct etnaviv_fence, base);
}

static const char *etnaviv_fence_get_driver_name(struct dma_fence *fence)
{
	return "etnaviv";
}

static const char *etnaviv_fence_get_timeline_name(struct dma_fence *fence)
{
	struct etnaviv_fence *f = to_etnaviv_fence(fence);

	return dev_name(f->gpu->dev);
}

static bool etnaviv_fence_signaled(struct dma_fence *fence)
{
	struct etnaviv_fence *f = to_etnaviv_fence(fence);

	return (s32)(f->gpu->completed_fence - f->base.seqno) >= 0;
}

static void etnaviv_fence_release(struct dma_fence *fence)
{
	struct etnaviv_fence *f = to_etnaviv_fence(fence);

	kfree_rcu(f, base.rcu);
}

static const struct dma_fence_ops etnaviv_fence_ops = {
	.get_driver_name = etnaviv_fence_get_driver_name,
	.get_timeline_name = etnaviv_fence_get_timeline_name,
	.signaled = etnaviv_fence_signaled,
	.release = etnaviv_fence_release,
};

static struct dma_fence *etnaviv_gpu_fence_alloc(struct etnaviv_gpu *gpu)
{
	struct etnaviv_fence *f;

	/*
	 * GPU lock must already be held, otherwise fence completion order might
	 * not match the seqno order assigned here.
	 */
	lockdep_assert_held(&gpu->lock);

	f = kzalloc(sizeof(*f), GFP_KERNEL);
	if (!f)
		return NULL;

	f->gpu = gpu;

	dma_fence_init(&f->base, &etnaviv_fence_ops, &gpu->fence_spinlock,
		       gpu->fence_context, ++gpu->next_fence);

	return &f->base;
}

/* returns true if fence a comes after fence b */
static inline bool fence_after(u32 a, u32 b)
{
	return (s32)(a - b) > 0;
}

/*
 * event management:
 */

static int event_alloc(struct etnaviv_gpu *gpu, unsigned nr_events,
	unsigned int *events)
{
	unsigned long timeout = msecs_to_jiffies(10 * 10000);
	unsigned i, acquired = 0, rpm_count = 0;
	int ret;

	for (i = 0; i < nr_events; i++) {
		unsigned long remaining;

		remaining = wait_for_completion_timeout(&gpu->event_free, timeout);

		if (!remaining) {
			dev_err(gpu->dev, "wait_for_completion_timeout failed");
			ret = -EBUSY;
			goto out;
		}

		acquired++;
		timeout = remaining;
	}

	spin_lock(&gpu->event_spinlock);

	for (i = 0; i < nr_events; i++) {
		int event = find_first_zero_bit(gpu->event_bitmap, ETNA_NR_EVENTS);

		events[i] = event;
		memset(&gpu->event[event], 0, sizeof(struct etnaviv_event));
		set_bit(event, gpu->event_bitmap);
	}

	spin_unlock(&gpu->event_spinlock);

	for (i = 0; i < nr_events; i++) {
		ret = pm_runtime_resume_and_get(gpu->dev);
		if (ret)
			goto out_rpm;
		rpm_count++;
	}

	return 0;

out_rpm:
	for (i = 0; i < rpm_count; i++)
		pm_runtime_put_autosuspend(gpu->dev);
out:
	for (i = 0; i < acquired; i++)
		complete(&gpu->event_free);

	return ret;
}

static void event_free(struct etnaviv_gpu *gpu, unsigned int event)
{
	if (!test_bit(event, gpu->event_bitmap)) {
		dev_warn(gpu->dev, "event %u is already marked as free",
			 event);
	} else {
		clear_bit(event, gpu->event_bitmap);
		complete(&gpu->event_free);
	}

	pm_runtime_put_autosuspend(gpu->dev);
}

/*
 * Cmdstream submission/retirement:
 */
int etnaviv_gpu_wait_fence_interruptible(struct etnaviv_gpu *gpu,
	u32 id, struct drm_etnaviv_timespec *timeout)
{
	struct dma_fence *fence;
	int ret;

	/*
	 * Look up the fence and take a reference. We might still find a fence
	 * whose refcount has already dropped to zero. dma_fence_get_rcu
	 * pretends we didn't find a fence in that case.
	 */
	rcu_read_lock();
	fence = xa_load(&gpu->user_fences, id);
	if (fence)
		fence = dma_fence_get_rcu(fence);
	rcu_read_unlock();

	if (!fence)
		return 0;

	if (!timeout) {
		/* No timeout was requested: just test for completion */
		ret = dma_fence_is_signaled(fence) ? 0 : -EBUSY;
	} else {
		unsigned long remaining = etnaviv_timeout_to_jiffies(timeout);

		ret = dma_fence_wait_timeout(fence, true, remaining);
		if (ret == 0)
			ret = -ETIMEDOUT;
		else if (ret != -ERESTARTSYS)
			ret = 0;

	}

	dma_fence_put(fence);
	return ret;
}

/*
 * Wait for an object to become inactive.  This, on it's own, is not race
 * free: the object is moved by the scheduler off the active list, and
 * then the iova is put.  Moreover, the object could be re-submitted just
 * after we notice that it's become inactive.
 *
 * Although the retirement happens under the gpu lock, we don't want to hold
 * that lock in this function while waiting.
 */
int etnaviv_gpu_wait_obj_inactive(struct etnaviv_gpu *gpu,
	struct etnaviv_gem_object *etnaviv_obj,
	struct drm_etnaviv_timespec *timeout)
{
	unsigned long remaining;
	long ret;

	if (!timeout)
		return !is_active(etnaviv_obj) ? 0 : -EBUSY;

	remaining = etnaviv_timeout_to_jiffies(timeout);

	ret = wait_event_interruptible_timeout(gpu->fence_event,
					       !is_active(etnaviv_obj),
					       remaining);
	if (ret > 0)
		return 0;
	else if (ret == -ERESTARTSYS)
		return -ERESTARTSYS;
	else
		return -ETIMEDOUT;
}

static void sync_point_perfmon_sample(struct etnaviv_gpu *gpu,
	struct etnaviv_event *event, unsigned int flags)
{
	const struct etnaviv_gem_submit *submit = event->submit;
	unsigned int i;

	for (i = 0; i < submit->nr_pmrs; i++) {
		const struct etnaviv_perfmon_request *pmr = submit->pmrs + i;

		if (pmr->flags == flags)
			etnaviv_perfmon_process(gpu, pmr, submit->exec_state);
	}
}

static void sync_point_perfmon_sample_pre(struct etnaviv_gpu *gpu,
	struct etnaviv_event *event)
{
	u32 val;

	mutex_lock(&gpu->lock);

	/* disable clock gating */
	val = gpu_read_power(gpu, VIVS_PM_POWER_CONTROLS);
	val &= ~VIVS_PM_POWER_CONTROLS_ENABLE_MODULE_CLOCK_GATING;
	gpu_write_power(gpu, VIVS_PM_POWER_CONTROLS, val);

	/* enable debug register */
	val = gpu_read(gpu, VIVS_HI_CLOCK_CONTROL);
	val &= ~VIVS_HI_CLOCK_CONTROL_DISABLE_DEBUG_REGISTERS;
	gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, val);

	sync_point_perfmon_sample(gpu, event, ETNA_PM_PROCESS_PRE);

	mutex_unlock(&gpu->lock);
}

static void sync_point_perfmon_sample_post(struct etnaviv_gpu *gpu,
	struct etnaviv_event *event)
{
	const struct etnaviv_gem_submit *submit = event->submit;
	unsigned int i;
	u32 val;

	mutex_lock(&gpu->lock);

	sync_point_perfmon_sample(gpu, event, ETNA_PM_PROCESS_POST);

	/* disable debug register */
	val = gpu_read(gpu, VIVS_HI_CLOCK_CONTROL);
	val |= VIVS_HI_CLOCK_CONTROL_DISABLE_DEBUG_REGISTERS;
	gpu_write(gpu, VIVS_HI_CLOCK_CONTROL, val);

	/* enable clock gating */
	val = gpu_read_power(gpu, VIVS_PM_POWER_CONTROLS);
	val |= VIVS_PM_POWER_CONTROLS_ENABLE_MODULE_CLOCK_GATING;
	gpu_write_power(gpu, VIVS_PM_POWER_CONTROLS, val);

	mutex_unlock(&gpu->lock);

	for (i = 0; i < submit->nr_pmrs; i++) {
		const struct etnaviv_perfmon_request *pmr = submit->pmrs + i;

		*pmr->bo_vma = pmr->sequence;
	}
}


/* add bo's to gpu's ring, and kick gpu: */
struct dma_fence *etnaviv_gpu_submit(struct etnaviv_gem_submit *submit)
{
	struct etnaviv_gpu *gpu = submit->gpu;
	struct dma_fence *gpu_fence;
	unsigned int i, nr_events = 1, event[3];
	int ret;

	/*
	 * if there are performance monitor requests we need to have
	 * - a sync point to re-configure gpu and process ETNA_PM_PROCESS_PRE
	 *   requests.
	 * - a sync point to re-configure gpu, process ETNA_PM_PROCESS_POST requests
	 *   and update the sequence number for userspace.
	 */
	if (submit->nr_pmrs)
		nr_events = 3;

	ret = event_alloc(gpu, nr_events, event);
	if (ret) {
		DRM_ERROR("no free events\n");
		pm_runtime_put_noidle(gpu->dev);
		return NULL;
	}

	mutex_lock(&gpu->lock);

	gpu_fence = etnaviv_gpu_fence_alloc(gpu);
	if (!gpu_fence) {
		for (i = 0; i < nr_events; i++)
			event_free(gpu, event[i]);

		goto out_unlock;
	}

	if (gpu->state == ETNA_GPU_STATE_INITIALIZED)
		etnaviv_gpu_start_fe_idleloop(gpu, submit->mmu_context);

	if (submit->prev_mmu_context)
		etnaviv_iommu_context_put(submit->prev_mmu_context);
	submit->prev_mmu_context = etnaviv_iommu_context_get(gpu->mmu_context);

	if (submit->nr_pmrs) {
		gpu->event[event[1]].sync_point = &sync_point_perfmon_sample_pre;
		kref_get(&submit->refcount);
		gpu->event[event[1]].submit = submit;
		etnaviv_sync_point_queue(gpu, event[1]);
	}

	gpu->event[event[0]].fence = gpu_fence;
	submit->cmdbuf.user_size = submit->cmdbuf.size - 8;
	etnaviv_buffer_queue(gpu, submit->exec_state, submit->mmu_context,
			     event[0], &submit->cmdbuf);

	if (submit->nr_pmrs) {
		gpu->event[event[2]].sync_point = &sync_point_perfmon_sample_post;
		kref_get(&submit->refcount);
		gpu->event[event[2]].submit = submit;
		etnaviv_sync_point_queue(gpu, event[2]);
	}

out_unlock:
	mutex_unlock(&gpu->lock);

	return gpu_fence;
}

/*
 * NPU deferred completion work: called from ISR via schedule_work when
 * GL_EVENT fires before NN/TP complete (VIPPico v3 where SMALL_BATCH
 * doesn't stall FE). Waits for NN/TP idle then signals the fence.
 *
 * Unlike the old CMD_END approach, FE stays alive (GL_EVENT path),
 * so no FE restart needed. Just poll idle and signal.
 */
static void etnaviv_gpu_npu_poll_work(struct work_struct *work)
{
	struct etnaviv_gpu *gpu = container_of(work, struct etnaviv_gpu,
					       npu_poll_work.work);
	int event = gpu->npu_poll_event;
	struct dma_fence *fence;
	u32 idle;
	int i;

	if (event < 0 || event >= ETNA_NR_EVENTS)
		return;

	/*
	 * Poll for NN and TP idle. The GL_EVENT already fired (FE
	 * processed the command buffer) but NN/TP are still executing.
	 * Typical NN ops take 1-500ms depending on layer size.
	 */
	for (i = 0; i < 100000; i++) {
		idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);
		if ((idle & VIVS_HI_IDLE_STATE_NN) &&
		    (idle & VIVS_HI_IDLE_STATE_TP))
			break;
		usleep_range(100, 200);
	}

	if (i >= 100000)
		dev_err(gpu->dev,
			"NPU poll: timeout idle=0x%08x\n", idle);

	/* Signal the fence -- NN/TP data is in DDR (AXI write-through) */
	fence = gpu->event[event].fence;
	if (fence) {
		gpu->event[event].fence = NULL;
		if (fence_after(fence->seqno, gpu->completed_fence))
			gpu->completed_fence = fence->seqno;
		dma_fence_signal(fence);
	}

	event_free(gpu, event);
	gpu->npu_poll_event = -1;
}

static void sync_point_worker(struct work_struct *work)
{
	struct etnaviv_gpu *gpu = container_of(work, struct etnaviv_gpu,
					       sync_point_work);
	struct etnaviv_event *event = &gpu->event[gpu->sync_point_event];
	u32 addr = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);

	event->sync_point(gpu, event);
	etnaviv_submit_put(event->submit);
	event_free(gpu, gpu->sync_point_event);

	/* restart FE last to avoid GPU and IRQ racing against this worker */
	etnaviv_gpu_start_fe(gpu, addr + 2, 2);
}

void etnaviv_gpu_recover_hang(struct etnaviv_gem_submit *submit)
{
	struct etnaviv_gpu *gpu = submit->gpu;
	char *comm = NULL, *cmd = NULL;
	struct task_struct *task;
	unsigned int i;

	{
		u32 dma_addr = gpu_read(gpu, VIVS_FE_DMA_ADDRESS);
		u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);
		u32 fe_debug = gpu_read(gpu, VIVS_FE_DMA_DEBUG_STATE);
		u32 intr_enbl = gpu_read(gpu, VIVS_HI_INTR_ENBL);
		u32 intr_ack = gpu_read(gpu, VIVS_HI_INTR_ACKNOWLEDGE);
		u32 pmc = gpu_read_power(gpu, VIVS_PM_MODULE_CONTROLS);
		u32 pms = gpu_read_power(gpu, VIVS_PM_MODULE_STATUS);
		u32 ppc = gpu_read_power(gpu, VIVS_PM_POWER_CONTROLS);

		dev_err(gpu->dev, "recover hung GPU! dma=0x%08x idle=0x%08x fe_dbg=0x%08x intr_en=0x%08x\n",
			dma_addr, idle, fe_debug, intr_enbl);
		dev_err(gpu->dev, "  intr_ack=0x%08x pmc=0x%08x pms=0x%08x ppc=0x%08x\n",
			intr_ack, pmc, pms, ppc);

		/* r91: dump cmdbuf bytes around the hang dma_addr to identify
		 * the stuck instruction. dma_addr is GPU VA; cmdbuf_va is
		 * the active submit's cmdbuf GPU VA. Dump 16 words before +
		 * 16 words after the hang point.
		 *
		 * R44 (2026-05-06): also dump kernel ring buffer when hang is
		 * in ring range -- needed for c64a/0x99 MMU-flush prelude
		 * stall (dma=0x10b0 in ring at va 0x1000). Pre-R44 only
		 * dumped per-op cmdbuf, missing ring-buffer hangs entirely.
		 */
		if (submit && submit->cmdbuf.vaddr && submit->cmdbuf.size) {
			u32 cmdbuf_va = etnaviv_cmdbuf_get_va(&submit->cmdbuf,
				&gpu->mmu_context->cmdbuf_mapping);
			if (dma_addr >= cmdbuf_va &&
			    dma_addr < cmdbuf_va + submit->cmdbuf.size) {
				u32 dma_off = dma_addr - cmdbuf_va;
				u32 *words = submit->cmdbuf.vaddr;
				int center = dma_off / 4;
				int start = center - 8;
				int end = center + 16;
				int w;
				if (start < 0) start = 0;
				if (end > (int)(submit->cmdbuf.size / 4))
					end = submit->cmdbuf.size / 4;
				dev_err(gpu->dev, "cmdbuf hang dump (cmdbuf_va=0x%08x dma_off=0x%x word=%d):\n",
					cmdbuf_va, dma_off, center);
				for (w = start; w < end; w++) {
					dev_err(gpu->dev, "  %s word[%4d]=0x%08x\n",
						w == center ? "**" : "  ",
						w, words[w]);
				}
			}
		}
		if (gpu->buffer.vaddr && gpu->buffer.size) {
			u32 ring_va = etnaviv_cmdbuf_get_va(&gpu->buffer,
				&gpu->mmu_context->cmdbuf_mapping);
			if (dma_addr >= ring_va &&
			    dma_addr < ring_va + gpu->buffer.size) {
				u32 dma_off = dma_addr - ring_va;
				u32 *words = gpu->buffer.vaddr;
				int center = dma_off / 4;
				int start = center - 16;
				int end = center + 16;
				int w;
				if (start < 0) start = 0;
				if (end > (int)(gpu->buffer.size / 4))
					end = gpu->buffer.size / 4;
				dev_err(gpu->dev, "ring hang dump (ring_va=0x%08x dma_off=0x%x word=%d):\n",
					ring_va, dma_off, center);
				for (w = start; w < end; w++) {
					dev_err(gpu->dev, "  %s ring[%4d]=0x%08x\n",
						w == center ? "**" : "  ",
						w, words[w]);
				}
			}
		}
	}

	task = get_pid_task(submit->pid, PIDTYPE_PID);
	if (task) {
		comm = kstrdup(task->comm, GFP_KERNEL);
		cmd = kstrdup_quotable_cmdline(task, GFP_KERNEL);
		put_task_struct(task);
	}

	if (comm && cmd)
		dev_err(gpu->dev, "offending task: %s (%s)\n", comm, cmd);

	kfree(cmd);
	kfree(comm);

	if (pm_runtime_get_sync(gpu->dev) < 0)
		goto pm_put;

	mutex_lock(&gpu->lock);

	etnaviv_hw_reset(gpu);

	/* complete all events, the GPU won't do it after the reset */
	spin_lock(&gpu->event_spinlock);
	for_each_set_bit(i, gpu->event_bitmap, ETNA_NR_EVENTS)
		event_free(gpu, i);
	spin_unlock(&gpu->event_spinlock);

	etnaviv_gpu_hw_init(gpu);

	mutex_unlock(&gpu->lock);
	pm_runtime_mark_last_busy(gpu->dev);
pm_put:
	pm_runtime_put_autosuspend(gpu->dev);
}

static void dump_mmu_fault(struct etnaviv_gpu *gpu)
{
	static const char *fault_reasons[] = {
		"slave not present",
		"page not present",
		"write violation",
		"out of bounds",
		"read security violation",
		"write security violation",
	};

	u32 status_reg, status;
	int i;

	if (gpu->sec_mode == ETNA_SEC_NONE)
		status_reg = VIVS_MMUv2_STATUS;
	else
		status_reg = VIVS_MMUv2_SEC_STATUS;

	status = gpu_read(gpu, status_reg);
	dev_err_ratelimited(gpu->dev, "MMU fault status 0x%08x\n", status);

	for (i = 0; i < 4; i++) {
		const char *reason = "unknown";
		u32 address_reg;
		u32 mmu_status;

		mmu_status = (status >> (i * 4)) & VIVS_MMUv2_STATUS_EXCEPTION0__MASK;
		if (!mmu_status)
			continue;

		if ((mmu_status - 1) < ARRAY_SIZE(fault_reasons))
			reason = fault_reasons[mmu_status - 1];

		if (gpu->sec_mode == ETNA_SEC_NONE)
			address_reg = VIVS_MMUv2_EXCEPTION_ADDR(i);
		else
			address_reg = VIVS_MMUv2_SEC_EXCEPTION_ADDR;

		dev_err_ratelimited(gpu->dev,
				    "MMU %d fault (%s) addr 0x%08x\n",
				    i, reason, gpu_read(gpu, address_reg));
	}
}

static irqreturn_t irq_handler(int irq, void *data)
{
	struct etnaviv_gpu *gpu = data;
	irqreturn_t ret = IRQ_NONE;

	u32 intr = gpu_read(gpu, VIVS_HI_INTR_ACKNOWLEDGE);

	if (gpu->identity.nn_core_count > 0)
		dev_info_ratelimited(gpu->dev, "NPU irq: intr=0x%08x\n", intr);

	if (intr != 0) {
		ktime_t now = ktime_get();
		int event;

		pm_runtime_mark_last_busy(gpu->dev);

		dev_dbg(gpu->dev, "intr 0x%08x\n", intr);

		if (intr & VIVS_HI_INTR_ACKNOWLEDGE_AXI_BUS_ERROR) {
			dev_err(gpu->dev, "AXI bus error\n");
			intr &= ~VIVS_HI_INTR_ACKNOWLEDGE_AXI_BUS_ERROR;
		}

		if (intr & VIVS_HI_INTR_ACKNOWLEDGE_MMU_EXCEPTION) {
			dump_mmu_fault(gpu);
			gpu->state = ETNA_GPU_STATE_FAULT;
			drm_sched_fault(&gpu->sched);
			intr &= ~VIVS_HI_INTR_ACKNOWLEDGE_MMU_EXCEPTION;
		}

		while ((event = ffs(intr)) != 0) {
			struct dma_fence *fence;

			event -= 1;

			intr &= ~(1 << event);

			dev_dbg(gpu->dev, "event %u\n", event);

			if (gpu->event[event].sync_point) {
				gpu->sync_point_event = event;
				queue_work(gpu->wq, &gpu->sync_point_work);
			}

			/*
			 * VIPPico v3 deferred completion: GL_EVENT fires
			 * from FE before NN/TP finish (FE doesn't stall).
			 * Defer fence signaling to npu_poll_work which
			 * waits for NN/TP idle then signals.
			 */
			if (gpu->npu_poll_completion &&
			    event == gpu->npu_poll_event) {
				schedule_work(&gpu->npu_poll_work.work);
				continue;
			}

			fence = gpu->event[event].fence;
			if (!fence)
				continue;

			gpu->event[event].fence = NULL;

			/*
			 * A311D (VIPNano-QI v8, customer_id 0x88) hardware
			 * drain wait. Without this ~1 ms delay before
			 * signaling the submit fence, mesa's
			 * pipe_buffer_read of the output BO returns stale
			 * data from the prior inference (MobileNet v1
			 * reused-interpreter off-by-one bug). The
			 * mechanism is a hardware drain (AXI write
			 * buffer or NN engine output FIFO) that completes
			 * 400-600 us after NN/TP go idle -- the NN/TP idle
			 * bits in VIVS_HI_IDLE_STATE are already set at
			 * fence-signal time, so polling them is useless.
			 *
			 * Matrix-driven sweep on 2026-04-13 showed:
			 *   < 400 us: deterministic 0/7 broken
			 *   400-500 us: stochastic, partial passes
			 *   >= 600 us: deterministic 7/7 fixed
			 * udelay(1000) is a conservative value above the
			 * stochastic boundary. Validated on A311D with
			 * 21/21 inferences correct across 3 runs at
			 * 266 ms/inference (vs 265 ms broken baseline,
			 * ~0.3 percent overhead).
			 *
			 * 1 ms hardirq block is within safe limits at the
			 * few-per-inference rate the NPU produces. An
			 * alternative defer-to-workqueue approach
			 * (schedule_work + npu_poll_work, as customer 0x99
			 * already uses) also fixes correctness but is ~2x
			 * slower due to workqueue wakeup latency. udelay
			 * is simpler and faster.
			 *
			 * Investigation log:
			 * amlogic/software/npu/ANALYSIS.md "A311D
			 * MobileNet v1 Off-By-One Bug -- BREAKTHROUGH"
			 * and subsequent matrix v1/v2/v3 sections.
			 *
			 * S905D3 (customer 0x99) does NOT need this --
			 * that path uses npu_poll_completion deferral
			 * which polls VIVS_HI_IDLE_STATE_NN/_TP before
			 * signaling, providing the same effective wait.
			 */
			if (gpu->identity.customer_id == 0x88)
				udelay(npu_drain_udelay); /* r79: parameterized */

			/*
			 * Events can be processed out of order.  Eg,
			 * - allocate and queue event 0
			 * - allocate event 1
			 * - event 0 completes, we process it
			 * - allocate and queue event 0
			 * - event 1 and event 0 complete
			 * we can end up processing event 0 first, then 1.
			 */
			if (fence_after(fence->seqno, gpu->completed_fence))
				gpu->completed_fence = fence->seqno;
			dma_fence_signal_timestamp(fence, now);

			event_free(gpu, event);
		}

		ret = IRQ_HANDLED;
	}

	return ret;
}

static int etnaviv_gpu_clk_enable(struct etnaviv_gpu *gpu)
{
	int ret;

	dev_info(gpu->dev, "clk_enable: reg\n");
	ret = clk_prepare_enable(gpu->clk_reg);
	if (ret)
		return ret;

	dev_info(gpu->dev, "clk_enable: bus\n");
	ret = clk_prepare_enable(gpu->clk_bus);
	if (ret)
		goto disable_clk_reg;

	dev_info(gpu->dev, "clk_enable: core\n");
	ret = clk_prepare_enable(gpu->clk_core);
	if (ret)
		goto disable_clk_bus;

	dev_info(gpu->dev, "clk_enable: shader\n");
	ret = clk_prepare_enable(gpu->clk_shader);
	if (ret)
		goto disable_clk_core;

	dev_info(gpu->dev, "clk_enable: all done\n");
	return 0;

disable_clk_core:
	clk_disable_unprepare(gpu->clk_core);
disable_clk_bus:
	clk_disable_unprepare(gpu->clk_bus);
disable_clk_reg:
	clk_disable_unprepare(gpu->clk_reg);

	return ret;
}

static int etnaviv_gpu_clk_disable(struct etnaviv_gpu *gpu)
{
	clk_disable_unprepare(gpu->clk_shader);
	clk_disable_unprepare(gpu->clk_core);
	clk_disable_unprepare(gpu->clk_bus);
	clk_disable_unprepare(gpu->clk_reg);

	return 0;
}

int etnaviv_gpu_wait_idle(struct etnaviv_gpu *gpu, unsigned int timeout_ms)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(timeout_ms);

	do {
		u32 idle = gpu_read(gpu, VIVS_HI_IDLE_STATE);

		if ((idle & gpu->idle_mask) == gpu->idle_mask)
			return 0;

		if (time_is_before_jiffies(timeout)) {
			dev_warn(gpu->dev,
				 "timed out waiting for idle: idle=0x%x\n",
				 idle);
			return -ETIMEDOUT;
		}

		udelay(5);
	} while (1);
}

static void etnaviv_gpu_hw_suspend(struct etnaviv_gpu *gpu)
{
	if (gpu->state == ETNA_GPU_STATE_RUNNING) {
		/* Replace the last WAIT with END */
		mutex_lock(&gpu->lock);
		etnaviv_buffer_end(gpu);
		mutex_unlock(&gpu->lock);

		/*
		 * We know that only the FE is busy here, this should
		 * happen quickly (as the WAIT is only 200 cycles).  If
		 * we fail, just warn and continue.
		 */
		etnaviv_gpu_wait_idle(gpu, 100);

		gpu->state = ETNA_GPU_STATE_INITIALIZED;
	}

	gpu->exec_state = -1;
}

static int etnaviv_gpu_hw_resume(struct etnaviv_gpu *gpu)
{
	int ret;

	ret = mutex_lock_killable(&gpu->lock);
	if (ret)
		return ret;

	etnaviv_gpu_update_clock(gpu);
	etnaviv_gpu_hw_init(gpu);

	mutex_unlock(&gpu->lock);

	return 0;
}

static int
etnaviv_gpu_cooling_get_max_state(struct thermal_cooling_device *cdev,
				  unsigned long *state)
{
	*state = 6;

	return 0;
}

static int
etnaviv_gpu_cooling_get_cur_state(struct thermal_cooling_device *cdev,
				  unsigned long *state)
{
	struct etnaviv_gpu *gpu = cdev->devdata;

	*state = gpu->freq_scale;

	return 0;
}

static int
etnaviv_gpu_cooling_set_cur_state(struct thermal_cooling_device *cdev,
				  unsigned long state)
{
	struct etnaviv_gpu *gpu = cdev->devdata;

	mutex_lock(&gpu->lock);
	gpu->freq_scale = state;
	if (!pm_runtime_suspended(gpu->dev))
		etnaviv_gpu_update_clock(gpu);
	mutex_unlock(&gpu->lock);

	return 0;
}

static const struct thermal_cooling_device_ops cooling_ops = {
	.get_max_state = etnaviv_gpu_cooling_get_max_state,
	.get_cur_state = etnaviv_gpu_cooling_get_cur_state,
	.set_cur_state = etnaviv_gpu_cooling_set_cur_state,
};

static int etnaviv_gpu_bind(struct device *dev, struct device *master,
	void *data)
{
	struct drm_device *drm = data;
	struct etnaviv_drm_private *priv = drm->dev_private;
	struct etnaviv_gpu *gpu = dev_get_drvdata(dev);
	int ret;

	if (IS_ENABLED(CONFIG_DRM_ETNAVIV_THERMAL)) {
		gpu->cooling = thermal_of_cooling_device_register(dev->of_node,
				(char *)dev_name(dev), gpu, &cooling_ops);
		if (IS_ERR(gpu->cooling))
			return PTR_ERR(gpu->cooling);
	}

	gpu->wq = alloc_ordered_workqueue(dev_name(dev), 0);
	if (!gpu->wq) {
		ret = -ENOMEM;
		goto out_thermal;
	}

	ret = etnaviv_sched_init(gpu);
	if (ret)
		goto out_workqueue;

	if (!IS_ENABLED(CONFIG_PM)) {
		ret = etnaviv_gpu_clk_enable(gpu);
		if (ret < 0)
			goto out_sched;
	}

	gpu->drm = drm;
	gpu->fence_context = dma_fence_context_alloc(1);
	xa_init_flags(&gpu->user_fences, XA_FLAGS_ALLOC);
	spin_lock_init(&gpu->fence_spinlock);

	INIT_WORK(&gpu->sync_point_work, sync_point_worker);
	init_waitqueue_head(&gpu->fence_event);

	priv->gpu[priv->num_gpus++] = gpu;

	return 0;

out_sched:
	etnaviv_sched_fini(gpu);

out_workqueue:
	destroy_workqueue(gpu->wq);

out_thermal:
	if (IS_ENABLED(CONFIG_DRM_ETNAVIV_THERMAL))
		thermal_cooling_device_unregister(gpu->cooling);

	return ret;
}

static void etnaviv_gpu_unbind(struct device *dev, struct device *master,
	void *data)
{
	struct etnaviv_gpu *gpu = dev_get_drvdata(dev);

	DBG("%s", dev_name(gpu->dev));

	destroy_workqueue(gpu->wq);

	etnaviv_sched_fini(gpu);

	if (IS_ENABLED(CONFIG_PM)) {
		pm_runtime_get_sync(gpu->dev);
		pm_runtime_put_sync_suspend(gpu->dev);
	} else {
		etnaviv_gpu_hw_suspend(gpu);
		etnaviv_gpu_clk_disable(gpu);
	}

	if (gpu->mmu_context)
		etnaviv_iommu_context_put(gpu->mmu_context);

	etnaviv_cmdbuf_free(&gpu->buffer);
	if (gpu->nn_data_vaddr)
		dma_free_wc(gpu->drm->dev, ETNAVIV_FLOP_DATA_SIZE,
			    gpu->nn_data_vaddr, gpu->nn_data_paddr);
	etnaviv_iommu_global_fini(gpu);

	gpu->drm = NULL;
	xa_destroy(&gpu->user_fences);

	if (IS_ENABLED(CONFIG_DRM_ETNAVIV_THERMAL))
		thermal_cooling_device_unregister(gpu->cooling);
	gpu->cooling = NULL;
}

static const struct component_ops gpu_ops = {
	.bind = etnaviv_gpu_bind,
	.unbind = etnaviv_gpu_unbind,
};

static const struct of_device_id etnaviv_gpu_match[] = {
	{
		.compatible = "vivante,gc"
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, etnaviv_gpu_match);

static int etnaviv_gpu_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct etnaviv_gpu *gpu;
	int err;

	gpu = devm_kzalloc(dev, sizeof(*gpu), GFP_KERNEL);
	if (!gpu)
		return -ENOMEM;

	gpu->dev = &pdev->dev;
	mutex_init(&gpu->lock);
	mutex_init(&gpu->sched_lock);

	/* Map registers: */
	gpu->mmio = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gpu->mmio))
		return PTR_ERR(gpu->mmio);

	/* Get Interrupt: */
	gpu->irq = platform_get_irq(pdev, 0);
	if (gpu->irq < 0)
		return gpu->irq;

	err = devm_request_irq(&pdev->dev, gpu->irq, irq_handler, 0,
			       dev_name(gpu->dev), gpu);
	if (err) {
		dev_err(dev, "failed to request IRQ%u: %d\n", gpu->irq, err);
		return err;
	}

	/* Get Clocks: */
	gpu->clk_reg = devm_clk_get_optional(&pdev->dev, "reg");
	DBG("clk_reg: %p", gpu->clk_reg);
	if (IS_ERR(gpu->clk_reg))
		return PTR_ERR(gpu->clk_reg);

	gpu->clk_bus = devm_clk_get_optional(&pdev->dev, "bus");
	DBG("clk_bus: %p", gpu->clk_bus);
	if (IS_ERR(gpu->clk_bus))
		return PTR_ERR(gpu->clk_bus);

	gpu->clk_core = devm_clk_get(&pdev->dev, "core");
	DBG("clk_core: %p", gpu->clk_core);
	if (IS_ERR(gpu->clk_core))
		return PTR_ERR(gpu->clk_core);
	gpu->base_rate_core = clk_get_rate(gpu->clk_core);

	gpu->clk_shader = devm_clk_get_optional(&pdev->dev, "shader");
	DBG("clk_shader: %p", gpu->clk_shader);
	if (IS_ERR(gpu->clk_shader))
		return PTR_ERR(gpu->clk_shader);
	gpu->base_rate_shader = clk_get_rate(gpu->clk_shader);

	/* TODO: figure out max mapped size */
	dev_set_drvdata(dev, gpu);

	/*
	 * We treat the device as initially suspended.  The runtime PM
	 * autosuspend delay is rather arbitary: no measurements have
	 * yet been performed to determine an appropriate value.
	 */
	pm_runtime_use_autosuspend(gpu->dev);
	pm_runtime_set_autosuspend_delay(gpu->dev, 200);
	pm_runtime_enable(gpu->dev);

	err = component_add(&pdev->dev, &gpu_ops);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register component: %d\n", err);
		return err;
	}

	return 0;
}

static void etnaviv_gpu_platform_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &gpu_ops);
	pm_runtime_disable(&pdev->dev);
}

static int etnaviv_gpu_rpm_suspend(struct device *dev)
{
	struct etnaviv_gpu *gpu = dev_get_drvdata(dev);
	u32 idle, mask;

	/* If there are any jobs in the HW queue, we're not idle */
	if (atomic_read(&gpu->sched.credit_count))
		return -EBUSY;

	/* Check whether the hardware (except FE and MC) is idle */
	mask = gpu->idle_mask & ~(VIVS_HI_IDLE_STATE_FE |
				  VIVS_HI_IDLE_STATE_MC);
	idle = gpu_read(gpu, VIVS_HI_IDLE_STATE) & mask;
	if (idle != mask) {
		dev_warn_ratelimited(dev, "GPU not yet idle, mask: 0x%08x\n",
				     idle);
		return -EBUSY;
	}

	etnaviv_gpu_hw_suspend(gpu);

	gpu->state = ETNA_GPU_STATE_IDENTIFIED;

	return etnaviv_gpu_clk_disable(gpu);
}

static int etnaviv_gpu_rpm_resume(struct device *dev)
{
	struct etnaviv_gpu *gpu = dev_get_drvdata(dev);
	int ret;

	ret = etnaviv_gpu_clk_enable(gpu);
	if (ret)
		return ret;

	/* Re-initialise the basic hardware state */
	if (gpu->state == ETNA_GPU_STATE_IDENTIFIED) {
		ret = etnaviv_gpu_hw_resume(gpu);
		if (ret) {
			etnaviv_gpu_clk_disable(gpu);
			return ret;
		}
	}

	return 0;
}

static const struct dev_pm_ops etnaviv_gpu_pm_ops = {
	RUNTIME_PM_OPS(etnaviv_gpu_rpm_suspend, etnaviv_gpu_rpm_resume, NULL)
};

struct platform_driver etnaviv_gpu_driver = {
	.driver = {
		.name = "etnaviv-gpu",
		.pm = pm_ptr(&etnaviv_gpu_pm_ops),
		.of_match_table = etnaviv_gpu_match,
	},
	.probe = etnaviv_gpu_platform_probe,
	.remove_new = etnaviv_gpu_platform_remove,
	.id_table = gpu_ids,
};
