// SPDX-License-Identifier: GPL-2.0
/*
 * vdec_diag.c -- AMRISC diagnostic hot-reload interface (Tier 1 test bench)
 *
 * Exposes a debugfs directory /sys/kernel/debug/meson-vdec/diag/vdec1/
 * with the following files:
 *
 *   firmware  (write-only, binary blob up to 16 KiB)
 *       Hot-upload a raw AMRISC firmware image into the VDEC_1 engine's
 *       IMEM, bypassing request_firmware() and the module reload path.
 *       Sequence: MPSR=0, CPSR=0, IMEM_DMA to mc_addr_map, MPSR=1.
 *       No byte-patches are applied (the uploaded blob is consumed as-is).
 *
 *   reset     (write-only, "1")
 *       Pulse DOS_SW_RESET0_MDEC | DOS_SW_RESET0_VLD, re-arm MPSR.
 *       Does NOT reload firmware; leaves whatever is in IMEM running.
 *
 *   state     (read-only, text key=value lines)
 *       Single snapshot of the interesting scratch and hardware registers:
 *       AV_SCRATCH_0..F, VLD_STATUS_CTRL, VLD_MEM_VIFIFO_LEVEL,
 *       VLD_MEM_VIFIFO_BYTES_AVAIL, MDEC_PIC_DC_STATUS, MPSR, MINT_VEC_BASE,
 *       MCPU_INTR_MSK.
 *
 *   peek      (read-write, "<offset_hex>")
 *       Write a DOS MMIO byte offset; read back the 32-bit value at that
 *       offset.  The offset is range-checked against 0x0000-0x3fff.
 *
 *   poke      (write-only, "<offset_hex> <value_hex>")
 *       Write a 32-bit value to a DOS MMIO byte offset.  Range-checked.
 *
 *   mode      (write-only, "<decimal>")
 *       Shortcut for writing AV_SCRATCH_8.  The diagnostic firmware reads
 *       this at entry to select which test to run.
 *
 *   cmd       (write-only, "<decimal>")
 *       Shortcut for writing AV_SCRATCH_9.  The diagnostic firmware reads
 *       this as the command register in the command-poll test.
 *
 *   lmem_dma_test     (write-only, "<ctrl_hex> <count_dec> [<adr_hex>]")
 *       Fire one LMEM_DMA pulse with the parsed CTRL/COUNT/ADR.  Polls
 *       LMEM_DMA_CTRL bit 15 for clear (10000-iter cap) and snapshots
 *       elapsed time + WRRSP_LMEM into the engine's last_result struct.
 *       Hangs are recorded as data, not rejected -- the harness is for
 *       characterizing CTRL bit fields against engine response.
 *
 *   lmem_dma_result   (read-only, key=value lines)
 *       Outputs the most recent lmem_dma_test outcome (result, ctrl,
 *       count, adr, iters, wrrsp, ms).  Returns "result=noinit" if no
 *       lmem_dma_test write has happened since boot.
 *
 *   workspace_paddr   (read-only, "0x????????")
 *       Physical address of the pre-allocated 4 KiB LMEM workspace.
 *       Use as the implicit ADR target for lmem_dma_test, or for
 *       arranging fixture memory the firmware can DMA from.
 *
 *   workspace_pattern (read-write, "<seed_hex>")
 *       Write: fill workspace[i] = seed + i*4 for i in [0, 1024).
 *       Read: returns the most recent seed (or 0 on init).
 *
 * Safety:
 *   - Module param gate (`diag=1` on meson_vdec_core) -- directory is
 *     not created when disabled.  Default: disabled.
 *   - Operations require a valid VDEC_1 session to own the engine so
 *     power/clock/reset state is established.  Firmware hot-upload
 *     while no session exists returns -ENODEV.
 *   - Firmware blob is size-checked against MC_SIZE (16 KiB).
 *   - peek/poke offsets are range-checked against 0x0000-0x3fff.
 *   - Every diag operation is logged with dev_info (audit trail in the
 *     kernel ring buffer).
 *
 * Design and rationale:
 *   amlogic/software/codec/decoder/stateless/test-bench-plan.md
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include "vdec.h"
#include "vdec_diag.h"
#include "vdec_helpers.h"
#include "dos_regs.h"

#define VDEC_DIAG_MC_SIZE		(4096 * 4)	/* 16 KiB, VDEC_1 IMEM budget */
#define VDEC_DIAG_DOS_OFFSET_MAX	0x3fff
#define VDEC_DIAG_PEEK_BUF_LEN		32

/*
 * LMEM_DMA characterization workspace -- 4 KiB pre-allocated DMA region
 * reused across lmem_dma_test fires.  4 KiB / 4 = 1024 32-bit words is
 * the maximum LMEM_DMA_COUNT the workspace can support; lmem_dma_test
 * range-checks count against this bound.
 */
#define VDEC_DIAG_WORKSPACE_SIZE	4096
#define VDEC_DIAG_WORKSPACE_WORDS	(VDEC_DIAG_WORKSPACE_SIZE / 4)

/* lmem_dma_test parser limit: "<ctrl_hex> <count_dec> <adr_hex>" + NUL. */
#define VDEC_DIAG_LMEM_TEST_BUF_LEN	64

/* Bound for the DMA-clear poll loop (usleep(1) per iter -> ~10 ms cap). */
#define VDEC_DIAG_LMEM_POLL_MAX		10000

/* LMEM_DMA_CTRL bit 15 -- engine busy flag (auto-clears on completion). */
#define LMEM_DMA_CTRL_START		BIT(15)

bool vdec_diag_enabled;
EXPORT_SYMBOL_GPL(vdec_diag_enabled);

/* Per-engine diagnostic state.  Tier 1 supports VDEC_1 only; HEVC and
 * HCODEC are stubs added in a later session. */
struct vdec_diag_engine {
	struct amvdec_core *core;
	const char *name;
	struct dentry *dir;
	struct mutex lock;
	/* DMA buffer for IMEM hot-uploads; allocated at init, reused. */
	void *fw_buf;
	dma_addr_t fw_buf_dma;
	size_t fw_buf_size;
	/* Remembered state for peek (user writes offset, then reads value) */
	u32 peek_offset;
	bool peek_armed;
	/* Set by diag-manual poweron (echo 1 > poweron). When true, peek /
	 * poke / firmware / mode / cmd operations are permitted WITHOUT
	 * a V4L2 session (the original gate). Clear by echo 0 > poweron. */
	bool manual_powered;

	/*
	 * LMEM_DMA characterization workspace.  4 KiB DMA-coherent buffer
	 * allocated at engine init, freed at exit.  Reused across every
	 * lmem_dma_test fire.  workspace_seed records the most recent value
	 * passed to workspace_pattern (read-back exposes it via debugfs).
	 */
	void *workspace_buf;
	dma_addr_t workspace_dma;
	u32 workspace_seed;

	/*
	 * Last lmem_dma_test result.  Populated on every lmem_dma_test
	 * write; read by lmem_dma_result.  result_valid is false until
	 * the first fire (read returns "result=noinit").
	 */
	bool last_result_valid;
	bool last_result_hang;	/* true = bit 15 never cleared within poll budget */
	u32 last_ctrl;
	u32 last_count;
	u32 last_adr;
	u32 last_iters;		/* iterations consumed before bit 15 cleared */
	u32 last_wrrsp;		/* WRRSP_LMEM after the pulse */
	u32 last_ms;		/* elapsed time in ms */
};

struct vdec_diag {
	struct dentry *root;		/* /sys/kernel/debug/meson-vdec */
	struct dentry *diag_dir;	/* /sys/kernel/debug/meson-vdec/diag */
	struct vdec_diag_engine vdec1;
};

/*
 * Engine must be "powered" (power rails up, AMRISC reachable) for any
 * diag op except `poweron` itself. Two sources:
 *  - an active V4L2 session owns the engine (core->cur_sess non-NULL)
 *  - the user ran `echo 1 > diag/vdec1/poweron` manually
 * Either gate is sufficient. `poweron` maintains its own additional
 * check that a V4L2 session isn't already holding the engine, so the
 * two paths don't collide.
 */
static bool engine_powered(struct vdec_diag_engine *eng)
{
	return eng->core->cur_sess || eng->manual_powered;
}

static struct vdec_diag_engine *engine_from_file(struct file *f)
{
	return f->f_inode->i_private;
}

/* ---- firmware: write binary blob, load to IMEM, re-arm MPSR ------------- */

static ssize_t vdec_diag_firmware_write(struct file *f, const char __user *ubuf,
					size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	struct device *dev = core->dev_dec;
	u32 waited = 1000;
	int ret;

	if (!engine_powered(eng)) {
		dev_warn(dev, "diag: firmware upload rejected -- no active session (power/clock not up)\n");
		return -ENODEV;
	}
	if (count == 0 || count > VDEC_DIAG_MC_SIZE)
		return -E2BIG;

	mutex_lock(&eng->lock);

	memset(eng->fw_buf, 0, eng->fw_buf_size);
	if (copy_from_user(eng->fw_buf, ubuf, count)) {
		ret = -EFAULT;
		goto out;
	}

	dev_info(dev, "diag: firmware hot-upload %zu bytes by pid=%d\n",
		 count, current->pid);

	/* Halt AMRISC, then full VDEC_1 reset (0xfffffffc matches the
	 * vdec_1_start reset mask -- resets the AMRISC CPU itself via
	 * bits 11-12, which is what makes PC restart at 0 after the new
	 * firmware is loaded.  Without the full reset, PC resumes at
	 * wherever the prior firmware was running when MPSR=0 latched
	 * and the new IMEM contents are never executed from PC 0. */
	amvdec_write_dos(core, MPSR, 0);
	amvdec_write_dos(core, CPSR, 0);
	amvdec_write_dos(core, DOS_SW_RESET0, 0xfffffffc);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);

	amvdec_write_dos(core, IMEM_DMA_ADR, eng->fw_buf_dma);
	amvdec_write_dos(core, IMEM_DMA_COUNT, VDEC_DIAG_MC_SIZE / 4);
	amvdec_write_dos(core, IMEM_DMA_CTRL, IMEM_DMA_CTRL_RUN);

	while (--waited &&
	       (amvdec_read_dos(core, IMEM_DMA_CTRL) & IMEM_DMA_CTRL_START))
		;

	if (waited == 0) {
		dev_err(dev, "diag: firmware hot-upload DMA hang\n");
		ret = -EIO;
		goto out;
	}

	amvdec_write_dos(core, MPSR, 1);

	dev_info(dev, "diag: firmware hot-upload OK -- MPSR=1\n");
	ret = count;

out:
	mutex_unlock(&eng->lock);
	return ret;
}

static const struct file_operations vdec_diag_firmware_fops = {
	.open = simple_open,
	.write = vdec_diag_firmware_write,
	.llseek = noop_llseek,
};

/* ---- reset: pulse MDEC|VLD reset, re-arm MPSR --------------------------- */

static ssize_t vdec_diag_reset_write(struct file *f, const char __user *ubuf,
				     size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	struct device *dev = core->dev_dec;
	u32 mode_saved;

	if (!engine_powered(eng)) {
		dev_warn(dev, "diag: reset rejected -- no active session\n");
		return -ENODEV;
	}

	mutex_lock(&eng->lock);

	dev_info(dev, "diag: engine reset by pid=%d\n", current->pid);

	/* Full reset matching vdec_1_start: 0xfffffffc resets the entire
	 * VDEC_1 block including the AMRISC CPU (bits 11-12), which is
	 * required to restart PC at 0.  IMEM contents are NOT cleared
	 * by this reset -- only the CPU state -- so the previously-
	 * uploaded firmware still executes, just from the beginning.
	 *
	 * Block reset DOES clear AV_SCRATCH_8.  The diag firmware reads
	 * mode from AV_SCRATCH_8 at boot (PC 0x0008), so any value
	 * written via the `mode` debugfs file before reset would be
	 * lost across the block reset and the firmware would always
	 * dispatch to mode 0 (alive test).  Snapshot AV_SCRATCH_8
	 * before the block reset and restore it before MPSR=1 so the
	 * mode value survives.  Discovered 2026-04-25 during M4.1.7
	 * probe-1 work; without this, all standalone probe firmwares
	 * have to hardcode their entry point.
	 */
	mode_saved = amvdec_read_dos(core, AV_SCRATCH_8);

	amvdec_write_dos(core, MPSR, 0);
	amvdec_write_dos(core, CPSR, 0);
	amvdec_write_dos(core, DOS_SW_RESET0, 0xfffffffc);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);

	amvdec_write_dos(core, AV_SCRATCH_8, mode_saved);

	amvdec_write_dos(core, MPSR, 1);

	mutex_unlock(&eng->lock);
	return count;
}

static const struct file_operations vdec_diag_reset_fops = {
	.open = simple_open,
	.write = vdec_diag_reset_write,
	.llseek = noop_llseek,
};

/* ---- state: dump all interesting registers as key=value ----------------- */

static int vdec_diag_state_show(struct seq_file *s, void *unused)
{
	struct vdec_diag_engine *eng = s->private;
	struct amvdec_core *core = eng->core;

	if (!engine_powered(eng)) {
		seq_puts(s, "error=no_active_session\n");
		return 0;
	}

	seq_printf(s, "scratch_0=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_0));
	seq_printf(s, "scratch_1=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_1));
	seq_printf(s, "scratch_2=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_2));
	seq_printf(s, "scratch_3=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_3));
	seq_printf(s, "scratch_4=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_4));
	seq_printf(s, "scratch_5=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_5));
	seq_printf(s, "scratch_6=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_6));
	seq_printf(s, "scratch_7=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_7));
	seq_printf(s, "scratch_8=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_8));
	seq_printf(s, "scratch_9=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_9));
	seq_printf(s, "scratch_a=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_A));
	seq_printf(s, "scratch_b=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_B));
	seq_printf(s, "scratch_c=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_C));
	seq_printf(s, "scratch_d=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_D));
	seq_printf(s, "scratch_e=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_E));
	seq_printf(s, "scratch_f=0x%08x\n", amvdec_read_dos(core, AV_SCRATCH_F));
	seq_printf(s, "vld_status_ctrl=0x%08x\n",
		   amvdec_read_dos(core, VLD_STATUS_CTRL));
	seq_printf(s, "vld_vififo_level=0x%08x\n",
		   amvdec_read_dos(core, VLD_MEM_VIFIFO_LEVEL));
	seq_printf(s, "vld_vififo_wp=0x%08x\n",
		   amvdec_read_dos(core, VLD_MEM_VIFIFO_WP));
	seq_printf(s, "mdec_pic_dc_status=0x%08x\n",
		   amvdec_read_dos(core, MDEC_PIC_DC_STATUS));
	seq_printf(s, "mpsr=0x%08x\n", amvdec_read_dos(core, MPSR));
	/* MINT_VEC_BASE is an AMRISC CR (internal), not a DOS MMIO --
	 * cannot read from host side. Firmware can mirror it to a
	 * scratch reg on boot if needed for diag. */
	seq_printf(s, "mcpu_intr_msk=0x%08x\n",
		   amvdec_read_dos(core, MCPU_INTR_MSK));
	return 0;
}

static int vdec_diag_state_open(struct inode *i, struct file *f)
{
	return single_open(f, vdec_diag_state_show, i->i_private);
}

static const struct file_operations vdec_diag_state_fops = {
	.open = vdec_diag_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* ---- peek: write offset, read value ------------------------------------- */

static ssize_t vdec_diag_peek_write(struct file *f, const char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	char buf[VDEC_DIAG_PEEK_BUF_LEN];
	u32 offset;
	int ret;

	if (count >= sizeof(buf))
		return -E2BIG;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	ret = kstrtou32(strim(buf), 0, &offset);
	if (ret)
		return ret;
	if (offset > VDEC_DIAG_DOS_OFFSET_MAX || (offset & 3))
		return -EINVAL;

	mutex_lock(&eng->lock);
	eng->peek_offset = offset;
	eng->peek_armed = true;
	mutex_unlock(&eng->lock);

	return count;
}

static ssize_t vdec_diag_peek_read(struct file *f, char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	char buf[VDEC_DIAG_PEEK_BUF_LEN];
	u32 val;
	int len;

	if (*ppos > 0)
		return 0;

	mutex_lock(&eng->lock);
	if (!eng->peek_armed) {
		mutex_unlock(&eng->lock);
		return -EINVAL;
	}
	if (!engine_powered(eng)) {
		mutex_unlock(&eng->lock);
		return -ENODEV;
	}
	val = amvdec_read_dos(core, eng->peek_offset);
	mutex_unlock(&eng->lock);

	len = scnprintf(buf, sizeof(buf), "0x%08x\n", val);
	if (copy_to_user(ubuf, buf, len))
		return -EFAULT;
	*ppos = len;
	return len;
}

static const struct file_operations vdec_diag_peek_fops = {
	.open = simple_open,
	.read = vdec_diag_peek_read,
	.write = vdec_diag_peek_write,
	.llseek = noop_llseek,
};

/* ---- poke: write "offset value" ----------------------------------------- */

static ssize_t vdec_diag_poke_write(struct file *f, const char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	char buf[VDEC_DIAG_PEEK_BUF_LEN];
	u32 offset, value;
	char *sep;
	int ret;

	if (count >= sizeof(buf))
		return -E2BIG;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	sep = strchr(buf, ' ');
	if (!sep)
		return -EINVAL;
	*sep = '\0';

	ret = kstrtou32(strim(buf), 0, &offset);
	if (ret)
		return ret;
	ret = kstrtou32(strim(sep + 1), 0, &value);
	if (ret)
		return ret;
	if (offset > VDEC_DIAG_DOS_OFFSET_MAX || (offset & 3))
		return -EINVAL;

	if (!engine_powered(eng)) {
		dev_warn(core->dev_dec, "diag: poke rejected -- no active session\n");
		return -ENODEV;
	}

	mutex_lock(&eng->lock);
	dev_info(core->dev_dec, "diag: poke 0x%04x = 0x%08x by pid=%d\n",
		 offset, value, current->pid);
	amvdec_write_dos(core, offset, value);
	mutex_unlock(&eng->lock);

	return count;
}

static const struct file_operations vdec_diag_poke_fops = {
	.open = simple_open,
	.write = vdec_diag_poke_write,
	.llseek = noop_llseek,
};

/* ---- poweron: bring the VDEC_1 engine up without a V4L2 session -------- */
/*
 * Minimal VDEC_1 power-on for characterization sweeps.  Mirrors the
 * power/clock/reset path of vdec_1_start but does NOT load firmware or
 * initialize any codec state -- just brings the engine to a state
 * where DOS MMIO reads return meaningful values.
 *
 * Usage:
 *   echo 1 > /sys/kernel/debug/meson-vdec/diag/vdec1/poweron   # bring up
 *   echo 0 > /sys/kernel/debug/meson-vdec/diag/vdec1/poweron   # tear down
 *
 * The minimal sequence is:
 *   1. enable vdec_1_clk at max rate
 *   2. clear AO_RTI_GEN_PWR_SLEEP0 VDEC_1 power bits
 *   3. DOS_SW_RESET0 = 0xfffffffc pulse
 *   4. clear AO_RTI_GEN_PWR_ISO0 VDEC_1 isolation
 *   5. DOS_GCLK_EN0 = DOS_GCLK_EN0_VDEC1
 *
 * The SM1 path (used on the 42b6a1870bb4 bringup board) uses the
 * AO_RTI_GEN_PWR_*_SM1 register bit positions.  Other platforms need
 * their respective bit masks.  For the first sweep pass we only
 * support SM1; other platforms can be added on demand.
 */
#include <linux/clk.h>
#include <linux/regmap.h>

/* AO_RTI_GEN_PWR_* byte offsets and VDEC_1 + HEVC power bit masks for
 * SM1.  Values copied from vdec_1.c:18-35 to keep this file
 * self-contained.  G12B/GXL use different bit positions; add them
 * when those platforms need diag poweron.
 *
 * Even though diag only touches VDEC_1, on SM1 the MDEC DC cache
 * shares hardware with the HEVC power domain (see vdec_1.c:480-484);
 * if HEVC is left power-gated while VDEC_1 runs, stale HEVC state
 * corrupts VDEC_1 output.  vdec_1_start clears BOTH power bits; we
 * mirror that. */
#define AO_RTI_GEN_PWR_SLEEP0	0xe8
#define AO_RTI_GEN_PWR_ISO0	0xec
#define GEN_PWR_VDEC_1_SM1	BIT(1)
#define GEN_PWR_VDEC_HEVC_SM1	BIT(2)
#define GEN_PWR_ISO0_VDEC_1_SM1	BIT(1)
#define GEN_PWR_ISO0_HEVC_SM1	BIT(2)
#define DOS_GCLK_EN0_VDEC1	0x3ff
#define MDEC_SW_RESET_MDEC_CORE	BIT(7)

/*
 * Diag VDEC_1 poweron -- mirrors vdec_1.c::vdec_1_start's power/clock/
 * reset sequence up to line 569, minus the firmware-load and codec
 * init.  An earlier version was a "minimal" subset of this sequence
 * and failed to produce a running AMRISC -- the missing steps
 * (HEVC power rail on SM1, 100us isolation-settle delay, secondary
 * MDEC|VLD reset that actually clears IMEM+pipeline, PSCALE disable)
 * were all load-bearing.  Match the full sequence.
 */
static int vdec_diag_poweron(struct amvdec_core *core)
{
	int ret;

	ret = clk_prepare_enable(core->vdec_1_clk);
	if (ret)
		return ret;
	clk_set_rate(core->vdec_1_clk, 667000000);

	/* Clear sleep bits for VDEC_1 + HEVC (SM1 needs both; MDEC DC
	 * cache shares hardware with HEVC power domain). */
	regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
			   GEN_PWR_VDEC_1_SM1 | GEN_PWR_VDEC_HEVC_SM1, 0);
	usleep_range(10, 20);

	/* First reset */
	amvdec_write_dos(core, DOS_SW_RESET0, 0xfffffffc);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);

	amvdec_write_dos(core, DOS_GCLK_EN0, DOS_GCLK_EN0_VDEC1);
	amvdec_write_dos(core, DOS_MEM_PD_VDEC, 0);

	/* Remove VDEC_1 + HEVC isolation (both on SM1) */
	regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
			   GEN_PWR_ISO0_VDEC_1_SM1 | GEN_PWR_ISO0_HEVC_SM1, 0);
	usleep_range(100, 200);

	/* DOS top-level register clears (matches vdec_1_start:526-529) */
	amvdec_write_dos(core, DOS_VDEC_MCRCC_STALL_CTRL, 0);
	amvdec_clear_dos_bits(core, MDEC_PIC_DC_CTRL, MDEC_PIC_DC_CTRL_OLD_MODE);

	/* Second reset (MDEC + VLD) after isolation removal.  This is
	 * what actually clears IMEM contents and the MDEC pipeline --
	 * the first reset before isolation removal doesn't propagate
	 * fully. */
	amvdec_write_dos(core, DOS_SW_RESET0,
			 DOS_SW_RESET0_MDEC | DOS_SW_RESET0_VLD);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_read_dos(core, DOS_SW_RESET0);
	amvdec_write_dos(core, MDEC_SW_RESET, MDEC_SW_RESET_MDEC_CORE);
	amvdec_write_dos(core, MDEC_SW_RESET, 0);

	/* Disable PSCALE (matches vdec_1_start:569) */
	amvdec_write_dos(core, PSCALE_CTRL, 0);

	return 0;
}

static void vdec_diag_poweroff(struct amvdec_core *core)
{
	amvdec_write_dos(core, DOS_SW_RESET0, DOS_SW_RESET0_VDEC1);
	amvdec_write_dos(core, DOS_SW_RESET0, 0);

	regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0,
			   GEN_PWR_ISO0_VDEC_1_SM1, GEN_PWR_ISO0_VDEC_1_SM1);
	amvdec_write_dos(core, DOS_MEM_PD_VDEC, 0xffffffff);
	regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
			   GEN_PWR_VDEC_1_SM1, GEN_PWR_VDEC_1_SM1);

	clk_disable_unprepare(core->vdec_1_clk);
}

static ssize_t vdec_diag_poweron_write(struct file *f, const char __user *ubuf,
				       size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	char buf[VDEC_DIAG_PEEK_BUF_LEN];
	u32 value;
	int ret;

	if (count >= sizeof(buf))
		return -E2BIG;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	ret = kstrtou32(strim(buf), 0, &value);
	if (ret)
		return ret;

	if (core->cur_sess) {
		dev_warn(core->dev_dec,
			 "diag: poweron/poweroff refused -- V4L2 session active\n");
		return -EBUSY;
	}

	mutex_lock(&eng->lock);
	if (value) {
		dev_info(core->dev_dec, "diag: manual poweron by pid=%d\n",
			 current->pid);
		ret = vdec_diag_poweron(core);
		if (ret) {
			mutex_unlock(&eng->lock);
			return ret;
		}
		eng->manual_powered = true;
	} else {
		eng->manual_powered = false;
		dev_info(core->dev_dec, "diag: manual poweroff by pid=%d\n",
			 current->pid);
		vdec_diag_poweroff(core);
	}
	mutex_unlock(&eng->lock);
	return count;
}

static const struct file_operations vdec_diag_poweron_fops = {
	.open = simple_open,
	.write = vdec_diag_poweron_write,
	.llseek = noop_llseek,
};

/* ---- mode/cmd: shortcuts for AV_SCRATCH_8 / AV_SCRATCH_9 ---------------- */

static ssize_t vdec_diag_scratch_write(struct file *f, const char __user *ubuf,
				       size_t count, loff_t *ppos, u32 reg,
				       const char *name)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	char buf[VDEC_DIAG_PEEK_BUF_LEN];
	u32 value;
	int ret;

	if (count >= sizeof(buf))
		return -E2BIG;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	ret = kstrtou32(strim(buf), 0, &value);
	if (ret)
		return ret;

	if (!engine_powered(eng)) {
		dev_warn(core->dev_dec, "diag: %s write rejected -- no active session\n",
			 name);
		return -ENODEV;
	}

	mutex_lock(&eng->lock);
	dev_info(core->dev_dec, "diag: %s = 0x%08x by pid=%d\n",
		 name, value, current->pid);
	amvdec_write_dos(core, reg, value);
	mutex_unlock(&eng->lock);

	return count;
}

static ssize_t vdec_diag_mode_write(struct file *f, const char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	return vdec_diag_scratch_write(f, ubuf, count, ppos, AV_SCRATCH_8, "mode");
}

static ssize_t vdec_diag_cmd_write(struct file *f, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	return vdec_diag_scratch_write(f, ubuf, count, ppos, AV_SCRATCH_9, "cmd");
}

static const struct file_operations vdec_diag_mode_fops = {
	.open = simple_open,
	.write = vdec_diag_mode_write,
	.llseek = noop_llseek,
};

static const struct file_operations vdec_diag_cmd_fops = {
	.open = simple_open,
	.write = vdec_diag_cmd_write,
	.llseek = noop_llseek,
};

/* ---- workspace_paddr: read-only physical address of the LMEM workspace -- */
/*
 * The LMEM_DMA characterization workspace is a 4 KiB DMA-coherent buffer
 * allocated at engine init.  Userspace reads its physical address here
 * so it can pass that exact address to lmem_dma_test (or omit the adr
 * arg, which uses the same workspace).  The address is stable for the
 * lifetime of the diag engine.
 */
static ssize_t vdec_diag_workspace_paddr_read(struct file *f, char __user *ubuf,
					      size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	char buf[VDEC_DIAG_PEEK_BUF_LEN];
	int len;

	if (*ppos > 0)
		return 0;

	len = scnprintf(buf, sizeof(buf), "0x%08x\n",
			(u32)eng->workspace_dma);
	if (len > count)
		len = count;
	if (copy_to_user(ubuf, buf, len))
		return -EFAULT;
	*ppos = len;
	return len;
}

static const struct file_operations vdec_diag_workspace_paddr_fops = {
	.open = simple_open,
	.read = vdec_diag_workspace_paddr_read,
	.llseek = noop_llseek,
};

/* ---- workspace_pattern: write seed -> fill 1024 words; read returns seed - */
/*
 * Userspace writes a 32-bit hex seed; the workspace is filled with
 * (seed + i*4) for i in [0, 1024).  This is the pre-DMA fixture pattern
 * the LMEM characterization sweep loads before firing lmem_dma_test, so
 * post-DMA inspection of LMEM contents (via lda probes from a follow-up
 * firmware run) can verify the DMA actually transferred data.
 *
 * Reading returns the most recent seed (or "0" if no write has occurred).
 */
static ssize_t vdec_diag_workspace_pattern_write(struct file *f,
						 const char __user *ubuf,
						 size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	char buf[VDEC_DIAG_PEEK_BUF_LEN];
	u32 seed, *words;
	int ret, i;

	if (count >= sizeof(buf))
		return -E2BIG;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';

	ret = kstrtou32(strim(buf), 0, &seed);
	if (ret)
		return ret;

	mutex_lock(&eng->lock);
	words = eng->workspace_buf;
	for (i = 0; i < VDEC_DIAG_WORKSPACE_WORDS; i++)
		words[i] = seed + (u32)(i * 4);
	eng->workspace_seed = seed;
	dev_info(core->dev_dec,
		 "diag: workspace pattern seed=0x%08x (%u words) by pid=%d\n",
		 seed, VDEC_DIAG_WORKSPACE_WORDS, current->pid);
	mutex_unlock(&eng->lock);

	return count;
}

static ssize_t vdec_diag_workspace_pattern_read(struct file *f, char __user *ubuf,
						size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	char buf[VDEC_DIAG_PEEK_BUF_LEN];
	u32 seed;
	int len;

	if (*ppos > 0)
		return 0;

	mutex_lock(&eng->lock);
	seed = eng->workspace_seed;
	mutex_unlock(&eng->lock);

	len = scnprintf(buf, sizeof(buf), "0x%08x\n", seed);
	if (len > count)
		len = count;
	if (copy_to_user(ubuf, buf, len))
		return -EFAULT;
	*ppos = len;
	return len;
}

static const struct file_operations vdec_diag_workspace_pattern_fops = {
	.open = simple_open,
	.read = vdec_diag_workspace_pattern_read,
	.write = vdec_diag_workspace_pattern_write,
	.llseek = noop_llseek,
};

/* ---- lmem_dma_test: fire one LMEM_DMA pulse, capture timing + WRRSP ----- */
/*
 * Format: "<ctrl_hex> <count_dec> [<adr_hex>]"
 *   ctrl  -- LMEM_DMA_CTRL value to write (full 32-bit, hardware uses low 16)
 *   count -- LMEM_DMA_COUNT value (number of 32-bit words; <=1024 with default
 *            workspace, but accepted up to 4096 for explicit-adr fires)
 *   adr   -- optional physical address for LMEM_DMA_ADR; omitting uses the
 *            pre-allocated workspace_dma
 *
 * Behavior:
 *   1. Write ADR, COUNT, CTRL (in that order; CTRL write triggers the engine).
 *   2. Poll CTRL & 0x8000 for clear, with a 10000-iter / ~10 ms cap.
 *   3. After bit 15 clears (or budget expires), read WRRSP_LMEM.
 *   4. Stash {ctrl, count, adr, hang_flag, iters, wrrsp, ms} for lmem_dma_result.
 *
 * Returns count from write even on engine hang -- the hang itself is data
 * the characterization sweep needs to record.  Returns -ENODEV / -EINVAL /
 * -EFAULT for parse / power-state / range-check failures (no result stash).
 */
static ssize_t vdec_diag_lmem_dma_test_write(struct file *f,
					     const char __user *ubuf,
					     size_t count, loff_t *ppos)
{
	struct vdec_diag_engine *eng = engine_from_file(f);
	struct amvdec_core *core = eng->core;
	struct device *dev = core->dev_dec;
	char buf[VDEC_DIAG_LMEM_TEST_BUF_LEN];
	char *p, *tok;
	u32 ctrl, dma_count, adr;
	bool adr_explicit = false;
	ktime_t t_start, t_end;
	u32 iters;
	int ret;

	if (count >= sizeof(buf))
		return -E2BIG;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	p = strim(buf);

	tok = strsep(&p, " \t");
	if (!tok || !*tok)
		return -EINVAL;
	ret = kstrtou32(tok, 0, &ctrl);
	if (ret)
		return ret;

	tok = strsep(&p, " \t");
	if (!tok || !*tok)
		return -EINVAL;
	ret = kstrtou32(tok, 0, &dma_count);
	if (ret)
		return ret;

	tok = strsep(&p, " \t");
	if (tok && *tok) {
		ret = kstrtou32(tok, 0, &adr);
		if (ret)
			return ret;
		adr_explicit = true;
	} else {
		adr = (u32)eng->workspace_dma;
	}

	/*
	 * count is bounded by the workspace size when using the default
	 * adr (workspace is 4 KiB = 1024 words).  For an explicit user-
	 * provided adr the user owns lifetime of that buffer, so we still
	 * accept up to 4096 words but never larger -- nothing about the
	 * engine has been characterized for >16 KiB transfers and the
	 * harness is for characterization, not bulk DMA.
	 */
	if (dma_count == 0)
		return -EINVAL;
	if (!adr_explicit && dma_count > VDEC_DIAG_WORKSPACE_WORDS)
		return -EINVAL;
	if (dma_count > 4096)
		return -EINVAL;

	if (!engine_powered(eng)) {
		dev_warn(dev, "diag: lmem_dma_test rejected -- engine not powered\n");
		return -ENODEV;
	}

	mutex_lock(&eng->lock);

	dev_info(dev,
		 "diag: lmem_dma_test ctrl=0x%08x count=%u adr=0x%08x%s by pid=%d\n",
		 ctrl, dma_count, adr,
		 adr_explicit ? " (explicit)" : " (workspace)",
		 current->pid);

	amvdec_write_dos(core, LMEM_DMA_ADR, adr);
	amvdec_write_dos(core, LMEM_DMA_COUNT, dma_count);

	t_start = ktime_get();
	amvdec_write_dos(core, LMEM_DMA_CTRL, ctrl);

	for (iters = 0; iters < VDEC_DIAG_LMEM_POLL_MAX; iters++) {
		if (!(amvdec_read_dos(core, LMEM_DMA_CTRL) & LMEM_DMA_CTRL_START))
			break;
		usleep_range(1, 2);
	}
	t_end = ktime_get();

	eng->last_result_valid = true;
	eng->last_result_hang = (iters >= VDEC_DIAG_LMEM_POLL_MAX);
	eng->last_ctrl = ctrl;
	eng->last_count = dma_count;
	eng->last_adr = adr;
	eng->last_iters = iters;
	eng->last_wrrsp = amvdec_read_dos(core, WRRSP_LMEM);
	eng->last_ms = (u32)ktime_to_ms(ktime_sub(t_end, t_start));

	dev_info(dev,
		 "diag: lmem_dma_test result=%s iters=%u wrrsp=0x%08x ms=%u\n",
		 eng->last_result_hang ? "hang" : "ok",
		 eng->last_iters, eng->last_wrrsp, eng->last_ms);

	mutex_unlock(&eng->lock);

	return count;
}

static const struct file_operations vdec_diag_lmem_dma_test_fops = {
	.open = simple_open,
	.write = vdec_diag_lmem_dma_test_write,
	.llseek = noop_llseek,
};

/* ---- lmem_dma_result: read back the most recent lmem_dma_test result ---- */
/*
 * Output is one key=value line per attribute.  If no lmem_dma_test write
 * has happened since boot, returns "result=noinit\n" only.
 */
static int vdec_diag_lmem_dma_result_show(struct seq_file *s, void *unused)
{
	struct vdec_diag_engine *eng = s->private;

	mutex_lock(&eng->lock);
	if (!eng->last_result_valid) {
		seq_puts(s, "result=noinit\n");
		mutex_unlock(&eng->lock);
		return 0;
	}

	seq_printf(s, "result=%s\n", eng->last_result_hang ? "hang" : "ok");
	seq_printf(s, "ctrl=0x%08x\n", eng->last_ctrl);
	seq_printf(s, "count=%u\n", eng->last_count);
	seq_printf(s, "adr=0x%08x\n", eng->last_adr);
	seq_printf(s, "iters=%u\n", eng->last_iters);
	seq_printf(s, "wrrsp=0x%08x\n", eng->last_wrrsp);
	seq_printf(s, "ms=%u\n", eng->last_ms);
	mutex_unlock(&eng->lock);
	return 0;
}

static int vdec_diag_lmem_dma_result_open(struct inode *i, struct file *f)
{
	return single_open(f, vdec_diag_lmem_dma_result_show, i->i_private);
}

static const struct file_operations vdec_diag_lmem_dma_result_fops = {
	.open = vdec_diag_lmem_dma_result_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* ---- init / exit -------------------------------------------------------- */

int vdec_diag_init(struct amvdec_core *core)
{
	struct vdec_diag *diag;
	struct vdec_diag_engine *eng;

	if (!vdec_diag_enabled)
		return 0;

	diag = devm_kzalloc(core->dev, sizeof(*diag), GFP_KERNEL);
	if (!diag)
		return -ENOMEM;

	/* Reuse vdec_debugfs's meson-vdec root; don't create a second
	 * copy (kernel would silently succeed-but-orphan the second dir).
	 * vdec_debugfs_init runs earlier in vdec_probe so core->debugfs.root
	 * is already set unless debugfs failed entirely. */
	if (!core->debugfs.root) {
		dev_warn(core->dev, "diag: vdec_debugfs root not available\n");
		return -ENODEV;
	}
	diag->root = core->debugfs.root;

	diag->diag_dir = debugfs_create_dir("diag", diag->root);
	if (IS_ERR(diag->diag_dir)) {
		debugfs_remove_recursive(diag->diag_dir);
		return PTR_ERR(diag->diag_dir);
	}

	eng = &diag->vdec1;
	eng->core = core;
	eng->name = "vdec1";
	mutex_init(&eng->lock);

	eng->fw_buf_size = VDEC_DIAG_MC_SIZE;
	eng->fw_buf = dma_alloc_coherent(core->dev, eng->fw_buf_size,
					 &eng->fw_buf_dma, GFP_KERNEL);
	if (!eng->fw_buf) {
		debugfs_remove_recursive(diag->diag_dir);
		return -ENOMEM;
	}

	/*
	 * LMEM_DMA characterization workspace.  4 KiB DMA-coherent buffer
	 * reused across every lmem_dma_test fire; freed in vdec_diag_exit.
	 * Allocation failure here unwinds fw_buf and the diag dir; the
	 * driver continues without diag.
	 */
	eng->workspace_buf = dma_alloc_coherent(core->dev,
						VDEC_DIAG_WORKSPACE_SIZE,
						&eng->workspace_dma,
						GFP_KERNEL);
	if (!eng->workspace_buf) {
		dma_free_coherent(core->dev, eng->fw_buf_size,
				  eng->fw_buf, eng->fw_buf_dma);
		eng->fw_buf = NULL;
		debugfs_remove_recursive(diag->diag_dir);
		return -ENOMEM;
	}

	eng->dir = debugfs_create_dir("vdec1", diag->diag_dir);

	debugfs_create_file("firmware", 0200, eng->dir, eng, &vdec_diag_firmware_fops);
	debugfs_create_file("reset", 0200, eng->dir, eng, &vdec_diag_reset_fops);
	debugfs_create_file("state", 0400, eng->dir, eng, &vdec_diag_state_fops);
	debugfs_create_file("peek", 0600, eng->dir, eng, &vdec_diag_peek_fops);
	debugfs_create_file("poke", 0200, eng->dir, eng, &vdec_diag_poke_fops);
	debugfs_create_file("mode", 0200, eng->dir, eng, &vdec_diag_mode_fops);
	debugfs_create_file("cmd", 0200, eng->dir, eng, &vdec_diag_cmd_fops);
	/* poweron: bring engine up without a V4L2 session (added with the
	 * poweron_write function in commit 3aba09efd706; the debugfs_create
	 * line was accidentally omitted from that commit). */
	debugfs_create_file("poweron", 0200, eng->dir, eng, &vdec_diag_poweron_fops);
	/*
	 * LMEM_DMA characterization quartet (M4.1.7 ROOT CAUSE workstream):
	 *   lmem_dma_test       -- fire one LMEM_DMA pulse with parsed args
	 *   lmem_dma_result     -- read the most recent fire's outcome
	 *   workspace_paddr     -- physical address of the DMA workspace
	 *   workspace_pattern   -- prefill the workspace with seed + i*4
	 * Used to empirically characterize LMEM_DMA_CTRL bit fields against
	 * the BSP firmware's observed encodings (see
	 * amlogic/software/codec/decoder/stateless/ANALYSIS.md
	 * "2026-04-24 -- M4.1.7 ROOT CAUSE").
	 */
	debugfs_create_file("lmem_dma_test", 0200, eng->dir, eng,
			    &vdec_diag_lmem_dma_test_fops);
	debugfs_create_file("lmem_dma_result", 0400, eng->dir, eng,
			    &vdec_diag_lmem_dma_result_fops);
	debugfs_create_file("workspace_paddr", 0400, eng->dir, eng,
			    &vdec_diag_workspace_paddr_fops);
	debugfs_create_file("workspace_pattern", 0600, eng->dir, eng,
			    &vdec_diag_workspace_pattern_fops);

	core->diag = diag;
	dev_info(core->dev, "diag: debugfs interface ready at /sys/kernel/debug/meson-vdec/diag/vdec1/\n");
	return 0;
}

void vdec_diag_exit(struct amvdec_core *core)
{
	struct vdec_diag *diag = core->diag;
	struct vdec_diag_engine *eng;

	if (!diag)
		return;

	eng = &diag->vdec1;

	/* If diag manual-powered the engine, power it off before teardown
	 * so clocks/rails are released cleanly. */
	if (eng->manual_powered) {
		vdec_diag_poweroff(core);
		eng->manual_powered = false;
	}

	if (eng->fw_buf)
		dma_free_coherent(core->dev, eng->fw_buf_size,
				  eng->fw_buf, eng->fw_buf_dma);
	if (eng->workspace_buf)
		dma_free_coherent(core->dev, VDEC_DIAG_WORKSPACE_SIZE,
				  eng->workspace_buf, eng->workspace_dma);
	/* diag->root is shared with vdec_debugfs; only remove the diag_dir
	 * subtree we own, not the shared root. vdec_debugfs_remove()
	 * handles the rest. */
	debugfs_remove_recursive(diag->diag_dir);
	core->diag = NULL;
}
