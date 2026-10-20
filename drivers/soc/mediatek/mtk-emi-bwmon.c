// SPDX-License-Identifier: GPL-2.0
/*
 * mtk-emi-bwmon: MT8195 EMI bus / DRAM-bandwidth traffic monitor (polled).
 *
 * The MT8195 (= MT8395 die) EMI0 controller carries a bus-traffic monitor block
 * inside EMI0_REG @ 0x10219000. It is a polled start/read-counter engine (no
 * interrupt): the counters free-run and accumulate DRAM transactions while the
 * monitor is enabled; software restarts them, waits a sample window, then reads
 * the accumulated deltas. The headline counters are:
 *
 *   EMI_WACT (0x420) -- double-word counter of ALL masters (total DRAM traffic).
 *                       Each count == one 8-byte double-word, so bytes = WACT*8.
 *   EMI_WSCT (0x428) -- double-word counter of the SELECTED masters (a subset
 *                       chosen by EMI_MSEL/EMI_BMID; bytes = WSCT*8).
 *   EMI_TACT (0x410) -- transaction counter of ALL masters.
 *   EMI_BACT (0x430) -- bandwidth counter of the SELECTED masters.
 *
 * Programming model (established empirically on an MT8195 Tomato Chromebook via
 * /dev/mem, since the datasheet gives the register map but not the BMEN bitfields):
 *   - The firmware leaves EMI_BMEN at a boot-default config (0x33ff0101 on this
 *     die) that selects the masters/mode; the counters are already free-running.
 *   - A RELIABLE restart is the two-write sequence: write EMI_BMEN = 0 (stop),
 *     then write EMI_BMEN = <config> (start). This clears the counters and lets
 *     them free-run from ~0. A single config write does NOT reliably restart.
 *   - The counters must be read WHILE the monitor runs. Writing EMI_BMEN = 0
 *     FREEZES the counters, so the window is bounded by our wall-clock msleep and
 *     the post-read happens before any stop.
 * The measurement is wall-clock based (elapsed ktime around the window), so it
 * needs neither the EMI clock frequency nor the auto-stop bitfield.
 *
 * Register map sourced from the NDA MT8395 Application Processor Registers v0.2
 * (MT8195 shares the MT8395 die); see
 * mediatek/reference/soc/mt8195/emi-bw-monitor-registers.md.
 *
 * Exposure: a debugfs interface (mtk_emi_bwmon/) rather than a drivers/perf PMU
 * -- the "restart a window, then read a free-running counter" model does not map
 * cleanly onto perf's per-event count/start/stop core, and a debugfs sample
 * reader is the simplest correct exposure for a readable DRAM-BW number:
 *
 *   window_ms  (rw) sample-window length in milliseconds (default 100).
 *   measure    (r)  run one window and print the WACT/WSCT/TACT/BACT deltas plus
 *                   the derived total-DRAM bandwidth in MB/s (from WACT*8).
 *   regs       (r)  raw dump of the monitor registers as they stand now.
 *   poke       (w)  "0xOFF 0xVAL" -- write VAL to register offset OFF within the
 *                   EMI0_REG block (root-only debug aid, e.g. to experiment with
 *                   EMI_MSEL/EMI_BMID master selection on-board without a rebuild).
 */
#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

/* Offsets within EMI0_REG (base 0x10219000). */
#define EMI_BMEN	0x400	/* bus monitor control / config                   */
#define EMI_BSTP	0x404	/* auto-stop sample-window count (unused, polled) */
#define EMI_BCNT	0x408	/* 3rd double-word counter, selected master       */
#define EMI_TACT	0x410	/* transaction counter, ALL masters               */
#define EMI_TSCT	0x418	/* transaction counter, selected masters          */
#define EMI_WACT	0x420	/* double-word counter, ALL masters (total DRAM)  */
#define EMI_WSCT	0x428	/* double-word counter, selected masters          */
#define EMI_BACT	0x430	/* bandwidth counter, selected masters            */
#define EMI_BSCT	0x438	/* overhead-word counter, selected masters        */
#define EMI_MSEL	0x440	/* master control of 2nd/3rd bus monitor          */

/* Boot-default EMI_BMEN config (master/mode select) if the register reads unset. */
#define EMI_BMEN_DEFAULT_CFG	0x33ff0101

#define EMI_REG_SPAN	0x1000
#define EMI_DWORD_BYTES	8	/* one WACT/WSCT count == one 8-byte double-word */

struct mtk_emi_bwmon {
	struct device	*dev;
	void __iomem	*base;
	struct dentry	*dbg;
	u32		window_ms;
	u32		cfg;		/* EMI_BMEN config used to (re)start the monitor */
};

struct emi_sample {
	u32 wact_pre, wact_post;	/* all-master dwords    */
	u32 wsct_pre, wsct_post;	/* selected-master dwords */
	u32 tact_pre, tact_post;	/* all-master txns      */
	u32 bact_pre, bact_post;	/* selected-master BW   */
	u64 elapsed_ns;
};

/*
 * Run one sample window and fill *out. The counters free-run once (re)started,
 * so we snapshot, sleep window_ms, snapshot again -- all while the monitor runs
 * (writing EMI_BMEN=0 would freeze them). WACT is 32-bit; a short window keeps
 * it clear of wrap even past 10 GB/s, and the deltas are u32 wrap-safe anyway.
 */
static void mtk_emi_run(struct mtk_emi_bwmon *bw, struct emi_sample *s)
{
	ktime_t t0, t1;

	/* Reliable restart: stop (clears counters) then re-arm with the config. */
	writel(0, bw->base + EMI_BMEN);
	writel(bw->cfg, bw->base + EMI_BMEN);
	readl(bw->base + EMI_BMEN);		/* settle the re-arm before snapshot */

	s->wact_pre = readl(bw->base + EMI_WACT);
	s->wsct_pre = readl(bw->base + EMI_WSCT);
	s->tact_pre = readl(bw->base + EMI_TACT);
	s->bact_pre = readl(bw->base + EMI_BACT);

	t0 = ktime_get();
	msleep(bw->window_ms);
	t1 = ktime_get();

	/* Read WHILE the monitor is still running -- do NOT stop first. */
	s->wact_post = readl(bw->base + EMI_WACT);
	s->wsct_post = readl(bw->base + EMI_WSCT);
	s->tact_post = readl(bw->base + EMI_TACT);
	s->bact_post = readl(bw->base + EMI_BACT);

	s->elapsed_ns = ktime_to_ns(ktime_sub(t1, t0));
}

static u64 emi_bw_mbps(u32 dwords, u64 elapsed_ns)
{
	u64 bytes = (u64)dwords * EMI_DWORD_BYTES;

	if (!elapsed_ns)
		return 0;
	/* MB/s = bytes/s / 1e6 = bytes*1e9/ns / 1e6 = bytes*1000/ns. */
	return div64_u64(bytes * 1000, elapsed_ns);
}

static int emi_measure_show(struct seq_file *m, void *unused)
{
	struct mtk_emi_bwmon *bw = m->private;
	struct emi_sample s;
	u32 wact_d, wsct_d, tact_d, bact_d;

	mtk_emi_run(bw, &s);

	wact_d = s.wact_post - s.wact_pre;	/* u32 wrap-safe */
	wsct_d = s.wsct_post - s.wsct_pre;
	tact_d = s.tact_post - s.tact_pre;
	bact_d = s.bact_post - s.bact_pre;

	seq_printf(m, "window_ms      : %u\n", bw->window_ms);
	seq_printf(m, "elapsed_ns     : %llu\n", s.elapsed_ns);
	seq_printf(m, "EMI_BMEN cfg   : 0x%08x\n", bw->cfg);
	seq_printf(m, "EMI_WACT       : %10u dwords (all masters)      [%#x -> %#x]\n",
		   wact_d, s.wact_pre, s.wact_post);
	seq_printf(m, "EMI_WSCT       : %10u dwords (selected masters) [%#x -> %#x]\n",
		   wsct_d, s.wsct_pre, s.wsct_post);
	seq_printf(m, "EMI_TACT       : %10u txns   (all masters)      [%#x -> %#x]\n",
		   tact_d, s.tact_pre, s.tact_post);
	seq_printf(m, "EMI_BACT       : %10u        (selected masters) [%#x -> %#x]\n",
		   bact_d, s.bact_pre, s.bact_post);
	seq_printf(m, "--\n");
	seq_printf(m, "total DRAM BW  : %llu MB/s (%llu bytes, WACT*8)\n",
		   emi_bw_mbps(wact_d, s.elapsed_ns),
		   (u64)wact_d * EMI_DWORD_BYTES);
	seq_printf(m, "selected-mst BW: %llu MB/s (%llu bytes, WSCT*8)\n",
		   emi_bw_mbps(wsct_d, s.elapsed_ns),
		   (u64)wsct_d * EMI_DWORD_BYTES);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(emi_measure);

static int emi_regs_show(struct seq_file *m, void *unused)
{
	struct mtk_emi_bwmon *bw = m->private;
	static const struct { const char *n; u32 off; } r[] = {
		{ "EMI_BMEN", EMI_BMEN }, { "EMI_BSTP", EMI_BSTP },
		{ "EMI_BCNT", EMI_BCNT }, { "EMI_TACT", EMI_TACT },
		{ "EMI_TSCT", EMI_TSCT }, { "EMI_WACT", EMI_WACT },
		{ "EMI_WSCT", EMI_WSCT }, { "EMI_BACT", EMI_BACT },
		{ "EMI_BSCT", EMI_BSCT }, { "EMI_MSEL", EMI_MSEL },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(r); i++)
		seq_printf(m, "%-9s +0x%03x = 0x%08x\n",
			   r[i].n, r[i].off, readl(bw->base + r[i].off));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(emi_regs);

static ssize_t emi_poke_write(struct file *file, const char __user *ubuf,
			      size_t len, loff_t *ppos)
{
	struct mtk_emi_bwmon *bw = file_inode(file)->i_private;
	char buf[64];
	u32 off, val;

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';

	if (sscanf(buf, "%i %i", &off, &val) != 2)
		return -EINVAL;
	if (off & 0x3 || off >= EMI_REG_SPAN)
		return -EINVAL;

	writel(val, bw->base + off);
	dev_info(bw->dev, "poke +0x%03x <= 0x%08x\n", off, val);
	return len;
}

static const struct file_operations emi_poke_fops = {
	.owner	= THIS_MODULE,
	.write	= emi_poke_write,
	.open	= simple_open,
	.llseek	= default_llseek,
};

static int mtk_emi_bwmon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_emi_bwmon *bw;

	bw = devm_kzalloc(dev, sizeof(*bw), GFP_KERNEL);
	if (!bw)
		return -ENOMEM;

	bw->dev = dev;
	bw->window_ms = 100;

	bw->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(bw->base))
		return dev_err_probe(dev, PTR_ERR(bw->base),
				     "failed to map EMI0_REG\n");

	/* Capture the firmware's boot-default monitor config (master/mode select).
	 * Force the enable + restart bits so a re-arm always (re)starts counting;
	 * fall back to the known default if the register reads unconfigured. */
	bw->cfg = readl(bw->base + EMI_BMEN);
	if (bw->cfg & 0xffff0000)
		bw->cfg |= 0x101;
	else
		bw->cfg = EMI_BMEN_DEFAULT_CFG;

	bw->dbg = debugfs_create_dir("mtk_emi_bwmon", NULL);
	debugfs_create_u32("window_ms", 0644, bw->dbg, &bw->window_ms);
	debugfs_create_file("measure", 0444, bw->dbg, bw, &emi_measure_fops);
	debugfs_create_file("regs", 0444, bw->dbg, bw, &emi_regs_fops);
	debugfs_create_file("poke", 0200, bw->dbg, bw, &emi_poke_fops);

	platform_set_drvdata(pdev, bw);
	dev_info(dev, "MT8195 EMI BW monitor ready (cfg=0x%08x, debugfs mtk_emi_bwmon/)\n",
		 bw->cfg);
	return 0;
}

static void mtk_emi_bwmon_remove(struct platform_device *pdev)
{
	struct mtk_emi_bwmon *bw = platform_get_drvdata(pdev);

	debugfs_remove_recursive(bw->dbg);
}

static const struct of_device_id mtk_emi_bwmon_match[] = {
	{ .compatible = "mediatek,mt8195-emi-bwmon" },
	{}
};
MODULE_DEVICE_TABLE(of, mtk_emi_bwmon_match);

static struct platform_driver mtk_emi_bwmon_driver = {
	.probe = mtk_emi_bwmon_probe,
	.remove = mtk_emi_bwmon_remove,
	.driver = {
		.name = "mtk-emi-bwmon",
		.of_match_table = mtk_emi_bwmon_match,
	},
};
module_platform_driver(mtk_emi_bwmon_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek MT8195 EMI DRAM-bandwidth traffic monitor (polled debugfs)");
