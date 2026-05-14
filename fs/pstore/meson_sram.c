// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic AO SRAM pstore backend
 * Copyright (C) 2026 Libre Computer
 * Author: Da Xue <da@libre.computer>
 *
 * Uses the always-on SRAM at 0xC8000000 (80KB) which retains state across
 * warm resets to provide persistent crash log and console storage.
 *
 * The BL30 mailbox regions occupy offsets 0x13000-0x137FF. The first 76KB
 * (offsets 0x0000-0x12FFF) is safe. A 4KB guard at offset 0x0000 avoids
 * any early-boot firmware scratch. Recommended DT region: 0x1000..0x5000.
 *
 * SRAM layout (all offsets relative to region base):
 *   [0x000] global header (PS_HDR_SIZE bytes)
 *   [PS_HDR_SIZE] console zone (con_size bytes) -- ring buffer
 *   [PS_HDR_SIZE + con_size] dmesg zone (dmg_size bytes) -- single slot
 *
 * Zone sizes are derived at probe from the DT region size (60% console,
 * 40% dmesg, 4-byte aligned).
 *
 * DT binding (child of sram@c8000000):
 *   pstore@1000 {
 *       compatible = "amlogic,meson-sram-pstore";
 *       reg = <0x1000 0x5000>;
 *   };
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kmsg_dump.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pstore.h>
#include <linux/slab.h>

#define MESON_PS_MAGIC    0x4D535053U  /* "MSPS" */
#define MESON_CON_SIG     0x434F4E53U  /* "CONS" */
#define MESON_DMG_SIG     0x444D5347U  /* "DMSG" */
#define MESON_PS_VERSION  1

/*
 * Global header (PS_HDR_SIZE = 32 bytes):
 *   +0  u32  magic
 *   +4  u8   version
 *   +5  u8   flags  (PS_FLAG_*)
 *   +6  u16  oops_count
 *   +8  u32  con_size (console zone total bytes, incl. header)
 *   +12 u32  dmg_size (dmesg zone total bytes, incl. header)
 *   +16 [16 bytes reserved]
 */
#define PS_HDR_MAGIC     0
#define PS_HDR_VERSION   4
#define PS_HDR_FLAGS     5
#define PS_HDR_OOPS_CNT  6
#define PS_HDR_CON_SIZE  8
#define PS_HDR_DMG_SIZE  12
#define PS_HDR_SIZE      32

#define PS_FLAG_CON_VALID  BIT(0)
#define PS_FLAG_DMG_VALID  BIT(1)

/*
 * Console zone header (CON_HDR_LEN = 12 bytes, followed by ring data):
 *   +0  u32  sig
 *   +4  u32  start  (read pointer -- offset into data, 0..data_size-1)
 *   +8  u32  size   (valid bytes -- 0..data_size)
 */
#define CON_HDR_SIG    0
#define CON_HDR_START  4
#define CON_HDR_SIZE   8
#define CON_HDR_LEN    12

/*
 * Dmesg zone header (DMG_HDR_LEN = 32 bytes, followed by crash data):
 *   +0  u32  sig
 *   +4  u32  size    (valid data bytes)
 *   +8  u32  reason  (enum kmsg_dump_reason)
 *   +12 u32  count   (oops count)
 *   +16 u32  tsec_lo
 *   +20 u32  tsec_hi
 *   +24 u32  tnsec_lo
 *   +28 u32  tnsec_hi
 */
#define DMG_HDR_SIG       0
#define DMG_HDR_SIZE      4
#define DMG_HDR_REASON    8
#define DMG_HDR_COUNT     12
#define DMG_HDR_TSEC_LO   16
#define DMG_HDR_TSEC_HI   20
#define DMG_HDR_TNSEC_LO  24
#define DMG_HDR_TNSEC_HI  28
#define DMG_HDR_LEN       32

struct meson_sram_ps {
	void __iomem      *base;
	size_t             total_size;
	size_t             con_size;   /* console zone total (incl. CON_HDR_LEN) */
	size_t             dmg_size;   /* dmesg zone total (incl. DMG_HDR_LEN) */
	int                read_step;
	struct pstore_info pstore;
};

#define to_meson_ps(p)  container_of((p), struct meson_sram_ps, pstore)

static inline void __iomem *con_zone(struct meson_sram_ps *ctx)
{
	return ctx->base + PS_HDR_SIZE;
}

static inline void __iomem *dmg_zone(struct meson_sram_ps *ctx)
{
	return ctx->base + PS_HDR_SIZE + ctx->con_size;
}

static inline size_t con_data_size(struct meson_sram_ps *ctx)
{
	return ctx->con_size - CON_HDR_LEN;
}

static inline size_t dmg_data_size(struct meson_sram_ps *ctx)
{
	return ctx->dmg_size - DMG_HDR_LEN;
}

/* ioread/iowrite handle LE byte order internally -- no cpu_to_le needed */

static void meson_sram_ps_init(struct meson_sram_ps *ctx)
{
	memset_io(ctx->base, 0, ctx->total_size);
	iowrite32(MESON_PS_MAGIC,      ctx->base + PS_HDR_MAGIC);
	iowrite8(MESON_PS_VERSION,     ctx->base + PS_HDR_VERSION);
	iowrite32(ctx->con_size,       ctx->base + PS_HDR_CON_SIZE);
	iowrite32(ctx->dmg_size,       ctx->base + PS_HDR_DMG_SIZE);
	iowrite32(MESON_CON_SIG,       con_zone(ctx) + CON_HDR_SIG);
	iowrite32(MESON_DMG_SIG,       dmg_zone(ctx) + DMG_HDR_SIG);
}

static bool meson_sram_ps_valid(struct meson_sram_ps *ctx)
{
	return ioread32(ctx->base + PS_HDR_MAGIC)    == MESON_PS_MAGIC  &&
	       ioread8(ctx->base + PS_HDR_VERSION)   == MESON_PS_VERSION &&
	       ioread32(ctx->base + PS_HDR_CON_SIZE) == ctx->con_size   &&
	       ioread32(ctx->base + PS_HDR_DMG_SIZE) == ctx->dmg_size   &&
	       ioread32(con_zone(ctx) + CON_HDR_SIG) == MESON_CON_SIG   &&
	       ioread32(dmg_zone(ctx) + DMG_HDR_SIG) == MESON_DMG_SIG;
}

static int meson_ps_open(struct pstore_info *psi)
{
	to_meson_ps(psi)->read_step = 0;
	return 0;
}

static int meson_ps_close(struct pstore_info *psi)
{
	return 0;
}

static ssize_t meson_ps_read(struct pstore_record *record)
{
	struct meson_sram_ps *ctx = to_meson_ps(record->psi);
	u8 flags = ioread8(ctx->base + PS_HDR_FLAGS);

	while (ctx->read_step < 2) {
		int step = ctx->read_step++;

		if (step == 0) {
			/* Console record */
			void __iomem *zone = con_zone(ctx);
			size_t dsz = con_data_size(ctx);
			u32 start, size;
			char *buf;

			if (!(flags & PS_FLAG_CON_VALID))
				continue;

			size  = ioread32(zone + CON_HDR_SIZE);
			start = ioread32(zone + CON_HDR_START);
			if (!size || size > dsz)
				continue;

			buf = kmalloc(size, GFP_KERNEL);
			if (!buf)
				return -ENOMEM;

			/*
			 * Ring buffer may wrap: copy in at most two segments.
			 * [start .. dsz-1] then [0 .. remainder-1]
			 */
			if (start + size <= dsz) {
				memcpy_fromio(buf,
					      zone + CON_HDR_LEN + start,
					      size);
			} else {
				size_t tail = dsz - start;
				memcpy_fromio(buf,
					      zone + CON_HDR_LEN + start,
					      tail);
				memcpy_fromio(buf + tail,
					      zone + CON_HDR_LEN,
					      size - tail);
			}

			record->type = PSTORE_TYPE_CONSOLE;
			record->id   = 0;
			record->buf  = buf;
			record->size = size;
			return size;

		} else {
			/* Dmesg record */
			void __iomem *zone = dmg_zone(ctx);
			size_t dsz = dmg_data_size(ctx);
			u32 size, reason, count;
			u64 tsec, tnsec;
			char *buf;

			if (!(flags & PS_FLAG_DMG_VALID))
				continue;

			size   = ioread32(zone + DMG_HDR_SIZE);
			reason = ioread32(zone + DMG_HDR_REASON);
			count  = ioread32(zone + DMG_HDR_COUNT);
			tsec   = (u64)ioread32(zone + DMG_HDR_TSEC_HI) << 32 |
				  ioread32(zone + DMG_HDR_TSEC_LO);
			tnsec  = (u64)ioread32(zone + DMG_HDR_TNSEC_HI) << 32 |
				  ioread32(zone + DMG_HDR_TNSEC_LO);

			if (!size || size > dsz)
				continue;

			buf = kmalloc(size, GFP_KERNEL);
			if (!buf)
				return -ENOMEM;

			memcpy_fromio(buf, zone + DMG_HDR_LEN, size);

			record->type         = PSTORE_TYPE_DMESG;
			record->id           = 1;
			record->reason       = (enum kmsg_dump_reason)reason;
			record->count        = count;
			record->time.tv_sec  = (time64_t)tsec;
			record->time.tv_nsec = (long)tnsec;
			record->buf          = buf;
			record->size         = size;
			return size;
		}
	}

	return 0; /* no more records */
}

static int meson_ps_write(struct pstore_record *record)
{
	struct meson_sram_ps *ctx = to_meson_ps(record->psi);

	if (record->type == PSTORE_TYPE_CONSOLE) {
		void __iomem *zone = con_zone(ctx);
		size_t dsz = con_data_size(ctx);
		size_t len = min_t(size_t, record->size, dsz);
		u32 start = ioread32(zone + CON_HDR_START);
		u32 cur   = ioread32(zone + CON_HDR_SIZE);
		u32 write_off = (start + cur) % dsz;

		if (write_off + len <= dsz) {
			memcpy_toio(zone + CON_HDR_LEN + write_off,
				    record->buf, len);
		} else {
			size_t tail = dsz - write_off;
			memcpy_toio(zone + CON_HDR_LEN + write_off,
				    record->buf, tail);
			memcpy_toio(zone + CON_HDR_LEN,
				    record->buf + tail, len - tail);
		}

		if (cur + len <= dsz) {
			cur += len;
		} else {
			/* Buffer full: advance start past overwritten bytes */
			start = (write_off + len) % dsz;
			cur = dsz;
		}

		iowrite32(start, zone + CON_HDR_START);
		iowrite32(cur,   zone + CON_HDR_SIZE);
		iowrite8(ioread8(ctx->base + PS_HDR_FLAGS) | PS_FLAG_CON_VALID,
			 ctx->base + PS_HDR_FLAGS);

	} else if (record->type == PSTORE_TYPE_DMESG) {
		void __iomem *zone = dmg_zone(ctx);
		size_t dsz = dmg_data_size(ctx);
		size_t len = min_t(size_t, record->size, dsz);
		u64 tsec  = (u64)record->time.tv_sec;
		u64 tnsec = (u64)record->time.tv_nsec;
		u16 oops;

		oops = ioread16(ctx->base + PS_HDR_OOPS_CNT);
		iowrite16(oops + 1, ctx->base + PS_HDR_OOPS_CNT);

		iowrite32((u32)len,              zone + DMG_HDR_SIZE);
		iowrite32((u32)record->reason,   zone + DMG_HDR_REASON);
		iowrite32((u32)record->count,    zone + DMG_HDR_COUNT);
		iowrite32((u32)(tsec  & 0xffffffffU), zone + DMG_HDR_TSEC_LO);
		iowrite32((u32)(tsec  >> 32),         zone + DMG_HDR_TSEC_HI);
		iowrite32((u32)(tnsec & 0xffffffffU), zone + DMG_HDR_TNSEC_LO);
		iowrite32((u32)(tnsec >> 32),         zone + DMG_HDR_TNSEC_HI);
		memcpy_toio(zone + DMG_HDR_LEN, record->buf, len);

		iowrite8(ioread8(ctx->base + PS_HDR_FLAGS) | PS_FLAG_DMG_VALID,
			 ctx->base + PS_HDR_FLAGS);
	}

	return 0;
}

static int meson_ps_erase(struct pstore_record *record)
{
	struct meson_sram_ps *ctx = to_meson_ps(record->psi);
	u8 flags = ioread8(ctx->base + PS_HDR_FLAGS);

	if (record->type == PSTORE_TYPE_CONSOLE) {
		memset_io(con_zone(ctx), 0, ctx->con_size);
		iowrite32(MESON_CON_SIG, con_zone(ctx) + CON_HDR_SIG);
		iowrite8(flags & ~PS_FLAG_CON_VALID, ctx->base + PS_HDR_FLAGS);
	} else if (record->type == PSTORE_TYPE_DMESG) {
		memset_io(dmg_zone(ctx), 0, ctx->dmg_size);
		iowrite32(MESON_DMG_SIG, dmg_zone(ctx) + DMG_HDR_SIG);
		iowrite8(flags & ~PS_FLAG_DMG_VALID, ctx->base + PS_HDR_FLAGS);
	}

	return 0;
}

static int meson_sram_ps_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct meson_sram_ps *ctx;
	struct resource *res;
	size_t usable;
	char *buf;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	ctx->total_size = resource_size(res);
	usable = ctx->total_size - PS_HDR_SIZE;

	if (ctx->total_size < PS_HDR_SIZE + CON_HDR_LEN + DMG_HDR_LEN + 128) {
		dev_err(dev, "region too small: %zu bytes\n", ctx->total_size);
		return -EINVAL;
	}

	/* Split usable space: 60% console, 40% dmesg, 4-byte aligned */
	ctx->con_size = ALIGN_DOWN((usable * 3) / 5, 4);
	ctx->dmg_size = usable - ctx->con_size;

	ctx->base = devm_ioremap(dev, res->start, ctx->total_size);
	if (!ctx->base)
		return -ENOMEM;

	if (!meson_sram_ps_valid(ctx)) {
		dev_info(dev, "SRAM header invalid or zone size mismatch -- reinitializing\n");
		meson_sram_ps_init(ctx);
	}

	/*
	 * pstore core writes crash data into psi->buf (DRAM) before calling
	 * write(); our write() copies it to SRAM.  Size to fit one dmesg slot.
	 */
	buf = devm_kzalloc(dev, dmg_data_size(ctx), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ctx->pstore.owner      = THIS_MODULE;
	ctx->pstore.name       = "meson-sram";
	ctx->pstore.flags      = PSTORE_FLAGS_DMESG | PSTORE_FLAGS_CONSOLE;
	ctx->pstore.buf        = buf;
	ctx->pstore.bufsize    = dmg_data_size(ctx);
	ctx->pstore.max_reason = KMSG_DUMP_PANIC;
	ctx->pstore.open       = meson_ps_open;
	ctx->pstore.close      = meson_ps_close;
	ctx->pstore.read       = meson_ps_read;
	ctx->pstore.write      = meson_ps_write;
	ctx->pstore.erase      = meson_ps_erase;
	raw_spin_lock_init(&ctx->pstore.buf_lock);

	platform_set_drvdata(pdev, ctx);

	ret = pstore_register(&ctx->pstore);
	if (ret) {
		dev_err(dev, "pstore_register failed: %d\n", ret);
		return ret;
	}

	dev_info(dev, "registered: base=%pa total=%zuKB con=%zuKB dmg=%zuKB\n",
		 &res->start,
		 ctx->total_size / 1024,
		 con_data_size(ctx) / 1024,
		 dmg_data_size(ctx) / 1024);
	return 0;
}

static void meson_sram_ps_remove(struct platform_device *pdev)
{
	struct meson_sram_ps *ctx = platform_get_drvdata(pdev);
	pstore_unregister(&ctx->pstore);
}

static const struct of_device_id meson_sram_ps_of_ids[] = {
	{ .compatible = "amlogic,meson-sram-pstore" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, meson_sram_ps_of_ids);

static struct platform_driver meson_sram_ps_driver = {
	.probe  = meson_sram_ps_probe,
	.remove = meson_sram_ps_remove,
	.driver = {
		.name           = "meson-sram-pstore",
		.of_match_table = meson_sram_ps_of_ids,
	},
};
module_platform_driver(meson_sram_ps_driver);

MODULE_AUTHOR("Da Xue <da@libre.computer>");
MODULE_DESCRIPTION("Amlogic AO SRAM pstore backend");
MODULE_LICENSE("GPL v2");
