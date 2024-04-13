/*
 * reset driver for Nexell SoCs
 * (C) Copyright 2016 Nexell
 * Bongyu, KOO <freestyle@nexell.co.kr>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <libfdt.h>
#include <asm/io.h>
#include <fdtdec.h>
#include <reset-uclass.h>
#include <mapmem.h>
#include <errno.h>

#define NX_ID_TO_BANK(id)	((id >> 16) & 0xFF)
#define NX_ID_TO_OFFS(id)	(id & 0xFF)
#define NX_NUM_TO_ID(bank,offs)	((bank << 16) | offs)

#define RST_WORD(offs)		((offs) / 32)
#define RST_BIT(offs)		((offs) % 32)
#define RST_MASK(offs)		(1UL << ((offs) % 32))

DECLARE_GLOBAL_DATA_PTR;


/* reset bank */
struct nexell_reset_bank {
	char		*name;
	u32		nr_resets;
	void __iomem	*base;
};

static struct nexell_reset_bank reset_banks[] = {
	{ .name = "reset_sys",     .nr_resets = 53, },
	{ .name = "reset_tbus",    .nr_resets = 14, },
	{ .name = "reset_lbus",    .nr_resets = 18, },
	{ .name = "reset_bbus",    .nr_resets = 36, },
	{ .name = "reset_coda",    .nr_resets = 8,  },
	{ .name = "reset_disp",    .nr_resets = 103,},
	{ .name = "reset_usb",     .nr_resets = 11, },
	{ .name = "reset_hdmi",    .nr_resets = 10, },
	{ .name = "reset_wave",    .nr_resets = 5,  },
	{ .name = "reset_drex",    .nr_resets = 7,  },
	{ .name = "reset_wave420", .nr_resets = 5,  },
	{ .name = "reset_cpu",     .nr_resets = 1,  },
	{ .name = "reset_periclk", .nr_resets = 3,  }
};

static int nexell_reset_request(struct reset_ctl *reset_ctl)
{
	return 0;
}

static int nexell_reset_free(struct reset_ctl *reset_ctl)
{
	return 0;
}

static int nexell_reset_assert(struct reset_ctl *reset_ctl)
{
	unsigned long id = reset_ctl->id;
	u32 bank = NX_ID_TO_BANK(id);
	u32 offset = NX_ID_TO_OFFS(id);
	u32 bit = RST_BIT(offset);
	void __iomem *reg =
		reset_banks[bank].base + (RST_WORD(offset) * 4);
	u32 val;
	unsigned long flags;

	debug("%s: %d.%d [%p.%d]\n", __func__, bank, offset, reg, bit);

	spin_lock_irqsave(&ctrl->lock, flags);

	val = readl(reg);
	writel(val & ~RST_MASK(offset), reg);

	spin_unlock_irqrestore(&ctrl->lock, flags);

	return 0;
}

static int nexell_reset_deassert(struct reset_ctl *reset_ctl)
{
	unsigned long id = reset_ctl->id;
	u32 bank = NX_ID_TO_BANK(id);
	u32 offset = NX_ID_TO_OFFS(id);
	u32 bit = RST_BIT(offset);
	void __iomem *reg =
		reset_banks[bank].base + (RST_WORD(offset) * 4);
	u32 val;
	unsigned long flags;

	debug("%s: %d.%d [%p.%d]\n", __func__, bank, offset, reg, bit);

	spin_lock_irqsave(&ctrl->lock, flags);

	val = readl(reg);
	writel(val | RST_MASK(offset), reg);

	spin_unlock_irqrestore(&ctrl->lock, flags);

	return 0;
}

static int nexell_reset_xlate(struct reset_ctl *reset_ctl,
			      struct fdtdec_phandle_args *args)
{
	u32 nr_resets;
	u32 bank, offs;

	bank = args->args[0];
	offs = args->args[1];

	nr_resets = reset_banks[bank].nr_resets;

	if (offs > nr_resets)
		return -EINVAL;

	return NX_NUM_TO_ID(bank, offs);
}

static struct reset_ops nexell_reset_ops = {
	.of_xlate	= nexell_reset_xlate,
	.request	= nexell_reset_request,
	.free		= nexell_reset_free,
	.rst_assert	= nexell_reset_assert,
	.rst_deassert	= nexell_reset_deassert,
};

static int nexell_reset_probe(struct udevice *dev)
{
	int i;
	int nr_banks = ARRAY_SIZE(reset_banks);

	for (i = 0; i < nr_banks; i++) {
		struct nexell_reset_bank *bank = &reset_banks[i];
		fdt_addr_t addr;
		fdt_size_t size;

		addr = fdtdec_get_addr_size_auto_noparent(gd->fdt_blob,
							  dev->of_offset, "reg",
							  i, &size);
		bank->base = map_sysmem(addr, size);
	}

	/* initialize default state */

	return 0;
}

static const struct udevice_id nexell_reset_match[] = {
	{ .compatible = "nexell,nxp5540-reset", },
	{ }
};

U_BOOT_DRIVER(nexell_reset) = {
	.name = "nexell-reset",
	.id = UCLASS_RESET,
	.of_match = nexell_reset_match,
	.probe = nexell_reset_probe,
	.ops = &nexell_reset_ops,
};
