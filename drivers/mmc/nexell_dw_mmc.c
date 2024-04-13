/*
 * (C) Copyright 2016 Nexell
 * Youngbok, Park <park@nexell.co.kr>
 *
 * SPDX-License-Identifier:      GPL-2.0+
 */

#include <common.h>
#include <malloc.h>
#include <dwmmc.h>
#include <asm/arch/nexell.h>
#include <asm/arch/clk.h>
#include <asm/arch/reset.h>
#include <asm/arch/nx_gpio.h>
#include <asm/arch/tieoff.h>
#include <asm-generic/errno.h>
#include <fdtdec.h>
#include <libfdt.h>

DECLARE_GLOBAL_DATA_PTR;

#define	DWMCI_NAME "NEXELL DWMMC"

#define DWMCI_CLKSEL			0x09C
#define DWMCI_SHIFT_0			0x0
#define DWMCI_SHIFT_1			0x1
#define DWMCI_SHIFT_2			0x2
#define DWMCI_SHIFT_3			0x3
#define DWMCI_SET_SAMPLE_CLK(x)	(x)
#define DWMCI_SET_DRV_CLK(x)	((x) << 16)
#define DWMCI_SET_DIV_RATIO(x)	((x) << 24)
#define DWMCI_CLKCTRL			0x114
#define NX_MMC_CLK_DELAY(x, y, a, b)	(((x & 0xFF) << 0) |\
					((y & 0x03) << 16) |\
					((a & 0xFF) << 8)  |\
					((b & 0x03) << 24))

struct nx_dwmci_dat {
	int dev;
	int frequency;
	int buswidth;
	int d_delay;
	int d_shift;
	int s_delay;
	int s_shift;
	int ddr;
	char name[50];
	struct clk *clk;
};

/* FIXME : This func will be remove after support pinctrl.
 * set mmc pad alternative func.
 */
#if defined(CONFIG_SPL_BUILD)
#if defined(CONFIG_ARCH_S5P4418) ||	\
	defined(CONFIG_ARCH_S5P6818)
static void dw_mmc_set_pin(u32 base, u32 bit, u32 value)
{
	register u32 data;

	base += (bit / 16) ? 0x24 : 0x20;	/* 0x24: ALTFN1, 0x20:ALTFN0 */
	bit &= 0xf;

	data = readl(base);
	data = (u32)(data & ~(3ul << (bit * 2)));
	data = (u32)(data | (value << (bit * 2)));

	writel(data, base);
}
#endif
#endif

static void set_pin_stat(int index, int bit, int value)
{
#if !defined(CONFIG_SPL_BUILD)
	nx_gpio_set_pad_function(index, bit, value);
#else
#if defined(CONFIG_ARCH_S5P4418) ||	\
	defined(CONFIG_ARCH_S5P6818)

	unsigned long base[5] = {
		PHY_BASEADDR_GPIOA, PHY_BASEADDR_GPIOB,
		PHY_BASEADDR_GPIOC, PHY_BASEADDR_GPIOD,
		PHY_BASEADDR_GPIOE,
	};

	dw_mmc_set_pin(base[index], bit, value);
#endif
#endif
}

static void nx_dw_mmc_set_pin(struct dwmci_host *host)
{
	switch (host->dev_index) {
	case 0:
		set_pin_stat(0, 29, 1);
		set_pin_stat(0, 31, 1);
		set_pin_stat(1, 1, 1);
		set_pin_stat(1, 3, 1);
		set_pin_stat(1, 5, 1);
		set_pin_stat(1, 7, 1);
		break;
	case 1:
		set_pin_stat(3, 22, 1);
		set_pin_stat(3, 23, 1);
		set_pin_stat(3, 24, 1);
		set_pin_stat(3, 25, 1);
		set_pin_stat(3, 26, 1);
		set_pin_stat(3, 27, 1);
		break;
	case 2:
		set_pin_stat(2, 18, 2);
		set_pin_stat(2, 19, 2);
		set_pin_stat(2, 20, 2);
		set_pin_stat(2, 21, 2);
		set_pin_stat(2, 22, 2);
		set_pin_stat(2, 23, 2);
		if (host->buswidth == 8) {
			set_pin_stat(4, 21, 2);
			set_pin_stat(4, 22, 2);
			set_pin_stat(4, 23, 2);
			set_pin_stat(4, 24, 2);
		}
		break;
	}
}

static unsigned int nx_dw_mmc_get_clk(struct dwmci_host *host, uint freq)
{
	struct clk *clk;
	struct nx_dwmci_dat *priv = host->priv;
	int index = host->dev_index;
	char name[50] = { 0, };

	clk = priv->clk;
	if (!clk) {
		sprintf(name, "%s.%d", DEV_NAME_SDHC, index);
		clk = clk_get((const char *)name);
		if (!clk)
			return 0;
		priv->clk = clk;
	}

	return clk_get_rate(clk) / 2;
}

static unsigned long nx_dw_mmc_set_clk(struct dwmci_host *host,
				       unsigned int rate)
{
	struct clk *clk;
	char name[50] = { 0, };
	struct nx_dwmci_dat *priv = host->priv;
	int index = host->dev_index;

	clk = priv->clk;
	if (!clk) {
		sprintf(name, "%s.%d", DEV_NAME_SDHC, index);
		clk = clk_get((const char *)name);
		if (!clk)
			return 0;
		priv->clk = clk;
	}

	clk_disable(clk);
	rate = clk_set_rate(clk, rate);
	clk_enable(clk);

	return rate;
}

static void nx_dw_mmc_clksel(struct dwmci_host *host)
{
	u32 val;

#ifdef CONFIG_BOOST_MMC
	val = DWMCI_SET_SAMPLE_CLK(DWMCI_SHIFT_0) |
	    DWMCI_SET_DRV_CLK(DWMCI_SHIFT_0) | DWMCI_SET_DIV_RATIO(1);
#else
	val = DWMCI_SET_SAMPLE_CLK(DWMCI_SHIFT_0) |
	    DWMCI_SET_DRV_CLK(DWMCI_SHIFT_0) | DWMCI_SET_DIV_RATIO(3);
#endif

	dwmci_writel(host, DWMCI_CLKSEL, val);
}

static void nx_dw_mmc_reset(int ch)
{
	int rst_id = RESET_ID_SDMMC0 + ch;

	nx_rstcon_setrst(rst_id, 0);
	nx_rstcon_setrst(rst_id, 1);
}

static void nx_dw_mmc_clk_delay(struct dwmci_host *host)
{
	unsigned int delay;
	struct nx_dwmci_dat *priv = host->priv;

	delay = NX_MMC_CLK_DELAY(priv->d_delay,
				 priv->d_shift, priv->s_delay, priv->s_shift);

	writel(delay, (host->ioaddr + DWMCI_CLKCTRL));
}

#if defined(CONFIG_OF_CONTROL)
static int nx_dw_mmc_of_list(const void *blob, int *node_list, int lists)
{
	return fdtdec_find_aliases_for_id(blob, "mmc",
					  COMPAT_NEXELL_DWMMC, node_list,
					  lists);
}

static int nx_dw_mmc_of_platdata(const void *blob, int node,
				 struct dwmci_host *host)
{
	struct nx_dwmci_dat *priv;
	int fifo_size = 0x20;
	int index, bus_w;
	int ddr;
	unsigned long base;

	index = fdtdec_get_int(blob, node, "index", 0);
	bus_w = fdtdec_get_int(blob, node, "nexell,bus-width", 0);
	if (0 >= bus_w) {
		printf("failed to bus width %d for dwmmc.%d\n", bus_w, index);
		return -EINVAL;
	}

	base = fdtdec_get_uint(blob, node, "reg", 0);
	if (!base) {
		printf("failed to invalud base for dwmmc.%d\n", index);
		return -EINVAL;
	}
	ddr = fdtdec_get_int(blob, node, "nexell,ddr", 0);

	priv = malloc(sizeof(struct nx_dwmci_dat));
	if (!priv) {
		printf("failed to private malloc for dwmmc.%d\n", index);
		return -ENOMEM;
	}

	priv->d_delay = fdtdec_get_int(blob, node, "nexell,drive_dly", 0);
	priv->d_shift = fdtdec_get_int(blob, node, "nexell,drive_shift", 0);
	priv->s_delay = fdtdec_get_int(blob, node, "nexell,sample_dly", 0);
	priv->s_shift = fdtdec_get_int(blob, node, "nexell,sample_shift", 0);
	priv->frequency = fdtdec_get_int(blob, node, "frequency", 0);
	priv->buswidth = bus_w;
	priv->ddr = ddr;

	host->priv = priv;
	host->ioaddr = (void *)base;
	host->dev_index = index;
	host->buswidth = bus_w;
	host->name = DWMCI_NAME;
	host->clksel = nx_dw_mmc_clksel;
	host->dev_id = host->dev_index;
	host->get_mmc_clk = nx_dw_mmc_get_clk;
	host->fifoth_val =
	    MSIZE(0x2) | RX_WMARK(fifo_size / 2 - 1) | TX_WMARK(fifo_size / 2);

	if (ddr)
		host->caps |= MMC_MODE_DDR_52MHz;

	return 0;
}
#else

#if defined(CONFIG_ARCH_S5P4418) || defined(CONFIG_ARCH_S5P6818)
static unsigned long nx_dwmci_base[3] = {
	0xc0062000, 0xc0068000, 0xc0069000
};
#else
#error "Not support architecture for dwmmc ...."
#endif

static struct nx_dwmci_dat dwmci_platdata[] = CONFIG_NEXELL_DWMMC_PLATFORM_DATA;

static int nx_dw_mmc_of_list(const void *blob, int *node_list, int lists)
{
	int count = ARRAY_SIZE(dwmci_platdata);
	int num, i;

	if (lists > count)
		num = count;
	else
		num = lists;

	for (i = 0; num > i; i++)
		node_list[i] = i;

	return count;
}

static int nx_dw_mmc_of_platdata(const void *blob, int node,
				 struct dwmci_host *host)
{
	struct nx_dwmci_dat *priv = dwmci_platdata;
	int fifo_size = 0x20;

	host->priv = &priv[node];
	host->dev_index = priv[node].dev;
	host->ioaddr = (void *)nx_dwmci_base[host->dev_index];
	host->buswidth = priv[node].buswidth;
	host->name = DWMCI_NAME;
	host->clksel = nx_dw_mmc_clksel;
	host->dev_id = host->dev_index;
	host->get_mmc_clk = nx_dw_mmc_get_clk;
	host->fifoth_val =
	    MSIZE(0x2) | RX_WMARK(fifo_size / 2 - 1) | TX_WMARK(fifo_size / 2);

	if (priv[node].ddr)
		host->caps |= MMC_MODE_DDR_52MHz;

	debug("mmc.%d index.%d, io=0x%p, bus.%d, ddr[%s]\n",
	      node, host->dev_index, host->ioaddr, host->buswidth,
	      priv[node].ddr ? "O" : "X");
	debug("mmc.%d, freq %d, delay 0x%02x:0x%02x:0x%02x:0x%02x\n",
	      node, priv->frequency,
	      priv->d_delay, priv->d_shift, priv->s_delay, priv->s_shift);

	return 0;
}
#endif
static int nx_dw_mmc_setup(const void *blob)
{
	struct dwmci_host *host;
	struct nx_dwmci_dat *priv;
	int node_list[3] = { 0, };
	int node, count;
	int i, err;

	count = nx_dw_mmc_of_list(blob, node_list, ARRAY_SIZE(node_list));

	for (i = 0; count > i; i++) {
		debug("[%d] mmc.%d setup\n", count, i);

		node = node_list[i];
		host = malloc(sizeof(struct dwmci_host));
		if (!host) {
			printf("failed to dwmci host malloc !!!\n");
			return -ENOMEM;
		}
		memset(host, 0x00, sizeof(*host));

		err = nx_dw_mmc_of_platdata(blob, node, host);
		if (err) {
			printf("failed to decode dwmmc dev %d\n", i);
			return err;
		}

#if defined (CONFIG_ARCH_S5P6818)
		if (host->buswidth == 8)
			nx_tieoff_set(NX_TIEOFF_MMC_8BIT, 1);
#endif
		priv = host->priv;

		nx_dw_mmc_set_pin(host);
		nx_dw_mmc_set_clk(host, priv->frequency * 4);
		nx_dw_mmc_reset(host->dev_index);
		nx_dw_mmc_clk_delay(host);

		/* add to dwmci */
		add_dwmci(host, priv->frequency, 400000);
	}

	return 0;
}

int nx_dw_mmc_init(const void *blob)
{
	int err = nx_dw_mmc_setup(blob);

	if (err)
		printf("fail to add nexell dw mmc driver!!!\n");

	return err;
}

int board_mmc_init(bd_t *bis)
{
	return nx_dw_mmc_init(gd->fdt_blob);
}
