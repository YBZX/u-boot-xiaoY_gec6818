/*
 * Copyright (C) Guangzhou FriendlyARM Computer Tech. Co., Ltd.
 * (http://www.friendlyarm.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 */

#include <config.h>
#include <common.h>
#include <memalign.h>
#ifdef CONFIG_PWM_NX
#include <pwm.h>
#endif
#include <i2c.h>
#include <asm/io.h>

#include <asm/arch/nexell.h>
#include <asm/arch/nx_gpio.h>
#include <asm/arch/display.h>
#include <asm/arch/display_dev.h>

#include <u-boot/md5.h>

#include "hwrev.h"
#include "onewire.h"
#include "nxp-fb.h"

DECLARE_GLOBAL_DATA_PTR;

enum gpio_group {
	gpio_a, gpio_b, gpio_c, gpio_d, gpio_e,
};

/* copy from android bootloader_message.h */
struct bootloader_message {
	char command[32];
	char status[32];
	char recovery[768];

	char stage[32];
	char slot_suffix[32];
	char reserved[192];
};

#ifdef CONFIG_PWM_NX
struct pwm_device {
	int grp;
	int bit;
	int io_fn;
};

static inline void bd_pwm_config_gpio(int ch)
{
	struct pwm_device pwm_dev[] = {
		[0] = { .grp = gpio_d, .bit = 1,  .io_fn = 0 },
		[1] = { .grp = gpio_c, .bit = 13, .io_fn = 1 },
		[2] = { .grp = gpio_c, .bit = 14, .io_fn = 1 },
		[3] = { .grp = gpio_d, .bit = 0,  .io_fn = 0 },
	};

	int gp = pwm_dev[ch].grp;
	int io = pwm_dev[ch].bit;

	/* pwm backlight OFF: HIGH, ON: LOW */
	nx_gpio_set_pad_function(gp, io, pwm_dev[ch].io_fn);
	nx_gpio_set_output_value(gp, io, 1);
	nx_gpio_set_output_enable(gp, io, 1);
}
#endif

static void bd_backlight_off(void)
{
#ifdef CONFIG_ONEWIRE
	onewire_set_backlight(0);

#elif defined(CONFIG_BACKLIGHT_CH)
	bd_pwm_config_gpio(CONFIG_BACKLIGHT_CH);
#endif
}

static void bd_backlight_on(void)
{
#ifdef CONFIG_ONEWIRE
	onewire_set_backlight(127);

#elif defined(CONFIG_BACKLIGHT_CH)
	/* pwm backlight ON: HIGH, ON: LOW */
	pwm_init(CONFIG_BACKLIGHT_CH,
		CONFIG_BACKLIGHT_DIV, CONFIG_BACKLIGHT_INV);
	pwm_config(CONFIG_BACKLIGHT_CH,
		TO_DUTY_NS(CONFIG_BACKLIGHT_DUTY, CONFIG_BACKLIGHT_HZ),
		TO_PERIOD_NS(CONFIG_BACKLIGHT_HZ));
#endif
}

static void bd_lcd_config_gpio(void)
{
	int i;

	for (i = 0; i < 28; i++) {
		nx_gpio_set_pad_function(gpio_a, i, 1);
		nx_gpio_set_drive_strength(gpio_a, i, 0);
		nx_gpio_set_pull_mode(gpio_a, i, 2);
	}

	nx_gpio_set_drive_strength(gpio_a, 0, 1);
}

/* DEFAULT mmc dev for eMMC boot (dwmmc.2) */
static int mmc_boot_dev = 0;

int board_mmc_bootdev(void)
{
	return mmc_boot_dev;
}

/* call from common/env_mmc.c */
int mmc_get_env_dev(void)
{
	return mmc_boot_dev;
}

#ifdef CONFIG_DISPLAY_BOARDINFO
int checkboard(void)
{
	printf("Board: %s\n", get_board_name());

	return 0;
}
#endif

int nx_display_fixup_dp(struct nx_display_dev *dp)
{
	char *lcdtype = getenv("lcdtype");
	struct nxp_lcd *lcd = bd_get_lcd();
	struct nxp_lcd_timing *timing;
	enum lcd_format fmt;
	struct dp_sync_info *sync = &dp->sync;
	struct dp_plane_info *plane = &dp->planes[0];
	int i;
	u32 clk = 800000000;
	u32 div;

	if (lcdtype) {
		/* Setup again as user specified LCD in env */
		bd_setup_lcd_by_name(lcdtype);

		lcd = bd_get_lcd();
		if (lcd->gpio_init)
			lcd->gpio_init();
	}

	timing = &lcd->timing;
	fmt = bd_get_lcd_format();

	sync->h_active_len = lcd->width;
	sync->h_sync_width = timing->h_sw;
	sync->h_back_porch = timing->h_bp;
	sync->h_front_porch = timing->h_fp;
	sync->h_sync_invert = !lcd->polarity.inv_hsync;

	sync->v_active_len = lcd->height;
	sync->v_sync_width = timing->v_sw;
	sync->v_back_porch = timing->v_bp;
	sync->v_front_porch = timing->v_fp;
	sync->v_sync_invert = !lcd->polarity.inv_vsync;

	/* calculates pixel clock */
	div  = timing->h_sw + timing->h_bp + timing->h_fp + lcd->width;
	div *= timing->v_sw + timing->v_bp + timing->v_fp + lcd->height;
	div *= lcd->freq ? : 60;
	clk /= div;

	dp->ctrl.clk_div_lv0 = clk;

	if (lcd->dpc_format > 0)
		dp->ctrl.out_format = lcd->dpc_format;

	dp->top.screen_width = lcd->width;
	dp->top.screen_height = lcd->height;

	for (i = 0; i < dp->top.plane_num; i++, plane++) {
		if (plane->enable) {
			plane->width = lcd->width;
			plane->height = lcd->height;
		}
	}

	/* initialize display device type */
	if (fmt == LCD_RGB) {
		dp->dev_type = DP_DEVICE_RGBLCD;

	} else if (fmt == LCD_HDMI) {
		struct dp_hdmi_dev *dev = (struct dp_hdmi_dev *) dp->device;

		dp->dev_type = DP_DEVICE_HDMI;
		if (lcd->width == 1920 && lcd->height == 1080)
			dev->preset = 1;
		else
			dev->preset = 0;

	} else {
		struct dp_lvds_dev *dev = (struct dp_lvds_dev *) dp->device;

		dp->dev_type = DP_DEVICE_LVDS;
		dev->lvds_format = (fmt & 0x3);
		dev->voltage_level = 0x11f;
	}

	return 0;
}

/* --------------------------------------------------------------------------
 * intialize board status.
 */

#define	MMC_BOOT_CH0		(0)
#define	MMC_BOOT_CH1		(1 <<  3)
#define	MMC_BOOT_CH2		(1 << 19)

static void bd_bootdev_init(void)
{
	unsigned int rst = readl(PHY_BASEADDR_CLKPWR + SYSRSTCONFIG);

	rst &= (1 << 19) | (1 << 3);
	if (rst == MMC_BOOT_CH0) {
		/* mmc dev 1 for SD boot */
		mmc_boot_dev = 1;
	}
}

static void bd_onewire_init(void)
{
	unsigned char lcd;
	unsigned short fw_ver;

	onewire_init();
	onewire_get_info(&lcd, &fw_ver);
}

static void bd_lcd_init(void)
{
	struct nxp_lcd *cfg;
	int id;
	int ret;

	id = onewire_get_lcd_id();
	/* -1: onwire probe failed
	 *  0: bad
	 * >0: identified */

	ret = bd_setup_lcd_by_id(id);
	if (id <= 0 || ret != id) {
		printf("Panel: N/A (%d)\n", id);
		bd_setup_lcd_by_name("HDMI720P60");

	} else {
		printf("Panel: %s\n", bd_get_lcd_name());

		cfg = bd_get_lcd();
		if (cfg->gpio_init)
			cfg->gpio_init();
	}
}

static int mac_read_from_generic_eeprom(u8 *addr)
{
	struct udevice *i2c_dev;
	int ret;

	/* Microchip 24AA02xxx EEPROMs with EUI-48 Node Identity */
	ret = i2c_get_chip_for_busnum(0, 0x51, 1, &i2c_dev);
	if (!ret)
		ret = dm_i2c_read(i2c_dev, 0xfa, addr, 6);

	return ret;
}

static void make_ether_addr(u8 *addr)
{
	u32 hash[20];

#define ETHER_MAC_TAG  "ethmac"
	memset(hash, 0, sizeof(hash));
	memcpy(hash + 12, ETHER_MAC_TAG, sizeof(ETHER_MAC_TAG));

	hash[4] = readl(PHY_BASEADDR_ECID + 0x00);
	hash[5] = readl(PHY_BASEADDR_ECID + 0x04);
	hash[6] = readl(PHY_BASEADDR_ECID + 0x08);
	hash[7] = readl(PHY_BASEADDR_ECID + 0x0c);

	md5((unsigned char *)&hash[4], 64, (unsigned char *)hash);

	hash[0] ^= hash[2];
	hash[1] ^= hash[3];

	memcpy(addr, (char *)hash, 6);
	addr[0] &= 0xfe;	/* clear multicast bit */
	addr[0] |= 0x02;
}

static void set_ether_addr(void)
{
	unsigned char mac[6];
	char ethaddr[20];
	int ret;

	ret = mac_read_from_generic_eeprom(mac);
	if (ret < 0) {
		if (getenv("ethaddr"))
			return;

		make_ether_addr(mac);
	}

	sprintf(ethaddr, "%02x:%02x:%02x:%02x:%02x:%02x",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	if (!ret)
		printf("MAC:  [%s]\n", ethaddr);

	setenv("ethaddr", ethaddr);
}

#ifdef CONFIG_REVISION_TAG
static void set_board_rev(void)
{
	char info[64] = {0, };

	snprintf(info, ARRAY_SIZE(info), "%02x", get_board_rev());
	setenv("board_rev", info);
}
#endif

static void set_dtb_name(void)
{
	char info[64] = {0, };

	snprintf(info, ARRAY_SIZE(info),
			"s5p6818-nanopi3-rev%02x.dtb", get_board_rev());
	setenv("dtb_name", info);
}

static void bd_update_env(void)
{
	char *lcddpi = getenv("lcddpi");
	char *bootargs = getenv("bootargs");
	const char *name;
	char *p = NULL;
	int rootdev = board_mmc_bootdev();
	int need_save = 0;

#define CMDLINE_LCD		" lcd="
	char cmdline[CONFIG_SYS_CBSIZE];
	int n = 1;

	if (rootdev != CONFIG_ROOT_DEV && !getenv("firstboot")) {
		setenv_ulong("rootdev", rootdev);
		setenv("firstboot", "0");
		need_save = 1;
	}

	name = bd_get_lcd_name();

	if (bootargs)
		n = strlen(bootargs);	/* isn't 0 for NULL */
	else
		cmdline[0] = '\0';

	if ((n + strlen(name) + sizeof(CMDLINE_LCD)) > sizeof(cmdline)) {
		printf("Error: `bootargs' is too large (%d)\n", n);
		goto __exit;
	}

	if (bootargs) {
		p = strstr(bootargs, CMDLINE_LCD);
		if (p) {
			n = (p - bootargs);
			p += strlen(CMDLINE_LCD);
		}
		strncpy(cmdline, bootargs, n);
	}

	/* add `lcd=NAME,NUMdpi' */
	strncpy(cmdline + n, CMDLINE_LCD, strlen(CMDLINE_LCD));
	n += strlen(CMDLINE_LCD);

	strcpy(cmdline + n, name);
	n += strlen(name);

	if (lcddpi) {
		n += sprintf(cmdline + n, ",%sdpi", lcddpi);
	} else {
		int dpi = bd_get_lcd_density();

		if (dpi > 0 && dpi < 600) {
			n += sprintf(cmdline + n, ",%ddpi", dpi);
		}
	}

	/* copy remaining of bootargs */
	if (p) {
		p = strstr(p, " ");
		if (p) {
			strcpy(cmdline + n, p);
			n += strlen(p);
		}
	}

	/* append `bootdev=2' */
#define CMDLINE_BDEV	" bootdev="
	if (rootdev > 0 && !strstr(cmdline, CMDLINE_BDEV)) {
		n += sprintf(cmdline + n, "%s2", CMDLINE_BDEV);
	}

	/* finally, let's update uboot env & save it */
	if (bootargs && strncmp(cmdline, bootargs, sizeof(cmdline))) {
		setenv("bootargs", cmdline);
		need_save = 1;
	}

__exit:
	if (need_save)
		saveenv();
}

static int bd_set_recovery_wipe_data(void)
{
	char devpart[64] = { 0, };
	block_dev_desc_t *desc;
	disk_partition_t info;
	int rootdev, miscpart, blkcnt, ret;
	struct bootloader_message *bmsg;
	ALLOC_CACHE_ALIGN_BUFFER(u8, buf,
			DIV_ROUND_UP(sizeof(struct bootloader_message),
				1024) * 1024);
	const char *bootargs = getenv("bootargs");

	if (!strstr(bootargs, "androidboot."))
		return -1;

	rootdev = getenv_ulong("rootdev", 0, CONFIG_ROOT_DEV);
	miscpart = getenv_ulong("miscpart", 0, 6);
	snprintf(devpart, ARRAY_SIZE(devpart), "%d:%d", rootdev, miscpart);

	ret = get_device_and_partition("mmc", devpart, &desc, &info, 0);

	/* Android misc partition should be 4MB */
	if (ret < 0 || info.size > 8192)
		return -1;

	blkcnt = DIV_ROUND_UP(sizeof(struct bootloader_message), 1024) * 2;
	ret = desc->block_read(rootdev, info.start, blkcnt, buf);
	if (ret != blkcnt)
		return -1;

	bmsg = (struct bootloader_message *)buf;
	if (strlen(bmsg->command) > 0)
		return 0;

	strcpy(bmsg->command, "boot-recovery");
	bmsg->status[0] = 0;
	strcpy(bmsg->recovery, "recovery\n--wipe_data");

	ret = desc->block_write(rootdev, info.start, blkcnt, buf);
	if (ret != blkcnt) {
		printf("Error setting bootloader message\n");
		return -1;
	}

	return 1;
}

static void bd_check_recovery_key(void)
{
	int alive_0, pin_status;
	int i;

	if (getenv_yesno("recovery_check") != 1)
		return;

#define SCR_ALIVEGPIOINPUTVALUE	(SCR_ALIVE_BASE + 0x11C)
	alive_0 = readl(SCR_ALIVEGPIOINPUTVALUE) & 1;

	/* GPIOB27 (hp-det) as input */
	nx_gpio_set_pad_function(gpio_b, 27, 1);
	nx_gpio_set_output_enable(gpio_b, 27, 0);

	pin_status = nx_gpio_get_input_value(gpio_b, 27);
	if (alive_0 || !pin_status)
		return;

	printf("checking recovery key...");

	/* GPIOB12 (status_led) as output */
	nx_gpio_set_pad_function(gpio_b, 12, 2);
	nx_gpio_set_output_enable(gpio_b, 12, 1);

	/* detecting falling edge */
	nx_gpio_set_detect_mode(gpio_b, 27, 0x2);
	nx_gpio_set_detect_enable(gpio_b, 27, 1);

	for (i = 0; i < 2500; i++) {
		alive_0 = readl(SCR_ALIVEGPIOINPUTVALUE) & 1;
		if (alive_0)
			break;

		mdelay(1);
		if (i == 500)
			nx_gpio_set_output_value(gpio_b, 12, 1);
	}

	nx_gpio_set_output_value(gpio_b, 12, 0);
	nx_gpio_set_detect_enable(gpio_b, 27, 0);
	pin_status = nx_gpio_get_detect_status(gpio_b, 27, 1);

	/* power key pressed 2.5s and gpio event detected */
	if (i >= 2500 && pin_status) {
		printf("\nenter recovery mode (wipe_data)\n");
		onewire_set_backlight(80);
		bd_set_recovery_wipe_data();
		run_command("setenv initrd_name ramdisk-recovery.img; boot", 0);
	}

	printf("none\n");
	return;
}

static void bd_check_reset(void)
{
	u32 reason;

#define SCR_USER_SIG1_READ		(SCR_ALIVE_BASE + 0x0B4)
#define SCR_USER_SIG1_RESET		(SCR_ALIVE_BASE + 0x0AC)
#define RECOVERY_SIGNATURE		(0x52455343)  /* (ASCII) : R.E.S.C */
#define FASTBOOT_SIGNATURE		(0x46535442)  /* (ASCII) : F.S.T.B */

	reason = readl(SCR_USER_SIG1_READ);
	debug("signature --> 0x%x\n", reason);

	if (reason == RECOVERY_SIGNATURE) {
		printf("enter recovery mode\n");
		writel(0xffffffff, SCR_USER_SIG1_RESET);
		run_command("setenv initrd_name ramdisk-recovery.img; boot", 0);
	} else if (reason == FASTBOOT_SIGNATURE) {
		printf("enter fastboot mode\n");
		writel(0xffffffff, SCR_USER_SIG1_RESET);
		run_command("fastboot 0", 0);
	}
}

/* --------------------------------------------------------------------------
 * call from u-boot
 */

int board_early_init_f(void)
{
	return 0;
}

int board_init(void)
{
	bd_hwrev_init();
	bd_bootdev_init();
	bd_onewire_init();

	bd_backlight_off();

	bd_lcd_config_gpio();
	bd_lcd_init();

#ifdef CONFIG_SILENT_CONSOLE
	gd->flags |= GD_FLG_SILENT;
#endif

	return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
	bd_update_env();

#ifdef CONFIG_REVISION_TAG
	set_board_rev();
#endif
	set_dtb_name();

	set_ether_addr();

#ifdef CONFIG_SILENT_CONSOLE
	gd->flags &= ~GD_FLG_SILENT;
#endif

	bd_backlight_on();
	printf("\n");

	bd_check_reset();
	bd_check_recovery_key();

	return 0;
}
#endif

#ifdef CONFIG_SPLASH_SOURCE
#include <splash.h>
static struct splash_location splash_locations[] = {
	{
	.name = "mmc_fs",
	.storage = SPLASH_STORAGE_MMC,
	.flags = SPLASH_STORAGE_FS,
	.devpart = __stringify(CONFIG_ROOT_DEV) ":" __stringify(CONFIG_BOOT_PART),
	},
};

int splash_screen_prepare(void)
{
	int err;
	char *env_cmd = getenv("load_splash");

	if (env_cmd) {
		err = run_command(env_cmd, 0);

	} else {
		char devpart[64] = { 0, };
		int bootpart = getenv_ulong("bootpart", 0, CONFIG_BOOT_PART);
		int rootdev;

		if (getenv("firstboot"))
			rootdev = getenv_ulong("rootdev", 0, CONFIG_ROOT_DEV);
		else
			rootdev = board_mmc_bootdev();

		snprintf(devpart, ARRAY_SIZE(devpart), "%d:%d", rootdev, bootpart);
		splash_locations[0].devpart = devpart;

		err = splash_source_load(splash_locations,
				ARRAY_SIZE(splash_locations));
	}

	if (!err) {
		char addr[64];

		sprintf(addr, "0x%lx", gd->fb_base);
		setenv("fb_addr", addr);
	}

	return err;
}
#endif

/* u-boot dram initialize */
int dram_init(void)
{
	gd->ram_size = CONFIG_SYS_SDRAM_SIZE;
	return 0;
}

/* u-boot dram board specific */
void dram_init_banksize(void)
{
#define SCR_USER_SIG6_READ		(SCR_ALIVE_BASE + 0x0F0)
	int g_NR_chip = readl(SCR_USER_SIG6_READ) & 0x3;

	/* set global data memory */
	gd->bd->bi_arch_number = machine_arch_type;
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x00000100;

	gd->bd->bi_dram[0].start = CONFIG_SYS_SDRAM_BASE;
	gd->bd->bi_dram[0].size  = CONFIG_SYS_SDRAM_SIZE;

	if (g_NR_chip > 1) {
		gd->bd->bi_dram[1].start = 0x80000000;
		gd->bd->bi_dram[1].size  = 0x40000000;
	}
}

#if defined(CONFIG_OF_BOARD_SETUP)
int ft_board_setup(void *blob, bd_t *bd)
{
	int nodeoff;
	unsigned int rootdev;
	unsigned int fb_addr;

	if (board_mmc_bootdev() > 0) {
		rootdev = fdt_getprop_u32_default(blob, "/board", "sdidx", 2);
		if (rootdev) {
			/* find or create "/chosen" node. */
			nodeoff = fdt_find_or_add_subnode(blob, 0, "chosen");
			if (nodeoff >= 0)
				fdt_setprop_u32(blob, nodeoff, "linux,rootdev", rootdev);
		}
	}

	fb_addr = getenv_ulong("fb_addr", 0, 0);
	if (fb_addr) {
		nodeoff = fdt_path_offset(blob, "/reserved-memory");
		if (nodeoff < 0)
			return nodeoff;

		nodeoff = fdt_add_subnode(blob, nodeoff, "display_reserved");
		if (nodeoff >= 0) {
			fdt32_t cells[2];

			cells[0] = cpu_to_fdt32(fb_addr);
			cells[1] = cpu_to_fdt32(0x800000);

			fdt_setprop(blob, nodeoff, "reg", cells, sizeof(cells[0]) * 2);
		}
	}

	return 0;
}
#endif
