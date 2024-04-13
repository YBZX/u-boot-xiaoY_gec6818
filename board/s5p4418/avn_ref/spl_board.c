/*
 * (C) Copyright 2016 Nexell
 * Hyunseok, Jung <hsjung@nexell.co.kr>
 *
 * SPDX-License-Identifier:      GPL-2.0+
 */

#include <config.h>
#include <common.h>
#include <asm/io.h>

#include <asm/arch/nexell.h>
#include <asm/arch/clk.h>
#include <asm/arch/reset.h>
#include <asm/arch/nx_gpio.h>

#include <usb/dwc2_udc.h>

DECLARE_GLOBAL_DATA_PTR;

#if !defined CONFIG_OF_CONTROL && defined CONFIG_USB_GADGET
static struct dwc2_plat_otg_data
		nx_otg_data = CONFIG_USB_GADGET_REGS;

static void board_udc_probe(void)
{
	dwc2_udc_probe(&nx_otg_data);
}
#endif

#ifdef CONFIG_VIDEO
#include <asm/arch/display.h>

#define DP_MODULE		0
#define DP_DEVICE_TYPE		DP_DEVICE_LVDS

static struct dp_lvds_dev dp_dev = {
	.lvds_format = 0,	/* 0: VESA, 1: JEIDA */
};

static struct nx_display_platdata dp_platdata = {
	.module = DP_MODULE,
	.sync  = {
		.pixel_clock_hz = 50000000,
		.h_active_len = 1024,
		.v_active_len = 600,
		.h_front_porch = 160,
		.h_back_porch = 140,
		.h_sync_width = 20,
		.v_front_porch = 12,
		.v_back_porch = 20,
		.v_sync_width = 3,
	},

	.ctrl  =  {
		.clk_src_lv0 = 0,
		.clk_div_lv0 = 16,
		.clk_src_lv1 = 7,
		.clk_div_lv1 = 1,
		.out_format = 2,
	},

	.top = {
		.screen_width = 1024,
		.screen_height = 600,
		.video_prior = 0,
		.back_color = 0x0,
	},

	.plane = {
		[0] = {
		.fb_base = 0x50000000,
		.width = 1024,
		.height = 600,
		.format = 0x06530000, /* 565: 0x44320000, A888: 0x06530000 */
		.pixel_byte = 4,
		},
	},

	.dev_type = DP_DEVICE_TYPE,
	.device = &dp_dev,
};
#endif

enum gpio_group {
	gpio_a,	gpio_b, gpio_c, gpio_d, gpio_e,
};

#ifdef CONFIG_VIDEO
void board_display_probe(void)
{
	nx_display_probe(&dp_platdata);
}
#endif

#ifdef CONFIG_SPLASH_SOURCE
#include <splash.h>

struct splash_location splash_locations[] = {
        {
                .name = "mmc_fs",
                .storage = SPLASH_STORAGE_MMC,
                .flags = SPLASH_STORAGE_FS,
                .devpart = "0:1",
        },
};

int splash_screen_prepare(void)
{
        return splash_source_load(splash_locations,
                                  ARRAY_SIZE(splash_locations));
}
#endif

int spl_display_probe(void)
{
	/* set pwm0 output off: 1 */
	nx_gpio_set_pad_function(gpio_d,  1, 0);
	nx_gpio_set_output_value(gpio_d,  1, 1);
	nx_gpio_set_output_enable(gpio_d,  1, 1);

#ifdef CONFIG_VIDEO
	board_display_probe();
#endif
	return 0;
}

int spl_late_init(void)
{
	/* set lcd enable */
	nx_gpio_set_pad_function(gpio_b, 25, 1);
	nx_gpio_set_pad_function(gpio_b, 27, 1);
	nx_gpio_set_pad_function(gpio_c, 11, 1);

	nx_gpio_set_output_value(gpio_b, 25, 1);
	nx_gpio_set_output_value(gpio_b, 27, 1);
	nx_gpio_set_output_value(gpio_c, 11, 1);

	nx_gpio_set_output_enable(gpio_b, 25, 1);
	nx_gpio_set_output_enable(gpio_b, 27, 1);
	nx_gpio_set_output_enable(gpio_c, 11, 1);

	return 0;
}

