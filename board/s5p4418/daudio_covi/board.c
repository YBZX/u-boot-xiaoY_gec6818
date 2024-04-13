/*
 * (C) Copyright 2017 FMS
 * Han Yibeom <yibhan@cafeel.co.kr>
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

enum gpio_group {
	gpio_a,
	gpio_b,
	gpio_c,
	gpio_d,
	gpio_e,
};

int board_init(void)
{
/* set pwm0 output off: 1 */
#if !defined(CONFIG_TARGET_S5P4418_DAUDIO_COVI)
	nx_gpio_set_pad_function(gpio_d, 1, 0);
	nx_gpio_set_output_value(gpio_d, 1, 1);
	nx_gpio_set_output_enable(gpio_d, 1, 1);
#endif

#ifdef CONFIG_SILENT_CONSOLE
	gd->flags |= GD_FLG_SILENT;
#endif
#if defined(CONFIG_TARGET_S5P4418_DAUDIO_COVI)
	daudio_covi_gpio_default_setting();
#endif

	return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
#ifdef CONFIG_SILENT_CONSOLE
	gd->flags &= ~GD_FLG_SILENT;
#endif

/* set lcd enable */
#if !defined(CONFIG_TARGET_S5P4418_DAUDIO_COVI)
	nx_gpio_set_pad_function(gpio_c, 11, 1);
	nx_gpio_set_pad_function(gpio_b, 25, 1);
	nx_gpio_set_pad_function(gpio_b, 27, 1);

	nx_gpio_set_output_value(gpio_c, 11, 1);
	nx_gpio_set_output_value(gpio_b, 25, 1);
	nx_gpio_set_output_value(gpio_b, 27, 1);

	nx_gpio_set_output_enable(gpio_c, 11, 1);
	nx_gpio_set_output_enable(gpio_b, 25, 1);
	nx_gpio_set_output_enable(gpio_b, 27, 1);
#endif

#ifdef CONFIG_RECOVERY_BOOT
#define ALIVE_SCRATCH1_READ_REGISTER    (0xc00108b4)
#define ALIVE_SCRATCH1_RESET_REGISTER   (0xc00108ac)
#define RECOVERY_SIGNATURE              (0x52455343)    /* (ASCII) : R.E.S.C */
    printf("signature --> 0x%x\n", readl(ALIVE_SCRATCH1_READ_REGISTER));
    if (readl(ALIVE_SCRATCH1_READ_REGISTER) == RECOVERY_SIGNATURE) {
        printf("reboot recovery!!!!\n");
        writel(0xffffffff, ALIVE_SCRATCH1_RESET_REGISTER);
        setenv("bootcmd", "run recoveryboot");
    }
#endif

	return 0;
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
    {
    .name = "mmc",
    .storage = SPLASH_STORAGE_MMC,
    .flags = SPLASH_STORAGE_RAW,
    .offset = CONFIG_SPLASH_MMC_OFFSET,
    },
};

int splash_screen_prepare(void)
{
	int err = splash_source_load(splash_locations,
				  sizeof(splash_locations)/sizeof(struct splash_location));

    if (!err) {
        char addr[64];

        sprintf(addr, "0x%lx", gd->fb_base);
        setenv("fb_addr", addr);
    }

	return err;
}
#endif

/* u-boot dram initialize  */
int dram_init(void)
{
	gd->ram_size = CONFIG_SYS_SDRAM_SIZE;
	return 0;
}

/* u-boot dram board specific */
void dram_init_banksize(void)
{
	/* set global data memory */
	gd->bd->bi_arch_number = machine_arch_type;
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x00000100;

	gd->bd->bi_dram[0].start = CONFIG_SYS_SDRAM_BASE;
	gd->bd->bi_dram[0].size = CONFIG_SYS_SDRAM_SIZE;
}

void daudio_covi_gpio_default_setting(void)
{
	nx_gpio_set_pad_function(gpio_e, 18, 0);
	nx_gpio_set_output_value(gpio_e, 18, 1);
	nx_gpio_set_output_enable(gpio_e, 18, 1);
	nx_gpio_set_pad_function(gpio_d, 13, 0);
	nx_gpio_set_output_value(gpio_d, 13, 1);
	nx_gpio_set_output_enable(gpio_d, 13, 1);
	nx_gpio_set_pad_function(gpio_e, 12, 0);
	nx_gpio_set_output_value(gpio_e, 12, 1);
	nx_gpio_set_output_enable(gpio_e, 12, 1);
	nx_gpio_set_pad_function(gpio_e, 16, 0);
	nx_gpio_set_output_value(gpio_e, 16, 1);
	nx_gpio_set_output_enable(gpio_e, 16, 1);
	nx_gpio_set_pad_function(gpio_e, 17, 0);
	nx_gpio_set_output_value(gpio_e, 17, 0);
	nx_gpio_set_output_enable(gpio_e, 17, 1);
	nx_gpio_set_pad_function(gpio_e, 11, 0);
	nx_gpio_set_output_value(gpio_e, 11, 0);
	nx_gpio_set_output_enable(gpio_e, 11, 1);
	nx_gpio_set_pad_function(gpio_a, 17, 0);
	nx_gpio_set_output_value(gpio_a, 17, 1);
	nx_gpio_set_output_enable(gpio_a, 17, 1);
	nx_gpio_set_pad_function(gpio_a, 18, 0);
	nx_gpio_set_output_value(gpio_a, 18, 1);
	nx_gpio_set_output_enable(gpio_a, 18, 1);
	nx_gpio_set_pad_function(gpio_a, 19, 0);
	nx_gpio_set_output_value(gpio_a, 19, 0);
	nx_gpio_set_output_enable(gpio_a, 19, 1);
}
