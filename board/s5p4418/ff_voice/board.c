/*
 * (C) Copyright 2016 Nexell
 * Deokjin, Lee <truevirtue@nexell.co.kr>
 *
 * SPDX-License-Identifier:      GPL-2.0+
 */

#include <config.h>
#include <common.h>
#ifdef CONFIG_PWM_NX
#include <pwm.h>
#endif
#include <asm/io.h>

#ifdef CONFIG_DM_PMIC_NXE2000
#include <dm.h>
#include <dm/uclass-internal.h>
#include <power/pmic.h>
#include <power/nxe2000.h>
#endif

#include <asm/arch/nexell.h>
#include <asm/arch/nx_gpio.h>

DECLARE_GLOBAL_DATA_PTR;

/*------------------------------------------------------------------------------
 * intialize nexell soc and board status.
 */

/* call from u-boot */
int board_early_init_f(void)
{
	return 0;
}


void board_gpio_init(void)
{
	nx_gpio_initialize();
	nx_gpio_set_base_address(0, (void *)PHY_BASEADDR_GPIOA);
	nx_gpio_set_base_address(1, (void *)PHY_BASEADDR_GPIOB);
	nx_gpio_set_base_address(2, (void *)PHY_BASEADDR_GPIOC);
	nx_gpio_set_base_address(3, (void *)PHY_BASEADDR_GPIOD);
	nx_gpio_set_base_address(4, (void *)PHY_BASEADDR_GPIOE);
}

int mmc_get_env_dev(void)
{
	return 0;
}

int board_init(void)
{
	board_gpio_init();
	return 0;
}

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
	gd->bd->bi_dram[0].size  = CONFIG_SYS_SDRAM_SIZE;
}

#ifdef CONFIG_DM_PMIC_NXE2000
void power_init_board(void)
{
    struct udevice *pmic;
#ifdef CONFIG_DM_REGULATOR_NXE2000
    struct dm_regulator_uclass_platdata *uc_pdata;
    struct udevice *dev;
    struct udevice *regulator;
#endif
    int ret = -ENODEV;

    ret = pmic_get("nxe2000_gpio@32", &pmic);
    if (ret)
        printf("Can't get PMIC: %s!\n", "nxe2000_gpio@32");

#ifdef CONFIG_DM_REGULATOR_NXE2000
    if (device_has_children(pmic)) {
        for (ret = uclass_find_first_device(UCLASS_REGULATOR, &dev);
            dev;
            ret = uclass_find_next_device(&dev)) {
            if (ret)
                continue;

            uc_pdata = dev_get_uclass_platdata(dev);
            if (!uc_pdata)
                continue;

            uclass_get_device_tail(dev, 0, &regulator);
        }
    }
#endif
}
#endif


static void nxcbl_pwm_init(int channel)
{
	pwm_init(channel, 0, 0);
	pwm_config(channel, TO_DUTY_NS(50, 8000000), TO_PERIOD_NS(8000000));
	pwm_enable(channel);
}

static void nxcbl_reset(void)
{
	u32 grp = 4 , bit = 15, pad = 0;

	nx_gpio_set_pad_function(grp, bit, pad);
	nx_gpio_set_output_enable(grp, bit, 1);

	nx_gpio_set_output_value(grp, bit, 0);

	mdelay(100);

	nx_gpio_set_output_value(grp, bit, 1);

	mdelay(100);
}

static void nxcbl_bootmode(int mode)
{
	u32 grp = 2, bit = 15, pad = 1;

	nx_gpio_set_pad_function(grp, bit, pad);
	nx_gpio_set_output_enable(grp, bit, 1);

	nx_gpio_set_output_value(grp, bit, mode);
}
static void uart2_pad_init(void)
{
	nx_gpio_set_pad_function(3, 16, 1);
	nx_gpio_set_output_enable(3, 16, 0);
	nx_gpio_set_pad_function(3, 20, 1);
	nx_gpio_set_output_enable(3, 20, 0);
}

int board_late_init(void)
{
	uart2_pad_init();
	nxcbl_pwm_init(1);
	nxcbl_bootmode(0);
	nxcbl_reset();

	return 0;
}

