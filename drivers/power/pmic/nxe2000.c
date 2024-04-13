/*
 * nxe2000.c  --  PMIC driver for the NEXELL NXE2000
 *
 * Copyright (C) 2016  Nexell Co., Ltd.
 * Author: Jongshin, Park <pjsin865@nexell.co.kr>
 *
 * SPDX-License-Identifier:     GPL-2.0+

 */

#include <common.h>
#include <fdtdec.h>
#include <errno.h>
#include <dm.h>
#include <i2c.h>
#include <power/pmic.h>
#include <power/regulator.h>
#include <power/nxe2000.h>
#ifdef CONFIG_REVISION_TAG
#include <asm/arch/nexell.h>
#include <asm/arch/nx_gpio.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

static const struct pmic_child_info pmic_reg_info[] = {
	{ .prefix = "BUCK", .driver = NXE2000_BUCK_DRIVER },
	{ .prefix = "LDO", .driver = NXE2000_LDO_DRIVER },
	{ },
};

static const struct pmic_child_info pmic_chg_info[] = {
	{ .prefix = "CHG", .driver = NXE2000_CHG_DRIVER },
	{ },
};

#ifdef CONFIG_REVISION_TAG
static void gpio_init(void)
{
	nx_gpio_set_pad_function(4, 4, 0);
	nx_gpio_set_pad_function(4, 5, 0);
	nx_gpio_set_pad_function(4, 6, 0);
	nx_gpio_set_output_value(4, 4, 0);
	nx_gpio_set_output_value(4, 5, 0);
	nx_gpio_set_output_value(4, 6, 0);
}

static u32 hw_revision(void)
{
	u32 val = 0;

	val |= nx_gpio_get_input_value(4, 6);
	val <<= 1;

	val |= nx_gpio_get_input_value(4, 5);
	val <<= 1;

	val |= nx_gpio_get_input_value(4, 4);

	return val;
}
#endif

static void nxe2000_print_chgstate(struct udevice *dev)
{
	uint8_t value = 0;
	const char *rdstatus[] = {
		"CHG OFF",
		"Charge Ready(VADP)",
		"Trickle Charge",
		"Rapid Charge",
		"Charge Complete",
		"Suspend",
		"VCHG Over Voltage",
		"Battery Error",
		"No Battery",
		"Battery Over Voltage",
		"Battery Temp Error",
		"Die Error",
		"Die Shutdown",
		"...",
		"...",
		"...",
		"No Battery2",
		"Charge Ready(VUSB)",
	};

	dm_i2c_read(dev, NXE2000_REG_CHGSTATE, &value, 1);
	value &= 0x1F;
	if (value <= 0x11)
		printf("CHGS:  %s\n", rdstatus[value]);

	return;
}


static int nxe2000_reg_count(struct udevice *dev)
{
	return NXE2000_NUM_OF_REGS;
}

static int nxe2000_write(struct udevice *dev, uint reg, const uint8_t *buff,
			  int len)
{
	if (dm_i2c_write(dev, reg, buff, len)) {
		error("write error to device: %p register: %#x!", dev, reg);
		return -EIO;
	}

	return 0;
}

static int nxe2000_read(struct udevice *dev, uint reg, uint8_t *buff, int len)
{
	if (dm_i2c_read(dev, reg, buff, len)) {
		error("read error from device: %p register: %#x!", dev, reg);
		return -EIO;
	}

	return 0;
}

#if defined(CONFIG_PMIC_REG_DUMP)
static int nxe2000_reg_dump(struct udevice *dev, const char *title)
{
	int i;
	int ret = 0;
	uint8_t value = 0;

	printf("############################################################\n");
	printf("##\e[31m %s \e[0m\n", title);
	printf("##       0  1  2  3   4  5  6  7   8  9  A  B   C  D  E  F\n");
	for (i = 0; i <= NXE2000_NUM_OF_REGS; i++) {
		if (i%16 == 0)
			printf("##  %02X:", i);
		if (i%4 == 0)
			printf(" ");
		ret = dm_i2c_read(dev, i, &value, 1);
		if (!ret)
			printf("%02x ", value);
		else
			printf("\e[31mxx\e[0m ");
		if ((i+1)%16 == 0)
			printf("\n");
	}
	printf("############################################################\n");

	return 0;
}
#endif

static int nxe2000_param_setup(struct udevice *dev, uint8_t *cache)
{
	struct dm_nxe2000_platdata *pdata = dev->platdata;
	uint8_t value = 0x0;

	/* for Key error. */
	dm_i2c_write(dev, NXE2000_REG_ADCCNT3, &value, 1);

	/* Set GPIO4 Condition : High(Hi-Z)*/
	dm_i2c_read(dev, NXE2000_REG_IOOUT, &value, 1);
	value |= 0x10;
	dm_i2c_write(dev, NXE2000_REG_IOOUT, &value, 1);

#if defined(CONFIG_SW_UBC_DETECT)
	/* Set GPIO4 Direction : input*/
	dm_i2c_read(dev, NXE2000_REG_IOSEL, &value, 1);
	value &= ~0x10;
	dm_i2c_write(dev, NXE2000_REG_IOSEL, &value, 1);
#else
	/* Set GPIO4 Direction : output*/
	dm_i2c_read(dev, NXE2000_REG_IOSEL, &value, 1);
	value |= 0x10;
	dm_i2c_write(dev, NXE2000_REG_IOSEL, &value, 1);
#endif

	dm_i2c_read(dev, NXE2000_REG_PWRONTIMSET
		, &cache[NXE2000_REG_PWRONTIMSET], 1);
	cache[NXE2000_REG_PWRONTIMSET] &=
		~(0x7 << NXE2000_POS_PWRONTIMSET_OFF_PRESS_PWRON);
	cache[NXE2000_REG_PWRONTIMSET] |=
		(pdata->off_press_time
			<< NXE2000_POS_PWRONTIMSET_OFF_PRESS_PWRON);
	if (pdata->off_press_time != -ENODATA)
		dm_i2c_write(dev, NXE2000_REG_PWRONTIMSET
			, &cache[NXE2000_REG_PWRONTIMSET], 1);

	dm_i2c_read(dev, NXE2000_REG_FG_CTRL, &cache[NXE2000_REG_FG_CTRL], 1);
	cache[NXE2000_REG_FG_CTRL] &= (1 << NXE2000_POS_FG_CTRL_FG_ACC);
	cache[NXE2000_REG_FG_CTRL] =
		(1 << NXE2000_POS_FG_CTRL_SRST0)
		| (1 << NXE2000_POS_FG_CTRL_SRST1);
	dm_i2c_write(dev, NXE2000_REG_FG_CTRL, &cache[NXE2000_REG_FG_CTRL], 1);

	cache[NXE2000_REG_VINDAC] = (pdata->vindac << NXE2000_POS_VINDAC);

	cache[NXE2000_REG_ADCCNT1] =
		((pdata->adc_ain0 << NXE2000_POS_ADC_AIN0)
		| (pdata->adc_ain1 << NXE2000_POS_ADC_AIN1)
		| (pdata->adc_vthm << NXE2000_POS_ADC_VTHM)
		| (pdata->adc_vsys << NXE2000_POS_ADC_VSYS)
		| (pdata->adc_vusb << NXE2000_POS_ADC_VUSB)
		| (pdata->adc_vadp << NXE2000_POS_ADC_VADP)
		| (pdata->adc_vbat << NXE2000_POS_ADC_VBAT)
		| (pdata->adc_ilim << NXE2000_POS_ADC_ILIM));

	cache[NXE2000_REG_ADCCNT3] =
		((pdata->adccnt3_adrq << NXE2000_POS_ADCCNT3_ADRQ)
		| (pdata->adccnt3_ave << NXE2000_POS_ADCCNT3_AVE)
		| (pdata->adccnt3_adsel << NXE2000_POS_ADCCNT3_ADSEL));

	cache[NXE2000_REG_CHGCTL1] =
		((pdata->chgp << NXE2000_POS_CHGCTL1_CHGP)
		| (pdata->chgcmp_dis << NXE2000_POS_CHGCTL1_CHGCMP_DIS)
		| (pdata->nobatovlim << NXE2000_POS_CHGCTL1_NOBATOVLIM)
		| (pdata->otg_boost_en << NXE2000_POS_CHGCTL1_OTG_BOOST_EN)
		| (pdata->suspend << NXE2000_POS_CHGCTL1_SUSPEND)
		| (pdata->jeitaen << NXE2000_POS_CHGCTL1_JEITAEN)
		| (pdata->vusbchgen << NXE2000_POS_CHGCTL1_VUSBCHGEN)
		| (pdata->vadpchgen << NXE2000_POS_CHGCTL1_VADPCHGEN));

	cache[NXE2000_REG_CHGCTL2] =
	((pdata->chg_usb_vcontmask << NXE2000_POS_CHGCTL2_USB_VCONTMASK)
	| (pdata->chg_adp_vcontmask << NXE2000_POS_CHGCTL2_ADP_VCONTMASK)
	| (pdata->chg_vbus_buck_ths << NXE2000_POS_CHGCTL2_VUSBBUCK_VTH)
	| (pdata->chg_vadp_buck_ths << NXE2000_POS_CHGCTL2_VADPBUCK_VTH));

	cache[NXE2000_REG_VSYSSET] =
		((pdata->vsys_vol << NXE2000_POS_VSYSSET_VSYSSET)
		| (pdata->vsys_over_vol << NXE2000_POS_VSYSSET_VSYSOVSET));

	cache[NXE2000_REG_TIMSET] =
		((pdata->rapid_ttime << NXE2000_POS_TIMSET_TTIMSET)
		| ((pdata->rapid_ctime & 3) << NXE2000_POS_TIMSET_CTIMSET)
		| (pdata->rapid_rtime & 3));

	cache[NXE2000_REG_BATSET1] =
		(((pdata->pwr_on_vol & 7) << NXE2000_POS_BATSET1_CHGPON)
		| ((pdata->vbatov_set & 1) << NXE2000_POS_BATSET1_VBATOVSET)
		| ((pdata->vweak & 3) << NXE2000_POS_BATSET1_VWEAK)
		| ((pdata->vdead & 1) << NXE2000_POS_BATSET1_VDEAD)
		| ((pdata->vshort & 1) << NXE2000_POS_BATSET1_VSHORT));

	cache[NXE2000_REG_DIESET] =
		((pdata->die_return_temp << NXE2000_POS_DIESET_DIERTNTEMP)
		| (pdata->die_error_temp << NXE2000_POS_DIESET_DIEERRTEMP)
		| (pdata->die_shutdown_temp << NXE2000_POS_DIESET_DIESHUTTEMP));

	cache[NXE2000_REG_BATSET2] =
		(((pdata->vfchg & 7) << NXE2000_POS_BATSET2_VFCHG)
		| (pdata->vfchg & 7));

	cache[NXE2000_REG_FG_CTRL] =
		((1 << NXE2000_POS_FG_CTRL_FG_ACC)
		| (1 << NXE2000_POS_FG_CTRL_FG_EN));

	cache[NXE2000_REG_REPCNT] =
		((pdata->repcnt_off_reseto << NXE2000_POS_REPCNT_OFF_RESETO)
		| (pdata->repcnt_repwrtim << NXE2000_POS_REPCNT_REPWRTIM)
		| (pdata->repcnt_repwron << NXE2000_POS_REPCNT_REPWRON));

	cache[NXE2000_REG_WATCHDOG] =
		((pdata->wdog_slpen << NXE2000_POS_WDOG_SLPEN)
		| (pdata->wdog_en << NXE2000_POS_WDOG_EN)
		| (pdata->wdog_tim << NXE2000_POS_WDOG_TIM));

	cache[NXE2000_REG_PWRIREN] =
		((pdata->pwrien_wdog << NXE2000_POS_PWRIREN_WDOG)
		| (pdata->pwrien_noe_off << NXE2000_POS_PWRIREN_NOE_OFF)
		| (pdata->pwrien_pwron_off << NXE2000_POS_PWRIREN_PWRON_OFF)
		| (pdata->pwrien_preot << NXE2000_POS_PWRIREN_PREOT)
		| (pdata->pwrien_prvindt << NXE2000_POS_PWRIREN_PRVINDT)
		| (pdata->pwrien_extin << NXE2000_POS_PWRIREN_EXTIN)
		| (pdata->pwrien_pwron << NXE2000_POS_PWRIREN_PWRON));

	return 0;
}

static int nxe2000_device_setup(struct udevice *dev, uint8_t *cache)
{
	struct dm_nxe2000_platdata *pdata = dev->platdata;

	if (
		(pdata->chgp != -ENODATA) &&
		(pdata->chgcmp_dis != -ENODATA) &&
		(pdata->nobatovlim != -ENODATA) &&
		(pdata->otg_boost_en != -ENODATA) &&
		(pdata->suspend != -ENODATA) &&
		(pdata->jeitaen != -ENODATA) &&
		(pdata->vusbchgen != -ENODATA) &&
		(pdata->vadpchgen != -ENODATA))
		dm_i2c_write(dev, NXE2000_REG_CHGCTL1
			, &cache[NXE2000_REG_CHGCTL1], 1);

	if (pdata->vindac != -ENODATA)
		dm_i2c_write(dev, NXE2000_REG_VINDAC
			, &cache[NXE2000_REG_VINDAC], 1);

	/* if (
		(pdata->chg_usb_vcontmask != -ENODATA) &&
		(pdata->chg_adp_vcontmask != -ENODATA) &&
		(pdata->chg_vbus_buck_ths != -ENODATA) &&
		(pdata->chg_vadp_buck_ths != -ENODATA))
		dm_i2c_write(dev, NXE2000_REG_CHGCTL2
			, &cache[NXE2000_REG_CHGCTL2], 1);
	*/

	if (
		(pdata->vsys_vol != -ENODATA) &&
		(pdata->vsys_over_vol != -ENODATA))
		dm_i2c_write(dev, NXE2000_REG_VSYSSET
			, &cache[NXE2000_REG_VSYSSET], 1);

	if (
		(pdata->rapid_ttime != -ENODATA) &&
		(pdata->rapid_ctime != -ENODATA) &&
		(pdata->rapid_rtime != -ENODATA))
		dm_i2c_write(dev, NXE2000_REG_TIMSET
			, &cache[NXE2000_REG_TIMSET], 1);

	if (
		(pdata->pwr_on_vol != -ENODATA) &&
		(pdata->vbatov_set != -ENODATA) &&
		(pdata->vweak != -ENODATA) &&
		(pdata->vdead != -ENODATA) &&
		(pdata->vshort != -ENODATA))
		dm_i2c_write(dev, NXE2000_REG_BATSET1
			, &cache[NXE2000_REG_BATSET1], 1);

	if (pdata->vfchg != -ENODATA)
		dm_i2c_write(dev, NXE2000_REG_BATSET2
			, &cache[NXE2000_REG_BATSET2], 1);

	if (
		(pdata->die_return_temp != -ENODATA) &&
		(pdata->die_error_temp != -ENODATA) &&
		(pdata->die_shutdown_temp != -ENODATA))
		dm_i2c_write(dev, NXE2000_REG_DIESET, &cache[NXE2000_REG_DIESET]
			, 1);

	if (
		(pdata->repcnt_off_reseto != -ENODATA) &&
		(pdata->repcnt_repwrtim != -ENODATA) &&
		(pdata->repcnt_repwron != -ENODATA))
		dm_i2c_write(dev, NXE2000_REG_REPCNT, &cache[NXE2000_REG_REPCNT]
			, 1);

	if (
		(pdata->wdog_slpen != -ENODATA) &&
		(pdata->wdog_en != -ENODATA) &&
		(pdata->wdog_tim != -ENODATA))
		dm_i2c_write(dev, NXE2000_REG_WATCHDOG
			, &cache[NXE2000_REG_WATCHDOG], 1);

	if (
		(pdata->pwrien_wdog != -ENODATA) &&
		(pdata->pwrien_noe_off != -ENODATA) &&
		(pdata->pwrien_pwron_off != -ENODATA) &&
		(pdata->pwrien_preot != -ENODATA) &&
		(pdata->pwrien_prvindt != -ENODATA) &&
		(pdata->pwrien_extin != -ENODATA) &&
		(pdata->pwrien_pwron != -ENODATA))
		dm_i2c_write(dev, NXE2000_REG_PWRIREN
			, &cache[NXE2000_REG_PWRIREN], 1);

	return 0;
}

static int nxe2000_probe(struct udevice *dev)
{
	uint8_t value = 0x0;
	uint8_t cache[256] = {0x0,};

	dm_i2c_write(dev, NXE2000_REG_BANKSEL, &value, 1);

#if defined(CONFIG_PMIC_REG_DUMP)
	nxe2000_reg_dump(dev, "PMIC Register Dump");
#endif

	nxe2000_param_setup(dev, cache);
	nxe2000_device_setup(dev, cache);

#if defined(CONFIG_PMIC_REG_DUMP)
	nxe2000_reg_dump(dev, "PMIC Setup Register Dump");
#endif

	nxe2000_print_chgstate(dev);

	return 0;
}

static int nxe2000_ofdata_to_platdata(struct udevice *dev)
{
	struct dm_nxe2000_platdata *pdata = dev->platdata;
	const void *blob = gd->fdt_blob;
	int nxe2000_node;

	nxe2000_node = fdt_subnode_offset(blob,
		fdt_path_offset(blob, "/"),	"init-nxe2000");

	pdata->off_press_time = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,off_press_time", -ENODATA);

	pdata->vindac = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,vindac", -ENODATA);

	pdata->chgp = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,chgp", -ENODATA);
	pdata->chgcmp_dis = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,chgcmp_dis", -ENODATA);
	pdata->nobatovlim = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,nobatovlim", -ENODATA);
	pdata->otg_boost_en = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,otg_boost_en", -ENODATA);
	pdata->suspend = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,suspend", -ENODATA);
	pdata->jeitaen = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,jeitaen", -ENODATA);
	pdata->vusbchgen = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,vusbchgen", -ENODATA);
	pdata->vadpchgen = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,vadpchgen", -ENODATA);

	pdata->chg_usb_vcontmask = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,chg_usb_vcontmask", -ENODATA);
	pdata->chg_adp_vcontmask = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,chg_adp_vcontmask", -ENODATA);
	pdata->chg_vbus_buck_ths = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,chg_vbus_buck_ths", -ENODATA);
	pdata->chg_vadp_buck_ths = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,chg_vadp_buck_ths", -ENODATA);

	pdata->vsys_vol = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,vsys_vol", -ENODATA);
	pdata->vsys_over_vol = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,vsys_over_vol", -ENODATA);

	pdata->rapid_ttime = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,rapid_ttime", -ENODATA);
	pdata->rapid_ctime = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,rapid_ctime", -ENODATA);
	pdata->rapid_rtime = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,rapid_rtime", -ENODATA);

	pdata->pwr_on_vol = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,pwr_on_vol", -ENODATA);
	pdata->vbatov_set = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,vbatov_set", -ENODATA);
	pdata->vweak = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,vweak", -ENODATA);
	pdata->vdead = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,vdead", -ENODATA);
	pdata->vshort = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,vshort", -ENODATA);

	pdata->die_return_temp = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,die_return_temp", -ENODATA);
	pdata->die_error_temp = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,die_error_temp", -ENODATA);
	pdata->die_shutdown_temp = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,die_shutdown_temp", -ENODATA);

	pdata->vfchg = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,vfchg", -ENODATA);

	pdata->repcnt_off_reseto = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,repcnt_off_reseto", -ENODATA);
	pdata->repcnt_repwrtim = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,repcnt_repwrtim", -ENODATA);
	pdata->repcnt_repwron = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,repcnt_repwron", -ENODATA);

	pdata->wdog_slpen = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,wdog_slpen", -ENODATA);
	pdata->wdog_tim = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,wdog_tim", -ENODATA);

	pdata->pwrien_wdog = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,pwrien_wdog", -ENODATA);
	pdata->pwrien_noe_off = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,pwrien_noe_off", -ENODATA);
	pdata->pwrien_pwron_off = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,pwrien_pwron_off", -ENODATA);
	pdata->pwrien_preot = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,pwrien_preot", -ENODATA);
	pdata->pwrien_prvindt = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,pwrien_prvindt", -ENODATA);
	pdata->pwrien_extin = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,pwrien_extin", -ENODATA);
	pdata->pwrien_pwron = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,pwrien_pwron", -ENODATA);

	pdata->adc_ain0 = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,adc_ain0", -ENODATA);
	pdata->adc_ain1 = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,adc_ain1", -ENODATA);
	pdata->adc_vthm = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,adc_vthm", -ENODATA);
	pdata->adc_vsys = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,adc_vsys", -ENODATA);
	pdata->adc_vusb = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,adc_vusb", -ENODATA);
	pdata->adc_vadp = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,adc_vadp", -ENODATA);
	pdata->adc_vbat = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,adc_vbat", -ENODATA);
	pdata->adc_ilim = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,adc_ilim", -ENODATA);

	pdata->adccnt3_adrq = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,adccnt3_adrq", -ENODATA);
	pdata->adccnt3_ave = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,adccnt3_ave", -ENODATA);
	pdata->adccnt3_adsel = fdtdec_get_int(blob, nxe2000_node,
		"nxe2000,adccnt3_adsel", -ENODATA);

	return 0;
}

static int nxe2000_bind(struct udevice *dev)
{
	int reg_node = 0;
	int chg_node = 0;
	const void *blob = gd->fdt_blob;
	int children;

#ifdef CONFIG_REVISION_TAG
	gpio_init();

	if ((hw_revision() < 3) && !strcmp(dev->name, "nxe2000_gpio@32"))
		return 0;

	if ((hw_revision() >= 3) && !strcmp(dev->name, "nxe2000@32"))
		return 0;
#endif
	debug("%s: dev->name:%s\n", __func__, dev->name);

	if (!strncmp(dev->name, "nxe2000", 7)) {
		reg_node = fdt_subnode_offset(blob, fdt_path_offset(blob, "/"),
				"voltage-regulators");
		chg_node = fdt_subnode_offset(blob, fdt_path_offset(blob, "/"),
				"init-charger");
	} else {
		reg_node = fdt_subnode_offset(blob, dev->of_offset,
				"voltage-regulators");
	}

	if (reg_node > 0) {
		debug("%s: found regulators subnode\n", __func__);
		children = pmic_bind_children(dev, reg_node, pmic_reg_info);
		if (!children)
			debug("%s: %s - no child found\n", __func__, dev->name);
	} else {
		debug("%s: regulators subnode not found!\n", __func__);
	}

	if (chg_node > 0) {
		debug("%s: found charger subnode\n", __func__);
		children = pmic_bind_children(dev, chg_node, pmic_chg_info);
		if (!children)
			debug("%s: %s - no child found\n", __func__, dev->name);
	} else {
		debug("%s: charger subnode not found!\n", __func__);
	}

	/* Always return success for this device */
	return 0;
}

static struct dm_pmic_ops nxe2000_ops = {
	.reg_count = nxe2000_reg_count,
	.read = nxe2000_read,
	.write = nxe2000_write,
};

static const struct udevice_id nxe2000_ids[] = {
	{ .compatible = "nexell,nxe1500", .data = (ulong)TYPE_NXE1500 },
	{ .compatible = "nexell,nxe2000", .data = (ulong)TYPE_NXE2000 },
	{ }
};

U_BOOT_DRIVER(pmic_nxe2000) = {
	.name = "nxe2000_pmic",
	.id = UCLASS_PMIC,
	.of_match = nxe2000_ids,
	.ofdata_to_platdata = nxe2000_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct dm_nxe2000_platdata),
	.bind = nxe2000_bind,
	.probe = nxe2000_probe,
	.ops = &nxe2000_ops,
};
