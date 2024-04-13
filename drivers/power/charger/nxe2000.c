/*
 * nxe2000.c  --  Charger driver for the NEXELL NXE2000
 *
 * Copyright (C) 2017  Nexell Co., Ltd.
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
#include <power/charger.h>
#include <power/nxe2000.h>

DECLARE_GLOBAL_DATA_PTR;

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
		ret = pmic_read(dev->parent, i, &value, 1);
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

static int nxe2000_get_value_cap(struct udevice *dev)
{
	struct dm_nxe2000_chg_platdata *pdata = dev->platdata;
	uint8_t val;
	uint8_t vol0, vol1;
	int ret = 0;

	ret = pmic_read(dev->parent, NXE2000_REG_SOC, &val, 1);
	pdata->state_of_chrg = val;

	ret |= pmic_read(dev->parent, NXE2000_REG_VBATDATAH, &vol1, 1);
	ret |= pmic_read(dev->parent, NXE2000_REG_VBATDATAL, &vol0, 1);

	val = (vol1 << 4) | vol0;
	debug("vfsoc: 0x%x\n", val);

	vol0 = (NXE2000_POS_ADCCNT3_ADRQ_SINGLE | NXE2000_POS_ADCCNT3_ADSEL_VBAT);
	pmic_write(dev->parent, NXE2000_REG_ADCCNT3, &vol0, 1);

	/* conversion unit 1 Unit is 1.22mV (5000/4095 mV) */
	val = (val * 5000) / 4095;

	/* return unit should be 1uV */
	pdata->voltage_uV = val * 1000;

	ret |= pmic_read(dev->parent, NXE2000_REG_RE_CAP_H, &vol1, 1);
	ret |= pmic_read(dev->parent, NXE2000_REG_RE_CAP_L, &vol0, 1);

	pdata->capacity = (vol1 << 8) | vol0;

	return ret;
}

static int nxe2000_get_value_vbatt(struct udevice *dev)
{
	struct dm_nxe2000_chg_platdata *pdata = dev->platdata;
	unsigned int voltage_uV = 0, count;
	uint8_t val;
	int ret = 0;

	ret = pmic_read(dev->parent, NXE2000_REG_LSIVER, &val, 1);
	pdata->version = val;

	for (count = 0; count < 2; count++) {
		nxe2000_get_value_cap(dev);
		mdelay(50);
	}
	for (count = 0; count < 3; count++) {
		nxe2000_get_value_cap(dev);
		voltage_uV += pdata->voltage_uV;
		mdelay(10);
	}
	pdata->voltage_uV = (voltage_uV / 3);

	debug("fg ver: 0x%x\n", pdata->version);
	printf("BAT: state_of_charge(SOC):%d%%\n",
		pdata->state_of_chrg);

	printf("     voltage: %d.%6.6d [V] (expected to be %d [mAh])\n",
		pdata->voltage_uV / 1000000,
		pdata->voltage_uV % 1000000,
		pdata->capacity);

	if (pdata->voltage_uV > 3850000)
		pdata->state = EXT_SOURCE;
	else if (pdata->voltage_uV < 3600000 || pdata->state_of_chrg < 5)
		pdata->state = CHARGE;
	else
		pdata->state = NORMAL;

	return ret;
}

static int nxe2000_get_charge_type(struct udevice *dev)
{
	return 0;
}

static int nxe2000_chg_probe(struct udevice *dev)
{
	struct dm_nxe2000_chg_platdata *pdata = dev->platdata;
	uint8_t cache[256] = {0x0,};

	pmic_read(dev->parent, NXE2000_REG_FG_CTRL, &cache[NXE2000_REG_FG_CTRL], 1);
	cache[NXE2000_REG_FG_CTRL] &= (1 << NXE2000_POS_FG_CTRL_FG_ACC);
	cache[NXE2000_REG_FG_CTRL] =
		(1 << NXE2000_POS_FG_CTRL_SRST0) | (1 << NXE2000_POS_FG_CTRL_SRST1);
	pmic_write(dev->parent, NXE2000_REG_FG_CTRL, &cache[NXE2000_REG_FG_CTRL], 1);

	cache[NXE2000_REG_CHGCTL1] =
		((pdata->priority << NXE2000_POS_CHGCTL1_CHGP)
		| (pdata->complete_dis << NXE2000_POS_CHGCTL1_CHGCMP_DIS)
		| (pdata->nobat_ovlim_en << NXE2000_POS_CHGCTL1_NOBATOVLIM)
		| (pdata->otg_boost << NXE2000_POS_CHGCTL1_OTG_BOOST_EN)
		| (pdata->suspend << NXE2000_POS_CHGCTL1_SUSPEND)
		| (pdata->jeitaen << NXE2000_POS_CHGCTL1_JEITAEN)
		| (pdata->usb_en << NXE2000_POS_CHGCTL1_VUSBCHGEN)
		| (pdata->adp_en << NXE2000_POS_CHGCTL1_VADPCHGEN));

	cache[NXE2000_REG_REGISET1] = (pdata->limit_adp_amps / 100000) - 1;

	cache[NXE2000_REG_REGISET2]	=
		0xE0 | ((pdata->limit_usbdata_amps / 100000) - 1);

	cache[NXE2000_REG_CHGISET] =
		((pdata->current_complete << NXE2000_POS_CHGISET_ICCHG)
		| ((pdata->chg_usb_amps / 100000) - 1));

	cache[NXE2000_REG_TIMSET] =
		((pdata->rapid_ttim_80 << NXE2000_POS_TIMSET_TTIMSET)
		| ((pdata->rapid_ctime & 3) << NXE2000_POS_TIMSET_CTIMSET)
		| (pdata->rapid_rtime & 3));

	cache[NXE2000_REG_BATSET1] =
		(((pdata->power_on_vol & 7) << NXE2000_POS_BATSET1_CHGPON)
		| ((pdata->vbatov_set & 1) << NXE2000_POS_BATSET1_VBATOVSET)
		| ((pdata->vweak & 3) << NXE2000_POS_BATSET1_VWEAK)
		| ((pdata->vdead & 1) << NXE2000_POS_BATSET1_VDEAD)
		| ((pdata->vshort & 1) << NXE2000_POS_BATSET1_VSHORT));

	cache[NXE2000_REG_BATSET2] =
		(((pdata->vfchg & 7) << NXE2000_POS_BATSET2_VFCHG)
		|(pdata->vfchg & 7));

	cache[NXE2000_REG_FG_CTRL] =
		((1 << NXE2000_POS_FG_CTRL_FG_ACC)
		| (1 << NXE2000_POS_FG_CTRL_FG_EN));

	if (
		(pdata->priority != -ENODATA) &&
		(pdata->complete_dis != -ENODATA) &&
		(pdata->nobat_ovlim_en != -ENODATA) &&
		(pdata->otg_boost != -ENODATA) &&
		(pdata->suspend != -ENODATA) &&
		(pdata->jeitaen != -ENODATA) &&
		(pdata->usb_en != -ENODATA) &&
		(pdata->adp_en != -ENODATA))
		pmic_write(dev->parent, NXE2000_REG_CHGCTL1
			, &cache[NXE2000_REG_CHGCTL1], 1);

	if (pdata->limit_adp_amps != -ENODATA)
		pmic_write(dev->parent, NXE2000_REG_REGISET1
			, &cache[NXE2000_REG_REGISET1], 1);

	if (pdata->limit_usbdata_amps != -ENODATA)
		pmic_write(dev->parent, NXE2000_REG_REGISET2
			, &cache[NXE2000_REG_REGISET2], 1);

	if (
		(pdata->current_complete != -ENODATA) &&
		(pdata->chg_usb_amps != -ENODATA))
		pmic_write(dev->parent, NXE2000_REG_CHGISET
			, &cache[NXE2000_REG_CHGISET], 1);

	if (
		(pdata->rapid_ttim_80 != -ENODATA) &&
		(pdata->rapid_ctime != -ENODATA) &&
		(pdata->rapid_rtime != -ENODATA))
		pmic_write(dev->parent, NXE2000_REG_TIMSET
			, &cache[NXE2000_REG_TIMSET], 1);

	if (
		(pdata->power_on_vol != -ENODATA) &&
		(pdata->vbatov_set != -ENODATA) &&
		(pdata->vweak != -ENODATA) &&
		(pdata->vdead != -ENODATA) &&
		(pdata->vshort != -ENODATA))
		pmic_write(dev->parent, NXE2000_REG_BATSET1
			, &cache[NXE2000_REG_BATSET1], 1);

	if (pdata->vfchg != -ENODATA)
		pmic_write(dev->parent, NXE2000_REG_BATSET2
			, &cache[NXE2000_REG_BATSET2], 1);

	pmic_write(dev->parent, NXE2000_REG_FG_CTRL
		, &cache[NXE2000_REG_FG_CTRL], 1);

#if defined(CONFIG_PMIC_REG_DUMP)
	nxe2000_reg_dump(dev, "Charger Setup Register Dump");
#endif
	return 0;
}

static int nxe2000_chg_ofdata_to_platdata(struct udevice *dev)
{
	struct dm_nxe2000_chg_platdata *pdata = dev->platdata;
	const void *blob = gd->fdt_blob;
	int offset = dev->of_offset;

	pdata->limit_adp_amps = fdtdec_get_int(blob, offset,
		"chg,limit_adp_amps", -ENODATA);
	pdata->limit_usb_amps = fdtdec_get_int(blob, offset,
		"chg,limit_usb_amps", -ENODATA);
	pdata->limit_usbdata_amps = fdtdec_get_int(blob, offset,
		"chg,limit_usbdata_amps", -ENODATA);

	pdata->chg_adp_amps = fdtdec_get_int(blob, offset,
		"chg,chg_adp_amps", -ENODATA);
	pdata->chg_usb_amps = fdtdec_get_int(blob, offset,
		"chg,chg_usb_amps", -ENODATA);
	pdata->current_complete = fdtdec_get_int(blob, offset,
		"chg,current_complete", -ENODATA);

	pdata->cutoff_vol = fdtdec_get_int(blob, offset,
		"chg,cutoff_vol", -ENODATA);
	pdata->lowbat_battery_vol = fdtdec_get_int(blob, offset,
		"chg,lowbat_battery_vol", -ENODATA);
	pdata->lowbat_adp_vol = fdtdec_get_int(blob, offset,
		"chg,lowbat_adp_vol", -ENODATA);
	pdata->lowbat_usb_vol = fdtdec_get_int(blob, offset,
		"chg,lowbat_usb_vol", -ENODATA);
	pdata->lowbat_usbdata_vol = fdtdec_get_int(blob, offset,
		"chg,lowbat_usbdata_vol", -ENODATA);

	pdata->priority = fdtdec_get_int(blob, offset,
		"chg,priority", -ENODATA);
	pdata->complete_dis = fdtdec_get_int(blob, offset,
		"chg,complete_dis", -ENODATA);
	pdata->nobat_ovlim_en = fdtdec_get_int(blob, offset,
		"chg,nobat_ovlim_en", -ENODATA);
	pdata->otg_boost = fdtdec_get_int(blob, offset,
		"chg,otg_boost", -ENODATA);
	pdata->suspend = fdtdec_get_int(blob, offset,
		"chg,suspend", -ENODATA);
	pdata->jeitaen = fdtdec_get_int(blob, offset,
		"chg,jeitaen", -ENODATA);
	pdata->usb_en = fdtdec_get_int(blob, offset,
		"chg,usb_en", -ENODATA);
	pdata->adp_en = fdtdec_get_int(blob, offset,
		"chg,adp_en", -ENODATA);

	pdata->rapid_ttim_80 = fdtdec_get_int(blob, offset,
		"chg,rapid_ttim_80", -ENODATA);
	pdata->rapid_ctime = fdtdec_get_int(blob, offset,
		"chg,rapid_ctime", -ENODATA);
	pdata->rapid_rtime = fdtdec_get_int(blob, offset,
		"chg,rapid_rtime", -ENODATA);

	pdata->power_on_vol = fdtdec_get_int(blob, offset,
		"chg,power_on_vol", -ENODATA);
	pdata->vbatov_set = fdtdec_get_int(blob, offset,
		"chg,vbatov_set", -ENODATA);
	pdata->vweak = fdtdec_get_int(blob, offset,
		"chg,vweak", -ENODATA);
	pdata->vdead = fdtdec_get_int(blob, offset,
		"chg,vdead", -ENODATA);
	pdata->vshort = fdtdec_get_int(blob, offset,
		"chg,vshort", -ENODATA);

	pdata->vfchg = fdtdec_get_int(blob, offset,
		"chg,vfchg", -ENODATA);

	return 0;
}

static int nxe2000_chg_bind(struct udevice *dev)
{
	return 0;
}

static const struct dm_charger_ops nxe2000_chg_ops = {
	.get_value_vbatt = nxe2000_get_value_vbatt,
	.get_charge_type = nxe2000_get_charge_type,
};

U_BOOT_DRIVER(nxe2000_charger) = {
	.name = NXE2000_CHG_DRIVER,
	.id = UCLASS_CHARGER,
	.ops = &nxe2000_chg_ops,
	.bind = nxe2000_chg_bind,
	.probe = nxe2000_chg_probe,
	.ofdata_to_platdata = nxe2000_chg_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct dm_nxe2000_chg_platdata),
};
