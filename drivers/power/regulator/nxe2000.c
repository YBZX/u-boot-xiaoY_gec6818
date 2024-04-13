/*
 * nxe2000.c  --  Regulator driver for the NEXELL NXE2000
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

DECLARE_GLOBAL_DATA_PTR;

static const struct sec_voltage_desc buck_v1 = {
	.min = 600000,
	.max = 3500000,
	.step = 12500,
};

static const struct sec_voltage_desc ldo_v1 = {
	.min = 900000,
	.max = 3500000,
	.step = 25000,
};

static const struct sec_voltage_desc ldo_v2 = {
	.min = 600000,
	.max = 3500000,
	.step = 25000,
};

static const struct sec_voltage_desc ldo_v3 = {
	.min = 1700000,
	.max = 3500000,
	.step = 50000,
};

static const struct sec_voltage_desc ldo_v4 = {
	.min = 900000,
	.max = 3500000,
	.step = 50000,
};

static const struct sec_voltage_desc ldo_v5 = {
	.min = 1200000,
	.max = 3500000,
	.step = 25000,
};

static const struct nxe2000_para nxe2000_buck_param[] = {
	{NXE2000_ID_DCDC1, 0x36, 0x3B, 0x0, 0xff, 0x2C, 0, &buck_v1},
	{NXE2000_ID_DCDC2, 0x37, 0x3C, 0x0, 0xff, 0x2E, 0, &buck_v1},
	{NXE2000_ID_DCDC3, 0x38, 0x3D, 0x0, 0xff, 0x30, 0, &buck_v1},
	{NXE2000_ID_DCDC4, 0x39, 0x3E, 0x0, 0xff, 0x32, 0, &buck_v1},
	{NXE2000_ID_DCDC5, 0x3A, 0x3F, 0x0, 0xff, 0x34, 0, &buck_v1},
};

static const struct nxe2000_para nxe1500_ldo_param[] = {
	{NXE2000_ID_LDO1, 0x4C, 0x0, 0x0, 0x7f, 0x44, 0, &ldo_v1},
	{NXE2000_ID_LDO2, 0x4D, 0x0, 0x0, 0x7f, 0x44, 1, &ldo_v1},
	{NXE2000_ID_LDO3, 0x4E, 0x0, 0x0, 0x7f, 0x44, 2, &ldo_v2},
	{NXE2000_ID_LDO4, 0x4F, 0x0, 0x0, 0x7f, 0x44, 3, &ldo_v1},
	{NXE2000_ID_LDO5, 0x50, 0x0, 0x0, 0x7f, 0x44, 4, &ldo_v1},
	{NXE2000_ID_LDO6, 0x51, 0x0, 0x0, 0x7f, 0x44, 5, &ldo_v2},
	{NXE2000_ID_LDO7, 0x52, 0x0, 0x0, 0x7f, 0x44, 6, &ldo_v1},
	{NXE2000_ID_LDO8, 0x53, 0x0, 0x0, 0x7f, 0x44, 7, &ldo_v2},
	{NXE2000_ID_LDO9, 0x54, 0x0, 0x0, 0x7f, 0x45, 0, &ldo_v1},
	{NXE2000_ID_LDO10, 0x55, 0x0, 0x0, 0x7f, 0x45, 1, &ldo_v1},
	{NXE2000_ID_LDORTC1, 0x56, 0x0, 0x0, 0x7f, 0x45, 4, &ldo_v5},
	{NXE2000_ID_LDORTC2, 0x57, 0x0, 0x0, 0x7f, 0x45, 5, &ldo_v1},
};

static const struct nxe2000_para nxe2000_ldo_param[] = {
	{NXE2000_ID_LDO1, 0x4C, 0x58, 0x0, 0x7f, 0x44, 0, &ldo_v1},
	{NXE2000_ID_LDO2, 0x4D, 0x59, 0x0, 0x7f, 0x44, 1, &ldo_v1},
	{NXE2000_ID_LDO3, 0x4E, 0x5A, 0x0, 0x7f, 0x44, 2, &ldo_v1},
	{NXE2000_ID_LDO4, 0x4F, 0x5B, 0x0, 0x7f, 0x44, 3, &ldo_v1},
	{NXE2000_ID_LDO5, 0x50, 0x5C, 0x0, 0x7f, 0x44, 4, &ldo_v2},
	{NXE2000_ID_LDO6, 0x51, 0x5D, 0x0, 0x7f, 0x44, 5, &ldo_v2},
	{NXE2000_ID_LDO7, 0x52, 0x5E, 0x0, 0x7f, 0x44, 6, &ldo_v1},
	{NXE2000_ID_LDO8, 0x53, 0x5F, 0x0, 0x7f, 0x44, 7, &ldo_v1},
	{NXE2000_ID_LDO9, 0x54, 0x60, 0x0, 0x7f, 0x45, 0, &ldo_v1},
	{NXE2000_ID_LDO10, 0x55, 0x61, 0x0, 0x7f, 0x45, 1, &ldo_v1},
	{NXE2000_ID_LDORTC1, 0x56, 0x0, 0x0, 0x7f, 0x45, 4, &ldo_v3},
	{NXE2000_ID_LDORTC2, 0x57, 0x0, 0x0, 0x7f, 0x45, 5, &ldo_v4},
};

static int nxe2000_reg_set_slp_value(struct udevice *dev,
	const struct nxe2000_para *param, int uv)
{
	const struct sec_voltage_desc *desc;
	int ret, val;

	desc = param->vol;
	if (uv < desc->min || uv > desc->max)
		return -EINVAL;

	val = (uv - desc->min) / desc->step;
	val = (val & param->vol_bitmask) << param->vol_bitpos;
	ret = pmic_clrsetbits(dev->parent, param->slp_vol_addr,
		param->vol_bitmask << param->vol_bitpos, val);

	return ret;
}


static int nxe2000_reg_get_value(struct udevice *dev,
	const struct nxe2000_para *param)
{
	const struct sec_voltage_desc *desc;
	int ret, uv, val;

	ret = pmic_reg_read(dev->parent, param->vol_addr);
	if (ret < 0)
		return ret;

	desc = param->vol;
	val = (ret >> param->vol_bitpos) & param->vol_bitmask;
	uv = desc->min + val * desc->step;

	return uv;
}

static int nxe2000_reg_set_value(struct udevice *dev,
	const struct nxe2000_para *param, int uv)
{
	const struct sec_voltage_desc *desc;
	int ret, val;

	desc = param->vol;
	if (uv < desc->min || uv > desc->max)
		return -EINVAL;

	val = (uv - desc->min) / desc->step;
	val = (val & param->vol_bitmask) << param->vol_bitpos;
	ret = pmic_clrsetbits(dev->parent, param->vol_addr,
		param->vol_bitmask << param->vol_bitpos, val);

	return ret;
}

static int nxe2000_ldo_set_slp_value(struct udevice *dev, int uv)
{
	const struct nxe2000_para *ldo_param = NULL;
	int ldo = dev->driver_data;

	switch (dev_get_driver_data(dev_get_parent(dev))) {
	case TYPE_NXE1500:
		ldo_param = &nxe1500_ldo_param[ldo];
		break;
	case TYPE_NXE2000:
		ldo_param = &nxe2000_ldo_param[ldo];
		break;
	default:
		debug("Unsupported NXE2000\n");
		return -EINVAL;
	}

	return nxe2000_reg_set_slp_value(dev, ldo_param, uv);
}

static int nxe2000_ldo_get_value(struct udevice *dev)
{
	const struct nxe2000_para *ldo_param = NULL;
	int ldo = dev->driver_data;

	switch (dev_get_driver_data(dev_get_parent(dev))) {
	case TYPE_NXE1500:
		ldo_param = &nxe1500_ldo_param[ldo];
		break;
	case TYPE_NXE2000:
		ldo_param = &nxe2000_ldo_param[ldo];
		break;
	default:
		debug("Unsupported NXE2000\n");
		return -EINVAL;
	}

	return nxe2000_reg_get_value(dev, ldo_param);
}

static int nxe2000_ldo_set_value(struct udevice *dev, int uv)
{
	const struct nxe2000_para *ldo_param = NULL;
	int ldo = dev->driver_data;

	switch (dev_get_driver_data(dev_get_parent(dev))) {
	case TYPE_NXE1500:
		ldo_param = &nxe1500_ldo_param[ldo];
		break;
	case TYPE_NXE2000:
		ldo_param = &nxe2000_ldo_param[ldo];
		break;
	default:
		debug("Unsupported NXE2000\n");
		return -EINVAL;
	}

	return nxe2000_reg_set_value(dev, ldo_param, uv);
}

static int nxe2000_reg_get_enable(struct udevice *dev,
	const struct nxe2000_para *param)
{
	bool enable;
	int ret;

	ret = pmic_reg_read(dev->parent, param->reg_enaddr);
	if (ret < 0)
		return ret;

	enable = (ret >> param->reg_enbitpos) & 0x1;

	return enable;
}

static int nxe2000_reg_set_enable(struct udevice *dev,
	const struct nxe2000_para *param, bool enable)
{
	int ret;

	ret = pmic_reg_read(dev->parent, param->reg_enaddr);
	if (ret < 0)
		return ret;

	ret = pmic_clrsetbits(dev->parent, param->reg_enaddr,
		0x1 << param->reg_enbitpos,
		enable ? 0x1 << param->reg_enbitpos : 0);

	return ret;
}

static bool nxe2000_ldo_get_enable(struct udevice *dev)
{
	const struct nxe2000_para *ldo_param = NULL;
	int ldo = dev->driver_data;

	switch (dev_get_driver_data(dev_get_parent(dev))) {
	case TYPE_NXE1500:
		ldo_param = &nxe1500_ldo_param[ldo];
		break;
	case TYPE_NXE2000:
		ldo_param = &nxe2000_ldo_param[ldo];
		break;
	default:
		debug("Unsupported NXE2000\n");
		return -EINVAL;
	}

	return nxe2000_reg_get_enable(dev, ldo_param);
}

static int nxe2000_ldo_set_enable(struct udevice *dev, bool enable)
{
	const struct nxe2000_para *ldo_param = NULL;
	int ldo = dev->driver_data;

	switch (dev_get_driver_data(dev_get_parent(dev))) {
	case TYPE_NXE1500:
		ldo_param = &nxe1500_ldo_param[ldo];
		break;
	case TYPE_NXE2000:
		ldo_param = &nxe2000_ldo_param[ldo];
		break;
	default:
		debug("Unsupported NXE2000\n");
		return -EINVAL;
	}

	return nxe2000_reg_set_enable(dev, ldo_param, enable);
}

static int nxe2000_ldo_probe(struct udevice *dev)
{
	struct dm_nxe2000_buck_platdata *pdata = dev->platdata;
	struct dm_regulator_uclass_platdata *uc_pdata;

	uc_pdata = dev_get_uclass_platdata(dev);

	uc_pdata->type = REGULATOR_TYPE_LDO;
	uc_pdata->mode_count = 0;

	if (pdata->vol != -ENODATA)
		nxe2000_ldo_set_value(dev, pdata->vol);

	if (pdata->slp_vol != -ENODATA)
		nxe2000_ldo_set_slp_value(dev, pdata->slp_vol);

	if (pdata->on == 1)
		nxe2000_ldo_set_enable(dev, 1);
	else if (pdata->on == 0)
		nxe2000_ldo_set_enable(dev, 0);

#if defined(CONFIG_PMIC_REG_DUMP)
	printf("LDO%02lu: %4dmV, %s\n", (dev->driver_data+1)
		, nxe2000_ldo_get_value(dev)/1000
		, nxe2000_ldo_get_enable(dev) ? "En" : "Dis");
#endif
	return 0;
}

static int nxe2000_ldo_ofdata_to_platdata(struct udevice *dev)
{
	struct dm_nxe2000_buck_platdata *pdata = dev->platdata;
	const void *blob = gd->fdt_blob;
	int offset = dev->of_offset;

	pdata->vol = fdtdec_get_int(blob, offset,
		"ldo,vol", -ENODATA);
	pdata->slp_vol = fdtdec_get_int(blob, offset,
		"ldo,slp_vol", -ENODATA);
	pdata->on = fdtdec_get_int(blob, offset,
		"ldo,on", -ENODATA);

	return 0;
}

static int nxe2000_ldo_bind(struct udevice *dev)
{
	return 0;
}

static int nxe2000_buck_set_slp_value(struct udevice *dev, int uv)
{
	int buck = dev->driver_data;

	return nxe2000_reg_set_slp_value(dev, &nxe2000_buck_param[buck], uv);
}

static int nxe2000_buck_get_value(struct udevice *dev)
{
	int buck = dev->driver_data;

	return nxe2000_reg_get_value(dev, &nxe2000_buck_param[buck]);
}

static int nxe2000_buck_set_value(struct udevice *dev, int uv)
{
	int buck = dev->driver_data;

	return nxe2000_reg_set_value(dev, &nxe2000_buck_param[buck], uv);
}

static bool nxe2000_buck_get_enable(struct udevice *dev)
{
	int buck = dev->driver_data;

	return nxe2000_reg_get_enable(dev, &nxe2000_buck_param[buck]);
}

static int nxe2000_buck_set_enable(struct udevice *dev, bool enable)
{
	int buck = dev->driver_data;

	return nxe2000_reg_set_enable(dev, &nxe2000_buck_param[buck], enable);
}

static int nxe2000_buck_probe(struct udevice *dev)
{
	struct dm_nxe2000_buck_platdata *pdata = dev->platdata;
	struct dm_regulator_uclass_platdata *uc_pdata;
	uint value;

	uc_pdata = dev_get_uclass_platdata(dev);

	uc_pdata->type = REGULATOR_TYPE_BUCK;
	uc_pdata->mode_count = 0;

	if (pdata->vol != -ENODATA)
		nxe2000_buck_set_value(dev, pdata->vol);

	if (pdata->slp_vol != -ENODATA)
		nxe2000_buck_set_slp_value(dev, pdata->slp_vol);

	/* DCDC control register */
	if (
		pdata->osc_freq != -ENODATA &&
		pdata->ramp_slop != -ENODATA &&
		pdata->cur_limit != -ENODATA &&
		pdata->limshut_en != -ENODATA) {
		value = ((pdata->osc_freq << NXE2000_POS_DCxCTL2_DCxOSC)
			| (pdata->ramp_slop << NXE2000_POS_DCxCTL2_DCxSR)
			| (pdata->cur_limit << NXE2000_POS_DCxCTL2_DCxLIM)
			| (pdata->limshut_en <<
			NXE2000_POS_DCxCTL2_DCxLIMSDEN));
		pmic_reg_write(dev->parent, NXE2000_REG_DC1CTL2, value);
	}

	if (
		pdata->slp_mode != -ENODATA &&
		pdata->mode != -ENODATA &&
		pdata->dsc_ctrl != -ENODATA) {
		value = ((pdata->slp_mode << NXE2000_POS_DCxCTL_DCxMODE_SLP)
			| (pdata->mode << NXE2000_POS_DCxCTL_DCxMODE)
			| (pdata->dsc_ctrl << NXE2000_POS_DCxCTL_DCxDIS));
		pmic_clrsetbits(dev->parent, NXE2000_REG_DC1CTL, 0xFE, value);
	}

	/* DCDC - Enable */
	if (pdata->on == 1)
		nxe2000_buck_set_enable(dev, 1);
	else if (pdata->on == 0)
		nxe2000_buck_set_enable(dev, 0);

	printf("DCDC%lu: %4dmV, %s\n", (dev->driver_data+1)
		, nxe2000_buck_get_value(dev)/1000
		, nxe2000_buck_get_enable(dev) ? "En" : "Dis");

	return 0;
}

static int nxe2000_buck_ofdata_to_platdata(struct udevice *dev)
{
	struct dm_nxe2000_buck_platdata *pdata = dev->platdata;
	const void *blob = gd->fdt_blob;
	int offset = dev->of_offset;

	pdata->vol = fdtdec_get_int(blob, offset,
		"buck,vol", -ENODATA);
	pdata->slp_vol = fdtdec_get_int(blob, offset,
		"buck,slp_vol", -ENODATA);

	pdata->slp_mode = fdtdec_get_int(blob, offset,
		"buck,slp_mode", -ENODATA);
	pdata->mode = fdtdec_get_int(blob, offset,
		"buck,mode", -ENODATA);
	pdata->dsc_ctrl = fdtdec_get_int(blob, offset,
		"buck,dsc_ctrl", -ENODATA);
	pdata->on = fdtdec_get_int(blob, offset,
		"buck,on", -ENODATA);

	pdata->osc_freq = fdtdec_get_int(blob, offset,
		"buck,osc_freq", -ENODATA);
	pdata->ramp_slop = fdtdec_get_int(blob, offset,
		"buck,ramp_slop", -ENODATA);
	pdata->cur_limit = fdtdec_get_int(blob, offset,
		"buck,cur_limit", -ENODATA);
	pdata->limshut_en = fdtdec_get_int(blob, offset,
		"buck,limshut_en", -ENODATA);

	return 0;
}

static int nxe2000_buck_bind(struct udevice *dev)
{
	return 0;
}

static const struct dm_regulator_ops nxe2000_ldo_ops = {
	.get_value  = nxe2000_ldo_get_value,
	.set_value  = nxe2000_ldo_set_value,
	.get_enable = nxe2000_ldo_get_enable,
	.set_enable = nxe2000_ldo_set_enable,
};

U_BOOT_DRIVER(nxe2000_ldo) = {
	.name = NXE2000_LDO_DRIVER,
	.id = UCLASS_REGULATOR,
	.ops = &nxe2000_ldo_ops,
	.bind = nxe2000_ldo_bind,
	.probe = nxe2000_ldo_probe,
	.ofdata_to_platdata = nxe2000_ldo_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct dm_nxe2000_ldo_platdata),
};

static const struct dm_regulator_ops nxe2000_buck_ops = {
	.get_value  = nxe2000_buck_get_value,
	.set_value  = nxe2000_buck_set_value,
	.get_enable = nxe2000_buck_get_enable,
	.set_enable = nxe2000_buck_set_enable,
};

U_BOOT_DRIVER(nxe2000_buck) = {
	.name = NXE2000_BUCK_DRIVER,
	.id = UCLASS_REGULATOR,
	.ops = &nxe2000_buck_ops,
	.bind = nxe2000_buck_bind,
	.probe = nxe2000_buck_probe,
	.ofdata_to_platdata = nxe2000_buck_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct dm_nxe2000_buck_platdata),
};
