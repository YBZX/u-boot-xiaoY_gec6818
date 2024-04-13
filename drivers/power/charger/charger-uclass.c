/*
 * charger  --  charger uclass for the NEXELL NXE2000
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
#include <dm/uclass-internal.h>
#include <power/pmic.h>
#include <power/charger.h>

DECLARE_GLOBAL_DATA_PTR;

int charger_get_value_vbatt(struct udevice *dev)
{
	const struct dm_charger_ops *ops = dev_get_driver_ops(dev);

	if (!ops || !ops->get_value_vbatt)
		return -ENOSYS;

	return ops->get_value_vbatt(dev);
}

int charger_get_charge_type(struct udevice *dev)
{
	const struct dm_charger_ops *ops = dev_get_driver_ops(dev);

	if (!ops || !ops->get_charge_type)
		return -ENOSYS;

	return ops->get_charge_type(dev);
}

int charger_get_charge_current(struct udevice *dev)
{
	const struct dm_charger_ops *ops = dev_get_driver_ops(dev);

	if (!ops || !ops->get_charge_type)
		return -ENOSYS;

	return ops->get_charge_type(dev);
}

int charger_set_charge_current(struct udevice *dev, int uA)
{
	const struct dm_charger_ops *ops = dev_get_driver_ops(dev);

	if (!ops || !ops->set_charge_current)
		return -ENOSYS;

	return ops->set_charge_current(dev, uA);
}

int charger_get_limit_current(struct udevice *dev)
{
	const struct dm_charger_ops *ops = dev_get_driver_ops(dev);

	if (!ops || !ops->get_limit_current)
		return -ENOSYS;

	return ops->get_limit_current(dev);
}

int charger_set_limit_current(struct udevice *dev, int type, int uA)
{
	const struct dm_charger_ops *ops = dev_get_driver_ops(dev);

	if (!ops || !ops->set_limit_current)
		return -ENOSYS;

	return ops->set_limit_current(dev, type, uA);
}

int charger_get_by_platname(const char *plat_name, struct udevice **devp)
{
	struct dm_charger_uclass_platdata *uc_pdata;
	struct udevice *dev;
	int ret;

	*devp = NULL;

	for (ret = uclass_find_first_device(UCLASS_CHARGER, &dev); dev;
	     ret = uclass_find_next_device(&dev)) {
		if (ret)
			continue;

		uc_pdata = dev_get_uclass_platdata(dev);
		if (!uc_pdata || strcmp(plat_name, uc_pdata->name))
			continue;

		return uclass_get_device_tail(dev, 0, devp);
	}

	debug("%s: can't find: %s\n", __func__, plat_name);

	return -ENODEV;
}

int charger_get_by_devname(const char *devname, struct udevice **devp)
{
	return uclass_get_device_by_name(UCLASS_CHARGER, devname, devp);
}

static bool charger_name_is_unique(struct udevice *check_dev,
				     const char *check_name)
{
	struct dm_charger_uclass_platdata *uc_pdata;
	struct udevice *dev;
	int check_len = strlen(check_name);
	int ret;
	int len;

	for (ret = uclass_find_first_device(UCLASS_CHARGER, &dev); dev;
	     ret = uclass_find_next_device(&dev)) {
		if (ret || dev == check_dev)
			continue;

		uc_pdata = dev_get_uclass_platdata(dev);
		len = strlen(uc_pdata->name);
		if (len != check_len)
			continue;

		if (!strcmp(uc_pdata->name, check_name))
			return false;
	}

	return true;
}

static int charger_post_bind(struct udevice *dev)
{
	struct dm_charger_uclass_platdata *uc_pdata;
	int offset = dev->of_offset;
	const void *blob = gd->fdt_blob;
	const char *property = "charger-name";

	uc_pdata = dev_get_uclass_platdata(dev);
	if (!uc_pdata)
		return -ENXIO;

	/* Charger mandatory constraint */
	uc_pdata->name = fdt_getprop(blob, offset, property, NULL);
	if (!uc_pdata->name) {
		debug("%s: dev: %s has no property 'charger-name'\n",
		      __func__, dev->name);
		uc_pdata->name = fdt_get_name(blob, offset, NULL);
		if (!uc_pdata->name)
			return -EINVAL;
	}

	if (charger_name_is_unique(dev, uc_pdata->name))
		return 0;

	debug("\"%s\" of dev: \"%s\", has nonunique value: \"%s\"",
	      property, dev->name, uc_pdata->name);

	return -EINVAL;
}

static int charger_pre_probe(struct udevice *dev)
{
	return 0;
}

UCLASS_DRIVER(charger) = {
	.id		= UCLASS_CHARGER,
	.name		= "charger",
	.post_bind	= charger_post_bind,
	.pre_probe	= charger_pre_probe,
	.per_device_platdata_auto_alloc_size =
				sizeof(struct dm_charger_uclass_platdata),
};
