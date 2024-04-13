 /*
 * charger.h  --  charger driver for the NEXELL NXE2000
 *
 * Copyright (C) 2017  Nexell Co., Ltd.
 * Author: Jongshin, Park <pjsin865@nexell.co.kr>
 *
 * SPDX-License-Identifier:     GPL-2.0+

 */

#ifndef __CHARGER_H__
#define __CHARGER_H__

struct dm_charger_uclass_platdata {
	const char *name;
};

/* Charger device operations */
struct dm_charger_ops {
	/**
	 * The vbatt output value function calls operates on a micro Volts.
	 *
	 * vbatt_get_value - get output value of battery voltage
	 * @dev       - charger device
	 */
	int (*get_value_vbatt)(struct udevice *dev);

	/**
	 * This function can know the charge type.
	 *
	 * get_charger_type - get output value of charge type
	 * @dev                 - charger device
	 */
	int (*get_charge_type)(struct udevice *dev);

	/**
	 * This function sets the charge current.
	 *
	 * get/set_charge_current - get/set charging current of battery
	 *     @dev                      - charger device
	 * Sets:
	 *     @type                     - charger type(USB/ADP)
	 *     @uA                        - set charging current [micro Amps]
	 *     @return output value [uA] on success or negative errno if fail.
	 */
	int (*get_charge_current)(struct udevice *dev);
	int (*set_charge_current)(struct udevice *dev, int uA);

	/**
	 * This function sets the limit current.
	 *
	 * get/set_limit_current - Set input current of PMIC
	 *     @dev                      - charger device
	 * Sets:
	 *     @type                     - charger type(USB/ADP)
	 *     @uA                        - set charging current [micro Amps]
	 *     @return output value [uA] on success or negative errno if fail.
	 */
	int (*get_limit_current)(struct udevice *dev);
	int (*set_limit_current)(struct udevice *dev, int type, int uA);
};

int charger_get_value_vbatt(struct udevice *dev);
int charger_get_charge_type(struct udevice *dev);
int charger_get_charge_current(struct udevice *dev);
int charger_set_charge_current(struct udevice *dev, int uA);
int charger_get_limit_current(struct udevice *dev);
int charger_set_limit_current(struct udevice *dev, int type, int uA);
int charger_get_by_platname(const char *plat_name, struct udevice **devp);
int charger_get_by_devname(const char *devname, struct udevice **devp);

#endif /* __CHARGER_H__ */
