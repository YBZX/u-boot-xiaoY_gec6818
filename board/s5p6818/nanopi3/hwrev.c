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
#include <asm/io.h>

#include <asm/arch/nexell.h>
#include <asm/arch/nx_gpio.h>

/* Board revision list: <PCB3 | PCB2 | PCB1>
 *  0b001 - NanoPC-T3
 *  0b100 - NanoPC-T3 Trunk
 *  0b011 - Smart6818
 *  0b101 - Fire 3
 *  0b111 - NanoPi M3
 */
#define __IO_GRP		2	/* GPIO_C */
#define __IO_PCB1			26
#define __IO_PCB2			27
#define __IO_PCB3			25

static int pcb_rev = -1;
static int base_rev	= 0;


static void smart6818_rev_init(void)
{
	/* nothing here yet */
}

static void bd_hwrev_config_gpio(void)
{
	int gpios[3][2] = {
		{ __IO_PCB1, 1 },
		{ __IO_PCB2, 1 },
		{ __IO_PCB3, 1 },
	};
	int i;

	/* gpio input mode, pull-down */
	for (i = 0; i < 3; i++) {
		nx_gpio_set_pad_function(__IO_GRP, gpios[i][0], gpios[i][1]);
		nx_gpio_set_output_enable(__IO_GRP, gpios[i][0], 0);
		nx_gpio_set_pull_mode(__IO_GRP, gpios[i][0], 0);
	}
}

void bd_hwrev_init(void)
{
	if (pcb_rev >= 0)
		return;

	bd_hwrev_config_gpio();

	pcb_rev  = nx_gpio_get_input_value(__IO_GRP, __IO_PCB1);
	pcb_rev |= nx_gpio_get_input_value(__IO_GRP, __IO_PCB2) << 1;
	pcb_rev |= nx_gpio_get_input_value(__IO_GRP, __IO_PCB3) << 2;

	/* Get extended revision for SmartXX18 */
	if (pcb_rev == 0x3)
		smart6818_rev_init();
}

/* To override __weak symbols */
u32 get_board_rev(void)
{
	return (base_rev << 8) | pcb_rev;
}

const char *get_board_name(void)
{
	bd_hwrev_init();

	switch (pcb_rev) {
		case 1:
			return "NanoPC-T3";
		case 3:
			return "Smart6818";
		case 4:
			return "NanoPC-T3T";
		case 5:
			return "NanoPi Fire 3";
		case 7:
			return "NanoPi M3";
		case 2:
			return "NanoPi M3B";
		default:
			return "s5p6818-X";
	}
}

