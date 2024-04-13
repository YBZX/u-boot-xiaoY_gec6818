/*
 * (C) Copyright 2016 Nexell
 * Youngbok, Park <park@nexell.co.kr>
 *
 * SPDX-License-Identifier:      GPL-2.0+
 */

#include <common.h>
#include <asm/arch/nx_gpio.h>
#include <asm/arch/nexell.h>
#include <asm/gpio.h>
#include <fdtdec.h>
#include <libfdt.h>

DECLARE_GLOBAL_DATA_PTR;

#define MAX_TR	8

struct board_rev {
	int gpio_num;
	struct gpio_desc gpios[4];
	int board_rev;
	int use_tr;
	int tr[MAX_TR][2];
	int read_ver;
};

static struct board_rev rev;

static int parse_dts_revision_gpio(void)
{
	int compat_id;
	int node = 0;
	int length = 0;
	const fdt32_t *list;
	int i, ret;

	compat_id = COMPAT_NEXELL_HWVER;
	ret = fdtdec_find_aliases_for_id(gd->fdt_blob, "board_rev",
				compat_id, &node, 1);
	if (!ret) {
		printf("not exist board_rev dts node!\n");
		return -1;
	}

	rev.gpio_num = gpio_request_list_by_name_nodev(gd->fdt_blob, node,
			"gpios", rev.gpios, ARRAY_SIZE(rev.gpios), GPIOD_IS_IN);
	list = fdt_getprop(gd->fdt_blob, node, "maps", &length);
	if (!list) {
		rev.use_tr = 0;
		return 0;
	}
	rev.use_tr = 1;

	length /= sizeof(*list);
	for (i = 0; i < length/2; i++) {
		rev.tr[i][0] = fdt32_to_cpu(*list++);
		rev.tr[i][1] = fdt32_to_cpu(*list++);
	}
	return 0;
}

static int read_ver(void)
{
	int i, ret;

	if (!rev.read_ver) {
		ret = parse_dts_revision_gpio();
		if (ret)
			return ret;

		for (i = 0; i < rev.gpio_num; i++)
			rev.board_rev |= dm_gpio_get_value(&rev.gpios[i]) << i;

		if (rev.use_tr) {
			for (i = 0; i < MAX_TR; i++) {
				if (rev.tr[i][0] == rev.board_rev) {
					rev.board_rev = rev.tr[i][1];
					break;
				}
			}
		}
	}
	printf("HW Revision : %d\n", rev.board_rev);

	rev.read_ver = 1;
	return rev.board_rev;
}

static void set_board_rev(void)
{
	char info[64] = {0, };

	if (rev.read_ver) {
		snprintf(info, ARRAY_SIZE(info), "%d", rev.board_rev);
		setenv("board_rev", info);
	}
}

int check_hw_revision(void)
{
	return read_ver();
}
EXPORT_SYMBOL_GPL(check_hw_revision);

static int do_load_rev(cmd_tbl_t *cmdtp, int flag, int argc,
		char * const argv[])
{
	int ret;

	ret = read_ver();
	if (ret < 0)
		return CMD_RET_FAILURE;

	set_board_rev();

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	check_hw, 1, 0, do_load_rev,
	"Check Board revision",
	"- Check HW board revision and set board_rev env. "
);
