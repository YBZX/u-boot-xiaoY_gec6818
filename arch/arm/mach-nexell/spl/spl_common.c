/*
 * (C) Copyright 2015 spl
 * jhkim <jhkim@nexell.co.kr>
 *
 * SPDX-License-Identifier:      GPL-2.0+
 */

#include <common.h>
#include <config.h>

DECLARE_GLOBAL_DATA_PTR;

#if defined CONFIG_USE_TINY_PRINTF && defined DEBUG
void __assert_fail(const char *assertion, const char *file, unsigned line,
		   const char *function)
{
	/* This will not return */
	panic("%s:%u: %s: Assertion `%s' failed.", file, line, function,
	      assertion);
}
#endif

#ifndef CONFIG_OF_LIBFDT
void arch_preboot_linux(int flag, int argc, char *const argv[],
			bootm_headers_t *images)
{
	/* set DTB address for linux kernel */
	if (!argc) {
		for (argc = 0;; argc++)
			if (!argv[argc])
				break;
	}

	if (argc > 2) {
		unsigned long ft_addr;

		ft_addr = simple_strtol(argv[2], NULL, 16);
		images->ft_addr = (char *)ft_addr;

		/*
		 * if not defined IMAGE_ENABLE_OF_LIBFDT,
		 * must be set to fdt address
		 */
		if (!IMAGE_ENABLE_OF_LIBFDT)
			gd->bd->bi_boot_params = ft_addr;
	}
}
#endif

#ifdef CONFIG_OF_LIBFDT
int arch_fixup_fdt(void *blob)
{
	bd_t *bd = gd->bd;
	int bank, ret;
	u64 start[CONFIG_NR_DRAM_BANKS];
	u64 size[CONFIG_NR_DRAM_BANKS];

	for (bank = 0; bank < CONFIG_NR_DRAM_BANKS; bank++) {
		start[bank] = bd->bi_dram[bank].start;
		size[bank] = bd->bi_dram[bank].size;
#ifdef CONFIG_ARMV7_NONSEC
		ret = armv7_apply_memory_carveout(&start[bank], &size[bank]);
		if (ret)
			return ret;
#endif
	}

	ret = fdt_fixup_memory_banks(blob, start, size, CONFIG_NR_DRAM_BANKS);
#ifdef CONFIG_ARMV7_NONSEC
	if (ret)
		return ret;

	ret = psci_update_dt(blob);
#endif
	return ret;
}
#endif

#undef show_boot_progress
void show_boot_progress(int val)
{
}
