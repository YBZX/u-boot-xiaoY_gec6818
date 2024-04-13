/*
 * (C) Copyright 2015 spl
 * jhkim <jhkim@nexell.co.kr>
 *
 * SPDX-License-Identifier:      GPL-2.0+
 */

#include <common.h>
#include <config.h>
#include <spl.h>

DECLARE_GLOBAL_DATA_PTR;

#ifndef CONFIG_SPL_LIBCOMMON_SUPPORT
void puts(const char *str)
{
	while (*str)
		putc(*str++);
}

void putc(char c)
{
	serial_putc(c);
}
#endif /* CONFIG_SPL_LIBCOMMON_SUPPORT */

