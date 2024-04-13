/*
 * (C) Copyright 2016
 * junghyun kim<jhkim@nexell.co.kr>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <config.h>
#include <asm-offsets.h>
#include <linux/linkage.h>

/*
 * entry point of crt0 sequence
 */

ENTRY(_s_main)

/*
 * Set up initial C runtime environment and call board_init_f(0).
 */

	ldr	sp, =(CONFIG_SPL_STACK)
	bic	sp, sp, #7	/* 8-byte alignment for ABI compliance */

	mov	r0, sp
	mov	r0, #0
	bl	board_init_f

/*
 * Set up intermediate environment (new sp and gd) and call
 */
	ldr	sp, [r9, #GD_START_ADDR_SP]	/* sp = gd->start_addr_sp */
	bic	sp, sp, #7	/* 8-byte alignment for ABI compliance */

	/* call board_init_r */
	ldr	pc, =spl_board_init_r

	/* we should not return here. */
ENDPROC(_s_main)
