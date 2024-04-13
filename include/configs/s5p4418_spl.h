/*
 * (C) Copyright 2016 Nexell
 * JungHyun, Kim <jhkim@nexell.co.kr>
 *
 * SPDX-License-Identifier:      GPL-2.0+
 */

/*-----------------------------------------------------------------------
 * SPL
 */
#ifdef CONFIG_SPL

/* Thumbs: to save space in SPL */
#ifdef CONFIG_SPL_BUILD
#ifndef CONFIG_SYS_THUMB_BUILD
#define CONFIG_SYS_THUMB_BUILD
#endif
#ifndef CONFIG_SYS_THUMB_BUILD
#define	CONFIG_SKIP_LOWLEVEL_INIT
#endif
#endif

/*
 * Build path
 */
#define CONFIG_SPL_LDSCRIPT	"arch/arm/cpu/armv7/s5p4418/spl/u-boot-spl.lds"
#define CONFIG_SPL_START_S_PATH	"arch/arm/cpu/armv7/s5p4418/spl"

/*
 * SPL System MAP
 *
 *	---------------------------
 *	| MALLOC SIZE		|
 *	--------------------------- SPL_STACK_BASE/ SYS_SPL_MALLOC_START
 *	| STACK SIZE		|
 *	--------------------------- SPL_MAX_FOOTPRINT
 *	|			|
 *	|	PGTABLE_SIZE	|
 *	|	__bss_end	|
 *	|	_start		|
 *	|			|
 *	--------------------------- SPL_TEXT_BASE
 *
 */
#define CONFIG_SPL_TEXT_BASE		0x43C00000
#define CONFIG_SPL_MAX_FOOTPRINT	0x100000 /* SPL max with PGTABLE_SIZE */
#define CONFIG_SPL_STACK_SIZE		0x100000
#define CONFIG_SPL_STACK		(CONFIG_SPL_TEXT_BASE + \
					CONFIG_SPL_MAX_FOOTPRINT +	\
					CONFIG_SPL_STACK_SIZE)

#define	CONFIG_SYS_SPL_MALLOC_START	CONFIG_SPL_STACK
#define	CONFIG_SYS_SPL_MALLOC_SIZE	0x400000

/*
 * SPL common library
 */
#define	CONFIG_SPL_LIBCOMMON_SUPPORT
#define	CONFIG_SPL_LIBGENERIC_SUPPORT

/*
 * Support Environments
 */
#ifdef CONFIG_SPL_CLI_FRAMEWORK
#define CONFIG_SPL_ENV_SUPPORT
#endif

/*
 * Support File System
 */
#define	CONFIG_SPL_EXT_SUPPORT
#define	CONFIG_SPL_LIBDISK_SUPPORT

/*
 * Support Devices
 */
#ifdef CONFIG_SPL_MMC_SUPPORT
#define CONFIG_BOUNCE_BUFFER
#endif	/* CONFIG_SPL_MMC_SUPPORT */

/*
 * undefined
 */
#ifdef CONFIG_SPL_BUILD
#undef CONFIG_DM
#undef CONFIG_OF_CONTROL
#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_PLAN9
#undef CONFIG_BOOTM_RTEMS
#undef CONFIG_BOOTM_VXWORKS
#undef CONFIG_CMD_IMI				/* image info	*/
#undef CONFIG_CMDLINE_EDITING			/* add command line history */
#undef CONFIG_CMDLINE_TAG			/* use bootargs commandline */
#undef CONFIG_CMD_FAT
#undef CONFIG_FS_FAT
#undef CONFIG_CMD_USB
#undef CONFIG_DM_USB
#endif

#ifdef CONFIG_FIT
#error "SPL not support FIT (Flattened Image Tree)"
#endif

#endif /* CONFIG_SPL */