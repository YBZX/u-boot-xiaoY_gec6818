/*
 * (C) Copyright 2016 Nexell
 * Hyunseok, Jung <hsjung@nexell.co.kr>
 *
 * SPDX-License-Identifier:      GPL-2.0+
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <linux/sizes.h>

/*-----------------------------------------------------------------------
 *  u-boot-2016.01
 */
#define CONFIG_SYS_LDSCRIPT "arch/arm/cpu/armv8/u-boot.lds"

#define	CONFIG_MACH_NXP5540

/*-----------------------------------------------------------------------
 *  System memory Configuration
 */

#define	CONFIG_SYS_TEXT_BASE			0x43C00000
/* init and run stack pointer */
#define	CONFIG_SYS_INIT_SP_ADDR			CONFIG_SYS_TEXT_BASE

/* malloc() pool */
#define	CONFIG_MEM_MALLOC_START			0x44000000
/* more than 2M for ubifs: MAX 16M */
#define CONFIG_MEM_MALLOC_LENGTH		(32*1024*1024)

/* when CONFIG_LCD */
#define CONFIG_FB_ADDR				0x46000000

/* Download OFFSET */
#define CONFIG_MEM_LOAD_ADDR			0x48000000

#define CONFIG_SYS_BOOTM_LEN    (64 << 20)      /* Increase max gunzip size */

/* AARCH64 */
#define COUNTER_FREQUENCY			200000000
#define CPU_RELEASE_ADDR			CONFIG_SYS_INIT_SP_ADDR

/*-----------------------------------------------------------------------
 *  High Level System Configuration
 */

/* Not used: not need IRQ/FIQ stuff	*/
#undef  CONFIG_USE_IRQ
/* decrementer freq: 1ms ticks */
#define CONFIG_SYS_HZ				1000

/* board_init_f */
#define	CONFIG_SYS_SDRAM_BASE			0x40000000
#define	CONFIG_SYS_SDRAM_SIZE			0x40000000

/* dram 1 bank num */
#define CONFIG_NR_DRAM_BANKS			1

/* relocate_code and  board_init_r */
#define	CONFIG_SYS_MALLOC_END			(CONFIG_MEM_MALLOC_START + \
						 CONFIG_MEM_MALLOC_LENGTH)
/* board_init_f, more than 2M for ubifs */
#define CONFIG_SYS_MALLOC_LEN \
	(CONFIG_MEM_MALLOC_LENGTH - 0x8000)

/* kernel load address */
#define CONFIG_SYS_LOAD_ADDR			CONFIG_MEM_LOAD_ADDR

/* memtest works on */
#define CONFIG_SYS_MEMTEST_START		CONFIG_SYS_MALLOC_END
#define CONFIG_SYS_MEMTEST_END			((ulong)CONFIG_SYS_SDRAM_BASE \
						 + (ulong)CONFIG_SYS_SDRAM_SIZE)

/*-----------------------------------------------------------------------
 *  System initialize options (board_init_f)
 */

/* board_init_f->init_sequence, call arch_cpu_init */
#define CONFIG_ARCH_CPU_INIT
/* board_init_f->init_sequence, call board_early_init_f */
#define	CONFIG_BOARD_EARLY_INIT_F
/* board_init_r, call board_early_init_f */
#define	CONFIG_BOARD_LATE_INIT
/* board_init_f->init_sequence, call print_cpuinfo */
#define	CONFIG_DISPLAY_CPUINFO
/* board_init_f, CONFIG_SYS_ICACHE_OFF */
#define	CONFIG_SYS_DCACHE_OFF
/* board_init_r, call arch_misc_init */
#define	CONFIG_ARCH_MISC_INIT
/*#define	CONFIG_SYS_ICACHE_OFF*/

/*-----------------------------------------------------------------------
 *	U-Boot default cmd
 */
#define	CONFIG_CMD_MEMTEST

/*-----------------------------------------------------------------------
 *	U-Boot Environments
 */
/* refer to common/env_common.c	*/
#define CONFIG_BOOTDELAY			3

/*-----------------------------------------------------------------------
 * Miscellaneous configurable options
 */
#ifdef CONFIG_SYS_PROMPT
#undef CONFIG_SYS_PROMPT
/* Monitor Command Prompt   */
#define CONFIG_SYS_PROMPT			"nxp5540_svt# "
#endif
/* undef to save memory	   */
#define CONFIG_SYS_LONGHELP
/* Console I/O Buffer Size  */
#define CONFIG_SYS_CBSIZE			1024
/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE			(CONFIG_SYS_CBSIZE + \
						 sizeof(CONFIG_SYS_PROMPT)+16)
/* max number of command args   */
#define CONFIG_SYS_MAXARGS			16
/* Boot Argument Buffer Size    */
#define CONFIG_SYS_BARGSIZE			CONFIG_SYS_CBSIZE

/*-----------------------------------------------------------------------
 * allow to overwrite serial and ethaddr
 */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_SYS_HUSH_PARSER			/* use "hush" command parser */
#ifdef CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#endif

/*-----------------------------------------------------------------------
 * Etc Command definition
 */
#define	CONFIG_CMD_IMI				/* image info	*/
#define CONFIG_CMDLINE_EDITING			/* add command line history */
#define	CONFIG_CMDLINE_TAG			/* use bootargs commandline */
#undef	CONFIG_BOOTM_NETBSD
#undef	CONFIG_BOOTM_RTEMS
#define CONFIG_INITRD_TAG

/*-----------------------------------------------------------------------
 * serial console configuration
 */
#define CONFIG_BAUDRATE				115200

/*-----------------------------------------------------------------------
 * NOR FLASH
 */
#define	CONFIG_SYS_NO_FLASH

/*-----------------------------------------------------------------------
 * Default environment organization
 */
#if !defined(CONFIG_ENV_IS_IN_MMC) && !defined(CONFIG_ENV_IS_IN_NAND) && \
	!defined(CONFIG_ENV_IS_IN_FLASH) && !defined(CONFIG_ENV_IS_IN_EEPROM)
	/* default: CONFIG_ENV_IS_NOWHERE */
	#define CONFIG_ENV_IS_NOWHERE
	#define	CONFIG_ENV_OFFSET		1024
	#define CONFIG_ENV_SIZE			(4*1024)	/* env size */
	/* imls - list all images found in flash, default enable so disable */
	#undef	CONFIG_CMD_IMLS
#endif

/*-----------------------------------------------------------------------
 * PLL
 */

#define CONFIG_SYS_PLLFIN			24000000UL

/*-----------------------------------------------------------------------
 * Timer
 */

#define CONFIG_TIMER_SYS_TICK_CH		0

/*-----------------------------------------------------------------------
 * OF_CONTROL
 */

#define CONFIG_FIT_BEST_MATCH
#define CONFIG_OF_LIBFDT

/*-----------------------------------------------------------------------
 * BOOTCOMMAND
 */

#define CONFIG_DEFAULT_CONSOLE		"console=ttySAC3,115200n8\0"

#define CONFIG_ROOT_DEV		0
#define CONFIG_BOOT_PART	1
#define CONFIG_MODULE_PART	2
#define CONFIG_ROOT_PART	3

#define CONFIG_EXTRA_ENV_SETTINGS					\
	"fdt_high=0xffffffffffffffff\0"					\
	"kerneladdr=0x40080000\0"					\
	"kernel_file=Image\0"						\
	"ramdiskaddr=0x49000000\0"					\
	"ramdisk_file=uInitrd\0"					\
	"fdtaddr=0x4a000000\0"						\
	"fdtfile=\0"							\
	"load_fdt="							\
		"if test -z \"$fdtfile\"; then "			\
		"loop=$board_rev; "					\
		"success=0; "						\
		"until test $loop -eq 0 || test $success -ne 0; do "	\
			"if test $loop -lt 10; then "			\
				"number=0$loop; "			\
			"else number=$loop; "				\
			"fi; "						\
			"ext4load mmc $rootdev:$bootpart $fdtaddr nxp5540-svt.dtb && setexpr success 1; " \
			"setexpr loop $loop - 1; "			\
			"done; "					\
		"if test $success -eq 0; then "				\
			"ext4load mmc $rootdev:$bootpart $fdtaddr nxp5540-svt.dtb;"	\
		"fi; "							\
		"else ext4load mmc $rootdev:$bootpart $fdtaddr $fdtfile; "	\
		"fi;\0"							\
	"bootdelay=" __stringify(CONFIG_BOOTDELAY) "\0"			\
	"console=" CONFIG_DEFAULT_CONSOLE				\
	"consoleon=setenv console=" CONFIG_DEFAULT_CONSOLE		\
		"; saveenv; reset\0"					\
	"consoleoff=setenv console=ram; saveenv; reset\0"		\
	"rootdev=" __stringify(CONFIG_ROOT_DEV) "\0"			\
	"rootpart=" __stringify(CONFIG_ROOT_PART) "\0"			\
	"bootpart=" __stringify(CONFIG_BOOT_PART) "\0"			\
	"root_rw=rw\0"							\
	"opts=loglevel=4\0"						\
	"rootfs_type=ext4\0"						\
	"lcd1_0=s6e8fa0\0"						\
	"lcd2_0=gst7d0038\0"						\
	"lcd_panel=s6e8fa0\0"						\
	"sdrecovery=sd_recovery mmc 1:3 48000000 partmap_emmc.txt\0"	\
	"factory_load=factory_info load mmc 0 "				\
		__stringify(CONFIG_FACTORY_INFO_START) " "		\
		__stringify(CONFIG_FACTORY_INFO_SIZE) "\0"		\
	"factory_save=factory_info save mmc 0 "				\
		__stringify(CONFIG_FACTORY_INFO_START) " "		\
		__stringify(CONFIG_FACTORY_INFO_SIZE) "\0"		\
	"factory_set_ethaddr=run factory_load; gen_eth_addr ;"		\
		"factory_info write ethaddr $ethaddr;"			\
		"run factory_save\0"					\
	"load_args=run factory_load; setenv bootargs ${console} "	\
		"root=/dev/mmcblk${rootdev}p${rootpart} ${root_rw} "	\
		"rootfstype=${rootfs_type} ${opts} ${recoverymode} "	\
		"drm_panel=$lcd_panel\0"				\
	"load_kernel=ext4load mmc ${rootdev}:${bootpart} $kerneladdr $kernel_file\0" \
	"load_initrd=ext4load mmc ${rootdev}:${bootpart} $ramdiskaddr $ramdisk_file\0" \
	"boot_cmd_initrd="						\
		"run load_fdt; run load_kernel; run load_initrd;"	\
		"booti $kerneladdr $ramdiskaddr $fdtaddr\0"		\
	"boot_cmd_mmcboot="						\
		"run load_fdt; run load_kernel;"			\
		"booti $kerneladdr - $fdtaddr\0"			\
	"ramfsboot=run load_args; run boot_cmd_initrd\0"		\
	"mmcboot=run load_args; run boot_cmd_mmcboot\0"			\
	"recovery_cmd=run sdrecovery; setenv recoverymode recovery\0"	\
	"recoveryboot=run recovery_cmd; run ramfsboot\0"		\
	"bootcmd=run ramfsboot\0"

#endif /* __CONFIG_H__ */
