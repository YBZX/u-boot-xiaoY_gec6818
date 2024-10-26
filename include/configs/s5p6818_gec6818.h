/*
 * Copyright (C) Guangzhou FriendlyARM Computer Tech. Co., Ltd.
 * (http://www.friendlyarm.com)
 *
 * (C) Copyright 2016 Nexell
 * Hyejung Kwon <cjscld15@nexell.co.kr>
 *
 * SPDX-License-Identifier:      GPL-2.0+
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <linux/sizes.h>

/*-----------------------------------------------------------------------
 *  u-boot-2016.01
 */
#define CONFIG_SYS_LDSCRIPT	"arch/arm/cpu/armv8/u-boot.lds"

#define CONFIG_MACH_S5P6818	1

/*-----------------------------------------------------------------------
 *  System memory Configuration
 */

#define CONFIG_SYS_TEXT_BASE		0x43C00000
#define CONFIG_SYS_INIT_SP_ADDR		CONFIG_SYS_TEXT_BASE
#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_TEXT_BASE

#define CONFIG_SYS_MEM_SIZE		0x40000000
#define CONFIG_SYS_RESERVE_MEM_SIZE	0x02500000 /* 37MB */
#define CONFIG_SYS_SDRAM_BASE		0x40000000
#define CONFIG_SYS_SDRAM_SIZE		(CONFIG_SYS_MEM_SIZE - CONFIG_SYS_RESERVE_MEM_SIZE)

#define CONFIG_SYS_MALLOC_LEN		(32*1024*1024)

/* when CONFIG_LCD */
#define CONFIG_FB_ADDR			0x46000000
#define CONFIG_BMP_LOAD_ADDR		0x47000000

/* dram 2 bank num */
#define CONFIG_NR_DRAM_BANKS		2

/* kernel load address */
#define CONFIG_SYS_LOAD_ADDR		0x48000000
#define CONFIG_KERNEL_DTB_ADDR		0x49000000

/* fastboot buffer start, size */
#define CONFIG_FASTBOOT_BUF_ADDR	0x4A000000
#define CONFIG_FASTBOOT_BUF_SIZE	(CONFIG_SYS_MEM_SIZE - 0x10000000)

/* AARCH64 */
#define COUNTER_FREQUENCY		200000000
#define CPU_RELEASE_ADDR		CONFIG_SYS_INIT_SP_ADDR

/* memtest works on */
#define CONFIG_SYS_MEMTEST_START	CONFIG_SYS_SDRAM_BASE
#define CONFIG_SYS_MEMTEST_END		((ulong)CONFIG_SYS_SDRAM_BASE + \
					 (ulong)CONFIG_SYS_SDRAM_SIZE)
/*-----------------------------------------------------------------------
 *  High Level System Configuration
 */

/* Not used: not need IRQ/FIQ stuff */
#undef  CONFIG_USE_IRQ
/* decrementer freq: 1ms ticks */
#define CONFIG_SYS_HZ			1000

/*-----------------------------------------------------------------------
 *  System initialize options (board_init_f)
 */

/* board_init_f->init_sequence, call arch_cpu_init */
#define CONFIG_ARCH_CPU_INIT
/* board_init_f->init_sequence, call board_early_init_f */
#define CONFIG_BOARD_LATE_INIT
/* board_init_f->init_sequence, call print_cpuinfo */
#define CONFIG_DISPLAY_CPUINFO
/* board_init_f->init_sequence, call show_board_info */
#define CONFIG_DISPLAY_BOARDINFO
/* board_init_f, CONFIG_SYS_ICACHE_OFF */
#define CONFIG_SYS_DCACHE_OFF
/* board_init_r, call arch_misc_init */
#define CONFIG_ARCH_MISC_INIT
/* #define CONFIG_SYS_ICACHE_OFF */

/*-----------------------------------------------------------------------
 *	U-Boot default cmd
 */
#define CONFIG_CMD_MEMTEST

/*-----------------------------------------------------------------------
 *	U-Boot Environments
 */
/* refer to common/env_common.c */
#define CONFIG_BOOTDELAY		1
#define CONFIG_ZERO_BOOTDELAY_CHECK

/*-----------------------------------------------------------------------
 * Miscellaneous configurable options
 */
#ifdef CONFIG_SYS_PROMPT
#undef CONFIG_SYS_PROMPT
/* Monitor Command Prompt */
#define CONFIG_SYS_PROMPT		"nanopi3# "
#endif
/* undef to save memory */
#define CONFIG_SYS_LONGHELP
/* Console I/O Buffer Size */
#define CONFIG_SYS_CBSIZE		1024
/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
/* max number of command args */
#define CONFIG_SYS_MAXARGS		16
/* Boot Argument Buffer Size */
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE

/*-----------------------------------------------------------------------
 * allow to overwrite serial and ethaddr
 */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_SYS_HUSH_PARSER		/* use "hush" command parser */
#ifdef CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#endif

/*-----------------------------------------------------------------------
 * Etc Command definition
 */
#define CONFIG_CMD_IMI			/* image info */
#define CONFIG_CMDLINE_EDITING		/* add command line history */
#define CONFIG_CMDLINE_TAG		/* use bootargs commandline */
#define CONFIG_INITRD_TAG
#define CONFIG_SUPPORT_RAW_INITRD	1
#define CONFIG_REVISION_TAG
#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_RTEMS

/*-----------------------------------------------------------------------
 * serial console configuration
 */
#define CONFIG_S5P_SERIAL
#define CONFIG_S5P_SERIAL_INDEX		0
#define CONFIG_S5P_SERIAL_CLOCK		50000000
#define CONFIG_S5P_SERIAL_PORT		(void *)PHY_BASEADDR_UART0

#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE \
		{ 9600, 19200, 38400, 57600, 115200 }
#define CONFIG_S5P_SERIAL_FLUSH_ON_INIT

#define CONFIG_UART_CLKGEN_CLOCK_HZ	CONFIG_S5P_SERIAL_CLOCK

/*-----------------------------------------------------------------------
 * NOR FLASH
 */
#define CONFIG_SYS_NO_FLASH

/*-----------------------------------------------------------------------
 * PLL
 */
#define CONFIG_SYS_PLLFIN		24000000UL

/*-----------------------------------------------------------------------
 * Timer
 */
#define CONFIG_TIMER_SYS_TICK_CH	0

/*-----------------------------------------------------------------------
 * PWM
 */
#define CONFIG_PWM_NX

/*-----------------------------------------------------------------------
 * BACKLIGHT
 */
#define CONFIG_ONEWIRE			1

#ifndef CONFIG_ONEWIRE
/* fallback to pwm */
#define CONFIG_BACKLIGHT_CH		0
#define CONFIG_BACKLIGHT_DIV		0
#define CONFIG_BACKLIGHT_INV		0
#define CONFIG_BACKLIGHT_DUTY		50
#define CONFIG_BACKLIGHT_HZ		1000
#endif

/*-----------------------------------------------------------------------
 * SD/MMC
 */
#define CONFIG_GENERIC_MMC
#define CONFIG_MMC
#define CONFIG_DWMMC
#define CONFIG_NEXELL_DWMMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_MMC

#if defined(CONFIG_MMC)
#define CONFIG_SYS_MMC_DEV		0
#define CONFIG_2NDBOOT_OFFSET		512
#define CONFIG_2NDBOOT_SIZE		(64*1024)
#define CONFIG_FIP_OFFSET		(CONFIG_2NDBOOT_OFFSET + CONFIG_2NDBOOT_SIZE)
#define CONFIG_FIP_SIZE			(2880*1024)
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV		CONFIG_SYS_MMC_DEV
#define CONFIG_ENV_OFFSET		(CONFIG_FIP_OFFSET + CONFIG_FIP_SIZE)
#define CONFIG_ENV_SIZE			(16*1024)	/* env size */

#define CONFIG_DOS_PARTITION
#define CONFIG_CMD_FAT
#define CONFIG_FS_FAT
#define CONFIG_FAT_WRITE

#define CONFIG_CMD_EXT4
#define CONFIG_CMD_EXT4_WRITE
#define CONFIG_FS_EXT4
#define CONFIG_EXT4_WRITE
#endif

/*-----------------------------------------------------------------------
 * Default environment organization
 */
#if !defined(CONFIG_ENV_IS_IN_MMC) && !defined(CONFIG_ENV_IS_IN_NAND) && \
	!defined(CONFIG_ENV_IS_IN_FLASH) && !defined(CONFIG_ENV_IS_IN_EEPROM)
	/* default: CONFIG_ENV_IS_NOWHERE */
	#define CONFIG_ENV_IS_NOWHERE
	#define CONFIG_ENV_OFFSET	1024
	#define CONFIG_ENV_SIZE		(4*1024)	/* env size */
	/* imls - list all images found in flash, default enable so disable */
	#undef CONFIG_CMD_IMLS
#endif

/*-----------------------------------------------------------------------
 * GPT
 */
#define CONFIG_CMD_GPT
#define CONFIG_EFI_PARTITION
#define CONFIG_PARTITION_UUIDS
#define CONFIG_RANDOM_UUID

/*-----------------------------------------------------------------------
 * Fastboot and USB OTG
 */
#define CONFIG_USB_FUNCTION_FASTBOOT
#define CONFIG_CMD_FASTBOOT
#define CONFIG_FASTBOOT_FLASH
#define CONFIG_FASTBOOT_FLASH_MMC_DEV	CONFIG_SYS_MMC_DEV
#define CONFIG_USB_GADGET
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USB_GADGET_VBUS_DRAW	0
#define CONFIG_USB_GADGET_DWC2_OTG
#define CONFIG_USB_GADGET_NX_UDC_OTG_PHY
#define CONFIG_USB_GADGET_DOWNLOAD
#define CONFIG_SYS_CACHELINE_SIZE	64
#define CONFIG_G_DNL_VENDOR_NUM		0x18d1  /* google */
#define CONFIG_G_DNL_PRODUCT_NUM	0x0002  /* nexus one */
#define CONFIG_G_DNL_MANUFACTURER	"Nexell Corporation"

/*-----------------------------------------------------------------------
 * Nexell USB Downloader
 */
#define CONFIG_NX_USBDOWN

/*-----------------------------------------------------------------------
 * OF_CONTROL
 */
#define CONFIG_FIT_BEST_MATCH
#define CONFIG_OF_LIBFDT
#define CONFIG_OF_BOARD_SETUP

/*-----------------------------------------------------------------------
 * VIDEO
 */
#define CONFIG_VIDEO
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV

#define CONFIG_VIDEO_LOGO
#define CONFIG_SPLASH_SCREEN

#ifdef CONFIG_VIDEO_LOGO
#define CONFIG_CMD_BMP
#ifdef CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SOURCE		1
#define CONFIG_SPLASH_SCREEN_ALIGN	1
#define CONFIG_SPLASH_FILE		"logo.bmp"
#endif
#endif

/*-----------------------------------------------------------------------
 * BOOTCOMMAND
  */
#define CONFIG_ROOT_DEV			CONFIG_SYS_MMC_DEV
#define CONFIG_BOOT_PART		1
#define CONFIG_ROOT_PART		2

/*-----------------------------------------------------------------------
 * ENV
 */
#define CONFIG_SYS_EXTRA_ENV_RELOC

#define CONFIG_BLOADER_MMC					\
	"ext4load mmc ${rootdev}:${bootpart} "

#ifdef CONFIG_OF_BOARD_SETUP
#define CONFIG_EXTRA_ENV_DTB_RESERVE				\
	"dtb_reserve="						\
	"if test -n \"$dtb_addr\"; then fdt addr $dtb_addr; fi\0"
#else
#define CONFIG_EXTRA_ENV_DTB_RESERVE				\
	"dtb_reserve="						\
	"if test -n \"$fb_addr\"; then "			\
	  "fdt addr $dtb_addr;"					\
	  "fdt resize;"						\
	  "fdt mk /reserved-memory display_reserved;"		\
	  "fdt set /reserved-memory/display_reserved reg <$fb_addr 0x800000>;"	\
	"fi;\0"
#endif

#define CONFIG_EXTRA_ENV_BOOT_LOGO				\
	"splashimage=" __stringify(CONFIG_BMP_LOAD_ADDR)"\0"	\
	"splashfile=" CONFIG_SPLASH_FILE "\0"			\
	"splashpos=m,m\0"					\
	"fb_addr=\0"						\
	CONFIG_EXTRA_ENV_DTB_RESERVE

#define CONFIG_EXTRA_ENV_SETTINGS				\
	"fdt_high=0xffffffffffffffff\0"				\
	"initrd_high=0xffffffffffffffff\0"			\
	"rootdev=" __stringify(CONFIG_ROOT_DEV) "\0"		\
	"rootpart=" __stringify(CONFIG_ROOT_PART) "\0"		\
	"bootpart=" __stringify(CONFIG_BOOT_PART) "\0"		\
	"kernel=Image\0"					\
	"loadaddr=0x40080000\0"					\
	"dtb_name=s5p6818-nanopi3-rev01.dtb\0"			\
	"dtb_addr=" __stringify(CONFIG_KERNEL_DTB_ADDR) "\0"	\
	"initrd_name=ramdisk.img\0"				\
	"initrd_addr=0x48000000\0"				\
	"initrd_size=0x600000\0"				\
	"load_dtb="						\
		CONFIG_BLOADER_MMC "${dtb_addr} ${dtb_name}; "	\
		"run dtb_reserve\0"				\
	"load_kernel="						\
		CONFIG_BLOADER_MMC "${loadaddr} ${kernel}\0"	\
	"load_initrd="						\
		CONFIG_BLOADER_MMC "${initrd_addr} ${initrd_name}; "	\
		"setenv initrd_size 0x${filesize}\0"		\
	"mmcboot="						\
		"run load_kernel; run load_initrd; run load_dtb; "	\
		"booti ${loadaddr} ${initrd_addr}:${initrd_size} ${dtb_addr}\0"	\
	"bootcmd=run mmcboot\0"					\
	CONFIG_EXTRA_ENV_BOOT_LOGO

#endif /* __CONFIG_H__ */
