/*
 * (C) Copyright 2017 FMS
 * Han Yibeom <yibhan@cafeel.co.kr>
 *
 * SPDX-License-Identifier:      GPL-2.0+
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <linux/sizes.h>
#include <asm/arch/nexell.h>
/*-----------------------------------------------------------------------
 *  u-boot-2016.01
 */
#define CONFIG_SYS_LDSCRIPT "arch/arm/cpu/armv7/s5p4418/u-boot.lds"

#define CONFIG_MACH_S5P4418

/*-----------------------------------------------------------------------
 *  System memory Configuration
 */
/*2GB*/
/*#define	CONFIG_SYS_TEXT_BASE			0x43C00000*/
/*512MB*/
/*#define	CONFIG_SYS_TEXT_BASE			0x94C00000*/
/*1GB*/
#define CONFIG_SYS_TEXT_BASE 0x74C00000
#define CONFIG_SYS_INIT_SP_ADDR CONFIG_SYS_TEXT_BASE
#define CONFIG_SYS_MONITOR_BASE CONFIG_SYS_TEXT_BASE

/*2GB*/
/*
#define	CONFIG_SYS_SDRAM_BASE			0x40000000
#define	CONFIG_SYS_SDRAM_SIZE			0x70000000
*/
/*512MB*/
/*
#define	CONFIG_SYS_SDRAM_BASE			0x95000000
#define	CONFIG_SYS_SDRAM_SIZE			0x1A000000
*/
/*1GB*/
#define CONFIG_SYS_SDRAM_BASE 0x71000000
#define CONFIG_SYS_SDRAM_SIZE 0x3E000000

#define CONFIG_SYS_MALLOC_LEN (32 * 1024 * 1024)

/* fastboot buffer start, size */
#define CONFIG_FASTBOOT_BUF_ADDR CONFIG_SYS_SDRAM_BASE
#define CONFIG_FASTBOOT_BUF_SIZE 0x20000000
/* align buffer is used by ext4_mmc_write for unaligned data */
#define CONFIG_ALIGNBUFFER_SIZE 0x02000000

/* when CONFIG_LCD */
#define CONFIG_FB_ADDR                                                         \
	(CONFIG_FASTBOOT_BUF_ADDR + CONFIG_FASTBOOT_BUF_SIZE +                 \
	 CONFIG_ALIGNBUFFER_SIZE)

/* dram 1 bank num */
#define CONFIG_NR_DRAM_BANKS 1

/* kernel load address */
/*2GB*/
/*
#define CONFIG_SYS_LOAD_ADDR			0x48000000
*/
/*512MB*/
/*#define CONFIG_SYS_LOAD_ADDR			0x99000000*/
/*1GB*/
#define CONFIG_SYS_LOAD_ADDR 0x79000000

/*-----------------------------------------------------------------------
 *  High Level System Configuration
 */

/* Not used: not need IRQ/FIQ stuff	*/
#undef CONFIG_USE_IRQ
/* decrementer freq: 1ms ticks */
#define CONFIG_SYS_HZ 1000

/* memtest works on */
#define CONFIG_SYS_MEMTEST_START CONFIG_SYS_SDRAM_BASE
#define CONFIG_SYS_MEMTEST_END                                                 \
	((ulong)CONFIG_SYS_SDRAM_BASE + (ulong)CONFIG_SYS_SDRAM_SIZE)

/*-----------------------------------------------------------------------
 *  System initialize options (board_init_f)
 */

/* board_init_f->init_sequence, call arch_cpu_init */
#define CONFIG_ARCH_CPU_INIT
/* board_init_f->init_sequence, call board_early_init_f */
/* #define	CONFIG_BOARD_EARLY_INIT_F  */
/* board_init_r, call board_early_init_f */
#define CONFIG_BOARD_LATE_INIT
/* board_init_f->init_sequence, call print_cpuinfo */
#define CONFIG_DISPLAY_CPUINFO
/* board_init_f, CONFIG_SYS_ICACHE_OFF */
#define CONFIG_SYS_DCACHE_OFF
/* board_init_r, call arch_misc_init */
#define CONFIG_ARCH_MISC_INIT
/*#define	CONFIG_SYS_ICACHE_OFF*/

/*-----------------------------------------------------------------------
 *	U-Boot default cmd
 */
#define CONFIG_CMD_MEMTEST

/*-----------------------------------------------------------------------
 *	U-Boot Environments
 */
/* refer to common/env_common.c	*/
#define CONFIG_BOOTDELAY 0
#define CONFIG_ZERO_BOOTDELAY_CHECK
/*-----------------------------------------------------------------------
 * Miscellaneous configurable options
 */
#ifdef CONFIG_SYS_PROMPT
#undef CONFIG_SYS_PROMPT
/* Monitor Command Prompt   */
#define CONFIG_SYS_PROMPT "s5p4418_daudio_covi# "
#endif
/* undef to save memory	   */
#define CONFIG_SYS_LONGHELP
/* Console I/O Buffer Size  */
#define CONFIG_SYS_CBSIZE 1024
/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
/* max number of command args   */
#define CONFIG_SYS_MAXARGS 16
/* Boot Argument Buffer Size    */
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

/*-----------------------------------------------------------------------
 * allow to overwrite serial and ethaddr
 */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_SYS_HUSH_PARSER /* use "hush" command parser */
#ifdef CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2 "> "
#endif

/*-----------------------------------------------------------------------
 * Etc Command definition
 */
#define CONFIG_CMD_IMI	 /* image info	*/
#define CONFIG_CMDLINE_EDITING /* add command line history */
#define CONFIG_CMDLINE_TAG     /* use bootargs commandline */
#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_RTEMS

/*-----------------------------------------------------------------------
 * serial console configuration
 */
#define CONFIG_PL011_SERIAL
#define CONFIG_CONS_INDEX 3
#define CONFIG_PL011_CLOCK 50000000
#define CONFIG_PL01x_PORTS                                                     \
	{                                                                      \
		(void *) PHY_BASEADDR_UART0, (void *)PHY_BASEADDR_UART1,       \
		    (void *)PHY_BASEADDR_UART2, (void *)PHY_BASEADDR_UART3     \
	}

#define CONFIG_BAUDRATE 115200
#define CONFIG_SYS_BAUDRATE_TABLE                                              \
	{                                                                      \
		9600, 19200, 38400, 57600, 115200                              \
	}
#define CONFIG_PL011_SERIAL_FLUSH_ON_INIT

/*-----------------------------------------------------------------------
 * NOR FLASH
 */
#define CONFIG_SYS_NO_FLASH

/*-----------------------------------------------------------------------
 * PLL
 */
#define CONFIG_SYS_PLLFIN 24000000UL

/*-----------------------------------------------------------------------
 * Timer
 */
#define CONFIG_TIMER_SYS_TICK_CH 0

/*-----------------------------------------------------------------------
 * PWM
 */
#define CONFIG_PWM_NX

/*-----------------------------------------------------------------------
 * BACKLIGHT
 */
#define CONFIG_BACKLIGHT_CH 0
#define CONFIG_BACKLIGHT_DIV 0
#define CONFIG_BACKLIGHT_INV 0
#define CONFIG_BACKLIGHT_DUTY 50
#define CONFIG_BACKLIGHT_HZ 1000

/*-----------------------------------------------------------------------
 * SD/MMC
 */
#define CONFIG_GENERIC_MMC
#define CONFIG_MMC
#define CONFIG_DWMMC
#define CONFIG_NEXELL_DWMMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_MMC
#define CONFIG_CMD_BOOTZ

#if defined(CONFIG_MMC)
#define CONFIG_2NDBOOT_OFFSET 512
#define CONFIG_2NDBOOT_SIZE (64 * 1024)
#define CONFIG_FIP_OFFSET (CONFIG_2NDBOOT_OFFSET + CONFIG_2NDBOOT_SIZE)
#define CONFIG_FIP_SIZE (3 * 1024 * 1024)
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV 0
#define CONFIG_ENV_OFFSET (0x2E0200)
#define CONFIG_ENV_SIZE (0x4000) /* env size */
#endif

#if defined(CONFIG_MMC)
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
#if !defined(CONFIG_ENV_IS_IN_MMC) && !defined(CONFIG_ENV_IS_IN_NAND) &&       \
    !defined(CONFIG_ENV_IS_IN_FLASH) && !defined(CONFIG_ENV_IS_IN_EEPROM)
/* default: CONFIG_ENV_IS_NOWHERE */
#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_OFFSET 1024
#define CONFIG_ENV_SIZE (4 * 1024) /* env size */
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
#define CONFIG_FASTBOOT_FLASH_MMC_DEV 0
#define CONFIG_USB_GADGET
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USB_GADGET_VBUS_DRAW 0
#define CONFIG_USB_GADGET_DWC2_OTG
#define CONFIG_USB_GADGET_NX_UDC_OTG_PHY
#define CONFIG_USB_GADGET_DOWNLOAD
#define CONFIG_SYS_CACHELINE_SIZE 64
#define CONFIG_G_DNL_VENDOR_NUM 0x18d1  /* google */
#define CONFIG_G_DNL_PRODUCT_NUM 0x0002 /* nexus one */
#define CONFIG_G_DNL_MANUFACTURER "Nexell Corporation"

/*-----------------------------------------------------------------------
 * Nexell USB Downloader
 */
#define CONFIG_NX_USBDOWN

/*-----------------------------------------------------------------------
 * OF_CONTROL
 */
#define CONFIG_FIT_BEST_MATCH
#define CONFIG_OF_LIBFDT

/*-----------------------------------------------------------------------
 * VIDEO
 */
#define CONFIG_VIDEO
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV

/* Support bootlogo */
#define CONFIG_VIDEO_LOGO
#undef	CONFIG_VIDEO_LOGO
#define CONFIG_SPLASH_SCREEN
#undef	CONFIG_SPLASH_SCREEN

/* Support bootanimation */
#define CONFIG_BOOTANIM

#ifdef CONFIG_VIDEO_LOGO
#define CONFIG_CMD_BMP
#ifdef CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SOURCE
#define CONFIG_SPLASH_MMC_OFFSET    0x2e4200
#endif
#endif

/*-----------------------------------------------------------------------
 * Support recovery boot
*/
#define CONFIG_SUPPORT_RAW_INITRD
#define CONFIG_RECOVERY_BOOT

/*-----------------------------------------------------------------------
 * ENV
 */
#define CONFIG_ROOT_DEV 0
#define CONFIG_BOOT_PART 1

#define CONFIG_KERNEL_DTB_ADDR  0x7A000000
#define CONFIG_BMP_LOAD_ADDR    0xA1000000

/* need to relocate env address */
#define CONFIG_SYS_EXTRA_ENV_RELOC

#define CONFIG_EXTRA_ENV_BOOT_LOGO \
    "splashimage=" __stringify(CONFIG_BMP_LOAD_ADDR)"\0" \
    "splashfile=logo.bmp\0" \
    "splashsource=mmc_fs\0" \
    "splashoffset=" __stringify(CONFIG_SPLASH_MMC_OFFSET)"\0" \
    "fb_addr=\0" \
    "dtb_reserve=" \
        "if test -n \"$fb_addr\"; then " \
        "fdt addr " __stringify(CONFIG_KERNEL_DTB_ADDR)"; " \
        "fdt resize; " \
        "fdt mk /reserved-memory display_reserved; " \
        "fdt set /reserved-memory/display_reserved reg <$fb_addr 0x300000>; " \
        "fi;\0"

#define CONFIG_RECOVERY_BOOT_CMD \
    "recoveryboot=run ramfsboot\0"

#ifdef CONFIG_BOOTANIM
#define CONFIG_BOOTCMD "bootargs=console=ttyAMA3,115200n8 root=/dev/mmcblk0p3 rw rootfstype=ext4 init=/usr/bin/bootanimation loglevel=4 rootwait quiet " \
 			         	"printk.time=1 consoleblank=0 nx_drm.fb_buffers=3 systemd.log_level=info systemd.show_status=false\0"
#else
#define CONFIG_BOOTCMD "bootargs=console=ttyAMA3,115200n8 root=/dev/mmcblk0p3 rw rootfstype=ext4 loglevel=4 rootwait quiet " \
				        "printk.time=1 consoleblank=0 nx_drm.fb_buffers=3 systemd.log_level=info systemd.show_status=false\0"
#endif

#define CONFIG_EXTRA_ENV_SETTINGS                                              \
	"fdt_high=0xffffffff\0"                                                \
	"initrd_high=0xffffffff\0"                                             \
	"kerneladdr=0x71008000\0"                                              \
	"kernel_file=zImage\0"                                                 \
	"fdtaddr=0x7A000000\0"                                                 \
	"load_fdt="                                                            \
	"if test -z \"$fdtfile\"; then "                                       \
	"loop=$board_rev; "                                                    \
	"number=$board_rev: "                                                  \
	"success=0; "                                                          \
	"until test $loop -eq 0 || test $success -ne 0; do "                   \
	"if test $loop -lt 10; then "                                          \
	"number=0$loop; "                                                      \
	"else number=$loop; "                                                  \
	"fi; "                                                                 \
	"ext4load mmc $rootdev:$bootpart $fdtaddr "                            \
	"s5p4418-daudio_covi-rev${number}.dtb && setexpr success 1; "          \
	"setexpr loop $loop - 1; "                                             \
	"done; "                                                               \
	"if test $success -eq 0; then "                                        \
	"ext4load mmc $rootdev:$bootpart $fdtaddr "                            \
	"s5p4418-daudio_covi-rev00.dtb;"                                       \
	"fi; "                                                                 \
	"else ext4load mmc $rootdev:$bootpart $fdtaddr $fdtfile; "             \
	"fi; \0"                                                               \
	"rootdev=" __stringify(                                                \
	    CONFIG_ROOT_DEV) "\0"                                              \
			     "bootpart=" __stringify(                          \
				 CONFIG_BOOT_PART) "\0"                        \
						   CONFIG_BOOTCMD			   \
						   "boot_cmd_mmcboot="         \
						   "check_hw;ext4load mmc "    \
						   "${rootdev}:${bootpart} "   \
						   "$kerneladdr "              \
						   "$kernel_file;run "         \
						   "load_fdt;"                 \
						   "run dtb_reserve;"          \
						   "bootz $kerneladdr - "      \
						   "$fdtaddr\0"                \
						   "mmcboot=run "              \
						   "boot_cmd_mmcboot \0"       \
						   "boot_cmd_ramfsboot="       \
						   "ext4load mmc 0:1 "         \
						   "0x71008000 zImage; "       \
						   "ext4load mmc 0:1 "         \
						   "0x79000000 uInitrd; "      \
						   "ext4load mmc 0:1 "         \
						   "0x7A000000 "               \
						   "s5p4418-daudio_covi-"      \
						   "rev00.dtb; "               \
						   "run dtb_reserve; "         \
						   "bootz 0x71008000 "         \
						   "0x79000000 0x7A000000\0"   \
						   "ramfsboot=setenv "         \
						   "bootargs "                 \
						   "console=ttyAMA3,115200n8 " \
						   "root=/dev/ram0 "           \
						   "loglevel=4 printk.time=1 " \
						   "consoleblank=0 "           \
						   "nx_drm.fb_buffers=3; "     \
						   "run boot_cmd_ramfsboot \0" \
						   "bootcmd=run mmcboot\0"	   \
						   CONFIG_RECOVERY_BOOT_CMD	   \
						   CONFIG_EXTRA_ENV_BOOT_LOGO

#endif /* __CONFIG_H__ */
