#
# (C) Copyright 2016 Nexell
# junghyun kim<jhkim@nexell.co.kr>
#
# SPDX-License-Identifier:      GPL-2.0+
#
U-BOOT_TOP :=../../../..

# librarys
obj-y += $(U-BOOT_TOP)/lib/qsort.o

obj-$(CONFIG_GZIP) += $(U-BOOT_TOP)/lib/gunzip.o
obj-$(CONFIG_GZIP) += $(U-BOOT_TOP)/lib/zlib/zlib.o
obj-$(CONFIG_LMB) += $(U-BOOT_TOP)/lib/lmb.o

# FDT libs
obj-$(CONFIG_OF_LIBFDT) += $(U-BOOT_TOP)/lib/libfdt/fdt.o
obj-$(CONFIG_OF_LIBFDT) += $(U-BOOT_TOP)/lib/libfdt/fdt_ro.o
obj-$(CONFIG_OF_LIBFDT) += $(U-BOOT_TOP)/lib/libfdt/fdt_rw.o
obj-$(CONFIG_OF_LIBFDT) += $(U-BOOT_TOP)/lib/libfdt/fdt_wip.o
obj-$(CONFIG_OF_LIBFDT) += $(U-BOOT_TOP)/lib/libfdt/fdt_strerror.o
obj-$(CONFIG_OF_LIBFDT) += $(U-BOOT_TOP)/lib/libfdt/fdt_addresses.o

# commands
obj-$(CONFIG_SPL_ENV_SUPPORT) += $(U-BOOT_TOP)/common/cmd_help.o
obj-$(CONFIG_SYS_HUSH_PARSER) += $(U-BOOT_TOP)/common/cli_hush.o

obj-$(CONFIG_CMD_GO) += $(U-BOOT_TOP)/common/cmd_boot.o
obj-$(CONFIG_CMD_BOOTM) += $(U-BOOT_TOP)/common/cmd_bootm.o \
			$(U-BOOT_TOP)/common/bootm.o	\
			$(U-BOOT_TOP)/common/bootm_os.o
obj-$(CONFIG_CMD_BOOTM) += $(U-BOOT_TOP)/arch/arm/lib/bootm.o

obj-$(CONFIG_CMD_BOOTL) += $(U-BOOT_TOP)/arch/arm/lib/bootm.o

obj-$(CONFIG_CMD_FS_GENERIC) += $(U-BOOT_TOP)/common/cmd_fs.o
obj-$(CONFIG_CMD_EXT4) += $(U-BOOT_TOP)/common/cmd_ext4.o
obj-$(CONFIG_CMD_EXT4) += $(U-BOOT_TOP)/fs/fs.o
obj-$(CONFIG_CMD_FASTBOOT) += $(U-BOOT_TOP)/common/cmd_fastboot.o

obj-$(CONFIG_NX_USBDOWN) += $(U-BOOT_TOP)/common/cmd_usbdown.o
obj-$(CONFIG_CMD_MMC) += $(U-BOOT_TOP)/common/cmd_mmc.o

# FDT commands
obj-$(CONFIG_OF_LIBFDT) += $(U-BOOT_TOP)/common/cmd_fdt.o
obj-$(CONFIG_OF_LIBFDT) += $(U-BOOT_TOP)/common/fdt_support.o

# devices
obj-$(CONFIG_USB_GADGET) += $(U-BOOT_TOP)/drivers/usb/gadget/
obj-$(CONFIG_GENERIC_MMC) += $(U-BOOT_TOP)/drivers/mmc/mmc_write.o
obj-$(CONFIG_VIDEO) += $(U-BOOT_TOP)/drivers/video/
obj-$(CONFIG_CFB_CONSOLE) += $(U-BOOT_TOP)/common/cmd_version.o
obj-$(CONFIG_SPLASH_SOURCE) += $(U-BOOT_TOP)/common/splash_source.o
