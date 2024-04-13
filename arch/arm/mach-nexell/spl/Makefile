#
# (C) Copyright 2016 Nexell
# junghyun kim<jhkim@nexell.co.kr>
#
# SPDX-License-Identifier:      GPL-2.0+
#

ifeq ($(CONFIG_SPL_CLI_FRAMEWORK),y)
SPLLIB :=arch/arm/mach-nexell/spl
include $(SPLLIB)/config-spl.mk
endif

obj-y += spl.o serial.o spl_common.o
