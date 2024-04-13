/*
 * (C) Copyright 2015 spl
 * jhkim <jhkim@nexell.co.kr>
 *
 * SPDX-License-Identifier:      GPL-2.0+
 */

#include <common.h>
#include <config.h>
#include <spl.h>
#include <image.h>
#include <linux/compiler.h>
#include <version.h>
#include <initcall.h>
#include <cli.h>
#include <malloc.h>
#include <mapmem.h>
#include <environment.h>
#include <console.h>
#include <serial.h>
#include <mmc.h>

DECLARE_GLOBAL_DATA_PTR;

static gd_t _gdata __attribute__ ((section(".data")));
static bd_t _bdata __attribute__ ((section(".data")));

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_FRAMEWORK)
u32 spl_boot_device(void)
{
	return CONFIG_SPL_BOOT_DEVICE;
}

u32 spl_boot_mode(void)
{
	return CONFIG_SPL_BOOT_MODE;
}
#endif

static inline int spl_cpu_init(void)
{
	arch_cpu_init();
	return 0;
}

static inline int spl_stack_relocate(void)
{
	ulong ptr;

	/* board data: under stack */
	ptr = CONFIG_SPL_STACK;
	ptr &= ~7;

	gd->start_addr_sp = ptr;

	return 0;
}

static inline int spl_global_data(void)
{
	/* align 64 Kbyte */
	ulong ptr = ALIGN((ulong)__bss_end, 0x10000);

	/* MMU table */
	gd->arch.tlb_addr = ptr;
	gd->arch.tlb_size = PGTABLE_SIZE;

	/* Memory table */
	gd->bd->bi_arch_number = machine_arch_type;
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x00000100;
	gd->bd->bi_dram[0].start = CONFIG_SYS_SDRAM_BASE;
	gd->bd->bi_dram[0].size = CONFIG_SYS_SDRAM_SIZE;

	return 0;
}

#define REG_GPIO_SLEW_DISABLE      0x44
#define REG_GPIO_DRV1_DISABLE      0x4C
#define REG_GPIO_DRV0_DISABLE      0x54
#define REG_GPIO_PULLSEL_DISABLE   0x5C
#define REG_GPIO_PULLENB_DISABLE   0x64

static inline int spl_gpio_init(void)
{
	unsigned long base[5] = {
		PHY_BASEADDR_GPIOA, PHY_BASEADDR_GPIOB,
		PHY_BASEADDR_GPIOC, PHY_BASEADDR_GPIOD,
		PHY_BASEADDR_GPIOE,
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(base); i++) {
		writel(0xFFFFFFFF, base[i] + REG_GPIO_SLEW_DISABLE);
		writel(0xFFFFFFFF, base[i] + REG_GPIO_DRV1_DISABLE);
		writel(0xFFFFFFFF, base[i] + REG_GPIO_DRV0_DISABLE);
		writel(0xFFFFFFFF, base[i] + REG_GPIO_PULLSEL_DISABLE);
		writel(0xFFFFFFFF, base[i] + REG_GPIO_PULLENB_DISABLE);
	}

	return 0;
}

#if defined(CONFIG_SPL_SERIAL_SUPPORT)
static inline int spl_console_init(void)
{
	gd->baudrate = CONFIG_BAUDRATE;
	gd->flags |= GD_FLG_SERIAL_READY;

	default_serial_console()->start();

	gd->have_console = 1;

	puts("\nU-Boot SPL " PLAIN_VERSION " (" U_BOOT_DATE " - "
	     U_BOOT_TIME ")\n");

	return 0;
}
#endif

static init_fnc_t init_sequence_f[] = {
	spl_cpu_init,
	spl_gpio_init,
	spl_global_data,
#if defined(CONFIG_SPL_SERIAL_SUPPORT)
	spl_console_init,
#endif
	NULL,
};

#if defined(CONFIG_SPL_CLI_FRAMEWORK)
static inline int spl_enable_caches(void)
{
	/* enable mmu */
	debug("tbl base:0x%lx, size:%ld\n",
	      gd->arch.tlb_addr, gd->arch.tlb_size);

	dcache_enable();
	return 0;
}

static inline int spl_malloc_init(void)
{
#if defined(CONFIG_SPL_SYS_MALLOC_SIMPLE)
	if (CONFIG_SYS_SPL_MALLOC_SIZE) {
		gd->malloc_base = CONFIG_SYS_SPL_MALLOC_START;
		gd->malloc_limit = CONFIG_SYS_SPL_MALLOC_SIZE;
		gd->malloc_ptr = 0;
	}

	debug("malloc start 0x%lx, size 0x%lx bss end:%p\n",
	      gd->malloc_base, gd->malloc_limit, __bss_end);

#else
	mem_malloc_init(CONFIG_SYS_SPL_MALLOC_START,
			CONFIG_SYS_SPL_MALLOC_SIZE);
	gd->flags |= GD_FLG_FULL_MALLOC_INIT;
#endif
	return 0;
}

#if defined(CONFIG_SPL_ENV_SUPPORT)
static inline int spl_env_init(void)
{
	env_init();

#if defined(CONFIG_ENV_IS_NOWHERE)
	set_default_env(NULL);
#else
	env_relocate_spec();
#endif

	return 0;
}
#endif

#if defined(CONFIG_SPL_MMC_SUPPORT)
static inline int spl_mmc_init(void)
{
	return mmc_initialize(gd->bd);
}
#endif

#if !defined CONFIG_OF_CONTROL && !defined CONFIG_DM_USB &&	\
	 defined CONFIG_USB_GADGET
#include <usb/dwc2_udc.h>

static struct dwc2_plat_otg_data
	nx_otg_data = CONFIG_USB_GADGET_REGS;

static inline int spl_udc_probe(void)
{
	dwc2_udc_probe(&nx_otg_data);
	return 0;
}
#endif

#if defined CONFIG_VIDEO
#include <stdio_dev.h>
#include <asm/arch/display.h>

__weak int spl_display_probe(void)
{
	return 0;
}

static int spl_video_probe(void)
{
	drv_video_init();
	return 0;
}
#endif

static int spl_bootdelay(void)
{
	int abort = 0;
	unsigned long ts;

	char *s = getenv("bootdelay");
	int bootdelay = s ? (int)simple_strtol(s, NULL, 10) : CONFIG_BOOTDELAY;

	if (bootdelay >= 0)
		printf("Hit any key to stop autoboot: %2d ", bootdelay);

#if defined CONFIG_ZERO_BOOTDELAY_CHECK
	/*
	 * Check if key already pressed
	 * Don't check if bootdelay < 0
	 */
	if (bootdelay >= 0) {
		if (tstc()) {	/* we got a key press   */
			(void)getc();	/* consume input        */
			puts("\b\b\b 0");
			abort = 1;	/* don't auto boot      */
		}
	}
#endif

	while ((bootdelay > 0) && (!abort)) {
		--bootdelay;
		/* delay 1000 ms */
		ts = get_timer(0);
		do {
			if (tstc()) {	/* we got a key press   */
				abort = 1;	/* don't auto boot      */
				bootdelay = 0;	/* no more delay        */
				(void)getc();	/* consume input        */
				break;
			}
			mdelay(10);
		} while (!abort && get_timer(ts) < 1000);

		printf("\b\b\b%2d ", bootdelay);
	}

	putc('\n');

#ifdef CONFIG_SILENT_CONSOLE
	/* show prompt */
	if (abort)
		gd->flags &= ~GD_FLG_SILENT;
#endif

	return abort;
}

static void spl_autoboot(void)
{
	char *s = getenv("bootcmd");

	debug("### main_loop: bootcmd=\"%s\"\n", s ? s : "<UNDEFINED>");

	if (!spl_bootdelay())
		run_command_list(s, -1, 0);
}

static inline void debug_core_status(void)
{
#if defined(DEBUG)
	uint pc, sp;

	asm("mov %0, pc" : "=r"(pc));
	asm("mov %0, sp" : "=r"(sp));

	debug("core pc: 0x%x, sp: 0x%x\n", pc, sp);
	debug("global gd: 0x%x, bd: 0x%x\n", (uint)gd, (uint)gd->bd);
#endif
}

static init_fnc_t init_sequence_r[] = {
	spl_enable_caches,
	spl_malloc_init,
#if defined(CONFIG_SPL_MMC_SUPPORT)
	spl_mmc_init,
#endif
#if defined(CONFIG_SPL_ENV_SUPPORT)
	spl_env_init,
#endif
#if !defined CONFIG_OF_CONTROL && !defined CONFIG_DM_USB &&	\
	 defined CONFIG_USB_GADGET
	spl_udc_probe,
#endif
#if defined CONFIG_VIDEO
	spl_display_probe,
	spl_video_probe,
#endif
	NULL,
};
#endif /* CONFIG_SPL_CLI_FRAMEWORK */

void board_init_f(ulong dummy)
{
	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* set global data */
	gd = &_gdata;

	/* set board data */
	gd->bd = &_bdata;

	spl_stack_relocate();
}

int spl_init_call_list(const init_fnc_t init_sequence[])
{
	const init_fnc_t *init_fnc_ptr;

	for (init_fnc_ptr = init_sequence; *init_fnc_ptr; ++init_fnc_ptr) {
		int ret;

		debug("initcall: %p\n", (char *)*init_fnc_ptr);
		ret = (*init_fnc_ptr) ();
		if (ret) {
			printf
			    ("SPL initcall %p failed call %p (err=%d)\n",
			     init_sequence, (char *)*init_fnc_ptr, ret);
			return -1;
		}
	}
	return 0;
}

__weak void spl_cli_init(void)
{
}

__weak int spl_late_init(void)
{
	return 0;
}

void spl_board_init_r(gd_t *dummy1, ulong dummy2)
{
	if (spl_init_call_list(init_sequence_f))
		hang();

#if defined(CONFIG_SPL_CLI_FRAMEWORK)
	if (spl_init_call_list(init_sequence_r))
		hang();

	debug_core_status();

	cli_init();

	if (spl_late_init())
		hang();

	spl_autoboot();
	cli_loop();

#elif defined(CONFIG_SPL_FRAMEWORK)
	board_init_r(NULL, 0);
#endif
	hang();
}
