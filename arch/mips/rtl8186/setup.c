// SPDX-License-Identifier: GPL-2.0
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/libfdt.h>

#include <asm/reboot.h>
#include <asm/io.h>
#include <asm/prom.h>
#include <asm/bootinfo.h>

#include <asm/mach-rtl8186/rtl8186.h>

const char *get_system_type(void)
{
	return "Realtek RTL8186";
}

void rtl8186_machine_restart(char *command)
{
	/* Disable all interrupts */
	local_irq_disable();

	/* Use watchdog to reset the system */
	writel(0x10, RTL8186_CDBR);
	writel(0x00, RTL8186_WDTCNR);

	for (;;)
		;
}

#define GPIO_A2 BIT(2)
#define GPIO_A3 BIT(3)
#define GPIO_A7 BIT(7)
#define GPIO_A8 BIT(8)

/* Temporary hack until rtl8186-gpio driver is implemented */
void __init rtl8186_edimax_br6204wg_setup_leds(void)
{
	unsigned int gpabdir, gpabdata;

	gpabdir = readl(RTL8186_GPABDIR);
	gpabdata = readl(RTL8186_GPABDATA);

	writel(gpabdir | (GPIO_A2 | GPIO_A3), RTL8186_GPABDIR);

	gpabdata &= ~GPIO_A2; /* Turn on A2 - green PWR */
	gpabdata |= GPIO_A3;  /* Turn off A3 - orange WLAN */
	writel(gpabdata, RTL8186_GPABDATA);
}

void __init *plat_get_fdt(void)
{
	return get_fdt();
}

void __init plat_mem_setup(void)
{
	void *dtb;

	_machine_restart = rtl8186_machine_restart;

	dtb = plat_get_fdt();
	if (!dtb)
		panic("no dtb found");

	__dt_setup_arch(dtb);
}

void __init device_tree_init(void)
{
	unflatten_and_copy_device_tree();

	if (of_machine_is_compatible("edimax,br-6204wg"))
		rtl8186_edimax_br6204wg_setup_leds();
}
