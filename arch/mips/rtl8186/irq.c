// SPDX-License-Identifier: GPL-2.0
#include <linux/init.h>
#include <linux/irqchip.h>

void __init arch_init_irq(void)
{
	irqchip_init();
}
