// SPDX-License-Identifier: GPL-2.0
/*
 * Realtek RTL8186 SoC interrupt controller driver.
 *
 * Copyright (C) 2019 Yasha Cherikovsky
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define RTL8186_NR_IRQS 11

#define GIMR 0x00
#define GISR 0x04

static struct {
	void __iomem *base;
	struct irq_domain *domain;
} intc;

asmlinkage void plat_irq_dispatch(void)
{
	u32 hwirq, virq;
	u32 gimr = readl(intc.base + GIMR);
	u32 gisr = readl(intc.base + GISR);
	u32 pending = gimr & gisr & ((1 << RTL8186_NR_IRQS) - 1);

	if (!pending) {
		spurious_interrupt();
		return;
	}

	while (pending) {
		hwirq = fls(pending) - 1;
		virq = irq_linear_revmap(intc.domain, hwirq);
		do_IRQ(virq);
		pending &= ~BIT(hwirq);
	}
}

static void rtl8186_irq_mask(struct irq_data *data)
{
	unsigned long irq = data->hwirq;

	writel(readl(intc.base + GIMR) & (~(BIT(irq))), intc.base + GIMR);
}

static void rtl8186_irq_unmask(struct irq_data *data)
{
	unsigned long irq = data->hwirq;

	writel((readl(intc.base + GIMR) | (BIT(irq))), intc.base + GIMR);
}

static struct irq_chip rtl8186_irq_chip = {
	.name = "RTL8186",
	.irq_mask = rtl8186_irq_mask,
	.irq_unmask = rtl8186_irq_unmask,
};

static int rtl8186_intc_irq_domain_map(struct irq_domain *d, unsigned int virq,
				       irq_hw_number_t hw)
{
	irq_set_chip_and_handler(virq, &rtl8186_irq_chip, handle_level_irq);
	return 0;
}

static const struct irq_domain_ops rtl8186_irq_ops = {
	.map = rtl8186_intc_irq_domain_map,
	.xlate = irq_domain_xlate_onecell,
};

static int __init rtl8186_intc_of_init(struct device_node *node,
				       struct device_node *parent)
{
	intc.base = of_io_request_and_map(node, 0, of_node_full_name(node));

	if (IS_ERR(intc.base))
		panic("%pOF: unable to map resource", node);

	intc.domain = irq_domain_add_linear(node, RTL8186_NR_IRQS,
					    &rtl8186_irq_ops, NULL);

	if (!intc.domain)
		panic("%pOF: unable to create IRQ domain\n", node);

	/* Start with all interrupts disabled */
	writel(0, intc.base + GIMR);

	/*
	 * Enable all hardware interrupts in CP0 status register.
	 * Software interrupts are disabled.
	 */
	set_c0_status(ST0_IM);
	clear_c0_status(STATUSF_IP0 | STATUSF_IP1);
	clear_c0_cause(CAUSEF_IP);

	return 0;
}

IRQCHIP_DECLARE(rtl8186_intc, "realtek,rtl8186-intc", rtl8186_intc_of_init);
