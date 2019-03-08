// SPDX-License-Identifier: GPL-2.0
/*
 * Realtek RTL8186 SoC timer driver.
 *
 * Timer0 (24bit): Unused
 * Timer1 (24bit): Unused
 * Timer2 (32bit): Used as clocksource
 * Timer3 (32bit): Used as clock event device
 *
 * Copyright (C) 2019 Yasha Cherikovsky
 */

#include <linux/init.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/sched_clock.h>
#include <linux/of_clk.h>
#include <linux/io.h>

#include "timer-of.h"

/* Timer registers */
#define TCCNR			0x0
#define TCIR			0x4
#define TC_DATA(t)		(0x10 + 4 * (t))
#define TC_CNT(t)		(0x20 + 4 * (t))

/* TCCNR register bits */
#define TCCNR_TC_EN_BIT(t)		BIT((t) * 2)
#define TCCNR_TC_MODE_BIT(t)		BIT((t) * 2 + 1)
#define TCCNR_TC_SRC_BIT(t)		BIT((t) + 8)

/* TCIR register bits */
#define TCIR_TC_IE_BIT(t)		BIT(t)
#define TCIR_TC_IP_BIT(t)		BIT((t) + 4)


/* Forward declaration */
static struct timer_of to;

static void __iomem *base;

#define RTL8186_TIMER_MODE_COUNTER	0
#define RTL8186_TIMER_MODE_TIMER	1

static void rtl8186_set_enable_bit(int timer, int enabled)
{
	u16 tccnr;

	tccnr = readl(base + TCCNR);
	tccnr &= ~(TCCNR_TC_EN_BIT(timer));

	if (enabled)
		tccnr |= TCCNR_TC_EN_BIT(timer);

	writel(tccnr, base + TCCNR);
}

static void rtl8186_set_mode_bit(int timer, int mode)
{
	u16 tccnr;

	tccnr = readl(base + TCCNR);
	tccnr &= ~(TCCNR_TC_MODE_BIT(timer));

	if (mode)
		tccnr |= TCCNR_TC_MODE_BIT(timer);

	writel(tccnr, base + TCCNR);
}

static irqreturn_t rtl8186_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *cd = dev_id;
	int status;

	status = readl(base + TCIR);
	writel(status, base + TCIR); /* Clear all interrupts */

	cd->event_handler(cd);

	return IRQ_HANDLED;
}

static int rtl8186_clockevent_set_next(unsigned long evt,
				       struct clock_event_device *cd)
{
	rtl8186_set_enable_bit(3, 0);
	writel(evt, base + TC_DATA(3));
	writel(evt, base + TC_CNT(3));
	rtl8186_set_enable_bit(3, 1);
	return 0;
}

static int rtl8186_set_state_periodic(struct clock_event_device *cd)
{
	unsigned long period = timer_of_period(to_timer_of(cd));

	rtl8186_set_enable_bit(3, 0);
	rtl8186_set_mode_bit(3, RTL8186_TIMER_MODE_TIMER);

	/* This timer should reach zero each jiffy */
	writel(period, base + TC_DATA(3));
	writel(period, base + TC_CNT(3));

	rtl8186_set_enable_bit(3, 1);
	return 0;
}

static int rtl8186_set_state_oneshot(struct clock_event_device *cd)
{
	rtl8186_set_enable_bit(3, 0);
	rtl8186_set_mode_bit(3, RTL8186_TIMER_MODE_COUNTER);
	return 0;
}

static int rtl8186_set_state_shutdown(struct clock_event_device *cd)
{
	rtl8186_set_enable_bit(3, 0);
	return 0;
}

static void rtl8186_timer_init_hw(void)
{
	/* Disable all timers */
	writel(0, base + TCCNR);

	/* Clear and disable all timer interrupts */
	writel(0xf0, base + TCIR);

	/* Reset all timers timeouts */
	writel(0, base + TC_DATA(0));
	writel(0, base + TC_DATA(1));
	writel(0, base + TC_DATA(2));
	writel(0, base + TC_DATA(3));

	/* Reset all counters */
	writel(0, base + TC_CNT(0));
	writel(0, base + TC_CNT(1));
	writel(0, base + TC_CNT(2));
	writel(0, base + TC_CNT(3));
}

static u64 notrace rtl8186_timer_sched_read(void)
{
	return ~readl(base + TC_CNT(2));
}

static int rtl8186_start_clksrc(void)
{
	/* We use Timer2 as a clocksource (monotonic counter). */
	writel(0xFFFFFFFF, base + TC_DATA(2));
	writel(0xFFFFFFFF, base + TC_CNT(2));

	rtl8186_set_mode_bit(2, RTL8186_TIMER_MODE_TIMER);
	rtl8186_set_enable_bit(2, 1);

	sched_clock_register(rtl8186_timer_sched_read, 32, timer_of_rate(&to));

	return clocksource_mmio_init(base + TC_CNT(2), "rtl8186-clksrc",
				     timer_of_rate(&to), 500, 32,
				     clocksource_mmio_readl_down);
}

static struct timer_of to = {
	.flags = TIMER_OF_BASE | TIMER_OF_CLOCK | TIMER_OF_IRQ,

	.clkevt = {
			.name = "rtl8186_tick",
			.rating = 200,
			.features = CLOCK_EVT_FEAT_ONESHOT |
				    CLOCK_EVT_FEAT_PERIODIC,
			.set_next_event = rtl8186_clockevent_set_next,
			.cpumask = cpu_possible_mask,
			.set_state_periodic = rtl8186_set_state_periodic,
			.set_state_oneshot = rtl8186_set_state_oneshot,
			.set_state_shutdown = rtl8186_set_state_shutdown,
	},

	.of_irq = {
			.handler = rtl8186_timer_interrupt,
			.flags = IRQF_TIMER,
	},
};

static int __init rtl8186_timer_init(struct device_node *node)
{
	int ret;

	ret = timer_of_init(node, &to);
	if (ret)
		return ret;

	base = timer_of_base(&to);

	rtl8186_timer_init_hw();

	ret = rtl8186_start_clksrc();
	if (ret) {
		pr_err("Failed to register clocksource\n");
		return ret;
	}

	clockevents_config_and_register(&to.clkevt, timer_of_rate(&to), 100,
					0xffffffff);

	/* Enable interrupts for Timer3. Disable interrupts for others */
	writel(TCIR_TC_IE_BIT(3), base + TCIR);

	return 0;
}

TIMER_OF_DECLARE(rtl8186_timer, "realtek,rtl8186-timer", rtl8186_timer_init);
