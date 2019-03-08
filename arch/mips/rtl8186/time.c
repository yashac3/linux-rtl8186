// SPDX-License-Identifier: GPL-2.0
#include <linux/init.h>
#include <linux/clocksource.h>
#include <linux/of_clk.h>

void __init plat_time_init(void)
{
	of_clk_init(NULL);
	timer_probe();
}
