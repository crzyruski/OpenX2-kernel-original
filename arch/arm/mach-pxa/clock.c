/*
 *  linux/arch/arm/mach-sa1100/clock.c
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <mach/pxa2xx-regs.h>
#include <mach/pxa2xx-gpio.h>
#include <mach/pxa_ispt.h>
#include <mach/hardware.h>

#include "devices.h"
#include "generic.h"
#include "clock.h"

static LIST_HEAD(clocks);
static DEFINE_MUTEX(clocks_mutex);
static DEFINE_SPINLOCK(clocks_lock);

static struct clk *clk_lookup(struct device *dev, const char *id)
{
	struct clk *p;

	list_for_each_entry(p, &clocks, node)
		if (strcmp(id, p->name) == 0 && p->dev == dev)
			return p;

	return NULL;
}

#ifdef CONFIG_ISPT
static int ispt_clock_msg(u32 msg, u32 param)
{
	return ispt_driver_msg(msg, param);
}
#else
static int ispt_clock_msg(u32 msg, u32 param) { return 0; }
#endif

struct clk *clk_get(struct device *dev, const char *id)
{
	struct clk *p, *clk = ERR_PTR(-ENOENT);

	mutex_lock(&clocks_mutex);
	p = clk_lookup(dev, id);
	if (!p)
		p = clk_lookup(NULL, id);
	if (p)
		clk = p;
	mutex_unlock(&clocks_mutex);

	if (!IS_ERR(clk) && clk->ops == NULL)
		clk = clk->other;

	return clk;
}
EXPORT_SYMBOL(clk_get);

void clk_put(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_put);

int clk_enable(struct clk *clk)
{
	unsigned long flags;

	spin_lock_irqsave(&clocks_lock, flags);
	if (clk->enabled++ == 0) {
		clk->ops->enable(clk);
		ispt_clock_msg(CT_P_CLOCK_ENABLE, clk->cken);
	}
	spin_unlock_irqrestore(&clocks_lock, flags);

	if (clk->delay)
		udelay(clk->delay);

	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;

	if (clk->enabled == 0)
		return;

	spin_lock_irqsave(&clocks_lock, flags);
	if (--clk->enabled == 0) {
		clk->ops->disable(clk);
		ispt_clock_msg(CT_P_CLOCK_DISABLE, clk->cken);
	}
	spin_unlock_irqrestore(&clocks_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
	unsigned long rate;

	rate = clk->rate;
	if (clk->ops->getrate)
		rate = clk->ops->getrate(clk);

	return rate;
}
EXPORT_SYMBOL(clk_get_rate);


void clk_cken_enable(struct clk *clk)
{
	CKEN |= 1 << clk->cken;
}

void clk_cken_disable(struct clk *clk)
{
	CKEN &= ~(1 << clk->cken);
}

const struct clkops clk_cken_ops = {
	.enable		= clk_cken_enable,
	.disable	= clk_cken_disable,
};

void clks_register(struct clk *clks, size_t num)
{
	int i;

	mutex_lock(&clocks_mutex);
	for (i = 0; i < num; i++)
		list_add(&clks[i].node, &clocks);
	mutex_unlock(&clocks_mutex);
}

int clk_add_alias(char *alias, struct device *alias_dev, char *id,
	struct device *dev)
{
	struct clk *r = clk_lookup(dev, id);
	struct clk *new;

	if (!r)
		return -ENODEV;

	new = kzalloc(sizeof(struct clk), GFP_KERNEL);

	if (!new)
		return -ENOMEM;

	new->name = alias;
	new->dev = alias_dev;
	new->other = r;

	mutex_lock(&clocks_mutex);
	list_add(&new->node, &clocks);
	mutex_unlock(&clocks_mutex);

	return 0;
}
