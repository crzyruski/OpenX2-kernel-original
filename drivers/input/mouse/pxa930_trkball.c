/*
 * linux/driver/input/mouse/pxa930_trkball.c
 *
 * PXA930 track ball mouse driver
 *
 * Copyright (C) 2007 Marvell International Ltd.
 * 2008-02-28: Yong Yao <yaoyong@marvell.com>
 *             initial version
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/input.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>

#include <mach/pxa930_trkball.h>

#define TBCR		0xC
#define TBCNTR		0x10
#define TBSBC		0x14

#define TBCR_TBSB	(0x1 << 10)
#define TBCR_TBRST	(0x1 <<  1)

#define TBCR_Y_FLT(n)	(((n) & 0xf) << 6)
#define TBCR_X_FLT(n)	(((n) & 0xf) << 2)

#define TBCNTR_YM(n)	(((n) >> 24) & 0xff)
#define TBCNTR_YP(n)	(((n) >> 16) & 0xff)
#define TBCNTR_XM(n)	(((n) >> 8) & 0xff)
#define TBCNTR_XP(n)	((n) & 0xff)

#define TBSBC_TBSBC	0x1
#define trkball_readl(off)	__raw_readl(trkball->mmio_base + (off))
#define trkball_writel(off, v)	__raw_writel((v), trkball->mmio_base + (off))

struct pxa930_trkball {
	struct pxa930_trkball_platform_data *pdata;

	/* Memory Mapped Register */
	struct resource *mem;
	void __iomem *mmio_base;

	struct input_dev *input;
};

static irqreturn_t pxa930_trkball_interrupt(int irq, void *dev_id)
{
	struct pxa930_trkball *trkball = dev_id;
	struct input_dev *input = trkball->input;
	int tbcntr, temp;
	int x, y;

	/* According to the spec software must read TBCNTR twice:
	 * if the read value is the same, the reading is valid
	 */
	temp = trkball_readl(TBCNTR);
	tbcntr = trkball_readl(TBCNTR);
	if (temp == tbcntr)
	{

		pr_debug("%s: tbcntr %x!\n", __func__, tbcntr);

		x = (TBCNTR_XP(tbcntr) - TBCNTR_XM(tbcntr))/2;
		y = (TBCNTR_YP(tbcntr) - TBCNTR_YM(tbcntr))/2;

		input_report_rel(input, REL_X, x);
		input_report_rel(input, REL_Y, y);
		input_sync(input);
	}

	trkball_writel(TBSBC, TBSBC_TBSBC);
	trkball_writel(TBSBC, 0);

	return IRQ_HANDLED;
}

static int trkball_write(struct pxa930_trkball *trkball, int off, int value)
{
	int i = 0;

	trkball_writel(off, value);
	while ((value != trkball_readl(off)) && (i < 100)) {
		i++;
		msleep(1);
	}

	if (i == 100) {
		pr_err("%s: TIMEOUT when write %x!\n", __func__, value);
		return -ETIMEDOUT;
	}

	return 0;
}

static int pxa930_trkball_config(struct pxa930_trkball *trkball)
{
	/* According to spec, need to write the filters of x,y to 0xf first! */
	trkball_write(trkball, TBCR, trkball_readl(TBCR) | TBCR_X_FLT(0xf) |
				     TBCR_Y_FLT(0xf));
	trkball_write(trkball, TBCR, TBCR_X_FLT(trkball->pdata->x_filter) |
				     TBCR_Y_FLT(trkball->pdata->y_filter));

	/* According to spec, set TBCR_TBRST first, before clearing it! */
	trkball_write(trkball, TBCR, trkball_readl(TBCR) | TBCR_TBRST);
	trkball_write(trkball, TBCR, trkball_readl(TBCR) & ~(TBCR_TBRST));

	trkball_writel(TBSBC, TBSBC_TBSBC);
	trkball_writel(TBSBC, 0);

	pr_debug("%s: config end tbcr %x!\n", __func__, trkball_readl(TBCR));
	return 0;
}

static int pxa930_trkball_open(struct input_dev *dev)
{
	struct pxa930_trkball *trkball = input_get_drvdata(dev);
	return pxa930_trkball_config(trkball);
}

static void pxa930_trkball_clk_disable(struct pxa930_trkball *trkball)
{
	trkball_write(trkball, TBCR, trkball_readl(TBCR) | TBCR_TBRST);
}

static void pxa930_trkball_close(struct input_dev *dev)
{
	struct pxa930_trkball *trkball = input_get_drvdata(dev);
	pxa930_trkball_clk_disable(trkball);
}

static int __init pxa930_trkball_probe(struct platform_device *pdev)
{
	struct pxa930_trkball *trkball;
	struct input_dev *input;
	struct resource *res;
	int irq, size;
	int ret = -EINVAL;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get trkball irq\n");
		return -ENXIO;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get register memory\n");
		return -ENXIO;
	}

	trkball = kzalloc(sizeof(struct pxa930_trkball), GFP_KERNEL);
	if (!trkball)
		return -ENOMEM;

	trkball->pdata = pdev->dev.platform_data;
	if (trkball->pdata == NULL) {
		dev_err(&pdev->dev, "no platform data defined\n");
		ret = -EINVAL;
		goto failed;
	}

	size = res->end - res->start + 1;
	trkball->mem = request_mem_region(res->start, size, pdev->name);
	if (trkball->mem == NULL) {
		dev_err(&pdev->dev, "failed to request register memory\n");
		ret = -EBUSY;
		goto failed;
	}

	trkball->mmio_base = ioremap_nocache(res->start, size);
	if (trkball->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap registers\n");
		ret = -ENXIO;
		goto failed_freemem;
	}

	/* disable the clock of the trkball */
	pxa930_trkball_clk_disable(trkball);

	ret = request_irq(irq, pxa930_trkball_interrupt, IRQF_DISABLED,
			pdev->name, trkball);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq: %d\n", ret);
		goto failed_freeio;
	}

	platform_set_drvdata(pdev, trkball);

	input = input_allocate_device();
	if (!input) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		ret = -ENOMEM;
		goto failed_freeirq;
	}

	input->name = pdev->name;
	input->id.bustype = BUS_HOST;
	input->open = pxa930_trkball_open;
	input->close = pxa930_trkball_close;
	input->dev.parent = &pdev->dev;
	input_set_drvdata(input, trkball);

	trkball->input = input;

	input_set_capability(input, EV_REL, REL_X);
	input_set_capability(input, EV_REL, REL_Y);

	__set_bit(REL_X, input->relbit);
	__set_bit(REL_Y, input->relbit);
	__set_bit(BTN_MOUSE, input->keybit);
	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_REL, input->evbit);

	ret = input_register_device(input);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register input device\n");
		input_free_device(input);
		goto failed_freeirq;
	}

	return 0;

failed_freeirq:
	free_irq(irq, trkball);
failed_freeio:
	iounmap(trkball->mmio_base);
failed_freemem:
	release_resource(trkball->mem);
	kfree(trkball->mem);
failed:
	kfree(trkball);
	return ret;
}

static int __devexit pxa930_trkball_remove(struct platform_device *pdev)
{
	struct pxa930_trkball *trkball = platform_get_drvdata(pdev);
	int irq;

	input_unregister_device(trkball->input);

	irq = platform_get_irq(pdev, 0);
	free_irq(irq, trkball);

	iounmap(trkball->mmio_base);

	release_resource(trkball->mem);
	kfree(trkball->mem);

	kfree(trkball);
	return 0;
}

static struct platform_driver pxa930_trkball_driver = {
	.driver		= {
		.name	= "pxa930-trkball",
	},
	.probe		= pxa930_trkball_probe,
	.remove		= __devexit_p(pxa930_trkball_remove),
};

static int __init pxa930_trkball_init(void)
{
	return platform_driver_register(&pxa930_trkball_driver);
}

static void __exit pxa930_trkball_exit(void)
{
	platform_driver_unregister(&pxa930_trkball_driver);
}

module_init(pxa930_trkball_init);
module_exit(pxa930_trkball_exit);

MODULE_AUTHOR("Yong Yao <yaoyong@marvell.com>");
MODULE_DESCRIPTION("PXA930 Trackball Mouse Driver");
MODULE_LICENSE("GPL");
