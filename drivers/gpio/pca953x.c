/*
 *  pca953x.c - 4/8/16 bit I/O ports
 *
 *  Copyright (C) 2005 Ben Gardner <bgardner@wabtec.com>
 *  Copyright (C) 2007 Marvell International Ltd.
 *
 *  Derived from drivers/i2c/chips/pca9539.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>

#include <asm/gpio.h>

#define PCA953X_INPUT          0
#define PCA953X_OUTPUT         1
#define PCA953X_INVERT         2
#define PCA953X_DIRECTION      3

static const struct i2c_device_id pca953x_id[] = {
	{ "pca9534", 8, },
	{ "pca9535", 16, },
	{ "pca9536", 4, },
	{ "pca9537", 4, },
	{ "pca9538", 8, },
	{ "pca9539", 16, },
	{ "pca9554", 8, },
	{ "pca9555", 16, },
	{ "pca9557", 8, },
	{ "max7310", 8, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca953x_id);

struct pca953x_chip {
	unsigned gpio_start;
	uint16_t reg_output;
	uint16_t reg_direction;

	struct i2c_client *client;
	struct gpio_chip gpio_chip;
#ifdef CONFIG_GPIO_PCA953X_GENERIC_IRQ
	uint16_t last_input;
	/*
	 * Note: Generic IRQ is not accessible within module code, the IRQ
	 * support will thus _only_ be available if the driver is built-in
	 */
	int irq;	/* IRQ for the chip itself */
	int irq_start;	/* starting IRQ for the on-chip GPIO lines */

	uint16_t irq_mask;
	uint16_t irq_falling_edge;
	uint16_t irq_rising_edge;

	struct irq_chip irq_chip;
	struct work_struct irq_work;
#endif
};

/* NOTE:  we can't currently rely on fault codes to come from SMBus
 * calls, so we map all errors to EIO here and return zero otherwise.
 */
static int pca953x_write_reg(struct pca953x_chip *chip, int reg, uint16_t val)
{
	int ret;

	if (chip->gpio_chip.ngpio <= 8)
		ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	else
		ret = i2c_smbus_write_word_data(chip->client, reg << 1, val);

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed writing register\n");
		return -EIO;
	}

	return 0;
}

static int pca953x_read_reg(struct pca953x_chip *chip, int reg, uint16_t *val)
{
	int ret;

	if (chip->gpio_chip.ngpio <= 8)
		ret = i2c_smbus_read_byte_data(chip->client, reg);
	else
		ret = i2c_smbus_read_word_data(chip->client, reg << 1);

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed reading register\n");
		return -EIO;
	}

	*val = (uint16_t)ret;
	return 0;
}

static int pca953x_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct pca953x_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pca953x_chip, gpio_chip);

	reg_val = chip->reg_direction | (1u << off);
	ret = pca953x_write_reg(chip, PCA953X_DIRECTION, reg_val);
	if (ret)
		return ret;

	chip->reg_direction = reg_val;
	return 0;
}

static int pca953x_gpio_direction_output(struct gpio_chip *gc,
		unsigned off, int val)
{
	struct pca953x_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pca953x_chip, gpio_chip);

	/* set output level */
	if (val)
		reg_val = chip->reg_output | (1u << off);
	else
		reg_val = chip->reg_output & ~(1u << off);

	ret = pca953x_write_reg(chip, PCA953X_OUTPUT, reg_val);
	if (ret)
		return ret;

	chip->reg_output = reg_val;

	/* then direction */
	reg_val = chip->reg_direction & ~(1u << off);
	ret = pca953x_write_reg(chip, PCA953X_DIRECTION, reg_val);
	if (ret)
		return ret;

	chip->reg_direction = reg_val;
	return 0;
}

static int pca953x_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct pca953x_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pca953x_chip, gpio_chip);

	ret = pca953x_read_reg(chip, PCA953X_INPUT, &reg_val);
	if (ret < 0) {
		/* NOTE:  diagnostic already emitted; that's all we should
		 * do unless gpio_*_value_cansleep() calls become different
		 * from their nonsleeping siblings (and report faults).
		 */
		return 0;
	}

	return (reg_val & (1u << off)) ? 1 : 0;
}

static void pca953x_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct pca953x_chip *chip;
	uint16_t reg_val;
	int ret;

	chip = container_of(gc, struct pca953x_chip, gpio_chip);

	if (val)
		reg_val = chip->reg_output | (1u << off);
	else
		reg_val = chip->reg_output & ~(1u << off);

	ret = pca953x_write_reg(chip, PCA953X_OUTPUT, reg_val);
	if (ret)
		return;

	chip->reg_output = reg_val;
}

static void pca953x_setup_gpio(struct pca953x_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input  = pca953x_gpio_direction_input;
	gc->direction_output = pca953x_gpio_direction_output;
	gc->get = pca953x_gpio_get_value;
	gc->set = pca953x_gpio_set_value;
	gc->can_sleep = 1;

	gc->base = chip->gpio_start;
	gc->ngpio = gpios;
	gc->label = chip->client->name;
	gc->dev = &chip->client->dev;
	gc->owner = THIS_MODULE;
}

#ifdef CONFIG_GPIO_PCA953X_GENERIC_IRQ
/* FIXME: change to schedule_delayed_work() here if reading out of
 * registers does not reflect the actual pin levels
 */

static void pca953x_irq_work(struct work_struct *work)
{
	struct pca953x_chip *chip;
	uint16_t input, mask, rising, falling;
	int ret, i;

	chip = container_of(work, struct pca953x_chip, irq_work);

	ret = pca953x_read_reg(chip, PCA953X_INPUT, &input);
	if (ret < 0)
		return;

	mask = (input ^ chip->last_input) & chip->irq_mask;
	rising = (input & mask) & chip->irq_rising_edge;
	falling = (~input & mask) & chip->irq_falling_edge;

	irq_enter();

	for (i = 0; i < chip->gpio_chip.ngpio; i++) {
		if ((rising | falling) & (1u << i)) {
			int irq = chip->irq_start + i;
			struct irq_desc *desc;

			desc = irq_desc + irq;
			desc_handle_irq(irq, desc);
		}
	}

	irq_exit();

	chip->last_input = input;
}

static void
pca953x_irq_demux(unsigned int irq, struct irq_desc *desc)
{
	struct pca953x_chip *chip = desc->handler_data;

	desc->chip->mask(chip->irq);
	desc->chip->ack(chip->irq);
	schedule_work(&chip->irq_work);
	desc->chip->unmask(chip->irq);
}

static void pca953x_irq_mask(unsigned int irq)
{
	struct irq_desc *desc = irq_desc + irq;
	struct pca953x_chip *chip = desc->chip_data;

	chip->irq_mask &= ~(1u << (irq - chip->irq_start));
}

static void pca953x_irq_unmask(unsigned int irq)
{
	struct irq_desc *desc = irq_desc + irq;
	struct pca953x_chip *chip = desc->chip_data;

	chip->irq_mask |= 1u << (irq - chip->irq_start);
}

static void pca953x_irq_ack(unsigned int irq)
{
	/* unfortunately, we have to provide an empty irq_chip.ack even
	 * if we do nothing here, Generic IRQ will complain otherwise
	 */
}

static int pca953x_irq_set_type(unsigned int irq, unsigned int type)
{
	struct irq_desc *desc = irq_desc + irq;
	struct pca953x_chip *chip = desc->chip_data;
	uint16_t mask = 1u << (irq - chip->irq_start);

	if (type == IRQT_PROBE) {
		if ((mask & chip->irq_rising_edge) ||
		    (mask & chip->irq_falling_edge) ||
		    (mask & ~chip->reg_direction))
			return 0;

		type = __IRQT_RISEDGE | __IRQT_FALEDGE;
	}

	gpio_direction_input(irq_to_gpio(irq));

	if (type & __IRQT_RISEDGE)
		chip->irq_rising_edge |= mask;
	else
		chip->irq_rising_edge &= ~mask;

	if (type & __IRQT_FALEDGE)
		chip->irq_falling_edge |= mask;
	else
		chip->irq_falling_edge &= ~mask;

	return 0;
}

static int pca953x_init_irq(struct pca953x_chip *chip)
{
	struct irq_chip *ic = &chip->irq_chip;
	int irq, irq_start = chip->irq_start;

	chip->irq = chip->client->irq;
	chip->irq_start = irq_start = gpio_to_irq(chip->gpio_start);

	/* do not install GPIO interrupts for the chip if
	 * 1. the PCA953X interrupt line is not used
	 * 2. or the GPIO interrupt number exceeds NR_IRQS
	 */
	if (chip->irq <= 0 || irq_start + chip->gpio_chip.ngpio >= NR_IRQS)
		return -EINVAL;

	chip->irq_mask	= 0;
	chip->irq_rising_edge  = 0;
	chip->irq_falling_edge = 0;

	ic->ack = pca953x_irq_ack;
	ic->mask = pca953x_irq_mask;
	ic->unmask = pca953x_irq_unmask;
	ic->set_type = pca953x_irq_set_type;

	for (irq = irq_start; irq < irq_start + chip->gpio_chip.ngpio; irq++) {
		set_irq_chip(irq, ic);
		set_irq_chip_data(irq, chip);
		set_irq_handler(irq, handle_edge_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}

	set_irq_type(chip->irq, IRQT_FALLING);
	set_irq_data(chip->irq, chip);
	set_irq_chained_handler(chip->irq, pca953x_irq_demux);

	INIT_WORK(&chip->irq_work, pca953x_irq_work);
	return 0;
}
#else
static inline int pca953x_init_irq(struct pca953x_chip *chip)
{
	return 0;
}
#endif /* CONFIG_GPIO_PCA953X_GENERIC_IRQ */


static int __devinit pca953x_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct pca953x_platform_data *pdata;
	struct pca953x_chip *chip;
	int ret;

	pdata = client->dev.platform_data;
	if (pdata == NULL)
		return -ENODEV;

	chip = kzalloc(sizeof(struct pca953x_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip->client = client;

	chip->gpio_start = pdata->gpio_base;

	/* initialize cached registers from their original values.
	 * we can't share this chip with another i2c master.
	 */
	pca953x_setup_gpio(chip, id->driver_data);

	ret = pca953x_read_reg(chip, PCA953X_OUTPUT, &chip->reg_output);
	if (ret)
		goto out_failed;

	ret = pca953x_read_reg(chip, PCA953X_DIRECTION, &chip->reg_direction);
	if (ret)
		goto out_failed;

	/* set platform specific polarity inversion */
	ret = pca953x_write_reg(chip, PCA953X_INVERT, pdata->invert);
	if (ret)
		goto out_failed;


	ret = gpiochip_add(&chip->gpio_chip);
	if (ret)
		goto out_failed;

	if (pdata->setup) {
		ret = pdata->setup(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_warn(&client->dev, "setup failed, %d\n", ret);
	}

	ret = pca953x_init_irq(chip);
	if (ret) {
		ret = gpiochip_remove(&chip->gpio_chip);
		goto out_failed;
	}

	i2c_set_clientdata(client, chip);
	return 0;

out_failed:
	kfree(chip);
	return ret;
}
#ifdef CONFIG_GPIO_PCA953X_GENERIC_IRQ
static int pca953x_remove(struct i2c_client *client)
{
	printk(KERN_ERR "failed to unload the driver with IRQ support\n");
	return -EINVAL;
}
#else
static int pca953x_remove(struct i2c_client *client)
{
	struct pca953x_platform_data *pdata = client->dev.platform_data;
	struct pca953x_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	if (pdata->teardown) {
		ret = pdata->teardown(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0) {
			dev_err(&client->dev, "%s failed, %d\n",
					"teardown", ret);
			return ret;
		}
	}

	ret = gpiochip_remove(&chip->gpio_chip);
	if (ret) {
		dev_err(&client->dev, "%s failed, %d\n",
				"gpiochip_remove()", ret);
		return ret;
	}

	kfree(chip);
	return 0;
}
#endif /* CONFIG_GPIO_PCA953X_GENERIC_IRQ */

static struct i2c_driver pca953x_driver = {
	.driver = {
		.name	= "pca953x",
	},
	.probe		= pca953x_probe,
	.remove		= pca953x_remove,
	.id_table	= pca953x_id,
};

static int __init pca953x_init(void)
{
	return i2c_add_driver(&pca953x_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(pca953x_init);

static void __exit pca953x_exit(void)
{
	i2c_del_driver(&pca953x_driver);
}
module_exit(pca953x_exit);

MODULE_AUTHOR("eric miao <eric.miao@marvell.com>");
MODULE_DESCRIPTION("GPIO expander driver for PCA953x");
MODULE_LICENSE("GPL");
