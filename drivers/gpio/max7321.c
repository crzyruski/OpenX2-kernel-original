/*
    max7321.c - 8-bit I/O port with interrupt and reset

    Copyright (C) 2005 Jack Ren <jack.ren@marvell.com>
    Copyright (C) 2007 Marvell Internation Ltd.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; version 2 of the License.
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/i2c/max7321.h>

#include <asm/gpio.h>

#define NR_MAX7321_GPIOS	8

struct max7321_chip {
	unsigned gpio_start;
	uint8_t io_levels;
	uint8_t io_directions;
	uint8_t last_io_levels; /* io levels at last IRQ */

	struct i2c_client *client;
	struct gpio_chip gpio_chip;
#ifdef CONFIG_GPIO_MAX7321_GENERIC_IRQ
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

static int max7321_write(struct max7321_chip *chip, uint8_t val)
{
	int ret;

	ret =  i2c_smbus_write_byte(chip->client, val);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to write\n", __FUNCTION__);
		return ret;
	}

	return 0;
}

static int max7321_read(struct max7321_chip *chip, uint8_t *val)
{
	int ret;

	ret =  i2c_smbus_read_byte(chip->client);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to read\n", __FUNCTION__);
		return ret;
	}
	chip->io_levels = (uint8_t)ret;
	*val = (uint8_t)chip->io_levels;

	return 0;
}

static int max7321_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct max7321_chip *chip;
	uint8_t reg_val;
	int ret;

	chip = container_of(gc, struct max7321_chip, gpio_chip);

	if (chip->io_directions & (1u << off))
		return 0;

	/* In order to configure that IO port as input, set it
	 * as logic high (open-drain IO port will act as high-impedance,
	 * so it can be used to measure the external levels)
	 */
	reg_val = chip->io_levels | (1u << off);
	ret = max7321_write(chip, reg_val);
	if (ret)
		return ret;

	chip->io_directions |= (1u << off);
	return 0;
}

static int max7321_gpio_direction_output(struct gpio_chip *gc,
		unsigned off, int val)
{
	struct max7321_chip *chip;
	uint8_t reg_val;
	int ret;

	chip = container_of(gc, struct max7321_chip, gpio_chip);

	/* Note: in order to configure that IO port as output, please make sure
	 * that IO port has been connected to a voltage more than 0.7xV+
	 * by a pullup resistor
	 */

	/* set output level */
	if (val)
		reg_val = chip->io_levels | (1u << off);
	else
		reg_val = chip->io_levels & ~(1u << off);

	ret = max7321_write(chip, reg_val);
	if (ret)
		return ret;

	chip->io_levels = reg_val;

	if (val)
		chip->io_directions |= (1u << off);
	else
		chip->io_directions &= ~(1u << off);

	return 0;
}

static int max7321_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct max7321_chip *chip;
	uint8_t reg_val;
	int ret;

	chip = container_of(gc, struct max7321_chip, gpio_chip);

	ret = max7321_read(chip, &reg_val);
	if (ret < 0)
		return ret;

	return (reg_val & (1u << off)) ? 1 : 0;
}

static void max7321_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct max7321_chip *chip;
	uint8_t reg_val;
	int ret;

	chip = container_of(gc, struct max7321_chip, gpio_chip);

	if (val)
		reg_val = chip->io_levels | (1u << off);
	else
		reg_val = chip->io_levels & ~(1u << off);

	ret = max7321_write(chip, reg_val);
	if (ret)
		return;

	chip->io_levels = reg_val;

	if (val)
		chip->io_directions |= (1u << off);
	else
		chip->io_directions &= ~(1u << off);
}

static int max7321_init_gpio(struct max7321_chip *chip)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input  = max7321_gpio_direction_input;
	gc->direction_output = max7321_gpio_direction_output;
	gc->get = max7321_gpio_get_value;
	gc->set = max7321_gpio_set_value;

	gc->base = chip->gpio_start;
	gc->ngpio = NR_MAX7321_GPIOS;
	gc->label = "max7321";

	return gpiochip_add(gc);
}

#ifdef CONFIG_GPIO_MAX7321_GENERIC_IRQ
/* We do not use the max7321's transition flags to determine
 * which port is the interrtupt source, because the every
 * read/write operation will clear transition flags which will
 * lead the transtion flags unreliable.
 * Instead, we use the level comparison to figure ou the interrupt
 * source.
 */
static void max7321_irq_work(struct work_struct *work)
{
	struct max7321_chip *chip;
	uint8_t io_levels, mask, rising, falling;
	int ret, i;

	chip = container_of(work, struct max7321_chip, irq_work);

	ret = max7321_read(chip, &io_levels);
	if (ret < 0)
		return;

	mask = (io_levels ^ chip->last_io_levels) & chip->irq_mask;
	rising = (io_levels & mask) & chip->irq_rising_edge;
	falling = (~io_levels & mask) & chip->irq_falling_edge;

	pr_debug("%s: io_levels:0x%01x last_io_levels: 0x%01x\n",
		       __FUNCTION__, io_levels, chip->last_io_levels);
	pr_debug("%s: irq_mask: 0x%01x rising: 0x%01x falling: 0x%01x\n",
			__FUNCTION__, chip->irq_mask, rising, falling);
	irq_enter();

	for (i = 0; i < NR_MAX7321_GPIOS; i++) {
		if ((rising | falling) & (1u << i)) {
			int irq = chip->irq_start + i;
			struct irq_desc *desc;

			desc = irq_desc + irq;
			desc_handle_irq(irq, desc);
		}
	}

	irq_exit();

	chip->last_io_levels = io_levels;
}

static void
max7321_irq_demux(unsigned int irq, struct irq_desc *desc)
{
	struct max7321_chip *chip = desc->handler_data;

	desc->chip->mask(chip->irq);
	desc->chip->ack(chip->irq);
	schedule_work(&chip->irq_work);
	desc->chip->unmask(chip->irq);
}

static void max7321_irq_mask(unsigned int irq)
{
	struct irq_desc *desc = irq_desc + irq;
	struct max7321_chip *chip = desc->chip_data;

	chip->irq_mask &= ~(1u << (irq - chip->irq_start));
}

static void max7321_irq_unmask(unsigned int irq)
{
	struct irq_desc *desc = irq_desc + irq;
	struct max7321_chip *chip = desc->chip_data;

	chip->irq_mask |= 1u << (irq - chip->irq_start);
}

static void max7321_irq_ack(unsigned int irq)
{
	/* unfortunately, we have to provide an empty irq_chip.ack even
	 * if we do nothing here, Generic IRQ will complain otherwise
	 */
}

static int max7321_irq_set_type(unsigned int irq, unsigned int type)
{
	struct irq_desc *desc = irq_desc + irq;
	struct max7321_chip *chip = desc->chip_data;
	uint8_t mask = 1u << (irq - chip->irq_start);

	if (type == IRQT_PROBE) {
		if ((mask & chip->irq_rising_edge) ||
		    (mask & chip->irq_falling_edge) ||
		    (mask & ~chip->io_directions))
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

static int max7321_init_irq(struct max7321_chip *chip)
{
	struct irq_chip *ic = &chip->irq_chip;
	int irq, irq_start = chip->irq_start;

	if (chip->client->irq <= 0)
		return -EINVAL;

	chip->irq = chip->client->irq;
	chip->irq_start = irq_start = gpio_to_irq(chip->gpio_start);

	/* do not install GPIO interrupts for the chip if
	 * 1. the MAX7321 interrupt line is not used
	 * 2. or the GPIO interrupt number exceeds NR_IRQS
	 */
	pr_debug("%s:irq_start:%d, NR_IRQS: %d\n",
			__FUNCTION__, irq_start, NR_IRQS);
	if (chip->irq <= 0 || irq_start + NR_MAX7321_GPIOS >= NR_IRQS) {
		printk(KERN_ERR "%s: irq number exceeds NR_IRQS!\n",
				__FUNCTION__);
		return -EINVAL;
	}

	chip->irq_mask	= 0;
	chip->irq_rising_edge  = 0;
	chip->irq_falling_edge = 0;

	ic->ack = max7321_irq_ack;
	ic->mask = max7321_irq_mask;
	ic->unmask = max7321_irq_unmask;
	ic->set_type = max7321_irq_set_type;

	for (irq = irq_start; irq < irq_start + NR_MAX7321_GPIOS; irq++) {
		set_irq_chip(irq, ic);
		set_irq_chip_data(irq, chip);
		set_irq_handler(irq, handle_edge_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}

	set_irq_type(chip->irq, IRQT_FALLING);
	set_irq_data(chip->irq, chip);
	set_irq_chained_handler(chip->irq, max7321_irq_demux);

	INIT_WORK(&chip->irq_work, max7321_irq_work);
	return 0;
}
#else
static inline int max7321_init_irq(struct max7321_chip *chip)
{
	return -EINVAL;
}
#endif /* CONFIG_GPIO_MAX7321_GENERIC_IRQ */

static int __devinit max7321_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct max7321_platform_data *pdata;
	struct max7321_chip *chip;
	int ret;

	pdata = client->dev.platform_data;
	if (pdata == NULL)
		return -ENODEV;

	chip = kzalloc(sizeof(struct max7321_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip->client = client;

	chip->gpio_start = pdata->gpio_base;

	/* detect if there is max7321 */
	if (!max7321_read(chip, &chip->io_levels)) {
		printk(KERN_INFO "max7321 (addr:0x%02x) is detected!\n",
				client->addr);
		pdata->detected = 1;
	} else {
		printk(KERN_INFO "failed to detect max7321 (addr:0x%02x)!\n",
				client->addr);
		pdata->detected = 0;
		goto out_failed;
	}

	chip->last_io_levels = chip->io_levels;
	chip->io_directions = 0;
	ret = max7321_init_gpio(chip);
	if (ret)
		goto out_failed;

	if (pdata->setup) {
		ret = pdata->setup(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_dbg(&client->dev, "setup failed, %d\n", ret);
	}

	if (max7321_init_irq(chip))
		printk(KERN_INFO "max7321: doesn't support IRQ\n");

	i2c_set_clientdata(client, chip);

	return 0;

out_failed:
	kfree(chip);
	return ret;
}
#ifdef CONFIG_GPIO_MAX7321_GENERIC_IRQ
static int max7321_remove(struct i2c_client *client)
{
	printk(KERN_ERR "failed to unload the driver with IRQ support\n");
	return -EINVAL;
}
#else
static int max7321_remove(struct i2c_client *client)
{
	struct max7321_platform_data *pdata = client->dev.platform_data;
	struct max7321_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	if (pdata->teardown) {
		ret = pdata->teardown(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_dbg(&client->dev, "teardown failed, %d\n", ret);
	}

	ret = gpiochip_remove(&chip->gpio_chip);
	if (ret) {
		dev_err(&client->dev, "failed remove gpio_chip\n");
		return ret;
	}

	kfree(chip);
	return 0;
}
#endif

static const struct i2c_device_id max7321_id[] = {
	{ "max7321", 0 },
	{ }
};

static struct i2c_driver max7321_driver = {
	.driver = {
		.name	= "max7321",
	},
	.probe		= max7321_probe,
	.remove		= max7321_remove,
	.id_table	= max7321_id,
};

static int __init max7321_init(void)
{
	return i2c_add_driver(&max7321_driver);
}

static void __exit max7321_exit(void)
{
	i2c_del_driver(&max7321_driver);
}

MODULE_AUTHOR("Jack Ren <jack.ren@marvell.com>");
MODULE_DESCRIPTION("MAX7321 driver");
MODULE_LICENSE("GPL");

module_init(max7321_init);
module_exit(max7321_exit);

