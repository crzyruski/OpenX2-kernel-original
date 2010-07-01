/*
    max7320.c - 16-bit I/O port with interrupt and reset

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
#include <linux/i2c/max7320.h>

#include <asm/gpio.h>

#define NR_MAX7320_GPIOS	8

struct max7320_chip {
	unsigned int gpio_start;
	uint8_t io_levels;

	struct i2c_client *client;
	struct gpio_chip gpio_chip;
};

static int max7320_write(struct max7320_chip *chip, uint8_t val)
{
	return i2c_smbus_write_byte(chip->client, val);
}

static int max7320_read(struct max7320_chip *chip, uint8_t *val)
{
	int ret;

	ret =  i2c_smbus_read_byte(chip->client);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to read\n", __FUNCTION__);
		return ret;
	}

	*val = (uint8_t)ret;
	return 0;
}

static int max7320_gpio_direction_output(struct gpio_chip *gc,
		unsigned off, int val)
{
	struct max7320_chip *chip;
	uint8_t reg_val;
	int ret;

	chip = container_of(gc, struct max7320_chip, gpio_chip);

	/* set output level */
	if (val)
		reg_val = chip->io_levels | (1u << off);
	else
		reg_val = chip->io_levels & ~(1u << off);

	ret = max7320_write(chip, reg_val);
	if (ret)
		return ret;

	chip->io_levels = reg_val;

	return 0;
}

static int max7320_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct max7320_chip *chip;
	uint8_t reg_val;
	int ret;

	chip = container_of(gc, struct max7320_chip, gpio_chip);

	ret = max7320_read(chip, &reg_val);
	if (ret < 0)
		return ret;

	return (reg_val & (1u << off)) ? 1 : 0;
}

static void max7320_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct max7320_chip *chip;
	uint8_t reg_val;
	int ret;

	chip = container_of(gc, struct max7320_chip, gpio_chip);

	if (val)
		reg_val = chip->io_levels | (1u << off);
	else
		reg_val = chip->io_levels & ~(1u << off);

	ret = max7320_write(chip, reg_val);
	if (ret)
		return;

	chip->io_levels = reg_val;
}

static int max7320_init_gpio(struct max7320_chip *chip)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input  = NULL;
	gc->direction_output = max7320_gpio_direction_output;
	gc->get = max7320_gpio_get_value;
	gc->set = max7320_gpio_set_value;

	gc->base = chip->gpio_start;
	gc->ngpio = NR_MAX7320_GPIOS;
	gc->label = "max7320";

	return gpiochip_add(gc);
}

static int __devinit max7320_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct max7320_platform_data *pdata;
	struct max7320_chip *chip;
	int ret;

	pdata = client->dev.platform_data;
	if (pdata == NULL)
		return -ENODEV;

	chip = kzalloc(sizeof(struct max7320_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip->client = client;

	chip->gpio_start = pdata->gpio_base;

	/* detect if there is max7320 */
	if (!max7320_read(chip, &chip->io_levels))
		printk(KERN_INFO "max7320 is detected!\n");
	else
		printk(KERN_INFO "failed to detect max7320!\n");

	ret = max7320_init_gpio(chip);
	if (ret)
		goto out_failed;

	if (pdata->setup) {
		ret = pdata->setup(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_dbg(&client->dev, "setup failed, %d\n", ret);
	}

	i2c_set_clientdata(client, chip);
	return 0;

out_failed:
	kfree(chip);
	return ret;
}

static int max7320_remove(struct i2c_client *client)
{
	struct max7320_platform_data *pdata = client->dev.platform_data;
	struct max7320_chip *chip = i2c_get_clientdata(client);
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

static const struct i2c_device_id max7320_id[] = {
	{ "max7320", 0 },
	{ }
};

static struct i2c_driver max7320_driver = {
	.driver = {
		.name	= "max7320",
	},
	.probe		= max7320_probe,
	.remove		= max7320_remove,
	.id_table	= max7320_id,
};

static int __init max7320_init(void)
{
	return i2c_add_driver(&max7320_driver);
}

static void __exit max7320_exit(void)
{
	i2c_del_driver(&max7320_driver);
}

MODULE_AUTHOR("Jack Ren <jack.ren@marvell.com>");
MODULE_DESCRIPTION("MAX7320 driver");
MODULE_LICENSE("GPL");

module_init(max7320_init);
module_exit(max7320_exit);

