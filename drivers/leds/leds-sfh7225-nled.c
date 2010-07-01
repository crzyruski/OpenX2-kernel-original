/*
 * sfh7225 notification led driver based on micco
 *
 * Copyright (C) 2008 Marvell Corporation
 * Kevin Liu <kevin.liu@marvell.com>
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2008 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

#include <mach/hardware.h>
#include <mach/pxa-regs.h>

#include <mach/micco.h>

#define	NLED_ON	0xff
#define NLED_OFF 0x00
#define NLED_ENABLE 0x08
#define NLED_DISABLE 0x00

const int NLEDCYCLETIME = 1280;
const int NLEDTOTALCYCLES = 32;
const int NLEDMSPERCYCLE = 40;
const int NLEDMAXBLANKTIME = 7000;

static void sfh7225_nled_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	int status = -EINVAL;
	if (value == LED_FULL) {
		if (micco_write(MICCO_LEDPC_CONTROL1, NLED_ON))
			goto cleanup;
		if (micco_write(MICCO_LEDPC_CONTROL2, NLED_ON))
			goto cleanup;
		if (micco_write(MICCO_LEDPC_CONTROL3, NLED_ON))
			goto cleanup;
		if (micco_write(MICCO_LEDPC_CONTROL4, NLED_ON))
			goto cleanup;
		if (micco_write(MICCO_LEDPC_CONTROL5, NLED_ENABLE))
			goto cleanup;
		status = 0;
	} else if (value == LED_OFF) {
		if (micco_write(MICCO_LEDPC_CONTROL1, NLED_OFF))
			goto cleanup;
		if (micco_write(MICCO_LEDPC_CONTROL2, NLED_OFF))
			goto cleanup;
		if (micco_write(MICCO_LEDPC_CONTROL3, NLED_OFF))
			goto cleanup;
		if (micco_write(MICCO_LEDPC_CONTROL4, NLED_OFF))
			goto cleanup;
		if (micco_write(MICCO_LEDPC_CONTROL5, NLED_DISABLE))
			goto cleanup;
		status = 0;

	} else {
		printk(KERN_WARNING "sfh7225_nled_set: the value for brightness is invalid\n");
	}
cleanup:
	if (status != 0)
		printk(KERN_WARNING "sfh7225_nled_set failed\n");
}

static int sfh7225_nled_blink(struct led_classdev *led_cdev,
		unsigned long *delay_on,
		unsigned long *delay_off)
{
	int status = -EINVAL;
	int blink_on_cycle, blink_off_cycle;
	unsigned int LEDReg = 0;
	int i;

	if ((*delay_on <= 0) || (*delay_off <= 0)) {
		printk(KERN_WARNING "sfh7225_nled_blink: the value for delay on or delay off should not be zero or negative\n");
		return status;
	}
	if ((*delay_on + *delay_off) != NLEDCYCLETIME) {
		printk(KERN_WARNING "sfh7225_nled_blink: use timer trigger to blink\n");
		return status;
	}
	printk(KERN_WARNING "sfh7225_nled_blink: use hardware acceleration to blink\n");

	blink_on_cycle = *delay_on / NLEDMSPERCYCLE;
	blink_off_cycle = *delay_off / NLEDMSPERCYCLE;

	for (i = 0; i < blink_on_cycle; i++)
		LEDReg |= (1<<i);
	if (micco_write(MICCO_LEDPC_CONTROL1, (u8)LEDReg))
		goto cleanup;
	if (micco_write(MICCO_LEDPC_CONTROL2, (u8)(LEDReg>>8)))
		goto cleanup;
	if (micco_write(MICCO_LEDPC_CONTROL3, (u8)(LEDReg>>16)))
		goto cleanup;
	if (micco_write(MICCO_LEDPC_CONTROL4, (u8)(LEDReg>>24)))
		goto cleanup;
	if (micco_write(MICCO_LEDPC_CONTROL5, NLED_ENABLE))
		goto cleanup;
	status = 0;

cleanup:
	if (status != 0)
		printk(KERN_WARNING "sfh7225_nled_blink failed\n");
	return status;

}

static struct led_classdev sfh7225_nled = {
	.name                   = "sfh7225_nled",
	.brightness_set         = sfh7225_nled_set,
	.blink_set              = sfh7225_nled_blink,
};

#ifdef CONFIG_PM
static int sfh7225_nled_suspend(struct platform_device *dev, pm_message_t state)
{
	led_classdev_suspend(&sfh7225_nled);
	return 0;
}

static int sfh7225_nled_resume(struct platform_device *dev)
{
	led_classdev_resume(&sfh7225_nled);
	return 0;
}
#endif

static int sfh7225_nled_probe(struct platform_device *pdev)
{
	int ret;

	ret = led_classdev_register(&pdev->dev, &sfh7225_nled);
	if (ret < 0)
		led_classdev_unregister(&sfh7225_nled);

	return ret;
}

static int sfh7225_nled_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&sfh7225_nled);
	return 0;
}


static struct platform_driver sfh7225_nled_driver = {
	.probe          = sfh7225_nled_probe,
	.remove         = sfh7225_nled_remove,
#ifdef CONFIG_PM
	.suspend        = sfh7225_nled_suspend,
	.resume         = sfh7225_nled_resume,
#endif
	.driver         = {
		.name           = "sfh7225_nled",
		.owner          = THIS_MODULE,
	},
};

static int __init sfh7225_nled_init(void)
{
	return platform_driver_register(&sfh7225_nled_driver);
}

static void __exit sfh7225_nled_exit(void)
{
	platform_driver_unregister(&sfh7225_nled_driver);
}

module_init(sfh7225_nled_init);
module_exit(sfh7225_nled_exit);

MODULE_DESCRIPTION("sfh7225 notification LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sfh7225-nled");

