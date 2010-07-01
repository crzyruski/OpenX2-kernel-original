/* drivers/video/backlight/pxa3xx_gpio_bl.c
 *
 * Author:	Mingwei Wang <mwwang@marvell.com>
 * Created:	Jan 12, 2007
 * Copyright:	Marvell International Ltd. All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/backlight.h>
#include <linux/fb.h>

#include <mach/hardware.h>
#include <mach/pxa-regs.h>
#include <mach/gpio.h>
#include <mach/micco.h>

#define	LCD_LED_MAX	0x1
#define	LCD_LED_MIN	0x0

struct gpio_bl_device {
	u32 gpio_backlight;
	u32 current_intensity;
	u32 gpio_bl_suspended;
};

static int gpio_bl_send_intensity(struct backlight_device *bd)
{
	struct gpio_bl_device *gpio_bl = dev_get_drvdata(&bd->dev);
	int intensity = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
	if (gpio_bl->gpio_bl_suspended)
		intensity = 0;

	gpio_set_value(gpio_bl->gpio_backlight, intensity);

	gpio_bl->current_intensity = intensity;

	return 0;
}

#ifdef CONFIG_PM
static int gpio_bl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	struct gpio_bl_device *gpio_bl = dev_get_drvdata(&bd->dev);

	gpio_bl->gpio_bl_suspended = 1;
	gpio_bl_send_intensity(bd);
	return 0;
}

static int gpio_bl_resume(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	struct gpio_bl_device *gpio_bl = dev_get_drvdata(&bd->dev);

	gpio_bl->gpio_bl_suspended = 0;
	gpio_bl_send_intensity(bd);
	return 0;
}
#else
#define gpio_bl_suspend	NULL
#define gpio_bl_resume	NULL
#endif

static int gpio_bl_set_intensity(struct backlight_device *bd)
{
	gpio_bl_send_intensity(bd);
	return 0;
}

static int gpio_bl_get_intensity(struct backlight_device *bd)
{
	struct gpio_bl_device *gpio_bl = dev_get_drvdata(&bd->dev);

	return gpio_bl->current_intensity;
}

static struct backlight_ops gpio_bl_ops = {
	.get_brightness = gpio_bl_get_intensity,
	.update_status  = gpio_bl_set_intensity,
};

static int gpio_bl_probe(struct platform_device *pdev)
{
	struct backlight_device *bd;
	struct gpio_bl_device *gpio_bl;
	struct resource *res;
	int ret = 0;

	gpio_bl = kzalloc(sizeof(struct gpio_bl_device), GFP_KERNEL);
	if (gpio_bl == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no io resource defined\n");
		ret = -ENODEV;
		return ret;
	}
	gpio_bl->gpio_backlight = res->start;

	bd = backlight_device_register("pxa3xx_gpio_bl", &pdev->dev, gpio_bl,
		    &gpio_bl_ops);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	platform_set_drvdata(pdev, bd);

	bd->props.power = FB_BLANK_UNBLANK;
	bd->props.brightness = LCD_LED_MAX;
	bd->props.max_brightness = LCD_LED_MAX;
	gpio_bl_send_intensity(bd);

	return 0;
}

static int gpio_bl_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	bd->props.brightness = 0;
	bd->props.power = 0;
	gpio_bl_send_intensity(bd);

	backlight_device_unregister(bd);

	return 0;
}

static struct platform_driver gpio_bl_driver = {
	.probe		= gpio_bl_probe,
	.remove		= gpio_bl_remove,
	.suspend	= gpio_bl_suspend,
	.resume		= gpio_bl_resume,
	.driver		= {
		.name	= "pxa3xx_gpio_bl",
	},
};

static int __init backlight_init(void)
{
	return platform_driver_register(&gpio_bl_driver);
}

static void __exit backlight_exit(void)
{
	platform_driver_unregister(&gpio_bl_driver);
}

late_initcall(backlight_init);
module_exit(backlight_exit);
MODULE_LICENSE("GPL");

