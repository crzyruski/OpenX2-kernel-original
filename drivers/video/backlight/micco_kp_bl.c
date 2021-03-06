/* drivers/video/backlight/micco_bl.c
 *
 * Author:	Yin, Fengwei
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

#include <mach/pxa-regs.h>
#include <mach/micco.h>

#define KEYPAD_LED_MAX	0x7f
#define	KEYPAD_LED_MIN	0x0

static int miccobl_suspended;
static int current_intensity;

static int miccobl_send_intensity(struct backlight_device *bd)
{
	int intensity = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
	if (miccobl_suspended)
		intensity = 0;

	micco_write(MICCO_LED1_CONTROL, intensity);
	micco_write(MICCO_LED2_CONTROL, intensity);

	current_intensity = intensity;

	return 0;
}

#ifdef CONFIG_PM
static int miccobl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	miccobl_suspended = 1;
	miccobl_send_intensity(bd);
	return 0;
}

static int miccobl_resume(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	miccobl_suspended = 0;
	miccobl_send_intensity(bd);
	return 0;
}
#else
#define miccobl_suspend	NULL
#define miccobl_resume	NULL
#endif

static int miccobl_set_intensity(struct backlight_device *bd)
{
	miccobl_send_intensity(bd);
	return 0;
}

static int miccobl_get_intensity(struct backlight_device *bd)
{
	return current_intensity;
}

static struct backlight_ops miccobl_ops = {
	.get_brightness = miccobl_get_intensity,
	.update_status  = miccobl_set_intensity,
};

static int __init miccobl_probe(struct platform_device *pdev)
{
	struct backlight_device *bd;

	bd = backlight_device_register("micco-kp-bl", &pdev->dev, NULL,
		    &miccobl_ops);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	platform_set_drvdata(pdev, bd);

	bd->props.power = FB_BLANK_UNBLANK;
	bd->props.brightness = KEYPAD_LED_MIN;
	bd->props.max_brightness = KEYPAD_LED_MAX;
	miccobl_send_intensity(bd);

	return 0;
}

static int miccobl_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	bd->props.brightness = 0;
	bd->props.power = 0;
	miccobl_send_intensity(bd);

	backlight_device_unregister(bd);

	return 0;
}

static struct platform_driver miccobl_driver = {
	.probe		= miccobl_probe,
	.remove		= miccobl_remove,
	.suspend	= miccobl_suspend,
	.resume		= miccobl_resume,
	.driver		= {
		.name	= "micco-kp-bl",
	},
};

static int __init backlight_init(void)
{
	return platform_driver_register(&miccobl_driver);
}

static void __exit backlight_exit(void)
{
	platform_driver_unregister(&miccobl_driver);
}

late_initcall(backlight_init);
module_exit(backlight_exit);
MODULE_LICENSE("GPL");
