/* drivers/video/backlight/pxa3xx_pwm_bl.c
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
#include <linux/clk.h>

#include <mach/hardware.h>
#include <mach/pxa-regs.h>
#include <mach/micco.h>

#define	LCD_LED_MAX	0x3ff
#define	LCD_LED_MIN	0x0

#define PWMCR		0x0
#define PWMDCR		0x4
#define PWMPCR		0x8

struct pwm_bl_device {
	u32 memio_base;
	u32 current_intensity;
	u32 pwm_bl_suspended;
	int enabled;
	struct clk *pwm0_clk;
	struct clk *pwm1_clk;
};

static int switch_pwm_clock(struct pwm_bl_device *pwm_bl, int enable)
{
	if (enable) {
		if (pwm_bl->enabled == 0) {
			pwm_bl->enabled = 1;
			clk_enable(pwm_bl->pwm0_clk);
			clk_enable(pwm_bl->pwm1_clk);
		}
	} else {
		clk_disable(pwm_bl->pwm1_clk);
		clk_disable(pwm_bl->pwm0_clk);
		pwm_bl->enabled = 0;
	}
	return 0;
}

static void lcd_backlight_power_off(struct backlight_device *bd)
{
	struct pwm_bl_device *pwm_bl = dev_get_drvdata(&bd->dev);
	unsigned long flags;

	local_irq_save(flags);
	__raw_writel(0x00, pwm_bl->memio_base + PWMCR);
	__raw_writel(0x00, pwm_bl->memio_base + PWMDCR);
	__raw_writel(0x00, pwm_bl->memio_base + PWMPCR);
	local_irq_restore(flags);
}

static int pwm_bl_send_intensity(struct backlight_device *bd)
{
	struct pwm_bl_device *pwm_bl = dev_get_drvdata(&bd->dev);
	int intensity = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
	if (pwm_bl->pwm_bl_suspended)
		intensity = 0;

	if (intensity == 0) {
		lcd_backlight_power_off(bd);
		switch_pwm_clock(pwm_bl, 0);
	} else {
		switch_pwm_clock(pwm_bl, 1);
		__raw_writel(0x08, pwm_bl->memio_base + PWMCR);
		__raw_writel(intensity, pwm_bl->memio_base + PWMDCR);
		__raw_writel(0x3ff, pwm_bl->memio_base + PWMPCR);
	}

	pwm_bl->current_intensity = intensity;

	return 0;
}

#ifdef CONFIG_PM
static int pwm_bl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	struct pwm_bl_device *pwm_bl = dev_get_drvdata(&bd->dev);

	pwm_bl->pwm_bl_suspended = 1;
	pwm_bl_send_intensity(bd);
	return 0;
}

static int pwm_bl_resume(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	struct pwm_bl_device *pwm_bl = dev_get_drvdata(&bd->dev);

	pwm_bl->pwm_bl_suspended = 0;
	pwm_bl_send_intensity(bd);
	return 0;
}
#else
#define pwm_bl_suspend	NULL
#define pwm_bl_resume	NULL
#endif

static int pwm_bl_set_intensity(struct backlight_device *bd)
{
	pwm_bl_send_intensity(bd);
	return 0;
}

static int pwm_bl_get_intensity(struct backlight_device *bd)
{
	struct pwm_bl_device *pwm_bl = dev_get_drvdata(&bd->dev);

	return pwm_bl->current_intensity;
}

static struct backlight_ops pwm_bl_ops = {
	.get_brightness = pwm_bl_get_intensity,
	.update_status  = pwm_bl_set_intensity,
};

static int pwm_bl_probe(struct platform_device *pdev)
{
	struct backlight_device *bd;
	struct pwm_bl_device *pwm_bl;
	struct resource *res;
	int ret = 0;
	char name[20];

	pwm_bl = kzalloc(sizeof(struct pwm_bl_device), GFP_KERNEL);
	if (pwm_bl == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory");
		return -ENOMEM;
	}

	pwm_bl->pwm0_clk = clk_get(NULL, "PWM0CLK");
	if (pwm_bl->pwm0_clk == NULL)
		return -EINVAL;
	pwm_bl->pwm1_clk = clk_get(NULL, "PWM1CLK");
	if (pwm_bl->pwm1_clk == NULL)
		return -EINVAL;
	pwm_bl->enabled = 0;

	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no io resource defined\n");
		ret = -ENODEV;
		return ret;
	}
	pwm_bl->memio_base = ioremap(res->start, res->end - res->start + 1);

	if (pdev->id != -1)
		sprintf(name, "pxa3xx_pwm_bl%d", pdev->id);
	else
		sprintf(name, "pxa3xx_pwm_bl");

	bd = backlight_device_register(name, &pdev->dev, pwm_bl,
		    &pwm_bl_ops);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	platform_set_drvdata(pdev, bd);

	bd->props.power = FB_BLANK_UNBLANK;
	bd->props.brightness = LCD_LED_MAX;
	bd->props.max_brightness = LCD_LED_MAX;
	pwm_bl_send_intensity(bd);

	return 0;
}

static int pwm_bl_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	bd->props.brightness = 0;
	bd->props.power = 0;
	pwm_bl_send_intensity(bd);

	backlight_device_unregister(bd);

	return 0;
}

static struct platform_driver pwm_bl_driver = {
	.probe		= pwm_bl_probe,
	.remove		= pwm_bl_remove,
	.suspend	= pwm_bl_suspend,
	.resume		= pwm_bl_resume,
	.driver		= {
		.name	= "pxa3xx_pwm_bl",
	},
};

static int __init backlight_init(void)
{
	return platform_driver_register(&pwm_bl_driver);
}

static void __exit backlight_exit(void)
{
	platform_driver_unregister(&pwm_bl_driver);
}

late_initcall(backlight_init);
module_exit(backlight_exit);
MODULE_LICENSE("GPL");
