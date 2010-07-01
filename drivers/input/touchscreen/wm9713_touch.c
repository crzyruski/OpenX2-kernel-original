/*
 *  drivers/input/touchscreen/wm9713_touch.c.
 *
 *  Author:	bin.yang@marvell.com
 *  Created:	July 26, 2006
 *  Copyright:	Marvell Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>

#include <asm/semaphore.h>
#include <mach/hardware.h>
#include <mach/pxa-regs.h>
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <sound/ac97_codec.h>
#include <linux/suspend.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <mach/hardware.h>
#include "../../../sound/soc/pxa/pxa3xx-ac97codec-pm.h"


#define ALSA_ZY_CARD_DEBUG
#undef ALSA_ZY_CARD_DEBUG

#ifdef ALSA_ZY_CARD_DEBUG
#define dbg(format, arg...) printk(format, ##arg)
#else
#define dbg(format, arg...)
#endif

#define DEBUG
#undef DEBUG
unsigned int start_time;
unsigned int end_time;
#ifdef DEBUG
unsigned int time;
#define PRINT_TIME() do {\
	time = ((end_time > start_time))?\
		(end_time - start_time)*100/325:\
		(0xffffffff - start_time + end_time)*100/325;\
	printk("\n%s:%dus\n", __FUNCTION__, time);\
} while (0)
#endif

#define EVENT_TYPE_NONE			0
#define EVENT_TYPE_PDN			1

#define PEN_DOWN 		1
#define PEN_UP			0
#define TS_SAMPLE_INTERVAL 	1

#define WM9713_GPIO_PIN_PDN	(0x1 << 13)  /* Pen down */
#define WM9713_PWR_PADCPD	(0x1 << 15)
#define WM9713_TOUCH_SAMPLE_X	1
#define WM9713_TOUCH_SAMPLE_Y	2
#define WM9713_PWR_MBIAS	(0x1 << 10)

int touch_client;

typedef struct {
	struct input_dev *idev;
	struct timer_list *timer;
	struct timer_list *reset_timer;
	int  use_count;
} codec_ts_t;

codec_ts_t codec_ts;

static struct input_dev *codec_ts_input;

#ifdef CONFIG_PM
static volatile int touch_suspend = 0 ;
#endif

static struct snd_ac97_bus pxa3xx_ac97_bus = {
	.ops = &soc_ac97_ops,
};

static struct snd_ac97 pxa3xx_ac97 = {
	.num = 0, 		/*the primary codec*/
	.bus = &pxa3xx_ac97_bus,
};

static unsigned int ac97_read(unsigned int reg)
{
	return soc_ac97_ops.read(&pxa3xx_ac97, reg);
}

static void ac97_write(unsigned int reg, unsigned int val)
{
	soc_ac97_ops.write(&pxa3xx_ac97, reg, val);
}

static void wm9713_event_ack(unsigned char event_type)
{
	unsigned short event_state = 0;

	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
	event_state = ac97_read(AC97_GPIO_STATUS);
	if (event_type == EVENT_TYPE_PDN)
		ac97_write(AC97_GPIO_STATUS,
			(event_state & (~WM9713_GPIO_PIN_PDN)));

	event_state = ac97_read(AC97_GPIO_STATUS);
	set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);
	return;
}

static void wm9713_get_event(unsigned char *event_type)
{
	unsigned short event_state = 0;
	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
	event_state = ac97_read(AC97_GPIO_STATUS);
	set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);
	if (event_state & WM9713_GPIO_PIN_PDN) {
		*event_type = EVENT_TYPE_PDN;
		return;
	}
	return;
}

static void wm9713_enable_touch(void)
{	/* enable touch functionality in the codec */
	unsigned short value;

	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
	/* power setting */
	ac97_write(AC97_POWERDOWN, 0);
	mdelay(1);
	value = ac97_read(AC97_EXTENDED_MID);
	value &= ~(WM9713_PWR_PADCPD | WM9713_PWR_MBIAS);
	ac97_write(AC97_EXTENDED_MID, value);

	/* basic touch setting */
	ac97_write(AC97_SIGMATEL_CIC2, 0xc008);
	ac97_write(AC97_SIGMATEL_CIC1, 0x6);

	/* 9713 powerdown virtual gpio setting (polarity, sticky, wakeup)
	 * 9713 gpio 2(pin45) route to IRQ
	 * Notes: Can use defaults for IRQ polarity, PENDOWN polarity in IRQ,
	 * sticky for PENDOWN in IRQ and wakeup for PENDOWN.
	 */

	value = ac97_read(AC97_GPIO_CFG);
	value &= ~(0x4);
	ac97_write(AC97_GPIO_CFG, value);

	value = ac97_read(AC97_MISC_AFE);
	value &= ~(0x4);
	ac97_write(AC97_MISC_AFE, value);

	value = ac97_read(AC97_GPIO_WAKEUP);
	value |= (0x2000);
	ac97_write(AC97_GPIO_WAKEUP, value);

	value = ac97_read(AC97_GPIO_STICKY);
	value |= (0x2000);
	ac97_write(AC97_GPIO_STICKY, value);

	set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);
}

static void wm9713_disable_touch(void)
{	/* disable touch functionality in the codec */
	unsigned short value;

	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
	/* power setting */
	value = ac97_read(AC97_EXTENDED_MID);
	value |= (WM9713_PWR_PADCPD | WM9713_PWR_MBIAS);
	ac97_write(AC97_EXTENDED_MID, value);
	set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);
}

static void acodec_get_adc_sample(unsigned short *p_sample_data,
		unsigned short adc_type, int *p_pen_down)
{
	unsigned short value;
	unsigned long wait;

	if (adc_type == WM9713_TOUCH_SAMPLE_X) {
		value = 0x202;
	} else {/* Y sample */
		value = 0x204;
	}

	ac97_write(AC97_SIGMATEL_MULTICHN, value);

	wait = 0;
	do {
		value = ac97_read(AC97_SIGMATEL_MULTICHN);
		if (!(value & 0x200))
			break;
	} while (100 > wait++);
	value = ac97_read(AC97_CSR_BDI_STATUS);
	if (value & 0x8000) {	/* means pen down */
		*p_pen_down = 1;
	} else {
		*p_pen_down = 0;
	}
	*p_sample_data = value & 0xfff;
}

/*
 * add a touch event
 */
static int codec_ts_evt_add(codec_ts_t *ts, u16 pressure, u16 x, u16 y)
{
	/* add event and remove adc src bits */
	static u16 pre_press;

	input_report_abs(ts->idev, ABS_X, x & 0xfff);
	input_report_abs(ts->idev, ABS_Y, y & 0xfff);
	if (pressure == pre_press)
		pressure--;

	pre_press = pressure;
	input_report_abs(ts->idev, ABS_PRESSURE, pressure & 0xfff);
	return 0;
}

/*
 * add a pen up event
 */
static void codec_ts_evt_release(codec_ts_t *ts)
{
	input_report_abs(ts->idev, ABS_PRESSURE, 0);
	wm9713_event_ack(EVENT_TYPE_PDN);
}

/*
 * Sample the touchscreen
 */
int ac97_poll_touch(codec_ts_t *ts)
{
	unsigned short x = 0, y = 0;
	int if_down = 0;

#ifdef DEBUG
	start_time = OSCR;
#endif

	/* get x value */
	acodec_get_adc_sample(&x, WM9713_TOUCH_SAMPLE_X, &if_down);
	dbg("x:0x%x\n", x);

	/* the pen is up */
	if (1 != if_down)
		return PEN_UP;

	/* get y vaule */
	acodec_get_adc_sample(&y, WM9713_TOUCH_SAMPLE_Y, &if_down);
	dbg("y:0x%x\n", y);

	/* the pen is up */
	if (1 != if_down)
		return PEN_UP;

	/* the pen is down, can not get the pressure value,
	 * so if pen is down, give the max pressure value
	 */
	codec_ts_evt_add(ts, 0xfff, x, y);

#ifdef DEBUG
	end_time = OSCR;
	PRINT_TIME();
#endif

	return PEN_DOWN;
}

/*
 * Use new timer to implement delay operation.
 * Invoid to use mdelay() in touch_timer_handler().
 */
static void codec_reset_handler(unsigned long step)
{
	codec_ts_t *ts = &codec_ts;

	if (step > 1)
		step = 0;
	switch (step) {
	case 0:
		GCR |= GCR_WARM_RST;
		ts->reset_timer->expires = jiffies + 1;
		ts->reset_timer->data = ++step;
		add_timer(ts->reset_timer);
		break;
	case 1:
		pxa3xx_ac97.bus->ops->reset(&pxa3xx_ac97);
		wm9713_enable_touch();
		ac97_write(AC97_GPIO_CFG, 0xFFF6);
		ac97_write(AC97_MISC_AFE, 0xFFF6);
		ac97_write(AC97_GPIO_STICKY, 0x0008);
		ts->timer->expires = jiffies + TS_SAMPLE_INTERVAL;
		add_timer(ts->timer);
		break;
	}
}

static void touch_timer_handler(unsigned long unused)
{
	codec_ts_t *ts = &codec_ts;
	int event;

	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
	if (ac97_read(AC97_RESET) != 0x6174) {
		printk(KERN_INFO "codec wake up error, retry!!\n");
		ts->reset_timer->expires = jiffies + 1;
		ts->reset_timer->data = 0;
		add_timer(ts->reset_timer);
		goto out;
	}

	event = ac97_poll_touch(ts);

	if (event == PEN_DOWN) {
		dbg("pen down!\n");
		ts->timer->expires = jiffies + TS_SAMPLE_INTERVAL;
		add_timer(ts->timer);
	} else if (event == PEN_UP) {
		dbg("pen up!\n");
		codec_ts_evt_release(ts);

	} else {
		printk(KERN_ERR "Access touch interface error!\n");
	}
out:
	set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);
	return;
}

/*
 * Start the touchscreen thread and
 * the touch digitiser.
 */
static int wm9713_touch_open(struct input_dev *idev)
{
	codec_ts_t *ts = (codec_ts_t *) &codec_ts;

#ifdef CONFIG_PM
	if (touch_suspend) {
		pr_info("touch is suspended!\n");
		return -1;
	}
#endif

	if (ts->use_count++ > 0)
		return 0;

	dbg("Touch is opened. Use count: %d\n", ts->use_count);
	ts->idev = idev;
	ts->timer = kmalloc(sizeof(struct timer_list), GFP_KERNEL);
	if (!ts->timer) {
		printk(KERN_ERR "Alloc memory error for timer!\n");
		return -ENOMEM;
	}

	init_timer(ts->timer);
	ts->timer->function = touch_timer_handler;
	ts->timer->data = 0;

	ts->reset_timer = kmalloc(sizeof(struct timer_list), GFP_KERNEL);
	if (!ts->reset_timer) {
		printk(KERN_ERR "Alloc memroy error for timer!\n");
		return -ENOMEM;
	}

	init_timer(ts->reset_timer);
	ts->reset_timer->function = codec_reset_handler;
	ts->reset_timer->data = 0;

	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
	wm9713_enable_touch();
	/* Modified by Paul Shen */
	input_report_abs(ts->idev, ABS_PRESSURE, 0);
	wm9713_event_ack(EVENT_TYPE_PDN);
	set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);

	return 0;
}

/*
 * Kill the touchscreen thread and stop
 * the touch digitiser.
 */
static void wm9713_touch_close(struct input_dev *idev)
{
	codec_ts_t *ts = (codec_ts_t *) &codec_ts;

#ifdef CONFIG_PM
	if (touch_suspend) {
		pr_info("touch is suspended!\n");
		return;
	}
#endif
	dbg("close ts input!\n");
	if (--ts->use_count == 0) {
		del_timer(ts->timer);
		if (ts->timer != NULL)
			kfree(ts->timer);
		set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
		wm9713_disable_touch();
		set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);
	}
}

/*
 * initilze the pxa touch screen
 */
static int ts_init(struct platform_device *pdev)
{
	int ret = -1;
	codec_ts_t *ts = &codec_ts;

	memset(ts, 0, sizeof(codec_ts_t));

	/* tell input system what we events we accept and register */
	codec_ts_input = input_allocate_device();
	if (codec_ts_input == NULL) {
		printk(KERN_ERR "%s: failed to allocate input dev\n",
			__FUNCTION__);
		return -ENOMEM;
	}

	codec_ts_input->name = "wm9713-touch";
	codec_ts_input->phys = "wm9713-touch/input1";
	codec_ts_input->dev.parent = &pdev->dev;

	codec_ts_input->open = wm9713_touch_open;
	codec_ts_input->close = wm9713_touch_close;
	__set_bit(EV_ABS, codec_ts_input->evbit);
	__set_bit(ABS_X, codec_ts_input->absbit);
	__set_bit(ABS_Y, codec_ts_input->absbit);
	__set_bit(ABS_PRESSURE, codec_ts_input->absbit);

	ret = input_register_device(codec_ts_input);
	if (ret) {
		printk(KERN_ERR
			"%s: unabled to register input device, ret = %d\n",
			__FUNCTION__, ret);
		return ret;
	}

	return 0;
}

static irqreturn_t wm9713_touch_irq(int irq, void *dev)
{
	unsigned char event_type = EVENT_TYPE_NONE;
	codec_ts_t *ts = &codec_ts;

	dbg("%s: enter codec event handler\n", __FUNCTION__);

#if 0
	/*if the touch is not open need not acknowledge the event*/
	if (ts->use_count > 0) {
		ts->timer->expires = jiffies + TS_SAMPLE_INTERVAL;
		if (!timer_pending(ts->timer))
			add_timer(ts->timer);
	}
#else
	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
	wm9713_get_event(&event_type);
	switch (event_type) {
	case EVENT_TYPE_PDN:
		/* if the touch is not open need not acknowledge
		 * the event
		 */
		if (ts->use_count <= 0)
			break;
		ts->timer->expires = jiffies + TS_SAMPLE_INTERVAL;
		add_timer(ts->timer);

		break;
	default:
		printk(KERN_ERR "unsupported codec event:0x%x\n", event_type);
	}
	set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);
#endif

	return IRQ_HANDLED;
}

static int __devinit wm9713_touch_probe(struct platform_device *pdev)
{
	int ret = 0;
	int irq;

	register_codec(&touch_client);
	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);

	ts_init(pdev);

	irq = platform_get_irq(pdev, 0);
	ret = request_irq(irq, wm9713_touch_irq, IRQF_TRIGGER_RISING,
			"wm9713 touch event interrupt", NULL);
	if (ret) {
		printk(KERN_ERR "Request IRQ for touch failed (%d).\n", ret);
		return ret;
	}

	set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);

	return 0;
}

#ifdef CONFIG_PM
static int wm9713_touch_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	int ret = 0;

	touch_suspend = 1;
	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
	set_codec_sub_state(touch_client, CODEC_SUB_POWER_OFF);
	return ret;
}

static int wm9713_touch_resume(struct platform_device *pdev)
{
	int ret = 0;

	touch_suspend = 0;
	set_codec_sub_state(touch_client, CODEC_SUB_POWER_ON);
	set_codec_sub_state(touch_client, CODEC_SUB_LOWPOWER);
	return ret;
}
#else
#define wm9713_touch_suspend	NULL
#define wm9713_touch_resume	NULL
#endif

static struct platform_driver wm9713_touch_driver = {
	.driver = {
		.name 	= "wm9713-touch",
	},
	.probe		= wm9713_touch_probe,
	.suspend	= wm9713_touch_suspend,
	.resume		= wm9713_touch_resume,
};

static int __init wm9713_touch_init(void)
{
	return platform_driver_register(&wm9713_touch_driver);
}

static void __exit wm9713_touch_exit(void)
{
	platform_driver_unregister(&wm9713_touch_driver);
}

module_init(wm9713_touch_init);
module_exit(wm9713_touch_exit);

MODULE_AUTHOR("bin.yang@marvell.com");
MODULE_DESCRIPTION("zylonite audio touch codec driver on SOC");
MODULE_LICENSE("GPL");

