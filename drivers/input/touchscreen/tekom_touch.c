/*
 *
 *  Tekom capcitive touch screen driver
 *
 *  Copyright (C) 2009, YuhuaTel
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/freezer.h>
#include <linux/proc_fs.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include <mach/mfp.h>
#include <mach/gpio.h>
#include <linux/i2c.h>

#define TOUCH_INT 		MFP_PIN_GPIO6_2 /* low active */
#define TOUCH_RESET 	MFP_PIN_GPIO3_2

struct tekom_touch_data {
	struct input_dev 	*tekom_ts_input_dev;
	int				pen_down;
	int				use_count;
	int				poll_jiffies;
	struct timer_list 	poll_timer;
	int				debug_level;
	struct i2c_client 	*client;
};

static void tekom_touch_gpio_init(void)
{
	pxa3xx_mfp_set_afds(TOUCH_INT, MFP_AF0, MFP_DS04X);
	gpio_direction_input(TOUCH_INT);
	pxa3xx_mfp_set_lpm(TOUCH_INT, MFP_LPM_PULL_HIGH);

	pxa3xx_mfp_set_afds(TOUCH_RESET, MFP_AF0, MFP_DS04X);
	gpio_direction_output(TOUCH_RESET, GPIO_LEVEL_LOW);
}

static int tekom_touch_read(struct tekom_touch_data *p_touch)
{
	int ret = -EIO;
	struct input_dev* p_input_dev = p_touch->tekom_ts_input_dev;
	int pen_down_state; // = !gpio_get_value(TOUCH_INT);
	u8 data[6] = {0};
	u16 rx = 0, ry = 0;

	ret = i2c_master_recv(p_touch->client, data, 6);
	if (data[0]&0x3)
		pen_down_state = 1;
	else
		pen_down_state = 0;
		
	if (pen_down_state) {
		rx = (data[1]<<8) | data[2];
		ry = (data[3]<<8) | data[4];
#if 1
		rx = rx*480/360 - 1;
		ry = ry*854/640 - 1;
#endif
		//printk("tekom_touch_read %d %d\n", rx, ry);

		p_touch->pen_down = 1;
		input_report_abs(p_input_dev, ABS_X, rx);
		input_report_abs(p_input_dev, ABS_Y, ry);
		input_report_key(p_input_dev, BTN_TOUCH, 1);
		input_sync(p_input_dev);
	} else {
		if (p_touch->pen_down) { /* pen up */
			input_report_key(p_input_dev, BTN_TOUCH, 0);
			input_sync(p_input_dev);
			p_touch->pen_down = 0;
		}
	}
	ret = 0;
	
out:
	if (ret<0)
		printk(KERN_ERR"tekom_touch_read err %d\n", ret);
	return ret;
}

static void tekom_poll_timer_handler(unsigned long data)
{
	struct tekom_touch_data *p_touch = (struct tekom_touch_data *)data;	
	tekom_touch_read(p_touch);
	if (p_touch->pen_down)
		mod_timer(&p_touch->poll_timer, jiffies+p_touch->poll_jiffies);
}

static int tekom_ts_input_open(struct input_dev *idev)
{
	struct tekom_touch_data *p_touch = (struct tekom_touch_data *)input_get_drvdata(idev);
	p_touch->use_count++;	
	return 0;
}

static void tekom_ts_input_close(struct input_dev *idev)
{
	struct tekom_touch_data *p_touch = (struct tekom_touch_data *)input_get_drvdata(idev);
	p_touch->use_count--;	
}

static irqreturn_t tekom_touch_interrupt(int irq, void *dev_id) 
{
	struct tekom_touch_data *p_tekom_touch = dev_id;
	//printk("tekom_touch_interrupt\n");
	
	if ((p_tekom_touch->use_count > 0) &&(!p_tekom_touch->pen_down)) {
		tekom_poll_timer_handler(p_tekom_touch);
	}
	
	return IRQ_HANDLED;
}


/* sys fs */
static ssize_t pollJiffies_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{					
	int poll_jiffies;
	struct tekom_touch_data *p_tekom_touch = (struct tekom_touch_data*)dev_get_drvdata(dev);
	
	sscanf(buf, "%d", &poll_jiffies);
	if(poll_jiffies && poll_jiffies<=20) 
		p_tekom_touch->poll_jiffies= poll_jiffies;
	else
		printk("Valid parameter[1-20]\n");
	return count;
}
static ssize_t pollJiffies_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	struct tekom_touch_data *p_tekom_touch = (struct tekom_touch_data*)dev_get_drvdata(dev);	
	return sprintf(buf, "%d\n", p_tekom_touch->poll_jiffies);
}
static DEVICE_ATTR(pollJiffies,0644,pollJiffies_show,pollJiffies_store);
static ssize_t debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct tekom_touch_data *p_tekom_touch = (struct tekom_touch_data*)dev_get_drvdata(dev);	
	sscanf(buf, "%d", &p_tekom_touch->debug_level);
	return count;
}
static ssize_t debug_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	struct tekom_touch_data *p_tekom_touch = (struct tekom_touch_data*)dev_get_drvdata(dev);	
	return sprintf(buf, "%d\n", p_tekom_touch->debug_level);
}
static DEVICE_ATTR(debug,0644,debug_show,debug_store);

static struct attribute *tekom_touch_attributes[] = {	
	&dev_attr_pollJiffies.attr,
	&dev_attr_debug.attr,
	NULL,
};
static struct attribute_group tekom_touch_attr_group ={
	.attrs = tekom_touch_attributes,
};

static int tekom_ts_probe(struct i2c_client *client, const struct i2c_device_id * id)
{
	int ret = -EIO;
	struct tekom_touch_data *p_tekom_touch;
	struct input_dev 	*tekom_ts_input_dev;	

	/* register input device */
	tekom_ts_input_dev = input_allocate_device();	
      	tekom_ts_input_dev->name = "Tekom capcitive touchscreen";
	tekom_ts_input_dev->open = tekom_ts_input_open;
	tekom_ts_input_dev->close = tekom_ts_input_close;
	__set_bit(EV_ABS, tekom_ts_input_dev->evbit);
	__set_bit(ABS_X, tekom_ts_input_dev->absbit);
	__set_bit(ABS_Y, tekom_ts_input_dev->absbit);
	__set_bit(ABS_PRESSURE, tekom_ts_input_dev->absbit);
	set_bit(EV_KEY, tekom_ts_input_dev->evbit);
	set_bit(BTN_TOUCH, tekom_ts_input_dev->keybit);

	/* Malloc p_touch context */
	p_tekom_touch = kzalloc(sizeof(struct tekom_touch_data), GFP_KERNEL);
	if (!p_tekom_touch) {
		ret = -ENOMEM;
		goto err;
	}
	
	p_tekom_touch->tekom_ts_input_dev = tekom_ts_input_dev;
	platform_set_drvdata(client, p_tekom_touch);
	input_set_drvdata(tekom_ts_input_dev, p_tekom_touch);
	ret = input_register_device(tekom_ts_input_dev);

	p_tekom_touch->client = client;
	p_tekom_touch->pen_down = 0;
	p_tekom_touch->use_count = 0;
	p_tekom_touch->poll_jiffies = 5;

	tekom_touch_gpio_init();
	ret = request_irq(IRQ_GPIO(MFP2GPIO(TOUCH_INT)), tekom_touch_interrupt, 
		IRQF_TRIGGER_FALLING, "Tekom_capcitive", p_tekom_touch);

	ret = request_irq(IRQ_GPIO(MFP2GPIO(SLIDE_INT)), slide_interrupt, 
		IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "SLIDE_INT", p_tekom_touch);

	init_timer(&p_tekom_touch->poll_timer);
	p_tekom_touch->poll_timer.data    = (unsigned long)p_tekom_touch;
	p_tekom_touch->poll_timer.function = tekom_poll_timer_handler;

	ret = sysfs_create_group(&client->dev.kobj, &tekom_touch_attr_group);
	
	printk("Android tekom capcitive touchscreen driver register succ\n");
	return 0;

err:	
	return ret;
}

static int tekom_ts_resume(struct i2c_client * client)
{	
	struct tekom_touch_data *p_tekom_touch = 
		(struct tekom_touch_data *)platform_get_drvdata(client);	
	mod_timer(&p_tekom_touch->poll_timer, jiffies+HZ); /* timer for read to trigger irq */
	return 0;
}

static int tekom_ts_suspend(struct i2c_client * client, pm_message_t state)
{
	struct tekom_touch_data *p_tekom_touch = 
		(struct tekom_touch_data *)platform_get_drvdata(client);

	// todo
	
	return 0;
}

static const struct i2c_device_id tekom_touch_i2c_id[] = {
	{"tekom_capcitive", 0 },
	{ }
};
static struct i2c_driver tekom_touch_i2c = {
	.driver = {
		.name	= "tekom_capcitive",
	},
	.id_table	= tekom_touch_i2c_id,
	.probe		= tekom_ts_probe,
	.resume 	= tekom_ts_resume,
	.suspend 	= tekom_ts_suspend,
};

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)
static int __init tekom_ts_init( void )
{
	i2c_add_driver(&tekom_touch_i2c);
	return 0;
}

static void __exit tekom_ts_exit( void )
{	
}

module_init(tekom_ts_init);
module_exit(tekom_ts_exit);

MODULE_AUTHOR("YuhuaTel");
MODULE_DESCRIPTION("Tekom capcitive touch screen driver");
MODULE_LICENSE("GPL");

