/* 
 * watch dog damon 
 * Copyright@yuhuatel 2009
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include <mach/pxa-regs.h>
#include <mach/pxa3xx-regs.h>

struct pxa3xx_watchdog_data {
	struct timer_list watchdog_timer;
	int enable;
	int timeoutSec; /* wt timeout */
	int pollSec; /* clear wt freq */
	unsigned int pollCount;
};

static ssize_t poll_count_show(struct device *dev, struct device_attribute *attr, char * buf)
{
	struct pxa3xx_watchdog_data *pWt = (struct pxa3xx_watchdog_data*)dev_get_drvdata(dev);	
	return sprintf(buf, "%d\n", pWt->pollCount);
}
static DEVICE_ATTR(count, 0644, poll_count_show, NULL);

static struct attribute *pxa3xx_wt_attributes[] = {
	&dev_attr_count.attr,
	NULL,
};
static struct attribute_group pxa3xx_wt_attr_group ={
	.attrs = pxa3xx_wt_attributes,
};

static inline void pxa3xx_hw_watchdog_set(struct pxa3xx_watchdog_data *pWt)
{
	OSMR3 = OSCR + 325*10000*pWt->timeoutSec; /* 3.25M clk */
}

static void pxa3xx_watchdog_timer_handler(unsigned long data)
{
	struct pxa3xx_watchdog_data *pWt = (struct pxa3xx_watchdog_data *)data;
	if (pWt->enable) {
		pWt->pollCount++;
		pxa3xx_hw_watchdog_set(pWt);
		mod_timer(&pWt->watchdog_timer, jiffies+pWt->pollSec*HZ);
	}
}

static int pxa3xx_watchdog_enable(struct pxa3xx_watchdog_data *pWt, int enable)
{
	if (enable!=pWt->enable) {
		if (enable) {
			OWER = OWER_WME;
			pxa3xx_hw_watchdog_set(pWt);			
			mod_timer(&pWt->watchdog_timer, jiffies+pWt->pollSec*HZ);	
		} else {
			OWER &= ~OWER_WME;
			del_timer(&pWt->watchdog_timer);
		}
		
		pWt->enable = enable;
	}
	
	return 0;
}

static int pxa3xx_watchdog_resume(struct platform_device *pdev)
{
	struct pxa3xx_watchdog_data *data = platform_get_drvdata(pdev);	
	pxa3xx_watchdog_enable(data, 1);
	return 0;
}

static int pxa3xx_watchdog_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct pxa3xx_watchdog_data *data = platform_get_drvdata(pdev);
	pxa3xx_watchdog_enable(data, 0);
	return 0;
}

static int pxa3xx_watchdog_probe(struct platform_device *pdev)
{
	int ret;
	struct pxa3xx_watchdog_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}

	platform_set_drvdata(pdev, data);

	data->pollSec = 3;
	data->timeoutSec = 5;
	OSSR = OSSR_M3; /* clear */	

	init_timer(&data->watchdog_timer);
	data->watchdog_timer.data    = (unsigned long)data;
	data->watchdog_timer.function = pxa3xx_watchdog_timer_handler;
	pxa3xx_watchdog_enable(data, 1);

	ret = sysfs_create_group(&pdev->dev.kobj, &pxa3xx_wt_attr_group);
	printk("Yuhua pxa3xx board watchdog register succ\n");
	return 0;

err_data_alloc_failed:
	printk("pxa3xx_watchdog_probe fail %d\n", ret);
	return ret;
}

static struct platform_driver pxa3xx_watchdog_device = {
	.probe		= pxa3xx_watchdog_probe,
	.resume 		= pxa3xx_watchdog_resume,
	.suspend		= pxa3xx_watchdog_suspend,
	.driver = {
		.name = "pxa3xx-watchdog"
	}
};

static int __init pxa3xx_watchdog_init(void)
{
	return platform_driver_register(&pxa3xx_watchdog_device);
}

static void __exit pxa3xx_watchdog_exit(void)
{
	platform_driver_unregister(&pxa3xx_watchdog_device);
}

module_init(pxa3xx_watchdog_init);
module_exit(pxa3xx_watchdog_exit);

MODULE_AUTHOR("YuhuaTel");
MODULE_DESCRIPTION("Watchdog driver for Yuhua Pxa3xx board");

