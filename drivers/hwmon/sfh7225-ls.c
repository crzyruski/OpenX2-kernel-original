/*
 * sfh7225 light sensor driver based on micco
 *
 * Copyright (C) 2008 Marvell Corporation
 * Kevin Liu <kevin.liu@marvell.com>
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2008 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/hwmon.h>
#include <linux/mutex.h>
#include <mach/micco.h>
#include <linux/delay.h>

static ssize_t sfh7225_sense_light(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 lightvalue, auto_control, man_control, man_control_orig;
	int status = -EINVAL;
	/*ensure ADC is enabled*/
	micco_read(MICCO_ADC_MAN_CONTROL, &man_control_orig);
	if (!(man_control_orig & 0x10)) {
		man_control = man_control_orig | 0x10;
		if (micco_write(MICCO_ADC_MAN_CONTROL, man_control))
			goto cleanup;
	}
	/*ensure AUTO5 is enabled*/
	micco_read(MICCO_ADC_AUTO_CONTROL_2, &auto_control);
	if (!(auto_control & 0x02)) {
		auto_control |= 0x02;
		if (micco_write(MICCO_ADC_AUTO_CONTROL_2, auto_control))
			goto cleanup;
	}
	if (micco_read(MICCO_AUTO5_RES, &lightvalue))
		goto cleanup;
	status = sprintf(buf, "%u\n", lightvalue);
	if (!(man_control_orig & 0x10))
		micco_write(MICCO_ADC_MAN_CONTROL, man_control_orig);
cleanup:
	if (status < 0)
		printk(KERN_WARNING "sfh7225_sense_light failed\n");
	return status;

}

static DEVICE_ATTR(lightness, 0444, sfh7225_sense_light, NULL);

static int sfh7225_ls_probe(struct platform_device *pdev)
{
	int status;
	if (IS_ERR(hwmon_device_register(&pdev->dev))) {
		printk(KERN_WARNING "hwmon_device_register failed.\n");
		status = -1;
		goto out_dev_reg_failed;
	}
	status = device_create_file(&pdev->dev, &dev_attr_lightness);
	if (status) {
		printk(KERN_WARNING "device_create_file failed.\n");
		goto out_dev_create_file_failed;
	}
	return 0;

out_dev_create_file_failed:
	device_remove_file(&pdev->dev, &dev_attr_lightness);
	hwmon_device_unregister(&pdev->dev);
out_dev_reg_failed:
	return status;
}

static int sfh7225_ls_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_lightness);
	hwmon_device_unregister(&pdev->dev);
	return 0;
}


static struct platform_driver sfh7225_ls_driver = {
	.probe          = sfh7225_ls_probe,
	.remove         = sfh7225_ls_remove,
	.driver         = {
		.name           = "sfh7225_ls",
		.owner          = THIS_MODULE,
	},
};


static int __init sfh7225_ls_init(void)
{

	return platform_driver_register(&sfh7225_ls_driver);

}

static void __exit sfh7225_ls_exit(void)
{
	platform_driver_unregister(&sfh7225_ls_driver);
}

module_init(sfh7225_ls_init);
module_exit(sfh7225_ls_exit);

MODULE_DESCRIPTION("sfh7225 light sensor driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sfh7225-ls");

