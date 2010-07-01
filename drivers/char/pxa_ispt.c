/*
 *  drivers/char/pxa_ispt.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.

 *  (C) Copyright 2008 Marvell International Ltd.
 *  All Rights Reserved
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>

#include <asm/types.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>

#include <mach/ssp.h>
#include <mach/regs-ssp.h>
#include <mach/pxa_ispt.h>
#include <mach/dvfm.h>

extern struct kobject *power_kobj;

static struct ssp_dev ispt_ssp;
static int ispt_ssp_inited = 0;
static int ispt_switch = 0;


static ssize_t ispt_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", ispt_switch);
}

static ssize_t ispt_store(struct kobject *kobj, struct kobj_attribute *attre, const char *buf, size_t len)
{
	sscanf(buf, "%u", &ispt_switch);
	return len;
}

static struct kobj_attribute ispt_attr = {
	.attr	= {
		.name = __stringify(ispt),
		.mode = 0644,
	},
	.show	= ispt_show,
	.store	= ispt_store,
};

static struct attribute * g[] = {
	&ispt_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

int ispt_power_msg(u32 msg)
{
	u32 data;

	if (!(ispt_ssp_inited && ispt_switch))
		return 0;
	/* data is a 16-bit half word */
	data = (ISPT_DEFAULT_HEADER << 12) | (ISPT_DEFAULT_LENGTH << 10)
		| (msg & ISPT_DEFAULT_MASK);
	ssp_write_word(&ispt_ssp, data);
	
	return 0;
}
EXPORT_SYMBOL(ispt_power_msg);

int ispt_dvfm_msg(u32 old, u32 new)
{
	u32 data = 0;

	if (!(ispt_ssp_inited && ispt_switch))
		return 0;
	if ((old == 0) && (new != 0))
		data = CT_P_PP_D0CS_NOT_IN_USE;
	else if ((old != 0) && (new != 0)) {
		switch (new) {
		case 1:
			data = CT_P_PP_1_IN_USE;
			break;
		case 2:
			data = CT_P_PP_2_IN_USE;
			break;
		case 3:
			data = CT_P_PP_3_IN_USE;
			break;
		case 4:
			data = CT_P_PP_4_IN_USE;
			break;
		}
	} else if (new == 0)
		data = CT_P_PP_D0CS_IN_USE;
	/* data is a 16-bit half word */
	data &= ISPT_DEFAULT_MASK;
	data |= (ISPT_DEFAULT_HEADER << 12) | (ISPT_DEFAULT_LENGTH << 10);
	ssp_write_word(&ispt_ssp, data);
	
	return 0;
}
EXPORT_SYMBOL(ispt_dvfm_msg);

int ispt_driver_msg(u32 msg, u32 param)
{
	u32 data;

	if (!(ispt_ssp_inited && ispt_switch))
		return 0;
	/* data is a 16-bit half word */
	data = (param & ISPT_DRIVER_MASK) | (msg & ISPT_DEFAULT_MASK);
	data |= (ISPT_DEFAULT_HEADER << 12) | (ISPT_DEFAULT_LENGTH << 10);
	ssp_write_word(&ispt_ssp, data);
	
	return 0;
}
EXPORT_SYMBOL(ispt_driver_msg);

int ispt_wakeup_src_msg(u32 wakesrc)
{
	u32 data;
	
	if (wakesrc > CT_P_WAKEUP_MAX)
		return -ENOENT;

	if (!(ispt_ssp_inited && ispt_switch))
		return 0;
	/* data is a 16-bit half word */
	data = (ISPT_DEFAULT_HEADER << 12) | (ISPT_DEFAULT_LENGTH << 10)
		| (wakesrc & ISPT_DEFAULT_MASK);
	ssp_write_word(&ispt_ssp, data);

	return 0;
}
EXPORT_SYMBOL(ispt_wakeup_src_msg);

int ispt_flush(void)
{
	if (!(ispt_ssp_inited && ispt_switch))
		return 0;
	ssp_flush(&ispt_ssp);
	
	return 0;
}
EXPORT_SYMBOL(ispt_flush);

/****************************************************************************
 * F i l e   S y s t e m   I / O   o p e r a t i o n s
 ****************************************************************************
 */

static int pxa_ispt_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int pxa_ispt_close(struct inode *inode, struct file *file)
{
	return 0;
}

static int pxa_ispt_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, void *arg)
{
	int ret = 0, count;
	unsigned long addr;

	switch (cmd) {
	case ISPT_START:
		ispt_switch = 1;
		break;
	case ISPT_STOP:
		ispt_switch = 0;
		break;
	case ISPT_GET_DRV_NUM:
		count = dvfm_query_device_num();
		if (copy_to_user((int *)arg, &count, sizeof(count)))
			ret = -EFAULT;
		break;
	case ISPT_GET_DRV_LIST:
		addr = get_zeroed_page(GFP_KERNEL);
		if (addr <= 0)
			return -ENOMEM;
		ret = dvfm_query_device_list((void *)addr, PAGE_SIZE);
		if (ret) {
			free_page(addr);
			ret = -EFAULT;
			break;
		}
		if (copy_to_user(arg, (void *)addr, PAGE_SIZE))
			ret = -EFAULT;
		free_page(addr);
		break;
	default:
		break;
	}
	return ret;

}

static struct file_operations pxa_ispt_fops = {
	.owner		= THIS_MODULE,
	.open		= pxa_ispt_open,
	.release	= pxa_ispt_close,
	.ioctl		= pxa_ispt_ioctl,
};

static struct miscdevice pxa_ispt_miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "ispt",
	.fops		= &pxa_ispt_fops,
};

static int ispt_probe(struct platform_device *pdev)
{
	u32 mode, flags, speed;
	int ret;

	/* ISPT use SSP4 */
	ret = ssp_init(&ispt_ssp, 4, SSP_NO_IRQ);
	ssp_disable(&ispt_ssp);

	mode = 0;
	mode &= ~SSCR0_FRF;	/* SPI format */
	/* clean FPCKE bit, FIFO packing mode disabled */
	mode &= ~(1 << 29);
	mode |= SSCR0_DSS | SSCR0_EDSS;	/* default datasize: 32 bit */

	flags = 0;
	/* The inactive or idle state of SSPSCLK is low */
	flags &= ~SSCR1_SPO;
	/* SSPCLK is inactive until one cycle after the start of frame
	 * and active until 1/2 cycle before end of frame.
	 */
	flags &= ~SSCR1_SPH;

	/* Tavor/Monahans as SSP clock master */
	flags &= ~SSCR1_SCLKDIR;
	flags &= ~SSCR1_SFRMDIR;

	speed = 0;	/* SSCR0[SCR] = 0 */
	ssp_config(&ispt_ssp, mode, flags, 0, speed);
	ssp_enable(&ispt_ssp);
	ispt_ssp_inited = 1;
	
	return 0;
}

static struct platform_driver ispt_driver = {
	.probe		= ispt_probe,
	.driver		= {
		.name	= "pxa-ispt",
	},
};

static int __init ispt_init(void)
{
	int ret;

	ret = platform_driver_register(&ispt_driver);
	if (ret)
		goto out;
	ret = misc_register(&pxa_ispt_miscdev);
	if (ret) {
		printk(KERN_INFO "Can't register ISPT device.\n");
		ret = -EBUSY;
	}
	if (sysfs_create_group(power_kobj, &attr_group))
		return -1;
out:
	return ret;
}

static void __exit ispt_exit(void)
{
	platform_driver_unregister(&ispt_driver);
}
late_initcall(ispt_init);
module_exit(ispt_exit);
MODULE_LICENSE("GPL");



