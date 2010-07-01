/*
 * drivers/char/micco_hsdetect.c
 *
 * The headset detect driver based on micco
 *
 * Copyright (2008) Marvell International Ltd.
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
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uio.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mman.h>
#include <asm/uaccess.h>
#include <mach/pxa-regs.h>
#include <mach/bitfield.h>
#include <mach/micco.h>
#include <mach/pxa3xx_pmic.h>
#include <mach/micco_hsdetect.h>


struct device *hsdetect_dev;
struct HS_IOCTL hs_detect;

static int  hsdetect_release(struct inode *inode, struct file *file);
static int  hsdetect_open(struct inode *inode, struct file *file);


void hook_detect_enable(void)
{
	u8 val;
	micco_read(MICCO_MISC, &val);
	val |= (MICCO_MISC_REMCON_AUTO | MICCO_MISC_REMCON_FILTER);
	micco_write(MICCO_MISC, val);

	micco_read(MICCO_AUDIO_REG_BASE + MICCO_REM_IN_POLLING_TIME, &val);
	val = PERIOD_1000_MS;
	micco_write(MICCO_AUDIO_REG_BASE + MICCO_REM_IN_POLLING_TIME, val);
}

/****************************************************************************
 * Interrupt Handler
 ***************************************************************************/
void micco_hsdetect_interrupt(unsigned long event)
{
	u8 val;

	if ((event & PMIC_EVENT_HSDETECT)) 
	{
		/* Headset detection */
		micco_read(MICCO_STATUS_B, &val);
		if (val & MICCO_STATUS_B_HEADSET) 
		{
			kobject_uevent(&hsdetect_dev->kobj, KOBJ_ADD);
			hs_detect.hsdetect_status = HEADSET_ADD;
			micco_read(MICCO_AUDIO_REG_BASE + MICCO_REM_IN_POLLING_TIME, &val);
			val = PERIOD_250_MS;
			micco_write(MICCO_AUDIO_REG_BASE + MICCO_REM_IN_POLLING_TIME, val);
		} else {
			kobject_uevent(&hsdetect_dev->kobj, KOBJ_REMOVE);
			hs_detect.hsdetect_status = HEADSET_REMOVE;
			micco_read(MICCO_AUDIO_REG_BASE + MICCO_REM_IN_POLLING_TIME, &val);
			val = PERIOD_1000_MS;
			micco_write(MICCO_AUDIO_REG_BASE + MICCO_REM_IN_POLLING_TIME, val);
		}
	} else if ((event & PMIC_EVENT_HOOKSWITCH)) 
	{
		/* Hook switch */
		micco_read(MICCO_STATUS_B, &val);
		if (val & MICCO_STATUS_B_HOOKSWITCH) 
		{
			kobject_uevent(&hsdetect_dev->kobj, KOBJ_ONLINE);
			hs_detect.hookswitch_status = HOOKSWITCH_PRESSED;
		} else {
			kobject_uevent(&hsdetect_dev->kobj, KOBJ_OFFLINE);
			hs_detect.hookswitch_status = HOOKSWITCH_RELEASED;
		}
	} 
}

/****************************************************************************
 * File System I/O operations
 ***************************************************************************/

static int hsdetect_open(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t hsdetect_read(struct file *file, char __user *buf, size_t count,
			    loff_t *ppos)
{
	return 0;
}

/* We assume that the user buf will not larger than kbuf size */
static ssize_t hsdetect_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	return count;
}

static int hsdetect_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int hsdetect_ioctl(struct inode *inode, struct file *file,unsigned int cmd, unsigned long arg)
{
	struct HS_IOCTL hs_ioctl;
	if (copy_from_user(&hs_ioctl, (void *)arg, sizeof(struct HS_IOCTL) ))
		return -EFAULT;
	
	switch(cmd)
	{
		case HSDETECT_STATUS:
			hs_ioctl.hsdetect_status = hs_detect.hsdetect_status;
			hs_ioctl.hookswitch_status = hs_detect.hookswitch_status;
			break;

		case HOOKSWITCH_STATUS:
			hs_ioctl.hookswitch_status = hs_detect.hookswitch_status;
			hs_ioctl.hsdetect_status = hs_detect.hsdetect_status;
			break;

		default:
			 return -ENOTTY;
	
	}
	return copy_to_user((void *)arg,&hs_ioctl, sizeof(struct HS_IOCTL));
}

static struct file_operations hsdetect_fops = {
	.owner		= THIS_MODULE,
	.open		= hsdetect_open,
	.release	= hsdetect_release,
	.ioctl		= hsdetect_ioctl,
	.write		= hsdetect_write,
	.read		= hsdetect_read,
};

static struct miscdevice hsdetect_miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "micco_hsdetect",
	.fops		= &hsdetect_fops,
};

/****************************************************************************
 * Initialization / Registeration / Removal
 ***************************************************************************/
static int hsdetect_probe(struct platform_device *pdev)
{
	u8 val;
	int ret;

	ret = misc_register(&hsdetect_miscdev);
	if (ret < 0)
		return ret;

	/* Charger should handle EVENT_CHARGER and VBUS */
	ret = pmic_callback_register((PMIC_EVENT_HSDETECT | PMIC_EVENT_HOOKSWITCH),
			micco_hsdetect_interrupt);
	if (ret)  {
		misc_deregister(&hsdetect_miscdev);
		return ret;
	}

	hsdetect_dev = &pdev->dev;

	hs_detect.hsdetect_status =0;
	hs_detect.hookswitch_status =0;

	pr_info("MICCO hsdetect initialized!\n");
	/* Enable interrupt:
	 * Enable Hookswitch
	 * Enable Headset
	 */
	micco_read(MICCO_IRQ_MASK_D, &val);
	val &= ~0x18;
	micco_write(MICCO_IRQ_MASK_D, val);

	hook_detect_enable();

	return 0;
}

static int hsdetect_remove(struct platform_device *pdev)
{
	u8 val;


	/* Mask IRQ */
	micco_read(MICCO_IRQ_MASK_A, &val);
	val |= 0x18;
	micco_write(MICCO_IRQ_MASK_A, val);

	pmic_callback_unregister(PMIC_EVENT_HSDETECT, micco_hsdetect_interrupt);
	misc_deregister(&hsdetect_miscdev);
	return 0;
}

static struct platform_driver hsdetect_driver = {
	.driver = {
		.name	= "micco-hsdetect",
	},
	.probe		= hsdetect_probe,
	.remove		= hsdetect_remove,
};

static int __devinit hsdetect_init(void)
{
	return platform_driver_register(&hsdetect_driver);
}

static void __exit hsdetect_exit(void)
{
	platform_driver_unregister(&hsdetect_driver);
}

module_init(hsdetect_init);
module_exit(hsdetect_exit);
MODULE_LICENSE("GPL");

