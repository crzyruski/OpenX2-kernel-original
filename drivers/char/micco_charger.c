/*
 * drivers/char/micco_charger.c
 *
 * The charger driver based on micco
 *
 * Copyright (2006) Marvell International Ltd.
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

static int charger_major = 253;
static int bat_aval;	/* Default no battary available*/

static int  charger_release(struct inode *inode, struct file *file);
static int  charger_open(struct inode *inode, struct file *file);

static void stop_charging(void)
{
	u8 val;

	micco_read(MICCO_CHARGE_CONTROL, &val);
	val &= ~0x80;
	micco_write(MICCO_CHARGE_CONTROL, val);
	return;
}

static void start_charging(void)
{
	micco_write(MICCO_CHARGE_CONTROL, 0xAA);
	return;
}

/****************************************************************************
 * Interrupt Handler
 ***************************************************************************/
/* Currently we don't enable the timeout for charging. So Just Event A is
 * enough.
 */
void micco_charger_interrupt(u8 event)
{
	u8 val;

	if (event & (PMIC_EVENT_TBAT | PMIC_EVENT_REV_IOVER |
			PMIC_EVENT_IOVER)) {
		/* if Battery over temperature, reverse mode over current or
		 * Charger over current, turn off the charging because the
		 * over temperature.
		 */
		stop_charging();
	} else if (event & PMIC_EVENT_CHDET) {
		/* Charge detection */
		/* TODO: Notify user space appliction: charger is plugged */
		micco_read(MICCO_STATUS_A, &val);
		if (val & MICCO_STATUS_A_CHDET) {
			pr_debug("%s: charger detection,start charging\n",
				__func__);
			if (bat_aval)
				start_charging();
		} else {
			pr_debug("%s: charger removed!\n", __func__);
		}
	} else if (event & PMIC_EVENT_VBATMON) {
		/* Charge voltage too low */
		/* TODO: Notify user space appliction: Need charging */
	} else if (event & PMIC_EVENT_VBUS) {
		/* USB cable detected */
		micco_read(MICCO_STATUS_B, &val);
		if (val & MICCO_STATUS_B_USBDEV) {
			pr_debug("%s: charger detected, start charging\n",
				__func__);
			if (bat_aval)
				start_charging();
		} else {
			pr_debug("%s: charger removed!\n", __func__);
		}
	}
}

/****************************************************************************
 * File System I/O operations
 ***************************************************************************/

static int charger_open(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t charger_read(struct file *file, char __user *buf, size_t count,
			    loff_t *ppos)
{
	return 0;
}

/* We assume that the user buf will not larger than kbuf size */
static ssize_t charger_write(struct file *file, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	return count;
}

static int charger_release(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations charger_fops = {
	.owner		= THIS_MODULE,
	.open		= charger_open,
	.release	= charger_release,
	.write		= charger_write,
	.read		= charger_read,
};

static struct miscdevice charger_miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "micco_charger",
	.fops		= &charger_fops,
};

/****************************************************************************
 * Initialization / Registeration / Removal
 ***************************************************************************/
static int charger_probe(struct platform_device *pdev)
{
	u8 val;
	int ret;

	ret = misc_register(&charger_miscdev);
	if (ret < 0)
		return ret;

	/* Charger should handle EVENT_CHARGER and VBUS */
	ret = pmic_callback_register((PMIC_EVENT_CHARGER | PMIC_EVENT_VBUS),
			micco_charger_interrupt);
	if (ret)  {
		misc_deregister(&charger_miscdev);
		return ret;
	}

	/* pxa3xx_pmic_set_pump(1);*/
	pr_info("MICCO charger initialized!\n");
	/* Enable interrupt:
	 * the over current on charger reverse,
	 * the VBAT less then VBATMON,
	 * the Battery over temperature,
	 * the Charger detection/removal.
	 */
	micco_read(MICCO_IRQ_MASK_A, &val);
	val &= ~0x78;
	micco_write(MICCO_IRQ_MASK_A, val);

	/* TODO:
	 * Need set the registers value:
	 * TBATHIGHP, TBATHIGHN, TBATLOW.
	 * ICHMAX_RES, ICHMIN_RES, VCHMAX_RES, VCHMIN_RES
	 */

	/* Enable the charging if there is a battery */
	micco_read(MICCO_MISC, &val);
	val |= MICCO_MISC_I_TBAT_ON;
	micco_write(MICCO_MISC, val);

	while (1) {
		/* reset ADC */
		micco_write(MICCO_ADC_MAN_CONTROL, 0x00);
		msleep(2);

		/* Enable ADC and setup input mux */
		micco_write(MICCO_ADC_MAN_CONTROL, 0x13);
		msleep(2);

		/* Enable manual conversion on TBAT voltage */
		micco_write(MICCO_ADC_MAN_CONTROL, 0x1b);
		msleep(2);

		micco_read(MICCO_ADC_MAN_CONTROL, &val);
		if (!(val & MICCO_ADC_MAN_CONT_CONV))
			break;
	}

	micco_read(MICCO_MAN_RES_MSB, &val);
	if (0xff != val) {
		/* Bettery available */
		/* TODO: Need adjust the value according to
		 * the charger type (wall power or USB host).
		 * Currently, just support USB host.
		 */
		bat_aval = 1;
		start_charging();
	}

	micco_read(MICCO_MISC, &val);
	val &= ~MICCO_MISC_I_TBAT_ON;
	micco_write(MICCO_MISC, val);

	/* Enable the TBAT, VCH, ICH and VBAT auto measurements */
	micco_read(MICCO_ADC_MAN_CONTROL, &val);
	val |= 0x10;
	micco_write(MICCO_ADC_MAN_CONTROL, val);

	micco_read(MICCO_ADC_AUTO_CONTROL_1, &val);
	val |= 0x2E;
	micco_write(MICCO_ADC_AUTO_CONTROL_1, val);

	return 0;
}

static int charger_remove(struct platform_device *pdev)
{
	/* Disable TBAT, VCH, ICH and VBAT auto measurements */
	u8 val;

	micco_read(MICCO_ADC_MAN_CONTROL, &val);
	val &= ~0x10;
	micco_write(MICCO_ADC_MAN_CONTROL, val);

	micco_read(MICCO_ADC_AUTO_CONTROL_1, &val);
	val &= ~0x2E;
	micco_write(MICCO_ADC_AUTO_CONTROL_1, val);

	/* Mask IRQ */
	micco_read(MICCO_IRQ_MASK_A, &val);
	val |= 0x78;
	micco_write(MICCO_IRQ_MASK_A, val);

	pxa3xx_pmic_set_pump(0);
	pmic_callback_unregister(PMIC_EVENT_CHARGER, micco_charger_interrupt);
	misc_deregister(&charger_miscdev);
	return 0;
}

static struct platform_driver charger_driver = {
	.driver = {
		.name	= "micco-charger",
	},
	.probe		= charger_probe,
	.remove		= charger_remove,
};

static int __devinit charger_init(void)
{
	return platform_driver_register(&charger_driver);
}

static void __exit charger_exit(void)
{
	platform_driver_unregister(&charger_driver);
}

module_init(charger_init);
module_exit(charger_exit);
MODULE_LICENSE("GPL");

