/*
 *	Real Time Clock interface for Linux on Monahans.
 *
 *	Copyright (c) 2000 Nils Faerber
 *
 *	Based on rtc.c by Paul Gortmaker
 *	Date/time conversion routines taken from arch/arm/kernel/time.c
 *			by Linus Torvalds and Russel King
 *		and the GNU C Library
 *	( ... I love the GPL ... just take what you need! ;)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 *
 *	1.00	2001-06-08	Nicolas Pitre <nico@cam.org>
 *	- added periodic timer capability using OSMR1
 *	- flag compatibility with other RTC chips
 *	- permission checks for ioctls
 *	- major cleanup, partial rewrite
 *
 *	0.03	2001-03-07	CIH <cih@coventive.com>
 *	- Modify the bug setups RTC clock.
 *
 *	0.02	2001-02-27	Nils Faerber <nils@@kernelconcepts.de>
 *	- removed mktime(), added alarm irq clear
 *
 *	0.01	2000-10-01	Nils Faerber <nils@@kernelconcepts.de>
 *	- initial release

 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <mach/pxa-regs.h>
#include <mach/pxa3xx-regs.h>
#include <asm/mach/time.h>
#include <mach/hardware.h>
#include <mach/dvfm.h>
#include <mach/pxa3xx_rtc.h>

//#define PXA_RTC_DBG
#if defined(PXA_RTC_DBG)
#define pxa_rtc_dbg(fmt, arg...) printk("PxaRtc,%s(line %d): " fmt, 	\
		__FUNCTION__, __LINE__, ##arg)
#else
#define pxa_rtc_dbg(fmt, arg...) do {} while (0)
#endif

#define TIMER_FREQ		3250000
/*
 * Calculate the next alarm time given the requested alarm time mask
 * and the current time.
 */
static void rtc_next_alarm_time(struct rtc_time *next, struct rtc_time *now, struct rtc_time *alrm)
{
	unsigned long next_time;
	unsigned long now_time;

	next->tm_year = now->tm_year;
	next->tm_mon = now->tm_mon;
	next->tm_mday = now->tm_mday;
	next->tm_hour = alrm->tm_hour;
	next->tm_min = alrm->tm_min;
	next->tm_sec = alrm->tm_sec;

	rtc_tm_to_time(now, &now_time);
	rtc_tm_to_time(next, &next_time);

	if (next_time < now_time) {
		/* Advance one day */
		next_time += 60 * 60 * 24;
		rtc_time_to_tm(next_time, next);
	}
}


static DEFINE_SPINLOCK(pxa_rtc_lock);

static unsigned long rtc_freq = 1000;
static unsigned long rtc_epoch = 1900;
#ifdef CONFIG_PXA3xx_DVFM
static int dvfm_dev_id;
#endif

static irqreturn_t pxa_rtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = to_platform_device(dev_id);
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	unsigned int rtsr;
	unsigned long num = 0, events = RTC_IRQF;

	spin_lock(&pxa_rtc_lock);

	rtsr = RTSR;

	/* clear interrupt sources */
	RTSR = 0;
	RTSR = (RTSR_AL | RTSR_HZ | RTSR_RDAL1 | RTSR_RDAL2 | RTSR_SWAL1 |
		RTSR_SWAL2 | RTSR_PIAL);

	/* clear alarm interrupt if it has occurred */
#ifdef	CONFIG_PXA_RTC_WRISTWATCH
	if (rtsr & (RTSR_RDAL1)) {
		rtsr &= ~(RTSR_RDALE1 | RTSR_RDALE2);
		num++;
		events |= RTC_AF;
	}
#else
	if (rtsr & RTSR_AL) {
		rtsr &= ~RTSR_ALE;
		num++;
		events |= RTC_AF;
	}
#endif
	if (rtsr & RTSR_HZ) {
		/* rtsr &= ~RTSR_HZE; */
		num++;
		events |= RTC_UF;
	}
	if (rtsr & RTSR_SWAL1) {
		rtsr &= ~RTSR_SWALE1;
		num++;
		events |= RTC_SWF;
	}
	if (rtsr & RTSR_SWAL2) {
		rtsr &= ~RTSR_SWALE2;
		num++;
		events |= RTC_SWF;
	}
	if (rtsr & RTSR_PIAL) {
		if (PIAR == 0)
			rtsr &= ~RTSR_PIALE;
		num++;
		events |= RTC_PF;
	}
	RTSR = rtsr & (RTSR_ALE | RTSR_HZE | RTSR_RDALE1 |
			RTSR_SWALE1 | RTSR_SWALE2 | RTSR_PIALE | RTSR_PICE |
			RTSR_SWCE);

	/* update irq data & counter */
	rtc_update_irq(rtc, num, events);

	spin_unlock(&pxa_rtc_lock);

	return IRQ_HANDLED;
}

static void wait_2_cycles(void)
{
	/* wait for 2 cycles of 32.768KHz */
	udelay(64);
}

#ifdef	CONFIG_PXA_RTC_WRISTWATCH
static void tm_to_wwtime(struct rtc_time *tm, unsigned int *dreg,
			unsigned int *yreg)
{
	unsigned int tmp;

	tmp = (tm->tm_mday << RTC_WW_MDAY_SHIFT) & RTC_WW_MDAY_MASK;
	tmp |= ((tm->tm_mon + 1) << RTC_WW_MON_SHIFT) & RTC_WW_MON_MASK;
	tmp |= ((tm->tm_year + 1900) << RTC_WW_YEAR_SHIFT) & RTC_WW_YEAR_MASK;
	*yreg = tmp;

	tmp = (tm->tm_sec << RTC_WW_SEC_SHIFT) & RTC_WW_SEC_MASK;
	tmp |= (tm->tm_min << RTC_WW_MIN_SHIFT) & RTC_WW_MIN_MASK;
	tmp |= (tm->tm_hour << RTC_WW_HOUR_SHIFT) & RTC_WW_HOUR_MASK;
	tmp |= (tm->tm_wday << RTC_WW_WDAY_SHIFT) & RTC_WW_WDAY_MASK;
	*dreg = tmp;
}

static int wwtime_to_tm(unsigned int dreg, unsigned int yreg,
			struct rtc_time *tm)
{
	tm->tm_sec = (dreg & RTC_WW_SEC_MASK) >> RTC_WW_SEC_SHIFT;
	tm->tm_min = (dreg & RTC_WW_MIN_MASK) >> RTC_WW_MIN_SHIFT;
	tm->tm_hour = (dreg & RTC_WW_HOUR_MASK) >> RTC_WW_HOUR_SHIFT;
	tm->tm_wday = (dreg & RTC_WW_WDAY_MASK) >> RTC_WW_WDAY_SHIFT;
	tm->tm_mday = (yreg & RTC_WW_MDAY_MASK) >> RTC_WW_MDAY_SHIFT;
	tm->tm_mon = (yreg & RTC_WW_MON_MASK) >> RTC_WW_MON_SHIFT;
	tm->tm_year = (yreg & RTC_WW_YEAR_MASK) >> RTC_WW_YEAR_SHIFT;

	if (tm->tm_year < 1900)
		return -EINVAL;
	tm->tm_year -= 1900;
	tm->tm_mon -= 1;

	return 0;
}
#endif

static unsigned int tm_to_swtime(struct sw_time *tm)
{
	unsigned int swreg;

	swreg = ((tm->tm_hour << RTC_SW_HOUR_SHIFT) & RTC_SW_HOUR_MASK) |
		((tm->tm_min << RTC_SW_MIN_SHIFT) & RTC_SW_MIN_MASK) |
		((tm->tm_sec << RTC_SW_SEC_SHIFT) & RTC_SW_SEC_MASK) |
		((tm->tm_hundreth << RTC_SW_HUNDR_SHIFT) & RTC_SW_HUNDR_MASK);
	return swreg;
}

static void swtime_to_tm(unsigned int swreg, struct sw_time *tm)
{
	tm->tm_min = (swreg & RTC_SW_MIN_MASK) >> RTC_SW_MIN_SHIFT;
	tm->tm_hour = (swreg & RTC_SW_HOUR_MASK) >> RTC_SW_HOUR_SHIFT;
	tm->tm_sec = (swreg & RTC_SW_SEC_MASK) >> RTC_SW_SEC_SHIFT;
	tm->tm_hundreth = (swreg & RTC_SW_HUNDR_MASK) >> RTC_SW_HUNDR_SHIFT;
}

static int pxa_sw_get(struct sw_time *tm, unsigned long cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	int ret;
	switch (cmd) {
	case RTC_RD_SWCNT:
		swtime_to_tm(SWCR, tm);
		break;
	case RTC_RD_SWAR1:
		swtime_to_tm(SWAR1, tm);
		break;
	case RTC_RD_SWAR2:
		swtime_to_tm(SWAR2, tm);
		break;
	}
	ret = copy_to_user(uarg, tm, sizeof(struct sw_time));
	if (ret)
		ret = -EFAULT;
	return ret;
}

static int pxa_sw_set(struct sw_time *tm, unsigned long cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	unsigned int tmp;
	int ret;

	ret = copy_from_user(tm, uarg, sizeof(struct sw_time));
	if (ret == 0) {
		switch (cmd) {
		case RTC_SET_SWCNT:
			SWCR = tm_to_swtime(tm);
			tmp = SWCR;
			break;
		case RTC_SET_SWAR1:
			SWAR1 = tm_to_swtime(tm);
			tmp = SWAR1;
			break;
		case RTC_SET_SWAR2:
			SWAR2 = tm_to_swtime(tm);
			tmp = SWAR2;
			break;
		}
		wait_2_cycles();
	} else
		ret = -EFAULT;
	return ret;
}

#ifdef	CONFIG_PXA_RTC_WRISTWATCH
static int pxa_rtc_gettime(struct device *dev, struct rtc_time *tm)
{
	pxa_rtc_dbg("\n");
	wwtime_to_tm(RDCR, RYCR, tm);
	return 0;
}

static int pxa_rtc_getalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time *p = NULL;
	pxa_rtc_dbg("\n");
	unsigned int	reg;
	p = &(alrm->time);
	wwtime_to_tm(RDAR1, RYAR1, p);
	reg = RTSR;
	alrm->enabled = (reg & RTSR_RDALE1) ? 1 : 0;
	alrm->pending = (reg & RTSR_RDAL1) ? 1 : 0;
	return 0;
}

static int pxa_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	struct rtc_time *p = NULL;
	struct rtc_time tmp;
	pxa_rtc_dbg("%d-%d-%d %d-%d-%d\n", tm->tm_year, tm->tm_mon, tm->tm_mday, 
			tm->tm_hour, tm->tm_min, tm->tm_sec);

	if ((tm->tm_year > (4095 - 1900)) || (tm->tm_year < 70))
		return -EINVAL;
	memcpy(&tmp, tm, sizeof(struct rtc_time));
	p = &tmp;
	tm_to_wwtime(p, (unsigned int *)&RDCR, (unsigned int *)&RYCR);
	wait_2_cycles();
	return 0;
}

static int pxa_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time next, now;
	pxa_rtc_dbg("%d-%d-%d %d-%d-%d\n", alrm->time.tm_year, alrm->time.tm_mon, alrm->time.tm_mday, 
		alrm->time.tm_hour, alrm->time.tm_min, alrm->time.tm_sec);

	if ((alrm->time.tm_year > (4095 - 1900))
		|| ((alrm->time.tm_year < 70) && (alrm->time.tm_year != -1)))
		return -EINVAL;

	pxa_rtc_gettime(dev, &now);
	rtc_next_alarm_time(&next, &now, &alrm->time);
	tm_to_wwtime(&next, (unsigned int *)&RDAR1, (unsigned int *)&RYAR1);
	spin_lock_irq(&pxa_rtc_lock);
	if (alrm->enabled)
		RTSR |= RTSR_RDALE1;
	else
		RTSR &= ~RTSR_RDALE1;
	spin_unlock_irq(&pxa_rtc_lock);
	wait_2_cycles();
	return 0;
}
#else
static int pxa_rtc_gettime(struct device *dev, struct rtc_time *tm)
{
	pxa_rtc_dbg("\n");
	rtc_time_to_tm(RCNR, tm);
	return 0;
}

static int pxa_rtc_getalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	unsigned int	reg;
	pxa_rtc_dbg("\n");
	rtc_time_to_tm(RTAR, &(alrm->time));
	reg = RTSR;
	alrm->enabled = (reg & RTSR_ALE) ? 1 : 0;
	alrm->pending = (reg & RTSR_AL) ? 1 : 0;
	return 0;
}

static int pxa_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	unsigned long time;
	int ret;
	pxa_rtc_dbg("%d-%d-%d %d-%d-%d\n", tm->tm_year, tm->tm_mon, tm->tm_mday, 
			tm->tm_hour, tm->tm_min, tm->tm_sec);

	/*
	 * max_days = 0xFFFFFFFF / 86400 = 49710
	 * max_years = 49710 / 365 = 136
	 */
	if ((tm->tm_year > 135 + 70) || (tm->tm_year < 70))
		return -EINVAL;
	ret = rtc_tm_to_time(tm, &time);
	if (ret == 0) {
		RCNR = time;
		wait_2_cycles();
	}
	return ret;
}

static int pxa_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	unsigned long reg;
	struct rtc_time next, now;
	int ret;
	pxa_rtc_dbg("%d-%d-%d %d-%d-%d\n", alrm->time.tm_year, alrm->time.tm_mon, alrm->time.tm_mday, 
		alrm->time.tm_hour, alrm->time.tm_min, alrm->time.tm_sec);

	/*
	 * max_days = 0xFFFFFFFF / 86400 = 49710
	 * max_years = 49710 / 365 = 136
	 */
	if (alrm->enabled) { /* check enabled alarm */
		if ((alrm->time.tm_year > 135 + 70)
			|| ((alrm->time.tm_year < 70) && (alrm->time.tm_year != -1))) {
			printk(KERN_ERR"pxa_rtc_setalarm err tm_year %d\n", alrm->time.tm_year);
			return -EINVAL;
		}
	}
	pxa_rtc_gettime(dev, &now);
	rtc_next_alarm_time(&next, &now, &alrm->time);
	ret = rtc_tm_to_time(&next, &reg);
	if (ret == 0) {
		spin_lock_irq(&pxa_rtc_lock);
		RTAR = reg;
		if (alrm->enabled)
			RTSR |= RTSR_ALE;
		else
			RTSR &= ~RTSR_ALE;
		spin_unlock_irq(&pxa_rtc_lock);
		wait_2_cycles();
	}
	return 0;
}
#endif

int sync_pxa3xx_rtc(struct rtc_time* tm)
{	
	return pxa_rtc_settime(NULL, tm);
}

static int pxa_rtc_open(struct device *dev)
{
	int ret;
	pxa_rtc_dbg("\n");

	/* find the IRQs */
	ret = request_irq(IRQ_RTC1Hz, pxa_rtc_interrupt,
			IRQF_DISABLED, "RTC 1Hz", dev);
	if (ret) {
		printk(KERN_ERR "RTC:IRQ %d already in use.\n", IRQ_RTC1Hz);
		goto IRQ_RTC1Hz_failed;
	}
	ret = request_irq(IRQ_RTCAlrm, pxa_rtc_interrupt,
			IRQF_DISABLED, "RTC Alrm", dev);
	if (ret) {
		printk(KERN_ERR "RTC:IRQ %d already in use.\n", IRQ_RTCAlrm);
		goto IRQ_RTCAlrm_failed;
	}
	return 0;

IRQ_RTCAlrm_failed:
	free_irq(IRQ_RTC1Hz, NULL);
IRQ_RTC1Hz_failed:
	return -EBUSY;
}

static void pxa_rtc_release(struct device *dev)
{
	spin_lock_irq(&pxa_rtc_lock);
	RTSR = 0;
	spin_unlock_irq(&pxa_rtc_lock);

	free_irq(IRQ_RTCAlrm, dev);
	free_irq(IRQ_RTC1Hz, dev);
}

extern int rtc_calib(void);
static int pxa_rtc_ioctl(struct device *dev, unsigned int cmd,
			unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	struct sw_time	sw_tm;
	unsigned int tmp;
	unsigned long reg;
	int ret = 0;

	switch (cmd) {
	case RTC_IRQP_READ:
		ret = copy_to_user(uarg, &rtc_freq, sizeof(unsigned long));
		break;
	case RTC_IRQP_SET:
		ret = copy_from_user(&reg, uarg, sizeof(unsigned long));
		if (ret == 0) {
			if (reg < 1 || reg > 1000) {
				ret = -EINVAL;
				break;
			}
			if ((reg > 64) && (!capable(CAP_SYS_RESOURCE))) {
				ret = -EACCES;
				break;
			}
		}
		rtc_freq = reg;
		break;
	case RTC_PIE_OFF:
		spin_lock_irq(&pxa_rtc_lock);
		RTSR &= ~(RTSR_PICE | RTSR_PIALE);
		spin_unlock_irq(&pxa_rtc_lock);
		break;
	case RTC_PIE_ON:
		if (rtc_freq == 0) {
			printk(KERN_WARNING "Periodic Frequency can't"
				" be 0Hz\n");
			ret = -EINVAL;
			break;
		}
		if (rtc_freq > 1000) {
			printk(KERN_WARNING "Periodic Frequency can't"
				" beyond 1KHz\n");
			ret = -EINVAL;
			break;
		}
		PIAR = 1000 / rtc_freq;
		wait_2_cycles();
		spin_lock_irq(&pxa_rtc_lock);
		RTSR |= (RTSR_PICE | RTSR_PIALE);
		spin_unlock_irq(&pxa_rtc_lock);
		break;
	case RTC_RD_SWCNT:
	case RTC_RD_SWAR1:
	case RTC_RD_SWAR2:
		ret = pxa_sw_get(&sw_tm, cmd, arg);
		break;
	case RTC_SET_SWCNT:
	case RTC_SET_SWAR1:
	case RTC_SET_SWAR2:
		ret = pxa_sw_set(&sw_tm, cmd, arg);
		break;

#ifdef	CONFIG_PXA_RTC_WRISTWATCH
	case RTC_AIE_OFF:
		spin_lock_irq(&pxa_rtc_lock);
		RTSR &= ~RTSR_RDALE1;
		spin_unlock_irq(&pxa_rtc_lock);
		break;
	case RTC_AIE_ON:
		spin_lock_irq(&pxa_rtc_lock);
		RTSR |= RTSR_RDALE1;
		spin_unlock_irq(&pxa_rtc_lock);
		break;
#else
	case RTC_AIE_OFF:
		spin_lock_irq(&pxa_rtc_lock);
		RTSR &= ~RTSR_ALE;
		spin_unlock_irq(&pxa_rtc_lock);
		break;
	case RTC_AIE_ON:
		spin_lock_irq(&pxa_rtc_lock);
		RTSR |= RTSR_ALE;
		spin_unlock_irq(&pxa_rtc_lock);
		break;
#endif
	case RTC_UIE_OFF:
		spin_lock_irq(&pxa_rtc_lock);
		RTSR &= ~RTSR_HZE;
		spin_unlock_irq(&pxa_rtc_lock);
		break;
	case RTC_UIE_ON:
		spin_lock_irq(&pxa_rtc_lock);
		RTSR |= RTSR_HZE;
		spin_unlock_irq(&pxa_rtc_lock);
		break;
	case RTC_SWAIE1_OFF:
		spin_lock_irq(&pxa_rtc_lock);
		RTSR &= ~(RTSR_SWCE | RTSR_SWALE1);
		spin_unlock_irq(&pxa_rtc_lock);
		break;
	case RTC_SWAIE1_ON:
		spin_lock_irq(&pxa_rtc_lock);
		RTSR |= (RTSR_SWCE | RTSR_SWALE1);
		/* wait hundreths field of SWCR to be synchronized with
		 * second field
		 */
		do {
			tmp = SWCR & RTC_SW_HUNDR_MASK;
		} while (tmp);
		spin_unlock_irq(&pxa_rtc_lock);
		break;
	case RTC_SWAIE2_OFF:
		spin_lock_irq(&pxa_rtc_lock);
		RTSR &= ~(RTSR_SWCE | RTSR_SWALE2);
		spin_unlock_irq(&pxa_rtc_lock);
		break;
	case RTC_SWAIE2_ON:
		spin_lock_irq(&pxa_rtc_lock);
		RTSR |= (RTSR_SWCE | RTSR_SWALE2);
		do {
			tmp = SWCR & RTC_SW_HUNDR_MASK;
		} while (tmp);
		spin_unlock_irq(&pxa_rtc_lock);
		break;
	case RTC_SW_PAUSE:
		spin_lock_irq(&pxa_rtc_lock);
		RTSR &= ~RTSR_SWCE;
		spin_unlock_irq(&pxa_rtc_lock);
		break;
	case RTC_SW_RESUME:
		spin_lock_irq(&pxa_rtc_lock);
		RTSR |= RTSR_SWCE;
		spin_unlock_irq(&pxa_rtc_lock);
		break;
	case RTC_EPOCH_SET:
		/*
		 * There were no RTC clocks before 1900.
		 */
		ret = copy_from_user(&reg, uarg, sizeof(unsigned long));
		if (ret == 0) {
			if ((reg < 1900) || (reg > 4000)) {
				ret = -EINVAL;
				break;
			}
			if (!capable(CAP_SYS_TIME)) {
				ret = -EACCES;
				break;
			}
			rtc_epoch = reg;
			break;
		}
	case RTC_EPOCH_READ:
		ret = copy_to_user(uarg, &rtc_epoch, sizeof(unsigned long));
		break;
	case RTC_CALIB:
#ifdef CONFIG_PXA3xx_DVFM
		dvfm_disable_op_name("D1", dvfm_dev_id);
		dvfm_disable_op_name("D2", dvfm_dev_id);
		if (cpu_is_pxa935())
			dvfm_disable_op_name("CG", dvfm_dev_id);
		ret = rtc_calib();
		dvfm_enable_op_name("D2", dvfm_dev_id);
		dvfm_enable_op_name("D1", dvfm_dev_id);
		if (cpu_is_pxa935())
			dvfm_enable_op_name("CG", dvfm_dev_id);

#else
		ret = rtc_calib();
#endif
		break;
	default:
		ret = -ENOIOCTLCMD;
	}
	if (ret)
		return ret;
	else
		return 0;
}



/* print out resources */
static int pxa_rtc_read_proc(struct device *dev, struct seq_file *seq)
{
	struct sw_time	tm, *p = NULL;

	p = &tm;
	memset(p, 0, sizeof(struct sw_time));
	swtime_to_tm(SWCR, p);
	seq_printf(seq, "StopWatch time\t: %02d:%02d:%02d:%02d\n",
		p->tm_hour, p->tm_min, p->tm_sec, p->tm_hundreth);
	memset(p, 0, sizeof(struct sw_time));
	swtime_to_tm(SWAR1, p);
	seq_printf(seq, "StopWatch alarm1\t: %02d:%02d:%02d:%02d\n",
		p->tm_hour, p->tm_min, p->tm_sec, p->tm_hundreth);
	memset(p, 0, sizeof(struct sw_time));
	swtime_to_tm(SWAR2, p);
	seq_printf(seq, "StopWatch alarm2\t: %02d:%02d:%02d:%02d\n",
		p->tm_hour, p->tm_min, p->tm_sec, p->tm_hundreth);

	return 0;
}


static const struct rtc_class_ops pxa_rtc_ops = {
	.open		= pxa_rtc_open,
	.release	= pxa_rtc_release,
	.ioctl		= pxa_rtc_ioctl,
	.read_time	= pxa_rtc_gettime,
	.set_time	= pxa_rtc_settime,
	.read_alarm	= pxa_rtc_getalarm,
	.set_alarm	= pxa_rtc_setalarm,
	.proc		= pxa_rtc_read_proc,
};

extern int yuhua_printk_rtc_time_enable(int enable);
static int pxa_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;

	/* initialization */
	RTSR = 0;
	PSBR = 1;

	rtc = rtc_device_register(pdev->name, &pdev->dev, &pxa_rtc_ops,
				THIS_MODULE);

	if (IS_ERR(rtc))
		return PTR_ERR(rtc);
	platform_set_drvdata(pdev, rtc);

	yuhua_printk_rtc_time_enable(1);

	return 0;
}

static int pxa_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);

	if (rtc)
		rtc_device_unregister(rtc);

	return 0;
}

#ifdef CONFIG_PM

static struct timespec pxa_rtc_delta;

static int pxa_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rtc_time tm;
	struct timespec time;

	memset(&time, 0, sizeof(struct timespec));

	pxa_rtc_gettime(&(pdev->dev), &tm);
	rtc_tm_to_time(&tm, &time.tv_sec);
	save_time_delta(&pxa_rtc_delta, &time);

	return 0;
}

static int pxa_rtc_resume(struct platform_device *pdev)
{
	struct rtc_time tm;
	struct timespec time;

	memset(&time, 0, sizeof(struct timespec));

	pxa_rtc_gettime(&(pdev->dev), &tm);
	rtc_tm_to_time(&tm, &time.tv_sec);
	restore_time_delta(&pxa_rtc_delta, &time);

	return 0;
}
#else
#define pxa_rtc_suspend	NULL
#define pxa_rtc_resume	NULL
#endif

static struct platform_driver pxa_rtc_drv = {
	.driver = {
		.name	= "pxa3xx-rtc",
	},
	.probe		= pxa_rtc_probe,
	.remove		= pxa_rtc_remove,
	/* let android alarm to control sys time */
	//.suspend	= pxa_rtc_suspend,
	//.resume		= pxa_rtc_resume,
};

static int __init pxa_rtc_init(void)
{
	int ret;
	ret = platform_driver_register(&pxa_rtc_drv);
	if (ret)
		printk(KERN_ERR "rtc: register error.\n");
#ifdef CONFIG_PXA3xx_DVFM
	ret = dvfm_register("RTC", &dvfm_dev_id);
#endif
	pr_info("PXA Real Time Clock driver v" DRIVER_VERSION "\n");
	return 0;
}

static void __exit pxa_rtc_exit(void)
{
	platform_driver_unregister(&pxa_rtc_drv);
#ifdef CONFIG_PXA3xx_DVFM
	dvfm_unregister("RTC", &dvfm_dev_id);
#endif
}


module_init(pxa_rtc_init);
module_exit(pxa_rtc_exit);

MODULE_AUTHOR("Nils Faerber <nils@@kernelconcepts.de>");
MODULE_DESCRIPTION("PXA3xx Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");

