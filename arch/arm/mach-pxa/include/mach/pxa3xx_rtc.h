/*
 * include/asm-arm/arch-pxa/pxa3xx_rtc.h
 *
 * Copyright (C) 2006, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __PXA3XX_RTC_H

#include <linux/rtc.h>


#define	DRIVER_VERSION		"1.04"


/* Those are the bits from a classic RTC we want to mimic */
#define RTC_IRQF		0x80	/* any of the following 3 is active */
#define RTC_PF			0x40
#define RTC_AF			0x20
#define RTC_UF			0x10
#define RTC_SWF			0x08



#define RTC_SWAIE1_ON	_IO('r', 0x11)
#define RTC_SWAIE1_OFF	_IO('r', 0x12)
#define RTC_SWAIE2_ON	_IO('r', 0x13)
#define RTC_SWAIE2_OFF	_IO('r', 0x14)
#define RTC_PIE_PAUSE	_IO('r', 0x15)
#define RTC_PIE_RESUME	_IO('r', 0x16)
#define RTC_SW_PAUSE	_IO('r', 0x17)
#define RTC_SW_RESUME	_IO('r', 0x18)

#define RTC_RD_SWCNT	_IOR('r', 0x26, unsigned long)
#define RTC_SET_SWCNT	_IOW('r', 0x27, unsigned long)
#define RTC_RD_SWAR1	_IOR('r', 0x28, unsigned long)
#define RTC_SET_SWAR1	_IOW('r', 0x29, unsigned long)
#define RTC_RD_SWAR2	_IOR('r', 0x2a, unsigned long)
#define RTC_SET_SWAR2	_IOW('r', 0x2b, unsigned long)

#define RTC_WW_MDAY_SHIFT	0
#define RTC_WW_MDAY_MASK	(0x1FU << RTC_WW_MDAY_SHIFT)
#define RTC_WW_MON_SHIFT	5
#define RTC_WW_MON_MASK		(0x0FU << RTC_WW_MON_SHIFT)
#define RTC_WW_YEAR_SHIFT	9
#define RTC_WW_YEAR_MASK	(0xFFFU << RTC_WW_YEAR_SHIFT)
#define RTC_WW_SEC_SHIFT	0
#define RTC_WW_SEC_MASK		(0x3FU << RTC_WW_SEC_SHIFT)
#define RTC_WW_MIN_SHIFT	6
#define RTC_WW_MIN_MASK		(0x3FU << RTC_WW_MIN_SHIFT)
#define RTC_WW_HOUR_SHIFT	12
#define RTC_WW_HOUR_MASK	(0x1FU << RTC_WW_HOUR_SHIFT)
#define RTC_WW_WDAY_SHIFT	17
#define RTC_WW_WDAY_MASK	(0x07U << RTC_WW_WDAY_SHIFT)
#define RTC_WW_MWEEK_SHIFT	20
#define RTC_WW_MWEEK_MASK	(0x07U << RTC_WW_MWEEK_SHIFT)


#define RTC_SW_HUNDR_SHIFT	0
#define RTC_SW_HUNDR_MASK	(0x7FU << RTC_SW_HUNDR_SHIFT)
#define RTC_SW_SEC_SHIFT	7
#define RTC_SW_SEC_MASK		(0x3FU << RTC_SW_SEC_SHIFT)
#define RTC_SW_MIN_SHIFT	13
#define RTC_SW_MIN_MASK		(0x3FU << RTC_SW_MIN_SHIFT)
#define RTC_SW_HOUR_SHIFT	19
#define RTC_SW_HOUR_MASK	(0x1FU << RTC_SW_HOUR_SHIFT)


struct sw_time {
	unsigned int	tm_hour;
	unsigned int	tm_min;
	unsigned int	tm_sec;
	unsigned int	tm_hundreth;
};
#endif


