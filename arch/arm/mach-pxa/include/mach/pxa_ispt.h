/*
 *  linux/arch/arm/mach-pxa/include/mach/pxa_ispt.h
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.

 *  (C) Copyright 2008 Marvell International Ltd.
 *  All Rights Reserved
 */

#ifndef	__MACH_PXA_ISPT_H__
#define	__MACH_PXA_ISPT_H__

#include <linux/ioctl.h>

enum {
	CT_P_RESERVED = 0x0,
	CT_P_SW_TASK,
	CT_P_SW_INTERRUPT,
	CT_P_SW_FUNC_STATE,
	CT_P_NEXT_BLOCK,
	CT_P_VAR_TRACE,
	CT_P_FREE_FORMAT,
	CT_P_CONTROL,
	CT_P_POWER_MESSAGE,
	CT_P_SW_FUNC_STOP,
	CT_P_UNKNOWN_MESSAGE = 0xF,
};

enum {
	/* Power State entry & exit messages */
	/* indicates C0 core state entry */
	CT_P_PWR_STATE_ENTRY_C0 = 0x0,
	/* indicates C1 core state entry */
	CT_P_PWR_STATE_ENTRY_C1 = 0x1,
	/* indicates D2 subsystem state entry, without shutting down VCTCXO */
	CT_P_PWR_STATE_ENTRY_D2_WO_VCTCXO_SD = 0x2,
	/* indicates D2 subsystem state entry */
	CT_P_PWR_STATE_ENTRY_D2 = 0x3,
	/* indicates that system was ready for D2, but only stayed in C1 */
	CT_P_PWR_MODE_EXIT_D2 = 0x04,
	/* Reserve 0x05~0x0F */


	/* Product Point in use messages */
	/* indicates APPS SS in D0CS mode */
	CT_P_PP_D0CS_IN_USE = 0x10,
	/* indicates system is in PP1 after DVFM was done */
	CT_P_PP_1_IN_USE = 0x11,
	/* indicates system is in PP2 after DVFM was done */
	CT_P_PP_2_IN_USE = 0x12,
	/* indicates system is in PP3 after DVFM was done */
	CT_P_PP_3_IN_USE = 0x13,
	/* indicates system is in PP4 after DVFM was done */
	CT_P_PP_4_IN_USE = 0x14,
	/* indicates APPS SS left D0CS mode and returned to previous PP */
	CT_P_PP_D0CS_NOT_IN_USE = 0x15,
	

	/* COMM (AC-IPC) messaging */
	/* indicates COMM requested DDR */
	CT_P_DDR_REQUEST = 0x20,
	/* Received COMM DDR REQ and sent ACK */
	CT_P_DDR_ACK = 0x21,
	/* indicates COMM relinguished DDR */
	CT_P_DDR_RELINQUISH = 0x22,
	/* indicates COMM requested HF DDR */
	CT_P_DDR_260_REQUEST = 0x23,
	/* indicates APPS sent HF DDR Ack */
	CT_P_DDR_260_ACK =0x24,
	/* indicates COMM relinguished HF DDR */
	CT_P_DDR_260_RELINQUISH = 0x25,
	/* indicates APPS received COMM HF DDR REL and sent Ack */
	CT_P_DDR_260_RELINQUISH_ACK = 0x26,


	/* Idle profiler messages */
	/*
	 * indicates that idle profiler measured x% CPU load
	 * last 7 bits are CPU %(0-100)
	 */
	CT_P_CPU_LOAD_IND = 0x80,


	/* D2 wakeup messages */
	/*
	 * indicates wakeup (Exit D2) source is xxx
	 * last 6 bits are wakeup source data
	 */
	CT_P_IS_D2_WAKEUP_SOURCE = 0x140,
	/*
	 * indicates when driver asks to enable wakeup port xxx
	 * last 6 bits are wakeup source data
	 */
	CT_P_D2_WAKEUP_ENABLE = 0x180,
	/*
	 * indicates when driver asks to disable wakeup port xxx
	 * last 6 bits are wakeup source data
	 */
	CT_P_D2_WAKEUP_DISABLE = 0x1C0,


	/* clock control messages */
	/*
	 * indicates when driver xxx asks to enable clock
	 * last 6 bits are clock data
	 */
	CT_P_CLOCK_ENABLE = 0x240,
	/*
	 * indicates when driver xxx asks to disable clock
	 * last 6 bits are clock data
	 */
	CT_P_CLOCK_DISABLE = 0x280,


	/* Driver PM control */
	/*
	 * indicates when driver xxx asks for DVFM blocking
	 * last 6 bits is driver data
	 */
	CT_P_DVFM_BLOCK_REQ = 0x300,
	/*
	 * indicates when driver xxx releases DVFM blocking
	 * last 6 bits is driver data
	 */
	CT_P_DVFM_BLOCK_REL = 0x340,
	/*
	 * indicates when driver xxx allows AP SS to enter D2
	 * last 6 bits is driver data
	 */
	CT_P_D2_ALLOW = 0x380,
	/*
	 * indicates when driver xxx doesn't allow AP SS to enter D2
	 * last 6 bits is driver data
	 */
	CT_P_D2_NOT_ALLOW = 0x3C0,

	// Extended messages that are not sent with PM header
	CT_P_SET_MIN_OP	= 0xA400, /* last 9 bits are for OP id (3 bit) and driver Id (6 bit) */
	CT_P_DEBUG_MSG_SND = 0xA600, /* last 8 bits are free Message data for debug */
};

enum {
	CT_P_WAKEUP_CWESR_UART = 0x0,
	CT_P_WAKEUP_CWESR_CGPIO,
	CT_P_WAKEUP_CWESR_WBSM,
	CT_P_WAKEUP_CWESR_GSSP1,
	CT_P_WAKEUP_CWESR_ACIPC,
	CT_P_WAKEUP_CWESR_GSSP2,
	CT_P_WAKEUP_CWESR_USIM = 0x7,
	CT_P_WAKEUP_CWESR_I2C = 0xA,
	CT_P_WAKEUP_CWESR_PMIC = 0xD,
	CT_P_WAKEUP_CWESR_TIMER = 0x11,
	CT_P_WAKEUP_CWESR_WDT,
	CT_P_WAKEUP_CWESR_GSM_SLK,
	CT_P_WAKEUP_EXT_WAKEUP0 = 0x14,
	CT_P_WAKEUP_EXT_WAKEUP1,
	CT_P_WAKEUP_GENERIC0 = 0x16,
	CT_P_WAKEUP_GENERIC1,
	CT_P_WAKEUP_GENERIC2,
	CT_P_WAKEUP_GENERIC3,
	CT_P_WAKEUP_GENERIC4,
	CT_P_WAKEUP_GENERIC5,
	CT_P_WAKEUP_GENERIC6,
	CT_P_WAKEUP_GENERIC7,
	CT_P_WAKEUP_GENERIC8,
	CT_P_WAKEUP_GENERIC9,
	CT_P_WAKEUP_GENERIC10,
	CT_P_WAKEUP_GENERIC11,
	CT_P_WAKEUP_GENERIC12,
	CT_P_WAKEUP_GENERIC13,
	CT_P_WAKEUP_USBOTG,
	CT_P_WAKEUP_USIM = 0x27,
	CT_P_WAKEUP_BSSP3,
	CT_P_WAKEUP_KEYPAD,
	CT_P_WAKEUP_USBOTG2,
	CT_P_WAKEUP_USBOTG3,
	/* don't distinguish enhanced rotary key and ACIPC */
	CT_P_WAKEUP_BOTH_ROTARY_ACIPC,
	CT_P_WAKEUP_USBC2 = 0x2E,
	CT_P_WAKEUP_OSTIMER = 0x32,
	CT_P_WAKEUP_RTC,
	CT_P_WAKEUP_ROTARY_KEY = 0x34,
	CT_P_WAKEUP_ACIPC,
	CT_P_WAKEUP_MAX = 0x3F,
};

/* HEAD = power message, short message */
#define	POWER_MSG_HEADER		(0x8400)
#define	POWER_MSG_MASK			(0xFF)
#define ISPT_DEFAULT_HEADER		(0x8)
#define ISPT_DEFAULT_LENGTH		(0x1)
#define ISPT_DEFAULT_MASK		(0x3FF)
#define ISPT_DRIVER_MASK		(0x3F)
#define ISPT_WAKEUP_MASK		(0x3F)

#define ISPT_IOC_MAGIC			'T'
#define ISPT_START			_IOW(ISPT_IOC_MAGIC, 0, int)
#define ISPT_STOP			_IOW(ISPT_IOC_MAGIC, 1, int)
#define ISPT_GET_DRV_NUM		_IOR(ISPT_IOC_MAGIC, 2, int *)
#define ISPT_GET_DRV_LIST		_IOR(ISPT_IOC_MAGIC, 3, void *)

/* ISPT Functions */
extern int ispt_power_msg(u32 msg);
extern int ispt_dvfm_msg(u32 old, u32 new);
extern int ispt_driver_msg(u32 msg, u32 dev_id);
extern int ispt_wakeup_src_msg(u32 wakeupsrc);
extern int ispt_flush(void);

#endif

