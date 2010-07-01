/*
 * Copyright (C) 2003-2004 Intel Corporation.
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __IPMC_H
#define __IPMC_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#endif

#include <mach/pxa3xx_dvfm.h>
#include <mach/pxa3xx_pm.h>

struct mspm_app_info;

/* Use 'K' as magic number */
#define IPMC_IOC_MAGIC  'K'
#define IPMC_IOCTL_SET_DEEPIDLE		_IOW(IPMC_IOC_MAGIC, 2, int)
#define IPMC_IOCTL_GET_FV_OP		_IOR(IPMC_IOC_MAGIC, 3, int)
#define IPMC_IOCTL_SET_FV_OP		_IOW(IPMC_IOC_MAGIC, 4, struct ipmc_ioctl_setfvop)
#define IPMC_IOCTL_GET_FV_OP_COUNT	_IOR(IPMC_IOC_MAGIC, 5, int)
#define IPMC_IOCTL_GET_FV_OP_INFO	_IOR(IPMC_IOC_MAGIC, 6, struct ipmc_ioctl_getfvopinfo)
#define IPMC_IOCTL_GET_SLEEPTIME	_IOR(IPMC_IOC_MAGIC, 8, unsigned int)
#define IPMC_IOCTL_SET_SLEEPTIME	_IOW(IPMC_IOC_MAGIC, 9, unsigned int)
#define IPMC_IOCTL_GET_WAKEUPSRC	_IOR(IPMC_IOC_MAGIC, 21, pm_wakeup_src_t)
#define IPMC_IOCTL_SET_WAKEUPSRC	_IOW(IPMC_IOC_MAGIC, 22, pm_wakeup_src_t)
#define IPMC_IOCTL_CLEAR_EVENTLIST	_IOW(IPMC_IOC_MAGIC, 24, int)
#define IPMC_IOCTL_SET_WINDOWSIZE	_IOW(IPMC_IOC_MAGIC, 30, int)
#define IPMC_IOCTL_REGISTER		_IOW(IPMC_IOC_MAGIC, 31, int)
#define IPMC_IOCTL_UNREGISTER		_IOW(IPMC_IOC_MAGIC, 32, int)
#define IPMC_IOCTL_SET_POLICY		_IOWR(IPMC_IOC_MAGIC, 33, int)
#define IPMC_IOCTL_QUERY_NUM		_IOR(IPMC_IOC_MAGIC, 34, int)
#define IPMC_IOCTL_QUERY_INFO		_IOWR(IPMC_IOC_MAGIC, 35, struct mspm_app_info)

/* 20 IPM event max */
#define MAX_IPME_NUM			20
#define INFO_SIZE			128

/*
 * Although the array is 256, but some data is occupied by device drivers.
 * When DVFM finds client number is full, MSPM can't register any more.
 * MSPM_MAX_CLIENT should be consistent with DVFM_MAX_CLIENT.
 */
#define MSPM_MAX_CLIENT			256
#define MSPM_APP_NAME_LEN		256

struct ipm_event {
	int type;			/* What type of IPM events. */
	int kind;			/* What kind, or sub-type of events. */
	unsigned char info[INFO_SIZE];	/* events specific data. */
};

struct ipmc_ioctl_setfvop {
	unsigned int op;
	unsigned int mode;
};

struct ipmc_ioctl_getfvopinfo {
	unsigned int op;
	struct pxa3xx_fv_info info;
};

enum {
	MSPM_POLICY_FREE = 0,		/* no constraint */
	MSPM_POLICY_NOIDLE_REPLACE,	/* constraint on deep idle */
	MSPM_POLICY_PERF,		/* constraint on op below 208MHz */
	MSPM_POLICY_HIGHPERF,		/* restrict in 624MHz */
	MSPM_POLICY_ERROR,		/* incorrect policy */
};

struct mspm_app_info {
	char	name[MSPM_APP_NAME_LEN];
	int	pid;
	int	mspm_cid;
	int	mspm_policy;
};

/* IPM event types. */
#define IPM_EVENT_PMU			0x1	/* PMU events, may not need. */
#define IPM_EVENT_UI			0x2	/* Device event. */
#define IPM_EVENT_POWER			0x3	/* Power fail/revocer event. */
#define IPM_EVENT_UITIMER		0x4	/* Device Timer timeout event. */
#define IPM_EVENT_CPU			0x5	/* CPU utilization change event. */
#define IPM_EVENT_DEVICE		0x6
#define IPM_EVENT_PROFILER		0x7	/* Profiler events. */
//#define IPM_EVENT_SYSTEM_WAKEUP	0x8
#define IPM_EVENT_SUSPEND_WAKEUP	0x9
#define IPM_EVENT_STANDBY_WAKEUP	0xa

/* IPM event kinds. */
#define IPM_EVENT_POWER_LOW		0x1
#define IPM_EVENT_POWER_FAULT		0x2
#define IPM_EVENT_POWER_OK		0x3
#define IPM_EVENT_IDLE_PROFILER		0x1
#define IPM_EVENT_PERF_PROFILER		0x2
#define IPM_EVENT_DEVICE_KEYPAD		0x0
#define IPM_EVENT_DEVICE_TSI		0x1
#define IPM_EVENT_DEVICE_TIMEOUT	0x2
#define IPM_EVENT_DEVICE_BUSY		0x3
#define IPM_EVENT_DEVICE_OUTD0CS	0x10
#define IPM_EVENT_DEVICE_OUT104		0x20
#define IPM_EVENT_DEVICE_OUT208		0x30
#define IPM_EVENT_DEVICE_OUT416		0x40
#define IPM_EVENT_WAKEUPTIMEOUT		0x0	/* Wakeup Timeout (invalid) */
#define IPM_EVENT_WAKEUPBYRTC		0x1
#define IPM_EVENT_WAKEUPBYUI		0x2	/* User Interface: Touch Screen, Keypad */

#define IPM_WAKEUPTIMER			0x1
#define IPM_UITIMER			0x2

/* IPM event infos, not defined yet. */
#define IPM_EVENT_NULLINFO		0x0

#ifdef __KERNEL__
/* IPM events queue */
struct ipme_queue{
        int head;
        int tail;
        int len;
        struct ipm_event  ipmes[MAX_IPME_NUM];
        wait_queue_head_t waitq;
};

extern int enable_deepidle;

/* IPM functions */
extern int ipm_event_notify(int type, int kind, void *info, unsigned int info_len);
#endif

#endif

