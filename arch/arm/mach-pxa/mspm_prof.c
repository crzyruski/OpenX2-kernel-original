/*
 * PXA3xx IPM Profiler
 *
 * Copyright (C) 2008 Marvell Corporation
 * Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2008 Marvell International Ltd.
 * All Rights Reserved
 */

/*
 * Behavior of profiler
 *
 * When profiler is enabled, the frequency can be adjusted by two reason.
 * 1. When sample window is finished, profiler calculates the mips in sample
 * window. System will be adjusted to suitable OP.
 *
 * 2. When sample window isn't finished and system enters low power mode
 * (D2/CG), sample window is reset. At this time, system may stays in high OP
 * or D0CS OP. As complementation, profiler start a workqueue and request
 * system stay in operating point higher than D0CS. Because entering low power
 * mode from D0CS is unacceptable and entering low power mode from high OP
 * costs more power.
 *    System entering low power can be consider as very low mips cost.
 *    If profiler is disabled, the workqueue must be flushed. Then system
 * will stay in highest OP (624MHz).
 *    If system is busy, profiler will calculate new mips in sample window.
 * And system will switch to suitable OP from this OP (just high than D0CS).
 *
 * So when profiler is disabled, system always stays in 624MHz. System can
 * enters D2 idle and D0CS idle, but the frequency won't be changed.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>

#include <mach/hardware.h>
#include <mach/pxa-regs.h>
#include <mach/mspm_prof.h>
#include <mach/ipmc.h>
#include <mach/prm.h>
#ifdef CONFIG_PXA3xx_DVFM
#include <mach/dvfm.h>
#include <mach/pxa3xx_dvfm.h>
#endif

/* idle watchdog timer */
#define IDLE_WATCHDOG_MSECS		40

#ifdef CONFIG_BOARD_LANDMARK
//#define CONFIG_PXA3xx_DVFM_PERFORMANCE 0x1
#endif

struct mspm_op_stats {
	int		op;
	int		idle;
	unsigned int	timestamp;
	unsigned int 	jiffies;
};

struct mspm_mips {
	int	mips;
	int	mem;
	int	xsi;
	int	h_thres;	/* high threshold */
	int	l_thres;	/* low threshold */
};

enum {
	MSPM_PRM_FREE = 0,
	MSPM_PRM_ALLOCATED,
	MSPM_PRM_COMMITED,
};

enum {
	DISABLE = 0,
	ENABLE,
};

extern struct kobject *power_kobj;

extern int mspm_idle_load(void);
extern void mspm_idle_clean(void);

/* records the PRM resource state of MSPM */
static int mspm_prm_state = MSPM_PRM_FREE;
static int mspm_ctrl_state = DISABLE, mspm_prof_state = DISABLE;
#if 1 /* open mspm in init script */
static int mspm_ctrl = DISABLE, mspm_prof = DISABLE;
#else
static int mspm_ctrl = ENABLE, mspm_prof = DISABLE;
#endif

/*
 * Store OP's MIPS in op_mips[].
 * The lowest frequency OP is the first entry.
 */
static struct mspm_mips op_mips[MAX_OP_NUM];

/* Store duration time of all OP in op_duration[]. */
static int op_duration[MAX_OP_NUM];

/* Store costed time in run_op_time[] & idle_op_time[] */
static int run_op_time[MAX_OP_NUM], idle_op_time[MAX_OP_NUM];

/*
 * Store the first timestamp of sample window in first_stats
 * Store the current timestamp of sample window in cur_stats
 */
static struct mspm_op_stats first_stats, cur_stats;

/* OP numbers used in IPM IDLE Profiler */
static int mspm_op_num = 0;

static struct timer_list idle_prof_timer, idle_watchdog;

/* PMU result is stored in it */
static struct pmu_results sum_pmu_res;

static int window_jif = 0, watchdog_jif = 0;
static int restore_op = 0x5a;	/* magic number */

static int mspm_prof_notifier_freq(struct notifier_block *nb,
				unsigned long val, void *data);
static struct notifier_block notifier_freq_block = {
	.notifier_call = mspm_prof_notifier_freq,
};

/* This workqueue is used to down frequency if system is entering D2 */
static struct work_struct down_freq;

static int mspm_mem_thres = 40;
static int mspm_window = DEF_SAMPLE_WINDOW;
/* Stores the flag of idle watchdog */
static int mspm_watchdog = 0;
/* Stores the miliseconds count of idle watchdog */
static int mspm_wtimer = 0;
static int dvfm_dev_idx;

/* PRM related staff */
#define MSPM_PMU_RES_NUM	6
static prm_resource_id resource[MSPM_PMU_RES_NUM] = {PRM_CCNT, PRM_PMN0,
				PRM_PMN1, PRM_PMN2, PRM_PMN3, PRM_COP};

unsigned int mspm_prm_id;		/* Client ID */

static unsigned int read_time(void)
{
#ifdef CONFIG_PXA_32KTIMER
	return OSCR4;
#else
	return OSCR0;
#endif
}

static unsigned int ext_mem_cnt = 0;
int mspm_calc_pmu(struct ipm_profiler_arg *arg)
{
	ext_mem_cnt = arg->pmn0;
	return 0;
}

static int calc_xsi_mem(unsigned int sum_time, int *xsi, int *mem)
{
	int i;
	if (sum_time == 0)
		return -EINVAL;
	for (i = 0, *xsi = 0, *mem = 0; i < mspm_op_num; i++) {
		*xsi += op_duration[i] * op_mips[i].xsi;
		*mem += op_duration[i] * op_mips[i].mem;
	}
	return 0;
}

static int check_memstall(int mips, unsigned sum_time)
{
	static int max_nonstall_cnt = 3, nonstall_cnt = 0;
	int count = 0, bandwidth = 0, stall = 0, ret = 0;
	int xsi = 0, mem = 0;
	calc_xsi_mem(sum_time, &xsi, &mem);
	/* 16-bit memory */
	/*
	 * At here, there's an assumption only 16-bit memory is configured.
	 * If configure 32-bit memory, multiply 4.
	 * Convert cycles of XSI to cycles of memory controller.
	 * One DDR bus cycle send two DDR data. One DDR bus cycle responses
	 * to two DDR controller cycles.
	 * XSI cycles / XSI freq = DDR controller cycles / DDR controller freq
	 * = DDR bus cycles / DDR bys freq = time
	 * DDR burst mode is 8 byte aligned.
	 */
	xsi = xsi >> 10;
	mem = mem >> 10;
	if (xsi != 0)
		count = ext_mem_cnt * mem / xsi;
	if (sum_time != 0)
		bandwidth = (count * 8 / 11) * (CLOCK_TICK_RATE / sum_time)
			>> 20;
	else
		bandwidth = 0;
	/* If bandwidth is larger than 20MB/s, return true */
	if (bandwidth > mspm_mem_thres)
		stall = 1;
	if (!stall) {
		if (nonstall_cnt < max_nonstall_cnt) {
			nonstall_cnt++;
			ret = 1;
		} else
			ret = 0;
	} else {
		nonstall_cnt = 0;
		ret = 1;
	}
	return ret;
}

/*
 * Adjust to the most appropriate OP according to MIPS result of
 * sample window
 */
int mspm_request_tune(int mips, int memstall, int mem, int cur_op)
{
	int i;
#ifdef CONFIG_PXA3xx_DVFM_PERFORMANCE
	struct op_info *info = NULL;
	int curop = dvfm_get_op(&info);
#endif

	/* If memory stall, disable d0csidle. Otherwise, enable it. */
	if (memstall || cur_op>1) /* if cur_op>104M, disable d0cs, by frank */
		dvfm_disable_op_name("D0CS", dvfm_dev_idx);
	else
		dvfm_enable_op_name_aync("D0CS", dvfm_dev_idx);
	for (i = mspm_op_num - 1; i >= 0; i--) {
		if (mips >= (op_mips[i].l_thres *
			op_mips[i].mips / 100))
			break;
	}
#ifdef CONFIG_PXA3xx_DVFM_PERFORMANCE  /* for best cpu performance */
#define PXA3xx_TOP_SPEED (4) /* 624M */
	if (i >curop) {	
		i = PXA3xx_TOP_SPEED;
	} else if (i < curop) {
		i = curop -1;
	}
#endif
	dvfm_request_op(i);
	return 0;
}

static int idle_wd_valid = 0;
static int check_idle_wd(int mips)
{
	int mips_thres = 200, max_idle_cnt = 3;
	static int idle_cnt = 0;
	/* if mspm_watchdog isn't enabled, don't calculate */
	if (!mspm_watchdog)
		return 0;
	if (mips < mips_thres) {
		if (idle_cnt < max_idle_cnt) {
			idle_cnt++;
			idle_wd_valid = 1;
		} else
			idle_wd_valid = 0;
	} else {
		/* mips is larger */
		idle_wd_valid = 1;
		idle_cnt = 0;
	}
	return 0;
}

static int is_idle_wd_valid(void)
{
	if (mspm_watchdog)
		return idle_wd_valid;
	return 0;
}

/*
 * Calculate the MIPS in sample window
 */
static int mspm_calc_mips(unsigned int first_time)
{
	int i, mips, stall, mem = 0, curop;
	unsigned int time, sum_time = 0, sum =0;
	struct op_info *info = NULL;

	curop = dvfm_get_op(&info);

	/* Store the last slot as RUN state */
	time = read_time();
	run_op_time[cur_stats.op] += (time > cur_stats.timestamp) ?
				(time - cur_stats.timestamp) :
				(OSCR_MASK - cur_stats.timestamp + time);
	cur_stats.timestamp = time;
	cur_stats.jiffies = jiffies;
	cur_stats.op = curop;
	cur_stats.idle = CPU_STATE_RUN;
	/* Calculate total time costed in sample window */
	for (i = 0; i < mspm_op_num; i++) {
		sum_time += run_op_time[i] + idle_op_time[i];
		sum += run_op_time[i] * op_mips[i].mips;
		op_duration[i] = run_op_time[i] + idle_op_time[i];
	}
	if (sum_time == 0) {
		/* CPU usage is 100% in current operating point */
		sum_time = (time > first_time) ? (time - first_time)
			: (OSCR_MASK - first_time + time);
		sum = sum_time * op_mips[curop].mips;
		op_duration[curop] = sum_time;
	}

	/*
	 * Calculate MIPS in sample window
	 * Formula: run_op_time[i] / sum_time * op_mips[i].mips
	 */
	mips = sum / sum_time;
	check_idle_wd(mips);
	stall = check_memstall(mips, sum_time);
	return mspm_request_tune(mips, stall, mem, curop);
}

/*
 * Record the OP index and RUN/IDLE state.
 */
int mspm_add_event(int op, int cpu_idle)
{
	unsigned int time;

	if (mspm_prof_state == ENABLE) {
		if (is_idle_wd_valid())
			mod_timer(&idle_watchdog, jiffies + watchdog_jif);
		time = read_time();
		/* sum the current sample window */
		if (cpu_idle == CPU_STATE_IDLE)
			idle_op_time[cur_stats.op] +=
				(time > cur_stats.timestamp) ?
				(time - cur_stats.timestamp) :
				(OSCR_MASK - cur_stats.timestamp + time);
		else if (cpu_idle == CPU_STATE_RUN)
			run_op_time[cur_stats.op] +=
				(time > cur_stats.timestamp) ?
				(time - cur_stats.timestamp) :
				(OSCR_MASK - cur_stats.timestamp + time);
		/* update start point of current sample window */
		cur_stats.op = op;
		cur_stats.idle = cpu_idle;
		cur_stats.timestamp = time;
		cur_stats.jiffies = jiffies;
	}
	return 0;
}
EXPORT_SYMBOL(mspm_add_event);

/*
 * Prepare to do a new sample.
 * Clear the index in mspm_op_stats table.
 */
static int mspm_do_new_sample(void)
{
	/* clear previous sample window */
	memset(&run_op_time, 0, sizeof(int) * MAX_OP_NUM);
	memset(&idle_op_time, 0, sizeof(int) * MAX_OP_NUM);
	/* prepare for the new sample window */
	first_stats.op = cur_stats.op;
	first_stats.idle = cur_stats.idle;
	first_stats.timestamp = read_time();
	first_stats.jiffies = jiffies;
	return 0;
}

/***************************************************************************
 * 			Idle Profiler
 ***************************************************************************
 */

static struct ipm_profiler_arg pmu_arg;
static int mspm_start_prof(void)
{
	struct pmu_results res;
	struct op_info *info = NULL;

	/* pmu_arg.window_size stores the number of miliseconds.
	 * window_jif stores the number of jiffies.
	 */
	memset(&pmu_arg, 0, sizeof(struct ipm_profiler_arg));
	pmu_arg.flags = IPM_IDLE_PROFILER | IPM_PMU_PROFILER;
	pmu_arg.pmn0 = PXA3xx_EVENT_EXMEM;
	pmu_arg.pmn1 = PMU_EVENT_POWER_SAVING;
	pmu_arg.pmn2 = PMU_EVENT_POWER_SAVING;
	pmu_arg.pmn3 = PMU_EVENT_POWER_SAVING;
	pmu_arg.window_size = mspm_window;
	window_jif = msecs_to_jiffies(pmu_arg.window_size);
	watchdog_jif = msecs_to_jiffies(IDLE_WATCHDOG_MSECS);
	mspm_wtimer = IDLE_WATCHDOG_MSECS;

	if (mspm_prm_state == MSPM_PRM_COMMITED) {
		if (pmu_arg.flags & IPM_PMU_PROFILER) {
			/* Collect PMU information */
			pmu_stop(&res);
			pmu_start(pmu_arg.pmn0, pmu_arg.pmn1, pmu_arg.pmn2,
				pmu_arg.pmn3);
		}
		/* start next sample window */
		cur_stats.op = dvfm_get_op(&info);
		cur_stats.idle = CPU_STATE_RUN;
		cur_stats.timestamp = read_time();
		cur_stats.jiffies = jiffies;
		mspm_do_new_sample();
		mod_timer(&idle_prof_timer, jiffies + window_jif);
		/* start idle watchdog */
		if (is_idle_wd_valid())
			mod_timer(&idle_watchdog, jiffies + watchdog_jif);
		mspm_prof_state = ENABLE;
	}
	return 0;
}

static int mspm_stop_prof(void)
{
	struct pmu_results res;
	/* flush workqueue */
	flush_scheduled_work();
	if (mspm_prm_state == MSPM_PRM_COMMITED) {
		if (pmu_arg.flags & IPM_PMU_PROFILER)
			pmu_stop(&res);
		del_timer(&idle_prof_timer);
		if (is_idle_wd_valid())
			del_timer(&idle_watchdog);
		/* try to use 624MHz if it's not magic number */
		if (restore_op != 0x5a)
			dvfm_request_op(restore_op);
		mspm_prof_state = DISABLE;
	}
	return 0;
}

static int calc_pmu_res(struct pmu_results *res)
{
	if (res == NULL)
		return -EINVAL;
	sum_pmu_res.ccnt += res->ccnt;
	sum_pmu_res.pmn0 += res->pmn0;
	sum_pmu_res.pmn1 += res->pmn1;
	sum_pmu_res.pmn2 += res->pmn2;
	sum_pmu_res.pmn3 += res->pmn3;
	return 0;
}

static void down_freq_worker(void)
{
	/* request operating point higher than D0CS */
	dvfm_request_op(1);
}

/*
 * Handler of IDLE PROFILER
 */
static void idle_prof_handler(unsigned long data)
{
	struct ipm_profiler_result out_res;
	struct pmu_results res;

	if (mspm_prm_state == MSPM_PRM_COMMITED) {
		if (pmu_arg.flags & IPM_PMU_PROFILER) {
			if (!pmu_stop(&res))
				calc_pmu_res(&res);
			pmu_start(pmu_arg.pmn0, pmu_arg.pmn1,
				pmu_arg.pmn2, pmu_arg.pmn3);
			memset(&out_res, 0,
				sizeof(struct ipm_profiler_result));
			out_res.pmu.ccnt = sum_pmu_res.ccnt;
			out_res.pmu.pmn0 = sum_pmu_res.pmn0;
			out_res.pmu.pmn1 = sum_pmu_res.pmn1;
			out_res.pmu.pmn2 = sum_pmu_res.pmn2;
			out_res.pmu.pmn3 = sum_pmu_res.pmn3;
		}
		mspm_calc_pmu((struct ipm_profiler_arg *)&out_res);
		mspm_calc_mips(first_stats.timestamp);
		
		/* start next sample window */
		mspm_do_new_sample();
		mod_timer(&idle_prof_timer, jiffies + window_jif);
		memset(&sum_pmu_res, 0, sizeof(struct pmu_results));
		if (is_idle_wd_valid())
			mod_timer(&idle_watchdog, jiffies + watchdog_jif);
	}
}

static void idle_watchdog_handler(unsigned long data)
{
	struct op_info *info = NULL;
	struct dvfm_md_opt *md_op = NULL;
	int mips;
	if (mspm_prof_state == ENABLE) {
#ifdef CONFIG_PXA3xx_DVFM
		int curop = dvfm_get_op(&info);
		md_op = (struct dvfm_md_opt *)info->op;
		mips = md_op->core;
		/* Now CPU usage is 100% in current operating point */
		mspm_request_tune(mips, 0, 0, curop);
#endif
		//mod_timer(&idle_watchdog, jiffies + watchdog_jif);
	}
}

extern void tick_nohz_update_jiffies(void);
/*
 * Pause idle profiler when system enter Low Power mode.
 * Continue it when system exit from Low Power mode.
 */
static int mspm_prof_notifier_freq(struct notifier_block *nb,
				unsigned long val, void *data)
{
	struct dvfm_freqs *freqs = (struct dvfm_freqs *)data;
	struct op_info *info = &(freqs->new_info);
	struct dvfm_md_opt *md = NULL;
	int cpu;

	if (mspm_prof_state == DISABLE) 
		return 0;
	/* only delete timer and stop tick for idle thread */
	cpu = smp_processor_id();
	if (!idle_cpu(cpu))
		return 0;
	md = (struct dvfm_md_opt *)(info->op);
	if (md->power_mode == POWER_MODE_D1 ||
		md->power_mode == POWER_MODE_D2 ||
		md->power_mode == POWER_MODE_CG) {
		switch (val) {
		case DVFM_FREQ_PRECHANGE:
			del_timer(&idle_prof_timer);
			if (is_idle_wd_valid())
				del_timer(&idle_watchdog);
			tick_nohz_stop_sched_tick(1);
			break;
		case DVFM_FREQ_POSTCHANGE:
#ifdef CONFIG_NO_HZ
			/* Update jiffies and touch watchdog process */
			tick_nohz_update_jiffies();
#endif
#if 0
			/*
			 * If system always stays in 624MHz, check jiffies.
			 * If jiffies reaches sample window end,
			 * request new operating point according to calculation.
			 */
			if (jiffies > (first_stats.jiffies + window_jif))
				mspm_calc_mips(first_stats.timestamp);
#endif
			/*
			 * Restart the idle profiler because it's only
			 * disabled before entering low power mode.
			 * If we just continue the sample window with
			 * left jiffies, too much OS Timer wakeup exist
			 * in system.
			 * Just restart the sample window.
			 */ 
			mod_timer(&idle_prof_timer, jiffies + window_jif);
			if (is_idle_wd_valid())
				mod_timer(&idle_watchdog,
					jiffies + watchdog_jif);
			first_stats.jiffies = jiffies;
			first_stats.timestamp = read_time();
			/* Try to down the frequency to at least 156MHz */
			schedule_work(&down_freq);
			break;
		}
	}
	return 0;
}

/* It's invoked by initialization code & sysfs interface */
static int launch_mspm(void)
{
	int ret = 0;
	if ((mspm_ctrl_state == DISABLE) &&
		(mspm_ctrl == ENABLE)) {
		/* enable mspm control */
		switch (mspm_prm_state) {
		case MSPM_PRM_COMMITED:
			mspm_idle_load();
			/* check whether profiler should be launched.
			 * At here, we check mspm_prof value.
			 */
			if (mspm_prof == ENABLE)
				mspm_start_prof();
			/* disable unused OP in MSPM */
			dvfm_disable_op_name("CUSTOM OP", dvfm_dev_idx);
			dvfm_disable_op_name("BOOT OP", dvfm_dev_idx);
			mspm_ctrl_state = ENABLE;
			break;
		case MSPM_PRM_ALLOCATED:
			mspm_ctrl_state = ENABLE;
			break;
		case MSPM_PRM_FREE:
		default:
			/* error condition */
			printk(KERN_WARNING "wrong prm state in %s\n",
				__FUNCTION__);
			ret = -EINVAL;
			goto out;
		}
	} else if ((mspm_ctrl_state == ENABLE) &&
		(mspm_ctrl == DISABLE)) {
		/* disable mspm control */
		switch (mspm_prm_state) {
		case MSPM_PRM_COMMITED:
			/* check whether profiler is launched.
			 * At here, we check mspm_prof_state.
			 */
			if (mspm_prof_state == ENABLE)
				mspm_stop_prof();
			mspm_idle_clean();
			/* enable unused OP in MSPM */
			dvfm_enable_op_name("CUSTOM OP", dvfm_dev_idx);
			dvfm_enable_op_name("BOOT OP", dvfm_dev_idx);
			mspm_ctrl_state = DISABLE;
			break;
		case MSPM_PRM_ALLOCATED:
			mspm_ctrl_state = DISABLE;
			break;
		case MSPM_PRM_FREE:
		default:
			/* error condition */
			printk(KERN_WARNING "wrong prm state in %s\n",
				__FUNCTION__);
			ret = -EINVAL;
			goto out;
		}
	}
	return 0;
out:
	return ret;
}

static void resource_handler(prm_event event, unsigned int group_id,
				void *data)
{
	switch (event) {
	case PRM_RES_READY:
		/* Resource is available to MSPM */
		pr_debug("resource available to MSPM\n");
		if (prm_commit_resources(mspm_prm_id, group_id)) {
			pr_debug("failed to commit PRM resources again\n");
			mspm_prm_state = MSPM_PRM_ALLOCATED;
			return;
		}
		mspm_prm_state = MSPM_PRM_COMMITED;

		pr_debug("%s, mspm_ctrl_state:%d, mspm_ctrl:%d\n", __func__,
			mspm_ctrl_state, mspm_ctrl);
		if (mspm_ctrl == ENABLE) {
			mspm_idle_load();
			/* mspm_stop_prof sets mspm_prof_state to disable */
			if (mspm_prof == ENABLE)
				mspm_start_prof();
			mspm_ctrl_state = ENABLE;
			/*
			 * When MSPM is running, profiler make descision on
			 * which OP system can stay in.
			 */
		}
		break;
	case PRM_RES_APPROPRIATED:
		/* Resource is unavailable to MSPM */
		pr_debug("resource unavailable to MSPM\n");
		/*
		 * If mspm is enabled, disable dvfm.
		 * Because we need disable deepidle and profiler.
		 * If mspm is disable, system stays in current operating point.
		 */
		pr_debug("%s, mspm_ctrl_state:%d, mspm_ctrl:%d\n", __func__,
			mspm_ctrl_state, mspm_ctrl);
		/* disable unused OP in MSPM */
		dvfm_disable_op_name("CUSTOM OP", dvfm_dev_idx);
		dvfm_disable_op_name("BOOT OP", dvfm_dev_idx);
		if (mspm_ctrl_state == ENABLE) {
			/* Try to raise 624MHz if restore_op is assigned. */
			if (restore_op != 0x5a)
				dvfm_request_op(restore_op);
			if (mspm_prof_state == ENABLE)
				mspm_stop_prof();
			mspm_idle_clean();
			mspm_ctrl_state = DISABLE;
		}
		if (mspm_prm_state == MSPM_PRM_COMMITED)
			mspm_prm_state = MSPM_PRM_ALLOCATED;
		break;
	default:
		break;
	}
}

/************************************************************************
 *	 		sysfs interface					*
 ************************************************************************
 */

#define mspm_attr(_name)				\
static struct kobj_attribute _name##_attr = {		\
	.attr	= {					\
		.name = __stringify(_name),		\
		.mode = 0644,				\
	},						\
	.show	= _name##_show,				\
	.store	= _name##_store,			\
}

/*
 * Show whether MSPM is enabled
 */
static ssize_t mspm_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%u\n", mspm_ctrl);
}

/*
 * Configure MSPM
 * When MSPM is enabled, mspm idle is loaded.
 */
static ssize_t mspm_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int data;
	sscanf(buf, "%u", &data);

	if (data)
		mspm_ctrl = ENABLE;
	else
		mspm_ctrl = DISABLE;
	pr_debug("mspm_ctrl:%d, mspm_ctrl_state:%d\n", mspm_ctrl,
			mspm_ctrl_state);
	pr_debug("mspm_prm_state:%d\n", mspm_prm_state);
	launch_mspm();
	pr_debug("mspm_ctrl:%d, mspm_ctrl_state:%d\n", mspm_ctrl,
			mspm_ctrl_state);
	return len;
}
mspm_attr(mspm);


int mspm_state()
{
	return mspm_ctrl;
}

void mspm_disable(void)
{
	mspm_ctrl = 0;
	launch_mspm();
}

void mspm_enable(void)
{
        mspm_ctrl = 1;
        launch_mspm();
}


EXPORT_SYMBOL(mspm_disable);
EXPORT_SYMBOL(mspm_enable);
EXPORT_SYMBOL(mspm_state);



/*
 * Show whether MSPM profiler in kerenl space is enabled
 */
static ssize_t prof_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%u\n", mspm_prof);
}

/*
 * Configure the MSPM profiler in kernel space
 * This interface can't control deepidle
 */
static ssize_t prof_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int data;
	sscanf(buf, "%u", &data);
	if (data)
		mspm_prof = ENABLE;
	else
		mspm_prof = DISABLE;
	pr_debug("mspm_prof:%d, mspm_prof_state:%d\n", mspm_prof,
			mspm_prof_state);
	pr_debug("mspm_prm_state:%d\n", mspm_prm_state);
	/*
	 * If mspm control is disabled, needn't enable/disable profiler here.
	 * When mspm control is enabled, it will enable/disable profiler.
	 */
	if (mspm_ctrl_state == DISABLE)
		goto out;

	/* enable/disable profiler */
	if ((mspm_prof_state == DISABLE) &&
		(mspm_prof == ENABLE)) {
		/* enable profiler */
		switch (mspm_prm_state) {
		case MSPM_PRM_COMMITED:
			mspm_start_prof();
			break;
		case MSPM_PRM_ALLOCATED:
			mspm_prof_state = ENABLE;
			break;
		case MSPM_PRM_FREE:
		default:
			printk(KERN_WARNING "wrong prm state in %s\n",
				__FUNCTION__);
			break;
		}
	} else if ((mspm_prof_state == ENABLE) &&
		(mspm_prof == DISABLE)) {
		/* disable profiler */
		switch (mspm_prm_state) {
		case MSPM_PRM_COMMITED:
			mspm_stop_prof();
			break;
		case MSPM_PRM_ALLOCATED:
			mspm_prof_state = DISABLE;
			break;
		case MSPM_PRM_FREE:
		default:
			printk(KERN_WARNING "wrong prm state in %s\n",
				__FUNCTION__);
			break;
		}
	}
out:
	pr_debug("mspm_prof:%d, mspm_prof_state:%d\n", mspm_prof,
			mspm_prof_state);
	return len;
}
mspm_attr(prof);

/*
 * stall interface is used to set the memory stall parameter
 */
static ssize_t mem_thres_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%uMB/s\n", mspm_mem_thres);
}

static ssize_t mem_thres_store(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	sscanf(buf, "%u", &mspm_mem_thres);
	return len;
}
mspm_attr(mem_thres);

/* Show the length of sample window */
static ssize_t window_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%ums\n", mspm_window);
}

static ssize_t window_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	sscanf(buf, "%u", &mspm_window);
	return len;
}
mspm_attr(window);

static ssize_t watchdog_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%u\n", mspm_watchdog);
}

static ssize_t watchdog_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	sscanf(buf, "%u", &mspm_watchdog);
	if (mspm_watchdog)
		mod_timer(&idle_watchdog, jiffies + watchdog_jif);
	else
		del_timer(&idle_watchdog);
	return len;
}
mspm_attr(watchdog);

static ssize_t wtimer_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%ums\n", mspm_wtimer);
}

static ssize_t wtimer_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	sscanf(buf, "%u", &mspm_wtimer);
	if (mspm_wtimer > 0)
		watchdog_jif = msecs_to_jiffies(mspm_wtimer);
	return len;
}
mspm_attr(wtimer);

static int dvfm_d0cs_dbg_dev_idx;
static int dvfm_d0cs_dbg_enable;
static ssize_t d0cs_dbg_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", dvfm_d0cs_dbg_enable);
}

static ssize_t d0cs_dbg_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	sscanf(buf, "%d", &dvfm_d0cs_dbg_enable);
	if (dvfm_d0cs_dbg_enable)
		dvfm_disable_op_name("D0CS", dvfm_d0cs_dbg_dev_idx);
	else
		dvfm_enable_op_name_aync("D0CS", dvfm_d0cs_dbg_dev_idx);
	return len;
}
mspm_attr(d0cs_dbg);

static struct attribute * g[] = {
	&mspm_attr.attr,
	&prof_attr.attr,
	&mem_thres_attr.attr,
	&window_attr.attr,
	&watchdog_attr.attr,
	&wtimer_attr.attr,
	&d0cs_dbg_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.name	= "mspm",
	.attrs	= g,
};


/*
 * Init MIPS of all OP
 * Return OP numbers
 */
int __init mspm_init_mips(void)
{
	struct op_info *info = NULL;
	struct dvfm_md_opt *md_op = NULL;
	int i, ret;
	memset(&op_mips, 0, MAX_OP_NUM * sizeof(struct mspm_mips));
#ifdef CONFIG_PXA3xx_DVFM
	mspm_op_num = dvfm_op_count();
	for (i = 0; i < mspm_op_num; i++) {
		ret = dvfm_get_opinfo(i, &info);
		if (ret)
			continue;
		md_op = (struct dvfm_md_opt *)info->op;
		op_mips[i].mips = md_op->core;
		op_mips[i].mem = md_op->dmcfs;
		if (op_mips[i].mips) {
			op_mips[i].h_thres = DEF_HIGH_THRESHOLD;
			if (!strcmp(md_op->name, "D0CS")) {
				op_mips[i].h_thres = 90;
				op_mips[i].xsi = 60;
			} else {
				op_mips[i].xsi = 13 * md_op->xl;
				if (!strcmp(md_op->name, "624M"))
					restore_op = i;
			}
		} else {
			/* Low Power mode won't be considered */
			mspm_op_num = i;
			break;
		}
	}
	for (i = 0; i < mspm_op_num - 1; i++)
		op_mips[i + 1].l_thres = op_mips[i].h_thres * op_mips[i].mips
				/ op_mips[i + 1].mips;
#endif
	return mspm_op_num;
}

static int __init mspm_prm_request(void)
{
	int ret, i;

	/* open a session to access PRM COP resources */
	ret = prm_open_session(PRI_IPMC, "MSPM", resource_handler,
				(void *)NULL);
	if (ret < 0) {
		printk(KERN_ERR "unable to open session of PMU access: %d\n",
				ret);
		return ret;
	}
	mspm_prm_id = ret;

	/* allocate necessary PMU & COP resources */
	for (i = 0; i < MSPM_PMU_RES_NUM; i++) {
		/* all resources belong to group 0 */
		if ((ret = prm_allocate_resource(mspm_prm_id, resource[i],
			0))) {
			pr_debug("mspm request PMU resource failure(%x)\n",
				ret);
			goto out_pmu;
		}
	}
	mspm_prm_state = MSPM_PRM_ALLOCATED;

	/* commit resources */
	ret = prm_commit_resources(mspm_prm_id, 0);
	if (ret) {
		pr_debug("mspm commit resource failure(%x)\n", ret);
		goto out_pmu;
	}
	mspm_prm_state = MSPM_PRM_COMMITED;
	return 0;
out_pmu:
	prm_free_resources(mspm_prm_id, 0);
	prm_close_session(mspm_prm_id);
	mspm_prm_state = MSPM_PRM_FREE;
	return ret;
}

static int dvfm_system_suspend_dev_idx;
static struct timer_list system_suspend_timer;
extern int register_pm_notifier(struct notifier_block *nb);
static void system_suspend_timer_handler(unsigned long data)
{
	dvfm_enable_op_name_aync("D0CS", dvfm_system_suspend_dev_idx);
}
static int system_suspend_notify(struct notifier_block *self, unsigned long action, void *data)
{
	if (action==PM_SUSPEND_PREPARE)
		dvfm_disable_op_name("D0CS", dvfm_system_suspend_dev_idx);
	else if (action==PM_POST_SUSPEND)
		mod_timer(&system_suspend_timer, jiffies+HZ*2);
	
	return 0;
}
static struct notifier_block system_suspend_nb = {
	.notifier_call = system_suspend_notify,
};

static void __exit mspm_prm_release(void)
{
	/* release all resources in group 0 */
	prm_free_resources(mspm_prm_id, 0);
	prm_close_session(mspm_prm_id);
	mspm_prm_state = MSPM_PRM_FREE;
}

int __init mspm_prof_init(void)
{
	if (!cpu_is_pxa3xx())
		return -EFAULT;

	if (sysfs_create_group(power_kobj, &attr_group))
		return -EFAULT;

	mspm_prm_request();

	/* It's used to trigger sample window.
	 * If system is idle, the timer could be deferred.
	 */
	init_timer_deferrable(&idle_prof_timer);
	idle_prof_timer.function = idle_prof_handler;
	idle_prof_timer.data = 0;

	/* It's used to monitor IDLE event */
	init_timer_deferrable(&idle_watchdog);
	idle_watchdog.function = idle_watchdog_handler;
	idle_watchdog.data = 0;

	mspm_op_num = mspm_init_mips();

	dvfm_register_notifier(&notifier_freq_block,
				DVFM_FREQUENCY_NOTIFIER);
	dvfm_register("MSPM PROF", &dvfm_dev_idx);

	dvfm_register("D0CS DEBUG", &dvfm_d0cs_dbg_dev_idx);

	dvfm_register("SYSTEM SUSPEND", &dvfm_system_suspend_dev_idx);
	register_pm_notifier(&system_suspend_nb);
	init_timer(&system_suspend_timer);
	system_suspend_timer.function = system_suspend_timer_handler;

	launch_mspm();
	INIT_WORK(&down_freq, down_freq_worker);
	return 0;
}

void __exit mspm_prof_exit(void)
{
	dvfm_unregister("MSPM PROF", &dvfm_dev_idx);
	dvfm_unregister_notifier(&notifier_freq_block,
				DVFM_FREQUENCY_NOTIFIER);

	mspm_prm_release();
}


