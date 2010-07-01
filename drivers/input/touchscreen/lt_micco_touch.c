/*
 *  linux/drivers/input/touchscreen/littleton.c
 *
 *  touch screen driver for Littleton Platform
 *
 *  Copyright (C) 2006, Marvell Corporation (fengwei.yin@Marvell.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/freezer.h>
#include <linux/proc_fs.h>

#include <asm/irq.h>
#include <asm/uaccess.h>

#include <mach/pxa-regs.h>
#if defined(CONFIG_IPM)
#include <mach/ipmc.h>
#endif
#include <mach/micco.h>
#include <mach/pxa3xx_pmic.h>
#include <mach/pxafb.h>
#ifdef CONFIG_YUHUA_MISC_DEV	
#include <mach/yuhua_board_dev_info.h>
#endif

#define LT_TOUCH_PROC_DEBUG
#ifdef LT_TOUCH_PROC_DEBUG
static int bTouchDBG;
#define	ts_dbg(fmt, arg...)   do{ if(bTouchDBG) printk(fmt,##arg);}while(0)
#define	ts_raw_dbg(fmt, arg...)   do{ if(bTouchDBG>1) printk(KERN_DEBUG fmt,##arg);}while(0)
#else
#define	ts_dbg(fmt, arg...)
#define	ts_raw_dbg(fmt, arg...)
#endif

/* Enable raw-data filter */
#define LT_TOUCH_ENABLE_FILTER	 	(0x1)
#ifdef LT_TOUCH_ENABLE_FILTER
/* Max distance between 2 steps */
#define LT_TOUCH_MAX_STEP			(0x60)
/* Max offset between 2 datas when one sample */
#define LT_TOUCH_SAMPLE_OFFSET	(0x20)
#endif

/* Enable touch-data statistics */
#define LT_TOUCH_ENABLE_STATISTIC	(0x1)

/* Poll data in timer handler context,undef it if you want to use kernel thread */
#define LT_TOUCH_TIMER_POLL	 	(0x1)
/* Default interval when sample data*/
#if 0
#define LT_TOUCH_SAMPLE_INTERVAL_US 	(200) /* 5k HZ */
#else
#define LT_TOUCH_SAMPLE_INTERVAL_US 	(300) /* No interval */
#endif
#define SAMPLE_UDELAY(us) do {if(us > 0) udelay(us);} while(0)
/* Poll sample every LT_TOUCH_POLL_JIFFLES*10 ms */
#ifdef LT_TOUCH_TIMER_POLL
#define LT_TOUCH_POLL_JIFFIES 		(5)
#else
#define LT_TOUCH_POLL_JIFFIES 		(1)
#endif
/* Read TOUCH_SAMPLE_NUM data when one sample */
#define TOUCH_SAMPLE_NUM 			(3)

#define TOUCH_SAMPLE_RETRY 		(3)

enum {
	TSI_PEN_UNKNOW =0 ,
	TSI_PEN_DOWN = 1,
	TSI_PEN_UP = 2,
	TSI_PEN_PRE_DOWN = 3,
};

enum LT_TSI_STAGE {
	TSI_STAGE_DETECT =0 ,
	TSI_STAGE_POLL,
	TSI_STAGE_OFF,
};

struct keyemu_matrix {
	int	active;
	int	x, y, w, h;
};

struct lt_touch_data {
	spinlock_t			ts_lock;	
	struct input_dev 		*littleton_ts_input_dev;
	int					suspended;
	int					pen_state;
	int					use_count;
	int					poll_jiffies;
	int					intervalUs_sample; /* interval when sample data*/
#ifdef LT_TOUCH_ENABLE_FILTER	
	u16 					last_x;
	u16					last_y;
	int 					move_detect;
	int					max_step;
	int					sample_offset;
#endif
#ifdef LT_TOUCH_ENABLE_STATISTIC
	int					int_count;
	int					data_count;
#endif
#ifdef LT_TOUCH_TIMER_POLL
	struct timer_list 		lt_touch_timer;	
#else	
	struct task_struct		*thread;
	wait_queue_head_t	ts_wait_queue;
	struct completion		thread_init;
	struct completion		thread_exit;
#endif
#ifdef CONFIG_INPUT_ANDROID
	struct keyemu_matrix	keyemu;
	int					inKeyEmu;
#endif
	struct delayed_work resume_work;
};
static struct lt_touch_data *lt_touch;

#define DIFF(x,y,result)		 \
	do{						\
		if(x>y) result=x-y;	\
		else result=y-x;		\
	} while(0)

#ifdef LT_TOUCH_ENABLE_FILTER
#define LT_TOUCH_CLEAR_LAST(lt_td) 			\
		do{									\
			lt_td->last_x = 0; lt_td->last_y = 0; lt_td->move_detect = 0;	\
		}while(0)
#else
#define LT_TOUCH_CLEAR_LAST(lt_td)
#endif	

// Warnning: if count != 3, pls rework it
static int pick_result(const u16* pData,u8 count,u16* result) 
{
	u8 i;
	u16 diff[3],diff_smallest;

#if (TOUCH_SAMPLE_NUM!=3)
	//#error pls rework it
#endif
	
	DIFF(pData[1],pData[0],diff[0]); 
	DIFF(pData[2],pData[0],diff[1]);
	DIFF(pData[2],pData[1],diff[2]);

	i = 0;
	if(diff[1]<diff[0]) {i++;diff[0]=diff[1];}
	if(diff[2]<diff[0]) i+=2;	

	if(i==0) {//Select diff[0]
		*result = (pData[0]+pData[1])/2;
		diff_smallest = diff[0];
	}else if(i==1) {//Select diff[1]
		*result = (pData[0]+pData[2])/2;
		diff_smallest = diff[1];
	}else { //Select diff[2]
		*result = (pData[1]+pData[2])/2;
		diff_smallest = diff[2];
	}

#ifdef LT_TOUCH_ENABLE_FILTER
{
	int sample_offset = lt_touch->sample_offset;
	if(sample_offset&&(diff_smallest>sample_offset)){
		ts_dbg("Find invalid sample,diff[0x%x,0x%x]\n",diff_smallest,sample_offset);
		return -EINVAL;
	}
}
#endif

	return 0;
}

static int lt_micco_touch_read(struct lt_touch_data *p_lt, u16 *x,  u16 *y, int *pen_state)
{
	int ret = -EIO;
	u8 i = 0;
	u16 t_x[2], t_y[2], diff[2];
	
	int intervalUs = p_lt->intervalUs_sample;

	micco_tsi_enable_tsi(1); /* Enable the auto measure of the TSI. */
	micco_tsi_readxy(&t_x[0], &t_x[0], *pen_state); /* readout old data */
	SAMPLE_UDELAY(intervalUs);
	micco_tsi_readxy(&t_x[0], &t_x[0], *pen_state); /* readout old data */
	
	for(i=0; i<TOUCH_SAMPLE_RETRY; i++) {
		SAMPLE_UDELAY(intervalUs);
		micco_tsi_readxy(&(t_x[0]), &(t_y[0]), *pen_state);	
		ts_raw_dbg("Raw ts data[0x%x,0x%x]\n", t_x[0], t_y[0]);		
		
		SAMPLE_UDELAY(intervalUs);
		micco_tsi_readxy(&(t_x[1]), &(t_y[1]), *pen_state);
		ts_raw_dbg("Raw ts data[0x%x,0x%x]\n", t_x[1], t_y[1]);

		DIFF(t_x[0], t_x[1], diff[0]);
		DIFF(t_y[0], t_y[1], diff[1]);

		if (diff[0]<6 && diff[1]<6) { //get it
			ret = 0;
			*x = (t_x[0]+t_x[1])/2;
			*y = (t_y[0]+t_y[1])/2;
			break;
		}
	}
	
	micco_tsi_enable_tsi(0);	
	return ret;
}

static void lt_touch_enable_stage(enum LT_TSI_STAGE stage)
{
	u8 val;

	if (stage==TSI_STAGE_POLL) {
		micco_enable_pen_down_irq(0);//Disable pen down irq while polling data
		//micco_tsi_enable_tsi(1); // enable poll
	} else if (stage==TSI_STAGE_OFF) {
		micco_enable_pen_down_irq(0);				
	} else {
		//micco_tsi_enable_tsi(0);
		micco_enable_pen_down_irq(1);/* Enable pen down irq */
		micco_read(MICCO_EVENT_C, &val);/* read to trigger irq */
	}
}

static int _lt_touch_report_data(struct lt_touch_data *p_lt, u16 rx, u16 ry)
{
#ifdef CONFIG_INPUT_TS_CALIBRATE
	struct ts_calibrate *pCalibInfo = &(p_lt->littleton_ts_input_dev->calibInfo);
	input_calibrate_tran(pCalibInfo, rx, ry);
	if (pCalibInfo->active && (rx>FB_WIDTH || ry>FB_HEIGHT)) /* invalid data */{
		ts_dbg("Invalid calibrated ts,x:%d,y:%d\n",rx, ry);
		return -EINVAL;
	}
#endif
	
	ts_dbg("Report ts data,x:0x%x,y:0x%x\n",rx, ry);
	input_report_abs(p_lt->littleton_ts_input_dev,ABS_X, rx & 0xfff);
	input_report_abs(p_lt->littleton_ts_input_dev,ABS_Y, ry & 0xfff);
#ifdef CONFIG_INPUT_ANDROID
	if (p_lt->inKeyEmu)
		input_report_abs(p_lt->littleton_ts_input_dev, ABS_PRESSURE, 0xfff);
	else 
		input_report_key(p_lt->littleton_ts_input_dev, BTN_TOUCH, 1);
#else
	input_report_abs(p_lt->littleton_ts_input_dev, ABS_PRESSURE, 0xfff);
#endif	
	input_sync(p_lt->littleton_ts_input_dev);

	return 0;
}

static int lt_touch_report_data(struct lt_touch_data *p_lt,u16 x,u16 y)
{
	int rx, ry;
#ifdef LT_TOUCH_ENABLE_FILTER
	u16 diff[2];
	int step = p_lt->max_step;
	
	if (step && (p_lt->last_x || p_lt->last_y)) {/* filter x,y */
		if (p_lt->last_x > x) {
			if ((p_lt->last_x-x)>step) {
				ts_dbg("Find sample x step[0x%x, 0x%x]\n",x, p_lt->last_x);
				x = p_lt->last_x - step;
			}
		} else {
			if ((x-p_lt->last_x)>step) {
				ts_dbg("Find sample x step[0x%x, 0x%x]\n",x, p_lt->last_x);
				x = p_lt->last_x + step;
			}
		}

		if (p_lt->last_y > y) {
			if ((p_lt->last_y-y)>step) {
				ts_dbg("Find sample y step[0x%x, 0x%x]\n",y, p_lt->last_y);
				y = p_lt->last_y - step;
			}
		} else {
			if ((y-p_lt->last_y)>step) {
				ts_dbg("Find  sample y step[0x%x, 0x%x]\n",y, p_lt->last_y);
				y = p_lt->last_y + step;
			}
		}
	}	
#endif

#ifdef LT_TOUCH_ENABLE_STATISTIC
	p_lt->data_count++;
#endif

	if (TSI_PEN_PRE_DOWN ==  p_lt->pen_state) {	
		p_lt->pen_state = TSI_PEN_DOWN;
		ts_dbg("Touch pen down!\n");
#ifdef CONFIG_INPUT_ANDROID /* check inKeyEmu */
		if (p_lt->keyemu.active && x>=p_lt->keyemu.x && x<=p_lt->keyemu.x+p_lt->keyemu.w &&
			y>=p_lt->keyemu.y && y<=p_lt->keyemu.y+p_lt->keyemu.h) {
			ts_dbg("Touch pen down in keyEmu \n");
			p_lt->inKeyEmu = 1;
		} else
			p_lt->inKeyEmu = 0;
#endif
	}

	if (p_lt->last_x || p_lt->last_y) {
		DIFF(p_lt->last_x, x, diff[0]);
		DIFF(p_lt->last_y, y, diff[1]);

		if ((diff[0]>5) || (diff[1]>5)) {
			p_lt->move_detect++;
		} else
			p_lt->move_detect = 0;
	}

#if 0
	if (p_lt->last_x || p_lt->last_y) { /* report last data to discard the unstable pen-up sample */
		rx = p_lt->last_x;
		ry = p_lt->last_y;
	} else {
		rx = x;
		ry = y;
	}
#else
	rx = x;
	ry = y;
#endif

	if (p_lt->move_detect==2)
		_lt_touch_report_data(p_lt, p_lt->last_x, p_lt->last_y);

	p_lt->last_x = x; p_lt->last_y = y; /* save to compare */

	if (p_lt->move_detect==1) /* report at next move detect */
		return 0;

	_lt_touch_report_data(p_lt, rx, ry);
	return 0;
}

/* The touchscreen main reader thread */
#ifdef LT_TOUCH_TIMER_POLL
/* Move TSI measure to workqueue??? */
static void lt_touch_timer_handler(unsigned long data)
{
	struct lt_touch_data *lt_td = (struct lt_touch_data *)data;
	u8 val;
	u16 tem_x,tem_y;
	int state = 0,ret;

	micco_read(MICCO_STATUS_A, &val); 
	if (val & 0x40) {	/* pen down */
		ret = lt_micco_touch_read(lt_td, &tem_x,&tem_y, &state);
		if (ret>=0) 
			lt_touch_report_data(lt_td,tem_x,tem_y);
		else
			LT_TOUCH_CLEAR_LAST(lt_td);
		if (lt_td->move_detect==1)
			mod_timer(&lt_td->lt_touch_timer, jiffies+lt_td->poll_jiffies/2);
		else
			mod_timer(&lt_td->lt_touch_timer, jiffies+lt_td->poll_jiffies);	
	} else if (TSI_PEN_DOWN ==  lt_td->pen_state) { /* Pen is up now */
#ifdef CONFIG_INPUT_ANDROID
		if (lt_td->inKeyEmu)
			input_report_abs(lt_td->littleton_ts_input_dev, ABS_PRESSURE, 0);
		else
			input_report_key(lt_td->littleton_ts_input_dev, BTN_TOUCH, 0);
#else
		input_report_abs(lt_td->littleton_ts_input_dev, ABS_PRESSURE, 0);
#endif
		input_sync(lt_td->littleton_ts_input_dev);

		lt_td->pen_state = TSI_PEN_UP;
		lt_touch_enable_stage(TSI_STAGE_DETECT);
		LT_TOUCH_CLEAR_LAST(lt_td);
		ts_dbg("Touch report pen up \n");
		
		mod_timer(&lt_td->lt_touch_timer, jiffies+HZ);/* timer for read to trigger irq */	
	} else if (TSI_PEN_PRE_DOWN ==  lt_td->pen_state) { /* Pen up */
		lt_td->pen_state = TSI_PEN_UP;
		lt_touch_enable_stage(TSI_STAGE_DETECT);
		mod_timer( &lt_td->lt_touch_timer, jiffies+HZ);/* timer for read to trigger irq */	
	} else if (TSI_PEN_UP==lt_td->pen_state) {
		micco_read(MICCO_EVENT_C, &val);/* read to trigger irq */
	}
}
 #else
static int ts_thread( void *d )
{
	struct lt_touch_data *lt_td = d;
	u16 tem_x,tem_y;
	int ret, state = 0;
	u8 val;

	DEFINE_WAIT(ts_wait);
	/* set up thread context */
	lt_td->thread = current;	
	daemonize("lt_touch_thread");	
	/* init is complete */
	complete(&lt_td->thread_init);

	/* touch reader loop */
	while (1) {	
		/* if the pen state is up, sleep */
		if (TSI_PEN_UP == lt_td->pen_state) {
			lt_touch_enable_pen_detect();
			
			try_to_freeze();
			prepare_to_wait(&lt_td->ts_wait_queue,
				&ts_wait, TASK_INTERRUPTIBLE);
			schedule();
			finish_wait(&lt_td->ts_wait_queue, &ts_wait);
			/* Someone wakeup us,do something */
		}

		ret = micco_read(MICCO_STATUS_A, &val); 
		if (val & 0x40) {	/* pen down */
			if (TSI_PEN_PRE_DOWN ==  lt_td->pen_state) {	
				lt_td->pen_state = TSI_PEN_DOWN;
				ts_dbg("Touch pen down \n");
#ifdef CONFIG_IPM
				ipm_event_notify(IPM_EVENT_UI, IPM_EVENT_DEVICE_TSI, NULL, 0);
#endif
			}

			ret = lt_micco_touch_read(lt_td, &tem_x,&tem_y, &state);			
			if(ret==0) lt_touch_report_data(lt_td,tem_x,tem_y);

			/* poll wait 10 ms */
			msleep(10*lt_td->poll_jiffies);
			continue;			
		} else if (TSI_PEN_DOWN ==  lt_td->pen_state) {	/* Pen is up now */
			input_report_abs(lt_td->littleton_ts_input_dev, ABS_PRESSURE, 0);
			ts_dbg("Touch report pen up \n");
			LT_TOUCH_CLEAR_LAST(lt_td);
			lt_td->pen_state = TSI_PEN_UP;
		}else if (TSI_PEN_PRE_DOWN ==  lt_td->pen_state) { /* Pen up */
			ts_dbg("Touch pen up when TSI_PEN_PRE_DOWN\n");
			lt_td->pen_state = TSI_PEN_UP;
		}

		if (!lt_td->thread)
			break;
	}

	lt_touch_enable_pen_detect();
	complete_and_exit(&lt_td->thread_exit, 0);
	return 0;
}
#endif

static int littleton_ts_input_open(struct input_dev *idev)
{
	unsigned long flags;
	struct lt_touch_data *lt_touch = (struct lt_touch_data *)input_get_drvdata(idev);
	int ret;

	pr_debug("%s: enter", __func__);

	if (lt_touch->suspended) {
		printk("touch has been suspended!\n");
		return -1;
	}

	spin_lock_irqsave(&lt_touch->ts_lock, flags);
	if (lt_touch->use_count++ ==0) {
		spin_unlock_irqrestore(&lt_touch->ts_lock, flags);

 #ifndef LT_TOUCH_TIMER_POLL
		init_completion(&lt_touch->thread_init);
		ret = kernel_thread(ts_thread, lt_touch, 0);
		if (ret < 0)
			return ret;
		wait_for_completion(&lt_touch->thread_init);
#else
		(void)ret;
#endif
	} else {
		spin_unlock_irqrestore(&lt_touch->ts_lock, flags);
	}

	mod_timer(&lt_touch->lt_touch_timer, jiffies+HZ);/* timer for read to trigger irq */
	
	return 0;
}

/* Kill the touchscreen thread and stop the touch digitiser. */
static void littleton_ts_input_close(struct input_dev *idev)
{
	unsigned long flags;
	struct lt_touch_data *lt_touch = (struct lt_touch_data *)input_get_drvdata(idev);

	pr_debug("%s: enter with use count = %d", __func__,lt_touch->use_count);

	spin_lock_irqsave(&lt_touch->ts_lock, flags);
	if (--lt_touch->use_count == 0) {
		spin_unlock_irqrestore(&lt_touch->ts_lock, flags);

 #ifndef LT_TOUCH_TIMER_POLL
		pr_debug("%s: kill thread", __func__);
		/* kill thread */
		if (lt_touch->thread) {
			init_completion(&lt_touch->thread_exit);
			lt_touch->thread = NULL;
			wake_up_interruptible(&lt_touch->ts_wait_queue);
			wait_for_completion(&lt_touch->thread_exit);
		}
#endif
	} else {
		spin_unlock_irqrestore(&lt_touch->ts_lock, flags);
	}
}

static void lt_touch_interrupt(unsigned long event)
{
	struct lt_touch_data *p_lt_touch = lt_touch;
	
	if ((p_lt_touch->use_count > 0) &&(TSI_PEN_UP== p_lt_touch->pen_state)) {
		p_lt_touch->pen_state = TSI_PEN_PRE_DOWN;
		lt_touch_enable_stage(TSI_STAGE_POLL);
		ts_dbg("%s enter \n",__FUNCTION__);
		
#ifdef LT_TOUCH_TIMER_POLL
		mod_timer(&p_lt_touch->lt_touch_timer,jiffies+p_lt_touch->poll_jiffies/2);
#else
		wake_up_interruptible(&p_lt_touch->ts_wait_queue);
#endif
#ifdef LT_TOUCH_ENABLE_STATISTIC
		p_lt_touch->int_count++;
#endif
	} else {
		ts_dbg("%s miss,use_count[%d],pen_state[%d]\n",__FUNCTION__,p_lt_touch->use_count,p_lt_touch->pen_state);
	}
	
	return;
}

/* sys fs */
#ifdef LT_TOUCH_ENABLE_FILTER
static ssize_t maxStep_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{					
	int max_step;
	struct lt_touch_data *p_lt_touch = (struct lt_touch_data*)dev_get_drvdata(dev);
	
	sscanf(buf, "%d", &max_step);
	if(max_step>=0) p_lt_touch->max_step = max_step;		
	return count;
}
static ssize_t maxStep_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	struct lt_touch_data *p_lt_touch = (struct lt_touch_data*)dev_get_drvdata(dev);	
	return sprintf(buf, "%d\n", p_lt_touch->max_step);
}
static ssize_t sampleOffset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{					
	int sample_offset;
	struct lt_touch_data *p_lt_touch = (struct lt_touch_data*)dev_get_drvdata(dev);
	
	sscanf(buf, "%d", &sample_offset);
	if(sample_offset>=0) p_lt_touch->sample_offset = sample_offset;		
	return count;
}
static ssize_t sampleOffset_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	struct lt_touch_data *p_lt_touch = (struct lt_touch_data*)dev_get_drvdata(dev);	
	return sprintf(buf, "%d\n", p_lt_touch->sample_offset);
}
static DEVICE_ATTR(maxStep,0644,maxStep_show,maxStep_store);
static DEVICE_ATTR(sampleOffset,0644,sampleOffset_show,sampleOffset_store);
#endif
#ifdef LT_TOUCH_ENABLE_STATISTIC
static ssize_t statistics_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	struct lt_touch_data *p_lt_touch = (struct lt_touch_data*)dev_get_drvdata(dev);	
	return sprintf(buf, "interrupt count %d,report data count %d\n",
					p_lt_touch->int_count,p_lt_touch->data_count);
}
static DEVICE_ATTR(statistics,0644,statistics_show,NULL);
#endif
static ssize_t pollJiffies_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{					
	int poll_jiffies;
	struct lt_touch_data *p_lt_touch = (struct lt_touch_data*)dev_get_drvdata(dev);
	
	sscanf(buf, "%d", &poll_jiffies);
	if(poll_jiffies && poll_jiffies<=20) 
		p_lt_touch->poll_jiffies= poll_jiffies;
	else
		printk("Valid parameter[1-20]\n");
	return count;
}
static ssize_t pollJiffies_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	struct lt_touch_data *p_lt_touch = (struct lt_touch_data*)dev_get_drvdata(dev);	
	return sprintf(buf, "%d\n", p_lt_touch->poll_jiffies);
}
static DEVICE_ATTR(pollJiffies,0644,pollJiffies_show,pollJiffies_store);
static ssize_t sampleIntervalUs_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int sampleIntervalUs;
	struct lt_touch_data *p_lt_touch = (struct lt_touch_data*)dev_get_drvdata(dev);
	
	sscanf(buf, "%d", &sampleIntervalUs);
	if(sampleIntervalUs>=0 && sampleIntervalUs<=1000) 
		p_lt_touch->intervalUs_sample= sampleIntervalUs;
	else
		printk("Valid parameter[0-1000]\n");
	return count;
}
static ssize_t sampleIntervalUs_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	struct lt_touch_data *p_lt_touch = (struct lt_touch_data*)dev_get_drvdata(dev);	
	return sprintf(buf, "%d\n", p_lt_touch->intervalUs_sample);
}
static DEVICE_ATTR(sampleIntervalUs,0644,sampleIntervalUs_show,sampleIntervalUs_store);

#ifdef CONFIG_INPUT_ANDROID
static ssize_t keyemu_matrix_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct lt_touch_data *p_lt_touch = (struct lt_touch_data*)dev_get_drvdata(dev);	
	struct keyemu_matrix* pkeyemu = &p_lt_touch->keyemu;	

	sscanf(buf, "%d %d %d %d %d\n", &pkeyemu->active,
			&pkeyemu->x, &pkeyemu->y, &pkeyemu->w, &pkeyemu->h);	
	return count;
}
static ssize_t keyemu_matrix_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	struct lt_touch_data *p_lt_touch = (struct lt_touch_data*)dev_get_drvdata(dev);	
	struct keyemu_matrix* pkeyemu = &p_lt_touch->keyemu;
	
	return sprintf(buf, "%sactive:%d %d %d %d\n", pkeyemu->active?"":"in", 
			pkeyemu->x, pkeyemu->y, pkeyemu->w, pkeyemu->h);
}
static DEVICE_ATTR(keyemu_matrix,0644,keyemu_matrix_show,keyemu_matrix_store);

static ssize_t sw_emulator_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct lt_touch_data *p_lt_touch = (struct lt_touch_data*)dev_get_drvdata(dev);		
	int code, value;

	sscanf(buf, "%d %d\n", &code, &value);
	input_report_switch(p_lt_touch->littleton_ts_input_dev, code, value);
	return count;
}
static DEVICE_ATTR(sw_emulator,0644,NULL,sw_emulator_store);
#endif

static ssize_t debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{	
	sscanf(buf, "%d", &bTouchDBG);
	return count;
}
static ssize_t debug_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	return sprintf(buf, "%d\n", bTouchDBG);
}
static DEVICE_ATTR(debug,0644,debug_show,debug_store);

static struct attribute *lt_touch_attributes[] = {	
	&dev_attr_pollJiffies.attr,
	&dev_attr_sampleIntervalUs.attr,
	&dev_attr_debug.attr,
#ifdef LT_TOUCH_ENABLE_FILTER
	&dev_attr_maxStep.attr,
	&dev_attr_sampleOffset.attr,
#endif
#ifdef LT_TOUCH_ENABLE_STATISTIC
	&dev_attr_statistics.attr,
#endif
#ifdef CONFIG_INPUT_ANDROID
	&dev_attr_keyemu_matrix.attr,
	&dev_attr_sw_emulator.attr,
#endif
	NULL,
};
static struct attribute_group lt_touch_attr_group ={
	.attrs=lt_touch_attributes,
};

static int lt_ts_probe(struct platform_device *pdev)
{
	int ret;
	struct lt_touch_data *p_lt_touch;
	struct input_dev 	*littleton_ts_input_dev;	

	/* register input device */
	littleton_ts_input_dev = input_allocate_device();	
      	littleton_ts_input_dev->name = "Littleton touchscreen";
	littleton_ts_input_dev->open = littleton_ts_input_open;
	littleton_ts_input_dev->close = littleton_ts_input_close;
	__set_bit(EV_ABS, littleton_ts_input_dev->evbit);
	__set_bit(ABS_X, littleton_ts_input_dev->absbit);
	__set_bit(ABS_Y, littleton_ts_input_dev->absbit);
	__set_bit(ABS_PRESSURE, littleton_ts_input_dev->absbit);
	
#ifdef CONFIG_INPUT_ANDROID
	set_bit(EV_KEY, littleton_ts_input_dev->evbit);
	set_bit(BTN_TOUCH, littleton_ts_input_dev->keybit);
#endif	

	/* Malloc lt_touch context */
	p_lt_touch = kzalloc(sizeof(struct lt_touch_data), GFP_KERNEL);
	if (!p_lt_touch) {
		ret = -ENOMEM;
		goto lt_touch_out;
	}
	lt_touch = p_lt_touch;
	
	p_lt_touch->littleton_ts_input_dev = littleton_ts_input_dev;
	platform_set_drvdata(pdev, p_lt_touch);
	input_set_drvdata(littleton_ts_input_dev, p_lt_touch);
	input_calibrate_enable(littleton_ts_input_dev);
	ret = input_register_device(littleton_ts_input_dev);

	ret = pmic_callback_register(PMIC_EVENT_TOUCH, lt_touch_interrupt);
	if (ret < 0)
		goto pmic_cb_out;

	spin_lock_init(&p_lt_touch->ts_lock);
	p_lt_touch->pen_state = TSI_PEN_UP;
	p_lt_touch->suspended = 0;
	p_lt_touch->use_count = 0;
	p_lt_touch->poll_jiffies = LT_TOUCH_POLL_JIFFIES;	
	p_lt_touch->intervalUs_sample = LT_TOUCH_SAMPLE_INTERVAL_US;
	
#ifdef LT_TOUCH_TIMER_POLL
	init_timer(&p_lt_touch->lt_touch_timer);
	p_lt_touch->lt_touch_timer.data    = (unsigned long)p_lt_touch;
	p_lt_touch->lt_touch_timer.function = lt_touch_timer_handler;
#else
	init_waitqueue_head(&p_lt_touch->ts_wait_queue);
#endif

#ifdef LT_TOUCH_ENABLE_FILTER
	p_lt_touch->sample_offset = LT_TOUCH_SAMPLE_OFFSET;
	p_lt_touch->max_step = LT_TOUCH_MAX_STEP;
#endif	

	ret = sysfs_create_group(&pdev->dev.kobj,&lt_touch_attr_group);

	micco_tsi_poweron();
	lt_touch_enable_stage(TSI_STAGE_DETECT);	
#ifdef CONFIG_YUHUA_MISC_DEV
	set_touch_detect(1);
#endif
	
	printk("Android micco touchscreen driver register succ\n");
	return 0;

pmic_cb_out:
	kfree(p_lt_touch);
lt_touch_out:
	input_unregister_device(littleton_ts_input_dev);
	return ret;	
}

static int lt_ts_remove(struct platform_device *pdev)
{
	struct lt_touch_data *p_lt_touch = (struct lt_touch_data *)platform_get_drvdata(pdev);
	sysfs_remove_group(&pdev->dev.kobj,&lt_touch_attr_group);
	input_unregister_device(p_lt_touch->littleton_ts_input_dev);
	return 0;
}

static int lt_ts_resume(struct platform_device *pdev)
{	
	struct lt_touch_data *p_lt_touch = (struct lt_touch_data *)platform_get_drvdata(pdev);
	micco_write(MICCO_ADC_AUTO_CONTROL_2, MICCO_ADC_AUTO_2_PENDET_EN);
	lt_touch_enable_stage(TSI_STAGE_DETECT);
	p_lt_touch->suspended = 0;
	mod_timer(&p_lt_touch->lt_touch_timer, jiffies+HZ);/* timer for read to trigger irq */
	return 0;
}

static int lt_ts_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct lt_touch_data *p_lt_touch = (struct lt_touch_data *)platform_get_drvdata(pdev);	
	p_lt_touch->suspended = 1; 
	lt_touch_enable_stage(TSI_STAGE_OFF);
	micco_write(MICCO_ADC_AUTO_CONTROL_2, 0);
	return 0;
}

static struct platform_driver lt_ts_drv = {
	.driver = {
		.name 	= "lt_ts", 
	},
	.probe		= lt_ts_probe,
	.remove		= lt_ts_remove,
	.resume 	= lt_ts_resume,
	.suspend 	= lt_ts_suspend,
};

static int __init littleton_ts_init( void )
{
	return platform_driver_register(&lt_ts_drv);
}

static void __exit littleton_ts_exit( void )
{
	/* We move these codes here because we want to detect the
	 * pen down event even when touch driver is not opened.
	 */
	micco_tsi_poweroff();	
	micco_enable_pen_down_irq(0);
	pmic_callback_unregister(PMIC_EVENT_TOUCH, lt_touch_interrupt);

	platform_driver_unregister(&lt_ts_drv);
}

module_init(littleton_ts_init);
module_exit(littleton_ts_exit);

MODULE_AUTHOR("Yin, Fengwei <fengwei.yin@marvell.com>");
MODULE_DESCRIPTION("Littleton Platfrom touch screen driver");
MODULE_LICENSE("GPL");
