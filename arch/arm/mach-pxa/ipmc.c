/*
 * linux/arch/arm/mach-pxa/ipmc.c
 *
 * Provide ioctl interface to application, to specify more detailed info and change
 * system's behaviour.
 *
 * Copyright (C) 2003-2004 Intel Corporation.
 *
 * Author:  Cain Yuan <cain.yuan@intel.com>
 *
 * Updated for DVFM by Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * If wana use it please use "mknod /dev/ipmc c 10 90" to create the dev file.
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/poll.h>

#include <asm/uaccess.h>
#include <asm/system.h>
#include <mach/hardware.h>
#include <mach/ipmc.h>
#include <mach/prm.h>
#include <mach/dvfm.h>

static struct mspm_app_info mspm_app_array[MSPM_MAX_CLIENT];
static DECLARE_MUTEX(app_sem);

static struct ipme_queue ipme_queue;

static spinlock_t ipmq_lock = SPIN_LOCK_UNLOCKED;

/* PRM client ID */
extern int mspm_prm_id;
extern int (*event_notify)(int, int, void *, unsigned int);

/*********************** IPM Queue Operation *****************************/

/* getting and handling events are only used by IPM */
static int ipmq_get(struct ipm_event *ipme)
{
	unsigned long flags;

	spin_lock_irqsave(&ipmq_lock, flags);

	if (!ipme_queue.len) {
		spin_unlock_irqrestore(&ipmq_lock, flags);
		return -1;
	}
	memcpy(ipme, ipme_queue.ipmes + ipme_queue.tail, sizeof(struct ipm_event));
	ipme_queue.len--;
	ipme_queue.tail = (ipme_queue.tail + 1) % MAX_IPME_NUM;

	spin_unlock_irqrestore(&ipmq_lock, flags);

	return 0;
}

static int ipmq_clear(void)
{
	unsigned long flags;

	spin_lock_irqsave(&ipmq_lock, flags);

	ipme_queue.head = ipme_queue.tail = 0;
	ipme_queue.len = 0;

	spin_unlock_irqrestore(&ipmq_lock, flags);

	return 0;
}

static int ipmq_put(struct ipm_event *ipme)
{
	unsigned long flags;

	spin_lock_irqsave(&ipmq_lock, flags);

	if (ipme_queue.len == MAX_IPME_NUM) {
		spin_unlock_irqrestore(&ipmq_lock, flags);
		return -1;
	}
	memcpy(ipme_queue.ipmes + ipme_queue.head, ipme, sizeof(struct ipm_event));
	ipme_queue.len++;
	ipme_queue.head = (ipme_queue.head + 1) % MAX_IPME_NUM;

	spin_unlock_irqrestore(&ipmq_lock, flags);

	/* wake up the waiting process */
	wake_up_interruptible(&ipme_queue.waitq);

	return 0;
}

static __inline int ipmq_empty(void)
{
	return (ipme_queue.len > 0) ? 0 : 1;
}


/* register application client into mspm_app_array */
static int add_client(int pid, char *name)
{
	int i, client, ret;
	int free_entry = -1;

	if ((name == NULL) || (pid <= 0))
		goto out_param;
	down(&app_sem);
	for (i = 0; i < MSPM_MAX_CLIENT; i++) {
		/* check whether it's already registered */
		if (pid == mspm_app_array[i].pid) {
			up(&app_sem);
			goto out_dup;
		}

		/* find the first free entry that can be registered */
		if ((mspm_app_array[i].pid == 0)
			&& (free_entry == -1))
			free_entry = i;
	}
	if ((free_entry == -1) && (i == MSPM_MAX_CLIENT)) {
		up(&app_sem);
		goto out_num;
	} else if ((free_entry == -1) && (i == MSPM_MAX_CLIENT))
		free_entry = i;

	ret = dvfm_register(name, &client);
	if (ret) {
		up(&app_sem);
		goto out_param;
	}

	/* add pid and name into tail of application array */
	memset(&mspm_app_array[free_entry], 0, sizeof(struct mspm_app_info));
	mspm_app_array[free_entry].pid = pid;
	memcpy(&mspm_app_array[free_entry].name, name,
		min(MSPM_APP_NAME_LEN, (int)strlen(name)));
	mspm_app_array[free_entry].mspm_cid = client;
	mspm_app_array[free_entry].mspm_policy = MSPM_POLICY_FREE;
	up(&app_sem);

	return 0;
out_num:
	return -EUSERS;
out_dup:
	return -EEXIST;
out_param:
	return -EINVAL;
}

/* unregister application client from mspm_app_array */
static int release_client(int pid)
{
	int i;

	down(&app_sem);
	for (i = 0; i < MSPM_MAX_CLIENT; i++) {
		if (pid == mspm_app_array[i].pid) {
			dvfm_unregister(mspm_app_array[i].name,
					&mspm_app_array[i].mspm_cid);
			memset(&mspm_app_array[i], 0,
				sizeof(struct mspm_app_info));
			up(&app_sem);
			return 0;
		}
	}
	up(&app_sem);
	/* can't find the client in mspm_app_array */
	if (i == MSPM_MAX_CLIENT)
		return -EINVAL;
	return 0;
}

/* convert the policy to DVFM constraint */
static int set_policy(int pid, int policy)
{
	int i, cid;

	down(&app_sem);
	for (i = 0; i < MSPM_MAX_CLIENT; i++) {
		if (pid == mspm_app_array[i].pid) {
			cid = mspm_app_array[i].mspm_cid;
			switch (policy) {
			case MSPM_POLICY_FREE:
				dvfm_enable(cid);
				break;
			case MSPM_POLICY_NOIDLE_REPLACE:
				dvfm_enable(cid);
				dvfm_disable_op_name("D0CS", cid);
				dvfm_disable_op_name("D1", cid);
				dvfm_disable_op_name("D2", cid);
				if (cpu_is_pxa935())
					dvfm_disable_op_name("CG", cid);
				break;
			case MSPM_POLICY_PERF:
				dvfm_enable(cid);
				/* disable all op below 208MHz */
				dvfm_disable_op_name("D0CS", cid);
				dvfm_disable_op_name("D1", cid);
				dvfm_disable_op_name("D2", cid);
				if (cpu_is_pxa935())
					dvfm_disable_op_name("CG", cid);
				dvfm_disable_op_name("104M", cid);
				dvfm_disable_op_name("156M", cid);
				dvfm_disable_op_name("208M", cid);
				break;
			case MSPM_POLICY_HIGHPERF:
				dvfm_disable(cid);
				break;
			default:
				up(&app_sem);
				return -EINVAL;
			}
			/* record policy */
			mspm_app_array[i].mspm_policy = policy;
			up(&app_sem);
			return 0;
		}
	}
	up(&app_sem);
	return -ENOENT;
}

/* return client numbers */
static int query_client_num(void)
{
	int i, num = 0;

	down(&app_sem);
	for (i = 0; i < MSPM_MAX_CLIENT; i++) {
		if (mspm_app_array[i].pid)
			num++;
	}
	up(&app_sem);
	return num;
}

/* query client information */
static int query_client_info(int pid, struct mspm_app_info *info)
{
	int i;

	down(&app_sem);
	for (i = 0; i < MSPM_MAX_CLIENT; i++) {
		if (pid == mspm_app_array[i].pid) {
			memcpy(info, &mspm_app_array[i],
				sizeof(struct mspm_app_info));
			up(&app_sem);
			return 0;
		}
	}
	up(&app_sem);
	return -ENOENT;
}

int ipmc_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int ret = 0;
	int data;
	struct pxa3xx_fv_info param;
	struct ipmc_ioctl_setfvop setop;
	struct ipmc_ioctl_getfvopinfo getinfo;
	struct mspm_app_info app_info;

	switch (cmd) {
	case IPMC_IOCTL_GET_FV_OP:
		if ((mspm_prm_id < 0) || (mspm_prm_id > MAX_CLIENTS)) {
			ret = -EINVAL;
			break;
		}
		/* read current operating point */
		if ((ret = cop_get_cur_cop(mspm_prm_id, &data, &param)))
			break;
		if (copy_to_user((int *)arg, &data, sizeof(data))) {
			pr_debug("Error from copy_to_user.\n");
			ret = -EFAULT;
			break;
		}
		break;

	case IPMC_IOCTL_SET_FV_OP:
		if ((mspm_prm_id < 0) || (mspm_prm_id > MAX_CLIENTS)) {
			ret = -EINVAL;
			break;
		}
		if (copy_from_user(&setop,
			(struct ipmc_ioctl_setfvop_setop *)arg,
			sizeof(setop))) {
			pr_debug("Error from copy_from_user.\n");
			ret = -EFAULT;
			break;
		}
		ret = cop_set_cop(mspm_prm_id, setop.op, setop.mode);
		break;

	case IPMC_IOCTL_GET_FV_OP_COUNT:
		data = cop_get_num_of_cops();
		if (copy_to_user((int *)arg, &data, sizeof(data)))
			ret = -EFAULT;
		break;

	case IPMC_IOCTL_GET_FV_OP_INFO:
		if ((mspm_prm_id < 0) || (mspm_prm_id > MAX_CLIENTS)) {
			ret = -EINVAL;
			break;
		}
		if (copy_from_user(&getinfo,
			(struct ipmc_ioctl_getfvopinfo *)arg,
			sizeof(getinfo))) {
			pr_debug("Error from copy_from_user.\n");
			ret = -EFAULT;
			break;
		}
		if ((ret = cop_get_cop(mspm_prm_id, getinfo.op,
			&(getinfo.info))))
			break;
		if (copy_to_user((struct ipmc_ioctl_getfvopinfo *)arg,
			&getinfo, sizeof(getinfo)))
			ret = -EFAULT;
		break;

	/* get rid of those event before. */
	case IPMC_IOCTL_CLEAR_EVENTLIST:
		ipmq_clear();
		break;


	case IPMC_IOCTL_SET_WAKEUPSRC:
		if (copy_from_user(&wakeup_src, (pm_wakeup_src_t *)arg,
			sizeof(pm_wakeup_src_t)))
			ret = -EFAULT;
		break;
	case IPMC_IOCTL_GET_WAKEUPSRC:
		if (copy_to_user((pm_wakeup_src_t *)arg, &wakeup_src,
			sizeof(pm_wakeup_src_t)))
			ret = -EFAULT;
		break;
	case IPMC_IOCTL_SET_DEEPIDLE:
		if (copy_from_user(&enable_deepidle, (unsigned int *)arg,
			sizeof(unsigned int)))
			ret = -EFAULT;
		break;


	case IPMC_IOCTL_REGISTER:
		if ((current->pid != 0) && (current->comm != NULL)) {
			ret = add_client((int)current->pid, current->comm);
			break;
		}
		printk(KERN_WARNING "Failed to register application:%d,%d\n",
				current->pid, ret);
		break;

	case IPMC_IOCTL_UNREGISTER:
		if (current->pid != 0) {
			ret = release_client((int)current->pid);
			break;
		}
		printk(KERN_WARNING "Failed to unregister application:%d,%d\n",
				current->pid, ret);
		break;

	case IPMC_IOCTL_SET_POLICY:
		/* get policy */
		if (copy_from_user(&data, (int *)arg, sizeof(int))) {
			ret = -EFAULT;
			break;
		}
		if (current->pid != 0) {
			ret = set_policy((int)current->pid, data);
			break;
		}
		ret = -EFAULT;
		break;

	case IPMC_IOCTL_QUERY_NUM:
		data = query_client_num();
		if (copy_to_user((int *)arg, &data, sizeof(int)))
			ret = -EFAULT;
		break;

	case IPMC_IOCTL_QUERY_INFO:
		if (current->pid != 0) {
			if (query_client_info(current->pid, &app_info)) {
				ret = -ENOENT;
				break;
			}
			if (copy_to_user((struct mspm_app_info *)arg,
					&app_info,
					sizeof(struct mspm_app_info))) {
				ret = -EFAULT;
				break;
			}
		}
		break;

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

/* Return a ipm event or get blocked until there is a event. */
ssize_t ipmc_read(struct file *filep, char *buf, size_t count,
		loff_t *f_pos)
{
	DECLARE_WAITQUEUE(wait, current);
	struct ipm_event data;
	ssize_t ret;

	if (count < sizeof(struct ipm_event))
		return -EINVAL;

	add_wait_queue(&ipme_queue.waitq, &wait);
	set_current_state(TASK_INTERRUPTIBLE);

	for (;;) {
		if (filep->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto out;
		}

		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			goto out;
		}
		/* Get the event out. If no, blocked. */
		if ( !ipmq_empty()){
			ipmq_get(&data);
			break;
		}
		/* No events and no error, block it. */
		schedule();
	}

	/* pass the event to user space. */
	ret = copy_to_user((struct ipm_event *)buf, &data,
			sizeof(struct ipm_event));
	if (ret) {
		ret = -EFAULT;
		goto out;
	}
	ret = sizeof(struct ipm_event);

out:
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&ipme_queue.waitq, &wait);
	return ret;
}

/*
 * Write will do nothing. If need to pass down some information use ioctl
 * interface instead.
 */
ssize_t ipmc_write(struct file *filep, const char *buf, size_t count,
                loff_t *f_pos)
{
	return 0;
}

/* poll for ipm event */
unsigned int ipmc_poll(struct file *file, poll_table *wait)
{
	poll_wait(file, &ipme_queue.waitq, wait);
	if (!ipmq_empty())
		return POLLIN | POLLRDNORM;

	return 0;
}

int ipmc_open(struct inode *inode, struct file *filep)
{
	return 0;
}

int ipmc_close(struct inode *inode, struct file *filep)
{
	if (current->pid != 0) {
		release_client((int)current->pid);
		goto out;
	}
	printk(KERN_WARNING "Failed to unregister application in MSPM\n");
out:
	return 0;
}

static struct file_operations ipmc_fops = {
	owner:		THIS_MODULE,
	open:		ipmc_open,
	read:		ipmc_read,
	write:		ipmc_write,
	poll:		ipmc_poll,
	ioctl:		ipmc_ioctl,
	release:	ipmc_close,
};

static struct miscdevice ipmc_misc_device = {
	minor:		IPMC_MINOR,
	name:		"ipmc",
	fops:		&ipmc_fops,
};

/*
 * Below is the event list maintain functions, the same as keypad.c
 */
static int ipmq_init(void)
{
	/* init ipme queue */
	ipme_queue.head = ipme_queue.tail = 0;
	ipme_queue.len = 0;
	init_waitqueue_head(&ipme_queue.waitq);
	return 0;
}

static int __init ipmc_init(void)
{
	int ret;

	/* Clear event queue. */
	ipmq_init();

	if ((ret = misc_register(&ipmc_misc_device)) != 0) {
		printk(KERN_ERR "Could not register device ipmc, res = %d.\n ",
				ret);
		return -EBUSY;
	}
	printk(KERN_INFO "Register device ipmc successful.\n ");

	event_notify = ipm_event_notify;

	/* clear array */
	memset(&mspm_app_array, 0, sizeof(mspm_app_array));
	return 0;
}

static void ipmc_exit(void)
{
	event_notify = NULL;
	misc_deregister(&ipmc_misc_device);
}

/*
 * IPM event can be posted by this function.
 * If we need to do some pre-processing of those events we can add code here.
 * Also attach the processed result to info.
 */
int ipm_event_notify(int type, int kind, void *info, unsigned int info_len)
{
	struct ipm_event events;
	int len = 0;

	if (info_len > INFO_SIZE)
		len = INFO_SIZE;
	else if ((info_len < INFO_SIZE) && (info_len > 0))
		len = info_len;
	memset(&events, 0, sizeof(struct ipm_event));
	events.type = type;
	events.kind = kind;
	if ((len > 0) && (info != NULL)) {
		memcpy(events.info, info, len);
	}
	ipmq_put(&events);

	return len;
}

EXPORT_SYMBOL(ipm_event_notify);

module_init(ipmc_init);
module_exit(ipmc_exit);

MODULE_DESCRIPTION("IPM control device");
MODULE_LICENSE("GPL");

