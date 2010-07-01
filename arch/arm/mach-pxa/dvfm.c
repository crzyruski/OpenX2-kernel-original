/*
 * DVFM Abstract Layer
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>
#include <linux/string.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/notifier.h>

#include <asm/atomic.h>
#include <mach/dvfm.h>

#define MAX_DEVNAME_LEN	32
/* This structure is used to dump device name list */
struct name_list {
	int	id;
	char	name[MAX_DEVNAME_LEN];
};


static ATOMIC_NOTIFIER_HEAD(dvfm_freq_notifier_list);

/* This list links log of dvfm operation */
static struct info_head dvfm_trace_list = {
	.list = LIST_HEAD_INIT(dvfm_trace_list.list),
	.lock = RW_LOCK_UNLOCKED,
	.device = {0, 0, 0, 0, 0, 0, 0, 0},
};

/* This idx is used for user debug */
static int dvfm_dev_idx;

struct dvfm_driver *dvfm_driver = NULL;
struct info_head *dvfm_op_list = NULL;

unsigned int cur_op;			/* current operating point */
unsigned int def_op;			/* default operating point */
unsigned int op_nums = 0;		/* number of operating point */

static atomic_t lp_count = ATOMIC_INIT(0);	/* number of blocking lowpower mode */

extern struct sysdev_class cpu_sysdev_class;

int dvfm_find_op(int index, struct op_info **op)
{
	struct op_info *p = NULL;
	unsigned long flags;

	read_lock_irqsave(&dvfm_op_list->lock, flags);
	if (list_empty(&dvfm_op_list->list)) {
		read_unlock_irqrestore(&dvfm_op_list->lock, flags);
		return -ENOENT;
	}
	list_for_each_entry(p, &dvfm_op_list->list, list) {
		if (p->index == index) {
			*op = p;
			read_unlock_irqrestore(&dvfm_op_list->lock, flags);
			return 0;
		}
	}
	read_unlock_irqrestore(&dvfm_op_list->lock, flags);
	return -ENOENT;
}

/* Display current operating point */
static ssize_t op_show(struct sys_device *sys_dev, struct sysdev_attribute * attr, char *buf)
{
	struct op_info *op = NULL;
	int len = 0, req_op;

	req_op = cur_op;

	if (dvfm_driver->dump) {
		if (!dvfm_find_op(req_op, &op)) {
			len = dvfm_driver->dump(dvfm_driver->priv, op, buf);
		}
	}

	return len;
}

/* Set current operating point */
static ssize_t op_store(struct sys_device *sys_dev, struct sysdev_attribute * attr, const char *buf,
				size_t len)
{
	int new_op;

	sscanf(buf, "%u", &new_op);
	dvfm_request_op(new_op);
	return len;
}
SYSDEV_ATTR(op, 0644, op_show, op_store);

/* Dump all operating point */
static ssize_t ops_show(struct sys_device *sys_dev, struct sysdev_attribute * attr, char *buf)
{
	struct op_info *entry = NULL;
	int len = 0;
	char *p = NULL;

	if (!dvfm_driver->dump)
		return 0;
	read_lock(&dvfm_op_list->lock);
	if (!list_empty(&dvfm_op_list->list)) {
		list_for_each_entry(entry, &dvfm_op_list->list, list) {
			p = buf + len;
			len += dvfm_driver->dump(dvfm_driver->priv, entry, p);
		}
	}
	read_unlock(&dvfm_op_list->lock);

	return len;
}
SYSDEV_ATTR(ops, 0444, ops_show, NULL);

/* Dump all enabled operating point */
static ssize_t enable_op_show(struct sys_device *sys_dev, struct sysdev_attribute * attr, char *buf)
{
	struct op_info *entry = NULL;
	int len = 0;
	char *p = NULL;

	if (!dvfm_driver->dump)
		return 0;
	read_lock(&dvfm_op_list->lock);
	if (!list_empty(&dvfm_op_list->list)) {
		list_for_each_entry(entry, &dvfm_op_list->list, list) {
			if (find_first_bit(entry->device, DVFM_MAX_CLIENT) == DVFM_MAX_CLIENT) {
				p = buf + len;
				len += dvfm_driver->dump(dvfm_driver->priv, entry, p);
			}
		}
	}
	read_unlock(&dvfm_op_list->lock);

	return len;
}

static ssize_t enable_op_store(struct sys_device *sys_dev, struct sysdev_attribute * attr, const char *buf,
				size_t len)
{
	int op, level;

	sscanf(buf, "%u,%u", &op, &level);
	if (level) {
		dvfm_enable_op(op, dvfm_dev_idx);
	} else
		dvfm_disable_op(op, dvfm_dev_idx);
	return len;
}
SYSDEV_ATTR(enable_op, 0644, enable_op_show, enable_op_store);

/*
 * Dump blocked device on specified OP.
 * And dump the device list that is tracked.
 */
static ssize_t trace_show(struct sys_device *sys_dev, struct sysdev_attribute * attr, char *buf)
{
	struct op_info *op_entry = NULL;
	struct dvfm_trace_info *entry = NULL;
	int len = 0, i;
	/* copy of constraint in operating point */
	unsigned long blocked[8];

	for (i = 0; i < op_nums; i++) {
		memset(blocked, 0, sizeof(blocked));

		read_lock(&dvfm_op_list->lock);
		/* op list shouldn't be empty because op_nums is valid */
		list_for_each_entry(op_entry, &dvfm_op_list->list, list) {
			if (op_entry->index == i)
				memcpy(blocked, (op_entry->device), sizeof(blocked));
		}
		read_unlock(&dvfm_op_list->lock);

		/* If no device is blocking this OP, continue to check next OP. */
		if (find_first_bit(blocked, DVFM_MAX_CLIENT) == DVFM_MAX_CLIENT)
			continue;

		len += sprintf(buf + len, "Blocked devices on OP%d:", i);

		read_lock(&dvfm_trace_list.lock);
		list_for_each_entry(entry, &dvfm_trace_list.list, list) {
			if (test_bit(entry->index, blocked))
				len += sprintf(buf + len, "%s, ", entry->name);
		}
		read_unlock(&dvfm_trace_list.lock);

		len += sprintf(buf + len, "\n");
	}
	if (len == 0)
		len += sprintf(buf + len, "None device block OP\n");
	len += sprintf(buf + len, "Trace device list:\n");

	read_lock(&dvfm_trace_list.lock);
	list_for_each_entry(entry, &dvfm_trace_list.list, list) {
		len += sprintf(buf + len, "%s, ", entry->name);
	}
	read_unlock(&dvfm_trace_list.lock);

	len += sprintf(buf + len, "\n");
	return len;
}
SYSDEV_ATTR(trace, 0444, trace_show, NULL);

static struct attribute *dvfm_attr[] = {
	&attr_op.attr,
	&attr_ops.attr,
	&attr_enable_op.attr,
	&attr_trace.attr,
};

int dvfm_op_count(void)
{
	int ret = -EINVAL;

	if (dvfm_driver && dvfm_driver->count)
		ret = dvfm_driver->count(dvfm_driver->priv, dvfm_op_list);
	return ret;
}
EXPORT_SYMBOL(dvfm_op_count);

int dvfm_get_op(struct op_info **p)
{
	int req_op;

	req_op = cur_op;

	if (dvfm_find_op(req_op, p))
		return -EINVAL;
	return req_op;
}
EXPORT_SYMBOL(dvfm_get_op);

int dvfm_get_defop(void)
{
	return def_op;
}
EXPORT_SYMBOL(dvfm_get_defop);

int dvfm_get_opinfo(int index, struct op_info **p)
{
	if (dvfm_find_op(index, p))
		return -EINVAL;
	return 0;
}
EXPORT_SYMBOL(dvfm_get_opinfo);

int dvfm_set_op(struct dvfm_freqs *freqs, unsigned int new,
		unsigned int relation)
{
	int ret = -EINVAL;

	/* check whether dvfm is enabled */
	if (!dvfm_driver || !dvfm_driver->count)
		return -EINVAL;
	if (dvfm_driver->set)
		ret = dvfm_driver->set(dvfm_driver->priv, freqs, new, relation);
	return ret;
}

/* Request operating point. System may set higher frequency because of
 * device constraint.
 */
int dvfm_request_op(int index)
{
	int ret = -EFAULT;

	/* check whether dvfm is enabled */
	if (!dvfm_driver || !dvfm_driver->count)
		return -EINVAL;
	if (dvfm_driver->request_set)
		ret = dvfm_driver->request_set(dvfm_driver->priv, index);
	return ret;
}
EXPORT_SYMBOL(dvfm_request_op);

/*
 * Device remove the constraint on OP.
 */
int dvfm_enable_op(int index, int dev_idx)
{
	struct op_info *p = NULL;
	int num;

	/* check whether dvfm is enabled */
	if (!dvfm_driver || !dvfm_driver->count)
		return -EINVAL;
	/* only registered device can invoke DVFM operation */
	if ((dev_idx >= DVFM_MAX_CLIENT) || dev_idx < 0)
		return -ENOENT;
	num = dvfm_driver->count(dvfm_driver->priv, dvfm_op_list);
	if (num <= index)
		return -ENOENT;
	if (!dvfm_find_op(index, &p)) {
		write_lock(&dvfm_op_list->lock);
		/* remove device ID */
		clear_bit(dev_idx, p->device);
		write_unlock(&dvfm_op_list->lock);
		dvfm_driver->enable_op(dvfm_driver->priv, index, RELATION_LOW);
	}
	return 0;
}

/*
 * Device set constraint on OP
 */
int dvfm_disable_op(int index, int dev_idx)
{
	struct op_info *p = NULL;
	int num;

	/* check whether dvfm is enabled */
	if (!dvfm_driver || !dvfm_driver->count)
		return -EINVAL;
	/* only registered device can invoke DVFM operation */
	if ((dev_idx >= DVFM_MAX_CLIENT) || dev_idx < 0)
		return -ENOENT;
	num = dvfm_driver->count(dvfm_driver->priv, dvfm_op_list);
	if (num <= index)
		return -ENOENT;
	if (!dvfm_find_op(index, &p)) {
		write_lock(&dvfm_op_list->lock);
		/* set device ID */
		set_bit(dev_idx, p->device);
		write_unlock(&dvfm_op_list->lock);
		dvfm_driver->disable_op(dvfm_driver->priv, index, RELATION_LOW);
	}
	return 0;
}
EXPORT_SYMBOL(dvfm_enable_op);
EXPORT_SYMBOL(dvfm_disable_op);

int dvfm_enable_op_name(char *name, int dev_idx)
{
	struct op_info *p = NULL;
	int index;

	if (!dvfm_driver || !dvfm_driver->name || !name)
		return -EINVAL;
	/* only registered device can invoke DVFM operation */
	if ((dev_idx >= DVFM_MAX_CLIENT) || dev_idx < 0)
		return -ENOENT;
	list_for_each_entry(p, &dvfm_op_list->list, list) {
		if (!strcmp(dvfm_driver->name(dvfm_driver->priv, p), name)) {
			index = p->index;
			write_lock(&dvfm_op_list->lock);
			clear_bit(dev_idx, p->device);
			write_unlock(&dvfm_op_list->lock);
			dvfm_driver->enable_op(dvfm_driver->priv,
					index, RELATION_LOW);
			break;
		}
	}
	return 0;
}

int dvfm_disable_op_name(char *name, int dev_idx)
{
	struct op_info *p = NULL;
	int index;

	if (!dvfm_driver || !dvfm_driver->name || !name)
		return -EINVAL;
	/* only registered device can invoke DVFM operation */
	if ((dev_idx >= DVFM_MAX_CLIENT) || dev_idx < 0)
		return -ENOENT;
	list_for_each_entry(p, &dvfm_op_list->list, list) {
		if (!strcmp(dvfm_driver->name(dvfm_driver->priv, p), name)) {
			index = p->index;
			write_lock(&dvfm_op_list->lock);
			set_bit(dev_idx, p->device);
			write_unlock(&dvfm_op_list->lock);
			dvfm_driver->disable_op(dvfm_driver->priv,
					index, RELATION_LOW);
			break;
		}
	}
	return 0;
}
EXPORT_SYMBOL(dvfm_enable_op_name);
EXPORT_SYMBOL(dvfm_disable_op_name);

/* donot request a new op */
int dvfm_enable_op_name_aync(char *name, int dev_idx)
{
	struct op_info *p = NULL;
	int index;

	if (!dvfm_driver || !dvfm_driver->name || !name)
		return -EINVAL;
	/* only registered device can invoke DVFM operation */
	if ((dev_idx >= DVFM_MAX_CLIENT) || dev_idx < 0)
		return -ENOENT;
	list_for_each_entry(p, &dvfm_op_list->list, list) {
		if (!strcmp(dvfm_driver->name(dvfm_driver->priv, p), name)) {
			index = p->index;
			write_lock(&dvfm_op_list->lock);
			clear_bit(dev_idx, (void *)&p->device);
			write_unlock(&dvfm_op_list->lock);
			break;
		}
	}
	return 0;
}

/* Only enable those safe operating point */
int dvfm_enable(int dev_idx)
{
	if (!dvfm_driver || !dvfm_driver->count || !dvfm_driver->enable_dvfm)
		return -ENOENT;
	return dvfm_driver->enable_dvfm(dvfm_driver->priv, dev_idx);
}

/* return whether the result is zero */
int dvfm_disable(int dev_idx)
{
	if (!dvfm_driver || !dvfm_driver->count || !dvfm_driver->disable_dvfm)
		return -ENOENT;
	return dvfm_driver->disable_dvfm(dvfm_driver->priv, dev_idx);
}

/* return whether the result is zero */
int dvfm_enable_pm(void)
{
	return atomic_inc_and_test(&lp_count);
}

/* return whether the result is zero */
int dvfm_disable_pm(void)
{
	return atomic_dec_and_test(&lp_count);
}

int dvfm_notifier_frequency(struct dvfm_freqs *freqs, unsigned int state)
{
	int ret;

	switch (state) {
	case DVFM_FREQ_PRECHANGE:
		ret = atomic_notifier_call_chain(&dvfm_freq_notifier_list,
					DVFM_FREQ_PRECHANGE, freqs);
		if (ret != NOTIFY_DONE)
			pr_debug("Failure in device driver before "
				"switching frequency\n");
		break;
	case DVFM_FREQ_POSTCHANGE:
		ret = atomic_notifier_call_chain(&dvfm_freq_notifier_list,
					DVFM_FREQ_POSTCHANGE, freqs);
		if (ret != NOTIFY_DONE)
			pr_debug("Failure in device driver after "
				"switching frequency\n");
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

int dvfm_register_notifier(struct notifier_block *nb, unsigned int list)
{
	int ret;

	switch (list) {
	case DVFM_FREQUENCY_NOTIFIER:
		ret = atomic_notifier_chain_register(
				&dvfm_freq_notifier_list, nb);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL(dvfm_register_notifier);

int dvfm_unregister_notifier(struct notifier_block *nb, unsigned int list)
{
	int ret;

	switch (list) {
	case DVFM_FREQUENCY_NOTIFIER:
		ret = atomic_notifier_chain_unregister(
				&dvfm_freq_notifier_list, nb);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL(dvfm_unregister_notifier);

/*
 * add device into trace list
 * return device index
 */
static int add_device(char *name)
{
	struct dvfm_trace_info	*entry = NULL, *new = NULL;
	int min;

	min = find_first_zero_bit(dvfm_trace_list.device, DVFM_MAX_CLIENT);
	/* client list is full */
	if (min == DVFM_MAX_CLIENT)
		return -EINVAL;

	/* If device trace table is NULL */
	new = kzalloc(sizeof(struct dvfm_trace_info), GFP_ATOMIC);
	if (new == NULL)
		goto out_mem;
	/* add new item */
	strcpy(new->name, name);
	new->index = min;
	/* insert the new item in increasing order */
	list_for_each_entry(entry, &dvfm_trace_list.list, list) {
		if (entry->index > min) {
			list_add_tail(&(new->list), &(entry->list));
			goto inserted;
		}
	}
	list_add_tail(&(new->list), &(dvfm_trace_list.list));
inserted:
	set_bit(min, dvfm_trace_list.device);

	return min;
out_mem:
	return -ENOMEM;
}

/*
 * Query the device number that registered in DVFM
 */
int dvfm_query_device_num(void)
{
	int count = 0;
	struct dvfm_trace_info *entry = NULL;

	read_lock(&dvfm_trace_list.lock);
	list_for_each_entry(entry, &dvfm_trace_list.list, list) {
		count++;
	}
	read_unlock(&dvfm_trace_list.lock);
	return count;
}
EXPORT_SYMBOL(dvfm_query_device_num);

/*
 * Query all device name that registered in DVFM
 */
int dvfm_query_device_list(void *mem, int len)
{
	int count = 0, size;
	struct dvfm_trace_info *entry = NULL;
	struct name_list *p = (struct name_list *)mem;

	count = dvfm_query_device_num();
	size = sizeof(struct name_list);
	if (len < count * size)
		return -ENOMEM;

	read_lock(&dvfm_trace_list.lock);
	list_for_each_entry(entry, &dvfm_trace_list.list, list) {
		p->id = entry->index;
		strcpy(p->name, entry->name);
		p++;
	}
	read_unlock(&dvfm_trace_list.lock);
	return 0;
}
EXPORT_SYMBOL(dvfm_query_device_list);

/*
 * Device driver register itself to DVFM before any operation.
 * The number of registered device is limited in 32.
 */
int dvfm_register(char *name, int *id)
{
	struct dvfm_trace_info	*p = NULL;
	int len, idx;

	if (name == NULL)
		return -EINVAL;

	/* device name is stricted in 32 bytes */
	len = strlen(name);
	if (len > DVFM_MAX_NAME)
		len = DVFM_MAX_NAME;
	write_lock(&dvfm_trace_list.lock);
	list_for_each_entry(p, &dvfm_trace_list.list, list) {
		if (!strcmp(name, p->name)) {
			/*
			 * Find device in device trace table
			 * Skip to allocate new ID
			 */
			*id = p->index;
			goto out;
		}
	}
	idx = add_device(name);
	if (idx < 0)
		goto out_num;
	*id = idx;
out:
	write_unlock(&dvfm_trace_list.lock);
	return 0;
out_num:
	write_unlock(&dvfm_trace_list.lock);
	return -EINVAL;
}
EXPORT_SYMBOL(dvfm_register);

/*
 * Release the device and free the device index.
 */
int dvfm_unregister(char *name, int *id)
{
	struct dvfm_trace_info	*p = NULL;
	int len, num, i;

	if (!dvfm_driver || !dvfm_driver->count || (name == NULL))
		return -EINVAL;

	/* device name is stricted in 32 bytes */
	len = strlen(name);
	if (len > DVFM_MAX_NAME)
		len = DVFM_MAX_NAME;

	num = dvfm_driver->count(dvfm_driver->priv, dvfm_op_list);

	write_lock(&dvfm_trace_list.lock);
	if (list_empty(&dvfm_trace_list.list))
		goto out;
	list_for_each_entry(p, &dvfm_trace_list.list, list) {
		if (!strncmp(name, p->name, len)) {
			/* remove all dvfm constraint on this device. */
			for (i = 0; i < num; i++)
				dvfm_enable_op(i, p->index);
			/* clear the device index */
			clear_bit(p->index, dvfm_trace_list.device);
			*id = -1;
			list_del(&p->list);
			kfree(p);
			break;
		}
	}
	write_unlock(&dvfm_trace_list.lock);
	return 0;
out:
	write_unlock(&dvfm_trace_list.lock);
	return -ENOENT;
}
EXPORT_SYMBOL(dvfm_unregister);

static int dvfm_add(struct sys_device *sys_dev)
{
	int i, n;
	int ret;

	n = ARRAY_SIZE(dvfm_attr);
	for (i = 0; i < n; i++) {
		ret = sysfs_create_file(&(sys_dev->kobj), dvfm_attr[i]);
		if (ret)
			return -EIO;
	}
	return 0;
}

static int dvfm_rm(struct sys_device *sys_dev)
{
	int i, n;
	n = ARRAY_SIZE(dvfm_attr);
	for (i = 0; i < n; i++) {
		sysfs_remove_file(&(sys_dev->kobj), dvfm_attr[i]);
	}
	return 0;
}

static int dvfm_suspend(struct sys_device *sysdev, pm_message_t pmsg)
{
	return 0;
}

static int dvfm_resume(struct sys_device *sysdev)
{
	return 0;
}

static struct sysdev_driver dvfm_sysdev_driver = {
	.add		= dvfm_add,
	.remove		= dvfm_rm,
	.suspend	= dvfm_suspend,
	.resume		= dvfm_resume,
};

int dvfm_register_driver(struct dvfm_driver *driver_data, struct info_head *op_list)
{
	int ret;
	if (!driver_data || !driver_data->set)
		return -EINVAL;
	if (dvfm_driver)
		return -EBUSY;
	dvfm_driver = driver_data;

	if (!op_list)
		return -EINVAL;
	dvfm_op_list = op_list;

	/* enable_op need to invoke dvfm operation */
	dvfm_register("User", &dvfm_dev_idx);
	ret = sysdev_driver_register(&cpu_sysdev_class, &dvfm_sysdev_driver);
	return ret;
}

int dvfm_unregister_driver(struct dvfm_driver *driver)
{
	sysdev_driver_unregister(&cpu_sysdev_class, &dvfm_sysdev_driver);
	dvfm_unregister("User", &dvfm_dev_idx);
	dvfm_driver = NULL;
	return 0;
}

MODULE_DESCRIPTION("Basic DVFM support for Monahans");
MODULE_LICENSE("GPL");

