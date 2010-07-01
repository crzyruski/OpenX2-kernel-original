/*
 * linux/sound/soc/pxa/pxa3xx-ac97codec-pm.c
 *
 * Copyright (C) 2007 Marvell International Ltd.
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
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <linux/mutex.h>
#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/pxa-regs.h>
#include <mach/audio.h>
#include <mach/mfp.h>
#include <mach/mfp-pxa300.h>
#include <mach/regs-ssp.h>
#include <asm/mach-types.h>
#include <linux/clk.h>
#include "pxa3xx-pcm.h"
#include "pxa3xx-ac97.h"
#include "pxa3xx-ac97codec-pm.h"


/*
#define PXA3XX_AC97CODEC_DEBUG 1
*/
#ifdef PXA3XX_AC97CODEC_DEBUG
#define dbg(format, arg...) \
		printk(KERN_INFO "<FUNC:%s Line:(%d)>" format,\
			__func__, __LINE__, ##arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif

/* 1 second interval */
#define LP_TIME_INTERVAL	100

#ifdef CONFIG_AC97_EXTCLK

#define	enable_extclk()		do {				\
					enable_oscc_pout();	\
				} while (0)
#define disable_extclk()	do {				\
					disable_oscc_pout();	\
				} while (0)
#else
#define enable_extclk()		do {} while (0)
#define disable_extclk()	do {} while (0)
#endif

struct codec_info_node {
	int			id;	/* node id */
	int			count;	/* reference count */
	codec_sub_state_t	state;	/* node state */
	struct list_head	list;
};

struct codec_info_t {
	int			count;
	codec_state_t		codec_state;
	struct timer_list	*timer;
	struct list_head	list;
	spinlock_t		codec_lock;
};


static u16 wm9713_reg[] = {
	0x6174, 0x8080, 0x8080, 0x8080, /* 6 */
	0xc880, 0xe808, 0xe808, 0x0808, /* e */
	0x00da, 0x8000, 0xd600, 0xaaa0, /* 16 */
	0xaaa0, 0xaaa0, 0x0000, 0x0000, /* 1e */
	0x0f0f, 0x0040, 0x0000, 0x7f00, /* 26 */
	0x0405, 0x0410, 0xbb80, 0xbb80, /* 2e */
	0x0000, 0xbb80, 0x0000, 0x4523, /* 36 */
	0x0000, 0x2000, 0x7eff, 0xffff, /* 3e */
	0x0000, 0x0000, 0x0080, 0x0000, /* 46 */
	0x0000, 0x0000, 0xfffe, 0xffff, /* 4e */
	0x0000, 0x0000, 0x0000, 0xfffe, /* 56 */
	0x4000, 0x0000, 0x0000, 0x0000, /* 5e */
	0xb032, 0x3e00, 0x0000, 0x0000, /* 66 */
	0x0000, 0x0000, 0x0000, 0x0000, /* 6e */
	0x0000, 0x0000, 0x0000, 0x0006, /* 76 */
	0x0001, 0x0000, 0x574d, 0x4c13, /* 7e */
};

struct clk *ac97_clk;

int codec_client = -1;
static struct codec_info_t codec_info;
static struct snd_ac97_bus pxa3xx_ac97_bus = {
	.ops = &soc_ac97_ops,
};

static struct snd_ac97 pxa3xx_ac97 = {
	.num = 0, 		/*the primary codec*/
	.bus = &pxa3xx_ac97_bus,
};


static unsigned int ac97_read(unsigned int reg)
{
	return soc_ac97_ops.read(&pxa3xx_ac97, reg);
}

static void ac97_write(unsigned int reg, unsigned int val)
{
	soc_ac97_ops.write(&pxa3xx_ac97, reg, val);
}

static void codec_warm_reset(void)
{
	pxa3xx_ac97.bus->ops->warm_reset(&pxa3xx_ac97);
}

static void codec_cold_reset(void)
{
	pxa3xx_ac97.bus->ops->reset(&pxa3xx_ac97);

	if (cpu_is_pxa300() || cpu_is_pxa310()) {
		if (ac97_read(AC97_RESET) != 0x6174) {
			pxa3xx_mfp_set_afds(MFP_PIN_GPIO17,
				MFP_AF3, MFP_DS03X);
			pxa3xx_mfp_set_afds(MFP_PIN_GPIO25,
				MFP_AF0, MFP_DS01X);
			pxa3xx_ac97.bus->ops->reset(&pxa3xx_ac97);
		}
		if (ac97_read(AC97_RESET) != 0x6174) {
			show_stack(NULL, NULL);
			panic("Codec cold reset error!!\n");
		}
	}
}

static void codec_specific_init(void)
{
	GCR |= GCR_WARM_RST;
	mdelay(100);
	ac97_write(0x26, 0x0);
	ac97_write(0x3c, 0xda00);
	ac97_write(0x3e, 0xb9f5);
	ac97_write(0x22, 0xc440);
	ac97_write(0x12, 0x0);
	ac97_write(0x2, 0x0);
	ac97_write(0x4, 0x0);
	ac97_write(0xc, 0x0808);
	ac97_write(0x1c, 0x9ba8);
	ac97_write(0x2a, 0x1);
	ac97_write(0x2c, 0xac44);
	ac97_write(0x32, 0x3e80);
	ac97_write(0x44, 0x82);
}

static void codec_specific_deinit(void)
{	/* do later: shut down all power */
	unsigned short value = 0;

	/* close the power of all units */
	ac97_write(AC97_EXTENDED_MID, 0xffff);
	ac97_write(AC97_EXTENDED_MSTATUS, 0xffff);
	value = ac97_read(AC97_EXTENDED_MID);
	value &= ~(0x1 << 10);
	ac97_write(AC97_EXTENDED_MID, value);
}


void codec_save(void)
{
	unsigned int i;

	for (i = 2; i < ARRAY_SIZE(wm9713_reg) << 1; i += 2)
		wm9713_reg[i>>1] = ac97_read(i);
}

void codec_restore(void)
{
	unsigned int i;

	codec_cold_reset();
	codec_warm_reset();

	ac97_write(AC97_POWERDOWN, 0);
	for (i = 2; i < (ARRAY_SIZE(wm9713_reg)-3) << 1; i += 2) {
		if (AC97_LINE1_LEVEL == i || AC97_GPIO_STATUS == i ||
			AC97_POWERDOWN == i) {
			continue;
		} else if (AC97_HANDSET_RATE == i) {
			continue;
		} else {
			ac97_write(i, wm9713_reg[i>>1]);
			/*
			printk("<restore>: 0x%x <== 0x%x, <readback>: 0x%x\n",
					i, wm9713_reg[i>>1], ac97_read(i));
			*/
		}
	}
	ac97_write(AC97_POWERDOWN, 0);
	dbg("<Restore>: readback ID = 0x%x\n", ac97_read(0));
}

/*
 * This function is used to change state of audio codec.
 */
void set_codec_state(codec_state_t state)
{
	struct codec_info_t	*info = &codec_info;

	dbg("$$$Power State Change$$$$<%s> ==>> <%s>\n",
		info->codec_state == CODEC_POWER_ON?"CODEC_POWER_ON":
		info->codec_state == CODEC_LOWPOWER?"CODEC_LOWPOWER":
		info->codec_state == CODEC_POWER_OFF?"CODEC_POWER_OFF":
		info->codec_state == CODEC_READY_LOWPOWER?
			"CODEC_READY_LOWPOWER":"UNKNOWN",
		state == CODEC_POWER_ON?"CODEC_POWER_ON":
		state == CODEC_LOWPOWER?"CODEC_LOWPOWER":
		state == CODEC_POWER_OFF?"CODEC_POWER_OFF":
		state == CODEC_READY_LOWPOWER?"CODEC_READY_LOWPOWER":"UNKNOWN");

	if (info->codec_state == CODEC_POWER_OFF) {
		switch (state) {
		case CODEC_POWER_ON:
			info->codec_state = state;
			/* init codec */
			enable_extclk();
			clk_enable(ac97_clk);
			codec_restore();
			break;
		default:
			/* invalid state */
			goto invalid_state;
		}
	} else if (info->codec_state == CODEC_POWER_ON) {
		switch (state) {
		case CODEC_READY_LOWPOWER:
			/* startup timer */
			codec_info.timer->expires = jiffies + LP_TIME_INTERVAL;
			add_timer(codec_info.timer);
			dbg("started timer\n");
			info->codec_state = state;
			break;
		case CODEC_POWER_ON:
			break;
		case CODEC_POWER_OFF:
			/* Only state will be set */
			info->codec_state = state;
			codec_save();
			clk_disable(ac97_clk);
			disable_extclk();
			break;
		default:
			/* invalid state */
			goto invalid_state;
		}
	} else if (info->codec_state == CODEC_READY_LOWPOWER) {
		switch (state) {
		case CODEC_POWER_ON:
			/* stop timer */
			info->codec_state = state;
			del_timer_sync(codec_info.timer);
			break;
		case CODEC_LOWPOWER:
			/* timer is already stopped */
			info->codec_state = state;
			break;
		case CODEC_POWER_OFF:
			/* stop timer */
			del_timer_sync(codec_info.timer);
			/* Only state will be set */
			info->codec_state = state;
			codec_save();
			clk_disable(ac97_clk);
			disable_extclk();
			break;
		default:
			/* invalid state */
			goto invalid_state;
		}
	} else if (info->codec_state == CODEC_LOWPOWER) {
		switch (state) {
		case CODEC_POWER_ON:
			info->codec_state = state;
			break;
		case CODEC_POWER_OFF:
			/* Only state will be set */
			info->codec_state = state;
			break;
		default:
			/* invalid state */
			goto invalid_state;
		}
	}
	dbg("%s: state is %d\n", __FUNCTION__, info->codec_state);
	return;
invalid_state:
	printk(KERN_ERR "%s: current state is %d, next invalid state is %d\n",
			__FUNCTION__, info->codec_state, state);
}

static void codec_lp_timer_handler(unsigned long unused)
{
	set_codec_state(CODEC_LOWPOWER);
}

static int calc_codec_state(void)
{
	struct codec_info_node *p = NULL;
	struct list_head *q = NULL;
	int codec_state = CODEC_SUB_POWER_OFF;

	spin_lock(&codec_info.codec_lock);
	if (!list_empty(&codec_info.list)) {
		list_for_each(q, &codec_info.list) {
			p = list_entry(q, struct codec_info_node, list);
			dbg("%s: node %d, state %d\n", __FUNCTION__, p->id,
				p->state);
			if (p->state == CODEC_SUB_POWER_ON) {
				/* Even one node is POWER_ON,
				 * codec can't be shutdown */
				codec_state = CODEC_POWER_ON;
				break;
			} else if (p->state == CODEC_SUB_LOWPOWER) {
				codec_state = CODEC_POWER_ON;
			}
		}
	}
	spin_unlock(&codec_info.codec_lock);
	dbg("%s: codec state:%d\n", __FUNCTION__, codec_state);
	return codec_state;
}

int set_codec_sub_state(int client, int state)
{
	struct codec_info_node *p = NULL;
	struct list_head *q = NULL;
	int codec_state = CODEC_SUB_POWER_ON;

	dbg("%s entry, client:%d, state:%d\n", __FUNCTION__, client, state);

	spin_lock(&codec_info.codec_lock);
	/* find the client */
	if (!list_empty(&codec_info.list)) {
		list_for_each(q, &codec_info.list) {
			p = list_entry(q, struct codec_info_node, list);
			if (client == p->id) {
				dbg("Found node as id %d\n", client);
				break;
			}
		}
		if (client != p->id) {
			spin_unlock(&codec_info.codec_lock);
			printk(KERN_ERR "No such node exists\n");
			return -EIO;
		}
	} else {
		spin_unlock(&codec_info.codec_lock);
		printk(KERN_ERR "No such node exists\n");
		return -EIO;
	}

	/* When reference count isn't zero, the device is in use.
	 * It can't be set as low power mode or power off mode.
	 */
	if (state == CODEC_SUB_POWER_ON) {
		p->count++;
		p->state = state;
	} else if ((state == CODEC_SUB_LOWPOWER) ||
			(state == CODEC_SUB_POWER_OFF)) {
		if (--p->count == 0)
			p->state = state;
	}
	if (p->count < 0)
		panic("Set state error!!\n");
	codec_state = calc_codec_state();
	dbg("Set client %d as state %d, count:%d\n", client, codec_state,
		p->count);
	set_codec_state(codec_state);
	spin_unlock(&codec_info.codec_lock);
	return 0;
}
EXPORT_SYMBOL(set_codec_sub_state);

/*
 * Initialize the state of codec.
 */
int init_codec_state(void)
{
	codec_info.timer = kmalloc(sizeof(struct timer_list), GFP_KERNEL);
	if (codec_info.timer == NULL) {
		printk(KERN_ERR "Can't allocate memory for codec state\n");
		return -ENOMEM;
	}

	init_timer(codec_info.timer);
	codec_info.timer->function = codec_lp_timer_handler;
	codec_info.timer->data = 0;

	INIT_LIST_HEAD(&codec_info.list);
	spin_lock_init(&codec_info.codec_lock);

	codec_info.codec_state = CODEC_POWER_OFF;

	return 0;
}

static void extra_reset(void)
{
	if (cpu_is_pxa300() || cpu_is_pxa310()) {
		pxa3xx_mfp_set_afds(MFP_PIN_GPIO17,
				MFP_AF0, MFP_DS03X);
		codec_cold_reset();
		codec_warm_reset();
		if (ac97_read(AC97_RESET) != 0x6174) {
			pxa3xx_mfp_set_afds(MFP_PIN_GPIO17,
					MFP_AF3, MFP_DS03X);
			pxa3xx_mfp_set_afds(MFP_PIN_GPIO25,
					MFP_AF0, MFP_DS01X);
			codec_cold_reset();
			codec_warm_reset();
		}
	}
	if (cpu_is_pxa320()) {
		codec_cold_reset();
		codec_warm_reset();
	}
}

int register_codec(int *client)
{
	struct codec_info_node	*p = NULL;
	struct list_head *q = NULL;
	int id = 0;

	dbg("%s: entry\n", __FUNCTION__);

	ac97_clk = clk_get(NULL, "AC97CLK");

	if (codec_info.count == 0) {
		clk_enable(ac97_clk);
		enable_extclk();

		extra_reset();

		ac97_write(AC97_POWERDOWN, 0);
#ifdef CONFIG_AC97_EXTCLK
		ac97_write(AC97_HANDSET_RATE, 0x0082);
#endif
		codec_save();
		init_codec_state();
	}
	codec_info.count++;

	spin_lock(&codec_info.codec_lock);
	if (!list_empty(&codec_info.list)) {
		list_for_each(q, &(codec_info.list)) {
			p = list_entry(q, struct codec_info_node, list);
			if (p->id > id)
				id = p->id;
		}
	}
	id++;
	dbg("%s node id:%d\n", __FUNCTION__, id);

	p = (struct codec_info_node *)vmalloc(sizeof(struct codec_info_node));
	if (p == NULL) {
		printk(KERN_ERR "%s: Can't allocate memory\n", __FUNCTION__);
		return -ENOMEM;
	}
	p->count = 0;
	p->id = id;

	list_add(&(p->list), &(codec_info.list));
	spin_unlock(&codec_info.codec_lock);

	*client = id;
	return 0;
}
EXPORT_SYMBOL(register_codec);

int unregister_codec(int client)
{
	struct codec_info_node	*p = NULL;
	struct list_head *q = NULL;

	dbg("%s: entry\n", __FUNCTION__);
	spin_lock(&codec_info.codec_lock);
	if (list_empty(&codec_info.list)) {
		printk(KERN_ERR "No node is registered for codec\n");
		return -EIO;
	}
	list_for_each(q, &codec_info.list) {
		p = list_entry(q, struct codec_info_node, list);
		if (client == p->id) {
			list_del(q);
			vfree(p);
		}
	}
	spin_unlock(&codec_info.codec_lock);
	return 0;
}
EXPORT_SYMBOL(unregister_codec);

static ssize_t wm9713_proc_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	char			*buf = page;
	char			*next = buf;
	unsigned		size = count;
	int 			t;
	int 			i;
	int 			reg;

	t = scnprintf(next, size, "AC97 regs: GCR: 0x%x, GSR: 0x%x\n",
			GCR, GSR);
	size -= t;
	next += t;

	t = scnprintf(next, size, "wm9713 regs: \n");
	size -= t;
	next += t;

	for (i = 0; i < 0x80; i += 8) {
		reg = ac97_read(i);
		t = scnprintf(next, size, "[0x%02x]=0x%04x  ", i, reg);
		size -= t;
		next += t;
		reg = ac97_read(i+2);
		t = scnprintf(next, size, "[0x%02x]=0x%04x  ", i+2, reg);
		size -= t;
		next += t;
		reg = ac97_read(i+4);
		t = scnprintf(next, size, "[0x%02x]=0x%04x  ", i+4, reg);
		size -= t;
		next += t;
		reg = ac97_read(i+6);
		t = scnprintf(next, size, "[0x%02x]=0x%04x  \n", i+6, reg);
		size -= t;
		next += t;
	}

	*eof = 1;
	return count - size;
}

static int wm9713_proc_write(struct file *file, const char __user *buffer,
			   unsigned long count, void *data)
{
	static char kbuf[4096];
	char *buf = kbuf;
	unsigned int i, reg, reg2;
	char cmd;

	if (count >= 4096)
		return -EINVAL;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	sscanf(buf, "%c 0x%x 0x%x", &cmd, &i, &reg);
	if ('r' == cmd) {
		if (i > 0x7e || (i&1)) {
			printk(KERN_INFO "invalid index!\n");
			goto error;
		}
		reg = ac97_read(i);
		printk(KERN_INFO "0x[%2x]=0x%4x\n", i, reg);
	} else if ('w' == cmd) {
		if (i > 0x7e || (i&1)) {
			printk(KERN_INFO "invalid index!\n");
			goto error;
		}
		if (reg > 0xffff) {
			printk(KERN_INFO "invalid value!\n");
			goto error;
		}
		ac97_write(i, reg);
		reg2 = ac97_read(i);
		printk(KERN_INFO
			"write 0x%4x to 0x[%2x], read back 0x%4x\n",
			reg, i, reg2);
	} else if ('W' == cmd) {
		__REG(i) = reg;
		printk(KERN_INFO
			"Write 0x%x to 0x%x, read back 0x%x\n",
			reg, i, __REG(i));
	} else if ('u' == cmd) {
		clk_enable(ac97_clk);
		enable_extclk();
		printk(KERN_INFO "Enable Clock\n");
	} else if ('d' == cmd) {
		clk_disable(ac97_clk);
		disable_extclk();
		printk(KERN_INFO "Disable Clock\n");
	} else if ('g' == cmd) {
		printk(KERN_INFO "GPIO registers (base = 0x40E00000)\n");
		reg = 0x40E00000;
		while (reg <= 0x40E004AC) {
			printk(KERN_INFO "reg[0x%x] = 0x%x\n", reg, __REG(reg));
			reg += 4;
		}
	} else if ('i' == cmd) {
		printk(KERN_INFO "Interrupt registers (base = 0x40D00000)\n");
		reg = 0x40D00000;
		while (reg < 0x40D00108) {
			printk(KERN_INFO "reg[0x%x] = 0x%x\n", reg, __REG(reg));
			reg += 4;
		}
	} else {
		printk(KERN_INFO "unknow opt!\n");
		goto error;
	}

	return count;
error:
	printk(KERN_INFO "r/w index(0x%%2x) value(0x%%4x)\n");
	return count;
}

static int __init ac97codec_pm_init(void)
{
	struct proc_dir_entry *wm9713_proc_entry;
	
	if (!machine_is_zylonite())
		return 0;

	wm9713_proc_entry = create_proc_entry("driver/codec", 0, NULL);
	if (wm9713_proc_entry) {
		wm9713_proc_entry->read_proc = wm9713_proc_read;
		wm9713_proc_entry->write_proc = wm9713_proc_write;
	}

	return 0;
}

static void __exit ac97codec_pm_exit(void)
{
}

module_init(ac97codec_pm_init);
module_exit(ac97codec_pm_exit);

/* Module information */
MODULE_AUTHOR("bin.yang@marvell.com");
MODULE_DESCRIPTION("AC97 Codec PM");
MODULE_LICENSE("GPL");



