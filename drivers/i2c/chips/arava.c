/*
 * PXA3xx ARAVA PMIC Management Routines
 *
 *
 * Copyright (C) 2004, Intel Corporation(chao.xie@intel.com).
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#undef DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <asm/ioctl.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <asm/uaccess.h>
#include <mach/pxa-regs.h>
#include <mach/pxa3xx-regs.h>
#include <mach/arava.h>
#include <mach/pxa3xx_pmic.h>

static struct pxa3xx_pmic_regs arava_regs[ARAVA_REG_NUM];

/* Unique ID allocation */
static struct i2c_client *g_client;

int arava_read(u8 reg, u8 *pval)
{
	int ret;

	if (g_client == NULL)	/* No global client pointer? */
		return -EINVAL;

	pr_debug("hit %d\n", arava_regs[reg].hit);
	if (arava_regs[reg].hit) {
		*pval = arava_regs[reg].data;
		return 0;
	}

	ret = i2c_smbus_read_byte_data(g_client, reg);
	pr_debug("i2c read ret:0x%x\n", ret);
	if (ret >= 0) {
		*pval = ret;
		/* If a register is uncacheable, the hit field can't be set */
		arava_regs[reg].hit = ~arava_regs[reg].mask;
		arava_regs[reg].data = ret;
		ret = 0;
	} else
		ret = -EIO;

	return ret;
}

int arava_write(u8 reg, u8 val)
{
	int ret;

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	ret = i2c_smbus_write_byte_data(g_client, reg, val);
	if (ret == 0) {
		/* If a register is uncacheable, the hit field can't be set */
		arava_regs[reg].hit = ~arava_regs[reg].mask;
		/* arava_regs[reg].hit = 0; */
		arava_regs[reg].data = val;
		ret = 0;
	} else
		ret = -EIO;

	return ret;
}

static int arava_set_pump(int enable)
{
	int ret;
	u8 val;
	unsigned long flags;

	local_irq_save(flags);
	if (enable) {
		ret = arava_read(ARAVA_MISCB, &val);
		if (ret)
			goto out;
		val |= ARAVA_MISCB_SESSION_VALID_EN;
		val &= ~ARAVA_MISCB_USBINT_BOTHEDGE;
		ret = arava_write(ARAVA_MISCB, val);
		if (ret)
			goto out;

		/* FIXME: We use EXTON as cable detect signal on ZYlonite.
		 * This depends on the cable signal is connected to EXTON
		 * of ARAVA.
		 * If the cable signal is not connected to EXTON, we need
		 * use other signal as cable detect signal.
		 */
		ret = arava_read(ARAVA_IRQ_MASK_A, &val);
		if (ret)
			goto out;
		val &= ~ARAVA_IRQMASK_A_EXTON;
		ret = arava_write(ARAVA_IRQ_MASK_A, val);
		if (ret)
			goto out;
		val = ARAVA_USBPUMP_EN_USBVE | ARAVA_USBPUMP_EN_USBVEP;
		ret = arava_write(ARAVA_USBPUMP, val);
	} else {
		ret = arava_read(ARAVA_MISCB, &val);
		if (ret)
			goto out;
		val &= ~(ARAVA_MISCB_SESSION_VALID_EN |
				ARAVA_MISCB_USBINT_BOTHEDGE);
		ret = arava_write(ARAVA_MISCB, val);
		if (ret)
			goto out;
		ret = arava_read(ARAVA_IRQ_MASK_A, &val);
		if (ret)
			goto out;
		val |= ARAVA_IRQMASK_A_EXTON;
		ret = arava_write(ARAVA_IRQ_MASK_A, val);
		if (ret)
			goto out;
		ret = arava_write(ARAVA_USBPUMP, 0);
	}
out:
	local_irq_restore(flags);
	return ret;
}

static int arava_set_vbus_supply(int enable, int srp)
{
	int ret;
	u8 val;

	ret = arava_read(ARAVA_USBPUMP, &val);
	if (ret)
		goto out;

	if (enable) {
		val |= ARAVA_USBPUMP_USBVE | ARAVA_USBPUMP_EN_USBVE | \
			ARAVA_USBPUMP_EN_USBVEP;
		if (srp)
			val |= ARAVA_USBPUMP_USBVEP;
		else
			val &= ~ARAVA_USBPUMP_USBVEP;
	} else {
		val |= ARAVA_USBPUMP_EN_USBVE | ARAVA_USBPUMP_EN_USBVEP;
		val &= ~(ARAVA_USBPUMP_USBVE | ARAVA_USBPUMP_USBVEP);
	}
	ret = arava_write(ARAVA_USBPUMP, val);
	pr_debug("%s enable %d srp %d val %x\n", __func__, enable, srp, val);

out:
	if (ret)
		printk(KERN_ALERT "i2c operation error %d\n", ret);
	return ret;
}

/* Set USB A-device events: VBUS/Session valid.
 * For ARAVA, only SRP_DETECT event occurs when peer side B-device send
 * VBUS pulse on the bus. Use SRP_DETECT event as A-device session valid
 * event, which meet the Min value of the threshold.
 */
static int arava_set_usbotg_a_mask(void)
{
	int ret;
	u8 val;

	ret = arava_read(ARAVA_IRQ_MASK_B, &val);
	if (ret)
		return ret;

	/* set interrupts that a device care about */
	val |= (ARAVA_EVENT_B_VBUS_4P0 | ARAVA_EVENT_B_VBUS_4P4 | \
		ARAVA_EVENT_B_SESSION_VALID | ARAVA_EVENT_B_SRP_DETECT);
	val &= ~(ARAVA_EVENT_B_SRP_DETECT | ARAVA_EVENT_B_VBUS_4P4);

	ret = arava_write(ARAVA_IRQ_MASK_B, val);
	return ret;
}

/* Set USB B-device events; Session valid/end
 * Current USB driver doesn't care about any B-device evnets.
 */
static int arava_set_usbotg_b_mask(void)
{
	int ret;
	u8 val;

	ret = arava_read(ARAVA_IRQ_MASK_B, &val);
	if (ret)
		return ret;

	/* set interrupts that b device care about */
	val |= (ARAVA_EVENT_B_VBUS_4P0 | ARAVA_EVENT_B_VBUS_4P4 | \
		ARAVA_EVENT_B_SESSION_VALID | ARAVA_EVENT_B_SRP_DETECT);

	ret = arava_write(ARAVA_IRQ_MASK_B, val);
	return ret;
}

static int is_arava_vbus_assert(void)
{
	u8 val;

	/* FIXME: We use EXTON as cable detect signal on ZYlonite.
	 * This depends on the cable signal is connected to EXTON
	 * of ARAVA.
	 * If the cable signal is not connected to EXTON, we need
	 * use other signal as cable detect signal.
	 */
	arava_read(ARAVA_STATUS, &val);
	if (val & ARAVA_STATUS_EXTON)
		return 1;
	return 0;
}

static unsigned long arava_event_change(void)
{
	int ret = 0;
	u8 val;

	arava_read(ARAVA_EVENT_A, &val);

	/* FIXME: We use EXTON as cable detect signal on ZYlonite.
	 * This depends on the cable signal is connected to EXTON
	 * of ARAVA.
	 * If the cable signal is not connected to EXTON, we need
	 * use other signal as cable detect signal.
	 */
	if (val & ARAVA_EVENT_A_EXTON)
		ret |= PMIC_EVENT_VBUS;

	arava_read(ARAVA_EVENT_B, &val);
	if (val & ARAVA_EVENT_B_SRP_DETECT) {
		arava_read(ARAVA_IRQ_MASK_B, &val);
		if (!(val & ARAVA_EVENT_B_SRP_DETECT))
			ret |= PMIC_EVENT_VBUS;
	}

	arava_read(ARAVA_EVENT_C, &val);

	return ret;
}

/* FIXME: Because the ARAVA silicon has some issues. So need check whether
 * IRQ is masked or not before check the VBUS state.
 */
static int is_arava_avbusvld(void)
{
	u8 val;

	arava_read(ARAVA_IRQ_MASK_B, &val);
	if (val & ARAVA_EVENT_B_VBUS_4P4)
		return 0;

	arava_read(ARAVA_USBPUMP, &val);

	if (val & ARAVA_USBPUMP_VBUS_VALID_4_4)
		return 1;
	else
		return 0;
}

static int is_arava_asessvld(void)
{
	u8 val;

	arava_read(ARAVA_IRQ_MASK_B, &val);

	if (val & ARAVA_EVENT_B_SESSION_VALID)
		return 0;

	arava_read(ARAVA_USBPUMP, &val);

	if (val & ARAVA_USBPUMP_SESSION_VALID)
		return 1;
	else
		return 0;
}

static int is_arava_bsessvld(void)
{
	u8 val;

	arava_read(ARAVA_IRQ_MASK_B, &val);

	if (val & ARAVA_EVENT_B_VBUS_4P0)
		return 0;

	arava_read(ARAVA_USBPUMP, &val);

	if (val & ARAVA_USBPUMP_VBUS_VALID_4_0)
		return 1;
	else
		return 0;
}

static int is_arava_srp_ready(void)
{
	u8 val;

	/* ARAVA reports unexpected SRP_DETECT event when
	 * VBUS is pulled HIGH/LOW, no matter whether the
	 * event is cared or not. Reture 0 when USB driver
	 * don't want to detect SRP_DETECT event.
	 */
	arava_read(ARAVA_IRQ_MASK_B, &val);

	if (val & ARAVA_EVENT_B_SRP_DETECT)
		return 0;

	arava_read(ARAVA_USBPUMP, &val);

	if (val & ARAVA_USBPUMP_SRP_DETECT)
		return 1;
	else
		return 0;
}

static int get_arava_voltage(int cmd, int *pmv)
{
	int ret;
	u8 val;

	*pmv = 0;
	start_calc_time();
	switch (cmd) {
	case VCC_CORE:
		ret = arava_read(ARAVA_BUCK2DVC1, &val);
		break;
	case VCC_SRAM:
		ret = arava_read(ARAVA_LDO1416, &val);
		break;
	case VCC_MVT:
		ret = arava_read(ARAVA_LDO1819, &val);
		break;
	case VCC_MEM:
		ret = arava_read(ARAVA_LDO1011, &val);
		break;
	case VCC_3V_APPS:
	case VCC_USB:
	case VCC_TSI:
	case VCC_LCD:
	case VCC_CAMERA_IO:
		ret = arava_read(ARAVA_LDO1819, &val);
		break;
	case VCC_SDIO:
		ret = arava_read(ARAVA_LDO1011, &val);
		break;
	case VCC_CAMERA_ANA:
		ret = arava_read(ARAVA_LDO17_SIMCP0, &val);
		break;
	default:
		printk(KERN_WARNING "input wrong command: %d\n", cmd);
		return -EINVAL;
	}

	if (ret != 0)
		return ret;

	switch (cmd) {
	case VCC_CORE:
		val &= 0x1f;
		*pmv = val * ARAVA_VBUCK2STEP + ARAVA_VBUCK2BASE;
		break;
	case VCC_SRAM:
		val = val >> 3;
		*pmv = val * ARAVA_VLDO16STEP + ARAVA_VLDO16BASE;
		break;
	case VCC_MVT:
		/* LDO19 is similar to LDO10 */
		val = (val >> 4) & 0x0f;
		*pmv = val * ARAVA_VLDO10STEP + ARAVA_VLDO10BASE;
		break;
	case VCC_MEM:
		val &= 0x0f;
		*pmv = val * ARAVA_VLDO10STEP + ARAVA_VLDO10BASE;
		break;
	case VCC_3V_APPS:
	case VCC_USB:
	case VCC_TSI:
	case VCC_LCD:
	case VCC_CAMERA_IO:
		/* LDO18 is similar to LDO10 */
		val &= 0x0f;
		*pmv = val * ARAVA_VLDO10STEP + ARAVA_VLDO10BASE;
		break;
	case VCC_SDIO:
		/* LDO11 is similar to LDO10 */
		val = (val >> 4) & 0x0f;
		*pmv = val * ARAVA_VLDO10STEP + ARAVA_VLDO10BASE;
		break;
	case VCC_CAMERA_ANA:
		/* LDO17 is similar to LDO10 */
		val &= 0x0f;
		*pmv = val * ARAVA_VLDO10STEP + ARAVA_VLDO10BASE;
		break;
	default:
		break;
	}
	end_calc_time();
	return ret;
}

int set_arava_voltage(int cmd, int mv)
{
	int ret;
	u8 val;

	start_calc_time();
	switch (cmd) {
	case VCC_CORE:
		if (mv < ARAVA_VBUCK2BASE && mv > ARAVA_VBUCK2MAX)
			return -EINVAL;
		ret = arava_read(ARAVA_BUCK2DVC1, &val);
		break;
	case VCC_SRAM:
		if (mv < ARAVA_VLDO16BASE && mv > ARAVA_VLDO16MAX)
			return -EINVAL;
		ret = arava_read(ARAVA_LDO1416, &val);
		break;
	case VCC_MVT:
		if (mv < ARAVA_VLDO10BASE && mv > ARAVA_VLDO10MAX)
			return -EINVAL;
		ret = arava_read(ARAVA_LDO1819, &val);
		break;
	case VCC_MEM:
		if (mv < ARAVA_VLDO10BASE && mv > ARAVA_VLDO10MAX)
			return -EINVAL;
		ret = arava_read(ARAVA_LDO1011, &val);
		break;
	case VCC_3V_APPS:
	case VCC_USB:
	case VCC_TSI:
	case VCC_LCD:
	case VCC_CAMERA_IO:
		if (mv < ARAVA_VLDO10BASE && mv > ARAVA_VLDO10MAX)
			return -EINVAL;
		ret = arava_read(ARAVA_LDO1819, &val);
		break;
	case VCC_SDIO:
		if (mv < ARAVA_VLDO10BASE && mv > ARAVA_VLDO10MAX)
			return -EINVAL;
		ret = arava_read(ARAVA_LDO1011, &val);
		break;
	case VCC_CAMERA_ANA:
		if (mv < ARAVA_VLDO10BASE && mv > ARAVA_VLDO10MAX)
			return -EINVAL;
		ret = arava_read(ARAVA_LDO17_SIMCP0, &val);
		break;
	default:
		printk(KERN_INFO "error command\n");
		return -EINVAL;
	}

	if (ret != 0)
		return ret;

	switch (cmd) {
	case VCC_CORE:
		val &= 0xe0;
		val |= ((mv - ARAVA_VBUCK2BASE) / ARAVA_VBUCK2STEP) | 0x80;
		ret = arava_write(ARAVA_BUCK2DVC1, val);
		break;
	case VCC_SRAM:
		val &= 0x07;
		val |= ((mv - ARAVA_VLDO16BASE) / ARAVA_VLDO16STEP) << 3;
		ret = arava_write(ARAVA_LDO1416, val);
		break;
	case VCC_MVT:
		val &= 0x0f;
		val |= ((mv - ARAVA_VLDO10BASE) / ARAVA_VLDO10STEP) << 4;
		ret = arava_write(ARAVA_LDO1819, val);
		break;
	case VCC_MEM:
		val &= 0xf0;
		val |= (mv - ARAVA_VLDO10BASE) / ARAVA_VLDO10STEP;
		ret = arava_write(ARAVA_LDO1011, val);
		break;
	case VCC_3V_APPS:
	case VCC_USB:
	case VCC_TSI:
	case VCC_LCD:
	case VCC_CAMERA_IO:
		val &= 0xf0;
		val |= (mv - ARAVA_VLDO10BASE) / ARAVA_VLDO10STEP;
		ret = arava_write(ARAVA_LDO1819, val);
		break;
	case VCC_SDIO:
		val &= 0x0f;
		val |= ((mv - ARAVA_VLDO10BASE) / ARAVA_VLDO10STEP) << 4;
		ret = arava_write(ARAVA_LDO1011, val);
		break;
	case VCC_CAMERA_ANA:
		val &= 0xf0;
		val |= (mv - ARAVA_VLDO10BASE) / ARAVA_VLDO10STEP;
		ret = arava_write(ARAVA_LDO17_SIMCP0, val);
		break;
	}
	end_calc_time();
	return ret;
}

static int arava_initchip(void)
{
	int i;

	memset(&arava_regs, 0, sizeof(struct pxa3xx_pmic_regs) * ARAVA_REG_NUM);

	/* set these registers as uncacheable */
	for (i = 0; i < 0x10; i++)
		arava_regs[i].mask = 1;
	/* arava_regs[0x15].mask = 1;
	 */
	arava_regs[0x19].mask = 1;
	for (i = 0x1C; i < 0x20; i++)
		arava_regs[i].mask = 1;
	arava_regs[0x24].mask = 1;
	arava_regs[0x27].mask = 1;
	for (i = 0x2C; i < 0x31; i++)
		arava_regs[i].mask = 1;
	for (i = 0x37; i < 0x50; i++)
		arava_regs[i].mask = 1;
	for (i = 0x51; i < 0x80; i++)
		arava_regs[i].mask = 1;
	for (i = 0x82; i < 0x90; i++)
		arava_regs[i].mask = 1;
	for (i = 0x9C; i < 0xA0; i++)
		arava_regs[i].mask = 1;
	for (i = 0xA7; i < ARAVA_REG_NUM; i++)
		arava_regs[i].mask = 1;

	return arava_write(ARAVA_SYSCTRL_A, 0xE8);
}

#ifdef	ARAVA_THREAD
static DECLARE_WAIT_QEUE_HEAD(arava_event_wait);
static int arava_event_handler()
{
	unsigned long event;

	DECLARE_WAITQUEUE(wait, current);
	current->flags |= PF_MEMALLOC | PF_NOFREEZE;
	daemonize("arava event");

	while (1) {
		disable_irq(IRQ_ARAVA_EVENT);

		event = arava_event_change();
		pmic_event_handle(event);

		add_wait_queue(&arava_event_wait, &wait);
		set_current_state(TASK_INTERRUPTIBLE);

		enable_irq(IRQ_ARAVA_EVENT);
		schedule();
		set_current_state(TASK_RUNNING);
		remove_wait_queue(&arava_event_wait, &wait);
	}
	return 0;
}
#endif

/*
 * Arava interrupt service routine.
 * In the ISR we need to check the Status bits in Arava and according to those
 * bits to check which kind of IRQ had happened.
 */
static irqreturn_t arava_irq_handler(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct arava_platform_data *pdata = client->dev.platform_data;
	unsigned long event;

	/* clear the irq */
	pdata->ack_irq();

#ifdef	ARAVA_THREAD
	wake_up_ionterruptible(&arava_event_wait);
#else
	event = arava_event_change();
	pmic_event_handle(event);
#endif

	return IRQ_HANDLED;
}

#ifdef CONFIG_PM
/*
 * Suspend the arava interface.
 */
static int arava_suspend(struct i2c_client *client, pm_message_t state)
{
	pr_info("arava: arava suspend\n");
	disable_irq(client->irq);

	return 0;
}


/*
 * Resume the arava interface.
 */
static int arava_resume(struct i2c_client *client)
{
	int i;

	pr_info("arava: arava resume\n");
	/* all registers need to be read again */
	for (i = 0; i < ARAVA_REG_NUM; i++)
		arava_regs[i].hit = 0;

	enable_irq(client->irq);

	return 0;
}

#else				/*  */
#define	arava_suspend		NULL
#define	arava_resume		NULL
#endif				/*  */

static struct pmic_ops arava_pmic_ops = {
	.get_voltage		= get_arava_voltage,
	.set_voltage		= set_arava_voltage,

	.is_vbus_assert 	= is_arava_vbus_assert,
	.is_avbusvld		= is_arava_avbusvld,
	.is_asessvld		= is_arava_asessvld,
	.is_bsessvld		= is_arava_bsessvld,
	.is_srp_ready		= is_arava_srp_ready,

	.set_pump		= arava_set_pump,
	.set_vbus_supply	= arava_set_vbus_supply,
	.set_usbotg_a_mask	= arava_set_usbotg_a_mask,
	.set_usbotg_b_mask	= arava_set_usbotg_b_mask,
};

#ifdef CONFIG_PROC_FS
#define	ARAVA_PROC_FILE	"driver/arava"
static struct proc_dir_entry *arava_proc_file;

static int arava_seq_show(struct seq_file *s, void *p)
{
	u8 val;

	arava_read(ARAVA_STATUS, &val);
	seq_printf(s, "Arava status regs: 0x%02x\n", val);

	arava_read(ARAVA_IRQ_MASK_A, &val);
	seq_printf(s, "Arava event mask A: 0x%02x\n", val);

	arava_read(ARAVA_IRQ_MASK_B, &val);
	seq_printf(s, "Arava event mask B: 0x%02x\n", val);

	arava_read(ARAVA_IRQ_MASK_C, &val);
	seq_printf(s, "Arava event mask C: 0x%02x\n", val);

	arava_read(ARAVA_EVENT_A, &val);
	seq_printf(s, "Arava event A:0x%02x\n", val);

	arava_read(ARAVA_EVENT_B, &val);
	seq_printf(s, "Arava event B:0x%02x\n", val);

	arava_read(ARAVA_EVENT_C, &val);
	seq_printf(s, "Arava event C:0x%02x\n", val);

	arava_read(ARAVA_USBPUMP, &val);
	seq_printf(s, "USB pump:0x%02x\n", val);

	arava_read(ARAVA_MISCB, &val);
	seq_printf(s, "Misc control reg B:0x%02x\n", val);
	return 0;
}

static int arava_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, &arava_seq_show, NULL);
}

static struct file_operations arava_seq_fops = {
	.owner		= THIS_MODULE,
	.open		= arava_seq_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int create_arava_proc_file(void)
{
	arava_proc_file = create_proc_entry(ARAVA_PROC_FILE, 0644, NULL);
	if (!arava_proc_file) {
		printk(KERN_INFO "Create proc file for Arava failed\n");
		return -ENOMEM;
	}

	arava_proc_file->proc_fops = &arava_seq_fops;
	return 0;
}

static void remove_arava_proc_file(void)
{
	remove_proc_entry(ARAVA_PROC_FILE, NULL);
}
#endif

static int arava_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct arava_platform_data *pdata;
	int ret;
	u8 value;
	int chip_id;

	g_client = client;

	chip_id = i2c_smbus_read_byte_data(g_client, ARAVA_CHIP_ID);
	if (chip_id < 0) {
		printk(KERN_ERR "arava unavailable!\n");
		return -ENXIO;
	} else {
		printk(KERN_INFO "arava(chip id:0x%02x) detected.\n", chip_id);
	}

	ret = arava_initchip();
	if (ret != 0)
		printk(KERN_WARNING "Initialize ARAVA failed\n");

	/* init irq */
	pdata = client->dev.platform_data;
	pdata->init_irq();

	ret = request_irq(client->irq, arava_irq_handler,
			IRQF_DISABLED, "Arava" , client);

	if (ret) {
		printk(KERN_WARNING "Request IRQ for ARAVA failed, \
					return :%d\n", ret);
		return ret;
	}

	/* Mask interrupts that are not needed */
	arava_write(ARAVA_IRQ_MASK_A, 0xFF);
	arava_write(ARAVA_IRQ_MASK_B, 0xFF);
	arava_write(ARAVA_IRQ_MASK_C, 0xFF);

	arava_write(ARAVA_REGCTRL1, 0xFF);
	arava_write(ARAVA_REGCTRL2, 0x43);
	/* On old zylonite board, SRAM LDO doesn't work well.
	 * We have to set ARAVA_APPSLEEP_CTRL, to invoid shutdown SRAM LDO.
	 */
	arava_write(ARAVA_APPSLEEP_CTRL, 0x27);

	/* IRQ is masked during the power-up sequence and will not be released
	 * until they have been read for the first time */
	arava_read(ARAVA_EVENT_A, &value);
	arava_read(ARAVA_EVENT_B, &value);
	arava_read(ARAVA_EVENT_C, &value);

	pmic_set_ops(&arava_pmic_ops);

#ifdef ARAVA_THREAD
	kernel_thread(arava_event_handler, NULL, CLONE_KERNEL);
#endif

#ifdef CONFIG_PROC_FS
	ret = create_arava_proc_file();
	if (ret) {
		printk(KERN_INFO "%s: Failed to create arava proc file.\n",
				__func__);
		free_irq(client->irq, NULL);
		return ret;
	}
#endif
	return 0;
}

static int arava_remove(struct i2c_client *client)
{
#ifdef	CONFIG_PROC_FS
	remove_arava_proc_file();
#endif
	pmic_set_ops(NULL);
	free_irq(client->irq, NULL);
	return 0;
}

static const struct i2c_device_id arava_id[] = {
	{ "arava", 0 },
	{ }
};

static struct i2c_driver arava_driver = {
	.driver = {
		.name	= "arava",
	},
	.probe		= arava_probe,
	.remove		= arava_remove,
	.id_table	= arava_id,
#ifdef CONFIG_PM
	.suspend	= arava_suspend,
	.resume		= arava_resume,
#endif
};

static int __init arava_init(void)
{
	int ret;

	ret = i2c_add_driver(&arava_driver);
	if (ret) {
		printk(KERN_WARNING "arava: Driver registration failed, \
					module not inserted.\n");
		return ret;
	}

	return ret;
}

static void __exit arava_exit(void)
{
	i2c_del_driver(&arava_driver);
}

subsys_initcall(arava_init);
module_exit(arava_exit);

MODULE_DESCRIPTION("Arava Driver");
MODULE_LICENSE("GPL");

