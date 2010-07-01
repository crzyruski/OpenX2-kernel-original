#ifdef CONFIG_CPU_PXA935
#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/workqueue.h>
#include <linux/usb/otg.h>
#include <linux/usb/gadget.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/workqueue.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/usb/cdc.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>



#include <asm/irq.h>
#include <asm/uaccess.h>
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/pxa-regs.h>
#include <mach/pxa3xx-regs.h>
#include <mach/pxa9xx_u2o.h>
#include <mach/pxa3xx_pmic.h>
#include <asm/uaccess.h>

#include "pxa3xx_otg.h"
#include "../otg/pxa3xx_otg.h"
#include "../gadget/mv/mvUsbDevApi.h"
#include "../gadget/mvUsb.h"

#ifdef CONFIG_PXA3xx_DVFM
#include <mach/dvfm.h>
#endif

#define DRIVER_VERSION "05-May-2008"
#define DRIVER_DESC "Marvell USB 2.0 OTG Controller"
static const char driver_name [] = "pxa9xx-u2otg";
static const char driver_desc [] = DRIVER_DESC;


struct pxa9xx_u2otg 
{
    struct device               *dev; 
    struct clk			*clk;
    struct pxa9xx_u2o_mach_info *mach;
};


static struct work_struct	pxa9xx_otgc_work;
static struct workqueue_struct 	*pxa9xx_otgc_work_queue;
extern struct mv_usb_dev *get_the_controller(void);

extern void pxa3xx_otg_require_bus(int require);


/*----------------------------------------------------------------*/
static int u2otg_proc_read(char *page, char **start, off_t off, int count,
		int *eof, void *_dev)
{
	char			*buf = page;
	char			*next = buf;
	unsigned		size = count;
	unsigned long		flags;
	int			t;

	local_irq_save(flags);
	t = scnprintf(next, size, DRIVER_DESC "\n"
		"%s\n\tU2xOTGSC@0x%x = 0x%x\n",
		driver_name, (int)&U2xOTGSC, U2xOTGSC);
	size -= t;
	next += t;

	local_irq_restore(flags);
	*eof = 1;
	return count - size;
}

static int u2otg_proc_write(struct file *filp, const char *buffer,
		unsigned long count, void *data)
{
	char kbuf[8];
	int index;

	if (count >= 8)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;
	index = (int)simple_strtoul(kbuf, NULL, 10);

	switch (index) {
	default:
		break;
	}
	return count;
}

#define create_proc_files() \
	do {	struct proc_dir_entry *ent;\
		ent = create_proc_entry("driver/u2otg", 0, NULL);\
		if (ent) { \
			ent->data = u2otg_dev; \
			ent->read_proc = u2otg_proc_read; \
			ent->write_proc = u2otg_proc_write; \
		} \
	} while (0);
#define remove_proc_files() \
	remove_proc_entry("driver/u2otg", NULL)


/******************************************************************
 * Pxa9xx OTG transceiver functions
 ******************************************************************
 */
static int xceiv_mode = USB_OTG_OFF;
static int pxa9xx_otgx_get_mode(void)
{
	return xceiv_mode;
}
int otg_is_client(void)
{
	struct mv_usb_dev *mv_dev = get_the_controller();
	return (mv_dev->transceiver->default_a != 1);
}
int otg_is_host(void)
{
	struct mv_usb_dev *mv_dev = get_the_controller();
	int ret = mv_dev->transceiver->state == OTG_STATE_A_WAIT_BCON;
	struct pxa_otg *pxa_otg = container_of(mv_dev->transceiver, \
                                               struct pxa_otg, otg);

	ret |= mv_dev->transceiver->state == OTG_STATE_A_HOST;
	ret |= (mv_dev->transceiver->state == OTG_STATE_A_IDLE) && pxa_otg->otg_ctrl->a_bus_req;
	ret |= mv_dev->transceiver->state == OTG_STATE_A_WAIT_VFALL;
	ret |= mv_dev->transceiver->state == OTG_STATE_A_WAIT_VRISE;
	return ret;
}

/* Configures USB 2.0 OTG controller to the desired mode */
extern void pxa9xx_gadget_init(void);
extern void pxa9xx_ehci_set(int);
extern void set_vbus(int);

static void pxa9xx_u2o_host_init(struct work_struct *work)
{
	volatile u32 status = U2xOTGSC;
	unsigned long flags;

	local_irq_save(flags);
	pxa9xx_ehci_set(1);
	local_irq_restore(flags);
}

static int pxa9xx_otgx_set_mode(int mode)
{
	int status = 0;

	switch (mode) {

	case USB_OTG_LP:
		break;
	case USB_INT_OTG_CLIENT_DP:
		pxa9xx_ehci_set(0);
		pxa9xx_gadget_init();
		break;

	case USB_INT_OTG_HOST_DP:
		INIT_WORK(&pxa9xx_otgc_work, 
			(void (*)(void *))pxa9xx_u2o_host_init);
		queue_work(pxa9xx_otgc_work_queue, &pxa9xx_otgc_work);
		break;
	case USB_OTG_PRE_SYNCH:
		break;
	default:
		status = OTG_INVALID_PARAMETER;
		break;
	}

	pr_debug("set mode %d USBINTR %x STS %x\n", 
			mode, U2xUSBINTR, U2xUSBSTS);
	xceiv_mode = mode;
	return status;
}


static void pxa9xx_otgx_init(void)
{
	printk("%s not implemented\n", __func__);
}

static int pxa9xx_otgx_dp_session(void)
{
	printk("%s not implemented\n", __func__);
	return 0;
}
static int pxa9xx_otgx_vbus_session(struct pxa_otg *pOtgHandle)
{
	printk("%s not implemented\n", __func__);
	return 0;
}
static int pxa9xx_otgx_check_b_hnp(void)
{
	printk("%s not implemented\n", __func__);
	return 0;
}
static int pxa9xx_otgx_check_vbus(void)
{
	printk(KERN_DEBUG "%s not implemented\n", __func__);
	return 0;
}
static int pxa9xx_otgx_start_autoresume(void)
{
	printk("%s not implemented\n", __func__);
	return 0;
}
static int pxa9xx_otgx_drive_resume(void)
{
	printk("%s not implemented\n", __func__);
	return 0;
}

static enum otg_function pxa9xx_otgx_detect_default_func(void)
{
	/* enable OTG ID pullup register */
	U2xOTGSC |= U2xOTGSC_IDPU;
	mdelay(3);

	/* ID detect */
	if (U2xOTGSC & U2xOTGSC_ID) {
		return OTG_B_DEVICE;
	}

	return OTG_A_DEVICE;
}

static struct otg_xceiv_ops pxa9xx_otg_xceiv_ops = {
	.otgx_get_mode                  = pxa9xx_otgx_get_mode,
	.otgx_set_mode                  = pxa9xx_otgx_set_mode,
	.otgx_init                      = pxa9xx_otgx_init,
	.otgx_detect_default_func	= pxa9xx_otgx_detect_default_func,
	.otgx_dp_session                = pxa9xx_otgx_dp_session,
	.otgx_vbus_session              = pxa9xx_otgx_vbus_session,
	.otgx_check_b_hnp               = pxa9xx_otgx_check_b_hnp,
	.otgx_check_vbus                = pxa9xx_otgx_check_vbus,
	.otgx_start_autoresume          = pxa9xx_otgx_start_autoresume,
	.otgx_drive_resume              = pxa9xx_otgx_drive_resume,
};

struct otg_xceiv_ops *init_pxa9xx_otg_xceiv_ops(void)
{
	return &pxa9xx_otg_xceiv_ops;
}

/******************************************************************
 * Pxa9xx OTG controller functions
 ******************************************************************
 */
static void pxa9xx_otgc_interrupt_init(void)
{

}

static int pxa9xx_otgc_set_mode(int mode)
{
	printk("%s %d not implemented\n", __func__, mode);
	return 0;
}

static int pxa9xx_otgc_reset(void)
{
	int delay;

	U2xUSBCMD &= ~U2xUSBCMD_RS;
	U2xUSBCMD |= U2xUSBCMD_RST;
	delay = 10000;
	while ((U2xUSBCMD & U2xUSBCMD_RST) && delay--);
	if (delay <= 0) {
		printk("%s reset timeout\n", __func__);
	}

	U2xUSBINTR = 0;
	U2xUSBMODE &= ~0x3;
	U2xUSBSTS = U2xUSBSTS;
	pr_debug("%s USBCMD %x USBSTS %x USBINTR %x\n", 
		__func__, U2xUSBCMD, U2xUSBSTS, U2xUSBINTR);
	return 0;
}

static int pxa9xx_otgc_interrupt_handle(struct pxa_otg *pOtgHandle)
{
	unsigned long flags;
	u32 status = (U2xOTGSC & U2xOTGSC_IS_MASK) & 
		((U2xOTGSC & U2xOTGSC_IE_MASK)>>8);
	u8 int_type;

	pr_debug("%s, status %x\n", __func__, status);
	U2xOTGSC |= U2xOTGSC;
	if (!status)
		return OTG_INT_INIT;
	local_irq_save(flags);

	pxa9xx_otgc_reset();

	if (U2xOTGSC & U2xOTGSC_ID) {
		int_type = OTG_INT_IDR;
	} else {
		int_type = OTG_INT_IDF;
	}
	local_irq_restore(flags);
	return int_type;
}

static void pxa9xx_otgc_init_gadget(void)
{
	U2xOTGSC &= ~(U2xOTGSC_IDIE | U2xOTGSC_IDPU);
	U2xOTGSC |= U2xOTGSC;
}

static int pxa9xx_otgc_init(void) 
{
	struct pxa9xx_u2otg *u2otg_dev;

	pxa9xx_otgc_reset();

	pr_debug("U2xUSBCMD %x U2xOTGSC %x\n", U2xUSBCMD, U2xOTGSC);
    	U2xOTGSC |= U2xOTGSC_IDIE | U2xOTGSC_IDPU;

	pxa9xx_otgc_work_queue = create_workqueue("pxa9xx_otgc_work_queue");

    	/* create_proc_files(); */
	return 0;
}

static void pxa9xx_otgc_deinit(void)
{
	printk("%s\n", __func__);
}

static struct otg_ctrl_ops pxa9xx_otg_ctrl_ops = {
	.otgc_init              = pxa9xx_otgc_init, 
	.otgc_deinit            = pxa9xx_otgc_deinit,
	.otgc_interrupt_init    = pxa9xx_otgc_interrupt_init,
	.otgc_interrupt_handle  = pxa9xx_otgc_interrupt_handle,
	.otgc_init_gadget       = pxa9xx_otgc_init_gadget,
};

struct  otg_ctrl_ops *init_pxa9xx_otg_ctrl_ops(void)
{
	return &pxa9xx_otg_ctrl_ops;
}

#endif /* CONFIG_CPU_PXA935 */
