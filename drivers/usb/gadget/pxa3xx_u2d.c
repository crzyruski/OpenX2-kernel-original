/*
 * linux/drivers/usb/gadget/pxa3xx_u2d.c
 * PXA3xx on-chip high speed USB device controllers
 *
 * Copyright (C) 2002 Intrinsyc, Inc. (Frank Becker)
 * Copyright (C) 2003 Robert Schwebel, Pengutronix
 * Copyright (C) 2003 Benedikt Spranger, Pengutronix
 * Copyright (C) 2003 David Brownell
 * Copyright (C) 2003 Joshua Wise
 * Copyright (C) 2004 Intel Corporation
 * Copyright (C) 2006 Marvell International Ltd
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
 * change log:
 * 2006.2.14	xiang,jing	modify for gpio-expander long interrupt lantency
 * 2007.5.16	brown,mark	added support for otg carkit mode
 */

/*
 #define	DEBUG
 #define	VERBOSE
 #undef	DBG_NOISY
 */

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
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/clk.h>

#include <asm/byteorder.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <asm/unaligned.h>
#include <mach/hardware.h>
#include <mach/pxa-regs.h>
#include <mach/pxa3xx-regs.h>
#include <asm/uaccess.h>
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/pxa3xx_pmic.h>
#include <mach/pxa3xx_u2d.h>
#include <mach/zylonite.h>

#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/usb/cdc.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,28) && defined(CONFIG_USB_ANDROID)
#define PXA_EP_AUTO
#endif
/*
 * This driver handles the USB 2.0 Device Controller (U2D) in Marvell's
 * PXA 3xx series processors.
 * Such controller drivers work with a gadget driver.  The gadget driver
 * returns descriptors, implements configuration and data protocols used
 * by the host to interact with this device, and allocates endpoints to
 * the different protocol interfaces.  The controller driver virtualizes
 * usb hardware so that the gadget drivers will be more portable.
 *
 * This U2D hardware wants to implement a bit too much USB protocol, so
 * it constrains the sorts of USB configuration change events that work.
 * The errata for these chips are misleading; some "fixed" bugs from
 * pxa250 a0/a1 b0/b1/b2 sure act like they're still there.
 */

#include <linux/kernel.h>

#undef U2D_USE_ISRAM
#ifdef U2D_USE_ISRAM
#include <mach/imm.h>
#endif

#include "../otg/pxa3xx_otg.h"

#include "pxa_comp.h"
#include "pxa27x_udc.h"
#include <mach/hardware.h>

#define	DRIVER_VERSION	"01-Jan-2006"
#define	DRIVER_DESC	"Monahans USB 2.0 Device Controller driver"

static const char driver_name [] = "pxa3xx_u2d";
static const char ep0name [] = "ep0";

static struct pxa27x_udc *the_controller;
#ifdef CONFIG_PROC_FS
#define	U2D_PROC_FILE
#endif

#ifdef CONFIG_EMBEDDED
/* few strings, and little code to use them */
#undef	DEBUG
#undef	U2D_PROC_FILE
#endif

#define	USE_SPEOREN
static int u2d_bugs;
static int connected;

#ifdef CONFIG_PXA3xx_DVFM
#include <mach/dvfm.h>
#endif

static int soft_dis_en;
static int skip_ep_num;
static int check_fifo;
static u8 u2d_ep_num;
static struct pxa27x_udc memory;

static void __u2d_disable(struct pxa27x_udc *dev);
static void __u2d_enable(struct pxa27x_udc *dev);
static void u2d_init_ep(struct pxa27x_udc *dev);
static void nuke(struct pxa27x_ep *ep, int status);

#ifdef CONFIG_USB_OTG_PXA3xx_U2D
extern void  pxa3xx_otg_require_bus(int status);
extern void  pxa3xx_otg_reset(void);
#endif
extern void enable_oscc_pout(void);
extern void disable_oscc_pout(void);

extern struct otg_xceiv_ops *init_pxa310_otg_xceiv_ops(void);
extern struct otg_xceiv_ops *init_pxa930_otg_xceiv_ops(void);
#define	DMASTR " (dma support)"

#ifdef	CONFIG_USB_PXA27X_SMALL
#define SIZE_STR	" (small)"
#else
#define SIZE_STR	""
#endif

#define U2DINT_EP0       0x7
#define U2DINT_MASK      (U2DINT_SPACKETCMP     \
			  | U2DINT_PACKETCMP | U2DINT_FIFOERR)
#define U2DISR_MASK      (0xFFFFFF)

#ifdef U2D_USE_ISRAM
static u32 zylu2d_immid;
#endif

#ifdef DEBUG
#define dmsg(format, args...) printk(KERN_DEBUG "%s: " format "\n", \
				     __FUNCTION__, ## args)
#else
#define dmsg(format, args...) do { } while (0)
#endif

/* WRITE_COMPLETE()
 *
 *	The macro below will ensure a write to "ptr" completes.
 *	"ptr" is usually a volatile pointer to a register.
 *
 *	A write register transaction can typically complete asynchronously.
 *	This greatly improves the performance of a system. However, there
 *	are some circumstances when software wants to gaurantee that a
 *	register update has completed before continuing exectuion. Setting
 *	and clearing interrupt status bits are some examples of these
 *	situations.
 *
 *	Ensuring the write to "ptr" completes is a two step process:
 *		step 1: read back the contents of "ptr"
 *		step 2: make any use of the value read.
 *
 *	If step 2 is omitted, then the pipelining architecture of the core
 *	may never stall at step 1 to wait for the write to complete. Step 2
 *	allows the pipelining architecture of the core to determine that
 *	there is a dependency on the completion of the read in step 1. So
 *	it is the read in step 1, plus the dependency on read completion in
 *	step 2, that cause the core to stall until these transactions complete.
 *
 *	This technique should be used whenever it is critical that a write
 *	to a register completes before code continues execution.
 */
#define WRITE_COMPLETE(ptr) {(void)__raw_readl(ptr); }

/*
 * u2d_bug_check
 * U2D_BUG_INMASS:	MHN_P_B0, MHN_P_B1, MHN_L_A0
 * U2D_BUG_SETINTF:	MHN_P_B0, MHN_P_B1
 * U2D_BUG_STALL:	MHN_P_B0, MHN_P_B1, MHN_L_A0
 * DDR_BUG_DMA:		MHN_P_B0, MHN_P_B1, MHN_L_A0
 * U2D_BUG_UTMID:	MHN_LV_A0
 * 	Both the U2DCR[UDE] and U2DOTGCR[ULE] register bits are
 * 	set to 0 when touching UTMID
 * U2D_FIX_ULPI_STP:	MHN_LV_A2 and above
 * 	MHN_LV_A2 fixed the bug that ULPI_STP would be asserted
 *	after system goes out of Low Power mode
 */

int get_u2d_bugs(void)
{
	return u2d_bugs;
}

struct pxa27x_udc *get_the_controller(void)
{
	return the_controller;
}

static int u2d_bug_check(void)
{
	unsigned int cpuid;

	/* read CPU ID */
	cpuid = read_cpuid(CPUID_ID);

	/* It's not xscale chip. */
	if ((cpuid & 0xFFFF0000) != 0x69050000)
		return U2D_BUG_NONE;

	/* It's MhnP Bx */
	if ((cpuid & 0x0000FFF0) == 0x00006820) {
		if (((cpuid & 0x0F) == 4) || ((cpuid & 0x0F) == 5))
			/* MhnP B0, B1 */
			return U2D_BUG_INMASS | U2D_BUG_SETINTF | \
			       U2D_BUG_STALL | DDR_BUG_DMA;
		else
			/* MhnP B2 */
			return U2D_BUG_NONE;
	}

	/* It's MhnL Ax */
	if ((cpuid & 0x0000FFF0) == 0x00006880) {
		if ((cpuid & 0x0F) == 0)
			/* MhnL A0 */
			return U2D_BUG_INMASS | U2D_BUG_STALL | \
			       DDR_BUG_DMA;
		else
			/* MhnL A1 and above */
			return U2D_BUG_NONE;
	}

	/* It's MhnLV Ax */
	if ((cpuid & 0x0000FFF0) == 0x00006890) {
		if ((cpuid & 0x0F) == 0)
			return U2D_BUG_UTMID;
		else
			return U2D_FIX_ULPI_STP;
	}

	return U2D_BUG_NONE;
}

#if defined(CONFIG_PXA3xx_DVFM)
static struct dvfm_lock dvfm_lock = {
	.lock   	= SPIN_LOCK_UNLOCKED,
	.dev_idx	= -1,
	.count  	= 0,
};

void set_dvfm_constraint(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	DMSG("pre set dvfm constraint, disable count:%d\n", dvfm_lock.count);
	if (dvfm_lock.count++ == 0) {
		if (u2d_bugs & DDR_BUG_DMA)
			dvfm_disable(dvfm_lock.dev_idx);
		else {
			/* Disable D0CS */
			dvfm_disable_op_name("D0CS", dvfm_lock.dev_idx);
			/* Disable Lowpower mode */
			dvfm_disable_op_name("D1", dvfm_lock.dev_idx);
			dvfm_disable_op_name("D2", dvfm_lock.dev_idx);
			if (cpu_is_pxa935())
				dvfm_disable_op_name("CG", dvfm_lock.dev_idx);
		}
	}
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}

void unset_dvfm_constraint(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	DMSG("pre unset dvfm constraint, disable count:%d\n", dvfm_lock.count);
	if (dvfm_lock.count == 0) {
		printk(KERN_DEBUG "U2D constraint has been removed.\n");
	} else if (--dvfm_lock.count == 0) {
		if (u2d_bugs & DDR_BUG_DMA)
			dvfm_enable(dvfm_lock.dev_idx);
		else {
			/* Enable D0CS */
			dvfm_enable_op_name("D0CS", dvfm_lock.dev_idx);
			/* Enable Lowpower mode */
			dvfm_enable_op_name("D1", dvfm_lock.dev_idx);
			dvfm_enable_op_name("D2", dvfm_lock.dev_idx);
			if (cpu_is_pxa935())
				dvfm_enable_op_name("CG", dvfm_lock.dev_idx);
		}
	}
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}

static void set_dvfm_constraint_by_force(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	/* Disable D0CS */
	dvfm_disable_op_name("D0CS", dvfm_lock.dev_idx);
	/* Disable Lowpower mode */
	dvfm_disable_op_name("D1", dvfm_lock.dev_idx);
	dvfm_disable_op_name("D2", dvfm_lock.dev_idx);
	if (cpu_is_pxa935())
		dvfm_disable_op_name("CG", dvfm_lock.dev_idx);
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}

static void unset_dvfm_constraint_by_force(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	/* Enable D0CS */
	dvfm_enable_op_name("D0CS", dvfm_lock.dev_idx);
	/* Enable Lowpower mode */
	dvfm_enable_op_name("D1", dvfm_lock.dev_idx);
	dvfm_enable_op_name("D2", dvfm_lock.dev_idx);
	if (cpu_is_pxa935())
		dvfm_enable_op_name("CG", dvfm_lock.dev_idx);
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}
#else
void set_dvfm_constraint(void) {}
void unset_dvfm_constraint(void) {}
static void set_dvfm_constraint_by_force(void) {}
static void unset_dvfm_constraint_by_force(void) {}
#endif

void u2d_clk_set(int enable)
{
	struct pxa27x_udc *dev = &memory;

	if (enable) {
		/* Enable clock for USB device */
		clk_enable(dev->clk);
		dev->u2d_clk_dis = 0;
	} else {
		/* Enable clock for USB device */
		clk_disable(dev->clk);
		dev->u2d_clk_dis = 1;
	}
}

void u2d_clk_enable(void)
{
	struct pxa27x_udc *dev = &memory;

	/* Enable clock for USB device */
	clk_enable(dev->clk);
}

void u2d_clk_restore(void)
{
	struct pxa27x_udc *dev = &memory;

	if (dev->u2d_clk_dis) {
		/* Enable clock for USB device */
		clk_disable(dev->clk);
	}
}

void u2d_irq_set(int en)
{
	struct pxa27x_udc *dev = &memory;
	unsigned long flags;

	local_irq_save(flags);
	if (!dev->u2d_irq_dis && !en) {
		disable_irq(IRQ_USB2);
		dev->u2d_irq_dis = 1;
	}

	if (dev->u2d_irq_dis && en) {
		enable_irq(IRQ_USB2);
		dev->u2d_irq_dis = 0;
	}
	local_irq_restore(flags);
}
/* platform related functions */

void ulpi_phy_init(struct pxa27x_udc *dev)
{
	u32 u2dotgcr;

	if (cpu_is_pxa930()) {
		u2dotgcr = U2DOTGCR;

		dev->xv_ops->otgx_init();
		/* set to peripheral mode */
		U2DOTGCR = 0;
		u2dotgcr = U2DOTGCR;

		u2dotgcr |= U2DOTGCR_ULE;
		U2DOTGCR = u2dotgcr;
	} else if (cpu_is_pxa310()) {
#ifndef CONFIG_USB_OTG_PXA3xx_U2D
		u2dotgcr = U2DOTGCR;

		dev->xv_ops->otgx_init();
		/* set to peripheral mode */
		U2DOTGCR = 0;
		u2dotgcr = U2DOTGCR;

		u2dotgcr |= U2DOTGCR_ULE;
		U2DOTGCR = u2dotgcr;
		u2dotgcr |= U2DOTGCR_ULAF;
		u2dotgcr &= ~(U2DOTGCR_SMAF | U2DOTGCR_CKAF);
		U2DOTGCR = u2dotgcr;
		u2dotgcr = U2DOTGCR;
#endif
	}
}

static irqreturn_t ulpi_dat3_irq(int irq, void *devid)
{
	struct pxa27x_udc *dev = (struct pxa27x_udc *)devid;

	if (cpu_is_pxa310()) {
		disable_irq(irq);
		dev->stp_gpio_irq_en = 0;
		dev->ulpi_dat3_work = 1;

		dev->xv_ops->ulpi_dat3_work();

		DMSG("%s end\n", __func__);
	}
	return IRQ_HANDLED;
}

/* detect USB cable attach and detach by GPIO46
 * 1 -- cable attached; 0 -- cable detached
 */
int is_cable_attached(void)
{
	struct pxa27x_udc *dev = &memory;
	int ret = 1;	/* default is connected */

#if defined(CONFIG_CHARGER_PXA3xx_HARDWARE)
	if (!dev->driver) {
#else
	if (!dev->driver || dev->stp_gpio_irq_en) {
#endif
		ret = 0;
		goto out;
	}

	dmsg("\n");
	if (cpu_is_pxa310()) {
#if defined(CONFIG_CHARGER_PXA3xx_HARDWARE)
		extern int pxa3xx_hwchg_cable_connect(void);
		ret = pxa3xx_hwchg_cable_connect();
#else
		ret = dev->xv_ops->otgx_check_vbus();
#endif
	} else if (cpu_is_pxa930())
		ret = pxa3xx_pmic_is_vbus_assert();
	else {
		unsigned int value;
		value = gpio_get_value(dev->u2d_dp);
		dmsg("value = %d\n", value);
		ret = ((value == 1));
	}
out:
	connected = ret;
	/* DMSG("%s, ret %d\n", __func__, ret); */
	return ret;
}

/* u2d soft disconnection */
static int u2d_soft_dis(int enable)
{
	struct pxa27x_udc *dev = &memory;

	if (cpu_is_pxa310() || cpu_is_pxa930()) {
		if (enable) {
			if (U2DCR & U2DCR_ADD)
				return 1;
			U2DCR |= U2DCR_ADD;
			mdelay(3);
		} else {
			if (!(U2DCR & U2DCR_ADD))
				return 1;
			U2DCR &= ~U2DCR_ADD;
			mdelay(3);
		}
	} else {
		if (enable) {
			if (soft_dis_en)
				return 1;
			soft_dis_en = 1;
			dev->mach->soft_dis(enable);
		} else {
			if (!soft_dis_en)
				return 1;
			dev->mach->soft_dis(enable);
			soft_dis_en = 0;
		}
	}
	DMSG("soft dis %s, U2DCR %x\n", enable ? "enable" : "disable", U2DCR);
	return 0;
}

int cable_detect_interrupt(void)
{
	struct pxa27x_udc *dev = &memory;
	unsigned long flags;
	int mode;

	DMSG("%s , connected %d\n", __func__,
	     is_cable_attached());
	local_irq_save(flags);
	if (dev->driver) {
		if (is_cable_attached()) {
			#ifdef CONFIG_PXA3xx_DVFM								
			/* U2D module has resided in kernel */
			DMSG("%s set dvfm constraint\n", __func__);
			set_dvfm_constraint_by_force();
			#endif
			if (dev->xv_ops) {
				mode = USB_INT_OTG_CLIENT_DP;
				dev->xv_ops->otgx_set_mode(mode);
			}
			__u2d_enable(dev);
			u2d_soft_dis(0);
			udc_stop(&dev->cdev, &dev->gadget, dev->driver, 0);
			//kobject_uevent(&dev->dev->kobj, KOBJ_ADD);
		} else {
			/* cable detached */
			u2d_soft_dis(1);
			if (dev->xv_ops) {
				mode = USB_OTG_LP;
				dev->xv_ops->otgx_set_mode(mode);
			}
			__u2d_disable(dev);
			udc_stop(&dev->cdev, &dev->gadget, dev->driver, 1);
			//kobject_uevent(&dev->dev->kobj, KOBJ_REMOVE);
			#ifdef CONFIG_PXA3xx_DVFM
			DMSG("%s unset dvfm constraint\n", __func__);
			unset_dvfm_constraint_by_force();
			#endif
		}
	}

	local_irq_restore(flags);
	return IRQ_HANDLED;
}

void pxa3xx_usb_event_change(unsigned int events)
{
	if (cpu_is_pxa930() && (events & PMIC_EVENT_VBUS))
			cable_detect_interrupt();
}

irqreturn_t cable_detect_irq(int irq, void *_dev, struct pt_regs *r)
{
	return cable_detect_interrupt();
}

static void cable_detect_init(void)
{
	int ret;
	int detect_irq;
	struct pxa27x_udc *dev = &memory;

	if (cpu_is_pxa310()) {
#ifndef CONFIG_USB_OTG_PXA3xx_U2D
		ulpi_phy_init(dev);
		U2DOTGICR = U2DOTGINT_SI | U2DOTGINT_RVV | U2DOTGINT_FVV;
#endif
	} else if (cpu_is_pxa930()) {
		ulpi_phy_init(dev);
		pxa3xx_pmic_set_pump(1);
	} else {
		ret = gpio_request(dev->u2d_dp, "U2D cable detect");
		if (ret) {
			gpio_free(dev->u2d_dp);
			printk(KERN_ERR "Request GPIO failed, return :%d\n",
			       ret);
		}
		/* clear the MFPR */
		gpio_direction_input(dev->u2d_dp);

		detect_irq = gpio_to_irq(dev->u2d_dp);
		/* request irq */
		ret = request_irq(detect_irq, (void *)cable_detect_irq,
				  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				  "U2D cable detect", NULL);

		if (ret) {
			gpio_free(dev->u2d_dp);
			printk(KERN_ERR "Request IRQ failed, return :%d\n",
			       ret);
		}
	}
}

static void cable_detect_deinit(void)
{
	int detect_irq;
	struct pxa27x_udc *dev = &memory;

	if (cpu_is_pxa310()) {
		dev->xv_ops->reset_xcvr_init();
#ifndef CONFIG_USB_OTG_PXA3xx_U2D
		U2DOTGICR &= ~(U2DOTGINT_SI | U2DOTGINT_RVV | U2DOTGINT_FVV);
#endif
	} else if (cpu_is_pxa930()) {
		dev->xv_ops->reset_xcvr_init();
		pxa3xx_pmic_set_pump(1);
	} else {
		detect_irq = gpio_to_irq(dev->u2d_dp);
		free_irq(detect_irq, NULL);
		gpio_free(dev->u2d_dp);
	}
}

/* ---------------------------------------------------------------------------
 * 	endpoint related parts of the api to the usb controller hardware,
 *	used by gadget driver; and the inner talker-to-hardware core.
 * ---------------------------------------------------------------------------
 */

static void pxa3xx_ep_fifo_flush(struct usb_ep *ep);

static int pxa3xx_dma_desc_alloc(struct pxa27x_ep *ep)
{
	if (ep->dma_desc_virt && ep->dma_desc_phys != -1)
		return 0;
#ifdef U2D_USE_ISRAM
	ep->dma_desc_virt = imm_malloc(DMA_DESC_SIZE,
				       IMM_MALLOC_HARDWARE | IMM_MALLOC_SRAM,
				       zylu2d_immid);
	if (ep->dma_desc_virt)
		ep->dma_desc_phys = imm_get_physical((void *)ep->dma_desc_virt,
						     zylu2d_immid);
	else {
		printk(KERN_ERR "can't malloc ISRAM by IMM successfully \n");
		return -ENOMEM;
	}
#else
	ep->dma_desc_virt = dma_alloc_coherent(ep->dev->dev, DMA_DESC_SIZE,
					       &ep->dma_desc_phys, GFP_ATOMIC);
#endif
	if (!ep->dma_desc_virt) {
		printk(KERN_ERR "%s: failed to allocate dma desc buf\n",
		       __FUNCTION__);
		return -ENOMEM;
	}
	ep->dma_desc_size = DMA_DESC_SIZE;

	return 0;
}

static int pxa3xx_dma_desc_free(struct pxa27x_ep *ep)
{
#ifdef U2D_USE_ISRAM
	if (ep->dma_desc_virt)
		imm_free((void *)ep->dma_desc_virt, zylu2d_immid);
#else
	if (irqs_disabled()) {
		local_irq_enable();
		dma_free_coherent(ep->dev->dev, ep->dma_desc_size, \
				  ep->dma_desc_virt, ep->dma_desc_phys);
		local_irq_disable();
	} else
		dma_free_coherent(ep->dev->dev, ep->dma_desc_size, \
				  ep->dma_desc_virt, ep->dma_desc_phys);
#endif
	ep->dma_desc_virt = NULL;
	ep->dma_desc_phys = ep->dma_desc_size = -1 ;
	ep->dma = -1;
	return 0;
}

static int pxa3xx_dma_buf_alloc(struct pxa27x_ep *ep)
{
	ep->dma_buf_size = DMA_BUF_SIZE;
	return 0;
}

static int pxa3xx_dma_buf_free(struct pxa27x_ep *ep)
{
	ep->dma_buf_virt = NULL;
	ep->dma_buf_phys = ep->dma_buf_size = -1 ;

	return 0;
}

static int get_mps(int speed, __u8 bmAttributes)
{
	switch (bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) {
	case USB_ENDPOINT_XFER_CONTROL:
		return EP0_MPS;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		return ISO_MPS(speed);
		break;
	case USB_ENDPOINT_XFER_BULK:
		return BULK_MPS(speed);
		break;
	case USB_ENDPOINT_XFER_INT:
		return INT_MPS(speed);
		break;
	default:
		return 0;
		break;
	}
}

static void change_mps(enum usb_device_speed speed)
{
	unsigned i;
	struct pxa27x_ep *pxa_ep = NULL;
	struct pxa27x_udc *dev = the_controller;

	DMSG("%s, speed = %s\n", __FUNCTION__,
	     (speed == USB_SPEED_HIGH) ? "high" : "full");
	/* find all validate EPs and change the MPS */
	for (i = 1; i < UDC_LG_EP_NUM; i++) {
		if (dev->ep[i].assigned) {
			pxa_ep = &dev->ep[i];
			switch (pxa_ep->ep_type & USB_ENDPOINT_XFERTYPE_MASK) {
			case USB_ENDPOINT_XFER_CONTROL:
				pxa_ep->ep.maxpacket = EP0_MPS;
				break;
			case USB_ENDPOINT_XFER_ISOC:
				pxa_ep->ep.maxpacket = ISO_MPS(speed);
				break;
			case USB_ENDPOINT_XFER_BULK:
				pxa_ep->ep.maxpacket = BULK_MPS(speed);
				break;
			case USB_ENDPOINT_XFER_INT:
				pxa_ep->ep.maxpacket = INT_MPS(speed);
				break;
			default:
				break;
			}
		}
	}
}

/*
 * endpoint enable/disable
 *
 * we need to verify the descriptors used to enable endpoints.  since pxa3xx
 * endpoint configurations are fixed, and are pretty much always enabled,
 * there's not a lot to manage here.
 *
 * because pxa3xx can't selectively initialize bulk (or interrupt) endpoints,
 * (resetting endpoint halt and toggle), SET_INTERFACE is unusable except
 * for a single interface (with only the default altsetting) and for gadget
 * drivers that don't halt endpoints (not reset by set_interface).  that also
 * means that if you use ISO, you must violate the USB spec rule that all
 * iso endpoints must be in non-default altsettings.
 */
static int pxa3xx_ep_enable(struct usb_ep *_ep,
			    const struct usb_endpoint_descriptor *desc)
{
	struct pxa27x_ep        *ep;
	struct pxa27x_udc       *dev;

	ep = container_of(_ep, struct pxa27x_ep, ep);
	if (!_ep || !desc || _ep->name == ep0name
	    || desc->bDescriptorType != USB_DT_ENDPOINT
	    || ep->fifo_size < le16_to_cpu(desc->wMaxPacketSize)) {
		DMSG("%s, bad ep or descriptor\n", __FUNCTION__);
		return -EINVAL;
	}

	/* xfer types must match, except that interrupt ~= bulk */
	if (ep->ep_type != USB_ENDPOINT_XFER_BULK
	     && desc->bmAttributes != USB_ENDPOINT_XFER_INT) {
		DMSG("%s, %s type mismatch\n", __FUNCTION__, _ep->name);
		return -EINVAL;
	}

	/* hardware _could_ do smaller, but driver doesn't */
	if ((desc->bmAttributes == USB_ENDPOINT_XFER_BULK
	     && le16_to_cpu(desc->wMaxPacketSize)
	     >= BULK_FIFO_SIZE)
	    || !desc->wMaxPacketSize) {
		DMSG("%s, bad %s maxpacket\n", __FUNCTION__, _ep->name);
		return -ERANGE;
	}

	dev = ep->dev;
	if (!dev->driver ||
	    dev->gadget.speed == USB_SPEED_UNKNOWN) {
		DMSG("%s, bogus device state\n", __FUNCTION__);
		return -ESHUTDOWN;
	}

	ep->desc = desc;
	ep->dma = ep->ep_num;
	ep->dma_buf_virt = NULL;
	ep->stopped = 0;
	ep->pio_irqs = ep->dma_irqs = 0;
	ep->ep.maxpacket = get_mps(dev->gadget.speed, desc->bmAttributes);

	/* flush fifo (mostly for OUT buffers) */
	pxa3xx_ep_fifo_flush(_ep);

	/* ... reset halt state too, if we could ... */

	/* for (some) bulk and ISO endpoints, try to get a DMA channel and
	 * bind it to the endpoint.  otherwise use PIO.
	 */
	DMSG("%s: called attributes=%d\n", __FUNCTION__, ep->ep_type);
	switch (ep->ep_type) {
	case USB_ENDPOINT_XFER_ISOC:
	case USB_ENDPOINT_XFER_INT:
		/* FIXME, is it necessary be 4B align
		 * if (le16_to_cpu(desc->wMaxPacketSize) % 32)
		 * 	break;
		 * 	 fall through
		 */
	case USB_ENDPOINT_XFER_BULK:
		/* request DMA descriptor buffer */
		pxa3xx_dma_desc_alloc(ep);
		/* request DMA buffer */
		pxa3xx_dma_buf_alloc(ep);

		DMSG("%s using dma%d\n", _ep->name, ep->dma);

		break;
	default:
		break;
	}

	DBG(DBG_VERBOSE, "enabled %s\n", _ep->name);
	return 0;
}

static int pxa3xx_ep_disable(struct usb_ep *_ep)
{
	struct pxa27x_ep *ep;

	u2d_clk_enable();

	ep = container_of(_ep, struct pxa27x_ep, ep);
	if (!_ep || !ep->desc) {
		DMSG("%s, %s not enabled\n", __FUNCTION__,
		     _ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	nuke(ep, -ESHUTDOWN);

	if ((ep->dma >= 0) && (ep->dma_buf_virt)) {

		/* free DMA buffer */
		pxa3xx_dma_buf_free(ep);

		/* free DMA descriptor buffer */
		pxa3xx_dma_desc_free(ep);

	}

	/* flush fifo (mostly for IN buffers) */
	pxa3xx_ep_fifo_flush(_ep);
	u2d_clk_restore();

	ep->desc = 0;
	ep->stopped = 1;

	DBG(DBG_VERBOSE, "%s disabled\n", _ep->name);
	return 0;
}

/*-------------------------------------------------------------------------*/

/* for the pxa3xx, these can just wrap kmalloc/kfree.  gadget drivers
 * must still pass correctly initialized endpoints, since other controller
 * drivers may care about how it's currently set up (dma issues etc).
 */

/*
 * 	pxa3xx_ep_alloc_request - allocate a request data structure
 */
static struct usb_request *
pxa3xx_ep_alloc_request(struct usb_ep *_ep, unsigned gfp_flags)
{
	struct pxa27x_request *req;

	req = kzalloc(sizeof *req, gfp_flags);
	if (!req)
		return 0;

	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}


/*
 * 	pxa3xx_ep_free_request - deallocate a request data structure
 */
static void
pxa3xx_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct pxa27x_request *req;

	req = container_of(_req, struct pxa27x_request, req);
	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}


/*-------------------------------------------------------------------------*/

/*
 *	done - retire a request; caller blocked irqs
 */
static void done(struct pxa27x_ep *ep, struct pxa27x_request *req, int status)
{
	DMSG("%s is called\n", __FUNCTION__);
	list_del_init(&req->queue);
	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	if (status && status != -ESHUTDOWN) {
		DBG(DBG_VERBOSE, "complete %s req %p stat %d len %u/%u\n",
		    ep->ep.name, &req->req, status,
		    req->req.actual, req->req.length);
	}

	/* don't modify queue heads during completion callback */
	if (req->req.complete)
		req->req.complete(&ep->ep, &req->req);
}


static inline void ep0_idle(struct pxa27x_udc *dev)
{
	dev->cdev.ep0state = EP0_IDLE;
	LED_EP0_OFF;
}

static int
write_packet(volatile u32 *uddr, struct pxa27x_request *req, unsigned max)
{
	u32		*buf;
	int length, count, remain;

	buf = (u32 *)(req->req.buf + req->req.actual);
	prefetch(buf);

	/* how big will this packet be? */
	length = min(req->req.length - req->req.actual, max);
	req->req.actual += length;

	remain = length & 0x3;
	count = length & ~(0x3);

	while (likely(count)) {
		*uddr = *buf++;
		count -= 4;
	}

	if (remain) {
		if (remain == 3)
			*uddr = *buf;
		else if (remain == 2)
			*(volatile u16 *)uddr = *(u16 *)buf;
		else
			*(volatile u8 *)uddr = *(u8 *)buf;
	}

	return length;
}

/* caller asserts req->pending (ep0 irq status nyet cleared); starts
 * ep0 data stage.  these chips want very simple state transitions.
 */
static inline
void ep0start(struct pxa27x_udc *dev, u32 flags, const char *tag)
{
	U2DCSR0 = flags /*|U2DCSR0_SA|U2DCSR0_OPC*/;
	WRITE_COMPLETE(&U2DCSR0);

	dev->req_pending = 0;
	DBG(DBG_VERY_NOISY, "%s %s, %02x/%02x\n",
	    __FUNCTION__, tag, U2DCSR0, flags);
}

static int
write_ep0_fifo(struct pxa27x_ep *ep, struct pxa27x_request *req)
{
	unsigned count;
	int is_short, adjusted = 0;

	count = write_packet(&U2DDR0, req, EP0_FIFO_SIZE);
	ep->dev->stats.write.bytes += count;

	/* last packet "must be" short (or a zlp) */
	is_short = (count != EP0_FIFO_SIZE);
	adjusted = ((count % 4) == 3);
	if (adjusted)
		adjusted = U2DCSR0_IPA;
	else
		adjusted = 0;

	DBG(DBG_VERY_NOISY, "ep0in %d bytes %d left %p,"
	    "is_short %d req_pending %d\n", count,
	    req->req.length - req->req.actual, &req->req,
	    is_short, ep->dev->req_pending);

	/* Note: don't access any U2D registers between load FIFO and set IPR
	 * bit. otherwise, it may cause transfer wrong number of bytes. */
	if (is_short) {
		if (ep->dev->req_pending)
			ep0start(ep->dev, U2DCSR0_IPR | adjusted, "short IN");
		else {
			U2DCSR0 = U2DCSR0_IPR | adjusted;
			WRITE_COMPLETE(&U2DCSR0);
		}

		count = req->req.length;
		done(ep, req, 0);
		ep0_idle(ep->dev);
		return is_short;
	} else if (ep->dev->req_pending) {
		ep0start(ep->dev, U2DCSR0_IPR, "IN");
		return is_short;
	}

	ep0start(ep->dev, U2DCSR0_IPR, "IN");     /* xj */
	return is_short;
}

/*
 * special ep0 version of the above.  no UBCR0 or double buffering; status
 * handshaking is magic.  most device protocols don't need control-OUT.
 * CDC vendor commands (and RNDIS), mass storage CB/CBI, and some other
 * protocols do use them.
 */
static int
read_ep0_fifo(struct pxa27x_ep *ep, struct pxa27x_request *req)
{
	u32		*buf, word;
	unsigned bufferspace;

	buf = (u32 *)(req->req.buf + req->req.actual);
	bufferspace = req->req.length - req->req.actual;

	while (U2DCSR0 & U2DCSR0_RNE) {
		/* FIXME
		 * if setup data is not multiple of 4, this routing will read
		 * some extra bytes
		 */
		word = U2DDR0;

		if (unlikely(bufferspace == 0)) {
			/* this happens when the driver's buffer
			 * is smaller than what the host sent.
			 * discard the extra data.
			 */
			if (req->req.status != -EOVERFLOW)
				DMSG("%s overflow\n", ep->ep.name);
			req->req.status = -EOVERFLOW;
		} else {
			*buf++ = word;
			req->req.actual += 4;
			bufferspace -= 4;
		}
	}
	U2DCSR0 = U2DCSR0_OPC;
	WRITE_COMPLETE(&U2DCSR0);

	DMSG("%s, req.actual %d req.length %d req.status %d, u2dcsr0 %x\n",
	     __func__, req->req.actual, req->req.length, req->req.status,
	     U2DCSR0);

	/* completion */
	if (req->req.actual >= req->req.length) {
		req->req.actual = req->req.length;
		return 1;
	}

	/* finished that packet.  the next one may be waiting... */
	return 0;
}

#ifdef	DEBUG
void dma_desc_dump(int ep_num)
{
	struct pxa27x_udc *dev = the_controller;
	struct pxa27x_ep *ep;
	pxa_dma_desc *desc;
	int i;

	ep = &dev->ep [ep_num];
	if (!ep->desc)
		printk(KERN_DEBUG "%s, desc not initialized, return\n",
		       __FUNCTION__);

	desc = ep->dma_desc_virt;
	for (i = 0; i < 3; i++) {
		printk(KERN_DEBUG "\tdadr:%x, dsdr:%x, dtdr:%x, dcmd:%x \n",
		       (unsigned)desc->ddadr, (unsigned)desc->dsadr,
		       (unsigned)desc->dtadr, (unsigned)desc->dcmd);
		desc++;
	}

}

void dma_info_dump(void)
{
	struct pxa27x_udc *dev = the_controller;
	struct pxa27x_ep *ep;
	const struct usb_endpoint_descriptor	*d;
	pxa_dma_desc *desc;
	int i, j;

	for (i = 1; i < UDC_LG_EP_NUM; i++) {
		ep = &dev->ep [i];
		d = ep->desc;
		if (!d)
			continue;

		/* DMA desc chain */
		printk(KERN_DEBUG "ep %d:\n"      \
		       "\tdesc %p(%p) num %d(%dB) buf %p(%p) size %uB\n", i,
		       (int *)ep->dma_desc_phys, ep->dma_desc_virt,
		       ep->dma_desc_size / 16, ep->dma_desc_size,
		       (int *)ep->dma_buf_phys, ep->dma_buf_virt,
		       ep->dma_buf_size);

		desc = ep->dma_desc_virt;
		for (j = 0; j < DMA_DESC_NUM; j++)
			desc++;
	}
}
#endif

static int config_dma_desc(struct pxa27x_ep *ep, struct dma_txfr_t *txfr)
{
	pxa_dma_desc *desc = ep->dma_desc_virt;
	unsigned desc_addr, buf_addr = ep->dma_buf_phys, i;
	unsigned packet_size = ep->ep.maxpacket, direct = ep->dir_in;
	volatile unsigned dumy;

	desc_addr = ep->dma_desc_phys;

	for (i = 0; i < DMA_DESC_NUM; i++) {
		desc_addr += sizeof(pxa_dma_desc);

		desc->ddadr = desc_addr;
		desc->dcmd  = 0;

		if (direct) {
			/* IN */
			desc->dsadr = buf_addr;
			desc->dcmd |= U2DMACMDR_XFRDIS | U2DMACMDR_PACKCOMP;
		} else /* OUT */
			desc->dtadr = buf_addr;

		if (txfr->len > packet_size) {
			desc->ddadr &= ~U2DMADADR_STOP;
			desc->dcmd |= packet_size;
		} else {
			desc->dcmd |= txfr->len;
			desc->ddadr |= U2DMADADR_STOP;
			if (txfr->end_irq_en)
				desc->dcmd |= U2DMACMDR_ENDIRQEN;

			if (direct) {
				/* IN */
				if (txfr->len > 0) {
					/* send short package */
					/* Desc->dcmd |= U2DMACMDR_PACKCOMP; */
				} else if (txfr->is_zero)
					/* Send Zero package */
					desc->dcmd |= U2DMACMDR_PACKCOMP;
			}
		}

		/* read back to make sure ddadr have been writed
		 * before start DMA
		 */
		dumy = desc->dcmd;
		dumy = desc->ddadr;
		dumy = desc->dsadr;
		dumy = desc->dtadr;

		buf_addr += packet_size;
		desc++;

		if (txfr->len >= packet_size)
			txfr->len -= packet_size;
		else
			break;
	}

	(desc - 1)->ddadr |= U2DMADADR_STOP;
	/* read back to make sure ddadr have been writed before start DMA */
	dumy = (desc - 1)->ddadr;

	U2DMADADR(ep->ep_num) = ep->dma_desc_phys;

	return 0;
}

static int kick_dma(struct pxa27x_ep *ep, struct pxa27x_request *req)
{
	u32 len = req->req.length, csr = U2DCSR(ep->ep_num);
	char *buf = (char *)req->req.buf;
	struct dma_txfr_t txfr;
	int count = 1000;

	/* check whether FIFO is empty */
	if (ep->dir_in && check_fifo) {
		do {
			csr = U2DCSR(ep->ep_num);
			if (csr & U2DCSR_BE)
				break;
			count--;
		} while (count);

		if (count <= 0) {
			DMSG("%s, IN fifo not empty!!!\n", __FUNCTION__);
			start_watchdog(ep->dev);
			return -ENOBUFS;
		}
	}

	buf += req->req.actual;
	len -= req->req.actual;
	ep->dma_con = 0;

	U2DMACSR(ep->ep_num) &= ~U2DMACSR_RUN;
	DMSG("%s: req:0x%p, buf:%p, length:%d, actual:%d dma:%d\n",
	     __FUNCTION__, &req->req, req->req.buf, req->req.length,
	     req->req.actual, ep->dma);

	if (!(U2DCSR(ep->ep_num) & U2DCSR_DME))
		/* should be added to ep_config, FIXME */
		U2DCSR(ep->ep_num) = U2DCSR_DME;

	if (len > ep->dma_buf_size)
		ep->dma_con = 1;


	len = min(len, (u32)ep->dma_buf_size);
	ep->dma_buf_virt = req->req.buf + req->req.actual;
	ep->dma_buf_phys = virt_to_phys(ep->dma_buf_virt);

	/* init the params for transfer */
	txfr.end_irq_en = 1;
	txfr.is_zero = (req->req.zero) ? 1 : 0;
	txfr.len = len;

	config_dma_desc(ep, &txfr);

	if (ep->dir_in)
		U2DMACSR(ep->ep_num) |= U2DMACSR_RUN | U2DMACSR_STOPIRQEN;
	else
		U2DMACSR(ep->ep_num) = U2DMACSR_RUN | U2DMACSR_STOPIRQEN
				       | U2DMACSR_EORSTOPEN;

	return 0;
}

/*
 * disable the desc, clear DME for the ep
 */
static void cancel_dma(struct pxa27x_ep *ep)
{
	struct pxa27x_request *req;
	u32 tmp;

	if (U2DMACSR(ep->dma) == 0 || list_empty(&ep->queue))
		return;

	DMSG("%s,dma:%d,dcsr:0x%x\n", __FUNCTION__, ep->dma, U2DMACSR(ep->dma));
	U2DMACSR(ep->dma) = 0;

	while ((U2DMACSR(ep->dma) & U2DMACSR_STOPINTR) == 0)
		cpu_relax();

	req = list_entry(ep->queue.next, struct pxa27x_request, queue);
	tmp = U2DMACMDR(ep->dma) & U2DMACMDR_LEN;
	if (req)
		req->req.actual = req->req.length - tmp;

	/* the last tx packet may be incomplete, so flush the fifo.
	 * FIXME correct req.actual if we can
	 */
	U2DCSR(ep->ep_num) = U2DCSR_FEF | U2DCSR_TRN | U2DCSR_PC;

}

static void u2dma_handler(int dmach, void *_ep)
{
	struct pxa27x_ep *ep = _ep;
	struct pxa27x_request *req, *req_next;
	u32 dcsr, dcmd, dadr, completed, remained, tmp32;
	unsigned length, desc_num;
	unsigned long flags;
	u32 dma = ep->dma;

	local_irq_save(flags);

	req = list_entry(ep->queue.next, struct pxa27x_request, queue);

	ep->dma_irqs++;
	ep->dev->stats.irqs++;
	HEX_DISPLAY(ep->dev->stats.irqs);

	completed = 0;
	remained = req->req.length - req->req.actual;

	dcsr = U2DMACSR(dmach);
	dcmd = U2DMACMDR(dmach);
	dadr = U2DMADADR(dmach);
	U2DMACSR(dma) &= ~(U2DMACSR_RUN | U2DMACSR_STOPIRQEN);

	DMSG("%s, buf:0x%p ch:%d dcsr:%x dcmd:%x dadr:%x u2dcsr:%x dma_con%d\n",
	     __FUNCTION__, req->req.buf, dmach, dcsr, dcmd, U2DMADADR(dmach),
	     U2DCSR(dmach), ep->dma_con);
	if (dcsr & U2DMACSR_BUSERRINTR) {
		printk(KERN_ERR " Bus Error\n");
		DMSG("dcsr:%x, ddadr:%x, dsadr:%x, dtadr:%x, dcmd:%x\n",
		     U2DMACSR(dmach), U2DMADADR(dmach), U2DMASADR(dmach),
		     U2DMATADR(dmach), U2DMACMDR(dmach));
		U2DMACSR(dmach) = U2DMACSR_BUSERRINTR;
		req->req.status = -EIO;
		completed = 1;
	} else if (dcsr & U2DMACSR_STARTINTR) {
		U2DMACSR(dmach) = U2DMACSR_STARTINTR;
		goto done;
	} else if ((dcsr & U2DMACSR_STOPINTR) || (dcsr & U2DMACSR_ENDINTR)) {
		U2DMACSR(dmach) = dcsr & (U2DMACSR_ENDINTR | U2DMACSR_EORINTR);
		if (ep->dir_in) {
			/* There are still packets to transfer */
			if (ep->dma_con) {
				DMSG("dma_con%s: more packets,length:%d,"
				     "actual:%d\n", __FUNCTION__,
				     req->req.length, req->req.actual);
				req->req.actual += ep->dma_buf_size;
			} else {  /* It is whole package*/
				  /* FIXME Sent a ZLP? */
				completed = 1;
				req->req.actual = req->req.length;
				DMSG("%s: req->req.zero=%d,"
				     " req->req.length=%d\n",
				     __FUNCTION__, req->req.zero,
				     req->req.length);
				if (req->req.zero && (req->req.length %
						      ep->ep.maxpacket) == 0) {
					/* ZLP needed */
					int count = 0;
					/*Wait for packet out */
					while ((count++ < 10000) &&
						!(U2DCSR(ep->ep_num) &
							U2DCSR_BNF));
					if (count >= 10000)
						DMSG("%s,Failed to send ZLP\n",
							__func__);
					else
						DMSG("%s: send ZLP\n",
							__func__);
					U2DCSR(ep->ep_num) = U2DCSR_SP |
						U2DCSR_PC |
						U2DCSR_TRN |
						(U2DCSR(ep->ep_num) &
							(U2DCSR_FST));
					req->req.zero = 0;
				}
			}
		} else {  /* OUT */
			if (ep->dma_con) {
				if (cpu_is_pxa310()) {
#ifdef USE_SPEOREN
					req->req.actual += ep->dma_buf_size;
#endif
				} else {
					req->req.actual += ep->ep.maxpacket;
				}

				if (dcsr & U2DMACSR_ENDINTR)
					goto irq_done;

				U2DMADADR(dma) = dadr;
				U2DMACSR(dma) |= U2DMACSR_RUN
						 | U2DMACSR_STOPIRQEN
						 | U2DMACSR_EORSTOPEN;

				goto done;
			} else { /* for out endpoints */
				u32 u2dcsr = 0;

				u2dcsr = (U2DCSR_SST | U2DCSR_TRN)
					  & U2DCSR(ep->ep_num);

				/* 11.5.6.6, before clear SST, stop DMA */
				if (u2dcsr & U2DCSR_SST)
					cancel_dma(ep);

				tmp32 = U2DMACSR_EORINTR;
#ifdef USE_SPEOREN
				if (cpu_is_pxa310())
					tmp32 |= U2DMACSR_ENDINTR;
#endif
				if (dcsr & tmp32) {
					if ((U2DMACMDR(dmach) & U2DMACMDR_LEN)
					    || (dcsr & U2DMACSR_ENDINTR)) {

						/* caculate the length */
						desc_num = ((U2DMADADR(dmach)
							     & 0xfffffff0)
							    - ep->dma_desc_phys)
							   / 16;
						length = ep->ep.maxpacket
							 * desc_num
							 - (dcmd
							    & U2DMACMDR_LEN);

						completed = 1;
						req->req.actual += length;

						DMSG("\tfully data received,"
						     "len=%d, completed:%d\n",
						     req->req.actual,
						     completed);

						goto irq_done;
					}

					U2DMADADR(dma) = dadr;
					U2DMACSR(dma) |= U2DMACSR_RUN
							 | U2DMACSR_STOPIRQEN
							 | U2DMACSR_EORSTOPEN;
					DMSG("\t csr %x dcsr %x dadr %x"
					     " dcmd %x\n", U2DCSR(ep->dma),
					     U2DMACSR(dma),
					     U2DMADADR(dma),
					     U2DMACMDR(dma));
				}

				goto done;
			}
		}
	} else
		DMSG("%s: Others dma:%d DCSR:0x%x DCMD:0x%x\n",
		     __FUNCTION__, dmach, U2DMACSR(dmach), U2DMACMDR(dmach));

irq_done:
	if (likely(completed)) {
		if (req->queue.next != &ep->queue) {
			req_next = list_entry(req->queue.next,
					      struct pxa27x_request, queue);
			kick_dma(ep, req_next);
		}
		done(ep, req, 0);
	} else
		kick_dma(ep, req);

done:
	local_irq_restore(flags);
}

/*-------------------------------------------------------------------------*/

int pxa3xx_ep_queue(struct usb_ep *_ep,
		    struct usb_request *_req, unsigned gfp_flags)
{
	struct pxa27x_ep *ep;
	struct pxa27x_request *req;
	struct pxa27x_udc *dev;
	unsigned long flags;
	int count = 1000;
	u32 csr;

	req = container_of(_req, struct pxa27x_request, req);
	if (unlikely(!_req || !_req->complete || !_req->buf ||
		     !list_empty(&req->queue))) {
		DMSG("%s, bad params\n", __FUNCTION__);
		return -EINVAL;
	}

	ep = container_of(_ep, struct pxa27x_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		DMSG("%s, bad ep\n", __FUNCTION__);
		return -EINVAL;
	}

	DMSG("%s, ep point %d is queue\n", __FUNCTION__, ep->ep_num);

	dev = ep->dev;
	if (unlikely(!dev->driver
		     || ((dev->cdev.ep0state != EP0_IN_FAKE)
			 && (dev->gadget.speed == USB_SPEED_UNKNOWN)))) {
		DMSG("%s, bogus device state\n", __FUNCTION__);
		return -ESHUTDOWN;
	}

	/* iso is always one packet per request, that's the only way
	 * we can report per-packet status.  that also helps with dma.
	 */
	if (unlikely(ep->ep_type == USB_ENDPOINT_XFER_ISOC
		     && req->req.length > le16_to_cpu
		     (ep->desc->wMaxPacketSize)))
		return -EMSGSIZE;

	/*	check whether FIFO is empty
	 */
	if (ep->dir_in && check_fifo) {
		do {
			csr = U2DCSR(ep->ep_num);
			if (csr & U2DCSR_BE)
				break;
			count--;
		} while (count);

		if (count <= 0)
			return -ENOBUFS;
	}

	if (ep->desc != 0)
		dma_cache_maint(req->req.buf, req->req.length,
				(ep->dir_in) ? DMA_TO_DEVICE : DMA_FROM_DEVICE);

	DBG(DBG_NOISY, "%s queue req %p, len %d buf %p\n",
	    _ep->name, _req, _req->length, _req->buf);

	u2d_clk_enable();

	local_irq_save(flags);

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	/* kickstart this i/o queue? */
	if (list_empty(&ep->queue) && !ep->stopped) {
		if (ep->desc == 0) {
			unsigned length = _req->length;

			switch (dev->cdev.ep0state) {
			case EP0_IN_DATA_PHASE:
				dev->stats.write.ops++;
				DMSG("%s, dev->cdev.req_config = %d\n",
				     __FUNCTION__, dev->cdev.req_config);
				if (dev->cdev.req_config) {
					if (dev->cdev.req_config > 1) {
						dev->cdev.req_config--;
						done(ep, req, 0);
						req = 0;
						break;
					}
					DMSG("ep0: set config finished,"
					     "u2dcsr0 %x\n", U2DCSR0);

					dev->cdev.req_config = 0;
					ep0_idle(dev);
					done(ep, req, 0);
					req = 0;
				} else if (write_ep0_fifo(ep, req))
					req = 0;
				break;

			case EP0_OUT_DATA_PHASE:
				dev->stats.read.ops++;
				if (dev->req_pending)
					ep0start(dev, 0, "OUT");

				if (length == 0
				    || ((U2DCSR0 & U2DCSR0_RNE) != 0
					&& read_ep0_fifo(ep, req))) {
					ep0_idle(dev);

					done(ep, req, 0);
					ep0start(dev, U2DCSR0_IPR | U2DCSR0_FTF,
						 "zero IN"); /* RNDIS */
					req = 0;
				}
				break;
			case EP0_NO_ACTION:
				ep0_idle(dev);
				req = 0;
				break;
			case EP0_IN_FAKE:
				DMSG("%s: in EP0_IN_FAKE\n", __FUNCTION__);
				dev->cdev.config_length = _req->length;
				get_fake_config(&dev->cdev, _req,
						dev->gadget.speed);
				ep0_idle(dev);
				req->req.actual = req->req.length;
				done(ep, req, 0);
				req = 0;
				break;
			default:
				DMSG("ep0 i/o, odd state %d\n", dev->cdev.ep0state);
				local_irq_restore(flags);
				return -EL2HLT;
			}
			/* either start dma or prime pio pump */
		} else if (ep->dma >= 0) {
			if ((_req->length == 0) && ep->dir_in) {
				/* ZLP */
				U2DCSR(ep->ep_num) = U2DCSR_SP | U2DCSR_DME;
				done(ep, req, 0);
				req = 0;
			} else
				kick_dma(ep, req);
			/* can the FIFO can satisfy the request immediately? */
		}

		DMSG("req:%p,ep->desc:%p,ep->dma:%d\n", req, ep->desc, ep->dma);
		if (likely(req && ep->desc)) {
			/* should be removed, only DMA related interrupt is
			 * enabled PC and SP are not handled just clear the
			 * interrupt
			 */
			if (!ep->ep_num)
				U2DICR |= U2DINT(ep->ep_num, U2DINT_SPACKETCMP
						 | U2DINT_PACKETCMP);
		}
	}

	/* pio or dma irq handler advances the queue. */
	if (likely(req))
		list_add_tail(&req->queue, &ep->queue);
	local_irq_restore(flags);

	u2d_clk_restore();
	return 0;
}


/*
 * 	nuke - dequeue ALL requests
 *  called by: stop_activity
 */
static void nuke(struct pxa27x_ep *ep, int status)
{
	struct pxa27x_request *req;
	u2d_clk_enable();

	/* called with irqs blocked */
	if (ep->dma > 0 && !ep->stopped)
		cancel_dma(ep);

	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct pxa27x_request, queue);
		done(ep, req, status);
	}
	if (ep->desc && ep->ep_num) {
		U2DMACSR(ep->ep_num) &= ~U2DMACSR_RUN;
		U2DCSR(ep->ep_num) &= ~U2DCSR_DME;
		if (ep->ep_num >= HALF_EP_NUM)
			U2DICR2 &= ~(U2DINT(ep->ep_num, U2DINT_SPACKETCMP
					    | U2DINT_PACKETCMP
					    | U2DINT_FIFOERR));
		else
			U2DICR &= ~(U2DINT(ep->ep_num, U2DINT_SPACKETCMP
					   | U2DINT_PACKETCMP
					   | U2DINT_FIFOERR));
		U2DMACSR(ep->ep_num) &= ~(U2DMACSR_STOPIRQEN |
					  U2DMACSR_EORIRQEN |
					  U2DMACSR_EORJMPEN |
					  U2DMACSR_EORSTOPEN |
					  U2DMACSR_RASIRQEN);
	}
	u2d_clk_restore();
}


/* dequeue JUST ONE request */
static int pxa3xx_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct pxa27x_ep *ep;
	struct pxa27x_request *req;
	unsigned long flags;

	u2d_clk_enable();
	ep = container_of(_ep, struct pxa27x_ep, ep);
	if (!_ep || ep->ep.name == ep0name)
		return -EINVAL;

	local_irq_save(flags);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		local_irq_restore(flags);
		return -EINVAL;
	}

	if (ep->dma >= 0 && ep->queue.next == &req->queue && !ep->stopped) {
		cancel_dma(ep);
		done(ep, req, -ECONNRESET);
		/* restart i/o */
		if (!list_empty(&ep->queue)) {
			req = list_entry(ep->queue.next,
					 struct pxa27x_request, queue);
			kick_dma(ep, req);
		}
	} else
		done(ep, req, -ECONNRESET);

	local_irq_restore(flags);
	u2d_clk_restore();
	return 0;
}

/*-------------------------------------------------------------------------*/

static int pxa3xx_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct pxa27x_ep *ep;
	unsigned long flags;

	ep = container_of(_ep, struct pxa27x_ep, ep);
	if (unlikely(!_ep
		     || (!ep->desc && ep->ep.name != ep0name))
	    || ep->ep_type == USB_ENDPOINT_XFER_ISOC) {
		DMSG("%s, bad ep\n", __FUNCTION__);
		return -EINVAL;
	}
	if (value == 0) {
		DMSG("only host can clear %s halt\n", _ep->name);
		return -EROFS;
	}

	if (!list_empty(&ep->queue) && ep->dir_in) {
		DMSG("%s, -EAGAIN\n", __FUNCTION__);
		return -EAGAIN;
	}

	u2d_clk_enable();
	local_irq_save(flags);

	/* ep0 needs special care, may not necessary for U2D */
	if (!ep->desc) {
		U2DCSR0 |= U2DCSR0_FTF;
		U2DCSR0 |= U2DCSR0_FST;
		WRITE_COMPLETE(&U2DCSR0);
		/* start_watchdog(ep->dev);	disabled for U2D test */
		ep->dev->req_pending = 0;
		ep->dev->cdev.ep0state = EP0_STALL;
		LED_EP0_OFF;

		/* and bulk/intr endpoints like dropping stalls too */
	} else {
		unsigned i;

		/* flush endpoint FIFO & force STALL */
		U2DCSR(ep->ep_num) = (U2DCSR(ep->ep_num)
				      & (U2DCSR_DME | U2DCSR_FST))
				     | U2DCSR_FEF | U2DCSR_FST;

		for (i = 0; i < 1000; i += 20) {
			if (U2DCSR(ep->ep_num) & U2DCSR_SST)
				break;

			udelay(20);
		}
	}
	local_irq_restore(flags);

	u2d_clk_restore();
	DBG(DBG_VERBOSE, "%s halt\n", _ep->name);
	return 0;
}

static int pxa3xx_ep_fifo_status(struct usb_ep *_ep)
{
	struct pxa27x_ep        *ep;

	ep = container_of(_ep, struct pxa27x_ep, ep);
	if (!_ep) {
		DMSG("%s, bad ep\n", __FUNCTION__);
		return -ENODEV;
	}
	/* pxa can't report unclaimed bytes from IN fifos */
	if (ep->dir_in)
		return -EOPNOTSUPP;
	if (ep->dev->gadget.speed == USB_SPEED_UNKNOWN
	    || (U2DCSR(ep->ep_num) & U2DCSR_FS) == 0)
		return 0;
	else
		return (U2DBCR(ep->ep_num) & 0xfff) + 1;
}

static void pxa3xx_ep_fifo_flush(struct usb_ep *_ep)
{
	struct pxa27x_ep        *ep;

	ep = container_of(_ep, struct pxa27x_ep, ep);
	if (!_ep || ep->ep.name == ep0name || !list_empty(&ep->queue)) {
		DMSG("%s, bad ep\n", __FUNCTION__);
		return;
	}

	/* toggle and halt bits stay unchanged */

	/* most IN status is the same, but ISO can't stall */
	/**ep->reg_u2dcsr = UDCCSR_PC|UDCCSR_FST|UDCCSR_TRN
	 | (ep->ep_type == USB_ENDPOINT_XFER_ISOC)
	? 0 : UDCCSR_SST;*/

	u2d_clk_enable();
	/* see above, any necessary ops for ISO eps */
	U2DCSR(ep->ep_num) = (U2DCSR(ep->ep_num) & (U2DCSR_DME | U2DCSR_FST))
			     | U2DCSR_FEF | U2DCSR_TRN | U2DCSR_PC;
	u2d_clk_restore();

}


static struct usb_ep_ops pxa3xx_ep_ops = {
	.enable		= pxa3xx_ep_enable,
	.disable	= pxa3xx_ep_disable,

	.alloc_request	= pxa3xx_ep_alloc_request,
	.free_request	= pxa3xx_ep_free_request,

	.queue		= pxa3xx_ep_queue,
	.dequeue	= pxa3xx_ep_dequeue,

	.set_halt	= pxa3xx_ep_set_halt,
	.fifo_status	= pxa3xx_ep_fifo_status,
	.fifo_flush	= pxa3xx_ep_fifo_flush,
};


/* ---------------------------------------------------------------------------
 * 	device-scoped parts of the api to the usb controller hardware
 * ---------------------------------------------------------------------------
 */

static int pxa3xx_u2d_get_frame(struct usb_gadget *_gadget)
{
	return U2DFNR & 0x7FF;
}

static int pxa3xx_u2d_wakeup(struct usb_gadget *_gadget)
{
	struct pxa27x_udc *dev;

	u2d_clk_enable();
	dev = container_of(_gadget, struct pxa27x_udc, gadget);
	/* if remote wakeup is not enabled, call SRP */
	if ((U2DCR & U2DCR_DWRE)) {
		U2DCR = (U2DCR & U2DCR_UDE) | U2DCR_UDR;
#ifdef USE_SPEOREN
		if (cpu_is_pxa310())
			U2DCR |=  U2DCR_SPEOREN;
#endif
	}
	u2d_clk_restore();
	return 0;
}

static int pxa3xx_u2d_vbus_session(struct usb_gadget *_gadget, int is_active)
{
	struct pxa27x_udc *dev;

	u2d_clk_enable();
	dev = container_of(_gadget, struct pxa27x_udc, gadget);
	if (is_active)
		__u2d_enable(dev);
	else
		__u2d_disable(dev);
	udc_stop(&dev->cdev, &dev->gadget, dev->driver, !is_active);
	u2d_clk_restore();
	return 0;
}

static int pxa3xx_u2d_pullup(struct usb_gadget *gadget, int is_on)
{
	u2d_clk_enable();
	if(is_on)
		U2DCR &= ~U2DCR_ADD;
	else
		U2DCR |= U2DCR_ADD;
	u2d_clk_restore();
	return 0;
}

static const struct usb_gadget_ops pxa3xx_u2d_ops = {
	.get_frame              = pxa3xx_u2d_get_frame,
	.wakeup                 = pxa3xx_u2d_wakeup,
	/* current versions must always be self-powered */
	.vbus_session           = pxa3xx_u2d_vbus_session,
	.pullup                 = pxa3xx_u2d_pullup,
};

#ifdef DEBUG
static void gadget_info_dump(void);
#endif
/*-------------------------------------------------------------------------*/

#ifdef U2D_PROC_FILE



static const char proc_node_name [] = "driver/u2d";
static const char none [] = "none";

static int
u2d_proc_read(char *page, char **start, off_t off, int count,
	      int *eof, void *_dev)
{
	char			*buf = page;
	struct pxa27x_udc *dev = _dev;
	char			*next = buf;
	unsigned size = count;
	unsigned long flags;
	int i, t;
	u32 tmp, u2dicr2 = 0, u2disr2 = 0;

	char		*name = (char *)none;

	if (dev->driver)
		name = (char *)dev->driver->driver.name;

	if (off != 0)
		return 0;

	local_irq_save(flags);

	if (!dev->driver || !(CKENA & (1 << CKEN_USB2))) {
		t = scnprintf(next, size, "no gadget driver,"
			      " or clock is disabled\n");
		size -= t;
		next += t;
		goto done;
	}

	/* basic device status */
	t = scnprintf(next, size, DRIVER_DESC "\n"
		      "%s version: %s\nGadget driver: %s, speed: %s\n",
		      driver_name, DRIVER_VERSION SIZE_STR DMASTR,
		      (dev->cdev.driver_count > 1) ? "(composite)" : name,
		      (dev->gadget.speed == USB_SPEED_HIGH) ? "high" : "full");

	size -= t;
	next += t;

	if (cpu_is_pxa310()) {
		u2dicr2 = U2DICR2;
		u2disr2 = U2DISR2;
	}
	/* registers for device and ep0 */
	t = scnprintf(next, size,
		      "uicr %02X uicr2 %02X, uisr %02X uisr2 %02X, ufnr %02X\n",
		      U2DICR, u2dicr2, U2DISR, u2disr2, U2DFNR);

	size -= t;
	next += t;

	tmp = U2DCR;
	t = scnprintf(next, size, "u2dcr %02X =%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s,"
		      "con=%d,inter=%d,altinter=%d\n", tmp,
		      (tmp & U2DCR_NDC) ? " ndc" : "",
		      (tmp & U2DCR_HSTC_MASK) ? " hstc" : "",
		      (tmp & U2DCR_SPEOREN) ? " speoren" : "",
		      (tmp & U2DCR_FSTC_MASK) ? " fstc" : "",
		      (tmp & U2DCR_UCLKOVR) ? " uclkovr" : "",
		      (tmp & U2DCR_ABP) ? " abp" : "",
		      (tmp & U2DCR_ADD) ? " add" : "",
		      (tmp & U2DCR_CC) ? " cc" : "",
		      (tmp & U2DCR_HS) ? " hs" : "",
		      (tmp & U2DCR_DWRE) ? " dwre" : "",
		      (tmp & U2DCR_SMAC) ? " smac" : "",
		      (tmp & U2DCR_EMCE) ? " emce" : "",
		      (tmp & U2DCR_UDR) ? " udr" : "",
		      (tmp & U2DCR_UDA) ? " uda" : "",
		      (tmp & U2DCR_UDE) ? " ude" : "",
		      (tmp & U2DCR_ACN) >> U2DCR_ACN_S,
		      (tmp & U2DCR_AIN) >> U2DCR_AIN_S,
		      (tmp & U2DCR_AAISN) >> U2DCR_AAISN_S);

	size -= t;
	next += t;

	if (cpu_is_pxa310()) {
		tmp = U2DOTGCR;
		t = scnprintf(next, size, "u2dotgcr %02X =%s%s%s%s%s%s%s%s%s%s,"
			      "xcvr_mode %d, ulpi_dat3_irq_en %d\n", tmp,
			      (tmp & U2DOTGCR_OTGEN) ? " otgen" : "",
			      (tmp & U2DOTGCR_AALTHNP) ? " aalthnp" : "",
			      (tmp & U2DOTGCR_AHNP) ? " ahnp" : "",
			      (tmp & U2DOTGCR_BHNP) ? " bhnp" : "",
			      (tmp & U2DOTGCR_CKAF) ? " ckaf" : "",
			      (tmp & U2DOTGCR_UTMID) ? " utmid" : "",
			      (tmp & U2DOTGCR_ULAF) ? " ulaf" : "",
			      (tmp & U2DOTGCR_SMAF) ? " smaf" : "",
			      (tmp & U2DOTGCR_RTSM) ? " rtsm" : "",
			      (tmp & U2DOTGCR_ULE) ? " ule" : "",
			      dev->xv_ops->otgx_get_mode(),
			      dev->stp_gpio_irq_en);

		size -= t;
		next += t;

		/* registers for device and ep0 */
		t = scnprintf(next, size,
			      "otgicr %08X otgisr %08X otgucr %08X otgusr %08X"
			      "p3cr %08X\n",
			      U2DOTGICR, U2DOTGISR, U2DOTGUCR, U2DOTGUSR,
			      U2DP3CR);

		size -= t;
		next += t;
	}

	if (cpu_is_pxa930()) {
		tmp = U2DOTGCR;
		t = scnprintf(next, size, "u2dotgcr %02X =%s%s%s%s%s%s,"
			      " xcvr_mode %d\n", tmp,
			      (tmp & U2DOTGCR_LPA) ? " lpa" : "",
			      (tmp & U2DOTGCR_IESI) ? " iesi" : "",
			      (tmp & U2DOTGCR_ISSI) ? " issi" : "",
			      (tmp & U2DOTGCR_UTMID) ? " utmid" : "",
			      (tmp & U2DOTGCR_RTSM) ? " rtsm" : "",
			      (tmp & U2DOTGCR_ULE) ? " ule" : "",
			      dev->xv_ops->otgx_get_mode());

		size -= t;
		next += t;

		/* registers for device and ep0 */
		t = scnprintf(next, size,
			      "otgucr %08X\n", U2DOTGUCR);
		size -= t;
		next += t;
	}

	tmp = U2DCSR0;
	t = scnprintf(next, size,
		      "u2dcsr0 %02X =%s%s%s%s%s%s%s%s%s\n", tmp,
		      (tmp & U2DCSR0_IPA) ? " ipa" : "",
		      (tmp & U2DCSR0_SA) ? " sa" : "",
		      (tmp & U2DCSR0_RNE) ? " rne" : "",
		      (tmp & U2DCSR0_FST) ? " fst" : "",
		      (tmp & U2DCSR0_SST) ? " sst" : "",
		      (tmp & U2DCSR0_DME) ? " dme" : "",
		      (tmp & U2DCSR0_FTF) ? " ftf" : "",
		      (tmp & U2DCSR0_IPR) ? " ipr" : "",
		      (tmp & U2DCSR0_OPC) ? " opc" : "");
	size -= t;
	next += t;

	if (!dev->driver)
		goto done;

	t = scnprintf(next, size, "ep0 IN %lu/%lu, OUT %lu/%lu\nirqs %lu\n\n",
		      dev->stats.write.bytes, dev->stats.write.ops,
		      dev->stats.read.bytes, dev->stats.read.ops,
		      dev->stats.irqs);
	size -= t;
	next += t;

	/* dump endpoint queues */
	for (i = 0; i < UDC_LG_EP_NUM; i++) {
		struct pxa27x_ep *ep = &dev->ep [i];
		struct pxa27x_request *req;
		int t;

		if (i != 0) {
			const struct usb_endpoint_descriptor	*d;

			d = ep->desc;
			if (!d)
				continue;
			tmp = U2DCSR(ep->ep_num);
			t = scnprintf(next, size,
				"%s %d max %d %d u2dcsr %02x u2dcr:0x%x,"
				"u2denr:0x%x\n",
				ep->ep.name, ep->ep_num,
				le16_to_cpu(d->wMaxPacketSize),
				ep->dma, tmp,
				U2DEPCR(ep->ep_num), U2DEN(ep->ep_num));
#ifdef CONFIG_USB_COMPOSITE
			{
			struct pxa3xx_comp_ep *comp_ep;
			comp_ep = find_ep_num(dev->cdev.first_ep, i);
			size -= t;
			next += t;
			t = scnprintf(next, size, " intf=%d(%d)\n",
				comp_ep->assigned_interface,
				comp_ep->interface);
			}
#endif

			/* TODO translate all five groups of u2dcs bits! */

		} else /* ep0 should only have one transfer queued */
			t = scnprintf(next, size, "ep0 max 64 pio irqs %lu,"
				      "u2den0:0x%x\n",
				      ep->pio_irqs, U2DEN0);
		if (t <= 0 || t > size)
			goto done;
		size -= t;
		next += t;

		/* DMA desc chain */
		t = scnprintf(next, size,
			      "\tu2dmacsr %x desc %p(%p) num %d(%dB) buf %p(%p)"
			      " size %uB\n", U2DMACSR(ep->dma),
			      (int *)ep->dma_desc_phys, ep->dma_desc_virt,
			      ep->dma_desc_size / 16, ep->dma_desc_size,
			      (int *)ep->dma_buf_phys, ep->dma_buf_virt,
			      ep->dma_buf_size);
		if (t <= 0 || t > size)
			goto done;
		size -= t;
		next += t;

		/* req queue */
		if (list_empty(&ep->queue)) {
			t = scnprintf(next, size, "\t(nothing queued)\n");
			if (t <= 0 || t > size)
				goto done;
			size -= t;
			next += t;
			continue;
		}
		list_for_each_entry(req, &ep->queue, queue) {
			if (ep->dma >= 0 && req->queue.prev == &ep->queue)
				t = scnprintf(next, size,
					      "\treq %p len %d/%d "
					      "buf %p \n "
					      "\t(dma%d csr:%08x cmd:%08x"
					      " da:%08x sa:%08x ta:%08x)\n",
					      &req->req, req->req.actual,
					      req->req.length, req->req.buf,
					      ep->dma, U2DMACSR(ep->dma),
					      U2DMACMDR(ep->dma),
					      U2DMADADR(ep->dma),
					      U2DMASADR(ep->dma),
					      U2DMATADR(ep->dma));
			else
				t = scnprintf(next, size,
					      "\treq %p len %d/%d buf %p\n",
					      &req->req, req->req.actual,
					      req->req.length, req->req.buf);
			if (t <= 0 || t > size)
				goto done;
			size -= t;
			next += t;
		}
	}

	t = scnprintf(next, size, "vbus level %x\n",
		      is_cable_attached());
	size -= t;
	next += t;

done:
	local_irq_restore(flags);
	*eof = 1;
	return count - size;
}

static int u2d_proc_write(struct file *filp, const char *buffer,
			  unsigned long count, void *data)
{
	char kbuf[8];
	int index;
	struct pxa27x_udc *dev = the_controller;

	if (count >= 8)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;
	index = (int)simple_strtoul(kbuf, NULL, 10);

	if (!dev->driver) {
		printk(KERN_ERR "no gadget driver, or clock is disabled\n");
		return -EINVAL;
	}

	switch (index) {
	default:
		return -EINVAL;
	}
	return count;
}

#ifndef CONFIG_PROC_FS
#define create_proc_files() \
	create_proc_read_entry(proc_node_name, 0, NULL, udc_proc_read, dev)
#else
#define create_proc_files() \
	do {	struct proc_dir_entry *ent; \
		ent = create_proc_entry(proc_node_name, 0, NULL); \
		if (ent) { \
			ent->data = dev; \
			ent->read_proc = u2d_proc_read; \
			ent->write_proc = u2d_proc_write; \
		} \
	} while (0);
#endif
#define remove_proc_files() \
	remove_proc_entry(proc_node_name, NULL)

#else	/* !U2D_PROC_FILE */
#define create_proc_files() do { } while (0)
#define remove_proc_files() do { } while (0)

#endif	/* U2D_PROC_FILE */

/* the simply ops to assign and release the physical eps
 */
static u32 phy_ep[PXA_U2D_EP_NUM];
static void init_ep_assign(void)
{
	phy_ep[0] = 1;
}

int assign_ep(int lg_ep)
{
	struct pxa27x_udc *dev = the_controller;
	struct pxa27x_ep *ep = &dev->ep[lg_ep];
	u32 i = 0;

	while (++i < u2d_ep_num)
		if (!phy_ep[i]) {
			phy_ep[i] = lg_ep;
			list_del(&ep->ep.ep_list);
			list_add_tail(&ep->ep.ep_list, &dev->used_ep_list);
			break;
		}
	if (i == u2d_ep_num)
		i = -1;
	return i;
}

static void release_ep(int lg_ep)
{
	struct pxa27x_udc *dev = the_controller;
	struct pxa27x_ep *ep = &dev->ep[lg_ep];

	if (lg_ep != 0) {
		phy_ep[ep->ep_num] = 0;
		INIT_LIST_HEAD(&ep->queue);
		list_del(&ep->ep.ep_list);
		list_add_tail(&ep->ep.ep_list, &dev->unused_ep_list);
	}
}

static int get_lg_ep(int phy_ep_num)
{
	return phy_ep[phy_ep_num];
}

static void set_ep(struct usb_endpoint_descriptor *ep_desc, int ep_num,
		   int config, int interface, int alt)
{
	struct pxa27x_udc *dev = the_controller;
	int k = ep_num;

	dev->ep[k].desc = ep_desc;
	dev->ep[k].assigned = 1;
	dev->ep[k].pio_irqs = dev->ep[k].dma_irqs = 0;
	dev->ep[k].dma = -1;
	if (!(ep_desc->wMaxPacketSize))
		ep_desc->wMaxPacketSize = dev->ep[k].ep.maxpacket;
	else
		dev->ep[k].ep.maxpacket = ep_desc->wMaxPacketSize;
	dev->ep[k].dir_in = (ep_desc->bEndpointAddress & USB_DIR_IN) ? 1 : 0;
	dev->ep[k].ep_type = ep_desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	dev->ep[k].stopped = 1;
	dev->ep[k].dma_con = 0;
	dev->ep[k].dma = dev->ep[k].ep_num;
	dev->ep[k].aisn = alt;
	comp_set_ep(&dev->cdev, k, config, interface);
}

/* set_eps
 * assign a physical ep to logic one, and
 * fill pxa_ep structure with their configuration, interface, alternate
 * settings, assigned interface number
 */
int set_eps(__u8 num_eps, struct usb_endpoint_descriptor *p_ep_desc, int len,
		int config, int interface, int alt)
{
	struct usb_endpoint_descriptor *ep_desc = p_ep_desc;
	struct pxa27x_udc *dev = the_controller;
	int ep_desc_length = len;
	int j, k, ret;

	DMSG("  ----%s----\n", __FUNCTION__);
	for (j = 0; j < num_eps; j++) {
		/* find the ep */
		ret = get_extra_descriptor((char *)p_ep_desc, ep_desc_length,
			USB_DT_ENDPOINT, (void **) &ep_desc);
		if (ret >= 0) {
			/* compare with the ep in pxa27x, if match,
			 * fill the config, interface and asin number fields */
			for (k = 1; k < 16; k++) {
				if (k == (ep_desc->bEndpointAddress & 0x0f)) {
					dev->ep[k].ep_num = assign_ep(k);
					if (dev->ep[k].ep_num < 0) {
						pr_err("no free ep!\n");
						return -1;
					}
					set_ep(ep_desc, k, config, interface,
					       alt);
					break;
				}
			}
		} else {
			DMSG("  ep desc not find, ep_desc_length=0x%x,"
			     " p_ep_desc=0x%x\n",
			     ep_desc_length, (int)p_ep_desc);
			return -EFAULT;
		}

		ep_desc_length -= (int)ep_desc - (int)p_ep_desc +
				  ep_desc->bLength;
		p_ep_desc = (struct usb_endpoint_descriptor *)
			    ((unsigned)ep_desc + ep_desc->bLength);
	}/* for(j=0;j<num_eps;j++) */
	return 0;
}

#ifdef PXA_EP_AUTO
extern struct usb_ep * __init usb_ep_autoconfig (
	struct usb_gadget		*gadget,
	struct usb_endpoint_descriptor	*desc
);

struct usb_ep* pxa3xx_ep_autoconfig(struct usb_gadget *gadget,
		struct usb_endpoint_descriptor *desc, int config, int interface, int alt)
{
	struct usb_ep *ep;
	struct pxa27x_udc *dev = the_controller;
	int k, ret;

	printk("%s: config %d, infterface %d, alt %d\n", __func__, config, interface, alt);
	ep = usb_ep_autoconfig(gadget, desc);
	ret = get_extra_descriptor((char *)desc, sizeof(struct usb_endpoint_descriptor),
		USB_DT_ENDPOINT, (void **) &desc);
	if (ret >= 0) {
		/* compare with the ep in pxa27x, if match,
		 * fill the config, interface and asin number fields */
		for (k = 1; k < 16; k++) {
			if (k == (desc->bEndpointAddress & 0x0f)) {
				dev->ep[k].ep_num = assign_ep(k);
				if (dev->ep[k].ep_num < 0) {
					pr_err("no free ep!\n");
					return NULL;
				}
				set_ep(desc, k, config, interface,
				       alt);
				break;
			}
		}
	}
	return ep;
}

#endif
/* u2d_eps_reset
 * clear the endpoint configuration and information register
 */
static void u2d_eps_reset(void)
{
	struct pxa27x_udc *dev = the_controller;
	struct pxa27x_ep *ep = NULL;
	int i;

	for (i = 1; i < UDC_LG_EP_NUM; i++) {
		ep = &dev->ep[i];
		if (ep->assigned)
			continue;
		U2DEN(ep->ep_num) = ep->ep_num;
		U2DEPCR(ep->ep_num) = 0;
	}
}

struct usb_gadget_driver *stop_udc(struct usb_gadget_driver *driver)
{
	struct pxa27x_udc *dev = the_controller;
	int i;

	dev->gadget.speed = USB_SPEED_UNKNOWN;

	/* prevent new request submissions, kill any outstanding requests  */
	for (i = 0; i < UDC_LG_EP_NUM; i++) {
		struct pxa27x_ep *ep = &dev->ep[i];
		if (ep->assigned) {
			ep->stopped = 1;
			nuke(ep, -ESHUTDOWN);
		}
	}
	del_timer_sync(&dev->timer);

	return driver;
}

/* u2d_eps_config
 * set the endpoint configuration and information register
 */
static int u2d_eps_config(int phase)
{
	struct pxa27x_udc *dev = the_controller;
	struct pxa27x_ep *ep = NULL;
	unsigned config_num, intf_num, i;

	for (i = 1; i < UDC_LG_EP_NUM; i++) {
		if (!dev->ep[i].assigned)
			continue;

		ep = &dev->ep[i];
		config_num = comp_calc_config(&dev->cdev, i);
		intf_num = comp_calc_interface(&dev->cdev, i);

		if (phase == 1)
			U2DEPCR(ep->ep_num) = (ep->fifo_size >> 2) | U2DEPCR_EE;
		else if (phase == 2) {
			if (u2d_bugs & U2D_BUG_SETINTF) {
				intf_num = 0;/* FIXME rndis, cdc acm */
			}
			U2DEN(ep->ep_num) = (i << 0) | (ep->dir_in << 4)
					    | (ep->ep_type << 5)
					    | (config_num << 7)
					    | (intf_num << 11)
					    | (ep->aisn << 15)
					    | (ep->ep.maxpacket << 19)
					    | (ep->hs_cmds << 30);
		}
	}

	return 0;
}

/*
 * 	__u2d_disable - disable USB device controller
 */
static void __u2d_disable(struct pxa27x_udc *dev)
{
	u32 u2dotgicr;

	u2d_clk_enable();

	if (!(U2DCR & U2DCR_UDE))
		goto done;

	U2DCR &= ~U2DCR_UDE;

	/* clear U2D interrupts, include endpoints and U2DMAs */
	U2DICR = 0x00000000;
	U2DISR = 0xfeffffff;

	if (cpu_is_pxa310()) {
		U2DICR2 = 0x00000000;
		U2DISR2 = 0x00ffffff;

		u2dotgicr = U2DOTGICR;
		U2DOTGICR = 0x00000000;

		U2DOTGCR &= ~U2DOTGCR_ULE;
#ifdef CONFIG_USB_OTG_PXA3xx_U2D
		U2DOTGCR = U2DOTGCR_UTMID | U2DOTGCR_OTGEN;
#else
		U2DOTGCR = U2DOTGCR_UTMID;
#endif
		U2DOTGCR |= U2DOTGCR_ULE;
		U2DOTGICR = u2dotgicr;
	} else if (cpu_is_pxa930()) {
		U2DOTGCR &= ~(U2DOTGCR_IESI);
		U2DOTGCR &= ~U2DOTGCR_ULE;
		U2DOTGCR = U2DOTGCR_UTMID;
		U2DOTGCR |= U2DOTGCR_ULE;
	}

	ep0_idle(dev);
	dev->gadget.speed = USB_SPEED_UNKNOWN;
	comp_val_init(&dev->cdev);

done:
	u2d_clk_restore();
}

/*
 * 	udc_reinit - initialize software state
 */
void udc_reinit(void)
{
	struct pxa27x_udc *dev = the_controller;
	u32 i;

	u2d_clk_enable();
	dev->cdev.ep0state = EP0_IDLE;

	/* basic endpoint records init */
	for (i = 0; i < UDC_LG_EP_NUM; i++) {
		struct pxa27x_ep *ep = &dev->ep[i];

		ep->stopped = 0;
		ep->pio_irqs = ep->dma_irqs = 0;
	}

	dev->cdev.configuration = 0;
	dev->cdev.interface = 0;
	dev->cdev.alternate = 0;
	/* the rest was statically initialized, and is read-only */
	u2d_clk_restore();
}

/* until it's enabled, this U2D should be completely invisible
 * to any USB host.
 */
static void __u2d_enable(struct pxa27x_udc *dev)
{
	int i;

	u2d_clk_enable();
	if (U2DCR & U2DCR_UDE)
		goto done;

	DMSG("%s\n", __func__);

	ep0_idle(dev);
	/* default speed, or should be unknown here? */
	dev->gadget.speed = USB_SPEED_UNKNOWN;
	dev->stats.irqs = 0;

	/* check whether U2DCR[EMCE]=1 */
	if (U2DCR_EMCE & U2DCR)
		pr_err("%s, Endpoint Memory Configuration Error\n", __func__);

	/* enable suspend/resume and reset irqs */
	U2DICR = U2DINT_CC | U2DINT_RU | U2DINT_SU | U2DINT_RS | U2DINT_DPE;

	/* enable ep0 irqs */
	U2DICR |= U2DINT(0, U2DINT_PACKETCMP);
	U2DICR |= U2DINT(0, U2DINT_SPACKETCMP);

	U2DCSR0 = U2DCSR0_FTF;
	WRITE_COMPLETE(&U2DCSR0);

	U2DEN0 = 0x2000000;
	for (i = 1; i < u2d_ep_num; i++)
		U2DEN(i) = i;
	DMSG("%s: U2DCR = 0x%x, U2DEN0(%p) = 0x%x, U2DICR = 0x%x\n",
	     __FUNCTION__, U2DCR, &U2DEN0, U2DEN0, U2DICR);

#ifdef EP0_OUT_DMA
	/* configure the DMA for OUT packets through EP0 */

	/* disable  */
	U2DICR &= ~(U2DINT(0, U2DINT_PACKETCMP));

#else
	/* receive FIFO not empty? */
	if (U2DCSR0 & U2DCSR0_RNE) {
		DMSG("%s, receive FIFO not empty? U2DCSR0=%x\n", __FUNCTION__,
		     U2DCSR0);
		if (U2DCSR0 & U2DCSR0_SA)
			U2DCSR0 = U2DCSR0_SA | U2DCSR0_OPC;
		else
			U2DCSR0 = U2DCSR0_OPC;
	}
	WRITE_COMPLETE(&U2DCSR0);
#endif	/* EP0_OUT_DMA */

	/* enable U2D */
	U2DCR = U2DCR_UDE;
#ifdef USE_SPEOREN
	if (cpu_is_pxa310())
		U2DCR |= U2DCR_SPEOREN;
#endif
	/* 
	 * workaround of silicon issue of U2D
	 * U2DEN(x) should be non zero, 
	 * otherwise,  mass storage failed when switch from other function, since EP0 could not get correct data
	 * */
	for (i = 1; i < u2d_ep_num; i++) {
		if(0 == U2DEN(i)){
			U2DEN(i) = i;
		}
	}
			
done:
#if defined(CONFIG_CHARGER_PXA3xx_HARDWARE) /* make a usb/ac query*/
{
	extern int u2d_enable_notify_hwcharger(void);
	u2d_enable_notify_hwcharger();
}
#endif
	u2d_clk_restore();
}

extern void otg_reset_d(voit);
/* when a driver is successfully registered, it will receive
 * control requests including set_configuration(), which enables
 * non-control requests.  then usb traffic follows until a
 * disconnect is reported.  then a host may connect again, or
 * the driver might get unbound.
 */
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct pxa27x_udc *dev = the_controller;
	int retval;

	if (!driver	|| driver->speed != USB_SPEED_HIGH
	    || !driver->bind
	    || !driver->disconnect
	    || !driver->setup
	)
		return -EINVAL;
	if (!dev)
		return -ENODEV;

	if (comp_is_dev_busy(&dev->cdev, dev->driver))
		return -EBUSY;

	/* set device constraint */
	DMSG("%s set dvfm constraint\n", __func__);
	set_dvfm_constraint();

	/* The pins of U2D are conflict with camera.
	 * When U2D gadget driver are loaded into kernel,
	 * camera should be disabled first.
	 */
	if (dev->cdev.driver_count == 0)
		if (dev->mach->reset_utmi)
			dev->mach->reset_utmi();

	if (cpu_is_pxa310())
		enable_oscc_pout();
	else if (cpu_is_pxa930())
		set_cwsbr();

	u2d_clk_set(1);

	local_irq_disable();
	__u2d_disable(dev);
	stop_gadget(&dev->cdev, &dev->gadget, dev->driver);
	local_irq_enable();

	pr_debug("register gadget driver \n");

	driver->driver.bus = NULL;

	/* first hook up the driver ... */
	dev->driver = driver;
	dev->gadget.dev.driver = &driver->driver;

	list_add_tail(&dev->used_ep_list, &dev->gadget.ep_list);
	list_del(&dev->gadget.ep_list);
	list_add_tail(&dev->gadget.ep_list, &dev->unused_ep_list);
	list_del(&dev->unused_ep_list);

	if (dev->cdev.driver_count == 0) {
		cable_detect_init();

		retval = device_add(&dev->gadget.dev);
		if (retval) {
			printk(KERN_ERR "cannot add device, ret: %d\n",
			       retval);
			goto err1;
		}

		if (u2d_bugs & U2D_FIX_ULPI_STP) {
			int irq = gpio_to_irq(dev->ulpi_int);
			retval = request_irq(irq, ulpi_dat3_irq,
					     IRQF_TRIGGER_RISING,
					     "ULPI DAT3 detect", (void *)dev);
			if (retval) {
				pr_err("ULPI: request DAT3 irq failed\n");
				return -EBUSY;
			}
			disable_irq(irq);
			dev->stp_gpio_irq_en = 0;
			dev->ulpi_dat3_work = 0;
		}
	}

	retval = driver->bind(&dev->gadget); /* will set_gadget_data */
	if (retval) {
		printk(KERN_ERR "bind to driver %s --> error %d\n",
		       driver->driver.name, retval);
		goto err2;
	}

#ifndef PXA_EP_AUTO
	comp_register_driver(&dev->cdev, &dev->gadget, driver);
#endif

	/* configure the endpoint FIFO allocation in 8K SRAM */
	u2d_eps_config(1);

	list_add_tail(&dev->unused_ep_list, &dev->gadget.ep_list);
	list_del(&dev->gadget.ep_list);
	list_add_tail(&dev->gadget.ep_list, &dev->used_ep_list);
	list_del(&dev->used_ep_list);

	check_fifo = 0;
	if (u2d_bugs & U2D_BUG_INMASS) {
		check_fifo = 1;
	}
	if (u2d_bugs & U2D_BUG_STALL)
		skip_ep_num = 4;

	if (cpu_is_pxa930()) {
		pmic_callback_register(PMIC_EVENT_USB, (void*)pxa3xx_usb_event_change);
	}

	/* ... then enable host detection and ep0; and we're ready
	 * for set_configuration as well as eventual disconnect.
	 * NOTE:  this shouldn't power up until later.
	 */
	DMSG("registered gadget driver '%s'\n", driver->driver.name);

#ifdef CONFIG_USB_OTG_PXA3xx_U2D
	if (cpu_is_pxa310())
		otg_set_peripheral(dev->transceiver, &dev->gadget);
#endif

	if (0 && is_cable_attached()) { /* Check boot cable status @ cable_detect_interrupt */
		DMSG("%s set dvfm constraint\n", __func__);
		set_dvfm_constraint();

		if (cpu_is_pxa310()) {
#ifdef CONFIG_USB_OTG_PXA3xx_U2D
			u2d_soft_dis(1);
			dev->xv_ops->otgx_set_mode(USB_OTG_LP);
			pxa3xx_otg_require_bus(USBOTG_VBUS_VALID);
#else
			dev->xv_ops->otgx_set_mode(USB_INT_OTG_CLIENT_DP);
			__u2d_enable(dev);
			u2d_soft_dis(0);
#endif
		} else if (cpu_is_pxa930()) {
			dev->xv_ops->otgx_set_mode(USB_INT_OTG_CLIENT_DP);
			__u2d_enable(dev);
		} else
			__u2d_enable(dev);
	} else {
		u2d_soft_dis(1);
		if (dev->xv_ops)
			dev->xv_ops->otgx_set_mode(USB_OTG_LP);
	}

	dev->cdev.driver_count++;

	DMSG("%s unset dvfm constraint\n", __func__);
	unset_dvfm_constraint();
	return 0;
err2:
	if (dev->cdev.driver_count == 0)
		device_del(&dev->gadget.dev);
err1:
	list_add_tail(&dev->unused_ep_list, &dev->gadget.ep_list);
	list_del(&dev->gadget.ep_list);
	list_add_tail(&dev->gadget.ep_list, &dev->used_ep_list);
	list_del(&dev->used_ep_list);

	dev->driver = 0;
	dev->gadget.dev.driver = 0;

	DMSG("%s unset dvfm constraint\n", __func__);
	unset_dvfm_constraint();

	return retval;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	int i;
	struct pxa27x_udc *dev = the_controller;

	if (!dev)
		return -ENODEV;

	if (!comp_check_driver(&dev->cdev, dev->driver, driver))
		return -EINVAL;

	DMSG("%s set dvfm constraint\n", __func__);
	set_dvfm_constraint();

	u2d_clk_set(1);

	/* clear all non-zero EPs configurations */
	u2d_eps_reset();

	local_irq_disable();
	u2d_soft_dis(1);
	mdelay(6);
 	if (dev->cdev.driver_count == 1) {
#ifdef CONFIG_USB_OTG_PXA3xx_U2D
		if (cpu_is_pxa310())
			otg_set_peripheral(dev->transceiver, NULL);
#endif
	}
	__u2d_disable(dev);
	cable_detect_deinit();

	stop_cur_gadget(&dev->cdev, &dev->gadget, driver);
	local_irq_enable();

	if (is_cable_attached()) {
		DMSG("%s unset dvfm constraint\n", __func__);
		unset_dvfm_constraint();
	}

	driver->unbind(&dev->gadget);

	for (i = 1; i < UDC_LG_EP_NUM; i++) {
		struct pxa27x_ep *ep = &dev->ep[i];
		if (ep->assigned) {
			ep->assigned = 0;
			ep->desc = NULL;
			release_ep(i);
		}
	}

	dev->cdev.driver_count--;
#ifndef PXA_EP_AUTO
	comp_unregister_driver(&dev->cdev, &dev->gadget, &dev->driver, driver);
#endif

	/* del the gadget abstract device */
	if (dev->cdev.driver_count == 0) {
		device_del(&dev->gadget.dev);
		dev->driver = 0;
		u2d_init_ep(dev);
		/* When all modules are removed, disable the USB 2.0 clock */
		u2d_clk_set(0);
		if (u2d_bugs & U2D_FIX_ULPI_STP) {
			int irq = gpio_to_irq(dev->ulpi_int);
			free_irq(irq, (void *)dev);
			/*destroy_workqueue(dev->stp_work_queue);*/
			if (dev->u2d_irq_dis) {
				enable_irq(IRQ_USB2);
				dev->u2d_irq_dis = 0;
			}
		}

		if (cpu_is_pxa930())
			clr_cwsbr();
	}

	if (cpu_is_pxa930()) {
		pmic_callback_unregister(PMIC_EVENT_USB,
					 (void*)pxa3xx_usb_event_change);
	}

	DMSG("unregistered gadget driver '%s'\n", driver->driver.name);
	/* dump_state(dev); */
	DMSG("%s unset dvfm constraint\n", __func__);
	unset_dvfm_constraint();
	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

#ifndef	enable_disconnect_irq
#define	enable_disconnect_irq()		do { } while (0)
#define	disable_disconnect_irq()	do { } while (0)
#endif


/*-------------------------------------------------------------------------*/

static inline void clear_ep_state(struct pxa27x_udc *dev)
{
	unsigned i;

	/* hardware SET_{CONFIGURATION,INTERFACE} automagic resets endpoint
	 * fifos, and pending transactions mustn't be continued in any case.
	 */
	for (i = 0; i < UDC_LG_EP_NUM; i++)
		nuke(&dev->ep[i], -ECONNABORTED);
}

/*
 * Though FST will be cleared, the U2D will continue to respond to
 * subsequent accesses to Endpoint 0 with a Stall handshake until an
 * access to Endpoint 0 occurs with a SETUP packet id (that is, until a new
 * set-up transfer is initiated.)
 */
static void u2d_watchdog(unsigned long _dev)
{
	struct pxa27x_udc *dev = the_controller;
	struct pxa27x_ep *ep = &dev->ep[1];
	struct pxa27x_request *req;
	unsigned csr, count = 1000;
	unsigned long flags;

	local_irq_save(flags);

	/*	check whether FIFO is empty
	 */
	if (ep->dir_in && check_fifo) {
		do {
			csr = U2DCSR(ep->ep_num);
			if (csr & U2DCSR_BNF)
				break;
			count--;
		} while (count);

		if (count <= 0) {
			DMSG("%s, IN fifo not empty!!!\n", __FUNCTION__);
			start_watchdog(ep->dev);
			goto done;
		}
	}

	/* kickstart this i/o queue? */
	if (list_empty(&ep->queue) && !ep->stopped) {
		req = list_entry(ep->queue.next, struct pxa27x_request, queue);
		kick_dma(ep, req);
	}
done:
	local_irq_restore(flags);
}

static void ep0_ctrl_request(struct pxa27x_udc *dev, struct pxa27x_ep *ep0)
{
	u32 u2dcsr0 = U2DCSR0;
	union {
		struct usb_ctrlrequest r;
		u8 raw [8];
		u32 word [2];
	} u;
	int i;

	nuke(ep0, -EPROTO);
	u.word [0] = 0;
	u.word [1] = 0;
	/* read SETUP packet */
	for (i = 0; i < 2; i++) {
		if (unlikely(!(U2DCSR0 & U2DCSR0_RNE))) {
bad_setup:
			DMSG("SETUP %d!, U2DBCR0 %x U2DCSR0"
			     " %x\n", i, U2DBCR0, U2DCSR0);
			goto stall;
		}
		u.word [i] =  U2DDR0;
	}
	if (unlikely((U2DCSR0 & U2DCSR0_RNE) != 0))
		goto bad_setup;

	u2dcsr0 &= ~U2DCSR0_RNE;
	/* clear OPC */
	U2DCSR0 = u2dcsr0;
	WRITE_COMPLETE(&U2DCSR0);

	le16_to_cpus(&u.r.wValue);
	le16_to_cpus(&u.r.wIndex);
	le16_to_cpus(&u.r.wLength);

	LED_EP0_ON;

	DBG(DBG_VERBOSE, "SETUP %02x.%02x v%04x i%04x l%04x\n",
	    u.r.bRequestType, u.r.bRequest,
	    u.r.wValue, u.r.wIndex, u.r.wLength);

	/* cope with automagic for some standard requests. */
	dev->req_std = (u.r.bRequestType & USB_TYPE_MASK)
		       == USB_TYPE_STANDARD;
	dev->cdev.req_config = 0;
	dev->req_pending = 1;

	if (u.r.bRequestType & USB_DIR_IN)
		dev->cdev.ep0state = EP0_IN_DATA_PHASE;
	else
		dev->cdev.ep0state = EP0_OUT_DATA_PHASE;

	if (u.r.wLength == 0)
		dev->cdev.ep0state = EP0_IN_DATA_PHASE;

	/* when only one driver is registered, do as original */
	if (dev->cdev.driver_count == 1)
		i = dev->driver->setup(&dev->gadget, &u.r);
	else if (dev->cdev.driver_count <= 0) {
		pr_err("%s, error: dev->cdev.driver_count = %d\n",
		       __FUNCTION__, dev->cdev.driver_count);
		return;
	} else
		i = comp_ep0_req(&dev->cdev, &dev->gadget, &ep0->ep, &u.r);

	if (i < 0) {
		/* hardware automagic preventing STALL... */
		if (dev->cdev.req_config) {
			/* hardware sometimes neglects to tell
			 * tell us about config change events,
			 * so later ones may fail...
			 */
			pr_warning("config change %02x fail %d?\n",
			     u.r.bRequest, i);
			return;
			/* TODO experiment:  if has_cfr,
			 * hardware didn't ACK; maybe we
			 * could actually STALL!
			 */
		}
		DBG(DBG_VERBOSE, "protocol STALL, "
		    "u2dcsr0 %02x err %d\n", U2DCSR0, i);
stall:
		/* the watchdog timer helps deal with cases
		 * where udc seems to clear FST wrongly, and
		 * then NAKs instead of STALLing.
		 * watchdog not necessary for U2D ?????
		 */
		printk(KERN_DEBUG "\t ep0 stall\n");
		ep0start(dev, U2DCSR0_FST | U2DCSR0_FTF, "stall");
		/* start_watchdog(dev); */
		dev->cdev.ep0state = EP0_STALL;
		LED_EP0_OFF;

		/* deferred i/o == no response yet */
	} else if (dev->req_pending) {
		if (likely(dev->cdev.ep0state == EP0_IN_DATA_PHASE
			   || dev->req_std || u.r.wLength))
			ep0start(dev, 0, "defer");
		else
			/* Wait for client to
			 * send 0 length ep0 request */
			ep0start(dev, 0, "defer/IPR");
	}
}

static void handle_ep0(struct pxa27x_udc *dev)
{
	u32 u2dcsr0 = U2DCSR0;
	struct pxa27x_ep *ep = &dev->ep [0];
	struct pxa27x_request *req;

	u32 i;

	DMSG("%s is called, ep0 state:%d\n", __FUNCTION__, dev->cdev.ep0state);

	if (list_empty(&ep->queue))
		req = 0;
	else
		req = list_entry(ep->queue.next, struct pxa27x_request, queue);

	/* U2D needn't clear stall status by driver,
	   it would clear STALL until next SETUP cmd */
	if (u2dcsr0 & U2DCSR0_SST) {
		nuke(ep, -EPIPE);

		DMSG("%s EP0 in STALL, ep0 state:%d\n", __FUNCTION__,
		     dev->cdev.ep0state);

		del_timer(&dev->timer);	/* maybe not necessary ????? */
		ep0_idle(dev);
	}

	/* previous request unfinished?  non-error iff back-to-back ... */
	if ((u2dcsr0 & U2DCSR0_SA) != 0 && dev->cdev.ep0state != EP0_IDLE) {
		DMSG("handle_ep0: Setup command again\n");
		nuke(ep, 0);
		del_timer(&dev->timer);
		ep0_idle(dev);
	}

	switch (dev->cdev.ep0state) {
	case EP0_NO_ACTION:
		printk(KERN_INFO "%s: Busy\n", __FUNCTION__);
		/*Fall through */
	case EP0_IDLE:
		/* late-breaking status? */
		u2dcsr0 = U2DCSR0;
		DMSG("%s EP0_IDLE u2dcsr0:0x%08x, u2dbcr 0x%x\n",
		     __FUNCTION__, u2dcsr0, U2DBCR0);
		/* start control request? */
		if (likely((u2dcsr0 & (U2DCSR0_OPC | U2DCSR0_SA | U2DCSR0_RNE))
			   == (U2DCSR0_OPC | U2DCSR0_SA | U2DCSR0_RNE))) {
			ep0_ctrl_request(dev, ep);
			/* expect at least one data or status stage irq */
			return;

		} else {
			/* some random early IRQ:
			 * - we acked FST
			 * - IPR cleared
			 * - OPC got set, without SA (likely status stage)
			 */
			U2DCSR0 = u2dcsr0 & (U2DCSR0_SA | U2DCSR0_OPC);
			WRITE_COMPLETE(&U2DCSR0);
		}
		break;
	case EP0_IN_DATA_PHASE:			/* GET_DESCRIPTOR etc */
		if (u2dcsr0 & U2DCSR0_OPC) {
			U2DCSR0 = U2DCSR0_OPC | U2DCSR0_FTF;
			WRITE_COMPLETE(&U2DCSR0);
			DBG(DBG_VERBOSE, "ep0in premature status\n");
			if (req)
				done(ep, req, 0);
			if (!dev->cdev.req_config)
				ep0_idle(dev);
			else
				pr_err("ep0in u2dcsr0 %x\n", u2dcsr0);
		} else {/* irq was IPR clearing */
			if (req)
				/* this IN packet might finish the request */
				(void)write_ep0_fifo(ep, req);
			/* else IN token before response was written */
		}
		break;
	case EP0_OUT_DATA_PHASE:		/* SET_DESCRIPTOR etc */
		i = 0;
		while ((i < 10000) && !(U2DCSR0 & U2DCSR0_OPC)) i++;
		if (U2DCSR0 & U2DCSR0_OPC) {

			if (req) {
				/* this OUT packet might finish the request */
				DMSG("%s, U2DCSR0=%x\n", __func__, U2DCSR0);
				if (read_ep0_fifo(ep, req)) {
					done(ep, req, 0);
					ep0start(dev, U2DCSR0_IPR | U2DCSR0_FTF,
						 "zero 1 IN"); /* RNDIS */
				}
				/* else more OUT packets expected */
			}       /* else OUT token before read was issued */
		} else {        /* irq was IPR clearing */
			DBG(DBG_VERBOSE, "ep0out premature status\n");
			if (req)
				done(ep, req, 0);
			ep0_idle(dev);
		}
		break;
	case EP0_STALL:
		U2DCSR0 = U2DCSR0_FST;
		WRITE_COMPLETE(&U2DCSR0);
		break;
	case EP0_IN_FAKE:
		printk(KERN_ERR "%s: impossible come here\n", __FUNCTION__);
		break;
	}
}

static void handle_ep(struct pxa27x_ep *ep)
{
	struct pxa27x_request *req, *req_next;
	int completed;
	u32 u2dcsr = 0, u2disr2 = 0;

	DMSG("%s is called, ep num:%d, in:%d\n", __FUNCTION__,
	     ep->ep_num, ep->dir_in);
	do {
		completed = 0;
		if (likely(!list_empty(&ep->queue)))
			req = list_entry(ep->queue.next,
					 struct pxa27x_request, queue);
		else
			req = 0;

		if (cpu_is_pxa310())
			u2disr2 = U2DISR2;

		DMSG("%s: req:%p, u2disr:0x%x u2disr2:0x%x u2dcsr:0x%x\n",
		     __func__, req, U2DISR, u2disr2, U2DCSR(ep->ep_num));

		if (unlikely(ep->dir_in)) {
			u2dcsr = (U2DCSR_SST | U2DCSR_TRN) & U2DCSR(ep->ep_num);

			/* 11.5.6.6, before clear SST, stop DMA */
			if (u2dcsr & U2DCSR_SST)
				cancel_dma(ep);

			if (unlikely(u2dcsr))
				U2DCSR(ep->ep_num) = u2dcsr;
		} else { /* for out endpoints */
			u2dcsr = (U2DCSR_SST | U2DCSR_TRN) & U2DCSR(ep->ep_num);

			/* 11.5.6.6, before clear SST, stop DMA */
			if (u2dcsr & U2DCSR_SST)
				cancel_dma(ep);

			/* clear SST & TRN if necessary */
			if (unlikely(u2dcsr))
				U2DCSR(ep->ep_num) = u2dcsr;

			if (likely(req)) {
				/* should not come here,
				 * tail bytes handled by U2DMA */
				if (req->queue.next != &ep->queue) {
					req_next = list_entry(req->queue.next,
							      struct pxa27x_request,
							      queue);
					kick_dma(ep, req_next);
				}
				done(ep, req, 0);
			} else {
				if (ep->ep_num >= HALF_EP_NUM) {
					U2DICR2 &= ~(U2DINT(ep->ep_num,
							    U2DINT_PACKETCMP));
					U2DICR2 &= ~(U2DINT(ep->ep_num,
							    U2DINT_SPACKETCMP));
					U2DICR2 &= ~(U2DINT(ep->ep_num,
							    U2DINT_FIFOERR));
				} else {
					U2DICR &= ~(U2DINT(ep->ep_num,
							   U2DINT_PACKETCMP));
					U2DICR &= ~(U2DINT(ep->ep_num,
							   U2DINT_SPACKETCMP));
					U2DICR &= ~(U2DINT(ep->ep_num,
							   U2DINT_FIFOERR));
				}

				U2DCSR(ep->ep_num) = (U2DCSR(ep->ep_num) &
						     (U2DCSR_DME |
						      U2DCSR_FST)) |
						     U2DCSR_FEF;

				U2DCSR(ep->ep_num) = U2DCSR_PC | U2DCSR_DME;
				DMSG("%s: no req for out data\n",
				     __FUNCTION__);
			}
		}
		ep->pio_irqs++;
	} while (completed);
}

static void pxa3xx_change_configuration(struct pxa27x_udc *dev)
{
	struct usb_ctrlrequest req ;

	req.bRequestType = 0;
	req.bRequest = USB_REQ_SET_CONFIGURATION;
	req.wValue = dev->cdev.configuration;
	req.wIndex = 0;
	req.wLength = 0;

	dev->cdev.ep0state = EP0_IN_DATA_PHASE;

	dev->cdev.req_config = dev->cdev.driver_count;
	comp_change_config(&dev->cdev, &dev->gadget, dev->driver, &req);
}

static void pxa3xx_change_interface(struct pxa27x_udc *dev)
{
	struct usb_ctrlrequest req;
	int active_interface = (U2DCR & U2DCR_AIN) >> U2DCR_AIN_S, ret;

	req.bRequestType = USB_RECIP_INTERFACE;
	req.bRequest = USB_REQ_SET_INTERFACE;
	req.wValue = dev->cdev.alternate;
	req.wIndex = dev->cdev.interface;
	req.wLength = 0;

	dev->driver = comp_change_interface(&dev->cdev, active_interface, &req,
				   &dev->gadget, dev->driver, &ret);
	dev->gadget.dev.driver = &((struct usb_gadget_driver *)
				   (dev->driver))->driver;
}

#if defined(CONFIG_PXA3xx_DVFM)
#define PXA_U2D_TIMEOUT 8*HZ
static void pxa3xx_u2d_dvfm_enable(unsigned long _dev)
{
	struct pxa27x_udc *dev = the_controller;
	if (!dev->transceiver->default_a && !is_cable_attached())
		unset_dvfm_constraint_by_force();
	else
		mod_timer(&dev->dvfm_timer, jiffies + PXA_U2D_TIMEOUT);
}
#endif

/*
 *	pxa3xx_u2d_irq - interrupt handler
 *
 * avoid delays in ep0 processing. the control handshaking isn't always
 * under software control (pxa250c0 and the pxa255 are better), and delays
 * could cause usb protocol errors.
 */
static irqreturn_t pxa3xx_u2d_irq(int irq, void *_dev)
{
	struct pxa27x_udc *dev = _dev;
	int handled;
	u32 u2disr;
	u32 otgisr, state, ep_num;
	u32 u2dint, i, temp;

#if defined(CONFIG_PXA3xx_DVFM) 
#if 0 /* mv D0CS to cable_detect_interrupt, by frank */
	set_dvfm_constraint_by_force();
	mod_timer(&dev->dvfm_timer, jiffies + PXA_U2D_TIMEOUT);
#endif
#endif
	dev->stats.irqs++;
	HEX_DISPLAY(dev->stats.irqs);


	DMSG("\n");
	DBG(DBG_VERBOSE, "Interrupt, U2DICR:0x%08x, U2DISR:0x%08x, "
	    "U2DCR:0x%08x, U2DMAINT:0x%x\n", U2DICR, U2DISR, U2DCR, U2DMAINT);

#ifdef CONFIG_USB_OTG_PXA3xx_U2D
	if (cpu_is_pxa310())
		DBG(DBG_VERBOSE, "\tU2DICR2:0x%08x U2DISR2:0x%08x U2DOTGCR"
			":0x%08x, U2DOTGICR:0x%08x, U2DOTGISR:0x%08x,\n",
			U2DICR2, U2DISR2, U2DOTGCR, U2DOTGICR, U2DOTGISR);
#endif

irq_loop:

	if (U2DMAINT & ((1 << u2d_ep_num) - 1))
		for (i = 1; i < UDC_LG_EP_NUM; i++) {
			ep_num = dev->ep[i].ep_num;
			if (dev->ep[i].assigned
			    && U2DMAINT & (0x1 << ep_num))
				u2dma_handler(ep_num, &dev->ep[i]);
		}

	u2dint = U2DISR & 0xFE000000;
	handled = 0;

	/* SUSpend Interrupt Request */
	if (unlikely(u2dint & U2DINT_SU)) {
		U2DISR = U2DINT_SU;
		temp = U2DISR;
		handled = 1;
		DBG(DBG_VERBOSE, "USB suspend,"
		    " u2dcr %x icr %x isr %x fnr %x dmacr %x int %x\n",
		    U2DCR, U2DICR, U2DISR, U2DFNR, U2DMACR, U2DMAINT);
		if (dev->gadget.speed != USB_SPEED_UNKNOWN && dev->driver)
			comp_driver_suspend(&dev->cdev, &dev->gadget, dev->driver);
		ep0_idle(dev);

#ifdef CONFIG_USB_OTG_PXA3xx_U2D
		if (cpu_is_pxa310()) {
			otg_host_suspend(dev->transceiver);
			goto irq_loop;
		}
#endif
	}

	/* RESume Interrupt Request */
	if (unlikely(u2dint & U2DINT_RU)) {
		U2DISR = U2DINT_RU;
		temp = U2DISR;
		handled = 1;
		DBG(DBG_VERBOSE, "USB resume,"
		    " u2dcr %x icr %x isr %x fnr %x dmacr %x int %x\n",
		    U2DCR, U2DICR, U2DISR, U2DFNR, U2DMACR, U2DMAINT);

		if (dev->gadget.speed != USB_SPEED_UNKNOWN && dev->driver)
			comp_driver_resume(&dev->cdev, &dev->gadget, dev->driver);
	}

	if (cpu_is_pxa310())
		if (U2DOTGISR) {
#ifdef CONFIG_USB_OTG_PXA3xx_U2D
			otg_interrupt(dev->transceiver);
			handled = 0;
			goto irq_end;
#else
			handled = 1;
			otgisr = U2DOTGISR;
			state = dev->xv_ops->otgx_get_mode();

			DMSG("\tU2DICR2:0x%08x U2DISR2:0x%08x U2DOTGCR:0x%08x,"
			     "U2DOTGICR:0x%08x, U2DOTGISR:0x%08x,\n",
			     U2DICR2, U2DISR2, U2DOTGCR, U2DOTGICR, U2DOTGISR);
			if (state != USB_INT_OTG_CLIENT_DP)
				if (otgisr & U2DOTGINT_SI) {
					U2DOTGISR = U2DOTGINT_SI;
					state = USB_INT_OTG_CLIENT_DP;
					dev->xv_ops->otgx_set_mode(state);
				}

			otgisr = U2DOTGISR;
			U2DOTGISR = otgisr;

			if (otgisr & (U2DOTGINT_RVV | U2DOTGINT_FVV)) {
				DMSG("otgisr: 0x%x, otgusr: 0x%x\n",
				     otgisr, U2DOTGUSR);
				cable_detect_interrupt();
				if (otgisr & U2DOTGINT_FVV) {
					dev->xv_ops->otgx_set_mode(USB_OTG_LP);
					handled = 0;
					goto irq_end;
				} else if (state != USB_INT_OTG_CLIENT_DP) {
					state = USB_INT_OTG_CLIENT_DP;
					dev->xv_ops->otgx_set_mode(state);
				}
			}
#endif
		}

	if (unlikely(u2dint & U2DINT_CC)) {
		unsigned config, interface, alternate;

		handled = 1;
		DBG(DBG_VERBOSE, "USB SET_CONFIGURATION or "
		    "SET_INTERFACE command received\n");

		config = (U2DCR & U2DCR_ACN) >> U2DCR_ACN_S;
		interface =  (U2DCR & U2DCR_AIN) >> U2DCR_AIN_S;
		alternate = (U2DCR & U2DCR_AAISN) >> U2DCR_AAISN_S;

		DBG(DBG_VERBOSE, "    config=%d,  interface=%d, alternate=%d,"
		    " u2dcr %x\n", config, interface, alternate, U2DCR);

		if (dev->cdev.configuration != config) {
			dev->cdev.configuration = config;
			pxa3xx_change_configuration(dev) ;
		} else if ((dev->cdev.interface != interface) || \
			    (dev->cdev.interface == 0) || \
			    (dev->cdev.alternate != alternate)) {
			dev->cdev.interface = interface;
			dev->cdev.alternate = alternate;
			pxa3xx_change_interface(dev);
		}

		U2DCR = (U2DCR & (U2DCR_UDE)) | U2DCR_CC | U2DCR_SMAC;
#ifdef USE_SPEOREN
		if (cpu_is_pxa310())
			U2DCR |= U2DCR_SPEOREN;
#endif
		DMSG("%s: u2dcr:0x%1x, u2dcsr0:0x%1x\n",
		     __FUNCTION__, U2DCR, U2DCSR0);

		while (U2DCR & U2DCR_SMAC) ;
		U2DISR = U2DINT_CC;
	}

	/* ReSeT Interrupt Request - USB reset */
	if (unlikely(u2dint & U2DINT_RS)) {
		U2DISR = U2DINT_RS;
		handled = 1;

		DBG(DBG_VERBOSE, "USB reset start\n");
		stop_gadget(&dev->cdev, &dev->gadget, dev->driver);
		dev->gadget.speed = USB_SPEED_UNKNOWN;

		/* enable SOF/uSOF interrupt to detect bus speed */
		U2DICR |= (U2DINT_SOF | U2DINT_USOF);

		memset(&dev->stats, 0, sizeof dev->stats);
		udc_reinit();
	}

	/* SOF/uSOF Interrupt, to detect bus speed */
	if (unlikely(U2DISR & (U2DINT_SOF | U2DINT_USOF))) {
		handled = 1;

		DBG(DBG_VERBOSE, "USB SOF/uSOF, u2disr %x, u2dcra %x,"
		    " u2dcsra %x\n", U2DISR, U2DEPCR(1), U2DCSR(1));

		/* clear SOF/uSOF interrupt */
		U2DICR &= ~(U2DINT_SOF | U2DINT_USOF);
		if (U2DISR & U2DINT_USOF)
			U2DISR = U2DINT_USOF;
		if (U2DISR & U2DINT_SOF)
			U2DISR = U2DINT_SOF;

		if (dev->gadget.speed == USB_SPEED_UNKNOWN) {
			if (U2DCR & U2DCR_HS)
				dev->gadget.speed = USB_SPEED_HIGH;
			else
				dev->gadget.speed = USB_SPEED_FULL;
		}

		/* change the endpoint MPS */
		change_mps(dev->gadget.speed);

		/* set the endpoint information register */
		u2d_eps_config(2);

	}

	u2disr = U2DISR;
	if (cpu_is_pxa310())
		u2disr |= U2DISR2;

	if (u2disr & U2DISR_MASK) {
		int i;
		u2disr = U2DISR ;

		DBG(DBG_VERY_NOISY, "irq %02x\n", u2disr);

		/* control traffic */
		if (u2disr & U2DINT_EP0) {
			DMSG("handle_ep0: U2DISR:%x, U2DCSR0:%x\n", \
			     U2DISR, U2DCSR0);

			U2DISR = U2DINT(0, U2DINT_MASK);

			if (u2disr & U2DINT_FIFOERR) {
				printk(KERN_WARNING "Endpoint 0 fifo Error\n");
				/* why delete originally????? */
				U2DISR = U2DINT(0, U2DINT_FIFOERR);
			}
			dev->ep[0].pio_irqs++;
			if ((u2disr & U2DINT_PACKETCMP)
			    || (u2disr & U2DINT_SPACKETCMP)) {
				handle_ep0(dev);
				handled = 1;
			}
			/* why delete originally????? */
			/* UDCISR0 = UDCISR_INT(0, UDCISR_INT_MASK); */
		}

		u2disr >>= 3;
		/* endpoint data transfers */
		for (i = 1; u2disr != 0 && i < u2d_ep_num; u2disr >>= 3, i++) {
			if (i >= HALF_EP_NUM) {
				u2disr = U2DISR2;
				U2DISR2 = U2DINT((i - HALF_EP_NUM),
						 U2DINT_MASK);
			} else
				U2DISR = U2DINT(i, U2DINT_MASK);

			if (u2disr & U2DINT_FIFOERR) {
				pr_warning(" Endpoint %d Fifo error,"
					   " csr:%x dcsr:%x da %x dcmd %x\n",
					   i, U2DCSR(i), U2DMACSR(i),
					   U2DMADADR(i), U2DMACMDR(i));
				if (i >= HALF_EP_NUM)
					U2DICR2 &= ~(U2DINT((i - HALF_EP_NUM),
							    U2DINT_FIFOERR));
				else
					U2DICR &= ~(U2DINT(i, U2DINT_FIFOERR));

				if (U2DCSR(i) & U2DCSR_TRN)
					U2DCSR(i) = U2DCSR_DME | U2DCSR_TRN;
				U2DMACSR(i) &= ~U2DMACSR_RUN;
				U2DCSR(i) = U2DCSR_FEF | U2DCSR_PC;
			}

			if ((u2disr & U2DINT_PACKETCMP) \
			    || (u2disr & U2DINT_FIFOERR)) {
				handle_ep(&dev->ep[get_lg_ep(i)]);
				handled = 1;
			}
		}
	}
	/* we could also ask for 1 msec SOF (SIR) interrupts */
	if (handled)
		goto irq_loop;

irq_end:
	DMSG("IRQ_HANDLED\n");
	return IRQ_HANDLED;
}

static void u2d_init_ep(struct pxa27x_udc *dev)
{
	int i;

	INIT_LIST_HEAD(&dev->unused_ep_list);
	INIT_LIST_HEAD(&dev->used_ep_list);
	INIT_LIST_HEAD(&dev->gadget.ep_list);
	INIT_LIST_HEAD(&dev->gadget.ep0->ep_list);

	for (i = 0; i < UDC_LG_EP_NUM; i++) {
		struct pxa27x_ep *ep = &dev->ep[i];

		ep->dma = -1;
		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &dev->unused_ep_list);
		INIT_LIST_HEAD(&ep->queue);
		ep->desc = NULL;
	}
}

/*-------------------------------------------------------------------------*/

static void nop_release(struct device *dev)
{
	DMSG("%s %s\n", __FUNCTION__, dev->bus_id);
}

/* this uses load-time allocation and initialization (instead of
 * doing it at run-time) to save code, eliminate fault paths, and
 * be more obviously correct.
 */
static struct pxa27x_udc memory = {
	.gadget                         = {
		.ops			= &pxa3xx_u2d_ops,
		.ep0			= &memory.ep[0].ep,
		.name			= driver_name,
		.dev                    = {
			.bus_id		= "gadget",
			.release	= nop_release,
		},
	},

	/* control endpoint */
	.ep[0] = {
		.ep                     = {
			.name		= ep0name,
			.ops		= &pxa3xx_ep_ops,
			.maxpacket	= EP0_FIFO_SIZE,
		},
		.dev			= &memory,
		.ep_num			= 0,
	},

	/* first group of endpoints */
	.ep[1] = {
		.ep                     = {
			.name		= "ep1in-bulk",
			.ops		= &pxa3xx_ep_ops,
			.maxpacket	= BULK_MPS(USB_SPEED_FULL),
		},
		.dev			= &memory,
		.fifo_size		= BULK_FIFO_SIZE,
		.ep_num			= 1,
	},
	.ep[2] = {
		.ep                     = {
			.name		= "ep2out-bulk",
			.ops		= &pxa3xx_ep_ops,
			.maxpacket	= BULK_MPS(USB_SPEED_FULL),
		},
		.dev			= &memory,
		.fifo_size		= BULK_FIFO_SIZE,
		.ep_num			= 2,
	},

	.ep[3] = {
		.ep                     = {
			.name		= "ep3in-iso",
			.ops		= &pxa3xx_ep_ops,
			.maxpacket	= ISO_MPS(USB_SPEED_FULL),
		},
		.dev			= &memory,
		.fifo_size		= ISO_FIFO_SIZE,
		.hs_cmds                = ISO_HS_CMDS,
	},
	.ep[4] = {
		.ep                     = {
			.name		= "ep4out-iso",
			.ops		= &pxa3xx_ep_ops,
			.maxpacket	= ISO_MPS(USB_SPEED_FULL),
		},
		.dev			= &memory,
		.fifo_size		= ISO_FIFO_SIZE,
		.hs_cmds                = ISO_HS_CMDS,
	},
	.ep[5] = {
		.ep                     = {
			.name		= "ep5in-int",
			.ops		= &pxa3xx_ep_ops,
			.maxpacket	= INT_MPS(USB_SPEED_FULL),
		},
		.dev			= &memory,
		.fifo_size		= INT_FIFO_SIZE,
		.hs_cmds                = INT_HS_CMDS,
		.ep_num			= 5,
	},

	/* second group of endpoints */
	.ep[6] = {
		.ep                     = {
			.name		= "ep6in-bulk",
			.ops		= &pxa3xx_ep_ops,
			.maxpacket	= BULK_MPS(USB_SPEED_FULL),
		},
		.dev			= &memory,
		.fifo_size		= BULK_FIFO_SIZE,
	},
	.ep[7] = {
		.ep                     = {
			.name		= "ep7out-bulk",
			.ops		= &pxa3xx_ep_ops,
			.maxpacket	= BULK_MPS(USB_SPEED_FULL),
		},
		.dev			= &memory,
		.fifo_size		= BULK_FIFO_SIZE,
	},
	.ep[8] = {
		.ep                     = {
			.name		= "ep8in-iso",
			.ops		= &pxa3xx_ep_ops,
			.maxpacket	= ISO_MPS(USB_SPEED_FULL),
		},
		.dev			= &memory,
		.fifo_size		= ISO_FIFO_SIZE,
		.hs_cmds                = ISO_HS_CMDS,
	},
	.ep[9] = {
		.ep                     = {
			.name		= "ep9out-iso",
			.ops		= &pxa3xx_ep_ops,
			.maxpacket	= ISO_MPS(USB_SPEED_FULL),
		},
		.dev			= &memory,
		.fifo_size		= ISO_FIFO_SIZE,
		.hs_cmds                = ISO_HS_CMDS,
	},
	.ep[10] = {
		.ep                     = {
			.name		= "ep10in-int",
			.ops		= &pxa3xx_ep_ops,
			.maxpacket	= INT_MPS(USB_SPEED_FULL),
		},
		.dev			= &memory,
		.fifo_size		= INT_FIFO_SIZE,
		.hs_cmds                = INT_HS_CMDS,
	},

	/* third group of endpoints */
	.ep[11] = {
		.ep                     = {
			.name		= "ep11in-bulk",
			.ops		= &pxa3xx_ep_ops,
			.maxpacket	= BULK_MPS(USB_SPEED_FULL),
		},
		.dev			= &memory,
		.fifo_size		= BULK_FIFO_SIZE,
	},
	.ep[12] = {
		.ep                     = {
			.name		= "ep12out-bulk",
			.ops		= &pxa3xx_ep_ops,
			.maxpacket	= BULK_MPS(USB_SPEED_FULL),
		},
		.dev			= &memory,
		.fifo_size		= BULK_FIFO_SIZE,
	},
	.ep[13] = {
		.ep                     = {
			.name		= "ep13in-iso",
			.ops		= &pxa3xx_ep_ops,
			.maxpacket	= ISO_MPS(USB_SPEED_FULL),
		},
		.dev			= &memory,
		.fifo_size		= ISO_FIFO_SIZE,
		.hs_cmds                = ISO_HS_CMDS,
	},
	.ep[14] = {
		.ep                     = {
			.name		= "ep14out-iso",
			.ops		= &pxa3xx_ep_ops,
			.maxpacket	= ISO_MPS(USB_SPEED_FULL),
		},
		.dev			= &memory,
		.fifo_size		= ISO_FIFO_SIZE,
		.hs_cmds                = ISO_HS_CMDS,
	},
	.ep[15] = {
		.ep                     = {
			.name		= "ep15in-int",
			.ops		= &pxa3xx_ep_ops,
			.maxpacket	= INT_MPS(USB_SPEED_FULL),
		},
		.dev			= &memory,
		.fifo_size		= INT_FIFO_SIZE,
		.hs_cmds                = INT_HS_CMDS,
	},

};

#ifdef DEBUG
#define CP15R0_VENDOR_MASK	0xffffe000
#define CP15R0_XSCALE_VALUE	0x69054000	/* intel/arm/xscale */
#endif

/*
 * 	probe - binds to the platform device
 */
static int __init pxa3xx_u2d_probe(struct platform_device *pdev)
{
	struct pxa27x_udc *dev = &memory;
	struct device *_dev = &pdev->dev;
	int retval;

	if (cpu_is_pxa310()) {
		u2d_ep_num = PXA_U2D_EP_NUM;
		dev->ulpi_int = PXA310_ULPI_INT;
	} else {
		u2d_ep_num = HALF_EP_NUM;
		if (machine_is_zylonite()) {
			if (cpu_is_pxa300())
				dev->u2d_dp = PXA300_U2D_DETECT;
			else
				dev->u2d_dp = PXA320_U2D_DETECT;
		}
	}

	dev->clk = clk_get(&pdev->dev, "U2DCLK");

	if (IS_ERR(dev->clk)) {
		retval = PTR_ERR(dev->clk);
		goto err_clk;
	}
	clk_enable(dev->clk);

	dev->cwsbr_base = ioremap_nocache(CWSBR_BASE, IORESOURCE_IO);
	if (dev->cwsbr_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap registers\n");
		retval = -ENXIO;
		goto err_irq;
	}

	/* other non-static parts of init */
	dev->dev = _dev;
	dev->mach = _dev->platform_data;

	init_timer(&dev->timer);
	dev->timer.function = u2d_watchdog;
	dev->timer.data = (unsigned long)dev;

#if defined(CONFIG_PXA3xx_DVFM)
	init_timer(&dev->dvfm_timer);
	dev->dvfm_timer.function = pxa3xx_u2d_dvfm_enable;
	dev->dvfm_timer.data = (unsigned long)dev;
#endif

	device_initialize(&dev->gadget.dev);
	dev->gadget.dev.parent = _dev;
	dev->gadget.dev.dma_mask = _dev->dma_mask;

	dev->gadget.is_dualspeed = 1;

	the_controller = dev;
	platform_set_drvdata(pdev, dev);

	/* __u2d_disable(dev); */
	init_ep_assign();
	u2d_init_ep(dev);
	udc_reinit();

	u2d_bugs = u2d_bug_check();

	/* reset the transceiver */
	if (cpu_is_pxa310()) {
		dev->xv_ops = init_pxa310_otg_xceiv_ops();
		dev->xv_ops->reset_xcvr_init();
	} else if (cpu_is_pxa930()) {
		dev->xv_ops = init_pxa930_otg_xceiv_ops();
		dev->xv_ops->reset_xcvr_init();
	}

	dev->mach->reset_xcvr();

#ifdef CONFIG_USB_OTG_PXA3xx_U2D
	if (cpu_is_pxa310()) {
		dev->gadget.is_otg = 1;
		dev->transceiver = otg_get_transceiver();
		dev->cdev.transceiver = otg_get_transceiver();
		if (!dev->transceiver)
			DMSG("failed to get transceiver\n");
	}
#endif

	if (cpu_is_pxa930())
		U2DMACR = 0x4;

	/* irq setup after old hardware state is cleaned up */
	retval = request_irq(IRQ_USB2, pxa3xx_u2d_irq,
			     IRQF_DISABLED, driver_name, dev);
	if (retval != 0) {
		printk(KERN_ERR "%s: can't get irq %i, err %d\n",
		       driver_name, IRQ_USB2, retval);
		goto err_irq;
	}

	dev->got_irq = 1;
	dev->u2d_irq_dis = 0;

	create_proc_files();

#ifdef U2D_USE_ISRAM
	zylu2d_immid = imm_register_kernel("pxa3xx_u2d");
#endif

    printk(KERN_INFO "Davis: U2DP3CR %08X\n", U2DP3CR);
	clk_disable(dev->clk);

	return 0;

err_irq:
	clk_put(dev->clk);
	clk_disable(dev->clk);
err_clk:
	return 0;
}

static int pxa3xx_u2d_remove(struct platform_device *pdev)
{
	struct pxa27x_udc *dev = platform_get_drvdata(pdev);

	remove_proc_files();
	/* usb_gadget_unregister_driver(dev->driver); */

	if (dev->got_irq) {
		free_irq(IRQ_USB2, dev);
		dev->got_irq = 0;
		dev->u2d_irq_dis = 1;
	}

#ifdef CONFIG_ARCH_LUBBOCK
	if (machine_is_lubbock() && dev->got_disc) {
		free_irq(LUBBOCK_USB_DISC_IRQ, dev);
		dev->got_disc = 0;
	}
#endif

	clk_put(dev->clk);

	platform_set_drvdata(pdev, NULL);
	the_controller = 0;

	if (machine_is_zylonite()) {
		free_irq(IRQ_USB2, dev);
		dev->u2d_irq_dis = 1;
	}

	return 0;
}

#ifdef CONFIG_PM
static void u2d_save_state(struct pxa27x_udc *dev)
{
	struct pxa27x_ep *ep = NULL;
	int i;

	dev->u2dcr = U2DCR;
	dev->u2dicr = U2DICR;
	dev->u2dcsr0 = U2DCSR0;
	for (i = 1; i < UDC_LG_EP_NUM; i++) {
		if (dev->ep[i].assigned) {
			ep = &dev->ep[i];
			ep->u2dcsr_value = U2DCSR(ep->ep_num);
			ep->u2dcr_value = U2DEPCR(ep->ep_num);
			ep->u2denr_value = U2DEN(ep->ep_num);
			DMSG("EP%d, u2dcsr:0x%x, u2dcr:0x%x, u2denr:0x%x\n",
			     i, U2DCSR(ep->ep_num), U2DEPCR(ep->ep_num),
			     U2DEN(ep->ep_num));
		}
	}

	if (cpu_is_pxa310()) {
		dev->u2dotgcr = U2DOTGCR;
		dev->u2dotgicr = U2DOTGICR;
	} else if (cpu_is_pxa930())
		dev->u2dotgcr = U2DOTGCR;
}

static void u2d_restore_state(struct pxa27x_udc *dev)
{
	struct pxa27x_ep *ep = NULL;
	int i;

	U2DCSR0 = dev->u2dcsr0 & (U2DCSR0_FST | U2DCSR0_DME);
	WRITE_COMPLETE(&U2DCSR0);
	U2DICR = dev->u2dicr;
	U2DCR = dev->u2dcr;
	for (i = 1; i < UDC_LG_EP_NUM; i++) {
		if (dev->ep[i].assigned) {
			ep = &dev->ep[i];

			U2DCSR(ep->ep_num) = ep->u2dcsr_value;
			U2DEPCR(ep->ep_num) = ep->u2dcr_value;
			U2DEN(ep->ep_num)  = ep->u2denr_value;
			printk(KERN_DEBUG "EP%d, u2dcsr:0x%x, u2dcr:0x%x\n",
			       i, U2DCSR(ep->ep_num), U2DEPCR(ep->ep_num));
		}
	}

	if (cpu_is_pxa310()) {
		U2DOTGCR = dev->u2dotgcr;
		U2DOTGICR = dev->u2dotgicr;
	} else if (cpu_is_pxa930())
		U2DOTGCR = dev->u2dotgcr;
}

static int pxa3xx_u2d_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct pxa27x_udc *dev;
	dev = (struct pxa27x_udc *)platform_get_drvdata(pdev);

	if (dev->driver) {
		clk_enable(dev->clk);
		if (cpu_is_pxa310()) {
			if (dev->stp_gpio_irq_en) {
				int irq = gpio_to_irq(dev->ulpi_int);
				disable_irq(irq);
				dev->ulpi_dat3_work = 1;
				dev->xv_ops->ulpi_dat3_work();
				dev->stp_gpio_irq_en = 0;
			}
		}
		if (0/*is_cable_attached()*/) {
			printk(KERN_ERR "Can't make system into suspend " \
			       "state when USB cable is attached\n");
			return -EACCES;
		}
		u2d_save_state(dev);
		stop_gadget(&dev->cdev, &dev->gadget, dev->driver);
		/* soft disconnect the device & disable UDE */
		if (dev->driver)
			u2d_soft_dis(1);
		if (cpu_is_pxa310() || cpu_is_pxa930())
			U2DCR &= ~U2DCR_UDE;

#ifdef CONFIG_USB_OTG
		otg_set_peripheral(dev->transceiver, NULL);
#endif
		u2d_irq_set(0);
		u2d_clk_set(0);
		clk_disable(dev->clk);
	}

	return 0;
}

static struct timer_list u2d_resume_timer;
static void u2d_resume_timer_handler( unsigned long ptr )
{
	struct platform_device *pdev = (struct platform_device *)(ptr);
	struct pxa27x_udc *dev = (struct pxa27x_udc *)platform_get_drvdata(pdev);
	int pre_connected = connected;
	int new_connected = is_cable_attached();
	/* if pre_connected!=new_connected, deal new u2d @ cable_detect_interrupt*/

	if (dev->driver) {
		clk_enable(dev->clk);
		u2d_clk_set(1);
		u2d_restore_state(dev);
		u2d_irq_set(1);

		if (pre_connected && new_connected) {
			if (cpu_is_pxa310() || cpu_is_pxa930())
				U2DCR |= U2DCR_UDE;
			u2d_soft_dis(0);
			__u2d_enable(dev);
		}
#ifdef CONFIG_USB_OTG
		otg_set_peripheral(dev->transceiver, &dev->gadget);
#endif
 		/* let u2d enter into low powe mode when usb not connected */
		if (!pre_connected && !new_connected && dev->xv_ops)
			dev->xv_ops->otgx_set_mode(USB_OTG_LP);
		clk_disable(dev->clk);
	}

	del_timer(&u2d_resume_timer);
}
static void u2d_resume_timer_init(struct platform_device *pdev)
{
    init_timer(&u2d_resume_timer);
    u2d_resume_timer.function = u2d_resume_timer_handler;
    u2d_resume_timer.data = (unsigned long)pdev;
    u2d_resume_timer.expires = jiffies+ HZ/2;
    add_timer(&u2d_resume_timer);
}

static int pxa3xx_u2d_resume(struct platform_device *pdev)
{
#if 0  // this is a real problem, when wake up, here will suspend the kernel.  
	u2d_resume_timer_handler(pdev);
#else
	u2d_resume_timer_init(pdev);
#endif

	return 0;
}
#else
#define pxa3xx_u2d_suspend	NULL
#define pxa3xx_u2d_resume	NULL
#endif

static void pxa3xx_u2d_shutdown(struct platform_device *pdev)
{
	if (connected) { /* cable connected, close u2d for power-off */
		struct pxa27x_udc *dev = (struct pxa27x_udc *)platform_get_drvdata(pdev);
		
		u2d_soft_dis(1);
		if (dev->xv_ops) {
			dev->xv_ops->otgx_set_mode(USB_OTG_LP);
		}
		__u2d_disable(dev);
		udc_stop(&dev->cdev, &dev->gadget, dev->driver, 1);
	}
}


/***************************************************************************/
static struct platform_driver u2d_driver = {
	.driver		= {
		.name	= "pxa3xx-u2d",
	},
	.probe		= pxa3xx_u2d_probe,
	.remove		= pxa3xx_u2d_remove,
	.suspend	= pxa3xx_u2d_suspend,
	.resume		= pxa3xx_u2d_resume,
	.shutdown	= pxa3xx_u2d_shutdown,
};

static int __init u2d_init(void)
{
#ifdef CONFIG_PXA3xx_DVFM
	dvfm_register("U2D", &dvfm_lock.dev_idx);
#endif
	return platform_driver_register(&u2d_driver);
}

static void __exit u2d_exit(void)
{
	platform_driver_unregister(&u2d_driver);
#ifdef CONFIG_PXA3xx_DVFM
	dvfm_unregister("U2D", &dvfm_lock.dev_idx);
#endif
}

module_init(u2d_init);
module_exit(u2d_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Frank Becker, Robert Schwebel, David Brownell");
MODULE_LICENSE("GPL");

