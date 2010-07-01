/*
 * Copyright (c) 2000-2002 by Dima Epshtein
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* #define DRIVER_AUTHOR "Dima Epshtein" */

#ifdef CONFIG_USB_DEBUG
    #define DEBUG
#else
    #undef DEBUG
#endif

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <asm/arch/pxa3xx-regs.h>
#include <asm/arch/pxa9xx_u2o.h>

#ifdef CONFIG_USB_OTG
#include <linux/usb/otg.h>
#endif

static struct ehci_hcd         *g_ehci;

void pxa9xx_u2o_host_enable (void)
{
	pr_debug("\n %s\n", __func__);
#ifndef CONFIG_USB_OTG
	/* Turn off IDPU bit of U2xOTGSC - we are not in OTG mode */
	U2xOTGSC |= U2xOTGSC_IDPU;
#endif

	/* Set hardware to Host Mode */
	U2xUSBMODE |= U2xUSBMODE_CM_MASK;

	pr_debug("%s: PORTSC 0x%x USBMODE 0x%x USBSTS 0x%x USBCMD 0x%x\n",
		__func__, U2xPORTSC, U2xUSBMODE, U2xUSBSTS, U2xUSBCMD);
}

void pxa9xx_ehci_set(int enable)
{
	struct usb_hcd *hcd = ehci_to_hcd(g_ehci);
	static int enabled;
	int retval = 0;

	pr_debug("%s %s enabled %d\n", __func__, 
		enable?"enable":"disable", enabled);
	
	if (enable) {
		if (hcd->rh_registered) {
			pr_debug("%s not removed???\n", __func__);
			usb_remove_hcd(hcd);
		}

		retval = usb_add_hcd(hcd, IRQ_U2O, IRQF_DISABLED | IRQF_SHARED);
		if (retval < 0) {
			printk(KERN_ERR "pxa9xx ehci: hc_reset failed\n");
		goto err;
		}
		enabled = 1;
	} else {
		/*
		if (hcd->rh_registered)
			usb_remove_hcd(hcd);
		*/
		enabled = 0;
	}
err:
	return;
}

#ifndef CONFIG_USB_OTG
static void clock_init(void)
{
	unsigned long flags;
	/* Actually, there is no need to disable interrupt, just to be safe */
	local_irq_save(flags);
	ACCR1 = ACCR1 | ACCR1_PU_OTG | ACCR1_PU_PLL | ACCR1_PU;
	local_irq_restore(flags);
}
#endif

static int pxa9xx_ehci_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd	*ehci = hcd_to_ehci(hcd);
	int		retval;

	pr_debug("%s\n", __func__);
	pxa9xx_u2o_host_enable();

	retval = ehci_halt(ehci);
	if (retval)
		return retval;

	/*
         * data structure init
         */
	retval = ehci_init(hcd);
	if (retval)
		return retval;

        ehci->is_tdi_rh_tt = 0;
	ehci->sbrn = 0x20;

	retval = ehci_reset(ehci);

	return retval;
}

static irqreturn_t pxa9xx_ehci_irq (struct usb_hcd *hcd)
{
#ifdef CONFIG_USB_OTG
	if (!otg_is_host()) {
		int	start = hcd->state;
		pr_debug("\n%s start %x\n", __func__, start);
		return IRQ_NONE;
	}
#endif

	return ehci_irq(hcd);
}

#ifdef CONFIG_USB_OTG
static void start_hnp(struct ehci_hcd *ehci)
{
	unsigned long   flags;

	otg_start_hnp(ehci->transceiver);

	local_irq_save(flags);
	ehci->transceiver->state = OTG_STATE_A_SUSPEND;
#if 0
	struct usb_hcd *hcd = ehci_to_hcd(ehci);
	const unsigned  port = hcd->self.otg_port - 1;
	writel(RH_PS_PSS, &ehci->regs->roothub.portstatus [port]);
#endif
	local_irq_restore(flags);
}

static int pxa9xx_ehci_connect(struct usb_hcd *hcd, struct usb_device *udev)
{
	return otg_connect((hcd_to_ehci(hcd))->transceiver, udev);
}

static int pxa9xx_ehci_disconnect(struct usb_hcd *hcd)
{
	return otg_disconnect((hcd_to_ehci(hcd))->transceiver);
}

#endif


static const struct hc_driver pxa9xx_ehci_hc_driver = {
	.description = hcd_name,
	.product_desc = "pxa9xx ehci",
	.hcd_priv_size = sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq = pxa9xx_ehci_irq,
	.flags = HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset = pxa9xx_ehci_setup,
	.start = ehci_run,
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,

#ifdef	CONFIG_USB_OTG
	.disconnect = pxa9xx_ehci_disconnect,
	.connect = pxa9xx_ehci_connect,
#endif
};

static int pxa9xx_ehci_probe(struct platform_device *pdev)
{
	struct device 		*dev = &pdev->dev;
	struct usb_hcd		*hcd;
	struct ehci_hcd         *ehci;
	int 			retval, i;

	if (usb_disabled())
		return -ENODEV;

	hcd = usb_create_hcd(&pxa9xx_ehci_hc_driver, dev, "pxa9xx ehci");
	if (!hcd) {
		printk(KERN_ERR "pxa9xx ehci: create_hcd failed\n");
		return -ENOMEM;
	}

	ehci = hcd_to_ehci(hcd);
	g_ehci = ehci;

	/* Request resources */
	for(i=0; i<pdev->num_resources; i++)
	{
	    if(pdev->resource[i].flags == IORESOURCE_IRQ)
	    {
	        hcd->irq = pdev->resource[i].start; 
	    }
	    else if(pdev->resource[i].flags == IORESOURCE_DMA)
	    {
		hcd->rsrc_start = pdev->resource[i].start;
		hcd->rsrc_len = pdev->resource[i].end - hcd->rsrc_start + 1;
	    }
	}     

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		pr_debug("request_mem_region failed");
		retval = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		pr_debug("ioremap failed");
		retval = -ENOMEM;
		goto err2;
	}

	hcd->self.controller = dev;
	hcd->self.bus_name = dev->bus_id;
	hcd->product_desc ="pxa9xx ehci";

        /*
         * registers start at offset
         */
        ehci->caps = hcd->regs;
        ehci->regs = hcd->regs +
                HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase));

        /*
         * cache this readonly data; minimize chip reads
         */
        ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

#ifndef CONFIG_USB_OTG
	/* enable clock & transceiver */
	clock_init();
	/*   xcvr_init(); */
	retval = usb_add_hcd(hcd, hcd->irq, IRQF_DISABLED | IRQF_SHARED);
	if (retval < 0) {
		printk(KERN_ERR "pxa9xx_ehci: usb_add_hcd failed.\n");
		goto err2;
	}
#else
	hcd->self.otg_port = 2;
	ehci->transceiver = otg_get_transceiver();
	if (ehci->transceiver) {
		otg_set_host(ehci->transceiver, &hcd->self);
	} else {
		dev_err(hcd->self.controller, "can't find otg transceiver\n");
		return -ENODEV;
	}
#endif
	return 0;

err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err1:
	usb_put_hcd(hcd);
	printk(KERN_ERR "pxa9xx ehci: init error, %d\n", retval);
	return retval;	
}

static int pxa9xx_ehci_remove(struct platform_device *dev)
{
	struct ehci_hcd		*ehci = dev_get_drvdata(&dev->dev);
	struct usb_hcd		*hcd = ehci_to_hcd(ehci);

	if (HC_IS_RUNNING(hcd->state))
		hcd->state = HC_STATE_QUIESCING;

	usb_disconnect(&hcd->self.root_hub);
	hcd->driver->stop (hcd);
	
	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
	return 0;
}

MODULE_ALIAS("pxa9xx-ehci");

static struct platform_driver pxa9xx_ehci_driver = {
	.probe = pxa9xx_ehci_probe,
	.remove = pxa9xx_ehci_remove,
	.shutdown = usb_hcd_platform_shutdown,
	.driver = {
		.name = "pxa9xx-ehci",
		.bus = &platform_bus_type
	}
};
