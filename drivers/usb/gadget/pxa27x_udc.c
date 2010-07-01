/*
 * linux/drivers/usb/gadget/pxa27x_udc.c
 * PXA3xx on-chip full speed USB device controllers
 *
 * Copyright (C) 2002 Intrinsyc, Inc. (Frank Becker)
 * Copyright (C) 2003 Robert Schwebel, Pengutronix
 * Copyright (C) 2003 Benedikt Spranger, Pengutronix
 * Copyright (C) 2003 David Brownell
 * Copyright (C) 2003 Joshua Wise
 * Copyright (C) 2004 Intel Corporation
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
#undef	DEBUG
#undef	VERBOSE

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

#include <asm/byteorder.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <asm/unaligned.h>
#include <mach/hardware.h>
#include <mach/pxa-regs.h>
#ifdef CONFIG_CPU_PXA27x
#include <mach/pxa27x-regs.h>
#endif
#include <mach/pxa3xx_pmic.h>
#include <asm/uaccess.h>

#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/usb/cdc.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>

#include <mach/udc.h>

/*
 * This driver handles the USB Device Controller (UDC) in Intel's PXA 27x
 * series processors.
 * Such controller drivers work with a gadget driver.  The gadget driver
 * returns descriptors, implements configuration and data protocols used
 * by the host to interact with this device, and allocates endpoints to
 * the different protocol interfaces.  The controller driver virtualizes
 * usb hardware so that the gadget drivers will be more portable.
 *
 * This UDC hardware wants to implement a bit too much USB protocol, so
 * it constrains the sorts of USB configuration change events that work.
 * The errata for these chips are misleading; some "fixed" bugs from
 * pxa250 a0/a1 b0/b1/b2 sure act like they're still there.
 */

#undef UDC_USE_ISRAM
#ifdef UDC_USE_ISRAM
#include <mach/imm.h>
#endif

#define	DRIVER_VERSION	"01-Jul-2005"
#define	DRIVER_DESC	"PXA 27x USB Device Controller driver"

static const char driver_name [] = "pxa27x_udc";

static const char ep0name [] = "ep0";

#define	USE_DMA
/* #define	DISABLE_TEST_MODE */

#ifdef CONFIG_PROC_FS
#define	UDC_PROC_FILE
#endif

#include "../otg/pxa3xx_otg.h"
#include "pxa_comp.h"
#include "pxa27x_udc.h"

static struct pxa27x_udc *the_controller;
#ifdef CONFIG_EMBEDDED
/* few strings, and little code to use them */
#undef	DEBUG
#undef	UDC_PROC_FILE
#endif

#ifdef CONFIG_PXA3xx_DVFM
#include <mach/dvfm.h>
#endif

#ifdef CONFIG_PXA3xx_PMIC
#define ENABLE_CABLE_DETECT
static int connected;
#endif

static void udc_init_ep(struct pxa27x_udc *dev);
static void udc_enable(struct pxa27x_udc *dev);
static void udc_disable(struct pxa27x_udc *dev);

int is_cable_attached(void);
static void pxa27x_usb_event_change(unsigned long events);
#ifdef	USE_DMA
static int use_dma = 1;
module_param(use_dma, bool, 0);
MODULE_PARM_DESC(use_dma, "true to use dma");

static void dma_nodesc_handler (int dmach, void *_ep);
static void kick_dma(struct pxa27x_ep *ep, struct pxa27x_request *req);

#define	DMASTR " (dma support)"

#ifdef UDC_USE_ISRAM
static u32 zyludc_immid;
#endif

#else	/* !USE_DMA */
#define	DMASTR " (pio only)"
#endif

#ifdef CONFIG_USB_PXA27X_SMALL
#define SIZE_STR        " (small)"
#else
#define SIZE_STR        ""
#endif

#define UDCISR0_IR0	 0x3
#define UDCISR_INT_MASK	 (UDC_INT_FIFOERROR | UDC_INT_PACKETCMP)
#define UDCICR_INT_MASK	 UDCISR_INT_MASK

#define UDCCSR_MASK	(UDCCSR_FST | UDCCSR_DME)

#if defined(CONFIG_PXA3xx_DVFM)
static struct dvfm_lock dvfm_lock = {
	.lock		= SPIN_LOCK_UNLOCKED,
	.dev_idx	= -1,
	.count		= 0,
};

static void set_dvfm_constraint(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	if (dvfm_lock.count++ == 0) {
		/* Disable D0CS */
		dvfm_disable_op_name("D0CS", dvfm_lock.dev_idx);
		/* Disable D1/D2 mode */
		dvfm_disable_op_name("D1", dvfm_lock.dev_idx);
		dvfm_disable_op_name("D2", dvfm_lock.dev_idx);
		if (cpu_is_pxa935())
			dvfm_disable_op_name("CG", dvfm_lock.dev_idx);
	}
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}

static void unset_dvfm_constraint(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	if (dvfm_lock.count == 0) {
		printk(KERN_WARNING "UDC constraint has been removed.\n");
	} else if (--dvfm_lock.count == 0) {
		/* Enable D0CS */
		dvfm_enable_op_name("D0CS", dvfm_lock.dev_idx);
		/* Enable D1/D2 mode */
		dvfm_enable_op_name("D1", dvfm_lock.dev_idx);
		dvfm_enable_op_name("D2", dvfm_lock.dev_idx);
		if (cpu_is_pxa935())
			dvfm_enable_op_name("CG", dvfm_lock.dev_idx);
	}
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}
#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
#endif

#ifdef CONFIG_USB_OTG
#include "../otg/pxa300_otg.h"
extern void pxa3xx_otg_require_bus(int require);
#endif
struct pxa27x_udc *get_the_controller(void)
{
	return the_controller;
}

void dump_buffer(char *buf, unsigned length)
{
	char *c = buf;
	int i;

	pr_debug("%s, buffer total length = %d\n", __func__, length);
	for (i = 0; i < length; i++) {
		if (0 == i % 10)
			pr_debug("\n");
		pr_debug(" 0x%x", c[i]);
	}
	pr_debug("\n");
}

/* ---------------------------------------------------------------------------
 * 	endpoint related parts of the api to the usb controller hardware,
 *	used by gadget driver; and the inner talker-to-hardware core.
 * ---------------------------------------------------------------------------
 */

static void pxa27x_ep_fifo_flush(struct usb_ep *ep);
void nuke(struct pxa27x_ep *, int status);

static void pio_irq_enable(int ep_num)
{
	if (ep_num < 16)
		UDCICR0 |= 3 << (ep_num * 2);
	else {
		ep_num -= 16;
		UDCICR1 |= 3 << (ep_num * 2);
	}
}

static void pio_irq_disable(int ep_num)
{
	ep_num &= 0xf;
	if (ep_num < 16)
		UDCICR0 &= ~(3 << (ep_num * 2));
	else {
		ep_num -= 16;
		UDCICR1 &= ~(3 << (ep_num * 2));
	}
}

/*
 * endpoint enable/disable
 *
 * we need to verify the descriptors used to enable endpoints.  since pxa27x
 * endpoint configurations are fixed, and are pretty much always enabled,
 * there's not a lot to manage here.
 *
 * because pxa27x can't selectively initialize bulk (or interrupt) endpoints,
 * (resetting endpoint halt and toggle), SET_INTERFACE is unusable except
 * for a single interface (with only the default altsetting) and for gadget
 * drivers that don't halt endpoints (not reset by set_interface).  that also
 * means that if you use ISO, you must violate the USB spec rule that all
 * iso endpoints must be in non-default altsettings.
 */
static int pxa27x_ep_enable(struct usb_ep *_ep,
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
					       != BULK_FIFO_SIZE)
			|| !desc->wMaxPacketSize) {
		DMSG("%s, bad %s maxpacket\n", __FUNCTION__, _ep->name);
		return -ERANGE;
	}

	dev = ep->dev;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN) {
		DMSG("%s, bogus device state\n", __FUNCTION__);
		return -ESHUTDOWN;
	}

	ep->desc = desc;
	ep->dma = -1;
	ep->stopped = 0;
	ep->pio_irqs = ep->dma_irqs = 0;
	ep->ep.maxpacket = le16_to_cpu(desc->wMaxPacketSize);

	/* flush fifo (mostly for OUT buffers) */
	pxa27x_ep_fifo_flush(_ep);

	/* ... reset halt state too, if we could ... */

#ifdef USE_DMA
	/* for (some) bulk and ISO endpoints, try to get a DMA channel and
	 * bind it to the endpoint.  otherwise use PIO.
	 */
	DMSG("%s: called attributes=%d\n", __FUNCTION__, ep->ep_type);
	switch (ep->ep_type) {
	case USB_ENDPOINT_XFER_ISOC:
		if (le16_to_cpu(desc->wMaxPacketSize) % 32)
			break;
		/*  fall through */
	case USB_ENDPOINT_XFER_BULK:
		if (!use_dma || !ep->reg_drcmr)
			break;
		ep->dma = pxa_request_dma((char *)_ep->name,
				(le16_to_cpu(desc->wMaxPacketSize) > 64)
					? DMA_PRIO_MEDIUM /* some iso */
					: DMA_PRIO_LOW,
				dma_nodesc_handler, ep);
		if (ep->dma >= 0) {
			*ep->reg_drcmr = DRCMR_MAPVLD | ep->dma;
			ep->dma_buf_size = DMA_BUF_SIZE;
#ifdef UDC_USE_ISRAM
			ep->dma_buf_virt = imm_malloc(DMA_BUF_SIZE,
				IMM_MALLOC_HARDWARE | IMM_MALLOC_SRAM,
				zyludc_immid);
			if (ep->dma_buf_virt) {
				ep->dma_buf_phys =
					imm_get_physical(
						(void *)ep->dma_buf_virt,
						zyludc_immid);
			} else {
				pr_err("can't malloc ISRAM by IMM!\n");
				return -ENOMEM;
			}
#else
			ep->dma_buf_virt = dma_alloc_coherent(dev->dev,
					DMA_BUF_SIZE, &ep->dma_buf_phys,
					GFP_ATOMIC);
#endif
			if (!ep->dma_buf_virt) {
				pr_err("%s: failed to allocate dma buffer\n",
				       __FUNCTION__);
				return -ENOMEM;
			}
			DMSG("%s using dma%d\n", _ep->name, ep->dma);
		}
		break;
	default:
		break;
	}
#endif
	DBG(DBG_VERBOSE, "enabled %s\n", _ep->name);
	return 0;
}

static int pxa27x_ep_disable(struct usb_ep *_ep)
{
	struct pxa27x_ep *ep;

	ep = container_of(_ep, struct pxa27x_ep, ep);
	if (!_ep || !ep->desc) {
		DMSG("%s, %s not enabled\n", __FUNCTION__,
			_ep ? ep->ep.name : NULL);
		return -EINVAL;
	}
	nuke(ep, -ESHUTDOWN);

#ifdef	USE_DMA
	if (ep->dma >= 0) {
		*ep->reg_drcmr = 0;
		pxa_free_dma(ep->dma);
#ifdef UDC_USE_ISRAM
		if (ep->dma_buf_virt)
			imm_free((void *)ep->dma_buf_virt, zyludc_immid);
#else
		if (irqs_disabled()) {
			local_irq_enable();
			dma_free_coherent(ep->dev->dev, ep->dma_buf_size, \
				ep->dma_buf_virt, ep->dma_buf_phys);
			local_irq_disable();
		} else {
			dma_free_coherent(ep->dev->dev, ep->dma_buf_size, \
				ep->dma_buf_virt, ep->dma_buf_phys);
		}
#endif
		ep->dma_buf_virt = NULL;
		ep->dma_buf_phys = ep->dma_buf_size = -1 ;
		ep->dma = -1;
	}
#endif

	/* flush fifo (mostly for IN buffers) */
	pxa27x_ep_fifo_flush(_ep);

	ep->desc = 0;
	ep->stopped = 1;

	DBG(DBG_VERBOSE, "%s disabled\n", _ep->name);
	return 0;
}

/*-------------------------------------------------------------------------*/

/* for the pxa27x, these can just wrap kmalloc/kfree.  gadget drivers
 * must still pass correctly initialized endpoints, since other controller
 * drivers may care about how it's currently set up (dma issues etc).
 */

/*
 * 	pxa27x_ep_alloc_request - allocate a request data structure
 */
static struct usb_request *
pxa27x_ep_alloc_request(struct usb_ep *_ep, unsigned gfp_flags)
{
	struct pxa27x_request *req;

	req = kmalloc(sizeof *req, gfp_flags);
	if (!req)
		return 0;

	memset(req, 0, sizeof *req);
	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}


/*
 * 	pxa27x_ep_free_request - deallocate a request data structure
 */
static void
pxa27x_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
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
	req->req.complete(&ep->ep, &req->req);  /* FIXME, better use tasklet */
}


static inline void ep0_idle(struct pxa27x_udc *dev)
{
	dev->cdev.ep0state = EP0_IDLE;
	LED_EP0_OFF;
}

static int
write_packet(volatile u32 *uddr, struct pxa27x_request *req, unsigned max)
{
	u32 *buf;
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
		volatile u8 *reg = (u8 *)uddr;
		char *rd = (u8 *)buf;

		while (remain--)
			*reg = *rd++;
	}

	return length;
}

/*
 * write to an IN endpoint fifo, as many packets as possible.
 * irqs will use this to write the rest later.
 * caller guarantees at least one packet buffer is ready (or a zlp).
 */
static int
write_fifo(struct pxa27x_ep *ep, struct pxa27x_request *req)
{
	unsigned max;

	max = le16_to_cpu(ep->desc->wMaxPacketSize);
	do {
		int	count;
		int	is_last, is_short;

		count = write_packet(ep->reg_udcdr, req, max);

		/* last packet is usually short (or a zlp) */
		if (unlikely(count != max))
			is_last = is_short = 1;
		else {
			if (likely(req->req.length != req->req.actual)
					|| req->req.zero)
				is_last = 0;
			else
				is_last = 1;
			/* interrupt/iso maxpacket may not fill the fifo */
			is_short = unlikely(max < ep->fifo_size);
		}

		DMSG("wrote %s count:%d bytes%s%s %d left %p\n",
			ep->ep.name, count,
			is_last ? "/L" : "", is_short ? "/S" : "",
			req->req.length - req->req.actual, &req->req);

		/* let loose that packet. maybe try writing another one,
		 * double buffering might work.  TSP, TPC, and TFS
		 * bit values are the same for all normal IN endpoints.
		 */
		*ep->reg_udccsr = UDCCSR_PC;
		if (is_short)
			*ep->reg_udccsr = UDCCSR_SP;

		/* requests complete when all IN data is in the FIFO */
		if (is_last) {
			done(ep, req, 0);
			if (list_empty(&ep->queue) || unlikely(ep->dma >= 0)) {
				pio_irq_disable(ep->ep_num);
#ifdef USE_DMA
				/* unaligned data and zlps couldn't use dma */
				if (unlikely(!list_empty(&ep->queue))) {
					req = list_entry(ep->queue.next,
						struct pxa27x_request, queue);
					kick_dma(ep, req);
					return 0;
				}
#endif
			}
			return 1;
		}

		/*  TODO experiment: how robust can fifo mode tweaking be?
		 *  double buffering is off in the default fifo mode, which
		 *  prevents TFS from being set here.
		 */
	} while (*ep->reg_udccsr & UDCCSR_FS);
	return 0;
}

/* caller asserts req->pending (ep0 irq status nyet cleared); starts
 * ep0 data stage.  these chips want very simple state transitions.
 */
static inline
void ep0start(struct pxa27x_udc *dev, u32 flags, const char *tag)
{
	UDCCSR0 = flags|UDCCSR0_SA|UDCCSR0_OPC|UDCCSR0_ACM|UDCCSR0_ODFCLR;
	dev->req_pending = 0;
	DBG(DBG_VERY_NOISY, "%s %s, %02x/%02x\n",
		__FUNCTION__, tag, UDCCSR0, flags);
}

static int
write_ep0_fifo(struct pxa27x_ep *ep, struct pxa27x_request *req)
{
	unsigned count;
	int is_short;

	count = write_packet(&UDCDR0, req, EP0_FIFO_SIZE);
	ep->dev->stats.write.bytes += count;

	/* last packet "must be" short (or a zlp) */
	is_short = (count != EP0_FIFO_SIZE);

	DBG(DBG_VERY_NOISY, "ep0in %d bytes %d left %p\n", count,
		req->req.length - req->req.actual, &req->req);

	if (unlikely(is_short)) {
		if (ep->dev->req_pending)
			ep0start(ep->dev, UDCCSR0_IPR, "short IN");
		else
			UDCCSR0 = UDCCSR0_ACM | UDCCSR0_IPR|UDCCSR0_ODFCLR;

		count = req->req.length;
		done(ep, req, 0);
		ep0_idle(ep->dev);
	} else if (ep->dev->req_pending)
		ep0start(ep->dev, 0, "IN");

	return is_short;
}


/*
 * read_fifo -  unload packet(s) from the fifo we use for usb OUT
 * transfers and put them into the request.  caller should have made
 * sure there's at least one packet ready.
 *
 * returns true if the request completed because of short packet or the
 * request buffer having filled (and maybe overran till end-of-packet).
 */
static int read_fifo(struct pxa27x_ep *ep, struct pxa27x_request *req)
{
	for (;;) {
		u32		*buf;
		int	bufferspace, count, is_short;

		/* make sure there's a packet in the FIFO.*/
		if (unlikely((*ep->reg_udccsr & UDCCSR_PC) == 0))
			break;
		buf = (u32 *) (req->req.buf + req->req.actual);
		prefetchw(buf);
		bufferspace = req->req.length - req->req.actual;

		/* read all bytes from this packet */
		if (likely(*ep->reg_udccsr & UDCCSR_BNE)) {
			count = 0x3ff & *ep->reg_udcbcr;
			req->req.actual += min(count, bufferspace);
		} else /* zlp */
			count = 0;

		is_short = (count < ep->ep.maxpacket);
		DMSG("read %s udccsr:%02x, count:%d bytes%s req %p %d/%d\n",
			ep->ep.name, *ep->reg_udccsr, count,
			is_short ? "/S" : "",
			&req->req, req->req.actual, req->req.length);

		count = min(count, bufferspace);
		while (likely(count > 0)) {
			*buf++ = *ep->reg_udcdr;
			count -= 4;
		}
		DMSG("Buf:0x%p\n", req->req.buf);

		*ep->reg_udccsr =  UDCCSR_PC;
		/* RPC/RSP/RNE could now reflect the other packet buffer */

		/* completion */
		if (is_short || req->req.actual == req->req.length) {
			done(ep, req, 0);
			if (list_empty(&ep->queue))
				pio_irq_disable(ep->ep_num);
			return 1;
		}

		/* finished that packet.  the next one may be waiting... */
	}
	return 0;
}

/*
 * special ep0 version of the above.  no UBCR0 or double buffering; status
 * handshaking is magic.  most device protocols don't need control-OUT.
 * CDC vendor commands (and RNDIS), mass storage CB/CBI, and some other
 * protocols do use them.
 */
static int read_ep0_fifo(struct pxa27x_ep *ep, struct pxa27x_request *req)
{
	u32		*buf, word;
	unsigned	bufferspace;

	buf = (u32 *) (req->req.buf + req->req.actual);
	bufferspace = req->req.length - req->req.actual;

	while (UDCCSR0 & UDCCSR0_RNE) {
		/* FIXME if setup data is not multiple of 4,
		 * this routing will read some extra bytes */
		word = UDCDR0;

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
	UDCCSR0 = UDCCSR0_ACM | UDCCSR0_OPC | UDCCSR0_ODFCLR;
	/* completion */
	if (req->req.actual >= req->req.length) {
		req->req.actual = req->req.length;
		return 1;
	}

	/* finished that packet.  the next one may be waiting... */
	return 0;
}

#ifdef	USE_DMA

#define	MAX_IN_DMA	((DCMD_LENGTH + 1) - BULK_FIFO_SIZE)

static void start_dma(struct pxa27x_ep *ep, struct pxa27x_request *req, int len)
{
	u32 buf  = (u32)ep->dma_buf_phys;
	u32 fifo = (u32)io_v2p((u32)(ep->reg_udcdr));

	DCSR(ep->dma) = DCSR_NODESC;

	if (ep->dir_in) {
		DSADR(ep->dma) = buf;
		DTADR(ep->dma) = fifo;
		DCMD(ep->dma) = DCMD_BURST32 | DCMD_WIDTH4 | DCMD_ENDIRQEN |
			DCMD_FLOWTRG | DCMD_INCSRCADDR | len;
	} else {
		DSADR(ep->dma) = fifo;
		DTADR(ep->dma) = buf;
		DCMD(ep->dma) = DCMD_BURST32 | DCMD_WIDTH4 | DCMD_ENDIRQEN |
			DCMD_FLOWSRC | DCMD_INCTRGADDR | len;

		if (ep->ep_num < 16)
			UDCICR0 |= (UDCICR_PKTCOMPL << (ep->ep_num << 1));
		else
			UDCICR1 |= (UDCICR_PKTCOMPL << (ep->ep_num << 1));
	}

	*(ep->reg_udccsr) = UDCCSR_DME;
	DCSR(ep->dma) = DCSR_RUN | DCSR_NODESC;
}

static void kick_dma(struct pxa27x_ep *ep, struct pxa27x_request *req)
{
	u32	len = req->req.length;
	char	*buf = (char *)req->req.buf;

	buf += req->req.actual;
	len -= req->req.actual;
	ep->dma_con = 0;

	DCSR(ep->dma) &= ~DCSR_RUN;
	DMSG("%s: req:0x%p, buf:%p, length:%d, actual:%d dma:%d\n",
			__FUNCTION__, &req->req, req->req.buf, req->req.length,
			req->req.actual, ep->dma);

	if (len > ep->dma_buf_size)
		ep->dma_con = 1;
	len = min(len, (u32)ep->dma_buf_size);
	if (ep->dir_in)
		memcpy(ep->dma_buf_virt, buf, len);

	start_dma(ep, req, len);
}

static void cancel_dma(struct pxa27x_ep *ep)
{
	struct pxa27x_request	*req;
	u32			tmp;

	if (DCSR(ep->dma) == 0 || list_empty(&ep->queue))
		return;

	DMSG("dma:%d,dcsr:0x%x\n", ep->dma, DCSR(ep->dma));
	DCSR(ep->dma) = 0;
	while ((DCSR(ep->dma) & DCSR_STOPSTATE) == 0)
		cpu_relax();

	req = list_entry(ep->queue.next, struct pxa27x_request, queue);
	tmp = DCMD(ep->dma) & DCMD_LENGTH;
	req->req.actual = req->req.length - tmp;

	/* the last tx packet may be incomplete, so flush the fifo.
	 * FIXME correct req.actual if we can
	 */
	*ep->reg_udccsr = UDCCSR_FEF;
}

static void dma_nodesc_handler(int dmach, void *_ep)
{
	struct pxa27x_ep	*ep = _ep;
	struct pxa27x_request	*req, *req_next;
	u32			dcsr, completed, remained, count;
	unsigned long flags;

	DMSG("\n");
	local_irq_save(flags);

	req = list_entry(ep->queue.next, struct pxa27x_request, queue);

	ep->dma_irqs++;
	ep->dev->stats.irqs++;
	HEX_DISPLAY(ep->dev->stats.irqs);

	completed = 0;
	remained = req->req.length - req->req.actual;

	dcsr = DCSR(dmach);
	DCSR(ep->dma) &= ~DCSR_RUN;

	if (dcsr & DCSR_BUSERR) {
		printk(KERN_ERR " Buss Error\n");
		DMSG("dcsr:%x, ddadr:%x, dsadr:%x, dtadr:%x, dcmd:%x\n",
				DCSR(dmach), DDADR(dmach), DSADR(dmach),
				DTADR(dmach), DCMD(dmach));
		DCSR(dmach) = DCSR_BUSERR;
		req->req.status = -EIO;
		completed = 1;
	} else if (dcsr & DCSR_ENDINTR) {
		DCSR(dmach) = DCSR_ENDINTR;
		if (ep->dir_in) {
			/* There are still packets to transfer */
			if (ep->dma_con) {
				DMSG("%s: more packets,length:%d,actual:%d\n",
					 __FUNCTION__, req->req.length,
					 req->req.actual);
					req->req.actual += ep->dma_buf_size;
			} else if (remained%BULK_FIFO_SIZE) {
				count = 0;

				*ep->reg_udccsr = UDCCSR_SP | \
					(*ep->reg_udccsr & UDCCSR_MASK);
				/*Wait for packet out */
				while ((count++ < 10000) && \
					!(*ep->reg_udccsr & UDCCSR_PC));
				if (count >= 10000)
					DMSG("Failed to send short packet\n");
				else
					DMSG("%s: short packet sent length:"
						"%d,actual:%d\n", __FUNCTION__,
					req->req.length, req->req.actual);
				completed = 1;
				req->req.actual = req->req.length;
			} else  { /* It is whole package*/
				/* FIXME Sent a ZLP? */
				completed = 1;
				req->req.actual = req->req.length;
				DMSG("%s: req->req.zero=%d, "
					"req->req.length=%d\n",
					__func__, req->req.zero,
					req->req.length);
				if (req->req.zero) {
					/* ZLP needed */
					int count = 0;
					/*Wait for packet out */
					while ((count++ < 10000) && \
						!(*ep->reg_udccsr & UDCCSR_FS));
					if (count >= 10000)
						DMSG("%s,Failed to send ZLP\n",
							__func__);
					else
						DMSG("%s: send ZLP\n",
							__func__);
					*ep->reg_udccsr = UDCCSR_SP | \
						(*ep->reg_udccsr & UDCCSR_MASK);
					req->req.zero = 0;
				}
			}
		} else {
			if (ep->dma_con) {
				memcpy((char *)req->req.buf + req->req.actual,\
						ep->dma_buf_virt, \
						ep->dma_buf_size);
				req->req.actual += ep->dma_buf_size;
			} else {
				completed = 1;
				memcpy((char *)req->req.buf + req->req.actual, \
						ep->dma_buf_virt, \
					req->req.length - req->req.actual);
				req->req.actual = req->req.length;
				DMSG("%s, fully data received\n", __FUNCTION__);
			}
		}
	} else
		DMSG("%s: Others dma:%d DCSR:0x%x DCMD:0x%x\n",
				__FUNCTION__, dmach, DCSR(dmach), DCMD(dmach));

	if (likely(completed)) {
		if (req->queue.next != &ep->queue) {
			req_next = list_entry(req->queue.next,
					struct pxa27x_request, queue);
			kick_dma(ep, req_next);
		}
		done(ep, req, 0);
	} else {
		kick_dma(ep, req);
	}

	local_irq_restore(flags);
}

#endif
/*-------------------------------------------------------------------------*/

int pxa27x_ep_queue(struct usb_ep *_ep, struct usb_request *_req,
			   unsigned gfp_flags)
{
	struct pxa27x_ep	*ep;
	struct pxa27x_request	*req;
	struct pxa27x_udc	*dev;
	unsigned long		flags;

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
	if (unlikely(!dev->driver || ((dev->cdev.ep0state != EP0_IN_FAKE)
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

	DBG(DBG_NOISY, "%s queue req %p, len %d buf %p\n",
	     _ep->name, _req, _req->length, _req->buf);

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
					DMSG("ep0: set config finished\n");
					UDCCSR0 = UDCCSR0_ACM | UDCCSR0_AREN |
						  UDCCSR0_ODFCLR;
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
					ep0start(dev, UDCCSR0_IPR, "OUT");
				if (length == 0 || ((UDCCSR0 & UDCCSR0_RNE) != 0
						&& read_ep0_fifo(ep, req))) {
					ep0_idle(dev);
					UDCCSR0 |= UDCCSR0_IPR | UDCCSR0_ODFCLR;
					done(ep, req, 0);
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
#ifdef USE_DMA
		/* either start dma or prime pio pump */
		} else if (ep->dma >= 0) {
			if ((_req->length == 0) && ep->dir_in) {
				/* ZLP */
				*ep->reg_udccsr = UDCCSR_SP | UDCCSR_DME;
				done(ep, req, 0);
				req = 0;
			} else
				kick_dma(ep, req);
#endif
		/* can the FIFO can satisfy the request immediately? */
		} else if (ep->dir_in
				&& (*ep->reg_udccsr & UDCCSR_FS) != 0
				&& write_fifo(ep, req))
			req = 0;
		else if (!ep->dir_in
				&& (*ep->reg_udccsr & UDCCSR_FS) != 0
				&& read_fifo(ep, req))
			req = 0;

		DMSG("req:%p,ep->desc:%p,ep->dma:%d\n", req, ep->desc, ep->dma);
		if (likely(req && ep->desc))
			pio_irq_enable(ep->ep_num);
	}

	/* pio or dma irq handler advances the queue. */
	if (likely(req))
		list_add_tail(&req->queue, &ep->queue);
	local_irq_restore(flags);

	return 0;
}


/*
 * 	nuke - dequeue ALL requests
 */
void nuke(struct pxa27x_ep *ep, int status)
{
	struct pxa27x_request *req;

	/* called with irqs blocked */
#ifdef	USE_DMA
	if (ep->dma >= 0 && !ep->stopped)
		cancel_dma(ep);
#endif
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct pxa27x_request, queue);
		done(ep, req, status);
	}
	if (ep->desc)
		pio_irq_disable(ep->ep_num);
}


/* dequeue JUST ONE request */
static int pxa27x_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct pxa27x_ep	*ep;
	struct pxa27x_request	*req;
	unsigned long		flags;

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

#ifdef	USE_DMA
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
#else
	done(ep, req, -ECONNRESET);
#endif

	local_irq_restore(flags);
	return 0;
}

/*-------------------------------------------------------------------------*/

static int pxa27x_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct pxa27x_ep	*ep;
	unsigned long		flags;

	DMSG("%s is called\n", __FUNCTION__);
	ep = container_of(_ep, struct pxa27x_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name)) ||
		ep->ep_type == USB_ENDPOINT_XFER_ISOC) {
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

	local_irq_save(flags);

	*ep->reg_udccr = UDCCSR_FST|UDCCSR_FEF;

	/* ep0 needs special care */
	if (!ep->desc) {
		start_watchdog(ep->dev);
		ep->dev->req_pending = 0;
		ep->dev->cdev.ep0state = EP0_STALL;
		LED_EP0_OFF;

	/* and bulk/intr endpoints like dropping stalls too */
	} else {
		unsigned i;

		for (i = 0; i < 1000; i += 20) {
			if (*ep->reg_udccsr & UDCCSR_SST)
				break;
			udelay(20);
		}
	}
	local_irq_restore(flags);

	DBG(DBG_VERBOSE, "%s halt\n", _ep->name);
	return 0;
}

static int pxa27x_ep_fifo_status(struct usb_ep *_ep)
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
			|| (*ep->reg_udccsr & UDCCSR_FS) == 0)
		return 0;
	else
		return (*ep->reg_udcbcr & 0xfff) + 1;
}

static void pxa27x_ep_fifo_flush(struct usb_ep *_ep)
{
	struct pxa27x_ep        *ep;

	ep = container_of(_ep, struct pxa27x_ep, ep);
	if (!_ep || ep->ep.name == ep0name || !list_empty(&ep->queue)) {
		DMSG("%s, bad ep\n", __FUNCTION__);
		return;
	}

	/* toggle and halt bits stay unchanged */

	/* for OUT, just read and discard the FIFO contents. */
	if (!ep->dir_in) {
		while (((*ep->reg_udccsr) & UDCCSR_BNE) != 0)
			(void) *ep->reg_udcdr;
		return;
	}

	/* most IN status is the same, but ISO can't stall */
	*ep->reg_udccsr = UDCCSR_PC|UDCCSR_FST|UDCCSR_TRN
		| (ep->ep_type == USB_ENDPOINT_XFER_ISOC)
			? 0 : UDCCSR_SST;
}


static struct usb_ep_ops pxa27x_ep_ops = {
	.enable		= pxa27x_ep_enable,
	.disable	= pxa27x_ep_disable,

	.alloc_request	= pxa27x_ep_alloc_request,
	.free_request	= pxa27x_ep_free_request,

	.queue		= pxa27x_ep_queue,
	.dequeue	= pxa27x_ep_dequeue,

	.set_halt	= pxa27x_ep_set_halt,
	.fifo_status	= pxa27x_ep_fifo_status,
	.fifo_flush	= pxa27x_ep_fifo_flush,
};


/* ---------------------------------------------------------------------------
 * 	device-scoped parts of the api to the usb controller hardware
 * ---------------------------------------------------------------------------
 */

static int pxa27x_udc_get_frame(struct usb_gadget *_gadget)
{
	return (UDCFNR & 0x3FF);
}

static int pxa27x_udc_wakeup(struct usb_gadget *_gadget)
{
	struct pxa27x_udc *dev;

	dev = container_of(_gadget, struct pxa27x_udc, gadget);
	/* if remote wakeup is not enabled, call SRP */
	if ((UDCCR & UDCCR_DWRE) == 0)  {
		if (dev->transceiver)
			return otg_start_srp(dev->transceiver);
	} else
		UDCCR = (UDCCR & (UDCCR_OEN | UDCCR_UDE)) | UDCCR_UDR;
	return 0;
}

static int pxa27x_udc_vbus_session(struct usb_gadget *_gadget, int is_active)
{
	struct pxa27x_udc *dev;

	dev = container_of(_gadget, struct pxa27x_udc, gadget);
	if (is_active)
		udc_enable(dev);
	else
		udc_disable(dev);
	udc_stop(&dev->cdev, &dev->gadget, dev->driver, !is_active);
	return 0;
}

static const struct usb_gadget_ops pxa27x_udc_ops = {
	.get_frame	 = pxa27x_udc_get_frame,
	.wakeup		 = pxa27x_udc_wakeup,
	/*  current versions must always be self-powered */
	.vbus_session	 = pxa27x_udc_vbus_session,
};

/*-------------------------------------------------------------------------*/

#ifdef UDC_PROC_FILE


static struct pxa27x_udc memory;

static const char proc_node_name [] = "driver/udc";
static const char none [] = "none";

static int
udc_proc_read(char *page, char **start, off_t off, int count,
		int *eof, void *_dev)
{
	char			*buf = page;
	struct pxa27x_udc	*dev = _dev;
	char			*next = buf;
	unsigned		size = count;
	unsigned long		flags;
	int			i, t;
	u32			tmp;

	char		*name = (char *)none;

	if (dev->driver)
		name = (char *)dev->driver->driver.name;

	if (off != 0)
		return 0;

	local_irq_save(flags);

	/* basic device status */
	t = scnprintf(next, size, DRIVER_DESC "\n"
		"%s version: %s\nGadget driver: %s\n",
		driver_name, DRIVER_VERSION SIZE_STR DMASTR,
		(dev->cdev.driver_count > 1) ? "(composite)" : name);
	size -= t;
	next += t;

	/* registers for device and ep0 */
	t = scnprintf(next, size,
		"uicr %02X.%02X, usir %02X.%02x, ufnr %02X\n",
		UDCICR1, UDCICR0, UDCISR1, UDCISR0, UDCFNR);
	size -= t;
	next += t;

#ifdef CONFIG_USB_OTG
	/* register for otg */
	t = scnprintf(next, size, "up2ocr:%02X, udcotgicr:%02X, "
			"udcotgisr:%02X\n", UP2OCR, UDCOTGICR, UDCOTGISR);
	size -= t;
	next += t;
#endif

	tmp = UDCCR;
	t = scnprintf(next, size, "udccr %02X =%s%s%s%s%s%s%s%s%s%s,"
		" con=%d,inter=%d,altinter=%d\n", tmp,
		(tmp & UDCCR_OEN) ? " oen":"",
		(tmp & UDCCR_AALTHNP) ? " aalthnp":"",
		(tmp & UDCCR_AHNP) ? " rem" : "",
		(tmp & UDCCR_BHNP) ? " rstir" : "",
		(tmp & UDCCR_DWRE) ? " dwre" : "",
		(tmp & UDCCR_SMAC) ? " smac" : "",
		(tmp & UDCCR_EMCE) ? " emce" : "",
		(tmp & UDCCR_UDR) ? " udr" : "",
		(tmp & UDCCR_UDA) ? " uda" : "",
		(tmp & UDCCR_UDE) ? " ude" : "",
		(tmp & UDCCR_ACN) >> UDCCR_ACN_S,
		(tmp & UDCCR_AIN) >> UDCCR_AIN_S,
		(tmp & UDCCR_AAISN) >> UDCCR_AAISN_S);

	size -= t;
	next += t;

	tmp = UDCCSR0;
	t = scnprintf(next, size,
		"udccsr0 %02X =%s%s%s%s%s%s%s\n", tmp,
		(tmp & UDCCSR0_SA) ? " sa" : "",
		(tmp & UDCCSR0_RNE) ? " rne" : "",
		(tmp & UDCCSR0_FST) ? " fst" : "",
		(tmp & UDCCSR0_SST) ? " sst" : "",
		(tmp & UDCCSR0_DME) ? " dme" : "",
		(tmp & UDCCSR0_IPR) ? " ipr" : "",
		(tmp & UDCCSR0_OPC) ? " opc" : "");
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
		struct pxa27x_ep	*ep = &dev->ep [i];
		struct pxa27x_request	*req;
		int			t;

		if (i != 0) {
			const struct usb_endpoint_descriptor	*d;

			d = ep->desc;
			if (!d)
				continue;
			tmp = *dev->ep [i].reg_udccsr;
			t = scnprintf(next, size,
				"%s ep_num %d max %d %d udccs %02x udccr:%x\n",
				ep->ep.name, ep->ep_num,
				le16_to_cpu(d->wMaxPacketSize),
				ep->dma, tmp, *dev->ep[i].reg_udccr);
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
			/* TODO translate all five groups of udccs bits! */

		} else /* ep0 should only have one transfer queued */
			t = scnprintf(next, size, "ep0 max 16 pio irqs %lu\n",
				ep->pio_irqs);
		if (t <= 0 || t > size)
			goto done;
		size -= t;
		next += t;

		if (list_empty(&ep->queue)) {
			t = scnprintf(next, size, "\t(nothing queued)\n");
			if (t <= 0 || t > size)
				goto done;
			size -= t;
			next += t;
			continue;
		}
		list_for_each_entry(req, &ep->queue, queue) {
#ifdef	USE_DMA
			if (ep->dma >= 0 && req->queue.prev == &ep->queue)
				t = scnprintf(next, size,
					"\treq %p len %d/%d "
					"buf %p (dma%d dcmd %08x)\n",
					&req->req, req->req.actual,
					req->req.length, req->req.buf,
					ep->dma, DCMD(ep->dma)
					/*  low 13 bits == bytes-to-go */
					);
			else
				t = scnprintf(next, size,
					"\treq %p len %d/%d buf %p\n",
					&req->req, req->req.actual,
					req->req.length, req->req.buf);
#else
			t = scnprintf(next, size,
				"\treq %p len %d/%d buf %p\n",
				&req->req, req->req.actual,
				req->req.length, req->req.buf);
#endif
			if (t <= 0 || t > size)
				goto done;
			size -= t;
			next += t;
		}
	}

done:
	local_irq_restore(flags);
	*eof = 1;
	return count - size;
}

static int udc_proc_write(struct file *filp, const char *buffer,
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
	/* Set IDON */
	case 1:
		/* UP2OCR |= 0x400 ; */
		break;
	/* Clear IDON */
	case 2:
		/* UP2OCR &= ~0x400; */
		break;

	case 3:
		break;

	case 4:
		pxa27x_udc_wakeup(&the_controller->gadget);
		break;

	default:
		return -EINVAL;
	}
	return count;
}

#define create_proc_files() \
	do {	struct proc_dir_entry *ent;\
		ent = create_proc_entry(proc_node_name, 0, NULL);\
		if (ent) { \
			ent->data = dev; \
			ent->read_proc = udc_proc_read; \
			ent->write_proc = udc_proc_write; \
		} \
	} while (0);
#define remove_proc_files() \
	remove_proc_entry(proc_node_name, NULL)

#else	/* !UDC_PROC_FILE */
#define create_proc_files() do {} while (0)
#define remove_proc_files() do {} while (0)

#endif	/* UDC_PROC_FILE */

static void save_eps_config(void)
{
#ifdef CONFIG_PM
	struct pxa27x_udc *dev = the_controller;
	int i;

	for (i = 1; i < UDC_LG_EP_NUM; i++) {
		if (dev->ep[i].assigned) {
			struct pxa27x_ep *ep = &dev->ep[i];

			ep->udccsr_value = *ep->reg_udccsr;
			ep->udccr_value = *ep->reg_udccr;
		}
	}
#endif
}

static void restore_eps_config(void)
{
#ifdef CONFIG_PM
	struct pxa27x_udc *dev = the_controller;
	int i;

	for (i = 1; i < UDC_LG_EP_NUM; i++) {
		if (dev->ep[i].assigned) {
			struct pxa27x_ep *ep = &dev->ep[i];

			*ep->reg_udccsr = ep->udccsr_value;
			*ep->reg_udccr = ep->udccr_value;
		}
	}
#endif
}

/*-------------------------------------------------------------------------*/

/*
 * 	udc_disable - disable USB device controller
 */
static void udc_disable(struct pxa27x_udc *dev)
{
	UDCISR0 = UDCISR1 = 0xffffffff;
	UDCICR0 = UDCICR1 = 0x00000000;

	/* disconnect from bus */
	UP2OCR &= ~(UP2OCR_DPPUE | UP2OCR_DPPDE |
		    UP2OCR_DMPUE | UP2OCR_DMPDE);

	/* save ep configurations */
	save_eps_config();

	/* disable the controller */
	UDCCR = 0;

	ep0_idle(dev);
	dev->gadget.speed = USB_SPEED_UNKNOWN;
	comp_val_init(&dev->cdev);
	LED_CONNECTED_OFF;
}

struct usb_gadget_driver *stop_udc(struct usb_gadget_driver *driver)
{
	struct pxa27x_udc *dev = the_controller;
	int i;

#ifndef CONFIG_USB_COMPOSITE
	/* don't disconnect drivers more than once */
	if (dev->gadget.speed == USB_SPEED_UNKNOWN)
		driver = 0;
#endif
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

/*
 * 	udc_reinit - initialize software state
 */
void udc_reinit()
{
	struct pxa27x_udc *dev = the_controller;
	u32	i;

	/* device/ep0 records init */
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
}

/* until it's enabled, this UDC should be completely invisible
 * to any USB host.
 */
static void udc_enable(struct pxa27x_udc *dev)
{
	ep0_idle(dev);
	dev->gadget.speed = USB_SPEED_FULL;
	dev->stats.irqs = 0;

	/* restore ep configurations */
	restore_eps_config();

	/* enable the controller */
	UDCCR |= UDCCR_UDE;

	/* clear all interrupt status */
	UDCISR0 = 0xffffffff;
	UDCISR1 = 0xf800ffff;


	DMSG("%s, UDCCR = 0x%x\n", __FUNCTION__, UDCCR);

	/* enable suspend/resume and reset irqs */
	UDCICR1 = UDCICR1_IECC | UDCICR1_IERU | UDCICR1_IESU | UDCICR1_IERS;

	/* enable ep0 irqs */
	UDCICR0 = UDCICR_INT(0, UDCICR_INT_MASK);

	UDCCSR0 = UDCCSR0_ACM | UDCCSR0_ODFCLR;

#ifndef CONFIG_USB_OTG
	/* use internal transceiver in a non-OTG way */
	UP2OCR = UP2OCR_HXOE | UP2OCR_DPPUE;
#endif
}

/* the simply ops to assign and release the physical eps
 */
static u32 phy_ep[UDC_EP_NUM];
static void init_ep_assign(void)
{
	phy_ep[0] = 1;
}

int assign_ep(int lg_ep)
{
	struct pxa27x_udc	*dev = the_controller;
	struct pxa27x_ep *ep = &dev->ep[lg_ep];
	u32 i = 0;

	while (++i < UDC_EP_NUM)
		if (!phy_ep[i]) {
			phy_ep[i] = lg_ep;
			list_del(&ep->ep.ep_list);
			list_add_tail(&ep->ep.ep_list, &dev->used_ep_list);
			break;
		}
	if (i == UDC_EP_NUM)
		i = -1;
	return i;
}

static void release_ep(int lg_ep)
{
	struct pxa27x_udc	*dev = the_controller;
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

static inline void set_fifo_size(struct pxa27x_ep *pxa_ep, u8 bmAttributes)
{
	switch (bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) {
	case USB_ENDPOINT_XFER_CONTROL:
		pxa_ep->fifo_size = EP0_FIFO_SIZE;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		pxa_ep->fifo_size = ISO_FIFO_SIZE;
		break;
	case USB_ENDPOINT_XFER_BULK:
		pxa_ep->fifo_size = BULK_FIFO_SIZE;
		break;
	case USB_ENDPOINT_XFER_INT:
		pxa_ep->fifo_size = INT_FIFO_SIZE;
		break;
	default:
		break;
	}
}

static void set_ep(struct usb_endpoint_descriptor *ep_desc, int ep_num,
		   int config, int interface, int alt)
{
	struct pxa27x_udc	*dev = the_controller;
	int k = ep_num;

	dev->ep[k].desc = ep_desc;
	dev->ep[k].assigned = 1;
	dev->ep[k].pio_irqs = dev->ep[k].dma_irqs = 0;
	dev->ep[k].dma = -1;
	if (!(ep_desc->wMaxPacketSize)) {
		set_fifo_size(&(dev->ep[k]), ep_desc->bmAttributes);
		ep_desc->wMaxPacketSize = dev->ep[k].fifo_size;
	} else
		dev->ep[k].fifo_size = ep_desc->wMaxPacketSize;
	dev->ep[k].dir_in = (ep_desc->bEndpointAddress & USB_DIR_IN) ? 1 : 0;
	dev->ep[k].ep_type = ep_desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	dev->ep[k].stopped = 1;
	dev->ep[k].dma_con = 0;
	dev->ep[k].reg_udccsr = &UDCCSR0 + dev->ep[k].ep_num;
	dev->ep[k].reg_udcbcr = &UDCBCR0 + dev->ep[k].ep_num;
	dev->ep[k].reg_udcdr = &UDCDR0 + dev->ep[k].ep_num ;
	dev->ep[k].reg_udccr = &UDCCRA - 1 + dev->ep[k].ep_num;
#ifdef USE_DMA
	dev->ep[k].reg_drcmr = &DRCMR24 + dev->ep[k].ep_num;
#endif
	dev->ep[k].ep.maxpacket = min((ushort)dev->ep[k].fifo_size,
				      ep_desc->wMaxPacketSize);
	dev->ep[k].aisn = alt;
	comp_set_ep(&dev->cdev, k, config, interface);
	DMSG("  found ep num = %d, old interface=%d, config %d intf %d alt %d\n", 
			k, interface, config, interface, alt);
}

/* set_eps
 * assign a physical ep to logic one, and
 * fill pxa_ep structure with their configuration, interface, alternate
 * settings, assigned interface number
 */
int set_eps(__u8 num_eps, struct usb_endpoint_descriptor *p_ep_desc, int len,
		int config, int interface, int alt)
{
	struct pxa27x_udc	*dev = the_controller;
	struct usb_endpoint_descriptor *ep_desc = p_ep_desc;
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
					DMSG("%s:ep %d config %d intf %d alt %d\n",
					    	__func__, k, config, interface, alt);
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

/* eps_config
 * fill the ep config table
 */
static int eps_config(struct pxa27x_udc *dev)
{
	struct pxa27x_ep *ep = NULL;
	uint32_t udccr;
	int config, interface;
	int i;

	for (i = 1; i < UDC_LG_EP_NUM; i++) {
		if (!dev->ep[i].assigned)
			continue;

		udccr = 0;
		ep = &dev->ep[i];
		config = comp_calc_config(&dev->cdev, i);
		interface = comp_calc_interface(&dev->cdev, i);

		udccr |= (config << UDCCONR_CN_S) & UDCCONR_CN;
		udccr |= (interface << UDCCONR_IN_S) & UDCCONR_IN;
		udccr |= (ep->aisn << UDCCONR_AISN_S) & UDCCONR_AISN;
		udccr |= (i << UDCCONR_EN_S) & UDCCONR_EN;
		udccr |= (ep->ep_type << UDCCONR_ET_S) & UDCCONR_ET;
		udccr |= (ep->dir_in) ? UDCCONR_ED : 0;
		udccr |= (ep->fifo_size << UDCCONR_MPS_S) & UDCCONR_MPS;
		udccr |= UDCCONR_DE | UDCCONR_EE;

		*ep->reg_udccr = udccr;
		DMSG("ep %d udccr %x, config %d intf %d\n", 
			i, *ep->reg_udccr, config, interface);
	}

	save_eps_config();
	return 0;
}

/* when a driver is successfully registered, it will receive
 * control requests including set_configuration(), which enables
 * non-control requests.  then usb traffic follows until a
 * disconnect is reported.  then a host may connect again, or
 * the driver might get unbound.
 */
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct pxa27x_udc	*dev = the_controller;
	int			retval = 0;

	if (!driver 	|| driver->speed != USB_SPEED_FULL
			|| !driver->bind
			|| !driver->disconnect
			|| !driver->setup
			) {
		printk(KERN_ERR "gadget driver parameter invalid\n");
		return -EINVAL;
	}
	if (!dev)
		return -ENODEV;

	if (comp_is_dev_busy(&dev->cdev, dev->driver))
		return -EBUSY;

	/* set device constraint */
	set_dvfm_constraint();

	/* when adding a configuration, we need stop the previous
	 * configuration to let host enumerate
	 */
	DMSG("dev=0x%x, driver=0x%x, speed=%d,"
			"bind=0x%x, unbind=0x%x, disconnect=0x%x, setup=0x%x\n",
			(unsigned)dev, (unsigned)driver, driver->speed,
			(unsigned)driver->bind, (unsigned)driver->unbind,
			(unsigned)driver->disconnect, (unsigned)driver->setup);
	local_irq_disable();
	udc_disable(dev);
	stop_gadget(&dev->cdev, &dev->gadget, dev->driver);
	local_irq_enable();

	pr_debug("register gadget driver \n");

	/* Enable clock for USB device */
	clk_enable(dev->clk);
	udc_disable(dev);	/* for some processors, it will enable
				   UDE after enbling the UDC clock!*/

	/* first hook up the driver ... */
	dev->driver = driver;
	dev->gadget.dev.driver = &driver->driver;

	list_add_tail(&dev->used_ep_list, &dev->gadget.ep_list);
	list_del(&dev->gadget.ep_list);
	list_add_tail(&dev->gadget.ep_list, &dev->unused_ep_list);
	list_del(&dev->unused_ep_list);

	if (dev->cdev.driver_count == 0) {
		retval = device_add(&dev->gadget.dev);
		if (retval) {
			printk(KERN_ERR "cannot add device, ret: %d\n",
			       retval);
			goto err1;
		}
	}

	retval = driver->bind(&dev->gadget); /* will set gadget_data */
	if (retval) {
		printk(KERN_ERR "bind to driver %s --> error %d\n",
				driver->driver.name, retval);
		goto err2;
	}

	comp_register_driver(&dev->cdev, &dev->gadget, driver);

	/* set ep config according to the configuration from the last function*/
	eps_config(dev);

	list_add_tail(&dev->unused_ep_list, &dev->gadget.ep_list);
	list_del(&dev->gadget.ep_list);
	list_add_tail(&dev->gadget.ep_list, &dev->used_ep_list);
	list_del(&dev->used_ep_list);

	/* ... then enable host detection and ep0; and we're ready
	 * for set_configuration as well as eventual disconnect.
	 * NOTE:  this shouldn't power up until later.
	 */
	DMSG("registered gadget driver '%s'\n", driver->driver.name);

	/* If OTG is enabled, USB client should be open all the way */
#if defined(ENABLE_CABLE_DETECT) || defined(CONFIG_USB_OTG)
	if (is_cable_attached())
		set_dvfm_constraint();
#endif

	udc_enable(dev);

	pmic_callback_register(PMIC_EVENT_USB, pxa27x_usb_event_change);
#ifdef ENABLE_CABLE_DETECT
	pxa3xx_pmic_set_pump(1);
#endif

#ifdef CONFIG_USB_OTG
	otg_set_peripheral(dev->transceiver, &dev->gadget);
#endif
	dump_state(dev);
	dev->cdev.driver_count++;
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

	return retval;
}
EXPORT_SYMBOL(usb_gadget_register_driver);

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	int i;
	struct pxa27x_udc	*dev = the_controller;

	if (!dev)
		return -ENODEV;
	if (!comp_check_driver(&dev->cdev, dev->driver, driver))
		return -EINVAL;

#if defined(ENABLE_CABLE_DETECT) || defined(CONFIG_USB_OTG)
	if (is_cable_attached()) {
		unset_dvfm_constraint();
	}
#endif
	set_dvfm_constraint();

	local_irq_disable();
 	if (dev->cdev.driver_count == 1) {
#ifdef CONFIG_USB_OTG
		otg_set_peripheral(dev->transceiver, NULL);
#endif
	}
	udc_disable(dev);

 	stop_cur_gadget(&dev->cdev, &dev->gadget, driver);
	local_irq_enable();

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
	comp_unregister_driver(&dev->cdev, &dev->gadget, &dev->driver, driver);
	/* del the gadget abstract device */
	if (dev->cdev.driver_count == 0) {
		device_del(&dev->gadget.dev);
		dev->driver = 0;
	}
	udc_init_ep(dev);

#ifdef ENABLE_CABLE_DETECT
	connected = 0;
	pxa3xx_pmic_set_pump(0);
#endif
	pmic_callback_unregister(PMIC_EVENT_USB, pxa27x_usb_event_change);

	DMSG("unregistered gadget driver '%s'\n", driver->driver.name);
	dump_state(dev);

	/* Disable clock for USB device */
	clk_disable(dev->clk);
	unset_dvfm_constraint();
	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

#ifndef	enable_disconnect_irq
#define	enable_disconnect_irq()		do {} while (0)
#define	disable_disconnect_irq()	do {} while (0)
#endif


/*-------------------------------------------------------------------------*/

static inline void clear_ep_state(struct pxa27x_udc *dev)
{
	unsigned i;

	/* hardware SET_{CONFIGURATION,INTERFACE} automagic resets endpoint
	 * fifos, and pending transactions mustn't be continued in any case.
	 */
	for (i = 1; i < UDC_LG_EP_NUM; i++)
		nuke(&dev->ep[i], -ECONNABORTED);
}

static void udc_watchdog(unsigned long _dev)
{
	struct pxa27x_udc	*dev = (void *)_dev;
	unsigned long flags;

	local_irq_save(flags);
	if (dev->cdev.ep0state == EP0_STALL
			&& (UDCCSR0 & UDCCSR0_FST) == 0
			&& (UDCCSR0 & UDCCSR0_SST) == 0) {
		UDCCSR0 = UDCCSR0_ACM|UDCCSR0_FST|UDCCSR0_FTF|UDCCSR0_ODFCLR;
		DBG(DBG_VERBOSE, "ep0 re-stall\n");
		start_watchdog(dev);
	}
	local_irq_restore(flags);
}

static void ep0_ctrl_request(struct pxa27x_udc *dev, struct pxa27x_ep *ep0)
{
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
		if (unlikely(!(UDCCSR0 & UDCCSR0_RNE))) {
bad_setup:
			DMSG("SETUP %d!, UDCBCR0 %x UDCCSR0"
			     " %x\n", i, UDCBCR0, UDCCSR0);
			goto stall;
		}
		u.word [i] =  UDCDR0;
	}
	if (unlikely((UDCCSR0 & UDCCSR0_RNE) != 0))
		goto bad_setup;

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
	} else {
		i = comp_ep0_req(&dev->cdev, &dev->gadget, &ep0->ep, &u.r);
	} /* if(dev->cdev.driver_count == 1) */

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
		    "udccsr0 %02x err %d\n", UDCCSR0, i);
stall:
		/* the watchdog timer helps deal with cases
		 * where udc seems to clear FST wrongly, and
		 * then NAKs instead of STALLing.
		 * watchdog not necessary for UDC ?????
		 */
		ep0start(dev, UDCCSR0_FST | UDCCSR0_FTF, "stall");
		start_watchdog(dev);
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
	u32			udccsr0 = UDCCSR0;
	struct pxa27x_ep	*ep = &dev->ep [0];
	struct pxa27x_request	*req;

	DMSG("%s is called, ep0 state:%d\n", __FUNCTION__, dev->cdev.ep0state);
	if (list_empty(&ep->queue))
		req = 0;
	else
		req = list_entry(ep->queue.next, struct pxa27x_request, queue);

	/* clear stall status */
	if (udccsr0 & UDCCSR0_SST) {
		nuke(ep, -EPIPE);
		UDCCSR0 = UDCCSR0_ACM|UDCCSR0_SST|UDCCSR0_ODFCLR;
		del_timer(&dev->timer);
		ep0_idle(dev);
	}

	/* previous request unfinished?  non-error iff back-to-back ... */
	if ((udccsr0 & UDCCSR0_SA) != 0 && dev->cdev.ep0state != EP0_IDLE) {
		DMSG("handle_ep0: Setup command again\n");
		nuke(ep, 0);
		del_timer(&dev->timer);
		ep0_idle(dev);
	}

	switch (dev->cdev.ep0state) {
	case EP0_NO_ACTION:
		printk(KERN_INFO"%s: Busy\n", __FUNCTION__);
		/*Fall through */
	case EP0_IDLE:
		/* late-breaking status? */
		udccsr0 = UDCCSR0;
		DMSG("%s EP0_IDLE udccsr0:0x%08x\n", __FUNCTION__, udccsr0);
		/* start control request? */
		if (likely((udccsr0 & (UDCCSR0_OPC|UDCCSR0_SA|UDCCSR0_RNE))
				== (UDCCSR0_OPC|UDCCSR0_SA|UDCCSR0_RNE))) {

			ep0_ctrl_request(dev, ep);
			/* expect at least one data or status stage irq */
			return;

		} else {
			/* some random early IRQ:
			 * - we acked FST
			 * - IPR cleared
			 * - OPC got set, without SA (likely status stage)
			 */
			UDCCSR0 = udccsr0 & (UDCCSR0_ACM | UDCCSR0_SA |
					     UDCCSR0_OPC | UDCCSR0_ODFCLR);
		}
		break;
	case EP0_IN_DATA_PHASE:			/* GET_DESCRIPTOR etc */
		if (udccsr0 & UDCCSR0_OPC) {
			UDCCSR0 = UDCCSR0_ACM | UDCCSR0_OPC |
				  UDCCSR0_FTF | UDCCSR0_ODFCLR;
			DBG(DBG_VERBOSE, "ep0in premature status\n");
			if (req)
				done(ep, req, 0);
			ep0_idle(dev);
		} else /* irq was IPR clearing */ {
			if (req) {
				/* this IN packet might finish the request */
				(void) write_ep0_fifo(ep, req);
			} /* else IN token before response was written */
		}
		break;
	case EP0_OUT_DATA_PHASE:		/* SET_DESCRIPTOR etc */
		if (UDCCSR0 & UDCCSR0_OPC) {
			if (req) {
				/* this OUT packet might finish the request */
				if (read_ep0_fifo(ep, req)) {
					UDCCSR0 = UDCCSR0_IPR|UDCCSR0_ODFCLR;
					done(ep, req, 0);
				}
				/* else more OUT packets expected */
			} /* else OUT token before read was issued */
		} else /* irq was IPR clearing */ {
			DBG(DBG_VERBOSE, "ep0out premature status\n");
			if (req)
				done(ep, req, 0);
			ep0_idle(dev);
		}
		break;
	case EP0_STALL:
		UDCCSR0 = UDCCSR0_ACM|UDCCSR0_FST|UDCCSR0_ODFCLR;
		break;
	case EP0_IN_FAKE:
		printk(KERN_ERR"%s: impossible come here\n", __FUNCTION__);
		break;
	}
/* 	UDCISR0 = UDCISR_INT(0, UDCISR_INT_MASK); */
}


static void handle_ep(struct pxa27x_ep *ep)
{
	struct pxa27x_request	*req, *req_next;
	int			completed, length, remain;
	u32			udccsr = 0;

	DMSG("%s is called, ep num:%d, in:%d\n", __FUNCTION__, ep->ep_num,
	     ep->dir_in);
	do {
		completed = 0;
		if (likely(!list_empty(&ep->queue))) {
			req = list_entry(ep->queue.next,
					struct pxa27x_request, queue);
		} else
			req = 0;

		DMSG("%s: req:%p, udcisr0:0x%x udccsr %p:0x%x\n", __FUNCTION__,
				req, UDCISR0, ep->reg_udccsr, *ep->reg_udccsr);
		if (unlikely(ep->dir_in)) {
			udccsr = (UDCCSR_SST | UDCCSR_TRN) & *ep->reg_udccsr;
			if (unlikely(udccsr))
				*ep->reg_udccsr = udccsr;

			if (req && likely((*ep->reg_udccsr & UDCCSR_FS) != 0))
				completed = write_fifo(ep, req);

		} else { /*  for out endpoints */
			udccsr = (UDCCSR_SST | UDCCSR_TRN) & *ep->reg_udccsr;
			if (unlikely(udccsr))
				*ep->reg_udccsr = udccsr;
#ifdef USE_DMA
		/* DMA enabled, Short packet received, transfer completed*/
			if (req && (ep->dma > 0) &&
				(*ep->reg_udccsr & UDCCSR_SP)) {
				DMSG("dcsr:%x, dcmd:%x\n", DCSR(ep->dma),
				     DCMD(ep->dma));
				DCSR(ep->dma) &= ~DCSR_RUN;
				remain = DCMD(ep->dma) & DCMD_LENGTH;
				length = req->req.length - req->req.actual -
					 remain;
				DMSG("%s, buf:%p, actual:%d,length:%d\n", \
						__FUNCTION__, req->req.buf, \
						req->req.actual, length);
				memcpy((char *)req->req.buf + req->req.actual,
				       ep->dma_buf_virt, length);
				req->req.actual += length ;
				*ep->reg_udccsr = UDCCSR_PC | UDCCSR_DME;

				if (req->queue.next != &ep->queue) {
					req_next = list_entry(req->queue.next,
						struct pxa27x_request, queue);
					kick_dma(ep, req_next);
				}
				done(ep, req, 0);
			/* fifos can hold packets, ready for reading... */
			} else
				if (likely(req)) {
					completed = read_fifo(ep, req);
				} else {
					pio_irq_disable(ep->ep_num);
					*ep->reg_udccsr = UDCCSR_FEF |
							  UDCCSR_PC;
					DMSG("%s: no req for out data\n",
						__FUNCTION__);
				}
#else
			if (likely(req)) {
				completed = read_fifo(ep, req);
			} else {
				pio_irq_disable(ep->ep_num);
				*ep->reg_udccsr = UDCCSR_FEF | UDCCSR_PC;
				DMSG("%s: no req for out data\n",
					__FUNCTION__);
			}
#endif
		}
		ep->pio_irqs++;
	} while (completed);
}

static void pxa27x_change_configuration(struct pxa27x_udc *dev)
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

static void pxa27x_change_interface(struct pxa27x_udc *dev)
{
	struct usb_ctrlrequest req;
	int active_interface = (UDCCR & UDCCR_AIN) >> UDCCR_AIN_S, ret;

	DMSG("%s\n", __FUNCTION__);

	req.bRequestType = USB_RECIP_INTERFACE;
	req.bRequest = USB_REQ_SET_INTERFACE;
	req.wValue = dev->cdev.alternate;
	req.wIndex = dev->cdev.interface;
	req.wLength = 0;

	dev->driver = comp_change_interface(&dev->cdev, active_interface, &req,
					    &dev->gadget, dev->driver, &ret);
	dev->gadget.dev.driver = &((struct usb_gadget_driver *)
				   (dev->driver))->driver;
	UDCCSR0 = UDCCSR0_ACM | UDCCSR0_AREN | UDCCSR0_ODFCLR; /* xj add */
}

/*
 *	pxa27x_udc_irq - interrupt handler
 *
 * avoid delays in ep0 processing. the control handshaking isn't always
 * under software control (pxa250c0 and the pxa255 are better), and delays
 * could cause usb protocol errors.
 */
static irqreturn_t pxa27x_udc_irq(int irq, void *_dev)
{
	struct pxa27x_udc	*dev = _dev;
	int			handled;
	u32 udcir;

	dev->stats.irqs++;
	HEX_DISPLAY(dev->stats.irqs);

	DMSG("\n");
	DBG(DBG_VERBOSE, "Interrupt, UDCISR0:0x%08x, UDCISR1:0x%08x, "
			"UDCCR:0x%08x\n", UDCISR0, UDCISR1, UDCCR);
#ifdef CONFIG_USB_OTG
	DMSG("UPO2CR:0x%08x, UDCOTGICR:0x%08x, UDCOTGISR:0x%08x\n",
			UP2OCR, UDCOTGICR, UDCOTGISR);
#endif

irq_loop:

#ifdef	CONFIG_USB_OTG
	if (UDCOTGISR)
		otg_interrupt(dev->transceiver);
#endif

	udcir = UDCISR1 & 0xF8000000;
	handled = 0;

	/* SUSpend Interrupt Request */
	if (unlikely(udcir & UDCISR1_IRSU)) {
		UDCISR1 = UDCISR1_IRSU;
		handled = 1;
		DBG(DBG_VERBOSE, "USB suspend\n");
		if (dev->gadget.speed != USB_SPEED_UNKNOWN && dev->driver)
			comp_driver_suspend(&dev->cdev, &dev->gadget, dev->driver);
		ep0_idle(dev);

#ifdef CONFIG_USB_OTG
		otg_host_suspend(dev->transceiver);
		goto irq_loop;
#endif
	}

	/* RESume Interrupt Request */
	if (unlikely(udcir & UDCISR1_IRRU)) {
		UDCISR1 = UDCISR1_IRRU;
		handled = 1;
		DBG(DBG_VERBOSE, "USB resume\n");

		if (dev->gadget.speed != USB_SPEED_UNKNOWN && dev->driver)
			comp_driver_resume(&dev->cdev, &dev->gadget, dev->driver);
	}

	if (unlikely(udcir & UDCISR1_IRCC)) {
		unsigned config, interface, alternate;

		handled = 1;
		DBG(DBG_VERBOSE, "USB SET_CONFIGURATION or "
			"SET_INTERFACE command received\n");

		UDCCR |= UDCCR_SMAC;

		UDCISR1 = UDCISR1_IRCC;
		config = (UDCCR & UDCCR_ACN) >> UDCCR_ACN_S;
		interface =  (UDCCR & UDCCR_AIN) >> UDCCR_AIN_S;
		alternate = (UDCCR & UDCCR_AAISN) >> UDCCR_AAISN_S;

		DBG(DBG_VERBOSE, "config=%d, interface=%d, alter=%d\n",
			config, interface, alternate);

		if (dev->cdev.configuration != config) {
			dev->cdev.configuration = config;
			pxa27x_change_configuration(dev);
		} else if ((dev->cdev.interface != interface) || \
				(dev->cdev.interface == 0) || \
				(dev->cdev.alternate != alternate)) {
			dev->cdev.interface = interface;
			dev->cdev.alternate = alternate;
			pxa27x_change_interface(dev);
		} else
			UDCCSR0 = UDCCSR0_ACM | UDCCSR0_AREN |
				  UDCCSR0_ODFCLR;

		DMSG("%s: con:%d,inter:%d,alt:%d\n",
			__FUNCTION__, config, interface, alternate);
	}

	/* ReSeT Interrupt Request - USB reset */
	if (unlikely(udcir & UDCISR1_IRRS)) {
		UDCISR1 = UDCISR1_IRRS;
		handled = 1;

		if ((UDCCR & UDCCR_UDA) == 0) {
			DBG(DBG_VERBOSE, "USB reset start\n");

			/* reset driver and endpoints,
			 * in case that's not yet done
			 */
			stop_gadget(&dev->cdev, &dev->gadget, dev->driver);
		}
		DMSG("USB reset\n");
		dev->gadget.speed = USB_SPEED_FULL;
		memset(&dev->stats, 0, sizeof dev->stats);

	}

	if (UDCISR0 || (UDCISR1 & 0xFFFF)) {
		u32	udcisr0 = UDCISR0 ;
		u32	udcisr1 = UDCISR1 & 0xFFFF;
		int	i;

		DBG(DBG_VERY_NOISY, "irq %02x.%02x\n", udcisr1, udcisr0);

		/* control traffic */
		if (udcisr0 & UDCISR0_IR0) {
			DMSG("handle_ep0: UDCISR0:%x, UDCCSR0:%x\n",\
					UDCISR0, UDCCSR0);
			UDCISR0 = UDCISR_INT(0, UDCISR_INT_MASK);
			if (udcisr0 & UDC_INT_FIFOERROR)
				pr_err("Endpoint 0 fifo Error\n");

			dev->ep[0].pio_irqs++;
			if (udcisr0 & UDC_INT_PACKETCMP) {
				handle_ep0(dev);
				handled = 1;
			}
		}

		udcisr0 >>= 2;
		/* endpoint data transfers */
		for (i = 1; udcisr0 != 0 && i < 16; udcisr0 >>= 2, i++) {
			UDCISR0 = UDCISR_INT(i, UDCISR_INT_MASK);

			if (udcisr0 & UDC_INT_FIFOERROR)
				pr_warning("Endpoint %d Fifo error\n",
					   i);
			if (udcisr0 & UDC_INT_PACKETCMP) {
				handle_ep(&dev->ep[get_lg_ep(i)]);
				handled = 1;
			}
		}

		for (i = 0; udcisr1 != 0 && i < 8; udcisr1 >>= 2, i++) {
			UDCISR1 = UDCISR_INT(i, UDCISR_INT_MASK);

			if (udcisr1 & UDC_INT_FIFOERROR) {
				pr_warning("Endpoint %d fifo error\n",
					   (i+16));
			}

			if (udcisr1 & UDC_INT_PACKETCMP) {
				handle_ep(&dev->ep[get_lg_ep(i+16)]);
				handled = 1;
			}
		}
	}

	/* we could also ask for 1 msec SOF (SIR) interrupts */
	if (handled)
		goto irq_loop;

/* 	UDCOTGISR = 0xffffffff; */
	DMSG("IRQ_HANDLED\n");
	return IRQ_HANDLED;
}

static void udc_init_ep(struct pxa27x_udc *dev)
{
	int i;

	init_ep_assign();
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
	.gadget = {
		.ops		= &pxa27x_udc_ops,
		.ep0		= &memory.ep[0].ep,
		.name		= driver_name,
		.dev = {
			.bus_id		= "gadget",
			.release	= nop_release,
		},
	},

	/* control endpoint */
	.ep[0] = {
		.ep = {
			.name		= ep0name,
			.ops		= &pxa27x_ep_ops,
			.maxpacket	= EP0_FIFO_SIZE,
		},
		.dev		= &memory,
		.reg_udccsr	= &UDCCSR0,
		.reg_udcdr	= &UDCDR0,
	},

	/* first group of endpoints */
	.ep[1] = {
		.ep = {
			.name		= "ep1in-bulk",
			.ops		= &pxa27x_ep_ops,
			.maxpacket	= BULK_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= BULK_FIFO_SIZE,
		.ep_num		= 1,
	},
	.ep[2] = {
		.ep = {
			.name		= "ep2out-bulk",
			.ops		= &pxa27x_ep_ops,
			.maxpacket	= BULK_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= BULK_FIFO_SIZE,
		.ep_num		= 2,
	},
	.ep[3] = {
		.ep = {
			.name		= "ep3in-iso",
			.ops		= &pxa27x_ep_ops,
			.maxpacket	= ISO_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= ISO_FIFO_SIZE,
		.ep_num		= 3,
	},
	.ep[4] = {
		.ep = {
			.name		= "ep4out-iso",
			.ops		= &pxa27x_ep_ops,
			.maxpacket	= ISO_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= ISO_FIFO_SIZE,
		.ep_num		= 4,
	},
	.ep[5] = {
		.ep = {
			.name		= "ep5in-int",
			.ops		= &pxa27x_ep_ops,
			.maxpacket	= INT_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= INT_FIFO_SIZE,
		.ep_num		= 5,
	},

	/* second group of endpoints */
	.ep[6] = {
		.ep = {
			.name		= "ep6in-bulk",
			.ops		= &pxa27x_ep_ops,
			.maxpacket	= BULK_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= BULK_FIFO_SIZE,
		.ep_num		= 6,
	},
	.ep[7] = {
		.ep = {
			.name		= "ep7out-bulk",
			.ops		= &pxa27x_ep_ops,
			.maxpacket	= BULK_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= BULK_FIFO_SIZE,
	},
	.ep[8] = {
		.ep = {
			.name		= "ep8in-iso",
			.ops		= &pxa27x_ep_ops,
			.maxpacket	= ISO_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= ISO_FIFO_SIZE,
	},
	.ep[9] = {
		.ep = {
			.name		= "ep9out-iso",
			.ops		= &pxa27x_ep_ops,
			.maxpacket	= ISO_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= ISO_FIFO_SIZE,
	},
	.ep[10] = {
		.ep = {
			.name		= "ep10in-int",
			.ops		= &pxa27x_ep_ops,
			.maxpacket	= INT_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= INT_FIFO_SIZE,
	},

	/* third group of endpoints */
	.ep[11] = {
		.ep = {
			.name		= "ep11in-bulk",
			.ops		= &pxa27x_ep_ops,
			.maxpacket	= BULK_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= BULK_FIFO_SIZE,
	},
	.ep[12] = {
		.ep = {
			.name		= "ep12out-bulk",
			.ops		= &pxa27x_ep_ops,
			.maxpacket	= BULK_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= BULK_FIFO_SIZE,
	},
	.ep[13] = {
		.ep = {
			.name		= "ep13in-iso",
			.ops		= &pxa27x_ep_ops,
			.maxpacket	= ISO_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= ISO_FIFO_SIZE,
	},
	.ep[14] = {
		.ep = {
			.name		= "ep14out-iso",
			.ops		= &pxa27x_ep_ops,
			.maxpacket	= ISO_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= ISO_FIFO_SIZE,
	},
	.ep[15] = {
		.ep = {
			.name		= "ep15in-int",
			.ops		= &pxa27x_ep_ops,
			.maxpacket	= INT_FIFO_SIZE,
		},
		.dev		= &memory,
		.fifo_size	= INT_FIFO_SIZE,
	},
};

/*
 * 	probe - binds to the platform device
 */
static int __init pxa27x_udc_probe(struct platform_device *pdev)
{
	struct pxa27x_udc *dev = &memory;
	struct device *_dev = &pdev->dev;
	int retval;

	dev->clk = clk_get(&pdev->dev, "UDCCLK");

	if (IS_ERR(dev->clk)) {
		retval = PTR_ERR(dev->clk);
		goto err_clk;
	}
	clk_enable(dev->clk);

	/* other non-static parts of init */
	dev->dev = _dev;
	dev->mach = _dev->platform_data;

	init_timer(&dev->timer);
	dev->timer.function = udc_watchdog;
	dev->timer.data = (unsigned long) dev;

	device_initialize(&dev->gadget.dev);
	dev->gadget.dev.parent = _dev;
	dev->gadget.dev.dma_mask = _dev->dma_mask;

	the_controller = dev;
	platform_set_drvdata(pdev, dev);

	udc_disable(dev);
	init_ep_assign();
	udc_init_ep(dev);
	udc_reinit();

#ifdef CONFIG_USB_OTG
	dev->gadget.is_otg = 1;
	dev->transceiver = otg_get_transceiver();
	dev->cdev.transceiver = otg_get_transceiver();
	if (!dev->transceiver) {
		DMSG("failed to get transceiver\n");
	}
#endif
	/* irq setup after old hardware state is cleaned up */
	retval = request_irq(IRQ_USB, pxa27x_udc_irq,
			IRQF_DISABLED, driver_name, dev);
	if (retval != 0) {
		printk(KERN_ERR "%s: can't get irq %i, err %d\n",
			driver_name, IRQ_USB, retval);
		goto err_irq;
	}
	dev->got_irq = 1;

	create_proc_files();

#ifdef UDC_USE_ISRAM
	zyludc_immid = imm_register_kernel("pxa27x_udc");
#endif
	clk_disable(dev->clk);

	return 0;

 err_irq:
	clk_put(dev->clk);
	clk_disable(dev->clk);
 err_clk:
	return retval;
}

static int __exit pxa27x_udc_remove(struct platform_device *pdev)
{
	struct pxa27x_udc *dev = platform_get_drvdata(pdev);

	clk_enable(dev->clk);
	udc_disable(dev);
	remove_proc_files();
	usb_gadget_unregister_driver(dev->driver);

	if (dev->got_irq) {
		free_irq(IRQ_USB, dev);
		dev->got_irq = 0;
	}
#ifdef CONFIG_ARCH_LUBBOCK
	if (machine_is_lubbock() && dev->got_disc) {
		free_irq(LUBBOCK_USB_DISC_IRQ, dev);
		dev->got_disc = 0;
	}
#endif

	clk_put(dev->clk);
	clk_disable(dev->clk);

	platform_set_drvdata(pdev, NULL);
	the_controller = 0;

	return 0;
}

/*
 * Interrupt comes from pmic when usb cable attached or detached
 */
#ifdef ENABLE_CABLE_DETECT
/* detect USB cable attach and detach by PMIC
 * 1 -- cable attached; 0 -- cable detached
 */
int is_cable_attached(void)
{
	struct pxa27x_udc *dev = &memory;

#ifdef ENABLE_CABLE_DETECT
	if (pxa3xx_pmic_is_vbus_assert() && dev->driver) {
		/* VBUS level is high, cable attached */
		connected = 1;
		return 1;
	} else {
		connected = 0;
		return 0;
	}
#endif
	return 1;
}

static int pxa27x_pmic_usb_status(unsigned long status)
{      int ret = 0;

	pr_debug("%s: enter\n", __func__);
	if ((status & PMIC_EVENT_VBUS) && pxa3xx_pmic_is_vbus_assert()) {
		pr_debug("%s: vbus A ssert\n", __func__);
		ret |= USBOTG_VBUS_VALID;
	}

	if ((status & PMIC_EVENT_VBUS) && pxa3xx_pmic_is_srp_ready()) {
		pr_debug("%s: srp ready\n", __func__);
		ret |= USBOTG_SRP_DETECT;
	}

	if ((status & PMIC_EVENT_VBUS) && pxa3xx_pmic_is_avbusvld()) {
		pr_debug("%s: vbus A valid\n", __func__);
		ret |= USBOTG_VBUS_VALID;
	}

	return ret;
}

static void pxa27x_usb_event_change(unsigned long events)
{
	unsigned long flags;
	int status;
	struct pxa27x_udc *dev = the_controller;

	DMSG("%s is called, events:%x\n", __func__, (int)events);
	local_irq_save(flags);
	if (dev && dev->driver && events) {
		/* bit EXTON_N indicate usb cable event */
		status = pxa27x_pmic_usb_status(events);
		if (status & USBOTG_STATUS_REQUIRED) {
			connected = 1;
			set_dvfm_constraint();
			kobject_uevent(&dev->dev->kobj, KOBJ_ADD);
#ifdef CONFIG_USB_OTG
			pxa3xx_otg_require_bus(status);
#else
			udc_enable(dev);
			udc_stop(&dev->cdev, &dev->gadget, dev->driver, 0);
#endif
		} else { /* Cable detached */
			connected = 0;
			kobject_uevent(&dev->dev->kobj, KOBJ_REMOVE);
#ifdef CONFIG_USB_OTG
			pxa3xx_otg_require_bus(status);
#else
			udc_disable(dev);
			udc_stop(&dev->cdev, &dev->gadget, dev->driver, 1);
#endif
			unset_dvfm_constraint();
		}
	}
	local_irq_restore(flags);
}
#else
static void pxa27x_usb_event_change(unsigned long events) {}
#endif

#ifdef CONFIG_PM
static int pxa27x_udc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct pxa27x_udc *dev;
	dev = (struct pxa27x_udc *)platform_get_drvdata(pdev);

	if (dev->driver) {
		DMSG("%s is called\n", __FUNCTION__);
		if (connected) {
			printk(KERN_ERR "Can't make system into susupend"
					"when USB cable is attached\n");
			return -EACCES;
		}
		clk_enable(dev->clk);
#ifdef CONFIG_USB_OTG
		if (dev->transceiver->default_a) {
			printk(KERN_ERR "Can't make system into susupend"
					"when USB cable is attached\n");
			return -EACCES;
		}
#endif

		save_eps_config();

		stop_gadget(&dev->cdev, &dev->gadget, dev->driver);

#ifdef ENABLE_CABLE_DETECT
		connected = 0;
		pxa3xx_pmic_set_pump(0);
#endif

		/* disable all the interrupts */
		UDCICR0 = UDCICR1 = 0;
		UDCISR0 = UDCISR1 = 0xffffffff;

		UP2OCR &= ~(UP2OCR_DPPUE | UP2OCR_DPPDE |
			    UP2OCR_DMPUE | UP2OCR_DMPDE);
#ifdef CONFIG_USB_OTG
		otg_set_peripheral(dev->transceiver, NULL);
#endif

		/* disable the controller */
		UDCCR = 0;
		/* Disable clock for USB device */
		clk_disable(dev->clk);

	}

	return 0;
}

static int pxa27x_udc_resume(struct platform_device *pdev)
{
	struct pxa27x_udc *dev;
	dev = (struct pxa27x_udc *)platform_get_drvdata(pdev);

	if (dev->driver) {
		DMSG("%s is called\n", __FUNCTION__);
		/* Enable clock for USB device */
		clk_enable(dev->clk);

		/* NOTES: we have assumption that the silicon
		 * response for VBUS keep registers value
		 * when doing suspend/resume. So needn't to
		 * set vbus pump here. If this behavior can not
		 * be guaranteed, we need set vbus pump here.
		 *
		 */
#if !defined(ENABLE_CABLE_DETECT) && !defined(CONFIG_USB_OTG)
		udc_enable(dev);
#endif
#ifdef CONFIG_USB_OTG
		udc_enable(dev);
		otg_set_peripheral(dev->transceiver, &dev->gadget);
#endif
		clk_disable(dev->clk);
	}

	return 0;
}

#endif

/*-------------------------------------------------------------------------*/

static struct platform_driver udc_driver = {
	.driver	= {
		.name	= "pxa27x-udc",
	},
	.probe		= pxa27x_udc_probe,
	.remove		= __exit_p(pxa27x_udc_remove),
#ifdef CONFIG_PM
	.suspend 	= pxa27x_udc_suspend,
	.resume 	= pxa27x_udc_resume
#endif
};

static int __init udc_init(void)
{
	printk(KERN_INFO "%s: version %s\n", driver_name, DRIVER_VERSION);
#ifdef CONFIG_PXA3xx_DVFM
	dvfm_register("UDC", &(dvfm_lock.dev_idx));
#endif
	return platform_driver_register(&udc_driver);
}
module_init(udc_init);

static void __exit udc_exit(void)
{
	platform_driver_unregister(&udc_driver);
#ifdef CONFIG_PXA3xx_DVFM
	dvfm_unregister("UDC", &(dvfm_lock.dev_idx));
#endif
}
module_exit(udc_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Frank Becker, Robert Schwebel, David Brownell");
MODULE_LICENSE("GPL");

