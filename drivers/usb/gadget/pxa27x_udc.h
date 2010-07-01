/*
 * linux/drivers/usb/gadget/pxa27x_udc.h
 * Intel PXA3xx on-chip full speed USB device controller
 *
 * Copyright (C) 2003 Robert Schwebel <r.schwebel@pengutronix.de>, Pengutronix
 * Copyright (C) 2003 David Brownell
 * Copyright (C) 2004 Intel Corporation
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

 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

/*
 * The CONFIG_USB_COMPOSITE macro enables the ability to support several
 * gadget drivers to be bould with this driver at the same time and work
 * together. This implements a device that has multiple interfaces con-
 * trolled independently of each other, which is referred to as a "compo-
 * site" device in the USB 2.0 spec.
 * As the original design of gadget driver doesn't support several ones
 * to work together, here list some limitations in the current version
 * implemented here:
 * 1. Only support gadget's default configuration
 *    the device configuration is not static, will be the gadget config
 *    value first inserted. Other configuration other than the default
 *    in one gadget driver won't be viewed by USB host.
 * 2. Gadget won't keep gadget driver's private data for each one any
 *    more
 *    gadget driver should maintain the driver's private data themselves,
 *    but not in the gadget any more. e.g. "driver_data" in struct usb_ep
 *    for ep0, and driver_data in "dev" for struct usb_gadget.
 * 3. Consider every gadget driver implements only one function
 *    For gadget driver that already has multiple interface and each
 *    implements a different function, we don't know that it's not one
 *    function and will consider it as one function with multiple
 *    interfaces. And software will add the interface association
 *    descriptor to USB host, which is unnecessary.
 * 4. Potentail conflict exists in class/vendor specific requests
 *    through the default pipe
 *    For class/vendor specific requests that also use the default
 *    pipe, it is hard to determine which gadget driver shoule the
 *    requset to be passed to. The current implementation will give the
 *    request to every gadget driver until it doesn't return EOPNOTSUPP.
 *    This will cause a little delay to the request reponse. And we have
 *    the premiss that no class/vendor specific requests are the same
 *    in two gadget drivers.
 * 5. String descriptors not well supported yet
 *    String descriptors are not well supported when multiple gadget
 *    drivers are inserted. It would pass the request to the "active"
 *    (gadget driver module last registered or with the interface that
 *    last SET_INTERFACE command issued.
 * 6. rmmod issue
 *    please remove all the modules simultaneously when change combination
 *    of the function.
 */

#ifndef __LINUX_USB_GADGET_PXA3XX_H
#define __LINUX_USB_GADGET_PXA3XX_H

#include <linux/types.h>

#ifdef CONFIG_USB_PXA3XX_U2D
/* USB 2.0 device controller */
#define U2DCR           __REG_3(0x54100000)	/* U2D Control Register */
#define U2DCR_NDC              (1 << 31)        /* NAK During Config */
#define U2DCR_HSTC_MASK	(0x7 << 28)		/* High Speed Timeout Calibration */
#define U2DCR_HSTC_S	(28)			/* shift */
#define U2DCR_SPEOREN          (1 << 27)        /* Short Packet EOR INTR generation Enable */
#define U2DCR_FSTC_MASK	(0x7 << 24)		/* Full Speed Timeout Calibration */
#define U2DCR_FSTC_S	(24)			/* shift */
#define U2DCR_UCLKOVR	(1 << 22)		/* UTM Clock Override */
#define U2DCR_ABP		(1 << 21)	/* Application Bus Power */
#define U2DCR_ADD		(1 << 20)	/* Application Device Disconnect */
#define U2DCR_CC		(1 << 19)	/* Configuration Change */
#define U2DCR_HS		(1 << 18)	/* High Speed USB Detection */
#define U2DCR_SMAC		(1 << 17)	/* Switch Endpoint Memofy to Actuve Configuration */
#define U2DCR_DWRE		(1 << 16)	/* Device Remote Wake-up Feature */
#define U2DCR_ACN		(0xf << 12)	/* Active U2D Configuration Number */
#define U2DCR_ACN_S		12		/* shift */
#define U2DCR_AIN		(0xf << 8)	/* Active U2D Interface Number */
#define U2DCR_AIN_S		8		/* shift */
#define U2DCR_AAISN		(0xf << 4)	/* Active U2D Alternate Interface Setting Number */
#define U2DCR_AAISN_S	4			/* shift */
#define U2DCR_EMCE		(1 << 3)	/* Endpoint Memory Configuration Error */
#define U2DCR_UDR		(1 << 2)	/* U2D Resume */
#define U2DCR_UDA		(1 << 1)	/* U2D Active */
#define U2DCR_UDE		(1 << 0)	/* U2D Enable */
#define U2DCR_MASK_BITS (U2DCR_CC | U2DCR_SMAC | U2DCR_EMCE | U2DCR_UDR)

#define U2DICR          __REG_3(0x54100004)	/* U2D Interrupt Control Register */
#define U2DISR          __REG_3(0x5410000C)	/* U2D Interrupt Status Register */
#define U2DINT_CC		(1 << 31)	/* Interrupt - Configuration Change */
#define U2DINT_SOF		(1 << 30)	/* Interrupt - SOF */
#define U2DINT_USOF		(1 << 29)	/* Interrupt - micro SOF */
#define U2DINT_RU		(1 << 28)	/* Interrupt - Resume */
#define U2DINT_SU		(1 << 27)	/* Interrupt - Suspend */
#define U2DINT_RS		(1 << 26)	/* Interrupt - Reset */
#define U2DINT_DPE		(1 << 25)	/* Interrupt - Data Packet Error */
#define U2DINT_FIFOERR		(0x4)		/* Interrupt - endpoint FIFO error */
#define U2DINT_PACKETCMP	(0x2)		/* Interrupt - endpoint packet complete */
#define U2DINT_SPACKETCMP	(0x1)		/* Interrupt - endpoint short packet complete */

#define U2DFNR          __REG_3(0x54100014)	/* U2D Frame Number Register */

#define U2DINT(n, intr)                 (((intr) & 0x07) << (((n) & 0x07) * 3))
#define U2DICR2                __REG_3(0x54100008)      /* U2D Interrupt Control Register 2 */
#define U2DISR2                __REG_3(0x54100010)      /* U2D Interrupt Status Register 2 */

#define U2DOTGCR               __REG_3(0x54100020)      /* U2D OTG Control Register */
#define U2DOTGCR_OTGEN         (1 << 31)                /* On-The-Go Enable */
#define U2DOTGCR_AALTHNP       (1 << 30)                /* A-device Alternate Host Negotiation Protocal Port Support */
#define U2DOTGCR_AHNP          (1 << 29)                /* A-device Host Negotiation Protocal Support */
#define U2DOTGCR_BHNP          (1 << 28)                /* B-device Host Negotiation Protocal Enable */

#ifdef CONFIG_CPU_PXA930
#define U2DOTGCR_LPA		(1 << 15)	/* ULPI low power mode active */
#define U2DOTGCR_IESI		(1 << 13)	/* OTG interrupt Enable */
#define U2DOTGCR_ISSI		(1 << 12)	/* OTG interrupt status */
#endif

#define U2DOTGCR_CKAF          (1 << 5)                         /* Carkit Mode Alternate Function Select */
#define U2DOTGCR_UTMID         (1 << 4)                         /* UTMI Interface Disable */
#define U2DOTGCR_ULAF          (1 << 3)                         /* ULPI Mode Alternate Function Select */
#define U2DOTGCR_SMAF          (1 << 2)                         /* Serial Mode Alternate Function Select */
#define U2DOTGCR_RTSM          (1 << 1)                         /* Return to Synchronous Mode (ULPI Mode) */
#define U2DOTGCR_ULE           (1 << 0)                         /* ULPI Wrapper Enable */

#define U2DOTGICR              __REG_3(0x54100024)              /* U2D OTG Interrupt Control Register */
#define U2DOTGISR              __REG_3(0x54100028)              /* U2D OTG Interrupt Status Register */

#define U2DOTGINT_SF           (1 << 17)                        /* OTG Set Feature Command Received */
#define U2DOTGINT_SI           (1 << 16)                        /* OTG Interrupt */
#define U2DOTGINT_RLS1         (1 << 14)                        /* RXCMD Linestate[1] Change Interrupt Rise */
#define U2DOTGINT_RLS0         (1 << 13)                        /* RXCMD Linestate[0] Change Interrupt Rise */
#define U2DOTGINT_RID          (1 << 12)                        /* RXCMD OTG ID Change Interrupt Rise */
#define U2DOTGINT_RSE          (1 << 11)                        /* RXCMD OTG Session End Interrupt Rise */
#define U2DOTGINT_RSV          (1 << 10)                        /* RXCMD OTG Session Valid Interrupt Rise */
#define U2DOTGINT_RVV          (1 << 9)                         /* RXCMD OTG Vbus Valid Interrupt Rise */
#define U2DOTGINT_RCK          (1 << 8)                         /* RXCMD Carkit Interrupt Rise */
#define U2DOTGINT_FLS1         (1 << 6)                         /* RXCMD Linestate[1] Change Interrupt Fall */
#define U2DOTGINT_FLS0         (1 << 5)                         /* RXCMD Linestate[0] Change Interrupt Fall */
#define U2DOTGINT_FID          (1 << 4)                         /* RXCMD OTG ID Change Interrupt Fall */
#define U2DOTGINT_FSE          (1 << 3)                         /* RXCMD OTG Session End Interrupt Fall */
#define U2DOTGINT_FSV          (1 << 2)                         /* RXCMD OTG Session Valid Interrupt Fall */
#define U2DOTGINT_FVV          (1 << 1)                         /* RXCMD OTG Vbus Valid Interrupt Fall */
#define U2DOTGINT_FCK          (1 << 0)                         /* RXCMD Carkit Interrupt Fall */

#define U2DOTGUSR              __REG_3(0x5410002C)              /* U2D OTG ULPI Status Register */
#define U2DOTGUSR_LPA          (1 << 31)                        /* ULPI Low Power Mode Active */
#define U2DOTGUSR_S6A          (1 << 30)                        /* ULPI Serial Mode (6-pin) Active */
#define U2DOTGUSR_S3A          (1 << 29)                        /* ULPI Serial Mode (3-pin) Active */
#define U2DOTGUSR_CKA          (1 << 28)                        /* ULPI Car Kit Mode Active */
#define U2DOTGUSR_LS1          (1 << 6)                         /* RXCMD Linestate 1 Status */
#define U2DOTGUSR_LS0          (1 << 5)                         /* RXCMD Linestate 0 Status */
#define U2DOTGUSR_ID           (1 << 4)                         /* OTG IDGnd Status */
#define U2DOTGUSR_SE           (1 << 3)                         /* OTG Session End Status */
#define U2DOTGUSR_SV           (1 << 2)                         /* OTG Session Valid Status */
#define U2DOTGUSR_VV           (1 << 1)                         /* OTG Vbus Valid Status */
#define U2DOTGUSR_CK           (1 << 0)                         /* Carkit Interrupt Status */

#define U2DOTGUCR              __REG_3(0x54100030)              /* U2D OTG ULPI Control Register */
#define U2DOTGUCR_RUN          (1 << 25)                        /* RUN */
#define U2DOTGUCR_RNW          (1 << 24)                        /* Read or Write operation */
#define U2DOTGUCR_ADDR         (0x3f << 16)                     /* Address of the ULPI PHY register to be accessed */
#define U2DOTGUCR_ADDR_S       16                               /* shift */
#define U2DOTGUCR_WDATA                (0xff << 8)              /* The data for a WRITE command */
#define U2DOTGUCR_WDATA_S      8                                /* shift */
#define U2DOTGUCR_RDATA                (0xff << 0)              /* The data for a READ command */
#define U2DOTGUCR_RDATA_S      0                                /* shift */

#define U2DP3CR                __REG_3(0x54100034)              /* U2D Port 3 Control Register */
#define U2DP3CR_P2SS           (0x3 << 8)                       /* Host Port 2 Serial Mode Select */
#define U2DP3CR_P2SS_S         8                                /* shift */
#define U2DP3CR_P3SS           (0x7 << 4)                       /* Host Port 3 Serial Mode Select */
#define U2DP3CR_P3SS_S         4                                /* shift */
#define U2DP3CR_VPVMBEN                (1 << 1)                 /* Host Port 3 Vp/Vm Block Enable */
#define U2DP3CR_CFG            (1 << 0)                         /* Host Port 3 Configuration */

#define U2DCSR0         __REG_3(0x54100100)			/* U2D Control/Status Register - Endpoint 0 */
#define U2DCSR0_IPA		(1 << 8)			/* IN Packet Adjusted */
#define U2DCSR0_SA		(1 << 7)			/* SETUP Active */
#define U2DCSR0_RNE		(1 << 6)			/* Receive FIFO Not Empty */
#define U2DCSR0_FST		(1 << 5)			/* Force Stall */
#define U2DCSR0_SST		(1 << 4)			/* Send Stall */
#define U2DCSR0_DME		(1 << 3)			/* DMA Enable */
#define U2DCSR0_FTF		(1 << 2)			/* Flush Transmit FIFO */
#define U2DCSR0_IPR		(1 << 1)			/* IN Packet Ready */
#define U2DCSR0_OPC		(1 << 0)			/* OUT Packet Complete */

#define U2DCSR(x)       __REG_3(0x54100100 + ((x) << 2))	/* U2D Control/Status Register - Endpoint x */
#define U2DCSR_BF		(1 << 10)			/* Buffer Full, for OUT eps */
#define U2DCSR_BE		(1 << 10)			/* Buffer Empty, for IN eps */
#define U2DCSR_DPE		(1 << 9)			/* Data Packet Error, for ISO eps only */
#define U2DCSR_FEF		(1 << 8)			/* Flush Endpoint FIFO */
#define U2DCSR_SP		(1 << 7)			/* Short Packet Control/Status, for OUT eps only, readonly */
#define U2DCSR_BNE		(1 << 6)			/* Buffer Not Empty, for OUT eps */
#define U2DCSR_BNF		(1 << 6)			/* Buffer Not Full, for IN eps */
#define U2DCSR_FST		(1 << 5)			/* Force STALL, write 1 set */
#define U2DCSR_SST		(1 << 4)			/* Sent STALL, write 1 clear */
#define U2DCSR_DME		(1 << 3)			/* DMA Enable */
#define U2DCSR_TRN		(1 << 2)			/* Tx/Rx NAK, write 1 clear */
#define U2DCSR_PC		(1 << 1)			/* Packet Complete, write 1 clear */
#define U2DCSR_FS		(1 << 0)			/* FIFO needs Service */

#define U2DBCR0         __REG_3(0x54100200)			/* U2D Byte Count Register - Endpoint 0 */
#define U2DBCR(x)       __REG_3(0x54100200 + ((x) << 2))	/* U2D Byte Count Register - Endpoint x */

#define U2DDR0		__REG_3(0x54100300)			/* U2D Data Register - Endpoint 0 */

#define U2DEPCR(x)	__REG_3(0x54100400 + ((x) << 2))	/* U2D Configuration Register - Endpoint x */
#define U2DEPCR_EE		(1 << 0)			/* Endpoint Enable */
#define U2DEPCR_BS_MASK	(0x3FE)					/* Buffer Size, BS*8=FIFO size, max 8184B = 8KB */

#define U2DSCA		__REG_3(0x54100500)			/* U2D Setup Command Address */
#define U2DSCA_VALUE	0x0120

#define U2DEN0		__REG_3(0x54100504)			/* U2D Endpoint Information Register - Endpoint 0 */
#define U2DEN(x)	__REG_3(0x54100504 + ((x) << 2))	/* U2D Endpoint Information Register - Endpoint x */

/* U2DMA registers */
#define U2DMACSR0		__REG_3(0x54101000)		/* U2DMA Control/Status Register - Channel 0 */
#define U2DMACSR(x)	__REG_3(0x54101000 + ((x) << 2))	/* U2DMA Control/Status Register - Channel x */
#define U2DMACSR_RUN		(1 << 31)			/* Run Bit (read / write) */
#define U2DMACSR_STOPIRQEN	(1 << 29)			/* Stop Interrupt Enable (read / write) */
#define U2DMACSR_EORIRQEN	(1 << 28)			/* End of Receive Interrupt Enable (R/W) */
#define U2DMACSR_EORJMPEN	(1 << 27)			/* Jump to next descriptor on EOR */
#define U2DMACSR_EORSTOPEN	(1 << 26)			/* STOP on an EOR */
#define U2DMACSR_RASIRQEN	(1 << 23)			/* Request After Cnannel Stopped Interrupt Enable */
#define U2DMACSR_MASKRUN	(1 << 22)			/* Mask Run */
#define U2DMACSR_SCEMC		(3 << 18)			/* System Bus Split Completion Error Message Class */
#define U2DMACSR_SCEMI		(0x1f << 13)			/* System Bus Split Completion Error Message Index */
#define U2DMACSR_BUSERRTYPE	(7 << 10)			/* PX Bus Error Type */
#define U2DMACSR_EORINTR	(1 << 9)			/* End Of Receive */
#define U2DMACSR_REQPEND	(1 << 8)			/* Request Pending */
#define U2DMACSR_RASINTR	(1 << 4)			/* Request After Channel Stopped (read / write 1 clear) */
#define U2DMACSR_STOPINTR	(1 << 3)			/* Stop Interrupt (read only) */
#define U2DMACSR_ENDINTR	(1 << 2)			/* End Interrupt (read / write 1 clear) */
#define U2DMACSR_STARTINTR	(1 << 1)			/* Start Interrupt (read / write 1 clear) */
#define U2DMACSR_BUSERRINTR	(1 << 0)			/* Bus Error Interrupt (read / write 1 clear) */

#define U2DMACR		__REG_3(0x54101080)			/* U2DMA Control Register */
#define U2DMAINT	__REG_3(0x541010F0)			/* U2DMA Interrupt Register */

#define U2DMABR0	__REG_3(0x54101100)			/* U2DMA Branch Register - Channel 0 */
#define U2DMABR(x)      __REG_3(0x54101100 + (x) << 2)		/* U2DMA Branch Register - Channel x */

#define U2DMADADR0      __REG_3(0x54101200)			/* U2DMA Descriptor Address Register - Channel 0 */
#define U2DMADADR(x)    __REG_3(0x54101200 + (x) * 0x10)	/* U2DMA Descriptor Address Register - Channel x */

#define U2DMADADR_STOP        (1U << 0)

#define U2DMASADR0      __REG_3(0x54101204)			/* U2DMA Source Address Register - Channel 0 */
#define U2DMASADR(x)    __REG_3(0x54101204 + (x) * 0x10)	/* U2DMA Source Address Register - Channel x */
#define U2DMATADR0      __REG_3(0x54101208)			/* U2DMA Target Address Register - Channel 0 */
#define U2DMATADR(x)    __REG_3(0x54101208 + (x) * 0x10)	/* U2DMA Target Address Register - Channel x */

#define U2DMACMDR0      __REG_3(0x5410120C)			/* U2DMA Command Address Register - Channel 0 */
#define U2DMACMDR(x)    __REG_3(0x5410120C + (x) * 0x10)	/* U2DMA Command Address Register - Channel x */

#define U2DMACMDR_XFRDIS		(1 << 31)		/* Transfer Direction */
#define U2DMACMDR_STARTIRQEN	(1 << 22)			/* Start Interrupt Enable */
#define U2DMACMDR_ENDIRQEN		(1 << 21)		/* End Interrupt Enable */
#define U2DMACMDR_PACKCOMP		(1 << 13)		/* Packet Complete */
#define U2DMACMDR_LEN			(0x07ff)		/* length mask (max = 2K - 1) */

#define CWSBR_BASE	0x42404000
#define CWSBR		0x8

#define cwsbr_readl()	__raw_readl(dev->cwsbr_base + CWSBR)
#define cwsbr_writel(v)	__raw_writel((v), dev->cwsbr_base + CWSBR)

#define set_cwsbr()	cwsbr_writel((cwsbr_readl()) | 0x10000)
#define clr_cwsbr()	cwsbr_writel((cwsbr_readl()) & ~(0x10000))

enum u2d_bug_type {
	U2D_BUG_NONE = 0,
	U2D_BUG_INMASS = 0x1,
	U2D_BUG_SETINTF = 0x2,
	U2D_BUG_STALL = 0x4,
	DDR_BUG_DMA = 0x8,
	U2D_BUG_UTMID = 0x10,
	U2D_FIX_ULPI_STP = 0x20,
};

enum u2d_phy_mode {
	SYNCH = 0,
	CARKIT = 0x1,
	SER_3PIN = 0x2,
	SER_6PIN = 0x4,
	LOWPOWER = 0x8,
	/* rtsm mode */
	PRE_SYNCH = 0x10,
};

enum hs_cmds_permf {
	HS_CMDS_0,
	HS_CMDS_1,
	HS_CMDS_2,
	HS_CMDS_3
};

#define ISO_HS_CMDS		HS_CMDS_0
#define INT_HS_CMDS		HS_CMDS_0

struct dma_txfr_t {
	unsigned int len;	/* Buffer Size */
	int is_zero;		/* Add zero package after transfer. Only valid for IN transfer. */
	int end_irq_en;		/* Enalbe/Disable End Interrupt at last descriptor */
};
#endif

struct pxa27x_udc;

struct pxa27x_ep {
	struct usb_ep				ep;
	struct pxa27x_udc			*dev;

	const struct usb_endpoint_descriptor	*desc;
	struct list_head			queue;
	unsigned long				pio_irqs;
	unsigned long				dma_irqs;

	int					dma;
	unsigned				fifo_size;
	unsigned				ep_num;
	unsigned				ep_type;

	unsigned				stopped : 1;
	unsigned				dma_con : 1;
	unsigned				dir_in : 1;
	unsigned				assigned : 1;

#ifdef CONFIG_USB_PXA3XX_U2D
	void *dma_desc_virt;
	dma_addr_t dma_desc_phys;
	unsigned dma_desc_size;

	/* HS only for ISO and interrupt endpoints */
	unsigned hs_cmds;
#endif
	void					*dma_buf_virt;
	dma_addr_t				dma_buf_phys;
	unsigned				dma_buf_size;

	unsigned				aisn;
#ifdef CONFIG_USB_PXA27X_UDC
	/* UDCCSR = UDC Control/Status Register for this EP
	 * UBCR = UDC Byte Count Remaining (contents of OUT fifo)
	 * UDCDR = UDC Endpoint Data Register (the fifo)
	 * UDCCR = UDC Endpoint Configuration Registers
	 * DRCM = DMA Request Channel Map
	 */
	volatile u32				*reg_udccsr;
	volatile u32				*reg_udcbcr;
	volatile u32				*reg_udcdr;
	volatile u32				*reg_udccr;
	volatile u32				*reg_drcmr;
#endif

#ifdef CONFIG_PM
#if defined(CONFIG_USB_PXA27X_UDC)
	unsigned				udccsr_value;
	unsigned				udccr_value;
#elif defined(CONFIG_USB_PXA3XX_U2D)
	unsigned u2dcsr_value;
	unsigned u2dcr_value;
	unsigned u2denr_value;
#endif
};

#if defined(CONFIG_USB_PXA27X_UDC)
#define EP0_FIFO_SIZE	((unsigned)16)
#define BULK_FIFO_SIZE	((unsigned)64)
#ifdef PATCH_TEMP
#define ISO_FIFO_SIZE	((unsigned)512)
#else
#define ISO_FIFO_SIZE	((unsigned)256)
#endif
#define INT_FIFO_SIZE	((unsigned)16)
#elif defined(CONFIG_USB_PXA3XX_U2D)
/* 8KB EP FIFO */
#define U2D_EPMEM_SIZE	((unsigned)8192)

#define EP0_MPS			((unsigned)64)
#define BULK_MPS(speed)	((speed) == USB_SPEED_HIGH) ? ((unsigned)512) : ((unsigned)64)
#define ISO_MPS(speed)	((speed) == USB_SPEED_HIGH) ? ((unsigned)1024) : ((unsigned)1023)
#define INT_MPS(speed)	((speed) == USB_SPEED_HIGH) ? ((unsigned)1024) : ((unsigned)16)

#define EP0_FIFO_SIZE	((unsigned)64)
#define BULK_FIFO_SIZE	(((unsigned)512 + 8) * 2)
#define ISO_FIFO_SIZE	((unsigned)1024 + 8)
#define INT_FIFO_SIZE	((unsigned)100 + 8)
#endif

#define DMA_BUF_SIZE	PAGE_SIZE
#define DMA_DESC_SIZE	PAGE_SIZE
#define DMA_DESC_NUM	(DMA_DESC_SIZE / 16)

struct udc_stats {
	struct ep0stats {
		unsigned long		ops;
		unsigned long		bytes;
	} read, write;
	unsigned long			irqs;
};

#ifndef	UDC_EP_NUM
#define	UDC_EP_NUM	24
#endif

#define	PXA_U2D_EP_NUM	16

#define HALF_EP_NUM	(PXA_U2D_EP_NUM / 2)

#define UDC_LG_EP_NUM	16


struct pxa27x_udc {
	struct usb_gadget			gadget;
	struct usb_gadget_driver		*driver;
	struct pxa3xx_comp			cdev;
	struct list_head unused_ep_list;		/* all the eps the driver can use*/
	struct list_head used_ep_list;			/* all the eps the driver has used*/
	struct otg_transceiver			*transceiver;
	struct udc_stats			stats;
	unsigned				got_irq : 1,
						got_disc : 1,
						has_cfr : 1,
						req_pending : 1,
						req_std : 1;

#define start_watchdog(dev) mod_timer(&dev->timer, jiffies + (HZ/200))
	struct timer_list			timer;

	struct device				*dev;
	struct clk				*clk;
	u64					dma_mask;
	struct pxa27x_ep			ep [UDC_LG_EP_NUM];

#if defined(CONFIG_USB_PXA27X_UDC)
	struct pxa2xx_udc_mach_info		*mach;
#ifdef CONFIG_PM
	unsigned				udccsr0;
#endif
#elif defined(CONFIG_USB_PXA3XX_U2D)
	struct pxa3xx_u2d_mach_info		*mach;
#ifdef CONFIG_PM
	unsigned u2dcr;
	unsigned u2dicr;
	unsigned u2dcsr0;
#ifdef CONFIG_CPU_PXA310
	unsigned u2dotgcr;
	unsigned u2dotgicr;
#endif
#endif
#endif

	unsigned stp_gpio_irq_en : 1,
u2d_clk_dis: 1,
u2d_irq_dis: 1,
ulpi_dat3_work: 1;

	int u2d_dp;
	int u2d_ts;
	int ulpi_int;

	struct otg_xceiv_ops		*xv_ops;

	void __iomem *cwsbr_base;
#endif
	struct timer_list dvfm_timer;
};

/*-------------------------------------------------------------------------*/
#if 0
#ifdef DEBUG
#define HEX_DISPLAY(n)	do { \
	if (machine_is_mainstone()) {\
		MST_LEDDAT1 = (n); } \
	} while (0)

#define HEX_DISPLAY1(n)	HEX_DISPLAY(n)

#define HEX_DISPLAY2(n)	do { \
	if (machine_is_mainstone()) {\
		MST_LEDDAT2 = (n); } \
	} while (0)

#endif /* DEBUG */
#endif
/*-------------------------------------------------------------------------*/

/* LEDs are only for debug */
#ifndef HEX_DISPLAY
#define HEX_DISPLAY(n)		do {} while (0)
#endif

#ifndef LED_CONNECTED_ON
#define LED_CONNECTED_ON	do {} while (0)
#define LED_CONNECTED_OFF	do {} while (0)
#endif
#ifndef LED_EP0_ON
#define LED_EP0_ON		do {} while (0)
#define LED_EP0_OFF		do {} while (0)
#endif

/*-------------------------------------------------------------------------*/

/*
 * Debugging support vanishes in non-debug builds.  DBG_NORMAL should be
 * mostly silent during normal use/testing, with no timing side-effects.
 */
#define DBG_NORMAL	1	/* error paths, device state transitions */
#define DBG_VERBOSE	2	/* add some success path trace info */
#define DBG_NOISY	3	/* ... even more: request level */
#define DBG_VERY_NOISY	4	/* ... even more: packet level */

#ifdef DEBUG

static const char *state_name[] = {
	"EP0_IDLE",
	"EP0_IN_DATA_PHASE", "EP0_OUT_DATA_PHASE",
	"EP0_END_XFER", "EP0_STALL"
};

#define DMSG(stuff...) printk(KERN_DEBUG stuff)

#ifdef VERBOSE
#    define UDC_DEBUG DBG_VERY_NOISY
#else
#    define UDC_DEBUG DBG_NORMAL
#endif

#if defined(CONFIG_USB_PXA27X_UDC)
static void __attribute__ ((__unused__))
dump_udccr(const char *label)
{
	u32	udccr = UDCCR;
	DMSG("%s 0x%08x =%s%s%s%s%s%s%s%s%s%s, con=%d,inter=%d,altinter=%d\n",
		label, udccr,
		(udccr & UDCCR_OEN) ? " oen":"",
		(udccr & UDCCR_AALTHNP) ? " aalthnp":"",
		(udccr & UDCCR_AHNP) ? " rem" : "",
		(udccr & UDCCR_BHNP) ? " rstir" : "",
		(udccr & UDCCR_DWRE) ? " dwre" : "",
		(udccr & UDCCR_SMAC) ? " smac" : "",
		(udccr & UDCCR_EMCE) ? " emce" : "",
		(udccr & UDCCR_UDR) ? " udr" : "",
		(udccr & UDCCR_UDA) ? " uda" : "",
		(udccr & UDCCR_UDE) ? " ude" : "",
		(udccr & UDCCR_ACN) >> UDCCR_ACN_S,
		(udccr & UDCCR_AIN) >> UDCCR_AIN_S,
		(udccr & UDCCR_AAISN) >> UDCCR_AAISN_S);
}

static void __attribute__ ((__unused__))
dump_udccsr0(const char *label)
{
	u32		udccsr0 = UDCCSR0;

	DMSG("%s 0x%08x =%s%s%s%s%s%s%s\n",
		label, udccsr0,
		(udccsr0 & UDCCSR0_SA) ? " sa" : "",
		(udccsr0 & UDCCSR0_RNE) ? " rne" : "",
		(udccsr0 & UDCCSR0_FST) ? " fst" : "",
		(udccsr0 & UDCCSR0_SST) ? " sst" : "",
		(udccsr0 & UDCCSR0_DME) ? " dme" : "",
		(udccsr0 & UDCCSR0_IPR) ? " ipr" : "",
		(udccsr0 & UDCCSR0_OPC) ? " opr" : "");
}

static void __attribute__ ((__unused__))
dump_state(struct pxa27x_udc *dev)
{
	unsigned	i;

	DMSG("%s udcicr %02X.%02X, udcsir %02X.%02x, udcfnr %02X\n",
		state_name[dev->cdev.ep0state],
		UDCICR1, UDCICR0, UDCISR1, UDCISR0, UDCFNR);
	dump_udccr("udccr");

	if (!dev->driver) {
		DMSG("no gadget driver bound\n");
		return;
	} else
		DMSG("ep0 driver '%s'\n", dev->driver->driver.name);


	dump_udccsr0("udccsr0");
	DMSG("ep0 IN %lu/%lu, OUT %lu/%lu\n",
		dev->stats.write.bytes, dev->stats.write.ops,
		dev->stats.read.bytes, dev->stats.read.ops);

	for (i = 1; i < UDC_EP_NUM; i++) {
		if (dev->ep[i].desc == 0)
			continue;
		DMSG("udccs%d = %02x\n", i, *dev->ep->reg_udccsr);
	}
}

#if 0
static void dump_regs(u8 ep)
{
	DMSG("EP:%d UDCCSR:0x%08x UDCBCR:0x%08x\n UDCCR:0x%08x\n",
		ep, UDCCSN(ep), UDCBCN(ep), UDCCN(ep));
}
static void dump_req(struct pxa27x_request *req)
{
	struct usb_request *r = &req->req;

	DMSG("%s: buf:0x%08x length:%d dma:0x%08x actual:%d\n",
			__FUNCTION__, (unsigned)r->buf, r->length,
			r->dma,	r->actual);
}
#endif
#elif defined(CONFIG_USB_PXA3XX_U2D)
static void __attribute__((__unused__))
dump_u2dcr(const char *label) {
}

static void __attribute__((__unused__))
dump_u2dcsr0(const char *label) {
}

#if 0
static void __attribute__((__unused__))
dump_state(struct pxa3xx_u2d *dev) {
	unsigned i;
	u32 ep_num = HALF_EP_NUM;

	if (cpu_is_pxa310())
		ep_num = PXA_U2D_EP_NUM;

	DMSG("%s u2dicr %02X, u2disr %02X, u2dfnr %02X\n",
	     state_name[dev->cdev.ep0state],
	     U2DICR, U2DISR, U2DFNR);
	dump_u2dcr("u2dcr");

	if (!dev->driver) {
		DMSG("no gadget driver bound\n");
		return;
	} else
		DMSG("ep0 driver '%s'\n", dev->driver->driver.name);


	dump_u2dcsr0("u2dcsr0");
	DMSG("ep0 IN %lu/%lu, OUT %lu/%lu\n",
	     dev->stats.write.bytes, dev->stats.write.ops,
	     dev->stats.read.bytes, dev->stats.read.ops);

	for (i = 1; i < ep_num; i++) {
		if (dev->ep [i].desc == 0)
			continue;
		DMSG("u2dcs%d = %02x\n", i, U2DCSR(i));
	}
}
#endif
#endif

#else

#define DMSG(stuff...)		do {} while (0)

#define	dump_udccr(x)	do {} while (0)
#define	dump_udccsr0(x)	do {} while (0)
#define	dump_state(x)	do {} while (0)

#define UDC_DEBUG ((unsigned)0)

#endif

#define DBG(lvl, stuff...) do { \
				if ((lvl) <= UDC_DEBUG) \
					DMSG(stuff); } while (0)

#if defined(CONFIG_USB_PXA3XX_U2D)
#define PXA320_U2D_TERMSEL	MFP_PIN_GPIO100
#define PXA300_U2D_DETECT	EXT0_GPIO(6)
#define PXA320_U2D_DETECT	MFP_PIN_GPIO46
#define PXA310_ULPI_INT	MFP_PIN_GPIO33
#endif

/*  for usb composite driver */
extern int set_eps(__u8 num_eps, struct usb_endpoint_descriptor *p_ep_desc, int len, int config, int interface, int alt);
/* the above function must call the func -- comp_set_ep*/
extern void udc_reinit(void);
extern struct usb_gadget_driver *stop_udc(struct usb_gadget_driver *driver);
extern int get_extra_descriptor(char *buffer, unsigned size, unsigned char type, void **ptr);
#endif /* __LINUX_USB_GADGET_PXA3XX_H */

