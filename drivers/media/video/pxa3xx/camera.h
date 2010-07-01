/*
    Copyright (C) 2005, Intel Corporation.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.


 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
*/

/*****************************************************************************
 * Abstract:
 *   contains all camera specific macros, typedefs, and prototypes.
 *   Declares no storage.
 *
 * Notes:Only valid for processor code named Monahans.
 *****************************************************************************/

#ifndef __CAMERA_HEADER__
#define __CAMERA_HEADER__

#include <linux/list.h>
#include <linux/workqueue.h>

#include <mach/camera.h>
#include <linux/pxa_camera.h>
#include <linux/wait.h>



#ifdef CONFIG_PXA3xx

#include <mach/pxa3xx-regs.h>

/* Camera register offset */

#define CICR0		(0x0000)	/* Control register 0 */
#define CICR1		(0x0004)	/* Control register 1 */
#define CICR2		(0x0008)	/* Control register 2 */
#define CICR3		(0x000C)	/* Control register 3 */
#define CICR4		(0x0010)	/* Control register 4 */
#define CISR		(0x0014)	/* Status register */
#define CITOR		(0x001C)	/* Time-Out register */
#define CIBR0		(0x0028)	/* Channel 0 Receive Buffer */
#define CIBR1		(0x0030)	/* Channel 1 Receive Buffer */
#define CIBR2		(0x0038)	/* Channel 2 Receive Buffer */
#define CIBR3		(0x0040)	/* Channel 3 Receive Buffer */
#define	CIRCD		(0x0044)

#define CIJPEG		(0x0048)	/* JPEG control and status register */

#define CIPSS		(0x0064)	/* Pixel Substitution Status register */
#define CIPBUF		(0x0068)	/* Pixel Substitution Buffer */
#define CIHST		(0x006C)	/* Histogram Configuration */
#define CISUM		(0x0070)	/* Histogram Summation register */
#define CICCR		(0x0074)	/* Compander Configuration */
#define CISSC		(0x007C)	/* Spatial Scaling Configuration */

#define CICMR		(0x0090)	/* Color Management register */
#define CICMC0		(0x0094)	/* Color Management Coefficients 0 */
#define CICMC1		(0x0098)	/* Color Management Coefficients 1 */
#define CICMC2		(0x009C)	/* Color Management Coefficients 2 */

#define CIFR0		(0x00B0)	/* FIFO Control register 0 */
#define CIFR1		(0x00B4)	/* FIFO Control register 1 */
#define CIFSR		(0x00C0)	/* FIFO Status register */

#define CIDADR0		(0x0240)	/* DMA Descriptor Address Channel 0 register */
#define CIDADR1		(0x0250)	/* DMA Descriptor Address Channel 1 register */
#define CIDADR2		(0x0260)	/* DMA Descriptor Address Channel 2 register */
#define CIDADR3		(0x0270)	/* DMA Descriptor Address Channel 3 register */
#define CISADR0		(0x0244) 	/* DMA Target Address Channel 0 register */
#define CISADR1		(0x0254) 	/* DMA Target Address Channel 1 register */
#define CISADR2		(0x0264) 	/* DMA Target Address Channel 2 register */
#define CISADR3		(0x0274) 	/* DMA Target Address Channel 3 register */
#define CITADR0		(0x0248) 	/* DMA Source Address Channel 0 register */
#define CITADR1		(0x0258) 	/* DMA Source Address Channel 1 register */
#define CITADR2		(0x0268) 	/* DMA Source Address Channel 2 register */
#define CITADR3		(0x0278) 	/* DMA Source Address Channel 3 register */
 
#define CICMD0		(0x024C)	/* DMA Command Channel 0 register */
#define CICMD1		(0x025C)	/* DMA Command Channel 1 register */
#define CICMD2		(0x026C)	/* DMA Command Channel 2 register */
#define CICMD3		(0x027C)	/* DMA Command Channel 3 register */
#define CIDBR0		(0x0220)	/* DMA Branch Channel 0 register */
#define CIDBR1		(0x0224)	/* DMA Branch Channel 1 register */
#define CIDBR2		(0x0228)	/* DMA Branch Channel 2 register */
#define CIDBR3		(0x022C)	/* DMA Branch Channel 3 register */
#define CIDCSR0		(0x0200)	/* DMA Ctrl/Status Channel 0 register */
#define CIDCSR1		(0x0204)	/* DMA Ctrl/Status Channel 1 register */
#define CIDCSR2		(0x0208)	/* DMA Ctrl/Status Channel 2 register */
#define CIDCSR3		(0x020C)	/* DMA Ctrl/Status Channel 3 register */

#define CICR0_DMAEN	(1 << 31)	/* DMA request enable */
#define CICR0_PAR_EN	(1 << 30)	/* Parity enable */
#define CICR0_SL_CAP_EN	(1 << 29)	/* Capture enable for slave mode */
#define CICR0_ENB	(1 << 28)	/* Camera interface enable */
#define CICR0_DIS	(1 << 27)	/* Camera interface disable */
#define CICR0_SIM	(0x7 << 24)	/* Sensor interface mode mask */
#define	CICR0_EOFXM	( 1 << 12 )
#define	CICR0_BSM	( 1 << 11 )
#define	CICR0_FUM	( 1 << 10 )
#define CICR0_TOM	(1 << 9)	/* Time-out mask */
#define CICR0_RDAVM	(1 << 8)	/* Receive-data-available mask */
#define CICR0_FEM	(1 << 7)	/* FIFO-empty mask */
#define CICR0_EOLM	(1 << 6)	/* End-of-line mask */
#define CICR0_PERRM	(1 << 5)	/* Parity-error mask */
#define CICR0_QDM	(1 << 4)	/* Quick-disable mask */
#define CICR0_CDM	(1 << 3)	/* Disable-done mask */
#define CICR0_SOFM	(1 << 2)	/* Start-of-frame mask */
#define CICR0_EOFM	(1 << 1)	/* End-of-frame mask */
#define CICR0_FOM	(1 << 0)	/* FIFO-overrun mask */

#define CICR1_TBIT	(1 << 31)	/* Transparency bit */
#define CICR1_RGBT_CONV	(0x3 << 30)	/* RGBT conversion mask */
#define CICR1_PPL	(0x3f << 15)	/* Pixels per line mask */
#define CICR1_RGB_CONV	(0x7 << 12)	/* RGB conversion mask */
#define CICR1_RGB_F	(1 << 11)	/* RGB format */
#define CICR1_YCBCR_F	(1 << 10)	/* YCbCr format */
#define CICR1_RGB_BPP	(0x7 << 7)	/* RGB bis per pixel mask */
#define CICR1_RAW_BPP	(0x3 << 5)	/* Raw bis per pixel mask */
#define CICR1_COLOR_SP	(0x3 << 3)	/* Color space mask */
#define CICR1_DW	(0x7 << 0)	/* Data width mask */

#define CICR2_BLW	(0xff << 24)	/* Beginning-of-line pixel clock wait count mask */
#define CICR2_ELW	(0xff << 16)	/* End-of-line pixel clock wait count mask */
#define CICR2_HSW	(0x3f << 10)	/* Horizontal sync pulse width mask */
#define CICR2_BFPW	(0x3f << 3)	/* Beginning-of-frame pixel clock wait count mask */
#define CICR2_FSW	(0x7 << 0)	/* Frame stabilization wait count mask */

#define CICR3_BFW	(0xff << 24)	/* Beginning-of-frame line clock wait count mask */
#define CICR3_EFW	(0xff << 16)	/* End-of-frame line clock wait count mask */
#define CICR3_VSW	(0x3f << 10)	/* Vertical sync pulse width mask */
#define CICR3_BFPW	(0x3f << 3)	/* Beginning-of-frame pixel clock wait count mask */
#define CICR3_LPF	(0x3ff << 0)	/* Lines per frame mask */

#define CICR4_MCLK_DLY	(0x3 << 24)	/* MCLK Data Capture Delay mask */
#define CICR4_PCLK_EN	(1 << 23)	/* Pixel clock enable */
#define CICR4_PCP	(1 << 22)	/* Pixel clock polarity */
#define CICR4_HSP	(1 << 21)	/* Horizontal sync polarity */
#define CICR4_VSP	(1 << 20)	/* Vertical sync polarity */
#define CICR4_MCLK_EN	(1 << 19)	/* MCLK enable */
#define CICR4_FR_RATE	(0x7 << 8)	/* Frame rate mask */
#define CICR4_DIV	(0xff << 0)	/* Clock divisor mask */

#define CISR_FTO	(1 << 15)	/* FIFO time-out */
#define CISR_RDAV_2	(1 << 14)	/* Channel 2 receive data available */
#define CISR_RDAV_1	(1 << 13)	/* Channel 1 receive data available */
#define CISR_RDAV_0	(1 << 12)	/* Channel 0 receive data available */
#define CISR_FEMPTY_2	(1 << 11)	/* Channel 2 FIFO empty */
#define CISR_FEMPTY_1	(1 << 10)	/* Channel 1 FIFO empty */
#define CISR_FEMPTY_0	(1 << 9)	/* Channel 0 FIFO empty */
#define CISR_EOL	(1 << 8)	/* End of line */
#define CISR_PAR_ERR	(1 << 7)	/* Parity error */
#define CISR_CQD	(1 << 6)	/* Camera interface quick disable */
#define CISR_SOF	(1 << 5)	/* Start of frame */
#define CISR_CDD	(1 << 4)	/* Camera interface disable done */
#define CISR_EOF	(1 << 3)	/* End of frame */
#define CISR_IFO_2	(1 << 2)	/* FIFO overrun for Channel 2 */
#define CISR_IFO_1	(1 << 1)	/* FIFO overrun for Channel 1 */
#define CISR_IFO_0	(1 << 0)	/* FIFO overrun for Channel 0 */

#define CIFR_FLVL2	(0x7f << 23)	/* FIFO 2 level mask */
#define CIFR_FLVL1	(0x7f << 16)	/* FIFO 1 level mask */
#define CIFR_FLVL0	(0xff << 8)	/* FIFO 0 level mask */
#define CIFR_THL_0	(0x3 << 4)	/* Threshold Level for Channel 0 FIFO */
#define CIFR_RESET_F	(1 << 3)	/* Reset input FIFOs */
#define CIFR_FEN2	(1 << 2)	/* FIFO enable for channel 2 */
#define CIFR_FEN1	(1 << 1)	/* FIFO enable for channel 1 */
#define CIFR_FEN0	(1 << 0)	/* FIFO enable for channel 0 */

#endif				/* CONFIG_PXA3xx */

/*****************************************************************************
 *	Register definitions
 *****************************************************************************/

#define	CI_CICR0_SIM_SHIFT		24
#define	CI_CICR0_SIM_SMASK		0x7
#define	CI_CICR0_INTERRUPT_MASK		(CICR0_FOM | CICR0_EOFM | \
		CICR0_SOFM | CICR0_CDM | \
		CICR0_QDM | CICR0_PERRM | \
		CICR0_EOLM | CICR0_FEM | \
		CICR0_TOM | CICR0_FUM | \
		CICR0_BSM | CICR0_EOFXM )

#define CI_CICR1_DW_SHIFT		0
#define CI_CICR1_DW_SMASK		0x7
#define CI_CICR1_COLOR_SP_SHIFT		3
#define CI_CICR1_COLOR_SP_SMASK		0x3
#define CI_CICR1_RAW_BPP_SHIFT		5
#define CI_CICR1_RAW_BPP_SMASK		0x3
#define CI_CICR1_RGB_BPP_SHIFT		7
#define CI_CICR1_RGB_BPP_SMASK		0x7
#define CI_CICR1_YCBCR_F		( 1UL << 10 )
#define CI_CICR1_RBG_F			( 1UL << 11 )
#define CI_CICR1_RGB_CONV_SHIFT		12
#define CI_CICR1_RGB_CONV_SMASK		0x7
#define CI_CICR1_PPL_SHIFT		15
#define CI_CICR1_PPL_SMASK		0xFFF
#define CI_CICR1_RESERVED		0x1C000000
#define CI_CICR1_RGBT_CONV_SHIFT	9
#define CI_CICR1_RGBT_CONV_SMASK	0x3
#define CI_CICR1_TBIT			( 1UL << 31 )


#define CI_CICR2_FSW_SHIFT	0
#define CI_CICR2_FSW_SMASK	0x3
#define CI_CICR2_BFPW_SHIFT	3
#define CI_CICR2_BFPW_SMASK	0x3F
#define CI_CICR2_RESERVED	0x00000200
#define CI_CICR2_HSW_SHIFT	10
#define CI_CICR2_HSW_SMASK	0x3F
#define CI_CICR2_ELW_SHIFT	16
#define CI_CICR2_ELW_SMASK	0xFF
#define CI_CICR2_BLW_SHIFT	24
#define CI_CICR2_BLW_SMASK	0xFF

#define CI_CICR3_LPF_SHIFT	0
#define CI_CICR3_LPF_SMASK	0x7FF
#define CI_CICR3_VSW_SHIFT	11
#define CI_CICR3_VSW_SMASK	0x1F
#define CI_CICR3_EFW_SHIFT	16
#define CI_CICR3_EFW_SMASK	0xFF
#define CI_CICR3_BFW_SHIFT	24
#define CI_CICR3_BFW_SMASK	0xFF

#define CI_CICR4_DIV_SHIFT	0
#define CI_CICR4_DIV_SMASK	0xFF
#define CI_CICR4_FR_RATE_SHIFT	8
#define CI_CICR4_FR_RATE_SMASK	0x7
#define CI_CICR4_MCLK_EN	( 1UL << 19 )
#define CI_CICR4_VSP		( 1UL << 20 )
#define CI_CICR4_HSP		( 1UL << 21 )
#define CI_CICR4_PCP		( 1UL << 22 )
#define CI_CICR4_PCLK_EN	( 1UL << 23 )

#define CI_CICR4_YCBCR_DS	(1UL << 27)

#define CI_CISR_EOF          ( 1 << 3 )
#define CI_CISR_SOF          ( 1 << 4 )
#define CI_CISR_CDD          ( 1 << 5 )
#define CI_CISR_CQD          ( 1 << 6 )
#define CI_CISR_PAR_ERR      ( 1 << 7 )
#define CI_CISR_EOL          ( 1 << 8 )
#define CI_CISR_HST_INT      ( 1 << 9 )
#define CI_CISR_CGU_INT      ( 1 << 10 )
#define CI_CISR_FTO          ( 1 << 15 )
#define CI_CISR_EOFX         ( 1 << 30 )
#define CI_CISR_SINT         ( 1 << 31 )
#define	CI_CISR_MASK	( CI_CISR_EOF | CI_CISR_SOF | CI_CISR_CDD | \
		CI_CISR_CQD | CI_CISR_PAR_ERR | CI_CISR_EOL | \
		CI_CISR_CGU_INT|CI_CISR_FTO | \
		CI_CISR_EOFX | CI_CISR_SINT)


#define	CI_CIFSR_EOF3	(1 << 10)


#define	CIPSS_PSU_EN	(1 << 31)

#define	CI_CIPBUF_DEADROW_SHIFT	0
#define	CI_CIPBUF_DEADROW_SMASK	0xFFF
#define	CI_CIPBUF_DEADCOL_SHIFT	16
#define	CI_CIPBUF_DEADCOL_SMASK	0xFFF

#define CI_CIHST_COLOR_SEL_SHIFT	0
#define CI_CIHST_COLOR_SEL_SMASK	0xF
#define CI_CIHST_COLOR_RED		0x1
#define CI_CIHST_COLOR_BLUE		0x2
#define CI_CIHST_COLOR_GREEN1		0x4
#define CI_CIHST_COLOR_GREEN2		0x8
#define CI_CIHST_SCALE_SHIFT		4
#define CI_CIHST_SCALE_SMASK		0x3
#define CI_CIHST_SCALE_0_TO_7		0x0
#define CI_CIHST_SCALE_0_TO_8		0x1
#define CI_CIHST_SCALE_1_TO_9		0x2
#define CI_CIHST_CLR_RAM		(1 << 6)

#define CI_CICCR_EN		(1 << 0 )
#define CI_CICCR_SCALE_SHIFT	1
#define CI_CICCR_SCALE_SMASK	0x3
#define CI_CICCR_SCALE_0_TO_7	0x0
#define CI_CICCR_SCALE_1_TO_8	0x1
#define CI_CICCR_SCALE_2_TO_9	0x2
#define CI_CICCR_CLUT_SHIFT	3
#define CI_CICCR_CLUT_SMASK	0xFF
#define CI_CICCR_CLUT_RED	0x00
#define CI_CICCR_CLUT_BLUE	0x20
#define CI_CICCR_CLUT_GREEN	0x40
#define CI_CICCR_BLC_SHIFT	11
#define CI_CICCR_BLC_SMASK	0xFF
#define CI_CICCR_LUT_LD		(1 << 19)

#define CI_CISSC_SCALE_SHIFT	0
#define CI_CISSC_SCALE_SMASK	0x3
#define CI_CISSC_SCALE_DISABLE	0x0
#define CI_CISSC_SCALE_2_TO_1	0x1
#define CI_CISSC_SCALE_4_TO_1	0x2

#define CI_CICMR_DMODE_SHIFT	0
#define CI_CICMR_DMODE_SMASK	0x3
#define CI_CICMR_DMODE_DISABLE	0x0
#define CI_CICMR_DMODE_RGB	0x1
#define CI_CICMR_DMODE_YUV	0x2

#define CI_CICMC0_COF02_SHIFT	0
#define CI_CICMC0_COF02_SMASK	0x3FF

#define CI_CICMC0_COF01_SHIFT	10
#define CI_CICMC0_COF01_SMASK	0x3FF

#define CI_CICMC0_COF00_SHIFT	20
#define CI_CICMC0_COF00_SMASK	0x3FF

#define CI_CICMC1_COF12_SHIFT	0
#define CI_CICMC1_COF12_SMASK	0x3FF

#define CI_CICMC1_COF11_SHIFT	10
#define CI_CICMC1_COF11_SMASK	0x3FF

#define CI_CICMC1_COF10_SHIFT	20
#define CI_CICMC1_COF10_SMASK	0x3FF

#define CI_CICMC2_COF22_SHIFT	0
#define CI_CICMC2_COF22_SMASK	0x3FF

#define CI_CICMC2_COF21_SHIFT	10
#define CI_CICMC2_COF21_SMASK	0x3FF

#define CI_CICMC2_COF20_SHIFT	20
#define CI_CICMC2_COF20_SMASK	0x3FF


#define CI_CIFR_THL_0_SHIFT	4
#define CI_CIFR_THL_0_SMASK	0x3

#define CI_CIFR_RESERVED1	0x000000C0

#define CI_CIFR_FLVL0_SHIFT	8
#define CI_CIFR_FLVL0_SMASK	0xFF

#define CI_CIFR_FLVL1_SHIFT	16
#define CI_CIFR_FLVL1_SMASK	0x7F

#define CI_CIFR_FLVL2_SHIFT	23
#define CI_CIFR_FLVL2_SMASK	0x7F

#define CI_CIFR_RESERVED2	0xC0000000
#define CI_CIFR_RESERVED	CI_CIFR_RESERVED1 | CI_CIFR_RESERVED2

#define	CI_CIFR_FEN3		( 1 << 0 )
#define	CI_CIFR_FLVL3_SHIFT	1
#define	CI_CIFR_FLVL3_SMASK	0xFF

#define	CI_CIDBR_BRA		( 1 << 0 )
#define	CI_CIDBR_BINT		( 1 << 1 )
#define	CI_CIDBR_SRCADDR_SMASK	0xFFFFFFF0

/*****************************************************************************
 *	Parameter Type definitions
 *****************************************************************************/

/* Interrupt mask */
#define CI_INT_IFO	( 1 << 0 )   /* FIFO Overrun Mask */
#define CI_INT_EOF	( 1 << 1 )   /* QCI End-of-Frame Mask */
#define CI_INT_SOF	( 1 << 2 )   /* QCI Start-of-Frame Mask */
#define CI_INT_CDD	( 1 << 3 )   /* QCI Disable Done Mask */
#define CI_INT_CQD	( 1 << 4 )   /* QCI Quick Disable Mask */
#define CI_INT_PAR_ERR	( 1 << 5 )   /* Parity Error Mask */
#define CI_INT_EOL	( 1 << 6 )   /* End-of-Line Mask */
#define CI_INT_FEMPTY	( 1 << 7 )   /* FIFO Empty Mask */
#define CI_INT_FTO	( 1 << 9 )   /* Time-Out Mask */
#define CI_INT_FU	( 1 << 10 )  /* Input FIFO Underrun Mask Channel 3 */
#define CI_INT_BS	( 1 << 11 )  /* Branch Status Mask */
#define CI_INT_EOFX	( 1 << 12 )  /* End-of-Frame Transfer to Memory Mask */
#define CI_INT_SC0	( 1 << 13 )  /* Stop Channel Interrupt Mask Channel 0*/
#define CI_INT_SC1	( 1 << 14 )  /* Stop Channel Interrupt Mask Channel 1*/
#define CI_INT_SC2	( 1 << 15 )  /* Stop Channel Interrupt Mask Channel 2*/
#define CI_INT_SC3	( 1 << 16 )  /* Stop Channel Interrupt Mask Channel 3*/

#define CICR0_VAL(ci_int_mask) \
	((ci_int_mask) & CI_CICR0_INTERRUPT_MASK )

/* convert CI_INT_SC0 to CIDCSR0[StopIrqEn] */
#define CIDCSR0_VAL(ci_int_mask) \
	(((!((ci_int_mask) & (1UL << 13))) >> 13) << 29)

/* convert CI_INT_SC1 to CIDCSR1[StopIrqEn] */
#define CIDCSR1_VAL(ci_int_mask) \
	(((!((ci_int_mask) & (1UL << 14))) >> 14) << 29)

/* convert CI_INT_SC2 to CIDCSR2[StopIrqEn] */
#define CIDCSR2_VAL(ci_int_mask) \
	(((!((ci_int_mask) & (1UL << 15))) >> 15) << 29)

/* convert CI_INT_SC3 to CIDCSR3[StopIrqEn] */
#define CIDCSR3_VAL(ci_int_mask) \
	(((!((ci_int_mask) & (1UL << 16))) >> 16) << 29)

#define CI_INT_MASK(cicr0_val, cidcsr0_val, cidcsr1_val,\
		cidcsr2_val, cidcsr3_val) \
	(((cicr0_val) & CI_CICR0_INTERRUPT_MASK) | \
	 ((!((cidcsr0_val) & (1UL << 29))) << 13) | \
	 ((!((cidcsr1_val) & (1UL << 29))) << 14) | \
	 ((!((cidcsr2_val) & (1UL << 29))) << 15) | \
	 ((!((cidcsr3_val) & (1UL << 29))) << 16)   \
)

/*
   notes:

   mapping between ci_int_mask and related registers bits:
   ci_int_mask:
   -----------------------------------------------------------------------------------------------
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
   |31|30|29|28|27|26|25|24|23|22|21|20|19|18|17|16|15|14|13|12|11|10| 9| 8| 7| 6| 5| 4| 3| 2| 1| 0|
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
   -----------------------------------------------------------------------------------------------
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--- CICR0[FOM]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------ CICR0[EOFM]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--------- CICR0[SOFM]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------------ CICR0[CDM]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--------------- CICR0[QDM]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------------------ CICR0[PERRM]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--------------------- CICR0[EOLM]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------------------------ CICR0[FEM]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--------------------------- N/A
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------------------------------ CICR0[TOM]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--------------------------------- CICR0[FUM]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------------------------------------ CICR0[BSM]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--------------------------------------- CICR0[EOFM]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------------------------------------------ CIDCSR0[StopIrqEn]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--------------------------------------------- CIDCSR1[StopIrqEn]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------------------------------------------------ CIDCSR2[StopIrqEn]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--------------------------------------------------- CIDCSR3[StopIrqEn]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------------------------------------------------------ N/A
   |  |  |  |  |  |  |  |  |  |  |  |  |  |--------------------------------------------------------- N/A
   |  |  |  |  |  |  |  |  |  |  |  |  |------------------------------------------------------------ N/A
   |  |  |  |  |  |  |  |  |  |  |  |--------------------------------------------------------------- N/A
   |  |  |  |  |  |  |  |  |  |  |------------------------------------------------------------------ N/A
   |  |  |  |  |  |  |  |  |  |--------------------------------------------------------------------- N/A
   |  |  |  |  |  |  |  |  |------------------------------------------------------------------------ N/A
   |  |  |  |  |  |  |  |--------------------------------------------------------------------------- N/A
   |  |  |  |  |  |  |------------------------------------------------------------------------------ N/A
   |  |  |  |  |  |--------------------------------------------------------------------------------- N/A
   |  |  |  |  |------------------------------------------------------------------------------------ N/A
   |  |  |  |--------------------------------------------------------------------------------------- N/A
   |  |  |------------------------------------------------------------------------------------------ N/A
   |  |--------------------------------------------------------------------------------------------- N/A
   |------------------------------------------------------------------------------------------------ N/A


#define CI_INT_MASK(cicr0_val, cidcsr0_val, cidcsr1_val,	\
			cidcsr2_val, cidcsr3_val) \
	(((cicr0_val) & CI_CICR0_INTERRUPT_MASK) | \
	((!((cidcsr0_val) & (1UL << 29))) << 13) | \
	((!((cidcsr1_val) & (1UL << 29))) << 14) | \
	((!((cidcsr2_val) & (1UL << 29))) << 15) | \
	((!((cidcsr3_val) & (1UL << 29))) << 16)   \
)
 */

/* Interrupt status */

/* End-of-Frame for Channel 3, depending on Channel 0
 * DMA channel descriptor Chain configuration
 */
#define CI_INTSTATUS_EOF3              ( 1 << 0  )
/* FIFO Underrun for Channel 3, masked by CI_INT_FU */
#define CI_INTSTATUS_FU_3              ( 1 << 1  )
/* End-of-Frame, masked by CI_INT_EOF */
#define CI_INTSTATUS_EOF               ( 1 << 3  )
/* Start-of-Frame, masked by CI_INT_SOF */
#define CI_INTSTATUS_SOF               ( 1 << 4  )
/* Quick Capture Interface Disable Done, masked by CI_INT_CDD */
#define CI_INTSTATUS_CDD               ( 1 << 5  )
/* Quick Capture Interface Quick Disable Status, masked by CI_INT_CQD */
#define CI_INTSTATUS_CQD               ( 1 << 6  )
/* Parity Error, masked by CI_INT_PAR_ERR */
#define CI_INTSTATUS_PAR_ERR           ( 1 << 7  )
/* End of Line, masked by CI_INT_EOL */
#define CI_INTSTATUS_EOL               ( 1 << 8  )
/* Histogram Interrupt, unmaskable */
#define CI_INTSTATUS_HST_INT           ( 1 << 9  )
/* Compander Interrupt, unmaskable */
#define CI_INTSTATUS_CGC_INT           ( 1 << 10 )
/* FIFO Overrun for Channel 0, masked by CI_INT_IFO */
#define CI_INTSTATUS_IFO_0             ( 1 << 11 )
/* FIFO Overrun for Channel 1, masked by CI_INT_IFO */
#define CI_INTSTATUS_IFO_1             ( 1 << 12 )
/* FIFO Overrun for Channel 2, masked by CI_INT_IFO */
#define CI_INTSTATUS_IFO_2             ( 1 << 13 )
/* FIFO Overrun for Channel 3, masked by CI_INT_IFO */
#define CI_INTSTATUS_IFO_3             ( 1 << 14 )
/* FIFO Time-out, masked by CI_INT_FTO */
#define CI_INTSTATUS_FTO               ( 1 << 15 )
/* Branch Status for Channel 0, masked by CI_INT_BS */
#define CI_INTSTATUS_BS0               ( 1 << 16 )
/* Branch Status for Channel 1, masked by CI_INT_BS */
#define CI_INTSTATUS_BS1               ( 1 << 17 )
/* Branch Status for Channel 2, masked by CI_INT_BS */
#define CI_INTSTATUS_BS2               ( 1 << 18 )
/* Branch Status for Channel 3, masked by CI_INT_BS */
#define CI_INTSTATUS_BS3               ( 1 << 19 )
/* Start-of-Frame for Channel 0, depending on Channel 0
 * DMA channel descriptor Chain configuration
 */
#define CI_INTSTATUS_SOF0              ( 1 << 20 )
/* Start-of-Frame for Channel 1, depending on Channel 1
 * DMA channel descriptor Chain configuration
 */
#define CI_INTSTATUS_SOF1              ( 1 << 21 )
/* Start-of-Frame for Channel 2, depending on Channel 2
 * DMA channel descriptor Chain configuration
 */
#define CI_INTSTATUS_SOF2              ( 1 << 22 )
/* Start-of-Frame for Channel 3, depending on Channel 3
 * DMA channel descriptor Chain configuration
 */
#define CI_INTSTATUS_SOF3              ( 1 << 23 )
/* Stop interrupt Channel 0, masked by CI_INT_SC0 */
#define CI_INTSTATUS_SC0               ( 1 << 24 )
/* Stop interrupt Channel 1, masked by CI_INT_SC1 */
#define CI_INTSTATUS_SC1               ( 1 << 25 )
/* Stop interrupt Channel 2, masked by CI_INT_SC2 */
#define CI_INTSTATUS_SC2               ( 1 << 26 )
/* Stop interrupt Channel 3, masked by CI_INT_SC3 */
#define CI_INTSTATUS_SC3               ( 1 << 27 )
/* Bus error in One or more DMA channels */
#define CI_INTSTATUS_BUSERR            ( 1 << 28 )
/* Subsequent Interrupt Status, unmaskable */
#define CI_INTSTATUS_SINT              ( 1 << 31 )
/* End-of-Frame Transferred to Memory (Channel 0-2,
 * not Channel 3) masked by CI_INT_EOFX
 */
#define CI_INTSTATUS_EOFX              ( 1 << 30 )

#define CISR_VAL(ci_int_status) \
	((ci_int_status) & CI_CISR_MASK)

#define CIFSR_VAL(ci_int_status) \
	( \
	  ((((ci_int_status) & (0xf << 11)) >> 11) << 0  ) | \
	  ((((ci_int_status) & (0x1 << 1 )) >>  1) << 28 ) | \
	  ((((ci_int_status) & (0xf << 16)) >> 16) << 21 ) | \
	  ((((ci_int_status) & (0x1 << 0 )) >> 0 ) << 10 ) | \
	  ((((ci_int_status) & (0xf << 20)) >> 20) << 14 )   \
	)

#define CIDCSR0_STATUS_VAL(ci_int_status) \
	((((ci_int_status) & (0x1 << 24)) >> 24) << 3  )

#define CIDCSR1_STATUS_VAL(ci_int_status) \
	((((ci_int_status) & (0x1 << 25)) >> 25) << 3  )

#define CIDCSR2_STATUS_VAL(ci_int_status) \
	((((ci_int_status) & (0x1 << 26)) >> 26) << 3  )

#define CIDCSR3_STATUS_VAL(ci_int_status) \
	((((ci_int_status) & (0x1 << 27)) >> 27) << 3  )

#define CI_INT_STATUS(cisr_val, cifsr_val, cidcsr0_val,		\
		 cidcsr1_val, cidcsr2_val, cidcsr3_val)		\
	(((cisr_val) & CI_CISR_MASK) |				\
	 ((((cifsr_val) & (0xf << 0 ) ) >> 0  ) << 11 ) |	\
	 ((((cifsr_val) & (0x1 << 10) ) >> 10 ) << 0  ) |	\
	 ((((cifsr_val) & (0xf << 14) ) >> 14 ) << 20 ) |	\
	 ((((cifsr_val) & (0xf << 21) ) >> 21 ) << 16 ) |	\
	 ((((cifsr_val) & (0x1 << 28) ) >> 28 ) << 1  ) |	\
	 ((((cidcsr0_val) & (0x1 << 3)) >> 3  ) << 24 ) |	\
	 ((((cidcsr1_val) & (0x1 << 3)) >> 3  ) << 25 ) |	\
	 ((((cidcsr2_val) & (0x1 << 3)) >> 3  ) << 26 ) |	\
	 ((((cidcsr3_val) & (0x1 << 3)) >> 3  ) << 27 ) |	\
	 ((((cidcsr0_val) & (0x1 << 0)) >> 0  ) << 28 ) |	\
	 ((((cidcsr1_val) & (0x1 << 0)) >> 0  ) << 28 ) |	\
	 ((((cidcsr2_val) & (0x1 << 0)) >> 0  ) << 28 ) |	\
	 ((((cidcsr3_val) & (0x1 << 0)) >> 0  ) << 28 )  	\
	)

/*
   note:

   mapping between ci_int_status and related registers bits:
   ci_int_status:
   -----------------------------------------------------------------------------------------------
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
   |31|30|29|28|27|26|25|24|23|22|21|20|19|18|17|16|15|14|13|12|11|10| 9| 8| 7| 6| 5| 4| 3| 2| 1| 0|
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
   -----------------------------------------------------------------------------------------------
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--- CIFSR[EOF3]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------ CIFSR[FU_3]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--------- N/A
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------------ CISR[EOF]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--------------- CISR[SOF]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------------------ CISR[CDD]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--------------------- CISR[CQD]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------------------------ CISR[PAR_ERR]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--------------------------- CISR[EOL]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------------------------------ CISR[HST_INT]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--------------------------------- CISR[CGC_INT]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------------------------------------ CIFSR[IFO_0]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--------------------------------------- CIFSR[IFO_1]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------------------------------------------ CIFSR[IFO_2]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--------------------------------------------- CIFSR[IFO_3]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------------------------------------------------ CISR[FTO]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |--------------------------------------------------- CIFSR[BS0]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |------------------------------------------------------ CIFSR[BS1]
   |  |  |  |  |  |  |  |  |  |  |  |  |  |--------------------------------------------------------- CIFSR[BS2]
   |  |  |  |  |  |  |  |  |  |  |  |  |------------------------------------------------------------ CIFSR[BS3]
   |  |  |  |  |  |  |  |  |  |  |  |--------------------------------------------------------------- CIFSR[SOF0]
   |  |  |  |  |  |  |  |  |  |  |------------------------------------------------------------------ CIFSR[SOF1]
   |  |  |  |  |  |  |  |  |  |--------------------------------------------------------------------- CIFSR[SOF2]
   |  |  |  |  |  |  |  |  |------------------------------------------------------------------------ CIFSR[SOF3]
   |  |  |  |  |  |  |  |--------------------------------------------------------------------------- CIDCSR0[StopIrq]
   |  |  |  |  |  |  |------------------------------------------------------------------------------ CIDCSR1[StopIrq]
   |  |  |  |  |  |--------------------------------------------------------------------------------- CIDCSR2[StopIrq]]
   |  |  |  |  |------------------------------------------------------------------------------------ CIDCSR3[StopIrq]
   |  |  |  |--------------------------------------------------------------------------------------- CIDCSR0[BusErrIntr] | CIDCSR1[BusErrIntr] | CIDCSR2[BusErrIntr]| CIDCSR3[BusErrIntr]
   |  |  |------------------------------------------------------------------------------------------ N/A
   |  |--------------------------------------------------------------------------------------------- CISR[EOFX]
   |------------------------------------------------------------------------------------------------ CISR[SINT]
*/

typedef enum CI_MODE{
	CI_MODE_MP,             /* Master-Parallel */
	CI_MODE_SP,             /* Slave-Parallel */
	CI_MODE_MS,             /* Master-Serial */
	CI_MODE_EP,             /* Embedded-Parallel */
	CI_MODE_ES              /* Embedded-Serial */
} CI_MODE;


typedef enum  {
	CI_FR_ALL = 0,          /* Capture all incoming frames */
	CI_FR_1_2,              /* Capture 1 out of every 2 frames */
	CI_FR_1_3,              /* Capture 1 out of every 3 frames */
	CI_FR_1_4,
	CI_FR_1_5,
	CI_FR_1_6,
	CI_FR_1_7,
	CI_FR_1_8
} CI_FRAME_CAPTURE_RATE;


typedef enum  {
	CI_FIFO_THL_32 = 0,
	CI_FIFO_THL_64,
	CI_FIFO_THL_96
} CI_FIFO_THRESHOLD;

typedef struct {
	unsigned int BFW;
	unsigned int BLW;
} CI_MP_TIMING, CI_MS_TIMING;

typedef struct {
	unsigned int BLW;
	unsigned int ELW;
	unsigned int HSW;
	unsigned int BFPW;
	unsigned int FSW;
	unsigned int BFW;
	unsigned int EFW;
	unsigned int VSW;
} CI_SP_TIMING;

typedef enum {
	CI_DATA_WIDTH4 = 0x0,
	CI_DATA_WIDTH5 = 0x1,
	CI_DATA_WIDTH8 = 0x2,
	CI_DATA_WIDTH9 = 0x3,
	CI_DATA_WIDTH10= 0x4
} CI_DATA_WIDTH;

typedef enum {
	CI_NO_DOWN_SAMPLE,
	CI_YUV_420_DOWN_SAMPLE,
} CI_CICR4_YCBCR_DOWN_SAMPLE;
/*****************************************************************************
 *	Configuration APIs
 *****************************************************************************/
void ci_set_frame_rate(CI_FRAME_CAPTURE_RATE frate);
CI_FRAME_CAPTURE_RATE ci_get_frame_rate(void);
void ci_set_image_format(int input_format, int output_format);
void ci_set_mode(CI_MODE mode, CI_DATA_WIDTH data_width);
void ci_configure_mp(unsigned int PPL, unsigned int LPF, CI_MP_TIMING* timing);
void ci_configure_sp(unsigned int PPL, unsigned int LPF, CI_SP_TIMING* timing);
void ci_configure_ms(unsigned int PPL, unsigned int LPF, CI_MS_TIMING* timing);
void ci_configure_ep(int parity_check);
void ci_configure_es(int parity_check);
void ci_set_clock(int pclk_enable, int mclk_enable, unsigned int mclk_mhz);
void ci_set_polarity(int pclk_sample_falling, int hsync_active_low,
	int vsync_active_low);
void ci_set_fifo(unsigned int timeout, CI_FIFO_THRESHOLD threshold,
	int fifo1_enable, int fifo2_enable);
void ci_reset_fifo(void);
void ci_set_interrupt_mask(unsigned int mask);
unsigned int ci_get_interrupt_mask(void);
void ci_clear_interrupt_status(unsigned int status);
unsigned int ci_get_interrupt_status(void);
void ci_set_register_value(unsigned int reg_offset, unsigned int value);
void ci_disable_complete(void);


/*JPEG*/

void ci_jpg_enable(void);

void ci_jpg_disable(void);

/* return 1, error occur; return 0, no error */
unsigned int ci_jpg_error(void);

/* return val is account */
unsigned int ci_jpg_cnt_get(void);

void ci_jpg_cnt_clear(void);



/*420 <--> 422*/

int ci_set_ycbcr_420_down_sample (CI_CICR4_YCBCR_DOWN_SAMPLE ycbcr_ds);

/*****************************************************************************
 *	Control APIs
 *****************************************************************************/
void ci_set_reg_base(void __iomem *reg_base, unsigned int phy_base);
unsigned int ci_get_p_add(unsigned int off);
unsigned int ci_get(unsigned int off);

void ci_init(void);
void ci_deinit(void);
void ci_enable(void);
int  ci_disable(int quick, int wait_for_disable_complete );
void ci_slave_capture_enable(void);
void ci_slave_capture_disable(void);

/*****************************************************************************
 *	CI RAW data processing chain APIs
 *****************************************************************************/

/* Histogram Unit(HSU) related functions */
typedef enum {
	CI_HISTO_RED     = CI_CIHST_COLOR_RED,
	CI_HISTO_BLUE    = CI_CIHST_COLOR_BLUE,
	CI_HISTO_GREEN1  = CI_CIHST_COLOR_GREEN1,
	CI_HISTO_GREEN2  = CI_CIHST_COLOR_GREEN2
} CI_HSU_COLOR_TYPE;

typedef enum {
	CI_HSU_MUX_0_TO_7  = CI_CIHST_SCALE_0_TO_7,    /* for 8bit raw data */
	CI_HSU_MUX_0_TO_8  = CI_CIHST_SCALE_0_TO_8,    /* for 9bit raw data */
	CI_HSU_MUX_1_TO_9  = CI_CIHST_SCALE_1_TO_9     /* for 10bit raw data */
} CI_HSU_MUX_SEL_TYPE;

int ci_hsu_get_histgram (
		CI_HSU_COLOR_TYPE  color_type,
		CI_HSU_MUX_SEL_TYPE mux_select,
		unsigned int *lut_buffer_virtual,
		unsigned int  lut_buffer_physical ,
		unsigned int *lut_dma_desc_virtual,
		unsigned int  lut_dma_desc_physical,
		unsigned int  size,
		unsigned int *sum);

/* Pixel Substitute(PSU) related functions */
int ci_psu_tag_bad_pixel(int column, int row);

int ci_psu_enable(int enable);

/* Compand and Gamma Correction (CGU) related functions */
typedef enum {
	CI_CGU_MUX_0_TO_7  = CI_CICCR_SCALE_0_TO_7,    /* for 8bit raw data */
	CI_CGU_MUX_1_TO_8  = CI_CICCR_SCALE_1_TO_8,    /* for 9bit raw data */
	CI_CGU_MUX_2_TO_9  = CI_CICCR_SCALE_2_TO_9     /* for 10bit raw data */
} CI_CGU_MUX_SEL_TYPE;

int ci_cgu_set_addr_mux_select(CI_CGU_MUX_SEL_TYPE mux_select);

typedef enum {
	CI_CGU_LUT_RED      = CI_CICCR_CLUT_RED,
	CI_CGU_LUT_BLUE     = CI_CICCR_CLUT_BLUE,
	CI_CGU_LUT_GREEN    = CI_CICCR_CLUT_GREEN
} CI_CGU_LUT_TYPE;

int ci_cgu_load_lut_ram(
		unsigned int  *histogram_lut_buffer_virtual,
		unsigned int   histogram_lut_buffer_physical,
		unsigned int  *histogram_lut_dma_descriptors_virtual,
		unsigned int   histogram_lut_dma_descriptors_physical,
		unsigned char *lut_ram);

int ci_cgu_set_black_level(unsigned char black_level);

int ci_cgu_enable(int enable);


#define CI_SSU_SCALE_DEFAULT CI_SSU_SCALE_DISABLE

int ci_ssu_set_scale(CI_SSU_SCALE scale);

/* Color Synthesis Unit(CSU) related functions */

/* Color Management Unit(CMU) related functions */
typedef enum {
	CI_CMU_DISABLE    = CI_CICMR_DMODE_DISABLE,
	CI_CMU_OUTPUT_RGB = CI_CICMR_DMODE_RGB,
	CI_CMU_OUTPUT_YUV = CI_CICMR_DMODE_YUV
} CI_CMU_USAGE;

typedef struct
{
	signed short k00, k01, k02;
	signed short k10, k11, k12;
	signed short k20, k21, k22;
} CI_CMU_COE_MATRIX;

/* just for debug use, will be removed later */
#define YUV_FLOAT_TO_INT(x) ((signed short)((float)x*(1UL << 7)) & 0x3ff)
/* example:
   static CI_CMU_COE_MATRIX cRGB24_to_YUV422_matrix = {
   YUV_FLOAT_TO_INT(0.257) , YUV_FLOAT_TO_INT(0.504) , YUV_FLOAT_TO_INT(0.098),
   YUV_FLOAT_TO_INT(-0.148),  YUV_FLOAT_TO_INT(0.291), YUV_FLOAT_TO_INT(0.439),
   YUV_FLOAT_TO_INT(0.439) , YUV_FLOAT_TO_INT(0.368) , YUV_FLOAT_TO_INT(0.071)
   };
 */

/* just for debug use, will be removed later */
#define RGB_FLOAT_TO_INT(x) ((signed short)((float)x*(1UL << 7)) & 0x3ff)
/* example:
   static CI_CMU_COE_MATRIX cRGB24_to_sRGB24_matrix = {
   RGB_FLOAT_TO_INT(1.780214),  RGB_FLOAT_TO_INT(-0.96883),
   RGB_FLOAT_TO_INT(0.188617),  RGB_FLOAT_TO_INT(-0.7987),
   RGB_FLOAT_TO_INT(1.790752), RGB_FLOAT_TO_INT(0.007949),
   RGB_FLOAT_TO_INT(-0.67645),  RGB_FLOAT_TO_INT(-1.60901),
   RGB_FLOAT_TO_INT(3.285467),
   };
 */

int ci_cmu_set_color_correction_coe(CI_CMU_COE_MATRIX *coe_matrix);

int ci_cmu_enable(CI_CMU_USAGE cmu_usage);


/*****************************************************************************
 *	CI dedicated DMAC APIs
 *****************************************************************************/
typedef enum {
	CI_DMA_CHANNEL_0 = 0,
	CI_DMA_CHANNEL_1,
	CI_DMA_CHANNEL_2,
	CI_DMA_CHANNEL_3
} CI_DMA_CHANNEL;

typedef struct
{
	volatile u32     ddadr;  /* descriptor address reg */
	volatile u32     dsadr;  /* source address register */
	volatile u32     dtadr;  /* target address register */
	volatile u32     dcmd ;  /* command address register */
} CI_DMAC_DESCRIPTOR_T, *P_CI_DMAC_DESCRIPTOR_T;

#define CI_DMAC_DCMD_LEN                  ( 1 << 0  )
#define CI_DMAC_DCMD_EOF_IRQ_EN           ( 1 << 21 )
#define CI_DMAC_DCMD_SOF_IRQ_EN           ( 1 << 22 )
#define CI_DMAC_DCMD_INC_TRG_ADDR         ( 1 << 30 )


int ci_dma_load_descriptor(unsigned int dma_desc_phy, CI_DMA_CHANNEL channel);

int ci_dma_set_branch (
		unsigned int branch_to_dma_desc_phy,
		int branch_int_enable,
		int branch_after_cur_frame,
		CI_DMA_CHANNEL channel);

/*****************************************************************************
 *          Macros
 *
 *****************************************************************************/

/* Interrupt mask */
#define CAMERA_INTMASK_FIFO_OVERRUN	CI_INT_IFO
#define CAMERA_INTMASK_END_OF_FRAME	CI_INT_EOF
#define CAMERA_INTMASK_START_OF_FRAME	CI_INT_SOF
#define CAMERA_INTMASK_CI_DISABLE_DONE	CI_INT_CDD
#define CAMERA_INTMASK_CI_QUICK_DISABLE	CI_INT_CQD
#define CAMERA_INTMASK_PARITY_ERROR	CI_INT_PAR_ERR
#define CAMERA_INTMASK_END_OF_LINE	CI_INT_EOL
#define CAMERA_INTMASK_FIFO_EMPTY	CI_INT_FEMPTY
#define CAMERA_INTMASK_TIME_OUT		CI_INT_FTO
#define CAMERA_INTMASK_FIFO3_UNDERRUN	CI_INT_FU
#define CAMERA_INTMASK_BRANCH_STATUS	CI_INT_BS
#define CAMERA_INTMASK_ENF_OF_FRAME_TRANSFER	CI_INT_EOFX
#define CAMERA_INTMASK_DMA_CHANNEL0_STOP	CI_INT_SC0
#define CAMERA_INTMASK_DMA_CHANNEL1_STOP	CI_INT_SC1
#define CAMERA_INTMASK_DMA_CHANNEL2_STOP	CI_INT_SC2
#define CAMERA_INTMASK_DMA_CHANNEL3_STOP	CI_INT_SC3


/* Capture mode */
#define CAMERA_MODE_VIDEO		0x0000
#define CAMERA_MODE_STILL		0x0001
#define CAMERA_MODE_NUM		2


/* buffer type */
#define VIDEO_CAPTURE_BUFFER		0
#define STILL_CAPTURE_BUFFER		1

/* clean buffer identifier */
#define CAMERA_CLEAN_BUFFER_IDENTIFIER	0x12345678

#define CAMERA_CAPTURE_SCALE_DISABLE     CI_SSU_SCALE_DISABLE
#define CAMERA_CAPTURE_SCALE_HALF        CI_SSU_SCALE_HALF
#define CAMERA_CAPTURE_SCALE_QUATER      CI_SSU_SCALE_QUARTER

/* sensor capabilities */
#define SENSOR_CAP_MANUAL_CONTRAST       0x0001
#define SENSOR_CAP_MANUAL_WHITEBALANCE   0x0002
#define SENSOR_CAP_MANUAL_EXPOSURE       0x0004
#define SENSOR_CAP_MANUAL_ZOOM           0x0008

/* sensor power mode */
#define CAMERA_POWER_OFF                 0
#define CAMERA_POWER_LOW                 1
#define CAMERA_POWER_MID                 2
#define CAMERA_POWER_FULL                3

/* sensor contrast mode and value */
#define SENSOR_MANUAL_CONTRAST           0
#define SENSOR_AUTO_CONTRAST             1

#define SENSOR_CONTRAST_LOWEST           0
#define SENSOR_CONTRAST_LOW              1
#define SENSOR_CONTRAST_MIDDLE           2
#define SENSOR_CONTRAST_HIGH             3
#define SENSOR_CONTRAST_HIGHEST          4

/* sensor white balance mode and value */
#define SENSOR_MANUAL_WHITEBALANCE       0
#define SENSOR_AUTO_WHITEBALANCE         1

#define SENSOR_WHITEBALANCE_AUTO         0
#define SENSOR_WHITEBALANCE_INCANDESCENT 1
#define SENSOR_WHITEBALANCE_SUNNY        2
#define SENSOR_WHITEBALANCE_FLUORESCENT  3

/* sensor exposure mode and value */
#define SENSOR_MANUAL_EXPOSURE           0
#define SENSOR_AUTO_EXPOSURE             1

#define SENSOR_EXPOSURE_LOWEST           0
#define SENSOR_EXPOSURE_LOW              1
#define SENSOR_EXPOSURE_MIDDLE           2
#define SENSOR_EXPOSURE_HIGH             3
#define SENSOR_EXPOSURE_HIGHEST          4

/* Buffer Pool Information */
/* Max Buffer number of Camera Buffer Pool has 64
 * First 32bytes: phantom buffer
 * Last 16 bytes: init dma descriptor for all channels
 * 32 bytes are enough: clear DCMD[IncTrgAddr] of
 * phantom buffer's DMA descriptor so as to save
 * memory, since phantom buffer is useless.
 */
#define MAX_CAMERA_FRAME_BUFFERS	32

#define PHANTOM_BUFFER_SIZE		112

/* Frame Buffer Information */
/* each channel of frame buffer has 3
 * special DMA descriptors, two for real
 * buffer, another for phantom buffer.
 */
#define SPECIAL_DMA_DESC_NUM_PER_CHANNEL_PER_FRAME      4

/* the first DMA descriptor of real buffer */
#define FIRST_DMA_DESC_FOR_REAL_BUFFER_INX              0

/* the last DMA descriptor of real buffer */
#define LAST_DMA_DESC_FOR_REAL_BUFFER_INX               1

/* the first DMA descriptor of phantom buffer */
#define FIRST_DMA_DESC_FOR_PHANTOM_BUFFER_INX           2

/* the last DMA descriptor of phantom buffer */
#define LAST_DMA_DESC_FOR_PHANTOM_BUFFER_INX            3

/*****************************************************************************
 *          Structures
 *****************************************************************************/
typedef struct camera_context_s camera_context_t, *p_camera_context_t;


typedef enum {
	YUV_HAVE_PADDING   = 0,
	YUV_NO_PADDING     = 1
} PADDING_TYPE;

typedef struct {
	
	char * name;
	int id;
	int *format_list;

	int width_max;
	int height_max;
	int width_min;
	int height_min;	

	int v_power;
	int v_io;
	
	int (*init)(p_camera_context_t camera_context);
	int (*deinit)(p_camera_context_t camera_context);
	int (*set_capture_format)( p_camera_context_t camera_context );
	int (*start_capture)( p_camera_context_t camera_context ,
			unsigned int frames);
	int (*stop_capture)( p_camera_context_t camera_context );
	int (*sleep)( p_camera_context_t camera_context );
	int (*wakeup)( p_camera_context_t camera_context );

	/*  functions for sensor control */
#if 0
	int (*read_8bit)( p_camera_context_t camera_context,
			u8 reg_addr, u8 *reg_val);
	int (*write_8bit)( p_camera_context_t camera_context,
			u8 reg_addr, u8 reg_val);
	int (*read_16bit)( p_camera_context_t camera_context,
			u16 reg_addr, u16 *reg_val);
	int (*write_16bit)( p_camera_context_t camera_context,
			u16 reg_addr, u16 reg_val);
	int (*read_32bit)( p_camera_context_t camera_context,
			u32 reg_addr, u32 *reg_val);
	int (*write_32bit)( p_camera_context_t camera_context,
			u32 reg_addr, u32 reg_val);
#else
	int (*read_reg)( p_camera_context_t camera_context,
			u32 reg_addr, u32 *reg_val);
	int (*write_reg)( p_camera_context_t camera_context,
			u32 reg_addr, u32 reg_val);
#endif
	int (*set_power_mode)(p_camera_context_t camera_context, u8 power_mode);
#if 0
	int (*set_contrast) ( p_camera_context_t camera_context,
			u8 mode, u32 value);
	int (*set_whitebalance) (p_camera_context_t camera_context,
			u8 mode, u32 value);
	int (*set_exposure) (p_camera_context_t camera_context,
			u8 mode, u32 value );
	int (*set_zoom) (p_camera_context_t camera_context, u32 value);
#else
	int (*set_mode) (p_camera_context_t camera_context, u32 mode, u32 value );
#endif
	int (*get_mode) (p_camera_context_t camera_context, u32 mode, u32* value );

	int (*get_framerate)(u32 format, u32 width, u32 height, u32 *numerator, u32 *denominator);

	int (*load_lut)(p_camera_context_t camera_context);
} camera_function_t, *p_camera_function_t;


typedef struct {
	unsigned char power_mode;

	unsigned int caps;
	/*  1 << 0:    Manual ContrastMode supported by sensor;
	 *  1 << 1:    Manual whitebalance supported by sensor;
	 *  1 << 2:    Manual exposure supported by sensor;
	 *  1 << 3:    Manual Zoom supported by sensor;
	 */

	unsigned char contrast_mode;
	unsigned char whitebalance_mode;
	unsigned char exposure_mode;

	unsigned int contrast_value;
	unsigned int whitebalance_value;
	unsigned int exposure_value;
	unsigned int zoom_value;
} sensor_status_t, *p_sensor_status_t;

typedef struct camera_frame_buffer_info_s camera_frame_buffer_info_t,\
		*p_camera_frame_buffer_info_t;
typedef struct camera_frame_buffer_queue_s camera_frame_buffer_queue_t,\
		*p_camera_frame_buffer_queue_t;

struct camera_frame_buffer_info_s {
	/* Information for frame buffer itself */

	/* Frame ID, indicates this buffer's
	 * position in the buffer pool.
	 */
	int			frame_id;

	/* the virtual address of this buffer,
	 * should be 1-page aligned. the physical
	 * address of the buffer is not defined here
	 * because the physical address may be an
	 * array and should be maintained by camera
	 * driver to save memory.
	 */
	volatile u32		*buffer_vir_addr;

	/* the buffer size, can be obtained via
	 * mcam_get_buffer_size( ) before
	 * allocating the memory for the buffer.
	 */
	int			buffer_size;

	/* the virtual address of the memory for DMA descriptors
	 * for this buffer, should be 16-bytes aligned.
	 */
	CI_DMAC_DESCRIPTOR_T	*dma_descriptors_virtual;

	/* the virtual address of the Y component */
	void			*pY;

	/* the virtual address of the Cb component */
	void			*pCb;

	/* the virtual address of the Cr component */
	void			*pCr;

	/* Information of the DMA descriptor Chains for this frame buffer */
	/* pointer to an array of physical address of
	 * channel s DMA chain for this buffer;
	 */
	volatile u32		dma_desc_phy_addr[3][SPECIAL_DMA_DESC_NUM_PER_CHANNEL_PER_FRAME];

	/* pointer to an array of virtual address of
	 * channel s DMA chain for this buffer;
	 */
	CI_DMAC_DESCRIPTOR_T	*dma_desc_vir_addr[3][SPECIAL_DMA_DESC_NUM_PER_CHANNEL_PER_FRAME];

	/* DMA descriptors number for channel  */
	int			dma_desc_num[3];

	/* DMA descriptors number of phantom buffer for channel  */
	int			phantom_dma_desc_num[3];

	/* next buffer in the queue, only used in
	 * video capture buffer queue or still
	 * capture buffer queue.
	 */
	camera_frame_buffer_info_t	*next_buffer;
};

struct camera_frame_buffer_queue_s {
	/* first frame buffer owned by the queue. When
	 * this buffer will be reported the
	 * application when it is filled with frame
	 * data, then this pointer will point to the
	 * next buffer in the queue. If the queue runs
	 * out of its buffers, this pointer will be
	 * NULL until one new free buffer added into
	 * the queue when this pointer will point to
	 * this new buffer.
	 */
	camera_frame_buffer_info_t   *head;

	/* last frame buffer in the queue, always
	 * followed by the phantom buffer. When there
	 * is no frame available in the queue, the DMA
	 * controller will transfer the frame data to
	 * its phantom buffer until the Camera driver
	 * submits new frames into the queue. The
	 * frame complete interrupt for phantom buffer
	 * should be ignored.
         */
	camera_frame_buffer_info_t   *tail;
};

#define SENSOR_LOW 0
#define SENSOR_HIGH 1
#define SENSOR_MAX 2


/*
 * buffer list
 */

struct buf_node {
	unsigned int io_type;           /* camera IO methods */
	struct list_head buf_head;	/* For buffer list */
	struct list_head report_head;	/* For report buffer list */
	void *vaddr;		/* vmap() return virtual address */
	struct page **pages;	/* physical pages */
	int page_num;		/* physical pages count */
	int buf_id;		/* buffer id to let driver access */
	int buf_index;		/* buffer index */
	int size;		/* buffer size */
	void *dma_desc_vaddr;	/* dma description virtual address */
	dma_addr_t dma_desc_paddr;	/* dma description physical address */
	int dma_desc_size;	/* dma description size */
	void *Y_vaddr;		/* Y virtual address */
	void *Cb_vaddr;		/* Cb virtual address */
	void *Cr_vaddr;		/* Cr virtual address */
	unsigned long paddr;	

	int fifo0_size;		/* fifo0 data transfer size */
	int fifo1_size;		/* fifo1 data transfer size */
	int fifo2_size;		/* fifo2 data transfer size */

	int bytesused;
	int offset;
};

struct cam_mode_s {

	int 		buf_ready;
	int 		buf_submited;
	int 		capture_started;
	struct 		list_head buf_head;
	struct 		list_head report_head;
	unsigned int 	timeperframe_numerator;
	unsigned int 	timeperframe_denominator;
	
	unsigned int width;
	unsigned int height;

	/* defines the scaling rate of sensor output vs QCI output.
	 * possible choices: 1:1(default) or 2:1 or 4:1.However, if there
	 * is need to convert RAW data to YUV/RGB, then this field must be
	 * set as 2:1 or 4:1
	 */
	unsigned int                       capture_scale;

	unsigned int                       input_format;

	unsigned int                       output_format;

	unsigned int                       fifo_transfer_size[3];

	/* frame buffer pool */
	/* frame buffer pool is maintained within the camera driver.
	 * When the application or camera driver is initialized, the
	 * frame buffer will be allocated, too. Those frame buffers
	 * will be added to this frame buffer pool. Afterward the
	 * application/camera driver will get buffers from this pool
	 * and submits them to the still capture buffer queue or the
	 * video capture buffer queue.
	 */

	camera_frame_buffer_info_t
		master_frame_buffer_list[MAX_CAMERA_FRAME_BUFFERS];

	camera_frame_buffer_queue_t   capture_buffer_queue;
	
};

/* context */
struct camera_context_s {

	int 		*p_buf_ready;
	int 		*p_buf_submited;
	int 		*p_capture_started;
	struct 		list_head *p_buf_head;
	struct 		list_head *p_report_head;
	unsigned int 	*p_timeperframe_numerator;
	unsigned int 	*p_timeperframe_denominator;
	wait_queue_head_t camera_wait_q;
	int 		task_waiting;
	int 		driver_opened;

	int 		i2c_inited;
	int 		suspended;
	

	unsigned int                       sensor_type;
	unsigned int                       capture_mode;

	struct cam_mode_s mode[CAMERA_MODE_NUM];

	/* frame rate control */
	/* frame_rate specifies the desired frame rate control.
	 * Use a 0 to specify the maximum frame rate.  For n > 0,
	 * it specifies that the interface should capture 1 out of
	 * every n+1 frames generated.  A future implementation may
	 * change the interpretation of the frame rate in order to
	 * specify the frames per second.
	 */
	unsigned int                       frame_rate;

	/* memory for phantom buffer */
	/* buffer with PHANTOM_BUFFER_SIZE bytes,
	 * allocated by camera driver, used as phantom buffer by all
	 * buffers, should be 16-bytes aligned.
	 */
	volatile u32                    *phantom_buffer_virtual;
	volatile u32                     phantom_buffer_physical;

	/* memory for getting Histogram or loading CGU LUT */
	/* buffer with 1024 bytes, allocated by camera
	 * driver, used for getting Histogram(512 or 1024bytes) or
	 * loading CGU LUT(64*3bytes), should be 16-bytes aligned.
	 */
	volatile u32                    *histogram_lut_buffer_virtual;
	volatile u32                     histogram_lut_buffer_physical;

	/* memory for DMA descriptors to get Histogram or load CGU LUT */
	/* memory for DMA descriptors allocated by Camera driver,
	 * 16-bytes aligned, should be 16-byte sized. This
	 * descriptor is required to get histogram data or load
 	 * Compander look-up-table (CGU LUT).
	 */
	CI_DMAC_DESCRIPTOR_T         	*histogram_lut_dma_descriptors_virtual;
	volatile u32                     histogram_lut_dma_descriptors_physical;

	/* os mapped register address */
	/* specifies the virtual address of the OST registers */
	/* unsigned int                       ost_reg_base; */

	/* specifies the virtual address of the GPIO registers */
	/* unsigned int                       gpio_reg_base; */

	/* specifies the virtual address of the CAMERA INTERFACE registers */
	unsigned int                       ci_reg_base;
	unsigned int                       ci_reg_end;

	/* specifies the virtual address of the Clock registers registers */
	/* unsigned int                       clk_reg_base; */

	/* function dispatch table */
	/* specify pointers to the given camera sensor functions */
	p_camera_function_t           camera_functions;



	/*ONLY consider two sensor in platform now, one is low resolution, the other is high resolution */

	p_camera_function_t 		sensor_func[SENSOR_MAX];

	/* INTERNALLY USED: DON'T TOUCH! */

	/* Sensor status */
	sensor_status_t               sensor_status;

	/* sensor related parameter or status will be put here */

	/* capture parameters:
	 *    those parameters are being used by Camera currently
	 *    for video capture or still capture, generated from
	 *    driver-filled parameters according to the parameter
	 *    capture_mode.
	 */

	/* current image resolution of the sensor output or QCI
	 * input, may be the same as QCI output, may be not,
	 * depending on the capture scale.
	 */
	unsigned int                       capture_input_width;
	unsigned int                       capture_input_height;

	/* current image resolution of the QCI output */
	unsigned int                       capture_output_width;
	unsigned int                       capture_output_height;

	/* current image format of the sensor output or QCI input */
	unsigned int                       capture_input_format;

	/* current image format of the QCI output */
	unsigned int                       capture_output_format;

	unsigned int                       fifo_transfer_size[3];

	/* indicates how many buffers the buffer pool currently has */
	int                                 frame_buffer_number;

	camera_frame_buffer_queue_t   queue_cur;

	/* QCI status */
	/* indicates whether QCI disabling is completed */
	unsigned int                       ci_disable_complete;

	/* indicates whether PSU is enabled */
	int                        psu_enable;

	/* indicates whether the CGU is enabled */
	int                        cgu_enable;

	/* indicates the SSU scale ratio */
	CI_SSU_SCALE                  ssu_scale;

	/* indicates how to use CMU. This varible will tell the sensor
	 * that it should choose which CMU matrix to load
	 */
	CI_CMU_USAGE                  cmu_usage;

	int                        dma_running;
	 
    	/*align_type indicate whether YUV have padding
           0: have padding
           1: no padding;
	   */
	int                       align_type;

	int branch_flag;

	CI_CICR4_YCBCR_DOWN_SAMPLE		ycbcr_ds;

	int jpg_offset;
	int jpg_left;
	int jpg_ci_count;
	int jpg_new_count;
	int jpg_old_count;
	int jpg_branch;

	int jpg_buf_num;

	int jpg_error;

	int jpg_des_base;
	int jpg_des_pre;
	int jpg_des_cnt;


	int irq;

	atomic_t		atomic_jpg_full;

	struct buf_node *jpg_buf_node;

	struct work_struct	work;
	struct workqueue_struct *cam_wq;

	/* platform data */
	struct cam_platform_data *platform_ops;
	
	/* clock */
	struct clk *clk;

	struct dentry *entry;

#ifdef CONFIG_BOARD_LANDMARK
	int old_mspm_state;
#endif	
};

/***********************************************************************
 *
 * sensor register and chosen
 *
 ***********************************************************************/

/* type is SENSOR_LOW SENSOR_HIGH */
int sensor_register(p_camera_function_t p_fun, int type);

int sensor_unregister(int type);

int sensor_choose(p_camera_context_t cam_ctx, int type);

#define SENSOR_CHECK_WH(camera_context, w, h) \
	(camera_context->capture_input_width==w && camera_context->capture_input_height==h)

#endif

