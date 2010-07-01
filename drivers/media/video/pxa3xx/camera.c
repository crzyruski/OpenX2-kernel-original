/*
 * camera.c - main file for camera driver
 *
 * Copyright (C) 2005, Intel Corporation.
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/pagemap.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <linux/pci.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/cpufreq.h>
#include <linux/list.h>
#include <linux/types.h>
#include <linux/semaphore.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <asm/dma.h>
#include <asm/irq.h>
#include <mach/irqs.h>
#include <mach/pxa3xx_pmic.h>
#include <linux/clk.h>

#include <linux/mm.h>
#include <linux/videodev.h>
#include <linux/videodev2.h>

#include <linux/pxa_camera.h>

#include <mach/camera.h>
#include "camera.h"
#if defined(CONFIG_PXA3xx_DVFM)
#include <mach/dvfm.h>
#include <mach/pxa3xx_dvfm.h>
#include <mach/mspm_prof.h>
#endif

#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/mfp-pxa300.h>

#define CAM_DEBUG_STATIC

#ifdef CAM_DEBUG_STATIC
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif

#define assert(expr) ((void) ((expr) ? 0 : ( \
				printk(KERN_ALERT		\
				"%s: assert failed at line %d",	\
				 __func__, __LINE__))))

#define	CAMERA_DEBUG
#undef CAMERA_DEBUG

#ifdef  CAMERA_DEBUG
#define CAMERA_DEBUG_IRQ
#define	dbg(fmt, arg...)    printk(KERN_INFO "%s(line %d): " fmt "\n", 	\
		__FUNCTION__, __LINE__, ##arg)
#define PRINTFUNC printk(KERN_INFO "%s\n", __FUNCTION__)
#define TESTPOINT do {							\
	printk(KERN_INFO "\n");						\
	printk(KERN_INFO "%s(%s):%d\n", __FUNCTION__, __FILE__, __LINE__);\
} while(0)
#else
#define	dbg(fmt, arg...)	do {} while (0)
#define	PRINTFUNC 		do {} while (0)
#define	TESTPOINT		do {} while (0)
#endif

#define        PXA_CAMERA_VERSION    KERNEL_VERSION(0,0,1)

//#define	DMA_OVERFLOW_WORKAROUND

/* 
 * To workaround the CI DMA overflow issue, We need preserve memory for
 * single DMA descriptor buffer. Also need disable CI in SOF interrupt
 * and enable CI in CDD interrupt again.
 */
#ifdef	DMA_OVERFLOW_WORKAROUND
#define	RESERVE_DMA_BUFFER
#define	CI_USERPOINTER		//for user pointer mode
#define	SINGLE_DMA_DESC
#define	CI_DIS_ENA
#endif

#ifdef	RESERVE_DMA_BUFFER
#define	PRE_DMA_BUF_SIZE	(1024 * 1024 * 2)	//increased from 1M to 2M to increase performance, otherwise can't allocate 4 buffers for VGA size in gstreamer camerasrc
static int g_order;
static struct page *g_dma_page;
static struct page *g_dma_jpeg_page;

static struct page *g_dma_freepage_start;
static void *g_dma_free_start;
static void *g_dma_free_size;
static void *g_dma_jpeg_size;
static spinlock_t dma_buf_lock; 
#endif

#define	JPG_DMA_BUF_SIZE	(4 * 1024 * 1024)

/*
 * main camera driver macros and data
 */

/* This mask just enable EOFX interrupt */
#define	ALL_INT_MASK    (CAMERA_INTMASK_FIFO_OVERRUN |	\
	CAMERA_INTMASK_END_OF_FRAME |			\
	CAMERA_INTMASK_START_OF_FRAME |			\
	CAMERA_INTMASK_CI_DISABLE_DONE |		\
	CAMERA_INTMASK_CI_QUICK_DISABLE |		\
	CAMERA_INTMASK_PARITY_ERROR |			\
	CAMERA_INTMASK_END_OF_LINE |			\
	CAMERA_INTMASK_FIFO_EMPTY  |			\
	CAMERA_INTMASK_TIME_OUT  |			\
	CAMERA_INTMASK_FIFO3_UNDERRUN |			\
	CAMERA_INTMASK_BRANCH_STATUS |			\
	CAMERA_INTMASK_ENF_OF_FRAME_TRANSFER | 		\
	CAMERA_INTMASK_DMA_CHANNEL0_STOP |		\
	CAMERA_INTMASK_DMA_CHANNEL1_STOP |		\
	CAMERA_INTMASK_DMA_CHANNEL2_STOP |		\
	CAMERA_INTMASK_DMA_CHANNEL3_STOP)

#define	JPG_INT_MASK	((ALL_INT_MASK) & ~CAMERA_INTMASK_ENF_OF_FRAME_TRANSFER)

/* If no disable/enable, jpg and preview mode all just enable EOFX interrupt
 * If disable/enable, jpg mode enable EOFX interrupt and preview mode enable
 * SOF and CDD interrupt.
 */
#ifdef	CI_DIS_ENA
#define	PRE_INT_MASK	((ALL_INT_MASK) &			\
	~(CAMERA_INTMASK_START_OF_FRAME | CAMERA_INTMASK_CI_DISABLE_DONE))
#else
#define	PRE_INT_MASK	JPG_INT_MASK
#endif

#define DMA_DESCRIPTOR_SIZE (sizeof(CI_DMAC_DESCRIPTOR_T))

/* default value */
#define WIDTH_DEFT		176
#define HEIGHT_DEFT		144
#define FRAMERATE_DEFT		0

static p_camera_context_t g_camera_context;

#if defined(CONFIG_PXA3xx_DVFM)
static int dvfm_dev_idx;
#endif

/* Internal function */
static int pxa_camera_ioctl_streamon(p_camera_context_t cam_ctx);
static void pxa_camera_ioctl_streamoff(p_camera_context_t cam_ctx);
static int pxa_camera_reset(p_camera_context_t cam_ctx);
static int pxa_camera_get_framerate(p_camera_context_t cam_ctx);

struct semaphore buf_list_sem;
static spinlock_t report_list_lock;	/* Spin lock for report_list */
static spinlock_t cam_queue_lock;	/* Spin lock for queue */

/* page cache */

#define        MAX_PAGE_CACHE         256
struct page_cache_head {
	struct list_head page_list;
	int page_count;
	spinlock_t lock;
};
static struct page_cache_head pc_head;

static void dump_buf(void *buf, int len)
{
	int i;
	unsigned long *tmp = buf;

	for (i = 0; i < len/4; i++) {
		printk("0x%08x\t", tmp[i]);
		if (0 == (i + 1) % 4)
			printk("\n");
	}

	printk("\n");
}

/*****************************************************************************
 *	QCI routines
 *****************************************************************************/
static void __iomem *cam_base;
static unsigned int  phy_cam_base;

#define READ_REG(off)	__raw_readl(cam_base + (off))
#define	WRITE_REG(off, v)	do {					\
	__raw_writel((v), cam_base + (off)); 	\
	__raw_readl(cam_base + (off));			\
}while(0)

unsigned int ci_get_p_add(unsigned int off)
{
	return (phy_cam_base + off);
}

unsigned int ci_get(unsigned int off)
{
	return READ_REG(off);
}

void ci_set_reg_base(void __iomem *reg_base, unsigned int phy_base)
{
	cam_base = reg_base;
	phy_cam_base = phy_base;
}

void ci_set_frame_rate(CI_FRAME_CAPTURE_RATE frate)
{
	volatile unsigned int value;
	/* write cicr4 */
	value = READ_REG(CICR4);
	value &= ~(CI_CICR4_FR_RATE_SMASK << CI_CICR4_FR_RATE_SHIFT);
	value |= (unsigned)frate << CI_CICR4_FR_RATE_SHIFT;
	WRITE_REG(CICR4, value);
}

/*if support more format, please add accordingly */
void ci_set_image_format(int input_format, int output_format)
{
	volatile unsigned int value, tbit, rgbt_conv, rgb_conv, rgb_f;
	volatile unsigned int ycbcr_f, rgb_bpp, raw_bpp, cspace;

	/* write cicr1: preserve ppl value and data width value */
	value = READ_REG(CICR1);
	value &= ( (CI_CICR1_PPL_SMASK << CI_CICR1_PPL_SHIFT) |
		((CI_CICR1_DW_SMASK) << CI_CICR1_DW_SHIFT));

	tbit = rgbt_conv = rgb_conv = rgb_f = ycbcr_f = rgb_bpp	\
		= raw_bpp = cspace = 0;

	switch(input_format) {
		case V4L2_PIX_FMT_JPEG:
		case V4L2_PIX_FMT_SRGGB8:
			cspace = 0;
			raw_bpp = 0;
			break;
		case V4L2_PIX_FMT_SRGGB10:
			cspace = 0;
			raw_bpp = 2;
			break;

		case V4L2_PIX_FMT_YUV422P:
			cspace = 2;

			if (cpu_is_pxa310() || cpu_is_pxa930() || cpu_is_pxa935()){

				if (output_format == V4L2_PIX_FMT_YUV422P 
						|| output_format == V4L2_PIX_FMT_YUV420) {
					ycbcr_f = 1;
				}
			}else{
				if (output_format == V4L2_PIX_FMT_YUV422P) {
					ycbcr_f = 1;
				}
			}
			break;
		case V4L2_PIX_FMT_RGB565X:
			cspace = 3;
			rgb_bpp = 2;
			rgb_f = 1;
			break;
		case V4L2_PIX_FMT_RGB24:
			cspace = 1;
			rgb_bpp = 4;
			break;
		default:
			break;
	}

	switch(input_format) {
		case V4L2_PIX_FMT_SRGGB8:
		case V4L2_PIX_FMT_SRGGB10:
			if (cpu_is_pxa310() || cpu_is_pxa930() || cpu_is_pxa935()){

				if (output_format == V4L2_PIX_FMT_YUV422P 
						|| output_format == V4L2_PIX_FMT_YUV420) {
					ycbcr_f = 1;
				}
			}else{
				if (output_format == V4L2_PIX_FMT_YUV422P){
					ycbcr_f = 1;
				}
			}

		case V4L2_PIX_FMT_RGB24:
			switch(output_format) {
				case V4L2_PIX_FMT_RGB24:
					rgb_f = 1;
					break;
				case V4L2_PIX_FMT_RGB565X:
					rgb_conv = 2;
					break;
				default:
					break;
			}
		default:
			break;
	}

	value |= (tbit==1) ? CI_CICR1_TBIT : 0;
	value |= rgbt_conv << CI_CICR1_RGBT_CONV_SHIFT;
	value |= rgb_conv << CI_CICR1_RGB_CONV_SHIFT;
	value |= (rgb_f==1) ? CI_CICR1_RBG_F : 0;
	value |= (ycbcr_f==1) ? CI_CICR1_YCBCR_F : 0;
	value |= rgb_bpp << CI_CICR1_RGB_BPP_SHIFT;
	value |= raw_bpp << CI_CICR1_RAW_BPP_SHIFT;
	value |= cspace << CI_CICR1_COLOR_SP_SHIFT;
	WRITE_REG(CICR1, value);

	return;
}

void ci_set_mode(CI_MODE mode, CI_DATA_WIDTH data_width)
{
	volatile unsigned int value;

	/* write mode field in cicr0 */
	value = READ_REG(CICR0);
	value &= ~(CI_CICR0_SIM_SMASK << CI_CICR0_SIM_SHIFT);
	value |= (unsigned int)mode << CI_CICR0_SIM_SHIFT;
	WRITE_REG(CICR0, value);
	
	/* write data width cicr1 */
	value = READ_REG(CICR1);
	value &= ~(CI_CICR1_DW_SMASK << CI_CICR1_DW_SHIFT);
	value |= ((unsigned)data_width) << CI_CICR1_DW_SHIFT;
	WRITE_REG(CICR1, value);
	
	return;
}

void ci_configure_mp(unsigned int PPL, unsigned int LPF, CI_MP_TIMING* timing)
{
	volatile unsigned int value;

	/* write ppl field in cicr1 */
	value = READ_REG(CICR1);
	value &= ~(CI_CICR1_PPL_SMASK << CI_CICR1_PPL_SHIFT);
	value |= (PPL & CI_CICR1_PPL_SMASK) << CI_CICR1_PPL_SHIFT;
	WRITE_REG(CICR1, value);

	/* write BLW, ELW in cicr2 */
	value = READ_REG(CICR2);
	value &= ~((unsigned int)CI_CICR2_BLW_SMASK << CI_CICR2_BLW_SHIFT |
		(unsigned int)CI_CICR2_ELW_SMASK << CI_CICR2_ELW_SHIFT );
	value |= (timing->BLW & CI_CICR2_BLW_SMASK) << CI_CICR2_BLW_SHIFT;
	WRITE_REG(CICR2, value);

	/* write BFW, LPF in cicr3 */
	value = READ_REG(CICR3);
	value &= ~((unsigned int)CI_CICR3_BFW_SMASK << CI_CICR3_BFW_SHIFT |
		(unsigned int)CI_CICR3_LPF_SMASK << CI_CICR3_LPF_SHIFT );
	value |= (timing->BFW & CI_CICR3_BFW_SMASK) << CI_CICR3_BFW_SHIFT;
	value |= (LPF & CI_CICR3_LPF_SMASK) << CI_CICR3_LPF_SHIFT;
	WRITE_REG(CICR3, value);
	return;
}

static unsigned int g_mclk_mhz;
/* hss 104, 156, 208 */
void ci_update_hss(int sys_hss)
{
	if (g_camera_context && (g_camera_context->driver_opened>0)) {
		volatile unsigned int ciclk,  value, div, hss;
		int p;
		unsigned int x;

		hss = sys_hss*100; /*unit: 10KHZ*/
		
		if (cpu_is_pxa310() || cpu_is_pxa930() || cpu_is_pxa935())
			ciclk = hss;
		else
			ciclk = hss/2;

		x = ((ciclk / g_mclk_mhz) - 2);
		p =  x/2;

		if (x%2)
			div = p + 1;
		else
			div = p;
		//printk("sys_hss %d, div %d\n", sys_hss, div);

		/* write cicr4 */
		value = READ_REG(CICR4);
		value &= ~(CI_CICR4_DIV_SMASK<<CI_CICR4_DIV_SHIFT);
		value |= div << CI_CICR4_DIV_SHIFT;
		WRITE_REG(CICR4, value);
	}
}

int ci_active(void)
{
	if (g_camera_context && (g_camera_context->driver_opened>0))
		return 1;
	else 
		return 0;
}

void ci_set_clock(int pclk_enable, int mclk_enable, unsigned int mclk_mhz)
{
	volatile unsigned int ciclk,  value, div, accr_hss, accr_d0cs, hss;
	int p;
	unsigned int x;
	g_mclk_mhz = mclk_mhz;

	/* determine the LCLK frequency programmed into the ACCR */
	accr_hss = (ACSR >> 14) & 0x3;
	accr_d0cs = (ACSR >> 26) & 0x1;

	hss = (accr_d0cs == 1)?60*100:
		(accr_hss == 0x0)?104*100:
		(accr_hss == 0x1)?156*100:
		208*100;/*unit: 10KHZ*/
	
	if (cpu_is_pxa310() || cpu_is_pxa930() || cpu_is_pxa935())
		ciclk = hss;
	else
		ciclk = hss/2;

	x = ((ciclk / mclk_mhz) - 2);
	p =  x/2;

	if (x%2)
		div = p + 1;
	else
		div = p;

	/* write cicr4 */
	value = READ_REG(CICR4);
	value &= ~(CI_CICR4_PCLK_EN | CI_CICR4_MCLK_EN |
		CI_CICR4_DIV_SMASK<<CI_CICR4_DIV_SHIFT);
	value |= (pclk_enable) ? CI_CICR4_PCLK_EN : 0;
	value |= (mclk_enable) ? CI_CICR4_MCLK_EN : 0;
	value |= div << CI_CICR4_DIV_SHIFT;
	WRITE_REG(CICR4, value);
	
	return;
}

void ci_set_polarity(int pclk_sample_falling,
		int hsync_active_low, int vsync_active_low)
{
	volatile unsigned int value;

	/* write cicr4 */
	value = READ_REG(CICR4);
	value &= ~(CI_CICR4_PCP | CI_CICR4_HSP | CI_CICR4_VSP);
	value |= (pclk_sample_falling)? CI_CICR4_PCP : 0;
	value |= (hsync_active_low) ? CI_CICR4_HSP : 0;
	value |= (vsync_active_low) ? CI_CICR4_VSP : 0;
	WRITE_REG(CICR4, value);
	return;
}

void ci_set_fifo(unsigned int timeout, CI_FIFO_THRESHOLD threshold,
		int fifo1_enable,int fifo2_enable)
{
	volatile unsigned int value;

	/* write citor */
	WRITE_REG(CITOR, timeout);

	/* write cifr0: always enable fifo 0! also reset input fifo */
	value = READ_REG(CIFR0);
	value &= ~(CIFR_FEN0 | CIFR_FEN1 | CIFR_FEN2 | CIFR_RESET_F |
			CI_CIFR_THL_0_SMASK<<CI_CIFR_THL_0_SHIFT);
	value |= (unsigned int)threshold << CI_CIFR_THL_0_SHIFT;
	value |= (fifo1_enable) ? CIFR_FEN1 : 0;
	value |= (fifo2_enable) ? CIFR_FEN2 : 0;
	value |= CIFR_RESET_F | CIFR_FEN0;
	WRITE_REG(CIFR0, value);

	return;
}

void ci_reset_fifo()
{
	volatile unsigned int value;
	value = READ_REG(CIFR0);
	value |= CIFR_RESET_F;
	WRITE_REG(CIFR0, value);
}

void ci_set_interrupt_mask(unsigned int mask)
{
	volatile unsigned int value;

	/* write mask in cicr0 */
	value = READ_REG(CICR0);
	value &= ~CI_CICR0_INTERRUPT_MASK;
	value |= (CICR0_VAL(mask) & CI_CICR0_INTERRUPT_MASK);
	WRITE_REG(CICR0, value);

	/* write mask in cidcsr0 */
	value = READ_REG(CIDCSR0);
	value |= CIDCSR0_VAL(mask);
	WRITE_REG(CIDCSR0, value);

	/* write mask in cidcsr1 */
	value = READ_REG(CIDCSR1);
	value |= CIDCSR1_VAL(mask);
	WRITE_REG(CIDCSR1, value);

	/* write mask in cidcsr2 */
	value = READ_REG(CIDCSR2);
	value |= CIDCSR2_VAL(mask);
	WRITE_REG(CIDCSR2, value);

	/* write mask in cidcsr3 */
	value = READ_REG(CIDCSR3);
	value |= CIDCSR3_VAL(mask);
	WRITE_REG(CIDCSR3, value);

	return;
}

unsigned int ci_get_interrupt_mask()
{
	volatile unsigned int cicr0_val, cidcsr0_val;
	volatile unsigned int  cidcsr1_val, cidcsr2_val, cidcsr3_val;

	/* get mask in cicr0 */
	cicr0_val = READ_REG(CICR0);

	/* get mask in cidcsr0 */
	cidcsr0_val = READ_REG(CIDCSR0);

	/* get mask in cidcsr1 */
	cidcsr1_val = READ_REG(CIDCSR1);

	/* get mask in cidcsr2 */
	cidcsr2_val = READ_REG(CIDCSR2);

	/* get mask in cidcsr3 */
	cidcsr3_val = READ_REG(CIDCSR3);

	return CI_INT_MASK(cicr0_val, cidcsr0_val,
		cidcsr1_val, cidcsr2_val, cidcsr3_val);
}

void ci_clear_interrupt_status(unsigned int status)
{
	volatile unsigned int cidcsr0_val, cidcsr1_val, cidcsr2_val, cidcsr3_val;

	/* write 1 to clear cisr interrupt status */
	WRITE_REG(CISR, CISR_VAL(status));

	/* write 1 to clear cifsr interrupt status */
	WRITE_REG(CIFSR, CIFSR_VAL(status));

	/* write 1 to clear cidcsr0 interrupt status */
	if (CIDCSR0_STATUS_VAL(status)) {	/* if DMA channel stopped */
		WRITE_REG(CIDCSR0, 0);		/* reset DMA channel */
	}

	/* write 1 to clear cidcsr1 interrupt status */
	if (CIDCSR1_STATUS_VAL(status)) {	/* if DMA channel stopped */
		WRITE_REG(CIDCSR1, 0);		/* reset DMA channel */
	}

	/* write 1 to clear cidcsr2 interrupt status */
	if (CIDCSR2_STATUS_VAL(status)) {	/* if DMA channe2 stopped */
		WRITE_REG(CIDCSR2, 0);		/* reset DMA channel */
	}

	/* write 1 to clear cidcsr3 interrupt status */
	if (CIDCSR3_STATUS_VAL(status)) {	/* if DMA channe3 stopped */
		WRITE_REG(CIDCSR3, 0);		/* reset DMA channel */
	}

	/* write 1 to clear Bus error status */
	if (status & (1UL << 28)){
		cidcsr0_val = READ_REG(CIDCSR0);
		cidcsr0_val |= 0x1;
		WRITE_REG(CIDCSR0, cidcsr0_val);

		cidcsr1_val = READ_REG(CIDCSR1);
		cidcsr1_val |= 0x1;
		WRITE_REG(CIDCSR1, cidcsr1_val);

		cidcsr2_val = READ_REG(CIDCSR2);
		cidcsr3_val |= 0x1;
		WRITE_REG(CIDCSR2, cidcsr2_val);

		cidcsr3_val = READ_REG(CIDCSR3);
		cidcsr3_val |= 0x1;
		WRITE_REG(CIDCSR3, cidcsr3_val);
	}
}

unsigned int ci_get_interrupt_status()
{
	volatile unsigned int cisr_val, cifsr_val,cidcsr0_val;
	volatile unsigned int cidcsr1_val, cidcsr2_val, cidcsr3_val;

	
	/* get interrupt status in cisr */
	cisr_val = READ_REG(CISR);

	/* get interrupt status in cifsr */
	cifsr_val = READ_REG(CIFSR);

	/* get interrupt status in cidcsr0 */
	cidcsr0_val = READ_REG(CIDCSR0);

	/* get interrupt status in cidcsr1 */
	cidcsr1_val = READ_REG(CIDCSR1);

	/* get interrupt status in cidcsr2 */
	cidcsr2_val = READ_REG(CIDCSR2);

	/* get interrupt status in cidcsr3 */
	cidcsr3_val = READ_REG(CIDCSR3);

	return CI_INT_STATUS(cisr_val, cifsr_val, cidcsr0_val,
		cidcsr1_val, cidcsr2_val, cidcsr3_val);
}

/*JPEG*/

void ci_jpg_enable(void)
{	
	volatile unsigned int value = 0;

	/* 24 bit 1 : enable JPEG */

	value = READ_REG(CIJPEG);	
	value |= 0x1 << 24;

	/*clear count */
	value &= ~(0x0ffffff);
	
	WRITE_REG(CIJPEG, value);
}

void ci_jpg_disable(void)
{
	volatile unsigned int value = 0;

	/* 24 bit 0 : disable JPEG */
	
	value = READ_REG(CIJPEG);
	value &= ~(0x1 << 24);

	/*clear count */
	value &= ~(0x0ffffff);
	
	WRITE_REG(CIJPEG, value);

	
}

/* return 1, error occur; return 0, no error */
unsigned int ci_jpg_error(void)
{
	volatile unsigned int value = 0;

	/* 25 bit,  1: error occur, 0: no error */

	value = READ_REG(CIJPEG);

	if(value & (0x1 << 25)){
		return 1;
	}else{
		return 0;
	}
}
void ci_jpg_cnt_clear(void)
{
	unsigned int cnt = 0;

	/*0 ~ 23 bit is cnt*/

	cnt = READ_REG(CIJPEG);
	cnt &= ~(0x0ffffff);
	WRITE_REG(CIJPEG, cnt);	
}


unsigned int ci_jpg_cnt_get(void)
{
	unsigned int cnt = 0;
	unsigned int val = 0;

	/*0 ~ 23 bit is cnt*/

	val = READ_REG(CIJPEG);
	cnt = val & 0x0ffffff;

	return cnt;
}

unsigned int ci_jpg_cnt_get_dma(int *cnt, int *p)
{
	unsigned int val = 0;

	/*0 ~ 20 bit is cnt, and last four byte must be 0*/

	val = READ_REG(CICMD0);
	*cnt = val & 0x01fffff;

	*p = READ_REG(CIDADR0);
	return 0;
}

/*420 <--> 422*/
int ci_set_ycbcr_420_down_sample (CI_CICR4_YCBCR_DOWN_SAMPLE ycbcr_ds)
{
	volatile unsigned int value;

	/* write cicr4 */
	value = READ_REG(CICR4);
	if(ycbcr_ds == CI_NO_DOWN_SAMPLE){
		value &= ~(CI_CICR4_YCBCR_DS);
	}
	else{
		value |= CI_CICR4_YCBCR_DS;
	}
	WRITE_REG(CICR4, value);

	return 0;
}

/*****************************************************************************
 *	Control APIs
 *****************************************************************************/
void ci_init(void)
{
	/* clear all CI registers */

	/* disable all interrupts */
	WRITE_REG(CICR0, CI_CICR0_INTERRUPT_MASK);
	WRITE_REG(CICR1, 0);
	WRITE_REG(CICR2, 0);
	WRITE_REG(CICR3, 0);
	WRITE_REG(CICR4, 0);
	WRITE_REG(CISR, ~0);
	WRITE_REG(CIFR0,  0);
	WRITE_REG(CITOR, 0);
	WRITE_REG(CIRCD, 0xF0000000);
}

void ci_deinit(void)
{
	return;
}

void ci_enable(void)
{
	volatile unsigned int value;
	
	/* clear branch register */

	WRITE_REG(CIDBR0, 0);
	WRITE_REG(CIDBR1, 0);
	WRITE_REG(CIDBR2, 0);
	WRITE_REG(CIDBR3, 0);
					
	/* write mask in cicr0 */
	value = READ_REG(CICR0);
	value |= CICR0_ENB;

	WRITE_REG(CICR0, value);
	return;
}

void ci_disable_complete()
{
	volatile unsigned int value;

	/* Clear the disable control bit */
	value = READ_REG(CICR0);
	value &= ~CICR0_DIS;
	WRITE_REG( CICR0, value );
}

int ci_disable(int quick, int wait_for_disable_complete )
{
	volatile unsigned int value, mask;
	int retry;


	value = READ_REG(CICR0);
	if(!(value&CICR0_ENB)){
		return 0;
	}

	value = READ_REG(CISR);
	WRITE_REG(CISR, value);

	/* write control bit in cicr0 */
	value = READ_REG(CICR0);
	if (quick) {
		value &= ~CICR0_ENB;
		mask = CI_CISR_CQD;
	}
	else {
		value |= CICR0_DIS;
		mask = CI_CISR_CDD;
	}
	WRITE_REG(CICR0, value);

	if( wait_for_disable_complete ){
		/* wait shutdown complete */
		retry = 50;
		while ( retry-- > 0 ) {
			value = READ_REG(CISR);
			if ( value & mask ) {
				WRITE_REG(CISR, mask);
				return 0;
			}
			msleep(10);
		}
	} else {
		return 0;
	}

	return -1;
}

int ci_psu_tag_bad_pixel(int column, int row)
{
	volatile unsigned int reg_val;

	reg_val = READ_REG(CIPBUF);

	reg_val &= ~((CI_CIPBUF_DEADCOL_SMASK << CI_CIPBUF_DEADCOL_SHIFT) |
			(CI_CIPBUF_DEADROW_SMASK << CI_CIPBUF_DEADROW_SHIFT));

	reg_val |= ((column << CI_CIPBUF_DEADCOL_SHIFT) |
			(row << CI_CIPBUF_DEADROW_SHIFT));

	WRITE_REG(CIPBUF, reg_val);

	return 0;
}

int ci_psu_enable(int enable)
{
	volatile unsigned int reg_val;

	reg_val = READ_REG(CIPSS);

	if (enable){
		reg_val |= CIPSS_PSU_EN;
	}else{
		reg_val &= ~CIPSS_PSU_EN;
	}

	WRITE_REG(CIPSS, reg_val);

	return 0;
}

int ci_cgu_set_black_level(unsigned char black_level)
{
	volatile unsigned int reg_val;

	reg_val = READ_REG(CICCR);

	reg_val &= ~(CI_CICCR_BLC_SMASK << CI_CICCR_BLC_SHIFT);
	reg_val &= ~(CI_CICCR_CLUT_SMASK << CI_CICCR_CLUT_SHIFT);
	reg_val |= (black_level << CI_CICCR_BLC_SHIFT);

	WRITE_REG(CICCR, reg_val);

	return 0;
}

int ci_cgu_set_addr_mux_select(CI_CGU_MUX_SEL_TYPE mux_select)
{
	volatile unsigned int reg_val;

	reg_val = READ_REG(CICCR);

	reg_val &= ~(CI_CICCR_SCALE_SMASK << CI_CICCR_SCALE_SHIFT);
	reg_val |= (mux_select << CI_CICCR_SCALE_SHIFT);

	WRITE_REG(CICCR, reg_val);

	return 0;
}

int ci_cgu_load_lut_ram(
		unsigned int  *histogram_lut_buffer_virtual,
		unsigned int   histogram_lut_buffer_physical,
		unsigned int  *histogram_lut_dma_descriptors_virtual,
		unsigned int   histogram_lut_dma_descriptors_physical,
		unsigned char *lut_ram)
{
	int tries;
	volatile CI_DMAC_DESCRIPTOR_T *dma_desc;
	volatile unsigned int reg_val;
	int i;
	unsigned char *src,*dst;

	/* enable FIFO3 */
	reg_val = READ_REG(CIFR1);
	reg_val |= CI_CIFR_FEN3;
	WRITE_REG(CIFR1, reg_val);

	/* clear CIFSR.EOF3 at first */
	WRITE_REG(CIFSR, CI_CIFSR_EOF3);

	/* memcopy the LUT data to the memory which can be used by DMA */
	src = lut_ram;
	dst = (unsigned char*)histogram_lut_buffer_virtual;
	for (i = 0; i < 192; i++){
		*dst = *src;
		dst++;
		src++;
	}
	for (i=0;i<3;i++) {
		/* configure the DMA for fifo3 */
		dma_desc = (CI_DMAC_DESCRIPTOR_T*)	\
			histogram_lut_dma_descriptors_virtual;
		dma_desc->ddadr = histogram_lut_dma_descriptors_physical;
		dma_desc->dsadr = histogram_lut_buffer_physical + i*64;
		dma_desc->dtadr = ci_get_p_add(CIBR3);
		dma_desc->dcmd  = 64;
		ci_dma_load_descriptor(histogram_lut_dma_descriptors_physical,
			CI_DMA_CHANNEL_3);
		
		/* start loading the LUT ram */
		reg_val = READ_REG(CICCR);
		reg_val &= ~(CI_CICCR_CLUT_SMASK << CI_CICCR_CLUT_SHIFT);
		reg_val |= (CI_CICCR_CLUT_RED << CI_CICCR_CLUT_SHIFT);
		reg_val |= CI_CICCR_LUT_LD;
		WRITE_REG(CICCR, reg_val);


		/* wait for LUT ram loading process completed */
		for (tries = 100; tries >= 0;tries--){
			if ((READ_REG(CICCR) & CI_CICCR_LUT_LD) == 0){
				break;
			}
			msleep(10);

			if (tries == 0){
				return -EBUSY;
			}
		}
	}
	
	/* disable FIFO3 */
	reg_val = READ_REG(CIFR1);
	reg_val &= ~CI_CIFR_FEN3;
	WRITE_REG(CIFR1, reg_val);

	return 0;
}

int ci_cgu_enable(int enable)
{
	volatile unsigned int reg_val;

	reg_val = READ_REG(CICCR);

	if (enable){
		reg_val |= CI_CICCR_EN;
	}else{
		reg_val &= ~CI_CICCR_EN;
	}

	WRITE_REG(CICCR, reg_val);

	return 0;
}

int ci_ssu_set_scale(CI_SSU_SCALE scale)
{
	volatile unsigned int reg_val;

	reg_val = READ_REG(CISSC);

	reg_val &= ~(CI_CISSC_SCALE_SMASK << CI_CISSC_SCALE_SHIFT);
	reg_val |= (scale << CI_CISSC_SCALE_SHIFT);

	WRITE_REG(CISSC, reg_val);

	return 0;
}

int ci_cmu_set_color_correction_coe(CI_CMU_COE_MATRIX *coe_matrix)
{
	volatile unsigned int reg_val;

	/* set k00, k01, k02 to CICMC0 */
	reg_val = READ_REG(CICMC0);
	reg_val &= ~((CI_CICMC0_COF02_SMASK << CI_CICMC0_COF02_SHIFT) |
			(CI_CICMC0_COF01_SMASK << CI_CICMC0_COF01_SHIFT) |
			(CI_CICMC0_COF00_SMASK << CI_CICMC0_COF00_SHIFT));
	reg_val |=  ((coe_matrix->k00 << CI_CICMC0_COF00_SHIFT) |
			(coe_matrix->k01 << CI_CICMC0_COF01_SHIFT) |
			(coe_matrix->k02 << CI_CICMC0_COF02_SHIFT));
	WRITE_REG(CICMC0, reg_val);

	/* set k10, k11, k12 to CICMC1 */
	reg_val = READ_REG(CICMC1);
	reg_val &= ~((CI_CICMC1_COF12_SMASK << CI_CICMC1_COF12_SHIFT) |
			(CI_CICMC1_COF11_SMASK << CI_CICMC1_COF11_SHIFT) |
			(CI_CICMC1_COF10_SMASK << CI_CICMC1_COF10_SHIFT));
	reg_val |=  ((coe_matrix->k10 << CI_CICMC1_COF10_SHIFT) |
			(coe_matrix->k11 << CI_CICMC1_COF11_SHIFT) |
			(coe_matrix->k12 << CI_CICMC1_COF12_SHIFT));
	WRITE_REG(CICMC1, reg_val);

	/* set k20, k21, k22 to CICMC2 */
	reg_val = READ_REG(CICMC2);
	reg_val &= ~((CI_CICMC2_COF22_SMASK << CI_CICMC2_COF22_SHIFT) |
			(CI_CICMC2_COF21_SMASK << CI_CICMC2_COF21_SHIFT) |
			(CI_CICMC2_COF20_SMASK << CI_CICMC2_COF20_SHIFT));
	reg_val |=  ((coe_matrix->k20 << CI_CICMC2_COF20_SHIFT) |
			(coe_matrix->k21 << CI_CICMC2_COF21_SHIFT) |
			(coe_matrix->k22 << CI_CICMC2_COF22_SHIFT));
	WRITE_REG(CICMC2, reg_val);

	return 0;
}

int ci_cmu_enable(CI_CMU_USAGE cmu_usage)
{
	volatile unsigned int reg_val;

	reg_val = READ_REG(CICMR);

	reg_val &= ~(CI_CICMR_DMODE_SMASK << CI_CICMR_DMODE_SHIFT);
	reg_val |= (cmu_usage << CI_CICMR_DMODE_SHIFT);

	WRITE_REG(CICMR, reg_val);

	return 0;
}

/*****************************************************************************
 *	CI dedicated DMAC APIs
 *****************************************************************************/
int ci_dma_load_descriptor(unsigned int dma_desc_phy, CI_DMA_CHANNEL channel)
{
	WRITE_REG( (CIDADR0 + channel*0x10), dma_desc_phy);
	
	return 0;
}

int ci_dma_set_branch (
		unsigned int branch_to_dma_desc_phy,
		int branch_int_enable,
		int branch_after_cur_frame,
		CI_DMA_CHANNEL channel)
{
	volatile unsigned int reg_val;

	/* note: to enable Branch Interrupt, CI_INT_BS should be enabled
	 * as well as branch_int_enable is set as 1
	 */
	reg_val = (branch_to_dma_desc_phy & CI_CIDBR_SRCADDR_SMASK);
	reg_val |= (branch_int_enable == 1)?CI_CIDBR_BINT:0;
	reg_val |= (branch_after_cur_frame == 1)?CI_CIDBR_BRA:0;

	WRITE_REG((CIDBR0+ channel*4), reg_val);
	
	return 0;
}


/***********************************************************************
 *
 * sensor register and chosen
 *
 ***********************************************************************/

/* type is SENSOR_LOW SENSOR_HIGH */
int sensor_register(p_camera_function_t p_fun, int type)
{
	if((type < 0) || (type > SENSOR_MAX)){
		printk("sensor exceed limit\n");
		return -1;
	}

	if(!p_fun){
		printk("function is empty\n");
		return -1;
	}

	if(g_camera_context->sensor_func[type]){
		printk("such type sensor has been registered\n");
		return -1;
	}

	g_camera_context->sensor_func[type] = p_fun;

	return 0;
}

int sensor_unregister(int type)
{
	if((type < 0) || (type > SENSOR_MAX)){
		printk("sensor exceed limit\n");
		return -1;
	}
	
	g_camera_context->sensor_func[type] = NULL;

	return 0;
}

int sensor_choose(p_camera_context_t cam_ctx, int type)
{
	if((type < 0) || (type > SENSOR_MAX)){
		printk("sensor exceed limit\n");
		return -1;
	}

	if(cam_ctx->sensor_func[type] == NULL){
		printk("sensor does not detected \n");
		return -1;
	}

	cam_ctx->camera_functions = cam_ctx->sensor_func[type];

	return 0; 
}

/***********************************************************************
 *
 * FORMAT
 *
 ***********************************************************************/

/* 
 * ALL supported FORMAT, all platform 
 */

static int format_list[] = {
	V4L2_PIX_FMT_SRGGB8,
	V4L2_PIX_FMT_SRGGB10,

	V4L2_PIX_FMT_YUV422P,
	V4L2_PIX_FMT_YUV420,

	V4L2_PIX_FMT_RGB565X,
	V4L2_PIX_FMT_RGB24,

	V4L2_PIX_FMT_JPEG,
	-1	
};
 
#define FMT_CONVERT 0
#define FMT_NONCONVERT 1
#define FMT_ERR -1
static int __format_match(int f_user, int f_sensor)
{
	/*non convert*/
	if(((f_sensor == V4L2_PIX_FMT_RGB565X) && (f_user == V4L2_PIX_FMT_RGB565X))
			||((f_sensor == V4L2_PIX_FMT_SRGGB8) && (f_user == V4L2_PIX_FMT_SRGGB8))
			||((f_sensor == V4L2_PIX_FMT_SRGGB10) && (f_user == V4L2_PIX_FMT_SRGGB10))
			||((f_sensor == V4L2_PIX_FMT_YUV422P) && (f_user == V4L2_PIX_FMT_YUV422P))){

		/*no convert */
		return FMT_NONCONVERT;
	}

	/*convert*/
	if(((f_sensor == V4L2_PIX_FMT_SRGGB8) && (f_user == V4L2_PIX_FMT_YUV422P))
			||((f_sensor == V4L2_PIX_FMT_SRGGB8) && (f_user == V4L2_PIX_FMT_RGB24))
			||((f_sensor == V4L2_PIX_FMT_SRGGB10) && (f_user == V4L2_PIX_FMT_SRGGB8))
			||((f_sensor == V4L2_PIX_FMT_SRGGB10) && (f_user == V4L2_PIX_FMT_RGB24))		
			||((f_sensor == V4L2_PIX_FMT_SRGGB10) && (f_user == V4L2_PIX_FMT_YUV422P))){

		/*convert by QCI */
		return FMT_CONVERT;
	}
		
	if(cpu_is_pxa310() || cpu_is_pxa930() || cpu_is_pxa935()){

		if((f_sensor == V4L2_PIX_FMT_JPEG) && (f_user == V4L2_PIX_FMT_JPEG)){
			/*no convert */
			return FMT_NONCONVERT;
		}
		
		if(((f_sensor == V4L2_PIX_FMT_SRGGB8) && (f_user == V4L2_PIX_FMT_YUV420))
				||((f_sensor == V4L2_PIX_FMT_SRGGB10) && (f_user == V4L2_PIX_FMT_YUV420))
				||((f_sensor == V4L2_PIX_FMT_YUV422P) && (f_user == V4L2_PIX_FMT_YUV420))){

			/*convert by QCI */
			return FMT_CONVERT;
		}
	}

	return FMT_ERR;
}

/* enum all formats supported by sensor */
static int __format_enum(p_camera_context_t cam_ctx, int index, int *format)
{
	int i = 0, j = 0;
	int find = 0;

	if(index >= ARRAY_SIZE(format_list)){
		return -EINVAL;
	}
	if(NULL == cam_ctx->camera_functions){
		printk("choose sensor first!\n");
		return -EINVAL;
	}

	for(j=0; -1 != format_list[j]; j++){
		for(i=0; cam_ctx->camera_functions->format_list[i] != -1; i++){
			if(__format_match(format_list[j], cam_ctx->camera_functions->format_list[i]) != FMT_ERR){
				if(index == find++){
					*format = format_list[j];
					return 0;
				
				}	
				break;
			}
		}
	}
	
	return -EINVAL;

}

/*
* input format is v4l2_fourcc
* output: 1: RAW; 2, RGB; 3: YUV; -1: error
*/

#define FORMAT_RAW 1
#define FORMAT_RGB 2
#define FORMAT_YUV 3
#define FORMAT_JPG 4

static int __format_check(unsigned int format)
{
	if((format == V4L2_PIX_FMT_SRGGB8)
		||(format == V4L2_PIX_FMT_SRGGB10)){
		return FORMAT_RAW;
	}

	if((format == V4L2_PIX_FMT_RGB565X)
		||(format == V4L2_PIX_FMT_RGB24)){
		return FORMAT_RGB;
	}

	if((format == V4L2_PIX_FMT_YUV422P)
		||(format == V4L2_PIX_FMT_YUV420)){
		return FORMAT_YUV;
	}

	if(format == V4L2_PIX_FMT_JPEG){
		return FORMAT_JPG;
	}

	return -1;
		
}

/***********************************************************************
 *
 * Capture APIs
 *
 ***********************************************************************/
#ifdef	SINGLE_DMA_DESC
#define SINGLE_DESCRIPTOR_TRANSFER_MAX		((1 << 21) - 32)
#define SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX  ((1 << 21) - 32)

#else
#define SINGLE_DESCRIPTOR_TRANSFER_MAX  4096
#define SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX  ((1<<21) - 32)

#endif

/* there is a QCI bug that QCI can not branch to next descriptor upon the
 * current frame is done. instead, it will branch to next descriptor once
 * current descriptor is done. 
 * workaround: assert branch only when last descriptor of current frame
 * is loaded */
#define  CI_DMAC_BRANCH_BUG

/***********************************************************************
 *
 * Private functions
 *
 ***********************************************************************/
static void cam_configure_dmac(p_camera_context_t cam_ctx)
{
	int i = 0;
	unsigned int des_physical;
	camera_frame_buffer_queue_t *queue;

	queue = &(cam_ctx->mode[cam_ctx->capture_mode].capture_buffer_queue);

	if (queue->head == NULL)
		return;
	
	for(i=0; i<3; i++){
		if (cam_ctx->fifo_transfer_size[i]) {
			des_physical = queue->head->
				dma_desc_phy_addr[i][FIRST_DMA_DESC_FOR_REAL_BUFFER_INX];
			ci_dma_load_descriptor(des_physical, CI_DMA_CHANNEL_0+i);
		}
	}
}

/* Set the image format */
static int mcam_set_capture_format( p_camera_context_t cam_ctx)
{
	int status;
	unsigned int capture_input_width, capture_input_height;
	unsigned int scale;
	int i = 0;
	unsigned int in_stype = 0;
	unsigned int out_stype = 0;

	/* caculate some parameters according to the driver parameters */

	/* get basic capture format and resolution info */

	for(i=0; i<3; i++){
		cam_ctx->fifo_transfer_size[i] = cam_ctx->mode[cam_ctx->capture_mode].fifo_transfer_size[i];
	}
	cam_ctx->capture_output_width =
		cam_ctx->mode[cam_ctx->capture_mode].width;
	cam_ctx->capture_output_height =
		cam_ctx->mode[cam_ctx->capture_mode].height;

	in_stype = __format_check(cam_ctx->capture_input_format);
	out_stype = __format_check(cam_ctx->capture_output_format);

	/* determine whether to enable PSU */
	cam_ctx->psu_enable = 
		(in_stype == FORMAT_RAW) ? 1 : 0;

	/* determine whether to enable CGC */
	cam_ctx->cgu_enable =
		((in_stype == FORMAT_RAW) &&
		(cam_ctx->capture_input_format !=
		cam_ctx->capture_output_format)) ?
		1 : 0;

	/* determine how to use SSU */
	cam_ctx->ssu_scale = cam_ctx->mode[cam_ctx->capture_mode].capture_scale;

	/* determine how to use CMU */
	if ((in_stype == FORMAT_RAW) && (out_stype == FORMAT_RGB)){			
		cam_ctx->cmu_usage = CI_CMU_OUTPUT_RGB;		
	} else if ((in_stype == FORMAT_RAW) && (out_stype == FORMAT_YUV)){	
		cam_ctx->cmu_usage = CI_CMU_OUTPUT_YUV;		
	} else {
		cam_ctx->cmu_usage = CI_CMU_DISABLE;
	}

	/*determine whether downsample */
	if (cpu_is_pxa310() || cpu_is_pxa930() || cpu_is_pxa935()){
		if(cam_ctx->capture_output_format == V4L2_PIX_FMT_YUV420){
			cam_ctx->ycbcr_ds = CI_YUV_420_DOWN_SAMPLE;
		}else{
			cam_ctx->ycbcr_ds = CI_NO_DOWN_SAMPLE;
		}
	}

	/* caculate the input resolution */
	capture_input_width = cam_ctx->capture_output_width;
	capture_input_height = cam_ctx->capture_output_height;

	if (cam_ctx->ssu_scale != CI_SSU_SCALE_DISABLE) {
		scale = (cam_ctx->ssu_scale == CAMERA_CAPTURE_SCALE_HALF) ?
				2:4;
		capture_input_width = (in_stype == FORMAT_RAW) ?
			(capture_input_width*scale + 2) :
			(capture_input_width*scale + 2);

		capture_input_height = (in_stype == FORMAT_RAW) ?
			(capture_input_height*scale + 2) :
			(capture_input_height*scale + 1);
	}

	if (cam_ctx->cmu_usage != CI_CMU_DISABLE) {
		if (CAMERA_CAPTURE_SCALE_HALF == cam_ctx->ssu_scale) {
			capture_input_height = capture_input_height + 2 + 1;
		} else if (CI_SSU_SCALE_QUARTER == cam_ctx->ssu_scale) {
			capture_input_height = capture_input_height + 4 + 3;
		} else if (CI_SSU_SCALE_DISABLE == cam_ctx->ssu_scale){
			capture_input_height = capture_input_height + 1;
		}
	}

	cam_ctx->capture_input_width = capture_input_width;
	cam_ctx->capture_input_height = capture_input_height;

	/* set sensor setting */
	if((status = cam_ctx->camera_functions->set_capture_format(cam_ctx)) < 0)
		return status;

	/* set CI setting */

	/* set scale to SSU */
	ci_ssu_set_scale(cam_ctx->ssu_scale);

	/* enable/disable CMU */
	ci_cmu_enable(cam_ctx->cmu_usage);

	ci_set_image_format(cam_ctx->capture_input_format, cam_ctx->capture_output_format);

	if (cpu_is_pxa310() || cpu_is_pxa930() || cpu_is_pxa935())
		ci_set_ycbcr_420_down_sample (cam_ctx->ycbcr_ds);

	return 0;
}

/* disable motion video image capture */
static void mcam_stop_video_capture(p_camera_context_t cam_ctx)
{

	/* stop capture */
	cam_ctx->camera_functions->stop_capture(cam_ctx);

	/* Clear the counts and the current status. */
	cam_ctx->ci_disable_complete = 0;
	ci_clear_interrupt_status(0xFFFFFFFF);

	if(V4L2_PIX_FMT_JPEG == cam_ctx->capture_input_format){
		printk("ci_jpg_disable\n");
		ci_jpg_disable();		

		cam_ctx->jpg_offset = 0;
		cam_ctx->jpg_left = 0;
		cam_ctx->jpg_old_count= 0;
		cam_ctx->jpg_new_count= 0;
		cam_ctx->jpg_ci_count = 0;

		cam_ctx->jpg_branch = 0;
		atomic_set(&(cam_ctx->atomic_jpg_full), 0); 
	}

	/* Depending upon the timing of using the command to change to single
	 * frame mode, the OV9640 may issue a VSYNC and 1 HSYNC signal and then
	 * stop.  This would leave the CI in a non-IDLE state.  The camera
	 * interface is now disabled before leaving the stop state.
	 */
	ci_disable(1,1);

	/* clear ci fifo */
	ci_reset_fifo();	
	cam_ctx->dma_running = 0;

	return;
}


/***********************************************************************
 *
 * Flow Control APIs
 *
 ***********************************************************************/

/* Add the buffer into the buffer pool
 * and generate the DMA chains for the buffer
 */

/* Load JPEG queue head, for restart */
static int cam_load_jpg_head(p_camera_context_t cam_ctx)
{
	camera_frame_buffer_info_t  *buffer_info;
	unsigned int des_physical;
	
	int id = cam_ctx->jpg_buf_node->buf_id;

	buffer_info = &(cam_ctx->mode[cam_ctx->capture_mode].master_frame_buffer_list[id]);

	des_physical = buffer_info->dma_desc_phy_addr[0]
			[FIRST_DMA_DESC_FOR_REAL_BUFFER_INX];

	ci_dma_load_descriptor(des_physical, CI_DMA_CHANNEL_0);
	
	return 0;
}

/* branch to queue head */
static int cam_branch_to_head(p_camera_context_t cam_ctx)
{
	camera_frame_buffer_queue_t *queue;
	unsigned int tf_size[3] = {0};
	int i = 0;
	volatile unsigned int cidadr0 = ci_get(CIDADR0);

	/* get the specified capture queue and other info */
	queue = &(cam_ctx->mode[cam_ctx->capture_mode].capture_buffer_queue);

#ifdef CI_DMAC_BRANCH_BUG
	if (queue->tail->dma_desc_phy_addr[0]
				[FIRST_DMA_DESC_FOR_PHANTOM_BUFFER_INX]
				!= cidadr0){
		return -1;
	}
#endif

	for(i=0; i<3; i++){
		tf_size[i] = cam_ctx->fifo_transfer_size[i];

		if(tf_size[i]){
			ci_dma_set_branch(queue->head->dma_desc_phy_addr[i]
				[FIRST_DMA_DESC_FOR_REAL_BUFFER_INX],
				1,
				1,
				CI_DMA_CHANNEL_0+i);
		}
	}
	cam_ctx->branch_flag = 1;
	return 0;
}

static void ci_resume(p_camera_context_t cam_ctx)
{
	/*counting caller to ensure para correct */
	
	/* clear ci fifo */
	ci_reset_fifo();
	ci_clear_interrupt_status(0xFFFFFFFF);

	if(V4L2_PIX_FMT_JPEG == cam_ctx->capture_input_format){

		unsigned char * vaddr = NULL;
		unsigned int fifo0_transfer_size  = PAGE_ALIGN(cam_ctx->fifo_transfer_size[0]);	

		vaddr = cam_ctx->jpg_buf_node->vaddr;
		
		if(0 != cam_ctx->jpg_buf_node->size)
			dma_cache_maint(vaddr, fifo0_transfer_size, DMA_FROM_DEVICE);

		/*using working buffer */

		cam_load_jpg_head(cam_ctx);
		ci_jpg_enable();	
		
	}else{
		cam_configure_dmac(cam_ctx);
	}
	
	if(cam_ctx->irq > 0)
		enable_irq(cam_ctx->irq);
}


/* disable motion video image capture */
static void ci_pause(p_camera_context_t cam_ctx)
{	
	if(V4L2_PIX_FMT_JPEG == cam_ctx->capture_input_format){
		ci_jpg_disable();		
	}

	ci_disable(1,1);

	ci_clear_interrupt_status(0xFFFFFFFF);

	if(cam_ctx->platform_ops->sync_to_gpio)
		cam_ctx->platform_ops->sync_to_gpio();	

	return;
}

static int mcam_submit_jpeg_buffer(p_camera_context_t cam_ctx, int buffer_id)
{
	camera_frame_buffer_info_t  *buffer_info =
		&(cam_ctx->mode[cam_ctx->capture_mode].master_frame_buffer_list[buffer_id]);

	/* get the specified capture queue and buffer and other info */
	camera_frame_buffer_queue_t *queue =
		&(cam_ctx->mode[cam_ctx->capture_mode].capture_buffer_queue);

	atomic_inc(&(cam_ctx->atomic_jpg_full));
	if((cam_ctx->jpg_buf_num == atomic_read(&(cam_ctx->atomic_jpg_full))) 
			&& (cam_ctx->jpg_branch)){

		unsigned int fifo0_transfer_size;
		unsigned char * vaddr = NULL;

		cam_ctx->jpg_branch = 0;
		cam_ctx->jpg_offset = 0;
		cam_ctx->jpg_left = 0;

		cam_ctx->jpg_new_count = 0;
		cam_ctx->jpg_old_count = 0;
		cam_ctx->jpg_ci_count = 0;

		fifo0_transfer_size =  PAGE_ALIGN(cam_ctx->fifo_transfer_size[0]);	

		/*using working buffer */

		vaddr = cam_ctx->jpg_buf_node->vaddr;

		memset(vaddr, 0, fifo0_transfer_size);

		/* restart */
		ci_resume(cam_ctx);
	}

	if(queue->tail != NULL){

		/* update the head of the queue, if necessary */
		if (queue->head == NULL){
			queue->head = buffer_info;
		}

		/* update the tail of the queue */
		if (queue->tail == NULL){
			queue->tail = buffer_info;
		} else {
			queue->tail->next_buffer = buffer_info;
			queue->tail = buffer_info;
		}
		queue->tail->next_buffer = NULL;
		
		return 0;
	}

	buffer_info->dma_desc_vir_addr[0]
		[LAST_DMA_DESC_FOR_REAL_BUFFER_INX]->ddadr =
		buffer_info->dma_desc_phy_addr[0]
		[FIRST_DMA_DESC_FOR_PHANTOM_BUFFER_INX];

	buffer_info->dma_desc_vir_addr[0]
		[LAST_DMA_DESC_FOR_PHANTOM_BUFFER_INX]->ddadr =
		buffer_info->dma_desc_phy_addr[0]
		[FIRST_DMA_DESC_FOR_PHANTOM_BUFFER_INX];

	/* update the head of the queue, if necessary */
	if (queue->head == NULL){
		queue->head = buffer_info;
	}

	/* update the tail of the queue */
	if (queue->tail == NULL){
		queue->tail = buffer_info;
	} else {
		queue->tail->next_buffer = buffer_info;
		queue->tail = buffer_info;
	}
	queue->tail->next_buffer = NULL;
	return 0;
}

static int mcam_submit_preview_buffer(p_camera_context_t cam_ctx, int buffer_id)
{
	camera_frame_buffer_info_t  *buffer_info;
	camera_frame_buffer_queue_t *queue;
	unsigned int tf_size[3] = {0};
	unsigned int need_branch = 0;
	int i = 0;

	buffer_info = &(cam_ctx->mode[cam_ctx->capture_mode].master_frame_buffer_list[buffer_id]);
	
	/* get the specified capture queue and buffer and other info */
	queue = &(cam_ctx->mode[cam_ctx->capture_mode].capture_buffer_queue);

	for(i=0; i<3; i++){
		tf_size[i] = cam_ctx->mode[cam_ctx->capture_mode].fifo_transfer_size[i];
	}

	/* ensure the tail's phantom buffer loop to itself */
	for(i=0; i<3; i++){
		if(tf_size[i]){
			buffer_info->dma_desc_vir_addr[i]
				[LAST_DMA_DESC_FOR_REAL_BUFFER_INX]->ddadr =
				buffer_info->dma_desc_phy_addr[i]
				[FIRST_DMA_DESC_FOR_PHANTOM_BUFFER_INX];

			buffer_info->dma_desc_vir_addr[i]
				[LAST_DMA_DESC_FOR_PHANTOM_BUFFER_INX]->ddadr =
				buffer_info->dma_desc_phy_addr[i]
				[FIRST_DMA_DESC_FOR_PHANTOM_BUFFER_INX];
		}
	}		

#ifndef	DMA_OVERFLOW_WORKAROUND
	/* attach the submitted buffer's DMA chain to the queue */
	if ((queue->tail != NULL) && (queue->tail != buffer_info)){

		for(i=0; i<3; i++){			
			if (tf_size[i]){
				queue->tail->dma_desc_vir_addr[i]
					[LAST_DMA_DESC_FOR_REAL_BUFFER_INX]->ddadr =
					buffer_info->dma_desc_phy_addr[i]
					[FIRST_DMA_DESC_FOR_REAL_BUFFER_INX];

				queue->tail->dma_desc_vir_addr[i]
					[LAST_DMA_DESC_FOR_PHANTOM_BUFFER_INX]->ddadr =
					buffer_info->dma_desc_phy_addr[i]
					[FIRST_DMA_DESC_FOR_REAL_BUFFER_INX];
			}
		}		

	} else if ((cam_ctx->dma_running) && (queue->tail == buffer_info)){
		need_branch = 1;
	}
#endif

	/* update the head of the queue, if necessary */
	if (queue->head == NULL){
		queue->head = buffer_info;
	}

	/* update the tail of the queue */
	if (queue->tail == NULL){
		queue->tail = buffer_info;
	} else {
		queue->tail->next_buffer = buffer_info;
		queue->tail = buffer_info;
	}
	queue->tail->next_buffer = NULL;

	
	/* tag the buffer as clean */
	*((unsigned int*)buffer_info->pY + tf_size[0]/4 - 2) =
		CAMERA_CLEAN_BUFFER_IDENTIFIER;
	*((unsigned int*)buffer_info->pY + tf_size[0]/4 - 1) =
		CAMERA_CLEAN_BUFFER_IDENTIFIER;

/* If has DMA OVERFLOW issue, we need reload the DMA in CDD interrupt */
#ifndef	DMA_OVERFLOW_WORKAROUND
	/* reload the dma descriptor if needed */
	if (need_branch){
		cam_branch_to_head(cam_ctx);
	}
#endif
	return 0;
}

/* Submit a buffer into the capture queue */
static int mcam_submit_buffer(p_camera_context_t  cam_ctx, int  buffer_id)
{
	if (V4L2_PIX_FMT_JPEG == cam_ctx->capture_input_format)
		return mcam_submit_jpeg_buffer(cam_ctx, buffer_id);
	else
		return mcam_submit_preview_buffer(cam_ctx, buffer_id);
}

/* detect if the queue is in dead lock */
static int queue_dead_locked(p_camera_context_t cam_ctx)
{
	camera_frame_buffer_queue_t *queue;
	volatile unsigned int cidadr0 = ci_get(CIDADR0);
	volatile unsigned int citadr0 = ci_get(CITADR0);
	unsigned int fifo0_transfer_size;

	/* get the specified capture queue and other info */
	queue = &(cam_ctx->mode[cam_ctx->capture_mode].capture_buffer_queue);

	fifo0_transfer_size =  cam_ctx->fifo_transfer_size[0];

	if (queue->head == NULL){
		return 0;
	}

	if ((*((unsigned int*)queue->head->pY + fifo0_transfer_size/4 - 2)
		       	!= CAMERA_CLEAN_BUFFER_IDENTIFIER)
			&& (*((unsigned int*)queue->head->pY + fifo0_transfer_size/4 - 1)
			 != CAMERA_CLEAN_BUFFER_IDENTIFIER)){
		return 0;

	}else {
		return ((citadr0 == cam_ctx->phantom_buffer_physical)
			&& (queue->tail->dma_desc_phy_addr[0]					
			[FIRST_DMA_DESC_FOR_PHANTOM_BUFFER_INX] <= cidadr0)
			&& (cidadr0 <= queue->tail->dma_desc_phy_addr[0]
			[LAST_DMA_DESC_FOR_PHANTOM_BUFFER_INX]));
	}
}
 
/* Get the buffer filled with valid frame data */
static int mcam_get_filled_buffer( p_camera_context_t  cam_ctx,
		int *buffer_id)
{
	camera_frame_buffer_queue_t *queue;
	unsigned int fifo0_transfer_size;
	int ret = 0;
	
	*buffer_id = -1;

	/* get the specified capture queue and other info */
	queue = &(cam_ctx->mode[cam_ctx->capture_mode].capture_buffer_queue);

	fifo0_transfer_size =  cam_ctx->fifo_transfer_size[0];

	/* check if there are any buffers filled with valid image data */
	if (queue->head == NULL) {
		return -EIO;
	} else {
		if ((*((unsigned int*)queue->head->pY + fifo0_transfer_size/4 - 2)
		       	!= CAMERA_CLEAN_BUFFER_IDENTIFIER)
			&& (*((unsigned int*)queue->head->pY + fifo0_transfer_size/4 - 1)
			 != CAMERA_CLEAN_BUFFER_IDENTIFIER)){
			*buffer_id  = queue->head->frame_id;
			queue->head = queue->head->next_buffer;
			
			if(cam_ctx->branch_flag){				
				cam_ctx->branch_flag = 0;
			}
			return 0;
		}
		
		if(cam_ctx->branch_flag){
			return -EIO;
		}
		
		if ((ret = queue_dead_locked(cam_ctx)))
			cam_branch_to_head(cam_ctx);
		
		return -EIO;
	}
}

static int camera_get_sensor_format(unsigned int user_format)
{
	p_camera_context_t cam_ctx = g_camera_context;	
	int i = 0;

	/* try non_convert first */
	for(i=0; cam_ctx->camera_functions->format_list[i] != -1; i++){
		if(__format_match(user_format, cam_ctx->camera_functions->format_list[i]) == FMT_NONCONVERT)
		return cam_ctx->camera_functions->format_list[i];
	}

	/* try QCI convert second */
	for(i=0; cam_ctx->camera_functions->format_list[i] != -1; i++){
		if(__format_match(user_format, cam_ctx->camera_functions->format_list[i]) == FMT_CONVERT)
		return cam_ctx->camera_functions->format_list[i];
	}

	return -1;
}

static struct buf_node *camera_get_buffer_from_id(int buf_id)
{
	struct list_head *pos;
	struct buf_node *buf_node;

	list_for_each(pos, g_camera_context->p_buf_head) {
		buf_node = list_entry(pos, struct buf_node, buf_head);
		if (buf_node->buf_id == buf_id)
			goto found;
	}
	return NULL;

found:
	return buf_node;
}

static struct buf_node *camera_get_buffer_from_index(int buf_index)
{
	struct list_head *pos;
	struct buf_node *buf_node;

	list_for_each(pos, g_camera_context->p_buf_head) {
		buf_node = list_entry(pos, struct buf_node, buf_head);
		if (buf_node->buf_index == buf_index)
			goto found;
	}
	return NULL;

found:
	return buf_node;
}

static int camera_get_buffer_num(void)
{
	struct list_head *pos;
	int buf_num = 0;

	list_for_each(pos, g_camera_context->p_buf_head) {
		buf_num++;
	}
	return buf_num;
}

static ssize_t camera_get_buffer_size(void)
{
	struct buf_node *buf_node;
	ssize_t page_num;

	if (!list_empty(g_camera_context->p_buf_head)) {
		buf_node =
			list_entry(g_camera_context->p_buf_head->next,
					struct buf_node, buf_head);
		page_num = buf_node->page_num;
	} else {
		page_num = 0;
	}

	return page_num * PAGE_SIZE;
}

static struct page *camera_alloc_page(void)
{
	unsigned long flags;
	struct page *page = NULL;

	if (!list_empty(&pc_head.page_list)) {
		spin_lock_irqsave(&pc_head.lock, flags);
		page = list_entry(pc_head.page_list.next, struct page, lru);
		list_del(&page->lru);
		pc_head.page_count--;
		spin_unlock_irqrestore(&pc_head.lock, flags);
	} else {
		page = alloc_page(GFP_KERNEL);
	}

	return page;
}

/* camera_free_page()
 *
 * Free page.
 * Param:
 *    page:    the page will be freed
 *    limit:
 *        0: The page will be added to camera page cache list.
 *           This will be very useful when app change capture mode
 *           and capture resolution dynamically. We needn't free all
 *           of the old pages and alloc new pages for new catpure
 *           mode/resolution. Just need alloc/free the delta pages.
 *        1: If the number of camera page cache list is lager than
 *           MAX_PAGE_CACHE, the page will be free using __free_page.
 *           Else the page will be added to page cache list.
 *
 */
static void camera_free_page(struct page *page, int limit)
{
	unsigned long flags;

	if (0 == limit) {
		spin_lock_irqsave(&pc_head.lock, flags);
		list_add_tail(&page->lru, &pc_head.page_list);
		pc_head.page_count++;
		spin_unlock_irqrestore(&pc_head.lock, flags);
	} else {
		if (pc_head.page_count < MAX_PAGE_CACHE) {
			spin_lock_irqsave(&pc_head.lock, flags);
			list_add_tail(&page->lru, &pc_head.page_list);
			pc_head.page_count++;
			spin_unlock_irqrestore(&pc_head.lock, flags);
		} else {
			atomic_set(&page->_count, 1);
			ClearPageReserved(page);
			__free_page(page);
		}
	}
}

static void camera_free_buffer_node(struct buf_node *buf_node, int limit)
{
	int i;
	struct page *page = NULL;

	/*
	 * vunmap will do TLB flush for us.
	 * We map uncachable memory, so needn't cache invalid operation here.
	 */
	vunmap(buf_node->vaddr);
	buf_node->vaddr = NULL;
	
	if (buf_node->io_type == V4L2_MEMORY_USERPTR)
		goto done;


#ifndef	DMA_OVERFLOW_WORKAROUND
	for (i = 0; i < buf_node->page_num; i++) {
		page = buf_node->pages[i];
		camera_free_page(page, limit);
	}
#endif
	
	if(buf_node->dma_desc_size > 0){
		dma_free_coherent(NULL, buf_node->dma_desc_size,
				buf_node->dma_desc_vaddr,
				buf_node->dma_desc_paddr);
		
		buf_node->dma_desc_size = 0;
	}
	
done:
	kfree(buf_node->pages);
	kfree(buf_node);
}

static void camera_free_buffer_list(int capture_mode, int limit)
{
	struct buf_node *buf_node;
	unsigned long flags;
	struct list_head *p_buf_head;
	struct list_head *p_report_head;
	p_camera_context_t cam_ctx = g_camera_context;

	p_buf_head = &(cam_ctx->mode[capture_mode].buf_head);
	p_report_head = &(cam_ctx->mode[capture_mode].report_head);

	down_interruptible(&buf_list_sem);
	
	while (!list_empty(p_buf_head)) {
		buf_node =
			list_entry(p_buf_head->next, struct buf_node, buf_head);
		list_del_init(p_buf_head->next);

		
		
		if(buf_node->size > 0){
			camera_free_buffer_node(buf_node, limit);
		}else{
		
			if(buf_node->dma_desc_size > 0){
				dma_free_coherent(NULL, buf_node->dma_desc_size,
						buf_node->dma_desc_vaddr,
						buf_node->dma_desc_paddr);
			}
		
			kfree(buf_node->pages);
			kfree(buf_node);
		}
	}
	up(&buf_list_sem);

	/* empty the report list */
	spin_lock_irqsave(&report_list_lock, flags);
	while (!list_empty(p_report_head)) {
		list_del_init(p_report_head->next);
	}
	spin_unlock_irqrestore(&report_list_lock, flags);

//#ifdef	DMA_OVERFLOW_WORKAROUND		
#ifdef	RESERVE_DMA_BUFFER	//should be RESERVE_DMA_BUFFER rather than DMA_OVERFLOW_WORKAROUND
	g_dma_free_start = page_address(g_dma_page);
	g_dma_free_size = PRE_DMA_BUF_SIZE;
	g_dma_freepage_start = g_dma_page;

	g_dma_jpeg_page = g_dma_page + JPG_DMA_BUF_SIZE / PAGE_SIZE;
	g_dma_jpeg_size = JPG_DMA_BUF_SIZE;
#endif

}

static unsigned long uva_to_pa(unsigned long addr, struct page **page)
{
	unsigned long ret = 0UL;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;

	pgd = pgd_offset(current->mm, addr);
	if (!pgd_none(*pgd)) {
		pud = pud_offset(pgd, addr);
		if (!pud_none(*pud)) {
			pmd = pmd_offset(pud, addr);
			if (!pmd_none(*pmd)) {
				pte = pte_offset_map(pmd, addr);
				if (!pte_none(*pte) && pte_present(*pte)) {
					(*page) = pte_page(*pte);
					ret = page_to_phys(*page);
					ret |= (addr & (PAGE_SIZE-1));
				}
			}
		}
	}
	return ret;
}

static void camera_compute_fifo_size(p_camera_context_t cam_ctx,
		int *f_0, int *f_1, int *f_2)
{	
	unsigned int format = 0;
	unsigned int width = 0;
	unsigned int height = 0;
	unsigned int size = 0;

	format = cam_ctx->mode[cam_ctx->capture_mode].output_format;
	width = cam_ctx->mode[cam_ctx->capture_mode].width;
	height = cam_ctx->mode[cam_ctx->capture_mode].height;

	switch (format) {
		case V4L2_PIX_FMT_SRGGB10:
			size = width * height * 2;
			*f_0 = size;
			*f_1 = 0;
			*f_2 = 0;
			break;

		case V4L2_PIX_FMT_SRGGB8:			
			size = width * height;
			*f_0 = size;
			*f_1 = 0;
			*f_2 = 0;
			break;
			
		case V4L2_PIX_FMT_JPEG:
			/* If in JPEG mode, we don't care the buffer number
			 * from user application. We just use one big buffer
			 * for JPEG image.
			 */
			/* size = width * height * cam_ctx->jpg_buf_num; */
			size = JPG_DMA_BUF_SIZE;
			*f_0 = size;
			*f_1 = 0;
			*f_2 = 0;
			break;
			
		case V4L2_PIX_FMT_RGB24:
			size = width * height * 3;
			*f_0 = size;
			*f_1 = 0;
			*f_2 = 0;
			break;
			
		case V4L2_PIX_FMT_RGB565X:
			size = width * height * 2;
			*f_0 = size;
			*f_1 = 0;
			*f_2 = 0;
			break;
	
		case V4L2_PIX_FMT_YUV422P:
			size = width * height * 2;
			*f_0 = size / 2;
			*f_1 = size / 4;
			*f_2 = size / 4;
			break;
			
		
		case V4L2_PIX_FMT_YUV420:
			if(cpu_is_pxa310() || cpu_is_pxa930() || cpu_is_pxa935()){
				size = width * height * 2;
				*f_0 = size / 2;
				*f_1 = size / 8;
				*f_2 = size / 8;
			}
			break;
		default:
			break;
	}
	return;
}

static int camera_alloc_buffer_node(p_camera_context_t cam_ctx, struct buf_node **buf_node, unsigned long userptr, 
	int f_0, int f_1, int f_2, int dma_desc_size)
{
	int page_num;
	int i, j;
	int id = 0;
	unsigned int ret = 0;
	struct page *page = NULL;
	struct buf_node *buf;
	unsigned int vaddr = PAGE_ALIGN(userptr);
	
	void *dma_desc_virt = NULL;
	dma_addr_t dma_desc_phy = 0;
	unsigned long flags;

	int size = f_0 + f_1 + f_2;

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf) {
		printk("Not enough memory\n");
		return -ENOMEM;
	}

	page_num = PAGE_ALIGN(size) / PAGE_SIZE;
	buf->pages =
		(struct page **)kzalloc(page_num * sizeof(long), GFP_KERNEL);
	if (!buf->pages) {
		printk("Not enough memory\n");
		ret = -ENOMEM;
		goto alloc_node_error;
	}

	if (!userptr) {
#ifdef	RESERVE_DMA_BUFFER
		if (g_dma_free_size < size) {
			return -ENOMEM;
		}

		if(V4L2_PIX_FMT_JPEG == cam_ctx->capture_input_format) {
			page = g_dma_jpeg_page;
		} else {
			page = g_dma_freepage_start;

			spin_lock_irqsave(&dma_buf_lock, flags);
			g_dma_free_start += page_num << PAGE_SHIFT;
			g_dma_free_size -= page_num << PAGE_SHIFT;
			g_dma_freepage_start += page_num;
			spin_unlock_irqrestore(&dma_buf_lock, flags);
		}

		buf->paddr = __pa(page_address(page));

		for (i = 0; i < page_num; i++) {
			if (!page) {
				printk("Not enough memory\n");
				ret = -ENOMEM;

				if(!(V4L2_PIX_FMT_JPEG == cam_ctx->capture_input_format)) {
					spin_lock_irqsave(&dma_buf_lock, flags);
					g_dma_free_start -= page_num << PAGE_SHIFT;
					g_dma_free_size += page_num << PAGE_SHIFT;
					g_dma_freepage_start -= page_num;
					spin_unlock_irqrestore(&dma_buf_lock, flags);
				}

				goto alloc_bufpage_err;
			}
			atomic_set(&page->_count, 1);
			SetPageReserved(page);
			buf->pages[i] = page;
			page++;
		}
#else
#ifdef DMA_OVERFLOW_WORKAROUND
		printk("DMA_OVERFLOW_WORKAROUND defined while RESERVE_DMA_BUFFER undefined is NOT supported in memory map mode!\n");
		ret = -ENOMEM;
		goto alloc_bufpage_err;
#endif	
		for (i = 0; i < page_num; i++) {
			page = camera_alloc_page();
			if (!page) {
				printk("Not enough memory\n");
				ret = -ENOMEM;
				goto alloc_pages_error;
			}
			atomic_set(&page->_count, 1);
			SetPageReserved(page);
			buf->pages[i] = page;
		}
#endif
	} else {
#ifdef CI_USERPOINTER
		static unsigned long 	page_first_address, page_num_address;
		page_first_address = uva_to_pa(vaddr, &page);
		buf->paddr = __pa(page_address(page));

		for (i = 0; i < page_num; i++) {
			if (vaddr){
				page_num_address = uva_to_pa(vaddr, &page);
				if ((page_num_address - page_first_address) != (PAGE_SIZE * i)) {
					printk("the malloced memory is not continuous!\n");
					ret = -ENOMEM;
					goto alloc_bufpage_err;
				}
				vaddr += PAGE_SIZE;
			}
			buf->pages[i] = page;
		}
#else
#ifdef DMA_OVERFLOW_WORKAROUND
                printk("DMA_OVERFLOW_WORKAROUND defined while CI_USERPOINTER undefined is NOT supported in user pointer mode!\n");
                ret = -ENOMEM;
                goto alloc_bufpage_err;
#endif
		for (i = 0; i < page_num; i++) {
			if (vaddr){
				uva_to_pa(vaddr, &page);
				vaddr += PAGE_SIZE;
			}
			buf->pages[i] = page;
		}
#endif
	}
	buf->page_num = page_num;
	buf->size = page_num * PAGE_SIZE;
	/* buf->buf_id = -1; */
	buf->vaddr =
		vmap(buf->pages, buf->page_num, VM_MAP,
				pgprot_noncached(pgprot_kernel));

	memset(buf->vaddr, 0, buf->size);

	/* check if the memory map is OK. */
	if (!buf->vaddr) {
		printk("vmap() failure\n");
		ret = -EFAULT;
		goto vmap_error;
	}
	id = camera_get_buffer_num();

	if(id >= MAX_CAMERA_FRAME_BUFFERS){
		printk(KERN_ERR "cam buffer node exceed max number :%d\n", MAX_CAMERA_FRAME_BUFFERS);
		goto vmap_error;
	}
	buf->buf_index = id;
	buf->buf_id = id;

	dma_desc_virt = dma_alloc_coherent(NULL, dma_desc_size,
		&dma_desc_phy, GFP_KERNEL);
	if (!dma_desc_virt) {
		dbg("Just alloc %d number buffer node\n", camera_get_buffer_num() - 1);
		goto vmap_error;
	}

	buf->fifo0_size = f_0;
	buf->fifo1_size = f_1;
	buf->fifo2_size = f_2;
	
	buf->io_type = userptr?V4L2_MEMORY_USERPTR : V4L2_MEMORY_MMAP;
	buf->dma_desc_vaddr = dma_desc_virt;
	buf->dma_desc_paddr = dma_desc_phy;
	buf->dma_desc_size = dma_desc_size;

	*buf_node = buf;

	return ret;

vmap_error:
alloc_pages_error:
#ifndef	RESERVE_DMA_BUFFER
	for (j = 0; j < i; j++) {
		page = buf->pages[j];
		if (page)
			camera_free_page(page, 1);
	}
#endif
alloc_bufpage_err:
	kfree(buf->pages);
alloc_node_error:
	kfree(buf);
	return ret;
}

static int camera_alloc_empty_buffer_node(p_camera_context_t cam_ctx, struct buf_node *buf_src)
{

	int i = 0;
	int ret = -1;
	int buffer_index = 0;
	struct buf_node *buf;
	camera_frame_buffer_info_t *buffer_info;	

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf) {
		printk("Not enough memory\n");
		return -ENOMEM;
	}

	buffer_index  = camera_get_buffer_num();
	if(buffer_index >= MAX_CAMERA_FRAME_BUFFERS){
		printk(KERN_ERR "cam buffer node exceed max number :%d\n", MAX_CAMERA_FRAME_BUFFERS);
		goto alloc_node_error;
	}
	
	buf->buf_index = buffer_index;
	buf->page_num = buf_src->page_num;

	buf->pages =
		(struct page **)kzalloc(buf->page_num * sizeof(long), GFP_KERNEL);
	if (!buf->pages) {
		dbg("Not enough memory\n");
		ret = -ENOMEM;
		goto alloc_node_error;
	}

	for (i = 0; i < buf->page_num; i++) {
		buf->pages[i] = buf_src->pages[i];
	}

	/* notify this buffer is empty buffer */
	buf->buf_id = buffer_index;
	buf->size = 0;		
	buf->dma_desc_size = 0;

	list_add_tail(&buf->buf_head, g_camera_context->p_buf_head);
	buffer_info = &(cam_ctx->mode[cam_ctx->capture_mode].master_frame_buffer_list[buffer_index]);

	buffer_info->frame_id = buffer_index;
	buffer_info->buffer_size = 0;

	return 0;

alloc_node_error:
	kfree(buf);
	return ret;
}

static int camera_prepare_buffer(p_camera_context_t cam_ctx, unsigned long userptr, 
		unsigned int buf_len, unsigned int *buf_index)
{
	int ret = 0;
	int buf_size, dma_desc_size;

	struct buf_node *buf_node = NULL;
	unsigned long flags;

	int tf_size[3] = {0};
	unsigned int phan_tf_size[3] = {0};
	unsigned int    num_descriptors_for_buffer = 0;

	camera_frame_buffer_info_t *buffer_info;
	CI_DMAC_DESCRIPTOR_T *cur_des_virtual;
	int buffer_index = 0;
	unsigned int buffer_page_seq = 0; 
	unsigned int dma_descriptor_seq = 0;
	int left = 0;
	int i = 0;
	int f_size = 0;

	int remain_size = 0;
	int des_transfer_size = 0;
	int offset = 0;
	int num_des = 0;	

	camera_compute_fifo_size(cam_ctx, &tf_size[0], &tf_size[1], &tf_size[2]);	

	buf_size = tf_size[0] + tf_size[1] + tf_size[2];

	if (userptr && buf_len < buf_size)
		return -EFAULT;

	for(i=0; i<3; i++){
		cam_ctx->mode[cam_ctx->capture_mode].fifo_transfer_size[i] = tf_size[i];		
	}

	for(i=0; i<3; i++){
		phan_tf_size[i] = (tf_size[i] +
			SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX - 1) /
			SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX;
	}

	num_descriptors_for_buffer = (tf_size[0] + tf_size[1] + tf_size[2] +
			SINGLE_DESCRIPTOR_TRANSFER_MAX - 1) / SINGLE_DESCRIPTOR_TRANSFER_MAX + 2;


	/* descriptors memory for real frame buffer and phantom buffer */
	dma_desc_size = sizeof(CI_DMAC_DESCRIPTOR_T)*(
		num_descriptors_for_buffer 
		+ phan_tf_size[0] + phan_tf_size[1] + phan_tf_size[2]);
	
	ret = camera_alloc_buffer_node(cam_ctx, &buf_node, userptr, tf_size[0], tf_size[1], tf_size[2], dma_desc_size);
	if (ret) {
		dbg("Alloc %dth buffer node failure\n", camera_get_buffer_num());		
		goto exit;
	}		

	spin_lock_irqsave(&cam_queue_lock, flags);	

	/* add the buffer into the buffer pool */
	buffer_index = (buf_node->buf_id == -1) ?
				cam_ctx->frame_buffer_number:buf_node->buf_id;
	buffer_info = &(cam_ctx->mode[cam_ctx->capture_mode].master_frame_buffer_list[buffer_index]);

	
	buffer_info->frame_id = buffer_index;
	buffer_info->buffer_vir_addr = (volatile u32*)buf_node->vaddr;
	buffer_info->buffer_size = buf_size;
	buffer_info->dma_descriptors_virtual =
		(CI_DMAC_DESCRIPTOR_T*)buf_node->dma_desc_vaddr;
	
	if (buf_node->buf_id == -1)	{
		cam_ctx->frame_buffer_number++;
		if (cam_ctx->frame_buffer_number >= MAX_CAMERA_FRAME_BUFFERS){
			printk(KERN_INFO "camera: frame buffer number exceed max buffers\n");
			cam_ctx->frame_buffer_number = 0;
		}
	}

	/* generate the DMA chains */
	cur_des_virtual = (CI_DMAC_DESCRIPTOR_T*)buf_node->dma_desc_vaddr;

	for(i=0; i<3; i++){	

		f_size = tf_size[i];
		
		
		/* 1 Build the primary DMA chain */	

		remain_size = f_size;
		num_des = 0;
		
		while(remain_size) {
			/* set descriptor */
			num_des++;
			if (remain_size > SINGLE_DESCRIPTOR_TRANSFER_MAX)
				des_transfer_size =
					SINGLE_DESCRIPTOR_TRANSFER_MAX;
			else
				des_transfer_size = remain_size;
				
			offset = 0;
			if((left > 0) 
				&& (left != SINGLE_DESCRIPTOR_TRANSFER_MAX)
				&& (remain_size == f_size)){
				buffer_page_seq -= 1;
				des_transfer_size = min(des_transfer_size, 
					(SINGLE_DESCRIPTOR_TRANSFER_MAX - left));
				offset = left;
			}

			cur_des_virtual->ddadr =
					buf_node->dma_desc_paddr + DMA_DESCRIPTOR_SIZE * (++dma_descriptor_seq);

			/* physical address */
			cur_des_virtual->dsadr = ci_get_p_add(CIBR0 + i*0x8);
#ifdef	SINGLE_DMA_DESC
			if (0 == i) {
				cur_des_virtual->dtadr = buf_node->paddr + (f_size - remain_size);
			} else if (1 == i) {
				cur_des_virtual->dtadr = buf_node->paddr + tf_size[0];
			} else {
				cur_des_virtual->dtadr = buf_node->paddr + tf_size[0] + tf_size[1];
			}
#else
			cur_des_virtual->dtadr =
				(unsigned int)__pa(page_address(buf_node->pages[buffer_page_seq])) + offset;
			buffer_page_seq++;
#endif
			cur_des_virtual->dcmd =
				des_transfer_size | CI_DMAC_DCMD_INC_TRG_ADDR;

			if (remain_size == f_size){
				/* The address of the first real descriptor */
				buffer_info->dma_desc_vir_addr[i]
					[FIRST_DMA_DESC_FOR_REAL_BUFFER_INX] =
					cur_des_virtual;
				buffer_info->dma_desc_phy_addr[i]
					[FIRST_DMA_DESC_FOR_REAL_BUFFER_INX] =
					buf_node->dma_desc_paddr + DMA_DESCRIPTOR_SIZE * (dma_descriptor_seq - 1);
			}

			if (remain_size <= SINGLE_DESCRIPTOR_TRANSFER_MAX){
				/* The address of the last real descriptor */
				buffer_info->dma_desc_num[i] =
					num_des;
				buffer_info->dma_desc_vir_addr[i]
					[LAST_DMA_DESC_FOR_REAL_BUFFER_INX] =
					cur_des_virtual;
				buffer_info->dma_desc_phy_addr[i]
					[LAST_DMA_DESC_FOR_REAL_BUFFER_INX] =
					buf_node->dma_desc_paddr + DMA_DESCRIPTOR_SIZE * (dma_descriptor_seq - 1);

#ifdef CI_DMAC_BRANCH_BUG
				if(i == 0)
					cur_des_virtual->dcmd |= CI_DMAC_DCMD_SOF_IRQ_EN;
#endif
			left = des_transfer_size;

			}

			/* advance pointers */
			remain_size -= des_transfer_size;
			cur_des_virtual++;
		}

		/* 2 Build the phantom DMA chain */
		remain_size = f_size;
		num_des = 0;
		while(remain_size) {
			/* set descriptor */
			num_des++;
			if (remain_size > SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX)
				des_transfer_size =
					SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX;
			else
				des_transfer_size = remain_size;

			cur_des_virtual->ddadr =
				buf_node->dma_desc_paddr + DMA_DESCRIPTOR_SIZE * (++dma_descriptor_seq);

			/* FIFO physical address */
			cur_des_virtual->dsadr = ci_get_p_add(CIBR0 + i*0x8);
			cur_des_virtual->dtadr =
				cam_ctx->phantom_buffer_physical;
			cur_des_virtual->dcmd = des_transfer_size;

			if (remain_size == f_size){
				/* The address of the first phantom descriptor */
				buffer_info->dma_desc_vir_addr[i]
					[FIRST_DMA_DESC_FOR_PHANTOM_BUFFER_INX] =
					cur_des_virtual;
				buffer_info->dma_desc_phy_addr[i]
					[FIRST_DMA_DESC_FOR_PHANTOM_BUFFER_INX] =
					buf_node->dma_desc_paddr + DMA_DESCRIPTOR_SIZE * (dma_descriptor_seq - 1);
					
			}

			if (remain_size <= SINGLE_DESCRIPTOR_PHANTOM_TRANSFER_MAX){
				/* The address of the last phantom descriptor */
				buffer_info->phantom_dma_desc_num[i] = num_des;
				buffer_info->dma_desc_vir_addr[i]
					[LAST_DMA_DESC_FOR_PHANTOM_BUFFER_INX] =
					cur_des_virtual;
				buffer_info->dma_desc_phy_addr[i]
					[LAST_DMA_DESC_FOR_PHANTOM_BUFFER_INX] =
					buf_node->dma_desc_paddr + DMA_DESCRIPTOR_SIZE * (dma_descriptor_seq - 1);						
				/* loop the phantom buffer to itself */
				cur_des_virtual->ddadr =
					buffer_info->dma_desc_phy_addr[i]
					[FIRST_DMA_DESC_FOR_PHANTOM_BUFFER_INX];
					
#ifdef CI_DMAC_BRANCH_BUG
				if(i == 0)
					cur_des_virtual->dcmd |= CI_DMAC_DCMD_SOF_IRQ_EN;
#endif

			}

			/* advance pointers */
			remain_size -= des_transfer_size;
			cur_des_virtual++;
		}
	}

	spin_unlock_irqrestore(&cam_queue_lock, flags);

#ifndef	DMA_OVERFLOW_WORKAROUND
	if ((unsigned int)buf_node->page_num < buffer_page_seq){			
		camera_free_buffer_node(buf_node, 1);		
		goto exit;
	}
#endif
		
	buffer_info->pY = (void*)buf_node->vaddr;
	buffer_info->pCb = (tf_size[1]) ?
		(void*)((unsigned int)buffer_info->pY + tf_size[0] ) : 0;
	buffer_info->pCr = (tf_size[2]) ?
		(void*)((unsigned int)buffer_info->pCb + tf_size[1]) : 0;
		
	/* return results */
	buf_node->buf_id  = buffer_info->frame_id;
	buf_node->Y_vaddr         = buffer_info->pY;
	buf_node->Cb_vaddr        = buffer_info->pCb;
	buf_node->Cr_vaddr        = buffer_info->pCr;

	list_add_tail(&buf_node->buf_head, g_camera_context->p_buf_head);

exit: 
	*(g_camera_context->p_buf_ready) = 1;

	if (buf_index && buf_node)	//buf_node may have been freed if camera_alloc_buffer_node failed
//	if (buf_index)
		*buf_index = buf_node->buf_index;

	return ret;
}

static int camera_prepare_buffers(p_camera_context_t cam_ctx, int buf_num)
{
	int i;

	struct buf_node *buf_node = NULL;
	unsigned int buf_indx = 0;
	

	if(V4L2_PIX_FMT_JPEG == cam_ctx->capture_input_format){
		
		cam_ctx->jpg_buf_num = buf_num; 
		if(buf_num >= 1){
			camera_prepare_buffer(cam_ctx, 0, 0, &buf_indx);
		}

		buf_node = camera_get_buffer_from_index(buf_indx);

		cam_ctx->jpg_buf_node = buf_node;

		for (i = 0; i < buf_num-1; i++){
			camera_alloc_empty_buffer_node(cam_ctx, buf_node);
		}
		
	}else{

		for (i = 0; i < buf_num; i++)
			camera_prepare_buffer(cam_ctx, 0, 0, NULL);
	}

	return camera_get_buffer_num();
}

static int camera_submit_buffer(p_camera_context_t cam_ctx,
		unsigned int buf_indx)
{
	struct buf_node *buf_node;
	int ret;
	unsigned long flags;

	buf_node = camera_get_buffer_from_index(buf_indx);

	if (!buf_node)
		goto exit;

	spin_lock_irqsave(&cam_queue_lock, flags);	
	ret = mcam_submit_buffer(cam_ctx, buf_node->buf_id);
	spin_unlock_irqrestore(&cam_queue_lock, flags);
	if (ret) {
		dbg("Submit %dth buffer node failure\n", buf_indx);
		goto exit;
	}

	*(g_camera_context->p_buf_submited) = 1;

	return 0;

exit:
	return -EFAULT;
}

static int camera_submit_buffers(p_camera_context_t cam_ctx)
{
	struct list_head *pos;
	struct buf_node *buf_node;
	int i = 0;
	unsigned long flags;

	if (!*(g_camera_context->p_buf_ready)) {
		dbg("buffer not ready!\n");
		goto exit;
	}

	list_for_each(pos, g_camera_context->p_buf_head) {
		buf_node = list_entry(pos, struct buf_node, buf_head);

		spin_lock_irqsave(&cam_queue_lock, flags);
		mcam_submit_buffer(cam_ctx, buf_node->buf_id);
		spin_unlock_irqrestore(&cam_queue_lock, flags);

		i++;
	}

	if (i == 0) {
		goto exit;
	}

	*(g_camera_context->p_buf_submited) = 1;

	return 0;

exit:
	return -EFAULT;
}

static irqreturn_t pxa_camera_vsync_irq(int irq, void *dev_id)
{
	p_camera_context_t cam_ctx = g_camera_context;	
	int io = 0;
	int val = 0;

	io = cam_ctx->platform_ops->vsync_gpio;

	if (gpio_request(io, "QCI vsync")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %X\n", io);
		return IRQ_HANDLED;
	}

	gpio_direction_input(io);
	val = gpio_get_value(io);
	
	if(!val){
		ci_enable();
		cam_ctx->platform_ops->sync_from_gpio();	
		disable_irq(cam_ctx->irq);
	}
	gpio_free(io);	
	
	return IRQ_HANDLED;
}



static int camera_start_capture(p_camera_context_t cam_ctx)
{
	int frame = 0;

	if (!*(cam_ctx->p_buf_submited)) {
		dbg("pxa_camera: buffer not submited!\n");
		goto exit;
	}

	if (V4L2_PIX_FMT_JPEG == cam_ctx->capture_input_format)
		ci_set_interrupt_mask(JPG_INT_MASK);
	else
		ci_set_interrupt_mask(PRE_INT_MASK);

	ci_clear_interrupt_status(0xFFFFFFFF);
	enable_irq(IRQ_CAMERA);	

	if(cam_ctx->irq > 0){
		cam_ctx->platform_ops->sync_to_gpio();
	}
	
	if (mcam_set_capture_format(cam_ctx)) {
		goto exit;
	}
	
	cam_ctx->frame_rate = pxa_camera_get_framerate(cam_ctx);
	ci_set_frame_rate((CI_FRAME_CAPTURE_RATE)cam_ctx->frame_rate);


	/* start capture */
	frame = ((CAMERA_MODE_VIDEO == cam_ctx->capture_mode) ? 0 : 1);
	if(cam_ctx->camera_functions->start_capture(cam_ctx, frame))
			goto exit;	
	
	/* clear ci fifo */
	ci_reset_fifo();

	if(V4L2_PIX_FMT_JPEG == cam_ctx->capture_input_format){

		unsigned char * vaddr = NULL;
		unsigned int fifo0_transfer_size  = PAGE_ALIGN(cam_ctx->fifo_transfer_size[0]);	

		vaddr = cam_ctx->jpg_buf_node->vaddr;
		
		if(0 != cam_ctx->jpg_buf_node->size)
			dma_cache_maint(vaddr, fifo0_transfer_size, DMA_FROM_DEVICE);


		/*using working buffer */

		cam_load_jpg_head(cam_ctx);
		ci_jpg_enable();	

		cam_ctx->jpg_des_base = cam_ctx->jpg_buf_node->dma_desc_paddr;
		cam_ctx->jpg_des_pre = cam_ctx->jpg_des_base;
		cam_ctx->jpg_des_cnt = cam_ctx->fifo_transfer_size[0]/SINGLE_DESCRIPTOR_TRANSFER_MAX;

		printk("ci_jpg_enable\n");

		
	}else{
		cam_configure_dmac(cam_ctx);
	}

	cam_ctx->dma_running = 1;

	if(cam_ctx->irq > 0){

		/*using VSYNC pull down to trigger ci_enable */
		
		if (request_irq(cam_ctx->irq, pxa_camera_vsync_irq, IRQF_TRIGGER_FALLING, 
			"PXA Camera VYSNC", NULL)) {
			printk(KERN_ALERT "PXA_CAMERA: Camera interrupt register failed \n");
			return -1;
		}

	}else{
		/* directly use vsync */
		ci_enable();
	}

	return 0;
exit:
	printk("camera_start_capture error return\n");

	return -EFAULT;
}

static void camera_stop_capture(p_camera_context_t cam_ctx)
{
	/*
	 * stop video capture
	 * stop still capture
	 * Note: a workaround of camera drv for stopping still capture
	 * which has no such stop still capture primitives
	 */
	 
	disable_irq(IRQ_CAMERA);
	mcam_stop_video_capture(cam_ctx);
	
	if(cam_ctx->irq > 0){
		free_irq(cam_ctx->irq, NULL);
	}

	while (!list_empty(cam_ctx->p_report_head)) {
		dbg("no empty item in report head list \n");
		list_del_init(cam_ctx->p_report_head->next);
	}
	
	return;

}

static void camera_desubmit_buffers(p_camera_context_t cam_ctx)
{
	/*
	 * stop capture: a workaround of camera drv
	 * which has no such desubmit buffer primitives
	 */

	cam_ctx->mode[cam_ctx->capture_mode].capture_buffer_queue.head = NULL;
	cam_ctx->mode[cam_ctx->capture_mode].capture_buffer_queue.tail = NULL;

	*(g_camera_context->p_buf_submited) = 0;

}

static void camera_deprepare_buffers(p_camera_context_t cam_ctx)
{
	camera_free_buffer_list(cam_ctx->capture_mode, 1);
	*(g_camera_context->p_buf_ready) = 0;

	return;
}


/***********************************************************************
 *
 * Init/Deinit APIs
 *
 ***********************************************************************/
static int camera_deinit(p_camera_context_t cam_ctx)
{
	int status = 0;

	cam_ctx->platform_ops->deinit();

	/* deinit sensor */
	if (cam_ctx->camera_functions && cam_ctx->camera_functions->deinit)
		cam_ctx->camera_functions->deinit(cam_ctx);

	/* capture interface deinit */
	ci_deinit();
	
	/* disable QCI clock  */
	if(cam_ctx->clk){
		clk_disable(cam_ctx->clk);
		clk_put(cam_ctx->clk);
		cam_ctx->clk = NULL;
	}

	return status;
}

static int camera_init(p_camera_context_t cam_ctx)
{
	struct clk *clk = NULL;

	int   status = 0;
	int i = 0, j = 0;

	/* parameter check */
	/* check the function dispatch table according to the sensor type */
	if ( !cam_ctx->camera_functions )
		return -EINVAL;

	if ( !cam_ctx->camera_functions->init ||
			!cam_ctx->camera_functions->deinit ||
			!cam_ctx->camera_functions->set_capture_format ||
			!cam_ctx->camera_functions->start_capture ||
			!cam_ctx->camera_functions->stop_capture )
		return -EINVAL;

	/* initialize some camera used parameters */
	for(i=0; i<3; i++){
		cam_ctx->fifo_transfer_size[i] = 0;
	}
	
	for(j = 0; j< CAMERA_MODE_NUM; j++){
		cam_ctx->mode[j].capture_buffer_queue.head = NULL;
		cam_ctx->mode[j].capture_buffer_queue.tail = NULL;
		
		for(i=0; i<3; i++){
			cam_ctx->mode[j].fifo_transfer_size[i] = 0;
		}
	}
	
	cam_ctx->capture_input_width = 0;
	cam_ctx->capture_input_height = 0;
	cam_ctx->capture_output_width = 0;
	cam_ctx->capture_output_height = 0;
	cam_ctx->capture_input_format = -1;
	cam_ctx->capture_output_format = -1;
	cam_ctx->frame_buffer_number = 0;
	
	cam_ctx->psu_enable = 0;
	cam_ctx->cgu_enable = 0;
	cam_ctx->ssu_scale = CI_SSU_SCALE_DISABLE;
	cam_ctx->cmu_usage = CI_CMU_DISABLE;
	cam_ctx->dma_running = 0;

	cam_ctx->jpg_offset = 0;
	cam_ctx->jpg_left = 0;
	cam_ctx->jpg_old_count= 0;
	cam_ctx->jpg_new_count= 0;
	cam_ctx->jpg_ci_count = 0;

	cam_ctx->jpg_error = 0;

	cam_ctx->jpg_branch = 0;
	atomic_set(&(cam_ctx->atomic_jpg_full), 0); 

	cam_ctx->branch_flag = 0;

	if(cam_ctx->platform_ops->init()){
		printk("sensor init error\n");
		goto camera_init_err;
	}

#if 0 /* rm by yuhua, config vol at start up, and donot change it anymore*/
	/* power supply */
	if(cam_ctx->camera_functions->v_power > 0){
		pxa3xx_pmic_set_voltage(VCC_CAMERA_ANA, cam_ctx->camera_functions->v_power);
	}
	if(cam_ctx->camera_functions->v_io > 0){
		pxa3xx_pmic_set_voltage(VCC_CAMERA_IO, cam_ctx->camera_functions->v_io);
	}
#endif
	
	/* enable QCI clock  */
	clk = clk_get(NULL, "CAMCLK");
	if (IS_ERR(clk)) {
		printk("camera failed to get camera clock\n");
		return -1;
	}
	clk_enable(clk);

	cam_ctx->clk = clk;
	

	/* capture interface init */
	ci_init();

	/* sensor init */
	status = cam_ctx->camera_functions->init(cam_ctx);
	if (status){
		printk("sensor init error\n");
		goto camera_init_err;
	}

	/* set frame rate */
	ci_set_frame_rate((CI_FRAME_CAPTURE_RATE)cam_ctx->frame_rate);

	return 0;

camera_init_err:
	camera_deinit(cam_ctx);
	return -EIO;

}

/***********************************************************************
 *
 * sleep/wakeup APIs
 *
 ***********************************************************************/
static int mcam_suspend(p_camera_context_t   cam_ctx)
{
	/* stop capture at first, if necessary */
	if(cam_ctx->dma_running) {
		if ((cam_ctx->capture_mode == CAMERA_MODE_VIDEO) 
			|| (cam_ctx->capture_mode == CAMERA_MODE_STILL)) {
			mcam_stop_video_capture(cam_ctx);
		}  else {
			return 0;
		}
		cam_ctx->dma_running = 1;
	}

	/* sleep sensor */
	cam_ctx->camera_functions->sleep(cam_ctx);

	ci_deinit();

	cam_ctx->platform_ops->suspend();

	return 0;
}

static int mcam_resume(p_camera_context_t   cam_ctx)
{
	cam_ctx->platform_ops->resume();

	
	/* wake up sensor */
	if (cam_ctx->camera_functions->wakeup(cam_ctx)){
		goto camera_init_err;
	}

	/* to be continued */
	
	return 0;

camera_init_err:
	camera_deinit(cam_ctx);

	return -EIO;
}



static void pxa_camera_set_mode(p_camera_context_t	cam_ctx)
{
	cam_ctx->p_buf_ready =
		&(cam_ctx->mode[cam_ctx->capture_mode].buf_ready);
	cam_ctx->p_buf_submited =
		&(cam_ctx->mode[cam_ctx->capture_mode].buf_submited);
	cam_ctx->p_capture_started =
		&(cam_ctx->mode[cam_ctx->capture_mode].capture_started);
	cam_ctx->p_buf_head =
		&(cam_ctx->mode[cam_ctx->capture_mode].buf_head);
	cam_ctx->p_report_head =
		&(cam_ctx->mode[cam_ctx->capture_mode].report_head);
	cam_ctx->p_timeperframe_numerator =
		&(cam_ctx->mode[cam_ctx->capture_mode].timeperframe_numerator);
	cam_ctx->p_timeperframe_denominator =
		&(cam_ctx->mode[cam_ctx->capture_mode].timeperframe_denominator);
	cam_ctx->capture_output_format =
		cam_ctx->mode[cam_ctx->capture_mode].output_format;
	cam_ctx->capture_input_format =
		cam_ctx->mode[cam_ctx->capture_mode].input_format;
		
	return;

}

static int camera_do_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, void *arg)
{
	int retval = 0;
	p_camera_context_t cam_ctx = g_camera_context;

	if (!cam_ctx) {
		return -EINVAL;
	}

	switch (cmd) {

		case VIDIOC_QUERYCAP:
			{
				struct v4l2_capability *cap = arg;
				retval = 0;
				memset(cap, 0, sizeof(*cap));
				strcpy(cap->driver, "pxa camera");
				strcpy(cap->card, "");
				cap->version = PXA_CAMERA_VERSION;
				cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
					V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;

				retval = 0;
				break;
			}

		case VIDIOC_ENUMINPUT:
			{
				struct v4l2_input *i = arg;
				unsigned int n;
				
				n = i->index;
				if (n >= SENSOR_MAX) {
					retval = -EINVAL;
					break;
				}
				
				memset(i, 0, sizeof(*i));
				i->index = n;
				i->type = V4L2_INPUT_TYPE_CAMERA;
				
				if(NULL == cam_ctx->sensor_func[n]){
					break;
				}else{
					strcpy(i->name, cam_ctx->sensor_func[n]->name);
				}
				
				break;
			}

		case VIDIOC_ENUM_FMT:
			{
				struct v4l2_fmtdesc *f = arg;
				enum v4l2_buf_type type;
				int index;

				dbg("IOCTL CMD = VIDIOC_ENUM_FMT\n");
				if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				type = f->type;
				index = f->index;

				memset(f, 0, sizeof(*f));
				f->index = index;
				f->type = type;
				retval = __format_enum(cam_ctx, index, &(f->pixelformat)); 
				
				break;
			}

		case VIDIOC_G_FMT:
			{
				struct v4l2_format *f = arg;

				if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				f->fmt.pix.pixelformat = cam_ctx->mode[cam_ctx->capture_mode].output_format;			
				f->fmt.pix.width =	cam_ctx->mode[cam_ctx->capture_mode].width;
				f->fmt.pix.height = cam_ctx->mode[cam_ctx->capture_mode].height;
				break;
			}
		
		case VIDIOC_S_FMT:
		case VIDIOC_TRY_FMT:			
			{
				struct v4l2_format *f = arg;
				int sensor_format = 0;
				int capture_width = 0;
				int capture_height = 0;

				if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if(NULL == cam_ctx->camera_functions){
					printk("please VIDIOC_S_INPUT first");
					retval = -EINVAL;
					break;
				}

				if((f->fmt.pix.width % 0x10)
					||(f->fmt.pix.height % 0x02)){

					printk("width should be 16 pixels multiple\n");
					printk("height should be 2 pixels multiple\n");

					retval = -EINVAL;
					break;
				}

#if 0 /* rm by yuhua */
				if((f->fmt.pix.width > cam_ctx->camera_functions->width_max)
					||(f->fmt.pix.width < cam_ctx->camera_functions->width_min)
					||(f->fmt.pix.height > cam_ctx->camera_functions->height_max)
					||(f->fmt.pix.height < cam_ctx->camera_functions->height_min)	){
					printk("width and height exceed\n");
					retval = -EINVAL;
					break;
				}
#endif

				if((sensor_format = camera_get_sensor_format(f->fmt.pix.pixelformat)) < 0){
					
					printk("user required format can not be accessed\n");
					retval = -EINVAL;
					break;
				}

				if(VIDIOC_TRY_FMT == cmd)
					break;

				capture_width = f->fmt.pix.width;
				capture_height = f->fmt.pix.height;
				pxa_camera_reset(cam_ctx);

				cam_ctx->mode[cam_ctx->capture_mode].output_format = f->fmt.pix.pixelformat;
				cam_ctx->mode[cam_ctx->capture_mode].input_format = sensor_format;
				cam_ctx->mode[cam_ctx->capture_mode].width = f->fmt.pix.width;
				cam_ctx->mode[cam_ctx->capture_mode].height = f->fmt.pix.height;

				cam_ctx->capture_output_format = f->fmt.pix.pixelformat;
                            cam_ctx->capture_input_format = sensor_format;

				break;
			}

		case VIDIOC_S_INPUT:
			{
				int id = *(int * )arg;

				if (*(cam_ctx->p_capture_started)) {
					pxa_camera_ioctl_streamoff(cam_ctx);
				}

				if((id >= SENSOR_MAX) ||(id < SENSOR_LOW)  ){
					printk(KERN_ERR "sensor does not exist\n");
					retval = -EINVAL;
					return retval;
				}

				if(sensor_choose(cam_ctx, id) < 0){
					printk(KERN_ERR "VIDIOC_S_INPUT error\n");
					retval = -EINVAL;
					return retval;
				}			

				retval = camera_init(cam_ctx);
				break;
			}

		case VIDIOC_G_INPUT:
			{
				int *p = arg;

				if(cam_ctx->camera_functions){
					*p = cam_ctx->camera_functions->id;
				}else{
					retval = -EINVAL;
				}			
				break;
			}

		case VIDIOC_G_PARM:
			{
				struct v4l2_streamparm *parm = arg;
				if (parm->type !=
					V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (CAMERA_MODE_STILL ==
					cam_ctx->capture_mode) {
					parm->parm.capture.capturemode =
						V4L2_MODE_HIGHQUALITY;
				} else {
					parm->parm.capture.capturemode = 0;
				}
				parm->parm.capture.timeperframe.numerator =
					*(cam_ctx->p_timeperframe_numerator);
				parm->parm.capture.timeperframe.denominator =
					*(cam_ctx->p_timeperframe_denominator);

				break;
			}

		case VIDIOC_S_PARM:
			{

				struct v4l2_streamparm *parm = arg;
				int capture_mode;

				if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (V4L2_MODE_HIGHQUALITY ==
						parm->parm.capture.capturemode) {
					capture_mode = CAMERA_MODE_STILL;

				} else {
					capture_mode = CAMERA_MODE_VIDEO;

				}
				cam_ctx->queue_cur = cam_ctx->mode[capture_mode].capture_buffer_queue;

				/* Spatial Scaling Unit(SSU) related functions */
				if (CI_SSU_SCALE_HALF ==
						parm->parm.capture.extendedmode) {
					cam_ctx->mode[capture_mode].capture_scale =
							CAMERA_CAPTURE_SCALE_HALF;
					
				} else if (CI_SSU_SCALE_QUARTER ==
						parm->parm.capture.extendedmode) {
					cam_ctx->mode[capture_mode].capture_scale =
						CAMERA_CAPTURE_SCALE_QUATER;
					
				} else {
					cam_ctx->mode[capture_mode].capture_scale = 
						CAMERA_CAPTURE_SCALE_DISABLE;
					
				}

				if ((capture_mode != cam_ctx->capture_mode) ||
					(*(cam_ctx->p_timeperframe_numerator) !=
					parm->parm.capture.timeperframe.numerator) ||
					(*(cam_ctx->p_timeperframe_denominator) !=
					parm->parm.capture.timeperframe.denominator)) {

					if (*(cam_ctx->p_capture_started)) {
						pxa_camera_ioctl_streamoff(cam_ctx);
					}

					if (capture_mode != cam_ctx->capture_mode) {
						cam_ctx->capture_mode = capture_mode;
						pxa_camera_set_mode(cam_ctx);
					}

					*(g_camera_context->p_timeperframe_numerator) =
						parm->parm.capture.timeperframe.numerator;
					*(g_camera_context->p_timeperframe_denominator) =
						parm->parm.capture.timeperframe.denominator;

				}

				break;

			}

		case VIDIOC_S_CTRL:
			{
				struct v4l2_control *ctrl = arg;
#if 0				
				switch (ctrl->id) {
					case V4L2_CID_GAIN:
						retval =
							cam_ctx->camera_functions->set_whitebalance(
							cam_ctx,
							SENSOR_MANUAL_WHITEBALANCE,
							ctrl->value);
						break;
					case V4L2_CID_AUTO_WHITE_BALANCE:
						retval =
							cam_ctx->camera_functions->set_whitebalance(
							cam_ctx,
							SENSOR_AUTO_WHITEBALANCE,
							ctrl->value);
						break;
					case V4L2_CID_EXPOSURE:
						retval =
							cam_ctx->camera_functions->set_exposure(
							cam_ctx,
							SENSOR_MANUAL_EXPOSURE,
							ctrl->value);
						break;
					case V4L2_CID_AUTOGAIN:
						retval =
							cam_ctx->camera_functions->set_exposure(
							cam_ctx,
							SENSOR_AUTO_EXPOSURE,
							ctrl->value);
						break;
					default:
						retval = -EINVAL;
						break;
				}
#else
				retval = cam_ctx->camera_functions->set_mode(cam_ctx,	ctrl->id, ctrl->value);
#endif
				break;
			}

		case VIDIOC_G_CTRL:
			{
				struct v4l2_control *ctrl = arg;
				if (cam_ctx->camera_functions->get_mode)
					retval = cam_ctx->camera_functions->get_mode(cam_ctx,	ctrl->id, &ctrl->value);
				break;
			}

		case VIDIOC_QUERYBUF:
			{
				struct v4l2_buffer *buf = arg;
				int buf_size;

				if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if (buf->memory != V4L2_MEMORY_MMAP) {
					retval = -EINVAL;
					break;
				}

				if (buf->index >= camera_get_buffer_num()) {
					retval = -EINVAL;
					break;
				}

				buf_size = camera_get_buffer_size();
				buf->length = buf_size;
				buf->m.offset = buf->index * buf_size;

				break;
			}

		case VIDIOC_QBUF:
			{
				struct v4l2_buffer *buf = arg;
				unsigned long flags;

				if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}
				if ((buf->memory != V4L2_MEMORY_MMAP)
				       && (buf->memory != V4L2_MEMORY_USERPTR))	{
					retval = -EINVAL;
					break;
				}
				 
				if ((buf->memory == V4L2_MEMORY_USERPTR)
						&& (buf->length > 0)){
					if (camera_prepare_buffer(cam_ctx,
						buf->m.userptr, buf->length, &buf->index)){
						retval = -EINVAL;
						break;
					}
				}

				if (buf->index >= camera_get_buffer_num()) {
					retval = -EINVAL;
					break;
				}

				spin_lock_irqsave(&report_list_lock, flags);
				retval = camera_submit_buffer(cam_ctx, buf->index);
				spin_unlock_irqrestore(&report_list_lock, flags);

				break;
			}

		case VIDIOC_DQBUF:
			{
				struct v4l2_buffer *buf = arg;
				struct buf_node *buf_node;
				unsigned long flags;

				if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}
				if ((buf->memory != V4L2_MEMORY_MMAP)
				       && (buf->memory != V4L2_MEMORY_USERPTR))	{
					retval = -EINVAL;
					break;
				}

				spin_lock_irqsave(&report_list_lock, flags);

				if (!list_empty(cam_ctx->p_report_head)) {
					buf_node =
						list_entry(
						cam_ctx->p_report_head->
						next,
						struct buf_node,
						report_head);
					assert(buf_node);

					list_del(&buf_node->report_head);

					buf->index = buf_node->buf_index;

					buf->bytesused = buf_node->bytesused;
					buf->m.offset = buf_node->offset;

					/* 
					 * invalidat cache, next time get data from memory 
					 * instead of cache.
					 * otherwise display may have error line
					 */
					if((0 != buf_node->vaddr) && (0 != buf_node->size))
						dma_cache_maint(buf_node->vaddr, buf_node->size, DMA_FROM_DEVICE);

				} else
					retval = -EIO;
				spin_unlock_irqrestore(&report_list_lock, flags);

				break;
			}

		case VIDIOC_REQBUFS:
			{
				struct v4l2_requestbuffers *req = arg;
				int count;

				if (req->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				if ((req->memory != V4L2_MEMORY_MMAP)
				       && (req->memory != V4L2_MEMORY_USERPTR))	{
					retval = -EINVAL;
					break;
				}

				camera_deprepare_buffers(cam_ctx);
				 
				if (req->memory == V4L2_MEMORY_USERPTR)
					break;

				count = camera_prepare_buffers(cam_ctx,
					req->count);
				printk("VIDIOC_REQBUFS %d %d\n", count, req->count);
				
				if (count < 0)
					retval = -EFAULT;
				else
					req->count = count;

				break;
			}

		case VIDIOC_STREAMON:
			{
				int *type = arg;

				if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					dbg("VIDIOC_STREAMON failed!\n");
					retval = -EINVAL;
					break;
				}

				retval = pxa_camera_ioctl_streamon(cam_ctx);

				break;
			}

		case VIDIOC_STREAMOFF:
			{
				int *type = arg;

				if (*type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
					retval = -EINVAL;
					break;
				}

				pxa_camera_ioctl_streamoff(cam_ctx);

				break;
			}

			/* Application extended IOCTL.  */
			/* Register access interface    */

		case WCAM_VIDIOCGCIREG:
			{
				struct reg_set_s *reg = arg;
				dbg("IOCTL CMD = WCAM_VIDIOCGCIREG\n");
				reg->val2 =  __raw_readl(
					((unsigned int)cam_ctx->ci_reg_base
					  + (reg->val1)));
				break;

			}

		case WCAM_VIDIOCSCIREG:
			{
				struct reg_set_s *reg = arg;
				volatile unsigned int regVal;

				dbg("IOCTL CMD = WCAM_VIDIOCSCIREG\n");
				__raw_writel(reg->val2, (unsigned int)cam_ctx->
					ci_reg_base + reg->val1);
				regVal =
					__raw_readl((unsigned int)cam_ctx->
						ci_reg_base + reg->val1);
				if (regVal != reg->val2) {
					retval = -EFAULT;
					break;
				}
				break;
			}

		case WCAM_VIDIOCGCAMREG:
			{
				struct reg_set_s *reg = arg;

				dbg("IOCTL CMD = WCAM_VIDIOCGCAMREG\n");
				
				if (cam_ctx->camera_functions->read_reg) {
					cam_ctx->camera_functions->read_reg(cam_ctx, 
							reg->val1, 
							&reg->val2);
				}
				break;
			}

		case WCAM_VIDIOCSCAMREG:
			{
				struct reg_set_s *reg = arg;
				dbg("IOCTL CMD = WCAM_VIDIOCSCAMREG\n");
				if (cam_ctx->camera_functions->write_reg) {
					retval = cam_ctx->camera_functions->write_reg(cam_ctx, reg->val1, reg->val2);
				} else 
					retval = -EIO;
				break;
			}

		default:
			{
				dbg("Invalid ioctl parameters.\n");
				retval = -ENOIOCTLCMD;
				break;
			}
	}

	return retval;
}


static int camera_get_jpg(p_camera_context_t cam_ctx, int *count, int *buffer_id, int *offset)
{
	camera_frame_buffer_queue_t *queue = NULL;
	unsigned int fifo0_transfer_size;
	unsigned char * vaddr = NULL;

	int num = *count;
	int off = cam_ctx->jpg_offset;


	*buffer_id = -1;

	queue = &(cam_ctx->mode[cam_ctx->capture_mode].capture_buffer_queue);
	
	/* fifo0_transfer_size =  PAGE_ALIGN(cam_ctx->fifo_transfer_size[0]);	*/
	fifo0_transfer_size =  (cam_ctx->fifo_transfer_size[0]);	

	/*using working buffer */

	vaddr = cam_ctx->jpg_buf_node->vaddr;

	if(NULL == vaddr){
		printk(KERN_INFO "NULL == vaddr\n");
		goto out;
	}
	
	if(cam_ctx->jpg_offset + num  >= fifo0_transfer_size){
		/* here is in funtum buffer */
		/* printk("here is in funtum buffer\n"); */
		cam_ctx->jpg_offset += num;
		ci_pause(cam_ctx);

		cam_ctx->jpg_offset = 0;
		
		
		if(cam_ctx->jpg_buf_num == atomic_read(&(cam_ctx->atomic_jpg_full))){

			cam_ctx->jpg_new_count = 0;
			cam_ctx->jpg_old_count = 0;
			cam_ctx->jpg_ci_count = 0;
			cam_ctx->jpg_branch = 0;
			
			/* restart */
			ci_resume(cam_ctx);	
			
		}else
			cam_ctx->jpg_branch = 1;
		
		goto out;

	
	}
	
	if((vaddr[off] != 0xFF)||(vaddr[off+1] != 0xD8)){
		printk(KERN_INFO "first data is not header, with offset: 0x%x\n", off);
		cam_ctx->jpg_error++;	
		goto err;
	}

	*offset = off;
	cam_ctx->jpg_offset = off + *count;		/*just delete all the data each time */


	/*submit user buffer */
	
	/* check if there are any buffers filled with valid image data */
	if (queue->head == NULL) {
		printk("queue->head == NULL\n");
		/* delete this frame */
		goto out;
	} else {		
		*buffer_id  = queue->head->frame_id;
		queue->head = queue->head->next_buffer;	
		
		atomic_dec(&(cam_ctx->atomic_jpg_full));
	}
	return 0;

err:
	cam_ctx->jpg_offset += num;			/* totally delete this frame */
	
	return -1;

out:	
	return -1;
	
}

#define JPG_CNT_DMA

static int cam_jpg_count(p_camera_context_t cam_ctx, int *num)
{

#ifndef JPG_CNT_DMA

	unsigned int count = 0, delta = 0; 
	int new_count = 0;
	int old_count = cam_ctx->jpg_old_count;

	count = ci_jpg_cnt_get();
	delta = count - cam_ctx->jpg_ci_count;

	/* align 8 bytes */
	cam_ctx->jpg_new_count += (delta+7)&0xfffffff8;
	cam_ctx->jpg_ci_count = count;

	new_count = cam_ctx->jpg_new_count;

	if(new_count >= old_count){
		*num = new_count - old_count;
	}else{
		/* 24 bit for count */
		/* printk("!!!!!!!!!!!!!!!!!bigger  than 24bits tmp=%x\n ", new_count); */
		*num =  0x0ffffff - old_count + new_count;
	}		
	
	cam_ctx->jpg_old_count = new_count;

#else

	int count = 0;
	int add = 0;
	int id = 0;
	unsigned long citadr;
	int frame_size_for_one_des = 0;
	
	ci_jpg_cnt_get_dma(&count, &add);

	id = (add - cam_ctx->jpg_des_pre) >> 4;

	citadr = READ_REG(CITADR0);
	/* if in phantom buffer, reset CI camera for JPEG*/
	if((citadr >= cam_ctx->phantom_buffer_physical) &&
			(citadr <= (cam_ctx->phantom_buffer_physical + PHANTOM_BUFFER_SIZE))) {
		
		ci_pause(cam_ctx);

		cam_ctx->jpg_offset = 0;

		cam_ctx->jpg_des_pre = cam_ctx->jpg_des_base;			
		
		if(cam_ctx->jpg_buf_num == atomic_read(&(cam_ctx->atomic_jpg_full))){

			cam_ctx->jpg_new_count = 0;
			cam_ctx->jpg_old_count = 0;
			cam_ctx->jpg_ci_count = 0;
			cam_ctx->jpg_branch = 0;
			
			/* restart */
			ci_resume(cam_ctx);	
			
		}else
			cam_ctx->jpg_branch = 1;

		return -1;
		
	}


	frame_size_for_one_des = min(SINGLE_DESCRIPTOR_TRANSFER_MAX, cam_ctx->fifo_transfer_size[0]);
	if (id > 0) {
		*num = (frame_size_for_one_des - count) +
			(id-1)  * frame_size_for_one_des + cam_ctx->jpg_old_count;
	} else {
		*num = cam_ctx->jpg_old_count - count;
	}

	cam_ctx->jpg_des_pre = add;
	cam_ctx->jpg_old_count = count;
#endif

	return 0;

	
}

/* static void cam_work_handler(struct work_struct *_work) */
static void cam_work(p_camera_context_t cam_ctx, int num)
{
	/* p_camera_context_t cam_ctx = container_of(_work, struct camera_context_s, work); */

	struct buf_node *buf_node;
	int buf_id = -1;

	unsigned long flags1, flags2;	
	int count = num;
	int offset = 0;

	if( cam_ctx->dma_running == 0){
		printk(KERN_INFO "cam_work_handler(): dma is not running\n");
		return;
	}
	
	/*seek for data and re-value count & offset */
	if(!camera_get_jpg(cam_ctx, &count, &buf_id, &offset)){
		spin_lock_irqsave(&report_list_lock, flags1);
		spin_lock_irqsave(&cam_queue_lock, flags2);

		buf_node = camera_get_buffer_from_id(buf_id);
		buf_node ->bytesused = count;
		buf_node ->offset = offset;
		
		if (&buf_node->report_head
			       	!= g_camera_context->p_report_head->next){
			list_add_tail(&buf_node->report_head,
				g_camera_context->p_report_head);
		}
		
		if (g_camera_context->task_waiting) {
			wake_up_interruptible(&
					(g_camera_context->camera_wait_q));
			g_camera_context->task_waiting = 0;
		}

		spin_unlock_irqrestore(&cam_queue_lock, flags2);
		spin_unlock_irqrestore(&report_list_lock, flags1);	
	}
}


static irqreturn_t pxa_camera_irq(int irq, void *dev_id)
{
	unsigned int int_state;
	int num = 0;
	p_camera_context_t cam_ctx = g_camera_context;
	struct buf_node *buf_node;
	unsigned long cicr0, des_physical;
	camera_frame_buffer_queue_t *queue = &(cam_ctx->mode[cam_ctx->capture_mode].capture_buffer_queue);

	int buf_id = -1, i;

	unsigned long flags1, flags2;
	
	int_state = ci_get_interrupt_status();
	
	if(V4L2_PIX_FMT_JPEG == cam_ctx->capture_input_format){

		#if 0
		if(int_state & 0x08){
			/* consider EOF if using JPEG cnt */
			queue_work(cam_ctx->cam_wq, &cam_ctx->work);	
		}
		#endif
		
		if(int_state & 0x40000000){
			/* JPEG mode consider EOFX, if using DMA count */
			if(cam_jpg_count(cam_ctx, &num) == 0)
				cam_work(cam_ctx, num);							
		}
		
	}else{

#ifdef	CI_DIS_ENA
		if (int_state & 0x10) {	/* Start Of Frame */
			/* Issue ci disable */
			cicr0 = READ_REG(CICR0);
			cicr0 |= CICR0_DIS;
			WRITE_REG(CICR0, cicr0);
		}

		if (int_state & 0x20) {	/* Disable Done */
#else
		/* non JPEG mode just consider EOFX */
		if(int_state & 0x40000000){
#endif

			spin_lock_irqsave(&report_list_lock, flags1);
			spin_lock_irqsave(&cam_queue_lock, flags2);
			
			if (!mcam_get_filled_buffer(cam_ctx, &buf_id)) {

				buf_node = camera_get_buffer_from_id(buf_id);				
				
				if (&buf_node->report_head
					       	!= g_camera_context->p_report_head->next){
					list_add_tail(&buf_node->report_head,
						g_camera_context->p_report_head);
				}
				if (g_camera_context->task_waiting) {
					wake_up_interruptible(&
							(g_camera_context->camera_wait_q));
					g_camera_context->task_waiting = 0;
				}
			}

#ifdef	CI_DIS_ENA
			if (queue && queue->head) {
				/*
				 * If queue->head is not NULL,
				 * load its DMA descriptor
				 */
				for(i=0; i<3; i++){
					des_physical = queue->head->
						dma_desc_phy_addr[i][FIRST_DMA_DESC_FOR_REAL_BUFFER_INX];
					ci_dma_load_descriptor(des_physical, CI_DMA_CHANNEL_0+i);
				}
			} else {
				/* 
				 * If queue->head is NULL, load the phantom
				 * buffer descriptor of queue->tail.
				 */
				for(i=0; i<3; i++){
					des_physical = queue->tail->
						dma_desc_phy_addr[i][FIRST_DMA_DESC_FOR_PHANTOM_BUFFER_INX];
					ci_dma_load_descriptor(des_physical, CI_DMA_CHANNEL_0+i);
				}
			}

			if (cam_ctx && cam_ctx->camera_functions && cam_ctx->camera_functions->load_lut)
				cam_ctx->camera_functions->load_lut(cam_ctx);

			/* Reenable Ci in CI disable done irq */
			cicr0 = READ_REG(CICR0);
			cicr0 |= CICR0_ENB;
			WRITE_REG(CICR0, cicr0);
#endif

			spin_unlock_irqrestore(&cam_queue_lock, flags2);
			spin_unlock_irqrestore(&report_list_lock, flags1);	
			
		}
		
	}
	
	ci_clear_interrupt_status(int_state);

	return IRQ_HANDLED;
}

/********************************************************************************************
 * Application interface
 *******************************************************************************************/

static int pxa_camera_get_framerate(p_camera_context_t cam_ctx)
{

	unsigned int framerate = 0;
	int err = -1;

	int format = cam_ctx->capture_input_format;
	int width = cam_ctx->capture_input_width;
	int height = cam_ctx->capture_input_height;

	int numerator = 0;	
	int denominator = 0;

	if ((0 == *(cam_ctx->p_timeperframe_denominator))
			|| (0 == *(cam_ctx->p_timeperframe_numerator))) {
		framerate = 0;
		return framerate;

	}

	err = cam_ctx->camera_functions->get_framerate(format, width, height, &numerator, &denominator);

	if(err < 0){
		return err;
	}
	
	if(denominator > 0){
		framerate =
			(*(cam_ctx->p_timeperframe_numerator)) * numerator/
			(*(cam_ctx->p_timeperframe_denominator)) * denominator;
	}

	return framerate;	

}

static int pxa_camera_reset(p_camera_context_t cam_ctx)
{
	if (*(cam_ctx->p_capture_started)) {
		pxa_camera_ioctl_streamoff(cam_ctx);
	}

	if (*(cam_ctx->p_buf_submited)) {
		camera_desubmit_buffers(cam_ctx);
	}

	if (*(cam_ctx->p_buf_ready)) {
		camera_deprepare_buffers(cam_ctx);
	}

	return 0;
}

static int pxa_camera_ioctl_streamon(p_camera_context_t cam_ctx)
{

	struct buf_node *buf_node;
	unsigned long flags;
	unsigned long camera_buffer_size;

	if (!*(cam_ctx->p_buf_ready)) {
		camera_buffer_size = cam_ctx->mode[cam_ctx->capture_mode].width
			* cam_ctx->mode[cam_ctx->capture_mode].height;

		if (camera_buffer_size > 1920000) {
			/* only allocate one buffer for 1600*1200 above to save memory */
			if (camera_prepare_buffers(cam_ctx, 1) < 0) {
				return -EIO;
			}
		} else {
			if (camera_prepare_buffers(cam_ctx, 2) < 0) {
				return -EIO;
			}
		}
	}

	if (!*(cam_ctx->p_buf_submited)) {
		if (camera_submit_buffers(cam_ctx)) {
			return -EIO;
		}
	}

	spin_lock_irqsave(&report_list_lock, flags);
	/* Empty report head and put its buf into head queue */
	while (!list_empty(cam_ctx->p_report_head)) {
		printk(KERN_DEBUG " report head isn't empty\n");
		buf_node =
			list_entry(cam_ctx->p_report_head->next,
					struct buf_node, report_head);
		list_del_init(&buf_node->report_head);
		if (camera_submit_buffer(cam_ctx, buf_node->buf_index)) {
			spin_unlock_irqrestore(&report_list_lock, flags);
			return -EIO;
		}
	}
	spin_unlock_irqrestore(&report_list_lock, flags);

	if (!*(cam_ctx->p_capture_started)) {
		if (camera_start_capture(cam_ctx)) {
			return -EIO;
		}
	}

	*(cam_ctx->p_capture_started) = 1;
	return 0;
}

static void pxa_camera_ioctl_streamoff(p_camera_context_t cam_ctx)
{

	camera_stop_capture(cam_ctx);
	*(cam_ctx->p_capture_started) = 0;

	return;
}

static int pxa_camera_open(struct inode *inode, struct file *file)
{
	int status = -1;
	int i = 0;
	dma_addr_t handle = 0;

	p_camera_context_t cam_ctx = g_camera_context;

	if (cam_ctx->suspended) {
		return status;
	}
#ifdef CONFIG_PXA3xx_DVFM 
	//dvfm_disable(dvfm_dev_idx);/* enable cpu freq adjust by frank */
	dvfm_disable_op_name("D0CS", dvfm_dev_idx);
#ifdef CONFIG_BOARD_LANDMARK
	cam_ctx->old_mspm_state = mspm_state();
	if (cam_ctx->old_mspm_state)
		mspm_disable();
#endif
#endif
	if(cam_ctx->driver_opened){
		printk(KERN_ERR "camera driver has already been opened\n");
		return -EACCES;
	}
	
	cam_ctx->driver_opened++;	


	init_waitqueue_head(&(cam_ctx->camera_wait_q));
	cam_ctx->task_waiting = 0;

	/* set default value */
	cam_ctx->capture_mode = CAMERA_MODE_VIDEO;
	pxa_camera_set_mode(cam_ctx);

	INIT_LIST_HEAD(&pc_head.page_list);
	pc_head.page_count = 0;
	spin_lock_init(&pc_head.lock);
	init_MUTEX(&buf_list_sem);
	spin_lock_init(&report_list_lock);
	spin_lock_init(&cam_queue_lock);

	for(i=0; i<CAMERA_MODE_NUM; i++){
		cam_ctx->mode[i].buf_ready = 0;
		cam_ctx->mode[i].buf_submited = 0;
		cam_ctx->mode[i].capture_started = 0;

		cam_ctx->mode[i].timeperframe_numerator = 0;
		cam_ctx->mode[i].timeperframe_denominator = 0;

		INIT_LIST_HEAD(&(cam_ctx->mode[i].buf_head));
		INIT_LIST_HEAD(&(cam_ctx->mode[i].report_head));
	
		cam_ctx->mode[i].width = WIDTH_DEFT;
		cam_ctx->mode[i].height = HEIGHT_DEFT;
		cam_ctx->mode[i].capture_scale = CAMERA_CAPTURE_SCALE_DISABLE;
		cam_ctx->mode[i].input_format =
			V4L2_PIX_FMT_YUV422P;
		cam_ctx->mode[i].output_format =
			V4L2_PIX_FMT_YUV422P;
	}

	cam_ctx->frame_rate = FRAMERATE_DEFT;
	
	/*
	 * we alloc histogram/LUT buffer and its dma descriptor here
	 */
	cam_ctx->histogram_lut_buffer_virtual = dma_alloc_coherent(NULL,
			1024, &handle, GFP_KERNEL);

	if (!cam_ctx->histogram_lut_buffer_virtual) {
		printk("Can't get memory for histogram buffer\n");
		goto alloc_histogram_buffer_error;
	} else
		cam_ctx->histogram_lut_buffer_physical = (volatile u32)handle;


	cam_ctx->histogram_lut_dma_descriptors_virtual =
		dma_alloc_coherent(NULL, 16, &handle, GFP_KERNEL);

	if (!cam_ctx->histogram_lut_dma_descriptors_virtual) {
		printk("Can't get memory for phantom buffer\n");
		goto alloc_histogram_dma_desc_error;
	} else
		cam_ctx->histogram_lut_dma_descriptors_physical =
			(volatile u32)handle;

	/*
	 * We alloc phantom buffer here. The buffer node (list) will be
	 * alloc when application start capturing.
	 */
	cam_ctx->phantom_buffer_virtual = dma_alloc_coherent(NULL,
			PHANTOM_BUFFER_SIZE,
			&handle,
			GFP_KERNEL);
	if (!cam_ctx->phantom_buffer_virtual) {
		dbg("Can't get memory for phantom buffer\n");
		goto alloc_phantom_buffer_error;
	} else {
		cam_ctx->phantom_buffer_physical = (volatile u32)handle;
	}
	
	/* set interrupt mask */
	if (V4L2_PIX_FMT_JPEG == cam_ctx->capture_input_format)
		ci_set_interrupt_mask(JPG_INT_MASK);
	else
		ci_set_interrupt_mask(PRE_INT_MASK);

	/* empty the report list */
	while (!list_empty(cam_ctx->p_report_head)) {
		dbg("no empty item in report head list \n");
		list_del_init(cam_ctx->p_report_head->next);
	}

	/* empty the head list */
	while (!list_empty(cam_ctx->p_buf_head)) {
		dbg("no empty item in report head list \n");
		list_del_init(cam_ctx->p_buf_head->next);
	}

	return 0;


alloc_phantom_buffer_error:

	dma_free_coherent(NULL, 16,
			(void *)cam_ctx->
			histogram_lut_dma_descriptors_virtual,
			cam_ctx->histogram_lut_dma_descriptors_physical);

alloc_histogram_dma_desc_error:

	dma_free_coherent(NULL, 1024,
			(void *)cam_ctx->histogram_lut_buffer_virtual,
			cam_ctx->histogram_lut_buffer_physical);

alloc_histogram_buffer_error:
	return status;
}

static int pxa_camera_close(struct inode *inode, struct file *file)
{
	p_camera_context_t cam_ctx = g_camera_context;
	struct page *page;
	unsigned long flags;
	int i = 0;

	cam_ctx->driver_opened--;

	if (*(cam_ctx->p_capture_started)) {
		pxa_camera_ioctl_streamoff(cam_ctx);
	}

	camera_deinit(cam_ctx);

	if (cam_ctx->phantom_buffer_virtual) {
		dma_free_coherent(NULL, PHANTOM_BUFFER_SIZE,
				(void *)cam_ctx->phantom_buffer_virtual,
				cam_ctx->phantom_buffer_physical);
	}
	if (cam_ctx->histogram_lut_dma_descriptors_virtual) {
		dma_free_coherent(NULL, 16,
				(void *)cam_ctx->
				histogram_lut_dma_descriptors_virtual,
				cam_ctx->
				histogram_lut_dma_descriptors_physical);
	}

	if (cam_ctx->histogram_lut_buffer_virtual) {
		dma_free_coherent(NULL, 1024,
				(void *)cam_ctx->histogram_lut_buffer_virtual,
				cam_ctx->histogram_lut_buffer_physical);
	}
	
	for(i=0; i<CAMERA_MODE_NUM; i++){
		/* empty the report list */
		while (!list_empty(&(cam_ctx->mode[i].report_head))) {
			list_del_init(cam_ctx->mode[i].report_head.next);
		}
		camera_free_buffer_list(i, 1);
	}
	
#ifdef CONFIG_PXA3xx_DVFM
	//dvfm_enable(dvfm_dev_idx); /* enable cpu freq adjust by frank */
	dvfm_enable_op_name_aync("D0CS", dvfm_dev_idx);
#ifdef CONFIG_BOARD_LANDMARK
	if (cam_ctx->old_mspm_state)
		mspm_enable();
#endif
#endif
	/* Free all the camera page cache */
	while (!list_empty(&pc_head.page_list)) {
		spin_lock_irqsave(&pc_head.lock, flags);
		page = list_entry(pc_head.page_list.next, struct page, lru);
		list_del(&page->lru);
		spin_unlock_irqrestore(&pc_head.lock, flags);

		atomic_set(&page->_count, 1);
		ClearPageReserved(page);
		__free_page(page);
	}
	pc_head.page_count = 0;
	cam_ctx->jpg_branch = 0;

	return 0;
}

static ssize_t pxa_camera_read(struct file *file, char __user * buf,
		size_t count, loff_t * ppos)
{
	p_camera_context_t cam_ctx = g_camera_context;
	ssize_t ret = 0;
	struct buf_node *buf_node;
	char __user *tmp_buf = buf;
	unsigned long flags;

	if (!*(cam_ctx->p_capture_started)) {
		if (pxa_camera_ioctl_streamon(cam_ctx)) {
			return -EIO;
		}
	}

	spin_lock_irqsave(&report_list_lock, flags);
	if (list_empty(cam_ctx->p_report_head)) {
		int ret;
		cam_ctx->task_waiting = 1;
		spin_unlock_irqrestore(&report_list_lock, flags);
		ret = wait_event_interruptible(cam_ctx->camera_wait_q,
				!list_empty(cam_ctx->p_report_head));
		if (ret)
			return ret;
	} else {
		spin_unlock_irqrestore(&report_list_lock, flags);
	}

	spin_lock_irqsave(&report_list_lock, flags);
	buf_node =
		list_entry(cam_ctx->p_report_head->next, struct buf_node,
				report_head);
	if (!buf_node) {
		ret = -EINVAL;
		spin_unlock_irqrestore(&report_list_lock, flags);
		goto out;
	}

	list_del(&buf_node->report_head);
	spin_unlock_irqrestore(&report_list_lock, flags);

	if (copy_to_user(tmp_buf, buf_node->Y_vaddr, buf_node->fifo0_size)) {
		ret = -EFAULT;
		goto out;
	}
	
	tmp_buf += buf_node->fifo0_size;
	if (buf_node->fifo1_size) {
		if (copy_to_user
				(tmp_buf, buf_node->Cb_vaddr, buf_node->fifo1_size)) {
			ret = -EFAULT;
			goto out;
		}
		tmp_buf += buf_node->fifo1_size;
	}

	if (buf_node->fifo2_size) {
		if (copy_to_user
				(tmp_buf, buf_node->Cr_vaddr, buf_node->fifo2_size)) {
			ret = -EFAULT;
			goto out;
		}
	}

	ret =
		buf_node->fifo0_size + buf_node->fifo1_size + buf_node->fifo2_size;

out:
	camera_submit_buffer(cam_ctx, buf_node->buf_index);

	return ret;
}

static int pxa_camera_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long iterators = (unsigned long)vma->vm_start;
	unsigned long size = 0;
	int i, ret = 0;
	struct buf_node *buf_node;
	unsigned int offset;
	unsigned int buf_index;

	p_camera_context_t cam_ctx = g_camera_context;
	unsigned long camera_buffer_size;
	unsigned long pgprot = pgprot_noncached(PAGE_SHARED);

	camera_buffer_size = cam_ctx->mode[cam_ctx->capture_mode].width 
		* cam_ctx->mode[cam_ctx->capture_mode].height;
		
#define DCACHE_SIZE 32768
	if (camera_buffer_size > 2*DCACHE_SIZE)
		pgprot |= L_PTE_CACHEABLE | L_PTE_BUFFERABLE;
#ifdef	CACHEABLE_BUFFER
	pgprot |= L_PTE_CACHEABLE | L_PTE_BUFFERABLE;
#endif

	offset = vma->vm_pgoff << PAGE_SHIFT;
	buf_index = offset / camera_get_buffer_size();

	down_interruptible(&buf_list_sem);
	buf_node = camera_get_buffer_from_index(buf_index);

	for (i = 0; i < buf_node->page_num; i++) {
		if (remap_pfn_range(vma,
				iterators,
				(__pa(page_address(buf_node->pages[i]))) >>
				PAGE_SHIFT, PAGE_SIZE, pgprot)
				) {
			ret = -EFAULT;
			up(&buf_list_sem);
			goto remap_page_error;
		}
		size += PAGE_SIZE;
		iterators += PAGE_SIZE;
	}
	up(&buf_list_sem);

	return ret;

remap_page_error:
	/* FIXME: Maybe we should use other function to unmap */
	do_munmap(vma->vm_mm, vma->vm_start, size);
	return ret;
}

static unsigned int pxa_camera_poll(struct file *file, poll_table * wait)
{
	unsigned long flags;
	p_camera_context_t cam_ctx = g_camera_context;

	if (!*(cam_ctx->p_capture_started)) {
		if (pxa_camera_ioctl_streamon(cam_ctx)) {
			return -EIO;
		}
	}

	spin_lock_irqsave(&report_list_lock, flags);
	if (!list_empty(cam_ctx->p_report_head)) {
		spin_unlock_irqrestore(&report_list_lock, flags);
		dbg("poll successfully!\n");
		return POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&report_list_lock, flags);

	cam_ctx->task_waiting = 1;
	poll_wait(file, &(cam_ctx->camera_wait_q), wait);

	spin_lock_irqsave(&report_list_lock, flags);
	if (!list_empty(cam_ctx->p_report_head)) {
		spin_unlock_irqrestore(&report_list_lock, flags);
		dbg("poll successfully!\n");
		return POLLIN | POLLRDNORM;
	} else {
		spin_unlock_irqrestore(&report_list_lock, flags);
		dbg("poll failed!\n");
		return 0;
	}
}

static int pxa_camera_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long param)
{
	return video_usercopy(inode, file, cmd, param, camera_do_ioctl);
}

static void pxa_camera_release(struct video_device *dev)
{
	printk(KERN_WARNING "pxa_camera_zl:  has no release callback. "
			"Please fix your driver for proper sysfs support, see "
			"http://lwn.net/Articles/36850/\n");

	return;
}

static struct file_operations pxa_camera_fops = {
	.owner 		= THIS_MODULE,
	.open 		= pxa_camera_open,
	.release 	= pxa_camera_close,
	.ioctl 		= pxa_camera_ioctl,
	.read 		= pxa_camera_read,
	.mmap 		= pxa_camera_mmap,
	.poll 		= pxa_camera_poll,
	.llseek 	= no_llseek,
};

static struct video_device vd = {
	.name 		= "PXA Camera",
	.fops 		= &pxa_camera_fops,
	.release 	= pxa_camera_release,
	.minor 		= -1,
};

#ifdef CONFIG_PM
/*
 * Suspend the Camera Module.
 */
static int pxa_camera_suspend(struct platform_device *pdev, pm_message_t state)
{
	p_camera_context_t cam_ctx = g_camera_context;

	cam_ctx->suspended = 1;

	if (!(cam_ctx->driver_opened)) {
		if (cam_ctx->i2c_inited) {
			cam_ctx->i2c_inited = 0;
		}
		return 0;
	}

	if (cam_ctx->driver_opened) {
		return -EIO;
	}

	if ((state.event == PM_EVENT_SUSPEND)) {
		printk(KERN_ALERT "PXA_CAMERA: camera suspend\n");
		mcam_suspend(cam_ctx);
		disable_irq(IRQ_CAMERA);
		if (cam_ctx->i2c_inited)
			cam_ctx->i2c_inited = 0;
		return 0;
	}

	/* cut down power supply */
	if(cam_ctx->camera_functions->v_power > 0){
		pxa3xx_pmic_set_voltage(VCC_CAMERA_ANA, 0);
	}

	if(cam_ctx->camera_functions->v_io > 0){
		pxa3xx_pmic_set_voltage(VCC_CAMERA_IO, 0);
	}
	
	/* enable QCI clock  */
	clk_disable(cam_ctx->clk);

	return -1;
}

/*
 * Resume the Camera Module.
 */
static int pxa_camera_resume(struct platform_device *pdev)
{
	p_camera_context_t cam_ctx = g_camera_context;
	struct buf_node *buf_node;
	unsigned long flags;

	cam_ctx->suspended = 0;

	if (!(cam_ctx->driver_opened)) {
		if (!cam_ctx->i2c_inited) {
			cam_ctx->i2c_inited = 1;
		}
		return 0;
	}

	if(cam_ctx->camera_functions->v_power > 0){
		pxa3xx_pmic_set_voltage(VCC_CAMERA_ANA, cam_ctx->camera_functions->v_power);
	}

	if(cam_ctx->camera_functions->v_io > 0){
		pxa3xx_pmic_set_voltage(VCC_CAMERA_IO, cam_ctx->camera_functions->v_io);
	}

	/* enable QCI clock  */
	clk_enable(cam_ctx->clk);

	spin_lock_irqsave(&report_list_lock, flags);
	/* Empty report head and put its buf into head queue */
	while (!list_empty(cam_ctx->p_report_head)) {
		printk(KERN_DEBUG " report head isn't empty\n");
		buf_node =
			list_entry(cam_ctx->p_report_head->next,
					struct buf_node, report_head);
		list_del_init(&buf_node->report_head);
		if (camera_submit_buffer(cam_ctx, buf_node->buf_index)) {
			spin_unlock_irqrestore(&report_list_lock, flags);
			return -EIO;
		}
	}
	spin_unlock_irqrestore(&report_list_lock, flags);

	mcam_resume(cam_ctx);

	if (V4L2_PIX_FMT_JPEG == cam_ctx->capture_input_format)
		ci_set_interrupt_mask(JPG_INT_MASK);
	else
		ci_set_interrupt_mask(PRE_INT_MASK);

	enable_irq(IRQ_CAMERA);

	if (cam_ctx->dma_running) {
		spin_lock_irqsave(&report_list_lock, flags);
		if (list_empty(cam_ctx->p_report_head)) {
			cam_ctx->task_waiting = 1;
			spin_unlock_irqrestore(&report_list_lock, flags);
			wait_event_interruptible(cam_ctx->camera_wait_q,
					!list_empty
					(cam_ctx->p_report_head));
		} else
			spin_unlock_irqrestore(&report_list_lock, flags);
	}
	return 0;
}
#endif

#ifdef CAM_DEBUG_STATIC

static int jpg_measure_show(struct seq_file *s, void *unused)
{
	p_camera_context_t cam_ctx = g_camera_context;
	
	seq_printf(s, "camera jpeg capture delete frames: %d\n", 
			cam_ctx->jpg_error);

	return 0;
}

static int jpg_measure_open(struct inode *inode, struct file *file)
{
	return single_open(file, jpg_measure_show, NULL);
}

static const struct file_operations static_op = {
	.open		= jpg_measure_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

static int pxa_camera_minor = 0;

static char*sensor_reg_usage= "echo \"w 0xfc(reg) 0x0(value)\" or \"r 0xfc(reg)\"";
#define PRINT_SENSOR_REG_USAGE printk("%s \n", sensor_reg_usage)
static ssize_t sensor_reg_proc_read(struct file *filp,
		char *buffer, size_t length, loff_t *offset)
{
	PRINT_SENSOR_REG_USAGE;
	return 0;
}

static ssize_t sensor_reg_proc_write(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[30];
	char *endp;
	char* pMsg;
	unsigned int reg,value;
	char bWriteReg;
	
	if (len > 30) len = 30;
	
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	pMsg = messages;
	messages[len] = 0x0;

	if(messages[1]!=' '){
		goto fail;
	}

	pMsg+=2;//Point to index
	if(messages[0]=='w'||messages[0]=='W'){
		bWriteReg = 1;		
	}else if(messages[0]=='r'||messages[0]=='R'){
		bWriteReg = 0;	
	}else{
		goto fail;
	}

	reg = simple_strtoul( pMsg, &endp, 0);

	if(bWriteReg){
		pMsg = endp+1;		
		value = simple_strtoul( pMsg, &endp, 0);
		if(g_camera_context->camera_functions->read_reg( g_camera_context,reg,value)>=0)
			printk("Write sensor reg[0x%x] to value[0x%x]\n",reg,value);
	}else{
		if(g_camera_context->camera_functions->write_reg( g_camera_context,reg,&value)>=0)
			printk("Read sensor reg[0x%x],value[0x%x]\n",reg,value);
	}

	return len;
fail:
	PRINT_SENSOR_REG_USAGE;
	return -EFAULT;	
}

static struct file_operations sensor_reg_proc_ops = {
	.read = sensor_reg_proc_read,
	.write = sensor_reg_proc_write,
};

static int pxa_camera_probe(struct platform_device *pdev)
{
	struct resource *res = NULL;
	void __iomem *reg_base = NULL;

	p_camera_context_t cam_ctx = NULL;

	/* allocte camera context */
	g_camera_context = kzalloc(sizeof(camera_context_t), GFP_KERNEL);
	if (!g_camera_context) {
		dbg("Can't allocate buffer for" "camera control structure \n");
		return -ENOMEM;
	}
	cam_ctx = g_camera_context;

	cam_ctx->platform_ops = pdev->dev.platform_data;
	if (cam_ctx->platform_ops == NULL) {
		printk("camera no platform data defined\n");
		return -ENODEV;
	}
	cam_ctx->irq = -1;
	if(cam_ctx->platform_ops->vsync_gpio > 0)
		cam_ctx->irq = gpio_to_irq(mfp_to_gpio(cam_ctx->platform_ops->vsync_gpio));

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk("no IO memory resource defined\n");
		return -ENODEV;
	}

	reg_base = ioremap(res->start, res->end - res->start + 1);
	if (NULL == reg_base) {
		printk("ioremap() failed\n");
		return -ENODEV;
	}
	
	ci_set_reg_base(reg_base, res->start);

	cam_ctx->ci_reg_base = (unsigned int)(res->start);
	cam_ctx->ci_reg_end = (unsigned int)(res->end);

	cam_ctx->i2c_inited = 1;
	cam_ctx->suspended = 0;

	cam_ctx->camera_functions = NULL;
	
	cam_ctx->sensor_func[SENSOR_LOW] = NULL;
	cam_ctx->sensor_func[SENSOR_HIGH] = NULL;	

#if 0
	cam_ctx->cam_wq = create_workqueue("camera_task");
	if (!cam_ctx->cam_wq)
		goto malloc_camera_functions_err;
#endif

/* 	INIT_WORK(&(cam_ctx->work), cam_work_handler); */

	if (video_register_device(&vd, VFL_TYPE_GRABBER, pxa_camera_minor) < 0) {
		printk(KERN_ALERT "PXA_CAMERA: video_register_device failed\n");
		goto register_video_error;
	} else {
		printk(KERN_ALERT "PXA_CAMERA: PXA Camera driver loaded for"
				"/dev/video%d \n", pxa_camera_minor);
	}

	cam_ctx->driver_opened = 0;
	/* request irq */
	if (request_irq(IRQ_CAMERA, pxa_camera_irq, IRQF_DISABLED, "PXA Camera", &vd)) {
		printk(KERN_ALERT
				"PXA_CAMERA: Camera interrupt register failed \n");
		goto register_video_error;
	}
	disable_irq(IRQ_CAMERA);

#ifdef	RESERVE_DMA_BUFFER
	g_order = get_order(JPG_DMA_BUF_SIZE + PRE_DMA_BUF_SIZE);
	g_dma_page = alloc_pages(GFP_KERNEL, g_order);

	if (!g_dma_page) {
		printk(KERN_ALERT "Can reserve 2M DMA buffer for Camera\n");
		free_irq(IRQ_CAMERA, &vd);
		goto register_video_error;
	}
	split_page(g_dma_page, g_order);

	spin_lock_init(&dma_buf_lock);

	/*
	 * 2M buffer is split to first 1M buffer which is for preview.
	 * And the second 1M buffer is for jpeg capture.
	 */
	g_dma_free_start = page_address(g_dma_page);
	g_dma_free_size = PRE_DMA_BUF_SIZE;
	g_dma_freepage_start = g_dma_page;

	g_dma_jpeg_page = g_dma_page + (PRE_DMA_BUF_SIZE) / PAGE_SIZE;
	g_dma_jpeg_size = JPG_DMA_BUF_SIZE;
#endif

#ifdef CAM_DEBUG_STATIC
	cam_ctx->entry =  debugfs_create_file("camera",
			S_IFREG | S_IRUGO, NULL, NULL, &static_op);

	if (!cam_ctx->entry)
		printk(KERN_ALERT "create debugfs entry failed\n");
#endif

	/* Creat a reg read-write proc */
	{
		struct proc_dir_entry *entry;
		entry = create_proc_entry( "sensor_reg", 0, NULL);
		if (entry) {
			entry->proc_fops = &sensor_reg_proc_ops;
		} 
	}

	return 0;

register_video_error:
	if (cam_ctx->camera_functions) {
		kfree(cam_ctx->camera_functions);
	}

malloc_camera_functions_err:
	
	if (cam_ctx->cam_wq){
		destroy_workqueue(cam_ctx->cam_wq);
		cam_ctx->cam_wq = NULL;
	}
	
	if (cam_ctx) {
		kfree(cam_ctx);
		cam_ctx = NULL;
	}
	return -EIO;
}

static int pxa_camera_remove(struct platform_device *pdev)
{
	p_camera_context_t cam_ctx = g_camera_context;
	
	cam_ctx->i2c_inited = 0;

	if (cam_ctx->cam_wq){
		destroy_workqueue(cam_ctx->cam_wq);
		cam_ctx->cam_wq = NULL;
	}

	if (cam_ctx) {
		kfree(cam_ctx);
		cam_ctx = NULL;
	}

	free_irq(IRQ_CAMERA, &vd);
	video_unregister_device(&vd);

#ifdef	RESERVE_DMA_BUFFER
	{
		int page_nums = 1 << g_order;
		struct page *page = g_dma_page;
		int i;
		
		for (i = 0; i < page_nums; i++) {
			free_page(page);
			page++;
		}
	}
#endif

#ifdef CAM_DEBUG_STATIC

	if (!cam_ctx->entry)
		debugfs_remove(cam_ctx->entry);
	
#endif


	printk(KERN_ALERT "PXA_CAMERA: PXA Camera driver unloaded.\n");
	return 0;
}

static struct platform_driver pxa_camera_driver = {
	.driver = {
		.name = "pxa3xx-camera"
	},
	.probe 		= pxa_camera_probe,
	.remove 	= pxa_camera_remove,
#ifdef CONFIG_PM
	.suspend 	= pxa_camera_suspend,
	.resume 	= pxa_camera_resume,
#endif
};

static int __devinit pxa_camera_init(void)
{	
#ifdef CONFIG_PXA3xx_DVFM
	dvfm_register("Camera", &dvfm_dev_idx);
#endif
	return platform_driver_register(&pxa_camera_driver);
}

static void __exit pxa_camera_exit(void)
{	
	platform_driver_unregister(&pxa_camera_driver);
#ifdef CONFIG_PXA3xx_DVFM
	dvfm_unregister("Camera", &dvfm_dev_idx);
#endif
}

module_init(pxa_camera_init);
module_exit(pxa_camera_exit);

MODULE_DESCRIPTION("Zylonite/PXA3xx Camera Interface driver");
MODULE_LICENSE("GPL");
