/*
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
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/mm.h>

#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <mach/pxa-regs.h>
#include <mach/mfp.h>

#include "pxafb.h"

#define MINILCD_SRAM_ADDR	(0x5C011000)
#define MINILCD_REG_ADDR	(0x46000000)
#define MLCCR0                  0x0 
#define MLCCR1                  0x4
#define MLCCR2                  0x8
#define MLSADD                  0xc
#define MLFRMCNT                0x10

#define MLCCR0_OEP              (1 << 11)
#define MLCCR0_PCP              (1 << 10)
#define MLCCR0_VSP              (1 << 9)
#define MLCCR0_HSP              (1 << 8)
#define MLCCR0_PCD(d)           ((d) & 0xff)

#define MLCCR1_BLW(n)           (((n) & 0xff) << 24)
#define MLCCR1_ELW(n)           (((n) & 0xff) << 16)
#define MLCCR1_HSW(n)           (((n) & 0x3f) << 10)
#define MLCCR1_PPL(n)           (((n) & 0x3ff)

#define MLCCR2_BFW(n)           (((n) & 0xff) << 24)
#define MLCCR2_EFW(n)           (((n) & 0xff) << 16)
#define MLCCR2_VSW(n)           (((n) & 0x3f) << 10)
#define MLCCR2_LPP(n)           (((n) & 0x3ff)

#define MLFRMCNT_WKUP           (1U << 31)
#define MLFRMCNT_FWKUP          (1U << 30)
#define MLFRMCNT_FRCOUNT(n)     ((n) & 0x3ff)
#define MLFRMCNT_FRCOUNT_MASK   (0x3ff)


extern void pxa3xx_enable_mlcd_pins(void);
extern void pxa3xx_enable_lcd_pins(void);
static void pxafb_minilcd_enable (struct pxafb_info *fbi);
static void pxafb_minilcd_disable(struct pxafb_info *fbi);
static int  pxafb_minilcd_encode(void *dst, void *src, int width, int height);

static struct pxafb_info *g_pxafb_info = NULL;
static unsigned char __iomem	*minilcd_base;
static struct clk *minilcd_clk;

#define minilcd_writel(value, reg) writel((value), minilcd_base + (reg));
#define minilcd_readl(reg) readl(minilcd_base + (reg));

int pxafb_minilcd_register(struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	struct pxafb_minilcd_info *mi;

	if ((g_pxafb_info = fbi) == NULL)
		return -EINVAL;

	mi = &fbi->minilcd_info;

	mi->enable = 1;		/* default to be enabled */
	mi->backlight = 1;	/* default to turn on backlight */
	mi->framecount = 0;	/* default to no framecount wakeup */
	mi->framedata  = NULL;	/* default to base frame buffer */

	minilcd_base = (void *)ioremap(MINILCD_REG_ADDR, 0x1000);
	minilcd_clk = clk_get(NULL, "MINLCDCLK");

	return 0;
}

int pxafb_minilcd_ioctl(struct fb_info *info, unsigned int cmd,
			unsigned long arg)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	struct pxafb_minilcd_info *mi;

	if (fbi == NULL)
		return -ENODEV;

	mi = &fbi->minilcd_info;

	switch (cmd) {
	case PXAFB_MINILCD_ENABLE:
		mi->enable = (uint32_t)(arg);
		break;

	case PXAFB_MINILCD_BACKLIGHT:
		mi->backlight = (uint32_t)(arg);
		break;

	case PXAFB_MINILCD_WAKEUP:
		if (arg == 0) {
			mi->framecount &=
				~(MLFRMCNT_WKUP | MLFRMCNT_FRCOUNT_MASK);
		} else {
			mi->framecount |= MLFRMCNT_FRCOUNT(arg);
			mi->framecount |= MLFRMCNT_WKUP;
		}
		break;

	case PXAFB_MINILCD_FWAKEUP:
		if (arg == 0) {
			mi->framecount &= ~(MLFRMCNT_FWKUP);
		} else {
			mi->framecount |= MLFRMCNT_FWKUP;
		}
		break;

	case PXAFB_MINILCD_FRAMEDATA:
	{
		unsigned int size;
	       
		size = fbi->fb.var.xres * fbi->fb.var.yres;
		size = (size * fbi->fb.var.bits_per_pixel) >> 3;

		if (arg == 0) {
			if (mi->framedata)
				vfree(mi->framedata);

			mi->framedata = NULL;
		}else{
			if (mi->framedata == NULL) {
				mi->framedata = vmalloc(size);
				if (mi->framedata == NULL){
					return -ENOMEM;
				}
			}
			if (copy_from_user(mi->framedata, (void *)arg, size))
				return -EFAULT;
		}
		break;
	}
	default:
		return -EINVAL;
	}

	return 0;
}
/* pxafb_minilcd_enter
 *
 * @brief this function will be called by power management routines when
 * entering into S0/D1/C2 (standby with LCD refreshing) mode.
 *
 * SRAM allocation is currently forced. Memory from MINILCD_SRAM_ADDR of
 * adequate size will be backed up into DRAM, and be restored later when
 * exiting from D1.
 */
int pxafb_minilcd_enter( void )
{
	void  *frame_src;
	size_t sram_size;
	struct pxafb_info *fbi;
	struct pxafb_minilcd_info *mi;

	if ((fbi = g_pxafb_info) == NULL)
		return -ENODEV;

	mi = &fbi->minilcd_info;

	if (!mi->enable)
		return 0;

	if ((fbi->overlay1fb->state == C_ENABLE) ||
	    (fbi->overlay2fb->state == C_ENABLE))
		return -EBUSY;

	if (fbi->fb.var.bits_per_pixel != 16)
		return -EINVAL;

	sram_size = (fbi->fb.var.xres + 1) * fbi->fb.var.yres * 2;
	sram_size = PAGE_ALIGN(sram_size);

	mi->sram_addr_virt = (void *)ioremap(MINILCD_SRAM_ADDR, sram_size);
	mi->sram_addr_phys = MINILCD_SRAM_ADDR;

	mi->sram_save_to   = vmalloc(sram_size);
	if (mi->sram_save_to == NULL) {
		iounmap(mi->sram_addr_virt);
		return -ENOMEM;
	}
	memcpy(mi->sram_save_to, mi->sram_addr_virt, sram_size);
	mi->sram_save_size = sram_size;

	frame_src = (mi->framedata) ? mi->framedata : fbi->fb.screen_base;

	if (pxafb_minilcd_encode(mi->sram_addr_virt, frame_src,
			fbi->fb.var.xres,fbi->fb.var.yres) < 0)
		return -EINVAL;

	pxafb_minilcd_enable(fbi);

	return 0;
}

EXPORT_SYMBOL(pxafb_minilcd_enter);

int pxafb_minilcd_exit( void )
{
	struct pxafb_info *fbi;
	struct pxafb_minilcd_info *mi;

	if ((fbi = g_pxafb_info) == NULL)
		return -ENODEV;

	mi = &fbi->minilcd_info;

	if (!mi->enable)
		return 0;

	pxafb_minilcd_disable(fbi);

	/* restore saved SRAM */
	if (mi->sram_save_to) {
		memcpy( mi->sram_addr_virt,
			mi->sram_save_to,
			mi->sram_save_size);

		iounmap(mi->sram_addr_virt);
		vfree(mi->sram_save_to);

		mi->sram_addr_phys = 0;
		mi->sram_addr_virt = NULL;
		mi->sram_save_to   = NULL;
		mi->sram_save_size = 0;
	}

	return 0;
}

EXPORT_SYMBOL(pxafb_minilcd_exit);

static void pxafb_minilcd_enable(struct pxafb_info *fbi)
{
	struct pxafb_minilcd_info *mi = &fbi->minilcd_info;
	//unsigned long pixclock = fbi->fb.var.pixclock;
	uint32_t mlccr0 = 0;

	clk_enable(minilcd_clk);

	/* FIXME:  hack the timing information from fbi->reg_lccrX
	 * instead of from platform specific struct pxafb_mach_info
	 */
	if (fbi->reg_lccr3 & LCCR3_OEP) mlccr0 |= MLCCR0_OEP;
	if (fbi->reg_lccr3 & LCCR3_PCP) mlccr0 |= MLCCR0_PCP;
	if (fbi->reg_lccr3 & LCCR3_HSP) mlccr0 |= MLCCR0_HSP;
	if (fbi->reg_lccr3 & LCCR3_VSP) mlccr0 |= MLCCR0_VSP;

	mlccr0 |= 0x04;// MLCCR0_PCD(pixclock * 39ul / 1000000ul - 1);

	minilcd_writel(mlccr0, MLCCR0);
	minilcd_writel(fbi->reg_lccr1,MLCCR1); 
	minilcd_writel(fbi->reg_lccr2, MLCCR2);
	minilcd_writel(mi->sram_addr_phys, MLSADD);
	minilcd_writel(mi->framecount, MLFRMCNT);

	pxa3xx_enable_mlcd_pins();

	return;
}

static void pxafb_minilcd_disable(struct pxafb_info *fbi)
{
	/* restore MFP pin settings back to LCD */
	pxa3xx_enable_lcd_pins();
	clk_disable(minilcd_clk);
}

#define rgb16to15(n) (uint16_t)(((n) & 0x1f) | (((n) & 0xffc0) >> 1))
/*
 * pxafb_minilcd_encode
 *
 * @brief this function encodes the pixels (RGB565) in the source buffers
 * "src", with the geometry of width x height, into the destination "dst"
 * (RGB555). The pixels will be run-length encoded. It returns the number
 * of the encoded bytes if successful, or a negative number if failed.
 *
 * Note: this function assumes the destination buffer is large enough.
 * This is usually true because the SRAM will only used by Mini-LCD in
 * D1 power mode.
 */
static int pxafb_minilcd_encode(void *dst, void *src, int width, int height)
{
	uint16_t last, curr;
	uint16_t *s = src;
	uint16_t *d = dst;
	int i, j, count;

	if ((src == NULL) || (dst == NULL))
		return -EINVAL;

	for (i = 0; i < height; i++) {
		curr = rgb16to15(*s); s++;
		*d++ = curr;
		last  = curr;
		count = 0;
		for (j = 1; j < width - 1; j++) {
			curr = rgb16to15(*s); s++;
			if (curr == last)
				count++;
			else {
				if (count) {
					*d++ = 0x8000 | count;
					count = 0;
				}
				*d++ = curr;
				last = curr;
			}
		}
		if (count)
			*d++ = 0x8000 | count;
		*d++ = rgb16to15(*s); s++;
		*d++ = 0x9001;	/* EOL */
	}
	*(d - 1) = 0xA001;	/* replace last EOL with EOF */

	while ((unsigned int)(d) & 0x7)
		*d++ = 0xF001;	/* padding to 64-bit aligned */

	return ((unsigned int)(d) - (unsigned int)(dst));
}

