/*
 *  linux/drivers/video/pxafb.c
 *
 *  Copyright (C) 1999 Eric A. Thomas.
 *  Copyright (C) 2004 Jean-Frederic Clere.
 *  Copyright (C) 2004 Ian Campbell.
 *  Copyright (C) 2004 Jeff Lackey.
 *   Based on sa1100fb.c Copyright (C) 1999 Eric A. Thomas
 *  which in turn is
 *   Based on acornfb.c Copyright (C) Russell King.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 *	        Intel PXA250/210 LCD Controller Frame Buffer Driver
 *
 * Please direct your questions and comments on this driver to the following
 * email address:
 *
 *	linux-arm-kernel@lists.arm.linux.org.uk
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/div64.h>
#include <mach/pxa-regs.h>
#include <mach/bitfield.h>
#include <mach/pxafb.h>
#include <mach/pxa2xx_v4l2ov2.h>
#ifdef CONFIG_PXA3xx_DVFM
#include <mach/dvfm.h>
#include <mach/pxa3xx_dvfm.h>
#endif

#include <mach/pxa3xx-regs.h>

/*
 * Complain if VAR is out of range.
 */
#define DEBUG_VAR 0

#include "pxafb.h"

/* Bits which should not be set in machine configuration structures */
#define LCCR0_INVALID_CONFIG_MASK (LCCR0_OUM|LCCR0_BM|LCCR0_QDM|LCCR0_DIS|LCCR0_EFM|LCCR0_IUM|LCCR0_SFM|LCCR0_LDM|LCCR0_ENB)
#define LCCR3_INVALID_CONFIG_MASK (LCCR3_HSP|LCCR3_VSP|LCCR3_PCD|LCCR3_BPP)

#define ENABLE_PXAFB_PCDDIV (0x1)

static void (*pxafb_backlight_power)(int);
static void (*pxafb_lcd_power)(int, struct fb_var_screeninfo *);
static int pxafb_page_fliping = 0;

static int pxafb_activate_var(struct fb_var_screeninfo *var, struct pxafb_info *);
void set_ctrlr_state(struct pxafb_info *fbi, u_int state);
extern int is_android(void);

#ifdef CONFIG_FB_PXA_PARAMETERS
#define PXAFB_OPTIONS_SIZE 256
static char g_options[PXAFB_OPTIONS_SIZE] __devinitdata = "";
#endif

static unsigned int lcd_update = 1;

#ifdef CONFIG_PXA3xx_DVFM
static int fb_notifier_freq(struct notifier_block *nb,
		unsigned long val, void *data);
static struct notifier_block notifier_freq_block = {
	.notifier_call = fb_notifier_freq,
};

static void *dev_id = NULL;

static int hss = 0;
static int pxafb_adjust_pcd(struct pxafb_info *fbi, int hss);
#endif

unsigned int __smart_panel_timing(struct pxafb_info *fbi, 
		unsigned long long lclk, int time) 
{
	unsigned long long value;
	unsigned long long remainder;

	value = lclk*time;
	remainder = do_div(value, 100000);
	if (remainder > 50000)
		value += 1;
	return (unsigned int) value;
}

unsigned int smart_panel_timing(struct pxafb_info *fbi, int time) 
{
	unsigned long long lclk;

	lclk = clk_get_rate(fbi->clk)/10000;/* in 10KHZ */
	return __smart_panel_timing(fbi, lclk, time);
}

static inline void pxafb_schedule_work(struct pxafb_info *fbi, u_int state)
{
	unsigned long flags;

	local_irq_save(flags);
	/*
	 * We need to handle two requests being made at the same time.
	 * There are two important cases:
	 *  1. When we are changing VT (C_REENABLE) while unblanking (C_ENABLE)
	 *     We must perform the unblanking, which will do our REENABLE for us.
	 *  2. When we are blanking, but immediately unblank before we have
	 *     blanked.  We do the "REENABLE" thing here as well, just to be sure.
	 */
	if (fbi->task_state == C_ENABLE && state == C_REENABLE)
		state = (u_int) -1;
	if (fbi->task_state == C_DISABLE && state == C_ENABLE)
		state = C_REENABLE;

	if (state != (u_int)-1) {
		fbi->task_state = state;
		schedule_work(&fbi->task);
	}
	local_irq_restore(flags);
}

static inline u_int chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int
pxafb_setpalettereg(u_int regno, u_int red, u_int green, u_int blue,
		       u_int trans, struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	u_int val;

	if (regno >= fbi->palette_size)
		return 1;

	if (fbi->fb.var.grayscale) {
		fbi->palette_cpu[regno] = ((blue >> 8) & 0x00ff);
		return 0;
	}

	switch (fbi->lccr4 & LCCR4_PAL_FOR_MASK) {
	case LCCR4_PAL_FOR_0:
		val  = ((red   >>  0) & 0xf800);
		val |= ((green >>  5) & 0x07e0);
		val |= ((blue  >> 11) & 0x001f);
		fbi->palette_cpu[regno] = val;
		break;
	case LCCR4_PAL_FOR_1:
		val  = ((red   << 8) & 0x00f80000);
		val |= ((green >> 0) & 0x0000fc00);
		val |= ((blue  >> 8) & 0x000000f8);
		((u32*)(fbi->palette_cpu))[regno] = val;
		break;
	case LCCR4_PAL_FOR_2:
		val  = ((red   << 8) & 0x00fc0000);
		val |= ((green >> 0) & 0x0000fc00);
		val |= ((blue  >> 8) & 0x000000fc);
		((u32*)(fbi->palette_cpu))[regno] = val;
		break;
	}

	return 0;
}

static int
pxafb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
		   u_int trans, struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	unsigned int val;
	int ret = 1;

	/*
	 * If inverse mode was selected, invert all the colours
	 * rather than the register number.  The register number
	 * is what you poke into the framebuffer to produce the
	 * colour you requested.
	 */
	if (fbi->cmap_inverse) {
		red   = 0xffff - red;
		green = 0xffff - green;
		blue  = 0xffff - blue;
	}

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (fbi->fb.var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
					7471 * blue) >> 16;

	switch (fbi->fb.fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = fbi->fb.pseudo_palette;

			val  = chan_to_field(red, &fbi->fb.var.red);
			val |= chan_to_field(green, &fbi->fb.var.green);
			val |= chan_to_field(blue, &fbi->fb.var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		ret = pxafb_setpalettereg(regno, red, green, blue, trans, info);
		break;
	}

	return ret;
}

/*
 *  pxafb_bpp_to_lccr3():
 *    Convert a bits per pixel value to the correct bit pattern for LCCR3
 */
static int pxafb_bpp_to_lccr3(struct fb_var_screeninfo *var)
{
	int bpp = 0;
	int depth = 0;
	u_int lccr3;

	depth = var->red.length + var->green.length + var->blue.length + var->transp.length;
	switch (var->bits_per_pixel) {
	case 1:  bpp = LCCR3_1BPP; break;
	case 2:  bpp = LCCR3_2BPP; break;
	case 4:  bpp = LCCR3_4BPP; break;
	case 8:  bpp = LCCR3_8BPP; break;
	case 16: bpp = LCCR3_16BPP; break;
	case 24:
		switch (depth) {
		case 18:
			switch (var->blue.offset) {
			case 0: bpp = LCCR3_18BPP_PACK; break; /* 18-bits/pixel packed */
			case 2: bpp = LCCR3_18BPP_PACK_8; break; /* 18-bits/pixel packed in (6R+2'0s, 6G+2'0s, 6B+2'0s) format */
			}
		case 19: bpp = LCCR3_19BPP_PACK; break; /* 19-bits/pixel packed */
		}
		break;
	case 32:
		switch (depth) {
		case 18:
			switch (var->blue.offset) {
			case 0: bpp = LCCR3_18BPP_UNPACK; break; /* 18-bits/pixel unpacked */
			case 2: bpp = LCCR3_18BPP_UNPACK_8; break; /* 18-bits/pixel unpacked in (6R+2'0s, 6G+2'0s, 6B+2'0s) format */
			}
			break;
		case 19:
			switch (var->blue.offset) {
			case 0: bpp = LCCR3_19BPP_UNPACK; break; /* 19-bits/pixel unpacked */
			case 2: bpp = LCCR3_19BPP_UNPACK_8; break; /* 19-bits/pixel unpacked in (6R+2'0s, 6G+2'0s, 6B+2'0s) format */
			}
			break;
		case 24: bpp = LCCR3_24BPP; break; /* 24-bits/pixel for smart panel */
		case 25: bpp = LCCR3_25BPP; break; /* 25-bits/pixel for smart panel */
		case 32: bpp = LCCR3_19BPP_UNPACK_8; break; /* faked RGBA888 since silicon don't support. used LCCR3_19BPP_UNPACK_8 instead */
		}
		break;
	}
	lccr3 = LCCR3_Bpp(bpp);
	switch (depth) {
	case 16: lccr3 |= var->transp.length ? LCCR3_PDFOR_3 : 0; break;
	case 18: lccr3 |= LCCR3_PDFOR_3; break;
	case 24: lccr3 |= var->transp.length ? LCCR3_PDFOR_2 : LCCR3_PDFOR_3; break;
	case 19:
	case 25: lccr3 |= LCCR3_PDFOR_0; break;
	}
	return lccr3;
}

#ifdef CONFIG_CPU_FREQ
/*
 *  pxafb_display_dma_period()
 *    Calculate the minimum period (in picoseconds) between two DMA
 *    requests for the LCD controller.  If we hit this, it means we're
 *    doing nothing but LCD DMA.
 */
static unsigned int pxafb_display_dma_period(struct fb_var_screeninfo *var)
{
       /*
        * Period = pixclock * bits_per_byte * bytes_per_transfer
        *              / memory_bits_per_pixel;
        */
       return var->pixclock * 8 * 16 / var->bits_per_pixel;
}

extern unsigned int get_clk_frequency_khz(int info);
#endif

/*
 * Select the smallest mode that allows the desired resolution to be
 * displayed. If desired parameters can be rounded up.
 */
static struct pxafb_mode_info *pxafb_getmode(struct pxafb_mach_info *mach, struct fb_var_screeninfo *var)
{
	struct pxafb_mode_info *mode = NULL;
	struct pxafb_mode_info *modelist = mach->modes;
	unsigned int best_x = 0xffffffff, best_y = 0xffffffff;
	unsigned int i;

	for (i = 0 ; i < mach->num_modes ; i++) {
		if (modelist[i].xres >= var->xres && modelist[i].yres >= var->yres &&
				modelist[i].xres < best_x && modelist[i].yres < best_y) {
			best_x = modelist[i].xres;
			best_y = modelist[i].yres;
			mode = &modelist[i];
		}
	}

	return mode;
}

static void pxafb_setmode(struct fb_var_screeninfo *var, struct pxafb_mode_info *mode)
{
	var->xres		= mode->xres;
	var->yres		= mode->yres;
	var->bits_per_pixel	= mode->bpp;
	var->pixclock		= mode->pixclock;
	var->hsync_len		= mode->hsync_len;
	var->left_margin	= mode->left_margin;
	var->right_margin	= mode->right_margin;
	var->vsync_len		= mode->vsync_len;
	var->upper_margin	= mode->upper_margin;
	var->lower_margin	= mode->lower_margin;
	var->sync		= mode->sync;
	var->grayscale		= mode->cmap_greyscale;
	var->xres_virtual 	= var->xres;
	var->vmode	= FB_VMODE_NONINTERLACED;

	if (is_android())
		var->yres_virtual	= var->yres * 2;
	else 
		var->yres_virtual	= var->yres;
}

/*
 *  pxafb_check_var():
 *    Get the video params out of 'var'. If a value doesn't fit, round it up,
 *    if it's too big, return -EINVAL.
 *
 *    Round up in the following order: bits_per_pixel, xres,
 *    yres, xres_virtual, yres_virtual, xoffset, yoffset, grayscale,
 *    bitfields, horizontal timing, vertical timing.
 */
static int pxafb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	struct pxafb_mach_info *inf = fbi->dev->platform_data;

	if (var->xres < MIN_XRES)
		var->xres = MIN_XRES;
	if (var->yres < MIN_YRES)
		var->yres = MIN_YRES;

	if (inf->fixed_modes) {
		struct pxafb_mode_info *mode;
		struct pxafb_mode_info mode2;

		mode = pxafb_getmode(inf, var);
		memcpy(&mode2, mode, sizeof(struct pxafb_mode_info));
		mode = &mode2;
		mode->bpp = var->bits_per_pixel;
		if (!mode)
			return -EINVAL;
		pxafb_setmode(var, mode);
		fbi->wr_setup_time = mode->wr_setup_time;
		fbi->wr_pulse_width = mode->wr_pulse_width;
		fbi->rd_setup_time = mode->rd_setup_time;
		fbi->rd_pulse_width = mode->rd_pulse_width;
		fbi->op_hold_time = mode->op_hold_time;
		fbi->cmd_inh_time = mode->cmd_inh_time;
		fbi->sync_cnt = mode->sync_cnt;
		fbi->update_framedata = mode->update_framedata;
	} else {
		if (var->xres > inf->modes->xres)
			return -EINVAL;
		if (var->yres > inf->modes->yres)
			return -EINVAL;
		if ((var->bits_per_pixel != 1) &&
			(var->bits_per_pixel != 2) &&
			(var->bits_per_pixel != 4) &&
			(var->bits_per_pixel != 8) &&
			(var->bits_per_pixel != 16) &&
			(var->bits_per_pixel != 18) &&
			(var->bits_per_pixel != 19) &&
			(var->bits_per_pixel != 24) &&
			(var->bits_per_pixel != 25) &&
			(var->bits_per_pixel != 32))
			return -EINVAL;
	}

	var->xres_virtual =
		max(var->xres_virtual, var->xres);
	var->yres_virtual =
		max(var->yres_virtual, var->yres);

        /*
	 * FIXME: Need to check and correct the RGB parameters for this display.
	 *
	 * The pixel packing format is described on page 7-11 of the
	 * PXA2XX Developer's Manual.
         */         

#ifdef CONFIG_CPU_FREQ
	pr_debug("pxafb: dma period = %d ps, clock = %d kHz\n",
		 pxafb_display_dma_period(var),
		 get_clk_frequency_khz(0));
#endif

	return 0;
}

static int pxafb_set_pixfmt(struct fb_var_screeninfo *var, int depth)
{
	if (depth == 0)
		depth = var->bits_per_pixel;
	switch (depth) {
	case 16:			/* set as rgb565 by default */
		var->red.offset   = 11; var->red.length   = 5;
		var->green.offset = 5;  var->green.length = 6;
		var->blue.offset  = 0;  var->blue.length  = 5;
		var->transp.offset = var->transp.length = 0;
		break;
	case 18:			/* set as 18bpp unpacked in (6R+2'0s, 6G+2'0s, 6B+2'0s) format by default */
		var->red.offset   = 18; var->red.length   = 6;
		var->green.offset = 10;	var->green.length = 6;
		var->blue.offset  = 2;	var->blue.length  = 6;
		var->transp.offset = var->transp.length = 0;
		break;
	case 19:			/* set as 19bpp unpacked in (6R+2'0s, 6G+2'0s, 6B+2'0s) format by default */
		var->red.offset   = 18; var->red.length   = 6;
		var->green.offset = 10;	var->green.length = 6;
		var->blue.offset  = 2;	var->blue.length  = 6;
		var->transp.offset = 24;
		var->transp.length = 1;
		break;
	case 24:
		var->red.offset   = 16; var->red.length   = 8;
		var->green.offset = 8;	var->green.length = 8;
		var->blue.offset  = 0;	var->blue.length  = 8;
		var->transp.offset = var->transp.length = 0;
		break;
	case 25:
		var->red.offset   = 16; var->red.length   = 8;
		var->green.offset = 8;	var->green.length = 8;
		var->blue.offset  = 0;	var->blue.length  = 8;
		var->transp.offset = 24;
		var->transp.length = 1;
		break;
	case 32:
		var->red.offset   = 16; var->red.length   = 8;
		var->green.offset = 8;	var->green.length = 8;
		var->blue.offset  = 0;	var->blue.length  = 8;
		var->transp.offset = 24;
		var->transp.length = 8;
		break;
	default:		
		var->red.offset = var->green.offset = var->blue.offset = var->transp.offset = 0;
		var->red.length   = 8;
		var->green.length = 8;
		var->blue.length  = 8;
		var->transp.length = 0;
	}
}

static inline void pxafb_set_truecolor(u_int is_true_color)
{
	pr_debug("pxafb: true_color = %d\n", is_true_color);
	// do your machine-specific setup if needed
}

#if defined(CONFIG_PXA3xx_DVFM_HSS_EOF_FB)
void pxafb_wait_eof(void)
{
#define MAX_TIMEOUT (200*1000)
	unsigned int lcsr;
	int i = 0;
	LCSR = LCSR_EOF;

	do {
		lcsr = LCSR;
		i++;
	} while (!(lcsr&LCSR_EOF) && i<MAX_TIMEOUT);
	
	LCSR = LCSR_EOF;
#ifdef CONFIG_FB_LCD_SHARP_WVGA
	if (i<MAX_TIMEOUT)
		udelay(6);
#endif
	//printk(KERN_DEBUG"pxafb_wait_eof, lcsr 0x%x, i %d\n", lcsr, i);
}
#endif

static int pxafb_set_par(struct fb_info *info)
{
	DECLARE_WAITQUEUE(wait, current);
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	u32 lcsr;	

	fbi->dmadesc_fbhigh_cpu->fsadr = fbi->screen_dma + 
		((fbi->fb.var.yoffset / fbi->fb.var.yres) * (fbi->fb.var.yres) * (fbi->fb.var.xres) * 2);
	//printk("db 0x%x 0x%x\n", fbi->fb.var.yoffset, fbi->dmadesc_fblow_cpu->fsadr);

	/* wait lcd frame display finish */
	set_current_state(TASK_UNINTERRUPTIBLE);
	add_wait_queue(&fbi->ctrlr_wait, &wait);	

	LCSR = 0xffffffff;	/* Clear LCD Status Register */
	LCCR0 &= ~(LCCR0_SFM);	/* Enable Start Frame Interrupt */		
	schedule_timeout(200 * HZ / 1000);
	remove_wait_queue(&fbi->ctrlr_wait, &wait);
	LCCR0 |= (LCCR0_SFM);	
	//printk("db end\n");
	return 0;
}

/*
 * pxafb_set_par():
 *	Set the user defined part of the display for the specified console
 */
static int pxafb_set_par_init(struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	struct fb_var_screeninfo *var = &info->var;
	unsigned long palette_mem_size;

	//printk("pxafb: set_par 0x%x\n", fbi->fb.var.yoffset);
	
  	if (var->bits_per_pixel == 16 || var->bits_per_pixel == 18 ||
 		var->bits_per_pixel == 19 || var->bits_per_pixel == 24 ||
 		var->bits_per_pixel == 25 || var->bits_per_pixel == 32)
		fbi->fb.fix.visual = FB_VISUAL_TRUECOLOR;
	else if (!fbi->cmap_static)
		fbi->fb.fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else {
		/*
		 * Some people have weird ideas about wanting static
		 * pseudocolor maps.  I suspect their user space
		 * applications are broken.
		 */
		fbi->fb.fix.visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
	}

	switch (var->bits_per_pixel) {
	case 16:
	case 24:
	case 32:
		fbi->fb.fix.line_length = var->xres_virtual * var->bits_per_pixel / 8;
		fbi->palette_size = 0;
		break;
	default:
		fbi->fb.fix.line_length = var->xres_virtual * var->bits_per_pixel / 8;
		fbi->palette_size = var->bits_per_pixel == 1 ? 4 : 1 << var->bits_per_pixel;
	}

	if ((fbi->lccr4 & LCCR4_PAL_FOR_MASK) == LCCR4_PAL_FOR_0)
		palette_mem_size = fbi->palette_size * sizeof(u16);
	else
		palette_mem_size = fbi->palette_size * sizeof(u32);

	pr_debug("pxafb: palette_mem_size = 0x%08lx\n", palette_mem_size);

	fbi->palette_cpu = (u16 *)(fbi->map_cpu + PAGE_SIZE - palette_mem_size);
	fbi->palette_dma = fbi->map_dma + PAGE_SIZE - palette_mem_size;

	/*
	 * Set (any) board control register to handle new color depth
	 */
	pxafb_set_truecolor(fbi->fb.fix.visual == FB_VISUAL_TRUECOLOR);

	if ((fbi->fb.var.bits_per_pixel == 16) || (fbi->fb.var.bits_per_pixel == 18) ||
	    (fbi->fb.var.bits_per_pixel == 19) || (fbi->fb.var.bits_per_pixel == 24) ||
	    (fbi->fb.var.bits_per_pixel == 25) || (fbi->fb.var.bits_per_pixel == 32))
		fb_dealloc_cmap(&fbi->fb.cmap);
	else
		fb_alloc_cmap(&fbi->fb.cmap, 1<<fbi->fb.var.bits_per_pixel, 0);

	pxafb_page_fliping = 1;
	pxafb_activate_var(var, fbi);
	pxafb_page_fliping = 0;

	return 0;
}

/*
 * Formal definition of the VESA spec:
 *  On
 *  	This refers to the state of the display when it is in full operation
 *  Stand-By
 *  	This defines an optional operating state of minimal power reduction with
 *  	the shortest recovery time
 *  Suspend
 *  	This refers to a level of power management in which substantial power
 *  	reduction is achieved by the display.  The display can have a longer
 *  	recovery time from this state than from the Stand-by state
 *  Off
 *  	This indicates that the display is consuming the lowest level of power
 *  	and is non-operational. Recovery from this state may optionally require
 *  	the user to manually power on the monitor
 *
 *  Now, the fbdev driver adds an additional state, (blank), where they
 *  turn off the video (maybe by colormap tricks), but don't mess with the
 *  video itself: think of it semantically between on and Stand-By.
 *
 *  So here's what we should do in our fbdev blank routine:
 *
 *  	VESA_NO_BLANKING (mode 0)	Video on,  front/back light on
 *  	VESA_VSYNC_SUSPEND (mode 1)  	Video on,  front/back light off
 *  	VESA_HSYNC_SUSPEND (mode 2)  	Video on,  front/back light off
 *  	VESA_POWERDOWN (mode 3)		Video off, front/back light off
 *
 *  This will match the matrox implementation.
 */

/*
 * pxafb_blank():
 *	Blank the display by setting all palette values to zero.  Note, the
 * 	16 bpp mode does not really use the palette, so this will not
 *      blank the display in all modes.
 */
int pxafb_blank(int blank, struct fb_info *info)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	int i;

	pr_debug("pxafb: blank=%d\n", blank);

	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		if (fbi->fb.fix.visual == FB_VISUAL_PSEUDOCOLOR ||
		    fbi->fb.fix.visual == FB_VISUAL_STATIC_PSEUDOCOLOR)
			for (i = 0; i < fbi->palette_size; i++)
				pxafb_setpalettereg(i, 0, 0, 0, 0, info);

		pxafb_schedule_work(fbi, C_DISABLE);
		//TODO if (pxafb_blank_helper) pxafb_blank_helper(blank);
		lcd_update = 0;
		break;

	case FB_BLANK_UNBLANK:
		lcd_update = 1;
		//TODO if (pxafb_blank_helper) pxafb_blank_helper(blank);
		if (fbi->fb.fix.visual == FB_VISUAL_PSEUDOCOLOR ||
		    fbi->fb.fix.visual == FB_VISUAL_STATIC_PSEUDOCOLOR)
			fb_set_cmap(&fbi->fb.cmap, info);
		pxafb_schedule_work(fbi, C_ENABLE);
	}
	return 0;
}

static int pxafb_mmap(struct fb_info *info,
		      struct vm_area_struct *vma)
{
	struct pxafb_info *fbi = (struct pxafb_info *)info;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;

	if (off < info->fix.smem_len) {
		vma->vm_pgoff += 1;
		return dma_mmap_writecombine(fbi->dev, vma, fbi->map_cpu,
					     fbi->map_dma, fbi->map_size);
	}
	return -EINVAL;
}

static struct fb_ops pxafb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= pxafb_check_var,
	.fb_set_par	= pxafb_set_par,
	.fb_setcolreg	= pxafb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	//.fb_blank	= pxafb_blank, /* we donnot support fb con, so disable it, by frank */
	.fb_mmap	= pxafb_mmap,
#ifdef CONFIG_FB_PXA_MINILCD
        .fb_ioctl       = pxafb_minilcd_ioctl,
#endif
};

/*
 * Calculate the PCD value from the clock rate (in picoseconds).
 * We take account of the PPCR clock setting.
 * From PXA Developer's Manual:
 *
 *   PixelClock =      LCLK
 *                -------------
 *                2 ( PCD + 1 )
 *
 *   PCD =      LCLK
 *         ------------- - 1
 *         2(PixelClock)
 *
 * Where:
 *   LCLK = LCD/Memory Clock
 *   PCD = LCCR3[7:0]
 *
 * PixelClock here is in Hz while the pixclock argument given is the
 * period in picoseconds. Hence PixelClock = 1 / ( pixclock * 10^-12 )
 *
 * The function get_lclk_frequency_10khz returns LCLK in units of
 * 10khz. Calling the result of this function lclk gives us the
 * following
 *
 *    PCD = (lclk * 10^4 ) * ( pixclock * 10^-12 )
 *          -------------------------------------- - 1
 *                          2
 *
 * Factoring the 10^4 and 10^-12 out gives 10^-8 == 1 / 100000000 as used below.
 */
static inline unsigned int __get_pcd(unsigned long long lclk, unsigned int pixclock)
{
	unsigned long long pcd;
	unsigned long long remainder;

	/* FIXME: Need to take into account Double Pixel Clock mode
         * (DPC) bit? or perhaps set it based on the various clock
         * speeds */

	pcd = (unsigned long long)lclk*pixclock;

#ifdef ENABLE_PXAFB_PCDDIV
	remainder = do_div(pcd, 100000000);
	if (remainder < 50000000)
		pcd -= 1;
#else
	if (LCCR4 & LCCR4_PCDDIV) {
		remainder = do_div(pcd, 100000000);
		if (remainder < 50000000)
			pcd -= 1;
	}
	else {
		remainder = do_div(pcd, 100000000 * 2);
		if (remainder < 100000000)
			pcd -= 1;
	}
#endif

	/* FIXME: for lcd clock(10khz) equals 10400 or 5200, special
	   PCD value is used. If we use the formula to calculate the
	   PCD value, the LCD will flicker when DVFM.
	*/
	return (unsigned int)pcd;
}

static inline unsigned int get_pcd(struct pxafb_info *fbi, unsigned int pixclock)
{
	unsigned long long lclk = clk_get_rate(fbi->clk) / 10000; /* in 10KHZ */

	if (fbi->flags & PXAFB_STD_SMART_PANEL)
		return __smart_panel_timing(fbi, lclk, fbi->cmd_inh_time);
	else 
		return __get_pcd(lclk, pixclock);
}

/*
 * Some touchscreens need hsync information from the video driver to
 * function correctly. We export it here.  Note that 'hsync_time' and
 * the value returned from pxafb_get_hsync_time() is the *reciprocal*
 * of the hsync period in seconds.
 */
static inline void set_hsync_time(struct pxafb_info *fbi, unsigned int pcd)
{
	unsigned long htime;

	if ((pcd == 0) || (fbi->fb.var.hsync_len == 0)) {
		fbi->hsync_time=0;
		return;
	}

	htime = clk_get_rate(fbi->clk) / (pcd * fbi->fb.var.hsync_len);

	fbi->hsync_time = htime;
}

unsigned long pxafb_get_hsync_time(struct device *dev)
{
	struct pxafb_info *fbi = dev_get_drvdata(dev);

	/* If display is blanked/suspended, hsync isn't active */
	if (!fbi || (fbi->state != C_ENABLE))
		return 0;

	return fbi->hsync_time;
}
EXPORT_SYMBOL(pxafb_get_hsync_time);

#ifdef CONFIG_PXA3xx_DVFM
static int dvfm_dev_idx;
static void set_dvfm_constraint(void)
{
	/* Disable Lowpower mode */
	/* no D0CS constraint for power saving */
	/* dvfm_disable_op_name("D0CS", dvfm_dev_idx); */
	dvfm_disable_op_name("D1", dvfm_dev_idx);
	dvfm_disable_op_name("D2", dvfm_dev_idx);
	if (cpu_is_pxa935())
		dvfm_disable_op_name("CG", dvfm_dev_idx);
}

static void unset_dvfm_constraint(void)
{
	/* Enable Lowpower mode */
	/* no D0CS constraint for power saving */
	/* dvfm_enable_op_name("D0CS", dvfm_dev_idx); */
	dvfm_enable_op_name("D1", dvfm_dev_idx);
	dvfm_enable_op_name("D2", dvfm_dev_idx);
	if (cpu_is_pxa935())
		dvfm_enable_op_name("CG", dvfm_dev_idx);
}

static int fb_notifier_freq(struct notifier_block *nb,
				unsigned long val, void *data)
{
	struct dvfm_freqs *freqs = (struct dvfm_freqs *)data;
	struct op_info *new = NULL;
	struct dvfm_md_opt *op;

	if (freqs) {
		new = &freqs->new_info;
	} else
		return 0;

	op = (struct dvfm_md_opt *)new->op;
	switch (val) {
	case DVFM_FREQ_PRECHANGE:
		if ((op->power_mode == POWER_MODE_D0) ||
			(op->power_mode == POWER_MODE_D0CS))
			hss = op->hss;
		else if ((op->power_mode == POWER_MODE_D1) ||
			(op->power_mode == POWER_MODE_D2) ||
			(op->power_mode == POWER_MODE_CG))
			lcd_update = 0;
		break;
	case DVFM_FREQ_POSTCHANGE:
		if ((op->power_mode == POWER_MODE_D1) ||
			(op->power_mode == POWER_MODE_D2) ||
			(op->power_mode == POWER_MODE_CG))
			lcd_update = 1;
		break;
	}
	return 0;
}

static int pxafb_adjust_pcd(struct pxafb_info *fbi, int hss)
{
	unsigned int pcd, lccr3, lclk;
	struct fb_var_screeninfo *var = &fbi->fb.var;
	u_int lines_per_panel;

	switch (hss) {
	case 104:
	case 156:
	case 208:
	case 60:
		/* D0CS mode, hss: 60M */
		lclk = hss * 100;
		break;
	default:
		/* otherwise, we just skip adjusting PCD */
		return 0;
	}

	if (fbi->flags & PXAFB_STD_SMART_PANEL)
		goto std_smart_panel;

	/* timing for non-smart panel */
	pcd = __get_pcd(lclk, fbi->fb.var.pixclock);

	/*printk(KERN_DEBUG "pxafb_adjust_pcd: hss=%d, pcd=%d\n", hss, pcd);
	 */

	lccr3 = (LCCR3 & ~0xff) | LCCR3_PixClkDiv(pcd);
	if (lccr3 != LCCR3) {
		LCCR3 = lccr3;
	}

	return 0;

std_smart_panel:
	/* addtional timings for smart panel */
	lines_per_panel = var->yres;
	if ((fbi->lccr0 & LCCR0_SDS) == LCCR0_Dual)
		lines_per_panel /= 2;

	LCCR1 = 
		LCCR1_DisWdth(var->xres) +
		LCCR1_HorSnchWdth(__smart_panel_timing(fbi, lclk, fbi->op_hold_time)) +
		LCCR1_BegLnDel(__smart_panel_timing(fbi, lclk, fbi->wr_pulse_width)) + 
		LCCR1_EndLnDel(__smart_panel_timing(fbi, lclk, fbi->wr_setup_time));
	LCCR2 =
		LCCR2_DisHght(lines_per_panel) +
		LCCR2_BegFrmDel(__smart_panel_timing(fbi, lclk, fbi->rd_pulse_width)) +
		LCCR2_EndFrmDel(__smart_panel_timing(fbi, lclk, fbi->rd_setup_time));

	pcd =  __smart_panel_timing(fbi, lclk, fbi->cmd_inh_time);

	LCCR3 = (LCCR3 & ~0xff) | LCCR3_PixClkDiv(pcd);

	return 0;
}

/* increase pclk for camera */
static u_long g_old_pclk = 146000;
int pxafb_adjust_pclk_workaround(int enable)
{
	struct pxafb_info *fbi = (struct pxafb_info *)dev_id;	

	if (enable) {
		g_old_pclk = fbi->fb.var.pixclock;
		fbi->fb.var.pixclock = 153000/2;
	} else {
		fbi->fb.var.pixclock = g_old_pclk;
	}
	
	return 0;
}

int pxafb_get_pcd(unsigned int hss)
{
	unsigned int pcd, lclk;
	struct pxafb_info *fbi = (struct pxafb_info *)dev_id;

	switch (hss) {
	case 104:
	case 156:
	case 208:
	case 60:
		/* D0CS mode, hss: 60M */
		lclk = hss * 100;
		break;
	default:
		/* otherwise, we just skip adjusting PCD */
		return 0;
	}

	pcd = __get_pcd(lclk, fbi->fb.var.pixclock);
	return pcd;
}

void pxafb_set_pcd(unsigned int pcd)
{
	struct pxafb_info *fbi = (struct pxafb_info *)dev_id;
	volatile unsigned long new_lccr3;

	new_lccr3 = (fbi->reg_lccr3 & ~0xff) | LCCR3_PixClkDiv(pcd);

	if (new_lccr3 != fbi->reg_lccr3) {
		fbi->reg_lccr3 = new_lccr3;
		LCCR3 = new_lccr3;
	}
}

EXPORT_SYMBOL(pxafb_get_pcd);
EXPORT_SYMBOL(pxafb_set_pcd);
#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
#endif

/*
 * pxafb_activate_var():
 *	Configures LCD Controller based on entries in var parameter.  Settings are
 *	only written to the controller if changes were made.
 */ 
static u_int pxafb_reenable_count; /* a workaround to avoid conflict between set_par and resume, fix me */
static int pxafb_activate_var(struct fb_var_screeninfo *var, struct pxafb_info *fbi)
{
	struct pxafb_lcd_reg new_regs;
	u_long flags;
	u_int lines_per_panel, pcd = get_pcd(fbi, var->pixclock);

	pr_debug("pxafb: Configuring PXA LCD\n");

	pr_debug("var: xres=%d hslen=%d lm=%d rm=%d\n",
		 var->xres, var->hsync_len,
		 var->left_margin, var->right_margin);
	pr_debug("var: yres=%d vslen=%d um=%d bm=%d\n",
		 var->yres, var->vsync_len,
		 var->upper_margin, var->lower_margin);
	pr_debug("var: pixclock=%d pcd=%d\n", var->pixclock, pcd);

#if DEBUG_VAR
	if (var->xres < 16        || var->xres > 1024)
		printk(KERN_ERR "%s: invalid xres %d\n",
			fbi->fb.fix.id, var->xres);
	switch(var->bits_per_pixel) {
	case 1:
	case 2:
	case 4:
	case 8:
	case 16:
	case 18:
	case 19:
	case 24:
	case 25:
	case 32:
		break;
	default:
		printk(KERN_ERR "%s: invalid bit depth %d\n",
		       fbi->fb.fix.id, var->bits_per_pixel);
		break;
	}
	if (var->hsync_len < 1    || var->hsync_len > 64)
		printk(KERN_ERR "%s: invalid hsync_len %d\n",
			fbi->fb.fix.id, var->hsync_len);
	if (var->left_margin < 1  || var->left_margin > 255)
		printk(KERN_ERR "%s: invalid left_margin %d\n",
			fbi->fb.fix.id, var->left_margin);
	if (var->right_margin < 1 || var->right_margin > 255)
		printk(KERN_ERR "%s: invalid right_margin %d\n",
			fbi->fb.fix.id, var->right_margin);
	if (var->yres < 1         || var->yres > 1024)
		printk(KERN_ERR "%s: invalid yres %d\n",
			fbi->fb.fix.id, var->yres);
	if (var->vsync_len < 1    || var->vsync_len > 64)
		printk(KERN_ERR "%s: invalid vsync_len %d\n",
			fbi->fb.fix.id, var->vsync_len);
	if (var->upper_margin < 0 || var->upper_margin > 255)
		printk(KERN_ERR "%s: invalid upper_margin %d\n",
			fbi->fb.fix.id, var->upper_margin);
	if (var->lower_margin < 0 || var->lower_margin > 255)
		printk(KERN_ERR "%s: invalid lower_margin %d\n",
			fbi->fb.fix.id, var->lower_margin);
#endif

	down(&fbi->ctrlr_sem);

	new_regs.lccr0 = fbi->lccr0 |
		(LCCR0_LDM | LCCR0_SFM | LCCR0_IUM | LCCR0_EFM |
#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
		LCCR0_OUC |
#endif
                 LCCR0_QDM | LCCR0_BM  | LCCR0_OUM);

	new_regs.lccr4 = LCCR4_REOFM0 | LCCR4_REOFM1 | LCCR4_REOFM2 |
		LCCR4_REOFM3 | LCCR4_REOFM4 | LCCR4_REOFM5 |
#ifdef ENABLE_PXAFB_PCDDIV
		LCCR4_PCDDIV |
#endif
		LCCR4_REOFM6;
	new_regs.lccr4 |= (LCCR4&0x1ff); /* get overlay1 transprant */

	new_regs.lccr5 = LCCR5_SOFM1 | LCCR5_SOFM2 | LCCR5_SOFM3 | 
		LCCR5_SOFM4 | LCCR5_SOFM5 | LCCR5_SOFM6 | 
		LCCR5_EOFM1 | LCCR5_EOFM2 | LCCR5_EOFM3 |
		LCCR5_EOFM4 | LCCR5_EOFM5 | LCCR5_EOFM6 | 
		LCCR5_BSM1 | LCCR5_BSM2 | LCCR5_BSM3 |
		LCCR5_BSM4 | LCCR5_BSM5 | LCCR5_BSM6 | 
		LCCR5_IUM1 | LCCR5_IUM2 | LCCR5_IUM3 |
		LCCR5_IUM4 | LCCR5_IUM5 | LCCR5_IUM6;

	if (fbi->flags & PXAFB_STD_SMART_PANEL)
		goto std_smart_panel;

	new_regs.lccr1 =
		LCCR1_DisWdth(var->xres) +
		LCCR1_HorSnchWdth(var->hsync_len) +
		LCCR1_BegLnDel(var->left_margin) +
		LCCR1_EndLnDel(var->right_margin);

	/*
	 * If we have a dual scan LCD, we need to halve
	 * the YRES parameter.
	 */
	lines_per_panel = var->yres;
	if ((fbi->lccr0 & LCCR0_SDS) == LCCR0_Dual)
		lines_per_panel /= 2;

	new_regs.lccr2 =
		LCCR2_DisHght(lines_per_panel) +
		LCCR2_VrtSnchWdth(var->vsync_len) +
		LCCR2_BegFrmDel(var->upper_margin) +
		LCCR2_EndFrmDel(var->lower_margin);

	new_regs.lccr3 = fbi->lccr3 | LCCR3_STALL |
		pxafb_bpp_to_lccr3(var) |
		(var->sync & FB_SYNC_HOR_HIGH_ACT ? LCCR3_HorSnchH : LCCR3_HorSnchL) |
		(var->sync & FB_SYNC_VERT_HIGH_ACT ? LCCR3_VrtSnchH : LCCR3_VrtSnchL);

	new_regs.lccr6 = LCCR6;

	new_regs.cmdcr = CMDCR;

	goto set_pcd;

std_smart_panel:
	lines_per_panel = var->yres;
	new_regs.lccr0 |= LCCR0_LCDT;
	new_regs.lccr1 = 
		LCCR1_DisWdth(var->xres) +
		LCCR1_HorSnchWdth(smart_panel_timing(fbi, fbi->op_hold_time)) +
		LCCR1_BegLnDel(smart_panel_timing(fbi, fbi->wr_pulse_width)) + 
		LCCR1_EndLnDel(smart_panel_timing(fbi, fbi->wr_setup_time));
	new_regs.lccr2 =
		LCCR2_DisHght(lines_per_panel) +
		LCCR2_BegFrmDel(smart_panel_timing(fbi, fbi->rd_pulse_width)) +
		LCCR2_EndFrmDel(smart_panel_timing(fbi, fbi->rd_setup_time));
	new_regs.lccr3 = fbi->lccr3 |
		pxafb_bpp_to_lccr3(var);
	new_regs.lccr6 = fbi->lccr6;
	new_regs.cmdcr = fbi->sync_cnt;

set_pcd:
	if (pcd)
		new_regs.lccr3 |= LCCR3_PixClkDiv(pcd);

	pr_debug("nlccr0 = 0x%08x\n", new_regs.lccr0);
	pr_debug("nlccr1 = 0x%08x\n", new_regs.lccr1);
	pr_debug("nlccr2 = 0x%08x\n", new_regs.lccr2);
	pr_debug("nlccr3 = 0x%08x\n", new_regs.lccr3);

	/* setup dma descriptors */
	fbi->dmadesc_cmd_cpu = (struct pxafb_dma_descriptor *)((unsigned int)fbi->palette_cpu - 4*16);
	fbi->dmadesc_fblow_cpu = (struct pxafb_dma_descriptor *)((unsigned int)fbi->palette_cpu - 3*16);
	fbi->dmadesc_fbhigh_cpu = (struct pxafb_dma_descriptor *)((unsigned int)fbi->palette_cpu - 2*16);
	fbi->dmadesc_palette_cpu = (struct pxafb_dma_descriptor *)((unsigned int)fbi->palette_cpu - 1*16);

	fbi->dmadesc_cmd_dma = fbi->palette_dma - 4*16;
	fbi->dmadesc_fblow_dma = fbi->palette_dma - 3*16;
	fbi->dmadesc_fbhigh_dma = fbi->palette_dma - 2*16;
	fbi->dmadesc_palette_dma = fbi->palette_dma - 1*16;

#define BYTES_PER_PANEL (lines_per_panel * fbi->fb.fix.line_length)

	/* populate descriptors */
	fbi->dmadesc_cmd_cpu->fdadr = 0; /* don't need point to next descriptor */
	fbi->dmadesc_cmd_cpu->fsadr = fbi->cmd_dma;
	fbi->dmadesc_cmd_cpu->fidr  = 0;
	fbi->dmadesc_cmd_cpu->ldcmd = 0;

	fbi->dmadesc_fblow_cpu->fdadr = fbi->dmadesc_fblow_dma;

	if (is_android())
    	        fbi->dmadesc_fblow_cpu->fsadr = fbi->screen_dma + BYTES_PER_PANEL + 
                        ((fbi->fb.var.yoffset / fbi->fb.var.yres) * (fbi->fb.var.yres) * (fbi->fb.var.xres) * 2);
        else
    	        fbi->dmadesc_fblow_cpu->fsadr = fbi->screen_dma + BYTES_PER_PANEL; 

	fbi->dmadesc_fblow_cpu->fidr  = 0;
	fbi->dmadesc_fblow_cpu->ldcmd = BYTES_PER_PANEL;

	fbi->fdadr1 = fbi->dmadesc_fblow_dma; /* only used in dual-panel mode */

        if (is_android())
	        fbi->dmadesc_fbhigh_cpu->fsadr = fbi->screen_dma + 
		((fbi->fb.var.yoffset / fbi->fb.var.yres) * (fbi->fb.var.yres) * (fbi->fb.var.xres) * 2);
	else
	        fbi->dmadesc_fbhigh_cpu->fsadr = fbi->screen_dma;
 
	fbi->dmadesc_fbhigh_cpu->fidr = 0;
	fbi->dmadesc_fbhigh_cpu->ldcmd = BYTES_PER_PANEL|(0x1<<22) /* SOFINT */
#if defined(CONFIG_PXA3xx_DVFM_HSS_EOF_FB) /* EOFINT */
		|(0x1<<21) 
#endif
		; 

	fbi->dmadesc_palette_cpu->fsadr = fbi->palette_dma;
	fbi->dmadesc_palette_cpu->fidr  = 0;
	if ((fbi->lccr4 & LCCR4_PAL_FOR_MASK) == LCCR4_PAL_FOR_0)
		fbi->dmadesc_palette_cpu->ldcmd = fbi->palette_size *
							sizeof(u16);
	else
		fbi->dmadesc_palette_cpu->ldcmd = fbi->palette_size *
							sizeof(u32);
	fbi->dmadesc_palette_cpu->ldcmd |= LDCMD_PAL;

	if ((var->bits_per_pixel == 16) || (var->bits_per_pixel == 18) ||
		 (var->bits_per_pixel == 19) || (var->bits_per_pixel == 24) ||
		 (var->bits_per_pixel == 25) || (var->bits_per_pixel == 32))
	{
		/* palette shouldn't be loaded in true-color mode */
		fbi->dmadesc_fbhigh_cpu->fdadr = fbi->dmadesc_fbhigh_dma;
		fbi->fdadr0 = fbi->dmadesc_fbhigh_dma; /* no pal just fbhigh */
		/* init it to something, even though we won't be using it */
		fbi->dmadesc_palette_cpu->fdadr = fbi->dmadesc_palette_dma;
	} else {
		fbi->dmadesc_palette_cpu->fdadr = fbi->dmadesc_fbhigh_dma;
		fbi->dmadesc_fbhigh_cpu->fdadr = fbi->dmadesc_palette_dma;
		fbi->fdadr0 = fbi->dmadesc_palette_dma; /* flips back and forth between pal and fbhigh */
	}
	fbi->fdadr6 = fbi->dmadesc_cmd_dma;

#if 0
	pr_debug("fbi->dmadesc_fblow_cpu = 0x%p\n", fbi->dmadesc_fblow_cpu);
	pr_debug("fbi->dmadesc_fbhigh_cpu = 0x%p\n", fbi->dmadesc_fbhigh_cpu);
	pr_debug("fbi->dmadesc_palette_cpu = 0x%p\n", fbi->dmadesc_palette_cpu);
	pr_debug("fbi->dmadesc_fblow_dma = 0x%x\n", fbi->dmadesc_fblow_dma);
	pr_debug("fbi->dmadesc_fbhigh_dma = 0x%x\n", fbi->dmadesc_fbhigh_dma);
	pr_debug("fbi->dmadesc_palette_dma = 0x%x\n", fbi->dmadesc_palette_dma);

	pr_debug("fbi->dmadesc_fblow_cpu->fdadr = 0x%x\n", fbi->dmadesc_fblow_cpu->fdadr);
	pr_debug("fbi->dmadesc_fbhigh_cpu->fdadr = 0x%x\n", fbi->dmadesc_fbhigh_cpu->fdadr);
	pr_debug("fbi->dmadesc_palette_cpu->fdadr = 0x%x\n", fbi->dmadesc_palette_cpu->fdadr);

	pr_debug("fbi->dmadesc_fblow_cpu->fsadr = 0x%x\n", fbi->dmadesc_fblow_cpu->fsadr);
	pr_debug("fbi->dmadesc_fbhigh_cpu->fsadr = 0x%x\n", fbi->dmadesc_fbhigh_cpu->fsadr);
	pr_debug("fbi->dmadesc_palette_cpu->fsadr = 0x%x\n", fbi->dmadesc_palette_cpu->fsadr);

	pr_debug("fbi->dmadesc_fblow_cpu->ldcmd = 0x%x\n", fbi->dmadesc_fblow_cpu->ldcmd);
	pr_debug("fbi->dmadesc_fbhigh_cpu->ldcmd = 0x%x\n", fbi->dmadesc_fbhigh_cpu->ldcmd);
	pr_debug("fbi->dmadesc_palette_cpu->ldcmd = 0x%x\n", fbi->dmadesc_palette_cpu->ldcmd);
#endif

	fbi->reg_lccr0 = new_regs.lccr0;
	fbi->reg_lccr1 = new_regs.lccr1;
	fbi->reg_lccr2 = new_regs.lccr2;
	fbi->reg_lccr3 = new_regs.lccr3;
	fbi->reg_lccr4 = new_regs.lccr4 & (~LCCR4_PAL_FOR_MASK);
	fbi->reg_lccr4 |= (fbi->lccr4 & LCCR4_PAL_FOR_MASK);
	fbi->reg_lccr5 = new_regs.lccr5;
	fbi->reg_lccr6 = new_regs.lccr6;
	fbi->reg_cmdcr = new_regs.cmdcr;
	set_hsync_time(fbi, pcd);
	up(&fbi->ctrlr_sem);

	/*
	 * Only update the registers if the controller is enabled
	 * and something has changed.
	 */
	if ((LCCR0  != fbi->reg_lccr0) || (LCCR1  != fbi->reg_lccr1) ||
	    (LCCR2  != fbi->reg_lccr2) || (LCCR3  != fbi->reg_lccr3) ||
	    (LCCR4  != fbi->reg_lccr4) || (LCCR5  != fbi->reg_lccr5) ||
	    (LCCR6  != fbi->reg_lccr6) || (CMDCR  != fbi->reg_cmdcr) ||
	    (FDADR0 != fbi->fdadr0)    || (FDADR1 != fbi->fdadr1)) {		
		if (pxafb_reenable_count>2) {/* sync screen by frank */
#ifdef CONFIG_PXA3xx_DVFM
			//dvfm_disable_op_name("D0CS", dvfm_dev_idx);
#endif

			pxafb_schedule_work(fbi, C_REENABLE);
			DECLARE_WAITQUEUE(wait, current);			
			add_wait_queue(&fbi->reenable_wait, &wait);
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout(msecs_to_jiffies(500));
			set_current_state(TASK_RUNNING);
			remove_wait_queue(&fbi->reenable_wait, &wait);
			
#ifdef CONFIG_PXA3xx_DVFM
			//dvfm_enable_op_name_aync("D0CS", dvfm_dev_idx);
#endif
		} else {
			pxafb_schedule_work(fbi, C_REENABLE);
			pxafb_reenable_count++;
		}
	}

	return 0;
}

/*
 * NOTE!  The following functions are purely helpers for set_ctrlr_state.
 * Do not call them directly; set_ctrlr_state does the correct serialisation
 * to ensure that things happen in the right way 100% of time time.
 *	-- rmk
 */
static inline void __pxafb_backlight_power(struct pxafb_info *fbi, int on)
{
	pr_debug("pxafb: backlight o%s\n", on ? "n" : "ff");

 	if (pxafb_backlight_power)
 		pxafb_backlight_power(on);
}

static inline void __pxafb_lcd_power(struct pxafb_info *fbi, int on)
{
	pr_debug("pxafb: LCD power o%s\n", on ? "n" : "ff");

	if (pxafb_lcd_power)
		pxafb_lcd_power(on, &fbi->fb.var);
}

void pxafb_set_backlight(int on)
{
	if (pxafb_backlight_power) {
		pxafb_backlight_power(on);
	}
}
EXPORT_SYMBOL(pxafb_set_backlight);

static void pxafb_setup_gpio(struct pxafb_info *fbi)
{
	int gpio, ldd_bits;
        unsigned int lccr0 = fbi->lccr0;

	if (cpu_is_pxa3xx())
		return;

	/*
	 * setup is based on type of panel supported
        */

	/* 4 bit interface */
	if ((lccr0 & LCCR0_CMS) == LCCR0_Mono &&
	    (lccr0 & LCCR0_SDS) == LCCR0_Sngl &&
	    (lccr0 & LCCR0_DPD) == LCCR0_4PixMono)
		ldd_bits = 4;

	/* 8 bit interface */
        else if (((lccr0 & LCCR0_CMS) == LCCR0_Mono &&
		  ((lccr0 & LCCR0_SDS) == LCCR0_Dual || (lccr0 & LCCR0_DPD) == LCCR0_8PixMono)) ||
                 ((lccr0 & LCCR0_CMS) == LCCR0_Color &&
		  (lccr0 & LCCR0_PAS) == LCCR0_Pas && (lccr0 & LCCR0_SDS) == LCCR0_Sngl))
		ldd_bits = 8;

	/* 16 bit interface */
	else if ((lccr0 & LCCR0_CMS) == LCCR0_Color &&
		 ((lccr0 & LCCR0_SDS) == LCCR0_Dual || (lccr0 & LCCR0_PAS) == LCCR0_Act))
		ldd_bits = 16;

	else {
	        printk(KERN_ERR "pxafb_setup_gpio: unable to determine bits per pixel\n");
		return;
        }

	for (gpio = 58; ldd_bits; gpio++, ldd_bits--)
		pxa_gpio_mode(gpio | GPIO_ALT_FN_2_OUT);
	pxa_gpio_mode(GPIO74_LCD_FCLK_MD);
	pxa_gpio_mode(GPIO75_LCD_LCLK_MD);
	pxa_gpio_mode(GPIO76_LCD_PCLK_MD);
	/*pxa_gpio_mode(GPIO77_LCD_ACBIAS_MD);*/
	/*GPIO77 is FFUART on yuhua 300 board*/
}

static void pxafb_enable_controller(struct pxafb_info *fbi)
{
	pr_debug("pxafb: Enabling LCD controller\n");
	pr_debug("fdadr0 0x%08x\n", (unsigned int) fbi->fdadr0);
	pr_debug("fdadr1 0x%08x\n", (unsigned int) fbi->fdadr1);
	pr_debug("reg_lccr0 0x%08x\n", (unsigned int) fbi->reg_lccr0);
	pr_debug("reg_lccr1 0x%08x\n", (unsigned int) fbi->reg_lccr1);
	pr_debug("reg_lccr2 0x%08x\n", (unsigned int) fbi->reg_lccr2);
	pr_debug("reg_lccr3 0x%08x\n", (unsigned int) fbi->reg_lccr3);
	pr_debug("reg_lccr6 0x%08x\n", (unsigned int) fbi->reg_lccr6);

	/* Sequence from 11.7.10 */
	CMDCR = fbi->reg_cmdcr;
	LCCR6 = fbi->reg_lccr6;
	LCCR5 = fbi->reg_lccr5;
	LCCR4 = fbi->reg_lccr4;
	LCCR3 = fbi->reg_lccr3;
	LCCR2 = fbi->reg_lccr2;
	LCCR1 = fbi->reg_lccr1;
	LCCR0 = fbi->reg_lccr0 & ~LCCR0_ENB;

	FDADR0 = fbi->fdadr0;
	FDADR1 = fbi->fdadr1;
	if (!(fbi->flags & PXAFB_SMART_PANEL))
		LCCR0 |= LCCR0_ENB;
	
	/* enable LCD controller clock */
	clk_enable(fbi->clk);
	msleep(1); /* wait lcd controller stable, add by frank */

	pr_debug("FDADR0 0x%08x\n", (unsigned int) FDADR0);
	pr_debug("FDADR1 0x%08x\n", (unsigned int) FDADR1);
	pr_debug("LCCR0 0x%08x\n", (unsigned int) LCCR0);
	pr_debug("LCCR1 0x%08x\n", (unsigned int) LCCR1);
	pr_debug("LCCR2 0x%08x\n", (unsigned int) LCCR2);
	pr_debug("LCCR3 0x%08x\n", (unsigned int) LCCR3);
	pr_debug("LCCR4 0x%08x\n", (unsigned int) LCCR4);
}

static void pxafb_disable_controller(struct pxafb_info *fbi)
{
	u32 lccr0;
	DECLARE_WAITQUEUE(wait, current);

	pr_debug("pxafb: disabling LCD controller\n");
	if(fbi->flags & PXAFB_SMART_PANEL)
		goto exit;

	set_current_state(TASK_UNINTERRUPTIBLE);
	add_wait_queue(&fbi->ctrlr_wait, &wait);

	LCSR = 0xffffffff;	/* Clear LCD Status Register */
	lccr0 = LCCR0;
	lccr0 &= ~LCCR0_LDM;	/* Enable LCD Disable Done Interrupt */
	lccr0 |= LCCR0_DIS;	/* Disable LCD Controller */
	LCCR0 = lccr0;

	schedule_timeout(500 * HZ / 1000);
	remove_wait_queue(&fbi->ctrlr_wait, &wait);
exit:
	/* disable LCD controller clock */
	clk_disable(fbi->clk);
}

int __pxafb_send_cmd(struct pxafb_info *fbi, 
		unsigned short *cmd, unsigned int num)
{
	int ret;
	unsigned short *cmd_buf = (unsigned short*)fbi->cmd_cpu;
	DECLARE_WAITQUEUE(wait, current);

	if (num == 0) return 0;

	pr_debug("pxafb: sending %d commands starting with command (0x%04x)\n", 
			num, *cmd >> 9);

	/* prepare the command buffer */
	memcpy(cmd_buf, cmd, num*sizeof(unsigned short));
	cmd_buf[num] = LCD_CMD_INT_PROC | LCD_CMD_A0_COMMAND;
	cmd_buf[num + 1] = LCD_CMD_WAIT_FOR_VSYNC | LCD_CMD_A0_COMMAND;
	if ((num + 2) & 0x1) {
		/* padding the command buffer to be 4 byte aligned */
		cmd_buf[num + 2] = LCD_CMD_NOP;
		fbi->dmadesc_cmd_cpu->ldcmd = (num + 3)*sizeof(unsigned short);
	} else {
		fbi->dmadesc_cmd_cpu->ldcmd = (num + 2)*sizeof(unsigned short);
	}

	/* send the commands */
	set_current_state(TASK_UNINTERRUPTIBLE);
	add_wait_queue(&fbi->ctrlr_wait, &wait);

	LCSR = 0xffffffff;	/* Clear LCD Status Register */

	clk_disable(fbi->clk);
	LCCR0 &= ~(LCCR0_ENB);
	PRSR |= PRSR_ST_OK | PRSR_CON_NT;
	FDADR6 = fbi->fdadr6;
	LCCR0 |= LCCR0_ENB;
	clk_enable(fbi->clk);
	
	/* wait for the commands done */
	LCCR0 &= ~LCCR0_CMDIM;	/* Enable command Interrupt */
	/*
	 * Workaround for LCD command timeout. Enable SMC clock when sending
	 * command and data to smart pannel. Otherwise, LCD command timeout
	 * happens.
	 * Disable SMC clock after sending command and data. Because we want
	 * to gate SMC clock.
	 */
	clk_enable(fbi->smc_clk);
	ret = schedule_timeout(4*HZ);
	clk_disable(fbi->smc_clk);
	remove_wait_queue(&fbi->ctrlr_wait, &wait);

	/* quick disable */
	PRSR &= ~(PRSR_ST_OK | PRSR_CON_NT);
	LCCR0 &= ~(LCCR0_ENB);
	LCSR |= LCSR_CMD_INT; 
	FDADR6 = 0;

	/* return */
	if (!ret){
		printk(KERN_ERR "pxafb: send commands timeout!\n");
		return -EIO;
	} else {
		pr_debug("pxafb: send commands OK!\n");
		return 0;
	}
}

int pxafb_send_cmd(struct fb_info *fbi, 
		unsigned short *cmd, unsigned int num)
{
	unsigned short *head = cmd, *tail = cmd;
	unsigned int ms;

	while (tail < cmd + num){
		if ((*tail & LCD_CMD_COMMAND_MASK) == LCD_CMD_WAIT) {
			if ( __pxafb_send_cmd(fbi, head, tail - head))
				return -EIO;
			ms = (*tail & LCD_CMD_DATA_MASK);
			if (ms) msleep(ms);
			head = tail + 1;
		}
		if (tail == cmd + num - 1) {
			if (__pxafb_send_cmd((struct pxafb_info *)fbi, head, tail - head + 1))
				return -EIO;
		}
		tail++;
	}

	return 0;
}

int pxafb_update_framedata(struct pxafb_info *fbi)
{
	int ret = 0;

	if (lcd_update == 0)
		return 0;

	FDADR0 = fbi->fdadr0;
 	if(fbi->overlay1fb && fbi->overlay1fb->state == C_ENABLE) {
		FDADR1 = fbi->fdadr1;
		OVL1C2 = fbi->reg_ovl1c2;
		OVL1C1 = fbi->reg_ovl1c1;
	}

	if ((fbi->overlay2fb && fbi->overlay2fb->state == C_ENABLE) || 
		(fbi->ov2_v4l2_dev && (fbi->ov2_v4l2_dev->device_state == C_ENABLE))) {
		FDADR2 = fbi->fdadr2;
		if ((fbi->overlay2fb && fbi->overlay2fb->format != 0) ||
			(fbi->ov2_v4l2_dev && fbi->ov2_v4l2_dev->format != 0)) {
			FDADR3 = fbi->fdadr3;
			FDADR4 = fbi->fdadr4;	
		}
		OVL2C2 = fbi->reg_ovl2c2;
		OVL2C1 = fbi->reg_ovl2c1;
	}

	if (fbi->update_framedata)
		ret = fbi->update_framedata(&fbi->fb);
	else
		printk(KERN_ERR "pxafb: lcd panel update frame routine is NULL!\n");

	FDADR0 = 0;
 	if ((fbi->overlay1fb && fbi->overlay1fb->state == C_ENABLE) ||
	      (OVL1C1 & OVL1C1_O1EN)) {
		OVL1C1 &= ~OVL1C1_O1EN;
		FDADR1 = 0;
	}

	if ((fbi->overlay2fb && fbi->overlay2fb->state == C_ENABLE) || 
		(fbi->ov2_v4l2_dev && (fbi->ov2_v4l2_dev->device_state == C_ENABLE)) ||
		(OVL2C1 & OVL2C1_O2EN)) {
		OVL2C1 &= ~OVL2C1_O2EN;
		FDADR2 = 0;
		if ((fbi->overlay2fb && fbi->overlay2fb->format != 0) ||
			(fbi->ov2_v4l2_dev && fbi->ov2_v4l2_dev->format != 0)) {
			FDADR3 = 0;
			FDADR4 = 0;
		}
		if (fbi->ov2_handle_eof)
			fbi->ov2_handle_eof();
	}

	return ret;
}

#define PXA_AUTO_REFRESH_INTERVAL_MS 15
static int pxafb_auto_refresh(void *arg)
{
	struct pxafb_info *fbi = (struct pxafb_info *)arg;
	int refresh_interval = PXA_AUTO_REFRESH_INTERVAL_MS*HZ/1000;
	DECLARE_WAITQUEUE(wait, current);

	if(!fbi)
	    return -1;
	pr_debug("pxafb: auto refresh enabled\n");

	daemonize("lcd_refresh");
	while(1) {
		down_interruptible(&fbi->ctrlr_sem);
		
		if (fbi->state == C_DISABLE){
			up(&fbi->ctrlr_sem);
			break;
		}
		
		while(fbi->state != C_ENABLE){
			pr_debug("pxafb: auto refresh sleeping...\n");
			up(&fbi->ctrlr_sem);	
			set_current_state(TASK_UNINTERRUPTIBLE);
			add_wait_queue(&fbi->ctrlr_wait, &wait);
			schedule();
			remove_wait_queue(&fbi->ctrlr_wait, &wait);
			down(&fbi->ctrlr_sem);
		}
		
		pxafb_update_framedata(fbi);
		up(&fbi->ctrlr_sem);
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(refresh_interval);
	}
	pr_debug("pxafb: auto refresh disabled!\n");

	return 0;
}

/*
 *  pxafb_handle_irq: Handle 'LCD DONE' interrupts.
 */
static irqreturn_t pxafb_handle_irq(int irq, void *dev_id)
{
	struct pxafb_info *fbi = dev_id;
	unsigned int lcsr = LCSR;
	unsigned int lcsr1 = LCSR1;

	//printk("lcsr 0x%x\n", lcsr);
	if (lcsr & LCSR_LDD || lcsr & LCSR_CMD_INT) {
		LCCR0 |= LCCR0_LDM | LCCR0_CMDIM;
		wake_up(&fbi->ctrlr_wait);
	}
	
	if (lcsr & LCSR_SOF && !(LCCR0&LCCR0_SFM)) {
		LCCR0 |= LCCR0_SFM;
		wake_up(&fbi->ctrlr_wait);
	}

	if(lcsr1 & LCSR1_BS2) {
		wake_up(&fbi->ctrlr_wait);
	}
	
	if ((lcsr1 & LCSR1_EOF2) && fbi->ov2_handle_eof)
		fbi->ov2_handle_eof();

	if ((lcsr1 & LCSR1_BS2) && fbi->ov2_handle_bra && (!(fbi->flags & PXAFB_SMART_PANEL)))	
		fbi->ov2_handle_bra();
	
	LCSR = lcsr;
	LCSR1 = lcsr1;

	return IRQ_HANDLED;
}

/*
 * This function must be called from task context only, since it will
 * sleep when disabling the LCD controller, or if we get two contending
 * processes trying to alter state.
 */
void set_ctrlr_state(struct pxafb_info *fbi, u_int state)
{
	u_int old_state;

	down(&fbi->ctrlr_sem);

	old_state = fbi->state;

	/*
	 * Hack around fbcon initialisation.
	 */
	if (old_state == C_STARTUP && state == C_REENABLE)
		state = C_ENABLE;

	switch (state) {
	case C_DISABLE_CLKCHANGE:
		/*
		 * Disable controller for clock change.  If the
		 * controller is already disabled, then do nothing.
		 */
		if (old_state != C_DISABLE && old_state != C_DISABLE_PM) {
			fbi->state = state;
			//TODO __pxafb_lcd_power(fbi, 0);
			if(fbi->set_overlay1_ctrlr_state)
				fbi->set_overlay1_ctrlr_state(fbi, C_DISABLE);
			if(fbi->set_overlay2_ctrlr_state)
				fbi->set_overlay2_ctrlr_state(fbi, C_DISABLE);
			if(fbi->set_cursorfb_ctrlr_state)
				fbi->set_cursorfb_ctrlr_state(fbi, C_DISABLE);
			pxafb_disable_controller(fbi);
		}
		break;

	case C_DISABLE_PM:
	case C_DISABLE:
		/*
		 * Disable controller
		 */
		if (old_state != C_DISABLE) {
			fbi->state = state;
			__pxafb_backlight_power(fbi, 0);
			__pxafb_lcd_power(fbi, 0);
			if(fbi->set_overlay1_ctrlr_state)
				fbi->set_overlay1_ctrlr_state(fbi, C_DISABLE);
			if(fbi->set_overlay2_ctrlr_state)
				fbi->set_overlay2_ctrlr_state(fbi, C_DISABLE);
			if(fbi->set_cursorfb_ctrlr_state)
				fbi->set_cursorfb_ctrlr_state(fbi, C_DISABLE);
			if (old_state != C_DISABLE_CLKCHANGE)
				pxafb_disable_controller(fbi);
			unset_dvfm_constraint();
		}
		break;

	case C_ENABLE_CLKCHANGE:
		/*
		 * Enable the controller after clock change.  Only
		 * do this if we were disabled for the clock change.
		 */
		if (old_state == C_DISABLE_CLKCHANGE) {
			fbi->state = C_ENABLE;
			pxafb_enable_controller(fbi);
			if (fbi->flags & PXAFB_AUTO_REFRESH)
				wake_up(&fbi->ctrlr_wait);
			//TODO __pxafb_lcd_power(fbi, 1);
			if(fbi->set_overlay1_ctrlr_state)
				fbi->set_overlay1_ctrlr_state(fbi, C_ENABLE);
			if(fbi->set_overlay2_ctrlr_state)
				fbi->set_overlay2_ctrlr_state(fbi, C_ENABLE);
			if(fbi->set_cursorfb_ctrlr_state)
				fbi->set_cursorfb_ctrlr_state(fbi, C_ENABLE);
		}
		break;

	case C_REENABLE:
		/*
		 * Re-enable the controller only if it was already
		 * enabled.  This is so we reprogram the control
		 * registers.
		 */
		if (old_state == C_ENABLE) {
			if(fbi->set_overlay1_ctrlr_state)
				fbi->set_overlay1_ctrlr_state(fbi, C_DISABLE);
			if(fbi->set_overlay2_ctrlr_state)
				fbi->set_overlay2_ctrlr_state(fbi, C_DISABLE);
			if(fbi->set_cursorfb_ctrlr_state)
				fbi->set_cursorfb_ctrlr_state(fbi, C_DISABLE);
			pxafb_disable_controller(fbi);
			pxafb_setup_gpio(fbi);
			pxafb_enable_controller(fbi);
			if(fbi->set_overlay1_ctrlr_state)
				fbi->set_overlay1_ctrlr_state(fbi, C_ENABLE);
			if(fbi->set_overlay2_ctrlr_state)
				fbi->set_overlay2_ctrlr_state(fbi, C_ENABLE);
			if(fbi->set_cursorfb_ctrlr_state)
				fbi->set_cursorfb_ctrlr_state(fbi, C_ENABLE);
		}
		wake_up(&fbi->reenable_wait);
		break;

	case C_ENABLE_PM:
		/*
		 * Re-enable the controller after PM.  This is not
		 * perfect - think about the case where we were doing
		 * a clock change, and we suspended half-way through.
		 */
		if (old_state != C_DISABLE_PM)
			break;
		if (fbi->flags & PXAFB_AUTO_REFRESH)
			wake_up(&fbi->ctrlr_wait);
		/* fall through */

	case C_ENABLE:
		/*
		 * Power up the LCD screen, enable controller, and
		 * turn on the backlight.
		 */
		if (old_state != C_ENABLE) {
			set_dvfm_constraint();
			fbi->state = C_ENABLE;
			pxafb_setup_gpio(fbi);
			pxafb_enable_controller(fbi);
			__pxafb_lcd_power(fbi, 1);
			__pxafb_backlight_power(fbi, 1);
			if(fbi->flags & PXAFB_AUTO_REFRESH) {
				if (kernel_thread(pxafb_auto_refresh, fbi, 
					CLONE_FS | CLONE_FILES | CLONE_SIGHAND) < 0){
					printk(KERN_ERR "pxafb: create kernel thread failed\n");
				} 
			}
			if(fbi->set_overlay1_ctrlr_state)
				fbi->set_overlay1_ctrlr_state(fbi, C_ENABLE);
			if(fbi->set_overlay2_ctrlr_state)
				fbi->set_overlay2_ctrlr_state(fbi, C_ENABLE);
			if(fbi->set_cursorfb_ctrlr_state)
				fbi->set_cursorfb_ctrlr_state(fbi, C_ENABLE);
		}
		break;
	}
	up(&fbi->ctrlr_sem);
}

/*
 * Our LCD controller task (which is called when we blank or unblank)
 * via keventd.
 */
static void pxafb_task(struct work_struct *work)
{
	struct pxafb_info *fbi =
		container_of(work, struct pxafb_info, task);
	u_int state = xchg(&fbi->task_state, -1);

	set_ctrlr_state(fbi, state);
}

#ifdef CONFIG_CPU_FREQ
/*
 * CPU clock speed change handler.  We need to adjust the LCD timing
 * parameters when the CPU clock is adjusted by the power management
 * subsystem.
 *
 * TODO: Determine why f->new != 10*get_lclk_frequency_10khz()
 */
static int
pxafb_freq_transition(struct notifier_block *nb, unsigned long val, void *data)
{
	struct pxafb_info *fbi = TO_INF(nb, freq_transition);
	//TODO struct cpufreq_freqs *f = data;
	u_int pcd;

	switch (val) {
	case CPUFREQ_PRECHANGE:
		set_ctrlr_state(fbi, C_DISABLE_CLKCHANGE);
		break;

	case CPUFREQ_POSTCHANGE:
		pcd = get_pcd(fbi, fbi->fb.var.pixclock);
		set_hsync_time(fbi, pcd);
		fbi->reg_lccr3 = (fbi->reg_lccr3 & ~0xff) | LCCR3_PixClkDiv(pcd);
		set_ctrlr_state(fbi, C_ENABLE_CLKCHANGE);
		break;
	}
	return 0;
}

static int
pxafb_freq_policy(struct notifier_block *nb, unsigned long val, void *data)
{
	struct pxafb_info *fbi = TO_INF(nb, freq_policy);
	struct fb_var_screeninfo *var = &fbi->fb.var;
	struct cpufreq_policy *policy = data;

	switch (val) {
	case CPUFREQ_ADJUST:
	case CPUFREQ_INCOMPATIBLE:
		pr_debug("min dma period: %d ps, "
			"new clock %d kHz\n", pxafb_display_dma_period(var),
			policy->max);
		// TODO: fill in min/max values
		break;
#if 0
	case CPUFREQ_NOTIFY:
		printk(KERN_ERR "%s: got CPUFREQ_NOTIFY\n", __FUNCTION__);
		do {} while(0);
		/* todo: panic if min/max values aren't fulfilled
		 * [can't really happen unless there's a bug in the
		 * CPU policy verification process *
		 */
		break;
#endif
	}
	return 0;
}
#endif

#ifdef CONFIG_PM
/*
 * Power management hooks.  Note that we won't be called from IRQ context,
 * unlike the blank functions above, so we may sleep.
 */
static int pxafb_suspend(struct platform_device *dev, pm_message_t state)
{
	struct pxafb_info *fbi = platform_get_drvdata(dev);

	pxafb_reenable_count = 0;
	set_ctrlr_state(fbi, C_DISABLE_PM);
	return 0;
}

static int pxafb_resume(struct platform_device *dev)
{
	struct pxafb_info *fbi = platform_get_drvdata(dev);

	set_ctrlr_state(fbi, C_ENABLE_PM);
	return 0;
}
#else
#define pxafb_suspend	NULL
#define pxafb_resume	NULL
#endif

/*
 * pxafb_map_video_memory():
 *      Allocates the DRAM memory for the frame buffer.  This buffer is
 *	remapped into a non-cached, non-buffered, memory region to
 *      allow palette and pixel writes to occur without flushing the
 *      cache.  Once this area is remapped, all virtual memory
 *      access to the video memory should occur at the new region.
 */
static int __init pxafb_map_video_memory(struct pxafb_info *fbi)
{
	u_long palette_mem_size;

	/*
	 * We reserve one page for the palette, plus the size
	 * of the framebuffer.
	 */
	fbi->map_size = PAGE_ALIGN(fbi->fb.fix.smem_len + PAGE_SIZE);
	fbi->map_cpu = dma_alloc_writecombine(fbi->dev, fbi->map_size,
					      &fbi->map_dma, GFP_KERNEL);

	if (fbi->map_cpu) {
		/* prevent initial garbage on screen */
		memset(fbi->map_cpu, 0, fbi->map_size);
		fbi->fb.screen_base = fbi->map_cpu + PAGE_SIZE;
		fbi->screen_dma = fbi->map_dma + PAGE_SIZE;
		/*
		 * FIXME: this is actually the wrong thing to place in
		 * smem_start.  But fbdev suffers from the problem that
		 * it needs an API which doesn't exist (in this case,
		 * dma_writecombine_mmap)
		 */
		fbi->fb.fix.smem_start = fbi->screen_dma;
		fbi->palette_size = fbi->fb.var.bits_per_pixel == 8 ? 256 : 16;

		if ((fbi->lccr4 & LCCR4_PAL_FOR_MASK) == LCCR4_PAL_FOR_0)
			palette_mem_size = fbi->palette_size * sizeof(u16);
		else
			palette_mem_size = fbi->palette_size * sizeof(u32);

		pr_debug("pxafb: palette_mem_size = 0x%08lx\n", palette_mem_size);

		fbi->palette_cpu = (u16 *)(fbi->map_cpu + PAGE_SIZE - palette_mem_size);
		fbi->palette_dma = fbi->map_dma + PAGE_SIZE - palette_mem_size;

		fbi->cmd_cpu = fbi->map_cpu;
		fbi->cmd_dma = fbi->map_dma;
	}

	return fbi->map_cpu ? 0 : -ENOMEM;
}

int pxafb_width;
int pxafb_height;
static struct pxafb_info * __init pxafb_init_fbinfo(struct device *dev)
{
	struct pxafb_info *fbi = NULL;
	void *addr;
	struct pxafb_mach_info *inf = dev->platform_data;
	struct pxafb_mode_info *mode = inf->modes;
	struct fb_videomode fb_mode;
	struct fb_var_screeninfo var;
	int i, smemlen;


	/* Alloc the pxafb_info and pseudo_palette in one step */
	fbi = kmalloc(sizeof(struct pxafb_info) + sizeof(u32) * 16, GFP_KERNEL);
	if (!fbi)
		goto fail;

	memset(fbi, 0, sizeof(struct pxafb_info));

	/* max size of overlay2 buffer
	 * RGB 25 bit. duel buffer. (640x480x4x2)
	 */
#if 0
	fbi->ov2_map_size = 2457600;
#else
	fbi->ov2_map_size = PAGE_ALIGN(inf->modes->xres * inf->modes->yres * 2 * 2 + PAGE_SIZE);
	printk("Alloc ov2 mem 0x%x\n", fbi->ov2_map_size);
#endif
	fbi->ov2_map_cpu = dma_alloc_writecombine(NULL, fbi->ov2_map_size,
			&(fbi->ov2_map_dma), GFP_KERNEL);

	if ((!fbi->ov2_map_cpu) || (!fbi->ov2_map_dma)){
		printk(KERN_ERR "pxafb failed to allocate memory!\n");
		goto fail;
	}

	fbi->ov1_map_size = PAGE_ALIGN(inf->modes->xres * inf->modes->yres * 2 * 2 + PAGE_SIZE);
	printk("Alloc ov1 mem 0x%x\n", fbi->ov1_map_size);
	fbi->ov1_map_cpu = dma_alloc_writecombine(NULL, fbi->ov1_map_size,
			&(fbi->ov1_map_dma), GFP_KERNEL);

	if ((!fbi->ov1_map_cpu) || (!fbi->ov1_map_dma)){
		printk(KERN_ERR "pxafb ov1 failed to allocate memory!\n");
		goto fail;
	}
	
	INIT_LIST_HEAD(&fbi->fb.modelist);
	for (i = 0;i < inf->num_modes;i++) {
		pxafb_setmode(&var, &inf->modes[i]);
		fb_var_to_videomode(&fb_mode, &var);
		fb_add_videomode(&fb_mode, &fbi->fb.modelist);
	}
	
	fbi->dev = dev;

	fbi->smc_clk = clk_get(NULL, "SMCCLK");

	fbi->clk = clk_get(dev, "LCDCLK");
	if (IS_ERR(fbi->clk)) {
		goto fail;
	}

	/* ensure the LCD clock is off */
	clk_enable(fbi->clk);
	clk_disable(fbi->clk);

	strcpy(fbi->fb.fix.id, PXA_NAME);

	fbi->fb.fix.type	= FB_TYPE_PACKED_PIXELS;
	fbi->fb.fix.type_aux	= 0;
	fbi->fb.fix.xpanstep	= 0;
	fbi->fb.fix.ypanstep	= 0;
	fbi->fb.fix.ywrapstep	= 0;
	fbi->fb.fix.accel	= FB_ACCEL_NONE;

	fbi->fb.var.nonstd	= 0;
	fbi->fb.var.activate	= FB_ACTIVATE_NOW;
	fbi->fb.var.height	= -1;
	fbi->fb.var.width	= -1;
	fbi->fb.var.accel_flags	= 0;
	fbi->fb.var.vmode	= FB_VMODE_NONINTERLACED;

	fbi->fb.fbops		= &pxafb_ops;
	fbi->fb.flags		= FBINFO_DEFAULT;
	fbi->fb.node		= -1;

	addr = fbi;
	addr = addr + sizeof(struct pxafb_info);
	fbi->fb.pseudo_palette	= addr;

	pxafb_width = inf->modes->xres;
	pxafb_height = inf->modes->yres;

	pxafb_setmode(&fbi->fb.var, mode);

	if (is_android()) {
		fbi->fb.var.xres                = mode->xres;
		fbi->fb.var.xres_virtual        = mode->xres;
		fbi->fb.var.yres                = mode->yres;
		fbi->fb.var.yres_virtual        = mode->yres * 2;
	}

	fbi->flags = mode->flags;
	fbi->wr_setup_time = mode->wr_setup_time;
	fbi->wr_pulse_width = mode->wr_pulse_width;
	fbi->rd_setup_time = mode->rd_setup_time;
	fbi->rd_pulse_width = mode->rd_pulse_width;
	fbi->op_hold_time = mode->op_hold_time;
	fbi->cmd_inh_time = mode->cmd_inh_time;
	fbi->sync_cnt = mode->sync_cnt;
	fbi->update_framedata = mode->update_framedata;

	fbi->cmap_inverse		= inf->cmap_inverse;
	fbi->cmap_static		= inf->cmap_static;

	fbi->lccr0			= inf->lccr0;
	fbi->lccr3			= inf->lccr3;
	fbi->lccr4			= inf->lccr4;
	fbi->lccr6			= inf->lccr6;
	fbi->state			= C_STARTUP;
	fbi->task_state			= (u_char)-1;

	for (i = 0; i < inf->num_modes; i++) {
		smemlen = mode[i].xres * mode[i].yres * mode[i].bpp / 8;

		if (is_android())
			smemlen *= 2;

		if (smemlen > fbi->fb.fix.smem_len)
			fbi->fb.fix.smem_len = smemlen;
	}

	init_waitqueue_head(&fbi->ctrlr_wait);
	init_waitqueue_head(&fbi->reenable_wait);
	INIT_WORK(&fbi->task, pxafb_task);
	init_MUTEX(&fbi->ctrlr_sem);

	return fbi;
	
fail:
	if (fbi->ov2_map_cpu)
		dma_free_writecombine(NULL, fbi->ov2_map_size,
			(void*)(fbi->ov2_map_cpu), fbi->ov2_map_dma);
	
	if (fbi->ov1_map_cpu)
		dma_free_writecombine(NULL, fbi->ov1_map_size,
			(void*)(fbi->ov1_map_cpu), fbi->ov1_map_dma);

	if(fbi)
		kfree(fbi);		

	return NULL;

}

#ifdef CONFIG_FB_PXA_PARAMETERS
static int __init pxafb_parse_options(struct device *dev, char *options)
{
	struct pxafb_mach_info *inf = dev->platform_data;
	char *this_opt;

        if (!options || !*options)
                return 0;

	dev_dbg(dev, "options are \"%s\"\n", options ? options : "null");

	/* could be made table driven or similar?... */
        while ((this_opt = strsep(&options, ",")) != NULL) {
                if (!strncmp(this_opt, "mode:", 5)) {
			const char *name = this_opt+5;
			unsigned int namelen = strlen(name);
			int res_specified = 0, bpp_specified = 0;
			unsigned int xres = 0, yres = 0, bpp = 0;
			int yres_specified = 0;
			int i;
			for (i = namelen-1; i >= 0; i--) {
				switch (name[i]) {
				case '-':
					namelen = i;
					if (!bpp_specified && !yres_specified) {
						bpp = simple_strtoul(&name[i+1], NULL, 0);
						bpp_specified = 1;
					} else
						goto done;
					break;
				case 'x':
					if (!yres_specified) {
						yres = simple_strtoul(&name[i+1], NULL, 0);
						yres_specified = 1;
					} else
						goto done;
					break;
				case '0' ... '9':
					break;
				default:
					goto done;
				}
			}
			if (i < 0 && yres_specified) {
				xres = simple_strtoul(name, NULL, 0);
				res_specified = 1;
			}
		done:
			if (res_specified) {
				dev_info(dev, "overriding resolution: %dx%d\n", xres, yres);
				inf->modes[0].xres = xres; inf->modes[0].yres = yres;
			}
			if (bpp_specified)
				switch (bpp) {
				case 1:
				case 2:
				case 4:
				case 8:
				case 16:
				case 18:
				case 19:
					inf->modes[0].bpp = bpp;
					dev_info(dev, "overriding bit depth: %d\n", bpp);
					break;
				default:
					dev_err(dev, "Depth %d is not valid\n", bpp);
				}
                } else if (!strncmp(this_opt, "pixclock:", 9)) {
                        inf->modes[0].pixclock = simple_strtoul(this_opt+9, NULL, 0);
			dev_info(dev, "override pixclock: %ld\n", inf->modes[0].pixclock);
                } else if (!strncmp(this_opt, "left:", 5)) {
                        inf->modes[0].left_margin = simple_strtoul(this_opt+5, NULL, 0);
			dev_info(dev, "override left: %u\n", inf->modes[0].left_margin);
                } else if (!strncmp(this_opt, "right:", 6)) {
                        inf->modes[0].right_margin = simple_strtoul(this_opt+6, NULL, 0);
			dev_info(dev, "override right: %u\n", inf->modes[0].right_margin);
                } else if (!strncmp(this_opt, "upper:", 6)) {
                        inf->modes[0].upper_margin = simple_strtoul(this_opt+6, NULL, 0);
			dev_info(dev, "override upper: %u\n", inf->modes[0].upper_margin);
                } else if (!strncmp(this_opt, "lower:", 6)) {
                        inf->modes[0].lower_margin = simple_strtoul(this_opt+6, NULL, 0);
			dev_info(dev, "override lower: %u\n", inf->modes[0].lower_margin);
                } else if (!strncmp(this_opt, "hsynclen:", 9)) {
                        inf->modes[0].hsync_len = simple_strtoul(this_opt+9, NULL, 0);
			dev_info(dev, "override hsynclen: %u\n", inf->modes[0].hsync_len);
                } else if (!strncmp(this_opt, "vsynclen:", 9)) {
                        inf->modes[0].vsync_len = simple_strtoul(this_opt+9, NULL, 0);
			dev_info(dev, "override vsynclen: %u\n", inf->modes[0].vsync_len);
                } else if (!strncmp(this_opt, "hsync:", 6)) {
                        if (simple_strtoul(this_opt+6, NULL, 0) == 0) {
				dev_info(dev, "override hsync: Active Low\n");
				inf->modes[0].sync &= ~FB_SYNC_HOR_HIGH_ACT;
			} else {
				dev_info(dev, "override hsync: Active High\n");
				inf->modes[0].sync |= FB_SYNC_HOR_HIGH_ACT;
			}
                } else if (!strncmp(this_opt, "vsync:", 6)) {
                        if (simple_strtoul(this_opt+6, NULL, 0) == 0) {
				dev_info(dev, "override vsync: Active Low\n");
				inf->modes[0].sync &= ~FB_SYNC_VERT_HIGH_ACT;
			} else {
				dev_info(dev, "override vsync: Active High\n");
				inf->modes[0].sync |= FB_SYNC_VERT_HIGH_ACT;
			}
                } else if (!strncmp(this_opt, "dpc:", 4)) {
                        if (simple_strtoul(this_opt+4, NULL, 0) == 0) {
				dev_info(dev, "override double pixel clock: false\n");
				inf->lccr3 &= ~LCCR3_DPC;
			} else {
				dev_info(dev, "override double pixel clock: true\n");
				inf->lccr3 |= LCCR3_DPC;
			}
                } else if (!strncmp(this_opt, "outputen:", 9)) {
                        if (simple_strtoul(this_opt+9, NULL, 0) == 0) {
				dev_info(dev, "override output enable: active low\n");
				inf->lccr3 = (inf->lccr3 & ~LCCR3_OEP) | LCCR3_OutEnL;
			} else {
				dev_info(dev, "override output enable: active high\n");
				inf->lccr3 = (inf->lccr3 & ~LCCR3_OEP) | LCCR3_OutEnH;
			}
                } else if (!strncmp(this_opt, "pixclockpol:", 12)) {
                        if (simple_strtoul(this_opt+12, NULL, 0) == 0) {
				dev_info(dev, "override pixel clock polarity: falling edge\n");
				inf->lccr3 = (inf->lccr3 & ~LCCR3_PCP) | LCCR3_PixFlEdg;
			} else {
				dev_info(dev, "override pixel clock polarity: rising edge\n");
				inf->lccr3 = (inf->lccr3 & ~LCCR3_PCP) | LCCR3_PixRsEdg;
			}
                } else if (!strncmp(this_opt, "color", 5)) {
			inf->lccr0 = (inf->lccr0 & ~LCCR0_CMS) | LCCR0_Color;
                } else if (!strncmp(this_opt, "mono", 4)) {
			inf->lccr0 = (inf->lccr0 & ~LCCR0_CMS) | LCCR0_Mono;
                } else if (!strncmp(this_opt, "active", 6)) {
			inf->lccr0 = (inf->lccr0 & ~LCCR0_PAS) | LCCR0_Act;
                } else if (!strncmp(this_opt, "passive", 7)) {
			inf->lccr0 = (inf->lccr0 & ~LCCR0_PAS) | LCCR0_Pas;
                } else if (!strncmp(this_opt, "single", 6)) {
			inf->lccr0 = (inf->lccr0 & ~LCCR0_SDS) | LCCR0_Sngl;
                } else if (!strncmp(this_opt, "dual", 4)) {
			inf->lccr0 = (inf->lccr0 & ~LCCR0_SDS) | LCCR0_Dual;
                } else if (!strncmp(this_opt, "4pix", 4)) {
			inf->lccr0 = (inf->lccr0 & ~LCCR0_DPD) | LCCR0_4PixMono;
                } else if (!strncmp(this_opt, "8pix", 4)) {
			inf->lccr0 = (inf->lccr0 & ~LCCR0_DPD) | LCCR0_8PixMono;
		} else {
			dev_err(dev, "unknown option: %s\n", this_opt);
			return -EINVAL;
		}
        }
        return 0;

}
#endif

#include <mach/mfp.h>
#include <mach/mfp-pxa300.h>
static struct clk *lcdbl_clk;
static void pxafb_lcdbl_on(void)
{
#if defined(CONFIG_YUHUA_MISC_DEV)
	extern int lcdbl_get_current_brightness(void);
	int brightness = lcdbl_get_current_brightness();
#else /* No lcd bl misc control */
	int brightness = 0x270;
#endif

	if (!lcdbl_clk)
#if defined(CONFIG_BOARD_X2_V4) /* Set gpio20 to PWM3 output */
		lcdbl_clk = clk_get(NULL, "PWM1CLK");
#else
		lcdbl_clk = clk_get(NULL, "PWM0CLK");
#endif
	if (!lcdbl_clk) {
		printk(KERN_ERR"failed to get lcdbl_clk clock\n");		
	} else
		clk_enable(lcdbl_clk);
	
#if defined(CONFIG_BOARD_X2_V4) /* Set gpio20 to PWM3 output */
    	pxa3xx_mfp_set_afds(MFP_PIN_GPIO20, GPIO20_PWM3_OUT, MFP_DS04X);
	PWM_CTRL3 = 0x1; /* freq */
	PWM_PERVAL3 = 0x3FF;
	PWM_PWDUTY3 = brightness&0x3FF;	
#else /* Set gpio19 to PWM2 output */
	pxa3xx_mfp_set_afds(MFP_PIN_GPIO19, GPIO19_PWM2_OUT, MFP_DS04X);
	#ifdef CONFIG_FB_LCD_SHARP_T6K90
	PWM_CTRL2 = 0x1F; /* freq */
	#else
	PWM_CTRL2 = 0x1; /* freq */
	#endif
	PWM_PERVAL2 = 0x3FF;
	PWM_PWDUTY2 = brightness&0x3FF;
#endif
	//printk("pxafb_lcdbl_on[%x]\n",brightness);
}

static void pxafb_lcdbl_off(void)
{ 
#if defined(CONFIG_BOARD_X2_V4)
	PWM_PWDUTY3 = 0;
#else
	PWM_PWDUTY2 = 0;
#endif
	if (lcdbl_clk)
   		clk_disable(lcdbl_clk);
	//printk("pxafb_lcdbl_off \n");
}

static void pxafb_lcdbl_power(int on)
{	
	if(on){
		pxafb_lcdbl_on();
	} else {
		pxafb_lcdbl_off();
	} 

	return;
}

static ssize_t lcdreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u_int reg, val;
	struct pxafb_mach_info *inf = dev->platform_data;
	
	sscanf(buf, "0x%x 0x%x", &reg, &val);
	if (inf && inf->set_lcd_reg) {
		printk("Set lcd reg 0x%x to val 0x%x\n", reg, val);
		inf->set_lcd_reg(reg, val);
	} else 
		printk("Not output set_lcd_reg from driver");
		
	return count;
}
static DEVICE_ATTR(lcdreg,0644,NULL,lcdreg_store);

static ssize_t lcdtest_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val;
	struct pxafb_mach_info *inf = dev->platform_data;
	
	sscanf(buf, "%d", &val);
	if (inf && inf->test_lcd) {
		printk("test_lcd new val %d\n", val);
		inf->test_lcd(val);
	} else 
		printk("Not output test_lcd from driver");
		
	return count;
}
static DEVICE_ATTR(lcdtest,0666,NULL,lcdtest_store);

static struct attribute *pxafb_attributes[] = {
	&dev_attr_lcdreg.attr,
	&dev_attr_lcdtest.attr,
	NULL,
};
static struct attribute_group pxafb_attr_group ={
	.attrs=pxafb_attributes,
};

static int __init pxafb_probe(struct platform_device *dev)
{
	struct pxafb_info *fbi;
	struct pxafb_mach_info *inf;
	int ret;

	dev_dbg(&dev->dev, "pxafb_probe\n");

	inf = dev->dev.platform_data;
	ret = -ENOMEM;
	fbi = NULL;
	if (!inf)
		goto failed;

#ifdef CONFIG_FB_PXA_PARAMETERS
	ret = pxafb_parse_options(&dev->dev, g_options);
	if (ret < 0)
		goto failed;
#endif

#ifdef ENABLE_PXAFB_PCDDIV
	inf->lccr4 |= LCCR4_PCDDIV;
	LCCR4 |= LCCR4_PCDDIV;
#endif

#ifdef DEBUG_VAR
        /* Check for various illegal bit-combinations. Currently only
	 * a warning is given. */

        if (inf->lccr0 & LCCR0_INVALID_CONFIG_MASK)
                dev_warn(&dev->dev, "machine LCCR0 setting contains illegal bits: %08x\n",
                        inf->lccr0 & LCCR0_INVALID_CONFIG_MASK);
        if (inf->lccr3 & LCCR3_INVALID_CONFIG_MASK)
                dev_warn(&dev->dev, "machine LCCR3 setting contains illegal bits: %08x\n",
                        inf->lccr3 & LCCR3_INVALID_CONFIG_MASK);
        if (inf->lccr0 & LCCR0_DPD &&
	    ((inf->lccr0 & LCCR0_PAS) != LCCR0_Pas ||
	     (inf->lccr0 & LCCR0_SDS) != LCCR0_Sngl ||
	     (inf->lccr0 & LCCR0_CMS) != LCCR0_Mono))
                dev_warn(&dev->dev, "Double Pixel Data (DPD) mode is only valid in passive mono"
			 " single panel mode\n");
        if ((inf->lccr0 & LCCR0_PAS) == LCCR0_Act &&
	    (inf->lccr0 & LCCR0_SDS) == LCCR0_Dual)
                dev_warn(&dev->dev, "Dual panel only valid in passive mode\n");
        if ((inf->lccr0 & LCCR0_PAS) == LCCR0_Pas &&
             (inf->modes->upper_margin || inf->modes->lower_margin))
                dev_warn(&dev->dev, "Upper and lower margins must be 0 in passive mode\n");
#endif

	dev_dbg(&dev->dev, "got a %dx%dx%d LCD\n",inf->modes->xres, inf->modes->yres, inf->modes->bpp);
	if (inf->modes->xres == 0 || inf->modes->yres == 0 || inf->modes->bpp == 0) {
		dev_err(&dev->dev, "Invalid resolution or bit depth\n");
		ret = -EINVAL;
		goto failed;
	}
	pxafb_backlight_power = inf->pxafb_backlight_power;
	if (!pxafb_backlight_power) { /* No special lcdbl, use platform default */
		pxafb_backlight_power = pxafb_lcdbl_power;
	}
	pxafb_lcd_power = inf->pxafb_lcd_power;
	fbi = pxafb_init_fbinfo(&dev->dev);
	if (!fbi) {
		dev_err(&dev->dev, "Failed to initialize framebuffer device\n");
		ret = -ENOMEM; // only reason for pxafb_init_fbinfo to fail is kmalloc
		goto failed;
	}


	/* Initialize video memory */
	ret = pxafb_map_video_memory(fbi);
	if (ret) {
		dev_err(&dev->dev, "Failed to allocate video RAM: %d\n", ret);
		ret = -ENOMEM;
		goto failed;
	}
	ret = request_irq(IRQ_LCD, pxafb_handle_irq, IRQF_DISABLED, "LCD", fbi);
	if (ret) {
		dev_err(&dev->dev, "request_irq failed: %d\n", ret);
		ret = -EBUSY;
		goto failed;
	}

	/*
	 * This makes sure that our colour bitfield
	 * descriptors are correctly initialised.
	 */
	pxafb_check_var(&fbi->fb.var, &fbi->fb);
	pxafb_set_pixfmt(&fbi->fb.var, inf->modes->depth);
	pxafb_set_par_init(&fbi->fb);

	platform_set_drvdata(dev, fbi);

	ret = register_framebuffer(&fbi->fb);
	if (ret < 0) {
		dev_err(&dev->dev, "Failed to register framebuffer device: %d\n", ret);
		goto failed;
	}

#ifdef CONFIG_PM
	// TODO
#endif

#ifdef CONFIG_CPU_FREQ
	fbi->freq_transition.notifier_call = pxafb_freq_transition;
	fbi->freq_policy.notifier_call = pxafb_freq_policy;
	cpufreq_register_notifier(&fbi->freq_transition, CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_register_notifier(&fbi->freq_policy, CPUFREQ_POLICY_NOTIFIER);
#endif

#ifdef CONFIG_PXA3xx_DVFM
	dev_id = fbi;
#endif

#ifdef CONFIG_FB_PXA_MINILCD
        pxafb_minilcd_register(&fbi->fb);
#endif
	/*
	 * Ok, now enable the LCD controller
	 */
	set_ctrlr_state(fbi, C_ENABLE);

	/* register sysfs */
	ret = sysfs_create_group(&dev->dev.kobj, &pxafb_attr_group);

	return 0;

failed:
	platform_set_drvdata(dev, NULL);
	if (fbi)
	kfree(fbi);
	return ret;
}

static struct platform_driver pxafb_driver = {
	.probe		= pxafb_probe,
#ifdef CONFIG_PM
	.suspend	= pxafb_suspend,
	.resume		= pxafb_resume,
#endif
	.driver		= {
		.name	= "pxa2xx-fb",
	},
};

#ifndef MODULE
static int __devinit pxafb_setup(char *options)
{
# ifdef CONFIG_FB_PXA_PARAMETERS
	if (options)
		strlcpy(g_options, options, sizeof(g_options));
# endif
	return 0;
}
#else
# ifdef CONFIG_FB_PXA_PARAMETERS
module_param_string(options, g_options, sizeof(g_options), 0);
MODULE_PARM_DESC(options, "LCD parameters (see Documentation/fb/pxafb.txt)");
# endif
#endif

static int __devinit pxafb_init(void)
{
#ifndef MODULE
	char *option = NULL;

	if (fb_get_options("pxafb", &option))
		return -ENODEV;
	pxafb_setup(option);
#endif

#ifdef CONFIG_PXA3xx_DVFM
	dvfm_register_notifier(&notifier_freq_block,
			DVFM_FREQUENCY_NOTIFIER);
	dvfm_register("LCD Base", &dvfm_dev_idx);
#endif

	return platform_driver_register(&pxafb_driver);
}

module_init(pxafb_init);

MODULE_DESCRIPTION("loadable framebuffer driver for PXA");
MODULE_LICENSE("GPL");
