/*
 *  linux/arch/arm/mach-pxa/include/mach/pxafb.h
 *
 *  Support for the xscale frame buffer.
 *
 *  Author:     Jean-Frederic Clere
 *  Created:    Sep 22, 2003
 *  Copyright:  jfclere@sinix.net
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __PXA_FB_H_
#define __PXA_FB_H_

#include <linux/fb.h>

/*
 * This structure describes the machine which we are running on.
 * It is set in linux/arch/arm/mach-pxa/machine_name.c and used in the probe routine
 * of linux/drivers/video/pxafb.c
 */
#define LCD_CMD_COMMAND_MASK		(0xf << 9)
#define LCD_CMD_FRAME_DATA_WRITE	(0x2 << 9)
#define LCD_CMD_WAIT_FOR_VSYNC		(0x3 << 9)
#define LCD_CMD_COMMAND_WRITE		(0x1 << 9)
#define LCD_CMD_DATA_WRITE          	(0x1 << 9)
#define LCD_CMD_NOP                 	(0x4 << 9)
#define LCD_CMD_INT_PROC		(0x5 << 9)
#define LCD_CMD_WAIT			(0x6 << 9)

#define LCD_CMD_A0_MASK			(0x1 << 8)
#define LCD_CMD_A0_COMMAND		(0x0 << 8)
#define LCD_CMD_A0_DATA			(0x1 << 8)

#define LCD_CMD_DATA_MASK		(0xff << 0)

#define MAKEUP_CMD(x)			(LCD_CMD_COMMAND_WRITE | LCD_CMD_A0_COMMAND | (x))
#define MAKEUP_DATA(x)			(LCD_CMD_DATA_WRITE | LCD_CMD_A0_DATA | (x))

/* flags */
#define PXAFB_SMART_PANEL		(1UL << 0)	/* the panel with internal frame buffer */
#define PXAFB_STD_SMART_PANEL		(1UL << 1)	/* standard smart panel */
#define PXAFB_AUTO_REFRESH		(1UL << 2)	/* auto refresh smart panel */
struct pxafb_mode_info {
	/* flags */
	u_int		flags;

	u_short		xres;
	u_short		yres;

	u_char		bpp;

	u_char		left_margin;
	u_char		right_margin;

	u_char		upper_margin;
	u_char		lower_margin;

	u_char		sync;

	/* non-smart panel only */
	u_long		pixclock;

	u_char		hsync_len;

	u_char		vsync_len;

	u_int		cmap_greyscale:1,
			depth : 8,
			unused : 23;

	/* smart panel only: timing parameters's unit is ns */
	u_long		wr_setup_time; /* A0 and CS Setup Time before LCD_PCLK_WR is asserted */
	u_long		wr_pulse_width; /* LCD_PCLK_WR pulse width */
	u_long		rd_setup_time; /* A0 and CS Setup Time before L_FCLK_RD is asserted */
	u_long		rd_pulse_width; /* LCD_PCLK_RD pulse width */
	u_long		op_hold_time; /* Output Hold time from L_FCLK_RD negation */
	u_long		cmd_inh_time;/* Command Inhibit time between two writes */
	u_int		sync_cnt; /* times of L_VSYNC asserted */
	int		(*update_framedata)(struct fb_info *fbi);
};

struct pxafb_mach_info {
	struct pxafb_mode_info *modes;
	unsigned int num_modes;

	u_int		fixed_modes:1,
			cmap_inverse:1,
			cmap_static:1,
			unused:29;

	/* The following should be defined in LCCR0
	 *      LCCR0_Act or LCCR0_Pas          Active or Passive
	 *      LCCR0_Sngl or LCCR0_Dual        Single/Dual panel
	 *      LCCR0_Mono or LCCR0_Color       Mono/Color
	 *      LCCR0_4PixMono or LCCR0_8PixMono (in mono single mode)
	 *      LCCR0_DMADel(Tcpu) (optional)   DMA request delay
	 *
	 * The following should not be defined in LCCR0:
	 *      LCCR0_OUM, LCCR0_BM, LCCR0_QDM, LCCR0_DIS, LCCR0_EFM
	 *      LCCR0_IUM, LCCR0_SFM, LCCR0_LDM, LCCR0_ENB
	 */
	u_int		lccr0;
	/* The following should be defined in LCCR3
	 *      LCCR3_OutEnH or LCCR3_OutEnL    Output enable polarity
	 *      LCCR3_PixRsEdg or LCCR3_PixFlEdg Pixel clock edge type
	 *      LCCR3_Acb(X)                    AB Bias pin frequency
	 *      LCCR3_DPC (optional)            Double Pixel Clock mode (untested)
	 *
	 * The following should not be defined in LCCR3
	 *      LCCR3_HSP, LCCR3_VSP, LCCR0_Pcd(x), LCCR3_Bpp
	 */
	u_int		lccr3;
	/* The following should be defined in LCCR4
	 *	LCCR4_PAL_FOR_0 or LCCR4_PAL_FOR_1 or LCCR4_PAL_FOR_2
	 *
	 * All other bits in LCCR4 should be left alone.
	 */
	u_int		lccr4;
	/* The following should be defined in LCCR6
	 *	LCCR6_TWO_WD_SM or LCCR6_SEP_TD_TIM
	 *
	 * All other bits in LCCR6 should be left alone.
	 */
	u_int		lccr6;
	void (*pxafb_backlight_power)(int);
	void (*pxafb_lcd_power)(int, struct fb_var_screeninfo *);
	int (*set_lcd_reg)(u_int reg, u_int val);
	int (*test_lcd)(int val);
};
void set_pxa_fb_info(struct pxafb_mach_info *hard_pxa_fb_info);
void set_pxa_fb_parent(struct device *parent_dev);
unsigned long pxafb_get_hsync_time(struct device *dev);
int pxafb_send_cmd(struct fb_info *fbi, unsigned short *cmd, unsigned int num);

extern int pxafb_width;
extern int pxafb_height;
#define FB_WIDTH 	(pxafb_width)
#define FB_HEIGHT 	(pxafb_height)

int pxafb_adjust_pclk_workaround(int enable);

#endif /* __PXA_FB_H_ */
