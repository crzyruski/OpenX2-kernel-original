#ifndef __PXAFB_H__
#define __PXAFB_H__

/*
 * linux/drivers/video/pxafb.h
 *    -- Intel PXA250/210 LCD Controller Frame Buffer Device
 *
 *  Copyright (C) 1999 Eric A. Thomas.
 *  Copyright (C) 2004 Jean-Frederic Clere.
 *  Copyright (C) 2004 Ian Campbell.
 *  Copyright (C) 2004 Jeff Lackey.
 *   Based on sa1100fb.c Copyright (C) 1999 Eric A. Thomas
 *  which in turn is
 *   Based on acornfb.c Copyright (C) Russell King.
 *
 *  2001-08-03: Cliff Brake <cbrake@acclent.com>
 *	 - ported SA1100 code to PXA
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */

#ifdef CONFIG_FB_PXA_MINILCD

#include <asm/ioctl.h>

/* commands for pxafb_minilcd_ioctl() */

#define PXAFB_MINILCD_ENABLE            _IOW('F', 0x80, unsigned int)
#define PXAFB_MINILCD_BACKLIGHT         _IOW('F', 0x81, unsigned int)
#define PXAFB_MINILCD_WAKEUP            _IOW('F', 0x82, unsigned int)
#define PXAFB_MINILCD_FWAKEUP           _IOW('F', 0x83, unsigned int)
#define PXAFB_MINILCD_FRAMEDATA         _IOW('F', 0x84, void *)

/* Shadows for Mini-LCD controller registers */
struct pxafb_minilcd_reg {
        uint32_t mlccr0;
        uint32_t mlccr1;
        uint32_t mlccr2;
        uint32_t mlsadd;
        uint32_t mlfrmcnt;
};

/*
 * pxafb_minilcd_info - run-time information to enable mini-lcd
 * enable     - enable in low power mode (S0/D1/C2)
 * framecount - shadow of register MLFRMCNT
 * frameaddr  - shadow of register MLSADR
 * framedata  - points to the encoded data from user specified buffer,
 *              or NULL if the base frame buffer is going to be used.
 * framesize  - size of the encoded frame data if 'framedata' is not NULL
 */
struct pxafb_minilcd_info {
        unsigned int    enable;
        unsigned int    backlight;
        uint32_t        framecount;
        void *          framedata;
        size_t          framesize;

        uint32_t        sram_addr_phys; /* Physical address of the SRAM */
        void *          sram_addr_virt; /* Virtual address of the SRAM */
        void *          sram_save_to;   /* address to backup SRAM into */
        size_t          sram_save_size; /* size of saved SRAM */
};

extern int pxafb_minilcd_register(struct fb_info *);
extern int pxafb_minilcd_ioctl(struct fb_info *info, unsigned int cmd,
                                unsigned long arg);

extern int pxafb_minilcd_enter(void);
extern int pxafb_minilcd_exit(void);
#endif



/* Shadows for LCD controller registers */
struct pxafb_lcd_reg {
	unsigned int lccr0;
	unsigned int lccr1;
	unsigned int lccr2;
	unsigned int lccr3;
	unsigned int lccr4;
	unsigned int lccr5;
	unsigned int lccr6;
	unsigned int cmdcr;
};

/* PXA LCD DMA descriptor */
struct pxafb_dma_descriptor {
	unsigned int fdadr;
	unsigned int fsadr;
	unsigned int fidr;
	unsigned int ldcmd;
};

#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
/* PXA Overlay Framebuffer Support */
struct overlayfb_info
{
	struct fb_info	fb;
	
	struct fb_var_screeninfo old_var;

	struct semaphore mutex;
	unsigned long	 refcount;

	struct pxafb_info *basefb;

	unsigned long	map_cpu;
	unsigned long 	screen_cpu;
	unsigned long	palette_cpu;
	unsigned long 	map_size;
	unsigned long   palette_size;

	dma_addr_t 	screen_dma;
	dma_addr_t	map_dma;
	dma_addr_t	palette_dma;

	volatile u_char	state;

	/* overlay specific info */
	unsigned long	xpos;		/* screen position (x, y)*/
	unsigned long	ypos;		
	unsigned long 	format;
	unsigned int	buffer_num;
	unsigned int	buffer_index;
	unsigned int	ylen;
	unsigned int	cblen;
	unsigned int	crlen;
	unsigned int	yoff;
	unsigned int	cboff; 
	unsigned int	croff;

	/* additional */
	union {
		struct pxafb_dma_descriptor *dma0;
		struct pxafb_dma_descriptor *dma1;
		struct {
			struct pxafb_dma_descriptor *dma2;
			struct pxafb_dma_descriptor *dma3;
			struct pxafb_dma_descriptor *dma4;
		};
		struct {
			struct pxafb_dma_descriptor *dma5_pal;
			struct pxafb_dma_descriptor *dma5_frame;
		};
	};
	unsigned int dma_changed;
};
#endif
 
struct pxafb_info {
	struct fb_info		fb;
	struct device		*dev;
	struct clk		*clk;
	struct clk		*smc_clk;	/* SMC clock */

	/*
	 * These are the addresses we mapped
	 * the framebuffer memory region to.
	 */
	/* raw memory addresses */
	dma_addr_t		map_dma;	/* physical */
	u_char *		map_cpu;	/* virtual */
	u_int			map_size;
	

	/* overlay2 reserve memory */
	dma_addr_t	ov2_map_dma; /* DMA address of overlay2 buffer */
	u_char		*ov2_map_cpu; /* cpu address of overlay2 buffer */
	
	/* max size of overlay2 buffer
	 * RGB 25 bit. duel buffer. (640x480x4x2)
	 */
	u_int ov2_map_size;

	/* overlay1 reserve memory */
	dma_addr_t	ov1_map_dma; /* DMA address of overlay1 buffer */
	u_char		*ov1_map_cpu; /* cpu address of overlay1 buffer */
	u_int ov1_map_size;

	/* addresses of pieces placed in raw buffer */
	u_char *		cmd_cpu;	/* virtual address of command buffer */
	dma_addr_t		cmd_dma;	/* physical address of command buffer */
	u_char *		screen_cpu;	/* virtual address of frame buffer */
	dma_addr_t		screen_dma;	/* physical address of frame buffer */
	u16 *			palette_cpu;	/* virtual address of palette memory */
	dma_addr_t		palette_dma;	/* physical address of palette memory */
	u_int			palette_size;

	/* DMA descriptors */
	struct pxafb_dma_descriptor * 	dmadesc_cmd_cpu;
	dma_addr_t		dmadesc_cmd_dma;
	struct pxafb_dma_descriptor * 	dmadesc_fblow_cpu;
	dma_addr_t		dmadesc_fblow_dma;
	struct pxafb_dma_descriptor * 	dmadesc_fbhigh_cpu;
	dma_addr_t		dmadesc_fbhigh_dma;
	struct pxafb_dma_descriptor *	dmadesc_palette_cpu;
	dma_addr_t		dmadesc_palette_dma;

	dma_addr_t		fdadr0;
	dma_addr_t		fdadr1;
	dma_addr_t		fdadr2;
	dma_addr_t		fdadr3;
	dma_addr_t		fdadr4;
	dma_addr_t		fdadr6;

	u_int			lccr0;
	u_int			lccr3;
	u_int			lccr4;
	u_int			lccr6;
	u_int			cmap_inverse:1,
				cmap_static:1,
				unused:30;

	u_int			reg_lccr0;
	u_int			reg_lccr1;
	u_int			reg_lccr2;
	u_int			reg_lccr3;
	u_int			reg_lccr4;
	u_int			reg_lccr5;
	u_int			reg_lccr6;
	u_int			reg_cmdcr;
	u_int			reg_ovl1c1;
	u_int			reg_ovl1c2;
	u_int			reg_ovl2c1;
	u_int			reg_ovl2c2;

	unsigned long	hsync_time;

	volatile u_char		state;
	volatile u_char		task_state;
	struct semaphore	ctrlr_sem;
	wait_queue_head_t	ctrlr_wait;
	wait_queue_head_t	reenable_wait; /* add by frank for android flicker by sync C_REENABLE */
	struct work_struct	task;

#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
	/* PXA Overlay Framebuffer Support */
	struct overlayfb_info  *overlay1fb;
	struct overlayfb_info  *overlay2fb;
	struct overlayfb_info  *cursorfb;
	struct pxa2xx_v4l2ov2_dev *ov2_v4l2_dev;
	void (*set_overlay1_ctrlr_state)(struct pxafb_info *, u_int);
	void (*set_overlay2_ctrlr_state)(struct pxafb_info *, u_int);
	void (*set_cursorfb_ctrlr_state)(struct pxafb_info *, u_int);
	void (*ov2_handle_eof)(void);	
	void (*ov2_handle_bra)(void);
#endif

	u_int		flags;
	/* time unit is ns for following timing parameters */
	u_long		wr_setup_time; /* A0 and CS Setup Time before LCD_PCLK_WR is asserted */
	u_long		wr_pulse_width; /* LCD_PCLK_WR pulse width */
	u_long		rd_setup_time; /* A0 and CS Setup Time before L_FCLK_RD is asserted */
	u_long		rd_pulse_width; /* LCD_PCLK_RD pulse width */
	u_long		op_hold_time; /* Output Hold time from L_FCLK_RD negation */
	u_long		cmd_inh_time; /* Command Inhibit time between two writes */
	u_int		sync_cnt; /* count of L_VSYNC asserted */
	int			(*update_framedata)(struct fb_info *fbi);

#ifdef CONFIG_CPU_FREQ
	struct notifier_block	freq_transition;
	struct notifier_block	freq_policy;
#endif

#ifdef CONFIG_FB_PXA_MINILCD
        struct pxafb_minilcd_info minilcd_info;
#endif
};

#define TO_INF(ptr,member) container_of(ptr,struct pxafb_info,member)

/*
 * These are the actions for set_ctrlr_state
 */
#define C_DISABLE		(0)
#define C_ENABLE		(1)
#define C_DISABLE_CLKCHANGE	(2)
#define C_ENABLE_CLKCHANGE	(3)
#define C_REENABLE		(4)
#define C_DISABLE_PM		(5)
#define C_ENABLE_PM		(6)
#define C_STARTUP		(7)

/*
 * Format of overlay 2
 */
#define OV2_FORMAT_RGB			(0)
#define OV2_FORMAT_YUV444_PACKED	(1)
#define OV2_FORMAT_YUV444_PLANAR	(2)
#define OV2_FORMAT_YUV422_PLANAR	(3)
#define OV2_FORMAT_YUV420_PLANAR	(4)

#define PXA_NAME	"PXA"

/*
 * Minimum X and Y resolutions
 */
#define MIN_XRES	64
#define MIN_YRES	64

#ifdef CONFIG_FB_PXA_MINILCD
int pxafb_minilcd_ioctl(struct fb_info *, unsigned int, unsigned long);
int pxafb_minilcd_register(struct fb_info *);
#endif

#endif /* __PXAFB_H__ */
