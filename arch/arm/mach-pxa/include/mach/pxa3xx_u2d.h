/*
 * linux/arch/arm/mach-pxa/include/mach/pxa3xx_u2d.h
 *
 * This supports machine-specific differences in how the PXA3xx
 * USB 2.0 Device Controller (U2D) is wired.
 *
 * It is set in linux/arch/arm/mach-pxa/<machine>.c and used in
 * the probe routine of linux/drivers/usb/gadget/pxa3xx_u2d.c
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */
struct pxa3xx_u2d_mach_info {
	int (*u2d_is_connected) (void);	/* do we see host? */
	void (*u2d_command) (int cmd);
	void (*reset_xcvr) (void);
	void (*reset_utmi) (void);
	void (*soft_dis) (int enable);
	void (*ulpi_dat3) (int enable);
#define	PXA2XX_U2D_CMD_CONNECT		0	/* let host see us */
#define	PXA2XX_U2D_CMD_DISCONNECT	1	/* so host won't see us */
};

extern void pxa_set_u2d_info(struct pxa3xx_u2d_mach_info *info);

