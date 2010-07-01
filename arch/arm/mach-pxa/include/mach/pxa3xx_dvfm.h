/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef PXA3XX_DVFM_H
#define PXA3XX_DVFM_H

#ifdef __KERNEL__
#include <mach/dvfm.h>
#include <mach/pxa3xx_pm.h>

#define DMEMC_FREQ_HIGH		0
#define DMEMC_FREQ_LOW		1
#define DMEMC_D0CS_ENTER	2
#define DMEMC_D0CS_EXIT		3

#define OP_NAME_LEN		16

enum {
	POWER_MODE_D0 = 0,
	POWER_MODE_D0CS,
	POWER_MODE_D1,
	POWER_MODE_D2,
	POWER_MODE_CG,
};

enum {
	OP_FLAG_FACTORY = 0,
	OP_FLAG_USER_DEFINED,
	OP_FLAG_BOOT,
	OP_FLAG_ALL,
};

enum {
	IDLE_D0 = 0,
	IDLE_D0CS = 1,
	IDLE_D1 = 2,
	IDLE_D2 = 4,
	IDLE_CG = 8,
};

struct dvfm_md_opt {
	int	vcc_core;
	int	vcc_sram;
	int	xl;
	int	xn;
	int	core;
	int	smcfs;
	int	sflfs;
	int	hss;
	int	dmcfs;
	int	df_clk;
	int	empi_clk;
	int	power_mode;
	int	flag;
	int	lpj;
	char	name[OP_NAME_LEN];
};

struct pxa3xx_freq_mach_info {
	int flags;
};

#define PXA3xx_USE_POWER_I2C  (1UL << 0)
extern void set_pxa3xx_freq_info(struct pxa3xx_freq_mach_info *info);
extern void set_pxa3xx_freq_parent(struct device *parent_dev);
#endif

/* This structure is similar to dvfm_md_opt.
 * Reserve this structure in order to keep compatible
 */
struct pxa3xx_fv_info {
	unsigned long xl;
	unsigned long xn;
	unsigned int vcc_core;
	unsigned int vcc_sram;
	unsigned long smcfs;
	unsigned long sflfs;
	unsigned long hss;
	unsigned long dmcfs;
	unsigned long df_clk;
	unsigned long empi_clk;
	unsigned long d0cs;
    /* WARNING: above fields must be consistent with PM_FV_INFO!!!*/

	unsigned long lpj; /* New value for loops_per_jiffy */
};

#ifdef __KERNEL__
extern int md2fvinfo(struct pxa3xx_fv_info *fv_info, struct dvfm_md_opt *orig);
#endif

#endif

