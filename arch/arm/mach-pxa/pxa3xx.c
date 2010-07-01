/*
 * linux/arch/arm/mach-pxa/pxa3xx.c
 *
 * code specific to pxa3xx aka Monahans
 *
 * Copyright (C) 2006 Marvell International Ltd.
 *
 * 2007-09-02: eric miao <eric.miao@marvell.com>
 *             initial version
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>
#include <mach/hardware.h>
#include <asm/cacheflush.h>
#include <mach/pxa3xx-regs.h>
#include <mach/pxa3xx_u2d.h>
#include <mach/reset.h>
#include <mach/ohci.h>
#include <mach/uart.h>
#include <mach/pm.h>
#include <mach/dma.h>
#include <mach/ssp.h>
#include <mach/i2c.h>

#include "generic.h"
#include "devices.h"
#include "clock.h"

/* Crystal clock: 13MHz */
#define BASE_CLK	13000000

/* Ring Oscillator Clock: 60MHz */
#define RO_CLK		60000000

#define ACCR_D0CS	(1 << 26)

#define pxa_inv_range		__glue(_CACHE,_dma_inv_range)
#define pxa_clean_range		__glue(_CACHE,_dma_clean_range)
#define pxa_flush_range		__glue(_CACHE,_dma_flush_range)

EXPORT_SYMBOL(pxa_inv_range);
EXPORT_SYMBOL(pxa_clean_range);
EXPORT_SYMBOL(pxa_flush_range);

static int android_project = 0;
static int __init android_setup(char *__unused)
{
	android_project = 1;
	return 1;
}
__setup("android", android_setup);

int is_android(void)
{
	return android_project;
}
EXPORT_SYMBOL(is_android);

/* crystal frequency to static memory controller multiplier (SMCFS) */
static unsigned char smcfs_mult[8] = { 6, 0, 8, 0, 0, 16, };

/* crystal frequency to HSIO bus frequency multiplier (HSS) */
static unsigned char hss_mult[4] = { 8, 12, 16, 0 };

/*
 * Get the clock frequency as reflected by CCSR and the turbo flag.
 * We assume these values have been applied via a fcs.
 * If info is not 0 we also display the current settings.
 */
unsigned int pxa3xx_get_clk_frequency_khz(int info)
{
	unsigned long acsr, xclkcfg;
	unsigned int t, xl, xn, hss, ro, XL, XN, CLK, HSS;

	/* Read XCLKCFG register turbo bit */
	__asm__ __volatile__("mrc\tp14, 0, %0, c6, c0, 0" : "=r"(xclkcfg));
	t = xclkcfg & 0x1;

	acsr = ACSR;

	xl  = acsr & 0x1f;
	xn  = (acsr >> 8) & 0x7;
	hss = (acsr >> 14) & 0x3;

	XL = xl * BASE_CLK;
	XN = xn * XL;

	ro = acsr & ACCR_D0CS;

	CLK = (ro) ? RO_CLK : ((t) ? XN : XL);
	HSS = (ro) ? RO_CLK : hss_mult[hss] * BASE_CLK;

	if (info) {
		pr_info("RO Mode clock: %d.%02dMHz (%sactive)\n",
			RO_CLK / 1000000, (RO_CLK % 1000000) / 10000,
			(ro) ? "" : "in");
		pr_info("Run Mode clock: %d.%02dMHz (*%d)\n",
			XL / 1000000, (XL % 1000000) / 10000, xl);
		pr_info("Turbo Mode clock: %d.%02dMHz (*%d, %sactive)\n",
			XN / 1000000, (XN % 1000000) / 10000, xn,
			(t) ? "" : "in");
		pr_info("HSIO bus clock: %d.%02dMHz\n",
			HSS / 1000000, (HSS % 1000000) / 10000);
	}

	return CLK / 1000;
}

/*
 * Return the current static memory controller clock frequency
 * in units of 10kHz
 */
unsigned int pxa3xx_get_memclk_frequency_10khz(void)
{
	unsigned long acsr;
	unsigned int smcfs, clk = 0;

	acsr = ACSR;

	smcfs = (acsr >> 23) & 0x7;
	clk = (acsr & ACCR_D0CS) ? RO_CLK : smcfs_mult[smcfs] * BASE_CLK;

	return (clk / 10000);
}

void pxa3xx_clear_reset_status(unsigned int mask)
{
	/* RESET_STATUS_* has a 1:1 mapping with ARSR */
}

/*
 * Return the current AC97 clock frequency.
 */
static unsigned long clk_pxa3xx_ac97_getrate(struct clk *clk)
{
	unsigned long rate = 312000000;
	unsigned long ac97_div;

	ac97_div = AC97_DIV;

	/* This may loose precision for some rates but won't for the
	 * standard 24.576MHz.
	 */
	rate /= (ac97_div >> 12) & 0x7fff;
	rate *= (ac97_div & 0xfff);

	return rate;
}

/*
 * Return the current HSIO bus clock frequency
 */
static unsigned long clk_pxa3xx_hsio_getrate(struct clk *clk)
{
	unsigned long acsr;
	unsigned int hss, hsio_clk;

	acsr = ACSR;

	hss = (acsr >> 14) & 0x3;
	hsio_clk = (acsr & ACCR_D0CS) ? RO_CLK : hss_mult[hss] * BASE_CLK;

	return hsio_clk;
}

/*
 * Return the NAND clock frequency
 * PXA320/PXA930:     104 MHz
 * PXA935:     156 MHz
 * PXA300/310: 208 MHz
 */
static unsigned long clk_pxa3xx_nand_getrate(struct clk *clk)
{
	if (cpu_is_pxa320() || cpu_is_pxa930())
		return 104 * 1000 * 1000;
	else if (cpu_is_pxa935())
		return 156 * 1000 * 1000;
	else
		return 208 * 1000 * 1000;
}

/*
 * Return the SRAM clock frequency
 */
static unsigned long clk_pxa3xx_isc_getrate(struct clk *clk)
{
	unsigned long acsr;
	unsigned int ro_s, sfl_s, isc_clk = 0;

	acsr = ACSR;

	ro_s = (acsr >> 26) & 0x1;
	if (ro_s) {
		isc_clk = 60 * 1000 * 1000;
	} else {
		sfl_s = (acsr >> 18) & 0x3;
		switch (sfl_s) {
		case 0x1:
			isc_clk = 156 * 1000 * 1000;
			break;
		case 0x2:
			isc_clk = 208 * 1000 * 1000;
			break;
		case 0x3:
			isc_clk = 312 * 1000 * 1000;
			break;
		}
	}

	return isc_clk;
}

/*
 * Return the Static Memory Controller clock frequency
 */
static unsigned long clk_pxa3xx_smc_getrate(struct clk *clk)
{
	unsigned long acsr;
	unsigned int ro_s, smc_s, smc_clk = 0;

	acsr = ACSR;

	ro_s = (acsr >> 26) & 0x1;
	if (ro_s) {
		smc_clk = 15 * 1000 * 1000;
	} else {
		smc_s = (acsr >> 23) & 0x3;
		switch (smc_s) {
		case 0x0:
			smc_clk = 78 * 1000 * 1000;
			break;
		case 0x2:
			smc_clk = 104 * 1000 * 1000;
			break;
		case 0x5:
			smc_clk = 208 * 1000 * 1000;
			break;
		}
	}

	return smc_clk;
}

void clk_pxa3xx_cken_enable(struct clk *clk)
{
	int hsio2_enable = 0;
	unsigned long mask = 1ul << (clk->cken & 0x1f);

	if (clk->cken < 32)
		CKENA |= mask;
	else if (clk->cken < 64)
		CKENB |= mask;
#ifdef CONFIG_CPU_PXA935
	else
		CKENC |= mask;
#endif

	if (clk->cken == CKEN_USB2)
		hsio2_enable++;

	if (cpu_is_pxa300() || cpu_is_pxa310()) {
		if (clk->cken == CKEN_GCU_PXA300)
			hsio2_enable++;
	} else {
		if (clk->cken == CKEN_GCU_PXA320)
			hsio2_enable++;
	}

	if (cpu_is_pxa310()) {
		if (clk->cken == CKEN_MVED)
			hsio2_enable++;
	}

	if (hsio2_enable)
		CKENB |= (0x1u << (CKEN_HSIO2-32));
}

void clk_pxa3xx_cken_disable(struct clk *clk)
{
	int hsio2_enable = 0;
	unsigned long mask = 1ul << (clk->cken & 0x1f);

	if (clk->cken < 32)
		CKENA &= ~mask;
	else if (clk->cken < 64)
		CKENB &= ~mask;
#ifdef CONFIG_CPU_PXA935
	else
		CKENC &= ~mask;
#endif

	if (CKENA & (0x1u << CKEN_USB2))
		hsio2_enable++;

	if (cpu_is_pxa300() || cpu_is_pxa310()) {
		if (CKENB & (0x1u << (CKEN_GCU_PXA300 - 32)))
			hsio2_enable++;
	} else {
		if (CKENA & (0x1u << CKEN_GCU_PXA320))
			hsio2_enable++;
	}

	if (cpu_is_pxa310()) {
		if (CKENB & (0x1u << (CKEN_MVED - 32)))
			hsio2_enable++;
	}

	if (!hsio2_enable)
		CKENB &= ~(0x1u << (CKEN_HSIO2-32));
}

/* enable both SMC & NAND clock, because NAND clock relies on SMC clock */
void clk_pxa3xx_smc_enable(struct clk *smc_clk)
{
	CKENA |= (1 << CKEN_SMC) | (1 << CKEN_NAND);
}

/* disable both SMC & NAND clock, because NAND clock relies on SMC clock */
void clk_pxa3xx_smc_disable(struct clk *smc_clk)
{
	CKENA &= ~((1 << CKEN_SMC) | (1 << CKEN_NAND));
}

void clk_pxa3xx_cken_null(struct clk *clk)
{
}

const struct clkops clk_pxa3xx_cken_ops = {
	.enable		= clk_pxa3xx_cken_enable,
	.disable	= clk_pxa3xx_cken_disable,
};

static const struct clkops clk_pxa3xx_hsio_ops = {
	.enable		= clk_pxa3xx_cken_enable,
	.disable	= clk_pxa3xx_cken_disable,
	.getrate	= clk_pxa3xx_hsio_getrate,
};

static const struct clkops clk_pxa3xx_ac97_ops = {
	.enable		= clk_pxa3xx_cken_enable,
	.disable	= clk_pxa3xx_cken_disable,
	.getrate	= clk_pxa3xx_ac97_getrate,
};

static unsigned long oscc_pout_refcount;

void enable_oscc_pout(void)
{
	unsigned long val;
	unsigned long flags;

	local_irq_save(flags);
	if (!oscc_pout_refcount++) {
		val = __raw_readl((void *)&OSCC);
		val |= OSCC_PEN;
		__raw_writel(val, (void *)&OSCC);
	}
	local_irq_restore(flags);
	return;
}

void disable_oscc_pout(void)
{
	unsigned long val;
	unsigned long flags;

	local_irq_save(flags);
	if (!oscc_pout_refcount) {
		printk("WARNING: oscc pout count had been zero\n");
	} else if (!--oscc_pout_refcount) {
		val = __raw_readl((void *)&OSCC);
		val &= ~OSCC_PEN;
		__raw_writel(val, (void *)&OSCC);
	}
	local_irq_restore(flags);
	return;
}

static unsigned long oscc_tout_s0_refcount;

void enable_oscc_tout_s0(void)
{
	unsigned long val;
	unsigned long flags;

	local_irq_save(flags);
	if (!oscc_tout_s0_refcount++) {
		val = __raw_readl((void *)&OSCC);
		val |= OSCC_TENS0;
		__raw_writel(val, (void *)&OSCC);
	}
	local_irq_restore(flags);
	return;
}

void disable_oscc_tout_s0(void)
{
	unsigned long val;
	unsigned long flags;

	local_irq_save(flags);
	if (!oscc_tout_s0_refcount) {
		printk("WARNING: oscc tout count had been zero\n");
	} else if (!--oscc_tout_s0_refcount) {
		val = __raw_readl((void *)&OSCC);
		val &= ~OSCC_TENS0;
		__raw_writel(val, (void *)&OSCC);
	}
	local_irq_restore(flags);
	return;
}

static const struct clkops clk_pxa3xx_isc_ops = {
	.enable		= clk_pxa3xx_cken_enable,
	.disable	= clk_pxa3xx_cken_disable,
	.getrate	= clk_pxa3xx_isc_getrate,
};

#ifdef CONFIG_MTD_NAND_PXA3xx_FIX1
/* When this workaround is enabled, clock gating workaround can't be used. */
static const struct clkops clk_pxa3xx_nand_ops = {
	.enable		= clk_pxa3xx_cken_enable,
	.disable	= clk_pxa3xx_cken_disable,
	.getrate	= clk_pxa3xx_nand_getrate,
};

static const struct clkops clk_pxa3xx_smc_ops = {
	.enable		= clk_pxa3xx_cken_null,
	.disable	= clk_pxa3xx_cken_null,
	.getrate	= clk_pxa3xx_smc_getrate,
};
#else
/*
 * SMC clock ops will also operate NAND clock.
 * If NAND driver wants to control NAND clock directly, clock gating on
 * SMC/NAND should be avoided.
 */
static const struct clkops clk_pxa3xx_nand_ops = {
	.enable		= clk_pxa3xx_cken_null,
	.disable	= clk_pxa3xx_cken_null,
	.getrate	= clk_pxa3xx_nand_getrate,
};

static const struct clkops clk_pxa3xx_smc_ops = {
	.enable		= clk_pxa3xx_smc_enable,
	.disable	= clk_pxa3xx_smc_disable,
	.getrate	= clk_pxa3xx_smc_getrate,
};
#endif

static void clk_pout_enable(struct clk *clk)
{
	OSCC |= OSCC_PEN;
}

static void clk_pout_disable(struct clk *clk)
{
	OSCC &= ~OSCC_PEN;
}

static const struct clkops clk_pout_ops = {
	.enable		= clk_pout_enable,
	.disable	= clk_pout_disable,
};

static void clk_dummy_enable(struct clk *clk)
{
}

static void clk_dummy_disable(struct clk *clk)
{
}

static const struct clkops clk_dummy_ops = {
	.enable		= clk_dummy_enable,
	.disable	= clk_dummy_disable,
};

static struct clk pxa3xx_clks[] = {
	{
		.name           = "CLK_POUT",
		.ops            = &clk_pout_ops,
		.rate           = 13000000,
		.delay          = 70,
	},

	/* Power I2C clock is always on */
	{
		.name		= "I2CCLK",
		.ops		= &clk_dummy_ops,
		.dev		= &pxa3xx_device_i2c_power.dev,
	},

	PXA3xx_CK("LCDCLK",  LCD,    &clk_pxa3xx_hsio_ops, &pxa_device_fb.dev),
	PXA3xx_CK("CAMCLK",  CAMERA, &clk_pxa3xx_hsio_ops, NULL),
	PXA3xx_CK("NANDCLK", NAND, &clk_pxa3xx_nand_ops, NULL),
	PXA3xx_CK("ISCCLK", ISC, &clk_pxa3xx_isc_ops, NULL),
	PXA3xx_CK("SMCCLK", SMC, &clk_pxa3xx_smc_ops, NULL),

	PXA3xx_CKEN("UARTCLK", FFUART, 14857000, 1, &pxa_device_ffuart.dev),
	PXA3xx_CKEN("UARTCLK", BTUART, 14857000, 1, &pxa_device_btuart.dev),
	PXA3xx_CKEN("UARTCLK", STUART, 14857000, 1, NULL),

	PXA3xx_CKEN("I2CCLK", I2C,  32842000, 0, &pxa_device_i2c.dev),
	PXA3xx_CKEN("UDCCLK", UDC,  48000000, 5, &pxa27x_device_udc.dev),
 	PXA3xx_CKEN("U2DCLK", USB2,  48000000, 5, &pxa3xx_device_u2d.dev),
	PXA3xx_CKEN("USBCLK", USBH, 48000000, 0, &pxa27x_device_ohci.dev),
	PXA3xx_CKEN("KBDCLK", KEYPAD,  32768, 0, &pxa27x_device_keypad.dev),
	PXA3xx_CKEN("ds1wm", 1WIRE, 13000000, 0, &pxa3xx_device_1wire.dev),
	PXA3xx_CK("AC97CLK", AC97,   &clk_pxa3xx_ac97_ops, NULL),
	PXA3xx_CKEN("WTMCLK", TPM, 130000000, 0, NULL),

	PXA3xx_CKEN("SSP1CLK", SSP1, 13000000, 0, &pxa27x_device_ssp1.dev),
	PXA3xx_CKEN("SSP2CLK", SSP2, 13000000, 0, &pxa27x_device_ssp2.dev),
	PXA3xx_CKEN("SSP3CLK", SSP3, 13000000, 0, &pxa27x_device_ssp3.dev),
	PXA3xx_CKEN("SSP4CLK", SSP4, 13000000, 0, &pxa3xx_device_ssp4.dev),
	PXA3xx_CKEN("PWMCLK", PWM0, 13000000, 0, &pxa27x_device_pwm0.dev),
	PXA3xx_CKEN("PWMCLK", PWM1, 13000000, 0, &pxa27x_device_pwm1.dev),

	PXA3xx_CKEN("GPIOCLK", GPIO, 0, 0, NULL),
	PXA3xx_CKEN("INTCLK", INTC, 0, 0, NULL),
	PXA3xx_CKEN("BOOTCLK", BOOT, 0, 0, NULL),
	PXA3xx_CKEN("HSIO2CLK", HSIO2, 0, 0, NULL),
	PXA3xx_CKEN("MINLCDCLK", MINI_LCD, 0, 0, NULL),
	PXA3xx_CKEN("MINIMCLK", MINI_IM, 0, 0, NULL),
	PXA3xx_CKEN("PWM0CLK", PWM0, 0, 0, NULL),
	PXA3xx_CKEN("PWM1CLK", PWM1, 0, 0, NULL),
	PXA3xx_CKEN("MSLCLK", MSL0, 0, 0, NULL),
};

static struct clk pxa300_gcu_clk[] = {
	PXA3xx_CKEN("M2DCLK", GCU_PXA300, 104000000, 0, &pxa3xx_device_gcu.dev),
	PXA3xx_CKEN("HSIOGCUCLK", HSIOGCU, 0, 0, &pxa3xx_device_gcu.dev),
};

static struct clk pxa320_gcu_clk[] = {
	PXA3xx_CKEN("M2DCLK", GCU_PXA320, 104000000, 0, &pxa3xx_device_gcu.dev),
	PXA3xx_CKEN("HSIOGCUCLK", HSIOGCU, 0, 0, &pxa3xx_device_gcu.dev),
};

void __init pxa3xx_init_irq(void)
{
	/* enable CP6 access */
	u32 value;
	__asm__ __volatile__("mrc p15, 0, %0, c15, c1, 0\n": "=r"(value));
	value |= (1 << 6);
	__asm__ __volatile__("mcr p15, 0, %0, c15, c1, 0\n": :"r"(value));

	pxa_init_irq(94, NULL);
	pxa_init_gpio(128, NULL);
}

/*
 * device registration specific to PXA3xx.
 */

static u64 pxa3xx_onenand_dma_mask = DMA_BIT_MASK(32);

static struct resource pxa3xx_resources_onenand[] = {
	[0] = {
		.start  = 0x10000000,
		.end    = 0x100fffff,
		.flags  = IORESOURCE_MEM,
	},
};

struct platform_device pxa3xx_device_onenand = {
	.name		= "onenand",
	.id		= -1,
	.dev		=  {
		.dma_mask	= &pxa3xx_onenand_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= pxa3xx_resources_onenand,
	.num_resources	= ARRAY_SIZE(pxa3xx_resources_onenand),
};

static u64 pxa3xx_nand_dma_mask = DMA_BIT_MASK(32);

static struct resource pxa3xx_resource_nand[] = {
	[0] = {
		.start	= 0x43100000,
		.end	= 0x431000ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_NAND,
		.end	= IRQ_NAND,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa3xx_device_nand = {
	.name		= "pxa3xx-nand",
	.id		= -1,
	.dev		= {
		.dma_mask = &pxa3xx_nand_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= pxa3xx_resource_nand,
	.num_resources	= ARRAY_SIZE(pxa3xx_resource_nand),
};

static struct resource i2c_power_resources[] = {
	{
		.start  = 0x40f500c0,
		.end    = 0x40f500d3,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_PWRI2C,
		.end	= IRQ_PWRI2C,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa3xx_device_i2c_power = {
	.name		= "pxa2xx-i2c",
	.id		= 1,
	.resource	= i2c_power_resources,
	.num_resources	= ARRAY_SIZE(i2c_power_resources),
};

void __init pxa3xx_set_i2c_power_info(struct i2c_pxa_platform_data *info)
{
	pxa3xx_device_i2c_power.dev.platform_data = info;
}

static struct pxa3xx_u2d_mach_info pxa_u2d_info;

void __init pxa_set_u2d_info(struct pxa3xx_u2d_mach_info *info)
{
	pxa_register_device(&pxa3xx_device_u2d, info);
}

static u64 u2d_dma_mask = ~(u32)0;

static struct resource pxa3xx_u2d_resources[] = {
	[0] = {
		.start	= 0x54100000,
		.end	= 0x5410ffff,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device pxa3xx_device_u2d = {
	.name		= "pxa3xx-u2d",
	.id		= -1,
	.resource	= pxa3xx_u2d_resources,
	.num_resources	= ARRAY_SIZE(pxa3xx_u2d_resources),
	.dev		=  {
		.platform_data	= &pxa_u2d_info,
		.dma_mask	= &u2d_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	}
};

static struct resource pxa3xx_resource_cam[] = {
	[0] = {
		.start	= 0x50000000,
		.end	= 0x5000ffff,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device pxa3xx_device_cam = {
	.name		= "pxa3xx-camera",
	.id		= -1,
	.resource	= pxa3xx_resource_cam,
	.num_resources	= ARRAY_SIZE(pxa3xx_resource_cam),
};
 

struct platform_device pxa3xx_device_otg = {
	.name	=	"pxa3xx-otg",
	.id	=	-1,
};

static struct resource pxa3xx_mved_resources[] = {
	[0] = {
		.start	= 0x56000000,
		.end	= 0x5603ffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 0x56100000,
		.end	= 0x56100fff,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= 0x56200000,
		.end	= 0x562000ff,
		.flags	= IORESOURCE_MEM,
	},
	[3] = {
		.start	= IRQ_MVED,
		.end	= IRQ_MVED,
		.flags	= IORESOURCE_IRQ,
	},
	[4] = {
		.start	= IRQ_MVED_DMA,
		.end	= IRQ_MVED_DMA,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa3xx_device_mved= {
	.name		= "pxa3xx-mved",
	.id		= -1,
	.resource	= pxa3xx_mved_resources,
	.num_resources	= ARRAY_SIZE(pxa3xx_mved_resources),
};

struct platform_device pxa3xx_device_watchdog= {
	.name		= "pxa3xx-watchdog",
	.id		= -1,
};

static struct platform_device *devices[] __initdata = {
#if 0 /* we donot use pwr i2c, rm by frank */
	&pxa3xx_device_i2c_power,
#endif
	&pxa27x_device_pwm0,
	&pxa27x_device_pwm1,
	&pxa_device_ffuart,
	&pxa_device_btuart,
	&pxa_device_stuart,
	&pxa_device_i2s,
	&pxa_device_v4l2ov2,
	&pxa3xx_device_rtc,
	&pxa3xx_device_otg,
	&pxa27x_device_ssp1,
	&pxa27x_device_ssp2,
	&pxa27x_device_ssp3,
	&pxa3xx_device_ssp4,
	&pxa3xx_device_1wire,
	&pxa3xx_device_gcu,
	&pxa3xx_device_mved,
#ifdef CONFIG_USB_ANDROID
	&android_usb_device,
#endif
	&pxa3xx_device_watchdog,
};

static struct i2c_pxa_platform_data pxa3xx_i2c_platform_data = {
	.use_pio	= 1,
#ifdef CONFIG_TOUCHSCREEN_TEKOM
	.fast_mode = 0,
#else
	.fast_mode = 1, /* enable by frank */
#endif
};

static int __init pxa3xx_init(void)
{
	int ret = 0;

	if (cpu_is_pxa320() || cpu_is_pxa930()) {
		clks_register(pxa320_gcu_clk, ARRAY_SIZE(pxa320_gcu_clk));
		CKENA &= ~(1 << CKEN_GCU_PXA320);
	} else {
		clks_register(pxa300_gcu_clk, ARRAY_SIZE(pxa300_gcu_clk));
		CKENA &= ~(1 << CKEN_GCU_PXA300);
	}

	if (cpu_is_pxa3xx()) {
		clks_register(pxa3xx_clks, ARRAY_SIZE(pxa3xx_clks));

		if ((ret = pxa_init_dma(32)))
			return ret;

		/* register i2c device */
		pxa_set_i2c_info(&pxa3xx_i2c_platform_data);

		return platform_add_devices(devices, ARRAY_SIZE(devices));
	}
	return 0;
}

postcore_initcall(pxa3xx_init);
