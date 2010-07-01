/*
 * linux/arch/arm/mach-pxa/pxa930.c
 *
 * Code specific to PXA930
 *
 * Copyright (C) 2007-2008 Marvell Internation Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/dma-mapping.h>

#include <mach/hardware.h>
#include <mach/pxa3xx-regs.h>
#include <mach/mfp-pxa9xx.h>
#include <mach/pxa3xx_pm.h>
#include <mach/pxa9xx_u2o.h>
#include <asm/mach-types.h>
#include <asm/delay.h>
#include "generic.h"
#include "devices.h"
#include "clock.h"

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)
static int comm_v75 = 0;
static int __init comm_v75_setup(char *__unused)
{
	comm_v75 = 1;
	return 1;
}
__setup("comm_v75", comm_v75_setup);

int is_comm_v75(void)
{
	return comm_v75;
}
EXPORT_SYMBOL(is_comm_v75);

static int uart_gpio = 0;
static int __init uart_gpio_setup(char *__unused)
{
	uart_gpio = 1;
	return 0;
}
__setup("uart_gpio", uart_gpio_setup);

int is_uart_gpio(void)
{
	return uart_gpio;
}

static struct pxa3xx_mfp_addr_map pxa930_mfp_addr_map[] __initdata = {

	MFP_ADDR(GPIO0, 0x02e0),
	MFP_ADDR(GPIO1, 0x02dc),
	MFP_ADDR(GPIO2, 0x02e8),
	MFP_ADDR(GPIO3, 0x02d8),
	MFP_ADDR(GPIO4, 0x02e4),
	MFP_ADDR(GPIO5, 0x02ec),
	MFP_ADDR(GPIO6, 0x02f8),
	MFP_ADDR(GPIO7, 0x02fc),
	MFP_ADDR(GPIO8, 0x0300),
	MFP_ADDR(GPIO9, 0x02d4),
	MFP_ADDR(GPIO10, 0x02f4),
	MFP_ADDR(GPIO11, 0x02f0),
	MFP_ADDR(GPIO12, 0x0304),
	MFP_ADDR(GPIO13, 0x0310),
	MFP_ADDR(GPIO14, 0x0308),
	MFP_ADDR(GPIO15, 0x030c),
	MFP_ADDR(GPIO16, 0x04e8),
	MFP_ADDR(GPIO17, 0x04f4),
	MFP_ADDR(GPIO18, 0x04f8),
	MFP_ADDR(GPIO19, 0x04fc),
	MFP_ADDR(GPIO20, 0x0518),
	MFP_ADDR(GPIO21, 0x051c),
	MFP_ADDR(GPIO22, 0x04ec),
	MFP_ADDR(GPIO23, 0x0500),
	MFP_ADDR(GPIO24, 0x04f0),
	MFP_ADDR(GPIO25, 0x0504),
	MFP_ADDR(GPIO26, 0x0510),
	MFP_ADDR(GPIO27, 0x0514),
	MFP_ADDR(GPIO28, 0x0520),
	MFP_ADDR(GPIO29, 0x0600),
	MFP_ADDR(GPIO30, 0x0618),
	MFP_ADDR(GPIO31, 0x0610),
	MFP_ADDR(GPIO32, 0x060c),
	MFP_ADDR(GPIO33, 0x061c),
	MFP_ADDR(GPIO34, 0x0620),
	MFP_ADDR(GPIO35, 0x0628),
	MFP_ADDR(GPIO36, 0x062c),
	MFP_ADDR(GPIO37, 0x0630),
	MFP_ADDR(GPIO38, 0x0634),
	MFP_ADDR(GPIO39, 0x0638),
	MFP_ADDR(GPIO40, 0x063c),
	MFP_ADDR(GPIO41, 0x0614),
	MFP_ADDR(GPIO42, 0x0624),
	MFP_ADDR(GPIO43, 0x0608),
	MFP_ADDR(GPIO44, 0x0604),
	MFP_ADDR(GPIO45, 0x050c),
	MFP_ADDR(GPIO46, 0x0508),
	MFP_ADDR(GPIO47, 0x02bc),
	MFP_ADDR(GPIO48, 0x02b4),
	MFP_ADDR(GPIO49, 0x02b8),
	MFP_ADDR(GPIO50, 0x02c8),
	MFP_ADDR(GPIO51, 0x02c0),
	MFP_ADDR(GPIO52, 0x02c4),
	MFP_ADDR(GPIO53, 0x02d0),
	MFP_ADDR(GPIO54, 0x02cc),
	MFP_ADDR(GPIO55, 0x029c),
	MFP_ADDR(GPIO56, 0x02a0),
	MFP_ADDR(GPIO57, 0x0294),
	MFP_ADDR(GPIO58, 0x0298),
	MFP_ADDR(GPIO59, 0x02a4),
	MFP_ADDR(GPIO60, 0x02a8),
	MFP_ADDR(GPIO61, 0x02b0),
	MFP_ADDR(GPIO62, 0x02ac),
	MFP_ADDR(GPIO63, 0x0640),
	MFP_ADDR(GPIO64, 0x065c),
	MFP_ADDR(GPIO65, 0x0648),
	MFP_ADDR(GPIO66, 0x0644),
	MFP_ADDR(GPIO67, 0x0674),
	MFP_ADDR(GPIO68, 0x0658),
	MFP_ADDR(GPIO69, 0x0654),
	MFP_ADDR(GPIO70, 0x0660),
	MFP_ADDR(GPIO71, 0x0668),
	MFP_ADDR(GPIO72, 0x0664),
	MFP_ADDR(GPIO73, 0x0650),
	MFP_ADDR(GPIO74, 0x066c),
	MFP_ADDR(GPIO75, 0x064c),
	MFP_ADDR(GPIO76, 0x0670),
	MFP_ADDR(GPIO77, 0x0678),
	MFP_ADDR(GPIO78, 0x067c),
	MFP_ADDR(GPIO79, 0x0694),
	MFP_ADDR(GPIO80, 0x069c),
	MFP_ADDR(GPIO81, 0x06a0),
	MFP_ADDR(GPIO82, 0x06a4),
	MFP_ADDR(GPIO83, 0x0698),
	MFP_ADDR(GPIO84, 0x06bc),
	MFP_ADDR(GPIO85, 0x06b4),
	MFP_ADDR(GPIO86, 0x06b0),
	MFP_ADDR(GPIO87, 0x06c0),
	MFP_ADDR(GPIO88, 0x06c4),
	MFP_ADDR(GPIO89, 0x06ac),
	MFP_ADDR(GPIO90, 0x0680),
	MFP_ADDR(GPIO91, 0x0684),
	MFP_ADDR(GPIO92, 0x0688),
	MFP_ADDR(GPIO93, 0x0690),
	MFP_ADDR(GPIO94, 0x068c),
	MFP_ADDR(GPIO95, 0x06a8),
	MFP_ADDR(GPIO96, 0x06b8),
	MFP_ADDR(GPIO97, 0x0418),
	MFP_ADDR(GPIO98, 0x0410),
	MFP_ADDR(GPIO99, 0x041c),
	MFP_ADDR(GPIO100, 0x0414),
	MFP_ADDR(GPIO101, 0x0408),
	MFP_ADDR(GPIO102, 0x0324),
	MFP_ADDR(GPIO103, 0x040c),
	MFP_ADDR(GPIO104, 0x0400),
	MFP_ADDR(GPIO105, 0x0328),
	MFP_ADDR(GPIO106, 0x0404),

	MFP_ADDR(nXCVREN, 0x0204),
	MFP_ADDR(DF_CLE_nOE, 0x020c),
	MFP_ADDR(DF_nADV1_ALE, 0x0218),
	MFP_ADDR(DF_SCLK_E, 0x0214),
	MFP_ADDR(DF_SCLK_S, 0x0210),
	MFP_ADDR(nBE0, 0x021c),
	MFP_ADDR(nBE1, 0x0220),
	MFP_ADDR(DF_nADV2_ALE, 0x0224),
	MFP_ADDR(DF_INT_RnB, 0x0228),
	MFP_ADDR(DF_nCS0, 0x022c),
	MFP_ADDR(DF_nCS1, 0x0230),
	MFP_ADDR(nLUA, 0x0254),
	MFP_ADDR(nLLA, 0x0258),
	MFP_ADDR(DF_nWE, 0x0234),
	MFP_ADDR(DF_nRE_nOE, 0x0238),
	MFP_ADDR(DF_ADDR0, 0x024c),
	MFP_ADDR(DF_ADDR1, 0x0250),
	MFP_ADDR(DF_ADDR2, 0x025c),
	MFP_ADDR(DF_ADDR3, 0x0260),
	MFP_ADDR(DF_IO0, 0x023c),
	MFP_ADDR(DF_IO1, 0x0240),
	MFP_ADDR(DF_IO2, 0x0244),
	MFP_ADDR(DF_IO3, 0x0248),
	MFP_ADDR(DF_IO4, 0x0264),
	MFP_ADDR(DF_IO5, 0x0268),
	MFP_ADDR(DF_IO6, 0x026c),
	MFP_ADDR(DF_IO7, 0x0270),
	MFP_ADDR(DF_IO8, 0x0274),
	MFP_ADDR(DF_IO9, 0x0278),
	MFP_ADDR(DF_IO10, 0x027c),
	MFP_ADDR(DF_IO11, 0x0280),
	MFP_ADDR(DF_IO12, 0x0284),
	MFP_ADDR(DF_IO13, 0x0288),
	MFP_ADDR(DF_IO14, 0x028c),
	MFP_ADDR(DF_IO15, 0x0290),

	MFP_ADDR(GSIM_UIO, 0x0314),
	MFP_ADDR(GSIM_UCLK, 0x0318),
	MFP_ADDR(GSIM_UDET, 0x031c),
	MFP_ADDR(GSIM_nURST, 0x0320),

	MFP_ADDR(PMIC_INT, 0x06c8),

	MFP_ADDR(RDY, 0x0200),

	MFP_ADDR_END,
};

#ifdef CONFIG_CPU_PXA935
static struct pxa3xx_mfp_addr_map pxa935_mfp_addr_map[] __initdata = {
	MFP_ADDR(GPIO159, 0x0524),
	MFP_ADDR(GPIO163, 0x0534),
	MFP_ADDR(GPIO167, 0x0544),
	MFP_ADDR(GPIO168, 0x0548),
	MFP_ADDR(GPIO169, 0x054c),
	MFP_ADDR(GPIO170, 0x0550),
	MFP_ADDR(GPIO171, 0x0554),
	MFP_ADDR(GPIO172, 0x0558),
	MFP_ADDR(GPIO173, 0x055c),

	MFP_ADDR_END,
};
#endif

/*
 * Tavor PV series processors device configuration
 *                    U2O/U2H  U2D  UDC/UHC  MMC3-5  MMC1/2  MVED
 *  P                    N      Y      Y        N      Y      N
 *  PV-1.1/P65e1.1       N	N      Y        N      Y      N
 *  PV-/P65e (PXA935)    Y      N      N        N      Y      N
 *  PV (PXA940)          Y      N      N        Y      N      Y
 */
#define DEVICES_ON_TAVOR_P	(device_u2d | device_udc | device_mmc | device_uhc)

int device_is_enabled(int device)
{

	if (cpu_is_pxa930()) {
		if (device & DEVICES_ON_TAVOR_P)
			return 1;
	}
#ifdef CONFIG_CPU_PXA935

#define SER_FUSE_REG6  		__REG(0x40f50218)
#define DEVICE_DIS_MASK		0x07FFFFFC

	if (cpu_is_pxa935()) {
		int device_dis;

		device_dis  = SER_FUSE_REG6 & DEVICE_DIS_MASK;

		//FIXME: hard coding to enable old mmc controller
//#define CONFIG_FORCE_FUSE_MMC_OLD
#ifdef CONFIG_FORCE_FUSE_MMC_OLD
		device_dis |= device_mmc_new;
#else
		if (!(device_dis & device_mmc_new))
			device_dis |= device_mmc;
#endif

		if (!(device & device_dis))
			return 1;
	}

#endif
	return 0;
}

static struct resource pxa930_resources_trkball[] = {
	[0] = {
		.start	= 0x42404000,
		.end	= 0x424040ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_TRKBALL,
		.end	= IRQ_TRKBALL,
		.flags	= IORESOURCE_IRQ,
	}
};

struct platform_device pxa930_device_trkball = {
	.name 		= "pxa930-trkball",
	.id 		= 0,
	.num_resources	= ARRAY_SIZE(pxa930_resources_trkball),
	.resource	= pxa930_resources_trkball,
};

static u64 pxa930_acipc_dma_mask = DMA_BIT_MASK(32);

static struct resource pxa930_resource_acipc[] = {
	[0] = {
		.start	= 0x42403000,
		.end	= 0x424030ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_ACIPC0,
		.end	= IRQ_ACIPC0,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= IRQ_ACIPC1,
		.end	= IRQ_ACIPC1,
		.flags	= IORESOURCE_IRQ,
	},
	[3] = {
		.start	= IRQ_ACIPC2,
		.end	= IRQ_ACIPC2,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa930_acipc_device = {
	.name		= "pxa930-acipc",
	.id		= -1,
	.dev		= {
		.dma_mask = &pxa930_acipc_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.resource	= pxa930_resource_acipc,
	.num_resources	= ARRAY_SIZE(pxa930_resource_acipc),
};

static struct platform_device *devices[] __initdata = {
	&pxa930_acipc_device,
};

static unsigned char __iomem *bpb_membase;

/*
 * Query whether the specified pin waked up system.
 *
 * return 1 -- yes, 0 -- no, negative -- failure
 */
int pxa930_query_gwsr(int pin)
{
	int off, ret;
	
	if (pin < 0)
		return -EINVAL;
	off = pin / 32 + 1;
	/* read data from GWSR(x), x is [1..4] */
	ret = __raw_readl(bpb_membase + GWSR(off));
	ret = (ret >> (pin - (off - 1)* 32)) & 1;
	return ret;
}

static struct clk pxa930_mmc_clks[] = {
	PXA3xx_CKEN("MMCCLK", MMC1, 19500000, 0, &pxa_device_mci.dev),
	PXA3xx_CKEN("MMCCLK", MMC2, 19500000, 0, &pxa3xx_device_mci2.dev),
};

#ifdef CONFIG_CPU_PXA935
static struct clk pxa940_mved_clks[] = {
	PXA3xx_CKEN("MVEDCLK", MVED, 152000000, 0, &pxa3xx_device_mved.dev),
};

static void clk_pxa9xx_mmc_enable(struct clk *clk)
{
	unsigned long mask = 1ul << (clk->cken & 0x1f);

	local_irq_disable();

	CKENC |= CKEN_AXI | mask;

	local_irq_enable();
}

static void clk_pxa9xx_mmc_disable(struct clk *clk)
{
	unsigned long mask = 1ul << (clk->cken & 0x1f);

	local_irq_disable();

	CKENC &= ~mask;

	local_irq_enable();
}

static const struct clkops clk_pxa9xx_mmc_ops = {
	.enable		= clk_pxa9xx_mmc_enable,
	.disable	= clk_pxa9xx_mmc_disable,
};

static struct clk pxa9xx_mmc_clks[] = {
	PXA3xx_CK("PXA9XX_MMCCLK", MMC3_AXI, &clk_pxa9xx_mmc_ops, &pxa9xx_device_mci3.dev),
	PXA3xx_CK("PXA9XX_MMCCLK", MMC4_AXI, &clk_pxa9xx_mmc_ops, &pxa9xx_device_mci4.dev),
	PXA3xx_CK("PXA9XX_MMCCLK", MMC5_AXI, &clk_pxa9xx_mmc_ops, &pxa9xx_device_mci5.dev),
};

/********************************************************************
 * USB 2.0 OTG controller 
 */

static void clk_pxa9xx_u2o_enable(struct clk *clk)
{
	unsigned long mask = 1ul << (clk->cken & 0x1f);

	local_irq_disable();

	CKENC |= mask;
	/* ACCR1 */
	ACCR1 |= ACCR1_PU_OTG|ACCR1_PU_PLL|ACCR1_PU;

	local_irq_enable();
}

static void clk_pxa9xx_u2o_disable(struct clk *clk)
{
	unsigned long mask = 1ul << (clk->cken & 0x1f);

	local_irq_disable();

	CKENC &= ~mask;
	/* ACCR1 */
	ACCR1 &= ~(ACCR1_PU_OTG|ACCR1_PU_PLL|ACCR1_PU);

	local_irq_enable();
}

static const struct clkops clk_pxa9xx_u2o_ops = {
	.enable		= clk_pxa9xx_u2o_enable,
	.disable	= clk_pxa9xx_u2o_disable,
};

static struct clk pxa9xx_u2o_clk[] = {
	PXA3xx_CK("U2OCLK", U2O, &clk_pxa9xx_u2o_ops, NULL),
};


static struct pxa9xx_u2o_mach_info pxa_u2o_info;

static u64 u2o_dma_mask = ~(u32)0;

static struct resource pxa9xx_u2o_resources[] = {
	[0] = {
		.start	= 0x55500000,
		.end	= 0x5550ffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_U2O,
		.end	= IRQ_U2O,
		.flags	= IORESOURCE_IRQ,
	},

};

static struct platform_device pxa9xx_device_u2o = {
	.name		= "pxa9xx-u2o",
	.id		= -1,
	.resource	= pxa9xx_u2o_resources,
	.num_resources	= ARRAY_SIZE(pxa9xx_u2o_resources),
	.dev		=  {
		.platform_data	= &pxa_u2o_info,
		.dma_mask	= &u2o_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	}
};

static struct resource pxa9xx_u2otg_resources[] = {
};

static struct platform_device pxa9xx_device_u2otg = {
	.name		= "pxa9xx-u2otg",
	.id		= -1,
	.resource	= pxa9xx_u2o_resources,
	.num_resources	= ARRAY_SIZE(pxa9xx_u2otg_resources),
	.dev		=  {
		.platform_data	= &pxa_u2o_info,
	}
};

void __init pxa_set_u2o_info(struct pxa9xx_u2o_mach_info *info)
{
	pxa_register_device(&pxa9xx_device_u2o, info);
}

void __init pxa_set_u2otg_info(struct pxa9xx_u2o_mach_info *info)
{
	pxa_register_device(&pxa9xx_device_u2otg, info);
}

void pxa9xx_u2o_xcvr_init(void)
{	
	static int init_done;
	int count;
	volatile unsigned int reg;

	if (init_done) {
		printk("re-init phy\n\n");
		/* return; */
	}

	/* check whether U2PPLL[READY] is set */
	count = 100000;
	do {
		count--;
	} while (!(U2PPLL & U2PPLL_READY) && count);
	if (count <= 0) {
		printk("%s TIMEOUT for U2PPLL[READY]\n", __func__);
	}

	/* PLL VCO and TX Impedance Calibration Timing:
	 *
	 * PU             _____|----------------------------------|
	 * VOCAL STAR     ______________|--|______________________|
	 * REG_RCAL_START ___________________________|--|_________|
	 *                     | 200us  |40| 200us   |40| 200us   |USB PHY READY
	 */

	/* SET REG_ARC_DPDM_MODE BIT OF U2PT0 */
	U2PT0 |= U2PT0_REG_ARC_DPDM_MODE; 
	reg = U2PT0;

	/* U2PPLL */
	U2PPLL &= ~(U2PPLL_ICP_MASK | U2PPLL_KVCO_MASK);
	if (machine_is_saar())
		U2PPLL |= (0x7<<U2PPLL_ICP_SHIFT) | (0x7<<U2PPLL_KVCO_SHIFT) \
			| U2PPLL_KVCO_EXT/* | U2PPLL_CLK_BLK_EN*/;
	else
		U2PPLL |= (0x6<<U2PPLL_ICP_SHIFT) | (0x4<<U2PPLL_KVCO_SHIFT) \
			| U2PPLL_KVCO_EXT | U2PPLL_CLK_BLK_EN;
	reg = U2PPLL;

	/* U2PRX, program squelch threshold to rise minumum 
	 * sensitivity for receiving
	 */
	if (machine_is_saar()) {
		U2PRX &= ~(U2PRX_SQ_THRESH_MASK);
		/* U2PRX |= (0x7 << U2PRX_SQ_THRESH_SHIFT); SV suggest value */
		U2PRX |= (0xf << U2PRX_SQ_THRESH_SHIFT);
		reg = U2PRX;
	}

	/* IVREF */
	U2IVREF &= ~(U2IVREF_BG_VSEL_MASK | U2IVREF_RXVDD18_MASK);
	udelay(200);

	/* U2PTX */
	U2PTX |= U2PTX_RCAL_START;
	reg = U2PTX;
	udelay(200);
	U2PTX &= ~U2PTX_RCAL_START;
	udelay(200);
	
	/* Make sure PHY is ready */
	count = 100000;
	do {
		count--;
	} while (!(U2PPLL & U2PPLL_READY) && count);
	if (count <= 0) {
		printk("%s TIMEOUT for U2PPLL[READY]\n", __func__);
	}
	
	init_done = 1;
}
#endif

/********************************************************************/

static int __init pxa930_init(void)
{
	if (cpu_is_pxa930() || cpu_is_pxa935()) {
		if (cpu_is_pxa930()) {
			clks_register(ARRAY_AND_SIZE(pxa930_mmc_clks));
			CKENA &= ~((1 << CKEN_UDC) | (1 << CKEN_USB2)
				| (1 << CKEN_AC97) | (1 << CKEN_BOOT)
				| (1 << CKEN_CIR) | (1 << CKEN_CAMERA)
				| (1 << CKEN_SSP1) | (1 << CKEN_SSP2)
				| (1 << CKEN_SSP3) | (1 << CKEN_SSP4)
				| (1 << CKEN_USIM0) | (1 << CKEN_USIM1)
				| (1 << CKEN_MMC1) | (1 << CKEN_MMC2)
				| (1 << CKEN_USBH) | (1 << CKEN_TPM));
			CKENB |= (1 << (CKEN_HSIO2 - 32));
	                CKENB &= ~((1 << (CKEN_MINI_IM - 32))
				| (1 << (CKEN_MINI_LCD - 32))
				| (1 << (CKEN_1WIRE - 32))
				| (1 << (CKEN_HSIOGCU - 32))
				| (1 << (CKEN_PWM0 - 32))
				| (1 << (CKEN_PWM1 - 32)));
		}

		if (cpu_is_pxa935()) {
			if (device_is_enabled(device_mmc))
				clks_register(ARRAY_AND_SIZE(pxa930_mmc_clks));

			if (device_is_enabled(device_mmc_new))
				clks_register(ARRAY_AND_SIZE(pxa9xx_mmc_clks));

			if (device_is_enabled(device_mved))
				clks_register(ARRAY_AND_SIZE(pxa940_mved_clks));
			
			if (device_is_enabled(device_u2o))
				clks_register(ARRAY_AND_SIZE(pxa9xx_u2o_clk));

			CKENA &= ~((1 << CKEN_UDC) | (1 << CKEN_USB2)
				| (1 << CKEN_AC97) | (1 << CKEN_BOOT)
				| (1 << CKEN_CIR) | (1 << CKEN_CAMERA)
				| (1 << CKEN_SSP1) | (1 << CKEN_SSP2)
				| (1 << CKEN_SSP3) | (1 << CKEN_SSP4)
				| (1 << CKEN_USIM0) | (1 << CKEN_USIM1)
				| (1 << CKEN_MMC1) | (1 << CKEN_MMC2)
				| (1 << CKEN_USBH));
	                CKENB &= ~((1 << (CKEN_MINI_IM - 32))
				| (1 << (CKEN_MINI_LCD - 32))
				| (1 << (CKEN_MVED - 32))
				| (1 << (CKEN_HSIO2 - 32))
				| (1 << (CKEN_1WIRE - 32))
				| (1 << (CKEN_PWM0 - 32))
				| (1 << (CKEN_PWM1 - 32)));
			CKENC &= ~((1 << (CKEN_U2H2 - 64))
				| (1 << (CKEN_U2H3 - 64))
				| (1 << (CKEN_U2H2_AXI - 64))
				| (1 << (CKEN_U2H3_AXI - 64)));
		}

		bpb_membase = ioremap(BPB_START, BPB_END - BPB_START + 1);
		pxa3xx_init_mfp();
		pxa3xx_mfp_init_addr(pxa930_mfp_addr_map);

		if (cpu_is_pxa935())
			pxa3xx_mfp_init_addr(pxa935_mfp_addr_map);

		return platform_add_devices(devices, ARRAY_SIZE(devices));
	}

	return 0;
}

core_initcall(pxa930_init);
