/*
 * Monahans Power Management Routines
 *
 * Copyright (C) 2004, Intel Corporation(chao.xie@intel.com).
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#undef DEBUG
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/kobject.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <mach/hardware.h>
#include <mach/pxa-regs.h>
#include <mach/pxa3xx-regs.h>
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/pxa3xx_pm.h>
#include <mach/ipmc.h>

#include "sleepwkr.h"

/* mtd.h declares another DEBUG macro definition */
#undef DEBUG
#include <linux/mtd/mtd.h>

/* The first 32KB is reserved and can't be accessed by kernel.
 * This restrict is only valid on BootROM V2.
 */
#define ISRAM_START	0x5c000000

/* MOBM_START should be larger than SRAM_START */
/* MOBM_START is used on MOBM V2.
 * The address is 0x5c014000. It means MOBM will be copied on the address.
 * On MOBM V3, it will be copied on 0x5c013000.
 */
#define MOBM_START	0x5c014000
#define MOBM_SIZE	(32 * 1024)
#define MOBM_OFFSET	8

/* for OVERHEATING */
#define FRQ_TEMP	(HZ)

enum {
	OVH_TEMP_40C = 0,
	OVH_TEMP_80C,
	OVH_TEMP_85C,
	OVH_TEMP_90C,
	OVH_TEMP_95C,
	OVH_TEMP_100C,
	OVH_TEMP_105C,
	OVH_TEMP_110C,
};

#define OVH_OTIS_DEFAULT	OVH_TEMP_100C
#define OVH_OVWF_DEFAULT	OVH_TEMP_105C
#define TSS_THRESHOLD		OVH_OTIS_DEFAULT
static struct timer_list temp_detecting_timer;
static int temp_of_core;

static int isram_size;

/* MOBM V2 is used on PXA320 B0/B1/B2 and PXA300 A0
 * MOBM V3 is used on PXA310 A0
 */
enum {
	PXA3xx_OBM_NULL,
	PXA3xx_OBM_V2,
	PXA3xx_OBM_V3,
	PXA3xx_OBM_INVAL,
};

enum pxa3xx_pm_mode {
	PXA3xx_PM_RUN = 0,
	PXA3xx_PM_IDLE = 1 ,
	PXA3xx_PM_LCDREFRESH = 2,
	PXA3xx_PM_STANDBY = 3,
	PXA3xx_PM_D0CS = 5,
	PXA3xx_PM_SLEEP = 6,
	PXA3xx_PM_DEEPSLEEP = 7,
	PXA3xx_PM_CG = 8,
};

extern struct kobject *power_kobj;

pm_wakeup_src_t wakeup_src;	/* interface to set up wakeup_src */
EXPORT_SYMBOL(wakeup_src);

static pm_wakeup_src_t waked;	/* It records the latest wakeup source */

static struct pxa3xx_peripheral_wakeup_ops *wakeup_ops = NULL;

/* How long we will in sleep mode if duty cycle. */
/*cuilong disable rtc wakeup,it's no use for us! from 58 -> 0*/
unsigned int  pm_sleeptime = 0;  /* In seconds. */
EXPORT_SYMBOL(pm_sleeptime);
unsigned int  pm_msleeptime = 0;	/* In miliseconds. */

/* Flag of reseting CP */
unsigned int pm_cp = 0;

#ifdef CONFIG_IPM
int enable_deepidle = 0;		/* IDLE_D0 -- 0 */
int save_deepidle = 0;

void (*event_notify)(int, int, void *, unsigned int) = NULL;
EXPORT_SYMBOL(event_notify);

#endif
static void (*orig_poweroff)(void) = NULL;

/* low level stanby and lcdrefresh routine need to access DMC regs */
unsigned char __iomem *dmc_membase;
EXPORT_SYMBOL(dmc_membase);
unsigned char __iomem *ost_membase;
EXPORT_SYMBOL(ost_membase);
unsigned char __iomem *pm_membase;
unsigned int __iomem *minilcd_membase;
EXPORT_SYMBOL(minilcd_membase);

extern void pxa3xx_cpu_sleep(unsigned int, unsigned int);
extern void pxa3xx_cpu_resume(void);
extern void pxa3xx_cpu_standby(unsigned int, unsigned int, unsigned int);
extern void pxa3xx_cpu_lcdrefresh(unsigned int, unsigned int, unsigned int);
extern void pxa3xx_init_standby(unsigned int);
extern void enable_oscc_pout(void);
extern void disable_oscc_pout(void);
extern void enable_oscc_tout_s0(void);
extern void disable_oscc_tout_s0(void);

static struct pxa3xx_pm_regs pxa3xx_pm_regs;

static unsigned long pm_state;

extern struct pxa3xx_freq_mach_info littleton_freq_mach_info;
static void yuhua_poweri2c_workaround(int poweri2c_enable)
{	
	u32 pcfr;
	if (littleton_freq_mach_info.flags & PXA3xx_USE_POWER_I2C) {		
		if (poweri2c_enable) {
			pcfr = PCFR;
			pcfr &= 0xF0FFFFFF;
			pcfr |= 0x07000000; 
			PCFR = pcfr;
			
			PVCR |= 0xe0000000; /* Enable FVE,PVE */
		} else {
			pcfr = PCFR;
			pcfr &= 0xF0FFFFFF;
			pcfr |= 0x0C000000; /* some device cannot wakup when pwr_del == 7(1/500 in X2) */
			PCFR = pcfr;
			
			PVCR &= 0x0fffffff; /* disable FVE,PVE,TVE,FVC bit */
			udelay(1); /* wait pwri2c to stable */
		}
		//printk("yuhua_poweri2c_workaround %d, pcfr 0x%x\n", poweri2c_enable, pcfr);
	}
}

int get_pm_state(void)
{
	return pm_state;
}

/*************************************************************************/
/* workaround for bug 2140448 */
static int is_wkr_2140448(void)
{
	unsigned int	cpuid;
	cpuid = read_cpuid(CPUID_ID);
	/* It's PXA320 B1 */
	if ((cpuid & 0x0000FFFF) == 0x00006825)
		return 1;
	/* It's PXA300 A0 */
	if ((cpuid & 0x0000FFFF) == 0x00006880)
		return 1;
	return 0;
}

/*
 * MOBM V2 is applied on chips taped out before PXA310 A0.
 * MOBM V3 is applied on chips taped out after PXA310 A0.
 * MOBM V3 is also applied on PXA310 A0.
 */
static int calc_obm_ver(void)
{
	unsigned int cpuid;
	/* read CPU ID */
	__asm__ (
		"mrc p15, 0, %0, c0, c0, 0\n"
		: "=r" (cpuid)
	);
	if ((cpuid & 0xFFFF0000) != 0x69050000) {
		/* It's not xscale chip. */
		return PXA3xx_OBM_INVAL;
	}
	if ((cpuid & 0x0000FFF0) == 0x00006420) {
		/* It's PXA320 Ax */
		return PXA3xx_OBM_V2;
	}
	if ((cpuid & 0x0000FFF0) == 0x00006820) {
		/* It's PXA320 Bx */
		if ((cpuid & 0x0F) <= 6)
			return PXA3xx_OBM_V2;
		else
			return PXA3xx_OBM_V3;
	}
	if ((cpuid & 0x0000FFF0) == 0x00006880) {
		/* It's PXA300 Ax */
		if ((cpuid & 0x0F) == 0)
			return PXA3xx_OBM_V2;
		else
			return PXA3xx_OBM_V3;
	}
	if ((cpuid & 0x0000FFF0) == 0x00006890) {
		/* It's PXA310 Ax */
		return PXA3xx_OBM_V3;
	}
	return PXA3xx_OBM_INVAL;
}

/* Return the address of OBM in RAM if successful.
 * Otherwise, return negative value.
 */
static int load_obm(void)
{
	void *addr = NULL;
	struct mtd_info *mtd = NULL;
	int obm_ver, retlen;

	mtd = get_mtd_device(NULL, 0);
	if (mtd == NULL || mtd == -ENODEV)
		return -ENODEV;
	addr = kmalloc(MOBM_SIZE, GFP_KERNEL);
	if (!addr)
		return -ENOMEM;

	obm_ver = calc_obm_ver();
	if (obm_ver == PXA3xx_OBM_V2) {
		/* MOBM begins from 0x0000 */
		if (mtd->writesize == 2048)
			mtd->read(mtd, 0x0, MOBM_SIZE, &retlen, addr);
		else {
#if (MOBM_SIZE > 16 * 1024)
			mtd->read(mtd, 0, 0x4000, &retlen, addr);
			mtd->read(mtd, 0x4000, MOBM_SIZE - 0x4000, &retlen,
				addr + 0x3e00);
#else
			mtd->read(mtd, 0, MOBM_SIZE, &retlen, addr);
#endif
		}
		addr += MOBM_OFFSET;
	} else if (obm_ver == PXA3xx_OBM_V3) {
		/* MOBM begins from 0x20000 */
		if (mtd->writesize == 2048)
			mtd->read(mtd, 0x20000, MOBM_SIZE, &retlen, addr);
	}
	pr_debug("load mobm into address: 0x%x\n", (unsigned int)addr);
	return (int)addr;
}

static void pxa3xx_intc_save(struct intc_regs *context)
{
	unsigned int temp, i;

	context->iccr = ICCR;
	for (i = 0; i < 32; i++) {
		context->ipr[i] = __raw_readl(&IPR0 + (i << 2));
	}
	for (i = 0; i < 21; i++) {
		context->ipr2[i] = __raw_readl(&IPR32 + (i << 2));
	}

	/* load registers by accessing co-processor */
	__asm__("mrc\tp6, 0, %0, c1, c0, 0" : "=r" (temp));
	context->icmr = temp;
	__asm__("mrc\tp6, 0, %0, c7, c0, 0" : "=r" (temp));
	context->icmr2 = temp;
	__asm__("mrc\tp6, 0, %0, c2, c0, 0" : "=r" (temp));
	context->iclr = temp;
	__asm__("mrc\tp6, 0, %0, c8, c0, 0" : "=r" (temp));
	context->iclr2 = temp;
}

static void pxa3xx_intc_restore(struct intc_regs *context)
{
	unsigned int temp, i;

	ICCR = context->iccr;
	for (i = 0; i < 32; i++) {
		__raw_writel(context->ipr[i], &IPR0 + (i << 2));
	}
	for (i = 0; i < 21; i++) {
		__raw_writel(context->ipr2[i], &IPR32 + (i << 2));
	}

	temp = context->icmr;
	__asm__("mcr\tp6, 0, %0, c1, c0, 0" : :"r"(temp));
	temp = context->icmr2;
	__asm__("mcr\tp6, 0, %0, c7, c0, 0" : :"r"(temp));
	temp = context->iclr;
	__asm__("mcr\tp6, 0, %0, c2, c0, 0" : :"r"(temp));
	temp = context->iclr2;
	__asm__("mcr\tp6, 0, %0, c8, c0, 0" : :"r"(temp));
}

static void pxa3xx_clk_save(struct clock_regs *context)
{
	context->aicsr = AICSR;
	context->ckena = CKENA;
	context->ckenb = CKENB;
	context->oscc = OSCC;
	/* Disable the processor to use the ring oscillator output clock
	 * as a clock source when transitioning from any low-power mode
	 * to D0 mode.
	 */
	ACCR &= ~ACCR_PCCE;
}

/* workaround for bootrom wrongly cleared OSCR on MHLV-A2 */
static int is_wkr_oscr(void)
{
       unsigned int    cpuid;

       /* read CPU ID */
       cpuid = read_cpuid(0) & 0xFFFF;
       /* It's Monahans LV A2 */
       if (cpuid == 0x6892 || cpuid == 0x6893)
               return 1;
       return 0;
}

static void pxa3xx_clk_restore(struct clock_regs *context)
{
	context->aicsr &= (AICSR_PCIE | AICSR_TCIE | AICSR_FCIE);
	AICSR = context->aicsr;
	CKENA = context->ckena;
	CKENB = context->ckenb;
	OSCC = context->oscc;
}

static void pxa3xx_ost_save(struct ost_regs *context)
{
	context->oscr4 = OSCR4;
	context->omcr4 = OMCR4;
	context->osmr4 = OSMR4;
	context->oier = OIER;
}

static void pxa3xx_ost_restore(struct ost_regs *context)
{
       if (is_wkr_oscr())
               OSCR = context->oscr;
	OSCR4 = context->oscr4;
	OMCR4 = context->omcr4;
	OSMR4 = context->osmr4;
	OIER = context->oier;
}

static void pxa3xx_mfp_save(struct mfp_regs *context)
{
	int i;

	for (i = 0; i < MAX_MFP_PINS; i++)
		context->mfp[i] = __raw_readl(&MFPR0 + (i << 2));
}

static void pxa3xx_mfp_restore(struct mfp_regs *context)
{
	int i;

	for (i = 0; i < MAX_MFP_PINS; i++)
		__raw_writel(context->mfp[i], &MFPR0 + (i << 2));
}

/* The setting of GPIO will be restored.
 * The status of GPIO Edge Status will be lost.
 */
static void pxa3xx_gpio_save(struct gpio_regs *context)
{
	context->gpdr0 = GPDR0;
	context->gpdr1 = GPDR1;
	context->gpdr2 = GPDR2;
	context->gpdr3 = GPDR3;

	context->gplr0 = GPLR0;
	context->gplr1 = GPLR1;
	context->gplr2 = GPLR2;
	context->gplr3 = GPLR3;

	context->grer0 = GRER0;
	context->grer1 = GRER1;
	context->grer2 = GRER2;
	context->grer3 = GRER3;

	context->gfer0 = GFER0;
	context->gfer1 = GFER1;
	context->gfer2 = GFER2;
	context->gfer3 = GFER3;
}

static void pxa3xx_gpio_restore(struct gpio_regs *context)
{
	GPDR0 = context->gpdr0;
	GPDR1 = context->gpdr1;
	GPDR2 = context->gpdr2;
	GPDR3 = context->gpdr3;

	GPSR0 = context->gplr0;
	GPSR1 = context->gplr1;
	GPSR2 = context->gplr2;
	GPSR3 = context->gplr3;
	GPCR0 = ~(context->gplr0);
	GPCR1 = ~(context->gplr1);
	GPCR2 = ~(context->gplr2);
	GPCR3 = ~(context->gplr3);

	GRER0 = context->grer0;
	GRER1 = context->grer1;
	GRER2 = context->grer2;
	GRER3 = context->grer3;

	GFER0 = context->gfer0;
	GFER1 = context->gfer1;
	GFER2 = context->gfer2;
	GFER3 = context->gfer3;
}

static void pxa3xx_sysbus_init(struct pxa3xx_pm_regs *context)
{
	context->smc.membase = ioremap(SMC_START, SMC_END - SMC_START + 1);
	context->arb.membase = ioremap(ARB_START, ARB_END - ARB_START + 1);

	dmc_membase = ioremap(DMC_START, DMC_END - DMC_START + 1);
	ost_membase = ioremap(OST_START, OST_END - OST_START + 1);
	pm_membase = ioremap(PM_START, PM_END - PM_START + 1);

	isram_size = (128 * 2 * 1024);
	if (cpu_is_pxa320())
		isram_size = (128 * 6 * 1024);
	else if (cpu_is_pxa930() || cpu_is_pxa935())
		isram_size = (128 * 1024);

	context->sram_map = ioremap(ISRAM_START, isram_size);
	context->sram = vmalloc(isram_size);
	context->obm = (void *)load_obm();
	/* Two words begun from 0xC0000000 are used to store key information.
	 */
	context->data_pool = (unsigned char *)0xC0000000;
}

static void pxa3xx_sysbus_save(struct pxa3xx_pm_regs *context)
{
	unsigned char __iomem *base = NULL;
	unsigned int tmp;

	/* static memory controller */
	base = context->smc.membase;
	context->smc.msc0 = readl(base + MSC0_OFF);
	context->smc.msc1 = readl(base + MSC1_OFF);
	context->smc.sxcnfg = readl(base + SXCNFG_OFF);
	context->smc.memclkcfg = readl(base + MEMCLKCFG_OFF);
	context->smc.cscfg0 = readl(base + CSADRCFG0_OFF);
	context->smc.cscfg1 = readl(base + CSADRCFG1_OFF);
	context->smc.cscfg2 = readl(base + CSADRCFG2_OFF);
	context->smc.cscfg3 = readl(base + CSADRCFG3_OFF);
	context->smc.csmscfg = readl(base + CSMSADRCFG_OFF);
	if (cpu_is_pxa320()) {
		context->smc.mecr = readl(base + MECR_OFF);
		context->smc.mcmem0 = readl(base + MCMEM0_OFF);
		context->smc.mcatt0 = readl(base + MCATT0_OFF);
		context->smc.mcio0 = readl(base + MCIO0_OFF);
		context->smc.cscfg_p = readl(base + CSADRCFG_P_OFF);
	}

	/* system bus arbiters */
	base = context->arb.membase;
	context->arb.ctl1 = readl(base + ARBCTL1_OFF);
	context->arb.ctl2 = readl(base + ARBCTL2_OFF);

	/* pmu controller */
	context->pmu.pecr = PECR;
	context->pmu.pvcr = PVCR;
	/* clear PSR */
	tmp = PSR;
	tmp &= 0x07;
	PSR = tmp;

	pxa3xx_intc_save(&(context->intc));
	pxa3xx_clk_save(&(context->clock));
	pxa3xx_ost_save(&(context->ost));
	pxa3xx_mfp_save(&(context->mfp));
	pxa3xx_gpio_save(&(context->gpio));
}

static void pxa3xx_sysbus_restore(struct pxa3xx_pm_regs *context)
{
	unsigned char __iomem *base = NULL;

	pxa3xx_mfp_restore(&(context->mfp));
	pxa3xx_gpio_restore(&(context->gpio));
	pxa3xx_ost_restore(&(context->ost));
	pxa3xx_intc_restore(&(context->intc));
	pxa3xx_clk_restore(&(context->clock));

	/* PMU controller */
	/* status information will be lost in PECR */
	PECR = 0xA0000000;
	PECR = (context->pmu.pecr | PECR_E1IS | PECR_E0IS);
	PVCR = context->pmu.pvcr;
/*may be we can not get the right value from context->pmu.pvcr.
for PVCR,we never use hardware power I2C,just  disable FVE,PVE,TVE,FVC bit  here*/
	//PVCR = context->pmu.pvcr & 0x0fffffff;

	/* system bus arbiters */
	base = context->arb.membase;
	writel(context->arb.ctl1, base + ARBCTL1_OFF);
	writel(context->arb.ctl2, base + ARBCTL2_OFF);

	/* static memory controller */
	base = context->smc.membase;
	writel(context->smc.msc0, base + MSC0_OFF);
	writel(context->smc.msc1, base + MSC1_OFF);
	writel(context->smc.sxcnfg, base + SXCNFG_OFF);
	writel(context->smc.memclkcfg, base + MEMCLKCFG_OFF);
	writel(context->smc.cscfg0, base + CSADRCFG0_OFF);
	writel(context->smc.cscfg1, base + CSADRCFG1_OFF);
	writel(context->smc.cscfg2, base + CSADRCFG2_OFF);
	writel(context->smc.cscfg3, base + CSADRCFG3_OFF);
	writel(context->smc.csmscfg, base + CSMSADRCFG_OFF);
	if (cpu_is_pxa320()) {
		writel(context->smc.mecr, base + MECR_OFF);
		writel(context->smc.mcmem0, base + MCMEM0_OFF);
		writel(context->smc.mcatt0, base + MCATT0_OFF);
		writel(context->smc.mcio0, base + MCIO0_OFF);
		writel(context->smc.cscfg_p, base + CSADRCFG_P_OFF);
	}
}

static void pxa3xx_pm_set_clk(char *id, int enable)
{
	struct clk *clk;

	clk = clk_get(NULL, id);
	if (IS_ERR(clk)) {
		printk(KERN_INFO "clk_get failed in pxa3xx_pm_set_cken,"
				" sleep may have problems\n");
		return ;
	}
	if (enable)
		clk_enable(clk);
	else
		clk_disable(clk);
}

/* This function is used to set unit clock before system enters sleep.
 */
static void pxa3xx_pm_set_cken(void)
{
	/*
	 * turn off SMC, GPIO,INTC clocks to save power in sleep mode.
	 * they will be turn on by BLOB during wakeup
	 */
	pxa3xx_pm_set_clk("SMCCLK", 0);
	pxa3xx_pm_set_clk("GPIOCLK", 0);
	pxa3xx_pm_set_clk("INTCLK", 0);

	/*
	 * turn on clocks used by bootrom during wakeup
	 * they will be turn off by BLOB during wakeup
	 * D0CKEN_A clocks: bootrom, No.19
	 */
	pxa3xx_pm_set_clk("BOOTCLK", 1);
	pxa3xx_pm_set_clk("WTMCLK", 1);
	/* This bit must be enabled before entering low power mode. */
	if (cpu_is_pxa300() || cpu_is_pxa310() || cpu_is_pxa320())
		pxa3xx_pm_set_clk("HSIO2CLK", 1);
	
	/* Trusted parts */
	pxa3xx_pm_set_clk("SMCCLK", 1);
	pxa3xx_pm_set_clk("MINLCDCLK", 1);
}

/* This function is used to restore unit clock after system resumes.
 */
static void pxa3xx_pm_restore_cken(void)
{
	pxa3xx_pm_set_clk("SMCCLK", 1);
	pxa3xx_pm_set_clk("GPIOCLK", 1);
	pxa3xx_pm_set_clk("INTCLK", 1);
	pxa3xx_pm_set_clk("BOOTCLK", 0);
	pxa3xx_pm_set_clk("WTMCLK", 0);

	pxa3xx_pm_set_clk("SMCCLK", 0);
	pxa3xx_pm_set_clk("MINLCDCLK", 0);
}

/* This function is used to clear power manager status.
 */
static void pxa3xx_clear_pm_status(int sys_level)
{
	unsigned int tmp;

	if (sys_level) {
		/* clear power manager status */
		tmp = PSR;
		tmp &= PSR_MASK;
		PSR = tmp;
	}
	/* clear application system status */
	tmp = ASCR;
	tmp &= ASCR_MASK;
	ASCR = tmp;
	/* clear all application subsystem reset status */
	tmp = ARSR;
	ARSR = tmp;
}

/* This function is used to set RTC time.
 * When it timeouts, it will wakeup system from low power mode.
 * There's limitation that only 65 seconds sleep time can be set by this way.
 * And user should avoid to use PIAR because it will be used as wakeup timer.
 *
 * Notice:
 * User can also choice use another RTC register to trigger wakeup event.
 * If so, keep pm_sleeptime as 0. Otherwise, those RTC registers event
 * will make user confused. System will only serve the first RTC event.
 */
static void pxa3xx_set_wakeup_sec(int sleeptime)
{
	unsigned int tmp;
	if (sleeptime) {
		/* PIAR can not more than 65535 */
		if (sleeptime > 65)
			sleeptime = 65;
		pr_debug("Set RTC to wakeup system after %d sec\n",
			sleeptime);
		tmp = RTSR;
		tmp &= ~(RTSR_PICE | RTSR_PIALE);
		RTSR = tmp;
		/* set PIAR to sleep time, in ms */
		PIAR = sleeptime * 1000;

		tmp = RTSR;
		tmp |= RTSR_PICE;
		RTSR = tmp;
    }
}

/* This function is used to set OS Timer4 time.
 * The time interval may not be accurate. Because it's derived from 32.768kHz
 * oscillator.
 */
static void pxa3xx_set_wakeup_msec(int msleeptime)
{
	unsigned int tmp;
	if (msleeptime) {
		pr_debug("Set OS Timer4 to wakeup system after %d msec\n",
			msleeptime);
		/* set the time interval of sleep */
		tmp = OSCR4;
		OSSR = 0x10;
		OSMR4 = tmp + msleeptime * 32768 / 1000;
	}
}

/*
 * Clear the wakeup source event.
 */
static void pm_clear_wakeup_src(pm_wakeup_src_t src)
{
	if (wakeup_ops->ext)
		wakeup_ops->ext(src, 0);
	if (wakeup_ops->key)
		wakeup_ops->key(src, 0);
	if (wakeup_ops->mmc)
		wakeup_ops->mmc(src, 0);
	if (wakeup_ops->uart)
		wakeup_ops->uart(src, 0);
	if (wakeup_ops->eth)
		wakeup_ops->eth(src, 0);
	if (wakeup_ops->tsi)
		wakeup_ops->tsi(src, 0);
}

static void pm_select_wakeup_src(enum pxa3xx_pm_mode lp_mode,
				pm_wakeup_src_t src)
{
	unsigned int tmp, reg_src = 0;

	if (wakeup_ops->ext)
		reg_src |= wakeup_ops->ext(src, 1);
	if (wakeup_ops->key)
		reg_src |= wakeup_ops->key(src, 1);
	if (wakeup_ops->mmc)
		reg_src |= wakeup_ops->mmc(src, 1);
	if (wakeup_ops->uart)
		reg_src |= wakeup_ops->uart(src, 1);
	if (wakeup_ops->eth)
		reg_src |= wakeup_ops->eth(src, 1);
	if (wakeup_ops->tsi)
		reg_src |= wakeup_ops->tsi(src, 1);
	if (src.bits.rtc) {
		pxa3xx_set_wakeup_sec(pm_sleeptime);
		reg_src |= PXA3xx_PM_WE_RTC;
	}
	if (src.bits.ost) {
		pxa3xx_set_wakeup_msec(pm_msleeptime);
		reg_src |= PXA3xx_PM_WE_OST;
	}
	if (src.bits.msl)
		reg_src |= PXA3xx_PM_WE_MSL0;

	/* set wakeup register */
	if (lp_mode == PXA3xx_PM_SLEEP) {
		PWSR = 0xFFFFFFFF;
		PWER = 0;
		AD3SR = 0xFFFFFFFF;
		AD3ER = 0;

		tmp = PWER;
		if (src.bits.rtc)
			tmp |= PWER_WERTC;
		if (src.bits.ext0)
			tmp |= (PWER_WER0 | PWER_WEF0);
		if (src.bits.ext1)
			tmp |= (PWER_WER1 | PWER_WEF1);
		PWER = tmp;

		AD3ER = reg_src;
	}
	if (lp_mode == PXA3xx_PM_DEEPSLEEP) {
		PWSR = 0xFFFFFFFF;
		PWER = 0;

		tmp = PWER;
		if (src.bits.rtc)
			tmp |= PWER_WERTC;
		if (src.bits.ext0)
			tmp |= (PWER_WER0 | PWER_WEF0);
		if (src.bits.ext1)
			tmp |= (PWER_WER1 | PWER_WEF1);
		PWER = tmp;
	}
	if (lp_mode == PXA3xx_PM_STANDBY) {
		AD2D0SR = 0xFFFFFFFF;
		AD2D0ER = 0;
		AD2D0ER = reg_src;
	}
	if (lp_mode == PXA3xx_PM_LCDREFRESH) {
		AD1D0SR = 0xFFFFFFFF;
		AD1D0ER = 0;
		/* add the minilcd wakeup event */
		AD1D0ER = reg_src | PXA3xx_PM_WE_MLCD;
	}
	if (lp_mode == PXA3xx_PM_CG) {
		ACGD0SR = 0xFFFFFFFF;
		/* add the interrupt and dmemc wakeup event */
		ACGD0ER = reg_src | PXA3xx_PM_WE_INTC | PXA3xx_PM_WE_DMC;
	}
}

static void pm_query_wakeup_src(void)
{
	unsigned int data;

	memset(&waked, 0, sizeof(pm_wakeup_src_t));

	if (ASCR & 0x07) {
		data = ASCR & 0x80000007;		
		ASCR = data;
		switch (data & 0x7) {
		case 4:
			/* check D1 wakeup source */
			data = AD1D0SR;
			AD1D0SR = data;
			if (wakeup_ops->query)
				wakeup_ops->query(data, &waked);
			break;
		case 2:
			/* check D2 wakeup source */
			data = AD2D0SR;
			AD2D0SR = data;
			if (wakeup_ops->query)
				wakeup_ops->query(data, &waked);
			break;
		case 1:
			/* check D3 wakeup source */
			data = AD3SR;
			AD3SR = data;
			printk(KERN_DEBUG "D3 wakeup source: 0x%x\n", data);
			if (wakeup_ops->query)
				wakeup_ops->query(data, &waked);
			data = PWSR;
			PWSR = data;
			if (data & PWSR_EERTC)
				waked.bits.rtc = 1;
			if (data & PWSR_EDR0)
				waked.bits.ext0 = 1;
			if (data & PWSR_EDR1)
				waked.bits.ext1 = 1;
			break;
		}
	} else if (ARSR & 0x04) {
		/* check S3 wakeup source */
		data = ARSR & 0x04;
		ARSR = data;
		data = PWSR;
		PWSR = data;
		if (data & PWSR_EERTC)
			waked.bits.rtc = 1;
		if (data & PWSR_EDR0)
			waked.bits.ext0 = 1;
		if (data & PWSR_EDR1)
			waked.bits.ext1 = 1;
	} else {
		/* check clock gate mode wakeup source */
		if (cpu_is_pxa935()) {
			data = ACGD0SR;
			ACGD0SR = data;
			if (wakeup_ops->query)
				wakeup_ops->query(data, &waked);
		}
	}
}

static void dump_wakeup_src(pm_wakeup_src_t *src)
{
	printk(KERN_DEBUG "wakeup source: ");
	if (src->bits.rtc)
		printk(KERN_DEBUG "rtc, ");
	if (src->bits.ost)
		printk(KERN_DEBUG "ost, ");
	if (src->bits.msl)
		printk(KERN_DEBUG "msl, ");
	if (src->bits.wifi)
		printk(KERN_DEBUG "wifi, ");
	if (src->bits.uart1)
		printk(KERN_DEBUG "uart1, ");
	if (src->bits.uart2)
		printk(KERN_DEBUG "uart2, ");
	if (src->bits.uart3)
		printk(KERN_DEBUG "uart3, ");
	if (src->bits.mkey)
		printk(KERN_DEBUG "mkey, ");
	if (src->bits.dkey)
		printk(KERN_DEBUG "dkey, ");
	if (src->bits.mlcd)
		printk(KERN_DEBUG "mlcd, ");
	if (src->bits.tsi)
		printk(KERN_DEBUG "tsi, ");
	if (src->bits.ext0)
		printk(KERN_DEBUG "ext0, ");
	if (src->bits.ext1)
		printk(KERN_DEBUG "ext1, ");
	if (src->bits.mmc1_cd)
		printk(KERN_DEBUG "mmc1 card detect, ");
	if (src->bits.mmc2_cd)
		printk(KERN_DEBUG "mmc2 card detect, ");
	if (src->bits.mmc3_cd)
		printk(KERN_DEBUG "mmc3 card detect, ");
	if (src->bits.mmc1_dat1)
		printk(KERN_DEBUG "mmc1 dat1, ");
	if (src->bits.mmc2_dat1)
		printk(KERN_DEBUG "mmc2 dat1, ");
	if (src->bits.mmc3_dat1)
		printk(KERN_DEBUG "mmc3 dat1, ");
	if (src->bits.eth)
		printk(KERN_DEBUG "eth, ");
	if (src->bits.gpio)
		printk(KERN_DEBUG "gpio, ");
}

void get_wakeup_source(pm_wakeup_src_t *src)
{
	memset(src, 0, sizeof(pm_wakeup_src_t));
	memcpy(src, &waked, sizeof(pm_wakeup_src_t));
}
EXPORT_SYMBOL(get_wakeup_source);

int pxa3xx_wakeup_register(struct pxa3xx_peripheral_wakeup_ops *ops)
{
	wakeup_ops = ops;

	/* set default wakeup source */
	if (wakeup_ops->init)
		wakeup_ops->init(&wakeup_src);

	/* clear the related wakeup source */
	pm_clear_wakeup_src(wakeup_src);

	return 0;
}

void pxa3xx_wakeup_unregister(void)
{
	wakeup_ops = NULL;
}
EXPORT_SYMBOL(pxa3xx_wakeup_register);
EXPORT_SYMBOL(pxa3xx_wakeup_unregister);

/*************************************************************************/
static void flush_cpu_cache(void)
{
	__cpuc_flush_kern_all();
	//__cpuc_flush_l2cache_all();
}

struct os_header {
	int	version;
	int	identifier;
	int	address;
	int	size;
	int	reserved;
};

#ifdef CONFIG_BOARD_BRAVA
static int is_menu_key_pressed()
{
	int ret;
	pxa3xx_mfp_set_afds(MFP_PIN_GPIO116, MFP_AF0, MFP_DS04X);
	gpio_direction_input(MFP2GPIO(MFP_PIN_GPIO116));
	ret = gpio_get_value(MFP2GPIO(MFP_PIN_GPIO116));
	pxa3xx_mfp_set_afds(MFP_PIN_GPIO116, MFP_AF1, MFP_DS04X);
	return ret;
}
#endif

int pxa3xx_pm_enter_sleep(struct pxa3xx_pm_regs *pm_regs)
{
	unsigned int tmp;
	
#ifdef CONFIG_BOARD_BRAVA
	if (is_menu_key_pressed()) {
		printk(KERN_DEBUG "Warning: Menu key pressed, do a workaround\n");
		goto out;
	}
#endif

	pxa3xx_sysbus_save(pm_regs);

        if (is_wkr_2140448())
                sleep_wkr_start(pm_membase + 0x84);

	/* disable when use pwri2c as sometimes it will crash pmu. */
	yuhua_poweri2c_workaround(0);

	pm_select_wakeup_src(PXA3xx_PM_SLEEP, wakeup_src);

	pxa3xx_pm_set_cken();

	/* should set:modeSaveFlags, areaAddress, flushFunc, psprAddress,
	 * extendedChecksumByteCount */
	pm_regs->pm_data.modeSaveFlags = 0x3f;	/* PM_MODE_SAVE_FLAG_SVC; */
	pm_regs->pm_data.flushFunc = flush_cpu_cache;
	pm_regs->pm_data.areaAddress = (unsigned int)&(pm_regs->pm_data);
	pm_regs->pm_data.psprAddress = (unsigned int)&PSPR;
	pm_regs->pm_data.extendedChecksumByteCount =
		sizeof(struct pxa3xx_pm_regs) - sizeof(struct pm_save_data);
	pr_debug("ext size:%d, save size%d\n",
		pm_regs->pm_data.extendedChecksumByteCount,
		sizeof(struct pm_save_data));

	/* save the resume back address into SDRAM */
	pm_regs->word0 = __raw_readl(pm_regs->data_pool);
	pm_regs->word1 = __raw_readl(pm_regs->data_pool + 4);
	__raw_writel(virt_to_phys(pxa3xx_cpu_resume), pm_regs->data_pool);
	__raw_writel(virt_to_phys(&(pm_regs->pm_data)), pm_regs->data_pool + 4);

	pxa3xx_clear_pm_status(1);

	/* make sure that sram bank 0 is not off */
	tmp = AD3R;
	tmp |= 0x101;
	AD3R = tmp;

	pr_debug("ready to sleep:0x%lx\n", virt_to_phys(&(pm_regs->pm_data)));

	/* go to Zzzz */
	pxa3xx_cpu_sleep((unsigned int)&(pm_regs->pm_data),
			virt_to_phys(&(pm_regs->pm_data)));

	/* come back */
        if (is_wkr_2140448())
                sleep_wkr_start(pm_membase + 0x84);
	__raw_writel(pm_regs->word0, pm_regs->data_pool);
	__raw_writel(pm_regs->word1, pm_regs->data_pool + 4);

	pxa3xx_pm_restore_cken();

	pm_query_wakeup_src();
	dump_wakeup_src(&waked);

	yuhua_poweri2c_workaround(1);
	pxa3xx_sysbus_restore(pm_regs);

	pxa3xx_clear_pm_status(1);

	/* clear RDH */
	ASCR &= ~ASCR_RDH;

	pm_clear_wakeup_src(wakeup_src);

	/* Clear this bit after returns from low power mode.
	 * Clear this bit can save power.
	 */
#if 0 /* keep HSIO2CLK for u2d to work, by frank */
	if (cpu_is_pxa300() || cpu_is_pxa310() || cpu_is_pxa320())
		pxa3xx_pm_set_clk("HSIO2CLK", 0);
#endif

out:

#ifdef CONFIG_IPM
	/* Need to post event to policy maker.  */
	if (event_notify) {
		event_notify(IPM_EVENT_SUSPEND_WAKEUP, PM_SUSPEND_MEM,
				&waked, sizeof(pm_wakeup_src_t));
	}
#endif

	pr_debug("*** made it back from sleep\n");

	return 0;
}

int pxa3xx_pm_enter_deepsleep(void)
{
	unsigned int mode = PXA3xx_PM_DEEPSLEEP;

	PMCR |= 0x01;	/* Workaround of Micco reset */
	pm_select_wakeup_src(mode, wakeup_src);

	pxa3xx_pm_set_cken();

	pxa3xx_clear_pm_status(1);
	__asm__ (
		"mcr	p14, 0, %0, c7, c0, 0\n"
		:
		:"r"(mode)
	);

	return 0;
}

void pxa3xx_pm_poweroff(void)
{
	unsigned int mode = PXA3xx_PM_DEEPSLEEP;

	yuhua_poweri2c_workaround(0);

	PMCR |= 0x01;	/* Workaround of Micco reset */
	/* Disable sleeptime interface at here */
	pm_sleeptime = 0;
	pm_select_wakeup_src(mode, wakeup_src);
	pxa3xx_pm_set_cken();
	pxa3xx_clear_pm_status(1);
	__asm__ (
		"mcr	p14, 0, %0, c7, c0, 0\n"
		:
		:"r"(mode)
	);
}

static void pm_preset_standby(void)
{
	/* This bit must be enabled before entering low power mode. */
	if (cpu_is_pxa300() || cpu_is_pxa310() || cpu_is_pxa320())
		pxa3xx_pm_set_clk("HSIO2CLK", 1);

	pxa3xx_clear_pm_status(0);

	if (is_wkr_2140448())
		sleep_wkr_start(pm_membase + 0x84);
}

static void pm_postset_standby(void)
{
	if (is_wkr_2140448())
		sleep_wkr_end(pm_membase + 0x84);

	pm_query_wakeup_src();
	/*dump_wakeup_src(&waked);*/

	/* clear RDH */
	ASCR &= ~ASCR_RDH;

	pxa3xx_clear_pm_status(0);
	pm_clear_wakeup_src(wakeup_src);
	/* This bit must be disabled after entering low power mode. */
	if (cpu_is_pxa300() || cpu_is_pxa310() || cpu_is_pxa320())
		pxa3xx_pm_set_clk("HSIO2CLK", 0);
}

int pxa3xx_pm_enter_standby(struct pxa3xx_pm_regs *pm_regs)
{
	unsigned long ticks = 1;

	pm_select_wakeup_src(PXA3xx_PM_STANDBY, wakeup_src);
	pm_preset_standby();

	pxa3xx_cpu_standby((unsigned int)pm_regs->sram_map + 0x8000,
			(unsigned int)pm_regs->sram_map + 0xa000,
			(unsigned int)&ticks);

	pm_postset_standby();

#ifdef CONFIG_IPM
	/* Need to post event to policy maker.  */
	if (event_notify) {
		event_notify(IPM_EVENT_STANDBY_WAKEUP, PM_SUSPEND_STANDBY,
				&waked, sizeof(pm_wakeup_src_t));
	}
#endif
	pr_debug("*** made it back from standby\n");

	return 0;
}

#ifdef CONFIG_FB_PXA_MINILCD
extern int pxafb_minilcd_enter(void);
extern int pxafb_minilcd_exit (void);
#endif

int pxa3xx_pm_enter_lcdrefresh(struct pxa3xx_pm_regs *pm_regs)
{
	unsigned long ticks = 1;

	/* This bit must be enabled before entering low power mode. */
	if (cpu_is_pxa300() || cpu_is_pxa310() || cpu_is_pxa320())
		pxa3xx_pm_set_clk("HSIO2CLK", 1);

	/* enable miniLCD clock and WKUP_EN in miniLCD counter register.
	 * Without these operations, system can't enter D1 from D0.
	 */
	pxa3xx_pm_set_clk("MINLCDCLK", 1);
	pxa3xx_pm_set_clk("MINIMCLK", 1);

#if defined(CONFIG_FB_PXA_LCD_VGA) || defined (CONFIG_FB_PXA_LCD_QVGA)
        *(minilcd_membase + 0x4) = 0x80000000;
#endif

#if defined(CONFIG_FB_PXA_MINILCD) && defined(CONFIG_FB_PXA_LCD_QVGA)
        pxafb_minilcd_enter();
#endif

	pxa3xx_clear_pm_status(0);

	pm_select_wakeup_src(PXA3xx_PM_LCDREFRESH, wakeup_src);

	pxa3xx_cpu_lcdrefresh((unsigned int)pm_regs->sram_map + 0x8000,
			(unsigned int)pm_regs->sram_map + 0xa000,
			(unsigned int)&ticks);

        if (is_wkr_2140448())
                sleep_wkr_end(pm_membase + 0x84);
 
	pm_query_wakeup_src();
	dump_wakeup_src(&waked);
	pm_clear_wakeup_src(wakeup_src);

	pxa3xx_clear_pm_status(0);

#if defined(CONFIG_FB_PXA_MINILCD) && defined(CONFIG_FB_PXA_LCD_QVGA)
        pxafb_minilcd_exit();
#endif

	/* This bit must be disabled after exiting low power mode. */
	if (cpu_is_pxa300() || cpu_is_pxa310() || cpu_is_pxa320())
		pxa3xx_pm_set_clk("HSIO2CLK", 0);
	pxa3xx_pm_set_clk("MINLCDCLK", 0);
	pxa3xx_pm_set_clk("MINIMCLK", 0);

#ifdef CONFIG_IPM
	/* Need to post event to policy maker.  */
	if (event_notify) {
		event_notify(IPM_EVENT_STANDBY_WAKEUP, PM_SUSPEND_LCDREFRESH,
				&waked, sizeof(pm_wakeup_src_t));
	}
#endif

	pr_debug("*** made it back from lcdrefresh\n");

	return 0;
}

#ifdef CONFIG_PXA3xx_DVFM

#define CKENA_RSVD_BITS 0xc20900c1
#define CKENB_RSVD_BITS 0xdffcf46c
#define CKENC_RSVD_BITS 0xfff00021

extern int calc_switchtime(unsigned int, unsigned int);

static void pm_postset_clockgate(void)
{
	pm_query_wakeup_src();
	/*dump_wakeup_src(&waked);*/

	pxa3xx_clear_pm_status(0);
	pm_clear_wakeup_src(wakeup_src);
}

void enter_lowpower_mode(int state)
{
	unsigned int start_tick = 0, end_tick = 0;
	unsigned int cken[3], icmr[3];
	unsigned int reg;

	if (state == POWER_MODE_D1)
		pxa3xx_pm_enter_lcdrefresh(&pxa3xx_pm_regs);
	else if (state == POWER_MODE_D2) {
		pm_select_wakeup_src(PXA3xx_PM_STANDBY, wakeup_src);
		pm_preset_standby();
		end_tick = OSCR4;
		disable_oscc_pout();
		pxa3xx_cpu_standby(
			(unsigned int)pxa3xx_pm_regs.sram_map + 0x8000,
			(unsigned int)pxa3xx_pm_regs.sram_map + 0xa000,
			(unsigned int)&start_tick);
		enable_oscc_pout();
#ifdef CONFIG_PXA3xx_DVFM_STATS
		calc_switchtime(end_tick, start_tick);
#endif
		pm_postset_standby();
	} else if ((state == POWER_MODE_CG) && cpu_is_pxa935()) {
		/* This state is only available in pxa935() */

		if (cpu_is_pxa935()) {
			pm_select_wakeup_src(PXA3xx_PM_CG, wakeup_src);
			
			end_tick = OSCR4;
			/* Mask all interrupts */
			icmr[0] = ICMR;
			icmr[1] = ICMR2;
			icmr[2] = ICMR3;
			ICMR = 0;
			ICMR2 = 0;
			ICMR3 = 0;
			reg = ICMR;

			cken[0] = CKENA;
			cken[1] = CKENB;
			cken[2] = CKENC;
			
			/*
			 * Turn off all clocks except PX1 bus clock,
			 * DMEMC clock and reserved bits
			 */
			CKENA = CKENA_RSVD_BITS | (1 << 8);
			reg = CKENA;
			CKENB = CKENB_RSVD_BITS | (1 << 29);
			reg = CKENB;
			CKENC = CKENC_RSVD_BITS;
			reg = CKENC;
			
			AGENP |= AGENP_SAVE_WK;

			/* Turn off PX1 bus clock */
			CKENB &= ~(1 << 29);
			
			/* enter clock gated mode by entering core idle */
			cpu_do_idle();
			
			/* restore clocks after exiting from clock gated mode */
			CKENA = cken[0];
			reg = CKENA;
			CKENB = cken[1];
			reg = CKENB;
			CKENC = cken[2];
			reg = CKENC;
			
			/* Unmask interrupts */
			ICMR = icmr[0];
			reg = ICMR;
			ICMR2 = icmr[1];
			reg = ICMR2;
			ICMR3 = icmr[2];
			reg = ICMR3;

			start_tick = OSCR4;
#ifdef CONFIG_PXA3xx_DVFM_STATS
			calc_switchtime(end_tick, start_tick);
#endif
			pm_postset_clockgate();
		}
	}
}
#endif

static int pxa3xx_pm_enter(suspend_state_t state)
{
	if (state == PM_SUSPEND_MEM)
		return pxa3xx_pm_enter_sleep(&pxa3xx_pm_regs);
	else if (state == PM_SUSPEND_STANDBY)
		return pxa3xx_pm_enter_standby(&pxa3xx_pm_regs);
	else if (state == PM_SUSPEND_LCDREFRESH)
		return pxa3xx_pm_enter_lcdrefresh(&pxa3xx_pm_regs);
	else if (state == PM_SUSPEND_DEEPSLEEP)
		return pxa3xx_pm_enter_deepsleep();
	else
		return -EINVAL;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int pxa3xx_pm_prepare(void)
{
	struct os_header header;
	int obm_ver;

#ifdef CONFIG_IPM
	/* Disable deep idle when system enters low power mode */
	save_deepidle = enable_deepidle;
	enable_deepidle = 0;
#endif
	if (pm_state == PM_SUSPEND_MEM) {
		/* backup data in ISRAM */
		memcpy(pxa3xx_pm_regs.sram, pxa3xx_pm_regs.sram_map, isram_size);
		obm_ver = calc_obm_ver();
		if (obm_ver == PXA3xx_OBM_V2) {
			/* load OBM into ISRAM
			 * The target address is 0x5c014000
			 */
			memcpy(pxa3xx_pm_regs.sram_map + 0x14000, pxa3xx_pm_regs.obm,
				MOBM_SIZE);
		} else if (obm_ver == PXA3xx_OBM_V3) {
			/* load OBM into ISRAM
			 * The target address is 0x5c013000
			 * The main purpose to load obm is to initialize DDR.
			 * When OBM found it's a resume process, it will jump
			 * to resume routine what resides in DDR.
			 */
			memset(&header, 0, sizeof(struct os_header));
			header.version = 3;
			header.identifier = 0x5265736D;		/* RESM */
			header.address = 0x5c013000;
			header.size = MOBM_SIZE;
			/* 0x5c008000 */
			memcpy(pxa3xx_pm_regs.sram_map + 0x8000, &header,
				sizeof(struct os_header));
			/* 0x5c013000 */
			memcpy(pxa3xx_pm_regs.sram_map + 0x13000, pxa3xx_pm_regs.obm,
				MOBM_SIZE);
		}
	} else if (pm_state == PM_SUSPEND_STANDBY) {
		/* FIXME: allocat SRAM to execute D1/D2 entry/exit code.
		 * Try not to use it in the future.
		 */
		/* backup data in ISRAM */
		memcpy(pxa3xx_pm_regs.sram, pxa3xx_pm_regs.sram_map, 1024);

	} else if (pm_state == PM_SUSPEND_LCDREFRESH) {
		/* FIXME: allocat SRAM to execute D1/D2 entry/exit code.
		 * Try not to use it in the future.
		 */
		/* backup data in ISRAM */
		memcpy(pxa3xx_pm_regs.sram, pxa3xx_pm_regs.sram_map, 1024);
	}

	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void pxa3xx_pm_finish(void)
{
#ifdef CONFIG_IPM
	enable_deepidle = save_deepidle;
#endif
	if (pm_state == PM_SUSPEND_MEM) {
		/* restore data in ISRAM */
		memcpy(pxa3xx_pm_regs.sram_map, pxa3xx_pm_regs.sram, isram_size);
	} else if (pm_state == PM_SUSPEND_STANDBY) {
		/* restore data in ISRAM */
		memcpy(pxa3xx_pm_regs.sram_map, pxa3xx_pm_regs.sram, 1024);
	} else if (pm_state == PM_SUSPEND_LCDREFRESH) {
		/* restore data in ISRAM */
		memcpy(pxa3xx_pm_regs.sram_map, pxa3xx_pm_regs.sram, 1024);
	}
	pm_state = PM_SUSPEND_ON;
}

static int pxa3xx_pm_valid(suspend_state_t state)
{
	int ret = 1;

	if (state == PM_SUSPEND_MEM) {
		pm_state = PM_SUSPEND_MEM;
	} else if (state == PM_SUSPEND_STANDBY) {
		pm_state = PM_SUSPEND_STANDBY;
	} else if (state == PM_SUSPEND_LCDREFRESH) {
		pm_state = PM_SUSPEND_LCDREFRESH;
	} else if (state == PM_SUSPEND_DEEPSLEEP) {
		pm_state = PM_SUSPEND_DEEPSLEEP;
	} else {
		ret = 0;
	}
	return ret;
}

/*
 * Set to PM_DISK_FIRMWARE so we can quickly veto suspend-to-disk.
 */
static struct platform_suspend_ops pxa3xx_pm_ops = {
	.valid		= pxa3xx_pm_valid,
	.prepare	= pxa3xx_pm_prepare,
	.enter		= pxa3xx_pm_enter,
	.finish		= pxa3xx_pm_finish,
};

#define pm_attr(_name, object)						\
static ssize_t _name##_store(struct kobject *kobj,			\
		struct kobj_attribute *attr,				\
		const char * buf, size_t n)				\
{									\
	sscanf(buf, "%u", &object);					\
	return n;							\
}									\
static ssize_t _name##_show(struct kobject *kobj,			\
		struct kobj_attribute *attr,				\
		char * buf)						\
{									\
	return sprintf(buf, "%u\n", object);				\
}									\
static struct kobj_attribute _name##_attr = {				\
	.attr	= {							\
		.name = __stringify(_name),				\
		.mode = 0644,						\
	},								\
	.show	= _name##_show,						\
	.store	= _name##_store,					\
}

pm_attr(sleeptime, pm_sleeptime);
pm_attr(msleeptime, pm_msleeptime);
#ifdef CONFIG_IPM

static int tokenizer(char **tbuf, const char *userbuf, ssize_t n,
			char **tokptrs, int maxtoks)
{
	char *cp, *tok;
	char *whitespace = " \t\r\n";
	int ntoks = 0;

	if (!(cp = kmalloc(n + 1, GFP_KERNEL)))
		return -ENOMEM;

	*tbuf = cp;
	memcpy(cp, userbuf, n);
	cp[n] = '\0';

	do {
		cp = cp + strspn(cp, whitespace);
		tok = strsep(&cp, whitespace);
		if ((*tok == '\0') || (ntoks == maxtoks))
			break;
		tokptrs[ntoks++] = tok;
	} while(cp);

	return ntoks;
}

static ssize_t deepidle_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int len = 0;

	if (enable_deepidle & IDLE_D0CS)
		len += sprintf(buf + len, "D0CSIDLE, ");
	if (enable_deepidle & IDLE_D1)
		len += sprintf(buf + len, "D1IDLE, ");
	if (enable_deepidle & IDLE_D2)
		len += sprintf(buf + len, "D2IDLE, ");
	if (enable_deepidle & IDLE_CG)
		len += sprintf(buf + len, "CGIDLE, ");
	len += sprintf(buf + len, "D0IDLE\n");
	len += sprintf(buf + len, "Command: echo [set|unset] [d0cs|d1|d2|cg] "
			"> deepidle\n");
	return len;
}

#define MAXTOKENS	80

static ssize_t deepidle_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int error = 0;
	char *tbuf = NULL;
	char *token[MAXTOKENS];
	int ntoks = tokenizer(&tbuf, buf, len, (char **)&token, MAXTOKENS);

	if (ntoks <= 0) {
		error = ntoks;
		goto out;
	}

	if (strcmp(token[0], "set") == 0) {
		if (strcmp(token[1], "d0cs") == 0)
			enable_deepidle |= IDLE_D0CS;
		else if (strcmp(token[1], "d1") == 0)
			enable_deepidle |= IDLE_D1;
		else if (strcmp(token[1], "d2") == 0)
			enable_deepidle |= IDLE_D2;
		else if (strcmp(token[1], "cg") == 0) {
			if (cpu_is_pxa935())
				enable_deepidle |= IDLE_CG;
			else
				printk(KERN_INFO "Clock Gated mode is not supported!\n");
		} else
			error = -EINVAL;
	} else if (strcmp(token[0], "unset") == 0) {
		if (strcmp(token[1], "d0cs") == 0)
			enable_deepidle &= ~IDLE_D0CS;
		else if (strcmp(token[1], "d1") == 0)
			enable_deepidle &= ~IDLE_D1;
		else if (strcmp(token[1], "d2") == 0)
			enable_deepidle &= ~IDLE_D2;
		else if (strcmp(token[1], "cg") == 0) {
			if (cpu_is_pxa935())
				enable_deepidle &= ~IDLE_CG;
			else
				printk(KERN_INFO "Clock Gated mode is not supported!\n");
		} else
			error = -EINVAL;
	} else {
		if (strcmp(token[0], "1") == 0)
			enable_deepidle = IDLE_D0CS;
		else if (strcmp(token[0], "0") == 0)
			enable_deepidle = IDLE_D0;
		else
			error = -EINVAL;
	}
out:
	if (tbuf)
		kfree(tbuf);
	return error ? error : len;
}

static struct kobj_attribute  deepidle_attr = {
	.attr	= {
		.name = __stringify(deepidle),
		.mode = 0644,
	},
	.show	= deepidle_show,
	.store	= deepidle_store,
};
#endif

static ssize_t cp_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%u\n", pm_cp);
}

static ssize_t cp_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	sscanf(buf, "%u", &pm_cp);
	if (pm_cp) {
		/* release CP */
		if (cpu_is_pxa930() || cpu_is_pxa935())
			__raw_writel(0x11, pm_membase + CSER_OFF);
	} else {
		/* reset CP */
		if (cpu_is_pxa930() || cpu_is_pxa935())
			__raw_writel(0x0, pm_membase + CSER_OFF);
	}
	return len;
}

static struct kobj_attribute cp_attr = {
	.attr	= {
		.name = __stringify(cp),
		.mode = 0644,
	},
	.show	= cp_show,
	.store	= cp_store,
};

static ssize_t temp_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int len = 0;
	if (cpu_is_pxa930() || cpu_is_pxa935()) {
		len += sprintf(buf + len, "OVH %x ", OVH);
		len += sprintf(buf + len, "PSR %x ", PSR);
		len += sprintf(buf + len, "temp_of_core %x\n", temp_of_core);
	}
	return len;
}

static struct kobj_attribute temp_attr = {
	.attr	= {
		.name = __stringify(temp),
		.mode = 0644,
	},
	.show	= temp_show,
};

static ssize_t reg_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int len = 0;

	len += sprintf(buf + len, "ARSR:0x%08x\n", ARSR);
	len += sprintf(buf + len, "PSR:0x%08x\n", PSR);
	len += sprintf(buf + len, "AD1D0SR:0x%08x\n", AD1D0SR);
	len += sprintf(buf + len, "AD2D0SR:0x%08x\n", AD2D0SR);
	len += sprintf(buf + len, "AD3SR:0x%08x\n", AD3SR);
	len += sprintf(buf + len, "PWSR:0x%08x\n", PWSR);
	len += sprintf(buf + len, "OVH:0x%08x\n", OVH);
	len += sprintf(buf + len, "PMCR:0x%08x\n", PMCR);

	return len;
}

static struct kobj_attribute reg_attr = {
	.attr	= {
		.name = __stringify(reg),
		.mode = 0644,
	},
	.show	= reg_show,
};

/* Entry to get boot tag */
static ssize_t pm_boottag_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "bootsrc %s, hardware version:%d\n", get_bootsrc_string(), getBoardHwVersion());
}

static struct kobj_attribute pm_boottag_attr = {
        .attr   = {
                .name = __stringify(bootag),
                .mode = 0644,
        },   
        .show   = pm_boottag_show, 
        .store  = NULL,
};

static struct attribute * g[] = {
	&sleeptime_attr.attr,
	&msleeptime_attr.attr,
#ifdef CONFIG_IPM
	&deepidle_attr.attr,
#endif
	&cp_attr.attr,
	&temp_attr.attr,
	&reg_attr.attr,
	&pm_boottag_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

static void log_overheating_event(unsigned char *str_log)
{
	struct timeval now;

	do_gettimeofday(&now);
	pr_warning("[%10lu]%s PSR.TSS 0x%x\n",
		   now.tv_sec, str_log, temp_of_core);
	/* TODO: record the warning into NVM and nodify the user */
}

static void detect_core_temp(unsigned long data)
{
	struct timer_list *timer = &temp_detecting_timer;

	/* detect the temp of core */
	temp_of_core = (PSR >> PSR_TSS_OFF) & 0x7;

	if (temp_of_core < TSS_THRESHOLD && !(PMCR & PMCR_TIE)) {
		log_overheating_event("INFO: AP core cooled down!");
		PMCR |= PMCR_TIE;
	} else {
		/* reset the timer */
		mod_timer(timer, jiffies + FRQ_TEMP);
	}
	
	pr_debug("%s:PSR 0x%x PMCR 0x%x OVH 0x%x\n", __func__, PSR, PMCR, OVH);
}

static irqreturn_t core_overhearting_irq(int irq, void *data)
{
	struct timer_list *timer = &temp_detecting_timer;

	pr_debug("%s:PSR 0x%x PMCR 0x%x OVH 0x%x\n", __func__, PSR, PMCR, OVH);
	if (PMCR & PMCR_TIS) {
		/* disable the interrupt & clear the status bit*/
		PMCR &= ~PMCR_TIE;
		PMCR |= PMCR_TIS;

		log_overheating_event("WARNING: AP core is OVERHEATING!");

		/* start the timer for measuring the temp of core */
		mod_timer(timer, jiffies + FRQ_TEMP);
	}
	return IRQ_HANDLED;
}

static void overheating_init(void)
{
	struct timer_list *timer = &temp_detecting_timer;
	int retval;

	/* irq setup after old hardware state is cleaned up */
	retval = request_irq(IRQ_SGP, core_overhearting_irq,
			     IRQF_DISABLED, "Overheating", NULL);
	if (retval != 0) {
		printk(KERN_ERR "%s: can't get irq %i, err %d\n",
		       "Overheating", IRQ_SGP, retval);
		return;
	}

	OVH = (OVH_OTIS_DEFAULT) << OVH_OTIF_OFF;
	OVH |= (OVH_OVWF_DEFAULT) << OVH_OVWF_OFF;
	PMCR |= (PMCR_TIE | PMCR_TIS);
	OVH |= OVH_TEMP_EN;
	OVH |= OVH_OWM;

	/* initialize the timer for measuring the temp of core */
	init_timer(timer);
	timer->function = detect_core_temp;
}

static int __init pxa3xx_pm_init(void)
{
	unsigned int oscc;
	suspend_set_ops(&pxa3xx_pm_ops);

#ifdef CONFIG_IPM
#ifdef CONFIG_IPM_DEEPIDLE
	enable_deepidle |= IDLE_D0CS;
#endif
#ifdef CONFIG_IPM_D2IDLE
	enable_deepidle |= IDLE_D2;
#endif
#ifdef CONFIG_IPM_CGIDLE
	if (cpu_is_pxa935())
		enable_deepidle |= IDLE_CG;
#endif
	oscc = OSCC;
	
	/* Disable CLK_TOUT in S0/S2/S3 state */
	//oscc &= ~0x600;
	oscc |= 0x600;
	/* if nowhere use tout_s0, it would be disable */
	enable_oscc_tout_s0();
	disable_oscc_tout_s0();
	/* Disable TD bit */
	oscc &= ~0x10000;
	/* set VCTSTB as 0x11 ~ 0.5ms */
	OSCC = (oscc & ~(0xFF << 24)) | (0x11 << 24);
	/* Enable CLK_POUT */
	enable_oscc_pout();
#endif
        orig_poweroff = pm_power_off;
        pm_power_off = pxa3xx_pm_poweroff;

	if (cpu_is_pxa930() || cpu_is_pxa935())
		overheating_init();

	if (sysfs_create_group(power_kobj, &attr_group))
		return -1;

	pxa3xx_sysbus_init(&pxa3xx_pm_regs);

	/* make sure that sram bank 0 is not off in D1 */
	if (cpu_is_pxa320())
		AD1R = 0x13F;
	else if (cpu_is_pxa930() || cpu_is_pxa935())
		AD1R = 0x101;
	else
		AD1R = 0x109;

	/* make sure that sram bank 0 is not off in D2 */
	if (cpu_is_pxa320())
		AD2R = 0x13F;
	else if (cpu_is_pxa930() || cpu_is_pxa935())
		AD2R = 0x101;
	else
		AD2R = 0x109;

	pxa3xx_init_standby((unsigned int)pxa3xx_pm_regs.sram_map + 0x8000);

	minilcd_membase  = ioremap(0x46000000, 0x20);
	

	printk("Detect cpu id 0x%x\n", read_cpuid(0));
	return 0;
}

late_initcall(pxa3xx_pm_init);

