/*  pxa_mved - main file for mved driver
 *
 *  Copyright (C) 2006, Intel Corporation.
 *  Copyright (C) 2007, Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */ 
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/pagemap.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/semaphore.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/memory.h>
#include <mach/hardware.h>
#include <asm/dma.h>
#include <linux/dma-mapping.h>
#include <asm/irq.h>
#include <mach/irqs.h>
#include <mach/pxa-regs.h>
#include <mach/pxa3xx-regs.h>
#include <linux/mm.h>
#include <mach/ipmc.h>
#include <linux/timer.h>
#include <mach/pxa_mved.h>
#ifdef CONFIG_IMM
#include <mach/imm.h>
#endif
#if defined(CONFIG_PXA3xx_DVFM)
#include <linux/notifier.h>
#include <linux/timer.h>
#include <mach/dvfm.h>
#include <mach/pxa3xx_dvfm.h>
#include <mach/pxa3xx_pm.h>
#endif 

#define MVED_REG(x)     	(*((volatile unsigned int *)(pxa3xx_mved->mvedbase+(x))))
#define MVED_DMA_REG(x)     	(*((volatile unsigned int *)(pxa3xx_mved->mveddmabase+(x))))
#define MVED_BPB2IMG_REG(x)	(*((volatile unsigned int *)(pxa3xx_mved->bpb2imgbase+(x))))

#define MVED_INTSTAT		MVED_REG(0x4)
#define MVED_INTENAB		MVED_REG(0x8)
#define MVED_INTCLEAR		MVED_REG(0xc)
#define MVED_CLKGATE		MVED_REG(0x10)

#define MVED_MVDA2INTSTAT	MVED_REG(0x2000+0x80)
#define MVED_MVDA2INTENAB	MVED_REG(0x2000+0x84)
#define MVED_MVDA2INTCLEAR	MVED_REG(0x2000+0x88)

#define MVED_DMA_DCSR0		MVED_DMA_REG(0x0)
#define MVED_DMA_DCSR1		MVED_DMA_REG(0x4)
#define MVED_DMA_DCSR2		MVED_DMA_REG(0x8)
#define MVED_DMA_DALGN		MVED_DMA_REG(0xa0)
#define MVED_DMA_DPCSR		MVED_DMA_REG(0xa4)
#define MVED_DMA_DRQSR0		MVED_DMA_REG(0xe0)
#define MVED_DMA_DRQSR1		MVED_DMA_REG(0xe4)
#define MVED_DMA_DRQSR2		MVED_DMA_REG(0xe8)
#define MVED_DMA_DINT		MVED_DMA_REG(0xf0)
#define MVED_DMA_DRCMR0		MVED_DMA_REG(0x100)
#define MVED_DMA_DRCMR1		MVED_DMA_REG(0x104)
#define MVED_DMA_DRCMR2		MVED_DMA_REG(0x108)
#define MVED_DMA_DRCMR3		MVED_DMA_REG(0x10c)
#define MVED_DMA_DDADR0		MVED_DMA_REG(0x200)
#define MVED_DMA_DSADR0		MVED_DMA_REG(0x204)
#define MVED_DMA_DTADR0		MVED_DMA_REG(0x208)
#define MVED_DMA_DCMD0		MVED_DMA_REG(0x20c)
#define MVED_DMA_DDADR1		MVED_DMA_REG(0x210)
#define MVED_DMA_DSADR1		MVED_DMA_REG(0x214)
#define MVED_DMA_DTADR1		MVED_DMA_REG(0x218)
#define MVED_DMA_DCMD1		MVED_DMA_REG(0x21c)
#define MVED_DMA_DDADR2		MVED_DMA_REG(0x220)
#define MVED_DMA_DSADR2		MVED_DMA_REG(0x224)
#define MVED_DMA_DTADR2		MVED_DMA_REG(0x228)
#define MVED_DMA_DCMD2		MVED_DMA_REG(0x22c)

/**
 * MVED_DMA_DCSR Register 
 */
#define MVED_DMA_DCSR_BUS_ERR_INTR       (1U<<0)
#define MVED_DMA_DCSR_START_INTR         (1U<<1)
#define MVED_DMA_DCSR_END_INTR           (1U<<2)
#define MVED_DMA_DCSR_STOP_INTR          (1U<<3)
#define MVED_DMA_DCSR_RAS_INTR           (1U<<4)
#define MVED_DMA_DCSR_REQ_PEND           (1U<<8)
#define MVED_DMA_DCSR_EOR_INTR           (1U<<9)
#define MVED_DMA_DCSR_MASK_RUN           (1U<<22)
#define MVED_DMA_DCSR_RAS_EN             (1U<<23)
#define MVED_DMA_DCSR_EOR_STOP_EN        (1U<<26)
#define MVED_DMA_DCSR_EOR_JMP_EN         (1U<<27)
#define MVED_DMA_DCSR_EOR_IRQ_EN         (1U<<28)
#define MVED_DMA_DCSR_STOP_IRQ_EN        (1U<<29)
#define MVED_DMA_DCSR_NO_DESC_FETCH      (1U<<30)
#define MVED_DMA_DCSR_RUN                (1U<<31)

/**
 * Mask of all writable bits in DCSR; others must be written as 0
 */
#define MVED_DMA_DCSR_WRITABLES_MSK    (MVED_DMA_DCSR_BUS_ERR_INTR |\
                                    MVED_DMA_DCSR_START_INTR   |\
                                    MVED_DMA_DCSR_END_INTR     |\
                                    MVED_DMA_DCSR_RAS_INTR     |\
                                    MVED_DMA_DCSR_EOR_INTR     |\
                                    MVED_DMA_DCSR_RAS_EN       |\
                                    MVED_DMA_DCSR_EOR_STOP_EN  |\
                                    MVED_DMA_DCSR_EOR_JMP_EN   |\
                                    MVED_DMA_DCSR_EOR_IRQ_EN   |\
                                    MVED_DMA_DCSR_STOP_IRQ_EN  |\
                                    MVED_DMA_DCSR_NO_DESC_FETCH|\
                                    MVED_DMA_DCSR_RUN         )
                                    
#define MVED_DMA_DCSR_WRITE_ONE_TO_CLEAR_BITS    (MVED_DMA_DCSR_BUS_ERR_INTR|\
                                                   MVED_DMA_DCSR_START_INTR  |\
                                                   MVED_DMA_DCSR_END_INTR    |\
                                                   MVED_DMA_DCSR_RAS_INTR    |\
                                                   MVED_DMA_DCSR_EOR_INTR)

#define BIMFSR			MVED_BPB2IMG_REG(0x0) 
#define BIMESR			MVED_BPB2IMG_REG(0x8) 
#define BIMDRMR			MVED_BPB2IMG_REG(0x10) 
#define BIMDBSR			MVED_BPB2IMG_REG(0x18) 
#define BIMISR			MVED_BPB2IMG_REG(0x20) 
#define BIMCRR			MVED_BPB2IMG_REG(0x28) 
#define BIMIER			MVED_BPB2IMG_REG(0x30) 
#define BIMSQE 			MVED_BPB2IMG_REG(0x38) 
#define BIMPMR1			MVED_BPB2IMG_REG(0x40) 
#define BIMPMR2			MVED_BPB2IMG_REG(0x48) 
#define BIMRES			MVED_BPB2IMG_REG(0x50) 

struct pxa3xx_mved_t {
	spinlock_t lock;
	struct rw_semaphore sem;
	unsigned long mvedphybase;
	unsigned char __iomem *mvedbase;
	unsigned long mveddmaphybase;
	unsigned char __iomem *mveddmabase;
	unsigned long bpb2imgphybase;
	unsigned char __iomem *bpb2imgbase;
	unsigned long mvedirq;
	unsigned long mveddmairq;
	struct clk *clk;
#ifdef CONFIG_MVED_RES_MEM
	unsigned long surfacemem;
	unsigned long surfacememphy;
	unsigned long surfacememlen;
	unsigned long devicemem;
	unsigned long devicememphy;
	unsigned long devicememlen;
#endif	
#ifdef CONFIG_IMM
	unsigned int immid;
#endif
	int usercount;

	int powersave;
	enum {MVED_POWERON, MVED_POWEROFF} powerstate;
	unsigned int mvedclkrate;
	
	unsigned int poweroffcount;
	unsigned int poweroncount;
	unsigned int dvfmcount;

	unsigned int decode_mb_enable;
	unsigned int decode_mb_count;
};

static struct pxa3xx_mved_t pxa3xx_mved_data;
static struct pxa3xx_mved_t *pxa3xx_mved = &pxa3xx_mved_data;
static int mved_fix_clkrate;

static DECLARE_WAIT_QUEUE_HEAD(mved_dma_wait);
static DECLARE_WAIT_QUEUE_HEAD(mved_mtx_wait);
static DECLARE_WAIT_QUEUE_HEAD(mved_mvea_wait);
static DECLARE_WAIT_QUEUE_HEAD(mved_displaydma_wait);
static DECLARE_WAIT_QUEUE_HEAD(mved_decode_mb_wait);

#define pxa_inv_range           __glue(_CACHE,_dma_inv_range)
#define pxa_clean_range         __glue(_CACHE,_dma_clean_range)
#define pxa_flush_range         __glue(_CACHE,_dma_flush_range)
extern void pxa_inv_range(const void *, const void *);
extern void pxa_clean_range(const void *, const void *);
extern void pxa_flush_range(const void *, const void *);

static irqreturn_t mved_dma_irq(int irq, void *ptr);
static irqreturn_t mved_irq(int irq, void *ptr);
static void display_dma_irq(int channel, void *data);

static ssize_t pxa3xx_mved_proc_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	char			*buf = page;
	char			*next = buf;
	unsigned		size = count;
	int 			t;
	int 			mved_clock;

	t = scnprintf(next, size, "MVED Info: \n");
	size -= t; next += t;
	t = scnprintf(next, size, "Power Save Mode:\t\t%s\n", 
			pxa3xx_mved->powersave?"On":"Off");
	size -= t; next += t;

	if (((ACCR & (0x3<<28)) >> 28) == 0)
		mved_clock = 104;
	else if (((ACCR & (0x3<<28)) >> 28) == 1)
		mved_clock = 156;
	else if (((ACCR & (0x3<<28)) >> 28) == 2)
		mved_clock = 208;
	else
		mved_clock = 78;
	t = scnprintf(next, size, "MVED Clock:\t\t\t\t%dMHz\n", mved_clock);
	size -= t; next += t;

	t = scnprintf(next, size, "PowerOn Count:\t\t\t%d\n", 
			pxa3xx_mved->poweroncount);
	size -= t; next += t;

	t = scnprintf(next, size, "PowerOff Count:\t\t\t%d\n", 
			pxa3xx_mved->poweroffcount);
	size -= t; next += t;

	t = scnprintf(next, size, "DVFM Count:\t\t\t%d\n", 
			pxa3xx_mved->dvfmcount);
	size -= t; next += t;

	t = scnprintf(next, size, "Help:\n");
	size -= t; next += t;

	t = scnprintf(next, size, 
		"echo 0 > /proc/driver/mved\t/*Disable Power Save Mode*/\n");
	size -= t; next += t;

	t = scnprintf(next, size, 
		"echo 1 > /proc/driver/mved\t/*Enable Power Save Mode*/\n");
	size -= t; next += t;
	
	*eof = 1;

	return count - size;
}

static int pxa3xx_mved_proc_write(struct file *file, const char __user *buffer,
			   unsigned long count, void *data)
{
	static char kbuf[1024];
	int mved_clock = 0;

	if (count >= 1024)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;

	sscanf(kbuf, "%d", &mved_clock);

	if (104 == mved_clock) {
		ACCR = (ACCR & (~(0x3<<28))) | (0x0<<28);
		mved_fix_clkrate = 104;
	} else if (156 == mved_clock) {
		ACCR = (ACCR & (~(0x3<<28))) | (0x1<<28);
		mved_fix_clkrate = 156;
	} else if (78 == mved_clock) {
		ACCR = (ACCR & (~(0x3<<28))) | (0x3<<28);
		mved_fix_clkrate = 78;
	} else if (1 == mved_clock) {
		pxa3xx_mved->powersave = 1;
	} else if (0 == mved_clock) {
		pxa3xx_mved->powersave = 0;
	} else{
		printk(KERN_ERR "error!!\n");
		printk(KERN_ERR "\techo 0 > /proc/driver/mved\t\t/*Disable powersave mode*/\n");
		printk(KERN_ERR "\techo 1 > /proc/driver/mved\t\t/*Enable powersave mode*/\n");
		printk(KERN_ERR "\techo 78 > /proc/driver/mved\t\t/*Set 78MHz*/\n");
		printk(KERN_ERR "\techo 104 > /proc/driver/mved\t\t/*Set 104MHz*/\n");
		printk(KERN_ERR "\techo 156 > /proc/driver/mved\t\t/*Set 156MHz*/\n");
	}

	return count;
}

#if defined(CONFIG_PXA3xx_DVFM)
static struct dvfm_lock dvfm_lock = {
	.lock		= SPIN_LOCK_UNLOCKED,
	.dev_idx	= -1,
	.count		= 0,
};

static void set_dvfm_constraint(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	if (dvfm_lock.count++ == 0) {
		/* Disable Low power mode */
		dvfm_disable_op_name("D0CS", dvfm_lock.dev_idx);
		dvfm_disable_op_name("D1", dvfm_lock.dev_idx);
		dvfm_disable_op_name("D2", dvfm_lock.dev_idx);
		if (cpu_is_pxa935())
			dvfm_disable_op_name("CG", dvfm_lock.dev_idx);
	} else
		dvfm_lock.count--;
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}

static void unset_dvfm_constraint(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	if (dvfm_lock.count == 0) {
		spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
		return;
	}
	if (--dvfm_lock.count == 0) {
		/* Enable Low power mode */
		dvfm_enable_op_name("D0CS", dvfm_lock.dev_idx);
		dvfm_enable_op_name("D1", dvfm_lock.dev_idx);
		dvfm_enable_op_name("D2", dvfm_lock.dev_idx);
		if (cpu_is_pxa935())
			dvfm_enable_op_name("CG", dvfm_lock.dev_idx);
	} else
		dvfm_lock.count++;
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}

static int mved_notifier_freq(struct notifier_block *nb,
		unsigned long val, void *data);
static struct notifier_block notifier_freq_block = {
	.notifier_call = mved_notifier_freq,
};
static int mved_notifier_freq(struct notifier_block *nb,
		unsigned long val, void *data)
{

	if (!pxa3xx_mved->usercount)
		return NOTIFY_DONE;

	if (val == DVFM_FREQ_PRECHANGE) {
		return NOTIFY_OK;
	}else if (val == DVFM_FREQ_POSTCHANGE){
		pxa3xx_mved->dvfmcount++;
		return NOTIFY_OK;
	}else {
		printk("Unknow DVFM Notifier!!\n");
		return NOTIFY_BAD;
	}
}
#else
static void set_dvfm_constraint(void)
{return;}
static void unset_dvfm_constraint(void)
{return;}
#endif

static unsigned long get_cur_freq(void)
{
#if defined(CONFIG_PXA3xx_DVFM)
	int op;
	struct dvfm_md_opt *opt;
	struct op_info *info = NULL;

	op = dvfm_get_op(&info);
	if (info==NULL)
		panic("Get OP Info Failed!!\n");
	opt = (struct dvfm_md_opt *)info->op;
	if (opt == NULL)
		panic("Get OPT Failed!!\n");
	
	if(opt->power_mode != POWER_MODE_D0)
		panic("Core is not in D0 mode!!!\n");
	return (opt->xl * opt->xn * 13);
#else
	return 624;
#endif
}

static int update_mved_freq(void)
{
	unsigned long 		cur_core_freq;

	cur_core_freq = get_cur_freq();
	if(mved_fix_clkrate == 78) {
		pxa3xx_mved->mvedclkrate = 78;
		ACCR = (ACCR&(~(0x3<<28)))|(0x3 <<28); /*mved clock = 78MHz*/
	}else if (mved_fix_clkrate == 104) {
		pxa3xx_mved->mvedclkrate = 104;
		ACCR = (ACCR&(~(0x3<<28)))|(0x0 <<28); /*mved clock = 104MHz*/
	}else if (mved_fix_clkrate == 156) {
		pxa3xx_mved->mvedclkrate = 156;
		ACCR = (ACCR&(~(0x3<<28)))|(0x1 <<28); /*mved clock = 156MHz*/
	}else {
		if (cur_core_freq == 104 || cur_core_freq == 208) {
			pxa3xx_mved->mvedclkrate = 78;
			ACCR = (ACCR&(~(0x3<<28)))|(0x3 <<28); /*mved clock = 78MHz*/
		}else if (cur_core_freq == 416) {
			pxa3xx_mved->mvedclkrate = 104;
			ACCR = (ACCR&(~(0x3<<28)))|(0x0 <<28); /*mved clock = 104MHz*/
		}else if (cur_core_freq == 624) {
			pxa3xx_mved->mvedclkrate = 156;
			ACCR = (ACCR&(~(0x3<<28)))|(0x1 <<28); /*mved clock = 156MHz*/
		}else {
			pxa3xx_mved->mvedclkrate = 156;
			ACCR = (ACCR&(~(0x3<<28)))|(0x1 <<28); /*mved clock = 156MHz*/
			printk(KERN_WARNING "Core freq %dMHz is not suitable for MVED\n",
										cur_core_freq);
			//panic("Core freq not supported!!!\n");
		}
	}

	return 0;
}

static void _mved_power(int on)
{
	if(on && pxa3xx_mved->powerstate==MVED_POWEROFF) {
		set_dvfm_constraint();
		update_mved_freq();
		clk_enable(pxa3xx_mved->clk);
		pxa3xx_mved->powerstate=MVED_POWERON;
		pxa3xx_mved->poweroncount++;
	}else if(!on && pxa3xx_mved->powerstate==MVED_POWERON) {
		clk_disable(pxa3xx_mved->clk);
		unset_dvfm_constraint();
		pxa3xx_mved->powerstate=MVED_POWEROFF;
		pxa3xx_mved->poweroffcount++;
	}else {
	}
}

static void mved_power(int on)
{
	down_write(&pxa3xx_mved->sem);
	_mved_power(on);
	up_write(&pxa3xx_mved->sem);
}

static unsigned long uva_to_pa(unsigned long addr)
{
	pgd_t *pgd = pgd_offset(current->mm, addr);
	pud_t *pud;
	pmd_t *pmd;
	pte_t *ptep;

	if (!pgd_none(*pgd)) {
		pud = pud_offset(pgd, addr);
		if (!pud_none(*pud)) {
			pmd = pmd_offset(pud, addr);
			if (!pmd_none(*pmd)) {
				ptep = pte_offset_kernel(pmd, addr);
				if (pte_present(*ptep)) {
					ptep = (pte_t*)((u32)ptep-2048);
					if(ptep)
						return (*ptep & 0xfffff000) 
							| (addr&0xfff);
				}
			}
		}
	}
	return 0;
}

static int check_phy_contiguous(unsigned long addr, unsigned long len)
{
	unsigned long phyaddr;
	unsigned long lastphyaddr;
	
	if ((phyaddr = uva_to_pa(addr)) == 0UL)
		return 0;
	lastphyaddr = phyaddr;
	len = (len>PAGE_SIZE) ? len-PAGE_SIZE : 0;
	addr += PAGE_SIZE;
	while (len) {
		if ((phyaddr = uva_to_pa(addr)) == 0UL)
			return 0;
		if (lastphyaddr+PAGE_SIZE != phyaddr)
			return 0;
		lastphyaddr = phyaddr;
		addr += PAGE_SIZE;
		len = (len>PAGE_SIZE) ? len-PAGE_SIZE : 0;
	}
	return 1;
}

static int 			display_dma_ch = -1;
static volatile int 		display_dma_end = 0;
static pxa_dma_desc 		*display_dma_desc;
static unsigned long 		display_dma_desc_p;
static int 			is_display_dma_pending;
static int display_do_dma(unsigned long srcphyaddr, 
			unsigned long desphyaddr, 
			unsigned long len)
{
#define MAX_DESC_NUM		0x1000
#define SINGLE_DESC_TRANS_MAX  	8000

	pxa_dma_desc 	*display_dma_desc_tmp;
	unsigned long 	display_dma_desc_p_tmp;
	unsigned long 	len_tmp;

	if (len > (MAX_DESC_NUM-2)*SINGLE_DESC_TRANS_MAX) {
		printk(KERN_ERR "display size is too large\n");
		return -1;
	}
	if (len & 0x1f) {
		printk(KERN_ERR "display size is not 32 bytes aligned\n");
		return -1;
	}

	if (display_dma_ch == -1) {
		display_dma_ch = pxa_request_dma("mved_display",
					DMA_PRIO_HIGH, 
					display_dma_irq, 
					NULL);
		if (display_dma_ch < 0) {
			printk(KERN_ERR 
				"MVED: Cann't request DMA for display\n");
			return -1;
		} 		
	}
	
	if (display_dma_desc == NULL) {
		display_dma_desc = dma_alloc_writecombine( NULL, 
					MAX_DESC_NUM * sizeof(pxa_dma_desc),
					(void *)&display_dma_desc_p, 
				GFP_KERNEL);
		if (display_dma_desc == NULL) {
			printk(KERN_ERR "display dma desc allocate error!!\n");
			return -1;
		}
	}

	display_dma_desc_tmp = display_dma_desc;
	display_dma_desc_p_tmp = display_dma_desc_p;
	while (len) {
		len_tmp = len > SINGLE_DESC_TRANS_MAX ? 
				SINGLE_DESC_TRANS_MAX : len;
	        display_dma_desc_tmp->ddadr = display_dma_desc_p_tmp
		       			+ sizeof(pxa_dma_desc);
		display_dma_desc_tmp->dsadr = srcphyaddr;
		display_dma_desc_tmp->dtadr = desphyaddr;
		display_dma_desc_tmp->dcmd = len_tmp | DCMD_INCSRCADDR 
					| DCMD_INCTRGADDR | DCMD_BURST32;
		len -= len_tmp;
		display_dma_desc_tmp ++;
		display_dma_desc_p_tmp += sizeof(pxa_dma_desc);
		srcphyaddr += len_tmp;
		desphyaddr += len_tmp;
	}
	
        display_dma_desc_tmp->ddadr = display_dma_desc_p_tmp 
					+ sizeof(pxa_dma_desc);
	display_dma_desc_tmp->dsadr = srcphyaddr;
	display_dma_desc_tmp->dtadr = desphyaddr;
	display_dma_desc_tmp->dcmd = 0 | DCMD_INCSRCADDR | DCMD_INCTRGADDR
	       				| DCMD_BURST32 | DCMD_ENDIRQEN;

	display_dma_end = 0;
        DDADR(display_dma_ch) = (int) display_dma_desc_p;
	is_display_dma_pending = 1;
        DCSR(display_dma_ch) |= DCSR_RUN;
	
	return 0;
}

static void display_dma_irq(int channel, void *data)
{
	DCSR(channel) = DCSR_STARTINTR|DCSR_ENDINTR|DCSR_BUSERR;
	display_dma_end	= 1;
	wake_up_interruptible(&mved_displaydma_wait);

	return;
}



static irqreturn_t mved_dma_irq(int irq, void *ptr)
{
	int dint = MVED_DMA_DINT;
	
	if (dint & (1<<0)) {/*Decode Cmd buffer interrupt*/
		if (MVED_DMA_DCSR0&MVED_DMA_DCSR_END_INTR)
				wake_up_interruptible(&mved_dma_wait);

		MVED_DMA_DCSR0 = MVED_DMA_DCSR_BUS_ERR_INTR
				|MVED_DMA_DCSR_START_INTR
				|MVED_DMA_DCSR_END_INTR;

	} else if (dint & (1<<1)) {
		if (MVED_DMA_DCSR1&MVED_DMA_DCSR_END_INTR)
				wake_up_interruptible(&mved_dma_wait);
	
		MVED_DMA_DCSR1 = MVED_DMA_DCSR_BUS_ERR_INTR
				|MVED_DMA_DCSR_START_INTR
				|MVED_DMA_DCSR_END_INTR;

	} else {
		if(dint)
			printk(KERN_ERR "unknow mved dma interrupt, dint:0x%x!!\n", dint);
	}

	return IRQ_HANDLED;
}

static irqreturn_t mved_irq(int irq, void *ptr)
{
	unsigned int intstat = MVED_INTSTAT;

	if (intstat & (1<<3)) {
		printk(KERN_ERR "mtx halt interrupt!!\n");
	}
	if (intstat & (1<<2)) {
		wake_up_interruptible(&mved_mtx_wait);

	}
	if (intstat & (1<<1)) {
		pr_debug("unknow mvea interrupt!!\n");
	}
	if (intstat & (1<<0)) {
		unsigned int mvda2intstat = MVED_MVDA2INTSTAT;
		if(mvda2intstat & (1<<1)) {
			pxa3xx_mved->decode_mb_count ++;
			wake_up_interruptible(&mved_decode_mb_wait);
			pr_debug("MC complete interrupt!!\n");
		}else if(mvda2intstat & (1<<2)) {
			pr_debug("Deblock complete interrupt!!\n");
		}else {
			printk(KERN_ERR "unknow mvda2 interrupt!!\n");
		}
		MVED_MVDA2INTCLEAR = 0x7;
	}
	MVED_INTCLEAR = 0xf;
	return IRQ_HANDLED;
}

static int mved_usercopy(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg,
		int (*func)(struct inode *inode, struct file *file,
		unsigned int cmd, void *arg))
{
	char	sbuf[128];
	void    *mbuf = NULL;
	void	*parg = NULL;
	int	err  = -EINVAL;

	/*  Copy arguments into temp kernel buffer  */
	switch (_IOC_DIR(cmd)) {
	case _IOC_NONE:
		parg = NULL;
		break;
	case _IOC_READ:
	case _IOC_WRITE:
	case (_IOC_WRITE | _IOC_READ):
		if (_IOC_SIZE(cmd) <= sizeof(sbuf)) {
			parg = sbuf;
		} else {
			/* too big to allocate from stack */
			mbuf = kmalloc(_IOC_SIZE(cmd),GFP_KERNEL);
			if (NULL == mbuf)
				return -ENOMEM;
			parg = mbuf;
		}
		
		err = -EFAULT;
		if ((_IOC_DIR(cmd) & _IOC_WRITE) &&
		    (copy_from_user(parg, (void __user *)arg, _IOC_SIZE(cmd))))
				goto out;
		break;
	}

	/* call driver */
	err = func(inode, file, cmd, parg);
	if (err == -ENOIOCTLCMD)
		err = -EINVAL;
	if (err < 0)
		goto out;

	/*  Copy results into user buffer  */
	switch (_IOC_DIR(cmd)) {
		case _IOC_READ:
		case (_IOC_WRITE | _IOC_READ):
			if (copy_to_user((void __user *)arg, parg, _IOC_SIZE(cmd)))
				err = -EFAULT;
			break;
	}

out:
	if (mbuf)
		kfree(mbuf);
	return err;
}


static int mved_do_ioctl(struct inode *inode, struct file *file,
                unsigned int cmd, void *arg)
{
	int retval = 0;

	switch (cmd) {
		case MVED_S_UPDATEMEMINFO:
		{
			struct MEMORY_INFO_TAG *meminfo = 
						(struct MEMORY_INFO_TAG *)arg;
			if (!(meminfo->flag & MM_FLAG_UVALID) 
				|| !(meminfo->flag & MM_FLAG_LVALID)) {
				printk(KERN_ERR 
					"User addr or len is not valid!!\n");
				break;
			}
			meminfo->p_addr = 
			    (void *)uva_to_pa((unsigned long)meminfo->u_addr);
			if (meminfo->p_addr != NULL);
				meminfo->flag |= MM_FLAG_PVALID;
			if (check_phy_contiguous((unsigned long)meminfo->u_addr,
						meminfo->len)) {
				meminfo->flag |= MM_FLAG_PHYCONT;
			} else {
				printk(KERN_ERR 
					"Memory is not phy contiguous!!\n");
				break;
			}
			break;
		}		
		case MVED_S_DISPLAY_DMA_ISSUE:
		{
			struct mem_dma_t *mem_dma = (struct mem_dma_t *)arg;
			unsigned long srcphyaddr, destphyaddr;

			srcphyaddr = uva_to_pa(mem_dma->srcuseraddr);
			destphyaddr = uva_to_pa(mem_dma->taruseraddr);

			if (srcphyaddr == 0 || destphyaddr == 0)
				return -1;
			retval = display_do_dma(srcphyaddr, destphyaddr, 
						mem_dma->len);
			break;
		}		
		case MVED_S_DISPLAY_DMA_SYNC:
		{
			int *poll = (int *)arg;
			
			DECLARE_WAITQUEUE(wait, current);

			if (!is_display_dma_pending)
				break;

			if (*poll) {
				while(!display_dma_end) {
					;
				}		
				is_display_dma_pending = 0;
				break;
			}
			
			add_wait_queue(&mved_displaydma_wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);
			disable_irq(IRQ_DMA);
			if (display_dma_end) {
				enable_irq(IRQ_DMA);
				set_current_state(TASK_RUNNING);
				remove_wait_queue(&mved_displaydma_wait, &wait);
				is_display_dma_pending = 0;
				break;
			}		
			enable_irq(IRQ_DMA);
			schedule();
			set_current_state(TASK_RUNNING);
			remove_wait_queue(&mved_displaydma_wait, &wait);
			is_display_dma_pending = 0;
			break;
		}		
		case MVED_S_POWER_ON:
		{
			mved_power(1);
			break;
		}
		case MVED_S_POWER_OFF:
		{
			mved_power(0);
			break;
		}		
		case MVED_S_POWER_QUERY:
		{
			*((int *)arg) = pxa3xx_mved->powersave;
			break;
		}
		case MVED_S_RESET:
		{
			down_write(&pxa3xx_mved->sem);
			if(pxa3xx_mved->powerstate == MVED_POWERON) {
				MVED_DMA_DCSR0 &= ~(MVED_DMA_DCSR_RUN);
				MVED_DMA_DCSR1 &= ~(MVED_DMA_DCSR_RUN);
				while(MVED_DMA_DCSR0 & MVED_DMA_DCSR_RUN) {
					schedule_timeout(10);
				}
				while(MVED_DMA_DCSR1 & MVED_DMA_DCSR_RUN) {
					schedule_timeout(10);
				}
			}else {
				_mved_power(1);
			}
			up_write(&pxa3xx_mved->sem);
			BIMRES = 0;
			schedule_timeout(10);
			BIMRES |= (1<<8);
			schedule_timeout(10);
			BIMRES = 0;
			mved_power(0);
			schedule_timeout(10);
			mved_power(1);
			BIMRES = 0;
			schedule_timeout(10);
			BIMRES |= (1<<8);
			BIMFSR = 0x444;
			BIMESR = 0x3;
			BIMDRMR = 0x1;
			BIMDBSR = 0x2;
			BIMISR = 0x2;
			BIMCRR = 0x0;
			BIMIER = 0x02;
			BIMSQE = 1;
			BIMPMR1 = 0;
			BIMPMR2 = 0;
			MVED_DMA_DRCMR2=0x80;
			MVED_DMA_DRCMR3=0x81;
			mved_power(0);
			break;
		}
		case MVED_S_FLUSHCACHE:
		{
			struct MEMORY_INFO_TAG *meminfo = 
				(struct MEMORY_INFO_TAG *)arg;
			int len = meminfo->len;
			void *start, *end, *kstart, *kend, *pstart;
		       	start = meminfo->u_addr;
			end = (void *)((unsigned long)start + len);
			pstart = (void *)uva_to_pa((unsigned long)start);

			if (0x5c000000 == ((unsigned long)pstart & 0xff000000)){ 
				kstart = (void *)imm_get_virtual((unsigned long)pstart, current);
			}else {
				kstart = (void *)phys_to_virt((unsigned long)pstart);
			}
			
			kend = (void *)((unsigned int)kstart + len);
			pxa_flush_range(start, end);
			pxa_flush_range(kstart, kend);
			/*xsc3_flush_user_cache_range(start, end, 0);
			dmac_flush_range((void *)kstart, (void *)kend);
			*/
			break;
		}		
		case MVED_S_MTX_WAIT:
		{
			unsigned int timeout = *((unsigned int *)arg);
			DECLARE_WAITQUEUE(wait, current);

			add_wait_queue(&mved_mtx_wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);

			if (MVED_INTSTAT & (1<<2)) {
				MVED_INTCLEAR = 0xf;
				set_current_state(TASK_RUNNING);
				remove_wait_queue(&mved_mtx_wait, &wait);
				break;
			}
			enable_irq(pxa3xx_mved->mvedirq);
			*((unsigned int *)arg) = schedule_timeout(
					msecs_to_jiffies(timeout));
			disable_irq(pxa3xx_mved->mvedirq);
			set_current_state(TASK_RUNNING);
			remove_wait_queue(&mved_mtx_wait, &wait);
			break;
		}
		case MVED_S_DMA_WAITLAST:
		{
			unsigned int timeout = *((unsigned int *)arg);
			DECLARE_WAITQUEUE(wait, current);

			add_wait_queue(&mved_dma_wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);

			if (!((MVED_DMA_DCSR0&MVED_DMA_DCSR_END_INTR)==0 
				|| (MVED_DMA_DCSR1&MVED_DMA_DCSR_END_INTR)==0)){
				set_current_state(TASK_RUNNING);
				remove_wait_queue(&mved_dma_wait, &wait);
				break;
			}		
			enable_irq(pxa3xx_mved->mveddmairq);	
			*((unsigned int *)arg) = schedule_timeout(
						msecs_to_jiffies(timeout));
			disable_irq(pxa3xx_mved->mveddmairq);				
			set_current_state(TASK_RUNNING);
			remove_wait_queue(&mved_dma_wait, &wait);
			break;
		}
		case MVED_S_DEC_MB_ENABLE:
		{
			down_write(&pxa3xx_mved->sem);
			if(*((int *)arg) && !pxa3xx_mved->decode_mb_enable) {
				enable_irq(pxa3xx_mved->mvedirq);
				pxa3xx_mved->decode_mb_enable = 1;
			} else if (!(*((int *)arg)) && pxa3xx_mved->decode_mb_enable) {
				disable_irq(pxa3xx_mved->mvedirq);
				pxa3xx_mved->decode_mb_enable = 0;
			}
			up_write(&pxa3xx_mved->sem);
			break;
		}
		case MVED_S_DEC_MB_SET:
		{
			unsigned int mb= *((unsigned int *)arg);

			down_read(&pxa3xx_mved->sem);
			if(!pxa3xx_mved->decode_mb_enable) {
				up_read(&pxa3xx_mved->sem);
				printk(KERN_ERR "Enable the decode MB counting first!!\n" );
				return -EINVAL;
			}
			up_read(&pxa3xx_mved->sem);
			down_write(&pxa3xx_mved->sem);
			pxa3xx_mved->decode_mb_count = mb;
			up_write(&pxa3xx_mved->sem);
			break;
		}
		case MVED_S_DEC_MB_WAIT:
		{
			struct mb_wait_t *mb_wait = (struct mb_wait_t *)arg;
			DECLARE_WAITQUEUE(wait, current);

			down_read(&pxa3xx_mved->sem);
			if(!pxa3xx_mved->decode_mb_enable) {
				up_read(&pxa3xx_mved->sem);
				printk(KERN_ERR "Enable the decode MB counting first!!\n" );
				return -EINVAL;
			}
			up_read(&pxa3xx_mved->sem);

			for (;;) {
				disable_irq(pxa3xx_mved->mvedirq);
				add_wait_queue(&mved_decode_mb_wait, &wait);
				set_current_state(TASK_INTERRUPTIBLE);

				down_read(&pxa3xx_mved->sem);
				mb_wait->finish = pxa3xx_mved->decode_mb_count;
				up_read(&pxa3xx_mved->sem);
				if(mb_wait->finish >= mb_wait->wait ||
						0 == mb_wait->timeout) {
					set_current_state(TASK_RUNNING);
					remove_wait_queue(&mved_decode_mb_wait, &wait);
					enable_irq(pxa3xx_mved->mvedirq);
					break;
				}
				enable_irq(pxa3xx_mved->mvedirq);
				mb_wait->timeout = schedule_timeout(
						msecs_to_jiffies(mb_wait->timeout));
				remove_wait_queue(&mved_decode_mb_wait, &wait);
			}		
			break;
		}
		default:
		{
			retval = -ENOIOCTLCMD;
			printk(KERN_ERR "IO Command 0x%x is not defined\n", cmd);
			break;
		}
	}
    
    return retval;
}

static int pxa3xx_mved_ioctl(struct inode *inode, struct file *file,
                unsigned int cmd, unsigned long param)
{
	return mved_usercopy(inode, file, cmd, param, mved_do_ioctl);
}


static void pxa3xx_mved_vma_close(struct vm_area_struct *vma)
{
	struct memblk_info *memblk = vma->vm_private_data;
	unsigned long addr, size;

	if (memblk->vma != vma)
		return;

	if (MVED_MMAP_MALLOC == (memblk->type&MVED_MMAP_CMD_MASK)) {
		if (!((MVED_MMAP_SURFACEMEM_MASK & memblk->type)
			|| (MVED_MMAP_DEVICEMEM_MASK & memblk->type)
			|| (MVED_MMAP_SRAM_MASK & memblk->type))) {
			addr = (unsigned long)memblk->k_addr;
			size = (unsigned long)memblk->len;
			while(size > 0) {
				ClearPageReserved(virt_to_page(addr));
				addr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
		}

		if (MVED_MMAP_SRAM_MASK&memblk->type) {
#ifdef CONFIG_IMM
			if(cpu_is_pxa310()){
				imm_free(memblk->k_addr, pxa3xx_mved->immid);
			}else if(cpu_is_pxa935()){
				addr = (unsigned long)memblk->k_addr;
				size = (unsigned long)memblk->len;
				while(size > 0) {
					ClearPageReserved(virt_to_page(addr));
					addr += PAGE_SIZE;
					size -= PAGE_SIZE;
				}
				free_pages((unsigned long)memblk->k_addr, 
						get_order(memblk->len));
			}else {
				printk(KERN_ERR "Process does not support. \n");
				return;
			}
#else
			addr = (unsigned long)memblk->k_addr;
			size = (unsigned long)memblk->len;
			while(size > 0) {
				ClearPageReserved(virt_to_page(addr));
				addr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
			free_pages((unsigned long)memblk->k_addr, 
					get_order(memblk->len));
#endif
		}else if (MVED_MMAP_SURFACEMEM_MASK&memblk->type){
		}else if (MVED_MMAP_DEVICEMEM_MASK&memblk->type){
		}else {
			free_pages((unsigned long)memblk->k_addr, 
					get_order(memblk->len));
		}
	}	
	kfree(memblk);
	memblk = NULL;
}

static void pxa3xx_mved_vma_open(struct vm_area_struct *vma)
{
}

static struct vm_operations_struct pxa3xx_mved_remap_vm_ops = {
	.open = pxa3xx_mved_vma_open,
	.close = pxa3xx_mved_vma_close,
};

static int pxa3xx_mved_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long map_size = vma->vm_end - vma->vm_start;
 	int type = (int)vma->vm_pgoff<<PAGE_SHIFT;
	struct memblk_info *memblk = NULL;
	struct page *page;
	unsigned long addr, size;
	unsigned int map_flag = 0;

	memblk = (struct memblk_info *)kmalloc(sizeof(struct memblk_info), 
							GFP_KERNEL);
	if (NULL == memblk)
		return -ENOMEM;

	memblk->len = map_size;
	memblk->type = type;
	if (MVED_MMAP_MVED_REGS == (type & MVED_MMAP_CMD_MASK)){
		if (remap_pfn_range(vma, (unsigned long)vma->vm_start, 
				(unsigned long)(MVED_REG_PHYBASE >> PAGE_SHIFT),
			       	map_size, pgprot_noncached(PAGE_SHARED))) 
			return -EACCES;
	} else if (MVED_MMAP_MVED_DMA_REGS == (type & MVED_MMAP_CMD_MASK)){
		if (remap_pfn_range(vma, (unsigned long)vma->vm_start, 
			(unsigned long)(MVED_DMA_REG_PHYBASE >> PAGE_SHIFT), 
			map_size, pgprot_noncached(PAGE_SHARED))) 
			return -EACCES;
	} else if (MVED_MMAP_BPB2IMG_REGS == (type & MVED_MMAP_CMD_MASK)){
		if (remap_pfn_range(vma, (unsigned long)vma->vm_start, 
			(unsigned long)(MVED_BPB2IMG_REG_PHYBASE >> PAGE_SHIFT),
		       	map_size, pgprot_noncached(PAGE_SHARED))) 
			return -EACCES;
	} else if (MVED_MMAP_MALLOC == (type & MVED_MMAP_CMD_MASK)) {
		if (MVED_MMAP_CACHE_MASK&type)
			map_flag = __pgprot(
				pgprot_val(PAGE_SHARED) & ~(L_PTE_BUFFERABLE));
		else if(MVED_MMAP_WRCOMB_MASK&type)
			map_flag = pgprot_writecombine(PAGE_SHARED);
		else
			map_flag = pgprot_noncached(PAGE_SHARED);
		
		if (MVED_MMAP_SURFACEMEM_MASK & type) {
#ifdef CONFIG_MVED_RES_MEM
			if (!pxa3xx_mved->surfacemem) {
				return -EINVAL;
			}
			size = (unsigned long)memblk->len;
			if (size > pxa3xx_mved->surfacememlen) {
				printk(KERN_ERR 
					"required memory size is too lage\n");
				return -EINVAL;
			}
			memblk->u_addr = (void *)vma->vm_start;
			memblk->k_addr = (void *)pxa3xx_mved->surfacemem;
			memblk->p_addr = (void *)pxa3xx_mved->surfacememphy;
#else
			printk(KERN_ERR
					"surface reserve memory not enabled\n");
			return -EINVAL;
#endif
		} else if (MVED_MMAP_DEVICEMEM_MASK & type) {
#ifdef CONFIG_MVED_RES_MEM
			if (!pxa3xx_mved->devicemem) {
				return -EINVAL;
			}
			size = (unsigned long)memblk->len;
			if (size > pxa3xx_mved->devicememlen) {
				printk(KERN_ERR 
					"required memory size is too lage\n");
				return -EINVAL;
			}
			memblk->u_addr = (void *)vma->vm_start;
			memblk->k_addr = (void *)pxa3xx_mved->devicemem;
			memblk->p_addr = (void *)pxa3xx_mved->devicememphy;
#else
			printk(KERN_ERR
					"device reserve memory not enabled\n");
			return -EINVAL;
#endif

		} else if (MVED_MMAP_SRAM_MASK & type) {
#ifdef CONFIG_IMM
			if(cpu_is_pxa310()){
				memblk->u_addr = (void *)vma->vm_start;
				memblk->k_addr = imm_malloc(map_size, 
						IMM_MALLOC_HARDWARE | IMM_MALLOC_SRAM, 
						pxa3xx_mved->immid);
				if (NULL == memblk->k_addr) {
					printk(KERN_ERR "imm_malloc error\n");
					return -EINVAL;
				}
				memblk->p_addr = (void *)imm_get_physical(
						memblk->k_addr, 
						pxa3xx_mved->immid);
			}else if(cpu_is_pxa935()){
				page = alloc_pages(GFP_KERNEL, 
						get_order(memblk->len));
				if (page == NULL) {
					printk(KERN_ERR "alloc_pages failed. \n");
					return -ENOMEM;
				}
				memblk->u_addr = (void *)vma->vm_start;
				memblk->k_addr = (void *)page_address(page);
				memblk->p_addr = (void *)virt_to_phys(
						memblk->k_addr);
				addr = (unsigned long)memblk->k_addr;
				size = (unsigned long)memblk->len;
				while(size > 0) {
					SetPageReserved(virt_to_page(addr));
					addr += PAGE_SIZE;
					size -= PAGE_SIZE;
				}
			}else {
				printk(KERN_ERR "Process does not support. \n");
				return -ENODEV;
			}
#else
			page = alloc_pages(GFP_KERNEL, 
						get_order(memblk->len));
			if (page == NULL) {
				printk(KERN_ERR "alloc_pages failed. \n");
				return -ENOMEM;
			}
			memblk->u_addr = (void *)vma->vm_start;
			memblk->k_addr = (void *)page_address(page);
			memblk->p_addr = (void *)virt_to_phys(
							memblk->k_addr);
			addr = (unsigned long)memblk->k_addr;
			size = (unsigned long)memblk->len;
			while(size > 0) {
				SetPageReserved(virt_to_page(addr));
				addr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
#endif
		} else {
			page = alloc_pages(GFP_KERNEL, 
						get_order(memblk->len));
			if (page == NULL) {
				printk(KERN_ERR "alloc_pages failed. \n");
				return -ENOMEM;
			}
			memblk->u_addr = (void *)vma->vm_start;
			memblk->k_addr = (void *)page_address(page);
			memblk->p_addr = (void *)virt_to_phys(
							memblk->k_addr);
			addr = (unsigned long)memblk->k_addr;
			size = (unsigned long)memblk->len;
			while(size > 0) {
				SetPageReserved(virt_to_page(addr));
				addr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
		}
		
		if (remap_pfn_range(vma, (unsigned long)memblk->u_addr, 
				((unsigned long)memblk->p_addr>>PAGE_SHIFT),
			       	map_size, map_flag))
		{
			printk(KERN_ERR "remap error!\n");
			return -EINVAL;
		}
		
		((unsigned long *)memblk->k_addr)[0] = 
					(unsigned long)memblk->k_addr;
		((unsigned long *)memblk->k_addr)[1] = 
					(unsigned long)memblk->u_addr;
		((unsigned long *)memblk->k_addr)[2] = 
					(unsigned long)memblk->p_addr;

		pxa_inv_range(memblk->u_addr, 
				(void *)((unsigned int)memblk->u_addr+memblk->len));
		pxa_flush_range(memblk->k_addr, 
				(void *)((unsigned int)memblk->k_addr+memblk->len));

		/*xsc3_flush_user_cache_range(
				(unsigned int)memblk->u_addr, 
				(unsigned int)memblk->u_addr+memblk->len, 0);
		dmac_flush_range(memblk->k_addr, 
				memblk->k_addr+memblk->len);
		*/
	} else {
		printk(KERN_ERR "command not support\n");
		return -EINVAL;
	}
	memblk->vma = vma;
	vma->vm_ops = &pxa3xx_mved_remap_vm_ops;
	vma->vm_private_data = memblk;
	return 0;
}


static int pxa3xx_mved_open(struct inode *inode, struct file *file)
{
	int first = 0;

	down_write(&pxa3xx_mved->sem);
	if (pxa3xx_mved->usercount >= 2){
		up_write(&pxa3xx_mved->sem);
		printk(KERN_ERR "MVED driver is busy!!!!\n");
		return -EBUSY;
	}
	if(pxa3xx_mved->usercount == 0)
		first = 1;
	pxa3xx_mved->usercount++;
	pxa3xx_mved->poweroncount = 0;
	pxa3xx_mved->poweroffcount = 0;
	pxa3xx_mved->dvfmcount = 0;
	pxa3xx_mved->decode_mb_count = 0;
	up_write(&pxa3xx_mved->sem);

	if (!first)
		return 0;       

	mved_power(1);
	BIMRES = 0;
	schedule_timeout(10);
	BIMRES |= (1<<8);

	BIMFSR = 0x444;
	BIMESR = 0x3;
	BIMDRMR = 0x1;
	BIMDBSR = 0x2;
	BIMISR = 0x2;
	BIMCRR = 0x0;
	BIMIER = 0x02;
	BIMSQE = 1;
	BIMPMR1 = 0;
	BIMPMR2 = 0;

	MVED_DMA_DRCMR2=0x80;
	MVED_DMA_DRCMR3=0x81;

	mved_power(0);

	return 0;
}

static int pxa3xx_mved_close(struct inode *inode, struct file *file)
{
	int last = 0;

	down_write(&pxa3xx_mved->sem);
	pxa3xx_mved->usercount--;
	if (pxa3xx_mved->usercount <0){
		up_write(&pxa3xx_mved->sem);
		printk(KERN_ERR "Can't close driver!\n");
		return -EPERM;
	}
	if(pxa3xx_mved->usercount == 0)
		last = 1;
	up_write(&pxa3xx_mved->sem);

	if (!last)
		return 0;       

	down_write(&pxa3xx_mved->sem);
	if(pxa3xx_mved->decode_mb_enable) {
		disable_irq(pxa3xx_mved->mvedirq);
		pxa3xx_mved->decode_mb_enable = 0;
	}
	if(pxa3xx_mved->powerstate == MVED_POWERON) {
		MVED_DMA_DCSR0 &= ~(MVED_DMA_DCSR_RUN);
		MVED_DMA_DCSR1 &= ~(MVED_DMA_DCSR_RUN);
		while(MVED_DMA_DCSR0 & MVED_DMA_DCSR_RUN) {
			schedule_timeout(10);
		}
		while(MVED_DMA_DCSR1 & MVED_DMA_DCSR_RUN) {
			schedule_timeout(10);
		}
	}else {
		_mved_power(1);
	}
	up_write(&pxa3xx_mved->sem);
	BIMRES = 0;
	schedule_timeout(10);
	BIMRES |= (1<<8);
	schedule_timeout(10);
	BIMRES = 0;
	mved_power(0);

	if (display_dma_desc != NULL) {
		dma_free_writecombine (NULL, 
				MAX_DESC_NUM * sizeof (pxa_dma_desc),
				display_dma_desc,
				(int)display_dma_desc_p);
		display_dma_desc = NULL;

	}
	if (display_dma_ch != -1) {
		pxa_free_dma(display_dma_ch);
		display_dma_ch = -1;

	}
	is_display_dma_pending = 0;		

	return 0;
}

static struct file_operations pxa3xx_mved_fops = {
	.owner		= THIS_MODULE,
	.open     	= pxa3xx_mved_open,
	.release	= pxa3xx_mved_close,
	.ioctl        	= pxa3xx_mved_ioctl,
	.mmap		= pxa3xx_mved_mmap,
};

static struct miscdevice pxa3xx_mved_miscdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "mved",
	.fops	= &pxa3xx_mved_fops,
};

#ifdef CONFIG_PM
static int pxa3xx_mved_resume(struct platform_device *pdev)
{	

	down_read(&pxa3xx_mved->sem);
	if(!pxa3xx_mved->usercount) {
		up_read(&pxa3xx_mved->sem);
		return 0;
	}
	up_read(&pxa3xx_mved->sem);
	mved_power(1);
	return 0;
}
static int pxa3xx_mved_suspend(struct platform_device *pdev, pm_message_t state)
{
	down_read(&pxa3xx_mved->sem);
	if(!pxa3xx_mved->usercount) {
		up_read(&pxa3xx_mved->sem);
		return 0;
	}
	up_read(&pxa3xx_mved->sem);
	mved_power(0);
	return 0;
}
#else
#define pxa3xx_mved_suspend      NULL
#define pxa3xx_mved_resume       NULL
#endif



static int __init pxa3xx_mved_probe(struct platform_device *pdev)
{
	int err = 0;
	struct resource *resource;
	struct proc_dir_entry *pxa3xx_mved_proc_entry;	

	if(!cpu_is_pxa310() && !cpu_is_pxa935())
		return -ENODEV;
	init_rwsem(&pxa3xx_mved->sem);
	pxa3xx_mved->clk = clk_get(&pdev->dev, "MVEDCLK");	
	if (IS_ERR(pxa3xx_mved->clk)) {
		err = PTR_ERR(pxa3xx_mved->clk);
		goto out;
	}
	unset_dvfm_constraint();
	pxa3xx_mved->powerstate=MVED_POWEROFF;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (resource == NULL) 
		goto res_err;
	pxa3xx_mved->mvedphybase = resource->start;
	pxa3xx_mved->mvedbase = ioremap_nocache(resource->start, 
			resource->end - resource->start + 1);
	if(pxa3xx_mved->mvedbase == NULL) 
		goto res_err;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (resource == NULL) 
		goto res_err;
	pxa3xx_mved->mveddmaphybase = resource->start;
	pxa3xx_mved->mveddmabase = ioremap_nocache(resource->start, 
			resource->end - resource->start + 1);
	if(pxa3xx_mved->mveddmabase == NULL) 
		goto res_err;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (resource == NULL) 
		goto res_err;
	pxa3xx_mved->bpb2imgphybase = resource->start;
	pxa3xx_mved->bpb2imgbase = ioremap_nocache(resource->start, 
			resource->end - resource->start + 1);
	if(pxa3xx_mved->bpb2imgbase == NULL) 
		goto res_err;

	resource = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (resource == NULL) 
		goto res_err;
	pxa3xx_mved->mvedirq = resource->start;

	resource = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (resource == NULL) 
		goto res_err;
	pxa3xx_mved->mveddmairq = resource->start;

#ifdef CONFIG_IMM
	pxa3xx_mved->immid = imm_register_kernel("mved");
#endif

#ifdef CONFIG_MVED_RES_MEM
	{
	struct page *page;
	unsigned long addr, size;

	page = alloc_pages(GFP_KERNEL, get_order(MVED_SURFACE_RES_MEM_LEN));
	if (page == NULL){
		dev_err(&pdev->dev, "alloc_pages failed\n");
		err = -ENOMEM;
		goto resmem_err;
	}
	pxa3xx_mved->surfacemem = (unsigned long)page_address(page);
	pxa3xx_mved->surfacememphy = (unsigned long)virt_to_phys(page_address(page));
	pxa3xx_mved->surfacememlen = MVED_SURFACE_RES_MEM_LEN;
	addr = pxa3xx_mved->surfacemem;
	size = pxa3xx_mved->surfacememlen;
	while(size > 0) {
		SetPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	page = alloc_pages(GFP_KERNEL, get_order(MVED_DEVICE_RES_MEM_LEN));
	if (page == NULL){
		dev_err(&pdev->dev, "alloc_pages failed\n");
		err = -ENOMEM;
		goto resmem_err;
	}
	pxa3xx_mved->devicemem = (unsigned long)page_address(page);
	pxa3xx_mved->devicememphy = (unsigned long)virt_to_phys(page_address(page));
	pxa3xx_mved->devicememlen = MVED_DEVICE_RES_MEM_LEN;
	addr = pxa3xx_mved->devicemem;
	size = pxa3xx_mved->devicememlen;
	while(size > 0) {
		SetPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	}
#endif

	err = request_irq(pxa3xx_mved->mveddmairq, mved_dma_irq, 0, 
			"mved_dma", NULL);
	if(err) 
		goto resmem_err;
	disable_irq(pxa3xx_mved->mveddmairq);				
	err = request_irq(pxa3xx_mved->mvedirq, mved_irq, 0, "mved", NULL);
	if(err) 
		goto free_mveddmairq;
	disable_irq(pxa3xx_mved->mvedirq);

	err = misc_register(&pxa3xx_mved_miscdev);
	if (err < 0) {
		dev_err(&pdev->dev, 
			"unable to register character device /dev/mved\n");
		goto free_mvedirq;
	}

	platform_set_drvdata(pdev, pxa3xx_mved);

	pxa3xx_mved_proc_entry = create_proc_entry("driver/mved", 0, NULL);
	if (pxa3xx_mved_proc_entry) { 
		pxa3xx_mved_proc_entry->data = NULL; 
		pxa3xx_mved_proc_entry->read_proc = pxa3xx_mved_proc_read; 
		pxa3xx_mved_proc_entry->write_proc = pxa3xx_mved_proc_write; 
	} 

	goto out;

free_mvedirq:
		free_irq(pxa3xx_mved->mvedirq, NULL);
free_mveddmairq:
		free_irq(pxa3xx_mved->mveddmairq, NULL);
resmem_err:
#ifdef CONFIG_MVED_RES_MEM
	{
	unsigned long addr, size;
	if(pxa3xx_mved->surfacemem) {
		addr = pxa3xx_mved->surfacemem;
		size = pxa3xx_mved->surfacememlen;
		while(size > 0) {
			ClearPageReserved(virt_to_page(addr));
			addr += PAGE_SIZE;
			size -= PAGE_SIZE;
		}
		free_pages(pxa3xx_mved->surfacemem, 
				get_order(pxa3xx_mved->surfacememlen));

	}
	if(pxa3xx_mved->devicemem) {
		addr = pxa3xx_mved->devicemem;
		size = pxa3xx_mved->devicememlen;
		while(size > 0) {
			ClearPageReserved(virt_to_page(addr));
			addr += PAGE_SIZE;
			size -= PAGE_SIZE;
		}
		free_pages(pxa3xx_mved->devicemem, 
				get_order(pxa3xx_mved->devicememlen));

	}
	}
#endif
	goto put_clk;
res_err:
	dev_err(&pdev->dev, "Get resource failed\n");
	if(pxa3xx_mved->mvedbase)
		iounmap(pxa3xx_mved->mvedbase);	
	if(pxa3xx_mved->mveddmabase)
		iounmap(pxa3xx_mved->mvedbase);	
	if(pxa3xx_mved->bpb2imgbase)
		iounmap(pxa3xx_mved->mvedbase);	
	err = -ENODEV;
put_clk:
	clk_put(pxa3xx_mved->clk);

out:
	return err;	
}

static int pxa3xx_mved_remove(struct platform_device *pdev)
{
	struct pxa3xx_mved_t *pxa3xx_mved = platform_get_drvdata(pdev);
	
	misc_deregister(&pxa3xx_mved_miscdev);
	free_irq(pxa3xx_mved->mvedirq, NULL);
	free_irq(pxa3xx_mved->mveddmairq, NULL);
#ifdef CONFIG_MVED_RES_MEM
	{
	unsigned long addr, size;
	if(pxa3xx_mved->surfacemem) {
		addr = pxa3xx_mved->surfacemem;
		size = pxa3xx_mved->surfacememlen;
		while(size > 0) {
			ClearPageReserved(virt_to_page(addr));
			addr += PAGE_SIZE;
			size -= PAGE_SIZE;
		}
		free_pages(pxa3xx_mved->surfacemem, 
				get_order(pxa3xx_mved->surfacememlen));

	}
	if(pxa3xx_mved->devicemem) {
		addr = pxa3xx_mved->devicemem;
		size = pxa3xx_mved->devicememlen;
		while(size > 0) {
			ClearPageReserved(virt_to_page(addr));
			addr += PAGE_SIZE;
			size -= PAGE_SIZE;
		}
		free_pages(pxa3xx_mved->devicemem, 
				get_order(pxa3xx_mved->devicememlen));

	}
	}
#endif
	if(pxa3xx_mved->mvedbase)
		iounmap(pxa3xx_mved->mvedbase);	
	if(pxa3xx_mved->mveddmabase)
		iounmap(pxa3xx_mved->mvedbase);	
	if(pxa3xx_mved->bpb2imgbase)
		iounmap(pxa3xx_mved->mvedbase);	
	clk_put(pxa3xx_mved->clk);
	platform_set_drvdata(pdev, NULL);

	return 0;
}


static struct platform_driver mved_driver = {
	.driver         = {
		.name   = "pxa3xx-mved",
	},
	.probe          = pxa3xx_mved_probe,
	.remove         = pxa3xx_mved_remove,
#if 0 /* rm by frank */
	//.suspend        = pxa3xx_mved_suspend,
	//.resume         = pxa3xx_mved_resume,
#endif
};

static int __init pxa3xx_mved_init(void)
{
#if defined(CONFIG_PXA3xx_DVFM)
	dvfm_register("MVED", &dvfm_lock.dev_idx);
	dvfm_register_notifier(&notifier_freq_block,
			DVFM_FREQUENCY_NOTIFIER);
#endif
	return platform_driver_register(&mved_driver);
}

static void __exit pxa3xx_mved_exit(void)
{
#if defined(CONFIG_PXA3xx_DVFM)
	dvfm_unregister_notifier(&notifier_freq_block,
			DVFM_FREQUENCY_NOTIFIER);
	dvfm_unregister("MVED", &dvfm_lock.dev_idx);
#endif
	platform_driver_unregister(&mved_driver);
}

module_init(pxa3xx_mved_init);
module_exit(pxa3xx_mved_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bin Yang (bin.yang@marvell.com)");
MODULE_DESCRIPTION("MVED Driver");



