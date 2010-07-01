/*
 *  linux/drivers/mmc/host/pxa9xx_mci.c - PXA9XX MMCI driver
 *
 *  Based on linux/drivers/mmc/host/pxamci.c
 *
 *  Copyright:	(C) Copyright 2008 Marvell International Ltd.
 *              Mingwei Wang <mwwang@marvell.com>
 *
 *  linux/drivers/mmc/host/pxa.c - PXA MMCI driver
 *
 *  Copyright (C) 2003 Russell King, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  This hardware is really sick:
 *   - No way to clear interrupts.
 *   - Have to turn off the clock whenever we touch the device.
 *   - Doesn't tell you how many data blocks were transferred.
 *  Yuck!
 *
 *	1 and 3 byte data transfers not supported
 *	max block length up to 1023
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/timer.h>

#include <asm/dma.h>
#include <asm/io.h>
#include <asm/sizes.h>

#include <mach/pxa-regs.h>
#include <mach/mmc.h>

#include "pxa9xx_mci.h"

#ifdef CONFIG_PXA3xx_DVFM
#include <mach/dvfm.h>
#include <mach/pxa3xx_dvfm.h>
#endif

#define DRIVER_NAME	"pxa9xx-mci"

#define NR_SG	32
#define CLKRT_OFF	(~0)
#define PXA_TIMER_TIMEOUT 500

struct adma_desc_table {
	u16 entry_attr;
	u16 entry_length;
	u32 entry_address;
};

struct pxa9xx_mci_host {
	struct mmc_host		*mmc;
	spinlock_t		lock;
	struct resource		*res;
	void __iomem		*base;
	struct clk		*clk;
	unsigned long		clkrate;
	int			irq;
	unsigned int		clkrt;
	unsigned int		dma_bufsz;
	unsigned int		segment;
	unsigned int		i_stat_sig;
	unsigned int		power_mode;
	struct pxamci_platform_data *pdata;

	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_data		*data;

	struct adma_desc_table	*sg_cpu;
	dma_addr_t		sg_dma;
	unsigned int		dma_len;
	unsigned int		dma_dir;

	struct timer_list	timer;
};

#if defined(CONFIG_PXA3xx_DVFM)
static struct dvfm_lock dvfm_lock = {
	.lock		= SPIN_LOCK_UNLOCKED,
	.count		= 0,
	.dev_idx	= -1,
};

static void set_dvfm_constraint(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	if (dvfm_lock.count++ == 0) {
		/* Disable Low power mode */
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
		dvfm_enable_op_name("D1", dvfm_lock.dev_idx);
		dvfm_enable_op_name("D2", dvfm_lock.dev_idx);
		if (cpu_is_pxa935())
			dvfm_enable_op_name("CG", dvfm_lock.dev_idx);
	} else
		dvfm_lock.count++;
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}
#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
#endif

static void dump_registers(struct pxa9xx_mci_host *host)
{
	unsigned int val;
	int offset;

	for (offset = 0; offset < 0x44; offset += 4) {
		val = readl(host->base + offset);
		pr_debug("%08x: %08x\n", (unsigned int)host->base + offset, val);
	}
	val = readl(host->base + 0x48);
	pr_debug("%08x: %08x\n", (unsigned int)host->base + 0x48, val);
	val = readl(host->base + 0xFC);
	pr_debug("%08x: %08x\n", (unsigned int)host->base + 0xFC, val);
}
static void pxa9xx_mci_stop_clock(struct pxa9xx_mci_host *host)
{
	unsigned int val;

	val = readl(host->base + MM4_CNTL2);
	writel(val & ~(MM4CLKEN | INTCLKEN), host->base + MM4_CNTL2);
}

static void pxa9xx_mci_start_clock(struct pxa9xx_mci_host *host)
{
	unsigned int val;
	unsigned long timeout = 10000;

	val = readl(host->base + MM4_CNTL2);
	writel(val | INTCLKEN, host->base + MM4_CNTL2);
	do {
		val = readl(host->base + MM4_CNTL2);
		if (val & INTCLKSTB)
			break;
		udelay(1);
	} while (timeout--);
	if (!(val & INTCLKSTB))
		dev_err(mmc_dev(host->mmc), "unable to start clock\n");

	val &= ~(0xff << SDFREQ_OFFSET);
	val |= host->clkrt << SDFREQ_OFFSET;
	writel(val, host->base + MM4_CNTL2);

	val = readl(host->base + MM4_CNTL2);
	writel(val | MM4CLKEN, host->base + MM4_CNTL2);
	do {
		val = readl(host->base + MM4_CNTL2);
		if (val & MM4CLKEN)
			break;
		udelay(1);
	} while (timeout--);

	if (!(val & MM4CLKEN))
		dev_err(mmc_dev(host->mmc), "unable to start clock\n");
}

static void pxa9xx_mci_enable_clock(struct mmc_host *mmc)
{
	struct pxa9xx_mci_host *host = mmc_priv(mmc);

	clk_enable(host->clk);
	pxa9xx_mci_start_clock(host);
}

static void pxa9xx_mci_disable_clock(struct mmc_host *mmc)
{
	struct pxa9xx_mci_host *host = mmc_priv(mmc);

	pxa9xx_mci_stop_clock(host);
	clk_disable(host->clk);
}

static void pxa9xx_mci_enable_irq(struct pxa9xx_mci_host *host, unsigned int mask)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->i_stat_sig |= mask;
	writel(host->i_stat_sig, host->base + MM4_I_STAT_EN);
	writel(host->i_stat_sig, host->base + MM4_I_SIG_EN);
	writel(host->i_stat_sig, host->base + MM4_I_STAT);
	spin_unlock_irqrestore(&host->lock, flags);
}

static void pxa9xx_mci_disable_irq(struct pxa9xx_mci_host *host, unsigned int mask)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->i_stat_sig &= ~mask;
	writel(host->i_stat_sig, host->base + MM4_I_STAT_EN);
	writel(host->i_stat_sig, host->base + MM4_I_SIG_EN);
	writel(host->i_stat_sig, host->base + MM4_I_STAT);
	spin_unlock_irqrestore(&host->lock, flags);
}

static void pxa9xx_mci_setup_data(struct pxa9xx_mci_host *host, struct mmc_data *data)
{
	unsigned int nob = data->blocks;

	host->segment = 0;

	host->data = data;

	if (data->flags & MMC_DATA_STREAM)
		nob = 0xffff;

	if (data->flags & MMC_DATA_READ) {
		host->dma_dir = DMA_FROM_DEVICE;
	} else {
		host->dma_dir = DMA_TO_DEVICE;
	}

	host->dma_len = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
				   host->dma_dir);
	writel(sg_dma_address(&data->sg[host->segment]), host->base +
		MM4_SYSADDR);

	pr_debug("PXA9XX_MCI: nob = %d, blksz = %d\n", nob, data->blksz);
	writel(nob << BLK_CNT_OFFSET | host->dma_bufsz << DMA_BUFSZ_OFFSET |
		data->blksz, host->base + MM4_BLK_CNTL);
}

static void pxa9xx_mci_setup_adma_data(struct pxa9xx_mci_host *host, struct mmc_data *data)
{
	unsigned int nob = data->blocks;
	unsigned int val;
	int i;

	host->data = data;

	if (data->flags & MMC_DATA_STREAM)
		nob = 0xffff;

	if (data->flags & MMC_DATA_READ) {
		host->dma_dir = DMA_FROM_DEVICE;
	} else {
		host->dma_dir = DMA_TO_DEVICE;
	}

	host->dma_len = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
				   host->dma_dir);

	for (i = 0; i < host->dma_len; i++) {
		host->sg_cpu[i].entry_address = sg_dma_address(&data->sg[i]);
		host->sg_cpu[i].entry_length = sg_dma_len(&data->sg[i]);
		host->sg_cpu[i].entry_attr = 0x0021;
	}
	/*The Last Link, Set End bit*/
	host->sg_cpu[host->dma_len - 1].entry_attr |= 0x2;

	/*Set ADMA System Address Reg*/
	writel(host->sg_dma, host->base + MMC4_AMDA_ADDR1);

	pr_debug("PXA9XX_MCI: nob = %d, blksz = %d\n", nob, data->blksz);
	writel(nob << BLK_CNT_OFFSET | host->dma_bufsz << DMA_BUFSZ_OFFSET |
		data->blksz, host->base + MM4_BLK_CNTL);

	val = readl(host->base + MM4_CNTL1);
	writel(val | 0x2 << DMA_SEL_OFFSET, host->base + MM4_CNTL1);
}

static void pxa9xx_mci_start_cmd(struct pxa9xx_mci_host *host, struct mmc_command *cmd)
{
	unsigned int rsp = 0;
	unsigned int xfrmd = 0;

	WARN_ON(host->cmd != NULL);
	host->cmd = cmd;

	/*Set Response Type*/
	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		break;

	case MMC_RSP_R1: /* r1, r5, r6, r7 */
		rsp = CMD_RSP_48BIT;
		xfrmd |= CRCCHKEN | IDXCHKEN;
		break;

	case MMC_RSP_R2: /* r2 */
		rsp = CMD_RSP_136BIT;
		xfrmd |= CRCCHKEN;
		break;

	case MMC_RSP_R3: /* r3, r4*/
		rsp = CMD_RSP_48BIT;
		break;

	case MMC_RSP_R1B: /* r1b */
		rsp = CMD_RSP_48BITB;
		xfrmd |= CRCCHKEN | IDXCHKEN;
		break;

	default:
		break;
	}

	/*Set Transfer mode regarding to data flag*/
	if (cmd->data)
	{
		/*Set Data Present*/
		xfrmd |= DPSEL | BLKCNTEN | DMA_EN | MS_BLKSEL;

		if (cmd->data->flags & MMC_DATA_READ)
			xfrmd |= DXFRDIR;
		else
			xfrmd &= ~DXFRDIR;
	}

	pxa9xx_mci_enable_irq(host, ~CDINT);

	writel(cmd->arg, host->base + MM4_ARG);
	xfrmd |= cmd->opcode << CMD_IDX_OFFSET | rsp << RES_TYPE_OFFSET;
	writel(xfrmd, host->base + MM4_CMD_XFRMD);

	pxa9xx_mci_enable_clock(host->mmc);
}

static void pxa9xx_mci_finish_request(struct pxa9xx_mci_host *host,
	struct mmc_request *mrq)
{
	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;
	mmc_request_done(host->mmc, mrq);
	unset_dvfm_constraint();
}

static int pxa9xx_mci_cmd_done(struct pxa9xx_mci_host *host, unsigned int stat)
{
	struct mmc_command *cmd = host->cmd;

	if (!cmd)
		return 0;

	host->cmd = NULL;

	if (mmc_resp_type(cmd) == MMC_RSP_R2) {
		cmd->resp[0] = readl(host->base + MM4_RESP3) << 8 |
				readl(host->base + MM4_RESP2) >> 24;
		cmd->resp[1] = readl(host->base + MM4_RESP2) << 8 |
				readl(host->base + MM4_RESP1) >> 24;
		cmd->resp[2] = readl(host->base + MM4_RESP1) << 8 |
				readl(host->base + MM4_RESP0) >> 24;
		cmd->resp[3] = readl(host->base + MM4_RESP0) << 8;
	} else {
		cmd->resp[0] = readl(host->base + MM4_RESP0);
		cmd->resp[1] = readl(host->base + MM4_RESP1);
		cmd->resp[2] = readl(host->base + MM4_RESP2);
		cmd->resp[3] = readl(host->base + MM4_RESP3);
	}

	if (stat & CTO)
		cmd->error = -ETIMEDOUT;
	else if (stat & CCRC && cmd->flags & MMC_RSP_CRC)
		cmd->error = -EILSEQ;

	pxa9xx_mci_disable_irq(host, CMDCOMP);
	if (!host->data || cmd->error) {
		pxa9xx_mci_finish_request(host, host->mrq);
	}

	return 1;
}

static int pxa9xx_mci_data_done(struct pxa9xx_mci_host *host, unsigned int stat)
{
	struct mmc_data *data = host->data;

	if (!data)
		return 0;

	dma_unmap_sg(mmc_dev(host->mmc), data->sg, host->dma_len,
		     host->dma_dir);

	if (stat & DTO)
		data->error = -ETIMEDOUT;
	else if (stat & DCRC)
		data->error = -EILSEQ;

	/*
	 * There appears to be a hardware design bug here.  There seems to
	 * be no way to find out how much data was transferred to the card.
	 * This means that if there was an error on any block, we mark all
	 * data blocks as being in error.
	 */
	if (!data->error)
		data->bytes_xfered = data->blocks * data->blksz;
	else
		data->bytes_xfered = 0;

	pxa9xx_mci_disable_irq(host, XFRCOMP);

	host->data = NULL;
	if (host->mrq->stop) {
		pxa9xx_mci_start_cmd(host, host->mrq->stop);
	} else {
		pxa9xx_mci_finish_request(host, host->mrq);
	}

	return 1;
}

static irqreturn_t pxa9xx_mci_irq(int irq, void *devid)
{
	struct pxa9xx_mci_host *host = devid;
	unsigned int stat, val;
	int handled = 0;

	stat = readl(host->base + MM4_I_STAT) & readl(host->base + MM4_I_STAT_EN);

	if (stat & CMDCOMP) {
		writel(CMDCOMP, host->base + MM4_I_STAT);
		handled |= pxa9xx_mci_cmd_done(host, stat);
	}
	if (stat & XFRCOMP) {
		writel(XFRCOMP, host->base + MM4_I_STAT);
		handled |= pxa9xx_mci_data_done(host, stat);
	}
	if (stat & DMAINT) {
		writel(DMAINT, host->base + MM4_I_STAT);
		if (++host->segment < host->dma_len)
			writel(sg_dma_address(&host->data->sg[host->segment]),
				host->base + MM4_SYSADDR);
		pr_debug("PXA9XX_MCI: len = %d, seg = %d\n", host->dma_len, host->segment);
		handled = 1;
	}
	if (stat & DEND) {
		writel(DEND, host->base + MM4_I_STAT);
		handled = 1;
	}
	if (stat & DCRC) {
		writel(DCRC, host->base + MM4_I_STAT);
		handled |= pxa9xx_mci_data_done(host, stat);
	}
	if (stat & DTO) {
		writel(DTO, host->base + MM4_I_STAT);
		handled |= pxa9xx_mci_data_done(host, stat);
	}
	if (stat & ERRINT) {
		writel(ERRINT, host->base + MM4_I_STAT);
		handled = 1;
	}
	if (stat & CDINT) {
		writel(~CDINT, host->base + MM4_I_STAT_EN);
		mmc_signal_sdio_irq(host->mmc);
		handled = 1;
	}
	if (stat & CCRC) {
		writel(CCRC, host->base + MM4_I_STAT);
		handled |= pxa9xx_mci_cmd_done(host, stat);
	}
	if (stat & CTO) {
		val = readl(host->base + MM4_CNTL2);
		writel(val | CMDSWRST, host->base + MM4_CNTL2);
		handled |= pxa9xx_mci_cmd_done(host, stat);
	}
	if (stat & ADMA) {
		val = readl(host->base + MMC4_AMDA_ERSTS);
		writel(ADMA, host->base + MM4_I_STAT);
		printk("PXA9XX_MCI: ADMA Error Status %x\n", val);
		handled = 1;
	}

	/* debug message moved here for less latency of irq handling*/
	pr_debug("PXA9XX_MCI: stat %08x\n", stat);

	return IRQ_RETVAL(handled);
}

static void pxa9xx_mci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct pxa9xx_mci_host *host = mmc_priv(mmc);
	unsigned int val;
	unsigned int timeout = 10000;

	WARN_ON(host->mrq != NULL);

	mod_timer(&host->timer, jiffies + PXA_TIMER_TIMEOUT);
	/* release this constraint when request is finished */
	set_dvfm_constraint();

	do {
		val = readl(host->base + MM4_STATE);
		if (!(val & DCMDINHBT || val & CCMDINHBT))
			break;
		udelay(1);
	} while (timeout--);
	if (val & DCMDINHBT) {
		pr_debug("DCMDINHBT of MM4_STATE: %08x\n", val);
		val = readl(host->base + MM4_CNTL2);
		writel(val | DATSWRST, host->base + MM4_CNTL2);
	}
	if (val & CCMDINHBT) {
		pr_debug("CCMDINHBT of MM4_STATE: %08x\n", val);
		val = readl(host->base + MM4_CNTL2);
		writel(val | CMDSWRST, host->base + MM4_CNTL2);
	}

	host->mrq = mrq;

	if (mrq->data)
#define CONFIG_MMC_ADMA
#ifdef CONFIG_MMC_ADMA
		pxa9xx_mci_setup_adma_data(host, mrq->data);
#else
		pxa9xx_mci_setup_data(host, mrq->data);
#endif

	pxa9xx_mci_start_cmd(host, mrq->cmd);
}

static int pxa9xx_mci_get_ro(struct mmc_host *mmc)
{
	struct pxa9xx_mci_host *host = mmc_priv(mmc);

	if (host->pdata && host->pdata->get_ro)
		return host->pdata->get_ro(mmc_dev(mmc));
	/* Host doesn't support read only detection so assume writeable */
	return 0;
}

static void pxa9xx_mci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	unsigned int val;
	struct pxa9xx_mci_host *host = mmc_priv(mmc);

	set_dvfm_constraint();
	if (ios->clock) {
		unsigned long rate = host->clkrate;
		unsigned int clk = rate / ios->clock;
		unsigned int shift;

		if (host->clkrt == CLKRT_OFF)
			clk_enable(host->clk);

		if (ios->clock == 48000000) {
			host->clkrt = 0x00;
			val = readl(host->base + MM4_CNTL1);
			writel(val | HISPEED, host->base + MM4_CNTL1);
		} else {
			if (!clk)
				clk = 1;

			shift = ffs(clk);
			if (rate / clk > ios->clock)
				shift++;
			host->clkrt = 1 << (shift - 2);
		}
		pxa9xx_mci_stop_clock(host);
		pxa9xx_mci_start_clock(host);
	} else {
		pxa9xx_mci_stop_clock(host);
		if (host->clkrt != CLKRT_OFF) {
			host->clkrt = CLKRT_OFF;
			clk_disable(host->clk);
		}
	}

	if (host->power_mode != ios->power_mode) {
		host->power_mode = ios->power_mode;

		if (host->pdata && host->pdata->setpower)
			host->pdata->setpower(mmc_dev(mmc), ios->vdd);

		if (ios->power_mode == MMC_POWER_ON) {
			val = readl(host->base + MM4_CNTL1);
			writel(val | BUSPWR, host->base + MM4_CNTL1);
		}
	}

	if (ios->bus_width == MMC_BUS_WIDTH_4) {
		val = readl(host->base + MM4_CNTL1);
		writel(val | _4BITMD, host->base + MM4_CNTL1);
	} else {
		val = readl(host->base + MM4_CNTL1);
		writel(val & ~_4BITMD, host->base + MM4_CNTL1);
	}

	unset_dvfm_constraint();
	pr_debug("PXA9XX_MCI: clkrt = %08x\n", host->clkrt);
}

static void pxa9xx_mci_enable_sdio_irq(struct mmc_host *host, int enable)
{
	struct pxa9xx_mci_host *pxa_host = mmc_priv(host);

	if (enable)
		pxa9xx_mci_enable_irq(pxa_host, CDINT);
	else
		pxa9xx_mci_disable_irq(pxa_host, CDINT);
}

static const struct mmc_host_ops pxa9xx_mci_ops = {
	.request		= pxa9xx_mci_request,
	.get_ro			= pxa9xx_mci_get_ro,
	.set_ios		= pxa9xx_mci_set_ios,
	.enable_sdio_irq	= pxa9xx_mci_enable_sdio_irq,
};

static irqreturn_t pxa9xx_mci_detect_irq(int irq, void *devid)
{
	struct pxa9xx_mci_host *host = mmc_priv(devid);

	mmc_detect_change(devid, host->pdata->detect_delay);
	return IRQ_HANDLED;
}

static void mmc_timer_handler(unsigned long data)
{
	struct mmc_host *mmc = (struct mmc_host *)data;

	if (mmc->card && !mmc_card_sdio(mmc->card)) {
		pxa9xx_mci_disable_clock(mmc);
	}
}

static int pxa9xx_mci_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct pxa9xx_mci_host *host = NULL;
	struct resource *r;
	int ret, irq, pages;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!r || irq < 0)
		return -ENXIO;

	r = request_mem_region(r->start, SZ_512K, DRIVER_NAME);
	if (!r)
		return -EBUSY;

	mmc = mmc_alloc_host(sizeof(struct pxa9xx_mci_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}

	mmc->ops = &pxa9xx_mci_ops;

	/*
	 * We can do SG-DMA, but we don't because we never know how much
	 * data we successfully wrote to the card.
	 */
	mmc->max_phys_segs = NR_SG;
	mmc->max_hw_segs = NR_SG;

	/*
	 * Our hardware DMA can handle a maximum of one page per SG entry.
	 */
	mmc->max_seg_size = PAGE_SIZE;

	mmc->max_blk_size = 2048;

	/*
	 * Block count register is 16 bits.
	 */
	mmc->max_blk_count = 65535;

	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->pdata = pdev->dev.platform_data;
	host->clkrt = CLKRT_OFF;

	host->clk = clk_get(&pdev->dev, "PXA9XX_MMCCLK");
	if (IS_ERR(host->clk)) {
		ret = PTR_ERR(host->clk);
		host->clk = NULL;
		goto out;
	}

	host->clkrate = 48000000;

	/*
	 * Calculate minimum clock rate, rounding up.
	 */
	mmc->f_min = (host->clkrate + 255) / 256;
	mmc->f_max = host->clkrate;

	mmc->ocr_avail = host->pdata ?
			 host->pdata->ocr_mask :
			 MMC_VDD_32_33|MMC_VDD_33_34;

	mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ |
			MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED;

	host->sg_cpu = dma_alloc_coherent(&pdev->dev, PAGE_SIZE, &host->sg_dma, GFP_KERNEL);
	if (!host->sg_cpu) {
		ret = -ENOMEM;
		goto out;
	}

	spin_lock_init(&host->lock);
	host->res = r;
	host->irq = irq;
	host->i_stat_sig = 0;

	host->base = ioremap(r->start, SZ_512K);
	if (!host->base) {
		ret = -ENOMEM;
		goto out;
	}

	/*
	 * Calculate dma_bufsz
	 */
	pages = mmc->max_seg_size/4096;
	host->dma_bufsz = ffs(pages);
	if (mmc->max_seg_size / pages == 4096)
		host->dma_bufsz--;

	/*
	 * Ensure that the host controller is shut down, and setup
	 * with our defaults.
	 */
	init_timer(&host->timer);
	host->timer.function = mmc_timer_handler;
	host->timer.data = (unsigned long)mmc;

	set_dvfm_constraint();

	pxa9xx_mci_stop_clock(host);
	pxa9xx_mci_disable_irq(host, 0xffffffff);

	ret = request_irq(host->irq, pxa9xx_mci_irq, 0, DRIVER_NAME, host);
	if (ret)
		goto out;

	platform_set_drvdata(pdev, mmc);

	if (host->pdata && host->pdata->init)
		host->pdata->init(&pdev->dev, pxa9xx_mci_detect_irq, mmc);

	mmc_add_host(mmc);
	unset_dvfm_constraint();
	return 0;

 out:
	if (host) {
		if (host->base)
			iounmap(host->base);
		if (host->sg_cpu)
			dma_free_coherent(&pdev->dev, PAGE_SIZE, host->sg_cpu, host->sg_dma);
		if (host->clk)
			clk_put(host->clk);
	}
	if (mmc)
		mmc_free_host(mmc);
	release_mem_region(r->start, SZ_512K);
	unset_dvfm_constraint();
	return ret;
}

static int pxa9xx_mci_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct pxa9xx_mci_host *host;

	platform_set_drvdata(pdev, NULL);

	if (mmc) {
		set_dvfm_constraint();
		host = mmc_priv(mmc);

		if (host->pdata && host->pdata->exit)
			host->pdata->exit(&pdev->dev, mmc);

		del_timer_sync(&host->timer);
		mmc_remove_host(mmc);

		pxa9xx_mci_stop_clock(host);
		pxa9xx_mci_disable_irq(host, 0xffffffff);

		free_irq(host->irq, host);
		iounmap(host->base);
		dma_free_coherent(&pdev->dev, PAGE_SIZE, host->sg_cpu, host->sg_dma);

		clk_put(host->clk);

		release_mem_region(host->res->start, SZ_512K);

		mmc_free_host(mmc);
		unset_dvfm_constraint();
	}
	return 0;
}

#ifdef CONFIG_PM
static int pxa9xx_mci_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	if (mmc)
		ret = mmc_suspend_host(mmc, state);

	return ret;
}

static int pxa9xx_mci_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	if (mmc)
		ret = mmc_resume_host(mmc);

	return ret;
}
#else
#define pxa9xx_mci_suspend	NULL
#define pxa9xx_mci_resume	NULL
#endif

static struct platform_driver pxa9xx_mci_driver = {
	.probe		= pxa9xx_mci_probe,
	.remove		= pxa9xx_mci_remove,
	.suspend	= pxa9xx_mci_suspend,
	.resume		= pxa9xx_mci_resume,
	.driver		= {
		.name	= DRIVER_NAME,
	},
};

static int __init pxa9xx_mci_init(void)
{
#ifdef CONFIG_PXA3xx_DVFM
	dvfm_register("PXA9XX_MMC", &(dvfm_lock.dev_idx));
#endif
	return platform_driver_register(&pxa9xx_mci_driver);
}

static void __exit pxa9xx_mci_exit(void)
{
	platform_driver_unregister(&pxa9xx_mci_driver);
#ifdef CONFIG_PXA3xx_DVFM
	dvfm_unregister("PXA9XX_MMC", &(dvfm_lock.dev_idx));
#endif
}

module_init(pxa9xx_mci_init);
module_exit(pxa9xx_mci_exit);

MODULE_DESCRIPTION("PXA9XX Multimedia Card Interface Driver");
MODULE_LICENSE("GPL");
