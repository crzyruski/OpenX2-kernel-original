/*
 * linux/sound/soc/pxa/pxa3xx-ac97.c
 * Base on linux/sound/soc/pxa/pxa2xx-ac97.c
 *
 * Copyright (C) 2007 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <mach/irqs.h>
#include <linux/mutex.h>
#include <mach/hardware.h>
#include <mach/pxa-regs.h>
#include <mach/audio.h>
#include <linux/clk.h>

#include "pxa3xx-pcm.h"
#include "pxa3xx-ac97.h"
#include "pxa3xx-ac97codec-pm.h"

static DEFINE_MUTEX(car_mutex);
static DECLARE_WAIT_QUEUE_HEAD(gsr_wq);
static volatile long gsr_bits;

/*
 * Beware PXA27x bugs:
 *
 *   o Slot 12 read from modem space will hang controller.
 *   o CDONE, SDONE interrupt fails after any slot 12 IO.
 *
 * We therefore have an hybrid approach for waiting on SDONE (interrupt or
 * 1 jiffy timeout if interrupt never comes).
 */

static unsigned short pxa3xx_ac97_read(struct snd_ac97 *ac97,
	unsigned short reg)
{
	unsigned short val = -1;
	volatile u32 *reg_addr;
	int timeout = 2000;
	int retry = 10;

	if (CAR&CAR_CAIP) {
		if (in_interrupt() || in_irq() || in_softirq()) {
			udelay(10);
		} else {
			schedule_timeout_interruptible(msecs_to_jiffies(20));
		}
	}

__retry:
	if (0 == retry) {
		printk(KERN_ERR "AC97 read from 0x%x error!!\n", reg);
		goto out;
	}
	retry--;

	/* set up primary or secondary codec/modem space */
	reg_addr = ac97->num ? &SAC_REG_BASE : &PAC_REG_BASE;
	reg_addr += (reg >> 1);

	/* start read access across the ac97 link */
	GSR = GSR_SDONE|GSR_RDCS;
	val = *reg_addr;

	while (!(GSR&GSR_SDONE) && (--timeout > 0))
		udelay(1);

	if ((GSR&GSR_SDONE) && !(GSR&GSR_RDCS)) {
		if (GSR & GSR_RDCS) {
			val = 0xffff;
			goto __retry;
		}
		GSR = GSR_SDONE|GSR_RDCS;
		val = *reg_addr;
		timeout = 2000;
		while (!(GSR&GSR_SDONE) && (--timeout > 0))
			udelay(1);
	} else {
		val = 0xffff;
		goto __retry;
	}

out:
	CAR = 0x0;
	return val;
}

static void pxa3xx_ac97_write(struct snd_ac97 *ac97, unsigned short reg,
	unsigned short val)
{
	volatile u32 *reg_addr;
	int timeout = 2000;
	int retry = 10;


	if (CAR&CAR_CAIP) {
		if (in_interrupt() || in_irq() || in_softirq()) {
			udelay(10);
		} else {
			schedule_timeout_interruptible(msecs_to_jiffies(20));
		}
	}
__retry:
	if (0 == retry) {
		printk(KERN_ERR "AC97 write to 0x%x error!!\n", reg);
		goto out;
	}
	retry--;

	/* set up primary or secondary codec/modem space */
	if (reg == AC97_GPIO_STATUS)
		reg_addr = ac97->num ? &SMC_REG_BASE : &PMC_REG_BASE;
	else
		reg_addr = ac97->num ? &SAC_REG_BASE : &PAC_REG_BASE;

	reg_addr += (reg >> 1);

	if (reg != AC97_GPIO_STATUS) {
		GSR = GSR_CDONE;
		*reg_addr = (unsigned long)val;
		while (!(GSR&GSR_CDONE) && (--timeout > 0))
			udelay(1);
		if (timeout <= 0)
			goto __retry;
	} else {
		*reg_addr = (unsigned long)val;
	}
out:
	CAR = 0x0;
}

static void pxa3xx_ac97_warm_reset(struct snd_ac97 *ac97)
{
	GCR |= GCR_WARM_RST;
	mdelay(100);
}

static void pxa3xx_ac97_cold_reset(struct snd_ac97 *ac97)
{
	int timeout = 100;

	GCR = 0;
	GCR |= (1 << 31);
	udelay(100);
	GCR &= ~(1 << 31);
	udelay(100);
	GCR |= GCR_WARM_RST|GCR_COLD_RST;
	udelay(100);
	while (!(GSR&GSR_PCR) && --timeout)
		mdelay(10);
}

struct snd_ac97_bus_ops soc_ac97_ops = {
	.read	= pxa3xx_ac97_read,
	.write	= pxa3xx_ac97_write,
	.warm_reset	= pxa3xx_ac97_warm_reset,
	.reset	= pxa3xx_ac97_cold_reset,
};
EXPORT_SYMBOL_GPL(soc_ac97_ops);

static struct pxa3xx_pcm_dma_params pxa3xx_ac97_pcm_stereo_out = {
	.name			= "AC97 PCM Stereo out",
	.dev_addr		= __PREG(PCDR),
	.drcmr			= &DRCMR(12),
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST32 | DCMD_WIDTH4,
};

static struct pxa3xx_pcm_dma_params pxa3xx_ac97_pcm_stereo_in = {
	.name			= "AC97 PCM Stereo in",
	.dev_addr		= __PREG(PCDR),
	.drcmr			= &DRCMR(11),
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST32 | DCMD_WIDTH4,
};

static struct pxa3xx_pcm_dma_params pxa3xx_ac97_pcm_aux_mono_out = {
	.name			= "AC97 Aux PCM (Slot 5) Mono out",
	.dev_addr		= __PREG(MODR),
	.drcmr			= &DRCMR(10),
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST16 | DCMD_WIDTH2,
};

static struct pxa3xx_pcm_dma_params pxa3xx_ac97_pcm_aux_mono_in = {
	.name			= "AC97 Aux PCM (Slot 5) Mono in",
	.dev_addr		= __PREG(MODR),
	.drcmr			= &DRCMR(9),
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST16 | DCMD_WIDTH2,
};

static struct pxa3xx_pcm_dma_params pxa3xx_ac97_pcm_mic_mono_in = {
	.name			= "AC97 Mic PCM (Slot 6) Mono in",
	.dev_addr		= __PREG(MCDR),
	.drcmr			= &DRCMR(8),
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST16 | DCMD_WIDTH2,
};

#ifdef CONFIG_PM
static int pxa3xx_ac97_suspend(struct platform_device *pdev,
	struct snd_soc_dai *dai)
{
	GCR |= GCR_ACLINK_OFF;
	clk_disable(ac97_clk);
	return 0;
}

static int pxa3xx_ac97_resume(struct platform_device *pdev,
	struct snd_soc_dai *dai)
{
	clk_enable(ac97_clk);
	return 0;
}

#else
#define pxa3xx_ac97_suspend	NULL
#define pxa3xx_ac97_resume	NULL
#endif

static int pxa3xx_ac97_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;

	clk_enable(ac97_clk);
#ifdef CONFIG_AC97_EXTCLK
	enable_oscc_pout();
#endif
	pxa3xx_ac97_cold_reset(NULL);

	return 0;
}

static void pxa3xx_ac97_remove(struct platform_device *pdev)
{
	GCR |= GCR_ACLINK_OFF;
	clk_disable(ac97_clk);
#ifdef CONFIG_AC97_EXTCLK
	disable_oscc_pout();
#endif
}

static int pxa3xx_ac97_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		cpu_dai->dma_data = &pxa3xx_ac97_pcm_stereo_out;
	else
		cpu_dai->dma_data = &pxa3xx_ac97_pcm_stereo_in;

	return 0;
}

static int pxa3xx_ac97_hw_aux_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		cpu_dai->dma_data = &pxa3xx_ac97_pcm_aux_mono_out;
	else
		cpu_dai->dma_data = &pxa3xx_ac97_pcm_aux_mono_in;

	return 0;
}

static int pxa3xx_ac97_hw_mic_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return -ENODEV;
	else
		cpu_dai->dma_data = &pxa3xx_ac97_pcm_mic_mono_in;

	return 0;
}

#define PXA3XX_AC97_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
		SNDRV_PCM_RATE_48000)

/*
 * There is only 1 physical AC97 interface for pxa3xx, but it
 * has extra fifo's that can be used for aux DACs and ADCs.
 */
struct snd_soc_dai pxa_ac97_dai[] = {
{
	.name = "pxa3xx-ac97",
	.id = 0,
	.type = SND_SOC_DAI_AC97,
	.probe = pxa3xx_ac97_probe,
	.remove = pxa3xx_ac97_remove,
	.suspend = pxa3xx_ac97_suspend,
	.resume = pxa3xx_ac97_resume,
	.playback = {
		.stream_name = "AC97 Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = PXA3XX_AC97_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.stream_name = "AC97 Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = PXA3XX_AC97_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = {
		.hw_params = pxa3xx_ac97_hw_params,},
},
{
	.name = "pxa3xx-ac97-aux",
	.id = 1,
	.type = SND_SOC_DAI_AC97,
	.playback = {
		.stream_name = "AC97 Aux Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = PXA3XX_AC97_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.stream_name = "AC97 Aux Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = PXA3XX_AC97_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = {
		.hw_params = pxa3xx_ac97_hw_aux_params,},
},
{
	.name = "pxa3xx-ac97-mic",
	.id = 2,
	.type = SND_SOC_DAI_AC97,
	.capture = {
		.stream_name = "AC97 Mic Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = PXA3XX_AC97_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = {
		.hw_params = pxa3xx_ac97_hw_mic_params,},
},
};
EXPORT_SYMBOL_GPL(pxa_ac97_dai);

MODULE_AUTHOR("bin.yang@marvell.com");
MODULE_DESCRIPTION("AC97 driver for the PXA3xx chip");
MODULE_LICENSE("GPL");
