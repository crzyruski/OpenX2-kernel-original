/*
 * linux/sound/soc/pxa/tavorevb.c
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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <asm/io.h>

#include <mach/mfp-pxa9xx.h>
#include <mach/pxa-regs.h>
#include <mach/hardware.h>
#include <mach/tavorevb.h>
#include <mach/audio.h>
#include <mach/regs-ssp.h>
#include <mach/ssp.h>
#include <mach/micco.h>

#include "pxa3xx-pcm.h"
#include "pxa3xx-ssp.h"

#if defined(CONFIG_PXA3xx_DVFM)
#include "mach/dvfm.h"
#endif


#define SOUND_SSP_DITHER_AUDIO_CLOCK

#define PCM_SSP_PORT	3	  /* used as ap pcm port  */
#define I2S_SSP_PORT	2	  /* used as ap hifi port */
#define PCM_CP_PORT	1	  /* used as cp pcm prot  */

#define ARRAY_AND_SIZE(x)       (x), ARRAY_SIZE(x)

static mfp_cfg_t gssp1_mfp_cfg[] = {
	/* GSSP1 */
	GPIO79_GSSP1_CLK,
	GPIO80_GSSP1_FRM,
	GPIO81_GSSP1_TXD,
	GPIO82_GSSP1_RXD,
};

static mfp_cfg_t ssp3_mfp_cfg[] = {
	/* SSP3 */
	GPIO79_SSP3_CLK,
	GPIO80_SSP3_FRM,
	GPIO81_SSP3_TXD,
	GPIO82_SSP3_RXD,
};

void enable_oscc_pout(void);
void disable_oscc_pout(void);

extern struct snd_soc_dai micco_dai[3];
extern struct snd_soc_codec_device soc_codec_dev_micco;

#if defined(CONFIG_PXA3xx_DVFM)

static int dvfm_dev_idx;

static void set_dvfm_constraint(void)
{
	/* Disable Lowpower mode */
	dvfm_disable_op_name("D0CS", dvfm_dev_idx);
	dvfm_disable_op_name("D1", dvfm_dev_idx);
	dvfm_disable_op_name("D2", dvfm_dev_idx);
	if (cpu_is_pxa935())
		dvfm_disable_op_name("CG", dvfm_dev_idx);
}

static void unset_dvfm_constraint(void)
{
	/* Enable Lowpower mode */
	dvfm_enable_op_name("D0CS", dvfm_dev_idx);
	dvfm_enable_op_name("D1", dvfm_dev_idx);
	dvfm_enable_op_name("D2", dvfm_dev_idx);
	if (cpu_is_pxa935())
		dvfm_enable_op_name("CG", dvfm_dev_idx);
}
#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
#endif

static int tavorevb_micco_init(struct snd_soc_codec *codec);

static int tavorevb_micco_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	unsigned long sscr0;

	set_dvfm_constraint();

	cpu_dai->playback.channels_min = 2;
	cpu_dai->playback.channels_max = 2;
	cpu_dai->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->playback.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_44100 |\
		SNDRV_PCM_RATE_16000;

	enable_oscc_pout();

	/* enable packing mode for cpu_dai->hw_params judge*/
	sscr0 = SSCR0_FPCKE;
	__raw_writel(sscr0, ssp->mmio_base + SSCR0);

	return 0;
}

static int tavorevb_micco_hifi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long rate = runtime->rate;
	unsigned long sscr0, sscr1, sspsp, sstsa;
	unsigned long ssacd, ssacdd, ssrsa;

	/* Because the internal 13M clock will be 10M in D0CS,
	 * we route SSP_CLK to GPIO126(EXT_CLK) and let SSP select
	 * NETWORK CLK as CLK source.
	 * This workaround need an ECO on Littleton mainboard.
	 */

	sscr0 = 0xE1C0003F;
	sscr1 = 0x00701DC0;
	sspsp = 0x40200004;
	sstsa = 0x00000003;
	ssrsa = 0x00000003;

#ifdef SOUND_SSP_DITHER_AUDIO_CLOCK
	ssacd = 0x60;
	ssacdd = 0x00000040;

	switch (rate) {
	case 48000:
		ssacdd |= (0x659 << 16);
		break;
	case 44100:
		ssacdd |= (0x6E9 << 16);
		break;
	case 16000:
		ssacdd |= (0x130B << 16);
		break;
	default:
		return -EINVAL;
	}
#else
	ssacd = 0x0;
	ssacdd = 0x0;

	switch (rate) {
	case 48000:
		ssacd |= (0 << 0) /* ACDS */
			| (0 << 3) /* SCDB */
			| (2 << 4) /* ACPS */
			| (0 << 7); /* SCDX8 */
		break;
	case 44100:
		ssacd |= (0 << 0) /* ACDS */
			| (0 << 3) /* SCDB */
			| (1 << 4) /* ACPS */
			| (0 << 7); /* SCDX8 */
		break;
	case 22050:
		ssacd |= (0 << 0) /* ACDS */
			| (0 << 3) /* SCDB */
			| (0 << 4) /* ACPS */
			| (0 << 7); /* SCDX8 */
		break;
	case 16000:
		ssacd |= (3 << 0) /* ACDS */
			| (0 << 3) /* SCDB */
			| (4 << 4) /* ACPS */
			| (0 << 7); /* SCDX8 */
		break;
	case 11025:
		ssacd |= (1 << 0) /* ACDS */
			| (0 << 3) /* SCDB */
			| (0 << 4) /* ACPS */
			| (0 << 7); /* SCDX8 */
		break;
	case 8000:
		ssacd |= (4 << 0) /* ACDS */
			| (0 << 3) /* SCDB */
			| (4 << 4) /* ACPS */
			| (0 << 7); /* SCDX8 */
		break;
	default:
		return -EINVAL;
	}
#endif
	__raw_writel(sscr0, ssp->mmio_base + SSCR0);
	__raw_writel(sscr1, ssp->mmio_base + SSCR1);
	__raw_writel(sspsp, ssp->mmio_base + SSPSP);
	__raw_writel(sstsa, ssp->mmio_base + SSTSA);
	__raw_writel(ssrsa, ssp->mmio_base + SSRSA);
	__raw_writel(ssacd, ssp->mmio_base + SSACD);
	__raw_writel(ssacdd, ssp->mmio_base + SSACDD);

	return 0;
}

static void tavorevb_micco_hifi_shutdown(struct snd_pcm_substream *substream)
{
	disable_oscc_pout();
	unset_dvfm_constraint();
}

static int tavorevb_micco_voice_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;

	set_dvfm_constraint();

	cpu_dai->playback.channels_min = 1;
	cpu_dai->playback.channels_max = 1;
	cpu_dai->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->playback.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | \
		SNDRV_PCM_RATE_32000;

	pxa3xx_mfp_config(ARRAY_AND_SIZE(ssp3_mfp_cfg));

	enable_oscc_pout();

	return 0;
}

static int tavorevb_micco_voice_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long rate = runtime->rate;
	unsigned long sscr0, sscr1, sspsp;

	cpu_dai->dai_ops.set_sysclk(cpu_dai, PXA3XX_SSP_CLK_PLL, 13000000, 0);

	sscr0 = 0x00C0003F;
	sscr1 = 0x00701DC0;
	sspsp = 0x00800085;

	switch (rate) {
	case 8000:
		sscr0 |= (89 << 8);
		break;
	case 16000:
		sscr0 |= (44 << 8);
		break;
	case 32000:
		sscr0 |= (22 << 8);
		break;
	default:
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		sscr1 &= ~0x00800000;
	} else {
		sscr1 |= 0x00800000;
	}

	__raw_writel(sscr0, ssp->mmio_base + SSCR0);
	__raw_writel(sscr1, ssp->mmio_base + SSCR1);
	__raw_writel(sspsp, ssp->mmio_base + SSPSP);

	return 0;
}

static void tavorevb_micco_voice_shutdown(struct snd_pcm_substream *substream)
{
	pxa3xx_mfp_config(ARRAY_AND_SIZE(gssp1_mfp_cfg));

	disable_oscc_pout();
	unset_dvfm_constraint();
}

static int tavorevb_micco_cp_voice_startup(struct snd_pcm_substream *substream)
{
	pxa3xx_mfp_config(ARRAY_AND_SIZE(gssp1_mfp_cfg));
	enable_oscc_pout();
	return 0;
}
static int tavorevb_micco_cp_voice_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}
static void tavorevb_micco_cp_voice_shutdown(struct snd_pcm_substream *substream)
{
	disable_oscc_pout();
	pxa3xx_mfp_config(ARRAY_AND_SIZE(ssp3_mfp_cfg));
}


/* machine stream operations */
static struct snd_soc_ops tavorevb_machine_ops[] = {
{
	.startup = tavorevb_micco_hifi_startup,
	.prepare = tavorevb_micco_hifi_prepare,
	.shutdown = tavorevb_micco_hifi_shutdown,
},
{
	.startup = tavorevb_micco_voice_startup,
	.prepare = tavorevb_micco_voice_prepare,
	.shutdown = tavorevb_micco_voice_shutdown,
},
{
	.startup = tavorevb_micco_cp_voice_startup,
	.prepare = tavorevb_micco_cp_voice_prepare,
	.shutdown = tavorevb_micco_cp_voice_shutdown,
},
};

/* tavorevb digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link tavorevb_dai[] = {
{
	.name = "I2S",
	.stream_name = "I2S HiFi",
	.cpu_dai = &pxa3xx_ssp_dai[I2S_SSP_PORT-1],
	.codec_dai = &micco_dai[0],
	.ops = &tavorevb_machine_ops[0],
	.init = tavorevb_micco_init,
},
{
	.name = "PCM",
	.stream_name = "PCM Voice",
	.cpu_dai = &pxa3xx_ssp_dai[PCM_SSP_PORT-1],
	.codec_dai = &micco_dai[1],
	.ops = &tavorevb_machine_ops[1],
	.init = tavorevb_micco_init,
},
{
	.name = "CP PCM",
	.stream_name = "CP PCM Voice",
	.cpu_dai = &pxa3xx_ssp_dai[4],
	.codec_dai = &micco_dai[2],
	.ops = &tavorevb_machine_ops[2],
	.init = tavorevb_micco_init,
},
};

/* tavorevb audio machine driver */
static struct snd_soc_machine snd_soc_machine_tavorevb = {
	.name = "tavorevb",
	.dai_link = tavorevb_dai,
	.num_links = ARRAY_SIZE(tavorevb_dai),
};

/* tavorevb audio subsystem */
static struct snd_soc_device tavorevb_snd_devdata = {
	.machine = &snd_soc_machine_tavorevb,
	.platform = &pxa3xx_soc_platform,
	.codec_dev = &soc_codec_dev_micco,
};

static struct platform_device *tavorevb_snd_device;

/*
 * Logic for a Micco as connected on a tavorevb Device
 */
static int tavorevb_micco_init(struct snd_soc_codec *codec)
{
	return 0;
}

static int __init tavorevb_init(void)
{
	int ret;

	if (!machine_is_tavorevb())
		return -ENODEV;

	tavorevb_snd_device = platform_device_alloc("soc-audio", -1);
	if (!tavorevb_snd_device)
		return -ENOMEM;

	platform_set_drvdata(tavorevb_snd_device, &tavorevb_snd_devdata);
	tavorevb_snd_devdata.dev = &tavorevb_snd_device->dev;
	ret = platform_device_add(tavorevb_snd_device);

	if (ret)
		platform_device_put(tavorevb_snd_device);

#ifdef CONFIG_PXA3xx_DVFM
	dvfm_register("Sound", &dvfm_dev_idx);
#endif

	return ret;
}

static void __exit tavorevb_exit(void)
{
	platform_device_unregister(tavorevb_snd_device);

#ifdef CONFIG_PXA3xx_DVFM
	dvfm_unregister("Sound", &dvfm_dev_idx);
#endif

}

module_init(tavorevb_init);
module_exit(tavorevb_exit);

/* Module information */
MODULE_AUTHOR("bshen9@marvell.com");
MODULE_DESCRIPTION("ALSA SoC MICCO Tavorevb");
MODULE_LICENSE("GPL");

