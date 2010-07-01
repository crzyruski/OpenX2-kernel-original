/*
 * linux/sound/soc/pxa/zylonite.c
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
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <mach/io.h>
#include <mach/pxa-regs.h>
#include <mach/hardware.h>
#include <mach/zylonite.h>
#include <mach/audio.h>
#include <mach/regs-ssp.h>
#include <mach/ssp.h>

#include "pxa3xx-ac97.h"
#include "pxa3xx-pcm.h"
#include "pxa3xx-ssp.h"
#include "../codecs/wm9713.h"
#include "pxa3xx-ac97codec-pm.h"

#if defined(CONFIG_PXA3xx_DVFM)
#include "mach/dvfm.h"
#endif

#define PCM_SSP_PORT	3
#define WM9713_EASCR_VRA   (0x1 << 0)    /* Enable Variable Rate Audio mode */

#if defined(CONFIG_PXA3xx_DVFM)

static int dvfm_dev_idx;

static void set_dvfm_constraint(void)
{
	/* Disable Lowpower mode */
	dvfm_disable_op_name("D1", dvfm_dev_idx);
	dvfm_disable_op_name("D2", dvfm_dev_idx);
}

static void unset_dvfm_constraint(void)
{
	/* Enable Lowpower mode */
	dvfm_enable_op_name("D1", dvfm_dev_idx);
	dvfm_enable_op_name("D2", dvfm_dev_idx);
}
#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
#endif

static int zylonite_wm9713_init(struct snd_soc_codec *codec);

static int zylonite_wm9713_hifi_startup(struct snd_pcm_substream *substream)
{
	set_dvfm_constraint();
	return 0;
}

static int zylonite_wm9713_hifi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	int reg;

	codec->write(codec, AC97_POWERDOWN, 0);
	mdelay(1);
	codec_dai->dai_ops.set_pll(codec_dai, 0, 13000000, 24576000);
	schedule_timeout_interruptible(msecs_to_jiffies(10));
	codec->write(codec, AC97_HANDSET_RATE, 0x0000);
	schedule_timeout_interruptible(msecs_to_jiffies(10));

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		reg = AC97_PCM_FRONT_DAC_RATE;
	else
		reg = AC97_PCM_LR_ADC_RATE;
	codec->write(codec, AC97_EXTENDED_STATUS, WM9713_EASCR_VRA);
	codec->write(codec, reg, runtime->rate);

	return 0;
}
static void zylonite_wm9713_hifi_shutdown(struct snd_pcm_substream *substream)
{
	unset_dvfm_constraint();
}



static int zylonite_wm9713_voice_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;

	set_dvfm_constraint();

	cpu_dai->playback.channels_min = 1;
	cpu_dai->playback.channels_max = 1;
	cpu_dai->playback.formats = SNDRV_PCM_FMTBIT_S8
				| SNDRV_PCM_FMTBIT_U8
				| SNDRV_PCM_FMTBIT_S16_LE;

	cpu_dai->playback.rates = SNDRV_PCM_RATE_8000
				| SNDRV_PCM_RATE_16000
				| SNDRV_PCM_RATE_32000
				| SNDRV_PCM_RATE_48000;

	cpu_dai->capture.channels_min = 1;
	cpu_dai->capture.channels_max = 1;
	cpu_dai->capture.formats = SNDRV_PCM_FMTBIT_S8
				| SNDRV_PCM_FMTBIT_U8
				| SNDRV_PCM_FMTBIT_S16_LE;

	cpu_dai->capture.rates = SNDRV_PCM_RATE_8000
				| SNDRV_PCM_RATE_16000
				| SNDRV_PCM_RATE_32000
				| SNDRV_PCM_RATE_48000;



	return 0;
}

static int zylonite_wm9713_voice_prepare(struct snd_pcm_substream *substream)
{
#define WM9713_DR_8000     0x1F40  /*  8000 samples/sec */
#define WM9713_DR_11025    0x2B11  /* 11025 samples/sec */
#define WM9713_DR_12000    0x2EE0  /* 12000 samples/sec */
#define WM9713_DR_16000    0x3E80  /* 16000 samples/sec */
#define WM9713_DR_22050    0x5622  /* 22050 samples/sec */
#define WM9713_DR_24000    0x5DC0  /* 24000 samples/sec */
#define WM9713_DR_32000    0x7D00  /* 32000 samples/sec */
#define WM9713_DR_44100    0xAC44  /* 44100 samples/sec */
#define WM9713_DR_48000    0xBB80  /* 48000 samples/sec */
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct ssp_device *ssp = cpu_dai->private_data;
	unsigned long rate = runtime->rate;
	unsigned short value;
	int div;
	u32 sscr0, sscr1, sspsp, sstsa, ssacd, ssacdd, ssrsa;
	unsigned short reg = 0x8000;

	div = rate/8000;
	div = 12/div;
	codec->write(codec, AC97_POWERDOWN, 0);
	mdelay(1);
	codec_dai->dai_ops.set_pll(codec_dai, 0, 13000000, 24576000/div);
	schedule_timeout_interruptible(msecs_to_jiffies(10));
	codec->write(codec, AC97_HANDSET_RATE, 0x0000);
	schedule_timeout_interruptible(msecs_to_jiffies(10));

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK) {
		switch (rate) {
		case 8000:
			value = WM9713_DR_8000;
			break;
		case 11025:
			value = WM9713_DR_11025;
			break;
		case 12000:
			value = WM9713_DR_12000;
			break;
		case 16000:
			value = WM9713_DR_16000;
			break;
		case 22050:
			value = WM9713_DR_22050;
			break;
		case 24000:
			value = WM9713_DR_24000;
			break;
		case 32000:
			value = WM9713_DR_32000;
			break;
		case 44100:
			value = WM9713_DR_44100;
			break;
		case 48000:
			value = WM9713_DR_48000;
			break;
		default:
			return -EINVAL;
		}
		/*enable VRA mode*/
		codec->write(codec, AC97_EXTENDED_STATUS, WM9713_EASCR_VRA);
		codec->write(codec, AC97_PCM_LR_ADC_RATE, value);
	}
	codec->write(codec, AC97_EXTENDED_MID,
		codec->read(codec, AC97_EXTENDED_MID)&(~(0xf<<2)));
	codec->write(codec, AC97_EXTENDED_MSTATUS,
		codec->read(codec, AC97_EXTENDED_MSTATUS)&(~(0x3<<9)));

	reg = 0x8000;
	/*master mode*/
	reg |= 0x4000;
	/*normal polarity*/
	/*output left ADC only*/
	reg |= 0x20;
	/*16bit DSP mode*/
	reg |= 0x3;
	reg |= (0x1 << 9);
	reg |= (0x0 << 8);
	codec->write(codec, AC97_CENTER_LFE_MASTER, reg);

	reg = codec->read(codec, AC97_GPIO_CFG);
	reg &= ~(0x1<<1);
	reg &= ~(0x1<<3);
	reg |= (0x1<<4);
	reg &= ~(0x1<<5);
	codec->write(codec, AC97_GPIO_CFG, reg);
	schedule_timeout_interruptible(msecs_to_jiffies(10));

	sscr0 = 0x1000b0;
	sscr1 = 0x3703403;
	sspsp = 0x7;
	sstsa = 0x0;
	ssrsa = 0x0;
	ssacd = 0x0;
	ssacdd = 0x0;

	__raw_writel(sscr0, ssp->mmio_base + SSCR0);
	__raw_writel(sscr1, ssp->mmio_base + SSCR1);
	__raw_writel(sspsp, ssp->mmio_base + SSPSP);
	__raw_writel(sstsa, ssp->mmio_base + SSTSA);
	__raw_writel(ssrsa, ssp->mmio_base + SSRSA);
	__raw_writel(ssacd, ssp->mmio_base + SSACD);
	__raw_writel(ssacdd, ssp->mmio_base + SSACDD);

	schedule_timeout_interruptible(msecs_to_jiffies(100));

	return 0;
}

static void zylonite_wm9713_voice_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;

	codec->write(codec, AC97_EXTENDED_MID,
		codec->read(codec, AC97_EXTENDED_MID)|((0xf<<2)));
	codec->write(codec, AC97_EXTENDED_MSTATUS,
		codec->read(codec, AC97_EXTENDED_MSTATUS)|(0x3<<9));

	unset_dvfm_constraint();

}

static int zylonite_ac97_suspend_post(struct platform_device *pdev,
		pm_message_t state)
{
	set_codec_sub_state(codec_client, CODEC_SUB_POWER_OFF);
	return 0;
}
static int zylonite_ac97_resume_post(struct platform_device *pdev)
{
	set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);
	return 0;
}

/* machine stream operations */
static struct snd_soc_ops zylonite_machine_ops[] = {
{
	.startup = zylonite_wm9713_hifi_startup,
	.prepare = zylonite_wm9713_hifi_prepare,
	.shutdown = zylonite_wm9713_hifi_shutdown,
},
{
	.startup = zylonite_wm9713_voice_startup,
	.prepare = zylonite_wm9713_voice_prepare,
	.shutdown = zylonite_wm9713_voice_shutdown,
},
};

/* zylonite digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link zylonite_dai[] = {
{
	.name = "AC97",
	.stream_name = "AC97 HiFi",
	.cpu_dai = &pxa_ac97_dai[PXA3XX_DAI_AC97_HIFI],
	.codec_dai = &wm9713_dai[WM9713_DAI_AC97_HIFI],
	.ops = &zylonite_machine_ops[0],
	.init = zylonite_wm9713_init,
},
{
	.name = "PCM",
	.stream_name = "PCM Voice",
	.cpu_dai = &pxa3xx_ssp_dai[PCM_SSP_PORT-1],
	.codec_dai = &wm9713_dai[WM9713_DAI_PCM_VOICE],
	.ops = &zylonite_machine_ops[1],
},
};

/* zylonite audio machine driver */
static struct snd_soc_machine snd_soc_machine_zylonite = {
	.name = "zylonite",
	.dai_link = zylonite_dai,
	.num_links = ARRAY_SIZE(zylonite_dai),
	.suspend_post = zylonite_ac97_suspend_post,
	.resume_post = zylonite_ac97_resume_post,
};

/* zylonite audio subsystem */
static struct snd_soc_device zylonite_snd_devdata = {
	.machine = &snd_soc_machine_zylonite,
	.platform = &pxa3xx_soc_platform,
	.codec_dev = &soc_codec_dev_wm9713,
};

static struct platform_device *zylonite_snd_device;
static int bb_enable;

static int bb_control_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int bb_control_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = bb_enable;
	return 0;
}

/* pay attention: application should lock the route set period. */
static int bb_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	unsigned short reg;
	struct snd_soc_codec *codec = wm9713_dai[WM9713_DAI_PCM_VOICE].codec;

	if (bb_enable == 1 && ucontrol->value.integer.value[0] == 0)
		printk(KERN_INFO "Disable BB!!\n");

	if (bb_enable == 0 && ucontrol->value.integer.value[0] == 1) {
		printk(KERN_INFO "Enable BB!!\n");

		snd_soc_dapm_stream_event(codec, "Voice Playback",
			SND_SOC_DAPM_STREAM_START);
		snd_soc_dapm_stream_event(codec, "Voice Capture",
			SND_SOC_DAPM_STREAM_START);
		/*enable VRA mode*/
		codec->write(codec, AC97_EXTENDED_STATUS, 0x1);
		codec->write(codec, AC97_PCM_LR_ADC_RATE, 0x1F40);
		codec->write(codec, AC97_EXTENDED_MID,
			codec->read(codec, AC97_EXTENDED_MID)&0xffc3);
		codec->write(codec, AC97_EXTENDED_MSTATUS,
			codec->read(codec, AC97_EXTENDED_MSTATUS)&0xf9f5);
		codec->write(codec, AC97_POWERDOWN, 0);
		codec->write(codec, AC97_HANDSET_RATE, 0x0f80);
		schedule_timeout_interruptible(msecs_to_jiffies(10));
		codec->write(codec, AC97_CENTER_LFE_MASTER, 0xa8b3);
		reg = codec->read(codec, AC97_GPIO_CFG);
		reg |= (0x1<<1);
		reg |= (0x1<<3);
		reg |= (0x1<<4);
		reg &= ~(0x1<<5);
		codec->write(codec, AC97_GPIO_CFG, reg);
		schedule_timeout_interruptible(msecs_to_jiffies(10));
	}
	bb_enable = ucontrol->value.integer.value[0];
	return 0;
}

static struct snd_kcontrol_new bb_kcontrol = {
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_CARD,
		.name = "BB",
		.private_value = 0,
		.info = bb_control_info,
		.get = bb_control_get,
		.put = bb_control_put,
};

static int zylonite_wm9713_init(struct snd_soc_codec *codec)
{
	snd_ctl_add(codec->card,
		snd_ctl_new1(&bb_kcontrol, NULL));
	return 0;
}

static int __init zylonite_init(void)
{
	int ret;

	if (!machine_is_zylonite())
		return -ENODEV;
	register_codec(&codec_client);
	set_codec_sub_state(codec_client, CODEC_SUB_POWER_ON);

	pxa_ac97_dai[0].probe = NULL;
	pxa_ac97_dai[0].remove = NULL;
	pxa_ac97_dai[0].suspend = NULL;
	pxa_ac97_dai[0].resume = NULL;
	soc_codec_dev_wm9713.suspend = NULL;
	soc_codec_dev_wm9713.resume = NULL;
	soc_codec_dev_wm9713.remove = NULL;

	zylonite_snd_device = platform_device_alloc("soc-audio", -1);
	if (!zylonite_snd_device)
		return -ENOMEM;

	platform_set_drvdata(zylonite_snd_device, &zylonite_snd_devdata);
	zylonite_snd_devdata.dev = &zylonite_snd_device->dev;
	ret = platform_device_add(zylonite_snd_device);

	if (ret)
		platform_device_put(zylonite_snd_device);

#ifdef CONFIG_PXA3xx_DVFM
	dvfm_register("Sound", &dvfm_dev_idx);
#endif

	return ret;
}

static void __exit zylonite_exit(void)
{
	platform_device_unregister(zylonite_snd_device);

#ifdef CONFIG_PXA3xx_DVFM
	dvfm_unregister("Sound", &dvfm_dev_idx);
#endif

}

module_init(zylonite_init);
module_exit(zylonite_exit);

/* Module information */
MODULE_AUTHOR("bin.yang@marvell.com");
MODULE_DESCRIPTION("ALSA SoC WM9713 zylonite");
MODULE_LICENSE("GPL");
