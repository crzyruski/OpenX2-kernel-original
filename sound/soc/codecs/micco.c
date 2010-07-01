/*
 * linux/sound/soc/codecs/micco.c
 * Base on linux/sound/soc/codecs/wm9712.c
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
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>

//#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/pxa-regs.h>
#include <mach/micco.h>
#include <mach/regs-ssp.h>
#include <mach/littleton.h>

#define DA9034_ERRATA_17	1

#define MICCO_SOC_PROC

#define	MICCO_MUX_MONO_V		0x16
#define	MICCO_MUX_BEAR_V		0x17
#define	MICCO_MUX_LINE_OUT_V		0x18
#define	MICCO_MUX_STEREO_CH1_V		0x19
#define	MICCO_MUX_STEREO_CH2_V		0x1a
#define	MICCO_MUX_TX_V			0x1b
#define MICCO_MIXER0_SELECT		0x1c
#define MICCO_MIXER1_SELECT		0x1d
#define	MICCO_MUX_VTCALLING_V	0x1e
#define	MICCO_MUX_DAC_V			0x1f
#define	MICCO_GSM_ADSW		0x20
/* Warnning, pls change MICCO_REG_SIZE and micco_regs when you add REG_V */
#define	MICCO_SOC_REG_SIZE	(MICCO_GSM_ADSW+1)

/* debug */
#define MICCO_DEBUG 0
#if MICCO_DEBUG
#define dbg(format, arg...) printk(KERN_INFO "micco: " format "\n", ## arg)
#else
#define dbg(format, arg...)
#endif

static unsigned int micco_soc_read(struct snd_soc_codec *codec,
	unsigned int reg);
static int micco_soc_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int val);
static int micco_soc_write_bit(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int val, unsigned int shift);
static int micco_soc_write_multibit(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int val, unsigned int shift,
	unsigned int bitnum);

static const u8 micco_regs[MICCO_SOC_REG_SIZE] = {
	0x00, 0x00, 0x00, 0x00,	/*0x00 ~ 0x03*/
	0x00, 0x25, 0x0, 0x80, /*0x04 ~ 0x07*/
	0x00, 0x8b, 0x3f, 0x00, /*0x08 ~ 0x0b*/
	0x0c, 0x04, 0x00, 0x00, /*0x0c ~ 0x0f*/
	0x04, 0x00, 0x00, 0x00,	/*0x10 ~ 0x13*/
	0x00, 0x00, 		/*0x14 ~ 0x15*/			
	0x00, 0x00, 0x00, 0x00,	/*0x16 ~ 0x19*/
	0x00, 0x00, 0x00, 0x00,	/*0x1a ~ 0x1d*/
	0x00, 0x00, 0x00,			/*0x1e ~ 0x1f*/
};

#include <mach/gpio.h>
#include <mach/mfp.h>
#if defined(CONFIG_BOARD_MN1)
#define PA_nSD  (MFP_PIN_GPIO81) //Speaker PA switch:GPIO 81
#elif defined(CONFIG_BOARD_X2_V4)
#define PA_nSD  (MFP_PIN_GPIO3_2) //Speaker PA switch:GPIO3_2
#elif defined(CONFIG_BOARD_TMQV1)||defined(CONFIG_BOARD_XPHONE) ||defined(CONFIG_BOARD_X2)
#define PA_nSD  (MFP_PIN_GPIO127) //Speaker PA switch:GPIO 127
#elif defined(CONFIG_BOARD_BRAVA) || defined(CONFIG_BOARD_X2G)
#define PA_nSD  (MFP_PIN_GPIO76) //Speaker PA switch:GPIO 127
#else
#pragma pls fix it
#define PA_nSD  (MFP_PIN_GPIO127) //Speaker PA switch:GPIO 127
#endif

#if defined(CONFIG_BOARD_LANDMARK)
#define PA_MICCO_GPO (0x1)
#endif

static void micco_open_PA(struct snd_soc_codec *codec)
{
#if defined(PA_MICCO_GPO)
	micco_soc_write_bit(codec, MICCO_AUDIO_LINE_AMP, 1, 6);
#else
	pxa3xx_mfp_set_afds(PA_nSD,MFP_AF0, MFP_DS04X);
	gpio_direction_output(mfp_to_gpio(PA_nSD), GPIO_LEVEL_HIGH);
	pxa3xx_mfp_set_lpm(PA_nSD, MFP_LPM_DRIVE_HIGH); /* let it high in sleep */
#endif
	printk(KERN_DEBUG"micco_open_PA\n");
}

static void micco_close_PA(struct snd_soc_codec *codec)
{
#if defined(PA_MICCO_GPO)
	micco_soc_write_bit(codec, MICCO_AUDIO_LINE_AMP, 0, 6);
#else
	pxa3xx_mfp_set_afds(PA_nSD,MFP_AF0, MFP_DS04X);
	gpio_direction_output(PA_nSD,GPIO_LEVEL_LOW);	//SPK_PA_GPIO->low
	pxa3xx_mfp_set_lpm(PA_nSD, MFP_LPM_FLOAT);
#endif
	printk(KERN_DEBUG"micco_close_PA\n");
}

#if defined(CONFIG_DUAL_MODEM)
#define GPIO_ADSW  (MFP_PIN_GPIO24)
static inline void gsm_adsw_set(int gsm)
{
	//printk("gsm_adsw_set %d\n", gsm);
	pxa3xx_mfp_set_afds(GPIO_ADSW, MFP_AF0, MFP_DS04X);
	if (gsm) {
		gpio_direction_output(GPIO_ADSW, GPIO_LEVEL_HIGH);
		pxa3xx_mfp_set_lpm(GPIO_ADSW, MFP_LPM_DRIVE_HIGH); /* let it high in sleep */
	} else {
		gpio_direction_output(GPIO_ADSW, GPIO_LEVEL_LOW);
		pxa3xx_mfp_set_lpm(GPIO_ADSW, MFP_LPM_FLOAT);
	}
}
#endif

#ifdef DA9034_ERRATA_17
static void micco_enable_pll( void )
{/* workaound for Micco record. */
	//micco_write(0x90, 0x01);
	//micco_write(0x94, 0x40);
	//micco_write(0x90, 0x00);
}
#endif

static const struct snd_kcontrol_new micco_direct_access[] = {
	SOC_SINGLE("MUX_MONO", MICCO_MUX_MONO, 0, 0xff, 0),
	SOC_SINGLE("MUX_BEAR", MICCO_MUX_BEAR, 0, 0xff, 0),
	SOC_SINGLE("MUX_LINE_OUT", MICCO_MUX_LINE_OUT, 0, 0xff, 0),
	SOC_SINGLE("MUX_STEREO_CH1", MICCO_MUX_STEREO_CH1, 0, 0xff, 0),
	SOC_SINGLE("MUX_STEREO_CH2", MICCO_MUX_STEREO_CH2, 0, 0xff, 0),
	SOC_SINGLE("AUDIO_LINE_AMP", MICCO_AUDIO_LINE_AMP, 0, 0xff, 0),
	SOC_SINGLE("STEREO_AMPLITUDE_CH1", MICCO_STEREO_AMPLITUDE_CH1, 0, 0xff, 0),
	SOC_SINGLE("STEREO_AMPLITUDE_CH2", MICCO_STEREO_AMPLITUDE_CH2, 0, 0xff, 0),
	SOC_SINGLE("HIFI_DAC_CONTROL", MICCO_HIFI_DAC_CONTROL, 0, 0xff, 0),
	SOC_SINGLE("MONO_VOL", MICCO_MONO_VOL, 0, 0xff, 0),
	SOC_SINGLE("BEAR_VOL", MICCO_BEAR_VOL, 0, 0xff, 0),
	SOC_SINGLE("I2S_CONTROL", MICCO_I2S_CONTROL, 0, 0xff, 0),
	SOC_SINGLE("TX_PGA", MICCO_TX_PGA, 0, 0xff, 0),
	SOC_SINGLE("MIC_PGA", MICCO_MIC_PGA, 0, 0xff, 0),
	SOC_SINGLE("TX_PGA_MUX", MICCO_TX_PGA_MUX, 0, 0xff, 0),
	SOC_SINGLE("VOICE_CODEC_ADC_CONTROL", MICCO_VCODEC_ADC_CONTROL, 0, 0xff, 0),
	SOC_SINGLE("VOICE_CODEC_VDAC_CONTROL", MICCO_VCODEC_VDAC_CONTROL, 0, 0xff, 0),
	SOC_SINGLE("SIDETONE", MICCO_SIDETONE, 0, 0xff, 0),
	SOC_SINGLE("PGA_AUXI1_2", MICCO_PGA_AUX1_2, 0, 0xff, 0),
	SOC_SINGLE("PGA_AUXI3", MICCO_PGA_AUX3, 0, 0xff, 0),
	SOC_SINGLE("PGA_DACS", MICCO_PGA_DACS, 0, 0xff, 0),
	SOC_SINGLE("SOFT_START_RAMP", MICCO_SOFT_START_RAMP, 0, 0xff, 0),
};

static int micco_id_get(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{
	int ret;
	unsigned char value;
	 
	ret = micco_read(0x00, &value);
	if (ret !=0 )
		return -1;

	ucontrol->value.integer.value[0] = value;
	return 0;
}


#if 0
//static const struct snd_kcontrol_new micco_id_read = 
	//SOC_SINGLE_EXT("MICCO_ID", MICCO_ID_V, 0, 0xff, 0, micco_id_get,NULL);
static int micco_add_direct_access(struct snd_soc_codec *codec)
{
	int err, i;
	for (i = 0; i < ARRAY_SIZE(micco_direct_access); i++) {
		err = snd_ctl_add(codec->card,
			snd_soc_cnew(&micco_direct_access[i], codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}
static int micco_id_access(struct snd_soc_codec *codec)
{
	int err;
	printk(KERN_ERR "micco_id_access\n");
	err = snd_ctl_add(codec->card, snd_soc_cnew(&micco_id_read, codec, NULL));
	if (err < 0)
		return err;

	return 0;
}
#endif

#ifdef CONFIG_MODEM_HUAWEI
extern int g_fmRadioOn;
#endif
static int do_post_event(struct snd_soc_dapm_widget *w, int event)
{
	u16 v_val;
	char *name = w->name;

	if (event & SND_SOC_DAPM_PRE_REG)
		return 0;

	if (!strcmp("MONO Mux", name)) {
		v_val = micco_soc_read(w->codec, MICCO_MUX_MONO_V);

		if (v_val!=7) /* AUDIO_HANDFREE, donot let codec sleep */
			soc_enable_suspend(AUDIO_HANDFREE, 0);
		else 
			soc_enable_suspend(AUDIO_HANDFREE, 1);
		
		if (v_val == 3||v_val == 4) // DAC1 or DAC2
			micco_soc_write(w->codec, MICCO_MUX_MONO, 0x18);
		else if (v_val == 8) {/* AUX1_2 */
#ifdef CONFIG_MODEM_HUAWEI /* inverted aux2 */
			if (!g_fmRadioOn)
				micco_soc_write(w->codec, MICCO_MUX_MONO, 0x41);
			else
#endif
				micco_soc_write(w->codec, MICCO_MUX_MONO, 0x3);
		} else if (v_val == 9) {/* DAC+AUX1_2 */
#ifdef CONFIG_MODEM_HUAWEI /* inverted aux2 */
			if (!g_fmRadioOn)
				micco_soc_write(w->codec, MICCO_MUX_MONO, 0x18|0x41);
			else
#endif
				micco_soc_write(w->codec, MICCO_MUX_MONO, 0x18|0x3);
		} else if (v_val == 10) {/* DAC+AUX3 */
			micco_soc_write(w->codec, MICCO_MUX_MONO, 0x18|0x4);
		} else if(v_val <= 6){
			micco_soc_write(w->codec, MICCO_MUX_MONO, 1<<(v_val));
		} else {
			micco_soc_write_multibit(w->codec, MICCO_MUX_MONO, 0, 0, 7);
		}
	} else if (!strcmp("BEAR Mux", name)) {
		v_val = micco_soc_read(w->codec, MICCO_MUX_BEAR_V);
		if(v_val <= 6){
			micco_soc_write(w->codec, MICCO_MUX_BEAR, 1<<(v_val));
		} else {
			micco_soc_write_multibit(w->codec, MICCO_MUX_BEAR, 0, 0, 7);
		}
	} else if (!strcmp("LINE_OUT Mux", name)) {
		v_val = micco_soc_read(w->codec, MICCO_MUX_LINE_OUT_V);
		if(v_val<=7){
			micco_soc_write(w->codec, MICCO_MUX_LINE_OUT, 1<<(v_val));
		} else {
			micco_soc_write_multibit(w->codec,MICCO_MUX_LINE_OUT, 0, 0, 8);
		}
	} else if (!strcmp("STEREO_CH1 Mux", name)) {
		v_val = micco_soc_read(w->codec, MICCO_MUX_STEREO_CH1_V);

		if (v_val!=7) /* AUDIO_HEADPHOE_CALL, donot let codec sleep */
			soc_enable_suspend(AUDIO_HEADPHONE, 0);
		else 
			soc_enable_suspend(AUDIO_HEADPHONE, 1);

		//printk("set STEREO_CH1 Mux v_val %d\n", v_val);
		if(v_val <=6) {
			micco_soc_write(w->codec, MICCO_MUX_STEREO_CH1, 1<<(v_val));
		} else if (v_val == 8) { // AUX1_2
			micco_soc_write(w->codec, MICCO_MUX_STEREO_CH1, 0x01);
		} else if (v_val == 9) { // DAC+AUX1_2
			micco_soc_write(w->codec, MICCO_MUX_STEREO_CH1, 0x08|0x01);
		} else if (v_val == 10) { //  DAC+AUX3
			micco_soc_write(w->codec, MICCO_MUX_STEREO_CH1, 0x08|0x4);
		} else {
			micco_soc_write_multibit(w->codec, MICCO_MUX_STEREO_CH1, 0, 0, 7);
		}
	} else if (!strcmp("STEREO_CH2 Mux", name)) {
		v_val = micco_soc_read(w->codec, MICCO_MUX_STEREO_CH2_V);
		if(v_val <=6){
			micco_soc_write(w->codec, MICCO_MUX_STEREO_CH2, 1<<(v_val));
		} else if (v_val == 8) { // AUX1_2
			micco_soc_write(w->codec, MICCO_MUX_STEREO_CH2, 0x02);
		} else if (v_val == 9) { // DAC+AUX1_2
			micco_soc_write(w->codec, MICCO_MUX_STEREO_CH2, 0x10|0x02);
		} else if (v_val == 10) { //  DAC+AUX3
			micco_soc_write(w->codec, MICCO_MUX_STEREO_CH2, 0x10|0x4);
		} else {
			micco_soc_write_multibit(w->codec,MICCO_MUX_STEREO_CH2, 0, 0, 7);
		}
	} else if (!strcmp("VTCALLING Mux", name)) {
		u8 mic_pga = micco_soc_read(w->codec, MICCO_MIC_PGA);
		v_val = micco_soc_read(w->codec, MICCO_MUX_VTCALLING_V);
		if (v_val == 1) { // Mic1 sel
			mic_pga &= ~MICCO_MIC_PGA_SELMIC_2;
			micco_soc_write(w->codec, MICCO_MIC_PGA, mic_pga);
		}else if(v_val == 2) { // Mic2 sel
			mic_pga |= MICCO_MIC_PGA_SELMIC_2;
			micco_soc_write(w->codec, MICCO_MIC_PGA, mic_pga);
		}
	} else if (!strcmp("TX Mux", name)) {
		u8 mic_pga = micco_soc_read(w->codec, MICCO_MIC_PGA);
		v_val = micco_soc_read(w->codec, MICCO_MUX_TX_V);
/* "AUX2", "AUX1", "MIC1", "MIC2", "AUX2inv",
   "AUX3", "DAC1", "DAC2", "AUX1_2+MIC1", "AUX3+MIC2", "NULL",
   "AUX1_2+MIC2", "AUX1_2" */
		if (v_val == 2) { //Microphone differential, Mic1 sel					
			micco_soc_write(w->codec, MICCO_TX_PGA_MUX, 0x0c);
			mic_pga &= ~MICCO_MIC_PGA_SELMIC_2;			
		}else if (v_val == 3) { //Microphone differential, Mic2 sel					
			micco_soc_write(w->codec, MICCO_TX_PGA_MUX, 0x0c);
			mic_pga |= MICCO_MIC_PGA_SELMIC_2;
			micco_soc_write(w->codec, MICCO_MIC_PGA, mic_pga);
		}else if (v_val == 8) { //AUX1_2+MIC1
			micco_soc_write(w->codec, MICCO_TX_PGA_MUX, 0x0c|0x03);
			mic_pga &= ~MICCO_MIC_PGA_SELMIC_2;
			micco_soc_write(w->codec, MICCO_MIC_PGA, mic_pga);
		}else if (v_val == 9) { //AUX3+MIC2
			micco_soc_write(w->codec, MICCO_TX_PGA_MUX, 0x20|0x0c);
			mic_pga |= MICCO_MIC_PGA_SELMIC_2;
			micco_soc_write(w->codec, MICCO_MIC_PGA, mic_pga);
		}else if (v_val == 11) { //AUX1_2+MIC2
			micco_soc_write(w->codec, MICCO_TX_PGA_MUX, 0x0c|0x03);
			mic_pga |= MICCO_MIC_PGA_SELMIC_2;
			micco_soc_write(w->codec, MICCO_MIC_PGA, mic_pga);
		}else if (v_val == 12) { //AUX1_2
			micco_soc_write(w->codec, MICCO_TX_PGA_MUX, 0x03);
		}else if (v_val <= 7) {
			micco_soc_write(w->codec, MICCO_TX_PGA_MUX, 1<<(v_val));
		}else{
			micco_soc_write_multibit(w->codec,MICCO_TX_PGA_MUX, 0, 0, 8);
		}
	}
	else if (!strcmp("SPK PA", name)) {
		//printk("SPK PA, event:0x%x %d\n", event, w->power);
		if (w->power)
			micco_open_PA(w->codec);
		else {
			micco_close_PA(w->codec);
		}
	} else {
		printk(KERN_ERR "Invalid widget name: %s\n", name);
		return -1;
	}

	return 0;
}

static int do_mixer0_event(struct snd_soc_dapm_widget *w, int event)
{
	u16 v_val;

	if (event & SND_SOC_DAPM_PRE_REG)
		return 0;

	v_val = micco_soc_read(w->codec, MICCO_MIXER0_SELECT);
	if (v_val == 0x1) {
		micco_soc_write_bit(w->codec, MICCO_MONO_VOL, 1, 6);
		micco_soc_write_bit(w->codec, MICCO_BEAR_VOL, 0, 6);
		micco_soc_write_bit(w->codec, MICCO_STEREO_AMPLITUDE_CH1, 0, 7);
	} else if (v_val == 0x2) {
		micco_soc_write_bit(w->codec, MICCO_MONO_VOL, 0, 6);
		micco_soc_write_bit(w->codec, MICCO_BEAR_VOL, 1, 6);
		micco_soc_write_bit(w->codec, MICCO_STEREO_AMPLITUDE_CH1, 0, 7);
	} else if (v_val == 0x3) {
		micco_soc_write_bit(w->codec, MICCO_MONO_VOL, 0, 6);
		micco_soc_write_bit(w->codec, MICCO_BEAR_VOL, 0, 6);
		micco_soc_write_bit(w->codec, MICCO_STEREO_AMPLITUDE_CH1, 1, 7);
	} else if (v_val == 0x4) {
		micco_soc_write_bit(w->codec, MICCO_MONO_VOL, 1, 6);
		micco_soc_write_bit(w->codec, MICCO_BEAR_VOL, 1, 6);
		micco_soc_write_bit(w->codec, MICCO_STEREO_AMPLITUDE_CH1, 0, 7);
	} else if (v_val == 0x5) {
		micco_soc_write_bit(w->codec, MICCO_MONO_VOL, 1, 6);
		micco_soc_write_bit(w->codec, MICCO_BEAR_VOL, 0, 6);
		micco_soc_write_bit(w->codec, MICCO_STEREO_AMPLITUDE_CH1, 1, 7);
	} else if (v_val == 0x6) {
		micco_soc_write_bit(w->codec, MICCO_MONO_VOL, 0, 6);
		micco_soc_write_bit(w->codec, MICCO_BEAR_VOL, 1, 6);
		micco_soc_write_bit(w->codec, MICCO_STEREO_AMPLITUDE_CH1, 1, 7);
	} else if (v_val == 0x7) {
		micco_soc_write_bit(w->codec, MICCO_MONO_VOL, 1, 6);
		micco_soc_write_bit(w->codec, MICCO_BEAR_VOL, 1, 6);
		micco_soc_write_bit(w->codec, MICCO_STEREO_AMPLITUDE_CH1, 1, 7);
	} else {
		micco_soc_write_bit(w->codec, MICCO_MONO_VOL, 0, 6);
		micco_soc_write_bit(w->codec, MICCO_BEAR_VOL, 0, 6);
		micco_soc_write_bit(w->codec, MICCO_STEREO_AMPLITUDE_CH1, 0, 7);
	}
	return 0;
}

static int do_mixer1_event(struct snd_soc_dapm_widget *w, int event)
{
	u16 v_val;

	if (event & SND_SOC_DAPM_PRE_REG)
		return 0;

	v_val = micco_soc_read(w->codec, MICCO_MIXER1_SELECT);
	if (v_val == 0x1)
		micco_soc_write_bit(w->codec, MICCO_MIC_PGA, 1, 3);
	else if (v_val == 0x2)
		micco_soc_write_bit(w->codec, MICCO_PGA_AUX1_2, 1, 3);
	else if (v_val == 0x3)
		micco_soc_write_bit(w->codec, MICCO_PGA_AUX1_2, 1, 7);
	else if (v_val == 0x4)
		micco_soc_write_bit(w->codec, MICCO_PGA_AUX3, 1, 3);
	else if (v_val == 0x5)
		micco_soc_write_bit(w->codec, MICCO_VCODEC_ADC_CONTROL, 1, 0);
	else if (v_val == 0x6)
		micco_soc_write_bit(w->codec, MICCO_VCODEC_VDAC_CONTROL, 1, 3);
	else if (v_val == 0x7)
		micco_soc_write_bit(w->codec, MICCO_HIFI_DAC_CONTROL, 1, 7);
	else if (v_val == 0x8)
		micco_soc_write_bit(w->codec, MICCO_AUDIO_LINE_AMP, 1, 4);
	else if (v_val == 0x9)
		micco_soc_write_bit(w->codec, MICCO_SIDETONE, 1, 7);
	else {
		micco_soc_write_bit(w->codec, MICCO_MIC_PGA, 0, 3);
		micco_soc_write_bit(w->codec, MICCO_PGA_AUX1_2, 0, 3);
		micco_soc_write_bit(w->codec, MICCO_PGA_AUX1_2, 0, 7);
		micco_soc_write_bit(w->codec, MICCO_PGA_AUX3, 0, 3);
		micco_soc_write_bit(w->codec, MICCO_VCODEC_ADC_CONTROL, 0, 0);
		micco_soc_write_bit(w->codec, MICCO_VCODEC_VDAC_CONTROL, 0, 3);
		micco_soc_write_bit(w->codec, MICCO_HIFI_DAC_CONTROL, 0, 7);
		micco_soc_write_bit(w->codec, MICCO_AUDIO_LINE_AMP, 0, 4);
		micco_soc_write_bit(w->codec, MICCO_SIDETONE, 0, 7);
	}

	return 0;
}

static int BT_PCM_enable = 0;
static int BT_PCM_control_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info * uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max =2;
	return 0;
}

static int BT_PCM_control_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	ucontrol->value.integer.value[0] = BT_PCM_enable;
	return 0;
}

extern void enable_oscc_pout(void);	
extern void disable_oscc_pout(void);
static struct clk* bt_ssp4_clk;
void BT_PCM_slave_enable_ssp4_clk(int rate)
{
	printk("Enable ssp4 clk[%d] \n",rate);
	if (!bt_ssp4_clk)
		bt_ssp4_clk = clk_get(NULL, "SSP4CLK");	
	if (!bt_ssp4_clk)
		clk_enable(bt_ssp4_clk);
	
	SSCR0_P4 &= ~SSCR0_SSE;
	/* hard code here, pls fix me */
	if(rate==8000)
		SSCR0_P4 = 0xc0593f;
	else
		SSCR0_P4 = 0xc02cbf;
	SSCR1_P4 = 0xf01dc0;
	SSPSP_P4 = 0x800085;
	SSCR0_P4 |= SSCR0_SSE;	
}

static void BT_PCM_slave_disable_ssp4_clk(void)
{
	SSCR0_P4 &= ~SSCR0_SSE;
	if (!bt_ssp4_clk)
		clk_disable(bt_ssp4_clk);
}

extern struct snd_soc_dai micco_dai[];
#define BT_PCM_MASTER_MODE 	1
#define BT_PCM_SLAVE_MODE 	2
static int BT_PCM_control_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_codec *codec = micco_dai[1].codec;
	u16 val;
	int new_BT_PCM_enable = ucontrol->value.integer.value[0];

	if (BT_PCM_enable==new_BT_PCM_enable)
		return 0;	

	soc_enable_suspend(AUDIO_BT_PCM, new_BT_PCM_enable);
	
	if (!new_BT_PCM_enable) {		
		printk(KERN_INFO "Disable bt pcm.\n");
		disable_oscc_pout();	/* 13M MCLK */
		
		if (BT_PCM_SLAVE_MODE == BT_PCM_enable)
			BT_PCM_slave_disable_ssp4_clk();
			
		pxa3xx_enable_ssp4_pins();
		
		snd_soc_dapm_stream_event(codec, 
			"Voice Playback", SND_SOC_DAPM_STREAM_STOP);
		snd_soc_dapm_stream_event(codec, 
			"Voice Capture", SND_SOC_DAPM_STREAM_STOP);
	} else {
		enable_oscc_pout(); /* 13M MCLK */		
#ifdef DA9034_ERRATA_17
		micco_enable_pll();
#endif

		if (BT_PCM_MASTER_MODE == new_BT_PCM_enable){ /* BT PCM master mode */
			printk(KERN_INFO "Enable bt pcm master mode.\n");
			pxa3xx_set_ssp4_pins_for_bt(1); 	
		} else if (BT_PCM_SLAVE_MODE == new_BT_PCM_enable){ /* BT PCM slave mode */
			printk(KERN_INFO "Enable bt pcm slave mode.\n");
			pxa3xx_set_ssp4_pins_for_bt(0);
			BT_PCM_slave_enable_ssp4_clk(8000); 
		}

		snd_soc_dapm_stream_event(codec, 
			"Voice Playback", SND_SOC_DAPM_STREAM_START);
		snd_soc_dapm_stream_event(codec, 
			"Voice Capture", SND_SOC_DAPM_STREAM_START);

		val = codec->read(codec, MICCO_VCODEC_ADC_CONTROL);
		val &= ~(0x03 << 3); /* 8k PCM */
		codec->write(codec, MICCO_VCODEC_ADC_CONTROL, val);
	}

	BT_PCM_enable = new_BT_PCM_enable;	
	return 0;
}

#ifdef DA9034_ERRATA_17
#define MUX_INPUT	\
	"AUX1", "AUX2", "AUX3", "DAC1", "DAC2", "DAC3", "AUX2inv", "NULL", "AUX1_2", "DAC+AUX1_2", "DAC+AUX3"
#else
#define MUX_INPUT	\
	"AUX1", "AUX2", "AUX2inv", "AUX3", "DAC1", "DAC2", "DAC3", "NULL", "AUX1_2", "DAC+AUX1_2", "DAC+AUX3"
#endif

#define MIXER0_SELECT	\
		"CLOSE", "MONO", "BEAR", "STEREO", "MONO+BEAR", \
		"MONO+STEREO", "BEAR+STEREO", "MONO+BEAR+STEREO"

#define MIXER1_SELECT   \
		"CLOSE", "MIC ENABLE", "AUX1 ENABLE", "AUX2 ENABLE",\
		"AUX3 ENABLE", "VOICE ADC ENABLE", "VOICE DAC ENABLE",\
		"HIFI DAC ENABLE", "LINE AMP ENABLE", "SIDETONE ENABLE"

static const char *micco_mono_mux[] = {MUX_INPUT};
static const char *micco_bear_mux[] = {MUX_INPUT};
static const char *micco_line_out_mux[] = {"AUX1", "AUX2", "AUX3", "DAC1", "DAC2", "DAC3", "AUX2inv", "MIC_P", "NULL"};
static const char *micco_stereo_ch1_mux[] = {MUX_INPUT};
static const char *micco_stereo_ch2_mux[] = {MUX_INPUT};
static const char *micco_tx_mux[] = {"AUX2", "AUX1", "MIC1", "MIC2", "AUX2inv",
       				"AUX3", "DAC1", "DAC2", "AUX1_2+MIC1", "AUX3+MIC2", "NULL",
       				"AUX1_2+MIC2", "AUX1_2"};
static const char *micco_select_mixer0[] = {MIXER0_SELECT};
static const char *micco_select_mixer1[] = {MIXER1_SELECT};
/* Open mic1 for calling, mic2 for headphone calling */
static const char *micco_virtual_calling_mux[] = {"NONE", "MIC1", "MIC2"};

static const struct soc_enum micco_enum[] = {
	SOC_ENUM_SINGLE(MICCO_MUX_MONO_V,	0, 11, micco_mono_mux), 
	SOC_ENUM_SINGLE(MICCO_MUX_BEAR_V,	0, 8, micco_bear_mux), 
	SOC_ENUM_SINGLE(MICCO_MUX_LINE_OUT_V,	0, 9, micco_line_out_mux),
	SOC_ENUM_SINGLE(MICCO_MUX_STEREO_CH1_V, 0, 11, micco_stereo_ch1_mux),
	SOC_ENUM_SINGLE(MICCO_MUX_STEREO_CH2_V, 0, 11, micco_stereo_ch2_mux),
	SOC_ENUM_SINGLE(MICCO_MUX_TX_V,		0, 13, micco_tx_mux),
	SOC_ENUM_SINGLE(MICCO_MIXER0_SELECT,	0, 8, micco_select_mixer0),
	SOC_ENUM_SINGLE(MICCO_MIXER1_SELECT,	0, 10, micco_select_mixer1),
	SOC_ENUM_SINGLE(MICCO_MUX_VTCALLING_V,	0, 3, micco_virtual_calling_mux),
};

static const struct snd_kcontrol_new micco_mono_mux_controls =
	SOC_DAPM_ENUM("Route", micco_enum[0]);
static const struct snd_kcontrol_new micco_bear_mux_controls =
	SOC_DAPM_ENUM("Route", micco_enum[1]);
static const struct snd_kcontrol_new micco_line_out_mux_controls =
	SOC_DAPM_ENUM("Route", micco_enum[2]);
static const struct snd_kcontrol_new micco_stereo_ch1_mux_controls =
	SOC_DAPM_ENUM("Route", micco_enum[3]);
static const struct snd_kcontrol_new micco_stereo_ch2_mux_controls =
	SOC_DAPM_ENUM("Route", micco_enum[4]);
static const struct snd_kcontrol_new micco_tx_mux_controls =
	SOC_DAPM_ENUM("Route", micco_enum[5]);
static const struct snd_kcontrol_new micco_select_mixer0_controls =
	SOC_DAPM_ENUM("Route", micco_enum[6]);
static const struct snd_kcontrol_new micco_select_mixer1_controls =
	SOC_DAPM_ENUM("Route", micco_enum[7]);
static const struct snd_kcontrol_new micco_vtcalling_mux_controls =
	SOC_DAPM_ENUM("Route", micco_enum[8]);

static const struct snd_soc_dapm_widget micco_dapm_widgets[] = {
#if 1 /* We want to use DAC1 DAC2 separatly */
	SND_SOC_DAPM_DAC("DAC1", "Left HiFi Playback", MICCO_MUX_DAC_V,
			0, 0),
	SND_SOC_DAPM_DAC("DAC2", "Right HiFi Playback", MICCO_MUX_DAC_V,
			1, 0),
#else
	SND_SOC_DAPM_DAC("DAC1", "Left HiFi Playback", MICCO_HIFI_DAC_CONTROL,
			7, 0),
	SND_SOC_DAPM_DAC("DAC2", "Right HiFi Playback", MICCO_HIFI_DAC_CONTROL,
			7, 0),
#endif
	SND_SOC_DAPM_DAC("DAC3", "Voice Playback", MICCO_VCODEC_VDAC_CONTROL,
			3, 0),
	SND_SOC_DAPM_ADC("ADC", "Voice Capture", MICCO_VCODEC_ADC_CONTROL,
			0, 0),

	SND_SOC_DAPM_MUX_E("MIXER0 Select", SND_SOC_NOPM,
			0, 0, &micco_select_mixer0_controls,
		do_mixer0_event, SND_SOC_DAPM_POST_REG),

	SND_SOC_DAPM_MUX_E("MIXER1 Select", SND_SOC_NOPM,
			0, 0, &micco_select_mixer1_controls,
		do_mixer1_event, SND_SOC_DAPM_POST_REG),
	SND_SOC_DAPM_MUX_E("VTCALLING Mux", SND_SOC_NOPM, 
			0, 0, &micco_vtcalling_mux_controls,
		do_post_event, SND_SOC_DAPM_POST_REG),		
	SND_SOC_DAPM_MUX_E("MONO Mux", SND_SOC_NOPM,
			0, 0, &micco_mono_mux_controls,
		do_post_event, SND_SOC_DAPM_POST_REG),
	SND_SOC_DAPM_MUX_E("BEAR Mux", SND_SOC_NOPM,
			0, 0, &micco_bear_mux_controls,
		do_post_event, SND_SOC_DAPM_POST_REG),
	SND_SOC_DAPM_MUX_E("LINE_OUT Mux", SND_SOC_NOPM, 0,
			0, &micco_line_out_mux_controls,
		do_post_event, SND_SOC_DAPM_POST_REG),
	SND_SOC_DAPM_MUX_E("STEREO_CH1 Mux", SND_SOC_NOPM,
			0, 0, &micco_stereo_ch1_mux_controls,
		do_post_event, SND_SOC_DAPM_POST_REG),
	SND_SOC_DAPM_MUX_E("STEREO_CH2 Mux", SND_SOC_NOPM,
			0, 0, &micco_stereo_ch2_mux_controls,
		do_post_event, SND_SOC_DAPM_POST_REG),
	SND_SOC_DAPM_MUX_E("TX Mux", SND_SOC_NOPM,
			0, 0, &micco_tx_mux_controls,
		do_post_event, SND_SOC_DAPM_POST_REG),

	SND_SOC_DAPM_PGA("MONO PGA", MICCO_MONO_VOL, 6, 0, NULL, 0),
	SND_SOC_DAPM_PGA("BEAR PGA", MICCO_BEAR_VOL, 6, 0, NULL, 0),

	SND_SOC_DAPM_PGA("LINE_OUT PGA", MICCO_AUDIO_LINE_AMP, 4, 0, NULL, 0),
	SND_SOC_DAPM_PGA("STEREO_CH PGA", MICCO_STEREO_AMPLITUDE_CH1, 
			7, 0, NULL, 0),

	SND_SOC_DAPM_PGA("MIC PGA", MICCO_MIC_PGA, 3, 0, NULL, 0),
	SND_SOC_DAPM_PGA("AUX1 PGA", MICCO_PGA_AUX1_2, 3, 0, NULL, 0),
	SND_SOC_DAPM_PGA("AUX2 PGA", MICCO_PGA_AUX1_2, 7, 0, NULL, 0),
	SND_SOC_DAPM_PGA("AUX3 PGA", MICCO_PGA_AUX3, 3, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("BEAR1"),
	SND_SOC_DAPM_OUTPUT("BEAR2"),
	SND_SOC_DAPM_OUTPUT("MONO1"),
	SND_SOC_DAPM_OUTPUT("MONO2"),
	SND_SOC_DAPM_OUTPUT("LINE_OUT"),
	SND_SOC_DAPM_OUTPUT("STEREO_CH1"),
	SND_SOC_DAPM_OUTPUT("STEREO_CH2"),
	SND_SOC_DAPM_OUTPUT("VTCALLING_OUT"),
	SND_SOC_DAPM_INPUT("AUX1"),
	SND_SOC_DAPM_INPUT("AUX2"),
	SND_SOC_DAPM_INPUT("AUX3"),
	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),
	SND_SOC_DAPM_INPUT("AUX1_2+MIC1"),  /* fm/modem_main_audio + mic */
	SND_SOC_DAPM_INPUT("AUX3+MIC2"), /* modem_second_audio+mic */
	SND_SOC_DAPM_INPUT("AUX1_2+MIC2"), /* fm/modem_main_audio+mic */
	SND_SOC_DAPM_INPUT("AUX1_2"),/* fm/modem_main_audio */
	SND_SOC_DAPM_INPUT("DAC+AUX1_2"), /* play sound while handfree */
	SND_SOC_DAPM_INPUT("DAC+AUX3"), /* play sound while headphone+calling */
	SND_SOC_DAPM_INPUT("MIC_P"),

	/*
	 * In TM,there is a PA between Speaker and Audio codec,
	 * and SPK PA connect to STEREO_CH1,CH2 of codec
	 */
	SND_SOC_DAPM_SPK("SPK PA", do_post_event),
	/* Mic Bias dynamic control */
	SND_SOC_DAPM_MICBIAS("MIC1 BIAS", MICCO_MIC_PGA, 5, 0),
	SND_SOC_DAPM_MICBIAS("MIC2 BIAS", MICCO_MIC_PGA, 6, 0),
};

static const char *micco_audio_map[][3] = {
	/*MONO Mux*/
	{"MONO Mux", "AUX1", "AUX1 PGA"},
	{"MONO Mux", "AUX2", "AUX2 PGA"},
	{"MONO Mux", "AUX3", "AUX3 PGA"},
	{"MONO Mux", "DAC1", "DAC1"},
	{"MONO Mux", "DAC2", "DAC2"},
	{"MONO Mux", "DAC3", "DAC3"},
	{"MONO Mux", "DAC+AUX1_2", "DAC1"},
	{"MONO Mux", "DAC+AUX1_2", "DAC2"},
	{"MONO Mux", "DAC+AUX1_2", "AUX1 PGA"},
	{"MONO Mux", "DAC+AUX1_2", "AUX2 PGA"},
	{"MONO Mux", "AUX1_2", "AUX1 PGA"},
	{"MONO Mux", "AUX1_2", "AUX2 PGA"},
	{"MONO Mux", "DAC+AUX3", "DAC1"},
	{"MONO Mux", "DAC+AUX3", "DAC2"},
	{"MONO Mux", "DAC+AUX3", "AUX3 PGA"},

	{"MONO PGA", NULL, "MONO Mux"},
	{"MONO1", NULL, "MONO PGA"},
	{"MONO2", NULL, "MONO PGA"},
	{"SPK PA", NULL, "MONO PGA"},

	/*BEAR Mux*/
	{"BEAR Mux", "AUX1", "AUX1 PGA"},
	{"BEAR Mux", "AUX2", "AUX2 PGA"},
	{"BEAR Mux", "AUX3", "AUX3 PGA"},
	{"BEAR Mux", "DAC1", "DAC1"},
	{"BEAR Mux", "DAC2", "DAC2"},
	{"BEAR Mux", "DAC3", "DAC3"},

	{"BEAR PGA", NULL, "BEAR Mux"},
	{"BEAR1", NULL, "BEAR PGA"},
	{"BEAR2", NULL, "BEAR PGA"},

	/*LINE OUT Mux */
	{"LINE_OUT Mux", "AUX1", "AUX1 PGA"},
	{"LINE_OUT Mux", "AUX2", "AUX2 PGA"},
	{"LINE_OUT Mux", "AUX3", "AUX3 PGA"},
	{"LINE_OUT Mux", "DAC1", "DAC1"},
	{"LINE_OUT Mux", "DAC2", "DAC2"},
	{"LINE_OUT Mux", "DAC3", "DAC3"},
	{"LINE_OUT Mux", "MIC_P", "MIC PGA"},
	
	{"LINE_OUT PGA", NULL, "LINE_OUT Mux"},
	{"LINE_OUT", NULL, "LINE_OUT PGA"},	

	/*STEREO_CH1 Mux*/
	{"STEREO_CH1 Mux", "AUX1", "AUX1 PGA"},
	{"STEREO_CH1 Mux", "AUX2", "AUX2 PGA"},
	{"STEREO_CH1 Mux", "AUX3", "AUX3 PGA"},
	{"STEREO_CH1 Mux", "DAC1", "DAC1"},
	{"STEREO_CH1 Mux", "DAC2", "DAC2"},
	{"STEREO_CH1 Mux", "DAC3", "DAC3"},
	{"STEREO_CH1 Mux", "DAC+AUX3", "DAC1"},
	{"STEREO_CH1 Mux", "DAC+AUX3", "AUX3 PGA"},
	{"STEREO_CH1 Mux", "DAC+AUX1_2", "DAC1"},
	{"STEREO_CH1 Mux", "DAC+AUX1_2", "AUX1 PGA"},

	{"STEREO_CH PGA", NULL, "STEREO_CH1 Mux"},
	{"STEREO_CH1", NULL, "STEREO_CH PGA"},
	
	/*STEREO_CH2 Mux*/
	{"STEREO_CH2 Mux", "AUX1", "AUX1 PGA"},
	{"STEREO_CH2 Mux", "AUX2", "AUX2 PGA"},
	{"STEREO_CH2 Mux", "AUX3", "AUX3 PGA"},
	{"STEREO_CH2 Mux", "DAC1", "DAC1"},
	{"STEREO_CH2 Mux", "DAC2", "DAC2"},
	{"STEREO_CH2 Mux", "DAC3", "DAC3"},
	{"STEREO_CH2 Mux", "DAC+AUX3", "DAC2"},
	{"STEREO_CH2 Mux", "DAC+AUX3", "AUX3 PGA"},
	{"STEREO_CH2 Mux", "DAC+AUX1_2", "DAC2"},
	{"STEREO_CH2 Mux", "DAC+AUX1_2", "AUX2 PGA"},

	{"STEREO_CH PGA", NULL, "STEREO_CH2 Mux"},
	{"STEREO_CH2", NULL, "STEREO_CH PGA"},

#if defined(CONFIG_BOARD_TMQV1)
	/* SPK PA staticly connected to STEREO_CH PGA in TMQ */
	{"SPK PA", NULL, "STEREO_CH PGA"},
#endif

	/*TX Mux*/
	{"TX Mux", "AUX1", "AUX1 PGA"},
	{"TX Mux", "AUX2", "AUX2 PGA"},
	{"TX Mux", "AUX3", "AUX3 PGA"},
	{"TX Mux", "DAC1", "DAC1"},
	{"TX Mux", "DAC2", "DAC2"},
	{"TX Mux", "MIC1", "MIC1 BIAS"},
	{"TX Mux", "MIC2", "MIC2 BIAS"},

	/* Ugly method for mixing several signals together,you can fix it */
	/* "AUX1_2+MIC1", "AUX3+MIC2", "NULL", "AUX1_2+MIC2", "AUX1_2" */
	{"TX Mux", "AUX1_2+MIC1", "AUX1 PGA"},
	{"TX Mux", "AUX1_2+MIC1", "AUX2 PGA"},
	{"TX Mux", "AUX1_2+MIC1", "MIC1 BIAS"},
	{"TX Mux", "AUX3+MIC2", "AUX3 PGA"},
	{"TX Mux", "AUX3+MIC2", "MIC2 BIAS"},
	{"TX Mux", "AUX1_2+MIC2", "AUX1 PGA"},
	{"TX Mux", "AUX1_2+MIC2", "AUX2 PGA"},
	{"TX Mux", "AUX1_2+MIC2", "MIC2 BIAS"},
	{"TX Mux", "AUX1_2", "AUX1 PGA"},
	{"TX Mux", "AUX1_2", "AUX2 PGA"},

	{"ADC", NULL, "TX Mux"},

	/* VTCALLING Mux */
	{"VTCALLING Mux", "MIC1", "MIC1 BIAS"},
	{"VTCALLING Mux", "MIC2", "MIC2 BIAS"},
	{"VTCALLING_OUT", NULL, "VTCALLING Mux"},

	/* MIC map */
	{"MIC1 BIAS", NULL, "MIC PGA"},
	{"MIC2 BIAS", NULL, "MIC PGA"},	
	{"MIC PGA", NULL, "MIC1"},
	{"MIC PGA", NULL, "MIC2"},

	/* AUX1,2,3 map */
	{"AUX1 PGA", NULL, "AUX1"},
	{"AUX2 PGA", NULL, "AUX2"},
	{"AUX3 PGA", NULL, "AUX3"},

	{NULL, NULL, NULL},
};

static const struct snd_kcontrol_new micco_snd_controls[] = {
	SOC_SINGLE("Mono Volume", MICCO_MONO_VOL, 0, 0x3f, 1),
	SOC_SINGLE("Bear Volume", MICCO_BEAR_VOL, 0, 0x3f, 1),
	SOC_SINGLE("Line Out Volume", MICCO_AUDIO_LINE_AMP, 0, 0xf, 1),
	SOC_SINGLE("Stereo Ch1 Volume", MICCO_STEREO_AMPLITUDE_CH1, 0, 0x3f, 1),
	SOC_SINGLE("Stereo Ch2 Volume", MICCO_STEREO_AMPLITUDE_CH2, 0, 0x3f, 1),
	SOC_SINGLE("Mic Volume", MICCO_MIC_PGA, 0, 0x7, 0),
	SOC_SINGLE("AUX1 Volume", MICCO_PGA_AUX1_2, 0, 0x3, 0),
	SOC_SINGLE("AUX2 Volume", MICCO_PGA_AUX1_2, 4, 0x3, 0),
	SOC_SINGLE("AUX3 Volume", MICCO_PGA_AUX3, 0, 0x3, 0),
	SOC_SINGLE("TX Volume", MICCO_TX_PGA, 1, 0xf, 0),
};

/* Add all new snd widget to here as we want to keep mux widget's id */
static const struct snd_kcontrol_new new_micco_snd_controls[] = {
	SOC_SINGLE("ADC Out Volume", MICCO_VCODEC_ADC_CONTROL , 5, 0x3, 1),
	SOC_SINGLE("Sidetone Volume", MICCO_SIDETONE, 0, 0x1f, 1),
	{/* bt pcm voice to gsm modem control */
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.iface = SNDRV_CTL_ELEM_IFACE_CARD,
		.name = "BT PCM",
		.private_value = 0,
		.info = BT_PCM_control_info,
		.get = BT_PCM_control_get,
		.put = BT_PCM_control_put,
	},
//#if defined(CONFIG_DUAL_MODEM)
#if 1 /* whatever let it exist */
	SOC_SINGLE("Gsm ADSW", MICCO_GSM_ADSW, 0, 0x1, 0),
#endif
	SOC_SINGLE("PCM LOOP", MICCO_VCODEC_VDAC_CONTROL, 7, 0x1, 0),
};

static int micco_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(micco_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
			snd_soc_cnew(&micco_snd_controls[i], codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

static int micco_add_new_snd_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(new_micco_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&new_micco_snd_controls[i],codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

static int micco_add_widgets(struct snd_soc_codec *codec)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(micco_dapm_widgets); i++)
		snd_soc_dapm_new_control(codec, &micco_dapm_widgets[i]);

	/* set up audio path audio_map nects */
	for (i = 0; micco_audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(codec, micco_audio_map[i][0],
			micco_audio_map[i][1], micco_audio_map[i][2]);
	}

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

static unsigned int micco_soc_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u8 *cache = codec->reg_cache;

	if (reg > (ARRAY_SIZE(micco_regs)))
		return -EIO;
	return cache[reg];
}

static int micco_soc_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int val)
{
	u8 *cache = codec->reg_cache;

	if (reg > (ARRAY_SIZE(micco_regs)))
		return -EIO;
	if (reg <= MICCO_SOFT_START_RAMP)
		micco_codec_write(reg, val);

	/* Add to use DAC1 DAC2 separatly */
	if (MICCO_MUX_DAC_V == reg) {
		unsigned int regVal = micco_soc_read(codec, MICCO_HIFI_DAC_CONTROL);
		if (val) {
			micco_soc_write(codec, MICCO_HIFI_DAC_CONTROL, regVal | (1<<7));
		} else {
			micco_soc_write(codec, MICCO_HIFI_DAC_CONTROL, regVal & (~1<<7));
		}
#if defined(CONFIG_DUAL_MODEM)
	} else if (MICCO_GSM_ADSW == reg) {
		gsm_adsw_set(val);
#endif
	}
	cache[reg] = val;

	return 0;
}

static int micco_soc_write_bit(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int val, unsigned int shift)
{
	unsigned int valtmp;

	if (reg > (ARRAY_SIZE(micco_regs)))
		return -EIO;
	if (reg <= MICCO_SOFT_START_RAMP) {

		valtmp = micco_soc_read(codec, reg);
		if (val)
			valtmp |= 1 << shift;
		else
			valtmp &= ~(1 << shift);

		micco_soc_write(codec, reg, valtmp);
	}
	return 0;
}

static int micco_soc_write_multibit(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int val, unsigned int shift,
	unsigned int bitnum)
{
	unsigned int valtmp;

	if (reg > (ARRAY_SIZE(micco_regs)))
		return -EIO;

	if (reg <= MICCO_SOFT_START_RAMP) {

		valtmp = micco_soc_read(codec, reg);

		valtmp = ((((valtmp >> bitnum) << bitnum)) | val) << shift;
		dbg("valtmp=%x\n", valtmp);

		micco_soc_write(codec, reg, valtmp);
	}
	return 0;

}

static int micco_reset(struct snd_soc_codec *codec, int try_warm)
{
	unsigned int reg;

	for (reg = 0; reg < ARRAY_SIZE(micco_regs); reg++)
		micco_soc_write(codec, reg, micco_regs[reg]);

	return 0;
}

static int micco_voice_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	u16 val;

#ifdef DA9034_ERRATA_17
	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK) {
		micco_enable_pll();
	}
#endif

	val = micco_soc_read(codec, MICCO_VCODEC_ADC_CONTROL);
	switch (runtime->rate) {
	case 8000:
		val &= ~(0x03 << 3);
		break;
	case 16000:
		val = (val & (~(0x03 << 3))) | (0x01 << 3);
		break;

	case 32000:
		val |= (0x03 << 3);
		break;
	default:
		return -EINVAL;
	}
	
	micco_soc_write(codec, MICCO_VCODEC_ADC_CONTROL, val);

	return 0;
}

static int micco_cp_voice_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static int micco_hifi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	u16 val;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		val = micco_soc_read(codec, MICCO_I2S_CONTROL);
		switch (runtime->rate) {
		case 8000:
			val &= 0xF0;
			break;
		case 11025:
			val &= 0xF0;
			val |= 0x01;
			break;
		case 12000:
			val &= 0xF0;
			val |= 0x02;
			break;
		case 16000:
			val &= 0xF0;
			val |= 0x03;
			break;
		case 22050:
			val &= 0xF0;
			val |= 0x04;
			break;
		case 24000:
			val &= 0xF0;
			val |= 0x05;
			break;
		case 32000:
			val &= 0xF0;
			val |= 0x06;
			break;
		case 44100:
			val &= 0xF0;
			val |= 0x07;
			break;
		case 48000:
			val &= 0xF0;
			val |= 0x0F;
			break;
		default:
			return -EINVAL;
		}
		val &= 0x0F;
		val |= 0x10;

		/* Set Micco as SSP Master. Also need use I2S normal mode
		 * instead of I2S justified mode to avoid noise.
		 */
		if (machine_is_saar() || machine_is_littleton()) {
			val &= ~0x10;
			val |= 0x20;
		}

		micco_soc_write(codec, MICCO_I2S_CONTROL, val);

	} else {
		printk(KERN_ERR "Micco HIFI does not support capture!\n");
		return -EINVAL;
	}
	return 0;
}

static void micco_hifi_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->codec;
	u16 val;

	if (machine_is_saar() || machine_is_littleton()) {
		val = micco_soc_read(codec, MICCO_I2S_CONTROL);
		val &= ~0x20;
		val |= 0x10;
		micco_soc_write(codec, MICCO_I2S_CONTROL, val);
	}

}

#define MICCO_HIFI_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000)

#define MICCO_VOICE_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | \
		SNDRV_PCM_RATE_32000)

#define MICCO_PCM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE)

struct snd_soc_dai micco_dai[] = {
{
	.name = "I2S HiFi",
	.playback = {
		.stream_name = "HiFi Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = MICCO_HIFI_RATES,
		.formats = MICCO_PCM_FORMATS,},
	.ops = {
		.prepare = micco_hifi_prepare,
		.shutdown = micco_hifi_shutdown,},
	},
	{
	.name = "PCM Voice",
	.playback = {
		.stream_name = "Voice Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = MICCO_VOICE_RATES,
		.formats = MICCO_PCM_FORMATS,},
	.capture = {
		.stream_name = "Voice Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = MICCO_VOICE_RATES,
		.formats = MICCO_PCM_FORMATS,},
	.ops = {
		.prepare = micco_voice_prepare,},
	},
	{
	.name = "CP PCM Voice",
	.playback = {
		.stream_name = "CP Voice Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = MICCO_VOICE_RATES,
		.formats = MICCO_PCM_FORMATS,},
	.capture = {
		.stream_name = "CP Voice Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = MICCO_VOICE_RATES,
		.formats = MICCO_PCM_FORMATS,},
	.ops = {
		.prepare = micco_cp_voice_prepare,},
	},
};
EXPORT_SYMBOL_GPL(micco_dai);

static int micco_set_bias_level(struct snd_soc_codec *codec, enum snd_soc_bias_level event)
{
	if (machine_is_tavorevb() || machine_is_saar()) {
		codec->bias_level = event;
		return 0;
	}

	switch (event) {
	case SND_SOC_BIAS_ON:
#ifdef DA9034_ERRATA_17
		micco_soc_write(codec, MICCO_HIFI_DAC_CONTROL,
			(micco_soc_read(codec, MICCO_HIFI_DAC_CONTROL)|(1<<7)));
#endif
		break;
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		break;
	case SND_SOC_BIAS_OFF:
#ifdef DA9034_ERRATA_17
		micco_soc_write(codec, MICCO_HIFI_DAC_CONTROL,
			(micco_soc_read(codec, MICCO_HIFI_DAC_CONTROL)
			&(~(1<<7))));
#endif

#if 0 /* rm by yuhua */
		micco_soc_write(codec, MICCO_AUDIO_LINE_AMP,
			(micco_soc_read(codec, MICCO_AUDIO_LINE_AMP)
			&(~MICCO_AUDIO_LINE_AMP_EN)));

		micco_soc_write(codec, MICCO_STEREO_AMPLITUDE_CH1,
			(micco_soc_read(codec, MICCO_STEREO_AMPLITUDE_CH1)
			&(~MICCO_STEREO_EN)));

		micco_soc_write(codec, MICCO_HIFI_DAC_CONTROL,
			(micco_soc_read(codec, MICCO_HIFI_DAC_CONTROL)
			&(~MICCO_HIFI_DAC_ON)));

		micco_soc_write(codec, MICCO_MONO_VOL,
			(micco_soc_read(codec, MICCO_MONO_VOL)
			&(~MICCO_MONO_EN)));

		micco_soc_write(codec, MICCO_BEAR_VOL,
			(micco_soc_read(codec, MICCO_BEAR_VOL)
			&(~MICCO_BEAR_EN)));

		micco_soc_write(codec, MICCO_MIC_PGA,
			(micco_soc_read(codec, MICCO_MIC_PGA)
			&(~(MICCO_MIC_PGA_EXT_EN | MICCO_MIC_PGA_INT_EN |
				MICCO_MIC_PGA_AMP_EN))));

		micco_soc_write(codec, MICCO_VCODEC_ADC_CONTROL,
			(micco_soc_read(codec, MICCO_VCODEC_ADC_CONTROL)
			&(~MICCO_VCODEC_ADC_ON_EN)));

		micco_soc_write(codec, MICCO_VCODEC_VDAC_CONTROL,
			(micco_soc_read(codec, MICCO_VCODEC_VDAC_CONTROL)
			&(~MICCO_VDAC_ON)));

		micco_soc_write(codec, MICCO_SIDETONE,
			(micco_soc_read(codec, MICCO_SIDETONE)
			&(~MICCO_SIDETONE_EN)));

		micco_soc_write(codec, MICCO_PGA_AUX1_2,
			(micco_soc_read(codec, MICCO_PGA_AUX1_2)
			&(~(MICCO_PGA_AUX1_EN | MICCO_PGA_AUX2_EN))));

		micco_soc_write(codec, MICCO_PGA_AUX3,
			(micco_soc_read(codec, MICCO_PGA_AUX3)
			&(~MICCO_PGA_AUX3_EN)));
#endif
		break;
	}

	codec->bias_level = event;
	return 0;
}

static int micco_soc_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	micco_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int micco_soc_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;
	u8 *cache = codec->reg_cache;
	unsigned int reg;


	for (reg = 0; reg < ARRAY_SIZE(micco_regs); reg++)
		micco_soc_write(codec, reg, cache[reg]);

	micco_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

#ifdef MICCO_SOC_PROC
static ssize_t micco_proc_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	char			*buf = page;
	char			*next = buf;
	unsigned		size = count;
	int 			t;
	int 			i;
	int 			reg;
	struct snd_soc_codec 	*codec = data;

	t = scnprintf(next, size, "Micco regs: \n");
	size -= t;
	next += t;

	for (i = 0; i < ARRAY_SIZE(micco_regs); i++) {
		reg = micco_soc_read(codec, i);
		t = scnprintf(next, size, "[0x%02x]=0x%02x  \n", i, reg);
		size -= t;
		next += t;
	}

	*eof = 1;
	return count - size;
}

static int micco_proc_write(struct file *file, const char __user *buffer,
			   unsigned long count, void *data)
{
	static char kbuf[30];
	char *buf = kbuf;
	struct snd_soc_codec 	*codec = data;
	unsigned int	i, reg, reg2;
	char cmd;	

	if (count >=30)
		return -EINVAL;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	sscanf(buf, "%c 0x%x 0x%x", &cmd, &i, &reg);
	
	if('r' == cmd) {
		if (i >= MICCO_SOC_REG_SIZE) {
			printk(KERN_ERR "invalid index!\n");
			goto error;
		}
		reg = micco_soc_read(codec, i);
		printk(KERN_INFO "0x[%x]=0x%x\n", i, reg);
	}else if('w' == cmd) {
		if (i >= MICCO_SOC_REG_SIZE) {
			printk(KERN_ERR "invalid index!\n");
			goto error;
		}
		if (reg > 0xff) {
			printk(KERN_ERR "invalid value!\n");
			goto error;
		}
		micco_soc_write(codec, i, reg);
		reg2 = micco_soc_read(codec, i);
		printk(KERN_INFO 
			"write 0x%x to 0x[%x], read back 0x%x\n", 
			reg, i, reg2);
	}else {
		printk(KERN_ERR "unknow opt!\n");
		goto error;
	}

	return count;
error:
	printk(KERN_INFO "r/w index(0x%%2x) value(0x%%2x)\n");
	return count;	
}
#endif

static ssize_t loop_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{	
	int enable;
	struct snd_soc_device *socdev = dev_get_drvdata(dev);
	struct snd_soc_codec *codec = socdev->codec;
	
	sscanf(buf, "%d", &enable);

	if (enable) {
		enable_oscc_pout(); /* 13M MCLK */	
#ifdef DA9034_ERRATA_17
		micco_enable_pll();
#endif
		
		micco_soc_write(codec, MICCO_MUX_MONO, 0x20); /* dac3->mono */
		micco_soc_write(codec, MICCO_MONO_VOL, 0xd0); /* mono amp */
		micco_soc_write(codec, MICCO_MIC_PGA, 0x2b); /* mic pga */
		micco_soc_write(codec, MICCO_TX_PGA_MUX, 0xc); /* mic->adc */
		micco_soc_write(codec, MICCO_VCODEC_ADC_CONTROL, /*adc on */
			MICCO_VCODEC_ADC_ON_EN|micco_soc_read(codec, MICCO_VCODEC_ADC_CONTROL)); 
		micco_soc_write(codec, MICCO_VCODEC_VDAC_CONTROL, /*dac on */
			(MICCO_VDAC_PCM_LOOP|MICCO_VDAC_ON)|micco_soc_read(codec, MICCO_VCODEC_VDAC_CONTROL)); 
			
		micco_open_PA(codec);
	} else {
		micco_close_PA(codec);
	}
		
	return count;
}

static DEVICE_ATTR(loop_test,0644,NULL,loop_test_store);
static struct attribute *micco_soc_attributes[] = {
	&dev_attr_loop_test.attr,
	NULL,
};
static struct attribute_group micco_soc_attr_group ={
	.attrs=micco_soc_attributes,
};

static int micco_soc_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;
#ifdef MICCO_SOC_PROC
	struct proc_dir_entry *micco_proc_entry;
#endif
	printk(KERN_INFO "Micco(DA9034) SoC Audio Codec\n");	
		
	socdev->codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);

	if (socdev->codec == NULL)
		return -ENOMEM;

	codec = socdev->codec;
	mutex_init(&codec->mutex);

	codec->reg_cache = kzalloc(sizeof(u16) * ARRAY_SIZE(micco_regs),
				GFP_KERNEL);
	if (codec->reg_cache == NULL) {
		ret = -ENOMEM;
		goto cache_err;
	}
	memcpy(codec->reg_cache, micco_regs,
		sizeof(u8) * ARRAY_SIZE(micco_regs));
	codec->reg_cache_size = sizeof(u8) * ARRAY_SIZE(micco_regs);
	codec->reg_cache_step = 1;
	codec->private_data = NULL;
	codec->name = "Micco";
	codec->owner = THIS_MODULE;
	codec->dai = micco_dai;
	codec->num_dai = ARRAY_SIZE(micco_dai);
	codec->write = micco_soc_write;
	codec->read = micco_soc_read;
	codec->set_bias_level = micco_set_bias_level;
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0)
		goto pcm_err;

	ret = micco_reset(codec, 0);
	if (ret < 0) {
		printk(KERN_ERR "Reset Micco error\n");
		goto reset_err;
	}
	micco_set_bias_level(codec, SND_SOC_BIAS_OFF);

	if (machine_is_tavorevb() || machine_is_saar()) {
		//micco_add_direct_access(codec);
		//micco_id_access(codec);
	} else {
		micco_add_controls(codec);
		micco_add_widgets(codec);
		micco_add_new_snd_controls(codec);
	}

	ret = snd_soc_register_card(socdev);
	if (ret < 0)
		goto reset_err;

#ifdef MICCO_SOC_PROC
	micco_proc_entry = create_proc_entry("driver/codec", 0, NULL);
	if (micco_proc_entry) {
		micco_proc_entry->data = codec;
		micco_proc_entry->read_proc = micco_proc_read;
		micco_proc_entry->write_proc = micco_proc_write;
	}
#endif

	ret = sysfs_create_group(&pdev->dev.kobj, &micco_soc_attr_group);

	micco_close_PA(codec);//Close spk pa firstly to avoid pop noise, why pA default high?

	return 0;

reset_err:
	snd_soc_free_pcms(socdev);

pcm_err:
	snd_soc_free_ac97_codec(codec);

cache_err:
	kfree(socdev->codec);
	socdev->codec = NULL;
	return ret;
}

static int micco_soc_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->codec;

	if (codec == NULL)
		return 0;

	snd_soc_dapm_free(socdev);
	snd_soc_free_pcms(socdev);
	snd_soc_free_ac97_codec(codec);
	kfree(codec->reg_cache);
	kfree(codec->dai);
	kfree(codec);
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_micco = {
	.probe = 	micco_soc_probe,
	.remove = 	micco_soc_remove,
	.suspend =	micco_soc_suspend,
	.resume = 	micco_soc_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_micco);

MODULE_DESCRIPTION("ASoC Micco driver");
MODULE_AUTHOR("bin.yang@marvell.com");
MODULE_LICENSE("GPL");


