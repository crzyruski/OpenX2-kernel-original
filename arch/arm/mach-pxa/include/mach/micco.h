/*
 * arch/arm/mach-pxa/include/mach/micco.h
 *
 * Copyright (C) 2006, Marvell Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _MICCO_H_
#define _MICCO_H_
#include <linux/i2c.h>
#include <linux/i2c-pxa.h>

#define MICCO_ADDRESS 		0x34

#define MICCO_CHIP_ID		0x00
#define MICCO_EVENT_A 		0x01
#define	MICCO_EA_ONKEY		(1)
#define	MICCO_EA_EXTON		(1 << 2)
#define	MICCO_EA_CHDET		(1 << 3)
#define	MICCO_EA_TBAT		(1 << 4)
#define	MICCO_EA_VBATMON	(1 << 5)
#define	MICCO_EA_REV_IOVER	(1 << 6)
#define	MICCO_EA_IOVER		(1 << 7)

#define MICCO_EVENT_B 		0x02
#define	MICCO_EB_CH_TCTO	(1 << 0)
#define	MICCO_EB_CH_CCTO	(1 << 1)
#define	MICCO_EB_USB_DEV	(1 << 2)
#define	MICCO_EB_OTGCP_IOVER	(1 << 3)
#define	MICCO_EB_VBUS_4P55	(1 << 4)
#define	MICCO_EB_VBUS_3P8	(1 << 5)
#define	MICCO_EB_SESSION_1P8	(1 << 6)
#define	MICCO_EB_SRP_READY	(1 << 7)

#define MICCO_EVENT_C 		0x03
#define	MICCO_EC_PEN_DOWN	(1 << 4)
#define	MICCO_EC_TSI_READY	(1 << 5)

#define MICCO_EVENT_D 		0x04
#define	MICCO_ED_HEADSET	(1 << 3)
#define	MICCO_ED_HOOKSWITCH	(1 << 4)

#define MICCO_STATUS_A 		0x05
#define	MICCO_STATUS_A_CHDET	(1 <<  3)

#define MICCO_STATUS_B 		0x06
#define	MICCO_STATUS_B_USBDEV	(1 << 0)
#define	MICCO_STATUS_B_HEADSET	(1 << 1)
#define	MICCO_STATUS_B_HOOKSWITCH	(1 << 2)
#define	MICCO_STATUS_B_VBUS_V_4P4	(1 << 4)
#define	MICCO_STATUS_B_VBUS_V_3P8	(1 << 5)
#define	MICCO_STATUS_B_SESS_V_1P8	(1 << 6)
#define	MICCO_STATUS_B_SRP_READY	(1 << 7)

#define MICCO_IRQ_MASK_A 	0x07
#define	IRQ_MACK_A_ONKEY	(1)
#define	IRQ_MASK_A_EXTON	(1 << 2)
#define	IRQ_MASK_A_CHDET	(1 << 3)
#define	IRQ_MASK_A_TBAT		(1 << 4)
#define	IRQ_MASK_A_VBATMON	(1 << 5)
#define	IRQ_MASK_A_REV_IOVER	(1 << 6)
#define	IRQ_MASK_A_CH_IOVER	(1 << 7)

#define MICCO_IRQ_MASK_B 	0x08
#define	IRQ_MASK_B_CH_TCTO		(1)
#define	IRQ_MASK_B_CH_CCTO		(1 << 1)
#define	IRQ_MASK_B_USB_DEV		(1 << 2)
#define	IRQ_MASK_B_OTGCP_IOVER		(1 << 3)
#define	IRQ_MASK_B_VBUS_VALID_4_55	(1 << 4)
#define	IRQ_MASK_B_VBUS_VALID_3_8	(1 << 5)
#define	IRQ_MASK_B_SESSION_VALID_1_8	(1 << 6)
#define	IRQ_MASK_B_SRP_READY_0_6	(1 << 7)


#define MICCO_IRQ_MASK_C 	0x09
#define	IRQ_MASK_C_ADC_MAN	(1)
#define	IRQ_MASK_C_ADC_AUTO4	(1 << 1)
#define	IRQ_MASK_C_ADC_AUTO5	(1 << 2)
#define	IRQ_MASK_C_ADC_AUTO6	(1 << 3)
#define	IRQ_MASK_C_PEN_DOWN	(1 << 4)
#define	IRQ_MASK_C_TSI_READY	(1 << 5)

#define MICCO_IRQ_MASK_D 	0x0A
#define IRQ_MASK_D_HOOKSWITCH	(1 << 4)
#define IRQ_MASK_D_HEADSET	(1 << 3)

#define MICCO_SYSCTRL_A		0x0B
#define MICCO_SYSCTRL_B		0x0C

#define MICCO_FAULT_LOG		0x0D

#define	MICCO_OVER1		0x10
#define	MICCO_APP_OVER2		0x11
#define	MICCO_APP_OVER3		0x12
#define	MICCO_LDO643		0x13
#define	MICCO_LDO987		0x14
#define	MICCO_LDO1110		0x15
#define	MICCO_LDO1312		0x16
#define	MICCO_LDO1514		0x17

#define	MICCO_BUCK_SLEEP	0x18
#define	MICCO_LDO_nSLEEP1	0x19
#define	MICCO_LDO_nSLEEP2	0x1A
#define	MICCO_LDO_SLEEP_BIT1	0x1B
#define	MICCO_LDO_SLEEP_BIT2	0x1C

#define	MICCO_VCC1		0x20
#define	MICCO_COMM_OVER2	0x21
#define	MICCO_COMM_OVER3	0x22
#define	MICCO_BUCK1_DVC1	0x23
#define	MICCO_BUCK1_DVC2	0x24
#define	MICCO_APPS_AVRC		0x25
#define	MICCO_COMM_CDTV1	0x26
#define	MICCO_COMM_CDTV2	0x27
#define	MICCO_COMM_CVRC		0x28
#define	MICCO_SRAM_DVC1		0x29
#define	MICCO_SRAM_DVC2		0x2A
#define	MICCO_SRAM_SVRC		0x2B

#define	MICCO_LDO1_MDTV1	0x32
#define	MICCO_LDO1_MDTV2	0x33
#define	MICCO_LDO1_MVRC		0x34

#define	MICCO_LED1_CONTROL	0x35
#define	MICCO_LED2_CONTROL	0x36
#define	MICCO_LEDPC_CONTROL1	0x37
#define	MICCO_LEDPC_CONTROL2	0x38
#define	MICCO_LEDPC_CONTROL3	0x39
#define	MICCO_LEDPC_CONTROL4	0x3A
#define	MICCO_LEDPC_CONTROL5	0x3B

#define	MICCO_WLED_CONTROL1	0x3C
#define	MICCO_WLED_CONTROL2	0x3D
#define	MICCO_FLASH_CONTROL1	0x3E
#define	MICCO_FLASH_CONTROL2	0x3F

#define	MICCO_VIBRA_CONTROL	0x40

#define	MICCO_MISC		0x41
#define	MICCO_MISC_VBUS_COMPS_EN	(1 << 7)
#define	MICCO_MISC_USBSR_EN		(1 << 6)
#define	MICCO_MISC_USBCP_EN		(1 << 5)
#define	MICCO_MISC_I_TBAT_ON		(1 << 4)
#define MICCO_MISC_REMCON_FILTER	(1 << 2)
#define MICCO_MISC_REMCON_ENABLE	(1 << 1)
#define MICCO_MISC_REMCON_AUTO		(1 << 0)

#define	MICCO_CHARGE_CONTROL	0x42

#define	MICCO_DAC_MSB		0x46
#define	MICCO_DAC_LSB		0x47

#define	MICCO_ADC_MAN_CONTROL	0x50
#define MICCO_ADC_MAN_CONT_LDOADC_EN	(1 << 4)
#define MICCO_ADC_MAN_CONTROL_IDBAT_EN	(1 << 5)
#define MICCO_ADC_MAN_CONT_CONV		(1 << 3)

#define	MICCO_ADC_AUTO_CONTROL_1	0x51
#define MICCO_ADC_AUTO_1_AUTOADC_SLEEP_EN  (1)
#define MICCO_ADC_AUTO_1_VBAT_EN           (1 << 1)
#define MICCO_ADC_AUTO_1_ICH_EN            (1 << 2)
#define MICCO_ADC_AUTO_1_VBAT_EXTRA_EN     (1 << 4)
#define MICCO_ADC_AUTO_1_TBAT_EN           (1 << 5)
#define MICCO_ADC_AUTO_1_DEB_VBAT_MON      (1 << 6)

#define	MICCO_ADC_AUTO_CONTROL_2	0x52
#define MICCO_ADC_AUTO_2_ADC4_EN        (1)
#define MICCO_ADC_AUTO_2_ADC5_EN        (1 << 1)
#define MICCO_ADC_AUTO_2_ADC6_EN        (1 << 2)
#define	MICCO_ADC_AUTO_2_PENDET_EN	(1 << 4)
#define	MICCO_ADC_AUTO_2_TSI_EN		(1 << 3)



#define	MICCO_TSI_CONTROL_1	0x53
#define	MICCO_TSI_CONTROL_2	0x54

#define	MICCO_VBATMON		0x55
#define	MICCO_VBATHIGHP		0x56
#define	MICCO_TBATHIGHN		0x57
#define	MICCO_TBATLOW		0x58
#define	MICCO_AUTO4HIGH		0x59
#define	MICCO_AUTO4LOW		0x5A
#define	MICCO_AUTO5HIGH		0x5B
#define	MICCO_AUTO5LOW		0x5C
#define	MICCO_AUTO6HIGH		0x5D
#define	MICCO_AUTO6LOW		0x5E

#define MICCO_MAN_RES_LSB	0x5F
#define	MICCO_MAN_RES_MSB	0x60
#define	MICCO_VBAT_RES		0x61
#define	MICCO_VBATMIN_RES	0x62
#define	MICCO_ICHMAX_RES	0x63
#define	MICCO_ICHMIN_RES	0x64
#define	MICCO_ICHAVERAGE_RES	0x65
#define	MICCO_VCHMAX_RES	0x66
#define	MICCO_VCHMIN_RES	0x67
#define	MICCO_TBAT_RES		0x68
#define	MICCO_AUTO4_RES		0x69
#define	MICCO_AUTO5_RES		0x6A
#define	MICCO_AUTO6_RES		0x6B

/* Micco TSI registers */
#define	MICCO_TSI_X_MSB		0x6C
#define	MICCO_TSI_Y_MSB		0x6D
#define	MICCO_TSI_XY_MSB	0x6E

/* Micco Audio control registers */
#define	MICCO_AUDIO_REG_BASE		0x70
#define	MICCO_AUDIO_REGS_NUM		22

#define	MICCO_MUX_MONO			0x0
#define	MICCO_MUX_BEAR			0x1
#define	MICCO_MUX_LINE_OUT		0x2
#define	MICCO_MUX_STEREO_CH1		0x3
#define	MICCO_MUX_STEREO_CH2		0x4


#define	MICCO_AUDIO_LINE_AMP		0x5
#define	MICCO_AUDIO_LINE_AMP_EN		(1 << 4)
#define	LINE_OUT_MIN_VOL		0
#define	LINE_OUT_MAX_VOL		15

#define	MICCO_STEREO_AMPLITUDE_CH1	0x6
#define	MICCO_STEREO_GAIN_SEPARATE	(1 << 6)
#define	MICCO_STEREO_EN			(1 << 7)
#define	STEREO_MIN_VOL			0
#define	STEREO_MAX_VOL			63

#define	MICCO_STEREO_AMPLITUDE_CH2	0x7
#define	MICCO_STEREO_FAST_START		(1 << 6)

#define	MICCO_HIFI_DAC_CONTROL		0x8
#define	MICCO_HIFI_DAC_ON		(1 << 7)
#define	MICCO_HIFI_HPF_BYPASS_2		(1 << 6)
#define	MICCO_HIFI_HPF_BYPASS_1		(1 << 5)
#define	MICCO_HIFI_DAC_MUTE2		(1 << 4)
#define	MICCO_HIFI_DAC_MUTE1		(1 << 3)
#define	MICCO_HIFI_DAC_INV_2		(1 << 2)
#define	MICCO_HIFI_DAC_INV_1		(1 << 1)

#define	MICCO_MONO_VOL			0x9
#define	MICCO_MONO_FAST_START		(1 << 7)
#define	MICCO_MONO_EN			(1 << 6)
#define	MONO_MIN_VOL			0
#define	MONO_MAX_VOL			51


#define	MICCO_BEAR_VOL			0xA
#define	MICCO_BEAR_FAST_START		(1 << 7)
#define	MICCO_BEAR_EN			(1 << 6)
#define	BEAR_MIN_VOL			0
#define	BEAR_MAX_VOL			51


#define	MICCO_I2S_CONTROL		0xB
#define	MICCO_I2S_SYNC_OUTPUT		(1 << 7)
#define	MICCO_I2S_SRM_EN		(1 << 6)
#define	MICCO_I2S_DA_MASTER		(1 << 5)
#define	MICCO_I2S_MSB_JU_MODE		(1 << 4)


#define	MICCO_TX_PGA			0xC
#define	MICCO_TX_PGA_DSP_0DB		(1)


#define	MICCO_MIC_PGA			0xD
#define	MICCO_MIC_PGA_EXT_EN		(1 << 6)
#define	MICCO_MIC_PGA_INT_EN		(1 << 5)
#define	MICCO_MIC_PGA_SELMIC_2		(1 << 4)
#define	MICCO_MIC_PGA_AMP_EN		(1 << 3)
#define	MIC_PGA_MIN_GAIN		0
#define	MIC_PGA_MAX_GAIN		7

#define	MICCO_TX_PGA_MUX		0xE

#define	MICCO_VCODEC_ADC_CONTROL	0xF
#define	MICCO_VCODEC_ADC_MUTE		(1 << 7)
#define	MICCO_VCODEC_ADC_ON_EN		(1)


#define	MICCO_VCODEC_VDAC_CONTROL	0x10
#define	MICCO_VDAC_PCM_LOOP		(1 << 7)
#define	MICCO_VDAC_PCM_SDOTRI		(1 << 6)
#define	MICCO_VDAC_ON			(1 << 3)
#define	MICCO_VDAC_HPF_BYPASS		(1 << 2)
#define	MICCO_VDAC_HPF_INV		(1 << 1)
#define	MICCO_VDAC_HPF_MUTE		(1)


#define	MICCO_SIDETONE			0x11
#define	MICCO_SIDETONE_EN		(1 << 7)
#define	MICCO_SIDETONE_GAIN_EN		(1 << 6)
#define	MICCO_SIDETONE_GAIN_STEREO	(1 << 5)

#define	MICCO_PGA_AUX1_2		0x12
#define	MICCO_PGA_AUX1_EN		(1 << 7)
#define	MICCO_PGA_AUX2_EN		(1 << 3)
#define	AUX1_2_MIN_VOL			0
#define	AUX1_2_MAX_VOL			3


#define	MICCO_PGA_AUX3			0x13
#define	MICCO_PGA_AUX3_EN		(1 << 3)
#define	AUX3_MIN_VOL			0
#define	AUX3_MAX_VOL			3


#define	MICCO_PGA_DACS			0x14
#define	MICCO_SOFT_START_RAMP		0x15
#define MICCO_REM_IN_POLLING_TIME	0x16
#define PERIOD_125_MS			0
#define PERIOD_250_MS			1
#define PERIOD_500_MS			2
#define PERIOD_1000_MS			3

#define	MICCO_VBUCK1_BASE		725
#define	MICCO_VBUCK1_STEP		25
#define	MICCO_VBUCK1_MAX		1500

#define	MICCO_VSRAM_BASE		725
#define	MICCO_VSRAM_STEP		25
#define	MICCO_VSRAM_MAX			1500

#define	MICCO_VLDO1_BASE		1700
#define	MICCO_VLDO1_STEP		25
#define	MICCO_VLDO1_MAX			2075

#define	MICCO_VLDO3_BASE		1800
#define	MICCO_VLDO3_STEP		100
#define	MICCO_VLDO3_MAX			3300

#define	MICCO_VLDO6_BASE		2500
#define	MICCO_VLDO6_STEP		50
#define	MICCO_VLDO6_MAX			2850

#define	MICCO_VLDO12_BASE		1700
#define	MICCO_VLDO12_BASE1		2700
#define	MICCO_VLDO12_STEP		50
#define	MICCO_VLDO12_MAX		3050

#define	MICCO_VLDO13_BASE		1800
#define	MICCO_VLDO13_STEP		100
#define	MICCO_VLDO13_MAX		3300

#define	MICCO_VLDO15_BASE		1800
#define	MICCO_VLDO15_STEP		100
#define	MICCO_VLDO15_MAX		3300

#define	MICCO_VLDO14_BASE		1800
#define	MICCO_VLDO14_STEP		100
#define	MICCO_VLDO14_MAX		3300

#define	MICCO_VLDO9_BASE		2700
#define	MICCO_VLDO9_STEP		50
#define	MICCO_VLDO9_MAX			3050

#define	MICCO_VLDO10_BASE		2700
#define	MICCO_VLDO10_STEP		50
#define	MICCO_VLDO10_MAX		3050

#define	MICCO_VLDO11_BASE		1800
#define	MICCO_VLDO11_STEP		100
#define	MICCO_VLDO11_MAX		3300


/* Micco audio definition. TODO: move to micco codec
 * driver.
 */

#define	MICCO_CODEC_INPUT_NUMBER	7
enum {
	CODEC_AUX1 = 0,
	CODEC_AUX2,
	CODEC_AUX3,
	CODEC_MIC1,
	CODEC_MIC2,
	CODEC_PCM,
	CODEC_HIFI,
};

#define	MICCO_CODEC_OUPUT_NUMBER	5
enum {
	CODEC_BEAR = 0,
	CODEC_MONO,
	CODEC_STEREO,
	CODEC_LINE_OUT,
	CODEC_ADC,
};

enum {
	MICCO_VOICE_PORT = 0,
	MICCO_HIFI_PORT,
};

enum {
	BUCK1 = 1,
	BUCK2,
	BUCK3,
	LDO1,
	LDO2,
	LDO3,
	LDO4,
	LDO5,
	LDO6,
	LDO7,
	LDO8,
	LDO9,
	LDO10,
	LDO11,
	LDO12,
	LDO13,
	LDO14,
	LDO15,
	USB_OTG,
	LDO_GPADC,
	LDO_AUDIO,
	LDO_PMCORE,
	LDO_BBAT,
};

struct power_supply_module {
	int command;
	int power_module;
};

struct power_chip {
#define MICCO_B0_ID	0x10
#define MICCO_EA_ID	0x30
#define MICCO_EB_ID	0x31
	int chip_id;
	char *chip_name;
	struct power_supply_module *power_supply_modules;
};

struct micco_platform_data {
	int		(*init_irq)(void);
	int		(*ack_irq)(void);
	void		(*platform_init)(void);
	spinlock_t	lock;
	struct work_struct	work;
	struct power_chip	*power_chips;
};

/* General */
int micco_read(u8 reg, u8 *val);
int micco_write(u8 reg, u8 val);

int micco_enable_pen_down_irq(int pen_en);
int micco_tsi_enable_pen(int pen_en);
int micco_tsi_enable_tsi(int tsi_en);
int micco_tsi_poweron(void);
int micco_tsi_poweroff(void);
int micco_tsi_readxy(u16 *x, u16 *y, int pen_state);

/* For Audio */
int micco_codec_read(u8 reg, u8 *val);
int micco_codec_write(u8 reg, u8 val);
int micco_codec_set_input_gain(int type, int gain);
int micco_codec_set_sample_rate(int port, int rate);
int micco_codec_enable_input(int type);
int micco_codec_disable_input(int type);
int micco_codec_enable_output(int type);
int micco_codec_disable_output(int type);
int micco_codec_enable_input(int type);
int micco_codec_disable_input(int type);
int micco_codec_set_output_vol(int type, int vol);
int micco_audio_init(void);

void micco_enable_LDO15(int enable);
void micco_enable_LDO13(int enable);
void micco_enable_LDO3(int enable);
void micco_enable_LDO7(int enable);
void micco_enable_LDO9(int enable);
void micco_enable_LDO11(int enable);

#endif

