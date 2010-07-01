/*
 *  linux/arch/arm/mach-pxa/littleton.c
 *
 *  Support for the Marvell Littleton Development Platform.
 *
 *  Author:	Jason Chagas (largely modified code)
 *  Created:	Nov 20, 2006
 *  Copyright:	(C) Copyright 2006 Marvell International Ltd.
 *
 *  2007-11-22  modified to align with latest kernel
 *              eric miao <eric.miao@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/i2c/max7320.h>
#include <linux/i2c/max7321.h>
#include <linux/spi/spi.h>
#include <linux/smc91x.h>
#include <linux/pda_power.h>

#include <asm/types.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/pxa-regs.h>
#include <mach/mfp-pxa300.h>
#include <mach/gpio.h>
#include <mach/pxafb.h>
#include <mach/ssp.h>
#include <mach/pxa2xx_spi.h>
#include <mach/pxa27x_keypad.h>
#include <mach/littleton.h>
#include <mach/pxa3xx_nand.h>
#include <mach/pxa3xx_pmic.h>
#include <mach/pxa3xx_dvfm.h>
#include <mach/micco.h>
#include <mach/ohci.h>
#include <mach/pxa3xx_u2d.h>
#include <mach/udc.h>
#include <mach/uart.h>
#include <mach/irda.h>
#include <mach/mmc.h>
#include <mach/imm.h>
#include <mach/pxa3xx_pm.h>
#include <mach/pmu.h>
#include <mach/part_table.h>
#include <mach/camera.h>
#include <mach/pxa3xx-regs.h>
#include <mach/yuhua_sign.h>
#include <linux/i2c/rda5802.h>

#include <linux/suspend.h>
#include "devices.h"
#include "generic.h"

#define MAX_SLOTS      3
struct platform_mmc_slot littleton_mmc_slot[MAX_SLOTS];
extern int is_android(void);

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

/* Littleton MFP configurations */
static mfp_cfg_t littleton_mfp_cfg[] __initdata = {
	//alex.chen added
#ifdef CONFIG_BT
	/* BTUART */
#if defined(CONFIG_SERIAL_GSM2_BT)
	GPIO107_UART3_CTS,
	//GPIO108_UART3_RTS,
	
	MFP_CFG_LPM(GPIO108, AF1, DRIVE_HIGH),/*set  RTS high,in sleep */
#else
	GPIO111_UART2_RTS,
	GPIO112_UART2_RXD,
	GPIO113_UART2_TXD,
	GPIO114_UART2_CTS,
#endif
#endif
	/* LCD */
	GPIO54_LCD_LDD_0,
	GPIO55_LCD_LDD_1,
	GPIO56_LCD_LDD_2,
	GPIO57_LCD_LDD_3,
	GPIO58_LCD_LDD_4,
	GPIO59_LCD_LDD_5,
	GPIO60_LCD_LDD_6,
	GPIO61_LCD_LDD_7,
	GPIO62_LCD_LDD_8,
	GPIO63_LCD_LDD_9,
	GPIO64_LCD_LDD_10,
	GPIO65_LCD_LDD_11,
	GPIO66_LCD_LDD_12,
	GPIO67_LCD_LDD_13,
	GPIO68_LCD_LDD_14,
	GPIO69_LCD_LDD_15,
	//GPIO70_LCD_LDD_16,
	//GPIO71_LCD_LDD_17,
	GPIO72_LCD_FCLK,
	GPIO73_LCD_LCLK,
	GPIO74_LCD_PCLK,
	GPIO75_LCD_BIAS,

	/* I2C */
	GPIO21_I2C_SCL,
	GPIO22_I2C_SDA,

	/* FFUART */
	GPIO30_GPIO,
	GPIO31_GPIO,
	GPIO77_UART1_RXD,
	GPIO78_UART1_TXD,
	GPIO79_UART1_CTS,
	MFP_CFG_LPM(GPIO84, AF0, PULL_HIGH),/*set  RTS high,in sleep */

#if defined(CONFIG_SERIAL_GSM2_BT)
	GPIO19_UART2_RXD,
	GPIO18_UART2_TXD,
	GPIO15_UART2_CTS,
	MFP_CFG_LPM(GPIO16, AF0, PULL_HIGH),/*set  RTS high,in sleep */
#endif


	/* STUART */
	GPIO109_UART3_TXD,
	GPIO110_UART3_RXD,

	/* Keypad */
    	MFP_CFG_LPM(GPIO115, AF1, PULL_LOW),
    	MFP_CFG_LPM(GPIO116, AF1, PULL_LOW),
    	MFP_CFG_LPM(GPIO117, AF1, PULL_LOW),
	MFP_CFG_LPM(GPIO118, AF1, PULL_LOW),
	GPIO121_KP_MKOUT_0,
	GPIO122_KP_MKOUT_1,
	GPIO123_KP_MKOUT_2,
#if defined(CONFIG_BOARD_X2_V4)
	MFP_CFG_LPM(GPIO127, AF5, PULL_LOW) | 0x8000, /* pull en */
	MFP_CFG_LPM(GPIO1_2, AF2, PULL_LOW) | 0x8000,
#elif defined(CONFIG_BOARD_X2) || defined(CONFIG_BOARD_BRAVA) /* direct key */
	MFP_CFG_LPM(GPIO107, AF2, PULL_LOW) | 0x8000, /* pull en */
	MFP_CFG_LPM(GPIO108, AF2, PULL_LOW) | 0x8000,
#endif

	/* SSP1 */
#if defined(CONFIG_SERIAL_XR20M)
	GPIO85_SSP1_SCLK,
	MFP_CFG(GPIO3_2, AF3), /* frm */
	GPIO88_SSP1_TXD,
	GPIO87_SSP1_RXD,
#endif

	/* SSP2 */
	GPIO25_SSP2_SCLK,
#if defined(CONFIG_BOARD_BRAVA)
	GPIO26_SSP2_FRM,
#elif defined(CONFIG_BOARD_X2_V4)
	GPIO17_SSP2_FRM,
#endif
	GPIO27_SSP2_TXD,

	/* SSP3 */
	GPIO91_SSP3_SCLK,
	GPIO92_SSP3_FRM,
	GPIO93_SSP3_TXD,
	GPIO94_SSP3_RXD,
	/* SSP3 NETWORK CLK */
	GPIO126_EXT_CLK,

	/* SSP4 */
	GPIO95_SSP4_SCLK,
	GPIO96_SSP4_FRM,
	GPIO97_SSP4_TXD,
	GPIO98_SSP4_RXD,

	/* NAND */

	/* MMC1 */
	GPIO3_MMC1_DAT0,
	GPIO4_MMC1_DAT1,
	GPIO5_MMC1_DAT2,
	GPIO6_MMC1_DAT3,
	GPIO7_MMC1_CLK,
	GPIO8_MMC1_CMD,
	GPIO15_GPIO, /* card detect */

#ifndef CONFIG_BOARD_X2_V4
/*there is no MMC2 on X2V4*/
	/* MMC2 */
	GPIO9_MMC2_DAT0,
	GPIO10_MMC2_DAT1,
	GPIO11_MMC2_DAT2,
	GPIO12_MMC2_DAT3,
	GPIO13_MMC2_CLK,
	GPIO14_MMC2_CMD,
#endif

#ifdef CONFIG_CPU_PXA310
	/* MMC3 */
	GPIO7_2_MMC3_DAT0,
	GPIO8_2_MMC3_DAT1,
	GPIO9_2_MMC3_DAT2,
	GPIO10_2_MMC3_DAT3,
	GPIO103_MMC3_CLK,
	GPIO105_MMC3_CMD,
#endif

#ifdef CONFIG_BOARD_X2_V4
	/* si4703 FM radio */
	GPIO10_GPIO, /* RDS interrupt*/
	GPIO11_GPIO, /* Stereo interrupt */
#endif	
	/* QCI */
	GPIO39_CI_DD_0,
	GPIO40_CI_DD_1,
	GPIO41_CI_DD_2,
	GPIO42_CI_DD_3,
	GPIO43_CI_DD_4,
	GPIO44_CI_DD_5,
	GPIO45_CI_DD_6,
	GPIO46_CI_DD_7,
#if defined(CONFIG_BOARD_X2)
#if defined(CONFIG_BOARD_X2_V4)
#else
	GPIO47_CI_DD_8,
	GPIO48_CI_DD_9,
#endif
#endif
	GPIO49_CI_MCLK,
	GPIO50_CI_PCLK,
};

static mfp_cfg_t littleton_ssp4_pins[] = {
	GPIO95_SSP4_SCLK,
	GPIO96_SSP4_FRM,
	GPIO97_SSP4_TXD,
	GPIO98_SSP4_RXD,
};

static mfp_cfg_t littleton_ssp4_pins_for_bt[] = {
	GPIO97_GPIO,
	GPIO98_GPIO,
	GPIO95_GPIO,
	GPIO96_GPIO,
};

void pxa3xx_gpio_in_group(mfp_cfg_t *mfp_cfgs, int num)
{
	int i, pin;
	mfp_cfg_t *mfp_cfg = mfp_cfgs;

	for (i = 0; i < num; i++, mfp_cfg++) {
		gpio_direction_input(MFP2GPIO(*mfp_cfg));
	}
}

void pxa3xx_enable_ssp4_pins(void)
{
	pxa3xx_mfp_config(ARRAY_AND_SIZE(littleton_ssp4_pins));
}

void pxa3xx_set_ssp4_pins_for_bt(int master)
{
	int size;
	
	if(master)
		size = ARRAY_SIZE(littleton_ssp4_pins_for_bt);
	else
		size = 2; /* Reserve clk and frm as bt slave in PCM */
	
	pxa3xx_mfp_config(littleton_ssp4_pins_for_bt, size);
	pxa3xx_gpio_in_group(littleton_ssp4_pins_for_bt, size);
}

#if defined(CONFIG_USB_PXA3XX_U2D) || defined(CONFIG_USB_USB_PXA3XX_U2D_MODULE)
static mfp_cfg_t littleton_u2d_cfg[] __initdata = {
	/* ULPI*/
	GPIO29_GPIO,
	GPIO38_ULPI_CLK,
	GPIO30_ULPI_DATA_OUT_0,
	GPIO31_ULPI_DATA_OUT_1,
	GPIO32_ULPI_DATA_OUT_2,
	GPIO33_ULPI_DATA_OUT_3,
	GPIO34_ULPI_DATA_OUT_4,
	GPIO35_ULPI_DATA_OUT_5,
	GPIO36_ULPI_DATA_OUT_6,
	GPIO37_ULPI_DATA_OUT_7,
	GPIO33_ULPI_OTG_INTR,
	GPIO106_ULPI_RESET,
	ULPI_DIR,
	ULPI_NXT,
	ULPI_STP,
};

static void littleton_reset_xcvr(void)
{
	int reset_pin;
	int err;

	reset_pin = ULPI_RESET_PIN;
	err = gpio_request(reset_pin, "U2D Reset");
	if (err) {
		gpio_free(reset_pin);
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d return :%d\n", reset_pin, err);
		return;
	}
	gpio_direction_output(reset_pin, 0);
	mdelay(100);
	gpio_set_value(reset_pin, 1);

	gpio_direction_input(reset_pin);

	gpio_free(reset_pin);
}

static mfp_cfg_t pxa310_ulpidat3_enable[] = {
	GPIO33_GPIO,
};

static mfp_cfg_t pxa310_ulpidat3_disable[] = {
	GPIO33_ULPI_DATA_OUT_3,
};

static void littleton_pxa310_ulpi_dat3(int enable)
{
	if (enable)
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa310_ulpidat3_enable));
	else
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa310_ulpidat3_disable));
}

static struct pxa3xx_u2d_mach_info littleton_u2d_info = {
	.reset_xcvr = littleton_reset_xcvr,
	.ulpi_dat3 = littleton_pxa310_ulpi_dat3,
};
#endif /* CONFIG_USB_PXA3XX_U2D || CONFIG_USB_PXA3XX_U2D_MODULE */

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
static unsigned int littleton_matrix_key_map[] = {
#if defined(CONFIG_BOARD_XPHONE)
	KEY(0, 1, KEY_VOLUMEUP), KEY(1, 0, KEY_VOLUMEDOWN), KEY(0, 2, KEY_CAMERA)/*camera*/,
	KEY(2, 1, KEY_BACK), KEY(3, 0, KEY_HOME),KEY(3, 1, KEY_KBDILLUMDOWN)/*menu*/,
#elif defined(CONFIG_BOARD_X2)	
	KEY(2, 2, KEY_BACK), KEY(1, 2, KEY_HOME),/*home or dial 2*/ KEY(0, 1, KEY_SEND),/*dial 1*/
	KEY(2, 0, KEY_LEFT), KEY(1, 0, KEY_RIGHT), KEY(0, 0, KEY_UP), KEY(2, 1, KEY_DOWN), 
	KEY(1, 1, KEY_KBDILLUMDOWN)/*menu*/, KEY(3, 0, KEY_VOLUMEUP), KEY(3, 1, KEY_VOLUMEDOWN),
#elif defined(CONFIG_BOARD_BRAVA)
	KEY(1, 2, KEY_SEND), KEY(1, 1, KEY_KBDILLUMDOWN)/*menu*/, KEY(1, 0, KEY_BACK),
	KEY(3, 2, KEY_HOME), KEY(3, 1, KEY_VOLUMEDOWN), KEY(3, 0, KEY_VOLUMEUP),
#else
	KEY(0, 0, KEY_1), KEY(0, 1, KEY_4), KEY(0, 2, KEY_7),
	KEY(1, 0, KEY_2), KEY(1, 1, KEY_5), KEY(1, 2, KEY_8), KEY(1, 3, KEY_0),
	KEY(2, 0, KEY_3), KEY(2, 1, KEY_6), KEY(2, 2, KEY_9),
	KEY(0, 3, KEY_KPASTERISK),
	KEY(2, 3, KEY_KPDOT),
	KEY(3, 0, KEY_BACK),KEY(3, 1, KEY_HOME),/*change for Xphone keymap*/
	KEY(3, 2, KEY_F22),
	KEY(3, 3, KEY_F23),
	KEY(4, 0, KEY_SEND),
	KEY(4, 1, KEY_END),
	KEY(4, 2, KEY_VOLUMEUP),
	KEY(4, 3, KEY_VOLUMEDOWN),
	KEY(5, 0, KEY_UP),
	KEY(5, 1, KEY_DOWN),
	KEY(5, 2, KEY_LEFT),
	KEY(5, 3, KEY_RIGHT),
	KEY(5, 4, KEY_ENTER),
#endif
};

static unsigned int littleton_direct_key_map[] = {
#if defined(CONFIG_BOARD_X2) || defined(CONFIG_BOARD_BRAVA)
	KEY_CAMERA, /* DK0 */
	KEY_SOUND, /* DK1 */
#endif
};

static struct pxa27x_keypad_platform_data littleton_keypad_info = {
	.matrix_key_rows	= 6,
	.matrix_key_cols	= 5,
	.matrix_key_map		= littleton_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(littleton_matrix_key_map),

	.direct_key_num		= ARRAY_SIZE(littleton_direct_key_map),
	.direct_key_map	=  littleton_direct_key_map,
	
	.enable_rotary0		= 0,
	.rotary0_up_key		= KEY_UP,
	.rotary0_down_key	= KEY_DOWN,

	.debounce_interval	= 30,

	.slideint_mfp = -1,
};
static void __init littleton_init_keypad(void)
{
	pxa_set_keypad_info(&littleton_keypad_info);
}
#else
static inline void littleton_init_keypad(void) {}
#endif

static struct platform_device micco_ts_device = {
	.name 		= "lt_ts",
	.id 		= -1,
};

static struct platform_device micco_kp_bl_device = {
	.name 		= "micco-kp-bl",
	.id 		= -1,
};

static struct resource pxa3xx_resource_imm[] = {
	[0] = {
		.name   = "phy_sram",
		.start	= 0x5c000000,
		.end	= 0x5c000000 + 2 * PHYS_SRAM_BANK_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#if !defined(CONFIG_ANDROID_RAM_CONSOLE)
	[1] = {
		.name   = "imm_sram",
		.start	= 0x5c000000 + PHYS_SRAM_BANK_SIZE,
		.end    = 0x5c000000 + 2 * PHYS_SRAM_BANK_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
#else	
	/*share 64K internel sram to ram console,after hardware reset all of the content in sram is 0
	 can we use some external sram for ram console?*/
	[1] = {
		.name   = "imm_sram",
		.start	= 0x5c000000 + PHYS_SRAM_BANK_SIZE,
		.end	= 0x5c02ffff,
		.flags	= IORESOURCE_MEM,
	},
#endif	
};

static struct platform_device pxa3xx_device_imm = {
	.name 		= "pxa3xx-imm",
	.id 		= -1,
	.num_resources	= ARRAY_SIZE(pxa3xx_resource_imm),
	.resource	= pxa3xx_resource_imm,
};

static struct platform_device micco_charger_device = {
	.name		= "micco-charger",
	.id		= -1,
};

static struct platform_device goldfish_battery_device = {
	.name		= "goldfish-battery",
	.id		= -1,
};

static struct platform_device modem_nxp5209_device = {
	.name		= "modem_nxp5209",
	.id		= -1,
};

static struct platform_device acc_2d_mxc6202_device = {
	.name		= "mxc6202",
	.id		= -1,
};

#if defined(CONFIG_BATTERY_PXA3xx)
static struct platform_device pxa3xx_battery_device = {
	.name		= "pxa3xx-battery",
	.id		= -1,
};
#endif

#if defined(CONFIG_CHARGER_PXA3xx_HARDWARE)
static struct platform_device pxa3xx_hwcharger_device = {
	.name		= "pxa3xx-hwcharger",
	.id		= -1,
};
#endif

#if defined(CONFIG_ANDROID_HAEDSET)
static struct platform_device android_headset_device = {
	.name		= "android-headset",
	.id		= -1,
};
#endif

#if defined(CONFIG_ANDROID_RAM_CONSOLE)
static struct resource pxa3xx_resource_ram_console[] = {
	{
		.start	= 0x5c040000,
		.end	= 0x5c04ffff,
		.flags	= IORESOURCE_MEM,
	}
};

static struct platform_device android_ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources  = ARRAY_SIZE(pxa3xx_resource_ram_console),
	.resource       = pxa3xx_resource_ram_console,
};

#endif

static struct platform_device *devices[] __initdata = {
	//&smc91x_device,
	&micco_ts_device,
	//&micco_bl_device,
	&pxa3xx_device_imm,
	&micco_charger_device,
	&goldfish_battery_device,
#if defined(CONFIG_BATTERY_PXA3xx)
	&pxa3xx_battery_device,
#endif
#if defined(CONFIG_CHARGER_PXA3xx_HARDWARE)
	&pxa3xx_hwcharger_device,
#endif
#if defined(CONFIG_ANDROID_HAEDSET)
	&android_headset_device,
#endif
	&modem_nxp5209_device,
	&acc_2d_mxc6202_device,
#if defined(CONFIG_ANDROID_RAM_CONSOLE)
	&android_ram_console_device,
#endif	
};

#if defined(CONFIG_MTD_NAND_PXA3xx) || defined(CONFIG_MTD_NAND_PXA3xx_MODULE)
static struct pxa3xx_nand_platform_data littleton_nand_info;
static void __init littleton_init_nand(void)
{
	if (is_android()) {
		littleton_nand_info.parts = pxa300_android_128m_partitions;
		littleton_nand_info.nr_parts = ARRAY_SIZE(pxa300_android_128m_partitions);
	} else {
		littleton_nand_info.parts = pxa300_128m_partitions;
		littleton_nand_info.nr_parts = ARRAY_SIZE(pxa300_128m_partitions);
	}

	pxa3xx_device_nand.dev.platform_data = &littleton_nand_info;
	platform_device_register(&pxa3xx_device_nand);
}
#else
static inline void littleton_init_nand(void) {}
#endif /* CONFIG_MTD_NAND_PXA3xx || CONFIG_MTD_NAND_PXA3xx_MODULE */

#if defined(CONFIG_PXA3xx_MICCO) || defined(CONFIG_PXA3xx_MICCO_MODULE)
#if defined(CONFIG_BOARD_BRAVA)
#define MFP_PMIC_INT (MFP_PIN_GPIO0_2)
#define MFP_PMIC_WAKE_AF (MFP_AF0) //gpio wake
#else
#define MFP_PMIC_INT (MFP_PIN_GPIO102)
#define MFP_PMIC_WAKE_AF (MFP_AF1)
#endif
static int micco_init_irq(void)
{
	//pxa3xx_mfp_config(ARRAY_AND_SIZE(littleton_pmic_pins));
	pxa3xx_mfp_set_afds(MFP_PMIC_INT, MFP_AF0, MFP_DS03X);
	pxa3xx_mfp_set_lpm(MFP_PMIC_INT, MFP_LPM_PULL_HIGH);
	gpio_direction_input(mfp_to_gpio(MFP_PMIC_INT));
	return 0;
}

static int micco_ack_irq(void)
{
	return 0;
}

static void littleton_micco_init(void)
{
	u8 value;

	/* Mask interrupts that are not needed */
	micco_write(MICCO_IRQ_MASK_A, 0xFE);
	micco_write(MICCO_IRQ_MASK_B, 0xFF);
	micco_write(MICCO_IRQ_MASK_C, 0xFF);
	micco_write(MICCO_IRQ_MASK_D, 0xFF);

	/* avoid SRAM power off during sleep*/
	micco_write(0x10, 0x05);
	micco_write(0x11, 0xff);
	micco_write(0x12, 0xff);

	/* Enable the ONKEY power down functionality */
	micco_write(MICCO_SYSCTRL_B, 0x20);
	micco_write(MICCO_SYSCTRL_A, 0x63); /* enable sleep */

	micco_write(MICCO_BUCK_SLEEP, 0x6D);

	/* IRQ is masked during the power-up sequence and will not be released
	 * until they have been read for the first time */
	micco_read(MICCO_EVENT_A, &value);
	micco_read(MICCO_EVENT_B, &value);
	micco_read(MICCO_EVENT_C, &value);
	micco_read(MICCO_EVENT_D, &value);
}

/* micco_power_module[] should be consistent with enum
 * in arch/arm/mach-pxa/include/mach/pxa3xx_pmic.h */
static struct power_supply_module miccoB0_power_modules[] = {
	/* {command,		power_module}, */
	{VCC_CORE,		BUCK1},
	{VCC_SRAM,		LDO2},
	{VCC_MVT,		LDO1},
	{VCC_3V_APPS,		LDO3},
	{VCC_SDIO,		LDO14},
	{VCC_CAMERA_ANA,	LDO6},
	{VCC_USB,		LDO3},
	{VCC_LCD,		LDO12},
	{VCC_TSI,		0},
	{VCC_CAMERA_IO,		LDO15},
	{VCC_1P8V,		LDO4},
	{VCC_MEM,		BUCK2},
	{HDMI_TX,		LDO9},
	{TECH_3V,		LDO10},
	{TECH_1P8V,		LDO11},
};

static struct power_chip micco_chips[] = {
	{0x00,	"miccoB0",	miccoB0_power_modules},
	{0x10,	"miccoB0",	miccoB0_power_modules},
	{0x11,	"miccoB0",	miccoB0_power_modules},
	{0,	NULL,		NULL},
};

static struct micco_platform_data micco_data = {
	.init_irq = micco_init_irq,
	.ack_irq = micco_ack_irq,
	.platform_init = littleton_micco_init,
	.power_chips = micco_chips,
};
#endif /* CONFIG_PXA3xx_MICCO || CONFIG_PXA3xx_MICCO_MODULE*/

#if defined(CONFIG_RADIO_RDA5802)
int rda5802_setup(struct i2c_client *client, void *context)
{
	int ldo346;
	micco_read(MICCO_LDO643, &ldo346);
	ldo346|=0xe0;/*set LDO6 to 2.85V*/
	micco_write(MICCO_LDO643,ldo346);	

	//int reset_pin = mfp_to_gpio(MFP_PIN_GPIO86);
	int irq_pin = mfp_to_gpio(MFP_PIN_GPIO10);
#if 0
	gpio_request(reset_pin, "rda5802 FM radio reset");
#endif	
	gpio_request(irq_pin, "rda5802 FM radio interrupt");
	/* configure interrupt pin as input */
	gpio_direction_input(irq_pin);
	
#if 0
	/* configure interrupt pin as input */
	gpio_direction_input(reset_pin);

	/* assert reset for 100 ms*/
	gpio_direction_output(reset_pin, 0);
	mdelay(100);

	/* deassert reset */
	gpio_set_value(reset_pin, 1);

	gpio_free(reset_pin);
#endif	
	gpio_free(irq_pin);

	return 0;
}

static struct rda5802_platform_data rda5802_data = {
	.setup = rda5802_setup,
};
#endif
static struct i2c_board_info littleton_i2c_board_info[] = {
#if defined(CONFIG_PXA3xx_MICCO) || defined(CONFIG_PXA3xx_MICCO_MODULE)
	{
		.type	= "micco",
		.addr		= 0x34,
		.platform_data	= &micco_data,
		.irq		= IRQ_GPIO(mfp_to_gpio(MFP_PMIC_INT)),
	},
#endif
#if defined(CONFIG_RADIO_RDA5802)
	{
	  .type           = "rda5802",
	  .addr           = 0x10,
	  .platform_data  = &rda5802_data,
	  .irq            = IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO10)),
	},
#endif

};

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
static struct pxaohci_platform_data littleton_ohci_info = {
	.port_mode	= PMM_PERPORT_MODE,
};
static void __init littleton_init_ohci(void)
{
	pxa_set_ohci_info(&littleton_ohci_info);
}
#else
static inline void littleton_init_ohci(void) {}
#endif /* CONFIG_USB_OHCI_HCD || CONFIG_USB_OHCI_HCD_MODULE */

struct pxa3xx_freq_mach_info littleton_freq_mach_info = {
/*	.flags = PXA3xx_USE_POWER_I2C,*/
/*if we use poweri2c,we can not poweron,after "poweroff" in kernel*/
	.flags = PXA3xx_USE_POWER_I2C,
};

#if defined(CONFIG_MMC) || defined(CONFIG_MMC_MODULE)
#if defined(CONFIG_BOARD_BRAVA)
#define WLAN_PD 		MFP_PIN_GPIO102
#define WLAN_PWREN 	MFP_PIN_GPIO101
#define WLAN_RST 		MFP_PIN_GPIO106
#define BT_WAKEUP MFP_PIN_GPIO104
#define BT_WAKEUP_HOST MFP_PIN_GPIO1_2
#define BT_RTS		MFP_PIN_GPIO111
#elif defined(CONFIG_BOARD_X2)

	#if defined(CONFIG_BOARD_X2_V4)
#define WLAN_PWREN 	MFP_PIN_GPIO112
#define BT_SW		MFP_PIN_GPIO114
#define BT_RTS		MFP_PIN_GPIO108
	#else
#define WLAN_PWREN 	MFP_PIN_GPIO1_2
#define BT_RTS		MFP_PIN_GPIO111
	#endif
#define WLAN_PD 		MFP_PIN_GPIO101
#define WLAN_RST 		MFP_PIN_GPIO119
#define BT_WAKEUP MFP_PIN_GPIO76
#define BT_WAKEUP_HOST MFP_PIN_GPIO94
#else
#define WLAN_PD 		MFP_PIN_GPIO101
#define WLAN_PWREN 	MFP_PIN_GPIO1_2
#define WLAN_RST 		MFP_PIN_GPIO119
#define BT_WAKEUP MFP_PIN_GPIO76
#define BT_WAKEUP_HOST MFP_PIN_GPIO94
#define BT_RTS		MFP_PIN_GPIO111
#endif

void set_bt_deepsleep()
{
	//gpio_direction_output(MFP2GPIO(BT_WAKEUP), GPIO_LEVEL_HIGH);
	//msleep(100);
	//gpio_direction_output(MFP2GPIO(BT_RTS), GPIO_LEVEL_HIGH);
	
	/*make sure GH381 GPIO5 high*/
	int count=5;
	int value=0;
	do
	{
		
		value=gpio_get_value(BT_WAKEUP_HOST);
		if(value)
			break;
		msleep(500);
	}while(value==0&&count--);

	pxa3xx_mfp_set_afds(BT_RTS,MFP_AF0, MFP_DS04X);
	gpio_direction_output(BT_RTS, GPIO_LEVEL_HIGH);
	pxa3xx_mfp_set_lpm(BT_RTS, MFP_LPM_DRIVE_HIGH);
	pxa3xx_mfp_set_rdh(BT_RTS, 1);
	
}
EXPORT_SYMBOL_GPL(set_bt_deepsleep);

void out_bt_deepsleep()
{
	//gpio_direction_output(MFP2GPIO(BT_WAKEUP), GPIO_LEVEL_LOW);
	//msleep(100);
	//gpio_direction_output(MFP2GPIO(BT_RTS), GPIO_LEVEL_LOW);
	//pxa3xx_mfp_set_afds(BT_RTS,MFP_AF0, MFP_DS04X);
	//gpio_direction_output(BT_RTS, GPIO_LEVEL_LOW);
	pxa3xx_mfp_set_afds(BT_RTS,MFP_AF1, MFP_DS04X);
}
EXPORT_SYMBOL_GPL(out_bt_deepsleep);
#ifdef CONFIG_PXA3xx_MODEM_DVFM
#include <mach/dvfm.h>
#include <mach/mspm_prof.h>
int mspmst=0;
#endif
void set_dsr_on(int index)
{

#ifdef CONFIG_PXA3xx_MODEM_DVFM
	//dvfm_disable_op_name("D0CS", index);
	//dvfm_disable_op_name("104M", index);
	//mspmst=mspm_state();
	//if(mspmst) {
	//	mspm_disable();
	//} 
#endif
	
	gpio_direction_output(MFP2GPIO(BT_WAKEUP), GPIO_LEVEL_LOW);
	msleep(50);
	//msleep(100);
	//gpio_direction_output(MFP2GPIO(BT_RTS), GPIO_LEVEL_LOW);
}
EXPORT_SYMBOL_GPL(set_dsr_on);

int set_dsr_off(int index)
{
	/*make sure GH381 GPIO5 high*/
	int value=0;
	value=gpio_get_value(BT_WAKEUP_HOST);
	if(value==0)
		return 1;
	//gpio_direction_output(MFP2GPIO(BT_RTS), GPIO_LEVEL_HIGH);
	//msleep(100);
	gpio_direction_output(MFP2GPIO(BT_WAKEUP), GPIO_LEVEL_HIGH);
	msleep(50);
#ifdef CONFIG_PXA3xx_MODEM_DVFM
	//dvfm_enable_op_name_aync("D0CS", index);
	//dvfm_enable_op_name_aync("104M", index);
	//if(mspmst){
	//	mspm_enable();
	//}
#endif
	return 0;
	//gpio_direction_output(MFP2GPIO(BT_RTS), GPIO_LEVEL_HIGH);
}
EXPORT_SYMBOL_GPL(set_dsr_off);
static void btpindown(void)
{
	gpio_direction_output(MFP2GPIO(WLAN_PD), GPIO_LEVEL_LOW);
	//pxa3xx_mfp_set_lpm(MFP_WLAN8686_PWDN_GPIO, MFP_LPM_PULL_HIGH);
	gpio_direction_output(MFP2GPIO(WLAN_PWREN), GPIO_LEVEL_LOW);
	//pxa3xx_mfp_set_lpm(MFP_PIN_GPIO1, MFP_LPM_PULL_HIGH);
	//gpio_direction_output(MFP2GPIO(WLAN_RST), GPIO_LEVEL_LOW);
}
extern void enable_oscc_tout_s0(void);
extern void disable_oscc_tout_s0(void);
static void initbtpin(void)
{	
	pxa3xx_mfp_set_afds(WLAN_PD, MFP_AF0, MFP_DS03X);
	pxa3xx_mfp_set_afds(WLAN_PWREN, MFP_AF0, MFP_DS03X);
	pxa3xx_mfp_set_afds(WLAN_RST, MFP_AF0, MFP_DS03X);
	//pxa3xx_mfp_set_afds(WLAN_WAKE, MFP_AF0, MFP_DS03X);
	pxa3xx_mfp_set_afds(BT_WAKEUP, MFP_AF0, MFP_DS03X);
	pxa3xx_mfp_set_afds(BT_WAKEUP_HOST, MFP_AF0, MFP_DS04X);
	//pxa3xx_mfp_set_afds(BT_RTS, MFP_AF0, MFP_DS03X);

	//gpio_direction_output(MFP2GPIO(BT_RTS), GPIO_LEVEL_LOW);
	//pxa3xx_mfp_set_lpm(BT_RTS, MFP_LPM_DRIVE_HIGH);
	//pxa3xx_mfp_set_rdh(BT_RTS, 1);
	enable_oscc_tout_s0();
	gpio_direction_input(BT_WAKEUP_HOST);
	//gpio_direction_output(BT_WAKEUP_HOST, GPIO_LEVEL_HIGH);
#if defined(CONFIG_BOARD_X2_V4)
	pxa3xx_mfp_set_afds(BT_SW, MFP_AF0, MFP_DS03X);
#if defined(CONFIG_BT_REL)
	gpio_direction_output(MFP2GPIO(BT_SW), GPIO_LEVEL_LOW);//make uart-->bt
	pxa3xx_mfp_set_lpm(BT_SW, MFP_LPM_DRIVE_LOW);
	pxa3xx_mfp_set_rdh(BT_SW, 1);
#else
	gpio_direction_output(MFP2GPIO(BT_SW), GPIO_LEVEL_HIGH);//make uart-->bt
#endif
#endif

	//gpio_direction_output(MFP2GPIO(BT_WAKEUP), GPIO_LEVEL_LOW);
	pxa3xx_mfp_set_lpm(BT_WAKEUP, MFP_LPM_DRIVE_HIGH);
	//pxa3xx_mfp_set_rdh(BT_WAKEUP, 1);
	gpio_direction_output(MFP2GPIO(BT_WAKEUP), GPIO_LEVEL_LOW);

	gpio_direction_output(MFP2GPIO(WLAN_PD), GPIO_LEVEL_HIGH);
	pxa3xx_mfp_set_lpm(WLAN_PD, MFP_LPM_DRIVE_HIGH);
	pxa3xx_mfp_set_rdh(WLAN_PD, 1);
	
	/*WLAN_PWREN*/
	gpio_direction_output(MFP2GPIO(WLAN_PWREN), GPIO_LEVEL_HIGH);
	pxa3xx_mfp_set_lpm(WLAN_PWREN, MFP_LPM_DRIVE_HIGH);
	pxa3xx_mfp_set_rdh(WLAN_PWREN, 1);
	msleep(5);
	
	/*WLAN_RST*/
	gpio_direction_output(MFP2GPIO(WLAN_RST), GPIO_LEVEL_LOW);
	msleep(5);
	gpio_direction_output(MFP2GPIO(WLAN_RST), GPIO_LEVEL_HIGH);
	pxa3xx_mfp_set_lpm(WLAN_RST, MFP_LPM_DRIVE_HIGH);
	pxa3xx_mfp_set_rdh(WLAN_RST, 1);
	//gpio_direction_input(BT_WAKEUP_HOST);

	pxa3xx_mfp_set_afds(BT_RTS,MFP_AF1, MFP_DS04X);
	gpio_direction_output(BT_RTS, GPIO_LEVEL_HIGH);
	pxa3xx_mfp_set_lpm(BT_RTS, MFP_LPM_DRIVE_HIGH);
	
}

static void wlan_8xxx_power_on(void)
{
	printk(KERN_INFO "wlan_power_on");
	initbtpin();
	return;
}

static void wlan_8xxx_power_off(void)
{
	printk(KERN_INFO "wlan_power_off");
	//btpindown();
	return;
}

static int littleton_mci_init(struct device *dev,
			     irq_handler_t littleton_detect_int,
			     void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	int err, cd_irq, gpio_cd;

	cd_irq = gpio_to_irq(littleton_mmc_slot[pdev->id].gpio_cd);
	gpio_cd = littleton_mmc_slot[pdev->id].gpio_cd;

	/*
	 * setup GPIO for littleton MMC controller
	 */
	#if 0
	err = gpio_request(gpio_cd, "mmc card detect");
	if (err)
		goto err_request_cd;
	gpio_direction_input(gpio_cd);

	err = request_irq(cd_irq, littleton_detect_int,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "MMC card detect", data);
	if (err) {
		printk(KERN_ERR "%s: MMC/SD/SDIO: "
				"can't request card detect IRQ\n", __func__);
		goto err_request_irq;
	}
	#endif

	if(pdev->id==2)//mmc3
		wlan_8xxx_power_on();
	
	return 0;

err_request_irq:
	gpio_free(gpio_cd);
err_request_cd:
	return err;
}

static void littleton_mci_exit(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	int cd_irq, gpio_cd;

	wlan_8xxx_power_off();

	cd_irq = gpio_to_irq(littleton_mmc_slot[pdev->id].gpio_cd);
	gpio_cd = littleton_mmc_slot[pdev->id].gpio_cd;

	free_irq(cd_irq, data);
	gpio_free(gpio_cd);
}

static struct pxamci_platform_data littleton_mci_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
	.init 		= littleton_mci_init,
	.exit		= littleton_mci_exit,
};

static struct pxamci_platform_data littleton_mci2_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
};

static void __init littleton_init_mmc(void)
{
	pxa_set_mci_info(&littleton_mci_platform_data);
	pxa3xx_set_mci2_info(&littleton_mci2_platform_data);
#if defined(CONFIG_MMC3)
	pxa3xx_set_mci3_info(&littleton_mci_platform_data);
#endif
}
#else
static inline void littleton_init_mmc(void) {}
#endif
#ifdef CONFIG_PM
static int littleton_init_wakeup(pm_wakeup_src_t *src)
{
	memset(src, 0, sizeof(pm_wakeup_src_t));
	src->bits.rtc = 1;
	/*diable rtc wakeup.no use*/
	src->bits.ost = 1;
	src->bits.ext0 = 1;
#if !defined(CONFIG_BOARD_X2_V4)
	src->bits.uart2 = 1;
#else
	src->bits.uart3 = 1;
#endif
	src->bits.uart1 = 1; /*enable modem FFUART wakeup*/
	src->bits.mkey = 1;
#if defined(CONFIG_CHARGER_PXA3xx_HARDWARE) /* chg detect-gpio112 */
	src->bits.dkey = 1; 
#endif
	return 0;
}

static int littleton_query_wakeup(unsigned int reg, pm_wakeup_src_t *src)
{
	memset(src, 0, sizeof(pm_wakeup_src_t));
	if (reg & PXA3xx_PM_WE_RTC)
		src->bits.rtc = 1;
	if (reg & PXA3xx_PM_WE_OST)
		src->bits.ost = 1;
	if (reg & PXA3xx_PM_WE_MSL0)
		src->bits.msl = 1;
	if (reg & PXA3xx_PM_WE_EXTERNAL0)
	        src->bits.ext0 = 1;
	if (reg & PXA3xx_PM_WE_GENERIC(3))
		src->bits.uart1 = 1;
	if (reg & PXA3xx_PM_WE_GENERIC(4))
		src->bits.uart2 = 1;
	if (reg & PXA3xx_PM_WE_GENERIC(5))
		src->bits.uart3 = 1;
	if (reg & PXA3xx_PM_WE_GENERIC(6))
	        src->bits.mkey = 1;
	if (reg & PXA3xx_PM_WE_KP)
	        src->bits.dkey = 1;
#if defined(CONFIG_DUAL_MODEM) /* for BB_HOST_WAKE_2 */ 
	if (reg & PXA3xx_PM_WE_GENERIC(10))
		src->bits.uart1 = 1;
#endif	
	
	return 0;
}

static int littleton_ext_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.ext0)
			ret |= PXA3xx_PM_WE_EXTERNAL0;
		if (src.bits.ext1)
			ret |= PXA3xx_PM_WE_EXTERNAL1;
	}
	return ret;
}

static int littleton_key_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.mkey) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO115_KP_MKIN_0), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO116_KP_MKIN_1), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO117_KP_MKIN_2), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO118_KP_MKIN_3), MFP_EDGE_BOTH);		
			ret |= PXA3xx_PM_WE_GENERIC(6);
		}
		if (src.bits.dkey) {
			//pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO107_KP_DKIN_0), MFP_EDGE_BOTH);
			//pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO108_KP_DKIN_1), MFP_EDGE_BOTH);
#if defined(CONFIG_CHARGER_PXA3xx_HARDWARE)
			pxa3xx_mfp_set_afds(MFP_CFG_PIN(GPIO111_KP_DKIN_4), MFP_AF2, MFP_DS04X);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO111_KP_DKIN_4), MFP_EDGE_BOTH);
#endif
			ret |= PXA3xx_PM_WE_KP;
		}
	} else {
		if (src.bits.mkey) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO115_KP_MKIN_0), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO116_KP_MKIN_1), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO117_KP_MKIN_2), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO118_KP_MKIN_3), MFP_EDGE_NONE);
		}
		if (src.bits.dkey) {
			//pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO107_KP_DKIN_0), MFP_EDGE_NONE);
			//pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO108_KP_DKIN_1), MFP_EDGE_NONE);
#if defined(CONFIG_CHARGER_PXA3xx_HARDWARE)
			pxa3xx_mfp_set_afds(MFP_CFG_PIN(GPIO111_GPIO), MFP_AF0, MFP_DS04X);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO111_GPIO), MFP_EDGE_NONE);
#endif
		}
	}
	return ret;
}

static int littleton_mmc_wakeup(pm_wakeup_src_t src, int enable)
{
	return 0;
}

static int littleton_uart_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.uart1) {
#if defined(CONFIG_BOARD_BRAVA) /* gpio83 gpio wake */
			pxa3xx_mfp_set_afds(MFP_PIN_GPIO83, MFP_AF0, MFP_DS03X); /*DKIN2 */
			pxa3xx_mfp_set_edge(MFP_PIN_GPIO83, MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(12);
#else
			pxa3xx_mfp_set_afds(MFP_CFG_PIN(GPIO81_UART1_DSR), MFP_AF1, MFP_DS04X);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO81_UART1_DSR), MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(3);
#endif

#if defined(CONFIG_DUAL_MODEM) /* for BB_HOST_WAKE_2 */ 
			pxa3xx_mfp_set_afds(MFP_CFG_PIN(GPIO29_MMC1_DAT0), MFP_AF3, MFP_DS04X);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO29_MMC1_DAT0), MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(10);
#endif

			/* add PMIC nIRQ to wake up source */
			pxa3xx_mfp_set_afds(MFP_PMIC_INT, MFP_PMIC_WAKE_AF, MFP_DS03X);
			pxa3xx_mfp_set_edge(MFP_PMIC_INT, MFP_EDGE_BOTH);
		}
		if (src.bits.uart2) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO112_UART2_RXD), MFP_EDGE_BOTH);
			#if 0
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO114_UART2_CTS), MFP_EDGE_BOTH);
			#endif
			ret |= PXA3xx_PM_WE_GENERIC(4);
		}
		if (src.bits.uart3) {
			pxa3xx_mfp_set_afds(BT_WAKEUP_HOST, MFP_AF2, MFP_DS04X);
			pxa3xx_mfp_set_edge(BT_WAKEUP_HOST, MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(5);
		}
	} else {
		if (src.bits.uart1) {
#if defined(CONFIG_BOARD_BRAVA)
			pxa3xx_mfp_set_afds(MFP_PIN_GPIO83, MFP_AF0, MFP_DS03X);
			pxa3xx_mfp_set_edge(MFP_PIN_GPIO83, MFP_EDGE_NONE);
#else
			pxa3xx_mfp_set_afds(MFP_CFG_PIN(GPIO81_UART1_DSR), MFP_AF0, MFP_DS04X);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO81_UART1_DSR), MFP_EDGE_NONE);
			/*restore the DSR to GPIO mode,ensure we can get the interrupt*/
#endif

#if defined(CONFIG_DUAL_MODEM) /* for BB_HOST_WAKE_2 */ 
			pxa3xx_mfp_set_afds(MFP_CFG_PIN(GPIO29_MMC1_DAT0), MFP_AF0, MFP_DS04X);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO29_MMC1_DAT0), MFP_EDGE_NONE);
#endif

			pxa3xx_mfp_set_edge(MFP_PMIC_INT, MFP_EDGE_NONE);
			micco_init_irq();
		}
		if (src.bits.uart2) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO112_UART2_RXD), MFP_EDGE_NONE);
			#if 0	
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO114_UART2_CTS), MFP_EDGE_NONE);
			#endif
		}
		if (src.bits.uart3) {
			pxa3xx_mfp_set_edge(BT_WAKEUP_HOST, MFP_EDGE_NONE);
			pxa3xx_mfp_set_afds(BT_WAKEUP_HOST, MFP_AF0, MFP_DS04X);
		}
	}
	return ret;
}

static struct pxa3xx_peripheral_wakeup_ops wakeup_ops = {
	.init	= littleton_init_wakeup,
	.query	= littleton_query_wakeup,
	.ext    = littleton_ext_wakeup,
	.key    = littleton_key_wakeup,
	.mmc    = littleton_mmc_wakeup,
	.uart   = littleton_uart_wakeup,
};
#endif

static void disable_unused_clock(void)
{
	CKENB &= ~(CKEN_PWM0|CKEN_PWM1|CKEN_MINI_IM|CKEN_MINI_LCD|
				CKEN_1WIRE|CKEN_HSIOGCU|CKEN_MVED);
	CKENA &= ~ (CKEN_USIM0 | CKEN_USIM1);
}

static void __init littleton_init(void)
{
	disable_unused_clock(); /* Disable the unused clock as all CKENB enable default */
	
	/* initialize MFP configurations */
	pxa3xx_mfp_config(ARRAY_AND_SIZE(littleton_mfp_cfg));

	/* dvfm device */
	set_pxa3xx_freq_info(&littleton_freq_mach_info);

	/* performance monitor unit */
	pxa3xx_set_pmu_info(NULL);

	/*
	 * Note: we depend bootloader set the correct
	 * value to MSC register for SMC91x.
	 */
	platform_add_devices(devices, ARRAY_SIZE(devices));

	/* littleton_init_spi(); */

	/* uart */
/*	pxa_set_ffuart_info(NULL);
	pxa_set_btuart_info(NULL);
	pxa_set_stuart_info(NULL);*/
	
	littleton_init_ohci();

	littleton_init_nand();

#if defined(CONFIG_PXA_CAMERA)
	cam_host_init();
#endif
	i2c_register_board_info(0, ARRAY_AND_SIZE(littleton_i2c_board_info));

#if defined(CONFIG_USB_PXA3XX_U2D) || defined(CONFIG_USB_USB_PXA3XX_U2D_MODULE)
	/* u2d */
	pxa3xx_mfp_config(ARRAY_AND_SIZE(littleton_u2d_cfg));
	pxa_set_u2d_info(&littleton_u2d_info);
#elif defined(CONFIG_USB_PXA27X_UDC) || defined(CONFIG_USB_USB_PXA27X_UDC_MODULE)
	pxa_set_udc_info(NULL);
#endif

	littleton_init_keypad();

	/* MMC card detect & write protect for controller 0 */
	littleton_mmc_slot[0].gpio_cd  = mfp_to_gpio(MFP_PIN_GPIO15);
	littleton_init_mmc();
#ifdef CONFIG_PM
	pxa3xx_wakeup_register(&wakeup_ops);
#endif
}

//MACHINE_START(LITTLETON, "Marvell Form Factor Development Platform (aka Littleton)")
MACHINE_START(LITTLETON, "yuhua "YUHUA_BOARD_NAME " on " "Marvell (Littleton)")
	.phys_io	= 0x40000000,
	.boot_params	= 0xa0000100,
	.io_pg_offst	= (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq	= pxa3xx_init_irq,
	.timer		= &pxa_timer,
	.init_machine	= littleton_init,
MACHINE_END
