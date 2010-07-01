/*
 *  linux/arch/arm/mach-pxa/saar.c
 *
 *  Support for the Marvell SAAR Development Platform.
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
#include <linux/mtd/partitions.h>
#include <mtd/mtd-abi.h>
#include <linux/irq.h>
#include <linux/i2c/si4703.h>

#include <asm/types.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach/flash.h>

#include <mach/pxa-regs.h>
#include <mach/mfp-pxa9xx.h>
#include <mach/mfp-pxa3xx.h>
#include <mach/pxa3xx_nand.h>
#include <mach/pxa27x_keypad.h>
#include <mach/pxa3xx_pmic.h>
#include <mach/gpio.h>
#include <mach/micco.h>
#include <mach/saar.h>
#include <mach/irda.h>
#include <mach/pxa3xx_u2d.h>
#include <mach/pxa9xx_u2o.h>
#include <mach/ehci.h>
#include <mach/udc.h>
#include <mach/uart.h>
#include <mach/mmc.h>
#include <mach/pxa3xx_dvfm.h>
#include <mach/imm.h>
#include <mach/pxafb.h>
#include <mach/pxa3xx_pm.h>
#include <mach/pxa930_trkball.h>
#include <mach/camera.h>
#include <mach/pmu.h>
#include <mach/part_table.h>
#include <mach/pxa3xx-regs.h>

#include "devices.h"
#include "generic.h"

#define MAX_SLOTS      3
struct platform_mmc_slot saar_mmc_slot[MAX_SLOTS];
extern int is_android(void);
extern int is_comm_v75(void);

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

#ifdef CONFIG_MACH_SAAR_PIN_MUX
/* SAAR MFP configurations */
static mfp_cfg_t saar_mfp_cfg[] __initdata = {
	/* DFC */
	DF_INT_RnB_ND_INT_RnB,
	DF_nRE_nOE_ND_nRE,
	DF_nWE_ND_nWE,
	DF_CLE_nOE_ND_CLE,
	DF_nADV1_ALE_ND_ALE,
	DF_nADV2_ALE_nCS3,
	DF_nCS0_ND_nCS0,
	DF_IO0_ND_IO0,
	DF_IO1_ND_IO1,
	DF_IO2_ND_IO2,
	DF_IO3_ND_IO3,
	DF_IO4_ND_IO4,
	DF_IO5_ND_IO5,
	DF_IO6_ND_IO6,
	DF_IO7_ND_IO7,
	DF_IO8_ND_IO8,
	DF_IO9_ND_IO9,
	DF_IO10_ND_IO10,
	DF_IO11_ND_IO11,
	DF_IO12_ND_IO12,
	DF_IO13_ND_IO13,
	DF_IO14_ND_IO14,
	DF_IO15_ND_IO15,

	/* FFUART */
	GPIO53_FFRXD,
	GPIO54_FFTXD,

	/* STUART */
	GPIO45_STRXD,
	GPIO46_STTXD,

 	/* ethernet */
	GPIO48_ETHER_nRD,
	GPIO49_ETHER_nWR,
	GPIO45_ETHER_nAE,
	GPIO59_ETHER_CP,
	GPIO62_ETHER_ARDY,
	GPIO60_ETHER_DQM0,
	GPIO61_ETHER_DQM1,
	GPIO97_ETHER_IRQ,
 

	/* Keypad*/
	GPIO0_KP_MKIN_0,
	GPIO1_KP_MKOUT_0,
	GPIO2_KP_MKIN_1,
	GPIO3_KP_MKOUT_1,
	GPIO4_KP_MKIN_2,
	GPIO5_KP_MKOUT_2,
	GPIO6_KP_MKIN_3,
	GPIO7_KP_MKOUT_3,
	GPIO8_KP_MKIN_4,
	GPIO9_KP_MKOUT_4,

	/* i2c bus */
	GPIO89_CI2C_SCL,
	GPIO90_CI2C_SDA,

	/* Micco interrupt */
	PMIC_INT_GPIO_83,
	/* avoid GPIO83 confliction */
	GPIO83_SSP3_RXD,

	/* STEREO_AMP_EN */
	GPIO84_GPIO,

	/* GSSP1 */
	GPIO79_GSSP1_CLK,
	GPIO80_GSSP1_FRM,
	GPIO81_GSSP1_TXD,
	GPIO82_GSSP1_RXD,

	/* SSP2 */
	GPIO85_SSP2_BITCLK,
	GPIO87_SSP2_SYNC,
	GPIO88_SSP2_DATA_OUT,

	/* QCI */
	GPIO65_CI_DD_7,
	GPIO66_CI_DD_6,
	GPIO67_CI_DD_5,
	GPIO68_CI_DD_4,
	GPIO69_CI_DD_3,
	GPIO70_CI_DD_2,
	GPIO71_CI_DD_1,
	GPIO72_CI_DD_0,
	GPIO75_CI_MCLK,
	GPIO76_CI_PCLK,

	GPIO77_GPIO,
	GPIO78_GPIO,

	/* MMC1 */
	GPIO20_GPIO,/* card dectect */

	/* MMC2 */
	GPIO101_MMC2_DAT3,
	GPIO102_MMC2_DAT2,
	GPIO103_MMC2_DAT1,
	GPIO104_MMC2_DAT0,
	GPIO105_MMC2_CMD,
	GPIO106_MMC2_CLK,

	/* LCD */
	GPIO23_LCD_DD0,
	GPIO24_LCD_DD1,
	GPIO25_LCD_DD2,
	GPIO26_LCD_DD3,
	GPIO27_LCD_DD4,
	GPIO28_LCD_DD5,
	GPIO29_LCD_DD6,
	GPIO44_LCD_DD7,
	GPIO21_LCD_CS,
	GPIO22_LCD_VSYNC,
	GPIO17_LCD_FCLK_RD,
	GPIO18_LCD_LCLK_A0,
	GPIO19_LCD_PCLK_WR,
	GPIO16_GPIO, /* LCD reset */

	/* 1 wire */
	GPIO95_OW_DQ_IN,

	/* Track Ball */
	GPIO47_GPIO,
	GPIO48_GPIO,
	GPIO49_GPIO,
	GPIO50_GPIO,

	/* si4703 FM radio */
	GPIO86_GPIO, /* reset */
	GPIO96_GPIO, /* interrupt */

	/* BT UART */
	GPIO91_BTRXD,
	GPIO92_BTTXD,
	GPIO93_BTCTS_N,
	GPIO94_BTRTS_N,

	/* WLAN */
	GPIO98_WLAN_PD,
	GPIO99_WLAN_BT_RESET,
	GPIO100_WLAN_WAKEUP,
};

static mfp_cfg_t saar25_mfp_cfg[] __initdata = {
	/* IRDA */
	GPIO32_IRDA_SHDN,

	/* MMC1 */
	GPIO10_MMC1_DAT3,
	GPIO11_MMC1_DAT2,
	GPIO12_MMC1_DAT1,
	GPIO13_MMC1_DAT0,
	GPIO14_MMC1_CMD,
	GPIO15_MMC1_CLK,
};

static mfp_cfg_t saar26_mfp_cfg[] __initdata = {
	/* IRDA */
	GPIO62_IRDA_SHDN,

	/* MMC1 */
	GPIO55_MMC1_CMD,
	GPIO56_MMC1_CLK,
	GPIO57_MMC1_DAT0,
	GPIO58_MMC1_DAT1,
	GPIO59_MMC1_DAT2,
	GPIO60_MMC1_DAT3,
};
#endif

#if defined(CONFIG_MOUSE_PXA930_TRKBALL) || \
	defined(CONFIG_MOUSE_PXA930_TRKBALL_MODULE)
static struct pxa930_trkball_platform_data saar_mouse_info = {
	.x_filter	= 2,
	.y_filter	= 2,
};

static void __init saar_init_trackball(void)
{
	pxa_register_device(&pxa930_device_trkball, &saar_mouse_info);
}
#else
static inline void saar_init_trackball(void) {}
#endif

static struct resource smc91x_resources[] = {
	[0] = {
		.start	= (SAAR_ETH_PHYS + 0x300),
		.end	= (SAAR_ETH_PHYS + 0xfffff),
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO97)),
		.end	= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO97)),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,

};

static struct platform_device micco_bl_device = {
	.name 		= "micco-bl",
	.id 		= -1,
};

static struct platform_device micco_kp_bl_device = {
	.name 		= "micco-kp-bl",
	.id 		= -1,
};

static struct platform_device micco_ts_device = {
	.name 		= "micco-ts",
	.id 		= -1,
};

static struct resource pxa3xx_resource_imm[] = {
	[0] = {
		.name   = "phy_sram",
		.start	= PHYS_SRAM_START + 0x10000, /* Moved up to offset 64KB due to collision with DVFM
						  * D2 entry code +
						  * The first 4KB are locked for pxa930 trusted */
		.end	= PHYS_SRAM_START + PHYS_SRAM_BANK_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name   = "imm_sram",
		.start	= PHYS_SRAM_START + 0x10000,
		.end	= PHYS_SRAM_START + PHYS_SRAM_BANK_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource pxa935_resource_imm[] = {
	[0] = {
		.name   = "phy_sram",
		.start	= PHYS_SRAM_START + 0x10000, /* Moved up to offset 64KB due to collision with DVFM
						  * D2 entry code */
		.end	= PHYS_SRAM_START + 2 * PHYS_SRAM_BANK_SIZE - 1, /* 2 banks on pxa935 */
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name   = "imm_sram",
		.start	= PHYS_SRAM_START + 0x10000,
		.end	= PHYS_SRAM_START + 2 * PHYS_SRAM_BANK_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
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

static struct platform_device micco_hsdetect_device = {
	.name		= "micco-hsdetect",
	.id		= -1,
};

static struct platform_device ispt_device = {
	.name		= "pxa-ispt",
	.id		= -1,
};

static struct platform_device sfh7225_nled_device = {
	.name 		= "sfh7225_nled",
	.id		= -1,
};

static struct platform_device sfh7225_ls_device = {
	.name           = "sfh7225_ls",
	.id             = -1,
};

static struct platform_device *devices[] __initdata = {
	&micco_bl_device,
	&micco_kp_bl_device,
	&micco_ts_device,
	&pxa3xx_device_imm,
	&micco_charger_device,
	&micco_hsdetect_device,
	&ispt_device,
	&sfh7225_nled_device,
	&sfh7225_ls_device,
};

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
static unsigned int saar_matrix_key_map[] = {
	/* KEY(row, col, key_code) */
	KEY(1, 3, KEY_0), KEY(0, 0, KEY_1), KEY(1, 0, KEY_2), KEY(2, 0, KEY_3),
	KEY(0, 1, KEY_4), KEY(1, 1, KEY_5), KEY(2, 1, KEY_6), KEY(0, 2, KEY_7),
	KEY(1, 2, KEY_8), KEY(2, 2, KEY_9),

	KEY(0, 3, KEY_KPASTERISK), 	/* * */
	KEY(2, 3, KEY_KPDOT), 		/* # */

	KEY(4, 0, KEY_HOME),
	KEY(3, 3, KEY_END),
	KEY(4, 1, KEY_BACK),

	KEY(0, 4, KEY_ENTER),		/* scroll push */

	KEY(3, 2, KEY_SEND),
	KEY(4, 2, KEY_VOLUMEUP),
	KEY(4, 3, KEY_VOLUMEDOWN),

	KEY(3, 0, KEY_F22),	/* soft1 */
	KEY(3, 1, KEY_F23),	/* soft2 */

	KEY(1, 4, KEY_CAMERA),
};

static struct pxa27x_keypad_platform_data saar_keypad_info = {
	.matrix_key_rows	= 5,
	.matrix_key_cols	= 5,
	.matrix_key_map		= saar_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(saar_matrix_key_map),
	.debounce_interval	= 30,
};
#endif /* CONFIG_KEYBOARD_PXA27x || CONFIG_KEYBOARD_PXA27x_MODULE */

#ifdef CONFIG_ISPT
static mfp_cfg_t pxa930_ispt_pins[] __initdata = {
	/* SSP4 */
	GPIO51_SSP4_CLK,
	GPIO52_SSP4_FRM,
	GPIO45_SSP4_TXD,
};

static void __init saar_init_ispt(void)
{
	pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa930_ispt_pins));
}
#endif

#if defined(CONFIG_MMC) || defined(CONFIG_MMC_MODULE)
static int saar_mci_init(struct device *dev,
			     irq_handler_t saar_detect_int,
			     void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	int err, cd_irq, gpio_cd;

	cd_irq = gpio_to_irq(saar_mmc_slot[pdev->id].gpio_cd);
	gpio_cd = saar_mmc_slot[pdev->id].gpio_cd;

	/*
	 * setup GPIO for saar MMC controller
	 */
	err = gpio_request(gpio_cd, "mmc card detect");
	if (err)
		goto err_request_cd;
	gpio_direction_input(gpio_cd);

	err = request_irq(cd_irq, saar_detect_int,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "MMC card detect", data);
	if (err) {
		printk(KERN_ERR "%s: MMC/SD/SDIO: "
				"can't request card detect IRQ\n", __func__);
		goto err_request_irq;
	}

	return 0;

err_request_irq:
	gpio_free(gpio_cd);
err_request_cd:
	return err;
}

static void saar_mci_exit(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	int cd_irq, gpio_cd;

	cd_irq = gpio_to_irq(saar_mmc_slot[pdev->id].gpio_cd);
	gpio_cd = saar_mmc_slot[pdev->id].gpio_cd;

	free_irq(cd_irq, data);
	gpio_free(gpio_cd);
}

static struct pxamci_platform_data saar_mci_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
	.init 		= saar_mci_init,
	.exit		= saar_mci_exit,
};

static struct pxamci_platform_data saar_mci2_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
};

static mfp_cfg_t pxa9xx_mmc_mfp_cfg[] __initdata = {
	/* MMC4 of Tavor PV */
	GPIO55_MMC4_CMD,
	GPIO56_MMC4_CLK,
	GPIO57_MMC4_DAT0,
	GPIO58_MMC4_DAT1,
	GPIO59_MMC4_DAT2,
	GPIO60_MMC4_DAT3,

	/* MMC5 of Tavor PV */
	GPIO101_MMC5_DAT3,
	GPIO102_MMC5_DAT2,
	GPIO103_MMC5_DAT1,
	GPIO104_MMC5_DAT0,
	GPIO105_MMC5_CMD,
	GPIO106_MMC5_CLK,
};

static void __init saar_init_mmc(void)
{
	if (device_is_enabled(device_mmc)) {
		pxa_set_mci_info(&saar_mci_platform_data);
		pxa3xx_set_mci2_info(&saar_mci2_platform_data);
		/* MMC card detect for controller 0 of pxamci */
		saar_mmc_slot[0].gpio_cd  = mfp_to_gpio(MFP_PIN_GPIO20);
	}
	if (device_is_enabled(device_mmc_new)) {
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa9xx_mmc_mfp_cfg));
		pxa9xx_set_mci4_info(&saar_mci_platform_data);
		pxa9xx_set_mci5_info(&saar_mci2_platform_data);
		/* MMC card detect for controller 1 of pxa9xx_mci */
		saar_mmc_slot[1].gpio_cd  = mfp_to_gpio(MFP_PIN_GPIO20);
	}
}
#else
static inline void saar_init_mmc(void) {}
#endif

#if defined(CONFIG_MTD_ONENAND) || defined(CONFIG_MTD_ONENAND_MODULE)
static struct flash_platform_data saar_onenand_info;
static void __init saar_init_onenand(void)
{
	if (is_android()) {
		if (is_comm_v75()) {
			saar_onenand_info.parts = android_256m_v75_partitions;
			saar_onenand_info.nr_parts = ARRAY_SIZE(android_256m_v75_partitions);
		} else {
			saar_onenand_info.parts = android_256m_partitions;
			saar_onenand_info.nr_parts = ARRAY_SIZE(android_256m_partitions);
		}
	} else {
		if (is_comm_v75()) {
			saar_onenand_info.parts = pxa930_256m_v75_partitions;
			saar_onenand_info.nr_parts = ARRAY_SIZE(pxa930_256m_v75_partitions);			
		} else {
			saar_onenand_info.parts = pxa930_256m_partitions;
			saar_onenand_info.nr_parts = ARRAY_SIZE(pxa930_256m_partitions);
		}
	}
		
	pxa3xx_device_onenand.dev.platform_data = &saar_onenand_info;
	platform_device_register(&pxa3xx_device_onenand);
}
#else
static inline void saar_init_onenand(void) {}
#endif

#if defined(CONFIG_MTD_NAND_PXA3xx) || defined(CONFIG_MTD_NAND_PXA3xx_MODULE)
static struct pxa3xx_nand_platform_data saar_nand_info;
static void __init saar_init_nand(void)
{
	if (is_android()) {
		saar_nand_info.parts = android_128m_partitions;
		saar_nand_info.nr_parts = ARRAY_SIZE(android_128m_partitions);
	} else {
		if (is_comm_v75()) {
			saar_nand_info.parts = pxa930_128m_v75_partitions;
			saar_nand_info.nr_parts = ARRAY_SIZE(pxa930_128m_v75_partitions);
		} else {
			saar_nand_info.parts = pxa930_128m_partitions;
			saar_nand_info.nr_parts = ARRAY_SIZE(pxa930_128m_partitions);
		}
	}

	pxa3xx_device_nand.dev.platform_data = &saar_nand_info;
	platform_device_register(&pxa3xx_device_nand);
}
#else
static inline void saar_init_nand(void) {}
#endif /* CONFIG_MTD_NAND_PXA3xx || CONFIG_MTD_NAND_PXA3xx_MODULE */

#if defined(CONFIG_USB_PXA27X_UDC) || defined(CONFIG_USB_PXA27X_UDC_MODULE)
static mfp_cfg_t pxa930_otg_init_pins[] = {
	GPIO64_GPIO,
};

static mfp_cfg_t pxa930_otg_pins[] = {
	GPIO64_USB_P2_7,
};

int saar_udc_is_miniA(void)
{
	int otg_id = mfp_to_gpio(MFP_PIN_GPIO64);
	int id_value;
	int err;

	pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa930_otg_init_pins));
	err = gpio_request(otg_id, "OTG ID");
	if (err) {
		gpio_free(otg_id);
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d return :%d\n", otg_id, err);
		return 0;
	}
	gpio_direction_input(otg_id);
	id_value = gpio_get_value(otg_id);
	gpio_free(otg_id);
	pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa930_otg_pins));

	return (id_value == 0);
}

static struct pxa2xx_udc_mach_info saar_udc_info = {
	.udc_is_miniA = saar_udc_is_miniA,
};
#endif

#if defined(CONFIG_USB_PXA3XX_U2D) || defined(CONFIG_USB_USB_PXA3XX_U2D_MODULE)
#ifdef CONFIG_MACH_SAAR_PIN_MUX
static mfp_cfg_t saar_u2d_cfg[] __initdata = {
	/* ULPI*/
	GPIO43_GPIO,
	GPIO31_USB_ULPI_D0,
	GPIO30_USB_ULPI_D1,
	GPIO33_USB_ULPI_D2,
	GPIO34_USB_ULPI_D3,
	GPIO35_USB_ULPI_D4,
	GPIO36_USB_ULPI_D5,
	GPIO41_USB_ULPI_D6,
	GPIO42_USB_ULPI_D7,
	GPIO37_USB_ULPI_DIR,
	GPIO38_USB_ULPI_CLK,
	GPIO39_USB_ULPI_STP,
	GPIO40_USB_ULPI_NXT,
	DF_ADDR3_CLK26MOUTDMD,
};
#endif

void saar_reset_xcvr(void)
{
	int reset_pin;
	int err;

	reset_pin = mfp_to_gpio(MFP_PIN_GPIO43);
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

	gpio_free(reset_pin);
}

static struct pxa3xx_u2d_mach_info saar_u2d_info = {
	.reset_xcvr = saar_reset_xcvr,
};
#endif /* CONFIG_USB_PXA3XX_U2D || CONFIG_USB_PXA3XX_U2D_MODULE */

#if defined(CONFIG_USB_PXA9XX_U2O) || defined(CONFIG_USB_PXA9XX_U2O_MODULE)
extern void pxa9xx_u2o_xcvr_init(void);
static struct pxa9xx_u2o_mach_info saar_u2o_info = {
	.xcvr_init = pxa9xx_u2o_xcvr_init,
};
#endif

#ifdef CONFIG_USB_EHCI_PXA9XX
static struct pxaehci_platform_data saar_ehci_info = {
/*	.port_mode	= PMM_PERPORT_MODE,
 */
};

static void __init saar_init_ehci(void)
{
	if (!device_is_enabled(device_u2o)) 
		return;

	pxa_set_ehci_info(&saar_ehci_info);
}
#else
static inline void saar_init_ehci(void) {}
#endif

#if defined(CONFIG_PXA3xx_MICCO) || defined(CONFIG_PXA3xx_MICCO_MODULE)
static int micco_init_irq(void)
{
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO83));

	return 0;
}

static int micco_ack_irq(void)
{
	return 0;
}

static void saar_micco_init(void)
{
	u8 value;

	/* Mask interrupts that are not needed */
	micco_write(MICCO_IRQ_MASK_A, 0xFE);
	micco_write(MICCO_IRQ_MASK_B, 0xFF);
	micco_write(MICCO_IRQ_MASK_C, 0xFF);
	micco_write(MICCO_IRQ_MASK_D, 0xFF);

	/* avoid SRAM power off during sleep*/
	micco_write(0x10, 0x07);
	micco_write(0x11, 0xff);
	micco_write(0x12, 0xbf); /*never enable LDO4: CP controls it via COMM_OVER3 reg (0x22)*/

	/* Enable the ONKEY power down functionality */
	micco_write(MICCO_SYSCTRL_B, 0x20);
	micco_write(MICCO_SYSCTRL_A, 0x60);

	/* IRQ is masked during the power-up sequence and will not be released
	 * until they have been read for the first time */
	micco_read(MICCO_EVENT_A, &value);
	micco_read(MICCO_EVENT_B, &value);
	micco_read(MICCO_EVENT_C, &value);
	micco_read(MICCO_EVENT_D, &value);

	/* enable ADC for the VBAT and RF measurement on CP side */
	micco_read(MICCO_ADC_MAN_CONTROL, &value);
	/* Enable internal LDO for auto ADC */
	value |= MICCO_ADC_MAN_CONT_LDOADC_EN;
	micco_write(MICCO_ADC_MAN_CONTROL, value);

	micco_read(MICCO_ADC_AUTO_CONTROL_1, &value);
	value |= MICCO_ADC_AUTO_1_AUTOADC_SLEEP_EN | MICCO_ADC_AUTO_1_VBAT_EN;
	micco_write(MICCO_ADC_AUTO_CONTROL_1, value);

	micco_read(MICCO_ADC_AUTO_CONTROL_2, &value);
	value |= MICCO_ADC_AUTO_2_ADC4_EN;
	micco_write(MICCO_ADC_AUTO_CONTROL_2, value);
}

/* micco_power_module[] should be consistent with enum
 * in arch/arm/mach-pxa/include/mach/pxa3xx_pmic.h */
static struct power_supply_module miccoB0_power_modules[] = {
	/* {command,		power_module}, */
	{VCC_CORE,		BUCK1},
	{VCC_SRAM,		BUCK2},
	{VCC_MVT,		LDO1},
	{VCC_3V_APPS,		LDO3},
	{VCC_SDIO,		LDO13},
	{VCC_CAMERA_ANA,	LDO3},
	{VCC_USB,		LDO5},
	{VCC_LCD,		LDO3},
	{VCC_TSI,		0},
	{VCC_CAMERA_IO,		BUCK3},
	{VCC_1P8V,		LDO4},
	{VCC_MEM,		BUCK3},
	{HDMI_TX,		0},
	{TECH_3V,		0},
	{TECH_1P8V,		0},
};

/* micco_power_module[] should be consistent with enum
 * in arch/arm/mach-pxa/include/mach/pxa3xx_pmic.h */
static struct power_supply_module miccoEA_power_modules[] = {
	/* {command,		power_module}, */
	{VCC_CORE,		BUCK1},
	{VCC_SRAM,		BUCK2},
	{VCC_MVT,		LDO1},
	{VCC_3V_APPS,		LDO3},
	{VCC_SDIO,		LDO13},
	{VCC_CAMERA_ANA,	LDO3},
	{VCC_USB,		LDO5},
	{VCC_LCD,		LDO3},
	{VCC_TSI,		0},
	{VCC_CAMERA_IO,		LDO2},
	{VCC_1P8V,		LDO4},
	{VCC_MEM,		LDO2},
	{HDMI_TX,		0},
	{TECH_3V,		0},
	{TECH_1P8V,		0},
};

static struct power_chip micco_chips[] = {
	{MICCO_B0_ID,	"miccoB0",	miccoB0_power_modules},
	{MICCO_EA_ID,	"miccoEA",	miccoEA_power_modules},
	{MICCO_EB_ID,	"miccoEB",	miccoEA_power_modules},
	{0,	NULL,		NULL},
};

static struct micco_platform_data micco_data = {
	.init_irq = micco_init_irq,
	.ack_irq = micco_ack_irq,
	.platform_init = saar_micco_init,
	.power_chips = micco_chips,
};
#endif /* CONFIG_PXA3xx_MICCO || CONFIG_PXA3xx_MICCO_MODULE*/

#if defined(CONFIG_PXA_IRDA) || defined(CONFIG_PXA_IRDA_MODULE)
static void saar_irda_transceiver_mode(struct device *dev, int mode)
{
	unsigned long flags;
	int err;
	static int irda_mfp_init;
	int gpio_ir_shdn = ((struct pxaficp_platform_data *)
			    dev->platform_data)->gpio_ir_shdn;

	if (!irda_mfp_init) {
		err = gpio_request(gpio_ir_shdn, "IRDA SHDN");
		if (err) {
			gpio_free(gpio_ir_shdn);
			printk(KERN_ERR "Request GPIO failed,"
			       "gpio: %d return :%d\n", gpio_ir_shdn, err);
			return;
		}
		gpio_direction_output(gpio_ir_shdn, 1);

		irda_mfp_init = 1;
	}

	local_irq_save(flags);
	if (mode & IR_SIRMODE) {
		gpio_set_value(gpio_ir_shdn, 0);
	} else if (mode & IR_FIRMODE) {
		/* do not support FIR */
	}
	if (mode & IR_OFF) {
		gpio_set_value(gpio_ir_shdn, 1);
	}
	local_irq_restore(flags);
}

static struct pxaficp_platform_data saar_ficp_platform_data = {
	.transceiver_cap  = IR_SIRMODE | IR_OFF,
	.transceiver_mode = saar_irda_transceiver_mode,
};
#endif /* (CONFIG_PXA_IRDA) || (CONFIG_PXA_IRDA_MODULE) */

#if defined(CONFIG_PXA_CAMERA)

/* sensor init */
static int sensor_power_on(int flag)
{
	/*
	 * flag, 0, low resolution
	 * flag, 1, high resolution
	 */
	if (gpio_request(QCI_HI_PWDN_PIN, "CAM_EANBLE_HI_SENSOR")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", QCI_HI_PWDN_PIN);
		return -EIO;
	}

	if (gpio_request(QCI_LO_PWDN_PIN, "CAM_EANBLE_LO_SENSOR")){
		gpio_free(QCI_HI_PWDN_PIN);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", QCI_LO_PWDN_PIN);
		return -EIO;
	}
	
	gpio_direction_output(QCI_HI_PWDN_PIN, 1);
	gpio_direction_output(QCI_LO_PWDN_PIN, 1);
	
	if(flag){
		gpio_direction_output(QCI_HI_PWDN_PIN, 0);
		gpio_direction_output(QCI_LO_PWDN_PIN, 1);
	}else{
		gpio_direction_output(QCI_LO_PWDN_PIN, 0);
		gpio_direction_output(QCI_HI_PWDN_PIN, 1);
	}
	
	gpio_free(QCI_HI_PWDN_PIN);
	gpio_free(QCI_LO_PWDN_PIN);

	return 0;
}

static int sensor_power_off(int flag)
{
	/*
	 * flag, 0, low resolution
	 * flag, 1, high resolution
	 */
	flag = flag; /* power off all */
	
	if (gpio_request(QCI_HI_PWDN_PIN, "CAM_EANBLE_HI_SENSOR")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", QCI_HI_PWDN_PIN);
		return -EIO;
	}

	if (gpio_request(QCI_LO_PWDN_PIN, "CAM_EANBLE_LO_SENSOR")){
		gpio_free(QCI_HI_PWDN_PIN);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", QCI_LO_PWDN_PIN);
		return -EIO;
	}

	gpio_direction_output(QCI_HI_PWDN_PIN, 1);
	gpio_direction_output(QCI_LO_PWDN_PIN, 1);
	
	gpio_free(QCI_HI_PWDN_PIN);
	gpio_free(QCI_LO_PWDN_PIN);
	
	return 0;
}

static struct sensor_platform_data ov3640_sensor_data = {
	.id = SENSOR_HIGH,
	.power_on = sensor_power_on,
	.power_off = sensor_power_off,
};

/* sensor init over */

/* camera platform data */
static mfp_cfg_t sync[] = {
	GPIO73_CI_HSYNC,
	GPIO74_CI_VSYNC,
};

static mfp_cfg_t sync_gpio[] = {
	GPIO73_CI_HSYNC_GPIO,
	GPIO74_CI_VSYNC_GPIO,
};

static void cam_sync_to_gpio(void)
{
	pxa3xx_mfp_config(ARRAY_AND_SIZE(sync_gpio));
}

static void cam_sync_from_gpio(void)
{
	pxa3xx_mfp_config(ARRAY_AND_SIZE(sync));
}

static int cam_init(void)
{
	return 0;
}

static void cam_deinit(void)
{
}

static void cam_suspend(void)
{
	sensor_power_off(SENSOR_LOW);
}

static void cam_resume(void)
{
	sensor_power_off(SENSOR_LOW);
}

static struct cam_platform_data cam_ops = {
	.vsync_gpio		= MFP_PIN_GPIO74,
	.init			= cam_init,
	.deinit			= cam_deinit,
	.suspend		= cam_suspend,
	.resume			= cam_resume,
	.sync_to_gpio		= cam_sync_to_gpio,
	.sync_from_gpio		= cam_sync_from_gpio,
};

static void __init saar_init_cam(void)
{
	pxa3xx_device_cam.dev.platform_data = &cam_ops;
	platform_device_register(&pxa3xx_device_cam);
}

/* QCI init over */

#endif

#if defined(CONFIG_RADIO_SI4703) || defined(CONFIG_RADIO_SI4703_MODULE)
int si4703_setup(struct i2c_client *client, void *context)
{
	int reset_pin = mfp_to_gpio(MFP_PIN_GPIO86);
	int irq_pin = mfp_to_gpio(MFP_PIN_GPIO96);

	gpio_request(reset_pin, "si4703 FM radio reset");
	gpio_request(irq_pin, "si4703 FM radio interrupt");

	/* clear GPIO96 edge detect */
	pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO96_GPIO), MFP_EDGE_NONE);

	/* configure interrupt pin as input */
	gpio_direction_input(reset_pin);

	/* assert reset for 100 ms*/
	gpio_direction_output(reset_pin, 0);
	mdelay(100);

	/* deassert reset */
	gpio_set_value(reset_pin, 1);

	gpio_free(reset_pin);
	gpio_free(irq_pin);

	return 0;
}

static struct si4703_platform_data si4703_data = {
	.setup = si4703_setup,
};
#endif

static struct i2c_board_info saar_i2c_board_info[] = {
#if defined(CONFIG_PXA3xx_MICCO) || defined(CONFIG_PXA3xx_MICCO_MODULE)
	{
		.type		= "micco",
		.addr		= 0x34,
		.platform_data	= &micco_data,
		.irq		= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO83)),
	},
#endif
#if defined(CONFIG_PXA_CAMERA)
	{
		.type		= "sensor_ov3640",
		.addr		= 0x3C,
		.platform_data	= &ov3640_sensor_data,
	},
#endif

#if defined(CONFIG_RADIO_SI4703) || defined(CONFIG_RADIO_SI4703_MODULE)
	{
		.type		= "si4703",
		.addr		= 0x10,
		.platform_data	= &si4703_data,
		.irq   		= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO96)),
	},
#endif
};

struct pxa3xx_freq_mach_info saar_freq_mach_info = {
	.flags = PXA3xx_USE_POWER_I2C,
};

#if defined(CONFIG_FB_PXA) || (CONFIG_FB_PXA_MODULE)
static unsigned short init_panel[] = {
	/* single frame */
	(LCD_CMD_NOP |	LCD_CMD_A0_COMMAND),	
	MAKEUP_CMD(0x00),
	(LCD_CMD_WAIT |	0),	

	(LCD_CMD_NOP |	LCD_CMD_A0_COMMAND),
	MAKEUP_CMD(0x00),
	(LCD_CMD_WAIT |	0),

	(LCD_CMD_NOP |	LCD_CMD_A0_COMMAND),
	MAKEUP_CMD(0x00),
	(LCD_CMD_WAIT |	0),

	(LCD_CMD_NOP |	LCD_CMD_A0_COMMAND),
	MAKEUP_CMD(0x00),
	(LCD_CMD_WAIT |	10),

	/* calibration control */
	MAKEUP_CMD(0x00),		
	MAKEUP_CMD(0xA4),
	MAKEUP_DATA(0x80),
	MAKEUP_DATA(0x01),
	(LCD_CMD_WAIT |	150),

	/*Power-On Init sequence*/
	MAKEUP_CMD(0x00),	/* output ctrl */
	MAKEUP_CMD(0x01),
	MAKEUP_DATA(0x01),
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x00),	/* wave ctrl */
	MAKEUP_CMD(0x02),		
	MAKEUP_DATA(0x07),
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x03),	/* entry mode */
	MAKEUP_DATA(0xD0),
	MAKEUP_DATA(0x30),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x08),	/* display ctrl 2 */
	MAKEUP_DATA(0x08),
	MAKEUP_DATA(0x08), 
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x09),	/* display ctrl 3 */
	MAKEUP_DATA(0x04),
	MAKEUP_DATA(0x2F),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x0A),	/* display ctrl 4 */
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x08),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x0D),	/* Frame Marker position */
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x08),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x60),	/* Driver output control */
	MAKEUP_DATA(0x27),
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x61),	/* Base image display control */
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x01),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x30),	/* Y settings 30h-3Dh */
	MAKEUP_DATA(0x07),
	MAKEUP_DATA(0x07),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x31),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x07),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x32),	/* Timing(3), ASW HOLD=0.5CLK */
	MAKEUP_DATA(0x04),
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x33),	/* Timing(4), CKV ST=0CLK, CKV ED=1CLK */
	MAKEUP_DATA(0x03),
	MAKEUP_DATA(0x03),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x34),		
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x35),		
	MAKEUP_DATA(0x02),
	MAKEUP_DATA(0x05),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x36),		
	MAKEUP_DATA(0x1F),
	MAKEUP_DATA(0x1F),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x37),		
	MAKEUP_DATA(0x07),
	MAKEUP_DATA(0x07),
	MAKEUP_CMD(0x00),		
	MAKEUP_CMD(0x38),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x07),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x39),		
	MAKEUP_DATA(0x04),
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x3A),		
	MAKEUP_DATA(0x03),
	MAKEUP_DATA(0x03),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x3B),		
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x3C),		
	MAKEUP_DATA(0x02),
	MAKEUP_DATA(0x05),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x3D),		
	MAKEUP_DATA(0x1F),
	MAKEUP_DATA(0x1F),
	MAKEUP_CMD(0x00),	/* Display control 1 */
	MAKEUP_CMD(0x07),		
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x01),
	MAKEUP_CMD(0x00),	/* Power control 5 */
	MAKEUP_CMD(0x17),		
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x01),
	MAKEUP_CMD(0x00),	/* Power control 1 */
	MAKEUP_CMD(0x10),		
	MAKEUP_DATA(0x10),
	MAKEUP_DATA(0xB0),
	MAKEUP_CMD(0x00),	/* Power control 2 */
	MAKEUP_CMD(0x11),		
	MAKEUP_DATA(0x01),
	MAKEUP_DATA(0x30),
	MAKEUP_CMD(0x00),	/* Power control 3 */
	MAKEUP_CMD(0x12),		
	MAKEUP_DATA(0x01),
	MAKEUP_DATA(0x9E),
	MAKEUP_CMD(0x00),	/* Power control 4 */
	MAKEUP_CMD(0x13),		
	MAKEUP_DATA(0x17),
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x00),	/* Power control 3 */
	MAKEUP_CMD(0x12),		
	MAKEUP_DATA(0x01),
	MAKEUP_DATA(0xBE),
	(LCD_CMD_WAIT |	100),

	/* display mode : 240*320 */
	MAKEUP_CMD(0x00),	/* RAM address set(H) 0*/
	MAKEUP_CMD(0x20),	
	MAKEUP_DATA(0x00), 
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x00),	/* RAM address set(V)   4*/
	MAKEUP_CMD(0x21), 
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x00), 
	MAKEUP_CMD(0x00),	/* Start of Window RAM address set(H) 8*/
	MAKEUP_CMD(0x50),	
	MAKEUP_DATA(0x00), 
	MAKEUP_DATA(0x00), 
	MAKEUP_CMD(0x00), 	/* End of Window RAM address set(H) 12*/
	MAKEUP_CMD(0x51),
	MAKEUP_DATA(0x00), 
	MAKEUP_DATA(0xEF), 
	MAKEUP_CMD(0x00),	/* Start of Window RAM address set(V) 16*/
	MAKEUP_CMD(0x52),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x00), 
	MAKEUP_CMD(0x00),	/* End of Window RAM address set(V) 20*/
	MAKEUP_CMD(0x53),
	MAKEUP_DATA(0x01), 
	MAKEUP_DATA(0x3F), 
	MAKEUP_CMD(0x00), 	/* Panel interface control 1 */
	MAKEUP_CMD(0x90),		
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x1A),
	MAKEUP_CMD(0x00), 	/* Panel interface control 2 */
	MAKEUP_CMD(0x92),		
	MAKEUP_DATA(0x04),
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x00), 	/* Panel interface control 3 */
	MAKEUP_CMD(0x93),		
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x05),
	(LCD_CMD_WAIT |	20),
};

static unsigned short turn_on_display[] = {
	MAKEUP_CMD(0x00),		
	MAKEUP_CMD(0x07),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x21),
	(LCD_CMD_WAIT |	1),

	MAKEUP_CMD(0x00),		
	MAKEUP_CMD(0x07),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x61),
	(LCD_CMD_WAIT |	100),

	MAKEUP_CMD(0x00),		
	MAKEUP_CMD(0x07),
	MAKEUP_DATA(0x01),
	MAKEUP_DATA(0x73),
	(LCD_CMD_WAIT |	1),
};

static unsigned short turn_off_display[] = {
	MAKEUP_CMD(0x00),		
	MAKEUP_CMD(0x07),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x72),
	(LCD_CMD_WAIT |	40),

	MAKEUP_CMD(0x00),		
	MAKEUP_CMD(0x07),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x01),
	(LCD_CMD_WAIT |	1),

	MAKEUP_CMD(0x00),		
	MAKEUP_CMD(0x07),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x00),
	(LCD_CMD_WAIT |	1),
};

static unsigned short power_off_display[] = {
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x10),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x80),

	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x11),
	MAKEUP_DATA(0x01),
	MAKEUP_DATA(0x60),

	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x12),
	MAKEUP_DATA(0x01),
	MAKEUP_DATA(0xAE),
	(LCD_CMD_WAIT | 40),

	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x10),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x00),
};

static unsigned short update_framedata[] = {
	/* set display ram: 240*320 */
	MAKEUP_CMD(0x00),	/* RAM address set(H) 0*/
	MAKEUP_CMD(0x20),	
	MAKEUP_DATA(0x00), 
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x00),	/* RAM address set(V) 4*/
	MAKEUP_CMD(0x21), 
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x00), 
	MAKEUP_CMD(0x00),	/* Start of Window RAM address set(H) 8 */
	MAKEUP_CMD(0x50),	
	MAKEUP_DATA(0x00), 
	MAKEUP_DATA(0x00), 
	MAKEUP_CMD(0x00), 	/* End of Window RAM address set(H) 12 */
	MAKEUP_CMD(0x51),
	MAKEUP_DATA(0x00), 
	MAKEUP_DATA(0xEF), 
	MAKEUP_CMD(0x00),	/* Start of Window RAM address set(V) 16 */
	MAKEUP_CMD(0x52),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x00), 
	MAKEUP_CMD(0x00),	/* End of Window RAM address set(V) 20 */
	MAKEUP_CMD(0x53),
	MAKEUP_DATA(0x01), 
	MAKEUP_DATA(0x3F), 

	/* wait for vsync cmd before transferring frame data */
	LCD_CMD_WAIT_FOR_VSYNC | LCD_CMD_A0_COMMAND,

	/* write ram */
	MAKEUP_CMD(0x00),		
	MAKEUP_CMD(0x22),

	/* write frame data */
	LCD_CMD_FRAME_DATA_WRITE | LCD_CMD_A0_DATA,
};

static void ltm022a97a_lcd_power(int on, struct fb_var_screeninfo *var)
{
	static int pin_requested = 0;
	struct fb_info *fbi = container_of(var, struct fb_info, var);
	
	if (!pin_requested) {
		if (gpio_request(MFP_PIN_GPIO16, "lcd reset"))
			return -EIO;
		else
			pin_requested = 1;
	}

	if (on) {
		gpio_direction_output(MFP_PIN_GPIO16,0);
		gpio_set_value(MFP_PIN_GPIO16,0);
		mdelay(100);
		gpio_set_value(MFP_PIN_GPIO16,1);
		mdelay(10);
		if (pxafb_send_cmd(fbi, ARRAY_AND_SIZE(init_panel)))
			printk(KERN_ERR "LTM 022A97A init: failed!\n");
		if (pxafb_send_cmd(fbi, ARRAY_AND_SIZE(turn_on_display)))
			printk(KERN_ERR "LTM 022A97A display on: failed!\n");
	} else {
		if (pxafb_send_cmd(fbi, ARRAY_AND_SIZE(turn_off_display)))
			printk(KERN_ERR "LTM 022A97A display off: failed!\n");
		if (pxafb_send_cmd(fbi, ARRAY_AND_SIZE(power_off_display)))
			printk(KERN_ERR "LTM 022A97A power off: failed!\n");
	}
	
	return 0;
}

static int ltm022a97a_update_framedata(struct fb_info *fbi)
{
	if (pxafb_send_cmd(fbi, ARRAY_AND_SIZE(update_framedata))) {
		printk(KERN_ERR "LTM 022A97A update frame data: failed!\n");
		return -EIO;
	} else {
		return 0;
	}
}

static struct pxafb_mode_info toshiba_ltm022a97a_modes[] = {
	[0] = {
		.flags			= PXAFB_SMART_PANEL | PXAFB_STD_SMART_PANEL |\
						PXAFB_AUTO_REFRESH,
		.xres			= 240,
		.yres			= 320,
		.bpp			= 16,
		.left_margin		= 0,
		.right_margin		= 0,
		.upper_margin		= 0,
		.lower_margin		= 0,		
		.wr_setup_time		= 30,
		.wr_pulse_width		= 30,
		.rd_setup_time 		= 30,
		.rd_pulse_width 	= 170,
		.op_hold_time 		= 30,
		.cmd_inh_time		= 60,
		.sync_cnt		= 1,
		.update_framedata	= ltm022a97a_update_framedata,
	},
};

static struct pxafb_mach_info saar_lcd_info = {
	.modes			= toshiba_ltm022a97a_modes,
	.num_modes		= 1,
	.lccr0			= LCCR0_Act,
	.lccr3			= LCCR3_PCP | LCCR3_HSP | LCCR3_VSP,
	.lccr6			= 0,
	.pxafb_lcd_power	= ltm022a97a_lcd_power,
};

static void __init saar_init_lcd(void)
{
	set_pxa_fb_info(&saar_lcd_info);
}
#else
static void __init saar_init_lcd(void) {}
#endif /* CONFIG_FB_PXA || CONFIG_FB_PXA_MODULE */

#ifdef CONFIG_PM
extern irqreturn_t uartrx_detect_irq(int irq, void *_dev);

static mfp_cfg_t saar_ffuart_enable_mfp_cfg[] = {
	/* FFUART */
	GPIO53_FFRXD,
	GPIO54_FFTXD,
	DF_ADDR0_GPIO_53,
};

static mfp_cfg_t saar_ffuart_disable_mfp_cfg[] = {
	/* FFUART */
	GPIO53_GPIO,
	GPIO54_GPIO,
	DF_ADDR0_NONE,
};

extern int is_uart_gpio(void);
static int saar_ffuart_switch(int enable)
{
	int gpio_rx, irq_rx;
	int ret;
	gpio_rx = mfp_to_gpio(MFP_PIN_GPIO53);
	irq_rx = gpio_to_irq(gpio_rx);

	if (enable) {
		gpio_free(gpio_rx);
		/* if edge detection is not clear, the altfun switch will
		 * gpio interrupt is generated on this gpio pin when
		 * switching pins from UART to gpio. so clear it.
		 */
		GRER(gpio_rx) &= ~GPIO_bit(gpio_rx);
		GFER(gpio_rx) &= ~GPIO_bit(gpio_rx);
		pxa3xx_mfp_config(ARRAY_AND_SIZE(saar_ffuart_enable_mfp_cfg));
	} else {
		pxa3xx_mfp_config(ARRAY_AND_SIZE(saar_ffuart_disable_mfp_cfg));
		pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO53_GPIO), MFP_EDGE_FALL);
		GRER(gpio_rx) |= GPIO_bit(gpio_rx);
		GFER(gpio_rx) |= GPIO_bit(gpio_rx);

		ret = gpio_request(gpio_rx, "FFUART RX");
		/* clear the MFPR */
		gpio_direction_input(gpio_rx);
	}
	return irq_rx;
}

static struct pxa_uart_mach_info saar_ffuart_info = {
	.uart_pin_switch = saar_ffuart_switch,
};

static mfp_cfg_t saar_stuart_enable_mfp_cfg[] = {
	/* STUART */
	GPIO45_STRXD,
	GPIO46_STTXD,
};

static mfp_cfg_t saar_stuart_disable_mfp_cfg[] = {
	/* STUART */
	GPIO45_GPIO,
	GPIO46_GPIO,
};

static int saar_stuart_switch(int enable)
{
	int gpio_rx, irq_rx;
	int ret;
	gpio_rx = mfp_to_gpio(MFP_PIN_GPIO45);
	irq_rx = gpio_to_irq(gpio_rx);

	if (enable) {
		gpio_free(gpio_rx);

		pxa3xx_mfp_config(ARRAY_AND_SIZE(saar_stuart_enable_mfp_cfg));
	} else {
		pxa3xx_mfp_config(ARRAY_AND_SIZE(saar_stuart_disable_mfp_cfg));
		pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO45_GPIO), MFP_EDGE_FALL);

		ret = gpio_request(gpio_rx, "STUART RX");
		/* clear the MFPR */
		gpio_direction_input(gpio_rx);
	}
	return irq_rx;
}

static struct pxa_uart_mach_info saar_stuart_info = {
	.uart_pin_switch = saar_stuart_switch,
};

static int saar_init_wakeup(pm_wakeup_src_t *src)
{
	memset(src, 0, sizeof(pm_wakeup_src_t));
	src->bits.rtc = 1;
	src->bits.ost = 1;
#ifdef CONFIG_PXA930_ACIPC
	src->bits.msl = 1;
#endif
	src->bits.ext0 = 1;
	src->bits.uart1 = 1;
	src->bits.mkey = 1;
	src->bits.eth = 1;
	src->bits.tsi = 1;
}

extern int pxa930_query_gwsr(int);
static int saar_query_wakeup(unsigned int reg, pm_wakeup_src_t *src)
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
	if (reg & PXA3xx_PM_WE_KP)
	        src->bits.mkey = 1;
	if (reg & PXA3xx_PM_WE_GENERIC(3))
		src->bits.tsi = 1;
	if (reg & PXA3xx_PM_WE_GENERIC(13)) {
		if (pxa930_query_gwsr(97))
			src->bits.eth = 1;
		if (pxa930_query_gwsr(53))
			src->bits.uart1 = 1;
	}
}

static int saar_ext_wakeup(pm_wakeup_src_t src, int enable)
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

static int saar_key_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.mkey) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO0_KP_MKIN_0), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO2_KP_MKIN_1), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO4_KP_MKIN_2), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO6_KP_MKIN_3), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO8_KP_MKIN_4), MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_KP;
		}
	} else {
		if (src.bits.mkey) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO0_KP_MKIN_0), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO2_KP_MKIN_1), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO4_KP_MKIN_2), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO6_KP_MKIN_3), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO8_KP_MKIN_4), MFP_EDGE_NONE);
		}
	}
	return ret;
}

static int saar_mmc_wakeup(pm_wakeup_src_t src, int enable)
{
	return 0;
}

static int saar_uart_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.uart1) {
			if (is_uart_gpio()) {
				pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO53_GPIO), MFP_EDGE_FALL);
				ret |= PXA3xx_PM_WE_GENERIC(13);
			} else {
				pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO53_FFRXD), MFP_EDGE_FALL);
				ret |= PXA3xx_PM_WE_GENERIC(9);
			}
		}
		if (src.bits.uart2) {
			if (is_uart_gpio()) {
				pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO45_GPIO), MFP_EDGE_FALL);
				ret |= PXA3xx_PM_WE_GENERIC(13);
			} else {
				pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO45_STRXD), MFP_EDGE_FALL);
				/* note: on pxa930, uart2 use this bit */
				ret |= PXA3xx_PM_WE_GENERIC(2);
			}
		}
	} else {
		if (src.bits.uart1) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO53_FFRXD), MFP_EDGE_NONE);
		}
		if (src.bits.uart2) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO45_STRXD), MFP_EDGE_NONE);
		}
	}
	return ret;
}

static int saar_eth_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.eth) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO97_ETHER_IRQ),
				MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(13);
		}
	} else {
		if (src.bits.eth) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO97_ETHER_IRQ),
				MFP_EDGE_NONE);
		}
	}
	return ret;
}

static int saar_tsi_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.tsi) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(PMIC_INT_GPIO_83),
				MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(3);
		}
	} else {
		if (src.bits.tsi) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(PMIC_INT_GPIO_83),
				MFP_EDGE_NONE);
		}
	}
	return ret;
}

static struct pxa3xx_peripheral_wakeup_ops wakeup_ops = {
	.init	= saar_init_wakeup,
	.query	= saar_query_wakeup,
	.ext    = saar_ext_wakeup,
	.key    = saar_key_wakeup,
	.mmc    = saar_mmc_wakeup,
	.uart   = saar_uart_wakeup,
	.eth	= saar_eth_wakeup,
	.tsi	= saar_tsi_wakeup,
};
#else
static struct pxa_uart_mach_info saar_ffuart_info = NULL;
static struct pxa_uart_mach_info saar_stuart_info = NULL;
#endif

void saar_stereo_amp_en(int value)
{
	int err;

	err = gpio_request(MFP_PIN_GPIO84, "STEREO_AMP_EN");
	if (err) {
		gpio_free(MFP_PIN_GPIO84);
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d return :%d\n", MFP_PIN_GPIO84, err);
		return;
	}
	gpio_direction_output(MFP_PIN_GPIO84, value);
	gpio_free(MFP_PIN_GPIO84);
}

#define I2C_CLK		(1UL << (CKEN_I2C - 32))
#define I2C_MICCO_ADDR	0x34
static void __init saar_i2c_init(void)
{
	/* Enable i2c clock */
	CKENB |= I2C_CLK;

	/* Initialize I2C Unit */
	ISAR = 1;
	ICR = ICR_UR;
	ICR = ICR_SCLE | ICR_IUE;
}

static void __init saar_i2c_deinit(void)
{
	ICR = ICR_UR;
	CKENB &= ~(I2C_CLK);
}

static int __init wait_i2c_rx_full(int timeout)
{
	u32 temp;
	while (timeout--) {
		temp = ISR;
		if ((temp & ISR_IRF) == ISR_IRF) {
			ISR = temp | ISR_IRF;
			return 0;
		}
		 mdelay(200);
	}
	return 1;
}


static int __init wait_i2c_tx_empty(int timeout)
{
	u32 temp;

	while (timeout--) {
		temp = ISR;
		if ((temp & ISR_ITE) == ISR_ITE) {
			ISR = temp | ISR_ITE;
			if ((temp & ISR_ALD) == ISR_ALD)
				ISR |= ISR_ALD;
			return 0;
		}
		mdelay(200);
	}
	return 1;
}

static int __init saar_i2c_write(int sreg, unsigned char *buf, int len)
{
	int icr;

	IDBR = (sreg << 1);

	icr = ICR & ~(ICR_STOP | ICR_ALDIE);
	ICR = icr | ICR_START | ICR_TB;

	if (wait_i2c_tx_empty(20))
		return -1;

	while (len--) {
		IDBR = (unsigned int) (*buf++);
		icr = (ICR & ~ICR_START) | ICR_ALDIE | ICR_TB;
		if (len == 0)
			icr |= ICR_STOP;

		ICR = icr;
		if (wait_i2c_tx_empty(250))
			return -1;
	}

	ICR &= ~ICR_STOP;
	return 0;
}

static int __init saar_i2c_read(int sreg, char *data_buf, int data_count)
{
	u32 reg;

	IDBR = (sreg << 1) | 1;
	reg = ICR;
	reg |= (ICR_START | ICR_TB);
	reg &= ~(ICR_STOP | ICR_ALDIE);
	ICR = reg;

	if (wait_i2c_tx_empty(20))
		return -1;

	while (data_count--) {
		reg = ICR;
		reg &= ~ICR_START;
		reg |= ICR_ALDIE | ICR_TB;
		if (data_count == 0)
			reg |= ICR_ACKNAK | ICR_STOP;
		else
			reg &= ~ICR_ACKNAK;

		ICR = reg;
		if (wait_i2c_rx_full(60))
			return -1;
		reg = IDBR & 0xFF;
		*data_buf++ = (u8) reg;
	}
	ICR &= ~(ICR_STOP | ICR_ACKNAK);
	return 0;
}

static int __init is_saar_2_6(void)
{
	u8 cur_chip_id;
	u8 reg;

	saar_i2c_init();

	/* set the reg address of the micco version */
	reg = 0x0; /* chip id reg */
	if (saar_i2c_write(I2C_MICCO_ADDR, &reg, 1)) {
		pr_err("ERROR: cannot write the address of micco reg!\n");
		return 0;
	}

	/* read the reg address of the micco version */
	if (saar_i2c_read(I2C_MICCO_ADDR, &cur_chip_id, 1)) {
		pr_err("ERROR: cannot write the address of micco reg!\n");
		return 0;
	}

	saar_i2c_deinit();

	pr_info("micco id %x!\n", cur_chip_id);
	return (cur_chip_id == MICCO_EB_ID);
}

static int __init is_saar_pv65(void)
{
	return cpu_is_pxa935();
}

static void saar_set_lowpower_mfp(void)
{
	/* It will change mfp value set by OBM to save more power,
	 * it should be merged into OBM mfp setting eventually. */

	/* these values have been merged int OBM
	 * pxa3xx_mfp_write(48, 0xa440);
	 * pxa3xx_mfp_write(49, 0xa440);
	 * pxa3xx_mfp_write(51, 0xa440);
	 */
}


#define SMC_RESOURCE_START	(SAAR_ETH_PHYS + 0x300)
#define SMC_IO_EXTENT		16
#define BANK_SELECT		14

static int smc91x_exist(void)
{
	unsigned char __iomem *smc_membase;
	unsigned char __iomem *smc91x_membase;
	unsigned int csadrcfg3, temp;

	smc91x_membase = ioremap(SMC_RESOURCE_START, SMC_IO_EXTENT);
	smc_membase = ioremap(SMC_START, SMC_END - SMC_START + 1);
	csadrcfg3 = readl(smc_membase + CSADRCFG3_OFF);
	writel((csadrcfg3 & 0xFFFFFFF0) | 0xD, smc_membase + CSADRCFG3_OFF);
	while((readl(smc_membase + CSADRCFG3_OFF) & 0xF) != 0xD)
		;
	temp = *(unsigned short *)(smc91x_membase + BANK_SELECT);
	writel(csadrcfg3, smc_membase + CSADRCFG3_OFF);
	while(readl(smc_membase + CSADRCFG3_OFF) != csadrcfg3)
                ;
	iounmap(smc_membase);
	iounmap(smc91x_membase);

	if (temp == 0x3300)
		return 1;
	else
		return 0;
}

static void __init saar_init(void)
{
#ifdef CONFIG_MACH_SAAR_PIN_MUX
	/* initialize MFP configurations */
	pxa3xx_mfp_config(ARRAY_AND_SIZE(saar_mfp_cfg));
#endif

	/* update ISRAM configration for pxa935 */
	if (cpu_is_pxa935())
		pxa3xx_device_imm.resource = pxa935_resource_imm;

	/* check whether the platform is 2.6 by micco's version currently */
	/* saar pxa935-1.1, same as saar v2.6 board for MMC and IrDA */
        if (is_saar_2_6() || is_saar_pv65()) {

#ifdef CONFIG_MACH_SAAR_PIN_MUX
		/* initialize MFP configurations */
		pxa3xx_mfp_config(ARRAY_AND_SIZE(saar26_mfp_cfg));
#endif
#if defined(CONFIG_PXA_IRDA) || defined(CONFIG_PXA_IRDA_MODULE)
		saar_ficp_platform_data.gpio_ir_shdn = mfp_to_gpio(MFP_PIN_GPIO62);
		saar_ficp_platform_data.uart_irq = IRQ_FFUART;
		saar_ficp_platform_data.uart_reg_base = __PREG(FFUART);
		saar_ficp_platform_data.p_dev = &pxa_device_ffuart;
#endif
	} else {
#ifdef CONFIG_MACH_SAAR_PIN_MUX
		/* initialize MFP configurations */
		pxa3xx_mfp_config(ARRAY_AND_SIZE(saar25_mfp_cfg));
#endif
#if defined(CONFIG_PXA_IRDA) || defined(CONFIG_PXA_IRDA_MODULE)
		saar_ficp_platform_data.gpio_ir_shdn = mfp_to_gpio(MFP_PIN_GPIO32);
		saar_ficp_platform_data.uart_irq = IRQ_STUART;
		saar_ficp_platform_data.uart_reg_base = __PREG(STUART);
		saar_ficp_platform_data.p_dev = NULL;
#endif
	}

	/* dvfm device */
	set_pxa3xx_freq_info(&saar_freq_mach_info);

	/* performance monitor unit */
	pxa3xx_set_pmu_info(NULL);

	/* lcd */
	saar_init_lcd();

	/* ethernet */
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO97));
	if(smc91x_exist())
		platform_device_register(&smc91x_device);

	platform_add_devices(devices, ARRAY_SIZE(devices));

	if(is_uart_gpio()) {
		pxa_set_ffuart_info(&saar_ffuart_info);
#ifndef CONFIG_ISPT
		pxa_set_stuart_info(&saar_stuart_info);
#endif
	} else {
		pxa_set_ffuart_info(NULL);
#ifndef CONFIG_ISPT
		pxa_set_stuart_info(NULL);
#endif
	}
	pxa_set_btuart_info(NULL);

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
	pxa_set_keypad_info(&saar_keypad_info);
#endif

	/* trackball */
	saar_init_trackball();

	/* nand */
	saar_init_nand();

	/* onenand */
	saar_init_onenand();

#ifdef CONFIG_ISPT
	saar_init_ispt();
#endif
#if defined(CONFIG_USB_PXA3XX_U2D) || defined(CONFIG_USB_USB_PXA3XX_U2D_MODULE)
	/* u2d */
	if (device_is_enabled(device_u2d)) {
#ifdef CONFIG_MACH_SAAR_PIN_MUX
		pxa3xx_mfp_config(ARRAY_AND_SIZE(saar_u2d_cfg));
#endif
		pxa_set_u2d_info(&saar_u2d_info);
	}
#endif

#if defined(CONFIG_USB_PXA27X_UDC) || defined(CONFIG_USB_USB_PXA27X_UDC_MODULE)
	if (device_is_enabled(device_udc)) {
		pxa_set_udc_info(&saar_udc_info);
	}
#endif

#if defined(CONFIG_USB_PXA9XX_U2O) || defined(CONFIG_USB_USB_PXA9XX_U2O_MODULE)
	/* u2o */
	if (device_is_enabled(device_u2o)) {
		pxa_set_u2o_info(&saar_u2o_info);
	}
#endif

	/* USB 2.0 Host */
	saar_init_ehci();

#if defined(CONFIG_PXA_CAMERA)
	/* initialize camera */
	saar_init_cam();
#endif
	/* i2c devices */
	i2c_register_board_info(0, ARRAY_AND_SIZE(saar_i2c_board_info));
#if defined(CONFIG_PXA_IRDA) || defined(CONFIG_PXA_IRDA_MODULE)
	pxa_set_ficp_info(&saar_ficp_platform_data);
#endif
	saar_init_mmc();

#ifdef CONFIG_PM
	pxa3xx_wakeup_register(&wakeup_ops);
#endif
	saar_set_lowpower_mfp();
}

MACHINE_START(SAAR, "PXA930 handheld Platform (SAAR)")
	.phys_io        = 0x40000000,
	.boot_params    = 0xa0000100,
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io         = pxa_map_io,
	.init_irq       = pxa3xx_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = saar_init,
MACHINE_END

