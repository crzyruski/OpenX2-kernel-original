/*
 *  linux/arch/arm/mach-pxa/tavorevb.c
 *
 *  Support for the Marvell PXA930 Evaluation Board
 *
 *  Copyright (C) 2007-2008 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/smc91x.h>
#include <linux/irq.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach/flash.h>

#include <mach/hardware.h>
#include <mach/pxa3xx-regs.h>
#include <mach/mfp-pxa9xx.h>
#include <mach/gpio.h>
#include <mach/pxafb.h>
#include <mach/pxa27x_keypad.h>
#include <mach/ssp.h>
#include <mach/tavorevb.h>
#include <mach/pxa3xx_nand.h>
#include <mach/pxa3xx_pmic.h>
#include <mach/micco.h>
#include <mach/pxa3xx_dvfm.h>
#include <mach/ohci.h>
#include <mach/ehci.h>
#include <mach/pxa3xx_u2d.h>
#include <mach/pxa9xx_u2o.h>
#include <mach/irda.h>
#include <mach/mmc.h>
#include <mach/udc.h>
#include <mach/uart.h>
#include <mach/imm.h>
#include <mach/pxa3xx_pm.h>
#include <mach/camera.h>
#include <mach/pmu.h>
#include <mach/part_table.h>

#include "devices.h"
#include "generic.h"

#define MAX_SLOTS      3
struct platform_mmc_slot tavorevb_mmc_slot[MAX_SLOTS];
extern int is_android(void);
extern int is_comm_v75(void);

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)
  
#ifdef CONFIG_MACH_TAVOREVB_PIN_MUX
/* Tavor EVB MFP configurations */
static mfp_cfg_t tavorevb_mfp_cfg[] __initdata = {
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

	/* keypad */
	GPIO0_KP_MKIN_0,
	GPIO2_KP_MKIN_1,
	GPIO4_KP_MKIN_2,
	GPIO6_KP_MKIN_3,
	GPIO8_KP_MKIN_4,
	GPIO10_KP_MKIN_5,
	GPIO12_KP_MKIN_6,
	GPIO1_KP_MKOUT_0,
	GPIO3_KP_MKOUT_1,
	GPIO5_KP_MKOUT_2,
	GPIO7_KP_MKOUT_3,
	GPIO9_KP_MKOUT_4,
	GPIO11_KP_MKOUT_5,
	GPIO13_KP_MKOUT_6,

	GPIO14_KP_DKIN_2,
	GPIO15_KP_DKIN_3,

	/* ethernet */
	GPIO48_ETHER_nRD,
	GPIO49_ETHER_nWR,
	GPIO45_ETHER_nAE,
	GPIO59_ETHER_CP,
	GPIO62_ETHER_ARDY,
	GPIO60_ETHER_DQM0,
	GPIO61_ETHER_DQM1,
	GPIO47_ETHER_IRQ,

	/* i2c bus */
	GPIO89_CI2C_SCL,
	GPIO90_CI2C_SDA,

	/* PMIC */
	PMIC_INT_GPIO_83,
	/* avoid GPIO83 confliction */
	GPIO83_SSP3_RXD,

	/* GSSP1 */
	GPIO79_GSSP1_CLK,
	GPIO80_GSSP1_FRM,
	GPIO81_GSSP1_TXD,
	GPIO82_GSSP1_RXD,

	/* SSP2 */
	GPIO85_SSP2_BITCLK,
	GPIO87_SSP2_SYNC,
	GPIO88_SSP2_DATA_OUT,

	/* IRDA */
	GPIO78_IRDA_SHDN,
	
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

};
#endif

static struct resource smc91x_resources[] = {
	[0] = {
		.start	= (TAVOREVB_ETH_PHYS + 0x300),
		.end	= (TAVOREVB_ETH_PHYS + 0xfffff),
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

static struct smc91x_platdata tavorevb_smc91x_info = {
	.flags	= SMC91X_USE_16BIT | SMC91X_NOWAIT | SMC91X_USE_DMA,
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
	.dev		= {
		.platform_data = &tavorevb_smc91x_info,
	},

};

static struct platform_device micco_ts_device = {
	.name 		= "micco-ts",
	.id 		= -1,
};

static struct resource pxa3xx_resource_pwm_bl1[] = {
	[0] = {
		.start	= 0x40B00010,/* PWM2 */
		.end	= 0x40B0001b,
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device pxa3xx_device_pwm_bl1 = {
	.name 		= "pxa3xx_pwm_bl",
	.id 		= 0,
	.num_resources	= ARRAY_SIZE(pxa3xx_resource_pwm_bl1),
	.resource	= pxa3xx_resource_pwm_bl1,
};

static struct resource pxa3xx_resource_pwm_bl2[] = {
	[0] = {
		.start	= 0x40B00000,/* PWM0 */
		.end	= 0x40B0000b,
		.flags	= IORESOURCE_IO,
	},
};

static struct platform_device pxa3xx_device_pwm_bl2 = {
	.name 		= "pxa3xx_pwm_bl",
	.id 		= 1,
	.num_resources	= ARRAY_SIZE(pxa3xx_resource_pwm_bl2),
	.resource	= pxa3xx_resource_pwm_bl2,
};

static struct platform_device micco_bl_device = {
       .name           = "micco-bl",
       .id             = -1,
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

static struct platform_device ispt_device = {
	.name		= "pxa-ispt",
	.id		= -1,
};

static struct platform_device *devices[] __initdata = {
	&smc91x_device,
	&micco_ts_device,
	&pxa3xx_device_imm,
	&ispt_device,
};

#if defined(CONFIG_PXA_IRDA) || defined(CONFIG_PXA_IRDA_MODULE)
static void tavorevb_irda_transceiver_mode(struct device *dev, int mode)
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

static struct pxaficp_platform_data tavorevb_ficp_platform_data = {
	.transceiver_cap  = IR_SIRMODE | IR_OFF,
	.transceiver_mode = tavorevb_irda_transceiver_mode,
	.gpio_ir_shdn = mfp_to_gpio(MFP_PIN_GPIO78),
	.uart_irq = IRQ_STUART,
	.uart_reg_base = __PREG(STUART),
};
#endif /* (CONFIG_PXA_IRDA) || (CONFIG_PXA_IRDA_MODULE) */

#if defined(CONFIG_USB_PXA27X_UDC) || defined(CONFIG_USB_PXA27X_UDC_MODULE)
static mfp_cfg_t pxa300_otg_init_pins[] = {
	GPIO50_GPIO,
};

static mfp_cfg_t pxa300_otg_pins[] = {
	GPIO50_USB_P2_7,
};

int tavorevb_udc_is_miniA(void)
{
	int otg_id = mfp_to_gpio(MFP_PIN_GPIO50);
	int id_value;
	int err;

	pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa300_otg_init_pins));
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
	pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa300_otg_pins));

	return (id_value == 0);
}

static struct pxa2xx_udc_mach_info tavorevb_udc_info = {
	.udc_is_miniA = tavorevb_udc_is_miniA,
};
#endif

#if defined(CONFIG_USB_PXA3XX_U2D) || defined(CONFIG_USB_USB_PXA3XX_U2D_MODULE)
#ifdef CONFIG_MACH_TAVOREVB_PIN_MUX
static mfp_cfg_t tavorevb_u2d_cfg[] __initdata = {
	/* ULPI*/
	DF_ADDR0_GPIO_53,
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

void tavorevb_reset_xcvr(void)
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

static struct pxa3xx_u2d_mach_info tavorevb_u2d_info = {
	.reset_xcvr = tavorevb_reset_xcvr,
};
#endif /* CONFIG_USB_PXA3XX_U2D || CONFIG_USB_PXA3XX_U2D_MODULE */

#if defined(CONFIG_USB_PXA9XX_U2O) || defined(CONFIG_USB_PXA9XX_U2O_MODULE)
extern void pxa9xx_u2o_xcvr_init(void);
static struct pxa9xx_u2o_mach_info tavorevb_u2o_info = {
	.xcvr_init = pxa9xx_u2o_xcvr_init,
};
#endif

#if defined(CONFIG_MTD_NAND_PXA3xx) || defined(CONFIG_MTD_NAND_PXA3xx_MODULE)
static struct pxa3xx_nand_platform_data tavorevb_nand_info;
static void __init tavorevb_init_nand(void)
{
	if (is_android()) {
		if (is_comm_v75()) {
			tavorevb_nand_info.parts = android_128m_v75_partitions;
			tavorevb_nand_info.nr_parts = ARRAY_SIZE(android_128m_v75_partitions);
		} else {
			tavorevb_nand_info.parts = android_128m_partitions;
			tavorevb_nand_info.nr_parts = ARRAY_SIZE(android_128m_partitions);
		}
	} else {
		if (is_comm_v75()) {
			tavorevb_nand_info.parts = pxa930_128m_v75_partitions;
			tavorevb_nand_info.nr_parts = ARRAY_SIZE(pxa930_128m_v75_partitions);
		} else {
			tavorevb_nand_info.parts = pxa930_128m_partitions;
			tavorevb_nand_info.nr_parts = ARRAY_SIZE(pxa930_128m_partitions);
		}
	}

	pxa3xx_device_nand.dev.platform_data = &tavorevb_nand_info;
	platform_device_register(&pxa3xx_device_nand);
}
#else
static inline void tavorevb_init_nand(void) {}
#endif /* CONFIG_MTD_NAND_PXA3xx || CONFIG_MTD_NAND_PXA3xx_MODULE */

#ifdef CONFIG_MTD_ONENAND
static struct flash_platform_data tavorevb_onenand_info;
static void __init tavorevb_init_onenand(void)
{
	if (is_android()) {
		tavorevb_onenand_info.parts = android_128m_partitions;
		tavorevb_onenand_info.nr_parts = ARRAY_SIZE(android_128m_partitions);
	} else {
		if (is_comm_v75()) {
			tavorevb_onenand_info.parts = pxa930_128m_v75_partitions;
			tavorevb_onenand_info.nr_parts = ARRAY_SIZE(pxa930_128m_v75_partitions);
		} else {
			tavorevb_onenand_info.parts = pxa930_128m_partitions;
			tavorevb_onenand_info.nr_parts = ARRAY_SIZE(pxa930_128m_partitions);
		}
	}

	pxa3xx_device_onenand.dev.platform_data = &tavorevb_onenand_info;
	platform_device_register(&pxa3xx_device_onenand);
}

#else
static inline void tavorevb_init_onenand(void) {}
#endif

#if defined(CONFIG_PXA3xx_MICCO) || defined(CONFIG_PXA3xx_MICCO_MODULE)
static int micco_init_irq(void)
{
	gpio_direction_input(PMIC_INT_PIN);

	return 0;
}

static int micco_ack_irq(void)
{
	return 0;
}

static void tavorevb_micco_init(void)
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
	{VCC_CAMERA_ANA,	LDO13},
	{VCC_USB,		LDO5},
	{VCC_LCD,		LDO3},
	{VCC_TSI,		0},
	{VCC_CAMERA_IO,		LDO3},
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
	{0x10,	"miccoB0",	miccoB0_power_modules},
	{0x30,	"miccoEA",	miccoEA_power_modules},
	{0x31,	"miccoEB",	miccoEA_power_modules},
	{0,	NULL,		NULL},
};

static struct micco_platform_data micco_data = {
	.init_irq = micco_init_irq,
	.ack_irq = micco_ack_irq,
	.platform_init = tavorevb_micco_init,
	.power_chips = micco_chips,
};
#endif /* CONFIG_PXA3xx_MICCO || CONFIG_PXA3xx_MICCO_MODULE*/

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

static struct sensor_platform_data ov7660_sensor_data = {
	.id = SENSOR_LOW,
	.power_on = sensor_power_on,
	.power_off = sensor_power_off,
};

static struct sensor_platform_data ov9653_sensor_data = {
	.id = SENSOR_HIGH,
	.power_on = sensor_power_on,
	.power_off = sensor_power_off,
};

/* sensor init over */

/* camera platform data */

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

static void __init tavorevb_init_cam(void)
{
	pxa3xx_device_cam.dev.platform_data = &cam_ops;
	platform_device_register(&pxa3xx_device_cam);
}

/* QCI init over */

#endif

static struct i2c_board_info tavorevb_i2c_board_info[] = {
#if defined(CONFIG_PXA3xx_MICCO) || defined(CONFIG_PXA3xx_MICCO_MODULE)
	{
		.type		= "micco",
		.addr		= 0x34,
		.platform_data	= &micco_data,
		.irq		= IRQ_GPIO(PMIC_INT_PIN),
	},
#endif
#if defined(CONFIG_PXA_CAMERA)
	{
		.type		= "sensor_ov7660",
		.addr		= 0x21,
		.platform_data	= &ov7660_sensor_data,
	},
	{
		.type		= "sensor_ov9653",
		.addr		= 0x30,
		.platform_data	= &ov9653_sensor_data,
	},
#endif
};

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
static struct pxaohci_platform_data tavorevb_ohci_info = {
	.port_mode	= PMM_PERPORT_MODE,
};

static void __init tavorevb_init_ohci(void)
{
	if (!device_is_enabled(device_uhc)) 
		return;

	pxa_set_ohci_info(&tavorevb_ohci_info);
}
#else
static inline void tavorevb_init_ohci(void) {}
#endif

#ifdef CONFIG_USB_EHCI_PXA9XX
static struct pxaehci_platform_data tavorevb_ehci_info = {
/*	.port_mode	= PMM_PERPORT_MODE,
 */
};

static void __init tavorevb_init_ehci(void)
{
	if (!device_is_enabled(device_u2o)) 
		return;

	pxa_set_ehci_info(&tavorevb_ehci_info);
}
#else
static inline void tavorevb_init_ehci(void) {}
#endif

#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
static unsigned int tavorevb_matrix_key_map[] = {
	/* KEY(row, col, key_code) */
	KEY(0, 4, KEY_A), KEY(0, 5, KEY_B), KEY(0, 6, KEY_C),
	KEY(1, 4, KEY_E), KEY(1, 5, KEY_F), KEY(1, 6, KEY_G),
	KEY(2, 4, KEY_I), KEY(2, 5, KEY_J), KEY(2, 6, KEY_K),
	KEY(3, 4, KEY_M), KEY(3, 5, KEY_N), KEY(3, 6, KEY_O),
	KEY(4, 5, KEY_R), KEY(4, 6, KEY_S),
	KEY(5, 4, KEY_U), KEY(5, 4, KEY_V), KEY(5, 6, KEY_W),

	KEY(6, 4, KEY_Y), KEY(6, 5, KEY_Z),

	KEY(0, 3, KEY_0), KEY(2, 0, KEY_1), KEY(2, 1, KEY_2), KEY(2, 2, KEY_3),
	KEY(2, 3, KEY_4), KEY(1, 0, KEY_5), KEY(1, 1, KEY_6), KEY(1, 2, KEY_7),
	KEY(1, 3, KEY_8), KEY(0, 2, KEY_9),

	KEY(6, 6, KEY_SPACE),
	KEY(0, 0, KEY_KPASTERISK), 	/* * */
	KEY(0, 1, KEY_KPDOT), 		/* # */

	KEY(4, 1, KEY_UP),
	KEY(4, 3, KEY_DOWN),
	KEY(4, 0, KEY_LEFT),
	KEY(4, 2, KEY_RIGHT),
	KEY(6, 0, KEY_HOME),
	KEY(3, 2, KEY_END),
	KEY(6, 1, KEY_DELETE),
	KEY(5, 2, KEY_BACK),
	KEY(6, 3, KEY_CAPSLOCK),	/* KEY_LEFTSHIFT), */

	KEY(4, 4, KEY_ENTER),		/* scroll push */
	KEY(6, 2, KEY_ENTER),		/* keypad action */

	KEY(3, 1, KEY_SEND),
	KEY(5, 3, KEY_RECORD),
	KEY(5, 0, KEY_VOLUMEUP),
	KEY(5, 1, KEY_VOLUMEDOWN),

	KEY(3, 0, KEY_F22),	/* soft1 */
	KEY(3, 3, KEY_F23),	/* soft2 */
};

static struct pxa27x_keypad_platform_data tavorevb_keypad_info = {
	.matrix_key_rows	= 7,
	.matrix_key_cols	= 7,
	.matrix_key_map		= tavorevb_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(tavorevb_matrix_key_map),

#ifdef CONFIG_CPU_PXA930
	.enable_enhanced_rotary	= 1,
	.enhanced_rotary_up_key	= KEY_UP,
	.enhanced_rotary_down_key = KEY_DOWN,
#endif

	.debounce_interval	= 30,
};
#endif /* CONFIG_KEYBOARD_PXA3xx || CONFIG_KEYBOARD_PXA3xx_MODULE */

#ifdef CONFIG_ISPT
static mfp_cfg_t pxa930_ispt_pins[] __initdata = {
	/* SSP4 */
	GPIO51_SSP4_CLK,
	GPIO52_SSP4_FRM,
	GPIO45_SSP4_TXD,
};

static void __init tavorevb_init_ispt(void)
{
	pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa930_ispt_pins));
}
#endif

#if defined(CONFIG_MMC)

static mfp_cfg_t pxa9xx_mmc_pins[] __initdata = {
	/* MMC1 */
	GPIO55_MMC1_CMD,
	GPIO56_MMC1_CLK,
	GPIO57_MMC1_DAT0,
	GPIO58_MMC1_DAT1,
	GPIO59_MMC1_DAT2,
	GPIO60_MMC1_DAT3,

	/* MMC2 */
	GPIO101_MMC2_DAT3,
	GPIO102_MMC2_DAT2,
	GPIO103_MMC2_DAT1,
	GPIO104_MMC2_DAT0,
	GPIO105_MMC2_CMD,
	GPIO106_MMC2_CLK,
};

static mfp_cfg_t pxa9xx_mmc_new_pins[] __initdata = {
#ifdef CONFIG_CPU_PXA935
	/* MMC4 */
	GPIO55_MMC4_CMD,
	GPIO56_MMC4_CLK,
	GPIO57_MMC4_DAT0,
	GPIO58_MMC4_DAT1,
	GPIO59_MMC4_DAT2,
	GPIO60_MMC4_DAT3,

	/* MMC5 */
	GPIO101_MMC5_DAT3,
	GPIO102_MMC5_DAT2,
	GPIO103_MMC5_DAT1,
	GPIO104_MMC5_DAT0,
	GPIO105_MMC5_CMD,
	GPIO106_MMC5_CLK,
#endif
};

static int tavorevb_mci_init(struct device *dev,
			     irq_handler_t tavorevb_detect_int,
			     void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	int err, cd_irq, gpio_cd;

	cd_irq = gpio_to_irq(tavorevb_mmc_slot[pdev->id].gpio_cd);
	gpio_cd = tavorevb_mmc_slot[pdev->id].gpio_cd;

	/*
	 * setup GPIO for tavorevb MMC controller
	 */
	err = gpio_request(gpio_cd, "mmc card detect");
	if (err)
		goto err_request_cd;
	gpio_direction_input(gpio_cd);

	err = request_irq(cd_irq, tavorevb_detect_int,
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

static void tavorevb_mci_exit(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	int cd_irq, gpio_cd;

	cd_irq = gpio_to_irq(tavorevb_mmc_slot[pdev->id].gpio_cd);
	gpio_cd = tavorevb_mmc_slot[pdev->id].gpio_cd;

	free_irq(cd_irq, data);
	gpio_free(gpio_cd);
}

static struct pxamci_platform_data tavorevb_mci_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
	.init 		= tavorevb_mci_init,
	.exit		= tavorevb_mci_exit,
};

static struct pxamci_platform_data tavorevb_mci2_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
};

static void __init tavorevb_init_mmc(void)
{
	if (device_is_enabled(device_mmc)) {
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa9xx_mmc_pins));
		pxa_set_mci_info(&tavorevb_mci_platform_data);
		pxa3xx_set_mci2_info(&tavorevb_mci2_platform_data);
		/* MMC card detect & write protect */
		if (cpu_is_pxa935()) {
			tavorevb_mmc_slot[0].gpio_cd  = mfp_to_gpio(MFP_PIN_GPIO172);
		} else {
			tavorevb_mmc_slot[0].gpio_cd  = mfp_to_gpio(MFP_PIN_GPIO97);
		}
	} else {
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa9xx_mmc_new_pins));
		pxa9xx_set_mci4_info(&tavorevb_mci_platform_data);
		pxa9xx_set_mci5_info(&tavorevb_mci2_platform_data);
		tavorevb_mmc_slot[1].gpio_cd  = mfp_to_gpio(MFP_PIN_GPIO172);
	}

}
#else
static inline void tavorevb_init_mmc(void) {}
#endif

struct pxa3xx_freq_mach_info tavorevb_freq_mach_info = {
	.flags = PXA3xx_USE_POWER_I2C,
};

#if defined(CONFIG_FB_PXA) || (CONFIG_FB_PXA_MODULE)
static unsigned short init_panel[] = {
	/* DSTB OUT */
	MAKEUP_CMD(0x00),		
	LCD_CMD_A0_COMMAND | LCD_CMD_NOP,		
	(LCD_CMD_WAIT |	1),	

	MAKEUP_CMD(0x00),		
	LCD_CMD_A0_COMMAND | LCD_CMD_NOP,		
	(LCD_CMD_WAIT |	1),	

	MAKEUP_CMD(0x00),		
	LCD_CMD_A0_COMMAND | LCD_CMD_NOP,		
	(LCD_CMD_WAIT |	1),	

	/* STB OUT */
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x1D),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x05),
	(LCD_CMD_WAIT |	1),	

	/* P-ON Init sequence */
	MAKEUP_CMD(0x00),	/* OSC ON */
	MAKEUP_CMD(0x00),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x01),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x01),	/* SOURCE DRIVER SHIFT DIRECTION and display RAM setting */
	MAKEUP_DATA(0x01),
	MAKEUP_DATA(0x27),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x02),	/* LINE INV */
	MAKEUP_DATA(0x02),
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x03),	/* IF mode(1) */
	MAKEUP_DATA(0x01), 	/* 8bit smart mode(8-8),high speed write mode */
	MAKEUP_DATA(0x30),  
	MAKEUP_CMD(0x07),
	MAKEUP_CMD(0x00),	/* RAM Write Mode */
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x03),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x07),	/* DISPLAY Setting,  262K, fixed(NO scroll), no split screen */
	MAKEUP_DATA(0x40),	/* 16/18/19 BPP */
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x08),	/* BP, FP Seting, BP=2H, FP=3H */
	MAKEUP_DATA(0x03),
	MAKEUP_DATA(0x02),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x0C),	/* IF mode(2), using internal clock & MPU */
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x0D),	/* Frame setting, 1Min. Frequence, 16CLK */
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x10),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x12),	/* Timing(1),ASW W=4CLK, ASW ST=1CLK */
	MAKEUP_DATA(0x03),
	MAKEUP_DATA(0x02),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x13),	/* Timing(2),OEV ST=0.5CLK, OEV ED=1CLK */
	MAKEUP_DATA(0x01),
	MAKEUP_DATA(0x02),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x14),	/* Timing(3), ASW HOLD=0.5CLK */
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x15),	/* Timing(4), CKV ST=0CLK, CKV ED=1CLK */
	MAKEUP_DATA(0x20),
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x1C),		
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x03),
	MAKEUP_CMD(0x00),		
	MAKEUP_DATA(0x04),
	MAKEUP_DATA(0x03),
	MAKEUP_CMD(0x03),
	MAKEUP_CMD(0x01),		
	MAKEUP_DATA(0x03),
	MAKEUP_DATA(0x04),
	MAKEUP_CMD(0x03),
	MAKEUP_CMD(0x02),		
	MAKEUP_DATA(0x04),
	MAKEUP_DATA(0x03),
	MAKEUP_CMD(0x03),		
	MAKEUP_CMD(0x03),
	MAKEUP_DATA(0x03),
	MAKEUP_DATA(0x03),
	MAKEUP_CMD(0x03),
	MAKEUP_CMD(0x04),		
	MAKEUP_DATA(0x01),
	MAKEUP_DATA(0x01),
	MAKEUP_CMD(0x03),
	MAKEUP_CMD(0x05),		
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x04),
	MAKEUP_CMD(0x02),		
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x04),
	MAKEUP_CMD(0x03),		
	MAKEUP_DATA(0x01),
	MAKEUP_DATA(0x3F),	
	(LCD_CMD_WAIT |	0),	

	/* DISP RAM setting: 240*320 */
	MAKEUP_CMD(0x04),	/* HADDR, START 0 */
	MAKEUP_CMD(0x06),	
	MAKEUP_DATA(0x00), 
	MAKEUP_DATA(0x00),	/* x1,3 */
	MAKEUP_CMD(0x04),	/* HADDR,  END   4 */
	MAKEUP_CMD(0x07), 
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0xEF), 	/* x2, 7 */
	MAKEUP_CMD(0x04),	/* VADDR, START 8 */
	MAKEUP_CMD(0x08),	
	MAKEUP_DATA(0x00), 	/* y1, 10 */
	MAKEUP_DATA(0x00), 	/* y1, 11 */
	MAKEUP_CMD(0x04), 	/* VADDR, END 12 */
	MAKEUP_CMD(0x09),
	MAKEUP_DATA(0x01), 	/* y2, 14 */
	MAKEUP_DATA(0x3F), 	/* y2, 15 */
	MAKEUP_CMD(0x02),	/* RAM ADDR SETTING 16 */
	MAKEUP_CMD(0x00),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x00), 	/* x1, 19 */
	MAKEUP_CMD(0x02),	/* RAM ADDR SETTING 20 */
	MAKEUP_CMD(0x01),
	MAKEUP_DATA(0x00), 	/* y1, 22 */
	MAKEUP_DATA(0x00), 	/* y1, 23 */
};

static unsigned short turn_on_display[] = {
	/* Power-IC ON */
	MAKEUP_CMD(0x01),
	MAKEUP_CMD(0x02),
	MAKEUP_DATA(0x07),
	MAKEUP_DATA(0x7D),
	MAKEUP_CMD(0x01),
	MAKEUP_CMD(0x03),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x05),
	MAKEUP_CMD(0x01),
	MAKEUP_CMD(0x04),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x00),
	MAKEUP_CMD(0x01),
	MAKEUP_CMD(0x05),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x15),
	MAKEUP_CMD(0x01),
	MAKEUP_CMD(0x00),
	MAKEUP_DATA(0xC0),
	MAKEUP_DATA(0x10),	
	(LCD_CMD_WAIT |	30),		

	/* DISP ON */
	MAKEUP_CMD(0x01),
	MAKEUP_CMD(0x01),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x01),
	MAKEUP_CMD(0x01),
	MAKEUP_CMD(0x00),
	MAKEUP_DATA(0xFF),
	MAKEUP_DATA(0xFE),
	(LCD_CMD_WAIT |	150),		
};

static unsigned short turn_off_display[] = {
	MAKEUP_CMD(0x00),
	MAKEUP_CMD(0x1E),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x0A),
	MAKEUP_CMD(0x01),
	MAKEUP_CMD(0x00),
	MAKEUP_DATA(0xFF),
	MAKEUP_DATA(0xEE),
	MAKEUP_CMD(0x01),
	MAKEUP_CMD(0x00),
	MAKEUP_DATA(0xF8),
	MAKEUP_DATA(0x12),
	MAKEUP_CMD(0x01),
	MAKEUP_CMD(0x00),
	MAKEUP_DATA(0xE8),
	MAKEUP_DATA(0x11),
	MAKEUP_CMD(0x01),
	MAKEUP_CMD(0x00),
	MAKEUP_DATA(0xC0),
	MAKEUP_DATA(0x11),
	MAKEUP_CMD(0x01),
	MAKEUP_CMD(0x00),
	MAKEUP_DATA(0x40),
	MAKEUP_DATA(0x11),
	MAKEUP_CMD(0x01),
	MAKEUP_CMD(0x00),
	MAKEUP_DATA(0x00),
	MAKEUP_DATA(0x10),	
};

static unsigned short update_framedata[] = {
	/* write ram */
	MAKEUP_CMD(0x02),		
	MAKEUP_CMD(0x02),

	/* write frame data */
	LCD_CMD_FRAME_DATA_WRITE | LCD_CMD_A0_DATA,
};

static void ltm020d550_lcd_power(int on, struct fb_var_screeninfo *var)
{
	struct fb_info *fbi = container_of(var, struct fb_info, var);
	
	if (on) {
		if (pxafb_send_cmd(fbi, ARRAY_AND_SIZE(init_panel)))
			printk(KERN_ERR "LTM 020D550 init: failed!\n");
		if (pxafb_send_cmd(fbi, ARRAY_AND_SIZE(turn_on_display)))
			printk(KERN_ERR "LTM 020D550 display on: failed!\n");
	} else {
		if (pxafb_send_cmd(fbi, ARRAY_AND_SIZE(turn_off_display)))
			printk(KERN_ERR "LTM 020D550 display off: failed!\n");
	}
}

static int ltm020d550_update_framedata(struct fb_info *fbi)
{
	if (pxafb_send_cmd(fbi, ARRAY_AND_SIZE(update_framedata))) {
		printk(KERN_ERR "LTM 020D550 update frame data: failed!\n");
		return -EIO;
	} else {
		return 0;
	}
}

static struct pxafb_mode_info toshiba_ltm020d550_modes[] = {
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
		.update_framedata	= ltm020d550_update_framedata,
	},
};

static struct pxafb_mach_info tavorevb_lcd_info = {
	.modes			= toshiba_ltm020d550_modes,
	.num_modes		= 1,
	.lccr0			= LCCR0_Act,
	.lccr3			= LCCR3_PCP | LCCR3_HSP | LCCR3_VSP,
	.lccr6			= LCCR6_TWO_WD_SM,
	.pxafb_lcd_power	= ltm020d550_lcd_power,
};

#ifdef CONFIG_MACH_TAVOREVB_PIN_MUX
static mfp_cfg_t tavorevb_smart_lcd_cfg[] __initdata = {
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
	GPIO22_LCD_CS2,
	
	GPIO17_LCD_FCLK_RD,
	GPIO18_LCD_LCLK_A0,
	GPIO19_LCD_PCLK_WR,

	/* LCD Backlight */
	GPIO43_PRI_BKL,
	GPIO32_SEC_BKL,
};
#endif

static void __init tavorevb_init_lcd(void)
{
#ifdef CONFIG_MACH_TAVOREVB_PIN_MUX
	pxa3xx_mfp_config(ARRAY_AND_SIZE(tavorevb_smart_lcd_cfg));
#endif
	set_pxa_fb_info(&tavorevb_lcd_info);
}

static int espon_lcd = 0;
static int __init espon_lcd_setup(char *__unused)
{
	espon_lcd = 1;
	return 1;
}
__setup("espon_lcd", espon_lcd_setup);

inline int is_espon_lcd(void)
{
	return espon_lcd;
}

#define ESPON_LCD
#define ESPON_LCD_COMMAND 0x0 << 9
#define ESPON_LCD_PARAMETERS 0x1 << 9
#define ESPON_LCD_COMMAND_SLPOUT 0x11
#define ESPON_LCD_COMMAND_SLPIN 0x10
#define ESPON_LCD_COMMAND_DISON 0x29
#define ESPON_LCD_COMMAND_DISOFF 0x28

#define ESPON_CMD(x) (ESPON_LCD_COMMAND | x)
#define ESPON_PARA(x) (ESPON_LCD_PARAMETERS | x)

static u16 espon_slpout[] = {
	ESPON_CMD(ESPON_LCD_COMMAND_SLPOUT),
};

static u16 espon_dison[] = {
	ESPON_CMD(ESPON_LCD_COMMAND_DISON),
};

static u16 espon_disoff[] = {
	ESPON_CMD(ESPON_LCD_COMMAND_DISOFF),
};

static u16 espon_slpin[] = {
	ESPON_CMD(ESPON_LCD_COMMAND_SLPIN),
};

void espon_lcd_panel_reset()
{
	int err;                        

	mdelay(1);
	err = gpio_request(MFP_PIN_GPIO16, "espon_lcd_panel_reset");
	if (err) {
		gpio_free(MFP_PIN_GPIO16);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d return :%d\n", MFP_PIN_GPIO16, err);
		return;
	}
	gpio_direction_output(MFP_PIN_GPIO16, 0);
	mdelay(1);              
	gpio_direction_output(MFP_PIN_GPIO16, 1);               
	gpio_free(MFP_PIN_GPIO16);
	mdelay(10);
}

static void espon_ssp_config(struct ssp_dev *dev)
{
	ssp_disable(dev);
	ssp_config(dev, 0x00000588, 0x18, 0, 0);
}

static void ssp_send_cmd_para(u32 *cmd,int num)
{
	static struct ssp_dev ssp;
	int i;
	static int ssp_initialized = 0;

	if (!ssp_initialized) {
		ssp_init(&ssp, 1, SSP_NO_IRQ);                  
		espon_ssp_config(&ssp);
		ssp_initialized = 1;
	}

	clk_enable(ssp.ssp->clk);

	for (i = 0; i < num; i++, cmd++) {              
		ssp_write_word(&ssp, *cmd & 0x1ff);             

		/* FIXME: ssp_flush() is mandatory here to work */
		ssp_flush(&ssp);
	}
	clk_disable(ssp.ssp->clk);
}

static void espon_lcd_power(int on, struct fb_var_screeninfo *var)
{
	if (on) {
		espon_lcd_panel_reset();        

		/* SLPOUT*/ 
		ssp_send_cmd_para(ARRAY_AND_SIZE(espon_slpout));
		mdelay(60);

		/* DISON */
		ssp_send_cmd_para(ARRAY_AND_SIZE(espon_dison));

	} else {
		/* DISOFF  */
		ssp_send_cmd_para(ARRAY_AND_SIZE(espon_disoff));

		mdelay(60);

		/* SLPIN */ 
		ssp_send_cmd_para(ARRAY_AND_SIZE(espon_slpin));
	}
}

static struct pxafb_mode_info espon_l4s00242p00_modes[] = {
	[0] = {
		/* VGA */
		.pixclock       = 100000,
		.xres           = 480,
		.yres           = 640,
		.bpp            = 18,
		.hsync_len      = 20,
		.left_margin    = 60,
		.right_margin   = 41,
		.vsync_len      = 10,
		.upper_margin   = 9,
		.lower_margin   = 4,
		.sync           = 0,
	},
};

static struct pxafb_mach_info tavorevb_espon_lcd_info = {
	.modes                  = espon_l4s00242p00_modes,
	.num_modes              = 1,
	.lccr0                  = LCCR0_Act,
	.lccr3                  = LCCR3_HSP | LCCR3_VSP | LCCR3_PCP,
	.lccr4  	        = LCCR4_PAL_FOR_2, 
	.pxafb_lcd_power        = espon_lcd_power,
};

#ifdef CONFIG_MACH_TAVOREVB_PIN_MUX
static mfp_cfg_t tavorevb_espon_lcd_cfg[] __initdata = {
	/* tavorEVB dumb panel */
	GPIO16_GPIO,
	GPIO17_LCD_FCLK_RD,         
	GPIO18_LCD_LCLK_A0,        
	GPIO19_LCD_PCLK_WR, 
	GPIO20_LCD_BIAS,
	GPIO23_LCD_DD0,
	GPIO24_LCD_DD1,
	GPIO25_LCD_DD2,
	GPIO27_LCD_DD4,
	GPIO28_LCD_DD5,
	GPIO29_LCD_DD6,

	/* those pins conflict with USB2.0 */
	GPIO30_LCD_DD7,
	GPIO31_LCD_DD8,
	GPIO32_LCD_DD9,
	GPIO33_LCD_DD10,
	GPIO34_LCD_DD11,
	GPIO35_LCD_DD12,
	GPIO36_LCD_DD13,
	GPIO37_LCD_DD14,
	GPIO38_LCD_DD15,
	GPIO39_LCD_DD16,
	GPIO40_LCD_DD17,

	GPIO97_SSP1_CLK,
	GPIO98_SSP1_FRM,
	GPIO163_SSP1_TXD,
};
#endif

static void __init tavorevb_init_espon_lcd(void)
{
#ifdef CONFIG_MACH_TAVOREVB_PIN_MUX
	pxa3xx_mfp_config(ARRAY_AND_SIZE(tavorevb_espon_lcd_cfg));
#endif
	set_pxa_fb_info(&tavorevb_espon_lcd_info);
}
#else
static void __init tavorevb_init_lcd(void) {}
static void __init tavorevb_init_espon_lcd(void) {}
#endif /* CONFIG_FB_PXA || CONFIG_FB_PXA_MODULE */

#ifdef CONFIG_PM
static int tavor_init_wakeup(pm_wakeup_src_t *src)
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
	src->bits.dkey = 1;
	src->bits.eth = 1;
	src->bits.tsi = 1;
	return 0;
}

extern int pxa930_query_gwsr(int);
static int tavor_query_wakeup(unsigned int reg, pm_wakeup_src_t *src)
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
	if (reg & PXA3xx_PM_WE_GENERIC(9))
		src->bits.uart1 = 1;
	if (reg & PXA3xx_PM_WE_KP)
	        src->bits.mkey = 1;
	if (reg & PXA3xx_PM_WE_GENERIC(0))
	        src->bits.dkey = 1;
	if (reg & PXA3xx_PM_WE_GENERIC(3))
		src->bits.tsi = 1;
	if (reg & PXA3xx_PM_WE_GENERIC(13)) {
		if (pxa930_query_gwsr(47))
			src->bits.eth = 1;
	}
	return 0;
}

static int tavor_ext_wakeup(pm_wakeup_src_t src, int enable)
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

static int tavor_key_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.mkey) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO0_KP_MKIN_0), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO2_KP_MKIN_1), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO4_KP_MKIN_2), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO6_KP_MKIN_3), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO8_KP_MKIN_4), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO10_KP_MKIN_5), MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_KP;
		}
		if (src.bits.dkey) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO14_KP_DKIN_2), MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO15_KP_DKIN_3), MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(0);
		}
	} else {
		if (src.bits.mkey) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO0_KP_MKIN_0), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO2_KP_MKIN_1), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO4_KP_MKIN_2), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO6_KP_MKIN_3), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO8_KP_MKIN_4), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO10_KP_MKIN_5), MFP_EDGE_NONE);
		}
		if (src.bits.dkey) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO14_KP_DKIN_2), MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO15_KP_DKIN_3), MFP_EDGE_NONE);
		}
	}
	return ret;
}

static int tavor_mmc_wakeup(pm_wakeup_src_t src, int enable)
{
	return 0;
}

static int tavor_uart_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.uart1) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO53_FFRXD), MFP_EDGE_FALL);
			ret |= PXA3xx_PM_WE_GENERIC(9);
		}
	} else {
		if (src.bits.uart1) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO53_FFRXD), MFP_EDGE_NONE);
		}
	}
	return ret;
}

static int tavor_eth_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.eth) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO47_ETHER_IRQ),
				MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(13);
		}
	} else {
		if (src.bits.eth) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO47_ETHER_IRQ),
				MFP_EDGE_NONE);
		}
	}
	return ret;
}

static int tavor_tsi_wakeup(pm_wakeup_src_t src, int enable)
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
	.init	= tavor_init_wakeup,
	.query	= tavor_query_wakeup,
	.ext    = tavor_ext_wakeup,
	.key    = tavor_key_wakeup,
	.mmc    = tavor_mmc_wakeup,
	.uart   = tavor_uart_wakeup,
	.eth	= tavor_eth_wakeup,
	.tsi	= tavor_tsi_wakeup,
};
#endif
	
static mfp_cfg_t pxa930_eth_gpio_pins[] = {
	GPIO47_GPIO,
};

static mfp_cfg_t pxa935_eth_gpio_pins[] = {
#ifdef CONFIG_CPU_PXA935
	GPIO173_GPIO,
#endif
};

static void __init tavorevb_init_eth(void)
{
	int eth_irq;

	if (cpu_is_pxa935()) {
		eth_irq = IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO173));
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa935_eth_gpio_pins));
	} else {
		eth_irq = IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO47));
		pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa930_eth_gpio_pins));
	}
	smc91x_resources[1].start = eth_irq;
	smc91x_resources[1].end = eth_irq;
}


static void __init tavorevb_init(void)
{
#ifdef CONFIG_MACH_TAVOREVB_PIN_MUX
	/* initialize MFP configurations */
	pxa3xx_mfp_config(ARRAY_AND_SIZE(tavorevb_mfp_cfg));
#endif

	/* update ISRAM configration for pxa935 */
	if (cpu_is_pxa935())
		pxa3xx_device_imm.resource = pxa935_resource_imm;

	/* dvfm device */
	set_pxa3xx_freq_info(&tavorevb_freq_mach_info);

	/* performance monitor unit */
	pxa3xx_set_pmu_info(NULL);

	/* lcd */
	if (is_espon_lcd()) {
		pxa_register_device(&micco_bl_device, NULL);
		tavorevb_init_espon_lcd();
	} else {
		pxa_register_device(&pxa3xx_device_pwm_bl1, NULL);
		pxa_register_device(&pxa3xx_device_pwm_bl2, NULL);
		tavorevb_init_lcd();
	}

	/* ethernet */
	tavorevb_init_eth();

	platform_add_devices(devices, ARRAY_SIZE(devices));

	/* uart */
	pxa_set_ffuart_info(NULL);
	pxa_set_btuart_info(NULL);
	pxa_set_stuart_info(NULL);

	/* nand */
	tavorevb_init_nand();

	/* onenand */
	tavorevb_init_onenand();

#ifdef CONFIG_ISPT
	tavorevb_init_ispt();
#endif
#if defined(CONFIG_PXA_CAMERA)
	/* initialize camera */
	tavorevb_init_cam();
#endif

	/* i2c devices */
	i2c_register_board_info(0, ARRAY_AND_SIZE(tavorevb_i2c_board_info));

#if defined(CONFIG_USB_PXA3XX_U2D) || defined(CONFIG_USB_USB_PXA3XX_U2D_MODULE)
	/* u2d */
	if (device_is_enabled(device_u2d)) {
		if (!is_espon_lcd()) {
#ifdef CONFIG_MACH_TAVOREVB_PIN_MUX
			pxa3xx_mfp_config(ARRAY_AND_SIZE(tavorevb_u2d_cfg));
#endif
			pxa_set_u2d_info(&tavorevb_u2d_info);
		}
	}
#endif

#if defined(CONFIG_USB_PXA9XX_U2O) || defined(CONFIG_USB_USB_PXA9XX_U2O_MODULE)
	/* u2o */
	if (device_is_enabled(device_u2o)) {
		pxa_set_u2o_info(&tavorevb_u2o_info);
	}
#endif

#if defined(CONFIG_PXA_IRDA) || defined(CONFIG_PXA_IRDA_MODULE)
		pxa_set_ficp_info(&tavorevb_ficp_platform_data);
#endif
#if defined(CONFIG_KEYBOARD_PXA27x) || defined(CONFIG_KEYBOARD_PXA27x_MODULE)
	pxa_set_keypad_info(&tavorevb_keypad_info);
#endif
#if defined(CONFIG_USB_PXA27X_UDC) || defined(CONFIG_USB_USB_PXA27X_UDC_MODULE)
	if (device_is_enabled(device_udc)) {
		pxa_set_udc_info(&tavorevb_udc_info);
	}
#endif

	/* mmc */
	tavorevb_init_mmc();

	/* uhc */
	tavorevb_init_ohci();
	tavorevb_init_ehci();

#if defined(CONFIG_USB_OTG) && defined(CONFIG_USB_PXA9XX_U2O)
	/* u2otg */
	if (device_is_enabled(device_u2o)) {
		pxa_set_u2otg_info(&tavorevb_u2o_info);
	}
#endif

#ifdef CONFIG_PM
	pxa3xx_wakeup_register(&wakeup_ops);
#endif
}

MACHINE_START(TAVOREVB, "PXA9xx Evaluation Board (aka TavorEVB)")
	/* Maintainer: Eric Miao <eric.miao@marvell.com> */
	.phys_io        = 0x40000000,
	.boot_params    = 0xa0000100,
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io         = pxa_map_io,
	.init_irq       = pxa3xx_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = tavorevb_init,
MACHINE_END
