/*
 * arch/arm/mach-pxa/include/mach/pxa3xx-regs.h
 *
 * PXA3xx specific register definitions
 *
 * Copyright (C) 2007 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_PXA3XX_REGS_H
#define __ASM_ARCH_PXA3XX_REGS_H

/*
 * Power Manager
 */
#define ASCR            __REG(0x40F40000)       /* Application Subsystem Power Status/Control Register */
#define ARSR		__REG(0x40F40004)	/* Application Subsystem Reset Status Register */
#define AD3ER		__REG(0x40F40008)	/* Application Subsystem D3 state Wakeup Enable Register */
#define AD3SR		__REG(0x40F4000C)	/* Application Subsystem D3 state Wakeup Status Register */
#define AD2D0ER		__REG(0x40F40010)	/* Application Subsystem D2 to D0 state Wakeup Enable Register */
#define AD2D0SR		__REG(0x40F40014)	/* Application Subsystem D2 to D0 state Wakeup Status Register */
#define AD2D1ER		__REG(0x40F40018)	/* Application Subsystem D2 to D1 state Wakeup Enable Register */
#define AD2D1SR		__REG(0x40F4001C)	/* Application Subsystem D2 to D1 state Wakeup Status Register */
#define AD1D0ER		__REG(0x40F40020)	/* Application Subsystem D1 to D0 state Wakeup Enable Register */
#define AD1D0SR		__REG(0x40F40024)	/* Application Subsystem D1 to D0 state Wakeup Status Register */
#if defined(CONFIG_CPU_PXA935)
#define ACGD0ER		__REG(0x41340028)	/* Application Subsystem CG to D0 state Wakeup Status Register */
#define ACGD0SR		__REG(0x4134002C)	/* Application Subsystem CG to D0 state Wakeup Status Register */
#endif

#define ASDCNT		__REG(0x40F40028)	/* Application Subsystem SRAM Drowsy Count Register */
#define AGENP		__REG(0x40F4002C)	/* Application Subsystem General Purpose Register */
#define AD3R		__REG(0x40F40030)	/* Application Subsystem D3 State Configuration Register */
#define AD2R		__REG(0x40F40034)	/* Application Subsystem D2 State Configuration Register */
#define AD1R		__REG(0x40F40038)	/* Application Subsystem D1 State Configuration Register */

#define PMCR		__REG(0x40F50000)	/* Power Manager Control Register */
#define PSR		__REG(0x40F50004)	/* Power Manager S2 Status Register */
#define PSPR		__REG(0x40F50008)	/* Power Manager Scratch Pad Register */
#define PSBR		__REG(0x40F500A0)
#define PCFR		__REG(0x40F5000C)	/* Power Manager General Configuration Register */
#define PWER		__REG(0x40F50010)	/* Power Manager Wake-up Enable Register */
#define PWSR		__REG(0x40F50014)	/* Power Manager Wake-up Status Register */
#define PECR		__REG(0x40F50018)	/* Power Manager EXT_WAKEUP[1:0] Control Register */
#define OVH		__REG(0x40F50020)	/* Overheating Controller Register */
#define DCDCSR		__REG(0x40F50080)	/* DC-DC Controller Status Register */
#define AVCR		__REG(0x40F50094)  	/* VCC_APPS Voltage Control Register */
#define SVCR		__REG(0x40F50098)  	/* VCC_SRAM Voltage Control Register */
#define PVCR		__REG(0x40F50100)	/* Power Manager Voltage Change Control Register */

#define PMCR_BIE       (1 << 0)        /* Interrupt Enable for nBATT_FAULT */
#define PMCR_BIS       (1 << 1)        /* Interrupt Status for nBATT_FAULT */
#define PMCR_TIE       (1 << 10)       /* Interrupt Enable for XScale Core Frequency Change */
#define PMCR_TIS       (1 << 11)       /* Interrupt Status for XScale Core Frequency Change */
#define PMCR_VIE       (1 << 12)       /* Interrupt Enable for VCC_APPS and VCC_SRAM Voltage Change */
#define PMCR_VIS       (1 << 13)       /* Interrupt Status for VCC_APPS and VCC_SRAM Voltage Change */
#define PMCR_SWGR      (1 << 31)       /* Software GPIO Reset */

#define PSR_TSS_OFF	(12)		
#define OVH_TEMP_EN	(1 << 0)		/* Enable for Temperature Sensor */
#define OVH_OWM		(1 << 7)		/* Over-heating WDT Enable */

#define OVH_OTIF_OFF	(1)		/* Over-heating Treshold Value for Generating TIS Software Interrupt */
#define OVH_OVWF_OFF	(4)		/* WDT Reset Temperature Over-heating Threshold */
/*
 * Application Subsystem Clock
 */
#define ACCR		__REG(0x41340000)	/* Application Subsystem Clock Configuration Register */
#define ACSR		__REG(0x41340004)	/* Application Subsystem Clock Status Register */
#define AICSR		__REG(0x41340008)	/* Application Subsystem Interrupt Control/Status Register */
#define CKENA		__REG(0x4134000C)	/* A Clock Enable Register */
#define CKENB		__REG(0x41340010)	/* B Clock Enable Register */
#define AC97_DIV	__REG(0x41340014)	/* AC97 clock divisor value register */

#define ACCR_XPDIS		(1 << 31)	/* Core PLL Output Disable */
#define ACCR_SPDIS		(1 << 30)	/* System PLL Output Disable */
#define ACCR_D0CS		(1 << 26)	/* D0 Mode Clock Select */
#define ACCR_PCCE		(1 << 11)	/* Power Mode Change Clock Enable */
#define ACCR_DDR_D0CS		(1 << 7)	/* DDR SDRAM clock frequency in D0CS (PXA31x only) */

#if 0
#define ACCR_SMCFS_MASK		(0x7 << 23)	/* Static Memory Controller Frequency Select */
#define ACCR_SFLFS_MASK		(0x3 << 18)	/* Frequency Select for Internal Memory Controller */
#define ACCR_XSPCLK_MASK	(0x3 << 16)	/* Core Frequency during Frequency Change */
#define ACCR_HSS_MASK		(0x3 << 14)	/* System Bus-Clock Frequency Select */
#define ACCR_DMCFS_MASK		(0x3 << 12)	/* Dynamic Memory Controller Clock Frequency Select */
#define ACCR_XN_MASK		(0x7 << 8)	/* Core PLL Turbo-Mode-to-Run-Mode Ratio */
#define ACCR_XL_MASK		(0x1f)		/* Core PLL Run-Mode-to-Oscillator Ratio */
#endif

#define ACCR_SMCFS(x)		(((x) & 0x7) << 23)
#define ACCR_SFLFS(x)		(((x) & 0x3) << 18)
#define ACCR_XSPCLK(x)		(((x) & 0x3) << 16)
#define ACCR_HSS(x)		(((x) & 0x3) << 14)
#define ACCR_DMCFS(x)		(((x) & 0x3) << 12)
#define ACCR_XN(x)		(((x) & 0x7) << 8)
#define ACCR_XL(x)		((x) & 0x1f)

#if defined(CONFIG_CPU_PXA935)
#define ACCR1		__REG(0x41340020)	/* Application Subsystem Clock Configuration Register 1 */
#define CKENC		__REG(0x41340024)	/* C Clock Enable Register */
#define ACCR1_DIS_DRX		(1 << 31)	/* Disable DRX */
#define ACCR1_PU_OTG		(1 << 12)	/* USB 2.0 PHY OTG power up */
#define ACCR1_PU_PLL		(1 << 11)	/* USB 2.0 PHY PLL power up */
#define ACCR1_PU		(1 << 10)	/* USB 2.0 PHY power up */
#define ACCR1_I2C_33_52		(1 << 8)	/* I2C frequency control: 0 = 624/19 Mhz, 1 = 624/12 Mhz */
#define ACCR1_MMC6_48_52	(1 << 6)	/* MMC6 frequency control: 0 = 624/13 Mhz, 1 = 624/12 Mhz */
#define ACCR1_MMC5_48_52	(1 << 4)	/* MMC5 frequency control: 0 = 624/13 Mhz, 1 = 624/12 Mhz */
#define ACCR1_MMC4_48_52	(1 << 2)	/* MMC4 frequency control: 0 = 624/13 Mhz, 1 = 624/12 Mhz */
#define ACCR1_MMC3_48_52	(1 << 0)	/* MMC3 frequency control: 0 = 624/13 Mhz, 1 = 624/12 Mhz */
#endif

/*
 * Clock Enable Bit
 */
#define CKEN_LCD	1	/* < LCD Clock Enable */
#define CKEN_USBH	2	/* < USB host clock enable */
#define CKEN_CAMERA	3	/* < Camera interface clock enable */
#define CKEN_NAND	4	/* < NAND Flash Controller Clock Enable */
#define CKEN_USB2	6	/* < USB 2.0 client clock enable. */
#define CKEN_DMC	8	/* < Dynamic Memory Controller clock enable */
#define CKEN_SMC	9	/* < Static Memory Controller clock enable */
#define CKEN_ISC	10	/* < Internal SRAM Controller clock enable */
#define CKEN_BOOT	11	/* < Boot rom clock enable */
#define CKEN_MMC1	12	/* < MMC1 Clock enable */
#define CKEN_MMC2	13	/* < MMC2 clock enable */
#define CKEN_KEYPAD	14	/* < Keypand Controller Clock Enable */
#define CKEN_CIR	15	/* < Consumer IR Clock Enable */
#define CKEN_USIM0	17	/* < USIM[0] Clock Enable */
#define CKEN_USIM1	18	/* < USIM[1] Clock Enable */
#define CKEN_TPM	19	/* < TPM clock enable */
#define CKEN_UDC	20	/* < UDC clock enable */
#define CKEN_BTUART	21	/* < BTUART clock enable */
#define CKEN_FFUART	22	/* < FFUART clock enable */
#define CKEN_STUART	23	/* < STUART clock enable */
#define CKEN_AC97	24	/* < AC97 clock enable */
#define CKEN_TOUCH	25	/* < Touch screen Interface Clock Enable */
#define CKEN_SSP1	26	/* < SSP1 clock enable */
#define CKEN_SSP2	27	/* < SSP2 clock enable */
#define CKEN_SSP3	28	/* < SSP3 clock enable */
#define CKEN_SSP4	29	/* < SSP4 clock enable */
#define CKEN_MSL0	30	/* < MSL0 clock enable */
#define CKEN_PWM0	32	/* < PWM[0] clock enable */
#define CKEN_PWM1	33	/* < PWM[1] clock enable */
#define CKEN_I2C	36	/* < I2C clock enable */
#define CKEN_INTC	38	/* < Interrupt controller clock enable */
#define CKEN_GPIO	39	/* < GPIO clock enable */
#define CKEN_1WIRE	40	/* < 1-wire clock enable */
#define CKEN_HSIO2	41	/* < HSIO2 clock enable */
#define CKEN_HSIOGCU	42	/* < HSIOGCU clock enable */
#if defined(CONFIG_CPU_PXA310) || defined(CONFIG_CPU_PXA935)
#define CKEN_MVED	43	/* < MVED clock enable */
#endif
#define CKEN_MINI_IM	48	/* < Mini-IM */
#define CKEN_MINI_LCD	49	/* < Mini LCD */

#if defined(CONFIG_CPU_PXA310)
#define CKEN_MMC3	5	/* < MMC3 Clock Enable */
#endif

/* Note: GCU clock enable bit differs on PXA300/PXA310 and PXA320 */
#define CKEN_GCU_PXA300	42	/* Graphics controller clock enable */
#define CKEN_GCU_PXA320	7	/* Graphics controller clock enable */

#if defined(CONFIG_CPU_PXA935)
#define CKEN_SBUS1	61	/* < System bus 1 clock enable */
#define CKEN_MMC3_PXA940	65	/* < PXA940 MMC3 clock enable */
#define CKEN_MMC4_PXA940	66	/* < PXA940 MMC4 clock enable */
#define CKEN_MMC5_PXA940	67	/* < PXA940 MMC5 clock enable */
#define CKEN_U2O	70	/* < USB0 (OTG) Peripheral Clock Enable */
#define CKEN_U2H1	71	/* < USB1 Peripheral Clock Enable */
#define CKEN_U2H2	72	/* < USB2 Peripheral Clock Enable */
#define CKEN_U2H3	73	/* < USB3 Peripheral Clock Enable */
#define CKEN_U2O_AXI	74	/* < USB0 (OTG) AXI Clock Enable */
#define CKEN_U2H1_AXI	75	/* < USB1 AXI Clock Enable */
#define CKEN_U2H2_AXI	76	/* < USB2 AXI Clock Enable */
#define CKEN_U2H3_AXI	77	/* < USB3 AXI Clock Enable */
#define CKEN_MMC3_AXI	78	/* < PXA940 MMC3 AXI Clock Enable */
#define CKEN_MMC4_AXI	79	/* < PXA940 MMC4 AXI Clock Enable */
#define CKEN_MMC5_AXI	80	/* < PXA940 MMC5 AXI Clock Enable */
#define CKEN_IMU_AXI	82	/* < IMU (ISLAND_MMC_USB) AXI Clock Enable */
#define CKEN_AXI	83	/* < AXI Bus Clock Enable */
#endif

#define OSCC		__REG(0x41350000)	/* Oscillator Configuration Register */
#define OSCC_PEN	(1 << 11)
#define OSCC_TENS3	(1 << 10)
#define OSCC_TENS2	(1 << 9)
#define OSCC_TENS0	(1 << 8)

#define ADCD __REG(0x41c00000)  /*Analog-to-Digital Converter Data register */
#define ADCS __REG(0x41c00004)  /*Analog-to-Digital Converter Setup register */
#define ADCE __REG(0x41c00008)  /*Analog-to-Digital Converter Enable register */

/*
 * Interrupt Controller - specific for PXA3xx
 */
#define ICHP		__REG(0x40D00018)  /* Interrupt Controller Highest Priority Register */
#define IPR0		__REG(0x40D0001C)  /* Interrupt Priority Register 0 */
#define ICIP2		__REG(0x40D0009C)  /* Interrupt Controller IRQ Pending Register 2 */
#define ICMR2		__REG(0x40D000A0)  /* Interrupt Controller Mask Register 2 */
#define ICLR2		__REG(0x40D000A4)  /* Interrupt Controller Level Register 2 */
#define ICFP2		__REG(0x40D000A8)  /* Interrupt Controller FRQ Pending Register 2 */
#define ICPR2		__REG(0x40D000AC)  /* Interrupt Controller Pending Register 2 */
#define IPR32		__REG(0x40D000B0)  /* Interrupt Priority Register 32 */

/*
 * Multi Function Pin
 */
#define MFPR0		__REG(0x40E10000)  /* Multi Funciotn Pin Register 0 */

#endif /* __ASM_ARCH_PXA3XX_REGS_H */
