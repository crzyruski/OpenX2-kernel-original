#ifndef __ASM_ARCH_TAVOREVB_H
#define __ASM_ARCH_TAVOREVB_H

#define TAVOREVB_ETH_PHYS	0x14000000

#define ULPI_RESET_PIN		53		/* DF_ADDR0 to GPIO53 */
#define PMIC_INT_PIN		83		/* PMIC_INT to GPIO83 */

/*
 * QCI HI & LO sensor enable
 */
#define QCI_HI_PWDN_PIN         mfp_to_gpio(MFP_PIN_GPIO63)
#define QCI_LO_PWDN_PIN         mfp_to_gpio(MFP_PIN_GPIO64)

struct platform_mmc_slot {
	int gpio_cd;
	int gpio_wp;
};

extern struct platform_mmc_slot tavorevb_mmc_slot[];

#endif /* __ASM_ARCH_TAVOREVB_H */

