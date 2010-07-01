#ifndef __ASM_ARCH_SAAR_H
#define __ASM_ARCH_SAAR_H

#define SAAR_ETH_PHYS	0x14000000

/*
 * QCI HI & LO sensor enable
 */
#define QCI_HI_PWDN_PIN         mfp_to_gpio(MFP_PIN_GPIO77)
#define QCI_LO_PWDN_PIN         mfp_to_gpio(MFP_PIN_GPIO78)

struct platform_mmc_slot {
	int gpio_cd;
	int gpio_wp;
};
extern struct platform_mmc_slot saar_mmc_slot[];

#endif /* __ASM_ARCH_SAAR_H */
