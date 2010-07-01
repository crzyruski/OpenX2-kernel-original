#ifndef ASMARM_ARCH_EHCI_H
#define ASMARM_ARCH_EHCI_H

struct device;

struct pxaehci_platform_data {
	int (*init)(struct device *);
	void (*exit)(struct device *);

	int port_mode;
#define PMM_NPS_MODE           1
#define PMM_GLOBAL_MODE        2
#define PMM_PERPORT_MODE       3

	int power_budget;
};

extern void pxa_set_ehci_info(struct pxaehci_platform_data *info);

#endif
