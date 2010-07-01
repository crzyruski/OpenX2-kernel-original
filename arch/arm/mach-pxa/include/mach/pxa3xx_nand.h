#ifndef __ASM_ARCH_PXA3XX_NAND_H
#define __ASM_ARCH_PXA3XX_NAND_H

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

struct pxa3xx_nand_platform_data {
	struct mtd_partition *parts;
	unsigned int    nr_parts;
};
#endif /* __ASM_ARCH_PXA3XX_NAND_H */

