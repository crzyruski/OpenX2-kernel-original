/*
 *  arch/arm/mach-pxa/include/mach/memory.h
 *
 * Author:	Nicolas Pitre
 * Copyright:	(C) 2001 MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/*
 * Physical DRAM offset.
 */
#define PHYS_OFFSET	UL(0xa0000000)

/* Override the ARM default */
#define CONSISTENT_DMA_SIZE	8 * 1024 * 1024

/*
 * Virtual view <-> DMA view memory address translations
 * virt_to_bus: Used to translate the virtual address to an
 *		address suitable to be passed to set_dma_addr
 * bus_to_virt: Used to convert an address for DMA operations
 *		to an address that the kernel can use.
 */
#define __virt_to_bus(x)	 __virt_to_phys(x)
#define __bus_to_virt(x)	 __phys_to_virt(x)

#ifndef CONFIG_DISCONTIGMEM
/*
 * The nodes are matched with the physical SDRAM banks as follows:
 *
 * 	node 0:  0xa0000000-0xa3ffffff	-->  0xc0000000-0xc3ffffff
 * 	node 1:  0xa4000000-0xa7ffffff	-->  0xc4000000-0xc7ffffff
 * 	node 2:  0xa8000000-0xabffffff	-->  0xc8000000-0xcbffffff
 * 	node 3:  0xac000000-0xafffffff	-->  0xcc000000-0xcfffffff
 *
 * This needs a node mem size of 26 bits.
 */
#define NODE_MEM_SIZE_BITS	26
#else
/*
 * The nodes are matched with the physical SDRAM banks as follows:
 *
 * 	node 0:  0xa0000000-0xa7ffffff	-->  0xc0000000-0xc7ffffff
 * 	node 1:  0xc0000000-0xc7ffffff	-->  0xc8000000-0xcfffffff
 *
 * This needs a node mem size of 27 bits.
 */
#define __virt_to_phys(x)	((x) - PAGE_OFFSET + PHYS_OFFSET \
	+ (((x) & 0x08000000) << 2) - ((x) & 0x08000000))
#define __phys_to_virt(x)	((x) - PHYS_OFFSET + PAGE_OFFSET \
	+ (((x) & 0x40000000) >> 3) - (((x) & 0x40000000) >> 1))

#define NODE_MEM_SIZE_MASK	((1 << 27) - 1)

/*
 * Given a kernel address, find the home node of the underlying memory.
 */
#define KVADDR_TO_NID(addr) \
	(((unsigned long)(addr) - PAGE_OFFSET) >> 27)

/*
 * Given a page frame number, convert it to a node id.
 */
#define PFN_TO_NID(pfn) \
	(((pfn) - PHYS_PFN_OFFSET) >> (29 - PAGE_SHIFT))

/*
 * Given a kaddr, LOCAL_MEM_MAP finds the owning node of the memory
 * and returns the index corresponding to the appropriate page in the
 * node's mem_map.
 */
#define LOCAL_MAP_NR(addr) \
	(((unsigned long)(addr) & NODE_MEM_SIZE_MASK) >> PAGE_SHIFT)

#endif
#if !defined(__ASSEMBLY__) && defined(CONFIG_MACH_ARMCORE) && defined(CONFIG_PCI)
void cmx2xx_pci_adjust_zones(int node, unsigned long *size,
			     unsigned long *holes);

#define arch_adjust_zones(node, size, holes) \
	cmx2xx_pci_adjust_zones(node, size, holes)

#define ISA_DMA_THRESHOLD	(PHYS_OFFSET + SZ_64M - 1)
#endif

#endif
