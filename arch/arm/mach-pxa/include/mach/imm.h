/*
 *  linux/include/mach/imm.h
 *
 *  Intel Memory Management
 *
 *  User/Driver level IMM Defines/Globals/Functions
 *
 *  Todd Brandt
 *  Copyright (c) 2004, Intel Corporation (todd.e.brandt@intel.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.

 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef _ASM_ARCH_IMM_H
#define _ASM_ARCH_IMM_H
#include <linux/mm.h>

/* -------- Hardware specific capabilities --------------- */
#ifdef CONFIG_PXA3xx /* Intel Monahans Processor on Zylonite*/
/* Hardware properties of SRAM */
#define PHYS_SRAM_START		0x5c000000 /* physical start of SRAM */
#define PHYS_SRAM_BANK_SIZE	(128*1024) /* physical size of an SRAM bank */
#define PHYS_SRAM_PAGES_PER_BANK	(PHYS_SRAM_BANK_SIZE / PAGE_SIZE)

/* Hardware properties of DRAM */
 /* Zylonite uses Infineon HYB18M512160AF-7.5 512-Mbit Mobile-RAM chips */
#define PHYS_DDR_START		(PHYS_OFFSET)
#define PHYS_DDR_SIZE		(128*1024*1024)
 /* JEDEC Standard for PASR says chips must support up to 3/4 powerdown */
 /* 0 = upper 3/4 of each ddr chip max may be shut down if unused (JEDEC standard) */
 /* 1 = upper 7/8 of each ddr chip max may be shut down if unused (Extension 1) */
 /* 2 = upper 15/16 of each ddr chip max may be shut down if unused (Extension 2) */
#define PHYS_PASR_LEVEL		2	/* Infineon supports 1/16 */
/* Software Configurable properties of IMM controlled DRAM */
#define IMM_DRAM_ORDER		9 /* space to use as swap for SRAM */
#endif

#ifdef CONFIG_PXA27x /* Intel Bulverde Processor on Mainstone III */
/* Hardware properties of SRAM  */
#define PHYS_SRAM_BANK_SIZE	(64*1024)	/* physical size of an SRAM bank */
#define PHYS_SRAM_BANK_COUNT	4		/* Total SRAM banks */
#define PHYS_SRAM_START		0x5c000000	/* physical start of SRAM */
#define PHYS_SRAM_SIZE		(PHYS_SRAM_BANK_COUNT*PHYS_SRAM_BANK_SIZE)
#define PHYS_SRAM_PAGES_PER_BANK	(PHYS_SRAM_BANK_SIZE / PAGE_SIZE)
/* Software Configurable properties of IMM controlled SRAM */
#define IMM_SRAM_FIRST_BANK	0		/* first IMM SRAM bank */
#define IMM_SRAM_START		(PHYS_SRAM_START)

/* Hardware properties of DRAM */
 /* Mainstone III uses Infineon HYB25L256160AC-7.5 256-Mbit Mobile-RAM chips */
#define PHYS_DDR_START		(PHYS_OFFSET)
#define PHYS_DDR_SIZE		(64*1024*1024)
 /* JEDEC Standard for PASR says chips must support up to 3/4 powerdown */
 /* 0 = upper 3/4 of each ddr chip max may be shut down if unused (JEDEC standard) */
 /* 1 = upper 7/8 of each ddr chip max may be shut down if unused (Extension 1) */
 /* 2 = upper 15/16 of each ddr chip max may be shut down if unused (Extension 2) */
#define PHYS_PASR_LEVEL		2	/* Infineon supports 1/16 */
#endif

/*********************************
 *     Driver Registration API   *
 *********************************
 *
 *--	Function name: imm_register_kernel
 *	Description: Register with IMM in order to use the API functions, this must be
 *		done by any driver wishing to use IMM
 *	Output:
 *		[u32] A unique identifier which must be passed to all subsequent IMM calls
 *
 *	Example:
 *
 *		u32 my_immid = imm_register_kernel();
 *
 */

extern u32 imm_register_kernel(char *);


/*********************************
 *	SRAM Allocation API	 *
 *********************************
 *
 *--	Function name: imm_malloc
 *	Description: Allocate a block of internal memory
 *	Inputs:
 *		(int)size - the byte size of the requested memory block
 *		(int)flags - the type and organization of the memory block
 *			IMM_MALLOC_SRAM: forces SRAM
 *			IMM_MALLOC_HARDWARE: force contiguous, immovable physical pages
 *				this option is typically used for DMA memory, without it
 *				the pages allocated may be fragmented and may also
 *				be remapped for performance purposes by IMM internally
 *		(u32)immid - the immid of the caller, obtained from imm_register_kernel
 *	Output:
 *		[void *] the address of the allocated memory, or NULL on failure
 *
 *--	Function name: imm_free
 *	Description: Free a block of internal memory returned from imm_malloc
 *	Inputs:
 *		(void *)addr - the address of the block to free as returned from
 *				a previous call to imm_malloc
 *		(u32)immid - the immid of the caller, obtained from imm_register_kernel
 *
 *--	Function name: imm_get_physical
 *	Description: Retrieve the physical address from a virtual (PTE lookup)
 *	Inputs:
 *		(void *)address - the virtual address of some allocateed memory
 *		(u32)immid - the immid of the caller, obtained from imm_register_kernel
 *	Output:
 *		[u32] the physical address mapping, 0 if not found 0-mapped
 *
 *	Examples:
 *
 *		u32 my_immid = imm_register_kernel();
 *
 *		-- Allocate a 64K block of DMA SRAM --
 *		int *vaddress, paddress;
 *
 *		vaddress = (int *)imm_malloc(64*1024,
 *			IMM_MALLOC_SRAM | IMM_MALLOC_HARDWARE, my_immid);
 *		if(vaddress == NULL) printk(KERN_DEBUG "SRAM Allocation Failure: %s\n",
 *			imm_error(my_immid));
 *		paddress = imm_get_physical(vaddress, my_immid);
 *
 *		imm_free(vaddress, my_immid);
 *
 *
 *		-- Allocate a 1K block of SRAM --
 *		int *array = (int *)imm_malloc(256*sizeof(int), IMM_MALLOC_SRAM,
 *			my_immid);
 *		if(array == NULL) printk(KERN_DEBUG "SRAM Allocation Failure: %s\n",
 *			imm_error(my_immid));
 *
 *		imm_free(array, my_immid);
 *
 */

#define IMM_MALLOC_L2CACHE	0x08 /* target L2 Cache */
#define IMM_MALLOC_SRAM		0x01 /* target SRAM */
#define IMM_MALLOC_DRAM		0x02 /* target DRAM */
#define IMM_MALLOC_HARDWARE	0x04 /* force contiguous, immovable physical pages */

extern void * imm_malloc(u32 size, u32 flags, u32 immid);
extern int imm_free(void *address, u32 immid);
extern u32 imm_get_physical(void *address, u32 immid);
extern u32 imm_get_virtual(u32 address, struct task_struct *);
extern u32 imm_get_freespace(u32 flags, u32 immid);


/*********************************
 *      Error Information        *
 *********************************
 *
 *--	Function name: imm_error
 *	Description: Retrieve the error string from the last imm function call
 *	Inputs:
 *		(u32)immid - the immid of the caller, obtained from imm_register_kernel
 *
 *	Example:
 *
 *		u32 my_immid = imm_register_kernel();
 *
 *		printk(KERN_DEBUG "Last IMM Error: %s\n", imm_error(my_immid));
 *
 */

extern const char * imm_error(u32 immid);

#define IMM_CLIENT_NAME_SIZE            16

/* user level ioctl commands for accessing APIs */
#define IMM_MALLOC		0
#define IMM_FREE		1
#define IMM_CACHEFETCHLOCK	2
#define IMM_CACHEUNLOCK		3
#define IMM_SETPAGEATTR		4
#define IMM_ERROR		5
#define IMM_GET_PHYSICAL	6
#define IMM_GETPAGEATTR		7
#define IMM_CACHEINVAL		8
#define IMM_GET_VIRTUAL		9
#define IMM_CACHEALLOCLOCK	10
#define IMM_GET_PHYS_SRAM_SIZE  11
#define IMM_GET_VIRT_SRAM_SIZE  12
#define IMM_GET_FREE_SPACE	13

struct imm_memory_block {
	unsigned long *start;   /* the starting address of the block of memory */
	unsigned long size;     /* the size in bytes of the block of memory */
	unsigned long flag;     /* flags associated with this action */
};

/* ---- For IMM Application level usage, cut below this line ---- */

#ifdef CONFIG_IMM_DEBUG
#define imm_debug(s, args...)   printk(KERN_DEBUG s, ## args)
#else
#define imm_debug(s, args...)
#endif
#define imm_failure(s)   printk(KERN_ERR "%s: file %s, line %d\n", s, __FILE__, __LINE__)

/* register a user level client */
extern u32 imm_register_user(u32 vstart, u32 pid);

extern int phys_sram_size;
extern int imm_malloc_map_size;
#endif


