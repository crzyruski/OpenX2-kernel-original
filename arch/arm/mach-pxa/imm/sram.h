/*
 *  linux/arch/arm/mach-pxa/imm/sram.h
 *
 *  Intel Memory Management
 *
 *  SRAM API Defines/Globals/Functions
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

#ifndef _ARCH_PXA_SRAM_H
#define _ARCH_PXA_SRAM_H

#include "imm.h"

/* imm internal flags */
#define IMM_INFO_USED           0x10
#define IMM_INFO_RESERVED   	0x20
#define IMM_INFO_DMA		IMM_MALLOC_HARDWARE
#define IMM_INFO_SRAM		IMM_MALLOC_SRAM

/* defines and macros */
#define PAGE_IS_MAPPED(x)	((x) & IMM_INFO_USED)
#define PAGE_IS_DMA(x)		((x) & IMM_INFO_DMA)

/* entries for the global physical memory list */
struct imm_page_t {
	u8 flag;		/* flags */
	u32 immid;		/* each page is associated with a process id */
	u32 virt_addr;		/* virtual address for this page (into the immid's space) */
	u32 phys_addr;		/* fixed physical address associated with this page */
#ifdef CONFIG_IMM_DEBUG
	s16 index;
#endif
};

/* global data */
extern u32 num_free_sram_pages;

#ifdef CONFIG_IMM_DEBUG
extern void print_page_map(int, struct imm_page_t **, s16);
extern void set_plist(int, int, int);
#endif

/* physical memory page list */
extern struct imm_page_t *page_list;
extern struct imm_info_t immid_list;

static inline void
page_map_reverse(struct imm_page_t **page_map, int n)
{
int i, j;
struct imm_page_t *p;

	for(i = 0, j = n - 1; i < j; i++, j--)
	{
		p = page_map[i];
		page_map[i] = page_map[j];
		page_map[j] = p;
	}
}
#endif

