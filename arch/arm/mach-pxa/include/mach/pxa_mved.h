/*  pxa_mved - main file for mved driver
 *
 *  Copyright (C) 2006, Intel Corporation.
 *  Copyright (C) 2007, Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */ 

#ifndef _PXA_MVED_H_
#define _PXA_MVED_H_
#include <asm/ioctl.h>
#ifdef __KERNEL__
#include <linux/list.h>
#include <mach/pxa-regs.h>
#include <mach/irqs.h>
#else

struct list_head {
	struct list_head *next, *prev;
};

#define INIT_LIST_HEAD(ptr) do { \
		(ptr)->next = (ptr); (ptr)->prev = (ptr); \
		} while (0)

static inline void __list_add(struct list_head *new1,
			      struct list_head *prev,
			      struct list_head *next)
{
	next->prev = new1;
	new1->next = next;
	new1->prev = prev;
	prev->next = new1;
}

static inline void list_add_tail(struct list_head *new1, struct list_head *head)
{
	__list_add(new1, head->prev, head);
}

static inline int list_empty(const struct list_head *head)
{
	return head->next == head;
}

static inline void __list_del(struct list_head * prev, struct list_head * next)
{
	next->prev = prev;
	prev->next = next;
}

static inline void list_del(struct list_head *entry)
{
	__list_del(entry->prev, entry->next);
	entry->next = 0;
	entry->prev = 0;
}

#endif

#ifndef PAGE_SHIFT
#define PAGE_SHIFT		12
#define PAGE_SIZE		(1UL << PAGE_SHIFT)
#define PAGE_MASK		(~(PAGE_SIZE-1))
#endif

#define MM_FLAG_PHYCONT		0x00000001
#define MM_FLAG_KVALID		0x00000002
#define MM_FLAG_UVALID		0x00000004
#define MM_FLAG_PVALID		0x00000008
#define MM_FLAG_LVALID		0x00000010
#define MM_FLAG_SURRES		0x10000000
#define MM_FLAG_DEVRES		0x20000000
#define MM_FLAG_BMM		0x40000000
#define MM_FLAG_KMALLOC		0x80000000

typedef struct MEMORY_INFO_TAG
{
	void 		*k_addr;
	void 		*u_addr;
	void 		*p_addr;
	unsigned long  	len;
	unsigned int   	flag;		
} MEMORY_INFO;

struct memblk_info {
	int  		len;
	void 		*k_addr;
	void 		*u_addr;
	void 		*p_addr;
	unsigned int 	type;
	void		*vma;
};


struct mem_dma_t {
	unsigned long 	srcuseraddr;
	unsigned long 	taruseraddr;
	unsigned long 	len;
};

struct mb_wait_t {
	unsigned long 	wait;
	unsigned long 	finish;
	unsigned long 	timeout;
};

#ifndef INFINITE
#define INFINITE 			0xffffffff
#endif

#define MVED_DEBUGCMD_PRTMEM		0
#define MVED_DEBUGCMD_INITMEM		1

#define MVED_REG_PHYBASE   		0x56000000
#define MVED_DMA_REG_PHYBASE   		0x56100000
#define MVED_BPB2IMG_REG_PHYBASE	0x56200000

#define MVED_REG_REGIONLEN 		0x40000

#define MVED_SURFACE_RES_MEM_LEN 	(8*1024*1024)
#define MVED_DEVICE_RES_MEM_LEN 	(2*1024*1024)

#define MVED_MMAP_MVED_REGS		(1<<PAGE_SHIFT)
#define MVED_MMAP_MALLOC		(2<<PAGE_SHIFT)
#define MVED_MMAP_MVED_DMA_REGS		(5<<PAGE_SHIFT)
#define MVED_MMAP_BPB2IMG_REGS		(6<<PAGE_SHIFT)

#define MVED_MMAP_CMD_MASK 		(0xff<<PAGE_SHIFT)
#define MVED_MMAP_CACHE_MASK		(0x100<<PAGE_SHIFT) 
#define MVED_MMAP_SRAM_MASK		(0x200<<PAGE_SHIFT) 
#define MVED_MMAP_SURFACEMEM_MASK	(0x400<<PAGE_SHIFT) 
#define MVED_MMAP_DEVICEMEM_MASK	(0x800<<PAGE_SHIFT) 
#define MVED_MMAP_WRCOMB_MASK		(0x1000<<PAGE_SHIFT)

#define MVED_S_FLUSHCACHE   		_IOWR('v', 810, struct MEMORY_INFO_TAG)
#define MVED_S_MTX_WAIT			_IOWR('v', 930, unsigned int)
#define MVED_S_MVEA_WAIT		_IOW('v', 931, int)
#define MVED_S_POWER_ON			_IOW('v', 940, int)
#define MVED_S_POWER_OFF		_IOW('v', 941, int) 
#define MVED_S_POWER_QUERY		_IOWR('v', 942, int)
#define MVED_S_RESET			_IOW('v', 943, int)
#define MVED_S_DISPLAY_DMA_ISSUE 	_IOW('v', 950, struct mem_dma_t)
#define MVED_S_DISPLAY_DMA_SYNC		_IOW('v', 951, int) 
#define MVED_S_UPDATEMEMINFO		_IOWR('v', 960, struct MEMORY_INFO_TAG) 
#define MVED_S_DMA_WAITLAST		_IOWR('v', 991, unsigned int)
#define MVED_S_DEC_MB_SET		_IOW('v', 992, unsigned int)
#define MVED_S_DEC_MB_WAIT		_IOWR('v', 993, struct mb_wait_t)
#define MVED_S_DEC_MB_ENABLE		_IOWR('v', 994, int)
 
#endif


