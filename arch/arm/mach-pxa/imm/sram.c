/*
 *  linux/arch/arm/mach-pxa/imm/sram.c
 *
 *  Intel Memory Management
 *
 *  SRAM Allocation API
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/linkage.h>
#include <linux/proc_fs.h>
#include <linux/compiler.h>
#include <linux/clk.h>

#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/pgalloc.h>
#include <asm/io.h>

#include <mach/pxa3xx-regs.h>

#include "sram.h"

int imm_sram_pages;
int imm_malloc_map_size;

extern int phys_sram_start;
extern int phys_sram_size;
extern int imm_sram_start;
extern int imm_sram_size;

#ifdef CONFIG_IMM_DEBUG
extern char *imm_errors[];
#endif

/* flag showing that this module was initialized */
static int module_initialized = 0;

u32 num_free_sram_pages;

/* kernel level virtual memory list ordered only by allocation start address */
struct imm_virt_t malloc_list = { 0, 0, 0, &malloc_list, &malloc_list };

/* physical memory page list */
struct imm_page_t *page_list;

int page_compatible(u32 start, u32 flags, u32 immid);

/*----------------------------------------------------------------------
 *
 *  High Level Call Tree:
 *
 *  [Application or Driver Call]
 *	imm_malloc
 *		page_compatible
 *		imm_alloc_pages
 *			imm_map_page
 *	imm_free
 *		release_entry
 *			imm_free_pages
 *				imm_unmap_page
 *	imm_get_freespace
 *
 *  [Process Exit - from Policy Manager]
 * 	imm_free_immid
 *		release_immid
 * 		imm_free_pages_immid
 *
 *  [Proc Filesystem]
 * 	/proc/imm/sram_vmap (read-only)
 * 		imm_sram_vmap_read_proc
 * 	/proc/imm/sram_pages (read-only)
 * 		imm_sram_pages_read_proc
 * 	/proc/imm/sram_usagemap (read-only)
 * 		imm_sram_usagemap_read_proc
 *
 *  [Module Initialization - from Policy Manager]
 *	imm_dau_sram_init
 *
 *--------------------------------------------------------------------*/

#ifdef CONFIG_IMM_DEBUG
extern struct imm_info_t immid_list;
char *plist;

void set_plist(int s, int d, int c)
{
	static char plist_ch = 'A';

	switch(c) {
	case 0: /* map */
		plist[s] = plist_ch;
		break;
	case 1: /* unmap */
		plist[s] = '.';
		break;
	case 2: /* remap */
		plist[d] = plist[s];
		plist[s] = '.';
		break;
	case 3: /* swap */
		plist[d] = plist[s];
		plist[s] = '.';
		break;
	case 4: /* update char */
		plist_ch++;
		if (plist_ch > 'Z') plist_ch = 'A';
		break;
	}
}

void print_page_list(char *s)
{
	int i, count, space=0;
	struct imm_page_t *p;

	if (!strcmp(s, "malloc")) set_plist(-1, -1, 4);

	printk(KERN_DEBUG "Page list changed as a result of %s\n", s);

	space = num_free_sram_pages;
	printk(KERN_DEBUG "%d pages total, %d free\n",
		&page_list[imm_sram_pages]-&page_list[0], space);
	for (count = 0, i = 0, p = &page_list[0]; p < &page_list[imm_sram_pages]; p++, i++) {
		if (!PAGE_IS_MAPPED(p->flag)) {
			count++;
		}
		printk(KERN_DEBUG "%c", plist[p->index]);
		if ((i+1)%64 == 0) printk(KERN_DEBUG "\n");
	}
	if (i%64) printk(KERN_DEBUG "\n");
	if (count != space)
		printk(KERN_WARNING "Free Count Mismatch, actual free pages is %d\n",
			count);
}

void print_page_map(int num, struct imm_page_t **page_map, s16 num_pages)
{
	int i;

	printk(KERN_DEBUG "Scan %d reveals this page_map:\n", num);
	for (i = 0; i < num_pages; i++) {
		printk(KERN_DEBUG "%03d:%03d ", i, (page_map[i])?page_map[i]->index:-1);
		if ((i+1)%9 == 0) printk(KERN_DEBUG "\n");
	}
	if ((i+1)%9) printk(KERN_DEBUG "\n");

}
#endif

/*************************************************************************
 *
 * IMM Single Page Functions
 *
 * imm_map_page: create a new virtual-physical page mapping
 * imm_unmap_page: remove a virtual-physical page mapping
 *
 *************************************************************************/
void imm_map_page(struct imm_page_t *page, u32 virtual, u32 flags,
		struct imm_info_t *imm_info)
{
	struct mm_struct *mm = imm_info->mm;
	struct vm_area_struct *vma;
	pgd_t *dir;
	u32 f;

#ifdef CONFIG_IMM_DEBUG
	set_plist(page->index, -1, 0);
#endif

	num_free_sram_pages--;

	page->flag |= IMM_INFO_USED | (flags & IMM_INFO_DMA);
	page->flag &= ~IMM_INFO_RESERVED;
	page->immid = imm_info->immid;
	page->virt_addr = virtual;

	/* user space mappings are handled differently from kernel */
	if (IMMID_USER(imm_info->immid)) {
		vma = find_vma(mm, page->virt_addr);
		if (unlikely(vma == NULL)) {
			imm_failure("find_vma failed");
			return;
		}

		/* first destroy the existing mapping */
		zap_page_range(vma, page->virt_addr, PAGE_SIZE, NULL);

		/* the initial cacheability depends on the HARDWARE flag */
		if (PAGE_IS_DMA(flags))
			f = PAGE_SHARED & (~L_PTE_CACHEABLE);
		else
			f = PAGE_SHARED;

		/* now map in the underlying physical memory */
		if (io_remap_pfn_range(vma, page->virt_addr, (page->phys_addr)
			>> PAGE_SHIFT, PAGE_SIZE, f))
			imm_failure("io_remap_pfn_range failed");
	} else {
		dir = pgd_offset_k(page->virt_addr);

		/* the initial cacheability depends on the HARDWARE flag */
		if (PAGE_IS_DMA(flags))
			f = L_PTE_BUFFERABLE | L_PTE_EXEC;
		else
			f = L_PTE_BUFFERABLE | L_PTE_CACHEABLE;

		/* now map in the underlying physical memory */
		if (remap_area_pages(page->virt_addr, (page->phys_addr)
			>> PAGE_SHIFT, PAGE_SIZE, f))
			imm_failure("remap_area_pages failed");
	}
}

void imm_unmap_page_lite(struct imm_page_t *page, struct imm_info_t *imm_info)
{

#ifdef CONFIG_IMM_DEBUG
	set_plist(page->index, -1, 1);
#endif

	num_free_sram_pages++;

	page->flag &= IMM_INFO_SRAM;
	page->immid = 0;
	page->virt_addr = 0;
}

extern void unmap_kernel_range(unsigned long addr, unsigned long size);

void imm_unmap_page(struct imm_page_t *page, struct imm_info_t *imm_info)
{
	struct vm_area_struct *vma;

	/* if the page isn't mapped, forget it */
	if (PAGE_IS_MAPPED(page->flag)) {
		if (IMMID_USER(imm_info->immid)) {
			vma = find_vma(imm_info->mm, page->virt_addr);
			if (unlikely(vma == NULL)) {
				imm_failure("find_vma failed\n");
				return;
			}
			zap_page_range(vma, page->virt_addr, PAGE_SIZE, NULL);
		} else {
			unmap_kernel_range(page->virt_addr, PAGE_SIZE);
		}
	}
	imm_unmap_page_lite(page, imm_info);
}

/*************************************************************************
 *
 * IMM Page Range Functions
 *
 * imm_alloc_pages: given a valid virtual address range, map available
 *   physical pages.
 * imm_free_pages: frees a specific range of physical pages
 * imm_free_pages_immid: frees all physical pages for a given immid
 *
 *************************************************************************/

int imm_alloc_pages(u32 virt, u32 size, u32 flags, struct imm_info_t *imm_info)
{
	int i;
	u32 virt_start = virt&PAGE_MASK;
	u32 virt_end = PAGE_ALIGN(virt+size);
	u32 pstart, pend, immid = imm_info->immid;
	u32 pages_needed, pages_found = 0;
	struct imm_page_t **page_map, **page_map_ptr, *p;

	imm_debug("imm_alloc_pages: virt=%08X, size=%d, flags=%03X, \
			immid=%d\n", virt, size, flags, immid);

	if ((virt%PAGE_SIZE)&&page_compatible(virt, flags, immid))
		pages_needed = (virt_end-(virt_start+PAGE_SIZE))/PAGE_SIZE;
	else
		pages_needed = (virt_end-virt_start)/PAGE_SIZE;

	/* if there aren't enough pages left leave now */
	if (pages_needed  > num_free_sram_pages)
		return IMM_ERROR_NOSPACE;

	/* create a physical page mapping for the new allocation */
	if ((page_map_ptr = kmalloc((pages_needed+2)
			*sizeof(struct imm_page_t *), GFP_ATOMIC)) == NULL)
		return IMM_ERROR_KMALLOC;

	/* add two extra places in case we find border pages */
	for (i = 0; i < pages_needed+2; i++) {
		page_map_ptr[i] = NULL;
	}
	page_map = &page_map_ptr[1];

	if (!PAGE_IS_DMA(flags)) {
		/* look for new and border pages */
		for (p = &page_list[0]; (p < &page_list[imm_sram_pages])&&
			(pages_found < pages_needed); p++)
		{
			/* check all memory types for border pages */
			if (PAGE_IS_MAPPED(p->flag)&&
				!PAGE_IS_DMA(p->flag)&&
				(p->immid == immid))
			{
				pstart = p->virt_addr;
				pend = p->virt_addr + PAGE_SIZE;
				if (virt_start == pstart) {
					pages_found++;
					page_map_ptr[0] = p;
				} else if (virt_end == pend) {
					pages_found++;
					page_map_ptr[pages_needed+1] = p;
				}
			}
			if (!PAGE_IS_MAPPED(p->flag)) {
				page_map[pages_found++] = p;
				p->flag |= IMM_INFO_RESERVED;
			}
		}

		/* we may have found up to two more than pages_needed */
		pages_found = min(pages_found, pages_needed);

		/* if we have found a starting border page, move the map forward */
		if (page_map_ptr[0]) page_map[0] = page_map_ptr[0];

		/* if we have found an ending border page, overwrite the last page
		 * with it
		 */
		if (page_map_ptr[pages_needed+1])
			page_map[pages_needed-1] = page_map_ptr[pages_needed+1];
	} else {
		/* next scan to pick up needed new pages */
		for (p = &page_list[0], pages_found = 0;
			(p < &page_list[imm_sram_pages])&&(pages_found < pages_needed);
			p++)
		{
			if (!PAGE_IS_MAPPED(p->flag))
				page_map[pages_found++] = p;
			else
				pages_found = 0;
		}
	}

	if (pages_found < pages_needed)
	{
		kfree(page_map_ptr);
		return IMM_ERROR_NOSPACE;
	}

#ifdef CONFIG_IMM_DEBUG
	for (i = 0; i < pages_needed; i++) {
		printk(KERN_DEBUG "V(0x%08X) --> P(0x%08X), page index = %03d, %s\n",
			(u32)(virt_start+(i*PAGE_SIZE)), page_map[i]->phys_addr,
			page_map[i]->index,
			(PAGE_IS_MAPPED(page_map[i]->flag))?"USED":"NEW");
	}
#endif

	for (i = 0; i < pages_needed; i++)
	{
		p = page_map[i];
		if (!PAGE_IS_MAPPED(p->flag))
		{
			imm_map_page(p, virt_start+(i*PAGE_SIZE),
				flags, imm_info);
		}
	}

	/* flush the cache and TLBs afterward to ensure the mapping took */
	flush_cache_all();
	flush_tlb_all();

	kfree(page_map_ptr);
	return IMM_ERROR_NONE;
}

void
imm_free_pages(u32 virt, u32 size, struct imm_info_t *imm_info)
{
	struct imm_page_t *p;
	u32 virt_start = virt&PAGE_MASK;
	u32 virt_end = PAGE_ALIGN(virt+size);
	s16 pages_freed, pages_found = 0;

	pages_freed = (virt_end - virt_start) / PAGE_SIZE;
	if (unlikely(!pages_freed)) {
		imm_failure("imm_free_pages called with 0 size");
		return;
	}

	/* first scan to pick up needed pages that are already mapped */
	for (p = &page_list[0]; (p < &page_list[imm_sram_pages])&&
		(pages_found < pages_freed); p++)
	{
		if (PAGE_IS_MAPPED(p->flag)
			&& (p->immid == imm_info->immid)
			&& ((p->virt_addr >= virt_start)
			&& (p->virt_addr < virt_end)))
		{
			imm_unmap_page(p, imm_info);
			pages_found++;
		}
	}

	flush_cache_all();
	flush_tlb_all();
}

int
imm_free_pages_immid(struct imm_info_t *imm_info)
{
	struct imm_page_t *p;
	u32 immid = imm_info->immid;

	/* first scan to pick up needed pages that are already mapped */
	for (p = &page_list[0]; p < &page_list[imm_sram_pages]; p++)
	{
		if (PAGE_IS_MAPPED(p->flag)&&
			(p->immid == immid))
		{
			imm_unmap_page_lite(p, imm_info);
		}
	}

	flush_cache_all();
	flush_tlb_all();

	return 0;
}

/*************************************************************************
 * virtual allocation list handlers
 *
 * release_entry: frees a specific malloc, or cacheability entry
 * release_immid: frees all instances of mallocs, hotswaps, or cacheability
 *   changes for a given immid.
 * page_compatible: (kernel pages only) determines if a given page is
 *   compatible with a new allocation's flags, this is necessary if a new
 *   allocation shares a page with an existing allocation
 *************************************************************************/

int release_entry(void *address, struct imm_info_t *imm_info)
{
	struct imm_virt_t *head = imm_info->malloc_list,
			*p = imm_info->malloc_list->next;
	u32 start=0, end=0, immid = imm_info->immid;
	int found = 0;

	while (p != head) {
		if ((p->immid == immid)&&(p->start == (u32)address)) {
			/* calculate the page address range */
			start = p->start&PAGE_MASK;
			end = PAGE_ALIGN(p->end);

			/* we need to see if other allocations overlap
			in the beginning or end pages of the range */
			if ((p->prev->immid == immid)&&
				(p->prev->end > start)) {
				start += PAGE_SIZE;
			}
			if ((p->next->immid == immid)&&
				(p->next->start < end)) {
				end -= PAGE_SIZE;
			}
			found = 1;
			break;
		}
		p = p->next;
	}

	if (!found) return IMM_ERROR_ENTRY_NOTFOUND;

	imm_info->used_space -= p->end - p->start;

	/* p now points to the memory block to be removed */
	p->prev->next = p->next;
	p->next->prev = p->prev;
	kfree(p);

	if (start < end) {
		imm_free_pages(start, end-start, imm_info);
	}
	return IMM_ERROR_NONE;
}

int release_immid(struct imm_info_t *imm_info)
{
	struct imm_virt_t *head = imm_info->malloc_list,
			*p = imm_info->malloc_list->next;
	u32 immid = imm_info->immid;

	while (p != head) {
		if (p->immid == immid) {
			/* p now points to the memory block to be removed */
			p->prev->next = p->next;
			p->next->prev = p->prev;
			kfree(p);
		}
		p = p->next;
	}
	return 0;
}

int page_compatible(u32 start, u32 flags, u32 immid)
{
	u32 vstart = start&PAGE_MASK;
	struct imm_page_t *p;

	/* if the dma flag is set, the page is not compatible */
	/* dma pages can't be moved, non-dma pages can, so */
	/* allocations can't share both */
	if (PAGE_IS_DMA(flags)) {
		return 0;
	}

	/* find the page in question */
	for (p = &page_list[0]; p < &page_list[imm_sram_pages]; p++) {
		if (IMMID_OVERLAP(p->immid, immid)&&
			PAGE_IS_MAPPED(p->flag)&&
			((p->virt_addr&PAGE_MASK) == vstart))
			break;
	}

	/* if the page isn't found, it's not compatible (shouldn't */
	/* occur, but just to be safe handle the error) */
	if (p >= &page_list[imm_sram_pages]) {
		return 0;
	}

	/* if the dma flag is set, the page is not compatible */
	/* dma pages can't be moved, non-dma pages can, so */
	/* allocations can't share both */
	if (PAGE_IS_DMA(p->flag)) {
		return 0;
	}

	return 1;
}

/*************************************************************************
 * Function: imm_malloc
 * Description: Allocates a block and returns the kernel/user space
 * address
 *
 * Arguments:
 * size - the size in bytes of the requested allocation
 * flags - malloc flags
 * immid - the id to assign this malloc to
 *
 * Return Value: the new allocation address (NULL for failure)
 *************************************************************************/

void * imm_malloc(size_t size, u32 flags, u32 immid)
{
	struct imm_virt_t *head, *p, *new;
	struct imm_info_t *imm_info;
	int res;
	u32 address=0, end;

	down(&imm_sem);
	imm_debug("imm_malloc: size=%d, flags=%02X, immid=%u\n",
			size, flags, immid);

	/* if this client isn't registered, fail */
	if ((imm_info = imm_info_getentry(immid)) == NULL) {
		up(&imm_sem);
		return NULL;
	}

	/* if this module isn't initialized (very rare), fail */
	if (!module_initialized) {
		register_error(immid, IMM_ERROR_UNINIT);
		up(&imm_sem);
		return(NULL);
	}

	/* if the size is invalid, fail */
	if (size < 1) {
		register_error(immid, IMM_ERROR_SIZETOOLOW);
		up(&imm_sem);
		return(NULL);
	}

	if ((!PAGE_IS_DMA(flags) && ((size+imm_info->used_space)
		> phys_sram_size)) || (PAGE_IS_DMA(flags)
		&&(size+imm_info->used_space > phys_sram_size))) {

		register_error(immid, IMM_ERROR_NOSPACE);
		up(&imm_sem);
		return(NULL);
	}

	p = head = imm_info->malloc_list;

	/* request is valid, let's do it */
	while (!address) {
		p = p->next;
                if (p->start - p->prev->end >= size) {
                        address = p->prev->end;
			/* if this is in the middle of a page, make sure */
			/* the existing page is compatible */
			if ((address%PAGE_SIZE)&&
				!page_compatible(address, flags, immid)) {
				/* if not, align the allocation to next page */
				address = PAGE_ALIGN(address);
				/* we may now not have enough space */
				if (p->start - address < size) {
					address = 0;
				}
			}
			if (address) {
				/* if we're still in business, check the end
				 * address
				 */
				end = address + size;
				if ((end%PAGE_SIZE) && ((end&PAGE_MASK)
					== (p->start&PAGE_MASK)) &&
					!page_compatible(end, flags, immid)) {
					/* in the unlikely event that an
					 * allocation is perfectly sandwiched
					 * between two others, and the end
					 * page is incompatible, move on
					 */
					address = 0;
				}
			}
                }
		if (p == head) break;
        }

        if (!address) {
		register_error(immid, IMM_ERROR_NOSPACE);
		up(&imm_sem);
		return NULL;
	}

	if ((res = imm_alloc_pages(address, size,
		flags, imm_info)) != 0) {

		register_error(immid, res);
		up(&imm_sem);
		return NULL;
	}

        /* p now points to the memory block just after the new block's dest */
        new = kmalloc(sizeof(struct imm_virt_t), GFP_ATOMIC);
        if (new == NULL) {
		register_error(immid, IMM_ERROR_KMALLOC);
		up(&imm_sem);
		return NULL;
	}

        new->start = address;
        new->end = address + size;
        new->immid = immid;

        new->next = p;
        new->prev = p->prev;
        new->next->prev = new;
        new->prev->next = new;

	imm_info->used_space += size;
#ifdef CONFIG_IMM_DEBUG
	print_page_list("malloc");
#endif
	register_error(immid, IMM_ERROR_NONE);
	up(&imm_sem);
	return (void *)new->start;
}

/*************************************************************************
 *
 * Function: imm_free
 * Description: Frees a block of imm memory
 *
 * Arguments:
 * ptr - the address of the block to free
 * flags - release flags, can be 0 or IMM_NO_REMAP
 *
 * Return Value:
 * the error condition, can be sent to imm_error to retrieve the
 * error string
 *************************************************************************/

int imm_free(void *address, u32 immid)
{
	int res;
	struct imm_info_t *imm_info;

	down(&imm_sem);
	imm_debug("imm_free: address=%08X, immid=%d, region=%s\n",
		(u32)address, immid, (IMMID_USER(immid))?"USER":"KERNEL");

	/* if this client isn't registered, forget it */
	if ((imm_info = imm_info_getentry(immid)) == NULL) {
		up(&imm_sem);
		return IMM_ERROR_UNKNOWN;
	}

	/* if this module isn't initialized (very rare), forget it */
	if (!module_initialized) {
		register_error(immid, IMM_ERROR_UNINIT);
		up(&imm_sem);
		return IMM_ERROR_UNINIT;
	}

	if (address == NULL) {
		register_error(immid, IMM_ERROR_ENTRY_NOTFOUND);
		up(&imm_sem);
		return IMM_ERROR_ENTRY_NOTFOUND;
	}

	res = release_entry(address, imm_info);
#ifdef CONFIG_IMM_DEBUG
	print_page_list("free");
#endif
	register_error(immid, res);
	up(&imm_sem);
	return res;
}

/*************************************************************************
 *
 * Function: imm_free_immid
 * Description: Frees all imm memory assigned to a given process, for
 * applications the immid is the process id, for drivers the immid is the value
 * retrieved from imm_immid().
 *
 * Arguments:
 * flags - release flags, can be 0 or IMM_NO_REMAP
 * immid - the immid for whom all sram allocations should be freed
 *
 * Return Value:
 * the error condition, can be sent to imm_error to retrieve the
 * error string
 *************************************************************************/

int imm_free_immid(u32 immid)
{
	int res;
	struct imm_info_t *imm_info;

	/* if this client isn't registered, forget it */
	if ((!module_initialized) || ((imm_info = imm_info_getentry(immid))
		== NULL)) {

		return 0;
	}
	res = release_immid(imm_info);
	res = imm_free_pages_immid(imm_info);
	imm_info->used_space = 0;
#ifdef CONFIG_IMM_DEBUG
	print_page_list("task exit");
#endif
	return res;
}

/*************************************************************************
 *
 * Function: imm_get_freespace
 * Description: Figure out how much SRAM space is left
 *
 * Arguments:
 * flags - flags for the type of memory to check the free space for
 * immid - the immid for the client who's asking
 *
 * Return Value:
 * (unsigned int) the amount of SRAM remaining in bytes
 *************************************************************************/

unsigned int imm_get_freespace(u32 flags, u32 immid) {
	struct imm_page_t *p;
	unsigned int bytes = phys_sram_size;
	struct imm_info_t *imm_info;

	down(&imm_sem);

	/* if this client isn't registered, forget it */
	if ((!module_initialized) || ((imm_info = imm_info_getentry(immid))
		== NULL)) {

		up(&imm_sem);
		return 0;
	}

	for (p = &page_list[0]; p < &page_list[imm_sram_pages]; p++) {
		if (PAGE_IS_MAPPED(p->flag)&&
			(immid != p->immid))
		{
			bytes -= PAGE_SIZE;
		}
	}
	bytes -= imm_info->used_space;
	if (unlikely(bytes < 0)) {
		imm_failure("used_space failure");
	}
	up(&imm_sem);
	return min(bytes, phys_sram_size-imm_info->used_space);
}

/*************************************************************************
 *
 * Functions: imm_sram_pages_read_proc, imm_sram_vmap_read_proc
 * Description: Proc fs functions which display the mapped pages
 * and virtual memory allocation lists
 *
 *************************************************************************/

READ_PROC_PROTO(imm_sram_usagemap_read_proc)
{
	READ_PROC_VARS
	int i, j;
	int imm_sram_first_bank;

	imm_sram_first_bank = (imm_sram_start - phys_sram_start)/PHYS_SRAM_BANK_SIZE;

	READ_PROC_INIT
	for (i = 0; i < imm_sram_pages; i+=PHYS_SRAM_PAGES_PER_BANK) {
		pprintf(" ");
                for (j = 0; j < PHYS_SRAM_PAGES_PER_BANK; j++)
                        pprintf("%c", (i - imm_sram_first_bank)?'-':'_');
                pprintf(" \n|");
                for (j = 0; j < PHYS_SRAM_PAGES_PER_BANK; j++) {
                        pprintf("%c", PAGE_IS_MAPPED(page_list[i+j].flag)?
				(IMMID_USER(page_list[i+j].immid)?'U':'K'):'.');
                }
                pprintf("| SRAM Bank %d\n", i/PHYS_SRAM_PAGES_PER_BANK + imm_sram_first_bank);
        }
        pprintf(" ");
        for (j = 0; j < PHYS_SRAM_PAGES_PER_BANK; j++)
                pprintf("-");
        pprintf("\n");
	READ_PROC_RETURN
}

READ_PROC_PROTO(imm_sram_pages_read_proc) {
	READ_PROC_VARS
	struct imm_page_t *p;
	int i;

	READ_PROC_INIT
	for (i = 0, p = &page_list[0]; p < &page_list[imm_sram_pages]; p++, i++) {
		if (PAGE_IS_MAPPED(p->flag)) {
                        pprintf("I%03d V:%08X P:%08X L:%d ID:%u \
                                FLAGS:%c%c%c\n", i, p->virt_addr, p->phys_addr,
                                0,
                                IMMID_VALUE(p->immid), IMMID_TYPEC(p->immid),
                                'S',
                                                PAGE_IS_DMA(p->flag)?'D':'.');
                }
        }

	READ_PROC_RETURN
}

/*************************************************************************
 * imm_init - initialize Intel Memory Management
 *************************************************************************/

void imm_dau_sram_init (struct proc_dir_entry *imm_dir)
{
	struct proc_dir_entry *entry;
	struct imm_page_t *p;
	int i;
	struct vm_struct *kernel_area;
	struct clk *isc_clk;

	imm_sram_pages = imm_sram_size / PAGE_SIZE;
	num_free_sram_pages = imm_sram_pages;
	imm_malloc_map_size = phys_sram_size << 1;

	page_list = kzalloc(sizeof(struct imm_page_t) * imm_sram_pages, GFP_KERNEL);
#ifdef CONFIG_IMM_DEBUG
	plist = kzalloc(sizeof(char) * imm_sram_pages, GFP_KERNEL);
#endif

	/* grab a vm_area struct (from mm/vmalloc.c) */
	kernel_area = get_vm_area(imm_malloc_map_size, VM_IOREMAP);
	if ((!kernel_area)||((u32)kernel_area->addr == 0)) {
		imm_failure("get_vm_area failed\n");
		return;
	}

	malloc_list.start = (u32)kernel_area->addr + imm_malloc_map_size;
	malloc_list.end = (u32)kernel_area->addr;

	for (i = 0, p = &page_list[0]; p < &page_list[imm_sram_pages]; p++, i++) {
#ifdef CONFIG_IMM_DEBUG
		p->index = i;
		plist[i] = '.';
#endif
		p->phys_addr = imm_sram_start + i * PAGE_SIZE;
		p->flag = IMM_INFO_SRAM;
		p->immid = 0;
		p->virt_addr = 0;
	}
	if (imm_dir) {
		REG_READ_PROC(entry, imm_dir, "sram_usagemap",
				imm_sram_usagemap_read_proc)
		REG_READ_PROC(entry, imm_dir, "sram_pages",
				imm_sram_pages_read_proc)
	}

	/* Enable clock */
	isc_clk = clk_get(NULL, "ISCCLK");
	clk_enable(isc_clk);

	pr_info("Intel(c) Memory Management - SRAM Allocation\n");
	module_initialized = 1;
}

EXPORT_SYMBOL(imm_malloc);
EXPORT_SYMBOL(imm_free);
EXPORT_SYMBOL(imm_get_freespace);
EXPORT_SYMBOL(imm_free_immid);



