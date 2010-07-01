/*
 *  linux/arch/arm/mach-pxa/imm/policy.c
 *
 *  Intel Memory Management
 *
 *  Policy Manager - focal point for IMM functionality
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

#include <linux/mm.h>
#include <linux/rmap.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <mach/hardware.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/profile.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include "imm.h"

int phys_sram_start;
int phys_sram_size;
int imm_sram_start;
int imm_sram_size;

extern int imm_malloc_map_size;
/* the imm semaphore */
DECLARE_MUTEX(imm_sem);

/* error list */
const char *imm_errors[] = {
        "Operation completed successfully",
        "System memory is too low, kernel malloc failed",
        "Insufficient space left",
        "The size argument is too small, size must be > 0 bytes",
        "The requested entry does not exist",
        "The address range requested is off limits to this process",
        "This functionality is currently unsupported",
	"The requested address range overlaps with an existing entry's \
		address range",
	"The requested IMM module failed to initialize due to a kernel error",
        "Unknown Error!"
};

/* immid list */
struct imm_info_t immid_list;
struct proc_dir_entry *proc_imm = NULL;
static u32 kernel_immid = IMM_FIRST_KERNEL_IMMID;
static int imm_initialized = 0;

/*----------------------------------------------------------------------
 *
 *  High Level Call Tree:
 *
 *  [Application or Driver Call]
 *	imm_register_kernel
 *		imm_info_newentry
 *	imm_register_user
 *		imm_info_newentry
 *	imm_get_physical
 *	imm_get_virtual
 *	imm_error
 *
 *  [IMM Module call (SRAM, Power)]
 *	imm_info_getentry
 *	register_error
 *	pprintf_helper
 *
 *  [Process Exit - Kernel Hook from System Profile Interface]
 * 	imm_task_exit
 *		imm_free_immid (sram.c)
 *		imm_info_delentry
 *
 *  [Module Initialization]
 *	imm_init
 *		imm_dau_sram_init (sram.c)
 *		imm_autopower_init (power.c)
 *
 *--------------------------------------------------------------------*/

/*************************************************************************
 *
 * Function: imm_get_physical
 * Description: Retrieve the physical address for a given imm virtual
 * address
 *
 * Arguments:
 * virtual - address returned from imm_malloc
 * immid - the id of the caller
 *
 * Return Value:
 * the physical address, 0 on failure
 *************************************************************************/

u32
imm_get_physical(void *v, u32 immid)
{
	pmd_t *pmd;
	pte_t *pte;
	pgd_t *pgd;
	u32 val = 0, virtual = (u32)v;
	struct mm_struct* mm;

	if (IMMID_USER(immid))
		mm = current->mm;
	else
		mm = &init_mm;

	pgd = pgd_offset(mm, virtual);
	if (!pgd_none(*pgd) && !pgd_bad(*pgd)) {
		/* 1st level entry pointer */
		pmd = pmd_offset(pgd, virtual);
		if (!pmd_none(*pmd) && !pmd_bad(*pmd)) {
			/* 2nd level entry pointer */
			pte = pte_offset_kernel(pmd, virtual);
			if (pte) {
				val = (*(u32 *)((u32)pte-2048))&PAGE_MASK;
				val += virtual%PAGE_SIZE;
			}
		} else if (!pmd_none(*pmd)) {
			/* work around */
			val = (unsigned long)virt_to_phys((void*)virtual);
			/*
			 * old method address conversion is not correct
			val = (*(u32 *)pmd) & 0xFFF00000;
			val += virtual%0x100000;
			*/
		}
	}

	return val;
}

/*************************************************************************
 *
 * Function: imm_get_virtual
 * Description: Retrieve the virtual address for a given physical
 * address
 *
 * Arguments:
 * physical - address of some memory in the immid's address space
 * immid - the id of the caller
 *
 * Return Value:
 * the virtual address, 0 on failure
 *************************************************************************/

u32
imm_get_virtual(u32 p, struct task_struct *tsk)
{
	pmd_t *pmd;
	pte_t *pte;
	pgd_t *pgd;
	u32 val, v;
	struct mm_struct* mm;
	struct vm_area_struct *vm_search;

	down(&imm_sem);
	if ((tsk == NULL)||(tsk->mm == NULL)) return 0;
	mm = tsk->mm;

	for (vm_search = mm->mmap; vm_search; vm_search = vm_search->vm_next) {
		for (v = vm_search->vm_start; v < vm_search->vm_end;
				v+=PAGE_SIZE) {
			pgd = pgd_offset(mm, v);
			if (!pgd_none(*pgd) && !pgd_bad(*pgd)) {
				pmd = pmd_offset(pgd, v);
				if (!pmd_none(*pmd) && !pmd_bad(*pmd)) {
					pte = pte_offset_kernel(pmd, v);
					if (pte) {
						val = (*(u32 *)((u32)pte-2048))&PAGE_MASK;
						val += p%PAGE_SIZE;
						if (p == val) {
							up(&imm_sem);
							return(v+(p%PAGE_SIZE));
						}
					}
				}
			}
		}
	}
	up(&imm_sem);
	return 0;
}

/*************************************************************************
 * imm_info list handlers
 ************************************************************************/

/* retrieve an imm_info object from the list */
struct imm_info_t *
imm_info_getentry(u32 immid)
{
	struct imm_info_t *head, *p;

	if (immid < 1)
		return NULL;
	head = &immid_list;
	p = head->next;

	/* loop through the list til you hit the end or the immid */
	while ((p != head) && (p->immid != immid)) p = p->next;
	if (p->immid == immid) {
		return p;
	} else {
		return NULL;
	}
}

/* create a new imm_info entry for a new immid */
static struct imm_info_t *
imm_info_newentry(u32 immid, char *name, u32 user_imm_space)
{
	struct imm_info_t *head, *p, *new;

	head = &immid_list;
	p = head->next;

	/* loop through the list til you hit the end or the immid */
	while ((p != head) && (p->immid != immid)) p = p->next;
	if (p->immid == immid) {
		return p;
	} else {
		imm_debug("imm_info_newentry called, initializing a new \
				imm_info entry\n");
	        new = kmalloc(sizeof(struct imm_info_t), GFP_ATOMIC);
	        if (new == NULL) {
			imm_debug("imm_info_newentry failed, kmalloc \
					returned NULL: %s, line %d\n",
					__FILE__, __LINE__);
			return NULL;
		}

		new->immid = immid;
		new->last_error = 0;
		memcpy(new->name, name, min(strlen(name)+1,
			(size_t)(IMM_CLIENT_NAME_SIZE-1)));
		new->name[IMM_CLIENT_NAME_SIZE-1] = '\0';
		if (IMMID_USER(immid)) {
			new->mm = current->mm;
		} else {
			new->mm = &init_mm;
		}

#ifdef CONFIG_IMM_API_SRAM
		if (IMMID_USER(immid)) {
			new->malloc_list = kmalloc(sizeof(struct imm_virt_t),
						GFP_ATOMIC);
			if (new->malloc_list == NULL) {
				imm_debug("imm_info_newentry failed, kmalloc \
						returned NULL: %s, line %d\n",
						__FILE__, __LINE__);
				kfree(new);
				return NULL;
			}
			new->malloc_list->immid = immid;
			new->malloc_list->start = user_imm_space
				+ imm_malloc_map_size;
			new->malloc_list->end = user_imm_space;
			new->malloc_list->prev = new->malloc_list;
			new->malloc_list->next = new->malloc_list;
		} else {
			new->malloc_list = &malloc_list;
		}
		new->used_space = 0;
#endif

	        new->next = p;
	        new->prev = p->prev;
	        new->next->prev = new;
	        new->prev->next = new;

		return new;
	}
}

#if defined(CONFIG_IMM_API_SRAM)
/* remove an immid from the list */
static int
imm_info_delentry(u32 immid)
{
	struct imm_info_t *head, *p;

	head = &immid_list;
	p = head->next;

	/* loop through the list til you hit the end or the immid */
	while ((p != head) && (p->immid != immid)) p = p->next;
	if (p->immid == immid) {
		if (IMMID_USER(immid)) {
			struct imm_virt_t *headv = p->malloc_list;
			struct imm_virt_t *pv = headv->next;
			while (pv != headv) {
				/* pv now points to the memory block
				 * to be removed
				 */
				pv->prev->next = pv->next;
				pv->next->prev = pv->prev;
				kfree(pv);
				pv = headv->next;
			}
			kfree(p->malloc_list);
		}
		p->prev->next = p->next;
		p->next->prev = p->prev;
		kfree(p);
		return 0;
	}
	return -1;
}

static int
imm_task_exit(struct notifier_block * self, unsigned long val, void * data) {
struct task_struct * tsk = (struct task_struct *)data;

	down(&imm_sem);
        if (tsk && imm_initialized) {
	/* cache must be cleared first since some of it may be */
	/* locked over SRAM */
		imm_free_immid(tsk->pid);
		imm_info_delentry(tsk->pid);
        }
	up(&imm_sem);

	return NOTIFY_OK;
}

static struct notifier_block imm_task_exit_nb = {
	.notifier_call  = imm_task_exit,
};
#endif

/* associate an error with an immid */
void
register_error(u32 immid, u32 err)
{
	struct imm_info_t *head, *p;

	head = &immid_list;
	p = head->next;

	/* loop through the list til you hit the end or the immid */
	while ((p != head) && (p->immid != immid)) p = p->next;
	if (p->immid == immid)
		p->last_error = err;
}

/*************************************************************************
 *
 * Function: imm_register(kernel and user)
 * Description: Create a new immid instance for either the kernel or user
 *   space.
 *
 * Return Value:
 * immid - a new unique id which will never conflict with user immids
 *
 *************************************************************************/

u32
imm_register_kernel(char *name)
{
	struct imm_info_t *imm_info;

	down(&imm_sem);
	if (name == NULL) name = IMM_DEFAULT_DRIVER_NAME;
	imm_debug("imm_register_kernel called for %s, new ID = %08X\n",
		name, kernel_immid);
	if ((imm_info = imm_info_newentry(kernel_immid, name, 0)) == NULL) {
		up(&imm_sem);
		return 0;
	}
	up(&imm_sem);
	return(kernel_immid++);
}

u32
imm_register_user(u32 vstart, u32 pid)
{
	struct imm_info_t *imm_info;

	down(&imm_sem);
	imm_debug("imm_register_user called for %s, vstart = %08X, pid = %d\n",
		current->comm, vstart, pid);
	if (!vstart || !pid ||
		((imm_info = imm_info_newentry(pid, current->comm, vstart))
		 == NULL)) {
		up(&imm_sem);
		return 0;
	}
	up(&imm_sem);
	return(pid);
}

/*************************************************************************
 *
 * Function: imm_error
 * Description: Return the last error for the given immid
 *
 * Return Value:
 * a const char pointer of the error string
 *
 *************************************************************************/

const char *
imm_error(u32 immid)
{
	struct imm_info_t *head, *p;

	down(&imm_sem);
	head = &immid_list;
	p = head->next;

	/* loop through the list til you hit the end or the immid */
	while ((p != head) && (p->immid != immid)) p = p->next;
	if (p->immid == immid) {
		if ((p->last_error >= 0)
			&& (p->last_error < IMM_ERROR_UNKNOWN)) {
			up(&imm_sem);
			return imm_errors[p->last_error];
		}
	}
	up(&imm_sem);
	return "The caller is not a client of IMM";
}

int
pprintf_helper(struct imm_proc_t *proc_head, char *page, char **start,
		int count, int *eof, int cmd, const char *fmt, ...)
{
	char buffer[1000];
	struct imm_proc_t *new;
	va_list args;
	int len, temp;

	switch(cmd) {
	case PCMD_CLEAN: /* init mode */
		for (new = proc_head->next; new != proc_head;
			new = proc_head->next) {
			new->prev->next = new->next;
			new->next->prev = new->prev;
			kfree(new);
		}
		break;
	case PCMD_STORE: /* input mode */
		va_start(args, fmt);
		len = vsprintf(buffer, fmt, args);
	        va_end(args);

		if ((new = kmalloc(sizeof(struct imm_proc_t), GFP_ATOMIC))
				!= NULL) {
			temp = strlen(buffer)+1;
			new->buffer = kmalloc(temp, GFP_ATOMIC);
			memcpy(new->buffer, buffer, temp);
			new->len = len;
			new->next = proc_head;
			new->prev = proc_head->prev;
			new->next->prev = new;
			new->prev->next = new;
		}
		break;
	case PCMD_EXPORT: /* output mode */
		len = 0;
		for (new = proc_head->next; new != proc_head;
			new = proc_head->next) {
			/* check if we're about to exceed count */
			if ((len + new->len) > count) {
				/* if we haven't reached count, write part
				 * of the current entry
				 */
				if (len < count) {
					/* copy out part of the entry */
					memcpy(page+len, new->buffer,
						count - len);
					/* remove the outputted data */
					memmove(new->buffer,
						new->buffer + (count - len),
						len + new->len - count + 1);
					new->len -= count - len;
				}
				/* we're not done yet, more printing is
				 * necessary
				 */
				*eof = 0;
				*start = page;
				return count;
			}

			memcpy(page+len, new->buffer, new->len);
			len += new->len;
			new->prev->next = new->next;
			new->next->prev = new->prev;
			kfree(new);
		}
		/* done printing */
		*eof = 1;
		*start = page;
		return len;
	}
	return 0;
}

static int __init __imm_init (void)
{
#if defined(CONFIG_IMM_API_SRAM)
int exit_cb_err = 1;
#endif

	kernel_immid = IMM_FIRST_KERNEL_IMMID;

#if defined(CONFIG_IMM_API_SRAM)
	/* the exit callback is needed for SRAM and Cache garbage collection */
	exit_cb_err = profile_event_register(PROFILE_TASK_EXIT,
						&imm_task_exit_nb);
	if (exit_cb_err)
		printk(KERN_ERR "Process exit callback failed to register, \
				all APIs have been disabled\n");
#endif

	immid_list.immid = 0;
	immid_list.last_error = 0;
	immid_list.mm = NULL;
        immid_list.prev = &immid_list;
        immid_list.next = &immid_list;
#ifdef CONFIG_IMM_API_SRAM
	immid_list.malloc_list = NULL;
	immid_list.used_space = 0;
#endif

	if ((proc_imm = proc_mkdir("imm", NULL)) == NULL) {
		printk(KERN_ERR "Failed to initialize the IMM proc \
				interface\n");
	}

#ifdef CONFIG_IMM_API_SRAM
	/* if the exit cb is available */
	if (!exit_cb_err)
		imm_dau_sram_init(proc_imm);
#endif
	pr_info("Intel(c) Memory Management is now Enabled\n");

	imm_initialized = 1;

	return 0;
}

static int __init imm_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		return ret;
	}
	phys_sram_start = res->start;
	phys_sram_size = res->end - res->start + 1;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		return ret;
	}
	imm_sram_start = res->start;
	imm_sram_size = res->end - res->start + 1;

	__imm_init();

	return 0;
}

static int imm_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver imm_driver = {
	.probe		= imm_probe,
	.remove		= imm_remove,
	.driver		= {
		.name	= "pxa3xx-imm",
	},
};

static int __init imm_init(void)
{
	return platform_driver_register(&imm_driver);
}

static void __exit imm_exit(void)
{
	platform_driver_unregister(&imm_driver);
}

module_init(imm_init);
module_exit(imm_exit);
MODULE_LICENSE("GPL");

#if defined(CONFIG_IMM_API_SRAM)
EXPORT_SYMBOL(imm_task_exit);
#endif

/* app/driver calls */
EXPORT_SYMBOL(imm_register_kernel);
EXPORT_SYMBOL(imm_register_user);
EXPORT_SYMBOL(imm_error);
EXPORT_SYMBOL(imm_get_physical);
EXPORT_SYMBOL(imm_get_virtual);

