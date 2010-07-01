/*
 *  linux/arch/arm/mach-pxa/imm.c
 *
 *  Intel Memory Management
 *
 *  User level driver interface to IMM APIs
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
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/random.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/raw.h>
#include <linux/tty.h>
#include <linux/capability.h>
#include <linux/shm.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/pgalloc.h>
#include <mach/imm.h>

struct priv_struct {
	u32 virt_sram_base;
	int virt_sram_base_valid;
	u32 pid;
};

static int num_instances = 0;

static int imm_ioctl(struct inode *inode, struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct priv_struct *pv = filp->private_data;
	struct imm_memory_block mb, *pmb = (struct imm_memory_block *)arg;
	int res = 0;
	char *s, *u;
	u32 size;

	/* we must have a struct included */
	if(arg == 0) return 0;
	memcpy(&mb, pmb, sizeof(struct imm_memory_block));

	switch(cmd) {
#ifdef CONFIG_IMM_API_SRAM
        case IMM_MALLOC:
		if(!pv->virt_sram_base_valid) {
			imm_debug("Cannot malloc until mmap is called");
			res = -1;
			break;
		}
                res = (int)imm_malloc(mb.size, mb.flag, pv->pid);
		imm_debug("IMM_MALLOC RETURN STATUS = %s\n", imm_error(pv->pid));
		break;
        case IMM_GET_FREE_SPACE:
		if(!pv->virt_sram_base_valid) {
			imm_debug("Cannot check free space until mmap is called");
			res = -1;
			break;
		}
		size = imm_get_freespace(mb.flag, pv->pid);
		copy_to_user(&pmb->size, &size, sizeof(unsigned int));
		imm_debug("IMM_GET_FREE_SPACE RETURN STATUS = %s\n", imm_error(pv->pid));
		break;
        case IMM_FREE:
		if(!pv->virt_sram_base_valid) {
			imm_debug("Cannot free memory until mmap is called");
			res = -1;
			break;
		}
                res = imm_free((void *)mb.start, pv->pid);
		imm_debug("IMM_FREE RETURN STATUS = %s\n", imm_error(pv->pid));
                break;
#else
        case IMM_MALLOC:
		imm_debug("Unsupported IMM command");
                res = 0;
		break;
#endif
	case IMM_GET_PHYS_SRAM_SIZE:
		size = phys_sram_size;
		copy_to_user(&pmb->size, &size, sizeof(unsigned int));
		break;
	case IMM_GET_VIRT_SRAM_SIZE:
		size = imm_malloc_map_size;
		copy_to_user(&pmb->size, &size, sizeof(unsigned int));
		break;
	case IMM_ERROR:
		s = (char *)imm_error(pv->pid);
		u = (char *)mb.start;
		copy_to_user(u, s, ((strlen(s)+1) > mb.size)?mb.size:(strlen(s)+1));
		break;
        case IMM_GET_VIRTUAL:
                res = (int)imm_get_virtual((u32)mb.start, current);
                break;
        case IMM_GET_PHYSICAL:
                res = (int)imm_get_physical((void *)mb.start, pv->pid);
                break;
	default:
		imm_debug("Unsupported IMM command");
		return -1;
	}
        return res;
}

static int imm_mmap(struct file *filp, struct vm_area_struct * vma)
{
	struct priv_struct *pv = filp->private_data;

	imm_debug("vma->vm_pgoff = %08X\n", vma->vm_pgoff << PAGE_SHIFT);
	if((vma->vm_end-vma->vm_start) < imm_malloc_map_size) {
		printk("IMM Driver mmap failure, size must be at least %d bytes\n",
			imm_malloc_map_size);
		return -EINVAL;
	}

	vma->vm_page_prot = (pgprot_t)PAGE_SHARED;

	/* Don't try to swap out physical pages */
	vma->vm_flags |= VM_RESERVED | VM_LOCKED;

	zap_page_range(vma, vma->vm_start, vma->vm_end-vma->vm_start, NULL);
	imm_debug("Virtual sram mmap from v(0x%08X) of %d bytes\n", (u32)vma->vm_start, vma->vm_end-vma->vm_start);
	if(!pv->virt_sram_base_valid)
		imm_register_user((u32)vma->vm_start, current->pid);
	pv->virt_sram_base = (u32)vma->vm_start;
	pv->virt_sram_base_valid = 1;
	return 0;
}

static int imm_driver_open(struct inode * inode, struct file * filp)
{
	struct priv_struct *pv;

	pv = kmalloc (sizeof (struct priv_struct), GFP_KERNEL);
	if (!pv) return -ENOMEM;

	num_instances++;
	pv->virt_sram_base = 0;
	pv->virt_sram_base_valid = 0;
	pv->pid = current->pid;

	imm_debug("imm_driver_open(current->pid = %d)\n", current->pid);

	filp->private_data = pv;

	return 0;
}

static int imm_driver_release(struct inode * inode, struct file * filp)
{
	struct priv_struct *pv = filp->private_data;

	num_instances--;
	if(num_instances < 0) num_instances = 0;
	if(pv) kfree(pv);
	filp->private_data = NULL;
        return 0;
}

static struct file_operations imm_fops = {
	mmap:		imm_mmap,
	open:		imm_driver_open,
	release:	imm_driver_release,
	ioctl:		imm_ioctl,
};

static struct miscdevice imm_misc = {
	minor : IMM_MINOR,
	name  : "imm",
	fops  : &imm_fops,
};

static int __init imm_init(void) {
	if (misc_register(&imm_misc)) {
		printk(KERN_ERR "Error registering device /dev/%s\n", imm_misc.name);
		return -EFAULT;
	}
	return 0;
}

static void __exit imm_exit(void) {
	misc_deregister(&imm_misc);
}

module_init(imm_init)
module_exit(imm_exit)
