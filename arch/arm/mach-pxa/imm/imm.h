/*
 *  linux/arch/arm/mach-pxa/imm/imm.h
 *
 *  Intel Memory Management
 *
 *  Internal IMM Defines/Globals/Functions
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

#ifndef _ARCH_PXA_IMM_H
#define _ARCH_PXA_IMM_H

#include <linux/list.h>
#include <mach/pxa-regs.h>
#include <mach/imm.h>

/* -------- IMM global data --------------- */

extern struct semaphore imm_sem;

/* error list */
enum error_cond {
        IMM_ERROR_NONE=0,
        IMM_ERROR_KMALLOC,
        IMM_ERROR_NOSPACE,
        IMM_ERROR_SIZETOOLOW,
        IMM_ERROR_ENTRY_NOTFOUND,
        IMM_ERROR_PERMISSION_DENIED,
        IMM_ERROR_UNSUPPORTED,
	IMM_ERROR_RANGEOVERLAP,
	IMM_ERROR_UNINIT,
        IMM_ERROR_UNKNOWN
};

/* defines and macros */
#define VMALLOC_VMADDR(x) 		((unsigned long)(x))
#define IMM_FIRST_KERNEL_IMMID 		0xc0000000
#define IMM_DEFAULT_DRIVER_NAME		"unnamed"
#define IMMID_USER(x)   		((u32)(x) < IMM_FIRST_KERNEL_IMMID)
/* #define IMMID_OVERLAP(id1, id2) 	\
 * 	((!IMMID_USER(id1)&&!IMMID_USER(id2))?1:(id1 == id2)) */
#define IMMID_OVERLAP(id1, id2) 	((id1) == (id2))
#define IMMID_VALUE(x)			((IMMID_USER(x))?(x): \
					((x)-IMM_FIRST_KERNEL_IMMID))
#define IMMID_TYPE(x)			((IMMID_USER(x))?"USER":"KERNEL")
#define IMMID_TYPEC(x)			((IMMID_USER(x))?'U':'K')

/* -------- IMM global functions --------------- */

extern void register_error(u32 immid, u32 err);
extern void imm_dau_sram_init(struct proc_dir_entry *);
extern int imm_free_immid(u32 immid);
/* __ioremap subfunction from ioremap.c */
extern int remap_area_pages(unsigned long, unsigned long,
				unsigned long, unsigned long);
extern void unmap_area_pmd(pgd_t *dir, unsigned long address,
				unsigned long size);

/* -------- IMM proc interface functions --------------- */

/* entry data to be read out from read_procs */
struct imm_proc_t {
	char *buffer;
	u32 len;
	struct imm_proc_t *prev;
	struct imm_proc_t *next;
};

/* pprintf_helper commands (passed through cmd var) */
#define PCMD_EXPORT			0
#define PCMD_STORE			1
#define PCMD_CLEAN			2

/* pprintf helper function, handles multiple calls to this proc to
 * export data greater than a page
 */
extern int pprintf_helper(struct imm_proc_t *proc_head, char *page,
			char **start, int count, int *eof, int cmd,
			const char *fmt, ...);

/* prototype for a read proc function, needed to get ubquity on variable
 * names
 */
#define READ_PROC_PROTO(name)	static int name(char* page, char** start, \
				off_t off, int count, int* eof, void* data)

/* internal variables that each read proc function must have (used only
 * by these helper macros)
 */
#define READ_PROC_VARS		static struct imm_proc_t proc_head = \
				{NULL, 0, &proc_head, &proc_head};

/* init call which is used to print additional pages in multipage outputs,
 * off = 0 means that this is a new call
 */
#define READ_PROC_INIT		if (off == 0) {				\
					pprintf_helper(&proc_head, page, \
							start, count, eof, \
							PCMD_CLEAN, NULL); \
				} else {				\
					return pprintf_helper(&proc_head, \
							page, start, count, \
							eof, PCMD_EXPORT, NULL);\
			       }

/* return call which stars exporting the data queued */
#define READ_PROC_RETURN		return pprintf_helper(&proc_head, page, start, count, eof, PCMD_EXPORT, NULL);
#define REG_READ_PROC(e, d, n, f)	if((e = create_proc_entry(n, S_IRUSR | S_IRGRP | S_IROTH, d)) != NULL) e->read_proc = f;
#define pprintf(s, args...)		pprintf_helper(&proc_head, page, start, count, eof, PCMD_STORE, s, ## args);

#define WRITE_PROC_PROTO(name)		static int name(struct file* file, const char* buffer, unsigned long count, void* data)
#define WRITE_PROC_VAR_NUM(s)		char val[s];
#define WRITE_PROC_GET_NUM(n, s)	copy_from_user(val, buffer, s); n = simple_strtol(val, NULL, 0);
#define WRITE_PROC_RETURN		return(count);
#define REG_RW_PROC(e, d, n, rf, wf)	if((e = create_proc_entry(n, S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH, d)) != NULL) \
{ e->read_proc = rf; e->write_proc = wf; }

/* -------- IMM client data --------------- */

#ifdef CONFIG_IMM_API_SRAM
extern struct imm_virt_t malloc_list;

/* entries for virtual memory sram malloc list */
struct imm_virt_t {
	u32 immid;
	u32 start; /* pointer to physical start of high speed memory block */
	u32 end;   /* pointer to first physical address after the high speed memory block */
	struct imm_virt_t *prev;
	struct imm_virt_t *next;
};
#endif

/* entries for the global immid list */
struct imm_info_t {
	/* general info for the imm instance */
	u32 immid;		/* imm ID of the caller */
	int last_error;		/* last registered error */
	char name[IMM_CLIENT_NAME_SIZE]; /* name of the client process/driver */
	struct mm_struct *mm;	/* memory struct used for remapping */

#ifdef CONFIG_IMM_API_SRAM
	struct imm_virt_t *malloc_list; /* pointer to virtual allocation list for this space */
	u32 used_space;
#endif
	/* list pointers */
	struct imm_info_t *prev;
	struct imm_info_t *next;
};

struct imm_info_t *imm_info_getentry(u32);
#endif

