/*
 * linux/drivers/mtd/pxa3xx_bbm.c
 *
 * Bad Block Table support for PXA3xx.
 * Copyright (C) 2007 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/onenand.h>
#include <linux/mtd/compatmac.h>
#include <linux/mtd/pxa3xx_bbm.h>

#include <mach/hardware.h>

/* MOBM V2 is used on MhnP B0/B1/B2, MhnPL B1 and MhnL A0
 * MOBM V3 is used on MhnLV A0
 */
enum {
	MHN_OBM_NULL,
	MHN_OBM_V2,
	MHN_OBM_V3,
	MHN_OBM_INVAL,
};

#define NAND_RELOC_HEADER	0x524e
#define	PXA930_MAX_RELOC_ENTRY	20
#define	PXA930_NEW_MAX_RELOC_ENTRY	40
#define	PXA3xx_MAX_RELOC_ENTRY	127

#define	YFW_DBG

#ifdef	YFW_DBG
#define yfw_dbg(format, arg...) printk(KERN_ERR __FILE__ ": " format "\n" , ## arg)

void dump_reloc_table(struct pxa3xx_bbm *bbm)
{
	int i;

	for (i = 0; i < bbm->table->total; i++) {
		if(65535 == bbm->reloc[i].to)
			continue;
		printk("block: %d is relocated to block: %d\n",
				bbm->reloc[i].from, bbm->reloc[i].to);
	}
}
#else
#define	yfw_dbg(format, arg...) do {} while (0)
void dump_reloc_table(struct pxa3xx_bbm *bbm) {}
#endif

static int max_bbt_slots = 24;

void dump_buf(unsigned char *buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		printk(" %02x", buf[i]);
		if (0 == ((i + 1) % 16))
			printk("\n");
	}
	printk("\n");
}

/*
 * The relocation table management is different between MOBM V2 and V3.
 *
 * MOBM V2 is applied on chips taped out before MhnLV A0.
 * MOBM V3 is applied on chips taped out after MhnLV A0. It's also applied
 * on MhnLV A0.
 */
static int calc_obm_ver(void)
{
	unsigned int	cpuid;

	cpuid = read_cpuid(CPUID_ID);

	/* TavorP65e series */
	if (cpu_is_pxa935()) {
		return MHN_OBM_V3;
	}

	/* It's not xscale chip. */
	if ((cpuid & 0xFFFF0000) != 0x69050000)
		return MHN_OBM_INVAL;
	/* It's MhnP Ax */
	if ((cpuid & 0x0000FFF0) == 0x00006420)
		return MHN_OBM_V2;
	/* It's MhnP Bx */
	if ((cpuid & 0x0000FFF0) == 0x00006820) {
		if ((cpuid & 0x0F) <= 6)
			return MHN_OBM_V2;
		else
			return MHN_OBM_V3;
	}
	/* It's MhnL Ax */
	if ((cpuid & 0x0000FFF0) == 0x00006880) {
		if ((cpuid & 0x0F) == 0)
			return MHN_OBM_V2;
		else
			return MHN_OBM_V3;
	}
	/* It's MhnLV Ax */
	if ((cpuid & 0x0000FFF0) == 0x00006890)
		return MHN_OBM_V3;

	/* It's Tavor P A0 */
	if ((cpuid & 0x0000FFFF) == 0x00006834)
		return MHN_OBM_V2;

	/* It's Tavor P B0*/
	if ((cpuid & 0x0000FFFF) == 0x00006835)
		return MHN_OBM_V3;

	/* It's Tavor P B1*/
	if ((cpuid & 0x0000FFFF) == 0x00006837)
		return MHN_OBM_V3;

	/* It's Tavor P B2*/
	if ((cpuid & 0x0000FFFF) == 0x00006838)
		return MHN_OBM_V3;

	return MHN_OBM_INVAL;
}

/* add the relocation entry into the relocation table
 * It's valid on MOBM V3.
 * If the relocated block is bad, an new entry will be added into the
 * bottom of the relocation table.
 */
static int update_reloc_tb(struct mtd_info *mtd, struct pxa3xx_bbm *bbm, int block)
{
	struct reloc_table *table = bbm->table;
	struct reloc_item *item = bbm->reloc;
	struct erase_info instr;
	int obm, reloc_block, ret, entry_num = -1;
	int i;

	obm = calc_obm_ver();
	if (obm == MHN_OBM_V3) {
		if (bbm->table_init == 0) {
			printk(KERN_ERR "Error: the initial relocation \
					table can't be read\n");
			memset(table, 0, sizeof(struct reloc_table));
			table->header = NAND_RELOC_HEADER;
			bbm->table_init = 1;
		}

		if (table->total > bbm->max_reloc_entry) {
			printk("Relocation table exceed max num,");
			printk("cannot relocate block 0x%x\n", block);
			return -ENOSPC;
		}
		reloc_block = (mtd->size  >> bbm->erase_shift) - 1;
		
		//identify whether the block has been relocated
		for(i = table->total - 1; i >= 0; i --) {
			if(block == item[i].from)
				entry_num = i;
		}

		//find the available block with the largest number in reservered area
		for(i = table->total - 1; i >= 0; i --) {
			if (item[i].to != 65535) {
				if (reloc_block >= item[i].to)
					reloc_block = item[i].to - 1;
			} else {
				if (reloc_block >= item[i].from)
					reloc_block = item[i].from - 1;
			}
		}

		if (reloc_block < ((mtd->size >> bbm->erase_shift) - bbm->max_reloc_entry))
			return -ENOSPC;

		/* Make sure that reloc_block is pointing to a valid block */
		for (; ; reloc_block--) {
			for (i = table->total-1; i >= 0; i--) {
				if (reloc_block == item[i].from) {
					if (item[i].to != 65535)
						printk(KERN_ERR "Res block marked invalid \
								in reloc table\n");
					reloc_block--;
					i = table->total-1;
					if (reloc_block <
							((mtd->size >> bbm->erase_shift) -
							 bbm->max_reloc_entry))
						return -ENOSPC;
				}
			}
			/* The relocate table is full */
			if (reloc_block <
					((mtd->size >> bbm->erase_shift) - bbm->max_reloc_entry))
				return -ENOSPC;
			
			memset(&instr, 0, sizeof(struct erase_info));
			instr.mtd = mtd;
			instr.addr = reloc_block << bbm->erase_shift;
			instr.len = (1 << bbm->erase_shift);

			ret = mtd->erase(mtd, &instr);
			if (!ret)
				break;
			else {
				printk("status:0x%x found at erasing reloc block %d\n", reloc_block);
				/* skip it if the reloc_block is also a 
				 * bad block
				 */
				item[table->total].from = reloc_block;
				item[table->total].to = 65535;
				table->total++;
			}
		}
		/* Create the relocated block information in the table */
		//when the block is relocated before, blob should modify the original entry to new
		//relocated block and the old relocated block point to 65535. If not the situation, 
		//create a new entry
		if (entry_num != -1) {
			item[table->total].from = item[entry_num].to;
			item[table->total].to = 65535;
			table->total++;
			item[entry_num].to = reloc_block;
		} else {
			item[table->total].from = block;
			item[table->total].to = reloc_block;
			table->total++;
		}
	} else {
		return -ENOSPC;
	}
	
	return 0;
}

/* Write the relocation table back to device, if there's room. */
static int sync_reloc_tb(struct mtd_info *mtd, struct pxa3xx_bbm *bbm, int *idx)
{
	int obm, start_page, len, retlen;
	uint8_t *tmp;


	if (*idx >= max_bbt_slots) {
		printk(KERN_ERR "Can't write relocation table to device \
				any more.\n");
		return -1;
	}

	if (*idx < 0) {
		printk(KERN_ERR "Wrong Slot is specified.\n");
		return -1;
	}

	len = 4;
	len += bbm->table->total << 2;
	obm = calc_obm_ver();
	if (obm == MHN_OBM_V3) {
		/* write to device */
		/* the write page should be after the current slot */
		/* the num 2 is specified for we sync reloc_tb only after xdb */
		start_page = (1 << (bbm->erase_shift - bbm->page_shift)) - 2;
		start_page = start_page - *idx;

		printk(KERN_DEBUG "DUMP relocation table before write. \
				page:0x%x\n", start_page);

		tmp = (uint8_t *)bbm->data_buf;
		mtd->write(mtd, start_page << bbm->page_shift,
				1 << bbm->page_shift, &retlen, tmp);
		/* write to idx */
		(*idx)++;
	}
	return 0;
}

static int pxa3xx_scan_reloc_tb(struct mtd_info *mtd, struct pxa3xx_bbm *bbm)
{
	struct reloc_table *table = bbm->table;
	int page, maxslot, obm, valid = 0;
	int retlen, ret;

	obm = calc_obm_ver();

	if (obm == MHN_OBM_V2) {
		/* On MOBM V2, the relocation table resides in the last page
		 * of the first block.
		 */
		page = (1 << (bbm->erase_shift - bbm->page_shift)) - 1;
		memset(bbm->data_buf, 0, mtd->writesize + mtd->oobsize);
		ret = mtd->read(mtd, page << bbm->page_shift, mtd->writesize,
				&retlen, bbm->data_buf);

		if (ret == 0) {
			if (table->header == NAND_RELOC_HEADER)
				valid = 1;
		}
	} else if (obm == MHN_OBM_V3) {
		/* On MOBM V3, there're several relocation tables in the first
		 * block.
		 * When new bad blocks are found, a new relocation table will
		 * be generated and written back to the first block. But the
		 * original relocation table won't be erased. Even if the new
		 * relocation table is written wrong, system can still find an
		 * old one.
		 * One page contains one slot.
		 */
		maxslot = 1 << (bbm->erase_shift - bbm->page_shift);
		page = maxslot - max_bbt_slots;
		for (; page < maxslot; page++) {
			memset(bbm->data_buf, 0,
					mtd->writesize + mtd->oobsize);
			ret = mtd->read(mtd, (page << bbm->page_shift),
					mtd->writesize, &retlen, bbm->data_buf);

			if (ret == 0) {
				if (table->header != NAND_RELOC_HEADER) {
					continue;
				} else {
					bbm->current_slot = maxslot - page - 1;	
					valid = 1;
					break;
				}
			}
		}
	} else {
		printk(KERN_ERR "The version of MOBM isn't supported\n");
	}

	if (valid) {
		printk(KERN_DEBUG "relocation table at page:%d\n", page);
		bbm->table_init = 1;
		dump_reloc_table(bbm);
	} else {
		/* There should be a valid relocation table slot at least. */
		printk(KERN_ERR "NO VALID relocation table can be \
				recognized\n");
		printk(KERN_ERR "CAUTION: It may cause unpredicated error\n");
		printk(KERN_ERR "Please re-initialize the NAND flash.\n");
		memset((unsigned char *)bbm->table, 0,
				sizeof(struct reloc_table));
		bbm->table_init = 0;
		return -EINVAL;
	}
	return 0;
}

static int pxa3xx_init_reloc_tb(struct mtd_info *mtd, struct pxa3xx_bbm *bbm)
{
	int size = mtd->writesize + mtd->oobsize;

	if (cpu_is_pxa930() || cpu_is_pxa935()) {
		if (mtd->size <= 128 << 20) {
			/* if mtd size = 128MB, we use 20 relocation table */
			bbm->max_reloc_entry = PXA930_MAX_RELOC_ENTRY;
		} else {
			/* if mtd size = 256MB, we use 40 relocation table */
			bbm->max_reloc_entry = PXA930_NEW_MAX_RELOC_ENTRY;
		}
	} else
		bbm->max_reloc_entry = PXA3xx_MAX_RELOC_ENTRY;

	bbm->table_init = 0;

	bbm->data_buf = kzalloc(size, GFP_KERNEL);
	if (!bbm->data_buf) {
		return -ENOMEM;
	}
	bbm->table = (struct reloc_table *)bbm->data_buf;
	memset(bbm->table, 0x0, sizeof(struct reloc_table));

	bbm->reloc = (struct reloc_item *)((uint8_t *)bbm->data_buf +
			sizeof (struct reloc_item));
	memset(bbm->reloc, 0x0,
			sizeof(struct reloc_item) * bbm->max_reloc_entry);

	return pxa3xx_scan_reloc_tb(mtd, bbm);
}

static int pxa3xx_uninit_reloc_tb(struct mtd_info *mtd, struct pxa3xx_bbm *bbm)
{
	kfree(bbm->data_buf);
	return 0;
}

/* Find the relocated block of the bad one.
 * If it's a good block, return 0. Otherwise, return a relocated one.
 * idx points to the next relocation entry
 * If the relocated block is bad, an new entry will be added into the
 * bottom of the relocation table.
 */
static int pxa3xx_search_reloc_tb(struct mtd_info *mtd, struct pxa3xx_bbm *bbm, unsigned int block)
{
	struct reloc_table *table = bbm->table;
	struct reloc_item *item = bbm->reloc;
	int i, max, reloc_block;

	if ((block <= 0) ||
			(block > ((mtd->size >> bbm->erase_shift) - bbm->max_reloc_entry)) ||
			(bbm->table_init == 0) || (table->total == 0))
		return block;

	if (table->total > bbm->max_reloc_entry)
		table->total = bbm->max_reloc_entry;

	/* If can't find reloc tb entry for block, return block */
	reloc_block = block;
	max = table->total;
	for (i = max-1; i >= 0; i--) {
		if (block == item[i].from) {
			reloc_block = item[i].to;
			break;
		}
	}

	return reloc_block;
}

static int pxa3xx_mark_reloc_tb(struct mtd_info *mtd, struct pxa3xx_bbm *bbm, unsigned int block)
{
	int ret = 0;

	ret = update_reloc_tb(mtd, bbm, block);
	if (ret)
		return ret;

	ret = sync_reloc_tb(mtd, bbm, &(bbm->current_slot));
	return ret;
}

struct pxa3xx_bbm* alloc_pxa3xx_bbm(void)
{
	/* FIXME: We don't want to add module_init entry
	 * here to avoid dependency issue.
	 */
	struct pxa3xx_bbm *bbm;

	bbm = kzalloc(sizeof(struct pxa3xx_bbm), GFP_KERNEL);
	if (!bbm)
		return NULL;

	bbm->init = pxa3xx_init_reloc_tb;
	bbm->uninit = pxa3xx_uninit_reloc_tb;
	bbm->search = pxa3xx_search_reloc_tb;
	bbm->markbad = pxa3xx_mark_reloc_tb;

	return bbm;
}
EXPORT_SYMBOL(alloc_pxa3xx_bbm);

void free_pxa3xx_bbm(struct pxa3xx_bbm *bbm)
{
	if (bbm) {
		kfree(bbm);
		bbm = NULL;
	}
}
EXPORT_SYMBOL(free_pxa3xx_bbm);

