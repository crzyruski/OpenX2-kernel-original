#ifndef	__PXA3XX_BBT_H__
#define	__PXA3XX_BBT_H__

enum flash_type {
	FLASH_NAND,
	FLASH_ONENAND,
};

struct reloc_item {
	unsigned short from;
	unsigned short to;
};

struct reloc_table {
	unsigned short header;
	unsigned short total;
};

struct pxa3xx_bbm {
	int			flash_type;

	u32			current_slot;

	/* NOTES: this field impact the partition table. Please make sure
	 * that this value align with partitions definition.
	 */
	u32			max_reloc_entry;

	void			*data_buf;

	/* These two fields should be in (one)nand_chip.
	 * Add here to handle onenand_chip and nand_chip
	 * at the same time.
	 */
	int			page_shift;
	int			erase_shift;

	unsigned int		table_init;
	struct reloc_table	*table;
	struct reloc_item	*reloc;

	int	(*init)(struct mtd_info *mtd, struct pxa3xx_bbm *bbm);
	int	(*uninit)(struct mtd_info *mtd, struct pxa3xx_bbm *bbm);
	int	(*search)(struct mtd_info *mtd, struct pxa3xx_bbm *bbm,
			unsigned int block);
	int	(*markbad)(struct mtd_info *mtd, struct pxa3xx_bbm *bbm,
			unsigned int block);
};

struct pxa3xx_bbm* alloc_pxa3xx_bbm(void);
void free_pxa3xx_bbm(struct pxa3xx_bbm *);
#endif

