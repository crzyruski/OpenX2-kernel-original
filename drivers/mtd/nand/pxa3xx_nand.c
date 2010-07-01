/*
 * drivers/mtd/nand/pxa3xx_nand.c
 *
 * Copyright (C) 2005 Intel Coporation (chao.xie@intel.com)
 * Copyright (C) 2006 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Overview:
 *   This is a device driver for the NAND flash device on zylonite board
 *   which utilizes the Samsung K9K1216Q0C parts. This is a 64Mibit NAND
 *   flash device.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/pxa3xx_bbm.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/delay.h>
#include <mach/dma.h>
#include <mach/system.h>

#include <mach/pxa-regs.h>
#include <mach/regs-nand.h>
#include <mach/pxa3xx_nand.h>
#if defined(CONFIG_PXA3xx_DVFM)
#include <mach/dvfm.h>
#include <mach/pxa3xx_dvfm.h>
#include <asm/atomic.h>
#endif
#include <mach/pxa3xx-regs.h>

struct dfc_context dfc_context;
static struct pxa3xx_bbm *pxa3xx_bbm;

#define CONFIG_MTD_NAND_PXA3xx_UNLOCK
/* #define CONFIG_MTD_NAND_PXA3xx_DEBUG */
#ifdef CONFIG_MTD_NAND_PXA3xx_DEBUG
#define D1(x) do { \
		printk(KERN_DEBUG "%s: ", __FUNCTION__); \
		x; \
	}while(0)

#define	DPRINTK(fmt,args...) printk(KERN_DEBUG fmt, ##args )
#define PRINT_BUF(buf, num)	print_buf(buf, num)
#else
#define D1(x)
#define DPRINTK(fmt,args...)
#define PRINT_BUF(buf, num)
#endif

/* DFC timing 0 register */
#define DFC_TIMING_tRP		0
#define DFC_TIMING_tRH		3
#define DFC_TIMING_tWP		8
#define DFC_TIMING_tWH		11
#define DFC_TIMING_tCS		16
#define DFC_TIMING_tCH		19

/* DFC timing 1 register */
#define DFC_TIMING_tAR		0
#define DFC_TIMING_tWHR		4
#define DFC_TIMING_tR		16

/* max value for each timing setting in DFC */
#define DFC_TIMING_MAX_tCH	7
#define DFC_TIMING_MAX_tCS	7
#define DFC_TIMING_MAX_tWH	7
#define DFC_TIMING_MAX_tWP	7
#define DFC_TIMING_MAX_tRH	7
#define DFC_TIMING_MAX_tRP	7
#define DFC_TIMING_MAX_tR	65535
#define DFC_TIMING_MAX_tWHR	15
#define DFC_TIMING_MAX_tAR	15

#define	CHIP_DELAY_TIMEOUT	(2 * HZ/10)

/*
 * The Data Flash Controller Flash timing structure
 * For NAND flash used on Zylonite board(Samsung K9K1216Q0C),
 * user should use value at end of each row of following member
 * bracketed.
 */
struct dfc_flash_timing {
	uint32_t   tCH;	/* Enable signal hold time */
	uint32_t   tCS;	/* Enable signal setup time */
	uint32_t   tWH;	/* ND_nWE high duration */
	uint32_t   tWP;	/* ND_nWE pulse time */
	uint32_t   tRH;	/* ND_nRE high duration */
	uint32_t   tRP;	/* ND_nRE pulse width */
	uint32_t   tR;	/* ND_nWE high to ND_nRE low for read */
	uint32_t   tWHR;/* ND_nWE high to ND_nRE low delay for status read */
	uint32_t   tAR;	/* ND_ALE low to ND_nRE low delay */
};

/* DFC command type */
enum {
	DFC_CMD_READ		= 0x00000000,
	DFC_CMD_PROGRAM		= 0x00200000,
	DFC_CMD_ERASE		= 0x00400000,
	DFC_CMD_READ_ID		= 0x00600000,
	DFC_CMD_STATUS_READ	= 0x00800000,
	DFC_CMD_RESET		= 0x00a00000
};

/*
 * The Data Flash Controller Flash specification structure
 * For NAND flash used on Zylonite board(Samsung K9K1216Q0C),
 * user should use value at end of each row of following member
 * bracketed.
 */
struct dfc_flash_info {
	struct dfc_flash_timing timing; /* NAND Flash timing */

	int	 enable_arbiter;/* Data flash bus arbiter enable (ND_ARB_EN) */
	uint32_t page_per_block;/* Pages per block (PG_PER_BLK) */
	uint32_t row_addr_start;/* Row address start position (RA_START) */
	uint32_t read_id_bytes;	/* returned ID bytes(RD_ID_CNT) */
	uint32_t dfc_mode;	/* NAND, CARBONDALE, PIXLEY... (ND_MODE) */
	uint32_t ncsx;		/* Chip select don't care bit (NCSX) */
	uint32_t page_size;	/* Page size in bytes (PAGE_SZ) */
	uint32_t oob_size;	/* OOB size */
	uint32_t flash_width;	/* Width of Flash memory (DWIDTH_M) */
	uint32_t dfc_width;	/* Width of flash controller(DWIDTH_C) */
	uint32_t num_blocks;	/* Number of physical blocks in Flash */
	uint32_t chip_id;
	uint32_t read_prog_cycles;	/* Read Program address cycles */
#ifdef CONFIG_MTD_NAND_PXA3xx_UNLOCK
	uint32_t unlock_block_cycles;	/* Unlock Block address cycles */
#endif

	/* command codes */
	uint32_t read1;		/* Read */
	uint32_t read2;		/* unused, DFC don't support yet */
	uint32_t program;	/* two cycle command */
	uint32_t read_status;
	uint32_t read_id;
	uint32_t erase;		/* two cycle command */
	uint32_t reset;
	uint32_t lock;		/* lock whole flash */
	uint32_t unlock1;	/* unlock from lower lock */
	uint32_t unlock2;	/* unlock from upper lock */
	uint32_t lock_status;	/* read block lock status */

	/* addr2ndcb1 - encode address cycles into register NDCB1 */
	/* ndbbr2addr - convert register NDBBR to bad block address */
	int (*addr2ndcb1)(uint16_t cmd, uint32_t addr, uint32_t *p);
	int (*ndbbr2addr)(uint16_t cmd, uint32_t ndbbr,uint32_t *p);
};

enum {
	DFC_FLASH_NULL = 0 ,
	DFC_FLASH_Samsung_512Mb_X_16 = 1,
	DFC_FLASH_Samsung_2Gb_X_8 = 2,
	DFC_FLASH_Micron_1Gb_X_8 = 3,
	DFC_FLASH_Micron_1Gb_X_16 = 4,
	DFC_FLASH_STM_1Gb_X_16 = 5,
	DFC_FLASH_STM_2Gb_X_16 = 6,
	DFC_FLASH_STM_MCP_1Gb_X_16 = 7,
	DFC_FLASH_Toshiba2GbX16 = 8,
	DFC_FLASH_Toshiba1GbX16=9,//For X1 128MB nand
	DFC_FLASH_Hynix2GbX16=10,//For X1 /X2 256MB nand
	DFC_FLASH_END,
};

static int dfc_get_flash_info(int type, struct dfc_flash_info **flash_info);

#define		DFC_NDCR	0
#define		DFC_NDTR0CS0	1
#define		DFC_NDTR1CS0	3
#define		DFC_NDSR	5
#define		DFC_NDPCR	6
#define		DFC_NDBDR0	7
#define		DFC_NDBDR1	8
#define		DFC_NDDB	16
#define		DFC_NDCB0	18
#define		DFC_NDCB1	19
#define		DFC_NDCB2	20
#define		DFC_NDREDEL	24

/* The Data Flash Controller Mode structure */
struct dfc_mode {
	int   enable_dma;	/* DMA, or nonDMA mode */
	int   enable_ecc;	/* ECC on/off */
	int   enable_spare;	/* Spare enable */
	int   chip_select;	/* CS0 or CS1 */
};

/* The Data Flash Controller Context structure */
struct dfc_context {
	struct clk		*clk;		/* clock of NAND */
	struct clk		*smc_clk;	/* clock of SMC */
	unsigned char __iomem	*membase;	/* DFC register base */
	struct dfc_mode 	*dfc_mode;	/* DFC mode */
	int 			data_dma_ch;	/* Data DMA channel number */
	int 			cmd_dma_ch;	/* CMD  DMA channel number */
	struct 			dfc_flash_info *flash_info; /* Flash Spec */
};

#define NDCB0_DMA_ADDR	0x43100048
#define NDDB_DMA_ADDR	0x43100040

#define NDSR_MASK	(cpu_is_pxa935()? 0x1FFF:0xFFF)

/* The following data is a rough evaluation */

/* microsecond, for readID/readStatus/reset */
#define NAND_OTHER_TIMEOUT 		10
/* microsecond, for readID/readStatus/reset */
#define NAND_CMD_TIMEOUT		10

#define BBT_BLOCK_BAD	0x03
#define BBT_BLOCK_GOOD	0x00
#define BBT_BLOCK_REV1	0x01
#define BBT_BLOCK_REV2	0x02

#define BUFLEN		(2048 + 64)

/*
 * DFC data size enumeration transfered from/to controller,
 * including padding (zero)to be a multiple of 32.
 */
enum {
	DFC_DATA_SIZE_STATUS = 8,	/* ReadStatus/ReadBlockLockStatus */
	DFC_DATA_SIZE_ID = 7,	/* ReadID */

	DFC_DATA_SIZE_32 = 32,
	DFC_DATA_SIZE_512 = 512,	/* R/W disabling spare area */
	DFC_DATA_SIZE_520 = 520,	/* Spare=1, ECC=1 */
	DFC_DATA_SIZE_528 = 528,	/* Spare=1, ECC=0 */
	DFC_DATA_SIZE_544 = 544,	/* R/W enabling spare area.(DMA mode)*/

	DFC_DATA_SIZE_64 = 64,
	DFC_DATA_SIZE_2048 = 2048, 	/* R/W disabling spare area */
	DFC_DATA_SIZE_2088 = 2088,	/* R/W enabling spare area with ecc */
	DFC_DATA_SIZE_2112 = 2112,	/* R/W enabling spare area without ecc*/
	DFC_DATA_SIZE_2096 = 2096,	/* R/W enabling spare area */
	DFC_DATA_SIZE_UNUSED = 0xFFFF
};

/* DFC padding size enumeration transfered from/to controller */
enum {
	/*
	 * ReadStatus/ReadBlockLockStatus/ReadID/
	 * Read/Program disabling spare area(Both 512 and 2048)
	 * Read/Program enabling spare area, disabling ECC
	 */
	DFC_PADDING_SIZE_0 = 0,

	/* Read/program with SPARE_EN=1, ECC_EN=0, pgSize=512 */
	DFC_PADDING_SIZE_16 = 16,
	/* for read/program with SPARE_EN=1, ECC_EN=1, pgSize=512 and 2048 */
	DFC_PADDING_SIZE_24 = 24,
	DFC_PADDING_SIZE_UNUSED = 0xFFFF
};

/* use cache read to avoid STM 2G NAND failure */
//#define CACHE_READ

static unsigned int flash_config = DFC_FLASH_NULL;

static void dfc_set_timing(struct dfc_context *context,
		struct dfc_flash_timing *t);
static void dfc_set_dma(struct dfc_context *context);
static void dfc_set_ecc(struct dfc_context *context);
static void dfc_set_spare(struct dfc_context *context);

static int dfc_get_pattern(struct dfc_context *context, uint16_t cmd,
		int *data_size, int *padding);

static int dfc_wait_event(struct dfc_context *context, uint32_t event,
		uint32_t *event_out, uint32_t timeout, int enable_int);

static int dfc_send_cmd(struct dfc_context *context, uint16_t cmd,
		uint32_t addr, int num_pages);

static void dfc_stop(struct dfc_context *context);
static void dfc_read_fifo_partial(struct dfc_context *context,
		uint8_t *buffer, int nbytes, int data_size);
static void dfc_write_fifo_partial(struct dfc_context *context,
		uint8_t *buffer, int nbytes, int data_size);

static void dfc_read_fifo(struct dfc_context *context,
		uint8_t *buffer, int nbytes);
static void dfc_write_fifo(struct dfc_context *context,
		uint8_t *buffer, int nbytes);

static void dfc_read_badblock_addr(struct dfc_context *context,
		uint32_t *bbaddr);

static void dfc_clear_int(struct dfc_context *context,
		uint32_t int_mask);
static void dfc_enable_int(struct dfc_context *context,
		uint32_t int_mask);
static void dfc_disable_int(struct dfc_context *context,
		uint32_t int_mask);

/* high level primitives */
static int dfc_init(struct dfc_context *context, int type);
static int dfc_init_no_gpio(struct dfc_context *context, int type);

static int dfc_reset_flash(struct dfc_context *context);

static int dfc_setup_cmd_dma(struct dfc_context *context,
		uint16_t cmd, uint32_t addr, int num_pages,
		uint32_t *buf, uint32_t buf_phys,
		uint32_t next_desc_phys, uint32_t dma_int_en,
		struct pxa_dma_desc *dma_desc);

static int dfc_setup_data_dma(struct dfc_context *context,
		uint16_t cmd, uint32_t buf_phys,
		uint32_t next_desc_phys, uint32_t dma_int_en,
		struct pxa_dma_desc *dma_desc);

static void dfc_start_cmd_dma(struct dfc_context *context,
			struct pxa_dma_desc *dma_desc);
static void dfc_start_data_dma(struct dfc_context *context,
			struct pxa_dma_desc *dma_desc);
static int pxa3xx_nand_dev_ready(struct mtd_info *mtd);

static void set_dvfm_constraint(void);
static void unset_dvfm_constraint(void);
#ifdef CONFIG_PXA3xx_DVFM
static int dvfm_dev_idx;

static int nand_notifier_freq(struct notifier_block *nb,
				unsigned long val, void *data);
static struct notifier_block notifier_freq_block = {
	.notifier_call = nand_notifier_freq,
};

#endif

/*****************************************************************************
 * The DFC registers read/write routines
 *****************************************************************************/
#ifdef CONFIG_MTD_NAND_PXA3xx_FIX1
/* When this workaround is used, clock gating on NAND/SMC should be avoided. */
static inline void dfc_write(struct dfc_context *context, int offset,
		unsigned long value)
{
	unsigned long flags;
	unsigned int reg;

	if (offset == DFC_NDCR || offset == DFC_NDTR0CS0 || offset == DFC_NDTR1CS0 ||
		offset == DFC_NDREDEL) {

		local_irq_save(flags);

		/* Data Memory Barrier */
		__asm__ ("mcr p15, 0, %0, c7, c10, 5\n"
				"mrc p15,0, %0, c2, c0, 0\n"
				"mov %0,%0\n"
				"sub pc, pc, #4\n"
				:"=r"(reg));

		/* Drain Write Buffer */
		__asm__ ("mcr p15, 0, %0, c7, c10, 4\n"
				"mrc p15,0, %0, c2, c0, 0\n"
				"mov %0,%0\n"
				"sub pc, pc, #4\n"
				:"=r"(reg));

		clk_disable(dfc_context.clk);
	}

	writel(value, context->membase + (offset << 2));
	__raw_readl(context->membase + (offset << 2));

	if (offset == DFC_NDCR || offset == DFC_NDTR0CS0 || offset == DFC_NDTR1CS0 ||
		offset == DFC_NDREDEL) {
		clk_enable(dfc_context.clk);
		local_irq_restore(flags);
	}
}
#else
static inline void dfc_write(struct dfc_context *context, int offset,
		unsigned long value)
{
	offset <<= 2;
	writel(value, context->membase + offset);
	__raw_readl(context->membase + offset);
}
#endif

static inline unsigned int dfc_read(struct dfc_context *context, int offset)
{
	offset <<= 2;
	return __raw_readl(context->membase + offset);
}

/****************************************************************************
 * Flash Information
 ***************************************************************************/

static int Samsung512MbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p);
static int Samsung512MbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p);

static struct dfc_flash_info samsung512MbX16 =
{
	.timing = {
		.tCH = 10,	/* tCH, Enable signal hold time */
		.tCS = 0,	/* tCS, Enable signal setup time */
		.tWH = 20,	/* tWH, ND_nWE high duration */
		.tWP = 40,	/* tWP, ND_nWE pulse time */
		.tRH = 30,	/* tRH, ND_nRE high duration */
		.tRP = 40,	/* tRP, ND_nRE pulse width */
		/* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		.tR = 11123,
		/* tWHR, ND_nWE high to ND_nRE low delay for status read */
		.tWHR = 110,
		.tAR = 10,	/* tAR, ND_ALE low to ND_nRE low delay */
	},
	.enable_arbiter = 1,	/* Data flash bus arbiter enable */
	.page_per_block = 32,	/* Pages per block */
	.row_addr_start = 0,	/* Second cycle start, Row address start position */
	.read_id_bytes = 2,	/* 2 bytes, returned ID bytes */
	.dfc_mode = 0,		/* NAND mode */
	.ncsx = 0,
	.page_size = 512,	/* Page size in bytes */
	.oob_size = 16,		/* OOB size in bytes */
	.flash_width = 16,	/* Width of Flash memory */
	.dfc_width = 16,	/* Width of flash controller */
	.num_blocks = 4096,	/* Number of physical blocks in Flash */
	.chip_id =  0x46ec,
	.read_prog_cycles = 4,	/* Read, Program Cycles */
#ifdef CONFIG_MTD_NAND_PXA3xx_UNLOCK
	.unlock_block_cycles = 0,	/* Unlock Block address cycles */
#endif

	/* command codes */
	.read1 = 0x0000,	/* Read */
	.read2 = 0x0050,	/* Read1 unused, current DFC don't support */
	.program = 0x1080,	/* Write, two cycle command */
	.read_status = 0x0070,	/* Read status */
	.read_id = 0x0090,	/* Read ID */
	.erase =  0xD060,	/* Erase, two cycle command */
	.reset = 0x00FF,	/* Reset */
	.lock = 0x002A,		/* Lock whole flash */
	.unlock1 = 0x23,	/* unlock from lower lock */
	.unlock2 = 0x24,	/* unlock to upper lock */
	.lock_status = 0x007A,	/* Read block lock status */
	.addr2ndcb1 = Samsung512MbX16Addr2NDCB1,
	.ndbbr2addr = Samsung512MbX16NDBBR2Addr,
};

static int Samsung512MbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p)
{
	uint32_t ndcb1 = 0;

	if (addr >= 0x4000000) return -EINVAL;

	if (cmd == samsung512MbX16.read1 || cmd == samsung512MbX16.program) {
		ndcb1 = (addr & 0xFF) | ((addr >> 1) & 0x01FFFF00);
	} else if (cmd == samsung512MbX16.erase || cmd == samsung512MbX16.unlock1 ||
			cmd == samsung512MbX16.unlock2) {
		ndcb1 = ((addr >> 9) & 0x00FFFFFF);
	}

	*p = ndcb1;
	return 0;

}

static int Samsung512MbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p)
{
	*p = ndbbr << 9;
	return 0;
}


static int Samsung2GbX8Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p);
static int Samsung2GbX8NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p);

static struct dfc_flash_info samsung2GbX8 =
{
	.timing = {
		.tCH = 10,	/* tCH, Enable signal hold time */
		.tCS = 35,	/* tCS, Enable signal setup time */
		.tWH = 15,	/* tWH, ND_nWE high duration */
		.tWP = 25,	/* tWP, ND_nWE pulse time */
		.tRH = 20,	/* tRH, ND_nRE high duration */
		.tRP = 25,	/* tRP, ND_nRE pulse width */
		/* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		.tR = 25000,
		/* tWHR, ND_nWE high to ND_nRE low delay for status read */
		.tWHR = 60,
		.tAR = 10,	/* tAR, ND_ALE low to ND_nRE low delay */
	},
	.enable_arbiter = 1,	/* Data flash bus arbiter enable */
	.page_per_block = 64,	/* Pages per block */
	.row_addr_start = 1,	/* Second cycle start, Row address start position */
	.read_id_bytes = 4,	/* Returned ID bytes */
	.dfc_mode = 0,		/* NAND mode */
	.ncsx = 0,
	.page_size = 2048,	/* Page size in bytes */
	.oob_size = 64,		/* OOB size in bytes */
	.flash_width = 8,	/* Width of Flash memory */
	.dfc_width = 8,		/* Width of flash controller */
	.num_blocks = 2048,	/* Number of physical blocks in Flash */
	.chip_id =  0xaaec,
	.read_prog_cycles = 5,	/* Read, Program Cycles */
#ifdef CONFIG_MTD_NAND_PXA3xx_UNLOCK
	.unlock_block_cycles = 0,	/* Unlock Block address cycles */
#endif

	/* command codes */
	.read1 = 0x3000,	/* Read */
	.read2 = 0x0050,	/* Read1 unused, current DFC don't support */
	.program = 0x1080,	/* Write, two cycle command */
	.read_status = 0x0070,	/* Read status */
	.read_id = 0x0090,	/* Read ID */
	.erase =  0xD060,	/* Erase, two cycle command */
	.reset = 0x00FF,	/* Reset */
	.lock = 0x002A,		/* Lock whole flash */
	.unlock1 = 0x23,	/* unlock from lower lock */
	.unlock2 = 0x24,	/* unlock to upper lock */
	.lock_status = 0x007A,	/* Read block lock status */
	.addr2ndcb1 = Samsung2GbX8Addr2NDCB1,
	.ndbbr2addr = Samsung2GbX8NDBBR2Addr,
};

static int Samsung2GbX8Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p)
{
	uint32_t ndcb1 = 0;
	uint32_t page;

	if (addr >= 0x10000000)
		return -EINVAL;
	page = addr / samsung2GbX8.page_size;
	addr =  (page / samsung2GbX8.page_per_block) << 18 |
		(page % samsung2GbX8.page_per_block) << 12;

	if (cmd == samsung2GbX8.read1 || cmd == samsung2GbX8.program) {
		ndcb1 = (addr & 0xFFF) | ((addr << 4) & 0xFFFF0000);
	}
	else if (cmd == samsung2GbX8.erase || cmd == samsung2GbX8.unlock1 ||
			cmd == samsung2GbX8.unlock2) {
		ndcb1 = ((addr >> 18) << 6) & 0x1FFFF;
	}

	*p = ndcb1;
	return 0;
}

static int Samsung2GbX8NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p)
{
	if (cmd == samsung2GbX8.read1 || cmd == samsung2GbX8.program) {
		*p = ((ndbbr & 0xF) << 8) | ((ndbbr >> 8) << 16);
	}
	else if (cmd == samsung2GbX8.erase) {
		*p = (ndbbr >> 6) << 18;
	}

	return 0;
}

static int Micron1GbX8Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p);
static int Micron1GbX8NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p);

static struct dfc_flash_info micron1GbX8 =
{
	.timing = {
		.tCH = 10,	/* tCH, Enable signal hold time */
		.tCS = 25,	/* tCS, Enable signal setup time */
		.tWH = 15,	/* tWH, ND_nWE high duration */
		.tWP = 25,	/* tWP, ND_nWE pulse time */
		.tRH = 15,	/* tRH, ND_nRE high duration */
		.tRP = 25,	/* tRP, ND_nRE pulse width */
		/* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		.tR = 25000,
		/* tWHR, ND_nWE high to ND_nRE low delay for status read */
		.tWHR = 60,
		.tAR = 10,	/* tAR, ND_ALE low to ND_nRE low delay */
	},
	.enable_arbiter = 1,	/* Data flash bus arbiter enable */
	.page_per_block = 64,	/* Pages per block */
	.row_addr_start = 1,	/* Second cycle start, Row address start position */
	.read_id_bytes = 4,	/* Returned ID bytes */
	.dfc_mode = 0,		/* NAND mode */
	.ncsx = 0,
	.page_size = 2048,	/* Page size in bytes */
	.oob_size = 64,		/* OOB size in bytes */
	.flash_width = 8,	/* Width of Flash memory */
	.dfc_width = 8,		/* Width of flash controller */
	.num_blocks = 1024,	/* Number of physical blocks in Flash */
	.chip_id =  0xa12c,
	.read_prog_cycles = 4,	/* Read, Program Cycles */
#ifdef CONFIG_MTD_NAND_PXA3xx_UNLOCK
	.unlock_block_cycles = 2,	/* Unlock Block address cycles */
#endif

	/* command codes */
	.read1 = 0x3000,	/* Read */
	.read2 = 0x0050,	/* Read1 unused, current DFC don't support */
	.program = 0x1080,	/* Write, two cycle command */
	.read_status = 0x0070,	/* Read status */
	.read_id = 0x0090,	/* Read ID */
	.erase =  0xD060,	/* Erase, two cycle command */
	.reset = 0x00FF,	/* Reset */
	.lock = 0x002A,		/* Lock whole flash */
	.unlock1 = 0x23,	/* unlock from lower lock */
	.unlock2 = 0x24,	/* unlock to upper lock */
	.lock_status = 0x007A,	/* Read block lock status */
	.addr2ndcb1 = Micron1GbX8Addr2NDCB1,
	.ndbbr2addr = Micron1GbX8NDBBR2Addr,
};

static int Micron1GbX8Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p)
{
	uint32_t ndcb1 = 0;
	uint32_t page;

	if (addr >= 0x8000000)
		return -EINVAL;
	page = addr / micron1GbX8.page_size;
	addr =  (page / micron1GbX8.page_per_block) << 18 |
		(page % micron1GbX8.page_per_block) << 12;

	if (cmd == micron1GbX8.read1 || cmd == micron1GbX8.program) {
		ndcb1 = (addr & 0xFFF) | ((addr << 4) & 0xFFFF0000);
	}
	else if (cmd == micron1GbX8.erase || cmd == micron1GbX8.unlock1 ||
			cmd == micron1GbX8.unlock2) {
		ndcb1 = ((addr >> 18) << 6) & 0xFFFF;
	}

	*p = ndcb1;
	return 0;
}

static int Micron1GbX8NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p)
{
	if (cmd == micron1GbX8.read1 || cmd == micron1GbX8.program) {
		*p = ((ndbbr & 0xF) << 8) | ((ndbbr >> 8) << 16);
	}
	else if (cmd == micron1GbX8.erase) {
		*p = (ndbbr >> 6) << 18;
	}

	return 0;
}


static int Micron1GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p);
static int Micron1GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p);

static struct dfc_flash_info micron1GbX16 =
{
	.timing = {
		.tCH = 10,	/* tCH, Enable signal hold time */
		.tCS = 25,	/* tCS, Enable signal setup time */
		.tWH = 15,	/* tWH, ND_nWE high duration */
		.tWP = 25,	/* tWP, ND_nWE pulse time */
		.tRH = 15,	/* tRH, ND_nRE high duration */
		.tRP = 25,	/* tRP, ND_nRE pulse width */
		/* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		.tR = 25000,
		/* tWHR, ND_nWE high to ND_nRE low delay for status read */
		.tWHR = 60,
		.tAR = 10,	/* tAR, ND_ALE low to ND_nRE low delay */
	},
	.enable_arbiter = 1,	/* Data flash bus arbiter enable */
	.page_per_block = 64,	/* Pages per block */
	.row_addr_start = 1,	/* Second cycle start, Row address start position */
	.read_id_bytes = 4,	/* Returned ID bytes */
	.dfc_mode = 0,		/* NAND mode */
	.ncsx = 0,
	.page_size = 2048,	/* Page size in bytes */
	.oob_size = 64,		/* OOB size in bytes */
	.flash_width = 16,	/* Width of Flash memory */
	.dfc_width = 16,	/* Width of flash controller */
	.num_blocks = 1024,	/* Number of physical blocks in Flash */
	.chip_id =  0xb12c,
	.read_prog_cycles = 4,	/* Read, Program Cycles */
#ifdef CONFIG_MTD_NAND_PXA3xx_UNLOCK
	.unlock_block_cycles = 2,	/* Unlock Block address cycles */
#endif

	/* command codes */
	.read1 = 0x3000,	/* Read */
	.read2 = 0x0050,	/* Read1 unused, current DFC don't support */
	.program = 0x1080,	/* Write, two cycle command */
	.read_status = 0x0070,	/* Read status */
	.read_id = 0x0090,	/* Read ID */
	.erase =  0xD060,	/* Erase, two cycle command */
	.reset = 0x00FF,	/* Reset */
	.lock = 0x002A,		/* Lock whole flash */
	.unlock1 = 0x23,	/* unlock from lower lock */
	.unlock2 = 0x24,	/* unlock to upper lock */
	.lock_status = 0x007A,	/* Read block lock status */
	.addr2ndcb1 = Micron1GbX16Addr2NDCB1,
	.ndbbr2addr = Micron1GbX16NDBBR2Addr,
};

static int Micron1GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p)
{
	uint32_t ndcb1 = 0;
	uint32_t page;

	if (addr >= 0x8000000)
		return -EINVAL;
	page = addr / micron1GbX16.page_size;
	addr =  (page / micron1GbX16.page_per_block) << 17 |
		(page % micron1GbX16.page_per_block) << 11;

	if (cmd == micron1GbX16.read1 || cmd == micron1GbX16.program) {
		ndcb1 = (addr & 0x7FF) | ((addr << 5) & 0xFFFF0000);
	}
	else if (cmd == micron1GbX16.erase || cmd == micron1GbX16.unlock1 ||
			cmd == micron1GbX16.unlock2) {
		ndcb1 = ((addr >> 17) << 6) & 0xFFFF;
	}
	*p = ndcb1;
	return 0;
}

static int Micron1GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p)
{
	if (cmd == micron1GbX16.read1 || cmd == micron1GbX16.program) {
		*p = ((ndbbr & 0x7) << 8) | ((ndbbr >> 8) << 16);
	}
	else if (cmd == micron1GbX16.erase) {
		*p = (ndbbr >> 6) << 17;
	}

	return 0;
}

static int STM1GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p);
static int STM1GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p);

static struct dfc_flash_info stm1GbX16 =
{
	.timing = {
		.tCH = 10,	/* tCH, Enable signal hold time */
		.tCS = 10,	/* tCS, Enable signal setup time */
		.tWH = 20,	/* tWH, ND_nWE high duration */
		.tWP = 25,	/* tWP, ND_nWE pulse time */
		.tRH = 20,	/* tRH, ND_nRE high duration */
		.tRP = 25,	/* tRP, ND_nRE pulse width */
		/* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		.tR = 25000,
		/* tWHR, ND_nWE high to ND_nRE low delay for status read */
		.tWHR = 60,
		.tAR = 10,	/* tAR, ND_ALE low to ND_nRE low delay */
	},
	.enable_arbiter = 1,	/* Data flash bus arbiter enable */
	.page_per_block = 64,	/* Pages per block */
	.row_addr_start = 1,	/* Second cycle start, Row address start position */
	.read_id_bytes = 4,	/* Returned ID bytes */
	.dfc_mode = 0,		/* NAND mode */
	.ncsx = 0,
	.page_size = 2048,	/* Page size in bytes */
	.oob_size = 64,		/* OOB size in bytes */
	.flash_width = 16,	/* Width of Flash memory */
	.dfc_width = 16,	/* Width of flash controller */
	.num_blocks = 1024,	/* Number of physical blocks in Flash */
	.chip_id =  0xc120,
	.read_prog_cycles = 4,	/* Read, Program Cycles */
#ifdef CONFIG_MTD_NAND_PXA3xx_UNLOCK
	.unlock_block_cycles = 2,	/* Unlock Block address cycles */
#endif

	/* command codes */
	.read1 = 0x3000,	/* Read */
	.read2 = 0x0050,	/* Read1 unused, current DFC don't support */
	.program = 0x1080,	/* Write, two cycle command */
	.read_status = 0x0070,	/* Read status */
	.read_id = 0x0090,	/* Read ID */
	.erase =  0xD060,	/* Erase, two cycle command */
	.reset = 0x00FF,	/* Reset */
	.lock = 0x002A,		/* Lock whole flash */
	.unlock1 = 0x23,	/* unlock from lower lock */
	.unlock2 = 0x24,	/* unlock to upper lock */
	.lock_status = 0x007A,	/* Read block lock status */
	.addr2ndcb1 = STM1GbX16Addr2NDCB1,
	.ndbbr2addr = STM1GbX16NDBBR2Addr,
};

static int STM1GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p)
{
	uint32_t ndcb1 = 0;
	uint32_t page;

	if (addr >= 0x8000000)
		return -EINVAL;
	page = addr / stm1GbX16.page_size;
	addr =  (page / stm1GbX16.page_per_block) << 17 |
		(page % stm1GbX16.page_per_block) << 11;

	if (cmd == stm1GbX16.read1 || cmd == stm1GbX16.program) {
		ndcb1 = (addr & 0x7FF) | ((addr << 5) & 0xFFFF0000);
	}
	else if (cmd == stm1GbX16.erase || cmd == stm1GbX16.unlock1 ||
			cmd == stm1GbX16.unlock2) {
		ndcb1 = ((addr >> 17) << 6) & 0xFFFF;
	}
	*p = ndcb1;
	return 0;
}

static int STM1GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p)
{
	if (cmd == stm1GbX16.read1 || cmd == stm1GbX16.program) {
		*p = ((ndbbr & 0x7) << 8) | ((ndbbr >> 8) << 16);
	}
	else if (cmd == stm1GbX16.erase) {
		*p = (ndbbr >> 6) << 17;
	}

	return 0;
}

static int STM70nm1GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p);
static int STM70nm1GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p);

static struct dfc_flash_info stm70nm1GbX16 =
{
	.timing = {
		.tCH = 30,	/* tCH, Enable signal hold time */
		.tCS = 35,	/* tCS, Enable signal setup time */
		.tWH = 15,	/* tWH, ND_nWE high duration */
		.tWP = 25,	/* tWP, ND_nWE pulse time */
		.tRH = 15,	/* tRH, ND_nRE high duration */
		.tRP = 25,	/* tRP, ND_nRE pulse width */
		/* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		.tR = 25000,
		/* tWHR, ND_nWE high to ND_nRE low delay for status read */
		.tWHR = 60,
		.tAR = 10,	/* tAR, ND_ALE low to ND_nRE low delay */
	},
	.enable_arbiter = 1,	/* Data flash bus arbiter enable */
	.page_per_block = 64,	/* Pages per block */
	.row_addr_start = 1,	/* Second cycle start, Row address start position */
	.read_id_bytes = 4,	/* Returned ID bytes */
	.dfc_mode = 0,		/* NAND mode */
	.ncsx = 0,
	.page_size = 2048,	/* Page size in bytes */
	.oob_size = 64,		/* OOB size in bytes */
	.flash_width = 16,	/* Width of Flash memory */
	.dfc_width = 16,	/* Width of flash controller */
	.num_blocks = 1024,	/* Number of physical blocks in Flash */
	.chip_id =  0xb120,
	.read_prog_cycles = 4,	/* Read, Program Cycles */
#ifdef CONFIG_MTD_NAND_PXA3xx_UNLOCK
	.unlock_block_cycles = 2,	/* Unlock Block address cycles */
#endif

	/* command codes */
#ifdef CACHE_READ
	.read1 = 0x3100,	/* Cache Read */
#else
 	.read1 = 0x3000,	/* Read */
#endif
	.read2 = 0x0050,	/* Read1 unused, current DFC don't support */
	.program = 0x1080,	/* Write, two cycle command */
	.read_status = 0x0070,	/* Read status */
	.read_id = 0x0090,	/* Read ID */
	.erase =  0xD060,	/* Erase, two cycle command */
	.reset = 0x00FF,	/* Reset */
	.lock = 0x002A,		/* Lock whole flash */
	.unlock1 = 0x23,	/* unlock from lower lock */
	.unlock2 = 0x24,	/* unlock to upper lock */
	.lock_status = 0x007A,	/* Read block lock status */
	.addr2ndcb1 = STM70nm1GbX16Addr2NDCB1,
	.ndbbr2addr = STM70nm1GbX16NDBBR2Addr,
};

static int STM70nm1GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p)
{
	uint32_t ndcb1 = 0;
	uint32_t page;

	if (addr >= 0x8000000)
		return -EINVAL;
	page = addr / stm70nm1GbX16.page_size;
	addr =  (page / stm70nm1GbX16.page_per_block) << 17 |
		(page % stm70nm1GbX16.page_per_block) << 11;

	if (cmd == stm70nm1GbX16.read1 || cmd == stm70nm1GbX16.program) {
		ndcb1 = (addr & 0x7FF) | ((addr << 5) & 0xFFFF0000);
	}
	else if (cmd == stm70nm1GbX16.erase || cmd == stm70nm1GbX16.unlock1 ||
			cmd == stm70nm1GbX16.unlock2) {
		ndcb1 = ((addr >> 17) << 6) & 0xFFFF;
	}
	*p = ndcb1;
	return 0;
}

static int STM70nm1GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p)
{
	if (cmd == stm70nm1GbX16.read1 || cmd == stm70nm1GbX16.program) {
		*p = ((ndbbr & 0x7) << 8) | ((ndbbr >> 8) << 16);
	}
	else if (cmd == stm70nm1GbX16.erase) {
		*p = (ndbbr >> 6) << 17;
	}

	return 0;
}

static int STM2GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p);
static int STM2GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p);

static struct dfc_flash_info stm2GbX16 =
{
	.timing = {
		.tCH = 10,      /* tCH, Enable signal hold time */
		.tCS = 35,      /* tCS, Enable signal setup time */
		.tWH = 15,      /* tWH, ND_nWE high duration */
		.tWP = 25,      /* tWP, ND_nWE pulse time */
		.tRH = 15,      /* tRH, ND_nRE high duration */
		.tRP = 25,      /* tRP, ND_nRE pulse width */
		/* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		.tR = 25000,
		/* tWHR, ND_nWE high to ND_nRE low delay for status read */
		.tWHR = 60,
		.tAR = 10,      /* tAR, ND_ALE low to ND_nRE low delay */
	},
	.enable_arbiter = 1,    /* Data flash bus arbiter enable */
	.page_per_block = 64,   /* Pages per block */
	.row_addr_start = 1,	/* Second cycle start, Row address start position */
	.read_id_bytes = 4,     /* Returned ID bytes */
	.dfc_mode = 0,          /* NAND mode */
	.ncsx = 0,
	.page_size = 2048,      /* Page size in bytes */
	.oob_size = 64,         /* OOB size in bytes */
	.flash_width = 16,      /* Width of Flash memory */
	.dfc_width = 16,        /* Width of flash controller */
	.num_blocks = 2048,     /* Number of physical blocks in Flash */
	.chip_id =  0xba20,
	.read_prog_cycles = 5,	/* Read, Program Cycles */
#ifdef CONFIG_MTD_NAND_PXA3xx_UNLOCK
	.unlock_block_cycles = 3,	/* Unlock Block address cycles */
#endif

	/* command codes */
#ifdef CACHE_READ
	.read1 = 0x3100,	/* Read */
#else
	.read1 = 0x3000,        /* Read */
#endif
	.read2 = 0x0050,        /* Read1 unused, current DFC don't support */
	.program = 0x1080,      /* Write, two cycle command */
	.read_status = 0x0070,  /* Read status */
	.read_id = 0x0090,      /* Read ID */
	.erase =  0xD060,       /* Erase, two cycle command */
	.reset = 0x00FF,        /* Reset */
	.lock = 0x002A,         /* Lock whole flash */
	.unlock1 = 0x23,	/* unlock from lower lock */
	.unlock2 = 0x24,	/* unlock to upper lock */
	.lock_status = 0x007A,  /* Read block lock status */
	.addr2ndcb1 = STM2GbX16Addr2NDCB1,
	.ndbbr2addr = STM2GbX16NDBBR2Addr,
};

static int STM2GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p)
{
	uint32_t ndcb1 = 0;
	uint32_t page;

	if (addr >= 0x10000000)
		return -EINVAL;
	page = addr / stm2GbX16.page_size;
	addr =  (page / stm2GbX16.page_per_block) << 17 |
		(page % stm2GbX16.page_per_block) << 11;

	if (cmd == stm2GbX16.read1 || cmd == stm2GbX16.program) {
		ndcb1 = (addr & 0x7FF) | ((addr << 5) & 0xFFFF0000);
	}
	else if (cmd == stm2GbX16.erase || cmd == stm2GbX16.unlock1 ||
			cmd == stm2GbX16.unlock2) {
		ndcb1 = ((addr >> 17) << 6) & 0x1FFFF;
	}
	*p = ndcb1;
	return 0;
}

static int STM2GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p)
{
	if (cmd == stm2GbX16.read1 || cmd == stm2GbX16.program) {
		*p = ((ndbbr & 0x7) << 8) | ((ndbbr >> 8) << 16);
	}
	else if (cmd == stm2GbX16.erase) {
		*p = (ndbbr >> 6) << 17;
	}

	return 0;
}

//The following two type is for Xphone /X2 256MB NAND 
static int HYNIX2GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p);
static int HYNIX2GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p);

static struct dfc_flash_info hynix2GbX16 =
{
	.timing = {
		.tCH = 10,      /* tCH, Enable signal hold time */
		.tCS = 35,      /* tCS, Enable signal setup time */
		.tWH = 15,      /* tWH, ND_nWE high duration */
		.tWP = 30,      /* tWP, ND_nWE pulse time */
		.tRH = 25,      /* tRH, ND_nRE high duration */
		.tRP = 50,      /* tRP, ND_nRE pulse width */
		/* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		.tR = 25000,
		/* tWHR, ND_nWE high to ND_nRE low delay for status read */
		.tWHR = 60,
		.tAR = 15,      /* tAR, ND_ALE low to ND_nRE low delay */
	},
	.enable_arbiter = 1,    /* Data flash bus arbiter enable */
	.page_per_block = 64,   /* Pages per block */
	.row_addr_start = 1,	/* Second cycle start, Row address start position */
	.read_id_bytes = 4,     /* Returned ID bytes */
	.dfc_mode = 0,          /* NAND mode */
	.ncsx = 0,
	.page_size = 2048,      /* Page size in bytes */
	.oob_size = 64,         /* OOB size in bytes */
	.flash_width = 16,      /* Width of Flash memory */
	.dfc_width = 16,        /* Width of flash controller */
	.num_blocks = 2048,     /* Number of physical blocks in Flash */
	.chip_id =  0xbaad,
	.read_prog_cycles = 5,	/* Read, Program Cycles */

	/* command codes */
	.read1 = 0x3000,        /* Read */
	.read2 = 0x0050,        /* Read1 unused, current DFC don't support */
	.program = 0x1080,      /* Write, two cycle command */
	.read_status = 0x0070,  /* Read status */
	.read_id = 0x0090,      /* Read ID */
	.erase =  0xD060,       /* Erase, two cycle command */
	.reset = 0x00FF,        /* Reset */
	.lock = 0x0050,         /* Lock whole flash */
	.unlock1 = 0x00,	/* unlock from lower lock */
	.unlock2 = 0x50,	/* unlock to upper lock */
	.lock_status = 0x0050,  /* Read block lock status */
	.addr2ndcb1 = HYNIX2GbX16Addr2NDCB1,
	.ndbbr2addr = HYNIX2GbX16NDBBR2Addr,
};

static int HYNIX2GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p)
{
	uint32_t ndcb1 = 0;
	uint32_t page;

	if (addr >= 0x10000000)
		return -EINVAL;
	page = addr / hynix2GbX16.page_size;
	addr =  (page / hynix2GbX16.page_per_block) << 18 |
		(page % hynix2GbX16.page_per_block) << 12;

	if (cmd == hynix2GbX16.read1 || cmd == hynix2GbX16.program) {
		ndcb1 = (addr & 0xFFF) | ((addr << 4) & 0xFFFF0000);
	}
	else if (cmd == hynix2GbX16.erase) {
		ndcb1 = ((addr >> 18) << 6) & 0x3FFFF;
	}
	*p = ndcb1;
	return 0;
}

static int HYNIX2GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p)
{
	if (cmd == hynix2GbX16.read1 || cmd == hynix2GbX16.program) {
		*p = ((ndbbr & 0xF) << 8) | ((ndbbr >> 8) << 16);
	}
	else if (cmd == hynix2GbX16.erase) {
		*p = (ndbbr >> 6) << 18;
	}

	return 0;
}


static int TOSHIBA1GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p);
static int TOSHIBA1GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p);

static struct dfc_flash_info toshiba1GbX16 =
{
	.timing = {
		.tCH =  6,      /* tCH, Enable signal hold time */
		.tCS = 10,      /* tCS, Enable signal setup time */
		.tWH = 15,      /* tWH, ND_nWE high duration */
		.tWP = 30,      /* tWP, ND_nWE pulse time */
		.tRH = 25,      /* tRH, ND_nRE high duration */
		.tRP = 50,      /* tRP, ND_nRE pulse width */
		/* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		.tR = 25000,
		/* tWHR, ND_nWE high to ND_nRE low delay for status read */
		.tWHR = 60,
		.tAR = 10,      /* tAR, ND_ALE low to ND_nRE low delay */
	},
	.enable_arbiter = 1,    /* Data flash bus arbiter enable */
	.page_per_block = 64,   /* Pages per block */
	.row_addr_start = 1,	/* Second cycle start, Row address start position */
	.read_id_bytes = 4,     /* Returned ID bytes */
	.dfc_mode = 0,          /* NAND mode */
	.ncsx = 0,
	.page_size = 2048,      /* Page size in bytes */
	.oob_size = 64,         /* OOB size in bytes */
	.flash_width = 16,      /* Width of Flash memory */
	.dfc_width = 16,        /* Width of flash controller */
	.num_blocks = 1024,     /* Number of physical blocks in Flash */
	.chip_id =  0xb198,
	.read_prog_cycles = 4,	/* Read, Program Cycles */

	/* command codes */
	.read1 = 0x3000,        /* Read */
	.read2 = 0x0050,        /* Read1 unused, current DFC don't support */
	.program = 0x1080,      /* Write, two cycle command */
	.read_status = 0x0070,  /* Read status */
	.read_id = 0x0090,      /* Read ID */
	.erase =  0xD060,       /* Erase, two cycle command */
	.reset = 0x00FF,        /* Reset */
	.lock = 0x002A,         /* Lock whole flash */
	.unlock1 = 0x00,	/* unlock from lower lock */
	.unlock2 = 0x2A,	/* unlock to upper lock */
	.lock_status = 0x007A,  /* Read block lock status */
    	.addr2ndcb1 = TOSHIBA1GbX16Addr2NDCB1,                                        
	.ndbbr2addr = TOSHIBA1GbX16NDBBR2Addr,                                        
};

static int TOSHIBA1GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p)
{
	uint32_t ndcb1 = 0;
	uint32_t page;

	if (addr >= 0x8000000)
		return -EINVAL;
	page = addr / toshiba1GbX16.page_size;
	addr =  (page / toshiba1GbX16.page_per_block) << 18 |
		(page % toshiba1GbX16.page_per_block) << 12;

	if (cmd == toshiba1GbX16.read1 || cmd == toshiba1GbX16.program) {
		ndcb1 = (addr & 0xFFF) | ((addr << 4) & 0xFFFF0000);
	}
	else if (cmd == toshiba1GbX16.erase) {
		ndcb1 = ((addr >> 18) << 6) & 0x1FFFF;
	}
	*p = ndcb1;
	return 0;
}

static int TOSHIBA1GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p)
{
	if (cmd == toshiba1GbX16.read1 || cmd == toshiba1GbX16.program) {
		*p = ((ndbbr & 0xF) << 8) | ((ndbbr >> 8) << 16);
	}
	else if (cmd == toshiba1GbX16.erase) {
		*p = (ndbbr >> 6) << 18;
	}

	return 0;
}

static int TOSHIBA2GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p);
static int TOSHIBA2GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p);

static struct dfc_flash_info toshiba2GbX16 =
{
	.timing = {
		.tCH =  6,      /* tCH, Enable signal hold time */
		.tCS = 10,      /* tCS, Enable signal setup time */
		.tWH = 15,      /* tWH, ND_nWE high duration */
		.tWP = 30,      /* tWP, ND_nWE pulse time */
		.tRH = 25,      /* tRH, ND_nRE high duration */
		.tRP = 50,      /* tRP, ND_nRE pulse width */
		/* tR = tR+tRR+tWB+1, ND_nWE high to ND_nRE low for read */
		.tR = 25000,
		/* tWHR, ND_nWE high to ND_nRE low delay for status read */
		.tWHR = 60,
		.tAR = 10,      /* tAR, ND_ALE low to ND_nRE low delay */
	},
	.enable_arbiter = 1,    /* Data flash bus arbiter enable */
	.page_per_block = 64,   /* Pages per block */
	.row_addr_start = 1,	/* Second cycle start, Row address start position */
	.read_id_bytes = 4,     /* Returned ID bytes */
	.dfc_mode = 0,          /* NAND mode */
	.ncsx = 0,
	.page_size = 2048,      /* Page size in bytes */
	.oob_size = 64,         /* OOB size in bytes */
	.flash_width = 16,      /* Width of Flash memory */
	.dfc_width = 16,        /* Width of flash controller */
	.num_blocks = 2048,     /* Number of physical blocks in Flash */
	.chip_id =  0xba98,
	.read_prog_cycles = 5,	/* Read, Program Cycles */
#ifdef CONFIG_MTD_NAND_PXA3xx_UNLOCK
	.unlock_block_cycles = 0,	/* Unlock Block address cycles */
#endif

	/* command codes */
	.read1 = 0x3000,        /* Read */
	.read2 = 0x0050,        /* Read1 unused, current DFC don't support */
	.program = 0x1080,      /* Write, two cycle command */
	.read_status = 0x0070,  /* Read status */
	.read_id = 0x0090,      /* Read ID */
	.erase =  0xD060,       /* Erase, two cycle command */
	.reset = 0x00FF,        /* Reset */
	.lock = 0x002A,         /* Lock whole flash */
	.unlock1 = 0x23,	/* unlock from lower lock */
	.unlock2 = 0x24,	/* unlock to upper lock */
	.lock_status = 0x007A,  /* Read block lock status */
	.addr2ndcb1 = TOSHIBA2GbX16Addr2NDCB1,
	.ndbbr2addr = TOSHIBA2GbX16NDBBR2Addr,
};

static int TOSHIBA2GbX16Addr2NDCB1(uint16_t cmd, uint32_t addr, uint32_t *p)
{
	uint32_t ndcb1 = 0;
	uint32_t page;

	if (addr >= 0x10000000)
		return -EINVAL;
	page = addr / toshiba2GbX16.page_size;
	addr =  (page / toshiba2GbX16.page_per_block) << 18 |
		(page % toshiba2GbX16.page_per_block) << 12;

	if (cmd == toshiba2GbX16.read1 || cmd == toshiba2GbX16.program) {
		ndcb1 = (addr & 0xFFF) | ((addr << 4) & 0xFFFF0000);
	}
	else if (cmd == toshiba2GbX16.erase || cmd == toshiba2GbX16.unlock1 ||
			cmd == toshiba2GbX16.unlock2) {
		ndcb1 = ((addr >> 18) << 6) & 0x1FFFF;
	}
	*p = ndcb1;
	return 0;
}

static int TOSHIBA2GbX16NDBBR2Addr(uint16_t cmd, uint32_t ndbbr, uint32_t *p)
{
	if (cmd == toshiba2GbX16.read1 || cmd == toshiba2GbX16.program) {
		*p = ((ndbbr & 0xF) << 8) | ((ndbbr >> 8) << 16);
	}
	else if (cmd == toshiba2GbX16.erase) {
		*p = (ndbbr >> 6) << 18;
	}

	return 0;
}

static struct {
	int type;
	struct dfc_flash_info *flash_info;
} type_info[] = {
	{ DFC_FLASH_Samsung_512Mb_X_16, &samsung512MbX16},
	{ DFC_FLASH_Samsung_2Gb_X_8, &samsung2GbX8},
	{ DFC_FLASH_Micron_1Gb_X_8, &micron1GbX8},
	{ DFC_FLASH_Micron_1Gb_X_16, &micron1GbX16},
	{ DFC_FLASH_STM_1Gb_X_16, &stm1GbX16},
	{ DFC_FLASH_STM_2Gb_X_16, &stm2GbX16},
	{ DFC_FLASH_STM_MCP_1Gb_X_16, &stm70nm1GbX16},
	{ DFC_FLASH_Toshiba2GbX16, &toshiba2GbX16},
	{ DFC_FLASH_Toshiba1GbX16,&toshiba1GbX16},
	{ DFC_FLASH_Hynix2GbX16, &hynix2GbX16},	
	{ DFC_FLASH_NULL, NULL},
};

static int dfc_get_flash_info(int type, struct dfc_flash_info **flash_info)
{
	uint32_t i = 0;

	while(type_info[i].type != DFC_FLASH_NULL) {
		if (type_info[i].type == type) {
			*flash_info = type_info[i].flash_info;
			return 0;
		}
		i++;
	}
	*flash_info = NULL;
	return -EINVAL;
}

/******************************************************************************
  dfc_set_timing

  Description:
	This function sets flash timing property in DFC timing register
	according to input timing value embodied in context structure.
	It is called once during the hardware initialization.
  Input Parameters:
  Output Parameters:
	None
  Returns:
	None
*******************************************************************************/

#define CLOCK_NS	DFC_CLOCK/1000

static void dfc_set_timing(struct dfc_context *context,
		struct dfc_flash_timing *t)
{
	struct dfc_flash_timing timing = *t;
	unsigned long DFC_CLOCK;

	uint32_t  r0 = 0;
	uint32_t  r1 = 0;

	DFC_CLOCK = clk_get_rate(context->clk) / 1000000;

	/*
	 * num of clock cycles = time (ns) / one clock sycle (ns) + 1
	 * - integer division will truncate the result, so add a 1 in all cases
	 * - subtract the extra 1 cycle added to all register timing values
	 */
	timing.tCH = min(((int) (timing.tCH * CLOCK_NS) + 1),
			DFC_TIMING_MAX_tCH);
	timing.tCS = min(((int) (timing.tCS * CLOCK_NS) + 1),
			DFC_TIMING_MAX_tCS);
	timing.tWH = min(((int) (timing.tWH * CLOCK_NS) + 1),
			DFC_TIMING_MAX_tWH);
	timing.tWP = min(((int) (timing.tWP * CLOCK_NS) + 1),
			DFC_TIMING_MAX_tWP);
	timing.tRH = min(((int) (timing.tRH * CLOCK_NS) + 1),
			DFC_TIMING_MAX_tRH);
	timing.tRP = min(((int) (timing.tRP * CLOCK_NS) + 1),
			DFC_TIMING_MAX_tRP);

	r0 = (timing.tCH << DFC_TIMING_tCH) |
		(timing.tCS << DFC_TIMING_tCS) |
		(timing.tWH << DFC_TIMING_tWH) |
		(timing.tWP << DFC_TIMING_tWP) |
		(timing.tRH << DFC_TIMING_tRH) |
		(timing.tRP << DFC_TIMING_tRP);
	/* FIXME:hardware issue cause it to be the max value */
	dfc_write(context, DFC_NDTR0CS0, r0);

	timing.tR   = min(((int) (timing.tR   * CLOCK_NS) + 1),
			DFC_TIMING_MAX_tR);
	timing.tWHR = min(((int) (timing.tWHR * CLOCK_NS) + 1),
			DFC_TIMING_MAX_tWHR);
	timing.tAR  = min(((int) (timing.tAR  * CLOCK_NS) + 1),
			DFC_TIMING_MAX_tAR);

	r1 = (timing.tR   << DFC_TIMING_tR)   |
		(timing.tWHR << DFC_TIMING_tWHR) |
		(timing.tAR  << DFC_TIMING_tAR);

	/* FIXME:hardware issue cause it to be the max value */
	dfc_write(context, DFC_NDTR1CS0, r1);
   	return;
}

/******************************************************************************
  dfc_set_dma

  Description:
	Enables or Disables DMA in line with setting in DFC mode of context
	structure. DMA mode of DFC. Performs a read-modify-write operation that
	only changes the driven DMA_EN bit field In DMA mode, all commands and
	data are transferred by DMA.  DMA can be enable/disable on the fly.
  Input Parameters:
	context -Pointer to DFC context structure
  	Output Parameters:
		None
	Returns:
		None
*******************************************************************************/
static void dfc_set_dma(struct dfc_context* context)
{
	uint32_t ndcr;

	ndcr = dfc_read(context, DFC_NDCR);
	if (context->dfc_mode->enable_dma)
		ndcr |= NDCR_DMA_EN;
	else
		ndcr &= ~NDCR_DMA_EN;

	dfc_write(context, DFC_NDCR, ndcr);

	/* Read again to make sure write work */
	ndcr = dfc_read(context, DFC_NDCR);
	return;
}


/******************************************************************************
  dfc_set_ecc

  Description:
	This function enables or disables hardware ECC capability of DFC in line
	with setting in DFC mode of context structure.
  Input Parameters:
	context -Pointer to DFC context structure
	Output Parameters:
		None
	Returns:
		None
*******************************************************************************/
static void dfc_set_ecc(struct dfc_context* context)
{
	uint32_t ndcr;

	ndcr = dfc_read(context, DFC_NDCR);
	if (context->dfc_mode->enable_ecc)
		ndcr |= NDCR_ECC_EN;
	else
		ndcr &= ~NDCR_ECC_EN;

	dfc_write(context, DFC_NDCR, ndcr);

	/* Read again to make sure write work */
	ndcr = dfc_read(context, DFC_NDCR);
	return;
}

/******************************************************************************
  dfc_set_spare

  Description:
	This function enables or disables accesses to spare area of NAND Flash
	through DFC in line with setting in DFC mode of context structure.
  Input Parameters:
	context -Pointer to DFC context structure
  Output Parameters:
	None
  Returns:
	None
*******************************************************************************/
static void dfc_set_spare(struct dfc_context* context)
{
	uint32_t ndcr;

	ndcr = dfc_read(context, DFC_NDCR);
	if (context->dfc_mode->enable_spare)
		ndcr |= NDCR_SPARE_EN;
	else
		ndcr &= ~NDCR_SPARE_EN;

	dfc_write(context, DFC_NDCR, ndcr);

	/* Read again to make sure write work */
	ndcr = dfc_read(context, DFC_NDCR);
	return;
}

static unsigned int get_delta (unsigned int start)
{
    unsigned int stop = OSCR;
    return (stop - start);
}

static int dfc_wait_event(struct dfc_context *context, uint32_t event,
		uint32_t *event_out, uint32_t timeout, int enable_int)
{
	uint32_t ndsr;
	uint32_t to = 3 * timeout;	/* 3 ticks ~ 1us */
	int status;
	int start = OSCR;

	if (enable_int)
		dfc_enable_int(context, event);

	while (1) {
		ndsr = dfc_read(context, DFC_NDSR);
		ndsr &= NDSR_MASK;
		if (ndsr & event) {
			/* event happened */
			*event_out = ndsr & event;
			dfc_clear_int(context, *event_out);
			status = 0;
			break;
		} else if (get_delta(start) > to) {
			status = -ETIME;
			break;
		}
	}

	if (enable_int)
		dfc_disable_int(context, event);
	return status;
}

/******************************************************************************
  dfc_get_pattern

  Description:
	This function is used to retrieve buffer size setting for a transaction
	based on cmd.
  Input Parameters:
	context - Pointer to DFC context structure
	cmd
	  Specifies type of command to be sent to NAND flash .The LSB of this
	  parameter defines the first command code for 2-cycles command. The
	  MSB defines the second command code for 2-cycles command. If MSB is
	  set to zero, this indicates that one cycle command
	Output Parameters:
	data_size
	  It is used to retrieve  length of data transferred to/from DFC,
	  which includes padding bytes
	padding
	  It is used to retrieve how many padding bytes there should be
	  in buffer of data_size.
	Returns:
	0
	  If size setting is returned successfully
	-EINVAL
	  If page size specified in flash spec of context structure is not 512 or
	  2048;If specified command index is not read1/program/erase/reset/readID/
	  readStatus.
*******************************************************************************/
static int dfc_get_pattern(struct dfc_context *context, uint16_t cmd,
			int *data_size, int *padding)
{
	struct dfc_mode* dfc_mode = context->dfc_mode;
	struct dfc_flash_info * flash_info = context->flash_info;
	uint32_t page_size = context->flash_info->page_size; /* 512 or 2048 */

	if (cmd == flash_info->read1 ||
		cmd == flash_info->program) {
		if (512 == page_size) {
			/* add for DMA */
			if (dfc_mode->enable_dma) {
				*data_size = DFC_DATA_SIZE_544;
				if (dfc_mode->enable_ecc)
					*padding = DFC_PADDING_SIZE_24;
				else
					*padding = DFC_PADDING_SIZE_16;
			} else if (!dfc_mode->enable_spare) {
				*data_size = DFC_DATA_SIZE_512;
				*padding = DFC_PADDING_SIZE_0;
			} else {

				if (dfc_mode->enable_ecc)
					*data_size = DFC_DATA_SIZE_520;
				else
					*data_size = DFC_DATA_SIZE_528;

				*padding = DFC_PADDING_SIZE_0;
			}
		} else if (2048 == page_size) {
			/* add for DMA */
			if (dfc_mode->enable_dma) {
				*data_size = DFC_DATA_SIZE_2112;
				if (dfc_mode->enable_ecc)
					*padding = DFC_PADDING_SIZE_24;
				else
					*padding = DFC_PADDING_SIZE_0;
			} else if (!dfc_mode->enable_spare) {
				*data_size = DFC_DATA_SIZE_2048;
				*padding = DFC_PADDING_SIZE_0;
			} else {

				if (dfc_mode->enable_ecc)
					*data_size = DFC_DATA_SIZE_2088;
				else
					*data_size = DFC_DATA_SIZE_2112;

				*padding = DFC_PADDING_SIZE_0;
			}
		} else /* if the page_size is neither 512 or 2048 */
			return -EINVAL;
	} else if (cmd == flash_info->read_id) {
		*data_size = DFC_DATA_SIZE_ID;
		*padding = DFC_PADDING_SIZE_0;
	} else if(cmd == flash_info->read_status) {
		*data_size = DFC_DATA_SIZE_STATUS;
		*padding = DFC_PADDING_SIZE_0;
	} else if (cmd == flash_info->erase || cmd == flash_info->reset) {
		*data_size = DFC_DATA_SIZE_UNUSED;
		*padding = DFC_PADDING_SIZE_UNUSED;
	} else if (cmd == flash_info->unlock1  || cmd == flash_info->unlock2) {
		*data_size = DFC_DATA_SIZE_ID;
		*padding = DFC_PADDING_SIZE_0;
	} else
		return -EINVAL;
	return 0;
}


/******************************************************************************
  dfc_send_cmd

  Description:
	This function configures DFC to send command through DFC to NAND flash
  Input Parameters:
	context
	  Pointer to DFC context structure
	cmd
	  Specifies type of command to be sent to NAND flash .The LSB of this
	  parameter defines the first command code for 2-cycles command. The
	  MSB defines the second command code for 2-cycles command. If MSB is
	  set to zero, this indicates that one cycle command
	addr
	  Address sent out to the flash device withthis command. For page read/
	  program commands , 4-cycles address is sent. For erase command only
	  3-cycles address is sent. If it is equal to 0xFFFFFFFF, the address
	  should not be used.
	num_pages
	  It specifies the number of pages of data to be transferred for
	  a program or read commands. Unused for any other commands than
	  read/program.

  Output Parameters:
	None
  Returns:
	0
	  If size setting is returned successfully
	-EINVAL
	  If specified command index is not read1/program/erase/reset/readID/
	  readStatus.
*******************************************************************************/
static int dfc_send_cmd(struct dfc_context *context, uint16_t cmd,
			uint32_t addr, int num_pages)
{
	struct dfc_flash_info *flash_info = context->flash_info;
	struct dfc_mode *dfc_mode = context->dfc_mode;
	uint8_t  cmd2;
	uint32_t event_out;
	uint32_t ndcb0=0, ndcb1=0, ndcb2=0, ndcr;
	int status;
#ifdef CONFIG_MTD_NAND_PXA3xx_FIX1
	unsigned long flags;
	unsigned int reg;
#endif

	/* It is a must to set ND_RUN firstly, then write command buffer
	 * If conversely,it does not work
	 */
	dfc_write(context, DFC_NDSR, NDSR_MASK);

	/* Set ND_RUN */
	ndcr = dfc_read(context, DFC_NDCR);
	dfc_write(context, DFC_NDCR, (ndcr | NDCR_ND_RUN));

	/* Wait for write command request */
	status = dfc_wait_event(context, NDSR_WRCMDREQ,
		&event_out, NAND_CMD_TIMEOUT, 0);

	if (status) /* Timeout */
		return status;

	cmd2 = (cmd>>8) & 0xFF;
	ndcb0 = cmd | (dfc_mode->chip_select<<24) | ((cmd2?1:0)<<19);

	if (cmd == flash_info->read1) {
		if (0xFFFFFFFF != addr) {
			ndcb0 |= NDCB0_ADDR_CYC(flash_info->read_prog_cycles);
			status = flash_info->addr2ndcb1(cmd, addr, &ndcb1);
			if (status)
				return status;
			ndcb2 = (num_pages - 1) << 8;
		}
		/* If has A27, we need to set addr5 */ 
		if (addr >= 0x8000000)
			ndcb2 |= 0x1;
	} else if (cmd == flash_info->program) {
		ndcb0 |= NDCB0_CMD_TYPE(1) | NDCB0_AUTO_RS;
		ndcb0 |= NDCB0_ADDR_CYC(flash_info->read_prog_cycles);
		status = flash_info->addr2ndcb1(cmd, addr, &ndcb1);
		if (status)
			return status;
		ndcb2 = (num_pages-1) << 8;
		/* If has A27, we need to set addr5 */ 
		if (addr >= 0x8000000)
			ndcb2 |= 0x1;
	} else if (cmd == flash_info->erase) {
		ndcb0 |= NDCB0_CMD_TYPE(2) | NDCB0_AUTO_RS;
		ndcb0 |= NDCB0_ADDR_CYC(3);
		status = flash_info->addr2ndcb1(cmd, addr, &ndcb1);
		if (status)
			return status;
	} else if (cmd == flash_info->read_id) {
		ndcb0 |= (NDCB0_ADDR_CYC(1) | NDCB0_CMD_TYPE(3));
	} else if(cmd == flash_info->read_status) {
		ndcb0 |= (NDCB0_ADDR_CYC(1) | NDCB0_CMD_TYPE(4));
	} else if(cmd == flash_info->reset) {
		ndcb0 |= NDCB0_CMD_TYPE(5);
	} else if (cmd == flash_info->lock) {
		ndcb0 |= NDCB0_CMD_TYPE(5);
#ifdef CONFIG_MTD_NAND_PXA3xx_UNLOCK
	} else if (cmd == flash_info->unlock1 || cmd == flash_info->unlock2) {
		ndcb0 |= NDCB0_CMD_TYPE(3);
		ndcb0 |= NDCB0_ADDR_CYC(flash_info->unlock_block_cycles);
		status = flash_info->addr2ndcb1(cmd, addr, &ndcb1);
		if (status)
			return status;
#endif
#ifdef CACHE_READ
	} else if(cmd == 0x34) {
		ndcb0 |= NDCB0_CMD_TYPE(4);
#endif
	} else
		return -EINVAL;

#ifdef CONFIG_MTD_NAND_PXA3xx_FIX1
	/* Write to DFC command register */
	local_irq_save(flags);

	/* Data Memory Barrier */
	__asm__ ("mcr p15, 0, %0, c7, c10, 5\n"
			"mrc p15,0, %0, c2, c0, 0\n"
			"mov %0,%0\n"
			"sub pc, pc, #4\n"
			:"=r"(reg));

	/* Drain Write Buffer */
	__asm__ ("mcr p15, 0, %0, c7, c10, 4\n"
			"mrc p15,0, %0, c2, c0, 0\n"
			"mov %0,%0\n"
			"sub pc, pc, #4\n"
			:"=r"(reg));

	clk_disable(context->clk);
#endif

	dfc_write(context, DFC_NDCB0, ndcb0);
	dfc_write(context, DFC_NDCB0, ndcb1);
	dfc_write(context, DFC_NDCB0, ndcb2);

#ifdef CONFIG_MTD_NAND_PXA3xx_FIX1
	clk_enable(context->clk);
	local_irq_restore(flags);
#endif
	return 0;
}

/******************************************************************************
  dfc_stop

  Description:
	This function clears ND_RUN bit of NDCR.
  Input Parameters:
	context--Pointer to DFC context structure
  Output Parameters:
	None
  Returns:
	None
*******************************************************************************/
static void dfc_stop(struct dfc_context *context)
{
	unsigned int ndcr;
	ndcr = dfc_read(context, DFC_NDCR);
	dfc_write(context, DFC_NDCR, (ndcr & ~NDCR_ND_RUN));
	ndcr = dfc_read(context, DFC_NDCR);

	return;
}

static int dfc_setup_cmd_dma(struct dfc_context *context,
		uint16_t cmd, uint32_t addr, int num_pages,
		uint32_t *buf, uint32_t buf_phys,
		uint32_t next_desc_phys, uint32_t dma_int_en,
		struct pxa_dma_desc *dma_desc)
{
	struct dfc_flash_info *flash_info = context->flash_info;
	struct dfc_mode *dfc_mode = context->dfc_mode;
	uint8_t  cmd2;
	uint32_t event_out;
	uint32_t ndcb0=0, ndcb1=0, ndcb2=0, ndcr;
	int status;

	/*
	 * It is a must to set ND_RUN firstly, then write command buffer
	 * If conversely,it does not work
	 */
	dfc_write(context, DFC_NDSR, NDSR_MASK);

	/* Set ND_RUN */
	ndcr = dfc_read(context, DFC_NDCR);
	ndcr |= NDCR_ND_RUN;
	dfc_write(context, DFC_NDCR, ndcr);

	/* Wait for write command request */
	status = dfc_wait_event(context, NDSR_WRCMDREQ,
		&event_out, NAND_CMD_TIMEOUT, 0);

	if (status)
		return status; /* Timeout */

	cmd2 = (cmd>>8) & 0xFF;
	ndcb0 = cmd | (dfc_mode->chip_select<<24) | ((cmd2?1:0)<<19);

	if (cmd == flash_info->read1) {
		if (0xFFFFFFFF != addr) {
			ndcb0 |= NDCB0_ADDR_CYC(flash_info->read_prog_cycles);
			status = flash_info->addr2ndcb1(cmd, addr, &ndcb1);
			if (status)
				return status;
			ndcb2 = (num_pages-1) << 8;
		}
		/* If flash size if larger than 0x8000000 bytes,
		 * we need set addr5 according to addr.
		 */
		ndcb2 |= (addr >> 27);
	} else if (cmd == flash_info->program) {
		ndcb0 |= NDCB0_CMD_TYPE(1) | NDCB0_AUTO_RS;
		ndcb0 |= NDCB0_ADDR_CYC(flash_info->read_prog_cycles);

		status = flash_info->addr2ndcb1(cmd, addr, &ndcb1);
		if (status)
			return status;
		ndcb2 = (num_pages-1) << 8;

		/* If flash size if larger than 0x8000000 bytes,
		 * we need set addr5 according to addr.
		 */
		ndcb2 |= (addr >> 27);
	} else if (cmd == flash_info->erase) {
		ndcb0 |= NDCB0_CMD_TYPE(2) | NDCB0_AUTO_RS;
		ndcb0 |= NDCB0_ADDR_CYC(3);

		status = flash_info->addr2ndcb1(cmd, addr, &ndcb1);
		if (status)
			return status;
	} else if (cmd == flash_info->read_id) {
		ndcb0 |= (NDCB0_ADDR_CYC(1) | NDCB0_CMD_TYPE(3));
	} else if (cmd == flash_info->read_status) {
		ndcb0 |= (NDCB0_ADDR_CYC(1) | NDCB0_CMD_TYPE(4));
	} else if (cmd == flash_info->reset) {
		ndcb0 |= NDCB0_CMD_TYPE(5);
	} else if (cmd == flash_info->lock) {
		ndcb0 |= NDCB0_CMD_TYPE(5);
#ifdef CONFIG_MTD_NAND_PXA3xx_UNLOCK
	} else if (cmd == flash_info->unlock1 || cmd == flash_info->unlock2) {
		ndcb0 |= NDCB0_CMD_TYPE(3);
		ndcb0 |= NDCB0_ADDR_CYC(flash_info->unlock_block_cycles);
		status = flash_info->addr2ndcb1(cmd, addr, &ndcb1);
		if (status)
			return status;
#endif
#ifdef CACHE_READ
	} else if (cmd == 0x34) {
		ndcb0 |= NDCB0_CMD_TYPE(4);
#endif
	} else
		return -EINVAL;

	*((uint32_t *)buf) = ndcb0;
	*((uint32_t *)buf + 1) = ndcb1;
	*((uint32_t *)buf + 2) = ndcb2;

	dma_int_en &= (DCMD_STARTIRQEN | DCMD_ENDIRQEN);

	dma_desc->ddadr = next_desc_phys;
	dma_desc->dsadr = buf_phys;
	dma_desc->dtadr = NDCB0_DMA_ADDR;
	dma_desc->dcmd  = DCMD_INCSRCADDR | DCMD_FLOWTRG | dma_int_en |
			  DCMD_WIDTH4 | DCMD_BURST16 | 12;
	return 0;
}

static int dfc_setup_data_dma(struct dfc_context* context,
		uint16_t cmd, uint32_t buf_phys,
		uint32_t next_desc_phys, uint32_t dma_int_en,
		struct pxa_dma_desc* dma_desc)
{
	struct dfc_flash_info * flash_info = context->flash_info;
	int data_size, padding;

	dfc_get_pattern(context, cmd, &data_size, &padding);

	dma_desc->ddadr = next_desc_phys;
	dma_int_en &= (DCMD_STARTIRQEN | DCMD_ENDIRQEN);

	if (cmd == flash_info->program) {

		dma_desc->dsadr = buf_phys;
		dma_desc->dtadr = NDDB_DMA_ADDR;
		dma_desc->dcmd  = DCMD_INCSRCADDR | DCMD_FLOWTRG | dma_int_en |
				  DCMD_WIDTH4 | DCMD_BURST32 | data_size;

	} else if (cmd == flash_info->read1 || cmd == flash_info->read_id ||
		   cmd == flash_info->read_status) {

		dma_desc->dsadr = NDDB_DMA_ADDR;
		dma_desc->dtadr = buf_phys;
		dma_desc->dcmd  = DCMD_INCTRGADDR | DCMD_FLOWSRC | dma_int_en |
				  DCMD_WIDTH4 | DCMD_BURST32 | data_size;
	} else
		return -EINVAL;
	return 0;
}

static void dfc_start_cmd_dma(struct dfc_context* context,
		struct pxa_dma_desc* dma_desc)
{
	DRCMR99 = DRCMR_MAPVLD | context->cmd_dma_ch;	/* NAND CMD DRCMR */
	DDADR(context->cmd_dma_ch) = (uint32_t)dma_desc;
	DCSR(context->cmd_dma_ch) |= DCSR_RUN;
}

static void dfc_start_data_dma(struct dfc_context* context,
		struct pxa_dma_desc* dma_desc)
{
	DRCMR97 = DRCMR_MAPVLD | context->data_dma_ch;
	DDADR(context->data_dma_ch) = (uint32_t)dma_desc;
	DCSR(context->data_dma_ch) |= DCSR_RUN;
}

/******************************************************************************
  dfc_read_fifo_partial

  Description:
	This function reads data from data buffer of DFC.Bytes can be any less than
	or equal to data_size, the left is ignored by ReadFIFO though they will be
	read from NDDB to clear data buffer.
  Input Parameters:
	context
	  Pointer to DFC context structure
	nbytes
	  Indicating how much data should be read into buffer.
	data_size
	  Specifing length of data transferred to/from DFC, which includes
	  padding bytes
  Output Parameters:
	pBuffer
	  Pointer to the data buffer where data should be placed.
	Returns:
	  None
*******************************************************************************/
static void dfc_read_fifo_partial(struct dfc_context *context,
		uint8_t *buffer, int nbytes, int data_size)
{
	uint32_t data = 0;
	uint32_t i = 0;
	uint32_t bytes_multi;
	uint32_t bytes_remain;
	uint8_t *buf = buffer;


	if (1 == data_size) {
		data = dfc_read(context, DFC_NDDB) & 0xFF;
		*buf++ = (uint8_t)data;
	} else if (2 == data_size) {
		data = dfc_read(context, DFC_NDDB) & 0xFFFF;
		*buf++ = data & 0xFF;
		*buf++ = (data >> 8) & 0xFF;
	} else {
		bytes_multi = (nbytes & 0xFFFFFFFC);
		bytes_remain = nbytes & 0x03;

		i = 0;
		/* Read the bytes_multi*4 bytes data */
		while (i < bytes_multi) {
			data = dfc_read(context, DFC_NDDB);
			/* FIXME: we don't know whether the buffer
			 * align to 4 bytes or not. Cast the buffer
			 * to int is not safe here. Especially under
			 * gcc 4.x. Use memcpy here. But the memcpy
			 * maybe not correct on BE architecture.
			 * --by Yin, Fengwei
			 */
			memcpy(buf, &data, sizeof(data));
			i += sizeof(data);
			buf += sizeof(data);
		}

		/* Read the left bytes_remain bytes data */
		if (bytes_remain) {
			data = dfc_read(context, DFC_NDDB);
			for (i = 0; i < bytes_remain; i++)
				*buf++ = (uint8_t)((data >> (8*i)) & 0xFF);
		}

		/* When read the remain bytes, we always read 4 bytes data
		 * to DFC. So the data_size should subtract following number.
		 */
		data_size -= bytes_multi + (bytes_remain ? sizeof(data) : 0);

		/* We need Read data_size bytes data totally */
		while (data_size > 0) {
			data = dfc_read(context, DFC_NDDB);
			data_size -= sizeof(data);
		}
	}
	return;
}

/******************************************************************************
  dfc_write_fifo_partial

  Description:
	Write to data buffer of DFC from a buffer. Bytes can be same as
	data_size, also can be data_size-padding, but can¡¯t be random value,
	the left will be automatically padded by WriteFIFO.
  Input Parameters:
	context
	  Pointer to DFC context structure
	bytes
	  Indicating how much data should be read into buffer.
	data_size
	  Specifing length of data transferred to/from DFC, which includes
	  padding bytes
	buffer
	  Pointer to the data buffer where data will be taken from to be written
	  to DFC data buffer
  Output Parameters:
	None
  Returns:
	None
*******************************************************************************/
static void dfc_write_fifo_partial(struct dfc_context *context,
		uint8_t *buffer, int nbytes, int data_size)
{
	uint32_t i = 0;

	uint32_t bytes_multi = (nbytes & 0xFFFFFFFC);
	uint32_t bytes_remain = nbytes & 0x03;
	uint32_t temp;
	uint8_t *buf = buffer;

	/*
	 * caller guarantee buffer contains appropriate data thereby
	 * it is impossible for nbytes not to be a multiple of 4 byte
	 */

	/* Write the bytes_multi*4 bytes data */
	while (i < bytes_multi) {
		temp = buf[0] | buf[1] << 8 |
				buf[2] << 16 | buf[3] << 24;
		dfc_write(context, DFC_NDDB, temp);
		buf += 4;
		i += 4;
	}

	/* Write the left bytes_remain bytes data */
	if (bytes_remain) {
		temp = 0xFFFFFFFF;
		for (i = 0; i < bytes_remain; i++)
			temp &= *buf++ << i*8;

		dfc_write(context, DFC_NDDB, temp);
	}

	/* When write the remain bytes, we always write 4 bytes data
	 * to DFC. So the data_size should subtract following number.
	 */
	data_size -= bytes_multi + (bytes_remain ? sizeof(temp) : 0);

	while (data_size > 0) {
		dfc_write(context, DFC_NDDB, 0xFFFFFFFF);
		data_size -= 4;
	}

	return;
}

/******************************************************************************
  dfc_read_fifo
  Description:
	This function reads data from data buffer of DFC.Bytes can be any less
	than or equal to data_size, the left is ignored by ReadFIFO though they
	will be read from NDDB to clear data buffer.
  Input Parameters:
	context
	  Pointer to DFC context structure
	nbytes
	  Indicating how much data should be read into buffer.
	data_size
	  Specifing length of data transferred to/from DFC, which includes
	  padding bytes
  Output Parameters:
	buffer
	  Pointer to the data buffer where data should be placed.
  Returns:
	None
*******************************************************************************/

static void dfc_read_fifo(struct dfc_context *context,
		uint8_t *buffer, int nbytes)
{
	uint32_t i = 0;

	uint32_t bytes_multi = (nbytes & 0xFFFFFFFC);
	uint32_t bytes_remain = nbytes & 0x03;
	uint32_t temp;
	uint8_t *buf = buffer;

	/* Read the bytes_multi*4 bytes data */
	while (i < bytes_multi) {
		temp = dfc_read(context, DFC_NDDB);
		/* FIXME: we don't know whether the buffer
		 * align to 4 bytes or not. Cast the buffer
		 * to int is not safe here. Especially under
		 * gcc 4.x. Use memcpy here. But the memcpy
		 * maybe not correct on BE architecture.
		 * --by Yin, Fengwei
		 */
		memcpy(buf, &temp, sizeof(temp));
		i += sizeof(temp);
		buf += sizeof(temp);
	}

	/* Read the left bytes_remain bytes data */
	temp = dfc_read(context, DFC_NDDB);
	for (i = 0; i < bytes_remain; i++) {
		*buffer++ = (uint8_t)((temp >> (8*i)) & 0xFF);
	}

	return;
}

/******************************************************************************
  dfc_write_fifo
  Description:
	Write to data buffer of DFC from a buffer.Bytes can be same as data_size,
	also can be data_size-padding, but can¡¯t be random value, the left will
	be automatically padded by WriteFIFO.
  Input Parameters:
	context
	  Pointer to DFC context structure
	nbytes
	  Indicating how much data should be read into buffer.
	data_size
	  Specifing length of data transferred to/from DFC, which includes
	  padding bytes
	buffer
	  Pointer to the data buffer where data will be taken from to be written to
	  DFC data buffer
  Output Parameters:
	None
  Returns:
	None
*******************************************************************************/
static void dfc_write_fifo(struct dfc_context *context,
		uint8_t *buffer, int nbytes)
{
	uint32_t bytes_multi = (nbytes & 0xFFFFFFFC);
	uint32_t bytes_remain = nbytes & 0x03;
	uint32_t i=0;
	uint32_t temp;
	uint8_t *buf = buffer;

	/* Write the bytes_multi*4 bytes data */
	while (i < bytes_multi) {
		temp = buf[0] | buf[1] << 8 |
				buf[2] << 16 | buf[3] << 24;
		dfc_write(context, DFC_NDDB, temp);
		buf += 4;
		i += 4;
	}

	/* Write the left bytes_remain bytes data */
	temp = 0xFFFFFFFF;
	for (i = 0; i < bytes_remain; i++)
		temp &= *buf++ << i*8;
	dfc_write(context, DFC_NDDB, temp);
}

/******************************************************************************
  dfc_read_badblock_addr

  Description:
	This function reads bad block address in units of block starting from 0
	if bad block is detected. It takes into the account if the operation is
	for CS0 or CS1  depending on settings of chip_select parameter of DFC
	Mode structure.
  Input Parameters:
	context
	  Pointer to DFC context structure
  Output Parameters:
	pBadBlockAddr
	  Used to retrieve bad block address back to caller if bad block is
	  detected
  Returns:
	None
  NOTES:
  	We never use this function in driver. Actually, this functions need
	be updated.
*******************************************************************************/
static void dfc_read_badblock_addr(struct dfc_context *context, uint32_t *bbaddr)
{
	uint32_t ndbdr;
	if (0 == context->dfc_mode->chip_select)
		ndbdr = dfc_read(context, DFC_NDBDR0);
	else
		ndbdr = dfc_read(context, DFC_NDBDR1);

	if (512 == context->flash_info->page_size) {
		ndbdr = (ndbdr >> 5) & 0xFFF;
		*bbaddr = ndbdr;
	} else if (2048 == context->flash_info->page_size) {
		/* 16 bits LB */
		ndbdr = (ndbdr >> 8);
		*bbaddr = ndbdr;
	}
	return;
}

/******************************************************************************
  dfc_enable_int

  Description:
	This function is used to enable DFC interrupts.	The bits in int_mask
	will be used to unmask NDCR register to enable corresponding interrupts.
  Input Parameters:
	context
	  Pointer to DFC context structure
	int_mask
	  Specifies what interrupts to enable
  Output Parameters:
	None
  Returns:
	None
*******************************************************************************/
static void dfc_enable_int(struct dfc_context *context, uint32_t int_mask)
{
	uint32_t ndcr;

	ndcr = dfc_read(context, DFC_NDCR);
	ndcr &= ~int_mask;
	dfc_write(context, DFC_NDCR, ndcr);

	ndcr = dfc_read(context, DFC_NDCR);
	return;
}

/******************************************************************************
  dfc_disable_int

  Description:
	This function is used to disable DFC interrupts.
	The bits inint_mask will be used to mask NDCR register to disable
	corresponding interrupts.
  Input Parameters:
	context
	  Pointer to DFC context structure
	int_mask
	  Specifies what interrupts to disable
  Output Parameters:
	None
  Returns:
	None
*******************************************************************************/
static void dfc_disable_int(struct dfc_context *context, uint32_t int_mask)
{
	uint32_t ndcr;

	ndcr = dfc_read(context, DFC_NDCR);
	ndcr |= int_mask;
	dfc_write(context, DFC_NDCR, ndcr);

	ndcr = dfc_read(context, DFC_NDCR);
	return;
}

/******************************************************************************
  dfc_clear_int

  Description:
	This function is used to disable DFC interrupts.
	The bits in int_mask will be used to clear corresponding interrupts
	in NDCR register
  Input Parameters:
	context
	  Pointer to DFC context structure
	int_mask
	  Specifies what interrupts to clear
  Output Parameters:
	None
  Returns:
	None
*******************************************************************************/
static void dfc_clear_int(struct dfc_context *context, uint32_t int_mask)
{
	dfc_write(context, DFC_NDSR, int_mask);

	dfc_read(context, DFC_NDSR);
	return;
}

/*
 * high level primitives
 */

/******************************************************************************
  dfc_init

  Description:
	This function does entire DFC initialization according to the NAND
	flash type currently used with platform, including setting MFP, set
	flash timing, set DFC mode, configuring specified flash parameters
	in DFC, clear ECC logic and page count register.
  Input Parameters:
	context
	  Pointer to DFC context structure
  Output Parameters:
	None
  Returns:
	0
	  if MFPRs are set correctly
	-EINVAL
	  if specified flash is not support by check bytes per page and pages per
	  block
******************************************************************************/

static int dfc_init(struct dfc_context* context, int type)
{
	int status;
	struct dfc_flash_info * flash_info;
	uint32_t ndcr = 0x00000FFF; /* disable all interrupts */

	status = dfc_get_flash_info(type, &flash_info);
	if (status)
		return status;
	context->flash_info = flash_info;

	dfc_set_timing(context, &context->flash_info->timing);

	if (flash_info->enable_arbiter)
		ndcr |= NDCR_ND_ARB_EN;

	if (64 == flash_info->page_per_block)
		ndcr |= NDCR_PG_PER_BLK;
	else if (32 != flash_info->page_per_block)
		return -EINVAL;

	if (flash_info->row_addr_start)
		ndcr |= NDCR_RA_START;

	ndcr |=  (flash_info->read_id_bytes)<<16;

	ndcr |= (flash_info->dfc_mode) << 21;

	if (flash_info->ncsx)
		ndcr |= NDCR_NCSX;

	if (2048 == flash_info->page_size)
		ndcr |= NDCR_PAGE_SZ;
	else if (512 != flash_info->page_size)
		return -EINVAL;

	if (16 == flash_info->flash_width)
		ndcr |= NDCR_DWIDTH_M;
	else if (8 != flash_info->flash_width)
		return -EINVAL;

	if (16 == flash_info->dfc_width)
		ndcr |= NDCR_DWIDTH_C;
	else if (8 != flash_info->dfc_width)
		return -EINVAL;

	dfc_write(context, DFC_NDCR, ndcr);

	dfc_set_dma(context);
	dfc_set_ecc(context);
	dfc_set_spare(context);

	return 0;
}

/******************************************************************************
  dfc_init_no_gpio

  Description:
	This function does entire DFC initialization according to the NAND
	flash type currently used with platform, including set flash timing,
	set DFC mode, configuring specified flash parameters in DFC, clear
	ECC logic and page count register. The only difference with dfc_init
	is that it does not set MFP&GPIO, very useful in OS loader
  Input Parameters:
	context
	  Pointer to DFC context structure
  Output Parameters:
	None
  Returns:
	0
	  if MFPRs are set correctly
	-EINVAL
	  if specified flash is not support by check bytes per page and pages
	  per block
******************************************************************************/
static int dfc_init_no_gpio(struct dfc_context* context, int type)
{
	struct dfc_flash_info * flash_info;
	uint32_t ndcr = 0x00000FFF; /* disable all interrupts */
	int status;

	status = dfc_get_flash_info(type, &flash_info);
	if (status)
		return status;
	context->flash_info = flash_info;

	dfc_set_timing(context, &context->flash_info->timing);

	if (flash_info->enable_arbiter)
		ndcr |= NDCR_ND_ARB_EN;

	if (64 == flash_info->page_per_block)
		ndcr |= NDCR_PG_PER_BLK;
	else if (32 != flash_info->page_per_block)
		return -EINVAL;

	if (flash_info->row_addr_start)
		ndcr |= NDCR_RA_START;

	ndcr |=  (flash_info->read_id_bytes)<<16;

	ndcr |= (flash_info->dfc_mode) << 21;

	if (flash_info->ncsx)
		ndcr |= NDCR_NCSX;

	if (2048 == flash_info->page_size)
		ndcr |= NDCR_PAGE_SZ;
	else if (512 != flash_info->page_size)
		return -EINVAL;

	if (16 == flash_info->flash_width)
		ndcr |= NDCR_DWIDTH_M;
	else if (8 != flash_info->flash_width)
		return -EINVAL;

	if (16 == flash_info->dfc_width)
		ndcr |= NDCR_DWIDTH_C;
	else if (8 != flash_info->dfc_width)
		return -EINVAL;

	dfc_write(context, DFC_NDCR, ndcr);

	dfc_set_dma(context);
	dfc_set_ecc(context);
	dfc_set_spare(context);

	return 0;
}

/*
 * This macro will be used in following NAND operation functions.
 * It is used to clear command buffer to ensure cmd buffer is empty
 * in case of operation is timeout
 */
#define ClearCMDBuf(ctx, timeout) 	do {		\
				dfc_stop(ctx); 		\
				udelay(timeout);	\
			} while (0)

/******************************************************************************
  dfc_reset_flash

  Description:
	It reset the flash. The function can be called at any time when the
	device is in Busy state during random read/program/erase mode and
	reset operation will abort all these operations. After reset operation
	the device is ready to wait for next command
  Input Parameters:
	context
	  Pointer to DFC context structure
  Output Parameters:
	None
  Returns:
	0
	  execution succeeds
	-ETIME
	  if timeout
*******************************************************************************/
static int dfc_reset_flash(struct dfc_context *context)
{
	struct dfc_flash_info *flash_info = context->flash_info;
	uint32_t event, event_out;
	unsigned long timeo;
	int status;

	/* Send command */
	dfc_send_cmd(context, (uint16_t)flash_info->reset, 0xFFFFFFFF, 0);

	event = (context->dfc_mode->chip_select)? \
			NDSR_CS1_CMDD : NDSR_CS0_CMDD;

	if (cpu_is_pxa935()) {
		/* wait RDY asserts or timeout */
		timeo = jiffies + msecs_to_jiffies(100);
		do {
			if (pxa3xx_nand_dev_ready(NULL))
				break;
		} while (time_before(jiffies, timeo));

		/* set NDSR.CSx_CMDD */
		status = dfc_wait_event(context, event, &event_out,
				NAND_OTHER_TIMEOUT, 0);
		if (status) {
			printk(KERN_INFO "CMDD done not set? ndsr %x\n", NDSR);
			ClearCMDBuf(context, NAND_OTHER_TIMEOUT);
			return status;
		}

		return 0;
	}

	/* Wait for CMDDM(command done successfully) */
	status = dfc_wait_event(context, event, &event_out,
		NAND_OTHER_TIMEOUT, 0);

	if (status) {
		ClearCMDBuf(context, NAND_OTHER_TIMEOUT);
		return status;
	}

	/* Wait until flash device is stable or timeout (10ms) */
	timeo = jiffies + msecs_to_jiffies(10);
	do {
		if (pxa3xx_nand_dev_ready(NULL))
			break;
	} while (time_before(jiffies, timeo));

	return 0;
}

static int dfc_readid(struct dfc_context *context, uint32_t *id)
{
	struct dfc_flash_info *flash_info = context->flash_info;
	uint32_t event_out;
	int status;
	char tmp[DFC_DATA_SIZE_ID];

	/* Send command */
	status = dfc_send_cmd(context, (uint16_t)flash_info->read_id,
			0xFFFFFFFF, 0);
	if (status) {
		ClearCMDBuf(context, NAND_OTHER_TIMEOUT);
		return status;
	}

	/* Wait for CMDDM(command done successfully) */
	status = dfc_wait_event(context, NDSR_RDDREQ, &event_out,
		NAND_OTHER_TIMEOUT, 0);
	if (status) {
		ClearCMDBuf(context, NAND_OTHER_TIMEOUT);
		return status;
	}
	dfc_read_fifo_partial(context, (unsigned char *)tmp,
			context->flash_info->read_id_bytes, DFC_DATA_SIZE_ID);

	*id = tmp[0] | (tmp[1] << 8);
	return 0;
}

#define ERR_NONE		0x0
#define ERR_DMABUSERR		(-0x01)
#define ERR_SENDCMD		(-0x02)
#define ERR_DBERR		(-0x03)
#define ERR_BBERR		(-0x04)
#define ERR_BUSY		(-0x05)

#define STATE_CMD_SEND		0x1
#define STATE_CMD_HANDLE	0x2
#define STATE_DMA_TRANSFER	0x3
#define STATE_DMA_DONE		0x4
#define STATE_READY		0x5
#define STATE_SUSPENDED		0x6
#define	STATE_DATA_TRANSFER	0x7

#define MAX_CHIP		1
#define NAND_CMD_DMA_LEN	12

#define MAX_TIM_SIZE	0x1000

struct pxa3xx_nand_info {
	unsigned int		state;
	struct dfc_context	*context;
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
	dma_addr_t 		data_buf_addr;
	char			*data_buf;
	int 			data_dma;
	struct pxa_dma_desc	*data_desc;
	dma_addr_t 		data_desc_addr;
	dma_addr_t 		cmd_buf_addr;
	char 			*cmd_buf;
	int 			cmd_dma;
	struct pxa_dma_desc	*cmd_desc;
	dma_addr_t 		cmd_desc_addr;
	u64 			dma_mask;
#else
	char 			*data_buf;
#endif
	/* relate to the command */
	unsigned int 		cmd;
	unsigned int		cur_cmd;
	unsigned int 		addr;
	unsigned int 		column;
	int 			retcode;
	unsigned int 		buf_count;
	struct completion 	cmd_complete;
#ifdef CONFIG_MTD_NAND_PXA3xx_UNLOCK
	int			locked;
#endif
};

static struct dfc_mode dfc_mode =
{
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
	1,	/* enable DMA */
#else
	0,
#endif
	1,	/* enable ECC */
	1,	/* enable SPARE */
	0,	/* CS0 */
};


struct dfc_context dfc_context =
{
	.dfc_mode	= &dfc_mode,
};


/*
 * MTD structure
 */
static struct mtd_info *monahans_mtd = NULL;

/* MHN_OBM_V2 is related to BBT in MOBM V2
 * MHN_OBM_V3 is related to BBT in MOBM V3
 */
enum {
	MHN_OBM_NULL = 0,
	MHN_OBM_V1,
	MHN_OBM_V2,
	MHN_OBM_V3,
	MHN_OBM_INVAL
} MHN_OBM_TYPE;

static uint8_t scan_ff_pattern[] = { 0xff, 0xff };
static uint8_t scan_main_bbt_pattern[] = { 'p', 'x', 'a', '1' };
static uint8_t scan_mirror_bbt_pattern[] = { '0', 'a', 'x', 'p' };

static struct nand_bbt_descr monahans_bbt_default = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
			| NAND_BBT_2BIT | NAND_BBT_VERSION,
	.maxblocks = 2,
	.len = 2,
	.offs = 0,
	.pattern = scan_ff_pattern,
};

static struct nand_bbt_descr monahans_bbt_main = {
	.options = NAND_BBT_ABSPAGE | NAND_BBT_CREATE | NAND_BBT_WRITE
			| NAND_BBT_2BIT | NAND_BBT_VERSION,
	.veroffs = 6,
	.maxblocks = 2,
        .offs = 2,
        .len = 4,
        .pattern = scan_main_bbt_pattern,
};

static struct nand_bbt_descr monahans_bbt_mirror = {
	.options = NAND_BBT_ABSPAGE | NAND_BBT_CREATE | NAND_BBT_WRITE
			| NAND_BBT_2BIT | NAND_BBT_VERSION,
	.veroffs = 6,
	.maxblocks = 2,
        .offs = 2,
        .len = 4,
        .pattern = scan_mirror_bbt_pattern,
};

#if 0
static struct nand_bbt_descr monahans_bbt_main = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
			| NAND_BBT_2BIT | NAND_BBT_VERSION,
	.veroffs = 2,
	.maxblocks = 2,
        .offs = 		0x0,
        .len = 			2,
        .pattern = 		scan_ff_pattern
};
static struct nand_bbt_descr monahans_bbt_mirror = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
			| NAND_BBT_2BIT | NAND_BBT_VERSION,
	.veroffs = 2,
	.maxblocks = 2,
        .offs = 0x0,
        .len = 2,
        .pattern = scan_ff_pattern
};
#endif

static struct nand_ecclayout monahans_lb_nand_oob = {
	.eccbytes = 24,
	.eccpos = {
		40, 41, 42, 43, 44, 45, 46, 47,
		48, 49, 50, 51, 52, 53, 54, 55,
		56, 57, 58, 59, 60, 61, 62, 63},
	.oobfree = { {2, 38} }
};

/*
 * Monahans OOB size is only 8 bytes, and the rest 8 bytes is controlled by
 * hardware for ECC. We construct virutal ECC buffer. Acutally, ECC is 6 bytes
 * and the remain 2 bytes are reserved.
 */
static struct nand_ecclayout monahans_sb_nand_oob = {
	.eccbytes = 6,
	.eccpos = {8, 9, 10, 11, 12, 13 },
	.oobfree = { {2, 6} }
};


static inline int is_buf_blank(u8 * buf, int size)
{
	int i = 0;
	while(i < size) {
		if (*((unsigned long *)(buf + i)) != 0xFFFFFFFF)
			return 0;
		i += 4;
	}
	if (i > size) {
		i -= 4;
		while( i < size) {
			if(*(buf + i) != 0xFF)
				return 0;
			i++;
		}
	}
	return 1;
}

static void print_buf(char *buf, int num)
{
	int i = 0;

	while (i < num) {
		printk(KERN_ERR "0x%08x: %02x %02x %02x %02x %02x %02x %02x"
		" %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
		(unsigned int) (i),  buf[i], buf[i+1], buf[i+2],
		buf[i+3], buf[i+4], buf[i+5], buf[i+6], buf[i+7],
		buf[i+8], buf[i+9], buf[i+10],buf[i+11], buf[i+12],
		buf[i+13], buf[i+14], buf[i+15]);
		i += 16;
	}
}

#if defined(CONFIG_PXA3xx_DVFM)
static void set_dvfm_constraint(void)
{
	/* Disable D0CS */
	dvfm_disable_op_name("D0CS", dvfm_dev_idx);
	/* Disable Low power mode */
	dvfm_disable_op_name("D1", dvfm_dev_idx);
	dvfm_disable_op_name("D2", dvfm_dev_idx);
	if (cpu_is_pxa935())
		dvfm_disable_op_name("CG", dvfm_dev_idx);
}

static void unset_dvfm_constraint(void)
{
	/* Enable D0CS */
	dvfm_enable_op_name("D0CS", dvfm_dev_idx);
	/* Enable Low power mode */
	dvfm_enable_op_name("D1", dvfm_dev_idx);
	dvfm_enable_op_name("D2", dvfm_dev_idx);
	if (cpu_is_pxa935())
		dvfm_enable_op_name("CG", dvfm_dev_idx);
}

static int nand_notifier_freq(struct notifier_block *nb,
				unsigned long val, void *data)
{
	struct dvfm_freqs *freqs = (struct dvfm_freqs *)data;
	struct op_info *new = NULL;
	struct dvfm_md_opt *md;

	if (freqs)
		new = &freqs->new_info;
	else
		return 0;
	md = (struct dvfm_md_opt *)new->op;
	switch (val) {
	case DVFM_FREQ_PRECHANGE:
		if ((md->power_mode == POWER_MODE_D1) ||
			(md->power_mode == POWER_MODE_D2) ||
			(md->power_mode == POWER_MODE_CG)) {
			;/* do nothing now */
		}
		break;
	case DVFM_FREQ_POSTCHANGE:
		/* It's existing from D1/D2.
		 * And new op_info won't be changed.
		 */
		if ((md->power_mode == POWER_MODE_D1) ||
			(md->power_mode == POWER_MODE_D2) ||
			(md->power_mode == POWER_MODE_CG)) {
			;/* do nothing now */
		}
		break;
	}
	return 0;
}
#else
static void set_dvfm_constraint() {}
static void unset_dvfm_constraint() {}
#endif

static int inline enable_dfc_dma(struct dfc_context *context, int enable)
{
	int ret = dfc_mode.enable_dma;
	unsigned long ndcr;

	if (!enable) {
		ndcr = dfc_read(context, DFC_NDCR);
		ndcr &= ~NDCR_DMA_EN;
		dfc_write(context, DFC_NDCR, ndcr);
		dfc_mode.enable_dma = 0;
	} else {
		ndcr = dfc_read(context, DFC_NDCR);
		ndcr |= NDCR_DMA_EN;
		dfc_write(context, DFC_NDCR, ndcr);
		dfc_mode.enable_dma = 1;
	}
	return ret;
}


static void inline dump_info(struct pxa3xx_nand_info *info)
{
	if (!info)
		return;

	printk(KERN_ERR "cmd:0x%x; addr:0x%x; retcode:%d; state:%d \n",
		info->cur_cmd, info->addr, info->retcode, info->state);
}

static void inline  enable_hw_ecc(struct dfc_context* context, int enable)
{
	unsigned long ndcr;

	clk_enable(context->smc_clk);
	if (!enable) {
		ndcr = dfc_read(context, DFC_NDCR);
		ndcr &= ~NDCR_ECC_EN;
		dfc_write(context, DFC_NDCR, ndcr);
		dfc_mode.enable_ecc = 0;
	} else {
		ndcr = dfc_read(context, DFC_NDCR);
		ndcr |= NDCR_ECC_EN;
		dfc_write(context, DFC_NDCR, ndcr);
		dfc_mode.enable_ecc = 1;
	}
	clk_disable(context->smc_clk);
}

/*
 * Now, we are not sure that the NDSR_RDY mean the flash is ready.
 * Need more test.
 */
static int pxa3xx_nand_dev_ready(struct mtd_info *mtd)
{
	struct pxa3xx_nand_info *info = NULL;
	struct dfc_context *context = NULL;
	int ret;
	if (mtd != NULL) {
		info = (struct pxa3xx_nand_info *)((struct nand_chip *)(mtd->priv))->priv;
		context = info->context;
		clk_enable(context->smc_clk);
		ret = dfc_read(context, DFC_NDSR);
		ret &= NDSR_RDY;
		clk_disable(context->smc_clk);
	} else {
		ret = NDSR & NDSR_RDY;
	}
	return ret;
}

/* each read, we can only read 4bytes from NDDB, we must buffer it */
static u_char pxa3xx_nand_read_byte(struct mtd_info *mtd)
{
	char retval = 0xFF;
	struct pxa3xx_nand_info *info = (struct pxa3xx_nand_info *)
				(((struct nand_chip *)(mtd->priv))->priv);

	if (info->column < info->buf_count) {
		/* Has just send a new command? */
		retval = info->data_buf[info->column++];
	}
	return retval;
}

static u16 pxa3xx_nand_read_word(struct mtd_info *mtd)
{
	u16 retval = 0xFFFF;
	struct pxa3xx_nand_info *info = (struct pxa3xx_nand_info *)
				(((struct nand_chip *)(mtd->priv))->priv);

	if (!(info->column & 0x01) && info->column < info->buf_count) {
		retval = *((u16 *)(info->data_buf+info->column));
		info->column += 2;
	}
	return retval;
}

static void pxa3xx_nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	struct pxa3xx_nand_info *info = (struct pxa3xx_nand_info *)
				(((struct nand_chip *)(mtd->priv))->priv);
	int real_len = min((unsigned int)len, info->buf_count - info->column);

	memcpy(buf, info->data_buf + info->column, real_len);
	info->column += real_len;
}

static void pxa3xx_nand_write_buf(struct mtd_info *mtd,
		const u_char *buf, int len)
{
	struct pxa3xx_nand_info *info = (struct pxa3xx_nand_info *)
				(((struct nand_chip *)(mtd->priv))->priv);
	int real_len = min((unsigned int)len, info->buf_count - info->column);

	memcpy(info->data_buf + info->column, buf, real_len);
	info->column += real_len;
}

static int pxa3xx_nand_verify_buf(struct mtd_info *mtd,
		const u_char *buf, int len)
{
	return 0;
}

#ifdef CONFIG_MTD_NAND_PXA3xx_UNLOCK
static int pxa3xx_nand_try_unlock_flash(struct mtd_info *mtd)
{
	struct nand_chip *this = (struct nand_chip *)mtd->priv;
	struct pxa3xx_nand_info *info = (struct pxa3xx_nand_info *)this->priv;
	struct dfc_context *context = (struct dfc_context *)info->context;
	struct dfc_flash_info *flash_info = (struct dfc_flash_info *)context->flash_info;
	int datasize, paddingsize;
	int status;
	int event, event_out;
	unsigned int low_bound_addr = 0;
	unsigned int high_bound_addr = this->chipsize - 1;
	unsigned int chip_num = high_bound_addr >> this->chip_shift;

	/* check if the NAND supports block_unlocking */
	info->locked = 0;
	if (flash_info->unlock_block_cycles == 0) goto exit;

	info->locked = 1;

	this->select_chip(mtd, chip_num);

	event = (context->dfc_mode->chip_select)? \
			NDSR_CS1_CMDD:NDSR_CS0_CMDD;
	printk(KERN_DEBUG "try to unlocking flash...\n");

	/* try to unlock the NAND, if locked */
	status = dfc_send_cmd(context, (uint16_t)flash_info->unlock1, low_bound_addr, 0);
	if (status) goto exit;

	status = dfc_wait_event(context, NDSR_RDDREQ, &event_out,
		NAND_OTHER_TIMEOUT, 0);
	if (status) goto exit;
	dfc_get_pattern(context, flash_info->read_id,
			&datasize, &paddingsize);

	pr_debug("low bound addr is %x. high is %x\n", low_bound_addr, high_bound_addr);
	dfc_read_fifo_partial(context, info->data_buf,
			flash_info->read_id_bytes, datasize);

	event = (context->dfc_mode->chip_select)? \
			NDSR_CS1_CMDD:NDSR_CS0_CMDD;
	status = dfc_wait_event(context, event, &event_out,
		NAND_OTHER_TIMEOUT, 0);
	if (status) goto exit;

	status = dfc_send_cmd(context, (uint16_t)flash_info->unlock2, high_bound_addr, 0);
	if (status) goto exit;

	status = dfc_wait_event(context, NDSR_RDDREQ, &event_out,
		NAND_OTHER_TIMEOUT, 0);
	if (status) goto exit;

	dfc_get_pattern(context, flash_info->read_id,
			&datasize, &paddingsize);
	dfc_read_fifo_partial(context, info->data_buf,
			flash_info->read_id_bytes, datasize);

	status = dfc_wait_event(context, event, &event_out,
		NAND_OTHER_TIMEOUT, 0);
	if (status) goto exit;

	/* NAND is unlocked now */
	info->locked = 0;

exit:
	if (info->locked)
		printk(KERN_DEBUG "nand is locked now\n");
	else
		printk(KERN_DEBUG "nand is unlocked now\n");

	return info->locked;
}
#endif

#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
#ifndef CONFIG_MTD_NAND_PXA3xx_FIX1
static void pxa3xx_nand_cmd_dma_irq(int channel, void *data)
{
	unsigned int dcsr;
	struct pxa3xx_nand_info *info = (struct pxa3xx_nand_info *)data;
	struct dfc_context* context = info->context;
	struct dfc_mode* dfc_mode = context->dfc_mode;
	unsigned int intm;

	dcsr = DCSR(channel);
	DCSR(channel) = dcsr;

	intm = (dfc_mode->chip_select) ? \
		(NDSR_CS1_BBD | NDSR_CS1_CMDD) : (NDSR_CS0_BBD | NDSR_CS0_CMDD);

	D1(printk("cmd dma interrupt, channel:%d, DCSR:0x%08x\n", \
			channel, dcsr));

	if (dcsr & DCSR_BUSERR) {
		info->retcode = ERR_DMABUSERR;
		complete(&info->cmd_complete);
	} else {
		if ((info->cmd == NAND_CMD_READ0) ||
				(info->cmd == NAND_CMD_READOOB)|| \
				(info->cmd == NAND_CMD_READID) || \
				(info->cmd == NAND_CMD_STATUS)) {
			dfc_enable_int(context, NDSR_RDDREQ | NDSR_DBERR);
		} else if (info->cmd == NAND_CMD_PAGEPROG)
			dfc_enable_int(context, NDSR_WRDREQ);
		else if (info->cmd == NAND_CMD_ERASE1)
			dfc_enable_int(context, intm);
	}

	return;
}
#endif

static void pxa3xx_nand_data_dma_irq(int channel, void *data)
{
	unsigned int dcsr, intm;
	struct pxa3xx_nand_info *info = (struct pxa3xx_nand_info *)data;
	struct dfc_context* context = info->context;
	struct dfc_mode* dfc_mode = context->dfc_mode;

	dcsr = DCSR(channel);
	DCSR(channel) = dcsr;

	intm = (dfc_mode->chip_select) ? \
		(NDSR_CS1_BBD | NDSR_CS1_CMDD) : (NDSR_CS0_BBD | NDSR_CS0_CMDD);

	D1(printk("data dma interrupt, channel:%d, DCSR:0x%08x\n",
			channel, dcsr));
	if (dcsr & DCSR_BUSERR) {
		info->retcode = ERR_DMABUSERR;
		complete(&info->cmd_complete);
	}

	if (info->cmd == NAND_CMD_PAGEPROG) {
		/* DMA interrupt may be interrupted by other IRQs*/
		info->state = STATE_DMA_DONE;
		dfc_enable_int(context, intm);
	} else {
		info->state = STATE_READY;
		complete(&info->cmd_complete);
	}

}
#endif

static irqreturn_t pxa3xx_nand_irq(int irq, void *devid)
{
	unsigned int status, event, intm;
	struct pxa3xx_nand_info *info = (struct pxa3xx_nand_info *)devid;
	struct dfc_context* context = info->context;
	struct dfc_mode* dfc_mode = context->dfc_mode;

	intm =  (dfc_mode->chip_select) ? \
		(NDSR_CS1_BBD | NDSR_CS1_CMDD) : (NDSR_CS0_BBD | NDSR_CS0_CMDD);
	event = (dfc_mode->chip_select) ? \
		(NDSR_CS1_BBD | NDSR_CS1_CMDD) : (NDSR_CS0_BBD | NDSR_CS0_CMDD);

	status = dfc_read(context, DFC_NDSR);
	D1(printk("DFC irq, NDSR:0x%x\n", status));
	if (status & (NDSR_RDDREQ | NDSR_DBERR)) {
		if (status & NDSR_DBERR) {
			info->retcode = ERR_DBERR;
		}

		dfc_disable_int(context, NDSR_RDDREQ | NDSR_DBERR);
		dfc_clear_int(context, NDSR_RDDREQ | NDSR_DBERR);

		if ((info->cmd != NAND_CMD_READID) &&
			(info->cmd != NAND_CMD_STATUS) &&
			(info->cmd != NAND_CMD_READ0) &&
			(info->cmd != NAND_CMD_READOOB)) {
			complete(&info->cmd_complete);
			printk(KERN_ERR "No according command:0x%x happens\n",
					info->cmd);
			goto out;
		}
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
#ifdef CACHE_READ
		if (info->cur_cmd == 0x34) {
			info->state = STATE_DATA_TRANSFER;
			complete(&info->cmd_complete);
			goto out;
		}
#endif
		info->state = STATE_DMA_TRANSFER;
		dfc_start_data_dma(context,
				(struct pxa_dma_desc*)info->data_desc_addr);
#else
		info->state = STATE_DATA_TRANSFER;
		complete(&info->cmd_complete);
#endif
	} else if (status & NDSR_WRDREQ) {
		dfc_disable_int(context, NDSR_WRDREQ);
		dfc_clear_int(context, NDSR_WRDREQ);
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
		info->state = STATE_DMA_TRANSFER;
		dfc_start_data_dma(context,
				(struct pxa_dma_desc*)info->data_desc_addr);
#else
		info->state = STATE_DATA_TRANSFER;
		complete(&info->cmd_complete);
#endif
	} else if (status & event) {
		if (status & NDSR_CS0_BBD) {
			info->retcode = ERR_BBERR;
		}

		dfc_disable_int(context, intm);
		dfc_clear_int(context, event);
		info->state = STATE_READY;
		complete(&info->cmd_complete);
	}
out:
	return IRQ_HANDLED;
}

static int dfc_send_command(struct mtd_info *mtd, unsigned int cmd,
				unsigned int addr, unsigned int num_pages,
				unsigned int event)
{

	struct pxa3xx_nand_info *info = (struct pxa3xx_nand_info *)
			(((struct nand_chip *)(mtd->priv))->priv);
	struct dfc_context* context = info->context;
	int status;
	int ret = 0;

	D1(printk("ready send command, cmd:0x%x, at address:0x%x,"
		" num_pages:%d, wait event:0x%x\n", cmd, addr, num_pages, event));

	clk_enable(context->smc_clk);
	info->state = STATE_CMD_SEND;
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA

	info->cur_cmd = cmd;
#ifdef CONFIG_MTD_NAND_PXA3xx_FIX1
	enable_dfc_dma(context, 0);
	status = dfc_send_cmd(context, cmd, addr, num_pages);
#else
	status = dfc_setup_cmd_dma(context, cmd, addr, num_pages,
			(uint32_t *)info->cmd_buf, info->cmd_buf_addr,
			DDADR_STOP, DCMD_ENDIRQEN, info->cmd_desc);
#endif	/* #ifdef CONFIG_MTD_NAND_PXA3xx_FIX1 */

#else
	status = dfc_send_cmd(context, cmd, addr, num_pages);
#endif	/* #ifdef CONFIG_MTD_NAND_PXA3xx_DMA */
	if (status) {
		info->retcode = ERR_SENDCMD;
		dfc_stop(context);
		udelay(20);
		printk(KERN_ERR "fail send command:0x%02x\n", cmd);
		clk_disable(context->smc_clk);
		return info->retcode;
	}
	info->state = STATE_CMD_HANDLE;
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
	dfc_setup_data_dma(context, cmd, info->data_buf_addr,
			DDADR_STOP, DCMD_ENDIRQEN, info->data_desc);
#ifdef CONFIG_MTD_NAND_PXA3xx_FIX1
	enable_dfc_dma(context, 1);
	dfc_enable_int(context, event);
#else
	dfc_start_cmd_dma(context, (struct pxa_dma_desc*)info->cmd_desc_addr);
#endif	/* #ifdef CONFIG_MTD_NAND_PXA3xx_FIX1 */
#endif	/* #ifdef CONFIG_MTD_NAND_PXA3xx_DMA */

#ifndef CONFIG_MTD_NAND_PXA3xx_DMA
	dfc_enable_int(context, event);
#endif
	ret = wait_for_completion_timeout(&info->cmd_complete, CHIP_DELAY_TIMEOUT);
	if (!ret){
		INIT_COMPLETION(info->cmd_complete);
		printk(KERN_ERR "Command time out\n");
		info->retcode = ERR_SENDCMD;
		dump_info(info);
		ret = info->retcode;	/* timeout */
	}
	D1(printk("command return, cmd:0x%x, retcode:%d\n",
			cmd, info->retcode));
	clk_disable(context->smc_clk);
	return ret;
}

static void pxa3xx_nand_command(struct mtd_info *mtd, unsigned command,
		int column, int page_addr )
{
	struct nand_chip *this = (struct nand_chip *)(mtd->priv);
	struct pxa3xx_nand_info *info =
			(struct pxa3xx_nand_info *)(this->priv);
	struct dfc_context *context = info->context;
	struct dfc_flash_info * flash_info = context->flash_info;
	int ret, pages_shift;
	int status;
#ifndef CONFIG_MTD_NAND_PXA3xx_DMA
	int datasize;
       	int paddingsize;
#endif
	unsigned int to;
	unsigned int tries;

	D1(printk("command:0x%x at address:0x%x, column:0x%x\n",
			command, page_addr, column));

	if (info->state != STATE_READY) {
		printk(KERN_ERR "CHIP is not ready. state: 0x%x\n", info->state);
		dump_info(info);
		info->retcode = ERR_BUSY;
		return;
	}
	info->retcode = ERR_NONE;
	set_dvfm_constraint();
	clk_enable(context->smc_clk);
	pages_shift = this->phys_erase_shift - this->page_shift;
	if (pxa3xx_bbm && pxa3xx_bbm->table_init && pxa3xx_bbm->search) {
		to = (int)(page_addr >> pages_shift);
		to = pxa3xx_bbm->search(mtd, pxa3xx_bbm, to);
		page_addr = (to << pages_shift) |
			(page_addr & ((1 << pages_shift) - 1));
	}

	switch ( command ) {
	case NAND_CMD_READOOB:
		/*
		 * DFC has mark the last 8 bytes OOB data if HARDEARE_ECC is
		 * enabled. We must first disable the HARDWARE_ECC for getting
		 * all the 16 bytes OOB
		 */
		enable_hw_ecc(context, 0);
		info->buf_count = mtd->writesize + mtd->oobsize;
		info->column = mtd->writesize + column;
		info->cmd = command;
		info->addr = page_addr << this->page_shift;
		ret = dfc_send_command(mtd, flash_info->read1, info->addr,
				1, NDSR_RDDREQ | NDSR_DBERR);
		/* Just care SENDCMD error here. The DB error will be handled
		 * specially.
		 */
		if (info->retcode == ERR_SENDCMD)
			break;
#ifndef CONFIG_MTD_NAND_PXA3xx_DMA
		dfc_get_pattern(context, flash_info->read1, &datasize,
				&paddingsize);
		dfc_read_fifo_partial(context, info->data_buf,
				min(info->buf_count, datasize), datasize);
		info->state = STATE_READY;
#endif

#ifdef CACHE_READ
		if ((flash_info->chip_id == 0xba20) || (flash_info->chip_id == 0xb120)){
			ret = dfc_send_command(mtd, 0x34,
					0xFFFFFFFF, 0, NDSR_RDDREQ);
			ClearCMDBuf(context, NAND_OTHER_TIMEOUT);
			info->state = STATE_READY;
		}
#endif
		/* We only are OOB, so if the data has error, does not matter */
		if (info->retcode == ERR_DBERR)
			info->retcode = ERR_NONE;
		enable_hw_ecc(context, 1);
		break;

	case NAND_CMD_READ0:
		tries = 10;
read_again:
		info->retcode = ERR_NONE;
		enable_hw_ecc(context, 1);
		info->column = column;
		info->cmd = command;
		info->buf_count = mtd->writesize + mtd->oobsize;
		memset(info->data_buf, 0xFF, info->buf_count);
		info->addr = page_addr << this->page_shift;

		ret = dfc_send_command(mtd, flash_info->read1, info->addr,
				1, NDSR_RDDREQ | NDSR_DBERR);

		/* Just care SENDCMD error here. The DB error will be handled
		 * specially.
		 */
		if (info->retcode == ERR_SENDCMD){
			tries--;
			if (tries){
				ClearCMDBuf(context, NAND_OTHER_TIMEOUT);
				info->state = STATE_READY;
				goto read_again;
			}
			break;
		}
#ifndef CONFIG_MTD_NAND_PXA3xx_DMA
		dfc_get_pattern(context, flash_info->read1, &datasize,
				&paddingsize);
		dfc_read_fifo_partial(context, info->data_buf,
				min(info->buf_count, datasize), datasize);
		info->state = STATE_READY;
#endif
#ifdef CACHE_READ
		if ((flash_info->chip_id == 0xba20) ||
				(flash_info->chip_id == 0xb120)){
			/* We don't care the return code of cmd 0x34
			 * because we always reset the NAND controller
			 * after 0x34 is sent out.
			 */
			ret = dfc_send_command(mtd, 0x34,
					0xFFFFFFFF, 0, NDSR_RDDREQ);
			ClearCMDBuf(context, NAND_OTHER_TIMEOUT);
			info->state = STATE_READY;
		}
#endif
		/* When the data buf is blank, the DFC will report DB error */
		if (info->retcode == ERR_DBERR && is_buf_blank(info->data_buf,
				mtd->writesize))
			info->retcode = ERR_NONE;

		if (info->retcode == ERR_DBERR) {
			if (tries){
				tries--;
				ClearCMDBuf(context, NAND_OTHER_TIMEOUT);
				info->state = STATE_READY;
				goto read_again;
			}else{
				printk(KERN_ERR "DB error at address 0x%x\n",
			 		info->addr);
		        	print_buf(info->data_buf, info->buf_count);
			}
		}
		break;
	case NAND_CMD_SEQIN:
		/* Write only OOB? */

		info->cmd = command;
		if (column >= mtd->writesize) {
			info->buf_count = mtd->writesize + mtd->oobsize;
			enable_hw_ecc(context, 0);
		} else {
			info->buf_count = mtd->writesize + mtd->oobsize;
			enable_hw_ecc(context, 1);
		}
		memset(info->data_buf, 0xFF, mtd->writesize + mtd->oobsize);
		info->column = column;
		info->addr = page_addr << this->page_shift;
		break;
	case NAND_CMD_PAGEPROG:
		/* prevois command is NAND_CMD_SEIN ?*/
		if (info->cmd != NAND_CMD_SEQIN) {
			info->cmd = command;
			info->retcode = ERR_SENDCMD;
			printk(KERN_ERR "Monahans NAND device: "
				"No NAND_CMD_SEQIN executed before.\n");
			enable_hw_ecc(context, 1);
			break;
		}
		info->cmd = command;
		ret = dfc_send_command(mtd, flash_info->program, info->addr,
				1, NDSR_WRDREQ);

		if (info->retcode == ERR_SENDCMD)
			break;

#ifndef CONFIG_MTD_NAND_PXA3xx_DMA
		dfc_get_pattern(context, flash_info->program, &datasize,
				&paddingsize);
		dfc_write_fifo_partial(context, info->data_buf, datasize,
				datasize);

		if (context->dfc_mode->chip_select)
			dfc_enable_int(info->context,
				NDSR_CS1_BBD | NDSR_CS1_CMDD);
		else
			dfc_enable_int(info->context,
				NDSR_CS0_BBD | NDSR_CS0_CMDD);

		ret = wait_for_completion_timeout(&info->cmd_complete, CHIP_DELAY_TIMEOUT);
		if (!ret){
			INIT_COMPLETION(info->cmd_complete);
			printk(KERN_ERR "Programm Command time out\n");
			dump_info(info);
		}

		if (info->retcode == ERR_BBERR) {
			mtd->block_markbad(mtd, info->addr);
		}
#endif
		break;
	case NAND_CMD_ERASE1:
		info->cmd = command;
		info->addr = (page_addr >> pages_shift) << this->phys_erase_shift;

		if (context->dfc_mode->chip_select)
			ret = dfc_send_command(mtd, flash_info->erase,
				info->addr, 0, NDSR_CS1_BBD | NDSR_CS1_CMDD);
		else
			ret = dfc_send_command(mtd, flash_info->erase,
				info->addr, 0, NDSR_CS0_BBD | NDSR_CS0_CMDD);

		if (info->retcode == ERR_BBERR) {
			mtd->block_markbad(mtd, info->addr);
		}
		break;
	case NAND_CMD_ERASE2:
		break;
	case NAND_CMD_READID:
		info->cmd = command;
		info->buf_count = flash_info->read_id_bytes;
		info->column = 0;
		info->addr = 0xFFFFFFFF;
		ret = dfc_send_command(mtd, flash_info->read_id, info->addr,
				0, NDSR_RDDREQ);

		if (info->retcode == ERR_SENDCMD)
			break;
#ifndef CONFIG_MTD_NAND_PXA3xx_DMA
		dfc_get_pattern(context, flash_info->read_id, &datasize,
				&paddingsize);
		dfc_read_fifo_partial(context, info->data_buf,
				info->buf_count, datasize);
		info->state = STATE_READY;
#endif
		D1(printk("ReadID, [1]:0x%x, [2]:0x%x\n",
			info->data_buf[0], info->data_buf[1]));
		break;
	case NAND_CMD_STATUS:
		info->cmd = command;
		info->buf_count = 1;
		info->column = 0;
		info->addr = 0xFFFFFFFF;
		ret = dfc_send_command(mtd, flash_info->read_status,
			info->addr, 0, NDSR_RDDREQ);

		if (info->retcode == ERR_SENDCMD)
			break;
#ifndef CONFIG_MTD_NAND_PXA3xx_DMA
		dfc_get_pattern(context, flash_info->read_status,
			&datasize, &paddingsize);
		dfc_read_fifo_partial(context, info->data_buf,
			info->buf_count, datasize);
		info->state = STATE_READY;
#endif

#ifdef CONFIG_MTD_NAND_PXA3xx_UNLOCK
                /* nand_base.c: nand_check_wp( ) will
                 * check if WP# is asserted.
                 * If yes, MTD will regard the NAND as protected.
                 * But for some NAND chips, they can be unlocked by
                 * issuing the unlock commands even if WP# is asserted,
                 * so we mark such chips as unprotected here, if they
                 * have been unlocked.
                 */
                if (unlikely(!info->locked && !(*(info->data_buf) & NAND_STATUS_WP))) {
                        *(info->data_buf) |= NAND_STATUS_WP;
                }
#endif
		break;

	case NAND_CMD_RESET:
		status = dfc_reset_flash(&dfc_context);
		if (status) {
			printk(KERN_WARNING "Monahans NAND device:"
				"NAND_CMD_RESET error\n");
		}
#ifdef CONFIG_MTD_NAND_PXA3xx_UNLOCK
		/* Some flash chips would be re-locked after reset,
		 * so we need to check and unlock it if needed
		 */
		if (pxa3xx_nand_try_unlock_flash(mtd))
			printk(KERN_WARNING "NAND is locked!\n");
#endif
		break;
	default:
		printk(KERN_WARNING "Monahans NAND device:"
			"Non-support the command.\n");
		break;
	}

	if (info->retcode != ERR_NONE){
		ClearCMDBuf(context, NAND_OTHER_TIMEOUT);
		info->state = STATE_READY;
	}
	if (info->state == STATE_READY)
		unset_dvfm_constraint();
	clk_disable(context->smc_clk);
}

static void pxa3xx_nand_select_chip(struct mtd_info *mtd, int chip)
{
	struct pxa3xx_nand_info *info = (struct pxa3xx_nand_info *)
			(((struct nand_chip *)(mtd->priv))->priv);

	if (chip <= MAX_CHIP)
		info->context->dfc_mode->chip_select = chip;
	else
		printk(KERN_ERR "Monahans NAND device:"
			"not select the NAND chips!\n");
}

static int pxa3xx_nand_waitfunc(struct mtd_info *mtd, struct nand_chip *this)
{
	struct pxa3xx_nand_info *info = (struct pxa3xx_nand_info *)
			(((struct nand_chip *)(mtd->priv))->priv);

	/* pxa3xx_nand_send_command has waited for command complete */
	if (this->state == FL_WRITING || this->state == FL_ERASING) {
		if (info->retcode == ERR_NONE)
			return 0;
		else {
			/*
			 * any error make it return 0x01 which will tell
			 * the caller the erase and write fail
			 */
			return 0x01;
		}
	}

	return 0;
}

static int pxa3xx_nand_calculate_ecc(struct mtd_info *mtd,
		const u_char *dat, u_char *ecc_code)
{
	return 0;
}

static int pxa3xx_nand_correct_data(struct mtd_info *mtd,
		u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
	struct pxa3xx_nand_info *info = (struct pxa3xx_nand_info *)
			(((struct nand_chip *)(mtd->priv))->priv);

	/*
	 * Any error include ERR_SEND_CMD, ERR_DBERR, ERR_BUSERR, we
	 * consider it as a ecc error which will tell the caller the
	 * read fail We have distinguish all the errors, but the
	 * nand_read_ecc only check this function return value
	 */
	if (info->retcode != ERR_NONE)
		return -1;

	return 0;
}

static void pxa3xx_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	return;
}

/*
 * Check whether the block is a bad one.
 * At first, it will search the relocation table.
 * If necessary, it will search the BBT. Because relocation table can only
 * maintain limited record. If there're more bad blocks, they can't be
 * recorded in relocation table. They can only be recorded in BBT.
 */
static int pxa3xx_nand_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	struct nand_chip *this = NULL;
	int page, block, chipnr, res = 0;
	u16 bad;

	/* At here, we only support one flash chip */
	this = (struct nand_chip *)mtd->priv;

	if (pxa3xx_bbm && pxa3xx_bbm->table_init && pxa3xx_bbm->search) {
		block = (int)(ofs >> this->phys_erase_shift);
		block = pxa3xx_bbm->search(mtd, pxa3xx_bbm, block);
		ofs = ((block << this->phys_erase_shift) |
				(ofs & ((1 << this->phys_erase_shift) - 1)));
	}

	/* search BBT
	 * Maybe the relocation table is full, but some bad blocks aren't
	 * recordered in it.
	 * The below code are copied from nand_block_bad().
	 */
	if (getchip) {
		page = (int)(ofs >> this->page_shift);
		chipnr = (int)(ofs >> this->chip_shift);

		/* Select the NAND chips */
		this->select_chip(mtd, chipnr);
	} else
		page = (int)ofs;

	if (this->options & NAND_BUSWIDTH_16) {
		this->cmdfunc(mtd, NAND_CMD_READOOB, this->badblockpos & 0xFE,
				page & this->pagemask);
		bad = cpu_to_le16(this->read_word(mtd));
		if (this->badblockpos & 0x1)
			bad >>= 1;
		if ((bad & 0xFF) != 0xFF)
			res = 1;
	} else {
		this->cmdfunc(mtd, NAND_CMD_READOOB, this->badblockpos,
				page & this->pagemask);
		if (this->read_byte(mtd) != 0xFF)
			res = 1;
	}

	return res;
}

static int pxa3xx_nand_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *this = NULL;
	struct pxa3xx_nand_info *info = NULL;
	unsigned char buf[2] = {0, 0};
	int block, page, ret;

	this = (struct nand_chip *)mtd->priv;
	info = (struct pxa3xx_nand_info *)(this->priv);
	/* Get block number */
	block = ((int)ofs) >> this->bbt_erase_shift;

	if (pxa3xx_bbm && pxa3xx_bbm->table_init && pxa3xx_bbm->markbad) {
		ret = pxa3xx_bbm->markbad(mtd, pxa3xx_bbm, block);
		if (!ret)
			return 0;
	}

	if (this->bbt)
		this->bbt[block >> 2] |= 0x01 << ((block & 0x03) << 1);

	/* Do we have a flash based bad block table ? */
	if (this->options & NAND_USE_FLASH_BBT)
		return nand_update_bbt(mtd, ofs);

	/* mark the bad block flag at the first two pages */
	page = block << (this->phys_erase_shift - this->page_shift);
	ofs = mtd->writesize + this->badblockpos;
	this->cmdfunc(mtd, NAND_CMD_SEQIN, ofs, page);
	this->write_buf(mtd, buf, 2);
	this->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	page++;
	this->cmdfunc(mtd, NAND_CMD_SEQIN, ofs, page);
	this->write_buf(mtd, buf, 2);
	this->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	return 0;
}

#ifdef CONFIG_MTD_NAND_PXA3xx_DEBUG
static int dump_bbt_flash(struct mtd_info *mtd)
{
	struct nand_chip *this = NULL;
	struct pxa3xx_nand_info *info = NULL;
	int block, page, totlen;

	this = (struct nand_chip *)mtd->priv;
	info = (struct pxa3xx_nand_info *)this->priv;
	block = (mtd->size >> this->phys_erase_shift) - 1;
	totlen = (mtd->size >> this->phys_erase_shift) >> 2;
	printk(KERN_ERR "totlen:0x%x\n", totlen);
	this->select_chip(mtd, 0);
	if (this->bbt_td) {
		printk(KERN_ERR "BBT page:0x%x\n", this->bbt_td->pages[0]);
		page = this->bbt_td->pages[0];
		if (this->bbt_td->pages[0] <= 0) {
			page = block << (this->phys_erase_shift
				- this->page_shift);
		}
		while (totlen > 0) {
			printk(KERN_ERR "page:0x%x\n", page);
			pxa3xx_nand_command(mtd, NAND_CMD_READ0, 0, page);
			printk(KERN_ERR "read result:0x%x\n", info->retcode);
			PRINT_BUF(info->data_buf, BUFLEN);
			totlen -= (1 << this->page_shift);
			page++;
		}
	}
	if (this->bbt_md) {
		printk(KERN_ERR "BBT page:0x%x\n", this->bbt_md->pages[0]);
		page = this->bbt_md->pages[0];
		if (this->bbt_td->pages[0] <= 0) {
			page = block << (this->phys_erase_shift
				- this->page_shift);
			}
		while (totlen > 0) {
			printk(KERN_ERR "page:0x%x\n", page);
			pxa3xx_nand_command(mtd, NAND_CMD_READ0, 0, page);
			printk(KERN_ERR "read result:0x%x\n", info->retcode);
			PRINT_BUF(info->data_buf, BUFLEN);
			totlen -= (1 << this->page_shift);
			page++;
		}

	}
	return 0;
}
#endif

#ifdef CONFIG_MTD_NAND_PXA3xx_DEBUG
static int dump_bbt_mem(struct mtd_info *mtd)
{
	struct nand_chip *this = NULL;

	this = (struct nand_chip *)mtd->priv;
	PRINT_BUF(this->bbt, 225);
	return 0;
}
#endif

static int pxa3xx_nand_scan_bbt(struct mtd_info *mtd)
{
	struct nand_chip *this = (struct nand_chip *)mtd->priv;
	int ret;

	if (pxa3xx_bbm && pxa3xx_bbm->init && !pxa3xx_bbm->table_init) {
		pxa3xx_bbm->flash_type = FLASH_NAND;
		pxa3xx_bbm->page_shift = this->page_shift;
		pxa3xx_bbm->erase_shift = this->phys_erase_shift;
		ret = pxa3xx_bbm->init(mtd, pxa3xx_bbm);
		if (ret) {
			return ret;
		}
	}

	nand_scan_bbt(mtd, &monahans_bbt_default);
#ifdef CONFIG_MTD_NAND_PXA3xx_DEBUG
	dump_bbt_mem(mtd);
#endif
	return 0;
}

static int pxa3xx_nand_probe(struct platform_device *pdev)
{
	struct pxa3xx_nand_platform_data *pdata;
	struct nand_chip *this;
	struct pxa3xx_nand_info *info;
	struct resource *res;
	struct clk *clk = NULL, *smc_clk = NULL;
	int status = -1;
	struct mtd_partition *parts;
	unsigned int data_buf_len;
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
	unsigned int buf_len;
#endif
	int i, ret = 0;
	int block_size, page_size;

	pdata = pdev->dev.platform_data;

	if (pdata == NULL) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -ENODEV;
	}

	/*
	 * Clock gating can only be used when disabling NAND clock workaround
	 * isn't used.
	 */
	/* Enable CKEN_SMC */
	smc_clk = clk_get(NULL, "SMCCLK");
	dfc_context.smc_clk = smc_clk;
	clk_enable(smc_clk);

	/* Enable CKEN_NAND */
	clk = clk_get(NULL, "NANDCLK");
	dfc_context.clk = clk;
	clk_enable(clk);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no IO memory resource defined\n");
		ret = -ENODEV;
		goto out_nand;
	}

	dfc_context.membase = ioremap(res->start, res->end - res->start + 1);
	if (dfc_context.membase == NULL) {
		dev_err(&pdev->dev, "ioremap() failed\n");
		ret = -ENODEV;
		goto out_nand;
	}

	for (i = DFC_FLASH_NULL + 1; i < DFC_FLASH_END; i++)
	{
		uint32_t id;

		status = dfc_init(&dfc_context, i);
		if (status)
			continue;
		status = dfc_readid(&dfc_context, &id);
		if (status)
			continue;
		printk(KERN_DEBUG "id:0x%x, chipid:0x%x\n",
			id, dfc_context.flash_info->chip_id);
		if (id == dfc_context.flash_info->chip_id)
			break;
	}

	if(i == DFC_FLASH_END) {
		printk(KERN_ALERT "Monahans NAND device:"
			"Nand Flash initialize failure!\n");
		ret = -ENXIO;
		goto out_nand;
	}
	flash_config = i;

	monahans_mtd = kzalloc(sizeof(struct mtd_info) + sizeof(struct nand_chip) +
			sizeof(struct pxa3xx_nand_info) , GFP_KERNEL);
	if (!monahans_mtd) {
		printk (KERN_ERR "Monahans NAND device:"
			"Unable to allocate NAND MTD device structure.\n");
		ret = -ENOMEM;
		goto out_nand;
        }

	/* Get pointer to private data */
	this = (struct nand_chip *)((void *)monahans_mtd + sizeof(struct mtd_info));
	info = (struct pxa3xx_nand_info *)((void *)this + sizeof(struct nand_chip));

	monahans_mtd->priv = this;
	this->priv = info;
	data_buf_len = dfc_context.flash_info->page_size +
		dfc_context.flash_info->oob_size;
	info->state = STATE_READY;
	init_completion(&info->cmd_complete);

#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
	info->dma_mask = 0xffffffffUL;

	pdev->dev.dma_mask = &info->dma_mask;
	pdev->dev.coherent_dma_mask = 0xffffffffUL;

#ifndef CONFIG_MTD_NAND_PXA3xx_FIX1
	/* alloc dma data buffer for data
	 * buffer + 2*descriptor + command buffer
	 */
	buf_len = ALIGN(2*sizeof(struct pxa_dma_desc), 32) +
		ALIGN(data_buf_len, 32) + ALIGN(NAND_CMD_DMA_LEN, 32);
#else
	buf_len = ALIGN(sizeof(struct pxa_dma_desc), 32) +
		ALIGN(data_buf_len, 32);
#endif
	printk(KERN_INFO "Try to allocate dma buffer(len:%d)"
		"for data buffer + 2*descriptor + command buffer\n", buf_len);
	info->data_desc = (struct pxa_dma_desc*)dma_alloc_writecombine(&pdev->dev,
			buf_len, &info->data_desc_addr, GFP_KERNEL);
	if (!info->data_desc) {
		printk(KERN_ERR "Monahans NAND device:"
			"Unable to alloc dma buffer\n");
		ret = -ENOMEM;
		goto free_mtd;
	}

#ifndef CONFIG_MTD_NAND_PXA3xx_FIX1
	info->cmd_desc = (struct pxa_dma_desc*)((char *)info->data_desc +
			sizeof(struct pxa_dma_desc));
	info->cmd_desc_addr = (dma_addr_t)((char *)info->data_desc_addr +
			sizeof(struct pxa_dma_desc));
#endif
	info->data_buf = (char *)info->data_desc +
		ALIGN(2*sizeof(struct pxa_dma_desc), 32);
	info->data_buf_addr = (dma_addr_t)((char *)info->data_desc_addr +
		ALIGN(2*sizeof(struct pxa_dma_desc), 32));
#ifndef CONFIG_MTD_NAND_PXA3xx_FIX1
	info->cmd_buf = (char *)info->data_buf + ALIGN(data_buf_len, 32);
	info->cmd_buf_addr = (dma_addr_t)((char *)info->data_buf_addr +
			ALIGN(data_buf_len, 32));
#endif
	D1(printk("Get dma buffer for data dma descriptor, virt:0x%x, phys0x:%x\n",
		(unsigned int)info->data_desc, info->data_desc_addr));
	D1(printk("Get dma buffer for command dma descriptors, virt:0x%x,"
		"phys0x:%x\n", (unsigned int)info->cmd_desc, info->cmd_desc_addr));
	D1(printk("Get dma buffer for data, virt:0x%x, phys0x:%x\n",
		(unsigned int)info->data_buf, info->data_buf_addr));
	D1(printk("Get dma buffer for command, virt:0x%x, phys0x:%x\n",
		(unsigned int)info->cmd_buf, info->cmd_buf_addr));

	D1(printk("Try to allocate dma channel for data\n"));

	info->data_dma = pxa_request_dma("NAND DATA", DMA_PRIO_LOW,
			pxa3xx_nand_data_dma_irq, info);
	if (info->data_dma < 0) {
		printk(KERN_ERR "Monahans NAND device:"
			"Unable to alloc dma channel for data\n");
		ret = info->data_dma;
		goto free_buf;
	}
	D1(printk("Get dma channel:%d for data\n", info->data_dma));

#ifndef CONFIG_MTD_NAND_PXA3xx_FIX1
	D1(printk("Try to allocate dma channel for command\n"));
	info->cmd_dma = pxa_request_dma("NAND CMD", DMA_PRIO_LOW,
			pxa3xx_nand_cmd_dma_irq, info);
	if (info->cmd_dma < 0) {
		printk(KERN_ERR "Monahans NAND device:"
			"Unable to alloc dma channel for command\n");
		ret = info->cmd_dma;
		goto free_data_dma;
	}
	D1(printk("Get dma channel:%d for command\n", info->cmd_dma));

	dfc_context.cmd_dma_ch  = info->cmd_dma;
#endif
	dfc_context.data_dma_ch = info->data_dma;
#else
	printk(KERN_DEBUG "Try to allocate data buffer(len:%d)\n", data_buf_len);
	info->data_buf = kmalloc(data_buf_len, GFP_KERNEL);
	if (!info->data_buf) {
		printk(KERN_ERR "Monahans NAND device:"
			"Unable to alloc data buffer\n");
		ret = -ENOMEM;
		goto free_mtd;
	}
#endif

	D1(printk("Try to request irq:%d\n", IRQ_NAND));
	ret = request_irq(IRQ_NAND, pxa3xx_nand_irq, IRQF_DISABLED, pdev->name, info);
	if (ret < 0) {
		printk(KERN_ERR "Monahans NAND device: Unable to request irq\n");
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
		goto free_cmd_dma;
#else
		goto free_buf;
#endif
	}

	D1(printk("Success request irq\n"));

	/* set address of NAND IO lines */
	this->options = (dfc_context.flash_info->flash_width == 16)? \
			NAND_BUSWIDTH_16: 0 | NAND_USE_FLASH_BBT;

	/* this->IO_ADDR_R = this->IO_ADDR_W = NDDB */
	this->waitfunc = pxa3xx_nand_waitfunc;
	this->select_chip = pxa3xx_nand_select_chip;
	this->dev_ready = pxa3xx_nand_dev_ready;
	this->cmdfunc = pxa3xx_nand_command;
	this->read_word= pxa3xx_nand_read_word;
	this->read_byte = pxa3xx_nand_read_byte;
	this->read_buf = pxa3xx_nand_read_buf;
	this->write_buf = pxa3xx_nand_write_buf;
	this->verify_buf = pxa3xx_nand_verify_buf;

	this->ecc.mode = NAND_ECC_HW;
	this->ecc.hwctl = pxa3xx_nand_enable_hwecc;
	this->ecc.calculate = pxa3xx_nand_calculate_ecc;
	this->ecc.correct = pxa3xx_nand_correct_data;

	this->block_bad = pxa3xx_nand_block_bad;
	this->block_markbad = pxa3xx_nand_block_markbad;
	this->scan_bbt = pxa3xx_nand_scan_bbt;
	this->chip_delay= 25;
	this->bbt_td = &monahans_bbt_main;
	this->bbt_md = &monahans_bbt_mirror;

	/* If the NAND flash is small block flash, only 512-byte pagesize
	 * is supported.
	 * Adjust parameters of BBT what is depended on large block nand
	 * flash or small block nand flash.
	 */
	page_size = dfc_context.flash_info->page_size;
	block_size = dfc_context.flash_info->page_per_block * page_size;
	parts = pdata->parts;
	if (dfc_context.flash_info->oob_size > 16) {
		this->ecc.layout = &monahans_lb_nand_oob;
		this->ecc.size = 2048;
		this->bbt_td->offs = 2;
		this->bbt_td->veroffs = 6;
		this->bbt_td->pages[0] =
			(parts[pdata->nr_parts - 1].offset +
			 block_size + this->bbt_md->maxblocks * block_size) / page_size;
		this->bbt_md->offs = 2;
		this->bbt_md->veroffs = 6;
		this->bbt_md->pages[0] =
			(parts[pdata->nr_parts - 1].offset +
			 block_size) / page_size;

		this->badblockpos = NAND_LARGE_BADBLOCK_POS;
		monahans_bbt_default.offs = NAND_LARGE_BADBLOCK_POS;
		monahans_bbt_default.len = 2;
		/* when scan_bbt() is executed, bbt version can get */
		monahans_bbt_default.veroffs = 2;
	} else {
		this->ecc.layout = &monahans_sb_nand_oob;
		this->ecc.size = 512;
		this->bbt_td->offs = 8;
		this->bbt_td->veroffs = 12;
		this->bbt_td->pages[0] =
			(parts[pdata->nr_parts - 1].offset +
			 block_size + this->bbt_md->maxblocks * block_size) / page_size;
		this->bbt_md->offs = 8;
		this->bbt_md->veroffs = 12;
		this->bbt_md->pages[0] =
			(parts[pdata->nr_parts - 1].offset +
			 block_size) / page_size;

		this->badblockpos = NAND_SMALL_BADBLOCK_POS;
		monahans_bbt_default.offs = NAND_SMALL_BADBLOCK_POS;
		monahans_bbt_default.len = 1;
		monahans_bbt_default.veroffs = 8;
	}

	info->context = &dfc_context;
	/* TODO: allocate dma buffer and channel */

	platform_set_drvdata(pdev, monahans_mtd);

	pxa3xx_bbm = alloc_pxa3xx_bbm();
	if (!pxa3xx_bbm) {
		goto free_irq;
	}

	if (nand_scan(monahans_mtd, 1)) {
		printk(KERN_ERR "Nand scan failed\n");
		ret = -ENXIO;
		goto free_bbm;
	}
	clk_disable(smc_clk);

	ret = add_mtd_partitions(monahans_mtd, pdata->parts, pdata->nr_parts);

#ifdef CONFIG_MTD_NAND_PXA3xx_UNLOCK
	/* try to unlock the flash if needed */
	if (pxa3xx_nand_try_unlock_flash(monahans_mtd))
		printk(KERN_WARNING "NAND is locked!\n");
#endif

	return ret;

free_bbm:
	free_pxa3xx_bbm(pxa3xx_bbm);
free_irq:
	free_irq(IRQ_NAND, info);
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
free_cmd_dma:
#ifndef CONFIG_MTD_NAND_PXA3xx_FIX1
	pxa_free_dma(info->cmd_dma);
#endif
free_data_dma:
	pxa_free_dma(info->data_dma);
free_buf:
	dma_free_writecombine(&pdev->dev, buf_len, info->data_desc, info->data_desc_addr);
#else
free_buf:
	kfree(info->data_buf);
#endif
free_mtd:
	kfree(monahans_mtd);
out_nand:
	clk_disable(clk);
	clk_disable(smc_clk);
	return ret;

}

static int pxa3xx_nand_remove(struct platform_device *pdev)
{
	struct mtd_info *mtd = (struct mtd_info *)platform_get_drvdata(pdev);
	struct pxa3xx_nand_info *info = (struct pxa3xx_nand_info *)
			(((struct nand_chip *)(mtd->priv))->priv);
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
	unsigned int data_buf_len = dfc_context.flash_info->page_size +
			dfc_context.flash_info->oob_size;
	unsigned int buf_len = ALIGN(2*sizeof(struct pxa_dma_desc), 32) +
			ALIGN(data_buf_len, 32) + ALIGN(NAND_CMD_DMA_LEN, 32);
#endif

	platform_set_drvdata(pdev, NULL);

	del_mtd_device(mtd);
	del_mtd_partitions(mtd);

	if (pxa3xx_bbm && pxa3xx_bbm->uninit) {
		pxa3xx_bbm->uninit(mtd, pxa3xx_bbm);
		free_pxa3xx_bbm(pxa3xx_bbm);
	}

	free_irq(IRQ_NAND, info);
#ifdef CONFIG_MTD_NAND_PXA3xx_DMA
#ifndef CONFIG_MTD_NAND_PXA3xx_FIX1
	pxa_free_dma(info->cmd_dma);
#endif
	pxa_free_dma(info->data_dma);
	dma_free_writecombine(&pdev->dev, buf_len, info->data_desc,
		info->data_desc_addr);
#else
	kfree(info->data_buf);
#endif
	kfree(mtd);

	return 0;
}

#ifdef CONFIG_PM
static int pxa3xx_nand_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mtd_info *mtd = (struct mtd_info *)platform_get_drvdata(pdev);
	struct pxa3xx_nand_info *info = (struct pxa3xx_nand_info *)
			(((struct nand_chip *)(mtd->priv))->priv);

	if (info->state != STATE_READY) {
		printk(KERN_ERR "current state is %d\n", info->state);
		return -EAGAIN;
	}
	info->state = STATE_SUSPENDED;
	/*
	 * The PM code need read the mobm from NAND.
	 * So the NAND clock can't be stop here.
	 * The PM code will cover this.
	 */
	/* pxa_set_cken(CKEN_NAND, 0); */
	return 0;
}

static int pxa3xx_nand_resume(struct platform_device *pdev)
{
	struct mtd_info *mtd = (struct mtd_info *)platform_get_drvdata(pdev);
	struct pxa3xx_nand_info *info = (struct pxa3xx_nand_info *)
			(((struct nand_chip *)(mtd->priv))->priv);
	int status;

	if (info->state != STATE_SUSPENDED)
		printk(KERN_WARNING "Error State after resume back\n");

	info->state = STATE_READY;

	clk_enable(dfc_context.smc_clk);
	status = dfc_init(&dfc_context, flash_config);
	clk_disable(dfc_context.smc_clk);
	if (status) {
		printk(KERN_ALERT "Monahans NAND device:"
			"Nand Flash initialize failure!\n");
		return -ENXIO;
	}

#ifdef CONFIG_MTD_NAND_PXA3xx_UNLOCK
	/* try to unlock the flash if needed */
	if (pxa3xx_nand_try_unlock_flash(monahans_mtd))
		printk(KERN_WARNING "NAND is locked!\n");
#endif
	return 0;
}
#endif

static struct platform_driver pxa3xx_nand_driver = {
	.driver = {
		.name	= "pxa3xx-nand",
	},
	.probe		= pxa3xx_nand_probe,
	.remove		= pxa3xx_nand_remove,
#ifdef CONFIG_PM
	.suspend	= pxa3xx_nand_suspend,
	.resume		= pxa3xx_nand_resume,
#endif
};

static void __exit pxa3xx_nand_exit(void)
{
#if defined(CONFIG_PXA3xx_DVFM)
	dvfm_unregister_notifier(&notifier_freq_block,
				DVFM_FREQUENCY_NOTIFIER);
	dvfm_unregister("NAND", &dvfm_dev_idx);
#endif
	platform_driver_unregister(&pxa3xx_nand_driver);
}

static int __init pxa3xx_nand_init(void)
{
#if defined(CONFIG_PXA3xx_DVFM)
	int ret;

	dvfm_register("NAND", &dvfm_dev_idx);
	ret = dvfm_register_notifier(&notifier_freq_block,
				DVFM_FREQUENCY_NOTIFIER);
	if (ret)
		return ret;
#endif
	return platform_driver_register(&pxa3xx_nand_driver);
}

module_init(pxa3xx_nand_init);
module_exit(pxa3xx_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jingqing.xu (jingqing.xu@intel.com)");
MODULE_DESCRIPTION("Glue logic layer for NAND flash on monahans DFC");

