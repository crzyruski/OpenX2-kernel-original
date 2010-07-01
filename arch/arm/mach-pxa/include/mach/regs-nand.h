#ifndef __ASM_ARCH_REGS_NAND_H
#define __ASM_ARCH_REGS_NAND_H

#define NDCR		(0x00) /* Data Flash Control register */
#define NDTR0CS0	(0x04) /* Data Controller Timing Parameter 0 */
#define NDTR1CS0	(0x0C) /* Data Controller Timing Parameter 1 */
#define NDSR		(0x14) /* Data Controller Status Register */
#define NDPCR		(0x18) /* Data Controller Page Count Register */
#define NDBDR0		(0x1C) /* Data Controller Bad Block Register 0 */
#define NDBDR1		(0x20) /* Data Controller Bad Block Register 1 */
#define NDDB		(0x40) /* Data Controller Data Buffer */
#define NDCB0		(0x48) /* Data Controller Command Buffer0 */
#define NDCB1		(0x4C) /* Data Controller Command Buffer1 */
#define NDCB2		(0x50) /* Data Controller Command Buffer2 */


#define NDCR_SPARE_EN		(0x1<<31)
#define NDCR_ECC_EN		(0x1<<30)
#define NDCR_DMA_EN		(0x1<<29)
#define NDCR_ND_RUN		(0x1<<28)
#define NDCR_DWIDTH_C		(0x1<<27)
#define NDCR_DWIDTH_M		(0x1<<26)
#define NDCR_PAGE_SZ		(0x1<<24)
#define NDCR_NCSX		(0x1<<23)
#define NDCR_ND_MODE		(0x3<<21)
#define NDCR_NAND_MODE   	(0x0)
#define NDCR_CLR_PG_CNT		(0x1<<20)
#define NDCR_CLR_ECC		(0x1<<19)
#define NDCR_RD_ID_CNT_MASK	(0x7<<16)
#define	NDCR_RD_ID_CNT(x)	(((x) << 16) & NDCR_RD_ID_CNT_MASK)

#define NDCR_RA_START		(0x1<<15)
#define NDCR_PG_PER_BLK		(0x1<<14)
#define NDCR_ND_ARB_EN		(0x1<<12)

#define NDSR_RDY		(0x1<<11)
#define NDSR_CS0_PAGED		(0x1<<10)
#define NDSR_CS1_PAGED		(0x1<<9)
#define NDSR_CS0_CMDD		(0x1<<8)
#define NDSR_CS1_CMDD		(0x1<<7)
#define NDSR_CS0_BBD		(0x1<<6)
#define NDSR_CS1_BBD		(0x1<<5)
#define NDSR_DBERR		(0x1<<4)
#define NDSR_SBERR		(0x1<<3)
#define NDSR_WRDREQ		(0x1<<2)
#define NDSR_RDDREQ		(0x1<<1)
#define NDSR_WRCMDREQ		(0x1)

#define NDCB0_AUTO_RS		(0x1<<25)
#define NDCB0_CSEL		(0x1<<24)
#define NDCB0_CMD_TYPE_MASK	(0x7<<21)
#define	NDCB0_CMD_TYPE(x)	(((x) << 21) & NDCB0_CMD_TYPE_MASK)
#define NDCB0_NC		(0x1<<20)
#define NDCB0_DBC		(0x1<<19)
#define NDCB0_ADDR_CYC_MASK	(0x7<<16)
#define	NDCB0_ADDR_CYC(x)	(((x) << 16) & NDCB0_ADDR_CYC_MASK)
#define NDCB0_CMD2_MASK		(0xff<<8)
#define NDCB0_CMD1_MASK		(0xff)
#define NDCB0_ADDR_CYC_SHIFT	(16)

#endif /* __ASM_ARCH_REGS_NAND_H */

