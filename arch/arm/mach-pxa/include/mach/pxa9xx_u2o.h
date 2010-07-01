/*
 * linux/arch/arm/mach-pxa/include/mach/pxa9xx_u2o.h
 *
 * This supports machine-specific differences in how the PXA9xx
 * USB 2.0 Device Controller (U2O) is wired.
 *
 * (C) Copyright 2008 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __ASM_ARCH_PXA9XX_U2O_H
#define __ASM_ARCH_PXA9XX_U2O_H

/* registers */
#define U2xUSBCMD				__REG_3(0x55502140)       /* U2O Command */
#define U2xUSBCMD_RST				(1<<1)      /* Reset */
#define U2xUSBCMD_RS				(1<<0)      /* Run/Stop */

#define U2xUSBSTS				__REG_3(0x55502144)       /* U2O Status */
#define U2xUSBINTR				__REG_3(0x55502148)       /* U2O Interrupt Enable */

#define U2xPORTSC				__REG_3(0x55502184)       /* U2O Port Status */
#define U2xPORTSC_PP                            (1<<12)   		  /* Port Power */
#define U2xPORTSC_PTS_MASK                      (3<<30)   		  /* Parallel Transceiver Select */

#define U2xUSBINTR				__REG_3(0x55502148)       /* U2O Interrupt Enable */
#define U2xUSBMODE				__REG_3(0x555021A8)       /* U2O Device Mode */
#define U2xUSBMODE_CM_MASK                      (3<<0)   		  /* U2O Controller Mode */

#define U2xOTGSC				__REG_3(0x555021A4)       /* U2O On-The-Go Status and Control */

/* OTG interrupt enable bit masks */
#define  U2xOTGSC_DPIE                         (0x40000000)   /* Data-line pulsing IE */
#define  U2xOTGSC_1MSIE                        (0x20000000)   /* 1 Millisecond timer IE */
#define  U2xOTGSC_BSEIE                        (0x10000000)   /* B-session end IE */
#define  U2xOTGSC_BSVIE                        (0x08000000)   /* B-session valid IE */
#define  U2xOTGSC_ASVIE                        (0x04000000)   /* A-session valid IE */
#define  U2xOTGSC_AVVIE                        (0x02000000)   /* A-V-bus valid IE */
#define  U2xOTGSC_IDIE                         (0x01000000)   /* OTG ID IE */
#define  U2xOTGSC_IE_MASK   		       (0x7F000000)

/* OTG interrupt status bit masks */
#define  U2xOTGSC_IS_MASK   (0x007F0000)
#define  U2xOTGSC_DPIS                         (0x00400000)   /* Data-line pulsing IS */
#define  U2xOTGSC_1MSIS                        (0x00200000)   /* 1 Millisecond timer IS */
#define  U2xOTGSC_BSEIS                        (0x00100000)   /* B-session end IS */
#define  U2xOTGSC_BSVIS                        (0x00080000)   /* B-session valid IS */
#define  U2xOTGSC_ASVIS                        (0x00040000)   /* A-session valid IS */
#define  U2xOTGSC_AVVIS                        (0x00020000)   /* A-Vbus valid IS */
#define  U2xOTGSC_IDIS                         (0x00010000)   /* OTG ID IS */

/* OTG status bit masks */
#define  U2xOTGSC_DPS                          (0x00004000)   /* Data-line pulsing */
#define  U2xOTGSC_1MST                         (0x00002000)   /* 1 Milliseconf timer toggle */
#define  U2xOTGSC_BSE                          (0x00001000)   /* B-session end */
#define  U2xOTGSC_BSV                          (0x00000800)   /* B-session valid */
#define  U2xOTGSC_ASV                          (0x00000400)   /* A-session valid */
#define  U2xOTGSC_AVV                          (0x00000200)   /* A-Vbus Valid */
#define  U2xOTGSC_ID                           (0x00000100)   /* OTG ID */

/* OTG control bit masks */
#define  U2xOTGSC_CTL_BITS                     (0x2F)
#define  U2xOTGSC_HABA                         (0x00000080)   /* hardware assisted B-Dis to A-Con */
#define  U2xOTGSC_HADP                         (0x00000040)   /* hardware assisted data pulse bits*/
#define  U2xOTGSC_IDPU                         (0x00000020)   /* ID pull-up enable */
#define  U2xOTGSC_DP                           (0x00000010)   /* Data-pulsing */
#define  U2xOTGSC_OT                           (0x00000008)   /* OTG termination */
#define  U2xOTGSC_HAAR                         (0x00000004)   /* Auto reset bit */
#define  U2xOTGSC_VC                           (0x00000002)   /* Vbus charge */
#define  U2xOTGSC_VD                           (0x00000001)   /* Vbus discharge */

/* PHY registers */
#define U2PPLL		__REG_3(0x5550A000)       /* U2O PHY PLL Control */
#define U2PTX		__REG_3(0x5550A004)       /* U2O PHY TX Control */
#define U2PRX		__REG_3(0x5550A008)       /* U2O PHY RX Control */
#define U2IVREF		__REG_3(0x5550A00C)       /* U2O PHY IVREF Control */
#define U2PT0		__REG_3(0x5550A010)       /* U2O PHY Test 0 Control */
#define U2PT1		__REG_3(0x5550A014)       /* U2O PHY Test 1 Control */
#define U2PT2		__REG_3(0x5550A018)       /* U2O PHY Test 2 Control */
#define U2PT3		__REG_3(0x5550A01C)       /* U2O PHY Test 3 Control */
#define U2PT4		__REG_3(0x5550A020)       /* U2O PHY Test 4 Control */
#define U2PT5		__REG_3(0x5550A024)       /* U2O PHY Test 5 Control */
#define U2PID		__REG_3(0x5550A028)       /* U2O PHY ID Register */
#define U2PRS		__REG_3(0x5550A02C)       /* U2O PHY Reserve Register */
#define U2PMN		__REG_3(0x5550A030)       /* U2O PHY Monitor Register */
#define U2OCG		__REG_3(0x5550A108)       /* U2O Clock Gate Register */

#define U2P480		__REG(0x42404078)         /* U2O PHY 480Mhz Control */

/* U2PPLL */
#define U2PPLL_CTRL_REG		        (0x000)
#define U2PPLL_TSTB			(1<<31)
#define U2PPLL_PLLVDD18_SHIFT		(29)
#define U2PPLL_PLLVDD18_MASK		(3<<29)
#define U2PPLL_PLLVDD12_SHIFT		(27)
#define U2PPLL_PLLVDD12_MASK		(3<<27)
#define U2PPLL_PLLCALI12_SHIFT		(25)
#define U2PPLL_PLLCALI12_MASK		(3<<25)
#define U2PPLL_CLK_BLK_EN		(1<<24)
#define U2PPLL_READY			(1<<23)
#define U2PPLL_KVCO_EXT			(1<<22)
#define U2PPLL_VCOCAL_START		(1<<21)
#define U2PPLL_BGP_VSEL_SHIFT		(19)
#define U2PPLL_BGP_VSEL_MASK		(3<<19)
#define U2PPLL_LOCKDET_ISEL		(1<<18)
#define U2PPLL_KVCO_SHIFT		(15)
#define U2PPLL_KVCO_MASK		(7<<15)
#define U2PPLL_ICP_SHIFT		(12)
#define U2PPLL_ICP_MASK			(7<<12)
#define U2PPLL_FBDIV_SHIFT		(4)
#define U2PPLL_FBDIV_MASK		(0xff<<4)
#define U2PPLL_REFDIV_SHIFT		(0)
#define U2PPLL_REFDIV_MASK		(0xf<<4)

/* U2PTX */
#define U2PTX_CTRL_REG                  (0x004)
#define U2PTX_RCAL_START		(1<<12)

/* U2PRX */
#define U2PRX_EARLY_VOS_ON_EN			(1<<31)
#define U2PRX_RXDATA_BLOCK_EN			(1<<30)
#define U2PRX_RXDATA_BLOCK_LENGTH_SHIFT		(28)
#define U2PRX_RXDATA_BLOCK_LENGTH_MASK		(3<<28)
#define U2PRX_EDGE_DET_SEL_SHIFT		(26)
#define U2PRX_EDGE_DET_SEL_MASK			(3<<26)
#define U2PRX_EDGE_DET_EN			(1<<25)
#define U2PRX_S2TO3_DLY_SEL_SHIFT		(23)
#define U2PRX_S2TO3_DLY_SEL_MASK		(3<<23)
#define U2PRX_CDR_COEF_SEL			(1<<22)
#define U2PRX_CDR_FASTLOCK_EN			(1<<21)
#define U2PRX_PHASE_FREEZE_DLY			(1<<20)
#define U2PRX_REG_USQ_LENGTH			(1<<19)
#define U2PRX_REG_ACQ_LENGTH_SHIFT		(17)
#define U2PRX_REG_ACQ_LENGTH_MASK		(3<<17)
#define U2PRX_REG_SQ_LENGTH_SHIFT		(15)
#define U2PRX_REG_SQ_LENGTH_MASK		(3<<15)
#define U2PRX_DLL_SEL_SHIFT			(13)
#define U2PRX_DLL_SEL_MASK			(3<<13)
#define U2PRX_CAP_SEL_SHIFT			(10)
#define U2PRX_CAP_SEL_MASK			(7<<10)
#define U2PRX_DISCON_THRESH_SHIFT		(8)
#define U2PRX_DISCON_THRESH_MASk		(3<<8)
#define U2PRX_SQ_THRESH_SHIFT			(4)
#define U2PRX_SQ_THRESH_MASK			(0xf<<4)
#define U2PRX_LPF_COEF_SHIFT			(2)
#define U2PRX_LPF_COEF_MASK			(3<<2)
#define U2PRX_INTPI_SHIFT			(0)
#define U2PRX_INTPI_MASK			(3<<0)

/* U2IVREF */
#define U2IVREF_SAMPLER_CTRL           		(1<<31)
#define U2IVREF_RXVDD18_SHIFT          		(29)
#define U2IVREF_RXVDD18_MASK           		(3<<29)
#define U2IVREF_SQ_CM_SEL              		(1<<10)
#define U2IVREF_BG_VSEL_SHIFT          		(8)
#define U2IVREF_BG_VSEL_MASK           		(3<<8)
#define U2IVREF_RXVDD12_SHIFT          		(6)
#define U2IVREF_RXVDD12_MASK           		(3<<6)
#define U2IVREF_FSDRV_EN_SHIFT         		(2)
#define U2IVREF_FSDRV_EN_MASK          		(0xf<<2)
#define U2IVREF_REG_IMP_CAL_DLY_SHIFT  		(0)
#define U2IVREF_REG_IMP_CAL_DLY_MASK   		(3<<0)

/* U2PT0 */
#define U2PT0_REG_ARC_DPDM_MODE			(1<<28)

struct pxa9xx_u2o_mach_info {
	int (*is_connected) (void);	/* do we see host? */
	void (*xcvr_init) (void);
#define PXA9XX_U2O_MODE_CLIENT	0
#define PXA9XX_U2O_MODE_HOST	1
};

extern void pxa_set_u2o_info(struct pxa9xx_u2o_mach_info *info);
extern void pxa_set_u2otg_info(struct pxa9xx_u2o_mach_info *info);

extern struct otg_xceiv_ops *init_pxa9xx_otg_xceiv_ops(void);
extern struct otg_ctrl_ops *init_pxa9xx_otg_ctrl_ops(void);

extern int otg_is_client(void);
extern int otg_is_host(void);

#endif /* __ASM_ARCH_PXA9XX_U2O_H */

