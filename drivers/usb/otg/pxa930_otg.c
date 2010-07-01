
#ifdef CONFIG_CPU_PXA930
#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/workqueue.h>
#include <linux/usb/otg.h>
#include <linux/usb/gadget.h>
#include <linux/irq.h>

#include <asm/irq.h>
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/pxa-regs.h>
#include <mach/pxa3xx-regs.h>
#include <mach/pxa3xx_u2d.h>

#include "pxa3xx_otg.h"
#include "pxa930_otg.h"

#include "../gadget/pxa_comp.h"
#include "../gadget/pxa27x_udc.h"

extern struct pxa27x_udc *get_the_controller(void);

static int xcvr_init = 1;
/*----------------------------------------------------------------
		Special for ULPI
------------------------------------------------------------------*/
static enum u2d_phy_mode xcvr_mode;

static void reset_xcvr_init(void)
{
	xcvr_init = 1;
}

static int ulpi_rtsm(void)
{
	struct pxa27x_udc *dev = get_the_controller();
	u32 u2dotgcr = U2DOTGCR;
	int count = 100000;

	DMSG("%s, U2DOTGCR %x U2DOTGUSR %x\n", __func__, U2DOTGCR, U2DOTGUSR);
	U2DOTGCR = 0;
	U2DOTGCR |= U2DOTGCR_RTSM;

	u2dotgcr = U2DOTGCR;
	U2DOTGCR = u2dotgcr;


	while ((U2DOTGCR & U2DOTGCR_LPA) && count) count--;

	if (count <= 0) {
		printk(KERN_ALERT "%s time out, reset_xcvr!!! USR %x\n",
		       __func__, U2DOTGCR);
		dev->mach->reset_xcvr();
	}

	xcvr_mode = PRE_SYNCH;
	DMSG("%s end, U2DOTGCR %x\n", __func__, U2DOTGCR);

	return 0;
}

static enum u2d_phy_mode ulpi_get_phymode(void)
{
	enum u2d_phy_mode state;

	state = (U2DOTGCR & 0x8000) >> 12;
	/* in case when set UDE it would enter LOWPOWER mode automatically
	 * if no SOFs longer than 3ms */
	if (state == LOWPOWER)
		xcvr_mode = LOWPOWER;

	if ((state != xcvr_mode) && (xcvr_mode != PRE_SYNCH)) {
		printk(KERN_DEBUG "ULPI mode %d not aligned, should be %d\n",
		       state, xcvr_mode);
		xcvr_mode = state;
		if (state)
			ulpi_rtsm();
	}

	return xcvr_mode;
}

static int ulpi_reg_read(u8 reg, u8 *value)
{
	int i = 50000;
	enum u2d_phy_mode state = ulpi_get_phymode();

	if ((state != SYNCH) && (state != PRE_SYNCH)) {
		pr_debug(" not in SYNCH mode!!!");
		return -1;
	}

	U2DOTGUCR = U2DOTGUCR_RUN | U2DOTGUCR_RNW | (reg << U2DOTGUCR_ADDR_S);

	while ((U2DOTGUCR & U2DOTGUCR_RUN) && i--) ;

	if (i <= 0) {
		printk(KERN_DEBUG "Read ULPI register Time out,"
		       " reg %x otgucr %x, usr %x ucr %x\n",
		       reg, U2DOTGUCR, U2DOTGUSR, U2DOTGCR);
		return -1;
	}

	*value = (u8)(U2DOTGUCR & U2DOTGUCR_RDATA);

	DMSG("read ulpi reg %x val %x\n", reg, *value);
	return 0;

}

static int ulpi_reg_write(u8 reg, u8 value)
{
	int i = 50000;
	enum u2d_phy_mode state = ulpi_get_phymode();

	if ((state != SYNCH) && (state != PRE_SYNCH)) {
		pr_debug(": not in SYNCH mode!!!");
		return -1;
	}

	U2DOTGUCR = U2DOTGUCR_RUN | (reg << U2DOTGUCR_ADDR_S) \
		    | (value << U2DOTGUCR_WDATA_S);

	while ((U2DOTGUCR & U2DOTGUCR_RUN) && i--) ;

	if (i <= 0) {
		printk(KERN_DEBUG "Write ULPI register Time out,"
		       " reg %x val %x\n", reg, (int)value);
		return -1;
	}
	DMSG("write ulpi reg %x val %x\n", reg, (int)value);

	return 0;
}

static int ulpi_set_phymode(enum u2d_phy_mode mode)
{
	u32 state;
	u32 u2dotgcr;

	state = ulpi_get_phymode();
	u2dotgcr = U2DOTGCR;

	if ((state == mode) && !xcvr_init) {
		if (mode == SYNCH)
			U2DOTGCR &= ~(U2DOTGCR_UTMID);

		return 0;
	}

	if (xcvr_init == 1)
		xcvr_init = 0;

	if ((state != SYNCH) && (state != PRE_SYNCH))
		ulpi_rtsm();

	switch (mode) {

	case SYNCH:
		/* disable D+/D- pulldown resistor */
		ulpi_reg_write(ULPI_OTG_CONTROL_CLEAR, ULPI_OC_DPPULLDOWN);
		ulpi_reg_write(ULPI_OTG_CONTROL_CLEAR, ULPI_OC_DMPULLDOWN);

		/* Enable ID pullup */
		ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_IDPULLUP);

		/* clear UDE and ULE before enable UTMI */
		U2DOTGCR &= ~(U2DOTGCR_IESI);
		U2DCR &= ~U2DCR_UDE;
		U2DOTGCR &= ~U2DOTGCR_ULE;

		/* enable the UTMI */
		u2dotgcr = U2DOTGCR;
		u2dotgcr &= ~(U2DOTGCR_UTMID);
		U2DOTGCR = u2dotgcr;

		/* enable the ULE again */
		U2DOTGCR |= U2DOTGCR_ULE;

		break;

	case LOWPOWER:
		/* enable D+/D- pulldown resistor */
		ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_DPPULLDOWN);
		ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_DMPULLDOWN);

		/* Enable ID pullup */
		ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_IDPULLUP);

		/* clear SuspendM in UPLI PHY  */
		ulpi_reg_write(ULPI_FUNCTION_CONTROL_CLEAR, ULPI_FC_SUSPENDM);

		break;

	case CARKIT:
	default:
		printk(KERN_ERR "unsupported ULPI operation modes %d\n", mode);
		return -1;
		break;
	}

	xcvr_mode = mode;

	return 0;
}

static void dump_ulpi_regs(void)
{
	u8 val;

	ulpi_reg_read(ULPI_VENDOR_LOW, &val);
	printk(KERN_DEBUG "vendor ID low %02X\n", val);

	ulpi_reg_read(ULPI_VENDOR_HIGH, &val);
	printk(KERN_DEBUG "vendor ID high %02X\n", val);

	ulpi_reg_read(ULPI_PRODUCT_LOW, &val);
	printk(KERN_DEBUG "vendor PRODUCT low %02X\n", val);

	ulpi_reg_read(ULPI_PRODUCT_HIGH, &val);
	printk(KERN_DEBUG "vendor PRODUCT high %02X\n", val);

	ulpi_reg_read(ULPI_FUNCTION_CONTROL, &val);
	printk(KERN_DEBUG "function control %02X\n", val);

	ulpi_reg_read(ULPI_INTERFACE_CONTROL, &val);
	printk(KERN_DEBUG "interface control %02X\n", val);

	ulpi_reg_read(ULPI_OTG_CONTROL, &val);
	printk(KERN_DEBUG "otg control %02X\n", val);

	ulpi_reg_read(ULPI_INT_RISE, &val);
	printk(KERN_DEBUG "interrupt enable rising %02X\n", val);

	ulpi_reg_read(ULPI_INT_FALL, &val);
	printk(KERN_DEBUG "interrupt enable falling %02X\n", val);

	ulpi_reg_read(ULPI_INT_STATUS, &val);
	printk(KERN_DEBUG "interrupt status %02X\n", val);

	ulpi_reg_read(ULPI_INT_LATCH, &val);
	printk(KERN_DEBUG "interrupt latch %02X\n", val);

}

/*----------------------------------------------------------------*/
/******************************************************************
 * Pxa3xx OTG transceiver functions
 ******************************************************************
 */
static int xceiv_mode = USB_OTG_OFF;
static int pxa930_otgx_get_mode(void)
{
	int state = ulpi_get_phymode();

	switch (state) {
	case LOWPOWER:
		xceiv_mode = USB_OTG_LP;
		break;
	case SYNCH:
		xceiv_mode = USB_INT_OTG_CLIENT_DP;
		break;
	case SER_6PIN:
		xceiv_mode = USB_INT_OTG_HOST_DP;
		break;
	case CARKIT:
		xceiv_mode = USB_OTG_CARKIT;
		break;
	case PRE_SYNCH:
		xceiv_mode = USB_OTG_PRE_SYNCH;
		break;
	default:
		break;
	}
	return xceiv_mode;
}

/* Configures USB host port2 to the desired mode */
extern int pxa930_otgx_set_mode(int mode)
{
	int status = 0;

	switch (mode) {

	case USB_OTG_LP:
		ulpi_set_phymode(LOWPOWER);
		break;
	case USB_INT_OTG_CLIENT_DP:
		ulpi_set_phymode(SYNCH);
		break;

	case USB_INT_OTG_HOST_DP:
		ulpi_set_phymode(SER_6PIN);
		break;
	case USB_OTG_CARKIT:
		ulpi_set_phymode(CARKIT);
		break;
	case USB_OTG_PRE_SYNCH:
		ulpi_set_phymode(PRE_SYNCH);
		break;
	default:
		status = OTG_INVALID_PARAMETER;
		break;
	}

	pr_debug("defore transceiver mode %d!\n", xceiv_mode);
	xceiv_mode = mode;
	pr_debug("set transceiver mode %d!\n", xceiv_mode);
	return status;
}

extern void pxa930_otgx_init(void)
{
	ulpi_rtsm();
}

static struct otg_xceiv_ops pxa930_otg_xceiv_ops = {
	.otgx_get_mode                  = pxa930_otgx_get_mode,
	.otgx_set_mode                  = pxa930_otgx_set_mode,
	.otgx_init                      = pxa930_otgx_init,
	.reset_xcvr_init		= reset_xcvr_init,
};

struct otg_xceiv_ops *init_pxa930_otg_xceiv_ops(void)
{
	return &pxa930_otg_xceiv_ops;
}
#endif /* CONFIG_CPU_PXA930 */
