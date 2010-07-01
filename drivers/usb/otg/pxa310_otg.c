/* #define DEBUG */
#ifdef CONFIG_CPU_PXA310
#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/workqueue.h>
#include <linux/usb/otg.h>
#include <linux/usb/gadget.h>
#include <linux/irq.h>

#include <asm/irq.h>
#include <asm/mach-types.h>
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/pxa-regs.h>
#include <mach/pxa3xx-regs.h>
#include <mach/pxa3xx_u2d.h>

#ifdef CONFIG_MACH_LITTLETON
#define ENABLE_USB_HEADSET
#endif

#include "pxa3xx_otg.h"
#include "pxa310_otg.h"

#include "../gadget/pxa_comp.h"
#include "../gadget/pxa27x_udc.h"
/* special for headset*/
#ifdef ENABLE_USB_HEADSET
#include <linux/timer.h>
#include <mach/micco.h>

struct timer_list otg_carkit_timer;


static void otg_set_headset_volume(int headset_type);
static void otg_carkit_timer_callback(unsigned long data);

#endif

extern void enable_oscc_pout(void);
extern void disable_oscc_pout(void);

extern void u2d_clk_enable(void);
extern void u2d_clk_restore(void);
extern void u2d_clk_set(int enable);
extern void u2d_irq_set(int en);
extern int get_u2d_bugs(void);
extern struct pxa27x_udc *get_the_controller(void);

extern void set_dvfm_constraint(void);
extern void unset_dvfm_constraint(void);
static int xcvr_init = 1;
static struct pxa_otg *p_pxa_otg;
/*----------------------------------------------------------------
		Special for ULPI
------------------------------------------------------------------*/
static enum u2d_phy_mode xcvr_mode;

int otg_id_pin_level()
{
    return (U2DOTGUSR & U2DOTGUSR_ID);
}

static void reset_xcvr_init(void)
{
	xcvr_init = 1;
}

static void ulpi_dat3_work(void)
{
	struct pxa27x_udc *dev = get_the_controller();
	unsigned long flags;
	u32 u2dotgcr;

	if (!dev->ulpi_dat3_work)
		return;
	local_irq_save(flags);
	/* enable u2d function */
	u2d_clk_set(1);

	dev->mach->ulpi_dat3(0);
	enable_oscc_pout();

	u2dotgcr = U2DOTGCR;
	u2dotgcr |= U2DOTGCR_ULE;
	U2DOTGCR = u2dotgcr;
	u2dotgcr = U2DOTGCR;

	DMSG("%s\n", __func__);
	u2d_irq_set(1);

	dev->ulpi_dat3_work = 0;
	local_irq_restore(flags);
}

static int ulpi_dat3_int_set(int enable)
{
	struct pxa27x_udc *dev = get_the_controller();
	int irq = gpio_to_irq(dev->ulpi_int);
	unsigned long flags;
	u32 u2dotgcr;
	int err;

	local_irq_save(flags);
	if (enable) {
		if (dev->stp_gpio_irq_en) {
			printk(KERN_ERR "re-enterance of %s\n", __func__);
			goto done;
		}

		dev->mach->ulpi_dat3(1);
		err = gpio_request(dev->ulpi_int, "ULPI INT");
		if (err) {
			gpio_free(dev->ulpi_int);
			goto done;
		}
		gpio_direction_input(dev->ulpi_int);

		enable_irq(irq);
		dev->stp_gpio_irq_en = 1;

		gpio_free(dev->ulpi_int);
		u2d_irq_set(0);
		u2dotgcr = U2DOTGCR;
		u2dotgcr &= ~(U2DOTGCR_ULAF | U2DOTGCR_ULE);
		U2DOTGCR = u2dotgcr;
		u2dotgcr = U2DOTGCR;

		u2d_clk_set(0);
	} else {
		if (!dev->stp_gpio_irq_en)
			goto done;
		disable_irq(irq);
		dev->ulpi_dat3_work = 1;
		ulpi_dat3_work();
		dev->stp_gpio_irq_en = 0;
	}
	DMSG("%s %d, orig %d\n", __func__, enable, dev->stp_gpio_irq_en);
	local_irq_restore(flags);
done:
	return 0;
}

static int ulpi_rtsm(void)
{
	struct pxa27x_udc *dev = get_the_controller();
	u32 u2dotgcr = U2DOTGCR;
	int count = 100000;

	DMSG("%s, U2DOTGCR %x U2DOTGUSR %x\n", __func__, U2DOTGCR, U2DOTGUSR);
	u2dotgcr |= U2DOTGCR_UTMID;
	if ((U2DOTGUSR & 0xf0000000)) {

		/* switch to SYNCH mode first */
		u2dotgcr = U2DOTGCR;
		u2dotgcr |= U2DOTGCR_UTMID;
		u2dotgcr &= ~(U2DOTGCR_SMAF | U2DOTGCR_CKAF | U2DOTGCR_ULAF);
		U2DOTGCR = u2dotgcr;

		u2dotgcr = U2DOTGCR;
		u2dotgcr |= U2DOTGCR_RTSM;
		U2DOTGCR = u2dotgcr;

		u2dotgcr = U2DOTGCR;
		u2dotgcr |= U2DOTGCR_ULAF;
		u2dotgcr &= ~(U2DOTGCR_SMAF | U2DOTGCR_CKAF);
		U2DOTGCR = u2dotgcr;
		u2dotgcr = U2DOTGCR;

	}

	while ((U2DOTGUSR & 0xf0000000) && (U2DOTGCR & U2DOTGCR_RTSM) && count)
		count--;

	if (count <= 0) {
		printk(KERN_ALERT "%s time out, reset_xcvr!!! USR %x\n",
		       __func__, U2DOTGUSR);
		dev->mach->reset_xcvr();
	}
	xcvr_mode = PRE_SYNCH;
	DMSG("%s end, U2DOTGCR %x U2DOTGUSR %x\n",
	     __func__, U2DOTGCR, U2DOTGUSR);

	return 0;
}

static enum u2d_phy_mode ulpi_get_phymode(void)
{
	enum u2d_phy_mode state;

	ulpi_dat3_int_set(0);

	state = (U2DOTGUSR & 0xF0000000) >> 28;
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

u8 ulpi_get_dbg_reg(void)
{
	u8 dbg_val;
	ulpi_reg_read(ULPI_DEBUG, &dbg_val);
	return dbg_val;
}

static int ulpi_set_phymode(enum u2d_phy_mode mode)
{
	u32 state;
	u32 u2dotgcr;
	u32 u2dp3cr;

	state = ulpi_get_phymode();
	ulpi_dat3_int_set(0);
	u2dotgcr = U2DOTGCR;
	u2dp3cr = U2DP3CR;

	if ((state == mode) && !xcvr_init) {
		if (mode == SYNCH)
			U2DOTGCR &= ~(U2DOTGCR_UTMID);

		if (mode == LOWPOWER)
			if (get_u2d_bugs() & U2D_FIX_ULPI_STP) {
				/* disable oscc reference clock,
				 * and u2d clock */
				disable_oscc_pout();

				/* enable ULPI_DAT3 gpio interrupt */
				ulpi_dat3_int_set(1);
			}

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
		U2DOTGICR = 0;
		U2DCR &= ~U2DCR_UDE;
		U2DOTGCR &= ~U2DOTGCR_ULE;

		/* enable the UTMI */
		u2dotgcr = U2DOTGCR;
		u2dotgcr &= ~(U2DOTGCR_UTMID | U2DOTGCR_SMAF);
		U2DOTGCR = u2dotgcr;
		u2dotgcr = U2DOTGCR;

		/* enable the ULE again */
		U2DOTGCR |= U2DOTGCR_ULE;
		U2DOTGICR = U2DOTGINT_DEFAULT;

		break;

	case SER_6PIN:
	case SER_3PIN:
		/* enable D+/D- pulldown resistor */
		ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_DPPULLDOWN);
		ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_DMPULLDOWN);

		/* switch to serial mode */
		u2dp3cr &= ~(U2DP3CR_P2SS);
		if (mode == SER_3PIN)
			u2dp3cr |= 0x1 << U2DP3CR_P2SS_S;
		U2DP3CR = u2dp3cr;

		/* set PHY into host mode */
		ulpi_reg_write(ULPI_FUNCTION_CONTROL_SET, 0x45);

		/* set ULPI PHY to serial mode */
		if (mode == SER_3PIN)
			ulpi_reg_write(ULPI_INTERFACE_CONTROL, ULPI_IC_3PIN);
		else
			ulpi_reg_write(ULPI_INTERFACE_CONTROL, ULPI_IC_6PIN);

		/* enable serial mode */
		u2dotgcr |= U2DOTGCR_SMAF;
		u2dotgcr &= ~(U2DOTGCR_ULAF | U2DOTGCR_CKAF);
		U2DOTGCR = u2dotgcr;
		pr_debug("U2DOTGCR %08X, U2DOTGICR %08X,"
			 " U2DP3CR %08x, U2DOTGUSR %08X\n",
			 U2DOTGCR, U2DOTGICR, U2DP3CR, U2DOTGUSR);
		break;

	case LOWPOWER:
		/* enable D+/D- pulldown resistor */
		ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_DPPULLDOWN);
		ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_DMPULLDOWN);

		/* Disable ID pullup */
		ulpi_reg_write(ULPI_OTG_CONTROL_CLEAR, ULPI_OC_IDPULLUP);

		/* clear SuspendM in UPLI PHY  */
		ulpi_reg_write(ULPI_FUNCTION_CONTROL_CLEAR, ULPI_FC_SUSPENDM);


		if (get_u2d_bugs() & U2D_FIX_ULPI_STP) {
			/* disable oscc reference clock, and u2d clock */
			disable_oscc_pout();

			/* enable ULPI_DAT3 gpio interrupt */
			ulpi_dat3_int_set(1);
		}
		break;

	case CARKIT:
#ifdef ENABLE_USB_HEADSET
		if (machine_is_littleton()) {
			/* Enable Carkit mode */

			/* disable D+/D- pulldown resistor */
			ulpi_reg_write(ULPI_OTG_CONTROL_CLEAR,
				ULPI_OC_DPPULLDOWN);
			ulpi_reg_write(ULPI_OTG_CONTROL_CLEAR,
				ULPI_OC_DMPULLDOWN);

			/* Enable ID pullup */
			ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_IDPULLUP);

			/* Configure Carkit Interrupts */
			ulpi_reg_write(ULPI_CARKIT_INT_ENABLE_SET,
				ULPI_CK_IDFLOATRISE);
			ulpi_reg_write(ULPI_CARKIT_INT_ENABLE_SET,
				ULPI_CK_IDFLOATFALL);
			ulpi_reg_write(ULPI_CARKIT_INT_ENABLE_SET,
				ULPI_CK_RIDINTEN);

			/* ULPI Switch to Carkit Mode */

			U2DOTGCR |= U2DOTGCR_UTMID;

			U2DOTGICR = U2DOTGINT_DEFAULT;

			U2DOTGCR |= U2DOTGCR_ULE;

			/* XCVR Carkit mode */
			ulpi_reg_write(ULPI_INTERFACE_CONTROL, ULPI_IC_CARKIT);

			/* Carkit Mode Alternate Function Select */
			u2dotgcr = U2DOTGCR;
			u2dotgcr &= ~(U2DOTGCR_SMAF);
			u2dotgcr &= ~(U2DOTGCR_ULAF);
			u2dotgcr |= U2DOTGCR_CKAF;
			U2DOTGCR = u2dotgcr;
		}
		break;
#endif
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

/******************************************************************
 * headset special
 ******************************************************************
 */
#ifdef ENABLE_USB_HEADSET
/* Attempt to detect USB headset via USB Cable's Resitor ID */
static int ulpi_detect_headset(void)
{
	u32 state = ulpi_get_phymode();
	u8 vendor_rid = 0;
	int ret = 0;
	int i = 0;

	if (xcvr_init)
		return 0;

	if ((state != SYNCH) && (state != PRE_SYNCH))
		ulpi_rtsm();

	/* disable RidConversionDone interrupt and RID Short Cycle */
	ulpi_reg_write(ULPI_VENDOR_RID_CONV, 0);

	/* perform conversion */
	ulpi_reg_write(ULPI_VENDOR_RID_CONV_SET, RID_CONV_START);

	/* Wait until RID Conversion is complete */
	for (i = 0; i < 10000; i++) {
		ulpi_reg_read(ULPI_VENDOR_RID_CONV, &vendor_rid);
		if (vendor_rid & RID_CONV_DONE)
			break;
	}

	pr_debug("USB Headset Vendor RID: 0x%08x\n", vendor_rid);

	vendor_rid &= RID_VALUE_MASK;

	if (vendor_rid == RID_0)
		return ret;

	if (vendor_rid == RID_100K) {
		printk(KERN_NOTICE "Headset detected RID = 100k ohm\n");
		/* May need adjustment based on headset */
		/* bin.yang@marvell.com change it base on the USB headset
		   of Motorola */
		ret = USB_HEADSET_MONO_MIC;
	}

	if (vendor_rid == RID_200K) {
		printk(KERN_NOTICE "Headset detected RID = 200k ohm\n");
		/* May need adjustment based on headset */
		ret = USB_HEADSET_STEREO;
	}

	if (vendor_rid == RID_440K) {
		printk(KERN_NOTICE "Headset detected RID = 440k ohm\n");
		/* May need adjustment based on headset */
		ret = USB_HEADSET_MONO_MIC;
	}

	if (!ret)
		printk(KERN_NOTICE "Headset configuration not available"
		       "for Vendor RID: 0x%08x\n", vendor_rid);

	return ret;
}

/* Configure default volume controls for headset */
static void otg_set_headset_volume(int headset_type)
{
	u32 state = ulpi_get_phymode();

	if ((state != SYNCH) && (state != PRE_SYNCH))
		ulpi_rtsm();

	/* Turn on stereo channels */
	micco_codec_write(MICCO_MUX_STEREO_CH1, 0x7f);
	micco_codec_write(MICCO_MUX_STEREO_CH2, 0x7f);

	switch (headset_type) {
	case USB_HEADSET_STEREO:
		/* Enable Stereo */
		/* Enable carkit regs */
		/* Set ULPI DP/DM Pins to audio channels */
		ulpi_reg_write(ULPI_CARKIT_CONTROL_SET, ULPI_CK_SPKLEFTEN);
		ulpi_reg_write(ULPI_CARKIT_CONTROL_SET, ULPI_CK_SPKRIGHTEN);
		micco_codec_write(MICCO_STEREO_AMPLITUDE_CH1, 0xd7);
		/* Enable onboard MIC */
		micco_codec_write(MICCO_MIC_PGA, 0x2f);
		printk(KERN_NOTICE "Stereo headset enabled\n");
		break;

	case USB_HEADSET_MONO_MIC:
		/* Enable Mono */
		micco_codec_write(MICCO_STEREO_AMPLITUDE_CH1, 0x97);
		/* Set ULPI DP/DM Pins to audio channels */
		ulpi_reg_write(ULPI_CARKIT_CONTROL_SET, ULPI_CK_SPKLEFTEN);
		ulpi_reg_write(ULPI_CARKIT_CONTROL_SET, ULPI_CK_SPKMICEN);
		micco_codec_write(MICCO_STEREO_AMPLITUDE_CH2, 0x3f);
		/* Enable headset MIC */
		micco_codec_write(MICCO_MIC_PGA, 0x5f);
		printk(KERN_NOTICE "Mono/mic headset enabled\n");
		break;

	default:
		printk(KERN_NOTICE "Invalid headset type=%d\n", headset_type);
		break;
	}

	/* enable sidetone */
	micco_codec_write(MICCO_SIDETONE, 0x80);
	return;
}

void otg_reset_d()
{

    ulpi_reg_write(0x4, 0x48);
    ulpi_reg_write(0x39, (1<<4));

    return ;
}

/* Poll ULPI XCVR to determine if OTG device is removed */
static void otg_carkit_timer_callback(unsigned long data)
{
	struct pxa_otg *pOtgHandle = p_pxa_otg;
	u8 tmp_reg = 0;
	u32 otgisr = 0;

	u2d_clk_enable();
	/* Disable OTG interrupts temporarily */
	otgisr = U2DOTGISR;
	U2DOTGISR = 0;
	ulpi_rtsm();
	U2DOTGISR = otgisr;

	/* Check if ID pin is floating */
	ulpi_reg_read(ULPI_CARKIT_INT_STATUS, &tmp_reg);

	if (tmp_reg & ULPI_CK_IDFLOAT) {
		/* Disable headset configuration */
		printk(KERN_NOTICE "USB Headset removed!\n");

		/* Disable speakers/mic on carkit regs */
		ulpi_reg_write(ULPI_CARKIT_CONTROL_CLEAR, ULPI_CK_SPKLEFTEN);
		ulpi_reg_write(ULPI_CARKIT_CONTROL_CLEAR, ULPI_CK_SPKRIGHTEN);
		ulpi_reg_write(ULPI_CARKIT_CONTROL_CLEAR, ULPI_CK_SPKMICEN);


		/* Disable ID pullup */
		ulpi_reg_write(ULPI_OTG_CONTROL_CLEAR, ULPI_OC_IDPULLUP);

		/* Configure Carkit Interrupts */
		ulpi_reg_write(ULPI_CARKIT_INT_ENABLE_CLEAR,
			       ULPI_CK_IDFLOATRISE);
		ulpi_reg_write(ULPI_CARKIT_INT_ENABLE_CLEAR,
			       ULPI_CK_IDFLOATFALL);
		ulpi_reg_write(ULPI_CARKIT_INT_ENABLE_CLEAR, ULPI_CK_RIDINTEN);

		pOtgHandle->pmic_ops->otg_set_vbus(VBUS_LOW);
		pOtgHandle->pmic_ops->otg_set_vbus_ic(OTG_B_DEVICE);

		ulpi_set_phymode(LOWPOWER);
	} else {
		/* Return to carkit mode */
		ulpi_set_phymode(CARKIT);
		otg_carkit_timer.expires = jiffies
			+ msecs_to_jiffies(OTG_CARKIT_POLL_DELAY);
		add_timer(&otg_carkit_timer);
	}
	u2d_clk_restore();

	return;
}
#endif

static int get_headset_type(struct pxa_otg *pOtgHandle)
{
	int headset_type = 0;

#ifdef ENABLE_USB_HEADSET
	if (machine_is_littleton()) {
		headset_type = ulpi_detect_headset();
		if (headset_type > 0) {
			p_pxa_otg = pOtgHandle;

			/* initialize the headset module*/
			pOtgHandle->pmic_ops->otg_set_vbus(VBUS_HIGH);
			otg_set_headset_volume(headset_type);
			otg_carkit_timer.data = headset_type;
			pOtgHandle->otg.default_a = OTG_B_DEVICE;
			pOtgHandle->otg.state = OTG_STATE_B_IDLE;
		}
	}
#endif
	return headset_type;
}

static void headset_init(void)
{
#ifdef ENABLE_USB_HEADSET
	if (machine_is_littleton()) {
		init_timer(&otg_carkit_timer);
		otg_carkit_timer.data = 0;
		otg_carkit_timer.function = otg_carkit_timer_callback;
	}
#endif
}

static void headset_deinit(void)
{
#ifdef ENABLE_USB_HEADSET
	if (machine_is_littleton())
		/* Delete Carkit polling timer */
		del_timer(&otg_carkit_timer);
#endif
}

static void headset_polling(void)
{
#ifdef ENABLE_USB_HEADSET
	if (machine_is_littleton()) {
		/* set carkit mode polling timer */
		otg_carkit_timer.expires = jiffies
			+ msecs_to_jiffies(OTG_CARKIT_POLL_DELAY);
		add_timer(&otg_carkit_timer);
	}
#endif
}


/*----------------------------------------------------------------*/
/******************************************************************
 * Pxa3xx OTG transceiver functions
 ******************************************************************
 */
static int xceiv_mode = USB_OTG_OFF;
static int pxa310_otgx_get_mode(void)
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
static int pxa310_otgx_set_mode(int mode)
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

static void pxa310_otgx_init(void)
{
	ulpi_rtsm();
}

#define USB_OTGID_PIN	MFP_PIN_GPIO106
static enum otg_function pxa310_otgx_detect_default_func(void)
{
	struct pxa27x_udc *dev = get_the_controller();
	u8 tmp;

	/* enable OTG ID pullup register */
	ulpi_dat3_int_set(0);
	ulpi_rtsm();
	if (ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_IDPULLUP))
		goto err;
	if (ulpi_reg_read(ULPI_INT_STATUS, &tmp))
		goto err;

	/* enable ID detect */
	if (tmp & ULPI_INT_IDGND)
		return OTG_B_DEVICE;
	else
		return OTG_A_DEVICE;

err:
	dev->mach->reset_xcvr();
	return OTG_B_DEVICE;
}

/* Called to start data-line SRP
 */
static int pxa310_otgx_dp_session(void)
{
	char func_ctrl, otg_ctrl;
	
#if defined(CONFIG_PXA3xx_DVFM)
	set_dvfm_constraint();
#endif
	ulpi_dat3_int_set(0);
	ulpi_rtsm();
	ulpi_reg_read(ULPI_FUNCTION_CONTROL, &func_ctrl);
	ulpi_reg_read(ULPI_OTG_CONTROL, &otg_ctrl);

	ulpi_reg_write(ULPI_OTG_CONTROL_CLEAR, ULPI_OC_DPPULLDOWN);

	/* Enable D+ pull-up resister to enable Data line SRP */
	ulpi_reg_write(ULPI_FUNCTION_CONTROL, 0x4 | ULPI_FC_SUSPENDM);
	ulpi_dat3_int_set(0);

	mdelay(T_B_DATA_PLS);

	/* restore values to stop Data line SRP */
	ulpi_reg_write(ULPI_FUNCTION_CONTROL, (0x1 << 3) | ULPI_FC_SUSPENDM);
	ulpi_reg_write(ULPI_OTG_CONTROL, otg_ctrl);
	ulpi_reg_write(ULPI_FUNCTION_CONTROL, func_ctrl);
#if defined(CONFIG_PXA3xx_DVFM)
	unset_dvfm_constraint();
#endif
	return 0;
}

/* Called to start SRP
 */
static int pxa310_otgx_vbus_session(struct pxa_otg *pOtgHandle)
{
#if defined(CONFIG_PXA3xx_DVFM)
	set_dvfm_constraint();
#endif
#ifdef PXA3xx_OTG_VBUS_PMIC
	pOtgHandle->pmic_ops->otg_set_vbus(VBUS_PULSE);
	mdelay(T_B_SRP_INIT - T_B_DATA_PLS);
	pOtgHandle->pmic_ops->otg_set_vbus(VBUS_LOW);
#else
	ulpi_rtsm();

	/* start VBUS pulse SRP */
	ulpi_reg_write(ULPI_OTG_CONTROL_CLEAR, ULPI_OC_DISCHRGVBUS);
	ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_CHRGVBUS);

	/* stop VBUS pulse SRP */
	mdelay(T_B_SRP_INIT - T_B_DATA_PLS);
	ulpi_reg_write(ULPI_OTG_CONTROL_CLEAR, ULPI_OC_CHRGVBUS);
	ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_DISCHRGVBUS);

#endif
#if defined(CONFIG_PXA3xx_DVFM)
	unset_dvfm_constraint();
#endif
	return 0;
}

static int pxa310_otgx_check_b_hnp(void)
{
	int ret = 0;

	u2d_clk_enable();
	if (U2DOTGCR_BHNP & U2DOTGCR)
		ret = 1;
	u2d_clk_restore();
	return ret;
}

static int pxa310_otgx_check_vbus(void)
{
	int ret = 0;

	u2d_clk_enable();
	if (U2DOTGUSR & U2DOTGUSR_VV)
		ret = 1;
	u2d_clk_restore();
	return ret;
}

#define U2DOTGRSM_IRQ (U2DOTGINT_RLS1 | U2DOTGINT_FLS0)

static int pxa310_otgx_start_autoresume(void)
{
	u2d_clk_enable();
	pxa310_otgx_init();
	U2DOTGICR |= U2DOTGRSM_IRQ;
	u2d_clk_restore();
	return 1;
}

static int pxa310_otgx_drive_resume(void)
{
	u2d_clk_enable();
	pxa310_otgx_init();
	U2DOTGICR &= ~U2DOTGRSM_IRQ;
	ulpi_reg_write(ULPI_INTERFACE_CONTROL_SET, 0x10);
	pxa310_otgx_set_mode(USB_OTG_LP);
	ulpi_dat3_int_set(0);
	pxa310_otgx_init();
	ulpi_reg_write(ULPI_FUNCTION_CONTROL, 0x55);
	ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_DPPULLDOWN);
	ulpi_reg_write(ULPI_OTG_CONTROL_SET, ULPI_OC_DMPULLDOWN);
	return 1;
}

static struct otg_xceiv_ops pxa310_otg_xceiv_ops = {
	.otgx_get_mode                  = pxa310_otgx_get_mode,
	.otgx_set_mode                  = pxa310_otgx_set_mode,
	.otgx_init                      = pxa310_otgx_init,
	.otgx_detect_default_func	= pxa310_otgx_detect_default_func,
	.otgx_dp_session                = pxa310_otgx_dp_session,
	.otgx_vbus_session              = pxa310_otgx_vbus_session,
	.otgx_check_b_hnp               = pxa310_otgx_check_b_hnp,
	.otgx_check_vbus                = pxa310_otgx_check_vbus,
	.otgx_start_autoresume          = pxa310_otgx_start_autoresume,
	.otgx_drive_resume              = pxa310_otgx_drive_resume,
	.reset_xcvr_init		= reset_xcvr_init,
	.ulpi_dat3_work			= ulpi_dat3_work,
};

struct otg_xceiv_ops *init_pxa310_otg_xceiv_ops(void)
{
	return &pxa310_otg_xceiv_ops;
}
/******************************************************************
 * Pxa3xx OTG controller functions
 ******************************************************************
 */
static int pxa310_otgc_init(void)
{
	u32 u2dotgcr;

	U2DOTGCR = U2DOTGCR_UTMID;
	u2dotgcr = U2DOTGCR;

	u2dotgcr |= U2DOTGCR_ULE;
	U2DOTGCR = u2dotgcr;

	u2dotgcr |= U2DOTGCR_ULAF;
	u2dotgcr &= ~(U2DOTGCR_SMAF | U2DOTGCR_CKAF);
	U2DOTGCR = u2dotgcr;

	headset_init();
	return 0;
}

static void pxa310_otgc_interrupt_init(void)
{
	U2DOTGICR = U2DOTGINT_DEFAULT;
	U2DOTGCR |= U2DOTGCR_OTGEN;
}

/* This function is called when USB OTG interrupt happened
 */
static int pxa310_otgc_interrupt_handle(struct pxa_otg *pOtgHandle)
{
	u32 otgisr;
	u8 interrupt_state = OTG_INT_INIT;
	int state;

	state = pOtgHandle->xceiv_ops->otgx_get_mode();

	if ((state != USB_INT_OTG_CLIENT_DP) && (state != USB_OTG_PRE_SYNCH)) {
		pr_debug("state %d -> synch\n", state);
		pOtgHandle->xceiv_ops->otgx_init();
	}
	if (U2DOTGISR & U2DOTGINT_SI)
		U2DOTGISR = U2DOTGINT_SI;

	pr_debug("otgicr %x otgisr %x otgusr %x\n",
		 U2DOTGICR, U2DOTGISR, U2DOTGUSR);

	otgisr = U2DOTGISR;
	U2DOTGISR = otgisr;

	if ((otgisr &  U2DOTGINT_RLS1)) {
		interrupt_state |= OTG_INT_LS;
		state = USB_INT_OTG_CLIENT_DP;
	} else if (otgisr & U2DOTGINT_FLS0)
		interrupt_state |= OTG_INT_LP_DIS;

	/* OTG SET FEATURE received */
	if (otgisr & U2DOTGINT_SF) {
		U2DOTGISR = U2DOTGINT_SF;
		if (U2DOTGCR & U2DOTGCR_BHNP)
			pOtgHandle->otg.gadget->b_hnp_enable = 1;

		if (U2DOTGCR & U2DOTGCR_AHNP)
			pOtgHandle->otg.gadget->a_hnp_support = 1;

		if (U2DOTGCR & U2DOTGCR_AALTHNP)
			pOtgHandle->otg.gadget->a_alt_hnp_support = 1;
	}

	if (U2DOTGUSR & U2DOTGUSR_VV) {
		if (!pOtgHandle->otg_ctrl->b_sess_vld &&
			!pOtgHandle->otg_ctrl->a_vbus_vld)
			interrupt_state |= OTG_INT_RVV;
	}
	else {
		if (pOtgHandle->otg_ctrl->b_sess_vld ||
			pOtgHandle->otg_ctrl->a_vbus_vld)
			interrupt_state |= OTG_INT_FVV;
	}

	if (U2DOTGUSR & U2DOTGUSR_SV) {
		if (pOtgHandle->otg.state == OTG_STATE_A_IDLE)
			interrupt_state |= OTG_INT_RSV;
	}

	/* mini-A is plugged in */
	if (otgisr & U2DOTGINT_FID) {
		U2DOTGISR = U2DOTGINT_FID;
		/* check the current of OTG ID from the PHY */
		if (U2DOTGUSR & U2DOTGUSR_ID)
			goto out1;

		if (OTG_B_DEVICE == pOtgHandle->otg.default_a) {
			if ((get_headset_type(pOtgHandle) > 0)) {
				state = USB_OTG_CARKIT;
				headset_polling();
			} else
				interrupt_state |= OTG_INT_IDF;
		}

	}
out1:
	/* mini-A is plugged out */
	if (otgisr & U2DOTGINT_RID) {
		U2DOTGISR = U2DOTGINT_RID;

		/* check the current of OTG ID from the PHY */
		if (!(U2DOTGUSR & U2DOTGUSR_ID))
			goto out2;

		if (OTG_A_DEVICE == pOtgHandle->otg.default_a)
			interrupt_state |= OTG_INT_IDR;
	}

out2:

	if ((state != USB_INT_OTG_CLIENT_DP)
	    && (state != USB_OTG_PRE_SYNCH)) {
		pr_debug("synch -> state %d \n ", state);
		pOtgHandle->xceiv_ops->otgx_set_mode(state);
	}

	/* set carkit mode polling timer */
	if (USB_OTG_CARKIT == state)
		interrupt_state = 0;

	pr_debug("%s: interrupt type %x\n", __func__, interrupt_state);
	return interrupt_state;
}

static void pxa310_otgc_init_gadget(void)
{
	/* what if no gadget driver exists, OTG function is disabled
	 * as above?
	 */
	U2DOTGICR = 0;
	U2DOTGISR = U2DOTGISR;
}

static void pxa310_otgc_deinit(void)
{
	headset_deinit();
}

static struct otg_ctrl_ops pxa310_otg_ctrl_ops = {
	.otgc_init              = pxa310_otgc_init,
	.otgc_interrupt_init    = pxa310_otgc_interrupt_init,
	.otgc_interrupt_handle  = pxa310_otgc_interrupt_handle,
	.otgc_init_gadget       = pxa310_otgc_init_gadget,
	.otgc_deinit            = pxa310_otgc_deinit,
};

struct  otg_ctrl_ops *init_pxa310_otg_ctrl_ops(void)
{
	return &pxa310_otg_ctrl_ops;
}
#endif /* CONFIG_CPU_PXA310 */

