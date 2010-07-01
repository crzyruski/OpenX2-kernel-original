
#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/workqueue.h>
#include <linux/usb/otg.h>

#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/pxa-regs.h>
#include <mach/udc.h>

#include "pxa3xx_otg.h"
#include "pxa300_otg.h"
#include "../gadget/pxa_comp.h"
#include "../gadget/pxa27x_udc.h"
extern struct pxa27x_udc *get_the_controller(void);
extern int is_cable_attached(void);
/******************************************************************
 * Pxa3xx OTG transceiver functions
 ******************************************************************
 */
static int xceiv_mode = USB_OTG_OFF;
static int pxa300_otgx_get_mode(void)
{
	return xceiv_mode;
}
/* Configures USB host port2 to the desired mode */
static int pxa300_otgx_set_mode(int mode)
{
	int status = 0;
	int up2ocr = UP2OCR;

	if (mode == USB_OTG_CARKIT)
		return 0;
	up2ocr &= ~(7 << OTG_UP2OCR_SEOS_SHIFT);
	up2ocr &= ~(OTG_UP2OCR_HXS | OTG_UP2OCR_HXOE);
	up2ocr &= ~(OTG_UP2OCR_DPPUE | OTG_UP2OCR_DMPUE);
	up2ocr &= ~(OTG_UP2OCR_DMPDE | OTG_UP2OCR_DPPDE);
	switch (mode) {
		/* Both differential port and single-ended port
		 * are turned off */
	case USB_OTG_OFF:
		break;
		/* Differential Port is off,
		 * Single-Ended Port2 is Non-OTG USB Client */
	case USB_NON_OTG_CLIENT_SEP:
		up2ocr |= (2 << OTG_UP2OCR_SEOS_SHIFT);
		break;

		/* Differential Port is off,
		 * Single-Ended Port2 is Non-OTG USB Host */
	case USB_NON_OTG_HOST_SEP:
		up2ocr |= (3 << OTG_UP2OCR_SEOS_SHIFT);
		break;

		/* Differential Port is off, Single-Ended Port2 is interfaced to
		 * external OTG transceiver and is directed into USB client
		 * controller
		 */
	case USB_EXT_OTG_CLIENT_SEP:
		up2ocr |= (4 << OTG_UP2OCR_SEOS_SHIFT);
		break;

		/* Differential Port is off, Single-Ended Port2 is interfaced to
		 * external OTG transceiver and is directed into USB host
		 * controller
		 */
	case USB_EXT_OTG_HOST_SEP:
		up2ocr |= (5 << OTG_UP2OCR_SEOS_SHIFT);
		break;
		/* Differential Port is Non-OTG USB client,
		 * Single-Ended Port 2 is Non-OTG USB host
		 */
	case USB_NON_OTG_HOST_SEP_CLIENT_DP:
		up2ocr |= OTG_UP2OCR_HXOE;
		up2ocr |= (3 << OTG_UP2OCR_SEOS_SHIFT);
		break;

		/* Differential Port is Non-OTG USB host, Single-Ended Port 2 is
		 * Non-OTG USB client
		 */
	case USB_NON_OTG_CLIENT_SEP_HOST_DP:
		up2ocr |= OTG_UP2OCR_HXOE;
		up2ocr |= OTG_UP2OCR_HXS;
		up2ocr |= (2 << OTG_UP2OCR_SEOS_SHIFT);
		break;

		/* Differential Port is USB OTG host, Single-Ended Port 2 is
		 * interfaced to external charge pump
		 */
	case USB_INT_OTG_HOST_DP:
		up2ocr |= OTG_UP2OCR_HXOE;
		up2ocr |= OTG_UP2OCR_HXS;
		up2ocr |= (7 << OTG_UP2OCR_SEOS_SHIFT);
		up2ocr |= (OTG_UP2OCR_DPPDE | OTG_UP2OCR_DMPDE);
		break;

		/* Differential Port is USB OTG client, Single-Ended Port 2 is
		 * interfaced to external charge pump
		 */
	case USB_INT_OTG_CLIENT_DP:
	case USB_OTG_LP:
		up2ocr |= OTG_UP2OCR_HXOE;
		up2ocr |= (6 << OTG_UP2OCR_SEOS_SHIFT);
		up2ocr |= OTG_UP2OCR_DPPUE;
		break;

		/* Differential Port is Non-OTG USB Client,
		 * Single-Ended Port2 is off */
	case USB_NON_OTG_CLIENT_DP:
		up2ocr |= OTG_UP2OCR_HXOE;
		break;

		/* Differential Port is Non-OTG USB Host,
		 * Single-Ended Port2 is off */
	case USB_NON_OTG_HOST_DP:
		up2ocr |= (OTG_UP2OCR_HXOE | OTG_UP2OCR_HXS);
		break;

	default:
		status = OTG_INVALID_PARAMETER;
		break;
	}
	UP2OCR = up2ocr;

	pr_debug("defore transceiver mode %d!\n", xceiv_mode);
	xceiv_mode = mode;
	pr_debug("set transceiver mode %d!\n", xceiv_mode);
	return status;
}

static void pxa300_otgx_init(void)
{
	pxa300_otgx_set_mode(USB_INT_OTG_CLIENT_DP);
}

static enum otg_function pxa300_otgx_detect_default_func(void)
{
	int value = OTG_B_DEVICE;
	struct pxa27x_udc *dev = get_the_controller();

	if (dev->mach->udc_is_miniA && dev->mach->udc_is_miniA())
		value = OTG_A_DEVICE;

	return value;
}

/* Called to start data-line SRP
 */
static int pxa300_otgx_dp_session(void)
{
	int up2ocr;

	/* Enable D+ pull-up resister to enable Data line SRP */
	up2ocr = UP2OCR;
	up2ocr &= ~OTG_UP2OCR_DPPDE;
	up2ocr |= OTG_UP2OCR_DPPUE;
	UP2OCR = up2ocr;
	mdelay(T_B_DATA_PLS);

	/* Remove D+ pull-up resister to stop data line SRP */
	up2ocr = UP2OCR;
	up2ocr &= ~OTG_UP2OCR_DPPUE;
	up2ocr |= OTG_UP2OCR_DPPDE;
	UP2OCR = up2ocr;
	return 0;
}

/* Called to start SRP
 */
static int pxa300_otgx_vbus_session(struct pxa_otg *pOtgHandle)
{
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
	return 0;
}

static int pxa300_otgx_check_b_hnp(void)
{
	int ret = 0;

	if (UDCCR_BHNP & UDCCR)
		ret = 1;
	return ret;
}

static int pxa300_otgx_check_vbus(void)
{
	return is_cable_attached();
}

static int pxa300_otgx_start_autoresume(void)
{
	return 0;
}

static int pxa300_otgx_drive_resume(void)
{
	return 0;
}

static struct otg_xceiv_ops pxa300_otg_xceiv_ops = {
	.otgx_get_mode                  = pxa300_otgx_get_mode,
	.otgx_set_mode                  = pxa300_otgx_set_mode,
	.otgx_init                      = pxa300_otgx_init,
	.otgx_detect_default_func       = pxa300_otgx_detect_default_func,
	.otgx_dp_session                = pxa300_otgx_dp_session,
	.otgx_vbus_session              = pxa300_otgx_vbus_session,
	.otgx_check_b_hnp               = pxa300_otgx_check_b_hnp,
	.otgx_check_vbus                = pxa300_otgx_check_vbus,
	.otgx_start_autoresume          = pxa300_otgx_start_autoresume,
	.otgx_drive_resume              = pxa300_otgx_drive_resume,
};

struct otg_xceiv_ops *init_pxa300_otg_xceiv_ops(void)
{
	return &pxa300_otg_xceiv_ops;
}
/******************************************************************
 * Pxa3xx OTG controller functions
 ******************************************************************
 */

int pxa300_otgc_init(void)
{
	if (UDCCR & UDCCR_UDE)
		UDCCR = UDCCR_UDE | UDCCR_OEN;
	else
		return OTG_UDC_DISABLED;
	return 0;
}

void pxa300_otgc_interrupt_init(void)
{
	pxa300_otgx_init();
	UP2OCR |= OTG_UP2OCR_IDON;
	UDCOTGICR |= (OTG_SETFEATURE | UDCOTGISR_IRIDR | UDCOTGISR_IRIDF);
}

/* This function is called when USB OTG interrupt happened
 */
int pxa300_otgc_interrupt_handle(struct pxa_otg *pOtgHandle)
{
	u32 otgisr;
	u8 interrupt_state = OTG_INT_INIT;
	struct pxa27x_udc *dev = get_the_controller();

	pr_debug("otgisr 0x%x, udccr 0x%x\n",
		 UDCOTGISR, UDCCR);
	otgisr = UDCOTGISR;

	/* OTG SET FEATURE received */
	if (otgisr & OTG_SETFEATURE) {
		if (UDCCR & UDCCR_BHNP)
			pOtgHandle->otg.gadget->b_hnp_enable = 1;

		if (UDCCR & UDCCR_AHNP)
			pOtgHandle->otg.gadget->a_hnp_support = 1;

		if (UDCCR & UDCCR_AALTHNP)
			pOtgHandle->otg.gadget->a_alt_hnp_support = 1;

		UDCOTGISR = OTG_SETFEATURE;
	}

	if (otgisr & (UDCOTGISR_IRIDF | UDCOTGISR_IRIDR)) {
		if (dev->mach->udc_is_miniA && dev->mach->udc_is_miniA())
			/* mini-A is plugged in */
			interrupt_state |= OTG_INT_IDF;
		else
			/* mini-A is plugged out */
			interrupt_state |= OTG_INT_IDR;

		UDCOTGISR = (UDCOTGISR_IRIDR | UDCOTGISR_IRIDF);
	}

	pr_debug("interrupt type %x\n", interrupt_state);
	return interrupt_state;
}

void pxa300_otgc_init_gadget(void)
{
	UDCOTGICR &= ~(OTG_SETFEATURE | UDCOTGISR_IRIDR | UDCOTGISR_IRIDF);
	UP2OCR &= ~(OTG_UP2OCR_IDON | (0x7 << OTG_UP2OCR_SEOS_SHIFT));
}

void pxa300_otgc_deinit(void)
{
}

static struct otg_ctrl_ops pxa300_otg_ctrl_ops = {
	.otgc_init              = pxa300_otgc_init,
	.otgc_interrupt_init    = pxa300_otgc_interrupt_init,
	.otgc_interrupt_handle  = pxa300_otgc_interrupt_handle,
	.otgc_init_gadget       = pxa300_otgc_init_gadget,
	.otgc_deinit            = pxa300_otgc_deinit,
};

struct otg_ctrl_ops *init_pxa300_otg_ctrl_ops(void)
{
	return &pxa300_otg_ctrl_ops;
}

