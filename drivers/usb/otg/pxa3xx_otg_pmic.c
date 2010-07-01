#include <linux/usb.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/workqueue.h>
#include <linux/usb/otg.h>

#include "pxa3xx_otg.h"

#include <mach/pxa3xx_pmic.h>
#include <mach/arava.h>
/******************************************************************
 * pmic special functions
 ******************************************************************
 */
static int otg_vbus_init(void)
{
	int status = 0;

#ifdef PXA3xx_OTG_VBUS_PMIC
	status = pxa3xx_pmic_set_pump(1);
	if (status)
		return OTG_I2C_ERROR;
#endif
	return 0;
}

/* Enable or disable USB Vbus
   session_enable: whether support to start a session by using VBUS
 */
static int otg_set_vbus(int vbus_type)
{
	int status = 0;

#ifdef PXA3xx_OTG_VBUS_PMIC
	pr_debug("vbus_type %d\n", vbus_type);
	switch (vbus_type) {
	case VBUS_PULSE:
		status = pxa3xx_pmic_set_vbus_supply(1, 1);
		break;
	case VBUS_HIGH:
		status = pxa3xx_pmic_set_vbus_supply(1, 0);
		break;
	case VBUS_LOW:
		status = pxa3xx_pmic_set_vbus_supply(0, 0);
		break;
	default:
		break;
	}
	if (status)
		return OTG_I2C_ERROR;
#endif
	return 0;
}

static int otg_set_vbus_ic(int function)
{
	int status = 0;

#ifdef PXA3xx_OTG_VBUS_PMIC
	switch (function) {
	case OTG_B_DEVICE:
		status = pxa3xx_pmic_set_usbotg_b_mask();
		break;
	case OTG_A_DEVICE:
		status = pxa3xx_pmic_set_usbotg_a_mask();
		break;
	default:
		break;
	}
	if (status)
		return OTG_I2C_ERROR;
#endif
	return 0;
}

static struct otg_pmic_ops pxa3xx_otg_pmic_ops = {
	.otg_vbus_init          = otg_vbus_init,
	.otg_set_vbus           = otg_set_vbus,
	.otg_set_vbus_ic        = otg_set_vbus_ic,
};

struct otg_pmic_ops *init_otg_pmic_ops(void)
{
	return &pxa3xx_otg_pmic_ops;
}


