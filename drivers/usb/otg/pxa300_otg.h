#ifndef __PXA300_USB_OTG_CONTROLLER__
#define __PXA300_USB_OTG_CONTROLLER__

/* fields  and bits for UP2OCR */
#define OTG_UP2OCR_CPVEN	(1u << 0)
#define OTG_UP2OCR_CPVEP	(1u << 1)
#define OTG_UP2OCR_DPPDE	(1u << 2)
#define OTG_UP2OCR_DMPDE	(1u << 3)
#define OTG_UP2OCR_DPPUE	(1u << 4)
#define OTG_UP2OCR_DMPUE	(1u << 5)
#define OTG_UP2OCR_DPPUBE	(1u << 6)
#define OTG_UP2OCR_DMPUBE	(1u << 7)
#define OTG_UP2OCR_EXSP		(1u << 8)
#define OTG_UP2OCR_EXSUS	(1u << 9)
#define OTG_UP2OCR_IDON		(1u << 10)
#define OTG_UP2OCR_HXS		(1u << 16)
#define OTG_UP2OCR_HXOE		(1u << 17)
#define OTG_UP2OCR_SEOS_SHIFT	(24)

/* fields and bits for UDCOTGISR */
#define UDCOTGISR_IRIDF               (1u << 0)
#define UDCOTGISR_IRIDR               (1u << 1)
#define OTG_SETFEATURE                (1u << 24)

#define UDCOTGISR	__REG(0x4060001C) /* UDC OTG Interrupt Status Register*/
#define UP3OCR		__REG(0x40600024) /* Port 3 control register */

#endif

