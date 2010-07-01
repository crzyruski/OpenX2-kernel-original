#ifndef __PXA310_USB_OTG_CONTROLLER__
#define __PXA310_USB_OTG_CONTROLLER__

#define ULPI_VENDOR_LOW				0x0
#define ULPI_VENDOR_HIGH			0x1
#define ULPI_PRODUCT_LOW			0x2
#define ULPI_PRODUCT_HIGH			0x3
#define ULPI_FUNCTION_CONTROL			0x4
#define ULPI_FUNCTION_CONTROL_CLEAR		0x6
#define ULPI_FUNCTION_CONTROL_SET		0x5
#define ULPI_INTERFACE_CONTROL			0x7
#define ULPI_INTERFACE_CONTROL_SET		0x8
#define ULPI_INTERFACE_CONTROL_CLEAR		0x9
#define ULPI_OTG_CONTROL			0xA
#define ULPI_OTG_CONTROL_SET			0xB
#define ULPI_OTG_CONTROL_CLEAR			0xC
#define ULPI_INT_RISE				0xD
#define ULPI_INT_RISE_SET			0xE
#define ULPI_INT_RISE_CLEAR			0xF
#define ULPI_INT_FALL				0x10
#define ULPI_INT_FALL_SET			0x11
#define ULPI_INT_FALL_CLEAR			0x12
#define ULPI_INT_STATUS				0x13
#define ULPI_INT_LATCH				0x14
#define ULPI_DEBUG				0x15
#define ULPI_SCRATCH				0x16
#define ULPI_SCRATCH_SET			0x17
#define ULPI_SCRATCH_CLEAR			0x18

#ifdef CONFIG_MACH_LITTLETON
#define ULPI_CARKIT_CONTROL			0x19
#define ULPI_CARKIT_CONTROL_SET			0x1A
#define ULPI_CARKIT_CONTROL_CLEAR		0x1B
#define ULPI_CARKIT_INT_ENABLE			0x1D
#define ULPI_CARKIT_INT_ENABLE_SET		0x1E
#define ULPI_CARKIT_INT_ENABLE_CLEAR		0x1F
#define ULPI_CARKIT_INT_STATUS			0x20
#define ULPI_VENDOR_RID_CONV			0x36
#define ULPI_VENDOR_RID_CONV_SET		0x37
#define ULPI_VENDOR_RID_CONV_CLEAR		0x38
#endif

#define ULPI_FC_RESET				(1 << 5)        /* XCVR Reset */
#define ULPI_FC_SUSPENDM			(1 << 6)        /* XCVR SuspendM, Low Power Mode */

#define ULPI_IC_6PIN				(1 << 0)        /* XCVR 6 pin mode */
#define ULPI_IC_3PIN				(1 << 1)        /* XCVR 3 pin mode */
#ifdef CONFIG_MACH_LITTLETON
#define ULPI_IC_CARKIT				(1 << 2)        /* Carkit mode */
#endif
#define ULPI_IC_CLKSUSPENDM			(1 << 3)        /* Active low clock suspend */

#define ULPI_OC_IDPULLUP			(1 << 0)        /* ID Pull Up, enable sampling of ID line */
#define ULPI_OC_DPPULLDOWN			(1 << 1)        /* Enable the 15K Ohm pull down resistor on D+ */
#define ULPI_OC_DMPULLDOWN			(1 << 2)        /* Enable the 15K Ohm pull down resistor on D- */
#define ULPI_OC_DISCHRGVBUS			(1 << 3)        /* Discharge Vbus */
#define ULPI_OC_CHRGVBUS			(1 << 4)        /* Charge Vbus, for Vbus pulsing SRP */
#define ULPI_OC_DRVVBUS				(1 << 5)        /* Drive 5V on Vbus */
#define ULPI_OC_DRVVBUSEXT			(1 << 6)        /* Drive Vbus using external supply */

#define ULPI_INT_HOSTDISCON			(1 << 0)        /* Host Disconnect */
#define ULPI_INT_VBUSVALID			(1 << 1)        /* Vbus Valid */
#define ULPI_INT_SESSIONVALID			(1 << 2)        /* Session Valid */
#define ULPI_INT_SESSIONEND			(1 << 3)        /* Session End */
#define ULPI_INT_IDGND				(1 << 4)        /* current status of IDGND */

#define U2DOTGINT_DEFAULT	(U2DOTGINT_RID | U2DOTGINT_FID | U2DOTGINT_SI \
				  | U2DOTGINT_RVV | U2DOTGINT_FVV | U2DOTGINT_SF \
				  | U2DOTGINT_RSV)

#ifdef ENABLE_USB_HEADSET
/* Carkit Control Register Flags */
#define ULPI_CK_SPKLEFTEN			(1 << 4)        /* Connect DM to Speaker Left Pin */
#define ULPI_CK_SPKRIGHTEN			(1 << 5)        /* Connect DP to Speaker Right/Mic Pin */
#define ULPI_CK_SPKMICEN			(1 << 6)        /* Connect DP to Speaker Right/Mic Pin */

/* Carkit Interrupt Enable Register Flags */
#define ULPI_CK_IDFLOATRISE			(1 << 0)        /* Interrupt enabled when ID Pin transitions from Non-floating to Floating */
#define ULPI_CK_IDFLOATFALL			(1 << 1)        /* Interrupt enabled when ID Pin transitions from Floating to Non-floating */
#define ULPI_CK_RIDINTEN			(1 << 5)        /* Interrupt enabled when RidConvesionDone bit is asserted */

/* Carkit Rid Values */
#define RID_VALUE_MASK				0x7     /* Mask for RidValue Bits on Vendor RID Conversion Reg */
#define RID_0					0x0     /* Rid Value at 0K ohms */
#define RID_100K				0x2     /* Rid Value at 100K ohms */
#define RID_200K				0x3     /* Rid Value at 200K ohms */
#define RID_440K				0x4     /* Rid Value at 440K ohms */
#define RID_FLOATING				0x5     /* Rid Value indicates Floating */
#define RID_ERROR				0x7     /* Rid Value indicates Error */

/* Carkit Vendor Rid Conversion Register Flags */
#define RID_CONV_DONE				(1 << 3)        /* Automatically Asserted when RID conversion is finished */
#define RID_CONV_START				(1 << 4)        /* Flag to allow Rid ADC to start conversion process */

/* Carkit Interrupt Status Register Flags */
#define ULPI_CK_IDFLOAT				0x1 /* Asserted when ID pin is floating */

/* USB headset macros */
#define USB_HEADSET_STEREO	1
#define USB_HEADSET_MONO_MIC	2

/* Delay Carkit timer for 500 msec */
#define OTG_CARKIT_POLL_DELAY	500
#endif
#endif

