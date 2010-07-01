#ifndef __PXA3XX_OV2650_HEADER__
#define __PXA3XX_OV2650_HEADER__

#include "camera.h"

//#define OV2650_DEBUG

#ifdef OV2650_DEBUG
#define OV2650_dbg(fmt, arg...) printk("OV2650 %s(line %d):" fmt , __FUNCTION__, __LINE__, ##arg)
#else
#define OV2650_dbg(fmt, arg...) do {} while (0)
#endif

#define	OV2650_info(fmt, arg...)    printk(KERN_DEBUG"OV2650,%s(line %d): " fmt, 	\
		__FUNCTION__, __LINE__, ##arg)
#define	OV2650_err(fmt, arg...)    printk(KERN_ERR"OV2650 err,%s(line %d): " fmt, 	\
		__FUNCTION__, __LINE__, ##arg)

#define CIF_MCLK_KHZ			(2600) /* 26M */
#define OV2650_ID				(0) /* main cam */

#define CIF_nPOWERDN		(MFP_PIN_GPIO18)
#define CIF_nRESET			(MFP_PIN_GPIO70)

typedef enum _OV2650_size{
	ov2650_size_1600_1200, //UXGA 2M
	ov2650_size_1280_960, //QuadVGA 1.3M
	ov2650_size_800_600, //SVGA
	ov2650_size_640_480, //VGA
}OV2650_size;

#endif

