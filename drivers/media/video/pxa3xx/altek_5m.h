#ifndef __PXA3XX_ALTEK_5M_HEADER__
#define __PXA3XX_ALTEK_5M_HEADER__
#include "camera.h"

//#define ALTEK5M_DEBUG
//#undef ALTEK5M_DEBUG

#ifdef ALTEK5M_DEBUG
#define altek5m_dbg(fmt, arg...) printk("altek5m %s(line %d):" fmt , __FUNCTION__, __LINE__, ##arg)
#else
#define altek5m_dbg(fmt, arg...) do {} while (0)
#endif

#define	altek5m_info(fmt, arg...)    printk(KERN_DEBUG"altek5m,%s(line %d): " fmt, 	\
		__FUNCTION__, __LINE__, ##arg)
#define	altek5m_err(fmt, arg...)    printk(KERN_ERR"altek5m err,%s(line %d): " fmt, 	\
		__FUNCTION__, __LINE__, ##arg)

#if defined(CONFIG_BOARD_X2_V4)
#define CIF_nCOREEN		(MFP_PIN_GPIO2_2)
#define CIF_nPOWER1V8		(MFP_PIN_GPIO48)
#define CIF_nPOWERDN		(MFP_PIN_GPIO87)
#define CIF_PWREN2V8		(MFP_PIN_GPIO88)
#define CIF_VEXIOVDD		(MFP_PIN_GPIO120)
#define CIF_nRESET		(MFP_PIN_GPIO47)
#elif defined(CONFIG_BOARD_LANDMARK) /* fix me */
#define CIF_nPOWERDN		(MFP_PIN_GPIO47)
#define CIF_nRESET		(MFP_PIN_GPIO23)
#define CIF_nCOREEN		(MFP_PIN_GPIO90)
#else
#define CIF_nCOREEN		(MFP_PIN_GPIO2_2)
#define CIF_nPOWER1V8		(MFP_PIN_GPIO48)
#define CIF_nPOWERDN		(MFP_PIN_GPIO87)
#define CIF_PWREN2V8		(MFP_PIN_GPIO88)
#define CIF_VEXIOVDD		(MFP_PIN_GPIO120)
#define CIF_nRESET		(MFP_PIN_GPIO47)
#endif

#define CIF_nCOREEN_ALT		(MFP_AF0)
#define CIF_nPOWER1V8_ALT	(MFP_AF0)
#define CIF_nPOWERDN_ALT	(MFP_AF0)
#define CIF_PWREN2V8_AF	(MFP_AF0)
#define CIF_VEXIOVDD_AF		(MFP_AF0)
#define CIF_nRESET_ALT		(MFP_AF0)

#define CIF_MCLK_KHZ			(1300) /* 13M */
#define altek5m_ID					(0) /* main cam */

#define TbootupMs				(2000)

typedef enum _ALTEK5M_size{
	altek5m_size_2560_1920, //5M
	altek5m_size_2048_1536, //QXGA 3M
	altek5m_size_1600_1200, //UXGA 2M
	altek5m_size_1280_960, //QuadVGA 1.3M
	altek5m_size_800_600, //SVGA
	altek5m_size_640_480, //VGA
	altek5m_size_320_240, //QVGA
	altek5m_size_352_288, //CIF
	altek5m_size_176_144, //QCIF
	altek5m_size_max,
}altek5m_size;

typedef enum _altek5m_work_type{
	altek5m_work_detect,
	altek5m_work_doAF,
	altek5m_work_releaseAF,
	altek5m_work_none,
	altek5m_work_doAE,
}altek5m_work_type;

typedef enum _altek5m_status{
	altek5m_status_powerup,
	altek5m_status_preview,
	altek5m_status_capture,
	altek5m_status_powerdown,
	altek5m_status_max,
}altek5m_status;

#define SI_SIZE_W(w)			(w&0xfff) /* max 4095 */
#define SI_SIZE_H(h)			((h&0xfff)<<12) /* max 4095 */
#define SI_SIZE_MODE(mode)	((mode&0xf)<<24)
#define SI_SIZE_GTE_W(s)		(s&0xfff)
#define SI_SIZE_GTE_H(s)		((s>>12)&0xfff) /* max 4095 */
#define SI_SIZE_GTE_WH(s)	(s&0xffffff)
#define SI_SIZE_GTE_MODE(s)	((s>>24)&0xf) /* max 4095 */
#define SI_SIZE(w, h, mode)	(SI_SIZE_W(w)|SI_SIZE_H(h)|SI_SIZE_MODE(mode))

#endif

