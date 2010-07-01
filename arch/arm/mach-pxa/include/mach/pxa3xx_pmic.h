#ifndef __PMIC_H__
#define __PMIC_H__

#include <linux/i2c.h>
#include <linux/interrupt.h>

/* this enum should be consistent with micco_power_module[]
 * in arch/arm/mach-pxa/xxx(platform).c */
enum {
	/* Set command > 0xFFFF0000 to avoid wrong
	 * parameter is used for pmic_set_voltage.
	 */
	VCC_CORE = 0xFFFF0000,
	VCC_SRAM,
	VCC_MVT,
	VCC_3V_APPS,
	VCC_SDIO,
	VCC_CAMERA_ANA,
	VCC_USB,
	VCC_LCD,
	VCC_TSI,
	VCC_CAMERA_IO,
	VCC_1P8V,
	VCC_MEM,
	HDMI_TX,
	TECH_3V,
	TECH_1P8V,

	/* add your command here */

	/* max command */
	MAX_CMD,
};

#define	PMIC_EVENT_EXTON	(1 << 0)
#define	PMIC_EVENT_VBUS		(1 << 1)
#define	PMIC_EVENT_USB		(PMIC_EVENT_EXTON | PMIC_EVENT_VBUS)

#define	PMIC_EVENT_TOUCH	(1 << 2)

#define PMIC_EVENT_OTGCP_IOVER	(1 << 3)

#define	PMIC_EVENT_TBAT		(1 << 4)
#define	PMIC_EVENT_REV_IOVER	(1 << 5)
#define	PMIC_EVENT_IOVER	(1 << 6)
#define	PMIC_EVENT_CHDET	(1 << 7)
#define	PMIC_EVENT_VBATMON	(1 << 8)
#define PMIC_EVENT_HSDETECT	(1 << 9)
#define PMIC_EVENT_HOOKSWITCH   (1 << 10)

#define PMIC_EVENT_ALARM (1 << 11)
/*add new event for TMQ CONFIG_DA9034_ON_SWITCH_INT*/
#define PMIC_EVENT_ONKEY (1 << 12)

#define	PMIC_EVENT_CHARGER	(PMIC_EVENT_TBAT |	\
				 PMIC_EVENT_REV_IOVER |	\
				 PMIC_EVENT_IOVER |	\
				 PMIC_EVENT_CHDET |	\
				 PMIC_EVENT_VBATMON)



struct pmic_ops {
	int (*get_voltage)(int cmd, int *pmv);
	int (*set_voltage)(int cmd, int mv);

	int (*is_vbus_assert)(void);
	int (*is_avbusvld)(void);
	int (*is_asessvld)(void);
	int (*is_bsessvld)(void);
	int (*is_srp_ready)(void);

	int (*set_pump)(int enable);
	int (*set_vbus_supply)(int enable, int srp);
	int (*set_usbotg_a_mask)(void);
	int (*set_usbotg_b_mask)(void);
	int (*is_usbpump_chg)(void);

	int (*init)(struct device *dev);
	int (*deinit)(struct device *dev);

	struct list_head list;	/* callback list */
	spinlock_t cb_lock;	/* spinlock for callback list */
};

struct pmic_callback {
	unsigned long event;	/* the event which callback care about */
	void (*func)(unsigned long event); /*callback function */
	struct list_head list;
};

struct pxa3xx_pmic_regs {
	unsigned int data:8;
	unsigned int hit:1;
	unsigned int mask:1;
};

extern void start_calc_time(void);
extern void end_calc_time(void);

extern int pxa3xx_pmic_write(u8 reg, u8 val);
extern int pxa3xx_pmic_read(u8 reg, u8 *pval);

extern void pmic_set_ops(struct pmic_ops *ops);

extern int pmic_callback_register(unsigned long event,
		void (*func)(unsigned long event));
extern int pmic_callback_unregister(unsigned long event,
		void (*func)(unsigned long event));

extern int pmic_event_handle(unsigned long event);

extern int pxa3xx_pmic_get_voltage(int cmd, int *pval);
extern int pxa3xx_pmic_set_voltage(int cmd, int val);
/* Check whether USB VBUS is asserted */
extern int pxa3xx_pmic_is_vbus_assert(void);
/* Check whether USB VBUS has gone above A-device VBUS valid threshold
 * Min: 4.4V	Max: N/A
 */
extern int pxa3xx_pmic_is_avbusvld(void);
/* Check whether VBUS has gone above A-device Session Valid threshold
 * Min: 0.8V	Max: 2.0V
 */
extern int pxa3xx_pmic_is_asessvld(void);
/* Check whether VBUS has gone above B-device Session Valid threshold
 * Min: 0.8V	Max: 4.0V
 */
extern int pxa3xx_pmic_is_bsessvld(void);
/* Check whether VBUS has gone above B-device Session End threshold
 * Min: 0.2V	Max: 0.8V
 */
extern int pxa3xx_pmic_is_srp_ready(void);
/* Initialize the USB PUMP */
extern int pxa3xx_pmic_set_pump(int enable);
/* Check the events change of PMIC */
extern int pxa3xx_pmic_event_change(void);
/* enable/disable VBUS supply */
extern int pxa3xx_pmic_set_vbus_supply(int enable, int srp);
/* Set events mask for USB A-device
 * A-device Sessino Valid event
 * A-device VBUS Valid event
 */
extern int pxa3xx_pmic_set_usbotg_a_mask(void);
/* Set events mask for USB B-device
 * B-device Session Valid event
 * B-device Session end event
 */
extern int pxa3xx_pmic_set_usbotg_b_mask(void);
#endif


