/* 
 * drivers/power/pxa3xx_hwcharger.c
 * Power supply driver for the yuhua pxa3xx board
 */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <linux/interrupt.h>

#if defined(CONFIG_BOARD_LANDMARK)
#define CHG_nCE_GPIO		(MFP_PIN_GPIO29) /* low enable */
#define CHG_WALL_EN_GPIO	(MFP_PIN_GPIO5_2)
#define CHG_FULL_GPIO		(MFP_PIN_GPIO53) /* low full */
#define USB_DETECT_GPIO		(MFP_PIN_GPIO1) /* low detect */
#elif defined(CONFIG_BOARD_BRAVA) || defined(CONFIG_BOARD_X2G)
#define CHG_nCE_GPIO		(MFP_PIN_GPIO80) /* low enable */
#define CHG_WALL_EN_GPIO	(MFP_PIN_GPIO89)
#define CHG_FULL_GPIO		(MFP_PIN_GPIO53) /* low full */
#define USB_DETECT_GPIO	(MFP_PIN_GPIO1) /* low detect */
#else
#define CHG_nCE_GPIO		(MFP_PIN_GPIO5_2) /* low enable */
#define CHG_WALL_EN_GPIO	(MFP_PIN_GPIO113)
#define CHG_FULL_GPIO		(MFP_PIN_GPIO12) /* low full */
#define USB_DETECT_GPIO		(MFP_PIN_GPIO111) /* low detect */
#endif

struct pxa3xx_hwcharger_data {
	spinlock_t lock;
	struct power_supply ac;
	struct power_supply usb;
	enum CABLE_TYPE cableType;
	int acChargerStatus;
	int usbChargerStatus;
	int cableConnect;
	int chargerFull; /* if we detect one charger full, set it */
	struct delayed_work cable_detect_work;
};
static struct pxa3xx_hwcharger_data *gHwCharger;

#define PXA_HWCHARGER_DEBUG
#ifdef PXA_BATT_DEBUG
#define pxa_hwchg_dbg(fmt, arg...) printk("PxaHwChg %s(line %d):" fmt , __FUNCTION__, __LINE__, ##arg)
#else
#define pxa_hwchg_dbg(fmt, arg...) do {} while (0)
#endif

extern u8 ulpi_get_dbg_reg(void);
extern int cable_detect_interrupt(void);
extern int otg_id_pin_level(void);

static int pxa3xx_hwchg_cable_connect_inter(void)
{
	if (gpio_get_value(USB_DETECT_GPIO))
		return 0;
	else
		return 1;
}

int pxa3xx_hwchg_cable_connect(void)
{
	if (gHwCharger && CABLE_TYPE_AC==gHwCharger->cableType)
		return 0; /* tricky to disable u2d when charger */
	else 
		return pxa3xx_hwchg_cable_connect_inter();
}

static void pxa3xx_hwcharger_start(enum CABLE_TYPE cableType)
{
	printk(KERN_DEBUG"pxa3xx_hwcharger_start %d\n", cableType);
	if (CABLE_TYPE_NONE==cableType) {
		pxa3xx_mfp_set_lpm(CHG_nCE_GPIO, MFP_LPM_PULL_HIGH);		
		gpio_direction_output(CHG_nCE_GPIO, GPIO_LEVEL_HIGH);
		
		pxa3xx_mfp_set_lpm(CHG_WALL_EN_GPIO, MFP_LPM_FLOAT);
		gpio_direction_output(CHG_WALL_EN_GPIO, GPIO_LEVEL_LOW);
	} else if (CABLE_TYPE_USB==cableType) {
		pxa3xx_mfp_set_lpm(CHG_nCE_GPIO, MFP_LPM_FLOAT);		
		gpio_direction_output(CHG_nCE_GPIO, GPIO_LEVEL_LOW);
#if 0 /* temp open wall also */
		pxa3xx_mfp_set_lpm(CHG_WALL_EN_GPIO, MFP_LPM_PULL_HIGH);
		gpio_direction_output(CHG_WALL_EN_GPIO, GPIO_LEVEL_HIGH);
#else
		pxa3xx_mfp_set_lpm(CHG_WALL_EN_GPIO, MFP_LPM_FLOAT);
		gpio_direction_output(CHG_WALL_EN_GPIO, GPIO_LEVEL_LOW);
#endif
	} else if (CABLE_TYPE_AC==cableType) {
		pxa3xx_mfp_set_lpm(CHG_nCE_GPIO, MFP_LPM_FLOAT);		
		gpio_direction_output(CHG_nCE_GPIO, GPIO_LEVEL_LOW);
		
		pxa3xx_mfp_set_lpm(CHG_WALL_EN_GPIO, MFP_LPM_DRIVE_HIGH);
		gpio_direction_output(CHG_WALL_EN_GPIO, GPIO_LEVEL_HIGH);
	}
}

static int pxa3xx_hwcharger_notify(enum CABLE_TYPE cableType)
{
	struct pxa3xx_hwcharger_data *data = gHwCharger;
	enum CABLE_TYPE oldType;

	if (!data)
		return -EIO;
	
	//printk(KERN_DEBUG"pxa3xx_hwcharger_notify %d\n", cableType);
	oldType = data->cableType;

	if (oldType!=cableType) {		
		pxa3xx_hwcharger_start(cableType);
		
		if (CABLE_TYPE_AC==oldType || CABLE_TYPE_AC==cableType)
			power_supply_changed(&data->ac);
		if (CABLE_TYPE_USB==oldType || CABLE_TYPE_USB==cableType)
			power_supply_changed(&data->usb);

		if (CABLE_TYPE_NONE==cableType) {	
			data->usbChargerStatus = POWER_SUPPLY_STATUS_DISCHARGING;
			data->acChargerStatus = POWER_SUPPLY_STATUS_DISCHARGING;
		} else if (CABLE_TYPE_USB==cableType) {
			data->usbChargerStatus = POWER_SUPPLY_STATUS_CHARGING;
			data->acChargerStatus = POWER_SUPPLY_STATUS_DISCHARGING;
		} else if (CABLE_TYPE_AC==cableType) {
			data->acChargerStatus = POWER_SUPPLY_STATUS_CHARGING;
			data->usbChargerStatus = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		
		data->cableType = cableType;
	}
	
	return 0;
}

/* call when u2d enable or u2d resume  */
int u2d_enable_notify_hwcharger(void)
{
	if (pxa3xx_hwchg_cable_connect_inter()) {
#ifdef CONFIG_USB_PXA3XX_U2D
		u8 dbg_val = ulpi_get_dbg_reg();
		printk(KERN_DEBUG"ULPI_DEBUG 0x%x\n", dbg_val);
#else
		u8 dbg_val = 0;
#endif
		dbg_val &= 0x3;
		if (dbg_val==3) {/* short connect */
			pxa3xx_hwcharger_notify(CABLE_TYPE_AC);			
#ifdef CONFIG_USB_PXA3XX_U2D /* tricky to disable u2d */
			cable_detect_interrupt();
#endif
		} else
			pxa3xx_hwcharger_notify(CABLE_TYPE_USB);
	} else
		pxa3xx_hwcharger_notify(CABLE_TYPE_NONE);

	return 0;
}

static void pxa3xx_cable_detect_work(struct work_struct *work)
{	
	struct pxa3xx_hwcharger_data *data = container_of(work, 
				struct pxa3xx_hwcharger_data, cable_detect_work.work);
#ifdef CONFIG_USB_OTG_PXA3XX
    if( otg_id_pin_level() == 0) {
        printk("USB is as host, dont' charge\n");
        return ;
    }
#endif    
	int newCableConnect = pxa3xx_hwchg_cable_connect_inter();

	if (data->cableConnect != newCableConnect) {
		data->cableConnect = newCableConnect;
#ifdef CONFIG_USB_PXA3XX_U2D
		cable_detect_interrupt(); /* let u2d to work */
#endif		
		if (data->cableConnect) {/* temp as usb now, detect again when u2d enable */
			if (data->cableType!=CABLE_TYPE_AC) /* workaround for ac */
				pxa3xx_hwcharger_notify(CABLE_TYPE_USB);				
		} else {
			pxa3xx_hwcharger_notify(CABLE_TYPE_NONE);
			data->chargerFull = 0;
		}
	}

	if (data->cableConnect) { /* query charger full */
		int chargerStatus;
		int cableType = data->cableType;

		if (data->chargerFull) {
			chargerStatus = POWER_SUPPLY_STATUS_FULL;
		} else if (gpio_get_value(CHG_FULL_GPIO)) {
			chargerStatus = POWER_SUPPLY_STATUS_FULL;
			data->chargerFull = 1;
		} else {
			chargerStatus = POWER_SUPPLY_STATUS_CHARGING;
		}
			
		if (CABLE_TYPE_AC==cableType) {
			if (data->acChargerStatus != chargerStatus) {
				power_supply_changed(&data->ac);
				data->acChargerStatus = chargerStatus;
			}
		} else if (CABLE_TYPE_USB==cableType) {
			if (data->usbChargerStatus != chargerStatus) {
				power_supply_changed(&data->usb);
				data->usbChargerStatus = chargerStatus;
			}
		}
	}
}

static irqreturn_t pxa3xx_cable_detect_irq(int irq, void *dev_id)
{
	struct pxa3xx_hwcharger_data *data = (struct pxa3xx_hwcharger_data*)dev_id;
	schedule_delayed_work(&data->cable_detect_work, HZ);
	return IRQ_HANDLED;
}

static irqreturn_t pxa3xx_hwcharger_full_irq(int irq, void *dev_id)
{/* fix me here */	
	struct pxa3xx_hwcharger_data *data = (struct pxa3xx_hwcharger_data*)dev_id;
	//printk(KERN_DEBUG"pxa3xx_hwcharger_full_irq\n");
	schedule_delayed_work(&data->cable_detect_work, HZ);	
	return IRQ_HANDLED;
}

static int pxa3xx_hwcharger_gpio_init(struct pxa3xx_hwcharger_data *data)
{
	int ret;	

	gpio_request(CHG_nCE_GPIO, "CHG_nCE");
	gpio_request(CHG_WALL_EN_GPIO, "CHG_WALL_EN");	
	
	pxa3xx_mfp_set_lpm(CHG_FULL_GPIO, MFP_LPM_PULL_HIGH);
	gpio_request(CHG_FULL_GPIO, "CHG_FULL");
	gpio_direction_input(CHG_FULL_GPIO);
	ret = request_irq(IRQ_GPIO(MFP2GPIO(CHG_FULL_GPIO)), pxa3xx_hwcharger_full_irq,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "CHG_FULL", data);
	printk("request CHG_FULL detect irq %d return %d\n", CHG_FULL_GPIO, ret);

	pxa3xx_mfp_set_lpm(USB_DETECT_GPIO, MFP_LPM_PULL_HIGH);
	gpio_request(USB_DETECT_GPIO, "usb detect");
	gpio_direction_input(USB_DETECT_GPIO);
	ret = request_irq(IRQ_GPIO(MFP2GPIO(USB_DETECT_GPIO)), pxa3xx_cable_detect_irq,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "usb detect", data);
	printk("request usb detect irq %d return %d\n", USB_DETECT_GPIO, ret);
	return ret;
}

static enum power_supply_property hwcharger_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};
static int hwcharger_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct pxa3xx_hwcharger_data *data = container_of(psy,
		struct pxa3xx_hwcharger_data, ac);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = data->acChargerStatus;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static enum power_supply_property hwcharger_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};
static int hwcharger_usb_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct pxa3xx_hwcharger_data *data = container_of(psy,
		struct pxa3xx_hwcharger_data, usb);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = data->usbChargerStatus;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

int pxa3xx_hwcharger_status(void)
{
	struct pxa3xx_hwcharger_data *data = gHwCharger;
	
	if (data && CABLE_TYPE_USB==data->cableType)
		return data->usbChargerStatus;
	else 	if (data && CABLE_TYPE_AC==data->cableType)
		return data->acChargerStatus;
	else
		return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int pxa3xx_hwcharger_resume(struct platform_device *pdev)
{
	struct pxa3xx_hwcharger_data *data = platform_get_drvdata(pdev);	
	schedule_delayed_work(&data->cable_detect_work, HZ);
	return 0;
}

/*static int pxa3xx_hwcharger_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct pxa3xx_hwcharger_data *data = platform_get_drvdata(pdev);
	return 0;
}*/

static int pxa3xx_hwcharger_probe(struct platform_device *pdev)
{
	int ret;
	struct pxa3xx_hwcharger_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	gHwCharger = data;
	
	spin_lock_init(&data->lock);
	data->cableType = CABLE_TYPE_UNKNOW;
	data->acChargerStatus = POWER_SUPPLY_STATUS_NOT_CHARGING;
	data->usbChargerStatus = POWER_SUPPLY_STATUS_NOT_CHARGING;
	data->ac.properties = hwcharger_ac_props;
	data->ac.num_properties = ARRAY_SIZE(hwcharger_ac_props);
	data->ac.get_property = hwcharger_ac_get_property;
	data->ac.name = "ac";
	data->ac.type = POWER_SUPPLY_TYPE_MAINS;
	ret = power_supply_register(&pdev->dev, &data->ac);

	data->usb.properties = hwcharger_usb_props;
	data->usb.num_properties = ARRAY_SIZE(hwcharger_usb_props);
	data->usb.get_property = hwcharger_usb_get_property;
	data->usb.name = "usb";
	data->usb.type = POWER_SUPPLY_TYPE_MAINS;
	ret += power_supply_register(&pdev->dev, &data->usb);	

	platform_set_drvdata(pdev, data);

	pxa3xx_hwcharger_gpio_init(data);

	INIT_DELAYED_WORK(&data->cable_detect_work, pxa3xx_cable_detect_work);
	schedule_delayed_work(&data->cable_detect_work, HZ*10); /* wait adb enable */

	printk("Yuhua pxa3xx board hwcharger register succ\n");
	return 0;

err_data_alloc_failed:
	printk("pxa3xx_hwcharger_probe fail %d\n", ret);
	return ret;
}

static int pxa3xx_hwcharger_remove(struct platform_device *pdev)
{
	struct pxa3xx_hwcharger_data *data = platform_get_drvdata(pdev);
	power_supply_unregister(&data->ac);
	power_supply_unregister(&data->usb);
	return 0;
}

static struct platform_driver pxa3xx_hwcharger_device = {
	.probe		= pxa3xx_hwcharger_probe,
	.remove		= pxa3xx_hwcharger_remove,
	.resume 		= pxa3xx_hwcharger_resume,
	//.suspend		= pxa3xx_hwcharger_suspend,
	.driver = {
		.name = "pxa3xx-hwcharger"
	}
};

static int __init pxa3xx_hwcharger_init(void)
{
	return platform_driver_register(&pxa3xx_hwcharger_device);
}

static void __exit pxa3xx_hwcharger_exit(void)
{
	platform_driver_unregister(&pxa3xx_hwcharger_device);
}

module_init(pxa3xx_hwcharger_init);
module_exit(pxa3xx_hwcharger_exit);

MODULE_AUTHOR("YuhuaTel");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("hwcharger driver for Yuhua Pxa3xx board");

