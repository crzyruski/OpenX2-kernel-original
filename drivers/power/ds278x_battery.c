/*
 * Driver for batteries with DS278x (DS2780/DS2781) chips inside.
 *
 * Copyright (c) Marvell 2009
 *
 * Modified from ds278x_battery driver
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

#include "../w1/w1.h"
#define DS278x_DATA_SIZE 50

struct ds278x_device_info {
	struct device *dev;

	/* DS278x data, valid after calling ds278x_battery_read_status() */
	unsigned long update_time;	/* jiffies when data read */
	char raw[DS278x_DATA_SIZE];	/* raw DS278x data */
	int voltage_raw;		/* units of 4.88 mV */
	int voltage_uV;			/* units of µV */
	int current_raw;		/* units of 0.625 mA */
	int current_uA;			/* units of µA */
	int accum_current_raw;		/* units of 0.25 mAh */
	int accum_current_uAh;		/* units of µAh */
	int temp_raw;			/* units of 0.125 °C */
	int temp_C;			/* units of 0.1 °C */
	int rated_capacity;		/* units of µAh */
	int rem_capacity;		/* percentage */
	int full_active_uAh;		/* units of µAh */
	int empty_uAh;			/* units of µAh */
	int life_sec;			/* units of seconds */
	int charge_status;		/* POWER_SUPPLY_STATUS_* */

	int full_counter;
	struct power_supply bat;
	struct device *w1_dev;
	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
};

static unsigned int cache_time = 1000;
module_param(cache_time, uint, 0644);
MODULE_PARM_DESC(cache_time, "cache time in milliseconds");

static void ds278x_battery_work(struct work_struct *work)
{
	struct ds278x_device_info *di = container_of(work,
		struct ds278x_device_info, monitor_work.work);
	const int interval = HZ * 60;

	dev_dbg(di->dev, "%s\n", __func__);

	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, interval);
}

#define to_ds278x_device_info(x) container_of((x), struct ds278x_device_info, \
					      bat);

static void ds278x_battery_external_power_changed(struct power_supply *psy)
{
	struct ds278x_device_info *di = to_ds278x_device_info(psy);

	dev_dbg(di->dev, "%s\n", __func__);

	cancel_delayed_work(&di->monitor_work);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ/10);
}

static int ds278x_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct ds278x_device_info *di = to_ds278x_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->charge_status;
		return 0;
	default:
		break;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->voltage_uV;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->current_uA;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = di->rated_capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = di->full_active_uAh;
		break;
	case POWER_SUPPLY_PROP_CHARGE_EMPTY:
		val->intval = di->empty_uAh;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = di->accum_current_uAh;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->temp_C;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property ds278x_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_EMPTY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static int ds278x_battery_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct ds278x_device_info *di;
	struct ds278x_platform_data *pdata;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		retval = -ENOMEM;
		goto di_alloc_failed;
	}

	platform_set_drvdata(pdev, di);

	pdata = pdev->dev.platform_data;
	di->dev		= &pdev->dev;
	di->w1_dev	= pdev->dev.parent;
	di->bat.name	   = "battery";
	di->bat.type	   = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties     = ds278x_battery_props;
	di->bat.num_properties = ARRAY_SIZE(ds278x_battery_props);
	di->bat.get_property   = ds278x_battery_get_property;
	di->bat.external_power_changed =
				  ds278x_battery_external_power_changed;

	di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;

	retval = power_supply_register(&pdev->dev, &di->bat);
	if (retval) {
		dev_err(di->dev, "failed to register battery\n");
		goto batt_failed;
	}

	INIT_DELAYED_WORK(&di->monitor_work, ds278x_battery_work);
	di->monitor_wqueue = create_singlethread_workqueue(pdev->dev.bus_id);
	if (!di->monitor_wqueue) {
		retval = -ESRCH;
		goto workqueue_failed;
	}
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ * 1);

	goto success;

workqueue_failed:
	power_supply_unregister(&di->bat);
batt_failed:
	kfree(di);
di_alloc_failed:
success:
	return retval;
}

static int ds278x_battery_remove(struct platform_device *pdev)
{
	struct ds278x_device_info *di = platform_get_drvdata(pdev);

	cancel_rearming_delayed_workqueue(di->monitor_wqueue,
					  &di->monitor_work);
	destroy_workqueue(di->monitor_wqueue);
	power_supply_unregister(&di->bat);

	return 0;
}

#ifdef CONFIG_PM

static int ds278x_battery_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	struct ds278x_device_info *di = platform_get_drvdata(pdev);

	di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;

	return 0;
}

static int ds278x_battery_resume(struct platform_device *pdev)
{
	struct ds278x_device_info *di = platform_get_drvdata(pdev);

	di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
	power_supply_changed(&di->bat);

	cancel_delayed_work(&di->monitor_work);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ);

	return 0;
}

#else

#define ds278x_battery_suspend NULL
#define ds278x_battery_resume NULL

#endif /* CONFIG_PM */

MODULE_ALIAS("platform:ds278x-battery");

static struct platform_driver ds278x_battery_driver = {
	.driver = {
		.name = "ds278x-battery",
	},
	.probe	  = ds278x_battery_probe,
	.remove   = ds278x_battery_remove,
	.suspend  = ds278x_battery_suspend,
	.resume	  = ds278x_battery_resume,
};

static int __init ds278x_battery_init(void)
{
	return platform_driver_register(&ds278x_battery_driver);
}

static void __exit ds278x_battery_exit(void)
{
	platform_driver_unregister(&ds278x_battery_driver);
}

module_init(ds278x_battery_init);
module_exit(ds278x_battery_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ds278x battery driver");
