/* 
 * drivers/power/pxa3xx_battery.c
 * Power supply driver for the yuhua pxa3xx board
 *
 */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <mach/micco.h>
#include <linux/delay.h>
#include <mach/pxa3xx_pmic.h>
#include <linux/wakelock.h>

//#define BATT_MONITOR_TIMER (0x1)

struct pxa3xx_battery_data {
	spinlock_t lock;
	struct power_supply battery;
#ifdef BATT_MONITOR_TIMER
	struct timer_list monitor_timer; /* monitor battery */
#else
	struct delayed_work monitor_work; /* monitor battery */
#endif
	int poll_secs;
	int last_batt_vol;
	int last_batt_cap;
	int pending_batt_cap;
	int last_batt_temp;
	int measure_on;
	int measure_count;
};

struct battery_curve_entry {
	int vol_point; /* mv */
	int cap_point;	/* ma */
};

//#define PXA_BATT_DEBUG
#ifdef PXA_BATT_DEBUG
#define pxa_batt_dbg(fmt, arg...) printk("PxaBatt %s(line %d):" fmt , __FUNCTION__, __LINE__, ##arg)
#else
#define pxa_batt_dbg(fmt, arg...) do {} while (0)
#endif
#define pxa_batt_info(fmt, arg...) printk("PxaBatt %s(line %d):" fmt , __FUNCTION__, __LINE__, ##arg)

#define PXA_BATT_POLL_INTERVAL		(60*HZ)
#define PXA_COUNT_BATVOL(x)		((x*2650)/256+2650)
#define PXA_COUNT_BATREG_FROM_VOL(val) ((val-2650)*256/2650)
#define PXA_COUNT_BATTEMP(x)		((x*2650)/256)
#define PXA_BATT_MAX_CAP_STEP		(2) /* max capacity step between 2 measure */

static struct battery_curve_entry battery_curve[] = {
	{4150, 1200},
	{4030, 1100},
	{3930, 1000},
	{3880, 900},
	{3820, 800},
	{3770, 700},
	{3720, 600},
	{3690, 500},
	{3670, 400},
	{3650, 300},
	{3610, 200},
	{3570, 100},
	{3450, 0}, /* 5209 shutdown vol: 3400 */
	{-1, 0}
};

static struct battery_curve_entry charger_curve[] = { /* fix me */
	//{4150, 1200},/* cv */
	{4170, 1200},
	{3950, 600},
	//{3800, 180},
	//{3720, 50},
	{3450, 0},  /* 5209 shutdown vol: 3400 */
	{-1, 0},
};

static inline int pxa3xx_charger_status(void)
{
#ifdef CONFIG_CHARGER_PXA3xx_HARDWARE
	extern int pxa3xx_hwcharger_status(void);
	return pxa3xx_hwcharger_status();
#else
	return POWER_SUPPLY_STATUS_DISCHARGING;
#endif
}

static void pxa3xx_measure_battery(struct pxa3xx_battery_data *data, int enable)
{
	if (data->measure_on != enable) {
		if (enable) {/* Enable the VBAT auto measurements */		
			//micco_write(MICCO_ADC_MAN_CONTROL, MICCO_ADC_MAN_CONT_LDOADC_EN);
			//micco_write(MICCO_ADC_AUTO_CONTROL_1, MICCO_ADC_AUTO_1_VBAT_EN);
		} else {
			//micco_write(MICCO_ADC_MAN_CONTROL, 0);
			//micco_write(MICCO_ADC_AUTO_CONTROL_1, 0);
		}
		data->measure_on = enable;
	}
}

#define DIFF(x,y,result)		 \
	do{						\
		if(x>y) result=x-y;	\
		else result=y-x;		\
	} while(0)
static int pxa3xx_pick_battery_vol(u8* pData, u8* result) 
{
	u8 i;
	u8 diff[3];
	
	DIFF(pData[1],pData[0],diff[0]); 
	DIFF(pData[2],pData[0],diff[1]);
	DIFF(pData[2],pData[1],diff[2]);

	i = 0;
	if(diff[1]<diff[0]) {i++;diff[0]=diff[1];}
	if(diff[2]<diff[0]) i+=2;	

	if(i==0) {//Select diff[0]
		*result = (pData[0]+pData[1])/2;
	}else if(i==1) {//Select diff[1]
		*result = (pData[0]+pData[2])/2;
	}else { //Select diff[2]
		*result = (pData[1]+pData[2])/2;
	}

	return 0;
}

static int pxa3xx_measure_battery_vol(struct pxa3xx_battery_data *data)
{
	int vol, preVol = -1, preCap = -1, curCap, maxCap; /* mv ma */
	int cap, curve_size, i;
	u8 val, pre_val[3];
	struct battery_curve_entry* pCurve;

	for (i=0; i<3; i++) {
		micco_read(MICCO_VBAT_RES, &pre_val[i]);
		if (i<2) {
#ifdef BATT_MONITOR_TIMER
			udelay(100);
#else
			msleep(100);
#endif
		}
	}
	pxa3xx_pick_battery_vol(pre_val, &val);
	//printk(KERN_DEBUG"battery_vol 0x%x 0x%x 0x%x 0x%x\n", pre_val[0], pre_val[1], pre_val[2], val);
	
	vol = PXA_COUNT_BATVOL(val);
	if (vol<2800) {	/* an adc error */
		printk(KERN_DEBUG"pxa3xx_measure_battery_vol too low %d\n", vol);
		vol = 0; /* use last */
	}
		
	if (val) {
		data->last_batt_vol = vol;
	} else
		vol = data->last_batt_vol;

	/* count capacity */
	if (POWER_SUPPLY_STATUS_CHARGING==pxa3xx_charger_status()) {
		pCurve = charger_curve;  /* fix measure @ cv stage */
		curve_size = ARRAY_SIZE(charger_curve);
	} else {
		pCurve = battery_curve;
		curve_size = ARRAY_SIZE(battery_curve);
	}
	maxCap = pCurve->cap_point;
	for (i=0; i<curve_size; i++) {
		if (vol > pCurve->vol_point)
			break;
		preVol = pCurve->vol_point;
		preCap = pCurve->cap_point;
		pCurve++;
	}

	if (i>0) {
		curCap = pCurve->cap_point + 
			(vol-pCurve->vol_point)*(preCap-pCurve->cap_point)/(preVol-pCurve->vol_point);
		cap = curCap*100/maxCap;
	} else
		cap = 100;		
	//printk("%d %d %d %d %d %d\n", preVol, preCap, pCurve->vol_point, pCurve->cap_point, vol, cap);
	
	if (cap<0)
		cap = 0;
	else if (cap>100)
		cap = 100;

	if (data->measure_count) { /* apply PXA_BATT_MAX_CAP_STEP */
		if ((data->last_batt_cap-cap)>PXA_BATT_MAX_CAP_STEP) {
			/* may pull-down by big current, balance it */
			printk(KERN_DEBUG"Detect fast pull-down capacity(%d,%d)\n", cap, data->last_batt_cap);
			cap = data->last_batt_cap - PXA_BATT_MAX_CAP_STEP;
			if (data->pending_batt_cap) 
				data->pending_batt_cap = 0;
			else
				data->pending_batt_cap = cap;
		} else {
			data->pending_batt_cap = 0;
		}
	} else {
		data->pending_batt_cap = 0;
		if (cap==0) {/* fix bug: x2 shutdown after cpu resume, but battery still has power */
			printk(KERN_ERR"Resume shutdown volage detect, judge again\n");
			cap = 1;
			data->pending_batt_cap = cap;
			wake_lock_comman_timeout(10*HZ);
		}
	}
	data->measure_count++;

	if (!data->pending_batt_cap)
		data->last_batt_cap = cap;

	printk(KERN_DEBUG"pxa3xx_measure_battery_vol %d(%d, pending %d)\n", vol, cap, data->pending_batt_cap);
	return vol;
}

static int pxa3xx_measure_battery_temp(struct pxa3xx_battery_data *data)
{
#if 1 /* fix to measured by BB */
	return 320;
#else
	int temp;
	u8 val;

	micco_read(MICCO_TBAT_RES, &val);
	if (val) {
		temp = PXA_COUNT_BATTEMP(val);
		data->last_batt_temp= temp;
	} else
		temp = data->last_batt_temp;
	
	pxa_batt_dbg("%d\n", temp);
	return temp;
#endif
}

static inline int pxa3xx_get_battery_vol(struct pxa3xx_battery_data *data)
{
	return data->last_batt_vol;
}

static inline int pxa3xx_get_battery_temp(struct pxa3xx_battery_data *data)
{/* fix to measured by BB */
	return 320;
}

static int pxa3xx_measure_intermediate(struct pxa3xx_battery_data *data, int interval)
{
	pxa3xx_measure_battery(data, 1);
#ifdef BATT_MONITOR_TIMER
	mod_timer(&data->monitor_timer, jiffies+interval);
#else
	cancel_delayed_work(&data->monitor_work);
	schedule_delayed_work(&data->monitor_work, interval);
#endif
	return 0;
}

static enum power_supply_property pxa3xx_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_batt_vol,
	POWER_SUPPLY_PROP_batt_temp,
	//POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int pxa3xx_battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct pxa3xx_battery_data *data = container_of(psy, struct pxa3xx_battery_data, battery);
	int ret = 0;
	//pxa_batt_dbg("psp %d\n", psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:/* fix me */
		val->intval = pxa3xx_charger_status(); 
		break;
	case POWER_SUPPLY_PROP_HEALTH:/* fix me */
		val->intval = POWER_SUPPLY_HEALTH_GOOD; 
		break;
	case POWER_SUPPLY_PROP_PRESENT:/* fix me */
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:			
		val->intval = data->last_batt_cap;
		break;
	case POWER_SUPPLY_PROP_batt_vol:
		val->intval = data->last_batt_vol;
		break;
	case POWER_SUPPLY_PROP_batt_temp:
		val->intval = data->last_batt_temp;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = pxa3xx_measure_battery_vol(data);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static void pxa3xx_battery_external_power_changed(struct power_supply *psy)
{
	struct pxa3xx_battery_data *data = container_of(psy, struct pxa3xx_battery_data, battery);
	pxa3xx_measure_intermediate(data, HZ/2);
}

#ifdef BATT_MONITOR_TIMER
static void pxa3xx_battery_timer_handler(unsigned long pData)
{
	struct pxa3xx_battery_data *data = (struct pxa3xx_battery_data *)pData;
	pxa_batt_dbg("\n");

	if (data->measure_on) {
		pxa3xx_measure_battery_vol(data);
		//pxa3xx_measure_battery_temp(data);
		mod_timer(&data->monitor_timer, jiffies+data->poll_secs*HZ);
		pxa3xx_measure_battery(data, 0);
		power_supply_changed(&data->battery);
	} else { /* open it */
		pxa3xx_measure_battery(data, 1);
		mod_timer(&data->monitor_timer, jiffies+HZ*1);
	}
}
#else
static void pxa3xx_battery_work(struct work_struct *work)
{
	struct pxa3xx_battery_data *data = container_of(work, struct pxa3xx_battery_data, monitor_work.work);
	pxa_batt_dbg("\n");

	if (data->measure_on) {
		pxa3xx_measure_battery_vol(data);
		//pxa3xx_measure_battery_temp(data);		
		pxa3xx_measure_battery(data, 0);
		
		power_supply_changed(&data->battery);	
		if (data->last_batt_cap<15)
			schedule_delayed_work(&data->monitor_work, 5*HZ);
		else if (data->last_batt_cap<30)
			schedule_delayed_work(&data->monitor_work, 10*HZ);
		else
			schedule_delayed_work(&data->monitor_work, data->poll_secs*HZ);
	} else { /* open it */
		pxa3xx_measure_battery(data, 1);
		schedule_delayed_work(&data->monitor_work, HZ*1);
	}
}
#endif

static int pxa3xx_battery_resume(struct platform_device *pdev)
{
	struct pxa3xx_battery_data *data = platform_get_drvdata(pdev);	
	//micco_write(MICCO_ADC_MAN_CONTROL, MICCO_ADC_MAN_CONT_LDOADC_EN);
	pxa3xx_measure_intermediate(data, 1*HZ);
	return 0;
}

static int pxa3xx_battery_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct pxa3xx_battery_data *data = platform_get_drvdata(pdev);
	data->measure_count = 0;
	data->pending_batt_cap = 0;
	pxa3xx_measure_battery(data, 0);
	//micco_write(MICCO_ADC_MAN_CONTROL, 0);
	return 0;
}

static ssize_t poll_seconds_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{	
	struct pxa3xx_battery_data *data = (struct pxa3xx_battery_data*)dev_get_drvdata(dev);
	sscanf(buf, "%d", &data->poll_secs);
	if (!data->measure_on) 
		pxa3xx_measure_battery(data, 1);
		
#ifdef BATT_MONITOR_TIMER
	mod_timer(&data->monitor_timer, jiffies+HZ);
#else
	cancel_delayed_work(&data->monitor_work);
	schedule_delayed_work(&data->monitor_work, HZ);
#endif
	return count;
}
static ssize_t poll_seconds_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	struct pxa3xx_battery_data *data = (struct pxa3xx_battery_data*)dev_get_drvdata(dev);	
	return sprintf(buf, "%d\n", data->poll_secs);
}
static DEVICE_ATTR(poll_seconds,0644,poll_seconds_show,poll_seconds_store);
static struct attribute *battery_attributes[] = {
	&dev_attr_poll_seconds.attr,
	NULL,
};
static struct attribute_group battery_attr_group ={
	.attrs=battery_attributes,
};

static void pxa3xx_battery_interrupt(unsigned long event)
{	
	if (event & PMIC_EVENT_VBATMON) { /* Battery low power, do noting, let monitor-work judge */
		pxa_batt_info("PMIC_EVENT_VBATMON\n");
	} 
}

static int pxa3xx_battery_probe(struct platform_device *pdev)
{
	int ret;
	struct pxa3xx_battery_data *data;
	u8 val;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	spin_lock_init(&data->lock);	

	data->battery.properties = pxa3xx_battery_props;
	data->battery.num_properties = ARRAY_SIZE(pxa3xx_battery_props);
	data->battery.get_property = pxa3xx_battery_get_property;
	data->battery.name = "battery";
	data->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	data->battery.external_power_changed = pxa3xx_battery_external_power_changed;

	ret = power_supply_register(&pdev->dev, &data->battery);
	if (ret)
		goto err_battery_failed;

	data->poll_secs = 59;
	micco_write(MICCO_ADC_MAN_CONTROL, MICCO_ADC_MAN_CONT_LDOADC_EN);
	micco_write(MICCO_ADC_AUTO_CONTROL_1, MICCO_ADC_AUTO_1_VBAT_EN|
		MICCO_ADC_AUTO_1_DEB_VBAT_MON | MICCO_ADC_AUTO_1_AUTOADC_SLEEP_EN);
	/* enable low-power wakeup */
	micco_write(MICCO_VBATMON, PXA_COUNT_BATREG_FROM_VOL(3450));
	micco_read(MICCO_IRQ_MASK_A, &val);
	val &= ~IRQ_MASK_A_VBATMON;
	micco_write(MICCO_IRQ_MASK_A, val);

	ret = pmic_callback_register(PMIC_EVENT_VBATMON, pxa3xx_battery_interrupt);
	
	pxa3xx_measure_battery(data, 1);
#ifdef BATT_MONITOR_TIMER
	init_timer(&data->monitor_timer);
	data->monitor_timer.data    = (unsigned long)data;
	data->monitor_timer.function = pxa3xx_battery_timer_handler;
	mod_timer(&data->monitor_timer, jiffies+HZ * 1);
#else
	INIT_DELAYED_WORK(&data->monitor_work, pxa3xx_battery_work);
	schedule_delayed_work(&data->monitor_work, HZ * 1);
#endif
	data->last_batt_vol = battery_curve[2].vol_point;
	data->last_batt_temp = 320; /* 32C */

	platform_set_drvdata(pdev, data);
	ret = sysfs_create_group(&pdev->dev.kobj, &battery_attr_group);
	printk("Yuhua pxa3xx board battery register succ\n");
	return 0;

err_battery_failed:	
	kfree(data);
err_data_alloc_failed:
	printk("pxa3xx_battery_probe fail %d\n", ret);
	return ret;
}

static int pxa3xx_battery_remove(struct platform_device *pdev)
{
	struct pxa3xx_battery_data *data = platform_get_drvdata(pdev);
	power_supply_unregister(&data->battery);
	return 0;
}

static struct platform_driver pxa3xx_battery_device = {
	.probe		= pxa3xx_battery_probe,
	.remove		= pxa3xx_battery_remove,
	.resume 		= pxa3xx_battery_resume,
	.suspend		= pxa3xx_battery_suspend,
	.driver = {
		.name = "pxa3xx-battery"
	}
};

static int __init pxa3xx_battery_init(void)
{
	return platform_driver_register(&pxa3xx_battery_device);
}

static void __exit pxa3xx_battery_exit(void)
{
	platform_driver_unregister(&pxa3xx_battery_device);
}

module_init(pxa3xx_battery_init);
module_exit(pxa3xx_battery_exit);

MODULE_AUTHOR("YuhuaTel");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Battery driver for Yuhua Pxa3xx board");

