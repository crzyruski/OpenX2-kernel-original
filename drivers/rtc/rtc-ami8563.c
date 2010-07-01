/*
 * An I2C driver for the AMI8563 RTC
 */
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <mach/micco.h>
#include <mach/pxa3xx_pmic.h>
#include <asm/mach/time.h>
#include <mach/pxa3xx_rtc.h>
#include <linux/notifier.h>
#include <linux/bcd.h>
#ifdef CONFIG_YUHUA_MISC_DEV	
#include <mach/yuhua_board_dev_info.h>
#endif

//#define AMI_DBG
#if defined(AMI_DBG)
#define ami8563_dbg(fmt, arg...) printk("ami8563,%s(line %d): " fmt, 	\
		__FUNCTION__, __LINE__, ##arg)
#else
#define ami8563_dbg(fmt, arg...) do {} while (0)
#endif

#define ami8563_info(fmt, arg...) printk(KERN_DEBUG"ami8563,%s(line %d): " fmt, 	\
		__FUNCTION__, __LINE__, ##arg)
#define ami8563_err(fmt, arg...) printk(KERN_ERR"ami8563 err,%s(line %d): " fmt, 	\
		__FUNCTION__, __LINE__, ##arg)

#define AMI8563_ADDR (0xA3 >> 1)
static unsigned short normal_i2c[] = {AMI8563_ADDR, I2C_CLIENT_END};
I2C_CLIENT_INSMOD_1(ami8563);
static struct i2c_client *g_client;

#define AMI8563_REG_ST1		0x00 /* status */
#define AMI8563_REG_ST2		0x01
#define AMI8563_REG_SC		0x02 /* datetime */
#define AMI8563_REG_MN		0x03
#define AMI8563_REG_HR		0x04
#define AMI8563_REG_DM		0x05
#define AMI8563_REG_DW		0x06
#define AMI8563_REG_MO		0x07
#define AMI8563_REG_YR		0x08
#define AMI8563_REG_AMN		0x09 /* alarm */
#define AMI8563_REG_AHR		0x0A
#define AMI8563_REG_ADM		0x0B
#define AMI8563_REG_ADW		0x0C
#define AMI8563_REG_CLKO	0x0D /* clock out */
#define AMI8563_REG_TMRC	0x0E /* timer control */
#define AMI8563_REG_TMR		0x0F /* timer */

#define AMI8563_SC_LV		0x80 /* low voltage */
#define AMI8563_MO_C		0x80 /* century */

#define SYNC_RTC_INTERVAL (HZ*60*5) /* 5min */

struct AMI8563 {
	struct i2c_client client;	
	//struct workqueue_struct* alarm_alert_workqueue;
	struct work_struct alarm_alert_work;	
	int alarm_status_power_on; /*0: mean no alarm, 1: mean alarm when power down*/
};

static inline int i2c_ami8563_read(struct i2c_client *client, u8 reg)
{
	return i2c_smbus_read_byte_data(client,reg);
}

static inline int i2c_ami8563_write(struct i2c_client *client, u8 reg,u8 val)
{
	return i2c_smbus_write_byte_data(client,reg,val);
}
#define ami8563_read(reg) i2c_ami8563_read(g_client, reg)
#define ami8563_write(reg, val) i2c_ami8563_write(g_client, reg, val)

static void rtc_sync_systime(struct rtc_time *tm)
{
	unsigned long time;
	struct timespec new_tv;

	rtc_tm_to_time(tm, &time);
	new_tv.tv_nsec = xtime.tv_nsec;
	new_tv.tv_sec = time;	
	do_settimeofday(&new_tv);
}

static void AMI8563_alarm_alert_notify(struct work_struct* work)
{
	struct rtc_device *rtc = i2c_get_clientdata(g_client);
	//ami8563_info("alarm on:KOBJ_CHANGE\n");
	if(rtc)
		kobject_uevent(&rtc->dev.kobj, KOBJ_CHANGE);
}

static int micco_enable_rtc_alarm_irq(int enable)
{
	int ret;
	u8 val;

	ret = micco_read(MICCO_IRQ_MASK_A, &val);
	if (enable) {	/* enable Alarm IRQ */
		val &= ~(IRQ_MASK_A_EXTON);
	} else {	/* disable Alarm IRQ */
		val |= (IRQ_MASK_A_EXTON);
	}
	ret += micco_write(MICCO_IRQ_MASK_A, val);
	
	return ret;
}

/* irq context */
static void AMI8563_alarm_event_handler(unsigned long event)
{
#if 0
	u8 st2;
	struct i2c_client* client = g_client;
        
	if (client) {
		st2 = i2c_ami8563_read(client, AMI8563_REG_ST2);
		if (st2&0x08) { /* alarm occur */
			struct rtc_device *rtc = i2c_get_clientdata(client);
			struct AMI8563 *pAMI8563 = container_of(client, struct AMI8563, client);
			
			st2 &= ~(0x1<<3|0x1<<1); /*Clear AF,clear AIE*/
			i2c_ami8563_write(client, AMI8563_REG_ST2, st2);			

			//schedule_work(&pAMI8563->alarm_alert_work); 
			rtc_update_irq(rtc, 1, event);
			ami8563_info("alarm occur\n");
		}
	}
#endif
}

static int AMI8563_get_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	u8 sc, mn, hr, dm, dw, mo, yr;

	sc = i2c_ami8563_read(client, AMI8563_REG_SC);
	mn = i2c_ami8563_read(client, AMI8563_REG_MN);
	hr = i2c_ami8563_read(client, AMI8563_REG_HR);
	dm = i2c_ami8563_read(client, AMI8563_REG_DM);
	dw = i2c_ami8563_read(client, AMI8563_REG_DW);
	mo = i2c_ami8563_read(client, AMI8563_REG_MO);
	yr = i2c_ami8563_read(client, AMI8563_REG_YR);

	tm->tm_sec = bcd2bin(sc & 0x7F);
	tm->tm_min = bcd2bin(mn & 0x7F);
	tm->tm_hour = bcd2bin(hr & 0x3F); /* rtc hr 0-23 */
	tm->tm_mday = bcd2bin(dm & 0x3F);
	tm->tm_wday = dw& 0x07;
	tm->tm_mon = bcd2bin(mo & 0x1F) - 1; /* rtc mn 1-12 */
	//tm->tm_year = buf[AMI8563_REG_MO]&0x80 ? (1900+BCD2BIN(buf[AMI8563_REG_YR])):(2000+BCD2BIN(buf[AMI8563_REG_YR]));
	tm->tm_year = bcd2bin(yr);
	if (tm->tm_year < 70)
		tm->tm_year += 100; /* assume we are in 1970...2069 */

	if (rtc_valid_tm(tm) < 0) {
		ami8563_err("date/time is not valid.\n");
	}

	ami8563_dbg("%d-%d-%d %d-%d-%d\n", tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday, 
		tm->tm_hour, tm->tm_min, tm->tm_sec);
	return 0;
}

static int AMI8563_set_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	u8 sc, mn, hr, dm, dw, mo, yr;	
	ami8563_info("%d-%d-%d %d-%d-%d\n", tm->tm_year, tm->tm_mon+1, tm->tm_mday, 
		tm->tm_hour, tm->tm_min, tm->tm_sec);

	sc = bin2bcd(tm->tm_sec);
	mn = bin2bcd(tm->tm_min);
	hr = bin2bcd(tm->tm_hour);
	dm = bin2bcd(tm->tm_mday);	
	mo = bin2bcd(tm->tm_mon + 1); /* month, 1 - 12 */	
	yr = bin2bcd(tm->tm_year % 100); /* year and century */
	if((tm->tm_year < 100))
		mo |= AMI8563_MO_C;
	dw = tm->tm_wday & 0x07;

	i2c_ami8563_write(client, AMI8563_REG_SC, sc);
	i2c_ami8563_write(client, AMI8563_REG_MN, mn);
	i2c_ami8563_write(client, AMI8563_REG_HR, hr);
	i2c_ami8563_write(client, AMI8563_REG_DM, dm);
	i2c_ami8563_write(client, AMI8563_REG_DW, dw);
	i2c_ami8563_write(client, AMI8563_REG_MO, mo);
	i2c_ami8563_write(client, AMI8563_REG_YR, yr);

	rtc_sync_systime(tm);
	return 0;
}

static int AMI8563_read_alarm(struct i2c_client *client, struct rtc_wkalrm* tm)
{
	u8 amn, ahr, adm, adw, st2;
	memset(tm, 0x0, sizeof(tm));

	st2 = i2c_ami8563_read(client, AMI8563_REG_ST2);
	if (st2&0x01<<1) {//check aie
		tm->enabled = 1;
		amn = i2c_ami8563_read(client, AMI8563_REG_AMN);	
		ahr = i2c_ami8563_read(client, AMI8563_REG_AHR);
		adm = i2c_ami8563_read(client, AMI8563_REG_ADM);
		adw = i2c_ami8563_read(client, AMI8563_REG_ADW);

		tm->time.tm_min = bcd2bin(amn);
		tm->time.tm_hour = bcd2bin(ahr);
		tm->time.tm_mday = bcd2bin(adm);
		tm->time.tm_wday = bcd2bin(adw);
	}

	ami8563_dbg("on:%d at %d-%d-%d\n", tm->enabled, tm->time.tm_mday,
		tm->time.tm_hour, tm->time.tm_min);	
	return 0;	
}

static int AMI8563_set_alram(struct i2c_client *client, struct rtc_wkalrm *tm)
{
	u8 amn, ahr, adm, adw, st2;

	if (tm->enabled) {
		amn = bin2bcd(tm->time.tm_min);
		ahr = bin2bcd(tm->time.tm_hour);
		adm = bin2bcd(tm->time.tm_mday);	
		adw = bin2bcd(tm->time.tm_wday);

		i2c_ami8563_write(client, AMI8563_REG_AMN, amn);	
		i2c_ami8563_write(client, AMI8563_REG_AHR, ahr);
		i2c_ami8563_write(client, AMI8563_REG_ADM, adm);
		i2c_ami8563_write(client, AMI8563_REG_ADW, adw);

		st2 = i2c_ami8563_read(client, AMI8563_REG_ST2);
		st2 &= ~(0x01<<3); /*Clear AF,enable AIE*/
		st2 |= 0x01<<1;
		i2c_ami8563_write(client, AMI8563_REG_ST2, st2);
	} else {
		st2 = i2c_ami8563_read(client, AMI8563_REG_ST2);
		st2 &= ~(0x01<<3 | 0x01<<1); /*Clear AF,clear AIE*/
		i2c_ami8563_write(client, AMI8563_REG_ST2, st2);
	}

	ami8563_info("on:%d at %d-%d-%d\n", tm->enabled, tm->time.tm_mday,
		tm->time.tm_hour, tm->time.tm_min);
	return 0;
}

static int AMI8563_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct i2c_client* client = (struct i2c_client*)to_i2c_client(dev);	
	struct AMI8563 *pAMI8563 = container_of(client, struct AMI8563, client);
	void __user *uarg = (void __user *)arg;
	int ret = 0;

	return ret;
}

static int AMI8563_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	return AMI8563_get_datetime((struct i2c_client*)to_i2c_client(dev), tm);
}

static int AMI8563_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	return AMI8563_set_datetime((struct i2c_client*)to_i2c_client(dev), tm);
}

static int AMI8563_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *tm)
{
	return AMI8563_read_alarm((struct i2c_client*)to_i2c_client(dev), tm);
}

static int AMI8563_rtc_set_alarm(struct device * dev, struct rtc_wkalrm * wkalrm)
{
	return AMI8563_set_alram(to_i2c_client(dev), wkalrm);
}

static const struct rtc_class_ops AMI8563_rtc_ops = {
	.read_time	= AMI8563_rtc_read_time,
	.set_time	= AMI8563_rtc_set_time,
	.set_alarm = AMI8563_rtc_set_alarm,
	.read_alarm = AMI8563_rtc_read_alarm,
	.ioctl = AMI8563_rtc_ioctl,
};

int AMI8563_update_rtc_time(void)
{
	unsigned long current_time = xtime.tv_sec;
	struct rtc_time tm;
	int status = 0;
	
	rtc_time_to_tm(current_time, &tm);

	if (g_client != NULL) {
		status = AMI8563_set_datetime(g_client, &tm);
	}

	return status;	
}
EXPORT_SYMBOL_GPL(AMI8563_update_rtc_time);

static struct timespec ami8563_rtc_delta;
static int AMI8563_rtc_suspend(struct i2c_client * client, pm_message_t mesg)
{
	struct rtc_time tm;
	struct timespec time;
	memset(&time, 0, sizeof(struct timespec));

	AMI8563_get_datetime(client, &tm);
	rtc_tm_to_time(&tm, &time.tv_sec);
	save_time_delta(&ami8563_rtc_delta, &time);
	return 0;
}

static int AMI8563_rtc_resume(struct i2c_client * client)
{
	struct rtc_time tm;
	struct timespec time;
	memset(&time, 0, sizeof(struct timespec));
	
	AMI8563_get_datetime(client, &tm);
	rtc_tm_to_time(&tm, &time.tv_sec);
	restore_time_delta(&ami8563_rtc_delta, &time);
	return 0;
}

#if defined(CONFIG_RTC_DRV_PXA3XX)
extern int sync_pxa3xx_rtc(struct rtc_time* tm);
static int AMI8563_sync_pxa3xx_rtc(struct i2c_client *client)
{
	struct rtc_time tm;
	ami8563_dbg("\n");
	
	AMI8563_get_datetime(client, &tm);
	return sync_pxa3xx_rtc(&tm);	
}
#else
static inline int AMI8563_sync_pxa3xx_rtc(struct i2c_client *client)
{
	return 0;
}
#endif

#define SYS_RECOVEY_MAGIC (0x42) /* 0x42 as magic */
static int ami_sys_reboot_notify(struct notifier_block *self, unsigned long action, void *data)
{
	if (action==SYS_RESTART && data) {
		if (0==strcmp("recovery", data)) {
			ami8563_write(AMI8563_REG_TMR, SYS_RECOVEY_MAGIC); 
			printk("Write rtc registr 0x%x for %s cmd\n", SYS_RECOVEY_MAGIC, (char*)data);
		}
	}
	
	return 0;
}

extern int register_reboot_notifier(struct notifier_block *nb);
static struct notifier_block ami_sys_reboot_nb = {
	.notifier_call = ami_sys_reboot_notify,
};

static struct i2c_driver AMI8563_driver;
static int AMI8563_probe(struct i2c_adapter *adapter, int address, int kind)
{
	struct AMI8563 *pAMI8563 = NULL;
	struct i2c_client *client = NULL;
	struct rtc_device *rtc = NULL;
	u8 value = 0;
	int err = 0;
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit;
	}

	if(address != AMI8563_ADDR) {
		err = -EIO;
		goto exit;
	}

	if (!(pAMI8563 = kzalloc(sizeof(struct AMI8563), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
	
	pAMI8563->alarm_status_power_on = 0;	
	client = &pAMI8563->client;
	client->addr = address;
	client->driver = &AMI8563_driver;
	client->adapter = adapter;
	strlcpy(client->name, AMI8563_driver.driver.name, I2C_NAME_SIZE);

	/* detect chip and check the alarm status*/
	err = i2c_ami8563_read(client, AMI8563_REG_ST2);
	if (err<0) {		
		goto exit_kfree;
	} else if (err&0x08) {
		ami8563_info("Power up as alarm on\n");
		pAMI8563->alarm_status_power_on = 1;
	}

	value = i2c_ami8563_read(client, AMI8563_REG_YR);
	ami8563_info("current year %d\n", value);
	value = bcd2bin(value);
#define CURRENT_YEAR (9) // 2009
	if (value>30 || value<CURRENT_YEAR) {
		ami8563_err("Detect invalid year %d, reset to default\n", value+2000);
		i2c_ami8563_write(client, AMI8563_REG_YR, bin2bcd(CURRENT_YEAR));
	}

	if ((err = i2c_attach_client(client)))
		goto exit_kfree;

	rtc = rtc_device_register(AMI8563_driver.driver.name, &client->dev,
				&AMI8563_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		err = PTR_ERR(rtc);
		goto exit_detach;
	}

	i2c_set_clientdata(client, rtc);
	g_client = client;

	/*Clear AF,enable AIE*/
	//micco_enable_rtc_alarm_irq(0);
	value = i2c_ami8563_read(client, AMI8563_REG_ST2);
	value &= ~0x1<<3;
	value |= 0x1<<1;
	i2c_ami8563_write(client, AMI8563_REG_ST2, value);
	micco_enable_rtc_alarm_irq(1);
		
	//err = pmic_callback_register(PMIC_EVENT_EXTON, AMI8563_alarm_event_handler);
	
	AMI8563_sync_pxa3xx_rtc(client);

	register_reboot_notifier(&ami_sys_reboot_nb); /* for enter into recovery mode */
#ifdef CONFIG_YUHUA_MISC_DEV	
	set_rtc_detect(1);
#endif
	
	printk("Rtc ami8563 drv register succ\n");
	return 0;

exit_detach:
	i2c_detach_client(client);
exit_kfree:
	kfree(pAMI8563);
exit:
	ami8563_err("err %d\n", err);
	return err;
}

static int AMI8563_attach(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, AMI8563_probe);
}

static int AMI8563_detach(struct i2c_client *client)
{
	struct AMI8563 *pAMI8563 = container_of(client, struct AMI8563, client);
	struct rtc_device *rtc = i2c_get_clientdata(client);

 	pmic_callback_unregister(PMIC_EVENT_EXTON, AMI8563_alarm_event_handler);
	
	if (rtc)
		rtc_device_unregister(rtc);
	
	i2c_detach_client(client);
	
	kfree(pAMI8563);
	
	return 0;
}

static struct i2c_driver AMI8563_driver = {
	.driver		= {
		.name	= "AMI8563",
	},
	.id		= 0,
	.attach_adapter = &AMI8563_attach,
	.detach_client	= &AMI8563_detach,
	//.suspend = &AMI8563_rtc_suspend,
	//.resume = &AMI8563_rtc_resume,
};

static int __init AMI8563_init(void)
{
	return i2c_add_driver(&AMI8563_driver);
}

static void __exit AMI8563_exit(void)
{
	i2c_del_driver(&AMI8563_driver);
}

MODULE_DESCRIPTION("AMI8563 RTC driver");
MODULE_LICENSE("GPL");
module_init(AMI8563_init);
module_exit(AMI8563_exit);

