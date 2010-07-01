/* Copyright (C) Yuhua Tel 2008 */
//#include <linux/config.h>
#include<linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/delay.h>    
#include <linux/interrupt.h>
#include <linux/timer.h>

#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/ioctl.h>
#include <linux/fs.h>
#include <mach/pxa-regs.h>
#include <mach/mfp-pxa300.h>
#include <mach/gpio.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>

#if 1
#define YH_PRINTK(fmt, arg...) printk(KERN_DEBUG"%s (%d): " fmt, 	\
			__FUNCTION__, MODULE_INDEX(pHuawei3g), ##arg)
#else
#define YH_PRINTK(format, arg...) do {} while(0)
#endif

struct yuhua_3g_modem {
	u32 hostWake_gpio;
	u32 onKey_gpio;
	u32 modemWake_gpio;
	u32 modemRest_gpio;
	u32 usbSuspend_gpio;

	u8 onoff;
	u8 active;	
	u8 index;

	spinlock_t	 modemwake_lock; /* lock between context and irq */
};

#define MODULE_INDEX(pHuawei) (pHuawei->index)

static int MODULEWAKE_ACTIVE(struct yuhua_3g_modem* pHuawei3g, int frmUsr) 
{
	spin_lock_irq(&pHuawei3g->modemwake_lock);
	
	if (!pHuawei3g->active) {
		pHuawei3g->active = 1;
		gpio_direction_output(pHuawei3g->modemWake_gpio, GPIO_LEVEL_LOW);	
		YH_PRINTK("user %d\n", frmUsr);
	}

	spin_unlock_irq(&pHuawei3g->modemwake_lock);
	return 0;
}	

static int MODULEWAKE_INACTIVE(struct yuhua_3g_modem* pHuawei3g, int frmUsr)
{
	spin_lock_irq(&pHuawei3g->modemwake_lock);
	
	if (pHuawei3g->active) {
		gpio_direction_output(pHuawei3g->modemWake_gpio, GPIO_LEVEL_HIGH);
		pHuawei3g->active = 0;
		YH_PRINTK("user %d\n", frmUsr);
	}

	spin_unlock_irq(&pHuawei3g->modemwake_lock);
	return 0;
}
	
//high---sleep state;  low---wakeup state
#define MODULEWAKE_STATUS(pHuawei3g) (!!gpio_get_value(pHuawei3g->hostWake_gpio))

#define huawei3g_NAME	"huawei_3g" 
#define IOCTL_MODULEWAKE_ACTIVE		_IO('T', 0)
#define IOCTL_MODULEWAKE_INACTIVE	_IO('T', 1)
#define IOCTL_POWER_ON					_IO('T', 2)
#define IOCTL_POWER_OFF				_IO('T', 3)
#define IOCTL_POWER_CON				_IO('T', 4)
#define IOCTL_POWER_COFF				_IO('T', 5)
#define IOCTL_GET_ONOFF					_IOR('T', 6, int)
#define IOCTL_POWER_QOFF				_IO('T', 7)
#define IOCTL_GET_MODULEWAKE			_IOR('T', 8, int)
#define IOCTL_S2PC						_IO('T', 9)
#define IOCTL_SETTTYS0					_IO('T', 10)
#define IOCTL_POWER_RESET				_IO('T',12)

static struct yuhua_3g_modem huawei3g_table[] = {
	{
		.hostWake_gpio = MFP_PIN_GPIO108,		
		.onKey_gpio = MFP_PIN_GPIO17,
		.modemWake_gpio = MFP_PIN_GPIO94,
		.modemRest_gpio = MFP_PIN_GPIO84,
		.usbSuspend_gpio = MFP_PIN_GPIO4_2,
	},
};

static struct file_operations huawei3g_fops;
static struct miscdevice huawei3g_miscdev[ARRAY_SIZE(huawei3g_table)] = {
	{ /* gsm 1 */
		.minor = MISC_DYNAMIC_MINOR,
		.name = huawei3g_NAME,
		.fops = &huawei3g_fops
	},
};

static long huawei3g_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct yuhua_3g_modem* pHuawei3g = (struct yuhua_3g_modem*)filp->private_data;
	int ret = 0;	
	
	switch( cmd ) {
	case IOCTL_MODULEWAKE_ACTIVE:
		MODULEWAKE_ACTIVE(pHuawei3g, 1);
		break;

	case IOCTL_MODULEWAKE_INACTIVE:
		MODULEWAKE_INACTIVE(pHuawei3g, 1);
		break;

	case IOCTL_POWER_ON:
		gpio_direction_output(pHuawei3g->onKey_gpio, GPIO_LEVEL_HIGH);
		msleep(1000);
		gpio_direction_output(pHuawei3g->onKey_gpio, GPIO_LEVEL_LOW);
		YH_PRINTK("huawei3g Power on\n");
		pHuawei3g->onoff = 1;
		break;
		
	case IOCTL_POWER_RESET:
		gpio_direction_output(pHuawei3g->modemRest_gpio, GPIO_LEVEL_LOW);
		msleep(1000);
		gpio_direction_input(pHuawei3g->modemRest_gpio);
		YH_PRINTK("huawei3g reset\n");
		break;

	case IOCTL_POWER_QOFF:
	case IOCTL_POWER_OFF:
		gpio_direction_output(pHuawei3g->onKey_gpio, GPIO_LEVEL_HIGH);
		msleep(3000);
		gpio_direction_output(pHuawei3g->onKey_gpio, GPIO_LEVEL_LOW);
		YH_PRINTK("huawei3g Power Off\n");
		pHuawei3g->onoff = 0;
		break;

	case IOCTL_POWER_CON:
		if(pHuawei3g->onoff == 0) {
			huawei3g_ioctl(filp, IOCTL_POWER_ON, arg);
		}
		break;

	case IOCTL_POWER_COFF:
		if(pHuawei3g->onoff == 1){
			huawei3g_ioctl( filp, IOCTL_POWER_OFF, arg);
		}
		break;

	case IOCTL_GET_ONOFF:
		put_user(pHuawei3g->onoff, (int *)arg);	
		break;

	case IOCTL_GET_MODULEWAKE:
		put_user(MODULEWAKE_STATUS(pHuawei3g), (int *)arg );
		break;
		
	case IOCTL_S2PC:	
	case IOCTL_SETTTYS0:
  		break;
		
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static irqreturn_t hostwake_irq_handler(int irq, void *dev_id) 
{
	struct yuhua_3g_modem* pHuawei3g = (struct yuhua_3g_modem*)dev_id;

	YH_PRINTK("Received irq and MODULEWAKE_STATUS=%d\n", MODULEWAKE_STATUS(pHuawei3g));
	//Received the falling trigger irq and current HOST_WAKEUP status is low, which means
	//that modem wakeup from sleep state, so active modem for double makesure correct modem state
	if(0 == MODULEWAKE_STATUS(pHuawei3g)){
		MODULEWAKE_ACTIVE(pHuawei3g, 0);
	}
	
	return IRQ_HANDLED;
}

static int huawei3g_open(struct inode *inode, struct file *filp)
{
	int i = iminor(inode);
	int ret = 0;
	filp->private_data = &huawei3g_table[0];
	return ret;
}

static int huawei3g_release( struct inode *inode, struct file *filp )
{
	return 0;
}

#ifdef CONFIG_PM
static int huawei3g_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i;
	for (i=0; i<ARRAY_SIZE(huawei3g_table); i++) {
		struct yuhua_3g_modem* pHuawei3g = (struct yuhua_3g_modem*)&huawei3g_table[i];
		//YH_PRINTK("inacitve Modem.\n");
		MODULEWAKE_INACTIVE(pHuawei3g, 0);
	}

	return 0;
}

static int huawei3g_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define huawei3g_suspend	NULL
#define huawei3g_resume	NULL
#endif

static struct file_operations huawei3g_fops =  {
	.owner = THIS_MODULE,
	.unlocked_ioctl = huawei3g_ioctl,
	.open = huawei3g_open,
	.release = huawei3g_release,	
};

static int huawei3g_data_init(struct yuhua_3g_modem* pHuawei3g)
{
	int ret;

	spin_lock_init(&pHuawei3g->modemwake_lock);

	/* init gpio */
	pxa3xx_mfp_set_afds(pHuawei3g->modemWake_gpio, MFP_AF0, MFP_DS03X);
	gpio_direction_output(pHuawei3g->modemWake_gpio, GPIO_LEVEL_LOW);
	pxa3xx_mfp_set_lpm(pHuawei3g->modemWake_gpio, MFP_LPM_DRIVE_HIGH); /* let it high when ap in sleep */

	pxa3xx_mfp_set_afds(pHuawei3g->onKey_gpio, MFP_AF0, MFP_DS03X);
	gpio_direction_output(pHuawei3g->onKey_gpio, GPIO_LEVEL_LOW);
	pxa3xx_mfp_set_lpm(pHuawei3g->onKey_gpio, MFP_LPM_DRIVE_LOW); /* let it low when ap in sleep */
	//pxa3xx_mfp_set_rdh(pHuawei3g->onKey_gpio, 1); /* Set RDH when ap in sleep */

	/*use reset as input,RSTON is HIGH*/
	pxa3xx_mfp_set_afds(pHuawei3g->modemRest_gpio, MFP_AF0, MFP_DS03X);
	gpio_direction_input(pHuawei3g->modemRest_gpio);
	pxa3xx_mfp_set_lpm(pHuawei3g->modemRest_gpio, MFP_LPM_DRIVE_HIGH); /* let it high when ap in sleep */
	pxa3xx_mfp_set_rdh(pHuawei3g->modemRest_gpio, 1); /* Set RDH when ap in sleep */

	pxa3xx_mfp_set_afds(pHuawei3g->usbSuspend_gpio, MFP_AF0, MFP_DS03X);
	gpio_direction_output(pHuawei3g->usbSuspend_gpio, GPIO_LEVEL_LOW);
	pxa3xx_mfp_set_lpm(pHuawei3g->usbSuspend_gpio, MFP_LPM_DRIVE_HIGH); /* let it high when ap in sleep */

	/* Active  modem in initial state */
	MODULEWAKE_ACTIVE(pHuawei3g, 0);

	/*if the modem want to send data,hostWake_gpio will be set,we will process modemWake_gpio in irq*/
	pxa3xx_mfp_set_afds(pHuawei3g->hostWake_gpio, MFP_AF0, MFP_DS03X);
	gpio_direction_input(pHuawei3g->hostWake_gpio);
	ret = request_irq(IRQ_GPIO(MFP2GPIO(pHuawei3g->hostWake_gpio)), hostwake_irq_handler,
					IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, huawei3g_NAME, pHuawei3g);
	if (ret<0) {
		printk("Request IRQ for huawei3g failed:%d\n", ret);
		return ret;
	}

	return ret;
}

static ssize_t gsm_state_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	int count = 0;
	int i;
	struct yuhua_3g_modem* pHuawei3g;
	
	for (i=0; i<ARRAY_SIZE(huawei3g_table); i++) {
		pHuawei3g = (struct yuhua_3g_modem*)&huawei3g_table[i];
		count += sprintf(buf, "Modem %d, actvie %d, onoff %d\n", pHuawei3g->index, pHuawei3g->active, pHuawei3g->onoff);
	}
	
	return count;
}
static DEVICE_ATTR(state, 0644, gsm_state_show, NULL);

static ssize_t modem_power_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct file filp;
	int power;
	
	sscanf(buf, "%d %d %d %d", &power);
	filp.private_data = &huawei3g_table[0];
	printk(KERN_ERR "modem_power_store %d\n", power);

	if (power)
		huawei3g_ioctl(&filp, IOCTL_POWER_ON, 0);
	else
		huawei3g_ioctl(&filp, IOCTL_POWER_OFF, 0);
	
	return count;
}
static DEVICE_ATTR(modem_power, 0644, NULL, modem_power_store);

static struct attribute *huawei3g_attributes[] = {
	&dev_attr_state.attr,
	&dev_attr_modem_power.attr,
	NULL,
};
static struct attribute_group huawei3g_attr_group ={
	.attrs= huawei3g_attributes,
};

static int __devinit huawei3g_probe(struct platform_device *pdev)
{
	int ret;
	int i;
	
	for (i=0; i<ARRAY_SIZE(huawei3g_table); i++) {
		huawei3g_table[i].index = i;
		huawei3g_data_init(&huawei3g_table[i]);		
	}	

	for (i=0; i<ARRAY_SIZE(huawei3g_miscdev); i++) {
		ret = misc_register(&huawei3g_miscdev[i]);
		if (ret<0)	{
			printk("Register device %s fail %d\n", huawei3g_miscdev[i].name, ret);
			return ret;
		}
	}
	
	ret = sysfs_create_group(&pdev->dev.kobj, &huawei3g_attr_group);
	
	printk("Huawei 3g-modem Driver registered \n");	
	return 0;
}

static int __devexit huawei3g_remove(struct platform_device *pdev)
{	
	return 0;
}

static struct platform_driver huawei3g_driver = {
	.probe		= huawei3g_probe,
	.remove		= __devexit_p(huawei3g_remove),
	.suspend	= huawei3g_suspend,
	.resume		= huawei3g_resume,
	.driver		= {
		.name	= "modem_huawei3g",
	},
};

static int __init huawei3g_init(void)
{
	return platform_driver_register(&huawei3g_driver);
}

static void __exit huawei3g_exit(void)
{
	platform_driver_unregister(&huawei3g_driver);
}

module_init(huawei3g_init);
module_exit(huawei3g_exit);
MODULE_LICENSE("GPL");

