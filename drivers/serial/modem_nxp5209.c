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

#ifdef CONFIG_PXA3xx_MODEM_DVFM
#include <mach/dvfm.h>
#include <mach/pxa3xx_dvfm.h>
#include <mach/pxa3xx_pm.h>
#endif

#if 1
#define YH_PRINTK(fmt, arg...) printk(KERN_DEBUG"%s (%d): " fmt, 	\
			__FUNCTION__, MODULE_INDEX(pNxp), ##arg)
#else
#define YH_PRINTK(format, arg...) do {} while(0)
#endif

#define GSM2_SERIAL_RTS (MFP_PIN_GPIO16)
#define GSM2_SERIAL_CTS (MFP_PIN_GPIO15)

struct nxp_5209{
	u32 hostWake_gpio;
	u32 onKey_gpio;
	u32 modemWake_gpio;
	u32 modemRest_gpio;	
	
	u8 modulewake_pending;
	u8 can_modulewake_active;
	u8 onoff;
	u8 active;
	
	struct timer_list modulewake_timer;
	
	int index;		
#ifdef CONFIG_PXA3xx_MODEM_DVFM
	int dvfm_dev_idx;
#endif

	spinlock_t	 modemwake_lock; /* lock between context and irq */

	struct device *p_dev;
	struct work_struct  modem_detect_work;
	
};

#define MODULE_INDEX(pNxp) (pNxp->index)

#if defined(CONFIG_SERIAL_XR20M)
extern int xr20m_RTS_level(int level);
#endif
static int MODULEWAKE_ACTIVE(struct nxp_5209* pNxp, int frmUsr) 
{
	spin_lock_irq(&pNxp->modemwake_lock);
	
	if (!pNxp->active) {
		pNxp->active = 1;
#ifdef CONFIG_PXA3xx_MODEM_DVFM
		spin_unlock_irq(&pNxp->modemwake_lock); /* disable D0CS in no_lock_irq mode */
		dvfm_disable_op_name("D0CS", pNxp->dvfm_dev_idx);
		spin_lock_irq(&pNxp->modemwake_lock);
#endif
		gpio_direction_output(pNxp->modemWake_gpio, GPIO_LEVEL_LOW);
		/* we enable RTS ,modem can send data from uart,when we active modem*/
		if (0==MODULE_INDEX(pNxp)) {
			gpio_direction_output(MFP_PIN_GPIO84, GPIO_LEVEL_LOW);
		} else {
#if defined(CONFIG_SERIAL_XR20M) 
			xr20m_RTS_level(GPIO_LEVEL_LOW); 
#elif defined(CONFIG_SERIAL_GSM2_BT)
			gpio_direction_output(GSM2_SERIAL_RTS, GPIO_LEVEL_LOW);
#endif
		}		
		//YH_PRINTK("user %d\n", frmUsr);
	}

	spin_unlock_irq(&pNxp->modemwake_lock);
	return 0;
}	

static int MODULEWAKE_INACTIVE(struct nxp_5209* pNxp, int frmUsr)
{
	spin_lock_irq(&pNxp->modemwake_lock);
	
	if (pNxp->active) {
		gpio_direction_output(pNxp->modemWake_gpio, GPIO_LEVEL_HIGH);
		/*we disable RTS,when we inactive modem,modem will not send any data in sleep mode*/
		if (0==MODULE_INDEX(pNxp)) { /* gsm1 */	
			/*gpio_direction_output(MFP_PIN_GPIO84, GPIO_LEVEL_HIGH);	*/
			gpio_direction_input(MFP_PIN_GPIO84);
		} else {
#if defined(CONFIG_SERIAL_XR20M)
			xr20m_RTS_level(GPIO_LEVEL_HIGH);
#elif defined(CONFIG_SERIAL_GSM2_BT)
			/*gpio_direction_output(GSM2_SERIAL_RTS, GPIO_LEVEL_HIGH);	*/
			gpio_direction_input(GSM2_SERIAL_RTS);
#endif
		} 

		pNxp->active = 0;
#ifdef CONFIG_PXA3xx_MODEM_DVFM
		dvfm_enable_op_name_aync("D0CS", pNxp->dvfm_dev_idx);
#endif
		//YH_PRINTK("user %d\n", frmUsr);
	}

	spin_unlock_irq(&pNxp->modemwake_lock);
	return 0;
}
	
//high inactive low active
#define MODULEWAKE_STATUS(pNxp) (!!gpio_get_value(pNxp->hostWake_gpio))

#define NXP5209_MAJOR	200
#define NXP5209_NAME	"NXP5209" 
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

static struct nxp_5209 nxp5209_table[] = {
	{
#if defined(CONFIG_BOARD_BRAVA) || defined(CONFIG_BOARD_X2G)
		.hostWake_gpio = MFP_PIN_GPIO83,		
		.onKey_gpio = MFP_PIN_GPIO82,
		.modemWake_gpio = MFP_PIN_GPIO81,
		.modemRest_gpio = MFP_PIN_GPIO17,
#else
		.hostWake_gpio = MFP_PIN_GPIO81,		
		.onKey_gpio = MFP_PIN_GPIO82,
		.modemWake_gpio = MFP_PIN_GPIO83,
		.modemRest_gpio = MFP_PIN_GPIO90,
#endif
		.modulewake_pending=0 ,
		.can_modulewake_active=1,
	},
#if defined(CONFIG_DUAL_MODEM)
	{
		.hostWake_gpio = MFP_PIN_GPIO29,
		.onKey_gpio = MFP_PIN_GPIO80,
		.modemWake_gpio = MFP_PIN_GPIO53,
		.modemRest_gpio = MFP_PIN_GPIO89,
		.modulewake_pending=0 ,
		.can_modulewake_active=1,
	},
#endif
};

static struct file_operations nxp5209_fops;
static struct miscdevice nxp5209_miscdev[ARRAY_SIZE(nxp5209_table)] = {
	{ /* gsm 1 */
		.minor = MISC_DYNAMIC_MINOR,
		.name = "yuhua_gsm",
		.fops = &nxp5209_fops
	},
#if defined(CONFIG_DUAL_MODEM)
	{ /* gsm 2 */
		.minor = MISC_DYNAMIC_MINOR,
		.name = "yuhua_gsm2",
		.fops = &nxp5209_fops
	},
#endif
};

static long nxp5209_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct nxp_5209* pNxp = (struct nxp_5209*)filp->private_data;
	int ret = 0;	
	
	switch( cmd ) {
	case IOCTL_MODULEWAKE_ACTIVE:
		MODULEWAKE_ACTIVE(pNxp, 1);
		break;

	case IOCTL_MODULEWAKE_INACTIVE:
		MODULEWAKE_INACTIVE(pNxp, 1);
#if 0	 /* always let 5209 can active us, by frank */
		pNxp->can_modulewake_active = 0;
		mod_timer(&pNxp->modulewake_timer, jiffies + 2);
#endif
		break;

	case IOCTL_POWER_ON:
	case IOCTL_POWER_RESET:
/*		gpio_direction_output(pNxp->modemRest_gpio, GPIO_LEVEL_LOW);
		msleep(100);
		gpio_direction_output(pNxp->modemRest_gpio, GPIO_LEVEL_HIGH);*/

		/* pull reset pin low when power on 5209, try to fix sometimes can't power off 5209 issue */
		/*if we set the reset to high,after thw AP wakeup from sleep,the modem will be reset!*/
		gpio_direction_output(pNxp->modemRest_gpio, GPIO_LEVEL_LOW);
		gpio_direction_output(pNxp->onKey_gpio, GPIO_LEVEL_HIGH);
		msleep(100);
		/*use reset as input,RSTON is HIGH*/
		gpio_direction_input(pNxp->modemRest_gpio);
		gpio_direction_output(pNxp->onKey_gpio, GPIO_LEVEL_LOW);

		YH_PRINTK("NXP5209 Power on\n");
		pNxp->onoff = 1;
		break;

	case IOCTL_POWER_QOFF:
	case IOCTL_POWER_OFF:
		gpio_direction_output(pNxp->onKey_gpio, GPIO_LEVEL_HIGH);
		msleep(3000);
		gpio_direction_output(pNxp->onKey_gpio, GPIO_LEVEL_LOW);
		YH_PRINTK("NXP5209 Power Off\n");
		pNxp->onoff = 0;
		break;

	case IOCTL_POWER_CON:
		if(pNxp->onoff == 0) {
		//	nxp5209_ioctl(filp, IOCTL_POWER_ON, arg);		
			/* pull reset pin low when power on 5209, try to fix sometimes can't power off 5209 issue */
		        /*if we set the reset to high,after thw AP wakeup from sleep,the modem will be reset!*/
		        gpio_direction_output(pNxp->modemRest_gpio, GPIO_LEVEL_LOW);
			gpio_direction_output(pNxp->onKey_gpio, GPIO_LEVEL_HIGH);
			msleep(100);
		        /*use reset as input,RSTON is HIGH*/
		        gpio_direction_input(pNxp->modemRest_gpio);
		        gpio_direction_output(pNxp->onKey_gpio, GPIO_LEVEL_LOW);

			YH_PRINTK("NXP5209 Power on\n");
			pNxp->onoff = 1;
		}
		break;

	case IOCTL_POWER_COFF:
		if(pNxp->onoff == 1){
			//nxp5209_ioctl( filp, IOCTL_POWER_OFF, arg);	
			gpio_direction_output(pNxp->onKey_gpio, GPIO_LEVEL_HIGH);
			//hongcheng.xie delete 2009-05-18 and let ril control the modem power off time		
			//msleep(3000);
			//gpio_direction_output(pNxp->onKey_gpio, GPIO_LEVEL_LOW);
			YH_PRINTK("NXP5209 Power Off\n");
			pNxp->onoff = 0;	
		}
		break;

	case IOCTL_GET_ONOFF:
		put_user(pNxp->onoff, (int *)arg);	
		break;

	case IOCTL_GET_MODULEWAKE:
		put_user(MODULEWAKE_STATUS(pNxp), (int *)arg );
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

static void modulewake_timer_handler(unsigned long ptr)
{
	struct nxp_5209* pNxp = (struct nxp_5209*)ptr;

	/*If there is pending request MODULEWAKE high*/
	if (pNxp->modulewake_pending) {
		YH_PRINTK("processing pending request!\n");
		MODULEWAKE_ACTIVE(pNxp, 0);
		pNxp->modulewake_pending = 0;
	}

	pNxp->can_modulewake_active = 1;
}

static void send_modem_event(struct work_struct *p_work)
{
	int ret = -1;
	char name_buf[120];
	char state_buf[120];
	char *prop_buf = NULL;
	char *envp[3];
	int env_offset = 0;
	struct nxp_5209 *pNxp = container_of(p_work, 
				struct nxp_5209, modem_detect_work);
	struct device *p_dev = pNxp->p_dev;

	if(p_dev != NULL){
		YH_PRINTK("----send_modem_event and SWITCH_NAME=%s----\n", 
			nxp5209_miscdev[MODULE_INDEX(pNxp)].name);
		snprintf(name_buf, sizeof(name_buf),"SWITCH_NAME=%s", 
			nxp5209_miscdev[MODULE_INDEX(pNxp)].name);
		envp[env_offset++] = name_buf;
					
		snprintf(state_buf, sizeof(state_buf),"SWITCH_STATE=%d", 0);
		envp[env_offset++] = state_buf;
					
		envp[env_offset] = NULL;
		ret = kobject_uevent_env(&p_dev->kobj, KOBJ_CHANGE, envp);
	}
}


static irqreturn_t hostwake_irq_handler(int irq, void *dev_id) 
{
	struct nxp_5209* pNxp = (struct nxp_5209*)dev_id;

	/* pls rewrite to set MODULEWAKE_ACTIVE always if below code work well */
	
	if (gpio_get_value(pNxp->hostWake_gpio)!=0) {
		YH_PRINTK("IRQ HOSTWAKE goes high\n");
		schedule_work(&pNxp->modem_detect_work);
		if (pNxp->can_modulewake_active) {
			//YH_PRINTK("NXP5209: MODULEWAKE goes HIGH\n");
			MODULEWAKE_ACTIVE(pNxp, 0);
		} else { /* make sure it active */
			YH_PRINTK("NXP5209: MODULEWAKE high request is pended!\n");
			MODULEWAKE_ACTIVE(pNxp, 0);
			pNxp->modulewake_pending = 1;
		}
	} else { /* make sure it active */
		/* falling edge, Ignoring this for now, let host decide when to sleep.*/
		//YH_PRINTK("NXP5209: IRQ HOSTWAKE goes low.\n");
		MODULEWAKE_ACTIVE(pNxp, 0);
	}

	return IRQ_HANDLED;
}

static int nxp5209_open(struct inode *inode, struct file *filp)
{
	int i = iminor(inode);
	int ret = 0;	

	if (nxp5209_miscdev[0].minor==i) /* gsm1 */
		filp->private_data = &nxp5209_table[0];
	else
		filp->private_data = &nxp5209_table[1];
	return ret;
}

static int nxp5209_release( struct inode *inode, struct file *filp )
{
	return 0;
}

#ifdef CONFIG_PM
static int nxp5209_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i;
	for (i=0; i<ARRAY_SIZE(nxp5209_table); i++) {
		struct nxp_5209* pNxp = (struct nxp_5209*)&nxp5209_table[i];
		//YH_PRINTK("inacitve Modem.\n");
		MODULEWAKE_INACTIVE(pNxp, 0);
	}

	return 0;
}

static int nxp5209_resume(struct platform_device *pdev)
{
#if 0 /* active modem @ multi */
	int i;

	for (i=0; i<ARRAY_SIZE(nxp5209_table); i++) {
		struct nxp_5209* pNxp = (struct nxp_5209*)&nxp5209_table[i];
		YH_PRINTK("acitve Modem.\n");
		MODULEWAKE_ACTIVE(pNxp);
	}
#endif
	return 0;
}
#else
#define nxp5209_suspend	NULL
#define nxp5209_resume	NULL
#endif

static struct file_operations nxp5209_fops =  {
	.owner = THIS_MODULE,
	.unlocked_ioctl = nxp5209_ioctl,
	.open = nxp5209_open,
	.release = nxp5209_release,	
};

static int nxp5209_data_init(struct nxp_5209* pNxp)
{
	int ret;

	spin_lock_init(&pNxp->modemwake_lock);
	
	init_timer(&pNxp->modulewake_timer);
	pNxp->modulewake_timer.function = modulewake_timer_handler;
	pNxp->modulewake_timer.data = (unsigned long)pNxp;

	/* init gpio */
	pxa3xx_mfp_set_afds(pNxp->modemWake_gpio, MFP_AF0, MFP_DS03X);
	gpio_direction_output(pNxp->modemWake_gpio, GPIO_LEVEL_HIGH);
	pxa3xx_mfp_set_lpm(pNxp->modemWake_gpio, MFP_LPM_DRIVE_HIGH); /* let it high in sleep */
	/*if the modem want to send data,hostWake_gpio will be set,we will process modemWake_gpio in irq*/

	pxa3xx_mfp_set_afds(pNxp->onKey_gpio, MFP_AF0, MFP_DS03X);
	gpio_direction_output(pNxp->onKey_gpio, GPIO_LEVEL_LOW);
	pxa3xx_mfp_set_lpm(pNxp->onKey_gpio, MFP_LPM_FLOAT); /* let it low in sleep */

	pxa3xx_mfp_set_afds(pNxp->modemRest_gpio, MFP_AF0, MFP_DS03X);
/*	pxa3xx_gpio_set_direction(pNxp->modemRest_gpio, GPIO_DIR_OUT);
	pxa3xx_gpio_set_level(pNxp->modemRest_gpio, GPIO_LEVEL_HIGH);*/
        /*if we set the reset to high,after thw AP wakeup from sleep,the modem will be reset!*/
	gpio_direction_input(pNxp->modemRest_gpio);
	pxa3xx_mfp_set_lpm(pNxp->modemRest_gpio, MFP_LPM_DRIVE_HIGH); /* let it high in sleep */
	/*use reset as input,RSTON is HIGH*/

	if (0==MODULE_INDEX(pNxp)) { /* gsm1 */
		pxa3xx_mfp_set_afds(MFP_PIN_GPIO84, MFP_AF0, MFP_DS03X);
		gpio_direction_output(MFP_PIN_GPIO84, GPIO_LEVEL_LOW);
		/*set the init value for RTS*/
	}
#if defined(CONFIG_SERIAL_GSM2_BT)
	if (1==MODULE_INDEX(pNxp)) { /* gsm2 */
		pxa3xx_mfp_set_afds(GSM2_SERIAL_RTS, MFP_AF0, MFP_DS03X);
		gpio_direction_output(GSM2_SERIAL_RTS, GPIO_LEVEL_LOW);
	}
#endif	

#ifdef CONFIG_PXA3xx_MODEM_DVFM
	if (0==MODULE_INDEX(pNxp))
		dvfm_register("GSM1", &(pNxp->dvfm_dev_idx));
	else if (1==MODULE_INDEX(pNxp))
		dvfm_register("GSM2", &(pNxp->dvfm_dev_idx));
	//YH_PRINTK("dvfm_dev_idx %d\n", pNxp->dvfm_dev_idx);
#endif

	MODULEWAKE_ACTIVE(pNxp, 0);
	
	pxa3xx_mfp_set_afds(pNxp->hostWake_gpio, MFP_AF0, MFP_DS03X);
	gpio_direction_input(pNxp->hostWake_gpio);
	ret = request_irq(IRQ_GPIO(MFP2GPIO(pNxp->hostWake_gpio)), hostwake_irq_handler,
					IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, NXP5209_NAME, pNxp);
	if (ret<0) {
		printk("Request IRQ for NXP5209 failed:%d\n", ret);
		return ret;
	}

	return ret;
}

#if defined(CONFIG_DUAL_MODEM)
static ssize_t gsm2_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct file filp;
	filp.private_data = &nxp5209_table[1];
	nxp5209_ioctl(&filp, IOCTL_POWER_ON, 0);
	return count;
}
static DEVICE_ATTR(gsm2_on, 0644, NULL, gsm2_on_store);
#endif

static ssize_t gsm_state_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	int count = 0;
	int i;
	struct nxp_5209* pNxp;
	
	for (i=0; i<ARRAY_SIZE(nxp5209_table); i++) {
		pNxp = (struct nxp_5209*)&nxp5209_table[i];
		count += sprintf(buf, "Modem %d, actvie %d, onoff %d\n", pNxp->index, pNxp->active, pNxp->onoff);
	}
	
	return count;
}
static DEVICE_ATTR(state, 0644, gsm_state_show, NULL);

static struct attribute *nxp5209_attributes[] = {	
#if defined(CONFIG_DUAL_MODEM)
	&dev_attr_gsm2_on.attr,
#endif
	&dev_attr_state.attr,
	NULL,
};
static struct attribute_group nxp5209_attr_group ={
	.attrs= nxp5209_attributes,
};

static int __devinit nxp5209_probe(struct platform_device *pdev)
{
	int ret;
	int i;
	
	for (i=0; i<ARRAY_SIZE(nxp5209_table); i++) {
		nxp5209_table[i].index = i;
		INIT_WORK(&nxp5209_table[i].modem_detect_work, send_modem_event);
		nxp5209_data_init(&nxp5209_table[i]);		
		
	}	

	for (i=0; i<ARRAY_SIZE(nxp5209_miscdev); i++) {
		ret = misc_register(&nxp5209_miscdev[i]);
		nxp5209_table[i].p_dev = nxp5209_miscdev[i].this_device;
		if (ret<0)	{
			printk("Register device %s fail %d\n", nxp5209_miscdev[i].name, ret);
			return ret;
		}
	}
	
	ret = sysfs_create_group(&pdev->dev.kobj, &nxp5209_attr_group);

	
	printk("NXP5209 Driver registered \n");	
	return 0;
}

static int __devexit nxp5209_remove(struct platform_device *pdev)
{
	unregister_chrdev(NXP5209_MAJOR, NXP5209_NAME);
	return 0;
}
static struct platform_driver nxp5209_driver = {
	.probe		= nxp5209_probe,
	.remove		= __devexit_p(nxp5209_remove),
	.suspend	= nxp5209_suspend,
	.resume		= nxp5209_resume,
	.driver		= {
		.name	= "modem_nxp5209",
	},
};

static int __init nxp5209_init(void)
{
	return platform_driver_register(&nxp5209_driver);
}

static void __exit nxp5209_exit(void)
{
	platform_driver_unregister(&nxp5209_driver);
}

module_init(nxp5209_init);
module_exit(nxp5209_exit);
MODULE_LICENSE("GPL");

