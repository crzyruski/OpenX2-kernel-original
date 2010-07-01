/*
 *  Copyright (C) Yuhua Tel 2009
 *  Interface for lcdbl,kpbl,vibrator etc...
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/errno.h>
#include <asm/current.h>
#include <asm/system.h>
#include <linux/delay.h>

#include <mach/micco.h>
#include <mach/yuhua_board_dev_info.h>
#include <mach/gpio.h>
#include <mach/mfp.h>

#define LCDBL_USER_LEVEL 100

int lcdbl_current_level = (40*LCDBL_USER_LEVEL)/100; /* Default 40% level */

#if defined(CONFIG_FB_LCD_LG_DP4551)
#define LCDBL_MAX 0x3ff
#define LCDBL_MIN 0x100
#else
#define LCDBL_MAX 0x300
#define LCDBL_MIN 0x0
#endif

#if defined(CONFIG_BOARD_LANDMARK) && defined(CONFIG_YUHUA_MISC_DEV_QWERTY_BL) 
/* warkaround for landmark */
#define YUHUA_MSIC_SWITCH_KEY_TO_QWERTY (0x1)
#endif

int boot_animation_stage;

/* Return value: form 0x0 to 0x3FF */
int lcdbl_get_current_brightness(void)
{
	int ret;
	if (lcdbl_current_level)
		ret = LCDBL_MIN + (lcdbl_current_level * (LCDBL_MAX-LCDBL_MIN))/LCDBL_USER_LEVEL;
	else
		ret = 0;
	//printk("lcdbl_get_current_brightness %d\n", ret);
	return ret;
}

extern void pxafb_set_backlight(int on);
int lcdbl_set_level(int level)
{
	if (level>=0&&level<=LCDBL_USER_LEVEL){
		if (level!=lcdbl_current_level) {
			lcdbl_current_level = level;
			pxafb_set_backlight(level);
		}
		return 0;
	}else{
		printk("Set illegal LCDBL_LEVEL[%d]\n",level);
		return -EFAULT;
	}
}

int lcdbl_get_level(void)
{
	return lcdbl_current_level;
}

static int g_kpbl_power = 0;
static int yuhua_misc_kpbl_set(int power)
{
	if(power<0||power>0x7f) 
		return -EINVAL;

	if (power!=g_kpbl_power) {
		micco_write(MICCO_LED1_CONTROL, power);
		g_kpbl_power = power;
	}
	return 0;
}

static struct semaphore vibr_sem;
static int g_vibr_power = 0;
static int yuhua_misc_vibr_set(int power)
{
	if(power<0||power>0x7f) {
		return -EFAULT;
	}

	if (power!=g_vibr_power) {
		micco_write(MICCO_VIBRA_CONTROL, power);
		g_vibr_power = power;
	}
	
	return 0;
}

static int g_qwertybl_power = 0;
#ifdef CONFIG_YUHUA_MISC_DEV_QWERTY_BL
static void yuhua_misc_qwertybl_init(void)
{
	//pxa3xx_mfp_set_afds(QWERTYBL_GPIO, MFP_AF0, MFP_DS04X);
	//gpio_direction_output(mfp_to_gpio(QWERTYBL_GPIO), GPIO_LEVEL_LOW);
}
static int yuhua_misc_qwertybl_set(int power)
{
	if (power!=g_qwertybl_power) {
		if (power) {
			//gpio_direction_output(mfp_to_gpio(QWERTYBL_GPIO), GPIO_LEVEL_HIGH);
		} else {
			//gpio_direction_output(mfp_to_gpio(QWERTYBL_GPIO), GPIO_LEVEL_LOW);
		}
		g_qwertybl_power = power;
	}
	return 0;
}
#else
static inline void yuhua_misc_qwertybl_init(void)
{
}
static inline int yuhua_misc_qwertybl_set(int power)
{
	return 0;
}
#endif

/* vibrator */
struct vibr_one_info {
	int power; /* 0-0x7f */
	int ms;
};

static int yuhua_misc_vibr_one_set(struct vibr_one_info* pInfo)
{	
	int ret = -EIO;
	
	if (pInfo->power<0 || pInfo->power>0x7f) {
		goto out;
	}	

	if (pInfo->ms==0) {
		ret = 0;
	} else if ((pInfo->ms>0) && (pInfo->ms<1000*60*10) && (g_vibr_power<=0)) {
		ret = down_timeout(&vibr_sem, msecs_to_jiffies(pInfo->ms));
		if (ret>=0) {
			yuhua_misc_vibr_set(pInfo->power);
			msleep(pInfo->ms);
			up(&vibr_sem);
		}
	}
	
out:
	if (ret < 0)
		printk(KERN_ERR"vibr, invalid %d or %d %d\n", pInfo->ms, g_vibr_power, pInfo->power);
	yuhua_misc_vibr_set(0);
	return 0;
}

static int yuhua_misc_open(struct inode * dev, struct file *filp)
{
	return 0;
}

static int yuhua_misc_release(struct inode * dev, struct file *filp)
{
	return 0;
}

int yuhua_misc_ioctl (struct inode *inode, struct file *file, unsigned int cmd,
		unsigned long arg)
{
	return 0;
}

/****************************************************************************
 * Initialization / Registeration / Removal
 ***************************************************************************/
static struct file_operations yuhua_misc_fops = {
	.owner		= THIS_MODULE,
	.open		= yuhua_misc_open,
	.release		= yuhua_misc_release,
	.ioctl		= yuhua_misc_ioctl,
};

static struct miscdevice yuhua_misc_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "yuhua_misc",
	.fops	= &yuhua_misc_fops
};

#if defined(CONFIG_BOARD_LANDMARK)
#define MAX_KEYBL (0x7f*100/100) /* 100% */
#elif defined(CONFIG_BOARD_BRAVA)
#define MAX_KEYBL (0x7f*4/100) /* 4% */
#else
#define MAX_KEYBL (0x7f*20/100) /* 20% */
#endif
static ssize_t keybl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{					
	int power;

	if (boot_animation_stage>=0)
		boot_animation_stage = -1; /* close keybl when system boot ok */
	
	sscanf(buf, "%d", &power);	
	if(power>=0 && power<=255) 
		yuhua_misc_kpbl_set(power*(MAX_KEYBL)/255);
	else
		printk("Valid parameter[0-255]\n");

	return count;
}
static ssize_t keybl_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	return sprintf(buf, "%d\n", g_kpbl_power*255/(MAX_KEYBL));
}

static ssize_t qwertybl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{					
	int power;
	
	if (boot_animation_stage>=0)
		boot_animation_stage = -1; /* close keybl when system boot ok */	
	
	sscanf(buf, "%d", &power);
	if(power>=0 && power<=255) 
		yuhua_misc_qwertybl_set(power*(MAX_KEYBL)/255);
	else
		printk("Valid parameter[0-255]\n");
	
	return count;
}
static ssize_t qwertybl_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	return sprintf(buf, "%d\n", g_qwertybl_power*255/(MAX_KEYBL));
}

#ifdef YUHUA_MSIC_SWITCH_KEY_TO_QWERTY
static DEVICE_ATTR(qwertybl, 0644, keybl_show, keybl_store);
static DEVICE_ATTR(keybl, 0644, qwertybl_show, qwertybl_store);
#else
static DEVICE_ATTR(keybl, 0644, keybl_show, keybl_store);
static DEVICE_ATTR(qwertybl, 0644, qwertybl_show, qwertybl_store);
#endif

static ssize_t vibrator_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
#if 0
	int power;
	
	sscanf(buf, "%d", &power);
	if(power>=0 && power<=255) 
		yuhua_misc_vibr_set(power*0x7f/255);
	else
		printk("Valid parameter[0-255]\n");
#else
	struct vibr_one_info info;
	info.power = 0x7f;
	sscanf(buf, "%d", &info.ms);
	//printk(KERN_DEBUG"vibrator_store on: %dms\n", info.ms);
	yuhua_misc_vibr_one_set(&info);
#endif

	return count;
}
static ssize_t vibrator_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	return sprintf(buf, "%d\n", g_vibr_power*255/0x7f);
}
static DEVICE_ATTR(vibrator, 0644, vibrator_show, vibrator_store);

static ssize_t lcdbl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{					
	int power;
	
	sscanf(buf, "%d", &power);
	if(power>=0 && power<=255) 
		lcdbl_set_level(power*LCDBL_USER_LEVEL/255);
	else
		printk("Valid parameter[0-255]\n");

	return count;
}
static ssize_t lcdbl_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	return sprintf(buf, "%d\n", lcdbl_current_level*255/LCDBL_USER_LEVEL);
}
static DEVICE_ATTR(lcdbl, 0644, lcdbl_show, lcdbl_store);

struct yuhua_board_info g_yuhua_board_info;
static ssize_t board_info_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	int count = 0;
#ifdef CONFIG_PXA_CAMERA
	count += sprintf(buf+count, "Camera: %s\n", g_yuhua_board_info.camera_detect?"OK":"Fail");
#endif
#ifdef CONFIG_CI_DUAL_CAMERA
	count += sprintf(buf+count, "Front Camera: %s\n", g_yuhua_board_info.camera2_detect?"OK":"Fail");
#endif
	count += sprintf(buf+count, "Wifi: %s\n", g_yuhua_board_info.wifi_detect?"OK":"Fail");
	count += sprintf(buf+count, "Sdcard: %s, %d M\n", 
		g_yuhua_board_info.sdcard_detect?"OK":"Not connect", g_yuhua_board_info.sdcard_capacity);
	count += sprintf(buf+count, "Fm-Radio: %s\n", g_yuhua_board_info.fm_detect?"OK":"Fail");
	count += sprintf(buf+count, "G-Sensor: %s\n", g_yuhua_board_info.gsensor_detect?"OK":"Fail");
	count += sprintf(buf+count, "Ext-RTC: %s\n", g_yuhua_board_info.rtc_detect?"OK":"Fail");
	count += sprintf(buf+count, "Touch: %s\n", g_yuhua_board_info.touch_detect?"OK":"Fail");
	
	return count;
}
static DEVICE_ATTR(board_info, 0644, board_info_show, NULL);

static ssize_t board_camera_id_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	int count = 0;
	count += sprintf(buf+count, "%s", 
		g_yuhua_board_info.camera_id?g_yuhua_board_info.camera_id:"null");	
	return count;
}
static DEVICE_ATTR(board_camera_id, 0644, board_camera_id_show, NULL);

static struct attribute *yh_misc_attributes[] = {	
	&dev_attr_lcdbl.attr,
	&dev_attr_vibrator.attr,
	&dev_attr_keybl.attr,
	&dev_attr_qwertybl.attr,
	&dev_attr_board_info.attr,
	&dev_attr_board_camera_id.attr,
	NULL,
};
static struct attribute_group yh_misc_attr_group ={
	.attrs = yh_misc_attributes,
};

#ifdef YUHUA_MSIC_SWITCH_KEY_TO_QWERTY
#define boot_animation_step (HZ)
#define boot_animation_max_stage (1)
#else
#define boot_animation_step (HZ)
#define boot_animation_max_stage (3)
#endif
static struct timer_list boot_animation_timer; /* add boot keybl animation */
#define MAX_BOOT_KEYBL (0x7f*50/100) /* 50% */
static void boot_animation_timer_handler(unsigned long data)
{
	int power = 0; /* max: 255 */
	(void)data;

	if (boot_animation_stage>=0) {			
		power = boot_animation_stage*255/boot_animation_max_stage;
			
		mod_timer(&boot_animation_timer, jiffies+boot_animation_step);
		boot_animation_stage++;
		if (boot_animation_stage > boot_animation_max_stage)
			boot_animation_stage = 0;

#ifdef YUHUA_MSIC_SWITCH_KEY_TO_QWERTY
		yuhua_misc_qwertybl_set(power*(MAX_BOOT_KEYBL)/255);
#else
		yuhua_misc_kpbl_set(power*(MAX_BOOT_KEYBL)/255);
#endif
	}
}

static int __init yuhua_misc_init(void)
{
	int result;
	
	result = misc_register(&yuhua_misc_device);
	if (result) {
		printk("Unable to register yuhua misc[%d]\n",result);
		return result;
	}

	result = sysfs_create_group(&yuhua_misc_device.this_device->kobj, &yh_misc_attr_group);

	init_MUTEX(&vibr_sem);

	init_timer(&boot_animation_timer);
	boot_animation_timer.function = boot_animation_timer_handler;
	boot_animation_stage = 0; /* -1: disable boot animation*/
	mod_timer(&boot_animation_timer, jiffies+boot_animation_step);

	yuhua_misc_qwertybl_init();
	
	printk("Register device yuhua misc\n");
	return 0;
}

static void __exit yuhua_misc_exit(void)
{
	misc_deregister(&yuhua_misc_device);
}

module_init(yuhua_misc_init);
module_exit(yuhua_misc_exit);

MODULE_AUTHOR("YUHUA");
MODULE_LICENSE("GPL");

