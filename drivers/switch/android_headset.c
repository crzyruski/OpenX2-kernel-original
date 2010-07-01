/* Copyright (C) 2008 YuhuaTel */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/input.h>

#include <mach/pxa3xx_pmic.h>
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/micco.h>
#include <sound/soc-dapm.h>

extern struct input_dev* pxa_keypad_input_dev;
static struct switch_dev micco_sdev;

#define KEY_ONSWITCH KEY_END

enum headset_detect_stage {
	headset_detect_off,
	headset_detect_auto,
	headset_detect_continuous,
};

#if defined(CONFIG_BOARD_X2)
#define USB_SW_GPIO  (MFP_PIN_GPIO6_2) 
#elif defined(CONFIG_BOARD_BRAVA) || defined(CONFIG_BOARD_X2G)
#define USB_SW_GPIO  (MFP_PIN_GPIO23) 
#endif
static int yuhua_usb_sw(int high)
{
	//printk(KERN_DEBUG"yuhua_usb_sw: %s\n", high?"high":"low");

#ifdef USB_SW_GPIO
	pxa3xx_mfp_set_afds(USB_SW_GPIO, MFP_AF0, MFP_DS04X);
	if (high) {
		gpio_direction_output(USB_SW_GPIO, GPIO_LEVEL_HIGH);
		pxa3xx_mfp_set_lpm(USB_SW_GPIO, MFP_LPM_DRIVE_HIGH); /* let it high in sleep */
	} else {
		gpio_direction_output(USB_SW_GPIO, GPIO_LEVEL_LOW);
		pxa3xx_mfp_set_lpm(USB_SW_GPIO, MFP_LPM_FLOAT);
	}
#endif

	return 0;
}

static int g_headset_detect_stage = -1;
static void micco_headset_detect_stage(int stage)
{
	u8 val = 0;	

	if (g_headset_detect_stage!=stage) {
		printk(KERN_DEBUG"micco_headset_detect_stage %d\n", stage);
		if (headset_detect_auto==stage) { 
			val = MICCO_MISC_REMCON_AUTO | MICCO_MISC_REMCON_FILTER;
			micco_write(MICCO_MISC, val);
		} else if (headset_detect_continuous==stage) {
			val = MICCO_MISC_REMCON_ENABLE| MICCO_MISC_REMCON_FILTER;
			micco_write(MICCO_MISC, val);
		} else {
			micco_write(MICCO_MISC, 0);
		}
		g_headset_detect_stage = stage;
	}
}

static void micco_headset_cb(unsigned long event)
{
	u8 val;	

	if (event & PMIC_EVENT_HSDETECT) {
		micco_read(MICCO_STATUS_B, &val);
		printk(KERN_DEBUG"PMIC_EVENT_HSDETECT 0x%x\n", val);
		if (MICCO_STATUS_B_HEADSET & val) {
			switch_set_state(&micco_sdev, 1);
			yuhua_usb_sw(1); /* high for headset */
			micco_headset_detect_stage(headset_detect_continuous);
		} else {
			switch_set_state(&micco_sdev, 0);
			yuhua_usb_sw(0); /* low for usb */
			//micco_headset_detect_stage(headset_detect_auto);
		}
	}

	if (pxa_keypad_input_dev && (event & PMIC_EVENT_HOOKSWITCH)) {		
		micco_read(MICCO_STATUS_B, &val);
		printk(KERN_DEBUG"PMIC_EVENT_HOOKSWITCH 0x%x\n", val);
		if (MICCO_STATUS_B_HOOKSWITCH & val)
			input_report_key(pxa_keypad_input_dev, KEY_MEDIA, 1);
		else 
			input_report_key(pxa_keypad_input_dev, KEY_MEDIA, 0);
	}

	if (pxa_keypad_input_dev && (event & PMIC_EVENT_ONKEY)) {
		micco_read(MICCO_STATUS_A, &val);
		if (val & (0x1<<0)) /* up */
			input_report_key(pxa_keypad_input_dev, KEY_ONSWITCH, 0);
		else 
			input_report_key(pxa_keypad_input_dev, KEY_ONSWITCH, 1);
	}		
}

static int headset_probe(struct platform_device *pdev)
{
	int ret;
	u8 val;

	micco_sdev.name = "h2w";
	ret = switch_dev_register(&micco_sdev);
	if (ret < 0)
		goto err;

	/* register headset detect, hookswitch and on-key */
	ret = pmic_callback_register(PMIC_EVENT_HSDETECT | PMIC_EVENT_HOOKSWITCH | 
				PMIC_EVENT_ONKEY, micco_headset_cb);

	micco_headset_detect_stage(headset_detect_continuous);

	/* HOOKSWITCH and HEADSET interrupt */
	micco_read(MICCO_IRQ_MASK_D, &val);
	val &= ~0x18;
	micco_write(MICCO_IRQ_MASK_D, val);

	/* ONKEY interrupt */
	micco_read(MICCO_IRQ_MASK_A, &val);
	val &= ~IRQ_MACK_A_ONKEY;
	micco_write(MICCO_IRQ_MASK_A, val);

	/* detect first */
	micco_headset_cb(PMIC_EVENT_HSDETECT);
err:
	printk("Headset driver for android register %s\n", ret<0?"fail":"succ");
	return ret;
}

static int __devexit headset_remove(struct platform_device *pdev)
{
	/* fix me */
	return 0;
}

static int headset_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (soc_get_suspend_bits()) { /* audio channel active */
	} else { /* close */
		micco_headset_detect_stage(headset_detect_off);
	}
	
	return 0;
}

static int headset_resume(struct platform_device *pdev)
{	
	micco_headset_detect_stage(headset_detect_continuous);
	/* micco will triger a event when open hsdetect */
	//micco_headset_cb(PMIC_EVENT_HSDETECT); /* detect again */
	return 0;
}

static struct platform_driver headset_driver = {
	.probe		= headset_probe,
	.remove		= __devexit_p(headset_remove),
	.suspend =	headset_suspend,
	.resume = 	headset_resume,
	.driver		= {
		.name	= "android-headset",
		.owner	= THIS_MODULE,
	},
};

static int __init headset_init(void)
{
	return platform_driver_register(&headset_driver);
}

static void __exit headset_exit(void)
{
	platform_driver_unregister(&headset_driver);
}

module_init(headset_init);
module_exit(headset_exit);

MODULE_DESCRIPTION("Headset driver");
MODULE_LICENSE("GPL");

