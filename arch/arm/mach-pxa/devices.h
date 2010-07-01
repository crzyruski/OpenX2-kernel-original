#ifndef __PXA_DEVICES_H
#define __PXA_DEVICES_H
extern struct platform_device pxa_device_mci;
extern struct platform_device pxa3xx_device_mci2;
extern struct platform_device pxa3xx_device_mci3;
extern struct platform_device pxa9xx_device_mci3;
extern struct platform_device pxa9xx_device_mci4;
extern struct platform_device pxa9xx_device_mci5;
extern struct platform_device pxa_device_udc;
extern struct platform_device pxa_device_fb;
extern struct platform_device pxa_device_v4l2ov2;
extern struct platform_device pxa_device_ffuart;
extern struct platform_device pxa_device_btuart;
extern struct platform_device pxa_device_stuart;
extern struct platform_device pxa_device_hwuart;
extern struct platform_device pxa_device_i2c;
extern struct platform_device pxa_device_i2s;
extern struct platform_device pxa_device_ficp;
extern struct platform_device pxa_device_rtc;
extern struct platform_device pxa_device_ac97;

extern struct platform_device pxa27x_device_i2c_power;
extern struct platform_device pxa27x_device_ohci;
extern struct platform_device pxa27x_device_udc;
extern struct platform_device pxa27x_device_keypad;

extern struct platform_device pxa25x_device_ssp;
extern struct platform_device pxa25x_device_nssp;
extern struct platform_device pxa25x_device_assp;
extern struct platform_device pxa27x_device_ssp1;
extern struct platform_device pxa27x_device_ssp2;
extern struct platform_device pxa27x_device_ssp3;
extern struct platform_device pxa3xx_device_nand;
extern struct platform_device pxa3xx_device_onenand;

extern struct platform_device pxa25x_device_pwm0;
extern struct platform_device pxa25x_device_pwm1;
extern struct platform_device pxa27x_device_pwm0;
extern struct platform_device pxa27x_device_pwm1;
extern struct platform_device pxa3xx_device_i2c_power;

extern struct platform_device pxa3xx_device_ssp4;
extern struct platform_device pxa3xx_device_rtc;
extern struct platform_device pxa3xx_device_1wire;
extern struct platform_device pxa3xx_device_gcu;
extern struct platform_device pxa3xx_device_u2d;
extern struct platform_device pxa3xx_device_otg;

extern struct platform_device pxa3xx_device_cam;
extern struct platform_device pxa3xx_device_freq;
extern struct platform_device pxa930_device_trkball;

extern struct platform_device pxa3xx_device_mved;
#ifdef CONFIG_USB_ANDROID
extern struct platform_device android_usb_device;
#endif

void __init pxa_register_device(struct platform_device *dev, void *data);
/*
 * Tavor PV series processors device configuration
 *                    U2O/U2H  U2D  UDC/UHC  MMC3-5  MMC1/2  MVED
 *  P                    N      Y      Y        N      Y      N
 *  PV-1.1/P65e1.1       N	N      Y        N      Y      N
 *  PV-/P65e (PXA935)    Y      N      N        N      Y      N
 *  PV (PXA940)          Y      N      N        Y      N      Y
 */
enum device_info {
	device_u2d = (1<<2),		/* Old USB 2.0 client */
	device_uhc = (1<<3),		/* Old USB 1.1 host */
	device_udc = (1<<4),		/* Old USB 1.1 client */
	device_u2o = (1<<5),		/* Chipidea USB 2.0 OTG controller */
	device_u2h = (1<<6),		/* Chipidea USB 2.0 Host */
	device_mmc_new = (1<<9),		/* New MMC controller */	
	device_mved = (1<<13),		/* MVED */

	device_mmc = (1<<31),		/* Old MMC controller */
};
extern int device_is_enabled(int device);

#endif /* __PXA_DEVICES_H */
