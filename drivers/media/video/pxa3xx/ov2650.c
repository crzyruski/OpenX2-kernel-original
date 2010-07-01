/* 
	Camera driver for OV2650 2M
	Copyright@yuhuatel 2009
	Exp by frank.du 
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/rtc.h>
	
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/littleton.h>
#include <mach/micco.h>
#ifdef CONFIG_YUHUA_MISC_DEV	
#include <mach/yuhua_board_dev_info.h>
#endif

#include "ov2650.h"

static struct i2c_client *g_ov2650_client;

#define read_cmos_sensor(reg, pVal) 	i2c_ov2650_read(g_ov2650_client, reg, pVal)
#define write_cmos_sensor(reg, val) 	i2c_ov2650_write(g_ov2650_client, reg, val)
#define write_sccb16(reg, val) 			i2c_ov2650_write(g_ov2650_client, reg, val)
#define write_i2c(reg, val) 				i2c_ov2650_write(g_ov2650_client, reg, val)

static int i2c_ov2650_read(struct i2c_client *client, u16 reg, u8* pVal)
{	
	char cmd[2];
	int ret;

	cmd[0] = reg>>8;
	cmd[1] = reg&0xff;
	
	ret = i2c_master_send(client, cmd, 2);
	if (ret<0)
		goto out;
	ret = i2c_master_recv(client, pVal, 1);
	//*pVal = ret&0xff;
	
out:
	if (ret<0)
		OV2650_err("ret %d\n", ret);
	return ret;
}

static int i2c_ov2650_write(struct i2c_client *client, u16 reg, u8 val)
{
	char cmd[3];
	int ret;

	cmd[0] = reg>>8;
	cmd[1] = reg&0xff;
	cmd[2] =  val;
	
	ret = i2c_master_send(client, cmd, 3);
	if (ret<0)
		OV2650_err("ret %d\n", ret);
	return ret;
}

static int ov2650hw_set_group_reg(u16* pInSettings, int len)
{
	int i, ret = -EIO;
	
	for(i=0; i<len; i+=2) {
		ret = i2c_ov2650_write(g_ov2650_client, pInSettings[i], (u8)pInSettings[i+1]);
		if (ret<0) {
			OV2650_err("at[%d],ret[%d],reg 0x%x,val 0x%x\n", i, ret, pInSettings[i], pInSettings[i+1]);
			break;
		}
	}

	return ret;
}
#define ov2650hw_set_group(settingArray) ov2650hw_set_group_reg(settingArray, ARRAY_SIZE(settingArray))

static void ov2650hw_gpio_init(void)
{
	pxa3xx_mfp_set_afds(CIF_nRESET, MFP_AF0, MFP_DS04X);
	gpio_direction_output(CIF_nRESET, GPIO_LEVEL_LOW);
	gpio_direction_output(CIF_nRESET, MFP_LPM_DRIVE_HIGH);

	pxa3xx_mfp_set_afds(CIF_nPOWERDN, MFP_AF0, MFP_DS04X); 
	gpio_direction_output(CIF_nPOWERDN, GPIO_LEVEL_LOW);
	pxa3xx_mfp_set_lpm(CIF_nPOWERDN, MFP_LPM_DRIVE_HIGH); /* high in sleep */
}

static void ov2650hw_power_on(void)
{
	/* 1.8V, 2.8V, 2.8V ANA */
	micco_enable_LDO13(1);
	micco_enable_LDO15(1);

	gpio_direction_output(CIF_nPOWERDN, GPIO_LEVEL_LOW);
	gpio_direction_output(CIF_nRESET, GPIO_LEVEL_LOW);
	msleep(3);/* reset */
	gpio_direction_output(CIF_nRESET, GPIO_LEVEL_HIGH);
	
	ci_set_clock(1, 1, CIF_MCLK_KHZ);
	msleep(100);
	OV2650_info("succ\n");
}

static void ov2650hw_power_off(void)
{
	ci_set_clock(0, 0, CIF_MCLK_KHZ);

	gpio_direction_output(CIF_nRESET, GPIO_LEVEL_LOW);
	
	/* 1.8V, 2.8V, 2.8V ANA */
	micco_enable_LDO13(0);
	micco_enable_LDO15(0);
	OV2650_info("succ\n");
}

static void ov2650hw_power_up(void)
{
#if 0
	ci_set_clock(1, 1, CIF_MCLK_KHZ);
	msleep(1);// 0.1ms
	gpio_direction_output(CIF_nPOWERDN, GPIO_LEVEL_LOW);
#else
	gpio_direction_output(CIF_nPOWERDN, GPIO_LEVEL_LOW);
	gpio_direction_output(CIF_nRESET, GPIO_LEVEL_LOW);
	msleep(3);/* reset */
	gpio_direction_output(CIF_nRESET, GPIO_LEVEL_HIGH);

	ci_set_clock(1, 1, CIF_MCLK_KHZ);
	msleep(100);
#endif
	OV2650_info("succ\n");
}

static void ov2650hw_power_down(void)
{
	write_sccb16(0x30ab, 0x00);
	write_sccb16(0x30ad, 0x0a);
	write_sccb16(0x30ae, 0x27);
	write_sccb16(0x363b, 0x01);
	gpio_direction_output(CIF_nPOWERDN, GPIO_LEVEL_HIGH);
	msleep(1);
	OV2650_info("succ\n");
	ci_set_clock(0, 0, CIF_MCLK_KHZ);
}

static void ov2650hw_init(void)
{	
	//Soft Reset
	write_cmos_sensor(0x3012 ,0x80);
	//Add some delay after register reset
	msleep(5);
	
	write_cmos_sensor(0x308c ,0x80);	
	write_cmos_sensor(0x308d ,0x0e);
	write_cmos_sensor(0x360b ,0x00);
	write_cmos_sensor(0x30b0 ,0xff);
	write_cmos_sensor(0x30b1 ,0xff);
	write_cmos_sensor(0x30b2 ,0x24);
	write_cmos_sensor(0x300e ,0x34);
	write_cmos_sensor(0x300f ,0xa6);
	write_cmos_sensor(0x3010 ,0x81);
	write_cmos_sensor(0x3082 ,0x01);
	write_cmos_sensor(0x30f4 ,0x01);
	write_cmos_sensor(0x3090 ,0x08); //0x33  // no mirror and flip
	write_cmos_sensor(0x3091 ,0xc0);
	write_cmos_sensor(0x30ac ,0x42);
	write_cmos_sensor(0x30d1 ,0x08);
	write_cmos_sensor(0x30a8 ,0x56);
	write_cmos_sensor(0x3015 ,0x02);//0x03
	write_cmos_sensor(0x3093 ,0x00);
	write_cmos_sensor(0x307e ,0xe5);
	write_cmos_sensor(0x3079 ,0x00);
	write_cmos_sensor(0x30aa ,0x42);
	write_cmos_sensor(0x3017 ,0x40);
	write_cmos_sensor(0x30f3 ,0x82);
	write_cmos_sensor(0x306a ,0x0c);
	write_cmos_sensor(0x306d ,0x00);
	write_cmos_sensor(0x336a ,0x3c);
	write_cmos_sensor(0x3076 ,0x6a);
	write_cmos_sensor(0x30d9 ,0x8c);
	write_cmos_sensor(0x3016 ,0x82);
	write_cmos_sensor(0x3601 ,0x30);
	write_cmos_sensor(0x304e ,0x88);
	write_cmos_sensor(0x30f1 ,0x82);
	write_cmos_sensor(0x306f ,0x14);
	write_cmos_sensor(0x302a ,0x02);
	write_cmos_sensor(0x302b ,0x6a);
	write_cmos_sensor(0x3012 ,0x10);
	write_cmos_sensor(0x3011 ,0x01);
	//AEC/AGC
	write_cmos_sensor(0x3013 ,0xf7);
	write_cmos_sensor(0x301c ,0x13);
	write_cmos_sensor(0x301d ,0x17);
	write_cmos_sensor(0x3070 ,0x5d);
	write_cmos_sensor(0x3072 ,0x4d);
	
	write_cmos_sensor(0x3018,0x82); //high limit //AE target和閃爍有關
	write_cmos_sensor(0x3019,0x72); //low limit 
	write_cmos_sensor(0x301a,0xa3); //0xa3 by Joe 0805 //AE step 
	
	//D5060
	write_cmos_sensor(0x30af ,0x00);
	write_cmos_sensor(0x3048 ,0x1f);
	write_cmos_sensor(0x3049 ,0x4e);
	write_cmos_sensor(0x304a ,0x20);
	write_cmos_sensor(0x304f ,0x20);
	write_cmos_sensor(0x304b ,0x02);
	write_cmos_sensor(0x304c ,0x00);
	write_cmos_sensor(0x304d ,0x02);
	write_cmos_sensor(0x304f ,0x20);
	write_cmos_sensor(0x30a3 ,0x10);
	write_cmos_sensor(0x3013 ,0xf7);
	write_cmos_sensor(0x3014 ,0x80);
	
	write_cmos_sensor(0x3071 ,0x00);
	write_cmos_sensor(0x3070 ,0x5d);
	
	write_cmos_sensor(0x3073 ,0x00);
	write_cmos_sensor(0x3072 ,0x4d);
	
	write_cmos_sensor(0x301c ,0x05);	// 0x05 danghui banding
	write_cmos_sensor(0x301d ,0x06);	// 0x06 danghui banding
	
	write_cmos_sensor(0x304d ,0x42);
	write_cmos_sensor(0x304a ,0x40);
	write_cmos_sensor(0x304f ,0x40);
	write_cmos_sensor(0x3095 ,0x07);
	write_cmos_sensor(0x3096 ,0x16);
	write_cmos_sensor(0x3097 ,0x1d);
	//Window Setup
	
#if 0	// VGA preview	
	write_cmos_sensor(0x300e ,0x38);
	write_cmos_sensor(0x3020 ,0x01);
	write_cmos_sensor(0x3021 ,0x18);
	write_cmos_sensor(0x3022 ,0x00);
	write_cmos_sensor(0x3023 ,0x06);
	write_cmos_sensor(0x3024 ,0x06);
	write_cmos_sensor(0x3025 ,0x58);
	write_cmos_sensor(0x3026 ,0x02);
	write_cmos_sensor(0x3027 ,0x61);
	write_cmos_sensor(0x3088 ,0x02); //SVGA 0x03
	write_cmos_sensor(0x3089 ,0x80);//SVGA 0x20
	write_cmos_sensor(0x308a ,0x01);//SVGA 0x02
	write_cmos_sensor(0x308b ,0xe0);//SVGA 0x58
	write_cmos_sensor(0x3316 ,0x64);
	write_cmos_sensor(0x3317 ,0x25);
	write_cmos_sensor(0x3318 ,0x80);
	write_cmos_sensor(0x3319 ,0x08);
	write_cmos_sensor(0x331a ,0x28);//SVGA 0x64
	write_cmos_sensor(0x331b ,0x1e);//SVGA 0x4b
	write_cmos_sensor(0x331c ,0x00);//SVGA 0x00
	write_cmos_sensor(0x331d ,0x38);
	write_cmos_sensor(0x3100 ,0x00);
#else
	// SVGA preview
	write_cmos_sensor(0x300e ,0x34);
	write_cmos_sensor(0x3020 ,0x01);
	write_cmos_sensor(0x3021 ,0x18);
	write_cmos_sensor(0x3022 ,0x00);
	write_cmos_sensor(0x3023 ,0x06);
	write_cmos_sensor(0x3024 ,0x06);
	write_cmos_sensor(0x3025 ,0x58);
	write_cmos_sensor(0x3026 ,0x02);
	write_cmos_sensor(0x3027 ,0x61);
	write_cmos_sensor(0x3088 ,0x03); //SVGA 0x03
	write_cmos_sensor(0x3089 ,0x20);//SVGA 0x20
	write_cmos_sensor(0x308a ,0x02);//SVGA 0x02
	write_cmos_sensor(0x308b ,0x58);//SVGA 0x58
	write_cmos_sensor(0x3316 ,0x64);
	write_cmos_sensor(0x3317 ,0x25);
	write_cmos_sensor(0x3318 ,0x80);
	write_cmos_sensor(0x3319 ,0x08);
	write_cmos_sensor(0x331a ,0x64);//SVGA 0x64
	write_cmos_sensor(0x331b ,0x4b);//SVGA 0x4b
	write_cmos_sensor(0x331c ,0x00);//SVGA 0x00
	write_cmos_sensor(0x331d ,0x38);
	write_cmos_sensor(0x3100 ,0x00);
#endif
	
	//AWB
	write_cmos_sensor(0x3320 ,0xfa);
	write_cmos_sensor(0x3321 ,0x11);
	write_cmos_sensor(0x3322 ,0x92);
	write_cmos_sensor(0x3323 ,0x01);
	write_cmos_sensor(0x3324 ,0x97);
	write_cmos_sensor(0x3325 ,0x02);
	write_cmos_sensor(0x3326 ,0xff);
	write_cmos_sensor(0x3327 ,0x0c);
	write_cmos_sensor(0x3328 ,0x10);
	write_cmos_sensor(0x3329 ,0x10);
	write_cmos_sensor(0x332a ,0x58);
	write_cmos_sensor(0x332b ,0x50);
	write_cmos_sensor(0x332c ,0xbe);
	write_cmos_sensor(0x332d ,0xe1);
	write_cmos_sensor(0x332e ,0x43);
	write_cmos_sensor(0x332f ,0x36);
	write_cmos_sensor(0x3330 ,0x4d);
	write_cmos_sensor(0x3331 ,0x44);
	write_cmos_sensor(0x3332 ,0xf8);
	write_cmos_sensor(0x3333 ,0x0a);
	write_cmos_sensor(0x3334 ,0xf0);
	write_cmos_sensor(0x3335 ,0xf0);
	write_cmos_sensor(0x3336 ,0xf0);
	write_cmos_sensor(0x3337 ,0x40);
	write_cmos_sensor(0x3338 ,0x40);
	write_cmos_sensor(0x3339 ,0x40);
	write_cmos_sensor(0x333a ,0x00);
	write_cmos_sensor(0x333b ,0x00);
	//Color Matrix
#if 0
	write_cmos_sensor(0x3380 ,0x28);
	write_cmos_sensor(0x3381 ,0x48);
	write_cmos_sensor(0x3382 ,0x10);
	write_cmos_sensor(0x3383 ,0x23);
	write_cmos_sensor(0x3384 ,0xc0);
	write_cmos_sensor(0x3385 ,0xe5);
	write_cmos_sensor(0x3386 ,0xc2);
	write_cmos_sensor(0x3387 ,0xb3);
	write_cmos_sensor(0x3388 ,0x0e);
	write_cmos_sensor(0x3389 ,0x98);
	write_cmos_sensor(0x338a ,0x01);
#else
	//from foxcon
	write_cmos_sensor(0x3380,0x2d); //28->2d
	write_cmos_sensor(0x3381,0x4d); //48->4d
	write_cmos_sensor(0x3382,0x13); //10->13 0819 Joe for 亮度
	write_cmos_sensor(0x3383,0x2f);  
	write_cmos_sensor(0x3384,0x8c);  
	write_cmos_sensor(0x3385,0xbb);  
	write_cmos_sensor(0x3386,0x9c);  
	write_cmos_sensor(0x3387,0x94);  
	write_cmos_sensor(0x3388,0x05);  
	write_cmos_sensor(0x3389,0x98);
	write_cmos_sensor(0x338a,0x01);
#endif
	//Gamma 
	write_cmos_sensor(0x3340 ,0x0e);//08
	write_cmos_sensor(0x3341 ,0x1a);//16	!!!
	write_cmos_sensor(0x3342 ,0x31);//2f
	write_cmos_sensor(0x3343 ,0x45);//45
	write_cmos_sensor(0x3344 ,0x5a);//55
	write_cmos_sensor(0x3345 ,0x69);//65
	write_cmos_sensor(0x3346 ,0x75);//70
	write_cmos_sensor(0x3347 ,0x7e);//7c
	write_cmos_sensor(0x3348 ,0x88);//86
	write_cmos_sensor(0x3349 ,0x96);//96
	write_cmos_sensor(0x334a ,0xa3);
	write_cmos_sensor(0x334b ,0xaf);
	write_cmos_sensor(0x334c ,0xc4);
	write_cmos_sensor(0x334d ,0xd7);
	write_cmos_sensor(0x334e ,0xe8);
	write_cmos_sensor(0x334f ,0x20);
	
	write_cmos_sensor(0x309d ,0x95);
	
	//Lens correction
#if 0
	write_cmos_sensor(0x3350 ,0x32);
	write_cmos_sensor(0x3351 ,0x25);
	write_cmos_sensor(0x3352 ,0x80);
	write_cmos_sensor(0x3353 ,0x1e);
	write_cmos_sensor(0x3354 ,0x00);
	write_cmos_sensor(0x3355 ,0x85);
	write_cmos_sensor(0x3356 ,0x32);
	write_cmos_sensor(0x3357 ,0x25);
	write_cmos_sensor(0x3358 ,0x80);
	write_cmos_sensor(0x3359 ,0x1b);
	write_cmos_sensor(0x335a ,0x00);
	write_cmos_sensor(0x335b ,0x85);
	write_cmos_sensor(0x335c ,0x32);
	write_cmos_sensor(0x335d ,0x25);
	write_cmos_sensor(0x335e ,0x80);
	write_cmos_sensor(0x335f ,0x1b);
	write_cmos_sensor(0x3360 ,0x00);
	write_cmos_sensor(0x3361 ,0x85);
	write_cmos_sensor(0x3363 ,0x70);
	write_cmos_sensor(0x3364 ,0x7f);
	write_cmos_sensor(0x3365 ,0x00);
	write_cmos_sensor(0x3366 ,0x00);
#else
	// from darling
	write_cmos_sensor(0x3350,0x33);
	write_cmos_sensor(0x3351,0x28);
	write_cmos_sensor(0x3352,0x0 );
	write_cmos_sensor(0x3353,0x16);
	write_cmos_sensor(0x3354,0x0 );
	write_cmos_sensor(0x3355,0x85);
	write_cmos_sensor(0x3356,0x35);
	write_cmos_sensor(0x3357,0x28);
	write_cmos_sensor(0x3358,0x0 );
	write_cmos_sensor(0x3359,0x13);
	write_cmos_sensor(0x335a,0x0 );
	write_cmos_sensor(0x335b,0x85);
	write_cmos_sensor(0x335c,0x34);
	write_cmos_sensor(0x335d,0x28);
	write_cmos_sensor(0x335e,0x0 );
	write_cmos_sensor(0x335f,0x13);
	write_cmos_sensor(0x3360,0x0 );
	write_cmos_sensor(0x3361,0x85);
	write_cmos_sensor(0x3363,0x70);
	write_cmos_sensor(0x3364,0x7f);
	write_cmos_sensor(0x3365,0x0 );
	write_cmos_sensor(0x3366,0x0 );	
#endif
	//UVadjust
	write_cmos_sensor(0x3301 ,0xff);
	write_cmos_sensor(0x338b ,0x11);//0x11
	write_cmos_sensor(0x338c ,0x11);	//0x10
	write_cmos_sensor(0x338d ,0x40);
	//Sharpness/De-noise
	write_cmos_sensor(0x3370 ,0xd0);
	write_cmos_sensor(0x3371 ,0x00);
	write_cmos_sensor(0x3372 ,0x00);
	write_cmos_sensor(0x3373 ,0x40);
	write_cmos_sensor(0x3374 ,0x10);
	write_cmos_sensor(0x3375 ,0x10);
	write_cmos_sensor(0x3376 ,0x04);
	write_cmos_sensor(0x3377 ,0x00);
	write_cmos_sensor(0x3378 ,0x04);
	write_cmos_sensor(0x3379 ,0x80);
	//BLC
	write_cmos_sensor(0x3069 ,0x84);
	write_cmos_sensor(0x307c ,0x01); // no mirror and flip
	write_cmos_sensor(0x3087 ,0x02);
	//Other functions
	write_cmos_sensor(0x3300 ,0xfc);
	write_cmos_sensor(0x3302 ,0x11);
	write_cmos_sensor(0x3400 ,0x02); //CbYCrY
	write_cmos_sensor(0x3606 ,0x20);
	write_cmos_sensor(0x3601 ,0x30);
	write_cmos_sensor(0x300e ,0x34);
	write_cmos_sensor(0x3011 ,0x01);
	write_cmos_sensor(0x30f3 ,0x83);
	write_cmos_sensor(0x304e ,0x88);
	write_cmos_sensor(0x3086 ,0x0f);
	write_cmos_sensor(0x3086 ,0x00);
	OV2650_info("succ\n");
}

void ov2650hw_power_mode(u8 power_mode)
{
	if (power_mode == CAMERA_POWER_OFF ) {
		ov2650hw_power_down();
	} else {
		ov2650hw_power_up();
	}
}

//15fps ~ 5fps night mode for 60/50Hz light environment, 24Mhz clock input,18Mhz pclk
static u16 ov2650_night_mode_on_settings[] = {
	0x300e, 0x34,
	0x3011, 0x01,
	0x302c, 0x00,
	0x3071, 0x00,
	0x3070, 0x5d,
	0x301c, 0x05,
	0x3073, 0x00,
	0x3072, 0x4d,
	0x301d, 0x07,
	0x3014, 0x0c,
	0x3015, 0x50,
	0x302e, 0x00,
	0x302d, 0x00,
};

static u16 ov2650_night_mode_off_settings[] = {
	0x3014, 0x04,
	0x3015, 0x00,
	0x302e, 0x00,
	0x302d, 0x00,
};

static int ov2650hw_set_mode(u32 mode,u32 value)
{	
	int ret = 0;
	OV2650_info("0x%x 0x%x\n", mode, value);
	
	switch(mode){
	case V4L2_CID_DO_WHITE_BALANCE:
		if (0==value) { //Auto 
			write_i2c(0x3306, 0x00);//AWB auto, bit[1]:0,auto
		} else if (1==value) { //Sunny
			write_i2c(0x3306, 0x02); //AWB off
			write_i2c(0x3337, 0x5e);
			write_i2c(0x3338, 0x40);
			write_i2c(0x3339, 0x46);
		} else if (2==value) { //Cloudy
			write_i2c(0x3306, 0x082);
			write_i2c(0x3337, 0x68);
			write_i2c(0x3338, 0x40);
			write_i2c(0x3339, 0x4e);
		} else if (3==value) { //Office
			write_i2c(0x3306, 0x02);
			write_i2c(0x3337, 0x52);
			write_i2c(0x3338, 0x40);
			write_i2c(0x3339, 0x58);
		} else if (4==value) { //Home
			write_i2c(0x3306, 0x02);
			write_i2c(0x3337, 0x44);
			write_i2c(0x3338, 0x40);
			write_i2c(0x3339, 0x70);
		}
		break;
	case V4L2_CID_BRIGHTNESS:
		if (0==value) { //Brightness +2
			write_i2c(0x3391, 0x04);
			write_i2c(0x3390, 0x41);
			write_i2c(0x339a, 0x20);
		} else if (1==value) { //Brightness +1
			write_i2c(0x3391, 0x04);
			write_i2c(0x3390, 0x41);
			write_i2c(0x339a, 0x10);
		} else if (2==value) { //Brightness 0
			write_i2c(0x3391, 0x00);
			write_i2c(0x3390, 0x41);
			write_i2c(0x339a, 0x00);
		} else if (3==value) { //Brightness -1
			write_i2c(0x3391, 0x04);
			write_i2c(0x3390, 0x49);
			write_i2c(0x339a, 0x10);
		} else if (4==value) { //Brightness -2
			write_i2c(0x3391, 0x04);
			write_i2c(0x3390, 0x49);
			write_i2c(0x339a, 0x20);
		}
		break;
	case V4L2_CID_FILTER:
		if (0==value) { //Normal 
			write_i2c(0x3391, 0x00);			
		} else if (1==value) { //B&W
			write_i2c(0x3391, 0x20);
		} else if (2==value) { //Negative
			write_i2c(0x3391, 0x40); //bit[6] negative
		} else if (3==value) { //Sepia(antique)
			write_i2c(0x3391, 0x18);	
		}
		break;
	case V4L2_CID_ANTIFLICKER:
		break;
	default:
		ret =  -EINVAL;
	}

	if (ret<0)
		OV2650_err("mode 0x%x, value 0x%x, ret %d\n", mode, value, ret);
	return 0;
}

static u16 ov2650_1600_1200_settings[] = {
	0x300e, 0x34,
	0x3011, 0x01,
	0x3012, 0x00,
	0x302A, 0x04,
	0x302B, 0xd4,
	0x306f, 0x54,
	
	0x3020, 0x01,
	0x3021, 0x18,
	0x3022, 0x00,
	0x3023, 0x0a,
	0x3024, 0x06,
	0x3025, 0x58,
	0x3026, 0x04,
	0x3027, 0xbc,
	0x3088, 0x06,
	0x3089, 0x40,
	0x308a, 0x04,
	0x308b, 0xb0,
	0x3316, 0x64,
	
	0x3317, 0x4b,
	0x3318, 0x00,
	0x3319, 0x2c,
	0x331a, 0x64,
	0x331b, 0x4b,
	0x331c, 0x00,
	0x331d, 0x4c,
	0x3302, 0x01,
};

static u16 ov2650_1280_960_settings[] = {
	0x300e, 0x34,
	0x3011, 0x01,
	0x3012, 0x00,
	0x302A, 0x04,
	0x302B, 0xd4,
	0x306f, 0x54,	
	0x3020, 0x01,
	0x3021, 0x18,
	0x3022, 0x00,
	0x3023, 0x0a,
	0x3024, 0x06,
	0x3025, 0x58,
	0x3026, 0x04,
	0x3027, 0xbc,
	0x3316, 0x64,	
	0x3317, 0x4b,
	0x3318, 0x00,
	0x3319, 0x2c,
	0x331d, 0x4c,

	0x3302, 0x11,
	0x3088, 0x05,
	0x3089, 0x00,
	0x308a, 0x03,
	0x308b, 0xc0,
	0x331a, 0x50,
	0x331b, 0x3c,
	0x331c, 0x00,
};

static u16 ov2650_800_600_settings[] = {
	0x300e, 0x34,
	0x3011, 0x01,
	0x3012, 0x10,
	0x302A, 0x02,
	0x302B, 0x6a,
	0x306f, 0x14,
	0x3020, 0x01,
	0x3021, 0x18,
	0x3022, 0x00,
	0x3023, 0x06,
	0x3024, 0x06,
	0x3025, 0x58,
	0x3026, 0x02,
	0x3027, 0x61,
	0x3088, 0x03,
	0x3089, 0x20,
	0x308a, 0x02,
	0x308b, 0x58,
	0x3316, 0x64,
	0x3317, 0x25,
	0x3318, 0x80,
	0x3319, 0x08,
	0x331a, 0x64,
	0x331b, 0x4b,
	0x331c, 0x00,
	0x331d, 0x38,
	0x3302, 0x11,
};

static u16 ov2650_640_480_settings[] = {
	0x3302, 0x11,
	0x3088, 0x02,
	0x3089, 0x80,
	0x308a, 0x01,
	0x308b, 0xe0,
	0x331a, 0x28,
	0x331b, 0x1e,
	0x331c, 0x00,
};

static int ov2650hw_set_format(OV2650_size size, int mode)
{
	int ret = -EIO;
	
	switch (size) {
	case ov2650_size_1600_1200:
		ret = ov2650hw_set_group(ov2650_1600_1200_settings);
		break;
	case ov2650_size_800_600:
		ret = ov2650hw_set_group(ov2650_800_600_settings);
		break;
	case ov2650_size_1280_960:
		ret = ov2650hw_set_group(ov2650_1280_960_settings);
		break;
	case ov2650_size_640_480:
		ret = ov2650hw_set_group(ov2650_640_480_settings);
		break;
	default:
		break;
	}

	return ret;
}

int ov2650_init(p_camera_context_t camera_context)
{	
	/* Configure master parallel with 8 data pins */
	ci_set_mode(CI_MODE_MP, CI_DATA_WIDTH8);

	/* data sample on falling and h,vsync active high */
	ci_set_polarity(0, 0, 0);

	/* fifo control */
	ci_set_fifo(0, CI_FIFO_THL_32, 1, 1);

	/* set black level */
	ci_cgu_set_black_level(0);

	/* CGU Mux Select */
	ci_cgu_set_addr_mux_select(CI_CGU_MUX_0_TO_7);

	/* Sensor Power on */
	ov2650hw_power_mode(CAMERA_POWER_FULL);

	/* Set initial code */
	ov2650hw_init();
	
	return 0;
}

int ov2650_deinit(p_camera_context_t camera_context)
{
	ov2650hw_power_mode(CAMERA_POWER_OFF);
	return 0;
}

int ov2650_sleep(p_camera_context_t camera_context)
{
	return ov2650_deinit(camera_context);
}

int ov2650_wake(p_camera_context_t camera_context)
{
	return ov2650_init(camera_context);
}

int ov2650_set_capture_format(p_camera_context_t camera_context )
{
	CI_MP_TIMING timing;
	OV2650_size size;
	
	OV2650_info("in %d out %d w %d h %d\n", camera_context->capture_input_format, 
		camera_context->capture_output_format, camera_context->capture_input_width,
		camera_context->capture_input_height);
	
	if (V4L2_PIX_FMT_YUV422P!=camera_context->capture_input_format) {
		OV2650_err("%d.\n", camera_context->capture_input_format);
		return -EINVAL;	
	}

	if (SENSOR_CHECK_WH(camera_context, 800, 600))
		size = ov2650_size_800_600;
	else if (SENSOR_CHECK_WH(camera_context, 1600, 1200))
		size = ov2650_size_1600_1200;
	else if (SENSOR_CHECK_WH(camera_context, 1280, 960))
		size = ov2650_size_1280_960;
	else if (SENSOR_CHECK_WH(camera_context, 640, 480))
		size = ov2650_size_640_480;
	else	{
		OV2650_err("size %d %d not support\n", 
			camera_context->capture_input_width, camera_context->capture_input_height);
		return -EINVAL;
	}

	ov2650hw_set_format(size, camera_context->capture_mode);

	/* set capture width/height and timing */
	timing.BFW = 0x0;
	timing.BLW = 0x0;
	ci_configure_mp(camera_context->capture_input_width-1,
			camera_context->capture_input_height-1, &timing);
#if defined(CONFIG_CPU_PXA310)
	ci_set_ycbcr_420_down_sample (camera_context->ycbcr_ds);
#endif

	return 0;
}

int ov2650_start_capture(p_camera_context_t camera_context,
		unsigned int frames)
{
	return 0;
}

int ov2650_stop_capture(  p_camera_context_t camera_context )
{
	return 0;
}

int ov2650_set_power_mode(p_camera_context_t camera_context, u8 mode)
{
	ov2650hw_power_mode(mode);
	return 0;
}

int ov2650_set_mode( p_camera_context_t camera_context,
		u32 mode, u32 value)
{
	return ov2650hw_set_mode(mode,value);
}


int ov2650_read_reg( p_camera_context_t camera_context,u32 reg_addr, u32* reg_val)
{
	return i2c_ov2650_read(g_ov2650_client, reg_addr, reg_val);
}

int ov2650_write_reg( p_camera_context_t camera_context,u32 reg_addr, u32 reg_val)
{
	return i2c_ov2650_write(g_ov2650_client, reg_addr, reg_val);
}

static int ov2650_format_list[] = {
	V4L2_PIX_FMT_YUV422P,
	-1	
};
static p_camera_function_t ov2650_functions;
static int ov2650_register(void)
{
	ov2650_functions = kzalloc(sizeof(camera_function_t), GFP_KERNEL);
	if (!ov2650_functions) {
		OV2650_err("fail\n");
		return -ENOMEM;
	}

	ov2650_functions->format_list = ov2650_format_list;	
	ov2650_functions->init = ov2650_init;     	
	ov2650_functions->deinit = ov2650_deinit;     
	ov2650_functions->set_capture_format = ov2650_set_capture_format; 	
	ov2650_functions->start_capture =	ov2650_start_capture;	 		
	ov2650_functions->stop_capture =	ov2650_stop_capture; 			
	ov2650_functions->sleep = ov2650_sleep;	
	ov2650_functions->wakeup = ov2650_wake;	
	ov2650_functions->read_reg = ov2650_read_reg;			
	ov2650_functions->write_reg = ov2650_write_reg;
	ov2650_functions->set_power_mode = ov2650_set_power_mode;			
	ov2650_functions->set_mode =	ov2650_set_mode;
	ov2650_functions->name = "ov2650";
	ov2650_functions->id = 0;

	if (sensor_register(ov2650_functions, OV2650_ID) < 0) {
		OV2650_err("sensor_register failed !\n");
		kfree(ov2650_functions);
		ov2650_functions = NULL;
		return -EFAULT;
	}
	
	return 0;
}

static int ov2650_unregister(void)
{
	sensor_unregister(OV2650_ID);
	
	if (ov2650_functions) {
		kfree(ov2650_functions);
		ov2650_functions = NULL;
	}
	return 0;
}



static int __devinit ov2650_probe(struct i2c_client *client, const struct i2c_device_id * id)
{
	int ret = -EIO;
	struct clk *clk;
	u8 val = 0;
	OV2650_dbg("\n");
	g_ov2650_client = client;	

	ov2650hw_gpio_init();
	clk = clk_get(NULL, "CAMCLK");
	if (!clk) {
		OV2650_err("failed to get camera clock\n");
		return -EIO;
	}
	clk_enable(clk);

	ov2650hw_power_on();
	ret = read_cmos_sensor(0x3012, &val);
	if (ret>=0) {
		//ov2650hw_init();
		ov2650hw_power_down();
	}
	//ov2650hw_power_off();

	clk_disable(clk);
	clk_put(clk);

	if (ret>=0) {
		ret = ov2650_register();
	}

#ifdef CONFIG_YUHUA_MISC_DEV
	if (ret>=0)
		set_camera_detect(1);
#endif
	
	printk("OV2650 camera register %s 0x%x\n", ret>=0?"succ":"fail", val);
	return ret;
}

static int ov2650_remove(struct i2c_client *client)
{	
	ov2650_unregister();
	g_ov2650_client = NULL;
	return 0;
}

static const struct i2c_device_id ov2650_id[] = {
	{ "OV2650", 0 },
	{ }
};

static struct i2c_driver ov2650_driver = {
	.driver = {
		.name	= "OV2650",
	},
	.id_table	= ov2650_id,
	.probe		= ov2650_probe,
	.remove		= ov2650_remove,
};

static int __init i2c_ov2650_init(void)
{
	return i2c_add_driver(&ov2650_driver);
}
static void __exit i2c_ov2650_exit(void)
{
	i2c_del_driver(&ov2650_driver);
}
MODULE_DESCRIPTION("OV2650 2M camera driver");
module_init(i2c_ov2650_init);
module_exit(i2c_ov2650_exit);

