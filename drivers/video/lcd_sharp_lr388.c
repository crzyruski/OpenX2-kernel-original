#include <linux/init.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <mach/pxa-regs.h>
#include <mach/pxafb.h>
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/ssp.h>
#include <mach/micco.h>

static struct ssp_dev ssp2;

#if defined(CONFIG_BOARD_X2_V4)
#define LR388_LCD_RESET 	MFP_PIN_GPIO28 //reset gpio
#else
#define LR388_LCD_RESET 	MFP_PIN_GPIO71 //reset gpio
#endif

static void Lr388_lcd_ssp_init(void);
static void Lr388_lcd_ssp_deinit(void);
extern void micco_enable_LDO3(int enable);

static void LCD_CtrlWrite_lr388(u8 index, u8 data)
{
	u16 command = ((index&0xFF)<<8) | (data&0xFF);

	ssp_write_word(&ssp2, command);
	ssp_flush(&ssp2);
}

#define LR_WaitMs(ms) 	do {	\
	if(ms>=10) msleep(ms);	\
	else {int i; for(i=0;i<ms;i++) udelay(1000);}	\
}while(0)
#define LR388_BANK_NUM (0x4)
#define LR388_PER_BANK (0x7F+0x1)
static u8 LR388_Gamma_Table[LR388_BANK_NUM][LR388_PER_BANK]={

//GMP            0x0  0x1  0x2  0x3  0x4   0x5   0x6  0x7  0x8  0x9   0xA   0xB  0xC  0xD  0xE  0xF
	//Bank 1
/*GMP 0x0*/{0xFF,0xF9,0xF5,0xF0,0xEB,0xE8,0xE4,0xE1,0xDF,0xDC,0xD9,0xD7,0xD4,0xD2,0xD0,0xCE,
/*GMP 0x1*/  0xCC,0xCA,0xC9,0xC7,0xC6,0xC4,0xC3,0xC1,0xC0,0xBE,0xBD,0xBC,0xBB,0xBA,0xB8,0xB7,
/*GMP 0x2*/  0xB6,0xB5,0xB4,0xB3,0xB2,0xB1,0xB0,0xAF,0xAE,0xAD,0xAC,0xAC,0xAB,0xAA,0xA9,0xA8,
/*GMP 0x3*/  0xA7,0xA7,0xA6,0xA5,0xA4,0xA4,0xA3,0xA2,0xA2,0xA1,0xA0,0xA0,0x9F,0x9E,0x9D,0x9D,
/*GMP 0x4*/  0x9C,0x9C,0x9B,0x9A,0x9A,0x99,0x99,0x98,0x97,0x97,0x96,0x96,0x95,0x95,0x94,0x94,
/*GMP 0x5*/  0x93,0x92,0x92,0x91,0x91,0x90,0x90,0x8F,0x8F,0x8E,0x8E,0x8D,0x8D,0x8C,0x8C,0x8B,
/*GMP 0x6*/  0x8B,0x8A,0x8A,0x8A,0x89,0x89,0x88,0x88,0x87,0x87,0x86,0x86,0x86,0x85,0x85,0x84,
/*GMP 0x7*/  0x84,0x83,0x83,0x83,0x82,0x82,0x81,0x81,0x81,0x80,0x80,0x7F,0x7F,0x7F,0x7E,0x7E},

//GMP            0x0  0x1  0x2  0x3  0x4   0x5   0x6  0x7  0x8  0x9   0xA   0xB  0xC  0xD  0xE  0xF
	//Bank 2
/*GMP 0x0*/{0x7D,0x7D,0x7D,0x7C,0x7C,0x7C,0x7B,0x7B,0x7B,0x7A,0x7A,0x79, 0x79,0x79,0x78,0x78,
/*GMP 0x1*/  0x78,0x77,0x77,0x76,0x76,0x76,0x75,0x75,0x75,0x74,0x74,0x74,0x73,0x73,0x73,0x72,
/*GMP 0x2*/  0x72,0x72,0x71,0x71,0x70,0x70,0x70,0x6F,0x6F,0x6F,0x6E,0x6E,0x6E,0x6D,0x6D,0x6D,
/*GMP 0x3*/  0x6C,0x6C,0x6C,0x6B,0x6B,0x6B,0x6A,0x6A,0x6A,0x69,0x69,0x69,0x68,0x68,0x68,0x67,
/*GMP 0x4*/  0x67,0x67,0x66,0x66,0x66,0x65,0x65,0x64,0x64,0x64,0x63,0x63,0x63,0x62,0x62,0x62,
/*GMP 0x5*/  0x61,0x61,0x61,0x60,0x60,0x5F,0x5F,0x5F,0x5E,0x5E,0x5D,0x5D,0x5D,0x5C,0x5C,0x5B,
/*GMP 0x6*/  0x5B,0x5A,0x5A,0x59,0x59,0x58,0x58,0x57,0x57,0x56,0x55,0x54,0x54,0x53,0x51,0x50,
/*GMP 0x7*/  0x4F,0x4D,0x4B,0x4B,0x46,0x42,0x3E,0x39,0x32,0x2A,0x22,0x18,0x0C,0x05,0x01,0x00},

//GMP            0x0  0x1  0x2  0x3  0x4   0x5   0x6  0x7  0x8  0x9   0xA   0xB  0xC  0xD  0xE  0xF
	//Bank 3 
/*GMP 0x0*/{0x00,0x06,0x0A,0x0F,0x14, 0x17, 0x1B,0x1E,0x20,0x23,0x26,0x28,0x2B,0x2D,0x2F,0x31,
/*GMP 0x1*/  0x33,0x35,0x36,0x38,0x39,0x3B,0x3C,0x3E,0x3F,0x41,0x42,0x43,0x44,0x45,0x47,0x48,
/*GMP 0x2*/  0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,0x50,0x51,0x52,0x53,0x53,0x54,0x55,0x56,0x57,
/*GMP 0x3*/  0x58,0x58,0x59,0x5A,0x5B,0x5B,0x5C,0x5D,0x5D,0x5E,0x5F,0x5F,0x60,0x61,0x62,0x62,
/*GMP 0x4*/  0x63,0x63,0x64,0x65,0x65,0x66,0x66,0x67,0x68,0x68,0x69,0x69,0x6A,0x6A,0x6B,0x6B,
/*GMP 0x5*/  0x6C,0x6D,0x6D,0x6E,0x6E,0x6F,0x6F,0x70,0x70,0x71,0x71,0x72,0x72,0x73,0x73,0x74,
/*GMP 0x6*/  0x74,0x75,0x75,0x75,0x76,0x76,0x77,0x77,0x78,0x78,0x79,0x79,0x79,0x7A,0x7A,0x7B,
/*GMP 0x7*/  0x7B,0x7C,0x7C,0x7C,0x7D,0x7D,0x7E,0x7E,0x7E,0x7F,0x7F,0x80,0x80,0x80,0x81,0x81},

//GMP            0x0  0x1  0x2  0x3  0x4   0x5   0x6  0x7  0x8  0x9   0xA   0xB  0xC  0xD  0xE  0xF
	//Bank 4
/*GMP 0x0*/{0x82,0x82,0x82, 0x83,0x83,0x83, 0x84, 0x84,0x84,0x85, 0x85, 0x86,0x86, 0x86,0x87,0x87,
/*GMP 0x1*/  0x87,0x88,0x88,0x89,0x89,0x89,0x8A,0x8A,0x8A,0x8B,0x8B,0x8B,0x8C,0x8C,0x8C,0x8D,
/*GMP 0x2*/  0x8D,0x8D,0x8E,0x8E,0x8F,0x8F,0x8F,0x90,0x90,0x90,0x91,0x91,0x91,0x92,0x92,0x92,
/*GMP 0x3*/  0x93,0x93,0x93,0x94,0x94,0x94,0x95,0x95,0x95,0x96,0x96,0x96,0x97,0x97,0x97,0x98,
/*GMP 0x4*/  0x98,0x98,0x99,0x99,0x99,0x9A,0x9A,0x9B,0x9B,0x9B,0x9C,0x9C,0x9C,0x9D,0x9D,0x9D,
/*GMP 0x5*/  0x9E,0x9E,0x9E,0x9F,0x9F,0xA0,0xA0,0xA0,0xA1,0xA1,0xA2,0xA2,0xA2,0xA3,0xA3,0xA4,
/*GMP 0x6*/  0xA4,0xA5,0xA5,0xA6,0xA6,0xA7,0xA7,0xA8,0xA8,0xA9,0xAA,0xAB,0xAB,0xAC,0xAE,0xAF,
/*GMP 0x7*/  0xB0,0xB2,0xB4,0xB7,0xB9,0xBD,0xC1,0xC6,0xCD,0xD5,0xDD,0xE7,0xF3,0xFA,0xFE,0xFF}

};

static void LCD_Sharp_lr388_Init(void)
{
	int index;

	LCD_CtrlWrite_lr388(0xC0, 0x01);
	LR_WaitMs(10);//Min 10

	LCD_CtrlWrite_lr388(0x89, 0x10);
	LCD_CtrlWrite_lr388(0xD0, 0x00);
	LCD_CtrlWrite_lr388(0xD1, 0x00);

	//Bank1 Y conversion
	LCD_CtrlWrite_lr388(0xEF, 0x01);
	for(index=0;index<LR388_PER_BANK;index++)
		LCD_CtrlWrite_lr388(index,LR388_Gamma_Table[0][index]);	
	//Bank2 Y conversion
	LCD_CtrlWrite_lr388(0xEF, 0x02);
	for(index=0;index<LR388_PER_BANK;index++)
		LCD_CtrlWrite_lr388(index,LR388_Gamma_Table[1][index]);	
	//Bank3 Y conversion
	LCD_CtrlWrite_lr388(0xEF, 0x03);
	for(index=0;index<LR388_PER_BANK;index++)
		LCD_CtrlWrite_lr388(index,LR388_Gamma_Table[2][index]);	
	//Bank4 Y conversion
	LCD_CtrlWrite_lr388(0xEF, 0x04);
	for(index=0;index<LR388_PER_BANK;index++)
		LCD_CtrlWrite_lr388(index,LR388_Gamma_Table[3][index]);
	//All Bank Y conversion active
	LCD_CtrlWrite_lr388(0xD0, 0x01);

	//Change to bank0
	LCD_CtrlWrite_lr388(0xEF, 0x00);
	
	LCD_CtrlWrite_lr388(0x10, 0x02);
	LCD_CtrlWrite_lr388(0x12, 0x00);
	LCD_CtrlWrite_lr388(0x13, 0x10);
	LCD_CtrlWrite_lr388(0x14, 0x03);
	LCD_CtrlWrite_lr388(0x15, 0x1D);
	LCD_CtrlWrite_lr388(0x16, 0xC8);
	LCD_CtrlWrite_lr388(0x17, 0x78);

	LCD_CtrlWrite_lr388(0x40, 0x11);
	LCD_CtrlWrite_lr388(0x41, 0x06);
	LCD_CtrlWrite_lr388(0x42, 0x0A);
	LCD_CtrlWrite_lr388(0x43, 0x0A);
	LCD_CtrlWrite_lr388(0x44, 0x02);
	LCD_CtrlWrite_lr388(0x45, 0x12);
	LCD_CtrlWrite_lr388(0x46, 0x20);
	LCD_CtrlWrite_lr388(0x47, 0x06);
	LCD_CtrlWrite_lr388(0x48, 0x05);
	LCD_CtrlWrite_lr388(0x4D, 0x88);
	LCD_CtrlWrite_lr388(0x4E, 0x00);
	
	LCD_CtrlWrite_lr388(0x36, 0x00);
	LCD_CtrlWrite_lr388(0x3B, 0x00);
	LCD_CtrlWrite_lr388(0x3C, 0x00);
	LCD_CtrlWrite_lr388(0x3D, 0x00);
	LCD_CtrlWrite_lr388(0x82, 0x01);
	LCD_CtrlWrite_lr388(0x83, 0x43);
	LCD_CtrlWrite_lr388(0x84, 0x10);

	LCD_CtrlWrite_lr388(0xB0, 0x00);
	LCD_CtrlWrite_lr388(0xB1, 0x00);
	LCD_CtrlWrite_lr388(0xB2, 0x00);
	LCD_CtrlWrite_lr388(0xB3, 0x00);
	LCD_CtrlWrite_lr388(0xB4, 0x00);
	LCD_CtrlWrite_lr388(0xB5, 0x00);
	LCD_CtrlWrite_lr388(0xB6, 0x01);
	LCD_CtrlWrite_lr388(0xB7, 0x00);
	LCD_CtrlWrite_lr388(0xB8, 0x00);
	LCD_CtrlWrite_lr388(0xB9, 0x00);
	LCD_CtrlWrite_lr388(0xBA, 0x00);

	//Positive number in a
#if 1
	#if defined(CONFIG_BOARD_BRAVA)
	#define LCD_OFFSET (-28)
	#elif defined(CONFIG_BOARD_X2)
	#define LCD_OFFSET (-18)
	#else
	#define LCD_OFFSET (-28)
	#endif
	LCD_CtrlWrite_lr388(0x49, 0x7F+LCD_OFFSET);
	LCD_CtrlWrite_lr388(0x4A, 0x5C+LCD_OFFSET);
	LCD_CtrlWrite_lr388(0x4B, 0x7F+LCD_OFFSET);
	LCD_CtrlWrite_lr388(0x4C, 0x5C+LCD_OFFSET);
#else
	LCD_CtrlWrite_lr388(0x49, 0x7F); 
	LCD_CtrlWrite_lr388(0x4A, 0x5C); 
	LCD_CtrlWrite_lr388(0x4B, 0x7F); 
	LCD_CtrlWrite_lr388(0x4C, 0x5C); 
#endif

	//Input data active
	LCD_CtrlWrite_lr388(0x11, 0x00); 
	LCD_CtrlWrite_lr388(0x80, 0x03); 
	LR_WaitMs(4);
	LCD_CtrlWrite_lr388(0x80, 0x07); 
	LR_WaitMs(20); //Min 20
	LCD_CtrlWrite_lr388(0x80, 0x0F); 
	LR_WaitMs(4);
	LCD_CtrlWrite_lr388(0x80, 0x1F); 
	LR_WaitMs(4);
	LCD_CtrlWrite_lr388(0x80, 0x5F); 
	LR_WaitMs(4);
	LCD_CtrlWrite_lr388(0x80, 0x7F); 
	LR_WaitMs(4);
	LCD_CtrlWrite_lr388(0x33, 0x01); 
	LR_WaitMs(4);
	LCD_CtrlWrite_lr388(0x81, 0x01); 
	LR_WaitMs(4);
	
	LCD_CtrlWrite_lr388(0x84, 0x02); 
	LR_WaitMs(1); // 1v-min
	LCD_CtrlWrite_lr388(0x32, 0x13); 
	LR_WaitMs(1); // 1v-min
	LCD_CtrlWrite_lr388(0x30, 0x02); 
	LR_WaitMs(1); // 1v-min
	LCD_CtrlWrite_lr388(0x32, 0x17); 
#if !defined(CONFIG_BOARD_XPHONE)    // h and v flip
	LCD_CtrlWrite_lr388(0x34, 0x70); 
#endif
	LCD_CtrlWrite_lr388(0x35, 0x00); 
	LCD_CtrlWrite_lr388(0x31, 0x01); 
	LR_WaitMs(1); // 1v-min
	LCD_CtrlWrite_lr388(0x10, 0x00);
	LCD_CtrlWrite_lr388(0x30, 0x03); 
	LR_WaitMs(1); // 1v-min
}

static void LCD_Sharp_lr388_off(void)
{
	Lr388_lcd_ssp_init();
	
	printk(KERN_DEBUG"LCD_Sharp_lr388_off \n");
	LCD_CtrlWrite_lr388(0x30, 0x02); 
	LR_WaitMs(1); // 1v-min
	LCD_CtrlWrite_lr388(0x35, 0x01);
	LCD_CtrlWrite_lr388(0x31, 0x01);
	LR_WaitMs(1); // 1v-min
	LCD_CtrlWrite_lr388(0x32, 0x13);
	LCD_CtrlWrite_lr388(0x30, 0x00);
	LR_WaitMs(1); // 1v-min
	LCD_CtrlWrite_lr388(0x32, 0x11);
	LR_WaitMs(1); // 1v-min
	LCD_CtrlWrite_lr388(0x81, 0x00);
	LR_WaitMs(1); // 1v-min
	LCD_CtrlWrite_lr388(0x33, 0x00);
	LR_WaitMs(1); // 1v-min
	
	LCD_CtrlWrite_lr388(0x80, 0x5F);
	LR_WaitMs(1); // 1v-min
	LCD_CtrlWrite_lr388(0x80, 0x1F);
	LR_WaitMs(1); // 1v-min
	LCD_CtrlWrite_lr388(0x80, 0x0F);
	LR_WaitMs(1); // 1v-min
	LCD_CtrlWrite_lr388(0x80, 0x07);
	LR_WaitMs(1); // 1v-min
	LCD_CtrlWrite_lr388(0x80, 0x03);
	LR_WaitMs(1); // 2v-min
	LCD_CtrlWrite_lr388(0x80, 0x00);
	LCD_CtrlWrite_lr388(0x11, 0x02);
	/* let reset low */
	gpio_direction_output(LR388_LCD_RESET,GPIO_LEVEL_LOW);
	Lr388_lcd_ssp_deinit();
	
	/* diable v_lcd_ana */
#if defined(CONFIG_BOARD_X2)
	micco_enable_LDO15(0);
#elif defined(CONFIG_BOARD_BRAVA)
	micco_enable_LDO7(0);
#endif
}

static void Lr388_Lcd_Reset(void)
{  
	pxa3xx_mfp_set_afds(LR388_LCD_RESET,MFP_AF0, MFP_DS04X);
   	gpio_direction_output(LR388_LCD_RESET,GPIO_LEVEL_HIGH);
	udelay(1);
   	gpio_direction_output(LR388_LCD_RESET,GPIO_LEVEL_LOW);
   	LR_WaitMs(20);
   	gpio_direction_output(LR388_LCD_RESET,GPIO_LEVEL_HIGH);
	udelay(1);
}

static void Lr388_lcd_ssp_init(void)
{
	/* grab the port, configure it, then enable it */
	ssp_init(&ssp2, 2, SSP_NO_IRQ);
	ssp_disable(&ssp2);
	ssp_config(&ssp2, 0x0, 0x100007D8, 0x0, 0x00C0048F);//0x00C0040F
	ssp_enable(&ssp2);
	ssp_flush(&ssp2);
}

static void Lr388_lcd_ssp_deinit(void)
{
	ssp_exit(&ssp2);	
}

static void configure_sharp_lr388_lcd(void)
{
	printk(KERN_DEBUG"Sharp_lr388 LCD panel on\n");	
#if defined(CONFIG_BOARD_X2)
	micco_enable_LDO15(1);
#elif defined(CONFIG_BOARD_BRAVA)
	micco_enable_LDO7(1);
#endif	
	Lr388_Lcd_Reset();
	Lr388_lcd_ssp_init();
	LCD_Sharp_lr388_Init();
	Lr388_lcd_ssp_deinit();
}

static void lcd_sharp_lr388_power(int on, struct fb_var_screeninfo *var)
{	
	if(on){ /* Turn LCD ON */
		configure_sharp_lr388_lcd();
	} else { /* Turn LCD OFF */
		LCD_Sharp_lr388_off();
	}

	return;
}

static int lcd_sharp_lr388_test(int value)
{	
	printk("lcd_sharp_lr388_test new val %d\n", value);
	
	Lr388_lcd_ssp_init();
	
	LCD_CtrlWrite_lr388(0x49, 0x7F+value);
	LCD_CtrlWrite_lr388(0x4A, 0x5C+value);
	LCD_CtrlWrite_lr388(0x4B, 0x7F+value);
	LCD_CtrlWrite_lr388(0x4C, 0x5C+value);
	
	Lr388_lcd_ssp_deinit();
	
	return 0;
}

static struct pxafb_mode_info tpo_sharp_lr388_mode =  {
	//.pixclock		= 120000, /* low hz to 50, keep it the same to d0cs */
	.pixclock		= 100*10000/7, // 7M
	.xres			= 240,
	.yres			= 400,// 400
	.bpp			= 16,
	.hsync_len		= 5, // 5
	.left_margin		= 23, // 24
	.right_margin		= 28,// 28
	.vsync_len		= 4,// 2
	.upper_margin		= (1-1),// 1
	.lower_margin		= (7-1),// 7
	.sync			= 0,
};
static struct pxafb_mach_info tpo_sharp_lr388_qvga = {
	.modes = &tpo_sharp_lr388_mode,
	.num_modes		= 1,
	.lccr0			= LCCR0_Act,
	.lccr3			= LCCR3_HSP | LCCR3_VSP,
	//.pxafb_backlight_power	= lcd_backlight_power,
	.pxafb_backlight_power	= NULL, /* move lcdbl to pxafb as it isn't lcd module dependent */
	.pxafb_lcd_power	= lcd_sharp_lr388_power,
	.test_lcd = lcd_sharp_lr388_test,
};

int __init sharp_lr388_lcd_init(void)
{
	set_pxa_fb_info(&tpo_sharp_lr388_qvga);
	return 0;
}

void __exit sharp_lr388_lcd_exit(void)
{
	set_pxa_fb_info(NULL);
	return;
}

module_init(sharp_lr388_lcd_init);
module_exit(sharp_lr388_lcd_exit);
MODULE_LICENSE("YUHUATEL");