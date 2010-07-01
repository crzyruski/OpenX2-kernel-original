/*
   Copyright (C) 2006, Marvell Corporation.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 */ 


#include <linux/init.h>
#include <linux/module.h>
#include "camera.h"
#include <linux/delay.h>

#include <mach/mfp.h>
#include <mach/hardware.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <mach/zylonite.h>

#include <linux/i2c.h>
#include <asm/gpio.h>
#include <mach/camera.h>



#define DENOISE		1
#define	WB_CORRECTION	1
#define FREQUENCE50     1       /* banding filter for 50HZ 60HZ */

#undef	VFLIP
#undef	HMIRROR
/* #define VFLIP	1 */
/* #define HMIRROR	1 */


#define D1MAXEXPLINE 614


/**********************************************************************
 *
 * Constants & Structures
 *
 **********************************************************************/
// CICLK for QCI
#define CICLK	1300


// Revision constants
#define PID_OV56XX              0x56
#define PID_5620                0x20
#define PID_5623		0x20
#define PID_5623_1              0x21

/*OV5620 Mode*/
typedef enum {
    OV5623_FULL = 0,		/* 2560X1920 */
    OV5623_UXGA,		/* 1600X1200 */
    OV5623_SXGA,		/* 1280X960  */
    OV5623_D1MD,		/* 864X600   */
    OV5623_VGA,			/* 640X480   */
    OV5623_INVALID
}OV5623_MODE;


#define OV5623_GAIN             0x00    /* AGC Gain Control */
#define OV5623_BLUE        	0x01    /* Blue Gain Control */
#define OV5623_RED 	        0x02    /* Red Gain Control */
#define OV5623_COM1             0x03    /* Common Control 1 */
#define OV5623_REG04            0x04    /* Register 04 */

/* 05-08 RSVD Reserved */

#define OV5623_COM2             0x09    /* Common control 2 */
#define OV5623_PIDH             0x0A    /* Product ID Number MSBs */
#define OV5623_PIDL             0x0B    /* Product ID Number LSBs */
#define OV5623_COM3             0x0C    /* Common control 3 */ 
#define OV5623_COM4             0x0D    /* Common control 4 */
#define OV5623_COM5             0x0E    /* Common control 5 */
#define OV5623_COM6             0x0F    /* Common control 6 */
#define OV5623_AEC              0x10    /* Automatic Exposure [10:3] */
#define OV5623_CLKRC            0x11    /* Clock Rate Control */
#define OV5623_COM7             0x12    /* Common Control 7 */
#define OV5623_COM8             0x13    /* Common control 8 */
#define OV5623_COM9             0x14    /* Common Control 9 */
#define OV5623_COM10            0x15    /* Common Control 10 */
#define OV5623_GREEN		0x16	/* Digital AWB Green Gain Control */
#define OV5623_HREFST           0x17    /* Horizontal Window Start */
#define OV5623_HREFEND          0x18    /* Horizontal window End */
#define OV5623_VSTRT            0x19    /* Vertical Window Line Start */
#define OV5623_VEND             0x1A    /* Vertical Window Line End */
#define OV5623_PSHFT            0x1B    /* Pixel Shift */
#define OV5623_MIDH             0x1C    /* Manufacturer ID Byte - High */
#define OV5623_MIDL             0x1D    /* Manufacturer ID Byte - Low */

/* 1E-23  RSVD Reserved */
/* Luminance Signal High Range for AEC/AGC Operation */
#define OV5623_AEW              0x24
/* Luminance Signal Low Range for AEC/AGC Operation */
#define OV5623_AEB              0x25
/* Fast Mode Large Step Range Threshold */
#define OV5623_VV               0x26

/* 27-29 RSVD Reserved */

#define OV5623_REG2A            0x2A    /* Register 2A */
/* 8 LSBs of EXHC - pixel count in horizontal blank */
#define OV5623_EXHCL            0x2B
/* 2C RSVD Reserved */

#define OV5623_ADDVSL           0x2D    /* VSYNC Pulse Width LSB 8 bits */
#define OV5623_ADDVSH           0x2E    /* VSYNC Pulse Width MSB 8 bits */
#define OV5623_YAVG             0x2F    /* Luminance Average */
/* HSYNC Position and Width Start Point LSB 8 bits */
#define OV5623_HSDY             0x30
/* HSYNC Position and Width End Lower 8 bits */
#define OV5623_HENY             0x31
#define OV5623_REG32            0x32    /* Register 32 */

/* 33-44 RSVD Reserved */

#define OV5623_REG45            0x45    /* Register 45 */
/* Number of vertical blanking Lines LSBs */
#define OV5623_DMLNL            0x46
/* Number of vertical blanking Lines MSBs */
#define OV5623_DMLNH            0x47
#define OV5623_ZOOMSL           0x48    /* Common Control 19 */
/* Zoom Mode Vertical Window Start Point 8 MSBs */
#define OV5623_ZOOMSH           0x49

/* 4A - 5E RSVD Reserved */

#define OV5623_COM30            0x5F    /* Common Control 30 */
#define OV5623_COM31            0x60    /* Common Control 31 */
#define OV5623_COM32            0x61    /* Common Control 32 */

/* 62 RSVD Reserved */

#define OV5623_COM34            0x63    /* Common Control 34 */

/* 64-7F RSVD Reserved */

#define OV5623_DSPEN            0x80    /* DSP Function Enable Control */
#define OV5623_DSP01            0x81    /* Sensor Internal Reference Control */


/* 82 RSVD Reserved */

#define OV5623_DGCTRL           0x83    /* Digital Gain Control */
#define OV5623_AWBBIAS          0x84    /* AWB Gain Bias Setting */
#define OV5623_DSPCTRL          0x85    /* DSP Control */

/* 86-88 RSVD Reserved */

#define OV5623_DSP09            0x89    /* DSP09 */

/* 8A RSVD Reserved */

#define OV5623_DSP0B            0x8B    /* DSP0B */

/* 8C-A7 RSVD Reserved */

#define OV5623_BOTLMT           0xA8    /* Pixel value Lower limit */
#define OV5623_TOPLMT           0xA9    /* Pixel value Upper limit */

/* AA-B7 RSVD Reserved */

#define OV5623_REDLMT		0xB8	/* Red Gain Limit */
#define OV5623_GREENLMT		0xB9	/* Green Gain Limit */
#define OV5623_BLUELMT		0xBA	/* Blue Gain Limit */

/* End of OV5623 register */
#define OV5623_REGEND         	0xFF


/*****************************************************************************
 *  
 *****************************************************************************/
u32 field_pclk =CICLK;
u32 field_max_line =617;
u32 field_max_pixel =1300;

u8 field_gain = 0 ;
u32 field_exp = 0 ; 
u8 field_redg = 0, field_greeng = 0, field_blueg = 0; 

int G_ov5623_mode_change = 0;

static struct i2c_client *g_client;

/*****************************************************************************
 *  Register Settings, Get from OmniVision
 ****************************************************************************/

const static u8 ov5623InitSetting[]=
{
	
	OV5623_COM8,     0x00,        
	OV5623_AEC,      0x1E,
	0x3B,		 0x07,

	0x5B,		 0x40,
	0x39,		 0x07,
	0x53,		 0x02,
	0x54,		 0x40,
	OV5623_REG04,    0x20,
	0x27,		 0x04,

	0x36,		 0x00,
	0xC5,		 0x04,

	0x4E,		 0x00,
	0x4F,		 0x93,
	0x50,		 0x7B,
	0xCA,		 0x0C,
	0xCB,		 0x0F,

	0x39,		 0x07,

	0x4A,		 0x10,

	0x3E,		 0x0A,
	0x3D,		 0x00,


#ifdef  HMIRROR
        #ifdef VFLIP
                0x04,   0xF0,
        #else
                0x04,   0xA0,
        #endif
#endif

#ifdef VFLIP
        #ifdef HMIRROR
                0x04,   0xF0,
        #else
                0x04,   0x70,
        #endif
#endif

#ifdef	WB_CORRECTION
	OV5623_DSP01,	 0x07,
#endif

	OV5623_REGEND,   0x00
};

const static u8 ov5623OtherSetting[]=
{
	OV5623_DSP09,	 0x20,
	OV5623_DGCTRL,	 0x80,
	0xB7,		 0x9D,
	0xB6,		 0x11,
	0xB5,		 0x55,
	0xB4,		 0x00,
	OV5623_TOPLMT,	 0xF0,
	OV5623_BOTLMT,	 0x0A,
	OV5623_REDLMT,	 0xF0,
	OV5623_GREENLMT, 0xF0,
	OV5623_BLUELMT,  0xF0,

        OV5623_COM8,     0xE7,
	OV5623_COM9,	 0x60,

	
	0x33,		 0x75,
	0x2C,		 0x00,
	OV5623_COM2,	 0x00,
	0x35,		 0x30,
	0x27,		 0x04,
	0x3C,		 0x07,	
	0x3A,		 0x0A,
	0x3B,		 0x07,
	OV5623_BLUE,	 0x40,
	OV5623_RED,	 0x40,
	OV5623_GREEN,	 0x40,
	0x52,		 0xB0,

	0x51,		 0x83,

	0x21,		 0xAB,	
	0x22,		 0x05,	
	0x23,		 0x00,	
	0x35,		 0x30,  

	0x20,		 0x90,
	0x28,		 0x30,
	0x73,		 0xE1,
	0x6C,		 0x00,
	0x6D,		 0x80,
	0x6E,		 0x00,
	0x70,		 0x04,
	0x71,		 0x00,	
	0x8D,		 0x04,
	0x64,		 0x00,
	0x65,		 0x00,
	0x66,		 0x00,
	0x67,		 0x00,
	0x68,		 0x00,	
	0x69,		 0x00,
	0x6A,		 0x00,
	0x6B,		 0x00,

	0xC5,		 0x03,

	0x71,		 0x93, 

	0x74,		 0x20,	
	0x8D,		 0x44,

	OV5623_DSPEN,	 0x09,
	OV5623_DSPCTRL,	 0xc0,

	0xD2,		 0x00,
	0xD3,		 0x00,
	0xD4,		 0x00,
	0xD5,		 0x00,
	0xD6,		 0x00,
	0xD7,		 0x00,


	OV5623_REGEND,   0x00
};

const static u8 ov5623_full[]=
{
	0x11,	0x00,
	0x12,	0x00,
	0x2a,	0x00,
	0x2b,	0x00,
	0x0c,	0x38,
	0x0d,	0x06,
	
	0x4e,	0x00,
	0x4f,	0x93,
	0x50,	0x7b,
	0xca,	0x0c,
	0xcb,	0x0f,

	0x17,	0x12,
	0x18,	0xb4,
	0x19,	0x01,
	0x1a,	0xf4,
	0x03,	0x4a,

	0x48,	0x08,
	0x46,	0x30,
	0x32,	0x00,
	0x53,	0x02,
	0x54,	0x00,

	0x60,	0x00,
	0x61,	0x00,
	0x38,	0x90,

	0x37,	0x82,
	0x44,	0x48,

	0x21,	0xab,
	0x22,	0x05,
	0x23,	0x00,
	0x35,	0x30,

	0x8d,	0x44,
	
	OV5623_REGEND,   0x00
};

const static u8 ov5623_sxga[]=
{
	0x11,	0x00,
	0x12,	0x40,
	0x2a,	0x10,
	0x2b,	0x86,
	0x0c,	0x0a,
	0x0d,	0xc6,

	0x4e,	0x10,
	0x4f,	0x24,
	0x50,	0xf4,
	0xca,	0x02,
	0xcb,	0x03,

	0x17,	0x12,
	0x19,	0x01,
	0x03,	0x42,
	
	0x48,	0x09,
	0x46,	0x11,

	0x60,	0x00,
	0x61,	0x00,
	0x38,	0x10,

	0x37,	0x80,
	0x44,	0x48,

	0x21,	0xab,
	0x22,	0x05,
	0x23,	0x00,	
	0x35,	0x30,

	0x8d,	0x64,

	OV5623_REGEND,   0x00
};

const static u8 ov5623_d1md[]=
{
	0x11,	0x00,
	0x12,	0x20,
	0x2a,	0x10,
	0x2b,	0xc4,
	0x0c,	0x0a,
	0x0d,	0x27,

	0x4e,	0x00,
	0x4f,	0xb8,
	0x50,	0x9a,
	0xca,	0x02,
	0xcb,	0x03,

	0x17,	0x11,
	0x19,	0x01,
	0x03,	0x42,
	
	0x48,	0x08,
	0x46,	0x11,

	0x32,	0x06,
	0x53,	0x02,
	0x54,	0x00,

	0x60,	0x00,
	0x61,	0x00,
	0x38,	0x10,

	0x37,	0x80,
	0x44,	0xc0,

	0x21,	0xbb,
	0x22,	0x15,
	0x23,	0x03,	
	0x35,	0x38,

	0x8d,	0x64,

	OV5623_REGEND,   0x00
};

static int read_sensor_reg(const u8, u8 *);
static int write_sensor_reg(const u8, u8 *);
static void sensor_power_on(void)
{
	struct sensor_platform_data *pdata;
	pdata = g_client->dev.platform_data;

	pdata->power_on(pdata->id);

}

static void sensor_power_off(void)
{
	int val = 0;
	struct sensor_platform_data *pdata;
	pdata = g_client->dev.platform_data;

	read_sensor_reg(OV5623_COM4, &val);
	val &= ~0x06;
	write_sensor_reg(OV5623_COM4, &val);

	pdata->power_off(pdata->id);
}

/* i2c operation */
static int read_sensor_reg( const u8 subAddress, u8 *bufP )
{
	int ret = 0;

	if( g_client == NULL )	/*	No global client pointer?	*/
		return -1;
	
	ret = i2c_smbus_write_byte(g_client, subAddress);
	if(ret){
		printk("sensor i2c read error\n");
		return ret;
	}

	ret = i2c_smbus_read_byte(g_client);
	if (ret >= 0) {
		*bufP = ret;
	}
	return ret;
}

static int write_sensor_reg( const u8 subAddress, u8 *bufP )
{
	if( g_client == NULL )	/*	No global client pointer?	*/
		return -1;

	return i2c_smbus_write_byte_data(g_client, subAddress, *bufP);
}
/*****************************************************************************
 *  Sensor read/write 
 *****************************************************************************/

static int ov5623hw_set_regs( const u8 *regP )
{
	u32 curReg = 0;
	int    status = 0;

	/* The list is a register number followed by the value */
	while( regP[curReg << 1] < OV5623_REGEND )
	{
		u8 regVal = regP[(curReg << 1) + 1];

		status = (write_sensor_reg( regP[curReg << 1], &regVal ) == 0)?
			0 : -EIO;

		curReg++;
	}
	

	return status;
}
/*****************************************************************************
 *  Power & Reset
 *****************************************************************************/
static void ov5623hw_power_down( u8 powerMode )
{
	/* OV5623 PWRDWN, 0 = NORMAL, 1=POWER DOWN */
	if( powerMode == CAMERA_POWER_OFF )
		sensor_power_off();
	else
		sensor_power_on();

}

static void ov5623hw_reset(void)
{
	u8 val =0x80;
	write_sensor_reg(0x12,&val);
	write_sensor_reg(0x12,&val);
	return;
}


/*****************************************************************************
 *  Settings
 *****************************************************************************/
static int ov5623hw_version_revision(u8 * pCmRevision, u8 *pSensorRevision)
{
	read_sensor_reg( OV5623_PIDH, pCmRevision );
	read_sensor_reg( OV5623_PIDL, pSensorRevision );
	return 0;
}

static void ov5623hw_auto_function_on(void)
{
	u8 val;
	read_sensor_reg( OV5623_COM8, &val );
	val |= 0x07;    /* don't disturb AWB */
	write_sensor_reg( OV5623_COM8, &val );
}

static void ov5623hw_auto_function_off(void)
{
	u8 val;
	read_sensor_reg( OV5623_COM8, &val );
	val &= ~0x07;    /* don't disturb AWB */
	write_sensor_reg( OV5623_COM8, &val );
}


static int ov5623hw_auto_ae(int type)
{
	u8 val;
	if(type){
		read_sensor_reg( OV5623_COM8, &val );
		val |= 0x05;    /* don't disturb AWB */
		write_sensor_reg( OV5623_COM8, &val );
	}else{
		read_sensor_reg( OV5623_COM8, &val );
		val &= ~0x05;    /* don't disturb AWB */
		write_sensor_reg( OV5623_COM8, &val );
	}
	return 0;
}

static int ov5623hw_auto_awb(int type)
{
	u8 val;

	if(type){
		read_sensor_reg( OV5623_COM8, &val );
		val |= 0x02;
		write_sensor_reg( OV5623_COM8, &val );
	}else{
		read_sensor_reg( OV5623_COM8, &val );
		val &= ~0x02;
		write_sensor_reg( OV5623_COM8, &val );
	}

	return 0;
}


/*****************************************************************************
 *Manual Exposure Control
 ****************************************************************************/
static int ov5623hw_calculate_pclk(u32 ciclk)
{
	u8 pll_divider;
	u8 pll_multiplier;
	u8 clk_divider;
	u8 clk_divider_flag;

	u32 pixel_clk = 0;

	read_sensor_reg(0x3d, &pll_divider);
	pll_divider &= 0x10;
	if (pll_divider > 0)
		pll_divider = 1;
	else if(pll_divider == 0)
		pll_divider = 10;


	read_sensor_reg(0x3e, &pll_multiplier);
	pll_multiplier &= 0x3f;
	
	read_sensor_reg(OV5623_CLKRC, &clk_divider);
	clk_divider_flag = clk_divider & 0x40;
	clk_divider_flag = clk_divider_flag >> 6;
	clk_divider &= 0x3f;

	if( clk_divider_flag == 0) {
		pixel_clk = ciclk  * pll_multiplier * 2 / pll_divider;
	} else if( clk_divider_flag == 1) {
		pixel_clk = ciclk * pll_multiplier * 2/
			(pll_divider * 2 *(clk_divider + 1));
	}
	

	return pixel_clk;
}

static int ov5623_sensor_snap2preview(void)
{
	u8 reg;
	write_sensor_reg(0x02,&field_redg);
	write_sensor_reg(0x16,&field_greeng);
	write_sensor_reg(0x01,&field_blueg);
	write_sensor_reg(0x00,&field_gain);
	/*write exposure line */
	read_sensor_reg(0x04,&reg);
	reg = reg & 0xf8;
	reg = reg |(field_exp & 0x07);
	write_sensor_reg(0x04,&reg);
	field_exp >>= 3;
	reg = field_exp & 0xff;
	write_sensor_reg(0x10,&reg);
	field_exp >>= 8;
	reg = field_exp & 0x1f;
	write_sensor_reg(0x45,&reg);
	return 0;
}


static int  ov5623hw_field_protect(u32 captureWidth, u32 captureHeight)
{
	u8 gain_val,aechm_val,aech_val,aecl_val,blueg_val,redg_val;
	OV5623_MODE mode;
	
	field_pclk = ov5623hw_calculate_pclk(CICLK);

	/* let the sensor work on proper mode */
        if((captureWidth <= 864) && (captureHeight <= 600)) {
                mode = OV5623_D1MD;
        } else if((captureWidth <= 1280) && (captureHeight <= 960)) {
                mode = OV5623_SXGA;
        } else if((captureWidth <= 2560) && (captureHeight <= 1920)) {
                mode = OV5623_FULL;
        } else {
                return -EINVAL;
        }

        switch(mode)
        {
        case OV5623_D1MD:
                field_max_pixel = 1300;
                field_max_line = 617;
                break;
        case OV5623_SXGA:
                field_max_pixel = 1640;
                field_max_line = 977;
                break;
        case OV5623_FULL:
                field_max_pixel = 3252;
                field_max_line = 1968;
                break;
        default:
                printk("manual_exp_gain:Wrong still image size.\n");
                break;
	}

	read_sensor_reg(OV5623_GAIN, &gain_val);
	read_sensor_reg(OV5623_REG45, &aechm_val);
	read_sensor_reg(OV5623_AEC, &aech_val);
	read_sensor_reg(OV5623_REG04, &aecl_val);
	read_sensor_reg(OV5623_BLUE, &blueg_val);
	read_sensor_reg(OV5623_RED, &redg_val);

	read_sensor_reg(0x16,&field_greeng);    // g gain
  
	field_gain = gain_val;
	field_exp = (((u32)(aechm_val & 0x3f)) << 11) +	\
		(((u32)(aech_val)) << 3) + (u32)(aecl_val & 0x07);
	field_blueg = blueg_val;
	field_redg = redg_val;

	return 0;
	
} 
	

static int ov5623hw_calculate_exp(u32 captureWidth, u32 captureHeight)
{
	u8 aecl_val,aech_val,aechm_val;
	u32 capture_exp, preview_exp;
	u32 capture_max_line, preview_max_line;
	u32 capture_max_pixel, preview_max_pixel;
	u32 capture_pixel_clk, preview_pixel_clk;
	u32 capture_exp_gain;
	u32 time_unit;
	u32 capture_gain;
	u8  tmp_gain;
	OV5623_MODE mode;

	/* let the sensor work on proper mode */
        if((captureWidth <= 864) && (captureHeight <= 600)) {
                mode = OV5623_D1MD;
        } else if((captureWidth <= 1280) && (captureHeight <= 960)) {
                mode = OV5623_SXGA;
        } else if((captureWidth <= 2560) && (captureHeight <= 1920)) {
                mode = OV5623_FULL;
        } else {
                return -EINVAL;
        }

	switch(mode)
	{
	case OV5623_D1MD:
		capture_max_pixel = 1300; 
		capture_max_line = 617;
		break;
	case OV5623_SXGA:
		capture_max_pixel = 1640;
		capture_max_line = 977;
		break;
	case OV5623_FULL:
		capture_max_pixel = 3252;
		capture_max_line = 1968;
		break;
	default:
		printk("manual_exp_gain:Wrong still image size.\n");
		break;
	}

	capture_pixel_clk = ov5623hw_calculate_pclk(CICLK); 

	preview_exp = field_exp;
	preview_max_line = field_max_line;
	preview_max_pixel = field_max_pixel;
	preview_pixel_clk = field_pclk;

	
	capture_exp = preview_exp * capture_pixel_clk *	\
		preview_max_pixel /( preview_pixel_clk * capture_max_pixel);

	/*convert reg 0x00 to field_gain * 16*/
	capture_gain =(field_gain & 0x0f) + 16;


	if (field_gain & 0x10)
		capture_gain = capture_gain << 1;
	if (field_gain & 0x20)
		capture_gain = capture_gain << 1;
	if (field_gain & 0x40)
		capture_gain = capture_gain << 1;
	if (field_gain & 0x80)
		capture_gain = capture_gain << 1;

	capture_exp_gain = capture_exp * capture_gain;


	/*calculate banding filter value*/
	/* 1/100s for 50Hz, 1/120s for 60Hz*/
#ifdef FREQUENCE50
	time_unit = capture_pixel_clk *10000 / capture_max_pixel / 100;
#elif  FREQUENCE60
	time_unit = capture_pixel_clk *10000 / capture_max_pixel / 120;
#endif


	/* calculate field_gain, exposure*/

	if (capture_exp_gain < (capture_max_line * 16))
	{
		capture_exp = capture_exp_gain / 16;
		
		if (capture_exp > time_unit) 
		{
			capture_exp /= time_unit;
			capture_exp *= time_unit;
		}
	}
	else
	{
		capture_exp = capture_max_line;
	}

	if (capture_exp == 0)
		capture_exp = 1;

	capture_gain = (capture_exp_gain*100 + 50)/capture_exp/100;

	/* calculate reg0x00 from field_gain * 16*/

	tmp_gain = 0;
	if (capture_gain > 31)
	{
		tmp_gain |= 0x10;
		capture_gain = capture_gain >> 1;
	}	
	if (capture_gain > 31)
	{
		tmp_gain |= 0x20;
		capture_gain = capture_gain >> 1;
	}	
	if (capture_gain > 31)
	{
		tmp_gain |= 0x40;
		capture_gain = capture_gain >> 1;
	}	
	if (capture_gain > 31)
	{
		tmp_gain |= 0x80;
		capture_gain = capture_gain >> 1;
	}	

	if (capture_gain > 16)
		tmp_gain |= ((capture_gain -16) & 0x0f);
	
	capture_gain = tmp_gain;


	/*set values to reg*/
	read_sensor_reg(OV5623_REG04, &aecl_val);
	aecl_val &= 0xf8;
	aecl_val |= (u8)(capture_exp & 0x07);

	read_sensor_reg(OV5623_REG45, &aechm_val);
	aechm_val &= 0xc0;
	aechm_val |= (u8)((capture_exp & 0x1f800) >> 11);

	aech_val = (u8)((capture_exp & 0x7f8) >> 3);


	write_sensor_reg(OV5623_GAIN, 	&tmp_gain);
	write_sensor_reg(OV5623_BLUE,	&field_blueg);
	write_sensor_reg(OV5623_RED,  	&field_redg);
	write_sensor_reg(0x16,          &field_greeng);
 

	write_sensor_reg(OV5623_REG45, 	&aechm_val);
	write_sensor_reg(OV5623_AEC, 	&aech_val);
	write_sensor_reg(OV5623_REG04, 	&aecl_val);
	

	return 0;
		
}


/*****************************************************************************
 *  Format 
 *****************************************************************************/
static int ov5623hw_set_format(u32 captureWidth, 
		u32 captureHeight, 
		u32 *winStartX, 
		u32 *winStartY, 
		u32 *winEndX, 
		u32 *winEndY, u32 type)
{
	OV5623_MODE mode;

	/* let the sensor work on proper mode */
	if((captureWidth <= 864) && (captureHeight <= 600)) {
		mode = OV5623_D1MD;
	} else if((captureWidth <= 1280) && (captureHeight <= 960)) {
		mode = OV5623_SXGA;
	} else if((captureWidth <= 2560) && (captureHeight <= 1920)) {
		mode = OV5623_FULL;
	} else {
		return -EINVAL;
	}

	ov5623hw_set_regs(ov5623InitSetting);

	if (mode == OV5623_D1MD) {
		int defclk = 48;
		int defframe = 60;
		int curpclk = 26;
		int curframe = 0;
		int bandingfilterValue =0;

		// 50hz banding filter
		u8 val;
		curframe = defframe * curpclk /defclk;
		bandingfilterValue = curframe * D1MAXEXPLINE /100;

		ov5623hw_set_regs(ov5623_d1md);

		val = 0x20;
		write_sensor_reg(0x41, &val);// manual 50 hz
		write_sensor_reg(0x4f, (u8 *)&bandingfilterValue);
		val = 0x00;
		write_sensor_reg(0x11,&val);

	} else if (mode == OV5623_SXGA) {
		ov5623hw_set_regs(ov5623_sxga);
	} else {
		ov5623hw_set_regs(ov5623_full);
	}

	/* Special for OV5623 */
	ov5623hw_set_regs(ov5623OtherSetting);


	*winStartX = 0;
	*winStartY = 0;
	*winEndX   = captureWidth;
	*winEndY   = captureHeight;

	if((G_ov5623_mode_change != 0)&&(G_ov5623_mode_change != type))
	{
		if(type == CAMERA_MODE_STILL){
			ov5623hw_calculate_exp(captureWidth, captureHeight);
		}else{
			ov5623_sensor_snap2preview();
		}
	}

	return 0;
}

/*****************************************************************************
 * Contrast
 *****************************************************************************/
/* FIX ME: TBD */
const static u8 ContrastLowestSettings[] = {
	OV5623_REGEND,     0x00
};

const static u8 ContrastLowSettings[] = {
	OV5623_REGEND,     0x00        
};

const static u8 ContrastMiddleSettings[] = {
	OV5623_REGEND,     0x00        
};

const static u8 ContrastHighSettings[] = {
	OV5623_REGEND,     0x00        
};

const static u8 ContrastHighestSettings[] = {
	OV5623_REGEND,     0x00        
};

static int ov5623hw_set_contrast(u32 value)
{
	const u8 *regP;

	regP = NULL;
	switch(value) {
		case SENSOR_CONTRAST_LOWEST:
			regP = ContrastLowestSettings;
			break;
		case SENSOR_CONTRAST_LOW:
			regP = ContrastLowSettings;
			break;
		case SENSOR_CONTRAST_MIDDLE:
			regP = ContrastMiddleSettings;
			break;
		case SENSOR_CONTRAST_HIGH:
			regP = ContrastHighSettings;
			break;
		case SENSOR_CONTRAST_HIGHEST:
			regP = ContrastHighestSettings;
			break;
		default:
			regP = ContrastMiddleSettings;
			break;
	}

	/* set hw */
	if (regP)        
		ov5623hw_set_regs(regP);

	return 0;
}

/*****************************************************************************
 * Exposure
 *****************************************************************************/
/* FIX ME: TBD */
const static u8 ExposureSettings[] = {
};

static int ov5623hw_set_exposure(u32 exp)
{
	 u8 reg = 0;

	 /*write exposure line */
	 read_sensor_reg(0x04,&reg);
	 reg = reg & 0xf8;
	 reg = reg |(exp & 0x07);
	 write_sensor_reg(0x04,&reg);
	 exp >>= 3;
	 reg = exp & 0xff;
	 write_sensor_reg(0x10,&reg);
	 exp >>= 8;
	 reg = exp & 0x3f;
	 write_sensor_reg(0x45,&reg);

	 return 0;
}

static int ov5623hw_set_gain(int gain)
{
	u8  ov5623_recBgain = (gain & 0xFF000000)>> 24;
	u8  ov5623_recGgain = (gain & 0x00FF0000)>> 16;
	u8  ov5623_recRgain = (gain & 0x0000FF00)>> 8;
	u8  ov5623_gaincode = (gain & 0x000000FF);
	
	write_sensor_reg(0x02,&ov5623_recRgain);        /* R gain */
	write_sensor_reg(0x16,&ov5623_recGgain);        /* G gain */
	write_sensor_reg(0x01,&ov5623_recBgain);        /* B gain */
	write_sensor_reg(0x00,&ov5623_gaincode);   	/* GAIN */

	return 0;
}
/*
static void ov5623hw_vflip()
{
	u8 reg = 0;

	read_sensor_reg(0x04,&reg);
	reg |= 0x40;
	write_sensor_reg(0x04,&reg);
}
*/

/*****************************************************************************
 * Abstract:  
 *    contains all primitive functions for OV5623
 *
 * Notes:Only valid for processor code named Monahans.
 *****************************************************************************/


/* LUT Table, TBD */
static unsigned char lut_table[] = {
	/* RED LUT */
	0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c,
	0x20, 0x24, 0x28, 0x2c, 0x30, 0x34, 0x38, 0x3c,
	0x40, 0x44, 0x48, 0x4c, 0x50, 0x54, 0x58, 0x5c,
	0x60, 0x64, 0x68, 0x6c, 0x70, 0x74, 0x78, 0x7c,
	0x80, 0x84, 0x88, 0x8c, 0x90, 0x94, 0x98, 0x9c,
	0xa0, 0xa4, 0xa8, 0xac, 0xb0, 0xb4, 0xb8, 0xbc,
	0xc0, 0xc4, 0xc8, 0xcc, 0xd0, 0xd4, 0xd8, 0xdc,
	0xe0, 0xe4, 0xe8, 0xec, 0xf0, 0xf4, 0xf8, 0xfc,

	/* BLUE LUT */
	0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c,
	0x20, 0x24, 0x28, 0x2c, 0x30, 0x34, 0x38, 0x3c,
	0x40, 0x44, 0x48, 0x4c, 0x50, 0x54, 0x58, 0x5c,
	0x60, 0x64, 0x68, 0x6c, 0x70, 0x74, 0x78, 0x7c,
	0x80, 0x84, 0x88, 0x8c, 0x90, 0x94, 0x98, 0x9c,
	0xa0, 0xa4, 0xa8, 0xac, 0xb0, 0xb4, 0xb8, 0xbc,
	0xc0, 0xc4, 0xc8, 0xcc, 0xd0, 0xd4, 0xd8, 0xdc,
	0xe0, 0xe4, 0xe8, 0xec, 0xf0, 0xf4, 0xf8, 0xfc,

	/* GREEN LUT */
	0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c,
	0x20, 0x24, 0x28, 0x2c, 0x30, 0x34, 0x38, 0x3c,
	0x40, 0x44, 0x48, 0x4c, 0x50, 0x54, 0x58, 0x5c,
	0x60, 0x64, 0x68, 0x6c, 0x70, 0x74, 0x78, 0x7c,
	0x80, 0x84, 0x88, 0x8c, 0x90, 0x94, 0x98, 0x9c,
	0xa0, 0xa4, 0xa8, 0xac, 0xb0, 0xb4, 0xb8, 0xbc,
	0xc0, 0xc4, 0xc8, 0xcc, 0xd0, 0xd4, 0xd8, 0xdc,
	0xe0, 0xe4, 0xe8, 0xec, 0xf0, 0xf4, 0xf8, 0xfc,
};

static CI_CMU_COE_MATRIX ov5623_cRGB_sRGB_COE = {
	RGB_FLOAT_TO_INT(1.769117),  RGB_FLOAT_TO_INT(-0.614235), RGB_FLOAT_TO_INT(-0.154883),
	RGB_FLOAT_TO_INT(-0.505265) , RGB_FLOAT_TO_INT(2.023034), RGB_FLOAT_TO_INT(-0.517769),
	RGB_FLOAT_TO_INT(-0.67645),  RGB_FLOAT_TO_INT(-1.101237), RGB_FLOAT_TO_INT(2.5579)
};

static CI_CMU_COE_MATRIX ov5623_cRGB_YUV_COE = {
	YUV_FLOAT_TO_INT(0.257),  YUV_FLOAT_TO_INT(0.504),
	YUV_FLOAT_TO_INT(0.098),
	
	YUV_FLOAT_TO_INT(0.439),  YUV_FLOAT_TO_INT(-0.368),
	YUV_FLOAT_TO_INT(-0.071),

	YUV_FLOAT_TO_INT(-0.148), YUV_FLOAT_TO_INT(-0.291), 
	YUV_FLOAT_TO_INT(0.439),
};

/* Bad pixel table, it's sensor specific, TBD */
#define END_MARKER -1
static int ov5623_badpixel_table[] = {
	/*  column,        row
	 *    0,           0,
	 *    1,           1,
	 *    2591,        1943,
	 */
	END_MARKER, END_MARKER
};

extern int G_ov5623_mode_change;

/*****************************************************************************
 * OV5623 Functions
 *****************************************************************************/
static int ov5623_init( p_camera_context_t camera_context )
{
	u8 cm_rev, cm_pid;
	int timeout;
	int status;

	/* provide informat about the capabilities of the sensor */
	camera_context->sensor_status.caps |= SENSOR_CAP_MANUAL_CONTRAST |
		SENSOR_CAP_MANUAL_WHITEBALANCE |
		SENSOR_CAP_MANUAL_EXPOSURE;

	/* Configure CI according to OV5623's hardware        
	 * master parallel with 8 data pins
	 */
	ci_set_mode( CI_MODE_MP, CI_DATA_WIDTH10);

	/* enable pixel clock(sensor will provide pclock)
	 * and master clock = 26MHZ
	 */
	ci_set_clock(1, 1, CICLK);

	/* data sample on rising and h,vsync active high */
	/* Paul Shen */
	ci_set_polarity( 0, 0, 1);	

	/* fifo control */
	ci_set_fifo( 0, CI_FIFO_THL_32, 1, 1);

	/* set black level */
	ci_cgu_set_black_level(0);

	/* CGU Mux Select */
	ci_cgu_set_addr_mux_select(CI_CGU_MUX_2_TO_9);

	/* OV5623 Power on sequence
	 * Take out of Power down mode (GPIO_57), PWRDWN=1, NORMAL=0
	 * Assert Reset
	 * Delay
	 * Remove reset
	 * Delay
	 */
	ov5623hw_power_down(CAMERA_POWER_FULL);
	ov5623hw_reset();

	/* read out version */
	timeout = 50;
	do {
		cm_pid = cm_rev = 0;
		status = ov5623hw_version_revision(&cm_pid, &cm_rev);

		/* Check to make sure we are working with an OV5623 */
		if( cm_pid != PID_OV56XX || ((cm_rev != PID_5623)&&(cm_rev != PID_5623_1)) )
		{
			ov5623hw_power_down(CAMERA_POWER_OFF );
			ov5623hw_power_down(CAMERA_POWER_FULL );
			ov5623hw_reset();
			mdelay(1);
		}
		if (--timeout == 0)
			return -EIO;
	} while( cm_pid != PID_OV56XX );

	/* turn sensor output off */
	G_ov5623_mode_change = 0;
	mdelay(30);
	return 0;
}

static int ov5623_deinit(  p_camera_context_t camera_context )
{
	/* power off the external module */
	ov5623hw_power_down(CAMERA_POWER_OFF );

	return 0;
}

static int ov5623_sleep(  p_camera_context_t camera_context )
{
	return ov5623_deinit( camera_context );    
}

static int ov5623_wake(  p_camera_context_t camera_context )
{
	return ov5623_init( camera_context );
}

static int ov5623_set_capture_format(  p_camera_context_t camera_context )
{
	CI_MP_TIMING timing;
	u32 winStartX = 0, winStartY = 0;
	u32 winEndX = 0, winEndY = 0;
	int           *padPixelX, *padPixelY;
	int           badPixelNum;

	/* Set CMU Coe matrix, if necessary */
	if((camera_context->capture_input_format != V4L2_PIX_FMT_SRGGB10)&&
		(camera_context->capture_input_format !=
			V4L2_PIX_FMT_SRGGB8))
		return -EINVAL;

	if (camera_context->cmu_usage == CI_CMU_OUTPUT_YUV) {
		ci_cmu_set_color_correction_coe(&ov5623_cRGB_YUV_COE);
	}

	if (camera_context->cmu_usage == CI_CMU_OUTPUT_RGB) {
		ci_cmu_set_color_correction_coe(&ov5623_cRGB_sRGB_COE);
	}

	ci_cmu_enable(camera_context->cmu_usage);

	/* Set OV5623 format */
	ov5623hw_set_format(
			camera_context->capture_input_width, 
			camera_context->capture_input_height,
			&winStartX,
			&winStartY,
			&winEndX,
			&winEndY, camera_context->capture_mode);

	/* configure PSU, if necessary */
	if (camera_context->psu_enable ) {
		ci_psu_enable(0);
	}

	padPixelX = ov5623_badpixel_table;
	padPixelY = ov5623_badpixel_table + 1;
	badPixelNum = 0; 
	while ((*padPixelX != END_MARKER) && (*padPixelY != END_MARKER)){

		if (((u32)(*padPixelX) >= winStartX) &&
			((u32)(*padPixelX) <= winEndX) &&
			((u32)(*padPixelY) >= winStartY) &&
			((u32)(*padPixelY) <= winEndY)) {

			/* the very first pixel of each color
			 * can't be tagged as bad!!!
			 */
			if ((*padPixelX - winStartX > 1) ||
				(*padPixelY - winStartY > 1)){
				ci_psu_tag_bad_pixel(
					*padPixelX - winStartX, 
					*padPixelY - winStartY);

				badPixelNum++;
				/* PSU only can substitute 128 bad pixels!! */
				if (badPixelNum == 128)
					break;
			}
		}

		padPixelX += 2;
		padPixelY += 2;
	}

	/* fill the last PSU RAM location with all 0's,
	 * signifying the end of the dead pixel addresses stored in the RAM
	 */
	if (badPixelNum < 128)
		ci_psu_tag_bad_pixel(0, 0);

	if (badPixelNum != 0){
			ci_psu_enable(1);
	}
	

	/* set capture width/height and timing */
	timing.BFW = 0x01;
	timing.BLW = 0x01;   /*GRBG to RGGB*/         
	ci_configure_mp(camera_context->capture_input_width-1,
		camera_context->capture_input_height-1, &timing);

	return 0;
}

static int ov5623_load_lut(p_camera_context_t camera_context)
{
	if (camera_context->cgu_enable) {
		ci_cgu_load_lut_ram((unsigned int*)camera_context->
					histogram_lut_buffer_virtual,
				camera_context->histogram_lut_buffer_physical,
				(unsigned int*)camera_context->
					histogram_lut_dma_descriptors_virtual,
				camera_context->
					histogram_lut_dma_descriptors_physical,
				lut_table);
	}

	ci_cgu_enable(camera_context->cgu_enable);
	return 0;
}

static int ov5623_start_capture(p_camera_context_t camera_context,
		unsigned int frames)
{   
	(void) camera_context;

	if (camera_context->cgu_enable) {
		ci_cgu_load_lut_ram((unsigned int*)camera_context->
					histogram_lut_buffer_virtual,
				camera_context->histogram_lut_buffer_physical,
				(unsigned int*)camera_context->
					histogram_lut_dma_descriptors_virtual,
				camera_context->
					histogram_lut_dma_descriptors_physical,
				lut_table);
	}

	ci_cgu_enable(camera_context->cgu_enable);


	/* turn auto function on only doing continues capture */
	if (frames == 0) {
		ov5623hw_auto_function_on();
	} else {
		ov5623hw_auto_function_off();

		/* manual calculate the exp */
		ov5623hw_calculate_exp(camera_context->capture_input_width, 
			camera_context->capture_input_height);
	}

	return 0;
}

static int ov5623_stop_capture(p_camera_context_t camera_context)
{
	(void) camera_context;

	G_ov5623_mode_change = camera_context->capture_mode;
	
	if(CAMERA_MODE_VIDEO == camera_context->capture_mode){
		ov5623hw_field_protect(camera_context->capture_input_width, 
			camera_context->capture_input_height);
	}

	return 0;
}

static int ov5623_set_power_mode(p_camera_context_t camera_context, u8 mode)
{
	(void) camera_context;

	ov5623hw_power_down(mode );
	return 0;
}

static int ov5623_read_8bit(p_camera_context_t camera_context,
		u8 reg_addr,  u8 *reg_val)
{
	(void) camera_context;

	read_sensor_reg(reg_addr, reg_val);

	return 0;
}

static int ov5623_write_8bit( p_camera_context_t camera_context,
		u8 reg_addr,  u8 reg_val)
{
	u8 buffer;
	int status;

	(void) camera_context;

	/* FIXME
	 * Turn off sensor AE/AWB if any gain is set
	 */
	if(reg_addr==0x0||reg_addr==0x01||reg_addr==0x02||reg_addr==0x16){
		ov5623hw_auto_function_off();
	}

	buffer = reg_val;
	status = (write_sensor_reg(reg_addr, &buffer) == 0) ?
		0 : -EIO;

	return status;
}

static int ov5623_set_contrast( p_camera_context_t camera_context, u8 mode, u32 value)
{
	(void) camera_context;

	if (mode == SENSOR_MANUAL_CONTRAST)
		return ov5623hw_set_contrast(value);
	else 
		return 0;        
}

static int ov5623_set_exposure( p_camera_context_t camera_context, u8 mode, u32 value)
{
	(void) camera_context;

	if (mode == SENSOR_MANUAL_EXPOSURE)
		return ov5623hw_set_exposure(value);
	else if(mode == SENSOR_AUTO_EXPOSURE){
		return ov5623hw_auto_ae(value);
	}
	else
		return 0;
}

static int ov5623_set_white_balance( p_camera_context_t camera_context,
		u8 mode, u32 value)
{
	(void) camera_context;

	if (mode == SENSOR_MANUAL_WHITEBALANCE){
		return ov5623hw_set_gain(value);
	}else if(mode == SENSOR_AUTO_WHITEBALANCE){
		return ov5623hw_auto_awb(value);
	}else
		return 0;
}

static int ov5623_get_framerate(u32 format, u32 width, u32 height, u32 *numerator, u32 *denominator)
{
	int ret = 0;
	switch (format) {
		case V4L2_PIX_FMT_SRGGB10:
			if ((width <= 800) && (height <= 600)) {
				/* CIF */
				*numerator = 15;	
				*denominator = 1;
			} else if ((width <= 1600) && (height <= 1200)) {
				/* UXGA */
				*numerator = 15;	
				*denominator = 2;
				
			} else if ((width <= 2560) && (height <= 1920)) {
				/* FULL */
				*numerator = 5;	
				*denominator = 2;				
			}else{
				ret = -1;
			}
			break;
			
		default:
			ret = -1;
			break;
		}

	return ret;
}

static char name[] = "Omnivision5623";

static p_camera_function_t camera_functions;

static int format_list[] = {
	V4L2_PIX_FMT_SRGGB10,
	-1	
};

static int ov5623_register(int id)
{

	/* allocte camera functions context */
	camera_functions = kzalloc(sizeof(camera_function_t), GFP_KERNEL);
	if (!camera_functions) {
		printk("Can't allocate buffer for camera functions structure \n");
		return -ENOMEM;
	}

	camera_functions->width_max = 2560;
	camera_functions->height_max = 1920;
	camera_functions->width_min = 2;
	camera_functions->height_min = 2;

	camera_functions->v_power = 2500;
	camera_functions->v_io = 1800;
		

	camera_functions->format_list = format_list;

	camera_functions->init = ov5623_init;     	
	camera_functions->deinit = ov5623_deinit;     
	camera_functions->set_capture_format = ov5623_set_capture_format; 	
	camera_functions->start_capture =	ov5623_start_capture;	 		
	camera_functions->stop_capture =	ov5623_stop_capture; 			
	camera_functions->sleep = ov5623_sleep;	
	camera_functions->wakeup = ov5623_wake;	
	camera_functions->read_8bit = ov5623_read_8bit;			
	camera_functions->write_8bit = ov5623_write_8bit;			
	camera_functions->read_16bit = NULL;		
	camera_functions->write_16bit = NULL;		
	camera_functions->read_32bit = NULL;	
	camera_functions->write_32bit = NULL;	
	camera_functions->set_power_mode = ov5623_set_power_mode;			
	camera_functions->set_contrast =	ov5623_set_contrast;			
	camera_functions->set_whitebalance =	ov5623_set_white_balance;		
	camera_functions->set_exposure =	 ov5623_set_exposure;			
	camera_functions->set_zoom = NULL;
	camera_functions->load_lut = ov5623_load_lut;

	camera_functions->get_framerate = ov5623_get_framerate;
	
	camera_functions->name = name;
	camera_functions->id = id;

	/*register 5623 as high resolution */

	if(sensor_register(camera_functions, id) < 0){
		printk("sensor_register failed !\n");
		kfree(camera_functions);
		camera_functions = NULL;
		return -1;
	}
	return 0;
}

static int ov5623_unregister(int id)
{
	sensor_unregister(id);
	
	if(!camera_functions){
		kfree(camera_functions);
		camera_functions = NULL;
	}
	return 0;
}

/*****************************************************************************
 *                              OV5623 I2C Client Driver
 *****************************************************************************/


static int i2c_read(struct i2c_client *client, u8 reg)
{
	int err = 0;

	err = i2c_smbus_write_byte(client, reg);
	if(err){
		printk("sensor i2c read error\n");
		return err;
	}
		
	return i2c_smbus_read_byte(client);
}
static int __devinit sensor_probe(struct i2c_client *client,
	const struct i2c_device_id *_id)
{
	struct sensor_platform_data *pdata;
	struct clk *clk = NULL;
	u8 pidh = 0, pidl = 0;
	int id = 0;

	pdata = client->dev.platform_data;
	id = pdata->id;
	
	clk = clk_get(NULL, "CAMCLK");
	if (IS_ERR(clk)) {
		printk("sensor failed to get camera clock\n");
		return -1;
	}
	clk_enable(clk);

	ci_set_clock(1, 1, CICLK);
	
	pdata->power_on(id);
	mdelay(1);
	
	pidh = i2c_read(client, OV5623_PIDH);
	pidl = i2c_read(client, OV5623_PIDL);
	if ((pidh != PID_OV56XX) || ((pidl != PID_5623 )&&(pidl != PID_5623_1))){
		goto err;
	}

	if(ov5623_register(id)<0)
		goto err;

	g_client = client;

	/*to avoid conflict between ov5623 and ov7673*/
	i2c_smbus_write_byte_data(client, OV5623_COM4, 0x0);

	printk("OV5623 detected.\n");
	
	ci_set_clock(0, 0, CICLK);
	pdata->power_off(id);

	clk_disable(clk);
	clk_put(clk);

	return 0;
err:
	printk("OV5623 detect failed.\n");
	ci_set_clock(0, 0, CICLK);
	pdata->power_off(id);
	if(clk){
		clk_disable(clk);
		clk_put(clk);

	}
	return -1;
}

static int sensor_remove(struct i2c_client *client)
{
	struct sensor_platform_data *pdata;
	pdata = client->dev.platform_data;
	
	ov5623_unregister(pdata->id);
	return 0;
}

static const struct i2c_device_id sensor_ov5623_id[] = {
	{ "sensor_ov5623", 0 },
	{ }
};

static struct i2c_driver sensor_driver = {
	.driver = {
		.name	= "sensor_ov5623",
	},
	.probe		= sensor_probe,
	.remove		= sensor_remove,
	.id_table	= sensor_ov5623_id,
};

static int __init sensor_init(void)
{
	return i2c_add_driver(&sensor_driver);
}

static void __exit sensor_exit(void)
{
	i2c_del_driver(&sensor_driver);
}

module_init(sensor_init);
module_exit(sensor_exit);

MODULE_DESCRIPTION("OV5623 I2C Client driver");
MODULE_LICENSE("GPL");

