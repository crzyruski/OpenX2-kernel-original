/*
   Copyright (C) 2005, Intel Corporation.

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


 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

/*****************************************************************************
 * Abstract:
 *	contains all hardware related functions for OV2630
 *
 * Notes:Only valid for processor code named Monahans.
 *****************************************************************************/
#include <linux/init.h>
#include <linux/module.h>
#include "camera.h"
#include <linux/delay.h>

#include <mach/mfp.h>
#include <mach/hardware.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <mach/zylonite.h>

#include <asm/gpio.h>
#include <linux/i2c.h>
#include <mach/camera.h>

/*****************************************************************************
 * Constants & Structures
 *****************************************************************************/
// CICLK for QCI
#define CICLK	2600

/* Revision constants */
#define PIDH_OV2630              0x26
#define PIDL_OV2630              0x33

/* OV2630 Mode */
typedef enum {
	OV2630_UXGA = 0,
	OV2630_CIF,
	OV2630_SVGA,
	OV2630_INVALID
}OV2630_MODE;

#define OV2630_GAIN             0x00    /* AGC Gain Control */
#define OV2630_BLUE_GAIN        0x01    /* Blue Gain Control */
#define OV2630_RED_GAIN         0x02    /* Red Gain Control */
#define OV2630_COMA             0x03    /* Common Control A */
#define OV2630_COMB             0x04    /* Common Control B */
#define OV2630_BAVG             0x05    /* B Channel Average */
/* G Channel Average - Picked G pixels in the same line with B pixels */
#define OV2630_GbAVE            0x06
/* G Channel Average - Picked G pixels in the same line with R pixels */
#define OV2630_GrAVE            0x07
#define OV2630_RAVG             0x08	/* R Channel Average */
#define OV2630_COMC             0x09	/* Common control C */
#define OV2630_PIDH             0x0A    /* Product ID Number MSBs */
#define OV2630_PIDL             0x0B    /* Product ID Number LSBs */
#define OV2630_COMD             0x0C    /* Common control D */
#define OV2630_COME             0x0D    /* Common control E */
#define OV2630_COMF             0x0E    /* Common control F */
#define OV2630_COMG             0x0F    /* Common control G */
#define OV2630_AEC              0x10    /* Automatic Exposure [10:3] */
#define OV2630_CLKRC            0x11    /* Clock Rate Control */
#define OV2630_COMH             0x12    /* Common Control H */
#define OV2630_COMI             0x13    /* Common control I */
#define OV2630_COMJ             0x14    /* Common Control J */
#define OV2630_COMK             0x15    /* Common Control K */
#define OV2630_HREFST           0x17    /* Horizontal Window Start */
#define OV2630_HREFEND          0x18    /* Horizontal window End */
#define OV2630_VSTRT            0x19    /* Vertical Window Line Start */
#define OV2630_VEND             0x1A    /* Vertical Window Line End */
#define OV2630_PSHFT            0x1B    /* Pixel Shift */
#define OV2630_MIDH             0x1C    /* Manufacturer ID Byte - High */
#define OV2630_MIDL             0x1D    /* Manufacturer ID Byte - Low */
#define OV2630_BOFF             0x20    /* B Channel Offset Adjustment */
#define OV2630_GbOFF            0x21    /* Gb Channel Offset Adjustment */
#define OV2630_GrOFF            0x22    /* Gr Channel Offset Adjustment */
#define OV2630_ROFF             0x23    /* R Channel Offset Adjustment */

/* Luminance Signal High Range for AEC/AGC Operation */
#define OV2630_AEW              0x24
/* Luminance Signal Low Range for AEC/AGC Operation */
#define OV2630_AEB              0x25
/* Fast Mode Large Step Range Threshold */
#define OV2630_VV               0x26
/* B Channel Offset Manual Adjustment Value */
#define OV2630_BBIAS            0x27
/* Gb Channel Offset Manual Adjustment Value */
#define OV2630_GbBIAS           0x28
/* Gr Channel Offset Manual Adjustment Value */
#define OV2630_GrBIAS           0x29
#define OV2630_COML             0x2A    /* Common Control L */
/* Line Interval Adjustment Value LSB 8 bits */
#define OV2630_FRARL            0x2B
/* R Channel Offset Manual Adjustment Value */
#define OV2630_RBIAS            0x2C
/* VSYNC Pulse Width LSB 8 bits */
#define OV2630_ADVSL            0x2D
/* VSYNC Pulse Width MSB 8 bits */
#define OV2630_ADVSH            0x2E
#define OV2630_YAVG             0x2F    /* Luminance Average */
/* HSYNC Position and Width Start Point LSB 8 bits */
#define OV2630_HSDY             0x30
/* HSYNC Position and Width End Lower 8 bits */
#define OV2630_HENY             0x31
#define OV2630_COMM             0x32    /* Common Control M */
#define OV2630_REG33            0x33    /* Current Control */
#define OV2630_VCHG             0x36    /* Sensor Precharge Voltage Control */
#define OV2630_ADC              0x37    /* ADC Reference Control */
#define OV2630_ACOM             0x38    /* Analog Common Control */
#define OV2630_REG39            0x39    /* R39 */
#define OV2630_REG3A            0x3A    /* Sensor Internal Reference Control */
#define OV2630_REG3C            0x3C    /* Sensor Internal Reference Control */
/* Gb/Gr Channel Manual Offset Compensation */
#define OV2630_DKOFFBR          0x3E
#define OV2630_REG3F            0x3F    /* Sensor Offset Control */
#define OV2630_BBLC             0x40    /* B Channel Black Level Value */
#define OV2630_RBLC             0x41    /* R Channel Black Level Value */
#define OV2630_GbBLC            0x42    /* Gb Channel Black Level Value */

/* End of OV2630 register */
#define OV2630_REGEND        ( 0x8D + 1 )

/* Unique ID allocation */
static struct i2c_client *g_client;

/*****************************************************************************
 *  Register Settings
 *****************************************************************************/
const static u8 ov2630InitSetting[]=
{
	0x12,	0x80,
	0x0e,	0x0,
	0x0f,	0x42,
	0x13,	0xe7,
	0x14,	0x4e,
	0x24,	0x6c,
	0x25,	0x60,
	0x35,	0x90,
	0x36,	0x88,
	0x37,	0x44,
	0x3a,	0x90,
	0x3b,	0x14,
	0x3f,	0x0f,
	0x40,	0x0,
	0x41,	0x0,
	0x42,	0x0,
	0x43,	0x0,
	0x44,	0x80,
	0x4b,	0x00,
	0x4c,	0x28,
	0x50,	0xf4,
	0x58,	0x7,
	0x59,	0x20,
	0x5f,	0x40,
	0x75,	0x0f,
	0x78,	0x40,
	0x7a,	0x10,
	0x84,	0x4,
	0x86,	0x20,
	0x88,	0x0c,
	0x89,	0x08,
	0x8a,	0x2,
	OV2630_REGEND,   0x00
};

/***********************************************************************
 *  Register Settings, Get from OmniVision
 ***********************************************************************/
static u8 UXGA_pxa310[] =
{
	0x11,   0x01,
	0x1e,	0x40,
	0x32,	0x0b,
};
static u8 UXGA_pxa300[] =
{
	0x11,	0x02,   /* workaroud for issue 193451 */
	0x1e, 	0xc0,
	0x32,	0x1b,
};

static u8 UXGA[] =
{
	0x34,	0xf0,
	0x03,	0x48,
	0x17,	0x2d,
	0x18,	0x02,
	0x19,	0x01,
	0x1a,	0x97,
	0x4d,	0xc0,
	0x5a,	0x00,
	0x87,	0x10,

	/* for non-zoom */
	0x0c,	0x21,
	0x16,	0x00,
	0x12,	0x00,
	0x48,	0x80,
	0x4a,	0x00,
	0x4e,	0x18,
	0x4f,	0x08,
	OV2630_REGEND,   0x00
};

static u8 SVGA[] =
{
	0x11,	0x01,
	0x34,   0x70,
	0x3,	0x0e,
	0x17,	0x3f,
	0x18,	0x02,
	0x19,	0x0,
	0x1a,	0x4b,
	0x1e,	0x40,
	0x32,	0x1f,
	0x4d,	0xc0,
	0x5a,	0x00,
	0x87,	0x00,

	/* for non-zoom */
	0x0c,	0xa0,
	0x16,	0x00,
	0x12,	0x41,
	0x48,	0x00,
	0x4a,	0x00,
	0x4e,	0x08,
	0x4f,	0x00,
	OV2630_REGEND,   0x00
};

static u8 CIF[] =
{
	0x11,	0x04,
	0x34,	0x70,
	0x03,	0x0a,
	0x17,	0x3f,
	0x18,	0x01,
	0x19,	0x00,
	0x1a,	0x25,
	0x1e,	0x40,
	0x32,	0xbf,
	0x4d,	0xc0,
	0x5a,	0x80,
	0x87,	0x00,

	/* for non-zoom */
	0x0c,	0xa0,
	0x16,	0x00,
	0x12,	0x21,
	0x48,	0x00,
	0x4a,	0x00,
	0x4e,	0x08,
	0x4f,	0x00,
	OV2630_REGEND,   0x00
};

const static u8 gSensorSlaveAddr = 0x30;
static int read_sensor_reg( const u8 subAddress, u8 *bufP );
static int write_sensor_reg( const u8 subAddress, u8 *bufP );


/*****************************************************************************
 *  Sensor read/write
 *****************************************************************************/

static int rmw_sensor_reg( const u8 subAddress, u8 *bufP,
		u8 andMask, u8 orMask )
{
	int status;
	status = read_sensor_reg( subAddress, bufP );
	if (!status) {
		*bufP &= andMask;
		*bufP |= orMask;
		status = write_sensor_reg( subAddress, bufP );
	}
	return status;
}

int ov2630hw_read_sensor_reg( const u8 subAddress, u8 *bufP )
{
	return read_sensor_reg(subAddress, bufP);
}

int ov2630hw_write_sensor_reg( const u8 subAddress, u8 *bufP )
{
	return write_sensor_reg(subAddress, bufP);
}

int ov2630hw_set_regs( const u8 *regP )
{
	u32 curReg = 0;
	int    status = 0;

	/* The list is a register number followed by the value */
	while( regP[curReg << 1] < OV2630_REGEND )
	{
		u8 regVal = regP[(curReg << 1) + 1];

		status = (write_sensor_reg( regP[curReg << 1], &regVal ) == 0) ?
			0 : -EIO;

		if( curReg == 0 )
			mdelay( 5 );

		curReg++;
	}

	return status;
}

int ov2630hw_read_all_regs( u8 *bufP, u32 numRegs )
{
	u32 curReg;

	for( curReg = 0; curReg < numRegs; curReg++, bufP++ )
		read_sensor_reg( (u8)curReg, bufP );


	return 0;
}

static void sensor_power_on(void)
{
	struct sensor_platform_data *pdata;
	pdata = g_client->dev.platform_data;

	pdata->power_on(pdata->id);

}

static void sensor_power_off(void)
{
	struct sensor_platform_data *pdata;
	pdata = g_client->dev.platform_data;

	pdata->power_off(pdata->id);
}
/*****************************************************************************
 *  Power & Reset
 *****************************************************************************/
void ov2630hw_power_down( u8 powerMode )
{
	if( powerMode == CAMERA_POWER_OFF )
		sensor_power_off();

	else
		sensor_power_on();

	mdelay(100);
}

void ov2630hw_reset(void)
{
	ov2630hw_set_regs(ov2630InitSetting);
	return;
}

/*****************************************************************************
 *  Settings
 *****************************************************************************/
int ov2630hw_version_revision(u8 * pCmRevision, u8 *pSensorRevision)
{
	read_sensor_reg( OV2630_PIDH, pCmRevision );
	read_sensor_reg( OV2630_PIDL, pSensorRevision );
	return 0;
}

void ov2630hw_set_hsync(void)
{
	u8 val;

	/* Makes HREF become HSYNC */
	read_sensor_reg( OV2630_COMK, &val );
	val |= 0x40;
	write_sensor_reg( OV2630_COMK, &val );
}

void ov2630hw_auto_function_on(void)
{
	u8 val;
	read_sensor_reg( OV2630_COMI, &val );
	val |= 0x07;    /* don't disturb AWB */
	write_sensor_reg( OV2630_COMI, &val );
}

void ov2630hw_auto_function_off(void)
{
	u8 val;
	read_sensor_reg( OV2630_COMI, &val );
	val &= ~0x07;    /* don't disturb AWB */
	write_sensor_reg( OV2630_COMI, &val );
}


/*****************************************************************************
 *  Viewfinder, still
 *****************************************************************************/
int ov2630hw_viewfinder_on(void)
{
	u8 com3;

	read_sensor_reg( OV2630_COMD, &com3 );
	com3 &= ~0x01;
	write_sensor_reg( OV2630_COMD, &com3 );

	return 0;
}


int ov2630hw_viewfinder_off(void)
{
	u8 com3;


	read_sensor_reg( OV2630_COMD, &com3 );
	com3 |= 0x01;
	write_sensor_reg( OV2630_COMD, &com3 );

	return 0;
}


int ov2630hw_halt_video_output(void)
{
	u8 com3;

	/* Set the camera to only output 1 frame */
	read_sensor_reg( OV2630_COMD, &com3 );
	com3 |= 1;
	write_sensor_reg( OV2630_COMD, &com3 );

	return 0;
}

int ov2630hw_resumeto_full_output_mode(void)
{
	u8 mode;

	/* Output still frames continuously
	 * Turn off single capture mode COM3.
	 */
	rmw_sensor_reg( OV2630_COMD, (&mode), ((u8) ~1), 0 );
	return 0;
}

int ov2630hw_get_single_image(void)
{
	u8 mode;

	rmw_sensor_reg( OV2630_COMD, &mode, (u8) ~1, 1 );
	return 0;
}

/*****************************************************************************
 *  Format
 *****************************************************************************/
int ov2630hw_set_format(u32 captureWidth,
		u32 captureHeight,
		u32 *winStartX,
		u32 *winStartY,
		u32 *winEndX,
		u32 *winEndY)
{
	OV2630_MODE mode;
	unsigned short hStart;
	unsigned short vStart;
	unsigned char regVal;

	/* let the sensor work on proper mode */
	if((captureWidth <= 400) && (captureHeight <= 292)){
		mode = OV2630_CIF;
	} else if((captureWidth <= 800) && (captureHeight <= 600)) {
		mode = OV2630_SVGA;
	} else if((captureWidth <= 1600) && (captureHeight <= 1200)) {
		mode = OV2630_UXGA;
	} else {
		return -EINVAL;
	}

	if (mode == OV2630_CIF){
		ov2630hw_set_regs(CIF);
	}else if (mode == OV2630_SVGA){
		ov2630hw_set_regs(SVGA);
	}else{
		ov2630hw_set_regs(UXGA);
		if(cpu_is_pxa310()){
			ov2630hw_set_regs(UXGA_pxa310);

		}
		if(cpu_is_pxa300()){
			ov2630hw_set_regs(UXGA_pxa300);

		}
	}

	/* set cropping window */
	if (mode == OV2630_CIF) {
		captureWidth *= 2;
	}


	if (mode == OV2630_CIF){
		hStart = (unsigned short)(511 + (800 -captureWidth)/2);
		vStart = (unsigned short)(2 + (292 - captureHeight)/4);
	} else if (mode == OV2630_SVGA){
		hStart = (unsigned short)(511 + (800 -captureWidth)/2);
		vStart = (unsigned short)(2 + (600 - captureHeight)/4);
	} else {
		hStart = (unsigned short)(363 + (1600 -captureWidth)/2);
		vStart = (unsigned short)(4 + (1200 - captureHeight)/4);
	}

	/* set Horizontal Window Start */
	regVal = hStart>>3;
	write_sensor_reg(OV2630_HREFST, &regVal);
	read_sensor_reg(OV2630_COMM, &regVal);
	regVal &= ~0x07;
	regVal |= hStart & 0x07;
	write_sensor_reg(OV2630_COMM, &regVal);


	// set Vertical Window Start
	regVal = vStart>>2;
	write_sensor_reg(OV2630_VSTRT, &regVal);
	read_sensor_reg(OV2630_COMA, &regVal);
	regVal &= ~0x03;
	regVal |= vStart & 0x03;
	write_sensor_reg(OV2630_COMA, &regVal);

	/* return window region */
	*winStartX = hStart;
	*winStartY = vStart;
	*winEndX   = hStart + captureWidth;
	*winEndY   = vStart + captureHeight;

	return 0;
}

/*****************************************************************************
 * Contrast
 *****************************************************************************/
/* FIX ME: TBD */
const static u8 ContrastLowestSettings[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

const static u8 ContrastLowSettings[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

const static u8 ContrastMiddleSettings[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

const static u8 ContrastHighSettings[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

const static u8 ContrastHighestSettings[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

int ov2630hw_set_contrast(u32 value)
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
		ov2630hw_set_regs(regP);
	return 0;
}

/*****************************************************************************
 * Exposure
 *****************************************************************************/
/* FIX ME: TBD */
const static u8 ExposureSettings[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

int ov2630hw_set_exposure(u32 value)
{
	u8 aew, aeb, vv;
	int index = -1;

	switch(value) {
		case SENSOR_EXPOSURE_LOWEST:
			index = 0;
			break;
		case SENSOR_EXPOSURE_LOW:
			index = 3;
			break;
		case SENSOR_EXPOSURE_MIDDLE:
			index = 6;
			break;
		case SENSOR_EXPOSURE_HIGH:
			index = 9;
			break;
		case SENSOR_EXPOSURE_HIGHEST:
			index = 12;
			break;
		default:
			break;
	}

	aew = aeb = vv = 0;

	if (index != -1){
		aew = ExposureSettings[index];
		aeb = ExposureSettings[index + 1];
		vv  = ExposureSettings[index + 2];
	}

	/* set hw */
	if( aew || aeb || vv){
		ov2630hw_write_sensor_reg( OV2630_AEW, &aew);
		ov2630hw_write_sensor_reg( OV2630_AEB, &aeb);
		ov2630hw_write_sensor_reg( OV2630_VV, &vv);
	}

	return 0;
}

/*****************************************************************************
 * Auto White Balance
 *****************************************************************************/
/* FIX ME: TBD */
const static u8 AWBAuto[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

const static u8 AWBFluorescent[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

const static u8 AWBOutdoor[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

const static u8 AWBIncandescent[] = {
	/* need to follow up */
	OV2630_REGEND,     0x00        /* End of list delimiter */
};

int ov2630hw_set_white_balance(u32 value)
{
	const u8 *regP;

	regP = NULL;
	switch(value) {
		case SENSOR_WHITEBALANCE_AUTO:		/* Auto */
			regP = AWBAuto;
			break;
		case SENSOR_WHITEBALANCE_INCANDESCENT:	/* Incandescent */
			regP = AWBIncandescent;
			break;
		case SENSOR_WHITEBALANCE_SUNNY:		/* Sunny */
			regP = AWBOutdoor;
			break;
		case SENSOR_WHITEBALANCE_FLUORESCENT:	/* Fluorescent */
			regP = AWBFluorescent;
			break;
		default:
			break;
	}

	/* set hw */
	if (regP) {
		ov2630hw_set_regs(regP);
	}
	return 0;
}

/*****************************************************************************
 * OV7660 Functions
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

/* CMU conversion matrixes, they're sensor specific */
static CI_CMU_COE_MATRIX ov2630_cRGB_sRGB_COE = {
	RGB_FLOAT_TO_INT(0),  RGB_FLOAT_TO_INT(0), RGB_FLOAT_TO_INT(1),
	RGB_FLOAT_TO_INT(0) ,  RGB_FLOAT_TO_INT(1), RGB_FLOAT_TO_INT(0),
	RGB_FLOAT_TO_INT(1),  RGB_FLOAT_TO_INT(0), RGB_FLOAT_TO_INT(0)
};


static CI_CMU_COE_MATRIX ov2630_cRGB_YUV_COE = {
	YUV_FLOAT_TO_INT(0.098),  YUV_FLOAT_TO_INT(0.504),
	YUV_FLOAT_TO_INT(0.257),

	YUV_FLOAT_TO_INT(0.439),  YUV_FLOAT_TO_INT(-0.291),
	YUV_FLOAT_TO_INT(-0.148),

	YUV_FLOAT_TO_INT(-0.071), YUV_FLOAT_TO_INT(-0.368),
	YUV_FLOAT_TO_INT(0.439),
};

/* Bad pixel table, it's sensor specific, TBD */
#define END_MARKER -1
static int ov2630_badpixel_table[] = {
	/*  column,        row
	 *    0,           0,
	 *    1,           1,
	 *    1615,        1207,
	 */
	END_MARKER, END_MARKER
};


/*****************************************************************************
 * OV2630 Functions
 *****************************************************************************/
int ov2630_init( p_camera_context_t camera_context )
{
	u8 cm_rev, cm_pid;
	u32 timeout;
	int status;

	/* provide informat about the capabilities of the sensor */
	camera_context->sensor_status.caps |= SENSOR_CAP_MANUAL_CONTRAST |
		SENSOR_CAP_MANUAL_WHITEBALANCE |
		SENSOR_CAP_MANUAL_EXPOSURE;

	/* Configure CI according to OV2630's hardware
	 * master parallel with 8 data pins
	 */
	ci_set_mode(CI_MODE_MP, CI_DATA_WIDTH10);

	/* enable pixel clock(sensor will provide pclock)
	 * and master clock = 26MHZ
	 */
	/* software work around*/
	ci_set_clock(1, 1, CICLK);

	/* data sample on rising and h,vsync active high */
	ci_set_polarity(0, 0, 0);

	/* fifo control */
	ci_set_fifo(0, CI_FIFO_THL_32, 1, 1);

	/* set black level */
	ci_cgu_set_black_level(0);

	/* CGU Mux Select */
	ci_cgu_set_addr_mux_select(CI_CGU_MUX_2_TO_9);

	/* OV2630 Power on sequence
	 * Take out of Power down mode (GPIO_57), PWRDWN=1, NORMAL=0
	 * Assert Reset
	 * Delay
	 * Remove reset
	 * Delay
	 */
	ov2630hw_power_down(CAMERA_POWER_FULL );
	ov2630hw_reset();

	/* read out version */
	timeout = 50;
	do {
		cm_pid = cm_rev = 0;
		status = ov2630hw_version_revision(&cm_pid, &cm_rev);

		/* Check to make sure we are working with an OV2630 */
		if( cm_pid != PIDH_OV2630 || cm_rev != PIDL_OV2630 )
		{
			ov2630hw_power_down( CAMERA_POWER_OFF );
			ov2630hw_power_down( CAMERA_POWER_FULL );
			ov2630hw_reset();
			mdelay(1);
		}
		if (--timeout == 0){
			return -EIO;
		}
	} while( cm_pid != PIDH_OV2630 );

	/* turn sensor output off */
	ov2630hw_viewfinder_off();

	return 0;
}

int ov2630_deinit(  p_camera_context_t camera_context )
{
	/* power off the external module */
	ov2630hw_power_down( CAMERA_POWER_OFF );

	return 0;
}

int ov2630_sleep(  p_camera_context_t camera_context )
{
	return ov2630_deinit( camera_context );
}

int ov2630_wake(  p_camera_context_t camera_context )
{
	return ov2630_init( camera_context );
}

int ov2630_set_capture_format(  p_camera_context_t camera_context )
{
	CI_MP_TIMING timing;
	u32 winStartX, winStartY;
	u32 winEndX, winEndY;
	int           *padPixelX, *padPixelY;
	int           badPixelNum;

	/* Set CMU Coe matrix, if necessary */
	if((camera_context->capture_input_format !=
			V4L2_PIX_FMT_SRGGB10)&&
		(camera_context->capture_input_format !=
			V4L2_PIX_FMT_SRGGB8))
		return -EINVAL;

	if (camera_context->cmu_usage == CI_CMU_OUTPUT_YUV) {
		ci_cmu_set_color_correction_coe(&ov2630_cRGB_YUV_COE);
	}

	if (camera_context->cmu_usage == CI_CMU_OUTPUT_RGB) {
		ci_cmu_set_color_correction_coe(&ov2630_cRGB_sRGB_COE);
	}

	ci_cmu_enable(camera_context->cmu_usage);

	/* Set OV2630 format */
	ov2630hw_set_format(
			camera_context->capture_input_width,
			camera_context->capture_input_height,
			&winStartX,
			&winStartY,
			&winEndX,
			&winEndY);

	/* configure PSU, if necessary */
	if (camera_context->psu_enable){
		ci_psu_enable(0);

		padPixelX = ov2630_badpixel_table;
		padPixelY = ov2630_badpixel_table + 1;
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
					/* PSU only can substitute
					 * 128 bad pixels!!
					 */
					if (badPixelNum == 128)
						break;
				}
			}

			padPixelX += 2;
			padPixelY += 2;
		}

		/* fill the last PSU RAM location with all 0's,
		 * signifying the end of the dead pixel addresses
		 * stored in the RAM
		 */
		if (badPixelNum < 128) {
			ci_psu_tag_bad_pixel(0, 0);
		}

		if (badPixelNum != 0){
			ci_psu_enable(1);
		}
		else{
			camera_context->psu_enable = 0;
		}
	}


	/* set capture width/height and timing */
	/* this is ok for raw data */
	timing.BFW = 0x01;
	timing.BLW = 0x01;   /* GRBG to RGGB */


	ci_configure_mp(camera_context->capture_input_width-1,
		camera_context->capture_input_height-1, &timing);

	if (cpu_is_pxa310())
		ci_set_ycbcr_420_down_sample (camera_context->ycbcr_ds);

	return 0;
}

static int ov2630_load_lut(p_camera_context_t camera_context)
{
	if(camera_context->cgu_enable){
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

int ov2630_start_capture(p_camera_context_t camera_context,unsigned int frames)
{
	(void) camera_context;

	if(camera_context->cgu_enable){
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
		ov2630hw_auto_function_on();
	} else {
		ov2630hw_auto_function_off();
	}

	/* turn viewfinder on */
	ov2630hw_viewfinder_on();
	return 0;
}

int ov2630_stop_capture(  p_camera_context_t camera_context )
{
	(void) camera_context;

	/* turn auto function off */
	ov2630hw_auto_function_off();

	/* turn viewfinder off */
	ov2630hw_viewfinder_off();

	return 0;
}

int ov2630_set_power_mode(p_camera_context_t camera_context, u8 mode)
{
	(void) camera_context;

	ov2630hw_power_down(mode );
	return 0;
}

int ov2630_read_8bit( p_camera_context_t camera_context,
		u8 reg_addr,  u8 *reg_val)
{
	(void) camera_context;

	ov2630hw_read_sensor_reg( reg_addr, reg_val );
	return 0;
}

int ov2630_write_8bit( p_camera_context_t camera_context,
		u8 reg_addr,  u8 reg_val)
{
	u8 buffer;
	int status;

	(void) camera_context;

	buffer = reg_val;
	status = (ov2630hw_write_sensor_reg( reg_addr, &buffer ) == 0) ?
		0 : -EIO;

	return status;
}

int ov2630_set_contrast( p_camera_context_t camera_context, u8 mode, u32 value)
{
	(void) camera_context;

	if (mode == SENSOR_MANUAL_CONTRAST)
		return ov2630hw_set_contrast(value);
	else
		return 0;
}

int ov2630_set_exposure( p_camera_context_t camera_context, u8 mode, u32 value)
{
	(void) camera_context;

	if (mode == SENSOR_MANUAL_EXPOSURE)
		return ov2630hw_set_exposure(value);
	else
		return 0;
}

int ov2630_set_white_balance( p_camera_context_t camera_context,
		u8 mode, u32 value)
{
	(void) camera_context;

	if (mode == SENSOR_MANUAL_WHITEBALANCE)
		return ov2630hw_set_white_balance(value);
	else
		return 0;
}

int ov2630_get_framerate(u32 format, u32 width, u32 height, u32 *numerator, u32 *denominator)
{
	int ret = 0;
	switch (format) {
		case V4L2_PIX_FMT_SRGGB10:
			if ((width <= 400) && (height <= 292)) {
				/* CIF */
				*numerator = 15;	
				*denominator = 1;
			} else if ((width <= 800) && (height <= 600)) {
				/* SVGA */
				*numerator = 15;	
				*denominator = 2;
				
			} else if ((width <= 1600) && (height <= 1200)) {
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

static char name[] = "Omnivision2630";

static int format_list[] = {
	V4L2_PIX_FMT_SRGGB10,
	-1	
};


static p_camera_function_t camera_functions;

int ov2630_register(int id)
{

	/* allocte camera functions context */
	camera_functions = kzalloc(sizeof(camera_function_t), GFP_KERNEL);
	if (!camera_functions) {
		printk("Can't allocate buffer for camera functions structure \n");
		return -ENOMEM;
	}

	camera_functions->format_list = format_list;

	camera_functions->width_max = 1600;
	camera_functions->height_max = 1200;
	camera_functions->width_min = 2;
	camera_functions->height_min = 2;

	camera_functions->v_power = 2800;
	
	camera_functions->init = ov2630_init;     	
	camera_functions->deinit = ov2630_deinit;     
	camera_functions->set_capture_format = ov2630_set_capture_format; 	
	camera_functions->start_capture =	ov2630_start_capture;	 		
	camera_functions->stop_capture =	ov2630_stop_capture; 			
	camera_functions->sleep = ov2630_sleep;	
	camera_functions->wakeup = ov2630_wake;	
	camera_functions->read_8bit = ov2630_read_8bit;			
	camera_functions->write_8bit = ov2630_write_8bit;			
	camera_functions->read_16bit = NULL;		
	camera_functions->write_16bit = NULL;		
	camera_functions->read_32bit = NULL;	
	camera_functions->write_32bit = NULL;	
	camera_functions->set_power_mode = ov2630_set_power_mode;			
	camera_functions->set_contrast =	ov2630_set_contrast;			
	camera_functions->set_whitebalance =	ov2630_set_white_balance;		
	camera_functions->set_exposure =	 ov2630_set_exposure;			
	camera_functions->set_zoom = NULL;
	camera_functions->load_lut = ov2630_load_lut;

	camera_functions->get_framerate = ov2630_get_framerate;

	camera_functions->name = name;
	camera_functions->id = id;

	/*register 2630 as HIGH resolution */

	if(sensor_register(camera_functions, id) < 0){
		printk("sensor_register failed !\n");
		kfree(camera_functions);
		camera_functions = NULL;
		return -1;
	}
	return 0;
}

int ov2630_unregister(int id)
{
	sensor_unregister(id);
	
	if(!camera_functions){
		kfree(camera_functions);
		camera_functions = NULL;
	}
	return 0;
}

/******************************************************************************
 *                              OV2630 I2C Client Driver
 ******************************************************************************/
static int read_sensor_reg( const u8 subAddress, u8 *bufP )
{
	int ret;

	if( g_client == NULL )	/* No global client pointer? */
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
	/*
	ret = i2c_smbus_read_byte_data(g_client, subAddress);
	if (ret >= 0) {
		*bufP = ret;
	}
	return ret;
	*/
}

static int write_sensor_reg( const u8 subAddress, u8 *bufP )
{
	if( g_client == NULL )	/* No global client pointer? */
		return -1;

	return i2c_smbus_write_byte_data(g_client, subAddress, *bufP);
}
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
	if(IS_ERR(clk)){
		printk("clk error\n");
	}
	if (!clk) {
		printk("sensor failed to get camera clock\n");
		return -1;
	}
	clk_enable(clk);
	
	ci_set_clock(1, 1, CICLK);
	
	pdata->power_on(id);
	mdelay(1);
	
	pidh = i2c_read(client, OV2630_PIDH);
	pidl = i2c_read(client, OV2630_PIDL);

	if ((pidh != PIDH_OV2630) || (pidl != PIDL_OV2630)){
		goto err;
	}

	if(ov2630_register(id)<0)
		goto err;

	g_client = client;

	printk("OV2630 detected.\n");
	
	ci_set_clock(0, 0, CICLK);
	pdata->power_off(id);

	clk_disable(clk);
	clk_put(clk);

	return 0;
err:
	printk("OV2630 detect failed.\n");
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
	
	ov2630_unregister(pdata->id);
	g_client = NULL;

	return 0;
}

static const struct i2c_device_id sensor_ov2630_id[] = {
	{ "sensor_ov2630", 0 },
	{ }
};

static struct i2c_driver sensor_driver = {
	.driver = {
		.name	= "sensor_ov2630",
	},
	.id_table	= sensor_ov2630_id,
	.probe		= sensor_probe,
	.remove		= sensor_remove,
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

MODULE_DESCRIPTION("OV2630 I2C Client driver");
MODULE_LICENSE("GPL");
