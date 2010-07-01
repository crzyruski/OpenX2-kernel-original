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
 *	Contains all hardware related functions for OV9653
 *
 * Notes:Only valid for processor code named Monahans.
 *****************************************************************************/
#include <linux/init.h>
#include <linux/module.h>
#include "camera.h"
#include <linux/delay.h>
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/err.h>


/*****************************************************************************
 * Constants & Structures
 *****************************************************************************/

#define CICLK 1300

/* Revision constants */
#define PID_OV96XX			0x96
#define PID_9653			0x52

/* Output Size & Format */
#define OV_SIZE_QQVGA			0x01
#define OV_SIZE_QVGA			( OV_SIZE_QQVGA << 1 )
#define OV_SIZE_VGA			( OV_SIZE_QQVGA << 2 )
#define OV_SIZE_SXGA			( OV_SIZE_QQVGA << 3 )
#define OV_SIZE_QQCIF			0x10
#define OV_SIZE_QCIF			( OV_SIZE_QQCIF << 1 )
#define OV_SIZE_CIF			( OV_SIZE_QQCIF << 2 )

#define OV_FORMAT_YUV_422		1
#define OV_FORMAT_RGB_565		2
#define OV_FORMAT_RAW8   		3

/* OV9653 Register Definitions */
#define OV9653_GAIN		0x0000
#define OV9653_BLUE		0x0001
#define OV9653_RED		0x0002
#define OV9653_VREF		0x0003
#define OV9653_COM1		0x0004
#define OV9653_BAVE		0x0005	/* U/B Average Level */
#define OV9653_GEAVE		0x0006	/* Y/Ge Average Level */
#define OV9653_GOAVE		0x0007	/* Y/Go Average Level */
#define OV9653_RAVE		0x0008	/* V/R Average level */
#define OV9653_COM2		0x0009	/* Common control 2 */
#define OV9653_PID		0x000A	/* Product ID */
#define OV9653_VER		0x000B	/* Version */
#define OV9653_COM3		0x000C
#define OV9653_COM4		0x000D
#define OV9653_COM5		0x000E
#define OV9653_COM6		0x000F
#define OV9653_AECH		0x0010
#define OV9653_CLKRC		0x0011
#define OV9653_COM7		0x0012
#define OV9653_COM8		0x0013
#define OV9653_COM9		0x0014
#define OV9653_COM10		0x0015
#define OV9653_WS		0x0016
#define OV9653_HSTART		0x0017
#define OV9653_HSTOP		0x0018
#define OV9653_VSTRT		0x0019
#define OV9653_VSTOP		0x001A
#define OV9653_PSHFT		0x001B
#define OV9653_MIDH		0x001C
#define OV9653_MIDL		0x001D
#define OV9653_DLY		0x001E
#define OV9653_LAEC		0x001F
#define OV9653_BOS		0x0020
#define OV9653_GBOS		0x0021
#define OV9653_GROS		0x0022
#define OV9653_ROS		0x0023
#define OV9653_AEW		0x0024
#define OV9653_AEB		0x0025
#define OV9653_VPT		0x0026
#define OV9653_BBIAS		0x0027
#define OV9653_GbBIAS		0x0028
#define OV9653_GrBIAS		0x0029
#define OV9653_EXHCH		0x002A
#define OV9653_EXHCL		0x002B
#define OV9653_RBIAS		0x002C
#define OV9653_ADVFL		0x002D
#define OV9653_ADVFH		0x002E
#define OV9653_YAVE		0x002F
#define OV9653_HSYST		0x0030
#define OV9653_HSYEN		0x0031
#define OV9653_HREF		0x0032
#define OV9653_CHLF		0x0033
#define OV9653_ARBLM		0x0034
#define OV9653_VRHL		0x0035
#define OV9653_VIDO		0x0036
#define OV9653_ADC		0x0037
#define OV9653_ACOM		0x0038
#define OV9653_OFON		0x0039
#define OV9653_TSLB		0x003A
#define OV9653_COM11		0x003B
#define OV9653_COM12		0x003C
#define OV9653_COM13		0x003D
#define OV9653_COM14		0x003E
#define OV9653_EDGE		0x003F
#define OV9653_COM15		0x0040
#define OV9653_COM16		0x0041
#define OV9653_COM17		0x0042
#define OV9653_AWBTH1		0x0043
#define OV9653_AWBTH2		0x0044
#define OV9653_AWBTH3		0x0045
#define OV9653_AWBTH4		0x0046
#define OV9653_AWBTH5		0x0047
#define OV9653_AWBTH6		0x0048
#define OV9653_RSVD49		0x0049
#define OV9653_MTX1		0x004F
#define OV9653_MTX2		0x0050
#define OV9653_MTX3		0x0051
#define OV9653_MTX4		0x0052
#define OV9653_MTX5		0x0053
#define OV9653_MTX6		0x0054
#define OV9653_MTX7		0x0055
#define OV9653_MTX8		0x0056
#define OV9653_MTX9		0x0057
#define OV9653_MTXS		0x0058
#define OV9653_AWBC1		0x0059
#define OV9653_AWBC2		0x005A
#define OV9653_AWBC3		0x005B
#define OV9653_AWBC4		0x005C
#define OV9653_AWBC5		0x005D
#define OV9653_AWBC6		0x005E
#define OV9653_AWBC7		0x005F
#define OV9653_AWBC8		0x0060
#define OV9653_AWBC9		0x0061
#define OV9653_LCC1		0x0062
#define OV9653_LCC2		0x0063
#define OV9653_LCC3		0x0064
#define OV9653_LCC4		0x0065
#define OV9653_LCC5		0x0066
#define OV9653_MANU		0x0067
#define OV9653_MANV		0x0068
#define OV9653_HV		0x0069
#define OV9653_MBD		0x006A
#define OV9653_DBLV		0x006B
#define OV9653_GSP0		0x006C
#define OV9653_GSP1		0x006D
#define OV9653_GSP2		0x006E
#define OV9653_GSP3		0x006F
#define OV9653_GSP4		0x0070
#define OV9653_GSP5		0x0071
#define OV9653_GSP6		0x0072
#define OV9653_GSP7		0x0073
#define OV9653_GSP8		0x0074
#define OV9653_GSP9		0x0075
#define OV9653_GSP10		0x0076
#define OV9653_GSP11		0x0077
#define OV9653_GSP12		0x0078
#define OV9653_GSP13		0x0079
#define OV9653_GSP14		0x007A
#define OV9653_GSP15		0x007B
#define OV9653_GST0		0x007C
#define OV9653_GST1		0x007D
#define OV9653_GST2		0x007E
#define OV9653_GST3		0x007F
#define OV9653_GST4		0x0080
#define OV9653_GST5		0x0081
#define OV9653_GST6		0x0082
#define OV9653_GST7		0x0083
#define OV9653_GST8		0x0084
#define OV9653_GST9		0x0085
#define OV9653_GST10		0x0086
#define OV9653_GST11		0x0087
#define OV9653_GST12		0x0088
#define OV9653_GST13		0x0089
#define OV9653_GST14		0x008A

#define OV9653_COM21		0x008B
#define OV9653_COM22		0x008C
#define OV9653_COM23		0x008D
#define OV9653_COM24		0x008E
#define OV9653_DBLC1		0x008F
#define OV9653_DBLC_B		0x0090
#define OV9653_DBLC_R		0x0091
#define OV9653_DM_LNL		0x0092
#define OV9653_DM_LNH		0x0093
#define OV9653_LCCFB		0x009D
#define OV9653_LCCFR		0x009E
#define OV9653_DBLC_GB		0x009F
#define OV9653_DBLC_GR		0x00A0
#define OV9653_AECHIM		0x00A1
#define OV9653_COM25		0x00A4
#define OV9653_COM26		0x00A5
#define OV9653_G_GAIN		0x00A6
#define OV9653_VGA_ST		0x00A7

/* End of OV9653 register */
#define OV9653_REGEND		( 0xAA + 1 )

/* Unique ID allocation */
static struct i2c_client *g_client;

/*****************************************************************************
 *  Register Settings
 *****************************************************************************/
const static u8 gRAW_CIF_INITIALIZE[] =
{
	/*30FPS*/
	0x12, 0x80,
	0x11, 0x87,
	0x6b, 0x4a,
	0x3b, 0x01,
	0x6a, 0x30,
	0x13, 0xE2,
	0x10, 0x00,
	0x00, 0x00,
	0x01, 0x80,
	0x02, 0x80,
	0x13, 0xE7,
	           
	0x39, 0x43,
	0x38, 0x12,
	0x37, 0x00,
	0x35, 0x91,
	0x0e, 0xa0,
	           
	0xA8, 0x80,
	0x12, 0x25,
	0x04, 0x00,
	0x0c, 0x04,
	0x0d, 0x80,
	0x18, 0x7c,
	0x17, 0x24,
	0x32, 0xa4,
	0x19, 0x00,
	0x1a, 0x24,
	0x03, 0x36,
	           
	0x1b, 0x00,
	0x16, 0x07,
	0x33, 0xe2,
	0x34, 0xbf,
	0x41, 0x00,
	0x96, 0x04,
	           
	0x3d, 0x19,
	0x69, 0x40,
	0x3a, 0x0d,
	0x8e, 0x00,
	           
	0x3c, 0x73,
	0x8f, 0xdF,
	0x8B, 0x06,
	0x8C, 0x20,
	0x94, 0x88,
	0x95, 0x88,
	0x40, 0xc1,
	0x29, 0x3f,
	0x0f, 0x42,
	0xA5, 0x80,
	0x1e, 0x04,
	0xa9, 0xb8,
	0xaa, 0x92,
	0xab, 0x0a,
	           
	0x90, 0x00,
	0x91, 0x00,
	0x9f, 0x00,
	0xa0, 0x00,
	           
	0x24, 0x68,
	0x25, 0x5c,
	0x26, 0xc3,
	0x14, 0x2e,
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gRAW_QCIF_INITIALIZE[] =
{
	/*30FPS*/
	0x12, 0x80,
	0x11, 0x8f,
	0x6b, 0x4a,
	0x3b, 0x01,
	0x6a, 0x18,
	0x13, 0xE2,
	0x10, 0x00,
	0x00, 0x00,
	0x01, 0x80,
	0x02, 0x80,
	0x13, 0xE7,
	           
	0x39, 0x43,
	0x38, 0x12,
	0x37, 0x00,
	0x35, 0x91,
	0x0e, 0xa0,
	           
	0xA8, 0x80,
	0x12, 0x0D,
	0x04, 0x00,
	0x0c, 0x04,
	0x0d, 0x80,
	0x18, 0x7c,
	0x17, 0x24,
	0x32, 0xa4,
	0x19, 0x00,
	0x1a, 0x12,
	0x03, 0x36,
	           
	0x1b, 0x00,
	0x16, 0x07,
	0x33, 0xe2,
	0x34, 0xbf,
	0x41, 0x00,
	0x96, 0x04,
	           
	0x3d, 0x19,
	0x69, 0x40,
	0x3a, 0x0d,
	0x8e, 0x00,
	           
	0x3c, 0x73,
	0x8f, 0xdF,
	0x8B, 0x06,
	0x8C, 0x20,
	0x94, 0x88,
	0x95, 0x88,
	0x40, 0xc1,
	0x29, 0x3f,
	0x0f, 0x42,
	0xA5, 0x80,
	0x1e, 0x04,
	0xa9, 0xb8,
	0xaa, 0x92,
	0xab, 0x0a,
	           
	0x90, 0x00,
	0x91, 0x00,
	0x9f, 0x00,
	0xa0, 0x00,
	           
	0x24, 0x68,
	0x25, 0x5c,
	0x26, 0xc3,
	0x14, 0x2e,
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gRAW_QQCIF_INITIALIZE[] =
{
	/*30FPS*/
	0x12, 0x80,
	0x11, 0x8f,
	0x6b, 0x4a,
	0x3b, 0x01,
	0x6a, 0x18,
	0x13, 0xE2,
	0x10, 0x00,
	0x00, 0x00,
	0x01, 0x80,
	0x02, 0x80,
	0x13, 0xE7,
	           
	0x39, 0x43,
	0x38, 0x12,
	0x37, 0x00,
	0x35, 0x91,
	0x0e, 0xa0,
	           
	0xA8, 0x80,
	0x12, 0x0D,
	0x04, 0x24,
	0x0c, 0x04,
	0x0d, 0x80,
	0x18, 0x7c,
	0x17, 0x24,
	0x32, 0xa4,
	0x19, 0x00,
	0x1a, 0x12,
	0x03, 0x36,
	           
	0x1b, 0x00,
	0x16, 0x07,
	0x33, 0xe2,
	0x34, 0xbf,
	0x41, 0x00,
	0x96, 0x04,
	           
	0x3d, 0x19,
	0x69, 0x40,
	0x3a, 0x0d,
	0x8e, 0x00,
	           
	0x3c, 0x73,
	0x8f, 0xdF,
	0x8B, 0x06,
	0x8C, 0x20,
	0x94, 0x88,
	0x95, 0x88,
	0x40, 0xc1,
	0x29, 0x3f,
	0x0f, 0x42,
	0xA5, 0x80,
	0x1e, 0x04,
	0xa9, 0xb8,
	0xaa, 0x92,
	0xab, 0x0a,
	           
	0x90, 0x00,
	0x91, 0x00,
	0x9f, 0x00,
	0xa0, 0x00,
	           
	0x24, 0x68,
	0x25, 0x5c,
	0x26, 0xc3,
	0x14, 0x2e,
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gRAW_QQVGA_INITIALIZE[] =
{
	/*30FPS*/
	0x12, 0x80,
	0x11, 0x87,
	0x6b, 0x4a,
	0x3b, 0x01,
	0x6a, 0x1f,
	0x13, 0xE2,
	0x10, 0x00,
	0x00, 0x00,
	0x01, 0x80,
	0x02, 0x80,
	0x13, 0xE7,
	           
	0x39, 0x43,
	0x38, 0x12,
	0x37, 0x00,
	0x35, 0x91,
	0x0e, 0xa0,
	           
	0xA8, 0x80,
	0x12, 0x15,
	0x04, 0x24,
	0x0c, 0x04,
	0x0d, 0x80,
	0x18, 0xc4,
	0x17, 0x24,
	0x32, 0xa4,
	0x19, 0x00,
	0x1a, 0x1e,
	0x03, 0x36,
	           
	0x1b, 0x00,
	0x16, 0x07,
	0x33, 0xe2,
	0x34, 0xbf,
	0x41, 0x00,
	0x96, 0x04,
	           
	0x3d, 0x19,
	0x69, 0x40,
	0x3a, 0x0d,
	0x8e, 0x00,
	           
	0x3c, 0x73,
	0x8f, 0xdF,
	0x8B, 0x06,
	0x8C, 0x20,
	0x94, 0x88,
	0x95, 0x88,
	0x40, 0xc1,
	0x29, 0x3f,
	0x0f, 0x42,
	0xA5, 0x80,
	0x1e, 0x04,
	0xa9, 0xb8,
	0xaa, 0x92,
	0xab, 0x0a,
	           
	0x90, 0x00,
	0x91, 0x00,
	0x9f, 0x00,
	0xa0, 0x00,
	           
	0x24, 0x68,
	0x25, 0x5c,
	0x26, 0xc3,
	0x14, 0x2e,
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gRAW_QVGA_INITIALIZE[] =
{
	/*30FPS*/
	0x12, 0x80,
	0x11, 0x87,
	0x6b, 0x4a,
	0x3b, 0x01,
	0x6a, 0x1f,
	0x13, 0xE2,
	0x10, 0x00,
	0x00, 0x00,
	0x01, 0x80,
	0x02, 0x80,
	0x13, 0xE7,
	           
	0x39, 0x43,
	0x38, 0x12,
	0x37, 0x00,
	0x35, 0x91,
	0x0e, 0xa0,
	           
	0xA8, 0x80,
	0x12, 0x15,
	0x04, 0x00,
	0x0c, 0x04,
	0x0d, 0x80,
	0x18, 0xc4,
	0x17, 0x24,
	0x32, 0xa4,
	0x19, 0x00,
	0x1a, 0x1e,
	0x03, 0x36,
	           
	0x1b, 0x00,
	0x16, 0x07,
	0x33, 0xe2,
	0x34, 0xbf,
	0x41, 0x00,
	0x96, 0x04,
	           
	0x3d, 0x19,
	0x69, 0x40,
	0x3a, 0x0d,
	0x8e, 0x00,
	           
	0x3c, 0x73,
	0x8f, 0xdF,
	0x8B, 0x06,
	0x8C, 0x20,
	0x94, 0x88,
	0x95, 0x88,
	0x40, 0xc1,
	0x29, 0x3f,
	0x0f, 0x42,
	0xA5, 0x80,
	0x1e, 0x04,
	0xa9, 0xb8,
	0xaa, 0x92,
	0xab, 0x0a,
	           
	0x90, 0x00,
	0x91, 0x00,
	0x9f, 0x00,
	0xa0, 0x00,
	           
	0x24, 0x68,
	0x25, 0x5c,
	0x26, 0xc3,
	0x14, 0x2e,
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gRAW_VGA_INITIALIZE[] =
{
	/*30FPS*/
	0x12, 0x80,                            
	0x11, 0x83,                            
	0x6b, 0x4a,                            
	0x3b, 0x01,                            
	0x6a, 0x3e,                            
	0x13, 0xE2,                            
	0x10, 0x00,                            
	0x00, 0x00,                            
	0x01, 0x80,                            
	0x02, 0x80,                            
	0x13, 0xE7,                            
	                                       
	0x39, 0x43,                            
	0x38, 0x12,                            
	0x37, 0x00,                            
	0x35, 0x91,                            
	0x0e, 0xa0,                            
	                                       
	0xA8, 0x80,                            
	0x12, 0x45,                            
	0x04, 0x00,                            
	0x0c, 0x04,                            
	0x0d, 0x80,                            
	0x18, 0xc4,                            
	0x17, 0x24,                            
	0x32, 0xa4,                            
	0x19, 0x00,                            
	0x1a, 0x3c,                            
	0x03, 0x36,                            
	                                       
	0x1b, 0x00,                            
	0x16, 0x07,                            
	0x33, 0xe2, /*c0 for internal regulator */
	0x34, 0xbf,                            
	0x41, 0x00,                            
	0x96, 0x04,                            
	                                       
	0x3d, 0x19,                            
	0x69, 0x40,                            
	0x3a, 0x0d,                            
	0x8e, 0x00,                            
	                                       
	0x3c, 0x73,                            
	0x8f, 0xdF,                            
	0x8B, 0x06,                            
	0x8C, 0x20,                            
	0x94, 0x88,                            
	0x95, 0x88,                            
	0x40, 0xc1,                            
	0x29, 0x3f, /*2f for internal regulator */
	0x0f, 0x42,                            
	0xA5, 0x80,                            
	0x1e, 0x04,                            
	0xa9, 0xb8,                            
	0xaa, 0x92,                            
	0xab, 0x0a,                            
	                                       
	0x90, 0x00,                            
	0x91, 0x00,                            
	0x9f, 0x00,                            
	0xa0, 0x00,                            
	                                       
	0x24, 0x68,                            
	0x25, 0x5c,                            
	0x26, 0xc3,                            
	0x14, 0x2e,                            
	                                       
	0x2a, 0x00, /*10 for 50Hz*/
	0x2b, 0x00, /*40 for 50Hz*/               
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gRAW_SXGA_INITIALIZE[] =
{
	/*15FPS*/
	0x12, 0x80,                                
	0x11, 0x83,                                
	0x6b, 0x4a,                                
	0x3b, 0x01,                                
	0x6a, 0x41,                                
	0x13, 0xE2,                                
	0x10, 0x00,                                
	0x00, 0x00,                                
	0x01, 0x80,                                
	0x02, 0x80,                                
	0x13, 0xE7,                                
	                                           
	0x39, 0x43,                                
	0x38, 0x12,                                
	0x37, 0x00,                                
	0x35, 0x91,                                
	0x0e, 0xa0,                                
	                                           
	0xA8, 0x80,                                
	0x12, 0x05,                                
	0x04, 0x00,                                
	0x0c, 0x00,                                
	0x0d, 0x00,                                
	0x18, 0xbb,                                
	0x17, 0x1b,                                
	0x32, 0xa4,                                
	0x19, 0x01,                                
	0x1a, 0x81,                                
	0x03, 0x12,                                
	                                           
	0x1b, 0x00,                                
	0x16, 0x07,                                
	0x33, 0xe2,	/*c0 for internal regulator*/
	0x34, 0xbf,                                
	0x41, 0x00,                                
	0x96, 0x04,                                
	                                           
	0x3d, 0x19,                                
	0x69, 0x40,                                
	0x3a, 0x0d,                                
	0x8e, 0x00,                                
	                                           
	0x3c, 0x73,                                
	0x8f, 0xDF,                                
	0x8B, 0x06,                                
	0x8C, 0x20,                                
	0x94, 0x88,                                
	0x95, 0x88,                                
	0x40, 0xc1,                                
	0x29, 0x3f, /*2f for internal regulator*/
	0x0f, 0x42,                                
	0xA5, 0x80,                                
	0x1e, 0x04,                                
	0xa9, 0xb8,                                
	0xaa, 0x92,                                
	0xab, 0x0a,                                
	                                           
	0x24, 0x68,                                
	0x25, 0x5c,                                
	0x26, 0xc3,                                
	0x14, 0x2e,                                
	                                           
	0x2a, 0x00, /*10 for 50Hz*/
	0x2b, 0x00, /*34 for 50Hz*/
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gRGB565_CIF_INITIALIZE[] =
{
	0x12, 0x80,                                                
	0x11, 0x87,                                                
	0x6b, 0x4a,                                                
	0x6a, 0x30,                                                
	0x3b, 0x09,                                                
	0x13, 0xe0,                                                
	0x01, 0x80,                                                
	0x02, 0x80,                                                
	0x00, 0x00,                                                
	0x10, 0x00,                                                
	0x13, 0xe5,                                                
	                                                           
	0x39, 0x43,                                                
	0x38, 0x12,                                                
	0x37, 0x00,                                                
	0x35, 0x91,                                                
	0x0e, 0xa0,                                                
	0x1e, 0x04,                                                
	                                                           
	0xA8, 0x80,                                                
	0x12, 0x24,                                                
	0x04, 0x00,                                                
	0x0c, 0x04,                                                
	0x0d, 0x80,                                                
	0x18, 0x7e,                                                
	0x17, 0x26,                                                
	0x32, 0x24,                                                
	0x03, 0x36,                                                
	0x1a, 0x24,                                                
	0x19, 0x00,                                                
	0x14, 0x2e,                                                
	0x15, 0x02,                                                
	0x3f, 0xa6,                                                
	0x41, 0x00,                                                
	0x42, 0x08,                                                
	                                                           
	0x1b, 0x00,                                                
	0x16, 0x06,                                                
	0x33, 0xe2, /*c0 for internal regulator*/
	0x34, 0xbf,                                                
	0x96, 0x04,                                                
	0x3a, 0x00,                                                
	0x8e, 0x00,                                                
	                                                           
	0x3c, 0x77,                                                
	0x8B, 0x06,                                                
	0x94, 0x88,                                                
	0x95, 0x88,                                                
	0x40, 0xd1,                                                
	0x29, 0x3f, /*2f for internal regulator*/
	0x0f, 0x42,                                                
	                                                           
	0x3d, 0x90,                                                
	0x69, 0x40,                                                
	0x5C, 0xb9,                                                
	0x5D, 0x96,                                                
	0x5E, 0x10,                                                
	0x59, 0xc0,                                                
	0x5A, 0xaf,                                                
	0x5B, 0x55,                                                
	0x43, 0xf0,                                                
	0x44, 0x10,                                                
	0x45, 0x68,                                                
	0x46, 0x96,                                                
	0x47, 0x60,                                                
	0x48, 0x80,                                                
	0x5F, 0xe0,                                                
	0x60, 0x8c, /*0c for advanced AWB (related to lens)*/
	0x61, 0x20,                                                
	0xa5, 0xd9,                                                
	0xa4, 0x74,                                                
	0x8d, 0x02,                                                
	0x13, 0xe7,                                                
	                                                           
	0x4f, 0xb7,                                                
	0x50, 0x2e,                                                
	0x51, 0x09,                                                
	0x52, 0x1f,                                                
	0x53, 0xb1,                                                
	0x54, 0x12,                                                
	0x55, 0x06,                                                
	0x56, 0x55,                                                
	0x57, 0xdb,                                                
	0x58, 0x77,                                                
	                                                           
	0x8C, 0x23,                                                
	0x3E, 0x02,                                                
	0xa9, 0xb8,                                                
	0xaa, 0x92,                                                
	0xab, 0x0a,                                                
	                                                           
	0x8f, 0xdf,                                                
	0x90, 0x00,                                                
	0x91, 0x00,                                                
	0x9f, 0x00,                                                
	0xa0, 0x00,                                                
	0x3A, 0x01,                                                
	                                                           
	0x24, 0x70,                                                
	0x25, 0x64,                                                
	0x26, 0xc3,                                                
	                                                           
	0x2a, 0x00, /*00 for 50Hz*/
	0x2b, 0x00, /*D2 for 50Hz*/
	                                                           
	/*gamma*/
	0x6c, 0x40,                                                
	0x6d, 0x30,                                                
	0x6e, 0x4b,                                                
	0x6f, 0x60,                                                
	0x70, 0x70,                                                
	0x71, 0x70,                                                
	0x72, 0x70,                                                
	0x73, 0x70,                                                
	0x74, 0x60,                                                
	0x75, 0x60,                                                
	0x76, 0x50,                                                
	0x77, 0x48,                                                
	0x78, 0x3a,                                                
	0x79, 0x2e,                                                
	0x7a, 0x28,                                                
	0x7b, 0x22,                                                
	0x7c, 0x04,                                                
	0x7d, 0x07,                                                
	0x7e, 0x10,                                                
	0x7f, 0x28,                                                
	0x80, 0x36,                                                
	0x81, 0x44,                                                
	0x82, 0x52,                                                
	0x83, 0x60,                                                
	0x84, 0x6c,                                                
	0x85, 0x78,                                                
	0x86, 0x8c,                                                
	0x87, 0x9e,                                                
	0x88, 0xbb,                                                
	0x89, 0xd2,                                                
	0x8a, 0xe6,                                                
	                                                           
	0x09, 0x11,                                                
	0x09, 0x01,                                                
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gRGB565_QCIF_INITIALIZE[] =
{
	0x12, 0x80,                                                     
	0x11, 0x8f,                                                     
	0x6b, 0x4a,                                                     
	0x6a, 0x18,                                                     
	0x3b, 0x09,                                                     
	0x13, 0xe0,                                                     
	0x01, 0x80,                                                     
	0x02, 0x80,                                                     
	0x00, 0x00,                                                     
	0x10, 0x00,                                                     
	0x13, 0xe5,                                                     
	                                                                
	0x39, 0x43,                                                     
	0x38, 0x12,                                                     
	0x37, 0x00,                                                     
	0x35, 0x91,                                                     
	0x0e, 0xa0,                                                     
	0x1e, 0x04,                                                     
	                                                                
	0xa8, 0x80,                                                     
	0x12, 0x0c,                                                     
	0x04, 0x00,                                                     
	0x0c, 0x04,                                                     
	0x0d, 0x80,                                                     
	0x18, 0x7e,                                                     
	0x17, 0x26,                                                     
	0x32, 0x24,                                                     
	0x03, 0x36,                                                     
	0x1a, 0x12,                                                     
	0x19, 0x00,                                                     
	0x14, 0x2e,                                                     
	0x15, 0x02,                                                     
	0x3f, 0xa6,                                                     
	0x41, 0x00,                                                     
	0x42, 0x08,                                                     
	                                                                
	0x1b, 0x00,                                                     
	0x16, 0x06,                                                     
	0x33, 0xe2, /*c0 for internal regulator*/                       
	0x34, 0xbf,                                                     
	0x96, 0x04,                                                     
	0x3a, 0x00,                                                     
	0x8e, 0x00,                                                     
	                                                                
	0x3c, 0x77,                                                     
	0x8B, 0x06,                                                     
	0x94, 0x88,                                                     
	0x95, 0x88,                                                     
	0x40, 0xd1,                                                     
	0x29, 0x3f, /*2f for internal regulator*/                       
	0x0f, 0x42,                                                     
	                                                                
	0x3d, 0x92,                                                     
	0x69, 0x40,                                                     
	0x5C, 0xb9,                                                     
	0x5D, 0x96,                                                     
	0x5E, 0x10,                                                     
	0x59, 0xc0,                                                     
	0x5A, 0xaf,                                                     
	0x5B, 0x55,                                                     
	0x43, 0xf0,                                                     
	0x44, 0x10,                                                     
	0x45, 0x68,                                                     
	0x46, 0x96,                                                     
	0x47, 0x60,                                                     
	0x48, 0x80,                                                     
	0x5F, 0xe0,                                                     
	0x60, 0x8c, /*0c for advanced AWB (related to lens)*/           
	0x61, 0x20,                                                     
	0xa5, 0xd9,                                                     
	0xa4, 0x74,                                                     
	0x8d, 0x02,                                                     
	0x13, 0xe7,                                                     
	                                                                
	0x4f, 0xb7,                                                     
	0x50, 0x2e,                                                     
	0x51, 0x09,                                                     
	0x52, 0x1f,                                                     
	0x53, 0xb1,                                                     
	0x54, 0x12,                                                     
	0x55, 0x06,                                                     
	0x56, 0x55,                                                     
	0x57, 0xdb,                                                     
	0x58, 0x77,                                                     
	                                                                
	0x8C, 0x23,                                                     
	0x3E, 0x02,                                                     
	0xa9, 0xb8,                                                     
	0xaa, 0x92,                                                     
	0xab, 0x0a,                                                     
	                                                                
	0x8f, 0xdf,                                                     
	0x90, 0x00,                                                     
	0x91, 0x00,                                                     
	0x9f, 0x00,                                                     
	0xa0, 0x00,                                                     
	0x3A, 0x01,                                                     
	                                                                
	0x24, 0x70,                                                     
	0x25, 0x64,                                                     
	0x26, 0xc3,                                                     
	                                                                
	0x2a, 0x00, /*00 for 50Hz*/                                     
	0x2b, 0x00, /*D2 for 50Hz*/                                     
	                                                                
	/*gamma*/                                                       
	0x6c, 0x40,                                                     
	0x6d, 0x30,                                                     
	0x6e, 0x4b,                                                     
	0x6f, 0x60,                                                     
	0x70, 0x70,                                                     
	0x71, 0x70,                                                     
	0x72, 0x70,                                                     
	0x73, 0x70,                                                     
	0x74, 0x60,                                                     
	0x75, 0x60,                                                     
	0x76, 0x50,                                                     
	0x77, 0x48,                                                     
	0x78, 0x3a,                                                     
	0x79, 0x2e,                                                     
	0x7a, 0x28,                                                     
	0x7b, 0x22,                                                     
	0x7c, 0x04,                                                     
	0x7d, 0x07,                                                     
	0x7e, 0x10,                                                     
	0x7f, 0x28,                                                     
	0x80, 0x36,                                                     
	0x81, 0x44,                                                     
	0x82, 0x52,                                                     
	0x83, 0x60,                                                     
	0x84, 0x6c,                                                     
	0x85, 0x78,                                                     
	0x86, 0x8c,                                                     
	0x87, 0x9e,                                                     
	0x88, 0xbb,                                                     
	0x89, 0xd2,                                                     
	0x8a, 0xe6,                                                     
	                                                                
	0x09, 0x11,                                                     
	0x09, 0x01,                                                     
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gRGB565_QQCIF_INITIALIZE[] =
{
	0x12, 0x80,                                               
	0x11, 0x8f,                                               
	0x6b, 0x4a,                                               
	0x6a, 0x18,                                               
	0x3b, 0x09,                                               
	0x13, 0xe0,                                               
	0x01, 0x80,                                               
	0x02, 0x80,                                               
	0x00, 0x00,                                               
	0x10, 0x00,                                               
	0x13, 0xe5,                                               
	                                                          
	0x39, 0x43,                                               
	0x38, 0x12,                                               
	0x37, 0x00,                                               
	0x35, 0x91,                                               
	0x0e, 0xa0,                                               
	0x1e, 0x04,                                               
	                                                          
	0xA8, 0x80,                                               
	0x12, 0x0c,                                               
	0x04, 0x24,                                               
	0x0c, 0x04,                                               
	0x0d, 0x80,                                               
	0x18, 0x7e,                                               
	0x17, 0x26,                                               
	0x32, 0xa4,                                               
	0x03, 0x36,                                               
	0x1a, 0x12,                                               
	0x19, 0x00,                                               
	0x14, 0x2e,                                               
	0x15, 0x02,                                               
	0x3f, 0xa6,                                               
	0x41, 0x00,                                               
	0x42, 0x08,                                               
	                                                          
	0x1b, 0x00,                                               
	0x16, 0x06,                                               
	0x33, 0xe2, /*c0 for internal regulator*/                 
	0x34, 0xbf,                                               
	0x96, 0x04,                                               
	0x3a, 0x00,                                               
	0x8e, 0x00,                                               
	                                                          
	0x3c, 0x77,                                               
	0x8B, 0x06,                                               
	0x94, 0x88,                                               
	0x95, 0x88,                                               
	0x40, 0xd1,                                               
	0x29, 0x3f, /*2f for internal regulator*/                 
	0x0f, 0x42,                                               
	                                                          
	0x3d, 0x92,                                               
	0x69, 0x40,                                               
	0x5C, 0xb9,                                               
	0x5D, 0x96,                                               
	0x5E, 0x10,                                               
	0x59, 0xc0,                                               
	0x5A, 0xaf,                                               
	0x5B, 0x55,                                               
	0x43, 0xf0,                                               
	0x44, 0x10,                                               
	0x45, 0x68,                                               
	0x46, 0x96,                                               
	0x47, 0x60,                                               
	0x48, 0x80,                                               
	0x5F, 0xe0,                                               
	0x60, 0x8c, /*0c for advanced AWB (related to lens)*/     
	0x61, 0x20,                                               
	0xa5, 0xd9,                                               
	0xa4, 0x74,                                               
	0x8d, 0x02,                                               
	0x13, 0xe7,                                               
	                                                          
	0x4f, 0xb7,                                               
	0x50, 0x2e,                                               
	0x51, 0x09,                                               
	0x52, 0x1f,                                               
	0x53, 0xb1,                                               
	0x54, 0x12,                                               
	0x55, 0x06,                                               
	0x56, 0x55,                                               
	0x57, 0xdb,                                               
	0x58, 0x77,                                               
	                                                          
	0x8C, 0x23,                                               
	0x3E, 0x02,                                               
	0xa9, 0xb8,                                               
	0xaa, 0x92,                                               
	0xab, 0x0a,                                               
	                                                          
	0x8f, 0xdf,                                               
	0x90, 0x00,                                               
	0x91, 0x00,                                               
	0x9f, 0x00,                                               
	0xa0, 0x00,                                               
	0x3A, 0x01,                                               
	                                                          
	0x24, 0x70,                                               
	0x25, 0x64,                                               
	0x26, 0xc3,                                               
	0x2a, 0x00, /*00 for 50Hz*/                               
	0x2b, 0x00, /*D2 for 50Hz*/                               
	                                                          
	/*gamma*/                                                 
	0x6c, 0x40,                                               
	0x6d, 0x30,                                               
	0x6e, 0x4b,                                               
	0x6f, 0x60,                                               
	0x70, 0x70,                                               
	0x71, 0x70,                                               
	0x72, 0x70,                                               
	0x73, 0x70,                                               
	0x74, 0x60,                                               
	0x75, 0x60,                                               
	0x76, 0x50,                                               
	0x77, 0x48,                                               
	0x78, 0x3a,                                               
	0x79, 0x2e,                                               
	0x7a, 0x28,                                               
	0x7b, 0x22,                                               
	0x7c, 0x04,                                               
	0x7d, 0x07,                                               
	0x7e, 0x10,                                               
	0x7f, 0x28,                                               
	0x80, 0x36,                                               
	0x81, 0x44,                                               
	0x82, 0x52,                                               
	0x83, 0x60,                                               
	0x84, 0x6c,                                               
	0x85, 0x78,                                               
	0x86, 0x8c,                                               
	0x87, 0x9e,                                               
	0x88, 0xbb,                                               
	0x89, 0xd2,                                               
	0x8a, 0xe6,                                               
	                                                          
	0x09, 0x11,                                               
	0x09, 0x01,                                               
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gRGB565_QQVGA_INITIALIZE[] =
{
	0x12, 0x80,                                                     
	0x11, 0x87,                                                     
	0x6b, 0x4a,                                                     
	0x6a, 0x1f,                                                     
	0x3b, 0x09,                                                     
	0x13, 0xe0,                                                     
	0x01, 0x80,                                                     
	0x02, 0x80,                                                     
	0x00, 0x00,                                                     
	0x10, 0x00,                                                     
	0x13, 0xe5,                                                     
	                                                                
	0x39, 0x43,                                                     
	0x38, 0x12,                                                     
	0x37, 0x00,                                                     
	0x35, 0x91,                                                     
	0x0e, 0xa0,                                                     
	0x1e, 0x04,                                                     
	                                                                
	0xA8, 0x80,                                                     
	0x12, 0x14,                                                     
	0x04, 0x24,                                                     
	0x0c, 0x04,                                                     
	0x0d, 0x80,                                                     
	0x18, 0xc6,                                                     
	0x17, 0x26,                                                     
	0x32, 0xa4,                                                     
	0x03, 0x36,                                                     
	0x1a, 0x1e,                                                     
	0x19, 0x00,                                                     
	0x14, 0x2e,                                                     
	0x15, 0x02,                                                     
	0x3f, 0xa6,                                                     
	0x41, 0x00,                                                     
	0x42, 0x08,                                                     
	                                                                
	0x1b, 0x00,                                                     
	0x16, 0x06,                                                     
	0x33, 0xe2, /*c0 for internal regulator*/                       
	0x34, 0xbf,                                                     
	0x96, 0x04,                                                     
	0x3a, 0x00,                                                     
	0x8e, 0x00,                                                     
	                                                                
	0x3c, 0x77,                                                     
	0x8B, 0x06,                                                     
	0x94, 0x88,                                                     
	0x95, 0x88,                                                     
	0x40, 0xd1,                                                     
	0x29, 0x3f, /*2f for internal regulator*/                       
	0x0f, 0x42,                                                     
	                                                                
	0x3d, 0x90,                                                     
	0x69, 0x40,                                                     
	0x5C, 0xb9,                                                     
	0x5D, 0x96,                                                     
	0x5E, 0x10,                                                     
	0x59, 0xc0,                                                     
	0x5A, 0xaf,                                                     
	0x5B, 0x55,                                                     
	0x43, 0xf0,                                                     
	0x44, 0x10,                                                     
	0x45, 0x68,                                                     
	0x46, 0x96,                                                     
	0x47, 0x60,                                                     
	0x48, 0x80,                                                     
	0x5F, 0xe0,                                                     
	0x60, 0x8C, /*0c for advanced AWB (Related to lens)*/           
	0x61, 0x20,                                                     
	0xa5, 0xd9,                                                     
	0xa4, 0x74,                                                     
	0x8d, 0x02,                                                     
	0x13, 0xe7,                                                     
	                                                                
	0x4f, 0xb7,                                                     
	0x50, 0x2e,                                                     
	0x51, 0x09,                                                     
	0x52, 0x1f,                                                     
	0x53, 0xb1,                                                     
	0x54, 0x12,                                                     
	0x55, 0x06,                                                     
	0x56, 0x55,                                                     
	0x57, 0xdb,                                                     
	0x58, 0x77,                                                     
	                                                                
	0x8C, 0x23,                                                     
	0x3E, 0x02,                                                     
	0xa9, 0xb8,                                                     
	0xaa, 0x92,                                                     
	0xab, 0x0a,                                                     
	                                                                
	0x8f, 0xdf,                                                     
	0x90, 0x00,                                                     
	0x91, 0x00,                                                     
	0x9f, 0x00,                                                     
	0xa0, 0x00,                                                     
	0x3A, 0x01,                                                     
	                                                                
	0x24, 0x70,                                                     
	0x25, 0x64,                                                     
	0x26, 0xc3,                                                     
	                                                                
	0x2a, 0x00, /*10 for 50Hz*/                                     
	0x2b, 0x00, /*40 for 50Hz*/                                     
	                                                                
	/*gamma*/                                                       
	0x6c, 0x40,                                                     
	0x6d, 0x30,                                                     
	0x6e, 0x4b,                                                     
	0x6f, 0x60,                                                     
	0x70, 0x70,                                                     
	0x71, 0x70,                                                     
	0x72, 0x70,                                                     
	0x73, 0x70,                                                     
	0x74, 0x60,                                                     
	0x75, 0x60,                                                     
	0x76, 0x50,                                                     
	0x77, 0x48,                                                     
	0x78, 0x3a,                                                     
	0x79, 0x2e,                                                     
	0x7a, 0x28,                                                     
	0x7b, 0x22,                                                     
	0x7c, 0x04,                                                     
	0x7d, 0x07,                                                     
	0x7e, 0x10,                                                     
	0x7f, 0x28,                                                     
	0x80, 0x36,                                                     
	0x81, 0x44,                                                     
	0x82, 0x52,                                                     
	0x83, 0x60,                                                     
	0x84, 0x6c,                                                     
	0x85, 0x78,                                                     
	0x86, 0x8c,                                                     
	0x87, 0x9e,                                                     
	0x88, 0xbb,                                                     
	0x89, 0xd2,                                                     
	0x8a, 0xe6,                                                     
	                                                                
	0x09, 0x11,                                                     
	0x09, 0x01,                                                     
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gRGB565_QVGA_INITIALIZE[] =
{
	0x12, 0x80,                                                       
	0x11, 0x87,                                                       
	0x6b, 0x4a,                                                       
	0x6a, 0x1f,                                                       
	0x3b, 0x09,                                                       
	0x13, 0xe0,                                                       
	0x01, 0x80,                                                       
	0x02, 0x80,                                                       
	0x00, 0x00,                                                       
	0x10, 0x00,                                                       
	0x13, 0xe5,                                                       
	                                                                  
	0x39, 0x43,                                                       
	0x38, 0x12,                                                       
	0x37, 0x00,                                                       
	0x35, 0x91,                                                       
	0x0e, 0xa0,                                                       
	0x1e, 0x04,                                                       
	                                                                  
	0xA8, 0x80,                                                       
	0x12, 0x14,                                                       
	0x04, 0x00,                                                       
	0x0c, 0x04,                                                       
	0x0d, 0x80,                                                       
	0x18, 0xc6,                                                       
	0x17, 0x26,                                                       
	0x32, 0x24,                                                       
	0x03, 0x36,                                                       
	0x1a, 0x1e,                                                       
	0x19, 0x00,                                                       
	0x3f, 0xa6,                                                       
	0x14, 0x2e,                                                       
	0x15, 0x02,                                                       
	0x41, 0x00,                                                       
	0x42, 0x08,                                                       
	                                                                  
	0x1b, 0x00,                                                       
	0x16, 0x06,                                                       
	0x33, 0xe2, /*c0 for internal regulator*/                         
	0x34, 0xbf,                                                       
	0x96, 0x04,                                                       
	0x3a, 0x00,                                                       
	0x8e, 0x00,                                                       
	                                                                  
	0x3c, 0x77,                                                       
	0x8B, 0x06,                                                       
	0x94, 0x88,                                                       
	0x95, 0x88,                                                       
	0x40, 0xd1,                                                       
	0x29, 0x3f, /*2f for internal regulator*/                         
	0x0f, 0x42,                                                       
	                                                                  
	0x3d, 0x90,                                                       
	0x69, 0x40,                                                       
	0x5C, 0x96,                                                       
	0x5D, 0x96,                                                       
	0x5E, 0x10,                                                       
	0x59, 0xeb,                                                       
	0x5A, 0x9c,                                                       
	0x5B, 0x55,                                                       
	0x43, 0xf0,                                                       
	0x44, 0x10,                                                       
	0x45, 0x55,                                                       
	0x46, 0x86,                                                       
	0x47, 0x64,                                                       
	0x48, 0x86,                                                       
	0x5F, 0xe0,                                                       
	0x60, 0x8c, /*0C for advanced AWB (relate to lens)*/              
	0x61, 0x20,                                                       
	0xa5, 0xd9,                                                       
	0xa4, 0x74,                                                       
	0x8d, 0x02,                                                       
	0x13, 0xe7,                                                       
	                                                                  
	0x4f, 0xb7,                                                       
	0x50, 0x2e,                                                       
	0x51, 0x09,                                                       
	0x52, 0x1f,                                                       
	0x53, 0xb1,                                                       
	0x54, 0x12,                                                       
	0x55, 0x06,                                                       
	0x56, 0x55,                                                       
	0x57, 0xdb,                                                       
	0x58, 0x77,                                                       
	                                                                  
	0x8C, 0x23,                                                       
	0x3E, 0x02,                                                       
	0xa9, 0xb8,                                                       
	0xaa, 0x92,                                                       
	0xab, 0x0a,                                                       
	                                                                  
	0x8f, 0xdf,                                                       
	0x90, 0x00,                                                       
	0x91, 0x00,                                                       
	0x9f, 0x00,                                                       
	0xa0, 0x00,                                                       
	0x3A, 0x01,                                                       
	                                                                  
	0x24, 0x70,                                                       
	0x25, 0x64,                                                       
	0x26, 0xc3,                                                       
	                                                                  
	0x2a, 0x00, /*10 for 50Hz*/                                       
	0x2b, 0x00, /*40 for 50Hz*/                                       
	                                                                  
	/*gamma*/                                                         
	0x6c, 0x40,                                                       
	0x6d, 0x30,                                                       
	0x6e, 0x4b,                                                       
	0x6f, 0x60,                                                       
	0x70, 0x70,                                                       
	0x71, 0x70,                                                       
	0x72, 0x70,                                                       
	0x73, 0x70,                                                       
	0x74, 0x60,                                                       
	0x75, 0x60,                                                       
	0x76, 0x50,                                                       
	0x77, 0x48,                                                       
	0x78, 0x3a,                                                       
	0x79, 0x2e,                                                       
	0x7a, 0x28,                                                       
	0x7b, 0x22,                                                       
	0x7c, 0x04,                                                       
	0x7d, 0x07,                                                       
	0x7e, 0x10,                                                       
	0x7f, 0x28,                                                       
	0x80, 0x36,                                                       
	0x81, 0x44,                                                       
	0x82, 0x52,                                                       
	0x83, 0x60,                                                       
	0x84, 0x6c,                                                       
	0x85, 0x78,                                                       
	0x86, 0x8c,                                                       
	0x87, 0x9e,                                                       
	0x88, 0xbb,                                                       
	0x89, 0xd2,                                                       
	0x8a, 0xe6,                                                       
	                                                                  
	0x09, 0x11,                                                       
	0x09, 0x01,                                                       
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gRGB565_VGA_INITIALIZE[] =
{
	0x12, 0x80,                                                         
	0x11, 0x83,                                                         
	0x6b, 0x4a,                                                         
	0x6a, 0x3e,                                                         
	0x3b, 0x09,                                                         
	0x13, 0xe0,                                                         
	0x01, 0x80,                                                         
	0x02, 0x80,                                                         
	0x00, 0x00,                                                         
	0x10, 0x00,                                                         
	0x13, 0xe5,                                                         

#ifdef MCLK_13M	                                                                    
	0x39, 0x43, /*43 for 15fps*/                                        
	0x38, 0x12, /*12 for 15fps*/                                        
	0x37, 0x00,                                                         
	0x35, 0x91, /*91 for 15fps*/
#else
	0x39, 0x50, /*50 for 30fps*/                                        
	0x38, 0x92, /*92 for 30fps*/                                        
	0x37, 0x00,                                                         
	0x35, 0x81, /*81 for 30fps*/
#endif
	0x0e, 0xa0,                                                         
	0x1e, 0x04,                                                         
	                                                                    
	0xA8, 0x80,                                                         
	0x12, 0x44,                                                         
	0x04, 0x00,                                                         
	0x0c, 0x04,                                                         
	0x0d, 0x80,                                                         
	0x18, 0xc6,                                                         
	0x17, 0x26,                                                         
	0x32, 0xad,                                                         
	0x03, 0x00,                                                         
	0x1a, 0x3d,                                                         
	0x19, 0x01,                                                         
	0x3f, 0xa6,                                                         
	0x14, 0x2e,                                                         
	0x15, 0x02,                                                         
	0x41, 0x00,                                                         
	0x42, 0x08,                                                         
	                                                                    
	0x1b, 0x00,                                                         
	0x16, 0x06,                                                         
	0x33, 0xe2, /*c0 for internal regulator*/                           
	0x34, 0xbf,                                                         
	0x96, 0x04,                                                         
	0x3a, 0x00,                                                         
	0x8e, 0x00,                                                         
	                                                                    
	0x3c, 0x77,                                                         
	0x8B, 0x06,                                                         
	0x94, 0x88,                                                         
	0x95, 0x88,                                                         
	0x40, 0xd1,                                                         
	0x29, 0x3f, /*2f for internal regulator*/                           
	0x0f, 0x42,                                                         
	                                                                    
	0x3d, 0x90,                                                         
	0x69, 0x40,                                                         
	0x5C, 0xb9,                                                         
	0x5D, 0x96,                                                         
	0x5E, 0x10,                                                         
	0x59, 0xc0,                                                         
	0x5A, 0xaf,                                                         
	0x5B, 0x55,                                                         
	0x43, 0xf0,                                                         
	0x44, 0x10,                                                         
	0x45, 0x68,                                                         
	0x46, 0x96,                                                         
	0x47, 0x60,                                                         
	0x48, 0x80,                                                         
	0x5F, 0xe0,                                                         
	0x60, 0x8c, /*0c for advanced AWB (related to lens)*/               
	0x61, 0x20,                                                         
	0xa5, 0xd9,                                                         
	0xa4, 0x74,                                                         
	0x8d, 0x02,                                                         
	0x13, 0xe7,                                                         
	                                                                    
	0x4f, 0xb7,                                                         
	0x50, 0x2e,                                                         
	0x51, 0x09,                                                         
	0x52, 0x1f,                                                         
	0x53, 0xb1,                                                         
	0x54, 0x12,                                                         
	0x55, 0x06,                                                         
	0x56, 0x55,                                                         
	0x57, 0xdb,                                                         
	0x58, 0x77,                                                         
	                                                                    
	0x8C, 0x23,                                                         
	0x3E, 0x02,                                                         
	0xa9, 0xb8,                                                         
	0xaa, 0x92,                                                         
	0xab, 0x0a,                                                         
	                                                                    
	0x8f, 0xdf,                                                         
	0x90, 0x00,                                                         
	0x91, 0x00,                                                         
	0x9f, 0x00,                                                         
	0xa0, 0x00,                                                         
	0x3A, 0x01,                                                         
	                                                                    
	0x24, 0x70,                                                         
	0x25, 0x64,                                                         
	0x26, 0xc3,                                                         
	                                                                    
	0x2a, 0x00, /*10 for 50Hz*/                                         
	0x2b, 0x00, /*40 for 50Hz*/                                         
	                                                                    
	/*gamma*/                                                           
	0x6c, 0x40,                                                         
	0x6d, 0x30,                                                         
	0x6e, 0x4b,                                                         
	0x6f, 0x60,                                                         
	0x70, 0x70,                                                         
	0x71, 0x70,                                                         
	0x72, 0x70,                                                         
	0x73, 0x70,                                                         
	0x74, 0x60,                                                         
	0x75, 0x60,                                                         
	0x76, 0x50,                                                         
	0x77, 0x48,                                                         
	0x78, 0x3a,                                                         
	0x79, 0x2e,                                                         
	0x7a, 0x28,                                                         
	0x7b, 0x22,                                                         
	0x7c, 0x04,                                                         
	0x7d, 0x07,                                                         
	0x7e, 0x10,                                                         
	0x7f, 0x28,                                                         
	0x80, 0x36,                                                         
	0x81, 0x44,                                                         
	0x82, 0x52,                                                         
	0x83, 0x60,                                                         
	0x84, 0x6c,                                                         
	0x85, 0x78,                                                         
	0x86, 0x8c,                                                         
	0x87, 0x9e,                                                         
	0x88, 0xbb,                                                         
	0x89, 0xd2,                                                         
	0x8a, 0xe6,                                                         
	                                                                    
	0x09, 0x11,                                                         
	0x09, 0x01,                                                         
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gRGB565_SXGA_INITIALIZE[] =
{
	/*15FPS*/
	0x12, 0x80,                                                        
	0x11, 0x81,                                                        
	0x6b, 0x4a,                                                        
	0x6a, 0x41,                                                        
	0x3b, 0x09,                                                        
	0x13, 0xe0,                                                        
	0x01, 0x80,                                                        
	0x02, 0x80,                                                        
	0x00, 0x00,                                                        
	0x10, 0x00,                                                        
	0x13, 0xe5,                                                        
	                                                                   
#ifdef MCLK_13M	                                                                    
	0x39, 0x43, /*43 for 7.5fps*/                                       
	0x38, 0x12, /*12 for 7.5fps*/                                       
	0x37, 0x00,                                                        
	0x35, 0x91, /*91 for 7.5fps*/                                       
#else
	0x39, 0x50, /*50 for 15fps*/                                        
	0x38, 0x93, /*93 for 15fps*/                                        
	0x37, 0x00,                                                         
	0x35, 0x81, /*81 for 15fps*/
#endif
	0x0e, 0xa0,                                                        
	0x1e, 0x04,                                                        
	                                                                   
	0xA8, 0x80,                                                        
	0x12, 0x04,                                                        
	0x04, 0x00,                                                        
	0x0c, 0x00,                                                        
	0x0d, 0x00,                                                        
	0x18, 0xbd,                                                        
	0x17, 0x1d,                                                        
	0x32, 0xad,                                                        
	0x03, 0x12,                                                        
	0x1a, 0x81,                                                        
	0x19, 0x01,                                                        
	0x14, 0x2e,                                                        
	0x15, 0x00,                                                        
	0x3f, 0xa6,                                                        
	0x41, 0x00,                                                        
	0x42, 0x08,                                                        
	                                                                   
	0x1b, 0x00,                                                        
	0x16, 0x06,                                                        
	0x33, 0xe2, /*c0 for internal regulator*/                          
	0x34, 0xbf,                                                        
	0x96, 0x04,                                                        
	0x3a, 0x00,                                                        
	0x8e, 0x00,                                                        
	                                                                   
	0x3c, 0x77,                                                        
	0x8B, 0x06,                                                        
	0x94, 0x88,                                                        
	0x95, 0x88,                                                        
	0x40, 0xd1,                                                        
	0x29, 0x3f, /*2f for internal regulator*/                          
	0x0f, 0x42,                                                        
	                                                                   
	0x3d, 0x90,                                                        
	0x69, 0x40,                                                        
	0x5C, 0xb9,                                                        
	0x5D, 0x96,                                                        
	0x5E, 0x10,                                                        
	0x59, 0xc0,                                                        
	0x5A, 0xaf,                                                        
	0x5B, 0x55,                                                        
	0x43, 0xf0,                                                        
	0x44, 0x10,                                                        
	0x45, 0x68,                                                        
	0x46, 0x96,                                                        
	0x47, 0x60,                                                        
	0x48, 0x80,                                                        
	0x5F, 0xe0,                                                        
	0x60, 0x8C, /*0c for advanced AWB (Related to lens)*/              
	0x61, 0x20,                                                        
	0xa5, 0xd9,                                                        
	0xa4, 0x74,                                                        
	0x8d, 0x02,                                                        
	0x13, 0xe7,                                                        
	                                                                   
	0x4f, 0xb7,                                                        
	0x50, 0x2e,                                                        
	0x51, 0x09,                                                        
	0x52, 0x1f,                                                        
	0x53, 0xb1,                                                        
	0x54, 0x12,                                                        
	0x55, 0x06,                                                        
	0x56, 0x55,                                                        
	0x57, 0xdb,                                                        
	0x58, 0x77,                                                        
	                                                                   
	0x8C, 0x23,                                                        
	0x3E, 0x02,                                                        
	0xa9, 0xb8,                                                        
	0xaa, 0x92,                                                        
	0xab, 0x0a,                                                        
	                                                                   
	0x8f, 0xdf,                                                        
	0x90, 0x00,                                                        
	0x91, 0x00,                                                        
	0x9f, 0x00,                                                        
	0xa0, 0x00,                                                        
	0x3A, 0x01,                                                        
	                                                                   
	0x24, 0x70,                                                        
	0x25, 0x64,                                                        
	0x26, 0xc3,                                                        
	                                                                   
	0x2a, 0x00, /*10 for 50Hz*/                                        
	0x2b, 0x00, /*34 for 50Hz*/                                        
	                                                                   
	/*gamma*/                                                          
	0x6c, 0x40,                                                        
	0x6d, 0x30,                                                        
	0x6e, 0x4b,                                                        
	0x6f, 0x60,                                                        
	0x70, 0x70,                                                        
	0x71, 0x70,                                                        
	0x72, 0x70,                                                        
	0x73, 0x70,                                                        
	0x74, 0x60,                                                        
	0x75, 0x60,                                                        
	0x76, 0x50,                                                        
	0x77, 0x48,                                                        
	0x78, 0x3a,                                                        
	0x79, 0x2e,                                                        
	0x7a, 0x28,                                                        
	0x7b, 0x22,                                                        
	0x7c, 0x04,                                                        
	0x7d, 0x07,                                                        
	0x7e, 0x10,                                                        
	0x7f, 0x28,                                                        
	0x80, 0x36,                                                        
	0x81, 0x44,                                                        
	0x82, 0x52,                                                        
	0x83, 0x60,                                                        
	0x84, 0x6c,                                                        
	0x85, 0x78,                                                        
	0x86, 0x8c,                                                        
	0x87, 0x9e,                                                        
	0x88, 0xbb,                                                        
	0x89, 0xd2,                                                        
	0x8a, 0xe6,                                                        
	                                                                   
	0x09, 0x11,                                                        
	0x09, 0x01,                                                        
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gYUV_CIF_INITIALIZE[] =
{
	0x12, 0x80,                                                            
	0x11, 0x87,                                                            
	0x6b, 0x4a,                                                            
	0x6a, 0x30,                                                            
	0x3b, 0x09,                                                            
	0x13, 0xe0,                                                            
	0x01, 0x80,                                                            
	0x02, 0x80,                                                            
	0x00, 0x00,                                                            
	0x10, 0x00,                                                            
	0x13, 0xe5,                                                            
	                                                                       
	0x39, 0x43,                                                            
	0x38, 0x12,                                                            
	0x37, 0x00,                                                            
	0x35, 0x91,                                                            
	0x0e, 0xa0,                                                            
	0x1e, 0x04,                                                            
	                                                                       
	0xA8, 0x80,                                                            
	0x12, 0x20,                                                            
	0x04, 0x00,                                                            
	0x0c, 0x04,                                                            
	0x0d, 0x80,                                                            
	0x18, 0x7e,                                                            
	0x17, 0x26,                                                            
	0x32, 0x24,                                                            
	0x03, 0x36,                                                            
	0x1a, 0x24,                                                            
	0x19, 0x00,                                                            
	0x14, 0x2e,                                                            
	0x15, 0x02,                                                            
	0x3f, 0xa6,                                                            
	0x41, 0x02,                                                            
	0x42, 0x08,                                                            
	                                                                       
	0x1b, 0x00,                                                            
	0x16, 0x06,                                                            
	0x33, 0xe2, /*c0 for internal regulator*/                              
	0x34, 0xbf,                                                            
	0x96, 0x04,                                                            
	0x3a, 0x00,                                                            
	0x8e, 0x00,                                                            
	                                                                       
	0x3c, 0x77,                                                            
	0x8B, 0x06,                                                            
	0x94, 0x88,                                                            
	0x95, 0x88,                                                            
	0x40, 0xc1,                                                            
	0x29, 0x3f, /*2f for internal regulator*/                              
	0x0f, 0x42,                                                            
	                                                                       
	0x3d, 0x92,                                                            
	0x69, 0x40,                                                            
	0x5C, 0xb9,                                                            
	0x5D, 0x96,                                                            
	0x5E, 0x10,                                                            
	0x59, 0xc0,                                                            
	0x5A, 0xaf,                                                            
	0x5B, 0x55,                                                            
	0x43, 0xf0,                                                            
	0x44, 0x10,                                                            
	0x45, 0x68,                                                            
	0x46, 0x96,                                                            
	0x47, 0x60,                                                            
	0x48, 0x80,                                                            
	0x5F, 0xe0,                                                            
	0x60, 0x8c, /*0c for advanced AWB (related to lens)*/                  
	0x61, 0x20,                                                            
	0xa5, 0xd9,                                                            
	0xa4, 0x74,                                                            
	0x8d, 0x02,                                                            
	0x13, 0xe7,                                                            
	                                                                       
	0x4f, 0x3a,                                                            
	0x50, 0x3d,                                                            
	0x51, 0x03,                                                            
	0x52, 0x12,                                                            
	0x53, 0x26,                                                            
	0x54, 0x38,                                                            
	0x55, 0x40,                                                            
	0x56, 0x40,                                                            
	0x57, 0x40,                                                            
	0x58, 0x0d,                                                            
	                                                                       
	0x8C, 0x23,                                                            
	0x3E, 0x02,                                                            
	0xa9, 0xb8,                                                            
	0xaa, 0x92,                                                            
	0xab, 0x0a,                                                            
	                                                                       
	0x8f, 0xdf,                                                            
	0x90, 0x00,                                                            
	0x91, 0x00,                                                            
	0x9f, 0x00,                                                            
	0xa0, 0x00,                                                            
	0x3a, 0x0d, /*01 YUYV*/
	                                                                       
	0x24, 0x70,                                                            
	0x25, 0x64,                                                            
	0x26, 0xc3,                                                            
	                                                                       
	0x2a, 0x00, /*00 for 50Hz*/                                            
	0x2b, 0x00, /*D2 for 50Hz*/                                            
	                                                                       
	/*gamma*/                                                              
	0x6c, 0x40,                                                            
	0x6d, 0x30,                                                            
	0x6e, 0x4b,                                                            
	0x6f, 0x60,                                                            
	0x70, 0x70,                                                            
	0x71, 0x70,                                                            
	0x72, 0x70,                                                            
	0x73, 0x70,                                                            
	0x74, 0x60,                                                            
	0x75, 0x60,                                                            
	0x76, 0x50,                                                            
	0x77, 0x48,                                                            
	0x78, 0x3a,                                                            
	0x79, 0x2e,                                                            
	0x7a, 0x28,                                                            
	0x7b, 0x22,                                                            
	0x7c, 0x04,                                                            
	0x7d, 0x07,                                                            
	0x7e, 0x10,                                                            
	0x7f, 0x28,                                                            
	0x80, 0x36,                                                            
	0x81, 0x44,                                                            
	0x82, 0x52,                                                            
	0x83, 0x60,                                                            
	0x84, 0x6c,                                                            
	0x85, 0x78,                                                            
	0x86, 0x8c,                                                            
	0x87, 0x9e,                                                            
	0x88, 0xbb,                                                            
	0x89, 0xd2,                                                            
	0x8a, 0xe6,                                                            
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gYUV_QCIF_INITIALIZE[] =
{
	0x12, 0x80,                                                       
	0x11, 0x8f,                                                       
	0x6b, 0x4a,                                                       
	0x6a, 0x18,                                                       
	0x3b, 0x09,                                                       
	0x13, 0xe0,                                                       
	0x01, 0x80,                                                       
	0x02, 0x80,                                                       
	0x00, 0x00,                                                       
	0x10, 0x00,                                                       
	0x13, 0xe5,                                                       
	                                                                  
	0x39, 0x43,                                                       
	0x38, 0x12,                                                       
	0x37, 0x00,                                                       
	0x35, 0x91,                                                       
	0x0e, 0xa0,                                                       
	0x1e, 0x04,                                                       
	                                                                  
	0xa8, 0x80,                                                       
	0x12, 0x08,                                                       
	0x04, 0x00,                                                       
	0x0c, 0x04,                                                       
	0x0d, 0x80,                                                       
	0x18, 0x7e,                                                       
	0x17, 0x26,                                                       
	0x32, 0x24,                                                       
	0x03, 0x36,                                                       
	0x1a, 0x12,                                                       
	0x19, 0x00,                                                       
	0x14, 0x2e,                                                       
	0x15, 0x02,                                                       
	0x3f, 0xa6,                                                       
	0x41, 0x02,                                                       
	0x42, 0x08,                                                       
	                                                                  
	0x1b, 0x00,                                                       
	0x16, 0x06,                                                       
	0x33, 0xe2, /*c0 for internal regulator*/                         
	0x34, 0xbf,                                                       
	0x96, 0x04,                                                       
	0x3a, 0x00,                                                       
	0x8e, 0x00,                                                       
	                                                                  
	0x3c, 0x77,                                                       
	0x8B, 0x06,                                                       
	0x94, 0x88,                                                       
	0x95, 0x88,                                                       
	0x40, 0xc1,                                                       
	0x29, 0x3f, /*2f for internal regulator*/                         
	0x0f, 0x42,                                                       
	                                                                  
	0x3d, 0x92,                                                       
	0x69, 0x40,                                                       
	0x5C, 0xb9,                                                       
	0x5D, 0x96,                                                       
	0x5E, 0x10,                                                       
	0x59, 0xc0,                                                       
	0x5A, 0xaf,                                                       
	0x5B, 0x55,                                                       
	0x43, 0xf0,                                                       
	0x44, 0x10,                                                       
	0x45, 0x68,                                                       
	0x46, 0x96,                                                       
	0x47, 0x60,                                                       
	0x48, 0x80,                                                       
	0x5F, 0xe0,                                                       
	0x60, 0x8c, /*0c for advanced AWB (related to lens)*/             
	0x61, 0x20,                                                       
	0xa5, 0xd9,                                                       
	0xa4, 0x74,                                                       
	0x8d, 0x02,                                                       
	0x13, 0xe7,                                                       
	                                                                  
	0x4f, 0x3a,                                                       
	0x50, 0x3d,                                                       
	0x51, 0x03,                                                       
	0x52, 0x12,                                                       
	0x53, 0x26,                                                       
	0x54, 0x38,                                                       
	0x55, 0x40,                                                       
	0x56, 0x40,                                                       
	0x57, 0x40,                                                       
	0x58, 0x0d,                                                       
	                                                                  
	0x8C, 0x23,                                                       
	0x3E, 0x02,                                                       
	0xa9, 0xb8,                                                       
	0xaa, 0x92,                                                       
	0xab, 0x0a,                                                       
	                                                                  
	0x8f, 0xdf,                                                       
	0x90, 0x00,                                                       
	0x91, 0x00,                                                       
	0x9f, 0x00,                                                       
	0xa0, 0x00,                                                       
	0x3a, 0x0d, /*01 YUYV*/
	                                                                  
	0x24, 0x70,                                                       
	0x25, 0x64,                                                       
	0x26, 0xc3,                                                       
	                                                                  
	0x2a, 0x00, /*00 for 50Hz*/                                       
	0x2b, 0x00, /*D2 for 50Hz*/                                       
	                                                                  
	/*gamma*/                                                         
	0x6c, 0x40,                                                       
	0x6d, 0x30,                                                       
	0x6e, 0x4b,                                                       
	0x6f, 0x60,                                                       
	0x70, 0x70,                                                       
	0x71, 0x70,                                                       
	0x72, 0x70,                                                       
	0x73, 0x70,                                                       
	0x74, 0x60,                                                       
	0x75, 0x60,                                                       
	0x76, 0x50,                                                       
	0x77, 0x48,                                                       
	0x78, 0x3a,                                                       
	0x79, 0x2e,                                                       
	0x7a, 0x28,                                                       
	0x7b, 0x22,                                                       
	0x7c, 0x04,                                                       
	0x7d, 0x07,                                                       
	0x7e, 0x10,                                                       
	0x7f, 0x28,                                                       
	0x80, 0x36,                                                       
	0x81, 0x44,                                                       
	0x82, 0x52,                                                       
	0x83, 0x60,                                                       
	0x84, 0x6c,                                                       
	0x85, 0x78,                                                       
	0x86, 0x8c,                                                       
	0x87, 0x9e,                                                       
	0x88, 0xbb,                                                       
	0x89, 0xd2,                                                       
	0x8a, 0xe6,                                                       
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gYUV_QQCIF_INITIALIZE[] =
{
	0x12, 0x80,                                                          
	0x11, 0x8f,                                                          
	0x6b, 0x4a,                                                          
	0x6a, 0x18,                                                          
	0x3b, 0x09,                                                          
	0x13, 0xe0,                                                          
	0x01, 0x80,                                                          
	0x02, 0x80,                                                          
	0x00, 0x00,                                                          
	0x10, 0x00,                                                          
	0x13, 0xe5,                                                          
	                                                                     
	0x39, 0x43,                                                          
	0x38, 0x12,                                                          
	0x37, 0x00,                                                          
	0x35, 0x91,                                                          
	0x0e, 0xa0,                                                          
	0x1e, 0x04,                                                          
	                                                                     
	0xA8, 0x80,                                                          
	0x12, 0x08,                                                          
	0x04, 0x24,                                                          
	0x0c, 0x04,                                                          
	0x0d, 0x80,                                                          
	0x18, 0x7e,                                                          
	0x17, 0x26,                                                          
	0x32, 0x24,                                                          
	0x03, 0x36,                                                          
	0x1a, 0x12,                                                          
	0x19, 0x00,                                                          
	0x14, 0x2e,                                                          
	0x15, 0x02,                                                          
	0x3f, 0xa6,                                                          
	0x41, 0x02,                                                          
	0x42, 0x08,                                                          
	                                                                     
	0x1b, 0x00,                                                          
	0x16, 0x06,                                                          
	0x33, 0xe2, /*c0 for internal regulator*/                            
	0x34, 0xbf,                                                          
	0x96, 0x04,                                                          
	0x3a, 0x00,                                                          
	0x8e, 0x00,                                                          
	                                                                     
	0x3c, 0x77,                                                          
	0x8B, 0x06,                                                          
	0x94, 0x88,                                                          
	0x95, 0x88,                                                          
	0x40, 0xc1,                                                          
	0x29, 0x3f, /*2f for internal regulator*/                            
	0x0f, 0x42,                                                          
	                                                                     
	0x3d, 0x92,                                                          
	0x69, 0x40,                                                          
	0x5C, 0xb9,                                                          
	0x5D, 0x96,                                                          
	0x5E, 0x10,                                                          
	0x59, 0xc0,                                                          
	0x5A, 0xaf,                                                          
	0x5B, 0x55,                                                          
	0x43, 0xf0,                                                          
	0x44, 0x10,                                                          
	0x45, 0x68,                                                          
	0x46, 0x96,                                                          
	0x47, 0x60,                                                          
	0x48, 0x80,                                                          
	0x5F, 0xe0,                                                          
	0x60, 0x8c, /*0c for advanced AWB (related to lens)*/                
	0x61, 0x20,                                                          
	0xa5, 0xd9,                                                          
	0xa4, 0x74,                                                          
	0x8d, 0x02,                                                          
	0x13, 0xe7,                                                          
	                                                                     
	0x4f, 0x3a,                                                          
	0x50, 0x3d,                                                          
	0x51, 0x03,                                                          
	0x52, 0x12,                                                          
	0x53, 0x26,                                                          
	0x54, 0x38,                                                          
	0x55, 0x40,                                                          
	0x56, 0x40,                                                          
	0x57, 0x40,                                                          
	0x58, 0x0d,                                                          
	                                                                     
	0x8C, 0x23,                                                          
	0x3E, 0x02,                                                          
	0xa9, 0xb8,                                                          
	0xaa, 0x92,                                                          
	0xab, 0x0a,                                                          
	                                                                     
	0x8f, 0xdf,                                                          
	0x90, 0x00,                                                          
	0x91, 0x00,                                                          
	0x9f, 0x00,                                                          
	0xa0, 0x00,                                                          
	0x3a, 0x0d, /*01 YUYV*/
	                                                                     
	0x24, 0x70,                                                          
	0x25, 0x64,                                                          
	0x26, 0xc3,                                                          
	                                                                     
	0x2a, 0x00, /*00 for 50Hz*/                                          
	0x2b, 0x00, /*D2 for 50Hz*/                                          
	                                                                     
	/*gamma*/                                                            
	0x6c, 0x40,                                                          
	0x6d, 0x30,                                                          
	0x6e, 0x4b,                                                          
	0x6f, 0x60,                                                          
	0x70, 0x70,                                                          
	0x71, 0x70,                                                          
	0x72, 0x70,                                                          
	0x73, 0x70,                                                          
	0x74, 0x60,                                                          
	0x75, 0x60,                                                          
	0x76, 0x50,                                                          
	0x77, 0x48,                                                          
	0x78, 0x3a,                                                          
	0x79, 0x2e,                                                          
	0x7a, 0x28,                                                          
	0x7b, 0x22,                                                          
	0x7c, 0x04,                                                          
	0x7d, 0x07,                                                          
	0x7e, 0x10,                                                          
	0x7f, 0x28,                                                          
	0x80, 0x36,                                                          
	0x81, 0x44,                                                          
	0x82, 0x52,                                                          
	0x83, 0x60,                                                          
	0x84, 0x6c,                                                          
	0x85, 0x78,                                                          
	0x86, 0x8c,                                                          
	0x87, 0x9e,                                                          
	0x88, 0xbb,                                                          
	0x89, 0xd2,                                                          
	0x8a, 0xe6,                                                          
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gYUV_QQVGA_INITIALIZE[] =
{
	0x12, 0x80,                                                       
	0x11, 0x87,                                                       
	0x6b, 0x4a,                                                       
	0x6a, 0x1f,                                                       
	0x3b, 0x09,                                                       
	0x13, 0xe0,                                                       
	0x01, 0x80,                                                       
	0x02, 0x80,                                                       
	0x00, 0x00,                                                       
	0x10, 0x00,                                                       
	0x13, 0xe5,                                                       
	                                                                  
	0x39, 0x43,                                                       
	0x38, 0x12,                                                       
	0x37, 0x00,                                                       
	0x35, 0x91,                                                       
	0x0e, 0xa0,                                                       
	0x1e, 0x04,                                                       
	                                                                  
	0xA8, 0x80,                                                       
	0x12, 0x10,                                                       
	0x04, 0x24,                                                       
	0x0c, 0x04,                                                       
	0x0d, 0x80,                                                       
	0x18, 0xc6,                                                       
	0x17, 0x26,                                                       
	0x32, 0xa4,                                                       
	0x03, 0x36,                                                       
	0x1a, 0x1e,                                                       
	0x19, 0x00,                                                       
	0x14, 0x2e,                                                       
	0x15, 0x02,                                                       
	0x3f, 0xa6,                                                       
	0x41, 0x02,                                                       
	0x42, 0x08,                                                       
	                                                                  
	0x1b, 0x00,                                                       
	0x16, 0x06,                                                       
	0x33, 0xe2, /*c0 for internal regulator*/                         
	0x34, 0xbf,                                                       
	0x96, 0x04,                                                       
	0x3a, 0x00,                                                       
	0x8e, 0x00,                                                       
	                                                                  
	0x3c, 0x77,                                                       
	0x8B, 0x06,                                                       
	0x94, 0x88,                                                       
	0x95, 0x88,                                                       
	0x40, 0xc1,                                                       
	0x29, 0x3f, /*2f for internal regulator*/                         
	0x0f, 0x42,                                                       
	                                                                  
	0x3d, 0x92,                                                       
	0x69, 0x40,                                                       
	0x5C, 0xb9,                                                       
	0x5D, 0x96,                                                       
	0x5E, 0x10,                                                       
	0x59, 0xc0,                                                       
	0x5A, 0xaf,                                                       
	0x5B, 0x55,                                                       
	0x43, 0xf0,                                                       
	0x44, 0x10,                                                       
	0x45, 0x68,                                                       
	0x46, 0x96,                                                       
	0x47, 0x60,                                                       
	0x48, 0x80,                                                       
	0x5F, 0xe0,                                                       
	0x60, 0x8C, /*0c for advanced AWB (Related to lens)*/             
	0x61, 0x20,                                                       
	0xa5, 0xd9,                                                       
	0xa4, 0x74,                                                       
	0x8d, 0x02,                                                       
	0x13, 0xe7,                                                       
	                                                                  
	0x4f, 0x3a,                                                       
	0x50, 0x3d,                                                       
	0x51, 0x03,                                                       
	0x52, 0x12,                                                       
	0x53, 0x26,                                                       
	0x54, 0x38,                                                       
	0x55, 0x40,                                                       
	0x56, 0x40,                                                       
	0x57, 0x40,                                                       
	0x58, 0x0d,                                                       
	                                                                  
	0x8C, 0x23,                                                       
	0x3E, 0x02,                                                       
	0xa9, 0xb8,                                                       
	0xaa, 0x92,                                                       
	0xab, 0x0a,                                                       
	                                                                  
	0x8f, 0xdf,                                                       
	0x90, 0x00,                                                       
	0x91, 0x00,                                                       
	0x9f, 0x00,                                                       
	0xa0, 0x00,                                                       
	0x3a, 0x0d, /*01 YUYV*/
	                                                                  
	0x24, 0x70,                                                       
	0x25, 0x64,                                                       
	0x26, 0xc3,                                                       
	                                                                  
	0x2a, 0x00, /*10 for 50Hz*/                                       
	0x2b, 0x00, /*40 for 50Hz*/                                       
	                                                                  
	/*gamma*/                                                         
	0x6c, 0x40,                                                       
	0x6d, 0x30,                                                       
	0x6e, 0x4b,                                                       
	0x6f, 0x60,                                                       
	0x70, 0x70,                                                       
	0x71, 0x70,                                                       
	0x72, 0x70,                                                       
	0x73, 0x70,                                                       
	0x74, 0x60,                                                       
	0x75, 0x60,                                                       
	0x76, 0x50,                                                       
	0x77, 0x48,                                                       
	0x78, 0x3a,                                                       
	0x79, 0x2e,                                                       
	0x7a, 0x28,                                                       
	0x7b, 0x22,                                                       
	0x7c, 0x04,                                                       
	0x7d, 0x07,                                                       
	0x7e, 0x10,                                                       
	0x7f, 0x28,                                                       
	0x80, 0x36,                                                       
	0x81, 0x44,                                                       
	0x82, 0x52,                                                       
	0x83, 0x60,                                                       
	0x84, 0x6c,                                                       
	0x85, 0x78,                                                       
	0x86, 0x8c,                                                       
	0x87, 0x9e,                                                       
	0x88, 0xbb,                                                       
	0x89, 0xd2,                                                       
	0x8a, 0xe6,                                                       
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gYUV_QVGA_INITIALIZE[] =
{
	0x12, 0x80,                                                        
	0x11, 0x87,                                                        
	0x6b, 0x4a,                                                        
	0x6a, 0x1f,                                                        
	0x3b, 0x09,                                                        
	0x13, 0xe0,                                                        
	0x01, 0x80,                                                        
	0x02, 0x80,                                                        
	0x00, 0x00,                                                        
	0x10, 0x00,                                                        
	0x13, 0xe5,                                                        
	                                                                   
	0x39, 0x43,                                                        
	0x38, 0x12,                                                        
	0x37, 0x00,                                                        
	0x35, 0x91,                                                        
	0x0e, 0xa0,                                                        
	0x1e, 0x04,                                                        
	                                                                   
	0xA8, 0x80,                                                        
	0x12, 0x10,                                                        
	0x04, 0x00,                                                        
	0x0c, 0x04,                                                        
	0x0d, 0x80,                                                        
	0x18, 0xc6,                                                        
	0x17, 0x26,                                                        
	0x32, 0xa4,                                                        
	0x03, 0x36,                                                        
	0x1a, 0x1e,                                                        
	0x19, 0x00,                                                        
	0x3f, 0xa6,                                                        
	0x14, 0x2e,                                                        
	0x15, 0x02,                                                        
	0x41, 0x02,                                                        
	0x42, 0x08,                                                        
	                                                                   
	0x1b, 0x00,                                                        
	0x16, 0x06,                                                        
	0x33, 0xe2, /*c0 for internal regulator*/                          
	0x34, 0xbf,                                                        
	0x96, 0x04,                                                        
	0x3a, 0x00,                                                        
	0x8e, 0x00,                                                        
	                                                                   
	0x3c, 0x77,                                                        
	0x8B, 0x06,                                                        
	0x94, 0x88,                                                        
	0x95, 0x88,                                                        
	0x40, 0xc1,                                                        
	0x29, 0x3f, /*2f for internal regulator*/                          
	0x0f, 0x42,                                                        
	                                                                   
	0x3d, 0x92,                                                        
	0x69, 0x40,                                                        
	0x5C, 0xb9,                                                        
	0x5D, 0x96,                                                        
	0x5E, 0x10,                                                        
	0x59, 0xc0,                                                        
	0x5A, 0xaf,                                                        
	0x5B, 0x55,                                                        
	0x43, 0xf0,                                                        
	0x44, 0x10,                                                        
	0x45, 0x68,                                                        
	0x46, 0x96,                                                        
	0x47, 0x60,                                                        
	0x48, 0x80,                                                        
	0x5F, 0xe0,                                                        
	0x60, 0x8c, /*0c for advanced AWB (related to lens)*/              
	0x61, 0x20,                                                        
	0xa5, 0xd9,                                                        
	0xa4, 0x74,                                                        
	0x8d, 0x02,                                                        
	0x13, 0xe7,                                                        
	                                                                   
	0x4f, 0x3a,                                                        
	0x50, 0x3d,                                                        
	0x51, 0x03,                                                        
	0x52, 0x12,                                                        
	0x53, 0x26,                                                        
	0x54, 0x38,                                                        
	0x55, 0x40,                                                        
	0x56, 0x40,                                                        
	0x57, 0x40,                                                        
	0x58, 0x0d,                                                        
	                                                                   
	0x8C, 0x23,                                                        
	0x3E, 0x02,                                                        
	0xa9, 0xb8,                                                        
	0xaa, 0x92,                                                        
	0xab, 0x0a,                                                        
	                                                                   
	0x8f, 0xdf,                                                        
	0x90, 0x00,                                                        
	0x91, 0x00,                                                        
	0x9f, 0x00,                                                        
	0xa0, 0x00,                                                        
	0x3a, 0x0d, /*01 YUYV*/
	                                                                   
	0x24, 0x70,                                                        
	0x25, 0x64,                                                        
	0x26, 0xc3,                                                        
	                                                                   
	0x2a, 0x00, /*10 for 50Hz*/                                        
	0x2b, 0x00, /*40 for 50Hz*/                                        
	                                                                   
	/*gamma*/                                                          
	0x6c, 0x40,                                                        
	0x6d, 0x30,                                                        
	0x6e, 0x4b,                                                        
	0x6f, 0x60,                                                        
	0x70, 0x70,                                                        
	0x71, 0x70,                                                        
	0x72, 0x70,                                                        
	0x73, 0x70,                                                        
	0x74, 0x60,                                                        
	0x75, 0x60,                                                        
	0x76, 0x50,                                                        
	0x77, 0x48,                                                        
	0x78, 0x3a,                                                        
	0x79, 0x2e,                                                        
	0x7a, 0x28,                                                        
	0x7b, 0x22,                                                        
	0x7c, 0x04,                                                        
	0x7d, 0x07,                                                        
	0x7e, 0x10,                                                        
	0x7f, 0x28,                                                        
	0x80, 0x36,                                                        
	0x81, 0x44,                                                        
	0x82, 0x52,                                                        
	0x83, 0x60,                                                        
	0x84, 0x6c,                                                        
	0x85, 0x78,                                                        
	0x86, 0x8c,                                                        
	0x87, 0x9e,                                                        
	0x88, 0xbb,                                                        
	0x89, 0xd2,                                                        
	0x8a, 0xe6,                                                        
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gYUV_VGA_INITIALIZE[] =
{
	0x12, 0x80,                                                   
	0x11, 0x83,                                                   
	0x6b, 0x4a,                                                   
	0x6a, 0x3e,                                                   
	0x3b, 0x09,                                                   
	0x13, 0xe0,                                                   
	0x01, 0x80,                                                   
	0x02, 0x80,                                                   
	0x00, 0x00,                                                   
	0x10, 0x00,                                                   
	0x13, 0xe5,                                                   

#ifdef MCLK_13M	                                                                    
	0x39, 0x43, /*43 for 15fps*/                                  
	0x38, 0x12, /*12 for 15fps*/                                  
	0x37, 0x00,                                                   
	0x35, 0x91, /*91 for 15fps*/ 
#else
	0x39, 0x50, /*50 for 30fps*/                                  
	0x38, 0x92, /*92 for 30fps*/                                  
	0x37, 0x00,                                                   
	0x35, 0x81, /*81 for 30fps*/ 
#endif
	0x0e, 0xa0,                                                   
	0x1e, 0x04,                                                   
	                                                              
	0xA8, 0x80,                                                   
	0x12, 0x40,                                                   
	0x04, 0x00,                                                   
	0x0c, 0x04,                                                   
	0x0d, 0x80,                                                   
	0x18, 0xc6,                                                   
	0x17, 0x26,                                                   
	0x32, 0xad,                                                   
	0x03, 0x00,                                                   
	0x1a, 0x3d,                                                   
	0x19, 0x01,                                                   
	0x3f, 0xa6,                                                   
	0x14, 0x2e,                                                   
	0x15, 0x02,                                                   
	0x41, 0x02,                                                   
	0x42, 0x08,                                                   
	                                                              
	0x1b, 0x00,                                                   
	0x16, 0x06,                                                   
	0x33, 0xe2, /*c0 for internal regulator*/                     
	0x34, 0xbf,                                                   
	0x96, 0x04,                                                   
	0x3a, 0x00,                                                   
	0x8e, 0x00,                                                   
	                                                              
	0x3c, 0x77,                                                   
	0x8B, 0x06,                                                   
	0x94, 0x88,                                                   
	0x95, 0x88,                                                   
	0x40, 0xc1,                                                   
	0x29, 0x3f, /*2f for internal regulator*/                     
	0x0f, 0x42,                                                   
	                                                              
	0x3d, 0x92,                                                   
	0x69, 0x40,                                                   
	0x5C, 0xb9,                                                   
	0x5D, 0x96,                                                   
	0x5E, 0x10,                                                   
	0x59, 0xc0,                                                   
	0x5A, 0xaf,                                                   
	0x5B, 0x55,                                                   
	0x43, 0xf0,                                                   
	0x44, 0x10,                                                   
	0x45, 0x68,                                                   
	0x46, 0x96,                                                   
	0x47, 0x60,                                                   
	0x48, 0x80,                                                   
	0x5F, 0xe0,                                                   
	0x60, 0x8c, /*0c for advanced AWB (related to lens)*/         
	0x61, 0x20,                                                   
	0xa5, 0xd9,                                                   
	0xa4, 0x74,                                                   
	0x8d, 0x02,                                                   
	0x13, 0xe7,                                                   
	                                                              
	0x4f, 0x3a,                                                   
	0x50, 0x3d,                                                   
	0x51, 0x03,                                                   
	0x52, 0x12,                                                   
	0x53, 0x26,                                                   
	0x54, 0x38,                                                   
	0x55, 0x40,                                                   
	0x56, 0x40,                                                   
	0x57, 0x40,                                                   
	0x58, 0x0d,                                                   
	                                                              
	0x8C, 0x23,                                                   
	0x3E, 0x02,                                                   
	0xa9, 0xb8,                                                   
	0xaa, 0x92,                                                   
	0xab, 0x0a,                                                   
	                                                              
	0x8f, 0xdf,                                                   
	0x90, 0x00,                                                   
	0x91, 0x00,                                                   
	0x9f, 0x00,                                                   
	0xa0, 0x00,                                                   
	0x3a, 0x0d, /*01 YUYV*/
	                                                              
	0x24, 0x70,                                                   
	0x25, 0x64,                                                   
	0x26, 0xc3,                                                   
	                                                              
	0x2a, 0x00, /*10 for 50Hz*/                                   
	0x2b, 0x00, /*40 for 50Hz*/                                   
	                                                              
	/*gamma*/                                                     
	0x6c, 0x40,                                                   
	0x6d, 0x30,                                                   
	0x6e, 0x4b,                                                   
	0x6f, 0x60,                                                   
	0x70, 0x70,                                                   
	0x71, 0x70,                                                   
	0x72, 0x70,                                                   
	0x73, 0x70,                                                   
	0x74, 0x60,                                                   
	0x75, 0x60,                                                   
	0x76, 0x50,                                                   
	0x77, 0x48,                                                   
	0x78, 0x3a,                                                   
	0x79, 0x2e,                                                   
	0x7a, 0x28,                                                   
	0x7b, 0x22,                                                   
	0x7c, 0x04,                                                   
	0x7d, 0x07,                                                   
	0x7e, 0x10,                                                   
	0x7f, 0x28,                                                   
	0x80, 0x36,                                                   
	0x81, 0x44,                                                   
	0x82, 0x52,                                                   
	0x83, 0x60,                                                   
	0x84, 0x6c,                                                   
	0x85, 0x78,                                                   
	0x86, 0x8c,                                                   
	0x87, 0x9e,                                                   
	0x88, 0xbb,                                                   
	0x89, 0xd2,                                                   
	0x8a, 0xe6,                                                   
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

const static u8 gYUV_SXGA_INITIALIZE[] =
{
	0x12, 0x80,                                                        
	0x11, 0x81,                                                        
	0x6b, 0x4a,                                                        
	0x6a, 0x41,                                                        
	0x3b, 0x09,                                                        
	0x13, 0xe0,                                                        
	0x01, 0x80,                                                        
	0x02, 0x80,                                                        
	0x00, 0x00,                                                        
	0x10, 0x00,                                                        
	0x13, 0xe5,                                                        

#ifdef MCLK_13M	                                                                    
	0x39, 0x43, /*43 for 7.5fps*/                                       
	0x38, 0x12, /*12 for 7.5fps*/                                       
	0x37, 0x00,                                                        
	0x35, 0x91, /*91 for 7.5fps*/ 
#else
	0x39, 0x50, /*50 for 15fps*/                                       
	0x38, 0x93, /*93 for 15fps*/                                       
	0x37, 0x00,                                                        
	0x35, 0x81, /*81 for 15fps*/ 
#endif
	0x0e, 0xa0,                                                        
	0x1e, 0x04,                                                        
	                                                                   
	0xA8, 0x80,                                                        
	0x12, 0x00,                                                        
	0x04, 0x00,                                                        
	0x0c, 0x00,                                                        
	0x0d, 0x00,                                                        
	0x18, 0xbd,                                                        
	0x17, 0x1d,                                                        
	0x32, 0xad,                                                        
	0x03, 0x12,                                                        
	0x1a, 0x81,                                                        
	0x19, 0x01,                                                        
	0x14, 0x2e,                                                        
	0x15, 0x00,                                                        
	0x3f, 0xa6,                                                        
	0x41, 0x02,                                                        
	0x42, 0x08,                                                        
	                                                                   
	0x1b, 0x00,                                                        
	0x16, 0x06,                                                        
	0x33, 0xe2, /*c0 for internal regulator*/                          
	0x34, 0xbf,                                                        
	0x96, 0x04,                                                        
	0x3a, 0x00,                                                        
	0x8e, 0x00,                                                        
	                                                                   
	0x3c, 0x77,                                                        
	0x8B, 0x06,                                                        
	0x94, 0x88,                                                        
	0x95, 0x88,                                                        
	0x40, 0xc1,                                                        
	0x29, 0x3f, /*2f for internal regulator*/                          
	0x0f, 0x42,                                                        
	                                                                   
	0x3d, 0x92,                                                        
	0x69, 0x40,                                                        
	0x5C, 0xb9,                                                        
	0x5D, 0x96,                                                        
	0x5E, 0x10,                                                        
	0x59, 0xc0,                                                        
	0x5A, 0xaf,                                                        
	0x5B, 0x55,                                                        
	0x43, 0xf0,                                                        
	0x44, 0x10,                                                        
	0x45, 0x68,                                                        
	0x46, 0x96,                                                        
	0x47, 0x60,                                                        
	0x48, 0x80,                                                        
	0x5F, 0xe0,                                                        
	0x60, 0x8C, /*0c for advanced AWB (Related to lens)*/              
	0x61, 0x20,                                                        
	0xa5, 0xd9,                                                        
	0xa4, 0x74,                                                        
	0x8d, 0x02,                                                        
	0x13, 0xe7,                                                        
	                                                                   
	0x4f, 0x3a,                                                        
	0x50, 0x3d,                                                        
	0x51, 0x03,                                                        
	0x52, 0x12,                                                        
	0x53, 0x26,                                                        
	0x54, 0x38,                                                        
	0x55, 0x40,                                                        
	0x56, 0x40,                                                        
	0x57, 0x40,                                                        
	0x58, 0x0d,                                                        
	                                                                   
	0x8C, 0x23,                                                        
	0x3E, 0x02,                                                        
	0xa9, 0xb8,                                                        
	0xaa, 0x92,                                                        
	0xab, 0x0a,                                                        
	                                                                   
	0x8f, 0xdf,                                                        
	0x90, 0x00,                                                        
	0x91, 0x00,                                                        
	0x9f, 0x00,                                                        
	0xa0, 0x00,                                                        
	0x3a, 0x0d, /*01 YUYV*/
	                                                                   
	0x24, 0x70,                                                        
	0x25, 0x64,                                                        
	0x26, 0xc3,                                                        
	                                                                   
	0x2a, 0x00, /*10 for 50Hz*/                                        
	0x2b, 0x00, /*34 for 50Hz*/                                        
	                                                                   
	/*gamma*/                                                          
	0x6c, 0x40,                                                        
	0x6d, 0x30,                                                        
	0x6e, 0x4b,                                                        
	0x6f, 0x60,                                                        
	0x70, 0x70,                                                        
	0x71, 0x70,                                                        
	0x72, 0x70,                                                        
	0x73, 0x70,                                                        
	0x74, 0x60,                                                        
	0x75, 0x60,                                                        
	0x76, 0x50,                                                        
	0x77, 0x48,                                                        
	0x78, 0x3a,                                                        
	0x79, 0x2e,                                                        
	0x7a, 0x28,                                                        
	0x7b, 0x22,                                                        
	0x7c, 0x04,                                                        
	0x7d, 0x07,                                                        
	0x7e, 0x10,                                                        
	0x7f, 0x28,                                                        
	0x80, 0x36,                                                        
	0x81, 0x44,                                                        
	0x82, 0x52,                                                        
	0x83, 0x60,                                                        
	0x84, 0x6c,                                                        
	0x85, 0x78,                                                        
	0x86, 0x8c,                                                        
	0x87, 0x9e,                                                        
	0x88, 0xbb,                                                        
	0x89, 0xd2,                                                        
	0x8a, 0xe6,                                                        
	OV9653_REGEND, 0x00	    /* End of list delimiter */
};

static int read_sensor_reg( const u8 subAddress, u8 *bufP );
static int write_sensor_reg( const u8 subAddress, u8 *bufP );

struct OV9653_SETTING
{
	int   format;
	int   resolution;
	const u8 *setting;
};

struct OV9653_SETTING ov9653_setting_table[] =
{
	{OV_FORMAT_RAW8   , OV_SIZE_QQVGA, gRAW_QQVGA_INITIALIZE},
	{OV_FORMAT_RAW8   , OV_SIZE_QVGA , gRAW_QVGA_INITIALIZE},
	{OV_FORMAT_RAW8   , OV_SIZE_VGA  , gRAW_VGA_INITIALIZE},
	{OV_FORMAT_RAW8   , OV_SIZE_SXGA , gRAW_SXGA_INITIALIZE},
	{OV_FORMAT_RAW8   , OV_SIZE_QQCIF, gRAW_QQCIF_INITIALIZE},
	{OV_FORMAT_RAW8   , OV_SIZE_QCIF , gRAW_QCIF_INITIALIZE},
	{OV_FORMAT_RAW8   , OV_SIZE_CIF  , gRAW_CIF_INITIALIZE},
	{OV_FORMAT_YUV_422, OV_SIZE_QQVGA, gYUV_QQVGA_INITIALIZE},
	{OV_FORMAT_YUV_422, OV_SIZE_QVGA , gYUV_QVGA_INITIALIZE},
	{OV_FORMAT_YUV_422, OV_SIZE_VGA  , gYUV_VGA_INITIALIZE},
	{OV_FORMAT_YUV_422, OV_SIZE_SXGA , gYUV_SXGA_INITIALIZE},
	{OV_FORMAT_YUV_422, OV_SIZE_QQCIF, gYUV_QQCIF_INITIALIZE},
	{OV_FORMAT_YUV_422, OV_SIZE_QCIF , gYUV_QCIF_INITIALIZE},
	{OV_FORMAT_YUV_422, OV_SIZE_CIF  , gYUV_CIF_INITIALIZE},
	{OV_FORMAT_RGB_565, OV_SIZE_QQVGA, gRGB565_QQVGA_INITIALIZE},
	{OV_FORMAT_RGB_565, OV_SIZE_QVGA , gRGB565_QVGA_INITIALIZE},
	{OV_FORMAT_RGB_565, OV_SIZE_VGA  , gRGB565_VGA_INITIALIZE},
	{OV_FORMAT_RGB_565, OV_SIZE_SXGA , gRGB565_SXGA_INITIALIZE},
	{OV_FORMAT_RGB_565, OV_SIZE_QQCIF, gRGB565_QQCIF_INITIALIZE},
	{OV_FORMAT_RGB_565, OV_SIZE_QCIF , gRGB565_QCIF_INITIALIZE},
	{OV_FORMAT_RGB_565, OV_SIZE_CIF  , gRGB565_CIF_INITIALIZE},
	{0, 0, 0}
} ;

static int ov9653hw_read_sensor_reg( const u8 subAddress, u8 *bufP )
{
	return read_sensor_reg(subAddress, bufP);
}

static int ov9653hw_write_sensor_reg( const u8 subAddress, u8 *bufP )
{
	return write_sensor_reg(subAddress, bufP);
}

static int ov9653hw_set_regs( const u8 *regP )
{
	u32	curReg = 0;
	int	status = 0;

	/* The list is a register number followed by the value */
	while( regP[curReg << 1] < OV9653_REGEND )
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
static void ov9653hw_power_down( u8 powerMode )
{
	/* OV9653 PWRDWN, 0 = NORMAL, 1=POWER DOWN */
	if( powerMode == CAMERA_POWER_OFF )
		sensor_power_off();
	else
		sensor_power_on();

	mdelay(100);
}

/*****************************************************************************
 *  Settings
 *****************************************************************************/
static int ov9653hw_version_revision(u8 * pCmRevision, u8 *pSensorRevision)
{
	read_sensor_reg( OV9653_PID, pCmRevision );
	read_sensor_reg( OV9653_VER, pSensorRevision );
	return 0;
}

/*****************************************************************************
 *  Viewfinder, still
 *****************************************************************************/
static int ov9653hw_viewfinder_on(void)
{
	u8	com3;

	read_sensor_reg( OV9653_COM3, &com3 );
	com3 &= ~0x01;
	write_sensor_reg( OV9653_COM3, &com3 );

	return 0;
}


static int ov9653hw_viewfinder_off(void)
{
	u8	com3;

	read_sensor_reg( OV9653_COM3, &com3 );
	com3 |= 0x01;
	write_sensor_reg( OV9653_COM3, &com3 );

	return 0;
}


/*****************************************************************************
 *  Format
 *****************************************************************************/
static int ov9653hw_set_format( u32 captureSizeFormat, u32 colorFormat, u32 mode)
{

	int	status;
	u8	*regsP, regValue;
	const u8	*defaultDataP;
	struct OV9653_SETTING  *ov9653_setting;

	defaultDataP  = NULL;
	for(ov9653_setting = ov9653_setting_table;
		ov9653_setting->format != 0;ov9653_setting++) {
		if ((colorFormat == ov9653_setting->format) &&
			(captureSizeFormat == ov9653_setting->resolution)) {
			defaultDataP = ov9653_setting->setting;
			break;
		}
	}

	if (defaultDataP == NULL)
		return -EPERM;

	/* Get the pointer to the basic setting.
	 * The pointer must be freed after exiting.
	 */
	regsP = (u8 *) defaultDataP;

	/* Blast the entire parameter tree into the part */
	status = ov9653hw_set_regs( regsP );

	/*Assert UYVY sequence*/
	if(colorFormat == OV_FORMAT_YUV_422){
		regValue = 0x0d;
		write_sensor_reg(OV9653_TSLB, &regValue);
	}

	return 0;
}

/*****************************************************************************
 * Contrast
 *****************************************************************************/
const static u8 ContrastLowestSettings[] = {
	0x6C, 0x80,
	0x6D, 0xa0,
	0x6E, 0x78,
	0x6F, 0x50,
	0x70, 0x48,
	0x71, 0x40,
	0x72, 0x48,
	0x73, 0x40,
	0x74, 0x40,
	0x75, 0x40,
	0x76, 0x40,
	0x77, 0x40,
	0x78, 0x3e,
	0x79, 0x3c,
	0x7A, 0x3c,
	0x7B, 0x28,
	0x7C, 0x8,
	0x7D, 0x12,
	0x7E, 0x21,
	0x7F, 0x35,
	0x80, 0x3e,
	0x81, 0x46,
	0x82, 0x4f,
	0x83, 0x57,
	0x84, 0x5f,
	0x85, 0x67,
	0x86, 0x77,
	0x87, 0x87,
	0x88, 0xa6,
	0x89, 0xc4,
	0x8A, 0xe2,
	OV9653_REGEND, 	0x00		/* End of list delimiter */
};

const static u8 ContrastLowSettings[] = {
	0x6C, 0x70,
	0x6D, 0x80,
	0x6E, 0x40,
	0x6F, 0x54,
	0x70, 0x58,
	0x71, 0x60,
	0x72, 0x60,
	0x73, 0x60,
	0x74, 0x50,
	0x75, 0x58,
	0x76, 0x44,
	0x77, 0x3c,
	0x78, 0x30,
	0x79, 0x28,
	0x7A, 0x22,
	0x7B, 0x44,
	0x7C, 0x7,
	0x7D, 0xf,
	0x7E, 0x17,
	0x7F, 0x2c,
	0x80, 0x37,
	0x81, 0x43,
	0x82, 0x4f,
	0x83, 0x5b,
	0x84, 0x65,
	0x85, 0x70,
	0x86, 0x81,
	0x87, 0x90,
	0x88, 0xa8,
	0x89, 0xbc,
	0x8A, 0xcd,
	OV9653_REGEND, 	0x00		/* End of list delimiter */
};

const static u8 ContrastMiddleSettings[] = {
	0x6C, 0x40,
	0x6D, 0x30,
	0x6E, 0x4B,
	0x6F, 0x60,
	0x70, 0x70,
	0x71, 0x70,
	0x72, 0x70,
	0x73, 0x70,
	0x74, 0x60,
	0x75, 0x60,
	0x76, 0x50,
	0x77, 0x48,
	0x78, 0x3A,
	0x79, 0x2E,
	0x7A, 0x28,
	0x7B, 0x22,
	0x7C, 0x4,
	0x7D, 0x7,
	0x7E, 0x10,
	0x7F, 0x28,
	0x80, 0x36,
	0x81, 0x44,
	0x82, 0x52,
	0x83, 0x60,
	0x84, 0x6C,
	0x85, 0x78,
	0x86, 0x8C,
	0x87, 0x9E,
	0x88, 0xBB,
	0x89, 0xD2,
	0x8A, 0xE6,
	OV9653_REGEND, 	0x00		/* End of list delimiter */
};

const static u8 ContrastHighSettings[] = {
	0x6c, 0x10,
	0x6d, 0x10,
	0x6e, 0x18,
	0x6f, 0x5c,
	0x70, 0x90,
	0x71, 0x90,
	0x72, 0x90,
	0x73, 0x90,
	0x74, 0x80,
	0x75, 0x80,
	0x76, 0x60,
	0x77, 0x5c,
	0x78, 0x44,
	0x79, 0x24,
	0x7a, 0x1a,
	0x7b, 0x10,
	0x7c, 0x1,
	0x7d, 0x2,
	0x7e, 0x5,
	0x7f, 0x1c,
	0x80, 0x2e,
	0x81, 0x40,
	0x82, 0x52,
	0x83, 0x64,
	0x84, 0x74,
	0x85, 0x84,
	0x86, 0x9c,
	0x87, 0xb3,
	0x88, 0xd5,
	0x89, 0xe7,
	0x8a, 0xf4,
	OV9653_REGEND, 	0x00		/* End of list delimiter */
};

const static u8 ContrastHighestSettings[] = {
	0x6c, 0x20,
	0x6d, 0x40,
	0x6e, 0x10,
	0x6f, 0x38,
	0x70, 0x80,
	0x71, 0xe0,
	0x72, 0xd0,
	0x73, 0xe8,
	0x74, 0xa0,
	0x75, 0x80,
	0x76, 0x80,
	0x77, 0x54,
	0x78, 0x30,
	0x79, 0x22,
	0x7a, 0x8,
	0x7b, 0x1,
	0x7c, 0x2,
	0x7d, 0x6,
	0x7e, 0x8,
	0x7f, 0x16,
	0x80, 0x26,
	0x81, 0x42,
	0x82, 0x5c,
	0x83, 0x79,
	0x84, 0x8d,
	0x85, 0x9d,
	0x86, 0xbd,
	0x87, 0xd2,
	0x88, 0xea,
	0x89, 0xfb,
	0x8a, 0xff,
	OV9653_REGEND, 	0x00		/* End of list delimiter */
};

static int ov9653hw_set_contrast(u32 value)
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
		ov9653hw_set_regs(regP);
	return 0;
}

/*****************************************************************************
 * Exposure
 *****************************************************************************/
const static u8 ExposureSettings[] = {
	0x40, 0x30, 0x81, /* EV-2 */
	0x58, 0x48, 0x91, /* EV-1 */
	0x88, 0x7c, 0x93, /* EV0 */
	0xa0, 0x90, 0xb4, /* EV+1 */
	0xc0, 0xb0, 0xd6, /* EV+2 */
};

static int ov9653hw_set_exposure(u32 value)
{
	u8 aew, aeb, vpt;
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

	aew = aeb = vpt = 0;

	if (index != -1){
		aew = ExposureSettings[index];
		aeb = ExposureSettings[index + 1];
		vpt  = ExposureSettings[index + 2];
	}

	/* set hw */
	if (aew || aeb || vpt)
	{
		ov9653hw_write_sensor_reg( 0x24, &aew);
		ov9653hw_write_sensor_reg( 0x25, &aeb);
		ov9653hw_write_sensor_reg( 0x26, &vpt);
	}

	return 0;
}

/*****************************************************************************
 * Auto White Balance
 *****************************************************************************/
const static u8 AWBAuto[] = {
	0x13, 0xad,
	0x01, 0x80,
	0x02, 0x80,
	0x60, 0x14,
	0x5f, 0x05,
	0x13, 0xaf,
	OV9653_REGEND, 	0x00		/* End of list delimiter */
};

const static u8 AWBFluorescent[] = {
	0x13, 0xad,
	0x01, 0x6c,
	0x02, 0x2e,
	0x5f, 0x05,
	OV9653_REGEND, 	0x00		/* End of list delimiter */
};

const static u8 AWBOutdoor[] = {
	0x13, 0xad,
	0x01, 0x44,
	0x02, 0x44,
	0x5f, 0x05,
	OV9653_REGEND, 	0x00		/* End of list delimiter */
};

const static u8 AWBIncandescent[] = {
	0x13, 0xad,
	0x01, 0x6c,
	0x02, 0x20,
	0x5f, 0x05,
	OV9653_REGEND, 	0x00		/* End of list delimiter */
};


static int ov9653hw_set_white_balance(u32 value)
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
		ov9653hw_set_regs(regP);
	}
	return 0;
}

static void ov9653hw_auto_function_on(void)
{
	ov9653hw_set_white_balance(SENSOR_WHITEBALANCE_AUTO);
}

static void ov9653hw_auto_function_off(void)
{
	u8 val;
	read_sensor_reg( OV9653_COM8, &val );
	val &= ~0x07;	/* don't disturb AWB */
	write_sensor_reg( OV9653_COM8, &val );
}

/*****************************************************************************
 * OV9653 Functions
 *****************************************************************************/
static int ov9653_init( p_camera_context_t camera_context )
{
	u8 cm_rev, cm_pid;
	u32 timeout;
	int status;

	/* provide informat about the capabilities of the sensor */
	camera_context->sensor_status.caps |= SENSOR_CAP_MANUAL_CONTRAST |
		SENSOR_CAP_MANUAL_WHITEBALANCE |
		SENSOR_CAP_MANUAL_EXPOSURE;

	/* Configure CI according to OV9653's hardware
	 * master parallel with 8 data pins
	 */
	ci_set_mode(CI_MODE_MP, CI_DATA_WIDTH8);

	/* enable pixel clock(sensor will provide pclock)
	 * and master clock = 13MHZ
	 */
	ci_set_clock(1, 1, CICLK);

	/* data sample on rising and h,vsync active high */
	ci_set_polarity(0, 0, 0);

	/* fifo control */
	ci_set_fifo(0, CI_FIFO_THL_32, 1, 1);

	/* set black level */
	ci_cgu_set_black_level(0);

	/* CGU Mux Select */
	ci_cgu_set_addr_mux_select(CI_CGU_MUX_0_TO_7);


	/* OV9653 Power on sequence
	 * Take out of Power down mode (GPIO_103), PWRDWN=1, NORMAL=0
	 * Assert Reset
	 * Delay
	 * Remove reset
	 * Delay
	 */
	ov9653hw_power_down(CAMERA_POWER_FULL);

	/* read out version */
	timeout = 50;
	do
	{
		cm_pid = cm_rev = 0;
		status = ov9653hw_version_revision(&cm_pid, &cm_rev);

		/* Check to make sure we are working with an OV9653 */
		if( cm_pid != PID_OV96XX || cm_rev != PID_9653 )
		{
			ov9653hw_power_down( CAMERA_POWER_OFF );
			ov9653hw_power_down( CAMERA_POWER_FULL );
			mdelay(1);
		}
		if (--timeout == 0) {
			return -EIO;
		}
	}
	while( cm_pid != PID_OV96XX );

	/* turn sensor output off */
	ov9653hw_viewfinder_off();

	return 0;
}

static int ov9653_deinit(  p_camera_context_t camera_context )
{
	/* power off the external module */
	ov9653hw_power_down( CAMERA_POWER_OFF );

	return 0;
}

static int ov9653_sleep(  p_camera_context_t camera_context )
{
	return ov9653_deinit( camera_context );
}

static int ov9653_wake(  p_camera_context_t camera_context )
{
	return ov9653_init( camera_context );
}


static int ov9653_set_capture_format(p_camera_context_t camera_context )
{
	CI_MP_TIMING timing;
	int status;
	u32 ovSizeFormat, ovFormat;

	/* Set the current mode */
	switch(camera_context->capture_input_format) {
		case V4L2_PIX_FMT_YUV422P:
			ovFormat = OV_FORMAT_YUV_422;
			break;
		case V4L2_PIX_FMT_RGB565X:
			ovFormat = OV_FORMAT_RGB_565;
			break;
		case V4L2_PIX_FMT_SRGGB8:
			ovFormat = OV_FORMAT_RAW8;
			break;
		default:
			ovFormat = OV_FORMAT_YUV_422;
			break;
	}

	if ( camera_context->capture_input_width == 88 &&
			camera_context->capture_input_height == 72 )
		ovSizeFormat = OV_SIZE_QQCIF;
	else if ( camera_context->capture_input_width == 176 &&
			camera_context->capture_input_height == 144 )
		ovSizeFormat = OV_SIZE_QCIF;
	else if ( camera_context->capture_input_width == 352 &&
			camera_context->capture_input_height == 288 )
		ovSizeFormat = OV_SIZE_CIF;
	else if ( camera_context->capture_input_width == 160 &&
			camera_context->capture_input_height == 120 )
		ovSizeFormat = OV_SIZE_QQVGA;
	else if ( camera_context->capture_input_width == 320 &&
			camera_context->capture_input_height == 240 )
		ovSizeFormat = OV_SIZE_QVGA;
	else if ( camera_context->capture_input_width == 640 &&
			camera_context->capture_input_height == 480 )
		ovSizeFormat = OV_SIZE_VGA;
	else if ( camera_context->capture_input_width == 1280 &&
			camera_context->capture_input_height == 1024 )
		ovSizeFormat = OV_SIZE_SXGA;
	else
		ovSizeFormat = OV_SIZE_SXGA;

	status = ov9653hw_set_format( ovSizeFormat,
			ovFormat, camera_context->capture_mode);

	/* set capture width/height and timing */
	if(ovSizeFormat == OV_SIZE_SXGA){
		timing.BFW = 0x01;
		timing.BLW = 0x00;
	}else{
		timing.BFW = 0x01; /*wait one line clock to delete the first line*/
		timing.BLW = 0x00;
	}

	ci_configure_mp(camera_context->capture_input_width-1,
			camera_context->capture_input_height-1, &timing);

	if (cpu_is_pxa930() || cpu_is_pxa935())
		ci_set_ycbcr_420_down_sample (camera_context->ycbcr_ds);

	return 0;
}

static int ov9653_start_capture(p_camera_context_t camera_context,
		unsigned int frames)
{
	(void) camera_context;

	/* turn auto function on only doing continues capture */
	if (frames == 0) {
		ov9653hw_auto_function_on();
	} else {
		ov9653hw_auto_function_off();
	}

	/* turn viewfinder on */
	ov9653hw_viewfinder_on();
	
	return 0;
}

static int ov9653_stop_capture(  p_camera_context_t camera_context )
{
	(void) camera_context;

	/* turn auto function off */
	ov9653hw_auto_function_off();

	/* turn viewfinder off */
	ov9653hw_viewfinder_off();

	return 0;
}

static int ov9653_set_power_mode(p_camera_context_t camera_context, u8 mode)
{
	(void) camera_context;

	ov9653hw_power_down( mode );
	return 0;
}

static int ov9653_read_8bit( p_camera_context_t camera_context,
		u8 reg_addr,  u8 *reg_val)
{
	(void) camera_context;

	ov9653hw_read_sensor_reg( reg_addr, reg_val );
	return 0;
}

static int ov9653_write_8bit( p_camera_context_t camera_context,
		u8 reg_addr,  u8 reg_val)
{
	u8 buffer;
	int status;

	(void) camera_context;

	buffer = reg_val;
	status = (ov9653hw_write_sensor_reg( reg_addr, &buffer ) == 0) ?
		0 : -EIO;

	return status;
}

static int ov9653_set_contrast( p_camera_context_t camera_context, u8 mode, u32 value)
{
	(void) camera_context;

	if (mode == SENSOR_MANUAL_CONTRAST)
		return ov9653hw_set_contrast(value);
	else
		return 0;
}

static int ov9653_set_exposure( p_camera_context_t camera_context, u8 mode, u32 value)
{
	(void) camera_context;

	if (mode == SENSOR_MANUAL_EXPOSURE)
		return ov9653hw_set_exposure(value);
	else
		return 0;
}

static int ov9653_set_white_balance( p_camera_context_t camera_context,
		u8 mode, u32 value)
{
	(void) camera_context;

	if (mode == SENSOR_MANUAL_WHITEBALANCE)
		return ov9653hw_set_white_balance(value);
	else
		return 0;
}

static int ov9653_get_framerate(u32 format, u32 width, u32 height, u32 *numerator, u32 *denominator)
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

static char name[] = "Omnivision9653";

static p_camera_function_t camera_functions;

static int format_list[] = {
	V4L2_PIX_FMT_YUV422P,
	V4L2_PIX_FMT_SRGGB8,
	V4L2_PIX_FMT_RGB565X,
	-1	
};

static int ov9653_register(int id)
{

	/* allocte camera functions context */
	camera_functions = kzalloc(sizeof(camera_function_t), GFP_KERNEL);
	if (!camera_functions) {
		printk("Can't allocate buffer for camera functions structure \n");
		return -ENOMEM;
	}

	camera_functions->width_max = 2048;
	camera_functions->height_max = 1536;
	camera_functions->width_min = 2;
	camera_functions->height_min = 2;

	camera_functions->v_power = 2800;		

	camera_functions->format_list = format_list;

	camera_functions->init = ov9653_init;     	
	camera_functions->deinit = ov9653_deinit;     
	camera_functions->set_capture_format = ov9653_set_capture_format; 	
	camera_functions->start_capture =	ov9653_start_capture;	 		
	camera_functions->stop_capture =	ov9653_stop_capture; 			
	camera_functions->sleep = ov9653_sleep;	
	camera_functions->wakeup = ov9653_wake;	
	camera_functions->read_8bit = ov9653_read_8bit;			
	camera_functions->write_8bit = ov9653_write_8bit;			
	camera_functions->read_16bit = NULL;		
	camera_functions->write_16bit = NULL;		
	camera_functions->read_32bit = NULL;	
	camera_functions->write_32bit = NULL;	
	camera_functions->set_power_mode = ov9653_set_power_mode;			
	camera_functions->set_contrast =	ov9653_set_contrast;			
	camera_functions->set_whitebalance =	ov9653_set_white_balance;		
	camera_functions->set_exposure =	 ov9653_set_exposure;			
	camera_functions->set_zoom = NULL;

	camera_functions->get_framerate = ov9653_get_framerate;
	
	camera_functions->name = name;
	camera_functions->id = id;

	/*register 9653 as high resolution */

	if(sensor_register(camera_functions, id) < 0){
		printk("sensor_register failed !\n");
		kfree(camera_functions);
		camera_functions = NULL;
		return -1;
	}
	return 0;
}

static int ov9653_unregister(int id)
{
	sensor_unregister(id);
	
	if(!camera_functions){
		kfree(camera_functions);
		camera_functions = NULL;
	}
	return 0;
}

/*****************************************************************************
 *                              OV9653 I2C Client Driver
 *****************************************************************************/
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
	if (IS_ERR(clk)) {
		printk("sensor failed to get camera clock\n");
		return -1;
	}
	clk_enable(clk);

	ci_set_clock(1, 1, CICLK);
	
	pdata->power_on(id);
	mdelay(1);
	
	pidh = i2c_read(client, OV9653_PID);
	pidl = i2c_read(client, OV9653_VER);
	if ((pidh != PID_OV96XX) || (pidl != PID_9653)){
		goto err;
	}

	if(ov9653_register(id)<0)
		goto err;

	g_client = client;

	printk("OV9653 detected.\n");
	
	ci_set_clock(0, 0, CICLK);
	pdata->power_off(id);

	clk_disable(clk);
	clk_put(clk);

	return 0;
err:
	printk("OV9653 detect failed.\n");
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
	
	ov9653_unregister(pdata->id);
	return 0;
}

static const struct i2c_device_id sensor_ov9653_id[] = {
	{ "sensor_ov9653", 0 },
	{ }
};

static struct i2c_driver sensor_driver = {
	.driver = {
		.name	= "sensor_ov9653",
	},
	.probe		= sensor_probe,
	.remove		= sensor_remove,
	.id_table	= sensor_ov9653_id,
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

MODULE_DESCRIPTION("OV9653 I2C Client driver");
MODULE_LICENSE("GPL");
