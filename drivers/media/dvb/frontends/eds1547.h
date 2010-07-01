/* eds1547.h Earda EDS-1547 tuner support
*
* Copyright (C) 2008 Igor M. Liplianin (liplianin@me.by)
*
*	This program is free software; you can redistribute it and/or modify it
*	under the terms of the GNU General Public License as published by the
*	Free Software Foundation, version 2.
*
* see Documentation/dvb/README.dvb-usb for more information
*/

#ifndef EDS1547
#define EDS1547

static u8 stv0288_earda_inittab[] = {
	0x01, 0x57,
	0x02, 0x20,
	0x03, 0x8e,
	0x04, 0x8e,
	0x05, 0x12,
	0x06, 0x00,
	0x07, 0x00,
	0x09, 0x00,
	0x0a, 0x04,
	0x0b, 0x00,
	0x0c, 0x00,
	0x0d, 0x00,
	0x0e, 0xd4,
	0x0f, 0x30,
	0x11, 0x44,
	0x12, 0x03,
	0x13, 0x48,
	0x14, 0x84,
	0x15, 0x45,
	0x16, 0xb7,
	0x17, 0x9c,
	0x18, 0x00,
	0x19, 0xa6,
	0x1a, 0x88,
	0x1b, 0x8f,
	0x1c, 0xf0,
	0x20, 0x0b,
	0x21, 0x54,
	0x22, 0x00,
	0x23, 0x00,
	0x2b, 0xff,
	0x2c, 0xf7,
	0x30, 0x00,
	0x31, 0x1e,
	0x32, 0x14,
	0x33, 0x0f,
	0x34, 0x09,
	0x35, 0x0c,
	0x36, 0x05,
	0x37, 0x2f,
	0x38, 0x16,
	0x39, 0xbd,
	0x3a, 0x00,
	0x3b, 0x13,
	0x3c, 0x11,
	0x3d, 0x30,
	0x40, 0x63,
	0x41, 0x04,
	0x42, 0x60,
	0x43, 0x00,
	0x44, 0x00,
	0x45, 0x00,
	0x46, 0x00,
	0x47, 0x00,
	0x4a, 0x00,
	0x50, 0x10,
	0x51, 0x36,
	0x52, 0x09,
	0x53, 0x94,
	0x54, 0x62,
	0x55, 0x29,
	0x56, 0x64,
	0x57, 0x2b,
	0x58, 0x54,
	0x59, 0x86,
	0x5a, 0x00,
	0x5b, 0x9b,
	0x5c, 0x08,
	0x5d, 0x7f,
	0x5e, 0x00,
	0x5f, 0xff,
	0x70, 0x00,
	0x71, 0x00,
	0x72, 0x00,
	0x74, 0x00,
	0x75, 0x00,
	0x76, 0x00,
	0x81, 0x00,
	0x82, 0x3f,
	0x83, 0x3f,
	0x84, 0x00,
	0x85, 0x00,
	0x88, 0x00,
	0x89, 0x00,
	0x8a, 0x00,
	0x8b, 0x00,
	0x8c, 0x00,
	0x90, 0x00,
	0x91, 0x00,
	0x92, 0x00,
	0x93, 0x00,
	0x94, 0x1c,
	0x97, 0x00,
	0xa0, 0x48,
	0xa1, 0x00,
	0xb0, 0xb8,
	0xb1, 0x3a,
	0xb2, 0x10,
	0xb3, 0x82,
	0xb4, 0x80,
	0xb5, 0x82,
	0xb6, 0x82,
	0xb7, 0x82,
	0xb8, 0x20,
	0xb9, 0x00,
	0xf0, 0x00,
	0xf1, 0x00,
	0xf2, 0xc0,
	0xff,0xff,
};

static struct stv0288_config earda_config = {
	.demod_address = 0x68,
	.min_delay_ms = 100,
	.inittab = stv0288_earda_inittab,
};

#endif
