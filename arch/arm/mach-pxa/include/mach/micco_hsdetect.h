/*
 * micco_hsdetect.h
 *
 * The headset detect driver based on micco
 *
 * Copyright (2008) Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _LINUX_MICCO_HSDETECT_H_
#define _LINUX_MICCO_HSDETECT_H_

#define HEADSET_REMOVE	      0
#define HEADSET_ADD	      1
#define HOOKSWITCH_PRESSED    1
#define HOOKSWITCH_RELEASED   0

struct HS_IOCTL {
	int hsdetect_status;
	int hookswitch_status;
	};

/*
 * ioctl calls that are permitted to the /dev/micco_hsdetect interface.
 */

#define HSDETECT_STATUS		_IO('d', 0x01)	/* Headset detection status*/
#define HOOKSWITCH_STATUS	_IO('d', 0x02)	/* Hook switch status */

#endif /* _LINUX_MICCO_HSDETECT_H_ */
