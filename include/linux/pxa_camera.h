/*
    pxa_camera - PXA camera driver header file

    Copyright (C) 2003, Intel Corporation

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

#ifndef __LINUX_PXA_CAMERA_H_
#define __LINUX_PXA_CAMERA_H_

#include <asm/ioctl.h>
#include <linux/videodev2.h>




/*
 * histogram feature
 */

typedef enum {
    HST_COLOR_RED,
    HST_COLOR_BLUE,
    HST_COLOR_GREEN1,
    HST_COLOR_GREEN2
} hst_color_type;


/*
 * private pxa-camera register I/O control
 */
struct reg_set_s {
    int  val1;
    int  val2;
};

#define WCAM_VIDIOCSCAMREG       _IOW('v', 211, struct reg_set_s)
#define WCAM_VIDIOCGCAMREG       _IOWR('v', 212, struct reg_set_s)
#define WCAM_VIDIOCSCIREG        _IOW('v', 213, struct reg_set_s)
#define WCAM_VIDIOCGCIREG        _IOWR('v', 214, struct reg_set_s)
#define WCAM_VIDIOCSINFOR        _IOW('v', 215, struct reg_set_s)
#define WCAM_VIDIOCGINFOR        _IOWR('v', 216, struct reg_set_s)

/*
 *Image format definition
 */


/*
 * SSU scaling parameter
 */
typedef enum {
    CI_SSU_SCALE_DISABLE   = 0,    // disabled
    CI_SSU_SCALE_HALF      = 1,    // 2:1
    CI_SSU_SCALE_QUARTER   = 2     // 4:1
} CI_SSU_SCALE;



/*
 * extended pixel format for V4l2
 */
#define V4L2_PIX_FMT_SRGGB8  v4l2_fourcc('R','A','8','1') /*  8  RGRG.. GBGB.. */
#define V4L2_PIX_FMT_SRGGB10 v4l2_fourcc('R','A','A','1') /*  10  RGRG.. GBGB.. */

#endif /* __LINUX_PXA_CAMERA_H_ */
