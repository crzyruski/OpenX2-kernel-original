/*
    pxa2xx_v4l2ov2.h - PXA lcd driver v4l2 overlay2 header file

    Copyright (C) 2007, Marvell  Corporation

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

#ifndef __PXA2XX_V4L2_OV2_H_
#define __PXA2XX_V4L2_OV2_H_

#include <linux/videodev2.h>
#include <linux/videodev.h>
#include <media/v4l2-common.h>
#include <mach/pxafb.h>

#define MAX_NR_PXA2XX_V4L2OV2_BUF 32

struct pxa2xx_v4l2ov2_format {
	char *name;
	int  fourcc;          /* video4linux 2      */
	int  depth;           /* bit/pixel          */
	int  flags;
};

/* PXA LCD DMA descriptor */
struct pxa2xx_lcd_dma_descriptor {
	unsigned int fdadr;
	unsigned int fsadr;
	unsigned int fidr;
	unsigned int ldcmd;
};

struct pxa2xx_v4l2ov2_dev {
	unsigned int format;
	unsigned int bpp;

	int 		task_waiting;
	
	/* frame buffer */
	enum v4l2_buf_type       type;	
	struct v4l2_framebuffer fbuf;

	/* overlay2 window */
	struct v4l2_window win;	
	unsigned int image_size;

	/* overlay2 status */
	unsigned char device_state;
	unsigned char driver_state;
	
	/* overlay2 special info */	
	u_int	ylen;
	u_int	cblen;
	u_int	crlen;
	u_int	yoff;
	u_int	cboff; 
	u_int	croff;	
	u_int	line_length;

	struct  pxa2xx_v4l2ov2_format *ovfmt;

	/* DMA */
	u_int map_size;
	u_char *map_cpu;
	dma_addr_t map_dma;
	u_int smem_start;
	u_int smem_len;
	u_int dma_changed;

	/* lcd base plane */
	struct pxafb_info *basefb;
		
	/* descriptor */
	struct {
		struct pxa2xx_lcd_dma_descriptor *dma2;
		struct pxa2xx_lcd_dma_descriptor *dma3;
		struct pxa2xx_lcd_dma_descriptor *dma4;
	};
	
	struct  pxa2xx_v4l2ov2_queue *queue;	

	/* driver status*/
	unsigned int streaming;
	struct device	*dev;
};

struct buffer_node {
	unsigned long io_type;
	struct list_head head;   /*free list*/
	int buf_index;
	int size;	
	u_int boff;
	u_int map_size;       	/*allocated size*/
	u_char * map_cpu;		/*vitural address*/
	dma_addr_t map_dma;		/*physical address*/
};

struct free_queue {		
	wait_queue_head_t       done;
	struct list_head free_head;   /*free list*/
};

struct pxa2xx_v4l2ov2_queue {
	struct free_queue *free_queue;   /*free list*/
	struct list_head overlay2_head;    /*overlay2 list*/
	struct buffer_node *last_buffer;   /*last buffer being displayed */	
	unsigned long ov_state;
	unsigned long buf_ready;
	struct buffer_node   bufs[MAX_NR_PXA2XX_V4L2OV2_BUF];
};

#endif /* __PXA2XX_V4L2_OV2_H_ */

