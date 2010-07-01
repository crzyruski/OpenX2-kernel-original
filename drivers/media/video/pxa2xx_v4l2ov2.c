/*
 * pxa2xx_v4l2ov2.c - main file for V4L2 for PXA2xx/3xx LCD overlay2
 *
 * Copyright (C) 2007, marvell Corporation.
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/videodev2.h>
#include <linux/videodev.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <mach/bitfield.h>
#include <mach/pxa-regs.h>
#include <asm-generic/fcntl.h>
#include <asm/ioctl.h>
#include <mach/pxafb.h>
#include <mach/pxa2xx_v4l2ov2.h>
#include "../../video/pxafb.h"

 
#define FORMAT_FLAGS_DITHER       0x01
#define FORMAT_FLAGS_PACKED       0x02
#define FORMAT_FLAGS_PLANAR       0x04
#define FORMAT_FLAGS_RAW          0x08
#define FORMAT_FLAGS_CrCb         0x10

#define LCSR0_REOF2  (1 << 15)

/*
 *Image format definition
 */
#define OV2_IMAGE_FORMAT_RAW                0x0
#define OV2_IMAGE_FORMAT_YCBCR422_PLANAR    0x03
#define OV2_IMAGE_FORMAT_YCBCR444_PLANAR    0x02
#define OV2_IMAGE_FORMAT_YCBCR420_PLANAR    0x04

#define OV2_IMAGE_FORMAT_MAX	\
		 OV2_IMAGE_FORMAT_YCBCR420_PLANAR  

#define PXA2XX_V4L2_OV2_VERSION KERNEL_VERSION(0,0,1)

struct pxa2xx_v4l2ov2_dev *v4l2ov2_dev = NULL;
static spinlock_t free_list_lock;	/* Spin lock for free_list */
static spinlock_t v4l2ov2_list_lock;  /* Spin lock for v4l2ov2_list */

static int v4l2ov2_enable(struct pxa2xx_v4l2ov2_dev *dev);
static int v4l2ov2_disable(struct pxa2xx_v4l2ov2_dev *dev);

static int v4l2ov2_submit(struct pxa2xx_v4l2ov2_dev *dev);


#define CLEAR_LCD_INTR(reg, intr) do {  \
	reg = (intr);			\
}while(0)

#define WAIT_FOR_LCD_INTR(reg,intr,timeout) ({	\
	int __done =0;				\
	int __t = timeout;			\
	while (__t) {				\
		__done = (reg) & (intr);	\
		if (__done) {			\
			(reg) = (intr);		\
			break;			\
		}				\
		mdelay(10);			\
		__t--;				\
	}					\
	if (!__t) pr_debug("wait " #intr " timeount");\
	__done;					\
})


/* format list */
static struct pxa2xx_v4l2ov2_format pxa2xx_v4l2ov2_formats[] = {
	{
		.name     = "4:2:2, planar, Y-Cb-Cr",
		.fourcc   = V4L2_PIX_FMT_YUV422P,
		.depth    = 16,
		.flags    = FORMAT_FLAGS_PLANAR,
	},
	{
		.name     = "4:2:0, planar, Y-Cb-Cr",
		.fourcc   = V4L2_PIX_FMT_YUV420,
		.depth    = 16,
		.flags    = FORMAT_FLAGS_PLANAR,
	}
};

static const unsigned int PXA2XX_V4L2_OV2_FORMATS = ARRAY_SIZE(pxa2xx_v4l2ov2_formats);

static unsigned int v4l2ov2_format_from_v4l2(__u32 fourcc)
{
	int format;
	
	switch (fourcc) {
	case V4L2_PIX_FMT_YUV422P:
		format = OV2_IMAGE_FORMAT_YCBCR422_PLANAR; 
		break;
	case V4L2_PIX_FMT_YUV420:
		format = OV2_IMAGE_FORMAT_YCBCR420_PLANAR;
		break;
	default:
		format = OV2_IMAGE_FORMAT_RAW;
		break;			
	}

	return format;
}


static int v4l2ov2_create_dma(struct pxa2xx_v4l2ov2_dev *dev)
{
	if(dev->format == 0) {
		/* setup dma descriptor */
		dev->dma2 = (struct pxa2xx_lcd_dma_descriptor*)
			(dev->map_cpu + PAGE_SIZE - sizeof(struct pxa2xx_lcd_dma_descriptor));
		dev->dma2->fdadr = (dev->map_dma + PAGE_SIZE -
			       	sizeof(struct pxa2xx_lcd_dma_descriptor));
		dev->dma2->fsadr = dev->map_dma + PAGE_SIZE;
		dev->dma2->fidr  = 0;
		dev->dma2->ldcmd = 0; 
		dev->basefb->fdadr2 = dev->dma2->fdadr;		
	}else {
		/* setup dma for Planar format */
		pr_debug("dev->map_dma = 0x%08x,dev->map_cpu = 0x%p\n",
				dev->map_dma, dev->map_cpu);
		dev->dma2 = (struct pxa2xx_lcd_dma_descriptor*)(dev->map_cpu + 
			PAGE_SIZE - sizeof(struct pxa2xx_lcd_dma_descriptor));
		dev->dma3 = dev->dma2 - 1;
		dev->dma4 = dev->dma3 - 1;
		
		/* Y vector */
		dev->dma2->fdadr = (dev->map_dma + PAGE_SIZE - 
				sizeof(struct pxa2xx_lcd_dma_descriptor));
		dev->dma2->fsadr = dev->map_dma + PAGE_SIZE + dev->yoff;
		dev->dma2->fidr  = 0;
		dev->dma2->ldcmd = dev->ylen | 0x200000;

		/* Cb vector */
		dev->dma3->fdadr = (dev->dma2->fdadr - sizeof(struct pxa2xx_lcd_dma_descriptor));
		dev->dma3->fsadr = (dev->map_dma + PAGE_SIZE + dev->cboff);
		dev->dma3->fidr  = 0;
		dev->dma3->ldcmd = dev->cblen;

		/* Cr vector */
		dev->dma4->fdadr = (dev->dma3->fdadr - sizeof(struct pxa2xx_lcd_dma_descriptor));
		dev->dma4->fsadr = (dev->map_dma + PAGE_SIZE + dev->croff);
		dev->dma4->fidr  = 0;
		dev->dma4->ldcmd = dev->crlen;
		pr_debug("dev->dma2 = 0x%p,dev->dma3 = 0x%p,dev->dma4 = 0x%p\n",
				dev->dma2, dev->dma3, dev->dma4);
		pr_debug("dev->yoff= %d,dev->cboff= %d,dev->croff= %d,",
				dev->yoff ,dev->cboff, dev->croff);
		pr_debug("dev->ylen= %d,dev->cblen= %d,dev->crlen= %d,",
				dev->ylen, dev->cblen, dev->crlen);

		/* make baseplane know the change */
		dev->basefb->fdadr2 = dev->dma2->fdadr;
		dev->basefb->fdadr3 = dev->dma3->fdadr;
		dev->basefb->fdadr4 = dev->dma4->fdadr;		
	}

	dev->dma_changed = 1;
	
	return 0;
}

static int v4l2ov2_map_yuv_memory(struct pxa2xx_v4l2ov2_dev *dev,unsigned int buffer_index)
{
	unsigned int aylen, acblen, acrlen;
	unsigned int aheight, awidth; 
	unsigned int nbytes;

	aylen = acblen = acrlen = 0;
	aheight = dev->fbuf.fmt.height;
	pr_debug("dev->format=0x%d\n",dev->format);
	
	switch(dev->format) {
	case 0x4: /* YCbCr 4:2:0 planar */
		pr_debug("420 planar\n");
		/* 16 pixels per line */
		awidth = ALIGN(dev->fbuf.fmt.width,16);	
		dev->line_length = awidth;
		nbytes = awidth * aheight;
		dev->ylen = nbytes;
		dev->cblen = dev->crlen = (nbytes/4);
		break;
	case 0x3: /* YCbCr 4:2:2 planar */
		/* 8 pixles per line */
		pr_debug("422 planar\n");
		awidth = (dev->fbuf.fmt.width + 0x7) & (~0x7);
		dev->line_length = awidth;
		nbytes = awidth * aheight;
		dev->ylen  = nbytes;
		dev->cblen = dev->crlen = (nbytes/2);
		break;
	case 0x2: /* YCbCr 4:4:4 planar */
		/* 4 pixels per line */
		pr_debug("444 planar\n");
		awidth = ALIGN(dev->fbuf.fmt.width,4); 
		dev->line_length = awidth;
		nbytes = awidth * aheight;
		dev->ylen  = dev->cblen = dev->crlen = nbytes;
		break;
	}

	/* 16-bytes alignment for DMA */
	aylen  = (dev->ylen + 0xf) & (~0xf);
	acblen = (dev->cblen + 0xf) & (~0xf);
	acrlen = (dev->crlen + 0xf) & (~0xf);

	/* offset */
	dev->yoff = 0;
	dev->cboff = aylen;
	dev->croff = dev->cboff + acblen;
	dev->image_size = aylen + acblen + acrlen;	
	pr_debug("dev->yoff = %d,dev->cboff = %d,dev->croff = %d\n",
			dev->yoff, dev->cboff, dev->croff);
	
	if(dev->image_size > dev->queue->bufs[buffer_index].map_size){		
		printk(KERN_ERR "The image size is bigger than buffer size\n");
		return -EINVAL;
	}

	dev->map_cpu = dev->queue->bufs[buffer_index].map_cpu;
	dev->smem_start = dev->queue->bufs[buffer_index].map_dma + PAGE_SIZE;
	dev->map_dma = dev->queue->bufs[buffer_index].map_dma;

	v4l2ov2_create_dma(dev);

	return 0;
};

int v4l2ov2_map_rgb_memory(struct pxa2xx_v4l2ov2_dev *dev,unsigned int buffer_index)
{
	return 0;
}

static int v4l2ov2_prepare_bufs(struct pxa2xx_v4l2ov2_dev *dev, int buf_num)
{
	unsigned int i;
	unsigned int width,height,size;
	unsigned int bpp;	
	struct pxa2xx_v4l2ov2_queue *queue;
	struct pxafb_info * fbi = dev->basefb;
	
	bpp = dev->ovfmt->depth;
	width = dev->fbuf.fmt.width;
	height = dev->fbuf.fmt.height;
	size = width * height * bpp >> 3;
	dev->map_size = PAGE_ALIGN(size + PAGE_SIZE);
	dev->smem_len = dev->map_size * buf_num;  	
	dev->bpp = dev->ovfmt->depth;
	queue = dev->queue;

	if(dev->map_size * buf_num > fbi->ov2_map_size){
		printk(KERN_ERR "V4L2 overlay2 can not malloc memory\n");
		return -ENOMEM;	
	}
		
	
	for(i = 0; i < buf_num; i++){
		queue->bufs[i].map_size = dev->map_size;
		queue->bufs[i].buf_index = i;				
		queue->bufs[i].map_cpu = fbi->ov2_map_cpu + i * dev->map_size;
		queue->bufs[i].map_dma = fbi->ov2_map_dma + i * dev->map_size;
		if ((!queue->bufs[i].map_cpu) || (!queue->bufs[i].map_dma))
			goto failed;

		list_add_tail(&queue->bufs[i].head, &dev->queue->free_queue->free_head);
		pr_debug("&queue->bufs[%d].head = 0x%p\n", i, &queue->bufs[i].head);
		queue->bufs[i].boff = queue->bufs[i].map_size * i;
	}
	
	return 0;

failed:
	pr_debug("in v4l2ov2_prepare_bufs,failed\n");	
	return -ENOMEM;	
}

static struct pxa2xx_v4l2ov2_format *format_by_fourcc(int fourcc)
{
	unsigned int i;

	for (i = 0; i < PXA2XX_V4L2_OV2_FORMATS; i++) {
		if (pxa2xx_v4l2ov2_formats[i].fourcc == fourcc)
			return pxa2xx_v4l2ov2_formats + i;
	}	
	return NULL;
}

static int v4l2ov2_do_dma(struct pxa2xx_v4l2ov2_dev *dev)
{
	int error;
	int buffer_index;
	struct buffer_node *buf_node;	
	unsigned long flags;
	struct free_queue *free_queue = dev->queue->free_queue;
	
	if (dev->queue->last_buffer) {
		if(!list_empty(&(dev->queue->overlay2_head))){
			/*if no buffer display, just wait to prevent conflict */
		
			spin_lock_irqsave(&v4l2ov2_list_lock, flags);
			list_add_tail(&dev->queue->last_buffer->head, &dev->queue->free_queue->free_head);
			dev->queue->last_buffer = NULL;
			spin_unlock_irqrestore(&v4l2ov2_list_lock, flags);

			if (dev->task_waiting) {
				wake_up_interruptible(&(free_queue->done));
				dev->task_waiting = 0;
			}	
		}
	}

	spin_lock_irqsave(&v4l2ov2_list_lock, flags);	
	if(!list_empty(&(dev->queue->overlay2_head))){	
		buf_node = list_entry(dev->queue->overlay2_head.next,
				struct buffer_node, head);
		list_del(&buf_node->head);

		spin_unlock_irqrestore(&v4l2ov2_list_lock, flags);

		buffer_index = buf_node->buf_index;
		dev->queue->last_buffer = buf_node;

		if(dev->format == 0)
			error = v4l2ov2_map_rgb_memory(dev, buffer_index);
		else
			error = v4l2ov2_map_yuv_memory(dev, buffer_index);
		if(error) 
			return error;		
		v4l2ov2_submit(dev);

  	}else{
		
		spin_unlock_irqrestore(&v4l2ov2_list_lock, flags);		
		/* non-smart pannel */
		if (!(dev->basefb->flags & PXAFB_SMART_PANEL)) {
			/*  if overlaye queue is empty, mask EOF irq */
			pr_debug("overlay2_head is empty\n");
			LCCR5 |= LCCR5_EOFM2;
		}			
	}
	return 0;
}

static int v4l2ov2_do_dma_eof(struct pxa2xx_v4l2ov2_dev *dev)
{
	int error;
	int buffer_index;
	struct buffer_node *buf_node;
	
	if(!list_empty(&(dev->queue->overlay2_head))){
		buf_node = list_entry(dev->queue->overlay2_head.next,
			struct buffer_node, head);
		buffer_index = buf_node->buf_index;
		
		if(dev->format == 0)
			error = v4l2ov2_map_rgb_memory(dev, buffer_index);
		else
			error = v4l2ov2_map_yuv_memory(dev, buffer_index);		
		if(error) 
			return error;	
		
		v4l2ov2_submit(dev);
	}
	return 0;
}

static int v4l2ov2_do_dma_bra(struct pxa2xx_v4l2ov2_dev *dev)
{
	int buffer_index;
	struct buffer_node *buf_node;	
	unsigned long flags;
	struct free_queue *free_queue = dev->queue->free_queue;
	
	if (dev->queue->last_buffer) {
		if(!list_empty(&(dev->queue->overlay2_head))){
			/*if no buffer display, just wait to prevent conflict */
		
			spin_lock_irqsave(&free_list_lock, flags);
			list_add_tail(&dev->queue->last_buffer->head, &dev->queue->free_queue->free_head);
			dev->queue->last_buffer = NULL;
			spin_unlock_irqrestore(&free_list_lock, flags);
			
			spin_lock_irqsave(&v4l2ov2_list_lock, flags);
			buf_node = list_entry(dev->queue->overlay2_head.next,
			struct buffer_node, head);
					list_del(&buf_node->head);
			spin_unlock_irqrestore(&v4l2ov2_list_lock, flags);
			buffer_index = buf_node->buf_index;
			dev->queue->last_buffer = buf_node;			

			if (dev->task_waiting) {
				wake_up_interruptible(&(free_queue->done));
				dev->task_waiting = 0;
			}	
		}else{
			/* non-smart pannel */
			if (!(dev->basefb->flags & PXAFB_SMART_PANEL)) {
				/*  if overlaye queue is empty, mask EOF  & BSM2 irq */
				pr_debug("overlay2_head is empty\n");
				LCCR5 |= LCCR5_EOFM2 | LCCR5_BSM2;
			}
		}
	}
	return 0;
}

void v4l2ov2_handle_irq_eof(void)
{
	struct pxa2xx_v4l2ov2_dev *dev = v4l2ov2_dev;
	
	if (!(dev->basefb->flags & PXAFB_SMART_PANEL))
		v4l2ov2_do_dma_eof(dev);
	else
		v4l2ov2_do_dma(dev);
	
}

void v4l2ov2_handle_irq_bra(void)
{
	struct pxa2xx_v4l2ov2_dev *dev = v4l2ov2_dev;

	v4l2ov2_do_dma_bra(dev);
}


static int v4l2ov2_do_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, void *arg)
{		
	int retval = 0;
	unsigned long flags;
	struct pxa2xx_v4l2ov2_dev *dev = file->private_data;
	struct pxa2xx_v4l2ov2_queue *queue = dev->queue;
	
	switch (cmd) {		
	case VIDIOC_QUERYCAP:
	{
		struct v4l2_capability *cap = arg;
		pr_debug("IOCTL CMD = VIDIOC_QUERYCAP\n");
		memset(cap, 0, sizeof(*cap));
		strcpy(cap->driver, "pxa v4l2 overlay2");
		strcpy(cap->card, "");
		cap->version = PXA2XX_V4L2_OV2_VERSION;
		cap->capabilities = V4L2_CAP_VIDEO_OVERLAY | V4L2_CAP_STREAMING;		
		retval = 0;
		break;	
	}
	case VIDIOC_ENUM_FMT:
	{
		struct v4l2_fmtdesc *f = arg;
		enum v4l2_buf_type type;
		int index;
		pr_debug("IOCTL CMD = VIDIOC_ENUM_FMT\n");
		if (f->type != V4L2_BUF_TYPE_VIDEO_OVERLAY) {				
			retval = -EINVAL;
			break;
		}
		if (f->index >= PXA2XX_V4L2_OV2_FORMATS) {
			retval = -EINVAL;
			break;
		}
		type = f->type;
		index = f->index;
		memset(f, 0, sizeof(*f));
		f->index = index;
		f->type = type;
		f->pixelformat = pxa2xx_v4l2ov2_formats[index].fourcc;
		strlcpy(f->description,
				pxa2xx_v4l2ov2_formats[index].name,
				sizeof(f->description));
		break;	
	}
	case VIDIOC_S_FMT:
	{
		struct v4l2_format *f = arg;
		pr_debug("IOCTL CMD = VIDIOC_S_FMT\n");
		if (f->type != V4L2_BUF_TYPE_VIDEO_OVERLAY) {
			retval = -EINVAL;
			break;
		}
		dev->win.w.height = f->fmt.win.w.height;
		dev->win.w.width = f->fmt.win.w.width;
		dev->win.w.left = f->fmt.win.w.left;
		dev->win.w.top = f->fmt.win.w.top;
		retval = 0;
		break;
	}
	case VIDIOC_G_FMT:
	{
		struct v4l2_format *f = arg;
		pr_debug("IOCTL CMD = VIDIOC_G_FMT\n");
		if(f->type != V4L2_BUF_TYPE_VIDEO_OVERLAY){
			retval = -EINVAL;
			break;
		}
		f->fmt.win.w.height=  dev->win.w.height;
		f->fmt.win.w.width =  dev->win.w.width;
		f->fmt.win.w.left =  dev->win.w.left;
		f->fmt.win.w.top =  dev->win.w.top;
		retval = 0;
		break;
	}
	case VIDIOC_S_FBUF:
	{
		struct v4l2_framebuffer *fb = arg;
		struct pxa2xx_v4l2ov2_format *fmt;
		pr_debug("IOCTL CMD = VIDIOC_S_FBUF\n");
		if(fb->flags != V4L2_FBUF_FLAG_OVERLAY){
			retval = -EINVAL;
			break;
		}
		pr_debug("fb->fmt.pixelformat = %d\n",fb->fmt.pixelformat);
		fmt = format_by_fourcc(fb->fmt.pixelformat);
		if (fmt == NULL)
			return -EINVAL;
		/*  ok, accept it    */
		dev->format = v4l2ov2_format_from_v4l2(fmt->fourcc);
		pr_debug("dev->format = %d\n",dev->format);
		dev->fbuf.base       = fb->base;
		dev->fbuf.fmt.width  = fb->fmt.width;
		dev->fbuf.fmt.height = fb->fmt.height;
		dev->ovfmt = fmt;
		if (0 != fb->fmt.bytesperline)
			dev->fbuf.fmt.bytesperline = fb->fmt.bytesperline;
		else
			dev->fbuf.fmt.bytesperline = dev->fbuf.fmt.width*fmt->depth/8;
		break;
	}
	case VIDIOC_G_FBUF:
	{
		struct v4l2_framebuffer *fb = arg;
		pr_debug("IOCTL CMD = VIDIOC_S_FBUF\n");
		*fb = dev->fbuf;
		fb->capability = V4L2_FBUF_FLAG_OVERLAY;
		if (dev->ovfmt)
			fb->fmt.pixelformat  = dev->ovfmt->fourcc;
		retval = 0;
		break;	
	}
	case VIDIOC_REQBUFS:
	{
		struct v4l2_requestbuffers *req = arg;
		pr_debug("IOCTL CMD = VIDIOC_REQBUFS\n");
		if (req->type != V4L2_BUF_TYPE_VIDEO_OVERLAY) {
			retval = -EINVAL;
			break;
		}
		if (req->memory != V4L2_MEMORY_MMAP){
			retval = -EINVAL;
			break;
		}
		if((req->count < 0) || (req->count > MAX_NR_PXA2XX_V4L2OV2_BUF)){
			retval = -EINVAL;
			break;
		}
		retval = v4l2ov2_prepare_bufs(dev, req->count);
		break;
	}
	case VIDIOC_QUERYBUF:
	{
		struct v4l2_buffer *buf = arg;
		pr_debug("IOCTL CMD = VIDIOC_QUERYBUF\n");
		if (buf->type != V4L2_BUF_TYPE_VIDEO_OVERLAY) {
			retval = -EINVAL;
			break;
		}
		if (buf->memory != V4L2_MEMORY_MMAP) {
			retval = -EINVAL;
			break;
		}
		if(buf->index < 0 || buf->index > MAX_NR_PXA2XX_V4L2OV2_BUF){
			retval = -EINVAL;
			break;	
		}
		if(queue->bufs[buf->index].map_size == 0){
			retval = -EINVAL;
			break;	
		}
		buf->length =  queue->bufs[buf->index].map_size;
		buf->m.offset = queue->bufs[buf->index].boff;
		break;
	}
	case VIDIOC_QBUF:
	{
		struct v4l2_buffer *buf = arg;
		struct buffer_node *buffer;
		unsigned int lccr5 = LCCR5;
		pr_debug("IOCTL CMD = VIDIOC_QBUF\n");
		if (buf->type != V4L2_BUF_TYPE_VIDEO_OVERLAY) {
			retval = -EINVAL;
			break;
		}
		if (buf->memory != V4L2_MEMORY_MMAP){
			retval = -EINVAL;
			break;
		}
		if(buf->index < 0 || buf->index > MAX_NR_PXA2XX_V4L2OV2_BUF){
			retval = -EINVAL;
			break;	
		}
		buffer = &queue->bufs[buf->index];
		pr_debug("buf->index = %d\n",buf->index);
		pr_debug("buffer->buf_index= %d\n",buffer->buf_index);
		if(buffer == NULL){
			retval = -EINVAL;
			break;
		}
		if ((dev->driver_state == C_ENABLE) && (dev->device_state == C_DISABLE)) {
			spin_lock_irqsave(&v4l2ov2_list_lock, flags);
			list_add_tail(&buffer->head, &(queue->free_queue->free_head));
			spin_unlock_irqrestore(&v4l2ov2_list_lock, flags);
			retval = 0;
			break;
		}

		spin_lock_irqsave(&v4l2ov2_list_lock, flags);
		list_add_tail(&buffer->head,&(queue->overlay2_head));			
		spin_unlock_irqrestore(&v4l2ov2_list_lock, flags);
		pr_debug("&buffer->head= 0x%p,queue->overlay2_head.prev = 0x%p\n",
				&buffer->head,queue->overlay2_head.prev);
		
		/* non-smart pannel */
		if (!(dev->basefb->flags & PXAFB_SMART_PANEL)) {
			/* If eof irq is mask, unmask it */
			if( lccr5 & LCCR5_EOFM2)
				LCCR5 &= (~LCCR5_EOFM2);	
		}
		retval = 0;
		break;
	}
	case VIDIOC_DQBUF:
	{
		struct v4l2_buffer *buf = arg;	
		struct buffer_node *buffer_node;
		pr_debug("IOCTL CMD = VIDIOC_DQBUF\n");
		if (buf->type != V4L2_BUF_TYPE_VIDEO_OVERLAY) {
			retval = -EINVAL;
			break;
		}
		if(buf->memory != V4L2_MEMORY_MMAP){
			retval = -EINVAL;
			break;
		}

		spin_lock_irqsave(&free_list_lock, flags);
		if(!list_empty(&dev->queue->free_queue->free_head)){
			buffer_node = list_entry(dev->queue->free_queue->free_head.next,
				struct buffer_node, head);
			list_del(&buffer_node->head);
			buf->index = buffer_node->buf_index;			
		}else
			retval = -EINVAL;
		
		spin_unlock_irqrestore(&free_list_lock, flags);
		
		break;
	}
	case VIDIOC_OVERLAY:
	{
		int *on = arg;
		pr_debug("IOCTL CMD = VIDIOC_OVERLAY\n");
		if (*on) {
			spin_lock_irqsave(&v4l2ov2_list_lock, flags);
			if(list_empty(&(dev->queue->overlay2_head))){
				printk(KERN_ERR "VIDIOC_OVERLAY: no buffers!\n");
				spin_unlock_irqrestore(&v4l2ov2_list_lock, flags);
				return -EIO;
			}
			/*first v4l2ov2_enable*/
			down(&dev->basefb->ctrlr_sem);
			retval = v4l2ov2_enable(dev);
			up(&dev->basefb->ctrlr_sem);

			spin_unlock_irqrestore(&v4l2ov2_list_lock, flags);
			/*Enable the first DMA*/
			down(&dev->basefb->ctrlr_sem);
			retval = v4l2ov2_do_dma(dev);
			up(&dev->basefb->ctrlr_sem);
			dev->driver_state = C_ENABLE ;
		} else {
			down(&dev->basefb->ctrlr_sem);
			retval = v4l2ov2_disable(dev);
			up(&dev->basefb->ctrlr_sem);
			dev->driver_state = C_DISABLE ;
		}
		break;
	}
	default:
		pr_debug("Invalid ioctl parameters.\n");
		retval = -ENOIOCTLCMD;
		break;
	}
	return retval;
}

static int v4l2ov2_enable(struct pxa2xx_v4l2ov2_dev *dev) 
{
	/* disable branch/start/end of frame interrupt */
	/* not smart pannel */
	if (dev->basefb->flags & PXAFB_SMART_PANEL) {
		LCCR5 |= (LCCR5_IUM4 | LCCR5_IUM3 | LCCR5_IUM2 |
			  LCCR5_BSM4 | LCCR5_BSM3 | LCCR5_BSM2 |
			  LCCR5_EOFM4 | LCCR5_EOFM3 | LCCR5_EOFM2 |
			  LCCR5_SOFM4 | LCCR5_SOFM3 | LCCR5_SOFM2);
	} else {
		LCCR5 |= (LCCR5_IUM4 | LCCR5_IUM3 | LCCR5_IUM2 |
			  LCCR5_BSM4 | LCCR5_BSM3 | /*LCCR5_BSM2 |*/
			  LCCR5_EOFM4 | LCCR5_EOFM3 | /*LCCR5_EOFM2 |*/
			  LCCR5_SOFM4 | LCCR5_SOFM3 | LCCR5_SOFM2);
	}

	dev->device_state = C_ENABLE;
	return 0;

}

static int v4l2ov2_submit(struct pxa2xx_v4l2ov2_dev *dev)
{
	unsigned int bpp2;
	unsigned int xres, yres, xpos, ypos,screen_width,screen_height;
	
	if (!dev->dma_changed)
	       	return 0;

	pr_debug("dev->bpp = %d,dev->format =%d,dev->ovfmt->depth= %d\n",dev->bpp,
		dev->format, dev->ovfmt->depth);
	
	xres = dev->fbuf.fmt.width;
	yres = dev->fbuf.fmt.height;
	xpos = dev->win.w.left;
	ypos = dev->win.w.top;
	screen_width = dev->win.w.width;
	screen_height = dev->win.w.height;
	if((screen_width < xres) || (screen_height < yres)) {
		printk(KERN_ERR "frame buffer size larger than overlay2 window!\n");
		return -EINVAL;
	}
	
	pr_debug("dev->ovfmt->depth = %d\n",dev->ovfmt->depth);
	switch(dev->ovfmt->depth) {
	case 16:
		bpp2 = 0x4;
		break;
	case 18:
		bpp2 = 0x6;
		break;
	case 19:
		bpp2 = 0x8;
		break;
	case 24:
		bpp2 = 0x9;
		break;
	case 25:
		bpp2 = 0xa;
		break;
	default:
		printk(KERN_ERR "bpp %d not supported!\n", dev->ovfmt->depth);
		return -EINVAL;
	}

	if (dev->format == 0){
		/*  overlay2 RGB resolution, will implement in next phase */

	} else {
		/* overlay2 YUV resolution */
		
		/* smart pannel */
		if(dev->basefb->flags & PXAFB_SMART_PANEL) {
			dev->basefb->reg_ovl2c2 = (dev->format << 20) | (ypos << 10) | xpos;
			dev->basefb->reg_ovl2c1 = OVL2C1_O2EN | (bpp2 << 20) 
				| ((yres-1)<<10) | (xres-1);	
			dev->basefb->fdadr2 = dev->dma2->fdadr;
			dev->basefb->fdadr3 = dev->dma3->fdadr;
			dev->basefb->fdadr4 = dev->dma4->fdadr;	
			dev->device_state = C_ENABLE;
		} else {
			OVL2C2 = (dev->format << 20) | (ypos << 10) | xpos;
			OVL2C1 = OVL2C1_O2EN | (bpp2 << 20) | ((yres-1)<<10) | (xres-1);
			if (dev->device_state == C_DISABLE) {
				FDADR2 = dev->dma2->fdadr;
				FDADR3 = dev->dma3->fdadr;			
				FDADR4 = dev->dma4->fdadr;
				dev->device_state = C_ENABLE;				
			} else {
				FBR2 = dev->dma2->fdadr | 0x3; 
				FBR3 = dev->dma3->fdadr | 0x1; 
				FBR4 = dev->dma4->fdadr | 0x1; 				
			}
		}
	}

	dev->dma_changed = 0;

	return 0;
}


static int v4l2ov2_disable(struct  pxa2xx_v4l2ov2_dev *dev )
{
	int done;
	int fdadr2,fbr2;
	unsigned long flag1, flag2;
	struct buffer_node *buffer_node;

	if (dev->device_state == C_DISABLE) 
		return 0;

	/* Move buffer node from report list to free list, in case in sleep mode */
	spin_lock_irqsave(&free_list_lock, flag1);
	spin_lock_irqsave(&v4l2ov2_list_lock, flag2);

	while (!list_empty(&(dev->queue->overlay2_head))) {
		buffer_node = list_entry(dev->queue->overlay2_head.next,
				struct buffer_node, head);
		list_del(&buffer_node->head);

		list_add_tail(&buffer_node->head, &(dev->queue->free_queue->free_head));

	}

	spin_unlock_irqrestore(&v4l2ov2_list_lock, flag2);
	spin_unlock_irqrestore(&free_list_lock, flag1);

	if (dev->task_waiting) {
		wake_up_interruptible(&(dev->queue->free_queue->done));
		dev->task_waiting = 0;
	}

	
	if (dev->basefb->flags & PXAFB_SMART_PANEL) {
		dev->device_state = C_DISABLE;
		return 0;
	}

	OVL2C1 &= ~OVL2C1_O2EN;
	pr_debug("enter v4l2ov2_disable\n");

	/* disable branch/start/end of frame interrupt */
	LCCR5 |= (LCCR5_IUM4 | LCCR5_IUM3 | LCCR5_IUM2 | 
		  LCCR5_BSM4 | LCCR5_BSM3 | LCCR5_BSM2 |
		  LCCR5_EOFM4 | LCCR5_EOFM3 | LCCR5_EOFM2 |
		  LCCR5_SOFM4 | LCCR5_SOFM3 | LCCR5_SOFM2);

	CLEAR_LCD_INTR(LCSR1, LCSR1_BS2);	

	if (dev->format == 0) 
		FBR2 = FDADR2 | 0x3;
	else {
		fdadr2 = FDADR2;
		pr_debug("fdadr2 = 0x%x\n",fdadr2);
		FBR2 = FDADR2 | 0x3;
		FBR3 = FDADR3 | 0x3;
		FBR4 = FDADR4 | 0x3;
		fbr2 = FBR2;
		pr_debug("fbr2 = 0x%x\n",fbr2);
		 
	}

	done = WAIT_FOR_LCD_INTR(LCSR1, LCSR1_BS2, 100);	
	LCSR1 &= LCSR1_EOF2;
	if (!done) {
		pr_debug(KERN_INFO "%s: timeout\n", __FUNCTION__);
		return -1;
	}

	dev->device_state = C_DISABLE;
	return 0;
}

void v4l2ov2_set_overlay2_ctrlr_state(struct pxafb_info *fbi, u_int state)
{
	switch (state) {
	case C_DISABLE:
		if (fbi->ov2_v4l2_dev && (fbi->ov2_v4l2_dev->device_state == C_ENABLE)) {
			v4l2ov2_disable((struct pxa2xx_v4l2ov2_dev *)fbi->ov2_v4l2_dev);
		}
		break;
	case C_ENABLE:
		if (fbi->ov2_v4l2_dev && (fbi->ov2_v4l2_dev->device_state == C_DISABLE)){
			v4l2ov2_enable((struct pxa2xx_v4l2ov2_dev *)fbi->ov2_v4l2_dev);
		}					
		break;
	default:
		break;
	}
}

static int is_pxafb_device(struct device * dev, void * data)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);

	return (strncmp(pdev->name, "pxa2xx-fb", 9) == 0);
}

static int pxa2xx_v4l2ov2_open(struct inode *inode, struct file *file)
{
	unsigned long flags;
	struct pxa2xx_v4l2ov2_dev *dev = v4l2ov2_dev;

	down(&dev->basefb->ctrlr_sem);
	if (dev->basefb->ov2_v4l2_dev && (dev->basefb->ov2_v4l2_dev->device_state == C_ENABLE)) {
		printk(KERN_ERR "overlay2 is being used by frame buffer driver!\n");
		up(&dev->basefb->ctrlr_sem);
		return -EBUSY;
	}
	dev->basefb->ov2_v4l2_dev  = v4l2ov2_dev;
	dev->basefb->set_overlay2_ctrlr_state = v4l2ov2_set_overlay2_ctrlr_state;
	dev->basefb->ov2_handle_eof = v4l2ov2_handle_irq_eof;
	dev->basefb->ov2_handle_bra = v4l2ov2_handle_irq_bra;
	dev->dma_changed = 0;
	up(&dev->basefb->ctrlr_sem);

	/* if basefb is disable, enable fb. */
	if (dev->basefb && dev->basefb->state != C_ENABLE)
		dev->basefb->fb.fbops->fb_blank(VESA_NO_BLANKING, 
				(struct fb_info *)(dev->basefb));
	
	pr_debug("enter pxa2xx_v4l2ov2_open\n");
	spin_lock_init(&free_list_lock);
	spin_lock_init(&v4l2ov2_list_lock);

	file->private_data = dev;
	dev->type = V4L2_BUF_TYPE_VIDEO_OVERLAY;
	dev->device_state = C_DISABLE;
	dev->driver_state = C_DISABLE;
	
	/* window QVGA size */
	dev->win.w.left = 0;
	dev->win.w.top = 0;
	dev->win.w.width = 240;
	dev->win.w.height = 320;
	
	/* frame buffer size */
	dev->fbuf.fmt.width = 240;
	dev->fbuf.fmt.height = 320;
	/* pixel format  ycbcr422 */
	dev->fbuf.fmt.pixelformat = 0x3;	
	
	INIT_LIST_HEAD(&(dev->queue->overlay2_head));
	INIT_LIST_HEAD(&(dev->queue->free_queue->free_head));
	dev->queue->last_buffer = NULL;
	
	init_waitqueue_head(&dev->queue->free_queue->done);
	dev->task_waiting = 0;
	 
	/* empty the overlay2 list */
	 spin_lock_irqsave(&v4l2ov2_list_lock, flags);	 
	while (!list_empty(&(dev->queue->overlay2_head))) 
		list_del_init(dev->queue->overlay2_head.next);
 
	spin_unlock_irqrestore(&v4l2ov2_list_lock, flags);
	
	/* empty the free  list */
	spin_lock_irqsave(&free_list_lock, flags);
	while (!list_empty(&(dev->queue->free_queue->free_head))) {
		pr_debug("no empty item in report head list \n");
		list_del_init(dev->queue->free_queue->free_head.next);
	}
	spin_unlock_irqrestore(&free_list_lock, flags);

	return 0;
}

static int pxa2xx_v4l2ov2_close(struct inode *inode, struct file *file)
{
	struct pxa2xx_v4l2ov2_dev *dev = file->private_data;
	struct buffer_node *buffer_node;
	
	/* if overlay2 is enable, disable overlay2 */
	if(dev->device_state == C_ENABLE)		
		v4l2ov2_disable(dev);		

	/* empty the report list */
	while (!list_empty(&(dev->queue->overlay2_head))){		 
		buffer_node = list_entry(dev->queue->overlay2_head.next, 
				struct buffer_node, head);
		list_del_init(dev->queue->overlay2_head.next);
	}

	/* empty the free list */
	while (!list_empty(&(dev->queue->free_queue->free_head))){ 
		buffer_node = list_entry(dev->queue->free_queue->free_head.next,
			       	struct buffer_node, head);
		list_del_init(dev->queue->free_queue->free_head.next);		  
	}

	down(&dev->basefb->ctrlr_sem);
	dev->basefb->ov2_v4l2_dev  = NULL;
	dev->basefb->set_overlay2_ctrlr_state = NULL;
	dev->basefb->ov2_handle_eof = NULL;	
	dev->basefb->ov2_handle_bra = NULL;
	up(&dev->basefb->ctrlr_sem);

	return 0;		
}


static int pxa2xx_v4l2ov2_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct pxa2xx_v4l2ov2_dev *dev = file->private_data;	
	unsigned long off;
	int buffer_index;
	unsigned long start,len;
	
	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
		return -EINVAL;
	
	off = vma->vm_pgoff << PAGE_SHIFT;
	pr_debug("off = %08x,vma->vm_pgoff = %08x\n", 
		(unsigned int)off, (unsigned int)vma->vm_pgoff);
	
	buffer_index = off/dev->map_size;
	start = dev->queue->bufs[buffer_index].map_dma + PAGE_SIZE;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + dev->map_size);
	start &= PAGE_MASK;

	pr_debug("vma->vm_end = %08x, vma->vm_start = %08x, off = %08x\n",
			(unsigned int)vma->vm_end, (unsigned int)vma->vm_start, 
			(unsigned int)off);	
	off = 0;
	
	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;

	off += start;
	vma->vm_pgoff = off >> PAGE_SHIFT;

	/* This is an IO map - tell maydump to skip this VMA */
	vma->vm_flags |= VM_IO | VM_RESERVED;
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);	
	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
				vma->vm_end - vma->vm_start, vma->vm_page_prot))		
		return -EAGAIN;

	return 0;
}

static unsigned int pxa2xx_v4l2ov2_poll(struct file *file, poll_table * wait)
{
	unsigned long flags;
	struct pxa2xx_v4l2ov2_dev *dev = file->private_data;	

	spin_lock_irqsave(&free_list_lock, flags);
	if(!list_empty(&dev->queue->free_queue->free_head)){
		spin_unlock_irqrestore(&free_list_lock, flags);
		return POLLOUT | POLLWRNORM;
	}
	dev->task_waiting = 1;
	spin_unlock_irqrestore(&free_list_lock, flags);
	
	poll_wait(file, &(dev->queue->free_queue->done), wait);

	spin_lock_irqsave(&free_list_lock, flags);
	if(!list_empty(&dev->queue->free_queue->free_head)){
		spin_unlock_irqrestore(&free_list_lock, flags);
		return POLLOUT | POLLWRNORM;
	}else {
		spin_unlock_irqrestore(&free_list_lock, flags);
		return 0;
	}
	
}


static int pxa2xx_v4l2ov2_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long param)
{
	return video_usercopy(inode, file, cmd, param, v4l2ov2_do_ioctl);
}

static void pxa2xx_v4l2ov2_release(struct video_device *dev)
{
	printk(KERN_WARNING "pxa2xx_v4l2ov2:  has no release callback. "
			"Please fix your driver for proper sysfs support, see "
			"http://lwn.net/Articles/36850/\n");
	return;
}

static struct file_operations pxa2xx_v4l2ov2_fops = {
	.owner 		= THIS_MODULE,
	.open 		= pxa2xx_v4l2ov2_open,
	.release 	= pxa2xx_v4l2ov2_close,
	.ioctl 		= pxa2xx_v4l2ov2_ioctl,
	.mmap 		= pxa2xx_v4l2ov2_mmap,
	.poll 		= pxa2xx_v4l2ov2_poll,
	.llseek 	= no_llseek,
};

static struct video_device pxa2xx_v4l2ov2_vd = {
	.name 		= "pxa2xx v4l2 overlay2",
	.fops 		= &pxa2xx_v4l2ov2_fops,
	.release 	= pxa2xx_v4l2ov2_release,
	.minor 		= -1,
};

static int pxa2xx_v4l2ov2_probe(struct platform_device *pdev)
{
	struct pxafb_info *fbi;
	struct device *dev;
	
	dev = bus_find_device(&platform_bus_type, NULL, NULL, is_pxafb_device);
	if(dev == NULL){	
		printk(KERN_ERR "pxa2xx_v4l2ov2: failed to get base framebuffer!\n");
		return -EIO;
	}
	fbi = dev_get_drvdata(dev);
	
	v4l2ov2_dev = kzalloc(sizeof(*v4l2ov2_dev),GFP_KERNEL);
	if (NULL == v4l2ov2_dev)
		return -ENOMEM;

	v4l2ov2_dev->queue = kzalloc(sizeof(struct pxa2xx_v4l2ov2_queue),GFP_KERNEL);	
	if (NULL == v4l2ov2_dev->queue){
		goto malloc_v4l2ov2_queue_err;
	}

	v4l2ov2_dev->queue->free_queue = kzalloc(sizeof(struct free_queue),GFP_KERNEL);
	if (NULL == v4l2ov2_dev->queue->free_queue){
		goto malloc_v4l2ov2_free_queue_err;
	}	
	
	v4l2ov2_dev->dev = &pdev->dev;
	v4l2ov2_dev->basefb = fbi;	
	
	if (video_register_device(&pxa2xx_v4l2ov2_vd, VFL_TYPE_GRABBER,  -1) < 0) 
		printk(KERN_ALERT "pxa2xx_v4l2ov2: video_register_device failed\n");		
	else 
		printk(KERN_ALERT "pxa2xx_v4l2ov2: PXA v4l2 overlay2 driver loaded for"
				"/dev/video%d \n", pxa2xx_v4l2ov2_vd.minor);
	
	return 0;
	
malloc_v4l2ov2_free_queue_err:
        if(v4l2ov2_dev->queue)
		kfree(v4l2ov2_dev->queue);
	
malloc_v4l2ov2_queue_err:
	if(v4l2ov2_dev)
		kfree(v4l2ov2_dev);

	return -EIO;
}

static int pxa2xx_v4l2ov2_remove(struct platform_device *pdev)
{
	video_unregister_device(&pxa2xx_v4l2ov2_vd);
	printk(KERN_ALERT "pxa2xx_v4l2ov2: PXA v4l2 overlay2 driver unloaded.\n");

	return 0;
}

static struct platform_driver pxa2xx_v4l2ov2_driver = {
	.driver	= {
		.name = "pxa2xx-v4l2ov2",
	},
	.probe 		= pxa2xx_v4l2ov2_probe,
	.remove 	= pxa2xx_v4l2ov2_remove,
};

static int __devinit pxa2xx_v4l2ov2_init(void)
{
	return platform_driver_register(&pxa2xx_v4l2ov2_driver);
}

static void __exit pxa2xx_v4l2ov2_exit(void)
{
	platform_driver_unregister(&pxa2xx_v4l2ov2_driver);

}

#ifndef MODULE
late_initcall(pxa2xx_v4l2ov2_init);
#else
module_init(pxa2xx_v4l2ov2_init);
#endif
module_exit(pxa2xx_v4l2ov2_exit);

MODULE_DESCRIPTION("PXA v4l2 overlay2 Interface driver");
MODULE_LICENSE("GPL");
