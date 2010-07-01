/*
 * chip-si4703 - main file for si4703 FM chip driver
 *
 * Copyright (C) 2008, Marvell Corporation.
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
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/i2c/si4703.h>
#include <linux/interrupt.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <asm/irq.h>
#ifdef CONFIG_PXA3xx_DVFM
#include <mach/hardware.h>
#include <mach/dvfm.h>
#include <mach/pxa3xx_dvfm.h>
#endif

/*
 * Version Information
 */
#include <linux/version.h>	/* for KERNEL_VERSION MACRO	*/

#define DRIVER_VERSION "v0.01"
#define RADIO_VERSION KERNEL_VERSION(0, 0, 1)

static struct v4l2_queryctrl chip_qctrl[] = {
	{
		.id            = V4L2_CID_AUDIO_MUTE,
		.name          = "Mute",
		.minimum       = 0,
		.maximum       = 1,
		.default_value = 1,
		.type          = V4L2_CTRL_TYPE_BOOLEAN,
	},
	{
		.id            = V4L2_CID_AUDIO_VOLUME,
		.name          = "Volume",
		.minimum       = 0,
		.maximum       = 16,
		.step          = 1,
		.default_value = 0xf,
		.type          = V4L2_CTRL_TYPE_INTEGER,
	}
};

/* Frequency limits in MHz -- these are European values.  For Japanese
 * devices, that would be 76MHz and 91MHz.
 */
#define REG_DEVICEID	0x00
#define REG_CHIPID	0x01
#define REG_POWERCFG	0x02
#define REG_CHANNEL	0x03
#define REG_SYSCONFIG1	0x04
#define REG_SYSCONFIG2	0x05
#define REG_SYSCONFIG3	0x06
#define REG_TEST1	0x07
#define REG_TEST2	0x08
#define REG_BOOTCFG	0x09
#define REG_RSSI	0x0a
#define REG_READCHAN	0x0b
#define REG_RDSA	0x0c
#define REG_RDSB	0x0d
#define REG_RDSC	0x0e
#define REG_RDSD	0x0f

/* REG_POWERCFG */
#define DMUTE		0x4000
#define MUTE		0x0000
#define MONO		0x2000
#define SKMODE		0x0400
#define SEEKUP		0x0200
#define SEEKDOWN	0x0000
#define SEEK		0x0100
#define DISABLE		0x0040
#define ENABLE		0x0001
#define DSOFTMUTE	0x8000

/* REG_CHANNEL */
#define TUNE		0x8000

/* REG_SYSCONFIG1 */
#define RDSIEN		0x8000
#define STCIEN		0x4000
#define RDS		0x1000
#define GPIO2_INTERRUPT	0x0004

/* REG_SYSCONFIG2 */
#define RECOMMENDED_SEEKTH	0x0c00
#define BAND_EURO		0x0000 /* 87.5-108MHz */
#define WIDEBAND_JAPAN		0x0040 /* 76-108MHZ */
#define BAND_JAPAN		0x0080 /* 76-90MHz */
#define SPACE_200KHZ		0x0000
#define SPACE_100KHZ		0x0010
#define SPACE_50KHZ		0x0020
#define VOLUME			0x000F
#define SPACINGSELECT		0x0010
#define VOLUMECTRL		0x000f

/* REG_SYSCONFIG3 */
#define SEN_THRESHOLD		0x0040
#define DETECTION_THRESHOLD	0x0008

/* REG_RSSI */
#define STC			0x4000
#define RSSI			0x00FF
#define AFCRL			0x1000

#define FREQ_MUL		1600

#ifdef CONFIG_RADIO_FM_EURO
#define BAND			BAND_EURO
#define SPACE			SPACE_100KHZ
#define CHANNEL_SPACE		100 /* in kHZ */
#define FREQ_MIN		87.5
#define FREQ_MAX		108
#endif

#ifdef CONFIG_RADIO_FM_JAPAN_WIDEBAND
#define BAND			WIDEBAND_JAPAN
#define SPACE			SPACE_100KHZ
#define CHANNEL_SPACE		100 /* in kHZ */
#define FREQ_MIN		76
#define FREQ_MAX		108
#endif

#ifdef CONFIG_RADIO_FM_JAPAN
#define BAND			BAND_JAPAN
#define SPACE			SPACE_100KHZ
#define CHANNEL_SPACE		100 /* in kHZ */
#define FREQ_MIN		76
#define FREQ_MAX		90
#endif

#define REG_READ_START REG_RSSI
#define REG_WRITE_START REG_POWERCFG
#define NR_SI4703_REGS 16
struct si4703_device {
	struct i2c_client *client;
	int irq;
	struct video_device *videodev;
	uint16_t regs[NR_SI4703_REGS];
	int curfreq;
	int stereo;
	int users;
	int removed;
	int curvol;
	int muted;
	wait_queue_head_t	wait;
};

extern void enable_oscc_tout_s0(void);
extern void disable_oscc_tout_s0(void);

#ifdef CONFIG_PXA3xx_DVFM
static int dvfm_radio_idx;
static void set_radio_dvfm_constraint(void)
{
        /* Disable Lowpower mode */
        dvfm_disable_op_name("D1", dvfm_radio_idx);
        dvfm_disable_op_name("D2", dvfm_radio_idx);
        if (cpu_is_pxa935())
                dvfm_disable_op_name("CG", dvfm_radio_idx);
}

static void unset_radio_dvfm_constraint(void)
{
        /* Enable Lowpower mode */
        dvfm_enable_op_name("D1", dvfm_radio_idx);
        dvfm_enable_op_name("D2", dvfm_radio_idx);
        if (cpu_is_pxa935())
                dvfm_enable_op_name("CG",dvfm_radio_idx);
}
#else
static void set_radio_dvfm_constraint(void) {}
static void unset_radio_dvfm_constraint(void) {}
#endif
int si4703_write(struct si4703_device *chip, uint8_t reg, uint16_t val)
{
	uint16_t count;
	uint8_t buf[NR_SI4703_REGS*2];
	int i;
	int ret;
	int index;

	chip->regs[reg] = val;

	for (i = 0; i < NR_SI4703_REGS; i++) {
		index = (REG_WRITE_START + i)%NR_SI4703_REGS;
		buf[2*i]     = chip->regs[index] >> 8;
		buf[2*i + 1] = chip->regs[index] & 0xff;
	}
	count = ((reg + NR_SI4703_REGS - REG_WRITE_START)%NR_SI4703_REGS + 1)*2;

	ret = i2c_master_send(chip->client, buf, count);
	if (ret < 0) {
		printk(KERN_ERR "si4703_write: write failed!\n");
		return -EIO;
	}

	return 0;
}

int  si4703_read(struct si4703_device *chip, uint8_t reg, uint16_t *val)
{
	int ret;
	uint16_t count;
	uint8_t buf[NR_SI4703_REGS*2];
	int i;

	count = ((reg + NR_SI4703_REGS - REG_READ_START)%NR_SI4703_REGS + 1)*2;
	ret = i2c_master_recv(chip->client, buf, count);
	if (ret < 0) {
		printk(KERN_ERR "si4703_read: read failed!\n");
		return -EIO;
	} else {
		 for (i = 0; i < count/2; i++)
			 chip->regs[(REG_READ_START + i)%NR_SI4703_REGS] =
				 (buf[2*i] << 8) + (buf[2*i + 1] & 0xff);
		 *val = chip->regs[reg];
	}

	return 0;
}

/* Low-level device interface begins here */
uint16_t si4703_get_rssi(struct si4703_device *chip)
{
	uint16_t val;

	si4703_read(chip, REG_RSSI, &val);

	return val;
}

uint16_t si4703_get_chan(struct si4703_device *chip)
{
	uint16_t val;

	si4703_read(chip, REG_READCHAN, &val);

	return val;
}

int si4703_power_up(struct si4703_device *chip)
{
	si4703_write(chip, REG_POWERCFG, DMUTE | ENABLE);
	si4703_write(chip, REG_CHANNEL, 0x0000);
	si4703_write(chip, REG_SYSCONFIG1, STCIEN |
			RDSIEN | RDS | GPIO2_INTERRUPT);
	si4703_write(chip, REG_SYSCONFIG2, RECOMMENDED_SEEKTH |
			SPACINGSELECT | VOLUMECTRL | SPACE | BAND);
	si4703_write(chip, REG_SYSCONFIG3, SEN_THRESHOLD |
			DETECTION_THRESHOLD);
	mdelay(50);

	return 0;
}

int si4703_power_down(struct si4703_device *chip)
{
	/*  Set DISABLE bit as 1, perform internal power-down sequence and
	 *  set ENABLE bit=0 later by device itself
	 */
	return si4703_write(chip, REG_POWERCFG, ENABLE | DISABLE);
}

/* switch on chip */
static int si4703_unmute(struct si4703_device *chip)
{
	uint16_t val;

	if (si4703_read(chip, REG_SYSCONFIG2, &val))
		return -EIO;

	val &= ~0xf;
	val |= chip->curvol;
	if (si4703_write(chip, REG_SYSCONFIG2, val))
		return -EIO;

	chip->muted = 0;
	return 0;
}

/* switch off chip */
static int si4703_mute(struct si4703_device *chip)
{
	uint16_t val;

	if (si4703_read(chip, REG_SYSCONFIG2, &val))
		return -EIO;

	chip->curvol = val & 0xf;
	val &= ~0xf;
	if (si4703_write(chip, REG_SYSCONFIG2, val))
		return -EIO;

	chip->muted = 1;
	return 0;
}

static int si4703_setvol_generic(struct si4703_device *chip, int vol)
{
	uint16_t val;

	if (si4703_read(chip, REG_SYSCONFIG2, &val))
		return -EIO;

	val &= ~0xf;
	if (si4703_write(chip, REG_SYSCONFIG2, val | vol))
		return -EIO;

	return 0;
}

static int si4703_setvol(struct si4703_device *chip, int vol)
{
	/* user is unmuting the card */
	if (chip->muted && vol != 0) {
		chip->curvol = vol;
		si4703_unmute(chip);
		return 0;
	}

	/* requested volume == current */
	if (vol == chip->curvol)
		return 0;

	/* volume == 0 means mute the card */
	if (vol == 0) {
		si4703_mute(chip);
		chip->curvol = vol;
		return 0;
	}
	si4703_setvol_generic(chip, vol);
	chip->curvol = vol;

	return 0;
}

/* set a frequency, freq unit: 1/16th kHz */
int si4703_setfreq(struct si4703_device *chip, unsigned int freq)
{
	long timeout;
	uint16_t val;
	uint16_t channel;
	unsigned int freq_base;
	unsigned int freq_step;

	freq_base = FREQ_MIN*FREQ_MUL;
	freq_step = CHANNEL_SPACE*FREQ_MUL/1000;
	channel = (freq - freq_base)/freq_step;
	pr_debug("%s:channel %d(freq: %d)\n", __FUNCTION__, channel, freq);
	if (si4703_write(chip, REG_CHANNEL, channel | TUNE))
		return -EIO;

	/* wait for STC to be set */
	timeout = wait_event_timeout(chip->wait, si4703_get_rssi(chip) & STC,
			60*HZ/1000);
	pr_debug("get channel: %d\n", si4703_get_chan(chip) & 0x3ff);

	if (timeout == 0)
		return -EIO;

	if (si4703_get_rssi(chip) & AFCRL)
		return -EIO;

	/* clear tune bit */
	timeout = 6;
	if (si4703_read(chip, REG_CHANNEL, &val))
		return -EIO;

	if (si4703_write(chip, REG_CHANNEL, ~TUNE & val))
		return -EIO;

	/* wait for STC to be clear */
	do {
		mdelay(10);
	} while ((si4703_get_rssi(chip) & STC) && timeout--);

	if (timeout == 0)
		return -EIO;

	return 0;
}


static int vidioc_querycap(struct file *file, void *priv,
					struct v4l2_capability *v)
{
	strlcpy(v->driver, "si4703", sizeof(v->driver));
	strlcpy(v->card, "Si4703 FM Radio", sizeof(v->card));
	sprintf(v->bus_info, "I2C");
	v->version = RADIO_VERSION;
	v->capabilities = V4L2_CAP_TUNER | V4L2_CAP_RADIO;
	return 0;
}

static int vidioc_g_tuner(struct file *file, void *priv,
				struct v4l2_tuner *v)
{
	struct si4703_device *chip = video_get_drvdata(video_devdata(file));

	if (v->index > 0)
		return -EINVAL;

	strcpy(v->name, "FM");
	v->type = V4L2_TUNER_RADIO;
	v->rangelow = FREQ_MIN*FREQ_MUL;
	v->rangehigh = FREQ_MAX*FREQ_MUL;
	v->rxsubchans = V4L2_TUNER_SUB_MONO | V4L2_TUNER_SUB_STEREO;
	v->capability = V4L2_TUNER_CAP_LOW;
	v->audmode = V4L2_TUNER_MODE_STEREO;
	v->signal = si4703_get_rssi(chip) & RSSI;
	return 0;
}

static int vidioc_s_tuner(struct file *file, void *priv,
				struct v4l2_tuner *v)
{
	if (v->index > 0)
		return -EINVAL;

	return 0;
}

static int vidioc_s_frequency(struct file *file, void *priv,
				struct v4l2_frequency *f)
{
	struct si4703_device *chip = video_get_drvdata(video_devdata(file));

	chip->curfreq = f->frequency;
	if (si4703_setfreq(chip, chip->curfreq)) {
		printk(KERN_ERR "Set frequency failed");
		return -EIO;
	}
	return 0;
}

static int vidioc_g_frequency(struct file *file, void *priv,
				struct v4l2_frequency *f)
{
	struct si4703_device *chip = video_get_drvdata(video_devdata(file));

	f->type = V4L2_TUNER_RADIO;
	f->frequency = chip->curfreq;
	return 0;
}

static int vidioc_queryctrl(struct file *file, void *priv,
				struct v4l2_queryctrl *qc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(chip_qctrl); i++) {
		if (qc->id && qc->id == chip_qctrl[i].id) {
			memcpy(qc, &(chip_qctrl[i]),
						sizeof(*qc));
			return 0;
		}
	}
	return -EINVAL;
}

static int vidioc_g_ctrl(struct file *file, void *priv,
				struct v4l2_control *ctrl)
{
	struct si4703_device *chip = video_get_drvdata(video_devdata(file));

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_MUTE:
		ctrl->value = chip->muted;
		return 0;
	case V4L2_CID_AUDIO_VOLUME:
		ctrl->value = chip->curvol;
		return 0;
	}
	return -EINVAL;
}

static int vidioc_s_ctrl(struct file *file, void *priv,
				struct v4l2_control *ctrl)
{
	struct si4703_device *chip = video_get_drvdata(video_devdata(file));

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_MUTE:
		if (ctrl->value) {
			if (si4703_mute(chip))
				printk(KERN_ERR "si4703: no reponse");
		} else {
			if (si4703_unmute(chip))
				printk(KERN_ERR "si4703: no reponse");
		}
		break;
	case V4L2_CID_AUDIO_VOLUME:
		si4703_setvol(chip, ctrl->value);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int vidioc_g_audio(struct file *file, void *priv,
				struct v4l2_audio *a)
{
	if (a->index > 1)
		return -EINVAL;

	strcpy(a->name, "Radio");
	a->capability = V4L2_AUDCAP_STEREO;
	return 0;
}

static int vidioc_g_input(struct file *filp, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int vidioc_s_input(struct file *filp, void *priv, unsigned int i)
{
	if (i != 0)
		return -EINVAL;
	return 0;
}

static int vidioc_s_audio(struct file *file, void *priv,
					struct v4l2_audio *a)
{
	if (a->index != 0)
		return -EINVAL;
	return 0;
}

static int si4703_open(struct inode *inode, struct file *file)
{
	struct si4703_device *chip = video_get_drvdata(video_devdata(file));

	set_radio_dvfm_constraint();

	/* enable clock */
	enable_oscc_tout_s0();

	chip->users = 1;
	chip->muted = 1;

	if (si4703_power_up(chip)) {
		printk(KERN_ERR "Radio did not start up properly");
		chip->users = 0;
		disable_oscc_tout_s0();
		return -EIO;
	}
	si4703_setfreq(chip, chip->curfreq);

	return 0;
}

static int si4703_close(struct inode *inode, struct file *file)
{
	struct si4703_device *chip = video_get_drvdata(video_devdata(file));

	unset_radio_dvfm_constraint();

	if (!chip) {
		disable_oscc_tout_s0();
		return -ENODEV;
	}

	if (si4703_power_down(chip))
		printk(KERN_ERR "Radio did not shutdown properly");

	chip->users = 0;
	if (chip->removed)
		kfree(chip);

	/* disable clock */
	disable_oscc_tout_s0();

	return 0;
}

/* File system interface */
static const struct file_operations si4703_fops = {
	.owner		= THIS_MODULE,
	.open		= si4703_open,
	.release	= si4703_close,
	.ioctl		= video_ioctl2,
	.compat_ioctl	= v4l_compat_ioctl32,
	.llseek		= no_llseek,
};

/* V4L2 interface */
static struct video_device si4703_videodev_template = {
	.name		= "si4703",
	.fops		= &si4703_fops,
	.release	= video_device_release,
};

static irqreturn_t si4703_irq_handler(int irq, void *dev_id)
{
	struct si4703_device *chip = dev_id;

	wake_up(&chip->wait);
	pr_debug("si4703: interrupt\n");
	return IRQ_HANDLED;
}

static int si4703_probe(struct i2c_client *client)
{
	uint16_t id;
	int ret;
	struct si4703_device *chip;
	struct si4703_platform_data *pdata = client->dev.platform_data;

	chip = kmalloc(sizeof(struct si4703_device), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->videodev = video_device_alloc();
	if (!chip->videodev) {
		kfree(chip);
		return -ENOMEM;
	}
	chip->client = client;
	chip->irq = client->irq;
	init_waitqueue_head(&chip->wait);
	i2c_set_clientdata(client, chip);

	enable_oscc_tout_s0();
	pdata->setup(client, pdata->context);
	if (chip->irq >= 0) {
		ret = request_irq(chip->irq, si4703_irq_handler,
			       IRQF_TRIGGER_FALLING, "si4703", chip);
		if (ret) {
			kfree(chip->videodev);
			kfree(chip);
			printk(KERN_ERR "request IRQ for si4703 failed!\n");
			return ret;
		}
	}

	/* init shadow registers */
	if (si4703_read(chip, REG_READ_START - 1, &id)) {
		free_irq(chip->irq, chip);
		kfree(chip->videodev);
		kfree(chip);
		printk(KERN_ERR "%s: failed to init shadow registers\n",
			      __FUNCTION__);
		disable_oscc_tout_s0();
		return -EIO;
	}
	si4703_power_up(chip);
	ret = si4703_read(chip, REG_CHIPID, &id);
	if (ret)
		printk(KERN_ERR "%s: failed to detect si4703\n",
			      __FUNCTION__);
	else
		printk(KERN_INFO "%s: si4703(0x%04x) detected\n",
				__FUNCTION__, id);
	si4703_power_down(chip);

	/* init volume to maxium */
	chip->curvol = 0xf;

	memcpy(chip->videodev, &si4703_videodev_template,
		sizeof(si4703_videodev_template));
	chip->removed = 0;
	chip->users = 0;
	chip->curfreq = FREQ_MIN*FREQ_MUL;
	video_set_drvdata(chip->videodev, chip);
	if (video_register_device(chip->videodev, VFL_TYPE_RADIO, 0)) {
		printk(KERN_ERR "Could not register video device");
		free_irq(chip->irq, chip);
		video_device_release(chip->videodev);
		kfree(chip->videodev);
		kfree(chip);
		disable_oscc_tout_s0();
		return -EIO;
	}

	disable_oscc_tout_s0();
	return 0;
}

static int si4703_remove(struct i2c_client *client)
{
	struct si4703_device *chip = i2c_get_clientdata(client);

	if (chip->irq >= 0)
		free_irq(chip->irq, chip);
	video_unregister_device(chip->videodev);
	kfree(chip->videodev);
	kfree(chip);

	return 0;
}

/* This is the driver that will be inserted */
static struct i2c_driver si4703_driver = {
	.driver = {
		.name	= "si4703",
	},
	.probe	= si4703_probe,
	.remove	= si4703_remove,
};

static int __init si4703_init(void)
{
#ifdef CONFIG_PXA3xx_DVFM
        dvfm_register("FM radio", &dvfm_radio_idx);
#endif

	return i2c_add_driver(&si4703_driver);
}

static void __exit si4703_exit(void)
{
#ifdef CONFIG_PXA3xx_DVFM
	dvfm_unregister("FM radio", &dvfm_radio_idx);
#endif
	i2c_del_driver(&si4703_driver);
}

#ifndef MODULE
late_initcall(si4703_init);
#else
module_init(si4703_init);
#endif
module_exit(si4703_exit);

MODULE_AUTHOR("Walter Shao <wshao@marvell.com>");
MODULE_DESCRIPTION("si4703 FM Radio driver");
MODULE_LICENSE("GPL");

