/*
 * chip-rda5802 - main file for rda5802 FM chip driver
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
#include <linux/i2c/rda5802.h>
#include <linux/interrupt.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <asm/irq.h>
#ifdef CONFIG_PXA3xx_DVFM
#include <mach/hardware.h>
#include <mach/dvfm.h>
#include <mach/pxa3xx_dvfm.h>
#endif

#include <asm/unaligned.h>

/*
 * Version Information
 */
#include <linux/version.h>	/* for KERNEL_VERSION MACRO	*/

#ifdef CONFIG_YUHUA_MISC_DEV	
#include <mach/yuhua_board_dev_info.h>
#endif
#define RDS_DEBUG 0
#define DRIVER_VERSION "v0.01"
#define RADIO_VERSION KERNEL_VERSION(0, 0, 1)
#define FM_PROC_FS	1

#define FM_DEBUG 1

#if defined(FM_DEBUG) 
#define fm_debug(fmt,...) \
		printk(KERN_INFO pr_fmt(fmt),##__VA_ARGS__)
#else
#define fm_debug(fmt,...) \
		({if(0) printk(KERN_INFO pr_fmt(fmt),##__VA_ARGS__);0;})
#endif

#ifdef FM_PROC_FS
#include <linux/proc_fs.h>
#endif

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
#define REG_CHIPID	0x00

#define REG_POWERCFG	0x02
#define REG_CHANNEL	0x03
#define REG_SYSCONFIG1	0x04
#define REG_SYSCONFIG2	0x05
#define REG_SYSCONFIG3	0x06
#define REG_TEST1	0x07
#define REG_TEST2	0x08
#define REG_BOOTCFG	0x09
#define REG_READCHAN	0x0a
#define REG_RSSI	0x0b
#define REG_RDSA	0x0c
#define REG_RDSB	0x0d
#define REG_RDSC	0x0e
#define REG_RDSD	0x0f
#define REG_BLKERR_CD	0x10

/* REG_POWERCFG */
#define DHIZ		(1<<15)
#define DMUTE		(1<<14)
#define MUTE		0x0
#define MONO		(1<<13)
#define BASS		(1<<12)
#define SKMODE		(1<<7)
#define SEEKUP		(1<<9)
#define SEEKDOWN	0x0
#define SEEK		(1<<8)

#define RDS_ENABLE	(1<<3)
#define SOFT_RESET	(1<<1)
#define DISABLE		0x0
#define ENABLE		(1<<0)


/* REG_CHANNEL */
#define CHAN_BIT	6
#define TUNE		(1<<4)

#define BAND_EURO_US	(0x0<<2)	/* 87-108MHz */
#define BAND_JAPAN	(0x1<<2)	/* 76-91MHz */
#define BAND_WORLDWIDE	(0x2<<2)	/* 76-108MHZ */
#define BAND_EAST_EURO	(0x3<<2)	/* 65-76MHz */

#define SPACE_100KHZ		0x0
#define SPACE_200KHZ		0x1
#define SPACE_50KHZ		0x2

/* REG_SYSCONFIG1 */
#define RDSIEN		(1<<15)
#define STCIEN		(1<<14)
#define RBDS		(1<<13)
#define RDSR_MODE	(1<<10)
#define SOFTMUTE_EN	(1<<9)
#define GPIO3_INTERRUPT	(1<<4)
#define GPIO2_INTERRUPT	(1<<2)


/* REG_SYSCONFIG2 */
#define INT_MODE		(1<<15)
#define RECOMMENDED_SEEKTH	(20<<8) /*SEEKTH*/
#define LNA_PORT_SEL 	(0x2<<6) /*LNAP*/
#define LNA_ICSEL_BIT 	(0x2<<4) /*2.5mA*/



#define RECOMMENDED_VOLUME	0xA  


/* REG_SYSCONFIG3 */

/* REG_READCHAN */
#define RDSR			(1<<15)
#define STC			(1<<14)
#define SF			(1<<13)
#define RDSS			(1<<12)
#define ST			(1<<10)
#define READCHAN		0x03FF

/* REG_RSSI */
#define RSSI			0xFE00
#define RSSI_BIT	9
#define	FM_TRUE		(1<<8)
#define BLK_ERR_LEVEL_A_BIT 2  
#define BLK_ERR_LEVEL_B_BIT 0

/*REG_BLKERR_CD*/
#define BLK_ERR_LEVEL_C_BIT 14  
#define BLK_ERR_LEVEL_D_BIT 12

#define CORRECTED_NONE          0
#define CORRECTED_ONE_TO_TWO    1
#define CORRECTED_THREE_TO_FIVE 2
#define UNCORRECTABLE           3



#define FREQ_MUL		10

#ifdef CONFIG_RADIO_FM_EURO
#define BAND			BAND_EURO_US
#define SPACE			SPACE_100KHZ
#define CHANNEL_SPACE		100 /* in kHZ */
#define FREQ_MIN		87
#define FREQ_MAX		108
#endif

#ifdef CONFIG_RADIO_FM_JAPAN_WIDEBAND
#define BAND			BAND_WORLDWIDE
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
#define FREQ_MAX		91
#endif

#define REG_READ_START REG_READCHAN
#define REG_WRITE_START REG_POWERCFG
#define NR_rda5802_REGS 0x40

struct rda5802_device {
	struct i2c_client *client;
	int irq;
	struct video_device *videodev;
	uint16_t regs[NR_rda5802_REGS];
	int curfreq;
	int stereo;
	int users;
	int removed;
	int curvol;
	int muted;
	int forcemono;
	int bassmode;
	wait_queue_head_t	 wait_irq;
 
	/* RDS receive buffer */
	struct delayed_work rds_work;
	wait_queue_head_t read_queue;
	struct mutex lock;		/* buffer locking */
	unsigned char *buffer;		/* size is always multiple of three */
	unsigned int buf_size;
	unsigned int rd_index;
	unsigned int wr_index;

	
};

#define RDS_BUF_SIZE 100

static wait_queue_head_t	status_changed_wq;

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
int rda5802_write(struct rda5802_device *chip, uint8_t reg, uint16_t val)
{
	uint16_t count;
	uint8_t buf[NR_rda5802_REGS*2];
	int i;
	int ret;
	int index;

	chip->regs[reg] = val;

	for (i = 0; i < NR_rda5802_REGS; i++) {
		index = (REG_WRITE_START + i)%NR_rda5802_REGS;
		buf[2*i]     = chip->regs[index] >> 8;
		buf[2*i + 1] = chip->regs[index] & 0xff;
	}
	count = ((reg + NR_rda5802_REGS - REG_WRITE_START)%NR_rda5802_REGS + 1)*2;

	ret = i2c_master_send(chip->client, buf, count);
	if (ret < 0) {
		printk(KERN_ERR "rda5802_write: write failed!\n");
		return -EIO;
	}

	return 0;
}

int  rda5802_read(struct rda5802_device *chip, uint8_t reg, uint16_t *val)
{
	int ret;
	uint16_t count;
	uint8_t buf[NR_rda5802_REGS*2];
	int i;

	count = ((reg + NR_rda5802_REGS - REG_READ_START)%NR_rda5802_REGS + 1)*2;
	ret = i2c_master_recv(chip->client, buf, count);
	if (ret < 0) {
		printk(KERN_ERR "rda5802_read: read failed!\n");
		return -EIO;
	} else {
		 for (i = 0; i < count/2; i++)
			 chip->regs[(REG_READ_START + i)%NR_rda5802_REGS] =
				 (buf[2*i] << 8) + (buf[2*i + 1] & 0xff);
		 *val = chip->regs[reg];
	}

	return 0;
}

/* Low-level device interface begins here */
uint16_t rda5802_get_rssi(struct rda5802_device *chip)
{
	uint16_t val;

	rda5802_read(chip, REG_RSSI, &val);

	return val;
}

uint16_t rda5802_get_chan(struct rda5802_device *chip)
{
	uint16_t val;

	rda5802_read(chip, REG_READCHAN, &val);

	return val;
}

uint16_t rda5802_cur_channel_is_station(struct rda5802_device *chip)
{
	uint16_t val;

	rda5802_read(chip, REG_RSSI, &val);
  
	return ((val&FM_TRUE)==FM_TRUE);
}

int rda5802_power_up(struct rda5802_device *chip)
{
	rda5802_write(chip, REG_POWERCFG,SOFT_RESET);
  	mdelay(100);	
	rda5802_write(chip, REG_POWERCFG, DHIZ|DMUTE|ENABLE|RDS_ENABLE);
	rda5802_write(chip, REG_CHANNEL, SPACE | BAND);
	rda5802_write(chip, REG_SYSCONFIG1, STCIEN | RDSIEN|
			SOFTMUTE_EN | RDSR_MODE | GPIO2_INTERRUPT|RBDS);
	//rda5802_write(chip, REG_SYSCONFIG2, RECOMMENDED_SEEKTH|LNA_PORT_SEL|LNA_ICSEL_BIT|RECOMMENDED_VOLUME);
       rda5802_write(chip, REG_SYSCONFIG2, RECOMMENDED_SEEKTH|LNA_PORT_SEL|LNA_ICSEL_BIT);
	/*rda5802_write(chip, REG_SYSCONFIG3, SEN_THRESHOLD |
			DETECTION_THRESHOLD);*/
	mdelay(50);

	return 0;
}

int rda5802_power_down(struct rda5802_device *chip)
{
	/*  Set DISABLE bit as 1, perform internal power-down sequence and
	 *  set ENABLE bit=0 later by device itself
	 */
	return rda5802_write(chip, REG_POWERCFG, DISABLE);
}

static int rda5802_unmute(struct rda5802_device *chip)
{
	uint16_t val;

	if (rda5802_read(chip, REG_POWERCFG, &val))
		return -EIO;

	val |= DMUTE;

	if (rda5802_write(chip, REG_POWERCFG, val))
		return -EIO;

	chip->muted = 0;
	return 0;
}

static int rda5802_mute(struct rda5802_device *chip)
{
	uint16_t val;

	if (rda5802_read(chip, REG_POWERCFG, &val))
		return -EIO;

	val &= ~DMUTE;
	if (rda5802_write(chip, REG_POWERCFG, val))
		return -EIO;

	chip->muted = 1;
	return 0;
}
/* forcemono 1:force mono,0:auto*/
static int rda5802_monoselect(struct rda5802_device *chip,int forcemono)
{
	uint16_t val;

	if (rda5802_read(chip, REG_POWERCFG, &val))
		return -EIO;
	if(forcemono==1)
	val |= MONO;
	else
	val &=~MONO;
	//printk("rda5802 mono set 1=mono 0=stero %d",forcemono);
	if (rda5802_write(chip, REG_POWERCFG, val))
		return -EIO;

	chip->forcemono = forcemono;
	return 0;
}

/* bass_enabled 1:bass boost enabled,0:bass disabled*/
static int rda5802_set_bass(struct rda5802_device *chip,int bass_enabled)
{
	uint16_t val;

	if (rda5802_read(chip, REG_POWERCFG, &val))
		return -EIO;
	if(bass_enabled==1)
	val |= BASS;
	else
	val &=~BASS;
	
	if (rda5802_write(chip, REG_POWERCFG, val))
		return -EIO;

	chip->bassmode = bass_enabled;
	return 0;
}

static int rda5802_setvol_generic(struct rda5802_device *chip, int vol)
{
	uint16_t val;

	if (rda5802_read(chip, REG_SYSCONFIG2, &val))
		return -EIO;

	val &= ~0xf;
	if (rda5802_write(chip, REG_SYSCONFIG2, val | vol))
		return -EIO;
     
	return 0;
}

static int rda5802_setvol(struct rda5802_device *chip, int vol)
{
	/* user is unmuting the card */
	int rev=0;
	if (chip->muted && vol != 0) {
		chip->curvol = vol;
		rda5802_unmute(chip);
		rda5802_setvol_generic(chip, vol);
		return 0;
	}

	/* requested volume == current */
	if (vol == chip->curvol)
		return 0;

	/* volume == 0 means mute the card */
	if (vol == 0) {
		rev=rda5802_mute(chip);
		chip->curvol = vol;
		return 0;
	}
	rev=rda5802_setvol_generic(chip, vol);
	chip->curvol = vol;
	return rev;
}

/* set a frequency, freq unit: 100 kHz */
int rda5802_setfreq(struct rda5802_device *chip, unsigned int freq)
{
	long timeout;
	uint16_t val;
	uint16_t channel;
	unsigned int freq_base;
	unsigned int freq_step;

	freq_base = FREQ_MIN*FREQ_MUL;
	freq_step = CHANNEL_SPACE*FREQ_MUL/1000;
	channel = (freq - freq_base)/freq_step;     
  fm_debug("%s:channel %d(freq: %d)\n", __FUNCTION__, channel, freq);

	if (rda5802_write(chip, REG_CHANNEL, (channel<<CHAN_BIT) | TUNE| SPACE | BAND))
		return -EIO;
	
	/*add some delay,after we set TUNE,make sure the STC will be cleared*/
	udelay(100);

	/* wait for STC to be set,for 200ms,if timeout,set freq fail!*/
		timeout = wait_event_timeout(chip->wait_irq, rda5802_get_chan(chip) & STC,
		200*HZ/1000);
	  fm_debug("set channel to: %d\n", (rda5802_get_chan(chip) & 0x3ff)*freq_step+freq_base);

	  if (timeout == 0)
		  return -EIO;

	return 0;
}

/* seek next channel,direction 1:seek up,0:seek down*/
int rda5802_seek_next(struct rda5802_device *chip, unsigned int seek_upward)
{
	long timeout;
	uint16_t val;
	uint16_t channel;
	unsigned int freq_base;
	unsigned int freq_step;
	freq_base = FREQ_MIN*FREQ_MUL;
	freq_step = CHANNEL_SPACE*FREQ_MUL/1000;

	  if(rda5802_read(chip, REG_POWERCFG, &val))
	  	return -EIO;
	  
	  if (seek_upward==0){
	  val|= SEEK;
	  val &=(~SEEKUP);
	  }else
	  val|= SEEK|SEEKUP;
  
	if (rda5802_write(chip, REG_POWERCFG, val))
		return -EIO;

	udelay(100);
	/*add some delay,after we set SEEK,we have to wait a moment,the the STC will be cleared
	    udelay(10) will lead to 90% STC still keep the value of last time. */
	int reg_chan;

	/* wait for STC to be set,for nearly 6s,if timeout,seek fail!
	30ms per setp(100kHz),87.5~108 200 steps*/
	timeout = wait_event_timeout(chip->wait_irq, (reg_chan=rda5802_get_chan(chip)) & STC,
		((FREQ_MAX-FREQ_MIN)*FREQ_MUL*100/CHANNEL_SPACE)*HZ);

	fm_debug("seek channel to: %d,reg_chan=0x%4x\n", (reg_chan & 0x3ff)*freq_step+freq_base,reg_chan);

	if (rda5802_get_chan(chip)&SF)
		{
		printk("seek failed!\n");
		return -EIO;
		}

	  if (timeout == 0)
		  return -EIO;

	return 0;
}


/**************************************************************************
 * RDS Driver Functions
 **************************************************************************/
/*
 * si470x_rds_on - switch on rds reception
 */
static int rda5802_rds_on(struct rda5802_device *radio)
{
	int retval;

	/* sysconfig 1 */
	mutex_lock(&radio->lock);
	retval = rda5802_write(radio,REG_POWERCFG,radio->regs[REG_POWERCFG]|RDS_ENABLE);
	if (retval < 0)
		radio->regs[REG_POWERCFG] &= ~RDS_ENABLE;
	mutex_unlock(&radio->lock);

	return retval;
}

/*
 * si470x_rds_on - switch on rds reception
 */
static int rda5802_rds_int_enable(struct rda5802_device *radio)
{
	int retval;

	/* sysconfig 1 */
	mutex_lock(&radio->lock);
	retval = rda5802_write(radio,REG_SYSCONFIG1,radio->regs[REG_SYSCONFIG1]|RDSIEN);
	if (retval < 0)
		{
		 radio->regs[REG_SYSCONFIG1] &= ~RDSIEN;
		 
	       }
	mutex_unlock(&radio->lock);
     
	return retval;
}


/*
 * rda5802_rds - rds processing function
 */
static void rda5802_rds(struct rda5802_device *chip)
{
	unsigned char blocknum;
	unsigned short bler; /* rds block errors */
	unsigned short rds;
	unsigned short temp_reg;
	unsigned char tmpbuf[3];

	/* get rds blocks */
  
		if(rda5802_read(chip,REG_BLKERR_CD,&temp_reg)<0)
			return;
    	       if((chip->regs[REG_READCHAN] & RDSS) == 0) {
			// RDS Data not Synchronized 
		 //schedule_delayed_work(&chip->rds_work, msecs_to_jiffies(100));
		 printk("rda5802:RDS Data not Synchronized\n");
		 return;
		 }
		
	
	if ((chip->regs[REG_READCHAN] & RDSR) == 0) {
		printk("rda5802:RDS Data not Ready\n");
		return;
	}
	
    
	/* copy all four RDS blocks to internal buffer */
	mutex_lock(&chip->lock);
	for (blocknum = 0; blocknum < 4; blocknum++) {
		switch (blocknum) {
		default:
			bler = (chip->regs[REG_RSSI] >>BLK_ERR_LEVEL_A_BIT)&0x3;
			rds = chip->regs[REG_RDSA];
			break;
		case 1:
			bler = (chip->regs[REG_RSSI])&0x3;
			rds = chip->regs[REG_RDSB];
			break;
		case 2:
			bler = (chip->regs[REG_BLKERR_CD] >>BLK_ERR_LEVEL_C_BIT)&0x3;
			rds = chip->regs[REG_RDSC];
			break;
		case 3:
			bler = (chip->regs[REG_BLKERR_CD] >>BLK_ERR_LEVEL_D_BIT)&0x3;
			rds = chip->regs[REG_RDSD];
			break;
		};

		/* Fill the V4L2 RDS buffer */
		put_unaligned_le16(rds, &tmpbuf);
		tmpbuf[2] = blocknum;		/* offset name */
		tmpbuf[2] |= blocknum << 3;	/* received offset */
		if (bler > CORRECTED_ONE_TO_TWO)
			tmpbuf[2] |= 0x80; /* uncorrectable errors */
		else if (bler > CORRECTED_NONE)
			tmpbuf[2] |= 0x40; /* corrected error(s) */

		/* copy RDS block to internal buffer */
		memcpy(&chip->buffer[chip->wr_index], &tmpbuf, 3);
		chip->wr_index += 3;

		/* wrap write pointer */
		if (chip->wr_index >= chip->buf_size)
			chip->wr_index = 0;

		/* check for overflow */
		if (chip->wr_index == chip->rd_index) {
			/* increment and wrap read pointer */
			chip->rd_index += 3;
			if (chip->rd_index >= chip->buf_size)
				chip->rd_index = 0;
		}

	}
	mutex_unlock(&chip->lock);
     
}

/*
 * rda5802_work - rds work function
 */
static void rda5802_rds_work(struct work_struct *work)
{
	struct rda5802_device *radio = container_of(work, struct rda5802_device,rds_work.work);

	/* safety checks */
	//if (radio->disconnected)
		//return;
	if ((radio->regs[REG_POWERCFG] & RDS_ENABLE) == 0)
		return;

	rda5802_rds(radio);

	/* wake up read queue */
	if (radio->wr_index != radio->rd_index)
		wake_up_interruptible(&radio->read_queue);

	
}


#ifdef FM_PROC_FS
#define FM_DEBUG_PROC_ENTRY	"fmradio"
static u16 base_ino = 0;
struct rda5802_device *proc_fs_chip;


static ssize_t fm_regs_proc_read(struct file *filp,
		char *buffer, size_t length, loff_t *offset)
{
	uint16_t temp_reg;
	/* init shadow registers */
	rda5802_read(proc_fs_chip, REG_READ_START - 1, &temp_reg);
		
/*print the shadow register*/			
		printk(KERN_INFO "%s: rda5802 registers\n",
				__FUNCTION__);
		int i;
		for(i=0;i<NR_rda5802_REGS;i++)
		{
			printk(KERN_INFO "[0x%2x]=0x%4x",i,proc_fs_chip->regs[i]);
			if ((i%8)==0)
				printk(KERN_INFO "\n");
		}
		
	return 0;
}

static struct file_operations fm_regs_proc_ops = {
	.read = fm_regs_proc_read,
	.write = NULL,
};

static ssize_t fm_poll_proc_read(struct file *filp,
		char *buffer, size_t length, loff_t *offset)
{
	uint16_t temp_reg;

     long int ret;
      ret=wait_event_timeout(status_changed_wq, rda5802_get_chan(proc_fs_chip) & STC,	6*HZ);
	printk("fm_poll_proc_read ,ret=%d\n",ret);

	return 0;
}

static struct file_operations fm_poll_proc_ops = {
	.read = fm_poll_proc_read,
	.write = NULL,
};

static ssize_t fm_cmd_proc_read(struct file *filp,
		char *buffer, size_t length, loff_t *offset)
{
	
	return 0;
}

static ssize_t fm_cmd_proc_write(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[30];
	char *endp;
	char* pMsg;
	unsigned int frequency,rssi;
/*	unsigned int power;*/
	
	if (len > 30) len = 30;
	
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	pMsg = messages;
	messages[len-1] = 0x0;

	if ('p'==pMsg[0]) {
				/*cmd:power,1 on,0 off */
				if('1'==pMsg[2])
					{
					set_radio_dvfm_constraint();
					/* enable clock */
					enable_oscc_tout_s0();
					
					proc_fs_chip->users = 1;
					proc_fs_chip->muted = 1;
					
					rda5802_power_up(proc_fs_chip) ;
					}
				else 
					{
					rda5802_power_down(proc_fs_chip);
					disable_oscc_tout_s0();
					unset_radio_dvfm_constraint();					
					}
	} else if ('f'==pMsg[0]) {
				/*cmd:f xxxx,set freqency */
				frequency = simple_strtoul( &pMsg[2], NULL, 10);
				//printk("%s,%s,%c,%c,%c,%c,%c,len=%d,frequency=%d\n",messages,pMsg,pMsg[0],pMsg[1],pMsg[2],pMsg[3],pMsg[4],len,frequency);
				if(frequency<870||frequency>1080){		
					goto fail;
				}
				pMsg = endp+1;	
				
				proc_fs_chip->curfreq = frequency;
				if (rda5802_setfreq(proc_fs_chip, proc_fs_chip->curfreq)!=0) {
					printk(KERN_ERR "Set frequency(%d) failed\n",frequency);
					}else	{
						printk(KERN_ERR "Set frequency(%d) succuss!,is_station=%d\n",frequency,rda5802_cur_channel_is_station(proc_fs_chip));
					}
	}else if ('r'==pMsg[0])	{
				/*cmd:r,get rssi */
				int i;
				for (i=0;i<20;i++)
				{
				rssi=rda5802_get_rssi(proc_fs_chip);
				printk(KERN_INFO "rssi_reg=0x%4x,rssi=%d,is_station=%d",rssi,(rssi & RSSI)>>RSSI_BIT,rda5802_cur_channel_is_station(proc_fs_chip));
				msleep(1000);
				}
	}else if ('s'==pMsg[0])	{
				/*cmd:s x,seek channel,1 seek up,0 seek down */
				int seekdir = simple_strtoul( &pMsg[2], NULL, 10);
				
				int ret;
				ret=rda5802_seek_next(proc_fs_chip,seekdir);
				printk(KERN_INFO "rda5802_seek_next=%d\n",ret);
	}else if ('g'==pMsg[0])	{
				/*cmd:g,get currtent channel */
				int reg_chan;
				reg_chan=rda5802_get_chan(proc_fs_chip);				
				
				fm_debug("current channel is: %d,reg_chan=0x%4x\n", (reg_chan & 0x3ff)+870,reg_chan);

				printk(KERN_INFO "rda5802_get_chan=%d\n",reg_chan);
	}else if ('m'==pMsg[0])	{
		    int ret;
				/*cmd:m x,1:force mono,0:auto */
				int forcemono = simple_strtoul( &pMsg[2], NULL, 10);
				
				ret=rda5802_monoselect(proc_fs_chip,forcemono);
				printk(KERN_INFO "rda5802_monoselect=%d\n",ret);
	}else if ('a'==pMsg[0])	{
				/*cmd:a x,1:audio out,0:audio mute */
				int audio_on = simple_strtoul( &pMsg[2], NULL, 10);
				if (audio_on==0)
					rda5802_mute(proc_fs_chip);
				else
					rda5802_unmute(proc_fs_chip);

				printk(KERN_INFO "audio_on=%d\n",audio_on);
	}else if ('b'==pMsg[0])	{
				/*cmd:a x,1:bass enable,0:bass disable*/
				int bass_mode;
				bass_mode= simple_strtoul( &pMsg[2], NULL, 10);
				rda5802_set_bass(proc_fs_chip,bass_mode);

				printk(KERN_INFO "rda5802_set_bass=%d\n",bass_mode);
    }else if ('d'==pMsg[0])	{
					/*cmd:d:get rds data*/
					rda5802_rds(proc_fs_chip);	
					
	int retval = 0;
	unsigned int block_count = 0;

	/* calculate block count from byte count */
	//count /= 3;

	/* copy RDS block out of internal buffer and to user buffer */
	mutex_lock(&proc_fs_chip->lock);
	//while (block_count < count) 
	   while(1)
		{
		if (proc_fs_chip->rd_index == proc_fs_chip->wr_index)
			break;

		/* always transfer rds complete blocks */
		//if (copy_to_user(buf, &radio->buffer[radio->rd_index], 3))
			/* retval = -EFAULT; */
		//	break;
		printk("rds:0x%2x,0x%2x,0x%2x\n",proc_fs_chip->buffer[proc_fs_chip->rd_index],proc_fs_chip->buffer[proc_fs_chip->rd_index+1],proc_fs_chip->buffer[proc_fs_chip->rd_index+2]);

		/* increment and wrap read pointer */
		proc_fs_chip->rd_index += 3;
		if (proc_fs_chip->rd_index >= proc_fs_chip->buf_size)
			proc_fs_chip->rd_index = 0;

		/* increment counters */
		block_count++;
		//chip += 3;
		retval += 3;
  }        
	mutex_unlock(&proc_fs_chip->lock);
	
					printk(KERN_INFO "rda5802_rd\n");
	}else{
		goto fail;
	}

	return len;
fail:
	
	return -EFAULT;	
}

static struct file_operations fm_cmd_proc_ops = {
	.read = fm_cmd_proc_read,
	.write = fm_cmd_proc_write,
};

static int create_fm_proc_file(struct rda5802_device *chip)
{	
	struct proc_dir_entry *entry;
	struct proc_dir_entry *dadir;
	int i;

	dadir = proc_mkdir( FM_DEBUG_PROC_ENTRY, NULL);
	if (dadir == NULL) {
		printk(KERN_ERR "can't create /proc/%s \n",FM_DEBUG_PROC_ENTRY);
		return -1;
	}

	entry = create_proc_entry("regs", S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH, dadir);
	if (entry) {
		entry->proc_fops = &fm_regs_proc_ops;
	}

	entry = create_proc_entry("cmd", S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH, dadir);
	if (entry) {
		entry->proc_fops = &fm_cmd_proc_ops;
	}

	entry = create_proc_entry("poll", S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH, dadir);
	if (entry) {
		entry->proc_fops = &fm_poll_proc_ops;
	}
	
	proc_fs_chip=chip;
  
	return 0;
}

static void remove_fm_proc_file(void)
{	
}
#endif

static int vidioc_querycap(struct file *file, void *priv,
					struct v4l2_capability *v)
{
	strlcpy(v->driver, "rda5802", sizeof(v->driver));
	strlcpy(v->card, "rda5802 FM Radio", sizeof(v->card));
	sprintf(v->bus_info, "I2C");
	v->version = RADIO_VERSION;
	v->capabilities = V4L2_CAP_TUNER | V4L2_CAP_RADIO;
	return 0;
}

static int vidioc_g_tuner(struct file *file, void *priv,
				struct v4l2_tuner *v)
{
	struct rda5802_device *chip = video_get_drvdata(video_devdata(file));

	if (v->index > 0)
		return -EINVAL;

	uint16_t val;
	rda5802_read(chip, REG_RSSI, &val);
  
	strcpy(v->name, "FM");
	v->type = V4L2_TUNER_RADIO;
	v->rangelow = FREQ_MIN*FREQ_MUL;
	v->rangehigh = FREQ_MAX*FREQ_MUL;
	v->rxsubchans = V4L2_TUNER_SUB_MONO | V4L2_TUNER_SUB_STEREO;
	v->capability = V4L2_TUNER_CAP_LOW;
	v->audmode = ((chip->regs[REG_READCHAN]& ST)==ST)?V4L2_TUNER_MODE_STEREO:V4L2_TUNER_MODE_MONO;
	v->signal = (chip->regs[REG_RSSI]& RSSI)>>RSSI_BIT;
	v->reserved[0]=((chip->regs[REG_RSSI] & FM_TRUE)==FM_TRUE);
	return 0;
}

static int vidioc_s_tuner(struct file *file, void *priv,
				struct v4l2_tuner *tuner)
{
	
	struct rda5802_device *chip = video_drvdata(file);
	int retval = -EINVAL;
#if 0
	/* safety checks */
	if (chip->disconnected) {
		retval = -EIO;
		goto done;
	}
	if (chip->index != 0)
		goto done;
#endif

	/* mono/stereo selector */
	switch (tuner->audmode) {
	case V4L2_TUNER_MODE_MONO:
		retval=rda5802_monoselect(chip,1);  /* force mono */
		break;
	case V4L2_TUNER_MODE_STEREO:
		retval=rda5802_monoselect(chip,0); /* try stereo */
		break;
	default:
		goto done;
	}

done:
	if (retval < 0)
		printk(KERN_WARNING": set tuner failed with %d\n", retval);
	return retval;
}

static int vidioc_s_frequency(struct file *file, void *priv,
				struct v4l2_frequency *f)
{
	struct rda5802_device *chip = video_get_drvdata(video_devdata(file));

	chip->curfreq = f->frequency;
	f->reserved[0]=0;
	if (rda5802_setfreq(chip, chip->curfreq)) {
		printk(KERN_ERR "Set frequency failed");
		return -EIO;
	}
  f->reserved[0]=rda5802_cur_channel_is_station(chip);
	return 0;
}

static int vidioc_g_frequency(struct file *file, void *priv,
				struct v4l2_frequency *f)
{
	struct rda5802_device *chip = video_get_drvdata(video_devdata(file));
	unsigned int freq_base;
	unsigned int freq_step;
	freq_base = FREQ_MIN*FREQ_MUL;
	freq_step = CHANNEL_SPACE*FREQ_MUL/1000;
	
	f->type = V4L2_TUNER_RADIO;
	f->frequency = (rda5802_get_chan(chip) & 0x3ff)*freq_step+freq_base;
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
	struct rda5802_device *chip = video_get_drvdata(video_devdata(file));

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
	struct rda5802_device *chip = video_get_drvdata(video_devdata(file));

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_MUTE:
		if (ctrl->value) {
			if (rda5802_mute(chip))
				printk(KERN_ERR "rda5802: no reponse");
		} else {
			if (rda5802_unmute(chip))
				printk(KERN_ERR "rda5802: no reponse");
		}
		break;
	case V4L2_CID_AUDIO_BASS:
		rda5802_set_bass(chip,ctrl->value);
	  break;
	case V4L2_CID_AUDIO_VOLUME:
		rda5802_setvol(chip, ctrl->value);
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

/*
 * rda5802 vidioc_s_hw_freq_seek - set hardware frequency seek
 */
static int vidioc_s_hw_freq_seek(struct file *file, void *priv,
		struct v4l2_hw_freq_seek *seek)
{
	struct rda5802_device *chip = video_get_drvdata(video_devdata(file));
	int retval = 0;

	/* safety checks */
/*	
	if (radio->disconnected) {
		retval = -EIO;
		goto done;
	}
	if (seek->tuner != 0) {
		retval = -EINVAL;
		goto done;
	}
*/
   retval=rda5802_seek_next(chip,seek->seek_upward);
	//retval = si470x_set_seek(radio, seek->wrap_around, seek->seek_upward);

done:
	if (retval < 0)
		printk(KERN_WARNING "set hardware frequency seek failed with %d\n",	retval);
	return retval;
}

int g_fmRadioOn; /* a trick way to set huawei evdo modem handfree, fix me */
static int rda5802_open(struct inode *inode, struct file *file)
{
	struct rda5802_device *chip = video_get_drvdata(video_devdata(file));
       chip->users ++;
	 if(chip->users==1)
    {
    chip->muted = 1;
	set_radio_dvfm_constraint();
	/* enable clock */
	enable_oscc_tout_s0();
	if (rda5802_power_up(chip)) {
		printk(KERN_ERR "Radio did not start up properly");
		chip->users = 0;
		disable_oscc_tout_s0();
		return -EIO;
	}
	rda5802_setfreq(chip, chip->curfreq);
	g_fmRadioOn = 1;
    }
	return 0;
}

static int rda5802_close(struct inode *inode, struct file *file)
{
	struct rda5802_device *chip = video_get_drvdata(video_devdata(file));
    	chip->users--;
	 if(chip->users==0)
	 	{
	unset_radio_dvfm_constraint();
	if (!chip) {
		disable_oscc_tout_s0();
		return -ENODEV;
	}
	if (rda5802_power_down(chip))
		 {	
		printk(KERN_ERR "Radio did not shutdown properly");
		 }
	      /* disable clock */
		disable_oscc_tout_s0();
		g_fmRadioOn = 0;
	 }
	return 0;
}

/*
 * rda5802_fops_read - read RDS data
 */
static ssize_t rda5802_fops_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct rda5802_device *radio = video_drvdata(file);
	int retval = 0;
	unsigned int block_count = 0;
        //fm_debug("enter %s\n",__FUNCTION__);
	/* switch on rds reception */
	if ((radio->regs[REG_POWERCFG] & RDS_ENABLE) == 0) {
		rda5802_rds_on(radio);
		rda5802_rds_int_enable(radio);

		//schedule_delayed_work(&radio->rds_work,msecs_to_jiffies(40));
	}

	/* block if no new data available */
  
	while (radio->wr_index == radio->rd_index) {
		if (file->f_flags & O_NONBLOCK) {
			retval = -EWOULDBLOCK;
			goto done;
		}
		if (wait_event_interruptible(radio->read_queue,
			radio->wr_index != radio->rd_index) < 0) {
			retval = -EINTR;
			goto done;
		}
	
	}

	/* calculate block count from byte count */
	count /= 3;

	/* copy RDS block out of internal buffer and to user buffer */
	mutex_lock(&radio->lock);
	while (block_count < count) {
		if (radio->rd_index == radio->wr_index)
			break;

		/* always transfer rds complete blocks */
		if (copy_to_user(buf, &radio->buffer[radio->rd_index], 3))
			/* retval = -EFAULT; */
			break;

		/* increment and wrap read pointer */
		radio->rd_index += 3;
		if (radio->rd_index >= radio->buf_size)
			radio->rd_index = 0;

		/* increment counters */
		block_count++;
		buf += 3;
		retval += 3;
	}
	mutex_unlock(&radio->lock);
    
done:
	
	return retval;
}


/*
 * rda5802_fops_poll -  RDS data
 */
static unsigned int rda5802_fops_poll(struct file *file,
		struct poll_table_struct *pts)
{
	struct rda5802_device *radio = video_drvdata(file);
	int retval = 0;

	/* switch on rds reception */
	if ((radio->regs[REG_POWERCFG] & RDS_ENABLE) == 0) {
		rda5802_rds_on(radio);
		rda5802_rds_int_enable(radio);
	}

	poll_wait(file, &radio->read_queue, pts);

	if (radio->rd_index != radio->wr_index)
		retval = POLLIN | POLLRDNORM;

	return retval;
}

/* File system interface */
static const struct file_operations rda5802_fops = {
	.owner		= THIS_MODULE,
	.open		= rda5802_open,
	.release	= rda5802_close,
	.read		= rda5802_fops_read,
	.poll		= rda5802_fops_poll,
	.ioctl		= video_ioctl2,
	.compat_ioctl	= v4l_compat_ioctl32,
	.llseek		= no_llseek,
};

/*
 * si470x_ioctl_ops - video device ioctl operations
 */
static const struct v4l2_ioctl_ops rda5802_ioctl_ops = {
	.vidioc_querycap	= vidioc_querycap,
	.vidioc_queryctrl	= vidioc_queryctrl,
	.vidioc_g_ctrl		= vidioc_g_ctrl,
	.vidioc_s_ctrl		= vidioc_s_ctrl,
	
	.vidioc_g_audio		= vidioc_g_audio,
	.vidioc_g_tuner		= vidioc_g_tuner,
	.vidioc_s_tuner		= vidioc_s_tuner,
	.vidioc_g_frequency	= vidioc_g_frequency,
	.vidioc_s_frequency	= vidioc_s_frequency,
	.vidioc_s_hw_freq_seek	= vidioc_s_hw_freq_seek,
};

/* V4L2 interface */
static struct video_device rda5802_videodev_template = {
	.name		= "rda5802",
	.fops		= &rda5802_fops,
	.release	= video_device_release,
	.ioctl_ops		= &rda5802_ioctl_ops,
};

static irqreturn_t rda5802_irq_handler(int irq, void *dev_id)
{
	struct rda5802_device *chip = dev_id;

	wake_up(&chip->wait_irq);
	schedule_work(&chip->rds_work);
	pr_debug("rda5802: interrupt\n");
	return IRQ_HANDLED;
}

static int __devinit rda5802_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	//uint16_t id;
	int ret;
	struct rda5802_device *chip;
	struct rda5802_platform_data *pdata = client->dev.platform_data;

  printk("rda5802_probe\n");
  
	chip = kmalloc(sizeof(struct rda5802_device), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	/* rds buffer allocation */
	chip->buf_size = RDS_BUF_SIZE * 3;
	chip->buffer = kmalloc(chip->buf_size, GFP_KERNEL);
	if (!chip->buffer) {
		 
		return -ENOMEM;
	}

	/* rds buffer configuration */
	chip->wr_index = 0;
	chip->rd_index = 0;
	init_waitqueue_head(&chip->read_queue);

	/* prepare rds work function */
	INIT_DELAYED_WORK(&chip->rds_work, rda5802_rds_work);

	mutex_init(&chip->lock);

	chip->videodev = video_device_alloc();
	if (!chip->videodev) {
		kfree(chip->buffer);
		kfree(chip);
		return -ENOMEM;
	}
	chip->client = client;
	chip->irq = client->irq;
	init_waitqueue_head(&chip->wait_irq);
	init_waitqueue_head(&status_changed_wq);
	
	i2c_set_clientdata(client, chip);

	enable_oscc_tout_s0();
	pdata->setup(client, pdata->context);
	if (chip->irq >= 0) {
		ret = request_irq(chip->irq, rda5802_irq_handler,
			       IRQF_TRIGGER_FALLING, "rda5802", chip);
		if (ret) {
			kfree(chip->videodev);
                        kfree(chip->buffer);
			kfree(chip);
			printk(KERN_ERR "request IRQ for rda5802 failed!\n");
			return ret;
		}
	}

	/* init shadow registers */
	if (rda5802_read(chip, REG_READ_START - 1, &id)) {
		free_irq(chip->irq, chip);
		kfree(chip->videodev);
                kfree(chip->buffer);
		kfree(chip);
		printk(KERN_ERR "%s: failed to init shadow registers\n",
			      __FUNCTION__);
		disable_oscc_tout_s0();
		return -EIO;
	}
	rda5802_power_up(chip);
	ret = rda5802_read(chip, REG_CHIPID, &id);
	if (ret)
		printk(KERN_ERR "%s: failed to detect rda5802\n",
			      __FUNCTION__);
	else {
#ifdef CONFIG_YUHUA_MISC_DEV	
		set_fm_detect(1);
#endif
		printk(KERN_INFO "%s: rda5802(0x%04x) detected\n",
				__FUNCTION__, id);
	}
#if 0
/*print the shadow register*/			
		printk(KERN_INFO "%s: rda5802 registers\n",
				__FUNCTION__);
		int i;
		for(i=0;i<NR_rda5802_REGS;i++)
		{
			printk(KERN_INFO "[0x%2x]=0x%4x",i,chip->regs[i]);
			if ((i%8)==0)
				printk(KERN_INFO "\n");
		}
#endif				
	rda5802_power_down(chip);

	/* init volume to maxium */
	chip->curvol = 0xf;

	memcpy(chip->videodev, &rda5802_videodev_template,
		sizeof(rda5802_videodev_template));
	chip->removed = 0;
	chip->users = 0;
	chip->curfreq = FREQ_MIN*FREQ_MUL;
	video_set_drvdata(chip->videodev, chip);
	if (video_register_device(chip->videodev, VFL_TYPE_RADIO, 0)) {
		printk(KERN_ERR "Could not register video device");
		free_irq(chip->irq, chip);
		video_device_release(chip->videodev);
		kfree(chip->videodev);
                kfree(chip->buffer);
		kfree(chip);
		disable_oscc_tout_s0();
		return -EIO;
	}

	disable_oscc_tout_s0();
	
#ifdef FM_PROC_FS
	create_fm_proc_file(chip);
#endif
	
	return 0;
}

static int rda5802_remove(struct i2c_client *client)
{
	struct rda5802_device *chip = i2c_get_clientdata(client);

	/* stop rds reception */
	cancel_delayed_work_sync(&chip->rds_work);
	
	/* cancel read processes */
	wake_up_interruptible(&chip->read_queue);

	if (chip->irq >= 0)
		free_irq(chip->irq, chip);
	video_unregister_device(chip->videodev);
	kfree(chip->videodev);
        kfree(chip->buffer);
	kfree(chip);

#ifdef	FM_PROC_FS
	remove_fm_proc_file();
#endif

	return 0;
}

static const struct i2c_device_id rda5802_id[] = {
	{ "rda5802", 0 },
	{ }
};

/* This is the driver that will be inserted */
static struct i2c_driver rda5802_driver = {
	.driver = {
		.name	= "rda5802",
	},
	.probe	= rda5802_probe,
	.remove	= rda5802_remove,
	.id_table	= rda5802_id,
};

static int __init rda5802_init(void)
{
#ifdef CONFIG_PXA3xx_DVFM
        dvfm_register("FM radio", &dvfm_radio_idx);
#endif

  int ret=i2c_add_driver(&rda5802_driver);
  printk("rda5802_init.i2c_add_driver=%d\n",ret);
	return ret;
}

static void __exit rda5802_exit(void)
{
#ifdef CONFIG_PXA3xx_DVFM
	dvfm_unregister("FM radio", &dvfm_radio_idx);
#endif
	i2c_del_driver(&rda5802_driver);
}

#ifndef MODULE
late_initcall(rda5802_init);
#else
module_init(rda5802_init);
#endif
module_exit(rda5802_exit);

MODULE_AUTHOR("Walter Shao <wshao@marvell.com>");
MODULE_DESCRIPTION("rda5802 FM Radio driver");
MODULE_LICENSE("GPL");

