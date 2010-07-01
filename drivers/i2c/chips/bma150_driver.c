/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2008 Bosch Sensortec GmbH
 * All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <asm/uaccess.h>
#include <linux/unistd.h>
#include <linux/module.h>
#ifdef CONFIG_YUHUA_MISC_DEV	
#include <mach/yuhua_board_dev_info.h>
#endif

#include "smb380.h"

#define BMA150_MAJOR	100
#define BMA150_MINOR	3

#define BMA150_IOC_MAGIC 'B'

#define BMA150_SOFT_RESET			_IO(BMA150_IOC_MAGIC,0)
#define BMA150_GET_OFFSET			_IOWR(BMA150_IOC_MAGIC,1, short)
#define BMA150_SET_OFFSET			_IOWR(BMA150_IOC_MAGIC,2, short)
#define BMA150_SELFTEST				_IOWR(BMA150_IOC_MAGIC,3, unsigned char)
#define BMA150_SET_RANGE			_IOWR(BMA150_IOC_MAGIC,4, unsigned char)
#define BMA150_GET_RANGE			_IOWR(BMA150_IOC_MAGIC,5, unsigned char)
#define BMA150_SET_MODE				_IOWR(BMA150_IOC_MAGIC,6, unsigned char)
#define BMA150_GET_MODE				_IOWR(BMA150_IOC_MAGIC,7, unsigned char)
#define BMA150_SET_BANDWIDTH			_IOWR(BMA150_IOC_MAGIC,8, unsigned char)
#define BMA150_GET_BANDWIDTH			_IOWR(BMA150_IOC_MAGIC,9, unsigned char)
#define BMA150_SET_WAKE_UP_PAUSE		_IOWR(BMA150_IOC_MAGIC,10,unsigned char)
#define BMA150_GET_WAKE_UP_PAUSE		_IOWR(BMA150_IOC_MAGIC,11,unsigned char)
#define BMA150_SET_LOW_G_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,12,unsigned char)
#define BMA150_GET_LOW_G_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,13,unsigned char)
#define BMA150_SET_LOW_G_COUNTDOWN		_IOWR(BMA150_IOC_MAGIC,14,unsigned char)
#define BMA150_GET_LOW_G_COUNTDOWN		_IOWR(BMA150_IOC_MAGIC,15,unsigned char)
#define BMA150_SET_HIGH_G_COUNTDOWN		_IOWR(BMA150_IOC_MAGIC,16,unsigned char)
#define BMA150_GET_HIGH_G_COUNTDOWN		_IOWR(BMA150_IOC_MAGIC,17,unsigned char)
#define BMA150_SET_LOW_G_DURATION		_IOWR(BMA150_IOC_MAGIC,18,unsigned char)
#define BMA150_GET_LOW_G_DURATION		_IOWR(BMA150_IOC_MAGIC,19,unsigned char)
#define BMA150_SET_HIGH_G_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,20,unsigned char)
#define BMA150_GET_HIGH_G_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,21,unsigned char)
#define BMA150_SET_HIGH_G_DURATION		_IOWR(BMA150_IOC_MAGIC,22,unsigned char)
#define BMA150_GET_HIGH_G_DURATION		_IOWR(BMA150_IOC_MAGIC,23,unsigned char)
#define BMA150_SET_ANY_MOTION_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,24,unsigned char)
#define BMA150_GET_ANY_MOTION_THRESHOLD		_IOWR(BMA150_IOC_MAGIC,25,unsigned char)
#define BMA150_SET_ANY_MOTION_COUNT		_IOWR(BMA150_IOC_MAGIC,26,unsigned char)
#define BMA150_GET_ANY_MOTION_COUNT		_IOWR(BMA150_IOC_MAGIC,27,unsigned char)
#define BMA150_SET_INTERRUPT_MASK		_IOWR(BMA150_IOC_MAGIC,28,unsigned char)
#define BMA150_GET_INTERRUPT_MASK		_IOWR(BMA150_IOC_MAGIC,29,unsigned char)
#define BMA150_RESET_INTERRUPT			_IO(BMA150_IOC_MAGIC,30)
#define BMA150_READ_ACCEL_X			_IOWR(BMA150_IOC_MAGIC,31,short)
#define BMA150_READ_ACCEL_Y			_IOWR(BMA150_IOC_MAGIC,32,short)
#define BMA150_READ_ACCEL_Z			_IOWR(BMA150_IOC_MAGIC,33,short)
#define BMA150_GET_INTERRUPT_STATUS		_IOWR(BMA150_IOC_MAGIC,34,unsigned char)
#define BMA150_SET_LOW_G_INT			_IOWR(BMA150_IOC_MAGIC,35,unsigned char)
#define BMA150_SET_HIGH_G_INT			_IOWR(BMA150_IOC_MAGIC,36,unsigned char)
#define BMA150_SET_ANY_MOTION_INT		_IOWR(BMA150_IOC_MAGIC,37,unsigned char)
#define BMA150_SET_ALERT_INT			_IOWR(BMA150_IOC_MAGIC,38,unsigned char)
#define BMA150_SET_ADVANCED_INT			_IOWR(BMA150_IOC_MAGIC,39,unsigned char)
#define BMA150_LATCH_INT			_IOWR(BMA150_IOC_MAGIC,40,unsigned char)
#define BMA150_SET_NEW_DATA_INT			_IOWR(BMA150_IOC_MAGIC,41,unsigned char)
#define BMA150_GET_LOW_G_HYST			_IOWR(BMA150_IOC_MAGIC,42,unsigned char)
#define BMA150_SET_LOW_G_HYST			_IOWR(BMA150_IOC_MAGIC,43,unsigned char)
#define BMA150_GET_HIGH_G_HYST			_IOWR(BMA150_IOC_MAGIC,44,unsigned char)
#define BMA150_SET_HIGH_G_HYST			_IOWR(BMA150_IOC_MAGIC,45,unsigned char)
#define BMA150_READ_ACCEL_XYZ			_IOWR(BMA150_IOC_MAGIC,46,short)
#define BMA150_READ_TEMPERATURE			_IOWR(BMA150_IOC_MAGIC,47,short)

#define IOCTL_SET_ACTIVE_SENSORS	_IO(BMA150_IOC_MAGIC, 48)
#define IOCTL_GET_ACTIVE_SENSORS	_IO(BMA150_IOC_MAGIC, 49)

#define BMA150_READ_ORIENTATION_X			_IOWR(BMA150_IOC_MAGIC,50,short)
#define BMA150_READ_ORIENTATION_Y			_IOWR(BMA150_IOC_MAGIC,51,short)
#define BMA150_READ_ORIENTATION_Z			_IOWR(BMA150_IOC_MAGIC,52,short)
#define BMA150_READ_ORIENTATION_XYZ		_IOWR(BMA150_IOC_MAGIC,53,short)

#define BMA150_IOC_MAXNR			54

#define DEBUG	0

static unsigned short ignore[] = { I2C_CLIENT_END };
static unsigned short normal_addr[] = {0x38, I2C_CLIENT_END};
static struct i2c_client *bma150_client = NULL;

struct bma150_data{
	struct i2c_client client;
};

static struct i2c_client_address_data addr_data = {
	.normal_i2c = normal_addr,
	.probe = ignore,
	.ignore = ignore,
};

static smb380_t smb380;
static int active_sensors;

static struct class *bma_dev_class;

static int bma150_attach_adapter(struct i2c_adapter *adapter);
static int bma150_detect(struct i2c_adapter *adapter, int address, int kind);
static int bma150_detach_client(struct i2c_client *client);

static char bma150_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len);
static char bma150_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len);

static int g_bma150_suspended;
int bma150_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int mode = SMB380_MODE_SLEEP;
	smb380_get_mode(&mode);
	if (SMB380_MODE_SLEEP!=mode) {
		printk(KERN_DEBUG"bma150_i2c_suspend %d\n", mode);
		smb380_set_mode(SMB380_MODE_SLEEP);
		g_bma150_suspended = 1;
	} else {
		g_bma150_suspended = 0;
	}
	
	return 0;
}

int bma150_i2c_resume(struct i2c_client *client)
{
	if (g_bma150_suspended)
		smb380_set_mode(SMB380_MODE_NORMAL);

	return 0;
}

static struct i2c_driver bma150_driver = {
	.driver = {
		.name	= "bma150",
	},
	.attach_adapter	= bma150_attach_adapter,
	.detach_client	= bma150_detach_client,
	.suspend = bma150_i2c_suspend,
	.resume = bma150_i2c_resume,
};

/*	i2c write routine for bma150	*/
static inline char bma150_i2c_write(unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	int dummy;	
	if( bma150_client == NULL )	/*	No global client pointer?	*/
		return -1;
	dummy = i2c_smbus_write_byte_data(bma150_client, reg_addr, data[0]);
	return dummy;	
}

/*	i2c read routine for bma150	*/
static inline char bma150_i2c_read(unsigned char reg_addr, unsigned char *data, unsigned char len) 
{
	int dummy=0;
	int i=0;
	if( bma150_client == NULL )	/*	No global client pointer?	*/
		return -1;
	while(i<len)
	{        
		dummy = i2c_smbus_read_word_data(bma150_client, reg_addr);
		if (dummy>=0)
		{         
			data[i] = dummy & 0x00ff;
			i++;
			if (i<len)
			{            
				data[i] = (dummy>>8)&0x00ff;
				i++;
			}
			reg_addr+=2;
		} 
		else 
			return dummy;
		dummy = len;
	}
	return dummy;
}

/*	read command for BMA150 device file	*/
static ssize_t bma150_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	#if DEBUG	
	smb380acc_t acc;	
	#endif
	if( bma150_client == NULL )
		return -1;
	#if DEBUG
	smb380_read_accel_xyz(&acc);
	printk("BMA150: X axis: %d\n" , acc.x);
	printk("BMA150: Y axis: %d\n" , acc.y); 
	printk("BMA150: Z axis: %d\n" , acc.z);  
	#endif
	return 0;
}

/*	write command for BMA150 device file	*/
static ssize_t bma150_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	if( bma150_client == NULL )
		return -1;
	#if DEBUG
	printk("BMA150 should be accessed with ioctl command\n");
	#endif
	return 0;
}

/*	open command for BMA150 device file	*/
static int bma150_driver_count;
static int bma150_open(struct inode *inode, struct file *file)
{
	if( bma150_client == NULL)
	{
		#if DEBUG
		printk("I2C driver not install\n"); 
		#endif
		return -1;
	}
	//smb380.bus_write = bma150_i2c_write;
	//smb380.bus_read = bma150_i2c_read;
	//smb380_init(&smb380);

	if (0==bma150_driver_count)
		smb380_set_mode(SMB380_MODE_NORMAL);
	bma150_driver_count++;

	if (smb380.chip_id>0)
	{
		#if 0
		printk("BMA150: ChipId: 0x%x\n" , smb380.chip_id); 
		printk("BMA150: ALVer: 0x%x MLVer: 0x%x\n", smb380.al_version, smb380.ml_version);
		#endif
	}
	else
	{
		#if 1
		printk("BMA150: open error\n"); 
		#endif
		return -1;
	}
	#if 0
	printk("BMA150 has been opened\n");
	#endif
	return 0;
}

/*	release command for BMA150 device file	*/
static int bma150_close(struct inode *inode, struct file *file)
{
	#if 0
	printk("BMA150 has been closed\n");	
	#endif
	if (bma150_driver_count>0) {
		bma150_driver_count--;
		if (0==bma150_driver_count)
			smb380_set_mode(SMB380_MODE_SLEEP);
	}
	return 0;
}


/*	ioctl command for BMA150 device file	*/
static int bma150_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];
	int sensors;

	/* check cmd */
	if(_IOC_TYPE(cmd) != BMA150_IOC_MAGIC)	
	{
		#if DEBUG		
		printk("cmd magic type error\n");
		#endif
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > BMA150_IOC_MAXNR)
	{
		#if DEBUG
		printk("cmd number error\n");
		#endif
		return -ENOTTY;
	}

	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if(err)
	{
		#if DEBUG
		printk("cmd access_ok error\n");
		#endif
		return -EFAULT;
	}
	/* check bam150_client */
	if( bma150_client == NULL)
	{
		#if DEBUG
		printk("I2C driver not install\n"); 
		#endif
		return -EFAULT;
	}
	
	/* cmd mapping */

	switch(cmd)
	{
	case BMA150_SOFT_RESET:
		err = smb380_soft_reset();
		return err;

	case BMA150_GET_OFFSET:
		//if (!((unsigned short*)data = kmalloc(4, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user((unsigned short*)data,(unsigned short*)arg,4)!=0)
		{
			#if DEBUG			
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_get_offset(*((unsigned short*)data),(unsigned short*)(data+2));
		if(copy_to_user((unsigned short*)arg,(unsigned short*)data,4)!=0)
		{
			#if DEBUG			
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_SET_OFFSET:
		//if (!((unsigned short*)data = kmalloc(4, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user((unsigned short*)data,(unsigned short*)arg,4)!=0)
		{
			#if DEBUG			
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_offset(*((unsigned short*)data),*(unsigned short*)(data+2));
		//kfree(data);
		return err;

	case BMA150_SELFTEST:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG			
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_selftest(*data);
		//kfree(data);
		return err;

	case BMA150_SET_RANGE:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG			
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_range(*data);
		//kfree(data);
		return err;

	case BMA150_GET_RANGE:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_get_range(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG			
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_SET_MODE:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG			
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_mode(*data);
		//kfree(data);
		return err;

	case BMA150_GET_MODE:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_get_mode(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG			
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_SET_BANDWIDTH:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_bandwidth(*data);
		//kfree(data);
		return err;

	case BMA150_GET_BANDWIDTH:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_get_bandwidth(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_SET_WAKE_UP_PAUSE:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_wake_up_pause(*data);
		//kfree(data);
		return err;

	case BMA150_GET_WAKE_UP_PAUSE:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_get_wake_up_pause(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_SET_LOW_G_THRESHOLD:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG			
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_threshold(*data);
		//kfree(data);
		return err;

	case BMA150_GET_LOW_G_THRESHOLD:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_get_low_g_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_SET_LOW_G_COUNTDOWN:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_countdown(*data);
		//kfree(data);
		return err;

	case BMA150_GET_LOW_G_COUNTDOWN:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_get_low_g_countdown(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_SET_HIGH_G_COUNTDOWN:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_countdown(*data);
		//kfree(data);
		return err;

	case BMA150_GET_HIGH_G_COUNTDOWN:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_get_high_g_countdown(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG			
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_SET_LOW_G_DURATION:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_duration(*data);
		//kfree(data);
		return err;

	case BMA150_GET_LOW_G_DURATION:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_get_low_g_duration(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_SET_HIGH_G_THRESHOLD:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_threshold(*data);
		//kfree(data);
		return err;

	case BMA150_GET_HIGH_G_THRESHOLD:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_get_high_g_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_SET_HIGH_G_DURATION:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_duration(*data);
		//kfree(data);
		return err;

	case BMA150_GET_HIGH_G_DURATION:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_get_high_g_duration(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_SET_ANY_MOTION_THRESHOLD:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_any_motion_threshold(*data);
		//kfree(data);
		return err;

	case BMA150_GET_ANY_MOTION_THRESHOLD:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_get_any_motion_threshold(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_SET_ANY_MOTION_COUNT:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_any_motion_count(*data);
		//kfree(data);
		return err;

	case BMA150_GET_ANY_MOTION_COUNT:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_get_any_motion_count(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_SET_INTERRUPT_MASK:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_interrupt_mask(*data);
		//kfree(data);
		return err;

	case BMA150_GET_INTERRUPT_MASK:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_get_interrupt_mask(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_RESET_INTERRUPT:
		err = smb380_reset_interrupt();
		return err;

	case BMA150_READ_ACCEL_X:
		//if (!((short*)data = kmalloc(2, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_read_accel_x((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_READ_ACCEL_Y:
		//if (!((short*)data = kmalloc(2, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_read_accel_y((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_READ_ACCEL_Z:
		//if (!((short*)data = kmalloc(2, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_read_accel_z((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_GET_INTERRUPT_STATUS:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_get_interrupt_status(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_SET_LOW_G_INT:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_int(*data);
		//kfree(data);
		return err;

	case BMA150_SET_HIGH_G_INT:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_int(*data);
		//kfree(data);
		return err;

	case BMA150_SET_ANY_MOTION_INT:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_any_motion_int(*data);
		//kfree(data);
		return err;

	case BMA150_SET_ALERT_INT:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_alert_int(*data);
		//kfree(data);
		return err;

	case BMA150_SET_ADVANCED_INT:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_advanced_int(*data);
		//kfree(data);
		return err;

	case BMA150_LATCH_INT:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_latch_int(*data);
		//kfree(data);
		return err;

	case BMA150_SET_NEW_DATA_INT:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_new_data_int(*data);
		//kfree(data);
		return err;

	case BMA150_GET_LOW_G_HYST:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_get_low_g_hysteresis(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_SET_LOW_G_HYST:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_low_g_hysteresis(*data);
		//kfree(data);
		return err;

	case BMA150_GET_HIGH_G_HYST:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_get_high_g_hysteresis(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_SET_HIGH_G_HYST:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
			#if DEBUG
			printk("copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = smb380_set_high_g_hysteresis(*data);
		//kfree(data);
		return err;

	case BMA150_READ_ACCEL_XYZ:
		//if (!((short*)data = kmalloc(6, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_read_accel_xyz((smb380acc_t*)data);
		if(copy_to_user((smb380acc_t*)arg,(smb380acc_t*)data,6)!=0)
		{
			#if DEBUG
			printk("copy_to error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_READ_ORIENTATION_X:
		//if (!((short*)data = kmalloc(2, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_read_orientation_x((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
		#if DEBUG
			printk("copy_to_user error\n");
		#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_READ_ORIENTATION_Y:
		//if (!((short*)data = kmalloc(2, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_read_orientation_y((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
		#if DEBUG
			printk("copy_to_user error\n");
		#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_READ_ORIENTATION_Z:
		//if (!((short*)data = kmalloc(2, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_read_orientation_z((short*)data);
		if(copy_to_user((short*)arg,(short*)data,2)!=0)
		{
		#if DEBUG
			printk("copy_to_user error\n");
		#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_READ_ORIENTATION_XYZ:
		//if (!((short*)data = kmalloc(6, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_read_orientation_xyz((smb380acc_t*)data);
		if(copy_to_user((smb380acc_t*)arg,(smb380acc_t*)data,6)!=0)
		{
		#if DEBUG
			printk("copy_to error\n");
		#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;

	case BMA150_READ_TEMPERATURE:
		//if (!((unsigned char*)data = kmalloc(1, GFP_KERNEL)))
		//	return = -ENOMEM;
		err = smb380_read_temperature(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
			#if DEBUG
			printk("copy_to_user error\n");
			#endif
			return -EFAULT;
		}
		//kfree(data);
		return err;
		
	case IOCTL_SET_ACTIVE_SENSORS:
		err=0;
		if (copy_from_user(&sensors, (int *)arg, sizeof(int)))
			err = -EFAULT;
		if(!err) 
		{
			active_sensors=sensors;
			#if DEBUG			
			printk("IOCTL_SET_ACTIVE_SENSORS=%d\n",active_sensors);
			#endif			
		}
		else
		{
			printk("IOCTL_SET_ACTIVE_SENSORS usercopy fail");
             }		
		return err;
	case IOCTL_GET_ACTIVE_SENSORS:
		err=0;
		sensors = active_sensors;
		if (copy_to_user((int *)arg, &sensors, sizeof(int)))
			err = -EFAULT;
		return err;
		
	default:
		return 0;
	}
}


static const struct file_operations bma150_fops = {
	.owner = THIS_MODULE,
	.read = bma150_read,
	.write = bma150_write,
	.open = bma150_open,
	.release = bma150_close,
	.ioctl = bma150_ioctl,
};

static int bma150_detect(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client;
	struct bma150_data *data;
	int err = 0;
	int tempvalue;
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		goto exit;

	/* OK. For now, we presume we have a valid client. We now create the
	   client structure, even though we cannot fill it completely yet. */
	if (!(data = kmalloc(sizeof(struct bma150_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct bma150_data));

	new_client = &data->client;
	i2c_set_clientdata(new_client, data);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &bma150_driver;
	new_client->flags = 0;

	if (i2c_smbus_read_byte(new_client) < 0)
	{
		goto exit_kfree;
	}
	else
	{
		printk("Bosch Sensortec Device detected!\n");
	}

	strlcpy(new_client->name, "bma150", I2C_NAME_SIZE);
	bma150_client = new_client;
		
	/* Tell the I2C layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto exit_kfree;

	/* read chip id */

	tempvalue = i2c_smbus_read_word_data(new_client, 0x00);
	if((tempvalue&0x00FF) == 0x0002)
	{
		printk("BMA150/SMB380 registered I2C driver!\n");
#ifdef CONFIG_YUHUA_MISC_DEV	
		set_gsensor_detect(1);
#endif
		smb380.bus_write = bma150_i2c_write;
		smb380.bus_read = bma150_i2c_read;
		smb380_init(&smb380);
		smb380_set_mode(SMB380_MODE_SLEEP);
		/* register a char dev	*/
		register_chrdev(BMA150_MAJOR, "BMA150", &bma150_fops);
	}
	else
	{
		printk("BMA150/SMB380 not registered 0x%x!\n", tempvalue);
		i2c_detach_client(bma150_client);
		bma150_client = NULL;
		goto exit_kfree;
	}

	return 0;

exit_kfree:
	kfree(data);
exit:
	return err;
}

static int bma150_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, bma150_detect);
}

static int bma150_detach_client(struct i2c_client *client)
{
	int err;

	if ((err = i2c_detach_client(client)))
	{
		dev_err(&client->dev,"Client deregistration failed, client can not be detached!\n");
		return err;
	}

	bma150_client = NULL;

	kfree(i2c_get_clientdata(client));
	return 0;
}


static int __init bma150_init(void)
{
	int res;
	struct device *dev;

	/* create BMA-dev device class */
	bma_dev_class = class_create(THIS_MODULE, "BMA-dev");
	if (IS_ERR(bma_dev_class)) {
		res = PTR_ERR(bma_dev_class);
		goto out_unreg_chrdev;
	}
	/* add i2c driver for bma150 */
	res = i2c_add_driver(&bma150_driver);
	if (res)
		goto out_unreg_class;
	/* create device node for bma150 */
	dev = device_create(bma_dev_class, NULL,
				     MKDEV(BMA150_MAJOR, 0),NULL,
				     "bma150");
	if (IS_ERR(dev)) {
		res = PTR_ERR(dev);
		goto error_destroy;
	}
	printk(KERN_INFO "BMA150 device create ok\n");
	
	return 0;

error_destroy:
	i2c_del_driver(&bma150_driver);
out_unreg_class:
	class_destroy(bma_dev_class);
out_unreg_chrdev:
	unregister_chrdev(BMA150_MAJOR, "BMA150");
out:
	printk(KERN_ERR "%s: Driver Initialisation failed\n", __FILE__);
	return res;
}

static void __exit bma150_exit(void)
{
	i2c_del_driver(&bma150_driver);
	class_destroy(bma_dev_class);
	unregister_chrdev(BMA150_MAJOR,"BMA150");
	printk(KERN_ERR "BMA150 exit\n");
}


MODULE_AUTHOR("Bin Du <bin.du@cn.bosch.com>");
MODULE_DESCRIPTION("BMA150 driver");
MODULE_LICENSE("GPL");

module_init(bma150_init);
module_exit(bma150_exit);

