/*
 * Relay device TTY line discipline
 *
 * Copyright (c) 2008-2010 Zhonghua Qin 
 *
 * This is a module that converts a tty line into a much simpler
 * 'serial io port' abstraction that the input device drivers use.
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/tty.h>
#include <linux/proc_fs.h>

MODULE_AUTHOR("ZhonghuaQin");
MODULE_DESCRIPTION("Relay device TTY line discipline");
MODULE_LICENSE("GPL");
MODULE_ALIAS_LDISC(N_MOUSE);

extern struct tty_driver *usb_serial_tty_driver;

static struct tty_driver *gs_tty_driver = NULL;
extern int usb_serial_set_interface(int);
extern int usb_serial_get_interface(void);
/*
 * port_ldisc_open() is the routine that is called upon setting our line
 * discipline on a tty. It prepares the serio struct.
 */

static int port_ldisc_open(struct tty_struct *tty)
{
	return 0;
}

/*
 * port_ldisc_close() is the opposite of serport_ldisc_open()
 */

static void port_ldisc_close(struct tty_struct *tty)
{
    return ;
}

/*
 * port_ldisc_receive() is called by the low level tty driver when characters
 * are ready for us. We forward the characters, one by one to the 'interrupt'
 * routine.
 */

static void port_ldisc_receive(struct tty_struct *tty, const unsigned char *cp, char *fp, int count)
{
    struct tty_struct *other = NULL;
    int writelen = 0;
    
    if(tty->driver == usb_serial_tty_driver) {
        //Only 1st port connecting to pc host
        //printk(KERN_INFO "receive data from modem\n");
        if(gs_tty_driver != NULL) {
            other = gs_tty_driver->ttys[0];
            if(other) {
                while(1) {
                    writelen = gs_tty_driver->ops->write(other, cp, count);
                    if(writelen < count){
                        count -= writelen;
                        cp += writelen;
                        udelay(10);
                    } else {
                        break;
                    }
                }
            }
        }
    } else if(gs_tty_driver != NULL && tty->driver == gs_tty_driver) {
        //Only write data to /dev/ttyUSB2.
        int tty_usb_id = usb_serial_get_interface();
        //printk(KERN_INFO "receive data from pc\n");
        other = usb_serial_tty_driver->ttys[tty_usb_id];
        if(other && other->driver_data) {
            while(1) {
                // Davis: don't run in interrupt context here in_interrupt()
                writelen = usb_serial_tty_driver->ops->write(other, cp, count);
                if(writelen < count) {
                    count -= writelen;
                    cp += writelen;
                    udelay(15);
                } else 
                    break;
            }
        }
    } else {
        panic("Cann't process these data");
    }
}

/*
 * port_ldisc_read() just waits indefinitely if everything goes well.
 * However, when the serio driver closes the serio port, it finishes,
 * returning 0 characters.
 */

static ssize_t port_ldisc_read(struct tty_struct * tty, struct file * file, unsigned char __user * buf, size_t nr)
{
	return 0;
}

/*
 * port_ldisc_ioctl() allows to set the port protocol, and device ID
 */

static int port_ldisc_ioctl(struct tty_struct * tty, struct file * file, unsigned int cmd, unsigned long arg)
{
		return 0;
}

static void port_ldisc_write_wakeup(struct tty_struct * tty)
{
}

void tty_port_relay_register_gs(struct tty_driver *driver) 
{
    gs_tty_driver = driver;
    return ;
}
EXPORT_SYMBOL(tty_port_relay_register_gs);
 
int tty_port_relay_unregister_gs(struct tty_driver *driver)
{
    if(gs_tty_driver == driver) {
        gs_tty_driver = NULL;
        return 0;
    }

    return -1;
}
EXPORT_SYMBOL(tty_port_relay_unregister_gs);

/*
 * The line discipline structure.
 */

static struct tty_ldisc_ops port_ldisc = {
	.owner =	THIS_MODULE,
	.name =		"port_relay",
	.open =		port_ldisc_open,
	.close =	port_ldisc_close,
	.read =		port_ldisc_read,
	.ioctl =	port_ldisc_ioctl,
	.receive_buf =	port_ldisc_receive,
	.write_wakeup =	port_ldisc_write_wakeup
};

/*
 * The functions for insering/removing us as a module.
 */
static ssize_t tty_usb_read(struct file *filp, char *buf, size_t length, loff_t *offset)
{
    printk("Current /dev/ttyUSB%d\n", usb_serial_get_interface());
    return 0;
}

static ssize_t tty_usb_write(struct file *filp,
            const char* buf, size_t length, loff_t *offset)
{
    int rc;
    char c;

    rc = get_user(c, buf);
    if(rc)
        return rc;
    if(c >= '0' && c <= '4') {
        int intf = c - '0';
        printk("new /dev/ttyUSB%d\n", intf);
        usb_serial_set_interface(intf);        
    } else {
        printk("Wrong interface: (0-4) \n" );
    }

    return length;
}

static struct file_operations tty_usb_proc_fops = {
    .read = tty_usb_read,
    .write = tty_usb_write,
};

static int __init port_relay_init(void)
{
	int retval;
	retval = tty_register_ldisc(N_MOUSE, &port_ldisc);
	if (retval)
		printk(KERN_ERR "port_relay.c: Error registering line discipline.\n");

#ifdef CONFIG_PROC_FS
    proc_create("tty/ttyUSB", 0, NULL, &tty_usb_proc_fops);
#endif
    gs_tty_driver = NULL;
	return  retval;
}

static void __exit port_relay_exit(void)
{
	tty_unregister_ldisc(N_MOUSE);

#ifdef CONFIG_PROC_FS
    remove_proc_entry("tty/ttyUSB", NULL);
#endif
}

module_init(port_relay_init);
module_exit(port_relay_exit);

