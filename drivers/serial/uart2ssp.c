/* 
	xr20m serial interface
	Copyright@yuhuatel 2008
*/
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/serial_reg.h>
#include <linux/circ_buf.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/mfp.h>
#include <asm/arch/pxa3xx_gpio.h>

#include "xr20m_spi.h"

#if defined(XR20M_DEBUG)
int bXr20mDbg = 0;
#endif

/* 0:close , 1 rw total, 2 each char */
static int xr20mRwDbg = 0;
u8 xr20m_ier;

static struct timer_list xr20m_timer; /* timer to read MSR: modem input */
enum {
	xr20m_sleep = 0,
	xr20m_sleep_pending,
	xr20m_active,
};
#define xr20m_timer_msec	(5*1000)
static int xr20m_sleep_state;

static void xr20m_sleep_enable(int enable)
{
	if (enable) { /* sleep */
		mod_timer(&xr20m_timer, jiffies+msecs_to_jiffies(xr20m_timer_msec));
		xr20m_sleep_state = xr20m_sleep_pending;
	} else { /* active */
		if (xr20m_sleep==xr20m_sleep_state) {
			xr20m_ier &= ~(XR20M_IER_SLEEPMODE);
			xr20m_write(XR20M_IER, xr20m_ier);
			udelay(1);
		}
		xr20m_sleep_state = xr20m_active;
	}
}

static void xr20m_timer_handler(unsigned long data)
{
	(void)data;
	if (xr20m_sleep_pending==xr20m_sleep_state) {
		if (xr20mRwDbg==1)
			printk("xr20m_timer_handler\n");
		xr20m_read(XR20M_MSR); /* read-out */
		xr20m_ier |= (XR20M_IER_SLEEPMODE);
		xr20m_write(XR20M_IER, xr20m_ier);
		xr20m_sleep_state = xr20m_sleep;
		//udelay(1);
	}
}

#define XR20M_INT (MFP_PIN_GPIO23)
static mfp_cfg_t xr20m_int_pins[] = {
	MFP_CFG_LPM(GPIO23, AF0, PULL_HIGH),
};

static void serial_xr20m_stop_tx(struct uart_port *port)
{
	/* fix me if use write interupt */
}

#define xr20m_tx_buff_max 64
static void uart2ssp_transmit_chars(struct uart_port *port)
{
	struct circ_buf *xmit = &port->info->xmit;
	int count = 0;
	u8 buf[xr20m_tx_buff_max];
	u8 txlvl, ier;

	if (xr20mRwDbg>=2)
		printk("Start send gsm2 string\n");

	ier = xr20m_ier;
	xr20m_write(XR20M_IER, 0); /* disable all int */	

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) 
		goto out;

	txlvl = xr20m_read(XR20M_TXLVL);	
	/* pack buff */	
	do {
		buf[count] = xmit->buf[xmit->tail];
		if (xr20mRwDbg>=2)
			printk(" 0x%x", buf[count]);
		count++;
		
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (count<xr20m_tx_buff_max && count<txlvl);	
	xr20m_fifo_write(buf, count);

out:
	if (xr20mRwDbg==1)
		printk("Send gsm2 %d chars\n", count);

	 /* restore int */
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		ier &= ~UART_IER_THRI;
		xr20m_write(XR20M_IER, ier);
	} else {
		ier |= UART_IER_THRI;
		xr20m_write(XR20M_IER, ier);
	}
	
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);	

	if (xr20mRwDbg>=2)
		printk("\n End send gsm2 string\n");
}

#define xr20m_rx_buff_max (256/2) //256
static void uart2ssp_receive_chars(struct uart_port *port)
{
	struct tty_struct *tty = port->info->tty;
	u8 rxlvl, buf[xr20m_rx_buff_max], ier;
	int sCount = 0;
	if (xr20mRwDbg>=2)
		printk("Start receive gsm2 string\n");
	ier = xr20m_ier;
	xr20m_write(XR20M_IER, 0);
	
	do { /* sort out all rx fifo */
		rxlvl = xr20m_read(XR20M_RXLVL);
		if (!rxlvl)
			break;
		if (xr20mRwDbg==1)
			printk("r%d\n", rxlvl);		
		
		if (sCount+rxlvl>xr20m_rx_buff_max) { /* reach buf max */
			tty_insert_flip_string(tty, buf, sCount);
			if (xr20mRwDbg==1)
				printk("receive %d gsm2 char\n", sCount);	
			sCount = 0;
		}
		
		xr20m_fifo_read(buf+sCount, rxlvl);		
		sCount += rxlvl;
		if (xr20mRwDbg>=2) {
			int i;
			for (i=0; i<XR20M_FIFO_LEN; i++)
				printk(" 0x%x", buf[i]);
		}
	}
#if XR20M_FIFO_LEN>=32
	while(0); /* only do once */
#else
	while(rxlvl>=XR20M_FIFO_LEN);
#endif

	if (sCount) {
		tty_insert_flip_string(tty, buf, sCount);
		tty_flip_buffer_push(tty);
	}

	xr20m_write(XR20M_IER, ier); /* enable int */
	if (xr20mRwDbg>=2)
		printk("\nreceive end\n");
	else if (xr20mRwDbg==1)
		printk("receive %d gsm2 char\n", sCount);
}

static irqreturn_t uart2ssp_irq_handler(int irq, void *dev_id)
{
	struct uart_port* port = (struct uart_port*)dev_id;
	u8 iir;
	xr20m_sleep_enable(0);
	
	iir = xr20m_read(XR20M_IIR);
	if (xr20mRwDbg>=2)
		printk("ier 0x%x start\n", iir);
	iir &= 0x3f;
	
	if (0x4==iir) { /* rx fifo full */
		uart2ssp_receive_chars(port);
	} else if (0xc==iir) { /* rx time out*/
		uart2ssp_receive_chars(port);
	} else if (0x2==iir) { /* tx ready */
		uart2ssp_transmit_chars(port);
	}

	xr20m_sleep_enable(1);
	return IRQ_HANDLED;
}

static unsigned int serial_xr20m_tx_empty(struct uart_port *port)
{
	unsigned long flags;
	unsigned int ret;

	spin_lock_irqsave(&port->lock, flags);	
	ret = xr20m_read(XR20M_LSR) & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
	spin_unlock_irqrestore(&port->lock, flags);
	return ret;
}

static void serial_xr20m_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	unsigned char mcr = 0;	
	
	if (mctrl & TIOCM_RTS) 
		mcr |= UART_MCR_RTS;	
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	xr20m_dbg("mcr 0x%x\n", mcr);	
	xr20m_write( XR20M_MCR, mcr);	
}

static unsigned int serial_xr20m_get_mctrl(struct uart_port *port)
{
	unsigned char status;
	unsigned int ret = 0;

	status = xr20m_read(XR20M_MSR);

	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;	
	
	xr20m_dbg("ret 0x%x\n", ret);
	return ret;
}

static void serial_xr20m_stop_rx(struct uart_port *port)
{
	u8 ier = xr20m_ier;
	ier &= ~UART_IER_THRI;
	xr20m_write(XR20M_IER, ier);
	xr20m_dbg("ier 0x%x\n", ier);
}

static void serial_xr20m_start_tx(struct uart_port *port)
{
	xr20m_sleep_enable(0);
#if defined(XR20M_TX_INT)
	u8 ier = xr20m_ier;	
	xr20m_write(XR20M_IER, ier | UART_IER_THRI);	
#else
	uart2ssp_transmit_chars(port);
	xr20m_sleep_enable(1);
#endif	
}

static void serial_xr20m_enable_ms(struct uart_port *port)
{	
	u8 ier = xr20m_ier;
	ier |= UART_IER_MSI;
	xr20m_write(XR20M_IER, ier);
	xr20m_dbg("ier 0x%x\n", ier);
}

static void serial_xr20m_break_ctl(struct uart_port *port, int break_state)
{
	unsigned long flags;
	unsigned lcr;
	spin_lock_irqsave(&port->lock, flags);

	lcr = xr20m_read(XR20M_LCR);	
	if (break_state == -1)
		lcr |= UART_LCR_SBC;
	else
		lcr &= ~UART_LCR_SBC;	
	xr20m_write(XR20M_LCR, lcr);
	
	spin_unlock_irqrestore(port->lock, flags);	
	xr20m_dbg("lcr 0x%x\n", lcr);
}

static int serial_xr20m_startup(struct uart_port *port)
{
	unsigned long flags;
	int ret;

	xr20m_sleep_enable(0);
	xr20m_write(XR20M_FCR, 0x7); /* Reset FIFO */
	ret = request_irq(IRQ_GPIO(MFP2GPIO(XR20M_INT)), uart2ssp_irq_handler, 
				IRQF_TRIGGER_FALLING, "xr20m int", port);
	if (ret) {
		xr20m_err("Request IRQ failed, return:%d\n", ret);
		return ret;
	}

	/* Now, initialize the UART */
	spin_lock_irqsave(&port->lock, flags);
	//port->mctrl |= TIOCM_OUT2;
	port->mctrl |= TIOCM_RTS;
	serial_xr20m_set_mctrl(port, port->mctrl);
	spin_unlock_irqrestore(&port->lock, flags);
	
	xr20m_dbg("\n");
  	return 0;
}

static void serial_xr20m_shutdown(struct uart_port *port)
{	
	free_irq(IRQ_GPIO(MFP2GPIO(XR20M_INT)), port);
}

static void serial_xr20m_set_termios(struct uart_port *port, 
			struct ktermios *termios, struct ktermios *old)
{
}

static void serial_xr20m_pm(struct uart_port *port, 
			unsigned int state, unsigned int oldstate)
{	
}

static const char * serial_xr20m_type(struct uart_port *port)
{
	return "xr20m";
}

static void serial_xr20m_release_port(struct uart_port *port)
{
}

static int serial_xr20m_request_port(struct uart_port *port)
{
	return 0;
}

static void serial_xr20m_config_port(struct uart_port *port, int flags)
{	
	port->type = PORT_PXA;
}

static int serial_xr20m_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	return -EINVAL;
}

struct uart_ops serial_xr20m_pops = {
	.tx_empty	= serial_xr20m_tx_empty,
	.set_mctrl	= serial_xr20m_set_mctrl,
	.get_mctrl	= serial_xr20m_get_mctrl,
	.stop_tx	= serial_xr20m_stop_tx,
	.start_tx	= serial_xr20m_start_tx,
	.stop_rx	= serial_xr20m_stop_rx,
	.enable_ms	= serial_xr20m_enable_ms,
	.break_ctl	= serial_xr20m_break_ctl,
	.startup	= serial_xr20m_startup,
	.shutdown	= serial_xr20m_shutdown,
	.set_termios	= serial_xr20m_set_termios,
	.pm		= serial_xr20m_pm,
	.type		= serial_xr20m_type,
	.release_port	= serial_xr20m_release_port,
	.request_port	= serial_xr20m_request_port,
	.config_port	= serial_xr20m_config_port,
	.verify_port	= serial_xr20m_verify_port,
};

static struct uart_port serial_xr20m_ports[] = {  
	{
		.type		= PORT_PXA,
		.iotype		= UPIO_MEM,
		//.membase	= (void *)&HWUART,
		//.mapbase	= __PREG(HWUART),
		//.irq		= IRQ_HWUART,
		.uartclk	= 921600 * 16,
		.fifosize	= 64,
		.ops		= &serial_xr20m_pops,
		.line		= 0,
	},
};

static struct uart_driver serial_xr20m = {
	.owner		= THIS_MODULE,
	.driver_name	= "PXA uart2ssp serial",
	.dev_name	= "ttySSP",
	.major		= TTY_MAJOR,
	.minor		= 80,
	.nr			= ARRAY_SIZE(serial_xr20m_ports),
	.cons		= NULL,
};

#if defined(XR20M_DEBUG)
static ssize_t xr20mDbg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{	
	sscanf(buf, "%d", &bXr20mDbg);
	return count;
}
static ssize_t xr20mDbg_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	return sprintf(buf, "%d\n", bXr20mDbg);
}
static DEVICE_ATTR(xr20mDbg, 0644, xr20mDbg_show, xr20mDbg_store);
#endif

static void xr20mRwDbg_info(void)
{
	if (xr20mRwDbg>=2)
		printk("each char dbg\n");
	else if (xr20mRwDbg==1)
		printk("total char dbg\n");
	else
		printk("off\n");
}
static ssize_t xr20mRwDbg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{	
	sscanf(buf, "%d", &xr20mRwDbg);
	xr20mRwDbg_info();
	return count;
}
static ssize_t xr20mRwDbg_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	xr20mRwDbg_info();
	return 0;
}
static DEVICE_ATTR(xr20mRwDbg, 0644, xr20mRwDbg_show, xr20mRwDbg_store);

static ssize_t xr20mReg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{/* r 0x1 or w 0x1 0x1 */
	char rw;
	int index, val;
	
	sscanf(buf, "%c %x %x", &rw, &index, &val);
	if (rw=='r') {
		printk("read xr20m reg:%d val:0x%x\n", index, xr20m_read(index));
	} else if (rw=='w') {
		xr20m_write(index, val);
		printk("write xr20m reg:%d val:0x%x\n", index, val);
	} else {
		printk("r/w 0x7 0x55\n");
	}
 
	return count;
}
static ssize_t xr20mReg_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	u8 reg;
	for (reg=0; reg<=XR20M_RXLVL; reg++) {
		printk("xr20m reg:%d val:0x%x\n", reg, xr20m_read(reg));
	} 
	return 0;
}
static DEVICE_ATTR(xr20mReg, 0644, xr20mReg_show, xr20mReg_store);

static ssize_t xr20mSleep_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int mode;	
	sscanf(buf, "%d", &mode);
	xr20m_sleep_enable(mode); 
	return count;
}
static ssize_t xr20mSleep_show(struct device *dev,struct device_attribute *attr,char * buf)
{
	printk("sleep state:%d\n", xr20m_sleep_state);
	return 0;
}
static DEVICE_ATTR(xr20mSleep, 0644, xr20mSleep_show, xr20mSleep_store);

static struct attribute *xr20m_attributes[] = {	
#if defined(XR20M_DEBUG)
	&dev_attr_xr20mDbg.attr,
#endif
	&dev_attr_xr20mRwDbg.attr,
	&dev_attr_xr20mReg.attr,
	&dev_attr_xr20mSleep.attr,
	NULL,
};
static struct attribute_group xr20m_attr_group ={
	.attrs= xr20m_attributes,
};

static int serial_xr20m_probe(struct platform_device *dev)
{
	int ret;

	xr20m_reset();
	xr20m_ssp1_init();
	ret = xr20m_detect();
	if (ret>=0) {
		xr20m_uart_init();
		xr20m_ier = xr20m_read(XR20M_IER);		
		
		ret = uart_add_one_port(&serial_xr20m, &serial_xr20m_ports[dev->id]);		
		serial_xr20m_ports[dev->id].dev = &dev->dev;
		platform_set_drvdata(dev, &serial_xr20m_ports[dev->id]);
		xr20m_dbg("ret[%d], id[%d]\n", ret, dev->id);

		/* setup int */
		pxa3xx_mfp_config(xr20m_int_pins, ARRAY_SIZE(xr20m_int_pins));	
		pxa3xx_gpio_set_direction(XR20M_INT, GPIO_DIR_IN);
		
		ret = sysfs_create_group(&dev->dev.kobj, &xr20m_attr_group);
		init_timer(&xr20m_timer);
		xr20m_timer.function = xr20m_timer_handler;
		xr20m_sleep_enable(1);
	}

	printk("XR20m spi-to-uart for gsm2 serial driver registered %s \n", ret<0?"fail":"succ");
	return ret;
}

static int serial_xr20m_remove(struct platform_device *dev)
{
	return 0;
}

static int serial_xr20m_suspend(struct platform_device *dev, pm_message_t state)
{
	xr20m_sleep_state = xr20m_sleep_pending;
	xr20m_timer_handler(0); /* let it sleep */
	xr20m_ssp1_deinit();
	return 0;
}

static int serial_xr20m_resume(struct platform_device *dev)
{
	xr20m_ssp1_init();
	//xr20m_uart_init();
	return 0;
}

static struct platform_driver serial_xr20m_driver = {
	.probe          = serial_xr20m_probe,
	.remove         = serial_xr20m_remove,
	.suspend_late	= serial_xr20m_suspend,
	.resume_early		= serial_xr20m_resume,
	.driver		= {
	        .name	= "xr20m-uart",
	},
};

static int __init serial_xr20m_init(void)
{
	int ret;

	ret = uart_register_driver(&serial_xr20m);
	if (ret<0)
		return ret;

	ret = platform_driver_register(&serial_xr20m_driver);
	if (ret<0)
		uart_unregister_driver(&serial_xr20m);

	return ret;
}

static void __exit serial_xr20m_exit(void)
{
	platform_driver_unregister(&serial_xr20m_driver);
	uart_unregister_driver(&serial_xr20m);
}

module_init(serial_xr20m_init);
module_exit(serial_xr20m_exit);
MODULE_AUTHOR("YUHUATEL");
MODULE_LICENSE("GPL");

