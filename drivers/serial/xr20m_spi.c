/* 
	xr20m spi interface
	Copyright@yuhuatel 2008
*/
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
	
#include <asm/types.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach-types.h>
#include <asm/hardware.h>
#include <asm/mach/arch.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/pxafb.h>
#include <asm/arch/mfp.h>
#include <asm/arch/pxa3xx_gpio.h>
#include <asm/arch/ssp.h>

#include "xr20m_spi.h"

static DEFINE_SPINLOCK(ssp1_spin_lock);

#define XR20M_RESET MFP_PIN_GPIO2_2 //reset gpio
#define XLLP_LO 		GPIO_LEVEL_LOW
#define XLLP_HI 		GPIO_LEVEL_HIGH

#define	MFP_SSP_1_FRM				(MFP_PIN_GPIO3_2)
#define	MFP_SSP_1_TXD				(MFP_PIN_GPIO88)
#define	MFP_SSP_1_RXD				(MFP_PIN_GPIO87)
#define	MFP_SSP_1_CLK				(MFP_PIN_GPIO85)	

void xr20m_reset(void)
{  
	pxa3xx_mfp_set_afds(XR20M_RESET, MFP_AF0, MFP_DS04X);
	pxa3xx_mfp_set_lpm(XR20M_RESET, MFP_LPM_PULL_HIGH); /* pull high in sleep */
  	pxa3xx_gpio_set_direction(XR20M_RESET, GPIO_DIR_OUT);
   	pxa3xx_gpio_set_level(XR20M_RESET, GPIO_LEVEL_HIGH);
   	udelay(1);
   	pxa3xx_gpio_set_level(XR20M_RESET, GPIO_LEVEL_LOW);
   	udelay(10);
   	//pxa3xx_gpio_set_level(XR20M_RESET, GPIO_LEVEL_HIGH);
	pxa3xx_gpio_set_direction(XR20M_RESET, GPIO_DIR_IN);
	udelay(1);
}

/* use gpio to emulate spi interface */
#if defined(XR20M_GPIO_SPI)
void xr20m_ssp1_init(void)
{
	pxa3xx_mfp_set_afds(MFP_SSP_1_CLK, MFP_AF0, MFP_DS04X);
	pxa3xx_mfp_set_afds(MFP_SSP_1_FRM, MFP_AF0, MFP_DS04X);
	pxa3xx_mfp_set_afds(MFP_SSP_1_TXD, MFP_AF0, MFP_DS04X);
	pxa3xx_mfp_set_afds(MFP_SSP_1_RXD, MFP_AF0, MFP_DS04X);

	pxa3xx_gpio_set_direction(MFP_SSP_1_FRM, GPIO_DIR_OUT);
	pxa3xx_gpio_set_level(MFP_SSP_1_FRM, GPIO_LEVEL_HIGH);

	pxa3xx_gpio_set_direction(MFP_SSP_1_CLK, GPIO_DIR_OUT);
	pxa3xx_gpio_set_level(MFP_SSP_1_CLK, GPIO_LEVEL_LOW);

	pxa3xx_gpio_set_direction(MFP_SSP_1_TXD, GPIO_DIR_OUT);
	pxa3xx_gpio_set_level(MFP_SSP_1_TXD, GPIO_LEVEL_LOW);

	pxa3xx_gpio_set_direction(MFP_SSP_1_RXD, GPIO_DIR_IN);
}

void xr20m_ssp1_deinit(void)
{
	return;
}

//#define ssp1_gpio_delay ndelay(1)	//gpio_ssp1_delay() //udelay(1)
#define ssp1_gpio_delay do{}while(0)
int xr20m_write(u8 reg, u8 val)
{
	int i;
	u16 command;
	u8 pack_reg;
	unsigned long flags;
	spin_lock_irqsave(&ssp1_spin_lock, flags);

	pxa3xx_gpio_set_level( MFP_SSP_1_FRM, XLLP_LO);

	pack_reg = (0x0<<7) | ((reg&0xF)<<3);
	command = ((pack_reg&0xFF)<<8) | (val&0xFF);

	/* send index and data, write data falling */
	for (i=0;i<16;i++) {
		pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_LO);
		if (command&(1<<(15-i)))
			pxa3xx_gpio_set_level( MFP_SSP_1_TXD, XLLP_HI);
		else
			pxa3xx_gpio_set_level( MFP_SSP_1_TXD, XLLP_LO);
		ssp1_gpio_delay;		
		pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_HI);
		ssp1_gpio_delay;
	}
	pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_LO);

	pxa3xx_gpio_set_level( MFP_SSP_1_FRM, XLLP_HI);

	spin_unlock_irqrestore(&ssp1_spin_lock, flags);
	return 0;
}

int xr20m_fifo_read(u8* buf, int len)
{
	u8 val = 0;
	u16 pack_reg;
	int i, index;
	int level;
	unsigned long flags;
	spin_lock_irqsave(&ssp1_spin_lock, flags);

	pxa3xx_gpio_set_level( MFP_SSP_1_FRM, XLLP_LO);

	pack_reg = (0x1<<7) | ((0x0&0xF)<<3);
	
	/* write index, write data falling */
	for (i=0;i<8;i++) {
		pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_LO);
		if (pack_reg&(1<<(7-i)))
			pxa3xx_gpio_set_level( MFP_SSP_1_TXD, XLLP_HI);
		else
			pxa3xx_gpio_set_level( MFP_SSP_1_TXD, XLLP_LO);
		ssp1_gpio_delay;		
		pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_HI);
		ssp1_gpio_delay;
	}

	/* read data, get data rising */
	for (index=0; index<len; index++) {
		val = 0;
		for (i=0; i<8; i++) {	
			pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_LO);
			ssp1_gpio_delay;

			level = pxa3xx_gpio_get_level(MFP_SSP_1_RXD);
			//dbg("i %d level %d\n", i, level);
			if (level)
				val |= 1<<(7-i);
			
			pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_HI);
			ssp1_gpio_delay;
		}
		buf[index] = val;
	}
	pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_LO);

	pxa3xx_gpio_set_level( MFP_SSP_1_FRM, XLLP_HI);

	spin_unlock_irqrestore(&ssp1_spin_lock, flags);
	return 0;
}

int xr20m_fifo_write(u8* buf, int len)
{
	int i, index;
	u8 val;
	unsigned long flags;
	spin_lock_irqsave(&ssp1_spin_lock, flags);

	pxa3xx_gpio_set_level( MFP_SSP_1_FRM, XLLP_LO);

	val = (0x0<<7) | ((0x0&0xF)<<3);
	/* send index, write data falling */
	for (i=0; i<8; i++) {
		pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_LO);
		if (val&(1<<(7-i)))
			pxa3xx_gpio_set_level( MFP_SSP_1_TXD, XLLP_HI);
		else
			pxa3xx_gpio_set_level( MFP_SSP_1_TXD, XLLP_LO);
		ssp1_gpio_delay;		
		pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_HI);
		ssp1_gpio_delay;
	}

	/* send data, write data falling */
	for (index=0; index<len; index++) {
		val = buf[index];
		for (i=0; i<8; i++) {
			pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_LO);
			if (val&(1<<(7-i)))
				pxa3xx_gpio_set_level( MFP_SSP_1_TXD, XLLP_HI);
			else
				pxa3xx_gpio_set_level( MFP_SSP_1_TXD, XLLP_LO);
			ssp1_gpio_delay;		
			pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_HI);
			ssp1_gpio_delay;
		}
	}	
	
	pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_LO);

	pxa3xx_gpio_set_level( MFP_SSP_1_FRM, XLLP_HI);

	spin_unlock_irqrestore(&ssp1_spin_lock, flags);
	return 0;
}

u8 xr20m_read(u8 reg)
{
	u8 val = 0;
	u16 pack_reg;
	int i;
	int level;
	unsigned long flags;
	spin_lock_irqsave(&ssp1_spin_lock, flags);

	pxa3xx_gpio_set_level( MFP_SSP_1_FRM, XLLP_LO);

	pack_reg = (0x1<<7) | ((reg&0xF)<<3);
	
	/* write index, write data falling */
	for (i=0;i<8;i++) {
		pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_LO);
		if (pack_reg&(1<<(7-i)))
			pxa3xx_gpio_set_level( MFP_SSP_1_TXD, XLLP_HI);
		else
			pxa3xx_gpio_set_level( MFP_SSP_1_TXD, XLLP_LO);
		ssp1_gpio_delay;		
		pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_HI);
		ssp1_gpio_delay;
	}	

	/* read data, get data rising */
	for (i=0;i<8;i++) {	
		pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_LO);
		ssp1_gpio_delay;

		level = pxa3xx_gpio_get_level(MFP_SSP_1_RXD);
		//dbg("i %d level %d\n", i, level);
		if (level)
			val |= 1<<(7-i);
		
		pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_HI);
		ssp1_gpio_delay;
	}
	pxa3xx_gpio_set_level( MFP_SSP_1_CLK, XLLP_LO);

	pxa3xx_gpio_set_level( MFP_SSP_1_FRM, XLLP_HI);

	spin_unlock_irqrestore(&ssp1_spin_lock, flags);
	return val;
}
#else
static struct ssp_dev xr20m_ssp1;
void xr20m_ssp1_init(void)
{
	pxa3xx_mfp_set_afds(MFP_SSP_1_FRM, MFP_AF0, MFP_DS04X);
  	pxa3xx_gpio_set_direction(MFP_SSP_1_FRM, GPIO_DIR_OUT);
   	pxa3xx_gpio_set_level(MFP_SSP_1_FRM, GPIO_LEVEL_HIGH);

	pxa_set_cken(CKEN_SSP1, 1);
	/* grab the port, configure it, then enable it */
	ssp_init(&xr20m_ssp1, 1, SSP_NO_IRQ);
	//pxa_set_cken(CKEN_SSP1, 1);
	
	ssp_disable(&xr20m_ssp1);
	//8bit, clk idle low, 4M
#if 1 /* try 13M */
	ssp_config(&xr20m_ssp1, 0x0, 0x000007C0, 0x0, 0x00C00007);
#else
	ssp_config(&xr20m_ssp1, 0x0, 0x000007C0, 0x0, 0x00C02007); //0x00C00207
#endif
	ssp_enable(&xr20m_ssp1);
	ssp_flush(&xr20m_ssp1);
}

void xr20m_ssp1_deinit(void)
{
	ssp_disable(&xr20m_ssp1);
	pxa_set_cken(CKEN_SSP1, 0);
}

int xr20m_write(u8 reg, u8 val)
{
	u32 cmd;	
	unsigned long flags;
	spin_lock_irqsave(&ssp1_spin_lock, flags);
	pxa3xx_gpio_set_level(MFP_SSP_1_FRM, XLLP_LO);

	cmd = (0x0<<7) | ((reg&0xF)<<3);
	ssp_write_word(&xr20m_ssp1, cmd); /* send index */
	ssp_flush(&xr20m_ssp1);

	cmd = val;
	ssp_write_word(&xr20m_ssp1, cmd); /* send data */
	ssp_flush(&xr20m_ssp1);

	pxa3xx_gpio_set_level( MFP_SSP_1_FRM, XLLP_HI);
	spin_unlock_irqrestore(&ssp1_spin_lock, flags);
	return 0;
}

u8 xr20m_read(u8 reg)
{
	u32 val = 0;
	u32 cmd;
	unsigned long flags;
	spin_lock_irqsave(&ssp1_spin_lock, flags);
	pxa3xx_gpio_set_level(MFP_SSP_1_FRM, XLLP_LO);
	
	cmd = (0x1<<7) | ((reg&0xF)<<3);
	ssp_write_word(&xr20m_ssp1, cmd); /* send index */
	ssp_flush(&xr20m_ssp1); /* clear rx */
	
	ssp_write_word(&xr20m_ssp1, 0); /* generate clock for rx */
	ssp_read_word(&xr20m_ssp1, &val); 

	pxa3xx_gpio_set_level(MFP_SSP_1_FRM, XLLP_HI);
	spin_unlock_irqrestore(&ssp1_spin_lock, flags);
	return (u8)val;
}

int xr20m_fifo_read(u8* buf, int len)
{
	u32 val;
	int index;
	unsigned long flags;
	spin_lock_irqsave(&ssp1_spin_lock, flags);
	pxa3xx_gpio_set_level( MFP_SSP_1_FRM, XLLP_LO);

	val = (0x1<<7) | ((0x0&0xF)<<3);
	ssp_write_word(&xr20m_ssp1, val); /* send index */
	ssp_flush(&xr20m_ssp1); /* clear rx */

	for (index=0; index<len; index++) {		
		ssp_write_word(&xr20m_ssp1, 0); /* generate clock for rx */
		ssp_read_word(&xr20m_ssp1, &val); 
		buf[index] = (u8)val;
	}

	pxa3xx_gpio_set_level( MFP_SSP_1_FRM, XLLP_HI);
	spin_unlock_irqrestore(&ssp1_spin_lock, flags);
	return 0;
}

int xr20m_fifo_write(u8* buf, int len)
{
	int index;
	u8 val;
	unsigned long flags;
	spin_lock_irqsave(&ssp1_spin_lock, flags);
	pxa3xx_gpio_set_level( MFP_SSP_1_FRM, XLLP_LO);

	val = (0x0<<7) | ((0x0&0xF)<<3);
	ssp_write_word(&xr20m_ssp1, val); /* send index */
	ssp_flush(&xr20m_ssp1); /* clear rx */

	for (index=0; index<len; index++) {
		val = buf[index];
		ssp_write_word(&xr20m_ssp1, val); /* send data */
		ssp_flush(&xr20m_ssp1); /* clear rx */
	}

	pxa3xx_gpio_set_level( MFP_SSP_1_FRM, XLLP_HI);
	spin_unlock_irqrestore(&ssp1_spin_lock, flags);
	return 0;
}
#endif

static int gsm2_uart_poll(void)
{
	u8 lsr = xr20m_read(XR20M_LSR);

	if(lsr & LSR_DR) {
		return 1;
	}
	
	if(lsr & (LSR_FIFOE | LSR_BI | LSR_FE | LSR_PE | LSR_OE)) {
		xr20m_err("lsr 0x%x\n", lsr);
		xr20m_write(XR20M_FCR, 0x7); /* Enable FIFO */
	}

	return 0;
}

int gsm2_uart_read(void)
{
	int rv;
	
	rv = gsm2_uart_poll();

	if(rv > 0) {
		rv= xr20m_read(XR20M_RBR) & 0xff;
		//if (rv>0) printk("gsm2_uart_read[0x%x %c]\n", rv, rv);	
	} else
		rv = -1;
	
	return rv;
}

int gsm2_uart_write(u8 c)
{
	u8 lsr;

	do {
		lsr = xr20m_read(XR20M_LSR);
		if(lsr & (LSR_FIFOE | LSR_BI | LSR_FE | LSR_PE | LSR_OE)) {
			xr20m_err("lsr 0x%x\n", lsr);
			xr20m_write(XR20M_FCR, 0x7); /* Enable FIFO */			
		}
	} while((lsr & LSR_TDRQ) == 0);
	//} while((lsr & (LSR_TDRQ|LSR_TEMT)) == 0);

	xr20m_dbg("c 0x%x\n", c);
	//printk("gsm2_uart_write c 0x%x\n", c);
	xr20m_write(XR20M_THR, c & 0xff);

	return 0;
}

static int xr20m_uart_setBaudrate(void)
{
	 u8 dll = 0, dlm = 0, dld = 0;
	 u8 lcr = xr20m_read(XR20M_LCR);
	 //printf("gsm2_uart_setBaudrate[0x%x]\n", baud);
	 
	 //case baud_460800:/* 13M/16/921600 = 1.7632 */
	 dll = 0x1; dld = 0xc;
	 
	 /* Set baud rate */
	 lcr |= LCR_DLAB;
	 xr20m_write(XR20M_LCR, lcr);
	 xr20m_write(XR20M_DLL, dll);
	 xr20m_write(XR20M_DLM, dlm);
	 xr20m_write(XR20M_DLD, dld);
	 lcr &= ~LCR_DLAB;
	 xr20m_write(XR20M_LCR, lcr);
	 
	 return 0;
}

int xr20m_detect(void)
{
	int ret = -EIO;
	u8 spr;
	u8 val = 0x55;

	xr20m_write(XR20M_SPR, val);
	spr = xr20m_read(XR20M_SPR);

	if (val==spr) {
		//printk("xr2om ssp1 detect ok\n");
		ret = 0;
	} else {
		printk("xr20m ssp1 fail[0x%x]\n", spr);
	}

	return ret;
}

/* 1: high 0:low */
int xr20m_RTS_level(int level)
{
	u8 mcr = xr20m_read(XR20M_MCR);
	
	if (GPIO_LEVEL_HIGH==level) {
		mcr &= ~0x2;
	} else {
		mcr |= 0x2;
	}
	xr20m_write(XR20M_MCR, mcr);
	
	return 0;
}

int xr20m_DSR_level(int level)
{
	u8 mcr = xr20m_read(XR20M_MCR);
	
	if (GPIO_LEVEL_HIGH==level) {
		mcr &= ~0x1;
	} else {
		mcr |= 0x1;
	}
	xr20m_write(XR20M_MCR, mcr);

	return 0;
}

void xr20m_uart_init(void)
{
	xr20m_write(XR20M_LCR, 0xBF);	// enable access to enhanced registers
#if defined(XR20M_AUTO_FLOW)
	xr20m_write(XR20M_EFR, 0xD0);// enable shaded bits and access to DLD, and no flow control
#else
	xr20m_write(XR20M_EFR, 0x10);// enable shaded bits and access to DLD, and no flow control
#endif
	xr20m_write(XR20M_LCR, LCR_WLS0 | LCR_WLS1);		
	xr20m_uart_setBaudrate();
	xr20m_write(XR20M_FCR, 0x7); /* Enable FIFO */	
	xr20m_write(XR20M_MCR, 0x3); /* RTS out low */
	xr20m_write(XR20M_IER, 0x1);
	
	/* Clear the FIFO buffers, rx trigger=16, tx trigger=8 */
#if XR20M_FIFO_LEN==16
	xr20m_write(XR20M_FCR, 0x47);
#elif XR20M_FIFO_LEN==56
	xr20m_write(XR20M_FCR, 0x87);
#elif XR20M_FIFO_LEN==8
	xr20m_write(XR20M_FCR, 0x07);
#endif

	/* Clear status */
	xr20m_read(XR20M_LSR);
	xr20m_read(XR20M_IIR);
	xr20m_read(XR20M_MSR);
}

#if 0
/* test at */
static void xr20m_at_test_read(void)
{
	int rv;
	
	for(;;) {
		rv = gsm2_uart_poll();

		if (rv > 0) {
			rv= xr20m_read(XR20M_RBR) & 0xff;
			if (rv>0) 
				printk("at_test_read[0x%x %c]\n", rv, rv);	
		} else
			break;
	}
}

void xr20m_test_spi(void)
{
	u8 spr;
	u8 val = 0;
	for (;;) {
		val++;
		xr20m_write(XR20M_SPR, val);
		spr = xr20m_read(XR20M_SPR);
		if (val!=spr)
			printk("xr20m ssp1 fail 0x%x 0x%x\n", spr, val);
	}
}


void xr20m_at_test(void)
{
	int i; 
	u8 at_cmd[] = {'a','t','*','y','h','t','e','s','t','=','1','3','0','\r'};
	u8 ate_cmd[] = {'a','t','e','0','\r'};
	u8 rx;

	/* nxp_on */
	pxa3xx_mfp_set_afds(MFP_PIN_GPIO80, MFP_AF0, MFP_DS03X);
	pxa3xx_gpio_set_direction(MFP_PIN_GPIO80, GPIO_DIR_OUT); 
	pxa3xx_gpio_set_level(MFP_PIN_GPIO80, GPIO_LEVEL_LOW);
	udelay(1);
	pxa3xx_gpio_set_level(MFP_PIN_GPIO80, GPIO_LEVEL_HIGH);
	msleep(100);
	pxa3xx_gpio_set_level(MFP_PIN_GPIO80, GPIO_LEVEL_LOW);

	for (i=0;i<500;i++) {
		xr20m_at_test_read();
		msleep(10);
	}

	for (i=0;i<ARRAY_SIZE(ate_cmd); i++)
		gsm2_uart_write(ate_cmd[i]);
	msleep(1);		
	xr20m_at_test_read();

	for(;;) {
		printk("Send at\n");
		for (i=0;i<ARRAY_SIZE(at_cmd); i++)
			gsm2_uart_write(at_cmd[i]);
		
		for (i=0; i<1000; i++) {
			udelay(1);			
			rx = xr20m_read(XR20M_RXLVL);
			if (rx>0) {
				printk("XR20M_RXLVL %x\n", rx);			
				xr20m_at_test_read();
			}
		}
		
		msleep(5000);
	}

}
#endif

