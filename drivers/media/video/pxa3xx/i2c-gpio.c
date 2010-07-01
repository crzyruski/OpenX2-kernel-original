/* 
	Gpio I2C emulator
	Copyright@yuhuatel 2009
	Exp by frank.du 
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
	
#include <mach/mfp.h>
#include <mach/gpio.h>

#include "i2c-gpio.h"

#define GPIO_I2C_SCL MFP_PIN_GPIO89
#define GPIO_I2C_SDA MFP_PIN_GPIO87

/* 400K speed, 2.5us/2 */
#define GPIO_I2C_DELAY udelay(2) // 2us

#define GPIO_I2C_SCL_HIGH gpio_direction_output(mfp_to_gpio(GPIO_I2C_SCL), GPIO_LEVEL_HIGH)
#define GPIO_I2C_SCL_LOW gpio_direction_output(mfp_to_gpio(GPIO_I2C_SCL), GPIO_LEVEL_LOW)
#define GPIO_I2C_SCL_IN gpio_direction_input(mfp_to_gpio(GPIO_I2C_SCL))
#define GPIO_I2C_SCL_VALUE gpio_get_value(mfp_to_gpio(GPIO_I2C_SCL))

#define GPIO_I2C_SDA_HIGH gpio_direction_output(mfp_to_gpio(GPIO_I2C_SDA), GPIO_LEVEL_HIGH)
#define GPIO_I2C_SDA_LOW gpio_direction_output(mfp_to_gpio(GPIO_I2C_SDA), GPIO_LEVEL_LOW)
#define GPIO_I2C_SDA_IN gpio_direction_input(mfp_to_gpio(GPIO_I2C_SDA))
#define GPIO_I2C_SDA_VALUE gpio_get_value(mfp_to_gpio(GPIO_I2C_SDA))

#define GPIO_I2C_RETRY (3)

//#define GPIO_I2C_DEBUG
#ifdef GPIO_I2C_DEBUG
#define gpio_i2c_dbg(fmt, arg...) printk("gpio_i2c %s(line %d):" fmt , __FUNCTION__, __LINE__, ##arg)
#else
#define gpio_i2c_dbg(fmt, arg...) do {} while (0)
#endif
#define gpio_i2c_err(fmt, arg...) printk(KERN_ERR"gpio_i2c err,%s(line %d): " fmt, __FUNCTION__, __LINE__, ##arg)
#define gpio_i2c_info(fmt, arg...) printk("gpio_i2c,%s(line %d): " fmt, __FUNCTION__, __LINE__, ##arg)

static void i2c_gpio_send_start(void)
{
	gpio_i2c_dbg("\n");
	GPIO_I2C_SCL_HIGH;
	GPIO_I2C_SDA_HIGH;
	GPIO_I2C_DELAY;
	GPIO_I2C_SDA_LOW;
	GPIO_I2C_DELAY;
}

static void i2c_gpio_send_stop(void)
{
	gpio_i2c_dbg("\n");
	GPIO_I2C_SCL_LOW;
	GPIO_I2C_DELAY;
	GPIO_I2C_SDA_LOW;
	GPIO_I2C_DELAY;
	GPIO_I2C_SCL_HIGH;
	GPIO_I2C_DELAY;
	GPIO_I2C_SDA_HIGH;
	GPIO_I2C_DELAY;
}
static void i2c_gpio_send_ack_nack(int ack)
{
	gpio_i2c_dbg("ack %d\n", ack);
	GPIO_I2C_SCL_LOW;
	GPIO_I2C_SDA_LOW;
	GPIO_I2C_DELAY;
	if (ack)
		GPIO_I2C_SDA_LOW;
	else 
		GPIO_I2C_SDA_HIGH;
	GPIO_I2C_DELAY;
	GPIO_I2C_SCL_HIGH;
	GPIO_I2C_DELAY;
}

static int i2c_gpio_get_ack_nack(int ack)
{
	int ret;

	gpio_i2c_dbg("ack %d\n", ack);	
	GPIO_I2C_SCL_LOW;
	GPIO_I2C_SDA_IN;
	GPIO_I2C_DELAY;
	GPIO_I2C_SCL_HIGH;
	GPIO_I2C_DELAY;
	if (ack)
		ret = (GPIO_I2C_SDA_VALUE)?-1:0;
	else 
		ret = (GPIO_I2C_SDA_VALUE)?0:-1;
	
	if (ret<0) 
		gpio_i2c_err("nack\n");
	return ret;
}

static int i2c_gpio_wait_clk_free(void)
{
	int ret = -ETIME;
	int retry = 2000*100, i;
	gpio_i2c_dbg("\n");	

	GPIO_I2C_SCL_IN;
	for (i=0; i<retry; i++) {
		if (GPIO_I2C_SCL_VALUE) { /* high, no device pull down it */
			ret = 0;
			break;
		}
		GPIO_I2C_DELAY;
	}

	if (ret<0) 
		gpio_i2c_err("%d %d\n", i, retry);
	return ret;
}

static int i2c_gpio_wait_data_free(void)
{
	int ret = -ETIME;
	int retry = 1000*100, i;
	gpio_i2c_dbg("\n");	

	GPIO_I2C_SDA_IN;
	for (i=0; i<retry; i++) {			
		if (GPIO_I2C_SDA_VALUE) { /* high, no device pull down it */
			ret = 0;
			break;
		}
		GPIO_I2C_DELAY;
	}	

	if (ret<0)
		gpio_i2c_err("%d %d\n", i, retry);
	//gpio_i2c_info("%d %d\n", i, retry);
	return ret;
}

static int i2c_gpio_send_data(u8 data)
{
	int i, ret;
	gpio_i2c_dbg("data 0x%x\n", data);

	ret = i2c_gpio_wait_clk_free();
	if (ret<0) 
		gpio_i2c_err("wait_clk_free time out\n");
	GPIO_I2C_SCL_LOW;
	GPIO_I2C_DELAY;	
	ret = i2c_gpio_wait_data_free();
	if (ret<0) 
		gpio_i2c_err("wait_data_free time out\n");
		
	// send data, SCL high now
	for (i=0; i<8; i++) {
		GPIO_I2C_SCL_LOW;
		//GPIO_I2C_DELAY;
		if (data&(0x1<<(7-i)))
			GPIO_I2C_SDA_HIGH;
		else
			GPIO_I2C_SDA_LOW;
		GPIO_I2C_DELAY;
		GPIO_I2C_SCL_HIGH;
		GPIO_I2C_DELAY;
	}
	//GPIO_I2C_SCL_LOW;
	
	return ret;
}

static int i2c_gpio_rec_data(u8* pData)
{
	int i, ret;	
	u8 rec_data = 0;

	ret = i2c_gpio_wait_clk_free();
	if (ret<0) 
		gpio_i2c_err("wait_clk_free time out\n");
	GPIO_I2C_SCL_LOW;
	GPIO_I2C_SDA_IN;
	
	// receive data, SCL high now
	for (i=0; i<8; i++) {
		GPIO_I2C_SCL_LOW;
		GPIO_I2C_DELAY;
		if (0==i) {
			ret = i2c_gpio_wait_clk_free();
			if (ret<0) 
				gpio_i2c_err("wait_clk_free time out\n");
		}
		GPIO_I2C_SCL_HIGH;
		if (GPIO_I2C_SDA_VALUE)
			rec_data |= 0x1<<(7-i);
		GPIO_I2C_DELAY;
	}
	gpio_i2c_dbg("rec_data 0x%x\n", rec_data);
	*pData = rec_data;
	return ret;
}

int i2c_gpio_send(unsigned short addr, const char *buf, int count)
{
	u8 data;
	int ret = -EIO, i = -1, retry = 0;	

err_retry:	
	i2c_gpio_send_start();
	
	data = addr<<1; // pack address and rw(0:w)
	ret = i2c_gpio_send_data(data);	
	if (ret<0)
		goto out;
	ret = i2c_gpio_get_ack_nack(1);
	if (ret<0)
		goto out;
	
	for (i=0; i<count; i++) {		
		data = buf[i];		
		ret = i2c_gpio_send_data(data); 
		if (ret<0)
			goto out;
		ret = i2c_gpio_get_ack_nack(1);
		if (ret<0)
			goto out;
	}

	ret = i2c_gpio_wait_clk_free();
out:
	i2c_gpio_send_stop();
	if (ret<0) {
		gpio_i2c_err("ret %d, i %d, count %d, addr 0x%x, retry %d\n", ret, i, count, addr, retry);
		retry++;
		if (retry < GPIO_I2C_RETRY && i>=0)
			goto err_retry;
	}
	return ret;
}

int i2c_gpio_recv(unsigned short addr, char *buf ,int count)
{	
	u8 data;
	int ret = -EIO, i = -1, retry = 0;	

err_retry:	
	i2c_gpio_send_start();
	
	data = addr<<1 | 0x1; // pack address and rw(1:r)
	ret = i2c_gpio_send_data(data);	
	if (ret<0)
		goto out;
	ret = i2c_gpio_get_ack_nack(1);
	if (ret<0)
		goto out;
	
	for (i=0; i<count; i++) {
		ret = i2c_gpio_rec_data(&buf[i]);
		if (ret<0)
			goto out;
		if (i<(count-1))
			i2c_gpio_send_ack_nack(1);
		else
			i2c_gpio_send_ack_nack(0);
	}

out:
	i2c_gpio_send_stop();
	if (ret<0) {
		gpio_i2c_err("ret %d, i %d, count %d, addr 0x%x, retry %d\n", ret, i, count, addr, retry);
		retry++;
		if (retry < GPIO_I2C_RETRY && i>=0)
			goto err_retry;
	}
	return ret;
}

static void i2c_gpio_hw_init(void)
{
	gpio_i2c_dbg("\n");
	pxa3xx_mfp_set_afds(GPIO_I2C_SDA, MFP_AF0, MFP_DS04X);	
	//pxa3xx_mfp_set_lpm(GPIO_I2C_SDA, MFP_LPM_PULL_HIGH);

	pxa3xx_mfp_set_afds(GPIO_I2C_SCL, MFP_AF0, MFP_DS04X);
	//pxa3xx_mfp_set_lpm(GPIO_I2C_SCL, MFP_LPM_PULL_HIGH);
}

void i2c_gpio_init(void)
{
	GPIO_I2C_SDA_HIGH;
	GPIO_I2C_SCL_HIGH;
}

void i2c_gpio_deinit(void)
{
	GPIO_I2C_SCL_LOW;
	GPIO_I2C_SDA_LOW;
}

static void i2c_gpio_reset(void)
{
	GPIO_I2C_SCL_LOW;
	GPIO_I2C_DELAY; 	
	GPIO_I2C_SCL_HIGH;
	GPIO_I2C_DELAY;
}

static int __init module_i2c_gpio_init(void)
{
	i2c_gpio_hw_init();
	i2c_gpio_deinit();
	return 0;
}
MODULE_DESCRIPTION("Gpio i2c driver");
module_init(module_i2c_gpio_init);

