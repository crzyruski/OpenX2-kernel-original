#ifndef __YUHUA_I2C_GPIO_H__
#define __YUHUA_I2C_GPIO_H__

int i2c_gpio_send(unsigned short addr, const char *buf, int count);
int i2c_gpio_recv(unsigned short addr, char *buf ,int count);

void i2c_gpio_init(void);
void i2c_gpio_deinit(void);

#endif

