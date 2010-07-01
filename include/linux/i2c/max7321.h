/* platform data for the max7321 8-bit I/O expander driver */

#ifndef __MAX7321_H
#define __MAX7321_H
struct max7321_platform_data {
	/* number of the first GPIO */
	unsigned int	gpio_base;
	int		gpio_irq;
	int		detected;

	void		*context;	/* param to setup/teardown */

	int		(*setup)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	int		(*teardown)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	void		(*poweron)(void);
};

#endif

