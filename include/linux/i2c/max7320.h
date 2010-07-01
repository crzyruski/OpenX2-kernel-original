/* platform data for the max7320 8-bit I/O expander driver */

#ifndef __MAX7320_H
#define __MAX7320_H
struct max7320_platform_data {
	/* number of the first GPIO */
	unsigned	gpio_base;

	void		*context;	/* param to setup/teardown */

	int		(*setup)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	int		(*teardown)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
};

#endif

