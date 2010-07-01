/* platform data for the si4703 8-bit I/O expander driver */

#ifndef __SI4703_H
#define __SI4703_H
struct si4703_platform_data {
	void		*context;	/* param to setup/teardown */

	int		(*setup)(struct i2c_client *client,
				void *context);
	int		(*teardown)(struct i2c_client *client,
				void *context);
};

#endif

