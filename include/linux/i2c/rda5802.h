/* platform data for the rda5802 8-bit I/O expander driver */

#ifndef __RDA5802_H
#define __RDA5802_H
struct rda5802_platform_data {
	void		*context;	/* param to setup/teardown */

	int		(*setup)(struct i2c_client *client,
				void *context);
	int		(*teardown)(struct i2c_client *client,
				void *context);
};

#endif

