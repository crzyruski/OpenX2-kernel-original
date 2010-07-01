#ifndef __ASM_ARCH_CAMERA_H__
#define __ASM_ARCH_CAMERA_H__

#define SENSOR_LOW  0
#define SENSOR_HIGH 1

struct cam_platform_data {
	unsigned int vsync_gpio;
	int (*init)(void);
	void (*deinit)(void);
	void (*suspend)(void);
	void (*resume)(void);
	void (*sync_to_gpio)(void);
	void (*sync_from_gpio)(void);
};

struct sensor_platform_data {
	int id;
	int (*power_on)(int);
	int (*power_off)(int);
};

void __init cam_host_init(void);

#if defined(CONFIG_PXA_CAMERA)
extern int ci_active(void);
#else
static int inline ci_active(void)
{
	return 0;
}
#endif

#endif

