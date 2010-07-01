/* 
	Camera host common operation
	Copyright@yuhuatel 2008
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/i2c.h>

#include <mach/camera.h>
#include <mach/mfp-pxa300.h>

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

static struct i2c_board_info cam_i2c_info[] = {
#if defined(CONFIG_CI_SAMSUNG_S5K4B)
	{
		.type	= "samung_s5k4b",
		.addr		= 0x29,
	},
#endif
#if defined(CONFIG_CI_SHARP_RJ6AB)
	{
		.type	= "sharp_vga_rj6ab",
		.addr		= 0x70,
	},
#endif
#if defined(CONFIG_CI_SHARP_RJ63SC)
	{
		.type	= "sharp_5m_rj63sc",
		.addr		= 0x10,
	},
	{
		.type	= "sharp_rj63sc_motor",
		.addr		= 0x0c,
	},
#endif
#if defined(CONFIG_CI_ALTEK_5M)
	{
		.type	= "altek_5m",
		.addr		= 0x1F,
	},
#endif
#if defined(CONFIG_CI_OV_2650)
	{
		.type	= "OV2650",
		.addr		= 0x30,
	},
#endif
#if defined(CONFIG_CI_MT9D113)
	{
		.type	= "mt9d113",
		.addr		= 0x78>>1,
	},
#endif
#if defined(CONFIG_CI_MT9T111)
	{
		.type	= "mt9t111",
		.addr		= 0x7a>>1,
	},
#endif
#if defined(CONFIG_CI_TOSHIBA_VGA)
	{
		.type	= "toshiba_vga",
		.addr		= 0x7C>>1,
	},
#endif
#if defined(CONFIG_CI_MCNEX_5M)
	{
		.type	= "mcnex_5m_addr",
		.addr		= 0x41,
	},
	{
		.type	= "mcnex_5m",
		.addr		= 0x40,
	},
#endif
};

/* camera platform data */
static mfp_cfg_t sync[] = {
	GPIO51_CI_HSYNC,
	GPIO52_CI_VSYNC,
};

static mfp_cfg_t sync_gpio[] = {
	GPIO51_CI_HSYNC_GPIO,
	GPIO52_CI_VSYNC_GPIO,
};

static void cam_sync_to_gpio(void)
{
	pxa3xx_mfp_config(ARRAY_AND_SIZE(sync_gpio));
}

static void cam_sync_from_gpio(void)
{
	pxa3xx_mfp_config(ARRAY_AND_SIZE(sync));
}

static int cam_init(void)
{
	return 0;
}

static void cam_deinit(void)
{
}

static void cam_suspend(void)
{
}

static void cam_resume(void)
{
}

static struct cam_platform_data cam_ops = {
	.vsync_gpio		= MFP_PIN_GPIO52, /* vsync */
	.init			= cam_init,
	.deinit 		= cam_deinit,
	.suspend		= cam_suspend,
	.resume 		= cam_resume,
	.sync_to_gpio		= cam_sync_to_gpio,
	.sync_from_gpio		= cam_sync_from_gpio,
};

extern struct platform_device pxa3xx_device_cam;
void __init cam_host_init(void)
{
	/* register platform info, like power, etc */
	pxa3xx_device_cam.dev.platform_data = &cam_ops;
	platform_device_register(&pxa3xx_device_cam);

	i2c_register_board_info(0, ARRAY_AND_SIZE(cam_i2c_info));
}

