#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#

obj-$(CONFIG_DS1682)		+= ds1682.o
obj-$(CONFIG_AT24)		+= at24.o
obj-$(CONFIG_SENSORS_EEPROM)	+= eeprom.o
obj-$(CONFIG_SENSORS_MAX6875)	+= max6875.o
obj-$(CONFIG_SENSORS_PCA9539)	+= pca9539.o
obj-$(CONFIG_SENSORS_PCA963X)	+= pca963x.o
obj-$(CONFIG_SENSORS_PCF8574)	+= pcf8574.o
obj-$(CONFIG_PCF8575)		+= pcf8575.o
obj-$(CONFIG_SENSORS_PCF8591)	+= pcf8591.o
obj-$(CONFIG_SENSORS_AKM8976)	+= akm8976.o
obj-$(CONFIG_ISP1301_OMAP)	+= isp1301_omap.o
obj-$(CONFIG_TPS65010)		+= tps65010.o
obj-$(CONFIG_MENELAUS)		+= menelaus.o
obj-$(CONFIG_SENSORS_TSL2550)	+= tsl2550.o
obj-$(CONFIG_MCU_MPC8349EMITX)	+= mcu_mpc8349emitx.o
obj-$(CONFIG_PXA3xx_ARAVA)	+= arava.o
obj-$(CONFIG_PXA3xx_MICCO)	+= micco.o

bosch_bma020-objs		:= smb380.o bma150_driver.o axis_to_orientation.o
obj-$(CONFIG_BOSCH_BMA150)	+= bosch_bma020.o

ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

