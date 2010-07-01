
#
# USB OTG controller driver
#

obj-$(CONFIG_USB_PXA3XX_U2D)	+= pxa310_otg.o
obj-$(CONFIG_USB_PXA3XX_U2D)	+= pxa930_otg.o
obj-$(CONFIG_USB_OTG_PXA3XX)	+= pxa3xx_otg_pmic.o
obj-$(CONFIG_USB_OTG_PXA3XX)	+= pxa3xx_otg.o
obj-$(CONFIG_USB_PXA9XX_U2O)	+= pxa9xx_u2o.o
ifeq ($(CONFIG_USB_PXA3XX_UDC),y)
obj-$(CONFIG_USB_OTG_PXA3XX)	+= pxa300_otg.o
endif
