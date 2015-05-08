#
# Makefile for the drivers of Marvell devices.
#
ifdef CONFIG_TZ_HYPERVISOR
obj-y				+= security/mmp-tzdd/src/
else
obj-y				+= security/mmp-geu/
endif
ifeq ($(BOARD_GFX_DRIVER_VERSION),)
  BOARD_GFX_DRIVER_VERSION := 4x
endif
obj-$(CONFIG_MRVL_GPU_GALCORE)	+= graphics/galcore_$(BOARD_GFX_DRIVER_VERSION)/
obj-$(CONFIG_MRVL_MMP_MODEM)	+= marvell-telephony/
