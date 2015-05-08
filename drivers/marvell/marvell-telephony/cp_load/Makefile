ccflags-y += -Idrivers/marvell/marvell-telephony/shmem
cploaddev-objs := pxa_cp_load.o
ifeq ($(CONFIG_CPU_PXA988),y)
cploaddev-objs += pxa988_cp.o
endif
obj-$(CONFIG_MRVL_MMP_MODEM)	+= cploaddev.o
