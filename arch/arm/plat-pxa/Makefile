#
# Makefile for code common across different PXA processor families
#

obj-y	:= dma.o debugfs.o

obj-$(CONFIG_PXA3xx)		+= mfp.o
obj-$(CONFIG_ARCH_MMP)		+= mfp.o
obj-$(CONFIG_DUMP_REGS)		+= dump_regs.o
obj-$(CONFIG_PXA_SSP)		+= ssp.o
obj-$(CONFIG_PXA_PANIC_FLUSH)	+= panic.o
