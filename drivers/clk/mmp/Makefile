#
# Makefile for mmp specific clk
#

obj-y += clk-apbc.o clk-apmu.o clk-frac.o

obj-$(CONFIG_CPU_PXA168) += clk-pxa168.o
obj-$(CONFIG_CPU_PXA910) += clk-pxa910.o
obj-$(CONFIG_CPU_MMP2) += clk-mmp2.o
obj-$(CONFIG_CPU_PXA988) += clk-pxa988.o clk-core.o clk-pll.o clk-peri.o clk-disp.o simple_dip.o
obj-$(CONFIG_CPU_EDEN) += clk-eden.o clk-pll-eden.o clk-disp.o clk-peri.o clk-core-eden.o \
clk-audio-eden.o clk-audio-map.o
obj-$(CONFIG_CPU_PXA1986) += clk-pxa1986.o clk-disp.o

ifeq ($(CONFIG_PXA_DVFS), y)
obj-y += dvfs.o
obj-$(CONFIG_CPU_PXA988) += dvfs-dvc.o dvfs-pxa1088.o dvfs-pxa1L88.o dvfs-pxa1u88.o
obj-$(CONFIG_CPU_EDEN) += dvfs-pxa1928.o
endif

obj-$(CONFIG_PXA_DVFS) += dvfs.o
obj-$(CONFIG_DEBUG_FS) += clk-dcstat.o
