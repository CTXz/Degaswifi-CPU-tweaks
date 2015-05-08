dtb-$(CONFIG_ARCH_VEXPRESS) += rtsm_ve-aemv8a.dtb foundation-v8.dtb
dtb-$(CONFIG_ARCH_MMP) += edenfpga.dtb edenfpga-icu.dtb edenfpga-psci.dtb \
	edenconcord.dtb edenconcord-discrete.dtb
dtb-$(CONFIG_ARCH_XGENE) += apm-mustang.dtb

targets += dtbs
targets += $(dtb-y)

dtbs: $(addprefix $(obj)/, $(dtb-y))

clean-files := *.dtb
