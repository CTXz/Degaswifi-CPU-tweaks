/*
 * Copyright (C) 2014 Marvell
 * Bill Zhou <zhoub@marvell.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 * Code common for all processor lines to return virtual or physical address of
 * register. To get virtual address, firstly we should retrieve the register
 * base address from device tree and then iomap to get the virtual address.
 * We retrieve the physical address and return it to the caller directly.
 * Interfaces function declaration in include/linux/regs-addr.h.
 */

#include <linux/of_address.h>

static void __iomem *iomap_register(const char *reg_name)
{
	void __iomem *reg_virt_addr;
	struct device_node *node;

	BUG_ON(!reg_name);
	node = of_find_compatible_node(NULL, NULL, reg_name);
	BUG_ON(!node);
	reg_virt_addr = of_iomap(node, 0);
	BUG_ON(!reg_virt_addr);

	return reg_virt_addr;
}

static phys_addr_t get_register_pa(const char *reg_name)
{
	phys_addr_t reg_phys_addr;
	struct device_node *node;
	u32 reg;

	BUG_ON(!reg_name);
	node = of_find_compatible_node(NULL, NULL, reg_name);
	BUG_ON(!node);
	of_property_read_u32(node, "reg", &reg);
	reg_phys_addr = reg;
	BUG_ON(!reg_phys_addr);

	return reg_phys_addr;
}

/* exported interfaces */
phys_addr_t get_apmu_base_pa(void)
{
	static phys_addr_t apmu_phys_addr;
	if (unlikely(!apmu_phys_addr))
		apmu_phys_addr = get_register_pa("mrvl,mmp-pmu-apmu");
	return apmu_phys_addr;
}
EXPORT_SYMBOL(get_apmu_base_pa);

void __iomem *get_apmu_base_va(void)
{
	static void __iomem *apmu_virt_addr;
	if (unlikely(!apmu_virt_addr))
		apmu_virt_addr = iomap_register("mrvl,mmp-pmu-apmu");
	return apmu_virt_addr;
}
EXPORT_SYMBOL(get_apmu_base_va);

void __iomem *get_mpmu_base_va(void)
{
	static void __iomem *mpmu_virt_addr;
	if (unlikely(!mpmu_virt_addr))
		mpmu_virt_addr = iomap_register("mrvl,mmp-pmu-mpmu");
	return mpmu_virt_addr;
}
EXPORT_SYMBOL(get_mpmu_base_va);

void __iomem *get_accu_base_va(void)
{
	static void __iomem *accu_virt_addr;
	if (unlikely(!accu_virt_addr))
		accu_virt_addr = iomap_register("mrvl,mmp-ccu-accu");
	return accu_virt_addr;
}
EXPORT_SYMBOL(get_accu_base_va);

void __iomem *get_mccu_base_va(void)
{
	static void __iomem *mccu_virt_addr;
	if (unlikely(!mccu_virt_addr))
		mccu_virt_addr = iomap_register("mrvl,mmp-ccu-mccu");
	return mccu_virt_addr;
}
EXPORT_SYMBOL(get_mccu_base_va);
