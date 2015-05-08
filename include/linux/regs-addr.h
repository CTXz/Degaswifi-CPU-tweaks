/*
 * interfaces for get base virtual or physical address of registers
 */
#ifndef _LINUX_REGS_ADDR_H
#define _LINUX_REGS_ADDR_H

#include <linux/types.h>

phys_addr_t get_apmu_base_pa(void);
void __iomem *get_apmu_base_va(void);
void __iomem *get_mpmu_base_va(void);
void __iomem *get_accu_base_va(void);
void __iomem *get_mccu_base_va(void);

#endif /* _LINUX_REGS_ADDR_H */
