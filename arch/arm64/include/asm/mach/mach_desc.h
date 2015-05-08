/*
 *  arch/arm64/include/asm/mach/mach_desc.h
 *
 *  Copyright (C) 2000 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/of_fdt.h>
struct machine_desc {
	const char		*name;		/* architecture name	*/
	const char *const 	*dt_compat;	/* array of device tree
						 * 'compatible' strings	*/
	void                    (*reserve)(void);/* reserve mem blocks  */
	void			(*init_time)(void);
	void			(*init_machine)(void);
};

/*
 * Current machine - only accessible during boot.
 */
#ifdef CONFIG_ARCH_MMP
extern struct machine_desc mmp_machine_desc __initdata;
#endif

static inline struct machine_desc *get_machine_desc_compatible(void)
{
	unsigned long dt_root;
	dt_root = of_get_flat_dt_root();
#ifdef CONFIG_ARCH_MMP
	if (of_flat_dt_match(dt_root, mmp_machine_desc.dt_compat) > 0)
		return &mmp_machine_desc;
#endif
	return NULL;
}
