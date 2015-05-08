/*
 * PXA CP load header
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef _PXA_CP_LOAD_H_
#define _PXA_CP_LOAD_H_

#include "linux_driver_types.h"

extern struct bus_type cpu_subsys;
extern void cp_releasecp(void);
extern void cp_holdcp(void);
extern bool cp_get_status(void);

extern uint32_t arbel_bin_phys_addr;

extern void (*watchdog_count_stop_fp)(void);

extern void __iomem *mpmu_base_addr;
/**
 * interface exported by kernel for disabling FC during hold/release CP
 */
extern void acquire_fc_mutex(void);
extern void release_fc_mutex(void);

#endif /* _PXA_CP_LOAD_H_ */
