#ifndef _POWER_DOMAIN_ISP_H
#define _POWER_DOMAIN_ISP_H
/*
 * gc, vpu, isp will access the same regsiter to pwr on/off,
 * add spinlock to protect the sequence
 */
extern spinlock_t gc_vpu_isp_pwr_lock;

int isp_pwr_ctrl(void *dev, int on);

#endif
