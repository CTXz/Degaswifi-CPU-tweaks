/*
 * arch/arm/mach-mmp/power_domain_vpu.c
 *
 * Hantro video decoder engine UIO driver.
 *
 * Xiaolong Ye <yexl@marvell.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk/mmp.h>

#include <mach/addr-map.h>
#include <mach/cputype.h>

#define APMU_VPU_CLK_RES_CTRL	0xa4
#define APMU_PWR_CTRL_REG       0xd8
#define APMU_PWR_BLK_TMR_REG    0xdc
#define APMU_PWR_STATUS_REG     0xf0

#define VPU_HW_MODE	(0x1 << 19)
#define VPU_AUTO_PWR_ON	(0x1 << 2)
#define VPU_PWR_STAT	(0x1 << 2)

extern spinlock_t gc_vpu_isp_pwr_lock;

void coda7542_power_switch(int on)
{
	unsigned int val;
	int timeout = 2000;
	static struct clk *vpu;
	void __iomem *apmu_base;

	apmu_base = ioremap(AXI_PHYS_BASE + 0x82800, SZ_4K);
	if (apmu_base == NULL) {
		pr_err("error to ioremap APMU base\n");
		return;
	}

	if (!vpu)
		vpu = clk_get(NULL, "VPUCLK");

	/* HW mode power on */
	if (on) {
		clk_prepare_enable(vpu);
		/* set VPU HW on/off mode  */
		val = __raw_readl(apmu_base + APMU_VPU_CLK_RES_CTRL);
		val |= VPU_HW_MODE;
		__raw_writel(val, apmu_base + APMU_VPU_CLK_RES_CTRL);

		spin_lock(&gc_vpu_isp_pwr_lock);
		/* on1, on2, off timer */
		__raw_writel(0x20001fff, apmu_base + APMU_PWR_BLK_TMR_REG);

		/* VPU auto power on */
		val = __raw_readl(apmu_base + APMU_PWR_CTRL_REG);
		val |= VPU_AUTO_PWR_ON;
		__raw_writel(val, apmu_base + APMU_PWR_CTRL_REG);
		spin_unlock(&gc_vpu_isp_pwr_lock);
		/*
		 * VPU power on takes 316us, usleep_range(280,290) takes about
		 * 300~320us, so it can reduce the duty cycle.
		 */
		usleep_range(280, 290);

		/* polling VPU_PWR_STAT bit */
		while (!(__raw_readl(apmu_base + APMU_PWR_STATUS_REG) & VPU_PWR_STAT)) {
			udelay(1);
			timeout -= 1;
			if (timeout < 0) {
				pr_err("%s: VPU power on timeout\n", __func__);
				return;
			}
		}
		clk_disable_unprepare(vpu);
	/* HW mode power off */
	} else {
		spin_lock(&gc_vpu_isp_pwr_lock);
		/* VPU auto power off */
		val = __raw_readl(apmu_base + APMU_PWR_CTRL_REG);
		val &= ~VPU_AUTO_PWR_ON;
		__raw_writel(val, apmu_base + APMU_PWR_CTRL_REG);
		spin_unlock(&gc_vpu_isp_pwr_lock);
		/*
		 * VPU power off takes 23us, add a pre-delay to reduce the
		 * number of polling
		 */
		udelay(20);

		/* polling VPU_PWR_STAT bit */
		while ((__raw_readl(apmu_base + APMU_PWR_STATUS_REG) & VPU_PWR_STAT)) {
			udelay(1);
			timeout -= 1;
			if (timeout < 0) {
				pr_err("%s: VPU power off timeout\n", __func__);
				return;
			}
		}
	}
}
EXPORT_SYMBOL_GPL(coda7542_power_switch);
