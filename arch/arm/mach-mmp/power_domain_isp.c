#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <mach/addr-map.h>

#include <mach/power_domain_isp.h>

#define APMU_ISPDXO	0x038
#define APMU_PWR_CTRL_REG       0x0d8
#define APMU_PWR_BLK_TMR_REG    0x0dc
#define APMU_PWR_STATUS_REG     0x0f0

#define ISP_HW_MODE         (0x1 << 15)
#define ISP_AUTO_PWR_ON     (0x1 << 4)
#define ISP_PWR_STAT        (0x1 << 4)


static int isp_pwr_refcnt = 0;
static void __iomem *apmu_base;
#define ISP_HW_MODE         (0x1 << 15)
#define ISP_AUTO_PWR_ON     (0x1 << 4)
#define APMU_CCIC_DBG		0x088
#define ISP_PWR_STAT        (0x1 << 4)
#define ISP_CLK_RST         ((1 << 0) | (1 << 8) | (1 << 10))
#define ISP_CLK_EN          ((1 << 1) | (1 << 9) | (1 << 11))

int isp_pwr_ctrl(void *dev,  int on)
{
	unsigned int val;
	int timeout = 5000;
	if (apmu_base == NULL) {
		apmu_base = ioremap(AXI_PHYS_BASE + 0x82800, SZ_4K);
		if (apmu_base == NULL) {
			pr_err("error to ioremap APMU base\n");
			return -EINVAL;
		}
	}
	/*  HW mode power on/off*/
	if (on) {
		if (isp_pwr_refcnt++ > 0)
			return 0;
		/* set isp HW mode*/
		val = __raw_readl(apmu_base + APMU_ISPDXO);
		val |= ISP_HW_MODE;
		__raw_writel(val, apmu_base + APMU_ISPDXO);
		spin_lock(&gc_vpu_isp_pwr_lock);
		/*  on1, on2, off timer */
		__raw_writel(0x20001fff, apmu_base + APMU_PWR_BLK_TMR_REG);

		/*  isp auto power on */
		val = __raw_readl(apmu_base + APMU_PWR_CTRL_REG);
		val |= ISP_AUTO_PWR_ON;
		__raw_writel(val, apmu_base + APMU_PWR_CTRL_REG);
		spin_unlock(&gc_vpu_isp_pwr_lock);
		__raw_writel(0x06000000 |
				__raw_readl(apmu_base + APMU_CCIC_DBG),
				apmu_base + APMU_CCIC_DBG);
		/*  polling ISP_PWR_STAT bit */
		while (!(__raw_readl(apmu_base + APMU_PWR_STATUS_REG)
					& ISP_PWR_STAT)) {
			udelay(500);
			timeout -= 500;
			if (timeout < 0) {
				pr_err("%s: isp power on timeout\n", __func__);
				return -ENODEV;
			}
		}

	} else {
               if (WARN_ON(isp_pwr_refcnt == 0))
                       return -EINVAL;

               if (--isp_pwr_refcnt > 0)
                       return 0;
		spin_lock(&gc_vpu_isp_pwr_lock);
		/*  isp auto power off */
		val = __raw_readl(apmu_base + APMU_PWR_CTRL_REG);
		val &= ~ISP_AUTO_PWR_ON;
		__raw_writel(val, apmu_base + APMU_PWR_CTRL_REG);
		spin_unlock(&gc_vpu_isp_pwr_lock);
		/*  polling ISP_PWR_STAT bit */
		while ((__raw_readl(apmu_base + APMU_PWR_STATUS_REG)
					& ISP_PWR_STAT)) {
			udelay(500);
			timeout -= 500;
			if (timeout < 0) {
				pr_err("%s: ISP power off timeout\n", __func__);
				return -ENODEV;
			}
		}
	}

	return 0;
}

EXPORT_SYMBOL_GPL(isp_pwr_ctrl);
