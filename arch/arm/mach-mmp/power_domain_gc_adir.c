#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/regs-addr.h>

#include <mach/regs-apmu.h>
#include <mach/regs-accu.h>

#define ACCU_GC3D_AHBRST	(1 << 3)
#define ACCU_GC3D_APBRST	(1 << 2)
#define ACCU_GC3D_RST		(1 << 0)  /* Reset Generation */

extern spinlock_t gc2d_lock;
extern spinlock_t gc3d_lock;

static DEFINE_SPINLOCK(gcpwr_reg_lock);
static DEFINE_SPINLOCK(gc2d_pwr_lock);
static DEFINE_SPINLOCK(gc3d_pwr_lock);
static DEFINE_SPINLOCK(gctp_pwr_lock);
static DEFINE_SPINLOCK(gc3d1_pwr_lock);
static DEFINE_SPINLOCK(gc3d2_pwr_lock);

static int gctop_on;
static bool gc2d_on;
static bool gc3d_on;
static bool gc3d1_on;
static bool gc3d2_on;

static void gc2d_clk_rst(u32 gc2d_reg)
{
	unsigned int regval;

	spin_lock(&gc2d_lock);
	/* reset gc2d, follow steps in app notes  */
	regval = readl_relaxed(get_accu_base_va() + gc2d_reg);
	regval |= ACCU_FNCLK | ACCU_APBCLK | ACCU_AHBCLK;
	regval &= ~(ACCU_RST | ACCU_APBRST | ACCU_AHBRST);
	writel_relaxed(regval, get_accu_base_va() + gc2d_reg);
	udelay(10);
	regval &= ~(ACCU_FNCLK | ACCU_APBCLK | ACCU_AHBCLK);
	regval &= ~(ACCU_RST | ACCU_APBRST | ACCU_AHBRST);
	writel_relaxed(regval, get_accu_base_va() + gc2d_reg);
	udelay(10);
	regval &= ~(ACCU_FNCLK | ACCU_APBCLK | ACCU_AHBCLK);
	regval |= ACCU_RST | ACCU_APBRST | ACCU_AHBRST;
	writel_relaxed(regval, get_accu_base_va() + gc2d_reg);
	spin_unlock(&gc2d_lock);
}

static void gc3d_clk_rst(u32 gc3d_reg)
{
	unsigned int regval;

	spin_lock(&gc3d_lock);
	/* enable ACCU level clk bits, enable shader & func clk */
	regval = readl_relaxed(get_accu_base_va() + ACCU_GC1_3D_CLK_CNTRL_REG);
	regval |= ACCU_GC3D_INTLCLK;
	writel_relaxed(regval, get_accu_base_va() + ACCU_GC1_3D_CLK_CNTRL_REG);

	regval = readl_relaxed(get_accu_base_va() + ACCU_GC2_3D_CLK_CNTRL_REG);
	regval |= ACCU_GC3D_INTLCLK;
	writel_relaxed(regval, get_accu_base_va() + ACCU_GC2_3D_CLK_CNTRL_REG);

	/* reset GC level clk bits */
	regval = readl_relaxed(get_accu_base_va() + gc3d_reg);
	regval |= ACCU_GC3D_FNCLK | ACCU_GC3D_APBCLK | ACCU_GC3D_AHBCLK;
	regval &= ~(ACCU_GC3D_RST | ACCU_GC3D_APBRST | ACCU_GC3D_AHBRST);
	writel_relaxed(regval, get_accu_base_va() + gc3d_reg);
	udelay(10);
	regval &= ~(ACCU_GC3D_FNCLK | ACCU_GC3D_APBCLK | ACCU_GC3D_AHBCLK);
	regval &= ~(ACCU_GC3D_RST | ACCU_GC3D_APBRST | ACCU_GC3D_AHBRST);
	writel_relaxed(regval, get_accu_base_va() + gc3d_reg);
	udelay(10);
	regval &= ~(ACCU_GC3D_FNCLK | ACCU_GC3D_APBCLK | ACCU_GC3D_AHBCLK);
	regval |= ACCU_GC3D_RST | ACCU_GC3D_APBRST | ACCU_GC3D_AHBRST;
	writel_relaxed(regval, get_accu_base_va() + gc3d_reg);
	spin_unlock(&gc3d_lock);
}

static int gc_top_pwr(unsigned int power_on)
{
	unsigned long regval;
	int timeout = 5000;
	spin_lock(&gctp_pwr_lock);
	if (power_on) {
		if (0 == gctop_on) {
			/* pwr on GC */
			spin_lock(&gcpwr_reg_lock);
			regval = readl_relaxed(get_apmu_base_va() + APMU_GPU_PWR_CTRL);
			regval |= GPU_SS_PWR_ON;
			writel_relaxed(regval, get_apmu_base_va() + APMU_GPU_PWR_CTRL);
			spin_unlock(&gcpwr_reg_lock);

			/* polling pwr status */
			while (!(readl_relaxed(get_apmu_base_va() + APMU_GPU_PWR_STATUS) & GPU_SS_PWR_ON)) {
				udelay(200);
				timeout -= 200;
				if (timeout < 0) {
					pr_err("%s: power on timeout\n", __func__);
					spin_unlock(&gctp_pwr_lock);
					return -EIO;
				}
			}
		}
		gctop_on++;
	} else {
		gctop_on--;
		if (0 == gctop_on) {
			/* pwr off GC */
			spin_lock(&gcpwr_reg_lock);
			regval = readl_relaxed(get_apmu_base_va() + APMU_GPU_PWR_CTRL);
			regval &= ~GPU_SS_PWR_ON;
			writel_relaxed(regval, get_apmu_base_va() + APMU_GPU_PWR_CTRL);
			spin_unlock(&gcpwr_reg_lock);
			udelay(20);
		}
	}

	spin_unlock(&gctp_pwr_lock);
	return 0;
}

void gc2d_pwr(unsigned int power_on)
{
	spin_lock(&gc2d_pwr_lock);
	if (gc2d_on == power_on) {
		spin_unlock(&gc2d_pwr_lock);
		return;
	}
	if (power_on) {
		if (gc_top_pwr(power_on)) {
			spin_unlock(&gc2d_pwr_lock);
			return;
		}
		gc2d_clk_rst(ACCU_GC_2D_CLK_CNTRL_REG);
	} else {
		if (gc_top_pwr(power_on)) {
			spin_unlock(&gc2d_pwr_lock);
			return;
		}
	}
	gc2d_on = power_on;
	spin_unlock(&gc2d_pwr_lock);
}
EXPORT_SYMBOL_GPL(gc2d_pwr);

/* the func is for debug purpose */
void gc3d_pwr(unsigned int power_on)
{
	unsigned long regval;
	int timeout = 5000;
	spin_lock(&gc3d_pwr_lock);
	if (gc3d_on == power_on) {
		spin_unlock(&gc3d_pwr_lock);
		return;
	}

	if (power_on) {
		if (gc_top_pwr(power_on)) {
			spin_unlock(&gc3d_pwr_lock);
			return;
		}
		/* pwr on GC */
		spin_lock(&gcpwr_reg_lock);
		regval = readl_relaxed(get_apmu_base_va() + APMU_GPU_PWR_CTRL);
		regval |= GPU1_3D_PWR_ON | GPU2_3D_PWR_ON;
		writel_relaxed(regval, get_apmu_base_va() + APMU_GPU_PWR_CTRL);
		spin_unlock(&gcpwr_reg_lock);

		/* polling pwr status */
		while ((!(readl_relaxed(get_apmu_base_va() + APMU_GPU_PWR_STATUS) & GPU2_3D_PWR_ON)) ||
			(!(readl_relaxed(get_apmu_base_va() + APMU_GPU_PWR_STATUS) & GPU1_3D_PWR_ON))) {
			udelay(200);
			timeout -= 200;
			if (timeout < 0) {
				pr_err("%s: power on timeout\n", __func__);
				spin_unlock(&gc3d_pwr_lock);
				return;
			}
		}
		gc3d_on = power_on;
		gc3d_clk_rst(ACCU_GC1_3D_CLK_CNTRL_REG);
		gc3d_clk_rst(ACCU_GC2_3D_CLK_CNTRL_REG);
	} else {
		/* pwr off GC */
		spin_lock(&gcpwr_reg_lock);
		regval = readl_relaxed(get_apmu_base_va() + APMU_GPU_PWR_CTRL);
		regval &= ~(GPU1_3D_PWR_ON | GPU2_3D_PWR_ON);
		writel_relaxed(regval, get_apmu_base_va() + APMU_GPU_PWR_CTRL);
		spin_unlock(&gcpwr_reg_lock);
		udelay(20);
		gc3d_on = power_on;

		if (gc_top_pwr(power_on)) {
			spin_unlock(&gc3d_pwr_lock);
			return;
		}
	}
	spin_unlock(&gc3d_pwr_lock);
}
EXPORT_SYMBOL_GPL(gc3d_pwr);

void gc3d1_pwr(unsigned int power_on)
{
	unsigned long regval;
	int timeout = 5000;
	spin_lock(&gc3d1_pwr_lock);
	if (gc3d1_on == power_on) {
		spin_unlock(&gc3d1_pwr_lock);
		return;
	}

	if (power_on) {
		if (gc_top_pwr(power_on)) {
			spin_unlock(&gc3d1_pwr_lock);
			return;
		}
		/* pwr on GC */
		spin_lock(&gcpwr_reg_lock);
		regval = readl_relaxed(get_apmu_base_va() + APMU_GPU_PWR_CTRL);
		regval |= GPU1_3D_PWR_ON;
		writel_relaxed(regval, get_apmu_base_va() + APMU_GPU_PWR_CTRL);
		spin_unlock(&gcpwr_reg_lock);

		/* polling pwr status */
		while (!(readl_relaxed(get_apmu_base_va() + APMU_GPU_PWR_STATUS) & GPU1_3D_PWR_ON)) {
			udelay(200);
			timeout -= 200;
			if (timeout < 0) {
				pr_err("%s: power on timeout\n", __func__);
				spin_unlock(&gc3d1_pwr_lock);
				return;
			}
		}
		gc3d1_on = power_on;
		gc3d_clk_rst(ACCU_GC1_3D_CLK_CNTRL_REG);
	} else {
		/* pwr off GC */
		spin_lock(&gcpwr_reg_lock);
		regval = readl_relaxed(get_apmu_base_va() + APMU_GPU_PWR_CTRL);
		regval &= ~(GPU1_3D_PWR_ON);
		writel_relaxed(regval, get_apmu_base_va() + APMU_GPU_PWR_CTRL);
		spin_unlock(&gcpwr_reg_lock);
		udelay(20);
		gc3d1_on = power_on;

		if (gc_top_pwr(power_on)) {
			spin_unlock(&gc3d1_pwr_lock);
			return;
		}
	}
	spin_unlock(&gc3d1_pwr_lock);
}
EXPORT_SYMBOL_GPL(gc3d1_pwr);

void gc3d2_pwr(unsigned int power_on)
{
	unsigned long regval;
	int timeout = 5000;
	spin_lock(&gc3d2_pwr_lock);
	if (gc3d2_on == power_on) {
		spin_unlock(&gc3d2_pwr_lock);
		return;
	}

	if (power_on) {
		if (gc_top_pwr(power_on)) {
			spin_unlock(&gc3d2_pwr_lock);
			return;
		}
		/* pwr on GC */
		spin_lock(&gcpwr_reg_lock);
		regval = readl_relaxed(get_apmu_base_va() + APMU_GPU_PWR_CTRL);
		regval |= GPU2_3D_PWR_ON;
		writel_relaxed(regval, get_apmu_base_va() + APMU_GPU_PWR_CTRL);
		spin_unlock(&gcpwr_reg_lock);

		/* polling pwr status */
		while (!(readl_relaxed(get_apmu_base_va() + APMU_GPU_PWR_STATUS) & GPU2_3D_PWR_ON)) {
			udelay(200);
			timeout -= 200;
			if (timeout < 0) {
				pr_err("%s: power on timeout\n", __func__);
				spin_unlock(&gc3d2_pwr_lock);
				return;
			}
		}
		gc3d2_on = power_on;
		gc3d_clk_rst(ACCU_GC2_3D_CLK_CNTRL_REG);
	} else {
		spin_lock(&gcpwr_reg_lock);
		/* pwr off GC */
		regval = readl_relaxed(get_apmu_base_va() + APMU_GPU_PWR_CTRL);
		regval &= ~(GPU2_3D_PWR_ON);
		writel_relaxed(regval, get_apmu_base_va() + APMU_GPU_PWR_CTRL);
		spin_unlock(&gcpwr_reg_lock);
		udelay(20);
		gc3d2_on = power_on;

		if (gc_top_pwr(power_on)) {
			spin_unlock(&gc3d2_pwr_lock);
			return;
		}
	}
	spin_unlock(&gc3d2_pwr_lock);
}
EXPORT_SYMBOL_GPL(gc3d2_pwr);
