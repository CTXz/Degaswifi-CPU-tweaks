#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/of.h>

#include <mach/addr-map.h>
#include <mach/cputype.h>

/* Macros for eden Zx stepping */
#define APMU_GC_CLK_RES_CTRL	(0xcc)
#define APMU_GC_CLK_RES_CTRL2	(0x27c)
#define APMU_ISLD_GC_CTRL	(0x1b4)

#define GC2D_CLK_EN			(1u << 15)
#define GC2D_CLK_RST		(1u << 14)
#define GC2D_AXICLK_EN		(1u << 19)
#define GC2D_AXICLK_RST		(1u << 18)

#define GC3D_CLK_EN			(1u << 3)
#define GC3D_CLK_RST		(1u << 1)
#define GC3D_AXICLK_EN		(1u << 2)
#define GC3D_AXICLK_RST		(1u << 0)

#define GC_PWRUP(n)		((n & 3) << 9)
#define GC_PWRUP_MSK		GC_PWRUP(3)
#define GC_ISB			(1u << 8)
#define APMU_ISLD_GC_CMEM_DMMYCLK_EN	(1 << 4)

static DEFINE_SPINLOCK(gc_lock);
static int gc_fab_cnt;

void gc3d_pwr(unsigned int power_on)
{
	unsigned int regval;
	void __iomem *apmu_base;
	apmu_base = ioremap(AXI_PHYS_BASE + 0x82800, SZ_4K);
	if (apmu_base == NULL) {
		pr_err("error to ioremap APMU base\n");
		return;
	}

	regval = __raw_readl(apmu_base + APMU_GC_CLK_RES_CTRL);
	if (power_on) {
		if (regval & (GC_PWRUP_MSK | GC_ISB))
			return; /*Pwr is already on*/

		/* increase using count of shared fabric */
		spin_lock(&gc_lock);
		gc_fab_cnt++;
		spin_unlock(&gc_lock);

		/* 1. slow ramp */
		regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
		regval |= GC_PWRUP(1);
		writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);
		usleep_range(9000, 10000);
		/* 2. power up */
		regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
		regval |= GC_PWRUP(3);
		writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);
		usleep_range(9000, 10000);
		/* 3. disable GC isolation */
		regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
		regval |= GC_ISB;
		writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);
		udelay(10);
		/* 4. enable dummy clocks to SRAM */
		regval = readl(apmu_base + APMU_ISLD_GC_CTRL);
		regval |= APMU_ISLD_GC_CMEM_DMMYCLK_EN;
		writel(regval, apmu_base + APMU_ISLD_GC_CTRL);
		udelay(300);
		regval = readl(apmu_base + APMU_ISLD_GC_CTRL);
		regval &= ~APMU_ISLD_GC_CMEM_DMMYCLK_EN;
		writel(regval, apmu_base + APMU_ISLD_GC_CTRL);
		udelay(10);
		/* 5. enable peripheral clock */
		regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
		regval |= GC3D_CLK_EN;
		writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);
		udelay(10);
		/* 6. enable AXI clock */
		regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
		regval |= GC3D_AXICLK_EN;
		writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);
		udelay(10);
		/* 7. de-assert pheriperal reset */
		regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
		regval |= GC3D_CLK_RST;
		writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);
		udelay(10);
		/* 8. de-assert AXI reset */
		regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
		regval |= GC3D_AXICLK_RST;
		writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);
		udelay(10);

		/* 9. gate GC3D AXI clock if only GC3D is working */
		spin_lock(&gc_lock);
		if (1 == gc_fab_cnt) {
			regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
			regval &= ~GC3D_AXICLK_EN;
			writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);
		}
		spin_unlock(&gc_lock);

		/* 10. gate peripheral clock */
		regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
		regval &= ~GC3D_CLK_EN;
		writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);

		pr_debug("GC3D power on\n");
	} else {

		if ((regval & (GC_PWRUP_MSK | GC_ISB)) == 0)
			return; /*Pwr is already off*/

		/* decrease using count of shared fabric */
		spin_lock(&gc_lock);
		if (gc_fab_cnt)
			gc_fab_cnt--;
		spin_unlock(&gc_lock);

		/* 1. isolation */
		regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
		regval &= ~GC_ISB;
		writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);

		/* 2. pheriperal reset */
		regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
		regval &= ~GC3D_CLK_RST;
		writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);

		/* 3. GC3D AXI reset */
		spin_lock(&gc_lock);
		if (gc_fab_cnt == 0) {
			regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
			regval &= ~GC3D_AXICLK_RST;
			writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);
		}
		spin_unlock(&gc_lock);

		/* 4. make sure clock disabled*/
		regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
		regval &= ~GC3D_CLK_EN;
		writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);
		/* disable GC3D AXI clock */
		spin_lock(&gc_lock);
		if (gc_fab_cnt == 0) {
			regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
			regval &= ~GC3D_AXICLK_EN;
			writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);
		}
		spin_unlock(&gc_lock);

		/* 5. turn off power */
		regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
		regval &= ~GC_PWRUP_MSK;
		writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);
		pr_debug("GC3D power off\n");
	}
}
EXPORT_SYMBOL(gc3d_pwr);

static atomic_t gc2d_pwr_count;
void gc2d_pwr(unsigned int power_on)
{
	unsigned int regval;
	void __iomem *apmu_base;
	apmu_base = ioremap(AXI_PHYS_BASE + 0x82800, SZ_4K);
	if (apmu_base == NULL) {
		pr_err("error to ioremap APMU base\n");
		return;
	}

	if (power_on) {
		if (0 == atomic_read(&gc2d_pwr_count)) {
			/* increase using count of shared fabric */
			spin_lock(&gc_lock);
			gc_fab_cnt++;
			spin_unlock(&gc_lock);

			/* 1. enable peripheral clock */
			regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL2);
			regval |= GC2D_CLK_EN;
			writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL2);
			udelay(10);
			/* 2a. enable GC2D AXI clock */
			regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL2);
			regval |= GC2D_AXICLK_EN;
			writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL2);
			/*2b.enable GC3D AXI clock since it's a shared fabric*/
			regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
			regval |= GC3D_AXICLK_EN;
			writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);
			udelay(10);
			/* 3. de-assert pheriperal reset */
			regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL2);
			regval |= GC2D_CLK_RST;
			writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL2);
			udelay(10);
			/* 4a. de-assert GC2D AXI reset */
			regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL2);
			regval |= GC2D_AXICLK_RST;
			writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL2);
			/*4b.de-assert GC3D AXI rst since it's a shared fabric*/
			regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
			regval |= GC3D_AXICLK_RST;
			writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);
			udelay(10);

			/* 5a. gate GC2D AXI clock */
			regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL2);
			regval &= ~GC2D_AXICLK_EN;
			writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL2);

			/* 5b. gate GC3D AXI clock if only GC2D is working*/
			spin_lock(&gc_lock);
			if (1 == gc_fab_cnt) {
				regval = readl(apmu_base
						+ APMU_GC_CLK_RES_CTRL);
				regval &= ~GC3D_AXICLK_EN;
				writel(regval, apmu_base
						+ APMU_GC_CLK_RES_CTRL);
			}
			spin_unlock(&gc_lock);

			/* 6. gate peripheral clock */
			regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL2);
			regval &= ~GC2D_CLK_EN;
			writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL2);

			pr_debug("GC2D power on\n");
			atomic_inc(&gc2d_pwr_count);
		}
	} else
		pr_debug("GC2D power off\n");
}
EXPORT_SYMBOL(gc2d_pwr);

/*
 * assert clk reset/disable as GC clock may
 * be touched before kernel starts up
 */
void gc_reset(void)
{
	unsigned long regval;
	void __iomem *apmu_base;
	apmu_base = ioremap(AXI_PHYS_BASE + 0x82800, SZ_4K);
	if (apmu_base == NULL) {
		pr_err("error to ioremap APMU base\n");
		return;
	}

	/* 3d */
	regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL);
	regval &= ~(GC3D_AXICLK_RST | GC3D_AXICLK_EN |
			GC3D_CLK_RST | GC3D_CLK_EN);
	writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL);

	/* 2d */
	regval = readl(apmu_base + APMU_GC_CLK_RES_CTRL2);
	regval &= ~(GC2D_AXICLK_RST | GC2D_AXICLK_EN |
			GC2D_CLK_RST | GC2D_CLK_EN);
	writel(regval, apmu_base + APMU_GC_CLK_RES_CTRL2);
}
EXPORT_SYMBOL(gc_reset);
