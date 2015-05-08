#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clkdev.h>
#include <linux/clk.h>
#include <linux/features.h>

#include <mach/addr-map.h>
#include <mach/cputype.h>

#define APMU_GC		0xcc
#define APMU_GC_2D	0xf4
#define APMU_PWR_CTRL_REG	0xd8
#define APMU_PWR_BLK_TMR_REG	0xdc
#define APMU_PWR_STATUS_REG	0xf0
/*
 * gc, vpu, isp will access the same regsiter to pwr on/off,
 * add spinlock to protect the sequence
 */
DEFINE_SPINLOCK(gc_vpu_isp_pwr_lock);

/* used to protect GC3D/2D power sequence */
static DEFINE_SPINLOCK(gc_pwr_lock);
static DEFINE_SPINLOCK(gc2d_pwr_lock);

/* record reference count on gc/gc2d power on/off */
static int gc_pwr_refcnt;
static int gc2d_pwr_refcnt;

/* GC power control */
#define GC_USE_HW_PWRCTRL	1
#define GC_AUTO_PWR_ON		(0x1 << 0)
#define GC2D_AUTO_PWR_ON	(0x1 << 6)

#define GC_CLK_EN	\
	((0x1 << 3) | (0x1 << 4) | (0x1 << 5))

#define GC_ACLK_RST	(0x1 << 0)
#define GC_FCLK_RST	(0x1 << 1)
#define GC_HCLK_RST	(0x1 << 2)
#define GC_CLK_RST	\
	(GC_ACLK_RST | GC_FCLK_RST | GC_HCLK_RST)
#define GC_SHADER_CLK_RST	(0x1 << 24)

#define GC_ISOB		(0x1 << 8)
#define GC_PWRON1	(0x1 << 9)
#define GC_PWRON2	(0x1 << 10)
#define GC_HWMODE	(0x1 << 11)

#define GC_FCLK_SEL_MASK	(0x3 << 6)
#define GC_ACLK_SEL_MASK	(0x3 << 20)
#define GC_FCLK_DIV_MASK	(0x7 << 12)
#define GC_ACLK_DIV_MASK	(0x7 << 17)
#define GC_FCLK_REQ		(0x1 << 15)
#define GC_ACLK_REQ		(0x1 << 16)

#define GC_CLK_SEL_WIDTH	(2)
#define GC_CLK_DIV_WIDTH	(3)
#define GC_FCLK_SEL_SHIFT	(6)
#define GC_ACLK_SEL_SHIFT	(20)
#define GC_FCLK_DIV_SHIFT	(12)
#define GC_ACLK_DIV_SHIFT	(17)

#define GC_REG_WRITE(val)	{	\
	__raw_writel(val, apmu_base + APMU_GC);	\
}

#define GC_2D_REG_WRITE(val)	{	\
	__raw_writel(val, apmu_base + APMU_GC_2D);	\
}

static void __iomem *apmu_base;

static void gc_pwr_on(void)
{
	unsigned int val;
	int timeout = 5000;
	unsigned int __attribute__ ((unused)) val_2d;

	if (apmu_base == NULL) {
		apmu_base = ioremap(AXI_PHYS_BASE + 0x82800, SZ_4K);
		if (apmu_base == NULL) {
			pr_err("error to ioremap APMU base\n");
			return;
		}
	}
	val = __raw_readl(apmu_base + APMU_GC);
	if (cpu_is_pxa1088())
		val_2d = __raw_readl(apmu_base + APMU_GC_2D);

#ifdef GC_USE_HW_PWRCTRL
	/* enable hw mode */
	val |= GC_HWMODE;
	GC_REG_WRITE(val);

	spin_lock(&gc_vpu_isp_pwr_lock);
	/* set PWR_BLK_TMR_REG to recommend value */
	__raw_writel(0x20001FFF, apmu_base + APMU_PWR_BLK_TMR_REG);

	/* pwr on GC */
	val = __raw_readl(apmu_base + APMU_PWR_CTRL_REG);
	val |= GC_AUTO_PWR_ON;
	__raw_writel(val, apmu_base + APMU_PWR_CTRL_REG);
	spin_unlock(&gc_vpu_isp_pwr_lock);

	/* polling pwr status */
	while (!(__raw_readl(apmu_base + APMU_PWR_STATUS_REG) & GC_AUTO_PWR_ON)) {
		udelay(200);
		timeout -= 200;
		if (timeout < 0) {
			pr_err("%s: power on timeout\n", __func__);
			return;
		}
	}
#else
	/* enable bus and function clock  */
	val |= GC_CLK_EN;
	GC_REG_WRITE(val);
	if (cpu_is_pxa1088()) {
		val_2d |= GC_CLK_EN;
		GC_2D_REG_WRITE(val_2d);
	}
	/* enable power_on1, wait at least 200us */
	val |= GC_PWRON1;
	GC_REG_WRITE(val);
	udelay(200);

	/* enable power_on2 */
	val |= GC_PWRON2;
	GC_REG_WRITE(val);

	/* fRst release */
	val |= GC_FCLK_RST;
	if (cpu_is_pxa1L88())
		val |= GC_SHADER_CLK_RST;
	GC_REG_WRITE(val);
	if (cpu_is_pxa1088()) {
		val_2d |= GC_FCLK_RST;
		GC_2D_REG_WRITE(val_2d);
	}
	udelay(100);

	/* aRst hRst release at least 48 cycles later than fRst */
	val |= (GC_ACLK_RST | GC_HCLK_RST);
	GC_REG_WRITE(val);
	if (cpu_is_pxa1088()) {
		val_2d |= (GC_ACLK_RST | GC_HCLK_RST);
		GC_2D_REG_WRITE(val_2d);
	}
	/* disable isolation */
	val |= GC_ISOB;
	GC_REG_WRITE(val);
#endif
}

static void gc_pwr_off(void)
{
	unsigned int val = __raw_readl(apmu_base + APMU_GC);
	int timeout = 5000;
	unsigned int __attribute__ ((unused)) val_2d = __raw_readl(apmu_base + APMU_GC_2D);

#ifdef GC_USE_HW_PWRCTRL
	spin_lock(&gc_vpu_isp_pwr_lock);
	/* pwr on GC */
	val = __raw_readl(apmu_base + APMU_PWR_CTRL_REG);
	val &= ~GC_AUTO_PWR_ON;
	__raw_writel(val, apmu_base + APMU_PWR_CTRL_REG);
	spin_unlock(&gc_vpu_isp_pwr_lock);

	/* polling pwr status */
	while ((__raw_readl(apmu_base + APMU_PWR_STATUS_REG) & GC_AUTO_PWR_ON)) {
		udelay(200);
		timeout -= 200;
		if (timeout < 0) {
			pr_err("%s: power off timeout\n", __func__);
			return;
		}
	}
#else
	/* enable isolation */
	val &= ~GC_ISOB;
	GC_REG_WRITE(val);

	/* disable power_on2 */
	val &= ~GC_PWRON2;
	GC_REG_WRITE(val);

	/* disable power_on1 */
	val &= ~GC_PWRON1;
	GC_REG_WRITE(val);

	/* fRst aRst hRst */
	val &= ~(GC_CLK_RST | GC_CLK_EN);

	if (cpu_is_pxa1L88())
		val &= ~GC_SHADER_CLK_RST;

	GC_REG_WRITE(val);

	if (cpu_is_pxa1088()) {
		val_2d &= ~(GC_CLK_RST | GC_CLK_EN);
		GC_2D_REG_WRITE(val_2d);
	}
	udelay(100);

#endif
}

static void gc2d_pwr_on(void)
{
	int timeout = 5000;
	unsigned int val;

	if (apmu_base == NULL) {
		apmu_base = ioremap(AXI_PHYS_BASE + 0x82800, SZ_4K);
		if (apmu_base == NULL) {
			pr_err("error to ioremap APMU base\n");
			return;
		}
	}

	val = __raw_readl(apmu_base + APMU_GC_2D);

#ifdef GC_USE_HW_PWRCTRL
	/* enable hw mode */
	val |= GC_HWMODE;
	GC_2D_REG_WRITE(val);

	spin_lock(&gc_vpu_isp_pwr_lock);
	/* set PWR_BLK_TMR_REG to recommend value */
	__raw_writel(0x20001FFF, apmu_base + APMU_PWR_BLK_TMR_REG);

	/* pwr on GC2D */
	val = __raw_readl(apmu_base + APMU_PWR_CTRL_REG);
	val |= GC2D_AUTO_PWR_ON;
	__raw_writel(val, apmu_base + APMU_PWR_CTRL_REG);
	spin_unlock(&gc_vpu_isp_pwr_lock);

	/* polling pwr status */
	while (!(__raw_readl(apmu_base + APMU_PWR_STATUS_REG) & GC2D_AUTO_PWR_ON)) {
		udelay(200);
		timeout -= 200;
		if (timeout < 0) {
			pr_err("%s: power on timeout\n", __func__);
			return;
		}
	}
#else
	/* enable bus and function clock  */
	val |= GC_CLK_EN;
	GC_2D_REG_WRITE(val);

	/* enable power_on1, wait at least 200us */
	val |= GC_PWRON1;
	GC2D_REG_WRITE(val);
	udelay(200);

	/* enable power_on2 */
	val |= GC_PWRON2;
	GC_2D_REG_WRITE(val);

	/* fRst release */
	val |= GC_FCLK_RST;
	GC_2D_REG_WRITE(val);

	udelay(100);

	/* aRst hRst release at least 48 cycles later than fRst */
	val |= (GC_ACLK_RST | GC_HCLK_RST);
	GC_2D_REG_WRITE(val);

	/* disable isolation */
	val |= GC_ISOB;
	GC_2D_REG_WRITE(val);
#endif
}

static void gc2d_pwr_off(void)
{
	int timeout = 5000;
	unsigned int val = __raw_readl(apmu_base + APMU_GC_2D);

#ifdef GC_USE_HW_PWRCTRL
	/* pwr on GC */
	spin_lock(&gc_vpu_isp_pwr_lock);
	val = __raw_readl(apmu_base + APMU_PWR_CTRL_REG);
	val &= ~GC2D_AUTO_PWR_ON;
	__raw_writel(val, apmu_base + APMU_PWR_CTRL_REG);
	spin_unlock(&gc_vpu_isp_pwr_lock);

	/* polling pwr status */
	while ((__raw_readl(apmu_base + APMU_PWR_STATUS_REG) & GC2D_AUTO_PWR_ON)) {
		udelay(200);
		timeout -= 200;
		if (timeout < 0) {
			pr_err("%s: power off timeout\n", __func__);
			return;
		}
	}
#else
	/* enable isolation */
	val &= ~GC_ISOB;
	GC_2D_REG_WRITE(val);

	/* disable power_on2 */
	val &= ~GC_PWRON2;
	GC_2D_REG_WRITE(val);

	/* disable power_on1 */
	val &= ~GC_PWRON1;
	GC_2D_REG_WRITE(val);

	/* fRst aRst hRst */
	val &= ~(GC_CLK_RST | GC_CLK_EN);
	GC_2D_REG_WRITE(val);
	udelay(100);
#endif
}

void gc2d_pwr(int power_on)
{
	static struct clk *gc2d_clk;
	if (!gc2d_clk)
		gc2d_clk = clk_get(NULL, "GC2DCLK");
	if (power_on)
		clk_prepare_enable(gc2d_clk);

	spin_lock(&gc2d_pwr_lock);

	if (power_on) {
		if (gc2d_pwr_refcnt == 0)
			gc2d_pwr_on();
		gc2d_pwr_refcnt++;
	} else {
		/*
		 * If try to power off gc2d, but gc2d_pwr_refcnt is 0,
		 * print warning and return.
		 */
		if (WARN_ON(gc2d_pwr_refcnt == 0))
			goto out;

		if (--gc2d_pwr_refcnt > 0)
			goto out;

		gc2d_pwr_off();
	}

out:
	spin_unlock(&gc2d_pwr_lock);

	if (power_on)
		clk_disable_unprepare(gc2d_clk);
}
EXPORT_SYMBOL(gc2d_pwr);


void gc_pwr(int power_on)
{
	static struct clk *gc_clk;
	if (!gc_clk)
		gc_clk = clk_get(NULL, "GCCLK");

	if (power_on) {
		clk_prepare_enable(gc_clk);
		if (has_feat_gc2d_on_gc_on())
			gc2d_pwr(power_on);
	}
	spin_lock(&gc_pwr_lock);

	if (power_on) {
		if (gc_pwr_refcnt == 0)
			gc_pwr_on();
		gc_pwr_refcnt++;
	} else {
		/*
		 * If try to power off gc, but gc_pwr_refcnt is 0,
		 * print warning and return.
		 */
		if (WARN_ON(gc_pwr_refcnt == 0))
			goto out;

		if (--gc_pwr_refcnt > 0)
			goto out;

		gc_pwr_off();
	}

out:
	spin_unlock(&gc_pwr_lock);

	if (power_on)
		clk_disable_unprepare(gc_clk);

	if (has_feat_gc2d_on_gc_on() && !power_on)
		gc2d_pwr(power_on);

}
EXPORT_SYMBOL(gc_pwr);

