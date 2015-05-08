#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/regs-addr.h>

/* GC 3D/2D pwr function */
#define APMU_ISLD_GC3D_PWRCTRL	0x20c
#define APMU_ISLD_GC_CTRL	0x1b4
#define APMU_ISLD_GC2D_CTRL	0x1b8
#define APMU_GC3D_CKGT		0x3c
#define APMU_GC3D_CLKCTRL	0x174
#define APMU_GC3D_RSTCTRL	0x170

#define APMU_ISLD_GC2D_PWRCTRL	0x210
#define APMU_FABRIC1_CKGT	0x64
#define APMU_GC2D_CLKCTRL	0x17c
#define APMU_GC2D_RSTCTRL	0x178

#define HWMODE_EN	(1u << 0)
#define PWRUP		(1u << 1)
#define PWR_STATUS	(1u << 4)
#define REDUN_STATUS	(1u << 5)
#define INT_CLR		(1u << 6)
#define INT_MASK	(1u << 7)
#define INT_STATUS	(1u << 8)

#define GC2D_ACLK_EN	(1u << 0)
#define GC2D_CLK1X_DIV_MASK	(7 << 8)
#define GC2D_CLK1X_DIV_SHIFT	8

#define GC2D_CLK1X_CLKSRC_SEL_MASK	(7 << 12)
#define GC2D_CLK1X_CLKSRC_SEL_SHIFT	12
#define	GC2D_HCLK_EN	(1u << 24)
#define	GC2D_UPDATE_RTCWTC	(1u << 31)

#define GC2D_ACLK_RST	(1u << 0)
#define GC2D_CLK1X_RST	(1u << 1)
#define GC2D_HCLK_RST	(1u << 2)
#define GC2D_PWRON_RST	(1u << 7)
#define GC2D_FC_EN	(1u << 9)

#define X2H_CKGT_DISABLE	(1u << 0)

#define GC3D_ACLK_RST	(1u << 0)
#define GC3D_CLK1X_RST	(1u << 1)
#define GC3D_HCLK_RST	(1u << 2)
#define GC3D_PWRON_RST	(1u << 7)
#define GC3D_FC_EN	(1u << 9)

#define GC3D_ALCK_DIV_MASK	(7 << 0)
#define GC3D_ACLK_DIV_SHIFT	0

#define GC3D_ACLKSRC_SEL_MASK	(7 << 4)
#define GC3D_ACLKSRC_SEL_SHIFT	4

#define GC3D_CLK1X_DIV_MASK	(7 << 8)
#define GC3D_CLK1X_DIV_SHIFT	8

#define GC3D_CLK1X_CLKSRC_SEL_MASK	(7 << 12)
#define GC3D_CLK1X_CLKSRC_SEL_SHIFT	12

#define GC3D_CLKSH_DIV_MASK	(7 << 16)
#define GC3D_CLKSH_DIV_SHIFT	16

#define	GC3D_CLKSH_CLKSRC_SEL_MASK	(7 << 20)
#define GC3D_CLKSH_CLKSRC_SEL_SHIFT	20

#define GC3D_HCLK_EN	(1u << 24)
#define GC3D_UPDATE_RTCWTC	(1u << 31)

#define GC3D_FAB_CKGT_DISABLE	(1u << 1)
#define MC_P4_CKGT_DISABLE	(1u << 0)

#define CMEM_DMMYCLK_EN		(1u << 4)

/* record reference count on gc2d/gc3d power on/off */
static int gc2d_pwr_refcnt;
static int gc3d_pwr_refcnt;

/* used to protect gc2d/gc3d power sequence */
static DEFINE_SPINLOCK(gc2d_pwr_lock);
static DEFINE_SPINLOCK(gc3d_pwr_lock);

static void gc2d_on(void)
{
	unsigned int regval;
	unsigned int timeout;
	void __iomem *apmu_base;

	apmu_base = get_apmu_base_va();

	/* Clock setup procedure. */
	/* 1. Enable Dummy Clocks to SRAMs. */
	regval = readl(apmu_base + APMU_ISLD_GC2D_CTRL);
	regval |= CMEM_DMMYCLK_EN;
	writel(regval, apmu_base + APMU_ISLD_GC2D_CTRL);
	udelay(1);

	/* 2. Disable Dummy Clocks to SRAMs. */
	regval = readl(apmu_base + APMU_ISLD_GC2D_CTRL);
	regval &= ~CMEM_DMMYCLK_EN;
	writel(regval, apmu_base + APMU_ISLD_GC2D_CTRL);
	udelay(1);

	/* 3. Enable GC2D HCLK clock. */
	regval = readl(apmu_base + APMU_GC2D_CLKCTRL);
	regval |= GC2D_HCLK_EN;
	writel(regval, apmu_base + APMU_GC2D_CLKCTRL);

	/* 4. Enalbe GC2D CLK1X clock. */
	regval = readl(apmu_base + APMU_GC2D_CLKCTRL);
	regval &= ~GC2D_CLK1X_DIV_MASK;
	regval |= (2 << GC2D_CLK1X_DIV_SHIFT);
	writel(regval, apmu_base + APMU_GC2D_CLKCTRL);

	/* 5. Enable GC2D ACLK clock. */
	regval = readl(apmu_base + APMU_GC2D_CLKCTRL);
	regval |= GC2D_ACLK_EN;
	writel(regval, apmu_base + APMU_GC2D_CLKCTRL);

	/* 6. Enable Frequency change. */
	regval = readl(apmu_base + APMU_GC2D_RSTCTRL);
	regval |= GC2D_FC_EN;
	writel(regval, apmu_base + APMU_GC2D_RSTCTRL);

	/* 7. Disable fabric1 x2h dynamic clock gating. */
	regval = readl(apmu_base + APMU_FABRIC1_CKGT);
	regval |= X2H_CKGT_DISABLE;
	writel(regval, apmu_base + APMU_FABRIC1_CKGT);

	/* Power up procedure. */
	/* 1. Enable HWMODE to power up the island. */
	regval = readl(apmu_base + APMU_ISLD_GC2D_PWRCTRL);
	regval |= HWMODE_EN;
	writel(regval, apmu_base + APMU_ISLD_GC2D_PWRCTRL);

	/* 2. Remove interrupt Mask. */
	regval = readl(apmu_base + APMU_ISLD_GC2D_PWRCTRL);
	regval &= ~INT_MASK;
	writel(regval, apmu_base + APMU_ISLD_GC2D_PWRCTRL);

	udelay(1);

	/* 3. Power up the island. */
	regval = readl(apmu_base + APMU_ISLD_GC2D_PWRCTRL);
	regval |= PWRUP;
	writel(regval, apmu_base + APMU_ISLD_GC2D_PWRCTRL);

	/* 4. Wait for island to be powered up. */
	timeout = 1000;
	do {
		if (--timeout == 0) {
			WARN(1, "GC2D: power up timeout!\n");
			return;
		}
		udelay(10);
		regval = readl(apmu_base + APMU_ISLD_GC2D_PWRCTRL);
	} while (!(regval & PWR_STATUS));

	/*
	 * 5. Wait for active interrupt pending, indicating
	 * completion of island power up sequence.
	 * */
	timeout = 1000;
	do {
		if (--timeout == 0) {
			WARN(1, "GC2D: active interrupt pending!\n");
			return;
		}
		udelay(10);
		regval = readl(apmu_base + APMU_ISLD_GC2D_PWRCTRL);
	} while (!(regval & INT_STATUS));

	/*
	 * 6. The island is now powered up. Clear the interrupt and
	 *    set the interrupt masks.
	 */
	regval = readl(apmu_base + APMU_ISLD_GC2D_PWRCTRL);
	regval |= INT_CLR | INT_MASK;
	writel(regval, apmu_base + APMU_ISLD_GC2D_PWRCTRL);


	/* Reset Procedure. */
	/* 1. FIXME De-assert GC2D PWRON Reset. */
	regval = readl(apmu_base + APMU_GC2D_RSTCTRL);
	regval |= GC2D_PWRON_RST;
	writel(regval, apmu_base + APMU_GC2D_RSTCTRL);

	/* 2. De-assert GC2D ACLK Reset, CLK1X Reset and HCLK Reset. */
	regval = readl(apmu_base + APMU_GC2D_RSTCTRL);
	regval |= (GC2D_ACLK_RST | GC2D_CLK1X_RST | GC2D_HCLK_RST);
	writel(regval, apmu_base + APMU_GC2D_RSTCTRL);

	/* 3. Wait for 1000ns. */
	udelay(1);

	/* 4. Enable fabric1 x2h dynamic clock gating. */
	regval = readl(apmu_base + APMU_FABRIC1_CKGT);
	regval &= ~(X2H_CKGT_DISABLE);
	writel(regval, apmu_base + APMU_FABRIC1_CKGT);

}

static void gc2d_off(void)
{
	unsigned int regval;
	unsigned int timeout;
	void __iomem *apmu_base;

	apmu_base = get_apmu_base_va();

	/* 1. Assert GC2D ACLK/CLK1X/HCLK Reset. */
	regval = readl(apmu_base + APMU_GC2D_RSTCTRL);
	regval &= ~(GC2D_ACLK_RST | GC2D_CLK1X_RST | GC2D_HCLK_RST);
	writel(regval, apmu_base + APMU_GC2D_RSTCTRL);

	/*
	 * 2. Wait a minimum of 32 cycles of slowest clock
	 * (AXI clock, GC2D CLK1X, GC2D HCLK) after reset has
	 * been asserted.
	 * */
	udelay(10); /* FIXME */

	/* 3. Disable GC2D HCLK clock. */
	regval = readl(apmu_base + APMU_GC2D_CLKCTRL);
	regval &= ~GC2D_HCLK_EN;
	writel(regval, apmu_base + APMU_GC2D_CLKCTRL);

	/* 4. Disable GC2D CLK1X clock. */
	regval = readl(apmu_base + APMU_GC2D_CLKCTRL);
	regval &= ~GC2D_CLK1X_DIV_MASK;
	writel(regval, apmu_base + APMU_GC2D_CLKCTRL);

	/* 5. Disable GC2D AXI clock. */
	regval = readl(apmu_base + APMU_GC2D_CLKCTRL);
	regval &= ~GC2D_ACLK_EN;
	writel(regval, apmu_base + APMU_GC2D_CLKCTRL);

	/* 6. Enable HWMODE to power down the island. */
	regval = readl(apmu_base + APMU_ISLD_GC2D_PWRCTRL);
	regval |= HWMODE_EN;
	writel(regval, apmu_base + APMU_ISLD_GC2D_PWRCTRL);

	/* 7. Power down the island. */
	regval = readl(apmu_base + APMU_ISLD_GC2D_PWRCTRL);
	regval &= ~PWRUP;
	writel(regval, apmu_base + APMU_ISLD_GC2D_PWRCTRL);

	/* 8. Wait for island to be powered down. */
	timeout = 1000;
	do {
		if (--timeout == 0) {
			WARN(1, "GC3D: power down timeout!\n");
			return;
		}
		udelay(10);
		regval = readl(apmu_base + APMU_ISLD_GC2D_PWRCTRL);
	} while (regval & PWR_STATUS);

}
void gc2d_pwr(unsigned int power_on)
{
	spin_lock(&gc2d_pwr_lock);
	if (power_on) {
		if (gc2d_pwr_refcnt == 0)
			gc2d_on();
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
		gc2d_off();
	}
out:
	spin_unlock(&gc2d_pwr_lock);
}
EXPORT_SYMBOL(gc2d_pwr);

static void gc3d_on(void)
{
	unsigned int regval;
	unsigned int timeout;
	void __iomem *apmu_base;

	apmu_base = get_apmu_base_va();

	/* Clock setup procedure. */
	/* 1. Enable Dummy Clocks to SRAMs. */
	regval = readl(apmu_base + APMU_ISLD_GC_CTRL);
	regval |= CMEM_DMMYCLK_EN;
	writel(regval, apmu_base + APMU_ISLD_GC_CTRL);

	/* 2. Wait for 500ns. */
	udelay(1);

	/* 3. Disable Dummy Clocks to SRAMs. */
	regval = readl(apmu_base + APMU_ISLD_GC_CTRL);
	regval &= ~CMEM_DMMYCLK_EN;
	writel(regval, apmu_base + APMU_ISLD_GC_CTRL);

	/* 4. Disable GC3D fabric dynamic clock gating. */
	regval = readl(apmu_base + APMU_GC3D_CKGT);
	regval |= (GC3D_FAB_CKGT_DISABLE | MC_P4_CKGT_DISABLE);
	writel(regval, apmu_base + APMU_GC3D_CKGT);

	/* 5. Enable GC3D AXI clock. */
	regval = readl(apmu_base + APMU_GC3D_CLKCTRL);
	regval &= ~GC3D_ALCK_DIV_MASK;
	regval |= (1 << GC3D_ACLK_DIV_SHIFT);
	writel(regval, apmu_base + APMU_GC3D_CLKCTRL);

	/* 6. Enalbe GC3D CLK1X clock. */
	regval = readl(apmu_base + APMU_GC3D_CLKCTRL);
	regval &= ~GC3D_CLK1X_DIV_MASK;
	regval |= (1 << GC3D_CLK1X_DIV_SHIFT);
	writel(regval, apmu_base + APMU_GC3D_CLKCTRL);

	/* 7. Enable GC3D CLKSH clock. */
	regval = readl(apmu_base + APMU_GC3D_CLKCTRL);
	regval &= ~GC3D_CLKSH_DIV_MASK;
	regval |= (1 << GC3D_CLKSH_DIV_SHIFT);
	writel(regval, apmu_base + APMU_GC3D_CLKCTRL);

	/* 8. Enable GC3D HCLK clock. */
	regval = readl(apmu_base + APMU_GC3D_CLKCTRL);
	regval |= GC3D_HCLK_EN;
	writel(regval, apmu_base + APMU_GC3D_CLKCTRL);

	/* 9. Enable Frequency change. */
	regval = readl(apmu_base + APMU_GC3D_RSTCTRL);
	regval |= GC3D_FC_EN;
	writel(regval, apmu_base + APMU_GC3D_RSTCTRL);

	/* Power up procedure. */
	/* 1. Enable HWMODE to power up the island. */
	regval = readl(apmu_base + APMU_ISLD_GC3D_PWRCTRL);
	regval |= HWMODE_EN;
	writel(regval, apmu_base + APMU_ISLD_GC3D_PWRCTRL);

	/* 2. Remove interrupt Mask. */
	regval = readl(apmu_base + APMU_ISLD_GC3D_PWRCTRL);
	regval &= ~INT_MASK;
	writel(regval, apmu_base + APMU_ISLD_GC3D_PWRCTRL);

	/* 3. Power up the island. */
	regval = readl(apmu_base + APMU_ISLD_GC3D_PWRCTRL);
	regval |= PWRUP;
	writel(regval, apmu_base + APMU_ISLD_GC3D_PWRCTRL);

	/* 4. Wait for island to be powered up. */
	timeout = 1000;
	do {
		if (--timeout == 0) {
			WARN(1, "GC3D: power up timeout!\n");
			return;
		}
		udelay(10);
		regval = readl(apmu_base + APMU_ISLD_GC3D_PWRCTRL);
	} while (!(regval & PWR_STATUS));

	/*
	 * 5. Wait for active interrupt pending, indicating
	 * completion of island power up sequence.
	 * */
	timeout = 1000;
	do {
		if (--timeout == 0) {
			WARN(1, "GC3D: active interrupt pending!\n");
			return;
		}
		udelay(10);
		regval = readl(apmu_base + APMU_ISLD_GC3D_PWRCTRL);
	} while (!(regval & INT_STATUS));

	/*
	 * 6. The island is now powered up. Clear the interrupt and
	 *    set the interrupt masks.
	 */
	regval = readl(apmu_base + APMU_ISLD_GC3D_PWRCTRL);
	regval |= INT_CLR | INT_MASK;
	writel(regval, apmu_base + APMU_ISLD_GC3D_PWRCTRL);

	/* Reset Procedure. */
	/* 1. De-assert GC3D PWRON Reset. */
	regval = readl(apmu_base + APMU_GC3D_RSTCTRL);
	regval |= GC3D_PWRON_RST;
	writel(regval, apmu_base + APMU_GC3D_RSTCTRL);

	/* 2. De-assert GC3D ACLK Reset, CLK1X Reset and HCLK Reset. */
	regval = readl(apmu_base + APMU_GC3D_RSTCTRL);
	regval |= (GC3D_ACLK_RST | GC3D_CLK1X_RST | GC3D_HCLK_RST);
	writel(regval, apmu_base + APMU_GC3D_RSTCTRL);

	/* 3. Wait for 1000ns. */
	udelay(1);

	/* 4. Enable GC3D fabric dynamic clock gating. */
	regval = readl(apmu_base + APMU_GC3D_CKGT);
	regval &= ~(GC3D_FAB_CKGT_DISABLE | MC_P4_CKGT_DISABLE);
	writel(regval, apmu_base + APMU_GC3D_CKGT);
}

static void gc3d_off(void)
{
	unsigned int regval;
	unsigned int timeout;
	void __iomem *apmu_base;

	apmu_base = get_apmu_base_va();

	/* 1. Assert GC3D ACLK/CLK1X/HCLK Reset. */
	regval = readl(apmu_base + APMU_GC3D_RSTCTRL);
	regval &= ~(GC3D_ACLK_RST | GC3D_CLK1X_RST | GC3D_HCLK_RST);
	writel(regval, apmu_base + APMU_GC3D_RSTCTRL);

	/*
	 * 2. Wait a minimum of 32 cycles of slowest clock
	 * (AXI clock, GC3D CLK1X, GC3D HCLK) after reset has
	 * been asserted.
	 * */
	udelay(100); /* FIXME */

	/* 3. Disable GC3D AXI clock. */
	regval = readl(apmu_base + APMU_GC3D_CLKCTRL);
	regval &= ~GC3D_ALCK_DIV_MASK;
	writel(regval, apmu_base + APMU_GC3D_CLKCTRL);

	/* 4. Disable GC3D CLK1X clock. */
	regval = readl(apmu_base + APMU_GC3D_CLKCTRL);
	regval &= ~GC3D_CLK1X_DIV_MASK;
	writel(regval, apmu_base + APMU_GC3D_CLKCTRL);

	/* 5. Disable GC3D HCLK clock. */
	regval = readl(apmu_base + APMU_GC3D_CLKCTRL);
	regval &= ~GC3D_HCLK_EN;
	writel(regval, apmu_base + APMU_GC3D_CLKCTRL);

	/* 6. Enable HWMODE to power down the island. */
	regval = readl(apmu_base + APMU_ISLD_GC3D_PWRCTRL);
	regval |= HWMODE_EN;
	writel(regval, apmu_base + APMU_ISLD_GC3D_PWRCTRL);

	/* 7. Power down the island. */
	regval = readl(apmu_base + APMU_ISLD_GC3D_PWRCTRL);
	regval &= ~PWRUP;
	writel(regval, apmu_base + APMU_ISLD_GC3D_PWRCTRL);

	/* 8. Wait for island to be powered down. */
	timeout = 1000;
	do {
		if (--timeout == 0) {
			WARN(1, "GC3D: power down timeout!\n");
			return;
		}
		udelay(10);
		regval = readl(apmu_base + APMU_ISLD_GC3D_PWRCTRL);
	} while (regval & PWR_STATUS);
}

void gc3d_pwr(unsigned int power_on)
{
	spin_lock(&gc3d_pwr_lock);
	if (power_on) {
		if (gc3d_pwr_refcnt == 0)
			gc3d_on();
		gc3d_pwr_refcnt++;
	} else {
		/*
		 * If try to power off gc3d, but gc3d_pwr_refcnt is 0,
		 * print warning and return.
		 */
		if (WARN_ON(gc3d_pwr_refcnt == 0))
			goto out;

		if (--gc3d_pwr_refcnt > 0)
			goto out;

		gc3d_off();
	}
out:
	spin_unlock(&gc3d_pwr_lock);
}
EXPORT_SYMBOL(gc3d_pwr);
