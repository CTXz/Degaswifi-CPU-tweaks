#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/regs-addr.h>
#include <linux/of_platform.h>

/* VPU pwr function */
#define APMU_ISLD_VPU_CTRL	0x1b0
#define APMU_VPU_RSTCTRL	0x1f0
#define APMU_VPU_CLKCTRL	0x1f4
#define APMU_ISLD_VPU_PWRCTRL	0x208
#define APMU_VPU_CKGT		0x6c

#define HWMODE_EN	(1u << 0)
#define PWRUP		(1u << 1)
#define PWR_STATUS	(1u << 4)
#define REDUN_STATUS	(1u << 5)
#define INT_CLR		(1u << 6)
#define INT_MASK	(1u << 7)
#define INT_STATUS	(1u << 8)
#define CMEM_DMMYCLK_EN		(1u << 4)

#define VPU_ACLK_RST	(1u << 0)
#define VPU_DEC_CLK_RST	(1u << 1)
#define VPU_ENC_CLK_RST	(1u << 2)
#define VPU_HCLK_RST	(1u << 3)
#define VPU_PWRON_RST	(1u << 7)
#define VPU_FC_EN	(1u << 9)

#define VPU_ACLK_DIV_MASK	(7 << 0)
#define VPU_ACLK_DIV_SHIFT	0

#define VPU_ACLKSRC_SEL_MASK	(7 << 4)
#define VPU_ACLKSRC_SEL_SHIFT	4

#define VPU_DCLK_DIV_MASK	(7 << 8)
#define	VPU_DCLK_DIV_SHIFT	8

#define	VPU_DCLKSRC_SEL_MASK	(7 << 12)
#define VPU_DCLKSRC_SEL_SHIFT	12

#define	VPU_ECLK_DIV_MASK	(7 << 16)
#define VPU_ECLK_DIV_SHIFT	16

#define VPU_ECLKSRC_SEL_MASK	(7 << 20)
#define VPU_ECLKSRC_SEL_SHIFT	20

#define VPU_UPDATE_RTCWTC	(1u << 31)

/* record reference count on hantro power on/off */
static int hantro_pwr_refcnt;

/* used to protect gc2d/gc3d power sequence */
static DEFINE_SPINLOCK(hantro_pwr_lock);

static void hantro_on(void)
{
	unsigned int regval;
	unsigned int timeout;
	void __iomem *apmu_base;

	apmu_base = get_apmu_base_va();

	/* Power up procedure. */
	/* 1. Enable HWMODE to power up the island. */
	regval = readl(apmu_base + APMU_ISLD_VPU_PWRCTRL);
	regval |= HWMODE_EN;
	writel(regval, apmu_base + APMU_ISLD_VPU_PWRCTRL);

	/* 2. Remove interrupt Mask. */
	regval = readl(apmu_base + APMU_ISLD_VPU_PWRCTRL);
	regval &= ~INT_MASK;
	writel(regval, apmu_base + APMU_ISLD_VPU_PWRCTRL);

	/* 3. Power up the island. */
	regval = readl(apmu_base + APMU_ISLD_VPU_PWRCTRL);
	regval |= PWRUP;
	writel(regval, apmu_base + APMU_ISLD_VPU_PWRCTRL);

	/* 4. Wait for island to be powered up. */
	timeout = 1000;
	do {
		if (--timeout == 0) {
			WARN(1, "VPU: power up timeout!\n");
			return;
		}
		udelay(10);
		regval = readl(apmu_base + APMU_ISLD_VPU_PWRCTRL);
	} while (!(regval & PWR_STATUS));

	/*
	 * 5. Wait for active interrupt pending, indicating
	 * completion of island power up sequence.
	 * */
	timeout = 1000;
	do {
		if (--timeout == 0) {
			WARN(1, "VPU: active interrupt pending!\n");
			return;
		}
		udelay(10);
		regval = readl(apmu_base + APMU_ISLD_VPU_PWRCTRL);
	} while (!(regval & INT_STATUS));

	/*
	 * 6. The island is now powered up. Clear the interrupt and
	 *    set the interrupt masks.
	 */
	regval = readl(apmu_base + APMU_ISLD_VPU_PWRCTRL);
	regval |= INT_CLR | INT_MASK;
	writel(regval, apmu_base + APMU_ISLD_VPU_PWRCTRL);

	/* Clock Enable and Reset De-assertion Procedure. */
	/* 1. Enable Dummy Clocks to SRAMs. */
	regval = readl(apmu_base + APMU_ISLD_VPU_CTRL);
	regval |= CMEM_DMMYCLK_EN;
	writel(regval, apmu_base + APMU_ISLD_VPU_CTRL);

	/* 2. Wait for 500ns. */
	udelay(1);

	/* 3. Disable Dummy Clocks to SRAMs. */
	regval = readl(apmu_base + APMU_ISLD_VPU_CTRL);
	regval &= ~CMEM_DMMYCLK_EN;
	writel(regval, apmu_base + APMU_ISLD_VPU_CTRL);

	/* 4. Disable VPU fabric Dynamic Clock gating. */
	regval = readl(apmu_base + APMU_VPU_CKGT);
	regval |= 0x1f;
	writel(regval, apmu_base + APMU_VPU_CKGT);

	/* 5. De-assert VPU HCLK Reset. */
	regval = readl(apmu_base + APMU_VPU_RSTCTRL);
	regval |= VPU_HCLK_RST;
	writel(regval, apmu_base + APMU_VPU_RSTCTRL);

	/* 6. Enable VPU AXI Clock. */
	regval = readl(apmu_base + APMU_VPU_CLKCTRL);
	regval &= ~VPU_ACLKSRC_SEL_MASK;
	regval |= (0 << VPU_ACLKSRC_SEL_SHIFT);
	regval &= ~VPU_ACLK_DIV_MASK;
	regval |= (2 << VPU_ACLK_DIV_SHIFT);
	writel(regval, apmu_base + APMU_VPU_CLKCTRL);

	/* 7. Enable VPU Peripheral Clock. */
	/* a. Enable Encoder Clock. */
	regval = readl(apmu_base + APMU_VPU_CLKCTRL);
	regval &= ~VPU_ECLKSRC_SEL_MASK;
	regval |= (0 << VPU_ECLKSRC_SEL_SHIFT);
	regval &= ~VPU_ECLK_DIV_MASK;
	regval |= (2 << VPU_ECLK_DIV_SHIFT);
	writel(regval, apmu_base + APMU_VPU_CLKCTRL);

	/* b. Enable Decoder Clock. */
	regval = readl(apmu_base + APMU_VPU_CLKCTRL);
	regval &= ~VPU_DCLKSRC_SEL_MASK;
	regval |= (0 << VPU_DCLKSRC_SEL_SHIFT);
	regval &= ~VPU_DCLK_DIV_MASK;
	regval |= (2 << VPU_DCLK_DIV_SHIFT);
	writel(regval, apmu_base + APMU_VPU_CLKCTRL);

	/* 8. Enable VPU Frequency Change. */
	regval = readl(apmu_base + APMU_VPU_RSTCTRL);
	regval |= VPU_FC_EN;
	writel(regval, apmu_base + APMU_VPU_RSTCTRL);

	/* 9. De-assert VPU ACLK/DCLK/ECLK Reset. */
	regval = readl(apmu_base + APMU_VPU_RSTCTRL);
	regval |= VPU_ACLK_RST | VPU_DEC_CLK_RST | VPU_ENC_CLK_RST;
	writel(regval, apmu_base + APMU_VPU_RSTCTRL);

	/* 10. Enable VPU fabric Dynamic Clock gating. */
	regval = readl(apmu_base + APMU_VPU_CKGT);
	regval &= ~0x1f;
	writel(regval, apmu_base + APMU_VPU_CKGT);

}

static void hantro_off(void)
{
	unsigned int regval;
	unsigned int timeout;
	void __iomem *apmu_base;

	apmu_base = get_apmu_base_va();

	/* 1. Assert Bus Reset and Perpheral Reset. */
	regval = readl(apmu_base + APMU_VPU_RSTCTRL);
	regval &= ~(VPU_ACLK_RST | VPU_DEC_CLK_RST | VPU_ENC_CLK_RST
			| VPU_HCLK_RST);
	writel(regval, apmu_base + APMU_VPU_RSTCTRL);

	/* 2. Disable Bus and Peripheral Clock. */
	regval = readl(apmu_base + APMU_VPU_CLKCTRL);
	regval &= ~(VPU_ACLK_DIV_MASK | VPU_DCLK_DIV_MASK | VPU_ECLK_DIV_MASK);
	writel(regval, apmu_base + APMU_VPU_CLKCTRL);

	/* 3. Enable HWMODE to power down the island. */
	regval = readl(apmu_base + APMU_ISLD_VPU_PWRCTRL);
	regval |= HWMODE_EN;
	writel(regval, apmu_base + APMU_ISLD_VPU_PWRCTRL);

	/* 4. Power down the island. */
	regval = readl(apmu_base + APMU_ISLD_VPU_PWRCTRL);
	regval &= ~PWRUP;
	writel(regval, apmu_base + APMU_ISLD_VPU_PWRCTRL);

	/* 5. Wait for island to be powered down. */
	timeout = 1000;
	do {
		if (--timeout == 0) {
			WARN(1, "GC3D: power down timeout!\n");
			return;
		}
		udelay(10);
		regval = readl(apmu_base + APMU_ISLD_VPU_PWRCTRL);
	} while (regval & PWR_STATUS);
}

void hantro_power_switch(unsigned int power_on)
{
	spin_lock(&hantro_pwr_lock);
	if (power_on) {
		if (hantro_pwr_refcnt == 0)
			hantro_on();
		hantro_pwr_refcnt++;
	} else {
		/*
		 * If try to power off hantro, but hantro_pwr_refcnt is 0,
		 * print warning and return.
		 */
		if (WARN_ON(hantro_pwr_refcnt == 0))
			goto out;

		if (--hantro_pwr_refcnt > 0)
			goto out;

		hantro_off();
	}
out:
	spin_unlock(&hantro_pwr_lock);
}
EXPORT_SYMBOL(hantro_power_switch);

void enable_smmu_power_domain(struct device *dev, int enable)
{
	u32 pd_index = 0;

	if (of_property_read_u32(dev->of_node, "pd_index", &pd_index)) {
		pr_err("missing pd_index property");
		return;
	}
	if (pd_index == 1)
		hantro_power_switch(enable);
}
