/*
 * linux/drivers/iommu/mmp-iommu.c
 *
 * This is lite version of the MMU400 iommu driver for Marvell silicon.
 *
 * Copyright:   (C) 2013 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/iommu.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/debugfs.h>

#include <asm/cacheflush.h>
#include <asm/tlbflush.h>

#include "mmp-iommu.h"

#undef PTRS_PER_PTE
#undef PTRS_PER_PMD
#undef PMD_TYPE_MASK
#undef PMD_TYPE_FAULT
#undef PMD_TYPE_TABLE
#undef PMD_TYPE_SECT
#undef PTE_TYPE_MASK
#undef PTE_TYPE_FAULT
#undef PTE_TYPE_PAGE
#undef PMD_MASK

#define PTRS_PER_PTE		512
#define PTRS_PER_PMD		2048

#define PMD_SHIFT		21
#define PTE_SHIFT		12

#define PMD_TYPE_MASK		3
#define PMD_TYPE_FAULT		0
#define PMD_TYPE_TABLE		3
#define PMD_TYPE_SECT		1

#define PTE_TYPE_MASK		3
#define PTE_TYPE_FAULT		0
#define PTE_TYPE_PAGE		3

#define PMD_SIZE		(1UL << PMD_SHIFT)
#define PTE_SIZE		(1UL << PTE_SHIFT)

#define PMD_MASK		(PMD_SIZE - 1)
#define PTE_MASK		(PTE_SIZE - 1)

#define PMD_RANGE		((1UL << PMD_SHIFT) - 1)
#define PTE_RANGE		((1UL << (PMD_SHIFT - PTE_SHIFT)) - 1)

#define TABLE_ADDR_MASK		(0xFFFFFF000ULL)

#define pmd_entry_type(e, type)	((*e & PMD_TYPE_MASK) == type)
#define pte_entry_type(e, type)	((*e & PTE_TYPE_MASK) == type)

#define pmd_entry_get(base, iova) (base + (iova >> PMD_SHIFT))
#define pte_entry_get(base, iova) (base + ((iova >> PTE_SHIFT) & PTE_RANGE));

#define pmd_phys_addr_sect(e)	((*e) & 0xFFFE00000ULL)
#define pmd_phys_addr_table(e)	((*e) & 0xFFFFFF000ULL)
#define pte_phys_addr_page(e)	((*e) & 0xFFFFFF000ULL)

#define add_pmd_count(e)	(*((u32 *)e) += (1 << 2))
#define sub_pmd_count(e)	(*((u32 *)e) -= (1 << 2))
#define get_pmd_count(e)	((*((u32 *)e) >> 2) & 0x3FF)


static struct mmp_iommu_drvdata *g_mmp_iommu_drvdata;

static inline u64 *pte_table_base(u64 *e)
{
	return (u64 *)__va((*e) & TABLE_ADDR_MASK);
}

static void mmp_iommu_cache_flush(void *start, unsigned long size)
{
	void *end = (void *)((unsigned long)start + size);

	dmac_flush_range(start, end);
#ifdef CONFIG_OUTER_CACHE
	outer_flush_range(virt_to_phys(start), virt_to_phys(end));
#endif
}

static void pgt_entry_set(u64 *e, phys_addr_t pa, u32 type)
{
	if (type)
		(*e) = pa | type;
	else
		(*e) = 0;
	mmp_iommu_cache_flush(e, sizeof(*e));
}

static void pte_entry_set(u64 *e, phys_addr_t pa, u32 type)
{
	if (type)
		(*e) = pa | type | 0x00400000000004c0;
	else
		(*e) = 0;
	mmp_iommu_cache_flush(e, sizeof(*e));
}

static int __dma_update_pte(pte_t *pte, pgtable_t token, unsigned long addr,
			    void *data)
{
	struct page *page = virt_to_page(addr);
	pgprot_t prot = *(pgprot_t *)data;

	set_pte_ext(pte, mk_pte(page, prot), 0);
	return 0;
}

static void __dma_remap(struct page *page, size_t size, pgprot_t prot)
{
	unsigned long start = (unsigned long) page_address(page);
	unsigned end = start + size;

	apply_to_page_range(&init_mm, start, size, __dma_update_pte, &prot);
	dsb();
	flush_tlb_kernel_range(start, end);
}

static void mmp_iommu_tlb_inv(struct mmp_iommu_domain *priv)
{
	struct mmp_iommu_drvdata *drvdata = priv->drvdata;
	void __iomem *gr0_base = drvdata->regbase;

	writel(0x00000000, gr0_base + SMMU_TLBIVMID);
	/* Global Synchronize TLB Invalidate */
	writel(0x00000000, gr0_base + SMMU_TLBGSYNC);
	mb();
}

static int mmp_iommu_map(struct iommu_domain *domain, unsigned long iova,
	phys_addr_t paddr, size_t size, int prot)
{
	struct mmp_iommu_domain *priv = domain->priv;
	u64 *pmd, *pte;
	u64 *pte_base;
	unsigned long flags;
	int ret = 0;

	BUG_ON(!priv || !priv->pgd);

	spin_lock_irqsave(&priv->pgtablelock, flags);

	pmd = pmd_entry_get(priv->pgd, iova);

	switch (size) {
	case SZ_2M:
	{
		WARN(!pmd_entry_type(pmd, PMD_TYPE_FAULT),
			"pmd entry in use! [%08lx] %llx", iova, *pmd);

		pgt_entry_set(pmd, paddr, PMD_TYPE_SECT);
		break;
	}
	case SZ_4K:
	{
		if (pmd_entry_type(pmd, PMD_TYPE_FAULT)) {
			pte_base = (u64 *)get_zeroed_page(GFP_ATOMIC);
			mmp_iommu_cache_flush(pte_base, SZ_4K);
			__dma_remap(virt_to_page(pte_base), SZ_4K,
				pgprot_noncached(pgprot_kernel));
			pgt_entry_set(pmd, __pa(pte_base), PMD_TYPE_TABLE);
		} else {
			pte_base = pte_table_base(pmd);
		}

		pte = pte_entry_get(pte_base, iova);
		pte_entry_set(pte, paddr, PTE_TYPE_PAGE);

		add_pmd_count(pmd);
		break;
	}
	default:
		pr_warn("iommu_map error: size %u not supported!", size);
		ret = -EINVAL;
	}

	spin_unlock_irqrestore(&priv->pgtablelock, flags);
	return ret;
}

static size_t mmp_iommu_unmap(struct iommu_domain *domain,
	unsigned long iova, size_t size)
{
	struct mmp_iommu_domain *priv = domain->priv;
	u64 *pmd, *pte;
	u64 *pte_base;
	unsigned long flags;

	BUG_ON(!priv || !priv->pgd);

	spin_lock_irqsave(&priv->pgtablelock, flags);

	pmd = pmd_entry_get(priv->pgd, iova);

	BUG_ON(pmd_entry_type(pmd, PMD_TYPE_FAULT));

	if (pmd_entry_type(pmd, PMD_TYPE_SECT)) {
		BUG_ON(size < SZ_2M);

		pgt_entry_set(pmd, 0, PMD_TYPE_FAULT);
		size = SZ_2M;
	} else {
		pte_base = pte_table_base(pmd);
		pte = pte_entry_get(pte_base, iova);

		BUG_ON(!pte_entry_type(pte, PTE_TYPE_PAGE));

		pte_entry_set(pte, 0, PTE_TYPE_FAULT);
		sub_pmd_count(pmd);

		if (get_pmd_count(pmd) == 0) {
			pgt_entry_set(pmd, 0, PMD_TYPE_FAULT);
			__dma_remap(virt_to_page(pte_base), SZ_4K,
				pgprot_kernel);
			free_page((unsigned long)pte_base);
		}
		size = SZ_4K;
	}

	mmp_iommu_tlb_inv(priv);
	spin_unlock_irqrestore(&priv->pgtablelock, flags);
	return size;
}

static phys_addr_t mmp_iommu_iova_to_phys(struct iommu_domain *domain,
					   dma_addr_t iova)
{
	struct mmp_iommu_domain *priv = domain->priv;
	u64 *pmd, *pte = NULL;
	u64 *pte_base;
	unsigned long flags;
	phys_addr_t phys = 0;

	spin_lock_irqsave(&priv->pgtablelock, flags);

	pmd = pmd_entry_get(priv->pgd, iova);

	if (pmd_entry_type(pmd, PMD_TYPE_FAULT))
		goto err;

	if (pmd_entry_type(pmd, PMD_TYPE_SECT)) {
		phys = (phys_addr_t)(pmd_phys_addr_sect(pmd)
				+ (iova & PMD_MASK));
		goto exit;
	}

	if (pmd_entry_type(pmd, PMD_TYPE_TABLE)) {
		pte_base = pte_table_base(pmd);
		pte = pte_entry_get(pte_base, iova);

		if (!pte_entry_type(pmd, PTE_TYPE_PAGE))
			goto err;

		phys = (phys_addr_t)(pte_phys_addr_page(pte)
				+ (iova & PTE_MASK));
		goto exit;
	}

err:
	WARN(1, "unmapped iova %08lx, pmd(%016llx) pte(%016llx)\n",
		(unsigned long)iova, *pmd, pte ? *pte : 0xDEADBEAF);
exit:
	spin_unlock_irqrestore(&priv->pgtablelock, flags);
	return phys;
}

static int mmp_iommu_domain_has_cap(struct iommu_domain *domain,
				     unsigned long cap)
{
	return 0;
}

static void mmp_iommu_reset(struct iommu_domain *domain)
{
	struct mmp_iommu_drvdata *drvdata;
	struct mmp_iommu_domain *priv = domain->priv;
	phys_addr_t pgd;
	void __iomem *gr0_base;
	void __iomem *cb0_base;

	drvdata = g_mmp_iommu_drvdata;
	gr0_base = drvdata->regbase;
	cb0_base = gr0_base + 0x8000;
	pgd = __pa(priv->pgd);

	/* Auxiliary Configuration Register */
	writel_relaxed(0x00000000, gr0_base + SMMU_NSACR);

	/* Stream Match register, ignored all mask bits, stream id 0 */
	writel_relaxed(0xffff0000, gr0_base + SMMU_SMR(0));

	/* Stream to Context register, strongly ordered, context bank 0 */
	writel_relaxed(0x00000000, gr0_base + SMMU_S2CR(0));

	/* System Control Register, enable MMU for context bank 0 */
	writel_relaxed(0x00000001, cb0_base + SMMU_CB_SCTLR);

	/* Translation Table Base Register */
	writel_relaxed((u32)pgd, cb0_base + SMMU_CB_TTBR0_L);
#ifdef CONFIG_64BIT
	writel_relaxed((u32)(pgd >> 32), cb0_base + SMMU_CB_TTBR0_H);
#else
	writel_relaxed(0x00000000, cb0_base + SMMU_CB_TTBR0_H);
#endif

	/* Translation Table Control Register, SL0=0 two level, T0SZ=1 */
	writel_relaxed(0x80000001, cb0_base + SMMU_CB_TTBCR);

	/* Configuration Register 0, CLIENTPD=0, enable MMU*/
	writel_relaxed(0xaa9b003e, gr0_base + SMMU_NSCR0);

	writel_relaxed(0x00000000, gr0_base + SMMU_STLBIALL);
	writel_relaxed(0x00000000, gr0_base + SMMU_TLBIALLH);
	writel_relaxed(0x00000000, gr0_base + SMMU_TLBIALLNSNH);

	/* Global Synchronize TLB Invalidate */
	writel_relaxed(0x00000000, gr0_base + SMMU_TLBIVMID);
	writel_relaxed(0x00000000, gr0_base + SMMU_TLBGSYNC);

	mb();
}

static int mmp_iommu_attach_dev(struct iommu_domain *domain,
				 struct device *dev)
{
	struct mmp_iommu_drvdata *drvdata;
	struct mmp_iommu_domain *priv = domain->priv;
	struct mmp_device_node *dnode;
	unsigned long flags;
	int ret = 0;

	dnode = kzalloc(sizeof(*dnode), GFP_KERNEL);

	dnode->dev = dev;
	INIT_LIST_HEAD(&dnode->node);

	drvdata = g_mmp_iommu_drvdata;
	spin_lock_irqsave(&priv->lock, flags);

	if (!priv->drvdata)
		priv->drvdata = drvdata;

	if (list_empty(&priv->node))
		list_add_tail(&priv->node, &drvdata->node_domains);

	list_add_tail(&dnode->node, &priv->node_devs);

	spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}

static void mmp_iommu_detach_dev(struct iommu_domain *domain,
				  struct device *dev)
{
	struct mmp_iommu_drvdata *drvdata;
	struct mmp_iommu_domain *priv = domain->priv;
	struct mmp_device_node *dnode, *tmpnode;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	drvdata = g_mmp_iommu_drvdata;
	list_for_each_entry_safe(dnode, tmpnode, &priv->node_devs, node) {
		if (dnode->dev == dev) {
			list_del(&dnode->node);
			kfree(dnode);
		}
	}

	priv->drvdata = NULL;
	spin_unlock_irqrestore(&priv->lock, flags);
}

static int mmp_iommu_domain_init(struct iommu_domain *domain)
{
	struct mmp_iommu_domain *priv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	INIT_LIST_HEAD(&priv->node);
	INIT_LIST_HEAD(&priv->node_devs);

	priv->pgd = (u64 *)
		__get_free_pages(GFP_KERNEL | __GFP_ZERO, get_order(SZ_16K));

	if (!priv->pgd)
		goto err_exit;

	mmp_iommu_cache_flush(priv->pgd, SZ_16K);
	__dma_remap(virt_to_page(priv->pgd), SZ_16K,
		pgprot_noncached(pgprot_kernel));

	spin_lock_init(&priv->lock);
	spin_lock_init(&priv->pgtablelock);

	domain->geometry.aperture_start = 0;
	domain->geometry.aperture_end   = ~0UL;
	domain->geometry.force_aperture = true;

	domain->priv = priv;
	return 0;

err_exit:
	if (priv->pgd)
		free_pages((unsigned long)priv->pgd, get_order(SZ_16K));
	kfree(priv);
	return -ENOMEM;
}

static void mmp_iommu_domain_destroy(struct iommu_domain *domain)
{
	struct mmp_iommu_domain *priv = domain->priv;

	if (priv->pgd)
		free_pages((unsigned long)priv->pgd, get_order(SZ_16K));

	kfree(priv);
	domain->priv = NULL;
}

static int mmp_iommu_add_device(struct device *dev)
{
	return 0;
}

static void mmp_iommu_remove_device(struct device *dev)
{
	dev->archdata.iommu = NULL;
}

static irqreturn_t mmp_iommu_fault_handler(int irq, void *data)
{
	struct mmp_iommu_drvdata *drvdata = (struct mmp_iommu_drvdata *)data;
	dev_info(drvdata->dev, "mmp_iommu_fault_handler\n");
	return IRQ_NONE;
}

static int mmp_iommu_custom(struct iommu_domain *domain, unsigned int cmd,
	unsigned long arg)
{
	mmp_iommu_reset(domain);
	return 0;
}

static struct iommu_ops mmp_iommu_ops = {
	.domain_init	= mmp_iommu_domain_init,
	.domain_destroy	= mmp_iommu_domain_destroy,
	.attach_dev	= mmp_iommu_attach_dev,
	.detach_dev	= mmp_iommu_detach_dev,
	.custom		= mmp_iommu_custom,
	.map		= mmp_iommu_map,
	.unmap		= mmp_iommu_unmap,
	.iova_to_phys	= mmp_iommu_iova_to_phys,
	.domain_has_cap	= mmp_iommu_domain_has_cap,
	.add_device	= mmp_iommu_add_device,
	.remove_device	= mmp_iommu_remove_device,
	.pgsize_bitmap	= SZ_4K | SZ_2M,
};

#ifdef CONFIG_DEBUG_FS
static void mid_pgtable_list(struct seq_file *s, struct mmp_iommu_domain *priv)
{
	unsigned long i, j;
	u64 *pmd, *pte;
	unsigned long total = 0;

	pmd = (u64 *)priv->pgd;

	seq_puts(s, "IOVA        PHYS       ENTRY_ADDR ENTRY_VALUE\n");
	for (i = 0; i < PTRS_PER_PMD; i++, pmd++) {
		if (pmd_entry_type(pmd, PMD_TYPE_FAULT))
			continue;

		if (pmd_entry_type(pmd, PMD_TYPE_SECT)) {
			seq_printf(s, "[%08lx]: %010llx %010lx %016llx\n",
				i << PMD_SHIFT,
				pmd_phys_addr_sect(pmd),
				__pa(pmd), *pmd);
			total += SZ_2M;
			continue;
		}

		BUG_ON(!pmd_entry_type(pmd, PMD_TYPE_TABLE));

		seq_printf(s, "[%08lx]: %010llx %010lx %016llx (%u)\n",
			i << PMD_SHIFT,
			pmd_phys_addr_table(pmd),
			__pa(pmd), *pmd,
			get_pmd_count(pmd));

		pte = pte_table_base(pmd);

		for (j = 0; j < PTRS_PER_PTE; j++, pte++) {
			if (pte_entry_type(pte, PTE_TYPE_FAULT))
				continue;

			BUG_ON(!pte_entry_type(pmd, PTE_TYPE_PAGE));

			seq_printf(s, "   %06lx : %010llx %010lx %016llx\n",
				j << PTE_SHIFT,
				pte_phys_addr_page(pte),
				__pa(pte), *pte);
			total += SZ_4K;
		}
	}

	seq_printf(s, "Total mapped: %lu (0x%08lx)\n", total, total);
}

static void mid_devices_show(struct seq_file *s, struct mmp_iommu_domain *priv)
{
	struct mmp_device_node *dnode;

	seq_puts(s, "Attached devices:");
	list_for_each_entry(dnode, &priv->node_devs, node) {
		seq_printf(s, " %s", dev_name(dnode->dev));
	}
	seq_puts(s, "\n");
}

static int mid_pgtable_show(struct seq_file *s, void *v)
{
	struct mmp_iommu_drvdata *drvdata = s->private;
	struct mmp_iommu_domain *priv;

	list_for_each_entry(priv, &drvdata->node_domains, node) {
		mid_pgtable_list(s, priv);
		mid_devices_show(s, priv);
	}

	return 0;
}

static int mid_pgtable_open(struct inode *inode, struct file *file)
{
	return single_open(file, mid_pgtable_show, inode->i_private);
}

const struct file_operations mid_pgtable_fops = {
	.open	= mid_pgtable_open,
	.read	= seq_read,
};

static int mid_stat_show(struct seq_file *s, void *v)
{
	struct mmp_iommu_drvdata *drvdata = s->private;
	struct mmp_iommu_domain *priv;

	seq_puts(s, "mmp_iommu_drvdata:\n");
	seq_printf(s, " regbase: %08lx\n", (unsigned long)drvdata->regbase);

	list_for_each_entry(priv, &drvdata->node_domains, node) {
		mid_devices_show(s, priv);
		seq_printf(s, " pgtable: %08lx\n",
			(unsigned long)priv->pgd);
	}

	return 0;
}

static int mid_stat_open(struct inode *inode, struct file *file)
{
	return single_open(file, mid_stat_show, inode->i_private);
}

const struct file_operations mid_stat_fops = {
	.open	= mid_stat_open,
	.read	= seq_read,
};

static void mmp_iommu_debugfs_delete(struct mmp_iommu_drvdata *drvdata)
{
	debugfs_remove_recursive(drvdata->debugfs_root);
}

static void mmp_iommu_debugfs_create(struct mmp_iommu_drvdata *drvdata)
{
	struct device *dev = drvdata->dev;
	struct dentry *root;
	struct dentry *file;

	root = debugfs_create_dir(dev_name(dev), NULL);
	if (!root) {
		dev_err(dev, "failed create debugfs!\n");
		goto err_exit;
	}
	drvdata->debugfs_root = root;

	file = debugfs_create_file("pgtable", 0664, root,
			drvdata, &mid_pgtable_fops);
	if (!file)
		goto err_exit;

	file = debugfs_create_file("stat", 0664, root,
			drvdata, &mid_stat_fops);
	if (!file)
		goto err_exit;

	return;
err_exit:
	mmp_iommu_debugfs_delete(drvdata);
	return;
}
#endif

static int mmp_iommu_probe(struct platform_device *pdev)
{
	int err = -ENODEV;
	int irq;
	struct resource *res;
	struct mmp_iommu_drvdata *drvdata;
	struct device *dev = &pdev->dev;

	drvdata = kzalloc(sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata) {
		dev_err(dev, "no drvdata memory\n");
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&drvdata->node_domains);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no mem resource\n");
		err = -ENODEV;
		goto err_mem;
	}

	res = request_mem_region(res->start, resource_size(res),
				 dev_name(&pdev->dev));
	if (!res) {
		dev_err(dev, "request_mem_region fail\n");
		err = -EIO;
		goto err_mem;
	}

	drvdata->regbase = ioremap(res->start, resource_size(res));
	if (!drvdata->regbase) {
		dev_err(dev, "ioremap fail\n");
		err = -ENOMEM;
		goto err_ioremap;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(dev, "no irq found\n");
		err = -ENODEV;
		goto err_irq;
	}

	err = request_irq(irq, mmp_iommu_fault_handler, 0,
		dev_name(dev), drvdata);

	if (err < 0) {
		dev_err(dev, "request_irq %d fail\n", irq);
		goto err_irq;
	}

	platform_set_drvdata(pdev, drvdata);
	drvdata->dev = dev;
	g_mmp_iommu_drvdata = drvdata;

#ifdef CONFIG_DEBUG_FS
	mmp_iommu_debugfs_create(drvdata);
#endif
	dev_info(dev, "device registered\n");
	return 0;

err_irq:
	iounmap(drvdata->regbase);
err_ioremap:
	release_mem_region(res->start, resource_size(res));
err_mem:
	kfree(drvdata);
	return err;
}

static int mmp_iommu_remove(struct platform_device *pdev)
{
	int irq;
	struct resource *res;
	struct mmp_iommu_drvdata *drvdata = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	platform_set_drvdata(pdev, NULL);

	irq = platform_get_irq(pdev, 0);
	free_irq(irq, dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));
	iounmap(drvdata->regbase);

	dev_info(dev, "device removed\n");
	kfree(drvdata);
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id mmp_iommu_of_match[] = {
	{ .compatible = "mmp,mmu-400", },
	{ },
};
MODULE_DEVICE_TABLE(of, mmp_iommu_of_match);
#endif

static struct platform_driver mmp_iommu_driver = {
	.probe		= mmp_iommu_probe,
	.remove		= mmp_iommu_remove,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "mmp-iommu",
		.of_match_table	= of_match_ptr(mmp_iommu_of_match),
	},
};

static int mmp_iommu_init(void)
{
	int ret;

	ret = platform_driver_register(&mmp_iommu_driver);
	if (ret)
		return ret;

	if (!iommu_present(&platform_bus_type))
		bus_set_iommu(&platform_bus_type, &mmp_iommu_ops);

	return 0;
}

static void mmp_iommu_exit(void)
{
	platform_driver_unregister(&mmp_iommu_driver);
}

subsys_initcall(mmp_iommu_init);
module_exit(mmp_iommu_exit);
