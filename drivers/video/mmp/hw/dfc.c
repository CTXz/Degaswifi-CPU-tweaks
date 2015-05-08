/*
 * linux/drivers/video/mmp/hw/dfc.c
 * dfc driver support
 *
 * Copyright (C) 2014 Marvell Technology Group Ltd.
 * Authors:  huangyh <huangyh@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/stat.h>
#include "mmp_dsi.h"
#include "mmp_ctrl.h"
#include <video/mipi_display.h>
#include <linux/clk-private.h>
#include "clk.h"

#define ROUND_RATE_RANGE_MHZ		2  /* unit: MHz */
#define DFC_RETRY_TIMEOUT	60

#define LCD_PN_SCLK	(0xd420b1a8)
#define	SCLK_SOURCE_SELECT(src)		((src)<<30)
#define	SCLK_SOURCE_SELECT_MASK		0xc0000000
#define	SCLK_SOURCE_AXI				(0x0 << 30)
#define	SCLK_SOURCE_DISP1				(0x1 << 30)
#define	SCLK_SOURCE_DISP2				(0x2 << 30)
#define	SCLK_SOURCE_DSI_PLL			(0x3 << 30)
#define	SCLK_DISABLE					(1<<28)
#define	DSI1_BITCLK_DIV(div)			(div<<8)
#define	DSI1_BITCLK_DIV_MASK			0x00000F00
#define	CLK_INT_DIV(div)				(div)
#define	CLK_INT_DIV_MASK				0x000000FF

#define AMPU_LCD (0xd428284c)
#define LCD_CLK_EN		(1 << 6)

static unsigned int parent_clk_tbl[8];
static int parent_clk_count;
static const char * const parent0_clk_tbl[] = {"pn_sclk"};
static const char * const parent1_clk_tbl[] = {"disp1", "dsi_pll"};
static const char * const parent2_clk_tbl[] = {
	"pll1_416m", "pll1_624", "pll3", "pll2", "pll1_832m"};
static unsigned long pll3_vco_default;
static unsigned long pll3_default;
/* PLL post divider table */
static struct div_map pll_post_div_tbl[] = {
	/* divider, reg vaule */
	{1, 0},
	{2, 2},
	{3, 4},
	{4, 5},
	{6, 7},
	{8, 8},
};

struct parent_index {
	int parent1;
	int parent2;
};

enum {
	DFC_FREQ_PARENT2_PLL_NONE,
	DFC_FREQ_PARENT2_PLL1_416M,
	DFC_FREQ_PARENT2_PLL1_624M,
	DFC_FREQ_PARENT2_PLL3,
	DFC_FREQ_PARENT2_PLL2,
	DFC_FREQ_PARENT2_PLL1_832M,
};

enum {
	DFC_FREQ_PARENT1_NONE,
	DFC_FREQ_PARENT1_DISPLAY1,
	DFC_FREQ_PARENT1_DSIPLL,
};

enum {
	DFC_CHANGE_LCD,
	DFC_DISABLE_PLL3,
	DFC_ENABLE_PLL3,
	DFC_CHANGE_PLL3,
	DFC_RESTORE_PLL3,
};

static void get_parent_index(unsigned long rate, struct parent_index *index)
{
	int i = 0;
	struct clk *clk;

	for (i = 0; i < ARRAY_SIZE(parent2_clk_tbl); i++) {
		clk = clk_get(NULL, parent2_clk_tbl[i]);
		if ((!IS_ERR(clk)) && (rate == (clk->rate / MHZ_TO_HZ)))
			index->parent2 = i + DFC_FREQ_PARENT2_PLL1_416M;
	}

	if ((index->parent2 == DFC_FREQ_PARENT2_PLL1_416M) ||
		(index->parent2 == DFC_FREQ_PARENT2_PLL1_624M))
		index->parent1 = DFC_FREQ_PARENT1_DISPLAY1;

	if ((index->parent2 == DFC_FREQ_PARENT2_PLL3) ||
		(index->parent2 == DFC_FREQ_PARENT2_PLL2) ||
		(index->parent2 == DFC_FREQ_PARENT2_PLL1_832M))
		index->parent1 = DFC_FREQ_PARENT1_DSIPLL;
}

static void add_to_parent_tbl(unsigned long rate)
{
	int i;

	rate /= 1000000;
	if (rate > 1000)
		return;
	for (i = 0; i < 8; i++) {
		if (parent_clk_tbl[i] == rate)
			return;
	}
	parent_clk_tbl[parent_clk_count] = rate;
	parent_clk_count++;
}

static void create_parent_tbl(struct clk *clk11)
{
	int i;
	struct clk *clk1;

	for (i = 0; i < clk11->num_parents; i++) {
		clk1 = clk_get(NULL, clk11->parent_names[i]);
		if (clk1) {
			add_to_parent_tbl(clk1->rate);
			create_parent_tbl(clk1);
		}
	}
}

static int find_best_parent(unsigned int rate, unsigned int *count)
{
	int i, j;
	unsigned int tmp = 0xff;
	unsigned int real_rate;
	int ret = 0;

	for (i = 0; i < parent_clk_count; i++) {
		for (j = 1; j < 15; j++) {
			real_rate = parent_clk_tbl[i] / j;
			if (rate > real_rate + ROUND_RATE_RANGE_MHZ)
				break;
			else if ((rate <= (real_rate + ROUND_RATE_RANGE_MHZ)) &&
				 (rate >= (real_rate - ROUND_RATE_RANGE_MHZ))) {
				/* round clock rate in 1MHz range */
				tmp = real_rate;
				*count = i;
				ret = 1;
				break;
			}
		}

		if (tmp != 0xff)
			break;
	}

	return ret;
}

static void get_current_parent_index(struct clk *clk,
	struct parent_index *index)
{
	struct clk *temp = clk;
	int i;

	while (!IS_ERR_OR_NULL(temp)) {
		for (i = 0; i < ARRAY_SIZE(parent2_clk_tbl); i++) {
			if (!strcmp(temp->name, parent2_clk_tbl[i])) {
				index->parent2 = i + DFC_FREQ_PARENT2_PLL1_416M;
				break;
			}
		}
		temp = clk_get_parent(temp);
	}
	if ((index->parent2 == DFC_FREQ_PARENT2_PLL1_416M) ||
		(index->parent2 == DFC_FREQ_PARENT2_PLL1_624M))
		index->parent1 = DFC_FREQ_PARENT1_DISPLAY1;

	if ((index->parent2 == DFC_FREQ_PARENT2_PLL3) ||
		(index->parent2 == DFC_FREQ_PARENT2_PLL2) ||
		(index->parent2 == DFC_FREQ_PARENT2_PLL1_832M))
		index->parent1 = DFC_FREQ_PARENT1_DSIPLL;
}

static int is_parent1_changed(struct mmphw_ctrl *ctrl)
{
	return strcmp(ctrl->dfc.current_parent1->name, ctrl->dfc.parent1->name);
}

static int is_parent2_changed(struct mmphw_ctrl *ctrl)
{
	return strcmp(ctrl->dfc.current_parent2->name, ctrl->dfc.parent2->name);
}

static void calculate_reg_value(struct mmp_dfc *dfc)
{
	unsigned int sclk;
	unsigned int apmu;
	u32  bclk_div, pclk_div;

	sclk = readl_relaxed(dfc->sclk_reg);
	apmu = readl_relaxed(dfc->apmu_reg);
	sclk &= ~(SCLK_SOURCE_SELECT_MASK |
		 DSI1_BITCLK_DIV_MASK |
		 CLK_INT_DIV_MASK);
	bclk_div = (dfc->parent2->rate + dfc->dsi_rate / 2) / dfc->dsi_rate;
	pclk_div = (dfc->parent2->rate + dfc->path_rate / 2) / dfc->path_rate;
	sclk |= DSI1_BITCLK_DIV(bclk_div) | CLK_INT_DIV(pclk_div);

	if (!strcmp(dfc->parent2->name,
		parent2_clk_tbl[DFC_FREQ_PARENT2_PLL1_624M - 1])) {
		apmu &= ~LCD_CLK_EN;
		sclk |= SCLK_SOURCE_SELECT(1);
	} else if (!strcmp(dfc->parent2->name,
		parent2_clk_tbl[DFC_FREQ_PARENT2_PLL1_416M - 1])) {
		apmu |= LCD_CLK_EN;
		sclk |= SCLK_SOURCE_SELECT(1);
	} else
		sclk |= SCLK_SOURCE_SELECT(3);

	dfc->apmu_value = apmu;
	dfc->sclk_value = sclk;
}


/* rate unit is Mhz */
static void dynamic_change_pll3(unsigned int rate, int type)
{
	struct clk *pll3, *pll3_vco;
	unsigned long i, pll3_rate, pll3vco_rate = 0;

	pll3 = clk_get(NULL, "pll3");
	pll3_vco = clk_get(NULL, "pll3_vco");

	switch (type) {
	case DFC_RESTORE_PLL3:
		if (!(clk_get_rate(pll3) == pll3_default)) {
			pll3_rate = pll3_default;
			pll3vco_rate = pll3_vco_default;
			clk_set_rate(pll3_vco, pll3vco_rate);
			clk_set_rate(pll3, pll3_rate);
		}
		break;
	case DFC_DISABLE_PLL3:
		clk_disable_unprepare(pll3);
		break;
	case DFC_ENABLE_PLL3:
		clk_prepare_enable(pll3);
		break;
	default:
		for (i = 0; i < ARRAY_SIZE(pll_post_div_tbl); i++) {
			pll3vco_rate = rate * pll_post_div_tbl[i].div;
			if ((pll3vco_rate > 1200) &&
					(pll3vco_rate < 2500))
				break;
		}

		if (i == ARRAY_SIZE(pll_post_div_tbl))
			BUG_ON("Multiplier is out of range\n");

		pll3_rate = rate * MHZ_TO_HZ;
		pll3vco_rate *= MHZ_TO_HZ;

		clk_set_rate(pll3_vco, pll3vco_rate);
		clk_set_rate(pll3, pll3_rate);
		break;
	}

//	usleep_range(500, 1000);
}

static int dynamic_change_lcd(struct mmphw_ctrl *ctrl, struct mmp_dfc *dfc)
{
	struct mmp_path *path = ctrl->path_plats[0].path;
	struct mmp_phy *phy = path->phy;
	struct mmp_dsi *dsi = (struct mmp_dsi *)phy->phy_data;
	struct mmphw_dsi_plat *dsi_plat =
		(struct mmphw_dsi_plat *)dsi->mmphw_dsi_plat;
	struct parent_index source;
	int count = 0;
	struct clk *dsi_clk = dsi_plat->clk;
	unsigned long flags;
	int ret = 0;

	memset(&source, 0, sizeof(source));
	get_current_parent_index(dsi_clk, &source);
	dfc->current_parent1 = clk_get(NULL,
		parent1_clk_tbl[source.parent1-1]);
	dfc->current_parent2 = clk_get(NULL,
		parent2_clk_tbl[source.parent2-1]);
	memcpy(&ctrl->dfc, dfc, sizeof(*dfc));
	calculate_reg_value(&ctrl->dfc);
	atomic_set(&ctrl->dfc.commit, 1);
	do {
		mmp_path_set_irq(path, 1);
		mmp_path_wait_special_vsync(path);
		mmp_path_set_irq(path, 0);
		count++;
	} while (atomic_read(&ctrl->dfc.commit) && (count < DFC_RETRY_TIMEOUT));

	spin_lock_irqsave(&ctrl->dfc.lock, flags);
	if (unlikely(count == DFC_RETRY_TIMEOUT)
		&& atomic_read(&ctrl->dfc.commit)) {
		atomic_set(&ctrl->dfc.commit, 0);
		spin_unlock_irqrestore(&ctrl->dfc.lock, flags);
		ret = EFAULT;
	} else {
		spin_unlock_irqrestore(&ctrl->dfc.lock, flags);
		if (is_parent1_changed(ctrl))
			clk_set_parent(ctrl->dfc.parent0, ctrl->dfc.parent1);
		if (is_parent2_changed(ctrl))
			clk_set_parent(ctrl->dfc.parent1, ctrl->dfc.parent2);
	}
	return ret;
}

static int dfc_prepare(struct mmphw_ctrl *ctrl, unsigned int rate)
{
	struct mmp_path *path = ctrl->path_plats[0].path;
	struct clk *path_clk = path_to_path_plat(path)->clk;
	struct mmp_phy *phy = path->phy;
	struct mmp_dsi *dsi = (struct mmp_dsi *)phy->phy_data;
	struct mmphw_dsi_plat *dsi_plat =
		(struct mmphw_dsi_plat *)dsi->mmphw_dsi_plat;
	struct parent_index target, source;
	int x, best_parent_count = 0;
	struct clk *dsi_clk = dsi_plat->clk;
	struct mmp_dfc *dfc;
	struct mmp_dfc temp;
	int ret = 0;

	memset(&target, 0, sizeof(target));
	memset(&temp, 0, sizeof(temp));
	memcpy(&temp, &ctrl->dfc, sizeof(temp));

	x = (dsi_plat->lanes > 2) ? 2 : 3;
	temp.dsi_rate = rate * MHZ_TO_HZ;
	temp.path_rate = (rate >> x) * MHZ_TO_HZ;
	temp.parent0 = clk_get(NULL, parent0_clk_tbl[0]);
	create_parent_tbl(path_clk);
	if (!find_best_parent(rate, &best_parent_count))
		temp.best_parent = DFC_FREQ_PARENT2_PLL_NONE;
	else {
		get_parent_index(parent_clk_tbl[best_parent_count], &target);
		temp.best_parent = target.parent2;
	}

	memset(&source, 0, sizeof(source));
	get_current_parent_index(dsi_clk, &source);
	temp.current_parent1 =
		clk_get(NULL, parent1_clk_tbl[source.parent1-1]);
	temp.current_parent2 =
		clk_get(NULL, parent2_clk_tbl[source.parent2-1]);

	while (!list_empty(&ctrl->dfc_list.queue)) {
		/* free the waitlist elements if any */
		dfc = list_first_entry(&ctrl->dfc_list.queue,
			struct mmp_dfc, queue);
		list_del(&dfc->queue);
		kfree(dfc);
	}

	switch (temp.best_parent) {
	case DFC_FREQ_PARENT2_PLL1_416M:
	case DFC_FREQ_PARENT2_PLL1_624M:
		/* prepare the first change */
		temp.parent2 =
			clk_get(NULL, parent2_clk_tbl[target.parent2-1]);
		temp.parent1 =
			clk_get(NULL, parent1_clk_tbl[target.parent1-1]);

		/* add dfc reqyest to list */
		dfc = kzalloc(sizeof(struct mmp_dfc),	GFP_KERNEL);
		if (dfc == NULL)
			goto prepare_fail;
		memcpy(dfc, &temp, sizeof(*dfc));
		dfc->name = DFC_CHANGE_LCD;
		list_add_tail(&dfc->queue, &ctrl->dfc_list.queue);

		/* prepare the second change */
		dfc = kzalloc(sizeof(struct mmp_dfc), GFP_KERNEL);
		if (dfc == NULL)
			goto prepare_fail;
		dfc->name = DFC_RESTORE_PLL3;
		list_add_tail(&dfc->queue, &ctrl->dfc_list.queue);
		break;
	case DFC_FREQ_PARENT2_PLL3:
		/* prepare enable pll3 change */
		dfc = kzalloc(sizeof(struct mmp_dfc), GFP_KERNEL);
		if (dfc == NULL)
			return -EFAULT;
		dfc->name = DFC_ENABLE_PLL3;
		list_add_tail(&dfc->queue, &ctrl->dfc_list.queue);

		/* prepare the first change */
		temp.parent2 =
			clk_get(NULL, parent2_clk_tbl[target.parent2-1]);
		temp.parent1 =
			clk_get(NULL, parent1_clk_tbl[target.parent1-1]);

		/* add dfc reqyest to list */
		dfc = kzalloc(sizeof(struct mmp_dfc), GFP_KERNEL);
		if (dfc == NULL)
			goto prepare_fail;
		memcpy(dfc, &temp, sizeof(*dfc));
		dfc->name = DFC_CHANGE_LCD;
		list_add_tail(&dfc->queue, &ctrl->dfc_list.queue);

		/* prepare disable pll3 change */
		dfc = kzalloc(sizeof(struct mmp_dfc), GFP_KERNEL);
		if (dfc == NULL)
			goto prepare_fail;
		dfc->name = DFC_DISABLE_PLL3;
		list_add_tail(&dfc->queue, &ctrl->dfc_list.queue);
		break;
	case DFC_FREQ_PARENT2_PLL_NONE:
		if (!strcmp(temp.current_parent2->name,
			parent2_clk_tbl[DFC_FREQ_PARENT2_PLL3 - 1])) {
			/* prepare the first change */
			temp.parent2 =
				clk_get(NULL, parent2_clk_tbl[DFC_FREQ_PARENT2_PLL1_416M - 1]);
			temp.parent1 =
				clk_get(NULL, parent1_clk_tbl[DFC_FREQ_PARENT1_DISPLAY1 - 1]);
			temp.dsi_rate = temp.parent2->rate;
			temp.path_rate = temp.dsi_rate >> x;
			/* add dfc reqyest to list */
			dfc = kzalloc(sizeof(struct mmp_dfc),	GFP_KERNEL);
			if (dfc == NULL)
				goto prepare_fail;
			memcpy(dfc, &temp, sizeof(*dfc));
			dfc->name = DFC_CHANGE_LCD;
			list_add_tail(&dfc->queue, &ctrl->dfc_list.queue);
		}

		/* prepare the second change */
		temp.dsi_rate = rate * MHZ_TO_HZ;
		temp.path_rate = (rate >> x) * MHZ_TO_HZ;
		dfc = kzalloc(sizeof(struct mmp_dfc),	GFP_KERNEL);
		if (dfc == NULL)
			goto prepare_fail;
		memcpy(dfc, &temp, sizeof(*dfc));
		dfc->name = DFC_CHANGE_PLL3;
		list_add_tail(&dfc->queue, &ctrl->dfc_list.queue);

		/* prepare enable pll3 change */
		dfc = kzalloc(sizeof(struct mmp_dfc), GFP_KERNEL);
		if (dfc == NULL)
			return -EFAULT;
		dfc->name = DFC_ENABLE_PLL3;
		list_add_tail(&dfc->queue, &ctrl->dfc_list.queue);

		/* prepare the third change */
		temp.parent2 =
			clk_get(NULL, parent2_clk_tbl[DFC_FREQ_PARENT2_PLL3 - 1]);
		temp.parent1 =
			clk_get(NULL, parent1_clk_tbl[DFC_FREQ_PARENT1_DSIPLL - 1]);
		dfc = kzalloc(sizeof(struct mmp_dfc), GFP_KERNEL);
		if (dfc == NULL)
			goto prepare_fail;
		memcpy(dfc, &temp, sizeof(*dfc));
		dfc->name = DFC_CHANGE_LCD;
		list_add_tail(&dfc->queue, &ctrl->dfc_list.queue);

		/* prepare disable pll3 change */
		dfc = kzalloc(sizeof(struct mmp_dfc), GFP_KERNEL);
		if (dfc == NULL)
			goto prepare_fail;
		dfc->name = DFC_DISABLE_PLL3;
		list_add_tail(&dfc->queue, &ctrl->dfc_list.queue);
		break;
	default:
		break;
	}
	return ret;
prepare_fail:
	while (!list_empty(&ctrl->dfc_list.queue)) {
		/* free the waitlist elements if any */
		dfc = list_first_entry(&ctrl->dfc_list.queue,
			struct mmp_dfc, queue);
		list_del(&dfc->queue);
		kfree(dfc);
	}
	return -EFAULT;
}

static int dfc_commit(struct mmphw_ctrl *ctrl)
{
	struct mmp_dfc *dfc;
	int ret = 0;

	while (!list_empty(&ctrl->dfc_list.queue)) {
		/* free the waitlist elements if any */
		dfc = list_first_entry(&ctrl->dfc_list.queue,
			struct mmp_dfc, queue);
		list_del(&dfc->queue);
		switch (dfc->name) {
		case DFC_CHANGE_LCD:
			ret = dynamic_change_lcd(ctrl, dfc);
			break;
		case DFC_DISABLE_PLL3:
		case DFC_ENABLE_PLL3:
		case DFC_CHANGE_PLL3:
		case DFC_RESTORE_PLL3:
			dynamic_change_pll3(dfc->dsi_rate / MHZ_TO_HZ, dfc->name);
			break;
		default:
			break;
		}
		kfree(dfc);
	}
	/* FIXME : msleep(100); Black screen issue from DEGAS
	 * Wait at least 3 frames for pending job & next succesive operation, do 6 frame sleep for more room
	 */
	msleep(100);
	return ret;
}

static void dfc_handle(void *data)
{
	u32 temp;

	struct device *dev = (struct device *)data;
	struct mmphw_ctrl *ctrl = dev_get_drvdata(dev);
	if (atomic_read(&ctrl->dfc.commit)) {
		/* disable sclk firstly */
		temp = readl_relaxed(ctrl->dfc.sclk_reg);
		temp |= 0x1 << 28;
		writel_relaxed(temp, ctrl->dfc.sclk_reg);
		/* change the source */
		writel_relaxed(ctrl->dfc.apmu_value, ctrl->dfc.apmu_reg);
		temp = (ctrl->dfc.sclk_value | (0x1 << 28));
		writel_relaxed(ctrl->dfc.sclk_value, ctrl->dfc.sclk_reg);
		/* enable sclk end */
		temp = readl_relaxed(ctrl->dfc.sclk_reg);
		temp &= ~(0x1 << 28);
		writel_relaxed(temp, ctrl->dfc.sclk_reg);
		atomic_set(&ctrl->dfc.commit, 0);
	}
}

#define    DIP_START   1
#define    DIP_END     0

int dfc_request(struct notifier_block *b, unsigned long val, void *v)
{
	struct mmp_path *path = mmp_get_path("mmp_pnpath");
	struct mmphw_ctrl *ctrl = path_to_ctrl(path);
	struct mmp_phy *phy = path->phy;
	struct mmp_dsi *dsi = (struct mmp_dsi *)phy->phy_data;
	struct mmphw_dsi_plat *dsi_plat =
		(struct mmphw_dsi_plat *)dsi->mmphw_dsi_plat;
	struct clk *dsi_clk = dsi_plat->clk;
	u32 rate;
	int ret;
	static int dfc_init_flag = 0;
	static int dfc_default_rate = 0;

	if (dfc_init_flag == 0){
		if (!IS_ERR(dsi_clk))
			dfc_default_rate = clk_get_rate(dsi_clk);
		else
			pr_info("Invalid dsi clk\n");
		pr_info("dfc_request: default MIPI clock = %d\n", dfc_default_rate);
		dfc_init_flag = 1;
	}

	if (val == DIP_START) {
		rate = *((u32 *)v);
		if (!rate) {
			pr_info("skip dfc_request\n");
			return NOTIFY_OK;
		}
		pr_info("dfc_request: rate = %d\n", rate);
	} else
		rate = dfc_default_rate / MHZ_TO_HZ;

	pr_info("dfc_request: val = %d, rate = %d\n", val, rate);
	mutex_lock(&ctrl->access_ok);
	if (path->status) {
		ret = dfc_prepare(ctrl, rate);
		if (ret < 0) {
			pr_info("dfc prepare failure\n");
			mutex_unlock(&ctrl->access_ok);
			return NOTIFY_BAD;
		}
		ret = dfc_commit(ctrl);
		if (ret < 0) {
			pr_info("dfc commit failure\n");
			mutex_unlock(&ctrl->access_ok);
			return NOTIFY_BAD;
		}
	} else {
		ctrl->dfc.old_rate = rate;
		atomic_set(&ctrl->dfc.delayed_commit, 1);
	}
	mutex_unlock(&ctrl->access_ok);

	return NOTIFY_OK;
}

static struct notifier_block dip_disp_notifier = {
	.notifier_call = dfc_request,
};

ssize_t freq_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct mmphw_ctrl *ctrl = dev_get_drvdata(dev);
	struct mmp_path *path = ctrl->path_plats[0].path;
	struct clk *path_clk = path_to_path_plat(path)->clk;
	struct clk *clk;
	unsigned long rate;
	struct mmp_phy *phy = path->phy;
	struct mmp_dsi *dsi = (struct mmp_dsi *)phy->phy_data;
	struct mmphw_dsi_plat *dsi_plat =
		(struct mmphw_dsi_plat *)dsi->mmphw_dsi_plat;

	struct clk *dsi_clk = dsi_plat->clk;
	int s = 0;

	mutex_lock(&ctrl->access_ok);
	clk = path_clk;
	while (!IS_ERR_OR_NULL(clk)) {
		rate = clk_get_rate(clk);
		pr_info("current clk parent : %s ...num:%d...rate: %lu..count : %d.\n",
			clk->name, clk->num_parents, rate, clk->enable_count);
		clk = clk_get_parent(clk);
	}
	s += sprintf(buf, "%d\n", clk_get_rate(dsi_clk) / MHZ_TO_HZ);
	mutex_unlock(&ctrl->access_ok);
	return s;
}

extern void ctrl_dfc_check(struct device *dev)
{
	struct mmphw_ctrl *ctrl = dev_get_drvdata(dev);
	struct mmp_path *path = ctrl->path_plats[0].path;
	int ret;

	if (path->status) {
		if (atomic_read(&ctrl->dfc.delayed_commit)) {
			pr_info("%s: for dipchannel debug\n", __FUNCTION__);
			atomic_set(&ctrl->dfc.delayed_commit, 0);
			ret = dfc_prepare(ctrl, ctrl->dfc.old_rate);
			if (ret < 0) {
				pr_info("dfc prepare failure\n");
			}
			ret = dfc_commit(ctrl);
			if (ret < 0)
				pr_info("dfc commit failure\n");
		}
	}
}

static ssize_t freq_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct mmphw_ctrl *ctrl = dev_get_drvdata(dev);
	struct mmp_path *path = ctrl->path_plats[0].path;
	unsigned long rate;
	int ret;

	mutex_lock(&ctrl->access_ok);
	if (size > 30) {
		mutex_unlock(&ctrl->access_ok);
		pr_err("%s size = %zd > max 30 chars\n", __func__, size);
		return size;
	}

	ret = (int)kstrtoul(buf, 0, &rate);
	if (ret < 0) {
		mutex_unlock(&ctrl->access_ok);
		dev_err(dev, "strtoul err.\n");
		return ret;
	}

	if (path->status) {
		atomic_set(&ctrl->dfc.delayed_commit, 0);
		ret = dfc_prepare(ctrl, rate);
		if (ret < 0) {
			pr_info("dfc prepare failure\n");
			mutex_unlock(&ctrl->access_ok);
			return size;
		}
		ret = dfc_commit(ctrl);
		if (ret < 0)
			pr_info("dfc commit failure\n");
	} else {
		ctrl->dfc.old_rate = rate;
		atomic_set(&ctrl->dfc.delayed_commit, 1);
	}
	mutex_unlock(&ctrl->access_ok);
	return size;
}
DEVICE_ATTR(freq, S_IRUGO | S_IWUSR, freq_show, freq_store);

void ctrl_dfc_init(struct device *dev)
{
	struct mmphw_ctrl *ctrl = dev_get_drvdata(dev);
	struct mmp_path *path = ctrl->path_plats[0].path;
	struct clk *path_clk = path_to_path_plat(path)->clk;
	struct mmp_vsync_notifier_node *dfc_notifier_node = NULL;

	dfc_notifier_node = devm_kzalloc(dev, sizeof(*dfc_notifier_node), GFP_KERNEL);
	if (dfc_notifier_node == NULL) {
		pr_err("dfc alloc failure\n");
		return;
	}

	dfc_notifier_node->cb_notify = dfc_handle;
	dfc_notifier_node->cb_data = dev;
	mmp_path_register_special_vsync_cb(path, dfc_notifier_node);

	create_parent_tbl(path_clk);
	atomic_set(&ctrl->dfc.commit, 0);
	atomic_set(&ctrl->dfc.delayed_commit, 0);
	spin_lock_init(&ctrl->dfc.lock);
	INIT_LIST_HEAD(&ctrl->dfc_list.queue);

	ctrl->dfc.apmu_reg = ioremap(AMPU_LCD , 4);
	ctrl->dfc.sclk_reg = ioremap(LCD_PN_SCLK, 4);

	pll3_vco_default = clk_get_rate(clk_get(NULL, "pll3_vco"));
	pll3_default = clk_get_rate(clk_get(NULL, "pll3"));

	device_create_file(dev, &dev_attr_freq);

	dip_register_notifier(&dip_disp_notifier, 0);
}
