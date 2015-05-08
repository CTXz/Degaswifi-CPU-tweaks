/*
 * linux/drivers/video/mmp/hw/mmp_vdma.c
 *
 * Copyright (C) 2013 Marvell Technology Group Ltd.
 * Authors:  Guoqing Li <ligq@marvell.com>
 *           Jing Xiang <jxiang@marvell.com>
 *           Baoyin Shan <byshan@marvell.com>
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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/pm_runtime.h>
#include "mmp_ctrl.h"
#include "mmp_vdma.h"

#define is_yuv420_fmt(pix_fmt) ((pix_fmt) == PIXFMT_YUV420P || \
		(pix_fmt) == PIXFMT_YVU420P || (pix_fmt) == PIXFMT_YUV420SP \
		|| (pix_fmt) == PIXFMT_YVU420SP)

#define INDEX_VID 1
#define INDEX_GFA 0
#define SUB_CHNNL_NUM_VID 3
#define SUB_CHNNL_NUM_GFA 1
/*
 * There were four vdma channels, including eight sub-channels
 * The relationship is like this:
 *       vdma0 <---> sub-channel0
 *       vdma1 <---> sub-channel1
 *       vdma2 <---> sub-channel2,3,4
 *       vdma3 <---> sub-channel5,6,7
 */
#define for_each_sub_channel(i, vdma_info, sub_ch_id) \
	for (i = 0, sub_ch_id = (vdma_info->vdma_id < 3) ? vdma_info->vdma_id \
			: 5; i < vdma_info->sub_ch_num; i++, sub_ch_id++)

static struct mmp_vdma *vdma;

static inline void channel_shadow_trigger(struct mmp_vdma_info *vdma_info)
{
	struct mmp_vdma_reg *vdma_reg =
		(struct mmp_vdma_reg *)(vdma->reg_base);
	u32 tmp;

	if (DISP_GEN4(vdma->version)) {
		tmp = readl_relaxed(&vdma_reg->main_ctrl) |
			SETTING_READY(vdma_info->vdma_id);
		writel_relaxed(tmp, &vdma_reg->main_ctrl);
	}
}

static void vdma_sram_free(struct mmp_vdma_info *vdma_info)
{
	vdma_info->status = VDMA_FREED;
	if (DISP_GEN4(vdma->version))
		return;
	gen_pool_free(vdma->pool, vdma_info->sram_vaddr, vdma_info->sram_size);
}

static u32 vdma_cal_line(struct mmp_vdma_info *vdma_info,
		struct mmp_win *win, u32 vdma_pix_fmt)
{
	u32 lines, least_lines, pitch, height, sram_size, mulfactor = 2;

	height = win->ysrc & 0xffff;
	pitch = win->pitch[0] & 0xffff;
	if (unlikely(!pitch)) {
		pr_warn("%s pitch null\n", __func__);
		goto failed;
	}
	/*
	 * FIXME: it indicates planar color format for vdma_pix_fmt != 0,
	 * the sram was divided into two or three parts:
	 *   yuv422planar: 2:1:1
	 *   yuv420planar: 2:1:1
	 *   yuv420semiplanar: 1:1
	 * we only calculate lines for Y
	 */
	sram_size = vdma_pix_fmt ? (vdma_info->sram_size >> 1) :
		vdma_info->sram_size;

	lines = (sram_size / pitch) & (~(mulfactor - 1));
	least_lines = is_yuv420_fmt(win->pix_fmt) ? 4 : 2;
	/* at least 2 lines */
	if (unlikely(lines < least_lines)) {
		pr_warn("%s SRAM too small for VDMA (>=2 lines)\n", __func__);
		goto failed;
	}

	if (lines > 64)
		lines = 64;
	while ((height % lines) && lines >= least_lines)
		lines -= mulfactor;
	if (unlikely(lines < least_lines)) {
		pr_warn("%s height:%d could not be divided by lines\n",
			__func__, height);
		goto failed;
	}

	return lines;
failed:
	if (!DISP_GEN4(vdma->version))
		vdma_sram_free(vdma_info);
	return 0;
}

static void vdma_squ_set(int overlay_id, int channel_num, u32 mask, u32 set)
{
	u8 layer2index[8] = {INDEX_VID, INDEX_GFA, INDEX_VID, INDEX_GFA,
		INDEX_VID, INDEX_GFA};
	u32 tmp, tmp1, tmp2;
	int i, index = layer2index[overlay_id];

	if (!DISP_GEN4(vdma->version)) {
		for (i = 0; i < channel_num; i++) {
			/* select SQU */
			squ_switch_index(overlay_id, index);
			/* configure SQU control */
			tmp1 = tmp2 = readl_relaxed(vdma->lcd_reg_base +
				SQULN_CTRL(overlay_id));
			tmp1 &= ~mask;
			tmp1 |= set;
			if (tmp1 != tmp2)
				writel_relaxed(tmp1, vdma->lcd_reg_base +
					SQULN_CTRL(overlay_id));
		}
	} else {
		if (overlay_is_vid(overlay_id)) {
			mask = mask << 8;
			set = set << 8;
		}
		tmp1 = tmp2 = readl_relaxed(vdma->lcd_reg_base +
				SQULN_CTRL(overlay_id));
		tmp1 &= ~mask;
		tmp1 |= set;
		if (tmp1 != tmp2)
			writel_relaxed(tmp1, vdma->lcd_reg_base +
					SQULN_CTRL(overlay_id));
	}
}

static void vdma_set_ctrl(int vdma_id, int channel_num, u32 mask, u32 set)
{
	struct mmp_vdma_reg *vdma_reg =
		(struct mmp_vdma_reg *)(vdma->reg_base);
	struct mmp_vdma_ch_reg *ch_reg;
	int i, sub_ch_id;
	u32 tmp1, tmp2;

	sub_ch_id = (vdma_id < 3) ? vdma_id : 5;
	for (i = 0; i < channel_num; i++, sub_ch_id++) {
		ch_reg = &vdma_reg->ch_reg[sub_ch_id];
		tmp1 = tmp2 = readl_relaxed(&ch_reg->ctrl);
		tmp1 &= ~mask;
		tmp1 |= set;
		if (tmp1 != tmp2)
			writel_relaxed(tmp1, &ch_reg->ctrl);
	}
}

static void vdma_set_on(struct mmp_vdma_info *vdma_info, int on)
{
	struct mmp_vdma_reg *vdma_reg =
		(struct mmp_vdma_reg *)(vdma->reg_base);
	int channel_num;
	u32 mask, enable, tmp;

	if (vdma_info->status == VDMA_FREED)
		return;

	mutex_lock(&vdma_info->access_ok);
	channel_num = overlay_is_vid(vdma_info->overlay_id) ?
		SUB_CHNNL_NUM_VID : SUB_CHNNL_NUM_GFA;
	if (!on) {
		/* clear enable bit */
		vdma_squ_set(vdma_info->overlay_id, channel_num, 1, 0);
		vdma_set_ctrl(vdma_info->vdma_id,
				channel_num, CH_ENA(1) | DC_ENA, 0);
	} else {
		vdma_squ_set(vdma_info->overlay_id, vdma_info->sub_ch_num, 1,
				1);
		if (!DISP_GEN4(vdma->version)) {
			mask = RD_BURST_SIZE(3) | WR_BURST_SIZE(3) |
				CH_ENA(1) | DC_ENA;
			enable = RD_BURST_SIZE(2) | WR_BURST_SIZE(2) |
				CH_ENA(1);
		} else {
			mask = AXI_RD_CNT_MAX(0x1f) | CH_ENA(1) | DC_ENA;
			enable = AXI_RD_CNT_MAX(0x10) | CH_ENA(1);
			/* FIXME: bypass decompression */
			tmp = readl_relaxed(&vdma_reg->dec_ctrl) | (1 << 31);
			writel_relaxed(tmp, &vdma_reg->dec_ctrl);
		}
		vdma_set_ctrl(vdma_info->vdma_id, vdma_info->sub_ch_num,
			mask, enable);
	}
	channel_shadow_trigger(vdma_info);
	mutex_unlock(&vdma_info->access_ok);
}

static u8 vdma_get_sub_ch_num(int pix_fmt)
{
	switch (pix_fmt) {
	case PIXFMT_YUV422P:
	case PIXFMT_YVU422P:
	case PIXFMT_YUV420P:
	case PIXFMT_YVU420P:
		return 3;
	case PIXFMT_YUV420SP:
	case PIXFMT_YVU420SP:
		return 2;
	default:
		return 1;
	}
}

static u32 vdma_get_pix_fmt(int pix_fmt)
{
	switch (pix_fmt) {
	case PIXFMT_YUV422P:
	case PIXFMT_YVU422P:
		return 4;
	case PIXFMT_YUV420P:
	case PIXFMT_YVU420P:
		return 6;
	case PIXFMT_YUV420SP:
	case PIXFMT_YVU420SP:
		return 7;
	default:
		return 0;
	}
}

static void vdma_ch_sel(struct mmp_vdma_info *vdma_info)
{
	u32 reg, vdma_rdy_id, vdma_map_id;
	u8 overlay_id = vdma_info->overlay_id;
	u8 vdma_id = vdma_info->vdma_id;
	/*
	 * mapped in LCD_PN2_SQULN2_CTRL @0x2f0: bit20:31
	 * PN graphic layer:
	 *   overlay_id: 1
	 *   bit21:20
	 * PN video layer:
	 *   overlay_id: 0
	 *   bit23:22
	 * TV graphic layer:
	 *   overlay_id: 3
	 *   bit25:24
	 * TV video layer:
	 *   overlay_id: 2
	 *   bit27:26
	 * PN2 graphic layer:
	 *   overlay_id: 5
	 *   bit29:28
	 * PN2 video layer:
	 *   overlay_id: 4
	 *   bit31:30
	 */
	u8 layer2vdma_rdy_sel[8] = {1, 0, 3, 2, 5, 4, 7, 6};
	/*
	 * mapped in LCD_PN2_SQULN2_CTRL @0x2f0: bit0:14
	 *  0x0: VDMA map to PN graphic
	 *  0x1: VDMA map to TV graphic
	 *  0x2: VDMA map to PN2 graphic
	 *  0x3: VDMA map to SCAL graphic
	 *  0x4: VDMA map to PN video
	 *  0x5: VDMA map to TV video
	 *  0x6: VDMA map to PN2 video
	 *  0x7: VDMA map to SCAL video
	 */
	u8 layer2vdma_map[8] = {4, 0, 5, 1, 6, 2, 7, 3};

	vdma_rdy_id = layer2vdma_rdy_sel[overlay_id];
	vdma_map_id = layer2vdma_map[overlay_id];
	reg = readl_relaxed(vdma->lcd_reg_base + LCD_PN2_SQULN2_CTRL);
	reg &= ~(VDMA_RDY_SEL_MASK(vdma_rdy_id) | VDMA_SEL_MASK(vdma_id));
	reg |= VDMA_RDY_SEL(vdma_rdy_id, vdma_id)
		| VDMA_SEL(vdma_map_id, vdma_id);
	writel_relaxed(reg, vdma->lcd_reg_base + LCD_PN2_SQULN2_CTRL);
}

static void vdma_set_dst_addr(struct mmp_vdma_info *vdma_info)
{
	struct mmp_vdma_reg *vdma_reg =
		(struct mmp_vdma_reg *)(vdma->reg_base);
	struct mmp_vdma_ch_reg *ch_reg;
	struct lcd_regs *lcdregs = (struct lcd_regs *)(vdma->lcd_reg_base +
			((vdma_info->overlay_id & 0x2) ? 0 : 0xc0));
	u32 tmp, dst_address = vdma_info->sram_paddr;
	u8 layer2index[8] = {INDEX_VID, INDEX_GFA, INDEX_VID, INDEX_GFA,
		INDEX_VID, INDEX_GFA};
	int index, sub_ch_id, i;

	index = layer2index[vdma_info->overlay_id];
	for_each_sub_channel(i, vdma_info, sub_ch_id) {
		/*
		 * FIXME: in order to support yuv planar format, we
		 * need to set three seperated dst address, y_addr,
		 * u_addr, v_addr. the ratio of sram size is:
		 * yuv422planar: 2:1:1
		 * yuv420planar: 2:1:1
		 * yuv420semiplanar: 1:1
		 */
		dst_address += i ? (vdma_info->sram_size >> i) : 0;
		if (!DISP_GEN4(vdma->version)) {
			/* select SQU */
			squ_switch_index(vdma_info->overlay_id, index);
			/* configure SQU control */
			tmp = readl_relaxed(vdma->lcd_reg_base +
				SQULN_CTRL(vdma_info->overlay_id));
			tmp &= 0x3f;
			tmp |= dst_address & (~0x3f);
			writel_relaxed(tmp, vdma->lcd_reg_base +
				SQULN_CTRL(vdma_info->overlay_id));
		} else {
			if (overlay_is_vid(vdma_info->overlay_id))
				writel_relaxed(dst_address & (~0x3f),
					i ? ((i & 1) ?
					&lcdregs->v_squln_u :
					&lcdregs->v_squln_v) :
					&lcdregs->v_squln_y);
			else
				writel_relaxed(dst_address & (~0x3f),
					&lcdregs->g_squln);
		}
		ch_reg = &vdma_reg->ch_reg[sub_ch_id];
		writel_relaxed(dst_address & (~0x3f), &ch_reg->dst_addr);
	}
}

static void vdma_set_win(struct mmp_vdma_info *vdma_info,
		struct mmp_win *win)
{
	struct mmp_vdma_reg *vdma_reg =
		(struct mmp_vdma_reg *)(vdma->reg_base);
	struct mmp_vdma_ch_reg *ch_reg;
	u32 pitch, size, src_sz, height, sram_lines, vdma_pix_fmt, tmp;
	int i, sub_ch_id, sub_ch_num;

	if (vdma_info->status == VDMA_FREED)
		return;

	vdma_pix_fmt = vdma_get_pix_fmt(win->pix_fmt);
	sram_lines = vdma_cal_line(vdma_info, win, vdma_pix_fmt);
	if (!sram_lines) {
		vdma_info->status = VDMA_FREED;
		return;
	}
	sub_ch_num = vdma_get_sub_ch_num(win->pix_fmt);
	if (vdma_info->sub_ch_num != sub_ch_num) {
		/*
		 * FIXME: if sub_ch_num changed,
		 * disable sub channels and re-enable
		 */
		vdma_info->sub_ch_num = sub_ch_num;
		vdma_set_on(vdma_info, 0);
		vdma_set_on(vdma_info, 1);
	}
	mutex_lock(&vdma_info->access_ok);
	vdma_ch_sel(vdma_info);
	vdma_set_dst_addr(vdma_info);
	vdma_squ_set(vdma_info->overlay_id, vdma_info->sub_ch_num,
			0x1f << 1, sram_lines - 2);
	height = win->ysrc & 0xffff;

	for_each_sub_channel(i, vdma_info, sub_ch_id) {
		ch_reg = &vdma_reg->ch_reg[sub_ch_id];
		pitch = win->pitch[i] & 0xffff;
		if ((i == 1) && is_yuv420_fmt(win->pix_fmt)) {
			height = height >> 1;
			sram_lines = sram_lines >> 1;
		}
		src_sz = pitch * height;
		size = height << 16 | pitch;
		/* FIXME: set dst and src with the same pitch */
		pitch |= pitch << 16;
		writel_relaxed(pitch, &ch_reg->pitch);
		writel_relaxed(src_sz, &ch_reg->src_size);
		writel_relaxed(size, &ch_reg->dst_size);
		tmp = readl_relaxed(&ch_reg->ctrl) &
			(~((0x7 << 16) | (0xff << 8)));
		tmp |= (sram_lines << 8) | (vdma_pix_fmt << 16);
		writel_relaxed(tmp, &ch_reg->ctrl);

	}
	channel_shadow_trigger(vdma_info);
	memcpy(&vdma_info->win_bakup, win, sizeof(struct mmp_win));
	mutex_unlock(&vdma_info->access_ok);
}

static void vdma_set_addr(struct mmp_vdma_info *vdma_info,
		struct mmp_addr *addr)
{
	struct mmp_vdma_reg *vdma_reg =
		(struct mmp_vdma_reg *)(vdma->reg_base);
	struct mmp_vdma_ch_reg *ch_reg;
	int i, sub_ch_id;

	if (vdma_info->status == VDMA_FREED)
		return;
	if (vdma_info->status == VDMA_RELEASED) {
		vdma_set_on(vdma_info, 0);
		vdma_sram_free(vdma_info);
	} else if (vdma_info->status == VDMA_REQUESTED) {
		vdma_set_win(vdma_info, &vdma_info->win_bakup);
		vdma_set_on(vdma_info, 1);
		vdma_info->status = VDMA_ALLOCATED;
	}

	mutex_lock(&vdma_info->access_ok);
	for_each_sub_channel(i, vdma_info, sub_ch_id) {
		ch_reg = &vdma_reg->ch_reg[sub_ch_id];
		writel_relaxed(addr->phys[i], &ch_reg->src_addr);
	}
	channel_shadow_trigger(vdma_info);
	mutex_unlock(&vdma_info->access_ok);
}

struct mmp_vdma_ops vdma_ops = {
	.set_on = vdma_set_on,
	.set_addr = vdma_set_addr,
	.set_win = vdma_set_win,
};

static int vdma_sram_alloc(struct mmp_vdma_info *vdma_info)
{
	if (DISP_GEN4(vdma->version))
		return 0;

	vdma_info->sram_vaddr =
		gen_pool_alloc(vdma->pool, vdma_info->sram_size);
	/* FIXME: VDMA HW support 32bit now, maybe 64bit someday */
	vdma_info->sram_paddr = gen_pool_virt_to_phys(vdma->pool,
		vdma_info->sram_vaddr);
	if (vdma_info->sram_paddr == -1) {
		dev_warn(vdma->dev,
			"%s: SRAM allocate failed\n", __func__);
		return -ENOMEM;
	} else {
		dev_info(vdma->dev,
			"VDMA%d allocated for layer%d, "
			"sram_paddr 0x%x, sram_size 0x%zx\n",
			vdma_info->vdma_id, vdma_info->overlay_id,
			vdma_info->sram_paddr, vdma_info->sram_size);
	}

	return 0;
}

static inline struct mmp_vdma_info *vdma_request(int overlay_id)
{
	struct mmp_vdma_info *vdma_info;
	int i;

	for (i = 0; i < vdma->vdma_channel_num; i++) {
		vdma_info = &vdma->vdma_info[i];
		if (vdma_info->vid == overlay_is_vid(overlay_id) &&
				vdma_info->status == VDMA_FREED)
			return vdma_info;
	}

	return NULL;
}

static inline struct mmp_vdma_info *vdma_is_allocated(int overlay_id)
{
	int i;

	for (i = 0; i < vdma->vdma_channel_num; i++)
		if (vdma->vdma_info[i].overlay_id == overlay_id &&
				vdma->vdma_info[i].status != VDMA_FREED)
			return &vdma->vdma_info[i];
	return NULL;
}

void mmp_vdma_free(int overlay_id)
{
	struct mmp_vdma_info *vdma_info;

	if (IS_ERR(vdma))
		return;
	if (DISP_GEN4(vdma->version))
		return;
	vdma_info = vdma_is_allocated(overlay_id);
	if (!vdma_info) {
		dev_warn(vdma->dev,
			"%s: no vdma allocated for the layer\n", __func__);
		return;
	}

	vdma_info->status = VDMA_RELEASED;
}

/*
 * allocate one VDMA channel
 *  called in lcd probe process, ioctl, and sysfs.
 */
struct mmp_vdma_info *mmp_vdma_alloc(int overlay_id, int sram_size)
{
	struct mmp_vdma_info *vdma_info;
	static int version_init;
	int i;

	if (!vdma)
		return NULL;
	if (!version_init) {
		vdma->version = readl_relaxed(vdma->lcd_reg_base + LCD_VERSION);
		version_init = 1;
		if (DISP_GEN4(vdma->version)) {
			/* FIXME: GEN4 owns the internal sram.
			 * sram allocation was fixed*/
			vdma->vdma_info[0].sram_paddr = 0;
			for (i = 1; i < vdma->vdma_channel_num; i++)
				vdma->vdma_info[i].sram_paddr = VDMA_SRAM_ALIGN(
					vdma->vdma_info[i - 1].sram_paddr +
					vdma->vdma_info[i - 1].sram_size);
		} else if (vdma->pool == NULL) {
			dev_err(vdma->dev, "isram pool not available\n");
			return NULL;
		}
	}
	vdma_info = vdma_is_allocated(overlay_id);
	if (vdma_info) {
		dev_warn(vdma->dev,
			"%s: the vdma for the layer has been allocated\n",
			__func__);
		return vdma_info;
	}

	vdma_info = vdma_request(overlay_id);
	if (!vdma_info) {
		dev_warn(vdma->dev,
			"%s: no vdma available for dma:%d\n",
			__func__, overlay_id);
		return NULL;
	}

	vdma_info->overlay_id = overlay_id;
	if (sram_size)
		vdma_info->sram_size = sram_size;
	if (vdma_sram_alloc(vdma_info))
		return NULL;
	vdma_info->status = VDMA_REQUESTED;

	return vdma_info;
}

static void vdma_init(struct mmp_mach_vdma_info *mi)
{
	int i;
	struct mmp_vdma_info *vdma_info;

	vdma->vdma_channel_num = mi->vdma_channel_num;
	vdma->name = mi->name;
	for (i = 0; i < mi->vdma_channel_num; i++) {
		vdma_info = &vdma->vdma_info[i];
		vdma_info->vdma_id = mi->vdma[i].id;
		vdma_info->sram_size = mi->vdma[i].sram_size;
		vdma_info->vid = mi->vdma[i].vid;
		vdma_info->ops = &vdma_ops;
		mutex_init(&vdma_info->access_ok);
		vdma_info->status = VDMA_FREED;
	}
}

static int mmp_vdma_probe(struct platform_device *pdev)
{
	struct mmp_mach_vdma_info *mi;
	struct mmp_mach_vdma_info dt_mi;
	struct vdma_channel vdma_ch[MAX_VDMA];
	struct resource *res0, *res1;
	struct device_node *np = pdev->dev.of_node, *child_np;
	struct gen_pool *pool;
	int i = 0, ret = 0;

	/* get resources from platform data */
	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res0 == NULL || res1 == NULL) {
		dev_err(&pdev->dev, "%s: no IO memory defined\n", __func__);
		return -ENOENT;
	}

	if (IS_ENABLED(CONFIG_OF)) {
		if (of_property_read_u32(np, "marvell,vdma-num",
					&dt_mi.vdma_channel_num)) {
			dev_err(&pdev->dev, "%s: vdma get num fail\n",
					__func__);
			return -EINVAL;
		}

		if (of_get_child_count(np) != dt_mi.vdma_channel_num) {
			dev_err(&pdev->dev, "%s: vdma channel not match\n",
					__func__);
			return -EINVAL;
		}
		for_each_child_of_node(np, child_np) {
			if (of_property_read_u32(child_np, "marvell,vdma-id",
						&vdma_ch[i].id))
				return -EINVAL;
			if (of_property_read_u32(child_np, "marvell,sram-size",
						&vdma_ch[i].sram_size))
				return -EINVAL;
			if (of_property_read_u32(child_np, "marvell,is_vid",
						&vdma_ch[i].vid))
				return -EINVAL;
			i++;
		}
		dt_mi.vdma = vdma_ch;
		mi = &dt_mi;
		pool = of_get_named_gen_pool(np, "isram", 0);
	} else {
		/* get configs from platform data */
		mi = pdev->dev.platform_data;
		if (mi == NULL) {
			dev_err(&pdev->dev, "%s: no platform data defined\n",
					__func__);
			return -EINVAL;
		}
		pool = dev_get_gen_pool(&pdev->dev);
	}

	vdma = devm_kzalloc(&pdev->dev, sizeof(struct mmp_vdma) +
			sizeof(struct mmp_vdma_info) * mi->vdma_channel_num,
			GFP_KERNEL);
	if (vdma == NULL) {
		dev_err(&pdev->dev, "vdma alloc fail\n");
		return -ENOMEM;
	}

	vdma_init(mi);
	vdma->pool = pool;
	vdma->dev = &pdev->dev;

	/* map registers.*/
	if (!devm_request_mem_region(vdma->dev, res0->start,
			resource_size(res0), vdma->name)) {
		dev_err(vdma->dev,
			"can't request region for resource %pR\n", res0);
		ret = -EINVAL;
		goto mem_fail;
	}

	vdma->reg_base = devm_ioremap_nocache(vdma->dev,
			res0->start, resource_size(res0));
	if (vdma->reg_base == NULL) {
		dev_err(vdma->dev, "%s: res0 %lx - %lx map failed\n", __func__,
			(unsigned long)res0->start, (unsigned long)res0->end);
		ret = -ENOMEM;
		goto ioremap_fail;
	}

	vdma->lcd_reg_base = devm_ioremap_nocache(vdma->dev,
			res1->start, resource_size(res1));
	if (vdma->lcd_reg_base == NULL) {
		dev_err(vdma->dev, "%s: res1%lx - %lx map failed\n", __func__,
			(unsigned long)res1->start, (unsigned long)res1->end);
		ret = -ENOMEM;
		goto ioremap1_fail;
	}

	/* get clock */
	vdma->clk = devm_clk_get(vdma->dev, "vdma_axi");
	if (IS_ERR(vdma->clk)) {
		dev_err(vdma->dev, "unable to get clk vdma_axi\n");
		ret = -ENOENT;
		goto ioremap1_fail;
	}
	platform_set_drvdata(pdev, vdma);
	vdma_dbg_init(vdma->dev);
#ifndef CONFIG_PM_RUNTIME
	clk_prepare_enable(vdma->clk);
#endif
	pm_runtime_enable(vdma->dev);
	pm_runtime_forbid(vdma->dev);
	dev_info(&pdev->dev, "vdma probe succeed\n");

	return 0;

ioremap1_fail:
	devm_iounmap(&pdev->dev, vdma->reg_base);
ioremap_fail:
	devm_release_mem_region(&pdev->dev, res0->start, resource_size(res0));
mem_fail:
	devm_kfree(&pdev->dev, vdma);
	dev_err(&pdev->dev, "vdma device init failed\n");

	return ret;
}

static int mmp_vdma_remove(struct platform_device *pdev)
{
	struct mmp_vdma *vdma = platform_get_drvdata(pdev);
	struct resource *res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct resource *res1 = platform_get_resource(pdev, IORESOURCE_MEM, 1);

	if (res0 == NULL || res1 == NULL) {
		dev_err(&pdev->dev, "%s: no IO memory defined\n", __func__);
		return -ENOENT;
	}

	vdma_dbg_uninit(vdma->dev);
	devm_iounmap(vdma->dev, vdma->lcd_reg_base);
	devm_iounmap(vdma->dev, vdma->reg_base);
	devm_release_mem_region(vdma->dev, res0->start, resource_size(res0));
	devm_release_mem_region(vdma->dev, res1->start, resource_size(res1));
	devm_clk_put(vdma->dev, vdma->clk);
	devm_kfree(vdma->dev, vdma);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mmp_vdma_dt_match[] = {
	{ .compatible = "marvell,mmp-vdma" },
	{},
};
#endif

#if defined(CONFIG_PM_SLEEP) || defined(CONFIG_PM_RUNTIME)
static int mmp_vdma_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mmp_vdma *vdma = platform_get_drvdata(pdev);

	clk_disable_unprepare(vdma->clk);

	return 0;
}

static int mmp_vdma_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mmp_vdma *vdma = platform_get_drvdata(pdev);

	clk_prepare_enable(vdma->clk);

	return 0;
}
#endif

static UNIVERSAL_DEV_PM_OPS(mmp_vdma_pm_ops, mmp_vdma_runtime_suspend,
		mmp_vdma_runtime_resume, NULL);

static struct platform_driver mmp_vdma_driver = {
	.probe          = mmp_vdma_probe,
	.remove         = mmp_vdma_remove,
	.driver         = {
		.name   = "mmp-vdma",
		.owner  = THIS_MODULE,
		.pm = &mmp_vdma_pm_ops,
		.of_match_table = of_match_ptr(mmp_vdma_dt_match),
	},
};

int mmp_vdma_register(void)
{
	return platform_driver_register(&mmp_vdma_driver);
}

void mmp_vdma_unregister(void)
{
	return platform_driver_unregister(&mmp_vdma_driver);
}

MODULE_AUTHOR("Guoqing Li<ligq@marvell.com>");
MODULE_DESCRIPTION("mmp vdma driver");
MODULE_LICENSE("GPL");
