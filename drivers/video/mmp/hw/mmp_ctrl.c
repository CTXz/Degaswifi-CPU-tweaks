/*
 * linux/drivers/video/mmp/hw/mmp_ctrl.c
 * Marvell MMP series Display Controller support
 *
 * Copyright (C) 2012 Marvell Technology Group Ltd.
 * Authors:  Guoqing Li <ligq@marvell.com>
 *          Lisa Du <cldu@marvell.com>
 *          Zhou Zhu <zzhu3@marvell.com>
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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>

#include "mmp_ctrl.h"
#define CREATE_TRACE_POINTS
#include <video/mmp_trace.h>

static inline void path_shadow_trigger(struct mmp_path *path)
{
	u32 tmp;

	if (DISP_GEN4(path_to_ctrl(path)->version)) {
		tmp = readl_relaxed(ctrl_regs(path) + LCD_SHADOW_CTRL) |
			SHADOW_TRIG(path->id);
		writel_relaxed(tmp, ctrl_regs(path) + LCD_SHADOW_CTRL);
	}
}

static void gamma_write(struct mmp_path *path, u32 addr, u32 gamma_id, u32 val)
{
	writel(val, ctrl_regs(path) + LCD_SPU_SRAM_WRDAT);
	val = (0x8 << 12) | (gamma_id << 8) | addr;
	writel(val, ctrl_regs(path) + LCD_SPU_SRAM_CTRL);
}

static u32 gamma_read(struct mmp_path *path, u32 addr, int gamma_id, int id)
{
	int count = 10000, val, pn2 = (id == 2) ? (1 << 16) : 0;

	val = pn2 | (0x0 << 12) | (gamma_id << 8) | addr;
	writel(val, ctrl_regs(path) + LCD_SPU_SRAM_CTRL);
	while ((readl(ctrl_regs(path) + LCD_SPU_SRAM_CTRL) & (1<<31))
		&& count--)
		;

	if (count > 0)
		val = readl(ctrl_regs(path) + (((path->id) & 1) ?
				LCD_TV_GAMMA_RDDAT : LCD_SPU_GAMMA_RDDAT))
			& CFG_GAMMA_RDDAT_MASK;
	else
		val = -1;

	return val;
}

static void gamma_dump(struct mmp_path *path, int lines)
{
	u32 i = 0, val;
	int id = path->id;

	if (!(readl_relaxed(ctrl_regs(path) + dma_ctrl(0, path->id))
		& CFG_GAMMA_ENA_MASK))
		dev_info(path->dev, "gamma correction not enabled yet\n");

	/* enable gamma correction table update */
	val = readl_relaxed(ctrl_regs(path) + LCD_SPU_SRAM_PARA1)
			| CFG_CSB_256x8_MASK;
	writel(val, ctrl_regs(path) + LCD_SPU_SRAM_PARA1);

	for (; i < lines; i++)
		dev_info(path->dev, "%3d: yr %3d, ug %3d, vb %3d\n", i,
			gamma_read(path, i, gamma_id_yr(id), id),
			gamma_read(path, i, gamma_id_ug(id), id),
			gamma_read(path, i, gamma_id_vb(id), id));

	val = readl_relaxed(ctrl_regs(path) + LCD_SPU_SRAM_PARA1)
			& ~CFG_CSB_256x8_MASK;
	writel(val, ctrl_regs(path) + LCD_SPU_SRAM_PARA1);
}

static int path_set_gamma(struct mmp_path *path, int flag, char *gamma_table)
{
	u32 tmp, val, i;

	/* disable gamma correction */
	tmp = readl_relaxed(ctrl_regs(path) + dma_ctrl(0, path->id));
	tmp &= ~CFG_GAMMA_ENA_MASK;
	tmp |= CFG_GAMMA_ENA(0);
	writel(tmp, ctrl_regs(path) + dma_ctrl(0, path->id));

	if (!(flag & GAMMA_ENABLE))
		goto dump;

	/* enable gamma correction table update */
	val = readl_relaxed(ctrl_regs(path) + LCD_SPU_SRAM_PARA1)
			| CFG_CSB_256x8_MASK;
	writel(val, ctrl_regs(path) + LCD_SPU_SRAM_PARA1);

	/* write gamma corrrection table */
	for (i = 0; i < GAMMA_TABLE_LEN; i++) {
		gamma_write(path, i, gamma_id_yr(path->id), gamma_table[i]);
		gamma_write(path, i, gamma_id_ug(path->id), gamma_table[i]);
		gamma_write(path, i, gamma_id_vb(path->id), gamma_table[i]);
	}

	val = readl_relaxed(ctrl_regs(path) + LCD_SPU_SRAM_PARA1)
			& ~CFG_CSB_256x8_MASK;
	writel(val, ctrl_regs(path) + LCD_SPU_SRAM_PARA1);

	/* enable gamma correction table */
	tmp = readl_relaxed(ctrl_regs(path) + dma_ctrl(0, path->id));
	tmp &= ~CFG_GAMMA_ENA_MASK;
	tmp |= CFG_GAMMA_ENA(1);
	writel(tmp, ctrl_regs(path) + dma_ctrl(0, path->id));

dump:
	if (flag & GAMMA_DUMP)
		gamma_dump(path, GAMMA_TABLE_LEN);

	return 0;
}

static int is_rbswap(struct mmp_overlay *overlay)
{
	int fmt = overlay->win.pix_fmt;
	if (fmt == PIXFMT_BGR565
		|| fmt == PIXFMT_BGR1555
		|| fmt == PIXFMT_BGR888PACK
		|| fmt == PIXFMT_BGR888UNPACK
		|| fmt == PIXFMT_BGRA888)
		return 1;
	else
		return 0;
}

static int overlay_set_colorkey_alpha(struct mmp_overlay *overlay,
					struct mmp_colorkey_alpha *ca)
{
	struct mmp_path *path = overlay->path;
	struct lcd_regs *regs = path_regs(overlay->path);
	struct mmphw_ctrl *ctrl = overlay_to_ctrl(overlay);
	u32 rb, x, layer, dma0, shift, r, b;

	dma0 = readl_relaxed(ctrl_regs(path) + dma_ctrl(0, path->id));
	shift = path->id ? 20 : 18;
	rb = layer = 0;
	r = ca->y_coloralpha;
	b = ca->v_coloralpha;

	/* reset to 0x0 to disable color key. */
	x = readl_relaxed(ctrl_regs(path) + dma_ctrl(1, path->id))
			& ~(CFG_COLOR_KEY_MASK | CFG_ALPHA_MODE_MASK
			| CFG_ALPHA_MASK);

	/* switch to color key mode */
	switch (ca->mode) {
	case FB_DISABLE_COLORKEY_MODE:
		/* do nothing */
		break;
	case FB_ENABLE_Y_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x1);
		break;
	case FB_ENABLE_U_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x2);
		break;
	case FB_ENABLE_V_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x4);
		dev_info(ctrl->dev,
			"V colorkey not supported, Chroma key instead\n");
		break;
	case FB_ENABLE_RGB_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x3);
		rb = 1;
		break;
	case FB_ENABLE_R_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x1);
		rb = 1;
		break;
	case FB_ENABLE_G_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x6);
		dev_info(ctrl->dev,
			"G colorkey not supported, Luma key instead\n");
		break;
	case FB_ENABLE_B_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x7);
		rb = 1;
		break;
	default:
		dev_info(ctrl->dev, "unknown mode\n");
		return -1;
	}

	/* switch to alpha path selection */
	switch (ca->alphapath) {
	case FB_VID_PATH_ALPHA:
		x |= CFG_ALPHA_MODE(0x0);
		layer = CFG_CKEY_DMA;
		if (rb) {
			rb = ((dma0 & CFG_DMA_SWAPRB_MASK) >> 4) ^
				(is_rbswap(overlay));
		}
		break;
	case FB_GRA_PATH_ALPHA:
		x |= CFG_ALPHA_MODE(0x1);
		layer = CFG_CKEY_GRA;
		if (rb) {
			rb = ((dma0 & CFG_GRA_SWAPRB_MASK) >> 12) ^
				(is_rbswap(overlay));
		}
		break;
	case FB_CONFIG_ALPHA:
		x |= CFG_ALPHA_MODE(0x2);
		rb = 0;
		break;
	default:
		dev_info(ctrl->dev, "unknown alpha path");
		return -1;
	}

	/* check whether DMA turn on RB swap for this pixelformat. */
	if (rb) {
		if (ca->mode == FB_ENABLE_R_COLORKEY_MODE) {
			x &= ~CFG_COLOR_KEY_MODE(0x1);
			x |= CFG_COLOR_KEY_MODE(0x7);
		}

		if (ca->mode == FB_ENABLE_B_COLORKEY_MODE) {
			x &= ~CFG_COLOR_KEY_MODE(0x7);
			x |= CFG_COLOR_KEY_MODE(0x1);
		}

		/* exchange r b fields. */
		r = ca->v_coloralpha;
		b = ca->y_coloralpha;

		/* only alpha_Y take effect, switch back from V */
		if (ca->mode == FB_ENABLE_RGB_COLORKEY_MODE) {
			r &= 0xffffff00;
			r |= (ca->y_coloralpha & 0xff);
		}
	}

	/* configure alpha */
	x |= CFG_ALPHA((ca->config & 0xff));
	writel(x, ctrl_regs(path) + dma_ctrl(1, path->id));
	writel(r, &regs->v_colorkey_y);
	writel(ca->u_coloralpha, &regs->v_colorkey_u);
	writel(b, &regs->v_colorkey_v);

	if (DISP_GEN4(ctrl->version) && (!path->id)) {
		shift = 1;
		x = readl_relaxed(ctrl_regs(path) + dma_ctrl(2, path->id));
		x &= ~(3 << shift);
		x |= layer << shift;
		writel(x, ctrl_regs(path) + dma_ctrl(2, path->id));
	} else if (path->id != 2) {
		/*
		 * enable DMA colorkey on graphics/video layer
		 * in panel/TV path. On GEN4 TV path keeps the same setting.
		 */
		x = readl_relaxed(ctrl_regs(path) + LCD_TV_CTRL1);
		x &= ~(3 << shift);
		x |= layer << shift;
		writel(x, ctrl_regs(path) + LCD_TV_CTRL1);
	}

	return 0;
}

static int path_set_alpha(struct mmp_overlay *overlay,
					struct mmp_alpha *pa)
{
	struct mmp_path *path = overlay->path;
	u32 alpha_mode = 0x0, mask, val;

	if ((pa->alphapath & ALPHA_TV_GRA_AND_TV_VID) ||
		(pa->alphapath & ALPHA_PN_GRA_AND_PN_VID)) {
		if (pa->config & ALPHA_PATH_VID_PATH_ALPHA)
			alpha_mode = 0x0;
		else if (pa->config & ALPHA_PATH_GRA_PATH_ALPHA)
			alpha_mode = 0x1;
		val = readl_relaxed(ctrl_regs(path) + dma_ctrl(1, path->id));
		mask = CFG_ALPHA_MODE_MASK;
		val &= ~mask;
		val |= CFG_ALPHA_MODE(alpha_mode);
		writel(val, ctrl_regs(path) + dma_ctrl(1, path->id));
		val = readl_relaxed(ctrl_regs(path) + LCD_AFA_ALL2ONE);
		mask = CFG_OVTOP_MASK | CFG_OVNXT_MASK;
		val &= ~mask;
		if (pa->alphapath == ALPHA_PN_GRA_AND_PN_VID)
			val |= CFG_OVTOP_SEL(1) | CFG_OVNXT_SEL(0);
		else
			val |= CFG_OVTOP_SEL(3) | CFG_OVNXT_SEL(2);
		writel(val, ctrl_regs(path) + LCD_AFA_ALL2ONE);
	} else if (pa->alphapath & ALPHA_PN_GRA_AND_TV_GRA) {
		if (pa->config & ALPHA_PATH_VID_PATH_ALPHA)
			alpha_mode = 0x0;
		else if (pa->config & ALPHA_PATH_GRA_PATH_ALPHA)
			alpha_mode = 0x1;
		val = readl_relaxed(ctrl_regs(path) + LCD_AFA_ALL2ONE);
		mask = CFG_GRATVG_MASK | CFG_OVTOP_MASK
			| CFG_OVNXT_MASK;
		val &= ~mask;
		val |= CFG_GRATVG_AMOD(alpha_mode) | CFG_OVTOP_SEL(1)
			| CFG_OVNXT_SEL(3);
		writel(val, ctrl_regs(path) + LCD_AFA_ALL2ONE);
	} else if (pa->alphapath & ALPHA_PN_GRA_AND_TV_VID) {
		if (pa->config & ALPHA_PATH_VID_PATH_ALPHA)
			alpha_mode = 0x0;
		else if (pa->config & ALPHA_PATH_GRA_PATH_ALPHA)
			alpha_mode = 0x1;
		val = readl_relaxed(ctrl_regs(path) + LCD_AFA_ALL2ONE);
		mask = CFG_GRATVD_MASK | CFG_OVTOP_MASK
			| CFG_OVNXT_MASK;
		val &= ~mask;
		val |= CFG_GRATVD_AMOD(alpha_mode) | CFG_OVTOP_SEL(1)
			| CFG_OVNXT_SEL(2);
		writel(val, ctrl_regs(path) + LCD_AFA_ALL2ONE);
	} else if (pa->alphapath & ALPHA_PN_VID_AND_TV_GRA) {
		if (pa->config & ALPHA_PATH_PN_PATH_ALPHA)
			alpha_mode = 0x0;
		else if (pa->config & ALPHA_PATH_TV_PATH_ALPHA)
			alpha_mode = 0x1;
		val = readl_relaxed(ctrl_regs(path) + LCD_AFA_ALL2ONE);
		mask = CFG_DMATVG_MASK | CFG_OVTOP_MASK
			| CFG_OVNXT_MASK;
		val &= ~mask;
		val |= CFG_DMATVG_AMOD(alpha_mode) | CFG_OVTOP_SEL(0)
			| CFG_OVNXT_SEL(3);
		writel(val, ctrl_regs(path) + LCD_AFA_ALL2ONE);
	} else if (pa->alphapath & ALPHA_PN_VID_AND_TV_VID) {
		if (pa->config & ALPHA_PATH_PN_PATH_ALPHA)
			alpha_mode = 0x0;
		else if (pa->config & ALPHA_PATH_TV_PATH_ALPHA)
			alpha_mode = 0x1;
		val = readl_relaxed(ctrl_regs(path) + LCD_AFA_ALL2ONE);
		mask = CFG_DMATVD_MASK | CFG_OVTOP_MASK
			| CFG_OVNXT_MASK;
		val &= ~mask;
		val |= CFG_DMATVD_AMOD(alpha_mode) | CFG_OVTOP_SEL(0)
			| CFG_OVNXT_SEL(2);
		writel(val, ctrl_regs(path) + LCD_AFA_ALL2ONE);
	} else {
		dev_info(overlay_to_ctrl(overlay)->dev, "unknown alpha path");
		return -1;
	}
	return 0;
}

static int overlay_set_path_alpha(struct mmp_overlay *overlay,
					struct mmp_alpha *pa)
{
	struct mmp_shadow *shadow = overlay->shadow;
	if (overlay->ops->trigger) {
		if (shadow && shadow->ops && shadow->ops->set_alpha)
			shadow->ops->set_alpha(shadow, pa);
	} else
		path_set_alpha(overlay, pa);

	return 0;
}

static int path_set_irq(struct mmp_path *path, int on)
{
	struct mmphw_ctrl *ctrl = path_to_ctrl(path);
	struct mmphw_path_plat *plat = path_to_path_plat(path);
	u32 tmp;
	u32 mask = display_done_imask(path->id) | vsync_imask(path->id);

	if(ctrl->clk)
		clk_prepare_enable(ctrl->clk);
	if (!on) {
		if (atomic_dec_and_test(&path->irq_en_ref)) {
			tmp = readl_relaxed(ctrl_regs(path) + SPU_IRQ_ENA);
			tmp &= ~mask;
			writel_relaxed(tmp, ctrl_regs(path) + SPU_IRQ_ENA);
			pm_qos_update_request(&plat->qos_idle_d1p,
					PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
			dev_dbg(path->dev, "%s eof intr off\n", path->name);
		}
	} else {
		if (!atomic_read(&path->irq_en_ref)) {
			/*
			 * FIXME: clear the interrupt status.
			 * It would not trigger handler if the status on
			 * before enable.
			 */
			pm_qos_update_request(&plat->qos_idle_d1p,
					plat->ctrl->lpm_qos_d1p);
			if (!DISP_GEN4(ctrl->version)) {
				tmp = readl(ctrl_regs(path) + SPU_IRQ_ISR);
				tmp &= ~mask;
			} else
				tmp = mask;
			writel_relaxed(tmp, ctrl_regs(path) + SPU_IRQ_ISR);
			tmp = readl(ctrl_regs(path) + SPU_IRQ_ENA);
			tmp |= mask;
			writel_relaxed(tmp, ctrl_regs(path) + SPU_IRQ_ENA);
			int retry = 0;
			while ((tmp != readl(ctrl_regs(path) + SPU_IRQ_ENA)) && retry < 10000) {
				retry++;
				writel_relaxed(tmp, ctrl_regs(path) + SPU_IRQ_ENA);
			}
			if (retry == 10000)
				pr_err("====== path_set_irq; write SPU_IRQ_ENA failture\n");
			dev_dbg(path->dev, "%s eof intr on, retry = %d\n", path->name, retry);
		}
		atomic_inc(&path->irq_en_ref);
	}
	if (ctrl->clk)
		clk_disable_unprepare(ctrl->clk);
	return atomic_read(&path->irq_en_ref) > 0;
}

static void handle_shadows(struct mmp_path *path)
{
	struct mmp_overlay *overlay;
	int i;
	/*Called in VSYNC interrupt, set shadow buffer to registers*/
	for (i = 0; i < path->overlay_num; i++) {
		overlay = &path->overlays[i];
		if (overlay->ops->trigger)
			overlay->ops->trigger(overlay);
		else {
			/*
			* Fix me, use hardware trigger
			* to do
			*/
		}
	}
}

static irqreturn_t ctrl_handle_irq(int irq, void *dev_id)
{
	struct mmphw_ctrl *ctrl = (struct mmphw_ctrl *)dev_id;
	struct mmp_path *path, *slave;
	u32 isr_en, disp_done, id, vsync_done;

	isr_en = readl_relaxed(ctrl->reg_base + SPU_IRQ_ISR) &
		readl_relaxed(ctrl->reg_base + SPU_IRQ_ENA);

	do {
		/* clear enabled irqs */
		if (!DISP_GEN4(ctrl->version))
			writel_relaxed(~isr_en, ctrl->reg_base + SPU_IRQ_ISR);
		else
			writel_relaxed(isr_en, ctrl->reg_base + SPU_IRQ_ISR);

		disp_done = isr_en & display_done_imasks;
		vsync_done = isr_en & vsync_imasks;

		for (id = 0; id < ctrl->path_num; id++) {
			path = ctrl->path_plats[id].path;
			if (path && path->irq_count.vsync_check) {
				path->irq_count.irq_count++;
				if (disp_done & display_done_imask(id))
					path->irq_count.dispd_count++;
				if (vsync_done & vsync_imask(id))
					path->irq_count.vsync_count++;
			}
		}

		if (!disp_done)
			return IRQ_HANDLED;
		for (id = 0; id < ctrl->path_num; id++) {
			if (!(disp_done & display_done_imask(id)))
				continue;
			path = ctrl->path_plats[id].path;
			if (path)
				slave = path->slave;
			if ((disp_done & display_done_imask(id)) &&
				(!(vsync_done & vsync_imasks))) {
				/* Some special usage need wait this kinde of vsync */
				if (path && path->special_vsync.handle_irq)
					path->special_vsync.handle_irq(
					&path->special_vsync);
			}
			if (path && path->vsync.handle_irq)
				path->vsync.handle_irq(&path->vsync);
			if (slave && slave->vsync.handle_irq)
				slave->vsync.handle_irq(&slave->vsync);
			if ((disp_done & display_done_imask(id)) &&
				(!(vsync_done & vsync_imasks))) {
				spin_lock(&path->commit_lock);
				if (path && atomic_read(&path->commit)) {
					handle_shadows(path);
					/* update slave path */
					if (path->slave)
						handle_shadows(path->slave);
					atomic_set(&path->commit, 0);
					trace_commit(path, 0);
				}
				spin_unlock(&path->commit_lock);
			}
		}
	} while ((isr_en = readl_relaxed(ctrl->reg_base + SPU_IRQ_ISR) &
				readl_relaxed(ctrl->reg_base + SPU_IRQ_ENA)));

	return IRQ_HANDLED;
}

static u32 fmt_to_reg(int overlay_id, int pix_fmt)
{
	u32 rbswap = 0, uvswap = 0, yuvswap = 0,
		csc_en = 0, val = 0,
		vid = overlay_is_vid(overlay_id);

	switch (pix_fmt) {
	case PIXFMT_RGB565:
	case PIXFMT_RGB1555:
	case PIXFMT_RGB888PACK:
	case PIXFMT_RGB888UNPACK:
	case PIXFMT_RGBA888:
		rbswap = 1;
		break;
	case PIXFMT_VYUY:
	case PIXFMT_YVU422P:
	case PIXFMT_YVU420P:
		uvswap = 1;
		break;
	case PIXFMT_YUYV:
		yuvswap = 1;
		break;
	default:
		break;
	}

	switch (pix_fmt) {
	case PIXFMT_RGB565:
	case PIXFMT_BGR565:
		break;
	case PIXFMT_RGB1555:
	case PIXFMT_BGR1555:
		val = 0x1;
		break;
	case PIXFMT_RGB888PACK:
	case PIXFMT_BGR888PACK:
		val = 0x2;
		break;
	case PIXFMT_RGB888UNPACK:
	case PIXFMT_BGR888UNPACK:
		val = 0x3;
		break;
	case PIXFMT_RGBA888:
	case PIXFMT_BGRA888:
		val = 0x4;
		break;
	case PIXFMT_UYVY:
	case PIXFMT_VYUY:
	case PIXFMT_YUYV:
		val = 0x5;
		csc_en = 1;
		break;
	case PIXFMT_YUV422P:
	case PIXFMT_YVU422P:
		val = 0x6;
		csc_en = 1;
		break;
	case PIXFMT_YUV420P:
	case PIXFMT_YVU420P:
		val = 0x7;
		csc_en = 1;
		break;
	case PIXFMT_YUV420SP:
	case PIXFMT_YVU420SP:
		val = 0xc;
		csc_en = 1;
		break;
	default:
		break;
	}

	return (dma_palette(0) | dma_fmt(vid, val) |
		dma_swaprb(vid, rbswap) | dma_swapuv(vid, uvswap) |
		dma_swapyuv(vid, yuvswap) | dma_csc(vid, csc_en));
}

static void overlay_set_fmt(struct mmp_overlay *overlay)
{
	u32 tmp;
	struct mmp_path *path = overlay->path;
	int overlay_id = overlay->id;

	tmp = readl_relaxed(ctrl_regs(path) + dma_ctrl(0, path->id));
	tmp &= ~dma_mask(overlay_is_vid(overlay_id));
	tmp |= fmt_to_reg(overlay_id, overlay->win.pix_fmt);
	writel_relaxed(tmp, ctrl_regs(path) + dma_ctrl(0, path->id));
	if (is_420sp(overlay->win.pix_fmt)) {
		tmp = readl_relaxed(ctrl_regs(path) + LCD_YUV420SP_FMT_CTRL);
		tmp &= ~SWAP_420SP(path->id);
		if (overlay->win.pix_fmt == PIXFMT_YVU420SP)
			tmp |= SWAP_420SP(path->id);
		writel_relaxed(tmp, ctrl_regs(path) + LCD_YUV420SP_FMT_CTRL);
	}
}

static void overlay_set_win(struct mmp_overlay *overlay, struct mmp_win *win)
{
	struct lcd_regs *regs = path_regs(overlay->path);
	struct mmphw_ctrl *ctrl = overlay_to_ctrl(overlay);
	struct mmp_vdma_info *vdma;
	int overlay_id = overlay->id;

	/* assert win supported */
	memcpy(&overlay->win, win, sizeof(struct mmp_win));

	mutex_lock(&overlay->access_ok);

	if (overlay_is_vid(overlay_id)) {
		writel_relaxed(win->pitch[0], &regs->v_pitch_yc);
		writel_relaxed(win->pitch[2] << 16 |
				win->pitch[1], &regs->v_pitch_uv);

		writel_relaxed((win->ysrc << 16) | win->xsrc, &regs->v_size);
		writel_relaxed((win->ydst << 16) | win->xdst, &regs->v_size_z);
		writel_relaxed(win->ypos << 16 | win->xpos, &regs->v_start);
	} else {
		writel_relaxed(win->pitch[0], &regs->g_pitch);

		writel_relaxed((win->ysrc << 16) | win->xsrc, &regs->g_size);
		if (!DISP_GEN4(ctrl->version))
			writel_relaxed((win->ydst << 16) | win->xdst,
					&regs->g_size_z);
		else {
			if (win->xsrc != win->xdst || win->ysrc != win->ydst)
				dev_warn(ctrl->dev,
					"resize not support for graphic "
					"layer of GEN4\n");
		}
		writel_relaxed(win->ypos << 16 | win->xpos, &regs->g_start);
	}

	overlay_set_fmt(overlay);
	path_shadow_trigger(overlay->path);
	vdma = overlay->vdma;
	if (vdma && vdma->ops && vdma->ops->set_win)
		vdma->ops->set_win(vdma, win);
	mutex_unlock(&overlay->access_ok);
}

static void dmafetch_onoff(struct mmp_overlay *overlay, int on)
{
	int overlay_id = overlay->id;
	u32 mask = overlay_is_vid(overlay_id) ? CFG_DMA_ENA_MASK :
		CFG_GRA_ENA_MASK;
	u32 enable = overlay_is_vid(overlay_id) ? CFG_DMA_ENA(1) :
		CFG_GRA_ENA(1);
	u32 tmp;
	struct mmp_path *path = overlay->path;
	struct mmp_vdma_info *vdma;

	vdma = overlay->vdma;
	if (vdma && vdma->ops && vdma->ops->set_on)
		vdma->ops->set_on(vdma, on);
	/* dma enable control */
	tmp = readl_relaxed(ctrl_regs(path) + dma_ctrl(0, path->id));
	tmp &= ~mask;
	tmp |= (on ? enable : 0);
	writel(tmp, ctrl_regs(path) + dma_ctrl(0, path->id));

	path_shadow_trigger(path);
}

static void path_enabledisable(struct mmp_path *path, int on)
{
	struct mmphw_path_plat *plat = path_to_path_plat(path);
	struct clk *clk = plat->clk;
	struct mmp_apical_info *apical;
	struct mmp_path *slave = path->slave;
	u32 tmp, dma_ctrl1;

	if (!clk)
		return;

	/* path enable control */
	tmp = readl_relaxed(ctrl_regs(path) + intf_ctrl(path->id));
	tmp &= ~CFG_DUMB_ENA_MASK;
	tmp |= (on ? CFG_DUMB_ENA(1) : 0);

	apical = path->apical;
	if (on) {
		pm_qos_update_request(&plat->qos_idle,
					plat->ctrl->lpm_qos);
		clk_prepare_enable(clk);
		writel_relaxed(tmp, ctrl_regs(path) + intf_ctrl(path->id));
		if (slave) {
			/* If slave path use the same CLK with master path,
			 * sync the timing
			 */
			dma_ctrl1 = readl_relaxed(ctrl_regs(slave) +
				dma_ctrl(1, slave->id));
			dma_ctrl1 &= ~CFG_TMSYNC_ENA_MASK;
			dma_ctrl1 |= CFG_TMSYNC_ENA(1);
			writel_relaxed(dma_ctrl1, ctrl_regs(slave) +
				dma_ctrl(1, slave->id));
			dma_ctrl1 &= ~CFG_TMSYNC_ENA_MASK;
			writel_relaxed(dma_ctrl1, ctrl_regs(slave) +
				dma_ctrl(1, slave->id));
		}
		if (apical && apical->ops && apical->ops->set_on)
			apical->ops->set_on(apical, 1);
		path_set_irq(path, 1);
	} else {
		writel_relaxed(tmp, ctrl_regs(path) + intf_ctrl(path->id));
		if (apical && apical->ops && apical->ops->set_on)
			apical->ops->set_on(apical, 0);
		pm_qos_update_request(&plat->qos_idle,
					PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
		path_set_irq(path, 0);
		clk_disable_unprepare(clk);
	}

	path_shadow_trigger(path);
}

static void path_set_timing(struct mmp_path *path)
{
	struct lcd_regs *regs = path_regs(path);
	u32 total_x, total_y, vsync_ctrl, tmp, mask, path_clk_req,
		link_config = path_to_path_plat(path)->link_config,
		h_porch, sync_ratio, master_mode;
	struct mmp_mode *mode = &path->mode;
	struct clk *clk = path_to_path_plat(path)->clk;
	struct mmphw_ctrl *ctrl = path_to_ctrl(path);

	if (PATH_OUT_PARALLEL != path->output_type
			&& (!path->phy || !path->phy->get_sync_val)) {
		dev_err(path->dev, "phy invalid\n");
		return;
	}

	/* polarity of timing signals */
	tmp = readl_relaxed(ctrl_regs(path) + intf_ctrl(path->id));
	tmp &= CFG_DUMB_ENA_MASK;
	tmp |= mode->vsync_invert ? 0x8 : 0;
	tmp |= mode->hsync_invert ? 0x4 : 0;
	tmp |= link_config & CFG_DUMBMODE_MASK;
	writel_relaxed(tmp, ctrl_regs(path) + intf_ctrl(path->id));

	/* interface rb_swap setting */
	tmp = readl_relaxed(ctrl_regs(path) + intf_rbswap_ctrl(path->id)) &
		(~(CFG_INTFRBSWAP_MASK));
	tmp |= link_config & CFG_INTFRBSWAP_MASK;
	writel_relaxed(tmp, ctrl_regs(path) + intf_rbswap_ctrl(path->id));

	/* x/y res, porch, sync */
	if (PATH_OUT_PARALLEL == path->output_type) {
		h_porch = (mode->left_margin << 16) | mode->right_margin;
		total_x = mode->xres + mode->left_margin + mode->right_margin +
			mode->hsync_len;
		path_clk_req = mode->pixclock_freq;
		vsync_ctrl = (mode->left_margin << 16) | (mode->left_margin);
	} else {
		sync_ratio = path->phy->get_sync_val(path->phy, PHY_SYNC_RATIO);
		h_porch = (mode->xres + mode->right_margin) * sync_ratio / 10 -
			mode->xres;
		h_porch |= (mode->left_margin * sync_ratio / 10) << 16;
		total_x = (mode->xres + mode->left_margin + mode->right_margin +
			mode->hsync_len) * sync_ratio / 10;
		vsync_ctrl = (((mode->xres + mode->right_margin) *
					sync_ratio / 10) << 16) |
			((mode->xres + mode->right_margin) * sync_ratio / 10);

		path_clk_req = path->phy->get_sync_val(path->phy,
			PHY_SYNC_CLKREQ);

		/* master mode setting of dsi phy */
		master_mode = path->phy->get_sync_val(path->phy,
			PHY_SYNC_MASTER_MODE);
		tmp = readl_relaxed(ctrl_regs(path) +
				(DISP_GEN4(ctrl->version) ?
				TIMING_MASTER_CONTROL_GEN4 :
				 TIMING_MASTER_CONTROL));
		if (!DISP_GEN4(ctrl->version))
			mask = MASTER_ENH(path->id) | MASTER_ENV(path->id) |
				DSI_START_SEL_MASK(path->id);
		else
			mask = MASTER_ENH_GEN4(path->id) |
				MASTER_ENV_GEN4(path->id) |
				DSI_START_SEL_MASK_GEN4(path->id);

		tmp &= ~mask;
		if (master_mode) {
			/*
			 * FIXME: only support DSI1, need dsi id and
			 * active panel id if more DSI added
			 */
			if (!DISP_GEN4(ctrl->version))
				tmp |= MASTER_ENH(path->id) |
					MASTER_ENV(path->id) |
					DSI_START_SEL(path->id, 0, 0);
			else
				tmp |= MASTER_ENH_GEN4(path->id) |
					MASTER_ENV_GEN4(path->id) |
					DSI_START_SEL_GEN4(path->id, 0);
		}
		writel_relaxed(tmp, ctrl_regs(path) +
				(DISP_GEN4(ctrl->version) ?
				TIMING_MASTER_CONTROL_GEN4 :
				 TIMING_MASTER_CONTROL));
	}
	writel_relaxed((mode->yres << 16) | mode->xres, &regs->screen_active);
	writel_relaxed(h_porch, &regs->screen_h_porch);
	writel_relaxed((mode->upper_margin << 16) | mode->lower_margin,
		&regs->screen_v_porch);
	total_y = mode->yres + mode->upper_margin + mode->lower_margin +
		mode->vsync_len;
	writel_relaxed((total_y << 16) | total_x, &regs->screen_size);
	writel_relaxed(vsync_ctrl, &regs->vsync_ctrl);

	/* set path_clk */
	if (clk && path_clk_req)
		clk_set_rate(clk, path_clk_req);
}

static void path_onoff(struct mmp_path *path, int on)
{
	if (path->status == on) {
		dev_info(path->dev, "path %s is already %s\n",
				path->name, stat_name(path->status));
		return;
	}

	mutex_lock(&path->access_ok);
	if (on) {
		path_enabledisable(path, 1);
		if (path->phy && path->phy->set_onoff)
			/* panel onoff would be called in phy onoff if exist */
			path->phy->set_onoff(path->phy, 1);
		else if (path->panel && path->panel->set_onoff)
			path->panel->set_onoff(path->panel, 1);
	} else {
		if (path->phy && path->phy->set_onoff)
			/* panel onoff would be called in phy onoff if exist */
			path->phy->set_onoff(path->phy, 0);
		else if (path->panel && path->panel->set_onoff)
			path->panel->set_onoff(path->panel, 0);
		path_enabledisable(path, 0);
	}
	path->status = on;

	mutex_unlock(&path->access_ok);
}

static void set_onoff(struct mmp_overlay *overlay, int on)
{
	struct mmphw_ctrl *ctrl = path_to_ctrl(overlay->path);

	mutex_lock(&ctrl->access_ok);

	overlay->status = on;

	dmafetch_onoff(overlay, on);

	if (overlay->path->ops.check_status(overlay->path)
			!= overlay->path->status)
		path_onoff(overlay->path, on);
	mutex_unlock(&ctrl->access_ok);

}

static void overlay_set_dma_onoff(struct mmp_overlay *overlay, int on)
{
	struct mmp_shadow *shadow = overlay->shadow;
	overlay->status = on;
	if (overlay->ops->trigger) {
		if (shadow && shadow->ops && shadow->ops->set_dmaonoff)
			shadow->ops->set_dmaonoff(shadow, on);
	} else
		dmafetch_onoff(overlay, on);
}

static void overlay_set_onoff(struct mmp_overlay *overlay, int on)
{
#ifdef CONFIG_MMP_DISP_DFC
	struct mmp_path *path = overlay->path;
	struct mmphw_ctrl *ctrl = path_to_ctrl(path);
#endif

	mutex_lock(&overlay->access_ok);
	if (on) {
		if (!atomic_read(&overlay->on_count))
			set_onoff(overlay, on);
		atomic_inc(&overlay->on_count);
		dev_dbg(overlay_to_ctrl(overlay)->dev, "set on: count %d\n",
				atomic_read(&overlay->on_count));
#ifdef CONFIG_MMP_DISP_DFC
		ctrl_dfc_check(ctrl->dev);
#endif
	} else {
		if (atomic_dec_and_test(&overlay->on_count))
			set_onoff(overlay, on);
		dev_dbg(overlay_to_ctrl(overlay)->dev, "set off: count %d\n",
				atomic_read(&overlay->on_count));
	}
	mutex_unlock(&overlay->access_ok);
}

static void overlay_reduced_onoff(struct mmp_overlay *overlay, int on)
{
	struct mmp_path *path = overlay->path;

	mutex_lock(&overlay->access_ok);
	overlay->status = on;
	if (!atomic_read(&overlay->on_count)) {
		path_enabledisable(path, 1);
		if (path->phy && path->phy->reduced_onoff)
			path->phy->reduced_onoff(path->phy, 1);
		if (path->panel && path->panel->reduced_onoff)
			path->panel->reduced_onoff(path->panel, 1);
		path->status = on;
	}
	atomic_inc(&overlay->on_count);
	dev_dbg(overlay_to_ctrl(overlay)->dev, "set on: count %d\n",
			atomic_read(&overlay->on_count));
	mutex_unlock(&overlay->access_ok);
}

static int overlay_set_addr(struct mmp_overlay *overlay, struct mmp_addr *addr)
{
	struct lcd_regs *regs = path_regs(overlay->path);
	struct mmp_vdma_info *vdma;
	int overlay_id = overlay->id;

	/* FIXME: assert addr supported */
	memcpy(&overlay->addr, addr, sizeof(struct mmp_addr));

	if (overlay_is_vid(overlay_id)) {
		writel_relaxed(addr->phys[0], &regs->v_y0);
		writel_relaxed(addr->phys[1], &regs->v_u0);
		writel_relaxed(addr->phys[2], &regs->v_v0);
	} else
		writel_relaxed(addr->phys[0], &regs->g_0);

	path_shadow_trigger(overlay->path);
	vdma = overlay->vdma;
	if (vdma && vdma->ops && vdma->ops->set_addr)
		vdma->ops->set_addr(vdma, addr);
	return overlay->addr.phys[0];
}

static void overlay_set_vsmooth_en(struct mmp_overlay *overlay, int en)
{
	struct mmp_path *path = overlay->path;
	struct mmphw_ctrl *ctrl = path_to_ctrl(path);
	u32 tmp;

	if (!DISP_GEN4(ctrl->version))
		return;
	/* only video layer support vertical smooth */
	if (!overlay_is_vid(overlay->id))
		return;
	mutex_lock(&overlay->access_ok);
	tmp = readl_relaxed(ctrl_regs(path) + dma_ctrl(2, path->id));
	tmp &= ~1;
	tmp |= !!en;
	writel(tmp, ctrl_regs(path) + dma_ctrl(2, path->id));
	mutex_unlock(&overlay->access_ok);
}

static int overlay_set_surface(struct mmp_overlay *overlay,
	struct mmp_win *win, struct mmp_addr *addr)
{
	struct lcd_regs *regs = path_regs(overlay->path);
	struct mmp_vdma_info *vdma;
	struct mmp_shadow *shadow;
	int overlay_id = overlay->id;

	/* FIXME: assert addr supported */
	memcpy(&overlay->addr, addr, sizeof(struct mmp_addr));
	memcpy(&overlay->win, win, sizeof(struct mmp_win));

	vdma = overlay->vdma;
	if (vdma && vdma->ops && vdma->ops->set_addr)
		vdma->ops->set_addr(vdma, addr);
	if (vdma && vdma->ops && vdma->ops->set_win)
		vdma->ops->set_win(vdma, win);

	shadow = overlay->shadow;
	/* Check whether there is software trigger */
	if (overlay->ops->trigger) {
		if (shadow && shadow->ops && shadow->ops->set_surface)
			shadow->ops->set_surface(shadow, win, addr);
	} else {
		if (overlay_is_vid(overlay_id)) {
			writel_relaxed(addr->phys[0], &regs->v_y0);
			writel_relaxed(addr->phys[1], &regs->v_u0);
			writel_relaxed(addr->phys[2], &regs->v_v0);
		} else
			writel_relaxed(addr->phys[0], &regs->g_0);

		if (overlay_is_vid(overlay_id)) {
			writel_relaxed(win->pitch[0], &regs->v_pitch_yc);
			writel_relaxed(win->pitch[2] << 16 |
					win->pitch[1], &regs->v_pitch_uv);

			writel_relaxed((win->ysrc << 16) | win->xsrc,
				&regs->v_size);
			writel_relaxed((win->ydst << 16) | win->xdst,
				&regs->v_size_z);
			writel_relaxed(win->ypos << 16 | win->xpos,
				&regs->v_start);
		} else {
			writel_relaxed(win->pitch[0], &regs->g_pitch);

			writel_relaxed((win->ysrc << 16) | win->xsrc,
				&regs->g_size);
			writel_relaxed((win->ydst << 16) | win->xdst,
				&regs->g_size_z);
			writel_relaxed(win->ypos << 16 | win->xpos,
				&regs->g_start);
		}

		overlay_set_fmt(overlay);

	}
	return overlay->addr.phys[0];
}

static void overlay_trigger(struct mmp_overlay *overlay)
{
	struct lcd_regs *regs = path_regs(overlay->path);
	int overlay_id = overlay->id;
	struct mmp_shadow *shadow = overlay->shadow;
	struct mmp_addr *addr;
	struct mmp_win *win;
	struct mmp_shadow_dma *dma;
	struct mmp_shadow_buffer *buffer;
	struct mmp_shadow_alpha *alpha;

	if (!list_empty(&shadow->buffer_list.queue)) {
		buffer = list_first_entry(
				&shadow->buffer_list.queue,
				struct mmp_shadow_buffer, queue);
		if (buffer)
			list_del(&buffer->queue);

		if (buffer && (buffer->flags & UPDATE_ADDR)) {
			addr = &buffer->addr;
			trace_addr(overlay->id, addr, 0);
			if (overlay_is_vid(overlay_id)) {
				writel_relaxed(addr->phys[0], &regs->v_y0);
				writel_relaxed(addr->phys[1], &regs->v_u0);
				writel_relaxed(addr->phys[2], &regs->v_v0);
			} else
				writel_relaxed(addr->phys[0], &regs->g_0);
			buffer->flags &= ~UPDATE_ADDR;
		}

		if (buffer && (buffer->flags & UPDATE_WIN)) {
			win = &buffer->win;
			trace_win(overlay->id, win, 0);
			if (overlay_is_vid(overlay_id)) {
				writel_relaxed(win->pitch[0],
					&regs->v_pitch_yc);
				writel_relaxed(win->pitch[2] << 16 |
					win->pitch[1], &regs->v_pitch_uv);
				writel_relaxed((win->ysrc << 16) | win->xsrc,
					&regs->v_size);
				writel_relaxed((win->ydst << 16) | win->xdst,
					&regs->v_size_z);
				writel_relaxed(win->ypos << 16 | win->xpos,
					&regs->v_start);
			} else {
				writel_relaxed(win->pitch[0], &regs->g_pitch);

				writel_relaxed((win->ysrc << 16) | win->xsrc,
					&regs->g_size);
				writel_relaxed((win->ydst << 16) | win->xdst,
					&regs->g_size_z);
				writel_relaxed(win->ypos << 16 | win->xpos,
					&regs->g_start);
			}

			overlay_set_fmt(overlay);
			buffer->flags &= ~UPDATE_WIN;
		}
		kfree(buffer);
	}

	if (!list_empty(&shadow->dma_list.queue)) {
		dma = list_first_entry(
			&shadow->dma_list.queue,
			struct mmp_shadow_dma, queue);
		if (dma) {
			list_del(&dma->queue);
			trace_dma(overlay->id, dma->dma_onoff, 0);
			dmafetch_onoff(overlay, dma->dma_onoff);
			dma->flags &= ~UPDATE_DMA;
		}
		kfree(dma);
	}

	if (!list_empty(&shadow->alpha_list.queue)) {
		alpha = list_first_entry(
			&shadow->alpha_list.queue,
			struct mmp_shadow_alpha, queue);
		if (alpha) {
			list_del(&alpha->queue);
			trace_alpha(overlay->id, &alpha->alpha, 0);
			path_set_alpha(overlay, &alpha->alpha);
			alpha->flags &= ~UPDATE_ALPHA;
		}
		kfree(alpha);
	}
}

static int is_mode_changed(struct mmp_mode *dst, struct mmp_mode *src)
{
	return (!src || !dst
		|| src->refresh != dst->refresh
		|| src->xres != dst->xres
		|| src->yres != dst->yres
		|| src->left_margin != dst->left_margin
		|| src->right_margin != dst->right_margin
		|| src->upper_margin != dst->upper_margin
		|| src->lower_margin != dst->lower_margin
		|| src->hsync_len != dst->hsync_len
		|| src->vsync_len != dst->vsync_len
		|| !!(src->hsync_invert) != !!(dst->hsync_invert)
		|| !!(src->vsync_invert) != !!(dst->vsync_invert)
		|| !!(src->invert_pixclock) != !!(dst->invert_pixclock));
}

/*
 * dynamically set mode is not supported.
 * if change mode when path on, path on/off is required.
 * or we would direct set path->mode
*/
static void path_set_mode(struct mmp_path *path, struct mmp_mode *mode)
{
	/* mode unchanged? do nothing */
	if (!is_mode_changed(&path->mode, mode))
		return;

	/* FIXME: assert mode supported */
	memcpy(&path->mode, mode, sizeof(struct mmp_mode));
	if (path->mode.xres != path->mode.real_xres)
		path->mode.xres = path->mode.real_xres;
	if (path->mode.yres != path->mode.real_yres)
		path->mode.yres = path->mode.real_yres;

	if (path->status) {
		path_onoff(path, 0);
		if (path->phy && path->phy->set_mode)
			path->phy->set_mode(path->phy, &path->mode);
		path_set_timing(path);
		path_onoff(path, 1);
	} else {
		if (path->phy && path->phy->set_mode)
			path->phy->set_mode(path->phy, &path->mode);
		path_set_timing(path);
	}
}

static struct mmp_overlay_ops mmphw_overlay_ops = {
	.set_onoff = overlay_set_onoff,
	.set_dma_onoff = overlay_set_dma_onoff,
	.set_win = overlay_set_win,
	.set_addr = overlay_set_addr,
	.set_surface = overlay_set_surface,
	.set_colorkey_alpha = overlay_set_colorkey_alpha,
	.set_alpha = overlay_set_path_alpha,
	.reduced_onoff = overlay_reduced_onoff,
	.set_vsmooth_en = overlay_set_vsmooth_en,
	.trigger = overlay_trigger,
};

static void ctrl_set_default(struct mmphw_ctrl *ctrl)
{
	u32 tmp, irq_mask;

	/*
	 * LCD Global control(LCD_TOP_CTRL) should be configed before
	 * any other LCD registers read/write, or there maybe issues.
	 */
	tmp = readl_relaxed(ctrl->reg_base + LCD_TOP_CTRL);

#ifdef CONFIG_CPU_PXA1986
	tmp |= 0xaaa0;
#else
	/* If define master/slave path, set all DMA objects to master path */
	if (ctrl->master_path_name && ctrl->slave_path_name)
		tmp |= 0x50fff0;
	else
		tmp |= 0xfff0;
#endif

	writel_relaxed(tmp, ctrl->reg_base + LCD_TOP_CTRL);

	/* clear all the interrupts */
	if (!DISP_GEN4(ctrl->version))
		writel_relaxed(0x0, ctrl->reg_base + SPU_IRQ_ISR);
	else
		writel_relaxed(0xffffffff, ctrl->reg_base + SPU_IRQ_ISR);

	/* disable all interrupts */
	irq_mask = path_imasks(0) | err_imask(0) |
		   path_imasks(1) | err_imask(1);
	tmp = readl_relaxed(ctrl->reg_base + SPU_IRQ_ENA);
	tmp &= ~irq_mask;
	writel_relaxed(tmp, ctrl->reg_base + SPU_IRQ_ENA);
}

static void path_set_default(struct mmp_path *path)
{
	struct lcd_regs *regs = path_regs(path);
	struct mmphw_ctrl *ctrl = path_to_ctrl(path);
	u32 dma_ctrl1, mask, tmp, path_config;

	path_config = path_to_path_plat(path)->path_config;

	mask = CFG_IOPADMODE_MASK | CFG_BURST_MASK | CFG_BOUNDARY_MASK;
	tmp = readl_relaxed(ctrl_regs(path) + SPU_IOPAD_CONTROL);
	tmp &= ~mask;
	tmp |= path_config;
	writel_relaxed(tmp, ctrl_regs(path) + SPU_IOPAD_CONTROL);

	/*
	 * Configure default bits: vsync triggers DMA,
	 * power save enable, configure alpha registers to
	 * display 100% graphics, and set pixel command.
	 */
	dma_ctrl1 = 0x2032ff81;

	/* If TV path as slave, diable interlace mode */
	if (ctrl->slave_path_name &&
		(!strcmp(ctrl->slave_path_name, path->name)))
		/* Fix me, when slave path isn't TV path */
		dma_ctrl1 &= ~CFG_TV_NIB_MASK;

	dma_ctrl1 |= CFG_VSYNC_INV_MASK;
	writel_relaxed(dma_ctrl1, ctrl_regs(path) + dma_ctrl(1, path->id));

	/* Configure default register values */
	writel_relaxed(0x00000000, &regs->blank_color);
	writel_relaxed(0x00000000, &regs->g_1);
	writel_relaxed(0x00000000, &regs->g_start);

	/*
	 * 1.enable multiple burst request in DMA AXI
	 * bus arbiter for faster read if not tv path;
	 * 2.enable horizontal smooth filter;
	 */
	mask = CFG_GRA_HSMOOTH_MASK | CFG_DMA_HSMOOTH_MASK | CFG_ARBFAST_ENA(1);
	tmp = readl_relaxed(ctrl_regs(path) + dma_ctrl(0, path->id));
	tmp |= mask;
	if (PATH_TV == path->id)
		tmp &= ~CFG_ARBFAST_ENA(1);
	writel_relaxed(tmp, ctrl_regs(path) + dma_ctrl(0, path->id));
}

static int path_init(struct mmphw_path_plat *path_plat,
		struct mmp_mach_path_config *config)
{
	struct mmphw_ctrl *ctrl = path_plat->ctrl;
	struct mmp_path_info *path_info;
	struct mmp_path *path = NULL;
	int i;

	dev_info(ctrl->dev, "%s: %s\n", __func__, config->name);

	/* init driver data */
	path_info = kzalloc(sizeof(struct mmp_path_info), GFP_KERNEL);
	if (!path_info) {
		dev_err(ctrl->dev, "%s: unable to alloc path_info for %s\n",
				__func__, config->name);
		return 0;
	}
	path_info->name = config->name;
	path_info->id = path_plat->id;
	path_info->dev = ctrl->dev;
	path_info->output_type = config->output_type;
	path_info->overlay_num = config->overlay_num;
	path_info->overlay_table = config->overlay_table;
	path_info->overlay_ops = &mmphw_overlay_ops;
	path_info->plat_data = path_plat;

	/* create/register platform device */
	path = mmp_register_path(path_info);
	if (!path) {
		kfree(path_info);
		return 0;
	}
	path->apical = mmp_apical_alloc(path->id);
	for (i = 0; i < path->overlay_num; i++) {
		path->overlays[i].vdma =
			mmp_vdma_alloc(path->overlays[i].id, 0);
		if (path->overlays[i].ops->trigger)
			path->overlays[i].shadow =
				mmp_shadow_alloc(&path->overlays[i]);
	}
	path_plat->path = path;
	path_plat->path_config = config->path_config;
	path_plat->link_config = config->link_config;
	/* get clock: path clock name same as path name */
	path_plat->clk = devm_clk_get(ctrl->dev, config->name);
	if (IS_ERR(path_plat->clk)) {
		/* it's possible to not have path_plat->clk */
		dev_info(ctrl->dev, "unable to get clk %s\n", config->name);
		path_plat->clk = NULL;
	}
	path_plat->qos_idle.name = path->name;
	pm_qos_add_request(&path_plat->qos_idle, PM_QOS_CPUIDLE_BLOCK,
				PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
	path_plat->qos_idle_d1p.name = path->name;
	pm_qos_add_request(&path_plat->qos_idle_d1p, PM_QOS_CPUIDLE_BLOCK,
				PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
	/* add operations after path set */
	mmp_vsync_init(path);
	path->ops.set_mode = path_set_mode;
	path->ops.set_irq = path_set_irq;
	path->ops.set_gamma = path_set_gamma;

	path_set_default(path);

	kfree(path_info);
	return 1;
}

static void path_deinit(struct mmphw_path_plat *path_plat)
{
	int i = 0;

	if (!path_plat)
		return;

	pm_qos_remove_request(&path_plat->qos_idle);
	pm_qos_remove_request(&path_plat->qos_idle_d1p);

	if (path_plat->clk)
		devm_clk_put(path_plat->ctrl->dev, path_plat->clk);

	if (path_plat->path && path_plat->path->overlays[i].ops->trigger) {
		for (i = 0; i < path_plat->path->overlay_num; i++)
			mmp_shadow_free(path_plat->path->overlays[i].shadow);
	}

	if (path_plat->path) {
		mmp_vsync_deinit(path_plat->path);
		mmp_unregister_path(path_plat->path);
	}
}

#ifdef CONFIG_OF
static const struct of_device_id mmp_disp_dt_match[] = {
	{ .compatible = "marvell,mmp-disp" },
	{},
};
#endif

static int mmphw_probe(struct platform_device *pdev)
{
	struct mmp_mach_plat_info *mi;
	struct resource *res;
	int ret, i, size, irq, path_num;
	struct mmphw_path_plat *path_plat;
	struct mmphw_ctrl *ctrl = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child_np;
	struct mmp_mach_path_config *paths_config;
	struct mmp_mach_path_config dt_paths_config[MAX_PATH];
	u32 overlay_num[MAX_PATH][MAX_OVERLAY];
	struct mmp_path *path = NULL;

	/* get resources from platform data */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "%s: no IO memory defined\n", __func__);
		ret = -ENOENT;
		goto res_fail;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "%s: no IRQ defined\n", __func__);
		ret = -ENOENT;
		goto res_fail;
	}

	if (IS_ENABLED(CONFIG_OF)) {
		if (of_property_read_u32(np, "marvell,path-num", &path_num)) {
			ret = -EINVAL;
			goto res_fail;
		}
		/* allocate ctrl */
		size = sizeof(struct mmphw_ctrl) +
			sizeof(struct mmphw_path_plat) * path_num;
		ctrl = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
		if (!ctrl) {
			ret = -ENOMEM;
			goto res_fail;
		}

		ctrl->path_num = path_num;
		if (of_property_read_string(np, "marvell,disp-name",
					&ctrl->name)) {
			ret = -EINVAL;
			goto prop_fail;
		}

		if (of_find_property(np, "marvell,master-path", NULL) &&
			of_property_read_string(np, "marvell,master-path",
					&ctrl->master_path_name)) {
			ret = -EINVAL;
			goto prop_fail;
		}

		if (of_find_property(np, "marvell,slave-path", NULL) &&
			of_property_read_string(np, "marvell,slave-path",
					&ctrl->slave_path_name)) {
			ret = -EINVAL;
			goto prop_fail;
		}

		if (of_property_read_u32(np, "lpm-qos", &ctrl->lpm_qos)) {
			ret = -EINVAL;
			dev_err(&pdev->dev, "%s: get lpm-ops failed!\n",
					__func__);
			goto prop_fail;
		}

		if (of_find_property(np, "lpm-qos-d1p", NULL)) {
			if (of_property_read_u32(np, "lpm-qos-d1p",
			&ctrl->lpm_qos_d1p)) {
				ret = -EINVAL;
				goto prop_fail;
			}
		} else
			ctrl->lpm_qos_d1p =
				PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE;

		if (of_get_child_count(np) != ctrl->path_num) {
			dev_err(&pdev->dev, "%s: path_num not match!\n",
					__func__);
			ret = -EINVAL;
			goto prop_fail;
		}

		i = 0;
		for_each_child_of_node(np, child_np) {
			if (of_property_read_string(child_np,
					"marvell,path-name",
					&dt_paths_config[i].name)) {
				ret = -EINVAL;
				goto prop_fail;
			}
			if (of_property_read_u32(child_np,
					"marvell,overlay-num",
					&dt_paths_config[i].overlay_num)) {
				ret = -EINVAL;
				goto prop_fail;
			}
			if (of_property_read_u32_array(child_np,
					"marvell,overlay-table",
					overlay_num[i],
					dt_paths_config[i].overlay_num)) {
				ret = -EINVAL;
				goto prop_fail;
			}
			dt_paths_config[i].overlay_table = overlay_num[i];
			if (of_property_read_u32(child_np,
					"marvell,output-type",
					&dt_paths_config[i].output_type)) {
				ret = -EINVAL;
				goto prop_fail;
			}
			if (of_property_read_u32(child_np,
					"marvell,path-config",
					&dt_paths_config[i].path_config)) {
				ret = -EINVAL;
				goto prop_fail;
			}
			if (of_property_read_u32(child_np,
					"marvell,link-config",
					&dt_paths_config[i].link_config)) {
				ret = -EINVAL;
				goto prop_fail;
			}
			i++;
		}
		paths_config = dt_paths_config;
	} else {
		/* get configs from platform data */
		mi = pdev->dev.platform_data;
		if (mi == NULL || !mi->path_num || !mi->paths) {
			dev_err(&pdev->dev, "%s: no platform data defined\n",
					__func__);
			ret = -EINVAL;
			goto res_fail;
		}

		/* allocate ctrl */
		size = sizeof(struct mmphw_ctrl) +
			sizeof(struct mmphw_path_plat) * mi->path_num;
		ctrl = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
		if (!ctrl) {
			ret = -ENOMEM;
			goto res_fail;
		}

		ctrl->path_num = mi->path_num;
		ctrl->name = mi->name;
		paths_config = mi->paths;
	}

	ctrl->dev = &pdev->dev;
	ctrl->irq = irq;
	platform_set_drvdata(pdev, ctrl);
	mutex_init(&ctrl->access_ok);

	/* map registers.*/
	if (!devm_request_mem_region(ctrl->dev, res->start,
			resource_size(res), ctrl->name)) {
		dev_err(ctrl->dev,
			"can't request region for resource %pR\n", res);
		ret = -EINVAL;
		goto prop_fail;
	}

	ctrl->reg_base = devm_ioremap_nocache(ctrl->dev,
			res->start, resource_size(res));
	if (ctrl->reg_base == NULL) {
		dev_err(ctrl->dev, "%s: res %lx - %lx map failed\n", __func__,
			(unsigned long)res->start, (unsigned long)res->end);
		ret = -ENOMEM;
		goto remap_fail;
	}

	/* get clock */
	ctrl->clk = devm_clk_get(ctrl->dev, "LCDCIHCLK");
	if (IS_ERR(ctrl->clk)) {
		dev_err(ctrl->dev, "unable to get clk LCDCIHCLK\n");
		ret = -ENOENT;
		goto clk_fail;
	}

#ifndef CONFIG_PM_RUNTIME
	clk_prepare_enable(ctrl->clk);
#endif
	pm_runtime_enable(ctrl->dev);
	pm_runtime_forbid(ctrl->dev);
	ctrl->version = readl_relaxed(ctrl->reg_base + LCD_VERSION);
	/* init global regs */
	ctrl_set_default(ctrl);

	/* init pathes from machine info and register them */
	for (i = 0; i < ctrl->path_num; i++) {
		/* get from config and machine info */
		path_plat = &ctrl->path_plats[i];
		path_plat->id = i;
		path_plat->ctrl = ctrl;

		/* path init */
		if (!path_init(path_plat, (paths_config + i))) {
			ret = -EINVAL;
			goto path_init_fail;
		}
	}

	for (i = 0; i < ctrl->path_num; i++) {
		path = ctrl->path_plats[i].path;
		path->master = NULL;
		path->slave = NULL;
		if (ctrl->master_path_name && ctrl->slave_path_name) {
			/* if current path is master path, assign to its master
			 *  member meanwhile, get the slave path and assign
			 *  to its slave member
			 */
			if (!strcmp(ctrl->master_path_name, path->name)) {
				path->master = path;
				path->slave =
					mmp_get_path(ctrl->slave_path_name);
			}
			/* if current path is salve path, assign to its slave
			 *  member meanwhile, get the master path and assign
			 *  to ist master member
			 */
			if (!strcmp(ctrl->slave_path_name, path->name)) {
				path->master =
					mmp_get_path(ctrl->master_path_name);
				path->slave = path;
			}
		}
	}

#ifdef CONFIG_MMP_DISP_SPI
	ret = lcd_spi_register(ctrl);
	if (ret < 0)
		goto path_init_fail;
#endif

	/* request irq */
	ret = devm_request_irq(ctrl->dev, ctrl->irq, ctrl_handle_irq,
		IRQF_SHARED, "lcd_controller", ctrl);
	if (ret < 0) {
		dev_err(ctrl->dev, "%s unable to request IRQ %d\n",
				__func__, ctrl->irq);
		ret = -ENXIO;
		goto path_init_fail;
	}

	ctrl_dbg_init(&pdev->dev);

#ifdef CONFIG_MMP_DISP_DFC
	ctrl_dfc_init(&pdev->dev);
#endif

	dev_info(ctrl->dev, "device init done\n");

	return 0;

path_init_fail:
	for (i = 0; i < ctrl->path_num; i++) {
		path_plat = &ctrl->path_plats[i];
		path_deinit(path_plat);
	}

	devm_clk_put(ctrl->dev, ctrl->clk);
clk_fail:
	devm_iounmap(ctrl->dev, ctrl->reg_base);
remap_fail:
	devm_release_mem_region(ctrl->dev, res->start, resource_size(res));
prop_fail:
	devm_kfree(ctrl->dev, ctrl);
res_fail:
	dev_err(&pdev->dev, "device init failed\n");

	return ret;
}

#if defined(CONFIG_PM_SLEEP) || defined(CONFIG_PM_RUNTIME)
static int mmphw_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mmphw_ctrl *ctrl = platform_get_drvdata(pdev);

	clk_disable_unprepare(ctrl->clk);

	return 0;
}

static int mmphw_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mmphw_ctrl *ctrl = platform_get_drvdata(pdev);

	clk_prepare_enable(ctrl->clk);

	return 0;
}
#endif

const struct dev_pm_ops mmphw_pm_ops = {
	SET_RUNTIME_PM_OPS(mmphw_runtime_suspend,
		mmphw_runtime_resume, NULL)
};

static struct platform_driver mmphw_driver = {
	.driver		= {
		.name	= "mmp-disp",
		.owner	= THIS_MODULE,
		.pm = &mmphw_pm_ops,
		.of_match_table = of_match_ptr(mmp_disp_dt_match),
	},
	.probe		= mmphw_probe,
};

static int mmphw_init(void)
{
	int ret;

	ret = mmp_vdma_register();
	if (ret < 0)
		return ret;

	ret = mmp_apical_register();
	if (ret < 0)
		goto apical_fail;

	ret = platform_driver_register(&mmphw_driver);
	if (ret < 0)
		goto ctrl_fail;

	ret = phy_dsi_register();
	if (ret < 0)
		goto phy_fail;

	return 0;

phy_fail:
	platform_driver_unregister(&mmphw_driver);
ctrl_fail:
	mmp_apical_unregister();
apical_fail:
	mmp_vdma_unregister();
	return ret;
}
module_init(mmphw_init);

MODULE_AUTHOR("Li Guoqing<ligq@marvell.com>");
MODULE_DESCRIPTION("Framebuffer driver for mmp");
MODULE_LICENSE("GPL");
