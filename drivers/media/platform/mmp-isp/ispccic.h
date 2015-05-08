/*
 * ispccic.h
 *
 * Marvell AREA51 ISP - DMA module
 *
 * Copyright:  (C) Copyright 2011 Marvell International Ltd.
 *              Henry Zhao <xzhao10@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */


#ifndef ISP_CCIC_H
#define ISP_CCIC_H

#include <media/area51.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <media/socisp-vdev.h>
#include <media/socisp-mdev.h>
#include <media/socisp-pdev.h>

/* This is not an exhaustive list */
enum isp_ccic_pix_formats {
	CCIC_PIX_FMT_OTHERS = 0,
	CCIC_PIX_FMT_YUV422_8BIT = 0x1e,
	CCIC_PIX_FMT_YUV422_8BIT_VP = 0x9e,
};

enum {
	AGENT_CCIC_CORE = 0,
	AGENT_CCIC_DPHY,
	AGENT_CCIC_END
};

enum {
	CCIC_CLOCK_FUN = 0,
	CCIC_CLOCK_PHY,
	CCIC_CLOCK_AXI,
	CCIC_CLOCK_AHB,
	CCIC_CLOCK_END
};

enum {
	CCIC_PADI_SNSR = 0,
	CCIC_PADO_DXO,
	CCIC_PADO_VDEV,
	CCIC_PADI_DPHY,
	CCIC_PAD_END,
	DPHY_PADI = 0,
	DPHY_PADO_CCIC,
	DPHY_PADO_DXO,
	DPHY_PAD_END,
};

enum ccic_dma_state {
	CCIC_DMA_IDLE = 0,
	CCIC_DMA_BUSY,
};

enum ccic_input_type {
	CCIC_INPUT_NONE = 0x0,
	CCIC_INPUT_SENSOR = 0x1,
};

enum ccic_output_type {
	CCIC_OUTPUT_NONE = 0x0,
	CCIC_OUTPUT_ISP = 0x1,
	CCIC_OUTPUT_MEMORY = 0x2,
};

enum isp_ccic_irq_type {
	CCIC_EOF0 = 0,
	CCIC_EOF1,
	CCIC_EOF2,
};

enum ccic_mipi_state {
	MIPI_NOT_SET = 0,
	MIPI_SET,
};

struct ccic_plat_data {
	unsigned long	fclk_mhz;
	unsigned long	mclk_mhz;
	unsigned long	avail_fclk_rate_num;
	unsigned long	avail_fclk_rate[CLK_RATE_NUM];
};

enum ccic_hardware_state {
	CCIC_STATE_IDLE = 0,
	CCIC_STATE_WORK,
};

struct ccic_device {
	struct area51_device		*isp;
	enum ccic_output_type		output;
	enum ccic_input_type		input;

	struct device		*dev;
	struct isp_subdev	agent;
	struct isp_block	block;
	struct isp_event	event_irq;
	struct isp_dev_ptr	desc;

	enum ccic_hardware_state	state;
	spinlock_t			irq_lock;

	struct mutex			ccic_mutex;

	enum ccic_dma_state		dma_state;
	int	stream_refcnt;
	struct ccic_plat_data		pdata;
#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
	struct mcd_dphy		mcd_dphy;
	/* in current design, DPHY and CCIC is coupled, so have to put
	 * mcd_dphy_hw here, should move to CSI2 subdev in future */
	struct mcd_dphy_hw	mcd_dphy_hw;
#endif
};

struct ccic_dphy_t {
	struct isp_subdev	agent;
	struct area51_device	*isp;
	struct isp_block	block;
	struct isp_dev_ptr	desc;
	struct device		*dev;
	int			lanes;
	struct csi_dphy_reg	dphy_cfg;
	enum ccic_mipi_state	mipi_config_flag;
	spinlock_t		mipi_flag_lock;
	int dphy_set;
	struct csi_dphy_desc dphy_desc;
#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
	struct mcd_dphy		mcd_dphy;
#endif
};

int pxa_ccic_init(struct ccic_device *ccic);
void pxa_ccic_cleanup(struct ccic_device *ccic);
void pxa_ccic_unregister_entities(struct ccic_device *ccic);
int pxa_ccic_register_entities(struct ccic_device *ccic,
				    struct v4l2_device *vdev);
void pxa_ccic_dma_isr_handler(struct ccic_device *ccic,
					unsigned long irq_status);
extern void ccic_set_mclk(struct ccic_device *ccic, int on);

#if 0
static inline u32 ccic_read(struct ccic_device *ccic, const u32 addr)
{
	return readl(ccic->reg_base + addr);
}

static inline void ccic_write(struct ccic_device *ccic,
				const u32 addr, const u32 value)
{
	writel(value, ccic->reg_base + addr);
}

static inline void ccic_setbit(struct ccic_device *ccic,
				const u32 addr, const u32 mask)
{
	u32 val = readl(ccic->reg_base + addr);
	val |= mask;
	writel(val, ccic->reg_base + addr);
}

static inline void ccic_clrbit(struct ccic_device *ccic,
				const u32 addr, const u32 mask)
{
	u32 val = readl(ccic->reg_base + addr);
	val &= ~mask;
	writel(val, ccic->reg_base + addr);
}
#else
#define ccic_read(ccic, addr)		isp_read(isp_sd2blk(&ccic->agent), addr)
#define ccic_write(ccic, adr, val)	isp_write(isp_sd2blk(&ccic->agent), adr, val)
#define ccic_setbit(ccic, addr, mask)	isp_set(isp_sd2blk(&ccic->agent), addr, mask)
#define ccic_clrbit(ccic, addr, mask)	isp_clr(isp_sd2blk(&ccic->agent), addr, mask)
#endif

#endif	/* ISP_CCIC_H */
