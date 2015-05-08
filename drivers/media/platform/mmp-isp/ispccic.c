/*
* ispccic.c
*
* Marvell AREA51 ISP - CCIC module
*
* Copyright:  (C) Copyright 2014 Marvell International Ltd.
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

#include <linux/module.h>
#include <linux/delay.h>
#include <media/v4l2-common.h>
#include <linux/v4l2-mediabus.h>
#include <linux/mm.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/export.h>
#include <linux/slab.h>

#include "isp.h"
#include "ispreg.h"
#include "ispccic.h"
#include "ispdma.h"

#define MC_CCIC_DRV_NAME	"ccic"
#define CCIC_IRQ_NAME		"ccic-irq"

#define MAX_CCIC_CH_USED	2

static int trace = 2;
module_param(trace, int, 0644);
MODULE_PARM_DESC(trace,
		"how many trace do you want to see? (0-4)"
		"0 - mute"
		"1 - only actual errors"
		"2 - milestone log"
		"3 - briefing log"
		"4 - detailed log");

static const struct pad_formats ccic_input_fmts[] = {
	{V4L2_MBUS_FMT_UYVY8_1X16, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_SBGGR10_1X10, V4L2_COLORSPACE_SRGB},
};

static const struct pad_formats ccic_output_fmts[] = {
	{V4L2_MBUS_FMT_UYVY8_1X16, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_SBGGR10_1X10, V4L2_COLORSPACE_SRGB},
};

static void __maybe_unused ccic_dump_regs(struct ccic_device *ccic)
{
	unsigned long regval;

	regval = ccic_read(ccic, CCIC_Y0_BASE_ADDR);
	dev_warn(ccic->dev, "ccic_set_stream Y0: 0x%08lX\n", regval);
	regval = ccic_read(ccic, CCIC_Y1_BASE_ADDR);
	dev_warn(ccic->dev, "ccic_set_stream Y1: 0x%08lX\n", regval);
	regval = ccic_read(ccic, CCIC_Y2_BASE_ADDR);
	dev_warn(ccic->dev, "ccic_set_stream Y2: 0x%08lX\n", regval);
	regval = ccic_read(ccic, CCIC_IRQ_RAW_STATUS);
	dev_warn(ccic->dev, "ccic_set_stream IRQRAWSTATE: 0x%08lX\n", regval);
	regval = ccic_read(ccic, CCIC_IRQ_STATUS);
	dev_warn(ccic->dev, "ccic_set_stream IRQSTATE: 0x%08lX\n", regval);
	regval = ccic_read(ccic, CCIC_IRQ_MASK);
	dev_warn(ccic->dev, "ccic_set_stream IRQMASK: 0x%08lX\n", regval);
	regval = ccic_read(ccic, CCIC_CTRL_0);
	dev_warn(ccic->dev, "ccic_set_stream CTRL0: 0x%08lX\n", regval);
	regval = ccic_read(ccic, CCIC_CTRL_1);
	dev_warn(ccic->dev, "ccic_set_stream CTRL1: 0x%08lX\n", regval);
	regval = ccic_read(ccic, CCIC_CLOCK_CTRL);
	dev_warn(ccic->dev, "ccic_set_stream CLKCTRL: 0x%08lX\n", regval);
	regval = ccic_read(ccic, CCIC_CSI2_IRQ_RAW_STATUS);
	dev_warn(ccic->dev, "ccic_set_stream MIPI STATUS: 0x%08lX\n", regval);
	regval = ccic_read(ccic, CCIC_CSI2_DPHY3);
	dev_warn(ccic->dev, "ccic_set_stream MIPI DPHY3: 0x%08lX\n", regval);
	regval = ccic_read(ccic, CCIC_CSI2_DPHY5);
	dev_warn(ccic->dev, "ccic_set_stream MIPI DPHY5: 0x%08lX\n", regval);
	regval = ccic_read(ccic, CCIC_CSI2_DPHY6);
	dev_warn(ccic->dev, "ccic_set_stream MIPI DPHY6: 0x%08lX\n", regval);
	regval = ccic_read(ccic, CCIC_IMG_SIZE);
	dev_warn(ccic->dev, "ccic_set_stream SIZE: 0x%08lX\n", regval);
	regval = ccic_read(ccic, CCIC_IMG_PITCH);
	dev_warn(ccic->dev, "ccic_set_stream PITCH: 0x%08lX\n", regval);
}

static int ccic_dump_registers(struct ccic_device *ccic,
			struct v4l2_ccic_dump_registers *regs)
{
	if (NULL == regs)
		return -EINVAL;

	regs->y0_base_addr = ccic_read(ccic, CCIC_Y0_BASE_ADDR);
	regs->y1_base_addr = ccic_read(ccic, CCIC_Y1_BASE_ADDR);
	regs->y2_base_addr = ccic_read(ccic, CCIC_Y2_BASE_ADDR);
	regs->irq_raw_status = ccic_read(ccic, CCIC_IRQ_RAW_STATUS);
	regs->irq_status = ccic_read(ccic, CCIC_IRQ_STATUS);
	regs->irq_mask = ccic_read(ccic, CCIC_IRQ_MASK);
	regs->ctrl_0 = ccic_read(ccic, CCIC_CTRL_0);
	regs->ctrl_1 = ccic_read(ccic, CCIC_CTRL_1);
	regs->clock_ctrl = ccic_read(ccic, CCIC_CLOCK_CTRL);
	regs->csi2_irq_raw_status = ccic_read(ccic, CCIC_CSI2_IRQ_RAW_STATUS);
	regs->csi2_dphy3 = ccic_read(ccic, CCIC_CSI2_DPHY3);
	regs->csi2_dphy5 = ccic_read(ccic, CCIC_CSI2_DPHY5);
	regs->csi2_dphy6 = ccic_read(ccic, CCIC_CSI2_DPHY6);
	regs->img_size = ccic_read(ccic, CCIC_IMG_SIZE);
	regs->img_pitch = ccic_read(ccic, CCIC_IMG_PITCH);

	return 0;
}


static void ccic_set_dma_addr(struct ccic_device *ccic,
	dma_addr_t	paddr, enum isp_ccic_irq_type irqeof)
{
	BUG_ON(paddr == 0);

	switch (irqeof) {
	case CCIC_EOF0:
		ccic_write(ccic, CCIC_Y0_BASE_ADDR, paddr);
		break;
	case CCIC_EOF1:
		ccic_write(ccic, CCIC_Y1_BASE_ADDR, paddr);
		break;
	case CCIC_EOF2:
		ccic_write(ccic, CCIC_Y2_BASE_ADDR, paddr);
		break;
	default:
		break;
	}

	return;
}

#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG

static void csi_dphy_write(void *hw_ctx, const struct csi_dphy_reg *regs)
{
	struct ccic_device *ccic = hw_ctx;
	u32 regval = 0;

	/* need stop CCIC ? */
	/* reset DPHY */
	/* Should not do APMU register R/W right here,
	 * better call platform interface*/
	regval = readl(APMU_CCIC_RST);
	writel(regval & ~0x2, APMU_CCIC_RST);
	writel(regval, APMU_CCIC_RST);

	regval = regs->hs_settle & 0xFF;
	regval = regs->hs_termen | (regval << 8);
	ccic_write(ccic, CCIC_CSI2_DPHY3, regval);

	regval = regs->cl_settle & 0xFF;
	regval = regs->cl_termen | (regval << 8);
	ccic_write(ccic, CCIC_CSI2_DPHY6, regval);

	regval = (1 << regs->lane) - 1;
	regval = regval | (regval << 4);
	ccic_write(ccic, CCIC_CSI2_DPHY5, regval);

	regval = (regs->lane - 1) & 0x03;	/* support 4 lane at most */
	regval = (regval << 1) | 0x41;
	ccic_write(ccic, CCIC_CSI2_CTRL0, regval);

	/* start CCIC */
	ccic_setbit(ccic, CCIC_CTRL_0, 0x1);
};

static void csi_dphy_read(void *hw_ctx, struct csi_dphy_reg *regs)
{
	struct ccic_device *ccic = hw_ctx;
	u32 phy3, phy5, phy6;

	phy3	= ccic_read(ccic, CCIC_CSI2_DPHY3);
	phy5	= ccic_read(ccic, CCIC_CSI2_DPHY5);
	phy6	= ccic_read(ccic, CCIC_CSI2_DPHY6);

	regs->cl_termen	= phy6 & 0xFF;
	regs->cl_settle	= (phy6>>8) & 0xFF;
	regs->cl_miss	= 0;
	regs->hs_termen = phy3 & 0xFF;
	regs->hs_settle	= (phy3>>8) & 0xFF;
	regs->hs_rx_to	= 0xFFFF;
	regs->lane	= 0;
	phy5 &= 0xF;
	while (phy5) {
		phy5 = phy5 & (phy5-1);
		regs->lane++;
	}
	return;
};
#endif

/* -----------------------------------------------------------------------------
 * ISP video operations */
static struct ccic_device *find_ccic_from_video(struct isp_vnode *video)
{
	struct isp_subdev *m_agent = video_get_ispsd(video);
	if (m_agent)
		return m_agent->drv_priv;
	else
		return NULL;
}

static int ccic_enable_hw(struct ccic_device *ccic)
{
	if (ccic->output & CCIC_OUTPUT_MEMORY) {
		ccic_write(ccic, CCIC_IRQ_STATUS, ~0);
		ccic_write(ccic, CCIC_IRQ_MASK, 0x3);
		ccic_setbit(ccic, CCIC_CTRL_0, 0x1);
		ccic->dma_state = CCIC_DMA_BUSY;
	}
	return 0;
}

static int ccic_disable_hw(struct ccic_device *ccic)
{
	if (ccic->output & CCIC_OUTPUT_MEMORY) {
		ccic_clrbit(ccic, CCIC_CTRL_0, 0x1);
		ccic_write(ccic, CCIC_IRQ_MASK, 0);
		ccic_write(ccic, CCIC_IRQ_STATUS, ~0);
		ccic->dma_state = CCIC_DMA_IDLE;
	}
	return 0;
}

static int ccic_load_buffer(struct ccic_device *ccic, int ch)
{
	struct isp_videobuf *buffer = NULL;
	unsigned long flags;

	buffer = isp_vnode_get_buffer(ispsd_get_video(&ccic->agent),
			MAX_CCIC_CH_USED);
	if (buffer == NULL)
		return -EINVAL;

	spin_lock_irqsave(&ispsd_get_video(&ccic->agent)->vb_lock, flags);
	ccic_set_dma_addr(ccic, buffer->paddr[ISPDMA_PADDR], ch);
	spin_unlock_irqrestore(&ispsd_get_video(&ccic->agent)->vb_lock, flags);

	return 0;
}


static int ccic_video_stream_on_notify(struct isp_vnode *vnode)
{
	struct ccic_device *ccic;
	int ch, ret;

	vnode->min_buf_cnt = MAX_CCIC_CH_USED;
	ccic = find_ccic_from_video(vnode);
	if (ccic == NULL)
		return -EINVAL;

	mutex_lock(&ccic->ccic_mutex);

	if (ccic->dma_state == CCIC_DMA_BUSY) {
		mutex_unlock(&ccic->ccic_mutex);
		return 0;
	}

	/*ccic_configure_mipi(get_dphy(ccic));*/

	for (ch = 0; ch < MAX_CCIC_CH_USED; ch++) {
		ret = ccic_load_buffer(ccic, ch);
		if (ret != 0)
			return -EINVAL;
	}

	ccic_enable_hw(ccic);

	mutex_unlock(&ccic->ccic_mutex);

	return 0;
}

static int ccic_video_stream_off_notify(struct isp_vnode *vnode)
{
	struct ccic_device *ccic;

	ccic = find_ccic_from_video(vnode);
	if (ccic == NULL)
		return -EINVAL;

	mutex_lock(&ccic->ccic_mutex);

	if (ccic->dma_state == CCIC_DMA_IDLE) {
		mutex_unlock(&ccic->ccic_mutex);
		return 0;
	}

	ccic_disable_hw(ccic);

	mutex_unlock(&ccic->ccic_mutex);

	return 0;
}

static int ccic_video_qbuf_notify(struct isp_vnode *vnode)
{
	return 0;
}

static int ccic_qbuf_handler(struct isp_subdev *ispsd,
		struct isp_event *event)
{
	return ccic_video_qbuf_notify(event->msg);
}

static int ccic_stream_handler(struct isp_subdev *ispsd,
		struct isp_event *event)
{
	if (event->type)
		return ccic_video_stream_on_notify(event->msg);
	else
		return ccic_video_stream_off_notify(event->msg);
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev operations
 */

static struct v4l2_mbus_framefmt *
__ccic_get_format(struct ccic_device *ccic, struct v4l2_subdev_fh *fh,
		  unsigned int pad, enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);
	else
		return &ccic->agent.fmt_pad[pad];
}

static int
ccic_try_format(struct ccic_device *ccic, struct v4l2_subdev_fh *fh,
		unsigned int pad, struct v4l2_mbus_framefmt *fmt,
		enum v4l2_subdev_format_whence which)
{
	unsigned int i;
	int ret = 0;

	switch (pad) {
	case CCIC_PADI_SNSR:
	case CCIC_PADI_DPHY:
		for (i = 0; i < ARRAY_SIZE(ccic_input_fmts); i++) {
			if (fmt->code == ccic_input_fmts[i].mbusfmt) {
				fmt->colorspace = ccic_input_fmts[i].colorspace;
				break;
			}
		}

		if (i >= ARRAY_SIZE(ccic_input_fmts))
			fmt->code = V4L2_MBUS_FMT_SBGGR10_1X10;

		break;

	case CCIC_PADO_DXO:
	case CCIC_PADO_VDEV:
		for (i = 0; i < ARRAY_SIZE(ccic_output_fmts); i++) {
			if (fmt->code == ccic_output_fmts[i].mbusfmt) {
				fmt->colorspace = ccic_input_fmts[i].colorspace;
				break;
			}
		}

		if (i >= ARRAY_SIZE(ccic_output_fmts))
			fmt->code = V4L2_MBUS_FMT_SBGGR10_1X10;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	fmt->field = V4L2_FIELD_NONE;

	return ret;
}

static int ccic_enum_mbus_code(struct v4l2_subdev *sd,
			       struct v4l2_subdev_fh *fh,
			       struct v4l2_subdev_mbus_code_enum *code)
{
	struct isp_subdev *agent = v4l2_get_subdev_hostdata(sd);
	struct ccic_device *ccic = agent->drv_priv;
	int ret = 0;

	mutex_lock(&ccic->ccic_mutex);
	switch (code->pad) {
	case CCIC_PADI_SNSR:
	case CCIC_PADI_DPHY:
		if (code->index >= ARRAY_SIZE(ccic_input_fmts)) {
			ret = -EINVAL;
			goto error;
		}
		code->code = ccic_input_fmts[code->index].mbusfmt;
		break;
	case CCIC_PADO_DXO:
	case CCIC_PADO_VDEV:
		if (code->index >= ARRAY_SIZE(ccic_output_fmts)) {
			ret = -EINVAL;
			goto error;
		}
		code->code = ccic_output_fmts[code->index].mbusfmt;
		break;
	default:
		ret = -EINVAL;
		break;
	}

error:
	mutex_unlock(&ccic->ccic_mutex);
	return ret;
}

static int ccic_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_frame_size_enum *fse)
{
	struct isp_subdev *agent = v4l2_get_subdev_hostdata(sd);
	struct ccic_device *ccic = agent->drv_priv;
	struct v4l2_mbus_framefmt format;
	int ret = 0;
	mutex_lock(&ccic->ccic_mutex);

	if (fse->index != 0) {
		ret = -EINVAL;
		goto error;
	}

	format.code = fse->code;
	format.width = 1;
	format.height = 1;
	ccic_try_format(ccic, fh, fse->pad, &format, V4L2_SUBDEV_FORMAT_TRY);
	fse->min_width = format.width;
	fse->min_height = format.height;

	if (format.code != fse->code) {
		ret = -EINVAL;
		goto error;
	}

	format.code = fse->code;
	format.width = -1;
	format.height = -1;
	ccic_try_format(ccic, fh, fse->pad, &format, V4L2_SUBDEV_FORMAT_TRY);
	fse->max_width = format.width;
	fse->max_height = format.height;

error:
	mutex_unlock(&ccic->ccic_mutex);
	return ret;
}

static int ccic_get_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_format *fmt)
{
	struct isp_subdev *agent = v4l2_get_subdev_hostdata(sd);
	struct ccic_device *ccic = agent->drv_priv;
	struct v4l2_mbus_framefmt *format;
	int ret = 0;
	mutex_lock(&ccic->ccic_mutex);

	format = __ccic_get_format(ccic, fh, fmt->pad, fmt->which);
	if (format == NULL) {
		ret = -EINVAL;
		goto error;
	}

	fmt->format = *format;
error:
	mutex_unlock(&ccic->ccic_mutex);
	return ret;
}

static int ccic_config_format(struct ccic_device *ccic, unsigned int pad)
{
	struct v4l2_mbus_framefmt *format = &ccic->agent.fmt_pad[pad];
	unsigned long width, height, regval, bytesperline;
	unsigned long ctrl0val = 0;
	unsigned long ypitch;
	int ret = 0;

	width = format->width;
	height = format->height;

	ctrl0val = ccic_read(ccic, CCIC_CTRL_0);
	switch (format->code) {
	case V4L2_MBUS_FMT_SBGGR10_1X10:
		ctrl0val &= ~0x000001E0;
		ctrl0val |= ((0x2 << 7) | (0x2 << 5));
		bytesperline = width * 10 / 8;
		ypitch = bytesperline / 4;
		break;
	case V4L2_MBUS_FMT_UYVY8_1X16:
		ctrl0val &= ~0x0003E1E0;
		ctrl0val |= (0x4 << 13);
		bytesperline = width * 16 / 8;
		ypitch = bytesperline / 4;
		break;
	default:
		return -EINVAL;
	}

	regval = (height << 16) | bytesperline;
	ccic_write(ccic, CCIC_IMG_SIZE, regval);
	regval = (ypitch << 2);
	ccic_write(ccic, CCIC_IMG_PITCH, regval);
	ccic_write(ccic, CCIC_IMG_OFFSET, 0);
	ccic_write(ccic, CCIC_CTRL_0, ctrl0val);
	return ret;
}

static int ccic_set_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_format *fmt)
{
	struct isp_subdev *agent = v4l2_get_subdev_hostdata(sd);
	struct ccic_device *ccic = agent->drv_priv;
	struct v4l2_mbus_framefmt *format;
	int ret = 0;
	mutex_lock(&ccic->ccic_mutex);

	format = __ccic_get_format(ccic, fh, fmt->pad, fmt->which);
	if (format == NULL) {
		ret = -EINVAL;
		goto error;
	}

	ret = ccic_try_format(ccic, fh, fmt->pad, &fmt->format, fmt->which);
	if (ret < 0)
		goto error;

	*format = fmt->format;

	if (fmt->which != V4L2_SUBDEV_FORMAT_TRY)
		ret = ccic_config_format(ccic, fmt->pad);
	if (ret < 0)
		goto error;
	memcpy(agent->fmt_pad + CCIC_PADI_DPHY, &fmt->format,
		sizeof(struct v4l2_mbus_framefmt));
error:
	mutex_unlock(&ccic->ccic_mutex);
	return ret;
}

static int ccic_init_formats(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_subdev_format format;
	struct v4l2_mbus_framefmt *format_active, *format_try;
	struct isp_subdev *agent = v4l2_get_subdev_hostdata(sd);
	struct ccic_device *ccic = agent->drv_priv;
	int ret = 0;

	if (fh == NULL) {
		memset(&format, 0, sizeof(format));
		format.pad = CCIC_PADI_SNSR;
		format.which =
			fh ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
		format.format.code = V4L2_MBUS_FMT_SBGGR10_1X10;
		format.format.width = 640;
		format.format.height = 480;
		format.format.colorspace = V4L2_COLORSPACE_SRGB;
		format.format.field = V4L2_FIELD_NONE;
		ret = ccic_set_format(sd, fh, &format);
		format.pad = CCIC_PADI_DPHY;
		ret = ccic_set_format(sd, fh, &format);

		format.pad = CCIC_PADO_DXO;
		format.which =
			fh ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
		format.format.code = V4L2_MBUS_FMT_SBGGR10_1X10;
		format.format.width = 640;
		format.format.height = 480;
		format.format.colorspace = V4L2_COLORSPACE_SRGB;
		ret = ccic_set_format(sd, fh, &format);
		format.pad = CCIC_PADO_VDEV;
		ret = ccic_set_format(sd, fh, &format);
	} else {
	/* Copy the active format to a newly opened fh structure */
		mutex_lock(&ccic->ccic_mutex);
		format_active = __ccic_get_format
			(ccic, fh, CCIC_PADI_DPHY, V4L2_SUBDEV_FORMAT_ACTIVE);
		if (format_active == NULL) {
			ret = -EINVAL;
			goto error;
		}
		format_try = __ccic_get_format
			(ccic, fh, CCIC_PADI_DPHY, V4L2_SUBDEV_FORMAT_TRY);
		if (format_try == NULL) {
			ret = -EINVAL;
			goto error;
		}
		memcpy(format_try, format_active,
				sizeof(struct v4l2_subdev_format));

		format_active = __ccic_get_format
			(ccic, fh, CCIC_PADO_DXO, V4L2_SUBDEV_FORMAT_ACTIVE);
		if (format_active == NULL) {
			ret = -EINVAL;
			goto error;
		}
		format_try = __ccic_get_format
			(ccic, fh, CCIC_PADO_DXO, V4L2_SUBDEV_FORMAT_TRY);
		if (format_try == NULL) {
			ret = -EINVAL;
			goto error;
		}
		memcpy(format_try, format_active,
				sizeof(struct v4l2_subdev_format));

		format_active = __ccic_get_format
			(ccic, fh, CCIC_PADO_VDEV, V4L2_SUBDEV_FORMAT_ACTIVE);
		if (format_active == NULL) {
			ret = -EINVAL;
			goto error;
		}
		format_try = __ccic_get_format
			(ccic, fh, CCIC_PADO_VDEV, V4L2_SUBDEV_FORMAT_TRY);
		if (format_try == NULL) {
			ret = -EINVAL;
			goto error;
		}
		memcpy(format_try, format_active,
				sizeof(struct v4l2_subdev_format));

error:
		mutex_unlock(&ccic->ccic_mutex);
	}

	return ret;
}

static int ccic_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct isp_subdev *agent = v4l2_get_subdev_hostdata(sd);
	struct ccic_device *ccic = agent->drv_priv;

	mutex_lock(&ccic->ccic_mutex);

	switch (enable) {
	case CCIC_STATE_WORK:
		if (ccic->stream_refcnt++ == 0) {
#if 0
			struct v4l2_subdev *dphy_sd =
				&(get_dphy(ccic)->agent.subdev);
			/* just a W/R for now, should be called by pipeline */
			v4l2_subdev_call(dphy_sd, video, s_stream, 1);
#endif
			ccic->state = enable;
		}
		break;
	case CCIC_STATE_IDLE:
		if (--ccic->stream_refcnt == 0) {
			/* struct v4l2_subdev *dphy_sd =
				&(get_dphy(ccic)->agent.subdev); */
			ccic_clrbit(ccic, CCIC_CTRL_0, 0x1);
			ccic_write(ccic, CCIC_IRQ_MASK, 0);
			ccic_write(ccic, CCIC_IRQ_STATUS, ~0);
			/* should call dphy fd, remove it */
			/* v4l2_subdev_call(dphy_sd, video, s_stream, 0); */

			ccic->state = enable;
		} else if (ccic->stream_refcnt < 0)
			ccic->stream_refcnt = 0;
		break;
	default:
		break;
	}

	mutex_unlock(&ccic->ccic_mutex);

	return 0;
}

static int ccic_dphy_set_stream(struct v4l2_subdev *sd, int enable);
static int ccic_io_set_stream(struct v4l2_subdev *sd, int *enable)
{
	enum ccic_hardware_state state;

	if ((NULL == sd) || (NULL == enable) || (*enable < 0))
		return -EINVAL;

	state = *enable ? CCIC_STATE_WORK : CCIC_STATE_IDLE;

	/* ccic core stream */
	ccic_set_stream(sd, state);
	/* ccic dphy stream */
	ccic_dphy_set_stream(sd, state);

	return 0;
}

static int ccic_io_config_mipi(struct ccic_dphy_t *dphy,
	struct v4l2_ccic_config_mipi *mipi_cfg)
{
	if (NULL == mipi_cfg)
		return -EINVAL;

	dphy->lanes = mipi_cfg->lanes;

	return 0;
}

static void ccic_set_mclk_rate(struct ccic_device *ccic, int on)
{
	struct ccic_plat_data *pdata = &ccic->pdata;
	u32 regval;
	if (on) {
		regval = (0x3 << 29) | (pdata->fclk_mhz / pdata->mclk_mhz);
		/*mclk is fixed at 25M for eden board*/
		ccic_write(ccic, CCIC_CLOCK_CTRL, regval);
	} else {
		ccic_write(ccic, CCIC_CLOCK_CTRL, 0);
	}
}


static inline int ccic_get_sensor_mclk(struct ccic_device *ccic,
		int *mclk)
{
	u32 ori_rate, regval;
	struct clk *fun_clk = ccic->block.clock[0];
	if (!mclk)
		return -EINVAL;
	ori_rate = clk_get_rate(fun_clk);
	regval = ccic_read(ccic, CCIC_CLOCK_CTRL)  & 0xffff;
	/* the user space mclk dimension is Hz */
	*mclk = ori_rate / regval;
	return 0;
}

static long ccic_ioctl(struct v4l2_subdev *sd
			, unsigned int cmd, void *arg)
{
	struct isp_subdev *agent = v4l2_get_subdev_hostdata(sd);
	struct ccic_device *ccic = agent->drv_priv;
	int ret;

	switch (cmd) {
#if 0
		/* here should call dphy ioctl, remove it from ccic core */
	case VIDIOC_PRIVATE_CCIC_CONFIG_MIPI:
		{/* This function don't belong to CCIC-CORE, this is a W/R */
			struct v4l2_subdev *dphy_sd =
				&(get_dphy(ccic)->agent.subdev);
			ret = v4l2_subdev_call(dphy_sd, core, ioctl, cmd, arg);
		}
		break;
#endif
	case VIDIOC_PRIVATE_CCIC_DUMP_REGISTERS:
		ret = ccic_dump_registers
			(ccic, (struct v4l2_ccic_dump_registers *)arg);
		break;
	case VIDIOC_PRIVATE_CCIC_SET_STREAM:
		ret = ccic_io_set_stream(sd, (int *)arg);
		break;
	case VIDIOC_PRIVATE_CCIC_GET_SENSOR_MCLK:
		ret = ccic_get_sensor_mclk(ccic, (int *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static int ccic_open(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh)
{
	struct isp_subdev *agent = v4l2_get_subdev_hostdata(sd);
	isp_block_tune_power(isp_sd2blk(agent), ~0);
#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
	struct isp_subdev *agent = v4l2_get_subdev_hostdata(sd);
	struct ccic_device *ccic = agent->drv_priv;
	struct area51_device *isp = ccic->isp;
	int ret;

	/* CSI attached, now add debug interface for it*/
	ccic->mcd_dphy = default_mcd_dphy;
	strcpy(ccic->mcd_dphy.entity.name, "dphy");
	ccic->mcd_dphy_hw.hw_ctx	= ccic;
	ccic->mcd_dphy_hw.reg_write	= &csi_dphy_write;
	ccic->mcd_dphy_hw.reg_read	= &csi_dphy_read;
	ccic->mcd_dphy.entity.priv = &ccic->mcd_dphy_hw;

	/* FIXME: mount the dphy node under LCD port, this is just a W/R
	 * need to modify MCD to support complex topology for MediaControl */
	ret = mcd_entity_init(&ccic->mcd_dphy.entity,
			&isp->mcd_root_display.mcd);
	if (ret < 0)
		return ret;
	isp->mcd_root_display.pitem[MCD_DPHY] = &ccic->mcd_dphy.entity;
	d_inf(2, "cam: mount node debugfs/%s/%s\n",
			isp->mcd_root_display.mcd.name,
			ccic->mcd_dphy.entity.name);
#endif
	return ccic_init_formats(sd, fh);
}

static int ccic_close(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh)
{
	struct isp_subdev *agent = v4l2_get_subdev_hostdata(sd);
	struct ccic_device *ccic = agent->drv_priv;

	if (NULL == ccic)
		return -EINVAL;

	mutex_lock(&ccic->ccic_mutex);
	if (ccic->state != CCIC_STATE_IDLE) {
		ccic->stream_refcnt = 1;
		mutex_unlock(&ccic->ccic_mutex);
		ccic_dphy_set_stream(sd, 0);
		ccic_set_stream(sd, 0);
	} else
		mutex_unlock(&ccic->ccic_mutex);
#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
	d_inf(2, "cam: dismount node debugfs/%s/%s\n",
		ccic->isp->mcd_root_display.mcd.name,
		ccic->mcd_dphy.entity.name);
	mcd_entity_remove(&ccic->mcd_dphy.entity);
#endif
	isp_block_tune_power(isp_sd2blk(agent), 0);
	return 0;
}

/* subdev core ooperations */
static const struct v4l2_subdev_core_ops ccic_ops = {
	.ioctl = ccic_ioctl,
};

/* subdev video operations */
static const struct v4l2_subdev_video_ops ccic_video_ops = {
	.s_stream = ccic_set_stream,
};

/* subdev pad operations */
static const struct v4l2_subdev_pad_ops ccic_pad_ops = {
	.enum_mbus_code = ccic_enum_mbus_code,
	.enum_frame_size = ccic_enum_frame_size,
	.get_fmt = ccic_get_format,
	.set_fmt = ccic_set_format,
};

/* subdev operations */
static const struct v4l2_subdev_ops ccic_core_map_ops = {
	.core = &ccic_ops,
	.video = &ccic_video_ops,
	.pad = &ccic_pad_ops,
};

/* subdev internal operations */
static const struct v4l2_subdev_internal_ops ccic_internal_ops = {
	.open = ccic_open,
	.close = ccic_close,
};

/* -----------------------------------------------------------------------------
 * Media entity operations
 */

/*
 * ccic_link_setup - Setup CCIC connections.
 * @entity : Pointer to media entity structure
 * @local  : Pointer to local pad array
 * @remote : Pointer to remote pad array
 * @flags  : Link flags
 * return -EINVAL or zero on success
 */
static int ccic_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct isp_subdev *agent = v4l2_get_subdev_hostdata(sd);
	struct ccic_device *ccic = agent->drv_priv;

	switch (local->index | media_entity_type(remote->entity)) {
	case CCIC_PADI_DPHY | MEDIA_ENT_T_V4L2_SUBDEV:
	case CCIC_PADI_SNSR | MEDIA_ENT_T_V4L2_SUBDEV:
		if (flags & MEDIA_LNK_FL_ENABLED)
			ccic->input |= CCIC_INPUT_SENSOR;
		else
			ccic->input &= ~CCIC_INPUT_SENSOR;
		break;
	case CCIC_PADO_VDEV | MEDIA_ENT_T_DEVNODE:
		if (flags & MEDIA_LNK_FL_ENABLED)
			ccic->output |= CCIC_OUTPUT_MEMORY;
		else
			ccic->output &= ~CCIC_OUTPUT_MEMORY;
		break;
	case CCIC_PADO_DXO | MEDIA_ENT_T_V4L2_SUBDEV:
		if (flags & MEDIA_LNK_FL_ENABLED)
			ccic->output |= CCIC_OUTPUT_ISP;
		else
			ccic->output &= ~CCIC_OUTPUT_ISP;
		break;
	default:
		/* Link from camera to CCIC is fixed... */
		return -EINVAL;
	}
	return 0;
}

/* media operations */
static const struct media_entity_operations ccic_media_ops = {
	.link_setup = ccic_link_setup,
};

static void ccic_isr_buffer(struct ccic_device *ccic,
		enum isp_ccic_irq_type irqeof)
{
	struct isp_videobuf *buffer;

	buffer = isp_vnode_xchg_buffer(
			ispsd_get_video(&ccic->agent), false);
	if (buffer != NULL) {
		ccic_set_dma_addr(ccic,
			buffer->paddr[ISPDMA_PADDR], irqeof);
	} else {
		ccic_disable_hw(ccic);
	}

	return;
}

void ccic_isr_dummy_buffer(struct ccic_device *ccic,
		enum isp_ccic_irq_type irqeof)
{
	struct isp_vnode *vnode;
	unsigned long flags;
	struct isp_videobuf *buffer;

	vnode = ispsd_get_video(&ccic->agent);
	spin_lock_irqsave(&vnode->vb_lock, flags);
	if (list_empty(&vnode->idle_buf)) {
		spin_unlock_irqrestore(&vnode->vb_lock, flags);
		return;
	}
	buffer = list_first_entry(&vnode->idle_buf,
		struct isp_videobuf, hook);
	list_del(&buffer->hook);
	vnode->idle_buf_cnt--;
	list_add_tail(&buffer->hook, &vnode->busy_buf);
	vnode->busy_buf_cnt++;
	ccic_set_dma_addr(ccic, buffer->paddr[ISPDMA_PADDR], irqeof);
	spin_unlock_irqrestore(&vnode->vb_lock, flags);

	return;
}

static irqreturn_t ccic_irq_handler(int irq, void *data)
{

	struct ccic_device *ccic = data;
	struct isp_event *event = &ccic->event_irq;
	u32 irq_status;
	u32 regval;
	unsigned long flag;

	spin_lock_irqsave(&ccic->irq_lock, flag);

	irq_status = ccic_read(ccic, CCIC_IRQ_STATUS);
	ccic_write(ccic, CCIC_IRQ_STATUS, irq_status);

	if (irq_status & CCIC_IRQ_STATUS_EOF0) {
		regval = ccic_read(ccic, CCIC_Y0_BASE_ADDR);
		ccic_isr_buffer(ccic, CCIC_EOF0);
	}

	if (irq_status & CCIC_IRQ_STATUS_EOF1) {
		regval = ccic_read(ccic, CCIC_Y1_BASE_ADDR);
		ccic_isr_buffer(ccic, CCIC_EOF1);
	}

	if (irq_status & CCIC_IRQ_STATUS_EOF2) {
		regval = ccic_read(ccic, CCIC_Y2_BASE_ADDR);
		ccic_isr_buffer(ccic, CCIC_EOF2);
	}

	/* And then send a event to all listening agent */
	event->type = irq;
	event->msg = (void *)irq_status;
	isp_event_dispatch(event);

	spin_unlock_irqrestore(&ccic->irq_lock, flag);

	return IRQ_HANDLED;
}

static int ccic_core_hw_init(struct isp_block *block)
{
#if 0	/* For dynamic resource allocation, don't check it for now
		 * here is just for debug */
	WARN_ON((agent->clock == NULL) && (agent->ops->set_clock == NULL));
	WARN_ON(agent->reg_base == NULL);
	WARN_ON(agent->irq_num == 0);
#endif
	d_inf(2, "ccic-core.%d: init done", block->id.dev_id);
	return 0;
}

static void ccic_core_hw_clean(struct isp_block *agent)
{
	d_inf(2, "ccic-core: map_hw will be disposed\n");
}

static int ccic_core_hw_power(void *agent, int level)
{
	/* CCIC-CORE can't be use alone, it must be use with CCIC-DPHY,
	 * so use CCIC-DPHY to call platform-LPM-update */

	return 0;
}

#if 0
/* keep this function, in future, when handle different resolution.*/
static int ccic_core_hw_qos(struct isp_block *agent, int qos_level)
{
	/* power off */
	/* platform: change function clock rate */
	/* power on */
	return 0;
}
#endif

static int ccic_core_hw_clk(struct isp_block *block, int level)
{
	/* Here keep CCIC1 function clk at 416M.
	 * so remove Variable frequency */
	/* FIXME: set CCIC1 function clk as 416M
	 * this is just temporary solution.
	 * we will fix it using dynamic change CCIC func clk. */

	u32 regval;
	struct clk *fun_clk = block->clock[0];
	static u32 ori_rate;

	/* FIXME: set 416M as ccic1 default func clk */
	if (level) {
		ori_rate = clk_get_rate(fun_clk);
		if (ori_rate < 416000000)
			clk_set_rate(fun_clk, 416000000);
	}

/* Here need to set CCIC1/CCIC2 Fclk for raw sensor. */
	if (level) {
		regval = isp_read(block, CCIC_CLOCK_CTRL)  & 0xffff;
		regval = (0x3 << 29) | regval;
		isp_write(block, CCIC_CLOCK_CTRL, regval);
	} else {
		isp_write(block, CCIC_CLOCK_CTRL, 0);
	}

	return 0;
}


struct isp_block_ops ccic_core_hw_ops = {
	.init		= ccic_core_hw_init,
	.clean		= ccic_core_hw_clean,
	.set_power	= ccic_core_hw_power,
/*  .set_qos	= ccic_core_hw_qos, */
	.set_clock	= ccic_core_hw_clk,
};

static int ccic_core_add(struct isp_subdev *agent)
{
	struct ccic_device *ccic = agent->drv_priv;
	struct v4l2_subdev *sd = &agent->subdev;
	struct media_pad *pads = agent->pads;
	int ret = 0;

	/* prepare drv_priv */
	ccic->state = CCIC_STATE_IDLE;
	ccic->dma_state = CCIC_DMA_IDLE;
	ccic->stream_refcnt = 0;
	spin_lock_init(&ccic->irq_lock);
	mutex_init(&ccic->ccic_mutex);

	/* prepare subdev */
	v4l2_subdev_init(sd, &ccic_core_map_ops);
	sd->internal_ops = &ccic_internal_ops;
	strlcpy(sd->name, ccic->block.name, sizeof(sd->name));
	kfree(ccic->block.name);
	ccic->block.name = sd->name;
	sd->grp_id = GID_ISP_SUBDEV;
	v4l2_set_subdevdata(sd, &ccic->agent);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.ops = &ccic_media_ops;
	sd->ctrl_handler = &ccic->agent.ctrl_handler;

	/* Prepare agent */
	pads[CCIC_PADO_DXO].flags = MEDIA_PAD_FL_SOURCE;
	pads[CCIC_PADO_VDEV].flags = MEDIA_PAD_FL_SOURCE;
	pads[CCIC_PADI_SNSR].flags = MEDIA_PAD_FL_SINK;
	pads[CCIC_PADI_DPHY].flags = MEDIA_PAD_FL_SINK;
	agent->single = 1;
	agent->pads_cnt	= CCIC_PAD_END;

	return ret;
}

static int ccic_core_open(struct isp_subdev *ispsd)
{
	struct ccic_device *ccic_core = ispsd->drv_priv;
	struct isp_vnode *vnode = ispsd_get_video(ispsd);
	struct isp_event *event;
	char tmp[32];
	int ret;

	sprintf(tmp, "%s:qbuf", vnode->vdev.name);
	event = isp_event_find(ispsd->build, tmp);
	if (event == NULL)
		return -ENOENT;
	ret = isp_event_subscribe(event, ispsd, &ccic_qbuf_handler);
	if (ret < 0)
		return ret;

	sprintf(tmp, "%s:stream", vnode->vdev.name);
	event = isp_event_find(ispsd->build, tmp);
	if (event == NULL)
		return -ENOENT;
	ret = isp_event_subscribe(event, ispsd, &ccic_stream_handler);
	if (ret < 0)
		return ret;

	ccic_set_mclk_rate(ccic_core, 1);
	ccic_write(ccic_core, CCIC_CTRL_1, CCIC_CTRL1_DMA_128B |
			CCIC_CTRL1_MAGIC_NUMBER | CCIC_CTRL1_2_FRAME);
	return 0;
}

static void ccic_core_close(struct isp_subdev *ispsd)
{
	struct ccic_device *ccic_core = ispsd->drv_priv;
	struct isp_vnode *vnode = ispsd_get_video(ispsd);
	struct isp_event *event;
	char tmp[32];

	sprintf(tmp, "%s:qbuf", vnode->vdev.name);
	event = isp_event_find(ispsd->build, tmp);
	if (event)
		isp_event_unsubscribe(event, ispsd);

	sprintf(tmp, "%s:stream", vnode->vdev.name);
	event = isp_event_find(ispsd->build, tmp);
	if (event)
		isp_event_unsubscribe(event, ispsd);

	ccic_write(ccic_core, CCIC_CTRL_1,
			CCIC_CTRL1_MAGIC_NUMBER | CCIC_CTRL1_2_FRAME);
	ccic_set_mclk_rate(ccic_core, 0);
}

struct isp_subdev_ops ccic_core_ops = {
	.add		= ccic_core_add,
	.open		= ccic_core_open,
	.close		= ccic_core_close,
};

void ccic_set_mclk(struct ccic_device *ccic, int on)
{
	/* pxa988/1088/lL88 ccic_func_clk apply mclk */
	if (on) {
		isp_block_tune_power(isp_sd2blk(&ccic->agent), on);
		ccic_core_open(&ccic->agent);
		ccic_set_mclk_rate(ccic, on);
	} else {
		ccic_set_mclk_rate(ccic, on);
		ccic_core_close(&ccic->agent);
		isp_block_tune_power(isp_sd2blk(&ccic->agent), on);
	}
}
EXPORT_SYMBOL(ccic_set_mclk);

/********************************* CCIC-DPHY *********************************/

static int ccic_clear_mipi(struct ccic_dphy_t *dphy)
{
	unsigned long mipi_lock_flags;

	spin_lock_irqsave(&dphy->mipi_flag_lock, mipi_lock_flags);
	if (dphy->mipi_config_flag == MIPI_NOT_SET) {
		spin_unlock_irqrestore(&dphy->mipi_flag_lock, mipi_lock_flags);
		return 0;
	}

	isp_write(isp_sd2blk(&dphy->agent), CCIC_CSI2_DPHY3, 0);
	isp_write(isp_sd2blk(&dphy->agent), CCIC_CSI2_DPHY5, 0);
	isp_write(isp_sd2blk(&dphy->agent), CCIC_CSI2_DPHY6, 0);
	isp_write(isp_sd2blk(&dphy->agent), CCIC_CSI2_CTRL0, 0);

	dphy->mipi_config_flag = MIPI_NOT_SET;
	spin_unlock_irqrestore(&dphy->mipi_flag_lock, mipi_lock_flags);
	return 0;
}

struct csi_dphy_calc dphy_calc_profiles[] = {
	{
		.name		= "safe",
		.hs_termen_pos	= 0,
		.hs_settle_pos	= 50,
	},
};

int dphy_calc_reg(struct csi_dphy_desc *pdesc,
		struct csi_dphy_calc *algo, struct csi_dphy_reg *preg)
{
	u32 ps_period, ps_ui, ps_termen_max, ps_prep_max, ps_prep_min;
	u32 ps_sot_min, ps_termen, ps_settle;
	ps_period = MHZ * 1000 / (pdesc->clk_freq / 1000);
	ps_ui = ps_period / 2;
	ps_termen_max	= 35000 + 4 * ps_ui;
	ps_prep_min	= 40000 + 4 * ps_ui;
	ps_prep_max	= 85000 + 6 * ps_ui;
	ps_sot_min	= 145000 + 10 * ps_ui;
	ps_termen	= ps_termen_max + algo->hs_termen_pos * ps_period;
	ps_settle	= 1000 * (pdesc->hs_prepare + pdesc->hs_zero *
						algo->hs_settle_pos / 100);

	preg->cl_termen	= 0x00;
	preg->cl_settle	= 0x04;
	preg->cl_miss	= 0x00;
	/* term_en = round_up(ps_termen / ps_period) - 1 */
	preg->hs_termen	= (ps_termen + ps_period - 1) / ps_period - 1;
	/* For Marvell DPHY, Ths-settle started from HS-0, not VILmax */
	ps_settle -= (preg->hs_termen + 1) * ps_period;
	/* round_up(ps_settle / ps_period) - 1 */
	preg->hs_settle = (ps_settle + ps_period - 1) / ps_period - 1;
	preg->hs_rx_to	= 0xFFFF;
	preg->lane	= pdesc->nr_lane;
	return 0;
}

static int ccic_csi_dphy_write(struct ccic_dphy_t *dphy)
{
	struct csi_dphy_reg *regs = &dphy->dphy_cfg;
	int lanes = dphy->dphy_desc.nr_lane;
	unsigned int dphy3_val;
	unsigned int dphy5_val;
	unsigned int dphy6_val;
	unsigned int ctrl0_val;

	if (lanes == 0) {
		dev_warn(dphy->dev, "CCIC lanes num not set, set it to 2\n");
		lanes = 2;
	}

	if (dphy->dphy_set) {
		dphy3_val = regs->hs_settle & 0xFF;
		dphy3_val = regs->hs_termen | (dphy3_val << 8);

		dphy6_val = regs->cl_settle & 0xFF;
		dphy6_val = regs->cl_termen | (dphy6_val << 8);
	} else {
		dphy3_val = 0x1806;
		dphy6_val = 0xA00;
	}

	dphy5_val = (1 << lanes) - 1;
	dphy5_val = dphy5_val | (dphy5_val << 4);

	ctrl0_val = (lanes - 1) & 0x03;
	ctrl0_val = (ctrl0_val << 1) | 0x41;

	isp_write(isp_sd2blk(&dphy->agent), CCIC_CSI2_DPHY3, dphy3_val);
	isp_write(isp_sd2blk(&dphy->agent), CCIC_CSI2_DPHY5, dphy5_val);
	isp_write(isp_sd2blk(&dphy->agent), CCIC_CSI2_DPHY6, dphy6_val);
	isp_write(isp_sd2blk(&dphy->agent), CCIC_CSI2_CTRL0, ctrl0_val);

	return 0;
}

static int ccic_configure_mipi(struct ccic_dphy_t *dphy)
{
	unsigned long mipi_lock_flags;

	spin_lock_irqsave(&dphy->mipi_flag_lock, mipi_lock_flags);
	if (dphy->mipi_config_flag != MIPI_NOT_SET) {
		spin_unlock_irqrestore(&dphy->mipi_flag_lock, mipi_lock_flags);
		return 0;
	}

	if (dphy->dphy_set)
		dphy_calc_reg(&dphy->dphy_desc,
				dphy_calc_profiles, &dphy->dphy_cfg);
	ccic_csi_dphy_write(dphy);

	dphy->mipi_config_flag = MIPI_SET;
	spin_unlock_irqrestore(&dphy->mipi_flag_lock, mipi_lock_flags);
	return 0;
}

static long ccic_dphy_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct isp_subdev *agent = v4l2_get_subdev_hostdata(sd);
	struct ccic_dphy_t *dphy = agent->drv_priv;
	int ret;

	switch (cmd) {
	case VIDIOC_PRIVATE_CCIC_CONFIG_MIPI:
		ret = ccic_io_config_mipi(dphy,
					(struct v4l2_ccic_config_mipi *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

static int ccic_dphy_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct isp_subdev *agent = v4l2_get_subdev_hostdata(sd);
	struct ccic_dphy_t *dphy = agent->drv_priv;

	if (enable)
		return ccic_configure_mipi(dphy);
	else
		return ccic_clear_mipi(dphy);
}

static int ccic_dphy_internal_open(struct v4l2_subdev *sd,
		struct v4l2_subdev_fh *fh)
{
	struct isp_subdev *agent = v4l2_get_subdev_hostdata(sd);
	return isp_block_tune_power(isp_sd2blk(agent), ~0);
}

static int ccic_dphy_internal_close(struct v4l2_subdev *sd,
		struct v4l2_subdev_fh *fh)
{
	struct isp_subdev *agent = v4l2_get_subdev_hostdata(sd);
	return isp_block_tune_power(isp_sd2blk(agent), 0);
}

static const struct v4l2_subdev_core_ops ccic_dphy_core_ops = {
	.ioctl = ccic_dphy_ioctl,
};

static const struct v4l2_subdev_video_ops ccic_dphy_video_ops = {
	.s_stream = ccic_dphy_set_stream,
};

static const struct v4l2_subdev_ops ccic_dphy_sd_ops = {
	.core	= &ccic_dphy_core_ops,
	.video	= &ccic_dphy_video_ops,
};

static const struct v4l2_subdev_internal_ops ccic_dphy_internal_ops = {
	.open = ccic_dphy_internal_open,
	.close = ccic_dphy_internal_close,
};

static void ccic_dphy_hw_clean(struct isp_block *agent)
{
	d_inf(2, "ccic-dphy: map_hw will be disposed\n");
}

static int ccic_dphy_hw_init(struct isp_block *block)
{
	d_inf(2, "ccic-dphy.%d: init done", block->id.dev_id);
	return 0;
}

#if 0
/* keep this function, in future, when handle different resolution.*/
static int ccic_dphy_hw_power(struct isp_block *agent, int level)
{
	return 0;
}
static int ccic_dphy_hw_qos(struct isp_block *agent, int qos_level)
{
	/* power off */
	/* platform: change function clock rate */
	/* power on */
	return 0;
}
#endif

struct isp_block_ops ccic_dphy_hw_ops = {
	.init		= ccic_dphy_hw_init,
	.clean		= ccic_dphy_hw_clean,
/*	.set_power	= ccic_dphy_hw_power,
 *	.set_qos	= ccic_dphy_hw_qos,
 */
};


static int dphy_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	d_inf(2, "ccic-dphy link setup not fulfilled\n");
	return 0;
}

static const struct media_entity_operations dphy_media_ops = {
	.link_setup = dphy_link_setup,
};

static int ccic_dphy_add(struct isp_subdev *agent)
{
	struct ccic_dphy_t *dphy = agent->drv_priv;
	struct v4l2_subdev *sd = &agent->subdev;

	/* Prepare struct dphy */
	spin_lock_init(&dphy->mipi_flag_lock);

	/* Prepare agent */
	v4l2_subdev_init(sd, &ccic_dphy_sd_ops);
	sd->internal_ops = &ccic_dphy_internal_ops;
	sd->grp_id = GID_ISP_SUBDEV;
	v4l2_set_subdevdata(sd, agent);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	agent->pads[DPHY_PADI].flags	= MEDIA_PAD_FL_SINK;
	agent->pads[DPHY_PADO_CCIC].flags	= MEDIA_PAD_FL_SOURCE;
	agent->pads[DPHY_PADO_DXO].flags	= MEDIA_PAD_FL_SOURCE;
	agent->pads_cnt = DPHY_PAD_END;
	agent->single = 1;

	sd->entity.ops = &dphy_media_ops;
	return 0;
}

static void ccic_dphy_remove(struct isp_subdev *agent)
{
}

static int __maybe_unused ccic_phy_isr(struct isp_subdev *dphy,
		struct isp_event *irq)
{
	return 0;
}

static int ccic_dphy_open(struct isp_subdev *ispsd)
{
	struct ccic_dphy_t *dphy = ispsd->drv_priv;
	struct area51_device *isp = dphy->isp;

	if (isp->plat_lpm_update)
		isp->plat_lpm_update(1);

	return 0;
}

static void ccic_dphy_close(struct isp_subdev *ispsd)
{
	struct ccic_dphy_t *dphy = ispsd->drv_priv;
	struct area51_device *isp = dphy->isp;

	if (isp->plat_lpm_update)
		isp->plat_lpm_update(0);

	return;
}

struct isp_subdev_ops ccic_dphy_ops = {
	.add		= ccic_dphy_add,
	.remove		= ccic_dphy_remove,
	.open		= ccic_dphy_open,
	.close		= ccic_dphy_close,
};

/******************************** CCIC IP Core ********************************/

static char *clock_name[] = {
	[CCIC_CLOCK_FUN]	= "CCICFUNCLK_0",
	[CCIC_CLOCK_PHY]	= "CCICPHYCLK",
	[CCIC_CLOCK_AXI]	= "CCICAXICLK",
	[CCIC_CLOCK_AHB]	= "LCDCIISPAXI",
};

static struct isp_res_req ccic_core_res[] = {
	/* {ISP_RESRC_MEM,	0}, */
	{ISP_RESRC_IRQ},
	{ISP_RESRC_CLK,	CCIC_CLOCK_FUN},
#ifndef CONFIG_CPU_EDEN
	{ISP_RESRC_CLK,	CCIC_CLOCK_AHB},
#endif
	{ISP_RESRC_CLK, CCIC_CLOCK_AXI},
	{ISP_RESRC_END}
};

static struct isp_res_req ccic_dphy_res[] = {
	/* {ISP_RESRC_MEM,	0x100}, */
	{ISP_RESRC_CLK,	CCIC_CLOCK_FUN},
	{ISP_RESRC_CLK,	CCIC_CLOCK_PHY},
	{ISP_RESRC_END}
};

static int mc_ccic_suspend(struct device *dev)
{
	return 0;
}

static int mc_ccic_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops mc_ccic_pm = {
	.suspend	= mc_ccic_suspend,
	.resume		= mc_ccic_resume,
};

static int mc_ccic_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct ccic_device *mc_ccic;
	struct ccic_dphy_t *ccic_dphy;
	struct resource *res, clk;
	struct block_id pdev_mask = {
		.dev_type = PCAM_IP_CCIC,
		.dev_id = pdev->dev.id,
		.mod_type = 0xFF,
		.mod_id = 0xFF,
	};
	struct isp_block *block;
	struct isp_subdev *ispsd;
	struct isp_dev_ptr *desc;
	struct ccic_plat_data pdata;
	const u32 *clk_val;
	int i, j, ret, len;

	i = of_alias_get_id(np, "area51-ccic");
	if (i < 0) {
		dev_err(&pdev->dev, "failed to get alias id, errno %d\n", i);
		return i;
	}
	pdev->id = i;
	pdev_mask.dev_id = pdev->dev.id =  pdev->id;

	mc_ccic = devm_kzalloc(&pdev->dev,
				sizeof(struct ccic_device), GFP_KERNEL);
	if (!mc_ccic) {
		dev_err(&pdev->dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, mc_ccic);
	mc_ccic->dev = &pdev->dev;

	ccic_dphy = devm_kzalloc(&pdev->dev, sizeof(struct ccic_dphy_t),
				GFP_KERNEL);
	if (!ccic_dphy) {
		dev_err(&pdev->dev, "could not allocate ccic_dphy struct\n");
		return -ENOMEM;
	}
	ccic_dphy->dev = &pdev->dev;

	/*FIXME:the ccic clock's dev_id is "mmp-camera", clk_get() need use
	 * it. So must set it to dev.init_name.
	 * */
	kfree(pdev->dev.init_name);
	pdev->dev.init_name = kasprintf(GFP_KERNEL, "%s.%d",
			"mmp-camera", pdev->id);
	if (!pdev->dev.init_name) {
		dev_err(&pdev->dev, "could not allocate dev init name\n");
		return -ENOMEM;
	}
#if 0
	/* request the mem region for the camera registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL)
		dev_err(&pdev->dev, "mem resource not found");
	ret = area51_resrc_register(&pdev->dev, res, res->name, pdev_mask, 0,
					NULL, NULL));
	if (ret < 0) {
		dev_err(&pdev->dev, "failed register mem resource %s",
			res->name);
		return ret;
	}
#else
	/* This is a W/R to avoid iomem conflict with smart sensor driver */
	switch (pdev->id) {
	case 0:
		mc_ccic->block.reg_base = (void __iomem *)CCIC1_VIRT_BASE;
		ccic_dphy->block.reg_base = (void __iomem *)CCIC1_VIRT_BASE;
		break;
	case 1:
		mc_ccic->block.reg_base = (void __iomem *)CCIC2_VIRT_BASE;
		ccic_dphy->block.reg_base = (void __iomem *)CCIC2_VIRT_BASE;
		break;
	default:
		return -EINVAL;
		break;
	}
#endif

	clk_val = of_get_property(np, "mclk-mhz", &len);
	if (!clk_val)
		return -EINVAL;
	else
		pdata.mclk_mhz = be32_to_cpup(clk_val);
	clk_val = of_get_property(np, "fclk-mhz", &len);
	if (!clk_val)
		return -EINVAL;
	else
		pdata.fclk_mhz = be32_to_cpup(clk_val);

	mc_ccic->pdata = pdata;

	/* get irqs */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get irq resource");
		return -ENXIO;
	}
	strcpy(mc_ccic->event_irq.name, CCIC_IRQ_NAME);
	memset(&mc_ccic->event_irq.dispatch_list, 0,
		sizeof(mc_ccic->event_irq.dispatch_list));
	/* We expect DxO wrapper only take one kind of irq: the DMA irq.
	 * so the handler is the same for all, but resource id is device id */
	ret = area51_resrc_register(&pdev->dev,
				res, CCIC_IRQ_NAME, pdev_mask, 0,
				/* irq handler */
				&ccic_irq_handler,
				/* irq context*/
				mc_ccic);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed register irq resource %s",
			res->name);
		return ret;
	}

	memset(&clk, 0, sizeof(struct resource));
	clk.flags = ISP_RESRC_CLK;
	/* get clocks */
	for (i = 0, j = 0; j < CCIC_CLOCK_END; i++) {
		clk.name = clock_name[j];
		ret = area51_resrc_register(&pdev->dev, &clk, NULL, pdev_mask,
						j, NULL, NULL);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed register clock resource %s",
				res->name);
			return ret;
		}
		j++;
	}

	ispsd = &mc_ccic->agent;
	block = &mc_ccic->block;
	desc = &mc_ccic->desc;
	block->id.dev_type = PCAM_IP_CCIC;
	block->id.dev_id = pdev->id;
	block->id.mod_type = ISP_BLOCK_DMA_OUT;
	block->id.mod_id = AGENT_CCIC_CORE;
	block->name = kasprintf(GFP_KERNEL, "%s.%d",
			"ccic-core", pdev->id);
	block->req_list	= ccic_core_res;
	block->ops		= &ccic_core_hw_ops;
	ispsd->ops	= &ccic_core_ops;
	ispsd->drv_priv	= mc_ccic;
	INIT_LIST_HEAD(&desc->hook);
	desc->ptr = block;
	desc->type = ISP_GDEV_BLOCK;
	ret = ccic_core_add(ispsd);
	if (ret < 0)
		return ret;
	INIT_LIST_HEAD(&ispsd->gdev_list);
	list_add_tail(&desc->hook, &ispsd->gdev_list);
	ispsd->sd_code = SDCODE_AREA51_CCIC1 + pdev->id;
	ispsd->sd_type = ISP_BLOCK_DMA_OUT;
	ret = area51_ispsd_register(ispsd);
	if (ret < 0)
		return ret;

	ispsd = &ccic_dphy->agent;
	block = &ccic_dphy->block;
	desc = &ccic_dphy->desc;
	block->id.dev_type = PCAM_IP_CCIC;
	block->id.dev_id = pdev->id;
	block->id.mod_type = ISP_BLOCK_NORMAL;
	block->id.mod_id = AGENT_CCIC_DPHY;
	block->name = kasprintf(GFP_KERNEL, "%s.%d",
			"ccic-dphy", pdev->id);
	block->req_list	= ccic_dphy_res;
	block->ops		= &ccic_dphy_hw_ops;
	ispsd->ops	= &ccic_dphy_ops;
	ispsd->drv_priv	= ccic_dphy;
	INIT_LIST_HEAD(&desc->hook);
	desc->ptr = block;
	desc->type = ISP_GDEV_BLOCK;
	ret = ccic_dphy_add(ispsd);
	if (ret < 0)
		return ret;
	INIT_LIST_HEAD(&ispsd->gdev_list);
	list_add_tail(&desc->hook, &ispsd->gdev_list);
	ispsd->sd_code = SDCODE_AREA51_DPHY1 + pdev->id;
	ret = area51_ispsd_register(ispsd);
	if (ret < 0)
		return ret;

	return 0;
}

static int mc_ccic_remove(struct platform_device *pdev)
{
	struct ccic_device *mc_ccic = platform_get_drvdata(pdev);

	if (mc_ccic->block.reg_base != NULL)
		devm_iounmap(mc_ccic->dev, mc_ccic->block.reg_base);
	if (mc_ccic->block.irq_num)
		devm_free_irq(mc_ccic->dev, mc_ccic->block.irq_num, mc_ccic);
	devm_kfree(mc_ccic->dev, mc_ccic);
	return 0;
}

static const struct of_device_id area51_ccic_dt_match[] = {
	{ .compatible = "marvell, AREA51-CCIC", .data = NULL },
	{},
};

struct platform_driver mc_ccic_driver = {
	.driver = {
		.name	= MC_CCIC_DRV_NAME,
		.pm	= &mc_ccic_pm,
		.of_match_table = of_match_ptr(area51_ccic_dt_match),
	},
	.probe	= mc_ccic_probe,
	.remove	= mc_ccic_remove,
};

module_platform_driver(mc_ccic_driver);
