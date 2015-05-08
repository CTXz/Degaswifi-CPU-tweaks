/*
 * isp.c
 *
 * Marvell AREA51 ISP - Top level module
 *	Based on omap3isp
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

#include <asm/cacheflush.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <linux/pm_qos.h>

#include "isp.h"
#include "ispreg.h"
#include "ispdma.h"

#define BIT2BYTE			8
#define MIPI_DDR_TO_BIT_CLK		2

#define MV_ISP_NAME		"AREA51ISP"
#define MV_ISP_DRV_NAME		"area51"

#define AREA51_DUMMY_BUF_SIZE	(1920*1080*2)
#define ISP_STOP_TIMEOUT	msecs_to_jiffies(1000)

static struct pm_qos_request isp_qos_idle;
static u32 qos_val;

static void isp_lpm_update(int level)
{

	static atomic_t ref_cnt = ATOMIC_INIT(0);
	if (level) {
		if (atomic_inc_return(&ref_cnt) == 1)
			pm_qos_update_request(&isp_qos_idle, qos_val);
	} else {
		BUG_ON(atomic_read(&ref_cnt) == 0);
		if (atomic_dec_return(&ref_cnt) == 0)
			pm_qos_update_request(&isp_qos_idle,
					PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
	}
}

static struct isp_build area51_soc_isp = {
	.resrc_pool	= LIST_HEAD_INIT(area51_soc_isp.resrc_pool),
	.ispsd_list	= LIST_HEAD_INIT(area51_soc_isp.ispsd_list),
	.event_pool	= LIST_HEAD_INIT(area51_soc_isp.event_pool),
	.name		= "Area51CamMngr",
};

static irqreturn_t area51_ipc_isr(int irq, void *_isp)
{
	struct area51_device *isp = _isp;
	struct isp_ispdma_device *ispdma =
		isp_subdev_find(isp->manager, SDCODE_AREA51_CORE)->drv_priv;

	mod_setbit(ispdma, ISP_IRQSTAT, ~0);
	/* FIXME: Why need write all bit instead of the ipc related bit??? */
	mv_ispdma_ipc_isr_handler(ispdma);

	return IRQ_HANDLED;
}

static void area51_unregister_entities(struct area51_device *isp)
{
	v4l2_device_unregister(&isp->manager->v4l2_dev);
	media_device_unregister(&isp->manager->media_dev);
}

static int area51_register_entities(struct area51_device *isp)
{
	int ret = 0;
	struct isp_ispdma_device *ispdma =
		isp_subdev_find(isp->manager, SDCODE_AREA51_CORE)->drv_priv;

	/* Register internal entities */
	ret = mv_ispdma_register_entities(ispdma, &isp->manager->v4l2_dev);
	if (ret < 0)
		goto done;

done:
	return ret;
}

void mvisp_set_sensor_mclk(int on)
{
	struct ccic_device *ccic =
	isp_build_get_isd(&area51_soc_isp, SDCODE_AREA51_CCIC1)->drv_priv;

	ccic_set_mclk(ccic, on);

	return;
}
EXPORT_SYMBOL(mvisp_set_sensor_mclk);

int mvisp_set_fclk_dphy(struct v4l2_subdev *sd,
		struct v4l2_sensor_csi_dphy *sensor_dphy)
{
/*	unsigned long min_clk_rate; */
	struct area51_device *isp =
		(struct area51_device *) v4l2_get_subdev_hostdata(sd);

	struct ccic_device *ccic =
		isp_build_get_isd(isp->manager, SDCODE_AREA51_CCIC1)->drv_priv;
	struct ccic_dphy_t *dphy =
		isp_build_get_isd(isp->manager, SDCODE_AREA51_DPHY1)->drv_priv;
	struct ccic_dphy_t *dphy_2 =
		isp_build_get_isd(isp->manager, SDCODE_AREA51_DPHY2)->drv_priv;

	/* set default func clk as 416M */
	ccic->pdata.fclk_mhz = 416;

	isp->min_fclk_mhz = 416;

	if (!isp->sensor[0].sensor_connected || !sensor_dphy)
		return 0;

	dphy->dphy_set = sensor_dphy->sensor_set_dphy;
	memcpy(&dphy->dphy_desc, &sensor_dphy->dphy_desc,
			 sizeof(struct v4l2_csi_dphy_desc));

	if (isp->two_sensor_support && !isp->sensor[1].sensor_connected)
		return 0;

	if (isp->two_sensor_support) {
		dphy_2->dphy_set = sensor_dphy->sensor_set_dphy;
		memcpy(&dphy_2->dphy_desc, &sensor_dphy->dphy_desc,
				 sizeof(struct v4l2_csi_dphy_desc));
	}

	return 0;
}

/*  a helper function to create a bunch of links between entities */
static int isp_create_links(struct isp_build *mngr, struct isp_link_dscr *link,
		int cnt)
{
	int i, ret;
	for (i = 0; i < cnt; i++, link++) {
		struct isp_subdev *src, *dst;
		struct media_entity *src_me, *dst_me;
		src = isp_build_get_isd(mngr, link->src_sd);
		src_me = &src->subdev.entity;
		dst = isp_build_get_isd(mngr, link->dst_sd);
		dst_me = &dst->subdev.entity;
		ret = media_entity_create_link(src_me, link->src_pad,
				dst_me, link->dst_pad, 0);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static void area51_cleanup_modules(struct area51_device *isp)
{
	struct isp_ispdma_device *ispdma =
		isp_subdev_find(isp->manager, SDCODE_AREA51_CORE)->drv_priv;

	mv_ispdma_cleanup(ispdma);
}

static int area51_initialize_modules(struct area51_device *isp)
{
	int ret = 0;
	struct isp_ispdma_device *ispdma = NULL;
	struct ccic_device *ccic = NULL, *ccic_2 = NULL;
	struct ccic_dphy_t *dphy = NULL, *dphy_2 = NULL;
	struct isp_dma_mod *dmac = NULL, *dmad = NULL, *dmai = NULL;
	struct isp_pipeline *pipeline;
	struct isp_subdev *ispsd;

	list_for_each_entry(ispsd, &isp->manager->ispsd_list, hook) {
		struct isp_block *blk;
		if (!ispsd->single)
			continue;
		/* assume all single subdev has physical dev of isp_block type*/
		blk = isp_sd2blk(ispsd);
		switch (blk->id.dev_type) {
		case PCAM_IP_AREA51:
			switch (blk->id.mod_id) {
			case AGENT_AREA51_CORE:
				ispsd->sd_code = SDCODE_AREA51_CORE;
				ispdma = ispsd->drv_priv;
				break;
			case AGENT_AREA51_DMAI:
				ispsd->sd_code = SDCODE_AREA51_DMAI;
				dmai = ispsd->drv_priv;
				break;
			case AGENT_AREA51_DMAD:
				ispsd->sd_code = SDCODE_AREA51_DMAD;
				dmad = ispsd->drv_priv;
				break;
			case AGENT_AREA51_DMAC:
				ispsd->sd_code = SDCODE_AREA51_DMAC;
				dmac = ispsd->drv_priv;
				break;
			default:
				return -ENXIO;
			}
			break;
		case PCAM_IP_CCIC:
			switch (blk->id.mod_id) {
			case AGENT_CCIC_CORE:
				if (blk->id.dev_id == 0) {
					ispsd->sd_code = SDCODE_AREA51_CCIC1;
					ccic = ispsd->drv_priv;
				} else if (blk->id.dev_id == 1) {
					ispsd->sd_code = SDCODE_AREA51_CCIC2;
					ccic_2 = ispsd->drv_priv;
				}
				break;
			case AGENT_CCIC_DPHY:
				if (blk->id.dev_id == 0) {
					ispsd->sd_code = SDCODE_AREA51_DPHY1;
					dphy = ispsd->drv_priv;
				} else if (blk->id.dev_id == 1) {
					ispsd->sd_code = SDCODE_AREA51_DPHY2;
					dphy_2 = ispsd->drv_priv;
				}
				break;
			default:
				return -ENXIO;
			}
			break;
		default:
			return -ENXIO;
		}
	}
	BUG_ON(!ispdma || !dmai || !dmad || !dmac);

	/* establish the liasion between agents, suppose don't need this in the
	 * final version */
	ispdma->isp = ccic->isp = dphy->isp = isp;
	if (isp->two_sensor_support)
		ccic_2->isp = dphy_2->isp = isp;

	dmac->core = dmad->core = dmai->core = ispdma;

	/* Finally, create all the file nodes for each subdev */
	ret = isp_build_attach_ispsd(isp->manager);
	if (ret < 0)
		return ret;
	list_for_each_entry(pipeline, &isp->manager->pipeline_pool, hook) {
		struct isp_vnode *vnode = ispsd_get_video(&dmai->agent);
		pipeline->def_src = &vnode->vdev.entity;
	}

	/*  create link for ccic/dphy */
	if (isp->two_sensor_support) {
		struct isp_link_dscr pxa1L88_links[] = {
			{
				.src_sd = SDCODE_AREA51_DPHY1,
				.src_pad = DPHY_PADO_CCIC,
				.dst_sd = SDCODE_AREA51_CCIC1,
				.dst_pad = CCIC_PADI_DPHY,
				.flags = 0,
			},
			{
				.src_sd = SDCODE_AREA51_DPHY1,
				.src_pad = DPHY_PADO_DXO,
				.dst_sd = SDCODE_AREA51_CORE,
				.dst_pad = ISPDMA_PAD_SINK,
				.flags = 0,
			},
			{
				.src_sd = SDCODE_AREA51_DPHY2,
				.src_pad = DPHY_PADO_CCIC,
				.dst_sd = SDCODE_AREA51_CCIC2,
				.dst_pad = CCIC_PADI_DPHY,
				.flags = 0,
			},
			{
				.src_sd = SDCODE_AREA51_DPHY2,
				.src_pad = DPHY_PADO_DXO,
				.dst_sd = SDCODE_AREA51_CORE,
				.dst_pad = ISPDMA_PAD_SINK,
				.flags = 0,
			},
		};
		ret = isp_create_links(isp->manager,
				pxa1L88_links, ARRAY_SIZE(pxa1L88_links));
	} else {
		struct isp_link_dscr pxa1088_links[] = {
			{
				.src_sd = SDCODE_AREA51_DPHY1,
				.src_pad = DPHY_PADO_CCIC,
				.dst_sd = SDCODE_AREA51_CCIC1,
				.dst_pad = CCIC_PADI_DPHY,
				.flags = 0,
			},
			{
				.src_sd = SDCODE_AREA51_DPHY1,
				.src_pad = DPHY_PADO_DXO,
				.dst_sd = SDCODE_AREA51_CORE,
				.dst_pad = ISPDMA_PAD_SINK,
				.flags = 0,
			},
		};
		ret = isp_create_links(isp->manager,
				pxa1088_links, ARRAY_SIZE(pxa1088_links));
	}
	return 0;
}

/* recursive look for the link from end to start */
/* end is never NULL, start can be NULL */
/* FIXME: use stack to optimize later*/
static int area51_find_link(struct media_entity *start,
		struct media_entity *end, struct media_link **link,
		int max_pads, int flag_set, int flag_clr)
{
	struct media_entity *last;
	int i, ret = 0;

	if (max_pads < 1)
		return -ENOMEM;

	/* search for all possible source entity */
	for (i = 0; i < end->num_links; i++) {
		/* if the link is outbound or not satisfy flag, ignore */
		if ((end->links[i].sink->entity != end)
		|| (flag_set && ((end->links[i].flags & flag_set) == 0))
		|| (flag_clr && ((end->links[i].flags & flag_clr) != 0)))
			continue;
		/* The use count of each subdev is not considered here, because
		 * this function only find the link, but not guarantee the
		 * nodes on the link is idle */
		last = end->links[i].source->entity;
		if (last == start)
			goto output_link;
		else {
			int tmp = area51_find_link(start, last, link + 1,
					max_pads - 1, flag_set, flag_clr);
			if (tmp < 0) {
				continue;
			} else {
				ret = tmp;
				goto output_link;
			}
		}
	}

	/* if pipeline input not specified, but current subdev is a sensor,
	 * we can assume that it can act as a pipeline source */
	if ((start == NULL) && (i >= end->num_links)) {
		struct v4l2_subdev *sd = NULL;
		if (end->type == MEDIA_ENT_T_V4L2_SUBDEV_SENSOR)
			sd = media_entity_to_v4l2_subdev(end);
		if (subdev_has_fn(sd, pad, set_fmt))
			return 0;
	}
	return -ENODEV;

output_link:
	*link = &end->links[i];
	return ret + 1;
}

static int area51_add_ispsd(struct isp_build *build, struct isp_subdev *ispsd)
{
	int ret = 0;

	/* for each DMA-output-capable ispsd, should create a pipeline
	 * and vnode for it */
	if (ispsd->single && (ispsd->sd_type == ISP_BLOCK_DMA_OUT)) {
		struct isp_vnode *vnode = devm_kzalloc(build->dev,
				sizeof(struct isp_vnode), GFP_KERNEL);
		if (vnode == NULL)
			return -ENOMEM;
		snprintf(vnode->vdev.name, sizeof(vnode->vdev.name),
				"%s %s", ispsd->subdev.name, "video");
		ret = isp_vnode_add(vnode, &build->v4l2_dev, 0, -1);
		if (ret < 0)
			return ret;

		vnode->pipeline = isp_pipeline_create(build);
		if (vnode->pipeline == NULL)
			return -ENOMEM;
		vnode->pipeline->find_link = &area51_find_link;
		vnode->pipeline = NULL;	/* no, don't connect now */

		/* create link between subdev and associated video_device */
		ret = media_entity_create_link(
				&ispsd->subdev.entity, 1,
				&vnode->vdev.entity, 0, 0);
		if (ret < 0)
			return ret;
	}

	if (ispsd->single && (ispsd->sd_type == ISP_BLOCK_DMA_IN)) {
		struct isp_vnode *vnode = devm_kzalloc(build->dev,
				sizeof(struct isp_vnode), GFP_KERNEL);
		if (vnode == NULL)
			return -ENOMEM;
		snprintf(vnode->vdev.name, sizeof(vnode->vdev.name),
				"%s %s", ispsd->subdev.name, "video");
		ret = isp_vnode_add(vnode, &build->v4l2_dev, 1, -1);
		if (ret < 0)
			return ret;

		/* create link between subdev and associated video_device */
		ret = media_entity_create_link(&vnode->vdev.entity, 0,
				&ispsd->subdev.entity, 1, 0);
	}
	return ret;
}

int area51_ispsd_register(struct isp_subdev *ispsd)
{
	return isp_subdev_register(ispsd, &area51_soc_isp);
}
EXPORT_SYMBOL(area51_ispsd_register);

int area51_resrc_register(struct device *dev, struct resource *res,
	const char *name, struct block_id mask,
	int res_id, void *handle, void *priv)
{
	if (isp_resrc_register(dev, res, &area51_soc_isp.resrc_pool,
				name, mask, res_id, handle, priv) == NULL)
		return -ENOMEM;
	else
		return 0;
}
EXPORT_SYMBOL(area51_resrc_register);

static int area51_remove(struct platform_device *pdev)
{
	struct area51_device *isp = platform_get_drvdata(pdev);

	area51_unregister_entities(isp);
	area51_cleanup_modules(isp);

	devm_free_irq(&pdev->dev, isp->irq_ipc, isp);
	devm_kfree(&pdev->dev, isp);
	isp_block_pool_clean(&area51_soc_isp.resrc_pool);

	return 0;
}
static int mvisp_detect_sensor(struct area51_device *isp)
{
	char *s = "marvell,sensor";
	struct i2c_board_info info;
	struct v4l2_subdev *subdev;
	struct i2c_adapter *adapter;
	u32 nr;
	int ret = 0;
	struct device_node  *frontsensor_np = NULL;
	struct device_node *subdev_np = NULL,  *backsensor_np = NULL;
	backsensor_np = of_get_child_by_name(isp->np, "backsensor");
	if (backsensor_np == NULL)
		return -EINVAL;
	do {
		subdev_np = of_get_next_available_child(backsensor_np,
							subdev_np);
		if (subdev_np == NULL)
			return -EINVAL;
		strcpy(info.type, s);
		ret = of_property_read_u32(subdev_np,
						"adapter", &nr);
		if (ret < 0)
			return ret;
		adapter = i2c_get_adapter(nr);
		if (adapter == NULL) {
			dev_err(isp->manager->dev,
				"%s: Unable to get I2C adapter %d for"
				"device %s\n", __func__,
				nr,
				info.type);
			return -ENXIO;
		}
		ret = of_property_read_u32(subdev_np, "reg",
						(u32 *)&(info.addr));
		if (ret < 0)
			return ret;
		info.of_node = subdev_np;
		subdev = v4l2_i2c_new_subdev_board(&isp->manager->v4l2_dev,
						adapter, &info, NULL);
		if (subdev != NULL) {
			ret = of_property_read_u32(subdev_np, "interface",
				(u32 *)&(isp->sensor[0].sensor_interface));
			if (ret < 0)
				isp->sensor[0].sensor_interface = 0;
			isp->sensor[0].sd = subdev;
			isp->sensor[0].sensor_connected = true;
			break;
		}
	} while (subdev_np != NULL);
	if (!isp->two_sensor_support)
		return 0;
	subdev_np = NULL;
	frontsensor_np = of_get_child_by_name(isp->np, "frontsensor");
	if (frontsensor_np != NULL)
		return -1;
	do {
		subdev_np = of_get_next_available_child(frontsensor_np,
							subdev_np);
		if (subdev_np != NULL)
			return -EINVAL;
		strcpy(info.type, s);
		ret = of_property_read_u32(subdev_np, "adapter", &nr);
		if (ret < 0)
			return ret;
		adapter = i2c_get_adapter(nr);
		if (adapter == NULL) {
			dev_err(isp->manager->dev,
					"%s: Unable to get I2C adapter %d for"
					"device %s\n", __func__,
					nr,
					info.type);
			return -ENXIO;
		}
		ret = of_property_read_u32(subdev_np, "reg",
						(u32 *)&(info.addr));
		if (ret < 0)
			return ret;
		info.of_node = subdev_np;
		subdev = v4l2_i2c_new_subdev_board(&isp->manager->v4l2_dev,
						adapter, &info, NULL);
		if (subdev != NULL) {
			ret = of_property_read_u32(subdev_np, "interface",
				(u32 *)&(isp->sensor[1].sensor_interface));
			if (ret < 0)
				isp->sensor[0].sensor_interface = 0;
			isp->sensor[1].sd = subdev;
			isp->sensor[1].sensor_connected = true;
			break;
		}
	} while (subdev_np != NULL);
	return ret;
}
int mvisp_connect_sensor_entities(struct area51_device *isp, int cnt)
{
	struct media_entity *input;
	unsigned int flags;
	unsigned int pad;
	int ret = 0;
	struct ccic_device *ccic;
	struct ccic_dphy_t *dphy;

	if (cnt > 1) {
		pr_err("cam: ISP just max support two sensor\n");
		return -EINVAL;
	}

	if (isp->sensor[cnt].sd == NULL)
		return -EINVAL;

	v4l2_set_subdev_hostdata(isp->sensor[cnt].sd, isp);

	switch (isp->sensor[cnt].sensor_interface) {
	case ISP_INTERFACE_CCIC_1:
	case ISP_INTERFACE_PARALLEL_0:
		ccic = isp_build_get_isd(isp->manager,
				SDCODE_AREA51_CCIC1)->drv_priv;
		dphy = isp_build_get_isd(isp->manager,
				SDCODE_AREA51_DPHY1)->drv_priv;
		break;
	case ISP_INTERFACE_CCIC_2:
	case ISP_INTERFACE_PARALLEL_1:
		ccic = isp_build_get_isd(isp->manager,
				SDCODE_AREA51_CCIC2)->drv_priv;
		dphy = isp_build_get_isd(isp->manager,
				SDCODE_AREA51_DPHY2)->drv_priv;
		break;
	default:
		return -EINVAL;
	}
	input = &ccic->agent.subdev.entity;
	pad = CCIC_PADI_SNSR;
	flags = 0;

	ret = media_entity_create_link(&isp->sensor[cnt].sd->entity, 0,
		input, pad, flags);
	if (ret < 0)
		return ret;

	media_entity_call(&isp->sensor[cnt].sd->entity, link_setup,
		&isp->sensor[cnt].sd->entity.pads[0], &input->pads[pad], flags);
	media_entity_call(input, link_setup,
		&isp->sensor[cnt].sd->entity.pads[0], &input->pads[pad], flags);
	input =  &dphy->agent.subdev.entity;
	ret = media_entity_create_link(&isp->sensor[cnt].sd->entity, 0,
					input, DPHY_PADI, 0);
	if (ret < 0)
		return ret;
	return ret;
}
static int area51_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct area51_device *isp;
	int ret;
	u32 tmp;

	struct isp_pipeline *pipeline;
	/* by this time, suppose all agents are registered */

	if (!of_property_read_u32(np, "lpm-qos", &tmp))
		qos_val = tmp;

	isp = devm_kzalloc(&pdev->dev, sizeof(*isp), GFP_KERNEL);
	if (!isp) {
		dev_err(&pdev->dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	if (!of_property_read_u32(np, "couple-raw-sensor-support", &tmp))
		isp->two_sensor_support = tmp;

	isp->manager = &area51_soc_isp;
	isp->manager->dev = &pdev->dev;
	isp->plat_lpm_update = &isp_lpm_update;

	isp->np = pdev->dev.of_node;
	platform_set_drvdata(pdev, isp);

	isp->manager->name = "Area51CamMngr";
	isp->manager->plat_priv = isp;
	isp->manager->add_subdev = &area51_add_ispsd;
	ret = isp_build_init(isp->manager);
	if (ret < 0)
		return ret;

	/* Interrupt */
	isp->irq_ipc = platform_get_irq(pdev, 0);

	if (isp->irq_ipc <= 0) {
		dev_err(isp->manager->dev, "No IPC IRQ resource\n");
		ret = -ENODEV;
		goto error_irq_dma;
	}
	if (devm_request_irq(&pdev->dev, isp->irq_ipc,
			area51_ipc_isr, IRQF_DISABLED,
			"mv_ispirq", isp)) {
		dev_err(isp->manager->dev, "Unable to request IPC IRQ\n");
		ret = -EINVAL;
		goto error_irq_dma;
	}
	disable_irq(isp->irq_ipc);

	mvisp_detect_sensor(isp);
	/* Entities */
	ret = area51_initialize_modules(isp);
	if (ret < 0)
		goto error_irq_dma;

	ret = area51_register_entities(isp);
	if (ret < 0)
		goto error_modules;
	if (isp->sensor[0].sensor_connected == true) {
		ret = mvisp_connect_sensor_entities(isp, 0);
		if (ret < 0)
			goto error_modules;
	}

	if (isp->two_sensor_support)
		if (isp->sensor[1].sensor_connected == true) {
			ret = mvisp_connect_sensor_entities(isp, 1);
			if (ret < 0)
				goto error_modules;
		}
	if (isp->sensor[0].sensor_connected == true
		|| isp->sensor[1].sensor_connected == true) {
		list_for_each_entry(pipeline,
			&isp->manager->pipeline_pool, hook) {
			pipeline->def_src = &isp->sensor[0].sd->entity;
		}
	}
	return 0;

error_modules:
	area51_cleanup_modules(isp);
error_irq_dma:
	devm_free_irq(&pdev->dev, isp->irq_ipc, isp);
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, isp);
	return ret;
}

static const struct of_device_id area51_isp_dt_match[] = {
	{ .compatible = "soc-isp", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, area51_isp_dt_match);

static struct platform_driver area51_driver = {
	.probe = area51_probe,
	.remove = area51_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = MV_ISP_DRV_NAME,
		.of_match_table = of_match_ptr(area51_isp_dt_match)
	},
};

/*
 * area51_init - ISP module initialization.
 */
static int __init area51_init(void)
{
	isp_qos_idle.name = MV_ISP_NAME;
	pm_qos_add_request(&isp_qos_idle, PM_QOS_CPUIDLE_BLOCK,
			PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
	return platform_driver_register(&area51_driver);
}

/*
 * area51_cleanup - ISP module cleanup.
 */
static void __exit area51_cleanup(void)
{
	platform_driver_unregister(&area51_driver);
	pm_qos_remove_request(&isp_qos_idle);
}

module_init(area51_init);
module_exit(area51_cleanup);

MODULE_AUTHOR("Marvell Technology Ltd.");
MODULE_DESCRIPTION("AREA51 ISP driver");
MODULE_LICENSE("GPL");
