/*
 * isp.h
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


#ifndef ISP_CORE_H
#define ISP_CORE_H

#include <media/v4l2-device.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/wait.h>

#include <media/mrvl-camera.h>
#include "ispdma.h"
#include "ispccic.h"

#define MHZ	1000000

struct pad_formats {
	enum v4l2_mbus_pixelcode mbusfmt;
	enum v4l2_colorspace colorspace;
};

struct isp_link_dscr {
	__u8	src_sd;
	__u8	dst_sd;
	u16		src_pad;
	u16		dst_pad;
	u32		flags;
};
enum mvisp_interface_type {
	ISP_INTERFACE_PARALLEL_0,
	ISP_INTERFACE_PARALLEL_1,
	ISP_INTERFACE_CCIC_1,
	ISP_INTERFACE_CCIC_2,
};
struct isp_sensor {
	struct v4l2_subdev	*sd;
	enum mvisp_interface_type sensor_interface;
	bool    sensor_connected;
};

struct area51_device {
	/* platform HW resources */
	struct isp_build	*manager;
	struct device_node *np;
	struct isp_sensor	sensor[2];
	/* interrupts within isp */
	unsigned int irq_ipc;

	int		two_sensor_support;
	unsigned long	min_fclk_mhz;
	void	(*plat_lpm_update)(int level);
};

enum {
	PCAM_IP_AREA51,
	PCAM_IP_CCIC,
	PCAM_IP_CNT,
};

enum area51_subdev_code {
	SDCODE_AREA51_CORE	= 0,
	SDCODE_AREA51_DMAI,
	SDCODE_AREA51_DMAD,
	SDCODE_AREA51_DMAC,
	SDCODE_AREA51_CCIC1,
	SDCODE_AREA51_CCIC2,
	SDCODE_AREA51_DPHY1,
	SDCODE_AREA51_DPHY2,
};

static inline struct isp_subdev
	*isp_build_get_isd(struct isp_build *build, __u8 sd_code)
{
	struct isp_subdev *isd;

	list_for_each_entry(isd, &build->ispsd_list, hook)
		if (isd->sd_code == sd_code)
				return isd;
	return NULL;
}

int area51_ispsd_register(struct isp_subdev *ispsd);
int area51_resrc_register(struct device *dev, struct resource *res,
	const char *name, struct block_id mask,
	int res_id, void *handle, void *priv);

#endif	/* ISP_CORE_H */
