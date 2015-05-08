/*
 * Marvell ISP sensor driver
 *
 * Copyright (C) 2009-2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Created:  2010 Jiaquan Su <jqsu@marvell.com>
 * Modified: 2010 Jiaquan Su <jqsu@marvell.com>
 * Modified: 2011 Henry Zhao <xzhao10@marvell.com>
 */


#ifndef _MVISP_SENSOR_H_
#define _MVISP_SENSOR_H_
#include <media/area51.h>

#define REG_TYPE_8	0
#define REG_TYPE_16	1

struct sensor_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace			colorspace;
};

struct regval_list {
	u16 reg_num;
	unsigned char value;
};

struct sensor_reg {
	unsigned int addr;
	unsigned char val;
};

extern int mvisp_set_fclk_dphy(struct v4l2_subdev *sd,
		struct v4l2_sensor_csi_dphy *sensor_dphy);
extern void mvisp_set_sensor_mclk(int on);
#endif
