/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * (C) Copyright 2012 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef _UIO_AREA51_H_
#define _UIO_AREA51_H_

struct area51_get_uio_name {
	char driver[16];
};

#define AREA51_IOC_MAGIC 'C'
#define AREA51_UIO_NAME \
		_IOWR(AREA51_IOC_MAGIC, 1, struct area51_get_uio_name)
#define AREA51_POWER_ON	_IO(AREA51_IOC_MAGIC, 2)
#define AREA51_POWER_OFF	_IO(AREA51_IOC_MAGIC, 3)
#define AREA51_CLK_ON		_IO(AREA51_IOC_MAGIC, 4)
#define AREA51_CLK_OFF	_IO(AREA51_IOC_MAGIC, 5)
#define AREA51_LOCK		_IOW(AREA51_IOC_MAGIC, 6, unsigned int)
#define AREA51_UNLOCK		_IO(AREA51_IOC_MAGIC, 7)
#define AREA51_GETSET_INFO	_IOW(AREA51_IOC_MAGIC, 8, unsigned int)
#define UIO_AREA51_NAME	"uio-area51"

#endif /* _UIO_AREA51_H_ */
