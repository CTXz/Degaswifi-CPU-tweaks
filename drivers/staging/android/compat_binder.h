/*
 * Copyright (C) 2008 Google, Inc.
 *
 * Based on, but no longer compatible with, the original
 * OpenBinder.org binder driver interface, which is:
 *
 * Copyright (c) 2005 Palmsource, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_COMPAT_BINDER_H
#define _LINUX_COMPAT_BINDER_H

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#else
#include <linux/types.h>
typedef u32		compat_size_t;
typedef u32		compat_ulong_t;
typedef s32		compat_long_t;
typedef u32		compat_uptr_t;
typedef s32		compat_pid_t;
typedef u32		__compat_uid_t;
/* do nothing actually */
static inline void __user *compat_ptr(compat_uptr_t uptr)
{
	return (void __user *)(unsigned long)uptr;
}

static inline compat_uptr_t ptr_to_compat(void __user *uptr)
{
	return (u32)(unsigned long)uptr;
}

#endif

struct compat_flat_binder_object {
	/* 8 bytes for large_flat_header. */
	__u32		type;
	__u32		flags;

	/* 8 bytes of data. */
	union {
		compat_uptr_t	binder;	/* local object */
		__u32	handle;	/* remote object */
	};

	/* extra data associated with local object */
	compat_uptr_t cookie;
};

struct compat_binder_write_read {
	compat_size_t	write_size;	/* bytes to write */
	compat_size_t	write_consumed;	/* bytes consumed by driver */
	compat_ulong_t	write_buffer;
	compat_size_t	read_size;	/* bytes to read */
	compat_size_t	read_consumed;	/* bytes consumed by driver */
	compat_uptr_t	read_buffer;
};

#define COMPAT_BINDER_WRITE_READ		_IOWR('b', 1, struct compat_binder_write_read)

struct compat_binder_transaction_data {
	union {
		__u32	handle;	/* target descriptor of command transaction */
		compat_uptr_t	ptr;	/* target descriptor of return transaction */
	} target;
	compat_uptr_t	cookie;	/* target object cookie */
	__u32		code;	/* transaction command */

	/* General information about the transaction. */
	__u32	flags;
	compat_pid_t	sender_pid;
	__compat_uid_t	sender_euid;
	compat_size_t	data_size;	/* number of bytes of data */
	compat_size_t	offsets_size;	/* number of bytes of offsets */

	union {
		struct {
			/* transaction data */
			compat_uptr_t	buffer;
			/* offsets from buffer to flat_binder_object structs */
			compat_uptr_t	offsets;
		} ptr;
		uint8_t	buf[8];
	} data;
};

static inline void binder_ctr_to_tr(struct binder_transaction_data *tr, struct compat_binder_transaction_data *ctr)
{
	tr->target.ptr = compat_ptr(ctr->target.ptr);
	tr->cookie = compat_ptr(ctr->cookie);
	tr->code = ctr->code;
	tr->flags = ctr->flags;
	tr->sender_pid = ctr->sender_pid;
	tr->sender_euid = ctr->sender_euid;
	tr->data_size = ctr->data_size;
	tr->offsets_size = ctr->offsets_size;
	tr->data.ptr.buffer = compat_ptr(ctr->data.ptr.buffer);
	tr->data.ptr.offsets = compat_ptr(ctr->data.ptr.offsets);
}

static inline void binder_tr_to_ctr(struct compat_binder_transaction_data *ctr, struct binder_transaction_data *tr)
{
	ctr->target.ptr = ptr_to_compat(tr->target.ptr);
	ctr->cookie = ptr_to_compat(tr->cookie);
	ctr->code = tr->code;
	ctr->flags = tr->flags;
	ctr->sender_pid = tr->sender_pid;
	ctr->sender_euid = tr->sender_euid;
	ctr->data_size = tr->data_size;
	ctr->offsets_size = tr->offsets_size;
	ctr->data.ptr.buffer = ptr_to_compat((void *)tr->data.ptr.buffer);
	ctr->data.ptr.offsets = ptr_to_compat((void *)tr->data.ptr.offsets);
}

struct compat_binder_ptr_cookie {
	compat_uptr_t	ptr;
	compat_uptr_t	cookie;
};

struct compat_binder_pri_ptr_cookie {
	int priority;
	compat_uptr_t	ptr;
	compat_uptr_t	cookie;
};

enum compat_binder_driver_return_protocol {
	COMPAT_BR_TRANSACTION = _IOR('r', 2, struct compat_binder_transaction_data),
	COMPAT_BR_REPLY = _IOR('r', 3, struct compat_binder_transaction_data),
	COMPAT_BR_INCREFS = _IOR('r', 7, struct compat_binder_ptr_cookie),
	COMPAT_BR_ACQUIRE = _IOR('r', 8, struct compat_binder_ptr_cookie),
	COMPAT_BR_RELEASE = _IOR('r', 9, struct compat_binder_ptr_cookie),
	COMPAT_BR_DECREFS = _IOR('r', 10, struct compat_binder_ptr_cookie),
	COMPAT_BR_ATTEMPT_ACQUIRE = _IOR('r', 11, struct compat_binder_pri_ptr_cookie),
	COMPAT_BR_DEAD_BINDER = _IOR('r', 15, compat_uptr_t),
	COMPAT_BR_CLEAR_DEATH_NOTIFICATION_DONE = _IOR('r', 16, compat_uptr_t),
};

enum compat_binder_driver_command_protocol {
	COMPAT_BC_TRANSACTION = _IOW('c', 0, struct compat_binder_transaction_data),
	COMPAT_BC_REPLY = _IOW('c', 1, struct compat_binder_transaction_data),
	COMPAT_BC_FREE_BUFFER = _IOW('c', 3, compat_uptr_t),
	COMPAT_BC_INCREFS_DONE = _IOW('c', 8, struct compat_binder_ptr_cookie),
	COMPAT_BC_ACQUIRE_DONE = _IOW('c', 9, struct compat_binder_ptr_cookie),
	COMPAT_BC_REQUEST_DEATH_NOTIFICATION = _IOW('c', 14, struct compat_binder_ptr_cookie),
	COMPAT_BC_CLEAR_DEATH_NOTIFICATION = _IOW('c', 15, struct compat_binder_ptr_cookie),
	COMPAT_BC_DEAD_BINDER_DONE = _IOW('c', 16, compat_uptr_t),
};

#endif /* _LINUX_COMPAT_BINDER_H */
