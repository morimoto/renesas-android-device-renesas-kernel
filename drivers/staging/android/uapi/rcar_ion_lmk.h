/*
 * Copyright (C) 2020 GlobalLogic
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

#ifndef __UAPI_RCAR_ION_LMK_H__
#define __UAPI_RCAR_ION_LMK_H__

#include <linux/sched.h>

/* This ifdef for Android header autogeneration scripts */
#ifndef TASK_COMM_LEN
#define TASK_COMM_LEN (16)
#endif

struct ion_proc_info {
	char comm[TASK_COMM_LEN];
	char state[TASK_COMM_LEN];
	__u64 mem;
	__u32 pid;
};

struct ion_proc_info_buf {
	size_t size;
	struct ion_proc_info *info;
};

#define RCAR_ION_LMK_MAGIC 'L'

#define ION_IOC_GET_ION_CONSUMERS_СNТR _IOWR(RCAR_ION_LMK_MAGIC, 0, size_t)
#define ION_IOC_GET_ION_CONSUMERS_INFO \
        _IOWR(RCAR_ION_LMK_MAGIC, 1, struct ion_proc_info_buf)

#endif /*__UAPI_RCAR_ION_LMK_H__ */
