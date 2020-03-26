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

#ifndef __UAPI_RCAR_ION_H__
#define __UAPI_RCAR_ION_H__

struct ion_custom_data {
	unsigned int cmd;
	unsigned long arg;
};

struct ion_phys_addr {
	unsigned int dma_fd;
	unsigned long phys_addr;
};

/*
* This is bitmask, as OOM_ERROR may accompany
* OOM_THRESHOLD event and vice versa
*/
enum OOM_EVENTS {
	OOM_THRESHOLD = 1,
	OOM_ERROR = 2,
};
/*OOM_STATE is used by Rcar-ion driver to check that only 1 oom callback is registered*/
enum OOM_STATE {
	OOM_RELEASED = 1,
	OOM_WAIT_EVENT,
};

/*
* In case of oom event we return also oom_threshold and
* amount of available memory
*/
struct ion_oom_event {
	__u64 memory_available;
	__u64 oom_threshold;
	__u32 oom_event;
	enum OOM_STATE state;
};

#define RCAR_ION_MAGIC		'R'
#define RCAR_ION_CUSTOM	_IOWR(RCAR_ION_MAGIC, 6, struct ion_custom_data)

/*Subcommands whithin custom IOCTL*/
enum RCAR_CUSTOM_COMMANDS {
	RCAR_GET_PHYS_ADDR = 1,
	RCAR_GET_OOM_EVENT = 2,
};

#endif /*__UAPI_RCAR_ION_H__*/
