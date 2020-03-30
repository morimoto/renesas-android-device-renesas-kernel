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

#define RCAR_ION_MAGIC		'R'
#define RCAR_ION_CUSTOM	_IOWR(RCAR_ION_MAGIC, 6, struct ion_custom_data)

/*Subcommands whithin custom IOCTL*/
enum RCAR_CUSTOM_COMMANDS {
	RCAR_GET_PHYS_ADDR = 1,
};

#endif /*__UAPI_RCAR_ION_H__*/
