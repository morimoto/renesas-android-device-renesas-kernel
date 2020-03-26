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

#ifndef __RCAR_ION_H__
#define __RCAR_ION_H__
#include "ion.h"
#include "../uapi/rcar_ion.h"

struct ion_rcar_heap {
	struct ion_heap heap;
	struct gen_pool *pool;
	phys_addr_t base;
	wait_queue_head_t wq_oom;
	struct ion_oom_event oom;
	spinlock_t rheap_lock;
};

void rcar_ion_set_heap(struct ion_rcar_heap *heap);
struct ion_rcar_heap *rcar_ion_get_heap(void);

#endif /*__RCAR_ION_H__*/
