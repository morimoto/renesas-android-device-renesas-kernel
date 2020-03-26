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

#ifndef __RCAR_ION_OOM_H__
#define __RCAR_ION_OOM_H__
#include "ion.h"
#include "../uapi/rcar_ion.h"

/**
 * rcar_ion_set_oom: Sets Out-Of-Memory state. It can set
 * OOM_THRESHOLD or OOM_ERROR, or both;
 *
 * @rheap: Pointer to ion_rcar_heap structure;
 * @oom: OOM state to set (OOM_THRESHOLD, OOM_ERROR);
 */
void rcar_ion_set_oom(struct ion_rcar_heap *rheap, enum OOM_EVENTS oom);
/**
 * rcar_ion_clear_oom: Clears Out-Of-Memory state;
 * @ rheap: Pointer to ion_rcar_heap structure;
 * @ oom: oom state to clear(OOM_THRESHOLD, OOM_ERROR);
 */
void rcar_ion_clear_oom(struct ion_rcar_heap *rheap, enum OOM_EVENTS oom);
/**
 * rcar_ion_check_oom_event: Checks Out-Of-Memory state;
 * @ rheap: Pointer to ion_rcar_heap structure;
 * @ return: bitbask of OOM_THRESHOLD or OOM_ERROR;
 */
u32 rcar_ion_check_oom_event(struct ion_rcar_heap *rheap);
/**
 * rcar_ion_increase_available: Increases amount of available memory
 * in the struct ion_oom_event used for OOM tracking and reporting;
 *
 * @ rheap: Pointer to ion_rcar_heap structure;
 * @ size: memory increase size;
 * @ return: true if amount of available memory less then defined threhold
 * or false otherwise;
 */
bool rcar_ion_increase_available(struct ion_rcar_heap *rheap, size_t size);
/**
 * rcar_ion_decrease_available: Decreases amount of available memory
 * in the struct ion_oom_event used for OOM tracking and reporting;
 *
 * @ rheap: Pointer to ion_rcar_heap structure;
 * @ size: memory decrease size;
 * @ return: true if amount of available memory less then defined threhold
 * or false otherwise;
 */
bool rcar_ion_decrease_available(struct ion_rcar_heap *rheap, size_t size);
/**
 * rcar_ion_get_oom_event: Enables oom tracking and event reporting.
 * Called from user space application by IOCTL. It gets the memory
 * threhold for OOM_THRESHOLD reporting.Function blocks till given
 * OOM_THRESHOLD or OOM_ERROR event received. It unblocks then
 * and returns the oom event and available memory to the client application
 * in the struct ion_oom_event variable. This function is not reentrant,
 * it assumes 1 user space client and will return -EFAULT if called more
 * than once at a time.
 *
 * @ arg: User space pointer to ion_oom_event structure;
 * @ return: 0 in case of success or negative error code otherwise;
 */
long rcar_ion_get_oom_event(unsigned long arg);

#endif /*__RCAR_ION_OOM_H__*/
