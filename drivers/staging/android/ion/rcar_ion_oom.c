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

#include <linux/uaccess.h>
#include "rcar_ion.h"
#include "rcar_ion_oom.h"

static int rcar_ion_oom_occupy(struct ion_rcar_heap *rheap)
{
	int ret = -1;

	spin_lock(&rheap->rheap_lock);
	if (rheap->oom.state == OOM_RELEASED) {
		rheap->oom.state = OOM_WAIT_EVENT;
		ret = 0;
	}
	spin_unlock(&rheap->rheap_lock);
	return ret;
}

static void rcar_ion_oom_release(struct ion_rcar_heap *rheap)
{
	spin_lock(&rheap->rheap_lock);
	rheap->oom.state = OOM_RELEASED;
	spin_unlock(&rheap->rheap_lock);
}

void rcar_ion_set_oom(struct ion_rcar_heap *rheap, enum OOM_EVENTS oom)
{
	spin_lock(&rheap->rheap_lock);
	rheap->oom.oom_event |= oom;
	spin_unlock(&rheap->rheap_lock);
}

void rcar_ion_clear_oom(struct ion_rcar_heap *rheap, enum OOM_EVENTS oom)
{
	spin_lock(&rheap->rheap_lock);
	rheap->oom.oom_event &= ~oom;
	spin_unlock(&rheap->rheap_lock);
}

static void rcar_ion_set_oom_threshold(struct ion_rcar_heap *rheap, u64 threshold)
{
	spin_lock(&rheap->rheap_lock);
	pr_debug("++--%s: Set threshold to %lld\n", __func__, threshold);
	rheap->oom.oom_threshold = threshold;
	if (rheap->oom.memory_available > rheap->oom.oom_threshold)
		rheap->oom.oom_event &= ~OOM_THRESHOLD;
	else
		rheap->oom.oom_event |= OOM_THRESHOLD;

	spin_unlock(&rheap->rheap_lock);
}

u32 rcar_ion_check_oom_event(struct ion_rcar_heap *rheap)
{
	u32 oom_event;

	spin_lock(&rheap->rheap_lock);
	oom_event = rheap->oom.oom_event;
	spin_unlock(&rheap->rheap_lock);
	return oom_event;
}

static struct ion_oom_event rcar_ion_get_oom_data(struct ion_rcar_heap *rheap)
{
	struct ion_oom_event oom_event;

	spin_lock(&rheap->rheap_lock);
	memcpy((void *)&oom_event, (const void *)&rheap->oom, sizeof(oom_event));
	spin_unlock(&rheap->rheap_lock);
	return oom_event;
}

bool rcar_ion_increase_available(struct ion_rcar_heap *rheap, size_t size)
{
	bool threshold = false;

	spin_lock(&rheap->rheap_lock);
	rheap->oom.memory_available += size;
	if (rheap->oom.oom_event & OOM_THRESHOLD) {
		if (rheap->oom.memory_available > rheap->oom.oom_threshold) {
			rheap->oom.oom_event &= ~OOM_THRESHOLD;
			pr_debug("++--%s: THREASHOLD CLEAR\n", __func__);
		} else {
			threshold = true;
		}
	}

	spin_unlock(&rheap->rheap_lock);
	return threshold;
}

bool rcar_ion_decrease_available(struct ion_rcar_heap *rheap, size_t size)
{
	bool threshold = false;

	spin_lock(&rheap->rheap_lock);
	if (size < rheap->oom.memory_available)
		rheap->oom.memory_available -= size;
	else
		rheap->oom.memory_available = 0;

	if (rheap->oom.oom_threshold) {
		if (rheap->oom.memory_available <= rheap->oom.oom_threshold) {
			threshold = true;
			pr_debug("++--%s: THREASHOLD\n", __func__);
			rheap->oom.oom_event |= OOM_THRESHOLD;
		}
	}
	spin_unlock(&rheap->rheap_lock);
	return threshold;
}

long rcar_ion_get_oom_event(unsigned long arg)
{
	struct ion_oom_event oom;
	struct ion_rcar_heap *rheap = rcar_ion_get_heap();
	int ret = -1;

	pr_debug("++%s\n", __func__);

	if (rcar_ion_oom_occupy(rheap)) {
		pr_err("%s: Error: oom event callback already pending\n", __func__);
		return -EFAULT;
	}

	if (copy_from_user(&oom, (void __user *)arg,
		sizeof(struct ion_oom_event))) {
		ret = -EFAULT;
		goto oom_release;
	}

	if (oom.oom_threshold)
		rcar_ion_set_oom_threshold(rheap, oom.oom_threshold);

	if (wait_event_interruptible(rheap->wq_oom, rcar_ion_check_oom_event(rheap))) {
		pr_debug("++--%s:Interrupted!!(%s)\n", __func__, current->comm);
		ret = -EINTR;
		goto oom_release;
	}
	/*We are here due to oom event. Report this to user space*/
	oom = rcar_ion_get_oom_data(rheap);
	if (copy_to_user((void __user *)arg, (const void *)&oom,
		sizeof(struct ion_oom_event))) {
		ret = -EFAULT;
		goto oom_release;
	}
	ret = 0;

oom_release:
	rcar_ion_oom_release(rheap);
	pr_debug("--%s\n", __func__);
	return ret;
}
