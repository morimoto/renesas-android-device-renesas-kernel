/*************************************************************************/ /*
 * DU CMM
 *
 * Copyright (C) 2018 Renesas Electronics Corporation
 *
 * License        Dual MIT/GPLv2
 *
 * The contents of this file are subject to the MIT license as set out below.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * Alternatively, the contents of this file may be used under the terms of
 * the GNU General Public License Version 2 ("GPL") in which case the provisions
 * of GPL are applicable instead of those above.
 *
 * If you wish to allow use of your version of this file only under the terms of
 * GPL, and not to allow others to use your version of this file under the terms
 * of the MIT license, indicate your decision by deleting the provisions above
 * and replace them with the notice and other provisions required by GPL as set
 * out in the file called "GPL-COPYING" included in this distribution. If you do
 * not delete the provisions above, a recipient may use your version of this
 * file under the terms of either the MIT license or GPL.
 *
 * This License is also included in this distribution in the file called
 * "MIT-COPYING".
 *
 * EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS
 * OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 * IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *
 * GPLv2:
 * If you wish to use this file under the terms of GPL, following terms are
 * effective.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */ /*************************************************************************/

#include <linux/workqueue.h>
#include <linux/syscalls.h>
#include <linux/reset.h>
#include <linux/sys_soc.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/rcar_du_drm.h>

#include "rcar_du_crtc.h"
#include "rcar_du_drv.h"
#include "rcar_du_kms.h"
#include "rcar_du_plane.h"
#include "rcar_du_regs.h"
#include <linux/clk.h>

/* #define DEBUG_PROCE_TIME 1 */

#define CMM_CLU_SAMPLES 17
#define CMM_LUT_NUM 256
#define CMM_CLU_NUM (CMM_CLU_SAMPLES * CMM_CLU_SAMPLES * CMM_CLU_SAMPLES)
#define CMM_HGO_NUM 64

enum {
	QUE_STAT_PENDING,
	QUE_STAT_ACTIVE,
	QUE_STAT_DONE,
};

static const struct soc_device_attribute rcar_du_cmm_r8a7795_es1[] = {
	{ .soc_id = "r8a7795", .revision = "ES1.*" },
	{ /* sentinel */ }
};

struct rcar_du_cmm;
struct rcar_du_cmm_file_priv;

struct rcar_du_cmm_pending_event {
	struct list_head link;
	struct list_head  fpriv_link;
	unsigned int event;
	unsigned int stat;
	unsigned long callback_data;
	struct drm_gem_object *gem_obj;
	struct rcar_du_cmm *du_cmm;
	struct rcar_du_cmm_file_priv *fpriv;
};

struct cmm_module_t {
	struct list_head list;
	union {
		struct {
			struct rcar_du_cmm_pending_event *p;
			int buf_mode;
			bool one_side;
		};
		int reset;
	};
};

struct cmm_reg_save {
#ifdef CONFIG_PM_SLEEP
	u32 *lut_table;
	u32 *clu_table;
#endif /* CONFIG_PM_SLEEP */
	wait_queue_head_t wait;

	u32 cm2_ctl0;	/* CM2_CTL0 */
	u32 hgo_offset;	/* CM2_HGO_OFFSET */
	u32 hgo_size;	/* CM2_HGO_SIZE */
	u32 hgo_mode;	/* CM2_HGO_MODE */
};

struct rcar_du_cmm {
	struct rcar_du_crtc *rcrtc;

	/* CMM base address */
	void __iomem *cmm_base;
	struct clk *clock;

	struct cmm_module_t lut;
	struct cmm_module_t clu;
	struct cmm_module_t hgo;

	struct mutex lock;	/* lock for register setting */
	struct workqueue_struct *workqueue;
	struct work_struct work;

	struct cmm_reg_save reg_save;
	bool active;
	bool dbuf;
	bool clu_dbuf;
	bool init;
	bool direct;
	bool vsync;
	bool authority;
	pid_t pid;
	bool soc_support;
};

struct rcar_du_cmm_file_priv {
	wait_queue_head_t event_wait;
	struct list_head list;
	struct list_head active_list;
	struct list_head *done_list;
};

static DEFINE_MUTEX(cmm_event_lock);
static DEFINE_SPINLOCK(cmm_direct_lock);

static inline void event_prev_cancel_locked(struct cmm_module_t *module);

static inline u32 cmm_index(struct rcar_du_cmm *_cmm)
{
	struct rcar_du_device *rcdu = _cmm->rcrtc->group->dev;

	if (rcar_du_has(rcdu, RCAR_DU_FEATURE_R8A77965_REGS)) {
		if ((_cmm)->rcrtc->index == 3)
			return 2;
	}
	return (_cmm)->rcrtc->index;
}

#define cmm_done_list(_cmm, _fpriv) \
	(&((_fpriv)->done_list[cmm_index(_cmm)]))

static inline u32 rcar_du_cmm_read(struct rcar_du_cmm *du_cmm, u32 reg)
{
	return ioread32(du_cmm->cmm_base + reg);
}

static inline void rcar_du_cmm_write(struct rcar_du_cmm *du_cmm,
				     u32 reg, u32 data)
{
	iowrite32(data, du_cmm->cmm_base + reg);
}

/* create default CLU table data */
static inline u32 index_to_clu_data(int index)
{
	int r, g, b;

	r = index % CMM_CLU_SAMPLES;
	index /= CMM_CLU_SAMPLES;
	g = index % CMM_CLU_SAMPLES;
	index /= CMM_CLU_SAMPLES;
	b = index % CMM_CLU_SAMPLES;

	r = (r << 20);
	if (r > (255 << 16))
		r = (255 << 16);
	g = (g << 12);
	if (g > (255 << 8))
		g = (255 << 8);
	b = (b << 4);
	if (b > (255 << 0))
		b = (255 << 0);

	return r | g | b;
}

static struct rcar_du_crtc *id_to_rcrtc(struct drm_device *dev, int crtc_id)
{
	struct drm_mode_object *obj;
	struct drm_crtc *crtc;

	obj = drm_mode_object_find(dev, crtc_id, DRM_MODE_OBJECT_CRTC);
	if (!obj)
		return NULL;

	crtc = obj_to_crtc(obj);

	return to_rcar_crtc(crtc);
}

static struct rcar_du_cmm *id_to_cmm(struct drm_device *dev, int crtc_id)
{
	struct rcar_du_crtc *rcrtc = id_to_rcrtc(dev, crtc_id);
	struct rcar_du_cmm *du_cmm;

	if (!rcrtc)
		return NULL;

	du_cmm = rcrtc->cmm_handle;

	if (!du_cmm)
		return NULL;

	if (!du_cmm->active)
		return NULL;

	return du_cmm;
}

#ifdef DEBUG_PROCE_TIME
static long long diff_timevals(struct timeval *start, struct timeval *end)
{
	return (end->tv_sec * 1000000LL + end->tv_usec) -
		(start->tv_sec * 1000000LL + start->tv_usec);
}
#endif

int rcar_du_cmm_config(struct drm_device *dev, void *data,
		       struct drm_file *file_priv)
{
	struct rcar_du_cmm_config *config;
	struct rcar_du_cmm *du_cmm;
	u32 cm2_ctl0;

	if (!data)
		return -ENOMEM;

	config = (struct rcar_du_cmm_config *)data;

	du_cmm = id_to_cmm(dev, config->crtc_id);
	if (!du_cmm)
		return -EINVAL;

	mutex_lock(&cmm_event_lock);

	if (du_cmm->authority) {
		if (du_cmm->pid != sys_getpid()) {
			mutex_unlock(&cmm_event_lock);
			return -EBUSY;
		}
	}
	du_cmm->authority = config->authority;
	du_cmm->pid = sys_getpid();

	mutex_unlock(&cmm_event_lock);

	cm2_ctl0 = du_cmm->reg_save.cm2_ctl0;
	cm2_ctl0 &= ~(CM2_CTL0_TM1_MASK | CM2_CTL0_TM0_MASK |
		      CM2_CTL0_YC | CM2_CTL0_DBUF | CM2_CTL0_CLUDB);

	switch (config->csc) {
	case CSC_CONVERT_NONE:
		break;

	case CSC_CONVERT_BT601_YCbCr240:
		cm2_ctl0 |= (CM2_CTL0_TM_BT601_YC240 | CM2_CTL0_YC);
		break;

	case CSC_CONVERT_BT601_YCbCr255:
		cm2_ctl0 |= (CM2_CTL0_TM_BT601_YC255 | CM2_CTL0_YC);
		break;

	case CSC_CONVERT_BT709_RGB255:
		cm2_ctl0 |= (CM2_CTL0_TM_BT709_RG255 | CM2_CTL0_YC);
		break;

	case CSC_CONVERT_BT709_RGB235:
		cm2_ctl0 |= (CM2_CTL0_TM_BT709_RG235 | CM2_CTL0_YC);
		break;

	default:
		return -EINVAL;
	}

	if (du_cmm->dbuf) {
		switch (config->lut_buf) {
		case LUT_DOUBLE_BUFFER_AUTO:
		case LUT_DOUBLE_BUFFER_A:
		case LUT_DOUBLE_BUFFER_B:
			cm2_ctl0 |= CM2_CTL0_DBUF;
			break;

		default:
			return -EINVAL;
		}

		du_cmm->lut.buf_mode = config->lut_buf;
	} else {
		return -EINVAL;
	}

	if (du_cmm->clu_dbuf) {
		switch (config->clu_buf) {
		case CLU_DOUBLE_BUFFER_AUTO:
		case CLU_DOUBLE_BUFFER_A:
		case CLU_DOUBLE_BUFFER_B:
			cm2_ctl0 |= CM2_CTL0_CLUDB;
			break;

		default:
			return -EINVAL;
		}

		du_cmm->clu.buf_mode = config->clu_buf;
	} else {
		return -EINVAL;
	}

	rcar_du_cmm_write(du_cmm, CM2_CTL0, cm2_ctl0);
	du_cmm->reg_save.cm2_ctl0 = cm2_ctl0;

	return 0;
}

int rcar_du_cmm_lut_set(struct drm_device *dev, void *data,
			struct drm_file *file_priv)
{
	struct rcar_du_cmm_table *table;
	struct rcar_du_cmm_file_priv *fpriv = file_priv->driver_priv;
	struct rcar_du_cmm *du_cmm;
	struct rcar_du_cmm_pending_event *p;
	struct drm_gem_object *gem_obj;

	if (!data)
		return -ENOMEM;
	table = (struct rcar_du_cmm_table *)data;

	if (table->buff_len < (CMM_LUT_NUM * 4))
		return -EINVAL;

	du_cmm = id_to_cmm(dev, table->crtc_id);
	if (!du_cmm)
		return -EINVAL;

	mutex_lock(&cmm_event_lock);

	if (du_cmm->authority) {
		if (du_cmm->pid != sys_getpid()) {
			mutex_unlock(&cmm_event_lock);
			return -EBUSY;
		}
	}

	mutex_unlock(&cmm_event_lock);

	gem_obj = drm_gem_object_lookup(file_priv, table->buff);
	if (!gem_obj)
		return -ENOMEM;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p) {
		drm_gem_object_unreference_unlocked(gem_obj);
		return -ENOMEM;
	}

	p->gem_obj = gem_obj;
	p->event = CMM_EVENT_LUT_DONE;
	p->stat = QUE_STAT_PENDING;
	p->callback_data = (uint64_t)table->user_data;
	p->du_cmm = du_cmm;
	p->fpriv = fpriv;

	mutex_lock(&cmm_event_lock);

	list_add_tail(&p->link, &du_cmm->lut.list);
	list_add_tail(&p->fpriv_link, &fpriv->list);

	event_prev_cancel_locked(&du_cmm->lut);

	if (du_cmm->direct)
		queue_work(du_cmm->workqueue, &du_cmm->work);

	drm_crtc_vblank_get(&du_cmm->rcrtc->crtc);

	mutex_unlock(&cmm_event_lock);

	return 0;
}

int rcar_du_cmm_clu_set(struct drm_device *dev, void *data,
			struct drm_file *file_priv)
{
	struct rcar_du_cmm_table *table;
	struct rcar_du_cmm_file_priv *fpriv = file_priv->driver_priv;
	struct rcar_du_cmm *du_cmm;
	struct rcar_du_cmm_pending_event *p;
	struct drm_gem_object *gem_obj;
	struct rcar_du_device *rcdu;

	if (!data)
		return -ENOMEM;
	table = (struct rcar_du_cmm_table *)data;

	du_cmm = id_to_cmm(dev, table->crtc_id);
	if (!du_cmm)
		return -EINVAL;

	if (!du_cmm->soc_support) {
		rcdu = du_cmm->rcrtc->group->dev;
		dev_err(rcdu->dev, "Double buffer of CLU is not supported in H3 ES1.x.\n");
		return -EINVAL;
	}

	mutex_lock(&cmm_event_lock);

	if (du_cmm->authority) {
		if (du_cmm->pid != sys_getpid()) {
			mutex_unlock(&cmm_event_lock);
			return -EBUSY;
		}
	}

	mutex_unlock(&cmm_event_lock);

	if (table->buff_len < (CMM_CLU_NUM * 4))
		return -EINVAL;

	gem_obj = drm_gem_object_lookup(file_priv, table->buff);
	if (!gem_obj)
		return -ENOMEM;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p) {
		drm_gem_object_unreference_unlocked(gem_obj);
		return -ENOMEM;
	}

	p->gem_obj = gem_obj;
	p->event = CMM_EVENT_CLU_DONE;
	p->stat = QUE_STAT_PENDING;
	p->callback_data = (uint64_t)table->user_data;
	p->du_cmm = du_cmm;
	p->fpriv = fpriv;

	mutex_lock(&cmm_event_lock);

	list_add_tail(&p->link, &du_cmm->clu.list);
	list_add_tail(&p->fpriv_link, &fpriv->list);

	event_prev_cancel_locked(&du_cmm->clu);

	if (du_cmm->direct)
		queue_work(du_cmm->workqueue, &du_cmm->work);

	drm_crtc_vblank_get(&du_cmm->rcrtc->crtc);

	mutex_unlock(&cmm_event_lock);

	return 0;
}

int rcar_du_cmm_hgo_set(struct drm_device *dev, void *data,
			struct drm_file *file_priv)
{
	struct rcar_du_cmm_hgo_config *hgo;
	struct rcar_du_cmm *du_cmm;
	const struct drm_display_mode *mode;
	u32 hists;
	u32 cm2_ctl0;

	if (!data)
		return -ENOMEM;
	hgo = (struct rcar_du_cmm_hgo_config *)data;

	du_cmm = id_to_cmm(dev, hgo->crtc_id);
	if (!du_cmm)
		return -EINVAL;

	mutex_lock(&cmm_event_lock);

	if (du_cmm->authority) {
		if (du_cmm->pid != sys_getpid()) {
			mutex_unlock(&cmm_event_lock);
			return -EBUSY;
		}
	}

	mutex_unlock(&cmm_event_lock);

	mode = &du_cmm->rcrtc->crtc.mode;

	if (hgo->width < 1 || hgo->height < 1 ||
	    (mode->hdisplay < (hgo->x_offset + hgo->width)) ||
	    (mode->vdisplay < (hgo->y_offset + hgo->height)))
		return -EINVAL;

	if ((hgo->mode & ~CM2_HGO_MODE_MASK) ||
	    ((hgo->mode & (0x3 << 0)) == (0x3 << 0)) ||
	    ((hgo->mode & (0x3 << 2)) == (0x3 << 2)))
		return -EINVAL;

	switch (hgo->ctrl) {
	case HGO_CTRL_BEFORE_CLU:
		hists = 0;
		break;
	case HGO_CTRL_BEFORE_LUT:
		hists = CM2_CTL0_HISTS;
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&du_cmm->lock);

	rcar_du_cmm_write(du_cmm, CM2_HGO_OFFSET,
			  (hgo->x_offset << 16) | (hgo->y_offset << 0));
	rcar_du_cmm_write(du_cmm, CM2_HGO_SIZE,
			  (hgo->width << 16) | (hgo->height << 0));
	rcar_du_cmm_write(du_cmm, CM2_HGO_MODE, hgo->mode);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB_TH, 0);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB0_H, 0);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB0_V, 0);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB1_H, 0);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB1_V, 0);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB2_H, 0);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB2_V, 0);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB3_H, 0);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB3_V, 0);

	cm2_ctl0 = du_cmm->reg_save.cm2_ctl0;
	cm2_ctl0 &= ~CM2_CTL0_HISTS;
	cm2_ctl0 |= hists;

	rcar_du_cmm_write(du_cmm, CM2_CTL0, cm2_ctl0);

	du_cmm->reg_save.cm2_ctl0 = cm2_ctl0;
	du_cmm->reg_save.hgo_offset =
		(hgo->x_offset << 16) | (hgo->y_offset << 0);
	du_cmm->reg_save.hgo_size =
		(hgo->width << 16) | (hgo->height << 0);
	du_cmm->reg_save.hgo_mode = hgo->mode;

	if (du_cmm->hgo.reset == 0) {
		du_cmm->hgo.reset = 1;
		drm_crtc_vblank_get(&du_cmm->rcrtc->crtc);
	}

	mutex_unlock(&du_cmm->lock);

	return 0;
}

int rcar_du_cmm_hgo_get(struct drm_device *dev, void *data,
			struct drm_file *file_priv)
{
	struct rcar_du_cmm_table *table;
	struct rcar_du_cmm_file_priv *fpriv = file_priv->driver_priv;
	struct rcar_du_cmm *du_cmm;
	struct rcar_du_cmm_pending_event *p;
	struct drm_gem_object *gem_obj;

	if (!data)
		return -ENOMEM;
	table = (struct rcar_du_cmm_table *)data;

	if (table->buff_len < (CMM_HGO_NUM * 4 * 3))
		return -EINVAL;

	du_cmm = id_to_cmm(dev, table->crtc_id);
	if (!du_cmm)
		return -EINVAL;

	mutex_lock(&cmm_event_lock);

	if (du_cmm->authority) {
		if (du_cmm->pid != sys_getpid()) {
			mutex_unlock(&cmm_event_lock);
			return -EBUSY;
		}
	}

	mutex_unlock(&cmm_event_lock);

	gem_obj = drm_gem_object_lookup(file_priv, table->buff);
	if (!gem_obj)
		return -ENOMEM;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p) {
		drm_gem_object_unreference_unlocked(gem_obj);
		return -ENOMEM;
	}

	/* histogram read end event */
	p->gem_obj = gem_obj;
	p->event = CMM_EVENT_HGO_DONE;
	p->stat = QUE_STAT_PENDING;
	p->callback_data = (uint64_t)table->user_data;
	p->du_cmm = du_cmm;
	p->fpriv = fpriv;

	mutex_lock(&cmm_event_lock);

	list_add_tail(&p->link, &du_cmm->hgo.list);
	list_add_tail(&p->fpriv_link, &fpriv->list);

	/* start DU Vsync interrupt */
	drm_crtc_vblank_get(&du_cmm->rcrtc->crtc);

	mutex_unlock(&cmm_event_lock);
	return 0;
}

int rcar_du_cmm_hgo_start(struct drm_device *dev, void *data,
			  struct drm_file *file_priv)
{
	int crtc_id;
	struct rcar_du_cmm *du_cmm;

	if (!data)
		return -ENOMEM;
	crtc_id = *(int *)data;

	du_cmm = id_to_cmm(dev, crtc_id);
	if (!du_cmm)
		return -EINVAL;

	mutex_lock(&cmm_event_lock);

	if (du_cmm->authority) {
		if (du_cmm->pid != sys_getpid()) {
			mutex_unlock(&cmm_event_lock);
			return -EBUSY;
		}
	}

	if (du_cmm->hgo.reset == 0) {
		du_cmm->hgo.reset = 1;
		drm_crtc_vblank_get(&du_cmm->rcrtc->crtc);
	}

	mutex_unlock(&cmm_event_lock);

	return 0;
}

static int wait_event_pop(struct rcar_du_cmm *du_cmm,
			  struct rcar_du_cmm_file_priv *fpriv,
			  unsigned int event_type,
			  struct rcar_du_cmm_pending_event **p)
{
	struct rcar_du_cmm_pending_event *_p;
	*p = NULL;
	list_for_each_entry(_p, cmm_done_list(du_cmm, fpriv), link) {
		if (_p->event & event_type) {
			*p = _p;
			break;
		}
	}

	return *p ? 1 : 0;
}

int rcar_du_cmm_wait_event(struct drm_device *dev, void *data,
			   struct drm_file *file_priv)
{
	int ret;
	struct rcar_du_cmm_event *event;
	struct rcar_du_cmm_file_priv *fpriv;
	struct rcar_du_cmm *du_cmm;
	struct rcar_du_cmm_pending_event *p;
	u32 mask;

	if (!data)
		return -ENOMEM;
	event = (struct rcar_du_cmm_event *)data;

	du_cmm = id_to_cmm(dev, event->crtc_id);
	if (!du_cmm)
		return -EINVAL;

	if (!du_cmm->active)
		return -EINVAL;

	if (!file_priv)
		return -EINVAL;
	fpriv = file_priv->driver_priv;

	mask = CMM_EVENT_CLU_DONE | CMM_EVENT_HGO_DONE | CMM_EVENT_LUT_DONE;
	if (!(event->event & mask))
		return -EINVAL;

	ret = wait_event_interruptible(fpriv->event_wait,
				       wait_event_pop(du_cmm,
						      fpriv,
						      event->event,
						      &p) ||
						      !du_cmm->active);

	if (ret < 0)
		return ret;

	if (p->event & event->event) {
		mutex_lock(&cmm_event_lock);
		list_del(&p->link);
		list_del(&p->fpriv_link);
		mutex_unlock(&cmm_event_lock);
	}

	event->event = p->event;
	event->callback_data = p->callback_data;

	kfree(p);

	return 0;
}

int rcar_du_cmm_alloc(struct drm_device *dev, void *data,
		      struct drm_file *file_priv)
{
	struct rcar_du_cmm_buf *buf;
	struct drm_gem_cma_object *cma_obj;
	struct drm_gem_object *gem_obj;
	u32 handle;
	u64 offset;
	int ret = 0;

	if (!data)
		return -ENOMEM;
	buf = (struct rcar_du_cmm_buf *)data;

	buf->handle = 0;

	/* create buffer */
	cma_obj = drm_gem_cma_create(dev, buf->size);
	if (IS_ERR(cma_obj))
		return -ENOMEM;

	gem_obj = &cma_obj->base;

	ret = drm_gem_handle_create(file_priv, gem_obj, &handle);
	if (ret) {
		dev->driver->gem_free_object(gem_obj);
		return -ENOMEM;
	}

	drm_gem_object_unreference_unlocked(gem_obj);

	ret = drm_gem_dumb_map_offset(file_priv, dev, handle, &offset);

	if (!ret) {
		/* set return value */
		buf->handle = handle;
		buf->mmap_offset = offset;
		buf->phy_addr = cma_obj->paddr;
	}

	return ret;
}

int rcar_du_cmm_free(struct drm_device *dev, void *data,
		     struct drm_file *file_priv)
{
	struct rcar_du_cmm_buf *buf;

	if (!data)
		return -ENOMEM;
	buf = (struct rcar_du_cmm_buf *)data;

	return drm_gem_handle_delete(file_priv, buf->handle);
}

static void du_cmm_clk(struct rcar_du_cmm *du_cmm, bool on)
{
	if (on)
		clk_prepare_enable(du_cmm->clock);
	else
		clk_disable_unprepare(du_cmm->clock);
}

int rcar_du_cmm_start_stop(struct rcar_du_crtc *rcrtc, bool on)
{
	struct rcar_du_cmm *du_cmm = rcrtc->cmm_handle;
	int i;
	u32 table_data;
	const struct drm_display_mode *mode;
	int w, h, x, y;

	if (!du_cmm)
		return -EINVAL;

	mutex_lock(&du_cmm->lock);

	if (!on) {
		du_cmm->active = false;

		rcar_du_cmm_write(du_cmm, CM2_LUT_CTRL, 0x00000000);
		rcar_du_cmm_write(du_cmm, CM2_CLU_CTRL, 0x00000000);

		du_cmm_clk(du_cmm, false);

		goto end;
	}

	du_cmm_clk(du_cmm, true);

	if (du_cmm->init)
		goto init_done;

	du_cmm->init = true;

	mode = &du_cmm->rcrtc->crtc.mode;

	x = (du_cmm->reg_save.hgo_offset >> 16) & 0xFFFF;
	y = (du_cmm->reg_save.hgo_offset >> 0)  & 0xFFFF;
	w = (du_cmm->reg_save.hgo_size >> 16) & 0xFFFF;
	h = (du_cmm->reg_save.hgo_size >> 0)  & 0xFFFF;
	if ((mode->hdisplay < (w + x)) || w == 0) {
		x = 0;
		w = mode->hdisplay;
	}
	if ((mode->vdisplay < (h + y)) || h == 0) {
		y = 0;
		h = mode->vdisplay;
	}
	du_cmm->reg_save.hgo_offset = (x << 16) | y;
	du_cmm->reg_save.hgo_size = (w << 16) | h;

	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		du_cmm->reg_save.cm2_ctl0 |= CM2_CTL0_VPOL;
	else
		du_cmm->reg_save.cm2_ctl0 &= ~CM2_CTL0_VPOL;

	rcar_du_cmm_write(du_cmm, CM2_CTL0, du_cmm->reg_save.cm2_ctl0);
	rcar_du_cmm_write(du_cmm, CM2_HGO_OFFSET, du_cmm->reg_save.hgo_offset);
	rcar_du_cmm_write(du_cmm, CM2_HGO_SIZE, du_cmm->reg_save.hgo_size);
	rcar_du_cmm_write(du_cmm, CM2_HGO_MODE, du_cmm->reg_save.hgo_mode);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB_TH, 0);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB0_H, 0);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB0_V, 0);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB1_H, 0);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB1_V, 0);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB2_H, 0);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB2_V, 0);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB3_H, 0);
	rcar_du_cmm_write(du_cmm, CM2_HGO_LB3_V, 0);

	/* init color table */
	for (i = 0; i < CMM_LUT_NUM; i++) {
	#ifdef CONFIG_PM_SLEEP
		table_data = du_cmm->reg_save.lut_table[i];
	#else
		table_data = ((i << 16) | (i << 8) | (i << 0));
	#endif /* CONFIG_PM_SLEEP */
		rcar_du_cmm_write(du_cmm, CM2_LUT_TBL_A(i), table_data);

		if (du_cmm->dbuf)
			rcar_du_cmm_write(du_cmm, CM2_LUT_TBL2_B(i),
					  table_data);
	}

	rcar_du_cmm_write(du_cmm, CM2_CLU_CTRL,
			  CM2_CLU_CTRL_AAI | CM2_CLU_CTRL_MVS);

	rcar_du_cmm_write(du_cmm, CM2_CLU_ADDR, 0);
	if (du_cmm->clu_dbuf)
		rcar_du_cmm_write(du_cmm, CM2_CLU_ADDR2, 0);

	for (i = 0; i < CMM_CLU_NUM; i++) {
	#ifdef CONFIG_PM_SLEEP
		table_data = du_cmm->reg_save.clu_table[i];
	#else
		table_data = index_to_clu_data(i);
	#endif /* CONFIG_PM_SLEEP */
		rcar_du_cmm_write(du_cmm, CM2_CLU_DATA, table_data);

		if (du_cmm->dbuf)
			rcar_du_cmm_write(du_cmm, CM2_CLU_DATA2,
					  table_data);
	}

init_done:
	/* enable color table */
	rcar_du_cmm_write(du_cmm, CM2_LUT_CTRL, CM2_LUT_CTRL_EN);
	rcar_du_cmm_write(du_cmm, CM2_CLU_CTRL, CM2_CLU_CTRL_AAI |
			  CM2_CLU_CTRL_MVS | CM2_CLU_CTRL_EN);

	du_cmm->active = true;
end:
	mutex_unlock(&du_cmm->lock);

	return 0;
}

#define gem_to_vaddr(gem_obj) \
	(container_of((gem_obj), struct drm_gem_cma_object, base)->vaddr)

static inline void cmm_vblank_put(struct rcar_du_cmm_pending_event *p)
{
	if (p->du_cmm)
		drm_crtc_vblank_put(&p->du_cmm->rcrtc->crtc);
}

static inline void
cmm_gem_object_unreference(struct rcar_du_cmm_pending_event *p)
{
	if (p->gem_obj)
		drm_gem_object_unreference_unlocked(p->gem_obj);
}

static inline void _event_done_locked(struct rcar_du_cmm_pending_event *p)
{
	cmm_gem_object_unreference(p);

	if (p->fpriv) {
		p->stat = QUE_STAT_DONE;
		list_del(&p->link); /* delete from p->fpriv->active_list */
		list_add_tail(&p->link, cmm_done_list(p->du_cmm, p->fpriv));
		wake_up_interruptible(&p->fpriv->event_wait);
	} else {
		/* link deleted by rcar_du_cmm_postclose */
		kfree(p);
	}
}

/* cancel from active_list (case of LUT/CLU double buffer mode) */
static inline void event_prev_cancel_locked(struct cmm_module_t *module)
{
	struct rcar_du_cmm_pending_event *p = module->p;

	if (!p)
		return;

	module->p = NULL;

	_event_done_locked(p);
}

static inline void event_done(struct rcar_du_cmm_pending_event *p)
{
	/* vblank is put */

	mutex_lock(&cmm_event_lock);

	_event_done_locked(p);

	mutex_unlock(&cmm_event_lock);
}

static inline void lc_event_done(struct cmm_module_t *module,
				 struct rcar_du_cmm_pending_event *p,
				 bool done)
{
	/* vblank is put */

	mutex_lock(&cmm_event_lock);

	if (!done && list_empty(&module->list))
		module->p = p;
	else
		_event_done_locked(p);

	mutex_unlock(&cmm_event_lock);
}

static inline struct rcar_du_cmm_pending_event *
event_pop_locked(struct cmm_module_t *module)
{
	struct rcar_du_cmm_pending_event *p =
		list_first_entry(&module->list,
				 struct rcar_du_cmm_pending_event,
				 link);

	p->stat = QUE_STAT_ACTIVE;
	list_del(&p->link); /* delete from du_cmm->[lut|clu|hgo].list */
	list_add_tail(&p->link, &p->fpriv->active_list);
	cmm_vblank_put(p);

	return p;
}

struct rcar_du_cmm_work_stat {
	union {
		struct {
			struct rcar_du_cmm_pending_event *p;
			bool done;
			bool table_copy;
		};
		struct {
			struct rcar_du_cmm_pending_event *p2;
			bool reset;
		};
	};
};

static inline void one_side(struct rcar_du_cmm *du_cmm,
			    struct cmm_module_t *module,
			    bool on)
{
	if (on && !module->one_side) {
		module->one_side = true;
		drm_crtc_vblank_get(&du_cmm->rcrtc->crtc);
	} else if (!on && module->one_side) {
		module->one_side = false;
		drm_crtc_vblank_put(&du_cmm->rcrtc->crtc);
	}
}

/* pop LUT que */
static int lut_pop_locked(struct rcar_du_cmm *du_cmm,
			  struct rcar_du_cmm_work_stat *stat)
{
	bool is_one_side = false;

	stat->done = true;
	stat->table_copy = false;

	if (!list_empty(&du_cmm->lut.list)) {
		stat->p = event_pop_locked(&du_cmm->lut);

		/* prev lut table */
		event_prev_cancel_locked(&du_cmm->lut);

		if (du_cmm->lut.buf_mode == LUT_DOUBLE_BUFFER_AUTO) {
			is_one_side = true;
			if (list_empty(&du_cmm->lut.list))
				stat->done = false;
		}

	} else if (du_cmm->lut.p) {
		/* prev lut table */
		stat->p = du_cmm->lut.p;
		du_cmm->lut.p = NULL;
	} else {
		stat->done = false;
		stat->p = NULL;
		stat->table_copy = du_cmm->lut.one_side;
	}

	one_side(du_cmm, &du_cmm->lut, is_one_side);

	return 0;
}

static int lut_table_copy(struct rcar_du_cmm *du_cmm)
{
	int i;
	u32 src, dst;

	if (rcar_du_cmm_read(du_cmm, CM2_CTL1) & CM2_CTL1_BFS) {
		dst = CM2_LUT_TBL_A(0);
		src = CM2_LUT_TBL2_B(0);
	} else {
		dst = CM2_LUT_TBL2_B(0);
		src = CM2_LUT_TBL_A(0);
	}

	for (i = 0; i < CMM_LUT_NUM; i++) {
		rcar_du_cmm_write(du_cmm, dst, rcar_du_cmm_read(du_cmm, src));
		dst += 4;
		src += 4;
	}

	return 0;
}

/* set 1D look up table */
static int lut_set(struct rcar_du_cmm *du_cmm,
		   struct rcar_du_cmm_work_stat *stat)
{
	int i;
	u32 lut_base;
	u32 *lut_buf;

	if (!stat->p) {
		if (stat->table_copy)
			lut_table_copy(du_cmm);
		return 0; /* skip */
	}

	/* set LUT */
	switch (du_cmm->lut.buf_mode) {
	case LUT_DOUBLE_BUFFER_A:
		lut_base = CM2_LUT_TBL_A(0);
		break;

	case LUT_DOUBLE_BUFFER_AUTO:
		if (rcar_du_cmm_read(du_cmm, CM2_CTL1) & CM2_CTL1_BFS) {
			lut_base = CM2_LUT_TBL_A(0);
			break;
		}
		lut_base = CM2_LUT_TBL2_B(0);
		break;
	case LUT_DOUBLE_BUFFER_B:
		lut_base = CM2_LUT_TBL2_B(0);
		break;

	default:
		return -EINVAL;
	}

	lut_buf = gem_to_vaddr(stat->p->gem_obj);
	for (i = 0; i < CMM_LUT_NUM; i++)
		rcar_du_cmm_write(du_cmm, lut_base + i * 4, lut_buf[i]);

	lc_event_done(&du_cmm->lut, stat->p, stat->done);

	return 0;
}

/* pop CLU que */
static int clu_pop_locked(struct rcar_du_cmm *du_cmm,
			  struct rcar_du_cmm_work_stat *stat)
{
	bool is_one_side = false;

	stat->done = true;
	stat->table_copy = false;

	if (!list_empty(&du_cmm->clu.list)) {
		stat->p = event_pop_locked(&du_cmm->clu);

		/* prev clu table */
		event_prev_cancel_locked(&du_cmm->clu);

		if (du_cmm->clu.buf_mode == CLU_DOUBLE_BUFFER_AUTO) {
			is_one_side = true;
			if (list_empty(&du_cmm->clu.list))
				stat->done = false;
		}

	} else if (du_cmm->clu.p) {
		/* prev clu table */
		stat->p = du_cmm->clu.p;
		du_cmm->clu.p = NULL;
	} else {
		stat->done = false;
		stat->p = NULL;
		stat->table_copy = du_cmm->clu.one_side;
	}

	one_side(du_cmm, &du_cmm->clu, is_one_side);

	return 0;
}

static int clu_table_copy(struct rcar_du_cmm *du_cmm)
{
	int i, j, k;
	u32 src_addr, src_data, dst_addr, dst_data;

	if (rcar_du_cmm_read(du_cmm, CM2_CTL1) & CM2_CTL1_BFS) {
		dst_addr = CM2_CLU_ADDR;
		dst_data = CM2_CLU_DATA;
		src_addr = CM2_CLU_ADDR2;
		src_data = CM2_CLU_DATA2;
	} else {
		dst_addr = CM2_CLU_ADDR2;
		dst_data = CM2_CLU_DATA2;
		src_addr = CM2_CLU_ADDR;
		src_data = CM2_CLU_DATA;
	}

	rcar_du_cmm_write(du_cmm, dst_addr, 0);
	for (i = 0; i < CMM_CLU_SAMPLES; i++) {
		for (j = 0; j < CMM_CLU_SAMPLES; j++) {
			for (k = 0; k < CMM_CLU_SAMPLES; k++) {
				rcar_du_cmm_write(du_cmm, src_addr,
						  (k << 16) | (j << 8) |
						  (i << 0));
				rcar_du_cmm_write(du_cmm, dst_data,
						  rcar_du_cmm_read(du_cmm,
								   src_data));
			}
		}
	}

	return 0;
}

/* set 3D look up table */
static int clu_set(struct rcar_du_cmm *du_cmm,
		   struct rcar_du_cmm_work_stat *stat)
{
	int i;
	u32 addr_reg, data_reg;
	u32 *clu_buf;

	if (!stat->p) {
		if (stat->table_copy)
			clu_table_copy(du_cmm);
		return 0; /* skip */
	}

	/* set CLU */
	switch (du_cmm->clu.buf_mode) {
	case CLU_DOUBLE_BUFFER_A:
		addr_reg = CM2_CLU_ADDR;
		data_reg = CM2_CLU_DATA;
		break;

	case CLU_DOUBLE_BUFFER_AUTO:
		if (rcar_du_cmm_read(du_cmm, CM2_CTL1) & CM2_CTL1_BFS) {
			addr_reg = CM2_CLU_ADDR;
			data_reg = CM2_CLU_DATA;
			break;
		}
		addr_reg = CM2_CLU_ADDR2;
		data_reg = CM2_CLU_DATA2;
		break;
	case CLU_DOUBLE_BUFFER_B:
		addr_reg = CM2_CLU_ADDR2;
		data_reg = CM2_CLU_DATA2;
		break;

	default:
		return -EINVAL;
	}

	clu_buf = gem_to_vaddr(stat->p->gem_obj);
	rcar_du_cmm_write(du_cmm, addr_reg, 0);
	for (i = 0; i < CMM_CLU_NUM; i++)
		rcar_du_cmm_write(du_cmm, data_reg, clu_buf[i]);

	lc_event_done(&du_cmm->clu, stat->p, stat->done);

	return 0;
}

/* pop HGO que */
static int hgo_pop_locked(struct rcar_du_cmm *du_cmm,
			  struct rcar_du_cmm_work_stat *stat)
{
	struct rcar_du_cmm_pending_event *_p = NULL;

	if (!list_empty(&du_cmm->hgo.list))
		_p = event_pop_locked(&du_cmm->hgo);

	if (du_cmm->hgo.reset) {
		drm_crtc_vblank_put(&du_cmm->rcrtc->crtc);
		du_cmm->hgo.reset = 0;
		stat->reset = true;
	} else {
		stat->reset = false;
	}

	stat->p2 = _p;

	return 0;
}

/* get histogram */
static int hgo_get(struct rcar_du_cmm *du_cmm,
		   struct rcar_du_cmm_work_stat *stat)
{
	int i, j;
	const u32 histo_offset[3] = {
		CM2_HGO_R_HISTO(0),
		CM2_HGO_G_HISTO(0),
		CM2_HGO_B_HISTO(0),
	};
	void *vaddr;

	if (!stat->p2) {
		if (stat->reset)
			goto hgo_reset;

		return 0; /* skip */
	}

	vaddr = gem_to_vaddr(stat->p2->gem_obj);
	for (i = 0; i < 3; i++) {
		u32 *hgo_buf = vaddr + CMM_HGO_NUM * 4 * i;

		for (j = 0; j < CMM_HGO_NUM; j++)
			hgo_buf[j] = rcar_du_cmm_read(du_cmm,
						      histo_offset[i] + j * 4);
	}

	event_done(stat->p2);

hgo_reset:
	rcar_du_cmm_write(du_cmm, CM2_HGO_REGRST, CM2_HGO_REGRST_RCLEA);

	return 0;
}

static bool du_cmm_vsync_get(struct rcar_du_cmm *du_cmm)
{
	unsigned long flags;
	bool vsync;

	spin_lock_irqsave(&cmm_direct_lock, flags);
	vsync = du_cmm->vsync;
	du_cmm->vsync = false;
	spin_unlock_irqrestore(&cmm_direct_lock, flags);

	return vsync;
}

static void du_cmm_vsync_set(struct rcar_du_cmm *du_cmm, bool vsync)
{
	unsigned long flags;

	spin_lock_irqsave(&cmm_direct_lock, flags);
	du_cmm->vsync = vsync;
	spin_unlock_irqrestore(&cmm_direct_lock, flags);
}

static void du_cmm_work(struct work_struct *work)
{
	struct rcar_du_cmm *du_cmm =
			container_of(work, struct rcar_du_cmm, work);
	struct rcar_du_cmm_work_stat s_lut;
	struct rcar_du_cmm_work_stat s_clu;
	struct rcar_du_cmm_work_stat s_hgo;
#ifdef DEBUG_PROCE_TIME
	struct timeval start_time, end_time;
	unsigned long lut_time, clu_time, hgo_time;
#endif
	bool vsync_status = false;

	memset(&s_lut, 0, sizeof(struct rcar_du_cmm_work_stat));
	memset(&s_clu, 0, sizeof(struct rcar_du_cmm_work_stat));
	memset(&s_hgo, 0, sizeof(struct rcar_du_cmm_work_stat));

	vsync_status = du_cmm_vsync_get(du_cmm);

	mutex_lock(&cmm_event_lock);

	lut_pop_locked(du_cmm, &s_lut);
	clu_pop_locked(du_cmm, &s_clu);
	if (vsync_status)
		hgo_pop_locked(du_cmm, &s_hgo);

	mutex_unlock(&cmm_event_lock);

	/* set LUT */
#ifdef DEBUG_PROCE_TIME
	do_gettimeofday(&start_time);
#endif
	lut_set(du_cmm, &s_lut);
#ifdef DEBUG_PROCE_TIME
	do_gettimeofday(&end_time);
	lut_time = (long)diff_timevals(&start_time, &end_time);
#endif

	/* set CLU */
#ifdef DEBUG_PROCE_TIME
	do_gettimeofday(&start_time);
#endif
	clu_set(du_cmm, &s_clu);
#ifdef DEBUG_PROCE_TIME
	do_gettimeofday(&end_time);
	clu_time = (long)diff_timevals(&start_time, &end_time);
#endif

	/* get HGO */
#ifdef DEBUG_PROCE_TIME
	do_gettimeofday(&start_time);
#endif
	if (vsync_status)
		hgo_get(du_cmm, &s_hgo);
#ifdef DEBUG_PROCE_TIME
	do_gettimeofday(&end_time);
	hgo_time = (long)diff_timevals(&start_time, &end_time);
#endif

	wake_up_interruptible(&du_cmm->reg_save.wait);

#ifdef DEBUG_PROCE_TIME
	{
		struct rcar_du_device *rcdu = du_cmm->rcrtc->group->dev;

		if (s_lut.p)
			dev_info(rcdu->dev, "LUT %ld usec.\n", lut_time);
		if (s_clu.p)
			dev_info(rcdu->dev, "LUT %ld usec.\n", clu_time);
		if (s_hgo.p2)
			dev_info(rcdu->dev, "HGO %ld usec.\n", hgo_time);
	}
#endif
}

static int du_cmm_que_empty(struct rcar_du_cmm *du_cmm)
{
	if (list_empty(&du_cmm->lut.list) && !du_cmm->lut.p &&
	    !du_cmm->lut.one_side &&
	    list_empty(&du_cmm->clu.list) && !du_cmm->clu.p &&
	    !du_cmm->clu.one_side &&
	    list_empty(&du_cmm->hgo.list) && !du_cmm->hgo.reset)
		return 1;

	return 0;
}

void rcar_du_cmm_kick(struct rcar_du_crtc *rcrtc)
{
	struct rcar_du_cmm *du_cmm = rcrtc->cmm_handle;

	if (!du_cmm)
		return;

	if (!du_cmm->active)
		return;

	if (!du_cmm_que_empty(du_cmm)) {
		du_cmm_vsync_set(du_cmm, true);
		queue_work(du_cmm->workqueue, &du_cmm->work);
	}
}

#ifdef CONFIG_PM_SLEEP
int rcar_du_cmm_pm_suspend(struct rcar_du_crtc *rcrtc)
{
	struct rcar_du_cmm *du_cmm = rcrtc->cmm_handle;
	struct rcar_du_device *rcdu = rcrtc->group->dev;
	int i, j, k, index;
	int ret;

	if (!du_cmm)
		return 0;

	ret = wait_event_timeout(du_cmm->reg_save.wait,
				 du_cmm_que_empty(du_cmm),
				 msecs_to_jiffies(500));
	if (ret == 0)
		dev_err(rcdu->dev, "rcar-du cmm suspend : timeout\n");

	if (!du_cmm->init)
		return 0;

	du_cmm->init = false;

	if (!du_cmm->active)
		du_cmm_clk(du_cmm, true);

	/* table save */
	for (i = 0; i < CMM_LUT_NUM; i++) {
		du_cmm->reg_save.lut_table[i] =
			rcar_du_cmm_read(du_cmm, CM2_LUT_TBL_A(i));
	}

	index = 0;
	for (i = 0; i < CMM_CLU_SAMPLES; i++) {
		for (j = 0; j < CMM_CLU_SAMPLES; j++) {
			for (k = 0; k < CMM_CLU_SAMPLES; k++) {
				rcar_du_cmm_write(du_cmm, CM2_CLU_ADDR,
						  (k << 16) | (j << 8) |
						  (i << 0));
				du_cmm->reg_save.clu_table[index++] =
					rcar_du_cmm_read(du_cmm, CM2_CLU_DATA);
			}
		}
	}

	if (!du_cmm->active)
		du_cmm_clk(du_cmm, false);

	return 0;
}

int rcar_du_cmm_pm_resume(struct rcar_du_crtc *rcrtc)
{
	/* none */
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

int rcar_du_cmm_driver_open(struct drm_device *dev, struct drm_file *file_priv)
{
	struct rcar_du_device *rcdu = dev->dev_private;
	struct rcar_du_cmm_file_priv *fpriv;
	int i;

	if (!rcar_du_has(rcdu, RCAR_DU_FEATURE_CMM))
		return 0;

	file_priv->driver_priv = NULL;

	fpriv = kzalloc(sizeof(*fpriv), GFP_KERNEL);
	if (unlikely(!fpriv))
		return -ENOMEM;

	fpriv->done_list = kcalloc(rcdu->num_crtcs,
				   sizeof(*fpriv->done_list),
				   GFP_KERNEL);
	if (unlikely(!fpriv->done_list)) {
		kfree(fpriv);
		return -ENOMEM;
	}

	init_waitqueue_head(&fpriv->event_wait);
	INIT_LIST_HEAD(&fpriv->list);
	INIT_LIST_HEAD(&fpriv->active_list);
	for (i = 0; i < rcdu->num_crtcs; i++)
		INIT_LIST_HEAD(&fpriv->done_list[i]);

	file_priv->driver_priv = fpriv;

	return 0;
}

void rcar_du_cmm_postclose(struct drm_device *dev, struct drm_file *file_priv)
{
	struct rcar_du_device *rcdu = dev->dev_private;
	struct rcar_du_cmm_file_priv *fpriv = file_priv->driver_priv;
	struct rcar_du_cmm_pending_event *p, *pt;
	struct rcar_du_crtc *rcrtc;
	struct rcar_du_cmm *du_cmm;
	int i, crtcs_cnt, ret;
	u32 table_data;

	if (!rcar_du_has(rcdu, RCAR_DU_FEATURE_CMM))
		return;

	mutex_lock(&cmm_event_lock);

	/* Unlink file priv events */
	list_for_each_entry_safe(p, pt, &fpriv->list, fpriv_link) {
		list_del(&p->fpriv_link);
		list_del(&p->link);
		switch (p->stat) {
		case QUE_STAT_PENDING:
			cmm_vblank_put(p);
			cmm_gem_object_unreference(p);
			kfree(p);
			break;
		case QUE_STAT_DONE:
			kfree(p);
			break;
		case QUE_STAT_ACTIVE:
			p->fpriv = NULL;
			break;
		}
	}

	mutex_unlock(&cmm_event_lock);

	kfree(fpriv->done_list);
	kfree(fpriv);
	file_priv->driver_priv = NULL;

	for (crtcs_cnt = 0; crtcs_cnt < rcdu->num_crtcs; crtcs_cnt++) {
		rcrtc = &rcdu->crtcs[crtcs_cnt];
		du_cmm = rcrtc->cmm_handle;
		if (!du_cmm)
			continue;
		if (du_cmm->authority && du_cmm->pid == sys_getpid()) {
			du_cmm->authority = false;
			du_cmm->pid = 0;
			ret = wait_event_timeout(du_cmm->reg_save.wait,
						 du_cmm_que_empty(du_cmm),
						 msecs_to_jiffies(500));
			if (ret == 0)
				dev_err(rcdu->dev, "rcar-du cmm close : timeout\n");

			for (i = 0; i < CMM_LUT_NUM; i++) {
				table_data = ((i << 16) | (i << 8) | (i << 0));
#ifdef CONFIG_PM_SLEEP
				du_cmm->reg_save.lut_table[i] = table_data;
#endif /* CONFIG_PM_SLEEP */
				rcar_du_cmm_write(du_cmm, CM2_LUT_TBL_A(i),
						  table_data);
				if (du_cmm->dbuf) {
					rcar_du_cmm_write(du_cmm,
							  CM2_LUT_TBL2_B(i),
							  table_data);
				}
			}

			for (i = 0; i < CMM_CLU_NUM; i++) {
				table_data = index_to_clu_data(i);
#ifdef CONFIG_PM_SLEEP
				du_cmm->reg_save.clu_table[i] = table_data;
#endif /* CONFIG_PM_SLEEP */
				rcar_du_cmm_write(du_cmm, CM2_CLU_DATA,
						  table_data);

				if (du_cmm->dbuf) {
					rcar_du_cmm_write(du_cmm, CM2_CLU_DATA2,
							  table_data);
				}
			}
		}
	}
}

int rcar_du_cmm_init(struct rcar_du_crtc *rcrtc)
{
	struct rcar_du_cmm *du_cmm;
	int ret;
#ifdef CONFIG_PM_SLEEP
	int i;
#endif
	struct rcar_du_device *rcdu = rcrtc->group->dev;
	char name[64];
	struct resource *mem;

	if (!rcar_du_has(rcdu, RCAR_DU_FEATURE_CMM))
		return 0;

	du_cmm = devm_kzalloc(rcdu->dev, sizeof(*du_cmm), GFP_KERNEL);
	if (!du_cmm) {
		ret = -ENOMEM;
		goto error_alloc;
	}

	/* DU-CMM mapping */
	sprintf(name, "cmm.%u", rcrtc->index);
	mem = platform_get_resource_byname(to_platform_device(rcdu->dev),
					   IORESOURCE_MEM, name);
	if (!mem) {
		dev_err(rcdu->dev, "rcar-du cmm init : failed to get memory resource\n");
		ret = -EINVAL;
		goto error_mapping_cmm;
	}
	du_cmm->cmm_base = devm_ioremap_nocache(rcdu->dev, mem->start,
						resource_size(mem));
	if (!du_cmm->cmm_base) {
		dev_err(rcdu->dev, "rcar-du cmm init : failed to map iomem\n");
		ret = -EINVAL;
		goto error_mapping_cmm;
	}
	du_cmm->clock = devm_clk_get(rcdu->dev, name);
	if (IS_ERR(du_cmm->clock)) {
		dev_err(rcdu->dev, "failed to get clock\n");
		ret = PTR_ERR(du_cmm->clock);
		goto error_clock_cmm;
	}

	du_cmm->rcrtc = rcrtc;

	du_cmm->reg_save.cm2_ctl0 = 0;
	du_cmm->reg_save.hgo_offset = 0;
	du_cmm->reg_save.hgo_size = 0;
	du_cmm->reg_save.hgo_mode = 0;

	du_cmm->dbuf = rcar_du_has(rcdu, RCAR_DU_FEATURE_CMM_LUT_DBUF);
	if (du_cmm->dbuf) {
		du_cmm->lut.buf_mode = LUT_DOUBLE_BUFFER_AUTO;
		du_cmm->reg_save.cm2_ctl0 |= CM2_CTL0_DBUF;
	} else {
		dev_err(rcdu->dev, "single buffer is not supported.\n");
		du_cmm->dbuf = true;
		du_cmm->lut.buf_mode = LUT_DOUBLE_BUFFER_AUTO;
		du_cmm->reg_save.cm2_ctl0 |= CM2_CTL0_DBUF;
	}

	du_cmm->clu_dbuf = rcar_du_has(rcdu, RCAR_DU_FEATURE_CMM_CLU_DBUF);
	if (du_cmm->clu_dbuf) {
		du_cmm->clu.buf_mode = CLU_DOUBLE_BUFFER_AUTO;
		du_cmm->reg_save.cm2_ctl0 |= CM2_CTL0_CLUDB;
	} else {
		dev_err(rcdu->dev, "single buffer is not supported.\n");
		du_cmm->clu_dbuf = true;
		du_cmm->clu.buf_mode = CLU_DOUBLE_BUFFER_AUTO;
		du_cmm->reg_save.cm2_ctl0 |= CM2_CTL0_CLUDB;
	}

#ifdef CONFIG_PM_SLEEP
	du_cmm->reg_save.lut_table =
		devm_kzalloc(rcdu->dev, CMM_LUT_NUM * 4, GFP_KERNEL);
	if (!du_cmm->reg_save.lut_table) {
		ret = -ENOMEM;
		goto error_lut_reg_save_buf;
	}
	for (i = 0; i < CMM_LUT_NUM; i++)
		du_cmm->reg_save.lut_table[i] = (i << 16) | (i << 8) | (i << 0);

	du_cmm->reg_save.clu_table =
		devm_kzalloc(rcdu->dev, CMM_CLU_NUM * 4, GFP_KERNEL);
	if (!du_cmm->reg_save.clu_table) {
		ret = -ENOMEM;
		goto error_clu_reg_save_buf;
	}
	for (i = 0; i < CMM_CLU_NUM; i++)
		du_cmm->reg_save.clu_table[i] = index_to_clu_data(i);

#endif /* CONFIG_PM_SLEEP */
	init_waitqueue_head(&du_cmm->reg_save.wait);
	if (soc_device_match(rcar_du_cmm_r8a7795_es1))
		du_cmm->soc_support = false;
	else
		du_cmm->soc_support = true;

	du_cmm->active = false;
	du_cmm->init = false;
	du_cmm->direct = true;

	mutex_init(&du_cmm->lock);
	INIT_LIST_HEAD(&du_cmm->lut.list);
	du_cmm->lut.p = NULL;
	du_cmm->lut.one_side = false;
	INIT_LIST_HEAD(&du_cmm->clu.list);
	du_cmm->clu.p = NULL;
	du_cmm->clu.one_side = false;
	INIT_LIST_HEAD(&du_cmm->hgo.list);
	du_cmm->hgo.reset = 0;

	sprintf(name, "du-cmm%d", rcrtc->index);
	du_cmm->workqueue = create_singlethread_workqueue(name);
	INIT_WORK(&du_cmm->work, du_cmm_work);

	rcrtc->cmm_handle = du_cmm;

	dev_info(rcdu->dev, "DU%d use CMM(%s buffer)\n",
		 rcrtc->index, du_cmm->dbuf ? "Double" : "Single");

	return 0;

#ifdef CONFIG_PM_SLEEP
error_clu_reg_save_buf:
error_lut_reg_save_buf:
#endif /* CONFIG_PM_SLEEP */
error_clock_cmm:
	devm_iounmap(rcdu->dev, du_cmm->cmm_base);
error_mapping_cmm:
	devm_kfree(rcdu->dev, du_cmm);
error_alloc:
	return ret;
}
