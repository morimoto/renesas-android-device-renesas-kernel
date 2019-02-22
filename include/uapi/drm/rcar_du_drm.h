/*
 * rcar_du_drm.h  --  R-Car Display Unit DRM driver
 *
 * Copyright (C) 2018 Renesas Electronics Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __RCAR_DU_DRM_H__
#define __RCAR_DU_DRM_H__

struct rcar_du_vmute {
	int crtc_id;	/* CRTCs ID */
	int on;		/* Vmute function ON/OFF */
};

/* DRM_IOCTL_RCAR_DU_SET_SCRSHOT:VSPD screen shot */
struct rcar_du_screen_shot {
	unsigned long	buff;
	unsigned int	buff_len;
	unsigned int	crtc_id;
	unsigned int	fmt;
	unsigned int	width;
	unsigned int	height;
};

/* DRM_RCAR_DU_CMM_SET_LUT, DU-CMM set LUT */
/* DRM_RCAR_DU_CMM_SET_CLU: DU-CMM set CLU */
/* DRM_RCAR_DU_CMM_GET_HGO: DU-CMM get histogram */
struct rcar_du_cmm_table {
	unsigned int	crtc_id;
	unsigned int	buff;	/* set DRM_RCAR_DU_CMM_ALLOC handle */
	unsigned int	buff_len;
	uint64_t	user_data;
};

/* DRM_RCAR_DU_CMM_SET_HGO: DU-CMM set HGO */
struct rcar_du_cmm_hgo_config {
	unsigned int	crtc_id;
	unsigned int	x_offset;
	unsigned int	y_offset;
	unsigned int	width;
	unsigned int	height;
	unsigned int	mode;
	unsigned int	ctrl;
};

/* DRM_RCAR_DU_CMM_WAIT_EVENT: DU-CMM get event */
struct rcar_du_cmm_event {
	unsigned int	crtc_id;
	unsigned int	event;
	uint64_t	callback_data;
};

/* DRM_RCAR_DU_CMM_CONFIG: DU-CMM set config */
struct rcar_du_cmm_config {
	unsigned int	crtc_id;
	unsigned int	csc;
	unsigned int	lut_buf;
	unsigned int	clu_buf;
	bool		authority;
};

/* DRM_RCAR_DU_CMM_ALLOC: DU-CMM alloc cma buffer */
/* DRM_RCAR_DU_CMM_FREE: DU-CMM free cma buffer */
struct rcar_du_cmm_buf {
	size_t		size;		/* in */
	uint64_t	mmap_offset;	/* out */
	uint64_t	phy_addr;	/* out */
	unsigned int	handle;		/* out */
};

/* DRM_RCAR_DU_CMM_WAIT_EVENT: DU-CMM done event */
#define CMM_EVENT_CLU_DONE		(1 << 0)
#define CMM_EVENT_HGO_DONE		(1 << 1)
#define CMM_EVENT_LUT_DONE		(1 << 2)

/* DRM_RCAR_DU_CMM_SET_HGO: DU-CMM set HGO mode */
#define HGO_MODE_MAXRGB			(1 << 7)
#define HGO_MODE_OFSB_R			(1 << 6)
#define HGO_MODE_OFSB_G			(1 << 5)
#define HGO_MODE_OFSB_B			(1 << 4)
#define HGO_MODE_HRATIO_NO_SKIPP	(0 << 2)
#define HGO_MODE_HRATIO_HALF_SKIPP	(1 << 2)
#define HGO_MODE_HRATIO_QUARTER_SKIPP	(2 << 2)
#define HGO_MODE_VRATIO_NO_SKIPP	(0 << 0)
#define HGO_MODE_VRATIO_HALF_SKIPP	(1 << 0)
#define HGO_MODE_VRATIO_QUARTER_SKIPP	(2 << 0)

#define HGO_CTRL_BEFORE_CLU		(0 << 0)
#define HGO_CTRL_BEFORE_LUT		(1 << 0)

/* DRM_RCAR_DU_CMM_CONFIG: DU-CMM config */
#define CSC_CONVERT_NONE		0
#define CSC_CONVERT_BT601_YCbCr240	1
#define CSC_CONVERT_BT601_YCbCr255	2
#define CSC_CONVERT_BT709_RGB255	3
#define CSC_CONVERT_BT709_RGB235	4

#define LUT_DOUBLE_BUFFER_AUTO		0
#define LUT_DOUBLE_BUFFER_A		1
#define LUT_DOUBLE_BUFFER_B		2

#define CLU_DOUBLE_BUFFER_AUTO		0
#define CLU_DOUBLE_BUFFER_A		1
#define CLU_DOUBLE_BUFFER_B		2

/* rcar-du + vspd specific ioctls */
#define DRM_RCAR_DU_SET_VMUTE		0
#define DRM_RCAR_DU_SCRSHOT		4

/* DU-CMM ioctl */
#define DRM_RCAR_DU_CMM_FUNC_BASE	(DRM_RCAR_DU_SCRSHOT + 1)
#define DRM_RCAR_DU_CMM_SET_CLU		(DRM_RCAR_DU_CMM_FUNC_BASE + 0)
#define DRM_RCAR_DU_CMM_SET_HGO		(DRM_RCAR_DU_CMM_FUNC_BASE + 1)
#define DRM_RCAR_DU_CMM_GET_HGO		(DRM_RCAR_DU_CMM_FUNC_BASE + 2)
#define DRM_RCAR_DU_CMM_START_HGO	(DRM_RCAR_DU_CMM_FUNC_BASE + 3)
#define DRM_RCAR_DU_CMM_WAIT_EVENT	(DRM_RCAR_DU_CMM_FUNC_BASE + 4)
#define DRM_RCAR_DU_CMM_CONFIG		(DRM_RCAR_DU_CMM_FUNC_BASE + 5)
#define DRM_RCAR_DU_CMM_SET_LUT		(DRM_RCAR_DU_CMM_FUNC_BASE + 6)
#define DRM_RCAR_DU_CMM_ALLOC		(DRM_RCAR_DU_CMM_FUNC_BASE + 7)
#define DRM_RCAR_DU_CMM_FREE		(DRM_RCAR_DU_CMM_FUNC_BASE + 8)

#define DRM_IOCTL_RCAR_DU_SET_VMUTE \
	DRM_IOW(DRM_COMMAND_BASE + DRM_RCAR_DU_SET_VMUTE, \
		struct rcar_du_vmute)

#define DRM_IOCTL_RCAR_DU_SCRSHOT \
	DRM_IOW(DRM_COMMAND_BASE + DRM_RCAR_DU_SCRSHOT, \
		struct rcar_du_screen_shot)
/* DU-CMM ioctl */
#define DRM_IOCTL_RCAR_DU_CMM_SET_CLU \
	DRM_IOW(DRM_COMMAND_BASE + DRM_RCAR_DU_CMM_SET_CLU, \
		struct rcar_du_cmm_table)

#define DRM_IOCTL_RCAR_DU_CMM_SET_HGO \
	DRM_IOW(DRM_COMMAND_BASE + DRM_RCAR_DU_CMM_SET_HGO, \
		struct rcar_du_cmm_hgo_config)

#define DRM_IOCTL_RCAR_DU_CMM_GET_HGO \
	DRM_IOW(DRM_COMMAND_BASE + DRM_RCAR_DU_CMM_GET_HGO, \
		struct rcar_du_cmm_table)

#define DRM_IOCTL_RCAR_DU_CMM_START_HGO \
	DRM_IOW(DRM_COMMAND_BASE + DRM_RCAR_DU_CMM_START_HGO, \
		int)

#define DRM_IOCTL_RCAR_DU_CMM_WAIT_EVENT \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_RCAR_DU_CMM_WAIT_EVENT, \
		struct rcar_du_cmm_event)

#define DRM_IOCTL_RCAR_DU_CMM_CONFIG \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_RCAR_DU_CMM_CONFIG, \
		struct rcar_du_cmm_config)

#define DRM_IOCTL_RCAR_DU_CMM_SET_LUT \
	DRM_IOW(DRM_COMMAND_BASE + DRM_RCAR_DU_CMM_SET_LUT, \
		struct rcar_du_cmm_table)

#define DRM_IOCTL_RCAR_DU_CMM_ALLOC \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_RCAR_DU_CMM_ALLOC, \
		struct rcar_du_cmm_buf)

#define DRM_IOCTL_RCAR_DU_CMM_FREE \
	DRM_IOW(DRM_COMMAND_BASE + DRM_RCAR_DU_CMM_FREE, \
		struct rcar_du_cmm_buf)

#endif /* __RCAR_DU_DRM_H__ */
