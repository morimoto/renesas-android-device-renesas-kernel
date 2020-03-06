/** ****************************************************************************
 *\file		xf-adsp-config.h
 *\brief	Header file for ADSP configuration
 *\addtogroup	ADSP Driver
 *******************************************************************************
 *\date		Oct. 21, 2017
 *\author	Renesas Electronics Corporation
 *******************************************************************************
 *\par		Copyright
 *
 * Copyright(c) 2016 Renesas Electoronics Corporation
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 ******************************************************************************/

#ifndef __XF_ADSP_CONFIG_H
#define __XF_ADSP_CONFIG_H

/* Equalizer definition */
#define XA_REL_EQZ_FILTER_NUM		(9) /**< number of filter */
#define XA_REL_EQZ_GRAPHIC_BAND_NUM	(5) /**< number of graphic band */

struct xf_buffer {
	void			*address;
	union {						/* PRQA S 750 3 */
		struct xf_buffer *next;
		struct xf_pool   *pool;
	} link;
};

struct xf_pool {
	/* length of individual buffer in a pool */
	u32			length;

	/* number of buffer in a pool */
	u32			number;

	/* pointer to pool memory */
	void			*p;

	/* pointer to first free buffer in a pool */
	struct xf_buffer	*free;

	/* individual buffer */
	struct xf_buffer	buffer[0];		/* PRQA S 1037 */
};

struct xf_message {
	/* pointer to the next item in the list */
	struct xf_message	*next;

	/* shmem session_id */
	u32			id;

	/* operation code */
	u32			opcode;

	/* length of attached message buffer */
	u32			length;

	/* message buffer */
	void			*buffer;
};

/*******************************************************************************
 * XF_GET_PARAM message
 ******************************************************************************/

/* ...message body (command/response) */
union xf_get_param_msg {			/* PRQA S 750 18 */
	/* ...command structure */
	struct {
		/* ...array of parameters requested */
		u32		id[0];	/* PRQA S 1037 */

	}   __attribute__((__packed__)) c;

	/* ...response structure */
	struct {
		/* ...array of parameters values */
		u32		value[0];/* PRQA S 1037 */

	}   __attribute__((__packed__)) r;

};

/* ...length of the XF_GET_PARAM command/response */
/* PRQA S 3453 2 */
#define XF_GET_PARAM_CMD_LEN(params)	(sizeof(u32) * (params))
#define XF_GET_PARAM_RSP_LEN(params)	(sizeof(u32) * (params))

/*******************************************************************************
 * XF_SET_PARAM message
 ******************************************************************************/

/* ...component initialization parameter */
struct xf_set_param_item {
	/* ...index of parameter passed to SET_CONFIG_PARAM call */
	u32				 id;

	/* ...value of parameter */
	u32				 value;

}   __attribute__ ((__packed__));

/* ...message body (no response message? - tbd) */
struct xf_set_param_msg {
	/* ...command message */
	struct xf_set_param_item	 item[0];	/* PRQA S 1037 */

}   __attribute__ ((__packed__));

/* ...length of the command message */
/* PRQA S 3453 */
#define XF_SET_PARAM_CMD_LEN(params) \
	(sizeof(struct xf_set_param_item) * (params))

/*******************************************************************************
 * XF_ROUTE definition
 ******************************************************************************/

/* ...port routing command */
struct xf_route_port_msg {
	/* ...source port specification */
	u32				 src;

	/* ...destination port specification */
	u32				 dst;

	/* ...number of buffers to allocate */
	u32				 alloc_number;

	/* ...length of buffer to allocate */
	u32				 alloc_size;

	/* ...alignment restriction for a buffer */
	u32				 alloc_align;

}	__attribute__((__packed__));

/*******************************************************************************
 * XF_UNROUTE definition
 ******************************************************************************/

/* ...port unrouting command */
struct xf_unroute_port_msg {
	/* ...source port specification */
	u32				 src;

	/* ...destination port specification */
	u32				 dst;

}	__attribute__((__packed__));

/* ...Capture states  */
enum xa_capture_state {
	XA_CAP_STATE_RUN	= 0,
	XA_CAP_STATE_IDLE	= 1,
	XA_CAP_STATE_PAUSE	= 2,
	XA_CAP_STATE_RESET	= 3
};

/* ...Renderer states  */
enum xa_renderer_state {
	XA_RDR_STATE_RUN	= 0,
	XA_RDR_STATE_IDLE	= 1,
	XA_RDR_STATE_PAUSE	= 2,
	XA_RDR_STATE_RESET	= 3
};

/*******************************************************************************
 * Message routing composition - move somewhere else - tbd
 ******************************************************************************/

/* ...adjust IPC client of message going from user-space */
#define XF_MSG_AP_FROM_USER(id, client) \
	(((id) & ~(0xF << 2)) | ((client) << 2))

/* ...wipe out IPC client from message going to user-space */
#define XF_MSG_AP_TO_USER(id)		   \
	((id) & ~(0xF << 18))

/* ...port specification (12 bits) */
#define __XF_PORT_SPEC(core, id, port)  ((core) | ((id) << 2) | ((port) << 8))
#define __XF_PORT_SPEC2(id, port)	((id) | ((port) << 8))
#define XF_PORT_CORE(spec)		((spec) & 0x3)
#define XF_PORT_CLIENT(spec)		(((spec) >> 2) & 0x3F)
#define XF_PORT_ID(spec)		(((spec) >> 8) & 0xF)

/* ...message id contains source and destination ports specification */
#define __XF_MSG_ID(src, dst)	(((src) & 0xFFFF) | (((dst) & 0xFFFF) << 16))
#define XF_MSG_SRC(id)			(((id) >> 0) & 0xFFFF)
#define XF_MSG_SRC_CORE(id)		(((id) >> 0) & 0x3)
#define XF_MSG_SRC_CLIENT(id)		(((id) >> 2) & 0x3F)
#define XF_MSG_SRC_PORT(id)		(((id) >> 8) & 0xF)
#define XF_MSG_SRC_PROXY(id)		(((id) >> 15) & 0x1)
#define XF_MSG_DST(id)			(((id) >> 16) & 0xFFFF)
#define XF_MSG_DST_CORE(id)		(((id) >> 16) & 0x3)
#define XF_MSG_DST_CLIENT(id)		(((id) >> 18) & 0x3F)
#define XF_MSG_DST_PORT(id)		(((id) >> 24) & 0xF)
#define XF_MSG_DST_PROXY(id)		(((id) >> 31) & 0x1)

/* ...special treatment of AP-proxy destination field */
#define XF_AP_IPC_CLIENT(id)		(((id) >> 18) & 0xF)
#define XF_AP_CLIENT(id)		(((id) >> 22) & 0x1FF)
#define __XF_AP_PROXY(core)		((core) | 0x8000)
#define __XF_DSP_PROXY(core)		((core) | 0x8000)
#define __XF_AP_CLIENT(core, client)	((core) | ((client) << 6) | 0x8000)

/*******************************************************************************
 * Opcode composition
 ******************************************************************************/

/* ...opcode composition with command/response data tags */
#define __XF_OPCODE(c, r, op)	(((c) << 31) | ((r) << 30) | ((op) & 0x3F))

/* ...accessors */
#define XF_OPCODE_CDATA(opcode)		((opcode) & (1 << 31))
#define XF_OPCODE_RDATA(opcode)		((opcode) & (1 << 30))
#define XF_OPCODE_TYPE(opcode)		((opcode) & (0x3F))

/*******************************************************************************
 * Opcode types
 ******************************************************************************/

/* ...unregister client */
#define XF_UNREGISTER			__XF_OPCODE(0, 0, 0)

/* ...register client at proxy */
#define XF_REGISTER			__XF_OPCODE(1, 0, 1)

/* ...port routing command */
#define XF_ROUTE			__XF_OPCODE(1, 0, 2)

/* ...port unrouting command */
#define XF_UNROUTE			__XF_OPCODE(1, 0, 3)

/* ...shared buffer allocation */
#define XF_ALLOC			__XF_OPCODE(0, 0, 4)

/* ...shared buffer freeing */
#define XF_FREE				__XF_OPCODE(0, 0, 5)

/* ...set component parameters */
#define XF_SET_PARAM			__XF_OPCODE(1, 0, 6)

/* ...get component parameters */
#define XF_GET_PARAM			__XF_OPCODE(1, 1, 7)

/* ...input buffer reception */
#define XF_EMPTY_THIS_BUFFER		__XF_OPCODE(1, 0, 8)

/* ...output buffer reception */
#define XF_FILL_THIS_BUFFER		__XF_OPCODE(0, 1, 9)

/* ...flush specific port */
#define XF_FLUSH			__XF_OPCODE(0, 0, 10)

/* ...start component operation */
#define XF_START			__XF_OPCODE(0, 0, 11)

/* ...stop component operation */
#define XF_STOP				__XF_OPCODE(0, 0, 12)

/* ...pause component operation */
#define XF_PAUSE			__XF_OPCODE(0, 0, 13)

/* ...resume component operation */
#define XF_RESUME			__XF_OPCODE(0, 0, 14)

/* ...memory map configuration */
#define XF_MMAP_THIS_BUFFER             __XF_OPCODE(0, 0, 17)

/* ...total amount of supported decoder commands */
#define __XF_OP_NUM			(18)

/*************************************************
 * Renderer - specific configuration parameters
 * **********************************************/

enum xa_config_param_renderer {
	XA_RDR_CONFIG_PARAM_STATE		= 0,
	XA_RDR_CONFIG_PARAM_PCM_WIDTH		= 1,
	XA_RDR_CONFIG_PARAM_CHANNELS		= 2,
	XA_RDR_CONFIG_PARAM_SAMPLE_RATE		= 3,
	XA_RDR_CONFIG_PARAM_FRAME_SIZE		= 4,
	XA_RDR_CONFIG_PARAM_OUTPUT1		= 5,
	XA_RDR_CONFIG_PARAM_DMACHANNEL1		= 6,
	XA_RDR_CONFIG_PARAM_OUTPUT2		= 7,
	XA_RDR_CONFIG_PARAM_DMACHANNEL2		= 8,
	XA_RDR_CONFIG_PARAM_OUT_SAMPLE_RATE	= 9,
	XA_RDR_CONFIG_PARAM_VOLUME_RATE		= 10,
	XA_RDR_CONFIG_PARAM_OUT_CHANNELS	= 11,
	XA_RDR_CONFIG_PARAM_MIX_CONTROL		= 12,
	XA_RDR_CONFIG_PARAM_RING_NUM		= 13,
	XA_RDR_CONFIG_PARAM_NUM			= 14
};

/*************************************************
 * Capture - specific configuration parameters
 * **********************************************/

enum xa_config_param_capture {
	XA_CAP_CONFIG_PARAM_CB			= 0,
	XA_CAP_CONFIG_PARAM_STATE		= 1,
	XA_CAP_CONFIG_PARAM_PCM_WIDTH		= 2,
	XA_CAP_CONFIG_PARAM_CHANNELS		= 3,
	XA_CAP_CONFIG_PARAM_SAMPLE_RATE		= 4,
	XA_CAP_CONFIG_PARAM_FRAME_SIZE		= 5,
	XA_CAP_CONFIG_PARAM_INPUT1		= 6,
	XA_CAP_CONFIG_PARAM_DMACHANNEL1		= 7,
	XA_CAP_CONFIG_PARAM_INPUT2		= 8,
	XA_CAP_CONFIG_PARAM_DMACHANNEL2		= 9,
	XA_CAP_CONFIG_PARAM_OUT_SAMPLE_RATE	= 10,
	XA_CAP_CONFIG_PARAM_VOLUME_RATE		= 11,
	XA_CAP_CONFIG_PARAM_RING_NUM		= 12,
	XA_CAP_CONFIG_PARAM_NUM			= 13
};

/*************************************************
 * Equalizer - specific configuration parameters
 * **********************************************/

enum xa_rel_eqz_filter_type {
	XA_REL_EQZ_TYPE_THROUGH = 0,
	XA_REL_EQZ_TYPE_PEAK	= 1,
	XA_REL_EQZ_TYPE_BASS	= 2,
	XA_REL_EQZ_TYPE_TREBLE	= 3
};

enum xa_rel_eqz_type {
	XA_REL_EQZ_TYPE_PARAMETRIC	= 0,
	XA_REL_EQZ_TYPE_GRAPHIC		= 1
};

/*****************************************************************************/
/* Additional subcommand indices */
/*****************************************************************************/

enum xa_add_cmd_type_generic {
	/* XA_API_CMD_SET_CONFIG_PARAM indices */
	XA_EQZ_CONFIG_PARAM_COEF_FS		= 0x0000,
	XA_EQZ_CONFIG_PARAM_PCM_WIDTH		= 0x0001,
	XA_EQZ_CONFIG_PARAM_CH			= 0x0002,
	XA_EQZ_CONFIG_PARAM_EQZ_TYPE		= 0x0003,

	XA_EQZ_CONFIG_PARAM_FILTER_0_COEF_FC	= 0x0010,
	XA_EQZ_CONFIG_PARAM_FILTER_1_COEF_FC	= 0x0011,
	XA_EQZ_CONFIG_PARAM_FILTER_2_COEF_FC	= 0x0012,
	XA_EQZ_CONFIG_PARAM_FILTER_3_COEF_FC	= 0x0013,
	XA_EQZ_CONFIG_PARAM_FILTER_4_COEF_FC	= 0x0014,
	XA_EQZ_CONFIG_PARAM_FILTER_5_COEF_FC	= 0x0015,
	XA_EQZ_CONFIG_PARAM_FILTER_6_COEF_FC	= 0x0016,
	XA_EQZ_CONFIG_PARAM_FILTER_7_COEF_FC	= 0x0017,
	XA_EQZ_CONFIG_PARAM_FILTER_8_COEF_FC	= 0x0018,

	XA_EQZ_CONFIG_PARAM_FILTER_0_COEF_TYPE	= 0x0020,
	XA_EQZ_CONFIG_PARAM_FILTER_1_COEF_TYPE	= 0x0021,
	XA_EQZ_CONFIG_PARAM_FILTER_2_COEF_TYPE	= 0x0022,
	XA_EQZ_CONFIG_PARAM_FILTER_3_COEF_TYPE	= 0x0023,
	XA_EQZ_CONFIG_PARAM_FILTER_4_COEF_TYPE	= 0x0024,
	XA_EQZ_CONFIG_PARAM_FILTER_5_COEF_TYPE	= 0x0025,
	XA_EQZ_CONFIG_PARAM_FILTER_6_COEF_TYPE	= 0x0026,
	XA_EQZ_CONFIG_PARAM_FILTER_7_COEF_TYPE	= 0x0027,
	XA_EQZ_CONFIG_PARAM_FILTER_8_COEF_TYPE	= 0x0028,

	XA_EQZ_CONFIG_PARAM_FILTER_0_COEF_BW	= 0x0030,
	XA_EQZ_CONFIG_PARAM_FILTER_1_COEF_BW	= 0x0031,
	XA_EQZ_CONFIG_PARAM_FILTER_2_COEF_BW	= 0x0032,
	XA_EQZ_CONFIG_PARAM_FILTER_3_COEF_BW	= 0x0033,
	XA_EQZ_CONFIG_PARAM_FILTER_4_COEF_BW	= 0x0034,
	XA_EQZ_CONFIG_PARAM_FILTER_5_COEF_BW	= 0x0035,
	XA_EQZ_CONFIG_PARAM_FILTER_6_COEF_BW	= 0x0036,
	XA_EQZ_CONFIG_PARAM_FILTER_7_COEF_BW	= 0x0037,
	XA_EQZ_CONFIG_PARAM_FILTER_8_COEF_BW	= 0x0038,

	XA_EQZ_CONFIG_PARAM_FILTER_0_COEF_GA	= 0x0040,
	XA_EQZ_CONFIG_PARAM_FILTER_1_COEF_GA	= 0x0041,
	XA_EQZ_CONFIG_PARAM_FILTER_2_COEF_GA	= 0x0042,
	XA_EQZ_CONFIG_PARAM_FILTER_3_COEF_GA	= 0x0043,
	XA_EQZ_CONFIG_PARAM_FILTER_4_COEF_GA	= 0x0044,
	XA_EQZ_CONFIG_PARAM_FILTER_5_COEF_GA	= 0x0045,
	XA_EQZ_CONFIG_PARAM_FILTER_6_COEF_GA	= 0x0046,
	XA_EQZ_CONFIG_PARAM_FILTER_7_COEF_GA	= 0x0047,
	XA_EQZ_CONFIG_PARAM_FILTER_8_COEF_GA	= 0x0048,

	XA_EQZ_CONFIG_PARAM_FILTER_0_COEF_BA	= 0x0050,
	XA_EQZ_CONFIG_PARAM_FILTER_1_COEF_BA	= 0x0051,
	XA_EQZ_CONFIG_PARAM_FILTER_2_COEF_BA	= 0x0052,
	XA_EQZ_CONFIG_PARAM_FILTER_3_COEF_BA	= 0x0053,
	XA_EQZ_CONFIG_PARAM_FILTER_4_COEF_BA	= 0x0054,
	XA_EQZ_CONFIG_PARAM_FILTER_5_COEF_BA	= 0x0055,
	XA_EQZ_CONFIG_PARAM_FILTER_6_COEF_BA	= 0x0056,
	XA_EQZ_CONFIG_PARAM_FILTER_7_COEF_BA	= 0x0057,
	XA_EQZ_CONFIG_PARAM_FILTER_8_COEF_BA	= 0x0058,

	XA_EQZ_CONFIG_PARAM_BAND_0_GCOEF_GA	= 0x0060,
	XA_EQZ_CONFIG_PARAM_BAND_1_GCOEF_GA	= 0x0061,
	XA_EQZ_CONFIG_PARAM_BAND_2_GCOEF_GA	= 0x0062,
	XA_EQZ_CONFIG_PARAM_BAND_3_GCOEF_GA	= 0x0063,
	XA_EQZ_CONFIG_PARAM_BAND_4_GCOEF_GA	= 0x0064
};

/* ...tdm-renderer-specific configuration parameters */
enum xa_config_param_tdm_renderer {
	XA_TDM_RDR_CONFIG_PARAM_PCM_WIDTH	= 0,
	XA_TDM_RDR_CONFIG_PARAM_CHANNEL_MODE	= 1,
	XA_TDM_RDR_CONFIG_PARAM_IN_SAMPLE_RATE  = 2,
	XA_TDM_RDR_CONFIG_PARAM_FRAME_SIZE	= 3,
	XA_TDM_RDR_CONFIG_PARAM_OUTPUT1		= 4,
	XA_TDM_RDR_CONFIG_PARAM_DMACHANNEL1	= 5,
	XA_TDM_RDR_CONFIG_PARAM_OUTPUT2		= 6,
	XA_TDM_RDR_CONFIG_PARAM_DMACHANNEL2	= 7,
	XA_TDM_RDR_CONFIG_PARAM_OUT_SAMPLE_RATE = 8,
	XA_TDM_RDR_CONFIG_PARAM_VOLUME_RATE	= 9
};

enum xa_rel_tdm_renderer_channel_mode {
	XA_TDM_RDR_CHANNEL_MODE_2X4 = 0, /**< 4 stereo TDM data	   */
	XA_TDM_RDR_CHANNEL_MODE_1X8 = 1, /**< 1 eight-channel TDM data*/
	/**< 1 six-channels plus 1 two-channels TDM data */
	XA_TDM_RDR_CHANNEL_MODE_6_2 = 2,
	XA_TDM_RDR_CHANNEL_MODE_2X3 = 3, /**< 3 stereo TDM data	   */
	XA_TDM_RDR_CHANNEL_MODE_1X6 = 4	 /**< 1 six-channel TDM data  */
};

/* ...TDM Capture-specific configuration parameters */
enum xa_config_param_tdm_capture {
	XA_TDM_CAP_CONFIG_PARAM_PCM_WIDTH	= 0,
	XA_TDM_CAP_CONFIG_PARAM_CHANNEL_MODE	= 1,
	XA_TDM_CAP_CONFIG_PARAM_IN_SAMPLE_RATE  = 2,
	XA_TDM_CAP_CONFIG_PARAM_FRAME_SIZE	= 3,
	XA_TDM_CAP_CONFIG_PARAM_INPUT1		= 4,
	XA_TDM_CAP_CONFIG_PARAM_DMACHANNEL1	= 5,
	XA_TDM_CAP_CONFIG_PARAM_INPUT2		= 6,
	XA_TDM_CAP_CONFIG_PARAM_DMACHANNEL2	= 7,
	XA_TDM_CAP_CONFIG_PARAM_OUT_SAMPLE_RATE = 8,
	XA_TDM_CAP_CONFIG_PARAM_VOLUME_RATE	= 9
};

enum xa_rel_tdm_capture_channel_mode {
	XA_TDM_CAP_CHANNEL_MODE_2X4 = 0, /**< 4 stereo TDM data  */
	XA_TDM_CAP_CHANNEL_MODE_1X8 = 1, /**< 1 eight-channel TDM data*/
	/**< 1 six-channels plus 1 two-channels TDM data */
	XA_TDM_CAP_CHANNEL_MODE_6_2 = 2,
	XA_TDM_CAP_CHANNEL_MODE_2X3 = 3, /**< 3 stereo TDM data	   */
	XA_TDM_CAP_CHANNEL_MODE_1X6 = 4	 /**< 1 six-channel TDM data  */
};

/*****************************************************************************/
/* HW supported	 */
/*****************************************************************************/
/* ...SSI modules supported by HW */
enum ssi_module {
	SSI00		= 0,
	SSI01		= 1,
	SSI02		= 2,
	SSI03		= 3,
	SSI04		= 4,
	SSI05		= 5,
	SSI06		= 6,
	SSI07		= 7,
	SSI10		= 10,
	SSI11		= 11,
	SSI12		= 12,
	SSI13		= 13,
	SSI14		= 14,
	SSI15		= 15,
	SSI16		= 16,
	SSI17		= 17,
	SSI20		= 20,
	SSI21		= 21,
	SSI22		= 22,
	SSI23		= 23,
	SSI24		= 24,
	SSI25		= 25,
	SSI26		= 26,
	SSI27		= 27,
	SSI30		= 30,
	SSI31		= 31,
	SSI32		= 32,
	SSI33		= 33,
	SSI34		= 34,
	SSI35		= 35,
	SSI36		= 36,
	SSI37		= 37,
	SSI40		= 40,
	SSI41		= 41,
	SSI42		= 42,
	SSI43		= 43,
	SSI44		= 44,
	SSI45		= 45,
	SSI46		= 46,
	SSI47		= 47,
	SSI5		= 50,
	SSI6		= 60,
	SSI7		= 70,
	SSI8		= 80,
	SSI90		= 90,
	SSI91		= 91,
	SSI92		= 92,
	SSI93		= 93,
	SSI94		= 94,
	SSI95		= 95,
	SSI96		= 96,
	SSI97		= 97
};

/* ...SRC modules supported by HW */
enum src_module {
	SRC0	= 110,			  /* SRC0 */
	SRC1	= 111,			  /* SRC1 */
	SRC2	= 112,			  /* SRC2 */
	SRC3	= 113,			  /* SRC3 */
	SRC4	= 114,			  /* SRC4 */
	SRC5	= 115,			  /* SRC5 */
	SRC6	= 116,			  /* SRC6 */
	SRC7	= 117,			  /* SRC7 */
	SRC8	= 118,			  /* SRC8 */
	SRC9	= 119,			  /* SRC9 */
	SRCMAX  = 120			   /* Maximum number of SRC modules */
};

/* ...PDMA supported by HW */
enum {
	PDMA_CH00 = 0,
	PDMA_CH01 = 1,
	PDMA_CH02 = 2,
	PDMA_CH03 = 3,
	PDMA_CH04 = 4,
	PDMA_CH05 = 5,
	PDMA_CH06 = 6,
	PDMA_CH07 = 7,
	PDMA_CH08 = 8,
	PDMA_CH09 = 9,
	PDMA_CH10 = 10,
	PDMA_CH11 = 11,
	PDMA_CH12 = 12,
	PDMA_CH13 = 13,
	PDMA_CH14 = 14,
	PDMA_CH15 = 15,
	PDMA_CH16 = 16,
	PDMA_CH17 = 17,
	PDMA_CH18 = 18,
	PDMA_CH19 = 19,
	PDMA_CH20 = 20,
	PDMA_CH21 = 21,
	PDMA_CH22 = 22,
	PDMA_CH23 = 23,
	PDMA_CH24 = 24,
	PDMA_CH25 = 25,
	PDMA_CH26 = 26,
	PDMA_CH27 = 27,
	PDMA_CH28 = 28,
	PDMA_CHMAX = 29
};

/* ...DMAC supported by HW */
enum {
	ADMAC_CH00 = PDMA_CHMAX + 0,
	ADMAC_CH01 = PDMA_CHMAX + 1,
	ADMAC_CH02 = PDMA_CHMAX + 2,
	ADMAC_CH03 = PDMA_CHMAX + 3,
	ADMAC_CH04 = PDMA_CHMAX + 4,
	ADMAC_CH05 = PDMA_CHMAX + 5,
	ADMAC_CH06 = PDMA_CHMAX + 6,
	ADMAC_CH07 = PDMA_CHMAX + 7,
	ADMAC_CH08 = PDMA_CHMAX + 8,
	ADMAC_CH09 = PDMA_CHMAX + 9,
	ADMAC_CH10 = PDMA_CHMAX + 10,
	ADMAC_CH11 = PDMA_CHMAX + 11,
	ADMAC_CH12 = PDMA_CHMAX + 12,
	ADMAC_CH13 = PDMA_CHMAX + 13,
	ADMAC_CH14 = PDMA_CHMAX + 14,
	ADMAC_CH15 = PDMA_CHMAX + 15,
	ADMAC_CH16 = PDMA_CHMAX + 16,
	ADMAC_CH17 = PDMA_CHMAX + 17,
	ADMAC_CH18 = PDMA_CHMAX + 18,
	ADMAC_CH19 = PDMA_CHMAX + 19,
	ADMAC_CH20 = PDMA_CHMAX + 20,
	ADMAC_CH21 = PDMA_CHMAX + 21,
	ADMAC_CH22 = PDMA_CHMAX + 22,
	ADMAC_CH23 = PDMA_CHMAX + 23,
	ADMAC_CH24 = PDMA_CHMAX + 24,
	ADMAC_CH25 = PDMA_CHMAX + 25,
	ADMAC_CH26 = PDMA_CHMAX + 26,
	ADMAC_CH27 = PDMA_CHMAX + 27,
	ADMAC_CH28 = PDMA_CHMAX + 28,
	ADMAC_CH29 = PDMA_CHMAX + 29,
	ADMAC_CH30 = PDMA_CHMAX + 30,
	ADMAC_CH31 = PDMA_CHMAX + 31,
	ADMAC_CHMAX = PDMA_CHMAX + 32
};

#endif
