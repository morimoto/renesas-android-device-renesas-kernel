/** *************************************************************************
 *\file		xf-adsp-base.h
 *\brief	Header file for ADSP Base Control layer
 *\addtogroup	ADSP Driver
 ****************************************************************************
 *\date		Oct. 21, 2017
 *\author	Renesas Electronics Corporation
 ****************************************************************************
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

#ifndef __XF_ADSP_BASE_H
#define __XF_ADSP_BASE_H

#include "xf-adsp-config.h"
#include <adsp_drv/xf-adsp-drv-ext.h>

#define XF_BUF_POOL_SIZE	(4)	 /**< number of buffer in a data pool */

/**< maximum number of DSP component can be registered */
#define MAX_HANDLE		(256)

/* define boolean */
#define TRUE			(1)
#define FALSE			(0)

/** \struct xf_callback_func
 *  \brief  callback function for ADSP's response message
 */
struct xf_callback_func {
	/** callback for empty buffer done message */
	int (*empty_buf_done)(void *data, int opcode, int length, char *buffer);

	/** callback for fill buffer done message */
	int (*fill_buf_done)(void *data, int opcode, int length, char *buffer);

	/** callback for event handler */
	int (*event_handler)(void *data);
};

/** \struct xf_adsp_renderer_params
 *  \brief  parameter structure for Renderer component
 */
struct xf_adsp_renderer_params {
	int	 channel;	/**< channel number		*/
	int	 pcm_width;	/**< PCM width			*/
	int	 frame_size;	/**< frame size			*/
	int	 in_rate;	/**< input sampling rate	*/
	int	 out_rate;	/**< output sampling rate	*/
	int	 vol_rate;	/**< volume rate		*/
	int	 dev1;		/**< 1st device index		*/
	int	 dev2;		/**< 2nd device index		*/
	int	 dma1;		/**< 1st DMA index		*/
	int	 dma2;		/**< 2nd DMA index		*/
	int	 out_channel;	/**< output channels		*/
	int	 mix_ctrl;	/**< mix control flag		*/
	int	 state;		/**< operation state		*/
	int  ring_num;
};

/** \struct xf_adsp_renderer
 *  \brief  Renderer component structure
 */
struct xf_adsp_renderer {
	struct xf_adsp_renderer_params   params; /**< parameter structure*/
	struct xf_pool   *buf_pool;  /**< buffer pool for data transfer  */
	int		 handle_id;  /**< ID of registered handle*/
};

/** \struct xf_adsp_capture_params
 *\brief  parameter structure of Capture component
 */
struct xf_adsp_capture_params {
	int	 channel;	/**< channel number		*/
	int	 pcm_width;	/**< PCM width			*/
	int	 frame_size;	/**< frame size			*/
	int	 in_rate;	/**< input sampling rate	*/
	int	 out_rate;	/**< output sampling rate	*/
	int	 vol_rate;	/**< volume rate		*/
	int	 dev1;		/**< 1st device index		*/
	int	 dev2;		/**< 2nd device index		*/
	int	 dma1;		/**< 1st DMA index		*/
	int	 dma2;		/**< 2nd DMA index		*/
	int	 state;		/**< operation state		*/
	int  ring_num;
};

/** \struct xf_adsp_capture
 *  \brief  Capture component structure
 */
struct xf_adsp_capture {
	struct xf_adsp_capture_params	params;	 /**< parameter structuer*/
	struct xf_pool   *buf_pool;  /**< buffer pool for data transfer  */
	int		 handle_id;  /**< ID of registered handle*/
};

/** \struct xf_equalizer_parametric_coef
 *\brief  Parametric Equalizer type's parameters
 */
struct xf_equalizer_parametric_coef {
	int  type[XA_REL_EQZ_FILTER_NUM];		/**< Filter type */
	int  fc[XA_REL_EQZ_FILTER_NUM];	  /**< Filter center frequency */
	int  gain[XA_REL_EQZ_FILTER_NUM];	/**< Filter gain */
	int  band_width[XA_REL_EQZ_FILTER_NUM];  /**< Filter band width	  */
	int  gain_base[XA_REL_EQZ_FILTER_NUM];   /**< Filter base gain	  */
};

/** \struct xf_equalizer_graphic_coef
 *  \brief  Graphic Equalizer type's parameters
 */
struct xf_equalizer_graphic_coef {
	int  gain_g[XA_REL_EQZ_GRAPHIC_BAND_NUM];/**< Graphic equalizer gain */
};

/** \struct xf_adsp_equalizer_params
 *\brief  Equalizer parameters
 */
struct xf_adsp_equalizer_params {
	int		 channel;	/**< channel number	 */
	int		 pcm_width;	/**< PCM width		 */
	int		 rate;		/**< sampling rate	 */
	int		 eqz_type;	/**< Equalizer type	 */
	struct xf_equalizer_parametric_coef  p_coef; /**< Parametric params */
	struct xf_equalizer_graphic_coef g_coef; /**< Graphic params */
};

/** \struct xf_adsp_equalizer
 *  \brief  Equalizer component's structure
 */
struct xf_adsp_equalizer {
	struct xf_adsp_equalizer_params   params;/**< Equalizer parameters   */
	struct xf_pool	  *buf_pool;  /**< buffer pool for transfer data  */
	int		  handle_id; /**< ID of registered handle	*/
};

/** \struct xf_adsp_tdm_renderer_params
 *  \brief  parameter structure for TDM Renderer component
 */
struct xf_adsp_tdm_renderer_params {
	int		 ch_mode;	/**< channel mode		*/
	int		 pcm_width;	/**< PCM width			*/
	int		 frame_size;	/**< frame size			*/
	int		 in_rate;	/**< input sampling rate	*/
	int		 out_rate;	/**< output sampling rate	*/
	int		 vol_rate;	/**< volume rate		*/
	int		 dev1;		/**< 1st device index		*/
	int		 dev2;		/**< 2nd device index		*/
	int		 dma1;		/**< 1st DMA index		*/
	int		 dma2;		/**< 2nd DMA index		*/
};

/** \struct xf_adsp_tdm_renderer
 *  \brief  TDM Renderer component structure
 */
struct xf_adsp_tdm_renderer {
	struct xf_adsp_tdm_renderer_params   params; /**< parameter structure*/
	struct xf_pool	  *buf_pool;  /**< buffer pool for data transfer  */
	int	 handle_id;  /**< ID of registered handle	*/
};

/** \struct xf_adsp_tdm_capture_params
 *  \brief  parameter structure for TDM Capture component
 */
struct xf_adsp_tdm_capture_params {
	int	 ch_mode;	/**< channel mode		*/
	int	 pcm_width;	/**< PCM width			*/
	int	 frame_size;	/**< frame size			*/
	int	 in_rate;	/**< input sampling rate	*/
	int	 out_rate;	/**< output sampling rate	*/
	int	 vol_rate;	/**< volume rate		*/
	int	 dev1;		/**< 1st device index		*/
	int	 dev2;		/**< 2nd device index		*/
	int	 dma1;		/**< 1st DMA index		*/
	int	 dma2;		/**< 2nd DMA index		*/
};

/** \struct xf_adsp_tdm_capture
 *  \brief  TDM Capture component structure
 */
struct xf_adsp_tdm_capture {
	struct xf_adsp_tdm_capture_params  params; /**< parameter structure*/
	struct xf_pool	   *buf_pool;  /**< buffer pool for data transfer  */
	int	 handle_id;  /**< ID of registered handle*/
};

/** \struct xf_handle
 *  \brief Handle struct for each ADSP component
 */
struct xf_handle {
	int	 comp_id;/**< ADSP component ID	  */
	struct xf_callback_func  *cb;/**< callback functions	 */
	void	*private_data;  /**< private data for callback functions*/
};

/** \struct xf_adsp_base
 *  \brief Base component structure
 */
struct xf_adsp_base {
	struct xf_adsp_base_cmd  cmd; /**< proxy commands	 */
	void	*client; /**< client data which registered to proxy  */
	struct xf_pool	 *aux_pool;  /**< auxiliary buffer pool data	 */
	struct xf_handle *handle[MAX_HANDLE];	/**< handler data   */
	struct task_struct *rsp_thread;/**< thread for response message*/
	wait_queue_head_t base_wait;	 /**< ADSP base's waiting queue	 */
	struct xf_message base_msg;   /**< ADSP base's response message   */
	int base_flag;  /**< flag to control its waiting queue  */
	int err_flag;  /**< flag to indicate a error from plugins */
	int wait_flag;  /**< flag to control the polling waiting*/
	spinlock_t lock; /**< spinlock data */
};

struct xf_pool *xf_adsp_allocate_mem_pool(int pool_size, int buf_length);
int xf_adsp_free_mem_pool(struct xf_pool *pool);
char *xf_adsp_get_data_from_pool(struct xf_pool *pool, int index);

int xf_adsp_empty_this_buffer(int handle_id, char *buffer, int length);
int xf_adsp_fill_this_buffer(int handle_id, char *buffer, int length);
int xf_adsp_mmap_this_buffer(int handle_id, char *buffer, int length);

int xf_adsp_route(int src_handle_id, int dst_handle_id
		 , int buf_cnt, int buf_size);

int xf_adsp_set_param(int handle_id, int index, int value);
int xf_adsp_get_param(int handle_id, int index, int *value);

int xf_adsp_renderer_create(struct xf_adsp_renderer **renderer,
			    struct xf_callback_func *cb, void *private_data);
int xf_adsp_renderer_destroy(struct xf_adsp_renderer *renderer);
int xf_adsp_renderer_set_params(struct xf_adsp_renderer *renderer);
int xf_adsp_renderer_get_params(struct xf_adsp_renderer *renderer);

int xf_adsp_capture_create(struct xf_adsp_capture **capture,
			   struct xf_callback_func *cb, void *private_data);
int xf_adsp_capture_destroy(struct xf_adsp_capture *capture);
int xf_adsp_capture_set_params(struct xf_adsp_capture *capture);
int xf_adsp_capture_get_params(struct xf_adsp_capture *capture);

int xf_adsp_equalizer_create(struct xf_adsp_equalizer **equalizer,
			     struct xf_callback_func *cb, void *private_data);
int xf_adsp_equalizer_destroy(struct xf_adsp_equalizer *equalizer);
int xf_adsp_equalizer_set_params(struct xf_adsp_equalizer *equalizer);
int xf_adsp_equalizer_get_params(struct xf_adsp_equalizer *equalizer);

int xf_adsp_tdm_renderer_create(struct xf_adsp_tdm_renderer **tdm_renderer,
				struct xf_callback_func *cb,
				void *private_data);
int xf_adsp_tdm_renderer_destroy(struct xf_adsp_tdm_renderer *tdm_renderer);
int xf_adsp_tdm_renderer_set_params(struct xf_adsp_tdm_renderer *tdm_renderer);
int xf_adsp_tdm_renderer_get_params(struct xf_adsp_tdm_renderer *tdm_renderer);

int xf_adsp_tdm_capture_create(struct xf_adsp_tdm_capture **tdm_capture,
			       struct xf_callback_func *cb,
			       void *private_data);
int xf_adsp_tdm_capture_destroy(struct xf_adsp_tdm_capture *tdm_capture);
int xf_adsp_tdm_capture_set_params(struct xf_adsp_tdm_capture *tdm_capture);
int xf_adsp_tdm_capture_get_params(struct xf_adsp_tdm_capture *tdm_capture);

#endif
