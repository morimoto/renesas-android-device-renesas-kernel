/** ***************************************************************************
 * \file	 xf-adsp-alsa.c
 * \brief	 Source file for ADSP ALSA Driver
 * \addtogroup	 ADSP Driver
 ******************************************************************************
 * \date	 Oct. 21, 2017
 * \author	 Renesas Electronics Corporation
 ******************************************************************************
 * \par		Copyright
 *
 * Copyright(c) 2016 Renesas Electoronics Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *****************************************************************************/

#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/sched/signal.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/pcm-indirect.h>
#include <linux/time.h>

#include "xf-adsp-base.h"

/* Name of Volume control for playback */
#define PLAYBACK_VOLUME_CTR_NAME	"PlaybackVolume"

/* Name of Volume control for capture */
#define CAPTURE_VOLUME_CTR_NAME		"CaptureVolume"

/* Name of Sample Rate control for playback */
#define PLAYBACK_OUT_RATE_CTR_NAME	"PlaybackOutRate"

/* Name of Output Channel control for playback */
#define PLAYBACK_OUT_CHANNEL_CTR_NAME	"PlaybackOutChannel"

/* Name of Sample Rate control for capture */
#define CAPTURE_IN_RATE_CTR_NAME	"CaptureInRate"

/* Name of Equalizer parameters control for playback */
#define PLAYBACK_EQZ_CTR_NAME		"PlaybackEQZControl"

/* Name of Equalizer parameters control for capture */
#define CAPTURE_EQZ_CTR_NAME		"CaptureEQZControl"

/* Name of Equalizer Switch control for playback */
#define PLAYBACK_EQZ_SWITCH_CTR_NAME	"PlaybackEQZSwitch"

/* Name of Equalizer Switch control for capture */
#define CAPTURE_EQZ_SWITCH_CTR_NAME	"CaptureEQZSwitch"

/* Name of Volume control for TDM playback */
#define TDM_PLAYBACK_VOLUME_CTR_NAME	"TDMPlaybackVolume"

/* Name of Volume control for TDM capture */
#define TDM_CAPTURE_VOLUME_CTR_NAME	"TDMCaptureVolume"

/* Name of Sample Rate control for TDM playback */
#define TDM_PLAYBACK_OUT_RATE_CTR_NAME	"TDMPlaybackOutRate"

/* Name of Sample Rate control for TDM capture */
#define TDM_CAPTURE_IN_RATE_CTR_NAME	"TDMCaptureInRate"

/* Prefix of Playback control name */
#define PREFIX_OF_PLAYBACK_CTR_NAME	PLAYBACK_VOLUME_CTR_NAME[0]

/* Prefix of Capture control name */
#define PREFIX_OF_CAPTURE_CTR_NAME	CAPTURE_VOLUME_CTR_NAME[0]

/* Prefix of TDM control name */
#define PREFIX_OF_TDM_CTR_NAME		TDM_PLAYBACK_VOLUME_CTR_NAME[0]

/* Prefix of TDM playback */
#define TDM_PLAYBACK			TDM_PLAYBACK_VOLUME_CTR_NAME[3]

/* Prefix of TDM record */
#define TDM_CAPTURE			TDM_CAPTURE_VOLUME_CTR_NAME[3]

/* Number of control for playback & capture */
#define RDR_CONTROL_NUM		(9)

/* Number of controls for TDM */
#define TDM_CONTROL_NUM		(4)

/* Indicate playback stream */
#define DIRECT_PLAYBACK		(0)

/* Indicate capture stream */
#define DIRECT_CAPTURE		(1)

/* Indicate stream number */
#define DIRECT_NUM		(2)

/* Supported frame size for playback/record function in driver */
#define FRAME_SIZE		(1024)

/* Supported frame size for TDM playback/record function in driver */
#define TDM_FRAME_SIZE		(1024)

/* Supported sample rate in driver */
#define SND_ADSP_SAMPLE_RATES	(SNDRV_PCM_RATE_32000 | \
				 SNDRV_PCM_RATE_44100 | \
				 SNDRV_PCM_RATE_48000)

/* Supported PCM width in driver */
#define SND_ADSP_PCM_WIDTHS	(SNDRV_PCM_FMTBIT_S16_LE | \
				 SNDRV_PCM_FMTBIT_S24_LE)

/* Macro to control DAI index */
/* DAI 0 index for playback/record functions of stereo/mono formats */
#define RDR_DAI_IDX0		(0)

/* DAI 1 index for playback/record functions of stereo/mono formats */
#define RDR_DAI_IDX1		(1)

/* DAI 2 index for playback/record functions of stereo/mono formats */
#define RDR_DAI_IDX2		(2)

/* DAI 3 index for playback/record functions of stereo/mono formats */
#define RDR_DAI_IDX3		(3)

/* DAI 4 index for playback/record functions of TDM formats */
#define TDM_DAI_IDX		(4)

/* Maximum number of DAI supported by driver */
#define MAX_DAI_IDX		(5)

/* Renderer/Capture software information */
/* Minimum channel number supported */
#define MIN_CHANNEL		(1)

/* Maximum channel number supported */
#define MAX_CHANNEL		(2)

/* Minimum buffer size in byte */
#define MIN_BUF_SIZE		(FRAME_SIZE * MIN_CHANNEL * 2)

/* Maximum buffer size in byte */
#define MAX_BUF_SIZE		(FRAME_SIZE * MAX_CHANNEL * 4)

/* Minimum numbers of period in the buffer */
#define MIN_PERIOD		(1)

/* Maximum numbers of period in the buffer */
#define MAX_PERIOD		(4)

/* Maximun numbers of bytes in ALSA buffer */
#define MAX_BUFFER_BYTES	(MAX_PERIOD * MAX_BUF_SIZE)

/* TDM software information */
/*< Minimum channel number supported in TDM plugin */
#define TDM_MIN_CHANNEL		(6)

/* Maximum channel number supported in TDM plugin */
#define TDM_MAX_CHANNEL		(8)

/* Minimum buffer size in byte for TDM format */
#define TDM_MIN_BUF_SIZE	(TDM_FRAME_SIZE * TDM_MIN_CHANNEL * 2)

/* Maximum buffer size in byte for TDM format */
#define TDM_MAX_BUF_SIZE	(TDM_FRAME_SIZE * TDM_MAX_CHANNEL * 4)

/* Minimum numbers of period in the buffer for TDM format */
#define TDM_MIN_PERIOD		(1)

/* Maximum numbers of period in the buffer for TDM format  */
#define TDM_MAX_PERIOD		(4)

/* Maximum numbers of bytes in ALSA buffer for TDM format */
#define TDM_MAX_BUFFER_BYTES	(TDM_MAX_PERIOD * TDM_MAX_BUF_SIZE)

/* Volume scale used when user set */
#define VOLUME_SCALE		(100)

/* Maximum element in Equalizer parameter control */
#define MAX_EQZ_PARAM_NUMBER	(55)

/* Equalizer control is disabled */
#define EQZ_OFF			(0)

/* Equalizer control is enabled */
#define EQZ_ON			(1)

/* Component status */
/* Handle state is NULL */
#define XF_HANDLE_NULL		(0)

/* Handle state is CREATED after creating handle successfully */
#define XF_HANDLE_CREATED	BIT(0)

/* Handle state is READY after finishing handle init */
#define XF_HANDLE_READY		BIT(1)

/* channels */
/* Mono stream */
#define MONAURAL		(1)

/* Stereo stream */
#define STEREO			(2)

/* define number of bytes in a sample of 24 bits format types */
/* store 24 bits data in 4 bytes LE */
#define FMTBIT_S24_LE_BYTES_PER_SAMPLE		(4)

/* store 24 bits data in 3 bytes LE */
#define FMTBIT_S24_3LE_BYTES_PER_SAMPLE		(3)

/* helper macro to get bytes per sample number */
#define BYTES_PER_SAMPLE(fmt)   (FMTBIT_##fmt##_BYTES_PER_SAMPLE)

/* check component is created */
#define COMPONENT_IS_CREATED(n)	(((n & XF_HANDLE_CREATED) != 0) ? TRUE : FALSE)

/* check component is ready */
#define COMPONENT_IS_READY(n)	(((n & XF_HANDLE_READY) != 0) ? TRUE : FALSE)

/* indicator of stream order */
#define MIX_UNUSED			(0)
#define FIRST_RUN			(2)
#define SECOND_RUN			(1)

/* check MIX usage */
#define MIX_ENABLED(mix_ctl)	((mix_ctl == SECOND_RUN) ? TRUE : FALSE)

/*******************************************************************
 * base structures for ADSP ALSA driver
 * ****************************************************************/

/** \struct snd_adsp_control
 *  \brief  Structure stores parameters from user
 */
struct snd_adsp_control {
	/* Volume rate for playback/record */
	int	   vol_rate[DIRECT_NUM][MAX_DAI_IDX - 1];

	/* Volume rate for TDM playback/TDM record */
	int	   tdm_vol_rate[DIRECT_NUM];

	/* Out sample rate with Renderer, in sample rate with Capture */
	int	   sample_rate[DIRECT_NUM][MAX_DAI_IDX - 1];

	/* Out sample rate for TDM Renderer, in sample rate for TDM Capture */
	int	   tdm_sample_rate[DIRECT_NUM];

	/* Output channel of playback */
	int	   rdr_out_ch[MAX_DAI_IDX - 1];

	/* Equalizer parameters */
	struct xf_adsp_equalizer_params eqz_params[DIRECT_NUM][MAX_DAI_IDX - 1];

	/* Equalizer switch */
	int	   eqz_switch[DIRECT_NUM][MAX_DAI_IDX - 1];

	/* Indicator of MIX usage */
	int	   mix_usage;
};

/** \struct snd_adsp_base_info
 *  \brief  Structure stores some base information of a stream
 */
struct snd_adsp_base_info {
	/* high resolution timer data */
	struct hrtimer			hrtimer;

	/* kernel time value in nanosecond */
	ktime_t				ktime;

	/* save interrupt state before getting lock */
	unsigned long			flag;

	/* high resolution timer state */
	int				hrt_state;

	/* target handle id of ALSA driver */
	int				handle_id;

	/* data buffer */
	char				*buffer[XF_BUF_POOL_SIZE];

	/* size of each allocated data buffer */
	int				buf_bytes;

	/* data index of buffer */
	int				buf_idx;

	/* queue index of buffer */
	int				buf_queue;

	/* HW index in bytes */
	int				hw_idx;

	/* number of bytes in a period */
	int				period_bytes;

	/* substream runtime object */
	struct snd_pcm_substream	*substream;

	/* indirect PCM data transfer */
	struct snd_pcm_indirect		pcm_indirect;

	/* spinlock data */
	spinlock_t			lock;

	/* runtime error indicator */
	int				runtime_err;
};

/** \struct snd_adsp_playback
 *  \brief  Structure stores data for playback function
 */
struct snd_adsp_playback {
	/* base information of stream */
	struct snd_adsp_base_info	base;

	/* Renderer component data */
	struct xf_adsp_renderer		*renderer;

	/* Equalizer component data */
	struct xf_adsp_equalizer	*equalizer;

	/* Renderer component state */
	int				rdr_state;

	/* Equalizer component state */
	int				eqz_state;
};

/** \struct snd_adsp_record
 *  \brief  Structure stores data for record function
 */
struct snd_adsp_record {
	/* base information of stream */
	struct snd_adsp_base_info	base;

	/* Capture component data */
	struct xf_adsp_capture		*capture;

	/* Equalizer component data */
	struct xf_adsp_equalizer	*equalizer;

	/* Capture component state */
	int				cap_state;

	/* Equalizer component state */
	int				eqz_state;
};

/** \struct snd_adsp_tdm_playback
 *  \brief  Structure stores data for TDM playback function
 */
struct snd_adsp_tdm_playback {
	/* base information of stream */
	struct snd_adsp_base_info	base;

	/* TDM Renderer component data */
	struct xf_adsp_tdm_renderer	*tdm_renderer;

	/* TDM Renderer component state */
	int				state;
};

/** \struct snd_adsp_tdm_record
 *  \brief  Structure stores data for TDM record function
 */
struct snd_adsp_tdm_record {
	/* base information of stream */
	struct snd_adsp_base_info	base;

	/* TDM Capture component data */
	struct xf_adsp_tdm_capture	*tdm_capture;

	/* TDM Capture component state */
	int				state;
};

/** \struct snd_adsp_card
 *  \brief  Structure stores data for ALSA sound card
 */
struct snd_adsp_card {
	/* playback data */
	struct snd_adsp_playback	*playback[MAX_DAI_IDX - 1];

	/* record data */
	struct snd_adsp_record		*record[MAX_DAI_IDX - 1];

	/* TDM playback data */
	struct snd_adsp_tdm_playback	*tdm_playback;

	/* TDM record data */
	struct snd_adsp_tdm_record	*tdm_record;

	/* Structure contains params information for control */
	struct snd_adsp_control		ctr_if;
};

/** HW configuration of ALSA ADSP card for Renderer/Capture */
static struct snd_pcm_hardware snd_pcm_adsp_hw = {
	.info		= (SNDRV_PCM_INFO_INTERLEAVED  /* PRQA S 1053 14 */
			 | SNDRV_PCM_INFO_RESUME
			 | SNDRV_PCM_INFO_BLOCK_TRANSFER
			 | SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats	   = SND_ADSP_PCM_WIDTHS,
	.rates		   = SND_ADSP_SAMPLE_RATES,
	.rate_min	   = 32000,
	.rate_max	   = 48000,
	.channels_min	   = MIN_CHANNEL,
	.channels_max	   = MAX_CHANNEL,

	/* maximum buffer size in bytes */
	.buffer_bytes_max  = MAX_BUFFER_BYTES,

	/* minimum size of the periods (frame) in bytes */
	.period_bytes_min  = MIN_BUF_SIZE,

	/* maximum size of the periods (frame) in bytes */
	.period_bytes_max  = MAX_BUF_SIZE,

	/* minimum periods (frames) in a buffer */
	.periods_min	   = MIN_PERIOD,

	/* maximum periods (frames) in a buffer */
	.periods_max	   = MAX_PERIOD,
};

/* HW configuration of ALSA ADSP card for TDM */
static struct snd_pcm_hardware snd_pcm_adsp_tdm_hw = {
	.info		  = (SNDRV_PCM_INFO_INTERLEAVED  /* PRQA S 1053 14 */
			   | SNDRV_PCM_INFO_RESUME
			   | SNDRV_PCM_INFO_BLOCK_TRANSFER
			   | SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats	  = SND_ADSP_PCM_WIDTHS,
	.rates		  = SND_ADSP_SAMPLE_RATES,
	.rate_min	  = 32000,
	.rate_max	  = 48000,
	.channels_min	  = TDM_MIN_CHANNEL,
	.channels_max	  = TDM_MAX_CHANNEL,

	/* maximum buffer size in bytes */
	.buffer_bytes_max = TDM_MAX_BUFFER_BYTES,

	/* minimum size of the periods (frame) in bytes */
	.period_bytes_min = TDM_MIN_BUF_SIZE,

	/* maximum size of the periods (frame) in bytes */
	.period_bytes_max = TDM_MAX_BUF_SIZE,

	/* minimum periods (frames) in a buffer */
	.periods_min	  = TDM_MIN_PERIOD,

	/* maximum periods (frames) in a buffer */
	.periods_max	  = TDM_MAX_PERIOD,
};

/*******************************************************************
 * function declaration
 * ****************************************************************/

static int
snd_adsp_rdr_empty_buf_done(void *data, int opcode, int length, char *buffer);
static int
snd_adsp_rdr_fill_buf_done(void *data, int opcode, int length, char *buffer);
static int
snd_adsp_cap_empty_buf_done(void *data, int opcode, int length, char *buffer);
static int
snd_adsp_cap_fill_buf_done(void *data, int opcode, int length, char *buffer);
static int
snd_adsp_get_dai_id_from_substream(struct snd_pcm_substream *substream);
static void *
snd_adsp_get_drvdata_from_substream(struct snd_pcm_substream *substream);
static struct snd_adsp_base_info *
snd_adsp_get_base_from_substream(struct snd_pcm_substream *substream);
static struct snd_adsp_base_info *
snd_adsp_get_base_from_hrt(struct hrtimer *hrt);
static enum hrtimer_restart snd_adsp_hrtimer_func(struct hrtimer *hrt);
static int snd_adsp_playback_init(struct snd_adsp_playback **data,
				  int eqz_flag,
				  struct snd_pcm_substream *substream);
static int snd_adsp_record_init(struct snd_adsp_record **data,
				int eqz_flag,
				struct snd_pcm_substream *substream);
static int snd_adsp_playback_prepare(struct snd_adsp_playback *playback,
				     struct snd_pcm_substream *substream);
static int snd_adsp_record_prepare(struct snd_adsp_record *record,
				   struct snd_pcm_substream *substream);
static int snd_adsp_playback_deinit(struct snd_adsp_playback *playback);
static int snd_adsp_record_deinit(struct snd_adsp_record *record);
static int snd_adsp_tdm_playback_init(struct snd_adsp_tdm_playback **data,
				      struct snd_pcm_substream *substream);
static int snd_adsp_tdm_record_init(struct snd_adsp_tdm_record **data,
				    struct snd_pcm_substream *substream);
static int
snd_adsp_tdm_playback_prepare(struct snd_adsp_tdm_playback *tdm_playback,
			      struct snd_pcm_substream *substream);
static int snd_adsp_tdm_record_prepare(struct snd_adsp_tdm_record *tdm_record,
				       struct snd_pcm_substream *substream);
static int
snd_adsp_tdm_playback_deinit(struct snd_adsp_tdm_playback *tdm_playback);
static int snd_adsp_tdm_record_deinit(struct snd_adsp_tdm_record *tdm_record);
static int snd_adsp_pcm_open(struct snd_pcm_substream *substream);
static int snd_adsp_pcm_close(struct snd_pcm_substream *substream);
static int snd_adsp_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *hw_params);
static int snd_adsp_pcm_hw_free(struct snd_pcm_substream *substream);
static int snd_adsp_pcm_prepare(struct snd_pcm_substream *substream);
static int snd_adsp_pcm_trigger(struct snd_pcm_substream *substream, int idx);
static snd_pcm_uframes_t
snd_adsp_pcm_pointer(struct snd_pcm_substream *substream);
static int snd_adsp_pcm_ack(struct snd_pcm_substream *substream);
static void snd_adsp_pcm_transfer(struct snd_pcm_substream *substream,
				  struct snd_pcm_indirect *rec, size_t bytes);
static int snd_adsp_control_volume_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo);
static int snd_adsp_control_volume_get(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol);
static int snd_adsp_control_volume_put(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol);
static int snd_adsp_control_eqz_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol);
static int snd_adsp_control_eqz_info(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_info *uinfo);
static int snd_adsp_control_eqz_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol);
static int
snd_adsp_control_eqz_switch_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
static int
snd_adsp_control_eqz_switch_info(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_info *uinfo);
static int
snd_adsp_control_sample_rate_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol);
static int
snd_adsp_control_sample_rate_info(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo);
static int
snd_adsp_control_sample_rate_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol);
static int
snd_adsp_control_eqz_switch_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol);
static void snd_adsp_get_eqz_params_from_control(
				struct xf_adsp_equalizer_params *eqz_params,
				struct xf_adsp_equalizer_params *eqz_ctr_params,
				bool flag);
static int snd_adsp_pcm_new(struct snd_soc_pcm_runtime *runtime);
static int snd_adsp_probe(struct platform_device *pdev);
static int snd_adsp_remove(struct platform_device *pdev);

/*******************************************************************
 * callback function of ADSP control interface
 * ****************************************************************/
/** **************************************************************************
 *  \brief	event handler callback to notify error from ADSP
 *  \internal
 *  \covers: DD_DRV_ALSA_01_004
 *
 *  \param[in]	data		Pointer to ADSP ALSA sound card
 *  \retval	0		Success
 *****************************************************************************/
static int snd_adsp_event_handler(void *data)
{
	struct snd_adsp_base_info *base = (struct snd_adsp_base_info *)data;

	if (base)
		base->runtime_err = TRUE;

	return 0;
}

/** **************************************************************************
 *  \brief	 empty buf done callback for playback/TDM playback functions
 *  \internal
 *  \covers: DD_DRV_ALSA_01_001
 *
 *  \param[in]	  data		Pointer to ADSP ALSA sound card
 *  \param[in]	  opcode	Opcode of message
 *  \param[in]	  length	Length of data buffer
 *  \param[in]	  buffer	Pointer to data buffer
 *  \retval	  0		Success
 *****************************************************************************/
static int
snd_adsp_rdr_empty_buf_done(void *data, int opcode, int length, char *buffer)
{
	struct snd_adsp_base_info *base = (struct snd_adsp_base_info *)data;

	if (base) {
		spin_lock_irqsave(&base->lock, base->flag);

		base->buf_queue++;
		base->hw_idx += length; /* increase the DMA buffer index */

		spin_unlock_irqrestore(&base->lock, base->flag);
	}

	return 0;
}

/** **************************************************************************
 *  \brief	 fill buf done callback for playback/TDM playback functions
 *  \internal
 *  \covers: DD_DRV_ALSA_01_002
 *
 *  \param[in]	  data		Pointer to ADSP ALSA sound card
 *  \param[in]	  opcode	Opcode of message
 *  \param[in]	  length	Length of data buffer
 *  \param[in]	  buffer	Pointer to data buffer
 *  \retval	  0		Success
 *****************************************************************************/
static int
snd_adsp_rdr_fill_buf_done(void *data, int opcode, int length, char *buffer)
{
	return 0;
}

/** **************************************************************************
 *  \brief	 empty buf done callback for record/TDM record functions
 *  \internal
 *  \covers: DD_DRV_ALSA_01_003
 *
 *  \param[in]	  data		Pointer to ADSP ALSA sound card
 *  \param[in]	  opcode	Opcode of message
 *  \param[in]	  length	Length of data buffer
 *  \param[in]	  buffer	Pointer to data buffer
 *  \retval	  0		Success
 *****************************************************************************/
static int
snd_adsp_cap_empty_buf_done(void *data, int opcode, int length, char *buffer)
{
	return 0;
}

/** **************************************************************************
 *  \brief	 fill buf done callback for record/TDM record functions
 *  \internal
 *  \covers: DD_DRV_ALSA_01_051
 *
 *  \param[in]	  data		Pointer to ADSP ALSA sound card
 *  \param[in]	  opcode	Opcode of message
 *  \param[in]	  length	Length of data buffer
 *  \param[in]	  buffer	Pointer to data buffer
 * \retval	  0		Success
 *****************************************************************************/
static int
snd_adsp_cap_fill_buf_done(void *data, int opcode, int length, char *buffer)
{
	struct snd_adsp_base_info *base = (struct snd_adsp_base_info *)data;

	if (base) {
		spin_lock_irqsave(&base->lock, base->flag);

		base->buf_queue++;
		base->hw_idx += length; /* increase the DMA buffer index */

		spin_unlock_irqrestore(&base->lock, base->flag);
	}

	return 0;
}

/** callback functions for playback/TDM playback */
static struct xf_callback_func rdr_callbacks =	/* PRQA S 3218 */
{
	.empty_buf_done = &snd_adsp_rdr_empty_buf_done,	/* PRQA S 1053 2 */
	.fill_buf_done = &snd_adsp_rdr_fill_buf_done,
	.event_handler = &snd_adsp_event_handler
};

/** callback functions for record/TDM record */
static struct xf_callback_func cap_callbacks =	/* PRQA S 3218 */
{
	.empty_buf_done = &snd_adsp_cap_empty_buf_done,	/* PRQA S 1053 2 */
	.fill_buf_done = &snd_adsp_cap_fill_buf_done,
	.event_handler = &snd_adsp_event_handler
};

/*******************************************************************
 * helper functions to get some internal data
 * ****************************************************************/

/** **************************************************************************
 *  \brief	Get current index of CPU DAI from runtime data of substream
 *  \internal
 *  \covers: DD_DRV_ALSA_01_011
 *
 *  \param[in]	  substream	Pointer to PCM stream data
 *  \retval	  id		Index of current CPU DAI
 *****************************************************************************/
static int
snd_adsp_get_dai_id_from_substream(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd;

	rtd = (struct snd_soc_pcm_runtime *)substream->private_data;
	return rtd->cpu_dai->id;
}

/** **************************************************************************
 *  \brief	Get ADSP ALSA driver's data from runtime data of substream
 *  \internal
 *  \covers: DD_DRV_ALSA_01_008
 *
 *  \param[in]	substream	Pointer to PCM stream data
 *  \retval	pointer		Pointer to driver's data
 *****************************************************************************/
static void *
snd_adsp_get_drvdata_from_substream(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd;

	rtd = (struct snd_soc_pcm_runtime *)substream->private_data;
	return snd_soc_dai_get_drvdata(rtd->cpu_dai);
}

/** **************************************************************************
 *  \brief   Get base's data of playback/record from runtime data of substream
 *  \internal
 *  \covers: DD_DRV_ALSA_01_010
 *
 *  \param[in]	substream	Pointer to PCM stream data
 *  \retval	pointer		Pointer to playback/record's base data
 *****************************************************************************/
static struct snd_adsp_base_info *
snd_adsp_get_base_from_substream(struct snd_pcm_substream *substream)
{
	struct snd_adsp_card *adsp_card;
	struct snd_adsp_base_info *base;
	int dai_idx;

	adsp_card = snd_adsp_get_drvdata_from_substream(substream);

	/* get DAI index of substream */
	dai_idx = snd_adsp_get_dai_id_from_substream(substream);

	/* get base data of the substream */
	if (dai_idx == RDR_DAI_IDX0 || dai_idx == RDR_DAI_IDX1 ||
	    dai_idx == RDR_DAI_IDX2 || dai_idx == RDR_DAI_IDX3) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			base = !adsp_card->playback[dai_idx] ?
				NULL : &adsp_card->playback[dai_idx]->base;
		} else {
			base = !adsp_card->record[dai_idx] ?
				NULL : &adsp_card->record[dai_idx]->base;
		}
	} else {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			base = !adsp_card->tdm_playback ?
				NULL : &adsp_card->tdm_playback->base;
		} else {
			base = !adsp_card->tdm_record ?
				NULL : &adsp_card->tdm_record->base;
		}
	}

	return base;
}

/** ***************************************************************************
 *  \brief    Get playback/record/TDM playback/TDM record's base data
 *  \internal
 *  \covers: DD_DRV_ALSA_01_009
 *	      from hr timer data
 *
 *  \param[in]	hrt		Pointer to hr timer data
 *  \retval	pointer		Pointer to playback/record's base data
 *****************************************************************************/
static struct snd_adsp_base_info *
snd_adsp_get_base_from_hrt(struct hrtimer *hrt)
{
	return (struct snd_adsp_base_info *)hrt;
}

/*****************************************************************************
 * hrtimer interrupt function
 * ***************************************************************************/

/** **************************************************************************
 *  \brief	Interrupt function of high resolution timer
 *  \internal
 *  \covers: DD_DRV_ALSA_01_012
 *
 *  \param[in]	hrt		Pointer to hr timer data
 *  \retval	HRTIMER_RESTART Restart the timer after expire time
 *****************************************************************************/
static enum hrtimer_restart snd_adsp_hrtimer_func(struct hrtimer *hrt)
{
	struct snd_adsp_base_info *base = snd_adsp_get_base_from_hrt(hrt);

	spin_lock_irqsave(&base->lock, base->flag);
	if (base->hw_idx != 0) {
		spin_unlock_irqrestore(&base->lock, base->flag);
		/* update PCM status for the next period */
		snd_pcm_period_elapsed(base->substream);
	} else {
		spin_unlock_irqrestore(&base->lock, base->flag);
	}

	hrtimer_forward_now(hrt, base->ktime);

	return HRTIMER_RESTART;
}

/*****************************************************************************
 * internal functions to manage playback and record functions
 * ***************************************************************************/

/** **************************************************************************
 *  \brief	Initialize playback data
 *  \internal
 *  \covers: DD_DRV_ALSA_01_013
 *
 *  \param[out]	  playback_data		Pointer to store playback data
 *  \param[in]	  eqz_flag		Flag to indicate equalizer usage
 *  \param[in]	  substream		Pointer to substream data
 *  \retval	  EINVAL		Failed to initialize playback data
 *  \retval	  0			Success
 *****************************************************************************/
static int snd_adsp_playback_init(struct snd_adsp_playback **playback_data,
				  int eqz_flag,
				  struct snd_pcm_substream *substream)
{
	struct snd_adsp_playback *playback;

	/* allocate memory for playback data */
	playback = kmalloc(sizeof(*playback), GFP_KERNEL);
	if (!playback)
		return -EINVAL;

	/* init params */
	memset(playback, 0, sizeof(struct snd_adsp_playback));

	/* save the playback data */
	*playback_data = playback;

	/* set handle state as NULL state */
	playback->rdr_state = XF_HANDLE_NULL;
	playback->eqz_state = XF_HANDLE_NULL;
	playback->base.hrt_state = XF_HANDLE_NULL;

	/* register renderer component */
	if (xf_adsp_renderer_create(&playback->renderer,
				    &rdr_callbacks,
				    (void *)&playback->base) < 0)
		return -EINVAL;

	/* mark renderer component created */
	playback->rdr_state = XF_HANDLE_CREATED;

	/* set target handle ID as renderer ID */
	playback->base.handle_id = playback->renderer->handle_id;

	if (eqz_flag == EQZ_ON) {
		/* create equalizer component when equalizer is used */
		if (xf_adsp_equalizer_create(&playback->equalizer,
					     &rdr_callbacks,
					     (void *)&playback->base) < 0)
			return -EINVAL;

		/* mark equalizer component created */
		playback->eqz_state = XF_HANDLE_CREATED;

		/* set target handle ID as equalizer ID */
		playback->base.handle_id = playback->equalizer->handle_id;
	}

	/* init lock */
	spin_lock_init(&playback->base.lock);

	/* save the substream data */
	playback->base.substream = substream;

	/* init high resolution timer for updating hw status */
	hrtimer_init(&playback->base.hrtimer,
		     CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);

	/* PRQA S 0563 1 */
	playback->base.hrtimer.function = &snd_adsp_hrtimer_func;
	playback->base.hrt_state = XF_HANDLE_CREATED;

	/* success */
	return 0;
}

/** **************************************************************************
 *  \brief	Initialize record data
 *  \internal
 *  \covers: DD_DRV_ALSA_01_014
 *
 *  \param[out]	 record_data	Pointer to store record data
 *  \param[in]	 eqz_flag	Flag to indicate equalizer usage
 *  \param[in]	 substream	Pointer to substream data
 *  \retval	-EINVAL		Failed to initialize record data
 *  \retval	 0		Success
 *****************************************************************************/
static int snd_adsp_record_init(struct snd_adsp_record **record_data,
				int eqz_flag,
				struct snd_pcm_substream *substream)
{
	struct snd_adsp_record *record;

	/* allocate memory for record data */
	record = kmalloc(sizeof(*record), GFP_KERNEL);
	if (!record)
		return -EINVAL;

	/* init params */
	memset(record, 0, sizeof(struct snd_adsp_record));

	/* save the record data */
	*record_data = record;

	/* set handle state as NULL state */
	record->cap_state = XF_HANDLE_NULL;
	record->eqz_state = XF_HANDLE_NULL;
	record->base.hrt_state = XF_HANDLE_NULL;

	/* register capture component */
	if (xf_adsp_capture_create(&record->capture,
				   &cap_callbacks,
				   (void *)&record->base) < 0)
		return -EINVAL;

	/* mark capture component created */
	record->cap_state = XF_HANDLE_CREATED;

	/* set target handle ID as capture ID */
	record->base.handle_id = record->capture->handle_id;

	/* create equalizer component in case of it being used */
	if (eqz_flag == EQZ_ON) {
		if (xf_adsp_equalizer_create(&record->equalizer,
					     &cap_callbacks,
					     (void *)&record->base) < 0)
			return -EINVAL;

		/* mark equalizer component created */
		record->eqz_state = XF_HANDLE_CREATED;

		/* set target handle ID as equalizer ID */
		record->base.handle_id = record->equalizer->handle_id;
	}

	/* init lock */
	spin_lock_init(&record->base.lock);

	/* save the substream data */
	record->base.substream = substream;

	/* init high resolution timer for updating hw status */
	hrtimer_init(&record->base.hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	/* PRQA S 0563 1 */
	record->base.hrtimer.function = &snd_adsp_hrtimer_func;
	record->base.hrt_state = XF_HANDLE_CREATED;

	/* success */
	return 0;
}

/** **************************************************************************
 *  \brief	Prepare playback function
 *  \internal
 *  \covers: DD_DRV_ALSA_01_017
 *
 *  \param[out]	  playback		Pointer to playback data
 *  \param[in]	  substream		Pointer to substream data
 *  \retval	 -EINVAL		Failed to prepare playback function
 *  \retval	  0			Success
 *****************************************************************************/
static int snd_adsp_playback_prepare(struct snd_adsp_playback *playback,
				     struct snd_pcm_substream *substream)
{
	struct snd_adsp_card *adsp_card;
	int dai_idx, pcm_width, ch, fs, frame_size, vol_rate, hw_buffer_size;
	int out_rate;
	struct snd_adsp_control *ctr_if;
	struct snd_pcm_runtime *runtime;
	struct xf_adsp_renderer *renderer;
	struct xf_adsp_equalizer *equalizer;
	struct snd_adsp_base_info *base;
	int i;

	adsp_card = snd_adsp_get_drvdata_from_substream(substream);
	dai_idx = snd_adsp_get_dai_id_from_substream(substream);
	ctr_if = &adsp_card->ctr_if;
	runtime = substream->runtime;
	renderer = playback->renderer;
	equalizer = playback->equalizer;
	base = &playback->base;

	/* runtime parameter */
	fs = runtime->rate;
	ch = runtime->channels;
	pcm_width = (runtime->format == SNDRV_PCM_FORMAT_S16_LE) ? 16 : 24;
	frame_size = runtime->period_size;
	vol_rate = ctr_if->vol_rate[DIRECT_PLAYBACK][dai_idx];
	out_rate = ctr_if->sample_rate[DIRECT_PLAYBACK][dai_idx];
	hw_buffer_size = snd_pcm_lib_buffer_bytes(substream);

	/* get number of bytes in a period */
	base->period_bytes = snd_pcm_lib_period_bytes(substream);

	if (pcm_width == 16)
		base->buf_bytes = base->period_bytes;
	else
		base->buf_bytes = (base->period_bytes *
			BYTES_PER_SAMPLE(S24_3LE)) / BYTES_PER_SAMPLE(S24_LE);

	/* pcm indirect configuration */
	base->pcm_indirect.hw_buffer_size = hw_buffer_size;
	base->pcm_indirect.sw_buffer_size = base->pcm_indirect.hw_buffer_size;

	/* it should equal to a period size in bytes */
	base->pcm_indirect.hw_queue_size = base->period_bytes;

	/* set parameters when Renderer is not ready */
	if (COMPONENT_IS_READY(playback->rdr_state) == FALSE) {
		struct xf_adsp_renderer_params *params = &renderer->params;

		/* apply renderer parameters */
		params->in_rate = fs;
		params->channel = ch;
		params->pcm_width = pcm_width;
		params->frame_size = frame_size;

		if (ctr_if->mix_usage == SECOND_RUN)
			params->mix_ctrl = ctr_if->mix_usage;
		else
			params->mix_ctrl = MIX_UNUSED;

		/* set flow as ADSP->PDMA0->SRC0->PDMA1->SSI0 */
		params->dev1 = SRC0;
		params->dev2 = SSI00;
		params->dma1 = PDMA_CH00;
		params->dma2 = PDMA_CH01;

		/* when MIX is enabled, change to DMAC transfer type to save */
		/* hw FIFO */
		if (MIX_ENABLED(params->mix_ctrl) == TRUE)
			params->dma1 = ADMAC_CH01;

		/* set volume rate if it is set by user or default value */
		/* is 100% */
		if (vol_rate >= 0)
			params->vol_rate = vol_rate;
		else
			params->vol_rate = (1 << 20);

		/* set output channel if it is set by user */
		if (ctr_if->rdr_out_ch[dai_idx] >= MONAURAL)
			params->out_channel = ctr_if->rdr_out_ch[dai_idx];
		else
			params->out_channel = params->channel;

		/* set sample rate output if it is set by user */
		if (out_rate >= 0)
			params->out_rate = out_rate;

		/* set parameters to ADSP Renderer plugin */
		if (xf_adsp_renderer_set_params(renderer) != 0)
			return -EINVAL;

		/* allocate buffer pool to prepare the execution */
		renderer->buf_pool = xf_adsp_allocate_mem_pool(
				XF_BUF_POOL_SIZE, base->buf_bytes);

		if (IS_ERR(renderer->buf_pool))	/* PRQA S 306*/
			return -EINVAL;

		for (i = 0; i < XF_BUF_POOL_SIZE; i++) {
			base->buffer[i] = xf_adsp_get_data_from_pool(
					renderer->buf_pool, i);

			base->buf_queue++;
			memset(base->buffer[i], 0, base->buf_bytes);
		}

		/* mark Renderer ready */
		playback->rdr_state |= XF_HANDLE_READY;

		/* set parameters for Equalizer if it is used */
		if (COMPONENT_IS_CREATED(playback->eqz_state) == TRUE) {
			/* apply Equalizer parameter setting */
			equalizer->params.channel = ch;
			equalizer->params.pcm_width = pcm_width;
			equalizer->params.rate = fs;

			/* get equalizer parameters from control interface */
			/* data */
			snd_adsp_get_eqz_params_from_control(
				&equalizer->params,
				&ctr_if->eqz_params[DIRECT_PLAYBACK][dai_idx],
				true);

			/* set parameters to Equalizer plugin */
			if (xf_adsp_equalizer_set_params(equalizer) != 0)
				return -EINVAL;

			/* route Equalizer to Renderer */
			if (xf_adsp_route(equalizer->handle_id,
					  renderer->handle_id,
					  XF_BUF_POOL_SIZE,
					  base->buf_bytes) != 0)
				return -EINVAL;

			/* mark Equalizer ready */
			playback->eqz_state |= XF_HANDLE_READY;

			/* prepare data before start PCM */
			/* PRQA S 2462 1 */ /* PRQA S 2463 1 */
			for (i = 0; i < XF_BUF_POOL_SIZE; i++) {
				/* send buffer to plugin to kick */
				/* init-processing */
				if (xf_adsp_empty_this_buffer(
							base->handle_id,
							base->buffer[i],
							base->buf_bytes) != 0)
					return -EINVAL;

				spin_lock_irqsave(&base->lock, base->flag);
				base->buf_queue--;
				spin_unlock_irqrestore(&base->lock, base->flag);
			}
		} else {
			/* send zero buffer to plugin to kick */
			/* init-processing */
			if (xf_adsp_empty_this_buffer(base->handle_id,
						      base->buffer[0],
						      base->buf_bytes) != 0)
				return -EINVAL;

			spin_lock_irqsave(&base->lock, base->flag);
			base->buf_queue--;
			spin_unlock_irqrestore(&base->lock, base->flag);
		}

		/* wait until all the buffer have been consummed */
		while (base->buf_queue != XF_BUF_POOL_SIZE) {
			schedule_timeout_interruptible(2);
			if (signal_pending(current))
				break;

			/* check the error from initialization */
			if (base->runtime_err)
				return -EINVAL;
		}

		/* reset HW index */
		base->hw_idx = 0;
	}

	/* success */
	return 0;
}

/** **************************************************************************
 *  \brief	Prepare record function
 *  \internal
 *  \covers: DD_DRV_ALSA_01_018
 *
 *  \param[out]	  record		Pointer to record data
 *  \param[in]	  substream		Pointer to substream data
 *  \retval	 -EINVAL		Failed to prepare record function
 *  \retval	  0			Success
 *****************************************************************************/
static int snd_adsp_record_prepare(struct snd_adsp_record *record,
				   struct snd_pcm_substream *substream)
{
	struct snd_adsp_card *adsp_card;
	int dai_idx, pcm_width, ch, fs, frame_size, vol_rate, in_rate;
	int hw_buffer_size, hw_queue_size;
	struct snd_adsp_control *ctr_if;
	struct snd_pcm_runtime *runtime;
	struct xf_adsp_capture *capture;
	struct xf_adsp_equalizer *equalizer;
	struct snd_adsp_base_info *base;
	int i;

	adsp_card = snd_adsp_get_drvdata_from_substream(substream);
	dai_idx = snd_adsp_get_dai_id_from_substream(substream);
	ctr_if = &adsp_card->ctr_if;
	runtime = substream->runtime;
	capture = record->capture;
	equalizer = record->equalizer;
	base = &record->base;

	/* runtime parameter */
	fs = runtime->rate;
	ch = runtime->channels;
	pcm_width = (runtime->format == SNDRV_PCM_FORMAT_S16_LE) ? 16 : 24;
	frame_size = runtime->period_size;
	vol_rate = ctr_if->vol_rate[DIRECT_CAPTURE][dai_idx];
	in_rate = ctr_if->sample_rate[DIRECT_CAPTURE][dai_idx];
	hw_buffer_size = snd_pcm_lib_buffer_bytes(substream);

	/* total size of allocated buffers */
	hw_queue_size = base->period_bytes * XF_BUF_POOL_SIZE;

	/* get number of bytes in a period */
	base->period_bytes = snd_pcm_lib_period_bytes(substream);

	if (pcm_width == 16)
		base->buf_bytes = base->period_bytes;
	else
		base->buf_bytes = ((base->period_bytes *
			BYTES_PER_SAMPLE(S24_3LE)) / BYTES_PER_SAMPLE(S24_LE));

	/* pcm indirect configuration */
	base->pcm_indirect.hw_buffer_size = hw_buffer_size;
	base->pcm_indirect.sw_buffer_size = base->pcm_indirect.hw_buffer_size;
	base->pcm_indirect.hw_queue_size = hw_queue_size;

	/* prepare parameters to set to Capture plugin when it is not yet */
	/* ready */
	if (COMPONENT_IS_READY(record->cap_state) == FALSE) {
		struct xf_adsp_capture_params *params = &capture->params;

		/* apply capture parameters */
		params->out_rate = fs;
		params->channel = ch;
		params->pcm_width = pcm_width;
		params->frame_size = frame_size;

		params->dev1 = SRC0;
		params->dev2 = SSI10;
		params->dma1 = PDMA_CH00;
		params->dma2 = PDMA_CH01;

		/* set volume rate if it is set by user or default volume as */
		/* 100% */
		if (vol_rate >= 0)
			params->vol_rate = vol_rate;
		else
			params->vol_rate = (1 << 20);

		/* set sample rate input if it is set by user */
		if (in_rate >= 0)
			params->in_rate = in_rate;

		/* allocate buffer pool to prepare the execution */
		capture->buf_pool = xf_adsp_allocate_mem_pool(
				XF_BUF_POOL_SIZE, base->buf_bytes);

		if (IS_ERR(capture->buf_pool))		 /* PRQA S 306 */
			return -EINVAL;

		for (i = 0; i < XF_BUF_POOL_SIZE; i++) {
			base->buffer[i] = xf_adsp_get_data_from_pool(
					capture->buf_pool, i);

			base->buf_queue++;
		}

		/* set parameters to ADSP Capture plugin */
		if (xf_adsp_capture_set_params(capture) != 0)
			return -EINVAL;

		/* mark Capture ready */
		record->cap_state |= XF_HANDLE_READY;

		/* set parameters for Equalizer if it's used */
		if (COMPONENT_IS_CREATED(record->eqz_state) == TRUE) {
			/* apply Equalizer parameter setting */
			equalizer->params.channel = ch;
			equalizer->params.pcm_width = pcm_width;
			equalizer->params.rate = fs;

			/* get equalizer parameter from control interface */
			/* data */
			snd_adsp_get_eqz_params_from_control(
				&equalizer->params,
				&ctr_if->eqz_params[DIRECT_CAPTURE][dai_idx],
				true);

			/* set parameters to Equalizer plugin */
			if (xf_adsp_equalizer_set_params(equalizer) != 0)
				return -EINVAL;

			/* route Capture to Equalizer */
			if (xf_adsp_route(capture->handle_id,
					  equalizer->handle_id,
					  XF_BUF_POOL_SIZE,
					  base->buf_bytes) != 0)
				return -EINVAL;

			/* mark Equalizer ready */
			record->eqz_state |= XF_HANDLE_READY;
		}

		/* kick init process by sending a zero buffer length */
		xf_adsp_fill_this_buffer(base->handle_id, base->buffer[0], 0);

		spin_lock_irqsave(&base->lock, base->flag);
		base->buf_queue--;
		spin_unlock_irqrestore(&base->lock, base->flag);

		/* wait until finishing initialization */
		while (base->buf_queue != XF_BUF_POOL_SIZE) {
			schedule_timeout_interruptible(2);
			if (signal_pending(current))
				break;

			/* check the error from initialization */
			if (base->runtime_err)
				return -EINVAL;
		}

		if (COMPONENT_IS_CREATED(record->eqz_state) == TRUE) {
			/* PRQA S 2462 1 */ /* PRQA S 2463 1 */
			for (i = 0; i < XF_BUF_POOL_SIZE; i++) {
				/* send buffer to plugin */
				if (xf_adsp_fill_this_buffer(
						base->handle_id,
						base->buffer[i],
						base->buf_bytes) != 0)
					return -EINVAL;

				spin_lock_irqsave(&base->lock, base->flag);
				base->buf_queue--;
				spin_unlock_irqrestore(&base->lock, base->flag);
			}

			/* wait until all the buffer have been responsed */
			while (base->buf_queue != XF_BUF_POOL_SIZE) {
				schedule_timeout_interruptible(2);
				if (signal_pending(current))
					break;

				/* check the error from initialization */
				if (base->runtime_err)
					return -EINVAL;
			}
		}

		/* reset hw data position */
		base->hw_idx = 0;
	}

	/* success */
	return 0;
}

/** **************************************************************************
 *  \brief	Deinitialize playback function
 *  \internal
 *  \covers: DD_DRV_ALSA_01_021
 *
 *  \param[out]	playback	Pointer to playback data
 *  \retval	-EINVAL		Failed to deinitialize playback function
 *  \retval	0		Success
 *****************************************************************************/
static int snd_adsp_playback_deinit(struct snd_adsp_playback *playback)
{
	int ret = 0;

	/* perform de-initialization if playback has been created already */
	if (playback) {
		/* perform completion process */
		if (COMPONENT_IS_CREATED(playback->rdr_state) == TRUE) {
			/* send buffer with zero length to plugin for */
			/* completion process *//* PRQA S 3200 2 */
			xf_adsp_empty_this_buffer(playback->base.handle_id,
						  NULL, 0);

			/* free buffer pool */ /* PRQA S 3200 1 */
			xf_adsp_free_mem_pool(playback->renderer->buf_pool);

			/* destroy Renderer */
			if (xf_adsp_renderer_destroy(playback->renderer) != 0)
				ret = -EINVAL;

			playback->renderer = NULL;
		}

		/* destroy Equalizer if it is used */
		if (COMPONENT_IS_CREATED(playback->eqz_state) == TRUE) {
			if (xf_adsp_equalizer_destroy(playback->equalizer))
				ret = -EINVAL;

			playback->equalizer = NULL;
		}

		/* canncel timer interrupt */
		if (COMPONENT_IS_CREATED(playback->base.hrt_state) == TRUE)
			hrtimer_cancel(&playback->base.hrtimer);

		/* free playback data */
		kfree(playback);
	}

	return ret;
}

/** **************************************************************************
 *  \brief	Deinitialize record function
 *  \internal
 *  \covers: DD_DRV_ALSA_01_022
 *
 *  \param[out]	 record		Pointer to record data
 *  \retval	 -EINVAL	Failed to deinitialize record function
 *  \retval	 0		Success
 *****************************************************************************/
static int snd_adsp_record_deinit(struct snd_adsp_record *record)
{
	int ret = 0;

	/* perform de-initialization if record has been created already */
	if (!record)
		return ret;

	/* perform completion process */
	if (COMPONENT_IS_CREATED(record->cap_state) == TRUE) {
		/* send buffer with zero length to plugin for */
		/* completion process *//* PRQA S 3200 2 */
		xf_adsp_empty_this_buffer(record->base.handle_id, NULL, 0);

		/* free buffer pool */ /* PRQA S 3200 1 */
		xf_adsp_free_mem_pool(record->capture->buf_pool);

		if (xf_adsp_capture_destroy(record->capture))
			ret = -EINVAL;

		record->capture = NULL;
	}

	/* destroy Equalizer if it is used */
	if (COMPONENT_IS_CREATED(record->eqz_state) == TRUE) {
		if (xf_adsp_equalizer_destroy(record->equalizer) != 0)
			ret = -EINVAL;

		record->equalizer = NULL;
	}

	/* canncel timer interrupt */
	if (COMPONENT_IS_CREATED(record->base.hrt_state) == TRUE)
		hrtimer_cancel(&record->base.hrtimer);

	/* free record data */
	kfree(record);

	return ret;
}

/*****************************************************************************
 * internal functions to manage TDM playback and TDM record functions
 * ***************************************************************************/

/** **************************************************************************
 *  \brief	Initialize TDM playback data
 *  \internal
 *  \covers: DD_DRV_ALSA_01_015
 *
 *  \param[out]	tdm_playback_data	Pointer to store TDM playback data
 *  \param[in]	  substream		Pointer to substream data
 *  \retval	  -EINVAL		Failed to initialize TDM playback data
 *  \retval	  0			Success
 *****************************************************************************/
static int
snd_adsp_tdm_playback_init(struct snd_adsp_tdm_playback **tdm_playback_data,
			   struct snd_pcm_substream *substream)
{
	struct snd_adsp_tdm_playback *tdm_playback;

	/* allocate memory for TDM playback data */
	tdm_playback = kmalloc(sizeof(*tdm_playback), GFP_KERNEL);

	if (!tdm_playback)
		return -EINVAL;

	/* init params */
	memset(tdm_playback, 0, sizeof(struct snd_adsp_tdm_playback));

	/* save the TDM playback data */
	*tdm_playback_data = tdm_playback;

	/* set handle state as NULL state */
	tdm_playback->state = XF_HANDLE_NULL;
	tdm_playback->base.hrt_state = XF_HANDLE_NULL;

	/* register TDM renderer component */
	if (xf_adsp_tdm_renderer_create(&tdm_playback->tdm_renderer,
					&rdr_callbacks,
					(void *)&tdm_playback->base) < 0)
		return -EINVAL;

	/* mark TDM renderer component created */
	tdm_playback->state = XF_HANDLE_CREATED;

	/* set target handle ID as TDM renderer ID */
	tdm_playback->base.handle_id = tdm_playback->tdm_renderer->handle_id;

	/* init lock */
	spin_lock_init(&tdm_playback->base.lock);

	/* save the substream data */
	tdm_playback->base.substream = substream;

	/* init high resolution timer for updating hw status */
	hrtimer_init(&tdm_playback->base.hrtimer,
		     CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);

	/* PRQA S 0563 1 */
	tdm_playback->base.hrtimer.function = &snd_adsp_hrtimer_func;
	tdm_playback->base.hrt_state = XF_HANDLE_CREATED;

	return 0;
}

/** **************************************************************************
 *  \brief	Initialize TDM record data
 *  \internal
 *  \covers: DD_DRV_ALSA_01_016
 *
 *  \param[out]	  tdm_record_data	Pointer to store TDM record data
 *  \param[in]	  substream		Pointer to substream data
 *  \retval	  -EINVAL		Failed to initialize TDM record data
 *  \retval	  0			Success
 *****************************************************************************/
static int
snd_adsp_tdm_record_init(struct snd_adsp_tdm_record **tdm_record_data,
			 struct snd_pcm_substream *substream)
{
	struct snd_adsp_tdm_record *tdm_record;

	/* allocate memory for TDM record data */
	tdm_record = kmalloc(sizeof(*tdm_record), GFP_KERNEL);
	if (!tdm_record)
		return -EINVAL;

	/* init params */
	memset(tdm_record, 0, sizeof(struct snd_adsp_tdm_record));

	/* save the TDM record data */
	*tdm_record_data = tdm_record;

	/* set handle state as NULL state */
	tdm_record->state = XF_HANDLE_NULL;
	tdm_record->base.hrt_state = XF_HANDLE_NULL;

	/* register TDM Capture component */
	if (xf_adsp_tdm_capture_create(&tdm_record->tdm_capture,
				       &cap_callbacks,
				       (void *)&tdm_record->base) != 0)
		return -EINVAL;

	/* mark TDM capture component created */
	tdm_record->state = XF_HANDLE_CREATED;

	/* set target handle ID as TDM capture ID */
	tdm_record->base.handle_id = tdm_record->tdm_capture->handle_id;

	/* init lock */
	spin_lock_init(&tdm_record->base.lock);

	/* save the substream data */
	tdm_record->base.substream = substream;

	/* init high resolution timer for updating hw status */
	hrtimer_init(&tdm_record->base.hrtimer,
		     CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);

	/* PRQA S 0563 1 */
	tdm_record->base.hrtimer.function = &snd_adsp_hrtimer_func;
	tdm_record->base.hrt_state = XF_HANDLE_CREATED;

	/* success */
	return 0;
}

/** **************************************************************************
 *  \brief	Prepare TDM playback function
 *  \internal
 *  \covers: DD_DRV_ALSA_01_019
 *
 *  \param[out]	  tdm_playback		Pointer to TDM playback data
 *  \param[in]	  substream		Pointer to substream data
 *  \retval	  -EINVAL		Failed to prepare TDM playback function
 *  \retval	  0			Success
 *****************************************************************************/
static int
snd_adsp_tdm_playback_prepare(struct snd_adsp_tdm_playback *tdm_playback,
			      struct snd_pcm_substream *substream)
{
	struct snd_adsp_card *adsp_card;
	struct snd_adsp_control *ctr_if;
	struct snd_pcm_runtime *runtime;
	struct xf_adsp_tdm_renderer *tdm_renderer;
	struct snd_adsp_base_info *base;
	int i;
	int pcm_width, ch_mode, fs, frame_size, hw_buffer_size;

	adsp_card = snd_adsp_get_drvdata_from_substream(substream);
	ctr_if = &adsp_card->ctr_if;
	runtime = substream->runtime;
	tdm_renderer = tdm_playback->tdm_renderer;
	base = &tdm_playback->base;

	/* runtime parameter */
	fs = runtime->rate;
	pcm_width = (runtime->format == SNDRV_PCM_FORMAT_S16_LE) ? 16 : 24;
	frame_size = runtime->period_size;

	ch_mode = (runtime->channels == 8) ?
		XA_TDM_RDR_CHANNEL_MODE_1X8 : XA_TDM_RDR_CHANNEL_MODE_1X6;

	hw_buffer_size = snd_pcm_lib_buffer_bytes(substream);

	/* get number of bytes in a period */
	base->period_bytes = snd_pcm_lib_period_bytes(substream);

	if (pcm_width == 16)
		base->buf_bytes = base->period_bytes;
	else
		base->buf_bytes = ((base->period_bytes *
			BYTES_PER_SAMPLE(S24_3LE)) / BYTES_PER_SAMPLE(S24_LE));

	/* pcm indirect configuration */
	base->pcm_indirect.hw_buffer_size = hw_buffer_size;
	base->pcm_indirect.sw_buffer_size = base->pcm_indirect.hw_buffer_size;

	/* it should equal to a period size in bytes */
	base->pcm_indirect.hw_queue_size = base->period_bytes;

	/* prepare parameters to set to TDM playback as it is not ready */
	if (COMPONENT_IS_READY(tdm_playback->state) == FALSE) {
		struct xf_adsp_tdm_renderer_params *params;

		params = &tdm_renderer->params;

		/* apply renderer parameters */
		params->in_rate = fs;
		params->ch_mode = ch_mode;
		params->pcm_width = pcm_width;
		params->frame_size = frame_size;

		/* setting Audio device indexes */
		params->dma1 = ADMAC_CH00; /* use DMAC for transfer data */
		params->dma2 = PDMA_CH03;
		params->dev1 = SRC1;
		params->dev2 = SSI30;  /* set SSI index to SSI30 */

		/* set volume rate if it is set by user or default value as */
		/* 100% */
		params->vol_rate = (ctr_if->tdm_vol_rate[DIRECT_PLAYBACK] >= 0)
			? ctr_if->tdm_vol_rate[DIRECT_PLAYBACK] : (1 << 20);

		/* set output sampling rate if it is set by user */
		if (ctr_if->tdm_sample_rate[DIRECT_PLAYBACK] >= 0)
			params->out_rate =
				ctr_if->tdm_sample_rate[DIRECT_PLAYBACK];

		/* set parameters to ADSP TDM Renderer plugin */
		if (xf_adsp_tdm_renderer_set_params(tdm_renderer) != 0)
			return -EINVAL;

		/* allocate buffer pool to prepare the execution */
		tdm_renderer->buf_pool = xf_adsp_allocate_mem_pool(
				XF_BUF_POOL_SIZE, base->buf_bytes);

		if (IS_ERR(tdm_renderer->buf_pool))	/* PRQA S 306*/
			return -EINVAL;

		for (i = 0; i < XF_BUF_POOL_SIZE; i++) {
			base->buffer[i] = xf_adsp_get_data_from_pool(
					tdm_renderer->buf_pool, i);

			base->buf_queue++;
		}

		/* mark TDM Renderer created */
		tdm_playback->state |= XF_HANDLE_READY;

		/* send zero buffer to plugin to kick init-processing */
		memset(base->buffer[0], 0, base->buf_bytes);

		if (xf_adsp_empty_this_buffer(base->handle_id, base->buffer[0],
					      base->buf_bytes) != 0)
			return -EINVAL;

		spin_lock_irqsave(&base->lock, base->flag);
		base->buf_queue--;
		spin_unlock_irqrestore(&base->lock, base->flag);

		/* wait until all the buffer have been consummed */
		while (base->buf_queue != XF_BUF_POOL_SIZE) {
			schedule_timeout_interruptible(2);
			if (signal_pending(current))
				break;

			/* check the error from initialization */
			if (base->runtime_err)
				return -EINVAL;
		}

		/* reset HW index */
		base->hw_idx = 0;
	}

	/* success */
	return 0;
}

/** **************************************************************************
 *  \brief	Prepare TDM record function
 *  \internal
 *  \covers: DD_DRV_ALSA_01_020
 *
 *  \param[out]	  tdm_record		Pointer to TDM record data
 *  \param[in]	  substream		Pointer to substream data
 *  \retval	  -EINVAL		Failed to prepare TDM record function
 *  \retval	  0			Success
 *****************************************************************************/
static int snd_adsp_tdm_record_prepare(struct snd_adsp_tdm_record *tdm_record,
				       struct snd_pcm_substream *substream)
{
	struct snd_adsp_card *adsp_card =
		snd_adsp_get_drvdata_from_substream(substream);
	struct snd_adsp_control *ctr_if = &adsp_card->ctr_if;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct xf_adsp_tdm_capture *tdm_capture = tdm_record->tdm_capture;
	struct snd_adsp_base_info *base = &tdm_record->base;
	int i;
	int pcm_width, ch_mode, fs, frame_size;

	/* runtime parameter */
	fs = runtime->rate;

	ch_mode = (runtime->channels == 8) ?
		XA_TDM_RDR_CHANNEL_MODE_1X8 : XA_TDM_RDR_CHANNEL_MODE_1X6;

	pcm_width = (runtime->format == SNDRV_PCM_FORMAT_S16_LE) ? 16 : 24;
	frame_size = runtime->period_size;

	/* get number of bytes in a period */
	base->period_bytes = snd_pcm_lib_period_bytes(substream);
	base->buf_bytes = (pcm_width == 16) ?
			   base->period_bytes : ((base->period_bytes *
						BYTES_PER_SAMPLE(S24_3LE)) /
						BYTES_PER_SAMPLE(S24_LE));

	/* pcm indirect configuration */
	base->pcm_indirect.hw_buffer_size = snd_pcm_lib_buffer_bytes(substream);
	base->pcm_indirect.sw_buffer_size = base->pcm_indirect.hw_buffer_size;

	/* total size of allocated buffers */
	base->pcm_indirect.hw_queue_size = base->period_bytes *
					   XF_BUF_POOL_SIZE;

	/* prepare parameters to set to TDM Capture as it is not yet ready */
	if (COMPONENT_IS_READY(tdm_record->state) == FALSE) {
		struct xf_adsp_tdm_capture_params *params;

		params = &tdm_capture->params;

		/* apply capture parameters */
		params->out_rate = fs;
		params->ch_mode = ch_mode;
		params->pcm_width = pcm_width;
		params->frame_size = frame_size;

		/* setting Audio device indexes */
		params->dma1 = PDMA_CH00;
		params->dma2 = PDMA_CH01;
		params->dev1 = SRC0;
		params->dev2 = SSI40;  /* set input device is SSI40 */

		/* set volume rate if it is set by user or default value as */
		/* 100% */
		params->vol_rate = (ctr_if->tdm_vol_rate[DIRECT_CAPTURE] >= 0)
			? ctr_if->tdm_vol_rate[DIRECT_CAPTURE] : (1 << 20);

		/* set input rate if it is set by user */
		if (ctr_if->tdm_sample_rate[DIRECT_CAPTURE] >= 0) {
			params->in_rate =
				ctr_if->tdm_sample_rate[DIRECT_CAPTURE];
		}

		/* allocate buffer pool to prepare the execution */
		tdm_capture->buf_pool = xf_adsp_allocate_mem_pool(
				XF_BUF_POOL_SIZE, base->buf_bytes);

		if (IS_ERR(tdm_capture->buf_pool))	/* PRQA S 306 */
			return -EINVAL;

		for (i = 0; i < XF_BUF_POOL_SIZE; i++) {
			base->buffer[i] = xf_adsp_get_data_from_pool(
					tdm_capture->buf_pool, i);

			base->buf_queue++;
		}

		/* set parameters to ADSP TDM Capture plugin */
		if (xf_adsp_tdm_capture_set_params(tdm_capture) != 0)
			return -EINVAL;

		/* mark TDM Capture ready */
		tdm_record->state |= XF_HANDLE_READY;

		/* kick init process by sending a zero buffer length */
		xf_adsp_fill_this_buffer(base->handle_id, base->buffer[0], 0);

		spin_lock_irqsave(&base->lock, base->flag);
		base->buf_queue--;
		spin_unlock_irqrestore(&base->lock, base->flag);

		/* wait until finishing initialization */
		while (base->buf_queue != XF_BUF_POOL_SIZE) {
			schedule_timeout_interruptible(2);
			if (signal_pending(current))
				break;

			/* check the error from initialization */
			if (base->runtime_err)
				return -EINVAL;
		}

		/* reset hw data position */
		base->hw_idx = 0;
	}

	/* success */
	return 0;
}

/** **************************************************************************
 *  \brief	Deinitialize TDM playback function
 *  \internal
 *  \covers: DD_DRV_ALSA_01_023
 *
 *  \param[out]	  tdm_playback	Pointer to TDM playback data
 *  \retval	  -EINVAL	Failed to deinitialize TDM playback function
 *  \retval	  0		Success
 *****************************************************************************/
static int
snd_adsp_tdm_playback_deinit(struct snd_adsp_tdm_playback *tdm_playback)
{
	int ret = 0;

	/* perform de-initialization if TDM playback has been created */
	if (!tdm_playback)
		return ret;

	/* perform completion process */
	if (COMPONENT_IS_CREATED(tdm_playback->state) == TRUE) {
		/* send buffer with zero length to plugin for */
		/* completion process */
		/* PRQA S 3200 2 */
		xf_adsp_empty_this_buffer(tdm_playback->base.handle_id,
					  NULL, 0);

		/* free buffer pool *//* PRQA S 3200 2 */
		xf_adsp_free_mem_pool(tdm_playback->tdm_renderer->buf_pool);

		/* destroy TDM Renderer component */
		if (xf_adsp_tdm_renderer_destroy(
				tdm_playback->tdm_renderer) != 0)
			ret = -EINVAL;

		tdm_playback->tdm_renderer = NULL;
	}

	/* canncel timer interrupt */
	if (COMPONENT_IS_CREATED(tdm_playback->base.hrt_state) == TRUE)
		hrtimer_cancel(&tdm_playback->base.hrtimer);

	/* free playback data */
	kfree(tdm_playback);

	return ret;
}

/** **************************************************************************
 *  \brief	Deinitialize TDM record function
 *  \internal
 *  \covers: DD_DRV_ALSA_01_024
 *
 *  \param[out]	 tdm_record	Pointer to TDM record data
 *  \retval	 -EINVAL	Failed to deinitialize TDM record function
 *  \retval	 0		Success
 *****************************************************************************/
static int snd_adsp_tdm_record_deinit(struct snd_adsp_tdm_record *tdm_record)
{
	int ret = 0;

	/* perform de-initialization if TDM record has been created already */
	if (!tdm_record)
		return ret;

	/* perform completion process */
	if (COMPONENT_IS_CREATED(tdm_record->state) == TRUE) {
		/* send buffer with zero length to plugin for */
		/* completion process */
		/* PRQA S 3200 2 */
		xf_adsp_empty_this_buffer(tdm_record->base.handle_id, NULL, 0);

		/* free buffer pool *//* PRQA S 3200 2 */
		xf_adsp_free_mem_pool(tdm_record->tdm_capture->buf_pool);

		/* destroy TDM Capture component */
		if (xf_adsp_tdm_capture_destroy(tdm_record->tdm_capture) != 0)
			ret = -EINVAL;

		tdm_record->tdm_capture = NULL;
	}

	/* canncel timer interrupt */
	if (COMPONENT_IS_CREATED(tdm_record->base.hrt_state) == TRUE)
		hrtimer_cancel(&tdm_record->base.hrtimer);

	/* free record data */
	kfree(tdm_record);

	return ret;
}

/*****************************************************************************
 * callback functions of ADSP ALSA driver
 * ***************************************************************************/

/** **************************************************************************
 *  \brief	 Open a playback/TDM playback or record/TDM record stream
 *  \internal
 *  \covers: DD_DRV_ALSA_01_028
 *
 *  \param[in]	substream		Pointer to substream object
 *  \retval	0			Success
 *  \retval	-EINVAL			Error
 *****************************************************************************/
static int snd_adsp_pcm_open(struct snd_pcm_substream *substream)
{
	/* get ADSP soundcard and CPU DAI index */
	struct snd_adsp_card *adsp_card;
	int dai_idx = snd_adsp_get_dai_id_from_substream(substream);
	struct snd_adsp_control *ctr_if;

	adsp_card = snd_adsp_get_drvdata_from_substream(substream);
	ctr_if = &adsp_card->ctr_if;

	if (dai_idx == RDR_DAI_IDX0 || dai_idx == RDR_DAI_IDX1 ||
	    dai_idx == RDR_DAI_IDX2 || dai_idx == RDR_DAI_IDX3) {
		/* register data for playback/record functions */
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			/* perform playback initialization */
			if (snd_adsp_playback_init(
				   &adsp_card->playback[dai_idx],
				   ctr_if->eqz_switch[DIRECT_PLAYBACK][dai_idx],
				   substream) < 0) {
				/* perform playback de-initialization when */
				/* the initialization fails */
				snd_adsp_playback_deinit(
						adsp_card->playback[dai_idx]);

				adsp_card->playback[dai_idx] = NULL;
				return -EINVAL;
			}
		} else {
			/* perform record initialization */
			if (snd_adsp_record_init(
				    &adsp_card->record[dai_idx],
				    ctr_if->eqz_switch[DIRECT_CAPTURE][dai_idx],
				    substream) < 0) {
				/* perform record de-initialization when the */
				/* initialization fails */
				snd_adsp_record_deinit(
						adsp_card->record[dai_idx]);

				adsp_card->record[dai_idx] = NULL;
				return -EINVAL;
			}
		}

		/* save the hardware parameters */
		snd_soc_set_runtime_hwparams(substream, &snd_pcm_adsp_hw);

		/* each period has a frame size */
		snd_pcm_hw_constraint_single(substream->runtime,
					     SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
					     FRAME_SIZE);
	} else {
		/* register data for TDM playback/record functions */
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			/* perform TDM playback initialization */
			if (snd_adsp_tdm_playback_init(
						&adsp_card->tdm_playback,
						substream) < 0) {
				/* perform TDM playback de-initialization */
				/* when the initialization fails */
				snd_adsp_tdm_playback_deinit(
						adsp_card->tdm_playback);

				adsp_card->tdm_playback = NULL;
				return -EINVAL;
			}
		} else {
			/* perform TDM record initialization */
			if (snd_adsp_tdm_record_init(&adsp_card->tdm_record,
						     substream) < 0) {
				/* perform TDM record de-initialization */
				/* when the initialization fails */
				snd_adsp_tdm_record_deinit(
						adsp_card->tdm_record);

				adsp_card->tdm_record = NULL;
				return -EINVAL;
			}
		}

		/* save the hardware parameters */
		snd_soc_set_runtime_hwparams(substream, &snd_pcm_adsp_tdm_hw);

		/* each period has a frame size */
		snd_pcm_hw_constraint_single(substream->runtime,
					     SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
					     TDM_FRAME_SIZE);
	}

	snd_pcm_hw_constraint_integer(substream->runtime,
				      SNDRV_PCM_HW_PARAM_PERIODS);

	return 0;
}

/** **************************************************************************
 *  \brief	 Close a playback/TDM playback or record/TDM record stream
 *  \internal
 *  \covers: DD_DRV_ALSA_01_029
 *
 *  \param[in]	substream		Pointer to substream object
 *  \retval	0			Success
 *  \retval	-EINVAL			Error
 *****************************************************************************/
static int snd_adsp_pcm_close(struct snd_pcm_substream *substream)
{
	/* get ADSP soundcard and CPU DAI index */
	struct snd_adsp_card *adsp_card =
		snd_adsp_get_drvdata_from_substream(substream);
	int dai_idx = snd_adsp_get_dai_id_from_substream(substream);
	int err = 0;

	if (dai_idx == RDR_DAI_IDX0 || dai_idx == RDR_DAI_IDX1 ||
	    dai_idx == RDR_DAI_IDX2 || dai_idx == RDR_DAI_IDX3) {
		/* destroy Renderer/Capture or Equalizer (if used) */
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			if (snd_adsp_playback_deinit(
					adsp_card->playback[dai_idx]) < 0)
				err = -EINVAL;

			adsp_card->playback[dai_idx] = NULL;
		} else {
			if (snd_adsp_record_deinit(
					adsp_card->record[dai_idx]) < 0)
				err = -EINVAL;

			adsp_card->record[dai_idx] = NULL;
		}
	} else {
		/* destroy TDM Renderer/TDM Capture */
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			if (snd_adsp_tdm_playback_deinit(
					adsp_card->tdm_playback) < 0)
				err = -EINVAL;

			adsp_card->tdm_playback = NULL;
		} else {
			if (snd_adsp_tdm_record_deinit(
					adsp_card->tdm_record) < 0)
				err = -EINVAL;

			adsp_card->tdm_record = NULL;
		}
	}

	return err;
}

/** **************************************************************************
 *  \brief	Allocate ALSA buffer and calculate expire time of hr timer
 *  \internal
 *  \covers: DD_DRV_ALSA_01_031
 *
 *  \param[in]	substream		Pointer to substream object
 *  \retval	0			Success
 *  \retval	-ENOMEM			Cannot allocate ALSA buffer
 *****************************************************************************/
static int snd_adsp_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *hw_params)
{
	struct snd_adsp_base_info *base;
	int err = 0;

	base = snd_adsp_get_base_from_substream(substream);

	/* set expire time of hrtimer, this value should be time for */
	/* transfer a frame */
	base->ktime = ns_to_ktime((1000000000 / params_rate(hw_params)) *
				  params_period_size(hw_params));

	err = snd_pcm_lib_malloc_pages(substream,
				       params_buffer_bytes(hw_params));

	/* reset DMA buffer area */
	memset(substream->runtime->dma_area, 0, substream->runtime->dma_bytes);

	return err;
}

/** **************************************************************************
 *  \brief	 Free the allocated ALSA buffer
 *  \internal
 *  \covers: DD_DRV_ALSA_01_032
 *
 *  \param[in]	substream		Pointer to substream object
 *  \retval	0			Success
 *  \retval	-EINVAL			Cannot deallocate buffer
 *****************************************************************************/
static int snd_adsp_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

/** **************************************************************************
 *  \brief	 Prepare playback/TDM playback or record/TDM record function
 *  \internal
 *  \covers: DD_DRV_ALSA_01_030
 *		   before transferring data
 *
 *  \param[in]	substream		Pointer to substream object
 *  \retval		 0		Success
 *  \retval		 -EINVAL	Error
 *****************************************************************************/
static int snd_adsp_pcm_prepare(struct snd_pcm_substream *substream)
{
	/* get ADSP soundcard and CPU DAI index */
	struct snd_adsp_card *
		adsp_card = snd_adsp_get_drvdata_from_substream(substream);
	int dai_idx = snd_adsp_get_dai_id_from_substream(substream);
	int err = 0;

	if (dai_idx == RDR_DAI_IDX0 || dai_idx == RDR_DAI_IDX1 ||
	    dai_idx == RDR_DAI_IDX2 || dai_idx == RDR_DAI_IDX3) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			/* get playback prepared to execute */
			err = snd_adsp_playback_prepare(
				adsp_card->playback[dai_idx], substream);
		else
			/* get record prepared to execute */
			err = snd_adsp_record_prepare(
				adsp_card->record[dai_idx], substream);
	} else {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			/* get TDM playback prepared to execute */
			err = snd_adsp_tdm_playback_prepare(
				adsp_card->tdm_playback, substream);
		else
			/* get TDM record prepared to execute */
			err = snd_adsp_tdm_record_prepare(
				adsp_card->tdm_record, substream);
	}

	return err;
}

/** **************************************************************************
 *  \brief	 Trigger playback/TDM playback or record/TDM record stream
 *  \internal
 *  \covers: DD_DRV_ALSA_01_033
 *		   to go to next phase
 *
 *  \param[in]	substream		Pointer to substream object
 *  \retval	0			Success
 *  \retval	-EINVAL			Error
 *****************************************************************************/
static int snd_adsp_pcm_trigger(struct snd_pcm_substream *substream, int idx)
{
	struct snd_adsp_base_info *
		base = snd_adsp_get_base_from_substream(substream);

	switch (idx) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		/* start high-resolution timer */
		hrtimer_start(&base->hrtimer, base->ktime, HRTIMER_MODE_REL);

		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			int buf_queue = 0;

			/* get current available buffer */
			spin_lock_irqsave(&base->lock, base->flag);
			buf_queue = base->buf_queue;
			spin_unlock_irqrestore(&base->lock, base->flag);

			/* send all available buffer to plugin to */
			/* get data */
			for (; buf_queue > 0; buf_queue--) {
				if (xf_adsp_fill_this_buffer(
						base->handle_id,
						base->buffer[base->buf_idx],
						base->buf_bytes) != 0)
					return -EINVAL;

				spin_lock_irqsave(&base->lock, base->flag);
				base->buf_queue--;
				spin_unlock_irqrestore(&base->lock, base->flag);

				base->buf_idx++;

				if (base->buf_idx >= XF_BUF_POOL_SIZE)
					base->buf_idx = 0;
			}
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/** **************************************************************************
 *  \brief	 Return HW data position
 *  \internal
 *  \covers: DD_DRV_ALSA_01_034
 *
 *  \param[in]	  substream		Pointer to substream object
 *  \retval	  position		HW data position
 *****************************************************************************/
static snd_pcm_uframes_t
snd_adsp_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_adsp_base_info *
		base = snd_adsp_get_base_from_substream(substream);
	unsigned int hw_idx, hw_buffer_size;
	snd_pcm_uframes_t pointer;

	/* convert hw index to correct as submitted bytes */
	spin_lock_irqsave(&base->lock, base->flag);
	hw_idx = (base->hw_idx / base->buf_bytes) * base->period_bytes;
	spin_unlock_irqrestore(&base->lock, base->flag);

	hw_buffer_size = base->pcm_indirect.hw_buffer_size;

	if (hw_idx >= hw_buffer_size) {
		spin_lock_irqsave(&base->lock, base->flag);

		base->hw_idx -= (hw_buffer_size / base->period_bytes) *
							base->buf_bytes;

		spin_unlock_irqrestore(&base->lock, base->flag);
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		pointer = snd_pcm_indirect_playback_pointer(substream,
							    &base->pcm_indirect,
							    hw_idx);
	else
		pointer = snd_pcm_indirect_capture_pointer(substream,
							   &base->pcm_indirect,
							   hw_idx);

	return pointer;
}

/** **************************************************************************
 *  \brief	  Call read/write process to transfer data
 *  \internal
 *  \covers: DD_DRV_ALSA_01_035
 *
 *  \param[in]	substream		Pointer to substream object
 *  \retval	0			Success
 *  \retval	-EINVAL			Error
 *****************************************************************************/
static int snd_adsp_pcm_ack(struct snd_pcm_substream *substream)
{
	struct snd_adsp_base_info *
		base = snd_adsp_get_base_from_substream(substream);

	if (base->runtime_err)
		return -EINVAL;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		snd_pcm_indirect_playback_transfer(substream,
						   &base->pcm_indirect,
						   snd_adsp_pcm_transfer);
	else
		snd_pcm_indirect_capture_transfer(substream,
						  &base->pcm_indirect,
						  snd_adsp_pcm_transfer);

	return 0;
}

/** ***************************************************************************
 *  \brief	  Copy data from source buffer to destination buffer
 *  \internal
 *  \covers: DD_DRV_ALSA_01_025
 *
 *  \params[in]	 dst		Destination buffer pointer
 *  \params[out] src		Source buffer pointer
 *  \params[in]	 dst_size	Destination buffer size in byte
 *  \params[in]	 src_size	Source buffer size in byte
 *  \retval	 None
 *****************************************************************************/
static inline void
snd_adsp_copy_data(void *dst, void *src, int dst_size, int src_size)
{
	unsigned char *data_dst = dst;
	unsigned char *data_src = src;
	int i;

	if (dst_size == src_size) {
		/* src and dst bufs are same size, does not need to convert */
		memcpy(data_dst, data_src, dst_size);

	} else if (dst_size < src_size) {
		for (i = 0; i < (dst_size - BYTES_PER_SAMPLE(S24_3LE));
				i += BYTES_PER_SAMPLE(S24_3LE)) {
			*(u32 *)data_dst = *(u32 *)data_src;

			data_dst += BYTES_PER_SAMPLE(S24_3LE);
			data_src += BYTES_PER_SAMPLE(S24_LE);
		}

		/* copy a S24_3LE sample from S24_LE sample */
		data_dst[0] = data_src[0];
		data_dst[1] = data_src[1];
		data_dst[2] = data_src[2];

	} else {
		unsigned int tmp;

		for (i = 0; i < (dst_size - BYTES_PER_SAMPLE(S24_LE));
				i += BYTES_PER_SAMPLE(S24_LE)) {
			tmp = *(u32 *)data_src;
			*(u32 *)data_dst = tmp & 0x0FFFFFF;

			data_dst += BYTES_PER_SAMPLE(S24_LE);
			data_src += BYTES_PER_SAMPLE(S24_3LE);
		}

		/* copy a S24_LE sample from S24_3LE sample */
		data_dst[0] = data_src[0];
		data_dst[1] = data_src[1];
		data_dst[2] = data_src[2];
		data_dst[3] = 0;
	}
}

/** **************************************************************************
 *  \brief	  Transfer data process between ALSA buffer and ADSP buffer
 *  \internal
 *  \covers: DD_DRV_ALSA_01_026
 *
 *  \param[in]	  substream		Pointer to substream object
 *  \param[in]	  rec			Pointer to indirect PCM data
 *  \param[in]	  bytes			Number of byte need to be transferred
 *  \retval	  None
 *****************************************************************************/
static void snd_adsp_pcm_transfer(struct snd_pcm_substream *substream,
				  struct snd_pcm_indirect *rec,
				  size_t bytes)
{
	struct snd_adsp_base_info *
		base = snd_adsp_get_base_from_substream(substream);
	int direct = substream->stream;
	int trans_bytes = bytes;
	int buf_bytes, period_bytes;
	void *dma_buf, *data_buff;

	/* get the DMA buffer pointer */
	dma_buf = (void *)(substream->runtime->dma_area + rec->sw_data);

	/* get information from base */
	buf_bytes = base->buf_bytes;
	period_bytes = base->period_bytes;

	/* make sure the available buffer and transfer size - TBD */
	while (trans_bytes > 0) {
		spin_lock_irqsave(&base->lock, base->flag);
		if (base->buf_queue > 0 && trans_bytes >= period_bytes) {
			base->buf_queue--;
			spin_unlock_irqrestore(&base->lock, base->flag);

			/* get the buffer pointer from stream */
			data_buff = base->buffer[base->buf_idx];

			if (direct == SNDRV_PCM_STREAM_PLAYBACK) {
				/* copy data from user *//* PRQA S 3200 2 */
				snd_adsp_copy_data(data_buff, dma_buf,
						   buf_bytes, period_bytes);

				/* send buffer to plugin */
				if (xf_adsp_empty_this_buffer(base->handle_id,
							      data_buff,
							      buf_bytes) < 0)
					base->runtime_err = TRUE;
			} else {
				/* copy data to user *//* PRQA S 3200 2 */
				snd_adsp_copy_data(dma_buf, data_buff,
						   period_bytes, buf_bytes);

				/* send buffer to plugin */
				if (xf_adsp_fill_this_buffer(base->handle_id,
							     data_buff,
							     buf_bytes) < 0)
					base->runtime_err = TRUE;
			}

			base->buf_idx++;
			if (base->buf_idx >= XF_BUF_POOL_SIZE)
				base->buf_idx = 0;
		} else {
			spin_unlock_irqrestore(&base->lock, base->flag);
			break;
		}

		trans_bytes -= period_bytes;
	}
}

/** **************************************************************************
 *  \brief	 Get info of Volume control
 *  \internal
 *  \covers: DD_DRV_ALSA_01_036
 *
 *  \param[in]	  kcontrol	Pointer to control instance
 *  \param[in]	  uinfo		Pointer to info structure of volume control
 *  \retval		 0	Success
 *****************************************************************************/
static int snd_adsp_control_volume_info(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = -1;
	uinfo->value.integer.max = 799;
	return 0;
}

/** **************************************************************************
 *  \brief	 Get volume value
 *  \internal
 *  \covers: DD_DRV_ALSA_01_037
 *
 *  \param[in]	kcontrol		Pointer to control instance
 *  \param[in]	ucontrol		Pointer to volume value
 *  \retval	0			Success
 *  \retval	-EINVAL			Error
 *****************************************************************************/
static int snd_adsp_control_volume_get(struct snd_kcontrol *kcontrol,
				       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_adsp_card *adsp_card = snd_kcontrol_chip(kcontrol);
	struct snd_adsp_control *ctr_if = &adsp_card->ctr_if;
	int handle_state, handle_id;
	int volume, cmd_idx, direction;
	unsigned int index;

	/* get the index */
	index = kcontrol->id.index;

	/* set handle state as NULL state */
	handle_state = XF_HANDLE_NULL;
	handle_id = -1;

	/* determine command index, direction, handle state, handle ID */
	if (kcontrol->id.name[0] == PREFIX_OF_PLAYBACK_CTR_NAME) {
		cmd_idx = XA_RDR_CONFIG_PARAM_VOLUME_RATE;
		direction = DIRECT_PLAYBACK;

		if (adsp_card->playback[index]) {
			handle_state = adsp_card->playback[index]->rdr_state;

			handle_id =
				adsp_card->playback[index]->renderer->handle_id;
		}
	} else if (kcontrol->id.name[0] == PREFIX_OF_CAPTURE_CTR_NAME) {
		cmd_idx = XA_CAP_CONFIG_PARAM_VOLUME_RATE;
		direction = DIRECT_CAPTURE;

		if (adsp_card->record[index]) {
			handle_state = adsp_card->record[index]->cap_state;

			handle_id =
				adsp_card->record[index]->capture->handle_id;
		}
	} else {
		if (kcontrol->id.name[3] == TDM_PLAYBACK) {
			cmd_idx = XA_TDM_RDR_CONFIG_PARAM_VOLUME_RATE;
			direction = DIRECT_PLAYBACK;

			if (adsp_card->tdm_playback) {
				handle_state = adsp_card->tdm_playback->state;

				handle_id =
			       adsp_card->tdm_playback->tdm_renderer->handle_id;
			}
		} else {
			cmd_idx = XA_TDM_CAP_CONFIG_PARAM_VOLUME_RATE;
			direction = DIRECT_CAPTURE;

			if (adsp_card->tdm_record) {
				handle_state = adsp_card->tdm_record->state;

				handle_id =
				  adsp_card->tdm_record->tdm_capture->handle_id;
			}
		}
	}

	/* get the volume's value */
	if (COMPONENT_IS_READY(handle_state) == TRUE) {
		/* get the volume's value from the plugin */
		if (xf_adsp_get_param(handle_id, cmd_idx, &volume) != 0)
			return -EINVAL;

		/* check the value after getting it and adjust it */
		ucontrol->value.integer.value[0] = (volume == 0xFFFFFFFF) ?
					(-1) : (volume * VOLUME_SCALE) >> 20;
	} else {
		if (kcontrol->id.name[0] != PREFIX_OF_TDM_CTR_NAME)
			ucontrol->value.integer.value[0] =
					(ctr_if->vol_rate[direction][index] *
					 VOLUME_SCALE) >> 20;
		else
			ucontrol->value.integer.value[0] =
					(ctr_if->tdm_vol_rate[direction] *
					 VOLUME_SCALE) >> 20;
	}

	return 0;
}

/** **************************************************************************
 *  \brief	 Set volume value
 *  \internal
 *  \covers: DD_DRV_ALSA_01_038
 *
 *  \param[in]	  kcontrol	Pointer to control instance
 *  \param[in]	  ucontrol	Pointer to volume value
 *  \retval	  1		Volume change
 *  \retval	  0		Volume does not change
 *  \retval	  -EINVAL	Error
 *****************************************************************************/
/* PRQA S 3673 1*/
static int
snd_adsp_control_volume_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_adsp_card *adsp_card = snd_kcontrol_chip(kcontrol);
	struct snd_adsp_control *ctr_if = &adsp_card->ctr_if;
	int volume;
	int ret;
	int handle_state, handle_id, cmd_idx, volume_get;
	int direction;
	unsigned int index;

	/* get the index */
	index = kcontrol->id.index;

	/* set handle state as NULL state and handle ID */
	handle_state = XF_HANDLE_NULL;
	handle_id = -1;

	/* get the value to set */
	if (ucontrol->value.integer.value[0] == -1) {
		volume = 0xFFFFFFFF;
	} else {
		/* round up the value if needed */
		volume = (ucontrol->value.integer.value[0] * (1 << 20)) /
			  VOLUME_SCALE;

		if ((ucontrol->value.integer.value[0] * (1 << 20)) >
		    (VOLUME_SCALE * volume))
			volume += 1;
	}

	/* determine command index, direction, handle state, handle ID */
	if (kcontrol->id.name[0] == PREFIX_OF_PLAYBACK_CTR_NAME) {
		cmd_idx = XA_RDR_CONFIG_PARAM_VOLUME_RATE;
		direction = DIRECT_PLAYBACK;

		if (adsp_card->playback[index]) {
			handle_state = adsp_card->playback[index]->rdr_state;

			handle_id =
				adsp_card->playback[index]->renderer->handle_id;
		}
	} else if (kcontrol->id.name[0] == PREFIX_OF_CAPTURE_CTR_NAME) {
		cmd_idx = XA_CAP_CONFIG_PARAM_VOLUME_RATE;
		direction = DIRECT_CAPTURE;

		if (adsp_card->record[index]) {
			handle_state = adsp_card->record[index]->cap_state;

			handle_id =
				adsp_card->record[index]->capture->handle_id;
		}
	} else {
		if (kcontrol->id.name[3] == TDM_PLAYBACK) {
			cmd_idx = XA_TDM_RDR_CONFIG_PARAM_VOLUME_RATE;
			direction = DIRECT_PLAYBACK;

			if (adsp_card->tdm_playback) {
				handle_state = adsp_card->tdm_playback->state;

				handle_id =
			       adsp_card->tdm_playback->tdm_renderer->handle_id;
			}
		} else {
			cmd_idx = XA_TDM_CAP_CONFIG_PARAM_VOLUME_RATE;
			direction = DIRECT_CAPTURE;

			if (adsp_card->tdm_record) {
				handle_state = adsp_card->tdm_record->state;

				handle_id =
				  adsp_card->tdm_record->tdm_capture->handle_id;
			}
		}
	}

	/* set volume to plugin */
	if (COMPONENT_IS_READY(handle_state) == TRUE) {
		/* TDM does not support set volume in runtime state */
		if (kcontrol->id.name[0] == PREFIX_OF_TDM_CTR_NAME)
			return -EINVAL;

		/* apply set volume value to ADSP from user setting */
		if (xf_adsp_set_param(handle_id, cmd_idx, volume) != 0)
			return -EINVAL;

		/* get volume from ADSP after setting to confirm */
		if (xf_adsp_get_param(handle_id, cmd_idx, &volume_get) != 0)
			return -EINVAL;

		/* check if the value has changed */
		ret = (volume_get == volume) ? 1 : 0;
	} else {
		if (kcontrol->id.name[0] != PREFIX_OF_TDM_CTR_NAME)
			ctr_if->vol_rate[direction][index] = volume;
		else
			ctr_if->tdm_vol_rate[direction] = volume;

		ret = 1;
	}

	return ret;
}

/** **************************************************************************
 *  \brief	 Get info of Sample Rate Converter control
 *  \internal
 *  \covers: DD_DRV_ALSA_01_039
 *
 *  \param[in]	kcontrol	Pointer to control instance
 *  \param[in]	uinfo		Pointer to info structure of sample rate
 *				converter control
 *  \retval	0		Success
 *****************************************************************************/
static int snd_adsp_control_sample_rate_info(struct snd_kcontrol *kcontrol,
					     struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = -1;
	uinfo->value.integer.max = 48000;
	return 0;
}

/** **************************************************************************
 *  \brief	 Get sample rate value
 *  \internal
 *  \covers: DD_DRV_ALSA_01_040
 *
 *  \param[in]	kcontrol	Pointer to control instance
 *  \param[in]	ucontrol	Pointer to sample rate value
 *  \retval	0		Success
 *  \retval	-EINVAL		Error
 *****************************************************************************/
static int snd_adsp_control_sample_rate_get(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_adsp_card *adsp_card = snd_kcontrol_chip(kcontrol);
	int *rate;
	int handle_id, handle_state, cmd_idx, direction;
	unsigned int index;

	rate = (int *)&ucontrol->value.integer.value[0]; /* PRQA S 310 */

	/* get the index */
	index = kcontrol->id.index;

	/* set handle state as NULL state and handle ID */
	handle_state = XF_HANDLE_NULL;
	handle_id = -1;

	/* determine command, direction, handle state, handle ID */
	if (kcontrol->id.name[0] == PREFIX_OF_PLAYBACK_CTR_NAME) {
		cmd_idx = XA_RDR_CONFIG_PARAM_OUT_SAMPLE_RATE;
		direction = DIRECT_PLAYBACK;

		if (adsp_card->playback[index]) {
			handle_state = adsp_card->playback[index]->rdr_state;

			handle_id =
				adsp_card->playback[index]->renderer->handle_id;
		}
	} else if (kcontrol->id.name[0] == PREFIX_OF_CAPTURE_CTR_NAME) {
		cmd_idx = XA_CAP_CONFIG_PARAM_SAMPLE_RATE;
		direction = DIRECT_CAPTURE;

		if (adsp_card->record[index]) {
			handle_state = adsp_card->record[index]->cap_state;

			handle_id =
				adsp_card->record[index]->capture->handle_id;
		}
	} else {
		if (kcontrol->id.name[3] == TDM_PLAYBACK) {
			cmd_idx = XA_TDM_RDR_CONFIG_PARAM_OUT_SAMPLE_RATE;
			direction = DIRECT_PLAYBACK;

			if (adsp_card->tdm_playback) {
				handle_state = adsp_card->tdm_playback->state;

				handle_id =
			       adsp_card->tdm_playback->tdm_renderer->handle_id;
			}
		} else {
			cmd_idx = XA_TDM_CAP_CONFIG_PARAM_IN_SAMPLE_RATE;
			direction = DIRECT_CAPTURE;

			if (adsp_card->tdm_record) {
				handle_state = adsp_card->tdm_record->state;

				handle_id =
				  adsp_card->tdm_record->tdm_capture->handle_id;
			}
		}
	}

	/* get parameter from plugin */
	if (COMPONENT_IS_READY(handle_state) == TRUE) {
		if (xf_adsp_get_param(handle_id, cmd_idx, rate) != 0)
			return -EINVAL;
	} else {
		if (kcontrol->id.name[0] != PREFIX_OF_TDM_CTR_NAME)
			*rate = adsp_card->ctr_if.sample_rate[direction][index];
		else
			*rate = adsp_card->ctr_if.tdm_sample_rate[direction];
	}

	/* success */
	return 0;
}

/** **************************************************************************
 *  \brief	 Set sample rate value
 *  \internal
 *  \covers: DD_DRV_ALSA_01_041
 *
 *  \param[in]	kcontrol	Pointer to control instance
 *  \param[in]	ucontrol	Pointer to sample rate value
 *  \retval	1		Sample rate change
 *  \retval	0		Sample rate does not change
 *  \retval	-EINVAL		Error
 *****************************************************************************/
/* PRQA S 3673 1 */
static int
snd_adsp_control_sample_rate_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_adsp_card *adsp_card = snd_kcontrol_chip(kcontrol);
	unsigned int index;
	int rate;
	int handle_state, cmd_idx, direction;
	int ret = 1;

	rate = (int)ucontrol->value.integer.value[0];

	/* get the index */
	index = kcontrol->id.index;

	/* set handle state as NULL state */
	handle_state = XF_HANDLE_NULL;

	/* determine command, direction and handle state */
	if (kcontrol->id.name[0] == PREFIX_OF_PLAYBACK_CTR_NAME) {
		cmd_idx = XA_RDR_CONFIG_PARAM_OUT_SAMPLE_RATE;
		direction = DIRECT_PLAYBACK;

		if (adsp_card->playback[index])
			handle_state = adsp_card->playback[index]->rdr_state;

	} else if (kcontrol->id.name[0] == PREFIX_OF_CAPTURE_CTR_NAME) {
		cmd_idx = XA_CAP_CONFIG_PARAM_SAMPLE_RATE;
		direction = DIRECT_CAPTURE;

		if (adsp_card->record[index])
			handle_state = adsp_card->record[index]->cap_state;

	} else {
		if (kcontrol->id.name[3] == TDM_PLAYBACK) {
			cmd_idx = XA_TDM_RDR_CONFIG_PARAM_OUT_SAMPLE_RATE;
			direction = DIRECT_PLAYBACK;

			if (adsp_card->tdm_playback)
				handle_state = adsp_card->tdm_playback->state;

		} else {
			cmd_idx = XA_TDM_CAP_CONFIG_PARAM_IN_SAMPLE_RATE;
			direction = DIRECT_CAPTURE;

			if (adsp_card->tdm_record)
				handle_state = adsp_card->tdm_record->state;
		}
	}

	/* get the value to set */
	if (COMPONENT_IS_READY(handle_state) == TRUE)
		return -EINVAL;

	if (kcontrol->id.name[0] != PREFIX_OF_TDM_CTR_NAME)
		adsp_card->ctr_if.sample_rate[direction][index] = rate;
	else
		adsp_card->ctr_if.tdm_sample_rate[direction] = rate;

	ret = 1;

	return ret;
}

/** **************************************************************************
 *  \brief	 Get info of Equalizer control
 *  \internal
 *  \covers: DD_DRV_ALSA_01_045
 *
 *  \param[in]	kcontrol	Pointer to control instance
 *  \param[in]	uinfo		Pointer to info structure of Equalizer control
 *  \retval	0		Success
 *****************************************************************************/
static int snd_adsp_control_eqz_info(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = MAX_EQZ_PARAM_NUMBER;
	uinfo->value.integer.min = -1;
	uinfo->value.integer.max = 0x7fffffff;
	return 0;
}

/** **************************************************************************
 *  \brief	 Get equalizer parameters value
 *  \internal
 *  \covers: DD_DRV_ALSA_01_046
 *
 *  \param[in]	  kcontrol		Pointer to control instance
 *  \param[in]	  ucontrol		Pointer to equalizer parameters value
 *  \retval	  0			Success
 *  \retval	  -EINVAL		Error
 *****************************************************************************/
static int snd_adsp_control_eqz_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_adsp_card *adsp_card = snd_kcontrol_chip(kcontrol);
	int handle_state, direction;
	int i, j, filter_index;
	unsigned int index;
	struct xf_adsp_equalizer *equalizer;
	struct xf_adsp_equalizer_params *eqz_params;

	/* get the index */
	index = kcontrol->id.index;

	/* determin direction of stream */
	direction = (kcontrol->id.name[0] == PREFIX_OF_PLAYBACK_CTR_NAME) ?
					     DIRECT_PLAYBACK : DIRECT_CAPTURE;

	/* set handle state as NULL state, handle ID, equalizer pointer */
	handle_state = XF_HANDLE_NULL;
	equalizer = NULL;

	/* get component's state */
	if (direction == DIRECT_PLAYBACK) {
		if (adsp_card->playback[index]) {
			handle_state = adsp_card->playback[index]->eqz_state;
			equalizer = adsp_card->playback[index]->equalizer;
		}
	} else {
		if (adsp_card->record[index]) {
			handle_state = adsp_card->record[index]->eqz_state;
			equalizer = adsp_card->record[index]->equalizer;
		}
	}

	/* perform parameters' values getting */
	if (COMPONENT_IS_READY(handle_state) == TRUE) {
		/* get equalizer's parameters */
		if (xf_adsp_equalizer_get_params(equalizer) != 0)
			return -EINVAL;

		eqz_params = &equalizer->params;
	} else {
		eqz_params = &adsp_card->ctr_if.eqz_params[direction][index];
	}

	/* get equalizer type: PARAMETRIC or GRAPHIC */
	ucontrol->value.integer.value[0] = eqz_params->eqz_type;

	/* get parameters' value from Equalizer plugin */
	if (eqz_params->eqz_type == XA_REL_EQZ_TYPE_PARAMETRIC) {
		for (i = 0, filter_index = 1;
		     filter_index <= XA_REL_EQZ_FILTER_NUM;
		     i++, filter_index++) {
			/* get filter index */
			ucontrol->value.integer.value[(i * 6) + 1] =
				filter_index;

			/* get frequency centre */
			ucontrol->value.integer.value[(i * 6) + 2] =
				eqz_params->p_coef.fc[i];

			/* get bandwidth */
			ucontrol->value.integer.value[(i * 6) + 3] =
				eqz_params->p_coef.band_width[i];

			/* get filter type */
			ucontrol->value.integer.value[(i * 6) + 4] =
				eqz_params->p_coef.type[i];

			/* get gain base */
			ucontrol->value.integer.value[(i * 6) + 5] =
				eqz_params->p_coef.gain_base[i];

			/* get gain */
			ucontrol->value.integer.value[(i * 6) + 6] =
				eqz_params->p_coef.gain[i];
		}
	} else {
		for (i = 0, filter_index = 1;
		     filter_index <= XA_REL_EQZ_GRAPHIC_BAND_NUM;
		     i++, filter_index++) {
			/* get band index */
			ucontrol->value.integer.value[(i * 2) + 1] =
				filter_index;

			/* get graphic gain */
			ucontrol->value.integer.value[(i * 2) + 2] =
				eqz_params->g_coef.gain_g[i];
		}

		j = (i * 2) + 1;

		/* make the rest of values silent */
		while (j < MAX_EQZ_PARAM_NUMBER) {
			ucontrol->value.integer.value[j] = -1;
			j++;
		}
	}

	return 0;
}

/** **************************************************************************
 *  \brief	 Set equalizer parameters value
 *  \internal
 *  \covers: DD_DRV_ALSA_01_047
 *
 *  \param[in]	  kcontrol	Pointer to control instance
 *  \param[in]	  ucontrol	Pointer to equalizer parameters value
 *  \retval	  1		Equalizer parameters change
 *  \retval	  0		Equalizer parameters does not change
 *  \retval	 -EINVAL	Error
 *****************************************************************************/
/* PRQA S 3673 1*/
static int snd_adsp_control_eqz_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_adsp_card *adsp_card = snd_kcontrol_chip(kcontrol);
	int handle_state, direction, filter_idx;
	int i;
	int index;
	struct xf_adsp_equalizer_params *eqz_params = NULL;

	/* get the index */
	index = kcontrol->id.index;

	/* determine the direction */
	direction = (kcontrol->id.name[0] == PREFIX_OF_PLAYBACK_CTR_NAME) ?
					     DIRECT_PLAYBACK : DIRECT_CAPTURE;

	/* set handle state as NULL state */
	handle_state = XF_HANDLE_NULL;

	/* get the handle state */
	if (direction == DIRECT_PLAYBACK) {
		if (adsp_card->playback[index])
			handle_state = adsp_card->playback[index]->eqz_state;
	} else {
		if (adsp_card->record[index])
			handle_state = adsp_card->record[index]->eqz_state;
	}

	/* perform equalizer's parameters setting */
	if (COMPONENT_IS_READY(handle_state) == TRUE)
		return -EINVAL;

	/* PRQA S 310 1*/
	eqz_params = &adsp_card->ctr_if.eqz_params[direction][index];

	i = 0;
	/* PRQA S 3440 1*/
	eqz_params->eqz_type = ucontrol->value.integer.value[i];
	i++;
	filter_idx = 0;

	if (eqz_params->eqz_type == XA_REL_EQZ_TYPE_PARAMETRIC) {
		while (i < MAX_EQZ_PARAM_NUMBER) {
			/* get index filter */
			/* PRQA S 3440*/
			filter_idx = ucontrol->value.integer.value[i];
			i++;

			/* valid index filter is range */
			/* (1 to XA_REL_EQZ_FILTER_NUM) */
			if (filter_idx >= 1 &&
			    filter_idx <= XA_REL_EQZ_FILTER_NUM) {
					/* PRQA S 3440 5*/
				eqz_params->p_coef.fc[filter_idx - 1] =
					ucontrol->value.integer.value[i];

				i++;
				eqz_params->p_coef.band_width[filter_idx - 1] =
					ucontrol->value.integer.value[i];

				i++;
				eqz_params->p_coef.type[filter_idx - 1] =
					ucontrol->value.integer.value[i];

				i++;
				eqz_params->p_coef.gain_base[filter_idx - 1] =
					ucontrol->value.integer.value[i];

				i++;
				eqz_params->p_coef.gain[filter_idx - 1] =
					ucontrol->value.integer.value[i];

				i++;
			}
			/* index filter = -1 means that user */
			/* does not set this filter */
			else if (filter_idx == -1 || filter_idx == 0) {
				i += 5;
			} else {
				return -EINVAL;
			}
		}
	} else if (eqz_params->eqz_type == XA_REL_EQZ_TYPE_GRAPHIC) {
		while (i < ((XA_REL_EQZ_GRAPHIC_BAND_NUM * 2) + 1)) {
			/*get index filter */
			/* PRQA S 3440 */
			filter_idx = ucontrol->value.integer.value[i++];

			if (filter_idx >= 1 &&
			    filter_idx <= XA_REL_EQZ_GRAPHIC_BAND_NUM) {
				eqz_params->g_coef.gain_g[filter_idx - 1] =
					ucontrol->value.integer.value[i];
				i++;

			} else if (filter_idx == -1 || filter_idx == 0) {
				i += 1;
			} else {
				return -EINVAL;
			}
		}
	} else {
		return -EINVAL;
	}

	return 1;
}

/** **************************************************************************
 *  \brief	 Get info of Equalizer Switch control
 *  \internal
 *  \covers: DD_DRV_ALSA_01_042
 *
 *  \param[in]	  kcontrol	Pointer to control instance
 *  \param[in]	  uinfo		Pointer to info structure of Equalizer
 *				Switch control
 *  \retval	  0		Success
 *****************************************************************************/
static int snd_adsp_control_eqz_switch_info(struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

/** **************************************************************************
 *  \brief	 Get equalizer switch value
 *  \internal
 *  \covers: DD_DRV_ALSA_01_043
 *
 *  \param[in]	  kcontrol	Pointer to control instance
 *  \param[in]	  ucontrol	Pointer to equalizer switch value
 *  \retval	  0		Success
 *  \retval	  -EINVAL	Error
 *****************************************************************************/
static int
snd_adsp_control_eqz_switch_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_adsp_card *adsp_card = snd_kcontrol_chip(kcontrol);
	int direction;
	unsigned int index;

	/* get the index */
	index = kcontrol->id.index;

	/* determine the direction */
	direction = (kcontrol->id.name[0] == PREFIX_OF_PLAYBACK_CTR_NAME) ?
					     DIRECT_PLAYBACK : DIRECT_CAPTURE;

	/* get the Equalizer switch status */
	ucontrol->value.integer.value[0] =
		adsp_card->ctr_if.eqz_switch[direction][index];

	return 0;
}

/** **************************************************************************
 *  \brief	 Set equalizer switch value
 *  \internal
 *  \covers: DD_DRV_ALSA_01_044
 *
 *  \param[in]	  kcontrol	Pointer to control instance
 *  \param[in]	  ucontrol	Pointer to equalizer switch value
 *  \retval	  1		Equalizer switch change
 *  \retval	  0		Equalizer switch does not change
 *  \retval	 -EINVAL	Error
 *****************************************************************************/
/* PRQA S 3673 1 */
static int
snd_adsp_control_eqz_switch_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_adsp_card *adsp_card = snd_kcontrol_chip(kcontrol);
	int eqz_switch;
	int handle_state, direction;
	unsigned int index;
	int ret = 0;

	eqz_switch = ucontrol->value.integer.value[0];

	/* get the index */
	index = kcontrol->id.index;

	/* determine the direction */
	direction = (kcontrol->id.name[0] == PREFIX_OF_PLAYBACK_CTR_NAME) ?
		     DIRECT_PLAYBACK : DIRECT_CAPTURE;

	/* set handle state as NULL state */
	handle_state = XF_HANDLE_NULL;

	/* determine handle state */
	if (direction == DIRECT_PLAYBACK) {
		if (adsp_card->playback[index])
			handle_state = adsp_card->playback[index]->rdr_state;
	} else {
		if (adsp_card->record[index])
			handle_state = adsp_card->record[index]->cap_state;
	}

	/* set the status of Equalizer */
	if (COMPONENT_IS_READY(handle_state) == TRUE) {
		/* runtime setting is not supported */
		ret = -EINVAL;
	} else {
		adsp_card->ctr_if.eqz_switch[direction][index] = eqz_switch;
		ret = 1;
	}

	return ret;
}

/** **************************************************************************
 *  \brief	 Get info of Renderer output channel
 *  \internal
 *  \covers: DD_DRV_ALSA_01_048
 *
 *  \param[in]	  kcontrol Pointer to control instance
 *  \param[in]	  uinfo	   Pointer to info structure of Renderer output channel
 *  \retval	  0	   Success
 *****************************************************************************/
static int
snd_adsp_control_rdr_out_channel_info(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = MONAURAL;
	uinfo->value.integer.max = STEREO;
	return 0;
}

/** **************************************************************************
 *  \brief	 Get Renderer output channel's value
 *  \internal
 *  \covers: DD_DRV_ALSA_01_049
 *
 *  \param[in]	  kcontrol	Pointer to control instance
 *  \param[in]	  ucontrol	Pointer to Renderer output channel value
 *  \retval	  0		Success
 *  \retval	 -EINVAL	Error
 *****************************************************************************/
static int
snd_adsp_control_rdr_out_channel_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_adsp_card *adsp_card = snd_kcontrol_chip(kcontrol);
	int rdr_out_ch;
	unsigned int index;
	int handle_state, handle_id;

	/* get the index */
	index = kcontrol->id.index;

	handle_state = XF_HANDLE_NULL;
	handle_id = -1;

	/* determine handle state and handle ID */
	if (adsp_card->playback[index]) {
		handle_state = adsp_card->playback[index]->rdr_state;
		handle_id = adsp_card->playback[index]->renderer->handle_id;
	}

	/* get Renderer output channel's value */
	if (COMPONENT_IS_READY(handle_state) == TRUE) {
		/* get Renderer output channel's value from Renderer plugin */
		if (xf_adsp_get_param(handle_id,
				      XA_RDR_CONFIG_PARAM_OUT_CHANNELS,
				      &rdr_out_ch) != 0) {
			return -EINVAL;
		}

		ucontrol->value.integer.value[0] = rdr_out_ch;
	} else {
		ucontrol->value.integer.value[0] =
		adsp_card->ctr_if.rdr_out_ch[index];
	}

	return 0;
}

/** **************************************************************************
 *  \brief	 Set Renderer output channel's value
 *  \internal
 *  \covers: DD_DRV_ALSA_01_050
 *
 *  \param[in]	  kcontrol	Pointer to control instance
 *  \param[in]	  ucontrol	Pointer to Renderer output channel value
 *  \retval	  1		Success
 *  \retval	  -EINVAL	Error
 *****************************************************************************/
static int
snd_adsp_control_rdr_out_channel_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_adsp_card *adsp_card = snd_kcontrol_chip(kcontrol);
	unsigned int index;
	int handle_state;
	int ret = 1;

	/* get the index */
	index = kcontrol->id.index;

	handle_state = XF_HANDLE_NULL;

	/* determine handle state and handle ID */
	if (adsp_card->playback[index])
		handle_state = adsp_card->playback[index]->rdr_state;

	/* get Renderer output channel's value */
	if (COMPONENT_IS_READY(handle_state) == TRUE)
		/* not support runtime setting */
		ret = -EINVAL;
	else
		adsp_card->ctr_if.rdr_out_ch[index] =
			ucontrol->value.integer.value[0];

	return ret;
}

/** control interface for playback's volume rate */
/* PRQA S 3218 */
static struct snd_kcontrol_new
snd_adsp_playback_volume_control[MAX_DAI_IDX - 1] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX0,
		.name = PLAYBACK_VOLUME_CTR_NAME,
		.info = &snd_adsp_control_volume_info,
		.get = &snd_adsp_control_volume_get,
		.put = &snd_adsp_control_volume_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX1,
		.name = PLAYBACK_VOLUME_CTR_NAME,
		.info = &snd_adsp_control_volume_info,
		.get = &snd_adsp_control_volume_get,
		.put = &snd_adsp_control_volume_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX2,
		.name = PLAYBACK_VOLUME_CTR_NAME,
		.info = &snd_adsp_control_volume_info,
		.get = &snd_adsp_control_volume_get,
		.put = &snd_adsp_control_volume_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX3,
		.name = PLAYBACK_VOLUME_CTR_NAME,
		.info = &snd_adsp_control_volume_info,
		.get = &snd_adsp_control_volume_get,
		.put = &snd_adsp_control_volume_put
	}
};

/** control interface for Capture's volume rate */
/* PRQA S 3218 1*/
static struct snd_kcontrol_new
snd_adsp_capture_volume_control[MAX_DAI_IDX - 1] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX0,
		.name = CAPTURE_VOLUME_CTR_NAME,
		.info = &snd_adsp_control_volume_info,
		.get = &snd_adsp_control_volume_get,
		.put = &snd_adsp_control_volume_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX1,
		.name = CAPTURE_VOLUME_CTR_NAME,
		.info = &snd_adsp_control_volume_info,
		.get = &snd_adsp_control_volume_get,
		.put = &snd_adsp_control_volume_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX2,
		.name = CAPTURE_VOLUME_CTR_NAME,
		.info = &snd_adsp_control_volume_info,
		.get = &snd_adsp_control_volume_get,
		.put = &snd_adsp_control_volume_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX3,
		.name = CAPTURE_VOLUME_CTR_NAME,
		.info = &snd_adsp_control_volume_info,
		.get = &snd_adsp_control_volume_get,
		.put = &snd_adsp_control_volume_put
	}
};

/** control interface for playback's output sample rate */
/* PRQA S 3218 1*/
static struct snd_kcontrol_new
snd_adsp_playback_sample_rate_out_control[MAX_DAI_IDX - 1] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX0,
		.name = PLAYBACK_OUT_RATE_CTR_NAME,
		.info = &snd_adsp_control_sample_rate_info,
		.get = &snd_adsp_control_sample_rate_get,
		.put = &snd_adsp_control_sample_rate_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX1,
		.name = PLAYBACK_OUT_RATE_CTR_NAME,
		.info = &snd_adsp_control_sample_rate_info,
		.get = &snd_adsp_control_sample_rate_get,
		.put = &snd_adsp_control_sample_rate_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX2,
		.name = PLAYBACK_OUT_RATE_CTR_NAME,
		.info = &snd_adsp_control_sample_rate_info,
		.get = &snd_adsp_control_sample_rate_get,
		.put = &snd_adsp_control_sample_rate_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX3,
		.name = PLAYBACK_OUT_RATE_CTR_NAME,
		.info = &snd_adsp_control_sample_rate_info,
		.get = &snd_adsp_control_sample_rate_get,
		.put = &snd_adsp_control_sample_rate_put
	}
};

/** control interface for Capture's input sample rate */
/* PRQA S 3218 */
static struct snd_kcontrol_new
snd_adsp_capture_sample_rate_in_control[MAX_DAI_IDX - 1] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX0,
		.name = CAPTURE_IN_RATE_CTR_NAME,
		.info = &snd_adsp_control_sample_rate_info,
		.get = &snd_adsp_control_sample_rate_get,
		.put = &snd_adsp_control_sample_rate_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX1,
		.name = CAPTURE_IN_RATE_CTR_NAME,
		.info = &snd_adsp_control_sample_rate_info,
		.get = &snd_adsp_control_sample_rate_get,
		.put = &snd_adsp_control_sample_rate_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX2,
		.name = CAPTURE_IN_RATE_CTR_NAME,
		.info = &snd_adsp_control_sample_rate_info,
		.get = &snd_adsp_control_sample_rate_get,
		.put = &snd_adsp_control_sample_rate_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX3,
		.name = CAPTURE_IN_RATE_CTR_NAME,
		.info = &snd_adsp_control_sample_rate_info,
		.get = &snd_adsp_control_sample_rate_get,
		.put = &snd_adsp_control_sample_rate_put
	}
};

/** control interface for Equalizer parameters in playback */
/* PRQA S 3218 1*/
static struct snd_kcontrol_new
snd_adsp_playback_equalizer_control[MAX_DAI_IDX - 1] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX0,
		.name = PLAYBACK_EQZ_CTR_NAME,
		.info = &snd_adsp_control_eqz_info,
		.get = &snd_adsp_control_eqz_get,
		.put = &snd_adsp_control_eqz_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX1,
		.name = PLAYBACK_EQZ_CTR_NAME,
		.info = &snd_adsp_control_eqz_info,
		.get = &snd_adsp_control_eqz_get,
		.put = &snd_adsp_control_eqz_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX2,
		.name = PLAYBACK_EQZ_CTR_NAME,
		.info = &snd_adsp_control_eqz_info,
		.get = &snd_adsp_control_eqz_get,
		.put = &snd_adsp_control_eqz_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX3,
		.name = PLAYBACK_EQZ_CTR_NAME,
		.info = &snd_adsp_control_eqz_info,
		.get = &snd_adsp_control_eqz_get,
		.put = &snd_adsp_control_eqz_put
	}
};

/** control interface for Equalizer parameters in record */
/* PRQA S 3218 */
static struct snd_kcontrol_new
snd_adsp_capture_equalizer_control[MAX_DAI_IDX - 1] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX0,
		.name = CAPTURE_EQZ_CTR_NAME,
		.info = &snd_adsp_control_eqz_info,
		.get = &snd_adsp_control_eqz_get,
		.put = &snd_adsp_control_eqz_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX1,
		.name = CAPTURE_EQZ_CTR_NAME,
		.info = &snd_adsp_control_eqz_info,
		.get = &snd_adsp_control_eqz_get,
		.put = &snd_adsp_control_eqz_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX2,
		.name = CAPTURE_EQZ_CTR_NAME,
		.info = &snd_adsp_control_eqz_info,
		.get = &snd_adsp_control_eqz_get,
		.put = &snd_adsp_control_eqz_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX3,
		.name = CAPTURE_EQZ_CTR_NAME,
		.info = &snd_adsp_control_eqz_info,
		.get = &snd_adsp_control_eqz_get,
		.put = &snd_adsp_control_eqz_put
	}
};

/** control interface for Equalizer usage in playback */
/* PRQA S 3218 */
static struct snd_kcontrol_new
snd_adsp_playback_equalizer_switch_control[MAX_DAI_IDX - 1] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX0,
		.name = PLAYBACK_EQZ_SWITCH_CTR_NAME,
		.info = &snd_adsp_control_eqz_switch_info,
		.get = &snd_adsp_control_eqz_switch_get,
		.put = &snd_adsp_control_eqz_switch_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX1,
		.name = PLAYBACK_EQZ_SWITCH_CTR_NAME,
		.info = &snd_adsp_control_eqz_switch_info,
		.get = &snd_adsp_control_eqz_switch_get,
		.put = &snd_adsp_control_eqz_switch_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX2,
		.name = PLAYBACK_EQZ_SWITCH_CTR_NAME,
		.info = &snd_adsp_control_eqz_switch_info,
		.get = &snd_adsp_control_eqz_switch_get,
		.put = &snd_adsp_control_eqz_switch_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX3,
		.name = PLAYBACK_EQZ_SWITCH_CTR_NAME,
		.info = &snd_adsp_control_eqz_switch_info,
		.get = &snd_adsp_control_eqz_switch_get,
		.put = &snd_adsp_control_eqz_switch_put
	}
};

/** control interface for Equalizer usage in record */
/* PRQA S 3218 1*/
static struct snd_kcontrol_new
snd_adsp_capture_equalizer_switch_control[MAX_DAI_IDX - 1] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX0,
		.name = CAPTURE_EQZ_SWITCH_CTR_NAME,
		.info = &snd_adsp_control_eqz_switch_info,
		.get = &snd_adsp_control_eqz_switch_get,
		.put = &snd_adsp_control_eqz_switch_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX1,
		.name = CAPTURE_EQZ_SWITCH_CTR_NAME,
		.info = &snd_adsp_control_eqz_switch_info,
		.get = &snd_adsp_control_eqz_switch_get,
		.put = &snd_adsp_control_eqz_switch_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX2,
		.name = CAPTURE_EQZ_SWITCH_CTR_NAME,
		.info = &snd_adsp_control_eqz_switch_info,
		.get = &snd_adsp_control_eqz_switch_get,
		.put = &snd_adsp_control_eqz_switch_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX3,
		.name = CAPTURE_EQZ_SWITCH_CTR_NAME,
		.info = &snd_adsp_control_eqz_switch_info,
		.get = &snd_adsp_control_eqz_switch_get,
		.put = &snd_adsp_control_eqz_switch_put
	}
};

/** control interface for playback's output channel */
static struct snd_kcontrol_new
snd_adsp_playback_out_channel_control[MAX_DAI_IDX - 1] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	/* PRQA S 1053 5 */
		.index = RDR_DAI_IDX0,
		.name = PLAYBACK_OUT_CHANNEL_CTR_NAME,
		.info = &snd_adsp_control_rdr_out_channel_info,
		.get = &snd_adsp_control_rdr_out_channel_get,
		.put = &snd_adsp_control_rdr_out_channel_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index = RDR_DAI_IDX1,
		.name = PLAYBACK_OUT_CHANNEL_CTR_NAME,
		.info = &snd_adsp_control_rdr_out_channel_info,
		.get = &snd_adsp_control_rdr_out_channel_get,
		.put = &snd_adsp_control_rdr_out_channel_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index = RDR_DAI_IDX2,
		.name = PLAYBACK_OUT_CHANNEL_CTR_NAME,
		.info = &snd_adsp_control_rdr_out_channel_info,
		.get = &snd_adsp_control_rdr_out_channel_get,
		.put = &snd_adsp_control_rdr_out_channel_put
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.index = RDR_DAI_IDX3,
		.name = PLAYBACK_OUT_CHANNEL_CTR_NAME,
		.info = &snd_adsp_control_rdr_out_channel_info,
		.get = &snd_adsp_control_rdr_out_channel_get,
		.put = &snd_adsp_control_rdr_out_channel_put
	}
};

/** control interface for TDM playback's volume rate */
static struct snd_kcontrol_new snd_adsp_tdm_playback_volume_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,		/* PRQA S 1053 5 */
	.name = TDM_PLAYBACK_VOLUME_CTR_NAME,
	.info = &snd_adsp_control_volume_info,
	.get = &snd_adsp_control_volume_get,
	.put = &snd_adsp_control_volume_put
};

/** control interface for TDM capture's volume rate */
static struct snd_kcontrol_new snd_adsp_tdm_capture_volume_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,		/* PRQA S 1053 5 */
	.name = TDM_CAPTURE_VOLUME_CTR_NAME,
	.info = &snd_adsp_control_volume_info,
	.get = &snd_adsp_control_volume_get,
	.put = &snd_adsp_control_volume_put
};

/** control interface for TDM playback's output sample rate */
static struct snd_kcontrol_new snd_adsp_tdm_playback_sample_rate_out_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,		/* PRQA S 1053 5 */
	.name = TDM_PLAYBACK_OUT_RATE_CTR_NAME,
	.info = &snd_adsp_control_sample_rate_info,
	.get = &snd_adsp_control_sample_rate_get,
	.put = &snd_adsp_control_sample_rate_put
};

/** control interface for TDM capture's input sample rate */
static struct snd_kcontrol_new snd_adsp_tdm_capture_sample_rate_in_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,		/* PRQA S 1053 5 */
	.name = TDM_CAPTURE_IN_RATE_CTR_NAME,
	.info = &snd_adsp_control_sample_rate_info,
	.get = &snd_adsp_control_sample_rate_get,
	.put = &snd_adsp_control_sample_rate_put
};

/** PCM callback function of the sound card */
static struct snd_pcm_ops snd_adsp_pcm_ops = {
	.open = &snd_adsp_pcm_open,	/* PRQA S 1053 10 */
	.close = &snd_adsp_pcm_close,
	.ioctl = &snd_pcm_lib_ioctl,
	.hw_params = &snd_adsp_pcm_hw_params,
	.hw_free = &snd_adsp_pcm_hw_free,
	.prepare = &snd_adsp_pcm_prepare,
	.trigger = &snd_adsp_pcm_trigger,
	.pointer = &snd_adsp_pcm_pointer,
	.ack = &snd_adsp_pcm_ack
};

/*******************************************************************
 * Internal functions
 * ****************************************************************/

/** **************************************************************************
 *  \brief	 Process information get from control structure
 *  \internal
 *  \covers: DD_DRV_ALSA_01_027
 *
 *  \param[in]	  eqz_params	 Equalizer parameters object
 *  \param[in]	  eqz_ctr_params Equalizer parameters stored in control object
 *  \param[in]	  flag		 Indicate playback or capture stream
 *****************************************************************************/
static void snd_adsp_get_eqz_params_from_control(
		struct xf_adsp_equalizer_params *eqz_params,
		struct xf_adsp_equalizer_params *eqz_ctr_params, bool flag)
{
	int filter_idx;

	if (flag) {
		if (eqz_ctr_params->eqz_type >= 0)
			eqz_params->eqz_type = eqz_ctr_params->eqz_type;

		for (filter_idx = 0; filter_idx < XA_REL_EQZ_FILTER_NUM;
		     filter_idx++) {
			if (eqz_ctr_params->p_coef.fc[filter_idx] >= 0)
				eqz_params->p_coef.fc[filter_idx] =
				  eqz_ctr_params->p_coef.fc[filter_idx];

			if (eqz_ctr_params->p_coef.band_width[filter_idx] >= 0)
				eqz_params->p_coef.band_width[filter_idx] =
				eqz_ctr_params->p_coef.band_width[filter_idx];

			if (eqz_ctr_params->p_coef.type[filter_idx] >= 0)
				eqz_params->p_coef.type[filter_idx] =
				  eqz_ctr_params->p_coef.type[filter_idx];

			if (eqz_ctr_params->p_coef.gain_base[filter_idx] >= 0)
				eqz_params->p_coef.gain_base[filter_idx] =
				  eqz_ctr_params->p_coef.gain_base[filter_idx];

			if (eqz_ctr_params->p_coef.gain[filter_idx] >= 0)
				eqz_params->p_coef.gain[filter_idx] =
				  eqz_ctr_params->p_coef.gain[filter_idx];
		}
		for (filter_idx = 0; filter_idx < XA_REL_EQZ_GRAPHIC_BAND_NUM;
		     filter_idx++) {
			if (eqz_ctr_params->g_coef.gain_g[filter_idx] >= 0)
				eqz_params->g_coef.gain_g[filter_idx] =
				  eqz_ctr_params->g_coef.gain_g[filter_idx];
		}
	} else {
		memcpy(eqz_ctr_params, eqz_params, sizeof(*eqz_params));
	}
}

/*******************************************************************
 * ALSA ADSP Platform driver interface
 * ****************************************************************/

/** ****************************************************************************
 *  \brief	 Register control interface and preallocate ALSA buffer
 *  \internal
 *  \covers: DD_DRV_ALSA_01_005
 *
 *  \param[in]	  runtime	Pointer to runtime PCM data
 *  \retval	  0		Success
 *  \retval	 -EINVAL	Cannot register control interface
 ******************************************************************************/
static int snd_adsp_pcm_new(struct snd_soc_pcm_runtime *runtime)
{
	int i = 0, err = 0;
	int id;
	struct snd_card *card;
	struct snd_adsp_card *adsp_card;
	int alsa_buf_sz = 0;

	/* get sound card data */
	card = runtime->card->snd_card;

	/* get driver data */
	adsp_card = snd_soc_dai_get_drvdata(runtime->cpu_dai);

	/* get the ID of CPU DAI */
	id = runtime->cpu_dai->id;

	/* register control interfaces */
	if (id == RDR_DAI_IDX0 || id == RDR_DAI_IDX1 ||
	    id == RDR_DAI_IDX2 || id == RDR_DAI_IDX3) {
		struct snd_kcontrol *kctl[RDR_CONTROL_NUM];
		void *rdr_ctr[RDR_CONTROL_NUM] = {
			&snd_adsp_playback_volume_control[id], /* PRQA S 1031 */
			&snd_adsp_capture_volume_control[id],
			&snd_adsp_playback_sample_rate_out_control[id],
			&snd_adsp_capture_sample_rate_in_control[id],
			&snd_adsp_playback_equalizer_control[id],
			&snd_adsp_capture_equalizer_control[id],
			&snd_adsp_playback_equalizer_switch_control[id],
			&snd_adsp_capture_equalizer_switch_control[id],
			&snd_adsp_playback_out_channel_control[id]
		};

		/* add basic control instance */
		for (i = 0; i < RDR_CONTROL_NUM; i++) {
			kctl[i] = snd_ctl_new1(rdr_ctr[i], adsp_card);
			err = snd_ctl_add(card, kctl[i]);
			if (err < 0)
				return -EINVAL;
		}

		/* assign ALSA buffer size */
		alsa_buf_sz = MAX_BUFFER_BYTES;

		/* enable MIX feature from the 2nd playback/record stream */
		if (adsp_card->ctr_if.mix_usage == MIX_UNUSED)
			adsp_card->ctr_if.mix_usage = FIRST_RUN;
		else if (adsp_card->ctr_if.mix_usage == FIRST_RUN)
			adsp_card->ctr_if.mix_usage = SECOND_RUN;

	} else {
		struct snd_kcontrol *kctl[TDM_CONTROL_NUM];
		void *tdm_ctr[TDM_CONTROL_NUM] = {
			&snd_adsp_tdm_playback_volume_control,
			&snd_adsp_tdm_playback_sample_rate_out_control,
			&snd_adsp_tdm_capture_volume_control,
			&snd_adsp_tdm_capture_sample_rate_in_control
		};

		/* add basic control instances */
		for (i = 0; i < TDM_CONTROL_NUM; i++) {
			kctl[i] = snd_ctl_new1(tdm_ctr[i], adsp_card);
			err = snd_ctl_add(card, kctl[i]);
			if (err < 0)
				return -EINVAL;
		}

		/* assign ALSA buffer size */
		alsa_buf_sz = TDM_MAX_BUFFER_BYTES;
	}

	return snd_pcm_lib_preallocate_pages_for_all(runtime->pcm,
			SNDRV_DMA_TYPE_CONTINUOUS,
			snd_dma_continuous_data(GFP_KERNEL),
			alsa_buf_sz, alsa_buf_sz);
}

/* ****************************************************************************
 * ALSA ADSP DAI register
 * ***************************************************************************/

/** callback function of platform driver */
static struct snd_soc_platform_driver snd_adsp_platform = {
	.pcm_new	= &snd_adsp_pcm_new, /* PRQA S 1053 *//* PRQA S 0674 */
	.ops		= &snd_adsp_pcm_ops,
};

/** component information of driver */
static const struct snd_soc_component_driver snd_adsp_component = {
	.name		= "snd_adsp",		   /* PRQA S 1053 */
};

/** DAI information of ADSP ALSA driver */
static struct snd_soc_dai_driver snd_adsp_dai[MAX_DAI_IDX] = {
	{
		/* PRQA S 1053 */
		.id			= RDR_DAI_IDX0,
		.name			= "adsp-dai.0",
		.playback.stream_name   = "Playback0",
		.capture.stream_name	= "Capture0",
	},
	{
		.id			= RDR_DAI_IDX1,
		.name			= "adsp-dai.1",
		.playback.stream_name   = "Playback1",
		.capture.stream_name	= "Capture1",
	},
	{
		.id			= RDR_DAI_IDX2,
		.name			= "adsp-dai.2",
		.playback.stream_name   = "Playback2",
		.capture.stream_name	= "Capture2",
	},
	{
		.id			= RDR_DAI_IDX3,
		.name			= "adsp-dai.3",
		.playback.stream_name   = "Playback3",
		.capture.stream_name	= "Capture3",
	},
	{
		.id			= TDM_DAI_IDX,
		.name			= "adsp-tdm-dai",
		.playback.stream_name   = "TDM Playback",
		.capture.stream_name	= "TDM Capture",
	}
};

/** ***************************************************************************
 *  \brief	 Register platform driver and ADSP ALSA sound card
 *  \internal
 *  \covers: DD_DRV_ALSA_01_006
 *
 *  \param[in]	  pdev		Pointer to platform driver data
 *  \retval	  0		Success
 *  \retval	 -ENOMEM	Cannot allocate driver's data
 *  \retval	 -EINVAL	Cannot register platform driver or sound card
 ****************************************************************************/
static int snd_adsp_probe(struct platform_device *pdev)
{
	struct snd_adsp_card *adsp_card;
	int i;

	/* allocate a card data structure */
	adsp_card = kmalloc(sizeof(*adsp_card), GFP_KERNEL);
	if (!adsp_card)
		return -ENOMEM;

	/* init parameters */
	memset(adsp_card, 0, sizeof(*adsp_card)); /* PRQA S 3200 */

	/* PRQA S 3200 1*/
	memset(&adsp_card->ctr_if, -1, sizeof(struct snd_adsp_control));

	/* disable Equalizer for all streams */
	for (i = 0; i < (MAX_DAI_IDX - 1); i++) {
		adsp_card->ctr_if.eqz_switch[DIRECT_CAPTURE][i] = 0;
		adsp_card->ctr_if.eqz_switch[DIRECT_PLAYBACK][i] = 0;
	}

	/* disable MIX function for all */
	adsp_card->ctr_if.mix_usage = MIX_UNUSED;

	/* save driver data */
	dev_set_drvdata(&pdev->dev, adsp_card);

	/* register platform device */
	if (snd_soc_register_platform(&pdev->dev, &snd_adsp_platform) < 0) {
		snd_soc_unregister_platform(&pdev->dev);
		return -EINVAL;
	}

	/* fill format information of sound DAI driver for Rdr/Cap function */
	for (i = 0; i < (MAX_DAI_IDX - 1); i++) {
		snd_adsp_dai[i].playback.rates = SND_ADSP_SAMPLE_RATES;
		snd_adsp_dai[i].playback.formats = SND_ADSP_PCM_WIDTHS;
		snd_adsp_dai[i].playback.channels_min = MIN_CHANNEL;
		snd_adsp_dai[i].playback.channels_max = MAX_CHANNEL;

		snd_adsp_dai[i].capture.rates = SND_ADSP_SAMPLE_RATES;
		snd_adsp_dai[i].capture.formats = SND_ADSP_PCM_WIDTHS;
		snd_adsp_dai[i].capture.channels_min = MIN_CHANNEL;
		snd_adsp_dai[i].capture.channels_max = MAX_CHANNEL;
	}

	/* fill format information of sound DAI driver for TDM function */
	snd_adsp_dai[i].playback.rates = SND_ADSP_SAMPLE_RATES;
	snd_adsp_dai[i].playback.formats = SND_ADSP_PCM_WIDTHS;
	snd_adsp_dai[i].playback.channels_min = TDM_MIN_CHANNEL;
	snd_adsp_dai[i].playback.channels_max = TDM_MAX_CHANNEL;

	snd_adsp_dai[i].capture.rates = SND_ADSP_SAMPLE_RATES;
	snd_adsp_dai[i].capture.formats = SND_ADSP_PCM_WIDTHS;
	snd_adsp_dai[i].capture.channels_min = TDM_MIN_CHANNEL;
	snd_adsp_dai[i].capture.channels_max = TDM_MAX_CHANNEL;

	/* register CPU dai */
	if (snd_soc_register_component(&pdev->dev, &snd_adsp_component,
				       snd_adsp_dai,
				       ARRAY_SIZE(snd_adsp_dai)) < 0) {
		snd_soc_unregister_platform(&pdev->dev);
		return -EINVAL;
	}

	dev_info(&pdev->dev, "probed\n");

	/* success */
	return 0;
}

/** **************************************************************************
 *  \brief	 Unregister platform driver and ADSP ALSA sound card
 *  \internal
 *  \covers: DD_DRV_ALSA_01_007
 *
 *  \param[in]	pdev		Pointer platform driver data
 *  \retval	0		Success
 *  \retval	-EINVAL		Invalid driver's data
 *****************************************************************************/
static int snd_adsp_remove(struct platform_device *pdev)
{
	/* get ADSP sound card data */
	struct snd_adsp_card *adsp_card = dev_get_drvdata(&pdev->dev);

	if (!adsp_card)
		return -ENODEV;

	/* release the ADSP sound card */
	kfree(adsp_card);

	/* unregister platform driver */
	snd_soc_unregister_component(&pdev->dev);
	snd_soc_unregister_platform(&pdev->dev);

	/* success */
	return 0;
}

/** ADSP ALSA driver information */
static const struct of_device_id snd_adsp_id[] = {
	{ .compatible = "renesas,rcar_adsp_sound_gen3", },  /* PRQA S 1053 */
};
MODULE_DEVICE_TABLE(of, snd_adsp_id);

/** platform driver of ADSP ALSA sound card */
static struct platform_driver snd_adsp_driver = {
	.driver	= {		 /* PRQA S 1053 */
		.name	= "rcar_adsp_sound",
		.of_match_table = snd_adsp_id,
	},
	.probe		= snd_adsp_probe,
	.remove		= snd_adsp_remove,
};
module_platform_driver(snd_adsp_driver);		/* PRQA S 0651 */

MODULE_AUTHOR("Renesas AudioDSP");
MODULE_DESCRIPTION("Platform driver for ADSP");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ADSP-PCM-AUDIO");
