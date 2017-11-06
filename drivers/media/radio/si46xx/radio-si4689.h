/*
 * Copyright (C) 2017 GlobalLogic
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* ------------------------------------------------------------------ */
struct si46xx_rds_data {
    uint16_t pi;
    uint8_t pty;
    char ps_name[9];
    char radiotext[129];
};

struct si46xx_fm_rds_data {
    uint8_t sync;
    uint16_t group_0a_flags;
    uint32_t group_2a_flags;
    struct si46xx_rds_data data;
};

struct si4689_device {
    struct v4l2_device v4l2_dev;
    struct video_device videodev;
    struct v4l2_ctrl_handler hdl;
    struct completion completion;
    struct i2c_client *client;
    struct mutex lock;

    int mode;
    int reset_gpio;

    int freq_khz;
    int rssi;
    int snr;

    struct si46xx_fm_rds_data rds;
};

/* ------------------------------------------------------------------ */
#define RADIO_FWFLASH_OFFSET_PATCH      0x00002000
#define RADIO_FWFLASH_OFFSET_FM         0x00006000
#define RADIO_FWFLASH_OFFSET_DAB        0x00086000
#define RADIO_FWFLASH_OFFSET_AM         0x00106000

/* ------------------------------------------------------------------ */
#define SI46XX_MODE_UNK                 0
#define SI46XX_MODE_BOOT                1
#define SI46XX_MODE_AM                  2
#define SI46XX_MODE_FM                  3
#define SI46XX_MODE_DAB                 4

/* ------------------------------------------------------------------ */
#define SI46XX_RD_REPLY                                     0x00
#define SI46XX_POWER_UP                                     0x01
#define SI46XX_HOST_LOAD                                    0x04
#define SI46XX_FLASH_LOAD                                   0x05
#define SI46XX_LOAD_INIT                                    0x06
#define SI46XX_BOOT                                         0x07

#define SI46XX_GET_PART_INFO                                0x08
#define SI46XX_GET_SYS_STATE                                0x09
#define SI46XX_SET_PROPERTY                                 0x13
#define SI46XX_GET_PROPERTY                                 0x14

#define SI46XX_FM_TUNE_FREQ                                 0x30
#define SI46XX_FM_SEEK_START                                0x31
#define SI46XX_FM_RSQ_STATUS                                0x32
#define SI46XX_FM_ACF_STATUS                                0x33
#define SI46XX_FM_RDS_STATUS                                0x34
#define SI46XX_FM_RDS_BLOCKCOUNT                            0x35

#define SI46XX_DAB_TUNE_FREQ                                0xB0
#define SI46XX_DAB_DIGRAD_STATUS                            0xB2
#define SI46XX_DAB_GET_SERVICE_LINKING_INFO                 0xB7
#define SI46XX_DAB_SET_FREQ_LIST                            0xB8
#define SI46XX_DAB_GET_DIGITAL_SERVICE_LIST                 0x80
#define SI46XX_DAB_START_DIGITAL_SERVICE                    0x81
#define SI46XX_DAB_GET_ENSEMBLE_INFO                        0xB4
#define SI46XX_DAB_GET_AUDIO_INFO                           0xBD
#define SI46XX_DAB_GET_SUBCHAN_INFO                         0xBE

#define SI46XX_AM_TUNE_FREQ                                 0x40
#define SI46XX_AM_SEEK_START                                0x41
#define SI46XX_AM_RSQ_STATUS                                0x42
#define SI46XX_AM_SEEK_BAND_BOTTOM                          0x4100
#define SI46XX_AM_SEEK_BAND_TOP                             0x4101
#define SI46XX_AM_SEEK_FREQUENCY_SPACING                    0x4102
#define SI46XX_AM_VALID_RSSI_THRESHOLD                      0x4202
#define SI46XX_AM_VALID_SNR_THRESHOLD                       0x4204

#define SI46XX_FM_INT_CTL_ENABLE                            0x0000
#define SI46XX_FM_INT_CTL_REPEAT                            0x0001
#define SI46XX_DIGITAL_IO_OUTPUT_SELECT                     0x0200
#define SI46XX_DIGITAL_IO_OUTPUT_SAMPLE_RATE                0x0201
#define SI46XX_DIGITAL_IO_OUTPUT_FORMAT                     0x0202
#define SI46XX_PIN_CONFIG_ENABLE                            0x0800
#define SI46XX_FM_SEEK_BAND_BOTTOM                          0x3100
#define SI46XX_FM_SEEK_BAND_TOP                             0x3101
#define SI46XX_FM_VALID_MAX_TUNE_ERROR                      0x3200
#define SI46XX_FM_VALID_RSSI_TIME                           0x3201
#define SI46XX_FM_VALID_RSSI_THRESHOLD                      0x3202
#define SI46XX_FM_VALID_SNR_TIME                            0x3203
#define SI46XX_FM_VALID_SNR_THRESHOLD                       0x3204
#define SI46XX_FM_SOFTMUTE_SNR_LIMITS                       0x3500
#define SI46XX_FM_SOFTMUTE_SNR_ATTENUATION                  0x3501
#define SI46XX_FM_TUNE_FE_CFG                               0x1712
#define SI46XX_FM_RDS_CONFIG                                0x3C02
#define SI46XX_FM_AUDIO_DE_EMPHASIS                         0x3900

#define SI46XX_DAB_TUNE_FE_CFG                              0x1712
#define SI46XX_DAB_TUNE_FE_VARM                             0x1710
#define SI46XX_DAB_TUNE_FE_VARB                             0x1711
#define SI46XX_DAB_CTRL_DAB_MUTE_ENABLE                     0xB400
#define SI46XX_DAB_CTRL_DAB_MUTE_SIGNAL_LEVEL_THRESHOLD     0xB501
#define SI46XX_DAB_CTRL_DAB_MUTE_SIGLOW_THRESHOLD           0xB505

#define SI46XX_DIGITAL_SERVICE_INT_SOURCE                   0x8100

/* 0x00=75us -> defaults to USA (default),
   0x01=50us -> defaults to Europe,
   0x02=disabled
*/
#define SI46XX_AUDIO_DE_EMPHASIS_EU                         0x01
#define SI46XX_AUDIO_DE_EMPHASIS_US                         0x00
#define SI46XX_AUDIO_DE_EMPHASIS_OFF                        0x02

/* The supported radio frequency ranges in kHz */
#define FM_FREQ_RANGE_LOW                                   (87500U * 16U)
#define FM_FREQ_RANGE_HIGH                                  (108000U * 16U)
#define AM_FREQ_RANGE_LOW                                   (520U * 16U)
#define AM_FREQ_RANGE_HIGH                                  (1710U * 16U)

