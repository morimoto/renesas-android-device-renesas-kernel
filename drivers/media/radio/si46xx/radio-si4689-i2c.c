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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/videodev2.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-device.h>

#include "radio-si4689.h"

#define DRIVER_NAME "radio-si4689"
#define DRIVER_DESC "I2C radio driver for Si4689 AM/FM/DAB Radio Receiver"
#define DRIVER_CARD "Silicon Labs Si4689 AM/FM/DAB Radio Receiver"

#define v4l2_dev_to_radio(d) container_of(d, struct si4689_device, v4l2_dev)

static const char *pup_states_names[] = {
    "out of reset",
    "reserved",
    "bootloader",
    "application"
};

static const struct v4l2_frequency_band bands[] = {
    {
        .type = V4L2_TUNER_RADIO,
        .index = 0,
        .capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_STEREO |
                V4L2_TUNER_CAP_RDS | V4L2_TUNER_CAP_RDS_BLOCK_IO |
                V4L2_TUNER_CAP_FREQ_BANDS |
                V4L2_TUNER_CAP_HWSEEK_BOUNDED |
                V4L2_TUNER_CAP_HWSEEK_WRAP,
        .rangelow   = FM_FREQ_RANGE_LOW,
        .rangehigh  = FM_FREQ_RANGE_HIGH,
        .modulation = V4L2_BAND_MODULATION_FM,
    },
    {
        .type = V4L2_TUNER_RADIO,
        .index = 1,
        .capability = V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_FREQ_BANDS,
        .rangelow   = AM_FREQ_RANGE_LOW,
        .rangehigh  = AM_FREQ_RANGE_HIGH,
        .modulation = V4L2_BAND_MODULATION_AM,
    },
};

static int si46xx_init_fm(struct si4689_device *rdev);
static int si46xx_init_am(struct si4689_device *rdev);
static int si46xx_tune_freq(struct si4689_device *rdev, uint32_t khz, uint16_t antcap);
static int si46xx_tune_wait(struct si4689_device *rdev, int timeout);
static int si46xx_rsq_status(struct si4689_device *rdev);
static int si46xx_seek_start(struct si4689_device *rdev, uint8_t up, uint8_t wrap);
static int si46xx_fm_rds_status(struct si4689_device *rdev);

/* ------------------------------------------------------------------ */
static ssize_t si4689_fops_read(struct file *file, char __user *buf,
        size_t count, loff_t *ppos)
{
    struct si4689_device *rdev = video_drvdata(file);
    int ret;

    if (count != sizeof(struct si46xx_rds_data))
        return -EINVAL;

    if (rdev->mode != SI46XX_MODE_FM)
        return 0;

    ret = si46xx_fm_rds_status(rdev);
    if (!ret && rdev->rds.sync != 0) {
        memcpy(buf, &rdev->rds.data, count);
        return count;
    }
    return ret;
}

static unsigned int si4689_fops_poll(struct file *file,
        struct poll_table_struct *pts)
{
    return v4l2_ctrl_poll(file, pts);
}

int si4689_fops_open(struct file *file)
{
    return v4l2_fh_open(file);
}

int si4689_fops_release(struct file *file)
{
    return v4l2_fh_release(file);
}

static const struct v4l2_file_operations si4689_fops = {
    .owner                      = THIS_MODULE,
    .read                       = si4689_fops_read,
    .poll                       = si4689_fops_poll,
    .unlocked_ioctl             = video_ioctl2,
    .open                       = si4689_fops_open,
    .release                    = si4689_fops_release,
};

/* ------------------------------------------------------------------ */
int si4689_vidioc_querycap(struct file *file, void *priv,
        struct v4l2_capability *capability)
{
    strlcpy(capability->driver, DRIVER_NAME, sizeof(capability->driver));
    strlcpy(capability->card, DRIVER_CARD, sizeof(capability->card));

    capability->device_caps = V4L2_CAP_HW_FREQ_SEEK | V4L2_CAP_READWRITE |
        V4L2_CAP_TUNER | V4L2_CAP_RADIO | V4L2_CAP_RDS_CAPTURE;

    capability->capabilities = capability->device_caps | V4L2_CAP_DEVICE_CAPS;

    return 0;
}

static int si4689_vidioc_g_tuner(struct file *file, void *priv,
        struct v4l2_tuner *tuner)
{
    struct si4689_device *rdev = video_drvdata(file);

    if (tuner->index != 0)
        return -EINVAL;

    tuner->signal = rdev->rssi;

    /* FIXME: To implement correct stereo signal detection */
    tuner->audmode = rdev->rssi > 0 ?
                V4L2_TUNER_MODE_STEREO : V4L2_TUNER_MODE_MONO;
    return 0;
}

static int si4689_vidioc_g_frequency(struct file *file, void *priv,
            struct v4l2_frequency *freq)
{
    struct si4689_device *rdev = video_drvdata(file);

    if (freq->tuner != 0)
        return -EINVAL;

    freq->frequency = rdev->freq_khz;
    return 0;
}

static int si4689_vidioc_s_frequency(struct file *file, void *priv,
            const struct v4l2_frequency *freq)
{
    struct si4689_device *rdev = video_drvdata(file);
    int ret;

    if (freq->tuner != 0)
        return -EINVAL;

    if (file->f_flags & O_NONBLOCK)
        return -EWOULDBLOCK;

    ret = si46xx_tune_freq(rdev, freq->frequency, 0);
    if (ret) {
        dev_err(&rdev->client->dev, "Tune frequency %u failed: %d\n",
            freq->frequency, ret);
        return ret;
    }

    ret = si46xx_tune_wait(rdev, 500 /*0.5 sec*/);
    if (ret) {
        dev_err(&rdev->client->dev, "Wait for tune frequency %u failed: %d\n",
            freq->frequency, ret);
        return ret;
    }

    si46xx_rsq_status(rdev);

    return 0;
}

static int si4689_vidioc_s_hw_freq_seek(struct file *file, void *priv,
            const struct v4l2_hw_freq_seek *seek)
{
    struct si4689_device *rdev = video_drvdata(file);
    int ret;

    dev_dbg(&rdev->client->dev, "%s\n", __func__);

    if (seek->tuner != 0)
        return -EINVAL;

    if (file->f_flags & O_NONBLOCK)
        return -EWOULDBLOCK;

    ret = si46xx_seek_start(rdev, seek->seek_upward, seek->wrap_around);
    if (ret) {
        dev_err(&rdev->client->dev, "Seeek failed: %d\n", ret);
        return ret;
    }

    ret = si46xx_tune_wait(rdev, 2000 /*2 sec*/);
    if (ret) {
        dev_err(&rdev->client->dev, "Wait for seek failed: %d\n", ret);
        return ret;
    }

    si46xx_rsq_status(rdev);

    return 0;
}

static int si4689_vidioc_enum_freq_bands(struct file *file, void *priv,
            struct v4l2_frequency_band *band)
{
    if (band->tuner != 0 || band->index >= ARRAY_SIZE(bands))
        return -EINVAL;

    *band = bands[band->index];
    return 0;
}

static int si4689_vidioc_s_ctrl(struct file *file, void *priv,
            struct v4l2_control *ctrl)
{
    struct si4689_device *rdev = video_drvdata(file);
    int ret = -ENOSYS;

    if (ctrl->id != V4L2_TUNER_RADIO)
        return -EINVAL;

    if (file->f_flags & O_NONBLOCK)
        return -EWOULDBLOCK;

    /* Init FM band mode */
    if (ctrl->value == V4L2_BAND_MODULATION_FM) {
        ret = si46xx_init_fm(rdev);
        if (ret) {
            dev_err(&rdev->client->dev, "Init FM mode failed: %d\n", ret);
            return ret;
        }
    /* Init AM band mode */
    } else if (ctrl->value == V4L2_BAND_MODULATION_AM) {
        ret = si46xx_init_am(rdev);
        if (ret) {
            dev_err(&rdev->client->dev, "Init AM mode failed: %d\n", ret);
            return ret;
        }
    }

    return ret;
}

static const struct v4l2_ioctl_ops si4689_ioctl_ops = {
    .vidioc_querycap            = si4689_vidioc_querycap,
    .vidioc_g_tuner             = si4689_vidioc_g_tuner,
    .vidioc_g_frequency         = si4689_vidioc_g_frequency,
    .vidioc_s_frequency         = si4689_vidioc_s_frequency,
    .vidioc_s_hw_freq_seek      = si4689_vidioc_s_hw_freq_seek,
    .vidioc_enum_freq_bands     = si4689_vidioc_enum_freq_bands,
    .vidioc_s_ctrl              = si4689_vidioc_s_ctrl,
    .vidioc_subscribe_event     = v4l2_ctrl_subscribe_event,
    .vidioc_unsubscribe_event   = v4l2_event_unsubscribe,
};

/* ------------------------------------------------------------------ */
static void si4689_v4l2_release(struct v4l2_device *v4l2_dev)
{
    struct si4689_device *rdev = v4l2_dev_to_radio(v4l2_dev);

    v4l2_device_unregister(&rdev->v4l2_dev);
    kfree(rdev);
}

/* ------------------------------------------------------------------ */
static int si46xx_i2c_xfer(struct si4689_device *rdev,
    void *buf, size_t tx_len, size_t rx_len)
{
    int cnt = (rx_len > 0) ? 2 : 1;

    struct i2c_msg msgs[2] = {
        {
            .addr = rdev->client->addr,
            .flags = 0,
            .len = tx_len,
            .buf = (void *)buf
        },
        {
            .addr = rdev->client->addr,
            .flags = I2C_M_RD,
            .len = rx_len,
            .buf = (void *)buf
        },
    };

    if (i2c_transfer(rdev->client->adapter, msgs, cnt) != cnt)
        return -EIO;

    return 0;
}

static int si46xx_read_status(struct si4689_device *rdev,
    char *buf, size_t len)
{
    struct device *dev = &rdev->client->dev;
    int ret, timeout = 2000;

    if(!buf || len < 4) {
        return -EINVAL;
    }

    while(--timeout)
    {
        buf[0] = SI46XX_RD_REPLY;

        ret = si46xx_i2c_xfer(rdev, buf, 1, len);
        if (ret) {
            dev_err(dev, "%s: SI46XX_RD_REPLY failed: %d\n", __func__, ret);
            return ret;
        }

        if (buf[0] & 0x80) {
            return 0;
        }

        if (buf[0] & 0x40) {
            return -EIO;
        }

        msleep(1);
    }

    dev_err(dev, "%s: Timeout waiting for CTS\n", __func__);
    return -ETIME;
}

static int si46xx_check_status(struct si4689_device *rdev)
{
    struct device *dev = &rdev->client->dev;
    int ret;
    char buf[4];

    ret = si46xx_read_status(rdev, buf, sizeof(buf));
    if (ret) {
        dev_err(dev, "%s: read status failed: %d\n", __func__, ret);
        return ret;
    }

    if (buf[0] & (1 << 6)) {
        ret = -1;

        dev_err(dev, "ERR_CMD: %02x %02x %02x %02x\n",
            buf[0], buf[1], buf[2], buf[3]);

        dev_err(dev, "PUP_STATE: %s\n", pup_states_names[buf[3] >> 6]);

        if (buf[3] & (1 << 3)) {
            dev_err(dev, "REPOFERR (reply too fast)\n");
            ret = -EBUSY;
        }
        if (buf[3] & (1 << 2)) {
            dev_err(dev, "CMDOFERR (CMD too fast)\n");
            ret = -EAGAIN;
        }
        if (buf[3] & (1 << 1)) {
            dev_err(dev, "ARBERR (arbiter error)\n");
            ret = -EIO;
        }
        if (buf[3] & (1 << 0)) {
            dev_err(dev, "ERRNR (non-recoverable error)\n");
            ret = -EINVAL;
        }
    }
    return ret;
}

static int si46xx_get_sys_mode(struct si4689_device *rdev)
{
    struct device *dev = &rdev->client->dev;
    int ret;
    uint8_t buf[6] = {SI46XX_GET_SYS_STATE, 0, 0, 0, 0, 0};

    ret = si46xx_i2c_xfer(rdev, &buf, 2, 0);
    if (ret) {
        dev_err(dev, "%s: SI46XX_GET_SYS_STATE failed: %d\n", __func__, ret);
        return ret;
    }

    buf[0] = SI46XX_RD_REPLY;

    ret = si46xx_i2c_xfer(rdev, &buf, 1, 6);
    if (ret) {
        dev_err(dev, "%s: SI46XX_RD_REPLY failed: %d\n", __func__, ret);
        return ret;
    }

    if (ret)
        return ret;

    switch(buf[4])
    {
        case 0:
            dev_info(dev, "%s: Bootloader is active\n", __func__);
            return SI46XX_MODE_BOOT;
        case 1:
        case 4:
            dev_info(dev, "%s: FMHD is active\n", __func__);
            return SI46XX_MODE_FM;
        case 2:
            dev_info(dev, "%s: DAB is active\n", __func__);
            return SI46XX_MODE_DAB;;
        case 3:
            dev_info(dev, "%s: TDMB or data only DAB image is active\n", __func__);
            return SI46XX_MODE_DAB;
        case 5:
            dev_info(dev, "%s: AMHD is active\n", __func__);
            return SI46XX_MODE_AM;
        case 6:
            dev_info(dev, "%s: AMHD Demod is active\n", __func__);
            return SI46XX_MODE_AM;
    }

    dev_info(dev, "%s: Unknown mode\n", __func__);
    return SI46XX_MODE_UNK;
}

static int si46xx_powerup(struct si4689_device *rdev)
{
    int ret;
    uint8_t data[16];

    dev_dbg(&rdev->client->dev, "%s\n", __func__);

    data[0] = SI46XX_POWER_UP;
    data[1] = 0x80; // ARG1
    data[2] = (1<<4) | (7<<0); // ARG2 CLK_MODE=0x1 TR_SIZE=0x7
    data[3] = 0x48; // ARG3 IBIAS=0x28
    data[4] = 0x00; // ARG4 XTAL
    data[5] = 0xF9; // ARG5 XTAL // F8
    data[6] = 0x24; // ARG6 XTAL
    data[7] = 0x01; // ARG7 XTAL 19.2MHz
    data[8] = 0x1F; // ARG8 CTUN
    data[9] = 0x00 | (1<<4); // ARG9
    data[10] = 0x00; // ARG10
    data[11] = 0x00; // ARG11
    data[12] = 0x00; // ARG12
    data[13] = 0x00; // ARG13 IBIAS_RUN
    data[14] = 0x00; // ARG14
    data[15] = 0x00; // ARG15

    ret = si46xx_i2c_xfer(rdev, data, 16, 0);
    if (ret)
        return ret;

    msleep(1); // wait 20us after powerup (datasheet)

    return si46xx_check_status(rdev);
}

static int si46xx_load_init(struct si4689_device *rdev)
{
    int ret;
    char data[2];

    dev_dbg(&rdev->client->dev, "%s\n", __func__);

    data[0] = SI46XX_LOAD_INIT;
    data[1] = 0;

    ret = si46xx_i2c_xfer(rdev, data, 2, 0);
    if (ret)
        return ret;

    msleep(4); // wait 4ms (datasheet)

    return si46xx_check_status(rdev);
}

static int si46xx_host_load_data(struct si4689_device *rdev,
    const uint8_t *buf, size_t len)
{
    int ret;
    size_t payload_size = len + 4;
    uint8_t *payload = kzalloc(payload_size, GFP_KERNEL);

    if (!payload) {
        dev_err(&rdev->client->dev, "%s: Allocate payload buffer (%zu bytes) failed\n",
            __func__, payload_size);
        return -ENOMEM;
    }

    payload[0] = SI46XX_HOST_LOAD;
    payload[1] = 0;
    payload[2] = 0;
    payload[3] = 0;

    memcpy(payload + 4, buf, len);

    ret = si46xx_i2c_xfer(rdev, payload, payload_size, 0);

    kfree(payload);
    return ret;
}

#define FW_LOAD_SIZE  4096
static int si46xx_upload_firmware_patch(struct si4689_device *rdev)
{
    const struct firmware *fw_p;
    const char *fw_name = "si46xx/patch.bin";
    struct device *dev = &rdev->client->dev;
    int ret, remaining_bytes, count_to;

    dev_dbg(dev, "%s\n", __func__);

    ret = si46xx_load_init(rdev);
    if (ret) {
        dev_err(dev, "LOAD_INIT failed: %d\n", ret);
        return ret;
    }

    ret = request_firmware(&fw_p, fw_name, dev);
    if (ret) {
        dev_err(dev, "%s - %s not found\n", __func__, fw_name);
        return ret;
    }

    dev_info(dev, "Loading FW: %s (%zu bytes)\n", fw_name, fw_p->size);

    remaining_bytes = fw_p->size;
    while(remaining_bytes) {
        if(remaining_bytes >= FW_LOAD_SIZE) {
            count_to = FW_LOAD_SIZE;
        } else {
            count_to = remaining_bytes;
        }

        si46xx_host_load_data(rdev, fw_p->data + (fw_p->size - remaining_bytes), count_to);

        remaining_bytes -= count_to;
    }

    msleep(4); // wait 4ms (datasheet)

    ret = si46xx_check_status(rdev);
    if (ret) {
        dev_err(dev, "FW load error: %d\n", ret);
        goto out;
    }

    msleep(4); // wait 4ms (datasheet)

    dev_info(dev, "%s: FW (%s) loaded\n", __func__, fw_name);
out:
    release_firmware(fw_p);
    return ret;
}

static int si46xx_flash_load(struct si4689_device *rdev, uint32_t offset)
{
    int ret;
    uint8_t cmd[16];

    dev_info(&rdev->client->dev, "%s: offset=0x%x\n", __func__, offset);

    cmd[0] = SI46XX_FLASH_LOAD;
    cmd[1] = 0;
    cmd[2] = 0;
    cmd[3] = 0;
    cmd[4] = (offset & 0xff);
    cmd[5] = (offset >> 8) & 0xff;
    cmd[6] = (offset >> 16) & 0xff;
    cmd[7] = (offset >> 24) & 0xff;
    cmd[8] = 0;
    cmd[9] = 0;
    cmd[10] = 0;
    cmd[11] = 0;

    ret = si46xx_i2c_xfer(rdev, cmd, 12, 0);
    if (ret)
        return ret;

    return si46xx_check_status(rdev);
}

static int si46xx_print_part_info(struct si4689_device *rdev)
{
    int ret;
    char buf[22];
    struct device *dev = &rdev->client->dev;

    buf[0] = SI46XX_GET_PART_INFO;
    buf[1] = 0;

    ret = si46xx_i2c_xfer(rdev, buf, 2, 0);
    if (ret) {
        dev_err(dev, "SI46XX_GET_PART_INFO failed: %d\n", ret);
        return ret;
    }

    ret = si46xx_read_status(rdev, buf, sizeof(buf));
    if (ret) {
        return ret;
    }

    dev_info(dev, "CHIPREV:\t0x%02x\n", buf[4]);
    dev_info(dev, "ROMID:\t0x%02x\n", buf[5]);
    dev_info(dev, "PART:\t%04d\n", (buf[9] << 8) | buf[8]);
    return 0;
}

static int si46xx_boot(struct si4689_device *rdev)
{
    int ret, attempts = 5;
    char cmd[2];

    dev_dbg(&rdev->client->dev, "%s\n", __func__);

    do {
        cmd[0] = SI46XX_BOOT;
        cmd[1] = 0;

        ret = si46xx_i2c_xfer(rdev, cmd, 2, 0);
        if (ret)
            return ret;

        msleep(300); // 63ms at analog fm, 198ms at DAB

        ret = si46xx_check_status(rdev);
    } while ((attempts--) && (ret));

    return ret;
}

static int si46xx_boot_flash(struct si4689_device *rdev, int offset)
{
    struct device *dev = &rdev->client->dev;
    int ret;

    dev_info(dev, "Boot from FLASH, offset 0x%x\n", offset);

    ret = si46xx_load_init(rdev);
    if (ret) {
        dev_err(dev, "LOAD_INIT failed: %d\n", ret);
        return ret;
    }

    ret = si46xx_flash_load(rdev, offset);
    if (ret) {
        dev_err(dev, "FLASH_LOAD failed: %d\n", ret);
        return ret;
    }

    ret = si46xx_boot(rdev);
    if (ret) {
        dev_err(dev, "BOOT failed: %d\n", ret);
        return ret;
    }

    return 0;
}

static int si46xx_seek_start(struct si4689_device *rdev, uint8_t up, uint8_t wrap)
{
    uint8_t cmd[6];
    int ret;

    dev_dbg(&rdev->client->dev, "%s: up=%d, wrap=%d\n", __func__, (int)up, (int)wrap);

    if (rdev->mode == SI46XX_MODE_AM)
        cmd[0] = SI46XX_AM_SEEK_START;
    else if (rdev->mode == SI46XX_MODE_FM)
        cmd[0] = SI46XX_FM_SEEK_START;
    else
        return -EINVAL;

    cmd[1] = 0;
    cmd[2] = (up & 0x01)<<1 | (wrap & 0x01);
    cmd[3] = 0;
    cmd[4] = 0;
    cmd[5] = 0;

    ret = si46xx_i2c_xfer(rdev, cmd, 6, 0);
    if (ret) {
        dev_err(&rdev->client->dev, "SI46XX_x_SEEK_START failed: %d\n", ret);
        return ret;
    }

    return si46xx_check_status(rdev);
}

int si46xx_set_property(struct si4689_device *rdev, uint16_t property_id, uint16_t value)
{
    int ret;
    uint8_t cmd[6];

    dev_dbg(&rdev->client->dev, "%s(id=0x%04x, value=0x%04x)\n",
        __func__, property_id, value);

    cmd[0] = SI46XX_SET_PROPERTY;
    cmd[1] = 0;
    cmd[2] = property_id & 0xFF;
    cmd[3] = (property_id >> 8) & 0xFF;
    cmd[4] = value & 0xFF;
    cmd[5] = (value >> 8) & 0xFF;

    ret = si46xx_i2c_xfer(rdev, cmd, 6, 0);
    if (ret) {
        dev_err(&rdev->client->dev, "SI46XX_SET_PROPERTY failed: %d\n", ret);
        return ret;
    }

    ret = si46xx_check_status(rdev);
    if (ret) {
        dev_err(&rdev->client->dev, "si46xx_set_property(0x%02x, 0x%02x) failed: %d\n",
            property_id, value, ret);
    }

    return ret;
}

static int si46xx_tune_freq(struct si4689_device *rdev, uint32_t khz, uint16_t antcap)
{
    int ret;
    uint8_t cmd[6];

    dev_dbg(&rdev->client->dev, "%s(khz=%d, antcap=%d)\n", __func__, khz, antcap);

    if (rdev->mode == SI46XX_MODE_AM) {
        cmd[0] = SI46XX_AM_TUNE_FREQ;
        khz *= 10;
    }
    else if (rdev->mode == SI46XX_MODE_FM)
        cmd[0] = SI46XX_FM_TUNE_FREQ;
    else {
        dev_err(&rdev->client->dev, "%s: Invalid mode %d\n",
            __func__, rdev->mode);
        return -EINVAL;
    }

    cmd[1] = 0;
    cmd[2] = ((khz/10) & 0xFF);
    cmd[3] = ((khz/10) >> 8) & 0xFF;
    cmd[4] = antcap & 0xFF;
    cmd[5] = 0;

    if (rdev->mode == SI46XX_MODE_AM)
        cmd[5] = (antcap >> 8) & 0xFF;

    ret = si46xx_i2c_xfer(rdev, cmd, 6, 0);
    if (ret) {
        dev_err(&rdev->client->dev, "SI46XX_x_TUNE_FREQ failed: %d\n", ret);
        return ret;
    }

    ret = si46xx_check_status(rdev);
    if (ret) {
        dev_err(&rdev->client->dev, "si46xx_tune_freq(khz=%d, antcap=%d) failed: %d\n",
            khz, antcap, ret);
    }

    return ret;
}

static int si46xx_tune_wait(struct si4689_device *rdev, int timeout)
{
    int ret;
    char status[5];

    do {
        ret = si46xx_read_status(rdev, status, sizeof(status));
        if (ret)
            return ret;

        if (status[0] & (1 << 0))
            return 0;

        msleep(1);
    } while (--timeout > 0);

    return -ETIME;
}

static int si46xx_rsq_status(struct si4689_device *rdev)
{
    struct device *dev = &rdev->client->dev;
    int ret, to_read;
    char cmd[20];

    dev_dbg(dev, "%s\n", __func__);

    if (rdev->mode == SI46XX_MODE_AM) {
        cmd[0] = SI46XX_AM_RSQ_STATUS;
        to_read = 16;
    } else if (rdev->mode == SI46XX_MODE_FM) {
        cmd[0] = SI46XX_FM_RSQ_STATUS;
        to_read = 20;
    } else {
        dev_err(dev, "%s EINVAL\n", __func__);
        return -EINVAL;
    }

    rdev->freq_khz = 0;
    rdev->rssi = 0;
    rdev->snr = 0;

    cmd[1] = 0;

    ret = si46xx_i2c_xfer(rdev, cmd, 2, 0);
    if (ret) {
        dev_err(dev, "SI46XX_x_RSQ_STATUS failed: %d\n", ret);
        return ret;
    }

    ret = si46xx_read_status(rdev, cmd, to_read);
    if (ret) {
        dev_err(dev, "Read status failed: %d\n", ret);
        return ret;
    }

    rdev->freq_khz = (cmd[7] << 8) | cmd[6];

    if (rdev->mode == SI46XX_MODE_FM)
        rdev->freq_khz *= 10;

    rdev->rssi = (signed char)cmd[9];
    rdev->snr = (signed char)cmd[10];

    dev_info(dev, "SNR:        %d dB\n", rdev->snr);
    dev_info(dev, "RSSI:       %d dBuV\n", rdev->rssi);
    dev_info(dev, "Frequency:  %dkHz\n", rdev->freq_khz);

    dev_info(dev, "FREQOFF:    %d\n", cmd[8]*2);
    dev_info(dev, "READANTCAP: %d\n", (cmd[12] | (cmd[13] << 8)));

    if (rdev->mode == SI46XX_MODE_AM) {
        dev_info(dev, " AM modulaton: %d%%\n", cmd[11]);
        dev_info(dev, " HDLEVEL:      %d%%\n", cmd[15]);
    }

    dev_dbg(dev, "%s <-\n", __func__);
    return 0;
}

static uint8_t si46xx_rds_parse(struct si4689_device *rdev, uint16_t *block)
{
    uint8_t addr;

    rdev->rds.data.pi = block[0];

    if ((block[1] & 0xF800) == 0x00) { // group 0A
        addr = block[1] & 0x03;

        rdev->rds.data.ps_name[addr*2] = (block[3] & 0xFF00)>>8;
        rdev->rds.data.ps_name[addr*2+1] = block[3] & 0xFF;

        rdev->rds.group_0a_flags |= (1 << addr);

    } else if ((block[1] & 0xF800) >> 11 == 0x04) { // group 2A
        addr = block[1] & 0x0F;

        if ((block[1] & 0x10) == 0x00) { // parse only string A
            rdev->rds.data.radiotext[addr*4]   = (block[2] & 0xFF00) >> 8;
            rdev->rds.data.radiotext[addr*4+1] = (block[2] & 0xFF);
            rdev->rds.data.radiotext[addr*4+2] = (block[3] & 0xFF00) >> 8;
            rdev->rds.data.radiotext[addr*4+3] = (block[3] & 0xFF);

            if (rdev->rds.data.radiotext[addr*4] == '\r') {
                rdev->rds.data.radiotext[addr*4] = 0;
                rdev->rds.group_2a_flags = 0xFFFF;
            }

            if (rdev->rds.data.radiotext[addr*4+1] == '\r') {
                rdev->rds.data.radiotext[addr*4+1] = 0;
                rdev->rds.group_2a_flags = 0xFFFF;
            }

            if (rdev->rds.data.radiotext[addr*4+2] == '\r') {
                rdev->rds.data.radiotext[addr*4+2] = 0;
                rdev->rds.group_2a_flags = 0xFFFF;
            }

            if (rdev->rds.data.radiotext[addr*4+3] == '\r') {
                rdev->rds.data.radiotext[addr*4+3] = 0;
                rdev->rds.group_2a_flags = 0xFFFF;
            }

            rdev->rds.group_2a_flags |= (1 << addr);
        }
    }

    if (rdev->rds.group_0a_flags == 0x0F && rdev->rds.group_2a_flags == 0xFFFF) {
        rdev->rds.data.ps_name[8] = 0;
        rdev->rds.data.radiotext[128] = 0;
        return 1;
    }

    return 0;
}

static int si46xx_fm_rds_status(struct si4689_device *rdev)
{
    struct device *dev = &rdev->client->dev;
    int ret, timeout = 2000;
    char buf[21];
    uint16_t blocks[4];

    dev_dbg(dev, "%s\n", __func__);

    memset(&rdev->rds, 0, sizeof(struct si46xx_fm_rds_data));

    while(--timeout) {

        buf[0] = SI46XX_FM_RDS_STATUS;
        buf[1] = 1;

        ret = si46xx_i2c_xfer(rdev, buf, 2, 0);
        if (ret) {
            dev_err(&rdev->client->dev, "SI46XX_FM_RDS_STATUS failed: %d\n", ret);
            return ret;
        }

        ret = si46xx_read_status(rdev, buf, 20);
        if (ret) {
            dev_err(&rdev->client->dev, "Read status failed: %d\n", ret);
            return ret;
        }

        blocks[0] = buf[12] + (buf[13]<<8);
        blocks[1] = buf[14] + (buf[15]<<8);
        blocks[2] = buf[16] + (buf[17]<<8);
        blocks[3] = buf[18] + (buf[19]<<8);

        rdev->rds.sync = (buf[5] & 0x02) ? 1 : 0;

        if (!rdev->rds.sync)
            break;

        mdelay(1);

        if (si46xx_rds_parse(rdev, blocks))
            break;

        if (rdev->rds.group_0a_flags == 0x0F) // stop at ps_name complete
            break;
    }

    if (!timeout) {
        dev_warn(&rdev->client->dev, "Timeout wait for RDS data sync.\n");
        return -ETIME;
    }

    dev_dbg(dev, "RDSSYNC: %u\n", (buf[5] & 0x02) ? 1 : 0);
    dev_dbg(dev, "PI: %d  Name:%s\n", rdev->rds.data.pi, rdev->rds.data.ps_name);
    return 0;
}

static int si46xx_init_boot_mode(struct si4689_device *rdev)
{
    int ret;

    /* Reset the chip */
    gpio_set_value_cansleep(rdev->reset_gpio, 0);
    msleep(10);
    gpio_set_value_cansleep(rdev->reset_gpio, 1);
    mdelay(100);

    rdev->mode = si46xx_get_sys_mode(rdev);

    if (rdev->mode != SI46XX_MODE_BOOT) {
        if (rdev->mode == SI46XX_MODE_UNK) {
            ret = si46xx_powerup(rdev);
            if (ret) {
                dev_err(&rdev->client->dev, "Power up failed: %d\n", ret);
                return ret;
            }
        }

        ret = si46xx_upload_firmware_patch(rdev);
        if (ret) {
            dev_err(&rdev->client->dev, "Patch load failed: %d\n", ret);
            return ret;
        }
    }

    si46xx_print_part_info(rdev);
    return 0;
}

static int si46xx_init_fm(struct si4689_device *rdev)
{
    int ret, offset = RADIO_FWFLASH_OFFSET_FM;

    if (rdev->mode == SI46XX_MODE_FM) /* Already in desired mode */
        return 0;

    ret = si46xx_init_boot_mode(rdev);
    if (ret) {
        dev_err(&rdev->client->dev, "si46xx_init_boot_mode failed: %d\n", ret);
        return ret;
    }

    ret = si46xx_boot_flash(rdev, offset);
    if (ret) {
        dev_err(&rdev->client->dev,
            "si46xx_boot_flash FM mode failed: %d\n" \
            "FLASH offset 0x%x is programmed?\n", ret, offset);
        return ret;
    }

    rdev->mode = si46xx_get_sys_mode(rdev);

    if (rdev->mode != SI46XX_MODE_FM) {
        dev_err(&rdev->client->dev,
            "Invalid sys mode. FLASH offset 0x%x is programmed?\n", offset);
        return -EINVAL;
    }

    /* enable I2S output */
    si46xx_set_property(rdev, SI46XX_PIN_CONFIG_ENABLE, 0x0003);
    si46xx_set_property(rdev, SI46XX_FM_SOFTMUTE_SNR_LIMITS, 0x0000); // set the SNR limits for soft mute attenuation
    si46xx_set_property(rdev, SI46XX_FM_TUNE_FE_CFG, 0x0000); // front end switch open
    si46xx_set_property(rdev, SI46XX_FM_SEEK_BAND_BOTTOM, 88000 / 10);
    si46xx_set_property(rdev, SI46XX_FM_SEEK_BAND_TOP, 108000 / 10);

    //si46xx_set_property(rdev, SI46XX_DIGITAL_IO_OUTPUT_SAMPLE_RATE, 48000);

    /* I2S master */
    si46xx_set_property(rdev, SI46XX_DIGITAL_IO_OUTPUT_SELECT, 0x8000);

    /* I2S slave */
    //si46xx_set_property(rdev, SI46XX_DIGITAL_IO_OUTPUT_SELECT, 0x0);

    si46xx_set_property(rdev, SI46XX_DIGITAL_IO_OUTPUT_FORMAT,
        (16 << 8) |     // sample size 16
        (4 << 4) |      // slot size 16
        (0 << 0));      // right_j mode

    si46xx_set_property(rdev, SI46XX_FM_RDS_CONFIG, 0x0001); // enable RDS
    si46xx_set_property(rdev, SI46XX_FM_AUDIO_DE_EMPHASIS, SI46XX_AUDIO_DE_EMPHASIS_EU); // set de-emphasis for Europe

    return 0;
}

static int si46xx_init_am(struct si4689_device *rdev)
{
    int ret, offset = RADIO_FWFLASH_OFFSET_AM;

    if (rdev->mode == SI46XX_MODE_AM) /* Already in desired mode */
        return 0;

    if (rdev->mode == SI46XX_MODE_AM) {
        dev_info(&rdev->client->dev, "si46xx_init_fm: already active AM mode\n");
        return 0;
    }

    ret = si46xx_init_boot_mode(rdev);
    if (ret) {
        dev_err(&rdev->client->dev, "si46xx_init_boot_mode failed: %d\n", ret);
        return ret;
    }

    ret = si46xx_boot_flash(rdev, offset);
    if (ret) {
        dev_err(&rdev->client->dev,
            "si46xx_boot_flash AM mode failed: %d\n" \
            "FLASH offset 0x%x is programmed?\n", ret, offset);
        return ret;
    }

    rdev->mode = si46xx_get_sys_mode(rdev);

    if (rdev->mode != SI46XX_MODE_AM) {
        dev_err(&rdev->client->dev,
            "Invalid sys mode. FLASH offset 0x%x is programmed?\n", offset);
        return -EINVAL;
    }

    /* enable I2S output */
    si46xx_set_property(rdev, SI46XX_PIN_CONFIG_ENABLE, 0x0003);

    si46xx_set_property(rdev, SI46XX_AM_SEEK_FREQUENCY_SPACING, 1);
    si46xx_set_property(rdev, SI46XX_AM_SEEK_BAND_BOTTOM, 500);
    si46xx_set_property(rdev, SI46XX_AM_SEEK_BAND_TOP, 1700);
    si46xx_set_property(rdev, SI46XX_AM_VALID_RSSI_THRESHOLD, 15);
    si46xx_set_property(rdev, SI46XX_AM_VALID_SNR_THRESHOLD, 2);

    //si46xx_set_property(rdev, SI46XX_DIGITAL_IO_OUTPUT_SAMPLE_RATE, 48000);

    /* I2S master */
    si46xx_set_property(rdev, SI46XX_DIGITAL_IO_OUTPUT_SELECT, 0x8000);

    /* I2S slave */
    //si46xx_set_property(rdev, SI46XX_DIGITAL_IO_OUTPUT_SELECT, 0x0);

    si46xx_set_property(rdev, SI46XX_DIGITAL_IO_OUTPUT_FORMAT,
        (16 << 8) |     // sample size 16
        (4 << 4) |      // slot size 16
        (0 << 0));      // right_j mode

    return 0;
}

/* ------------------------------------------------------------------ */
static int si4689_i2c_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
    struct si4689_device *rdev;
    struct v4l2_device *v4l2_dev;
    struct device_node *np = client->dev.of_node;
    int ret = 0;

    /* private data allocation and initialization */
    rdev = kzalloc(sizeof(struct si4689_device), GFP_KERNEL);
    if (!rdev) {
        ret = -ENOMEM;
        goto err_alloc;
    }

    /* */
    v4l2_dev = &rdev->v4l2_dev;
    v4l2_dev->release = si4689_v4l2_release;

    ret = v4l2_device_register(&client->dev, v4l2_dev);
    if (ret < 0) {
        v4l2_err(v4l2_dev, "couldn't register v4l2_device\n");
        goto err_radio;
    }

    of_property_read_u32(np, "reset_gpio", (u32 *)&rdev->reset_gpio);

    ret = gpio_request(rdev->reset_gpio, "si4689_reset");
    if (unlikely(ret)) {
        dev_err(&client->dev, "gpio %d request failed ", rdev->reset_gpio);
        goto err_gpio_req;
    }

    ret = gpio_direction_output(rdev->reset_gpio, 0);
    if (unlikely(ret)) {
        dev_err(&client->dev, "unable to configure gpio %d", rdev->reset_gpio);
        goto err_all;
    }

    rdev->client = client;
    rdev->mode = SI46XX_MODE_UNK;

    mutex_init(&rdev->lock);
    init_completion(&rdev->completion);

    /* video device initialization */
    strcpy(rdev->videodev.name, DRIVER_NAME);

    rdev->videodev.v4l2_dev    = v4l2_dev;
    rdev->videodev.fops        = &si4689_fops;
    rdev->videodev.ioctl_ops   = &si4689_ioctl_ops;
    rdev->videodev.release     = video_device_release_empty;

    video_set_drvdata(&rdev->videodev, rdev);

    /* register video device */
    ret = video_register_device(&rdev->videodev, VFL_TYPE_RADIO, -1);
    if (ret) {
        dev_warn(&client->dev, "Could not register video device, err=%d\n", ret);
        goto err_all;
    }

    i2c_set_clientdata(client, rdev);

    dev_info(&client->dev, "successfully registered");
    return 0;

err_all:
    gpio_free(rdev->reset_gpio);

err_gpio_req:
    v4l2_device_unregister(&rdev->v4l2_dev);

err_radio:
    kfree(rdev);

err_alloc:
    return ret;
}

static int si4689_i2c_remove(struct i2c_client *client)
{
    struct si4689_device *rdev = i2c_get_clientdata(client);

    gpio_set_value_cansleep(rdev->reset_gpio, 0);
    gpio_free(rdev->reset_gpio);

    video_unregister_device(&rdev->videodev);
    kfree(rdev);

    return 0;
}


#ifdef CONFIG_PM_SLEEP
static int si4689_i2c_suspend(struct device *dev)
{
    return 0;
}

static int si4689_i2c_resume(struct device *dev)
{
    return 0;
}

static SIMPLE_DEV_PM_OPS(si4689_i2c_pm, si4689_i2c_suspend, si4689_i2c_resume);
#endif


/* I2C Device ID List */
static const struct i2c_device_id si4689_i2c_id[] = {
    { "si4689", 0 },
    { /* End of list */ }
};
MODULE_DEVICE_TABLE(i2c, si4689_i2c_id);

/*
 * si4689_i2c_driver - i2c driver interface
 */
static struct i2c_driver si4689_i2c_driver = {
    .driver = {
        .name       = "si4689",
#ifdef CONFIG_PM_SLEEP
        .pm     = &si4689_i2c_pm,
#endif
    },
    .probe          = si4689_i2c_probe,
    .remove         = si4689_i2c_remove,
    .id_table       = si4689_i2c_id,
};

module_i2c_driver(si4689_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("oleksii.gulchenko@globallogic.com");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION("0.1");
