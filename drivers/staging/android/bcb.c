/*
 * bcb.c: Reboot hook to write bootloader commands to
 *        the Android bootloader control block
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Andrew Boie <andrew.p.boie at intel.com>
 *
 * Copyright (C) 2019 GlobalLogic
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/blkdev.h>
#include <linux/reboot.h>
#include <linux/crc32.h>
#include <linux/bootreason.h>

/* Persistent area written by Android recovery console and Linux bcb driver
 * reboot hook for communication with the bootloader. Bootloader must
 * gracefully handle this area being unitinitailzed */
struct bootloader_message {
	/* Directive to the bootloader on what it needs to do next.
	 * Possible values:
	 *   boot-NNN - Automatically boot into label NNN
	 *   bootonce-NNN - Automatically boot into label NNN, clearing this
	 *     field afterwards
	 *   anything else / garbage - Boot default label */
	char command[32];

	/* Storage area for error codes when using the BCB to boot into special
	 * boot targets, e.g. for baseband update. Not used here. */
	char status[32];

	/* Area for recovery console to stash its command line arguments
	 * in case it is reset and the cache command file is erased.
	 * Not used here. */
	char recovery[768];
	char stage[32];
	char reserved[1184];
};

#ifdef CONFIG_BOOT_REASON
static char *shutdown_reason;
static char *recovery_command;

void shutdown_reason_setup(char *s)
{
	shutdown_reason = s;
}

void recovery_command_setup(char *s)
{
	recovery_command = s;
}
#endif

/* TODO: device names/partition numbers are unstable. Add support for looking
 * by GPT partition UUIDs */
static char *bootdev = "mmcblk0";
module_param(bootdev, charp, S_IRUGO);
MODULE_PARM_DESC(bootdev, "Block device for bootloader communication");

/* NOTE: This partition number MUST be synchronized with struct
 * oem_part_info oem_partition_table[FASTBOOT_OEM_PARTITIONS] in U-Boot! */
static int partno = 1;
module_param(partno, int, S_IRUGO);
MODULE_PARM_DESC(partno, "Partition number for bootloader communication");

static int device_match(struct device *dev, const void *data)
{
	if (strcmp(dev_name(dev), bootdev) == 0)
		return 1;
	return 0;
}

static struct block_device *get_emmc_bdev(void)
{
	struct block_device *bdev;
	struct device *disk_device;

	disk_device = class_find_device(&block_class, NULL, NULL, device_match);
	if (!disk_device) {
		pr_err("bcb: device %s not found!\n", bootdev);
		return NULL;
	}
	bdev = bdget_disk(dev_to_disk(disk_device), partno);
	if (!bdev) {
		dev_err(disk_device, "bcb: unable to get disk (%s,%d)\n",
				bootdev, partno);
		return NULL;
	}
	/* Note: this bdev ref will be freed after first
	   bdev_get/bdev_put cycle */
	return bdev;
}

static u64 last_lba(struct block_device *bdev)
{
	if (!bdev || !bdev->bd_inode)
		return 0;
	return div_u64(bdev->bd_inode->i_size,
		       bdev_logical_block_size(bdev)) - 1ULL;
}

static size_t read_lba(struct block_device *bdev,
		       u64 lba, u8 *buffer, size_t count)
{
	size_t totalreadcount = 0;
	sector_t n = lba * (bdev_logical_block_size(bdev) / 512);

	if (!buffer || lba > last_lba(bdev))
		return 0;

	while (count) {
		int copied = 512;
		Sector sect;
		unsigned char *data = read_dev_sector(bdev, n++, &sect);
		if (!data)
			break;
		if (copied > count)
			copied = count;
		memcpy(buffer, data, copied);
		put_dev_sector(sect);
		buffer += copied;
		totalreadcount += copied;
		count -= copied;
	}
	return totalreadcount;
}

static size_t write_lba(struct block_device *bdev,
		       u64 lba, u8 *buffer, size_t count)
{
	size_t totalwritecount = 0;
	sector_t n = lba * (bdev_logical_block_size(bdev) / 512);

	if (!buffer || lba > last_lba(bdev))
		return 0;

	while (count) {
		int copied = 512;
		Sector sect;
		unsigned char *data = read_dev_sector(bdev, n++, &sect);
		if (!data)
			break;
		if (copied > count)
			copied = count;
		memcpy(data, buffer, copied);
		set_page_dirty(sect.v);
		unlock_page(sect.v);
		put_dev_sector(sect);
		buffer += copied;
		totalwritecount += copied;
		count -= copied;
	}
	sync_blockdev(bdev);
	return totalwritecount;
}

static int bcb_reboot_notifier_call(
		struct notifier_block *notifier,
		unsigned long what, void *data)
{
	int ret = NOTIFY_DONE;
	char *cmd = (char *)data;
	struct block_device *bdev = NULL;
	struct bootloader_message *bcb = NULL;

	if (what != SYS_RESTART || !data)
		goto out;

	bdev = get_emmc_bdev();
	if (!bdev)
		goto out;

	/* make sure the block device is open rw */
	if (blkdev_get(bdev, FMODE_READ | FMODE_WRITE, NULL) < 0) {
		pr_err("bcb: blk_dev_get failed!\n");
		goto out;
	}

	bcb = kmalloc(sizeof(*bcb), GFP_KERNEL);
	if (!bcb) {
		pr_err("bcb: out of memory\n");
		goto out;
	}

	if (read_lba(bdev, 0, (u8 *)bcb, sizeof(*bcb)) != sizeof(*bcb)) {
		pr_err("bcb: couldn't read bootloader control block\n");
		goto out;
	}

	/* When the bootloader reads this area, it will null-terminate it
	* and check if it matches any existing boot labels */
	snprintf(bcb->command, sizeof(bcb->command), "bootonce-%s", cmd);

#ifdef CONFIG_BOOT_REASON
	if (recovery_command)
		strncpy(bcb->recovery, recovery_command, sizeof(bcb->recovery));
#endif

	if (write_lba(bdev, 0, (u8 *)bcb, sizeof(*bcb)) != sizeof(*bcb)) {
		pr_err("bcb: couldn't write bootloader control block\n");
		goto out;
	}

	ret = NOTIFY_OK;
out:
	kfree(bcb);
	if (bdev)
		blkdev_put(bdev, FMODE_READ | FMODE_WRITE);

	return ret;
}

#ifdef CONFIG_BOOT_REASON
static int bcb_shutdown_notifier_call(
		struct notifier_block *notifier,
		unsigned long what, void *data)
{
	const size_t REASON_MAX_LEN = 128;
	int ret = NOTIFY_DONE;
	char *cmd = (char *)data;
	struct block_device *bdev = NULL;
	struct bootloader_message *bcb = NULL;
	struct bootreason_message msg;

	if (what != SYS_POWER_OFF)
		goto out;

	bdev = get_emmc_bdev();
	if (!bdev)
		goto out;

	/* make sure the block device is open rw */
	if (blkdev_get(bdev, FMODE_READ | FMODE_WRITE, NULL) < 0) {
		pr_err("bcb: blk_dev_get failed!\n");
		goto out;
	}

	bcb = kmalloc(sizeof(*bcb), GFP_KERNEL);
	if (!bcb) {
		pr_err("bcb: out of memory\n");
		goto out;
	}

	if (read_lba(bdev, 0, (u8 *)bcb, sizeof(*bcb)) != sizeof(*bcb)) {
		pr_err("bcb: couldn't read bootloader control block\n");
		goto out;
	}

	/* Write only to reserved[] part of this structure */
	memset(bcb->reserved, 0, sizeof(bcb->reserved));

	memset(&msg, 0, sizeof(struct bootreason_message));
	/* Set default 'shutdown,' reason (trailing comma is intentional) */
	snprintf(msg.reason, sizeof(msg.reason), "%s", "shutdown,");
	/* Override shutdown reason if any */
	if (cmd) {
		if (strlen(cmd) && strlen(cmd) < REASON_MAX_LEN) {
			if (strncmp(cmd, "userrequested",
					strlen("userrequested")) == 0) {
				memset(msg.reason, 0, sizeof(msg.reason));
				snprintf(msg.reason, sizeof(msg.reason), "%s",
						"shutdown,userrequested");
			}
			/* Add some mapping here... */
		}
	} else if (shutdown_reason) {
		snprintf(msg.reason, sizeof(msg.reason), "%s", shutdown_reason);
		shutdown_reason = NULL;
	}

	/* Calculate crc32 */
	msg.crc = crc32(0 ^ 0xffffffff, msg.reason,
			sizeof(msg.reason)) ^ 0xffffffff;
	/* Copy bootreason struct */
	memcpy(bcb->reserved, &msg, sizeof(struct bootreason_message));

	if (write_lba(bdev, 0, (u8 *)bcb, sizeof(*bcb)) != sizeof(*bcb)) {
		pr_err("bcb: couldn't write bootloader control block\n");
		goto out;
	}

	ret = NOTIFY_OK;
out:
	kfree(bcb);
	if (bdev)
		blkdev_put(bdev, FMODE_READ | FMODE_WRITE);

	return ret;
}
#endif

static struct notifier_block bcb_reboot_notifier = {
	.notifier_call = bcb_reboot_notifier_call,
};

#ifdef CONFIG_BOOT_REASON
static struct notifier_block bcb_shutdown_notifier = {
	.notifier_call = bcb_shutdown_notifier_call,
};
#endif

static int __init bcb_init(void)
{
	if (partno < 1) {
		pr_err("bcb: partition number not specified\n");
		return -1;
	}
	if (register_reboot_notifier(&bcb_reboot_notifier)) {
		pr_err("bcb: unable to register reboot notifier\n");
		return -1;
	}
#ifdef CONFIG_BOOT_REASON
	if (register_reboot_notifier(&bcb_shutdown_notifier)) {
		pr_err("bcb: unable to register shutdown notifier\n");
		unregister_reboot_notifier(&bcb_reboot_notifier);
		return -1;
	}
#endif
	pr_info("bcb: writing commands to (%s,%d)\n",
			bootdev, partno);
	return 0;
}
module_init(bcb_init);

static void __exit bcb_exit(void)
{
	unregister_reboot_notifier(&bcb_reboot_notifier);
#ifdef CONFIG_BOOT_REASON
	unregister_reboot_notifier(&bcb_shutdown_notifier);
#endif
}
module_exit(bcb_exit);

MODULE_AUTHOR("Andrew Boie <andrew.p.boie at intel.com>");
MODULE_DESCRIPTION("bootloader communication module");
MODULE_LICENSE("GPL v2");
