/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_BOOTREASON_H_
#define _LINUX_BOOTREASON_H_

/*
 * Message written by Linux bootreason driver.
 * Bootloader must parse it and append to Android bootargs.
 * Note: sync structure with bootloader and bcb driver.
 */
struct bootreason_message {
	char reason[128];
	u32 crc;
};

#ifdef CONFIG_BOOT_REASON
void shutdown_reason_setup(char *s);
void recovery_command_setup(char *s);
#endif
void reboot_reason_setup(char *s);

#endif /* _LINUX_BOOTREASON_H_ */
