/*
 * bootreason.c: Store reboot reason into RAM
 *
 * Copyright (C) 2019 GlobalLogic
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
#include <linux/kernel.h>
#include <linux/reboot.h>
#include <linux/memblock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <asm/page.h>
#include <linux/crc32.h>
#include <linux/bootreason.h>

static const char *DEV_NAME = "rambootreason";
static const size_t REASON_MAX_LEN = 128;
static size_t bootreason_mem_size;
static phys_addr_t bootreason_mem_start;
void *bootreason_mem_vaddr;
static u8 *bootreason_mem_buffer;

static char *reboot_reason;

void reboot_reason_setup(char *s)
{
	reboot_reason = s;
}

static int reboot_notifier_call(
		struct notifier_block *notifier,
		unsigned long what, void *data)
{
	int ret = NOTIFY_DONE;
	char *cmd = (char *)data;
	struct bootreason_message msg;

	if (what != SYS_RESTART && what != SYS_WATCHDOG)
		goto out;

	memset(&msg, 0, sizeof(struct bootreason_message));
	/* Set default 'reboot' reason (trailing comma is intentional) */
	snprintf(msg.reason, sizeof(msg.reason), "%s", "reboot,");
	/* Override boot reason if any */
	if (cmd) {
		if (strlen(cmd) && strlen(cmd) < REASON_MAX_LEN) {
			if (strncmp(cmd, "bootloader",
					strlen("bootloader")) == 0) {
				memset(msg.reason, 0, sizeof(msg.reason));
				snprintf(msg.reason, sizeof(msg.reason), "%s",
						"reboot,bootloader");
			} else if (strncmp(cmd, "recovery",
					strlen("recovery")) == 0) {
				memset(msg.reason, 0, sizeof(msg.reason));
				snprintf(msg.reason, sizeof(msg.reason), "%s",
						"reboot,recovery");
			} else if (strncmp(cmd, "userrequested",
					strlen("userrequested")) == 0) {
				memset(msg.reason, 0, sizeof(msg.reason));
				snprintf(msg.reason, sizeof(msg.reason), "%s",
						"reboot,userrequested");
			} else if (strncmp(cmd, "watchdog",
					strlen("watchdog")) == 0) {
				memset(msg.reason, 0, sizeof(msg.reason));
				snprintf(msg.reason, sizeof(msg.reason), "%s",
						"watchdog");
			} else if (strncmp(cmd, "adb",
					strlen("adb")) == 0) {
				memset(msg.reason, 0, sizeof(msg.reason));
				snprintf(msg.reason, sizeof(msg.reason), "%s",
						"reboot,adb");
			} else if (strncmp(cmd, "shell",
					strlen("shell")) == 0) {
				memset(msg.reason, 0, sizeof(msg.reason));
				snprintf(msg.reason, sizeof(msg.reason), "%s",
						"reboot,shell");
			}
			/* Add some mapping here... */
		}
	} else if (reboot_reason) {
		snprintf(msg.reason, sizeof(msg.reason), "%s", reboot_reason);
		reboot_reason = NULL;
	}

	/* Calculate crc32 */
	msg.crc = crc32(0 ^ 0xffffffff, msg.reason,
			sizeof(msg.reason)) ^ 0xffffffff;
	/* Write bootreason struct into RAM */
	memcpy_toio(bootreason_mem_buffer, &msg,
			sizeof(struct bootreason_message));

	ret = NOTIFY_OK;
out:
	return ret;
}

static int panic_notifier_call(
		struct notifier_block *notifier,
		unsigned long what, void *data)
{
	struct bootreason_message msg;

	memset(&msg, 0, sizeof(struct bootreason_message));
	/* Set default 'panic' reason */
	snprintf(msg.reason, sizeof(msg.reason), "%s", "kernel_panic");
	/* Calculate crc32 */
	msg.crc = crc32(0 ^ 0xffffffff, msg.reason,
			sizeof(msg.reason)) ^ 0xffffffff;
	/* Write bootreason struct into RAM */
	memcpy_toio(bootreason_mem_buffer, &msg,
			sizeof(struct bootreason_message));

	return NOTIFY_OK;
}

static struct notifier_block bootreason_reboot_notifier = {
	.notifier_call = reboot_notifier_call,
};

static struct notifier_block bootreason_panic_notifier = {
	.notifier_call	= panic_notifier_call,
};

static int __init bootreason_init(void)
{
	struct device_node *node = NULL;
	struct resource res;

	node = of_find_node_by_name(NULL, DEV_NAME);
	if (!node) {
		pr_err("bootreason: no '%s' device found in DT\n", DEV_NAME);
		return -ENODEV;
	}

	if (of_address_to_resource(node, 0, &res)) {
		pr_err("bootreason: no 'reg' resource found\n");
		return -EINVAL;
	}

	pr_info("bootreason: device '%s': start=0x%08X - end=0x%08X, size=%d\n",
			DEV_NAME, (u32)res.start, (u32)res.end, (u32)resource_size(&res));

	bootreason_mem_start = res.start;
	bootreason_mem_size = resource_size(&res);

	if (pfn_valid(bootreason_mem_start >> PAGE_SHIFT)) {
		struct page **pages = NULL;
		phys_addr_t page_start = 0;
		unsigned int page_count = 0;
		pgprot_t prot;
		unsigned int i = 0;

		page_start = bootreason_mem_start -
				offset_in_page(bootreason_mem_start);
		page_count = DIV_ROUND_UP(bootreason_mem_size +
				offset_in_page(bootreason_mem_start),
				PAGE_SIZE);

		prot = pgprot_writecombine(PAGE_KERNEL);

		pages = kmalloc_array(page_count, sizeof(struct page *),
				GFP_KERNEL);
		if (!pages) {
			pr_err("bootreason: failed to allocate array for %u pages\n",
					page_count);
			return -ENOMEM;
		}

		for (i = 0; i < page_count; i++) {
			phys_addr_t addr = page_start + i * PAGE_SIZE;

			pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
		}
		bootreason_mem_vaddr = vmap(pages, page_count, VM_MAP, prot);
		kfree(pages);

		if (!bootreason_mem_vaddr) {
			pr_err("bootreason: failed to map pages\n");
			return -ENOMEM;
		}

	} else {
		pr_err("bootreason: device '%s' does not support I/O remap\n",
				DEV_NAME);
		return -EPERM;
	}

	bootreason_mem_buffer = bootreason_mem_vaddr +
			offset_in_page(bootreason_mem_start);

	if (register_reboot_notifier(&bootreason_reboot_notifier)) {
		pr_err("bootreason: unable to register reboot notifier\n");
		goto err;
	}

	if (atomic_notifier_chain_register(&panic_notifier_list,
			&bootreason_panic_notifier)) {
		pr_err("bootreason: unable to register panic notifier\n");
		goto failed_panic;
	}

	return 0;

failed_panic:
	unregister_reboot_notifier(&bootreason_reboot_notifier);
err:
	vunmap(bootreason_mem_vaddr);
	bootreason_mem_vaddr = NULL;
	return -EPERM;

}
module_init(bootreason_init);

static void __exit bootreason_exit(void)
{
	unregister_reboot_notifier(&bootreason_reboot_notifier);
	atomic_notifier_chain_unregister(&panic_notifier_list,
			&bootreason_panic_notifier);

	vunmap(bootreason_mem_vaddr);
}
module_exit(bootreason_exit);

MODULE_AUTHOR("Dmytro Prokopchuk <dmytro.prokopchuk@globallogic.com>");
MODULE_DESCRIPTION("Storing bootreason into RAM driver");
MODULE_LICENSE("GPL v2");
