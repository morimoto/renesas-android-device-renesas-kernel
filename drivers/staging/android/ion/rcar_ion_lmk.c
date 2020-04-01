/*
 * Copyright (C) 2020 GlobalLogic
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
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/genalloc.h>
#include <linux/slab.h>
#include <linux/sched/task.h>
#include <linux/sched/signal.h>
#include <linux/fdtable.h>
#include <linux/dma-buf.h>
#include <linux/uaccess.h>
#include <linux/list.h>

#include "ion.h"
#include "../uapi/rcar_ion_lmk.h"

#define DMABUF_FNAME_MAGIC "dmabuf"
#define ION_EXP_MAGIC "ion"
#define DEVICE_NAME "rcar_ion_lmk"

struct ion_consumers {
	struct ion_proc_info info;
	struct list_head node;
};

static LIST_HEAD(consumers_list);
static DEFINE_MUTEX(consumers_list_lock);

static void free_consumers_info(void)
{
	struct ion_consumers *entry, *e;

	list_for_each_entry_safe(entry, e, &consumers_list, node) {
		list_del(&entry->node);
		kfree(entry);
	}
}

static size_t update_consumers_info(void)
{
	struct task_struct *task;
	struct file *file;
	size_t consumers_num = 0;
	size_t max_fds;
	size_t ion_mem_consumed;
	int fd;

	free_consumers_info();

	for_each_process(task) {
		struct files_struct *files = get_files_struct(task);

		if (!files)
			continue;

		ion_mem_consumed = 0;

		spin_lock(&files->file_lock);

		max_fds = files_fdtable(files)->max_fds;
		for (fd = 0; fd < max_fds; fd++) {
			file = fcheck_files(files, fd);
			if (!file)
				continue;

			get_file_rcu(file);

			if (!strncmp(file->f_path.dentry->d_name.name,
				     DMABUF_FNAME_MAGIC,
				     strlen(DMABUF_FNAME_MAGIC))) {
				struct dma_buf *dmabuf = file->private_data;

				if (!strncmp(dmabuf->exp_name, ION_EXP_MAGIC,
				     strlen(ION_EXP_MAGIC)))
					ion_mem_consumed += dmabuf->size;
			}

			fput_atomic(file);
		}

		if (ion_mem_consumed) {
			struct ion_consumers *new_elem =
			    kcalloc(1, sizeof(*new_elem), GFP_KERNEL);

			if (!new_elem) {
				pr_err("%s: Failed to allocate memory for a new element\n",
				       __func__);
				free_consumers_info();
				spin_unlock(&files->file_lock);
				put_files_struct(files);
				return consumers_num;
			}
			new_elem->info.mem = ion_mem_consumed;
			new_elem->info.pid = task->pid;
			memcpy(&new_elem->info.comm[0], task->comm,
			       TASK_COMM_LEN);
			list_add_tail(&new_elem->node, &consumers_list);
			consumers_num++;
		}
		spin_unlock(&files->file_lock);
		put_files_struct(files);
	}

	return consumers_num;
}

static long ion_get_consumers_info(char __user *u_buf)
{
	struct ion_proc_info_buf buf;
	struct ion_consumers *entry;
	struct ion_proc_info *cpy_addr;
	size_t consumers_num = 0;
	int ret = 0;

	if (copy_from_user(&buf, u_buf, sizeof(buf))) {
		ret = -EINVAL;
		goto exit;
	}

	cpy_addr = buf.info;

	list_for_each_entry(entry, &consumers_list, node) {
		if (consumers_num > buf.size) {
			pr_err("Not enough space in user buffer, coried only %zu elements\n",
			       consumers_num);
			ret = -EINVAL;
			break;
		}
		if (copy_to_user(cpy_addr, &entry->info, sizeof(entry->info))) {
			ret = -EINVAL;
			break;
		}
		cpy_addr++;
		consumers_num++;
	}

exit:
	return ret;
}

static long ion_lmk_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	char __user *u_buf = (void *)arg;
	size_t consumers_num;
	int ret = 0;

	mutex_lock(&consumers_list_lock);

	switch (cmd) {
	case ION_IOC_GET_ION_CONSUMERS_СNТR:
		consumers_num = update_consumers_info();
		ret = put_user(consumers_num, (size_t *)u_buf);
		break;
	case ION_IOC_GET_ION_CONSUMERS_INFO:
		ret = ion_get_consumers_info(u_buf);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&consumers_list_lock);

	return ret;
}

static const struct file_operations rcar_ion_lmk_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = ion_lmk_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= ion_lmk_ioctl,
#endif
};

static struct miscdevice *dev;

static int __init rcar_ion_lmk_init(void)
{
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->minor = MISC_DYNAMIC_MINOR;
	dev->name = DEVICE_NAME;
	dev->fops = &rcar_ion_lmk_fops;

	ret = misc_register(dev);
	if (ret) {
		pr_err("%s: failed to register misc device.\n", __func__);
		kfree(dev);
		return ret;
	}

	return 0;
}

module_init(rcar_ion_lmk_init);

static void __exit rcar_ion_lmk_exit(void)
{
	free_consumers_info();
	misc_deregister(dev);
	kfree(dev);
}
module_exit(rcar_ion_lmk_exit);

MODULE_AUTHOR("Leonid Komaryanskiy <leonid.komaryanskiy@globallogic.com>");
MODULE_DESCRIPTION("Controlling ION memory consumption");
MODULE_LICENSE("GPL v2");
