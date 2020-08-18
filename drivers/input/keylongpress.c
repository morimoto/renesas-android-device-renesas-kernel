/*
 * Copyright (C) 2018 - 2020 GlobalLogic
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/reboot.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/bootreason.h>

#define DEFAULT_KEY_DOWN_DELAY (2*HZ)
#define KEYLONGPRESS_NAME "keylongpress"

struct keylongpress_task {
	unsigned long keybits[BITS_TO_LONGS(KEY_CNT)];
	unsigned int delay;
	int target_pressed;
	int currently_pressed;
	const char *action;
	struct delayed_work wait_key_down;

};

struct keylongpress_data {
	struct input_handler handler;
	spinlock_t lock;
	int tasks_count;
	struct keylongpress_task *active;
	struct keylongpress_task tasks[];
};


static void do_action(struct work_struct *work)
{
	struct delayed_work *dwork =
		container_of(work, struct delayed_work, work);
	struct keylongpress_task *task =
		container_of(dwork, struct keylongpress_task, wait_key_down);

	if (!strcmp("reboot", task->action)) {
		kernel_restart("android");
	} else if (!strcmp("recovery", task->action)) {
		kernel_restart("recovery");
	} else if (!strcmp("fastboot", task->action)) {
#ifdef CONFIG_BOOT_REASON
		recovery_command_setup("recovery\n--fastboot\n");
#endif
		kernel_restart("recovery");
	} else {
		pr_err("Keylongpress unknown acton %s\n", task->action);
	}
}

static void stop_task(struct keylongpress_data *priv, struct keylongpress_task *task)
{
	if (task != priv->active)
		return;

	cancel_delayed_work(&task->wait_key_down);
	priv->active = NULL;
}

static void start_task(struct keylongpress_data *priv, struct keylongpress_task *task)
{
	if (priv->active == task)
		return;

	if (!priv->active) {
		schedule_delayed_work(&task->wait_key_down, task->delay);
		priv->active = task;
		return;
	}

	if (task->target_pressed > priv->active->target_pressed) {
		stop_task(priv, priv->active);
		schedule_delayed_work(&task->wait_key_down, task->delay);
		priv->active = task;
	}
}

static void keylongpress_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	struct keylongpress_task *task;
	struct keylongpress_data *priv = handle->private;
	int i;

	if (type != EV_KEY)
		return;

	spin_lock(&priv->lock);
	for (i = 0; i < priv->tasks_count; i++) {
		task = &priv->tasks[i];

		if (test_bit(code, task->keybits)) {
			if (value)
				task->currently_pressed++;
			else
				task->currently_pressed--;
		}
		if (task->currently_pressed == task->target_pressed)
			start_task(priv, task);
		else
			stop_task(priv, task);
	}
	spin_unlock(&priv->lock);
}

static int keylongpress_connect(struct input_handler *handler,
		struct input_dev *dev,
		const struct input_device_id *id)
{
	int ret;
	struct input_handle *handle;
	struct keylongpress_data *priv = container_of(handler,
			struct keylongpress_data, handler);

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = KEYLONGPRESS_NAME;
	handle->private = priv;

	ret = input_register_handle(handle);
	if (ret)
		goto err;
	ret = input_open_device(handle);
	if (ret)
		goto err;

	return 0;
err:
	pr_err("keylongpress: Unable to connect input device: %d\n", ret);
	kfree(handle);
	return ret;
}

static void keylongpress_disconnect(struct input_handle *handle)
{
	input_unregister_handle(handle);
	input_close_device(handle);
	kfree(handle);
}

static const struct input_device_id device_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) }
	},
	{ }
}
MODULE_DEVICE_TABLE(input, device_ids);

static int keylongpress_probe(struct platform_device *pdev)
{
	struct keylongpress_data *priv;
	struct device *dev = &pdev->dev;
	struct device_node *child;
	int num_children, ret;
	int key, keycount;
	int count = 0;
	int i;

	num_children = of_get_available_child_count(dev->of_node);
	priv = devm_kzalloc(dev, sizeof(priv) +
			sizeof(priv->tasks[0]) * num_children, GFP_KERNEL);
	if (!priv) {
		pr_err("Failed to allocate memory for keylongpress_data\n");
		return -ENOMEM;
	}

	priv->tasks_count = num_children;

	for_each_child_of_node(dev->of_node, child) {
		keycount = of_property_count_u32_elems(child, "keys");

		if (keycount < 1) {
			dev_err(dev, "Invalid keylongpress definiton %s: can't read keys\n", child->name);
			continue;
		}
		priv->tasks[count].target_pressed = keycount;

		for (i = 0; i < keycount; i++) {
			ret = of_property_read_u32_index(child, "keys", i,
					&key);
			if (ret) {
				dev_err(dev, "Invalid keylongpress definiton %s: can't read keys\n", child->name);
				continue;
			}
			__set_bit(key, priv->tasks[count].keybits);
		}

		ret = of_property_read_u32(child, "timeout-ms",
				&priv->tasks[count].delay);
		if (ret)
			priv->tasks[count].delay = DEFAULT_KEY_DOWN_DELAY;
		else
			priv->tasks[count].delay = msecs_to_jiffies(priv->tasks[count].delay);

		ret = of_property_read_string(child, "action",
				&priv->tasks[count].action);
		if (ret) {
			dev_err(dev, "Invalid keylongpress definiton %s: can't read action\n", child->name);
			continue;
		}

		INIT_DELAYED_WORK(&priv->tasks[count].wait_key_down, do_action);
		count++;
	}
	priv->tasks_count = count;
	spin_lock_init(&priv->lock);

	priv->handler.name = KEYLONGPRESS_NAME;
	priv->handler.id_table = device_ids;
	priv->handler.event = keylongpress_event;
	priv->handler.connect = keylongpress_connect;
	priv->handler.disconnect = keylongpress_disconnect;

	ret = input_register_handler(&priv->handler);
	if (ret) {
		dev_err(dev, "Unable to register input handler\n");
		goto err;
	}

	platform_set_drvdata(pdev, priv);
	return 0;
err:
	return ret;
}

static int keylongpress_remove(struct platform_device *pdev)
{
	struct keylongpress_data *priv = platform_get_drvdata(pdev);
	int i;

	input_unregister_handler(&priv->handler);
	for (i = 0; i < priv->tasks_count; i++)
		cancel_delayed_work_sync(&priv->tasks[i].wait_key_down);
	return 0;
}

static const struct of_device_id keylongpress_of_match[] = {
	{
		.compatible = KEYLONGPRESS_NAME,
	},
	{ },
};

static struct platform_driver keylongpress_driver = {
	.driver = {
		.name = KEYLONGPRESS_NAME,
		.of_match_table = keylongpress_of_match,
	},
	.probe = keylongpress_probe,
	.remove = keylongpress_remove
};

static int __init keylongpress_init(void)
{
	return platform_driver_register(&keylongpress_driver);
}

static void __exit keylongpress_exit(void)
{
	return platform_driver_unregister(&keylongpress_driver);
}

module_init(keylongpress_init);
module_exit(keylongpress_exit);

MODULE_AUTHOR("Mykyta Poturai <mykyta.poturai@globallogic.com>");
MODULE_DESCRIPTION("Long press button driver");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");
