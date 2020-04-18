/*
 * Copyright (C) 2018 GlobalLogic
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
#include <linux/keyreset.h>
#include <linux/reboot.h>
#include <linux/of.h>

#define DEFAULT_KEY_DOWN_DELAY 10000

int keylongpress_fn(void)
{
	orderly_poweroff(true);
	return 0;
}

static struct keyreset_platform_data keylongpress_pdata = {
	.keys_down = {
		KEY_POWER,
		0
	},
	.reset_fn = keylongpress_fn,
	.key_down_delay = DEFAULT_KEY_DOWN_DELAY,
};

static struct platform_device keylongpress_device = {
	.name = KEYRESET_NAME,
	.dev.platform_data = &keylongpress_pdata,
};

static int keylongpress_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *name;

	device_property_read_u32(dev, "key_down_delay",
			&keylongpress_pdata.key_down_delay);

	platform_device_register(&keylongpress_device);

	pr_info("keylongpress: button code %d, delay %d ms\n",
			keylongpress_pdata.keys_down[0], keylongpress_pdata.key_down_delay);
	return 0;
}

static const struct of_device_id keylongpress_of_match[] = {
	{ .compatible = "key-long-press", },
	{ },
};
MODULE_DEVICE_TABLE(of, keylongpress_of_match);

static struct platform_driver keylongpress_driver = {
	.probe		= keylongpress_probe,
	.driver		= {
		.name   = "keylongpress",
		.of_match_table = keylongpress_of_match,
	}
};

static int __init keylongpress_init(void)
{
	return platform_driver_register(&keylongpress_driver);
}

static void __exit keylongpress_exit(void)
{
	platform_driver_unregister(&keylongpress_driver);
	platform_device_unregister(&keylongpress_device);
}

module_init(keylongpress_init);
module_exit(keylongpress_exit);

MODULE_AUTHOR("Dmytro Prokopchuk <dmytro.prokopchuk@globallogic.com>");
MODULE_DESCRIPTION("Long press button driver");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");
