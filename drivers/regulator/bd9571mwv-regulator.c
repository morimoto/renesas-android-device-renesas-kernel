/*
 * ROHM BD9571MWV-M regulator driver
 *
 * Copyright (C) 2017 Marek Vasut <marek.vasut+renesas@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether expressed or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License version 2 for more details.
 *
 * Based on the TPS65086 driver
 *
 * NOTE: VD09 is missing
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>

#include <linux/mfd/bd9571mwv.h>

struct bd9571mwv_reg {
	struct bd9571mwv *bd;

	/* DDR Backup Power */
	u8 bkup_mode_cnt_keepon;	/* from "rohm,ddr-backup-power" */
	u8 bkup_mode_cnt_shadow;
};

enum bd9571mwv_regulators { VD09, VD18, VD25, VD33, DVFS };

#define BD9571MWV_REG(_name, _of, _id, _ops, _vr, _vm, _nv, _min, _step, _lmin)\
	{							\
		.name			= _name,		\
		.of_match		= of_match_ptr(_of),	\
		.regulators_node	= "regulators",		\
		.id			= _id,			\
		.ops			= &_ops,		\
		.n_voltages		= _nv,			\
		.type			= REGULATOR_VOLTAGE,	\
		.owner			= THIS_MODULE,		\
		.vsel_reg		= _vr,			\
		.vsel_mask		= _vm,			\
		.min_uV			= _min,			\
		.uV_step		= _step,		\
		.linear_min_sel		= _lmin,		\
	}

static int bd9571mwv_avs_get_moni_state(struct regulator_dev *rdev)
{
	unsigned int val;
	int ret;

	ret = regmap_read(rdev->regmap, BD9571MWV_AVS_SET_MONI, &val);
	if (ret != 0)
		return ret;

	return val & BD9571MWV_AVS_SET_MONI_MASK;
}

static int bd9571mwv_avs_set_voltage_sel_regmap(struct regulator_dev *rdev,
						unsigned int sel)
{
	int ret;

	ret = bd9571mwv_avs_get_moni_state(rdev);
	if (ret < 0)
		return ret;

	return regmap_write_bits(rdev->regmap, BD9571MWV_AVS_VD09_VID(ret),
				 rdev->desc->vsel_mask, sel);
}

static int bd9571mwv_avs_get_voltage_sel_regmap(struct regulator_dev *rdev)
{
	unsigned int val;
	int ret;

	ret = bd9571mwv_avs_get_moni_state(rdev);
	if (ret < 0)
		return ret;

	ret = regmap_read(rdev->regmap, BD9571MWV_AVS_VD09_VID(ret), &val);
	if (ret != 0)
		return ret;

	val &= rdev->desc->vsel_mask;
	val >>= ffs(rdev->desc->vsel_mask) - 1;

	return val;
}

static int bd9571mwv_reg_set_voltage_sel_regmap(struct regulator_dev *rdev,
						unsigned int sel)
{
	return regmap_write_bits(rdev->regmap, BD9571MWV_DVFS_SETVID,
				 rdev->desc->vsel_mask, sel);
}

/* Operations permitted on AVS voltage regulator */
static struct regulator_ops avs_ops = {
	.set_voltage_sel	= bd9571mwv_avs_set_voltage_sel_regmap,
	.map_voltage		= regulator_map_voltage_linear,
	.get_voltage_sel	= bd9571mwv_avs_get_voltage_sel_regmap,
	.list_voltage		= regulator_list_voltage_linear,
};

/* Operations permitted on voltage regulators */
static struct regulator_ops reg_ops = {
	.set_voltage_sel	= bd9571mwv_reg_set_voltage_sel_regmap,
	.map_voltage		= regulator_map_voltage_linear,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.list_voltage		= regulator_list_voltage_linear,
};

/* Operations permitted on voltage monitors */
static struct regulator_ops vid_ops = {
	.map_voltage		= regulator_map_voltage_linear,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.list_voltage		= regulator_list_voltage_linear,
};

static struct regulator_desc regulators[] = {
	BD9571MWV_REG("VD09", "vd09", VD09, avs_ops, 0, 0x7f,
		      0x80, 600000, 10000, 0x3c),
	BD9571MWV_REG("VD18", "vd18", VD18, vid_ops, BD9571MWV_VD18_VID, 0xf,
		      16, 1625000, 25000, 0),
	BD9571MWV_REG("VD25", "vd25", VD25, vid_ops, BD9571MWV_VD25_VID, 0xf,
		      16, 2150000, 50000, 0),
	BD9571MWV_REG("VD33", "vd33", VD33, vid_ops, BD9571MWV_VD33_VID, 0xf,
		      11, 2800000, 100000, 0),
	BD9571MWV_REG("DVFS", "dvfs", DVFS, reg_ops,
		      BD9571MWV_DVFS_MONIVDAC, 0x7f,
		      0x80, 600000, 10000, 0x3c),
};

static int bd9571mwv_bkup_mode_set(struct bd9571mwv_reg *bdreg, u8 mode)
{
	int ret;

	ret = regmap_write(bdreg->bd->regmap, BD9571MWV_BKUP_MODE_CNT, mode);
	if (ret) {
		dev_err(bdreg->bd->dev,
			"Failed to configure backup mode 0x%x (ret=%i)\n",
			mode, ret);
		return ret;
	}

	bdreg->bkup_mode_cnt_shadow = mode;
	return 0;
}

static ssize_t backup_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bd9571mwv_reg *bdreg = dev_get_drvdata(dev);
	int enabled = bdreg->bkup_mode_cnt_shadow &
		      BD9571MWV_BKUP_MODE_CNT_KEEPON_MASK;

	return sprintf(buf, "%s\n", enabled ? "on" : "off");
}

static ssize_t backup_mode_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	struct bd9571mwv_reg *bdreg = dev_get_drvdata(dev);
	bool enable;
	ssize_t ret;
	u8 mode;

	if (!count)
		return 0;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	mode = bdreg->bkup_mode_cnt_shadow &
	       ~BD9571MWV_BKUP_MODE_CNT_KEEPON_MASK;
	if (enable)
		mode |= bdreg->bkup_mode_cnt_keepon;

	if (mode == bdreg->bkup_mode_cnt_shadow)
		return count;

	ret = bd9571mwv_bkup_mode_set(bdreg, mode);
	if (ret)
		return ret;

	return count;
}

DEVICE_ATTR_RW(backup_mode);

#ifdef CONFIG_PM_SLEEP
static int bd9571mwv_resume(struct device *dev)
{
	struct bd9571mwv_reg *bdreg = dev_get_drvdata(dev);
	int ret;
	unsigned int val;
	ret = regmap_read(bdreg->bd->regmap, BD9571MWV_BKUP_MODE_CNT, &val);
	if (ret) {
		dev_err(bdreg->bd->dev, "Failed to read backup mode (ret=%i)\n", ret);
		return bd9571mwv_bkup_mode_set(bdreg, bdreg->bkup_mode_cnt_shadow);
	}
	/* Restore the role of the ACC switch from a wake-up switch to a power switch.*/
	return bd9571mwv_bkup_mode_set(bdreg, val & ~BD9571MWV_BKUP_MODE_CNT_KEEPON_MASK);
}

static const struct dev_pm_ops bd9571mwv_pm  = {
	SET_SYSTEM_SLEEP_PM_OPS(NULL, bd9571mwv_resume)
};

#define DEV_PM_OPS	&bd9571mwv_pm
#else
#define DEV_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

static int bd9571mwv_regulator_probe(struct platform_device *pdev)
{
	struct bd9571mwv *bd = dev_get_drvdata(pdev->dev.parent);
	struct regulator_config config = { };
	struct bd9571mwv_reg *bdreg;
	struct regulator_dev *rdev;
	unsigned int val;
	int i, ret;

	bdreg = devm_kzalloc(&pdev->dev, sizeof(*bdreg), GFP_KERNEL);
	if (!bdreg)
		return -ENOMEM;

	bdreg->bd = bd;

	platform_set_drvdata(pdev, bdreg);

	config.dev = &pdev->dev;
	config.dev->of_node = bd->dev->of_node;
	config.driver_data = bd;
	config.regmap = bd->regmap;

	for (i = 0; i < ARRAY_SIZE(regulators); i++) {
		rdev = devm_regulator_register(&pdev->dev, &regulators[i],
					       &config);
		if (IS_ERR(rdev)) {
			dev_err(bd->dev, "failed to register %s regulator\n",
				pdev->name);
			return PTR_ERR(rdev);
		}
	}

	val = 0;
	of_property_read_u32(bd->dev->of_node, "rohm,ddr-backup-power", &val);
	if (val & ~BD9571MWV_BKUP_MODE_CNT_KEEPON_MASK) {
		dev_err(bd->dev, "invalid %s mode %u\n",
			"rohm,ddr-backup-power", val);
		return -EINVAL;
	}
	bdreg->bkup_mode_cnt_keepon = val;

	ret = regmap_read(bd->regmap, BD9571MWV_BKUP_MODE_CNT, &val);
	if (ret) {
		dev_err(bd->dev, "Failed to read backup mode (ret=%i)\n", ret);
		return ret;
	}
	bdreg->bkup_mode_cnt_shadow = val;

	return device_create_file(&pdev->dev, &dev_attr_backup_mode);
}

static int bd9571mwv_regulator_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_backup_mode);
	return 0;
}

static const struct platform_device_id bd9571mwv_regulator_id_table[] = {
	{ "bd9571mwv-regulator", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, bd9571mwv_regulator_id_table);

static struct platform_driver bd9571mwv_regulator_driver = {
	.driver = {
		.name = "bd9571mwv-regulator",
		.pm = DEV_PM_OPS,
	},
	.probe = bd9571mwv_regulator_probe,
	.remove = bd9571mwv_regulator_remove,
	.id_table = bd9571mwv_regulator_id_table,
};
module_platform_driver(bd9571mwv_regulator_driver);

MODULE_AUTHOR("Marek Vasut <marek.vasut+renesas@gmail.com>");
MODULE_DESCRIPTION("BD9571MWV Regulator driver");
MODULE_LICENSE("GPL v2");
