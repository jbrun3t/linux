// SPDX-License-Identifier: GPL-2.0
/*
 * Dummy regulator consumer test module
 *
 * Copyright (c) 2024 BayLibre, SAS.
 * Author: Jerome Brunet <jbrunet@baylibre.com>
 */
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>

static DEFINE_IDA(drc_ida);
static struct dentry *drc_dir;

struct drc_data {
	struct regulator *regulator;
	struct dentry *root;
	int id;
};

static int drc_enable_get(void *data, u64 *val)
{
	struct drc_data *priv = data;

	*val = regulator_is_enabled(priv->regulator);
	return 0;
}

static int drc_enable_set(void *data, u64 val)
{
	struct drc_data *priv = data;
	int ret = 0;

	switch (val) {
	case 0:
		ret = regulator_disable(priv->regulator);
		break;

	case 1:
		ret = regulator_enable(priv->regulator);
		break;
	}

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(drc_enable_fops,
			 drc_enable_get,
			 drc_enable_set,
			 "%llu\n");

static int drc_count_voltage_get(void *data, u64 *val)
{
	struct drc_data *priv = data;

	*val = regulator_count_voltages(priv->regulator);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(drc_count_voltage_fops,
			 drc_count_voltage_get,
			 NULL,
			 "%llu\n");

static int drc_list_voltage_set(void *data, u64 val)
{
	struct drc_data *priv = data;

	return regulator_list_voltage(priv->regulator, val);
}

DEFINE_DEBUGFS_ATTRIBUTE(drc_list_voltage_fops,
			 NULL,
			 drc_list_voltage_set,
			 "%llu\n");

static const struct of_device_id drc_of_match[] = {
	{
		.compatible = "linux,dummy-regulator-consumer",
	}, {}
};
MODULE_DEVICE_TABLE(of, drc_of_match);

static int drc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct drc_data *priv;
	char *name;
	int ret;

        priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;
	platform_set_drvdata(pdev, priv);

	priv->regulator = devm_regulator_get(dev, "dummy");
	if (IS_ERR(priv->regulator))
		dev_err_probe(dev, PTR_ERR(priv->regulator), "failed to get dummy supply\n");

	if (!drc_dir) {
		dev_err(dev, "not debugfs available\n");
		return -ENOENT;
	}

	priv->id = ida_alloc(&drc_ida, GFP_KERNEL);
	if (priv->id < 0)
		return priv->id;
		
	name = devm_kasprintf(dev, GFP_KERNEL, "drc%d", priv->id);
	if (!name) {
		ret = -ENOMEM;
		goto ida_free;
	}

	priv->root = debugfs_create_dir(name, drc_dir);
	debugfs_create_file("enable", 0600, priv->root,
			    priv, &drc_enable_fops);
	debugfs_create_file("count_voltage", 0600, priv->root,
			    priv, &drc_count_voltage_fops);
	debugfs_create_file("list_voltage", 0600, priv->root,
			    priv, &drc_list_voltage_fops);	

	return 0;

ida_free:
	ida_free(&drc_ida, priv->id);
	return ret;			  
}

static void drc_remove(struct platform_device *pdev)
{
	struct drc_data *priv = platform_get_drvdata(pdev);
	
	debugfs_remove_recursive(priv->root);
	ida_free(&drc_ida, priv->id);
}

static struct platform_driver drc_pdrv = {
	.probe	= drc_probe,
	.remove = drc_remove,
	.driver = {
		.name		= "dummy_reg_consummer",
		.of_match_table	= drc_of_match,
	},
};

static int __init drc_init(void)
{
	drc_dir = debugfs_create_dir("drc", NULL);
	if (IS_ERR(drc_dir))
		drc_dir = NULL;

	return platform_driver_register(&drc_pdrv);
}
module_init(drc_init);

static void __exit drc_exit(void)
{
	platform_driver_unregister(&drc_pdrv);
	debugfs_remove_recursive(drc_dir);
}
module_exit(drc_exit);

MODULE_AUTHOR("Jerome Brunet <jbrunet@baylibre.com>");
MODULE_DESCRIPTION("Dummy regulator test driver");
MODULE_LICENSE("GPL");
