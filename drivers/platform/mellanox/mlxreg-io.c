#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_data/mlxreg.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/* Attribute parameters. */
#define MLXREG_IO_ATT_SIZE	10
#define MLXREG_IO_ATT_NUM	48

/**
 * struct mlxreg_io_priv_data - driver's private data:
 *
 * @pdev: platform device;
 * @pdata: platform data;
 * @hwmon: hwmon device;
 * @mlxreg_io_attr: sysfs attributes array;
 * @mlxreg_io_dev_attr: sysfs sensor device attribute array;
 * @group: sysfs attribute group;
 * @groups: list of sysfs attribute group for hwmon registration;
 */
struct mlxreg_io_priv_data {
	struct platform_device *pdev;
	struct mlxreg_core_platform_data *pdata;
	struct device *hwmon;
	struct attribute *mlxreg_io_attr[MLXREG_IO_ATT_NUM + 1];
	struct sensor_device_attribute mlxreg_io_dev_attr[MLXREG_IO_ATT_NUM];
	struct attribute_group group;
	const struct attribute_group *groups[2];
};

static ssize_t
mlxreg_io_attr_show(struct device *dev, struct device_attribute *attr,
		    char *buf)
{
	struct mlxreg_io_priv_data *priv = dev_get_drvdata(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct mlxreg_core_data *data = priv->pdata->data + index;
	u32 regval = 0;
	int ret;

	ret = regmap_read(priv->pdata->regmap, data->reg, &regval);
	if (ret)
		goto access_error;

	if (!data->bit)
		regval = !!(regval & ~data->mask);

	return sprintf(buf, "%u\n", regval);

access_error:
	return ret;
}

static ssize_t
mlxreg_io_attr_store(struct device *dev, struct device_attribute *attr,
		     const char *buf, size_t len)
{
	struct mlxreg_io_priv_data *priv = dev_get_drvdata(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct mlxreg_core_data *data = priv->pdata->data + index;
	u32 val, regval;
	int ret;

	ret = kstrtou32(buf, MLXREG_IO_ATT_SIZE, &val);
	if (ret)
		return ret;

	ret = regmap_read(priv->pdata->regmap, data->reg, &regval);
	if (ret)
		goto access_error;

	regval &= data->mask;

	val = !!val;
	if (val)
		regval |= ~data->mask;
	else
		regval &= data->mask;

	ret = regmap_write(priv->pdata->regmap, data->reg, regval);
	if (ret)
		goto access_error;

	return len;

access_error:
	dev_err(&priv->pdev->dev, "Bus access error\n");
	return ret;
}

static int mlxreg_io_attr_init(struct mlxreg_io_priv_data *priv)
{
	int i;

	priv->group.attrs = devm_kzalloc(&priv->pdev->dev,
					 priv->pdata->counter *
					 sizeof(struct attribute *),
					 GFP_KERNEL);
	if (!priv->group.attrs)
		return -ENOMEM;

	for (i = 0; i < priv->pdata->counter; i++) {
		priv->mlxreg_io_attr[i] =
				&priv->mlxreg_io_dev_attr[i].dev_attr.attr;

		/* Set attribute name as a label. */
		priv->mlxreg_io_attr[i]->name =
				devm_kasprintf(&priv->pdev->dev, GFP_KERNEL,
					       priv->pdata->data[i].label);

		if (!priv->mlxreg_io_attr[i]->name) {
			dev_err(&priv->pdev->dev, "Memory allocation failed for sysfs attribute %d.\n",
				i + 1);
			return -ENOMEM;
		}

		priv->mlxreg_io_dev_attr[i].dev_attr.attr.mode =
						priv->pdata->data[i].mode;
		switch (priv->pdata->data[i].mode) {
		case 0200:
			priv->mlxreg_io_dev_attr[i].dev_attr.store =
							mlxreg_io_attr_store;
			break;

		case 0444:
			priv->mlxreg_io_dev_attr[i].dev_attr.show =
							mlxreg_io_attr_show;
			break;

		case 0644:
			priv->mlxreg_io_dev_attr[i].dev_attr.show =
							mlxreg_io_attr_show;
			priv->mlxreg_io_dev_attr[i].dev_attr.store =
							mlxreg_io_attr_store;
			break;

		default:
			dev_err(&priv->pdev->dev, "Bad access mode %u for attribute %s.\n",
				priv->pdata->data[i].mode,
				priv->mlxreg_io_attr[i]->name);
			return -EINVAL;
		}

		priv->mlxreg_io_dev_attr[i].dev_attr.attr.name =
					priv->mlxreg_io_attr[i]->name;
		priv->mlxreg_io_dev_attr[i].index = i;
		sysfs_attr_init(&priv->mlxreg_io_dev_attr[i].dev_attr.attr);
	}

	priv->group.attrs = priv->mlxreg_io_attr;
	priv->groups[0] = &priv->group;
	priv->groups[1] = NULL;

	return 0;
}

static int mlxreg_io_probe(struct platform_device *pdev)
{
	struct mlxreg_io_priv_data *priv;
	int err;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pdata = dev_get_platdata(&pdev->dev);
	if (!priv->pdata) {
		dev_err(&pdev->dev, "Failed to get platform data.\n");
		return -EINVAL;
	}

	priv->pdev = pdev;

	err = mlxreg_io_attr_init(priv);
	if (err) {
		dev_err(&priv->pdev->dev, "Failed to allocate attributes: %d\n",
			err);
		return err;
	}

	priv->hwmon = devm_hwmon_device_register_with_groups(&pdev->dev,
					"mlxreg_io", priv, priv->groups);
	if (IS_ERR(priv->hwmon)) {
		dev_err(&pdev->dev, "Failed to register hwmon device %ld\n",
			PTR_ERR(priv->hwmon));
		return PTR_ERR(priv->hwmon);
	}

	dev_set_drvdata(&pdev->dev, priv);

	return 0;
}

static struct platform_driver mlxreg_io_driver = {
	.driver = {
	    .name = "mlxreg-io",
	},
	.probe = mlxreg_io_probe,
};

module_platform_driver(mlxreg_io_driver);

MODULE_AUTHOR("Vadim Pasternak <vadimp@mellanox.com>");
MODULE_DESCRIPTION("Mellanox regmap I/O access driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:mlxreg-io");
