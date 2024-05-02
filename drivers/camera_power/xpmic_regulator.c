/*
* Copyright (c) 2022-2023 Huawei Technologies Co., Ltd. All Rights Reserved.
*
* Description: xpmic driver
*/

#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/time64.h>
#include <linux/delay.h>
#include <linux/list.h>

#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/of_regulator.h>

#include "et5907_regulator.h"
#include "wl2868c_regulator.h"
#include "xpmic_regulator.h"

#define XPMIC_MAX_DEFER_TIME 45

static int xpmic_gpio_count = 0;

struct pmic_submodule {
	int (*init)(void);
	void (*exit)(void);
};

const struct pmic_submodule pmic_table[] = {
	/* pmic init here */
	{&et5907_regulator_i2c_init, &et5907_regulator_i2c_exit},
	{&wl2868c_regulator_i2c_init, &wl2868c_regulator_i2c_exit},
};

enum {
	XPMIC_REGULATOR_LDO1 = 0,
	XPMIC_REGULATOR_LDO2,
	XPMIC_REGULATOR_LDO3,
	XPMIC_REGULATOR_LDO4,
	XPMIC_REGULATOR_LDO5,
	XPMIC_REGULATOR_LDO6,
	XPMIC_REGULATOR_LDO7,
	XPMIC_MAX_REGULATORS,
};

struct xpmic_info {
	struct regulator_desc *rdesc;
	struct regulator_dev *rdev;
	struct device_node *of_node;
	struct regulator *reg;
	struct device *dev;
};

struct xpmic_regulator_data {
	const char *xpmic_reg;
	const char *wl_reg;
	const char *et_reg;
};

static struct xpmic_regulator_data reg_data[] = {
	/* xpmic              wl           et */
	{"xpmic_ldo1", "wl2868c_l1", "et07_ldo1"},
	{"xpmic_ldo2", "wl2868c_l2", "et07_ldo2"},
	{"xpmic_ldo3", "wl2868c_l3", "et07_ldo3"},
	{"xpmic_ldo4", "wl2868c_l4", "et07_ldo4"},
	{"xpmic_ldo5", "wl2868c_l5", "et07_ldo5"},
	{"xpmic_ldo6", "wl2868c_l6", "et07_ldo6"},
	{"xpmic_ldo7", "wl2868c_l7", "et07_ldo7"},
};

int xpmic_gpio_ops(int gpio, int ops)
{
	int ret = 0;

	if (!gpio_is_valid(gpio)) {
		pr_err("%s: Invaild gpio\n", __func__);
		return -EPERM;
	}

	switch (ops) {
	case XPMIC_GPIO_PULL_HIGH: {
		if (!xpmic_gpio_count) {
			ret = gpio_request_one(gpio, GPIOF_OUT_INIT_HIGH, "XPMIC RESET CONTRL");
			if (ret) {
				pr_err("%s: failed to request RESET GPIO\n", __func__);
				return ret;
			}
		}
		xpmic_gpio_count++;
		break;
	}
	case XPMIC_GPIO_PULL_LOW: {
		if (!xpmic_gpio_count) {
			return 0;
		} else if (xpmic_gpio_count == 1) {
			ret = gpio_direction_output(gpio, 0);
			if (ret < 0) {
				pr_err("%s: failed to pull low GPIO\n", __func__);
				return ret;
			}
			gpio_free(gpio);
		}
		xpmic_gpio_count--;
		break;
	}
	default:
		pr_err("%s: Invaild gpio ops\n", __func__);
		return -EPERM;
	}

	return ret;
}

static int xpmic_set_enable(struct regulator_dev *rdev)
{
	struct xpmic_info *xpmic = rdev_get_drvdata(rdev);
	struct regulator *reg = NULL;
	reg = xpmic->reg;
	pr_info("xpmic_set_enable");
	return regulator_enable(reg);
}

static int xpmic_set_disable(struct regulator_dev *rdev)
{
	struct xpmic_info *xpmic = rdev_get_drvdata(rdev);
	struct regulator *reg = NULL;
	reg = xpmic->reg;
	pr_info("xpmic_set_disable");
	return regulator_disable(reg);
}

static int xpmic_is_enabled(struct regulator_dev *rdev)
{
	struct xpmic_info *xpmic = rdev_get_drvdata(rdev);
	struct regulator *reg = NULL;
	reg = xpmic->reg;
	pr_info("xpmic_is_enabled");
	return regulator_is_enabled(reg);
}

static int xpmic_set_voltage(struct regulator_dev *rdev,
				int min_uv, int max_uv, unsigned int *selector)
{
	struct xpmic_info *xpmic = rdev_get_drvdata(rdev);
	struct regulator *reg = NULL;
	reg = xpmic->reg;
	pr_info("xpmic_set_voltage min%d max%d", min_uv, max_uv);
	return regulator_set_voltage(reg, min_uv, max_uv);
}

static int xpmic_get_voltage(struct regulator_dev *rdev)
{
	struct xpmic_info *xpmic = rdev_get_drvdata(rdev);
	struct regulator *reg = NULL;
	reg = xpmic->reg;
	pr_info("xpmic_get_voltage");
	return regulator_get_voltage(reg);
}

static const struct regulator_ops xpmic_regulator_ops = {
	.enable = xpmic_set_enable,
	.disable = xpmic_set_disable,
	.is_enabled = xpmic_is_enabled,
	.set_voltage = xpmic_set_voltage,
	.get_voltage = xpmic_get_voltage,
};

/* set n_voltages for qcom driver call regulator_count_voltages */
#define XPMIC_REGL_DESC(_id, _name) \
	[XPMIC_REGULATOR_##_id] = { \
		.name = #_name, \
		.id = XPMIC_REGULATOR_##_id, \
		.ops = &xpmic_regulator_ops, \
		.type = REGULATOR_VOLTAGE, \
		.owner = THIS_MODULE, \
		.n_voltages = 1, \
	}

static struct regulator_desc xpmic_regls_desc[XPMIC_MAX_REGULATORS] = {
	XPMIC_REGL_DESC(LDO1, xpmic_ldo1),
	XPMIC_REGL_DESC(LDO2, xpmic_ldo2),
	XPMIC_REGL_DESC(LDO3, xpmic_ldo3),
	XPMIC_REGL_DESC(LDO4, xpmic_ldo4),
	XPMIC_REGL_DESC(LDO5, xpmic_ldo5),
	XPMIC_REGL_DESC(LDO6, xpmic_ldo6),
	XPMIC_REGL_DESC(LDO7, xpmic_ldo7),
};

static struct regulator *xpmic_match_pre_reg(struct device *dev, int index)
{
	struct regulator *reg = NULL;
	unsigned long long time_from_boot;

	reg = devm_regulator_get_optional(dev, reg_data[index].wl_reg);
	if (!IS_ERR(reg))
		return reg;

	reg = devm_regulator_get_optional(dev, reg_data[index].et_reg);
	if (!IS_ERR(reg))
		return reg;

	time_from_boot = ktime_to_timespec64(ktime_get_boottime()).tv_sec;
	if (time_from_boot < (unsigned long long)XPMIC_MAX_DEFER_TIME)
		return ERR_PTR(-EPROBE_DEFER);

	pr_err("xpmic_match_pre_reg failed, time_from_boot: %d \n", time_from_boot);
	return reg;
}

static int xpmic_regulator_init(struct xpmic_info *xpmic, const char *name)
{
	struct regulator_config config = { };
	struct regulator_init_data *init_data = NULL;
	struct regulator *reg = NULL;
	struct device *dev = xpmic->dev;
	struct device_node *reg_node = xpmic->of_node;
	int ret = 0;
	int i;

	/* get regulator data */
	for (i = 0; i < XPMIC_MAX_REGULATORS; i++) {
		if (!strcmp(reg_data[i].xpmic_reg, name)) {
			break;
		}
	}
	/* get init data from dts */
	init_data = of_get_regulator_init_data(dev, reg_node,
							xpmic->rdesc);
	if (init_data == NULL) {
		pr_err("%s: failed to get regulator data\n", name);
		return -ENODATA;
	}
	if (!init_data->constraints.name) {
		pr_err("%s: regulator name missing\n", name);
		return -EINVAL;
	}
	/* get desc data */
	xpmic->rdesc = &xpmic_regls_desc[i];
	/* get struct regulator  */
	reg = xpmic_match_pre_reg(dev, i);
	if (IS_ERR(reg)) {
		ret = PTR_ERR(reg);
		return ret;
	}
	xpmic->reg = reg;
	/* get config data */
	config.dev = dev;
	config.driver_data = xpmic;
	config.of_node = reg_node;
	init_data->constraints.valid_ops_mask |= REGULATOR_CHANGE_STATUS
				| REGULATOR_CHANGE_VOLTAGE
				| REGULATOR_CHANGE_MODE
				| REGULATOR_CHANGE_DRMS;
	config.init_data = init_data;

	xpmic->rdev = devm_regulator_register(dev, xpmic->rdesc, &config);
	if (IS_ERR(xpmic->rdev)) {
		ret = PTR_ERR(xpmic->rdev);
		dev_err(dev, "Failed to register regulator(%s):%d \n", xpmic->rdesc->name, ret);
		return ret;
	}
	pr_err("xpmic_regulator_init: LDO%d, successful name %s", (i + 1), xpmic->rdesc->name);
	return 0;
}

static int xpmic_regulator_probe(struct platform_device *pdev)
{
	int rc = 0;
	const char *name = NULL;
	struct device_node *child = NULL;
	struct xpmic_info *xpmic = NULL;

	for_each_available_child_of_node(pdev->dev.of_node, child) {
		xpmic = devm_kzalloc(&pdev->dev, sizeof(*xpmic), GFP_KERNEL);
		if (!xpmic)
			return -ENOMEM;

		xpmic->of_node = child;
		xpmic->dev = &pdev->dev;
		rc = of_property_read_string(child, "regulator-name", &name);
		if (rc)
			continue;
		rc = xpmic_regulator_init(xpmic, name);
		if (rc < 0) {
			pr_err("failed to register regulator %s rc=%d\n",
							name, rc);
			return rc;
		}
	}
	return 0;
}

static const struct of_device_id xpmic_regulator_match_table[] = {
	{
		.compatible = "huawei,xpmic-regulator",
		.data = NULL,
	},
	{},
};

static struct platform_driver xpmic_regulator_driver = {
	.driver = {
		.name = "huawei,xpmic-regulator",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(xpmic_regulator_match_table),
	},
	.probe = xpmic_regulator_probe,
};

static int xpmic_regulator_drv_init(void)
{
	uint i;
	int rc;
	pr_info("%s", __func__);
	for (i = 0; i < ARRAY_SIZE(pmic_table); i++) {
		pr_info("Xpmic Init regulator %d\n", i);
		rc = pmic_table[i].init();
	}
	return platform_driver_register(&xpmic_regulator_driver);
}

static void xpimc_regulator_drv_exit(void)
{
	uint i;
	pr_info("%s", __func__);
	for (i = 0; i < ARRAY_SIZE(pmic_table); i++) {
		pr_info("Xpmic exit regulator %d\n", i);
		pmic_table[i].exit();
	}
	platform_driver_unregister(&xpmic_regulator_driver);
}

module_init(xpmic_regulator_drv_init);
module_exit(xpimc_regulator_drv_exit);

MODULE_AUTHOR("HUAWEI");
MODULE_DESCRIPTION("xpmic regulator driver");
MODULE_LICENSE("GPL");
