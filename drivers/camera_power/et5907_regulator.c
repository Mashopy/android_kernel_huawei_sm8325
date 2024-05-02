/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2023. All rights reserved.
 *
 * Description: cam pmic dev driver
 */
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regmap.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/regulator/of_regulator.h>
#include <securec.h>

#include "et5907_regulator.h"
#include "xpmic_regulator.h"

#define DEFAULT_IRQ_EVENT_DELAY 20
#define REGULATOR_1_2_MAXUV 1800000 // 1.8V
#define REGULATOR_1_2_MINUV 600000  // 0.6V
#define REGULATOR_MAXUV 3750000 // 3.75V
#define REGULATOR_MINUV 1200000 // 1.2V
#define REGULATOR_1_2_STEP 6000
#define REGULATOR_STEP 10000
#define ET5907_MAX_DEFER_TIME 6 /* 6s */
static int dbg_enable = 1;
module_param_named(dbg_level, dbg_enable, int, 0644);

static const struct regmap_config et5907_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = ET5907_REG_CONTROL,
};

int binary2char(unsigned char *binary, int len, char *buff, int size)
{
	int i, n, ret;
	n = 0;
	ret = 0;

	for (i = 0; i < len; i++) {
		ret = sprintf_s(buff + i * 2, size - i * 2, "%02X", binary[i]);
		if (ret == -1) {
			pr_err("binary2char sprintf_s failed");
			return 0;
		}
		n += ret;
	}

	if (n > size) {
		pr_err("binary2char data large than array size");
		return 0;
	}
	return n;
}

int char2binary(const char *token, int len, unsigned char *binary, int size)
{
	const char *p = NULL;
	int i, n, m;
	char buf[3] = { 0, 0, 0 };
	int ret;

	m = len % 2 ? (len - 1) / 2 : len / 2;

	if (m > size) {
		pr_err("char2binary data large than array size");
		return 0;
	}
	p = token;

	for (i = 0; i < m; i++) {
		p = token + i * 2;
		buf[0] = p[0];
		buf[1] = p[1];

		n = 0;
		ret = sscanf_s(buf, "%X", &n);
		if (ret != 1)
			return ret;
		binary[i] = n;
	}

	if (len % 2) {
		buf[0] = p[2];
		buf[1] = 0;
		n = 0;
		ret = sscanf_s(buf, "%X", &n);
		if (ret != 1)
			return ret;
		binary[i] = n;
		i++;
	}

	return i;
}

static int et5907_regulator_dump_regs(struct et5907_regulator *regular, u8 *reg_arr)
{
	regmap_bulk_read(regular->regmap, 0x00, reg_arr, 0x1F);
	pr_info("CHIPID: 0x%x\n", reg_arr[ET5907_REG_CHIPID]);
	pr_info("VERID: 0x%x\n", reg_arr[ET5907_REG_VERID]);
	pr_info("LDO_ILIMIT: 0x%x\n", reg_arr[ET5907_REG_LDO_ILIMIT]);
	pr_info("LDO_EN: 0x%x\n", reg_arr[ET5907_REG_LDO_EN]);
	pr_info("LDO1_VSET: 0x%x\n", reg_arr[ET5907_REG_LDO1_VSET]);
	pr_info("LDO2_VSET: 0x%x\n", reg_arr[ET5907_REG_LDO2_VSET]);
	pr_info("LDO3_VSET: 0x%x\n", reg_arr[ET5907_REG_LDO3_VSET]);
	pr_info("LDO4_VSET: 0x%x\n", reg_arr[ET5907_REG_LDO4_VSET]);
	pr_info("LDO5_VSET: 0x%x\n", reg_arr[ET5907_REG_LDO5_VSET]);
	pr_info("LDO6_VSET: 0x%x\n", reg_arr[ET5907_REG_LDO6_VSET]);
	pr_info("LDO7_VSET: 0x%x\n", reg_arr[ET5907_REG_LDO7_VSET]);
	pr_info("LDO12_SEQ: 0x%x\n", reg_arr[ET5907_REG_LDO12_SEQ]);
	pr_info("LDO34_SEQ: 0x%x\n", reg_arr[ET5907_REG_LDO34_SEQ]);
	pr_info("LDO56_SEQ: 0x%x\n", reg_arr[ET5907_REG_LDO56_SEQ]);
	pr_info("LDO7_SEQ: 0x%x\n", reg_arr[ET5907_REG_LDO7_SEQ]);
	pr_info("SEQ_CTR: 0x%x\n", reg_arr[ET5907_REG_SEQ_CTR]);
	pr_info("LDO_DIS: 0x%x\n", reg_arr[ET5907_REG_LDO_DIS]);
	pr_info("RESET: 0x%x\n", reg_arr[ET5907_REG_RESET]);
	pr_info("I2C_ADDR: 0x%x\n", reg_arr[ET5907_REG_I2C_ADDR]);
	pr_info("UVP_INT: 0x%x\n", reg_arr[ET5907_REG_UVP_INT]);
	pr_info("OVP_INT: 0x%x\n", reg_arr[ET5907_REG_OCP_INT]);
	pr_info("TSD_UVLO_INT: 0x%x\n", reg_arr[ET5907_REG_TSD_UVLO_INT]);
	pr_info("UVP_STAU: 0x%x\n", reg_arr[ET5907_REG_UVP_STAU]);
	pr_info("OCP_STAU: 0x%x\n", reg_arr[ET5907_REG_OCP_STAU]);
	pr_info("TSD_UVLO_STAU: 0x%x\n", reg_arr[ET5907_REG_TSD_UVLO_STAU]);
	pr_info("SUSD_STAU: 0x%x\n", reg_arr[ET5907_REG_SUSD_STAU]);
	pr_info("UVP_INTMA: 0x%x\n", reg_arr[ET5907_REG_UVP_INTMA]);
	pr_info("OCP_INTMA: 0x%x\n", reg_arr[ET5907_REG_OCP_INTMA]);
	pr_info("TSD_UVLO_INTMA: 0x%x\n", reg_arr[ET5907_REG_TSD_UVLO_INTMA]);

	return 0;
}

ssize_t et5907_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len = 0;
	u8 reg_arr[0x1F];

	struct et5907_regulator *regular = dev_get_drvdata(dev);
	et5907_regulator_dump_regs(regular, reg_arr);
	len = binary2char(reg_arr, 0x1e, buf, 200);
	if (len > 0) {
		pr_info("%s: learn buf = %s  \n", __func__, buf);
		return len;
	}
	return len;
}

ssize_t et5907_reg_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int val = 0;
	int addr;
	int ret;
	struct et5907_regulator *regular =
		(struct et5907_regulator *)dev_get_drvdata(dev);

	ret = sscanf_s(buf, "%x:%x", &addr, &val);
	pr_info("REG%x: 0x%x\n", addr, val);
	if (ret != 2)
		return -EINVAL;
	ret = regmap_write(regular->regmap, addr, val);
	if (ret < 0)
		return ret;

	return count;
}

ssize_t et5907_regulator_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct et5907_regulator *regular = dev_get_drvdata(dev);
	int val = 0;
	int ret = 0;

	ret = regmap_read(regular->regmap, ET5907_REG_LDO_EN, &val);
	if (ret < 0)
		return ret;

	return 0;
}

ssize_t et5907_regulator_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct et5907_regulator *regular = (struct et5907_regulator *)dev_get_drvdata(dev);
	int ret;
	char cmd;
	u32 input[2], addr, data;

	ret = sscanf_s(buf, "%c:", &cmd, sizeof(char));
	switch (cmd) {
	case 'w':
		ret = sscanf_s(buf, "%c:%x:%x", &cmd, &input[0], &input[1]);
		if (ret != 3) {
			pr_err("erro! cmd format: echo w [addr] [value]\n");
			goto out;
		};
		addr = input[0] & 0xff;
		data = input[1] & 0xff;
		pr_info("cmd : %c %x %x\n\n", cmd, input[0], input[1]);
		regmap_write(regular->regmap, addr, data);
		regmap_read(regular->regmap, addr, &data);
		pr_info("new: %x %x\n", addr, data);
		break;
	case 'r':
		ret = sscanf_s(buf, "%c:%x", &cmd, &input[0]);
		if (ret != 2) {
			pr_err("erro! cmd format: echo r [addr]\n");
			goto out;
		};
		pr_info("cmd : %c %x\n\n", cmd, input[0]);
		addr = input[0] & 0xff;
		regmap_read(regular->regmap, addr, &data);
		pr_info("%x %x\n", input[0], data);
		break;
	case 'd':
		ret = sscanf_s(buf, "%c:%x:%x", &cmd, &input[0], &input[1]);
		if (ret != 3) {
			pr_err("erro! cmd format: echo d [addr]\n");
			goto out;
		};
		pr_info("cmd : %c %x %x\n\n", cmd, input[0], input[1]);
		addr = input[0] & 0xff;
		data = input[1] & 0xff;
		regmap_update_bits(regular->regmap, ET5907_REG_LDO12_SEQ, addr, data);
		regmap_read(regular->regmap, ET5907_REG_LDO12_SEQ, &data);
		pr_info("ET5907_REG_LDO12_SEQ: %x %x\n", ET5907_REG_LDO12_SEQ, data);
		break;
	default:
		pr_err("Unknown command\n");
		break;
	}
out:
	return count;
}

static struct device_attribute et5907_regulator_attrs[] = {
	__ATTR(et5907_regulator_info, 0664, et5907_reg_show, et5907_reg_store),
	__ATTR(et5907_regulator_set, 0664, et5907_regulator_show,
		et5907_regulator_store),
};

static void et5907_init_sysfs(struct et5907_regulator *regular)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(et5907_regulator_attrs); i++) {
		ret = sysfs_create_file(&regular->dev->kobj,
			&et5907_regulator_attrs[i].attr);
		if (ret)
			pr_err("create et5907 regulator node(%s) error\n",
				et5907_regulator_attrs[i].attr.name);
	}
}

static int et5907_regulator_enable(struct regulator_dev *rdev)
{
	struct et5907_regulator *info = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);

	pr_info("%s  id= %d\n", __func__, id);
	if (info == NULL) {
		pr_err("regulator info null pointer\n");
		return -EINVAL;
	}

	return regmap_update_bits(rdev->regmap, ET5907_REG_LDO_EN,
		rdev->desc->vsel_mask, rdev->desc->vsel_mask);
}

static int et5907_regulator_disable(struct regulator_dev *rdev)
{
	struct et5907_regulator *info = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);

	pr_info("%s id= %d\n", __func__, id);
	if (info == NULL) {
		pr_err("regulator info null pointer\n");
		return -EINVAL;
	}

	return regmap_update_bits(rdev->regmap, ET5907_REG_LDO_EN,
		rdev->desc->vsel_mask, 0);
}

static int et5907_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct et5907_regulator *info = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int val;

	if (info == NULL) {
		pr_err("regulator info null pointer\n");
		return -EINVAL;
	}
	regmap_read(rdev->regmap, ET5907_REG_LDO_EN, &val);
	pr_info("%s, id=%d, val= %d, vsel_mask=%d",
			__func__, id, val, rdev->desc->vsel_mask);
	if (val & rdev->desc->vsel_mask)
		return 1;

	return 0;
}

__attribute__((unused)) static int et5907_get_voltage_regulator
	(struct regulator_dev *rdev)
{
	int ret;
	unsigned int val;
	struct et5907_regulator *info = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);

	pr_info("%s= %d", __func__, id);
	if (info == NULL) {
		pr_err("regulator info null pointer\n");
		return -EINVAL;
	}
	ret = regmap_read(rdev->regmap, et5907_ldo_reg_index(id), &val);
	if (ret != 0)
		return ret;

	return val;
}

__attribute__((unused)) static int et5907_set_voltage_regulator
	(struct regulator_dev *rdev, unsigned int sel)
{
	struct et5907_regulator *info = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);

	pr_info("%s id= %d", __func__, id);
	if (info == NULL) {
		pr_err("regulator info null pointer\n");
		return -EINVAL;
	}

	return regmap_write(rdev->regmap, et5907_ldo_reg_index(id), sel);
}

static int et5907_set_voltage(struct regulator_dev *rdev, int min_uv,
	int max_uv, unsigned *selector)
{
	struct et5907_regulator *info = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	u8 val = 0;
	int ret = 0;
	int mask = rdev->desc->vsel_mask;

	pr_info("et5907_set_voltage id= %d min_uv=%d max_uv=%d mask=%d", id,
		min_uv, max_uv, mask);
	if (info == NULL) {
		pr_err("regulator info null pointer\n");
		return -EINVAL;
	}

	if (id == ET5907_ID_REGULATOR1 || id == ET5907_ID_REGULATOR2) {
		if (min_uv > REGULATOR_1_2_MAXUV || min_uv < REGULATOR_1_2_MINUV) {
			pr_err("min_uv or max_uv out of range\n");
			return -EINVAL;
		}
		val = (min_uv - REGULATOR_1_2_MINUV) / REGULATOR_1_2_STEP;
	} else {
		if (min_uv > REGULATOR_MAXUV || min_uv < REGULATOR_MINUV) {
			pr_err("min_uv or max_uv out of range\n");
			return -EINVAL;
		}
		val = (min_uv - REGULATOR_MINUV) / REGULATOR_STEP;
	}
	ret = regmap_write(rdev->regmap, et5907_ldo_reg_index(id), val);
	if (ret < 0) {
		pr_err("Failed to et5907_set_voltage: %d\n", ret);
		return ret;
	}

	return ret;
}

static int et5907_voltage_get(struct regulator_dev *rdev)
{
	int id = rdev_get_id(rdev);
	int err;
	int val = 0;

	err = regmap_read(rdev->regmap, et5907_ldo_reg_index(id), &val);
	if (err < 0) {
		pr_err("Failed to et5907_voltage_get: %d\n", err);
		return -EINVAL;
	}
	if (id == ET5907_ID_REGULATOR1 || id == ET5907_ID_REGULATOR2) {
		pr_info("et5907_voltage_get %d",
			(val * REGULATOR_1_2_STEP + REGULATOR_1_2_MINUV));
		return val * REGULATOR_1_2_STEP + REGULATOR_1_2_MINUV;
	} else {
		pr_info("et5907_voltage_get %d",
			(val * REGULATOR_STEP + REGULATOR_MINUV));
		return val * REGULATOR_STEP + REGULATOR_MINUV;
	}
}

static const struct regulator_ops et5907_regulator_ops = {
	.enable = et5907_regulator_enable,
	.disable = et5907_regulator_disable,
	.is_enabled = et5907_regulator_is_enabled,
	.set_voltage = et5907_set_voltage,
	.get_voltage = et5907_voltage_get,
};

int getid_by_reg(int reg, int reg_val)
{
	if (reg == ET5907_REG_LDO_EN || reg == ET5907_REG_UVP_INT ||
		reg == ET5907_REG_OCP_INT) {
		if ((reg_val & ET5907_BIT_0) == ET5907_BIT_0)
			return ET5907_ID_REGULATOR1;
		if ((reg_val & ET5907_BIT_1) == ET5907_BIT_1)
			return ET5907_ID_REGULATOR2;
		if ((reg_val & ET5907_BIT_2) == ET5907_BIT_2)
			return ET5907_ID_REGULATOR3;
		if ((reg_val & ET5907_BIT_3) == ET5907_BIT_3)
			return ET5907_ID_REGULATOR4;
		if ((reg_val & ET5907_BIT_4) == ET5907_BIT_4)
			return ET5907_ID_REGULATOR5;
		if ((reg_val & ET5907_BIT_5) == ET5907_BIT_5)
			return ET5907_ID_REGULATOR6;
		if ((reg_val & ET5907_BIT_6) == ET5907_BIT_6)
			return ET5907_ID_REGULATOR7;
	}
	return -1;
}

#ifdef ET5907_IRQ_EN

static void et5907_irq_event_work(struct work_struct *work)
{
	int err = 0;
	int id = 0;
	int reg_uvp, reg_ocp, reg_tsd;

	struct et5907_regulator *chip = container_of(
		work, struct et5907_regulator, irq_event_work.work);
	pr_info("et5907_irq_event_work");
	err = regmap_read(chip->regmap, ET5907_REG_UVP_INT, &reg_uvp);
	if (err < 0)
		goto error_i2c;
	err = regmap_read(chip->regmap, ET5907_REG_OCP_INT, &reg_ocp);
	if (err < 0)
		goto error_i2c;
	err = regmap_read(chip->regmap, ET5907_REG_TSD_UVLO_INT, &reg_tsd);
	if (err < 0)
		goto error_i2c;

	pr_info("REG_UVP=%x REG_OCP=%x reg_tsd=%x ", reg_uvp, reg_ocp, reg_tsd);
	if (reg_uvp || reg_ocp || reg_tsd) {
		/* irq inter */
		if (chip->irq_num > DEFAULT_IRQ_NUM) {
			if (reg_uvp) {
				id = getid_by_reg(ET5907_REG_UVP_INT, reg_uvp);
				if (id < 0)
					goto error_id;
				regulator_notifier_call_chain(
					chip->rdev[id],
					REGULATOR_EVENT_UNDER_VOLTAGE,
					&reg_uvp);
			}
			chip->irq_num = 0;
		} else {
		/* first one to delay 20ms */
			chip->irq_num++;
			schedule_delayed_work(&chip->irq_event_work,
				DEFAULT_IRQ_EVENT_DELAY);
			return;
		}
		/* ocp tsd */
		if (reg_ocp) {
			id = getid_by_reg(ET5907_REG_OCP_INT, reg_ocp);
			if (id < 0)
				goto error_id;
			regulator_notifier_call_chain(
				chip->rdev[id], REGULATOR_EVENT_OVER_CURRENT,
				&reg_uvp);
		}
		if (reg_tsd) {
			if (((reg_tsd & ET5907_TSD_TSD) == ET5907_TSD_TSD) ||
			    ((reg_tsd & ET5907_TSD_TSD_WRN) ==
			     ET5907_TSD_TSD_WRN)) {
				regulator_notifier_call_chain(
					chip->rdev[id],
					REGULATOR_EVENT_OVER_TEMP, &reg_tsd);
			} else if ((reg_tsd & ET5907_TSD_UVLO_VSYS) ==
				   ET5907_TSD_UVLO_VSYS) {
				regulator_notifier_call_chain(
					chip->rdev[id],
					REGULATOR_EVENT_VOLTAGE_CHANGE,
					&reg_tsd);
			} else {
				regulator_notifier_call_chain(
					chip->rdev[id], REGULATOR_EVENT_FAIL,
					&reg_tsd);
			}
		}
	} else {
		pr_info("et5907 no irq");
		chip->irq_num = 0;
	}
	return;

error_id:
	pr_err("ID error : %d\n", err);
	chip->irq_num = 0;
	return;
error_i2c:
	pr_err("I2C error : %d\n", err);
	chip->irq_num = 0;
}

static irqreturn_t et5907_irq_handler(int irq, void *data)
{
	struct et5907_regulator *chip = data;
	schedule_delayed_work(&chip->irq_event_work, DEFAULT_IRQ_EVENT_DELAY);

	return IRQ_NONE;
}
#endif

static struct regulator_desc et5907_regulator_desc[ET5907_MAX_REGULATORS] = {
	{
		.name = "et07_ldo1",
		.id = ET5907_ID_REGULATOR1,
		.ops = &et5907_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.vsel_mask = BIT(0),
		.n_voltages = 1,
	},
	{
		.name = "et07_ldo2",
		.id = ET5907_ID_REGULATOR2,
		.ops = &et5907_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.vsel_mask = BIT(1),
		.n_voltages = 1,
	},
	{
		.name = "et07_ldo3",
		.id = ET5907_ID_REGULATOR3,
		.ops = &et5907_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.vsel_mask = BIT(2),
		.n_voltages = 1,
	},
	{
		.name = "et07_ldo4",
		.id = ET5907_ID_REGULATOR4,
		.ops = &et5907_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.vsel_mask = BIT(3),
		.n_voltages = 1,
	},
	{
		.name = "et07_ldo5",
		.id = ET5907_ID_REGULATOR5,
		.ops = &et5907_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.vsel_mask = BIT(4),
		.n_voltages = 1,
	},
	{
		.name = "et07_ldo6",
		.id = ET5907_ID_REGULATOR6,
		.ops = &et5907_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.vsel_mask = BIT(5),
		.n_voltages = 1,
	},
	{
		.name = "et07_ldo7",
		.id = ET5907_ID_REGULATOR7,
		.ops = &et5907_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.vsel_mask = BIT(6),
		.n_voltages = 1,
	}
};

static int et5907_regulator_init(struct et5907_regulator *chip)
{
	struct regulator_config config = {};
	int i, ret;

	/* init disable ldos */
	ret = regmap_write(chip->regmap, ET5907_REG_LDO_EN, 0x00);
	if (ret < 0) {
		pr_err("Failed to set ET5907_REG_LDO_EN reg: %d\n",
			ret);
		return ret;
	}
	/* ET5907 SOFTRESET */
	ret = regmap_write(chip->regmap, ET5907_REG_RESET, 0x0B);
	if (ret < 0) {
		pr_err("Failed to set ET5907_REG_RESET reg: %d\n",
			ret);
		return ret;
	}
	/* Set up regulators */
	/* Register the regulators */
	for (i = 0; i < ET5907_MAX_REGULATORS; i++) {
		config.init_data = chip->pdata->init_data[i];
		config.dev = chip->dev;
		config.driver_data = chip;
		config.regmap = chip->regmap;
		config.of_node = chip->pdata->reg_node[i];
		chip->rdev[i] = devm_regulator_register(
			chip->dev, &et5907_regulator_desc[i], &config);
		if (IS_ERR(chip->rdev[i])) {
			ret = PTR_ERR(chip->rdev[i]);
			pr_err(
				"Failed to register ET5907 regulator[ %s: %d]\n",
				et5907_regulator_desc[i].name, ret);
			return ret;
		}
	}

	return 0;
}

__attribute__((unused)) static struct of_regulator_match
	et5907_regulator_matches[ET5907_MAX_REGULATORS] = {
		[ET5907_ID_REGULATOR1] = { .name = "et07_ldo1" },
		[ET5907_ID_REGULATOR2] = { .name = "et07_ldo2" },
		[ET5907_ID_REGULATOR3] = { .name = "et07_ldo3" },
		[ET5907_ID_REGULATOR4] = { .name = "et07_ldo4" },
		[ET5907_ID_REGULATOR5] = { .name = "et07_ldo5" },
		[ET5907_ID_REGULATOR6] = { .name = "et07_ldo6" },
		[ET5907_ID_REGULATOR7] = { .name = "et07_ldo7" },
	};

static struct et5907_pdata *et5907_regulator_parse_dt(struct device *dev)
{
	struct et5907_pdata *pdata;
	struct device_node *node;
	int i, ret, n;
	struct device_node *np = dev->of_node;

	if (np == NULL) {
		pr_err("Error: et-changer np = NULL\n");
		return ERR_PTR(-ENODEV);
	}
	node = of_get_child_by_name(dev->of_node, "regulators");
	if (!node) {
		pr_err("regulator node not found\n");
		return ERR_PTR(-ENODEV);
	}
	ret = of_regulator_match(dev, node, et5907_regulator_matches,
		ARRAY_SIZE(et5907_regulator_matches));
	of_node_put(node);
	if (ret < 0) {
		pr_err("Error parsing regulator init data: %d\n", ret);
		return ERR_PTR(-EINVAL);
	}
	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);
	n = 0;
	for (i = 0; i < ARRAY_SIZE(et5907_regulator_matches); i++) {
		if (!et5907_regulator_matches[i].init_data)
			continue;
		pdata->init_data[n] = et5907_regulator_matches[i].init_data;
		pdata->init_data[n]->constraints.valid_ops_mask |= REGULATOR_CHANGE_STATUS
				| REGULATOR_CHANGE_VOLTAGE
				| REGULATOR_CHANGE_MODE
				| REGULATOR_CHANGE_DRMS;
		pdata->reg_node[n] = et5907_regulator_matches[i].of_node;
		n++;
	}

	return pdata;
}

static int et5907_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int ret;
	unsigned int data;
	struct et5907_regulator *chip;
	unsigned long long time_from_boot;

	chip = devm_kzalloc(&i2c->dev, sizeof(struct et5907_regulator),
		GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto error;
	}

	chip->dev = &i2c->dev;
	chip->regmap = devm_regmap_init_i2c(i2c, &et5907_regmap_config);
	if (IS_ERR(chip->regmap)) {
		ret = PTR_ERR(chip->regmap);
		pr_err("Failed to allocate register map: %d\n", ret);
		goto error;
	}

	chip->reset_gpio = of_get_named_gpio_flags(chip->dev->of_node, "et5907,reset", 0, NULL);
	ret = xpmic_gpio_ops(chip->reset_gpio, XPMIC_GPIO_PULL_HIGH);
	if (ret) {
		pr_err("GPIO[%d] request failed[%d] \n", chip->reset_gpio, ret);
		goto error;
	}

	i2c_set_clientdata(i2c, chip);

	chip->pdata = i2c->dev.platform_data;

	ret = regmap_read(chip->regmap, ET5907_REG_CHIPID, &data);
	if (ret < 0)
		pr_err("Failed to read DEVICE_ID reg: %d\n", ret);

	switch (data) {
	case ET5907_DEVICE_ID:
		chip->chip_id = ET5907;
		break;
	default:
		pr_err("Unsupported device id = 0x%x\n", data);
		ret = -ENODEV;
		goto error;
	}

	if (!chip->pdata)
		chip->pdata = et5907_regulator_parse_dt(chip->dev);

	if (IS_ERR(chip->pdata)) {
		pr_err("No regulators defined for the platform\n");
		ret = PTR_ERR(chip->pdata);
		goto error;
	}

#ifdef ET5907_IRQ_EN
	chip->chip_irq = i2c->irq;

	if (chip->chip_irq != 0) {
		ret = regmap_write(chip->regmap, ET5907_REG_UVP_INTMA, 0x00);
		if (ret < 0) {
			pr_err(
				"Failed to mask ET5907_REG_UVP_INTMA reg: %d\n", ret);
			goto error;
		}

		ret = regmap_write(chip->regmap, ET5907_REG_OCP_INTMA, 0x00);
		if (ret < 0) {
			pr_err(
				"Failed to mask ET5907_REG_OCP_INTMA reg: %d\n", ret);
			goto error;
		}

		ret = regmap_write(chip->regmap, ET5907_REG_TSD_UVLO_INTMA, 0x00);
		if (ret < 0) {
			pr_err(
				"Failed to mask ET5907_REG_TSD_UVLO_INTMA reg: %d\n", ret);
			goto error;
		}

		ret = devm_request_threaded_irq(chip->dev, chip->chip_irq, NULL,
			et5907_irq_handler,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			"et5907", chip);
		if (ret != 0) {
			pr_err("Failed to request IRQ: %d\n", chip->chip_irq);
			goto error;
		}
		INIT_DELAYED_WORK(&chip->irq_event_work, et5907_irq_event_work);
		chip->irq_num = 0;
	} else {
		dev_warn(chip->dev, "No IRQ configured\n");
	}
#endif

	et5907_init_sysfs(chip);

	ret = et5907_regulator_init(chip);
	if (ret < 0) {
		pr_err("Failed to initialize regulator: %d\n", ret);
		goto error;
	}

	pr_info("initialize et5907 success");
	return ret;

error:
	pr_err("initialize et5907 failed");
	if (chip->reset_gpio) {
		ret = xpmic_gpio_ops(chip->reset_gpio, XPMIC_GPIO_PULL_LOW);
		if (ret)
			pr_err("Error: release RESET GPIO %d\n", chip->reset_gpio);
	}

	time_from_boot = ktime_to_timespec64(ktime_get_boottime()).tv_sec;
	if (time_from_boot < (unsigned long long)ET5907_MAX_DEFER_TIME) {
		pr_err("et5907 defer probe\n");
		return -EPROBE_DEFER;
	}
	return ret;
}

static int et5907_i2c_remove(struct i2c_client *client)
{
	int ret = 0;
	struct et5907_regulator *chip = i2c_get_clientdata(client);
	if (chip->reset_gpio > 0)
		ret = xpmic_gpio_ops(chip->reset_gpio, XPMIC_GPIO_PULL_LOW);
	return ret;
}

static const struct i2c_device_id et5907_i2c_id[] = {
	{ "et5907", ET5907 },
	{},
};

MODULE_DEVICE_TABLE(i2c, et5907_i2c_id);

static const struct of_device_id et5907_dt_ids[] = {
	{ .compatible = "etek,et5907",
	  .data = &et5907_i2c_id[0]
	},
	{},
};

MODULE_DEVICE_TABLE(of, et5907_dt_ids);

static struct i2c_driver et5907_regulator_driver = {
	.driver = {
		.name = "et5907",
		.of_match_table = of_match_ptr(et5907_dt_ids),
	},
	.probe    = et5907_i2c_probe,
	.id_table = et5907_i2c_id,
	.remove   = et5907_i2c_remove,
};

int et5907_regulator_i2c_init(void)
{
	pr_debug("%s", __func__);
	return i2c_add_driver(&et5907_regulator_driver);
}

void et5907_regulator_i2c_exit(void)
{
	pr_debug("%s", __func__);
	i2c_del_driver(&et5907_regulator_driver);
}

MODULE_AUTHOR("HUAWEI");
MODULE_DESCRIPTION("et5907 regulator driver");
MODULE_LICENSE("GPL");
