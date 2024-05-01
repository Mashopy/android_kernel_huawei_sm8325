/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor sensor utility
 */
#include "vendor_sensor_util.h"
#include "vendor_cam_hiview.h"
#include "cam_debug_util.h"
#include "cam_sensor_dev.h"
#include "cam_sensor_util.h"
#include "cam_sensor_cmn_header.h"
#include "cam_mem_mgr.h"
#include "cam_res_mgr_api.h"
#include <chipset_common/hwpower/hardware_ic/boost_5v.h>

#define VALIDATE_VOLTAGE(min, max, config_val) ((config_val) && \
	(config_val >= min) && (config_val <= max))

int32_t vendor_cam_write_reg(struct camera_io_master *io_master_info,
	struct cam_sensor_config_reg *cam_write_reg_data)
{
	uint32_t i;
	int rc = 0;
	struct cam_sensor_i2c_reg_setting  i2c_reg_settings = {0};
	struct cam_sensor_i2c_reg_array    i2c_reg_array    = {0};
	if (!io_master_info || !cam_write_reg_data) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	i2c_reg_settings.addr_type = cam_write_reg_data->addr_type;
	i2c_reg_settings.data_type = cam_write_reg_data->data_type;
	i2c_reg_settings.size      = cam_write_reg_data->size;
	i2c_reg_settings.delay     = 0;

	for (i = 0; i < cam_write_reg_data->size; i++) {
		i2c_reg_array.reg_addr  = cam_write_reg_data->data[i].reg_addr;
		i2c_reg_array.reg_data  = cam_write_reg_data->data[i].reg_data;
		i2c_reg_array.delay     = cam_write_reg_data->data[i].delay;
		i2c_reg_array.data_mask = 0;

		i2c_reg_settings.reg_setting = &i2c_reg_array;
		CAM_DBG(CAM_SENSOR, "set reg: slave_addr = 0x%x, addr = 0x%x, data = 0x%x",
			io_master_info->cci_client->sid << 1,
			i2c_reg_array.reg_addr,
			i2c_reg_array.reg_data);

		rc = camera_io_dev_write(io_master_info, &i2c_reg_settings);
		if (rc) {
			CAM_ERR(CAM_SENSOR, "set reg failed, rc = %d", rc);
			break;
		}
	}
	return rc;
}

int32_t vendor_cam_read_reg(struct camera_io_master *io_master_info,
	struct cam_sensor_config_reg *cam_read_reg_data)
{
	uint32_t i;
	int rc = 0;
	if (!io_master_info || !cam_read_reg_data) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	for (i = 0; i < cam_read_reg_data->size; i++) {
		rc = camera_io_dev_read(io_master_info,
				cam_read_reg_data->data[i].reg_addr,
				&(cam_read_reg_data->data[i].reg_data),
				cam_read_reg_data->addr_type,
				cam_read_reg_data->data_type);
		if (rc) {
			CAM_ERR(CAM_SENSOR, "get reg failed, rc = %d", rc);
			break;
		} else {
			CAM_DBG(CAM_SENSOR, "get reg: slave_addr = 0x%x, addr = 0x%x, data = 0x%x",
				io_master_info->cci_client->sid << 1,
				cam_read_reg_data->data[i].reg_addr,
				cam_read_reg_data->data[i].reg_data);
		}
	}
	return rc;
}

int vendor_sensor_util_i2c_revert_byte(struct i2c_settings_list *i2c_list)
{
	int32_t  rc = 0;
	uint32_t raw_data;
	uint32_t new_data = 0;
	uint32_t size;
	uint32_t i;

	if (!i2c_list) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	size = i2c_list->i2c_settings.size;
	switch (i2c_list->i2c_settings.data_type) {
	case CAMERA_SENSOR_I2C_TYPE_BYTE: {
		CAM_INFO(CAM_SENSOR, "No need to revert");
		break;
	}
	case CAMERA_SENSOR_I2C_TYPE_WORD: {
		for (i = 0; i < size; i++) {
			raw_data = i2c_list->i2c_settings.reg_setting[i].reg_data;
			new_data = ((raw_data & 0xFF) << 24) | (((raw_data >> 8) & 0xFF) << 16);
			i2c_list->i2c_settings.reg_setting[i].reg_data = new_data;
		}
		break;
	}
	case CAMERA_SENSOR_I2C_TYPE_3B: {
		for (i = 0; i < size; i++) {
			raw_data = i2c_list->i2c_settings.reg_setting[i].reg_data;
			new_data = ((raw_data & 0xFF) << 24) | (((raw_data >> 8) & 0xFF) << 16) |
				(((raw_data >> 16) & 0xFF) << 8);
			i2c_list->i2c_settings.reg_setting[i].reg_data = new_data;
		}
		break;
	}
	case CAMERA_SENSOR_I2C_TYPE_DWORD: {
		for (i = 0; i < size; i++) {
			raw_data = i2c_list->i2c_settings.reg_setting[i].reg_data;
			new_data = ((raw_data & 0xFF) << 24) | (((raw_data >> 8) & 0xFF) << 16) |
				(((raw_data >> 16) & 0xFF) << 8) | ((raw_data >> 24) & 0xFF);
			i2c_list->i2c_settings.reg_setting[i].reg_data = new_data;
		}
		break;
	}
	default:
		CAM_ERR(CAM_SENSOR, "Wrong Opcode: %d", i2c_list->op_code);
		rc = -EINVAL;
		break;
	}

	return rc;
}

void vendor_fill_vreg_params(
	struct cam_hw_soc_info *soc_info,
	struct cam_sensor_power_setting *power_setting, uint16_t i)
{
	int32_t rc = 0, j = 0;
	int num_vreg = soc_info->num_rgltr;
	char mixed_name[MIXED_PIN_NAME_LEN] = {0};

	switch (power_setting[i].seq_type) {
	case SENSOR_CUSTOM_MIXED_PIN1:
	case SENSOR_CUSTOM_MIXED_PIN2:
	case SENSOR_CUSTOM_MIXED_PIN3:
	case SENSOR_CUSTOM_MIXED_PIN4:
	case SENSOR_CUSTOM_MIXED_PIN5:
		rc = sprintf(mixed_name, "cam_mixed%d",
			power_setting[i].seq_type - SENSOR_CUSTOM_MIXED_PIN1 + 1);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "sprintf fail with rc=%d", rc);
			rc = 0;
			break;
		}
		rc = 0;
		for (j = 0; j < num_vreg; j++) {

			if (!strcmp(soc_info->rgltr_name[j],
				mixed_name)) {
				CAM_DBG(CAM_SENSOR,
					"i:%d j:%d cam_mixed:%s", i, j, mixed_name);
				power_setting[i].seq_val = j;

				if (VALIDATE_VOLTAGE(
					soc_info->rgltr_min_volt[j],
					soc_info->rgltr_max_volt[j],
					power_setting[i].config_val)) {
					soc_info->rgltr_min_volt[j] =
					soc_info->rgltr_max_volt[j] =
					power_setting[i].config_val;
				}
				break;
			}
		}
		if (j == num_vreg)
			power_setting[i].seq_val = INVALID_VREG;
		break;
	default:
		break;
	}
}

int vendor_sensor_util_init_gpio_pin_tbl(
	struct cam_hw_soc_info *soc_info,
	struct msm_camera_gpio_num_info **pgpio_num_info)
{
	int i = 0;
	int type = SENSOR_CUSTOM_MIXED_PIN1;
	char mixed_name[MIXED_PIN_NAME_LEN] = {0};
	int rc = 0, val = 0;
	uint32_t gpio_array_size;
	struct device_node *of_node = soc_info->dev->of_node;
	struct cam_soc_gpio_data *gconf = soc_info->gpio_data;
	struct msm_camera_gpio_num_info *gpio_num_info = NULL;

	gpio_array_size = gconf->cam_gpio_common_tbl_size;
	gpio_num_info = *pgpio_num_info;

	rc = of_property_read_u32(of_node, "gpio-btb-det", &val);
	if (rc != -EINVAL) {
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "read gpio-btb-det failed rc %d", rc);
			return rc;
		} else if (val >= gpio_array_size) {
			CAM_ERR(CAM_SENSOR, "gpio-btb-det invalid %d", val);
			gpio_num_info->valid[SENSOR_BTB_DET] = 0;
		} else {
			gpio_num_info->gpio_num[SENSOR_BTB_DET] =
				gconf->cam_gpio_common_tbl[val].gpio;
			gpio_num_info->valid[SENSOR_BTB_DET] = 1;
		}

		CAM_DBG(CAM_SENSOR, "gpio-btb-det %d",
			gpio_num_info->gpio_num[SENSOR_BTB_DET]);
	}

	rc = of_property_read_u32(of_node, "gpio-vaf-pwctrl", &val);
	if (rc != -EINVAL) {
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "read gpio-vaf-pwctrl failed rc %d", rc);
			return rc;
		} else if (val >= gpio_array_size) {
			CAM_ERR(CAM_SENSOR, "gpio-vaf-pwctrl invalid %d", val);
			gpio_num_info->valid[SENSOR_VAF_PWCTRL] = 0;
		} else {
			gpio_num_info->gpio_num[SENSOR_VAF_PWCTRL] =
				gconf->cam_gpio_common_tbl[val].gpio;
			gpio_num_info->valid[SENSOR_VAF_PWCTRL] = 1;
		}

		CAM_INFO(CAM_SENSOR, "gpio-vaf-pwctrl %d",
			gpio_num_info->gpio_num[SENSOR_VAF_PWCTRL]);
	}

	rc = of_property_read_u32(of_node, "gpio-custom3", &val);
	if (rc != -EINVAL) {
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"read gpio-custom3 failed rc %d", rc);
			return rc;
		} else if (val >= gpio_array_size) {
			CAM_ERR(CAM_SENSOR, "gpio-custom3 invalid %d", val);
			gpio_num_info->valid[SENSOR_CUSTOM_GPIO3] = 0;
		} else {
			gpio_num_info->gpio_num[SENSOR_CUSTOM_GPIO3] =
				gconf->cam_gpio_common_tbl[val].gpio;
			gpio_num_info->valid[SENSOR_CUSTOM_GPIO3] = 1;
		}

		CAM_INFO(CAM_SENSOR, "gpio-custom3 %d",
			gpio_num_info->gpio_num[SENSOR_CUSTOM_GPIO3]);
	} else {
		rc = 0;
	}

	for (; i < MIXED_PIN_MAX_NUM; i++) {
		type = SENSOR_CUSTOM_MIXED_PIN1 + i;
		rc = sprintf(mixed_name, "gpio-mixed%d", i + 1);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "sprintf fail with rc=%d", rc);
			rc = 0;
			break;
		}
		rc = of_property_read_u32(of_node, mixed_name, &val);
		if (rc != -EINVAL) {
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"read gpio-mixed failed rc %d", rc);
				return rc;
			} else if (val >= gpio_array_size) {
				CAM_ERR(CAM_SENSOR, "gpio-mixed invalid %d", val);
				gpio_num_info->valid[type] = 0;
			} else {
				gpio_num_info->gpio_num[type] =
					gconf->cam_gpio_common_tbl[val].gpio;
				gpio_num_info->valid[type] = 1;
			}

			CAM_INFO(CAM_SENSOR, "gpio-mixed type:%d, num:%d",
				type, gpio_num_info->gpio_num[type]);
		} else {
			rc = 0;
		}
	}
	return rc;
}

void vendor_sensor_power_up(struct cam_sensor_power_ctrl_t *ctrl,
		struct cam_hw_soc_info *soc_info)
{
	if (soc_info->boost5v_enable) {
		boost_5v_enable(true, soc_info->dev_name);
		CAM_INFO(CAM_SENSOR, "Camera %s enable vboost 5V", soc_info->dev_name);
	}
}

void vendor_sensor_power_down(struct cam_sensor_power_ctrl_t *ctrl,
		struct cam_hw_soc_info *soc_info)
{
	if (soc_info->boost5v_enable) {
		boost_5v_enable(false, soc_info->dev_name);
		CAM_INFO(CAM_SENSOR, "Camera%s disable vboost 5V", soc_info->dev_name);
	}
}

int vendor_sensor_core_power_reset(struct cam_sensor_power_ctrl_t *ctrl,
		struct cam_hw_soc_info *soc_info, int val)
{
	int rc = 0;
	struct msm_camera_gpio_num_info *gpio_num_info = NULL;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;
	int state;

	gpio_num_info = ctrl->gpio_num_info;
	s_ctrl = container_of(soc_info, struct cam_sensor_ctrl_t, soc_info);

	if (soc_info->btb_check_enable) {
		state = gpio_get_value(gpio_num_info->gpio_num[SENSOR_RESET]) ? 1 : 0;
		CAM_INFO(CAM_SENSOR, "%s camera gpio:SENSOR_RESET read val is %d",
			soc_info->dev_name, state);
		if (state) {
			CAM_ERR(CAM_SENSOR, "%s camera btb check fail, position_id is %d",
				soc_info->dev_name,
				s_ctrl->sensordata->slave_info.sensor_probe_info.position_id);
			vendor_cam_hiview_handle(BTB_CHECK_ERR, s_ctrl,
				"(gpio:SENSOR_RESET is HIGH_STATE)");
		}

		if (gpio_num_info->valid[SENSOR_BTB_DET]) {
			state = gpio_get_value(gpio_num_info->gpio_num[SENSOR_BTB_DET]) ? 1 : 0;
			CAM_INFO(CAM_SENSOR, "%s camera gpio:SENSOR_BTB_DET read val is %d",
				soc_info->dev_name, state);
			if (state) {
				CAM_ERR(CAM_SENSOR, "%s camera btb check fail, position_id is %d",
					soc_info->dev_name,
					s_ctrl->sensordata->slave_info.sensor_probe_info.position_id);
				vendor_cam_hiview_handle(BTB_CHECK_ERR, s_ctrl,
					"(gpio:SENSOR_BTB_DET is HIGH_STATE)");
			}
		}

		rc = gpio_direction_output(gpio_num_info->gpio_num[SENSOR_RESET], 0);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "%s camera gpio:SENSOR_RESET set direction output fail",
				soc_info->dev_name);
			return rc;
		}
	}
	rc = msm_cam_sensor_handle_reg_gpio(SENSOR_RESET, gpio_num_info, val);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR,
			"Error in handling VREG GPIO");
		return rc;
	}
	return rc;
}

MODULE_DESCRIPTION("vendor sensor utility");
MODULE_LICENSE("GPL v2");
