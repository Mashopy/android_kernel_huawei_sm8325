/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor sensor utility
 */
#ifndef _VENDOR_SENSOR_UTIL_H_
#define _VENDOR_SENSOR_UTIL_H_

#include <linux/kernel.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include "cam_sensor_io.h"
#include "media/custom/vendor_cam_sensor.h"

#define MIXED_PIN_MAX_NUM      5
#define MIXED_PIN_NAME_LEN     12
#define CONFIG_GPIO            0
#define CONFIG_LDO             1

/* reg write/read for HAL */
int32_t vendor_cam_write_reg(struct camera_io_master *io_master_info,
	struct cam_sensor_config_reg *cam_write_reg_data);
int32_t vendor_cam_read_reg(struct camera_io_master *io_master_info,
	struct cam_sensor_config_reg *cam_read_reg_data);
int vendor_sensor_util_i2c_revert_byte(struct i2c_settings_list *i2c_list);
void vendor_fill_vreg_params(
	struct cam_hw_soc_info *soc_info,
	struct cam_sensor_power_setting *power_setting, uint16_t i);
int vendor_sensor_util_init_gpio_pin_tbl(
	struct cam_hw_soc_info *soc_info,
	struct msm_camera_gpio_num_info **pgpio_num_info);
int vendor_sensor_core_power_reset(struct cam_sensor_power_ctrl_t *ctrl,
	struct cam_hw_soc_info *soc_info, int val);
void vendor_sensor_power_up(struct cam_sensor_power_ctrl_t *ctrl,
	struct cam_hw_soc_info *soc_info);
void vendor_sensor_power_down(struct cam_sensor_power_ctrl_t *ctrl,
	struct cam_hw_soc_info *soc_info);

#endif
/* _VENDOR_SENSOR_UTIL_H_ */
