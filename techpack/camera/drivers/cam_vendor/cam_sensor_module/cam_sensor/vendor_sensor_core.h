/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor sensor core driver
 */
#ifndef _VENDOR_SENSOR_CORE_H_
#define _VENDOR_SENSOR_CORE_H_

#include "cam_sensor_dev.h"
#include "media/custom/vendor_cam_sensor.h"

struct sensor_i2c_dump_data {
	uint16_t data_num;
	struct vendor_sensor_i2c_dump_info *dump_info;
};

struct vendor_sensor_global_data_t {
	uint8_t module_match_count;
	uint16_t pre_sensor_id;
	uint32_t pre_slot_id;
	struct sensor_i2c_dump_data dump_data;
};

int32_t vendor_sensor_driver_cmd(struct cam_sensor_ctrl_t *s_ctrl, void *arg);
int32_t vendor_module_match_id(struct cam_sensor_ctrl_t *s_ctrl);
void vendor_sensor_dump_reg(struct cam_sensor_ctrl_t *s_ctrl);
int32_t vendor_cam_handle_probe_reg_settings(uint32_t num_cmd_buf,
	struct cam_sensor_ctrl_t *s_ctrl, struct cam_cmd_buf_desc *cmd_desc);
void vendor_sensor_query_cap(struct cam_sensor_ctrl_t *s_ctrl,
	struct  cam_sensor_query_cap *query_cap);
void vendor_sensor_update_slave_info(struct cam_cmd_probe *probe_info,
	struct cam_sensor_ctrl_t *s_ctrl);
void vendor_sensor_apply_reg_settings(struct cam_sensor_ctrl_t *s_ctrl);
void vendor_sensor_start_dev(struct cam_sensor_ctrl_t *s_ctrl);
void vendor_sensor_stop_dev(struct cam_sensor_ctrl_t *s_ctrl);
void vendor_sensor_get_sub_module_index(struct device_node *of_node,
	struct cam_sensor_board_info *s_info);
#endif
/* _VENDOR_SENSOR_CORE_H_ */
