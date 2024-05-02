/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor actuator core driver
 */
#ifndef _VENDOR_ACTUATOR_CORE_H_
#define _VENDOR_ACTUATOR_CORE_H_

#include "cam_actuator_dev.h"
#include <chipset_common/camera/cam_debug/cam_debug_common.h>

int32_t vendor_actuator_driver_cmd(struct cam_actuator_ctrl_t *a_ctrl, void *arg);

int32_t vendor_actuator_apply_settings(struct cam_actuator_ctrl_t *a_ctrl,
	struct i2c_settings_array *i2c_set);

void vendor_actuator_revert_byte(struct cam_actuator_ctrl_t *a_ctrl,
	struct i2c_settings_list *i2c_list);

void vendor_actuator_pkt_parser(struct cam_actuator_ctrl_t *a_ctrl,
	struct cam_cmd_i2c_info *i2c_info);

int32_t vendor_actuator_apply_request(struct cam_actuator_ctrl_t *a_ctrl,
	struct cam_req_mgr_apply_request *apply);

void vendor_actuator_update_req_mgr(struct cam_actuator_ctrl_t *a_ctrl,
	struct cam_packet *csl_packet, struct cam_req_mgr_add_request *add_req);

void vendor_actuator_init_pwctrl_state(struct cam_actuator_ctrl_t *a_ctrl);

int32_t vendor_actuator_retry_dw9781(struct camera_io_master *master_info,
	struct i2c_settings_list *i2c_list, int32_t ret_val);

#endif
/* _VENDOR_ACTUATOR_CORE_H_ */
