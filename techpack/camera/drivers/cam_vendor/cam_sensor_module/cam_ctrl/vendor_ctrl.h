/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vcm, ois, sensor ctrl driver
 */
#ifndef _VENDOR_CTRL_H_
#define _VENDOR_CTRL_H_

#include "vendor_debug_adapter.h"

struct camera_elem_t {
	struct list_head node;
	int slot_id;
	int vcm_id;
	int ois_id;
	int eeprom_id;
	int phy_id;
	int va_id;
	struct cam_sensor_ctrl_t *s_ctrl;
	struct cam_actuator_ctrl_t *a_ctrl;
	struct cam_ois_ctrl_t *o_ctrl;
};

enum custom_mode {
	SHUTDOWN_MODE,
	RESET,
};

struct custom_config_info {
	unsigned short sensor_slotID;
	enum dev_type dev_type;
	enum custom_mode config_mode;
};

struct cam_actuator_ctrl_t* vendor_get_vcm_ctrl(int slot_id);
struct cam_sensor_ctrl_t* vendor_get_sensor_ctrl(int slot_id, int64_t device_type);
struct cam_ois_ctrl_t* vendor_get_ois_ctrl(int slot_id);

void vendor_actuator_ctrl_register(struct cam_actuator_ctrl_t *a_ctrl);
void vendor_ois_ctrl_register(struct cam_ois_ctrl_t *o_ctrl);
void vendor_sensor_ctrl_register(struct cam_sensor_ctrl_t *s_ctrl);

int cam_crtl_init(void);
void cam_crtl_exit(void);
#endif
