/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor actuator dev driver
 */
#ifndef _VENDOR_ACTUATOR_DEV_H_
#define _VENDOR_ACTUATOR_DEV_H_

#include "cam_sensor_cmn_header.h"

enum vendor_actuator_type {
	CAM_ACTUATOR_VCM,
	CAM_ACTUATOR_BIVCM,
	CAM_ACTUATOR_VA,
};

struct vendor_actuator_ctrl {
	uint32_t dual_slave_addr;
	uint32_t is_big_endian;
	enum vendor_actuator_type dev_type;
	bool     is_sof_applied;
	int64_t  frame_id_applied;
	uint8_t  low_power_flag[MAX_PER_FRAME_ARRAY];
	uint8_t  pre_low_power_flag;
};

#endif
/* _VENDOR_ACTUATOR_DEV_H_ */
