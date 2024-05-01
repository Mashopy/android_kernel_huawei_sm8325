/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: debug adapter driver
 */
#ifndef _VENDOR_DEBUG_ADAPTER_H_
#define _VENDOR_DEBUG_ADAPTER_H_

#include <chipset_common/camera/cam_debug/cam_debug_common.h>
#include <chipset_common/camera/cam_debug/cam_debug_log.h>
#include "cam_sensor_dev.h"

int i2c_read(struct debug_msg *recv_data, struct debug_msg *send_data,
	struct camera_io_master *io_master_info);
int i2c_write(struct debug_msg *recv_data,
	struct camera_io_master *io_master_info);

#ifdef CONFIG_SPECTRA_DEBUG
void vendor_actuator_debug_update(struct dev_msg_t* debug_dev);
void vendor_ois_debug_update(struct dev_msg_t* debug_dev);
void vendor_sensor_debug_update(struct cam_sensor_ctrl_t *s_ctrl, struct dev_msg_t* debug_dev);
int debug_adapter_init(void);
void debug_adapter_exit(void);
#else
static inline void vendor_actuator_debug_update(struct dev_msg_t* debug_dev)
{
}
static inline void vendor_ois_debug_update(struct dev_msg_t* debug_dev)
{
}
static inline void vendor_sensor_debug_update(struct cam_sensor_ctrl_t *s_ctrl, struct dev_msg_t* debug_dev)
{
}
static inline int debug_adapter_init(void)
{
    return 0;
}
static inline void debug_adapter_exit(void)
{
}
#endif

#endif
