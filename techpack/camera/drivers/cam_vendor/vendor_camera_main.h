/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor_camera_main function
 *
 */
#ifndef VENDOR_CAMERA_MAIN_H
#define VENDOR_CAMERA_MAIN_H

#include "huawei_cam_class.h"
#include "cam_debug_common.h"
#include "vendor_debug_adapter.h"
#include "vendor_laser_dev.h"
#include "vendor_cfg_dev.h"
#include "vendor_cam_fs.h"
#include "vendor_ois_core.h"
#include "vendor_ctrl.h"

struct vendor_submodule_component {
	int (*init)(void);
	void (*exit)(void);
};

struct vendor_submodule_component vendor_module[] = {
	/* --- chipset common driver entrance --- */
	{&camera_class_init, &camera_class_exit},
	{&camera_debug_init, &camera_debug_exit},
	/* --- qcom private driver entrance ---*/
	{&cam_crtl_init, &cam_crtl_exit},
	{&debug_adapter_init, &debug_adapter_exit},
	{&cam_laser_driver_init, &cam_laser_driver_exit},
	{&cam_cfgdev_driver_init, &cam_cfgdev_driver_exit},
	{&camerafs_module_init, &camerafs_module_deinit},
	/* ois init here */
	{&aw86006_driver_init, &aw86006_driver_exit},
	{&dw9781_driver_init, &dw9781_driver_exit},
	{&dw9787_driver_init, &dw9787_driver_exit},
	{&lc898129_driver_init, &lc898129_driver_exit},
	{&rumbas10_driver_init, &rumbas10_driver_exit},
};

#endif /* VENDOR_CAMERA_MAIN_H */
