/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor Camera fs virtul device
 *
 */
#ifndef _VENDOR_CAM_FS_H_
#define _VENDOR_CAM_FS_H_

struct cameraprotect_info {
	char actuator_status;
	char reserve[3];
	int actuator_shake_time;
};

void hw_actuator_protect_work(struct cameraprotect_info *protect_info);

int camerafs_module_init(void);

/**
 * @brief : API to remove JPEG ENC Hw from platform framework.
 */
void camerafs_module_deinit(void);

#endif