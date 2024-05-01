/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor Camera fs virtul device
 *
 */
#ifndef _VENDOR_CAM_FS_H_
#define _VENDOR_CAM_FS_H_

int camerafs_module_init(void);

/**
 * @brief : API to remove JPEG ENC Hw from platform framework.
 */
void camerafs_module_deinit(void);

#endif