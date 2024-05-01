/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: cam torch pmic driver
 */
#ifndef _VENDOR_FLASH_CLASS_H_
#define _VENDOR_FLASH_CLASS_H_

#include <linux/device.h>

int register_pmic_flashlight_data(struct device *dev);

#endif