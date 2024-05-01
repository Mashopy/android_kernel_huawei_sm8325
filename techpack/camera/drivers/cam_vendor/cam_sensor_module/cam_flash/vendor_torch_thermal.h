/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: cam torch thermal protect classdev driver
 */
#ifndef _VENDOR_TORCH_THERMAL_H_
#define _VENDOR_TORCH_THERMAL_H_

#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/string.h>

enum flash_lock_status {
	FLASH_STATUS_UNLOCK,
	FLASH_STATUS_LOCKED,
};

int cam_torch_thermal_protect_get_value(void);
int cam_torch_thermal_protect_classdev_register(struct platform_device *pdev,
	void *data);

#endif /* _VENDOR_TORCH_THERMAL_H_ */
