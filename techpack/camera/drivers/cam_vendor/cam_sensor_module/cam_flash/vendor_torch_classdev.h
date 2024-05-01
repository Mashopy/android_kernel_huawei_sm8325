/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: cam torch classdev driver
 */
#ifndef _CAM_FLASH_CLASSDEV_H_
#define _CAM_FLASH_CLASSDEV_H_

#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/device.h>

#define LOWEST_CURRENT_LEVEL 3
struct torch_classdev_data {
	struct device *dev;
	struct led_classdev cdev_torch;
	struct led_trigger *torch_trigger;
	struct led_trigger *switch_trigger;
	struct mutex lock;
};

int cam_torch_classdev_register(struct platform_device *pdev, void *data);

#endif /* _CAM_FLASH_CLASSDEV_H_ */