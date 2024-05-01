/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: cam torch pmic driver
 */
#include <linux/mutex.h>
#include "vendor_torch_pmic.h"
#include "cam_flash_dev.h"
#include "cam_flash_soc.h"
#include "cam_flash_core.h"
#include "cam_common_util.h"
#include "cam_res_mgr_api.h"
#include "cam_flash_test.h"

enum {
	PMIC_FLASH_ON_VAL = 5,
	PMIC_FLASH_OFF_VAL = 0,
};

static struct mutex pmic_flash_test_lock;

static void set_cam_torch_switch(struct device *dev, int value)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct cam_flash_ctrl *fctrl = platform_get_drvdata(pdev);

	if (!fctrl->torch_trigger[0]) {
		CAM_ERR(CAM_FLASH, "torch_trigger is Null");
		return;
	}

	mutex_lock(&pmic_flash_test_lock);
	cam_res_mgr_led_trigger_event(fctrl->torch_trigger[0], (enum led_brightness)value);

	if (!fctrl->switch_trigger) {
		CAM_ERR(CAM_FLASH, "No switch_trigger");
		mutex_unlock(&pmic_flash_test_lock);
		return;
	}

	if (value > 0)
		cam_res_mgr_led_trigger_event(fctrl->switch_trigger,
			(enum led_brightness)LED_SWITCH_ON);
	else
		cam_res_mgr_led_trigger_event(fctrl->switch_trigger,
			(enum led_brightness)LED_SWITCH_OFF);

	mutex_unlock(&pmic_flash_test_lock);
}

static void back_cam_torch_turn_on(struct device *dev)
{
	set_cam_torch_switch(dev, PMIC_FLASH_ON_VAL);
}

static void back_cam_torch_turn_off(struct device *dev)
{
	set_cam_torch_switch(dev, PMIC_FLASH_OFF_VAL);
}

int register_pmic_flashlight_data(struct device *dev)
{
	int rc;
	struct flashlight_test_operations flashlight_back_ops = {
		.dev = dev,
		.cam_torch_turn_on = back_cam_torch_turn_on,
		.cam_torch_turn_off = back_cam_torch_turn_off,
	};

	rc = cam_flashlight_classdev_register(&flashlight_back_ops, FLASH_LIGHT_BACK, PMIC_CTRL_TYPE);
	if (rc < 0) {
		CAM_ERR(CAM_FLASH, "register classdev failed");
		return rc;
	}

	return 0;
}