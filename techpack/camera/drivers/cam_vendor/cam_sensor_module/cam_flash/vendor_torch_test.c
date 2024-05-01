/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: cam torch test driver
 */
#include "vendor_torch_pmic.h"

int register_test_flashlight_data(struct device *dev)
{
	int rc;

	rc = register_pmic_flashlight_data(dev);
	if (rc < 0)
		pr_err("%s, register_pmic_flashlight_data failed\n", __func__);

	return rc;
}