/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2023. All rights reserved.
 *
 * Description: xpmic driver
 */

#ifndef __XPMIC_REGISTERS_H__
#define __XPMIC_REGISTERS_H__

enum {
	XPMIC_GPIO_PULL_LOW = 0,
	XPMIC_GPIO_PULL_HIGH,
	XPMIC_GPIO_MAX_OPS,
};

int xpmic_gpio_ops(int gpio, int ops);

#endif
/* __XPMIC_REGISTERS_H__ */
