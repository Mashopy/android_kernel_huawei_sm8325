/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 *
 * Description: The Linux Foundation
 *
 * Create: 2021-12-09
 */

#ifndef __VENDOR_CAM_DEFS_H__
#define __VENDOR_CAM_DEFS_H__

#include <linux/ioctl.h>

/* camera op codes vendor */
#define CAM_VENDOR_OPCODE_BASE    0x250
#define CAM_REG_CONFIG            (CAM_VENDOR_OPCODE_BASE + 0x1) /* custom i2c write/read reg */
#define CAM_DUMP_REG_CONFIG       (CAM_VENDOR_OPCODE_BASE + 0x2) /* sensor i2c dump reg */
#define CAM_DATA_CONFIG           (CAM_VENDOR_OPCODE_BASE + 0x3) /* custom data config */

/* camera IOCTL vendor */
#define  VIDIOC_CAM_CONTROL_VENDOR \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 0x30, struct cam_control)

#endif
/* __VENDOR_CAM_DEFS_H__ */
