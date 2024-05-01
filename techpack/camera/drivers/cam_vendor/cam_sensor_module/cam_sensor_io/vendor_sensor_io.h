/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor sensor io driver
 */
#ifndef _VENDOR_SENSOR_IO_H_
#define _VENDOR_SENSOR_IO_H_

#include "cam_sensor_io.h"

/**
 * @io_master_info: I2C/SPI master information
 * @addr: I2C address
 * @data: I2C data
 * @addr_type: I2C addr_type
 * @data_type: I2C data type
 *
 * This API abstracts read functionality based on master type, not support dmd error report
 */
int32_t vendor_camera_io_dev_read(struct camera_io_master *io_master_info,
	uint32_t addr, uint32_t *data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type);

#endif /* _VENDOR_SENSOR_IO_H_ */
