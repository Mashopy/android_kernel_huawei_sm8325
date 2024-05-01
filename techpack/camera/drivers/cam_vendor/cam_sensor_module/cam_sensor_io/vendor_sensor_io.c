/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor sensor io driver
 */
#include "vendor_sensor_io.h"
#include "cam_debug_util.h"
#include "cam_sensor_i2c.h"

int32_t vendor_camera_io_dev_read(struct camera_io_master *io_master_info,
	uint32_t addr, uint32_t *data,
	enum camera_sensor_i2c_type addr_type,
	enum camera_sensor_i2c_type data_type)
{
	if (!io_master_info) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	if (io_master_info->master_type == CCI_MASTER) {
		return cam_cci_i2c_read(io_master_info->cci_client,
			addr, data, addr_type, data_type);
	} else if (io_master_info->master_type == I2C_MASTER) {
		return cam_qup_i2c_read(io_master_info->client,
			addr, data, addr_type, data_type);
	} else if (io_master_info->master_type == SPI_MASTER) {
		return cam_spi_read(io_master_info,
			addr, data, addr_type, data_type);
	} else {
		CAM_ERR(CAM_SENSOR, "Invalid Comm. Master:%d",
			io_master_info->master_type);
		return -EINVAL;
	}
}