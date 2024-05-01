/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor eeprom core driver
 */
#include <linux/timer.h>
#include <media/cam_sensor.h>
#include "cam_common_util.h"
#include "vendor_eeprom_core.h"

#define DW9781_EEPROM_LOGIC_SLAVE_ID   (0xE4 >> 1)
#define DW9781_EEPROM_SLAVE_ID         (0xA4 >> 1)
#define DW9781_LOGIC_RESET_ADDRESS     0xD002
#define DW9781_LOGIC_RESET_VAL         0x0001

static int32_t eeprom_w_reg16_val16(struct cam_eeprom_ctrl_t *e_ctrl,
	uint16_t reg, uint16_t value)
{
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct cam_sensor_i2c_reg_array i2c_reg_array;
	int32_t rc = 0;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.size = 1;
	i2c_reg_setting.delay = 0;

	i2c_reg_setting.reg_setting = &i2c_reg_array;

	i2c_reg_setting.reg_setting[0].reg_addr = reg;
	i2c_reg_setting.reg_setting[0].reg_data = value;
	i2c_reg_setting.reg_setting[0].delay = 0;
	i2c_reg_setting.reg_setting[0].data_mask = 0;

	CAM_INFO(CAM_EEPROM, "eeprom write reg:%x, value:%x", reg, value);
	rc = camera_io_dev_write(&(e_ctrl->io_master_info),
		&i2c_reg_setting);
	if (rc < 0)
		CAM_ERR(CAM_EEPROM, "eeprom write failed reg:%x,value:%x,rc:%d", reg, value, rc);

	return rc;
}

bool vendor_eeprom_read_need_retry(struct cam_eeprom_ctrl_t *e_ctrl)
{
	int32_t rc = 0;

	if (e_ctrl->io_master_info.cci_client->sid == DW9781_EEPROM_SLAVE_ID) {
		e_ctrl->io_master_info.cci_client->sid = DW9781_EEPROM_LOGIC_SLAVE_ID;
		CAM_INFO(CAM_EEPROM, "update slave id = 0x%x", DW9781_EEPROM_LOGIC_SLAVE_ID);
		rc = eeprom_w_reg16_val16(e_ctrl,
			DW9781_LOGIC_RESET_ADDRESS, DW9781_LOGIC_RESET_VAL);
		if (rc)
			CAM_ERR(CAM_EEPROM, "dw9781b ois logic reset failed");

		usleep_range(10 * 1000, 11 * 1000); /* delay 10ms after dw9781 ois reset */
		e_ctrl->io_master_info.cci_client->sid = DW9781_EEPROM_SLAVE_ID;
		return true;
	}
	return false;
}
