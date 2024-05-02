/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor sensor core driver
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <securec.h>
#include "vendor_sensor_io.h"
#include "vendor_sensor_core.h"
#include "cam_sensor_cmn_header.h"
#include "cam_sensor_core.h"
#include "vendor_sensor_util.h"
#include "vendor_soc_util.h"
#include "vendor_ctrl.h"
#include "media/custom/vendor_cam_defs.h"
#include "media/custom/vendor_cam_sensor.h"

static struct vendor_sensor_global_data_t g_vendor_sensor_data;

static void vendor_sensor_save_dump_data(
	struct vendor_sensor_i2c_dump_info dump_info)
{
	uint16_t data_num, size;
	errno_t ret;
	struct vendor_sensor_i2c_dump_info *dump_info_temp = NULL;

	g_vendor_sensor_data.dump_data.data_num++;
	data_num = g_vendor_sensor_data.dump_data.data_num;
	size = sizeof(struct vendor_sensor_i2c_dump_info);
	CAM_INFO(CAM_SENSOR, "slotId:0x%x, sensorId:0x%x, data_num: %u, size:%u",
		dump_info.slot_id, dump_info.sensor_id, data_num, size);
	dump_info_temp =
		(struct vendor_sensor_i2c_dump_info *)kmalloc(size * data_num, GFP_KERNEL);
	if (dump_info_temp == NULL) {
		CAM_ERR(CAM_SENSOR, "kmalloc fail");
		return;
	}
	if (g_vendor_sensor_data.dump_data.dump_info == NULL) {
		ret = memcpy_s(dump_info_temp, size * data_num, &dump_info, size);
	} else {
		ret = memcpy_s(dump_info_temp, size * data_num,
			g_vendor_sensor_data.dump_data.dump_info, size * (data_num - 1));
		ret += memcpy_s(dump_info_temp + data_num - 1,
			size * data_num, &dump_info, size);
	}
	if (ret != EOK) {
		CAM_ERR(CAM_SENSOR, "memcpy fail, ret = %d", ret);
		kfree(dump_info_temp);
		return;
	}

	if (g_vendor_sensor_data.dump_data.dump_info != NULL)
		kfree(g_vendor_sensor_data.dump_data.dump_info);
	g_vendor_sensor_data.dump_data.dump_info = dump_info_temp;
}

int32_t vendor_sensor_driver_cmd(struct cam_sensor_ctrl_t *s_ctrl,
	void *arg)
{
	int rc = 0;
	struct cam_control *cmd = (struct cam_control *)arg;

	if (!s_ctrl || !cmd) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_SENSOR, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}
	CAM_DBG(CAM_SENSOR, "Opcode to sensor: %d", cmd->op_code);

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	switch (cmd->op_code) {
	case CAM_REG_CONFIG: {
		struct cam_sensor_config_reg sensor_reg_data;
		rc = copy_from_user(&sensor_reg_data,
			u64_to_user_ptr(cmd->handle),
			sizeof(sensor_reg_data));
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Failed copy from user");
			goto release_mutex;
		}

		if (sensor_reg_data.data_direct == CAM_WRITE_REG) {
			rc = vendor_cam_write_reg(&(s_ctrl->io_master_info),
				&sensor_reg_data);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR, "sensor %s set register failed",
					s_ctrl->device_name);
				goto release_mutex;
			}
		} else if (sensor_reg_data.data_direct == CAM_READ_REG) {
			rc = vendor_cam_read_reg(&(s_ctrl->io_master_info),
				&sensor_reg_data);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR, "sensor %s get register failed",
					s_ctrl->device_name);
				goto release_mutex;
			} else {
				if (copy_to_user(u64_to_user_ptr(cmd->handle),
					&sensor_reg_data,
					sizeof(sensor_reg_data))) {
					CAM_ERR(CAM_SENSOR, "Failed copy to user");
					rc = -EFAULT;
					goto release_mutex;
				}
			}
		} else {
			CAM_ERR(CAM_SENSOR, "Invalid data direct %d",
				sensor_reg_data.data_direct);
		}
	}
		break;
	case CAM_DUMP_REG_CONFIG: {
		struct vendor_sensor_i2c_dump_info dump_info;
		rc = copy_from_user(&dump_info,
			u64_to_user_ptr(cmd->handle), sizeof(dump_info));
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Failed copy from user");
			goto release_mutex;
		}
		dump_info.slot_id = s_ctrl->soc_info.index;
		vendor_sensor_save_dump_data(dump_info);
	}
		break;
	default:
		CAM_ERR(CAM_SENSOR, "Invalid Opcode %d", cmd->op_code);
	}

release_mutex:
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));

	return rc;
}

static int vendor_cloud_module_match_id(struct cam_camera_slave_info *slave_info)
{
	struct custom_module_qcom_info module_qcom_info = {0};

	/* Check module id default */
	CAM_INFO(CAM_SENSOR, "read module code from oem");
	if (slave_info->module_probe_info.cloud_module_code_data !=
		slave_info->module_probe_info.module_code_data) {
		CAM_WARN(CAM_SENSOR, "oem module code:0x%x expected module code:0x%x",
			slave_info->module_probe_info.cloud_module_code_data,
			slave_info->module_probe_info.module_code_data);
		return -ENODEV;
	}
	CAM_INFO(CAM_SENSOR, "read module code is the same as expected :0x%x",
		slave_info->module_probe_info.module_code_data);

	/* Check module qcom enable if need */
	module_qcom_info = slave_info->module_probe_info.module_qcom;
	if (module_qcom_info.module_qcom_flag) {
		CAM_INFO(CAM_SENSOR, "read module qcom enable info from oem");
		if (slave_info->module_probe_info.cloud_module_qcom_data !=
			module_qcom_info.module_qcom_data) {
			CAM_WARN(CAM_SENSOR, "oem qcom enable:0x%x expected :0x%x",
				slave_info->module_probe_info.cloud_module_qcom_data,
				module_qcom_info.module_qcom_data);
			return -ENODEV;
		}
		CAM_INFO(CAM_SENSOR, "read module enable is the same as expected :0x%x",
			module_qcom_info.module_qcom_data);
	}

	return 0;
}

static int vendor_eeprom_module_match_id(struct cam_sensor_ctrl_t *s_ctrl,
	struct cam_camera_slave_info *slave_info)
{
	int rc = 0;
	uint32_t module_code = 0;
	uint32_t module_version = 0;
	uint32_t module_qcom = 0;
	uint16_t tmp_sid;
	struct custom_module_version_info module_version_info = {0};
	struct custom_module_qcom_info module_qcom_info = {0};

	/* Save the SID value (sensor I2C address), modify the value for I/O read */
	tmp_sid = s_ctrl->io_master_info.cci_client->sid;
	s_ctrl->io_master_info.cci_client->sid =
		slave_info->module_probe_info.eeprom_slave_addr >> 1;

	/* Check module id default */
	rc = vendor_camera_io_dev_read(
		&(s_ctrl->io_master_info),
		slave_info->module_probe_info.module_code_addr,
		&module_code,
		slave_info->module_probe_info.module_code_addr_type,
		slave_info->module_probe_info.module_code_data_type);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "camera_io_dev_read moduleid occur error");
		goto end;
	}
	CAM_INFO(CAM_SENSOR, "read module code from eeprom");
	if (module_code != slave_info->module_probe_info.module_code_data) {
		CAM_WARN(CAM_SENSOR, "read module code:0x%x expected module code:0x%x",
			module_code, slave_info->module_probe_info.module_code_data);
		rc = -ENODEV;
		goto end;
	}

	CAM_INFO(CAM_SENSOR, "read module code is the same as expected module code:0x%x",
		slave_info->module_probe_info.module_code_data);

	/* Check module qcom enable if need */
	module_qcom_info = slave_info->module_probe_info.module_qcom;
	if (module_qcom_info.module_qcom_flag) {
		rc = vendor_camera_io_dev_read(
			&(s_ctrl->io_master_info),
			module_qcom_info.module_qcom_addr,
			&module_qcom,
			slave_info->module_probe_info.module_code_addr_type,
			slave_info->module_probe_info.module_code_data_type);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "camera_io_dev_read module qcom occur error");
			goto end;
		}
		CAM_INFO(CAM_SENSOR, "read module qcom enable info from eeprom");
		if (module_qcom != module_qcom_info.module_qcom_data) {
			CAM_WARN(CAM_SENSOR, "read module qcom enable:0x%x expected :0x%x",
				module_qcom, module_qcom_info.module_qcom_data);
			rc = -ENODEV;
			goto end;
		}
		CAM_INFO(CAM_SENSOR, "read module qcom enable is the same as expected :0x%x",
			module_qcom_info.module_qcom_data);
	}

	/* Check module vesion if need */
	module_version_info = slave_info->module_probe_info.module_version;
	if (module_version_info.module_version_flag) {
		rc = vendor_camera_io_dev_read(
			&(s_ctrl->io_master_info),
			module_version_info.module_version_addr,
			&module_version,
			slave_info->module_probe_info.module_code_addr_type,
			slave_info->module_probe_info.module_code_data_type);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "camera_io_dev_read module version occur error");
			goto end;
		}
		if (module_version != module_version_info.module_version_data) {
			CAM_WARN(CAM_SENSOR, "read module version:0x%x expected :0x%x",
				module_version, module_version_info.module_version_data);
			rc = -ENODEV;
			goto end;
		}
		CAM_INFO(CAM_SENSOR, "read version is same as expected :0x%x",
			module_version);
	}

end:
	s_ctrl->io_master_info.cci_client->sid = tmp_sid;
	return rc;
}

static int vendor_module_probe(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	struct cam_camera_slave_info *slave_info = NULL;

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "s_ctrl is null");
		return -EINVAL;
	}

	slave_info = &(s_ctrl->sensordata->slave_info);
	if (!slave_info) {
		CAM_ERR(CAM_SENSOR, " slave_info is null");
		return -EINVAL;
	}

	if (!(slave_info->module_probe_info.module_code_support)) {
		CAM_INFO(CAM_SENSOR, "skip check module code");
		return rc;
	}

	/* reset module_match_count When the positions or sensors are different */
	if (s_ctrl->soc_info.index != g_vendor_sensor_data.pre_slot_id ||
		slave_info->sensor_id != g_vendor_sensor_data.pre_sensor_id)
		g_vendor_sensor_data.module_match_count = 0;

	rc = vendor_eeprom_module_match_id(s_ctrl, slave_info);
	if (rc < 0 && slave_info->module_probe_info.cloud_module_code_support)
		rc = vendor_cloud_module_match_id(slave_info);

	CAM_INFO(CAM_SENSOR, "sensor module num:%u",
		slave_info->module_probe_info.sensor_module_num);
	/* sensor_module_num != 0 --> select default module when moduleid match fail */
	if (rc < 0 && slave_info->module_probe_info.sensor_module_num != 0) {
		g_vendor_sensor_data.module_match_count++;
		if (g_vendor_sensor_data.module_match_count ==
			slave_info->module_probe_info.sensor_module_num) {
			CAM_WARN(CAM_SENSOR, "module id match fail, use default module:0x%x",
				slave_info->module_probe_info.module_code_data);
			rc = 0;
		}
	}
	g_vendor_sensor_data.pre_slot_id = s_ctrl->soc_info.index;
	g_vendor_sensor_data.pre_sensor_id = slave_info->sensor_id;
	return rc;
}

static int vendor_sensor_read_imx_fuseid(struct cam_sensor_ctrl_t *s_ctrl,
	struct custom_sensor_fuseid_info *sensor_fuseid_info, uint8_t *value)
{
	int rc = 0;
	int32_t num_bytes;

	/* poll status check */
	rc = camera_io_dev_poll(
		&(s_ctrl->io_master_info),
		sensor_fuseid_info->status_addr,
		sensor_fuseid_info->status_val,
		sensor_fuseid_info->status_mask,
		sensor_fuseid_info->fuseid_addr_type,
		sensor_fuseid_info->fuseid_data_type,
		5);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "i2c poll apply setting Fail: %d", rc);
		return rc;
	}

	num_bytes = (int32_t)(sensor_fuseid_info->fuseid_len);
	rc = camera_io_dev_read_seq(
		&(s_ctrl->io_master_info), sensor_fuseid_info->fuseid_addr,
		value, sensor_fuseid_info->fuseid_addr_type,
		sensor_fuseid_info->fuseid_data_type, num_bytes);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Failed camera_io_dev_read_seq:%d", rc);
	}
	return rc;
}

static int vendor_sensor_read_gc_fuseid(struct cam_sensor_ctrl_t *s_ctrl,
	struct custom_sensor_fuseid_info *sensor_fuseid_info, uint8_t *value)
{
	int rc = 0;
	int32_t num_bytes;
	struct cam_sensor_i2c_reg_setting i2c_reg_setting = {0};
	struct cam_sensor_i2c_reg_array i2c_reg_array = {0};
	uint8_t h_addr = 0x00;
	uint8_t l_addr = 0x00;
	int8_t i = 0;

	i2c_reg_setting.size = 1;
	i2c_reg_setting.addr_type = sensor_fuseid_info->fuseid_addr_type;
	i2c_reg_setting.data_type = sensor_fuseid_info->fuseid_data_type;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = &i2c_reg_array;
	i2c_reg_array.delay = 0;
	i2c_reg_array.data_mask = 0;

	num_bytes = (int32_t)(sensor_fuseid_info->fuseid_len);
	for (i = 0; i < num_bytes; i++) {
		h_addr = (sensor_fuseid_info->fuseid_addr & 0xff00) >> 8;
		l_addr = sensor_fuseid_info->fuseid_addr & 0x00ff;
		i2c_reg_array.reg_addr = sensor_fuseid_info->target_addr_h;
		i2c_reg_array.reg_data = h_addr;
		rc = camera_io_dev_write(&(s_ctrl->io_master_info), &i2c_reg_setting);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "camera_io_dev_write Failed rc:%d", rc);
			return rc;
		}
		i2c_reg_array.reg_addr = sensor_fuseid_info->target_addr_l;
		i2c_reg_array.reg_data = l_addr;
		rc = camera_io_dev_write(&(s_ctrl->io_master_info), &i2c_reg_setting);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "camera_io_dev_write Failed rc:%d", rc);
			return rc;
		}
		i2c_reg_array.reg_addr = sensor_fuseid_info->read_enable_addr;
		i2c_reg_array.reg_data = sensor_fuseid_info->read_enable_val;
		rc = camera_io_dev_write(&(s_ctrl->io_master_info), &i2c_reg_setting);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "camera_io_dev_write Failed rc:%d", rc);
			return rc;
		}
		rc = camera_io_dev_read_seq(
			&(s_ctrl->io_master_info), sensor_fuseid_info->read_addr,
			&value[i], sensor_fuseid_info->fuseid_addr_type,
			sensor_fuseid_info->fuseid_data_type, 1);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "camera_io_dev_read_seq Failed rc:%d", rc);
			return rc;
		}
		sensor_fuseid_info->fuseid_addr += sensor_fuseid_info->step;
	}
	return rc;
}

static int vendor_sensor_read_ov_fuseid(struct cam_sensor_ctrl_t *s_ctrl,
	struct custom_sensor_fuseid_info *sensor_fuseid_info, uint8_t *value)
{
	int rc = 0;
	int32_t num_bytes;

	num_bytes = (int32_t)(sensor_fuseid_info->fuseid_len);
	rc = camera_io_dev_read_seq(
		&(s_ctrl->io_master_info),
		sensor_fuseid_info->fuseid_addr,
		value,
		sensor_fuseid_info->fuseid_addr_type,
		sensor_fuseid_info->fuseid_data_type,
		num_bytes);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Failed camera_io_dev_read_seq:%d", rc);
	}
	return rc;
}

static int vendor_sensor_read_fuseid_retry(struct cam_sensor_ctrl_t *s_ctrl,
	struct custom_sensor_fuseid_info *sensor_fuseid_info, int32_t num_bytes, uint8_t *value)
{
	int rc = 0;
	int8_t i = 0;
	uint8_t *value_compare = NULL;

	value_compare = kzalloc(num_bytes + 1, GFP_KERNEL);
	if (!value_compare) {
		CAM_ERR(CAM_SENSOR, "HwReadFuseid, kzalloc fail!");
		kfree(value_compare);
		return -EFAULT;
	}
	rc = vendor_sensor_read_ov_fuseid(s_ctrl, sensor_fuseid_info, value_compare);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Failed read ov fuseid:%d", rc);
		kfree(value_compare);
		return rc;
	}
	for (i = 0; (i < num_bytes) && (i < FUSEID_LEN_MAX); i++) {
		if (value_compare[i] != value[i]) {
			CAM_ERR(CAM_SENSOR, "Twice fuseids are different:id=%d,value_compare=0x%x,value=0x%x.try again",
				i, value_compare[i], value[i]);
			rc = vendor_sensor_read_ov_fuseid(s_ctrl, sensor_fuseid_info, value);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR, "Failed read ov fuseid:%d", rc);
				kfree(value_compare);
				return rc;
			}
			break;
		}
	}
	kfree(value_compare);
	return rc;
}

static int vendor_sensor_read_fuseid(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	int8_t i = 0;
	uint8_t *value = NULL;
	struct cam_camera_slave_info *slave_info = NULL;
	struct custom_sensor_fuseid_info *sensor_fuseid_info = NULL;
	int32_t num_bytes;

	slave_info = &(s_ctrl->sensordata->slave_info);
	sensor_fuseid_info = &(slave_info->sensor_probe_info.sensor_fuseid_info);
	CAM_DBG(CAM_SENSOR, "fuseid support:%d", sensor_fuseid_info->fuseid_flag);
	if (sensor_fuseid_info->fuseid_flag) {
		CAM_DBG(CAM_SENSOR, "sensorFuseidInfo : 0x%x status_val : %d len : %d",
			sensor_fuseid_info->fuseid_addr,
			sensor_fuseid_info->status_val, sensor_fuseid_info->fuseid_len);

		num_bytes = (int32_t)(sensor_fuseid_info->fuseid_len);
		value = kzalloc(num_bytes + 1, GFP_KERNEL);
		if (!value) {
			CAM_ERR(CAM_SENSOR, "HwReadFuseid, kzalloc fail!");
			return -EFAULT;
		}
		switch (sensor_fuseid_info->read_pattern) {
			case 0:
				/* read fuseid for imx */
				rc = vendor_sensor_read_imx_fuseid(s_ctrl,
					sensor_fuseid_info, value);
				if (rc < 0) {
					CAM_ERR(CAM_SENSOR, "Failed read imx fuseid:%d", rc);
					return rc;
				}
				break;
			case 1:
				/* read fuseid for hi */
				break;
			case 2:
				/* read fuseid for gc */
				rc = vendor_sensor_read_gc_fuseid(s_ctrl,
					sensor_fuseid_info, value);
				if (rc < 0) {
					CAM_ERR(CAM_SENSOR, "Failed read gc fuseid:%d", rc);
					return rc;
				}
				break;
			case 3:
				/* read fuseid for ov */
				rc = vendor_sensor_read_ov_fuseid(s_ctrl,
					sensor_fuseid_info, value);
				if (rc < 0) {
					CAM_ERR(CAM_SENSOR, "Failed read ov fuseid:%d", rc);
					return rc;
				}
				if (sensor_fuseid_info->is_retry)
					rc = vendor_sensor_read_fuseid_retry(s_ctrl, sensor_fuseid_info, num_bytes, value);
				break;
			default:
				break;
		}

		/* copy to fuseid_data */
		for (i = 0; (i < num_bytes) && (i < FUSEID_LEN_MAX); i++) {
			s_ctrl->sensordata->fuseid_data[i] = value[i];
			CAM_INFO(CAM_SENSOR, "sensorFuseidInfo, fuseid: %x", value[i]);
		}
	}

	return rc;
}

void vendor_sensor_dump_reg(struct cam_sensor_ctrl_t *s_ctrl)
{
	int i;
	int index = -1;
	uint32_t data = 0;

	for (i = 0; i < g_vendor_sensor_data.dump_data.data_num; i++) {
		if (g_vendor_sensor_data.dump_data.dump_info[i].slot_id ==
			s_ctrl->soc_info.index &&
			g_vendor_sensor_data.dump_data.dump_info[i].sensor_id ==
			s_ctrl->sensordata->slave_info.sensor_id) {
			index = i;
			break;
		}
	}

	if (index == -1) {
		CAM_DBG(CAM_SENSOR, "no valid dump info");
		return;
	}
	CAM_INFO(CAM_SENSOR, "index:%d, slot_id:0x%x, sensorId:0x%x",
		index, s_ctrl->soc_info.index, s_ctrl->sensordata->slave_info.sensor_id);
	for (i = 0; i < g_vendor_sensor_data.dump_data.dump_info[index].reg_num; i++) {
		cam_cci_i2c_read(s_ctrl->io_master_info.cci_client,
			g_vendor_sensor_data.dump_data.dump_info[index].reg_addr[i],
			&data,
			g_vendor_sensor_data.dump_data.dump_info[index].addr_type,
			g_vendor_sensor_data.dump_data.dump_info[index].data_type);
		CAM_INFO(CAM_SENSOR, "sensor dump reg, addr: 0x%x, data: 0x%x",
			g_vendor_sensor_data.dump_data.dump_info[index].reg_addr[i], data);
	}
}

/*
 * cam_sensor_i2c_modes_util is a copy from cam_sensor_core.c
 */
static int32_t cam_sensor_i2c_modes_util(
	struct camera_io_master *io_master_info,
	struct i2c_settings_list *i2c_list)
{
	int32_t rc = 0;
	uint32_t i, size;

	if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_RANDOM) {
		rc = camera_io_dev_write(io_master_info,
			&(i2c_list->i2c_settings));
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to random write I2C settings: %d",
				rc);
			return rc;
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_SEQ) {
		rc = camera_io_dev_write_continuous(
			io_master_info,
			&(i2c_list->i2c_settings),
			0);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to seq write I2C settings: %d",
				rc);
			return rc;
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_BURST) {
		rc = camera_io_dev_write_continuous(
			io_master_info,
			&(i2c_list->i2c_settings),
			1);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to burst write I2C settings: %d",
				rc);
			return rc;
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
		size = i2c_list->i2c_settings.size;
		for (i = 0; i < size; i++) {
			rc = camera_io_dev_poll(
			io_master_info,
			i2c_list->i2c_settings.reg_setting[i].reg_addr,
			i2c_list->i2c_settings.reg_setting[i].reg_data,
			i2c_list->i2c_settings.reg_setting[i].data_mask,
			i2c_list->i2c_settings.addr_type,
			i2c_list->i2c_settings.data_type,
			i2c_list->i2c_settings.reg_setting[i].delay);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"i2c poll apply setting Fail: %d", rc);
				return rc;
			}
		}
	}

	return rc;
}

static int32_t vendor_sensor_apply_settings_array(struct cam_sensor_ctrl_t *s_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list = NULL;
	int32_t rc = 0;
	if (!s_ctrl || !i2c_set) {
		CAM_ERR(CAM_SENSOR, "Invalid Args");
		return -EINVAL;
	}

	if (i2c_set->is_settings_valid != 1) {
		CAM_ERR(CAM_SENSOR, " Invalid settings");
		return -EINVAL;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		rc = cam_sensor_i2c_modes_util(
			&(s_ctrl->io_master_info),
			i2c_list);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR, "Failed apply settings: %d", rc);
	}

	return rc;
}

/* Apply probe register settings */
void vendor_sensor_apply_reg_settings(struct cam_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct cam_camera_slave_info *slave_info = NULL;
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "failed: %pK", s_ctrl);
		return;
	}

	if (!(s_ctrl->sensordata)) {
		CAM_ERR(CAM_SENSOR, "failed: %pK", s_ctrl->sensordata);
		delete_request(&s_ctrl->i2c_data.sensor_reg_settings);
		return;
	}

	slave_info = &(s_ctrl->sensordata->slave_info);
	CAM_INFO(CAM_SENSOR, "sensor register settings type is %d",
		slave_info->sensor_probe_info.sensor_reg_settings_type);
	if (slave_info->sensor_probe_info.sensor_reg_settings_type != 0) {
		rc = vendor_sensor_apply_settings_array(s_ctrl,
			&s_ctrl->i2c_data.sensor_reg_settings);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR, "Cannot apply custom register settings");
	}

	delete_request(&s_ctrl->i2c_data.sensor_reg_settings);
	return;
}

static int32_t vendor_sensor_apply_fuseid_reg_settings(
	struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	struct cam_camera_slave_info *slave_info =
		&(s_ctrl->sensordata->slave_info);
	uint8_t fuseid_reg_setting_exists =
		slave_info->sensor_probe_info.sensor_fuseid_info.setting_exists;

	if (fuseid_reg_setting_exists == 0) {
		CAM_INFO(CAM_SENSOR, "fuseid register settings not exists");
		return 0;
	}

	rc = vendor_sensor_apply_settings_array(s_ctrl,
		&s_ctrl->i2c_data.sensor_fuseid_reg_settings);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR, "Cannot apply fuseid register settings");
	delete_request(&s_ctrl->i2c_data.sensor_fuseid_reg_settings);
	return rc;
}

static int32_t vendor_handle_reg_settings_cmd_buffers(
	struct cam_sensor_ctrl_t *s_ctrl,
	struct cam_cmd_buf_desc *cmd_desc,
	struct i2c_settings_array *i2c_reg_settings)
{
	int rc = 0;
	/* i2c_reg_settings must be deleted before command parser */
	delete_request(i2c_reg_settings);
	i2c_reg_settings->request_id = 0;
	i2c_reg_settings->is_settings_valid = 1;
	rc = cam_sensor_i2c_command_parser(
		&s_ctrl->io_master_info,
		i2c_reg_settings,
		cmd_desc, 1, NULL);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR, "pkt parsing failed: %d", rc);
	return rc;
}

int32_t vendor_cam_handle_probe_reg_settings(uint32_t num_cmd_buf,
	struct cam_sensor_ctrl_t *s_ctrl, struct cam_cmd_buf_desc *cmd_desc)
{
	int32_t rc = 0;
	struct cam_camera_slave_info *slave_info = NULL;
	int cmd_buff_index = 2;
	uint8_t probe_reg_setting_type = 0;
	uint8_t fuseid_reg_setting_exists = 0;

	if (num_cmd_buf <= 2) {
		return 0;
	}
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "failed: %pK", s_ctrl);
		return -EINVAL;
	}
	if (!(s_ctrl->sensordata)) {
		CAM_ERR(CAM_SENSOR, "failed: %pK", s_ctrl->sensordata);
		delete_request(&s_ctrl->i2c_data.sensor_fuseid_reg_settings);
		return -EINVAL;
	}
	slave_info = &(s_ctrl->sensordata->slave_info);
	probe_reg_setting_type =
		slave_info->sensor_probe_info.sensor_reg_settings_type;
	fuseid_reg_setting_exists =
		slave_info->sensor_probe_info.sensor_fuseid_info.setting_exists;

	CAM_DBG(CAM_SENSOR,
		"probe_reg_setting_type:%d, fuseid_reg_setting_exists:%d",
		probe_reg_setting_type, fuseid_reg_setting_exists);

	/* Do not change the order of judge probe_reg_setting_type and fuseid_reg_setting_exists */
	if (probe_reg_setting_type != 0) {
		rc = vendor_handle_reg_settings_cmd_buffers(s_ctrl,
			&(cmd_desc[cmd_buff_index]),
			&s_ctrl->i2c_data.sensor_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to parse the probe register settings command Buffer");
			return -EINVAL;
		}
		cmd_buff_index++;
	}
	if (fuseid_reg_setting_exists != 0) {
		rc = vendor_handle_reg_settings_cmd_buffers(s_ctrl,
			&(cmd_desc[cmd_buff_index]),
			&s_ctrl->i2c_data.sensor_fuseid_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Failed to parse the fuseid register settings command Buffer");
			return -EINVAL;
		}
	}
	return rc;
}

int32_t vendor_module_match_id(struct cam_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

	/* Match module ID */
	rc = vendor_module_probe(s_ctrl);
	if (rc < 0) {
		cam_sensor_power_down(s_ctrl);
		msleep(20);
		return rc;
	}

	rc = vendor_sensor_apply_fuseid_reg_settings(s_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Apply probe register fuse id settings failed");
		cam_sensor_power_down(s_ctrl);
		return rc;
	}

	/* Read fuseid */
	rc = vendor_sensor_read_fuseid(s_ctrl);
	if (rc < 0) {
		cam_sensor_power_down(s_ctrl);
		msleep(20);
		return rc;
	}
	return rc;
}

void vendor_sensor_query_cap(struct cam_sensor_ctrl_t *s_ctrl,
	struct  cam_sensor_query_cap *query_cap)
{
	uint8_t i;

	query_cap->va_slot_id =
		s_ctrl->sensordata->subdev_id[SUB_MODULE_VA];

	for (i = 0; i < FUSEID_LEN_MAX; i++)
		query_cap->fuseid_data[i] = s_ctrl->sensordata->fuseid_data[i];
}

void vendor_sensor_update_slave_info(struct cam_cmd_probe *probe_info,
	struct cam_sensor_ctrl_t *s_ctrl)
{
	s_ctrl->sensordata->slave_info.sensor_probe_info =
		probe_info->sensor_probe_info;
	s_ctrl->sensordata->slave_info.module_probe_info =
		probe_info->module_probe_info;
}

void vendor_sensor_start_dev(struct cam_sensor_ctrl_t *s_ctrl)
{
	uint32_t index = s_ctrl->sensordata->subdev_id[SUB_MODULE_CSIPHY];

	/* this is workwround only for bali */
	if (vendor_skip_pre_stream_overflow(index) == true) {
		/* BALI delay 30ms after apply stream on setting */
		msleep(30);
		vendor_set_cam_stream_state(true);
	}
}

void vendor_sensor_stop_dev(struct cam_sensor_ctrl_t *s_ctrl)
{
	/* this is workwround only for bali */
	vendor_set_cam_stream_state(false);
}

void vendor_sensor_get_sub_module_index(struct device_node *of_node,
	struct cam_sensor_board_info *s_info)
{
	int rc = 0;
	uint32_t val = 0;
	struct device_node *src_node = NULL;
	struct cam_sensor_board_info *sensor_info;

	sensor_info = s_info;

	src_node = of_parse_phandle(of_node, "va-src", 0);
	if (!src_node) {
		CAM_DBG(CAM_SENSOR, "src_node NULL");
	} else {
		rc = of_property_read_u32(src_node, "cell-index", &val);
		CAM_DBG(CAM_SENSOR, "va cell index %d, rc %d", val, rc);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "failed %d", rc);
			of_node_put(src_node);
			return;
		}
		sensor_info->subdev_id[SUB_MODULE_VA] = val;
		of_node_put(src_node);
	}
}

int vendor_sensor_match_id_with_retry(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint16_t retry_times = 0;
	struct cam_camera_slave_info *slave_info = NULL;
	uint16_t i = 0;

	slave_info = &(s_ctrl->sensordata->slave_info);
	if (!slave_info) {
		CAM_ERR(CAM_SENSOR, "slave_info is null");
		return -EINVAL;
	}

	retry_times = slave_info->sensor_probe_info.match_id_retry_times_without_powerdown;
	CAM_INFO(CAM_SENSOR, " retry_times: %d", retry_times);
	do {
		rc = cam_sensor_match_id(s_ctrl);
		i++;
	} while (rc < 0 && i < retry_times);

	return rc;
}

MODULE_DESCRIPTION("vendor sensor driver");
MODULE_LICENSE("GPL v2");