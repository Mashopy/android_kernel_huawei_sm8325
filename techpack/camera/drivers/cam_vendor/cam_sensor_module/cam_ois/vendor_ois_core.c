/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor OIS core driver
 */
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/dma-contiguous.h>
#include <cam_sensor_cmn_header.h>
#include <securec.h>
#include "vendor_ois_core.h"
#include "vendor_sensor_util.h"
#include "vendor_ctrl.h"
#include "cam_ois_core.h"

static LIST_HEAD(g_ois);

int32_t ois_r_reg16_val16(struct vendor_ois_ctrl *ois,
		uint16_t reg, uint16_t *val)
{
	int32_t rc = 0;
	struct cam_ois_ctrl_t *o_ctrl = ois->o_ctrl;
	uint32_t val_temp = 0;

	rc = camera_io_dev_read(&(o_ctrl->io_master_info),
		(uint32_t)reg, &val_temp,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0)
		CAM_ERR(CAM_OIS, "ois read failed reg:%x,rc:%d", reg, rc);

	*val = (uint16_t)(val_temp & 0xFFFF); /* mask to uint16_t */
	return rc;
}

int32_t ois_w_reg16_val16(struct vendor_ois_ctrl *ois,
	uint16_t reg, uint16_t val)
{
	struct cam_sensor_i2c_reg_setting i2c_reg_setting;
	struct cam_sensor_i2c_reg_array i2c_reg_array;
	int32_t rc = 0;
	struct cam_ois_ctrl_t *o_ctrl = ois->o_ctrl;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.size = 1;
	i2c_reg_setting.delay = 0;

	i2c_reg_setting.reg_setting = &i2c_reg_array;

	i2c_reg_setting.reg_setting[0].reg_addr = reg;
	i2c_reg_setting.reg_setting[0].reg_data = val;
	i2c_reg_setting.reg_setting[0].delay = 0;
	i2c_reg_setting.reg_setting[0].data_mask = 0;

	rc = camera_io_dev_write(&(o_ctrl->io_master_info),
		&i2c_reg_setting);
	if (rc < 0)
		CAM_ERR(CAM_OIS, "ois write failed reg:%x,val:%x,rc:%d", reg, val, rc);

	return rc;
}

int32_t ois_r_reg16_val32(struct vendor_ois_ctrl *ois,
		uint16_t reg, uint32_t *val)
{
	int32_t rc = 0;
	struct cam_ois_ctrl_t *o_ctrl = ois->o_ctrl;

	rc = camera_io_dev_read(&(o_ctrl->io_master_info),
		(uint32_t)reg, (uint32_t *)val,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_DWORD);
	if (rc < 0)
		CAM_ERR(CAM_OIS, " ois read failed reg:%x,rc:%d", reg, rc);

	return rc;
}

int32_t ois_w_reg16_val32(struct vendor_ois_ctrl *ois,
		uint16_t reg, uint32_t val)
{
	struct cam_sensor_i2c_reg_setting i2c_reg_setting;
	struct cam_sensor_i2c_reg_array i2c_reg_array;
	int32_t rc = 0;
	struct cam_ois_ctrl_t *o_ctrl = ois->o_ctrl;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD;
	i2c_reg_setting.size = 1;
	i2c_reg_setting.delay = 0;

	i2c_reg_setting.reg_setting = &i2c_reg_array;

	i2c_reg_setting.reg_setting[0].reg_addr = reg;
	i2c_reg_setting.reg_setting[0].reg_data = val;
	i2c_reg_setting.reg_setting[0].delay = 0;
	i2c_reg_setting.reg_setting[0].data_mask = 0;

	rc = camera_io_dev_write(&(o_ctrl->io_master_info),
		&i2c_reg_setting);
	if (rc < 0)
		CAM_ERR(CAM_OIS, "ois write failed reg:%x,val:%x,rc:%d", reg, val, rc);

	return rc;
}

/* revert reg read&write */
int32_t ois_r_reg16_val16_rvt(struct vendor_ois_ctrl *ois,
		uint16_t reg, uint16_t *val)
{
	int32_t rc = 0;
	uint8_t data[2] = {0};
	rc = ois_r_reg16_val16(ois, reg, (uint16_t *)data);
	*val = (uint16_t)((data[0] << 8) | data[1]);
	return rc;
}

int32_t ois_w_reg16_val16_rvt(struct vendor_ois_ctrl *ois,
		uint16_t reg, uint16_t val)
{
	uint16_t data = 0;
	data = ((val & 0xFF) << 8) | ((val >> 8) & 0xFF);
	return ois_w_reg16_val16(ois, reg, data);
}

int32_t ois_r_reg16_val32_rvt(struct vendor_ois_ctrl *ois,
		uint16_t reg, uint32_t *val)
{
	int32_t rc = 0;
	uint8_t data[4] = {0};
	rc = ois_r_reg16_val32(ois, reg, (uint32_t *)data);
	*val = (uint32_t)((data[0] << 24) | (data[1] << 16) |
		(data[2] << 8) | data[3]);
	return rc;
}

int32_t ois_w_reg16_val32_rvt(struct vendor_ois_ctrl *ois,
		uint16_t reg, uint32_t val)
{
	uint32_t data = 0;
	data = ((val & 0xFF) << 24) | (((val >> 8) & 0xFF) << 16) |
	       (((val >> 16) & 0xFF) << 8) | ((val >> 24) & 0xFF);
	return ois_w_reg16_val32(ois, reg, data);
}

/* block reg read&write with sequence or burst */
int32_t ois_w_reg16_val16_blk(struct vendor_ois_ctrl *ois,
		uint16_t reg, uint16_t *wr_buf, int32_t length, int32_t flag)
{
	uint32_t pkt_size;
	uint16_t *ptr = NULL;
	int32_t rc = 0;
	int32_t cnt;
	struct cam_sensor_i2c_reg_setting i2c_reg_setting;
	void *vaddr = NULL;
	struct cam_ois_ctrl_t *o_ctrl = ois->o_ctrl;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.size = length;
	i2c_reg_setting.delay = 0;
	pkt_size = (sizeof(struct cam_sensor_i2c_reg_array) * length);
	vaddr = vmalloc(pkt_size);
	if (!vaddr) {
		CAM_ERR(CAM_OIS,
			"Failed in allocating i2c_array: pkt_size: %u", pkt_size);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		vaddr);

	for (cnt = 0, ptr = wr_buf; cnt < length; cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr = reg;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	/* we use busrt write MSM_CCI_I2C_WRITE_BURST */
	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, flag);
	if (rc < 0)
		CAM_ERR(CAM_OIS, "ois block write failed %d", rc);

	vfree(vaddr);
	vaddr = NULL;

	return rc;
}

int32_t ois_w_reg8_val8_blk(struct vendor_ois_ctrl *ois, uint8_t reg,
		uint8_t *wr_buf, int32_t length, int32_t flag)
{
	struct cam_sensor_i2c_reg_setting i2c_reg_setting;
	int32_t ret = -1;
	uint32_t cnt;
	struct cam_ois_ctrl_t *o_ctrl = ois->o_ctrl;

	if (o_ctrl == NULL || wr_buf == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args o_ctrl: %pK, wr_buf: %pK",
			o_ctrl, wr_buf);
		return -EINVAL;
	}

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = length;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting =
	    (struct cam_sensor_i2c_reg_array *)
	    kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * length,
		    GFP_KERNEL);
	if (i2c_reg_setting.reg_setting == NULL) {
		CAM_ERR(CAM_OIS, "kzalloc failed");
		return -EINVAL;
	}
	i2c_reg_setting.reg_setting[0].reg_addr = reg;
	i2c_reg_setting.reg_setting[0].reg_data = wr_buf[0];
	i2c_reg_setting.reg_setting[0].delay = 0;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	for (cnt = 1; cnt < length; cnt++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr = 0;
		i2c_reg_setting.reg_setting[cnt].reg_data = wr_buf[cnt];
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}
	ret = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
					   &i2c_reg_setting, flag);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "cci write_continuous failed %d", ret);
		kfree(i2c_reg_setting.reg_setting);
		return ret;
	}
	kfree(i2c_reg_setting.reg_setting);
	return ret;
}

int32_t ois_w_reg16_val8_blk(struct vendor_ois_ctrl *ois,
		uint16_t reg, uint8_t *wr_buf, int32_t length, int32_t flag)
{
	uint32_t pkt_size;
	uint8_t *ptr = NULL;
	int32_t rc = 0;
	int32_t cnt;
	struct cam_sensor_i2c_reg_setting i2c_reg_setting;
	void *vaddr = NULL;
	struct cam_ois_ctrl_t *o_ctrl = ois->o_ctrl;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = length;
	i2c_reg_setting.delay = 0;
	pkt_size = (sizeof(struct cam_sensor_i2c_reg_array) * length);
	vaddr = vmalloc(pkt_size);
	if (!vaddr) {
		CAM_ERR(CAM_OIS,
			"Failed in allocating i2c_array: pkt_size: %u", pkt_size);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		vaddr);

	for (cnt = 0, ptr = wr_buf; cnt < length; cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr = reg;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	/* we use busrt write MSM_CCI_I2C_WRITE_BURST */
	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, flag);
	if (rc < 0)
		CAM_ERR(CAM_OIS, " ois block write reg:%x failed, rc:%d", reg, rc);

	vfree(vaddr);
	vaddr = NULL;

	return rc;
}

int32_t ois_r_reg_val_blk(struct vendor_ois_ctrl *ois,
		uint16_t reg, uint32_t *wr_buf, int32_t length,
		enum camera_sensor_i2c_type addr_type,
		enum camera_sensor_i2c_type data_type)
{
	int32_t rc = 0;
	int32_t i;
	uint8_t *data = (uint8_t *)wr_buf;
	uint16_t *data_short = NULL;
	struct cam_ois_ctrl_t *o_ctrl = ois->o_ctrl;

	rc = camera_io_dev_read_seq(&(o_ctrl->io_master_info), (uint32_t)reg, data,
		addr_type, data_type, (int32_t)length);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, " ois read failed reg:%x,rc:%d", reg, rc);
	} else {
		if (data_type == CAMERA_SENSOR_I2C_TYPE_BYTE) {
			return rc;
		} else if (data_type == CAMERA_SENSOR_I2C_TYPE_WORD) {
			data_short = (uint16_t *)data;
			for (i = 0; i < length; i = i + 2)
				data_short[i / 2] = data[i] << 8 | data[i + 1];
		} else if (data_type == CAMERA_SENSOR_I2C_TYPE_DWORD) {
			for (i = 0; i < length; i = i + 4)
				wr_buf[i / 4] = data[i] << 24 | data[i + 1] << 16 |
					data[i + 2] << 8 | data[i + 3];
		} else {
			CAM_ERR(CAM_OIS, "data type invalid");
		}
	}

	return rc;
}

/**
 * vendor_ois_find - match ois with ois_name
 * @name:     ois name
 *
 * Returns ois handle or NULL if fail
 */
static struct vendor_ois_ctrl *vendor_ois_find(const char *name)
{
	struct vendor_ois_ctrl *ois = NULL;

	list_for_each_entry(ois, &g_ois, list)
		if (!strcmp(ois->name, name))
			return ois;

	return NULL;
}

/**
 * vendor_request_firmware - request a named ois firmware
 * @ois:     ctrl structure
 *
 * Returns private data handle
 */
const struct firmware *vendor_request_firmware(struct vendor_ois_ctrl *ois)
{
	int32_t rc = 0;
	const char *fw_name = NULL;
	char name_buf[FW_NAME_SIZE] = {0};
	const struct firmware *fw = NULL;
	struct cam_ois_ctrl_t *o_ctrl = ois->o_ctrl;
	struct device *dev = &(o_ctrl->pdev->dev);

	if (strcmp(o_ctrl->opcode.fw_ver_name, "") == 0 ||
		strcmp(o_ctrl->opcode.fw_ver_name, "-") == 0)
			rc = snprintf_s(name_buf, FW_NAME_SIZE, FW_NAME_SIZE - 1,
				"%s.prog", o_ctrl->ois_name);
	else
		rc = snprintf_s(name_buf, FW_NAME_SIZE, FW_NAME_SIZE - 1,
			"%s_%s.prog", o_ctrl->ois_name, o_ctrl->opcode.fw_ver_name);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "snprintf_s failed");
		return NULL;
	}
	fw_name = name_buf;
	CAM_INFO(CAM_OIS, "ois_name:%s, fw_ver_name:%s, fw:%s",
		o_ctrl->ois_name, o_ctrl->opcode.fw_ver_name, fw_name);
	/* Load FW */
	rc = request_firmware(&fw, fw_name, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name);
		return NULL;
	}

	return fw;
}

/**
 * vendor_ois_get_drvdata - get private data handle for ois driver
 * @ois:     ctrl structure
 *
 * Returns private data handle
 */
void *vendor_ois_get_drvdata(struct vendor_ois_ctrl *ois)
{
	return ois->driver_data;
}

/**
 * vendor_ois_set_drvdata - store private data for ois driver
 * @ois:     ctrl structure
 * @data:    ois private data to be stored
 *
 * Returns success or failure
 */
void vendor_ois_set_drvdata(struct vendor_ois_ctrl *ois, void *data)
{
	ois->driver_data = data;
}

/**
 * vendor_ois_download - download firmware to ois IC
 * @o_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
int32_t vendor_ois_download(struct cam_ois_ctrl_t *o_ctrl)
{
	struct vendor_ois_ctrl *ois = NULL;

	ois = vendor_ois_find(o_ctrl->ois_name);
	if (ois && ois->ops && ois->ops->fw_download) {
		ois->o_ctrl = o_ctrl;
		ois->ops->fw_download(ois);
		return 0;
	}

	return -1;
}

/**
 * vendor_ois_register - register a vendor ois driver
 * @ois:     ctrl structure
 *
 * Returns success or failure
 */
int32_t vendor_ois_register(struct vendor_ois_ctrl *ois)
{
	if (!ois) {
		CAM_ERR(CAM_OIS, "ois is NULL");
		return -EINVAL;
	}
	if (vendor_ois_find(ois->name)) {
		CAM_ERR(CAM_OIS, "ois %s has been registered", ois->name);
		return -EINVAL;
	}
	list_add_tail(&ois->list, &g_ois);

	return 0;
}

/**
 * vendor_ois_unregister - unregister a vendor ois driver
 * @ois:     ctrl structure
 *
 * Returns success or failure
 */
int32_t vendor_ois_unregister(struct vendor_ois_ctrl *ois)
{
	if (vendor_ois_find(ois->name) == NULL) {
		CAM_ERR(CAM_OIS, "ois %s not found", ois->name);
		return -EINVAL;
	}
	list_del(&ois->list);

	return 0;
}

int32_t vendor_ois_driver_cmd(struct cam_ois_ctrl_t *o_ctrl,
	void *arg)
{
	int rc = 0;
	struct cam_control *cmd = (struct cam_control *)arg;

	if (!o_ctrl || !cmd) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_OIS, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}
	CAM_DBG(CAM_OIS, "Opcode to OIS: %d", cmd->op_code);

	mutex_lock(&(o_ctrl->ois_mutex));
	switch (cmd->op_code) {
	case CAM_REG_CONFIG: {
		struct cam_sensor_config_reg ois_reg_data;
		rc = copy_from_user(&ois_reg_data,
			u64_to_user_ptr(cmd->handle),
			sizeof(ois_reg_data));
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Failed copy from user");
			goto release_mutex;
		}

		if (ois_reg_data.data_direct == CAM_WRITE_REG) {
			rc = vendor_cam_write_reg(&(o_ctrl->io_master_info), &ois_reg_data);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "ois %s set register failed", o_ctrl->device_name);
				goto release_mutex;
			}
		} else if (ois_reg_data.data_direct == CAM_READ_REG) {
			rc = vendor_cam_read_reg(&(o_ctrl->io_master_info), &ois_reg_data);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "ois %s get register failed", o_ctrl->device_name);
				goto release_mutex;
			} else {
				if (copy_to_user(u64_to_user_ptr(cmd->handle),
					&ois_reg_data,
					sizeof(ois_reg_data))) {
					CAM_ERR(CAM_OIS, "Failed copy to user");
					rc = -EFAULT;
					goto release_mutex;
				}
			}
		} else {
			CAM_ERR(CAM_OIS, "Invalid data direct %d", ois_reg_data.data_direct);
		}
	}
		break;
	default:
		CAM_ERR(CAM_OIS, "Invalid Opcode %d", cmd->op_code);
	}

release_mutex:
	mutex_unlock(&(o_ctrl->ois_mutex));

	return rc;
}

void vendor_ois_update_time(struct i2c_settings_list *i2c_list, uint64_t qtime_ns)
{
	uint32_t size = 0;
	uint32_t i = 0;
	uint16_t qtime_ic = 0;

	if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_RANDOM) {
		size = i2c_list->i2c_settings.size;
		qtime_ic = (qtime_ns / 100000) % 65536;
		/* qtimer is 2 bytes so validate here */
		for (i = 0; i < size; i++) {
			i2c_list->i2c_settings.reg_setting[i].reg_data = qtime_ic;
			CAM_DBG(CAM_OIS, "reg_addr:0x%x, reg_data[%d]: 0x%x, delay:%d",
				i2c_list->i2c_settings.reg_setting[i].reg_addr,
				i, i2c_list->i2c_settings.reg_setting[i].reg_data,
				i2c_list->i2c_settings.reg_setting[i].delay);
			i2c_list->i2c_settings.reg_setting[i].delay = 1000;
		}
	}
}

/*
 * this is copy from cam_ois_core.c to split from it
 */
 static int cam_ois_apply_settings(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t i, size;

	if (o_ctrl == NULL || i2c_set == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (i2c_set->is_settings_valid != 1) {
		CAM_ERR(CAM_OIS, " Invalid settings");
		return -EINVAL;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_RANDOM) {
			rc = camera_io_dev_write(&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings));
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed in Applying i2c wrt settings");
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_SEQ) {
			rc = camera_io_dev_write_continuous(
				&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings),
				0);
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed to seq write I2C settings: %d",
					rc);
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
			size = i2c_list->i2c_settings.size;
			for (i = 0; i < size; i++) {
				rc = camera_io_dev_poll(
				&(o_ctrl->io_master_info),
				i2c_list->i2c_settings.reg_setting[i].reg_addr,
				i2c_list->i2c_settings.reg_setting[i].reg_data,
				i2c_list->i2c_settings.reg_setting[i].data_mask,
				i2c_list->i2c_settings.addr_type,
				i2c_list->i2c_settings.data_type,
				i2c_list->i2c_settings.reg_setting[i].delay);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"i2c poll apply setting Fail");
					return rc;
				}
			}
		}
	}

	return rc;
}

/*
 * this is copy from cam_ois_core.c to split from it
 */
static int cam_ois_slaveInfo_pkt_parser(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t rc = 0;
	struct cam_cmd_ois_info *ois_info;

	if (!o_ctrl || !cmd_buf || len < sizeof(struct cam_cmd_ois_info)) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	ois_info = (struct cam_cmd_ois_info *)cmd_buf;
	if (o_ctrl->io_master_info.master_type == CCI_MASTER) {
		o_ctrl->io_master_info.cci_client->i2c_freq_mode =
			ois_info->i2c_freq_mode;
		o_ctrl->io_master_info.cci_client->sid =
			ois_info->slave_addr >> 1;
		o_ctrl->ois_fw_flag = ois_info->ois_fw_flag;
		o_ctrl->is_ois_calib = ois_info->is_ois_calib;
		memcpy(o_ctrl->ois_name, ois_info->ois_name, OIS_NAME_LEN);
		o_ctrl->ois_name[OIS_NAME_LEN - 1] = '\0';
		o_ctrl->io_master_info.cci_client->retries = 3;
		o_ctrl->io_master_info.cci_client->id_map = 0;
		memcpy(&(o_ctrl->opcode), &(ois_info->opcode),
			sizeof(struct cam_ois_opcode));
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x Freq Mode: %d",
			ois_info->slave_addr, ois_info->i2c_freq_mode);
	} else if (o_ctrl->io_master_info.master_type == I2C_MASTER) {
		o_ctrl->io_master_info.client->addr = ois_info->slave_addr;
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x", ois_info->slave_addr);
	} else {
		CAM_ERR(CAM_OIS, "Invalid Master type : %d",
			o_ctrl->io_master_info.master_type);
		rc = -EINVAL;
	}

	return rc;
}

int vendor_ois_custom_config_pkt_parse(struct cam_packet *csl_packet)
{
	int32_t                       rc = -EINVAL;
	struct i2c_settings_array     *i2c_reg_settings = NULL;
	struct cam_cmd_buf_desc       *cmd_desc = NULL;
	uint32_t                      *offset = NULL;
	struct                        custom_config_info *config_info;
	struct                        cam_ois_ctrl_t *ctrl = NULL;
	uintptr_t                     cmd_buf_ptr;
	size_t                        len_of_buffer;

	if (csl_packet == NULL) {
		CAM_ERR(CAM_OIS, "invalid ptr");
		return rc;
	}

	/* here is for ois extra config */
	offset = (uint32_t *)&csl_packet->payload;
	offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
	cmd_desc = (struct cam_cmd_buf_desc *)(offset);
	rc = cam_mem_get_cpu_buf(cmd_desc->mem_handle, &cmd_buf_ptr, &len_of_buffer);
	if (rc) {
		CAM_ERR(CAM_OIS, "Fail in get buffer: %d", rc);
		return rc;
	}
	if ((len_of_buffer < sizeof(struct custom_config_info)) ||
		(cmd_desc->offset > (len_of_buffer - sizeof(struct custom_config_info)))) {
		CAM_ERR(CAM_OIS, "Not enough buffer");
		rc = -EINVAL;
		return rc;
	}
	/* get hal config info */
	config_info = (struct custom_config_info *)((uint8_t *)cmd_buf_ptr +
		cmd_desc->offset);
	ctrl = vendor_get_ois_ctrl(config_info->sensor_slotID);
	if (!ctrl) {
		CAM_ERR(CAM_SENSOR, "The conflict dev ctrl is null, do nothing");
		return rc;
	}
	rc = camera_io_init(&ctrl->io_master_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "shutdown config cci_init failed: rc: %d", rc);
		return rc;
	}
	CAM_INFO(CAM_OIS, "get ois custom config mode %d", config_info->config_mode);
	switch (config_info->config_mode) {
	case SHUTDOWN_MODE: {
		i2c_reg_settings = &(ctrl->i2c_shutdown_data);
		if(i2c_reg_settings->is_settings_valid == 1) {
			rc = cam_ois_apply_settings(ctrl, i2c_reg_settings);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "Cannot apply mode settings");
				camera_io_release(&ctrl->io_master_info);
				return rc;
			}
			CAM_INFO(CAM_OIS, "apply shutdown mode settings suc");
		} else {
			CAM_INFO(CAM_OIS, "no shutdown mode settings");
		}
		break;
	}
	default:
		CAM_ERR(CAM_OIS, "no such mode %d", config_info->config_mode);
	}
	camera_io_release(&ctrl->io_master_info);
	return rc;
}

int vendor_ois_custom_init_pkt_parse(struct cam_ois_ctrl_t *o_ctrl,
	struct cam_packet *csl_packet)
{
	int32_t                        rc = -EINVAL;
	int32_t                        i = 0;
	uint32_t                       total_cmd_buf_in_bytes = 0;
	struct common_header           *cmm_hdr = NULL;
	uintptr_t                      generic_ptr;
	struct cam_cmd_buf_desc        *cmd_desc = NULL;
	struct i2c_settings_array      *i2c_reg_settings = NULL;
	size_t                         remain_len = 0;
	size_t                         len_of_buff = 0;
	uint32_t                       *offset = NULL;
	uint32_t                       *cmd_buf = NULL;

	if (o_ctrl == NULL || csl_packet == NULL) {
		CAM_ERR(CAM_OIS, "invalid ptr");
		return rc;
	}
	offset = (uint32_t *)&csl_packet->payload;
	offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
	cmd_desc = (struct cam_cmd_buf_desc *)(offset);
	for (i = 0; i < csl_packet->num_cmd_buf; i++) {
		total_cmd_buf_in_bytes = cmd_desc[i].length;
		if (!total_cmd_buf_in_bytes) continue;

		rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
			&generic_ptr, &len_of_buff);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Failed to get cpu buf : 0x%x",
				cmd_desc[i].mem_handle);
			return rc;
		}
		cmd_buf = (uint32_t *)generic_ptr;
		if (!cmd_buf) {
			CAM_ERR(CAM_OIS, "invalid cmd buf");
			return -EINVAL;
		}

		if ((len_of_buff < sizeof(struct common_header)) ||
			(cmd_desc[i].offset > (len_of_buff - sizeof(struct common_header)))) {
			CAM_ERR(CAM_OIS, "Invalid length for sensor cmd");
			return -EINVAL;
		}
		remain_len = len_of_buff - cmd_desc[i].offset;
		cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
		cmm_hdr = (struct common_header *)cmd_buf;
		CAM_INFO(CAM_OIS, "Get cmdbuf type %d", cmm_hdr->cmd_type);
		switch (cmm_hdr->cmd_type) {
		case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
			rc = cam_ois_slaveInfo_pkt_parser(o_ctrl, cmd_buf, remain_len);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "Failed in parsing slave info");
				return rc;
			}
			break;
		case CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR:
			i2c_reg_settings = &(o_ctrl->i2c_shutdown_data);
			i2c_reg_settings->request_id = 0;
			rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
				i2c_reg_settings, cmd_desc, 1, NULL);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "OIS pkt parsing failed: %d", rc);
				return rc;
			}
			CAM_INFO(CAM_OIS, "Get shutdown mode settings suc %d, rc %d",
				csl_packet->num_cmd_buf, rc);
		}
	}
	return rc;
}

MODULE_DESCRIPTION("vendor ois driver");
MODULE_LICENSE("GPL v2");
