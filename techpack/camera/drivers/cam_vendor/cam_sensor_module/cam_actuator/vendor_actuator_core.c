/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor actuator core driver
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include "vendor_actuator_core.h"
#include "vendor_sensor_util.h"
#include "vendor_ctrl.h"
#include "media/custom/vendor_cam_defs.h"
#include "media/custom/vendor_cam_sensor.h"
#include <securec.h>

static void vendor_vaf_pwctrl_data_proc(struct cam_actuator_ctrl_t *a_ctrl,
	struct cam_sensor_config_data cfg_data);

int32_t vendor_actuator_driver_cmd(struct cam_actuator_ctrl_t *a_ctrl, void *arg)
{
	int rc = 0;
	struct cam_control *cmd = (struct cam_control *)arg;

	if (!a_ctrl || !cmd) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_ACTUATOR, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	CAM_DBG(CAM_ACTUATOR, "Opcode to Actuator: %d, dev_type: %d",
		cmd->op_code, a_ctrl->v_ctrl.dev_type);

	mutex_lock(&(a_ctrl->actuator_mutex));
	switch (cmd->op_code) {
	case CAM_REG_CONFIG: {
		struct cam_sensor_config_reg actuator_reg_data;
		rc = copy_from_user(&actuator_reg_data,
			u64_to_user_ptr(cmd->handle),
			sizeof(actuator_reg_data));
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "Failed copy from user");
			goto release_mutex;
		}

		if (actuator_reg_data.data_direct == CAM_WRITE_REG) {
			rc = vendor_cam_write_reg(&(a_ctrl->io_master_info), &actuator_reg_data);
			if (rc < 0) {
				CAM_ERR(CAM_ACTUATOR, "Actuator %s set register failed", a_ctrl->device_name);
				goto release_mutex;
			}
		} else if (actuator_reg_data.data_direct == CAM_READ_REG) {
			rc = vendor_cam_read_reg(&(a_ctrl->io_master_info), &actuator_reg_data);
			if (rc < 0) {
				CAM_ERR(CAM_ACTUATOR, "Actuator %s get register failed", a_ctrl->device_name);
				goto release_mutex;
			} else {
				if (copy_to_user(u64_to_user_ptr(cmd->handle),
					&actuator_reg_data,
					sizeof(actuator_reg_data))) {
					CAM_ERR(CAM_ACTUATOR, "Failed copy to user");
					rc = -EFAULT;
					goto release_mutex;
				}
			}
		} else {
			CAM_ERR(CAM_ACTUATOR, "Invalid data direct %d", actuator_reg_data.data_direct);
		}
	}
		break;
	case CAM_DATA_CONFIG: {
		struct cam_sensor_config_data actuator_config_data;
		rc = copy_from_user(&actuator_config_data,
			u64_to_user_ptr(cmd->handle),
			sizeof(actuator_config_data));
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "Failed copy from user");
			goto release_mutex;
		}
		vendor_vaf_pwctrl_data_proc(a_ctrl, actuator_config_data);
	}
		break;
	default:
		CAM_ERR(CAM_ACTUATOR, "Invalid Opcode %d", cmd->op_code);
	}

release_mutex:
	mutex_unlock(&(a_ctrl->actuator_mutex));

	return rc;
}

static void vendor_va_config_power_for_req(struct cam_actuator_ctrl_t *a_ctrl,
	uint64_t req_id)
{
	int32_t request_id;
	struct cam_actuator_soc_private  *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;
	unsigned int gpio;
	int value;

	request_id = req_id % MAX_PER_FRAME_ARRAY;
	if (a_ctrl->v_ctrl.low_power_flag[request_id] !=
		a_ctrl->v_ctrl.pre_low_power_flag) {
		soc_private =
			(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;
		power_info = &soc_private->power_info;

		CAM_DBG(CAM_ACTUATOR, "dev_type:%d, gpio num: %d, valid:%d, low_power_flag:%d",
			a_ctrl->v_ctrl.dev_type,
			power_info->gpio_num_info->gpio_num[SENSOR_VAF_PWCTRL],
			power_info->gpio_num_info->valid[SENSOR_VAF_PWCTRL],
			a_ctrl->v_ctrl.low_power_flag[request_id]);

		if (power_info->gpio_num_info->valid[SENSOR_VAF_PWCTRL]) {
			gpio = power_info->gpio_num_info->gpio_num[SENSOR_VAF_PWCTRL];
			value = a_ctrl->v_ctrl.low_power_flag[request_id];
			/* low power: gpio output high; High power: gpio output low */
			gpio_set_value_cansleep(gpio, value);
		}
		a_ctrl->v_ctrl.pre_low_power_flag = a_ctrl->v_ctrl.low_power_flag[request_id];
	}
}

static void vendor_vaf_pwctrl_data_proc(struct cam_actuator_ctrl_t *a_ctrl,
	struct cam_sensor_config_data cfg_data)
{
	if (cfg_data.vaf_pwctrl_data.pwctrl_support == false)
		return;

	a_ctrl->v_ctrl.low_power_flag[cfg_data.request_id % MAX_PER_FRAME_ARRAY] =
		cfg_data.vaf_pwctrl_data.low_power_flag;
	/* immediately config power when requestId is 0 for VA */
	if (cfg_data.request_id == 0)
		vendor_va_config_power_for_req(a_ctrl, 0);
}

static int32_t vendor_actuator_update_slave_address(struct cam_actuator_ctrl_t *a_ctrl,
	uint32_t slave_addr)
{
	int32_t rc = 0;

	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	if (a_ctrl->io_master_info.master_type == CCI_MASTER) {
		a_ctrl->io_master_info.cci_client->sid =
			slave_addr >> 1;
		CAM_DBG(CAM_ACTUATOR, "update Slave addr: 0x%x",
			slave_addr);
	} else if (a_ctrl->io_master_info.master_type == I2C_MASTER) {
		a_ctrl->io_master_info.client->addr = slave_addr;
		CAM_DBG(CAM_ACTUATOR, "update Slave addr: 0x%x", slave_addr);
	} else {
		CAM_ERR(CAM_ACTUATOR, "Invalid Master type: %d",
			a_ctrl->io_master_info.master_type);
		rc = -EINVAL;
	}

	return rc;
}

static int32_t vendor_actuator_get_slave_address(struct cam_actuator_ctrl_t *a_ctrl,
	uint32_t *slave_addr)
{
	int32_t rc = 0;

	if (!a_ctrl || !slave_addr) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	if (a_ctrl->io_master_info.master_type == CCI_MASTER) {
		*slave_addr = a_ctrl->io_master_info.cci_client->sid << 1;
		CAM_DBG(CAM_ACTUATOR, "get Slave addr: 0x%x",
			*slave_addr);
	} else if (a_ctrl->io_master_info.master_type == I2C_MASTER) {
		*slave_addr = a_ctrl->io_master_info.client->addr;
		CAM_DBG(CAM_ACTUATOR, "get Slave addr: 0x%x", *slave_addr);
	} else {
		CAM_ERR(CAM_ACTUATOR, "Invalid Master type: %d",
			a_ctrl->io_master_info.master_type);
		rc = -EINVAL;
	}

	return rc;
}

/*
 * cam_actuator_i2c_modes_util is a copy from cam_actuator_core.c
 */
static int32_t cam_actuator_i2c_modes_util(
	struct camera_io_master *io_master_info,
	struct i2c_settings_list *i2c_list)
{
	int32_t rc = 0;
	uint32_t i, size;

	if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_RANDOM) {
		rc = camera_io_dev_write(io_master_info,
			&(i2c_list->i2c_settings));
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
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
			CAM_ERR(CAM_ACTUATOR,
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
			CAM_ERR(CAM_ACTUATOR,
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
				CAM_ERR(CAM_ACTUATOR,
					"i2c poll apply setting Fail: %d", rc);
				return rc;
			}
		}
	}

	return rc;
}

int32_t vendor_actuator_apply_settings(struct cam_actuator_ctrl_t *a_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t temp_addr = 0;
	vendor_actuator_get_slave_address(a_ctrl, &temp_addr);

	if (temp_addr == 0 || a_ctrl->v_ctrl.dual_slave_addr == 0)
		return rc;

	vendor_actuator_update_slave_address(a_ctrl, a_ctrl->v_ctrl.dual_slave_addr);
	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		rc = cam_actuator_i2c_modes_util(
			&(a_ctrl->io_master_info),
			i2c_list);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed to apply settings: %d",
				rc);
		} else {
			CAM_DBG(CAM_ACTUATOR,
				"Success:request ID: %d",
				i2c_set->request_id);
		}
	}

	vendor_actuator_update_slave_address(a_ctrl, temp_addr);
	return rc;
}


void vendor_actuator_revert_byte(struct cam_actuator_ctrl_t *a_ctrl,
	struct i2c_settings_list *i2c_list)
{
	int32_t rc = 0;

	if (a_ctrl->v_ctrl.is_big_endian == 1) {
		rc = vendor_sensor_util_i2c_revert_byte(i2c_list);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"i2c revert failed: %d", rc);
		}
	}
}

void vendor_actuator_pkt_parser(struct cam_actuator_ctrl_t *a_ctrl,
	struct cam_cmd_i2c_info *i2c_info)
{
	a_ctrl->v_ctrl.dual_slave_addr = i2c_info->dual_slave_addr;
	a_ctrl->v_ctrl.is_big_endian = i2c_info->is_big_endian;
	a_ctrl->v_ctrl.dev_type = i2c_info->dev_type;
	CAM_DBG(CAM_ACTUATOR, "slave dev_type: %d", a_ctrl->v_ctrl.dev_type);
}

int32_t vendor_actuator_apply_request(struct cam_actuator_ctrl_t *a_ctrl,
	struct cam_req_mgr_apply_request *apply)
{
	int32_t rc = 0, request_id;

	if (a_ctrl->v_ctrl.dev_type != CAM_ACTUATOR_VA)
		return rc;

	request_id = apply->request_id % MAX_PER_FRAME_ARRAY;

	vendor_va_config_power_for_req(a_ctrl, apply->request_id);

	CAM_DBG(CAM_ACTUATOR, "trigger_point:%d, is_sof_applied:%d, frame_id:%d, frame_id_applied:%d",
		apply->trigger_point, a_ctrl->v_ctrl.is_sof_applied,
		apply->frame_id, a_ctrl->v_ctrl.frame_id_applied);
	if ((apply->trigger_point == CAM_TRIGGER_POINT_EOF) &&
		(a_ctrl->v_ctrl.is_sof_applied == TRUE) &&
		(apply->frame_id == a_ctrl->v_ctrl.frame_id_applied)) {
		a_ctrl->v_ctrl.is_sof_applied = false;
		CAM_WARN(CAM_ACTUATOR, "Request Id: %lld need apply at next frame eof",
			apply->request_id);
		return -EINVAL;
	}

	CAM_DBG(CAM_ACTUATOR, "Request Id: %lld, a_ctrl reqID:%lld, is_settings_valid:%d",
		apply->request_id,
		a_ctrl->i2c_data.per_frame[request_id].request_id,
		a_ctrl->i2c_data.per_frame[request_id].is_settings_valid);

	if (apply->trigger_point == CAM_TRIGGER_POINT_SOF) {
		a_ctrl->v_ctrl.is_sof_applied = true;
		a_ctrl->v_ctrl.frame_id_applied = apply->frame_id;
	}
	return rc;
}

void vendor_actuator_update_req_mgr(struct cam_actuator_ctrl_t *a_ctrl,
	struct cam_packet *csl_packet, struct cam_req_mgr_add_request *add_req)
{
	int32_t request_id;
	if (a_ctrl->v_ctrl.dev_type != CAM_ACTUATOR_VA)
		return;

	request_id = add_req->req_id % MAX_PER_FRAME_ARRAY;
	if ((csl_packet->header.op_code & 0xFFFFFF) == CAM_ACTUATOR_PACKET_MANUAL_MOVE_LENS &&
		a_ctrl->v_ctrl.low_power_flag[request_id] == false) {
		add_req->trigger_eof = true;
		add_req->skip_at_sof = 2;
	}
}

void vendor_actuator_init_pwctrl_state(struct cam_actuator_ctrl_t *a_ctrl)
{
	int index;
	errno_t ret;
	struct cam_actuator_soc_private *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;
	struct cam_sensor_power_setting *power_setting;

	soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if (!power_info || !power_info->gpio_num_info || !power_info->power_setting) {
		CAM_WARN(CAM_ACTUATOR, "param is null");
		return;
	}

	if (!power_info->gpio_num_info->valid[SENSOR_VAF_PWCTRL])
		return;

	for (index = 0; index < power_info->power_setting_size; index++) {
		power_setting = &power_info->power_setting[index];
		if (power_setting->seq_type == SENSOR_VAF_PWCTRL) {
			a_ctrl->v_ctrl.pre_low_power_flag =
				(int)power_setting->config_val;
			CAM_INFO(CAM_ACTUATOR, "pre_low_power_flag:%d",
				(int)power_setting->config_val);
		}
	}

	ret = memset_s(a_ctrl->v_ctrl.low_power_flag, sizeof(uint8_t) * MAX_PER_FRAME_ARRAY,
		0, sizeof(uint8_t) * MAX_PER_FRAME_ARRAY);
	if (ret != EOK)
		CAM_ERR(CAM_ACTUATOR, "%s memset_s fail, ret = %d", __func__, ret);
}

#define DW9781_ACTUATOR_LOGIC_SLAVE_ID   (0xE4 >> 1)
#define DW9781_ACTUATOR_SLAVE_ID         (0x54 >> 1)
#define DW9781_LOGIC_RESET_ADDRESS       0xD002
#define DW9781_LOGIC_RESET_VAL           0x0001

int32_t vendor_actuator_retry_dw9781(struct camera_io_master *master_info,
		struct i2c_settings_list *i2c_list, int32_t ret_val)
{
	int32_t rc = 0;
	struct cam_sensor_i2c_reg_setting i2c_reg_setting;
	struct cam_sensor_i2c_reg_array i2c_reg_array;
	if (master_info->cci_client->sid != DW9781_ACTUATOR_SLAVE_ID)
		return ret_val;
	master_info->cci_client->sid = DW9781_ACTUATOR_LOGIC_SLAVE_ID;
	CAM_INFO(CAM_ACTUATOR, "update dw9781 slave id = 0x%x", DW9781_ACTUATOR_LOGIC_SLAVE_ID);
	i2c_reg_array.reg_addr = DW9781_LOGIC_RESET_ADDRESS;
	i2c_reg_array.reg_data = DW9781_LOGIC_RESET_VAL;
	i2c_reg_array.delay = 0;
	i2c_reg_array.data_mask = 0;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.size = 1;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = &i2c_reg_array;
	CAM_INFO(CAM_ACTUATOR, "actuator write dw9781 reset reg:%x, value:%x",
		DW9781_LOGIC_RESET_ADDRESS, DW9781_LOGIC_RESET_VAL);
	rc = camera_io_dev_write(master_info, &i2c_reg_setting);
	if (rc < 0)
		CAM_ERR(CAM_ACTUATOR, "actuator dw9781 write failed reg:%x,value:%x,rc:%d",
			DW9781_LOGIC_RESET_ADDRESS, DW9781_LOGIC_RESET_VAL, rc);
	usleep_range(10 * 1000, 11 * 1000); /* delay 10ms after dw9781 ois reset */
	master_info->cci_client->sid = DW9781_ACTUATOR_SLAVE_ID;
	rc = cam_actuator_i2c_modes_util(master_info, i2c_list);
	if (rc)
		CAM_ERR(CAM_ACTUATOR, "retry dw9781 apply settings failed: %d", rc);
	return rc;
}

MODULE_DESCRIPTION("vendor actuator driver");
MODULE_LICENSE("GPL v2");
