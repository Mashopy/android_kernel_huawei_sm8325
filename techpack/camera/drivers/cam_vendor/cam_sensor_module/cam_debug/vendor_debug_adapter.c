/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: debug adapter driver
 */
#include "vendor_debug_adapter.h"
#include "vendor_ctrl.h"
#include <securec.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include "cam_actuator_dev.h"
#include "cam_ois_dev.h"
#include "cam_sensor_dev.h"

/* Configure the functions to be supported. */
static struct function_config g_function_policy[] = {
	{SUPPORT_FUNCTION,    GET_DEV_DATA,         NON_DEV},
	{CAMERA_EXIST,        GET_DEV_DATA,         NON_DEV},
	{SENSOR_I2C_READ,     GET_DEBUG_DATA,       SENSOR},
	{SENSOR_I2C_WRITE,    GET_DEBUG_DATA,       SENSOR},
	{VCM_I2C_READ,        GET_DEBUG_DATA,       VCM},
	{VCM_I2C_WRITE,       GET_DEBUG_DATA,       VCM},
	{OIS_I2C_READ,        GET_DEBUG_DATA,       OIS},
	{OIS_I2C_WRITE,       GET_DEBUG_DATA,       OIS},
};

static enum camera_sensor_i2c_type get_bit(unsigned int bit)
{
	switch (bit) {
	case 8:   // 8 bit
		return CAMERA_SENSOR_I2C_TYPE_BYTE;
	case 16:  // 16 bit
		return CAMERA_SENSOR_I2C_TYPE_WORD;
	case 24:  // 24 bit
		return CAMERA_SENSOR_I2C_TYPE_3B;
	case 32:  // 32 bit
		return CAMERA_SENSOR_I2C_TYPE_DWORD;
	default:
		return CAMERA_SENSOR_I2C_TYPE_INVALID;
	}
}

int i2c_read(struct debug_msg *recv_data, struct debug_msg *send_data,
			 struct camera_io_master *io_master_info)
{
	struct i2c_data *debug_data = NULL;
	struct i2c_data *read_data = NULL;
	int i;
	enum camera_sensor_i2c_type reg_bit;
	enum camera_sensor_i2c_type val_bit;
	int rc = 0;
	if (!recv_data || !send_data || !io_master_info) {
		debug_err("data or io_master_info is null.");
		return -1;
	}
	debug_data = &recv_data->u.i2c_debug_data;
	if (!debug_data) {
		debug_err("i2c_data is null.");
		return -1;
	}
	read_data = &send_data->u.i2c_debug_data;

	reg_bit = get_bit(debug_data->reg_bit);
	if (reg_bit == CAMERA_SENSOR_I2C_TYPE_INVALID)
		reg_bit = CAMERA_SENSOR_I2C_TYPE_WORD;
	val_bit = get_bit(debug_data->val_bit);
	if (val_bit == CAMERA_SENSOR_I2C_TYPE_INVALID)
		val_bit = CAMERA_SENSOR_I2C_TYPE_BYTE;
	debug_dbg("reg_bit %d, val_bit %d", reg_bit, val_bit);

	for (i = 0; i < debug_data->num; i++) {
		rc = camera_io_dev_read(
			io_master_info,
			debug_data->data[i].reg, &read_data->data[i].val, reg_bit, val_bit);
		if (rc < 0) {
			return -1;
		}
		debug_dbg("read iic reg 0x%x, val 0x%x",
			debug_data->data[i].reg, read_data->data[i].val);
		read_data->data[i].reg = debug_data->data[i].reg;
		mdelay(debug_data->data[i].delay_ms);
	}
	read_data->num = debug_data->num;
	return 0;
}

int i2c_write(struct debug_msg *recv_data,
			  struct camera_io_master *io_master_info)
{
	struct i2c_data *debug_data = NULL;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array i2c_array[MAX_I2C_SIZE] = {{0}};
	int i;
	enum camera_sensor_i2c_type reg_bit;
	enum camera_sensor_i2c_type val_bit;
	int rc = 0;

	if (!recv_data || !io_master_info) {
		debug_err("data or io_master_info is null.");
		return -1;
	}
	debug_data = &recv_data->u.i2c_debug_data;
	if (!debug_data) {
		debug_err("i2c_data is null.");
		return -1;
	}

	reg_bit = get_bit(debug_data->reg_bit);
	if (reg_bit == CAMERA_SENSOR_I2C_TYPE_INVALID)
		reg_bit = CAMERA_SENSOR_I2C_TYPE_WORD;
	val_bit = get_bit(debug_data->val_bit);
	if (val_bit == CAMERA_SENSOR_I2C_TYPE_INVALID)
		val_bit = CAMERA_SENSOR_I2C_TYPE_BYTE;
	debug_dbg("reg_bit %d, val_bit %d", reg_bit, val_bit);
	write_setting.size = debug_data->num;
	write_setting.addr_type = reg_bit;
	write_setting.data_type = val_bit;
	write_setting.delay = 0;
	write_setting.read_buff = NULL;
	write_setting.read_buff_len = 0;

	for (i = 0; i < debug_data->num; i++) {
		i2c_array[i].reg_addr = debug_data->data[i].reg;
		i2c_array[i].reg_data = debug_data->data[i].val;
		i2c_array[i].delay = debug_data->data[i].delay_ms;
		i2c_array[i].data_mask = 0;
		debug_dbg("write iic reg 0x%x, val 0x%x", debug_data->data[i].reg,
			debug_data->data[i].val);
	}
	write_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)&i2c_array[0];
	rc = camera_io_dev_write(io_master_info, &write_setting);
	if (rc < 0) {
		debug_err("failed to read i2c");
		return -1;
	}
	return 0;
}

static void vendor_actuator_debug_adapter(struct debug_msg *recv_data,
	struct debug_msg *send_data)
{
	struct cam_actuator_ctrl_t *ctrl = NULL;
	ctrl = vendor_get_vcm_ctrl(recv_data->dev_id);
	debug_dbg("enter");
	if (!ctrl) {
		debug_err("vcm ctrl is null");
		return;
	}
	switch (recv_data->command) {
	case VCM_I2C_READ:
		debug_dbg("VCM_I2C_READ");
		send_data->state = i2c_read(recv_data, send_data, &(ctrl->io_master_info));
		break;
	case VCM_I2C_WRITE:
		debug_dbg("VCM_I2C_WRITE");
		send_data->state = i2c_write(recv_data, &(ctrl->io_master_info));
		break;
	default:
		debug_err("no function");
	}
}

static void vendor_ois_debug_adapter(struct debug_msg *recv_data,
	struct debug_msg *send_data)
{
	struct cam_ois_ctrl_t *ctrl = NULL;
	ctrl = vendor_get_ois_ctrl(recv_data->dev_id);
	debug_dbg("enter");
	if (!ctrl) {
		CAM_ERR(CAM_SENSOR, "ctrl is null");
		return;
	}
	switch (recv_data->command) {
	case OIS_I2C_READ:
		debug_dbg("OIS_I2C_READ");
		send_data->state = i2c_read(recv_data, send_data,
			&(ctrl->io_master_info));
		break;
	case OIS_I2C_WRITE:
		debug_dbg("OIS_I2C_WRITE");
		send_data->state = i2c_write(recv_data, &(ctrl->io_master_info));
		break;
	default:
		debug_err("no function");
	}
}

static void vendor_sensor_debug_adapter(struct debug_msg *recv_data,
	struct debug_msg *send_data)
{
	struct cam_sensor_ctrl_t *ctrl = NULL;
	debug_dbg("enter, camid %d, cmd %d", recv_data->dev_id,
		recv_data->command);
	ctrl = vendor_get_sensor_ctrl(recv_data->dev_id, CAM_SENSOR);
	if (!ctrl) {
		debug_err("ctrl is null");
		return;
	}
	switch (recv_data->command) {
		case SENSOR_I2C_READ: {
			debug_dbg("SENSOR_I2C_READ");
			send_data->state = i2c_read(recv_data, send_data,
				&(ctrl->io_master_info));
			break;
		}
		case SENSOR_I2C_WRITE: {
			debug_dbg("SENSOR_I2C_WRITE");
			send_data->state = i2c_write(recv_data, &(ctrl->io_master_info));
			break;
		}
		default:
			debug_err("no_func");
	}
}

void vendor_actuator_debug_update(struct dev_msg_t* debug_dev)
{
	if (debug_dev->dev_id != -1) {
		debug_dev->handle = vendor_actuator_debug_adapter;
		strcpy(debug_dev->dev_name, "VCM");
		updata_dev_msg(debug_dev);
		updata_state_by_position(debug_dev->dev_id, VCM, SENSOR_IS_EXIST);
	}
}

void vendor_ois_debug_update(struct dev_msg_t* debug_dev)
{
	if (debug_dev->dev_id != -1) {
		debug_dev->handle = vendor_ois_debug_adapter;
		strcpy(debug_dev->dev_name, "OIS");
		updata_dev_msg(debug_dev);
		updata_state_by_position(debug_dev->dev_id, OIS, SENSOR_IS_EXIST);
	}
}

void vendor_sensor_debug_update(struct cam_sensor_ctrl_t *s_ctrl, struct dev_msg_t* debug_dev)
{
	debug_dev->dev_id = s_ctrl->soc_info.index;
	debug_dev->handle = vendor_sensor_debug_adapter;
	strcpy(debug_dev->dev_name, "SENSOR");
	updata_dev_msg(debug_dev);
	updata_state_by_position(s_ctrl->soc_info.index, SENSOR, SENSOR_IS_EXIST);
}

int debug_adapter_init(void)
{
	updata_func_config(g_function_policy, ARRAY_SIZE(g_function_policy));
	return 0;
}

void debug_adapter_exit(void)
{
}
