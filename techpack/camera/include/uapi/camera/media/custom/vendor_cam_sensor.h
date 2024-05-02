/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 *
 * Description: The Linux Foundation
 *
 * Create: 2021-12-09
 */

#ifndef __VENDOR_CAM_SENSOR_H__
#define __VENDOR_CAM_SENSOR_H__

#include <media/custom/vendor_cam_defs.h>

#define REG_CONFIG_MAX_SIZE       64
#define DUMP_REG_MAX_SIZE         50
#define FUSEID_LEN_MAX            30

/**
 * struct cam_sensor_reg_payload - Structure carry register payload
 * @addr_type  :     The register address
 * @data_type  :     The register data
 * @delay      :     The delay between two I2C options
 */
struct cam_sensor_reg_payload {
	__u32 reg_addr;
	__u32 reg_data;
	__u32 delay;
} __attribute__((packed));

/**
 * struct cam_sensor_config_reg - Structure used to read and write register
 * @data_direct :     Write or read register
 * @addr_type   :     The type of address
 * @data_type   :     The type of data
 * @size        :     The numbers of data
 * @data        :     The data
 */
struct cam_sensor_config_reg {
	__u32  data_direct;
	__u32  addr_type;
	__u32  data_type;
	__u32  size;
	struct cam_sensor_reg_payload data[REG_CONFIG_MAX_SIZE];
} __attribute__((packed));

struct cam_vaf_pwctrl_data {
	__u8 pwctrl_support;
	__u8 low_power_flag;
} __attribute__((packed));

struct cam_sensor_config_data {
	__u64 request_id;
	struct cam_vaf_pwctrl_data vaf_pwctrl_data;
} __attribute__((packed));

struct vendor_sensor_i2c_dump_info {
	__u32 slot_id;
	__u16 sensor_id;
	__u16 reg_num;
	__u16 addr_type;
	__u16 data_type;
	__u32 reg_addr[DUMP_REG_MAX_SIZE];
} __attribute__((packed));

enum cam_sensor_reg_config_direct {
	CAM_WRITE_REG,
	CAM_READ_REG,
};

struct custom_module_version_info {
	__u8 module_version_flag;
	__u16 module_version_addr;
	__u16 module_version_data;
} __attribute__((packed));

struct custom_module_qcom_info {
	__u8 module_qcom_flag;
	__u16 module_qcom_addr;
	__u16 module_qcom_data;
} __attribute__((packed));

struct custom_sensor_fuseid_info {
	__u8 fuseid_flag;
	__u8 fuseid_data_type;
	__u8 fuseid_addr_type;
	__u16 status_addr;
	__u8 status_val;
	__u8 status_mask;
	__u16 fuseid_addr;
	__u8 fuseid_len;
	__u8 read_pattern;
	__u16 target_addr_h;
	__u16 target_addr_l;
	__u16 read_addr;
	__u16 read_enable_addr;
	__u8 read_enable_val;
	__u8 step;
	bool is_retry;
	__u8 setting_exists;
} __attribute__((packed));

struct custom_sensor_probe_info {
	/* probe sensor info */
	__u8 sensor_reg_settings_type;
	struct custom_sensor_fuseid_info sensor_fuseid_info;
	__s32 position_id;
	__u16 match_id_retry_times_without_powerdown;
} __attribute__((packed));

struct custom_module_probe_info {
	/* probe module info */
	__u8 module_code_support;
	__u8 cloud_module_code_support;
	__u8 module_code_data_type;
	__u8 module_code_addr_type;
	__u16 eeprom_slave_addr;
	__u16 module_code_data;
	__u16 module_code_addr;
	__u16 cloud_module_code_data;
	__u16 cloud_module_qcom_data;
	__u16 sensor_module_num; /* the same sensor total module number */
	struct custom_module_version_info module_version;
	struct custom_module_qcom_info module_qcom;
} __attribute__((packed));

#endif
/* __VENDOR_CAM_SENSOR_H__ */
