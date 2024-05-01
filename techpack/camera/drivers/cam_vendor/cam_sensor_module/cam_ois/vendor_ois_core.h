/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor OIS core driver
 */
#ifndef _VENDOR_OIS_CORE_H_
#define _VENDOR_OIS_CORE_H_

#include <linux/firmware.h>
#include <linux/cma.h>
#include "cam_ois_dev.h"
#include "media/custom/vendor_cam_defs.h"
#include "media/custom/vendor_cam_sensor.h"

#include "ois_aw86006.h"
#include "ois_dw9781.h"
#include "ois_dw9787.h"
#include "ois_lc898129.h"
#include "ois_rumbas10.h"

#define OIS_NAME_SIZE 64
#define FW_NAME_SIZE 128

struct vendor_ois_ctrl;

struct vendor_ois_ops {
	/* download firmware for ois IC */
	int32_t (*fw_download)(struct vendor_ois_ctrl *ctrl);
};

struct vendor_ois_ctrl {
	/* ois name which should be unique */
	char name[OIS_NAME_SIZE];
	const struct vendor_ois_ops *ops;

	/* store driver private define */
	void *driver_data;

	/* platform define */
	struct cam_ois_ctrl_t *o_ctrl;
	struct list_head list;
};

int32_t vendor_ois_download(struct cam_ois_ctrl_t *o_ctrl);
int32_t vendor_ois_register(struct vendor_ois_ctrl *ois);
int32_t vendor_ois_unregister(struct vendor_ois_ctrl *ois);
void *vendor_ois_get_drvdata(struct vendor_ois_ctrl *ois);
void vendor_ois_set_drvdata(struct vendor_ois_ctrl *ois, void *data);
const struct firmware *vendor_request_firmware(struct vendor_ois_ctrl *ois);
/* standard reg read&write */
int32_t ois_r_reg16_val16(struct vendor_ois_ctrl *ois, uint16_t reg, uint16_t *val);
int32_t ois_w_reg16_val16(struct vendor_ois_ctrl *ois, uint16_t reg, uint16_t val);
int32_t ois_r_reg16_val32(struct vendor_ois_ctrl *ois, uint16_t reg, uint32_t *val);
int32_t ois_w_reg16_val32(struct vendor_ois_ctrl *ois, uint16_t reg, uint32_t val);
/* revert reg read&write */
int32_t ois_r_reg16_val16_rvt(struct vendor_ois_ctrl *ois, uint16_t reg, uint16_t *val);
int32_t ois_w_reg16_val16_rvt(struct vendor_ois_ctrl *ois, uint16_t reg, uint16_t val);
int32_t ois_r_reg16_val32_rvt(struct vendor_ois_ctrl *ois, uint16_t reg, uint32_t *val);
int32_t ois_w_reg16_val32_rvt(struct vendor_ois_ctrl *ois, uint16_t reg, uint32_t val);
/* block reg read&write with sequence or burst */
int32_t ois_w_reg16_val16_blk(struct vendor_ois_ctrl *ois, uint16_t reg,
	uint16_t *wr_buf, int32_t length, int32_t flag);
int32_t ois_w_reg8_val8_blk(struct vendor_ois_ctrl *ois, uint8_t reg,
	uint8_t *wr_buf, int32_t length, int32_t flag);
int32_t ois_w_reg16_val8_blk(struct vendor_ois_ctrl *ois, uint16_t reg,
	uint8_t *wr_buf, int32_t length, int32_t flag);
int32_t ois_r_reg_val_blk(struct vendor_ois_ctrl *ois, uint16_t reg,
	uint32_t *wr_buf, int32_t length,
	enum camera_sensor_i2c_type addr_type, enum camera_sensor_i2c_type data_type);

int32_t vendor_ois_driver_cmd(struct cam_ois_ctrl_t *o_ctrl, void *arg);
void vendor_ois_update_time(struct i2c_settings_list *i2c_list, uint64_t qtime_ns);
int vendor_ois_custom_config_pkt_parse(struct cam_packet *csl_packet);
int vendor_ois_custom_init_pkt_parse(struct cam_ois_ctrl_t *o_ctrl,
	struct cam_packet *csl_packet);

#endif
/* _VENDOR_OIS_CORE_H_ */
