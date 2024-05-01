/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor AW86006 OIS driver
 */
#include <linux/module.h>
#include <linux/string.h>
#include <linux/dma-contiguous.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>
#include <cam_sensor_cmn_header.h>
#include <cam_ois_core.h>
#include <cam_ois_soc.h>
#include <cam_sensor_util.h>
#include <cam_debug_util.h>
#include <cam_res_mgr_api.h>
#include <cam_common_util.h>
#include <cam_packet_util.h>

#include "vendor_ois_core.h"
#include "ois_aw86006.h"
#include "securec.h"

struct aw_fw_info {
	uint32_t checksum;
	uint32_t app_checksum;
	uint32_t app_length;
	uint32_t app_version;
	uint32_t app_id;
	uint32_t move_checksum;
	uint32_t move_version;
	uint32_t move_length;
	uint32_t update_flag;
	uint32_t size;
	uint8_t *data_p;
};

struct soc_protocol {
	uint8_t checksum;
	uint8_t protocol_ver;
	uint8_t addr;
	uint8_t module;
	uint8_t event;
	uint8_t len[2];
	uint8_t ack;
	uint8_t sum;
	uint8_t reserved[3];
	uint8_t ack_juge;
	uint8_t *p_data;
};

struct ois_ctrl_priv {
	bool already_download;
	struct aw_fw_info fw_info;
	uint8_t checkinfo_fw[AW_FW_INFO_LENGTH];
	uint8_t checkinfo_rd[AW_FW_INFO_LENGTH];
};

static int32_t aw86006_cci_stand_read(struct vendor_ois_ctrl *ois,
	uint32_t addr, uint32_t addr_type, uint8_t *data, uint32_t num_byte)
{
	enum i2c_freq_mode temp_freq;
	int32_t ret;
	struct cam_ois_ctrl_t *o_ctrl = ois->o_ctrl;

	if (o_ctrl == NULL || data == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args o_ctrl: %pK, data: %pK",
			o_ctrl, data);
		return -EINVAL;
	}
	if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
		CAM_WARN(CAM_OIS, "Not in right state to start soc reads: %d",
			 o_ctrl->cam_ois_state);
		return -EINVAL;
	}
	temp_freq = o_ctrl->io_master_info.cci_client->i2c_freq_mode;

	/* Modify i2c freq to 100K */
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_STANDARD_MODE;
	ret = camera_io_dev_read_seq(&(o_ctrl->io_master_info), addr, data,
		addr_type, CAMERA_SENSOR_I2C_TYPE_BYTE, num_byte);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "cci read failed!");
		ret = OIS_ERROR;
	}
	o_ctrl->io_master_info.cci_client->i2c_freq_mode = temp_freq;

	return ret;
}

static int32_t aw86006_soc_buf_build(struct vendor_ois_ctrl *ois,
	uint8_t *buf, struct soc_protocol *soc_struct)
{
	uint8_t *p_head = (uint8_t *)soc_struct;
	uint8_t i = 0;
	uint8_t checksum = 0;
	uint8_t data_sum = 0;
	int32_t ret = OIS_ERROR;

	if ((buf == NULL) || ((soc_struct == NULL)))
		return ret;
	if (soc_struct->p_data == NULL)
		soc_struct->len[0] = 0;
	soc_struct->protocol_ver = SOC_VERSION;
	soc_struct->ack = SOC_ACK;
	soc_struct->addr = ((soc_struct->ack_juge == SOC_CTL) ?
		SOC_ADDR : SOC_READ_ADDR);
	for (i = 0; i < soc_struct->len[0]; i++) {
		data_sum += soc_struct->p_data[i];
		buf[i + SOC_PROTOCAL_HEAD] = soc_struct->p_data[i];
	}
	soc_struct->sum = data_sum;
	for (i = 1; i < SOC_PROTOCAL_HEAD; i++) {
		checksum += p_head[i];
		buf[i] = p_head[i];
	}
	soc_struct->checksum = checksum;
	buf[0] = p_head[0];
	ret = OIS_SUCCESS;

	return ret;
}

static int32_t aw86006_soc_connect_check(struct vendor_ois_ctrl *ois)
{
	struct soc_protocol soc_struct = {0};
	uint8_t w_buf[14] = {0};
	uint8_t r_buf[14] = {0};
	uint8_t cmp_buf[5] = {0x00, 0x01, 0x00, 0x00, 0x00};
	int32_t ret = OIS_ERROR;

	soc_struct.module = SOC_HANK;
	soc_struct.event = SOC_HANK_CONNECT;
	soc_struct.len[0] = 0;
	soc_struct.p_data = NULL;
	soc_struct.ack_juge = SOC_CTL;
	aw86006_soc_buf_build(ois, w_buf, &soc_struct);

	ret = ois_w_reg8_val8_blk(ois, w_buf[0],
		&w_buf[1], SOC_CONNECT_WRITE_LEN - 1, 0);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "cci write error:%d", ret);
		return ret;
	}
	usleep_range(SOC_CONNECT_DELAY, SOC_CONNECT_DELAY + TIME_50US);
	ret = aw86006_cci_stand_read(ois, AW_SOC_ADDRESS_NONE,
		CAMERA_SENSOR_I2C_TYPE_BYTE, &r_buf[0], 14);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "cci read error:%d", ret);
		return ret;
	}

	soc_struct.event = SOC_HANK_CONNECT_ACK;
	soc_struct.ack_juge = SOC_ACK;
	soc_struct.len[0] = 5;
	soc_struct.p_data = cmp_buf;
	aw86006_soc_buf_build(ois, w_buf, &soc_struct);
	ret = memcmp(w_buf, r_buf, 14);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "soc read check error");
		return OIS_ERROR;
	}

	return ret;
}

static int32_t aw86006_flash_erase_check(struct vendor_ois_ctrl *ois,
	uint32_t addr, uint32_t len)
{
	struct soc_protocol soc_struct = {0};
	uint32_t erase_block = len / AW_FLASH_ERASE_LEN +
		((len % AW_FLASH_ERASE_LEN) ? 1 : 0);
	uint8_t i = 0;
	uint8_t temp_buf[6] = {0};
	uint8_t cmp_buf[1] = {0};
	uint8_t w_buf[15] = {0};
	uint8_t r_buf[10] = {0};
	int32_t ret = OIS_ERROR;
	int32_t loop = AW_SOC_ACK_ERROR_LOOP;

	temp_buf[0] = (uint8_t)erase_block;
	temp_buf[1] = 0x00;
	for (i = 0; i < 4; i++)
		temp_buf[i + 2] = (uint8_t)(addr >> (i * 8));
	soc_struct.module = SOC_FLASH;
	do {
		soc_struct.event = SOC_FLASH_ERASE_BLOCK;
		soc_struct.len[0] = SOC_ERASE_STRUCT_LEN;
		soc_struct.p_data = temp_buf;
		soc_struct.ack_juge = SOC_CTL;
		aw86006_soc_buf_build(ois, w_buf, &soc_struct);
		ret = ois_w_reg8_val8_blk(ois, w_buf[0],
			&w_buf[1], SOC_ERASE_WRITE_LEN - 1, 0);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "cci write error:%d", ret);
			continue;
		}
		msleep(erase_block * SOC_ERASE_BLOCK_DELAY);
		ret = aw86006_cci_stand_read(ois, AW_SOC_ADDRESS_NONE,
			CAMERA_SENSOR_I2C_TYPE_BYTE, &r_buf[0], 10);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "cci read error:%d", ret);
			continue;
		}
		soc_struct.event = SOC_FLASH_ERASE_BLOCK_ACK;
		soc_struct.len[0] = 1;
		soc_struct.ack_juge = SOC_ACK;
		soc_struct.p_data = cmp_buf;
		aw86006_soc_buf_build(ois, w_buf, &soc_struct);
		ret = memcmp(w_buf, r_buf, 10);
		if (ret != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "soc read check error");
			continue;
		} else {
			break;
		}
	} while (--loop);
	if (loop == 0) {
		CAM_ERR(CAM_OIS, "flash erase error!");
		return OIS_ERROR;
	}

	return ret;
}

static int32_t aw86006_flash_read_check(struct vendor_ois_ctrl *ois,
		uint32_t addr, uint8_t *bin_buf, uint32_t len)
{
	struct soc_protocol soc_struct = {0};
	uint8_t temp_buf[SOC_READ_STRUCT_LEN] = {0};
	uint8_t w_buf[SOC_READ_WRITE_LEN] = {0};
	uint8_t r_buf[100] = {0};
	uint8_t checksum = 0;
	uint8_t i = 0;
	int32_t ret = OIS_ERROR;
	int32_t loop = AW_SOC_ACK_ERROR_LOOP;

	temp_buf[0] = len;
	temp_buf[1] = 0;
	for (i = 0; i < 4; i++)
		temp_buf[i + 2] = (uint8_t) (addr >> (i * 8));
	soc_struct.module = SOC_FLASH;
	do {
		soc_struct.event = SOC_FLASH_READ;
		soc_struct.len[0] = SOC_READ_STRUCT_LEN;
		soc_struct.p_data = temp_buf;
		soc_struct.ack_juge = SOC_CTL;
		aw86006_soc_buf_build(ois, w_buf, &soc_struct);

		ret = ois_w_reg8_val8_blk(ois, w_buf[0],
			&w_buf[1], SOC_READ_WRITE_LEN - 1, 0);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "cci write error:%d", ret);
			continue;
		}
		usleep_range(SOC_READ_BLOCK_DELAY, SOC_READ_BLOCK_DELAY + TIME_50US);

		ret = aw86006_cci_stand_read(ois, AW_SOC_ADDRESS_NONE,
			CAMERA_SENSOR_I2C_TYPE_BYTE, &r_buf[0], 14 + len);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "cci read error:%d", ret);
			continue;
		}
		/* check error flag */
		if (r_buf[9] != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "error flag wrong:%d", r_buf[10]);
			continue;
		}
		checksum = 0;
		/* compute data checksum */
		for (i = 0; i < len + 5; i++)
			checksum += r_buf[9 + i];
		if (checksum != r_buf[8]) {
			CAM_ERR(CAM_OIS, "data checksum error:0x%02x != 0x%02x",
				checksum, r_buf[8]);
			continue;
		}
		checksum = 0;
		/* compute head checksum */
		for (i = 1; i < 9; i++)
			checksum += r_buf[i];
		if (checksum != r_buf[0]) {
			CAM_ERR(CAM_OIS, "head checksum error:0x%02x != 0x%02x",
				checksum, r_buf[0]);
			continue;
		} else {
			ret = memcpy_s(bin_buf, len, &r_buf[14], len);
			break;
		}
	} while (--loop);
	if (loop == 0) {
		CAM_ERR(CAM_OIS, "flash read error!");
		return OIS_ERROR;
	}

	return ret;
}

static int32_t aw86006_flash_write_check(struct vendor_ois_ctrl *ois,
	uint32_t addr, uint32_t block_num, uint8_t *bin_buf, uint32_t len)
{
	struct soc_protocol soc_struct = {0};
	uint8_t temp_buf[68] = {0};
	uint8_t w_buf[77] = {0};
	uint8_t r_buf[10] = {0};
	uint8_t cmp_buf[1] = {0};
	uint8_t i;
	int32_t ret = OIS_ERROR;
	int32_t loop = AW_SOC_ACK_ERROR_LOOP;

	for (i = 0; i < 4; i++)
		temp_buf[i] = (uint8_t)((addr +
		block_num * AW_FLASH_WRITE_LEN) >> (i * 8));
	for (i = 0; i < len; i++)
		temp_buf[i + 4] = (bin_buf + block_num * AW_FLASH_WRITE_LEN)[i];
	soc_struct.module = SOC_FLASH;
	do {
		soc_struct.event = SOC_FLASH_WRITE;
		soc_struct.len[0] = (uint8_t)(4 + len);
		soc_struct.p_data = temp_buf;
		soc_struct.ack_juge = SOC_CTL;
		aw86006_soc_buf_build(ois, w_buf, &soc_struct);

		ret = ois_w_reg8_val8_blk(ois, w_buf[0],
			&w_buf[1], SOC_WRITE_BLOCK_HEAD - 1 + len, 0);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "cci write error:%d", ret);
			continue;
		}
		usleep_range(SOC_WRITE_BLOCK_DELAY, SOC_WRITE_BLOCK_DELAY + TIME_50US);
		ret = aw86006_cci_stand_read(ois, AW_SOC_ADDRESS_NONE,
			CAMERA_SENSOR_I2C_TYPE_BYTE, &r_buf[0], 10);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "cci read error:%d", ret);
			continue;
		}
		soc_struct.event = SOC_FLASH_WRITE_ACK;
		soc_struct.len[0] = 1;
		soc_struct.p_data = cmp_buf;
		soc_struct.ack_juge = SOC_ACK;
		aw86006_soc_buf_build(ois, w_buf, &soc_struct);
		ret = memcmp(w_buf, r_buf, 10);
		if (ret != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "soc read check error");
			continue;
		} else {
			break;
		}
	} while (--loop);
	if (loop == 0) {
		CAM_ERR(CAM_OIS, "flash write error!");
		return OIS_ERROR;
	}

	return ret;
}

static int32_t aw86006_flash_download_check(struct vendor_ois_ctrl *ois,
	uint32_t addr, uint8_t *bin_buf, size_t len)
{
	uint32_t flash_block = len / AW_FLASH_WRITE_LEN;
	uint32_t flash_tail = len % AW_FLASH_WRITE_LEN;
	uint32_t flash_checkinfo = 0;
	uint32_t i;
	int32_t ret = OIS_ERROR;
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	if (addr == AW_FLASH_BASE_ADDR) {
		flash_checkinfo = AW_FLASH_MOVE_LENGTH / AW_FLASH_WRITE_LEN;
		/* first erase app data */
		ret = aw86006_flash_erase_check(ois, AW_FLASH_APP_ADDR,
			len - AW_FLASH_MOVE_LENGTH);
		if (ret != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "erase checkinfo error!");
			return OIS_ERROR;
		}
		/* then erase move data */
		ret = aw86006_flash_erase_check(ois, AW_FLASH_BASE_ADDR,
			AW_FLASH_MOVE_LENGTH);
		if (ret != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "flash erase error!");
			return OIS_ERROR;
		}
		for (i = 0; i < flash_checkinfo; i++) {
			ret = aw86006_flash_write_check(ois, addr, i,
				bin_buf, AW_FLASH_WRITE_LEN);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "flash write block:%d error!", i);
				return OIS_ERROR;
			}
		}
		for (i = flash_checkinfo + 1; i < flash_block; i++) {
			ret = aw86006_flash_write_check(ois, addr,
				i, bin_buf, AW_FLASH_WRITE_LEN);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "flash write block:%d error!", i);
				return OIS_ERROR;
			}
		}
		if (flash_tail != 0) {
			ret = aw86006_flash_write_check(ois, addr,
				i, bin_buf, flash_tail);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "flash write tail error!");
				return OIS_ERROR;
			}
		}
	} else if (addr == AW_FLASH_APP_ADDR) {
		flash_checkinfo = 0;
		ret = aw86006_flash_erase_check(ois, AW_FLASH_APP_ADDR, len);
		if (ret != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "erase app error!");
			return OIS_ERROR;
		}
		for (i = 1; i < flash_block; i++) {
			ret = aw86006_flash_write_check(ois, addr,
				i, bin_buf, AW_FLASH_WRITE_LEN);
				if (ret != OIS_SUCCESS) {
					CAM_ERR(CAM_OIS, "flash write block:%d error!", i);
					return OIS_ERROR;
			}
		}
		if (flash_tail != 0) {
			ret = aw86006_flash_write_check(ois, addr,
				i, bin_buf, flash_tail);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "flash write tail error!");
				return OIS_ERROR;
			}
		}
	} else {
		CAM_ERR(CAM_OIS, "wrong addr!");
		return OIS_ERROR;
	}

	/* Final flash write checkinfo data */
	ret = aw86006_flash_write_check(ois, AW_FLASH_APP_ADDR,
		0, priv->checkinfo_fw, AW_FLASH_WRITE_LEN);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "flash write checkinfo error!");
		ret = OIS_ERROR;
	}

	return ret;
}

static int32_t aw86006_ois_reset(struct vendor_ois_ctrl *ois)
{
	uint8_t boot_cmd_1[10] = {
		0xFF, 0xF0, 0x20, 0x20, 0x02,
		0x02, 0x19, 0x29, 0x19, 0x29
	};
	uint8_t boot_cmd_2[3] = {0xFF, 0xC4, 0xC4};
	uint8_t boot_cmd_3[3] = {0xFF, 0xC4, 0x00};
	uint8_t boot_cmd_4[3] = {0xFF, 0x10, 0xC3};
	uint16_t temp_addr;
	int32_t ret;
	struct cam_ois_ctrl_t *o_ctrl = ois->o_ctrl;

	/* first: shutdown */
	temp_addr = o_ctrl->io_master_info.cci_client->sid;
	o_ctrl->io_master_info.cci_client->sid = AW_SHUTDOWN_I2C_ADDR;
	ret = ois_w_reg8_val8_blk(ois, boot_cmd_1[0],
		&boot_cmd_1[1], 9, 0); /* 9, data length */
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "write boot_cmd_1 error:%d", ret);
		goto err_exit;
	}
	ret = ois_w_reg8_val8_blk(ois, boot_cmd_2[0],
		&boot_cmd_2[1], sizeof(boot_cmd_2) - 1, 0);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "write boot_cmd_2 error:%d", ret);
		goto err_exit;
	}
	usleep_range(AW_SHUTDOWN_DELAY, AW_SHUTDOWN_DELAY + TIME_50US);
	/* second: wake up */
	o_ctrl->io_master_info.cci_client->sid = AW_WAKEUP_I2C_ADDR;
	ret = ois_w_reg8_val8_blk(ois, boot_cmd_3[0],
		&boot_cmd_3[1], sizeof(boot_cmd_3) - 1, 0);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "write boot_cmd_3 error:%d", ret);
		goto err_exit;
	}
	ret = ois_w_reg8_val8_blk(ois, boot_cmd_4[0],
		&boot_cmd_4[1], sizeof(boot_cmd_4) - 1, 0);
	if (ret < 0) {
		CAM_ERR(CAM_OIS, "write boot_cmd_4 error:%d", ret);
		goto err_exit;
	}

	ret = OIS_SUCCESS;
err_exit:
	o_ctrl->io_master_info.cci_client->sid = temp_addr;

	return ret;
}

static int32_t aw86006_jump_boot(struct vendor_ois_ctrl *ois)
{
	uint8_t boot_cmd[2] = {0xAC, 0xAC};
	int32_t ret = OIS_SUCCESS;
	int32_t i;
	int32_t loop = AW_SOC_JUMP_BOOT_LOOP;
	uint32_t ms_count_reset, ms_count_stop;
	ktime_t kstart, kend;

	CAM_DBG(CAM_OIS, "enter");
	ois->o_ctrl->io_master_info.not_need_dmd_report = true;
	do {
		ret = aw86006_ois_reset(ois);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "AW86006 reset error:%d", ret);
			continue;
		}
		kstart = ktime_get();
		for (i = AW_SOC_RESET_DELAY_MAX; i > 0; i--) {
			mdelay(1);
			kend = ktime_get();
			ms_count_reset = ktime_to_ms(ktime_sub(kend, kstart));
			/* aw suggest mdelay 2ms, less than 6ms */
			if ((ms_count_reset >= AW_SOC_RESET_DELAY) && (ms_count_reset < AW_SOC_RESET_DELAY_MAX))
				break;
		}
		if (ms_count_reset >= AW_SOC_RESET_DELAY_MAX) {
			CAM_ERR(CAM_OIS, "ms_count_reset timeout: %u, loop: %d", ms_count_reset, loop);
			continue;
		}

		for (i = 0; i < AW_SOC_WRITE_ACK_LOOP; i++) {
			ret = ois_w_reg8_val8_blk(ois, boot_cmd[0],
				&boot_cmd[1], sizeof(boot_cmd) - 1, 0);
			if ((ret < 0) && (i >= AW_STOP_CMD_LOOP))
				break;
		}
		kend = ktime_get();
		ms_count_stop = ktime_to_ms(ktime_sub(kend, kstart));
		CAM_INFO(CAM_OIS, "ms_count_reset: %u, ms_count_stop: %u", ms_count_reset, ms_count_stop);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "write 0xAC error:%d, i: %d", ret, i);
			continue;
		}

		ret = aw86006_soc_connect_check(ois);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "connect failed: %d, loop:%d", ret, loop);
			ret = OIS_ERROR;
		} else {
			CAM_INFO(CAM_OIS, "connect success!");
			ret = OIS_SUCCESS;
			break;
		}
	} while (--loop);

	ois->o_ctrl->io_master_info.not_need_dmd_report = false;
	if (loop == 0) {
		CAM_ERR(CAM_OIS, "aw_soc_jump_boot error!");
		return OIS_ERROR;
	}

	return ret;
}

static int32_t aw86006_flash_update(struct vendor_ois_ctrl *ois,
	uint32_t addr, u8 *data_p, size_t fw_size)
{
	int32_t ret;
	int32_t i;

	CAM_INFO(CAM_OIS, "update 0x%08x via soc", addr);
	if (!data_p) {
		CAM_ERR(CAM_OIS, "error allocating memory");
		return OIS_ERROR;
	}
	/* enter boot mode */
	for (i = 0; i < AW_ERROR_LOOP; i++) {
		ret = aw86006_jump_boot(ois);
		if (ret != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "jump boot failed!,loop:%d", i);
		} else {
			CAM_DBG(CAM_OIS, "jump boot success");
			break;
		}
		if (i == (AW_ERROR_LOOP - 1))
			return OIS_ERROR;
	}
	ret = aw86006_flash_download_check(ois, addr, data_p, fw_size);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "flash download failed!");
		return ret;
	}
	ret = aw86006_ois_reset(ois);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "reset failed!");
		return ret;
	}
	msleep(AW_RESET_DELAY);
	CAM_DBG(CAM_OIS, "soc update success!");

	return ret;
}

static int32_t aw86006_get_standby_flag(struct vendor_ois_ctrl *ois, uint8_t *pflag)
{
	uint8_t temp_addr;
	int32_t ret = OIS_SUCCESS;

	temp_addr = ois->o_ctrl->io_master_info.cci_client->sid;
	ois->o_ctrl->io_master_info.cci_client->sid = AW_SHUTDOWN_I2C_ADDR;
	ret = ois_r_reg_val_blk(ois, AW86006_STANDBY_ADDRESS, (uint32_t *)pflag, 1,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
	ois->o_ctrl->io_master_info.cci_client->sid = temp_addr;
	if (ret != OIS_SUCCESS)
		CAM_ERR(CAM_OIS, "read standby flag error, ret: %d", ret);
	else
		CAM_INFO(CAM_OIS, "standby flag: %d", *pflag);

	return ret;
}

static int32_t aw86006_isp_jump_move(struct vendor_ois_ctrl *ois)
{
	uint8_t boot_cmd[2] = {0xF0, 0xF0};
	uint8_t version_cmd[2] = {0x00, 0x55};
	uint8_t version_ack[5] = {0x00};
	int32_t ret = OIS_SUCCESS;
	int32_t jump_loop = AW_ERROR_LOOP;
	uint32_t move_version;
	int32_t i;
	uint32_t ms_count_reset, ms_count_stop;
	ktime_t kstart, kend;
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	ois->o_ctrl->io_master_info.not_need_dmd_report = true;
	do {
		ret = aw86006_ois_reset(ois);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "software reset error:%d", jump_loop);
			continue;
		}
		kstart = ktime_get();
		for (i = AW_ISP_RESET_DELAY_MAX; i > 0; i--) {
			mdelay(1);
			kend = ktime_get();
			ms_count_reset = ktime_to_ms(ktime_sub(kend, kstart));
			/* aw suggest mdelay 9ms, less than 16ms */
			if ((ms_count_reset >= AW_ISP_RESET_DELAY) && (ms_count_reset < AW_ISP_RESET_DELAY_MAX))
				break;
		}
		if (ms_count_reset >= AW_ISP_RESET_DELAY_MAX) {
			CAM_ERR(CAM_OIS, "ms_count_reset timeout: %u, loop: %d", ms_count_reset, jump_loop);
			continue;
		}

		/* enter isp mode */
		for (i = 0; i < AW_ISP_WRITE_ACK_LOOP; i++) {
			ret = ois_w_reg8_val8_blk(ois, boot_cmd[0],
					 &boot_cmd[1], sizeof(boot_cmd) - 1, 0);
			if ((ret < 0) && (i >= AW_STOP_CMD_LOOP))
				break;
		}
		kend = ktime_get();
		ms_count_stop = ktime_to_ms(ktime_sub(kend, kstart));
		CAM_INFO(CAM_OIS, "ms_count_reset: %u, ms_count_stop: %u", ms_count_reset, ms_count_stop);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "write 0xF0 error:%d, i: %d", ret, i);
			continue;
		}
		ret = ois_w_reg8_val8_blk(ois, version_cmd[0],
				 &version_cmd[1], sizeof(version_cmd) - 1, 0);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "write version_cmd:%d", jump_loop);
			continue;
		}
		usleep_range(ISP_READ_VERSION_DELAY, ISP_READ_VERSION_DELAY + TIME_50US);
		ret = aw86006_cci_stand_read(ois, ISP_VERS_CONNECT_ACK,
			CAMERA_SENSOR_I2C_TYPE_BYTE,
			&version_ack[0], ISP_VERSION_ACK_LEN);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "read version error:%d", jump_loop);
			continue;
		}
		if (version_ack[0] != ISP_EVENT_OK) {
			CAM_ERR(CAM_OIS, "wrong version_ack:%d, loop:%d",
				version_ack[0], jump_loop);
			continue;
		} else {
			move_version = (version_ack[4] << AW86006_OFFSET_24BIT) |
				(version_ack[3] << AW86006_OFFSET_16BIT) |
				(version_ack[2] << AW86006_OFFSET_8BIT) | version_ack[1];
			if (move_version != priv->fw_info.move_version) {
				CAM_ERR(CAM_OIS, "move_version error: 0x%08X", move_version);
				continue;
			} else {
				CAM_DBG(CAM_OIS, "Jump move success!");
				ret = OIS_SUCCESS;
			}
			break;
		}
	} while (--jump_loop);
	if (jump_loop == 0)
		ret = OIS_ERROR;
	ois->o_ctrl->io_master_info.not_need_dmd_report = false;

	return ret;
}

static int32_t aw86006_runtime_check(struct vendor_ois_ctrl *ois)
{
	uint16_t version = 0;
	uint16_t chip_id = 0;
	uint8_t reg_val[4] = {0};
	int32_t ret;
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	ret = aw86006_cci_stand_read(ois, AW86006_CHIP_ID_ADDRESS,
		CAMERA_SENSOR_I2C_TYPE_WORD, &reg_val[0], 4);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "read reg failed!");
		return OIS_ERROR;
	}
	chip_id = (reg_val[0] << 8) | (reg_val[1]);
	version = (reg_val[2] << 8) | (reg_val[3]);
	CAM_INFO(CAM_OIS, "chip_id:0x%02X, version:0x%02X", chip_id, version);
	if ((chip_id != priv->fw_info.app_id) ||
		(version != priv->fw_info.app_version))
		return OIS_ERROR;
	CAM_INFO(CAM_OIS, "pass!");

	return OIS_SUCCESS;
}


static int32_t aw86006_checkinfo_analyse(struct vendor_ois_ctrl *ois,
	uint8_t *checkinfo_rd, struct aw_fw_info *info, int32_t len)
{
	uint32_t temp = 0;
	int32_t ret;
	int32_t i;
	uint32_t *check_ptr = NULL;
	char fw_check_str[] = {'A', 'W', 'I', 'N', 'I', 'C', 0, 0};
	uint32_t offset = AW_FLASH_MOVE_LENGTH;

	if ((checkinfo_rd == NULL) || (info == NULL)) {
		CAM_ERR(CAM_OIS, "checkinfo empty!");
		return OIS_ERROR;
	}
	if (len > AW_FW_INFO_LENGTH)
		return OIS_ERROR;
	ret = memcmp(fw_check_str, (char *)checkinfo_rd, 8);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "checkinfo not match!");
		return OIS_ERROR;
	}

	info->checksum =
		*(uint32_t *)(&checkinfo_rd[AW_FW_SHIFT_CHECKSUM - offset]);
	info->app_checksum =
		*(uint32_t *)(&checkinfo_rd[AW_FW_SHIFT_APP_CHECKSUM - offset]);
	info->app_length =
		*(uint32_t *)(&checkinfo_rd[AW_FW_SHIFT_APP_LENGTH - offset]) +
		AW_FW_INFO_LENGTH; /* add check_info size */
	info->app_version =
		*(uint32_t *)(&checkinfo_rd[AW_FW_SHIFT_APP_VERSION - offset]);
	info->app_id =
		(checkinfo_rd[AW_FW_SHIFT_APP_ID - offset] << 8) |
		(checkinfo_rd[AW_FW_SHIFT_APP_ID - offset + 1]);
	info->move_checksum =
		*(uint32_t *)(&checkinfo_rd[AW_FW_SHIFT_MOVE_CHECKSUM - offset]);
	info->move_version =
		*(uint32_t *)(&checkinfo_rd[AW_FW_SHIFT_MOVE_VERSION - offset]);
	info->move_length =
		*(uint32_t *)(&checkinfo_rd[AW_FW_SHIFT_MOVE_LENGTH - offset]);

	CAM_INFO(CAM_OIS,
		"checkinfo: checksum:0x%08X, app_checksum:0x%08X, app_length:0x%04X, \
		app_version:0x%04X, app_id:0x%04X", info->checksum, info->app_checksum,
		info->app_length, info->app_version, info->app_id);
	CAM_INFO(CAM_OIS, "checkinfo: move_checksum:0x%08X, move_version:0x%08X, \
		move_length:0x%04X", info->move_checksum,
		info->move_version, info->move_length);

	/* info checksum check */
	check_ptr = (uint32_t *) &checkinfo_rd[AW_FW_SHIFT_CHECKSUM_ADDR - offset];
	for (i = 0; i < (AW_FW_INFO_LENGTH - 8 - 4) / 4; i++)
		temp += check_ptr[i];
	if (temp != info->checksum) {
		CAM_ERR(CAM_OIS, "checkinfo_rd checksum error:0x%08X != 0x%08X",
			info->checksum, temp);
		return OIS_ERROR;
	}
	CAM_INFO(CAM_OIS, "pass!");

	return OIS_SUCCESS;
}

static int32_t aw86006_mem_download(struct vendor_ois_ctrl *ois,
	const struct firmware *fw)
{
	int32_t i;
	int32_t ret = OIS_ERROR;
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);
	uint8_t *all_buf_ptr = (uint8_t *)fw->data;
	size_t all_buf_size = priv->fw_info.app_length + AW_FLASH_MOVE_LENGTH;
	uint8_t *app_buf_ptr = (uint8_t *) fw->data + AW_FLASH_MOVE_LENGTH;
	struct aw_fw_info info_rd;

	ret = aw86006_jump_boot(ois);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "jump boot error!");
		return OIS_ERROR;
	}
	ret = aw86006_flash_read_check(ois, AW_FLASH_APP_ADDR,
		&priv->checkinfo_rd[0], AW_FLASH_READ_LEN);
	if (ret != OIS_SUCCESS)
		CAM_ERR(CAM_OIS, "readback checkinfo error!");
	priv->fw_info.update_flag =
		priv->checkinfo_rd[AW_ARRAY_SHIFT_UPDATE_FLAG];

	if (priv->fw_info.update_flag != 0x01) {
		CAM_INFO(CAM_OIS, "update_flag error!");
		ret = aw86006_flash_update(ois, AW_FLASH_BASE_ADDR,
			all_buf_ptr, all_buf_size);
		if (ret != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "flash update failed!");
			return ret;
		}
	} else { /* update success last time */
		CAM_INFO(CAM_OIS, "update_flag ok!");
		ret = memcmp(priv->checkinfo_rd, priv->checkinfo_fw,
			AW_FW_INFO_LENGTH);
		if (ret != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "read checkinfo not match!");
			ret = aw86006_checkinfo_analyse(ois,
				priv->checkinfo_rd, &info_rd, AW_FW_INFO_LENGTH);
			if ((ret != OIS_SUCCESS) || (info_rd.move_version !=
				priv->fw_info.move_version)) {
				CAM_ERR(CAM_OIS, "checkinfo or move not match, update all!");
				ret = aw86006_flash_update(ois, AW_FLASH_BASE_ADDR,
					all_buf_ptr, all_buf_size);
				if (ret != OIS_SUCCESS) {
					CAM_ERR(CAM_OIS, "flash update failed!");
					return ret;
				}
			} else if ((info_rd.app_version != priv->fw_info.app_version) ||
				(info_rd.app_id != priv->fw_info.app_id)) {
				CAM_ERR(CAM_OIS, "app not match, update app!");
				ret = aw86006_flash_update(ois, AW_FLASH_APP_ADDR,
					app_buf_ptr, priv->fw_info.app_length);
				if (ret != OIS_SUCCESS) {
					CAM_ERR(CAM_OIS, "flash update failed!");
					return ret;
				}
			} else {
				CAM_ERR(CAM_OIS, "other errors, update all!");
				ret = aw86006_flash_update(ois, AW_FLASH_BASE_ADDR,
					all_buf_ptr, all_buf_size);
				if (ret != OIS_SUCCESS) {
					CAM_ERR(CAM_OIS, "flash update failed!");
					return ret;
				}
			}
		} else {
			ret = aw86006_isp_jump_move(ois);
			if (ret != OIS_SUCCESS) {
				CAM_ERR(CAM_OIS, "aw86006_isp_jump_move fail, update all!");
				ret = aw86006_flash_update(ois, AW_FLASH_BASE_ADDR,
					all_buf_ptr, all_buf_size);
				if (ret != OIS_SUCCESS) {
					CAM_ERR(CAM_OIS, "flash update failed!");
					return ret;
				}
			} else {
				ret = aw86006_ois_reset(ois);
				if (ret != OIS_SUCCESS) {
					CAM_ERR(CAM_OIS, "reset failed!");
					return OIS_ERROR;
				}
				msleep(AW_RESET_DELAY);
			}
		}
	}

	for (i = 0; i <= AW_ERROR_LOOP; i++) {
		if (aw86006_runtime_check(ois) == OIS_SUCCESS) {
			CAM_DBG(CAM_OIS, "runtime_check pass, no need to update fw!");
			ret = OIS_SUCCESS;
			break;
		} else {
			CAM_ERR(CAM_OIS, "runtime_check failed! loop:%d", i);
		}
		if (i == AW_ERROR_LOOP) {
			CAM_ERR(CAM_OIS, "fw update failed!");
			ret = OIS_ERROR;
			break;
		}
		ret = aw86006_flash_update(ois, AW_FLASH_APP_ADDR,
			app_buf_ptr, priv->fw_info.app_length);
		if (ret != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "flash update failed! loop:%d", i);
			continue;
		}
	}

	return ret;
}

static int32_t aw86006_download_ois_fw(struct vendor_ois_ctrl *ois,
	const struct firmware *fw)
{
	int32_t ret = OIS_ERROR;
	uint8_t standby_flag = 0;
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	if (priv->already_download) {
		CAM_INFO(CAM_OIS, "fw already downloaded");
		aw86006_ois_reset(ois);
		msleep(AW_RESET_DELAY);
		return OIS_SUCCESS;
	}
	ret = aw86006_ois_reset(ois);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "AW86006 reset error:%d", ret);
		return ret;
	}
	msleep(AW_RESET_DELAY);

	/* Get standby flag */
	ret = aw86006_get_standby_flag(ois, &standby_flag);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "AW86006 get standby flag failed:%d", ret);
		return ret;
	}
	if (standby_flag == AW_IC_STANDBY) {
		ret = aw86006_runtime_check(ois);
		if (ret == OIS_SUCCESS)
			CAM_INFO(CAM_OIS, "AW86006 runtime_check pass, no need to update fw!");
	}
	if ((standby_flag != AW_IC_STANDBY) || (ret != OIS_SUCCESS)) {
		/* update flash */
		ret = aw86006_mem_download(ois, fw);
		if (ret != OIS_SUCCESS) {
			CAM_ERR(CAM_OIS, "aw86006_mem_download fail:%d", ret);
			return ret;
		} else {
			priv->already_download = false; /* true masked */
			CAM_INFO(CAM_OIS, "AW86006 fw update success!");
		}
	}

	return OIS_SUCCESS;
}

static int32_t aw86006_fw_analysis(struct vendor_ois_ctrl *ois,
	const struct firmware *fw)
{
	uint32_t temp = 0;
	uint32_t len;
	int32_t i, ret;
	uint32_t *check_ptr = NULL;
	char fw_check_str[] = {'A', 'W', 'I', 'N', 'I', 'C', 0, 0};
	char *identify = (char *) fw->data + AW_FW_SHIFT_IDENTIFY;
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	ret = memcmp(fw_check_str, identify, 8);
	if (ret != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "loaded wrong firmware!");
		return OIS_ERROR;
	}

	priv->fw_info.size = fw->size;
	priv->fw_info.checksum =
		*(uint32_t *)(&fw->data[AW_FW_SHIFT_CHECKSUM]);
	priv->fw_info.app_checksum =
		*(uint32_t *)(&fw->data[AW_FW_SHIFT_APP_CHECKSUM]);
	priv->fw_info.app_length =
		*(uint32_t *)(&fw->data[AW_FW_SHIFT_APP_LENGTH]) +
		AW_FW_INFO_LENGTH; /* add check_info size */
	priv->fw_info.app_version =
		*(uint32_t *)(&fw->data[AW_FW_SHIFT_APP_VERSION]);
	priv->fw_info.app_id =
		(fw->data[AW_FW_SHIFT_APP_ID] << 8) |
		(fw->data[AW_FW_SHIFT_APP_ID + 1]);
	priv->fw_info.move_checksum =
		*(uint32_t *)(&fw->data[AW_FW_SHIFT_MOVE_CHECKSUM]);
	priv->fw_info.move_version =
		*(uint32_t *)(&fw->data[AW_FW_SHIFT_MOVE_VERSION]);
	priv->fw_info.move_length =
		*(uint32_t *)(&fw->data[AW_FW_SHIFT_MOVE_LENGTH]);

	CAM_INFO(CAM_OIS, "info_checksum:0x%08X, fw->size:0x%04X",
		priv->fw_info.checksum, priv->fw_info.size);
	CAM_INFO(CAM_OIS, "app_checksum:0x%08X, app_length:0x%04X, \
		app_version:0x%04X, app_id:0x%04X",
		priv->fw_info.app_checksum, priv->fw_info.app_length,
		priv->fw_info.app_version, priv->fw_info.app_id);
	CAM_INFO(CAM_OIS, "move_checksum:0x%08X, move_version:0x%08X, \
		move_length:0x%04X", priv->fw_info.move_checksum,
		priv->fw_info.move_version, priv->fw_info.move_length);

	/* length check */
	temp = priv->fw_info.move_length + priv->fw_info.app_length;
	if (priv->fw_info.size != temp) {
		CAM_ERR(CAM_OIS, "fw->size error: 0x%X != 0x%X",
			priv->fw_info.size, temp);
		return OIS_ERROR;
	}
	/* move checksum check */
	check_ptr = (uint32_t *)&fw->data[0];
	for (i = temp = 0; i < priv->fw_info.move_length / 4; i++)
		temp += check_ptr[i];
	if (temp != priv->fw_info.move_checksum) {
		CAM_ERR(CAM_OIS, "move checksum error:0x%08X != 0x%08X",
			priv->fw_info.move_checksum, temp);
		return OIS_ERROR;
	}

	/* info checksum check */
	check_ptr = (uint32_t *)&fw->data[AW_FW_SHIFT_CHECKSUM_ADDR];
	for (i = temp = 0; i < (AW_FW_INFO_LENGTH - 8 - 4) / 4; i++)
		temp += check_ptr[i];
	if (temp != priv->fw_info.checksum) {
		CAM_ERR(CAM_OIS, "check_info checksum error:0x%08X != 0x%08X",
			priv->fw_info.checksum, temp);
		return OIS_ERROR;
	}

	/* app checksum check */
	check_ptr = (uint32_t *)&fw->data[AW_FLASH_MOVE_LENGTH +
		AW_FW_INFO_LENGTH];
	len = (priv->fw_info.app_length - AW_FW_INFO_LENGTH) / 4;
	for (i = temp = 0; i < len; i++)
		temp += check_ptr[i];
	if (temp != priv->fw_info.app_checksum) {
		CAM_ERR(CAM_OIS, "app checksum error:0x%08X != 0x%08X",
			priv->fw_info.app_checksum, temp);
		return OIS_ERROR;
	}
	ret = memcpy_s(&priv->checkinfo_fw[0], AW_FW_INFO_LENGTH,
		fw->data + AW_FLASH_MOVE_LENGTH, AW_FW_INFO_LENGTH);
	CAM_INFO(CAM_OIS, "pass!");

	return OIS_SUCCESS;
}

static int32_t aw86006_fw_download(struct vendor_ois_ctrl *ois)
{
	int32_t rc = 0;
	const struct firmware *fw = NULL;

	if (!ois || !ois->o_ctrl) {
		CAM_ERR(CAM_OIS, "handle is NULL");
		return -EINVAL;
	}

	fw = vendor_request_firmware(ois);
	if (!fw)
		return -EINVAL;

	/* check fw */
	rc = aw86006_fw_analysis(ois, fw);
	if (rc != OIS_SUCCESS) {
		CAM_ERR(CAM_OIS, "aw86006_fw_analysis fail : %d", rc);
		release_firmware(fw);
		return rc;
	}

	rc = aw86006_download_ois_fw(ois, fw);
	if (rc != OIS_SUCCESS)
		CAM_ERR(CAM_OIS, "aw86006_download_ois_fw fail : %d", rc);

	release_firmware(fw);

	return rc;
}

static const struct vendor_ois_ops aw86006_ops = {
	.fw_download = aw86006_fw_download,
};

static struct vendor_ois_ctrl aw86006_driver = {
	.name = "aw86006",
	.ops = &aw86006_ops,
};

int32_t aw86006_driver_init(void)
{
	int32_t rc = 0;
	struct ois_ctrl_priv *priv = NULL;

	priv = (struct ois_ctrl_priv *)kzalloc(
		sizeof(struct ois_ctrl_priv), GFP_KERNEL);
	if (!priv) {
		CAM_ERR(CAM_OIS, "aw86006_download_ois_fw fail : %d", rc);
		return -EINVAL;
	}
	vendor_ois_set_drvdata(&aw86006_driver, priv);
	rc = vendor_ois_register(&aw86006_driver);
	if (rc)
		CAM_ERR(CAM_OIS, "vendor register ois fail : %d", rc);

	return rc;
}

void aw86006_driver_exit(void)
{
	struct ois_ctrl_priv *priv = NULL;

	priv = (struct ois_ctrl_priv *)vendor_ois_get_drvdata(&aw86006_driver);
	if (priv)
		kfree(priv);
	vendor_ois_unregister(&aw86006_driver);
}

MODULE_DESCRIPTION("aw86006 OIS driver");
MODULE_LICENSE("GPL v2");
