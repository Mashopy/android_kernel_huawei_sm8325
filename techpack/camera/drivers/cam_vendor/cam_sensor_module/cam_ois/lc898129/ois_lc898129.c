/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: lc898129 OIS driver
 */
#include <linux/module.h>
#include <linux/dma-contiguous.h>
#include <cam_sensor_cmn_header.h>
#include <cam_ois_core.h>
#include <cam_ois_soc.h>
#include <cam_sensor_util.h>
#include <cam_debug_util.h>
#include <cam_res_mgr_api.h>
#include <cam_common_util.h>
#include <cam_packet_util.h>

#include "vendor_ois_core.h"
#include "ois_lc898129.h"
#include "ois_lc898129_fun.h"

struct fw_desc_t {
	uint32_t fw_checksum_size;
	uint32_t fw_checksum;
	uint32_t fw_version;
};

struct ois_ctrl_priv {
	bool already_download;
	bool force_download;
	struct fw_desc_t desc;
};

static int32_t lc898129_user_mat_write(struct vendor_ois_ctrl *ois,
	const struct firmware *fw)
{
	/* 64 bytes written at a time */
	int32_t write_num1 = (fw->size - FW_OFFSET) / 64;
	int32_t write_num2 = (fw->size - FW_OFFSET) % 64;
	uint8_t data[67]; /* 4*16 + 3 = 67 bytes */
	uint32_t n;
	const uint8_t *pd = (uint8_t *)fw->data;
	uint32_t read_val;
	int32_t i, j;

	if (unlock_code_set129(ois) == SUCCESS) { /* Unlock Code Set */
		hs_write_permission129(ois); /* Write Permission */
		/* Additional Unlock Code Set */
		addtional_unlock_code_set129(ois);
		/* Updata FlashAccess Command Table */
		ois_w_reg16_val32(ois, 0xF007, 0x00000000);
		/* FromCmd Addr Configuration */
		ois_w_reg16_val32(ois, 0xF00A, 0x00000000);
		/* Transfer every 64byte */
		for (i = 0; i < write_num1; i++) {
			n = 0;
			data[n++] = 0xF0; /* CmdH */
			data[n++] = 0x08; /* CmdL */
			data[n++] = 0x00; /* FromCmd address of the bufferA */
			for (j = 0; j < 16; j++) {
				data[n++] = *pd++; /* 1byte */
				data[n++] = *pd++; /* 2byte */
				data[n++] = *pd++; /* 3byte */
				data[n++] = *pd++; /* 4byte */
			}
			hs_cnt_write(ois, data, 67); /* 4*16 + 3 = 67 bytes */

			/* FromCmd Configuration Example */
			ois_w_reg16_val32(ois, 0xF00B, 0x00000010);
			/* Setting FromCmd.Control (Write) */
			ois_w_reg16_val32(ois, 0xF00C, 0x00000004);
			do /* Write completion judgment */
				ois_r_reg16_val32(ois, 0xF00C, &read_val); /* Read from Cmd.Control */
			while (read_val != 0);
		}
		/* Transfer 64byte or less */
		if (write_num2 != 0) {
			n = 0;
			data[n++] = 0xF0; /* CmdH */
			data[n++] = 0x08; /* CmdL */
			data[n++] = 0x00; /* FromCmd address of the bufferA */
			for (j = 0; j < write_num2 / 4; j++) {
				data[n++] = *pd++; /* 1byte */
				data[n++] = *pd++; /* 2byte */
				data[n++] = *pd++; /* 3byte */
				data[n++] = *pd++; /* 4byte */
			}
			hs_cnt_write(ois, data, write_num2 * 4 + 3);
			/* FromCmd Configuration Example */
			ois_w_reg16_val32(ois, 0xF00B, write_num2 / 4);
			/* FromCmd Control Settings (Write) */
			ois_w_reg16_val32(ois, 0xF00C, 0x00000004);
			do { /* write end judgment */
				/* FromCmd readout of Control */
				ois_r_reg16_val32(ois, 0xF00C, &read_val);
				msleep(1);
			} while (read_val != 0);
		}

		if (unlock_code_clear129(ois) == FAILURE) /* Unlock Code Clear */
			return FAILURE;
	} else {
		return FAILURE;
	}
	return SUCCESS;
}

static void lc898129_code_updata_write(struct vendor_ois_ctrl *ois,
	const struct firmware *fw_updata)
{
	/* Calculate the number of elements in the Pmem data. */
	int32_t write_num1 = UP_DATA_CODE_SIZE * 5 / 50;
	int32_t write_num2 = UP_DATA_CODE_SIZE * 5 % 50;
	int32_t write_num3 = write_num2 + 2;
	uint32_t i, num;
	uint8_t data[52] = {0}; /* 5*10+2=52bytes */
	uint32_t n;
	const uint8_t *pd = (uint8_t *)fw_updata->data;

	/* Pmem address set */
	data[0] = 0x30; /* CmdH */
	data[1] = 0x00; /* CmdL */
	data[2] = 0x00; /* DataH */
	data[3] = 0x08; /* DataMH */
	data[4] = 0x00; /* DataML */
	data[5] = 0x00; /* DataL */

	hs_cnt_write(ois, data, 6);
	/* Pmem data write */
	data[0] = 0x40; /* CmdH */
	data[1] = 0x00; /* CmdL */
	for (num = 0; num < write_num1; num++) {
		n = 2;
		for (i = 0; i < 10; i++) {
			data[n++] = *pd++;
			data[n++] = *pd++;
			data[n++] = *pd++;
			data[n++] = *pd++;
			data[n++] = *pd++;
		}
		hs_cnt_write(ois, data, 52); /* 5*10+2=52bytes */
	}
	if (write_num2 != 0) {
		n = 2;
		for (i = 0; i < write_num2; i++)
			data[n++] = *pd++;
		hs_cnt_write(ois, data, write_num3);
	}
}

static uint8_t lc898129_code_updata_read(struct vendor_ois_ctrl *ois,
	const struct firmware *fw_updata)
{
	uint8_t data[6];
	uint8_t read_data[5];
	uint8_t ng_flg = SUCCESS; /* 0x01 = NG  0x00 = OK */
	uint32_t i, j;
	const uint8_t *pd = (uint8_t *)fw_updata->data;

	/* PMEM Address Set */
	data[0] = 0x30; /* CmdH */
	data[1] = 0x00; /* CmdL */
	data[2] = 0x00; /* DataH */
	data[3] = 0x08; /* DataMH */
	data[4] = 0x00; /* DataML */
	data[5] = 0x00; /* DataL */
	hs_cnt_write(ois, data, 6);

	/* Pmem Data Read & Verify */
	for (i = 0; i < UP_DATA_CODE_SIZE; i++) {
		ois_r_reg_val_blk(ois, 0x4000, (uint32_t *)read_data, 5,
			CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
		for (j = 0; j < 5; j++) {
			if (read_data[j] != *pd++)
				ng_flg = FAILURE;
		}
	}
	return ng_flg; /* Verify Result */
}

static uint8_t lc898129_code_updata_checksum(struct vendor_ois_ctrl *ois)
{
	uint8_t data[6];
	uint8_t read_data[8];
	uint32_t ulcnt;
	uint32_t read_val;
	uint8_t uc_snd_dat = SUCCESS; /* 0x01 = NG  0x00 = OK */
	int64_t checksum_code = UP_DATA_CODE_CHECKSUM;
	uint8_t *p = (uint8_t *)&checksum_code;
	uint8_t i;

	/* Launching the CheckSum of Program RAM */
	data[0] = 0xF0; /* CmdID */
	data[1] = 0x0E; /* CmdID */
	/* Write Data (MSB) */
	data[2] = (uint8_t)((UP_DATA_CODE_SIZE >> 8) & 0x000000FF);
	data[3] = (uint8_t)(UP_DATA_CODE_SIZE & 0x000000FF);
	data[4] = 0x00; /* Write Data */
	data[5] = 0x00; /* Write Data(LSB) */
	hs_cnt_write(ois, data, 6);

	/* Checksum termination judgment */
	ulcnt = 0;
	do {
		if (ulcnt++ > 100) {
			uc_snd_dat = FAILURE;
			break;
		}
		/* Reading PMCheck.ExecFlag */
		ois_r_reg16_val32(ois, 0x0088, &read_val);
	} while (read_val != 0);

	/* Read CheckSum Values */
	if (uc_snd_dat == SUCCESS) {
		ois_r_reg_val_blk(ois, 0xF00E, (uint32_t *)read_data, 8,
			CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);

		/*
		 * Determine the CheckSum value
		 * The expected value is defined in the Header
		 */
		for (i = 0; i < 8; i++) {
			if (read_data[7 - i] != *p++) /* CheckSum code judgment */
				uc_snd_dat = FAILURE;
		}
	}
	return uc_snd_dat;
}

static uint8_t lc898129_user_mat_erase(struct vendor_ois_ctrl *ois)
{
	uint32_t addr;
	uint32_t ulcnt;
	uint32_t read_val;
	uint8_t uc_snd_dat = SUCCESS; /* 0x01 = NG  0x00 = OK */
	uint32_t i;

	if (unlock_code_set129(ois) == SUCCESS) { /* Unlock Code Set */
		hs_write_permission129(ois); /* Write Permission */
		addtional_unlock_code_set129(ois); /* Additional Unlock Code Set */
		/* Updata FlashAccess Command Table */
		ois_w_reg16_val32(ois, 0xF007, 0x00000000);

		/* Clear User Mat Block  */
		for (i = 0; i < ERASE_BLOCKS; i++) {
			addr = i << 10;
			/* FromCmd Addr Configuration */
			ois_w_reg16_val32(ois, 0xF00A, addr);
			/* FromCmd control settings (clear blocks)) */
			ois_w_reg16_val32(ois, 0xF00C, 0x00000020);

			/* Block Erase End Judgment */
			ulcnt = 0;
			do {
				if (ulcnt++ > 100) {
					uc_snd_dat = -3; /* FAILURE; */
					break;
				}
				/* FromCmd readout of Control */
				ois_r_reg16_val32(ois, 0xF00C, &read_val);
				msleep(1);
			} while (read_val != 0);
			if (uc_snd_dat == -3)
				break;
		}

		if (unlock_code_clear129(ois) == FAILURE) /* Unlock Code Clear */
			uc_snd_dat = -2; /* FAILURE; */
	} else {
		uc_snd_dat = -1; /* FAILURE; */
	}

	return uc_snd_dat;
}

static uint8_t lc898129_user_mat_checksum(struct vendor_ois_ctrl *ois,
	const struct firmware *fw)
{
	uint32_t read_val;
	uint8_t uc_snd_dat = SUCCESS; /* 0x01 = NG  0x00 = OK */
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	/* FromCmd Addr Configuration */
	ois_w_reg16_val32(ois, 0xF00A, 0x00000000);
	/* ptr->SizeFromCodeValid   Set the CheckSum size */
	ois_w_reg16_val32(ois, 0xF00D, priv->desc.fw_checksum_size);
	/* FromCmd Set Control (Execute CheckSum) */
	ois_w_reg16_val32(ois, 0xF00C, 0x00000100);
	do { /* write end judgment */
		/* FromCmd readout of Control */
		ois_r_reg16_val32(ois, 0xF00C, &read_val);
		msleep(1);
	} while (read_val != 0);

	/* Read CheckSum Values */
	ois_r_reg16_val32(ois, 0xF00D, &read_val);
	/* ptr->SizeFromCodeCksm */
	if (read_val != (uint32_t)priv->desc.fw_checksum)
		uc_snd_dat = FAILURE;

	return uc_snd_dat;
}

static int32_t lc898129_updata_hs_flash(struct vendor_ois_ctrl *ois,
	const struct firmware *fw, const struct firmware *fw_updata)
{
	uint32_t uc_snd_dat = SUCCESS; /* 0x01 = FAILURE, 0x00 = SUCCESS */
	uint32_t ul_data_val;

	/* Adjusting OSC, LDO, and SMA Measurement Currents */
	uc_snd_dat = hs_drv_off_adj(ois);
	if (uc_snd_dat != 0 && uc_snd_dat != 1)
		return uc_snd_dat;
	/* Go to Boot Mode */
	hs_boot_mode(ois);

	/* UpData Execution */
	hs_io_read32(ois, 0xD000AC, &ul_data_val);
	if ((ul_data_val & 0x00000001) == 0) {
		/* Turn off MC_IGNORE2 */
		hs_io_write32(ois, 0xD000AC, 0x00000000);
		/* Write Updateta Code to Program RAM */
		lc898129_code_updata_write(ois, fw_updata);
	} else {
		return -1; /* Boot Mode Migration Failed */
	}
	if (lc898129_code_updata_read(ois, fw_updata) == FAILURE)
		return -2; /* The Read Verify of the UpData code is incorrect */
	if (lc898129_code_updata_checksum(ois) == FAILURE)
		return -3; /* UpData the value of CheckSum in the code is NG */
	/* FLASH Standby Disable */
	hs_io_write32(ois, 0xE0701C, 0x00000000);
	if (lc898129_user_mat_erase(ois) == FAILURE)
		return -4; /* Flash Memory Error */
	if (lc898129_user_mat_write(ois, fw) == FAILURE)
		return -5; /* Flash Memory Write Failed */
	if (lc898129_user_mat_checksum(ois, fw) == FAILURE)
		return -6; /* The value of CheckSum in the Flash Memory is NG */
	hs_io_write32(ois, 0xD000AC, 0x00001000); /* CORE_RST ON */
	msleep(30); /* 30 [msec] Waiting */
	hs_io_read32(ois, 0xD000AC, &ul_data_val);
	if (ul_data_val != 0x000000A1)
		return -7; /* ReMap Failed */

	return SUCCESS;
}

static uint8_t lc898129_status_read(struct vendor_ois_ctrl *ois,
	uint32_t bit_check)
{
	uint32_t read_val;

	ois_r_reg16_val32(ois, OIS129_CMD_READ_STATUS, &read_val);
	if (bit_check)
		read_val &= OIS129_READ_STATUS_INI;
	if (!read_val)
		return 0;
	else
		return 1;
}

static void lc898129_set_active_mode(struct vendor_ois_ctrl *ois)
{
	uint8_t st_rd = 1;
	uint32_t st_cnt = 0;

	hs_io_write32(ois, 0xD01008, 0x00000090);
	ois_w_reg16_val32(ois, 0xF019, 0x00000000);

	while (st_rd && (st_cnt++ < OIS129_CNT050MS))
		st_rd = lc898129_status_read(ois, 1);
}

static void lc898129_protocol_updata(struct vendor_ois_ctrl *ois)
{
	uint32_t tmp = 0;
	static uint16_t g_ois_data_protocol = 0;

	ois_r_reg16_val32(ois, EIS_DATA_VERSION, (uint32_t *)&tmp);
	g_ois_data_protocol = (tmp & (EIS_VERSION_MASK << EIS_VERSION_MASK_SHIFT))
		>> EIS_VERSION_MASK_SHIFT;
}

static int32_t lc898129_fw_download_internel(struct vendor_ois_ctrl *ois,
	const struct firmware *fw, const struct firmware *fw_updata)
{
	uint32_t fw_version_current = 0;
	int32_t result = 0;
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	ois_r_reg16_val32(ois, FW_VER_ADDR,
		(uint32_t *)&fw_version_current);
	CAM_INFO(CAM_OIS, "lc898129 fw_version = 0x%x", fw_version_current);
	if (fw_version_current == priv->desc.fw_version) {
		CAM_INFO(CAM_OIS, "Latest version");
		return 0;
	}
	lc898129_set_active_mode(ois);
	result = lc898129_updata_hs_flash(ois, fw, fw_updata);
	hs_io_write32(ois, 0xE0701C, 0x00000002); /* FLASH Standby Enable */
	if (result == SUCCESS)
		CAM_INFO(CAM_OIS, "fw download success");
	else
		CAM_ERR(CAM_OIS, "fw download failed ,result = %d", result);
	lc898129_protocol_updata(ois);

	return 0;
}

static void lc898129_fw_analysis(struct vendor_ois_ctrl *ois,
	const struct firmware *fw)
{
	uint32_t index = fw->size;
	const uint8_t *pd = (uint8_t *)fw->data - 1;
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	priv->desc.fw_version = pd[index] + (pd[index - 1] << 8) +
		(pd[index - 2] << 16) + (pd[index - 3] << 24);
	index -= 4; /* fw_version 4bytes = 32 bits */
	priv->desc.fw_checksum = pd[index] + (pd[index - 1] << 8) +
		(pd[index - 2] << 16) + (pd[index - 3] << 24);
	index -= 4; /* fw_checksum 4bytes = 32 bits */
	priv->desc.fw_checksum_size = pd[index] + (pd[index - 1] << 8) +
		(pd[index - 2] << 16) + (pd[index - 3] << 24);
	CAM_INFO(CAM_OIS, "fw_version = 0x%x, fw_chesum = 0x%x, size = 0x%x",
		priv->desc.fw_version, priv->desc.fw_checksum,
		priv->desc.fw_checksum_size);
}

static int32_t lc898129_fw_download(struct vendor_ois_ctrl *ois)
{
	int32_t rc;
	const struct firmware *fw = NULL;
	const struct firmware *fw_updata = NULL;
	struct device *dev = NULL;
	struct cam_ois_ctrl_t *o_ctrl = NULL;

	if (!ois || !ois->o_ctrl) {
		CAM_ERR(CAM_OIS, "handle is NULL");
		return -EINVAL;
	}
	o_ctrl = ois->o_ctrl;
	dev = &(o_ctrl->pdev->dev);

	fw = vendor_request_firmware(ois);
	if (!fw)
		return -EINVAL;

	/* Load updatacode FW */
	rc = request_firmware(&fw_updata, FW_UPDATA_NAME, dev);
	if (rc) {
		release_firmware(fw);
		CAM_ERR(CAM_OIS, "Failed to locate %s", FW_UPDATA_NAME);
		return rc;
	}
	lc898129_fw_analysis(ois, fw);
	rc = lc898129_fw_download_internel(ois, fw, fw_updata);
	if (rc)
		CAM_ERR(CAM_OIS, "download_ois_fw fail");
	release_firmware(fw);
	release_firmware(fw_updata);

	return rc;
}

static const struct vendor_ois_ops lc898129_ops = {
	.fw_download = lc898129_fw_download,
};

static struct vendor_ois_ctrl lc898129_driver = {
	.name = "lc898129",
	.ops = &lc898129_ops,
};

int32_t lc898129_driver_init(void)
{
	int32_t rc = 0;
	struct ois_ctrl_priv *priv = NULL;

	priv = (struct ois_ctrl_priv *)kzalloc(
		sizeof(struct ois_ctrl_priv), GFP_KERNEL);
	if (!priv) {
		CAM_ERR(CAM_OIS, "lc898129_driver_init fail : %d", rc);
		return -EINVAL;
	}
	vendor_ois_set_drvdata(&lc898129_driver, priv);
	rc = vendor_ois_register(&lc898129_driver);
	if (rc)
		CAM_ERR(CAM_OIS, "vendor register ois fail : %d", rc);

	return rc;
}

void lc898129_driver_exit(void)
{
	struct ois_ctrl_priv *priv = NULL;

	priv = (struct ois_ctrl_priv *)vendor_ois_get_drvdata(&lc898129_driver);
	if (priv)
		kfree(priv);
	vendor_ois_unregister(&lc898129_driver);
}

MODULE_DESCRIPTION("lc898129 OIS driver");
MODULE_LICENSE("GPL v2");
