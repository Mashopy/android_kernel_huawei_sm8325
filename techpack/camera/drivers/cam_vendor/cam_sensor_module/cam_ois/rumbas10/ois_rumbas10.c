/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor rumbas10 OIS driver
 */
#include <cam_sensor_cmn_header.h>

#include "ois_rumbas10.h"
#include "vendor_ois_core.h"

struct fw_desc_t {
	uint32_t fw_size;
	uint16_t checksum;
	uint16_t version;
};

struct ois_ctrl_priv {
	bool already_download;
	struct fw_desc_t desc;
};

static int32_t rumbas10_checksum_veritfy(struct vendor_ois_ctrl *ois)
{
	uint32_t i;
	uint32_t read_data = 0;
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	/* Set the checksum */
	ois_w_reg16_val32_rvt(ois, CHECKSUM_DATA_ADDR, priv->desc.checksum);

	/* Request the checksum */
	ois_w_reg16_val32_rvt(ois, CHECKSUM_CTRL_ADDR, 0x00000001);
	/* Wait for 1s at most */
	for (i = 0; i < 500; i++) {
		ois_r_reg16_val32_rvt(ois, CHECKSUM_CTRL_ADDR, &read_data);
		read_data &= 0x00000001;
		if (read_data == 0x00000000)
			break;
		msleep(2);
	}
	if (i >= 500) {
		CAM_ERR(CAM_OIS, "rumbas10 request chechsum timeout");
		return -ETIME;
	}

	/* Check the checksum */
	ois_r_reg16_val32_rvt(ois, OIS_ERROR_ADDR, &read_data);
	if (read_data == 0) {
		CAM_INFO(CAM_OIS, "checksum success");
	} else {
		CAM_ERR(CAM_OIS, "checksum failed, the read data 0x%08x", read_data);
		return -EIO;
	}

	return 0;
}

static int32_t rumbas10_flash_write(struct vendor_ois_ctrl *ois,
	const struct firmware *fw)
{
	uint32_t i;
	uint32_t j;
	uint32_t read_data = 0;
	const uint8_t *firmware_data = (uint8_t *)fw->data;

	/* Release protection */
	ois_w_reg16_val32_rvt(ois, PROTECTION_ADDR, 0x0070FFF9);
	/* Select Flash page block */
	ois_w_reg16_val32_rvt(ois, FLASH_ADDR, 0x00000000);
	for (i = 0; i < BLOCK_WRITE_LOOP_CONUT; i++) {
		/* Execute write command */
		ois_w_reg16_val32_rvt(ois, FLASH_CMD_ADDR, 0x00000002);
		/* Write data */
		ois_w_reg16_val8_blk(ois, FLASH_DATA_ADDR,
			(uint8_t *)(firmware_data + BLOCK_WRITE_SIZE * i), BLOCK_WRITE_SIZE, 1);

		/* Wait for 10ms at most */
		mdelay(2);
		for (j = 0; j < 800; j++) {
			ois_r_reg16_val32_rvt(ois, FLASH_STATUS_ADDR, &read_data);
			read_data &= 0x00000002;
			if (read_data == 0x00000000)
				break;
			udelay(10);
		}
		if (j >= 800) {
			CAM_ERR(CAM_OIS, "rumbas10 write flash timeout");
			return -ETIME;
		}
	}
	/* Execute NOP command */
	ois_w_reg16_val32_rvt(ois, FLASH_CMD_ADDR, 0x00000000);
	return 0;
}

static int32_t rumbas10_flash_erase_block(struct vendor_ois_ctrl *ois,
	uint16_t flcad_dada, uint16_t flccmd_dada)
{
	uint32_t i;
	uint32_t read_data = 0;

	/* Select erase block */
	ois_w_reg16_val32_rvt(ois, FLASH_ADDR, flcad_dada);
	/* Execute erase command */
	ois_w_reg16_val32_rvt(ois, FLASH_CMD_ADDR, flccmd_dada);
	/* Wait for 10ms at most */
	msleep(5);
	for (i = 0; i < 500; i++) {
		ois_r_reg16_val32_rvt(ois, FLASH_STATUS_ADDR, &read_data);
		read_data &= 0x00000002;
		if (read_data == 0x00000000)
			break;
		udelay(10);
	}
	if (i >= 500) {
		CAM_ERR(CAM_OIS, "rumbas10 erase flash timeout");
		return -ETIME;
	}
	/* Execute NOP command */
	ois_w_reg16_val32_rvt(ois, FLASH_CMD_ADDR, 0x00000000);
	return 0;
}

static int32_t rumbas10_flash_erase(struct vendor_ois_ctrl *ois)
{
	int32_t rc = 0;

	/* Release protection (mat 0) */
	ois_w_reg16_val32_rvt(ois, PROTECTION_ADDR, 0x00000009);
	/* Flash erase (mat 0) */
	rc = rumbas10_flash_erase_block(ois, 0x00000000, 0x00000005);
	if (rc) {
		CAM_ERR(CAM_OIS, "rumbas10 erase mat 0 error, result = %d", rc);
		return rc;
	}

	/* Release protection (block 12-14) */
	ois_w_reg16_val32_rvt(ois, PROTECTION_ADDR, 0x00700001);
	/* Flash erase (block 12) */
	rc = rumbas10_flash_erase_block(ois, 0x00003000, 0x00000004);
	if (rc) {
		CAM_ERR(CAM_OIS, "rumbas10 erase block 12 error, result = %d", rc);
		return rc;
	}
	/* Flash erase (block 13) */
	rc = rumbas10_flash_erase_block(ois, 0x00003400, 0x00000004);
	if (rc) {
		CAM_ERR(CAM_OIS, "rumbas10 erase block 13 error, result = %d", rc);
		return rc;
	}
	/* Flash erase (block 14) */
	rc = rumbas10_flash_erase_block(ois, 0x00003800, 0x00000004);
	if (rc) {
		CAM_ERR(CAM_OIS, "rumbas10 erase block 14 error, result = %d", rc);
		return rc;
	}

	return rc;
}

static int32_t rumbas10_flash_download(struct vendor_ois_ctrl *ois,
	const struct firmware *fw)
{
	uint32_t read_data = 0;
	uint16_t fw_current_version = 0;
	int32_t  rc;
	int32_t  i;
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	/* Stop the CPU */
	ois_w_reg16_val32_rvt(ois, 0x0534, 0x00000001);
	/* 1. Flash erase */
	rc = rumbas10_flash_erase(ois);
	if (rc) {
		CAM_ERR(CAM_OIS, "rumbas10 erase flash error, result = %d", rc);
		return rc;
	}
	/* 2. Flash write */
	rc = rumbas10_flash_write(ois, fw);
	if (rc) {
		CAM_ERR(CAM_OIS, "rumbas10 write flash error, result = %d", rc);
		return rc;
	}
	/* Boot address setting */
	ois_w_reg16_val32_rvt(ois, 0x401C, 0x000000C1);
	/* Restart the CPU */
	ois_w_reg16_val32_rvt(ois, 0x0534, 0x00000000);

	/* 3. Check the firmware status (idle) */
	/* Wait for 1s at most */
	for (i = 0; i < 500; i++) {
		/* OIS_STS */
		ois_r_reg16_val32_rvt(ois, OIS_STATUS_ADDR, &read_data);
		if (read_data == 0x00000001)
			break;
		msleep(2);
	}
	if (i >= 500) {
		CAM_ERR(CAM_OIS, "rumbas10 is not at idle status");
		return -ETIME;
	}

	/* 4. Veritfy checksum */
	rc = rumbas10_checksum_veritfy(ois);
	if (rc) {
		CAM_ERR(CAM_OIS, "the firmware checksum is error, result = %d", rc);
		return rc;
	} else {
		CAM_INFO(CAM_OIS, "the firmware checksum is right");
	}

	/* Check version */
	ois_r_reg16_val16_rvt(ois, FW_VERSION_ADDR, &fw_current_version);
	if (fw_current_version != priv->desc.version) {
		CAM_ERR(CAM_OIS, "fw download error, fw_version = 0x%x, current_version",
			priv->desc.version, fw_current_version);
		return -EIO;
	}

	return rc;
}

static int32_t rumbas10_fw_download_internel(struct vendor_ois_ctrl *ois,
	const struct firmware *fw)
{
	uint16_t fw_current_version = 0;
	int32_t  result;
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	ois_r_reg16_val16_rvt(ois, FW_VERSION_ADDR,
		&fw_current_version);
	CAM_INFO(CAM_OIS, "rumbas10 fw_version = 0x%x", fw_current_version);

	if (fw_current_version >= priv->desc.version) {
		CAM_INFO(CAM_OIS, "no need to updata fw");
		return 0;
	}

	result = rumbas10_flash_download(ois, fw);
	if (result != 0)
		CAM_ERR(CAM_OIS, "fw download failed, result = %d", result);
	else
		CAM_INFO(CAM_OIS, "fw download success");

	return result;
}

static uint16_t rumbas10_calculate_checksum(const struct firmware *fw)
{
	uint16_t checksum = 0;
	uint32_t loop;
	uint32_t i;
	const uint16_t *firmware_data = (uint16_t *)fw->data;

	loop = RUMBAS10_FW_SIZE >> 1;
	for (i = 0; i < loop; i++)
		checksum += firmware_data[i];

	return checksum;
}

static void rumbas10_fw_analysis(struct vendor_ois_ctrl *ois,
	const struct firmware *fw)
{
	const uint16_t *firmware_data = (uint16_t *)fw->data;
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	priv->desc.version  = firmware_data[FW_VERSION_INDEX];
	priv->desc.checksum = rumbas10_calculate_checksum(fw);
	priv->desc.fw_size  = RUMBAS10_FW_SIZE;

	CAM_INFO(CAM_OIS, "version = 0x%x, checksum = 0x%x, fw_size = 0x%x",
		priv->desc.version, priv->desc.checksum, priv->desc.fw_size);
}

static int32_t rumbas10_fw_download(struct vendor_ois_ctrl *ois)
{
	int32_t rc;
	const struct firmware *fw = NULL;

	if (!ois || !ois->o_ctrl) {
		CAM_ERR(CAM_OIS, "handle is NULL");
		return -EINVAL;
	}

	fw = vendor_request_firmware(ois);
	if (!fw)
		return -EINVAL;

	rumbas10_fw_analysis(ois, fw);
	rc = rumbas10_fw_download_internel(ois, fw);
	if (rc)
		CAM_ERR(CAM_OIS, "download_ois_fw fail");
	release_firmware(fw);

	return rc;
}

static const struct vendor_ois_ops rumbas10_ops = {
	.fw_download = rumbas10_fw_download,
};

static struct vendor_ois_ctrl rumbas10_driver = {
	.name = "rumbas10",
	.ops = &rumbas10_ops,
};

int32_t rumbas10_driver_init(void)
{
	int32_t rc = 0;
	struct ois_ctrl_priv *priv = NULL;

	priv = (struct ois_ctrl_priv *)kzalloc(
		sizeof(struct ois_ctrl_priv), GFP_KERNEL);
	if (!priv) {
		CAM_ERR(CAM_OIS, "rumbas10_download_ois_fw fail : %d", rc);
		return -EINVAL;
	}
	vendor_ois_set_drvdata(&rumbas10_driver, priv);
	rc = vendor_ois_register(&rumbas10_driver);
	if (rc)
		CAM_ERR(CAM_OIS, "vendor register ois fail : %d", rc);

	return rc;
}

void rumbas10_driver_exit(void)
{
	struct ois_ctrl_priv *priv = NULL;

	priv = (struct ois_ctrl_priv *)vendor_ois_get_drvdata(&rumbas10_driver);
	if (priv)
		kfree(priv);
	vendor_ois_unregister(&rumbas10_driver);
}

MODULE_DESCRIPTION("rumbas10 OIS driver");
MODULE_LICENSE("GPL v2");
