/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor dw9781 OIS driver
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
#include "ois_dw9781.h"
#include "vendor_cam_hiview.h"

struct fw_desc_t {
	uint32_t fw_size;
	uint32_t checksum;
	uint16_t version;
	uint16_t chip_id;
};

struct ois_ctrl_priv {
	bool already_download;
	struct fw_desc_t desc;
};

static int32_t dw9781_i2c_fail_handle(struct vendor_ois_ctrl *ois)
{
	int32_t ret;
	struct cam_ois_ctrl_t *o_ctrl = ois->o_ctrl;

	o_ctrl->io_master_info.cci_client->sid = DW9781_LOGIC_SLAVE_ID;
	ret = ois_w_reg16_val16(ois,
		DW9781_LOGIC_RESET_ADDRESS, DW9781_LOGIC_RESET_VAL);
	if (ret < 0)
		CAM_ERR(CAM_OIS, "dw9781 ois logic reset failed");

	o_ctrl->io_master_info.cci_client->sid = DW9781_SLAVE_ID;
	msleep(10); /* delay 10ms for dw9781 reset */

	return ret;
}

static int32_t dw9781_w_reg16_val16(struct vendor_ois_ctrl *ois,
	uint16_t reg, uint16_t value)
{
	int32_t rc = 0;
	struct cam_ois_ctrl_t *o_ctrl = ois->o_ctrl;

	rc = ois_w_reg16_val16(ois, reg, value);
	if (rc < 0 && o_ctrl->io_master_info.cci_client->sid == DW9781_SLAVE_ID) {
		CAM_ERR(CAM_OIS, "dw9781 i2c write:%x val:%x fail, set logic reset",
			reg, value);
		dw9781_i2c_fail_handle(ois);
		rc = ois_w_reg16_val16(ois, reg, value);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "dw9781 ois after logic reset write fail");
			vendor_cam_i2c_hiview_handle(&(o_ctrl->io_master_info),
				"dw9781 i2c write fail");
		}
	}

	return rc;
}

static int32_t dw9781_r_reg16_val16(struct vendor_ois_ctrl *ois,
	uint16_t reg, uint16_t *value)
{
	int32_t rc = 0;
	struct cam_ois_ctrl_t *o_ctrl = ois->o_ctrl;

	rc = ois_r_reg16_val16(ois, reg, value);
	if (rc < 0 && o_ctrl->io_master_info.cci_client->sid == DW9781_SLAVE_ID) {
		CAM_ERR(CAM_OIS, "dw9781 i2c read:%x fail, set logic reset", reg);
		dw9781_i2c_fail_handle(ois);
		rc = ois_r_reg16_val16(ois, reg, value);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "dw9781 ois after logic reset read fail");
			vendor_cam_i2c_hiview_handle(&(o_ctrl->io_master_info),
				"dw9781 i2c read fail");
		}
	}

	return rc;
}

static void dw9781_ois_reset(struct vendor_ois_ctrl *ois)
{
	/* Logic reset */
	dw9781_w_reg16_val16(ois, 0xD002, 0x0001);
	msleep(4); /* wait time */
	/* Active mode (DSP ON) */
	dw9781_w_reg16_val16(ois, 0xD001, 0x0001);
	msleep(25); /* ST gyro - over wait 25ms, default Servo On */
	/* User protection release */
	dw9781_w_reg16_val16(ois, 0xEBF1, 0x56FA);
	CAM_INFO(CAM_OIS, "ois reset finish");
}

static int32_t dw9781_ready_check(struct vendor_ois_ctrl *ois)
{
	uint16_t flag_t = 0;
	uint16_t flag_l = 0;

	dw9781_w_reg16_val16(ois, 0xd000, 0x0001); /* active mode */
	msleep(4); /* wait time */
	dw9781_w_reg16_val16(ois, 0xd001, 0x0000); /* dsp mode */
	dw9781_w_reg16_val16(ois, 0xFAFA, 0x98AC); /* All protection(1) */
	dw9781_w_reg16_val16(ois, 0xF053, 0x70BD); /* All protection(2) */
	dw9781_r_reg16_val16(ois, 0x9FF9, &flag_t); /* T PRJ checksum flag */
	dw9781_r_reg16_val16(ois, 0xA7F9, &flag_l); /* L PRJ checksum flag */

	if (flag_t == 0xCC33 || flag_l == 0xCC33) {
		dw9781_ois_reset(ois); /* ois reset */
		return 0;
	} else {
		dw9781_w_reg16_val16(ois, 0xD002, 0x0001); /* logic reset */
		msleep(4); /* reset delay time */
		CAM_ERR(CAM_OIS, "previous fw download fail");
		return -1;
	}
}

static void dw9781_set_download_mode(struct vendor_ois_ctrl *ois)
{
	/* release all protection */
	dw9781_w_reg16_val16(ois, 0xFAFA, 0x98AC);
	msleep(1);
	dw9781_w_reg16_val16(ois, 0xF053, 0x70BD);
	msleep(1);
	dw9781_w_reg16_val16(ois, 0xD041, 0x000E); /* set spi SSB1 */
	dw9781_w_reg16_val16(ois, 0xD043, 0x000E); /* set spi SCLK */
	dw9781_w_reg16_val16(ois, 0xD044, 0x000E); /* set spi SDAT */
	dw9781_w_reg16_val16(ois, 0xDD02, 0x01C8); /* set spi input mode */
	CAM_INFO(CAM_OIS, "set download mode");
}

static void dw9781_erase_mtp(struct vendor_ois_ctrl *ois)
{
	int32_t i;
	uint16_t sector[SECTOR_NUM] = {
		0x0000, 0x0008, 0x0010, 0x0018, 0x0020
	};

	/* Erase each 4k Sector */
	for (i = 0; i < SECTOR_NUM; i++) {
		dw9781_w_reg16_val16(ois, 0xDE03, sector[i]);
		/* 4k Sector Erase */
		dw9781_w_reg16_val16(ois, 0xDE04, 0x0002);
		msleep(10);
	}
	CAM_INFO(CAM_OIS, "erase mtp finish");
}

static void dw9781_erase_for_rewritefw(struct vendor_ois_ctrl *ois)
{
	/* last 512byte select */
	dw9781_w_reg16_val16(ois, 0xDE03, 0x0027);
	/* last 512byte Erase */
	dw9781_w_reg16_val16(ois, 0xDE04, 0x0008);
	msleep(10); /* wait time */
}

static int32_t dw9781_download_fw(struct vendor_ois_ctrl *ois,
	const struct firmware *fw)
{
	int32_t ret = 0;
	uint32_t i;
	uint16_t addr;
	uint16_t *buf_r = NULL;
	uint16_t *ptr = (uint16_t *)fw->data;
	uint32_t fw_size = fw->size / 2;
	uint16_t write_cnt = fw_size / DATPKT_SIZE;
	uint16_t write_index;

	buf_r = (uint16_t *)vmalloc(fw_size * sizeof(uint16_t) + 2);
	if (!buf_r) {
		CAM_ERR(CAM_OIS, "malloc failed");
		return -EIO;
	}
	memset(buf_r, 0, fw_size * sizeof(uint16_t) + 2);

	/* step 1: MTP setup */
	dw9781_set_download_mode(ois);
	CAM_INFO(CAM_OIS, "flash ready,fw_size:%x", fw->size);

	/* step 2: MTP Erase and DSP Disable for FW 0x8000 write */
	dw9781_w_reg16_val16(ois, 0xd001, 0x0000);
	dw9781_erase_mtp(ois);

	/* step 3: Seq Write All Flash */
	for (write_index = 0; write_index < write_cnt; write_index++) {
		addr = MTP_START_ADDRESS + write_index * DATPKT_SIZE;
		/* 0: MSM_CCI_I2C_WRITE_SEQ */
		ois_w_reg16_val16_blk(ois, addr,
			(uint16_t *)(ptr + write_index * DATPKT_SIZE), DATPKT_SIZE, 0);
	}
	CAM_INFO(CAM_OIS, "flash write complete");

	/* step 4: Firmware memory read */
	for (i = 0; i < fw_size; i = i + DATPKT_SIZE) {
		addr = MTP_START_ADDRESS + i;
		ois_r_reg_val_blk(ois, addr, (uint32_t *)(buf_r + i), DATPKT_SIZE * 2,
			CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	}
	CAM_INFO(CAM_OIS, "flash read complete");

	/* step 5: Verify */
	for (i = 0; i < fw_size; i++) {
		if (ptr[i] != buf_r[i]) {
			ret = -EIO;
			CAM_ERR(CAM_OIS, "fw verify error, addr:%4xh--fw:%4xh--read:%4xh",
				MTP_START_ADDRESS + i, ptr[i], buf_r[i]);
			break;
		}
	}

	if (ret != 0) {
		dw9781_erase_for_rewritefw(ois);
		/* Shut download mode */
		dw9781_w_reg16_val16(ois, 0xd000, 0x0000);
		CAM_INFO(CAM_OIS, "FW Download NG!!!");
		return ret;
	}

	CAM_INFO(CAM_OIS, "download_FW finished :%d", ret);
	vfree(buf_r);
	return ret;
}

static uint16_t dw9781_fw_checksum_verify(struct vendor_ois_ctrl *ois)
{
	uint16_t data = 0;
	/* FW checksum command */
	dw9781_w_reg16_val16(ois, 0x7011, 0x2000);
	/* command  start */
	dw9781_w_reg16_val16(ois, 0x7010, 0x8000);
	msleep(10);
	/* calc the checksum to write the 0x7005 */
	dw9781_r_reg16_val16(ois, 0x7005, &data);
	return data;
}

static uint16_t dw9781_fw_checksum(struct vendor_ois_ctrl *ois,
	const struct firmware *fw)
{
	uint16_t fw_checksum_current;
	uint16_t fw_checksum_result;
	uint32_t checksum_retry = 0;
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	while (checksum_retry < RETRY_TIME) {
		/* read fw checksum value from IC register: 0x7005 */
		fw_checksum_current = dw9781_fw_checksum_verify(ois);
		fw_checksum_result = (priv->desc.checksum ==
			fw_checksum_current) ? VERIFY_OK : VERIFY_ERROR;
		CAM_INFO(CAM_OIS, "fw_checksum:0x%x, checksum:0x%x, result:0x%x",
			priv->desc.checksum, fw_checksum_current, fw_checksum_result);
		dw9781_ois_reset(ois); /* must ois reset after fw checksum */
		if (fw_checksum_result == VERIFY_ERROR)
			checksum_retry++;
		else
			return VERIFY_OK;
	}

	return VERIFY_ERROR;
}

static int32_t dw9781_download_ois_fw(struct vendor_ois_ctrl *ois,
	const struct firmware *fw)
{
	int32_t ret;
	uint16_t fw_version_latest;
	uint16_t fw_version_current = 0;
	uint16_t chip_id = 0;
	uint16_t second_id = 0;
	uint16_t fw_checksum_result = VERIFY_OK;
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	/* 2. check if loaded */
	if (priv->already_download) {
		CAM_INFO(CAM_OIS, "fw already downloaded");
		dw9781_w_reg16_val16(ois, 0xd000, 0x0001); /* active mode */
		msleep(4); /* wait time */
		return 0;
	}
	dw9781_ready_check(ois); /* check dw9781 checksum flag */
	/* 6. ready to download */
	dw9781_r_reg16_val16(ois, DW9781_CHIP_ID_ADDRESS, &chip_id);
	CAM_INFO(CAM_OIS, "chipid_value = 0x%x", chip_id);

	/* 6.1 if error occured last time, chip id maybe tampered */
	if (chip_id != priv->desc.chip_id) {
		dw9781_set_download_mode(ois);
		/* second_info: 0x0020 */
		dw9781_r_reg16_val16(ois, 0xd060, &second_id);
		if (second_id == 0x0020) {
			/* Need to forced update OIS FW again. */
			ret = dw9781_download_fw(ois, fw);
			CAM_INFO(CAM_OIS, "finish flash download");
			if (ret) {
				/* Shut download mode */
				dw9781_w_reg16_val16(ois, 0xd000, 0x0000);
				CAM_ERR(CAM_OIS, "select download error, ret = 0x%x", ret);
				return -EIO;
			}
		} else {
			/* Shut download mode */
			dw9781_w_reg16_val16(ois, 0xd000, 0x0000);
			CAM_ERR(CAM_OIS, "second info check fail");
			return -EIO;
		}
	} else {
		dw9781_r_reg16_val16(ois, 0x7001, &fw_version_current);
		fw_version_latest = priv->desc.version;

		CAM_INFO(CAM_OIS, "fw version_current:0x%x, version_latest:0x%x",
			fw_version_current, fw_version_latest);

		if ((fw_version_current & 0xFF) == (fw_version_latest & 0xFF))
			fw_checksum_result = dw9781_fw_checksum(ois, fw);

		/* download fw, check if need update, download fw to flash */
		if (((fw_version_current & 0xFF) != (fw_version_latest & 0xFF)) ||
			fw_checksum_result == VERIFY_ERROR) {
			ret = dw9781_download_fw(ois, fw);
			if (ret) {
				/* Shut download mode */
				dw9781_w_reg16_val16(ois, 0xd000, 0x0000);
				CAM_ERR(CAM_OIS, "select download error, ret = 0x%x", ret);
				return -EIO;
			}
		} else {
			CAM_INFO(CAM_OIS, "ois fw version is updated, skip download");
		}
	}

	priv->already_download = true;
	return 0;
}

static void dw9781_fw_analysis(struct vendor_ois_ctrl *ois,
	const struct firmware *fw)
{
	uint16_t *ptr = (uint16_t *)fw->data;
	uint32_t size = fw->size / 2; /* size in 2bytes */
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	/* analysis info*/
	priv->desc.chip_id = ptr[size - DW9781_CHIPID_OFFSET];
	priv->desc.version = ptr[size - DW9781_VERSION_OFFSET];
	priv->desc.checksum = ptr[size - DW9781_CHECKSUM_OFFSET];
	priv->desc.fw_size = DW9781_FW_SIZE;

	CAM_INFO(CAM_OIS, "checksum:%x,fw_size:%x,size:%x",
		priv->desc.checksum, priv->desc.fw_size, size);
	CAM_INFO(CAM_OIS, "chip_id:%x,ver:%x",
		priv->desc.chip_id, priv->desc.checksum);
}

static int32_t dw9781_fw_download(struct vendor_ois_ctrl *ois)
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

	dw9781_fw_analysis(ois, fw);
	ois->o_ctrl->io_master_info.not_need_dmd_report = true;
	rc = dw9781_download_ois_fw(ois, fw);
	if (rc)
		CAM_ERR(CAM_OIS, "download_ois_fw fail");
	release_firmware(fw);
	return rc;
}

static const struct vendor_ois_ops dw9781_ops = {
	.fw_download = dw9781_fw_download,
};

static struct vendor_ois_ctrl dw9781_driver = {
	.name = "dw9781",
	.ops = &dw9781_ops,
};

int32_t dw9781_driver_init(void)
{
	int32_t rc = 0;
	struct ois_ctrl_priv *priv = NULL;

	priv = (struct ois_ctrl_priv *)kzalloc(
		sizeof(struct ois_ctrl_priv), GFP_KERNEL);
	if (!priv) {
		CAM_ERR(CAM_OIS, "dw9781_download_ois_fw fail : %d", rc);
		return -EINVAL;
	}
	vendor_ois_set_drvdata(&dw9781_driver, priv);
	rc = vendor_ois_register(&dw9781_driver);
	if (rc)
		CAM_ERR(CAM_OIS, "vendor register ois fail : %d", rc);

	return rc;
}

void dw9781_driver_exit(void)
{
	struct ois_ctrl_priv *priv = NULL;

	priv = (struct ois_ctrl_priv *)vendor_ois_get_drvdata(&dw9781_driver);
	if (priv)
		kfree(priv);
	vendor_ois_unregister(&dw9781_driver);
}

MODULE_DESCRIPTION("dw9781 OIS driver");
MODULE_LICENSE("GPL v2");
