/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor dw9787 OIS driver
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
#include "ois_dw9787.h"

struct mem_desc_t {
	uint32_t type;
	uint16_t addr;
	uint32_t num_pages;
	uint32_t section_size;
	uint16_t section_value;
	uint32_t pkt_size;
	uint32_t mem_size;
};

struct fw_desc_t {
	uint32_t fw_size;
	uint32_t mcs_checksum;
	uint32_t if_checksum;
	uint16_t ver_major;
	uint16_t ver_minor;
	uint16_t ver_patch;
	uint16_t chip_id;
};

struct ois_ctrl_priv {
	bool already_download;
	bool force_download;
	struct fw_desc_t desc;
};

static struct mem_desc_t g_mem_desc[] = {
	{ CODE_SECTION,  MCS_START_ADDRESS, 16, 0x800,
	  MCS_VALUE, MCS_PKT_SIZE, 0x8000 },
	{ DATA_SECTION,  IF_START_ADDRESS,  4,  0x200,
	  IF_VALUE,  IF_PKT_SIZE,  0x0800 }
};

static void dw9787_flash_access(struct vendor_ois_ctrl *ois, uint16_t type)
{
	/* chip enable */
	ois_w_reg16_val16(ois, 0x0020, 0x0001);
	/* stanby mode(MCU off) */
	ois_w_reg16_val16(ois, 0x0024, 0x0000);
	/* code protection */
	ois_w_reg16_val16(ois, 0x0220, 0xC0D4);
	/* select program flash */
	ois_w_reg16_val16(ois, 0x3000, type);
	mdelay(1); /* flash_access delay 1ms */
}

static uint32_t dw9787_checksum_read(struct vendor_ois_ctrl *ois,
	uint16_t type)
{
	uint16_t addr, size;
	uint16_t csh = 0;
	uint16_t csl = 0;
	uint32_t checksum;

	dw9787_flash_access(ois, type);
	/* Set the checksum area */
	if (type == CODE_SECTION) {
		addr = MCS_START_ADDRESS;
		size = 0x2000;
		CAM_INFO(CAM_OIS, "MCS Select");
	} else if (type == DATA_SECTION) {
		addr = IF_START_ADDRESS;
		size = 0x0200;
		CAM_INFO(CAM_OIS, "IF Select");
	} else {
		CAM_ERR(CAM_OIS, "not MCS or IF");
		return 0;
	}
	ois_w_reg16_val16(ois, 0x3048, addr); /* write addr */
	ois_w_reg16_val16(ois, 0x304C, size); /* 32bit(4byte) * 8192 */
	/* write cmd read checksum */
	ois_w_reg16_val16(ois, 0x3050, 0x0001);
	msleep(10); /* checksum_read delay 10ms, error when delay 1ms */
	ois_r_reg16_val16(ois, 0x3054, &csh); /* read csh */
	ois_r_reg16_val16(ois, 0x3058, &csl); /* read csl */
	checksum = ((uint32_t)(csh << 16)) | csl;
	CAM_INFO(CAM_OIS, "checksum calculated value: 0x%08x", checksum);
	/* code protection on */
	ois_w_reg16_val16(ois, 0x0220, 0x0000);

	return checksum;
}

static int32_t dw9787_internel_download(struct vendor_ois_ctrl *ois,
	 const struct firmware *fw, enum mem_type type)
{
	int32_t i;
	uint16_t addr;
	uint32_t checksum;
	uint32_t fw_checksum;
	uint16_t *ptr = NULL;
	struct mem_desc_t *desc = &g_mem_desc[type];
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	CAM_INFO(CAM_OIS, "fw size:%d, mem_size:%d", fw->size, desc->mem_size);
	if (type == CODE_SECTION) {
		fw_checksum = priv->desc.mcs_checksum;
		ptr = (uint16_t *)fw->data;
	} else {
		fw_checksum = priv->desc.if_checksum;
		ptr = (uint16_t *)fw->data + MCS_FW_OFFSET_W;
	}

	dw9787_flash_access(ois, desc->type);

	addr = desc->addr;
	/* erase MCS:32kBytes = 2K*16, IF: 2kBytes = 512B*4*/
	for (i = 0; i < desc->num_pages; i++) {
		/* erase address */
		ois_w_reg16_val16(ois, 0x3008, addr); /* Set sddress */
		/* erase sector MCS:2kbyte, IF:512byte */
		ois_w_reg16_val16(ois, 0x300C, desc->section_value);
		addr += desc->section_size; /* 2kbyte */
		msleep(5); /* Sector Erase delay 5ms */
	}
	/* Set mem write start Address */
	ois_w_reg16_val16(ois, 0x3028, desc->addr);

	/* flash fw write */
	for (i = 0; i < desc->mem_size / 2; i += desc->pkt_size)
		/* program sequential write 2K byte */
		ois_w_reg16_val16_blk(ois, 0x302C,
			(uint16_t *)(ptr + i), desc->pkt_size, 1);

	/* Set the checksum area */
	checksum = dw9787_checksum_read(ois, desc->type);
	if (fw_checksum != checksum) {
		CAM_ERR(CAM_OIS, "checksum fail, bin: 0x%08x, read: 0x%08x",
			fw_checksum, checksum);
		/* chip disable */
		ois_w_reg16_val16(ois, 0x0220, 0x0000);
		return -EIO;
	}

	return 0;
}

static int32_t dw9787_auto_read_check(struct vendor_ois_ctrl *ois)
{
	uint16_t autord_rv = 0;

	/* Check if flash data is normally auto read */
	ois_r_reg16_val16(ois, 0x305C, &autord_rv);
	if (autord_rv != 0) {
		CAM_ERR(CAM_OIS, "auto_read fail 0x%04X", autord_rv);
	} else {
		ois_w_reg16_val16(ois, 0x0018, 0x0001); /* Logic reset */
		msleep(4); /* reset reg 0x0018 delay 4ms */
		/* Idle mode(MCU ON) */
		ois_w_reg16_val16(ois, 0x0024, 0x0001);
		msleep(20); /* after MCU ON delay 20ms */
	}
	return autord_rv;
}

static int32_t dw9787_chipid_check(struct vendor_ois_ctrl *ois)
{
	uint16_t chip_id = 0;
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	ois_r_reg16_val16(ois, 0x0004, &chip_id); /* ID read */
	/* Check the chip_id of OIS ic */
	if (chip_id != priv->desc.chip_id) {
		CAM_ERR(CAM_OIS, "The module's OIS IC is not right 0x%04X", chip_id);
		return -ENODEV;
	}
	return 0;
}

static int32_t dw9787_mem_download(struct vendor_ois_ctrl *ois,
	const struct firmware *fw)
{
	uint16_t fw_ver_major = 0;
	uint16_t fw_ver_minor = 0;
	uint16_t fw_ver_patch = 0;
	uint32_t checksum;
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	/* check if loaded */
	if (priv->already_download) {
		CAM_INFO(CAM_OIS, "fw already downloaded");
		return 0;
	}

	if (dw9787_auto_read_check(ois))
		priv->force_download = true;

	/* Check the chip_id of OIS ic */
	if (dw9787_chipid_check(ois))
		return -ENODEV;

	/* Check the firmware version */
	ois_r_reg16_val16(ois, 0x9B08, &fw_ver_major); /* fw_ver_major */
	ois_r_reg16_val16(ois, 0x9B0A, &fw_ver_minor); /* fw_ver_minor */
	ois_r_reg16_val16(ois, 0x9B0C, &fw_ver_patch); /* fw_ver_patch */
	CAM_INFO(CAM_OIS, "fw version: %u.%u.%u",
		fw_ver_major, fw_ver_minor, fw_ver_patch);

	if ((fw_ver_major != priv->desc.ver_major) ||
		(fw_ver_minor != priv->desc.ver_minor) ||
		(fw_ver_patch != priv->desc.ver_patch) ||
		priv->force_download) {
		CAM_INFO(CAM_OIS, "another version is checked and download it");
		/* firmware download function */
		if (dw9787_internel_download(ois, fw, CODE_SECTION))
			return -EIO;
		if (dw9787_internel_download(ois, fw, DATA_SECTION))
			return -EIO;
	} else {
		CAM_INFO(CAM_OIS, "the ois firmware is lastest");
		checksum = dw9787_checksum_read(ois, CODE_SECTION);
		if (priv->desc.mcs_checksum != checksum) {
			CAM_ERR(CAM_OIS, "checksum error and download again");
			if (dw9787_internel_download(ois, fw, CODE_SECTION))
				return -EIO;
		}
		checksum = dw9787_checksum_read(ois, DATA_SECTION);
		if (priv->desc.if_checksum != checksum) {
			CAM_ERR(CAM_OIS, "checksum error and download again");
			if (dw9787_internel_download(ois, fw, DATA_SECTION))
				return -EIO;
		}
	}
	priv->already_download = true;
	CAM_INFO(CAM_OIS, "download_fw success");

	return 0;
}

static void dw9787_fw_analysis(struct vendor_ois_ctrl *ois,
	const struct firmware *fw)
{
	uint16_t *ptr = (uint16_t *)fw->data;
	uint32_t index = fw->size / 2 - 1;
	struct ois_ctrl_priv *priv =
		(struct ois_ctrl_priv *)vendor_ois_get_drvdata(ois);

	/* analysis info*/
	index -= 2; /* reserved 2 bytes */
	priv->desc.chip_id = ptr[index--];
	priv->desc.ver_patch = ptr[index--];
	priv->desc.ver_minor = ptr[index--];
	priv->desc.ver_major = ptr[index--];
	priv->desc.if_checksum = ptr[index] + (ptr[index - 1] << 16);
	index -= 2; /* 2bytes for mcs checksum*/
	priv->desc.mcs_checksum = ptr[index] + (ptr[index - 1] << 16);
	index -= 2; /* 2bytes for if checksum*/
	priv->desc.fw_size = ptr[index] + (ptr[index - 1] << 16);
	CAM_INFO(CAM_OIS, "checksum:mcs:%x,if:%x,fw_size:%x",
		priv->desc.mcs_checksum, priv->desc.if_checksum,
		priv->desc.fw_size);
	CAM_INFO(CAM_OIS, "chip_id:%x,ver:%d.%d.%d",
		priv->desc.chip_id, priv->desc.ver_major,
		priv->desc.ver_minor, priv->desc.ver_patch);
}

static int32_t dw9787_fw_download(struct vendor_ois_ctrl *ois)
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

	dw9787_fw_analysis(ois, fw);
	rc = dw9787_mem_download(ois, fw);
	if (rc) {
		CAM_ERR(CAM_OIS, "dw9787_mem_download fail : %d", rc);
		return rc;
	}
	release_firmware(fw);

	return rc;
}

static const struct vendor_ois_ops dw9787_ops = {
	.fw_download = dw9787_fw_download,
};

static struct vendor_ois_ctrl dw9787_driver = {
	.name = "dw9787",
	.ops = &dw9787_ops,
};

int32_t dw9787_driver_init(void)
{
	int32_t rc = 0;
	struct ois_ctrl_priv *priv = NULL;

	priv = (struct ois_ctrl_priv *)kzalloc(
		sizeof(struct ois_ctrl_priv), GFP_KERNEL);
	if (!priv) {
		CAM_ERR(CAM_OIS, "dw9787_download_ois_fw fail : %d", rc);
		return -EINVAL;
	}
	vendor_ois_set_drvdata(&dw9787_driver, priv);
	rc = vendor_ois_register(&dw9787_driver);
	if (rc)
		CAM_ERR(CAM_OIS, "vendor register ois fail : %d", rc);

	return rc;
}

void dw9787_driver_exit(void)
{
	struct ois_ctrl_priv *priv = NULL;

	priv = (struct ois_ctrl_priv *)vendor_ois_get_drvdata(&dw9787_driver);
	if (priv)
		kfree(priv);
	vendor_ois_unregister(&dw9787_driver);
}

MODULE_DESCRIPTION("dw9787 OIS driver");
MODULE_LICENSE("GPL v2");
