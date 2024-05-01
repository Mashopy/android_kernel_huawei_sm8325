/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: lc898129 OIS driver
 */
#ifndef _OIS_LC898129_FUN_H_
#define _OIS_LC898129_FUN_H_

#include <linux/cma.h>
#include <cam_ois_dev.h>


#define CMD_IO_ADR_ACCESS         0xC000 /* IO Write Access */
#define CMD_IO_DAT_ACCESS         0xD000 /* IO Read Access */
#define SHIFT_8BIT                0x8
#define SUCCESS                   0x00
#define FAILURE                   0x01
#define FW_VER_ADDR               0x8000
#define SYSDSP_REMAP              0xD000AC

#define I2C_TIME_MODE             1 /* 0 : disable, 1 : enable */

/* data area */
#define FROM_CHECKSUM_SIZE_01_04    0x00002731
#define FROM_CHECKSUM_01_04         0x31fa81e8

#define SYSDSP_CVER               0xD00100
#define USER_MAT                  0
#define INF_MAT0                  1
#define INF_MAT1                  2
#define INF_MAT2                  4
#define TRIM_MAT                  16

#if 1
#define MAKER_CODE                0x7777 /* ON Semi */
#else
#define MAKER_CODE                0xAAAA /* LVI */
#endif

#define PERICLKON                 0xD00000
#define SYSDSP_DSPDIV             0xD00014
#define SYSDSP_SOFTRES            0xD0006C
#define FRQTRM                    0xD00098
#define SYSDSP_REMAP              0xD000AC
#define OSCCNT                    0xD000D4
#define SYSDSP_CVER               0xD00100
#define OSCCKCNT                  0xD00108
#define ROMINFO                   0xE050D4

#define GAIN_XP                   0x0B /* TRM.12h[15:12]    INF2.22h[15:12] */
#define GAIN_XM                   0x0B /* TRM.12h[31:28]    INF2.22h[31:28] */
#define GAIN_YP                   0x0B /* TRM.13h[15:12]    INF2.23h[15:12] */
#define GAIN_YM                   0x0B /* TRM.13h[31:28]    INF2.23h[31:28] */
#define GAIN_AFP                  0x09 /* TRM.14h[15:12]    INF2.24h[15:12] */
#define GAIN_AFM                  0x09 /* TRM.14h[31:28]    INF2.24h[31:28] */
#define GAIN_VAL                  0x0B

#define OFST_XP                   0x40
#define OFST_XM                   0x40
#define OFST_YP                   0x40
#define OFST_YM                   0x40
#define OFST_AFP                  0x0100
#define OFST_AFM                  0x0100
#define OFST_VAL                  0x40

void hs_cnt_write(struct vendor_ois_ctrl *ois, uint8_t *data, int16_t len);
void addtional_unlock_code_set129(struct vendor_ois_ctrl *ois);
void hs_write_permission129(struct vendor_ois_ctrl *ois);
uint8_t unlock_code_set129(struct vendor_ois_ctrl *ois);
void hs_io_write32(struct vendor_ois_ctrl *ois,
	uint32_t io_adrs, uint32_t io_data);
void hs_io_read32(struct vendor_ois_ctrl *ois,
	uint32_t io_adrs, uint32_t *io_data);
uint8_t unlock_code_clear129(struct vendor_ois_ctrl *ois);
void hs_boot_mode(struct vendor_ois_ctrl *ois);
uint8_t hs_drv_off_adj(struct vendor_ois_ctrl *ois);

#endif
/* _OIS_LC898129_FUN_H_ */