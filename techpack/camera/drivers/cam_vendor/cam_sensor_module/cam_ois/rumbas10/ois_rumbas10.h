/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor rumbas10 OIS driver
 */
#ifndef _VENDOR_OIS_RUMBA_S10_H_
#define _VENDOR_OIS_RUMBA_S10_H_

#include <linux/cma.h>
#include <cam_ois_dev.h>

#define PROTECTION_ADDR    0x4000
#define FLASH_CMD_ADDR     0x4004
#define FLASH_ADDR         0x4008
#define FLASH_STATUS_ADDR  0x4050
#define OIS_ERROR_ADDR     0x500C
#define OIS_STATUS_ADDR    0x5010
#define FW_VERSION_ADDR    0x50BC
#define CHECKSUM_CTRL_ADDR 0x50C0
#define CHECKSUM_DATA_ADDR 0x50C4
#define FLASH_DATA_ADDR    0x7000

#define RUMBAS10_FW_SIZE        0xF000  /* 60 * 1024 bytes */
#define FW_VERSION_INDEX       ((RUMBAS10_FW_SIZE >> 1) - 4)
#define BLOCK_WRITE_SIZE        64
#define BLOCK_WRITE_LOOP_CONUT  960     /* 60 * 1024 / BLOCK_WRITE_SIZE */

int32_t rumbas10_driver_init(void);
void rumbas10_driver_exit(void);

#endif
/* _VENDOR_OIS_RUMBA_S10_H_ */
