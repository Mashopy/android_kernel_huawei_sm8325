/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
 *
 * Description: vendor AW86006 OIS driver
 */
#ifndef _OIS_AW86006_H_
#define _OIS_AW86006_H_

#include <linux/cma.h>
#include <cam_ois_dev.h>

#define AW_FLASH_BASE_ADDR             0x01000000
#define AW_FLASH_APP_ADDR              0x01002000
#define AW_FLASH_MOVE_LENGTH           (AW_FLASH_APP_ADDR - AW_FLASH_BASE_ADDR)
#define AW_FLASH_FULL_SIZE             0x10000
#define AW_FLASH_FIRMWARE_SIZE         0xA000
#define AW_FLASH_TOP_ADDR              (AW_FLASH_BASE_ADDR + AW_FLASH_FULL_SIZE)
#define AW_ERROR_LOOP                  5
#define AW_FLASH_ERASE_LEN             512
#define AW_FLASH_WRITE_LEN             64
#define AW_FLASH_READ_LEN              64
#define AW_SHUTDOWN_I2C_ADDR           (0xC2 >> 1)
#define AW_WAKEUP_I2C_ADDR             (0xD2 >> 1)
#define AW_FW_INFO_LENGTH              64
#define AW_FW_SHIFT_IDENTIFY           (AW_FLASH_MOVE_LENGTH)
#define AW_FW_SHIFT_CHECKSUM           (AW_FW_SHIFT_IDENTIFY + 8)
#define AW_FW_SHIFT_APP_CHECKSUM       (AW_FW_SHIFT_CHECKSUM + 4)
#define AW_FW_SHIFT_CHECKSUM_ADDR      (AW_FW_SHIFT_APP_CHECKSUM)
#define AW_FW_SHIFT_APP_LENGTH         (AW_FW_SHIFT_APP_CHECKSUM + 4)
#define AW_FW_SHIFT_APP_VERSION        (AW_FW_SHIFT_APP_LENGTH + 4)
#define AW_FW_SHIFT_APP_ID             (AW_FW_SHIFT_APP_VERSION + 4)
#define AW_FW_SHIFT_MOVE_CHECKSUM      (AW_FW_SHIFT_APP_ID + 4)
#define AW_FW_SHIFT_MOVE_VERSION       (AW_FW_SHIFT_MOVE_CHECKSUM + 4)
#define AW_FW_SHIFT_MOVE_LENGTH        (AW_FW_SHIFT_MOVE_VERSION + 4)
#define AW_FW_SHIFT_UPDATE_FLAG        (AW_FW_SHIFT_MOVE_LENGTH + 8)
#define AW_ARRAY_SHIFT_UPDATE_FLAG     (AW_FW_SHIFT_UPDATE_FLAG - AW_FLASH_MOVE_LENGTH)
#define AW86006_CHIP_ID_ADDRESS        0x0000
#define AW86006_VERSION_ADDRESS        0x0002
#define AW86006_STANDBY_ADDRESS        0xFF11
#define AW86006_OFFSET_0BIT            0
#define AW86006_OFFSET_8BIT            8
#define AW86006_OFFSET_16BIT           16
#define AW86006_OFFSET_24BIT           24
#define AW86006_ISP_WRITE_LOOP         5
#define AW_RESET_DELAY                 100
#define AW_JUMP_DELAY                  50
#define AW_SHUTDOWN_DELAY              2000 /* us */
#define AW_ISP_WRITE_ACK_LOOP          60
#define AW_ISP_RESET_DELAY             9
#define AW_ISP_RESET_DELAY_MAX         16
#define AW_SOC_WRITE_ACK_LOOP          60
#define AW_STOP_CMD_LOOP               20
#define AW_SOC_ACK_ERROR_LOOP          2
#define AW_SOC_JUMP_BOOT_LOOP          6
#define AW_SOC_RESET_DELAY             2
#define AW_SOC_RESET_DELAY_MAX         6
#define AW_SOC_ADDRESS_NONE            0x00
#define OIS_ERROR                      (-1)
#define OIS_SUCCESS                    0
#define TIME_50US                      50
#define AW_FLASH_READ_LENGTH_16BIT     16
#define AW_FLASH_READ_LENGTH_32BIT     32
#define AW_FLASH_READ_LENGTH_64BIT     64
#define AW_IC_STANDBY                  1

/*************** SOC ***************/
enum soc_module {
	SOC_HANK = 0x01,
	SOC_SRAM = 0x02,
	SOC_FLASH = 0x03,
	SOC_END = 0x04,
};

enum soc_enum {
	SOC_VERSION = 0x01,
	SOC_CTL = 0x00,
	SOC_ACK = 0x01,
	SOC_ADDR = 0x84,
	SOC_READ_ADDR = 0x48,
	SOC_ERASE_STRUCT_LEN = 6,
	SOC_READ_STRUCT_LEN = 6,
	SOC_PROTOCAL_HEAD = 9,
	SOC_CONNECT_WRITE_LEN = 9,
	SOC_ERASE_WRITE_LEN = 15,
	SOC_READ_WRITE_LEN = 15,
	SOC_ERASE_BLOCK_DELAY = 8,
	SOC_WRITE_BLOCK_HEAD = 13,
	SOC_WRITE_BLOCK_DELAY = 1000, /* us */
	SOC_READ_BLOCK_DELAY = 2000, /* us */
	SOC_CONNECT_DELAY = 2000, /* us */
};

enum soc_flash_event {
	SOC_FLASH_WRITE = 0x01,
	SOC_FLASH_WRITE_ACK = 0x02,
	SOC_FLASH_READ = 0x11,
	SOC_FLASH_READ_ACK = 0x12,
	SOC_FLASH_ERASE_BLOCK = 0x21,
	SOC_FLASH_ERASE_BLOCK_ACK = 0x22,
	SOC_FLASH_ERASE_CHIP = 0x23,
	SOC_FLASH_ERASE_CHIP_ACK = 0x24,
};

enum soc_hank_event {
	SOC_HANK_CONNECT = 0x01,
	SOC_HANK_CONNECT_ACK = 0x02,
};

/*************** ISP ***************/
enum isp_event_status {
	ISP_EVENT_IDLE = 0x00,
	ISP_EVENT_OK = 0x01,
	ISP_EVENT_ERR = 0x02,
};

enum isp_enum {
	ISP_FLASH_JUMP_DELAY = 2000, /* us */
	ISP_FLASH_HANK_DELAY = 2000, /* us */
	ISP_ERASE_BLOCK_DELAY = 8,
	ISP_FLASH_WRITE_DELAY = 600, /* us */
	ISP_FLASH_WRITE_HEAD_LEN = 8,
	ISP_JUMP_ACK_LEN = 1,
	ISP_HANK_ACK_LEN = 5,
	ISP_ERASE_ACK_LEN = 1,
	ISP_WRITE_ACK_LEN = 1,
	ISP_VERSION_ACK_LEN = 5,
	ISP_READ_VERSION_DELAY = 500, /* us */
};

enum isp_vers_event {
	ISP_VERS_VERSION = 0x00,
	ISP_VERS_CONNECT_ACK = 0x01,
};

int32_t aw86006_driver_init(void);
void aw86006_driver_exit(void);
#endif
/* _OIS_AW86006_H_ */
