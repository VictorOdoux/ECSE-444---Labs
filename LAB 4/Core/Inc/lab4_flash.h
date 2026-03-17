#ifndef __LAB4_FLASH_H
#define __LAB4_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "stm32l4s5i_iot01_qspi.h"

/* This wrapper keeps all Lab 4 flash access in one place.
 * Part 3 uses it for a simple erase/write/read self-test, and Part 4 can
 * reuse the same API for sensor logging instead of calling BSP_QSPI_*
 * functions directly from application code.
 *
 * The BSP is not designed around concurrent callers. If Part 4 needs flash
 * access from multiple RTOS tasks, keep one task in charge of logging or add
 * a mutex around calls into this module. */

typedef enum
{
  LAB4_FLASH_STATUS_OK = 0,
  LAB4_FLASH_STATUS_ERROR = -1,
  LAB4_FLASH_STATUS_RANGE_ERROR = -2,
  LAB4_FLASH_STATUS_NOT_INITIALIZED = -3,
  LAB4_FLASH_STATUS_VERIFY_ERROR = -4
} Lab4FlashStatus_t;

/* Block 0 is reserved for the Part 3 smoke test so Part 4 can start logging
 * at the next block without mixing validation data and sensor logs. */
#define LAB4_FLASH_TEST_BLOCK_ADDRESS  0x00000000UL
#define LAB4_FLASH_LOG_START_ADDRESS   MX25R6435F_BLOCK_SIZE
#define LAB4_FLASH_SELF_TEST_SIZE      64U

Lab4FlashStatus_t Lab4Flash_Init(void);
Lab4FlashStatus_t Lab4Flash_EraseBlock(uint32_t blockAddress);
Lab4FlashStatus_t Lab4Flash_Write(uint32_t address, const uint8_t *data, uint32_t size);
Lab4FlashStatus_t Lab4Flash_Read(uint32_t address, uint8_t *data, uint32_t size);
Lab4FlashStatus_t Lab4Flash_RunSelfTest(void);
const QSPI_Info *Lab4Flash_GetInfo(void);

#ifdef __cplusplus
}
#endif

#endif /* __LAB4_FLASH_H */
