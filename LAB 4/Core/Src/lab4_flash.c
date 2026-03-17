#include "lab4_flash.h"

#include <stddef.h>
#include <string.h>

/* This module keeps a small amount of local state after QSPI startup:
 * - s_flashInfo caches the geometry reported by the BSP driver
 * - s_flashReady prevents application code from touching flash before init */
static QSPI_Info s_flashInfo;
static uint8_t s_flashReady = 0U;

static Lab4FlashStatus_t Lab4Flash_CheckRange(uint32_t address, uint32_t size)
{
  /* All public read/write/erase helpers go through this guard first so the
   * rest of the module can assume the flash is initialized and the requested
   * address range stays inside the device. */
  if (s_flashReady == 0U)
  {
    return LAB4_FLASH_STATUS_NOT_INITIALIZED;
  }

  if ((size > 0U) && (address >= s_flashInfo.FlashSize))
  {
    return LAB4_FLASH_STATUS_RANGE_ERROR;
  }

  if (size > (s_flashInfo.FlashSize - address))
  {
    return LAB4_FLASH_STATUS_RANGE_ERROR;
  }

  return LAB4_FLASH_STATUS_OK;
}

Lab4FlashStatus_t Lab4Flash_Init(void)
{
  /* BSP_QSPI_Init() performs the board-specific sequence needed to bring the
   * external flash out of reset and configure the OCTOSPI peripheral. */
  if (BSP_QSPI_Init() != QSPI_OK)
  {
    return LAB4_FLASH_STATUS_ERROR;
  }

  /* Cache the flash geometry once so Part 4 code can query capacity, page
   * size, and erase size without hard-coding those values elsewhere. */
  if (BSP_QSPI_GetInfo(&s_flashInfo) != QSPI_OK)
  {
    return LAB4_FLASH_STATUS_ERROR;
  }

  s_flashReady = 1U;
  return LAB4_FLASH_STATUS_OK;
}

Lab4FlashStatus_t Lab4Flash_EraseBlock(uint32_t blockAddress)
{
  Lab4FlashStatus_t status;

  /* The block erase API only accepts block-aligned addresses. Keeping that
   * check here avoids repeating device-specific alignment rules in callers. */
  if ((blockAddress % MX25R6435F_BLOCK_SIZE) != 0U)
  {
    return LAB4_FLASH_STATUS_RANGE_ERROR;
  }

  status = Lab4Flash_CheckRange(blockAddress, MX25R6435F_BLOCK_SIZE);
  if (status != LAB4_FLASH_STATUS_OK)
  {
    return status;
  }

  /* BSP_QSPI_Erase_Block() is blocking, which keeps the Part 3 smoke test
   * simple and makes startup behaviour easier to reason about. */
  if (BSP_QSPI_Erase_Block(blockAddress) != QSPI_OK)
  {
    return LAB4_FLASH_STATUS_ERROR;
  }

  return LAB4_FLASH_STATUS_OK;
}

Lab4FlashStatus_t Lab4Flash_Write(uint32_t address, const uint8_t *data, uint32_t size)
{
  Lab4FlashStatus_t status;

  if ((data == NULL) && (size > 0U))
  {
    return LAB4_FLASH_STATUS_RANGE_ERROR;
  }

  status = Lab4Flash_CheckRange(address, size);
  if (status != LAB4_FLASH_STATUS_OK)
  {
    return status;
  }

  /* The BSP write API takes a mutable pointer, but it only reads from the
   * buffer. The cast keeps the const-correct application API on our side. */
  if (BSP_QSPI_Write((uint8_t *)data, address, size) != QSPI_OK)
  {
    return LAB4_FLASH_STATUS_ERROR;
  }

  return LAB4_FLASH_STATUS_OK;
}

Lab4FlashStatus_t Lab4Flash_Read(uint32_t address, uint8_t *data, uint32_t size)
{
  Lab4FlashStatus_t status;

  if ((data == NULL) && (size > 0U))
  {
    return LAB4_FLASH_STATUS_RANGE_ERROR;
  }

  status = Lab4Flash_CheckRange(address, size);
  if (status != LAB4_FLASH_STATUS_OK)
  {
    return status;
  }

  if (BSP_QSPI_Read(data, address, size) != QSPI_OK)
  {
    return LAB4_FLASH_STATUS_ERROR;
  }

  return LAB4_FLASH_STATUS_OK;
}

Lab4FlashStatus_t Lab4Flash_RunSelfTest(void)
{
  uint32_t i;
  uint8_t writeBuffer[LAB4_FLASH_SELF_TEST_SIZE];
  uint8_t readBuffer[LAB4_FLASH_SELF_TEST_SIZE];
  Lab4FlashStatus_t status;

  if (s_flashReady == 0U)
  {
    return LAB4_FLASH_STATUS_NOT_INITIALIZED;
  }

  /* The self-test uses a simple fixed pattern so the expected data is obvious
   * when debugging UART output or memory views in CubeIDE. */
  for (i = 0U; i < LAB4_FLASH_SELF_TEST_SIZE; ++i)
  {
    writeBuffer[i] = (uint8_t)(0xA0U + i);
  }

  /* Part 3 validates the full basic flash workflow:
   * 1. erase one block
   * 2. confirm erased bytes read back as 0xFF
   * 3. write a known buffer
   * 4. read it back and compare byte-for-byte */
  status = Lab4Flash_EraseBlock(LAB4_FLASH_TEST_BLOCK_ADDRESS);
  if (status != LAB4_FLASH_STATUS_OK)
  {
    return status;
  }

  memset(readBuffer, 0x00, sizeof(readBuffer));
  status = Lab4Flash_Read(LAB4_FLASH_TEST_BLOCK_ADDRESS, readBuffer, sizeof(readBuffer));
  if (status != LAB4_FLASH_STATUS_OK)
  {
    return status;
  }

  for (i = 0U; i < LAB4_FLASH_SELF_TEST_SIZE; ++i)
  {
    if (readBuffer[i] != 0xFFU)
    {
      return LAB4_FLASH_STATUS_VERIFY_ERROR;
    }
  }

  status = Lab4Flash_Write(LAB4_FLASH_TEST_BLOCK_ADDRESS, writeBuffer, sizeof(writeBuffer));
  if (status != LAB4_FLASH_STATUS_OK)
  {
    return status;
  }

  memset(readBuffer, 0x00, sizeof(readBuffer));
  status = Lab4Flash_Read(LAB4_FLASH_TEST_BLOCK_ADDRESS, readBuffer, sizeof(readBuffer));
  if (status != LAB4_FLASH_STATUS_OK)
  {
    return status;
  }

  if (memcmp(writeBuffer, readBuffer, sizeof(writeBuffer)) != 0)
  {
    return LAB4_FLASH_STATUS_VERIFY_ERROR;
  }

  return LAB4_FLASH_STATUS_OK;
}

const QSPI_Info *Lab4Flash_GetInfo(void)
{
  /* Returning NULL here makes it clear to callers that geometry is only valid
   * after Lab4Flash_Init() has completed successfully. */
  if (s_flashReady == 0U)
  {
    return NULL;
  }

  return &s_flashInfo;
}
