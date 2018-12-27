#ifndef __BSP_W25Q128_H__
#define __BSP_W25Q128_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "gpio.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
//#define  SPI_FLASH_ID                       0xEF3015     //W25X16   2MB
#define SPI_FLASH_ID 0xEF4015 //W25Q16   2MB
//#define  SPI_FLASH_ID                       0XEF4017     //W25Q64   8MB
//#define  SPI_FLASH_ID                       0XEF4018     //W25Q128  16MB YS-F1Pro����Ĭ��ʹ��

#define FLASH_SPI_CS_ENABLE() HAL_GPIO_WritePin(W25_CS_GPIO_Port, W25_CS_Pin, GPIO_PIN_RESET)
#define FLASH_SPI_CS_DISABLE() HAL_GPIO_WritePin(W25_CS_GPIO_Port, W25_CS_Pin, GPIO_PIN_SET)

/* ��չ���� ------------------------------------------------------------------*/
extern SPI_HandleTypeDef hspiflash;

/* �������� ------------------------------------------------------------------*/

void SPI_FLASH_SectorErase(uint32_t SectorAddr);
void SPI_FLASH_BulkErase(void);
void SPI_FLASH_PageWrite(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void SPI_FLASH_BufferWrite(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void SPI_FLASH_BufferRead(uint8_t *pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t SPI_FLASH_ReadID(void);
uint32_t SPI_FLASH_ReadDeviceID(void);
void SPI_FLASH_StartReadSequence(uint32_t ReadAddr);
void SPI_Flash_PowerDown(void);
void SPI_Flash_WAKEUP(void);

uint8_t SPI_FLASH_ReadByte(void);
uint8_t SPI_FLASH_SendByte(uint8_t byte);
void SPI_FLASH_WriteEnable(void);
void SPI_FLASH_WaitForWriteEnd(void);

#endif /* __BSP_SPIFLASH_H__ */

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
