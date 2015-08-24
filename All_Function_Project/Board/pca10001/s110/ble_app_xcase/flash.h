
#ifndef __FLASH_H
#define __FLASH_H
#include "boards.h"

/******************************************************************************
** SPI_FLASH
********************************************************************************/
typedef enum
{
    RET_SUCCESS = 0,
    RET_FlashRead_ERROR,
    RET_WriteCmd_ERROR,
    RET_WaitBusy_ERROR
}SpiRetValueType;
/*****************************************************************************
**
*****************************************************************************/

#define    SPIFlash_Disable_POWER    nrf_gpio_pin_clear(FLASH_POWER_EN)   //片选有效
#define    SPIFlash_Enable_POWER   nrf_gpio_pin_set(FLASH_POWER_EN)     //片选无效
#define    SPIFlash_Enable_CS    nrf_gpio_pin_clear(SPI_FLASH_CS)   //片选有效
#define    SPIFlash_Disable_CS   nrf_gpio_pin_set(SPI_FLASH_CS)     //片选无效
#define    SPIFlash_Set_SCLK     nrf_gpio_pin_set(SPI_FLASH_CLK)    //时钟拉高
#define    SPIFlash_Clr_SCLK     nrf_gpio_pin_clear(SPI_FLASH_CLK)  //时钟置低
#define    SPIFlash_Set_DO       nrf_gpio_pin_set(SPI_FLASH_MOSI)   //MOSI拉高
#define    SPIFlash_Clr_DO       nrf_gpio_pin_clear(SPI_FLASH_MOSI) //MOSI置低
#define    SPIFlash_Get_DI       nrf_gpio_pin_read(SPI_FLASH_MISO)

#define    SPIFLASH_CMD_LENGTH        0x03
#define    SPIFLASH_WRITE_BUSYBIT     0X01
#define    SPIFlash_ReadData_CMD      0x03
#define    SPIFlash_WriteEnable_CMD   0x06
#define    SPIFlash_WriteDisable_CMD  0x04
#define    SPIFlash_PageProgram_CMD   0x02
#define    SPIFlash_WriteSR_CMD       0x01
#define    SPIFlash_ReadSR_CMD        0x05
#define    SPIFlash_SecErase_CMD      0x20
#define    SPIFlash_BlockErase_CMD    0xD8
#define    SPIFlash_PAGEBYTE_LENGTH   256
#define    SPIFlash_SECBYTE_LENGTH   (1024*4)
#define    FLASH_BLOCK_NUMBLE         7
#define    FLASH_PAGE_NUMBLE          8
#define    FLASH_WRITE_NUMBLE         3


/*******************************************************************************
**
*******************************************************************************/
uint8_t SpiFlash_ReadOneByte(void);
void SpiFlash_WriteOneByte(uint8_t DataBuffer);
uint8_t SpiFlash_Read(uint8_t *pBuffer,uint32_t ReadAddr,uint32_t ReadBytesNum);
uint8_t SpiFlash_Write_Page(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t WriteBytesNum);
uint8_t SpiFlash_Write_MorePage(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t WriteBytesNum);
uint8_t SpiFlash_Write_MoreSector(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t WriteBytesNum);
uint16_t SPIFlash_GPIO_GetBit(void);
void SpiFlash_Write_Data(char *pBuffer,uint8_t Block_Num,uint8_t Page_Num,uint32_t WriteBytesNum);
void SpiFlash_Read_Data(uint8_t *pBuffer,uint8_t Block_Num ,uint8_t Page_Num ,uint32_t ReadBytesNum);
void SPIFlash_Erase_Block(uint8_t BlockNum);
void Spi_Flash_Init(void);
void SpiFlash_Write_Sector_Data(char *pBuffer,uint8_t Block_Num,uint8_t Sector_Num,uint32_t WriteBytesNum);
void SpiFlash_Read_Sector_Data(char *pBuffer,uint8_t Block_Num ,uint8_t Sector_Num,uint32_t ReadBytesNum);
void SPIFlash_Erase_Sector(uint8_t Block_Num,uint8_t Sector_Number);

#if defined(FLASH_TEST)
void Spi_Flash_test(void);
#endif
#endif
