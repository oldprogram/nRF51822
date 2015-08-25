/*****************************************************************************
** 文件名称：flash.c
** 功    能：
** 修改日志：
******************************************************************************/
//#include "includes.h"
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "simple_uart.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "flash.h"
#include "pic.h"
#include "nrf_delay.h"
#include "screen.h"
#if defined(FLASH_TEST)
uint8_t FLASH_Buf[10];
#endif


extern unsigned char gImage_A[];
extern unsigned char gImage_B[];
extern unsigned char gImage_C[];
/*****************************************************************************
** 文件名称：uint8_t SpiFlash_ReadOneByte(void)
** 功    能：
** 修改日志：
******************************************************************************/

uint8_t SpiFlash_ReadOneByte(void)
{
    uint8_t BitCount = 0;
    uint8_t retValue = 0;
    SPIFlash_Set_SCLK;            //时钟线拉高,恢复时钟线为高电平

    for(BitCount = 0; BitCount < 8; BitCount++)
    {
        retValue <<= 1;
        SPIFlash_Set_SCLK;            //时钟线拉高,恢复时钟线为高电平
        if(SPIFlash_Get_DI)
        {
            retValue |= 0x01;
        }
        else
        {
            retValue &= 0xFE;
        }
        SPIFlash_Clr_SCLK;            //时钟线拉低,产生下降沿读出数据
    }
    SPIFlash_Set_SCLK;
    return (retValue);
}

/*****************************************************************************
** 文件名称：void SpiFlash_WriteOneByte(uint8_t DataBuffer)
** 功    能：
** 修改日志：
******************************************************************************/
void SpiFlash_WriteOneByte(uint8_t DataBuffer)
{
    uint8_t BitCount = 0;
    SPIFlash_Clr_SCLK;          //时钟线拉低，恢复时钟线为低电平
    for(BitCount = 0; BitCount < 8; BitCount++)
    {
        SPIFlash_Clr_SCLK;          //时钟线拉低，恢复时钟线为低电平
        if(DataBuffer & 0x80)
        {
            SPIFlash_Set_DO;
        }
        else
        {
            SPIFlash_Clr_DO;
        }
        DataBuffer <<= 1;
        SPIFlash_Set_SCLK;            //时钟线拉高，产生上升沿写入数据
    }
    SPIFlash_Clr_SCLK;
    SPIFlash_Set_DO;                //一字节数据传送完毕，MOSI数据线置高表示空闲状态
}

/*****************************************************************************
** 文件名称：static uint8_t SpiFlash_Write_CMD(uint8_t *CMD)
** 功    能：
** 修改日志：
******************************************************************************/
static uint8_t SpiFlash_Write_CMD(uint8_t *CMD)
{
    uint32_t i = 0;
    for(i = 0; i < SPIFLASH_CMD_LENGTH; i++)
    {
        SpiFlash_WriteOneByte(CMD[i]);  //打开SD卡的片选信号并写入命令数据
    }

    return RET_SUCCESS;
}

/*****************************************************************************
** 文件名称：static uint8_t SpiFlash_ReadSR(void)
** 功    能：
** 修改日志：
******************************************************************************/
static uint8_t SpiFlash_ReadSR(void)
{
    uint8_t retValue = 0;
    SPIFlash_Enable_CS;
    SpiFlash_WriteOneByte(SPIFlash_ReadSR_CMD);
    retValue = SpiFlash_ReadOneByte();
    SPIFlash_Disable_CS;
    return retValue;
}

/*****************************************************************************
** 文件名称：static uint8_t SpiFlash_Wait_Busy(void)
** 功    能：
** 修改日志：
******************************************************************************/
static uint8_t SpiFlash_Wait_Busy(void)
{
    //sDelayT.cDelay_Count=0;
    //  while((SpiFlash_ReadSR()&SPIFLASH_WRITE_BUSYBIT) == 0X01)
    //  {
    //       if(sDelayT.cDelay_Count==100)
    //         break;
    // }
    nrf_delay_ms(1);
    return RET_SUCCESS;
}

/*****************************************************************************
** 文件名称：uint8_t SpiFlash_Read(uint8_t *pBuffer,uint32_t ReadAddr,uint32_t ReadBytesNum)
** 功    能：从ReadAddr开始读ReadBytesNum个数据放到pBuffer中
** 修改日志：
******************************************************************************/
uint8_t SpiFlash_Read(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t ReadBytesNum)
{
    uint32_t i = 0;
    uint8_t pcmd[SPIFLASH_CMD_LENGTH] = {0};
    uint8_t ret = 0;

    SPIFlash_Enable_CS;              //打开spiflash片选

    SpiFlash_WriteOneByte(SPIFlash_ReadData_CMD);

    pcmd[0] = (uint8_t)((ReadAddr & 0x00ff0000) >> 16);
    pcmd[1] = (uint8_t)((ReadAddr & 0x0000ff00) >> 8);
    pcmd[2] = (uint8_t)ReadAddr;
    ret = SpiFlash_Write_CMD(pcmd);
    if(ret != RET_SUCCESS)
    {
        return RET_WriteCmd_ERROR;
    }

    for(i = 0; i < ReadBytesNum; i++)
    {
        pBuffer[i] = SpiFlash_ReadOneByte();  //读取SPIflash中指定bytes字节数据

    }
    SPIFlash_Disable_CS;
    return RET_SUCCESS;
}

/*****************************************************************************
** 文件名称：static void SpiFlash_Write_Enable(void)
** 功    能：
** 修改日志：
******************************************************************************/
static void SpiFlash_Write_Enable(void)
{
    SPIFlash_Enable_CS;
    SpiFlash_WriteOneByte(SPIFlash_WriteEnable_CMD);
    SPIFlash_Disable_CS;
}

/*****************************************************************************
** 文件名称： void SPIFlash_Erase_Sector(uint8_t Block_Num,uint8_t Sector_Number)
** 功    能：
** 修改日志：
******************************************************************************/

void SPIFlash_Erase_Sector(uint8_t Block_Num, uint8_t Sector_Number)
{
    uint8_t pcmd[3] = {0};
    SpiFlash_Write_Enable();
    SPIFlash_Enable_CS;
    SpiFlash_WriteOneByte(SPIFlash_SecErase_CMD);

    pcmd[0] = Block_Num;
    pcmd[1] = Sector_Number << 4;
    pcmd[2] = 0;
    SpiFlash_Write_CMD(pcmd);
    SPIFlash_Disable_CS;
    SpiFlash_Wait_Busy();    //每次擦除数据都要延时等待写入结束

    return ;
}


/*****************************************************************************
** 文件名称：uint8_t SpiFlash_Write_Page(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t WriteBytesNum)
** 功    能：
** 修改日志：
******************************************************************************/
uint8_t SpiFlash_Write_Page(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t WriteBytesNum)
{
    uint32_t i = 0;
    uint8_t pcmd[SPIFLASH_CMD_LENGTH] = {0};
    uint8_t ret = 0;

    SpiFlash_Write_Enable();
    SPIFlash_Enable_CS;
    SpiFlash_WriteOneByte(SPIFlash_PageProgram_CMD);
    pcmd[0] = (uint8_t)((WriteAddr & 0x00ff0000) >> 16);
    pcmd[1] = (uint8_t)((WriteAddr & 0x0000ff00) >> 8);
    pcmd[2] = (uint8_t)WriteAddr;
    ret = SpiFlash_Write_CMD(pcmd);

    if(ret != RET_SUCCESS)
    {
        return RET_WriteCmd_ERROR;
    }

    for(i = 0; i < WriteBytesNum; i++)
    {

        SpiFlash_WriteOneByte(pBuffer[i]);  //向SPIflash中写入最大一页256bytes字节数据
    }
    SPIFlash_Disable_CS;
    ret = SpiFlash_Wait_Busy();                   //每次写入一定数量的数据都要延时等待写入结束
    if(ret != RET_SUCCESS)
        return RET_WaitBusy_ERROR;

    return RET_SUCCESS;
}


/*****************************************************************************
** 文件名称：uint8_t SpiFlash_Write_MorePage(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t WriteBytesNum)
** 功    能：
** 修改日志：
******************************************************************************/
uint8_t SpiFlash_Write_MorePage(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t WriteBytesNum)
{
    uint16_t PageByteRemain = 0;
    PageByteRemain = SPIFlash_PAGEBYTE_LENGTH - WriteAddr % SPIFlash_PAGEBYTE_LENGTH;//计算零头（按页256）
    if(WriteBytesNum <= PageByteRemain)//如果写的数据量小于零头就直接写入
    {
        PageByteRemain = WriteBytesNum;
    }
    while(1)
    {
        SpiFlash_Write_Page(pBuffer, WriteAddr, PageByteRemain);//先把零头写进去
        if(WriteBytesNum == PageByteRemain)//如果总数据没零头多就直接退出
        {
            break;
        }
        else//否则按页将剩下的写完
        {
            pBuffer += PageByteRemain;
            WriteAddr += PageByteRemain;
            WriteBytesNum -= PageByteRemain;
            if(WriteBytesNum > SPIFlash_PAGEBYTE_LENGTH)
            {
                PageByteRemain = SPIFlash_PAGEBYTE_LENGTH;
            }
            else
            {
                PageByteRemain = WriteBytesNum;
            }
        }
    }
    return RET_SUCCESS;
}

//
///*****************************************************************************
//** 文件名称：uint8_t SpiFlash_Write_MoreSector(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t WriteBytesNum)
//** 功    能：
//** 修改日志：
//******************************************************************************/
//uint8_t SpiFlash_Write_MoreSector(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t WriteBytesNum)
//{
//    uint32_t Secpos = 0;
//    uint16_t Secoff = 0;
//    uint16_t Secremain = 0;
//    uint32_t i = 0;
//    uint8_t  SPIBufferTemp[SPIFlash_SECBYTE_LENGTH] = {0};
//    Secpos = WriteAddr/SPIFlash_SECBYTE_LENGTH;
//    Secoff = WriteAddr%SPIFlash_SECBYTE_LENGTH;
//    Secremain = SPIFlash_SECBYTE_LENGTH - Secoff;
//    if(WriteBytesNum <= Secremain)
//    {
//        Secremain = WriteBytesNum;
//    }
//
//    while(1)
//    {
//        if(SpiFlash_Read(SPIBufferTemp,Secpos*SPIFlash_SECBYTE_LENGTH,SPIFlash_SECBYTE_LENGTH) != RET_SUCCESS)
//        {
//             return RET_FlashRead_ERROR;
//        }
//        for(i = 0;i < Secremain;i++)
//        {
//            if(SPIBufferTemp[Secoff+i] != 0xFF)
//                break;                  //需要擦除一个扇区
//        }
//        if(i < Secremain)               //需要擦除一个扇区
//        {
//            SPIFlash_Erase_Sector(Secpos);      //擦除这个扇区
//            for(i = 0;i < Secremain;i++)
//            {
//                SPIBufferTemp[i+Secoff] = pBuffer[i];
//            }
//            SpiFlash_Write_MorePage(SPIBufferTemp,Secpos*SPIFlash_SECBYTE_LENGTH,SPIFlash_SECBYTE_LENGTH);
//        }
//        else
//        {
//            SpiFlash_Write_MorePage(pBuffer,WriteAddr,Secremain);
//        }
//
//        if(WriteBytesNum == Secremain)
//        {
//            break;
//        }
//        else
//        {
//            Secpos++;
//            Secoff = 0;
//            pBuffer += Secremain;
//            WriteAddr += Secremain;
//            WriteBytesNum -=Secremain;
//            if(WriteBytesNum > SPIFlash_SECBYTE_LENGTH)
//            {
//                Secremain = SPIFlash_SECBYTE_LENGTH;
//            }
//            else
//            {
//                Secremain = WriteBytesNum;
//            }
//        }
//    }
//    return RET_SUCCESS;
//}
//

/*****************************************************************************
** 文件名称：uint16_t SPIFlash_GPIO_GetBit(void)
** 功    能：
** 修改日志：
******************************************************************************/
uint16_t SPIFlash_GPIO_GetBit(void)
{
    uint16_t retValue = 0;
    uint16_t valueTemp = 0;
    // valueTemp = P8IN;
    //  retValue = (valueTemp &= BIT1)>>0x01;            //BIT1即是0x02
    return retValue;
}


/*****************************************************************************
** 文件名称：void SPIFlash_Erase_Block(uint8_t BlockNum)
** 功    能：擦除BLOCK
** 修改日志：
******************************************************************************/
void SPIFlash_Erase_Block(uint8_t BlockNum)
{
    uint8_t pcmd[3] = {0};
    SpiFlash_Write_Enable();   //写使能
    SPIFlash_Enable_CS;        //片选拉低
    SpiFlash_WriteOneByte(SPIFlash_BlockErase_CMD);  //传输Block擦除指令
    pcmd[0] = BlockNum ;       //传24位地址 因为单片机为16位 所以分两次传
    SpiFlash_Write_CMD(pcmd);
    SPIFlash_Disable_CS;
    SpiFlash_Wait_Busy();                   //每次擦除数据都要延时等待写入结束
    return ;
}

/*****************************************************************************
** 文件名称：uint8_t SpiFlash_Write_Data(uint8_t *pBuffer,uint8_t Block_Num,int8_t Page_Num,uint32_t WriteBytesNum)
** 功    能：写哪个块中哪一页的数据
** 修改日志：
******************************************************************************/
void SpiFlash_Write_Data(char *pBuffer, uint8_t Block_Num, uint8_t Page_Num, uint32_t WriteBytesNum)
{
    uint8_t pcmd[3] = {0};
    SpiFlash_Write_Enable();
    SPIFlash_Enable_CS;
    SpiFlash_WriteOneByte(SPIFlash_PageProgram_CMD);
    pcmd[0] = Block_Num;
    pcmd[1] = Page_Num;
    pcmd[2] = 0;
    SpiFlash_Write_CMD(pcmd);
    for(uint32_t i = 0; i < WriteBytesNum; i++)
    {
        SpiFlash_WriteOneByte(pBuffer[i]);  //向SPIflash中写入最大一页256bytes字节数据
    }
    SPIFlash_Disable_CS;
    SpiFlash_Wait_Busy();                   //每次写入一定数量的数据都要延时等待写入结束
    return ;
}
/*****************************************************************************
** 文件名称：void SpiFlash_Write_Sector_Data(uint8_t *pBuffer,uint8_t Block_Num,int8_t Sector_Num,uint32_t WriteBytesNum)
** 功    能：写哪个块中哪个sector中的数据
** 修改日志：
******************************************************************************/
void SpiFlash_Write_Sector_Data(char *pBuffer, uint8_t Block_Num, uint8_t Sector_Num, uint32_t WriteBytesNum)
{
    uint8_t Write_Page_Num = 0;

    Write_Page_Num = WriteBytesNum / 256;
    if((WriteBytesNum % 256) != 0) Write_Page_Num += 1;
    for(uint8_t i = 0; i < Write_Page_Num; i++)
    {
        SpiFlash_Write_Data(pBuffer, Block_Num, i + (Sector_Num << 4), 256);
        pBuffer = pBuffer + 256;
    }
    return ;
}

/*****************************************************************************
** 文件名称：void SpiFlash_Read_Sector_Data(uint8_t *pBuffer,uint8_t Block_Num ,uint8_t Sector_Num ,uint32_t ReadBytesNum)
** 功    能：
** 修改日志：
******************************************************************************/
void SpiFlash_Read_Sector_Data(char *pBuffer, uint8_t Block_Num , uint8_t Sector_Num, uint32_t ReadBytesNum)
{
    uint32_t i = 0;
    uint8_t pcmd[SPIFLASH_CMD_LENGTH] = {0};

    SPIFlash_Enable_CS;              //打开spiflash片选
    SpiFlash_WriteOneByte(SPIFlash_ReadData_CMD);
    pcmd[0] = Block_Num ;
    pcmd[1] = Sector_Num << 4;
    pcmd[2] = 0;
    SpiFlash_Write_CMD(pcmd);

    for(i = 0; i < ReadBytesNum; i++)
    {
        pBuffer[i] = SpiFlash_ReadOneByte();  //读取SPIflash中指定bytes字节数据

    }
    SPIFlash_Disable_CS;

    return ;
}

/*****************************************************************************
** 文件名称：uint8_t SpiFlash_Read_Data(uint8_t *pBuffer,uint8_t Block_Num ,uint8_t Page_Num ,uint32_t ReadBytesNum)
** 功    能：
** 修改日志：
******************************************************************************/
void SpiFlash_Read_Data(uint8_t *pBuffer, uint8_t Block_Num , uint8_t Page_Num , uint32_t ReadBytesNum)
{
    uint8_t pcmd[SPIFLASH_CMD_LENGTH] = {0};

    SPIFlash_Enable_CS;              //打开spiflash片选
    SpiFlash_WriteOneByte(SPIFlash_ReadData_CMD);
    pcmd[0] = Block_Num ;
    pcmd[1] = Page_Num ;
    pcmd[2] = 0;
    SpiFlash_Write_CMD(pcmd);

    for(uint32_t i = 0 ; i < ReadBytesNum; i++)
    {
        pBuffer[i] = SpiFlash_ReadOneByte();  //读取SPIflash中指定bytes字节数据

    }
    SPIFlash_Disable_CS;
    return ;
}
/*******************************************************************************
** 函数名称：void Spi_Flash_init(void)
** 功    能：
** 修改日志：
*******************************************************************************/
void Spi_Flash_Init(void)
{
    nrf_gpio_cfg_output(FLASH_WP);
    nrf_gpio_cfg_output(FLASH_HOLD);
    nrf_gpio_cfg_output(FLASH_POWER_EN);

	nrf_gpio_pin_set(FLASH_WP);
	nrf_gpio_pin_set(FLASH_HOLD);
    SPIFlash_Enable_POWER;

    nrf_gpio_cfg_output(SPI_FLASH_CS);
    nrf_gpio_cfg_output(SPI_FLASH_CLK);
    nrf_gpio_cfg_output(SPI_FLASH_MOSI);
    nrf_gpio_cfg_input(SPI_FLASH_MISO, NRF_GPIO_PIN_NOPULL);
}
#if defined(FLASH_TEST)
void Spi_Flash_test(void)
{
//    SpiFlash_Write_Data("NRF", FLASH_BLOCK_NUMBLE, FLASH_PAGE_NUMBLE, FLASH_WRITE_NUMBLE);
//    nrf_delay_ms(100);
//    SpiFlash_Read_Data(FLASH_Buf, FLASH_BLOCK_NUMBLE , FLASH_PAGE_NUMBLE , FLASH_WRITE_NUMBLE);


	SpiFlash_Write_MorePage(gImage_A, 0, 40960);
	SpiFlash_Write_MorePage(gImage_B, 1<<16, 40960);
	SpiFlash_Write_MorePage(gImage_C, 2*(1<<16), 40960);


//    if((FLASH_Buf[0] == 'N') && (FLASH_Buf[1] == 'R') && (FLASH_Buf[2] == 'F'))
//    {
//        DispColor(RED);//blue
//    }
//    else
//    {
//        DispColor(GREEN);
//    }

	DispPicFromSD(0);
	DispPicFromSD(1);
	DispPicFromSD(2);
}
#endif





