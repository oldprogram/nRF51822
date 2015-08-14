#include "nrf_lcd.h"
unsigned short int  PIC[ROW][COL];//the 2-array for save the picture


//----------------------------------------------------------------------
void GPIO_Init()
{
	nrf_gpio_cfg_output(CS);
    nrf_gpio_cfg_output(RS);
    nrf_gpio_cfg_output(RET);
    nrf_gpio_cfg_output(SCL);
    nrf_gpio_cfg_output(SDA);
}

////////////////////////////////////////////////////////////////////////////////////
//下面是SPI函数及实现LCD通信基础的写数据和写命令函数
//有了这两个函数就能实现复杂的LCD初始化和GUI了
////////////////////////////////////////////////////////////////////////////////////
/*
SPI Send Data
*/
void SendDataSPI(unsigned char dat)
{
    unsigned char i;
    for(i = 0; i < 8; i++)
    {
        if( (dat & 0x80) != 0 ) SDA_SET;
        else SDA_CLEAR;

        dat <<= 1;

        SCL_CLEAR;
        SCL_SET;
    }
}


/*
LCD WRITE COMMAND AND DATA
*/
void WriteComm(unsigned int i)
{
    CS_CLEAR;
    RS_CLEAR;
    SendDataSPI(i);
    CS_SET;
}
void WriteData(unsigned int i)
{
    CS_CLEAR;
    RS_SET;
    SendDataSPI(i);
    CS_SET;
}

/*
写显示数据向屏幕（屏幕显示数据需要2字节）
*/
void WriteDispData(unsigned char DataH, unsigned char DataL)
{
    SendDataSPI(DataH);
    SendDataSPI(DataL);
}
////////////////////////////////////////////////////////////////////////////////////

/*
LCD初始化函数
*/
void LCD_Init(void)
{
	DELAY_MS(100);
    RET_SET;
    DELAY_MS(100);
    RET_CLEAR;
    DELAY_MS(100);
    RET_SET;
    DELAY_MS(100);

    //-------------Start Initial Sequence--------//
    WriteComm(0x11); //Exit Sleep
    DELAY_MS(10);//20
    WriteComm(0x26); //Set Default Gamma
    WriteData(0x04);

    WriteComm(0xB1);//Set Frame Rate
    WriteData(0x0B);
    WriteData(0x14);

    WriteComm(0xC0); //Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD
    WriteData(0x0C);
    WriteData(0x05);

    WriteComm(0xC1); //Set BT[2:0] for AVDD & VCL & VGH & VGL
    WriteData(0x02);

    WriteComm(0xC5); //Set VMH[6:0] & VML[6:0] for VOMH & VCOML
    WriteData(0x3F);//44
    WriteData(0x48);

    WriteComm(0xC7);// Set VMF
    WriteData(0xC2);

    WriteComm(0x2A); //Set Column Address
    WriteData(0x00);
    WriteData(0x00);
    WriteData(0x00);
    WriteData(0x7F);
    WriteComm(0x2B); //Set Page Address
    WriteData(0x00);
    WriteData(0x00);
    WriteData(0x00);
    WriteData(0x9F);

    WriteComm(0x3A); //Set Color Format
    WriteData(0x55);
    WriteComm(0x36);
    WriteData(0xC8);

    WriteComm(0xF2); //Enable Gamma bit
    WriteData(0x01);
    WriteComm(0xE0);
    WriteData(0x3F);//p1
    WriteData(0x25);//p2
    WriteData(0x21);//p3
    WriteData(0x24);//p4
    WriteData(0x1D);//p5
    WriteData(0x0D);//p6
    WriteData(0x4C);//p7
    WriteData(0xB8);//p8
    WriteData(0x38);//p9
    WriteData(0x17);//p10
    WriteData(0x0F);//p11
    WriteData(0x08);//p12
    WriteData(0x04);//p13
    WriteData(0x02);//p14
    WriteData(0x00);//p15
    WriteComm(0xE1);
    WriteData(0x00);//p1
    WriteData(0x1A);//p2
    WriteData(0x1E);//p3
    WriteData(0x0B);//p4
    WriteData(0x12);//p5
    WriteData(0x12);//p6
    WriteData(0x33);//p7
    WriteData(0x47);//p8
    WriteData(0x47);//p9
    WriteData(0x08);//p10
    WriteData(0x20);//p11
    WriteData(0x27);//p12
    WriteData(0x3C);//p13
    WriteData(0x3D);//p14
    WriteData(0x3F);//p15


    WriteComm(0x36); //MX, MY, RGB mode
    WriteData(0xC0);//c8竖屏  68横屏

    WriteComm(0x29); // Display On

    WriteComm(0x2C);
}

/*
LCD块写（大量数据修改，相当于擦除）
*/
void BlockWrite(unsigned int Xstart, unsigned int Xend, unsigned int Ystart, unsigned int Yend)
{
    //ILI9163C
    WriteComm(0x2A);
    WriteData(Xstart >> 8);
    WriteData(Xstart);
    WriteData(Xend >> 8);
    WriteData(Xend);

    WriteComm(0x2B);
    WriteData(Ystart >> 8);
    WriteData(Ystart);
    WriteData(Yend >> 8);
    WriteData(Yend);

    WriteComm(0x2c);
}


/*
LCD显示颜色（颜色已在.h文件中定义）
*/
void DispColor(unsigned int color)
{
    unsigned int i, j;
    BlockWrite(0, COL - 1, 0, ROW - 1);

    CS_CLEAR;
    RS_SET;
    for(i = 0; i < ROW; i++)
    {
        for(j = 0; j < COL; j++)
        {
            SendDataSPI(color >> 8);
            SendDataSPI(color);
        }
    }
    CS_SET;
}

/*
写一个点（带颜色）
*/
void WriteOneDot(unsigned int color)
{
    CS_CLEAR;
    RS_SET;

    SendDataSPI(color >> 8);
    SendDataSPI(color);

    CS_SET;
}

/*
绘制一个像素点
*/
void PutPixel(unsigned int x, unsigned int y, unsigned int color)
{
    BlockWrite(x, x, y, y);
    CS_CLEAR;
    RS_SET;
    SendDataSPI(color >> 8);
    SendDataSPI(color);
    CS_SET;
}
