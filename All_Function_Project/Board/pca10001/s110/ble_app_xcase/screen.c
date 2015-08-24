#include "screen.h"


//----------------------------------------------------------------------
static void GPIO_Init()
{
    nrf_gpio_cfg_output(CS);
    nrf_gpio_cfg_output(RS);
    nrf_gpio_cfg_output(RET);
    nrf_gpio_cfg_output(SCL);
    nrf_gpio_cfg_output(SDA);
    nrf_gpio_cfg_output(LCD_POWER_EN);
    nrf_gpio_cfg_output(LCD_BCK_LED_EN);
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
    GPIO_Init();//GPIO
    DELAY_MS(100);
	My_PWM_Init();
    LCD_Enable_POWER;

    DELAY_MS(100);
    RET_SET;
    DELAY_MS(100);
    RET_CLEAR;
    DELAY_MS(100);
    RET_SET;
    DELAY_MS(100);

    //-------------Start Initial Sequence--------//
	WriteComm(0x11);// Sleep out

    DELAY_MS(120);                //Delay 120ms

    WriteComm(0x36);//Memory data access control
    WriteData(0x00);

    WriteComm(0x3A);//Interface pixel format
    WriteData(0x05);

    WriteComm(0xB2);
    WriteData(0x0C);
    WriteData(0x0C);
    WriteData(0x00);
    WriteData(0x33);
    WriteData(0x33);

    WriteComm(0xB7);
    WriteData(0x35);

    WriteComm(0xBB);
    WriteData(0x1A);

    WriteComm(0xC0);
    WriteData(0x2C);

    WriteComm(0xC2);
    WriteData(0x01);

    WriteComm(0xC3);
    WriteData(0x0B);

    WriteComm(0xC4);
    WriteData(0x20);

    WriteComm(0xC6);
    WriteData(0x0F);

    WriteComm(0xD0);
    WriteData(0xA4);
    WriteData(0xA1);

    WriteComm(0x21);

    WriteComm(0xE0);
    WriteData(0x00);
    WriteData(0x19);
    WriteData(0x1E);
    WriteData(0x0A);
    WriteData(0x09);
    WriteData(0x15);
    WriteData(0x3D);
    WriteData(0x44);
    WriteData(0x51);
    WriteData(0x12);
    WriteData(0x03);
    WriteData(0x00);
    WriteData(0x3F);
    WriteData(0x3F);

    WriteComm(0xE1);
    WriteData(0x00);
    WriteData(0x18);
    WriteData(0x1E);
    WriteData(0x0A);
    WriteData(0x09);
    WriteData(0x25);
    WriteData(0x3F);
    WriteData(0x43);
    WriteData(0x52);
    WriteData(0x33);
    WriteData(0x03);
    WriteData(0x00);
    WriteData(0x3F);
    WriteData(0x3F);


    WriteComm(0x29);
	
	DispColor(BLUE);
	     
	WriteComm(0x11);
	LCD_BCKLIGHT_Enable_POWER;
}





/*
LCD块写（大量数据修改，相当于擦除）
*/
void BlockWrite(unsigned int Xstart, unsigned int Xend, unsigned int Ystart, unsigned int Yend)
{
    //ILI9163C
    WriteComm(0x2A);//0x2A indicates X label
    WriteData(Xstart >> 8);
    WriteData(Xstart);
    WriteData(Xend >> 8);
    WriteData(Xend);

    WriteComm(0x2B);//0x2A indicates Y label
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
绘制一片区域（名字为线，其实可以刷一个面）
*/
void DrawRectangle(unsigned int Xstart, unsigned int Xend, unsigned int Ystart, unsigned int Yend, unsigned int color)
{
    unsigned int i, j;
    BlockWrite(Xstart, Xend, Ystart, Yend);

    CS_CLEAR;
    RS_SET;
    for(i = Ystart; i < Yend + 1; i++)
    {
        for(j = Xstart; j < Xend + 1; j++)
        {
            WriteDispData(color>>8,color);
        }
    }
	CS_SET;
}


///////////////////////////////////////////////////
#define Rect_Dis	10   //矩形区域距离边的距离
#define Rect_Width	80	 //矩形区域宽
#define Rect_Length	20	 //矩形区域长

void DrawBand(uint16_t sensor_state)
{
	static 	uint16_t  pre_sensor_state=0;
	uint16_t xo=sensor_state^pre_sensor_state;

	if((xo & 0x01)==0x01){
		DrawRectangle(ROW-(Rect_Dis+Rect_Length),ROW-Rect_Dis,(COL-Rect_Width)/2,(COL+Rect_Width)/2,
		((sensor_state & 0x01)==0x01) ? RED : BLUE);//right	
	}
	if((xo & 0x02)==0x02){
		DrawRectangle((ROW-Rect_Width)/2,(ROW+Rect_Width)/2,COL-(Rect_Dis+Rect_Length),COL-Rect_Dis,
	   ((sensor_state & 0x02)==0x02) ? RED : BLUE);//down
	}
	if((xo & 0x04)==0x04){
		DrawRectangle(Rect_Dis,Rect_Dis+Rect_Length,(COL-Rect_Width)/2,(COL+Rect_Width)/2,
		((sensor_state & 0x04)==0x04) ? RED : BLUE);//left
	}
	if((xo & 0x08)==0x08){
	  	DrawRectangle((ROW-Rect_Width)/2,(ROW+Rect_Width)/2,Rect_Dis,Rect_Dis+Rect_Length,
		((sensor_state & 0x08)==0x08) ? RED : BLUE);//up
	}
	
	pre_sensor_state=sensor_state;	
}