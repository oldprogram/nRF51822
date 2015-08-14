#ifndef _NRF_LCD_H
#define _NRF_LCD_H


#include "pca10001.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "font.h"

/*
引脚高低电平宏定义
*/
#define CS_SET		nrf_gpio_pin_set(CS)
#define CS_CLEAR 	nrf_gpio_pin_clear(CS)  
#define RS_SET		nrf_gpio_pin_set(RS)
#define RS_CLEAR 	nrf_gpio_pin_clear(RS) 
#define RET_SET		nrf_gpio_pin_set(RET)
#define RET_CLEAR 	nrf_gpio_pin_clear(RET)
#define SCL_SET		nrf_gpio_pin_set(SCL)
#define SCL_CLEAR 	nrf_gpio_pin_clear(SCL)   
#define SDA_SET		nrf_gpio_pin_set(SDA)
#define SDA_CLEAR 	nrf_gpio_pin_clear(SDA)
/*
宏定义等待函数
*/
#define DELAY_MS(n) nrf_delay_ms(n)



//------------------------------------------------------


#define ROW  160		    //显示的行、列数
#define COL  128


#define BLUE   0xF800		 //定义颜色常量 
#define GREEN  0x07E0
#define RED    0x001F
#define WHITE  0xFFFF
#define BLACK  0x0000
#define GRAY   0xEF5D	     //0x2410
#define GRAY75 0x39E7 
#define GRAY50 0x7BEF	
#define GRAY25 0xADB5


void GPIO_Init(void);//SPI的GPIO初始化
void LCD_Init(void);//LCD初始化
void WriteDispData(unsigned char DataH, unsigned char DataL);//写显示数据向屏幕（屏幕显示数据需要2字节）
void BlockWrite(unsigned int Xstart, unsigned int Xend, unsigned int Ystart, unsigned int Yend);//LCD块写（大量数据修改，相当于擦除，这句话之后调用SPI向屏幕写数据就从对应的矩形区域开始填充）
void DispColor(unsigned int color);//LCD显示颜色（颜色已在.h文件中定义）
void WriteOneDot(unsigned int color);//写一个点（带颜色）
void PutPixel(unsigned int x, unsigned int y, unsigned int color);//绘制一个像素点

#endif
