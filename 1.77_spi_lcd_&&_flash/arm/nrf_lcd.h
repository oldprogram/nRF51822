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
#define PIC_WIDTH    160	 //预备向LCD显示区域填充的图片的大小
#define PIC_HEIGHT   160

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


void GPIO_Init(void);
void LCD_Init(void);
void Disp(unsigned short int width, unsigned short int height, unsigned short int *p);
void DispColor(unsigned int color);
void DispInt(unsigned int i, unsigned int Xstart, unsigned int Ystart, unsigned int TextColor, unsigned int BackColor);
void DispStr(unsigned char *str, unsigned int Xstart, unsigned int Ystart, unsigned int TextColor, unsigned int BackColor);
//void DrawLine(unsigned int Xstart, unsigned int Xend, unsigned int Ystart, unsigned int Yend, unsigned int color);
void DrawBand(unsigned int Xstart, unsigned int Xend, unsigned int Ystart, unsigned int Yend, unsigned int color);
void edge_dispband(uint16_t sensor_state);
#endif
