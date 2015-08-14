#ifndef _DRAW_PICTURE_H
#define _DRAW_PICTURE_H


#define EVAL_PIC

#include "nrf_lcd.h"
void WriteOneDot(unsigned int color);//写一个点（带颜色）
void BlockWrite(unsigned int Xstart, unsigned int Xend, unsigned int Ystart, unsigned int Yend);//LCD块写（大量数据修改，相当于擦除）
void WriteDispData(unsigned char DataH, unsigned char DataL);//写显示数据向屏幕（屏幕显示数据需要2字节）


#define PIC_WIDTH    160	 //预备向LCD显示区域填充的图片的大小
#define PIC_HEIGHT   160






#endif

