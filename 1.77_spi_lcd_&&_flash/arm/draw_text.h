#ifndef _DRAW_TEXT_H
#define _DRAW_TEXT_H

#include "font.h"
#include "nrf_lcd.h"
void WriteOneDot(unsigned int color);//写一个点（带颜色）
void BlockWrite(unsigned int Xstart, unsigned int Xend, unsigned int Ystart, unsigned int Yend);//LCD块写（大量数据修改，相当于擦除）



void DispOneChar(unsigned char ord, unsigned int Xstart, unsigned int Ystart, unsigned int TextColor, unsigned int BackColor);// ord:0~95
void DispStr(unsigned char *str, unsigned int Xstart, unsigned int Ystart, unsigned int TextColor, unsigned int BackColor);//显示一个字符串函数
void DispInt(unsigned int i, unsigned int Xstart, unsigned int Ystart, unsigned int TextColor, unsigned int BackColor);//显示一个number



#endif
