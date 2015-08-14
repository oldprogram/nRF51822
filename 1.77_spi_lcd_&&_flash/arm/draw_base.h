#ifndef _DRAW_BASE_H
#define _DRAW_BASE_H

#include "nrf_lcd.h"
void WriteOneDot(unsigned int color);//写一个点（带颜色）
void BlockWrite(unsigned int Xstart, unsigned int Xend, unsigned int Ystart, unsigned int Yend);//LCD块写（大量数据修改，相当于擦除）






void DrawLine(unsigned int Xstart, unsigned int Xend, unsigned int Ystart, unsigned int Yend, unsigned int color);
#define DrawBand(a,b,c,d,e) DrawLine((a),(b),(c),(d),(e))




#endif
