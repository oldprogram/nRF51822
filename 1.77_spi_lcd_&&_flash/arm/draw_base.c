#include "draw_base.h"




////////////////////////////////////////////////////////////////////////////////////
//绘制基本的图形点等函数
////////////////////////////////////////////////////////////////////////////////////
/*
绘制一片区域（名字为线，其实可以刷一个面）
*/
void DrawLine(unsigned int Xstart, unsigned int Xend, unsigned int Ystart, unsigned int Yend, unsigned int color)
{
    unsigned int i, j;

    BlockWrite(Xstart, Xend, Ystart, Yend);

    for(i = Ystart; i < Yend + 1; i++)
    {
        for(j = Xstart; j < Xend + 1; j++)
        {
            WriteOneDot(color);
        }
    }
}
