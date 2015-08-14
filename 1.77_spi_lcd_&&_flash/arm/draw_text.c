#include "draw_text.h"


extern unsigned char ascii[];



////////////////////////////////////////////////////////////////////////////////////
//显示字符串或显示数字
////////////////////////////////////////////////////////////////////////////////////
//ascii 32~90(!~Z), (32~47)空格~/,(48~57)0~9,(58~64):~@,(65~126)A~~
//ord 0~95, (48~57)0~9,(65~126)A~z,(33~47)!~/,(58~64):~@
/*
转换函数
*/
unsigned char ToOrd(unsigned char ch)
{
    if(ch < 32)
    {
        ch = 95;
    }
    else if((ch >= 32) && (ch <= 47)) //(32~47)空格~/
    {
        ch = (ch - 32) + 10 + 62;
    }
    else if((ch >= 48) && (ch <= 57)) //(48~57)0~9
    {
        ch = ch - 48;
    }
    else if((ch >= 58) && (ch <= 64)) //(58~64):~@
    {
        ch = (ch - 58) + 10 + 62 + 16;
    }
    else if((ch >= 65) && (ch <= 126)) //(65~126)A~~
    {
        ch = (ch - 65) + 10;
    }
    else if(ch > 126)
    {
        ch = 95;
    }

    return ch;
}

/*
显示一个字符函数
*/
void  DispOneChar(unsigned char ord, unsigned int Xstart, unsigned int Ystart, unsigned int TextColor, unsigned int BackColor)	 // ord:0~95
{
    unsigned char i, j;
    unsigned char  *p;
    unsigned char dat;
    unsigned int index;

    BlockWrite(Xstart, Xstart + (FONT_W - 1), Ystart, Ystart + (FONT_H - 1));

    index = ord;

    if(index > 95)	 //95:ASCII CHAR NUM
        index = 95;

    index = index * ((FONT_W / 8) * FONT_H);

    p = ascii;
    p = p + index;

    for(i = 0; i < (FONT_W / 8 * FONT_H); i++)
    {
        dat = *p++;
        for(j = 0; j < 8; j++)
        {
            if((dat << j) & 0x80)
            {
                WriteOneDot(TextColor);
            }
            else
            {
                WriteOneDot(BackColor);
            }
        }
    }
}

/*
显示一个字符串函数
*/
void DispStr(unsigned char *str, unsigned int Xstart, unsigned int Ystart, unsigned int TextColor, unsigned int BackColor)
{

    while(!(*str == '\0'))
    {
        DispOneChar(ToOrd(*str++), Xstart, Ystart, TextColor, BackColor);

        if(Xstart > ((COL - 1) - FONT_W))
        {
            Xstart = 0;
            Ystart = Ystart + FONT_H;
        }
        else
        {
            Xstart = Xstart + FONT_W;
        }

        if(Ystart > ((ROW - 1) - FONT_H))
        {
            Ystart = 0;
        }
    }
    BlockWrite(0, COL - 1, 0, ROW - 1);
}

/*
显示一个number
*/
void DispInt(unsigned int i, unsigned int Xstart, unsigned int Ystart, unsigned int TextColor, unsigned int BackColor)
{
    if(Xstart > ((COL - 1) - FONT_W * 4))
    {
        Xstart = (COL - 1) - FONT_W * 4;
    }
    if(Ystart > ((ROW - 1) - FONT_H))
    {
        Ystart = (Ystart - 1) - FONT_H;
    }

    DispOneChar((i >> 12) % 16, Xstart, Ystart, TextColor, BackColor); //ID value
    DispOneChar((i >> 8) % 16, Xstart + FONT_W, Ystart, TextColor, BackColor);
    DispOneChar((i >> 4) % 16, Xstart + FONT_W * 2, Ystart, TextColor, BackColor);
    DispOneChar(i % 16, Xstart + FONT_W * 3, Ystart, TextColor, BackColor);

    BlockWrite(0, COL - 1, 0, ROW - 1);
}
////////////////////////////////////////////////////////////////////////////////////
