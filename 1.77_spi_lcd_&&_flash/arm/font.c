#include "font.h"


//宋体12,点阵为：宽x高=8x16
#ifdef CHAR_FONT_W8_H16
//ascii 32~90(!~Z), (32~47)空格~/,(48~57)0~9,(58~64):~@,(65~126)A~~
//ord 0~95, (48~57)0~9,(65~126)A~z,(33~47)!~/,(58~64):~@
unsigned char const ascii[] =
{
    //宋体12,点阵为：宽x高=8x16
    //0(ord:0)
    0x00, 0x00, 0x00, 0x18, 0x24, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x24, 0x18, 0x00, 0x00,

    //(ord:1)~1
    0x00, 0x00, 0x00, 0x10, 0x70, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x7C, 0x00, 0x00,

    //(ord:2)~2
    0x00, 0x00, 0x00, 0x3C, 0x42, 0x42, 0x42, 0x04, 0x04, 0x08, 0x10, 0x20, 0x42, 0x7E, 0x00, 0x00,

    //(ord:3)~3
    0x00, 0x00, 0x00, 0x3C, 0x42, 0x42, 0x04, 0x18, 0x04, 0x02, 0x02, 0x42, 0x44, 0x38, 0x00, 0x00,

    //(ord:4)~4
    0x00, 0x00, 0x00, 0x04, 0x0C, 0x14, 0x24, 0x24, 0x44, 0x44, 0x7E, 0x04, 0x04, 0x1E, 0x00, 0x00,

    //(ord:5)~5
    0x00, 0x00, 0x00, 0x7E, 0x40, 0x40, 0x40, 0x58, 0x64, 0x02, 0x02, 0x42, 0x44, 0x38, 0x00, 0x00,

    //(ord:0)~6
    0x00, 0x00, 0x00, 0x1C, 0x24, 0x40, 0x40, 0x58, 0x64, 0x42, 0x42, 0x42, 0x24, 0x18, 0x00, 0x00,

    //(ord:0)~7
    0x00, 0x00, 0x00, 0x7E, 0x44, 0x44, 0x08, 0x08, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00,

    //(ord:0)~8
    0x00, 0x00, 0x00, 0x3C, 0x42, 0x42, 0x42, 0x24, 0x18, 0x24, 0x42, 0x42, 0x42, 0x3C, 0x00, 0x00,

    //(ord:0)~9
    0x00, 0x00, 0x00, 0x18, 0x24, 0x42, 0x42, 0x42, 0x26, 0x1A, 0x02, 0x02, 0x24, 0x38, 0x00, 0x00,

    // A~Z,宋体12;  此字体下对应的点阵为：宽x高=8x16
    //(ord:10)~A
    0x00, 0x00, 0x00, 0x10, 0x10, 0x18, 0x28, 0x28, 0x24, 0x3C, 0x44, 0x42, 0x42, 0xE7, 0x00, 0x00,

    //(ord:11)~B
    0x00, 0x00, 0x00, 0xF8, 0x44, 0x44, 0x44, 0x78, 0x44, 0x42, 0x42, 0x42, 0x44, 0xF8, 0x00, 0x00,

    //(ord:12)~C
    0x00, 0x00, 0x00, 0x3E, 0x42, 0x42, 0x80, 0x80, 0x80, 0x80, 0x80, 0x42, 0x44, 0x38, 0x00, 0x00,

    //(ord:13)~D
    0x00, 0x00, 0x00, 0xF8, 0x44, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x44, 0xF8, 0x00, 0x00,

    //(ord:14)~E
    0x00, 0x00, 0x00, 0xFC, 0x42, 0x48, 0x48, 0x78, 0x48, 0x48, 0x40, 0x42, 0x42, 0xFC, 0x00, 0x00,

    //(ord:15)~F
    0x00, 0x00, 0x00, 0xFC, 0x42, 0x48, 0x48, 0x78, 0x48, 0x48, 0x40, 0x40, 0x40, 0xE0, 0x00, 0x00,

    //(ord:16)~G
    0x00, 0x00, 0x00, 0x3C, 0x44, 0x44, 0x80, 0x80, 0x80, 0x8E, 0x84, 0x44, 0x44, 0x38, 0x00, 0x00,

    //(ord:17)~H
    0x00, 0x00, 0x00, 0xE7, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x42, 0x42, 0x42, 0x42, 0xE7, 0x00, 0x00,

    //(ord:18)~I
    0x00, 0x00, 0x00, 0x7C, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x7C, 0x00, 0x00,

    //(ord:19)~J
    0x00, 0x00, 0x00, 0x3E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x88, 0xF0,

    //(ord:20)~K
    0x00, 0x00, 0x00, 0xEE, 0x44, 0x48, 0x50, 0x70, 0x50, 0x48, 0x48, 0x44, 0x44, 0xEE, 0x00, 0x00,

    //(ord:21)~L
    0x00, 0x00, 0x00, 0xE0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x42, 0xFE, 0x00, 0x00,

    //(ord:22)~M
    0x00, 0x00, 0x00, 0xEE, 0x6C, 0x6C, 0x6C, 0x6C, 0x54, 0x54, 0x54, 0x54, 0x54, 0xD6, 0x00, 0x00,

    //(ord:23)~N
    0x00, 0x00, 0x00, 0xC7, 0x62, 0x62, 0x52, 0x52, 0x4A, 0x4A, 0x4A, 0x46, 0x46, 0xE2, 0x00, 0x00,

    //(ord:24)~O
    0x00, 0x00, 0x00, 0x38, 0x44, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x44, 0x38, 0x00, 0x00,

    //(ord:25)~P
    0x00, 0x00, 0x00, 0xFC, 0x42, 0x42, 0x42, 0x42, 0x7C, 0x40, 0x40, 0x40, 0x40, 0xE0, 0x00, 0x00,

    //(ord:26)~Q
    0x00, 0x00, 0x00, 0x38, 0x44, 0x82, 0x82, 0x82, 0x82, 0x82, 0xB2, 0xCA, 0x4C, 0x38, 0x06, 0x00,

    //(ord:27)~R
    0x00, 0x00, 0x00, 0xFC, 0x42, 0x42, 0x42, 0x7C, 0x48, 0x48, 0x44, 0x44, 0x42, 0xE3, 0x00, 0x00,

    //(ord:28)~S
    0x00, 0x00, 0x00, 0x3E, 0x42, 0x42, 0x40, 0x20, 0x18, 0x04, 0x02, 0x42, 0x42, 0x7C, 0x00, 0x00,

    //(ord:29)~T
    0x00, 0x00, 0x00, 0xFE, 0x92, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x38, 0x00, 0x00,

    //(ord:30)~U
    0x00, 0x00, 0x00, 0xE7, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C, 0x00, 0x00,

    //(ord:31)~V
    0x00, 0x00, 0x00, 0xE7, 0x42, 0x42, 0x44, 0x24, 0x24, 0x28, 0x28, 0x18, 0x10, 0x10, 0x00, 0x00,

    //(ord:32)~W
    0x00, 0x00, 0x00, 0xD6, 0x92, 0x92, 0x92, 0x92, 0xAA, 0xAA, 0x6C, 0x44, 0x44, 0x44, 0x00, 0x00,

    //(ord:33)~X
    0x00, 0x00, 0x00, 0xE7, 0x42, 0x24, 0x24, 0x18, 0x18, 0x18, 0x24, 0x24, 0x42, 0xE7, 0x00, 0x00,

    //(ord:34)~Y
    0x00, 0x00, 0x00, 0xEE, 0x44, 0x44, 0x28, 0x28, 0x10, 0x10, 0x10, 0x10, 0x10, 0x38, 0x00, 0x00,

    //(ord:35)~Z
    0x00, 0x00, 0x00, 0x7E, 0x84, 0x04, 0x08, 0x08, 0x10, 0x20, 0x20, 0x42, 0x42, 0xFC, 0x00, 0x00,

    //(ord:36)~ [
    0x00, 0x1E, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1E, 0x00,

    //(ord:37)~ '\'
    0x00, 0x00, 0x40, 0x40, 0x20, 0x20, 0x10, 0x10, 0x10, 0x08, 0x08, 0x04, 0x04, 0x04, 0x02, 0x02,

    //(ord:38)~ ]
    0x00, 0x78, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x78, 0x00,

    //(ord:39)~ ^
    0x00, 0x1C, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    //(ord:40)~ _
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,

    //(ord:41)~ `
    0x00, 0x60, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    //(ord:42)~ a
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x42, 0x1E, 0x22, 0x42, 0x42, 0x3F, 0x00, 0x00,

    //(ord:43)~ b
    0x00, 0x00, 0x00, 0xC0, 0x40, 0x40, 0x40, 0x58, 0x64, 0x42, 0x42, 0x42, 0x64, 0x58, 0x00, 0x00,

    //(ord:44)~ c
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x22, 0x40, 0x40, 0x40, 0x22, 0x1C, 0x00, 0x00,

    //(ord:45)~ d
    0x00, 0x00, 0x00, 0x06, 0x02, 0x02, 0x02, 0x1E, 0x22, 0x42, 0x42, 0x42, 0x26, 0x1B, 0x00, 0x00,

    //(ord:46)~ e
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x42, 0x7E, 0x40, 0x40, 0x42, 0x3C, 0x00, 0x00,

    //(ord:47)~ f
    0x00, 0x00, 0x00, 0x0F, 0x11, 0x10, 0x10, 0x7E, 0x10, 0x10, 0x10, 0x10, 0x10, 0x7C, 0x00, 0x00,

    //(ord:48)~ g
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x44, 0x44, 0x38, 0x40, 0x3C, 0x42, 0x42, 0x3C,

    //(ord:49)~ h
    0x00, 0x00, 0x00, 0xC0, 0x40, 0x40, 0x40, 0x5C, 0x62, 0x42, 0x42, 0x42, 0x42, 0xE7, 0x00, 0x00,

    //(ord:50)~ i
    0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x70, 0x10, 0x10, 0x10, 0x10, 0x10, 0x7C, 0x00, 0x00,

    //(ord:51)~ j
    0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00, 0x00, 0x1C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x44, 0x78,

    //(ord:52)~ k
    0x00, 0x00, 0x00, 0xC0, 0x40, 0x40, 0x40, 0x4E, 0x48, 0x50, 0x68, 0x48, 0x44, 0xEE, 0x00, 0x00,

    //(ord:53)~ l
    0x00, 0x00, 0x00, 0x70, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x7C, 0x00, 0x00,

    //(ord:54)~ m
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x49, 0x49, 0x49, 0x49, 0x49, 0xED, 0x00, 0x00,

    //(ord:55)~ n
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0x62, 0x42, 0x42, 0x42, 0x42, 0xE7, 0x00, 0x00,

    //(ord:56)~ o
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C, 0x00, 0x00,

    //(ord:57)~ p
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD8, 0x64, 0x42, 0x42, 0x42, 0x44, 0x78, 0x40, 0xE0,

    //(ord:58)~ q
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x22, 0x42, 0x42, 0x42, 0x22, 0x1E, 0x02, 0x07,

    //(ord:59)~ r
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEE, 0x32, 0x20, 0x20, 0x20, 0x20, 0xF8, 0x00, 0x00,

    //(ord:60)~ s
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x42, 0x40, 0x3C, 0x02, 0x42, 0x7C, 0x00, 0x00,

    //(ord:61)~ t
    0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x7C, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0C, 0x00, 0x00,

    //(ord:62)~ u
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC6, 0x42, 0x42, 0x42, 0x42, 0x46, 0x3B, 0x00, 0x00,

    //(ord:63)~ v
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE7, 0x42, 0x24, 0x24, 0x28, 0x10, 0x10, 0x00, 0x00,

    //(ord:64)~ w
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD7, 0x92, 0x92, 0xAA, 0xAA, 0x44, 0x44, 0x00, 0x00,

    //(ord:65)~ x
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6E, 0x24, 0x18, 0x18, 0x18, 0x24, 0x76, 0x00, 0x00,

    //(ord:66)~ y
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE7, 0x42, 0x24, 0x24, 0x28, 0x18, 0x10, 0x10, 0xE0,

    //(ord:67)~ z
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x44, 0x08, 0x10, 0x10, 0x22, 0x7E, 0x00, 0x00,

    //(ord:68)~ {
    0x00, 0x03, 0x04, 0x04, 0x04, 0x04, 0x04, 0x08, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x03, 0x00,

    //(ord:69)~ |
    0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,

    //(ord:70)~ }
    0x00, 0x60, 0x10, 0x10, 0x10, 0x10, 0x10, 0x08, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x60, 0x00,

    //(ord:71)~ ~
    0x30, 0x4C, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,


    //(ord:72)~ 空格
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    //(ord:73)~ !
    0x00, 0x00, 0x00, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00,

    //(ord:74)~ "
    0x00, 0x12, 0x36, 0x24, 0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    //(ord:75)~ #
    0x00, 0x00, 0x00, 0x24, 0x24, 0x24, 0xFE, 0x48, 0x48, 0x48, 0xFE, 0x48, 0x48, 0x48, 0x00, 0x00,

    //(ord:76)~ $
    0x00, 0x00, 0x10, 0x38, 0x54, 0x54, 0x50, 0x30, 0x18, 0x14, 0x14, 0x54, 0x54, 0x38, 0x10, 0x10,

    //(ord:77)~ %
    0x00, 0x00, 0x00, 0x44, 0xA4, 0xA8, 0xA8, 0xA8, 0x54, 0x1A, 0x2A, 0x2A, 0x2A, 0x44, 0x00, 0x00,

    //(ord:78)~ &
    0x00, 0x00, 0x00, 0x30, 0x48, 0x48, 0x48, 0x50, 0x6E, 0xA4, 0x94, 0x88, 0x89, 0x76, 0x00, 0x00,

    //(ord:79)~ '
    0x00, 0x60, 0x60, 0x20, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    //(ord:80)~ (
    0x00, 0x02, 0x04, 0x08, 0x08, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x08, 0x08, 0x04, 0x02, 0x00,

    //(ord:81)~ )
    0x00, 0x40, 0x20, 0x10, 0x10, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x10, 0x10, 0x20, 0x40, 0x00,

    //(ord:82)~ *
    0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0xD6, 0x38, 0x38, 0xD6, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00,

    //(ord:83)~ +
    0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x10, 0x10, 0xFE, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00,

    //(ord:84)~ ,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x20, 0xC0,

    //(ord:85)~ -
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    //(ord:86)~ .
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00,

    //(ord:87)~ /
    0x00, 0x00, 0x01, 0x02, 0x02, 0x04, 0x04, 0x08, 0x08, 0x10, 0x10, 0x20, 0x20, 0x40, 0x40, 0x00,

    //(ord:88)~ :
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00,

    //(ord:89)~ ;
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x20,

    //(ord:90)~ <
    0x00, 0x00, 0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x00,

    //(ord:91)~ =
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00,

    //(ord:92)~ >
    0x00, 0x00, 0x00, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x00, 0x00,

    //(ord:93)~ ?
    0x00, 0x00, 0x00, 0x3C, 0x42, 0x42, 0x62, 0x02, 0x04, 0x08, 0x08, 0x00, 0x18, 0x18, 0x00, 0x00,

    //(ord:94)~ @
    0x00, 0x00, 0x00, 0x38, 0x44, 0x5A, 0xAA, 0xAA, 0xAA, 0xAA, 0xB4, 0x42, 0x44, 0x38, 0x00, 0x00,

    //(ord:95)~ 0xff
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,

};
#endif

//宋体16,点阵为：宽x高=16x21
#ifdef CHAR_FONT_W16_H21
//ascii 32~90(!~Z), (32~47)空格~/,(48~57)0~9,(58~64):~@,(65~126)A~~
//ord 0~95, (48~57)0~9,(65~126)A~z,(33~47)!~/,(58~64):~@
unsigned char const ascii[] =
{
    //宋体16,点阵为：宽x高=16x21
    /*--  文字:  0  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x31, 0x80, 0x31, 0x80, 0x60, 0xC0,
    0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x31, 0x80,
    0x31, 0x80, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  1  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x1E, 0x00, 0x06, 0x00, 0x06, 0x00,
    0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00,
    0x06, 0x00, 0x1F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  2  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x61, 0x80, 0x60, 0xC0, 0x60, 0xC0,
    0x60, 0xC0, 0x01, 0x80, 0x01, 0x80, 0x03, 0x00, 0x06, 0x00, 0x0C, 0x00, 0x18, 0x00, 0x30, 0xC0,
    0x60, 0xC0, 0x7F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  3  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x63, 0x00, 0x61, 0x80, 0x61, 0x80,
    0x01, 0x80, 0x03, 0x00, 0x0F, 0x00, 0x01, 0x80, 0x00, 0xC0, 0x00, 0xC0, 0x60, 0xC0, 0x60, 0xC0,
    0x61, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  4  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x03, 0x80, 0x07, 0x80, 0x07, 0x80, 0x0F, 0x80,
    0x1B, 0x80, 0x1B, 0x80, 0x33, 0x80, 0x63, 0x80, 0x63, 0x80, 0x3F, 0xC0, 0x03, 0x80, 0x03, 0x80,
    0x03, 0x80, 0x0F, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  5  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xC0, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00,
    0x30, 0x00, 0x3F, 0x00, 0x39, 0x80, 0x30, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x60, 0xC0, 0x60, 0xC0,
    0x61, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  6  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x18, 0xC0, 0x30, 0xC0, 0x30, 0x00,
    0x60, 0x00, 0x7F, 0x00, 0x71, 0x80, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x30, 0xC0,
    0x31, 0x80, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  7  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xC0, 0x61, 0x80, 0x63, 0x00, 0x03, 0x00,
    0x06, 0x00, 0x06, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x18, 0x00, 0x18, 0x00, 0x18, 0x00, 0x18, 0x00,
    0x18, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  8  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x31, 0x80, 0x60, 0xC0, 0x60, 0xC0,
    0x60, 0xC0, 0x31, 0x80, 0x1F, 0x00, 0x31, 0x80, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0,
    0x31, 0x80, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  9  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x31, 0x80, 0x61, 0x80, 0x60, 0xC0,
    0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x31, 0xC0, 0x1F, 0xC0, 0x00, 0xC0, 0x01, 0x80, 0x61, 0x80,
    0x63, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  A  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x0E, 0x00, 0x0E, 0x00, 0x0F, 0x00,
    0x1B, 0x00, 0x1B, 0x00, 0x1B, 0x00, 0x31, 0x80, 0x3F, 0x80, 0x31, 0x80, 0x31, 0x80, 0x60, 0xC0,
    0x60, 0xC0, 0xF1, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  B  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x31, 0x80, 0x30, 0xC0, 0x30, 0xC0,
    0x30, 0xC0, 0x31, 0x80, 0x3F, 0x00, 0x31, 0xC0, 0x30, 0x60, 0x30, 0x60, 0x30, 0x60, 0x30, 0x60,
    0x30, 0xC0, 0x7F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  C  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xE0, 0x18, 0xE0, 0x30, 0x60, 0x30, 0x60,
    0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x60, 0x30, 0x60,
    0x38, 0xC0, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  D  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x31, 0x80, 0x30, 0xC0, 0x30, 0xC0,
    0x30, 0x60, 0x30, 0x60, 0x30, 0x60, 0x30, 0x60, 0x30, 0x60, 0x30, 0x60, 0x30, 0xC0, 0x30, 0xC0,
    0x31, 0x80, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  E  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xC0, 0x30, 0xC0, 0x30, 0x60, 0x30, 0x00,
    0x31, 0x80, 0x31, 0x80, 0x3F, 0x80, 0x31, 0x80, 0x31, 0x80, 0x30, 0x00, 0x30, 0x00, 0x30, 0x60,
    0x30, 0xC0, 0x7F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  F  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xC0, 0x30, 0xC0, 0x30, 0x60, 0x30, 0x00,
    0x31, 0x80, 0x31, 0x80, 0x3F, 0x80, 0x31, 0x80, 0x31, 0x80, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00,
    0x30, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  G  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xC0, 0x19, 0xC0, 0x30, 0xC0, 0x30, 0xC0,
    0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x61, 0xE0, 0x60, 0xC0, 0x30, 0xC0, 0x30, 0xC0,
    0x18, 0xC0, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  H  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79, 0xE0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0,
    0x30, 0xC0, 0x30, 0xC0, 0x3F, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0,
    0x30, 0xC0, 0x79, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  I  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xC0, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00,
    0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00,
    0x06, 0x00, 0x3F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  J  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xF0, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80,
    0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80,
    0x01, 0x80, 0x61, 0x80, 0x63, 0x00, 0x3E, 0x00, 0x00, 0x00,

    /*--  文字:  K  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7B, 0xE0, 0x30, 0xC0, 0x31, 0x80, 0x33, 0x00,
    0x36, 0x00, 0x3C, 0x00, 0x3E, 0x00, 0x36, 0x00, 0x33, 0x00, 0x33, 0x00, 0x31, 0x80, 0x31, 0x80,
    0x30, 0xC0, 0x79, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  L  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00,
    0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x60,
    0x30, 0xC0, 0x7F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  M  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF0, 0x70, 0xE0, 0x70, 0xE0, 0x70, 0xE0,
    0x79, 0xE0, 0x79, 0xE0, 0x79, 0xE0, 0x7B, 0x60, 0x6F, 0x60, 0x6F, 0x60, 0x6F, 0x60, 0x66, 0x60,
    0x66, 0x60, 0xF6, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  N  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79, 0xE0, 0x38, 0xC0, 0x3C, 0xC0, 0x3C, 0xC0,
    0x3C, 0xC0, 0x36, 0xC0, 0x36, 0xC0, 0x36, 0xC0, 0x33, 0xC0, 0x33, 0xC0, 0x31, 0xC0, 0x31, 0xC0,
    0x31, 0xC0, 0x78, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  O  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x19, 0x80, 0x30, 0xC0, 0x60, 0x60,
    0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x30, 0xC0,
    0x19, 0x80, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  P  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x80, 0x30, 0xC0, 0x30, 0x60, 0x30, 0x60,
    0x30, 0x60, 0x30, 0xC0, 0x3F, 0x80, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00,
    0x30, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  Q  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x19, 0x80, 0x30, 0xC0, 0x60, 0x60,
    0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x6E, 0x60, 0x3B, 0xC0,
    0x3B, 0xC0, 0x0F, 0x80, 0x01, 0xE0, 0x01, 0xC0, 0x00, 0x00,

    /*--  文字:  R  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x31, 0x80, 0x30, 0xC0, 0x30, 0xC0,
    0x30, 0xC0, 0x31, 0x80, 0x3F, 0x00, 0x36, 0x00, 0x33, 0x00, 0x33, 0x00, 0x31, 0x80, 0x31, 0x80,
    0x30, 0xC0, 0x78, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  S  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xC0, 0x31, 0xC0, 0x60, 0xC0, 0x60, 0xC0,
    0x60, 0x00, 0x30, 0x00, 0x1C, 0x00, 0x07, 0x00, 0x01, 0x80, 0x00, 0xC0, 0x60, 0xC0, 0x60, 0xC0,
    0x71, 0x80, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  T  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xC0, 0x66, 0x60, 0x66, 0x60, 0x06, 0x00,
    0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00,
    0x06, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  U  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79, 0xE0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0,
    0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0,
    0x19, 0x80, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  V  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF1, 0xF0, 0x60, 0x60, 0x30, 0xC0, 0x30, 0xC0,
    0x30, 0xC0, 0x19, 0x80, 0x19, 0x80, 0x19, 0x80, 0x19, 0x80, 0x0F, 0x00, 0x0F, 0x00, 0x0F, 0x00,
    0x06, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  W  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xF0, 0x66, 0x60, 0x66, 0x60, 0x66, 0x60,
    0x66, 0xC0, 0x37, 0xC0, 0x3F, 0xC0, 0x3F, 0xC0, 0x3F, 0xC0, 0x3F, 0xC0, 0x3F, 0xC0, 0x19, 0x80,
    0x19, 0x80, 0x19, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  X  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79, 0xE0, 0x30, 0xC0, 0x19, 0x80, 0x19, 0x80,
    0x0F, 0x00, 0x0F, 0x00, 0x06, 0x00, 0x06, 0x00, 0x0F, 0x00, 0x0F, 0x00, 0x19, 0x80, 0x19, 0x80,
    0x30, 0xC0, 0x79, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  Y  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79, 0xE0, 0x30, 0xC0, 0x19, 0x80, 0x19, 0x80,
    0x1B, 0x00, 0x0F, 0x00, 0x0F, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00,
    0x06, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  Z  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xE0, 0x30, 0xC0, 0x60, 0xC0, 0x01, 0x80,
    0x03, 0x00, 0x03, 0x00, 0x06, 0x00, 0x06, 0x00, 0x0C, 0x00, 0x18, 0x00, 0x18, 0x00, 0x30, 0x60,
    0x30, 0xC0, 0x7F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  [  --*/
    0x00, 0x00, 0x00, 0x00, 0x0F, 0xC0, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00,
    0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00,
    0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0F, 0xC0, 0x00, 0x00,

    /*--  文字:  \  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x30, 0x00, 0x18, 0x00, 0x18, 0x00, 0x18, 0x00,
    0x0C, 0x00, 0x0C, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x03, 0x00, 0x03, 0x00, 0x01, 0x80,
    0x01, 0x80, 0x01, 0x80, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0x00,

    /*--  文字:  ]  --*/
    0x00, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00,
    0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00,
    0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x7E, 0x00, 0x00, 0x00,

    /*--  文字:  ^  --*/
    0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x1E, 0x00, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  _  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xE0,

    /*--  文字:  `  --*/
    0x00, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  a  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x1F, 0x00, 0x31, 0x80, 0x31, 0x80, 0x0F, 0x80, 0x39, 0x80, 0x61, 0x80, 0x61, 0x80,
    0x63, 0xE0, 0x3F, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  b  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00,
    0x30, 0x00, 0x3F, 0x80, 0x38, 0xC0, 0x30, 0x60, 0x30, 0x60, 0x30, 0x60, 0x30, 0x60, 0x30, 0x60,
    0x38, 0xC0, 0x3F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  c  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x1F, 0x00, 0x31, 0x80, 0x61, 0x80, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0xC0,
    0x31, 0x80, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  d  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0,
    0x00, 0xC0, 0x1F, 0xC0, 0x31, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0,
    0x31, 0xE0, 0x1F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  e  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x1F, 0x00, 0x31, 0x80, 0x60, 0xC0, 0x7F, 0xC0, 0x60, 0x00, 0x60, 0x00, 0x60, 0xC0,
    0x31, 0x80, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  f  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0x06, 0x60, 0x0C, 0x00, 0x0C, 0x00,
    0x0C, 0x00, 0x7F, 0x80, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00,
    0x0C, 0x00, 0x3F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  g  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x1F, 0xF0, 0x30, 0xF0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x1F, 0x80, 0x30, 0x00,
    0x3F, 0x80, 0x30, 0xE0, 0x60, 0x60, 0x70, 0x60, 0x3F, 0xC0,

    /*--  文字:  h  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00,
    0x30, 0x00, 0x3F, 0x80, 0x38, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0,
    0x30, 0xC0, 0x79, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  i  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x1E, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00,
    0x06, 0x00, 0x1F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  j  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x07, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80,
    0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x33, 0x00, 0x3E, 0x00,

    /*--  文字:  k  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x30, 0x00, 0x30, 0x00, 0x30, 0x00,
    0x30, 0x00, 0x33, 0xC0, 0x33, 0x00, 0x36, 0x00, 0x3C, 0x00, 0x3E, 0x00, 0x33, 0x00, 0x31, 0x80,
    0x30, 0xC0, 0x79, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  l  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00,
    0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00,
    0x06, 0x00, 0x3F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  m  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFF, 0xE0, 0x77, 0x60, 0x66, 0x60, 0x66, 0x60, 0x66, 0x60, 0x66, 0x60, 0x66, 0x60,
    0x66, 0x60, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  n  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7F, 0x80, 0x38, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0,
    0x30, 0xC0, 0x79, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  o  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x1F, 0x00, 0x31, 0x80, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0,
    0x31, 0x80, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  p  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7F, 0x80, 0x38, 0xC0, 0x30, 0x60, 0x30, 0x60, 0x30, 0x60, 0x30, 0x60, 0x30, 0x60,
    0x38, 0xC0, 0x3F, 0x80, 0x30, 0x00, 0x30, 0x00, 0x78, 0x00,

    /*--  文字:  q  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x1F, 0xC0, 0x31, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0, 0x60, 0xC0,
    0x31, 0xC0, 0x1F, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x01, 0xE0,

    /*--  文字:  r  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7B, 0xE0, 0x1E, 0x60, 0x1C, 0x00, 0x18, 0x00, 0x18, 0x00, 0x18, 0x00, 0x18, 0x00,
    0x18, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  s  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x1F, 0xC0, 0x31, 0xC0, 0x30, 0xC0, 0x38, 0x00, 0x0F, 0x00, 0x01, 0xC0, 0x30, 0xC0,
    0x38, 0xC0, 0x3F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  t  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x0C, 0x00,
    0x0C, 0x00, 0x3F, 0x80, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0xC0,
    0x0C, 0xC0, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  u  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x71, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0, 0x30, 0xC0,
    0x31, 0xE0, 0x1F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  v  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x79, 0xE0, 0x30, 0xC0, 0x19, 0x80, 0x19, 0x80, 0x19, 0x80, 0x0F, 0x00, 0x0F, 0x00,
    0x06, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  w  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFF, 0xF0, 0x66, 0x60, 0x66, 0xE0, 0x3F, 0xC0, 0x3F, 0xC0, 0x3F, 0xC0, 0x3F, 0xC0,
    0x19, 0x80, 0x19, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  x  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x79, 0xE0, 0x30, 0xC0, 0x19, 0x80, 0x0F, 0x00, 0x06, 0x00, 0x0F, 0x00, 0x19, 0x80,
    0x30, 0xC0, 0x79, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  y  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7B, 0xE0, 0x30, 0xC0, 0x19, 0x80, 0x19, 0x80, 0x0F, 0x00, 0x0F, 0x00, 0x06, 0x00,
    0x06, 0x00, 0x06, 0x00, 0x0C, 0x00, 0x3C, 0x00, 0x38, 0x00,

    /*--  文字:  z  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7F, 0x80, 0x63, 0x00, 0x66, 0x00, 0x06, 0x00, 0x0C, 0x00, 0x18, 0x00, 0x30, 0xC0,
    0x61, 0x80, 0x7F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  {  --*/
    0x00, 0x00, 0x00, 0x00, 0x01, 0xC0, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00,
    0x03, 0x00, 0x07, 0x00, 0x0C, 0x00, 0x07, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00,
    0x03, 0x00, 0x03, 0x00, 0x03, 0x00, 0x01, 0xC0, 0x00, 0x00,

    /*--  文字:  |  --*/
    0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00,
    0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00,
    0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00,

    /*--  文字:  }  --*/
    0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00,
    0x0C, 0x00, 0x0E, 0x00, 0x03, 0x00, 0x0E, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00,
    0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x38, 0x00, 0x00, 0x00,

    /*--  文字:  ~  --*/
    0x3C, 0x00, 0x76, 0x30, 0x63, 0x70, 0x01, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:     --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  !  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x0E, 0x00, 0x0E, 0x00, 0x0E, 0x00, 0x0E, 0x00,
    0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0E, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  "  --*/
    0x00, 0x00, 0x00, 0x00, 0x1F, 0x80, 0x1F, 0x80, 0x3F, 0x00, 0x36, 0x00, 0x6C, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  #  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xC0, 0x18, 0xC0, 0x18, 0xC0, 0x7F, 0xE0,
    0x18, 0xC0, 0x18, 0xC0, 0x18, 0xC0, 0x31, 0x80, 0x31, 0x80, 0x7F, 0xE0, 0x31, 0x80, 0x31, 0x80,
    0x31, 0x80, 0x31, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  $  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x0F, 0x80, 0x1E, 0xC0, 0x36, 0xC0, 0x37, 0xC0,
    0x36, 0x00, 0x1E, 0x00, 0x0E, 0x00, 0x07, 0x00, 0x07, 0x80, 0x06, 0xC0, 0x3E, 0xC0, 0x36, 0xC0,
    0x37, 0x80, 0x1F, 0x00, 0x06, 0x00, 0x06, 0x00, 0x00, 0x00,

    /*--  文字:  %  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0xC0, 0x6D, 0x80, 0x6D, 0x80, 0x6F, 0x00,
    0x6F, 0x00, 0x6F, 0x00, 0x3E, 0x00, 0x07, 0xC0, 0x0F, 0x60, 0x0F, 0x60, 0x1B, 0x60, 0x1B, 0x60,
    0x1B, 0x60, 0x31, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  &  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x33, 0x00, 0x33, 0x00, 0x33, 0x00,
    0x36, 0x00, 0x3F, 0xC0, 0x39, 0x80, 0x79, 0x80, 0xCF, 0x00, 0xCF, 0x00, 0xC7, 0x00, 0xC6, 0x00,
    0x67, 0x60, 0x3D, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  '  --*/
    0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x78, 0x00, 0x18, 0x00, 0x30, 0x00, 0xE0, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  (  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0xC0, 0x01, 0x80, 0x03, 0x00, 0x03, 0x00, 0x03, 0x00,
    0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x03, 0x00, 0x03, 0x00,
    0x03, 0x00, 0x01, 0x80, 0x00, 0xC0, 0x00, 0x60, 0x00, 0x00,

    /*--  文字:  )  --*/
    0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x30, 0x00, 0x18, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00,
    0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x0C, 0x00, 0x0C, 0x00,
    0x0C, 0x00, 0x18, 0x00, 0x30, 0x00, 0x60, 0x00, 0x00, 0x00,

    /*--  文字:  *  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x06, 0x00, 0x66, 0x60,
    0x7F, 0xE0, 0x1F, 0x80, 0x06, 0x00, 0x1F, 0x80, 0x7F, 0xE0, 0x66, 0x60, 0x06, 0x00, 0x06, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  +  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00,
    0x06, 0x00, 0x06, 0x00, 0x7F, 0xE0, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  ,  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x78, 0x00, 0x78, 0x00, 0x18, 0x00, 0x30, 0x00, 0xE0, 0x00,

    /*--  文字:  -  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x7F, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  .  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x70, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  /  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0xC0, 0x00, 0xC0, 0x01, 0xC0, 0x01, 0x80, 0x01, 0x80,
    0x03, 0x00, 0x03, 0x00, 0x06, 0x00, 0x06, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x18, 0x00, 0x18, 0x00,
    0x38, 0x00, 0x30, 0x00, 0x30, 0x00, 0x60, 0x00, 0x00, 0x00,

    /*--  文字:  :  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0E, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0E, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  ;  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00,

    /*--  文字:  <  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x01, 0x80, 0x03, 0x00, 0x06, 0x00, 0x0C, 0x00,
    0x18, 0x00, 0x30, 0x00, 0x60, 0x00, 0x30, 0x00, 0x18, 0x00, 0x0C, 0x00, 0x06, 0x00, 0x03, 0x00,
    0x01, 0x80, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  =  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x7F, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  >  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x30, 0x00, 0x18, 0x00, 0x0C, 0x00, 0x06, 0x00,
    0x03, 0x00, 0x01, 0x80, 0x00, 0xC0, 0x01, 0x80, 0x03, 0x00, 0x06, 0x00, 0x0C, 0x00, 0x18, 0x00,
    0x30, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  ?  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x31, 0x80, 0x60, 0xC0, 0x70, 0xC0, 0x70, 0xC0,
    0x00, 0xC0, 0x00, 0xC0, 0x01, 0x80, 0x03, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x00, 0x00,
    0x0E, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /*--  文字:  @  --*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x19, 0xC0, 0x30, 0xC0, 0x37, 0xE0,
    0x6F, 0xE0, 0x6D, 0xE0, 0x79, 0xE0, 0x7B, 0x60, 0x7B, 0x60, 0x7B, 0xC0, 0x6F, 0xC0, 0x30, 0x60,
    0x18, 0xC0, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    //ord:95
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};
#endif

