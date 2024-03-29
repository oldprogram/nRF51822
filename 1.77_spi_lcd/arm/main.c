/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
* @brief Example project on GPIO usage to drive LEDs
* @defgroup gpio_example_pca10001 GPIO example
* @{
* @ingroup nrf_examples_pca10001
*
* @brief Example of GPIO output usage.
*
* This example shows how to configure GPIO pins as outputs which can also be used to drive LEDs.
* Each LED is set on one at a time and each state lasts 100 milliseconds.
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_lcd.h"

/*
指针结构体
*/
struct DIS{
	unsigned int cen_x,cen_y;//圆心位置
	unsigned int len;//指针长度
	unsigned int dir;//为箭头方向0,1,2,3->上下左右
	unsigned int pre_dir;//记录上次绘制位置
	bool dir_change;//箭头方向是否改变,1改变，0没变	
}myDis;
void drawDIS(bool isPre){//绘制方向指针的函数
	if(myDis.dir_change==1){
	//	dir_change=0;
		switch(isPre==0 ? myDis.dir:myDis.pre_dir){
		case 0:	
			DrawLine(myDis.cen_x,myDis.cen_x,myDis.cen_y-myDis.len,myDis.cen_y,isPre==0 ? GREEN : RED);
			break;
		case 1:
			DrawLine(myDis.cen_x,myDis.cen_x,myDis.cen_y,myDis.cen_y+myDis.len,isPre==0 ? GREEN : RED);
			break;
		case 2:
			DrawLine(myDis.cen_x-myDis.len,myDis.cen_x,myDis.cen_y,myDis.cen_y,isPre==0 ? GREEN : RED);
			break;
		case 3:
			DrawLine(myDis.cen_x,myDis.cen_x+myDis.len,myDis.cen_y,myDis.cen_y,isPre==0 ? GREEN : RED);
			break;
		default:break;
		}
		if(isPre==0)DrawLine(myDis.cen_x-2,myDis.cen_x+2,myDis.cen_y-2,myDis.cen_y+2,BLACK);
		else drawDIS(0);
	}
}

/**
 * main() function
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
	unsigned int x,y,cnt;
	int dis;//方向标志，1为向下走，-1为向上走

	myDis.cen_x=COL/2;
	myDis.cen_y=ROW-50;
	myDis.dir=0;
	myDis.pre_dir=0;
	myDis.len=20;
	myDis.dir_change=1;


	GPIO_Init();
	LCD_Init();
	DispColor(RED);
	
	x=y=cnt=0;
	dis=1;


   	while(1<2)
	{
		cnt++;
		myDis.pre_dir=myDis.dir;
		myDis.dir=cnt%4;
		DispInt(cnt,COL/2-FONT_W*2,ROW,BLUE,RED);
		DispStr("X-CASE",x,y,BLACK,RED); 
		drawDIS(1);

		x+=dis*1;
		y+=dis*2;
		if(x>=COL-FONT_W*6)
		{
			dis=-1;	
		}
		if(x<=0)
		{
		 	dis=1;
		}  
		nrf_delay_ms(100);
	}
}
