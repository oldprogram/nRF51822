/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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
#ifndef PCA10001_H
#define PCA10001_H

#define LED_START      18
#define LED0           18
#define LED_STOP       19
#define LED1           19
#define LED_PORT       NRF_GPIO_PORT_SELECT_PORT2
#define LED_OFFSET     2

#define BUTTON_START   16
#define BUTTON0        16
#define BUTTON_STOP    17
#define BUTTON1        17

#define RX_PIN_NUMBER  11
#define TX_PIN_NUMBER  9
#define CTS_PIN_NUMBER 10
#define RTS_PIN_NUMBER 8
#define HWFC           true

#define BLINKY_STATE_MASK   0x01


////////////////////////////////////////
#define CS	15
#define RS	11
#define RET	13
#define SCL	10
#define	SDA	14
////////////////////////////////////////
//#define LCD_SCL  10
//#define LCD_SDA  14
//#define LCD_RST  13
//#define LCD_DC   11
//#define LCD_CS   15
#define LCD_POWER_EN   12
#define XLevelL		0x00
#define XLevelH		0x10
#define XLevel	    ((XLevelH&0x0F)*16+XLevelL)
#define Max_Column	128
#define Max_Row		  64
#define	Brightness	0xCF 
#define X_WIDTH 128
#define Y_WIDTH 64

#endif
