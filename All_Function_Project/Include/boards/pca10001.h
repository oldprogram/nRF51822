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

#include "nrf_gpio.h"

//for xcase

//sensor, high: none magnetic,low: has magnetic 
#define SEN_LEFT   1
#define SEN_UP     2
#define SEN_DOWN   3
#define SEN_RIGHT  4

#define BUTTON_PULL    NRF_GPIO_PIN_NOPULL

#define SEN_INT    5     //high if any sensor GPIO is low
#define SEN_POWER_EN   18//enable power if high

// on/off button, low if preesed
#define BUTTON_ON_OFF           6

// spi flash

#define FLASH_POWER_EN   8//enable power if high
#if defined(FLASH_1Gb)  
#define SPI_FLASH_CS       9//FLASH_0_CS 
#elif defined(FLASH_64Mb)
#define SPI_FLASH_CS       12//FLASH_1_CS 
#endif
#define FLASH_WP         10//
#define SPI_FLASH_MISO    11//
#define SPI_FLASH_MOSI    13//
#define SPI_FLASH_CLK       14//
#define FLASH_HOLD       15//

//dbg serial or GPIO
#define S_IO_0   16//
#define S_IO_1   17//

//#define LED_0          19

#define IND_LED   19//led is on if output low

//LCD 
#define CS	25
#define RS	21
#define RET	24
#define SCL	23
#define	SDA	22

//#define LCD_SDA  22
//#define LCD_RST  24
//#define LCD_CS   25
//#define LCD_SCL  23
//#define LCD_DC   21

#define LCD_POWER_EN   28
#define LCD_BCK_LED_EN   29		  //backlignt

//others config as output for safe
//7 20

#endif
