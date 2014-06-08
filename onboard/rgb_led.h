/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011, 2012, 2013  Bill Nesbitt
*/
/*
    This file is part of AutoQuad For PX4FMU/PixHawk.
    Copyright © 2014  Ben Zeng

*/

#ifndef _rgb_led_h
#define _rgb_led_h

#include "stm32f4xx_i2c.h"
#include <CoOS.h>

#ifndef true
#define true 1
#endif
#ifndef false
#define false 0
#endif
#ifndef bool
typedef unsigned char bool;
#endif

#define RGBLED_STACK_SIZE   105
#define RGBLED_PRIORITY	    34

#define PX4_I2C_OBDEV_LED_ADDR  0x55	/**< I2C adress of TCA62724FMG */
#define SUB_ADDR_START		0x01	/**< write everything (with auto-increment) */
#define SUB_ADDR_PWM0		0x81	/**< blue     (without auto-increment) */
#define SUB_ADDR_PWM1		0x82	/**< green    (without auto-increment) */
#define SUB_ADDR_PWM2		0x83	/**< red      (without auto-increment) */
#define SUB_ADDR_SETTINGS	0x84	/**< settings (without auto-increment)*/

#define SETTING_NOT_POWERSAVE	0x01	/**< power-save mode not off */
#define SETTING_ENABLE   	0x02	/**< on */


/* Maximum Timeout values for flags and events waiting loops. These timeouts are
   not based on accurate values, they just guarantee that the application will 
   not remain stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define RGBLED_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define RGBLED_LONG_TIMEOUT         ((uint32_t)(30 * RGBLED_FLAG_TIMEOUT))


/**
  * @brief  I2C RGB LED Interface pins. RGB LED Driver is on I2C2.
  */  
#define RGBLED_I2C                          I2C2
#define RGBLED_I2C_CLK                      RCC_APB1Periph_I2C2

#define RGBLED_I2C_SCL_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define RGBLED_I2C_SCL_PIN                  GPIO_Pin_10                  /* PB.10*/
#define RGBLED_I2C_SCL_GPIO_PORT            GPIOB                        /* GPIOB */
#define RGBLED_I2C_SCL_SOURCE               GPIO_PinSource10
#define RGBLED_I2C_SCL_AF                   GPIO_AF_I2C2

#define RGBLED_I2C_SDA_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define RGBLED_I2C_SDA_PIN                  GPIO_Pin_11                  /* PB.11 */
#define RGBLED_I2C_SDA_GPIO_PORT            GPIOB                        /* GPIOB */
#define RGBLED_I2C_SDA_SOURCE               GPIO_PinSource11
#define RGBLED_I2C_SDA_AF                   GPIO_AF_I2C2

// #define PERIPH_BASE           ((uint32_t)0x40000000) 
// #define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
// #define DMA2_BASE             (AHB1PERIPH_BASE + 0x6400)
// #define DMA2_Stream3_BASE     (DMA2_BASE + 0x058)
#define RGBLED_I2C_DMA                      DMA2  
#define RGBLED_I2C_DMA_CHANNEL              DMA_Channel_4
#define RGBLED_I2C_DMA_STREAM_TX            DMA2_Stream5
#define RGBLED_I2C_DMA_CLK                  RCC_AHB1Periph_DMA2
#define RGBLED_I2C_DMA_TX_IRQn              DMA2_Stream5_IRQn
#define RGBLED_I2C_DMA_TX_IRQHandler        DMA2_Stream5_IRQHandler

// #define I2C2_BASE             (APB1PERIPH_BASE + 0x5800)
// #define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define RGBLED_I2C_DR_Address               ((uint32_t)0x40005810)       // I2C2->DR  DR(Data register) offset 0x10
   

   
#define RGBLED_TX_DMA_FLAG_FEIF             DMA_FLAG_FEIF5
#define RGBLED_TX_DMA_FLAG_DMEIF            DMA_FLAG_DMEIF5
#define RGBLED_TX_DMA_FLAG_TEIF             DMA_FLAG_TEIF5
#define RGBLED_TX_DMA_FLAG_HTIF             DMA_FLAG_HTIF5
#define RGBLED_TX_DMA_FLAG_TCIF             DMA_FLAG_TCIF5


#define RGBLED_I2C_DMA_PREPRIO              0
#define RGBLED_I2C_DMA_SUBPRIO              0 

typedef enum {
	RGBLED_COLOR_OFF,
	RGBLED_COLOR_RED,
	RGBLED_COLOR_YELLOW,
	RGBLED_COLOR_PURPLE,
	RGBLED_COLOR_GREEN,
	RGBLED_COLOR_BLUE,
	RGBLED_COLOR_WHITE,
	RGBLED_COLOR_AMBER,
	RGBLED_COLOR_DIM_RED,
	RGBLED_COLOR_DIM_YELLOW,
	RGBLED_COLOR_DIM_PURPLE,
	RGBLED_COLOR_DIM_GREEN,
	RGBLED_COLOR_DIM_BLUE,
	RGBLED_COLOR_DIM_WHITE,
	RGBLED_COLOR_DIM_AMBER
} rgbled_color_t;

typedef enum {
	RGBLED_MODE_OFF,
	RGBLED_MODE_ON,
	RGBLED_MODE_BLINK_SLOW,
	RGBLED_MODE_BLINK_NORMAL,
	RGBLED_MODE_BLINK_FAST,
	RGBLED_MODE_BREATHE,
	RGBLED_MODE_PATTERN
} rgbled_mode_t;

#define RGBLED_PATTERN_LENGTH 20

typedef struct {
	rgbled_color_t color[RGBLED_PATTERN_LENGTH];
	unsigned duration[RGBLED_PATTERN_LENGTH];
} rgbled_pattern_t;

typedef struct {
    rgbled_mode_t	_mode;
    rgbled_pattern_t	_pattern;

    uint8_t	_r;
    uint8_t	_g;
    uint8_t	_b;
    float	_brightness;

    BOOL	_running;
    int		_led_interval;
    BOOL	_should_run;
    int		_counter;

}RgbLedStruct_t;

extern RgbLedStruct_t rgbledData;

void    rgbledInit( void );
void    rbgledSetColor(rgbled_color_t color);
void 	rgbledSetRGB(uint8_t r, uint8_t	g, uint8_t b );
void    rgbledSetMode(rgbled_mode_t mode);
void    set_pattern(rgbled_pattern_t *pattern);









#endif