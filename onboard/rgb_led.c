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
    DMA will be used to transfer data to TCA62724 instead of calling I2C_SendData()( one byte for each call).
    Copyright © 2014  Ben Zeng

*/
#include "aq.h"
#include "util.h"
#include "rgb_led.h"
#include <string.h>
#include <stdio.h>

// Instance data
RgbLedStruct_t rgbledData;
//static DMA_InitTypeDef    rgbledDMA_InitStructure; 
static NVIC_InitTypeDef   NVIC_InitStructure;

__IO uint16_t  RGBLED_Address = 0xAA;//PX4_I2C_OBDEV_LED_ADDR << 1;   
__IO uint32_t  RGBLED_Timeout = RGBLED_LONG_TIMEOUT;  

OS_STK *rgbledTaskStack;

/**
  * @brief  Initializes peripherals used by the I2C RGB LED driver.
  * @param  None
  * @retval None
  */
static void rgbledLowLevelInit( void )
{
    GPIO_InitTypeDef  GPIO_InitStructure; 

    /* RCC Configuration */
    /*I2C Peripheral clock enable */
    RCC_APB1PeriphClockCmd(RGBLED_I2C_CLK, ENABLE);

    /*SDA GPIO clock enable */
    RCC_AHB1PeriphClockCmd(RGBLED_I2C_SCL_GPIO_CLK, ENABLE);

    /*SCL GPIO clock enable */
    RCC_AHB1PeriphClockCmd(RGBLED_I2C_SDA_GPIO_CLK, ENABLE);

    /* Reset I2Cx IP */
    RCC_APB1PeriphResetCmd(RGBLED_I2C_CLK, ENABLE);

    /* Release reset signal of I2Cx IP */
    RCC_APB1PeriphResetCmd(RGBLED_I2C_CLK, DISABLE);

    /*!< GPIO configuration */
    /* Connect PXx to I2C_SCL*/
    GPIO_PinAFConfig(RGBLED_I2C_SCL_GPIO_PORT, RGBLED_I2C_SCL_SOURCE, RGBLED_I2C_SCL_AF);
    
    /* Connect PXx to I2C_SDA*/
    GPIO_PinAFConfig(RGBLED_I2C_SDA_GPIO_PORT, RGBLED_I2C_SDA_SOURCE, RGBLED_I2C_SDA_AF);  


    /* GPIO Configuration */
    /*Configure I2C SCL pin */
    GPIO_InitStructure.GPIO_Pin = RGBLED_I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(RGBLED_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

    /*Configure I2C SDA pin */
    GPIO_InitStructure.GPIO_Pin = RGBLED_I2C_SDA_PIN;
    GPIO_Init(RGBLED_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);

// Ben: TCA62724FMG LED driver support DMA mode ? I tried, but seems don't work. 
#if 0
    /* Configure and enable I2C DMA TX Channel interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RGBLED_I2C_DMA_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = RGBLED_I2C_DMA_PREPRIO;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = RGBLED_I2C_DMA_SUBPRIO;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

#if 0
    /*!< I2C DMA TX channels configuration */
    /* Enable the DMA clock */
    RCC_AHB1PeriphClockCmd(RGBLED_I2C_DMA_CLK, ENABLE);

    /* Clear any pending flag on Rx Stream  */
    DMA_ClearFlag(RGBLED_I2C_DMA_STREAM_TX, 
				RGBLED_TX_DMA_FLAG_FEIF | 
				RGBLED_TX_DMA_FLAG_DMEIF | 
				RGBLED_TX_DMA_FLAG_TEIF | 
	       RGBLED_TX_DMA_FLAG_HTIF | 
			   RGBLED_TX_DMA_FLAG_TCIF);

    /* Disable the RGBLED I2C Tx DMA stream */
    DMA_Cmd(RGBLED_I2C_DMA_STREAM_TX, DISABLE);

    /* Configure the DMA stream for the RGBLED I2C peripheral TX direction */
    DMA_DeInit(RGBLED_I2C_DMA_STREAM_TX);
    rgbledDMA_InitStructure.DMA_Channel = RGBLED_I2C_DMA_CHANNEL;
    rgbledDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)RGBLED_I2C_DR_Address;
    rgbledDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)0;    
    rgbledDMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; 
    rgbledDMA_InitStructure.DMA_BufferSize = 0xFFFF;              
    rgbledDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    rgbledDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    rgbledDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    rgbledDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    rgbledDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    rgbledDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    rgbledDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    rgbledDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    rgbledDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    rgbledDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(RGBLED_I2C_DMA_STREAM_TX, &rgbledDMA_InitStructure);

    /* Enable the DMA Channels Interrupts */
    DMA_ITConfig(RGBLED_I2C_DMA_STREAM_TX, DMA_IT_TC, ENABLE);

#endif

#endif
}

/**
  * @brief  This function handles the DMA Tx Channel interrupt Handler.
  * @param  None
  * @retval None
  */
void RGBLED_I2C_DMA_TX_IRQHandler(void)
{
    /* Check if the DMA transfer is complete */
    if(DMA_GetFlagStatus(RGBLED_I2C_DMA_STREAM_TX, RGBLED_TX_DMA_FLAG_TCIF) != RESET)
    {  
	/* Disable the DMA Tx Stream and Clear TC flag */  
	DMA_Cmd(RGBLED_I2C_DMA_STREAM_TX, DISABLE);
	DMA_ClearFlag(RGBLED_I2C_DMA_STREAM_TX, RGBLED_TX_DMA_FLAG_TCIF);

	/*!< Wait till all data have been physically transferred on the bus */
	RGBLED_Timeout = RGBLED_LONG_TIMEOUT;  
	while(!I2C_GetFlagStatus(RGBLED_I2C, I2C_FLAG_BTF))
	{
	    if((RGBLED_Timeout--) == 0) 
		return;
	}

	/*!< Send STOP condition */
	I2C_GenerateSTOP(RGBLED_I2C, ENABLE);
    }
}


/**
  * @brief  Initializes DMA channel used by the I2C RGB LED driver.
  * @param  None
  * @retval None
  */
static void ledrgbLowLevelDMAConfig(uint32_t pBuffer, uint32_t BufferSize)
{ 
    DMA_InitTypeDef    rgbledDMA_InitStructure;

/* Clear any pending flag on Rx Stream  */
    DMA_ClearFlag( RGBLED_I2C_DMA_STREAM_TX, 
		   RGBLED_TX_DMA_FLAG_FEIF | 
		   RGBLED_TX_DMA_FLAG_DMEIF | 
		   RGBLED_TX_DMA_FLAG_TEIF | 
	           RGBLED_TX_DMA_FLAG_HTIF | 
	           RGBLED_TX_DMA_FLAG_TCIF);
  
    DMA_Cmd(RGBLED_I2C_DMA_STREAM_TX, DISABLE);
    DMA_DeInit(RGBLED_I2C_DMA_STREAM_TX);

    /* Configure the DMA Tx Stream with the buffer address and the buffer size */
#if 0
    rgbledDMA_InitStructure.DMA_Channel = RGBLED_I2C_DMA_CHANNEL;
    rgbledDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)RGBLED_I2C_DR_Address;
    rgbledDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)pBuffer;    
    rgbledDMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; 
    rgbledDMA_InitStructure.DMA_BufferSize = (uint32_t)BufferSize/2;              
    rgbledDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    rgbledDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    rgbledDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    rgbledDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    rgbledDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    rgbledDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    rgbledDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    rgbledDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    rgbledDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    rgbledDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
#endif

    rgbledDMA_InitStructure.DMA_Channel = RGBLED_I2C_DMA_CHANNEL;
    rgbledDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)RGBLED_I2C_DR_Address;
    rgbledDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)pBuffer;
    rgbledDMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    rgbledDMA_InitStructure.DMA_BufferSize = BufferSize;
    rgbledDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    rgbledDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    rgbledDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    rgbledDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    rgbledDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    rgbledDMA_InitStructure.DMA_Priority = DMA_Priority_High;
    rgbledDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    rgbledDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    rgbledDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
    rgbledDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
    
    DMA_Init(RGBLED_I2C_DMA_STREAM_TX, &rgbledDMA_InitStructure);  


    DMA_FlowControllerConfig(RGBLED_I2C_DMA_STREAM_TX, DMA_FlowCtrl_Peripheral);
    DMA_Cmd(RGBLED_I2C_DMA_STREAM_TX, ENABLE);
}

static void transfer( const uint8_t *msg, uint32_t BufferSize )
{
    RGBLED_Timeout = RGBLED_LONG_TIMEOUT;  
    while(I2C_GetFlagStatus(RGBLED_I2C, I2C_FLAG_BUSY))
    {
	if((RGBLED_Timeout--) == 0) 
	    return;
    }

    /*!< Send START condition */
    I2C_GenerateSTART(RGBLED_I2C, ENABLE);

    /*!< Test on EV5 and clear it */
    RGBLED_Timeout = RGBLED_LONG_TIMEOUT;
    while(!I2C_CheckEvent(RGBLED_I2C, I2C_EVENT_MASTER_MODE_SELECT))
    {
	if((RGBLED_Timeout--) == 0) 
	    return;
    }

    /*!< Send RGB LED address for write */
    I2C_Send7bitAddress(RGBLED_I2C, RGBLED_Address, I2C_Direction_Transmitter);

    /*!< Test on EV6 and clear it */
    RGBLED_Timeout = RGBLED_LONG_TIMEOUT;
    while(!I2C_CheckEvent(RGBLED_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
	if((RGBLED_Timeout--) == 0) 
	    return;
    }

#if 0
    //******************************************************************************/
     /*                     Now Ready for DMA or I2C_SendData                      */
     /******************************************************************************/
     /* Enable the RGBLED_I2C peripheral DMA requests */
    I2C_DMACmd(RGBLED_I2C, ENABLE);

    ledrgbLowLevelDMAConfig( (uint32_t)msg, BufferSize );    
#else 

    while( BufferSize > 0 )
    {
	I2C_SendData(RGBLED_I2C, *msg);
   
	/*!< Test on EV8 and clear it */
	RGBLED_Timeout = RGBLED_LONG_TIMEOUT;
	while(!I2C_CheckEvent(RGBLED_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
	    if((RGBLED_Timeout--) == 0) 
		return;
	}

	--BufferSize;
	++msg;
    }

     /*!< STOP condition */    
      I2C_GenerateSTOP(RGBLED_I2C, ENABLE);

#endif
}

/**
 * Sent ENABLE flag to LED driver
 */
static void send_led_enable(BOOL enable)
{
    uint8_t settings_byte = 0;

    if (enable)
	settings_byte |= SETTING_ENABLE;

    settings_byte |= SETTING_NOT_POWERSAVE;

    const uint8_t msg[2] = { SUB_ADDR_SETTINGS, settings_byte};

    transfer(msg, sizeof(msg));
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
static void send_led_rgb()
{
    /* To scale from 0..255 -> 0..15 shift right by 4 bits */
    const uint8_t msg[6] = {
	    SUB_ADDR_PWM0, (uint8_t)((int)(rgbledData._b * rgbledData._brightness) >> 4),
	    SUB_ADDR_PWM1, (uint8_t)((int)(rgbledData._g * rgbledData._brightness) >> 4),
	    SUB_ADDR_PWM2, (uint8_t)((int)(rgbledData._r * rgbledData._brightness) >> 4)
    };
    transfer(msg, sizeof(msg));
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
void rgbledSetRGB(uint8_t r, uint8_t g, uint8_t b )
{
    rgbledData._r = r;
    rgbledData._g = g;
    rgbledData._b = b;

    send_led_rgb();
}

/**
 * Parse color constant and set _r _g _b values
 */
void rbgledSetColor(rgbled_color_t color)
{
    switch (color) {
	case RGBLED_COLOR_OFF:
	    rgbledData._r = 0;
	    rgbledData._g = 0;
	    rgbledData._b = 0;
	    break;

	case RGBLED_COLOR_RED:
	    rgbledData._r = 255;
	    rgbledData._g = 0;
	    rgbledData._b = 0;
	    break;

	case RGBLED_COLOR_YELLOW:
	    rgbledData._r = 255;
	    rgbledData._g = 200;
	    rgbledData._b = 0;
	    break;

	case RGBLED_COLOR_PURPLE:
	    rgbledData._r = 255;
	    rgbledData._g = 0;
	    rgbledData._b = 255;
	    break;

	case RGBLED_COLOR_GREEN:
	    rgbledData._r = 0;
	    rgbledData._g = 255;
	    rgbledData._b = 0;
	    break;

	case RGBLED_COLOR_BLUE:
	    rgbledData._r = 0;
	    rgbledData._g = 0;
	    rgbledData._b = 255;
	    break;

	case RGBLED_COLOR_WHITE:
	    rgbledData._r = 255;
	    rgbledData._g = 255;
	    rgbledData._b = 255;
	    break;

	case RGBLED_COLOR_AMBER:
	    rgbledData._r = 255;
	    rgbledData._g = 80;
	    rgbledData._b = 0;
	    break;

	case RGBLED_COLOR_DIM_RED:
	    rgbledData._r = 90;
	    rgbledData._g = 0;
	    rgbledData._b = 0;
	    break;

	case RGBLED_COLOR_DIM_YELLOW:
	    rgbledData._r = 80;
	    rgbledData._g = 30;
	    rgbledData._b = 0;
	    break;

	case RGBLED_COLOR_DIM_PURPLE:
	    rgbledData._r = 45;
	    rgbledData._g = 0;
	    rgbledData._b = 45;
	    break;

	case RGBLED_COLOR_DIM_GREEN:
	    rgbledData._r = 0;
	    rgbledData._g = 90;
	    rgbledData._b = 0;
	    break;

	case RGBLED_COLOR_DIM_BLUE:
	    rgbledData._r = 0;
	    rgbledData._g = 0;
	    rgbledData._b = 90;
	    break;

	case RGBLED_COLOR_DIM_WHITE:
	    rgbledData._r = 30;
	    rgbledData._g = 30;
	    rgbledData._b = 30;
	    break;

	case RGBLED_COLOR_DIM_AMBER:
	    rgbledData._r = 80;
	    rgbledData._g = 20;
	    rgbledData._b = 0;
	    break;

	default:
	    break;
    }

    send_led_rgb();
}

/**
 * Set mode, if mode not changed has no any effect (doesn't reset blinks phase)
 */
void rgbledSetMode(rgbled_mode_t mode)
{
    if (mode != rgbledData._mode) {
	rgbledData._mode = mode;

	switch (mode) {
	case RGBLED_MODE_OFF:
	    rgbledData._should_run = false;
	    send_led_enable(false);
	    break;

	case RGBLED_MODE_ON:
	    rgbledData._brightness = 1.0f;
	    send_led_rgb();
	    send_led_enable(true);
	    break;

	case RGBLED_MODE_BLINK_SLOW:
	    rgbledData._should_run = true;
	    rgbledData._counter = 0;
	    rgbledData._led_interval = 2000;
	    rgbledData._brightness = 1.0f;
	    send_led_rgb();
	    break;

	case RGBLED_MODE_BLINK_NORMAL:
	    rgbledData._should_run = true;
	    rgbledData._counter = 0;
	    rgbledData._led_interval = 500;
	    rgbledData._brightness = 1.0f;
	    send_led_rgb();
	    break;

	case RGBLED_MODE_BLINK_FAST:
	    rgbledData._should_run = true;
	    rgbledData._counter = 0;
	    rgbledData._led_interval = 100;
	    rgbledData._brightness = 1.0f;
	    send_led_rgb();
	    break;

	case RGBLED_MODE_BREATHE:
	    rgbledData._should_run = true;
	    rgbledData._counter = 0;
	    rgbledData._led_interval = 25;
	    send_led_enable(true);
	    break;

	case RGBLED_MODE_PATTERN:
	    rgbledData._should_run = true;
	    rgbledData._counter = 0;
	    rgbledData._brightness = 1.0f;
	    send_led_enable(true);
	    break;

	default:
	    break;
	}
    }
}

/**
 * Set pattern for PATTERN mode, but don't change current mode
 */
void set_pattern(rgbled_pattern_t *pattern)
{
    memcpy(&rgbledData._pattern, pattern, sizeof(rgbled_pattern_t));
}


/**
 * Main loop function
 */
void rgbledTaskCode()
{
    while(1)
    {
	if(!rgbledData._should_run) {
	    rgbledData._running = false;
	    yield( 100 );
	    continue;
	}

	switch (rgbledData._mode) {
	case RGBLED_MODE_BLINK_SLOW:
	case RGBLED_MODE_BLINK_NORMAL:
	case RGBLED_MODE_BLINK_FAST:
	    if (rgbledData._counter >= 2)
		rgbledData._counter = 0;

	    send_led_enable(rgbledData._counter == 0);

	    break;

	case RGBLED_MODE_BREATHE:

	    if (rgbledData._counter >= 62)
		rgbledData._counter = 0;

	    int n;

	    if (rgbledData._counter < 32) {
		n = rgbledData._counter;
	    } else {
		n = 62 - rgbledData._counter;
	    }

	    rgbledData._brightness = n * n / (31.0f * 31.0f);
	    send_led_rgb();
	    break;

	case RGBLED_MODE_PATTERN:

	    /* don't run out of the pattern array and stop if the next frame is 0 */
	    if (rgbledData._counter >= RGBLED_PATTERN_LENGTH || rgbledData._pattern.duration[rgbledData._counter] <= 0)
		rgbledData._counter = 0;

	    rbgledSetColor(rgbledData._pattern.color[rgbledData._counter]);
	    send_led_rgb();
	    rgbledData._led_interval = rgbledData._pattern.duration[rgbledData._counter];
	    break;

	default:
	    break;
	}

	rgbledData._counter++;

	yield( rgbledData._led_interval );
    }
}



void rgbledInit( void )
{
    I2C_InitTypeDef  I2C_InitStructure;

    memset( &rgbledData, 0, sizeof(rgbledData) );
    //memset( &rgbledDMA_InitStructure, 0, sizeof(rgbledDMA_InitStructure) );
    memset( &NVIC_InitStructure, 0, sizeof(NVIC_InitStructure) );
    rgbledLowLevelInit();

    
    /*!< I2C configuration */
    /* sEE_I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0;//0x1a << 1;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;

    /* sEE_I2C Peripheral Enable */
    I2C_Cmd(RGBLED_I2C, ENABLE);
    /* Apply RGBLED_I2C configuration after enabling it */
    I2C_Init(RGBLED_I2C, &I2C_InitStructure);


    // Start LED Driver task
    rgbledTaskStack = aqStackInit(RGBLED_STACK_SIZE, "RGBLED");
    CoCreateTask(rgbledTaskCode, (void *)0, RGBLED_PRIORITY, &rgbledTaskStack[RGBLED_STACK_SIZE-1], RGBLED_STACK_SIZE);

    //rbgledSetColor(RGBLED_COLOR_RED);
    //rgbledSetMode(RGBLED_MODE_BLINK_NORMAL);

    rgbled_pattern_t ledPattern = 
    {
	{RGBLED_COLOR_RED, RGBLED_COLOR_YELLOW, RGBLED_COLOR_PURPLE, RGBLED_COLOR_GREEN, RGBLED_COLOR_BLUE, RGBLED_COLOR_DIM_AMBER, 0},
	{500,              500,                 500,                 500,                500,               500,                    0}
    };
    set_pattern( &ledPattern );
    rgbledSetMode(RGBLED_MODE_PATTERN);

}

