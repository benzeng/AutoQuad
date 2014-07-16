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
    This module handles the coordination of multiple SPI clients which may be
    sharing a SPI bus.  Once a transfer request has been queued, the caller
    will be notified via a flag or callback at their option when complete.
    Requests are handled on a FIFO basis.  It is up to the client(s) to make sure
    that they do not call spiTransaction() concurrently as it is not thread safe.
    This can easily be handled if clients all submit transaction requests via
    data-ready ISR's of the same priority and preemption level.

    The Ethernet periphreal's interrupt lines are stolen for the SPI scheduler.
    If Ethernet is one day needed, this must be changed.
*/

#include "spi.h"
#include "aq_timer.h"
#include "util.h"
#include "digital.h"

spiStruct_t spiData[3];

static void spiTriggerSchedule(uint8_t interface) {
    switch (interface) {
	case 0:
	    NVIC->STIR = ETH_IRQn;
	    break;
	case 1:
	    NVIC->STIR = ETH_WKUP_IRQn;
	    break;
	case 2:
	    NVIC->STIR = DCMI_IRQn;
	    break;
    }
}

#ifdef SPI_SPI1_CLOCK
void spi1Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    if (!spiData[0].initialized) {
	// SPI interface
	RCC_APB2PeriphClockCmd(SPI_SPI1_CLOCK, ENABLE);

        /*!< Enable GPIO clocks */
	// Ben+ : ToDo: Test this line !
	RCC_AHB1PeriphClockCmd( SPI_SPI1_GPIO_CLOCK | PX4_BARO_SPI_CS_CLK , ENABLE );

        // Connect SPI pins to Alternate Function
	GPIO_PinAFConfig(SPI_SPI1_SCK_PORT,SPI_SPI1_SCK_SOURCE, SPI_SPI1_AF);
	GPIO_PinAFConfig(SPI_SPI1_MISO_PORT, SPI_SPI1_MISO_SOURCE, SPI_SPI1_AF);
	GPIO_PinAFConfig(SPI_SPI1_MOSI_PORT, SPI_SPI1_MOSI_SOURCE, SPI_SPI1_AF);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

	// SPI SCK / MOSI / MISO pin configuration
	GPIO_InitStructure.GPIO_Pin = SPI_SPI1_SCK_PIN;
	GPIO_Init(SPI_SPI1_SCK_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_SPI1_MISO_PIN;
	GPIO_Init(SPI_SPI1_MISO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_SPI1_MOSI_PIN;
	GPIO_Init(SPI_SPI1_MOSI_PORT, &GPIO_InitStructure);

        /*!< Configure MS5611 CS pin in output pushpull mode ********************/
	GPIO_InitStructure.GPIO_Pin = PX4_BARO_SPI_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(PX4_BARO_SPI_CS_PORT, &GPIO_InitStructure);


	// SPI configuration
	SPI_I2S_DeInit(SPI1);
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;// SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

	// RX DMA
	DMA_DeInit(SPI_SPI1_DMA_RX);
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = SPI_SPI1_DMA_RX_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)spiData;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;		    // note the buffer must be word aligned
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(SPI_SPI1_DMA_RX, &DMA_InitStructure);

	// store flags for later use
	spiData[0].intRxFlags = SPI_SPI1_DMA_RX_FLAGS;

	DMA_ClearITPendingBit(SPI_SPI1_DMA_RX, spiData[0].intRxFlags);
	DMA_ITConfig(SPI_SPI1_DMA_RX, DMA_IT_TC, ENABLE);

	// Enable RX DMA global Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = SPI_SPI1_DMA_RX_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// TX DMA - one shot
	DMA_DeInit(SPI_SPI1_DMA_TX);
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = SPI_SPI1_DMA_TX_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;		    // note the buffer must be word aligned
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(SPI_SPI1_DMA_TX, &DMA_InitStructure);

	// store flags for later use
	spiData[0].intTxFlags = SPI_SPI1_DMA_TX_FLAGS;

	DMA_ClearITPendingBit(SPI_SPI1_DMA_TX, spiData[0].intTxFlags);

	spiData[0].spi = SPI1;
	spiData[0].rxDMAStream = SPI_SPI1_DMA_RX;
	spiData[0].txDMAStream = SPI_SPI1_DMA_TX;
	spiData[0].initialized = 1;

	// Enable Ethernet Interrupt (for our stack management)
	NVIC_InitStructure.NVIC_IRQChannel = ETH_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Ben+
        SPI_Cmd(SPI1, ENABLE);
    }
}
#endif

#ifdef SPI_SPI2_CLOCK
void spi2Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    if (!spiData[1].initialized) {
	// SPI interface
	RCC_APB1PeriphClockCmd(SPI_SPI2_CLOCK, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

	// SPI SCK / MOSI / MISO pin configuration
	GPIO_InitStructure.GPIO_Pin = SPI_SPI2_SCK_PIN;
	GPIO_Init(SPI_SPI2_SCK_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_SPI2_MISO_PIN;
	GPIO_Init(SPI_SPI2_MISO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_SPI2_MOSI_PIN;
	GPIO_Init(SPI_SPI2_MOSI_PORT, &GPIO_InitStructure);

	// Connect SPI pins to AF6
	GPIO_PinAFConfig(SPI_SPI2_SCK_PORT,SPI_SPI2_SCK_SOURCE, SPI_SPI2_AF);
	GPIO_PinAFConfig(SPI_SPI2_MISO_PORT, SPI_SPI2_MISO_SOURCE, SPI_SPI2_AF);
	GPIO_PinAFConfig(SPI_SPI2_MOSI_PORT, SPI_SPI2_MOSI_SOURCE, SPI_SPI2_AF);

	// SPI configuration
	SPI_I2S_DeInit(SPI2);
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);

	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

	// RX DMA
	DMA_DeInit(SPI_SPI2_DMA_RX);
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = SPI_SPI2_DMA_RX_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)spiData;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;		    // note the buffer must be word aligned
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(SPI_SPI2_DMA_RX, &DMA_InitStructure);

	// store flags for later use
	spiData[1].intRxFlags = SPI_SPI2_DMA_RX_FLAGS;

	DMA_ClearITPendingBit(SPI_SPI2_DMA_RX, spiData[1].intRxFlags);
	DMA_ITConfig(SPI_SPI2_DMA_RX, DMA_IT_TC, ENABLE);

	// Enable RX DMA global Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = SPI_SPI2_DMA_RX_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// TX DMA - one shot
	DMA_DeInit(SPI_SPI2_DMA_TX);
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = SPI_SPI2_DMA_TX_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;		    // note the buffer must be word aligned
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(SPI_SPI2_DMA_TX, &DMA_InitStructure);

	// store flags for later use
	spiData[1].intTxFlags = SPI_SPI2_DMA_TX_FLAGS;
	DMA_ClearITPendingBit(SPI_SPI2_DMA_TX, spiData[1].intTxFlags);

	spiData[1].spi = SPI2;
	spiData[1].rxDMAStream = SPI_SPI2_DMA_RX;
	spiData[1].txDMAStream = SPI_SPI2_DMA_TX;
	spiData[1].initialized = 1;

	// Enable Ethernet Interrupt (for our stack management)
	NVIC_InitStructure.NVIC_IRQChannel = ETH_WKUP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    }
}
#endif

#ifdef SPI_SPI3_CLOCK
void spi3Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    if (!spiData[2].initialized) {
	// SPI interface
	RCC_APB1PeriphClockCmd(SPI_SPI3_CLOCK, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

	// SPI SCK / MOSI / MISO pin configuration
	GPIO_InitStructure.GPIO_Pin = SPI_SPI3_SCK_PIN;
	GPIO_Init(SPI_SPI3_SCK_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_SPI3_MISO_PIN;
	GPIO_Init(SPI_SPI3_MISO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_SPI3_MOSI_PIN;
	GPIO_Init(SPI_SPI3_MOSI_PORT, &GPIO_InitStructure);

	// Connect SPI pins to AF6
	GPIO_PinAFConfig(SPI_SPI3_SCK_PORT, SPI_SPI3_SCK_SOURCE, SPI_SPI3_AF);
	GPIO_PinAFConfig(SPI_SPI3_MISO_PORT, SPI_SPI3_MISO_SOURCE, SPI_SPI3_AF);
	GPIO_PinAFConfig(SPI_SPI3_MOSI_PORT, SPI_SPI3_MOSI_SOURCE, SPI_SPI3_AF);

	// SPI configuration
	SPI_I2S_DeInit(SPI3);
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI3, &SPI_InitStructure);

	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

	// RX DMA
	DMA_DeInit(SPI_SPI3_DMA_RX);
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = SPI_SPI3_DMA_RX_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI3->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)spiData;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;		    // note the buffer must be word aligned
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(SPI_SPI3_DMA_RX, &DMA_InitStructure);

	// store flags for later use
	spiData[2].intRxFlags = SPI_SPI3_DMA_RX_FLAGS;

	DMA_ClearITPendingBit(SPI_SPI3_DMA_RX, spiData[2].intRxFlags);
	DMA_ITConfig(SPI_SPI3_DMA_RX, DMA_IT_TC, ENABLE);

	// Enable RX DMA global Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = SPI_SPI3_DMA_RX_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// TX DMA - one shot
	DMA_DeInit(SPI_SPI3_DMA_TX);
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = SPI_SPI3_DMA_TX_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI3->DR;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;		    // note the buffer must be word aligned
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(SPI_SPI3_DMA_TX, &DMA_InitStructure);

	// store flags for later use
	spiData[2].intTxFlags = SPI_SPI3_DMA_TX_FLAGS;
	DMA_ClearITPendingBit(SPI_SPI3_DMA_TX, spiData[2].intTxFlags);

	spiData[2].spi = SPI3;
	spiData[2].rxDMAStream = SPI_SPI3_DMA_RX;
	spiData[2].txDMAStream = SPI_SPI3_DMA_TX;
	spiData[2].initialized = 1;

	// Enable Ethernet Interrupt (for our stack management)
	NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    }
}
#endif

static void spiDeselect(spiClient_t *client) {
    digitalHi(client->cs);
}

static void spiSelect(spiClient_t *client) {
    digitalLo(client->cs);
}

static void spiNotify(spiClient_t *client) {
    if (client->flag)
	*client->flag = timerMicros();

    if (client->callback)
	client->callback(0);
}

static void spiDisableSPI(spiStruct_t *interface) {
    while (!(interface->spi->SR & SPI_I2S_FLAG_TXE))
	;
    while (interface->spi->SR & SPI_I2S_FLAG_BSY)
	;

    SPI_Cmd(interface->spi, DISABLE);
}

static void spiDisableDMA(spiStruct_t *interface) {
    DMA_Cmd(interface->txDMAStream, DISABLE);
    DMA_Cmd(interface->rxDMAStream, DISABLE);

    DMA_ClearITPendingBit(interface->rxDMAStream, interface->intRxFlags);
    DMA_ClearITPendingBit(interface->txDMAStream, interface->intTxFlags);
}

static void spiStartTxn(spiStruct_t *interface) {
    spiSlot_t *slot = &interface->slots[interface->tail];
    spiClient_t *client = slot->client;
    uint32_t tmp;

    // is the current txn taking too long?
    if (interface->txRunning != 0 && (timerMicros() - interface->txnStart) > SPI_MAX_TXN_TIME) {
	spiDisableDMA(interface);
	spiDisableSPI(interface);
	spiDeselect(client);

	interface->tail = (interface->tail + 1) % SPI_SLOTS;

	slot = &interface->slots[interface->tail];
	client = slot->client;

	interface->txRunning = 0;
	interface->txnTimeouts++;
    }

    if (interface->tail != interface->head && interface->txRunning == 0) {
	interface->txRunning = 1;
	interface->txnStart = timerMicros();

	// set baud rate
	tmp = interface->spi->CR1 & SPI_BAUD_MASK;
	tmp |= client->baud;
	interface->spi->CR1 = tmp;

	// clear DR
	SPI_I2S_ReceiveData(interface->spi);

	spiSelect(client);

	// specify "in transaction"
	if (client->flag)
	    *client->flag = 0;

	interface->rxDMAStream->M0AR = slot->rxBuf;
	interface->rxDMAStream->NDTR = slot->size;
	DMA_Cmd(interface->rxDMAStream, ENABLE);

	interface->txDMAStream->M0AR = slot->txBuf;
	interface->txDMAStream->NDTR = slot->size;
	DMA_Cmd(interface->txDMAStream, ENABLE);

	SPI_Cmd(interface->spi, ENABLE);
    }
}

static void spiEndTxn(spiStruct_t *interface) {
    uint8_t tail = interface->tail;
    spiClient_t *client = interface->slots[tail].client;
    uint32_t tmp;

    spiDisableSPI(interface);
    spiDisableDMA(interface);

    // record longest txn
    tmp = timerMicros() - interface->txnStart;
    if (tmp > interface->txnMaxTime)
	interface->txnMaxTime = tmp;

    spiNotify(client);

    tail = (tail + 1) % SPI_SLOTS;
    interface->tail = tail;

    spiDeselect(client);

    interface->txRunning = 0;

    //spiTriggerSchedule(client->interface);
}

void spiChangeBaud(spiClient_t *client, uint16_t baud) {

#ifndef PX4FMU

    // SPI1 runs at twice the speed of SPI2
    if (client->interface == 0) {
	if (baud == SPI_BaudRatePrescaler_2)
	    baud = SPI_BaudRatePrescaler_4;
	else if (baud == SPI_BaudRatePrescaler_4)
	    baud = SPI_BaudRatePrescaler_8;
	else if (baud == SPI_BaudRatePrescaler_8)
	    baud = SPI_BaudRatePrescaler_16;
	else if (baud == SPI_BaudRatePrescaler_16)
	    baud = SPI_BaudRatePrescaler_32;
	else if (baud == SPI_BaudRatePrescaler_32)
	    baud = SPI_BaudRatePrescaler_64;
	else if (baud == SPI_BaudRatePrescaler_64)
	    baud = SPI_BaudRatePrescaler_128;
	else if (baud == SPI_BaudRatePrescaler_128)
	    baud = SPI_BaudRatePrescaler_256;
    }

#endif

    client->baud = baud;
}

void spiChangeCallback(spiClient_t *client, spiCallback_t *callback) {
    client->callback = callback;
}

// TODO: not yet thread safe
void spiTransaction(spiClient_t *client, volatile void *rxBuf, void *txBuf, uint16_t size) {
    spiStruct_t *interface = &spiData[client->interface];
    uint8_t head = interface->head;
    spiSlot_t *slot = &interface->slots[head];

    slot->size = size;
    slot->rxBuf = (uint32_t)rxBuf;
    slot->txBuf = (uint32_t)txBuf;
    slot->client = client;

    interface->head = (head + 1) % SPI_SLOTS;

    spiTriggerSchedule(client->interface);
}

void spi1Transaction(spiClient_t *client, volatile void *rxBuf, void *txBuf, uint16_t size) {
    spiStruct_t *interface = &spiData[client->interface];
    uint8_t head = interface->head;
    spiSlot_t *slot = &interface->slots[head];

    slot->size = size;
    slot->rxBuf = (uint32_t)rxBuf;
    slot->txBuf = (uint32_t)txBuf;
    slot->client = client;

    interface->head = (head + 1) % SPI_SLOTS;

    
    spiStartTxn(&spiData[0]);
}



/**
  * @brief  Sends a byte through the SPI interface and return the byte received
  *         from the SPI bus.
  * @param  byte: byte to send.
  * @retval none
  */
uint8_t spiExchangeByte(spiStruct_t *interface, uint8_t byte)
{
    static int n = 0;

  /*!< Loop while DR register in not emplty   */
  while (SPI_I2S_GetFlagStatus( interface->spi, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(interface->spi, byte);

if( n == 0 )
{
    ++n;
//    return 0;
}

  /*!< Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(interface->spi, SPI_I2S_FLAG_RXNE) == RESET)
    delay(5);

  return SPI_I2S_ReceiveData( interface->spi );
}

void spiReadBuffer(spiClient_t *client, volatile void *rxBuf, void *txBuf, uint16_t rxSize, uint16_t txSize) {
    spiStruct_t *interface = &spiData[client->interface];
    char* pTx = (char*)txBuf;
    char* pRx = (char*)rxBuf;

    spiSelect( client );
    
    // Send data first
    while ( txSize-- > 1 ) 
    {
	spiExchangeByte( interface, *pTx );
	pTx++;
    }
    *pRx = spiExchangeByte( interface, *pTx );
    ++pRx;
    --rxSize;

    while( rxSize-- > 0 )
    {
	// dummy value: 0xA5
	*pRx = spiExchangeByte( interface, 0xA5 );
	++pRx;
    }

    spiDeselect( client );
}




spiClient_t *spiClientInit(SPI_TypeDef *spi, uint16_t baud, GPIO_TypeDef *csPort, uint16_t csPin, volatile uint32_t *flag, spiCallback_t *callback) {
    spiClient_t *client;

    client = (spiClient_t *)aqCalloc(1, sizeof(spiClient_t));

#ifdef SPI_SPI1_CLOCK
    if (spi == SPI1) {
	client->interface = 0;
	spi1Init();
    }
#endif
#ifdef SPI_SPI2_CLOCK
    if (spi == SPI2) {
	client->interface = 1;
	spi2Init();
    }
#endif
#ifdef SPI_SPI3_CLOCK
    if (spi == SPI3) {
	client->interface = 2;
	spi3Init();
    }
#endif

    client->cs = digitalInit(csPort, csPin);
    spiDeselect(client);

    spiChangeBaud(client, baud);
    client->flag = flag;
    client->callback = callback;

    return client;
}

#ifdef SPI_SPI1_CLOCK
void SPI_SPI1_DMA_RX_HANDLER(void) {
    // finish transaction
    spiEndTxn(&spiData[0]);
}
#endif

#ifdef SPI_SPI2_CLOCK
void SPI_SPI2_DMA_RX_HANDLER(void) {
    // finish transaction
    spiEndTxn(&spiData[1]);
}
#endif

#ifdef SPI_SPI3_CLOCK
void SPI_SPI3_DMA_RX_HANDLER(void) {
    // finish transaction
    spiEndTxn(&spiData[2]);
}
#endif


//#define SPI_CR1_CPHA              (1 << 0)  /* Bit 0: Clock Phase */
//#define SPI_CR1_CPOL              (1 << 1)  /* Bit 1: Clock Polarity */
#define SPI_CR1_BR_SHIFT          (3)       /* Bits 5:3 Baud Rate Control */
#define SPI_CR1_BR_MASK           (7 << SPI_CR1_BR_SHIFT)
#  define SPI_CR1_FPCLCKd2        (0 << SPI_CR1_BR_SHIFT) /* 000: fPCLK/2 */
#  define SPI_CR1_FPCLCKd4        (1 << SPI_CR1_BR_SHIFT) /* 001: fPCLK/4 */
#  define SPI_CR1_FPCLCKd8        (2 << SPI_CR1_BR_SHIFT) /* 010: fPCLK/8 */
#  define SPI_CR1_FPCLCKd16       (3 << SPI_CR1_BR_SHIFT) /* 011: fPCLK/16 */
#  define SPI_CR1_FPCLCKd32       (4 << SPI_CR1_BR_SHIFT) /* 100: fPCLK/32 */
#  define SPI_CR1_FPCLCKd64       (5 << SPI_CR1_BR_SHIFT) /* 101: fPCLK/64 */
#  define SPI_CR1_FPCLCKd128      (6 << SPI_CR1_BR_SHIFT) /* 110: fPCLK/128 */
#  define SPI_CR1_FPCLCKd256      (7 << SPI_CR1_BR_SHIFT) /* 111: fPCLK/256 */

// co-op the Ethernet IRQ for our purposes
void ETH_IRQHandler(void) {

#if 0
    // PX4FMU
    uint32_t tmp;
    tmp = spiData[0].spi->CR1;
    tmp |= SPI_CR1_CPHA | SPI_CR1_CPOL;

    //tmp &= ~SPI_CR1_BR_MASK;
    //tmp |= SPI_CR1_FPCLCKd2;

    spiData[0].spi->CR1 = tmp;
#endif

    spiStartTxn(&spiData[0]);
}

void ETH_WKUP_IRQHandler(void) {
    spiStartTxn(&spiData[1]);
}

void DCMI_IRQHandler(void) {
    spiStartTxn(&spiData[2]);
}
