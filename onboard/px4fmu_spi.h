/*
    This file is part of AutoQuad For PX4FMU(Pixhawk)

    Ported from \nuttx\include\nuttx\spi.h

    Copyright (C) 2014  BenZeng
*/

#ifndef __INCLUDE_PX4FMU_SPI_H
#define __INCLUDE_PX4FMU_SPI_H

#include "px4fmu_chip.h"
#include "px4fmu_rcc.h"
#include "px4fmu_gpio.h"

// Setup SPI1,SPI2
#define CONFIG_STM32_SPI1
//#define CONFIG_STM32_SPI2

/*
#define STM32_SYSCLK_FREQUENCY  168000000ul


#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  // same as above, to satisfy compiler

// APB1 clock (PCLK1) is HCLK/4 (42MHz) 
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

// APB2 clock (PCLK2) is HCLK/2 (84MHz) 
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)
*/

#define STM32_SPI1_BASE 0x40013000 
#define STM32_SPI2_BASE 0x40003800 

/* Maximum allowed speed as per specifications for all SPIs */
#  define STM32_SPI_CLK_MAX     18000000UL

/* Register Bitfield Definitions ****************************************************/

/* SPI Control Register 1 */
#ifndef SPI_CR1_CPHA
#define SPI_CR1_CPHA              (1 << 0)  /* Bit 0: Clock Phase */
#define SPI_CR1_CPOL              (1 << 1)  /* Bit 1: Clock Polarity */
#define SPI_CR1_MSTR              (1 << 2)  /* Bit 2: Master Selection */
#endif
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
#ifndef SPI_CR1_SPE
#define SPI_CR1_SPE               (1 << 6)  /* Bit 6: SPI Enable */
#define SPI_CR1_LSBFIRST          (1 << 7)  /* Bit 7: Frame Format */
#define SPI_CR1_SSI               (1 << 8)  /* Bit 8: Internal slave select */
#define SPI_CR1_SSM               (1 << 9)  /* Bit 9: Software slave management */
#define SPI_CR1_RXONLY            (1 << 10) /* Bit 10: Receive only */
#define SPI_CR1_DFF               (1 << 11) /* Bit 11: Data Frame Format */
#define SPI_CR1_CRCNEXT           (1 << 12) /* Bit 12: Transmit CRC next */
#define SPI_CR1_CRCEN             (1 << 13) /* Bit 13: Hardware CRC calculation enable */
#define SPI_CR1_BIDIOE            (1 << 14) /* Bit 14: Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE          (1 << 15) /* Bit 15: Bidirectional data mode enable */
#endif

/* SPI Control Register 2 */
#ifndef SPI_CR2_RXDMAEN
#define SPI_CR2_RXDMAEN           (1 << 0)  /* Bit 0: Rx Buffer DMA Enable */
#define SPI_CR2_TXDMAEN           (1 << 1)  /* Bit 1: Tx Buffer DMA Enable */
#define SPI_CR2_SSOE              (1 << 2)  /* Bit 2: SS Output Enable */

#define SPI_CR2_ERRIE             (1 << 5)  /* Bit 5: Error interrupt enable */
#define SPI_CR2_RXNEIE            (1 << 6)  /* Bit 6: RX buffer not empty interrupt enable */
#define SPI_CR2_TXEIE             (1 << 7)  /* Bit 7: Tx buffer empty interrupt enable */
#endif


/* Register Offsets *****************************************************************/
#define STM32_SPI_CR1_OFFSET       0x0000  /* SPI Control Register 1 (16-bit) */
#define STM32_SPI_CR2_OFFSET       0x0004  /* SPI control register 2 (16-bit) */
#define STM32_SPI_SR_OFFSET        0x0008  /* SPI status register (16-bit) */
#define STM32_SPI_DR_OFFSET        0x000c  /* SPI data register (16-bit) */
#define STM32_SPI_CRCPR_OFFSET     0x0010  /* SPI CRC polynomial register (16-bit) */
#define STM32_SPI_RXCRCR_OFFSET    0x0014  /* SPI Rx CRC register (16-bit) */
#define STM32_SPI_TXCRCR_OFFSET    0x0018  /* SPI Tx CRC register (16-bit) */




/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_SPI_OWNBUS - Set if there is only one active device on the SPI bus.
 *   No locking or SPI configuration will be performed. It is not necessary
 *   for clients to lock, re-configure, etc..
 * CONFIG_SPI_EXCHANGE - Driver supports a single exchange method
 *   (vs a recvblock() and sndblock ()methods).
 * CONFIG_SPI_CMDDATA - Devices on the SPI bus require out-of-band support
 *   to distinguish command transfers from data transfers.  Such devices
 *   will often support either 9-bit SPI (yech) or 8-bit SPI and a GPIO
 *   output that selects between command and data.
 */

/* Access macros ************************************************************/

/****************************************************************************
 * Name: SPI_LOCK
 *
 * Description:
 *   On SPI busses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
#  define SPI_LOCK(d,l) (d)->ops->lock(d,l)
#else
#  define SPI_LOCK(d,l) 0
#endif

/****************************************************************************
 * Name: SPI_SELECT
 *
 * Description:
 *   Enable/disable the SPI chip select.   The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselected.
 *   Required.
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   devid -    Identifies the device to select
 *   selected - true: slave selected, false: slave de-selected
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define SPI_SELECT(d,id,s) ((d)->ops->select(d,id,s))

/****************************************************************************
 * Name: SPI_SETFREQUENCY
 *
 * Description:
 *   Set the SPI frequency. Required.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

#define SPI_SETFREQUENCY(d,f) ((d)->ops->setfrequency(d,f))

/****************************************************************************
 * Name: SPI_SETMODE
 *
 * Description:
 *   Set the SPI mode. Optional.  See enum spi_mode_e for mode definitions.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

#define SPI_SETMODE(d,m) \
  do { if ((d)->ops->setmode) (d)->ops->setmode(d,m); } while (0)

/****************************************************************************
 * Name: SPI_SETBITS
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests.
 *           If value is greater > 0 then it implies MSB first
 *           If value is below < 0, then it implies LSB first with -nbits
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

#define SPI_SETBITS(d,b) \
  do { if ((d)->ops->setbits) (d)->ops->setbits(d,b); } while (0)

/****************************************************************************
 * Name: SPI_STATUS
 *
 * Description:
 *   Get SPI/MMC status.  Optional.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   devid - Identifies the device to report status on
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines)
 *
 ****************************************************************************/

#define SPI_STATUS(d,id) \
  ((d)->ops->status ? (d)->ops->status(d, id) : SPI_STATUS_PRESENT)

/* SPI status bits -- Some dedicated for SPI MMC/SD support and may have no
 * relationship to SPI other than needed by the SPI MMC/SD interface
 */

#define SPI_STATUS_PRESENT     0x01 /* Bit 0=1: MMC/SD card present */
#define SPI_STATUS_WRPROTECTED 0x02 /* Bit 1=1: MMC/SD card write protected */

/****************************************************************************
 * Name: SPI_CMDDATA
 *
 * Description:
 *   Some devices require and additional out-of-band bit to specify if the
 *   next word sent to the device is a command or data. This is typical, for
 *   example, in "9-bit" displays where the 9th bit is the CMD/DATA bit.
 *   This function provides selection of command or data.
 *
 *   This "latches" the CMD/DATA state.  It does not have to be called before
 *   every word is transferred; only when the CMD/DATA state changes.  This
 *   method is required if CONFIG_SPI_CMDDATA is selected in the NuttX
 *   configuration
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   cmd - TRUE: The following word is a command; FALSE: the following words
 *         are data.
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#  define SPI_CMDDATA(d,id,cmd) ((d)->ops->cmddata(d,id,cmd))
#endif

/****************************************************************************
 * Name: SPI_SEND
 *
 * Description:
 *   Exchange one word on SPI. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   Received value
 *
 ****************************************************************************/

#define SPI_SEND(d,wd) ((d)->ops->send(d,(uint16_t)wd))

/****************************************************************************
 * Name: SPI_SNDBLOCK
 *
 * Description:
 *   Send a block of data on SPI. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nwords - the length of data to send from the buffer in number of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_EXCHANGE
#  define SPI_SNDBLOCK(d,b,l) ((d)->ops->exchange(d,b,0,l))
#else
#  define SPI_SNDBLOCK(d,b,l) ((d)->ops->sndblock(d,b,l))
#endif

/****************************************************************************
 * Name: SPI_RECVBLOCK
 *
 * Description:
 *   Receive a block of data from SPI. Required.
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to recieve data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-
 *            per-word selected for the SPI interface.  If nbits <= 8, the
 *            data is packed into uint8_t's; if nbits >8, the data is packed
 *            into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_EXCHANGE
#  define SPI_RECVBLOCK(d,b,l) ((d)->ops->exchange(d,0,b,l))
#else
#  define SPI_RECVBLOCK(d,b,l) ((d)->ops->recvblock(d,b,l))
#endif

/****************************************************************************
 * Name: SPI_EXCHANGE
 *
 * Description:
 *   Exahange a block of data from SPI. Required.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to recieve data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

//#ifdef CONFIG_SPI_EXCHANGE
#  define SPI_EXCHANGE(d,t,r,l) ((d)->ops->exchange(d,t,r,l))
//#endif

/****************************************************************************
 * Name: SPI_REGISTERCALLBACK
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change (i.e, anything that would be reported differently by SPI_STATUS).
 *   Optional
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The funtion to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

#define SPI_REGISTERCALLBACK(d,c,a) \
  ((d)->ops->registercallback ? (d)->ops->registercallback(d,c,a) : -ENOSYS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The type of the media change callback function */

typedef void (*spi_mediachange_t)( void *arg);

/* If the board supports multiple SPI devices, this enumeration identifies
 * which is selected or de-seleted.
 */

enum spi_dev_e
{
  SPIDEV_NONE = 0,    /* Not a valid value */
  SPIDEV_MMCSD,       /* Select SPI MMC/SD device */
  SPIDEV_FLASH,       /* Select SPI FLASH device */
  SPIDEV_ETHERNET,    /* Select SPI ethernet device */
  SPIDEV_DISPLAY,     /* Select SPI LCD/OLED display device */
  SPIDEV_WIRELESS,    /* Select SPI Wireless device */
  SPIDEV_TOUCHSCREEN, /* Select SPI touchscreen device */
  SPIDEV_EXPANDER,    /* Select SPI I/O expander device */
  SPIDEV_MUX,         /* Select SPI multiplexer device */
  SPIDEV_AUDIO_DATA,  /* Select SPI audio codec device data port */
  SPIDEV_AUDIO_CTRL,  /* Select SPI audio codec device control port */
};

/* Certain SPI devices may required differnt clocking modes */

enum spi_mode_e
{
  SPIDEV_MODE0 = 0,   /* CPOL=0 CHPHA=0 */
  SPIDEV_MODE1,       /* CPOL=0 CHPHA=1 */
  SPIDEV_MODE2,       /* CPOL=1 CHPHA=0 */
  SPIDEV_MODE3        /* CPOL=1 CHPHA=1 */
};

/* The SPI vtable */

struct spi_dev_s;
struct spi_ops_s
{

  int      (*lock)( struct spi_dev_s *dev, bool lock);

  void     (*select)(struct spi_dev_s *dev, enum spi_dev_e devid,
                     bool selected);
  uint32_t (*setfrequency)(struct spi_dev_s *dev, uint32_t frequency);
  void     (*setmode)(struct spi_dev_s *dev, enum spi_mode_e mode);
  void     (*setbits)(struct spi_dev_s *dev, int nbits);
  uint8_t  (*status)( struct spi_dev_s *dev, enum spi_dev_e devid);

  uint16_t (*send)( struct spi_dev_s *dev, uint16_t wd);

  void     (*exchange)(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                       FAR void *rxbuffer, size_t nwords);

  void     (*sndblock)( struct spi_dev_s *dev,  const void *buffer,
                       size_t nwords);
  void     (*recvblock)( struct spi_dev_s *dev,  void *buffer,
                        size_t nwords);

  int     (*registercallback)( struct spi_dev_s *dev, spi_mediachange_t callback,
                              void *arg);
};

/* SPI private data.  This structure only defines the initial fields of the
 * structure visible to the SPI client.  The specific implementation may 
 * add additional, device specific fields
 */

struct spi_dev_s
{
  const struct spi_ops_s *ops;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize the selected SPI port.
 *
 *   This is a generic prototype for the SPI initialize logic.  Specific
 *   architectures may support different SPI initialization functions if,
 *   for example, those architectures support multiple, incompatible SPI
 *   implementations.  In any event, the prototype of those architecture-
 *   specific initialization functions should be the same as
 *   up_spiinitialize()
 *
 *   As an example, the LPC17xx family supports an SPI block and several SSP
 *   blocks that may be programmed to support the SPI function.  In this
 *   case, the LPC17xx architecture supports these two initialization
 *   functions:
 *
 *     FAR struct spi_dev_s *lpc17_spiinitialize(int port);
 *     FAR struct spi_dev_s *lpc17_sspinitialize(int port);
 *
 *   Another example would be the STM32 families that support both SPI
 *   blocks as well as USARTs that can be configured to perform the SPI
 *   function as well (the STM32 USARTs do not suppor SPI as of this
 *   writing).
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *up_spiinitialize(int port);


void stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);
void stm32_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_PX4FMU_SPI_H */
