/*
    This file is part of AutoQuad For PX4FMU(Pixhawk)

    Ported from nuttx\arch\arm\src\stm32\stm32_spi.c

    Copyright (C) 2014  BenZeng
*/
#include "px4fmu_config.h"
#include "px4fmu_flash.h"
#include "px4fmu_rcc.h"
#include "px4fmu_pwr.h"
#include "px4fmu_board.h"
#include "px4fmu_types.h"
#include "util.h"
#include "CoOS.h"
#include "px4fmu_spi.h"

struct stm32_spidev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  uint32_t         spibase;    /* SPIn base address */
  uint32_t         spiclock;   /* Clocking for the SPI module */

  OS_EventID       exclsem;    /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          nbits;      /* Width of word in bits (8 or 16) */
  uint8_t          mode;       /* Mode 0,1,2,3 */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Helpers */
static inline uint16_t spi_getreg( struct stm32_spidev_s *priv, uint8_t offset);
static inline void spi_putreg( struct stm32_spidev_s *priv, uint8_t offset,
                                 uint16_t value);
static inline uint16_t spi_readword( struct stm32_spidev_s *priv);
static inline void spi_writeword( struct stm32_spidev_s *priv, uint16_t byte);
static inline bool spi_16bitmode( struct stm32_spidev_s *priv);


/* SPI methods */
static int         spi_lock( struct spi_dev_s *dev, bool lock);
static uint32_t    spi_setfrequency( struct spi_dev_s *dev, uint32_t frequency);
static void        spi_setmode( struct spi_dev_s *dev, enum spi_mode_e mode);
static void        spi_setbits( struct spi_dev_s *dev, int nbits);
static uint16_t    spi_send( struct spi_dev_s *dev, uint16_t wd);
static void        spi_exchange_nodma( struct spi_dev_s *dev,  const void *txbuffer,
                                 void *rxbuffer, size_t nwords);

static void        spi_sndblock( struct spi_dev_s *dev,  const void *txbuffer,
                                size_t nwords);
static void        spi_recvblock( struct spi_dev_s *dev,  void *rxbuffer,
                                 size_t nwords);


/* Initialization */
static void        spi_portinitialize( struct stm32_spidev_s *priv);


/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI1
static const struct spi_ops_s g_spi1ops =
{
  .lock              = spi_lock,

  .select            = stm32_spi1select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = stm32_spi1status,
  .exchange          = spi_exchange_nodma,
  .send              = spi_send,

  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,

  .registercallback  = 0,
};

static struct stm32_spidev_s g_spi1dev =
{
  .spidev   = { &g_spi1ops },
  .spibase  = STM32_SPI1_BASE,
  .spiclock = STM32_PCLK2_FREQUENCY,
};
#endif

#ifdef CONFIG_STM32_SPI2
static const struct spi_ops_s g_spi2ops =
{
  .lock              = spi_lock,

  .select            = stm32_spi2select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = stm32_spi2status,
  .exchange          = spi_exchange_nodma,
  .send              = spi_send,

  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,

  .registercallback  = 0,
};

static struct stm32_spidev_s g_spi2dev =
{
  .spidev   = { &g_spi2ops },
  .spibase  = STM32_SPI2_BASE,
  .spiclock = STM32_PCLK1_FREQUENCY,
};
#endif


/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/


/************************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ************************************************************************************/

static inline uint16_t spi_getreg(FAR struct stm32_spidev_s *priv, uint8_t offset)
{
  return getreg16(priv->spibase + offset);
}

/************************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a 16-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 16-bit value to be written
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ************************************************************************************/

static inline void spi_putreg(FAR struct stm32_spidev_s *priv, uint8_t offset, uint16_t value)
{
  putreg16(value, priv->spibase + offset);
}

/************************************************************************************
 * Name: spi_readword
 *
 * Description:
 *   Read one byte from SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   Byte as read
 *
 ************************************************************************************/

static inline uint16_t spi_readword(FAR struct stm32_spidev_s *priv)
{
  /* Wait until the receive buffer is not empty */

  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_RXNE) == 0);

  /* Then return the received byte */

  return spi_getreg(priv, STM32_SPI_DR_OFFSET);
}

/************************************************************************************
 * Name: spi_writeword
 *
 * Description:
 *   Write one byte to SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   byte - Byte to send
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static inline void spi_writeword(FAR struct stm32_spidev_s *priv, uint16_t word)
{
  /* Wait until the transmit buffer is empty */

  while ((spi_getreg(priv, STM32_SPI_SR_OFFSET) & SPI_SR_TXE) == 0);

  /* Then send the byte */

  spi_putreg(priv, STM32_SPI_DR_OFFSET, word);
}

/************************************************************************************
 * Name: spi_16bitmode
 *
 * Description:
 *   Check if the SPI is operating in 16-bit mode
 *
 * Input Parameters:
 *   priv     - Device-specific state data
 *
 * Returned Value:
 *   true: 16-bit mode, false: 8-bit mode
 *
 ************************************************************************************/

static inline bool spi_16bitmode(FAR struct stm32_spidev_s *priv)
{
  return ((spi_getreg(priv, STM32_SPI_CR1_OFFSET) & SPI_CR1_DFF) != 0);
}


/************************************************************************************
 * Name: spi_modifycr1
 *
 * Description:
 *   Clear and set bits in the CR1 register
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_modifycr1(FAR struct stm32_spidev_s *priv, uint16_t setbits, uint16_t clrbits)
{
  uint16_t cr1;
  cr1 = spi_getreg(priv, STM32_SPI_CR1_OFFSET);
  cr1 &= ~clrbits;
  cr1 |= setbits;
  spi_putreg(priv, STM32_SPI_CR1_OFFSET, cr1);
}

/************************************************************************************
 * Name: spi_lock
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
 ************************************************************************************/
static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;

  if (lock)
    {
      /* Take the semaphore (perhaps waiting) */

      while (CoPendSem(priv->exclsem,0) != E_OK)
        {
          /* The only case that an error should occur here is if the wait was awakened
           * by a signal.
           */
          //ASSERT(errno == EINTR);
        }
    }
  else
    {
      (void)CoPostSem(priv->exclsem);
    }
  return OK;
}


/************************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ************************************************************************************/

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
  uint16_t setbits;
  uint32_t actual;

  /* Limit to max possible (if STM32_SPI_CLK_MAX is defined in board.h) */

  if (frequency > STM32_SPI_CLK_MAX)
    {
      frequency = STM32_SPI_CLK_MAX;
    }

  /* Has the frequency changed? */
  if (frequency != priv->frequency)
    {
      /* Choices are limited by PCLK frequency with a set of divisors */

      if (frequency >= priv->spiclock >> 1)
        {
          /* More than fPCLK/2.  This is as fast as we can go */

          setbits = SPI_CR1_FPCLCKd2; /* 000: fPCLK/2 */
          actual = priv->spiclock >> 1;
        }
      else if (frequency >= priv->spiclock >> 2)
        {
          /* Between fPCLCK/2 and fPCLCK/4, pick the slower */

          setbits = SPI_CR1_FPCLCKd4; /* 001: fPCLK/4 */
          actual = priv->spiclock >> 2;
       }
      else if (frequency >= priv->spiclock >> 3)
        {
          /* Between fPCLCK/4 and fPCLCK/8, pick the slower */

          setbits = SPI_CR1_FPCLCKd8; /* 010: fPCLK/8 */
          actual = priv->spiclock >> 3;
        }
      else if (frequency >= priv->spiclock >> 4)
        {
          /* Between fPCLCK/8 and fPCLCK/16, pick the slower */

          setbits = SPI_CR1_FPCLCKd16; /* 011: fPCLK/16 */
          actual = priv->spiclock >> 4;
        }
      else if (frequency >= priv->spiclock >> 5)
        {
          /* Between fPCLCK/16 and fPCLCK/32, pick the slower */

          setbits = SPI_CR1_FPCLCKd32; /* 100: fPCLK/32 */
          actual = priv->spiclock >> 5;
        }
      else if (frequency >= priv->spiclock >> 6)
        {
          /* Between fPCLCK/32 and fPCLCK/64, pick the slower */

          setbits = SPI_CR1_FPCLCKd64; /*  101: fPCLK/64 */
          actual = priv->spiclock >> 6;
        }
      else if (frequency >= priv->spiclock >> 7)
        {
          /* Between fPCLCK/64 and fPCLCK/128, pick the slower */

          setbits = SPI_CR1_FPCLCKd128; /* 110: fPCLK/128 */
          actual = priv->spiclock >> 7;
        }
      else
        {
          /* Less than fPCLK/128.  This is as slow as we can go */

          setbits = SPI_CR1_FPCLCKd256; /* 111: fPCLK/256 */
          actual = priv->spiclock >> 8;
        }

      spi_modifycr1(priv, setbits, SPI_CR1_BR_MASK);

      /* Save the frequency selection so that subsequent reconfigurations will be
       * faster.
       */
      priv->frequency = frequency;
      priv->actual    = actual;
    }
  return priv->actual;
}

/************************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ************************************************************************************/

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
  uint16_t setbits;
  uint16_t clrbits;

  //spivdbg("mode=%d\n", mode);

  /* Has the mode changed? */
  if (mode != priv->mode)
    {
      /* Yes... Set CR1 appropriately */

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          setbits = 0;
          clrbits = SPI_CR1_CPOL|SPI_CR1_CPHA;
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          setbits = SPI_CR1_CPHA;
          clrbits = SPI_CR1_CPOL;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          setbits = SPI_CR1_CPOL;
          clrbits = SPI_CR1_CPHA;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          setbits = SPI_CR1_CPOL|SPI_CR1_CPHA;
          clrbits = 0;
          break;

        default:
          return;
        }

        spi_modifycr1(priv, setbits, clrbits);

        /* Save the mode so that subsequent re-configurations will be faster */
        priv->mode = mode;
    }
}

/************************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
  uint16_t setbits;
  uint16_t clrbits;

  //spivdbg("nbits=%d\n", nbits);

  /* Has the number of bits changed? */
  if (nbits != priv->nbits)
    {
      /* Yes... Set CR1 appropriately */

      switch (nbits)
        {
        case 8:
          setbits = 0;
          clrbits = SPI_CR1_DFF;
          break;

        case 16:
          setbits = SPI_CR1_DFF;
          clrbits = 0;
          break;

        default:
          return;
        }

      spi_modifycr1(priv, 0, SPI_CR1_SPE);
      spi_modifycr1(priv, setbits, clrbits);
      spi_modifycr1(priv, SPI_CR1_SPE, 0);
      
      /* Save the selection so the subsequence re-configurations will be faster */
      priv->nbits = nbits;
    }
}

/************************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ************************************************************************************/

static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;
  uint32_t regval;
  uint16_t ret;

  spi_writeword(priv, wd);
  ret = spi_readword(priv);

  /* Check and clear any error flags (Reading from the SR clears the error flags) */
  regval = spi_getreg(priv, STM32_SPI_SR_OFFSET);

  return ret;
}

/************************************************************************************
 * Name: spi_exchange (no DMA).  aka spi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchaned in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/
static void spi_exchange_nodma(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                               FAR void *rxbuffer, size_t nwords)
{
  FAR struct stm32_spidev_s *priv = (FAR struct stm32_spidev_s *)dev;

  /* 8- or 16-bit mode? */
  if (spi_16bitmode(priv))
    {
      /* 16-bit mode */

      const uint16_t *src  = (const uint16_t*)txbuffer;;
            uint16_t *dest = (uint16_t*)rxbuffer;
            uint16_t  word;

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              word = *src++;
            }
          else
          {
              word = 0xffff;
          }

          /* Exchange one word */

          word = spi_send(dev, word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = word;
            }
        }
    }
  else
    {
      /* 8-bit mode */

      const uint8_t *src  = (const uint8_t*)txbuffer;;
            uint8_t *dest = (uint8_t*)rxbuffer;
            uint8_t  word;

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              word = *src++;
            }
          else
          {
              word = 0xff;
          }

          /* Exchange one word */

          word = (uint8_t)spi_send(dev, (uint16_t)word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = word;
            }
        }
    }
}


/*************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer, size_t nwords)
{
     spi_exchange_nodma(dev, txbuffer, NULL, nwords);
}


/************************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to recieve data
 *   nwords   - the length of data that can be received in the buffer in number
 *              of words.  The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer, size_t nwords)
{
  return spi_exchange_nodma(dev, NULL, rxbuffer, nwords);
}


/************************************************************************************
 * Name: spi_portinitialize
 *
 * Description:
 *   Initialize the selected SPI port in its default state (Master, 8-bit, mode 0, etc.)
 *
 * Input Parameter:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void spi_portinitialize(FAR struct stm32_spidev_s *priv)
{
  uint16_t setbits;
  uint16_t clrbits;

  /* Configure CR1. Default configuration:
   *   Mode 0:                        CPHA=0 and CPOL=0
   *   Master:                        MSTR=1
   *   8-bit:                         DFF=0
   *   MSB tranmitted first:          LSBFIRST=0
   *   Replace NSS with SSI & SSI=1:  SSI=1 SSM=1 (prevents MODF error)
   *   Two lines full duplex:         BIDIMODE=0 BIDIOIE=(Don't care) and RXONLY=0
   */

  clrbits = SPI_CR1_CPHA|SPI_CR1_CPOL|SPI_CR1_BR_MASK|SPI_CR1_LSBFIRST|
            SPI_CR1_RXONLY|SPI_CR1_DFF|SPI_CR1_BIDIOE|SPI_CR1_BIDIMODE;
  setbits = SPI_CR1_MSTR|SPI_CR1_SSI|SPI_CR1_SSM;
  spi_modifycr1(priv, setbits, clrbits);


  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;


  /* Select a default frequency of approx. 400KHz */
  spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* CRCPOLY configuration */
  spi_putreg(priv, STM32_SPI_CRCPR_OFFSET, 7);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

  priv->exclsem = CoCreateSem(1,1,EVENT_SORT_TYPE_FIFO);


  /* Enable spi */
  spi_modifycr1(priv, SPI_CR1_SPE, 0);
}





/************************************************************************************
 * Public Functions
 ************************************************************************************/

void stm32_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* there can only be one device on this bus, so always select it */
	stm32_gpiowrite(GPIO_SPI_CS_FRAM, !selected);
}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	/* FRAM is always present */
	return SPI_STATUS_PRESENT;
}

void stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	switch (devid) {
	case PX4_SPIDEV_GYRO:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_GYRO, !selected);
		stm32_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
		stm32_gpiowrite(GPIO_SPI_CS_BARO, 1);
		stm32_gpiowrite(GPIO_SPI_CS_MPU, 1);
		break;

	case PX4_SPIDEV_ACCEL_MAG:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_GYRO, 1);
		stm32_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, !selected);
		stm32_gpiowrite(GPIO_SPI_CS_BARO, 1);
		stm32_gpiowrite(GPIO_SPI_CS_MPU, 1);
		break;

	case PX4_SPIDEV_BARO:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_GYRO, 1);
		stm32_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
		stm32_gpiowrite(GPIO_SPI_CS_BARO, !selected);
		stm32_gpiowrite(GPIO_SPI_CS_MPU, 1);
		break;

	case PX4_SPIDEV_MPU:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI_CS_GYRO, 1);
		stm32_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
		stm32_gpiowrite(GPIO_SPI_CS_BARO, 1);
		stm32_gpiowrite(GPIO_SPI_CS_MPU, !selected);
		break;

	default:
		break;
	}
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	return SPI_STATUS_PRESENT;
}

/****************************************************************************
 * Name: modifyreg8
 *
 * Description:
 *   Atomically modify the specified bits in a memory mapped register
 *
 ****************************************************************************/

void modifyreg8(unsigned int addr, uint8_t clearbits, uint8_t setbits)
{
  //irqstate_t flags;
  uint8_t    regval;

  CoSchedLock(); // irqsave()
  regval  = getreg8(addr);
  regval &= ~clearbits;
  regval |= setbits;
  putreg8(regval, addr);
  CoSchedUnlock();
}


/****************************************************************************
 * Name: modifyreg16
 *
 * Description:
 *   Atomically modify the specified bits in a memory mapped register
 *
 ****************************************************************************/

void modifyreg16(unsigned int addr, uint16_t clearbits, uint16_t setbits)
{
  //irqstate_t flags;
  uint16_t   regval;

  CoSchedLock();
  regval  = getreg16(addr);
  regval &= ~clearbits;
  regval |= setbits;
  putreg16(regval, addr);
  CoSchedUnlock();
}

/****************************************************************************
 * Name: modifyreg32
 *
 * Description:
 *   Atomically modify the specified bits in a memory mapped register
 *
 ****************************************************************************/

void modifyreg32(unsigned int addr, uint32_t clearbits, uint32_t setbits)
{
  //irqstate_t flags;
  uint32_t   regval;

  CoSchedLock();
  regval  = getreg32(addr);
  regval &= ~clearbits;
  regval |= setbits;
  putreg32(regval, addr);
  CoSchedUnlock();
}


/************************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ************************************************************************************/

FAR struct spi_dev_s *up_spiinitialize(int port)
{
  FAR struct stm32_spidev_s *priv = NULL;

  CoSchedLock();
  //irqstate_t flags = irqsave();

#ifdef CONFIG_STM32_SPI1
  if (port == 1)
    {
      /* Select SPI1 */

      priv = &g_spi1dev;

      /* Only configure if the port is not already configured */

      if ((spi_getreg(priv, STM32_SPI_CR1_OFFSET) & SPI_CR1_SPE) == 0)
        {
          /* Configure SPI1 pins: SCK, MISO, and MOSI */

          stm32_configgpio(GPIO_SPI1_SCK);
          stm32_configgpio(GPIO_SPI1_MISO);
          stm32_configgpio(GPIO_SPI1_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          spi_portinitialize(priv);
        }
    }
  else
#endif

#ifdef CONFIG_STM32_SPI2
  if (port == 2)
    {
      /* Select SPI2 */

      priv = &g_spi2dev;

      /* Only configure if the port is not already configured */

      if ((spi_getreg(priv, STM32_SPI_CR1_OFFSET) & SPI_CR1_SPE) == 0)
        {
          /* Configure SPI2 pins: SCK, MISO, and MOSI */

          stm32_configgpio(GPIO_SPI2_SCK);
          stm32_configgpio(GPIO_SPI2_MISO);
          stm32_configgpio(GPIO_SPI2_MOSI);

          /* Set up default configuration: Master, 8-bit, etc. */

          spi_portinitialize(priv);
        }
    }
  else
#endif

    {
      //spidbg("ERROR: Unsupported SPI port: %d\n", port);
      return NULL;
    }

  //irqrestore(flags);
  CoSchedUnlock();

  return (FAR struct spi_dev_s *)priv;
}


/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ************************************************************************************/

void  stm32_spiinitialize(void)
{
#ifdef CONFIG_STM32_SPI1
	stm32_configgpio(GPIO_SPI_CS_GYRO);
	stm32_configgpio(GPIO_SPI_CS_ACCEL_MAG);
	stm32_configgpio(GPIO_SPI_CS_BARO);
	stm32_configgpio(GPIO_SPI_CS_MPU);

	/* De-activate all peripherals,
	 * required for some peripheral
	 * state machines
	 */
	stm32_gpiowrite(GPIO_SPI_CS_GYRO, 1);
	stm32_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
	stm32_gpiowrite(GPIO_SPI_CS_BARO, 1);
	stm32_gpiowrite(GPIO_SPI_CS_MPU, 1);

	stm32_configgpio(GPIO_EXTI_GYRO_DRDY);
	stm32_configgpio(GPIO_EXTI_MAG_DRDY);
	stm32_configgpio(GPIO_EXTI_ACCEL_DRDY);
	stm32_configgpio(GPIO_EXTI_MPU_DRDY);
#endif

#ifdef CONFIG_STM32_SPI2
	stm32_configgpio(GPIO_SPI_CS_FRAM);
	stm32_gpiowrite(GPIO_SPI_CS_FRAM, 1);
#endif
}


//static struct spi_dev_s *spi2 = NULL;
void InitPx4fmuSPI( void )
{
    struct spi_dev_s *spi1 = NULL;

    /* configure SPI interfaces */
    stm32_spiinitialize();

    spi1 = up_spiinitialize(1);
    if( spi1 != NULL )
    {
        /* Default SPI1 to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi1, 10000000);
	SPI_SETBITS(spi1, 8);
	SPI_SETMODE(spi1, SPIDEV_MODE3);
	SPI_SELECT(spi1, PX4_SPIDEV_GYRO, false);
	SPI_SELECT(spi1, PX4_SPIDEV_ACCEL_MAG, false);
	SPI_SELECT(spi1, PX4_SPIDEV_BARO, false);
	SPI_SELECT(spi1, PX4_SPIDEV_MPU, false);
	yield(20);
    }
}

void SpiTransfer( SPI_INSTANCE_DATA *pThis, uint8_t *send, uint8_t *recv, unsigned len )
{
    SPI_LOCK( pThis->_dev, true );

    SPI_SETFREQUENCY(pThis->_dev, pThis->_frequency);
    SPI_SETMODE(pThis->_dev, pThis->_mode);
    SPI_SETBITS(pThis->_dev, 8);
    SPI_SELECT(pThis->_dev, pThis->_device, true);

    /* do the transfer */
    SPI_EXCHANGE(pThis->_dev, send, recv, len);

    /* and clean up */
    SPI_SELECT(pThis->_dev, pThis->_device, false);

    SPI_LOCK( pThis->_dev, false );
}

void sensor_reset( void )
{
/* disable SPI bus */
	stm32_configgpio(GPIO_SPI_CS_GYRO_OFF);
	stm32_configgpio(GPIO_SPI_CS_ACCEL_MAG_OFF);
	stm32_configgpio(GPIO_SPI_CS_BARO_OFF);
	stm32_configgpio(GPIO_SPI_CS_MPU_OFF);

	stm32_gpiowrite(GPIO_SPI_CS_GYRO_OFF, 0);
	stm32_gpiowrite(GPIO_SPI_CS_ACCEL_MAG_OFF, 0);
	stm32_gpiowrite(GPIO_SPI_CS_BARO_OFF, 0);
	stm32_gpiowrite(GPIO_SPI_CS_MPU_OFF, 0);

	stm32_configgpio(GPIO_SPI1_SCK_OFF);
	stm32_configgpio(GPIO_SPI1_MISO_OFF);
	stm32_configgpio(GPIO_SPI1_MOSI_OFF);

	stm32_gpiowrite(GPIO_SPI1_SCK_OFF, 0);
	stm32_gpiowrite(GPIO_SPI1_MISO_OFF, 0);
	stm32_gpiowrite(GPIO_SPI1_MOSI_OFF, 0);

	stm32_configgpio(GPIO_GYRO_DRDY_OFF);
	stm32_configgpio(GPIO_MAG_DRDY_OFF);
	stm32_configgpio(GPIO_ACCEL_DRDY_OFF);
	stm32_configgpio(GPIO_EXTI_MPU_DRDY_OFF);

	stm32_gpiowrite(GPIO_GYRO_DRDY_OFF, 0);
	stm32_gpiowrite(GPIO_MAG_DRDY_OFF, 0);
	stm32_gpiowrite(GPIO_ACCEL_DRDY_OFF, 0);
	stm32_gpiowrite(GPIO_EXTI_MPU_DRDY_OFF, 0);

	/* set the sensor rail off */
	stm32_configgpio(GPIO_VDD_3V3_SENSORS_EN);
	stm32_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, 0);

	/* wait for the sensor rail to reach GND */
	yield( 100 );
	//warnx("reset done, %d ms", ms);

	/* re-enable power */

	/* switch the sensor rail back on */
	stm32_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, 1);

	/* wait a bit before starting SPI, different times didn't influence results */
	yield(2 * 1000);

	/* reconfigure the SPI pins */
#ifdef CONFIG_STM32_SPI1
	stm32_configgpio(GPIO_SPI_CS_GYRO);
	stm32_configgpio(GPIO_SPI_CS_ACCEL_MAG);
	stm32_configgpio(GPIO_SPI_CS_BARO);
	stm32_configgpio(GPIO_SPI_CS_MPU);

	/* De-activate all peripherals,
	 * required for some peripheral
	 * state machines
	 */
	stm32_gpiowrite(GPIO_SPI_CS_GYRO, 1);
	stm32_gpiowrite(GPIO_SPI_CS_ACCEL_MAG, 1);
	stm32_gpiowrite(GPIO_SPI_CS_BARO, 1);
	stm32_gpiowrite(GPIO_SPI_CS_MPU, 1);

	// // XXX bring up the EXTI pins again
	// stm32_configgpio(GPIO_GYRO_DRDY);
	// stm32_configgpio(GPIO_MAG_DRDY);
	// stm32_configgpio(GPIO_ACCEL_DRDY);
	// stm32_configgpio(GPIO_EXTI_MPU_DRDY);

#endif
}

#if 0

#define ADDR_RESET_CMD			0x1E	/* write to this address to reset chip */
#define ADDR_CMD_CONVERT_D1		0x48	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2		0x58	/* write to this address to start pressure conversion */
#define ADDR_DATA			0x00	/* address of 3 bytes / 32bit pressure data */
#define ADDR_PROM_SETUP			0xA0	/* address of 8x 2 bytes factory and calibration data */
#define ADDR_PROM_C1			0xA2	/* address of 6x 2 bytes calibration data */

/* SPI protocol address bits */
#define DIR_READ			(1<<7)
#define DIR_WRITE			(0<<7)
#define ADDR_INCREMENT			(1<<6)

/**
 * Calibration PROM as reported by the device.
 */
#pragma pack(push,1)
struct prom_s {
	uint16_t factory_setup;
	uint16_t c1_pressure_sens;
	uint16_t c2_pressure_offset;
	uint16_t c3_temp_coeff_pres_sens;
	uint16_t c4_temp_coeff_pres_offset;
	uint16_t c5_reference_temp;
	uint16_t c6_temp_coeff_temp;
	uint16_t serial_and_crc;
};

/**
 * Grody hack for crc4()
 */
union prom_u {
	uint16_t c[8];
	struct prom_s s;
};
#pragma pack(pop)




SPI_INSTANCE_DATA spiMS5611;


void ResetMS5611( SPI_INSTANCE_DATA *pThis )
{
    uint8_t cmd = ADDR_RESET_CMD | DIR_WRITE;

    return  SpiTransfer(pThis, &cmd, NULL, 1);
}


static void _measure(SPI_INSTANCE_DATA *pThis, unsigned addr)
{
	uint8_t cmd = addr | DIR_WRITE;

	SpiTransfer(pThis, &cmd, NULL, 1);
}

static uint16_t _reg16(SPI_INSTANCE_DATA *pThis, unsigned reg)
{
	uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0 };

	SpiTransfer(pThis, cmd, cmd, sizeof(cmd));

	return (uint16_t)(cmd[1] << 8) | cmd[2];
}

/**
 * MS5611 crc4 cribbed from the datasheet
 */
static bool _crc4(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */
	crc_read = n_prom[7];

	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (0x000F & crc_read) == (n_rem ^ 0x00);
}

static int ReadPROM( SPI_INSTANCE_DATA *pThis )
{
	union prom_u _prom;

	/*
	 * Wait for PROM contents to be in the device (2.8 ms) in the case we are
	 * called immediately after reset.
	 */
	yield(3000);

	/* read and convert PROM words */
        bool all_zero = true;
	for (int i = 0; i < 8; i++) {
		uint8_t cmd = (ADDR_PROM_SETUP + (i * 2));
		_prom.c[i] = _reg16(pThis, cmd);
                if (_prom.c[i] != 0)
			all_zero = false;
                //debug("prom[%u]=0x%x", (unsigned)i, (unsigned)_prom.c[i]);
	}

	/* calculate CRC and return success/failure accordingly */
	int ret = _crc4(&_prom.c[0]) ? 0 : -1;
        if (ret != OK) {
		//debug("crc failed");
		return -1;
        }
        if (all_zero) {
		//debug("prom all zero");
		ret = -1;
        }
        return ret;
}


void TestMS5611( void )
{
    //void stm32_clockconfig(void);
    //stm32_clockconfig();
    //InitPx4fmuSPI();
    void px4fmu_boardinitialize(void);
    px4fmu_boardinitialize();

    spiMS5611._bus = 1;
    spiMS5611._device = PX4_SPIDEV_BARO;
    spiMS5611._mode = SPIDEV_MODE3;
    spiMS5611._frequency = 6*1000*1000;
    spiMS5611._dev = up_spiinitialize(spiMS5611._bus);
    if( spiMS5611._dev == NULL )
	return;
    /* deselect device to ensure high to low transition of pin select */
    SPI_SELECT(spiMS5611._dev, spiMS5611._device, false);

    /* send reset command */
    ResetMS5611( &spiMS5611 );

    /* read PROM */
    ReadPROM( &spiMS5611 );
}
#endif
