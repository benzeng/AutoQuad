/*
    This file is part of AutoQuad For PX4FMU(Pixhawk)

    Ported from \nuttx\include\nuttx\spi.h

    Copyright (C) 2014  BenZeng
*/

#ifndef __INCLUDE_PX4FMU_I2C_H
#define __INCLUDE_PX4FMU_I2C_H

#include "px4fmu_chip.h"
#include "px4fmu_rcc.h"
#include "px4fmu_gpio.h"

#define CONFIG_I2C_POLLED


/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_I2C_CR1_OFFSET    0x0000  /* Control register 1 (16-bit) */
#define STM32_I2C_CR2_OFFSET    0x0004  /* Control register 2 (16-bit) */
#define STM32_I2C_OAR1_OFFSET   0x0008  /* Own address register 1 (16-bit) */
#define STM32_I2C_OAR2_OFFSET   0x000c  /* Own address register 2 (16-bit) */
#define STM32_I2C_DR_OFFSET     0x0010  /* Data register (16-bit) */
#define STM32_I2C_SR1_OFFSET    0x0014  /* Status register 1 (16-bit) */
#define STM32_I2C_SR2_OFFSET    0x0018  /* Status register 2 (16-bit) */
#define STM32_I2C_CCR_OFFSET    0x001c  /* Clock control register (16-bit) */
#define STM32_I2C_TRISE_OFFSET  0x0020  /* TRISE Register (16-bit) */
#ifdef CONFIG_STM32_STM32F427
#  define STM32_I2C_FLTR_OFFSET   0x0024  /* FLTR Register (16-bit) */
#endif

/* Register Addresses ***************************************************************/

#if STM32_NI2C > 0
#  define STM32_I2C1_CR1        (STM32_I2C1_BASE+STM32_I2C_CR1_OFFSET)
#  define STM32_I2C1_CR2        (STM32_I2C1_BASE+STM32_I2C_CR2_OFFSET)
#  define STM32_I2C1_OAR1       (STM32_I2C1_BASE+STM32_I2C_OAR1_OFFSET)
#  define STM32_I2C1_OAR2       (STM32_I2C1_BASE+STM32_I2C_OAR2_OFFSET)
#  define STM32_I2C1_DR         (STM32_I2C1_BASE+STM32_I2C_DR_OFFSET)
#  define STM32_I2C1_SR1        (STM32_I2C1_BASE+STM32_I2C_SR1_OFFSET)
#  define STM32_I2C1_SR2        (STM32_I2C1_BASE+STM32_I2C_SR2_OFFSET)
#  define STM32_I2C1_CCR        (STM32_I2C1_BASE+STM32_I2C_CCR_OFFSET)
#  define STM32_I2C1_TRISE      (STM32_I2C1_BASE+STM32_I2C_TRISE_OFFSET)
#  ifdef STM32_I2C_FLTR_OFFSET
#    define STM32_I2C1_FLTR       (STM32_I2C1_BASE+STM32_I2C_FLTR_OFFSET)
#  endif
#endif

#if STM32_NI2C > 1
#  define STM32_I2C2_CR1        (STM32_I2C2_BASE+STM32_I2C_CR1_OFFSET)
#  define STM32_I2C2_CR2        (STM32_I2C2_BASE+STM32_I2C_CR2_OFFSET)
#  define STM32_I2C2_OAR1       (STM32_I2C2_BASE+STM32_I2C_OAR1_OFFSET)
#  define STM32_I2C2_OAR2       (STM32_I2C2_BASE+STM32_I2C_OAR2_OFFSET)
#  define STM32_I2C2_DR         (STM32_I2C2_BASE+STM32_I2C_DR_OFFSET)
#  define STM32_I2C2_SR1        (STM32_I2C2_BASE+STM32_I2C_SR1_OFFSET)
#  define STM32_I2C2_SR2        (STM32_I2C2_BASE+STM32_I2C_SR2_OFFSET)
#  define STM32_I2C2_CCR        (STM32_I2C2_BASE+STM32_I2C_CCR_OFFSET)
#  define STM32_I2C2_TRISE      (STM32_I2C2_BASE+STM32_I2C_TRISE_OFFSET)
#  ifdef STM32_I2C_FLTR_OFFSET
#    define STM32_I2C2_FLTR       (STM32_I2C2_BASE+STM32_I2C_FLTR_OFFSET)
#  endif
#endif

#if STM32_NI2C > 2
#  define STM32_I2C3_CR1        (STM32_I2C3_BASE+STM32_I2C_CR1_OFFSET)
#  define STM32_I2C3_CR2        (STM32_I2C3_BASE+STM32_I2C_CR2_OFFSET)
#  define STM32_I2C3_OAR1       (STM32_I2C3_BASE+STM32_I2C_OAR1_OFFSET)
#  define STM32_I2C3_OAR2       (STM32_I2C3_BASE+STM32_I2C_OAR2_OFFSET)
#  define STM32_I2C3_DR         (STM32_I2C3_BASE+STM32_I2C_DR_OFFSET)
#  define STM32_I2C3_SR1        (STM32_I2C3_BASE+STM32_I2C_SR1_OFFSET)
#  define STM32_I2C3_SR2        (STM32_I2C3_BASE+STM32_I2C_SR2_OFFSET)
#  define STM32_I2C3_CCR        (STM32_I2C3_BASE+STM32_I2C_CCR_OFFSET)
#  define STM32_I2C3_TRISE      (STM32_I2C3_BASE+STM32_I2C_TRISE_OFFSET)
#  ifdef STM32_I2C_FLTR_OFFSET
#    define STM32_I2C3_FLTR       (STM32_I2C3_BASE+STM32_I2C_FLTR_OFFSET)
#  endif
#endif

/* Register Bitfield Definitions ****************************************************/

/* Control register 1 */

#define I2C_CR1_PE              (1 << 0)  /* Bit 0: Peripheral Enable */
#define I2C_CR1_SMBUS           (1 << 1)  /* Bit 1: SMBus Mode */
#define I2C_CR1_SMBTYPE         (1 << 3)  /* Bit 3: SMBus Type */
#define I2C_CR1_ENARP           (1 << 4)  /* Bit 4: ARP Enable */
#define I2C_CR1_ENPEC           (1 << 5)  /* Bit 5: PEC Enable */
#define I2C_CR1_ENGC            (1 << 6)  /* Bit 6: General Call Enable */
#define I2C_CR1_NOSTRETCH       (1 << 7)  /* Bit 7: Clock Stretching Disable (Slave mode) */
#define I2C_CR1_START           (1 << 8)  /* Bit 8: Start Generation */
#define I2C_CR1_STOP            (1 << 9)  /* Bit 9: Stop Generation */
#define I2C_CR1_ACK             (1 << 10) /* Bit 10: Acknowledge Enable */
#define I2C_CR1_POS             (1 << 11) /* Bit 11: Acknowledge/PEC Position (for data reception) */
#define I2C_CR1_PEC             (1 << 12) /* Bit 12: Packet Error Checking */
#define I2C_CR1_ALERT           (1 << 13) /* Bit 13: SMBus Alert */
#define I2C_CR1_SWRST           (1 << 15) /* Bit 15: Software Reset */

/* Control register 2 */

#define I2C_CR2_FREQ_SHIFT      (0)       /* Bits 5-0: Peripheral Clock Frequency */
#define I2C_CR2_FREQ_MASK       (0x3f << I2C_CR2_FREQ_SHIFT)
#define I2C_CR2_ITERREN         (1 << 8)  /* Bit 8: Error Interrupt Enable */
#define I2C_CR2_ITEVFEN         (1 << 9)  /* Bit 9: Event Interrupt Enable */
#define I2C_CR2_ITBUFEN         (1 << 10) /* Bit 10: Buffer Interrupt Enable */
#define I2C_CR2_DMAEN           (1 << 11) /* Bit 11: DMA Requests Enable */
#define I2C_CR2_LAST            (1 << 12) /* Bit 12: DMA Last Transfer */

#define I2C_CR2_ALLINTS         (I2C_CR2_ITERREN|I2C_CR2_ITEVFEN|I2C_CR2_ITBUFEN)

/* Own address register 1 */

#define I2C_OAR1_ADD0           (1 << 0)  /* Bit 0: Interface Address */
#define I2C_OAR1_ADD8_SHIFT     (1)       /* Bits 7-1: Interface Address */
#define I2C_OAR1_ADD8_MASK      (0x007f << I2C_OAR1_ADD8_SHIFT)
#define I2C_OAR1_ADD10_SHIFT    (1)       /* Bits 9-1: Interface Address (10-bit addressing mode)*/
#define I2C_OAR1_ADD10_MASK     (0x01ff << I2C_OAR1_ADD10_SHIFT)
#define I2C_OAR1_ONE            (1 << 14) /* Bit 14: Must be configured and kept at 1 */
#define I2C_OAR1_ADDMODE        (1 << 15) /* Bit 15: Addressing Mode (Slave mode) */

/* Own address register 2 */

#define I2C_OAR2_ENDUAL         (1 << 0)  /* Bit 0: Dual addressing mode enable */
#define I2C_OAR2_ADD2_SHIFT     (1)       /* Bits 7-1: Interface address */
#define I2C_OAR2_ADD2_MASK      (0x7f << I2C_OAR2_ADD2_SHIFT)

/* Data register */

#define I2C_DR_SHIFT            (0)       /* Bits 7-0: 8-bit Data Register */
#define I2C_DR_MASK             (0x00ff << I2C_DR_SHIFT)

/* Status register 1 */

#define I2C_SR1_SB              (1 << 0)  /* Bit 0: Start Bit (Master mode) */
#define I2C_SR1_ADDR            (1 << 1)  /* Bit 1: Address sent (master mode)/matched (slave mode) */
#define I2C_SR1_BTF             (1 << 2)  /* Bit 2: Byte Transfer Finished */
#define I2C_SR1_ADD10           (1 << 3)  /* Bit 3: 10-bit header sent (Master mode) */
#define I2C_SR1_STOPF           (1 << 4)  /* Bit 4: Stop detection (Slave mode) */
                                          /* Bit 5: Reserved */
#define I2C_SR1_RXNE            (1 << 6)  /* Bit 6: Data Register not Empty (receivers) */
#define I2C_SR1_TXE             (1 << 7)  /* Bit 7: Data Register Empty (transmitters) */
#define I2C_SR1_BERR            (1 << 8)  /* Bit 8: Bus Error */
#define I2C_SR1_ARLO            (1 << 9)  /* Bit 9: Arbitration Lost (master mode) */
#define I2C_SR1_AF              (1 << 10) /* Bit 10: Acknowledge Failure */
#define I2C_SR1_OVR             (1 << 11) /* Bit 11: Overrun/Underrun */
#define I2C_SR1_PECERR          (1 << 12) /* Bit 12: PEC Error in reception */
                                          /* Bit 13: Reserved */
#define I2C_SR1_TIMEOUT         (1 << 14) /* Bit 14: Timeout or Tlow Error */
#define I2C_SR1_SMBALERT        (1 << 15) /* Bit 15: SMBus Alert */

#define I2C_SR1_ERRORMASK       (I2C_SR1_BERR|I2C_SR1_ARLO|I2C_SR1_AF|I2C_SR1_OVR|\
                                 I2C_SR1_PECERR|I2C_SR1_TIMEOUT|I2C_SR1_SMBALERT)

/* Status register 2 */

#define I2C_SR2_MSL             (1 << 0)  /* Bit 0: Master/Slave */
#define I2C_SR2_BUSY            (1 << 1)  /* Bit 1: Bus Busy */
#define I2C_SR2_TRA             (1 << 2)  /* Bit 2: Transmitter/Receiver */
#define I2C_SR2_GENCALL         (1 << 4)  /* Bit 4: General Call Address (Slave mode) */
#define I2C_SR2_SMBDEFAULT      (1 << 5)  /* Bit 5: SMBus Device Default Address (Slave mode) */
#define I2C_SR2_SMBHOST         (1 << 6)  /* Bit 6: SMBus Host Header (Slave mode) */
#define I2C_SR2_DUALF           (1 << 7)  /* Bit 7: Dual Flag (Slave mode) */
#define I2C_SR2_PEC_SHIFT       (1)       /* Bits 15-8: Packet Error Checking Register */
#define I2C_SR2_PEC_MASK        (0xff << I2C_SR2_PEC_SHIFT)

/* Clock control register */

#define I2C_CCR_CCR_SHIFT       (0)       /* Bits 11-0: Clock Control Register in Fast/Standard mode (Master mode) */
#define I2C_CCR_CCR_MASK        (0x0fff << I2C_CCR_CCR_SHIFT)
#define I2C_CCR_DUTY            (1 << 14) /* Bit 14: Fast Mode Duty Cycle */
#define I2C_CCR_FS              (1 << 15) /* Bit 15: Fast Mode Selection */

/* TRISE Register */

#define I2C_TRISE_SHIFT         (0) /* Bits 5-0: Maximum Rise Time in Fast/Standard mode (Master mode) */
#define I2C_TRISE_MASK          (0x3f << I2C_TRISE_SHIFT)

/* FLTR Register */

#ifdef STM32_I2C_FLTR_OFFSET
#  define I2C_FLTR_ANOFF	(1 << 4)  /* Bit 4: Analog noise filter disable */
#  define I2C_FLTR_DNF_SHIFT	0         /* Bits 0-3: Digital noise filter */
#  define I2C_FLTR_DNF_MASK	(0xf << I2C_FLTR_DNF_SHIFT)
#endif




/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* If a dynamic timeout is selected, then a non-negative, non-zero micro-
 * seconds per byte vale must be provided as well.
 */

#ifdef CONFIG_STM32_I2C_DYNTIMEO
#  if CONFIG_STM32_I2C_DYNTIMEO_USECPERBYTE < 1
#    warning "Ignoring CONFIG_STM32_I2C_DYNTIMEO because of CONFIG_STM32_I2C_DYNTIMEO_USECPERBYTE"
#    undef CONFIG_STM32_I2C_DYNTIMEO
#  endif
#endif

/* I2C address calculation.  Convert 7- and 10-bit address to 8-bit and
 * 16-bit read/write address
 */

#define I2C_READBIT          0x01

/* Conver 7- to 8-bit address */

#define I2C_ADDR8(a)         ((a) << 1)
#define I2C_WRITEADDR8(a)    I2C_ADDR8(a)
#define I2C_READADDR8(a)     (I2C_ADDR8(a) | I2C_READBIT)

/* Convert 10- to 16-bit address */

#define I2C_ADDR10H(a)       (0xf0 | (((a) >> 7) & 0x06))
#define I2C_ADDR10L(a)       ((a) & 0xff)

#define I2C_WRITEADDR10H(a)  I2C_ADDR10H(a)
#define I2C_WRITEADDR10L(a)  I2C_ADDR10L(a)

#define I2C_READADDR10H(a)   (I2C_ADDR10H(a) | I2C_READBIT)
#define I2C_READADDR10L(a)   I2C_ADDR10L(a)

/* Bit definitions for the flags field in struct i2c_ops_s */

#define I2C_M_READ           0x0001          /* read data, from slave to master */
#define I2C_M_TEN            0x0002          /* ten bit address */
#define I2C_M_NORESTART      0x0080          /* message should not begin with (re-)start of transfer */

/* Access macros */

/****************************************************************************
 * Name: I2C_SETFREQUENCY
 *
 * Description:
 *   Set the I2C frequency. This frequency will be retained in the struct
 *   i2c_dev_s instance and will be used with all transfers.  Required.
 *
 * Input Parameters:
 *   dev       - Device-specific state data
 *   frequency - The I2C frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

#define I2C_SETFREQUENCY(d,f) ((d)->ops->setfrequency(d,f))

/****************************************************************************
 * Name: I2C_SETADDRESS
 *
 * Description:
 *   Set the I2C slave address. This frequency will be retained in the struct
 *   i2c_dev_s instance and will be used with all transfers.  Required.
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   address - The I2C slave address
 *   nbits   - The number of address bits provided (7 or 10)
 *
 * Returned Value:
 *   Returns OK on success; a negated errno on failure.
 *
 ****************************************************************************/

#define I2C_SETADDRESS(d,a,n) ((d)->ops->setaddress(d,a,n))

/****************************************************************************
 * Name: I2C_SETOWNADDRESS
 *
 * Description:
 *   Set our own I2C address. Calling this function enables Slave mode and
 *   disables Master mode on given instance (note that I2C is a bus, where
 *   multiple masters and slave may be handled by one device driver).
 * 
 *   One may register callback to be notifyed about reception. During the
 *   slave mode reception, the function READ and WRITE must be used to 
 *   to handle reads and writes from a master.
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   address - Our own slave address; If it is 0x00, then the device driver
 *             listens to general call
 *   nbits   - The number of address bits provided (7 or 10)
 *
 * Returned Value:
 *   OK on valid address and if the same address has not been assigned
 *   to other existance sharing the same port. Otherwise ERROR is returned.
 *
 ****************************************************************************/

#define I2C_SETOWNADDRESS(d,a,n)  ((d)->ops->setownaddress(d,a,n))

/****************************************************************************
 * Name: I2C_WRITE
 *
 * Description:
 *   Send a block of data on I2C using the previously selected I2C
 *   frequency and slave address. Each write operational will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this write completes. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to the read-only buffer of data to be written to device
 *   buflen - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#define I2C_WRITE(d,b,l) ((d)->ops->write(d,b,l))

/****************************************************************************
 * Name: I2C_READ
 *
 * Description:
 *   Receive a block of data from I2C using the previously selected I2C
 *   frequency and slave address. Each read operational will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this read completes. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   buffer - A pointer to a buffer of data to receive the data from the device
 *   buflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#define I2C_READ(d,b,l) ((d)->ops->read(d,b,l))

/****************************************************************************
 * Name: I2C_WRITEREAD
 *
 * Description:
 *   Send a block of data on I2C using the previously selected I2C
 *   frequency and slave address, followed by restarted read access. 
 *   It provides a convenient wrapper to the transfer function.
 *
 * Input Parameters:
 *   dev     - Device-specific state data
 *   wbuffer - A pointer to the read-only buffer of data to be written to device
 *   wbuflen - The number of bytes to send from the buffer
 *   rbuffer - A pointer to a buffer of data to receive the data from the device
 *   rbuflen - The requested number of bytes to be read
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/

#define I2C_WRITEREAD(d,wb,wl,rb,rl) ((d)->ops->writeread(d,wb,wl,rb,rl))

/****************************************************************************
 * Name: I2C_TRANSFER
 *
 * Description:
 *   Perform a sequence of I2C transfers, each transfer is started with a 
 *   START and the final transfer is completed with a STOP. Each sequence 
 *   will be an 'atomic'  operation in the sense that any other I2C actions 
 *   will be serialized and pend until this read completes. Optional.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   msgs     - A pointer to a set of message descriptors
 *   msgcount - The number of transfers to perform
 *
 * Returned Value:
 *   The number of transfers completed
 *
 ****************************************************************************/

#define I2C_TRANSFER(d,m,c) ((d)->ops->transfer(d,m,c))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The I2C vtable */

struct i2c_dev_s;
struct i2c_msg_s;
struct i2c_ops_s
{
  uint32_t (*setfrequency)(FAR struct i2c_dev_s *dev, uint32_t frequency);
  int    (*setaddress)(FAR struct i2c_dev_s *dev, int addr, int nbits);
  int    (*write)(FAR struct i2c_dev_s *dev, const uint8_t *buffer, int buflen);
  int    (*read)(FAR struct i2c_dev_s *dev, uint8_t *buffer, int buflen);
#ifdef CONFIG_I2C_WRITEREAD
  int    (*writeread)(FAR struct i2c_dev_s *inst, const uint8_t *wbuffer, int wbuflen,
                        uint8_t *rbuffer, int rbuflen);
#endif
#ifdef CONFIG_I2C_TRANSFER
  int    (*transfer)(FAR struct i2c_dev_s *dev, FAR struct i2c_msg_s *msgs, int count);
#endif
#ifdef CONFIG_I2C_SLAVE
  int    (*setownaddress)(FAR struct i2c_dev_s *dev, int addr, int nbits);
  int    (*registercallback)(FAR struct i2c_dev_s *dev, int (*callback)(void) );
#endif
};

/* I2C transaction segment beginning with a START.  A number of these can
 * be transfered together to form an arbitrary sequence of write/read transfer
 * to an I2C slave device.
 */

struct i2c_msg_s
{
  uint16_t  addr;                  /* Slave address */
  uint16_t  flags;                 /* See I2C_M_* definitions */
  uint8_t  *buffer;
  int       length;
};

/* I2C private data.  This structure only defines the initial fields of the
 * structure visible to the I2C client.  The specific implementation may 
 * add additional, device specific fields after the vtable.
 */

struct i2c_dev_s
{
  const struct i2c_ops_s *ops; /* I2C vtable */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: up_i2cinitialize
 *
 * Description:
 *   Initialize the selected I2C port. And return a unique instance of struct
 *   struct i2c_dev_s.  This function may be called to obtain multiple
 *   instances of the interface, each of which may be set up with a 
 *   different frequency and slave address.
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple I2C interfaces)
 *
 * Returned Value:
 *   Valid I2C device structre reference on succcess; a NULL on failure
 *
 ****************************************************************************/

EXTERN FAR struct i2c_dev_s *up_i2cinitialize(int port);

/****************************************************************************
 * Name: up_i2cuninitialize
 *
 * Description:
 *   De-initialize the selected I2C port, and power down the device.
 *
 * Input Parameter:
 *   Device structure as returned by the up_i2cinitalize()
 *
 * Returned Value:
 *   OK on success, ERROR when internal reference count missmatch or dev
 *   points to invalid hardware device. 
 *
 ****************************************************************************/

EXTERN int up_i2cuninitialize(FAR struct i2c_dev_s *dev);

/************************************************************************************
 * Name: up_i2creset
 *
 * Description:
 *   Reset an I2C bus
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_RESET
EXTERN int up_i2creset(FAR struct i2c_dev_s *dev);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif







#endif