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

#ifndef _eeprom_h
#define _eeprom_h

#include "spi.h"

#ifdef PX4FMU

#define FM25V01_SPI_BAUD	    SPI_BaudRatePrescaler_2	// 50 Mhz ?

#define DIMU_EEPROM_SIZE	    0x4000    // Flash Ram Size = 16K Bytes
#define DIMU_EEPROM_BLOCK_SIZE	    0x40
#define DIMU_EEPROM_MASK	    0x7fff

#define EEPROM_VERSION		    0x00010001
#define EEPROM_SIGNATURE	    0xdeafbeef

/* RAMTRON Indentification register values */

#define RAMTRON_MANUFACTURER         0x7F
#define RAMTRON_MEMORY_TYPE          0xC2

/* Instructions:
 *      Command          Value       N Description             Addr Dummy Data */
#define RAMTRON_WREN      0x06    /* 1 Write Enable              0   0     0 */
#define RAMTRON_WRDI      0x04    /* 1 Write Disable             0   0     0 */
#define RAMTRON_RDSR      0x05    /* 1 Read Status Register      0   0     >=1 */
#define RAMTRON_WRSR      0x01    /* 1 Write Status Register     0   0     1 */
#define RAMTRON_READ      0x03    /* 1 Read Data Bytes           A   0     >=1 */
#define RAMTRON_FSTRD     0x0b    /* 1 Higher speed read         A   1     >=1 */
#define RAMTRON_WRITE     0x02    /* 1 Write                     A   0     1-256 */
#define RAMTRON_SLEEP     0xb9    // TODO:
#define RAMTRON_RDID      0x9f    /* 1 Read Identification       0   0     1-3 */
#define RAMTRON_SN        0xc3    // TODO:


/* Status register bit definitions */

#define RAMTRON_SR_WIP            (1 << 0)                /* Bit 0: Write in progress bit */
#define RAMTRON_SR_WEL            (1 << 1)                /* Bit 1: Write enable latch bit */
#define RAMTRON_SR_BP_SHIFT       (2)                     /* Bits 2-4: Block protect bits */
#define RAMTRON_SR_BP_MASK        (7 << RAMTRON_SR_BP_SHIFT)
#  define RAMTRON_SR_BP_NONE      (0 << RAMTRON_SR_BP_SHIFT) /* Unprotected */
#  define RAMTRON_SR_BP_UPPER64th (1 << RAMTRON_SR_BP_SHIFT) /* Upper 64th */
#  define RAMTRON_SR_BP_UPPER32nd (2 << RAMTRON_SR_BP_SHIFT) /* Upper 32nd */
#  define RAMTRON_SR_BP_UPPER16th (3 << RAMTRON_SR_BP_SHIFT) /* Upper 16th */
#  define RAMTRON_SR_BP_UPPER8th  (4 << RAMTRON_SR_BP_SHIFT) /* Upper 8th */
#  define RAMTRON_SR_BP_UPPERQTR  (5 << RAMTRON_SR_BP_SHIFT) /* Upper quarter */
#  define RAMTRON_SR_BP_UPPERHALF (6 << RAMTRON_SR_BP_SHIFT) /* Upper half */
#  define RAMTRON_SR_BP_ALL       (7 << RAMTRON_SR_BP_SHIFT) /* All sectors */
#define RAMTRON_SR_SRWD           (1 << 7)                /* Bit 7: Status register write protect */

#define RAMTRON_DUMMY     0xa5

#else //*********** AQ

#define DIMU_EEPROM_SPI_BAUD	    SPI_BaudRatePrescaler_4	// 10.5 MHz

#define DIMU_EEPROM_SIZE	    0x8000
#define DIMU_EEPROM_BLOCK_SIZE	    0x40
#define DIMU_EEPROM_MASK	    0x7fff
//#define DIUM_EEPROM_SIZE		    0x10000
//#define DIUM_EEPROM_BLOCK_SIZE	    0x80
//#define DIUM_EEPROM_MASK		    0xffff

#define EEPROM_WREN		    0b0110
#define EEPROM_WRDI		    0b0100
#define EEPROM_RDSR		    0b0101
#define EEPROM_WRSR		    0b0001
#define EEPROM_READ		    0b0011
#define EEPROM_WRITE		    0b0010

#define EEPROM_VERSION		    0x00010001
#define EEPROM_SIGNATURE	    0xdeafbeef

#endif //// #ifdef PX4FMU


typedef struct {
    uint8_t cmd;
    uint8_t addr[2];
    uint8_t data[DIMU_EEPROM_BLOCK_SIZE];
}  __attribute__((packed)) eepromBuf_t;

typedef struct {
    uint32_t signature;
    uint32_t version;
    uint32_t seq;
    uint16_t size;
    uint16_t start;
    uint8_t fileCk[2];
    uint8_t headerCk[2];
} __attribute__((packed)) eepromHeader_t;

typedef struct {
    spiClient_t *spi;
    volatile uint32_t spiFlag;
    eepromBuf_t buf;
    eepromHeader_t header;
    uint16_t readPointer;
    uint8_t ck[2];
    uint8_t status;
} eepromStruct_t;

extern void eepromPreInit(void);
extern void eepromInit(void);
extern uint8_t *eepromOpenWrite(void);
extern void eepromWrite(void);
extern void eepromClose(void);
extern uint8_t *eepromOpenRead(void);
extern uint8_t eepromRead(int size);
extern void eepromTest( void );
#endif