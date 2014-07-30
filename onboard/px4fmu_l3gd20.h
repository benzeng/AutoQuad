/*
    This file is part of AutoQuad For PX4FMU(Pixhawk)

    

    Copyright (C) 2014  BenZeng
*/
#ifndef __PX4FMU_STM32F247_L3GD20_H
#define __PX4FMU_STM32F247_L3GD20_H

#include "px4fmu_types.h"
#include "LowPassFilter.h"
#include "util.h"
#include <math.h>

/* Orientation on board */
#define SENSOR_BOARD_ROTATION_000_DEG	0
#define SENSOR_BOARD_ROTATION_090_DEG	1
#define SENSOR_BOARD_ROTATION_180_DEG	2
#define SENSOR_BOARD_ROTATION_270_DEG	3

/* SPI protocol address bits */
#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT				(1<<6)

/* register addresses */
#define ADDR_WHO_AM_I			0x0F
#define WHO_I_AM_H 			0xD7
#define WHO_I_AM			0xD4

#define ADDR_CTRL_REG1			0x20
#define REG1_RATE_LP_MASK			0xF0 /* Mask to guard partial register update */
/* keep lowpass low to avoid noise issues */
#define RATE_95HZ_LP_25HZ		((0<<7) | (0<<6) | (0<<5) | (1<<4))
#define RATE_190HZ_LP_25HZ		((0<<7) | (1<<6) | (0<<5) | (1<<4))
#define RATE_190HZ_LP_50HZ		((0<<7) | (1<<6) | (1<<5) | (0<<4))
#define RATE_190HZ_LP_70HZ		((0<<7) | (1<<6) | (1<<5) | (1<<4))
#define RATE_380HZ_LP_20HZ		((1<<7) | (0<<6) | (1<<5) | (0<<4))
#define RATE_380HZ_LP_25HZ		((1<<7) | (0<<6) | (0<<5) | (1<<4))
#define RATE_380HZ_LP_50HZ		((1<<7) | (0<<6) | (1<<5) | (0<<4))
#define RATE_380HZ_LP_100HZ		((1<<7) | (0<<6) | (1<<5) | (1<<4))
#define RATE_760HZ_LP_30HZ		((1<<7) | (1<<6) | (0<<5) | (0<<4))
#define RATE_760HZ_LP_35HZ		((1<<7) | (1<<6) | (0<<5) | (1<<4))
#define RATE_760HZ_LP_50HZ		((1<<7) | (1<<6) | (1<<5) | (0<<4))
#define RATE_760HZ_LP_100HZ		((1<<7) | (1<<6) | (1<<5) | (1<<4))

#define ADDR_CTRL_REG2			0x21
#define ADDR_CTRL_REG3			0x22
#define ADDR_CTRL_REG4			0x23
#define REG4_RANGE_MASK			0x30 /* Mask to guard partial register update */
#define RANGE_250DPS			(0<<4)
#define RANGE_500DPS			(1<<4)
#define RANGE_2000DPS			(3<<4)

#define ADDR_CTRL_REG5			0x24
#define ADDR_REFERENCE			0x25
#define ADDR_OUT_TEMP			0x26
#define ADDR_STATUS_REG			0x27
#define ADDR_OUT_X_L			0x28
#define ADDR_OUT_X_H			0x29
#define ADDR_OUT_Y_L			0x2A
#define ADDR_OUT_Y_H			0x2B
#define ADDR_OUT_Z_L			0x2C
#define ADDR_OUT_Z_H			0x2D
#define ADDR_FIFO_CTRL_REG		0x2E
#define ADDR_FIFO_SRC_REG		0x2F
#define ADDR_INT1_CFG			0x30
#define ADDR_INT1_SRC			0x31
#define ADDR_INT1_TSH_XH		0x32
#define ADDR_INT1_TSH_XL		0x33
#define ADDR_INT1_TSH_YH		0x34
#define ADDR_INT1_TSH_YL		0x35
#define ADDR_INT1_TSH_ZH		0x36
#define ADDR_INT1_TSH_ZL		0x37
#define ADDR_INT1_DURATION		0x38


/* Internal configuration values */
#define REG1_POWER_NORMAL			(1<<3)
#define REG1_Z_ENABLE				(1<<2)
#define REG1_Y_ENABLE				(1<<1)
#define REG1_X_ENABLE				(1<<0)

#define REG4_BDU				(1<<7)
#define REG4_BLE				(1<<6)
//#define REG4_SPI_3WIRE			(1<<0)

#define REG5_FIFO_ENABLE			(1<<6)
#define REG5_REBOOT_MEMORY			(1<<7)

#define STATUS_ZYXOR				(1<<7)
#define STATUS_ZOR				(1<<6)
#define STATUS_YOR				(1<<5)
#define STATUS_XOR				(1<<4)
#define STATUS_ZYXDA				(1<<3)
#define STATUS_ZDA				(1<<2)
#define STATUS_YDA				(1<<1)
#define STATUS_XDA				(1<<0)

#define FIFO_CTRL_BYPASS_MODE			(0<<5)
#define FIFO_CTRL_FIFO_MODE			(1<<5)
#define FIFO_CTRL_STREAM_MODE			(1<<6)
#define FIFO_CTRL_STREAM_TO_FIFO_MODE		(3<<5)
#define FIFO_CTRL_BYPASS_TO_STREAM_MODE		(1<<7)

#define L3GD20_GYO_SCALE                        1         // Ben: what should I do ?
#define L3GD20_DEFAULT_RATE			760
#define L3GD20_DEFAULT_RANGE_DPS		2000
#define L3GD20_DEFAULT_FILTER_FREQ		30

#define L3GD20_SLOTS	    80						    // 100Hz bandwidth
#define L3GD20_DRATE_SLOTS  40						    // 200Hz


#define DIMU_ORIENT_GYO_X	    (-in[0])
#define DIMU_ORIENT_GYO_Y	    (-in[1])
#define DIMU_ORIENT_GYO_Z	    (+in[2])


/**
 * gyro report structure.  Reads from the device must be in multiples of this
 * structure.
 */
struct gyro_report {
    uint64_t timestamp;
    uint64_t error_count;
    float x;		/**< angular velocity in the NED X board axis in rad/s */
    float y;		/**< angular velocity in the NED Y board axis in rad/s */
    float z;		/**< angular velocity in the NED Z board axis in rad/s */
    float temperature;	/**< temperature in degrees celcius */
    float range_rad_s;
    float scaling;

    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
    int16_t temperature_raw;
};

/** gyro scaling factors; Vout = (Vin * Vscale) + Voffset */
struct gyro_scale {
    float	x_offset;
    float	x_scale;
    float	y_offset;
    float	y_scale;
    float	z_offset;
    float	z_scale;
};


typedef struct {

    uint8_t             enabled;
    float               rawGyo[3];
    float               dRateRawGyo[3];
    float               gyoOffset[3];
    volatile float      gyo[3];
    volatile float      dRateGyo[3];
    volatile uint32_t   lastUpdate;

    utilFilter_t tempFilter;
    float rawTemp;
    volatile float temp;

    unsigned		_call_interval;
    OS_TID              task;

    volatile uint8_t    slot;
    struct gyro_report  _gyro_report[L3GD20_SLOTS];
    struct gyro_scale	_gyro_scale;
    float		_gyro_range_scale;
    float		_gyro_range_rad_s;

    unsigned		_current_rate;
    unsigned		_orientation;

    unsigned		_read;

    LowPassFilter2p	_gyro_filter_x;
    LowPassFilter2p	_gyro_filter_y;
    LowPassFilter2p	_gyro_filter_z;
} l3gd20Struct_t;

extern l3gd20Struct_t l3gd20Data;


uint8_t l3gd20Init(void);
void l3gd20Decode(void);




#endif