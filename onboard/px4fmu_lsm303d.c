/*
    This file is part of AutoQuad For PX4FMU(Pixhawk)

    

    Copyright (C) 2014  BenZeng
*/
#include "CoOS.h"
#include "px4fmu_config.h"
#include "px4fmu_types.h"
#include "px4fmu_board.h"
#include "px4fmu_spi.h"
#include "px4fmu_rcc.h"
#include "px4fmu_errno.h"
#include "LowPassFilter.h"
#include "px4fmu_lsm303d.h"

#include <math.h>
#include <string.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/* SPI protocol address bits */
#define DIR_READ			(1<<7)
#define DIR_WRITE			(0<<7)
#define ADDR_INCREMENT			(1<<6)

/* register addresses: A: accel, M: mag, T: temp */
#define ADDR_WHO_AM_I			0x0F
#define WHO_I_AM			0x49

#define ADDR_OUT_TEMP_L			0x05
#define ADDR_OUT_TEMP_H			0x06
#define ADDR_STATUS_M			0x07
#define ADDR_OUT_X_L_M          	0x08
#define ADDR_OUT_X_H_M          	0x09
#define ADDR_OUT_Y_L_M          	0x0A
#define ADDR_OUT_Y_H_M			0x0B
#define ADDR_OUT_Z_L_M			0x0C
#define ADDR_OUT_Z_H_M			0x0D

#define ADDR_INT_CTRL_M			0x12
#define ADDR_INT_SRC_M			0x13
#define ADDR_REFERENCE_X		0x1c
#define ADDR_REFERENCE_Y		0x1d
#define ADDR_REFERENCE_Z		0x1e

#define ADDR_STATUS_A			0x27
#define ADDR_OUT_X_L_A			0x28
#define ADDR_OUT_X_H_A			0x29
#define ADDR_OUT_Y_L_A			0x2A
#define ADDR_OUT_Y_H_A			0x2B
#define ADDR_OUT_Z_L_A			0x2C
#define ADDR_OUT_Z_H_A			0x2D

#define ADDR_CTRL_REG0			0x1F
#define ADDR_CTRL_REG1			0x20
#define ADDR_CTRL_REG2			0x21
#define ADDR_CTRL_REG3			0x22
#define ADDR_CTRL_REG4			0x23
#define ADDR_CTRL_REG5			0x24
#define ADDR_CTRL_REG6			0x25
#define ADDR_CTRL_REG7			0x26

#define ADDR_FIFO_CTRL			0x2e
#define ADDR_FIFO_SRC			0x2f

#define ADDR_IG_CFG1			0x30
#define ADDR_IG_SRC1			0x31
#define ADDR_IG_THS1			0x32
#define ADDR_IG_DUR1			0x33
#define ADDR_IG_CFG2			0x34
#define ADDR_IG_SRC2			0x35
#define ADDR_IG_THS2			0x36
#define ADDR_IG_DUR2			0x37
#define ADDR_CLICK_CFG			0x38
#define ADDR_CLICK_SRC			0x39
#define ADDR_CLICK_THS			0x3a
#define ADDR_TIME_LIMIT			0x3b
#define ADDR_TIME_LATENCY		0x3c
#define ADDR_TIME_WINDOW		0x3d
#define ADDR_ACT_THS			0x3e
#define ADDR_ACT_DUR			0x3f

#define REG1_RATE_BITS_A		((1<<7) | (1<<6) | (1<<5) | (1<<4))
#define REG1_POWERDOWN_A		((0<<7) | (0<<6) | (0<<5) | (0<<4))
#define REG1_RATE_3_125HZ_A		((0<<7) | (0<<6) | (0<<5) | (1<<4))
#define REG1_RATE_6_25HZ_A		((0<<7) | (0<<6) | (1<<5) | (0<<4))
#define REG1_RATE_12_5HZ_A		((0<<7) | (0<<6) | (1<<5) | (1<<4))
#define REG1_RATE_25HZ_A		((0<<7) | (1<<6) | (0<<5) | (0<<4))
#define REG1_RATE_50HZ_A		((0<<7) | (1<<6) | (0<<5) | (1<<4))
#define REG1_RATE_100HZ_A		((0<<7) | (1<<6) | (1<<5) | (0<<4))
#define REG1_RATE_200HZ_A		((0<<7) | (1<<6) | (1<<5) | (1<<4))
#define REG1_RATE_400HZ_A		((1<<7) | (0<<6) | (0<<5) | (0<<4))
#define REG1_RATE_800HZ_A		((1<<7) | (0<<6) | (0<<5) | (1<<4))
#define REG1_RATE_1600HZ_A		((1<<7) | (0<<6) | (1<<5) | (0<<4))

#define REG1_BDU_UPDATE			(1<<3)
#define REG1_Z_ENABLE_A			(1<<2)
#define REG1_Y_ENABLE_A			(1<<1)
#define REG1_X_ENABLE_A			(1<<0)

#define REG2_ANTIALIAS_FILTER_BW_BITS_A	((1<<7) | (1<<6))
#define REG2_AA_FILTER_BW_773HZ_A		((0<<7) | (0<<6))
#define REG2_AA_FILTER_BW_194HZ_A		((0<<7) | (1<<6))
#define REG2_AA_FILTER_BW_362HZ_A		((1<<7) | (0<<6))
#define REG2_AA_FILTER_BW_50HZ_A		((1<<7) | (1<<6))

#define REG2_FULL_SCALE_BITS_A	((1<<5) | (1<<4) | (1<<3))
#define REG2_FULL_SCALE_2G_A	((0<<5) | (0<<4) | (0<<3))
#define REG2_FULL_SCALE_4G_A	((0<<5) | (0<<4) | (1<<3))
#define REG2_FULL_SCALE_6G_A	((0<<5) | (1<<4) | (0<<3))
#define REG2_FULL_SCALE_8G_A	((0<<5) | (1<<4) | (1<<3))
#define REG2_FULL_SCALE_16G_A	((1<<5) | (0<<4) | (0<<3))

#define REG5_ENABLE_T			(1<<7)

#define REG5_RES_HIGH_M			((1<<6) | (1<<5))
#define REG5_RES_LOW_M			((0<<6) | (0<<5))

#define REG5_RATE_BITS_M		((1<<4) | (1<<3) | (1<<2))
#define REG5_RATE_3_125HZ_M		((0<<4) | (0<<3) | (0<<2))
#define REG5_RATE_6_25HZ_M		((0<<4) | (0<<3) | (1<<2))
#define REG5_RATE_12_5HZ_M		((0<<4) | (1<<3) | (0<<2))
#define REG5_RATE_25HZ_M		((0<<4) | (1<<3) | (1<<2))
#define REG5_RATE_50HZ_M		((1<<4) | (0<<3) | (0<<2))
#define REG5_RATE_100HZ_M		((1<<4) | (0<<3) | (1<<2))
#define REG5_RATE_DO_NOT_USE_M	((1<<4) | (1<<3) | (0<<2))

#define REG6_FULL_SCALE_BITS_M	((1<<6) | (1<<5))
#define REG6_FULL_SCALE_2GA_M	((0<<6) | (0<<5))
#define REG6_FULL_SCALE_4GA_M	((0<<6) | (1<<5))
#define REG6_FULL_SCALE_8GA_M	((1<<6) | (0<<5))
#define REG6_FULL_SCALE_12GA_M	((1<<6) | (1<<5))

#define REG7_CONT_MODE_M		((0<<1) | (0<<0))


#define INT_CTRL_M              0x12
#define INT_SRC_M               0x13

/* default values for this device */
#define LSM303D_ACCEL_DEFAULT_RANGE_G			8
#define LSM303D_ACCEL_DEFAULT_RATE			800
#define LSM303D_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ	50
#define LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define LSM303D_MAG_DEFAULT_RANGE_GA			2
#define LSM303D_MAG_DEFAULT_RATE			100

#define LSM303D_ONE_G					9.80665f


#define LSM303D_STACK_SIZE          250
#define LSM303D_PRIORITY	    11
static OS_STK *lsm303dTaskStack;

// LSM303D Instance
static SPI_INSTANCE_DATA spiLSM303D;
lsm303dStruct_t lsm303dData;




uint8_t
LSM303D_read_reg(SPI_INSTANCE_DATA *pThis, unsigned reg)
{
    uint8_t cmd[2];

    cmd[0] = reg | DIR_READ;
    cmd[1] = 0;

    SpiTransfer(pThis, cmd, cmd, sizeof(cmd));

    return cmd[1];
}

void
LSM303D_write_reg(SPI_INSTANCE_DATA *pThis, unsigned reg, uint8_t value)
{
    uint8_t	cmd[2];

    cmd[0] = reg | DIR_WRITE;
    cmd[1] = value;

    SpiTransfer(pThis, cmd, NULL, sizeof(cmd));
}

void
LSM303D_modify_reg(SPI_INSTANCE_DATA *pThis, unsigned reg, uint8_t clearbits, uint8_t setbits)
{
    uint8_t	val;

    val = LSM303D_read_reg(pThis, reg);
    val &= ~clearbits;
    val |= setbits;
    LSM303D_write_reg(pThis, reg, val);
}

int
LSM303D_accel_set_range(SPI_INSTANCE_DATA *pThis, unsigned max_g)
{
    uint8_t setbits = 0;
    uint8_t clearbits = REG2_FULL_SCALE_BITS_A;
    float new_scale_g_digit = 0.0f;

    lsm303dStruct_t *plsm303dData = (lsm303dStruct_t *)pThis->pData;

    if (max_g == 0)
	max_g = 16;

    if (max_g <= 2) {
	plsm303dData->_accel_range_m_s2 = 2.0f*LSM303D_ONE_G;
	setbits |= REG2_FULL_SCALE_2G_A;
	new_scale_g_digit = 0.061e-3f;

    } else if (max_g <= 4) {
	plsm303dData->_accel_range_m_s2 = 4.0f*LSM303D_ONE_G;
	setbits |= REG2_FULL_SCALE_4G_A;
	new_scale_g_digit = 0.122e-3f;

    } else if (max_g <= 6) {
	plsm303dData->_accel_range_m_s2 = 6.0f*LSM303D_ONE_G;
	setbits |= REG2_FULL_SCALE_6G_A;
	new_scale_g_digit = 0.183e-3f;

    } else if (max_g <= 8) {
	plsm303dData->_accel_range_m_s2 = 8.0f*LSM303D_ONE_G;
	setbits |= REG2_FULL_SCALE_8G_A;
	new_scale_g_digit = 0.244e-3f;

    } else if (max_g <= 16) {
	plsm303dData->_accel_range_m_s2 = 16.0f*LSM303D_ONE_G;
	setbits |= REG2_FULL_SCALE_16G_A;
	new_scale_g_digit = 0.732e-3f;

    } else {
	return -EINVAL;
    }

    plsm303dData->_accel_range_scale = new_scale_g_digit * LSM303D_ONE_G;


    LSM303D_modify_reg(pThis, ADDR_CTRL_REG2, clearbits, setbits);

    return OK;
}

int
LSM303D_mag_set_range(SPI_INSTANCE_DATA *pThis, unsigned max_ga)
{
    uint8_t setbits = 0;
    uint8_t clearbits = REG6_FULL_SCALE_BITS_M;
    float new_scale_ga_digit = 0.0f;

    lsm303dStruct_t *plsm303dData = (lsm303dStruct_t *)pThis->pData;

    if (max_ga == 0)
	max_ga = 12;

    if (max_ga <= 2) {
	plsm303dData->_mag_range_ga = 2;
	setbits |= REG6_FULL_SCALE_2GA_M;
	new_scale_ga_digit = 0.080e-3f;

    } else if (max_ga <= 4) {
	plsm303dData->_mag_range_ga = 4;
	setbits |= REG6_FULL_SCALE_4GA_M;
	new_scale_ga_digit = 0.160e-3f;

    } else if (max_ga <= 8) {
	plsm303dData->_mag_range_ga = 8;
	setbits |= REG6_FULL_SCALE_8GA_M;
	new_scale_ga_digit = 0.320e-3f;

    } else if (max_ga <= 12) {
	plsm303dData->_mag_range_ga = 12;
	setbits |= REG6_FULL_SCALE_12GA_M;
	new_scale_ga_digit = 0.479e-3f;

    } else {
	return -EINVAL;
    }

    plsm303dData->_mag_range_scale = new_scale_ga_digit;

    LSM303D_modify_reg(pThis, ADDR_CTRL_REG6, clearbits, setbits);

    return OK;
}

int
LSM303D_accel_set_onchip_lowpass_filter_bandwidth(SPI_INSTANCE_DATA *pThis, unsigned bandwidth)
{
    uint8_t setbits = 0;
    uint8_t clearbits = REG2_ANTIALIAS_FILTER_BW_BITS_A;

    lsm303dStruct_t *plsm303dData = (lsm303dStruct_t *)pThis->pData;

    if (bandwidth == 0)
	bandwidth = 773;

    if (bandwidth <= 50) {
	setbits |= REG2_AA_FILTER_BW_50HZ_A;
	plsm303dData->_accel_onchip_filter_bandwith = 50;

    } else if (bandwidth <= 194) {
	setbits |= REG2_AA_FILTER_BW_194HZ_A;
	plsm303dData->_accel_onchip_filter_bandwith = 194;

    } else if (bandwidth <= 362) {
	setbits |= REG2_AA_FILTER_BW_362HZ_A;
	plsm303dData->_accel_onchip_filter_bandwith = 362;

    } else if (bandwidth <= 773) {
	setbits |= REG2_AA_FILTER_BW_773HZ_A;
	plsm303dData->_accel_onchip_filter_bandwith = 773;

    } else {
	return -EINVAL;
    }

    LSM303D_modify_reg(pThis, ADDR_CTRL_REG2, clearbits, setbits);

    return OK;
}

int
LSM303D_accel_set_driver_lowpass_filter(SPI_INSTANCE_DATA *pThis, float samplerate, float bandwidth)
{
    lsm303dStruct_t *plsm303dData = (lsm303dStruct_t *)pThis->pData;

    LowPassFilter2p_set_cutoff_frequency(&plsm303dData->_accel_filter_x, samplerate, bandwidth);
    LowPassFilter2p_set_cutoff_frequency(&plsm303dData->_accel_filter_y, samplerate, bandwidth);
    LowPassFilter2p_set_cutoff_frequency(&plsm303dData->_accel_filter_z, samplerate, bandwidth);

    return OK;
}

int
LSM303D_accel_set_samplerate(SPI_INSTANCE_DATA *pThis, unsigned frequency)
{
    uint8_t setbits = 0;
    uint8_t clearbits = REG1_RATE_BITS_A;

    lsm303dStruct_t *plsm303dData = (lsm303dStruct_t *)pThis->pData;

    if (frequency == 0)
	frequency = 1600;

    if (frequency <= 100) {
	setbits |= REG1_RATE_100HZ_A;
	plsm303dData->_accel_samplerate = 100;

    } else if (frequency <= 200) {
	setbits |= REG1_RATE_200HZ_A;
	plsm303dData->_accel_samplerate = 200;

    } else if (frequency <= 400) {
	setbits |= REG1_RATE_400HZ_A;
	plsm303dData->_accel_samplerate = 400;

    } else if (frequency <= 800) {
	setbits |= REG1_RATE_800HZ_A;
	plsm303dData->_accel_samplerate = 800;

    } else if (frequency <= 1600) {
	setbits |= REG1_RATE_1600HZ_A;
	plsm303dData->_accel_samplerate = 1600;

    } else {
	return -EINVAL;
    }

    LSM303D_modify_reg(pThis, ADDR_CTRL_REG1, clearbits, setbits);
    plsm303dData->_reg1_expected = (plsm303dData->_reg1_expected & ~clearbits) | setbits;

    return OK;
}

int
LSM303D_mag_set_samplerate(SPI_INSTANCE_DATA *pThis, unsigned frequency)
{
    uint8_t setbits = 0;
    uint8_t clearbits = REG5_RATE_BITS_M;

    lsm303dStruct_t *plsm303dData = (lsm303dStruct_t *)pThis->pData;

    if (frequency == 0)
	frequency = 100;

    if (frequency <= 25) {
	setbits |= REG5_RATE_25HZ_M;
	plsm303dData->_mag_samplerate = 25;

    } else if (frequency <= 50) {
	setbits |= REG5_RATE_50HZ_M;
	plsm303dData->_mag_samplerate = 50;

    } else if (frequency <= 100) {
	setbits |= REG5_RATE_100HZ_M;
	plsm303dData->_mag_samplerate = 100;

    } else {
	return -EINVAL;
    }

    LSM303D_modify_reg(pThis, ADDR_CTRL_REG5, clearbits, setbits);

    return OK;
}


void
LSM303D_disable_i2c(SPI_INSTANCE_DATA *pThis)
{
    uint8_t a = LSM303D_read_reg(pThis, 0x02);
    LSM303D_write_reg(pThis, 0x02, (0x10 | a));
    a = LSM303D_read_reg(pThis, 0x02);
    LSM303D_write_reg(pThis, 0x02, (0xF7 & a));
    a = LSM303D_read_reg(pThis, 0x15);
    LSM303D_write_reg(pThis, 0x15, (0x80 | a));
    a = LSM303D_read_reg(pThis, 0x02);
    LSM303D_write_reg(pThis, 0x02, (0xE7 & a));
}

void
LSM303D_reset(SPI_INSTANCE_DATA *pThis)
{
    lsm303dStruct_t *plsm303dData = (lsm303dStruct_t *)pThis->pData;

    // ensure the chip doesn't interpret any other bus traffic as I2C
    LSM303D_disable_i2c(pThis);

    /* enable accel*/
    plsm303dData->_reg1_expected = REG1_X_ENABLE_A | REG1_Y_ENABLE_A | REG1_Z_ENABLE_A | REG1_BDU_UPDATE | REG1_RATE_800HZ_A;
    LSM303D_write_reg(pThis, ADDR_CTRL_REG1, plsm303dData->_reg1_expected);

    /* enable mag */
    plsm303dData->_reg7_expected = REG7_CONT_MODE_M;
    LSM303D_write_reg(pThis, ADDR_CTRL_REG7, plsm303dData->_reg7_expected);
    LSM303D_write_reg(pThis, ADDR_CTRL_REG5, REG5_RES_HIGH_M);
    LSM303D_write_reg(pThis, ADDR_CTRL_REG3, 0x04); // DRDY on ACCEL on INT1
    LSM303D_write_reg(pThis, ADDR_CTRL_REG4, 0x04); // DRDY on MAG on INT2

    LSM303D_accel_set_range(pThis, LSM303D_ACCEL_DEFAULT_RANGE_G);
    LSM303D_accel_set_samplerate(pThis, LSM303D_ACCEL_DEFAULT_RATE);
    LSM303D_accel_set_driver_lowpass_filter(pThis, (float)LSM303D_ACCEL_DEFAULT_RATE, (float)LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ);

    // we setup the anti-alias on-chip filter as 50Hz. We believe
    // this operates in the analog domain, and is critical for
    // anti-aliasing. The 2 pole software filter is designed to
    // operate in conjunction with this on-chip filter
    LSM303D_accel_set_onchip_lowpass_filter_bandwidth(pThis, LSM303D_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ);

    LSM303D_mag_set_range(pThis, LSM303D_MAG_DEFAULT_RANGE_GA);
    LSM303D_mag_set_samplerate(pThis, LSM303D_MAG_DEFAULT_RATE);

    plsm303dData->_accel_read = 0;
    plsm303dData->_mag_read = 0;
}

int
LSM303D_probe(SPI_INSTANCE_DATA *pThis)
{
    /* read dummy value to void to clear SPI statemachine on sensor */
    (void)LSM303D_read_reg(pThis, ADDR_WHO_AM_I);

    /* verify that the device is attached and functioning */
    bool success = (LSM303D_read_reg(pThis, ADDR_WHO_AM_I) == WHO_I_AM);
    
    if (success)
	return OK;

    return -EIO;
}


void
LSM303D_measure( SPI_INSTANCE_DATA *pThis )
{
    lsm303dStruct_t *plsm303dData = (lsm303dStruct_t *)pThis->pData;

    // if the accel doesn't have any data ready then re-schedule
    // for 100 microseconds later. This ensures we don't double
    // read a value and then miss the next value
    if (stm32_gpioread(GPIO_EXTI_ACCEL_DRDY) == 0) {
	//perf_count(_accel_reschedules);
	//hrt_call_delay(&_accel_call, 100);
        delay(100);
	return;
    }
    if (LSM303D_read_reg(pThis, ADDR_CTRL_REG1) != plsm303dData->_reg1_expected) {
	//perf_count(plsm303dData->_reg1_resets);
	LSM303D_reset( pThis );
	return;
    }

    /* status register and data as read back from the device */

#pragma pack(push, 1)
    struct {
	uint8_t		cmd;
	uint8_t		status;
	int16_t		x;
	int16_t		y;
	int16_t		z;
    } raw_accel_report;
#pragma pack(pop)

    //accel_report accel_report;

    /* start the performance counter */
    //perf_begin(_accel_sample_perf);

    /* fetch data from the sensor */
    memset(&raw_accel_report, 0, sizeof(raw_accel_report));
    raw_accel_report.cmd = ADDR_STATUS_A | DIR_READ | ADDR_INCREMENT;
    SpiTransfer(pThis, (uint8_t *)&raw_accel_report, (uint8_t *)&raw_accel_report, sizeof(raw_accel_report));

    /*
    * 1) Scale raw value to SI units using scaling from datasheet.
    * 2) Subtract static offset (in SI units)
    * 3) Scale the statically calibrated values with a linear
    *    dynamically obtained factor
    *
    * Note: the static sensor offset is the number the sensor outputs
    * 	 at a nominally 'zero' input. Therefore the offset has to
    * 	 be subtracted.
    *
    *	 Example: A gyro outputs a value of 74 at zero angular rate
    *	 	  the offset is 74 from the origin and subtracting
    *		  74 from all measurements centers them around zero.
    */


    plsm303dData->accel_report.timestamp = GetTimerMicros(); //hrt_absolute_time();
    plsm303dData->accel_report.error_count = 0; // not reported

    plsm303dData->accel_report.x_raw = raw_accel_report.x;
    plsm303dData->accel_report.y_raw = raw_accel_report.y;
    plsm303dData->accel_report.z_raw = raw_accel_report.z;

    float x_in_new = ((plsm303dData->accel_report.x_raw * plsm303dData->_accel_range_scale) - plsm303dData->_accel_scale.x_offset) * plsm303dData->_accel_scale.x_scale;
    float y_in_new = ((plsm303dData->accel_report.y_raw * plsm303dData->_accel_range_scale) - plsm303dData->_accel_scale.y_offset) * plsm303dData->_accel_scale.y_scale;
    float z_in_new = ((plsm303dData->accel_report.z_raw * plsm303dData->_accel_range_scale) - plsm303dData->_accel_scale.z_offset) * plsm303dData->_accel_scale.z_scale;

    plsm303dData->accel_report.x = LowPassFilter2p_apply(&plsm303dData->_accel_filter_x, x_in_new);
    plsm303dData->accel_report.y = LowPassFilter2p_apply(&plsm303dData->_accel_filter_y, y_in_new);
    plsm303dData->accel_report.z = LowPassFilter2p_apply(&plsm303dData->_accel_filter_z, z_in_new);

    plsm303dData->accel_report.scaling = plsm303dData->_accel_range_scale;
    plsm303dData->accel_report.range_m_s2 = plsm303dData->_accel_range_m_s2;

    //_accel_reports->force(&accel_report);

    /* notify anyone waiting for data */
    //poll_notify(POLLIN);

    //if (_accel_topic > 0 && !(_pub_blocked)) {
	///* publish it */
	//orb_publish(ORB_ID(sensor_accel), _accel_topic, &accel_report);
    //}

    plsm303dData->_accel_read++;

    /* stop the perf counter */
    //perf_end(_accel_sample_perf);
}

void
LSM303D_mag_measure( SPI_INSTANCE_DATA *pThis )
{
    lsm303dStruct_t *plsm303dData = (lsm303dStruct_t *)pThis->pData;

    if (LSM303D_read_reg(pThis, ADDR_CTRL_REG7) != plsm303dData->_reg7_expected) {
	//perf_count(_reg7_resets);
	LSM303D_reset( pThis );
	return;
    }

    /* status register and data as read back from the device */
#pragma pack(push, 1)
    struct {
	uint8_t		cmd;
	uint8_t		status;
	int16_t		x;
	int16_t		y;
	int16_t		z;
    } raw_mag_report;
#pragma pack(pop)

    //mag_report mag_report;

    /* start the performance counter */
    //perf_begin(_mag_sample_perf);

    /* fetch data from the sensor */
    memset(&raw_mag_report, 0, sizeof(raw_mag_report));
    raw_mag_report.cmd = ADDR_STATUS_M | DIR_READ | ADDR_INCREMENT;
    SpiTransfer(pThis, (uint8_t *)&raw_mag_report, (uint8_t *)&raw_mag_report, sizeof(raw_mag_report));

    /*
    * 1) Scale raw value to SI units using scaling from datasheet.
    * 2) Subtract static offset (in SI units)
    * 3) Scale the statically calibrated values with a linear
    *    dynamically obtained factor
    *
    * Note: the static sensor offset is the number the sensor outputs
    * 	 at a nominally 'zero' input. Therefore the offset has to
    * 	 be subtracted.
    *
    *	 Example: A gyro outputs a value of 74 at zero angular rate
    *	 	  the offset is 74 from the origin and subtracting
    *		  74 from all measurements centers them around zero.
    */


    plsm303dData->mag_report.timestamp = GetTimerMicros(); //hrt_absolute_time();

    plsm303dData->mag_report.x_raw = raw_mag_report.x;
    plsm303dData->mag_report.y_raw = raw_mag_report.y;
    plsm303dData->mag_report.z_raw = raw_mag_report.z;
    plsm303dData->mag_report.x = ((plsm303dData->mag_report.x_raw * plsm303dData->_mag_range_scale) - plsm303dData->_mag_scale.x_offset) * plsm303dData->_mag_scale.x_scale;
    plsm303dData->mag_report.y = ((plsm303dData->mag_report.y_raw * plsm303dData->_mag_range_scale) - plsm303dData->_mag_scale.y_offset) * plsm303dData->_mag_scale.y_scale;
    plsm303dData->mag_report.z = ((plsm303dData->mag_report.z_raw * plsm303dData->_mag_range_scale) - plsm303dData->_mag_scale.z_offset) * plsm303dData->_mag_scale.z_scale;
    plsm303dData->mag_report.scaling = plsm303dData->_mag_range_scale;
    plsm303dData->mag_report.range_ga = (float)plsm303dData->_mag_range_ga;

    //_mag_reports->force(&mag_report);

    /* XXX please check this poll_notify, is it the right one? */
    /* notify anyone waiting for data */
    //poll_notify(POLLIN);

    //if (_mag->_mag_topic > 0 && !(_pub_blocked)) {
	///* publish it */
	//orb_publish(ORB_ID(sensor_mag), _mag->_mag_topic, &mag_report);
    //}

    plsm303dData->_mag_read++;

    /* stop the perf counter */
    //perf_end(_mag_sample_perf);
}

// ACCEL Task
static void lsm303dTaskCode1(void *pInst )
{
    SPI_INSTANCE_DATA *pThis = (SPI_INSTANCE_DATA *)pInst;
    lsm303dStruct_t *plsm303dData = (lsm303dStruct_t *)pThis->pData;

    yield(10);
    while(1)
    {
	LSM303D_measure( pThis );

	// Every interval
	yield( plsm303dData->_call_accel_interval );
    }
}

// MAG Task
static void lsm303dTaskCode2(void *pInst )
{
    SPI_INSTANCE_DATA *pThis = (SPI_INSTANCE_DATA *)pInst;
    lsm303dStruct_t *plsm303dData = (lsm303dStruct_t *)pThis->pData;

    yield(10);
    while(1)
    {
	LSM303D_mag_measure( pThis );

	// Every interval
	yield( plsm303dData->_call_mag_interval );
    }
}


uint8_t lsm303dInit(void)
{
    // default scale factors
    lsm303dData._accel_scale.x_offset = 0.0f;
    lsm303dData._accel_scale.x_scale  = 1.0f;
    lsm303dData._accel_scale.y_offset = 0.0f;
    lsm303dData._accel_scale.y_scale  = 1.0f;
    lsm303dData._accel_scale.z_offset = 0.0f;
    lsm303dData._accel_scale.z_scale  = 1.0f;

    lsm303dData._mag_scale.x_offset = 0.0f;
    lsm303dData._mag_scale.x_scale = 1.0f;
    lsm303dData._mag_scale.y_offset = 0.0f;
    lsm303dData._mag_scale.y_scale = 1.0f;
    lsm303dData._mag_scale.z_offset = 0.0f;
    lsm303dData._mag_scale.z_scale = 1.0f;

    lsm303dData._call_accel_interval = 100;
    lsm303dData._call_mag_interval = 100;

    spiLSM303D.pData = &lsm303dData;
    spiLSM303D._bus = 1;
    spiLSM303D._device = PX4_SPIDEV_ACCEL_MAG;
    spiLSM303D._mode = SPIDEV_MODE3;
    spiLSM303D._frequency = 8000000;
    spiLSM303D._dev = up_spiinitialize(spiLSM303D._bus);
    if( spiLSM303D._dev == NULL )
	return 0;
    /* deselect device to ensure high to low transition of pin select */
    SPI_SELECT(spiLSM303D._dev, spiLSM303D._device, false);

    LSM303D_reset( &spiLSM303D );
    /* fill report structures */
    LSM303D_measure( &spiLSM303D );
    LSM303D_mag_measure( &spiLSM303D );

    lsm303dTaskStack = aqStackInit(LSM303D_STACK_SIZE, "DIMU_LSM303D_ACCEL");
    CoCreateTask(lsm303dTaskCode1, (void *)&spiLSM303D, LSM303D_PRIORITY, &lsm303dTaskStack[LSM303D_STACK_SIZE-1], LSM303D_STACK_SIZE);

    lsm303dTaskStack = aqStackInit(LSM303D_STACK_SIZE, "DIMU_LSM303D_MAG");
    CoCreateTask(lsm303dTaskCode2, (void *)&spiLSM303D, LSM303D_PRIORITY, &lsm303dTaskStack[LSM303D_STACK_SIZE-1], LSM303D_STACK_SIZE);

    return 1;
}


void lsm303dDecode(void)
{
    lsm303dData.lastUpdateMag = lsm303dData.mag_report.timestamp;
    lsm303dData.mag[0] = lsm303dData.mag_report.x;
    lsm303dData.mag[1] = lsm303dData.mag_report.y;
    lsm303dData.mag[2] = lsm303dData.mag_report.z;

    lsm303dData.lastUpdateAccel = lsm303dData.accel_report.timestamp;
    lsm303dData.acc[0] = lsm303dData.accel_report.x;
    lsm303dData.acc[1] = lsm303dData.accel_report.y;
    lsm303dData.acc[2] = lsm303dData.accel_report.z;
}