/*
    This file is part of AutoQuad For PX4FMU(Pixhawk)

    

    Copyright (C) 2014  BenZeng
*/
#include "px4fmu_config.h"
#include "px4fmu_types.h"
#include "util.h"
#include "px4fmu_board.h"
#include "px4fmu_spi.h"
#include "px4fmu_l3gd20.h"
#include "px4fmu_rcc.h"
#include "LowPassFilter.h"
#include "CoOS.h"
#include "aq_timer.h"
#include "d_imu.h"
#include "imu.h"
#include "config.h"
#include <math.h>
#include <string.h>

// L3GD20 Instance
static SPI_INSTANCE_DATA spiL3GD20;
l3gd20Struct_t l3gd20Data;

#define L3GD20_STACK_SIZE   250
#define L3GD20_PRIORITY	    11
static OS_STK *l3gd20TaskStack;


/******************************************************************************/
/* Do something just like l3gd20 driver                                       */
/******************************************************************************/
static uint8_t read_reg(SPI_INSTANCE_DATA *pThis, unsigned reg)
{
    uint8_t cmd[2];

    cmd[0] = reg | DIR_READ;
    cmd[1] = 0;

    //transfer(cmd, cmd, sizeof(cmd));
    SpiTransfer(pThis, cmd, cmd, sizeof(cmd));

    return cmd[1];
}

static void write_reg(SPI_INSTANCE_DATA *pThis, unsigned reg, uint8_t value)
{
    uint8_t	cmd[2];

    cmd[0] = reg | DIR_WRITE;
    cmd[1] = value;

    //transfer(cmd, nullptr, sizeof(cmd));
    SpiTransfer(pThis, cmd, NULL, sizeof(cmd));
}

static void modify_reg(SPI_INSTANCE_DATA *pThis, unsigned reg, uint8_t clearbits, uint8_t setbits)
{
    uint8_t	val;

    val = read_reg(pThis, reg);
    val &= ~clearbits;
    val |= setbits;
    write_reg(pThis, reg, val);
}

static int set_range(SPI_INSTANCE_DATA *pThis, unsigned max_dps)
{
    uint8_t bits = REG4_BDU;
    float new_range_scale_dps_digit;
    float new_range;

    l3gd20Struct_t *pL3GD20Data = (l3gd20Struct_t *)pThis->pData;

    if (max_dps == 0) {
	max_dps = 2000;
    }
    if (max_dps <= 250) {
	new_range = 250;
	bits |= RANGE_250DPS;
	new_range_scale_dps_digit = 8.75e-3f;

    } else if (max_dps <= 500) {
	new_range = 500;
	bits |= RANGE_500DPS;
	new_range_scale_dps_digit = 17.5e-3f;

    } else if (max_dps <= 2000) {
	new_range = 2000;
	bits |= RANGE_2000DPS;
	new_range_scale_dps_digit = 70e-3f;

    } else {
	return -EINVAL;
    }

    pL3GD20Data->_gyro_range_rad_s = new_range / 180.0f * M_PI_F;
    pL3GD20Data->_gyro_range_scale = new_range_scale_dps_digit / 180.0f * M_PI_F;
    write_reg(pThis, ADDR_CTRL_REG4, bits);

    return OK;
}


static bool probe( SPI_INSTANCE_DATA *pThis )
{
    bool success = false;
    l3gd20Struct_t *pL3GD20Data = (l3gd20Struct_t *)pThis->pData;

    /* read dummy value to void to clear SPI statemachine on sensor */
    (void)read_reg(pThis, ADDR_WHO_AM_I);

    /* verify that the device is attached and functioning, accept L3GD20 and L3GD20H */
    if (read_reg(pThis, ADDR_WHO_AM_I) == WHO_I_AM) {
	pL3GD20Data->_orientation = SENSOR_BOARD_ROTATION_270_DEG;
	success = true;
    }

    if (read_reg(pThis, ADDR_WHO_AM_I) == WHO_I_AM_H) {
	pL3GD20Data->_orientation = SENSOR_BOARD_ROTATION_180_DEG;
	success = true;
    }

    return success;
}

static int set_samplerate(SPI_INSTANCE_DATA *pThis, unsigned frequency)
{
    uint8_t bits = REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE;

    l3gd20Struct_t *pL3GD20Data = (l3gd20Struct_t *)pThis->pData;

    if (frequency == 0)
	frequency = 760;

    /* use limits good for H or non-H models */
    if (frequency <= 100) {
	pL3GD20Data->_current_rate = 95;
	bits |= RATE_95HZ_LP_25HZ;

    } else if (frequency <= 200) {
	pL3GD20Data->_current_rate = 190;
	bits |= RATE_190HZ_LP_50HZ;

    } else if (frequency <= 400) {
	pL3GD20Data->_current_rate = 380;
	bits |= RATE_380HZ_LP_50HZ;

    } else if (frequency <= 800) {
	pL3GD20Data->_current_rate = 760;
	bits |= RATE_760HZ_LP_50HZ;
    } else {
	return -EINVAL;
    }

    write_reg(pThis, ADDR_CTRL_REG1, bits);

    return OK;
}

static void set_driver_lowpass_filter(SPI_INSTANCE_DATA *pThis, float samplerate, float bandwidth)
{
    l3gd20Struct_t *pL3GD20Data = (l3gd20Struct_t *)pThis->pData;

    LowPassFilter2p_set_cutoff_frequency(&(pL3GD20Data->_gyro_filter_x), samplerate, bandwidth);
    LowPassFilter2p_set_cutoff_frequency(&(pL3GD20Data->_gyro_filter_y), samplerate, bandwidth);
    LowPassFilter2p_set_cutoff_frequency(&(pL3GD20Data->_gyro_filter_z), samplerate, bandwidth);
}

static void disable_i2c(SPI_INSTANCE_DATA *pThis)
{
    uint8_t retries = 10;
    while (retries--) {
	// add retries
	uint8_t a = read_reg(pThis, 0x05);
	write_reg(pThis, 0x05, (0x20 | a));
	if (read_reg(pThis, 0x05) == (a | 0x20)) {
	    return;
	}
    }
    //debug("FAILED TO DISABLE I2C");
}

static void reset( SPI_INSTANCE_DATA *pThis )
{
    l3gd20Struct_t *pL3GD20Data = (l3gd20Struct_t *)pThis->pData;

    // ensure the chip doesn't interpret any other bus traffic as I2C
    disable_i2c( pThis );

    /* set default configuration */
    write_reg(pThis, ADDR_CTRL_REG1, REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE);
    write_reg(pThis, ADDR_CTRL_REG2, 0);	    /* disable high-pass filters */
    write_reg(pThis, ADDR_CTRL_REG3, 0x08);        /* DRDY enable */
    write_reg(pThis, ADDR_CTRL_REG4, REG4_BDU);
    write_reg(pThis, ADDR_CTRL_REG5, 0);

    write_reg(pThis, ADDR_CTRL_REG5, REG5_FIFO_ENABLE);		/* disable wake-on-interrupt */

    /* disable FIFO. This makes things simpler and ensures we
     * aren't getting stale data. It means we must run the hrt
     * callback fast enough to not miss data. */
    write_reg(pThis, ADDR_FIFO_CTRL_REG, FIFO_CTRL_BYPASS_MODE);

    set_samplerate(pThis, 0); // 760Hz
    set_range(pThis, L3GD20_DEFAULT_RANGE_DPS);
    set_driver_lowpass_filter(pThis, L3GD20_DEFAULT_RATE, L3GD20_DEFAULT_FILTER_FREQ);

    pL3GD20Data->_read = 0;
}


#ifdef GPIO_EXTI_GYRO_DRDY
# define L3GD20_USE_DRDY 1
#else
# define L3GD20_USE_DRDY 0
#endif
static void measure( SPI_INSTANCE_DATA *pThis )
{
    uint8_t    slot;
    l3gd20Struct_t *pL3GD20Data = (l3gd20Struct_t *)pThis->pData;
    slot = pL3GD20Data->slot;

#if L3GD20_USE_DRDY
    // if the gyro doesn't have any data ready then re-schedule
    // for 100 microseconds later. This ensures we don't double
    // read a value and then miss the next value
    if (stm32_gpioread(GPIO_EXTI_GYRO_DRDY) == 0) {
	yield(100);
	return;
    }
#endif

    /* status register and data as read back from the device */
#pragma pack(push, 1)
    struct {
	    uint8_t		cmd;
	    uint8_t		temp;
	    uint8_t		status;
	    int16_t		x;
	    int16_t		y;
	    int16_t		z;
    } raw_report;
#pragma pack(pop)

    //gyro_report report;

    /* fetch data from the sensor */
    memset(&raw_report, 0, sizeof(raw_report));
    raw_report.cmd = ADDR_OUT_TEMP | DIR_READ | ADDR_INCREMENT;
    //transfer((uint8_t *)&raw_report, (uint8_t *)&raw_report, sizeof(raw_report));
    SpiTransfer(pThis, (uint8_t *)&raw_report, (uint8_t *)&raw_report, sizeof(raw_report));

#if L3GD20_USE_DRDY
    if ((raw_report.status & 0xF) != 0xF) {
	/*
	  we waited for DRDY, but did not see DRDY on all axes
	  when we captured. That means a transfer error of some sort
	 */
	return;
    }
#endif
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
    pL3GD20Data->_gyro_report[slot].error_count = 0; // not recorded
    
    switch (pL3GD20Data->_orientation) {

    case SENSOR_BOARD_ROTATION_000_DEG:
	/* keep axes in place */
	pL3GD20Data->_gyro_report[slot].x_raw = raw_report.x;
	pL3GD20Data->_gyro_report[slot].y_raw = raw_report.y;
	break;

    case SENSOR_BOARD_ROTATION_090_DEG:
	/* swap x and y */
	pL3GD20Data->_gyro_report[slot].x_raw = raw_report.y;
	pL3GD20Data->_gyro_report[slot].y_raw = raw_report.x;
	break;

    case SENSOR_BOARD_ROTATION_180_DEG:
	/* swap x and y and negate both */
	pL3GD20Data->_gyro_report[slot].x_raw = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
	pL3GD20Data->_gyro_report[slot].y_raw = ((raw_report.y == -32768) ? 32767 : -raw_report.y);
	break;

    case SENSOR_BOARD_ROTATION_270_DEG:
	/* swap x and y and negate y */
	pL3GD20Data->_gyro_report[slot].x_raw = raw_report.y;
	pL3GD20Data->_gyro_report[slot].y_raw = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
	break;
    }

    pL3GD20Data->_gyro_report[slot].z_raw = raw_report.z;

    pL3GD20Data->_gyro_report[slot].x = ((pL3GD20Data->_gyro_report[slot].x_raw * pL3GD20Data->_gyro_range_scale) - pL3GD20Data->_gyro_scale.x_offset) * pL3GD20Data->_gyro_scale.x_scale;
    pL3GD20Data->_gyro_report[slot].y = ((pL3GD20Data->_gyro_report[slot].y_raw * pL3GD20Data->_gyro_range_scale) - pL3GD20Data->_gyro_scale.y_offset) * pL3GD20Data->_gyro_scale.y_scale;
    pL3GD20Data->_gyro_report[slot].z = ((pL3GD20Data->_gyro_report[slot].z_raw * pL3GD20Data->_gyro_range_scale) - pL3GD20Data->_gyro_scale.z_offset) * pL3GD20Data->_gyro_scale.z_scale;

    pL3GD20Data->_gyro_report[slot].x = LowPassFilter2p_apply(&(pL3GD20Data->_gyro_filter_x), pL3GD20Data->_gyro_report[slot].x);
    pL3GD20Data->_gyro_report[slot].y = LowPassFilter2p_apply(&(pL3GD20Data->_gyro_filter_y), pL3GD20Data->_gyro_report[slot].y);
    pL3GD20Data->_gyro_report[slot].z = LowPassFilter2p_apply(&(pL3GD20Data->_gyro_filter_z), pL3GD20Data->_gyro_report[slot].z);

    pL3GD20Data->_gyro_report[slot].scaling = pL3GD20Data->_gyro_range_scale;
    pL3GD20Data->_gyro_report[slot].range_rad_s = pL3GD20Data->_gyro_range_rad_s;

    // Ben+
    pL3GD20Data->_gyro_report[slot].temperature_raw = raw_report.temp;


    pL3GD20Data->_read++;    
    pL3GD20Data->slot = (pL3GD20Data->slot + 1) % L3GD20_SLOTS;
    pL3GD20Data->_gyro_report[slot].timestamp = timerMicros();
}


static void l3gd20TaskCode(void *pInst )
{
    SPI_INSTANCE_DATA *pThis = (SPI_INSTANCE_DATA *)pInst;
    l3gd20Struct_t *pL3GD20Data = (l3gd20Struct_t *)pThis->pData;

    yield(1000);
    while(1)
    {
	measure( pThis );

	// Every interval
	yield( pL3GD20Data->_call_interval );
    }
}



uint8_t l3gd20Init(void)
{
    spiL3GD20.pData = &l3gd20Data;
    spiL3GD20._bus = 1;
    spiL3GD20._device = PX4_SPIDEV_GYRO;
    spiL3GD20._mode = SPIDEV_MODE3;
    spiL3GD20._frequency = 8000000;
    spiL3GD20._dev = up_spiinitialize(spiL3GD20._bus);
    if( spiL3GD20._dev == NULL )
	return 0;
    /* deselect device to ensure high to low transition of pin select */
    SPI_SELECT(spiL3GD20._dev, spiL3GD20._device, false);

    // Init
    l3gd20Data.enabled = true;
    l3gd20Data._gyro_range_scale = 0.0f;
    l3gd20Data._gyro_range_rad_s = 0.0f;
    l3gd20Data._current_rate = 0;
    l3gd20Data._orientation = SENSOR_BOARD_ROTATION_270_DEG;
    l3gd20Data._read = 0;
    /* convert hz to hrt interval via microseconds */
    l3gd20Data._call_interval = 1000000 / L3GD20_DEFAULT_RATE;

    /* adjust filters */
    /*
    float cutoff_freq_hz = _gyro_filter_x.get_cutoff_freq();
    float sample_rate = 1.0e6f/ticks;
    set_driver_lowpass_filter(sample_rate, cutoff_freq_hz);
    */

    // default scale factors
    l3gd20Data._gyro_scale.x_offset = 0;
    l3gd20Data._gyro_scale.x_scale  = 1.0f;
    l3gd20Data._gyro_scale.y_offset = 0;
    l3gd20Data._gyro_scale.y_scale  = 1.0f;
    l3gd20Data._gyro_scale.z_offset = 0;
    l3gd20Data._gyro_scale.z_scale  = 1.0f;

    l3gd20Data.slot = 0;
    memset( &l3gd20Data._gyro_report[0], 0, sizeof(l3gd20Data._gyro_report) );

    if( !probe( &spiL3GD20 ) )
	return 0;

    reset( &spiL3GD20 );
    measure( &spiL3GD20 );

    l3gd20TaskStack = aqStackInit(L3GD20_STACK_SIZE, "DIMU_L3GD20");
    l3gd20Data.task = CoCreateTask(l3gd20TaskCode, (void *)&spiL3GD20, L3GD20_PRIORITY, &l3gd20TaskStack[L3GD20_STACK_SIZE-1], L3GD20_STACK_SIZE);

    return 1;
}

/******************************************************************************/
/* Do something just like mpu6000.c                                           */
/******************************************************************************/


static void l3gd20ScaleGyo(int32_t *in, float *out, float divisor) {
    float scale;

/*
    scale = 1.0f / ((1<<16) / (MPU6000_GYO_SCALE * 2.0f));

    out[0] = DIMU_ORIENT_GYO_X * divisor * scale * DEG_TO_RAD;
    out[1] = DIMU_ORIENT_GYO_Y * divisor * scale * DEG_TO_RAD;
    out[2] = DIMU_ORIENT_GYO_Z * divisor * scale * DEG_TO_RAD;
*/

    scale = 1.0f / ((1<<16) / (L3GD20_GYO_SCALE * 2.0f));

    // Ben: Already RAD
    out[0] = DIMU_ORIENT_GYO_X * divisor * scale; // * DEG_TO_RAD;
    out[1] = DIMU_ORIENT_GYO_Y * divisor * scale; // * DEG_TO_RAD;
    out[2] = DIMU_ORIENT_GYO_Z * divisor * scale; // * DEG_TO_RAD;

}


static void l3gd20CalibGyo(float *in, volatile float *out) {
    float a, b, c;
    float x, y, z;

    // bias
    a = +(in[0] + l3gd20Data.gyoOffset[0] + p[IMU_GYO_BIAS_X] + p[IMU_GYO_BIAS1_X]*dImuData.dTemp + p[IMU_GYO_BIAS2_X]*dImuData.dTemp2 + p[IMU_GYO_BIAS3_X]*dImuData.dTemp3);
    b = -(in[1] + l3gd20Data.gyoOffset[1] + p[IMU_GYO_BIAS_Y] + p[IMU_GYO_BIAS1_Y]*dImuData.dTemp + p[IMU_GYO_BIAS2_Y]*dImuData.dTemp2 + p[IMU_GYO_BIAS3_Y]*dImuData.dTemp3);
    c = -(in[2] + l3gd20Data.gyoOffset[2] + p[IMU_GYO_BIAS_Z] + p[IMU_GYO_BIAS1_Z]*dImuData.dTemp + p[IMU_GYO_BIAS2_Z]*dImuData.dTemp2 + p[IMU_GYO_BIAS3_Z]*dImuData.dTemp3);

    // misalignment
    x = a + b*p[IMU_GYO_ALGN_XY] + c*p[IMU_GYO_ALGN_XZ];
    y = a*p[IMU_GYO_ALGN_YX] + b + c*p[IMU_GYO_ALGN_YZ];
    z = a*p[IMU_GYO_ALGN_ZX] + b*p[IMU_GYO_ALGN_ZY] + c;

    // scale
    x /= p[IMU_GYO_SCAL_X];
    y /= p[IMU_GYO_SCAL_Y];
    z /= p[IMU_GYO_SCAL_Z];

    // IMU rotation
    out[0] = x * imuData.cosRot - y * imuData.sinRot;
    out[1] = y * imuData.cosRot + x * imuData.sinRot;
    out[2] = z;
}


void l3gd20InitialBias(void) {
    uint32_t lastUpdate = l3gd20Data.lastUpdate;
    float tempSum;
    int i;

    tempSum = 0.0f;

    for (i = 0; i < 50; i++) {
	while (lastUpdate == l3gd20Data.lastUpdate)
	    delay(1);
	lastUpdate = l3gd20Data.lastUpdate;

	tempSum += l3gd20Data.rawTemp;
    }

    l3gd20Data.temp = tempSum / 50.0f;
    utilFilterReset(&l3gd20Data.tempFilter, l3gd20Data.temp);
}


void l3gd20DrateDecode(void)
{
    int32_t gyo[3];
    float divisor;
    int s, i;

    for (i = 0; i < 3; i++)
	gyo[i] = 0;

    divisor = (float)L3GD20_DRATE_SLOTS;
    s = l3gd20Data.slot - 1;
    if (s < 0)
	s = L3GD20_SLOTS - 1;

    for (i = 0; i < L3GD20_DRATE_SLOTS; i++) {
	gyo[0] += l3gd20Data._gyro_report[s].x;
        gyo[1] += l3gd20Data._gyro_report[s].y;
        gyo[2] += l3gd20Data._gyro_report[s].z;

	if (--s < 0)
	    s = L3GD20_SLOTS - 1;
    }

    divisor = 1.0f / divisor;

    l3gd20ScaleGyo(gyo, l3gd20Data.dRateRawGyo, divisor);
    l3gd20CalibGyo(l3gd20Data.dRateRawGyo, l3gd20Data.dRateGyo);
}

void l3gd20Decode(void)
{
    int32_t  temp, gyo[3];
    float divisor;
    int i;

    for (i = 0; i < 3; i++) {
	gyo[i] = 0;
    }
    temp = 0;

    divisor = (float)L3GD20_SLOTS;
    for (i = 0; i < L3GD20_SLOTS; i++) {
	gyo[0] += l3gd20Data._gyro_report[i].x;
        gyo[1] += l3gd20Data._gyro_report[i].y;
        gyo[2] += l3gd20Data._gyro_report[i].z;

	temp += l3gd20Data._gyro_report[i].temperature_raw;
    }

    divisor = 1.0f / divisor;

    l3gd20Data.rawTemp = temp * divisor * (1.0f / 340.0f) + 36.53f;
    l3gd20Data.temp = utilFilter(&l3gd20Data.tempFilter, l3gd20Data.rawTemp);

    l3gd20ScaleGyo(gyo, l3gd20Data.rawGyo, divisor);
    l3gd20CalibGyo(l3gd20Data.rawGyo, l3gd20Data.gyo);

    l3gd20Data.lastUpdate = timerMicros();
}
