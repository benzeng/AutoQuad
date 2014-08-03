/*
    This file is part of AutoQuad For PX4FMU(Pixhawk)

    

    Copyright (C) 2014  BenZeng
*/
#ifndef __PX4FMU_STM32F247_LSM303D_H
#define __PX4FMU_STM32F247_LSM303D_H

/**
 * accel report structure.  Reads from the device must be in multiples of this
 * structure.
 */
struct accel_report {
    uint64_t timestamp;
    uint64_t error_count;
    float x;		/**< acceleration in the NED X board axis in m/s^2 */
    float y;		/**< acceleration in the NED Y board axis in m/s^2 */
    float z;		/**< acceleration in the NED Z board axis in m/s^2 */
    float temperature;	/**< temperature in degrees celsius */
    float range_m_s2;	/**< range in m/s^2 (+- this value) */
    float scaling;

    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
    int16_t temperature_raw;
};

/** accel scaling factors; Vout = Vscale * (Vin + Voffset) */
struct accel_scale {
    float	x_offset;
    float	x_scale;
    float	y_offset;
    float	y_scale;
    float	z_offset;
    float	z_scale;
};

/**
 * mag report structure.  Reads from the device must be in multiples of this
 * structure.
 *
 * Output values are in gauss.
 */
struct mag_report {
    uint64_t timestamp;
    uint64_t error_count;
    float x;
    float y;
    float z;
    float range_ga;
    float scaling;

    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
};

/** mag scaling factors; Vout = (Vin * Vscale) + Voffset */
struct mag_scale {
    float	x_offset;
    float	x_scale;
    float	y_offset;
    float	y_scale;
    float	z_offset;
    float	z_scale;
};

typedef struct {
    //volatile uint8_t rxBuf[HMC5983_SLOT_SIZE*HMC5983_SLOTS];
    //volatile uint8_t slot;

    uint8_t initialized;

    // AQ MAG
    float rawMag[3];
    float mag[3];
    volatile uint32_t lastUpdateMag;
    
    // AQ ACCEL
    float rawTemp;
    float rawAcc[3];
    volatile float acc[3];
    //volatile float temp;
    volatile uint32_t lastUpdateAccel;



    unsigned		_call_accel_interval;
    unsigned		_call_mag_interval;
    
    struct accel_scale	_accel_scale;
    unsigned		_accel_range_m_s2;
    float		_accel_range_scale;
    unsigned		_accel_samplerate;
    unsigned		_accel_onchip_filter_bandwith;

    struct mag_scale	_mag_scale;
    unsigned		_mag_range_ga;
    float		_mag_range_scale;
    unsigned		_mag_samplerate;

    unsigned		_accel_read;
    unsigned		_mag_read;

    LowPassFilter2p	_accel_filter_x;
    LowPassFilter2p	_accel_filter_y;
    LowPassFilter2p	_accel_filter_z;

    // expceted values of reg1 and reg7 to catch in-flight
    // brownouts of the sensor
    uint8_t		_reg1_expected;
    uint8_t		_reg7_expected;


    struct accel_report accel_report;
    struct mag_report   mag_report;

} lsm303dStruct_t;

extern lsm303dStruct_t lsm303dData;

void lsm303dDecode(void);



#endif
