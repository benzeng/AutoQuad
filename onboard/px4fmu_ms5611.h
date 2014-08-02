/*
    This file is part of AutoQuad For PX4FMU(Pixhawk)

    

    Copyright (C) 2014  BenZeng
*/
#ifndef __PX4FMU_STM32F247_MS5611_H
#define __PX4FMU_STM32F247_MS5611_H

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

union _ms5611Result {
    uint8_t	b[4];
    uint32_t    w;
};
#pragma pack(pop)

/**
 * baro report structure.  Reads from the device must be in multiples of this
 * structure.
 */
struct baro_report {
    float pressure;
    float altitude;
    float temperature;
    uint64_t timestamp;
    uint64_t error_count;

    // raw MS5611 values for debugging
    uint32_t ms5611_D1;
    uint32_t ms5611_D2;
};

#undef ms5611Struct_t
#undef bool
typedef uint8_t            bool;
typedef struct {

    //
    // For AQ
    //
    uint8_t             enabled;
    uint8_t             initialized;
    utilFilter_t        tempFilter;
    volatile float      rawTemp;
    volatile float      temp;
    volatile float      pres;
    volatile uint32_t   lastUpdate;

    OS_TID task;

    //
    // For PX4
    //
    union                prom_u prom;
    struct               baro_report brp;
    unsigned            _measure_ticks;
    bool	        _collect_phase;
    unsigned	        _measure_phase;

    /* intermediate temperature values per MS5611 datasheet */
    int32_t		_TEMP;
    int64_t		_OFF;
    int64_t		_SENS;
    float		_P;
    float		_T;
    float		_Alt;
    uint32_t            _D1;
    uint32_t            _D2;
    /* altitude conversion calibration */
    unsigned		_msl_pressure;	/* in kPa */
} ms5611Struct_t;

extern ms5611Struct_t ms5611Data;

void ms5611Decode(void);
uint8_t ms5611Init(void);

#endif