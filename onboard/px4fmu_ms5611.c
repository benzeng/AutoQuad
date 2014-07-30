/*
    This file is part of AutoQuad For PX4FMU(Pixhawk)

    

    Copyright (C) 2014  BenZeng
*/

#include "px4fmu_config.h"
#include "px4fmu_types.h"
#include "util.h"
#include "px4fmu_board.h"
#include "px4fmu_spi.h"
#include "px4fmu_ms5611.h"
#include "px4fmu_rcc.h"
#include "CoOS.h"
#include "aq_timer.h"
#include <math.h>



// MS5611 Instance
static SPI_INSTANCE_DATA spiMS5611;
ms5611Struct_t ms5611Data;

#define MS5611_STACK_SIZE   250
#define MS5611_PRIORITY	    11
static OS_STK *ms5611TaskStack;

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { __typeof__(_x) _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

/* internal conversion time: 9.17 ms, so should not be read at rates higher than 100 Hz */
#define MS5611_CONVERSION_INTERVAL	10000	/* microseconds */
#define MS5611_MEASUREMENT_RATIO	3	/* pressure measurements per temperature measurement */


static void ResetMS5611( SPI_INSTANCE_DATA *pThis )
{
    uint8_t cmd = ADDR_RESET_CMD | DIR_WRITE;

    return  SpiTransfer(pThis, &cmd, NULL, 1);
}


static void _measure(SPI_INSTANCE_DATA *pThis, unsigned addr)
{
    uint8_t cmd = addr | DIR_WRITE;

    SpiTransfer(pThis, &cmd, NULL, 1);
}

static int _read(SPI_INSTANCE_DATA *pThis, void *data )
{
    union _ms5611Result *cvt = (union _ms5611Result *)data;
    uint8_t buf[4] = { 0 | DIR_WRITE, 0, 0, 0 };

    /* read the most recent measurement */
    SpiTransfer(pThis, &buf[0], &buf[0], sizeof(buf));

    /* fetch the raw value */
    cvt->b[0] = buf[3];
    cvt->b[1] = buf[2];
    cvt->b[2] = buf[1];
    cvt->b[3] = 0;

    return 4;
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

bool ReadPROM( SPI_INSTANCE_DATA *pThis )
{
    /*
     * Wait for PROM contents to be in the device (2.8 ms) in the case we are
     * called immediately after reset.
     */
    yield(1000);

    /* read and convert PROM words */
    bool all_zero = true;
    for (int i = 0; i < 8; i++) {
	    uint8_t cmd = (ADDR_PROM_SETUP + (i * 2));
	    ms5611Data.prom.c[i] = _reg16(pThis, cmd);
	    if (ms5611Data.prom.c[i] != 0)
		    all_zero = false;
	    //debug("prom[%u]=0x%x", (unsigned)i, (unsigned)_prom.c[i]);
    }

    /* calculate CRC and return success/failure accordingly */
    if( !_crc4(&ms5611Data.prom.c[0]) )
	return false;
    if( all_zero )
	return false;

    return true;
}



void ms5611PreInit(void){}
void ms5611Enable(void){}

void ms5611InitialBias(void) {
    uint32_t lastUpdate = ms5611Data.lastUpdate;
    float tempSum;
    int i;

    if (ms5611Data.initialized) {
      tempSum = 0.0f;

      for (i = 0; i < 10; i++) {
          while (lastUpdate == ms5611Data.lastUpdate)
              delay(1);
          lastUpdate = ms5611Data.lastUpdate;

          tempSum += ms5611Data.rawTemp;
      }

      ms5611Data.temp = tempSum / 10.0f;
      utilFilterReset(&ms5611Data.tempFilter, ms5611Data.temp);
    }
}


static int collect( SPI_INSTANCE_DATA *pThis )
{
    uint32_t raw;
    ms5611Struct_t *pMS5611Data = (ms5611Struct_t *)pThis->pData;

    /* this should be fairly close to the end of the conversion, so the best approximation of the time */
    //ms5611Data.brp.timestamp = timerMicros();

    /* read the most recent measurement - read offset/size are hardcoded in the interface */
    _read( pThis, (void *)&raw );

    // report the raw D1/D2 values to help diagnose problems with
    // transfers at higher temperatures
    if (pMS5611Data->_measure_phase == 0) {
	pMS5611Data->_D1 = raw;
    } else {
	pMS5611Data->_D2 = raw;
    }

    pMS5611Data->brp.ms5611_D1 = pMS5611Data->_D1;
    pMS5611Data->brp.ms5611_D2 = pMS5611Data->_D2;

    /* handle a measurement */
    if (pMS5611Data->_measure_phase == 0) {

	/* temperature offset (in ADC units) */
	int32_t dT = (int32_t)raw - ((int32_t)pMS5611Data->prom.s.c5_reference_temp << 8);

	/* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
	pMS5611Data->_TEMP = 2000 + (int32_t)(((int64_t)dT * pMS5611Data->prom.s.c6_temp_coeff_temp) >> 23);

	/* base sensor scale/offset values */
	pMS5611Data->_SENS = ((int64_t)pMS5611Data->prom.s.c1_pressure_sens << 15) + (((int64_t)pMS5611Data->prom.s.c3_temp_coeff_pres_sens * dT) >> 8);
	pMS5611Data->_OFF  = ((int64_t)pMS5611Data->prom.s.c2_pressure_offset << 16) + (((int64_t)pMS5611Data->prom.s.c4_temp_coeff_pres_offset * dT) >> 7);

	/* temperature compensation */
	if (pMS5611Data->_TEMP < 2000) {

	    int32_t T2 = POW2(dT) >> 31;

	    int64_t f = POW2((int64_t)pMS5611Data->_TEMP - 2000);
	    int64_t OFF2 = 5 * f >> 1;
	    int64_t SENS2 = 5 * f >> 2;

	    if (pMS5611Data->_TEMP < -1500) {
		int64_t f2 = POW2(pMS5611Data->_TEMP + 1500);
		OFF2 += 7 * f2;
		SENS2 += 11 * f2 >> 1;
	    }

	    pMS5611Data->_TEMP -= T2;
	    pMS5611Data->_OFF  -= OFF2;
	    pMS5611Data->_SENS -= SENS2;
	}

    } else {

	/* pressure calculation, result in Pa */
	int32_t P = (((raw * pMS5611Data->_SENS) >> 21) - pMS5611Data->_OFF) >> 15;
	pMS5611Data->_P = P * 0.01f;
	pMS5611Data->_T = pMS5611Data->_TEMP * 0.01f;

	/* generate a new report */
	pMS5611Data->brp.temperature = pMS5611Data->_TEMP / 100.0f;
	pMS5611Data->brp.pressure = P / 100.0f;		/* convert to millibar */

	/* altitude calculations based on http://www.kansasflyer.org/index.asp?nav=Avi&sec=Alti&tab=Theory&pg=1 */

	/*
	 * PERFORMANCE HINT:
	 *
	 * The single precision calculation is 50 microseconds faster than the double
	 * precision variant. It is however not obvious if double precision is required.
	 * Pending more inspection and tests, we'll leave the double precision variant active.
	 *
	 * Measurements:
	 * 	double precision: ms5611_read: 992 events, 258641us elapsed, min 202us max 305us
	 *	single precision: ms5611_read: 963 events, 208066us elapsed, min 202us max 241us
	 */

	/* tropospheric properties (0-11km) for standard atmosphere */
	const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
	const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
	const double g  = 9.80665;	/* gravity constant in m/s/s */
	const double R  = 287.05;	/* ideal gas constant in J/kg/K */

	/* current pressure at MSL in kPa */
	double p1 = pMS5611Data->_msl_pressure / 1000.0;

	/* measured pressure in kPa */
	double p = P / 1000.0;

	/*
	 * Solve:
	 *
	 *     /        -(aR / g)     \
	 *    | (p / p1)          . T1 | - T1
	 *     \                      /
	 * h = -------------------------------  + h1
	 *                   a
	 */
	pMS5611Data->brp.altitude = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;
	pMS5611Data->_Alt = pMS5611Data->brp.altitude;
        pMS5611Data->brp.timestamp = timerMicros();
    }

    /* update the measurement state machine */
    INCREMENT(pMS5611Data->_measure_phase, MS5611_MEASUREMENT_RATIO + 1);

    return OK;
}


int measure( SPI_INSTANCE_DATA *pThis )
{   
     ms5611Struct_t *pMS5611Data = (ms5611Struct_t *)pThis->pData;
    /*
     * In phase zero, request temperature; in other phases, request pressure.
     */
    unsigned addr = (pMS5611Data->_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;

    /*
     * Send the command to begin measuring.
     */
     _measure(pThis, addr);

    return OK;
}



static void ms5611TaskCode(void *pInst )
{
    int ret;

    SPI_INSTANCE_DATA *pThis = (SPI_INSTANCE_DATA *)pInst;
    ms5611Struct_t *pMS5611Data = (ms5611Struct_t *)pThis->pData;

    while(1)
    {
	/* collection phase? */
	if (pMS5611Data->_collect_phase) {

	    /* perform collection */
	    ret = collect( pThis );
	    if (ret != OK) {
		/*
		 * The ms5611 seems to regularly fail to respond to
		 * its address; this happens often enough that we'd rather not
		 * spam the console with a message for this.
		 */

		/* reset the collection state machine and try again */
                /* reset the report ring and state machine */
		pMS5611Data->_collect_phase = false;
		pMS5611Data->_measure_phase = 0;
                delay(1);
		continue;
	    }

	    /* next phase is measurement */
	    pMS5611Data->_collect_phase = false;

	    /*
	     * Is there a collect->measure gap?
	     * Don't inject one after temperature measurements, so we can keep
	     * doing pressure measurements at something close to the desired rate.
	     */
	    if ((pMS5611Data->_measure_phase != 0) &&
		(pMS5611Data->_measure_ticks > USEC2TICK(MS5611_CONVERSION_INTERVAL))) {

		/* schedule a fresh cycle call when we are ready to measure again */
		yield( pMS5611Data->_measure_ticks - USEC2TICK(MS5611_CONVERSION_INTERVAL) );
		continue;
	    }
	}

	/* measurement phase */
	ret = measure( pThis );
	if (ret != OK) {
	    //log("measure error %d", ret);
	    /* reset the collection state machine and try again */
            pMS5611Data->_collect_phase = false;
	    pMS5611Data->_measure_phase = 0;
            delay(1);
	    continue;
	}

	/* next phase is collection */
	pMS5611Data->_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
        yield( USEC2TICK(MS5611_CONVERSION_INTERVAL) );
    }
}


uint8_t ms5611Init(void)
{
    spiMS5611.pData = &ms5611Data;
    spiMS5611._bus = 1;
    spiMS5611._device = PX4_SPIDEV_BARO;
    spiMS5611._mode = SPIDEV_MODE3;
    spiMS5611._frequency = 6*1000*1000;
    spiMS5611._dev = up_spiinitialize(spiMS5611._bus);
    if( spiMS5611._dev == NULL )
	return 0;
    /* deselect device to ensure high to low transition of pin select */
    SPI_SELECT(spiMS5611._dev, spiMS5611._device, false);

    /* send reset command */
    ResetMS5611( &spiMS5611 );

    /* read PROM */
    if( ReadPROM( &spiMS5611 ) )
	ms5611Data.initialized = 1;
    else
	return 0;

    ms5611Data.enabled = true;
    ms5611Data._measure_ticks = USEC2TICK(MS5611_CONVERSION_INTERVAL);;
    ms5611Data._collect_phase = false;
    ms5611Data._measure_phase = 0;
    ms5611Data._TEMP = 0;
    ms5611Data._OFF = 0;
    ms5611Data._SENS = 0;
    ms5611Data._msl_pressure = 101325;

    // Generate first result
    measure( &spiMS5611 );
    yield(ms5611Data._measure_ticks);
    collect( &spiMS5611 );
    yield(ms5611Data._measure_ticks);
    collect( &spiMS5611 );

    ms5611TaskStack = aqStackInit(MS5611_STACK_SIZE, "DIMU_MS5611");
    ms5611Data.task = CoCreateTask(ms5611TaskCode, (void *)&spiMS5611, MS5611_PRIORITY, &ms5611TaskStack[MS5611_STACK_SIZE-1], MS5611_STACK_SIZE);

    return 1;
}


void ms5611Decode(void)
{
    ms5611Data.rawTemp = ms5611Data.brp.ms5611_D1;
    ms5611Data.temp = ms5611Data.brp.temperature;
    ms5611Data.pres = ms5611Data.brp.pressure;
    ms5611Data.lastUpdate = ms5611Data.brp.timestamp;
}