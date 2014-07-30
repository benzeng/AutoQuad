/*
    This file is part of AutoQuad For PX4FMU(Pixhawk)

    

    Copyright (C) 2014  BenZeng
*/
#include "px4fmu_types.h"
#include "LowPassFilter.h"
#include <math.h>


void LowPassFilter2p_set_cutoff_frequency(LowPassFilter2p *pThis, float sample_freq, float cutoff_freq)
{
    pThis->_cutoff_freq = cutoff_freq;
    if (pThis->_cutoff_freq <= 0.0f) {
        // no filtering
        return;
    }
    float fr = sample_freq/pThis->_cutoff_freq;
    float ohm = tanf(M_PI_F/fr);
    float c = 1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
    pThis->_b0 = ohm*ohm/c;
    pThis->_b1 = 2.0f*pThis->_b0;
    pThis->_b2 = pThis->_b0;
    pThis->_a1 = 2.0f*(ohm*ohm-1.0f)/c;
    pThis->_a2 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
}

float LowPassFilter2p_apply(LowPassFilter2p *pThis, float sample)
{
    if (pThis->_cutoff_freq <= 0.0f) {
        // no filtering
        return sample;
    }
    // do the filtering
    float delay_element_0 = sample - pThis->_delay_element_1 * pThis->_a1 - pThis->_delay_element_2 * pThis->_a2;
    if (isnan(delay_element_0) || isinf(delay_element_0)) {
        // don't allow bad values to propogate via the filter
        delay_element_0 = sample;
    }
    float output = delay_element_0 * pThis->_b0 + pThis->_delay_element_1 * pThis->_b1 + pThis->_delay_element_2 * pThis->_b2;
    
    pThis->_delay_element_2 = pThis->_delay_element_1;
    pThis->_delay_element_1 = delay_element_0;

    // return the value.  Should be no need to check limits
    return output;
}