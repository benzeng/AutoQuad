/*
    This file is part of AutoQuad For PX4FMU(Pixhawk)

    

    Copyright (C) 2014  BenZeng
*/
#ifndef __PX4FMU_LOW_PASS_FILTER_H
#define __PX4FMU_LOW_PASS_FILTER_H

#define M_PI_F			3.14159265358979323846f

typedef struct 
{
    float           _cutoff_freq; 
    float           _a1;
    float           _a2;
    float           _b0;
    float           _b1;
    float           _b2;
    float           _delay_element_1;        // buffered sample -1
    float           _delay_element_2;        // buffered sample -2
}LowPassFilter2p;

void LowPassFilter2p_set_cutoff_frequency(LowPassFilter2p *pThis, float sample_freq, float cutoff_freq);
float LowPassFilter2p_apply(LowPassFilter2p *pThis, float sample);


#endif