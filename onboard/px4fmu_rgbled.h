/*
    This file is part of AutoQuad For PX4FMU(Pixhawk)

    

    Copyright (C) 2014  BenZeng
*/
#ifndef __PX4FMU_RGBLED_H
#define __PX4FMU_RGBLED_H



#ifndef true
#define true 1
#endif
#ifndef false
#define false 0
#endif
#ifndef bool
typedef unsigned char bool;
#endif




typedef enum {
	RGBLED_COLOR_OFF,
	RGBLED_COLOR_RED,
	RGBLED_COLOR_YELLOW,
	RGBLED_COLOR_PURPLE,
	RGBLED_COLOR_GREEN,
	RGBLED_COLOR_BLUE,
	RGBLED_COLOR_WHITE,
	RGBLED_COLOR_AMBER,
	RGBLED_COLOR_DIM_RED,
	RGBLED_COLOR_DIM_YELLOW,
	RGBLED_COLOR_DIM_PURPLE,
	RGBLED_COLOR_DIM_GREEN,
	RGBLED_COLOR_DIM_BLUE,
	RGBLED_COLOR_DIM_WHITE,
	RGBLED_COLOR_DIM_AMBER
} rgbled_color_t;

typedef enum {
	RGBLED_MODE_OFF,
	RGBLED_MODE_ON,
	RGBLED_MODE_BLINK_SLOW,
	RGBLED_MODE_BLINK_NORMAL,
	RGBLED_MODE_BLINK_FAST,
	RGBLED_MODE_BREATHE,
	RGBLED_MODE_PATTERN
} rgbled_mode_t;

#define RGBLED_PATTERN_LENGTH 20

typedef struct {
	rgbled_color_t color[RGBLED_PATTERN_LENGTH];
	unsigned duration[RGBLED_PATTERN_LENGTH];
} rgbled_pattern_t;



bool    rgbledInit( void );
void    RGBLED_set_color(rgbled_color_t color);
void    RGBLED_set_mode(rgbled_mode_t mode);
void    RGBLED_set_pattern(rgbled_pattern_t *pattern);











#endif

