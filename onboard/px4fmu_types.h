/*
    This file is part of AutoQuad For PX4FMU(Pixhawk)

    Ported from nuttx\arch\arm\src\stm32\stm32_spi.h

    Copyright (C) 2014  BenZeng
*/
#ifndef __PX4FMU_STM32F247_TYPES_H
#define __PX4FMU_STM32F247_TYPES_H

/* A pointer is 4 bytes */

typedef signed int         _intptr_t;
typedef unsigned int       _uintptr_t;

typedef signed char        _int8_t;
typedef unsigned char      _uint8_t;

typedef signed short       _int16_t;
typedef unsigned short     _uint16_t;

typedef signed int         _int32_t;
typedef unsigned int       _uint32_t;

typedef signed long long   _int64_t;
typedef unsigned long long _uint64_t;

typedef _int8_t      int8_t;
typedef _uint8_t     uint8_t;

typedef _int16_t     int16_t;
typedef _uint16_t    uint16_t;

typedef _int32_t     int32_t;
typedef _uint32_t    uint32_t;

/* Integer types capable of holding object pointers */
typedef _intptr_t    intptr_t;
typedef _uintptr_t   uintptr_t;


typedef uint8_t            bool;
typedef uint32_t           size_t;


#define FAR


typedef unsigned int       irqstate_t;

#ifndef true
#define true  1
#define false 0
#endif

#ifndef NULL
#define NULL ((void*)0)
#endif

#undef  OK
#define OK                  0
#define EINVAL              22



#endif