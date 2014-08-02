/*
    This file is part of AutoQuad For PX4FMU(Pixhawk)

    

    Copyright (C) 2014  BenZeng
*/
#include "CoOS.h"
#include "px4fmu_config.h"
#include "px4fmu_board.h"
#include "px4fmu_types.h"
#include "px4fmu_i2c.h"
#include "px4fmu_rgbled.h"

#include <string.h>
#include <stdio.h>


#define RGBLED_STACK_SIZE   105
#define RGBLED_PRIORITY	    34

static OS_STK *rgbledTaskStack;

#define PX4_I2C_OBDEV_LED_ADDR  0x55	/**< I2C adress of TCA62724FMG */
#define SUB_ADDR_START		0x01	/**< write everything (with auto-increment) */
#define SUB_ADDR_PWM0		0x81	/**< blue     (without auto-increment) */
#define SUB_ADDR_PWM1		0x82	/**< green    (without auto-increment) */
#define SUB_ADDR_PWM2		0x83	/**< red      (without auto-increment) */
#define SUB_ADDR_SETTINGS	0x84	/**< settings (without auto-increment)*/

#define SETTING_NOT_POWERSAVE	0x01	/**< power-save mode not off */
#define SETTING_ENABLE   	0x02	/**< on */


/* Maximum Timeout values for flags and events waiting loops. These timeouts are
   not based on accurate values, they just guarantee that the application will 
   not remain stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define RGBLED_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define RGBLED_LONG_TIMEOUT         ((uint32_t)(30 * RGBLED_FLAG_TIMEOUT))

typedef struct {
    rgbled_mode_t	_mode;
    rgbled_pattern_t	_pattern;

    uint8_t	_r;
    uint8_t	_g;
    uint8_t	_b;
    float	_brightness;

    bool	_running;
    int		_led_interval;
    bool	_should_run;
    int		_counter;

    /**
    * The number of times a read or write operation will be retried on
    * error.
    */
    unsigned		_retries;

    /**
    * The I2C bus number the device is attached to.
    */
    int			_bus;

    uint16_t		_address;
    uint32_t		_frequency;
    struct i2c_dev_s	*_dev;

}RgbLedStruct_t;

// Only one instance
static RgbLedStruct_t rgbledData;


int I2C_transfer1(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
    struct i2c_msg_s msgv[2];
    unsigned msgs;
    int ret;
    unsigned retry_count = 0;

    do {
	//	debug("transfer out %p/%u  in %p/%u", send, send_len, recv, recv_len);

	msgs = 0;

	if (send_len > 0) {
	    msgv[msgs].addr = rgbledData._address;
	    msgv[msgs].flags = 0;
	    msgv[msgs].buffer = (uint8_t *)(send);
	    msgv[msgs].length = send_len;
	    msgs++;
	}

	if (recv_len > 0) {
	    msgv[msgs].addr = rgbledData._address;
	    msgv[msgs].flags = I2C_M_READ;
	    msgv[msgs].buffer = recv;
	    msgv[msgs].length = recv_len;
	    msgs++;
	}

	if (msgs == 0)
	    return -EINVAL;

	/*
	 * I2C architecture means there is an unavoidable race here
	 * if there are any devices on the bus with a different frequency
	 * preference.  Really, this is pointless.
	 */
	I2C_SETFREQUENCY(rgbledData._dev, rgbledData._frequency);
	ret = I2C_TRANSFER(rgbledData._dev, &msgv[0], msgs);

	/* success */
	if (ret == OK)
	    break;

	/* if we have already retried once, or we are going to give up, then reset the bus */
	if ((retry_count >= 1) || (retry_count >= rgbledData._retries))
	    up_i2creset(rgbledData._dev);

    } while (retry_count++ < rgbledData._retries);

    return ret;

}

int I2C_transfer2(struct i2c_msg_s *msgv, unsigned msgs)
{
    int ret;
    unsigned retry_count = 0;

    /* force the device address into the message vector */
    for (unsigned i = 0; i < msgs; i++)
	msgv[i].addr = rgbledData._address;


    do {
	/*
	 * I2C architecture means there is an unavoidable race here
	 * if there are any devices on the bus with a different frequency
	 * preference.  Really, this is pointless.
	 */
	I2C_SETFREQUENCY(rgbledData._dev, rgbledData._frequency);
	ret = I2C_TRANSFER(rgbledData._dev, msgv, msgs);

	/* success */
	if (ret == OK)
	    break;

	/* if we have already retried once, or we are going to give up, then reset the bus */
	if ((retry_count >= 1) || (retry_count >= rgbledData._retries))
	    up_i2creset(rgbledData._dev);

    } while (retry_count++ < rgbledData._retries);

    return ret;
}

int
RGBLED_get(bool *on, bool *powersave, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t result[2];
    int ret;

    ret = I2C_transfer1(NULL, 0, &result[0], 2);

    if (ret == OK) {
	*on = result[0] & SETTING_ENABLE;
	*powersave = !(result[0] & SETTING_NOT_POWERSAVE);
	/* XXX check, looks wrong */
	*r = (result[0] & 0x0f) << 4;
	*g = (result[1] & 0xf0);
	*b = (result[1] & 0x0f) << 4;
    }

    return ret;
}

/**
 * Sent ENABLE flag to LED driver
 */
int
RGBLED_send_led_enable(bool enable)
{
    uint8_t settings_byte = 0;

    if (enable)
	settings_byte |= SETTING_ENABLE;

    settings_byte |= SETTING_NOT_POWERSAVE;

    const uint8_t msg[2] = { SUB_ADDR_SETTINGS, settings_byte};

    return I2C_transfer1(msg, sizeof(msg), NULL, 0);
}

/**
 * Send RGB PWM settings to LED driver according to current color and brightness
 */
int
RGBLED_send_led_rgb()
{
    /* To scale from 0..255 -> 0..15 shift right by 4 bits */
    const uint8_t msg[6] = {
	SUB_ADDR_PWM0, (uint8_t)((int)(rgbledData._b * rgbledData._brightness) >> 4),
	SUB_ADDR_PWM1, (uint8_t)((int)(rgbledData._g * rgbledData._brightness) >> 4),
	SUB_ADDR_PWM2, (uint8_t)((int)(rgbledData._r * rgbledData._brightness) >> 4)
    };
    return I2C_transfer1(msg, sizeof(msg), NULL, 0);
}


int RGBLED_probe()
{
    int ret;
    bool on, powersave;
    uint8_t r, g, b;

    /**
       this may look strange, but is needed. There is a serial
       EEPROM (Microchip-24aa01) on the PX4FMU-v1 that responds to
       a bunch of I2C addresses, including the 0x55 used by this
       LED device. So we need to do enough operations to be sure
       we are talking to the right device. These 3 operations seem
       to be enough, as the 3rd one consistently fails if no
       RGBLED is on the bus.
     */
    if ((ret=RGBLED_get(&on, &powersave, &r, &g, &b)) != OK ||
	(ret=RGBLED_send_led_enable(false) != OK) ||
	(ret=RGBLED_send_led_enable(false) != OK)) {
	    return false;
    }

    return true;
}




/**
 * Parse color constant and set _r _g _b values
 */
void RGBLED_set_color(rgbled_color_t color)
{
    switch (color) {
    case RGBLED_COLOR_OFF:
	rgbledData._r = 0;
	rgbledData._g = 0;
	rgbledData._b = 0;
	break;

    case RGBLED_COLOR_RED:
	rgbledData._r = 255;
	rgbledData._g = 0;
	rgbledData._b = 0;
	break;

    case RGBLED_COLOR_YELLOW:
	rgbledData._r = 255;
	rgbledData._g = 200;
	rgbledData._b = 0;
	break;

    case RGBLED_COLOR_PURPLE:
	rgbledData._r = 255;
	rgbledData._g = 0;
	rgbledData._b = 255;
	break;

    case RGBLED_COLOR_GREEN:
	rgbledData._r = 0;
	rgbledData._g = 255;
	rgbledData._b = 0;
	break;

    case RGBLED_COLOR_BLUE:
	rgbledData._r = 0;
	rgbledData._g = 0;
	rgbledData._b = 255;
	break;

    case RGBLED_COLOR_WHITE:
	rgbledData._r = 255;
	rgbledData._g = 255;
	rgbledData._b = 255;
	break;

    case RGBLED_COLOR_AMBER:
	rgbledData._r = 255;
	rgbledData._g = 80;
	rgbledData._b = 0;
	break;

    case RGBLED_COLOR_DIM_RED:
	rgbledData._r = 90;
	rgbledData._g = 0;
	rgbledData._b = 0;
	break;

    case RGBLED_COLOR_DIM_YELLOW:
	rgbledData._r = 80;
	rgbledData._g = 30;
	rgbledData._b = 0;
	break;

    case RGBLED_COLOR_DIM_PURPLE:
	rgbledData._r = 45;
	rgbledData._g = 0;
	rgbledData._b = 45;
	break;

    case RGBLED_COLOR_DIM_GREEN:
	rgbledData._r = 0;
	rgbledData._g = 90;
	rgbledData._b = 0;
	break;

    case RGBLED_COLOR_DIM_BLUE:
	rgbledData._r = 0;
	rgbledData._g = 0;
	rgbledData._b = 90;
	break;

    case RGBLED_COLOR_DIM_WHITE:
	rgbledData._r = 30;
	rgbledData._g = 30;
	rgbledData._b = 30;
	break;

    case RGBLED_COLOR_DIM_AMBER:
	rgbledData._r = 80;
	rgbledData._g = 20;
	rgbledData._b = 0;
	break;

    default:
	//warnx("color unknown");
	break;
    }
}


/**
 * Set mode, if mode not changed has no any effect (doesn't reset blinks phase)
 */
void
RGBLED_set_mode(rgbled_mode_t mode)
{
    if (mode != rgbledData._mode) {
	rgbledData._mode = mode;

	switch (mode) {
	case RGBLED_MODE_OFF:
	    rgbledData._should_run = false;
	    RGBLED_send_led_enable(false);
	    break;

	case RGBLED_MODE_ON:
	    rgbledData._brightness = 1.0f;
	    RGBLED_send_led_rgb();
	    RGBLED_send_led_enable(true);
	    break;

	case RGBLED_MODE_BLINK_SLOW:
	    rgbledData._should_run = true;
	    rgbledData._counter = 0;
	    rgbledData._led_interval = 2000;
	    rgbledData._brightness = 1.0f;
	    RGBLED_send_led_rgb();
	    break;

	case RGBLED_MODE_BLINK_NORMAL:
	    rgbledData._should_run = true;
	    rgbledData._counter = 0;
	    rgbledData._led_interval = 500;
	    rgbledData._brightness = 1.0f;
	    RGBLED_send_led_rgb();
	    break;

	case RGBLED_MODE_BLINK_FAST:
	    rgbledData._should_run = true;
	    rgbledData._counter = 0;
	    rgbledData._led_interval = 100;
	    rgbledData._brightness = 1.0f;
	    RGBLED_send_led_rgb();
	    break;

	case RGBLED_MODE_BREATHE:
	    rgbledData._should_run = true;
	    rgbledData._counter = 0;
	    rgbledData._led_interval = 25;
	    RGBLED_send_led_enable(true);
	    break;

	case RGBLED_MODE_PATTERN:
	    rgbledData._should_run = true;
	    rgbledData._counter = 0;
	    rgbledData._brightness = 1.0f;
	    RGBLED_send_led_enable(true);
	    break;

	default:
	    //warnx("mode unknown");
	    break;
	}

	/* if it should run now, start the workq */
	if (rgbledData._should_run && !rgbledData._running) {
	    rgbledData._running = true;
	    //work_queue(LPWORK, &_work, (worker_t)&RGBLED::led_trampoline, this, 1);
	}
    }
}

/**
 * Set pattern for PATTERN mode, but don't change current mode
 */
void
RGBLED_set_pattern(rgbled_pattern_t *pattern)
{
    memcpy(&rgbledData._pattern, pattern, sizeof(rgbled_pattern_t));
}



/**
 * Main loop function
 */
void rgbledTaskCode()
{
    while(1)
    {
	if(!rgbledData._should_run) {
	    rgbledData._running = false;
	    CoTickDelay( 100 );
	    continue;
	}

	switch (rgbledData._mode) {
	case RGBLED_MODE_BLINK_SLOW:
	case RGBLED_MODE_BLINK_NORMAL:
	case RGBLED_MODE_BLINK_FAST:
	    if (rgbledData._counter >= 2)
		rgbledData._counter = 0;

	    RGBLED_send_led_enable(rgbledData._counter == 0);

	    break;

	case RGBLED_MODE_BREATHE:

	    if (rgbledData._counter >= 62)
		rgbledData._counter = 0;

	    int n;

	    if (rgbledData._counter < 32) {
		n = rgbledData._counter;
	    } else {
		n = 62 - rgbledData._counter;
	    }

	    rgbledData._brightness = n * n / (31.0f * 31.0f);
	    RGBLED_send_led_rgb();
	    break;

	case RGBLED_MODE_PATTERN:

	    /* don't run out of the pattern array and stop if the next frame is 0 */
	    if (rgbledData._counter >= RGBLED_PATTERN_LENGTH || rgbledData._pattern.duration[rgbledData._counter] <= 0)
		rgbledData._counter = 0;

	    RGBLED_set_color(rgbledData._pattern.color[rgbledData._counter]);
	    RGBLED_send_led_rgb();
	    rgbledData._led_interval = rgbledData._pattern.duration[rgbledData._counter];
	    break;

	default:
	    break;
	}

	rgbledData._counter++;

	CoTickDelay( rgbledData._led_interval );
    }
}


bool  rgbledInit( void )
{
    rgbledData._should_run = true;
    rgbledData._bus = PX4_I2C_BUS_LED;
    rgbledData._frequency = 100000;
    rgbledData._address = PX4_I2C_OBDEV_LED_ADDR;
    rgbledData._retries = 0;
    rgbledData._dev = up_i2cinitialize(rgbledData._bus);

   
    // call the probe function to check whether the device is present
    bool ret = RGBLED_probe();
    if (!ret) {
	//debug("probe failed");
	//goto out;
	return false;
    }

    // Start LED Driver task
    rgbledTaskStack = aqStackInit(RGBLED_STACK_SIZE, "RGBLED");
    CoCreateTask(rgbledTaskCode, (void *)0, RGBLED_PRIORITY, &rgbledTaskStack[RGBLED_STACK_SIZE-1], RGBLED_STACK_SIZE);

    //rbgledSetColor(RGBLED_COLOR_RED);
    //rgbledSetMode(RGBLED_MODE_BLINK_NORMAL);

    rgbled_pattern_t ledPattern = 
    {
	{RGBLED_COLOR_RED, RGBLED_COLOR_YELLOW, RGBLED_COLOR_PURPLE, RGBLED_COLOR_GREEN, RGBLED_COLOR_BLUE, RGBLED_COLOR_DIM_AMBER, 0},
	{50,              50,                 50,                 50,                50,               50,                    0}
    };
    RGBLED_set_pattern( &ledPattern );
    RGBLED_set_mode(RGBLED_MODE_PATTERN);

    return true;
}
