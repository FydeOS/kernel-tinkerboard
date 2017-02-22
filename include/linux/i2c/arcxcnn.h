/*
 * Backlight driver for ArcticSand ARC2C0608 Backlight Devices
 *
 * Copyright 2016 ArcticSand, Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _ARCXCNN_H
#define _ARCXCNN_H

enum arcxcnn_chip_id {
	ARC2C0608
};

enum arcxcnn_brightness_source {
	ARCXCNN_PWM_ONLY,
	ARCXCNN_I2C_ONLY = 2,
};

#define ARCXCNN_MAX_PROGENTRIES	48	/* max a/v pairs for custom */

/**
 * struct arcxcnn_platform_data
 * @name : Backlight driver name. If it is not defined, default name is set.
 * @initial_brightness : initial value of backlight brightness
 * @led_str	 : initial LED string enables, upper bit is global on/off
 * @led_config_0 : fading speed (period between intensity steps)
 * @led_config_1 : misc settings, see datasheet
 * @dim_freq	 : pwm dimming frequency if in pwm mode
 * @comp_config	 : misc config, see datasheet
 * @filter_config: RC/PWM filter config, see datasheet
 * @trim_config	 : full scale current trim, see datasheet
 * @led_config_0_set	: the value in led_config_0 is valid
 * @led_config_1_set	: the value in led_config_1 is valid
 * @dim_freq_set	: the value in dim_freq is valid
 * @comp_config_set	: the value in comp_config is valid
 * @filter_config_set	: the value in filter_config is valid
 * @trim_config_set	: the value in trim_config is valid
 *
 * the _set flags are used to indicate that the value was explicitly set
 * in the device tree or platform data. settings not set are left as default
 * power-on default values of the chip except for led_str and led_config_1
 * which are set by the driver (led_str is specified indirectly in the
 * device tree via "led-sources")
 */
struct arcxcnn_platform_data {
	const char *name;
	u16 initial_brightness;
	u8	led_str;

	u8	led_config_0;
	u8	led_config_1;
	u8	dim_freq;
	u8	comp_config;
	u8	filter_config;
	u8	trim_config;

	bool	led_config_0_set;
	bool	led_config_1_set;
	bool	dim_freq_set;
	bool	comp_config_set;
	bool	filter_config_set;
	bool	trim_config_set;
};

#endif
