#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <Arduino.h>

/*
*****************************************************************************************
* FEATURES
*****************************************************************************************
*/
#define MOTOR_DRIVER_TB6612FNG  0
#define MOTOR_DRIVER_DRV8833    1
#define MOTOR_DRIVER_DC_ESC     2
#define MOTOR_PWM_LIMIT         180

/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/
#define DIR_STOP        0x00
#define DIR_FWD         0x10
#define DIR_REV         0x20
#define DIR_MASK        0x30
#define DIR_LEFT        0x01
#define DIR_RIGHT       0x02

#define DEG_CENTER      90
#define DEG_STEERING    33

#define WIFI_SSID       "TJ's House"
#define WIFI_PASSWORD   "cafebabe12"

/*
*****************************************************************************************
* H/W CONSTANTS (PINS)
*****************************************************************************************
*/

#define PIN_LED         5

#define PIN_SENSOR1     4
#define PIN_SENSOR1_DIR 0
#define PIN_IN1         16
#define PIN_IN2         17
#define PIN_SENSOR2     18
#define PIN_IN3         23
#define PIN_IN4         19

#define PIN_2WD_IN1     12
#define PIN_2WD_IN2     14
#define PIN_2WD_PWM     27
#define PIN_2WD_STEER   13
#define PIN_2WD_ESC_PWM PIN_2WD_IN1

/*
*****************************************************************************************
* MACROS & STRUCTURES
*****************************************************************************************
*/

#endif