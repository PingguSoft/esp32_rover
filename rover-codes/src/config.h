#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <Arduino.h>

/*
*****************************************************************************************
* FEATURES
*****************************************************************************************
*/
#define MOTOR_PWM_LIMIT         120

/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/
#define WIFI_SSID       "TJ's House"
#define WIFI_PASSWORD   "cafebabe12"

#define WHEEL_RADIUS_MM     (65 / 2)
#define AXLE_WIDTH_MM       150
#define TICKS_PER_CYCLE     20

/*
*****************************************************************************************
* H/W CONSTANTS (PINS)
*****************************************************************************************
*/
#define PIN_NONE        -1

#define PIN_LED         5
#define PIN_DRV_EN      2
#define PIN_L_CTR       4
#define PIN_L_DRV_IN1   23
#define PIN_L_DRV_IN2   19
#define PIN_R_CTR       18
#define PIN_R_DRV_IN1   16
#define PIN_R_DRV_IN2   17

#define PIN_LIDAR_RX    13
#define PIN_LIDAR_PWM   14


/*
*****************************************************************************************
* MACROS & STRUCTURES
*****************************************************************************************
*/

#endif