#ifndef _WHEEL_DRIVER_H_
#define _WHEEL_DRIVER_H_
#include <ESP32Servo.h>
#include "config.h"
#include "VecRot.h"


/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/


/*
*****************************************************************************************
* MACROS & STRUCTURES
*****************************************************************************************
*/


/*
*****************************************************************************************
* CLASS
*****************************************************************************************
*/
class WheelDriver {
public:
    enum {
        ESC = 0,
        DRV8833,
        TB6612FNG
    };

    WheelDriver(int8_t pin_esc, int8_t pin_ctr, int8_t pin_ctr_dir);                                    // ESC
    WheelDriver(int8_t pin_in1, int8_t pin_in2, int8_t pin_ctr, int8_t pin_ctr_dir);                    // DRV8833
    WheelDriver(int8_t pin_in1, int8_t pin_in2, int8_t pin_pwm, int8_t pin_ctr, uint8_t pin_ctr_dir);   // TB6612FNG
    void setup();
    void setSpeed(int speed);
    int  getSpeed()             { return _speed; }

    void    reset();
    long    getTicks();
    int     getDegreePerTick()  { return 360 / TICKS_PER_CYCLE; }
    int     getDegree()         { return (getTicks() % TICKS_PER_CYCLE) * getDegreePerTick(); }

    friend void isrHandlerPCNT(void *arg);

private:
    uint8_t limitSpeed(int speed);

    // for tracking odometry
    uint8_t  _unit;
    long     _ticks_mult;

    uint8_t  _type;
    int8_t   _pin_in1;
    int8_t   _pin_in2;
    int8_t   _pin_pwm;
    int8_t   _pin_esc;

    int8_t   _pin_ctr;
    int8_t   _pin_ctr_dir;
    int      _speed;
    ESP32PWM *_pPwm[2];
    Servo    *_pESC;

    // for counting instances
    static uint8_t _num;
    static const uint8_t _tblSpeed[];
};
#endif
