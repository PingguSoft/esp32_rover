#ifndef _WHEEL_DRIVER_H_
#define _WHEEL_DRIVER_H_
#include <ESP32Servo.h>
#include "config.h"


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

    WheelDriver(int8_t pin_esc, int8_t pin_ctr, int8_t pin_ctr_dir, bool reverse,                                   // ESC
                uint16_t radius, uint16_t tpr);
    WheelDriver(int8_t pin_in1, int8_t pin_in2, int8_t pin_ctr, int8_t pin_ctr_dir, bool reverse,                   // DRV8833
                uint16_t radius, uint16_t tpr);
    WheelDriver(int8_t pin_in1, int8_t pin_in2, int8_t pin_pwm, int8_t pin_ctr, uint8_t pin_ctr_dir, bool reverse,  // TB6612FNG
                uint16_t radius, uint16_t tpr);
    void     setup();
    void     reset();
    uint16_t getTPR()           { return _tpr;       }
    uint16_t getRadius()        { return _radius;    }

    void     setSpeed(int speed);
    int      getSpeed()         { return _speed;     }
    long     getTicks();
    int      getDegreePerTick() { return 360 / _tpr; }
    int      getDegree()        { return (getTicks() % _tpr) * getDegreePerTick(); }

    friend void isrHandlerPCNT(void *arg);

private:
    uint8_t limitSpeed(int speed);

    // for tracking odometry
    uint8_t  _unit;
    long     _ticks_mult;

    uint8_t  _type;
    bool     _reverse;
    int8_t   _pin_in1;
    int8_t   _pin_in2;
    int8_t   _pin_pwm;
    int8_t   _pin_esc;

    int8_t   _pin_ctr;
    int8_t   _pin_ctr_dir;
    int      _speed;
    ESP32PWM *_pPwm[2];
    Servo    *_pESC;

    uint16_t  _radius;
    uint16_t  _tpr;

    // for counting instances
    static uint8_t _num;
};
#endif
