#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_
#include <ESP32Servo.h>
#include "config.h"
#include "VecRot.h"
#include "WheelDriver.h"


/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/
enum {
    IDX_LWHEEL = 0,
    IDX_RWHEEL
};

/*
*****************************************************************************************
* MACROS & STRUCTURES
*****************************************************************************************
*/
typedef struct {
    unsigned long   millis;     // ms
    long            x;          // mm
    long            y;          // mm
    uint16_t        d_dist;     // mm
    float           theta;      // radian
} __attribute__((packed)) pose_t;

typedef struct {
    unsigned long   millis;     // ms
    long            left;       // ticks
    long            right;      // ticks
} __attribute__((packed)) odometry_t;

/*
*****************************************************************************************
* CLASS
*****************************************************************************************
*/
class Actuator {
public:
    enum {
        STEERING = 0,
        DIFFERENTIAL_DRIVE = 1
    };

    Actuator(WheelDriver *engine, int8_t pin_steer);    // steering wheel
    Actuator(WheelDriver *left, WheelDriver *right);    // differential drive

    void    setup();
    void    setMotor(int speedL, int speedR);
    void    drive(int angle, int speed);
    void    getOdometry(odometry_t *pOdometry);
    void    getPose(pose_t* pPose);
    void    resetPose();
    void    calibrate(int key);

private:
    void    getDelta(Odometry &odo, float *dtheta, float *dist);

    uint8_t         _type;
    int8_t          _pin_steer;
    WheelDriver     *_pDriver[2];
    Servo           *_pServo;

    pose_t          _pose;
    Odometry        _odo;
    Odometry        _last_odo;
};

#endif
