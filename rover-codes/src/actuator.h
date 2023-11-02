#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_
#include <ESP32Servo.h>
#include "config.h"
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


/*
*****************************************************************************************
* CLASS
*****************************************************************************************
*/
class Ticks {
public:
    typedef struct {
        unsigned long   millis;     // ms
        long            left;       // ticks
        long            right;      // ticks
    } __attribute__((packed)) ticks_t;

    Ticks() {
        set(0, 0, 0);
    }

    Ticks(unsigned long ts, long l, long r) {
        set(ts, l, r);
    }

    void set(unsigned long ts, long l, long r) {
        _t.millis = ts;
        _t.left   = l;
        _t.right  = r;
    }

    ticks_t get() {
        return _t;
    }

    void reset() {
        set(0, 0, 0);
    }

	Ticks operator-(Ticks &ref) {
		return Ticks(_t.millis - ref._t.millis, _t.left - ref._t.left, _t.right - ref._t.right);
	}

private:
    ticks_t  _t;
};

class Odometry {
public:
    typedef struct {
        unsigned long   millis;     // ms
        long            x;          // mm
        long            y;          // mm
        float           theta;      // radian
    } __attribute__((packed)) odometry_t;

    Odometry() {
        set(0, 0, 0, 0);
    }

    Odometry(unsigned long ts, long x, long y, float theta) {
        set(ts, x, y, theta);
    }

    void set(unsigned long ts, long x, long y, float theta) {
        _odo.millis = ts;
        _odo.x      = x;
        _odo.y      = y;
        _odo.theta  = theta;
    }

    void acc(unsigned long ts, long dx, long dy, float dtheta) {
        float pi2 = (2 * M_PI);

        _odo.millis = ts;
        _odo.x += dx;
        _odo.y += dy;
        _odo.theta += dtheta;
        if (_odo.theta >= pi2)
            _odo.theta -= pi2;
        else if (_odo.theta <= -pi2)
            _odo.theta += pi2;
    }

    odometry_t get() {
        return _odo;
    }

    void reset() {
        set(0, 0, 0, 0);
    }

	Odometry operator-(Odometry &ref) {
		return Odometry(_odo.millis - ref._odo.millis, _odo.x - ref._odo.x, _odo.y - ref._odo.y,
            _odo.theta - ref._odo.theta);
	}

private:
    odometry_t  _odo;

private:
};

class Actuator {
public:
    enum {
        STEERING = 0,
        DIFFERENTIAL_DRIVE = 1
    };

    Actuator(WheelDriver *engine, int8_t pin_steer, uint16_t axle_width);    // steering wheel
    Actuator(WheelDriver *left, WheelDriver *right, uint16_t axle_width);    // differential drive

    void    setup();
    void    setMotor(int speedL, int speedR);
    void    drive(int angle, int speed);
    void    reset(bool driver=true);
    void    calibrate(int key);
    Ticks       getTicks(Ticks *ticks=NULL);
    bool        updateOdometry();
    Odometry*   getOdometry()                   { return &_odometry; }

private:
    bool    getDelta(Ticks &a, Ticks &b, float *dtheta, float *ddist);

    uint8_t         _type;
    int8_t          _pin_steer;
    WheelDriver     *_pDriver[2];
    Servo           *_pServo;
    uint16_t        _axle_width;

    Odometry        _odometry;
    int             _angle;
    int             _last_angle;
    Ticks           _ticks;
    Ticks           _last_ticks;
};

#endif
