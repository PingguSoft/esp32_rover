#include <math.h>
#include "utils.h"
#include "actuator.h"
#include "driver/pcnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"


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
* VARIABLES
*****************************************************************************************
*/


/*
*****************************************************************************************
* FUNCTIONS
*****************************************************************************************
*/
Actuator::Actuator(WheelDriver *engine, int8_t pin_steer, uint16_t axle_width) :
    _type(STEERING),
    _pin_steer(pin_steer),
    _axle_width(axle_width),
    _pDriver { engine, NULL }
{
    reset(false);
}

Actuator::Actuator(WheelDriver *left, WheelDriver *right, uint16_t axle_width) :
    _type(DIFFERENTIAL_DRIVE),
    _pin_steer(PIN_NONE),
    _axle_width(axle_width),
    _pDriver { left, right }
{
    reset(false);
}

void Actuator::setup() {
    for (int i = 0; i < ARRAY_SIZE(_pDriver); i++) {
        if (_pDriver[i])
            _pDriver[i]->setup();
    }

    if (_type == STEERING) {
        _pServo = new Servo();
        _pServo->attach(_pin_steer, 500, 2500);
        _pServo->setPeriodHertz(50);
        _pServo->write(1500);
    }
}

void Actuator::setMotor(int speedL, int speedR) {
    if (_type == DIFFERENTIAL_DRIVE) {
        if (_pDriver[IDX_LWHEEL])
            _pDriver[IDX_LWHEEL]->setSpeed(speedL);
        if (_pDriver[IDX_RWHEEL])
            _pDriver[IDX_RWHEEL]->setSpeed(speedR);
    }
}

void Actuator::drive(int angle, int speed) {
    if (_type == DIFFERENTIAL_DRIVE) {
        int speedL;
        int speedR;

        if (angle >= 0) {
            float rad = radians(angle);
            speedL = speed;
            speedR = speed * cos(rad);
        } else {
            float rad = radians(-angle);
            speedL = speed * cos(rad);
            speedR = speed;
        }
        //LOG("ang:%6d,  spd:%6d, %6d\n", angle, speedL, speedR);
        setMotor(speedL, speedR);
        _angle = angle;
    } else if (_type == STEERING) {
        _pDriver[IDX_LWHEEL]->setSpeed(speed);
        _pServo->write(angle);
    }
}

void Actuator::getDelta(Ticks &a, Ticks &b, float *dtheta, float *ddist) {
    Ticks dlt = a - b;
    float len_l  = 2 * M_PI * _pDriver[IDX_LWHEEL]->getRadius();
    float dist_l = len_l * dlt.get().left  / _pDriver[IDX_LWHEEL]->getTPR();

    float len_r  = (_type == DIFFERENTIAL_DRIVE) ? (2 * M_PI * _pDriver[IDX_RWHEEL]->getRadius()) : len_l;
    float dist_r = (_type == DIFFERENTIAL_DRIVE) ? (len_r * dlt.get().right / _pDriver[IDX_RWHEEL]->getTPR()) : dist_l;
    *dtheta = (dist_r - dist_l) / _axle_width;
    *ddist  = (dist_r + dist_l) / 2;
}

Ticks Actuator::getTicks(Ticks *ticks) {
    Ticks t;

    if (_type == STEERING) {
        t.set(millis(), _pDriver[IDX_LWHEEL]->getTicks(), _pDriver[IDX_LWHEEL]->getTicks());
    } else {
        t.set(millis(), _pDriver[IDX_LWHEEL]->getTicks(), _pDriver[IDX_RWHEEL]->getTicks());
    }
    if (ticks)
        *ticks = t;
    return t;
}

Odometry* Actuator::updateOdometry() {
    if (_type == STEERING) {
        _ticks.set(millis(), _pDriver[IDX_LWHEEL]->getTicks(), _pDriver[IDX_LWHEEL]->getTicks());
    } else {
        _ticks.set(millis(), _pDriver[IDX_LWHEEL]->getTicks(), _pDriver[IDX_RWHEEL]->getTicks());
    }

    float dtheta, ddist;
    getDelta(_ticks, _last_ticks, &dtheta, &ddist);
    if (_type == STEERING) {
        dtheta = _angle - _last_angle;
        _last_angle = _angle;
    }
    _last_ticks = _ticks;

    float dx  = ddist * cos(_odometry.get().theta);
    float dy  = ddist * sin(_odometry.get().theta);
    _odometry.acc(millis(), dx, dy, dtheta);

    // LOG("%8ld tick:%8ld,%8ld, dtheta:%6.2f, dist:%6.2f, delta_xy:%6.2f, %6.2f, odometry:%8ld, %8ld, %6.2f\n",
    //     _odometry.millis, _ticks.get().left, _ticks.get().left, dtheta, ddist, dx, dy, _odometry.x, _odometry.y, _odometry.theta);
    return &_odometry;
}

void Actuator::reset(bool driver) {
    if (driver) {
        for (int i = 0; i < ARRAY_SIZE(_pDriver); i++) {
            _pDriver[i]->reset();
        }
    }
    _ticks.reset();
    _last_ticks.reset();
    _odometry.reset();
    _angle = 0;
    _last_angle = 0;
}

void Actuator::calibrate(int key) {
    static int idx = 0;
    int      speed;
    float    rot;
    Ticks delta;

    switch (key) {
        case '1':
        case '2':
            idx = (_type == DIFFERENTIAL_DRIVE) ? (key - '1') : 0;
            break;

        case ',':
            speed = _pDriver[idx]->getSpeed();
            speed -= 5;
            _pDriver[idx]->setSpeed(speed);
            LOG("wheel:%d, speed:%4d\n", idx, speed);
            break;

        case '.':
            speed = _pDriver[idx]->getSpeed();
            speed += 5;
            _pDriver[idx]->setSpeed(speed);
            LOG("wheel:%d, speed:%4d\n", idx, speed);
            break;

        case 'r':
            reset();
            // no break

        case '/':
            _last_ticks = _ticks;
            if (_type == STEERING) {
                _ticks.set(millis(), _pDriver[IDX_LWHEEL]->getTicks(), _pDriver[IDX_LWHEEL]->getTicks());
            } else {
                _ticks.set(millis(), _pDriver[IDX_LWHEEL]->getTicks(), _pDriver[IDX_RWHEEL]->getTicks());
            }

            delta = _ticks - _last_ticks;
            rot  = delta.get().left / _pDriver[IDX_LWHEEL]->getTPR();
            rot  = rot * 60000 / delta.get().millis;
            LOG("wheel_l, ctr:%8ld, rpm:%6.1f\n", _ticks.get().left,  rot);

            if (_type == DIFFERENTIAL_DRIVE) {
                rot  = delta.get().right / _pDriver[IDX_RWHEEL]->getTPR();
                rot  = rot * 60000 / delta.get().millis;
                LOG("wheel_r, ctr:%8ld, rpm:%6.1f\n", _ticks.get().right, rot);
            }
            break;

        case 'd':
            for (int i = 0; i < ARRAY_SIZE(_pDriver); i++) {
                if (_pDriver[i])
                    _pDriver[i]->setSpeed(0);
            }
            LOG("stop !!!\n");
            break;
    }
}
