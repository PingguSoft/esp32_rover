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
Actuator::Actuator(WheelDriver *engine, int8_t pin_steer) :
    _type(STEERING),
    _pin_steer(pin_steer),
    _pDriver { engine, NULL },
    _pose { 0, 0, 0 }
{
    _odo.reset();
    _last_odo.reset();
}

Actuator::Actuator(WheelDriver *left, WheelDriver *right) :
    _type(DIFFERENTIAL_DRIVE),
    _pin_steer(PIN_NONE),
    _pDriver { left, right },
    _pose { 0, 0, 0 }
{
    _odo.reset();
    _last_odo.reset();
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
        // LOG(">o1:%d\n>o2:%d\n", speedL, speedR);
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
    } else if (_type == STEERING) {
        _pDriver[IDX_LWHEEL]->setSpeed(speed);
        _pServo->write(angle);
    }
}

void Actuator::getDelta(Odometry &odo, float *dtheta, float *dist) {
    float w_circ = 2 * M_PI * WHEEL_RADIUS_MM;
    float dist_l = w_circ * odo.left  / TICKS_PER_CYCLE;
    float dist_r = w_circ * odo.right / TICKS_PER_CYCLE;
    *dtheta = (dist_r - dist_l) / (2 * AXLE_HALF_WIDTH_MM);
    *dist   = (dist_r + dist_l) / 2;
}

void Actuator::getPose(pose_t *pPose) {
    _last_odo = _odo;

    if (_type == STEERING) {
        _odo.set(millis(), _pDriver[IDX_LWHEEL]->getTicks(), _pDriver[IDX_LWHEEL]->getTicks());
    } else {
        _odo.set(millis(), _pDriver[IDX_LWHEEL]->getTicks(), _pDriver[IDX_RWHEEL]->getTicks());
    }

    Odometry diffOdo = _odo - _last_odo;
    float dtheta, dist;
    getDelta(diffOdo, &dtheta, &dist);

    if (_type == STEERING) {
        dtheta = 1; // check!!! - calc angle diff in future
    }

    float dx  = dist * cos(_pose.theta);
    float dy  = dist * sin(_pose.theta);
    float pi2 = (2 * M_PI);

    // global coord information
    _pose.millis = millis();
    _pose.d_dist = dist;
    _pose.x     += dx;
    _pose.y     += dy;
    _pose.theta += dtheta;
    if (_pose.theta >= pi2)
        _pose.theta -= pi2;
    else if (_pose.theta <= -pi2)
        _pose.theta += pi2;

    // LOG("%8ld tick:%8ld,%8ld, diff_tick:%8ld,%8ld, dtheta:%6.2f, dist:%6.2f, delta_xy:%6.2f, %6.2f, pose:%8ld, %8ld, %6.2f\n",
    //     _pose.millis, tick_l, tick_r, tick_diff_l, tick_diff_r, dtheta, dist, dx, dy, _pose.x, _pose.y, _pose.theta);
    *pPose = _pose;
}

void Actuator::resetPose() {
    for (int i = 0; i < ARRAY_SIZE(_pDriver); i++) {
        _pDriver[i]->reset();
    }
    _odo.reset();
    _last_odo.reset();
    _pose = { 0, 0, 0};
}

void Actuator::calibrate(int key) {
    static int idx = 0;
    int      speed;
    float    rot;
    Odometry delta;

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
            for (int i = 0; i < ARRAY_SIZE(_pDriver); i++) {
                if (_pDriver[i])
                    _pDriver[i]->reset();
            }
            // no break

        case '/':
            _last_odo = _odo;
            if (_type == STEERING) {
                _odo.set(millis(), _pDriver[IDX_LWHEEL]->getTicks(), _pDriver[IDX_LWHEEL]->getTicks());
            } else {
                _odo.set(millis(), _pDriver[IDX_LWHEEL]->getTicks(), _pDriver[IDX_RWHEEL]->getTicks());
            }
            delta = _odo - _last_odo;
            rot  = delta.left / TICKS_PER_CYCLE;
            rot  = rot * 60000 / delta.millis;
            LOG("wheel_l, ctr:%8ld, rpm:%6.1f\n", _odo.left,  rot);
            rot  = delta.right / TICKS_PER_CYCLE;
            rot  = rot * 60000 / delta.millis;
            LOG("wheel_r, ctr:%8ld, rpm:%6.1f\n", _odo.right, rot);
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
