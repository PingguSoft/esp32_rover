#include <math.h>
#include "utils.h"
#include "WheelDriver.h"
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
const uint8_t WheelDriver::_tblSpeed[27] = {
        0,  50,  60,  70,  80,  90, 100, 110, 115, 120,
    125, 130, 135, 140, 145, 150, 155, 160, 165, 170,
    175, 180, 185, 190, 195, 200, 205
};

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
uint8_t WheelDriver::_num = 0;
static const int16_t  _k_ctr_limit = 16384;

/*
*****************************************************************************************
* FUNCTIONS
*****************************************************************************************
*/

/*
*****************************************************************************************
* ISR
*****************************************************************************************
*/
void IRAM_ATTR isrHandlerPCNT(void* arg) {
    struct WheelDriver *pDriver = reinterpret_cast<WheelDriver*>(arg);
    uint32_t            status;

    pcnt_get_event_status((pcnt_unit_t)pDriver->_unit, &status);
    if (status & PCNT_EVT_L_LIM) {
        pDriver->_ticks_mult--;
    } else if (status & PCNT_EVT_H_LIM) {
        pDriver->_ticks_mult++;
    }
}

void initPCNT(pcnt_unit_t unit, int gpio_pulse, int gpio_ctrl = PCNT_PIN_NOT_USED,
        pcnt_channel_t channel = PCNT_CHANNEL_0, int16_t h_lim = 16384, int16_t l_lim = -16384) {

    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = gpio_pulse,       // set gpio for pulse input gpio
        .ctrl_gpio_num = gpio_ctrl,         // set gpio for control
        .lctrl_mode = PCNT_MODE_KEEP,       // Rising A on LOW B  = CW Step
        .hctrl_mode = PCNT_MODE_KEEP,       // Rising A on HIGH B = CCW Step
        .pos_mode = PCNT_COUNT_INC,         // increment the counter on positive edge
        .neg_mode = PCNT_COUNT_DIS,         // do nothing on falling edge
        .counter_h_lim = h_lim,
        .counter_l_lim = l_lim,
        .unit = unit,                       // PCNT unit number
        .channel = channel
    };
    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(unit, 50);
    pcnt_filter_enable(unit);

    /* Set threshold 0 and 1 values and enable events to watch */
    // pcnt_set_event_value(unit, PCNT_EVT_THRES_1, thres1);
    // pcnt_event_enable(unit, PCNT_EVT_THRES_1);
    // pcnt_set_event_value(unit, PCNT_EVT_THRES_0, thres0);
    // pcnt_event_enable(unit, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);

    pcnt_counter_pause(unit);
    pcnt_intr_enable(unit);
    pcnt_counter_resume(unit);
    pcnt_counter_clear(unit);
}


/*
*****************************************************************************************
* WheelDriver
*****************************************************************************************
*/
WheelDriver::WheelDriver(int8_t pin_esc, int8_t pin_ctr, int8_t pin_ctr_dir) {
    _type    = ESC;
    _pin_esc = pin_esc;
    _pin_ctr = pin_ctr;
    _pin_ctr_dir = pin_ctr_dir;
    _unit = _num++;
}

WheelDriver::WheelDriver(int8_t pin_in1, int8_t pin_in2, int8_t pin_ctr, int8_t pin_ctr_dir) {
    _type    = DRV8833;
    _pin_in1 = pin_in1;
    _pin_in2 = pin_in2;
    _pin_ctr = pin_ctr;
    _pin_ctr_dir = pin_ctr_dir;
    _unit = _num++;
}

WheelDriver::WheelDriver(int8_t pin_in1, int8_t pin_in2, int8_t pin_pwm, int8_t pin_ctr, uint8_t pin_ctr_dir) {
    _pin_esc = TB6612FNG;
    _pin_in1 = pin_in1;
    _pin_in2 = pin_in2;
    _pin_pwm = pin_pwm;
    _pin_ctr = pin_ctr;
    _pin_ctr_dir = pin_ctr_dir;
    _unit = _num++;
}

void WheelDriver::setup() {
    _speed = 0;
    if (_type == ESC) {
        _pESC = new Servo();
        _pESC->attach(_pin_esc, 1000, 2000);
        _pESC->setPeriodHertz(50);
        _pESC->write(1500);
    } else {
        pinMode(_pin_in1, OUTPUT);
        pinMode(_pin_in2, OUTPUT);
        digitalWrite(_pin_in1, LOW);
        digitalWrite(_pin_in2, LOW);

        if (_type == TB6612FNG) {
            pinMode(_pin_pwm, OUTPUT);
            digitalWrite(_pin_pwm, LOW);
            _pPwm[0] = new ESP32PWM();
            _pPwm[0]->attachPin(_pin_pwm, 500, 8);
        } else if (_type == DRV8833) {
            _pPwm[0] = new ESP32PWM();
            _pPwm[0]->attachPin(_pin_in1, 500, 8);
            LOG("DRV8833 PWM Ch:%d\n", _pPwm[0]->getChannel());

            _pPwm[1] = new ESP32PWM();
            _pPwm[1]->attachPin(_pin_in2, 500, 8);
            LOG("DRV8833 PWM Ch:%d\n", _pPwm[1]->getChannel());
        }
    }

    if (_pin_ctr_dir != PIN_NONE)
        pinMode(_pin_ctr_dir, INPUT_PULLUP);

    if (_pin_ctr != PIN_NONE) {
        pinMode(_pin_ctr, INPUT_PULLUP);
        initPCNT((pcnt_unit_t)_unit, _pin_ctr, _pin_ctr_dir, PCNT_CHANNEL_0, _k_ctr_limit, -_k_ctr_limit);
        pcnt_isr_service_install(0);
        pcnt_isr_handler_add((pcnt_unit_t)_unit, isrHandlerPCNT, (void*)this);
        reset();
    }
}

void WheelDriver::setSpeed(int speed) {
    int spd;

    speed = constrain(speed, -255, 255);
    if (speed >= 0) {
        if (_speed < 0 && _pin_ctr_dir == PIN_NONE) {
            pcnt_set_mode((pcnt_unit_t)_unit, PCNT_CHANNEL_0, PCNT_COUNT_INC, PCNT_COUNT_DIS, PCNT_MODE_KEEP, PCNT_MODE_KEEP);
        }
    } else {
        if (_speed >= 0 && _pin_ctr_dir == PIN_NONE) {
            pcnt_set_mode((pcnt_unit_t)_unit, PCNT_CHANNEL_0, PCNT_COUNT_DIS, PCNT_COUNT_DEC, PCNT_MODE_KEEP, PCNT_MODE_KEEP);
        }
    }

    switch (_type) {
        case ESC:
            spd = map(speed, -255, 255, 1000, 2000);
            _pESC->writeMicroseconds(spd);
            break;

        case DRV8833:
            spd = limitSpeed(abs(speed));
            if (speed >= 0) {
                _pPwm[0]->write(spd);
                _pPwm[1]->write(0);
            } else {
                _pPwm[0]->write(0);
                _pPwm[1]->write(spd);
            }
            break;

        case TB6612FNG:
            if (speed == 0) {
                digitalWrite(_pin_in1, LOW);
                digitalWrite(_pin_in2, LOW);
            } else if (speed > 0) {
                digitalWrite(_pin_in1, LOW);
                digitalWrite(_pin_in2, HIGH);
            } else {
                digitalWrite(_pin_in1, HIGH);
                digitalWrite(_pin_in2, LOW);
            }
            spd = limitSpeed(abs(speed));
            _pPwm[0]->write(spd);
            break;
    }
    _speed = speed;
}

uint8_t WheelDriver::limitSpeed(int speed) {
    uint8_t idx = speed / 10;
    uint8_t rem = speed % 10;
    float step  = (_tblSpeed[idx + 1] - _tblSpeed[idx]) / 10;

    uint8_t out = _tblSpeed[idx] + uint8_t(rem * step);
    out = map(out, 0, 255, 0, MOTOR_PWM_LIMIT);
    return out;
}

long WheelDriver::getTicks() {
    int16_t cnt;

    pcnt_get_counter_value((pcnt_unit_t)_unit, &cnt);
    return _ticks_mult * long(_k_ctr_limit) + cnt;
}

void WheelDriver::reset() {
    _ticks_mult = 0;
    if (_pin_ctr != PIN_NONE) {
        pcnt_counter_pause((pcnt_unit_t)_unit);
        pcnt_counter_clear((pcnt_unit_t)_unit);
        pcnt_counter_resume((pcnt_unit_t)_unit);
    }
}
