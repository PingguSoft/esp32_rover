#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>
#include <arduino-timer.h>
#include "driver/pcnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "utils.h"
#include "ControlStick.h"
using namespace std::placeholders;

/*
*****************************************************************************************
* VARIABLES
*****************************************************************************************
*/
static ControlStick     _joy;
static Timer<>          _timer = timer_create_default();

struct param_rc {
    int16_t  roll;
    int16_t  pitch;
    int16_t  yaw;
    int16_t  throttle;
    int16_t  aux[10];
    uint8_t  flag;
} __attribute__((packed));

typedef struct {
    uint8_t         unit;
    uint32_t        status;
    int16_t         count;
    unsigned long   timeStamp;
} pcnt_evt_t;


static xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
static pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle

/*
*****************************************************************************************
* ISR
*****************************************************************************************
*/
static void IRAM_ATTR pcnt_intr_handler(void* arg) {
    unsigned long   ts = millis();
    pcnt_evt_t      evt;
    portBASE_TYPE   HPTaskAwoken = pdFALSE;
    int             unit = reinterpret_cast<int>(arg);

    evt.unit         = unit;
    evt.timeStamp    = ts;
    pcnt_get_event_status((pcnt_unit_t)unit, &evt.status);
    pcnt_get_counter_value((pcnt_unit_t)unit, &evt.count);
    xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);

    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }

}

void pcnt_init_channel(pcnt_unit_t unit, int pulse_gpio, int ctrl_gpio = PCNT_PIN_NOT_USED,
        pcnt_channel_t channel = PCNT_CHANNEL_0, int16_t h_lim = 32767, int16_t l_lim = 0,
        int16_t thres1 = 50, int16_t thres0 = -50) {

    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = pulse_gpio,       // set gpio for pulse input gpio
        .ctrl_gpio_num = ctrl_gpio,         // no gpio for control
        .lctrl_mode = PCNT_MODE_REVERSE,    // when control signal is low, counter mode : decrease
        .hctrl_mode = PCNT_MODE_KEEP,       // when control signal is high, keep the primary counter mode
        .pos_mode = PCNT_COUNT_INC,         // increment the counter on positive edge
        .neg_mode = PCNT_COUNT_DIS,         // do nothing on falling edge
        .counter_h_lim = h_lim,
        .counter_l_lim = l_lim,
        .unit = unit,                       /*!< PCNT unit number */
        .channel = channel
    };

    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);

    /* Set threshold 0 and 1 values and enable events to watch */
    // pcnt_set_event_value(unit, PCNT_EVT_THRES_1, thres1);
    // pcnt_event_enable(unit, PCNT_EVT_THRES_1);
    // pcnt_set_event_value(unit, PCNT_EVT_THRES_0, thres0);
    // pcnt_event_enable(unit, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(unit, PCNT_EVT_ZERO);
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    //pcnt_event_enable(unit, PCNT_EVT_L_LIM);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    //pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(unit);
    pcnt_counter_resume(unit);
}

int countRPM(int firstTime, int lastTime, int pulseTotal, int pulsePerRev) {
    int timeDelta = (lastTime - firstTime);
    if (timeDelta <= 0) {
        return -1;
    }
    return ((60000 * (pulseTotal / pulsePerRev)) / timeDelta);
}



/*
*****************************************************************************************
* CLASS Car
*****************************************************************************************
*/
class Car {
public:
    Car(int option) {
        _option = option;
    }

    virtual void setup() = 0;
    virtual void drive(int angle, int speed) = 0;
    virtual void stick(int yaw, int throttle, int pitch, int roll, int btn) = 0;
    virtual void loop() { }

protected:
    int _option;
};


/*
*****************************************************************************************
* CLASS Tank
*****************************************************************************************
*/
class Tank : public Car {
private:
    ESP32PWM _pwm[4];

    const uint8_t _tblSpeed[27] = {
          0,  50,  60,  70,  80,  90, 100, 110, 115, 120,
        125, 130, 135, 140, 145, 150, 155, 160, 165, 170,
        175, 180, 185, 190, 195, 200, 205
    };

    uint8_t getSpeed(uint8_t spd) {
        uint8_t idx = spd / 10;
        uint8_t rem = spd % 10;
        float step  = (_tblSpeed[idx + 1] - _tblSpeed[idx]) / 10;

        uint8_t out = _tblSpeed[idx] + uint8_t(rem * step);

        out = map(out, 0, 255, 0, 100);
        return out;
    }

public:
    Tank(int option) : Car(option) {
    }

    void setup() {
        pinMode(PIN_IN1, OUTPUT);
        pinMode(PIN_IN2, OUTPUT);
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, LOW);

        pinMode(PIN_IN3, OUTPUT);
        pinMode(PIN_IN4, OUTPUT);
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, LOW);

        pinMode(PIN_SENSOR1,     INPUT_PULLUP);
        pinMode(PIN_SENSOR1_DIR, INPUT_PULLUP);
        pinMode(PIN_SENSOR2,     INPUT_PULLUP);
        
        
        pcnt_init_channel(PCNT_UNIT_0, PIN_SENSOR1, PIN_SENSOR1_DIR);
        pcnt_init_channel(PCNT_UNIT_1, PIN_SENSOR2);
        pcnt_isr_service_install(0);
        pcnt_isr_handler_add(PCNT_UNIT_0, pcnt_intr_handler, (void*)PCNT_UNIT_0);
        pcnt_isr_handler_add(PCNT_UNIT_1, pcnt_intr_handler, (void*)PCNT_UNIT_1);

        _pwm[0].attachPin(PIN_IN1, 500, 8);
        _pwm[1].attachPin(PIN_IN2, 500, 8);
        _pwm[2].attachPin(PIN_IN3, 500, 8);
        _pwm[3].attachPin(PIN_IN4, 500, 8);
    }

    void loop() {
        pcnt_evt_t      evt;
        portBASE_TYPE   res;

        res = xQueueReceive(pcnt_evt_queue, &evt, 0);
        if (res == pdTRUE) {
            LOG("Event PCNT unit%d => status: %2x, count:%5u\n", evt.unit, evt.status, evt.count);
        }
    }

    void setMotor(int speedL, int speedR) {
        // left
        uint8_t spdL = getSpeed(abs(speedL));
        uint8_t spdR = getSpeed(abs(speedR));

        if (speedL >= 0) {
            _pwm[2].write(spdL);
            _pwm[3].write(0);
            //analogWrite(PIN_IN3, spdL);
            //analogWrite(PIN_IN4, 0);
        } else {
            _pwm[2].write(0);
            _pwm[3].write(spdL);
            //analogWrite(PIN_IN3, 0);
            //analogWrite(PIN_IN4, spdL);
        }

        // right
        if (speedR >= 0) {
            _pwm[0].write(spdR);
            _pwm[1].write(0);
            // analogWrite(PIN_IN1, spdR);
            // analogWrite(PIN_IN2, 0);
        } else {
            _pwm[0].write(0);
            _pwm[1].write(spdR);
            // analogWrite(PIN_IN1, 0);
            // analogWrite(PIN_IN2, spdR);
        }
        LOG("%6d=>%6d : %6d=>%6d\n", speedL, spdL, speedR, spdR);
    }

    void drive(int angle, int speed) {
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
    }

    void stick(int yaw, int throttle, int pitch, int roll, int btn) {
#if 1
        int speedL = map(throttle, 1000, 2000, -255, 255);
        int speedR = map(pitch,    1000, 2000, -255, 255);
        setMotor(speedL, speedR);
#else
        int angle;
        int speed1, speed;

        pitch -= 1500;
        roll -= 1500;
        speed1 = constrain(sqrt(pitch * pitch + roll * roll), 0, 500);
        angle = (roll == 0) ? 0 : (90 - degrees(atan2((float)pitch, (float)roll)));
        speed = map(speed1, 0, 500, 0, MOTOR_PWM_LIMIT);

        if (speed > 0) {
            LOG("pit:%6d, rol:%6d, ang:%6d, speed:%6d => %6d\n", pitch, roll, angle, speed1, speed);
        }
        drive(angle, speed);
#endif
    }
};


/*
*****************************************************************************************
* CLASS Car2WD
*****************************************************************************************
*/
class Car2WD : public Car {
private:
    ESP32PWM _pwm[2];
    Servo   _servoSteer;
    Servo   _dcESC;
    float   _speedSmooth;

public:
    Car2WD(int option) : Car(option) {
        _speedSmooth = 0;
    }

    void setup() {
        switch (_option) {
            case MOTOR_DRIVER_DRV8833:
                pinMode(PIN_2WD_IN1, OUTPUT);
                pinMode(PIN_2WD_IN2, OUTPUT);
                digitalWrite(PIN_2WD_IN1, LOW);
                digitalWrite(PIN_2WD_IN2, LOW);
                _pwm[0].attachPin(PIN_2WD_IN1, 100, 8);
                _pwm[1].attachPin(PIN_2WD_IN2, 100, 8);
                break;

            case MOTOR_DRIVER_TB6612FNG:
                pinMode(PIN_2WD_PWM, OUTPUT);
                digitalWrite(PIN_2WD_PWM, LOW);
                break;

            case MOTOR_DRIVER_DC_ESC:
                _dcESC.attach(PIN_2WD_ESC_PWM, 1000, 2000);
                _dcESC.setPeriodHertz(50);
                _dcESC.write(1500);
                break;
        }
        _servoSteer.attach(PIN_2WD_STEER, 500, 2500);
        _servoSteer.setPeriodHertz(50);
        _servoSteer.write(DEG_CENTER);
    }

    void drive(int angle, int speed) {
        int dir = 0;
        int limAbsSpeed = min(abs(speed), 255);

        switch (_option) {
            case MOTOR_DRIVER_DRV8833:
                if (speed >= 0) {
                    _pwm[0].write(0);
                    _pwm[1].write(limAbsSpeed);
                    dir = 1;
                } else {
                    _pwm[0].write(limAbsSpeed);
                    _pwm[1].write(0);
                    dir = -1;
                }
                break;

            case MOTOR_DRIVER_TB6612FNG:
                if (speed == 0) {
                    digitalWrite(PIN_2WD_IN1, LOW);
                    digitalWrite(PIN_2WD_IN2, LOW);
                    dir = 0;
                } else if (speed > 0) {
                    digitalWrite(PIN_2WD_IN1, LOW);
                    digitalWrite(PIN_2WD_IN2, HIGH);
                    dir = 1;
                } else {
                    digitalWrite(PIN_2WD_IN1, HIGH);
                    digitalWrite(PIN_2WD_IN2, LOW);
                    dir = -1;
                }
                analogWrite(PIN_2WD_PWM, limAbsSpeed);
                break;

            case MOTOR_DRIVER_DC_ESC:
                limAbsSpeed = map(speed, -255, 255, 1000, 2000);
                _dcESC.write(limAbsSpeed);
                break;
        }
        _servoSteer.write(angle + DEG_CENTER);
        LOG("ang:%6d,  spd:%6d\n", angle, limAbsSpeed);
    }

    void stick(int yaw, int throttle, int pitch, int roll, int btn) {
        int speed = map(throttle, 1000, 2000, -255, 255);
        int angle = map(roll, 1000, 2000, -DEG_STEERING, DEG_STEERING);

        //_speedSmooth = (speed * 0.05) + (_speedSmooth * 0.95);
        drive(angle, int(speed));
    }
};


/*
*****************************************************************************************
* CLASS StickCB
*****************************************************************************************
*/
class StickCB : public StickCallback {
public:
    StickCB(Car* car) {
        _pCar = car;
    }

    void onConnect() {
    }

    void onDisconnect() {
    }

    uint16_t filterDeadZone(uint16_t val) {
        if (1480 <= val && val <= 1520) {
            val = 1500;
        }
        return val;
    }

    void onStickChanged(int axisX, int axisY, int axisZ, int axisRZ, int axisLT, int axisRT, int dpad, int btns) {
        static struct param_rc rc = {
            1500, 1500, 1500, 1500,
            1000, 1000, 1000, 1000, 1000,
            1000, 1000, 1000, 1000, 1000,
            1
        };

        static const uint8_t _kTblDPadMap[] = {
            0x00,
            0x01,   // up
            0x03,
            0x02,   // right
            0x06,
            0x04,   // down
            0x0c,
            0x08,   // left
            0x09,
        };

        rc.yaw = filterDeadZone(map(axisX, 0, 1024, 1000, 2000));
        rc.throttle = filterDeadZone(map(axisY, 0, 1024, 2000, 1000));
        rc.roll = filterDeadZone(map(axisZ, 0, 1024, 1000, 2000));
        rc.pitch = filterDeadZone(map(axisRZ, 0, 1024, 2000, 1000));

        uint8_t  dp = (dpad > 8) ? 0 : _kTblDPadMap[dpad];
        uint32_t btn = (int(dp) << 16) | btns;

        _pCar->stick(rc.yaw, rc.throttle, rc.pitch, rc.roll, btn);
    }

private:
    Car* _pCar;
    float   _speedSmooth;
};


/*
*****************************************************************************************
* setup
*****************************************************************************************
*/
static Car* _pCar = new Tank(MOTOR_DRIVER_DRV8833);
// static Car* _pCar = new Car2WD(MOTOR_DRIVER_DC_ESC);
static StickCB _cb(_pCar);

static bool checkTurnLight(void* param) {
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    return true;
}

void setup() {
    setCpuFrequencyMhz(80);
    Serial.begin(115200);
    LOG("setup start !!! : heap:%d, psram:%d\n", ESP.getFreeHeap(), ESP.getPsramSize());
    pinMode(PIN_LED, OUTPUT);

    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));

    _pCar->setup();
    _timer.every(500, checkTurnLight);
    _joy.setStickCallback(&_cb);
    _joy.addSupportedDevices();
    _joy.begin();
}


/*
*****************************************************************************************
* loop
*****************************************************************************************
*/
void command(int m) {
    static int    speed = 0;
    static int8_t angle = 0;
    int16_t cnt1, cnt2;

    switch (m) {
        case ',':
            if (speed > -255)
                speed--;
            break;

        case '.':
            if (speed < 255)
                speed++;
            break;

        case ' ':
            angle = 0;
            speed = 0;
            break;

        case '[':
            if (angle > -90)
                angle--;
            break;

        case ']':
            if (angle < 90)
                angle++;
            break;

        case 'p':
            pcnt_get_counter_value(PCNT_UNIT_0, &cnt1);
            pcnt_get_counter_value(PCNT_UNIT_1, &cnt2);
            LOG("sensor1:%8d, sensor2:%8d\n", cnt1, cnt2);
            break;

        case 'q':
        {
            int s1 = digitalRead(PIN_SENSOR1);
            int s2 = digitalRead(PIN_SENSOR2);

            while (!Serial.available()) {
                int s1r = digitalRead(PIN_SENSOR1);
                int s2r = digitalRead(PIN_SENSOR2);

                if (s1r != s1 || s2r != s2) {
                    LOG("sensor:%d, %d\n", s1r, s2r);
                    s1 = s1r;
                    s2 = s2r;
                }
            }
        }
        break;
    }
    _pCar->drive(angle, speed);
}

void loop() {
    if (_joy.isConnecting()) {
        _joy.connect();
        if (_joy.isConnected()) {
            LOG("Connected to Joystick\n");
        } else {
            LOG("Failed to connect to Joystick\n");
        }
    }

    if (Serial.available()) {
        int ch = Serial.read();
        command(ch);
    }

    _timer.tick();
    _pCar->loop();
}
