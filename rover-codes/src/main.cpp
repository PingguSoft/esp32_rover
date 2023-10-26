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
#include "actuator.h"
#include "ydlidar_x2.h"
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
    Actuator *_pActuator;

public:
    Tank(int option) : Car(option) {
        _pActuator = new Actuator(new WheelDriver(PIN_L_DRV_IN1, PIN_L_DRV_IN2, PIN_L_CTR, PIN_NONE),
                                  new WheelDriver(PIN_R_DRV_IN1, PIN_R_DRV_IN2, PIN_R_CTR, PIN_NONE));
    }

    void setup() {
        pinMode(PIN_DRV_EN, OUTPUT);
        digitalWrite(PIN_DRV_EN, HIGH);
        _pActuator->setup();
    }

    void command(int m) {
        static int    speed = 0;
        static int8_t angle = 0;

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
        }
        _pActuator->drive(angle, speed);
    }

    void loop() {
        if (Serial.available()) {
            int ch = Serial.read();
            //command(ch);
            _pActuator->calibrate(ch);
        }
    }

    void stick(int yaw, int throttle, int pitch, int roll, int btn) {
#if 1
        int speedL = map(throttle, 1000, 2000, -255, 255);
        int speedR = map(pitch,    1000, 2000, -255, 255);
        _pActuator->setMotor(speedL, speedR);
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
        _actuator.drive(angle, speed);
#endif
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
    Car*    _pCar;
    float   _speedSmooth;
};


/*
*****************************************************************************************
* setup
*****************************************************************************************
*/
static Car* _pCar = new Tank(0);
static StickCB _cb(_pCar);

static YDLidarX2 lidar;

static bool checkTurnLight(void* param) {
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    return true;
}

void setup() {
    // setCpuFrequencyMhz(80);
    Serial.begin(230400);
    Serial.setDebugOutput(true);
    esp_log_level_set("*", ESP_LOG_WARN);

    LOG("setup start !!! : heap:%d, psram:%d\n", ESP.getFreeHeap(), ESP.getPsramSize());
    pinMode(PIN_LED, OUTPUT);

    _timer.every(500, checkTurnLight);
    _joy.setStickCallback(&_cb);
    _joy.addSupportedDevices();
    _joy.begin();
    _pCar->setup();
    lidar.setup();
    lidar.enable(true);
}

/*
*****************************************************************************************
* loop
*****************************************************************************************
*/
void loop() {
    if (_joy.isConnecting()) {
        _joy.connect();
        if (_joy.isConnected()) {
            LOG("Connected to Joystick\n");
        } else {
            LOG("Failed to connect to Joystick\n");
        }
    }
    _timer.tick();
    //_pCar->loop();
    lidar.process();
}
