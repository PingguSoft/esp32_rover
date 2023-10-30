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
#include "wifi_service.h"
#include "TimeEvent.h"
using namespace std::placeholders;

/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/
#define FORWARD_NONE    0
#define FORWARD_SERIAL  1
#define FORWARD_WIFI    2
#define DEBUG_FORWARD   FORWARD_WIFI


/*
*****************************************************************************************
* CLASS SerialService
*****************************************************************************************
*/
#if (DEBUG_FORWARD == FORWARD_SERIAL)
class SerialService  : public MSPInterface, public CommService {
public:
    SerialService() {
    }

    //
    // MSPInterface implementation
    //
    virtual void write(uint8_t *pBuf, uint16_t size) {
        uint16_t frag;
        uint8_t  *data = pBuf;

        while (size > 0) {
            frag = (size > 512) ? 512 : size;
            Serial.write(data, frag);
            data += frag;
            size -= frag;
        }
    }

    //
    // CommService implementation
    //
    void setup(char *ssid, char *password, int port, MSPCallback *cb) {
        _queue_comm  = xQueueCreate(5, sizeof(comm_q_t));
        xTaskCreate(&task_comm, "task_comm", 2560, this, 8, NULL);

        _pMSP = new MSP(1300);
        _pMSP->setInterface(this);
        _pMSP->setCallback(cb);
    }

    void send(cmd_t cmd, uint8_t *buf, uint16_t size, bool reqBufDel=pdFALSE, bool reqSeqHeader=pdFALSE) {
        comm_q_t q = { cmd, buf, size, reqBufDel, reqSeqHeader };
        xQueueSend(_queue_comm, &q, portMAX_DELAY);
    }

    //
    //
    //
    static void task_comm(void* arg) {
        SerialService *pSvc = (SerialService*)arg;
        comm_q_t *q = new comm_q_t;

        LOG("task comm started\n");
        while (true) {
            if (xQueueReceive(pSvc->_queue_comm, q, portMAX_DELAY) == pdTRUE) {
                pSvc->_pMSP->send(q->cmd, q->pData, q->size);
                if (q->reqBufDel)
                    delete q->pData;
            }
        }
        delete q;
        vTaskDelete(NULL);
    }

private:
    MSP          *_pMSP;
    QueueHandle_t _queue_comm;
};
#endif

/*
*****************************************************************************************
* CLASS Vehicle
*****************************************************************************************
*/
static const uint8_t _tblSpeed[27] = {
        0,  50,  60,  70,  80,  90, 100, 110, 115, 120,
    125, 130, 135, 140, 145, 150, 155, 160, 165, 170,
    175, 180, 185, 190, 195, 200, 205
};

#define BTN_SHIFT_L         _BV(ControlStick::BTN_L2)
#define BTN_SHIFT_R         _BV(ControlStick::BTN_R2)
#define BTN_MODE0           _BV(ControlStick::BTN_A)
#define BTN_MODE1           _BV(ControlStick::BTN_B)

class Vehicle : public MSPCallback, public StickCallback {
private:
    Actuator       *_pActuator;
    int             _oldBtn;
    int             _mode;
    ButtonTracker   _btn_trk;
    TimeEvent       _evt;

#if (DEBUG_FORWARD == FORWARD_SERIAL)
    SerialService   _service;
#elif (DEBUG_FORWARD == FORWARD_WIFI)
    WiFiService     _service;
#endif

public:
    Vehicle(int option) : _btn_trk(BTN_SHIFT_L | BTN_SHIFT_R), _evt(5)
    {
        _pActuator = new Actuator(new WheelDriver(PIN_L_DRV_IN1, PIN_L_DRV_IN2, PIN_L_CTR, PIN_NONE, false),
                                  new WheelDriver(PIN_R_DRV_IN1, PIN_R_DRV_IN2, PIN_R_CTR, PIN_NONE, false));
        _oldBtn      = 0;
        _mode        = 0;
    }

    void setup() {
        pinMode(PIN_DRV_EN, OUTPUT);
        digitalWrite(PIN_DRV_EN, HIGH);
        _pActuator->setup();
        _service.setup((char*)WIFI_SSID, (char*)WIFI_PASSWORD, 8080, this);
    }

    CommService *getComm() {
#if (DEBUG_FORWARD == FORWARD_NONE)
        return NULL;
#else
        return &_service;
#endif
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
        _evt.tick();
    }

    int16_t limitSpeed(int speed) {
        uint8_t  spd = abs(speed);
        uint8_t  idx = spd / 10;
        uint8_t  rem = spd % 10;
        float step  = (_tblSpeed[idx + 1] - _tblSpeed[idx]) / 10;
        uint8_t out = _tblSpeed[idx] + uint8_t(rem * step);

#ifdef MOTOR_PWM_LIMIT
        out = map(out, 0, 255, 0, MOTOR_PWM_LIMIT);
#endif
        return (speed < 0) ? -out : out;
    }

    void stick(int yaw, int throttle, int pitch, int roll, int btn) {
        _btn_trk.begin(btn);

        if (_btn_trk.isPressed(BTN_MODE0)) {
            _mode = 0;
            LOG("mode:%d\n", _mode);
        }
        if (_btn_trk.isPressed(BTN_MODE1)) {
            _mode = 1;
            LOG("mode:%d\n", _mode);
        }

        if (_mode == 0) {
            int speedL = map(throttle, 1000, 2000, -255, 255);
            int speedR = map(pitch,    1000, 2000, -255, 255);
            _pActuator->setMotor(limitSpeed(speedL), limitSpeed(speedR));
        } else if (_mode == 1) {
            int angle = map(yaw,   1000, 2000,  -60,  60);
            int speed = map(pitch, 1000, 2000, -255, 255);
            _pActuator->drive(angle, limitSpeed(speed));
        }

        _btn_trk.end();
    }

    //
    // MSPCallback implementation
    //
    virtual int16_t onCommand(uint8_t cmd, uint8_t *pData, uint16_t size, uint8_t *pRes) {
        return 0;
    }

    //
    // StickCallback implementation
    //
    struct param_rc {
        int16_t  roll;
        int16_t  pitch;
        int16_t  yaw;
        int16_t  throttle;
        int16_t  aux[10];
        uint8_t  flag;
    } __attribute__((packed));

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

        rc.yaw = filterDeadZone(map(axisX, 0, 1024, 1000, 2000));
        rc.throttle = filterDeadZone(map(axisY, 0, 1024, 2000, 1000));
        rc.roll = filterDeadZone(map(axisZ, 0, 1024, 1000, 2000));
        rc.pitch = filterDeadZone(map(axisRZ, 0, 1024, 2000, 1000));
        stick(rc.yaw, rc.throttle, rc.pitch, rc.roll, btns);
    }
};


/*
*****************************************************************************************
* task_lidar
*****************************************************************************************
*/
void task_lidar(void* arg) {
    YDLidarX2   *pLidar = new YDLidarX2();
    YDLidarX2::scan_frame_t *pFrame;
    CommService *pSvc = (CommService*)arg;

    LOG("task lidar started\n");
    pLidar->setup();
    pLidar->enable(true);
    while (true) {
        lidar_state_t state = pLidar->process();
        switch (state) {
            case LIDAR_NO_PROCESS:
                vTaskDelay(pdMS_TO_TICKS(20));
                break;

            case LIDAR_SCAN_COMPLETED:
                pFrame = pLidar->getScans();
                if (pSvc && pFrame)
                    pSvc->send(CMD_LIDAR, (uint8_t*)pFrame, sizeof(YDLidarX2::scan_frame_t));
                break;
        }
    }
    vTaskDelete(NULL);
}


/*
*****************************************************************************************
* VARIABLES
*****************************************************************************************
*/
static ControlStick     _joy;
// static Timer<>          _timer = timer_create_default();
static TimeEvent        _evt(5);
static Vehicle*         _pCar = new Vehicle(0);


/*
*****************************************************************************************
* setup
*****************************************************************************************
*/
static bool blinkLed(void* param) {
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

    _pCar->setup();
    xTaskCreate(&task_lidar, "task_lidar", 2048, _pCar->getComm(), 2, NULL);

    // _timer.every(500, blinkLed);
    _joy.setStickCallback(_pCar);
    _joy.addSupportedDevices();
    _joy.begin();
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
    // _timer.tick();

    if (_evt.every(500)) {
        blinkLed(NULL);
    }

    _evt.tick();
    _pCar->loop();
}
