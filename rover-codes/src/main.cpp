#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>
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

typedef enum {
    CMD_LIDAR      = 0x01,
    CMD_TICKS      = 0x02,
    CMD_ODOMETRY   = 0x03,
    CMD_BUTTON     = 0x04,

    CMD_RESET      = 0x10,
    CMD_SPEED      = 0x11,
} cmd_t;

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
#define BTN_RECORD          _BV(ControlStick::BTN_START)

class Vehicle : public MSPCallback, public StickCallback {
private:
#if (DEBUG_FORWARD == FORWARD_SERIAL)
    SerialService   _service;
#elif (DEBUG_FORWARD == FORWARD_WIFI)
    WiFiService     _service;
#endif
    Actuator       *_pActuator;
    uint8_t         _mode;
    ButtonTracker   _btn_trk;
    TimeEvent       _evt;
    uint8_t         _max_pwm;
    int8_t          _angle;

public:
    Vehicle(int option) : _btn_trk(BTN_SHIFT_L | BTN_SHIFT_R), _evt(5)
    {
        _mode      = 0;
        _angle     = 0;
        _pActuator = new Actuator(
                        new WheelDriver(PIN_L_DRV_IN1, PIN_L_DRV_IN2, PIN_L_CTR, PIN_NONE, false, WHEEL_RADIUS_MM, TICKS_PER_CYCLE),
                        new WheelDriver(PIN_R_DRV_IN1, PIN_R_DRV_IN2, PIN_R_CTR, PIN_NONE, false, WHEEL_RADIUS_MM, TICKS_PER_CYCLE),
                        AXLE_WIDTH_MM);
    }

    void setup() {
        pinMode(PIN_DRV_EN, OUTPUT);
        digitalWrite(PIN_DRV_EN, HIGH);
#ifdef MOTOR_PWM_LIMIT
        _max_pwm = MOTOR_PWM_LIMIT;
#else
        _max_pwm = 255;
#endif
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

    int8_t *getAnglePtr() {
        return &_angle;
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

        if (_evt.every(100)) {
            static Odometry::odometry_t odo;
            static Ticks::ticks_t   ticks;

            if (_pActuator->updateOdometry()) {
                odo = _pActuator->getOdometry()->get();
                // LOGI("odo: x:%5d, y:%5d, theta:%5.1f\n", odo.x, odo.y, degrees(odo.theta));
                _service.send(CMD_ODOMETRY, (uint8_t*)&odo, sizeof(odo));

                // ticks = _pActuator->getTicks().get();
                // _service.send(CMD_TICKS, (uint8_t*)&ticks, sizeof(ticks));
            }
        }

        _evt.tick();
    }

    int16_t limitSpeed(int speed) {
        uint8_t  spd = abs(speed);
        uint8_t  idx = spd / 10;
        uint8_t  rem = spd % 10;
        float step  = (_tblSpeed[idx + 1] - _tblSpeed[idx]) / 10;
        uint8_t out = _tblSpeed[idx] + uint8_t(rem * step);

        out = map(out, 0, 255, 0, _max_pwm);
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
        if (_btn_trk.isToggled()) {
            int *pData = new int[1];
            pData[0] = btn;
            getComm()->send(CMD_BUTTON, (uint8_t*)pData, sizeof(int), true);
        }

        if (_mode == 0) {
            int speedL = map(throttle, 1000, 2000, -255, 255);
            int speedR = map(pitch,    1000, 2000, -255, 255);
            _pActuator->setMotor(limitSpeed(speedL), limitSpeed(speedR));
        } else if (_mode == 1) {
            _angle = map(roll,     1000, 2000,  -60,  60);
            int speed = map(throttle, 1000, 2000, -255, 255);
            _pActuator->drive(_angle, limitSpeed(speed));
        }

        _btn_trk.end();
    }

    //
    // MSPCallback implementation
    //
    virtual int16_t onCommand(uint8_t cmd, uint8_t *pData, uint16_t size, uint8_t *pRes) {
        switch (cmd) {
            case CMD_RESET:
                _pActuator->reset();
                LOGI("reset !!\n");
                break;

            case CMD_SPEED:
                _max_pwm = *pData;
                LOGI("max speed : %3d\n", _max_pwm);
                break;
        }
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
typedef struct {
    CommService *comm;
    int8_t      *aux;
} task_param_t;

void task_lidar(void* arg) {
    YDLidarX2   *pLidar = new YDLidarX2();
    YDLidarX2::scan_frame_t *pFrame;
    task_param_t *param = (task_param_t*)arg;
    CommService *pSvc = param->comm;

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
                if (pSvc && pFrame) {
                    pFrame->aux = *(param->aux);
                    // LOG("> scans:%3d\n", pFrame->scan_num);
                    pSvc->send(CMD_LIDAR, (uint8_t*)pFrame, sizeof(YDLidarX2::scan_frame_t));
                }
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
static TimeEvent        _evt(5);
static Vehicle*         _pCar = new Vehicle(0);


/*
*****************************************************************************************
* setup
*****************************************************************************************
*/
void blinkLed() {
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
}

static task_param_t _param;

void setup() {
    // setCpuFrequencyMhz(80);
    Serial.begin(230400);
    Serial.setDebugOutput(true);
    esp_log_level_set("*", ESP_LOG_WARN);

    LOG("setup start !!! : heap:%d, psram:%d\n", ESP.getFreeHeap(), ESP.getPsramSize());
    pinMode(PIN_LED, OUTPUT);

    _pCar->setup();
    _param = {_pCar->getComm(), _pCar->getAnglePtr()};
    xTaskCreate(&task_lidar, "task_lidar", 2048, &_param, 2, NULL);

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

    if (_evt.every(500)) {
        blinkLed();
    }

    _evt.tick();
    _pCar->loop();
}
