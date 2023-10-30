#ifndef _YDLIDAR_X2_H_
#define _YDLIDAR_X2_H_
#include <ESP32Servo.h>
#include "config.h"

/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/
#define MUTEX_ENABLE    1

typedef enum {
    LIDAR_NO_PROCESS = -1,
    LIDAR_IN_PROCESS,
    LIDAR_SCAN_COMPLETED,
} lidar_state_t;

/*
*****************************************************************************************
* MACROS & STRUCTURES
*****************************************************************************************
*/
#if MUTEX_ENABLE == 1
#define MUTEX_LOCK()    do {} while (xSemaphoreTake(_lock, portMAX_DELAY) != pdPASS)
#define MUTEX_UNLOCK()  xSemaphoreGive(_lock)
#else
#define MUTEX_LOCK()
#define MUTEX_UNLOCK()
#endif

/*
*****************************************************************************************
* CLASS
*****************************************************************************************
*/
class YDLidarX2 {
public:
    typedef struct {
        unsigned long   ts;
        uint16_t        scan_num;
        uint16_t        scans[600];
    } __attribute__((packed)) scan_frame_t;

    typedef struct {
        uint16_t    sync;
        uint8_t     type;
        uint8_t     samples;
        uint16_t    start;
        uint16_t    end;
        uint16_t    check;
    } __attribute__((packed)) pkt_header_t;

    YDLidarX2();
    void setup();
    void enable(bool en);
    lidar_state_t process();
    scan_frame_t *getScans();
    bool     isEnabled()    { return _enabled; }
    uint8_t *getRawBuf()    { return _pkt_buf; }
    uint8_t  getPwm()       { return _pwm_duty; }

private:
    bool     decodePacket(uint8_t *buf, uint16_t len);
    uint16_t calcCheksum(uint8_t *data, uint16_t len);

    typedef enum {
        PKT_IDLE = 0,
        PKT_SYNC = 1,
        PKT_HEADER = 2,
        PKT_DATA = 3,
    } pkt_state_t;

    bool                _enabled;
    uint8_t             _pkt_pos;
    uint8_t             _pkt_len;
    uint8_t             _pkt_buf[120];
    pkt_state_t         _pkt_state;
    uint8_t             _pkt_dlen;

    scan_frame_t        _frames[6];
    uint8_t             _frame_in;
    uint8_t             _frame_out;
    uint8_t             _frame_ctr;

    uint8_t             _pwm_duty;
    ESP32PWM            *_pPwm;

#if MUTEX_ENABLE == 1
    SemaphoreHandle_t   _lock;
#endif
};

#endif
