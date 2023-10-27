#ifndef _YDLIDAR_X2_H_
#define _YDLIDAR_X2_H_
#include <ESP32Servo.h>
#include "config.h"

/*
*****************************************************************************************
* CONSTANTS
*****************************************************************************************
*/
typedef enum {
    LIDAR_NO_PROCESS = -1,
    LIDAR_IN_PROCESS,
    LIDAR_PACKET_DECODED,
    LIDAR_SCAN_COMPLETED,
} lidar_state_t;

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
class YDLidarX2 {
public:
    struct _scan_frame {
        unsigned long   ts;
        uint16_t        scans[720];
    };

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
    lidar_state_t process();
    bool getOutput(struct _scan_frame *frame);
    void enable(bool en);
    bool isEnabled()        { return _enabled; }
    uint8_t *getRawBuf()    { return _pkt_buf; }
    uint8_t  getPwm()       { return _pwm_duty; }

private:
    int      decodePacket(uint8_t *buf, uint16_t len);
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

    struct _scan_frame  _frames[5];
    uint8_t             _frame_in;
    uint8_t             _frame_out;
    uint8_t             _frame_ctr;

    uint8_t             _pwm_duty;
    ESP32PWM            *_pPwm;
};

#endif
