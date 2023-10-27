#include <ESP32Servo.h>
#include "utils.h"
#include "ydlidar_x2.h"

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
YDLidarX2::YDLidarX2() {
    _enabled   = true;
    _pwm_duty  = 255;
    _pkt_state = PKT_IDLE;
    _frame_in  = 0;
    _frame_out = 0;
    _frame_ctr = 0;
}

void YDLidarX2::setup() {
    Serial2.begin(115200, SERIAL_8N1, PIN_LIDAR_RX, -1);
    pinMode(PIN_LIDAR_PWM, OUTPUT);
    _pPwm = new ESP32PWM();
    _pPwm->attachPin(PIN_LIDAR_PWM, 10000, 8);
    _pPwm->write(_pwm_duty);
    LOG("Lidar PWM Ch:%d\n", _pPwm->getChannel());
}

void YDLidarX2::enable(bool en) {
    if (en) {
        _pPwm->write(_pwm_duty);
    } else {
        _pPwm->write(0);
    }
    _enabled = en;
}

uint16_t YDLidarX2::calcCheksum(uint8_t *data, uint16_t len) {
    uint16_t crc = 0x000;
    uint16_t length = len / 2;
    uint16_t *ptr = (uint16_t *)data;

    for(uint16_t i = 0; i < length; i++) {
        crc ^= *ptr++;
    }
    return crc;
}

int YDLidarX2::decodePacket(uint8_t *data, uint16_t len) {
    int          ret  = 0;
    pkt_header_t *pkt = (pkt_header_t*)data;
    uint16_t     *pSample = (uint16_t*)&data[10];

    float As = float(pkt->start >> 1) / 64.0;
    float Ae = float(pkt->end >> 1) / 64.0;
    float Ad = Ae - As;
    if (Ad < 0.0) {
        Ad = Ad + 360.0;
    } else if (Ad > 360) {
        Ad = Ad - 360.0;
    }
    float Ads = (pkt->samples > 1) ? (Ad / (pkt->samples - 1)) : 0.0;
    LOG("type:%02X, As:%6.2f, Ae:%6.2f, Ad:%6.2f, Ads:%6.2f, samples:%d\n", pkt->type, As, Ae, Ad, Ads, pkt->samples);

    uint8_t  idx = _frame_in % ARRAY_SIZE(_frames);
    if (pkt->type & 0x01) {
        // complete previous scans
        if (_frames[idx].ts != 0) {
            LOG("complete index:%2d\n", idx);
            _frame_in++;
            _frame_ctr++;
            ret = 1;
        }

        // new scans
        idx = _frame_in % ARRAY_SIZE(_frames);
        _frames[idx].ts  = millis();
        LOG("start    index:%2d\n", idx);
    } else {
        for (int i = 0; i < pkt->samples; i++, pSample++) {
            float Ac;
            float Ai =  As + (Ads * i);
            float Di = *pSample / 4.0;

            Ac = (*pSample == 0) ? 0 : degrees(atan(21.8*(155.3 - Di) / (155.3 * Di)));
            // Ac = (*pSample == 0) ? 0 : degrees(atan2(21.8 * (155.3 - Di), 155.3 * Di));
            Ai = Ai + Ac;
            if (Ai > 360.0) {
                Ai = Ai - 360.0;
            } else if (Ai < 0) {
                Ai = Ai + 360.0;
            }

            int pos = roundf(Ai / Ads);
            if (0 <= pos && pos < ARRAY_SIZE(_frames[idx].scans))
                _frames[idx].scans[pos] = Di;
        }
    }

    return ret;
}

bool YDLidarX2::getOutput(struct _scan_frame *frame) {
    if (_frame_ctr == 0)
        return false;

    int8_t idx = _frame_out % ARRAY_SIZE(_frames);
    memcpy(frame, &_frames[idx], sizeof(struct _scan_frame));
    _frame_out++;
    _frame_ctr--;
    return true;
}

lidar_state_t YDLidarX2::process() {
    lidar_state_t ret = LIDAR_NO_PROCESS;
    int     b = (Serial2.available() > 0) ? Serial2.read() : -1;

    if (_enabled && b >= 0) {
        ret = LIDAR_IN_PROCESS;
        switch (_pkt_state) {
            case PKT_IDLE:
                if (b == 0xAA) {
                    _pkt_pos =  0;
                    _pkt_buf[_pkt_pos++] = b;
                    _pkt_state = PKT_SYNC;
                }
                break;

            case PKT_SYNC:
                if (b == 0x55) {
                    _pkt_buf[_pkt_pos++] = b;
                    _pkt_state = PKT_HEADER;
                } else {
                    _pkt_state = PKT_IDLE;
                }
                break;

            case PKT_HEADER:
                if (_pkt_pos < 10) {
                    _pkt_buf[_pkt_pos++] = b;
                }
                if (_pkt_pos == 10) {
                    pkt_header_t *pkt = (pkt_header_t*)_pkt_buf;
                    _pkt_dlen  = pkt->samples * 2;
                    _pkt_state = PKT_DATA;
                }
                break;

            case PKT_DATA:
                if (_pkt_pos < 10 + _pkt_dlen) {
                    _pkt_buf[_pkt_pos++] = b;
                }

                // last data ?
                if (_pkt_pos == 10 + _pkt_dlen) {
                    if (calcCheksum(_pkt_buf, _pkt_pos) == 0) {
                         ret = decodePacket(_pkt_buf, _pkt_pos) ? LIDAR_SCAN_COMPLETED : LIDAR_PACKET_DECODED;
                    }
                    _pkt_state = PKT_IDLE;
                }
                break;
        }
    }

    if (Serial.available()) {
        int ch = Serial.read();
        switch (ch) {
            case '.':
                _pwm_duty++;
                _pPwm->write(_pwm_duty);
                // LOG("\nPWM:%3d\n", _pwm_duty);
                break;

            case ',':
                _pwm_duty--;
                _pPwm->write(_pwm_duty);
                // LOG("\nPWM:%3d\n", _pwm_duty);
                break;
        }
    }

    return ret;
}
