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
    _rpm_cur   = 0;
    _rpm_tgt   = 300;
    _frame_in  = 0;
    _frame_out = 0;
    _frame_ctr = 0;

    _pid_rpm.configure(2.8f, 2.3f, 0.1f, 100);
    _pid_rpm.setOutputRange(0, 255);
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

uint16_t YDLidarX2::calcChecksum(uint8_t *data) {
    uint32_t chk32 = 0;
    uint16_t d;
    uint16_t checksum;

    // compute the checksum on 32 bits
    for (int i = 0; i < 10; i++) {
        d = data[2 * i] + (uint16_t(data[2 * i + 1]) << 8);
        chk32 = (chk32 << 1) + d;
    }
    checksum = (chk32 & 0x7FFF) + (chk32 >> 15);    // wrap around to fit into 15 bits
    checksum = checksum & 0x7FFF;                   // truncate to 15 bits

    return checksum;
}

int hex2string(uint8_t *in, int inlen, char *out) {
    int i = 0;
    char *pos = out;

    for (i = 0; i < inlen; i++)
        pos += sprintf(pos, "%02X ", in[i]);

    return pos - out + 1;
}

char szBuf[2048];

uint16_t calc_crc(uint8_t *data, uint16_t len) {
    uint16_t crc = 0x000;
    uint16_t length = len / 2;
    uint16_t *ptr = (uint16_t *)data;

    for(uint16_t i = 0; i < length; i++) {
        crc ^= *ptr++;
    }
    return crc;
}

int YDLidarX2::decodePacket(uint8_t *data, uint16_t len) {
    // hex2string(data, len, szBuf);
    // LOG("%s : %04x\n", szBuf, crc);

    uint16_t angle = 0;
    pkt_header_t *pkt = (pkt_header_t*)data;

    uint16_t *pSample = (uint16_t*)&data[10];
    float As = float(pkt->start >> 1) / 64.0;
    float Ae = float(pkt->end >> 1) / 64.0;
    float Ad = Ae - As;
    if (Ad < 0.0) {
        Ad = Ad + 360.0;
    } else if (Ad > 360) {
        Ad = Ad - 360.0;
    }
    float Aa = (pkt->samples > 1) ? (Ad / (pkt->samples - 1)) : 0.0;
    LOG("pwm:%3d, type:%02X, As:%6.2f, Ae:%6.2f, Ad:%6.2f, Aa:%6.2f, samples:%d\n", _pwm_duty, pkt->type, As, Ae, Ad, Aa, pkt->samples);
    // LOG("pwm:%3d, type:%02X, samples:%2d, ", _pwm_duty, pkt->type, pkt->samples);

    if (pkt->type & 0x01) {
        LOG("start ts:%8ld, scan_freq:%3d\n", millis(), pkt->type >> 1);
    } else {
        for (int i = 0; i < pkt->samples; i++, pSample++) {
            float Ac;
            float Ai =  As + (Aa * i);
            float Di = *pSample / 4.0;

            if (*pSample == 0) {
                Ac = 0;
            } else {
                // Ac = degrees(atan2(21.8 * (155.3 - Di), 155.3 * Di));
                Ac = degrees(atan(21.8*(155.3 - Di) / (155.3 * Di)));
            }

            Ai = Ai + Ac;
            if (Ai > 360.0) {
                Ai = Ai - 360.0;
            } else if (Ai < 0) {
                Ai = Ai + 360.0;
            }

            //
            // Ai, Di
            //
            // LOG("%6.2f, ", Ai);
        }
        // LOG("\n");
    }


    // uint16_t in_chksum   = (uint16_t(data[21]) << 8) | data[20];
    // uint16_t calc_chksum = calcChecksum(data);
    // uint16_t rpm;

    // // LOG("decode : %u / %u\n", calc_chksum, in_chksum);
    // if (in_chksum != calc_chksum)
    //     return -1;

    // uint16_t    angle = (data[1] - 0xA0) * 4;
    // uint8_t     idx = _frame_in % ARRAY_SIZE(_frames);

    // rpm = float(data[2] | (data[3] << 8)) / 64.0;
    // _rpm_cur = (0 <= rpm && rpm <= 800) ? rpm : 0;

    // // 1st scan?
    // if (0 <= angle && angle <= 3) {
    //     _frames[idx].ts  = millis();
    //     _frames[idx].rpm = _rpm_cur;
    // }
    // for (int i = 0; i < 4; i++) {
    //     uint16_t    theta = angle + i;
    //     uint16_t    dist_mm = data[(i*4)+4] | ((data[(i*4)+5] & 0x1f) << 8);
    //     uint16_t    quality = data[(i*4)+6] | (data[(i*4)+7] << 8);
    //     uint8_t     bad = data[(i*4)+5] & 0x80;
    //     uint8_t     bad2 = data[(i*4)+5] & 0x40;

    //     if (bad == 0 && bad2 == 0) {
    //         _frames[idx].scans[theta].dist = dist_mm;
    //         _frames[idx].scans[theta].quality = quality;
    //     }
    // }
    // if (angle >= 356) {   // last angle
    //     _frame_in++;
    //     _frame_ctr++;
    // }
    return angle;
}

bool YDLidarX2::getOutput(struct _scan_frame *frame) {
    if (_frame_ctr == 0)
        return false;

    // uint8_t avail = (_frame_in > _frame_out) ? (_frame_in - _frame_out) :
    //                 (ARRAY_SIZE(_frames) - (_frame_out - _frame_in));

    int8_t idx = _frame_out % ARRAY_SIZE(_frames);
    memcpy(frame, &_frames[idx], sizeof(struct _scan_frame));
    _frame_out++;
    _frame_ctr--;
    return true;
}

lidar_state_t YDLidarX2::process() {
    static long last_ts = 0;
    long    ts = millis();
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
                    memcpy((void*)&_pkt_hdr, _pkt_buf, sizeof(_pkt_hdr));
                    _pkt_state = PKT_DATA;
                }
                break;

            case PKT_DATA:
                if (_pkt_pos < 10 + (_pkt_hdr.samples * 2)) {
                    _pkt_buf[_pkt_pos++] = b;
                }
                if (_pkt_pos == 10 + (_pkt_hdr.samples * 2)) {
                    // completed ?
                    if (calc_crc(_pkt_buf, _pkt_pos) == 0)
                        decodePacket(_pkt_buf, _pkt_pos);
                    _pkt_state = PKT_IDLE;
                }
                break;
        }
    }

    // if (_enabled && IS_ELAPSED(ts, last_ts, 10)) {
    //     _pwm_duty = _pid_rpm.step(_rpm_tgt, _rpm_cur);
    //     analogWrite(PIN_LIDAR_PWM, _pwm_duty);
    //     // LOG("rpm:%5d, pwm:%4d\n", _rpm_cur, _pwm_duty);
    //     last_ts = ts;
    // }

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
