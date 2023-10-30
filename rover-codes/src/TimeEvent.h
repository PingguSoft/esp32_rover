#ifndef _TIME_EVENT_H_
#define _TIME_EVENT_H_

#include <Arduino.h>

#define LOG(...)    printf(__VA_ARGS__)

class TimeEvent {
private:
    uint8_t _evts;
    uint8_t _max_evts;

    typedef struct {
        uint16_t interval;
        unsigned long updated;
        unsigned long last;
    } evt_t;

    evt_t  *_pEvts;

public:
    TimeEvent(uint8_t max) {
        unsigned long ts = millis();

        _pEvts = new evt_t[max];
        for (uint8_t i = 0; i < max; i++) {
            _pEvts[i].last = ts;
        }
        _evts = 0;
        _max_evts = max;
    }

    bool every(int interval) {
        int8_t  idx = -1;
        bool    ret = false;

        for (int8_t i = 0; i < _evts; i++) {
            if (_pEvts[i].interval == interval) {
                idx = i;
                break;
            }
        }

        if (idx == -1) {
            if (_evts >= _max_evts) {
                LOG("increase max TimeEvent\n");
            } else {
                LOG("new interval : %d\n", interval);
                _pEvts[_evts++].interval = interval;
            }
        }

        unsigned long ts = millis();
        uint16_t diff = ts - _pEvts[idx].last;
        if (diff >= interval) {
            ret = true;
            _pEvts[idx].updated = ts;
        }
        return ret;
    }

    void tick() {
        for (int8_t i = 0; i < _evts; i++) {
            if (_pEvts[i].updated > 0) {
                _pEvts[i].last = _pEvts[i].updated;
                _pEvts[i].updated = 0;
            }
        }
    }
};
#endif
