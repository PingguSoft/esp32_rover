#include "utils.h"

void Utils::dump(String name, uint8_t* data, uint16_t cnt) {
    uint8_t  i;
    uint8_t  b;
    uint16_t addr = 0;

    printf(PSTR("-- %s buf size : %d -- \n"), name.c_str(), cnt);
    while (cnt) {
        printf(PSTR("%08x - "), addr);

        for (i = 0; (i < 16) && (i < cnt); i++) {
            b = *(data + i);
            printf(PSTR("%02x "), b);
        }

        printf(PSTR(" : "));
        for (i = 0; (i < 16) && (i < cnt); i++) {
            b = *(data + i);
            if ((b > 0x1f) && (b < 0x7f))
                printf(PSTR("%c"), b);
            else
                printf(PSTR("."));
        }
        printf(PSTR("\n"));
        data += i;
        addr += i;
        cnt -= i;
    }
}

void Utils::log(...) {
}

float Utils::mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    const float dividend = out_max - out_min;
    const float divisor = in_max - in_min;
    const float delta = x - in_min;

    return (delta * dividend) / divisor + out_min;
}


int Utils::hex2string(uint8_t *in, int inlen, char *out) {
    int i = 0;
    char *pos = out;

    for (i = 0; i < inlen; i++)
        pos += sprintf(pos, "%02X ", in[i]);

    return pos - out + 1;
}
