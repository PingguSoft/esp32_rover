#ifndef _UTILS_H_
#define _UTILS_H_

#include "config.h"

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
#define ARRAY_SIZE(x)                   (sizeof(x) / sizeof((x)[0]))
#define IS_ELAPSED(ts, last, duration)  ((ts) - (last) > (duration))

#define LOG_FMT(letter, format) "[%6u][" #letter "][%16s:%6u] %16s(): " format, (unsigned long) (esp_timer_get_time() / 1000ULL), pathToFileName(__FILE__), __LINE__, __FUNCTION__
#define LOG(format, ...)         printf(LOG_FMT(V, format), ##__VA_ARGS__)
#define LOGV(format, ...)        printf(LOG_FMT(V, format), ##__VA_ARGS__)
#define LOGI(format, ...)        printf(LOG_FMT(I, format), ##__VA_ARGS__)
#define LOGW(format, ...)        printf(LOG_FMT(W, format), ##__VA_ARGS__)
#define LOGE(format, ...)        printf(LOG_FMT(E, format), ##__VA_ARGS__)
#define DUMP(name, data, cnt)   Utils::dump(name, data, cnt)

class Utils {
public:
    static void  dump(String name, uint8_t* data, uint16_t cnt);
    static void  log(...);
    static float mapf(float x, float in_min, float in_max, float out_min, float out_max);
    static int   hex2string(uint8_t *in, int inlen, char *out);
};

#endif
