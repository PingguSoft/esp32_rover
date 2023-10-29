#ifndef _WIFI_SERVICE_H_
#define _WIFI_SERVICE_H_
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include "config.h"
#include "actuator.h"
#include "MSP.h"

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
typedef enum {
    CMD_LIDAR      = 1,
    CMD_ODOMETRY   = 2,
    CMD_POSE       = 3,
} cmd_t;

typedef struct {
    cmd_t       cmd;
    uint8_t     *pData;
    uint16_t    size;
    bool        reqBufDel;
    bool        reqSeqHeader;
} comm_q_t;

class CommService {
    public:
        virtual void setup(char *ssid, char *password, int port, MSPCallback *cb = NULL);
        virtual void send(cmd_t cmd, uint8_t *buf, uint16_t size, bool reqDataDel=false, bool reqSeqHeader=false);
};

/*
*****************************************************************************************
* CLASS
*****************************************************************************************
*/
class WiFiService : public MSPInterface, public CommService {
public:
    WiFiService();
    void setup(char *ssid, char *password, int port, MSPCallback *cb = NULL);
    bool isConnected();
    QueueHandle_t getQueue()   { return _queue_comm; }

    void send(cmd_t cmd, uint8_t *buf, uint16_t size, bool reqDataDel=false, bool reqSeqHeader=false);

    // MSPInterface implementation
    virtual void write(uint8_t *pBuf, uint16_t size);

    static void task_comm(void* arg);
private:
    void process();

    WiFiServer  *_pServer;
    bool        _isServerStated;
    bool        _isClientConnected;
    WiFiClient  _client;
    uint8_t     _buf[2048];
    char        _ssid[32];
    char        _password[64];
    QueueHandle_t _queue_comm;
    MSP         *_pMSP;
};

#endif
