#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "utils.h"
#include "wifi_service.h"

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
void task_comm(void* arg);

WiFiService::WiFiService() {
    _isServerStated = false;
    _isClientConnected = false;
}

//
// MSPInterface implementation
//
void WiFiService::write(uint8_t *pBuf, uint16_t size) {
    if (_isClientConnected) {
        uint16_t frag;
        uint8_t  *data = pBuf;

        while (size > 0) {
            frag = (size > 1500) ? 1500 : size;
            _client.write(data, frag);
            data += frag;
            size -= frag;
        }
    }
}

void WiFiService::setup(char *ssid, char *password, int port, MSPCallback *cb) {
    _pServer = new WiFiServer(port);
    _pServer->setNoDelay(true);
    _pMSP = new MSP(1300);
    _pMSP->setInterface(this);
    _pMSP->setCallback(cb);
    strncpy(_ssid, ssid, sizeof(_ssid));
    strncpy(_password, password, sizeof(_password));
    _queue_comm  = xQueueCreate(20, sizeof(comm_q_t));
    xTaskCreate(&task_comm, "task_comm", 2560, this, 8, NULL);
}

bool WiFiService::isConnected() {
    return (WiFi.status() == WL_CONNECTED);
}

void WiFiService::process() {
    if (!_isServerStated && isConnected()) {
        LOG("connected to AP : %s\n", WiFi.localIP().toString().c_str());
        _pServer->begin();
        _isServerStated = true;
    }

    if (_isClientConnected) {
        if (_client.connected()) {
            int sz = _client.available();
            if (sz > 0) {
                _client.read(_buf, sz);
                _pMSP->processRx(_buf, sz);
                // LOG("RX:%s\n", _buf);
            }
        } else {
            LOG("Client is gone\n");
            _client.stop();
            _isClientConnected = false;
        }
    } else {
        _client = _pServer->available();
        if (_client) {
            if (_client.connected()) {
                LOG("Client is connected from %s\n", _client.localIP().toString().c_str());
                _isClientConnected = true;
            }
        }
    }
}

void WiFiService::send(cmd_t cmd, uint8_t *buf, uint16_t size, bool reqBufDel, bool reqSeqHeader) {
    if (_isClientConnected) {
        comm_q_t q = { cmd, buf, size, reqBufDel, reqSeqHeader };
        xQueueSend(_queue_comm, &q, portMAX_DELAY);
    }
}

void WiFiService::task_comm(void* arg) {
    WiFiService *pSvc = (WiFiService*)arg;

    LOG("task_comm started\n");
    WiFi.mode(WIFI_STA);
    WiFi.begin(pSvc->_ssid, pSvc->_password);

    comm_q_t *q = new comm_q_t;
    while (true) {
        pSvc->process();
        if (xQueueReceive(pSvc->_queue_comm, q, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (pSvc->_isClientConnected) {
                pSvc->_pMSP->send(q->cmd, q->pData, q->size);
                if (q->reqBufDel)
                    delete q->pData;
            }
        }
    }
    delete q;
    vTaskDelete(NULL);
}
