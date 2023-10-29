#ifndef _MSP_H_
#define _MSP_H_
#include "config.h"


/*
*****************************************************************************************
* MACROS
*****************************************************************************************
*/

/*
*****************************************************************************************
* Class
*****************************************************************************************
*/
class MSPCallback {
    public:
        virtual ~MSPCallback() { }
        virtual int16_t onCommand(uint8_t cmd, uint8_t *pData, uint16_t size, uint8_t *pRes) = 0;
};

class MSPInterface {
    public:
        virtual ~MSPInterface() { }
        virtual void write(uint8_t *pBuf, uint16_t size) = 0;
};

class MSP {
    public:
        typedef enum {
            REPLY_NO = 0,
            REPLY_OK,
            REPLY_NG,
        } reply_t;


        MSP(uint16_t wPacketSize);
        ~MSP();
        void setInterface(MSPInterface *pInterface);
        void setCallback(MSPCallback *pCallback);
        void processRx(uint8_t *pBuf, uint16_t size);
        void callback(uint8_t cmd, uint8_t *pData, uint16_t size, uint8_t *pRes);
        void send(uint8_t cmd, uint8_t *data, uint16_t size, reply_t reply = REPLY_NO, bool isSeqHeader = false);

    private:
        enum state_t {
            STATE_IDLE,
            STATE_HEADER_START,
            STATE_HEADER_M,
            STATE_HEADER_ARROW,
            STATE_HEADER_SIZE_1,
            STATE_HEADER_SIZE_2,
            STATE_HEADER_CMD
        };

        uint8_t           *m_pRxPackets;
        uint8_t           *m_pResultPackets;
        uint16_t          m_wPacketSize;
        uint8_t           m_ucSeq;
        uint8_t           m_ucSubSeq;

        enum state_t      m_stateRx;
        uint16_t          m_wOffset;
        uint16_t          m_wDataSize;
        uint8_t           m_ucCmd;
        MSPCallback  *m_pCallback;
        MSPInterface *m_pInterface;
};

#endif
