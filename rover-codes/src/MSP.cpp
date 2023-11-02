#include "MSP.h"

MSP::MSP(uint16_t wPacketSize) {
    m_stateRx    = STATE_IDLE;
    m_wOffset   = 0;
    m_wDataSize = 0;
    m_ucSeq      = 0;
    m_ucSubSeq   = 0;
    m_wPacketSize = wPacketSize;
    m_pRxPackets = new uint8_t[m_wPacketSize];
    m_pResultPackets = new uint8_t[m_wPacketSize];
}

MSP::~MSP() {
    delete m_pRxPackets;
    delete m_pResultPackets;
}

void MSP::setInterface(MSPInterface *pInterface) {
    m_pInterface = pInterface;
}

void MSP::setCallback(MSPCallback *pCallback) {
    m_pCallback  = pCallback;
}

void MSP::send(uint8_t cmd, uint8_t *data, uint16_t size, reply_t reply, bool isSeqHeader) {
    const uint8_t reply_tbl[3] = { '>', '<', '!'};
    uint8_t  header[8];
    uint8_t *buf = data;
    uint16_t szPacket;
    uint16_t szHeader;

    if (m_pInterface) {
        szPacket = isSeqHeader ? (m_wPacketSize - 2) : m_wPacketSize;
        szHeader = isSeqHeader ? 8 : 6;

        while (size >= 0) {
            uint16_t frag   = (size > szPacket) ? szPacket : size;
            uint16_t sz_ext = isSeqHeader ? frag + 2 : frag;

            header[0] = '$';
            header[1] = 'M';
            header[2] = reply_tbl[reply];
            header[3] = (sz_ext >> 8) & 0xff;
            header[4] = sz_ext & 0xff;
            header[5] = cmd;

            if (frag > 0)
                size -= frag;

            // 2bytes additional header = should be considered as data
            if (isSeqHeader) {
                header[6] = m_ucSeq;
                header[7] = (size == 0) ? (0x80 | m_ucSubSeq) : (m_ucSubSeq);
                m_ucSubSeq++;
            }
            m_pInterface->write(header, szHeader);

            if (frag > 0) {
                m_pInterface->write(buf, frag);
                buf  += frag;
            }
            if (size == 0)
                break;
        }

        if (isSeqHeader) {
            m_ucSeq++;
            m_ucSubSeq = 0;
        }
    }
}

void MSP::callback(uint8_t cmd, uint8_t *pData, uint16_t size, uint8_t *pRes) {
    if (m_pCallback) {
        int16_t ret = m_pCallback->onCommand(cmd, pData, size, pRes);
        if (ret >= 0) {
            send(cmd, pRes, ret, REPLY_OK);
        }
    }
}

void MSP::processRx(uint8_t *pBuf, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        uint8_t ch = pBuf[i];

        switch (m_stateRx) {
            case STATE_IDLE:
                if (ch == '$')
                    m_stateRx = STATE_HEADER_START;
                break;

            case STATE_HEADER_START:
                m_stateRx = (ch == 'M') ? STATE_HEADER_M : STATE_IDLE;
                break;

            case STATE_HEADER_M:
                m_stateRx = (ch == '>') ? STATE_HEADER_ARROW : STATE_IDLE;
                break;

            case STATE_HEADER_ARROW:
                m_wDataSize = (ch << 8);
                m_stateRx    = STATE_HEADER_SIZE_1;
                break;

            case STATE_HEADER_SIZE_1:
                m_wDataSize |= ch;
                if (m_wDataSize > m_wPacketSize) {
                    m_stateRx = STATE_IDLE;
                    continue;
                }
                m_wOffset = 0;
                m_stateRx  = STATE_HEADER_SIZE_2;
                break;

            case STATE_HEADER_SIZE_2:
                m_ucCmd    = ch;
                m_stateRx  = STATE_HEADER_CMD;
                if (m_wDataSize == 0) {
                    callback(m_ucCmd, m_pRxPackets, m_wDataSize, m_pResultPackets);
                    m_stateRx = STATE_IDLE;
                }
                break;

            case STATE_HEADER_CMD:
                if (m_wOffset < m_wDataSize) {
                    m_pRxPackets[m_wOffset++] = ch;
                }

                if (m_wOffset == m_wDataSize) {
                    callback(m_ucCmd, m_pRxPackets, m_wDataSize, m_pResultPackets);
                    m_stateRx = STATE_IDLE;
                }
                break;
        }
    }
}
