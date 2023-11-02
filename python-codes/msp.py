class MSP:
    STATE_IDLE = 0
    STATE_HEADER_START = 1
    STATE_HEADER_M = 2
    STATE_HEADER_ARROW = 3
    STATE_HEADER_SIZE_1 = 4
    STATE_HEADER_SIZE_2 = 5
    STATE_HEADER_CMD = 6

    REPLY_NO = 0
    REPLY_OK = 1
    REPLY_NG = 2

    def __init__(self, write, callback):
        self._packet_size = 2048
        self._state = MSP.STATE_IDLE
        self._cmd = 0
        self._size = 0
        self._packet = bytearray(self._packet_size)
        self._pos = 0

        self._seq = 0
        self._sub_seq = 0
        self._write = write
        self._cb = callback

    def send(self, cmd, data, size, reply=REPLY_NO, isSeqHeader=False):
        reply_tbl = ['>', '<', '!']
        szPacket = 0
        szHeader = 0
        pos = 0

        szPacket = (self._packet_size - 2) if isSeqHeader else self._packet_size
        szHeader = 8 if isSeqHeader else 6
        header = []

        while size >= 0:
            frag = szPacket if size > szPacket else size
            sz_ext = (frag + 2) if isSeqHeader else frag
            header.append(ord('$'))
            header.append(ord('M'))
            header.append(ord(reply_tbl[reply]))
            header.append((sz_ext >> 8) & 0xff)
            header.append(sz_ext & 0xff)
            header.append(cmd)

            if frag > 0:
                size -= frag

            if isSeqHeader:
                header.append(self._seq)
                ss = (0x80 | self._sub_seq) if size == 0 else self._sub_seq
                header.append(ss)
                self._sub_seq += 1
            self._write(bytes(header))

            if frag > 0:
                buf = data[pos:pos + frag]
                self._write(buf)
                pos += frag

            if size == 0:
                break

        if isSeqHeader:
            self._seq += 1
            self._sub_seq = 0

    def callback(self, cmd, data):
        if self._cb is not None:
            resp = self._cb(cmd, data)
            if resp is not None:
                self.send(cmd, resp, len(resp), MSP.REPLY_OK)

    def processRx(self, buf):
        for ch in buf:
            # print(
            #     f'{ch:02X} {self._state:1d} {self._pos:3d} / {self._size:3d}')
            if self._state == MSP.STATE_IDLE:
                if ch == ord('$'):
                    self._state = MSP.STATE_HEADER_START

            elif self._state == MSP.STATE_HEADER_START:
                self._state = MSP.STATE_HEADER_M if ch == ord(
                    'M') else MSP.STATE_IDLE

            elif self._state == MSP.STATE_HEADER_M:
                self._state = MSP.STATE_HEADER_ARROW if ch == ord(
                    '>') else MSP.STATE_IDLE

            elif self._state == MSP.STATE_HEADER_ARROW:
                self._size = ch << 8
                self._state = MSP.STATE_HEADER_SIZE_1

            elif self._state == MSP.STATE_HEADER_SIZE_1:
                self._size |= ch
                if self._size > self._packet_size:
                    self._state = MSP.STATE_IDLE
                    continue
                self._pos = 0
                self._state = MSP.STATE_HEADER_SIZE_2

            elif self._state == MSP.STATE_HEADER_SIZE_2:
                self._cmd = ch
                self._state = MSP.STATE_HEADER_CMD
                if self._size == 0:
                    self.callback(self._cmd, self._packet)
                    self._state = MSP.STATE_IDLE

            elif self._state == MSP.STATE_HEADER_CMD:
                if self._pos < self._size:
                    self._packet[self._pos] = ch
                    self._pos = self._pos + 1
                    # self._packet.append(ch)
                if self._pos == self._size:
                    self.callback(self._cmd, self._packet)
                    self._state = MSP.STATE_IDLE
