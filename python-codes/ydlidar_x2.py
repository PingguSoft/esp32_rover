import pygame
import serial
import time
import threading
import struct
import math

"""
LiadrX2 class
"""
class LidarX2(object):
    def __init__(self, port, sceen=None):
        self._port = port
        self._serial = None
        self._thread = None
        self._sig_kill = threading.Event()
        self._protocol = self.Protocol()
        self._screen = screen

    def open(self):
        self._thread = threading.Thread(target=self.thread_serial_reader, args=(self._sig_kill,))
        self._thread.start()

    def thread_serial_reader(self, sig_kill):
        print('thread_reader started')
        self._serial = serial.Serial(self._port, 115200)

        while not sig_kill.is_set():
            try:
                avail = self._serial.in_waiting
                if avail > 0:
                    buf = self._serial.read(avail)
                    state = self._protocol.process(buf)
                    if state == self.Protocol.LIDAR_SCAN_COMPLETED and self._screen != None:
                        frame = self._protocol.get_scans()
                        if frame is not None:
                            self.draw(self._screen, frame)
                else:
                    time.sleep(0.005)
            except OSError as msg:
                print(msg)

        print('thread_reader terminated')
        self._serial.close()

    def close(self):
        self._sig_kill.set()
        self._thread.join()

    def draw(self, screen, frame):
        screen.fill((250, 250, 250))
        xpos = 0
        ypos = 0
        ratio = 6
        delta = 0.6
        scan_num2 = int(360 / delta)
        for x in range(scan_num2):
            theta = delta * x
            dist = frame['scans'][x]
            dist /= ratio

            c = (0, 0, 0)
            r = 1
            x = -dist * math.cos(math.radians(theta))
            y = -dist * math.sin(math.radians(theta))
            pygame.draw.circle(screen, c, (xpos + 400 + x, ypos + 400 + y), r)
        pygame.draw.circle(screen, (252, 132, 3), (xpos + 400, ypos + 400), 12)
        pygame.display.flip()

    """
    inner Protocol class for decoding packets
    """
    class Protocol(object):
        PKT_IDLE = 0
        PKT_SYNC = 1
        PKT_HEADER = 2
        PKT_DATA = 3

        LIDAR_IN_PROCESS = 0
        LIDAR_PACKET_DECODED = 1
        LIDAR_SCAN_COMPLETED = 2

        def __init__(self):
            self._pkt_pos = 0
            self._pkt_len = 0
            self._pkt_buf = bytearray(120)
            self._pkt_state = self.PKT_IDLE
            self._pkt_dlen = 0

            self._max_scans = 600
            self._frame_max = 10
            self._frames = []
            self._frame_in = 0
            self._frame_out = 0
            self._frame_ctr = 0
            for i in range(self._frame_max):
                self._frames.append({'ts': 0, 'scan_num': 0, 'scans': [0] * self._max_scans, 'scans_raw': []})

        def calc_checksum(self, buf, len):
            crc = 0
            for i in range(len//2):
                d = struct.unpack('<H', buf[i*2:i*2+2])
                crc = crc ^ d[0]
            return crc

        def process(self, buf):
            ret = self.LIDAR_IN_PROCESS
            for b in buf:
                if self._pkt_state == self.PKT_IDLE:
                    if b == 0xAA:
                        self._pkt_pos = 0
                        self._pkt_buf[self._pkt_pos] = b
                        self._pkt_pos += 1
                        self._pkt_state = self.PKT_SYNC

                elif self._pkt_state == self.PKT_SYNC:
                    if b == 0x55:
                        self._pkt_buf[self._pkt_pos] = b
                        self._pkt_pos += 1
                        self._pkt_state = self.PKT_HEADER
                    else:
                        self._pkt_state = self.PKT_IDLE

                elif self._pkt_state == self.PKT_HEADER:
                    if self._pkt_pos < 10:
                        self._pkt_buf[self._pkt_pos] = b
                        self._pkt_pos += 1
                    if self._pkt_pos == 10:
                        _, _, self._dlen = struct.unpack('<HBB', self._pkt_buf[0:4])
                        self._dlen *= 2
                        self._pkt_state = self.PKT_DATA

                elif self._pkt_state == self.PKT_DATA:
                    if self._pkt_pos < (10 + self._dlen):
                        self._pkt_buf[self._pkt_pos] = b
                        self._pkt_pos += 1
                    if self._pkt_pos == (10 + self._dlen):
                        if self.calc_checksum(self._pkt_buf, self._pkt_pos) == 0:
                            state = self.decode_packet(self._pkt_buf, self._pkt_pos)
                            ret = self.LIDAR_SCAN_COMPLETED if state else self.LIDAR_PACKET_DECODED
                        self._pkt_state = self.PKT_IDLE
            return ret

        def decode_packet(self, buf, len):
            ret = False
            kMult = 64
            kMult360 = int(360 * kMult)

            _, type, samples, start, end = struct.unpack('<HBBHH', buf[0:8])
            As = (start >> 1)
            Ae = (end >> 1)
            Ad = Ae - As
            if Ad < 0.0:
                Ad += kMult360
            elif Ad > kMult360:
                Ad -= kMult360
            Ads = (Ad // (samples - 1)) if samples > 1 else 1
            if Ads == 0:
                return ret

            idx = self._frame_in % self._frame_max
            if (type & 0x01) == 0x01:
                if self._frames[idx]['ts'] != 0:
                    print(
                        f"complete index:{idx:2d}, ts:{self._frames[idx]['ts']}, num:{self._frames[idx]['scan_num']:3d}")
                    print('\n')
                    self._frame_in += 1
                    self._frame_ctr += 1
                    ret = True
                # new scan
                idx = self._frame_in % self._frame_max
                self._frames[idx]['ts'] = int(time.time_ns() / 1000000)
                self._frames[idx]['scan_num'] = int(0)
                for i in range(self._max_scans):
                    self._frames[idx]['scans'][i] = int(0)
                self._frames[idx]['scans_raw'] = []
                print('-' * 100)
                print(f"start    index:{idx:2d}, ts:{self._frames[idx]['ts']}")

            # print(f'{type=:2x}, {As=:6.2f}, {Ae=:6.2f}, {Ad=:6.2f}, {Ads=:6.2f}, {samples=:2d}')
            for i in range(samples):
                Ai = As + (Ads * i)
                sample = struct.unpack('<H', buf[10+i*2:12+i*2])
                Di = sample[0] // 4.0
                # Ac = 0 if Di == 0 else math.degrees(math.atan(21.8*(155.3 - Di) / (155.3 * Di)))
                Ac = 0 if Di == 0 else math.degrees(math.atan2(21.8 * (155.3 - Di), 155.3 * Di))
                Ai = (Ai / kMult) + Ac
                if Ai > 360.0:
                    Ai -= 360.0
                elif Ai < 0:
                    Ai += 360.0

                pos = math.floor((Ai + 0.3) / 0.6)
                if 0 <= pos and pos < self._max_scans:
                    self._frames[idx]['scans'][pos] = Di
                self._frames[idx]['scans_raw'].append((Ai, Di))

            self._frames[idx]['scan_num'] += int(samples)
            return ret

        def get_scans(self):
            if self._frame_ctr == 0:
                return None
            idx = self._frame_out % self._frame_max
            self._frame_out += 1
            self._frame_ctr -= 1
            return self._frames[idx]


if __name__ == "__main__":
    pygame.init()
    screen = pygame.display.set_mode([800, 800])
    sysfont = pygame.font.get_default_font()
    font1 = pygame.font.SysFont(sysfont, 72)

    lidar = LidarX2('COM8')
    lidar.open()

    running = True
    while running:
        try:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    key_event = pygame.key.get_pressed()
                    if key_event[pygame.K_ESCAPE]:
                        running = False
            time.sleep(0.01)
        except OSError as msg:
            print(msg)
        except KeyboardInterrupt:
            break

    lidar.close()
    pygame.quit()
