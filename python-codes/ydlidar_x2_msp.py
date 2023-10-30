import socket
import pygame
import time
import threading
import struct
import math
from msp import MSP


class LidarX2(object):
    def __init__(self, port, host=None, sceen=None):
        self._port = port
        self._thread = None
        self._sig_kill = threading.Event()
        self._host = host
        self._screen = screen
        self._client = None
        self._max_scans = 600

    def open(self):
        self._thread = threading.Thread(target=self.thread_tcp_reader, args=(self._sig_kill,))
        self._thread.start()

    def thread_tcp_reader(self, sig_kill):
        print('thread_tcp_reader started')
        self._msp = MSP(self.write, self.cmd_callback)

        # connection loop
        while True:
            try:
                self._client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            except OSError as msg:
                print(msg)

            self._client.settimeout(3)
            if self._client:
                try:
                    print(">> Connect Server")
                    self._client.connect((self._host, self._port))
                    print("connected...")
                    break
                except OSError as msg:
                    print(msg)
                except KeyboardInterrupt:
                    break

        # reading loop
        while self._client and not sig_kill.is_set():
            try:
                buf = self._client.recv(30)
                self._msp.processRx(buf)
            except OSError as msg:
                print(msg)

        if self._client:
            self._client.close()
        print('thread_tcp_reader terminated')

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

    # MSP write interface
    def write(self, data):
        self._client.send(data)

    # callback from MSP
    def cmd_callback(self, cmd, data):
        if cmd == 1:
            frame = {'ts': 0, 'scan_num': 0, 'scans': [0] * self._max_scans}
            frame['ts'], frame['scan_num'] = struct.unpack('<LH', data[0:6])
            for i in range(self._max_scans):
                frame['scans'][i] = struct.unpack('<H', data[6+2*i:8+2*i])[0]
            print(f"{frame['ts']:8d} : {frame['scan_num']:3d}")
            self.draw(self._screen, frame)


if __name__ == "__main__":
    HOST = "192.168.0.155"
    PORT = 8080

    pygame.init()
    screen = pygame.display.set_mode([800, 800])
    sysfont = pygame.font.get_default_font()
    font1 = pygame.font.SysFont(sysfont, 72)

    lidar = LidarX2(PORT, HOST, screen)
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
