import socket
import pygame
import time
import threading
import struct
import math
from msp import MSP


class Robot(object):
    CMD_LIDAR = 0x01
    CMD_TICKS = 0x02
    CMD_ODOMETRY = 0x03

    CMD_RESET = 0x10
    CMD_SPEED = 0x11

    def __init__(self, port, host=None, sceen=None):
        self._port = port
        self._thread = None
        self._sig_kill = threading.Event()
        self._host = host
        self._screen = screen
        self._client = None
        self._max_scans = 600
        self._xpos = 0
        self._ypos = 0
        self._theta = 0
        self._traj = []

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
        w, h = screen.get_size()
        xpos = w // 2
        ypos = h // 2
        dist_div = 5
        angle_res = 0.6
        start = int(0 / angle_res)
        end = int(360 / angle_res)
        # print(frame['scans'])
        # print('-' * 20)

        screen.fill((250, 250, 250))
        for x in range(start, end, 1):
            theta = 180 - (angle_res * x)                   # lidar installed with -90 degree and rotate CW
            dist = frame['scans'][x]                        # but screen coordinates CCW so 180 -
            dist /= dist_div

            c = (0, 0, 0)
            r = 2
            x = dist * math.cos(math.radians(theta))
            y = dist * math.sin(math.radians(theta))    # screen y is reversed
            pygame.draw.circle(screen, c, (xpos + x, ypos - y), r)
        pygame.draw.circle(screen, (252, 132, 3), (xpos, ypos), 12)

        x = 20 * math.cos((math.pi / 2) - self._theta)
        y = 20 * math.sin((math.pi / 2) - self._theta)
        pygame.draw.line(screen, (255, 0, 0), (xpos, ypos), (xpos + x, ypos - y), 3)

        for x, y in self._traj:
            x /= dist_div
            y /= dist_div
            pygame.draw.circle(screen, (0, 0, 255), (xpos + x, ypos - y), 2)

        pygame.display.flip()

    def handle_keys(self, key):
        if key[pygame.K_r]:
            print("reset !!")
            self._traj = []
            self._msp.send(Robot.CMD_RESET, None, 0)
        elif key[pygame.K_1]:
            spd = bytes([128])
            self._msp.send(Robot.CMD_SPEED, spd, 1)
            print("speed:128")
        elif key[pygame.K_2]:
            spd = bytes([255])
            self._msp.send(Robot.CMD_SPEED, spd, 1)
            print("speed:255")

    # MSP write interface

    def write(self, data):
        self._client.send(data)

    # callback from MSP
    def cmd_lidar(self, data):
        frame = {'ts': 0, 'scan_num': 0, 'scans': [0] * self._max_scans}
        frame['ts'], frame['scan_num'] = struct.unpack('<LH', data[0:6])
        for i in range(self._max_scans):
            frame['scans'][i] = struct.unpack('<H', data[6+2*i:8+2*i])[0]
        # print(f"{frame['ts']:8d} : {frame['scan_num']:3d}")
        self.draw(self._screen, frame)

    def cmd_odometry(self, data):
        ts, self._xpos, self._ypos, self._theta = struct.unpack('<Lllf', data[0:16])
        # print(f"{self._xpos=:8d}, {self._ypos=:8d}, {math.degrees(self._theta):5.2f}")
        if len(self._traj) == 0 or self._traj[-1] != (self._xpos, self._ypos):
            self._traj.append((self._xpos, self._ypos))

    def cmd_ticks(self, data):
        ts, l, r = struct.unpack('<Lll', data[0:12])
        # print(f"{l=:8d}, {r=:8d}")

    def cmd_callback(self, cmd, data):
        cmd_func_table = {Robot.CMD_LIDAR: self.cmd_lidar,
                          Robot.CMD_ODOMETRY: self.cmd_odometry,
                          Robot.CMD_TICKS: self.cmd_ticks}
        return cmd_func_table[cmd](data)


if __name__ == "__main__":
    HOST = "192.168.0.155"
    PORT = 8080

    pygame.init()
    screen = pygame.display.set_mode([800, 800])
    sysfont = pygame.font.get_default_font()
    font1 = pygame.font.SysFont(sysfont, 72)

    robot = Robot(PORT, HOST, screen)
    robot.open()

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
                    robot.handle_keys(key_event)
            time.sleep(0.01)
        except OSError as msg:
            print(msg)
        except KeyboardInterrupt:
            break

    robot.close()
    pygame.quit()
