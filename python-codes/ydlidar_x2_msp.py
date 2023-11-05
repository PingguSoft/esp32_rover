import socket
import time
import threading
import struct
import math
from msp import MSP
from odom_icp.utils import *
from odom_icp.icp import *
import matplotlib.pyplot as plt
from datetime import datetime
import cv2

plt.switch_backend('agg')


class Robot(object):
    CMD_LIDAR = 0x01
    CMD_TICKS = 0x02
    CMD_ODOMETRY = 0x03
    CMD_BUTTON = 0x04

    CMD_RESET = 0x10
    CMD_SPEED = 0x11

    BTN_START = 0x800

    def __init__(self):
        self._thread = None
        self._sig_kill = threading.Event()
        self._is_record = False
        self._rec_file = None
        #
        self._port = 0
        self._host = None
        self._client = None
        #
        self._max_scans = 600
        self._frame = None
        self._scale_div = 10
        #
        self._xpos = 0
        self._ypos = 0
        self._theta = 0
        self._traj = []
        #
        self._lidar = Lidar(-math.pi, math.pi, 600, 0.10, 60)
        self._pose = [0, 0, 0]
        self._lidar_traj = []
        self._scan_before = None
        #
        self._sig_draw = {'lidar': threading.Event(), 'pose': threading.Event(), 'slam': threading.Event()}

    def open(self, host, port):
        self._port = port
        self._host = host
        self._thread = threading.Thread(target=self.thread_tcp_reader, args=(self._sig_kill,))
        self._thread.start()

    def open_file(self, file):
        self._rec_file = open(file, "rt")
        self._thread = threading.Thread(target=self.thread_replay, args=(self._sig_kill,))
        self._thread.start()

    #
    # thread_replay
    #
    def format_line(self, line):
        line = line.replace("[", "")
        line = line.replace("]", "")
        list = line.split(",")
        for i, l in enumerate(list):
            list[i] = int(l)
        return list

    def thread_replay(self, sig_kill):
        print('thread_replay started')
        # reading loop
        cnt = 0
        while self._rec_file and not sig_kill.is_set():
            try:
                line = self._rec_file.readline()
                if line:
                    data = self.format_line(line)
                    self._frame = {'ts': 0, 'aux': 0, 'scan_num': 0, 'scans': None}
                    self._frame['aux'] = data[0]
                    self._frame['scans'] = data[1:]
                    self.calc_lidar_odometry(self._frame)
                    self._sig_draw['lidar'].set()
                    cnt += 1
                    print(f"frame : {cnt:5d}")
                else:
                    self._rec_file.seek(0)
                    print("------")
                time.sleep(0.01)
            except OSError as msg:
                print(msg)

        if self._rec_file:
            self._rec_file.close()
        print('thread_replay terminated')

    #
    # thread_tcp_reader
    #
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

    def calc_lidar_odometry(self, frame):
        scan_current = pol2cart(self._lidar.angles, np.array(frame['scans']).reshape(-1, 600) / self._scale_div)
        if self._scan_before is not None:
            scan_before_global = localToGlobal(self._pose, self._scan_before)
            scan_current_global = localToGlobal(self._pose, scan_current)
            self._lidar_traj.append(self._pose)

            T = icp(scan_current_global, scan_before_global)
            pose_T = v2t(self._pose)
            self._pose = t2v(np.copy(np.dot(T, pose_T)))
            frame = np.ones((3, scan_current_global.shape[0]))
            frame[:2, :] = scan_current_global.T
            self._result = (T @ frame)[:2, :].T
            self._sig_draw['pose'].set()
        self._scan_before = scan_current

    def handle_keys(self, key):
        if key == ord('r'):
            print("reset !!")
            self._traj = []
            self._msp.send(Robot.CMD_RESET, None, 0)
        elif key == ord('1'):
            spd = bytes([128])
            self._msp.send(Robot.CMD_SPEED, spd, 1)
            print("speed:128")
        elif key == ord('2'):
            spd = bytes([255])
            self._msp.send(Robot.CMD_SPEED, spd, 1)
            print("speed:255")

    #
    # MSP write interface
    #
    def write(self, data):
        self._client.send(data)

    #
    # callback from MSP
    #
    def cmd_lidar(self, data):
        self._frame = {'ts': 0, 'aux': 0, 'scan_num': 0, 'scans': [0] * self._max_scans}
        self._frame['ts'], self._frame['aux'], self._frame['scan_num'] = struct.unpack('<LbH', data[0:7])
        for i in range(self._max_scans):
            self._frame['scans'][i] = struct.unpack('<H', data[7+2*i:9+2*i])[0]
        # print(f"{self._frame['ts']:8d} : {self._frame['aux']:3d}, {self._frame['scan_num']:3d}")

        self.calc_lidar_odometry(self._frame)
        if self._is_record and self._rec_file:
            self._rec_file.write(f"{self._frame['aux']:3d}, {self._frame['scans']}\n")
            # print(f"{self._frame['aux']:3d}, {self._frame['scans']}")
        self._sig_draw['lidar'].set()

    def cmd_odometry(self, data):
        ts, self._xpos, self._ypos, self._theta = struct.unpack('<Lllf', data[0:16])
        # print(f"{self._xpos=:8d}, {self._ypos=:8d}, {math.degrees(self._theta):5.2f}")
        if len(self._traj) == 0 or self._traj[-1] != (self._xpos, self._ypos):
            self._traj.append((self._xpos, self._ypos))

    def cmd_ticks(self, data):
        ts, l, r = struct.unpack('<Lll', data[0:12])
        # print(f"{l=:8d}, {r=:8d}")

    def cmd_button(self, data):
        btn = struct.unpack('<I', data[0:4])[0]
        # print(f"{btn=:4x}")
        if (btn & Robot.BTN_START) == Robot.BTN_START:
            self._is_record = not self._is_record

            if self._is_record and self._rec_file is None:
                now = datetime.now()
                filename = now.strftime("%m%d_%H%M%S")
                self._rec_file = open(filename + ".log", "wt")
                print(f"log file  : {filename}")
            elif not self._is_record and self._rec_file:
                print(f"log saved")
                self._rec_file.close()
                self._rec_file = None

    def cmd_callback(self, cmd, data):
        cmd_func_table = {Robot.CMD_LIDAR: self.cmd_lidar,
                          Robot.CMD_ODOMETRY: self.cmd_odometry,
                          Robot.CMD_TICKS: self.cmd_ticks,
                          Robot.CMD_BUTTON: self.cmd_button}
        return cmd_func_table[cmd](data)

    #
    # show lidar image
    #
    def show_lidar_image(self):
        fig = plt.figure(figsize=(8, 8))
        plt.clf()
        plt.autoscale(False)
        plt.plot(0, 0, "go", markersize=8)
        plt.axvline(0, color="g", ls="--")
        plt.axhline(0, color="g", ls="--")
        lim = 4000 / self._scale_div
        plt.xlim([-lim, lim])
        plt.ylim([-lim, lim])

        angle_res = 0.6
        start = int(0 / angle_res)
        end = int(360 / angle_res)

        # draw lidar image
        for x in range(start, end, 1):
            theta = 180 - (angle_res * x)                       # lidar installed with -90 degree and rotate CW
            dist = self._frame['scans'][x] / self._scale_div    # but screen coordinates CCW so 180 -

            c = (0, 0, 0)
            r = 2
            x = dist * math.cos(math.radians(theta))
            y = dist * math.sin(math.radians(theta))    # screen y is reversed
            plt.plot(x, y, 'o', markersize=1, color='black')
        plt.plot(0, 0, 'o', markersize=5, color='blue')

        # draw pose direction
        x = 20 * math.cos((math.pi / 2) - self._theta)
        y = 20 * math.sin((math.pi / 2) - self._theta)

        # pygame.draw.line(screen, (255, 0, 0), (xpos, ypos), (xpos + x, ypos - y), 3)

        # draw trajectory
        # list = self._traj / dist_div
        # plt.plot(list[0], list[1], 'o', markersize=1, color='red')

        # for x, y in self._traj:
        #     x /= dist_div
        #     y /= dist_div
        #     pygame.draw.circle(screen, (0, 0, 255), (xpos + x, ypos - y), 2)

        # pygame.display.flip()
        fig.canvas.draw()
        img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        plt.close(fig)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.putText(img, f"{self._frame['aux']:3d}", (img.shape[1] // 2 - 40, img.shape[0] - 20),
                    cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 255), 2)
        cv2.imshow('lidar', img)
        self._sig_draw['lidar'].clear()

    #
    # show lidar pose
    #
    def show_lidar_pose(self):
        if not self._sig_draw['pose'].is_set():
            return

        fig = plt.figure(figsize=(8, 8))
        plt.clf()
        plt.autoscale(False)
        plt.plot(0, 0, "go", markersize=8)
        plt.axvline(0, color="g", ls="--")
        plt.axhline(0, color="g", ls="--")
        lim = 4000 / self._scale_div
        plt.xlim([-lim, lim])
        plt.ylim([-lim, lim])

        if len(self._lidar_traj) > 0:
            pose = self._lidar_traj[-1]
            plt.plot(pose[0], pose[1], 'o', color='blue', markersize=3)
            traj_array = np.array(self._lidar_traj)
            plt.plot(traj_array[:, 0], traj_array[:, 1], color='black')

        if self._result is not None:
            plt.plot(self._result[:, 0], self._result[:, 1], 'o', markersize=1, color='red')

        fig.canvas.draw()
        img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        plt.close(fig)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imshow('lidar pose', img)
        self._sig_draw['pose'].clear()

    def show_images(self):
        sig_func_table = {('lidar', self.show_lidar_image),
                          ('pose', self.show_lidar_pose)}

        for sig, func in sig_func_table:
            if self._sig_draw[sig].is_set():
                func()
                self._sig_draw[sig].clear()


if __name__ == "__main__":
    robot = Robot()
    robot.open(host="192.168.0.155", port=8080)
    # robot.open("train.log")

    running = True
    while running:
        try:
            key = cv2.pollKey()
            if key == 27:
                running = False
                break
            robot.handle_keys(key)
            robot.show_images()
            time.sleep(0.05)
        except OSError as msg:
            print(msg)
        except KeyboardInterrupt:
            running = False
            break

    robot.close()
