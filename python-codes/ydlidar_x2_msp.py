import socket
import time
import threading
import struct
import math
from msp import MSP
from odom_icp.utils import *
from odom_icp.icp import *
import matplotlib.pyplot as plt
import matplotlib.lines as lines
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
        self._fig = None
        self._axes = None
        self._canvas = {'lidar': {'sig': threading.Event(), 'ax': None, 'func': self.show_lidar_image},
                        'pose': {'sig': threading.Event(), 'ax': None, 'func': self.show_lidar_pose},
                        'slam': {'sig': threading.Event(), 'ax': None, 'func': None}}

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
        while self._rec_file and not sig_kill.is_set():
            try:
                line = self._rec_file.readline()
                if line:
                    data = self.format_line(line)
                    self._frame = {'ts': 0, 'aux': 0, 'scan_num': 0, 'scans': None}
                    self._frame['aux'] = data[0]
                    self._frame['scans'] = data[1:]
                    self._canvas['lidar']['sig'].set()
                    self.calc_lidar_odometry(self._frame)
                    time.sleep(0.2)
                else:
                    self._rec_file.seek(0)
                    print("-" * 100)
                    time.sleep(5)
            except Exception as e:
                print(e)
                break

        if self._rec_file:
            self._rec_file.close()
        print('thread_replay terminated')

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
            self._canvas['pose']['sig'].set()
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
        self._canvas['lidar']['sig'].set()

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
    def show_lidar_image(self, ax):
        angle_res = 0.6
        start = int(0 / angle_res)
        end = int(360 / angle_res)

        roi_start = int(30 / angle_res)
        roi_end = int(150 / angle_res)

        # draw lidar image
        xlist = []
        ylist = []
        for i in range(start, end, 1):
            theta = 180 - (angle_res * i)                       # lidar installed with -90 degree and rotate CW
            dist = self._frame['scans'][i] / self._scale_div    # but screen coordinates CCW so 180 -
            x = dist * math.cos(math.radians(theta))
            y = dist * math.sin(math.radians(theta))    # screen y is reversed
            xlist.append(x)
            ylist.append(y)
        ax.plot(xlist, ylist, 'o', markersize=1, color='black')
        ax.plot(xlist[roi_start:roi_end], ylist[roi_start:roi_end], 'o', markersize=1, color='red')


        # draw pose direction
        if self._rec_file is not None:
            self._theta = math.radians(self._frame['aux'])

        x = int(50 * math.cos((math.pi / 2) - self._theta))
        y = int(50 * math.sin((math.pi / 2) - self._theta))
        line = lines.Line2D([0, x], [0, y], lw=2, color='cyan', axes=ax)
        ax.add_line(line)

        # draw trajectory
        xlist = []
        ylist = []
        for x, y in self._traj:
            x /= self._scale_div
            y /= self._scale_div
            xlist.append(x)
            ylist.append(y)
        ax.plot(xlist, ylist, 'o', markersize=1, color='lime')

        # draw steering angle
        ax.text(-40, ax.get_ylim()[0], f"{self._frame['aux']:3d}", fontsize=20.0, color='red')

    #
    # show lidar pose
    #
    def show_lidar_pose(self, ax):
        if len(self._lidar_traj) > 0:
            pose = self._lidar_traj[-1]
            ax.plot(pose[0], pose[1], 'o', color='blue', markersize=3)
            traj_array = np.array(self._lidar_traj)
            ax.plot(traj_array[:, 0], traj_array[:, 1], color='red')

        if self._result is not None:
            ax.plot(self._result[:, 0], self._result[:, 1], 'o', markersize=1, color='black')

    #
    def set_axes_default(self, ax, title):
        ax.clear()
        ax.set_title(title)
        ax.set_autoscale_on(False)
        ax.plot(0, 0, "go", markersize=8)
        ax.axvline(0, color="lightgray", ls="--")
        ax.axhline(0, color="lightgray", ls="--")
        lim = 4000 / self._scale_div
        ax.set_xlim([-lim, lim])
        ax.set_ylim([-lim, lim])
        ax.text(ax.get_ylim()[0], ax.get_ylim()[1], f"scale=1/{self._scale_div} mm", fontsize=10.0, color='red')

    #
    def show_outputs(self):
        if self._fig is None:
            self._fig, self._axes = plt.subplots(1, len(self._canvas), figsize=(6 * len(self._canvas), 6))
            self._fig.set_tight_layout(True)
            for i, c in enumerate(self._canvas):
                self._canvas[c]['ax'] = self._axes[i]

        is_dirty = True
        for i, c in enumerate(self._canvas):
            if self._canvas[c]['sig'].is_set() and self._canvas[c]['func'] != None:
                s = time.monotonic()
                self.set_axes_default(self._canvas[c]['ax'], c)
                self._canvas[c]['func'](self._axes[i])
                print(f"draw elapsed {c:10s} {time.monotonic() - s:5.3f}")
                self._canvas[c]['sig'].clear()
                is_dirty = True

        if is_dirty:
            self._fig.canvas.draw()
            img = np.frombuffer(self._fig.canvas.tostring_rgb(), dtype=np.uint8)
            img = img.reshape(self._fig.canvas.get_width_height()[::-1] + (3,))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cv2.imshow('images', img)

#
# main
#
if __name__ == "__main__":
    robot = Robot()
    # robot.open(host="192.168.0.155", port=8080)
    robot.open_file("train_01.log")

    running = True
    while running:
        try:
            robot.show_outputs()
            key = cv2.waitKey(1)
            if key == 27:
                running = False
                break
            robot.handle_keys(key)
        except KeyboardInterrupt:
            running = False
            break
        except Exception as e:
            print(e)
            running = False
            break

    robot.close()
