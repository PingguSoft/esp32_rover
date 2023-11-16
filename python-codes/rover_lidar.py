import socket
import time
import threading
import traceback
from queue import Queue, Empty
import struct
import math
from msp import MSP
from odom_icp.utils import *
from odom_icp.icp import *
import matplotlib.pyplot as plt
import matplotlib.lines as lines
from datetime import datetime
import cv2
from pyplotutil import PyplotUtil
from cv_lkas import CVLkas

plt.switch_backend('agg')


class Rover(object):
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
        self._queue = Queue(20)
        self._is_record = False
        self._log_file = None
        self._log_msg = None
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
        self._plot = PyplotUtil(1, 3, figsize=(6 * 3, 6))
        self._canvas = {'lidar': self._plot.get_ax()[0],
                        'cv': self._plot.get_ax()[1],
                        'pose': self._plot.get_ax()[2],
                        }
        #
        self._plot_cv = PyplotUtil(1, 1, figsize=(2, 2))
        self._cv_proc = CVLkas()

    def open(self, host, port):
        self._port = port
        self._host = host
        self._thread = threading.Thread(target=self.thread_tcp_reader, args=(self._sig_kill,))
        self._thread.start()

    def open_file(self, file):
        self._log_file = file
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
            except OSError:
                print(traceback.format_exc())

            self._client.settimeout(3)
            if self._client:
                try:
                    print(">> Connect Server")
                    self._client.connect((self._host, self._port))
                    print("connected...")
                    break
                except OSError:
                    print(traceback.format_exc())
                except KeyboardInterrupt:
                    break

        # reading loop
        while self._client and not sig_kill.is_set():
            try:
                buf = self._client.recv(30)
                self._msp.processRx(buf)
            except OSError:
                print(traceback.format_exc())

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
        d_list = []
        for l in list:
            d_list.append(int(l))
        return d_list

    def thread_replay(self, sig_kill):
        print('thread_replay started')

        with open(self._log_file, "rt") as f:
            lines = f.readlines()
            x = []
            y = []
            for line in lines:
                data = self.format_line(line)
                x.append(data[1:])
                y.append(data[0])

        # reading loop
        idx = 0
        is_block = True
        while not sig_kill.is_set():
            if idx < len(y):
                self._log_msg = f"{idx:5d} / {len(y):5d}"
                self._frame = {'ts': 0, 'aux': 0, 'scan_num': 0, 'scans': None}
                self._frame['aux'] = y[idx]
                self._frame['scans'] = x[idx]

                self.update(self._frame)

                try:
                    key = self._queue.get(block=is_block)
                    if key == 0x250000 and idx > 0:     # left arrow
                        idx -= 1
                    elif key == 0x270000:               # right arrow
                        idx += 1
                    elif key == ord(' '):
                        is_block = not is_block
                    elif key == ord('d'):
                        print(f"delete {idx:6d} {y[idx]:3d}")
                        x.pop(idx)
                        y.pop(idx)
                        idx += 1
                    elif key == ord('s'):
                        print(f"save logs file")
                        with open(self._log_file, "wt") as f:
                            for i in range(len(y)):
                                f.write(f"{y[i]:3d}, [")
                                f.write(", ".join(str(e) for e in x[i]))
                                f.write("]\n")
                except Empty:
                    pass

                if not is_block:
                    idx += 1
            else:
                print("-" * 100)
                time.sleep(5)
                idx = 0

        print('thread_replay terminated')

    def close(self):
        self._queue.put(0)
        self._sig_kill.set()
        self._thread.join()

    def calc_lidar_odometry(self, frame):
        ret = True
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
        else:
            ret = False
        self._scan_before = scan_current
        return ret

    def handle_keys(self, key):
        if key == ord('r'):
            print("reset !!")
            self._traj = []
            self._msp.send(Rover.CMD_RESET, None, 0)
        elif key == ord('1'):
            spd = bytes([128])
            self._msp.send(Rover.CMD_SPEED, spd, 1)
            print("speed:128")
        elif key == ord('2'):
            spd = bytes([255])
            self._msp.send(Rover.CMD_SPEED, spd, 1)
            print("speed:255")
        elif key != -1:
            self._queue.put(key)

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

        if self._is_record and self._log_file:
            self._log_file.write(f"{self._frame['aux']:3d}, {self._frame['scans']}\n")
            # print(f"{self._frame['aux']:3d}, {self._frame['scans']}")

        self.update(self._frame)


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
        if (btn & Rover.BTN_START) == Rover.BTN_START:
            self._is_record = not self._is_record

            if self._is_record and self._log_file is None:
                now = datetime.now()
                filename = now.strftime("%m%d_%H%M%S")
                self._log_file = open(filename + ".log", "wt")
                print(f"log file  : {filename}")
            elif not self._is_record and self._log_file:
                print(f"log saved")
                self._log_file.close()
                self._log_file = None

    def cmd_callback(self, cmd, data):
        cmd_func_table = {Rover.CMD_LIDAR: self.cmd_lidar,
                          Rover.CMD_ODOMETRY: self.cmd_odometry,
                          Rover.CMD_TICKS: self.cmd_ticks,
                          Rover.CMD_BUTTON: self.cmd_button}
        return cmd_func_table[cmd](data)

    #
    # update
    #
    def update(self, frame):
        self._plot.set(ax=self._canvas['lidar'], title='lidar', lim=200, scale_div=self._scale_div)
        self.draw_lidar_image(self._canvas['lidar'], frame, 'black', 6000)

        self._plot.set(ax=self._canvas['cv'], title='cv', lim=200)
        self.draw_cv_image(self._canvas['cv'], frame)

        # self._plot.set(ax=self._canvas['pose'], title='pose', lim=200, scale_div=self._scale_div)
        # if self.calc_lidar_odometry(frame):
        #     self.draw_lidar_pose(self._canvas['pose'])

        self._plot.finish(cv2.COLOR_RGB2BGR)

    #
    # draw lidar image
    #
    def draw_lidar_image(self, ax, frame, color, max_dist, deco=True):
        if deco:
            ax.plot(0, 0, "go", markersize=8)

        angle_res = 0.6
        start = int(0 / angle_res)
        end = int(360 / angle_res)

        roi_start = int(30 / angle_res)
        roi_end = int(150 / angle_res)

        # draw lidar image
        xlist = []
        ylist = []
        for i in range(start, end, 1):
            # lidar installed with -90 degree and rotate CW
            theta = 180 - (angle_res * i)
            # but screen coordinates CCW so 180 -
            dist = frame['scans'][i]
            if dist > max_dist:
                continue

            dist /= self._scale_div
            x = dist * math.cos(math.radians(theta))
            y = dist * math.sin(math.radians(theta))    # screen y is reversed
            xlist.append(x)
            ylist.append(y)

        if deco:
            ax.plot(xlist, ylist, 'o', markersize=1, color=color)
            ax.plot(xlist[roi_start:roi_end], ylist[roi_start:roi_end], 'o', markersize=1, color='red')
        else:
            ax.plot(xlist[roi_start:roi_end], ylist[roi_start:roi_end], 'o', markersize=1, color=color)

        if deco:
            # draw pose direction
            if self._log_file is not None:
                self._theta = math.radians(frame['aux'])

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
            ax.text(ax.get_xlim()[1] - 150, ax.get_ylim()[0], self._log_msg, fontsize=10.0, color='red')
            # draw steering angle
            ax.text(-40, ax.get_ylim()[0], f"{self._frame['aux']:3d}", fontsize=20.0, color='red')

    #
    # draw lidar pose
    #
    def draw_lidar_pose(self, ax):
        if len(self._lidar_traj) > 0:
            pose = self._lidar_traj[-1]
            ax.plot(pose[0], pose[1], 'o', color='blue', markersize=3)
            traj_array = np.array(self._lidar_traj)
            ax.plot(traj_array[:, 0], traj_array[:, 1], color='red')

        if self._result is not None:
            ax.plot(self._result[:, 0], self._result[:, 1], 'o', markersize=1, color='black')


    #
    # draw CV image
    #
    def draw_cv_image(self, ax:plt.Axes, frame):
        self._plot_cv.set(title=None, lim=200, color='white', axis=False)
        self.draw_lidar_image(self._plot_cv.get_ax(), frame, 'black', 500, False)
        img = 255 - self._plot_cv.finish()  # invert
        img_seg, img_head, angle = self._cv_proc.process(img)
        ax.imshow(img_seg,  origin='upper', extent=(-200, 0, 0, 200))
        ax.imshow(img_head, origin='upper', extent=(0, 200, 0, 200))

    #
    def show_outputs(self):
        if self._plot.is_updated():
            image = self._plot.get_image()
            if image is not None:
                cv2.imshow("outputs", image)
            self._plot.clear_updated()

        key = cv2.waitKeyEx(10)
        self.handle_keys(key)
        return key


#
# main
#
if __name__ == "__main__":
    robot = Rover()
    # robot.open(host="192.168.0.155", port=8080)
    robot.open_file("train_01.log")

    running = True
    while running:
        try:
            key = robot.show_outputs()
            if key == 27:
                running = False
                break
        except KeyboardInterrupt:
            running = False
            break
        except Exception as e:
            print(traceback.format_exc())
            running = False
            break

    robot.close()
