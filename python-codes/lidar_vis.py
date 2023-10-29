import pygame
import math
import numpy
import serial
import time
import threading
from msp import MSP
import struct
import copy


def GetDataFromArduino(line):
    global distances_list, first_run
    # [:-3] get rid of end of line sign and additional comma separator that is sent from arduino
    data = line[:-3]
    # print(data)
    d_list = data.split(",")
    return d_list


def GenerateLinePositions(numberOfLines):
    angle = 360 / numberOfLines
    lines = []
    for x in range(numberOfLines):
        # lines.append([300 * math.cos((x+1)*angle/180 * math.pi), 300 * math.sin((x+1)*angle/180 * math.pi)])
        rad = math.radians((x + 1) * angle)
        lines.append([300 * math.cos(rad), 300 * math.sin(rad)])
    return lines


def main1():
    pygame.init()

    # constant based on lidar resolution
    LIDAR_RESOLUTION = 240
    # lidar resolution divided by 4 to simplify the visualization
    VISUALIZATION_RESOLUTION = 240

    distances_list = []
    first_run = True

    line_positions = GenerateLinePositions(VISUALIZATION_RESOLUTION)

    # Set up the drawing window
    screen = pygame.display.set_mode([800, 800])
    sysfont = pygame.font.get_default_font()
    font1 = pygame.font.SysFont(sysfont, 72)

    file1 = open('test7.txt', 'r')
    Lines = file1.readlines()

    for line in Lines:
        distances = GetDataFromArduino(line)
        # print(len(distances))
        if (len(distances) == LIDAR_RESOLUTION):
            # Did the user click the window close button?
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Fill the background with white
            screen.fill((250, 250, 250))

            for x in range(VISUALIZATION_RESOLUTION):
                a = int(distances[x])/2000
                c = (0, 0, 0)
                r = 2
                if x in [141, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 203, 204, 205, 206, 207]:
                    c = (255, 0, 0)
                    r = 3
                pygame.draw.circle(
                    screen, c, (line_positions[x][0]*a+400, line_positions[x][1]*a+400), r)

            pygame.draw.circle(screen, (252, 132, 3), (400, 400), 12)
            # Flip the display
            pygame.display.flip()
            pygame.time.wait(100)

    pygame.quit()


# ----------------------------------------------------------------------------------------------
# Protocol class
# ----------------------------------------------------------------------------------------------
class Protocol(object):
    """\
    Protocol as used by the ReaderThread. This base class provides empty
    implementations of all methods.
    """

    def connection_made(self, transport):
        """Called when reader thread is started"""

    def data_received(self, data):
        """Called with snippets received from the serial port"""

    def connection_lost(self, exc):
        """\
        Called when the serial port is closed or the reader loop terminated
        otherwise.
        """
        if isinstance(exc, Exception):
            raise exc


# ----------------------------------------------------------------------------------------------
# ReaderThread class
# ----------------------------------------------------------------------------------------------
class ReaderThread(threading.Thread):
    def __init__(self, serial_instance, protocol_factory):
        """\
        Initialize thread.
        Note that the serial_instance' timeout is set to one second!
        Other settings are not changed.
        """
        super(ReaderThread, self).__init__()
        self.daemon = True
        self.serial = serial_instance
        self.protocol_factory = protocol_factory
        self.alive = True
        self._lock = threading.Lock()
        self._connection_made = threading.Event()
        self.protocol = None

    def stop(self):
        """Stop the reader thread"""
        self.alive = False
        if hasattr(self.serial, 'cancel_read'):
            self.serial.cancel_read()
        self.join(2)

    def run(self):
        """Reader loop"""
        if not hasattr(self.serial, 'cancel_read'):
            self.serial.timeout = 1
        self.protocol = self.protocol_factory()
        try:
            self.protocol.connection_made(self)
        except Exception as e:
            self.alive = False
            self.protocol.connection_lost(e)
            self._connection_made.set()
            return
        error = None
        self._connection_made.set()
        while self.alive and self.serial.is_open:
            try:
                # read all that is there or wait for one byte (blocking)
                data = self.serial.read(self.serial.in_waiting or 1)
            except serial.SerialException as e:
                # probably some I/O problem such as disconnected USB serial
                # adapters -> exit
                error = e
                break
            else:
                if data:
                    # make a separated try-except for called used code
                    try:
                        self.protocol.data_received(data)
                    except Exception as e:
                        print(e)
                        error = e
                        break
        self.alive = False
        self.protocol.connection_lost(error)
        self.protocol = None

    def write(self, data):
        """Thread safe writing (uses lock)"""
        with self._lock:
            self.serial.write(data)

    def close(self):
        """Close the serial port and exit reader thread (uses lock)"""
        # use the lock to let other threads finish writing
        with self._lock:
            # first stop reading, so that closing can be done on idle port
            self.stop()
            self.serial.close()

    def connect(self):
        """
        Wait until connection is set up and return the transport and protocol
        instances.
        """
        if self.alive:
            self._connection_made.wait()
            if not self.alive:
                raise RuntimeError('connection_lost already called')
            return (self, self.protocol)
        else:
            raise RuntimeError('already stopped')

    def getProtocol(self):
        return self.protocol

    # - -  context manager, returns protocol

    def __enter__(self):
        """\
        Enter context handler. May raise RuntimeError in case the connection
        could not be created.
        """
        self.start()
        self._connection_made.wait()
        if not self.alive:
            raise RuntimeError('connection_lost already called')
        return self.protocol

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Leave context: close port"""
        self.close()


# ----------------------------------------------------------------------------------------------
# ProtocalYDLidar class
# ----------------------------------------------------------------------------------------------
class ProtocalYDLidar(Protocol):
    def __init__(self):
        self.msp = MSP(self.write, self.cmd_callback)
        self.data = None
        self.sig_draw = threading.Event()

    def connection_made(self, transport):
        self.transport = transport
        self.running = True

    def connection_lost(self, exc):
        self.transport = None

    def data_received(self, data):
        self.msp.processRx(data)
        # print(", ".join(format(x, "02X") for x in data))

    def write(self, data):
        self.transport.write(data)

    def isRunning(self):
        return self.running

    def cmd_callback(self, cmd, data):
        self.data = copy.deepcopy(data)
        self.sig_draw.set()

    def is_updated(self):
        return self.sig_draw.is_set()

    def draw(self, screen):
        ts, scan_num = struct.unpack('<LH', self.data[0:6])
        print(f'{scan_num=:3d}')

        # Fill the background with white
        screen.fill((250, 250, 250))
        xpos = 0
        ypos = 0
        delta = 0.6
        ratio = 6
        scan_num = int(360 / delta)
        for x in range(scan_num):
            theta = delta * x
            dist = struct.unpack('<H', self.data[6+2*x:8+2*x])
            dist = dist[0] / ratio
            if theta < 0:
                theta += 360
            elif theta > 360:
                theta -= 360

            c = (0, 0, 0)
            r = 1
            # if x in [141, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 203, 204, 205, 206, 207]:
            #     c = (255, 0, 0)
            #     r = 3

            x = -dist * math.cos(math.radians(theta))
            y = -dist * math.sin(math.radians(theta))
            pygame.draw.circle(screen, c, (xpos + 400 + x, ypos + 400 + y), r)
        pygame.draw.circle(screen, (252, 132, 3), (xpos + 400, ypos + 400), 12)
        # Flip the display
        pygame.display.flip()
        self.sig_draw.clear()


def main2():
    pygame.init()
    screen = pygame.display.set_mode([800, 800])
    sysfont = pygame.font.get_default_font()
    font1 = pygame.font.SysFont(sysfont, 72)

    running = True
    # ser = serial.Serial(port='COM3', baudrate=230400)
    ser = serial.Serial(port='COM3', baudrate=230400)
    with ReaderThread(ser, ProtocalYDLidar) as p:
        while running and p.isRunning():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    key_event = pygame.key.get_pressed()
                    if key_event[pygame.K_ESCAPE]:
                        running = False

            try:
                if p.is_updated():
                    p.draw(screen)
                else:
                    time.sleep(0.01)
            except OSError as msg:
                print(msg)
            except KeyboardInterrupt:
                break

    pygame.quit()


if __name__ == "__main__":
    main2()
