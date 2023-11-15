import logging
from pyplotutil import PyplotUtil
import math
import matplotlib.pyplot as plt
import cv2
import numpy as np


class CVLkas(object):
    def __init__(self):
        pass

    def length_of_line_segment(self, line):
        x1, y1, x2, y2 = line
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def detect_line_segments(self, cropped_edges):
        # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
        rho = 1  # precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # degree in radian, i.e. 1 degree
        min_threshold = 10  # minimal of votes
        line_segments = cv2.HoughLinesP(
            cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=8, maxLineGap=4
        )

        if line_segments is not None:
            for line_segment in line_segments:
                logging.debug("detected line_segment:")
                logging.debug("%s of length %s" % (line_segment, self.length_of_line_segment(line_segment[0])))

        return line_segments

    def display_lines(self, frame, lines, line_color=(0, 255, 0), line_width=10):
        line_image = np.zeros_like(frame)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
        line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        return line_image

    def make_points(self, frame, line):
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height // 2  # center of the frame
        y2 = y1 - (height // 4)  # int(y1 * 1 / 2)  # make points from middle of the frame down

        # bound the coordinates within the frame
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]

    def average_slope_intercept(self, frame, line_segments):
        """
        This function combines line segments into one or two lane lines
        If all line slopes are < 0: then we only have detected left lane
        If all line slopes are > 0: then we only have detected right lane
        """
        lane_lines = []
        if line_segments is None:
            logging.info("No line_segment segments detected")
            return lane_lines

        height, width, _ = frame.shape
        left_fit = []
        right_fit = []

        boundary = 1 / 2
        left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
        right_region_boundary = (width * boundary)  # right lane line segment should be on left 2/3 of the screen

        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    logging.info("skipping vertical line segment (slope=inf): %s" % line_segment)
                    continue
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        left_fit.append((slope, intercept))
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        right_fit.append((slope, intercept))

        left_fit_average = np.average(left_fit, axis=0)
        if len(left_fit) > 0:
            lane_lines.append(self.make_points(frame, left_fit_average))

        right_fit_average = np.average(right_fit, axis=0)
        if len(right_fit) > 0:
            lane_lines.append(self.make_points(frame, right_fit_average))

        logging.debug("lane lines: %s" % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

        return lane_lines

    def compute_steering_angle(self, frame, lane_lines):
        """Find the steering angle based on lane line coordinate
        We assume that camera is calibrated to point to dead center
        """
        if len(lane_lines) == 0:
            logging.info("No lane lines detected, do nothing")
            return -90

        height, width, _ = frame.shape
        if len(lane_lines) == 1:
            logging.debug("Only detected one lane line, just follow it. %s" % lane_lines[0])
            x1, _, x2, _ = lane_lines[0][0]
            x_offset = x2 - x1
        else:
            _, _, left_x2, _ = lane_lines[0][0]
            _, _, right_x2, _ = lane_lines[1][0]
            camera_mid_offset_percent = 0.02  # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
            mid = int(width / 2 * (1 + camera_mid_offset_percent))
            x_offset = (left_x2 + right_x2) / 2 - mid

        # find the steering angle, which is angle between navigation direction to end of center line
        y_offset = int(height / 2)

        angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
        steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel
        logging.debug("new steering angle: %s" % steering_angle)
        return steering_angle

    def display_heading_line(self, frame, steering_angle, line_color=(0, 0, 255), line_width=5):
        heading_image = np.zeros_like(frame)
        height, width, _ = frame.shape

        # figure out the heading line from steering angle
        # heading line (x1,y1) is always center bottom of the screen
        # (x2, y2) requires a bit of trigonometry

        # Note: the steering angle of:
        # 0-89 degree: turn left
        # 90 degree: going straight
        # 91-180 degree: turn right
        steering_angle_radian = steering_angle / 180.0 * math.pi
        x1 = int(width / 2)
        y1 = int(height / 2)
        x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
        y2 = y1 - (height // 4)
        cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
        heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

        return heading_image

    def process(self, img):
        img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        line_segments = self.detect_line_segments(img_gray)
        line_segment_image = self.display_lines(img, line_segments, line_color=(255, 0, 0), line_width=1)
        lane_lines = self.average_slope_intercept(img, line_segments)
        lane_lines_image = self.display_lines(img, lane_lines, line_color=(255, 0, 0), line_width=2)

        new_steering_angle = self.compute_steering_angle(lane_lines_image, lane_lines)
        curr_heading_image = self.display_heading_line(lane_lines_image, new_steering_angle)

        return line_segment_image, curr_heading_image, new_steering_angle - 90
