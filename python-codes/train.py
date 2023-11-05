import numpy as np
import pandas as pd
from sklearn.feature_selection import SelectKBest
from sklearn.feature_selection import f_classif
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
import matplotlib.pyplot as plt
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, classification_report
import math
import cv2
import numpy
from random import *


class TrainLidarSteering(object):
    def __init__(self, start, end, res):
        self._X = []
        self._y = []
        self._start = int(start / res)
        self._end = int(end / res)
        self._scale_div = 10

    def format_line(self, line):
        line = line.replace("[", "")
        line = line.replace("]", "")
        list = line.split(",")
        d_list = []
        for l in list:
            d_list.append(int(l))
        return d_list

    def open(self, file):
        file = open(file, "rt")
        if file:
            lines = file.readlines()
            x = []
            y = []
            for line in lines:
                data = self.format_line(line)
                x.append(data[1:])
                y.append(data[0])
            file.close()
            self._X = np.array(x)
            self._y = np.array(y)
            print(self._X.shape)
            print(self._y.shape)

    def flip_h(self):
        x1 = np.flip(self._X[:, :300], axis=1)
        x2 = np.flip(self._X[:, 300:], axis=1)
        y = -self._y
        return np.concatenate((x1, x2), axis=1), y

    def train(self):
        X_train, X_test, y_train, y_test = train_test_split(self._X, self._y, test_size=0.2, random_state=42)

        # k = 15
        # k_best = SelectKBest(score_func=f_classif, k=k)
        # k_best.fit(X_train, y_train)
        # selected_feature_indices = k_best.get_support(indices=True)
        # print("selected_feature_indices: ", selected_feature_indices)

        clf = RandomForestClassifier(max_depth=50, random_state=42)
        X_train_lim = X_train[:, self._start:self._end]
        clf.fit(X_train_lim, y_train)

        X_test_lim = X_test[:, self._start:self._end]
        y_pred = clf.predict(X_test_lim)
        accuracy = accuracy_score(y_test, y_pred)
        print(f'Accuracy: {accuracy}')

    def show_lidar_image(self, title, frame, angle):
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
            dist = frame[x] / self._scale_div    # but screen coordinates CCW so 180 -
            if self._start <= x and x <= self._end:
                c = 'red'
            else:
                c = 'black'
            r = 2
            x = dist * math.cos(math.radians(theta))
            y = dist * math.sin(math.radians(theta))    # screen y is reversed
            plt.plot(x, y, 'o', markersize=r, color=c)
        plt.plot(0, 0, 'o', markersize=5, color='blue')

        fig.canvas.draw()
        img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        plt.close(fig)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.putText(img, f"{angle:3d}", (img.shape[1] // 2 - 40, img.shape[0] - 20),
                    cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 255), 2)
        cv2.imshow(title, img)

    def show_data(self):
        aug_x, aug_y = self.flip_h()
        i = randint(0, len(self._X))
        self.show_lidar_image('original', self._X[i], self._y[i])
        self.show_lidar_image('aug', aug_x[i], aug_y[i])
        cv2.waitKey(0)

        self._X = np.concatenate((self._X, aug_x))
        self._y = np.concatenate((self._y, aug_y))


if __name__ == "__main__":
    robot = TrainLidarSteering(0, 180, 0.6)
    robot.open("train.log")
    robot.show_data()
    # robot.train()
