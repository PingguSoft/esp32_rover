import matplotlib.pyplot as plt
import cv2
import numpy as np


class PyplotUtil(object):
    def __init__(self, figsize):
        self._fig, self._ax = plt.subplots(1, 1, figsize=figsize)
        self._fig.set_tight_layout(True)
        self._image = None
        self._updated = False

    def set(self, lim, color='white', axis=True, scale_div=None):
        self._updated = False
        self._ax.clear()
        self._ax.set_autoscale_on(False)
        self._ax.set_facecolor(color)
        if axis:
            self._ax.axvline(0, color="lightgray", ls="--")
            self._ax.axhline(0, color="lightgray", ls="--")
        else:
            self._ax.xaxis.set_visible(False)
            self._ax.yaxis.set_visible(False)
        self._ax.set_xlim([-lim, lim])
        self._ax.set_ylim([-lim, lim])
        if scale_div is not None:
            self._ax.text(self._ax.get_xlim()[0], self._ax.get_ylim()[1],
                          f"scale=1/{scale_div} mm", fontsize=10.0, color='red')

    def flip(self, fmt):
        self._fig.canvas.draw()
        img = np.frombuffer(self._fig.canvas.tostring_rgb(), dtype=np.uint8)
        img = img.reshape(self._fig.canvas.get_width_height()[::-1] + (3,))
        self._image = cv2.cvtColor(img, fmt)
        self._updated = True
        return self._image

    def get_image(self):
        return self._image

    def get_ax(self):
        return self._ax

    def is_updated(self):
        return self._updated

    def clear_updated(self):
        self._updated = False
