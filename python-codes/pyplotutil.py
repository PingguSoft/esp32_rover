import matplotlib.pyplot as plt
import cv2
import numpy as np


class PyplotUtil(object):
    def __init__(self, row, col, figsize):
        self._fig, self._ax = plt.subplots(row, col, figsize=figsize)
        self._fig.set_tight_layout(True)
        self._image = None
        self._updated = False

    def set(self, ax=None, title=None, lim=None, color='white', axis=True, scale_div=None):
        if ax is None:
            ax = self._ax

        ax.clear()
        if title is not None:
            ax.set_title(title)
        ax.set_autoscale_on(False)
        if lim is not None:
            ax.set_xlim([-lim, lim])
            ax.set_ylim([-lim, lim])

        if axis:
            ax.axvline(0, color="lightgray", ls="--")
            ax.axhline(0, color="lightgray", ls="--")
        else:
            ax.set_axis_off()

        ax.set_facecolor(color)
        if scale_div is not None:
            ax.text(ax.get_xlim()[0], ax.get_ylim()[1],
                          f"scale=1/{scale_div} mm", fontsize=10.0, color='red')

    def finish(self, fmt=None):
        self._fig.canvas.draw()
        img = np.frombuffer(self._fig.canvas.tostring_rgb(), dtype=np.uint8)
        self._image = img.reshape(self._fig.canvas.get_width_height()[::-1] + (3,))
        if fmt is not None:
            self._image = cv2.cvtColor(self._image, fmt)

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
