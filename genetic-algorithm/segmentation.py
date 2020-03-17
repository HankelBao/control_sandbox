import numpy as np
import matplotlib.pyplot as plt

from control_utilities.track import Track
from control_utilities.path import Path

class Segmentations:
    def __init__(self, track, k_precision=0.800):
        self.left = []
        self.right = []
        self.width = []
        self.track = track
        self.size = 0
        self.precision = k_precision

    def create_segmentations(self):
        # sk = curvature * distance
        sk_sum = 0.0

        right_points = []
        left_points = []
        width = []
        for i in range(self.track.center.length):
            distance = 0 if i == 0 else self.track.center.ps[i-1]
            k = np.abs(self.track.center.k[i])
            sk_sum += distance * k
            if sk_sum >= self.precision:
                sk_sum = 0

                left = self.track.left_waypoints[i]
                right = self.track.right_waypoints[i]
                left_points.append(left)
                right_points.append(right)

                width.append(np.sqrt(np.square(left[0]-right[0])+np.square(left[1]-right[1])))
                self.size += 1

        self.left = Path(left_points, closed=True, raw_mode=True)
        self.right = Path(right_points, closed=True, raw_mode=True)
        self.width = np.array(width)

    def get_point(self, index, a):
        x1 = self.left.x[index]
        y1 = self.left.y[index]
        x2 = self.right.x[index]
        y2 = self.right.y[index]
        x = x1 + (x2-x1)/self.width[index]*a
        y = y1 + (y2-y1)/self.width[index]*a
        return [x, y]

    def plot(self):
        for i in range(self.size):
            plt.plot([self.left.x[i], self.right.x[i]], [self.left.y[i], self.right.y[i]], "g-")

class TrackPath(Path):
    def __init__(self, segmentation: Segmentations, a):
        self.size = segmentation.size
        self.a = a

        points = []
        for i in range(segmentation.size):
            points.append(segmentation.get_point(i, a[i]))
        super().__init__(points, closed=True, raw_mode=True)

        self.adaptability = -np.sum(self.t)
        self.point_adapt = self.v_max

    def plot_path(self, show=False, color="r^-"):
        self.plot(show=show, color=color)

    def generate_final_path(self):
        points = []
        for i in range(self.size):
            points.append([self.x[i], self.y[i]])

        final_path = Path(points, num_points=500)
        points = []
        for i in range(final_path.length):
            points.append([final_path.x[i], final_path.y[i]])
        final_track_path = Path(points, raw_mode=True)
        self.final_path = final_track_path
