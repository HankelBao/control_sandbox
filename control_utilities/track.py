from control_utilities.path import Path, RandomPathGenerator
import numpy as np
import sys

class Track:
    def __init__(self, points, thickness=5):
        self.points = points
        self.center = Path(points)
        self.thickness = thickness

    def generateTrack(self):
        left, right = [], []
        i = 0
        for t in self.center.s:
            ix, iy = self.center.x[i], self.center.y[i]
            dx, dy = self.center.dx[i], self.center.dy[i]
            len = np.linalg.norm(np.array([dx,dy]))
            dx, dy = dx / len, dy / len
            dx, dy = dx * self.thickness, dy * self.thickness
            left.append([ix-dy, iy+dx])
            right.append([ix+dy, iy-dx])
            i+=1
        if left[0] != left[-1] and right[0] != right[-1]:
            left.append(left[0])
            right.append(right[0])
        self.left = Path(left)
        self.right = Path(right)

    def plot(self, show=True):
        import matplotlib.pyplot as plt

        # from scipy.interpolate import interp1d
        # m = interp1d([min(abs(self.center.k)), max(abs(self.center.k))],[0,1])
        # color = [str(m(abs(k))) for k in self.center.k]
        # plt.scatter(self.center.x, self.center.y, c=color, cmap='Blues')
        # color = [str((abs(s/m))) for s in self.center.s]
        # plt.scatter(self.center.x[:-1], self.center.y[:-1], c=color, cmap='Reads')
        plt.plot(self.center.x, self.center.y, '-r')
        plt.plot(self.left.x, self.left.y)
        plt.plot(self.right.x, self.right.y)
        if show:
            plt.show()


class RandomTrack:
    def __init__(self, width=100, height=100, thickness=2):
        self.width = width
        self.height = height
        self.thickness = thickness

    def generateTrack(self, seed=1.0, reversed=0):
        self.createCenterline(seed,reversed)

    def createCenterline(self, seed=1.0, reversed=0):
        self.generator = RandomPathGenerator(
            width=self.width, height=self.width)
        self.center = Path(self.generator.generatePath(seed=seed,reversed=reversed))
        left, right = [], []
        i = 0
        for t in self.center.s:
            ix, iy = self.center.x[i], self.center.y[i]
            dx, dy = self.center.dx[i], self.center.dy[i]
            len = np.linalg.norm(np.array([dx,dy]))
            dx, dy = dx / len, dy / len
            dx, dy = dx * self.thickness, dy * self.thickness
            left.append([ix-dy, iy+dx])
            right.append([ix+dy, iy-dx])
            i+=1
        left.append(left[0])
        right.append(right[0])
        self.left = Path(left)
        self.right = Path(right)

    def plot(self, seed=1.0, show=True):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Slider

        plot_ax = plt.axes([0.1, 0.2, 0.8, 0.75])
        seed_axes = plt.axes([0.1, 0.05, 0.8, 0.05])
        seed_slider = Slider(
            seed_axes, "Seed", 0, 100, valinit=int(seed), valstep=1
        )
        plt.sca(plot_ax)

        def update(val):
            self.generateTrack(seed=val)
            plt.cla()
            # from scipy.interpolate import interp1d
            # m = interp1d([min(abs(self.center.k)), max(abs(self.center.k))],[0,1])
            # color = [str(m(abs(k))) for k in self.center.k]
            # plt.scatter(self.center.x, self.center.y, c=color, cmap='Blues')
            # m = max(self.center.s)
            # color = [str((abs(s/m))) for s in self.center.s]
            # plt.scatter(self.center.x[:-1], self.center.y[:-1], c=color, cmap='Blues')
            plt.plot(self.center.x, self.center.y)
            plt.plot(self.left.x, self.left.y)
            plt.plot(self.right.x, self.right.y)

        update(seed)

        seed_slider.on_changed(update)
        if show:
            plt.show()
