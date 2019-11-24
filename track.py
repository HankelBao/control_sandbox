from path import Path, RandomPathGenerator
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
        self.left = Path(left)
        self.right = Path(right)

    def plot(self, show=True):
        import matplotlib.pyplot as plt

        # color = [str((abs(k/255.))) for k in self.center.k]
        # plt.scatter(self.center.x, self.center.y, c=color, cmap='Blues')
        # m = max(self.center.s)
        # color = [str((1 - abs(s/m))) for s in track.center.s]
        # plt.scatter(self.center.x, self.center.y, c=color, cmap='Blues')
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

    def generateTrack(self, seed=1.0):
        self.createCenterline(seed)

    def createCenterline(self, seed=1.0):
        self.generator = RandomPathGenerator(
            width=self.width, height=self.width)
        self.center = Path(self.generator.generatePath(seed=seed))
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
        self.left = Path(left)
        self.right = Path(right)

    def plot(self, seed=1.0, show=True):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Slider

        plot_ax = plt.axes([0.1, 0.2, 0.8, 0.75])
        seed_axes = plt.axes([0.1, 0.05, 0.8, 0.05])
        seed_slider = Slider(
            seed_axes, "Seed", 1, 100, valinit=int(seed), valstep=1
        )
        plt.sca(plot_ax)

        def update(val):
            self.generateTrack(seed=val)
            plt.cla()
            # color = [str((abs(k/255.))) for k in self.center.k]
            # plt.scatter(self.center.x, self.center.y, c=color, cmap='Blues')
            # m = max(self.center.s)
            # color = [str((1-abs(s/m))) for s in track.center.s]
            # plt.scatter(self.center.x, self.center.y, c=color, cmap='Blues')
            plt.plot(self.center.x, self.center.y)
            plt.plot(self.left.x, self.left.y)
            plt.plot(self.right.x, self.right.y)

        update(seed)

        seed_slider.on_changed(update)
        if show:
            plt.show()


if __name__ == "__main__":
    if len(sys.argv) == 2:
        seed = int(sys.argv[1])
    else:
        seed = 1.0

    rand_track = RandomTrack()
    rand_track.plot(seed)

    points = [[49.8, 132.9], [60.3, 129.3], [75.6, 129.0], [87.9, 131.7], [96.9, 129.6], [111.0, 120.0], [115.2, 110.7], [120.6, 96.9], [127.8, 88.5], [135.9, 77.4], [135.9, 65.1], [133.2, 51.3], [128.4, 43.2], [119.7, 36.3], [105.0, 35.7], [90.0, 36.3], [82.5, 46.2], [82.5, 63.6], [83.4, 82.2], [77.1, 93.9], [61.2, 88.5], [55.5, 73.5], [
        57.9, 54.6], [66.6, 45.0], [75.9, 36.3], [79.2, 25.5], [78.0, 13.2], [65.1, 6.0], [50.7, 6.0], [36.6, 11.7], [29.1, 21.3], [24.0, 36.9], [24.0, 56.1], [29.1, 70.8], [24.9, 77.7], [13.5, 77.7], [6.3, 81.6], [5.7, 92.7], [6.3, 107.7], [8.7, 118.2], [15.3, 122.7], [24.3, 125.4], [31.2, 126.0], [40.8, 129.6], [49.8, 132.9]]
    track = Track(points)
    track.generateTrack()
    track.plot()
