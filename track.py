from path import Path, RandomPathGenerator
import numpy as np
import sys

class Track:
    def __init__(self, path):
        self.center = path


class RandomTrack:
    def __init__(self, width=1000, height=1000, thickness=5):
        self.width = width
        self.height = height
        self.thickness = thickness

    def generateTrack(self, seed=1.0):
        self.createCenterline(seed)

    def createCenterline(self, seed=1.0):
        self.generator = RandomPathGenerator(width=self.width, height=self.width)
        self.center = Path(self.generator.generatePath(seed=seed))
        self.left = Path(self.generator.generatePath(seed=seed, scale=1.1))
        self.right = Path(self.generator.generatePath(seed=seed, scale=.90))


def plot(track, seed=1.0):
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider

    plot_ax = plt.axes([0.1, 0.2, 0.8, 0.75])
    seed_axes = plt.axes([0.1, 0.05, 0.8, 0.05])
    seed_slider = Slider(
        seed_axes, "Seed", 1, 100, valinit=int(seed), valstep=1
    )
    plt.sca(plot_ax)

    def update(val):
        track.generateTrack(seed=val)
        plt.cla()
        # color = [str((abs(k/255.))) for k in track.center.k]
        # plt.scatter(track.center.x, track.center.y, c=color, cmap='Blues')
        # m = max(track.center.s)
        # color = [str((abs(s/m))) for s in track.center.s]
        # plt.scatter(track.center.x, track.center.y, c=color, cmap='Blues')
        plt.plot(track.center.x, track.center.y)
        plt.plot(track.left.x, track.left.y)
        plt.plot(track.right.x, track.right.y)

    update(seed)

    seed_slider.on_changed(update)
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) == 2:
        seed = int(sys.argv[1])
    else:
        seed = 1.0

    track = RandomTrack()
    plot(track, seed)
    points = [[-2.5, 0.7], [0.0, -6], [2.5, 5],
              [5.0, 6.5], [7.5, 0.0], [3.0, 5.0], [-1.0, -2.0]]
