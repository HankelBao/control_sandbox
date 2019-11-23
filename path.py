import random
import math
import sys
import numpy as np

from spline import Spline2D


class Path:
    def __init__(self, width=100, height=100):
        self.generator = RandomPathGenerator(width, height)

    def generatePath(self, seed=1.0):
        self.hull = self.generator.generatePath(seed)

        self.spline = Spline2D([point[0] for point in self.hull], [point[1]
                                                          for point in self.hull])
        ds = .1
        s = np.arange(0, self.spline.s[-3], ds)

        rx, ry, ryaw, rk = [], [], [], []
        for i_s in s:
            ix, iy = self.spline.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(self.spline.calc_yaw(i_s))
            rk.append(self.spline.calc_curvature(i_s))

        self.x = rx
        self.y = ry
        self.yaw = ryaw
        self.rk = rk
        self.prev_ind = 0

    def calc_nearest_index(self, state):

        search_space = 10
        dx = [
            state.x - icx for icx in self.x[self.prev_ind:(self.prev_ind + search_space)]]
        dy = [
            state.y - icy for icy in self.y[self.prev_ind:(self.prev_ind + search_space)]]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind) + self.prev_ind

        dxl = self.x[ind] - state.x
        dyl = self.y[ind] - state.y

        if self.prev_ind >= ind:
            ind = self.prev_ind

        self.prev_ind = ind

        return ind

    def getInitLoc(self, i=0):
        return [self.hull[i][0], self.hull[i][1]]

    def getInitRot(self, i=0):
        v1 = [self.hull[i + 1][0] - self.hull[i][0],
              self.hull[i + 1][1] - self.hull[i][1]]
        v2 = [-1, 0]
        ang = math.acos(np.dot(v1, v2) /
                        (np.linalg.norm(v1) * np.linalg.norm(v2)))
        return ang + np.deg2rad(180)

    def __getitem__(self, i):
        return self.x[i], self.y[i]


class RandomPathGenerator:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.max_distance = 1000
        self.isLooping = True
        self.maxDisplacement = .1
        self.difficulty = 1e-2

    def generatePath(self, seed):
        self.seed = seed
        random.seed(self.seed)
        self.points = []
        self.hull = []
        self.generatePoints()
        self.computeConvexHull(self.points)
        # self.displace()
        self.hull.insert(0, self.hull[len(self.hull) - 1])
        self.hull.append(self.hull[1])
        self.hull.append(self.hull[2])
        return self.hull

    def generatePoints(self):
        pointCount = 20
        for i in range(pointCount):
            x = (random.random() - 0.5) * self.width
            y = (random.random() - 0.5) * self.height
            self.points.append([x, y])

    def displace(self):
        newHull = []
        for i in range(len(self.hull)):
            dispLen = (random.random() ** self.difficulty) * self.maxDisplacement
            disp = [0, 1]
            disp = self.rotateVector(disp, random.random() * 360)
            disp = [dispLen * disp[0], dispLen * disp[1]]
            newHull.append(self.hull[i])
            point = self.hull[i]
            point2 = self.hull[(i + 1) % len(self.hull)]
            x = (point[0] + point2[0]) / 2 + disp[0]
            y = (point[1] + point2[1]) / 2 + disp[1]
            newHull.append([x, y])

        self.hull = newHull

    def rotateVector(self, v, degrees):
        radians = degrees * (math.pi / 180)
        sin = math.sin(radians)
        cos = math.cos(radians)

        tx = v[0]
        ty = v[1]

        return [cos * tx - sin * ty, sin * tx + cos * ty]

    def computeConvexHull(self, points):
        points.sort()
        upper = points[0:2]		# Initialize upper part
        # Compute the upper part of the hull
        for i in range(2, len(points)):
            upper.append(points[i])
            while len(upper) > 2 and self.isLeft(upper[-1], upper[-2], upper[-3]):
                del upper[-2]

        lower = list(reversed(points[-2:]))  # Initialize the lower part
        # Compute the lower part of the hull
        for i in range(len(points) - 3, -1, -1):
            lower.append(points[i])
            while len(lower) > 2 and self.isLeft(lower[-1], lower[-2], lower[-3]):
                del lower[-2]
        del lower[0]
        del lower[-1]
        self.hull = upper + lower		# Build the full hull

    def isLeft(self, p1, p2, p3):
        return (p3[1] - p1[1]) * (p2[0] - p1[0]) < (p2[1] - p1[1]) * (p3[0] - p1[0])

    def GetPath(self, z=0.5):
        hull = []
        for point in self.hull:
            point.append(z)
            hull.append(point)
        return hull


def plot(path):
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider

    generator = path.generator

    plot_ax = plt.axes([0.1, 0.2, 0.8, 0.75])
    seed_axes = plt.axes([0.1, 0.05, 0.8, 0.05])
    seed_slider = Slider(
        seed_axes, "Seed", 1, 100, valinit=int(generator.seed), valstep=1
    )
    plt.sca(plot_ax)

    def update(val):
        path.generatePath(seed=seed_slider.val)
        plt.cla()
        x = [point[0] for point in generator.hull]
        y = [point[1] for point in generator.hull]
        xx = [point[0] for point in generator.points]
        yy = [point[1] for point in generator.points]
        plt.plot(x, y)
        plt.scatter(xx, yy)

        plt.plot(path.x, path.y)
        x = [point[0] for point in path.hull]
        y = [point[1] for point in path.hull]
        plt.plot(x, y, ".r")

    update(generator.seed)

    seed_slider.on_changed(update)
    plt.show()


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    if len(sys.argv) == 2:
        seed = int(sys.argv[1])
    else:
        seed = 1

    path = Path()
    path.generatePath(seed)
    plot(path)
