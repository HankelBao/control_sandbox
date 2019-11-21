import random
import math
from functools import cmp_to_key
import sys
import numpy as np

from spline import Spline2D

class Track:
    def __init__(self):
        self.generator = RandomTrackGenerator(100,100,2,1,50)
        self.generateTrack()

    def generateTrack(self, seed=90.0):
        self.hull = self.generator.generateTrack(seed)

        sp = Spline2D([point[0] for point in self.hull], [point[1] for point in self.hull])
        ds = .1
        s = np.arange(0, sp.s[-3], ds)

        rx, ry, ryaw, rk = [], [], [], []
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(sp.calc_yaw(i_s))
            rk.append(sp.calc_curvature(i_s))

        self.x = rx
        self.y = ry
        self.yaw = ryaw
        self.rk = rk
        self.prev_ind = 0

    def calc_nearest_index(self, state):

        search_space = 10
        dx = [state.x - icx for icx in self.x[self.prev_ind:(self.prev_ind + search_space)]]
        dy = [state.y - icy for icy in self.y[self.prev_ind:(self.prev_ind + search_space)]]

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
        v1 = [self.hull[i+1][0] - self.hull[i][0], self.hull[i+1][1] - self.hull[i][1]]
        v2 = [-1,0]
        ang = math.acos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
        return ang + np.deg2rad(180)

    def __getitem__(self, i):
        return self.x[i], self.y[i]

    def plot(self):
        self.generator.plot()

        import matplotlib.pyplot as plt

        plt.plot(self.x, self.y)
        x = [point[0] for point in self.hull]
        y = [point[1] for point in self.hull]
        print(x[0])
        plt.plot(x, y,".r")
        # plt.show()



class RandomTrackGenerator:
    def __init__(self, width, height, maxDisplacement, steps, difficulty):
        self.width = width
        self.height = height
        self.maxDisplacement = maxDisplacement
        self.pushIterations = 3
        self.minDistance = 1
        self.angle = 100
        self.isLooping = True
        self.steps = steps
        self.difficulty = difficulty

    def generateTrack(self, seed):
        self.seed = seed
        random.seed(self.seed)
        self.points = []
        self.hull = []
        self.generatePoints()
        self.computeConvexHull(self.points)

        for i in range(self.pushIterations):
            self.pushApart()

        self.displace()
        for i in range(10):
            self.fixAngles()
            self.pushApart()
        self.normalizeSize()
        self.hull.insert(0,self.hull[len(self.hull)-1])
        self.hull.append(self.hull[1])
        self.hull.append(self.hull[2])
        return self.hull

    def generatePoints(self):
        pointCount = 20
        for i in range(pointCount):
            x = (random.random() - 0.5) * self.width
            y = (random.random() - 0.5) * self.height
            self.points.append([x, y])

    def pushApart(self):
        for i in range(len(self.hull)):
            for j in range(len(self.hull)):
                hl = (
                    (self.hull[i][0] - self.hull[j][0]) ** 2
                    + (self.hull[i][1] - self.hull[j][1]) ** 2
                ) ** 1 / 2
                if hl == 0:
                    hl = 0.1

                if hl < self.minDistance:
                    hx = self.hull[j][0] - self.hull[i][0]
                    hy = self.hull[j][1] - self.hull[i][1]
                    hx /= hl
                    hy /= hl
                    diff = self.minDistance - hl
                    hx *= diff
                    hy *= diff
                    self.hull[j][0] += hx
                    self.hull[j][1] += hy
                    self.hull[i][0] -= hx
                    self.hull[i][1] -= hy

    def normalizeSize(self):
        maxX = 0
        maxY = 0
        for i in range(len(self.hull)):
            if abs(self.hull[i][0]) > maxX:
                maxX = abs(self.hull[i][0])

            if abs(self.hull[i][1]) > maxY:
                maxY = abs(self.hull[i][1])

        for i in range(len(self.hull)):
            self.hull[i][0] = (self.hull[i][0] / maxX) * self.width / 2
            self.hull[i][1] = (self.hull[i][1] / maxY) * self.height / 2

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
        # push apart again, so we can stabilize the points distances.
        for i in range(self.pushIterations):
            self.pushApart()

    def rotateVector(self, v, degrees):
        radians = degrees * (math.pi / 180)
        sin = math.sin(radians)
        cos = math.cos(radians)

        tx = v[0]
        ty = v[1]

        return [cos * tx - sin * ty, sin * tx + cos * ty]

    def fixAngles(self):
        for i in range(len(self.hull)):
            previous = len(self.hull) - 1 if i - 1 < 0 else i - 1
            next = (i + 1) % len(self.hull)
            px = self.hull[i][0] - self.hull[previous][0]
            py = self.hull[i][1] - self.hull[previous][1]
            pl = (px * px + py * py) ** 1 / 2
            px /= pl
            py /= pl

            nx = self.hull[i][0] - self.hull[next][0]
            ny = self.hull[i][1] - self.hull[next][1]
            nx = -nx
            ny = -ny
            nl = (nx * nx + ny * ny) ** 1 / 2
            nx /= nl
            ny /= nl
            # I got a vector going to the next and to the previous points, normalised.

            a = math.atan2(
                px * ny - py * nx, px * nx + py * ny
            )  # perp dot product between the previous and next point.

            if abs(a * (180 / math.pi)) <= self.angle:
                continue

            nA = self.angle * self.sign(a) * (math.pi / 180)
            diff = nA - a
            cos = math.cos(diff)
            sin = math.sin(diff)
            newX = nx * cos - ny * sin
            newY = nx * sin + ny * cos
            newX *= nl
            newY *= nl
            self.hull[next][0] = self.hull[i][0] + newX
            self.hull[next][1] = self.hull[i][1] + newY
            # I got the difference between the current angle and 100degrees, and built a new vector that puts the next point at 100 degrees.

    def sign(self, num):
        return math.copysign(1, num)

    def computeConvexHull(self, points):
        def sort_fun(a, b):
            if a[0] == b[0]:
                return a[1] - b[1]
            elif a[0] > b[0]:
                return 1
            else:
                return -1

        points.sort(key=cmp_to_key(sort_fun))


        L_upper = [points[0], points[1]]		# Initialize upper part
        # Compute the upper part of the hull
        for i in range(2,len(points)):
            L_upper.append(points[i])
            while len(L_upper) > 2 and not self.cw(L_upper[-1],L_upper[-2],L_upper[-3]):
                del L_upper[-2]

        L_lower = [points[-1], points[-2]]	# Initialize the lower part
        # Compute the lower part of the hull
        for i in range(len(points)-3,-1,-1):
            L_lower.append(points[i])
            while len(L_lower) > 2 and not self.cw(L_lower[-1],L_lower[-2],L_lower[-3]):
                del L_lower[-2]
        del L_lower[0]
        del L_lower[-1]
        self.hull = L_upper + L_lower		# Build the full hull

    def cw(self, p1, p2, p3):
        if (p3[1]-p1[1])*(p2[0]-p1[0]) >= (p2[1]-p1[1])*(p3[0]-p1[0]):
            return False
        return True

    def GetTrack(self, z=0.5):
        hull = []
        for point in self.hull:
            point.append(z)
            hull.append(point)
        return hull

    def plot(self):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Slider

        plot_ax = plt.axes([0.1, 0.3, 0.8, 0.65])
        seed_axes = plt.axes([0.1, 0.15, 0.8, 0.05])
        diff_axes = plt.axes([0.1, 0.10, 0.8, 0.05])
        displ_axes = plt.axes([0.1, 0.05, 0.8, 0.05])
        step_axes = plt.axes([0.1, 0.0, 0.8, 0.05])
        seed_slider = Slider(
            seed_axes, "Seed", 1, 100, valinit=int(self.seed), valstep=1
        )
        diff_slider = Slider(
            diff_axes, "Diff", 1, 1000, valinit=self.difficulty, valstep=1
        )
        displ_slider = Slider(
            displ_axes, "Displ", 1, 100, valinit=self.maxDisplacement, valstep=1
        )
        step_slider = Slider(step_axes, "Step", 1, 100, valinit=self.steps, valstep=1)

        plt.sca(plot_ax)
        x = [point[0] for point in self.hull]
        y = [point[1] for point in self.hull]
        xx = [point[0] for point in self.points]
        yy = [point[1] for point in self.points]
        plt.plot(x, y)
        plt.scatter(xx,yy)

        def update(val):
            self.generateTrack(seed=seed_slider.val)
            self.difficulty = diff_slider.val
            self.maxDisplacement = displ_slider.val
            self.steps = step_slider.val
            plt.cla()
            x = [point[0] for point in self.hull]
            y = [point[1] for point in self.hull]
            xx = [point[0] for point in self.points]
            yy = [point[1] for point in self.points]
            plt.plot(x, y)
            plt.scatter(xx,yy)

        seed_slider.on_changed(update)
        diff_slider.on_changed(update)
        displ_slider.on_changed(update)
        step_slider.on_changed(update)

if __name__ == "__main__":
    # if len(sys.argv) == 2:
    #     seed = int(sys.argv[1])
    # else:
    #     seed = 1
    # random_track_generator = RandomTrackGenerator(
    #     width=100, height=100, maxDisplacement=2, steps=1,
    # )
    # random_track_generator.generateTrack(
    #     difficulty=50, seed=seed,
    # )
    # random_track_generator.plot()

    track = Track()
    track.plot()
