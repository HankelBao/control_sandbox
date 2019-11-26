import random
import math
import sys
import numpy as np

from scipy.interpolate import splprep, splev

class Path:
    def __init__(self, points):
        tck, u = splprep(np.array(points).T, u=None, k=3, s=10.0, per=1)
        u_new = np.linspace(u.min(), u.max(), 1000)
        self.x, self.y = splev(u_new, tck, der=0)
        self.dx, self.dy = splev(u_new, tck, der=1)
        self.ddx, self.ddy = splev(u_new, tck, der=2)
        self.k = self.curvature(self.dx, self.dy, self.ddx, self.ddy)
        self.s = self.distance(self.x, self.y)
        self.last_index = 0
        self.last_dist = 0
        self.track_length = self.s[-1]
        self.times_looped = 0
        self.length = len(self)

    def curvature(self, dx, dy, ddx, ddy):
        """
        Compute curvature at one point given first and second derivatives.
        """
        return (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** (3 / 2)

    def distance(self, x, y):
        """
        Compute distance from beginning given two points.
        """
        return np.cumsum(np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2))

    def get_arc_length(self, x, y, n=5):
        besti = self.last_index
        bestd = self.distance([x, self.x[self.last_index]],[y, self.y[self.last_index]])
        for i in range(max(0, self.last_index-n), self.last_index+n):
            if i >= self.length-1:
                ii = i-self.length
            else:
                ii = i
            temp = self.distance([x, self.x[ii]],[y, self.y[ii]])
            if temp < bestd:
                bestd = temp
                besti = ii

        self.last_index = besti
        if self.last_dist > self.s[besti]:
            self.times_looped += 1
        self.last_dist = self.s[besti]
        return self.s[besti] + self.track_length*self.times_looped, besti

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

    def __len__(self):
        return len(self.x)


class RandomPathGenerator:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.max_distance = 1000
        self.isLooping = True
        self.maxDisplacement = 10
        self.minDistance = 20
        self.difficulty = 5
        self.pushIterations = 3
        self.angle = 110

    def generatePath(self, seed=1.0, reversed=0):
        self.seed = seed
        random.seed(self.seed)
        self.points = []
        self.hull = []
        self.generatePoints()
        self.computeConvexHull(self.points)
        self.pushApart()
        self.displace()
        self.fixAngles()
        # self.normalizeSize()
        self.hull.insert(0, self.hull[len(self.hull) - 1])
        if reversed:
            self.hull.reverse()
        return self.hull

    def pushApart(self, iterations=3):
        for _ in range(iterations):
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

    def fixAngles(self, iterations=10):
        for _ in range(iterations):
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
