import random
import math
import sys
import numpy as np

from scipy.interpolate import splprep, splev

class Path:
    def __init__(self, points):
        tck, u = splprep(np.array(points).T, u=None, s=4.0, per=1)
        u_new = np.linspace(u.min(), u.max(), 1000)
        self.x, self.y = splev(u_new, tck, der=0)
        self.dx, self.dy = splev(u_new, tck, der=1)
        self.ddx, self.ddy = splev(u_new, tck, der=2)
        self.k = self.curvature(self.dx, self.dy, self.ddx, self.ddy)
        self.s = self.distance(self.x, self.y)
        self.last_index = 0

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
        for i in range(max(0, self.last_index-n), min(len(self)-1, self.last_index+n)):
            temp = self.distance([x, self.x[i]],[y, self.y[i]])
            if temp < bestd:
                bestd = temp
                besti = i

        self.last_index = besti
        return self.s[i], besti
        
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
        self.maxDisplacement = 1
        self.difficulty = 1e-2

    def generatePath(self, seed=1.0):
        self.seed = seed
        random.seed(self.seed)
        self.points = []
        self.hull = []
        self.generatePoints()
        self.computeConvexHull(self.points)
        self.hull.insert(0, self.hull[len(self.hull) - 1])
        return self.hull

    def generatePoints(self):
        pointCount = 20
        for i in range(pointCount):
            x = (random.random() - 0.5) * self.width
            y = (random.random() - 0.5) * self.height
            self.points.append([x, y])

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
