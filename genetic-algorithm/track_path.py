import control_utilities.track
from matplotlib import pyplot as plt
import numpy as np
import random
import heapq
import math
from copy import deepcopy
import pychrono as chrono

class TrackPath:
    def __init__(self, track, a):
        self.track = track
        self.size = track.center.x.size-1
        self.a = a

        self.create_path()
        self.create_curvature()
        self.create_distance()
        self.create_angle()

        self.point_adapt = 1/self.c
        # self.point_adapt = 1/self.c+self.angle
        self.adaptability = 1/np.average(np.power(self.c, 1))-1/np.max(self.c)+np.average(np.power(self.angle*2, 1))+np.min(self.angle)*2
        # self.adaptability = 1/np.average(self.c) + np.average(np.power(self.angle, 1.5))

    def average_curvature(self):
        return np.average(self.c)

    def average_angle(self):
        return np.average(self.angle)

    def average_distance(self):
        return np.average(self.distance)

    def create_curvature(self):
        self.c = np.full(self.size, 0.0)
        for i in range(self.size):
            self.c[i] = np.abs(self.__calc_curvature(i))

    def create_distance(self):
        self.distance = np.full(self.size, 0.0)
        for i in range(self.size):
            (x1, y1) = (self.x[i], self.y[i])
            (x2, y2) = self.__next_point(i)
            self.distance[i] = TrackPath.__distance(x1, y1, x2, y2)

    def create_angle(self):
        self.angle = np.full(self.size, 0.0)
        for i in range(self.size):
            self.angle[i] = np.abs(self.__calc_angle(i))

    def __calc_angle(self, n):
        (x1, y1) = self.__prev_point(n)
        (x2, y2) = (self.x[n], self.y[n])
        (x3, y3) = self.__next_point(n)

        p1 = chrono.ChVectorD(x1, y1, 0)
        p2 = chrono.ChVectorD(x2, y2, 0)
        p3 = chrono.ChVectorD(x3, y3, 0)

        v1 = p2 - p1
        v2 = p2 - p3
        ang = math.atan2((v1 % v2).Length(), v1 ^ v2)
        if chrono.ChVectorD(0, 0, 1) ^ (v1 % v2) > 0.0:
            ang *= 1
        return ang#np.arccos((np.square(p12)+np.square(p13)-np.square(p23))/(2*p12*p13))

    def __calc_curvature(self, n):
        (x1, y1) = self.__prev_point(n)
        (x2, y2) = (self.x[n], self.y[n])
        (x3, y3) = self.__next_point(n)
        area = TrackPath.__area(x1, y1, x2, y2, x3, y3)
        l1 = TrackPath.__distance(x1, y1, x2, y2)
        l2 = TrackPath.__distance(x1, y1, x3, y3)
        l3 = TrackPath.__distance(x2, y2, x3, y3)

        curvature = 0 if l1*l2*l3 == 0 else 4.0*area/(l1*l2*l3)
        return curvature

    @staticmethod
    def __area(x1, y1, x2, y2, x3, y3):
        return (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1)

    def __closest_points(self, n):
        if n==0 or n==self.size-1:
            return (self.x[self.size-2], self.y[self.size-2], self.x[1], self.y[1])
        else:
            return (self.x[n-1], self.y[n-1], self.x[n+1], self.y[n+1])

    def __next_point(self, n):
        if n==self.size-1:
            return (self.x[0], self.y[0])
        else:
            return (self.x[n+1], self.y[n+1])

    def __prev_point(self, n):
        if n==0:
            return (self.x[-1], self.y[-1])
        else:
            return (self.x[n-1], self.y[n-1])

    @staticmethod
    def __distance(x1, y1, x2, y2):
        return np.sqrt(np.square(x2-x1)+np.square(y2-y1))

    #def plot_sections(self):
    #    plt.plot([self.track.left_waypoints.x, self.track.right_waypoints.x], [self.track.left_waypoints.y, self.track.right_waypoints.y], "g-")

    def plot_path(self, color="r^-"):
        x = np.append(self.x, self.x[0])
        y = np.append(self.y, self.y[0])
        plt.plot(x, y, color)

    def create_path(self):
        self.x = np.full(self.size, 0.0)
        self.y = np.full(self.size, 0.0)
        for i in range(self.size):
            x1 = self.track.left_waypoints[i][0]
            y1 = self.track.left_waypoints[i][1]
            x2 = self.track.right_waypoints[i][0]
            y2 = self.track.right_waypoints[i][1]
            self.x[i] = x1 + (x2-x1)*self.a[i]
            self.y[i] = y1 + (y2-y1)*self.a[i]

INIT = 0
SEARCH = 1
OPTIMIZE = 2
EXTREME = 3

class GAConfig():
    def __init__(self, track):
        self.track_size = track.center.x.size-1
        self.initial_a = np.full(self.track_size, 0.5)
        self.init_config()

    def init_config(self):
        self.state = INIT

    def search_config(self):
        self.state = SEARCH
        self.population_ratio = 1
        self.cross_ratio = 0.4
        self.smutation_ratio = 0.4
        self.gmutation_ratio = 0.4
        self.copy_ratio = 0.0
        self.mutation_range = np.full(self.track_size, 0.5)
        self.safe_boundary = 0.3
        self.stablized_generation = 2

    def optimize_config(self):
        self.state = OPTIMIZE
        self.population_ratio = 5
        self.cross_ratio = 0.4
        self.smutation_ratio = 0.4
        self.gmutation_ratio = 0.2
        self.copy_ratio = 0.0
        self.mutation_range = np.full(self.track_size, 0.3)
        self.safe_boundary = 0.2
        self.stablized_generation = 6

    def extreme_config(self):
        self.state = EXTREME
        self.population_ratio = 30
        self.cross_ratio = 0.4
        self.smutation_ratio = 0.4
        self.gmutation_ratio = 0.4
        self.copy_ratio = 0.0
        self.mutation_range = np.full(self.track_size, 0.3)
        self.safe_boundary = 0.1
        self.stablized_generation = 10

    def upgrade(self):
        if self.state == INIT:
            self.search_config()
            return
        if self.state == SEARCH:
            self.optimize_config()
            return
        if self.state == OPTIMIZE:
            self.extreme_config()
            return

    def upgradable(self):
        if self.state == EXTREME:
            return False
        return True


class GAPathGenerator:
    def __init__(self, track, config):
        self.track = track
        self.track_points = self.track.center.x.size-1
        self.path = []
        self.selection_p = []

        self.population_size = int(self.track_points*config.population_ratio)
        self.cross_size = int(self.track_points*config.cross_ratio)
        self.smutation_size = int(self.track_points*config.smutation_ratio)
        self.gmutation_size = int(self.track_points*config.gmutation_ratio)
        self.copy_size = int(self.track_points*config.copy_ratio)

        self.config = config
        self.a_min = np.maximum(config.initial_a-config.mutation_range, np.full(self.track_points, config.safe_boundary))
        self.a_max = np.minimum(config.initial_a+config.mutation_range, np.full(self.track_points, 1-config.safe_boundary))

        self.best_path = None
        self.stablized_adaptability = 0.0
        self.stablized_generation = 0

        self.init_generation()

    def update_selection_p(self):
        adaptability_sum = 0
        for i in range(len(self.path)):
            adaptability_sum += self.path[i].adaptability

        self.selection_p = []
        for i in range(len(self.path)):
            self.selection_p.append(self.path[i].adaptability/adaptability_sum)

    def init_generation(self):
        for _ in range(self.population_size):
            a = np.random.uniform(self.a_min, self.a_max)
            self.path.append(TrackPath(self.track, a))

    def next_generation(self):
        for _ in range(self.cross_size):
            self.cross()

        for _ in range(self.smutation_size):
            self.selected_mutation()

        for _ in range(self.gmutation_size):
            self.general_mutation()

        self.copy(self.copy_size)

    def rws(self):
        sum = 0
        rand = random.uniform(0, 1)

        # Note: It cannot be path here!!
        for i in range(len(self.selection_p)):
            sum += self.selection_p[i]
            if sum >= rand:
                return i
        return len(self.selection_p)-1

    def cross(self):
        path1 = self.path[self.rws()]
        path2 = self.path[self.rws()]

        a = []
        for i in range(self.track_points):
            if path1.point_adapt[i] > path2.point_adapt[i]:
                a.append(path1.a[i])
            else:
                a.append(path2.a[i])

        a = np.array(a)
        self.path.append(TrackPath(self.track, a))

    def selected_mutation(self):
        path = self.path[self.rws()]
        a = deepcopy(path.a)
        worst_a = np.min(path.point_adapt)

        places = np.where(path.point_adapt == worst_a)
        for index in places:
            a[index] = np.random.uniform(0, 1)
            influence_range = random.randint(0, 3)
            for i in range(influence_range):
                try:
                    a[index-i] = np.random.uniform(self.a_min[index-i], self.a_max[index-i])
                    a[index+i] = np.random.uniform(self.a_min[index-i], self.a_max[index+i])
                except:
                    pass

        self.path.append(TrackPath(self.track, a))

    def general_mutation(self):
        path = self.path[self.rws()]
        a = deepcopy(path.a)
        index = random.randint(0, self.track_points-1)
        a[index] = np.random.uniform(0, 1)
        self.path.append(TrackPath(self.track, a))

    def copy(self, times):
        largest = heapq.nlargest(times, self.path, key=lambda k: k.adaptability)
        for path in largest:
            self.path.append(path)

    def die(self):
        num_delete = len(self.path) - self.population_size
        if num_delete <= 0:
            return
        smallest = heapq.nsmallest(num_delete, self.path, key=lambda k: k.adaptability)
        for path in smallest:
            self.path.remove(path)
        if len(self.path) == 0:
            print("Really?")

    def ga_advance(self):
        self.next_generation()
        self.die()
        self.update_selection_p()
        self.best_path = heapq.nlargest(1, self.path, key=lambda k: k.adaptability)[0]

        if self.stablized_adaptability == self.best_path.adaptability:
            self.stablized_generation += 1
        else:
            self.stablized_adaptability = self.best_path.adaptability
            self.stablized_generation = 0

    def plot_best_path(self):
        self.best_path.plot_path()

        # plt.savefig("fig"+str(generation)+".png")
        # save_unit = 10
        # if generation % save_unit == 0:
        #     plt.savefig("fig"+str(generation)+".png")
        #     if generation != 100 or generation != 1000 or generation != 10000 or generation != 100000:
        #         if generation % 1000 != 0:
        #             if int(generation/save_unit) != 1:
        #                 os.remove("fig"+str(generation-save_unit)+".png")
