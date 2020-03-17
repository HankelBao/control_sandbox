"""

This module enables Genetic Algorithm on tracks for racing.

Copyright (C) 2020, Hankel Bao <hankelbao@outlook.com>

"""
from control_utilities.track import Track
from control_utilities.path import Path
from control_utilities.segmentation import Segmentations
from matplotlib import pyplot as plt
import numpy as np
import random
import heapq
import math
from copy import deepcopy
import pychrono as chrono


class TrackPath(Path):
    def __init__(self, segmentation: Segmentations, a):
        self.size = segmentation.size
        self.a = a

        points = segmentation.get_points(a)
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

class GAConfig():
    def __init__(self, segmentation):
        self.segmentation_size = segmentation.size
        self.initial_a = np.full(segmentation.size, 0.5)
        self.a_min = np.full(segmentation.size, 0.1)
        self.a_max = np.full(segmentation.size, 0.9)

        self.population = int(segmentation.size*5)

        self.c = 0.4
        self.gc = 0.4
        self.gmu = 0.4
        self.pmu = 0.4
        self.smu = 0.4
        self.cmu = 0.4

        self.mutation_range = np.full(self.segmentation_size, 0.5)
        self.safe_boundary = 0.3
        self.stablized_generation = 5


class GAPathGenerator:
    def __init__(self, segmentation, config):
        self.segmentation = segmentation
        self.s_size = segmentation.size
        self.path = []
        self.selection_p = []

        self.config = config
        self.a_min = config.a_min
        self.a_max = config.a_max

        self.best_path = None
        self.stablized_adaptability = 0.0
        self.stablized_generation = 0

        self.init_generation()
        self.generation = 0

    def update_selection_p(self):
        adaptability_sum = 0
        for i in range(len(self.path)):
            adaptability_sum += self.path[i].adaptability

        self.selection_p = []
        for i in range(len(self.path)):
            self.selection_p.append(self.path[i].adaptability/adaptability_sum)

    def init_generation(self):
        for _ in range(self.config.population-1):
            a = np.random.uniform(self.a_min, self.a_max)
            self.path.append(TrackPath(self.segmentation, a))

    def next_generation(self):
        population = self.config.population
        for _ in range(int(population*self.config.c)):
            self.cross()

        for _ in range(int(population*self.config.gc)):
            self.gcross()

        for _ in range(int(population*self.config.smu)):
            self.selected_mutation()

        for _ in range(int(population*self.config.pmu)):
            self.peek_mutation()

        for _ in range(int(population*self.config.gmu)):
            self.general_mutation()

        for _ in range(int(population*self.config.cmu)):
            self.chain_mutation()

    def rws(self):
        sum = 0
        rand = random.uniform(0, 1)

        # Note: It cannot be path here!!
        for i in range(len(self.selection_p)):
            sum += self.selection_p[i]
            if sum >= rand:
                return i
        return len(self.selection_p)-1

    def local_rws(self, path):
        rand = random.uniform(0, 1)
        adapt_sum = np.sum(path.point_adapt)
        sum = 0.0
        for i in range(len(path.point_adapt)):
            sum += path.point_adapt[i]/adapt_sum
            if sum >= rand:
                return i
        return len(path.point_adapt)-1

    def cross(self):
        path1 = self.path[self.rws()]
        path2 = self.path[self.rws()]

        a = []
        for i in range(self.s_size):
            if path1.point_adapt[i] > path2.point_adapt[i]:
                a.append(path1.a[i])
            else:
                a.append(path2.a[i])

        a = np.array(a)
        self.path.append(TrackPath(self.segmentation, a))

    def gcross(self):
        path1 = self.path[self.rws()]
        path2 = self.path[self.rws()]

        index = random.randint(1, self.s_size-1)
        a = np.append(path1.a[:index], path2.a[index:])
        self.path.append(TrackPath(self.segmentation, a))

    def selected_mutation(self):
        """
        Make small tweaks to bad points
        Should be used in final stages
        """
        path = self.path[self.rws()]
        a = deepcopy(path.a)

        index = self.local_rws(path)
        a[index] = np.clip(np.random.normal(a[index], (self.a_max[index]-self.a_min[index])/4), self.a_min[index], self.a_max[index])#np.random.uniform(0, 1)
        self.path.append(TrackPath(self.segmentation, a))

    def peek_mutation(self):
        """
        Optimize the worst point
        Accelerate the finding process
        """
        path = self.path[self.rws()]
        a = deepcopy(path.a)
        worst_a = np.min(path.point_adapt)

        places = np.where(path.point_adapt == worst_a)
        for index in places:
            a[index] = np.random.uniform(self.a_min[index], self.a_max[index])

        self.path.append(TrackPath(self.segmentation, a))

    def chain_mutation(self):
        """
        Optimize the worst point and points around it
        """
        path = self.path[self.rws()]
        a = deepcopy(path.a)
        worst_a = np.min(path.point_adapt)

        places = np.where(path.point_adapt == worst_a)
        for index in places:
            a[index] = np.random.uniform(self.a_min[index], self.a_max[index])
            influence_range = 1
            for i in range(influence_range):
                try:
                    a[index-i] = np.random.uniform(self.a_min[index-i], self.a_max[index-i])
                    a[index+i] = np.random.uniform(self.a_min[index-i], self.a_max[index+i])
                except:
                    pass

        self.path.append(TrackPath(self.segmentation, a))

    def general_mutation(self):
        """
        Randomly modify bad points
        """
        path = self.path[self.rws()]
        a = deepcopy(path.a)
        index = self.local_rws(path)
        a[index] = np.random.uniform(self.a_min[index], self.a_max[index])
        self.path.append(TrackPath(self.segmentation, a))

    def die(self):
        adapt = np.array([self.path[i].adaptability for i in range(len(self.path))])

        old_path = self.path
        self.path = []

        # Third quartile (Q3) 
        Q3 = np.percentile(adapt, 75)
        for path in old_path:
            if path.adaptability >= Q3:
                self.path.append(path)

        # smallest = heapq.nsmallest(num_delete, self.path, key=lambda k: k.adaptability)
        # for path in smallest:
        #     self.path.remove(path)

        left_len = len(self.path)
        while len(self.path) < self.config.population:
            for i in range(left_len):
                self.path.append(self.path[i])

    def ga_advance(self):
        self.next_generation()
        self.generation += 1

        self.die()
        self.update_selection_p()
        self.best_path = heapq.nlargest(1, self.path, key=lambda k: k.adaptability)[0]

        if self.stablized_adaptability == self.best_path.adaptability:
            self.stablized_generation += 1
        else:
            self.stablized_adaptability = self.best_path.adaptability
            self.stablized_generation = 0

        adjusted_population = int(self.segmentation.size * 2 * (1.3 ** self.stablized_generation))
        if adjusted_population > self.segmentation.size * 8:
            adjusted_population = self.segmentation.size * 8
        self.config.population = adjusted_population

    def plot_best_path(self):
        self.best_path.plot_path()
