from control_utilities.track import Track
from control_utilities.path import Path
from matplotlib import pyplot as plt
import numpy as np
import random
import heapq
import math
from copy import deepcopy
import pychrono as chrono

class Segmentations:
    def __init__(self, track, k_precision=0.800):
        self.left = []
        self.right = []
        self.width = []
        self.track = track
        self.size = 0
        self.precision = k_precision

    def create_segmentations(self):
        k_sum = 0.0

        right_points = []
        left_points = []
        width = []
        for i in range(self.track.center.length):
            k_sum += np.abs(self.track.center.k[i])
            if k_sum >= self.precision:
                k_sum = 0

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

class Cell:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.fCost = 0.0

class RAStar:
    def __init__(self, segmentation, neightbors_ratio=0.2, divisions=10):
        # f(x) = g(x) + h(x)
        self.segmentation = segmentation
        self.neightbors = int(divisions*neightbors_ratio)
        self.divisions = divisions
        self.width = self.segmentation.size
        self.height = self.divisions

        self.t_break = 1+1/(self.width + self.height)
        self.g_score = np.full([self.width, self.height], np.inf)
        self.division_val = 1/divisions

    def find_a(self, start_a):
        start_y = int(start_a*self.divisions)
        self.start_y = start_y

        self.g_score[0][start_y] = 0
        currentCell = Cell(0, start_y)
        currentCell.fCost = self.g_score[0][start_y] + self.t_break*self.calc_heuristic(0, start_y)
        opl = [ currentCell ]

        while (np.min(self.g_score[self.width-1]) == np.inf):
            cof_cell = heapq.nsmallest(1, opl, key=lambda k: k.fCost)[0]
            opl.remove(cof_cell)

            for cell in self.get_neighbours(cof_cell):
                if self.g_score[cell.x][cell.y] == np.inf:
                    self.g_score[cell.x][cell.y] = self.calc_gcost(cof_cell.x, cof_cell.y, cell.x, cell.y)
                    cell.fCost = self.g_score[cell.x][cell.y] + self.t_break*self.calc_heuristic(cell.x, cell.y)

                    if not any((cell.x == existing_cell.x and cell.y == existing_cell.y) for existing_cell in opl):
                        opl.append(cell)

            # print(self.g_score)

        currentCell = Cell(0, start_y)
        a = [start_y]
        while currentCell.x != self.width-1:
            neighbours = self.get_neighbours(currentCell)
            best_next = heapq.nsmallest(1, neighbours, key=lambda k:self.g_score[k.x][k.y])[0]
            currentCell = best_next
            a.append(currentCell.y)

        a = np.array(a)
        return a/self.divisions

    def get_neighbours(self, cell):
        y_min = max(0, cell.y-self.neightbors)
        y_max = min(self.divisions, cell.y+self.neightbors)

        neighbours = []
        for i in range(y_min, y_max):
            neighbours.append(Cell(cell.x+1, i))
        return neighbours

    def calc_gcost(self, x1, y1, x2, y2):
        (px1, py1) = self.segmentation.get_point(x1, y1*self.division_val)
        (px2, py2) = self.segmentation.get_point(x2, y2*self.division_val)
        return self.g_score[x1][y1] + np.sqrt(np.square(px2-px1)+np.square(py2-py1))

    def calc_heuristic(self, x, y):
        # return np.abs(self.width-x) + np.abs(self.start_y-y)
        return self.g_score[x][y]/x*(self.width-x)

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


class GAConfig():
    def __init__(self, segmentation):
        self.segmentation_size = segmentation.size
        self.initial_a = np.full(segmentation.size, 0.5)
        self.a_min = np.full(segmentation.size, 0.1)
        self.a_max = np.full(segmentation.size, 0.9)

        self.preserve_a = False
        self.population = 50

        self.c = 0.4
        self.gc = 0.4*0.1
        self.gmu = 0.4*0.4
        self.pmu = 0.4*0.2
        self.smu = 0.4*0.4

        self.mutation_range = np.full(self.segmentation_size, 0.5)
        self.safe_boundary = 0.3
        self.stablized_generation = 6


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

    def update_selection_p(self):
        adaptability_sum = 0
        for i in range(len(self.path)):
            adaptability_sum += self.path[i].adaptability

        self.selection_p = []
        for i in range(len(self.path)):
            self.selection_p.append(self.path[i].adaptability/adaptability_sum)

    def init_generation(self):
        if self.config.preserve_a:
            self.path.append(TrackPath(self.segmentation, self.config.initial_a))

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
        influence_range = 0
        for i in range(influence_range):
            try:
                a[index-i] = np.random.uniform(self.a_min[index-i], self.a_max[index-i])
                a[index+i] = np.random.uniform(self.a_min[index-i], self.a_max[index+i])
            except:
                pass

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
            influence_range = 0
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
        num_delete = len(self.path) - self.config.population
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
