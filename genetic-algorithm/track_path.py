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

BRK = 10
ACC = 5
class TrackPath(Path):
    def __init__(self, segmentation: Segmentations, a):
        self.size = segmentation.size
        self.a = a

        points = []
        for i in range(segmentation.size):
            points.append(segmentation.get_point(i, a[i]))
        super().__init__(points, closed=True, raw_mode=True)

        self.update_vmax()
        self.update_profile()

        self.adaptability = -np.sum(self.t)
        self.point_adapt = 0-np.append(np.array([0]), self.t)

    def update_vmax(self):
        for i in range(self.size):
            v_max = self.v_max[i]

            s = 0
            for j in range(1, i+1):
                # Check v_max of all points before it
                index = i-j
                s += self.ps[index]
                local_v_max = np.sqrt(2*BRK*s + np.square(v_max))
                if local_v_max < self.v_max[index]:
                    self.v_max[index] = local_v_max

    def update_profile(self):
        brk_s = []
        acc_s = []
        v = []
        t = []

        current_speed = 0
        for i in range(len(self.ps)):
            v.append(current_speed)

            ps = self.ps[i]
            v_max = self.v_max[i+1]

            if current_speed <= v_max:
                top_speed = np.sqrt(2*ACC*ps + current_speed**2)
                v_end = v_max if top_speed > v_max else top_speed
                acc_distance = (v_end**2 - current_speed**2) / (2 * ACC)
                time = (top_speed-current_speed)/ACC + (ps-acc_distance)/v_end

                acc_s.append(acc_distance)
                brk_s.append(0)
                t.append(time)

                current_speed = v_end
            else:
                v_end = v_max
                brk_distance = (current_speed**2 - v_end**2) / (2 * BRK)
                time = (ps-brk_distance)/current_speed + (current_speed-v_end)/BRK

                acc_s.append(0)
                brk_s.append(brk_distance)
                t.append(time)

                current_speed = v_end

        self.brk_s = np.array(brk_s)
        self.acc_s = np.array(acc_s)
        self.v = np.array(v)
        self.t = np.array(t)

    def plot_path(self, show=False, color="r^-"):
        self.plot(show=show, color=color)

    def generate_final_path(self):
        points = []
        for i in range(self.size):
            points.append([self.x[i], self.y[i]])
        self.final_path = Path(points, num_points=500)

    def plot_speed_profile(self):
        # Speed
        plt.plot(self.s, self.v, "g-")

        # Time
        plt.plot(self.s, self.t, "y-")

        # Curvature
        # plt.plot(np.abs(self.k*100), "b-")

        # Max Speed based on global curvature
        # plt.plot(np.abs(self.v_max), "k-")

        # Throttle
        # plt.plot(np.array(self.adjust_a)*10+5, "c+")

        plt.show()

# class TrackPath:
#     def __init__(self, segmentation, a):
#         self.segmentation = segmentation
#         self.a = a
#         self.size = segmentation.size
#
#         self.create_path()
#         self.create_curvature()
#         self.create_distance()
#         self.create_angle()
#
#         self.point_adapt = 1/self.c
#         self.adaptability = 1/np.average(np.power(self.c, 2))+np.average(np.power(self.angle*2, 2))+np.min(self.angle)*2-np.average(self.distance)
#
#     def plot_path(self, color="r^-"):
#         x = np.append(self.x, self.x[0])
#         y = np.append(self.y, self.y[0])
#         plt.plot(x, y, color)
#
#     def create_curvature(self):
#         self.c = np.full(self.size, 0.0)
#         for i in range(self.size):
#             self.c[i] = np.abs(self.__calc_curvature(i))
#
#     def create_distance(self):
#         self.distance = np.full(self.size, 0.0)
#         for i in range(self.size):
#             (x1, y1) = (self.x[i], self.y[i])
#             (x2, y2) = self.__next_point(i)
#             self.distance[i] = TrackPath.__distance(x1, y1, x2, y2)
#
#     def create_angle(self):
#         self.angle = np.full(self.size, 0.0)
#         for i in range(self.size):
#             self.angle[i] = np.abs(self.__calc_angle(i))
#
#     def __calc_angle(self, n):
#         (x1, y1) = self.__prev_point(n)
#         (x2, y2) = (self.x[n], self.y[n])
#         (x3, y3) = self.__next_point(n)
#
#         p1 = chrono.ChVectorD(x1, y1, 0)
#         p2 = chrono.ChVectorD(x2, y2, 0)
#         p3 = chrono.ChVectorD(x3, y3, 0)
#
#         v1 = p2 - p1
#         v2 = p2 - p3
#         ang = math.atan2((v1 % v2).Length(), v1 ^ v2)
#         if chrono.ChVectorD(0, 0, 1) ^ (v1 % v2) > 0.0:
#             ang *= 1
#         return ang#np.arccos((np.square(p12)+np.square(p13)-np.square(p23))/(2*p12*p13))
#
#     def __calc_curvature(self, n):
#         (x1, y1) = self.__prev_point(n)
#         (x2, y2) = (self.x[n], self.y[n])
#         (x3, y3) = self.__next_point(n)
#         area = TrackPath.__area(x1, y1, x2, y2, x3, y3)
#         l1 = TrackPath.__distance(x1, y1, x2, y2)
#         l2 = TrackPath.__distance(x1, y1, x3, y3)
#         l3 = TrackPath.__distance(x2, y2, x3, y3)
#
#         curvature = 0 if l1*l2*l3 == 0 else 4.0*area/(l1*l2*l3)
#         return curvature
#
#     @staticmethod
#     def __area(x1, y1, x2, y2, x3, y3):
#         return (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1)
#
#     def __closest_points(self, n):
#         if n==0 or n==self.size-1:
#             return (self.x[self.size-2], self.y[self.size-2], self.x[1], self.y[1])
#         else:
#             return (self.x[n-1], self.y[n-1], self.x[n+1], self.y[n+1])
#
#     def __next_point(self, n):
#         if n==self.size-1:
#             return (self.x[0], self.y[0])
#         else:
#             return (self.x[n+1], self.y[n+1])
#
#     def __prev_point(self, n):
#         if n==0:
#             return (self.x[-1], self.y[-1])
#         else:
#             return (self.x[n-1], self.y[n-1])
#
#     @staticmethod
#     def __distance(x1, y1, x2, y2):
#         return np.sqrt(np.square(x2-x1)+np.square(y2-y1))
#
#     def create_path(self):
#         self.x = np.full(self.size, 0.0)
#         self.y = np.full(self.size, 0.0)
#         for i in range(self.size):
#             (self.x[i], self.y[i]) = self.segmentation.get_point(i, self.a[i])
#             # x1 = self.segmentation.left[i][0]
#             # y1 = self.segmentation.left[i][1]
#             # x2 = self.segmentation.right[i][0]
#             # y2 = self.segmentation.right[i][1]
#             # self.x[i] = x1 + (x2-x1)*self.a[i]
#             # self.y[i] = y1 + (y2-y1)*self.a[i]

INIT = 0
SEARCH = 1
OPTIMIZE = 2
EXTREME = 3

class GAConfig():
    def __init__(self, segmentation):
        self.segmentation_size = segmentation.size
        self.initial_a = np.full(segmentation.size, 0.5)
        self.a_min = np.full(segmentation.size, 0.1)
        self.a_max = np.full(segmentation.size, 0.9)
        self.init_config()

    def init_config(self):
        self.state = INIT

    def search_config(self):
        self.state = SEARCH
        self.population_ratio = 1
        self.cross_ratio = 0.4
        self.smutation_ratio = 0.4
        self.gmutation_ratio = 0.4
        self.copy_ratio = 0
        self.mutation_range = np.full(self.segmentation_size, 0.5)
        self.safe_boundary = 0.3
        self.stablized_generation = 10

    def optimize_config(self):
        self.state = OPTIMIZE
        self.population_ratio = 1
        self.cross_ratio = 0.4
        self.smutation_ratio = 0.4
        self.gmutation_ratio = 0.4
        self.copy_ratio = 0
        self.mutation_range = np.full(self.segmentation_size, 0.5)
        self.safe_boundary = 0.1
        self.stablized_generation = 40

    def extreme_config(self):
        self.state = EXTREME
        self.population_ratio = 30
        self.cross_ratio = 0.4
        self.smutation_ratio = 0.4
        self.gmutation_ratio = 0.4
        self.copy_ratio = 0.0
        self.mutation_range = np.full(self.segmentation_size, 0.3)
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
        if self.state == SEARCH:
            return False
        return True


class GAPathGenerator:
    def __init__(self, segmentation, config):
        self.segmentation = segmentation
        self.s_size = segmentation.size
        self.path = []
        self.selection_p = []

        s_size = segmentation.size
        self.population_size = int(s_size*config.population_ratio)
        self.cross_size = int(s_size*config.cross_ratio)
        self.smutation_size = int(s_size*config.smutation_ratio)
        self.gmutation_size = int(s_size*config.gmutation_ratio)
        self.copy_size = int(s_size*config.copy_ratio)

        self.config = config
        # self.a_min = np.maximum(config.initial_a-config.mutation_range, np.full(s_size, config.safe_boundary))
        # self.a_max = np.minimum(config.initial_a+config.mutation_range, np.full(s_size, 1-config.safe_boundary))
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
        # self.path.append(TrackPath(self.segmentation, self.config.initial_a))
        for _ in range(self.population_size-1):
            a = np.random.uniform(self.a_min, self.a_max)
            self.path.append(TrackPath(self.segmentation, a))

    def next_generation(self):
        for _ in range(self.cross_size):
            self.cross()

        for _ in range(int(self.cross_size*0.1)):
            self.gcross()

        for _ in range(int(self.smutation_size*1)):
            self.selected_mutation()

        for _ in range(int(self.smutation_size)):
            self.peek_mutation()

        for _ in range(self.gmutation_size):
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
        def local_rws(path):
            rand = random.uniform(0, 1)
            adapt_sum = np.sum(path.point_adapt)
            sum = 0.0
            for i in range(len(path.point_adapt)):
                sum += path.point_adapt[i]/adapt_sum
                if sum >= rand:
                    return i
            return len(path.point_adapt)-1

        path = self.path[self.rws()]
        a = deepcopy(path.a)

        index = local_rws(path)
        a[index] = np.clip(np.random.normal(a[index], 0.3), self.a_min[index], self.a_max[index])#np.random.uniform(0, 1)
        influence_range = 0 #random.randint(0, 1)
        for i in range(influence_range):
            try:
                a[index-i] = np.random.uniform(self.a_min[index-i], self.a_max[index-i])
                a[index+i] = np.random.uniform(self.a_min[index-i], self.a_max[index+i])
            except:
                pass

        self.path.append(TrackPath(self.segmentation, a))

    def peek_mutation(self):
        path = self.path[self.rws()]
        a = deepcopy(path.a)
        worst_a = np.min(path.point_adapt)

        places = np.where(path.point_adapt == worst_a)
        for index in places:
            a[index] = np.random.uniform(0, 1)
            influence_range = 0 #random.randint(0, 3)
            for i in range(influence_range):
                try:
                    a[index-i] = np.random.uniform(self.a_min[index-i], self.a_max[index-i])
                    a[index+i] = np.random.uniform(self.a_min[index-i], self.a_max[index+i])
                except:
                    pass

        self.path.append(TrackPath(self.segmentation, a))

    def general_mutation(self):
        path = self.path[self.rws()]
        a = deepcopy(path.a)
        index = random.randint(0, self.s_size-1)
        a[index] = np.random.uniform(self.a_min[index], self.a_max[index])
        self.path.append(TrackPath(self.segmentation, a))

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
