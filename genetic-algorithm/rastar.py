import numpy as np
import heapq
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
