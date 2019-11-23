from path import Path, RandomPathGenerator
import numpy as np

class Track:
    def __init__(self, width=100, height=100, thickness=5):
        self.width = width
        self.height = height
        self.thickness = thickness

    def createCenterline(self):
        self.generator = RandomPathGenerator(width=100, height=100)
        self.centerline = Path(self.generator.generatePath())
        self.centerline.generatePath()
        self.thickness = 5

    def createTrack(self):
        self.left, self.right = [], []
        sp = self.centerline.spline
        ds = .1
        s = np.arange(0, sp.s[-3], ds)
        for t in s:
            ix, iy = sp.calc_position(t)
            dx, dy = sp.sx.calcd(t), sp.sy.calcd(t)
            len = np.linalg.norm(np.array([dx,dy]))
            dx, dy = dx / len, dy / len
            dx, dy = dx * self.thickness, dy * self.thickness
            self.left.append([ix-dy, iy+dx])
            self.right.append([ix+dy, iy-dx])
        # lx = [point[0] for point in self.left]
        # ly = [point[1] for point in self.left]
        # rx = [point[0] for point in self.right]
        # ry = [point[1] for point in self.right]
        # cx = [point[0] for point in self.centerline]
        # cy = [point[1] for point in self.centerline]
        # import matplotlib.pyplot as plt
        # plt.plot(lx, ly, '-b')
        # plt.plot(rx, ry, '-g')
        # plt.plot(cx, cy, '-r')
        # plt.show()



if __name__ == "__main__":
    track = Track()
    track.createTrack()
