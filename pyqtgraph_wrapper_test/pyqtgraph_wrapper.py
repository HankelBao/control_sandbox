import numpy as np
import math
# import matplotlib.pyplot as plt
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import time
from control_utilities.chrono_utilities import calcAngle

class PyQtGraphWrapper:
    def __init__(self, step_size, vehicle, render_step_size=1.0/60):
        self.step_size = step_size
        self.time = 0

        # Time interval between two render frames
        self.render_step_size = render_step_size  # FPS = 60
        self.render_steps = int(math.ceil(self.render_step_size / self.step_size))
        self.step_number = 0
        self.exit = False
        self.vehicle = vehicle

        self.plot = pg.plot()

    def exec(self):
        import sys
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            app = QtWidgets.QApplication(sys.argv)

    def close(self):
        plt.close()

    def plotTrack(self, track):
        track.center.plot(color='-r', show=False)
        track.left.plot(color='-k', show=False)
        track.right.plot(color='-k', show=False)

    def plotObstacles(self, obstacles):
        for _, o in obstacles.items():
            outline = np.array([[-o.length, o.length, o.length, -o.length, -o.length], [o.width / 2, o.width / 2, - o.width / 2, -o.width / 2, o.width / 2]])

            ang = calcAngle(o.p1, o.p2)

            Rot1 = np.array([[math.cos(ang), math.sin(ang)], [-math.sin(ang), math.cos(ang)]])
            outline = (outline.T.dot(Rot1)).T
            outline[0, :] += p1.x
            outline[1, :] += p1.y
            outline, = plt.plot(np.array(outline[0, :]).flatten(), np.array(outline[1, :]).flatten(), color)

    def Advance(self, step):
        if self.step_number % self.render_steps == 0:
            self.plotVehicle()
            self.plotText()
            if len(plt.get_fignums()) == 0:
                return False
        self.step_number += 1
        self.time += step
        return True

    def plotText(self):
        str = '<div style="text-align: center><span style="color: #000; font-size: 10;">Time :: {0:0.1f}<br>Throttle :: {1:0.2f}<br>Steering :: {2:0.2f}<br>Braking :: {3:0.2f}<br>Speed :: {4:0.2f}'.format(
            self.time, self.vehicle.driver.GetThrottle(), self.vehicle.driver.GetSteering(), self.vehicle.driver.GetBraking(), self.vehicle.vehicle.GetVehicleSpeed())
        if not hasattr(self, 'annotation'):
            self.annotation = pg.TextItem(str, anchor=(.97, .7), border='w', fill=(255,255,255,90))
            self.plot.addItem(self.annotation)
        else:
            self.annotation.setText(str)
            # Now let's add your additional information

    def plotVehicle(self, cabcolor="-r", truckcolor="-k"):  # pragma: no cover
        state = self.vehicle.GetState()

        outline = np.array([[-self.vehicle.backtowheel, (self.vehicle.length - self.vehicle.backtowheel), (self.vehicle.length - self.vehicle.backtowheel), -self.vehicle.backtowheel, -self.vehicle.backtowheel],
                            [self.vehicle.width / 2, self.vehicle.width / 2, - self.vehicle.width / 2, -self.vehicle.width / 2, self.vehicle.width / 2]])

        fr_wheel = np.array([[self.vehicle.wheel_len, -self.vehicle.wheel_len, -self.vehicle.wheel_len, self.vehicle.wheel_len, self.vehicle.wheel_len],
                             [-self.vehicle.wheel_width - self.vehicle.tread, -self.vehicle.wheel_width - self.vehicle.tread, self.vehicle.wheel_width - self.vehicle.tread, self.vehicle.wheel_width - self.vehicle.tread, -self.vehicle.wheel_width - self.vehicle.tread]])

        rr_wheel = np.copy(fr_wheel)

        fl_wheel = np.copy(fr_wheel)
        fl_wheel[1, :] *= -1
        rl_wheel = np.copy(rr_wheel)
        rl_wheel[1, :] *= -1

        Rot1 = np.array([[math.cos(state.yaw), math.sin(state.yaw)],
                         [-math.sin(state.yaw), math.cos(state.yaw)]])
        Rot2 = np.array([[math.cos(self.vehicle.driver.GetSteering()), math.sin(self.vehicle.driver.GetSteering())],
                         [-math.sin(self.vehicle.driver.GetSteering()), math.cos(self.vehicle.driver.GetSteering())]])

        fr_wheel = (fr_wheel.T.dot(Rot2)).T
        fl_wheel = (fl_wheel.T.dot(Rot2)).T
        fr_wheel[0, :] += self.vehicle.wb
        fl_wheel[0, :] += self.vehicle.wb

        fr_wheel = (fr_wheel.T.dot(Rot1)).T
        fl_wheel = (fl_wheel.T.dot(Rot1)).T

        outline = (outline.T.dot(Rot1)).T
        rr_wheel = (rr_wheel.T.dot(Rot1)).T
        rl_wheel = (rl_wheel.T.dot(Rot1)).T

        outline[0, :] += state.x
        outline[1, :] += state.y
        fr_wheel[0, :] += state.x
        fr_wheel[1, :] += state.y
        rr_wheel[0, :] += state.x
        rr_wheel[1, :] += state.y
        fl_wheel[0, :] += state.x
        fl_wheel[1, :] += state.y
        rl_wheel[0, :] += state.x
        rl_wheel[1, :] += state.y


        if not hasattr(self, 'outline'):
            print(np.array(outline[0, :]).flatten())
            self.outline = pg.plot(np.array(outline[0, :]).flatten(), np.array(outline[1, :]).flatten(), truckcolor)
            self.fr = pg.plot(np.array(fr_wheel[0, :]).flatten(), np.array(fr_wheel[1, :]).flatten(), truckcolor)
            self.rr = pg.plot(np.array(rr_wheel[0, :]).flatten(), np.array(rr_wheel[1, :]).flatten(), truckcolor)
            self.fl = pg.plot(np.array(fl_wheel[0, :]).flatten(), np.array(fl_wheel[1, :]).flatten(), truckcolor)
            self.rl = pg.plot(np.array(rl_wheel[0, :]).flatten(), np.array(rl_wheel[1, :]).flatten(), truckcolor)
        else:
            self.outline.set_ydata(np.array(outline[1, :]).flatten())
            self.outline.set_xdata(np.array(outline[0, :]).flatten())
            self.fr.set_ydata(np.array(fr_wheel[1, :]).flatten())
            self.fr.set_xdata(np.array(fr_wheel[0, :]).flatten())
            self.rr.set_ydata(np.array(rr_wheel[1, :]).flatten())
            self.rr.set_xdata(np.array(rr_wheel[0, :]).flatten())
            self.fl.set_ydata(np.array(fl_wheel[1, :]).flatten())
            self.fl.set_xdata(np.array(fl_wheel[0, :]).flatten())
            self.rl.set_ydata(np.array(rl_wheel[1, :]).flatten())
            self.rl.set_xdata(np.array(rl_wheel[0, :]).flatten())
        # plt.plot(state.x, state.y, "*")
