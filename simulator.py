import numpy as np
import math
import matplotlib.pyplot as plt

class Simulator:
    def __init__(self, track):
        self.track = track

    def plot(self, veh):
        plt.cla()
        self.plot_car(veh)
        self.plot_track()
        plt.pause(0.000001)

    def plot_track(self, trackcolor="-b"):
        plt.plot(self.track.x, self.track.y, trackcolor)

    def plot_car(self, veh, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

        outline = np.array([[-veh.backtowheel, (veh.length - veh.backtowheel), (veh.length - veh.backtowheel), -veh.backtowheel, -veh.backtowheel],
                            [veh.width / 2, veh.width / 2, - veh.width / 2, -veh.width / 2, veh.width / 2]])

        fr_wheel = np.array([[veh.wheel_len, -veh.wheel_len, -veh.wheel_len, veh.wheel_len, veh.wheel_len],
                             [-veh.wheel_width - veh.tread, -veh.wheel_width - veh.tread, veh.wheel_width - veh.tread, veh.wheel_width - veh.tread, -veh.wheel_width - veh.tread]])

        rr_wheel = np.copy(fr_wheel)

        fl_wheel = np.copy(fr_wheel)
        fl_wheel[1, :] *= -1
        rl_wheel = np.copy(rr_wheel)
        rl_wheel[1, :] *= -1

        Rot1 = np.array([[math.cos(veh.state.yaw), math.sin(veh.state.yaw)],
                         [-math.sin(veh.state.yaw), math.cos(veh.state.yaw)]])
        Rot2 = np.array([[math.cos(veh.steer), math.sin(veh.steer)],
                         [-math.sin(veh.steer), math.cos(veh.steer)]])

        fr_wheel = (fr_wheel.T.dot(Rot2)).T
        fl_wheel = (fl_wheel.T.dot(Rot2)).T
        fr_wheel[0, :] += veh.wb
        fl_wheel[0, :] += veh.wb

        fr_wheel = (fr_wheel.T.dot(Rot1)).T
        fl_wheel = (fl_wheel.T.dot(Rot1)).T

        outline = (outline.T.dot(Rot1)).T
        rr_wheel = (rr_wheel.T.dot(Rot1)).T
        rl_wheel = (rl_wheel.T.dot(Rot1)).T

        outline[0, :] += veh.state.x
        outline[1, :] += veh.state.y
        fr_wheel[0, :] += veh.state.x
        fr_wheel[1, :] += veh.state.y
        rr_wheel[0, :] += veh.state.x
        rr_wheel[1, :] += veh.state.y
        fl_wheel[0, :] += veh.state.x
        fl_wheel[1, :] += veh.state.y
        rl_wheel[0, :] += veh.state.x
        rl_wheel[1, :] += veh.state.y


        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), truckcolor)
        plt.plot(np.array(fr_wheel[0, :]).flatten(),
                 np.array(fr_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(rr_wheel[0, :]).flatten(),
                 np.array(rr_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(fl_wheel[0, :]).flatten(),
                 np.array(fl_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(rl_wheel[0, :]).flatten(),
                 np.array(rl_wheel[1, :]).flatten(), truckcolor)
        plt.plot(veh.state.x, veh.state.y, "*")
