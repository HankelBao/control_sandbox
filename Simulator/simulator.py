import numpy as np
import math
import matplotlib.pyplot as plt


class Simulator:
    def __init__(self):
        pass

    def plot(self, track, vehicle):
        # plt.cla()
        if not hasattr(self, 'center'):
            self.plot_track(track)
        if hasattr(self, 'outline'):
            self.outline.remove()
            self.fr.remove()
            self.rr.remove()
            self.fl.remove()
            self.rl.remove()
            self.annotation.remove()
        self.plot_car(vehicle, vehicle.GetState())
        self.plot_text(vehicle)
        plt.pause(0.00001)

    def plot_text(self, vehicle):
        str = 'Throttle :: {0:0.2f}\nSteering :: {1:0.2f}\nBraking :: {2:0.2f}\nSpeed :: {3:0.2f}'.format(
            vehicle.driver.GetThrottle(), vehicle.driver.GetSteering(), vehicle.driver.GetBraking(), vehicle.vehicle.GetVehicleSpeed())
        self.annotation = plt.annotate(str, xy=(.97, .70), xytext=(0, 10), xycoords=(
            'axes fraction', 'figure fraction'), textcoords='offset points', size=10, ha='right', va='bottom')

    def plot_track(self, track, trackcolor="-b"):
        self.center = plt.plot(track.center.x, track.center.y, '-r')
        self.left = plt.plot(track.left.x, track.left.y)
        self.right = plt.plot(track.right.x, track.right.y)

    def plot_car(self, veh, state, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

        outline = np.array([[-veh.backtowheel, (veh.length - veh.backtowheel), (veh.length - veh.backtowheel), -veh.backtowheel, -veh.backtowheel],
                            [veh.width / 2, veh.width / 2, - veh.width / 2, -veh.width / 2, veh.width / 2]])

        fr_wheel = np.array([[veh.wheel_len, -veh.wheel_len, -veh.wheel_len, veh.wheel_len, veh.wheel_len],
                             [-veh.wheel_width - veh.tread, -veh.wheel_width - veh.tread, veh.wheel_width - veh.tread, veh.wheel_width - veh.tread, -veh.wheel_width - veh.tread]])

        rr_wheel = np.copy(fr_wheel)

        fl_wheel = np.copy(fr_wheel)
        fl_wheel[1, :] *= -1
        rl_wheel = np.copy(rr_wheel)
        rl_wheel[1, :] *= -1

        Rot1 = np.array([[math.cos(state.yaw), math.sin(state.yaw)],
                         [-math.sin(state.yaw), math.cos(state.yaw)]])
        Rot2 = np.array([[math.cos(veh.driver.GetSteering()), math.sin(veh.driver.GetSteering())],
                         [-math.sin(veh.driver.GetSteering()), math.cos(veh.driver.GetSteering())]])

        fr_wheel = (fr_wheel.T.dot(Rot2)).T
        fl_wheel = (fl_wheel.T.dot(Rot2)).T
        fr_wheel[0, :] += veh.wb
        fl_wheel[0, :] += veh.wb

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

        self.outline, = plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), truckcolor)
        self.fr, = plt.plot(np.array(fr_wheel[0, :]).flatten(),
                 np.array(fr_wheel[1, :]).flatten(), truckcolor)
        self.rr, = plt.plot(np.array(rr_wheel[0, :]).flatten(),
                 np.array(rr_wheel[1, :]).flatten(), truckcolor)
        self.fl, = plt.plot(np.array(fl_wheel[0, :]).flatten(),
                 np.array(fl_wheel[1, :]).flatten(), truckcolor)
        self.rl, = plt.plot(np.array(rl_wheel[0, :]).flatten(),
                 np.array(rl_wheel[1, :]).flatten(), truckcolor)
        # plt.plot(state.x, state.y, "*")
