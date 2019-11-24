import numpy as np
import math
import matplotlib.pyplot as plt

class Simulator:
    def __init__(self):
        pass

    def plot(self, track, vehicle):
        plt.cla()
        self.plot_car(vehicle, vehicle.GetState())
        self.plot_track(track)
        plt.pause(0.000001)

    def plot_track(self, track, trackcolor="-b"):
        plt.plot(track.center.x, track.center.y, '-r')
        plt.plot(track.left.x, track.left.y)
        plt.plot(track.right.x, track.right.y)

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
        plt.plot(state.x, state.y, "*")
