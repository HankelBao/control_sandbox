import numpy as np
import math


class GenericCar():
    """
    generic car class
    """
    class State:
        """
        vehicle state class
        """

        def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, w_e=0.0, w_e_dot=0.0):
            self.x = x
            self.y = y
            self.yaw = yaw
            self.v = v
            self.w_e = w_e
            self.w_e_dot = w_e_dot
            self.predelta = None

        def __getitem__(self, key):
            return self[key]

        def __str__(self):
            return str('({}, {}, {}, {})'.format(self.x, self.y, self.yaw, self.v))

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.max_steer = np.deg2rad(45.0)  # maximum steering angle [rad]
        self.max_dsteer = np.deg2rad(30.0)  # maximum steering speed [rad/s]
        self.max_speed = 55.0 / 3.6  # maximum speed [m/s]
        self.min_speed = -20.0 / 3.6  # minimum speed [m/s]
        self.max_accel = 1.0  # maximum accel [m/ss]

        # Vehicle parameters
        self.length = 4.5  # [m]
        self.width = 2.0  # [m]
        self.backtowheel = 1.0  # [m]
        self.wheel_len = 0.3  # [m]
        self.wheel_width = 0.2  # [m]
        self.tread = 0.7  # [m]
        self.wb = 2.5  # [m]
        self.mass = 1500 # [kg]

        self.state = self.State(x, y, yaw, v)

        self.steer = 0
        self.throttle = 0
        self.braking = 0

    def Update(self, throttle, steering, braking, step):
        steering = np.deg2rad(steering)

        # input check
        if steering >= self.max_steer:
            steering = self.max_steer
        elif steering <= -self.max_steer:
            steering = -self.max_steer

        F = throttle * 100
        a = F / self.mass

        self.state.x = self.state.x + self.state.v * \
            math.cos(self.state.yaw) * step
        self.state.y = self.state.y + self.state.v * \
            math.sin(self.state.yaw) * step
        self.state.yaw = self.state.yaw + self.state.v / \
            self.wb * math.tan(steering) * step
        self.state.v = self.state.v + a * step

        if self.state. v > self.max_speed:
            self.state.v = self.max_speed
        elif self.state. v < self.min_speed:
            self.state.v = self.min_speed

        self.steer = steering
        self.throttle = throttle
        self.braking = braking

    def __str__(self):
        return str(self.state)
