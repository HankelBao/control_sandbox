import pychrono as chrono
import numpy as np
import math

class PIDController:
    def __init__(self, lat_controller, long_controller):
        self.lat_controller = lat_controller
        self.long_controller = long_controller

    def GetTargetAndSentinel(self):
        return self.lat_controller.target, self.lat_controller.sentinel

    def Advance(self, step, vehicle):
        self.lat_controller.Advance(step, vehicle)
        self.long_controller.Advance(step, vehicle)
        # return self.steering, self.throttle, self.braking

class PIDLateralController:
    def __init__(self, path):
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

        self.dist = 0
        self.target = chrono.ChVectorD(0, 0, 0)
        self.sentinel = chrono.ChVectorD(0, 0, 0)

        self.steering = 0

        self.err = 0
        self.errd = 0
        self.erri = 0

        self.path = path

    def SetGains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def SetLookAheadDistance(self, dist):
        self.dist = dist

    def Advance(self, step, vehicle):
        state = vehicle.GetState()
        self.sentinel = chrono.ChVectorD(
            self.dist * math.cos(state.yaw) + state.x,
            self.dist * math.sin(state.yaw) + state.y,
            0,
        )

        self.target = self.path.calcClosestPoint(self.sentinel)

        # The "error" vector is the projection onto the horizontal plane (z=0) of
        # vector between sentinel and target
        err_vec = self.target - self.sentinel
        err_vec.z = 0

        # Calculate the sign of the angle between the projections of the sentinel
        # vector and the target vector (with origin at vehicle location).
        sign = self.calcSign(state)

        # Calculate current error (magnitude)
        err = sign * err_vec.Length()

        # Estimate error derivative (backward FD approximation).
        self.errd = (err - self.err) / step

        # Calculate current error integral (trapezoidal rule).
        self.erri += (err + self.err) * step / 2 
        # Cache new error
        self.err = err

        # Return PID output (steering value)
        self.steering = np.clip(
            self.Kp * self.err + self.Ki * self.erri + self.Kd * self.errd, -1.0, 1.0
        )

        vehicle.driver.SetTargetSteering(self.steering)

        return self.steering

    def calcSign(self, state):
        """
        Calculate the sign of the angle between the projections of the sentinel vector
        and the target vector (with origin at vehicle location).
        """

        sentinel_vec = self.sentinel - state.pos
        target_vec = self.target - state.pos

        temp = (sentinel_vec % target_vec) ^ chrono.ChVectorD(0, 0, 1)

        return (temp > 0) - (temp < 0)


class PIDLongitudinalController:
    """
    Steps to get the p/i/d here:
    1. Set the target_speed to a reachable speed and tweak p and i of acc and brk so that the vehicle could stay at that speed nicely and slowly move to other speeds
    2. Set kd so that the vehicle could accelerate as fast as it could. Vice Versa for brk.

    In order to let the vehicle safely follow the speed profile, please take time to tweak.
    Again, IT TAKES TIME to tweak to last digit.
    """
    def __init__(self, path):
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

        self.err = 0
        self.errd = 0
        self.erri = 0

        self.speed = 0
        self.target_speed = 0

        self.path = path

    def SetGains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def SetTargetSpeed(self, speed):
        self.target_speed = speed

    def SetLookAheadDistance(self, dist):
        self.dist = dist

    def __acc_until_reach_advance(self, vehicle, target_speed):
        if not hasattr(self, 'reached'):
            self.reached = False

        if self.reached:
            return True

        if self.speed < target_speed:
            vehicle.driver.SetTargetThrottle(1)
            return False
        else:
            self.reached = True
            return True


    def Advance(self, step, vehicle):
        state = vehicle.GetState()
        self.sentinel = chrono.ChVectorD(
            self.dist * math.cos(state.yaw) + state.x,
            self.dist * math.sin(state.yaw) + state.y,
            0,
        )

        self.target = self.path.calcClosestPoint(self.sentinel)

        self.target_speed = self.path.calcSpeed(self.target)

        self.speed = state.v

        # err
        err = self.target_speed - self.speed

        # errd
        if np.sign(err) == np.sign(self.err):
            self.errd += abs((err - self.err) / step)
        else:
            self.errd = 0

        # erri
        self.erri += (err + self.err) * step / 2

        # Cache new error
        self.err = err

        # if not self.__acc_until_reach_advance(vehicle, 10):
        #     return

        if self.speed < self.target_speed:
            self.braking = 0
            self.throttle = np.clip(
                1.3 * self.err - 0.001 * self.erri + 0.002*self.errd, 0, 1.0
            )
        else:
            self.braking = -np.clip(
                0.15 * self.err + 0.011 * self.erri - 0.00* self.errd, -1.0, 0
            )
            self.throttle = 0

        vehicle.driver.SetTargetThrottle(self.throttle)
        vehicle.driver.SetTargetBraking(self.braking)

        return self.throttle, self.braking
