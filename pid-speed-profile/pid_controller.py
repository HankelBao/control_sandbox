import pychrono as chrono
import numpy as np
import math


class PIDSteeringController:
    def __init__(self, path):
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

        self.dist = 0
        self.target = chrono.ChVectorD(0, 0, 0)

        self.steering = 0

        self.err = 0
        self.errd = 0
        self.erri = 0

        self.path = path
        # self.tracker = path.tracker

    def SetGains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def SetLookAheadDistance(self, dist):
        self.dist = dist

    def Advance(self, step, veh_model):
        state = veh_model.GetState()
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
        sign = self.calcSign(veh_model)

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

        return self.steering

    def calcSign(self, veh_model):
        """
        Calculate the sign of the angle between the projections of the sentinel vector
        and the target vector (with origin at vehicle location).
        """

        pos = veh_model.GetState()
        pos = chrono.ChVectorD(pos.x, pos.y, 0)
        sentinel_vec = self.sentinel - pos
        sentinel_vec.z = 0
        target_vec = self.target - pos
        target_vec.z = 0

        temp = (sentinel_vec % target_vec) ^ chrono.ChVectorD(0, 0, 1)

        return (temp > 0) - (temp < 0)


class PIDThrottleController:
    def __init__(self, path):
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

        self.err = 0
        self.errd = 0
        self.erri = 0

        self.speed = 0
        self.target_speed = 0

        self.throttle_threshold = 0.2

        self.path = path

    def SetGains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def SetTargetSpeed(self, speed):
        self.target_speed = speed

    def Advance(self, step, veh_model):
        self.target_speed = self.path.calcSpeed(veh_model.vehicle.GetVehiclePos())

        self.speed = veh_model.GetState().v

        # Calculate current error
        err = self.target_speed - self.speed

        # Estimate error derivative (backward FD approximation)
        self.errd = (err - self.err) / step

        # Calculate current error integral (trapezoidal rule).
        self.erri += (err + self.err) * step / 2

        # Cache new error
        self.err = err

        # Return PID output (steering value)
        throttle = np.clip(
            self.Kp * self.err + self.Ki * self.erri + self.Kd * self.errd, -1.0, 1.0
        )

        if throttle > 0:
            # Vehicle moving too slow
            self.braking = 0
            self.throttle = throttle
        elif veh_model.driver.GetTargetThrottle() > self.throttle_threshold:
            # Vehicle moving too fast: reduce throttle
            self.braking = 0
            self.throttle = veh_model.driver.GetTargetThrottle() + throttle
        else:
            # Vehicle moving too fast: apply brakes
            self.braking = -throttle
            self.throttle = 0
        return self.throttle, self.braking
