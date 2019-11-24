from utilities import Vec
import numpy as np
import math

class SteeringController:
    def __init__(self, ):
        self.steer = 0
        self.throttle = 0
        self.braking = 0

        self.d = 1

        self.err = 0
        self.errd = 0
        self.erri = 0

    def SetGains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def Advance(self, state, step):
        target_ind = self.track.calc_nearest_index(state)
        x,y = self.track[target_ind]
        target = np.array([x,y,0])
        sentinel = np.array([self.d * math.cos(state.yaw) + state.x, self.d * math.sin(state.yaw) + state.y, 0])

        # The "error" vector is the projection onto the horizontal plane (z=0) of
        # vector between sentinel and target
        err_vec = target - sentinel

        # Calculate the sign of the angle between the projections of the sentinel
        # vector and the target vector (with origin at vehicle location).
        pos = np.array([state.x, state.y, 0])
        sign = self.calcSign(sentinel, target, pos)

        # Calculate current error (magnitude)
        err = sign * np.linalg.norm(err_vec)

        # Estimate error derivative (backward FD approximation).
        self.errd = (err - self.err) / step;

        # Calculate current error integral (trapezoidal rule).
        self.erri += (err + self.err) * step / 2;

        # Cache new error
        self.err = err

        # Return PID output (steering value)
        self.steer = -np.clip(self.Kp * self.err + self.Ki * self.erri + self.Kd * self.errd, np.deg2rad(-180), np.deg2rad(180))

    def calcSign(self, sentinel, target, pos):
        '''
        Calculate the sign of the angle between the projections of the sentinel vector
        and the target vector (with origin at vehicle location).
        '''

        sentinel_vec = sentinel - pos
        target_vec = target - pos
        # print(target_vec)

        temp = (np.cross(target_vec, sentinel_vec)).dot(np.array([0,0,1]))
        return int((temp > 0)) - int((temp < 0))

class SpeedController:
    def __init__(self, track):
        self.steer = 0
        self.throttle = 0
        self.braking = 0

        self.track = track

        self.d = 1

        self.speed = 0
        self.target_speed = 0

        self.throttle_threshold = 0.2

        self.err = 0
        self.errd = 0
        self.erri = 0

        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

    def SetGains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def SetTargetSpeed(self, speed):
        self.target_speed = speed

    def Advance(self, state, step):
        self.speed = state.v

        # Calculate current error
        err = self.target_speed - self.speed

        # Estimate error derivative (backward FD approximation)
        self.errd = (err - self.err) / step;

        # Calculate current error integral (trapezoidal rule).
        self.erri += (err + self.err) * step / 2;

        # Cache new error
        self.err = err

        # Return PID output (steering value)
        thr = np.clip(self.Kp * self.err + self.Ki * self.erri + self.Kd * self.errd, -1.0, 1.0)

        if thr > 0:
            # Vehicle moving too slow
            self.throttle = thr
        else:
            # Vehicle moving too fast: apply brakes
            self.throttle = 0
            self.braking = thr
