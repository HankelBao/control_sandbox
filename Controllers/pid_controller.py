import pychrono as chrono
import numpy as np

class PIDSteeringController():
    def __init__(self, path):
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

        self.dist = 0
        self.target = chrono.ChVectorD(0,0,0)

        self.steering = 0

        self.err = 0
        self.errd = 0
        self.erri = 0

        self.path = path

    def initTracker(self, path, z=0.5):
        new_path = chrono.vector_ChVectorD()
        for x,y in zip(path.x, path.y):
            point = chrono.ChVectorD(x, y, z)
            new_path.push_back(point)
        self.tracker = chrono.ChBezierCurveTracker(chrono.ChBezierCurve(new_path), False)

    def SetGains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def SetLookAheadDistance(self, dist):
        self.dist = dist

    def Advance(self, step, vehicle):
        self.sentinel = vehicle.vehicle.GetChassisBody().GetFrame_REF_to_abs().TransformPointLocalToParent(chrono.ChVectorD(self.dist, 0, 0))

        self.tracker.calcClosestPoint(self.sentinel, self.target)

        # The "error" vector is the projection onto the horizontal plane (z=0) of
        # vector between sentinel and target
        err_vec = self.target - self.sentinel
        err_vec.z = 0

        # Calculate the sign of the angle between the projections of the sentinel
        # vector and the target vector (with origin at vehicle location).
        sign = self.calcSign(vehicle)

        # Calculate current error (magnitude)
        err = sign * err_vec.Length()

        # Estimate error derivative (backward FD approximation).
        self.errd = (err - self.err) / step;

        # Calculate current error integral (trapezoidal rule).
        self.erri += (err + self.err) * step / 2;

        # Cache new error
        self.err = err

        # Return PID output (steering value)
        self.steering = np.clip(self.Kp * self.err + self.Ki * self.erri + self.Kd * self.errd, -1.0, 1.0)
        return self.steering

    def calcSign(self, vehicle):
        '''
        Calculate the sign of the angle between the projections of the sentinel vector
        and the target vector (with origin at vehicle location).
        '''

        sentinel_vec = self.sentinel - vehicle.vehicle.GetVehiclePos()
        sentinel_vec.z = 0
        target_vec = self.target - vehicle.vehicle.GetVehiclePos()
        target_vec.z = 0

        temp = (sentinel_vec % target_vec) ^ chrono.ChVectorD(0,0,1)

        return (temp > 0) - (temp < 0)

class PIDThrottleController():
    def __init__(self):
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

        self.err = 0
        self.errd = 0
        self.erri = 0

        self.speed = 0
        self.target_speed = 0

        self.throttle_threshold = 0.2

    def SetGains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def SetTargetSpeed(self, speed):
        self.target_speed = speed

    def Advance(self, step, vehicle):
        self.speed = vehicle.vehicle.GetVehicleSpeed()

        # Calculate current error
        err = self.target_speed - self.speed

        # Estimate error derivative (backward FD approximation)
        self.errd = (err - self.err) / step;

        # Calculate current error integral (trapezoidal rule).
        self.erri += (err + self.err) * step / 2;

        # Cache new error
        self.err = err

        # Return PID output (steering value)
        throttle = np.clip(self.Kp * self.err + self.Ki * self.erri + self.Kd * self.errd, -1.0, 1.0)

        if throttle > 0:
            # Vehicle moving too slow
            self.braking = 0
            self.throttle = throttle
        elif vehicle.driver.GetTargetThrottle() > self.throttle_threshold:
            # Vehicle moving too fast: reduce throttle
            self.braking = 0
            self.throttle = vehicle.driver.GetTargetThrottle() + throttle
        else:
            # Vehicle moving too fast: apply brakes
            self.braking = -throttle
            self.throttle = 0
        return self.throttle, self.braking
