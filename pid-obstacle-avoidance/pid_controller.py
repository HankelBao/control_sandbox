import pychrono as chrono
import numpy as np
import math
from math import atan
from math import cos
from math import sin
from math import pi
from matplotlib import pyplot as plt


class PIDSteeringController:
    def __init__(self, path, obstacles):
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

        self.lookDist = 0
        self.obsDist = 0
        self.target = chrono.ChVectorD(0, 0, 0)

        self.steering = 0

        self.err = 0
        self.errd = 0
        self.erri = 0

        self.path = path

        self.obstacles = obstacles
        self.gap = 0
        self.distanceToObstacle = 0
        # self.tracker = path.tracker
        self.obstacleInRange = False

    def SetGains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def SetLookAheadDistance(self, lookDist):
        self.lookDist = lookDist

    def SetObstacleDistance(self, obsDist):
        self.obsDist = obsDist

    def calcObstacleAvoidanceAngle(self, left, it, targ, state):
        p1 = self.path.getPoint(it-1)
        p2 = self.path.getPoint(it + 1)
        obsp1x = self.obstacle.p1.x
        obsp2x = self.obstacle.p2.x
        obsp1y = self.obstacle.p1.y
        angle = 0

        x = 0
        if obsp2x > obsp1x and p2.x >= p1.x:
            if left:
                x = 1
            else:
                x = -1
        elif obsp2x > obsp1x and p2.x < p1.x:
            if left:
                x = -1
            else:
                x = 1
        elif obsp2x < obsp1x and p2.x <= p1.x:
            if left:
                x = -1
            else:
                x = 1
        elif obsp2x < obsp1x and p2.x > p1.x:
            if left:
                x = 1
            else:
                x = -1
        else:
            if targ.y > state.y:
                if left:
                   x = 1
                else:
                   x = -1
            else:
                if left:
                   x = -1
                else:
                   x = 1
        angle = atan((p2.y - p1.y) / (p2.x - p1.x)) + x*pi / 2

        return angle


    #def calculateWayPoint(self, lor, s, obstacle):
    #    centerX = (obstacle.p1.x + obstacle.p2.x)/2
    #    centerY = (obstacle.p1.y + obstacle.p2.y)/2
    #    wayPointX = 0
    #    wayPointY = 0
    #    angle = self.calcObstacleAvoidanceAngle(lor, obstacle)
    #    if(lor):
    #        wayPointX = centerX - s*cos(angle)
    #        wayPointY = centerY - s*sin(angle)
    #    else:
    #        wayPointX = centerX + s*cos(angle)
    #        wayPointY = centerY + s*sin(angle)
    #    return wayPointX, wayPointY

    def calculateObstacleOffset(self, s, a, xV):
        offset = s/(1 + math.exp(-a*(xV)))
        return offset

    def alterTargetForObstacle(self, it, left, s, a, xV, targ, state):
        angle = self.calcObstacleAvoidanceAngle(left, it, targ, state)
        offset = self.calculateObstacleOffset(s, a, xV)


    def Advance(self, step, veh_model):
        state = veh_model.GetState()
        self.sentinel = chrono.ChVectorD(
            self.lookDist * math.cos(state.yaw) + state.x,
            self.lookDist * math.sin(state.yaw) + state.y,
            0,
        )

        self.target = self.path.calcClosestPoint(self.sentinel)
        targ = chrono.ChVectorD(self.target.x, self.target.y, 0.5)
        self.sentinel.z = 0.5

        io = self.path.calcIndex(state) + self.obsDist

        ist = self.path.calcIndex(state)

        it = self.path.calcIndex(targ)

        loc = 0
        #print(ist)
        self.obstacle = ()
        for i in range(ist, io+1):
             try:
                self.obstacle = self.obstacles[i]
                loc = i
                self.obstacleInRange = True
                if self.gap == 0:
                    self.gap = self.path.getDistance(loc) - self.path.getDistance(it)
             except KeyError:
                x = 0

        # print(self.obstacleInRange)
        if ist > loc and self.obstacleInRange:
            self.obstacleInRange = False
            self.gap = 0

        if self.obstacleInRange:
            self.distanceToObstacle = self.path.getDistance(loc) - self.path.getDistance(it)
            self.alterTargetForObstacle(it, False, 10, 0.75, -((self.distanceToObstacle/self.gap)*10 - 5), targ, state)
            #print("NEW Target X: " + str(targ.x))
            #print("NEW Target Y: " + str(targ.y))


        if hasattr(veh_model, 'ballS'):
            veh_model.ballS.SetPos(self.sentinel)
            veh_model.ballT.SetPos(targ)

        # The "error" vector is the projection onto the horizontal plane (z=0) of
        # vector between sentinel and target
        err_vec = targ - self.sentinel
        err_vec.z = 0

        # Calculate the sign of the angle between the projections of the sentinel
        # vector and the target vector (with origin at vehicle location).
        sign = self.calcSign(veh_model, targ)

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

        return self.steering, self.obstacleInRange

    def calcSign(self, veh_model, targ):
        """
        Calculate the sign of the angle between the projections of the sentinel vector
        and the target vector (with origin at vehicle location).
        """

        pos = veh_model.GetState()
        pos = chrono.ChVectorD(pos.x, pos.y, 0)
        sentinel_vec = self.sentinel - pos
        sentinel_vec.z = 0
        target_vec = targ - pos
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
        self.dist = 0

    def SetGains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def SetLookAheadDistance(self, dist):
        self.dist = dist

    def SetTargetSpeed(self, speed):
        self.target_speed = speed

    def Advance(self, step, veh_model, obstacleInRange):
        state = veh_model.GetState()
        self.sentinel = chrono.ChVectorD(
            self.dist * math.cos(state.yaw) + state.x,
            self.dist * math.sin(state.yaw) + state.y,
            0,
        )

        self.target = self.path.calcClosestPoint(self.sentinel)

        print(obstacleInRange)

        if obstacleInRange:
            self.target_speed = 6
        else:
            self.target_speed = self.path.calcSpeed(self.target)


        self.speed = veh_model.GetState().v
        # print(self.speed)

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
