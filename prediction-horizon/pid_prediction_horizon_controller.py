import pychrono as chrono
import numpy as np
import math
from control_utilities.track import Track
from control_utilities.path import Path

# class for prediction horizon


class PIDPredictionHorizon:
    def __init__(self, track):
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0

        self.dist = 0
        self.target = chrono.ChVectorD(0, 0, 0)

        self.steering = 0

        self.err = 0
        self.errd = 0
        self.erri = 0

        self.track = Track

        # self.tracker = path.tracker

    def SetGains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def setLookAheadDistance(self, dist):
        self.dist = dist

    # run every time step
    # takes in vehicle model
    # get sentinal
    #   in front of car
    # sets target point
    # subtract diff : error
    # derivative of last error
    # return path
    def Advance(self, step, veh_model):
        """
        calculate horizonatal line across the path
        find the center of the line 
        track 
        params:
            - state
            - vehicle model
        returns:
            - path 
        """
        state = veh_model.GetState()
        self.sentinel = chrono.ChVectorD(
            self.dist * math.cos(state.yaw) + state.x,
            self.dist * math.sin(state.yaw) + state.y,
            0,
        )
        self.target = self.path.calcClosestPoint(self.sentinel)
        
        leftPath = Path(self.track.left)
        rightPath = Path(self.track.right)

        # ---find closest points on left and right path
        # ---calculate line from these points

        # points on the boundaries 
        leftPoint = leftPath.calcClosestPoint(self.sentinal)
        rightPoint = rightPath.calcClosestPoint(self.sentinel)

        # def line fuction
        a = (leftPoint[1] - leftPoint[0]) / (rightPoint[1] - leftPoint[0])

        b = leftPoint[0] - a * rightPoint[0]


