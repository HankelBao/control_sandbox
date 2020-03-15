import pychrono as chrono
import numpy as np
import math
import matplotlib.pyplot as plt
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
        self.sentinel = chrono.ChVectorD(0, 0, 0)

        self.steering = 0

        self.err = 0
        self.errd = 0
        self.erri = 0

        self.track = track

    def SetGains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def SetLookAheadDistance(self, dist):
        self.dist = dist

    # run every time step
    # takes in vehicle model
    # get sentinal
    #   in front of car
    # sets target point
    # subtract diff : error
    # derivative of last error
    # return path
    def Advance(self, step, vehicle):
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
        state = vehicle.GetState()
        self.sentinel = chrono.ChVectorD(
            self.dist * math.cos(state.yaw) + state.x,
            self.dist * math.sin(state.yaw) + state.y,
            0,
        )
        #self.target = self.path.calcClosestPoint(self.sentinel)

        leftPath = self.track.left
        rightPath = self.track.right

        # ---find closest points on left and right path
        # ---calculate line from these points

        # points on the boundaries
        leftPoint = leftPath.calcClosestPoint(self.sentinel)
        rightPoint = rightPath.calcClosestPoint(self.sentinel)

        # plot line
        plt.plot([leftPoint.x, leftPoint.y], [rightPoint.x, rightPoint.y])
