import pychrono as ch

class MPCController:
    def __init__(self):
        pass

    def UpdateState(self, chrono):
        """Updates State :: [x,y,v,heading]"""
        self.state = [
            chrono.vehicle.GetVehiclePos().x,
            chrono.vehicle.GetVehiclePos().y,
            chrono.vehicle.GetVehicleSpeed(),
            chrono.vehicle.GetVehicleRot().Q_to_Euler123().z,
        ]

    def Advance(self, step, chrono):
        # Controls algorithm here
        throttle, braking, steering = 1, 0, 0

        # To get current driver positions
        chrono.driver.GetThrottle()
        chrono.driver.GetSteering()
        chrono.driver.GetBraking()

        pos = ch.ChVectorD(self.state[0], self.state[1], .5)

        # Get closest point on centerline
        pos = chrono.track.center.calcClosestPoint(pos)

        # Get current progression along centerline
        distance = chrono.track.center.calcDistance(pos)
        # print(distance)

        # Get current curvature values of centerline
        curvature = chrono.track.center.calcCurvature(pos)
        # print(curvature)

        # Get index of the closest point on the centerline
        index = chrono.track.center.calcIndex(pos)

        # Get derivative of outer boundaries
        left_dx, left_dy = chrono.track.left.dx[index], chrono.track.left.dy[index]
        # print([left_dx, left_dy])
        right_dx, right_dy = chrono.track.right.dx[index], chrono.track.right.dy[index]
        # print([right_dx, right_dy])
        # Can also get second derivative
        left_ddx, left_ddy = chrono.track.left.ddx[index], chrono.track.left.ddy[index]
        # print([left_ddx, left_ddy])
        right_ddx, right_ddy = chrono.track.right.ddx[index], chrono.track.right.ddy[index]
        # print([right_ddx, right_ddy])

        self.UpdateState(chrono)

        # Set target positions for "ecu" to attempt to get to
        return throttle, steering, braking
