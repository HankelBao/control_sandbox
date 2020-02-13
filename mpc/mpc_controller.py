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

        # Get closest point on centerline
        index = chrono.track.center.get_index(self.state[0], self.state[1])

        # Get current progression along centerline
        # distance, index = chrono.track.center.get_arc_length(self.state[0], self.state[1]) or
        distance = chrono.track.center.s[index]
        print(distance)
        # Get current curvature values of centerline
        curvature = chrono.track.center.k[index]
        print(curvature)

        # Get derivative of outer boundaries
        left_dx, left_dy = chrono.track.left.dx[index], chrono.track.left.dy[index]
        print([left_dx, left_dy])
        right_dx, right_dy = chrono.track.right.dx[index], chrono.track.right.dy[index]
        print([right_dx, right_dy])
        # Can also get second derivative
        left_ddx, left_ddy = chrono.track.left.ddx[index], chrono.track.left.ddy[index]
        print([left_ddx, left_ddy])
        right_ddx, right_ddy = chrono.track.right.ddx[index], chrono.track.right.ddy[index]
        print([right_ddx, right_ddy])

        self.UpdateState(chrono)

        # Set target positions for "ecu" to attempt to get to
        return throttle, steering, braking
