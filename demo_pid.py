from Simulator.chrono_vehicle import ChronoVehicle, GetInitPose
from Controllers.pid_controller import PIDSteeringController, PIDThrottleController
from Track.track import RandomTrack
from Simulator.simulator import Simulator

import random
import sys

def main():
    if len(sys.argv) == 2:
        seed = int(sys.argv[1])
    else:
        seed = 1.0

    # Render preferences
    matplotlib = True
    irrlicht = True

    # Simulation step size
    step_size = 2e-2

    # ------------
    # Create track
    # ------------
    track = RandomTrack()
    track.generateTrack(seed)

    # --------------------
    # Create controller(s)
    # --------------------
    steering_controller = PIDSteeringController(track.center)
    steering_controller.SetGains(Kp=0.4, Ki=0, Kd=0.25)
    steering_controller.SetLookAheadDistance(dist=5.0)
    steering_controller.initTracker(track.center)

    throttle_controller = PIDThrottleController()
    throttle_controller.SetGains(Kp=0.4, Ki=0, Kd=0)
    throttle_controller.SetTargetSpeed(speed=15.0)

    initLoc, initRot = GetInitPose([track.center.x[0],track.center.y[0]], [track.center.x[1], track.center.y[1]])

    vehicle = ChronoVehicle(step_size=step_size, track=track, initLoc=initLoc, initRot=initRot, irrlicht=irrlicht)

    simulator = Simulator()

    while True:
        # Update controllers
        steering = steering_controller.Advance(step_size, vehicle)
        throttle, braking = throttle_controller.Advance(step_size, vehicle)

        # print('steering :: {}'.format(steering))
        # print('Throttle :: {}'.format(throttle))
        # print('Braking :: {}'.format(braking))

        vehicle.driver.SetTargetSteering(steering)
        vehicle.driver.SetTargetThrottle(throttle)
        vehicle.driver.SetTargetBraking(braking)

        vehicle.Advance(step_size)

        if matplotlib:
            simulator.plot(track, vehicle)

if __name__ == "__main__":
    main()
