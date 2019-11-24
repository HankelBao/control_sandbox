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
    irrlicht = False

    # Chrono Simulation step size
    ch_step_size = 3e-3
    # Matplotlib Simulation step size
    mat_step_size = 1e-2

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
    throttle_controller.SetTargetSpeed(speed=10.0)

    initLoc, initRot = GetInitPose([track.center.x[0],track.center.y[0]], [track.center.x[1], track.center.y[1]])

    vehicle = ChronoVehicle(step_size=ch_step_size, track=track, initLoc=initLoc, initRot=initRot, irrlicht=irrlicht)

    simulator = Simulator()

    ch_time = mat_time = 0
    while True:
        # Update controllers
        steering = steering_controller.Advance(ch_step_size, vehicle)
        throttle, braking = throttle_controller.Advance(ch_step_size, vehicle)

        # print('steering :: {}'.format(steering))
        # print('Throttle :: {}'.format(throttle))
        # print('Braking :: {}'.format(braking))

        vehicle.driver.SetTargetSteering(steering)
        vehicle.driver.SetTargetThrottle(throttle)
        vehicle.driver.SetTargetBraking(braking)

        vehicle.Advance(ch_step_size)

        if matplotlib and ch_time >= mat_time:
            simulator.plot(track, vehicle)
            mat_time += mat_step_size

        ch_time += ch_step_size

if __name__ == "__main__":
    main()
