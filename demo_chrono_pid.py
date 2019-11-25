from Simulator.chrono_sim import ChronoSim, GetInitPose
from Controllers.pid_controller import PIDSteeringController, PIDThrottleController
from Track.track import RandomTrack
from Simulator.mat_sim import MatSim

import random
import sys

def main():
    if len(sys.argv) == 2:
        seed = int(sys.argv[1])
    else:
        seed = 1.0

    # Render preferences
    matplotlib = 1
    irrlicht = 1

    # Chrono Simulation step size
    ch_step_size = 1e-2
    # Matplotlib Simulation step size
    mat_step_size = 1

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

    initLoc, initRot = GetInitPose(
        [track.center.x[0], track.center.y[0]], [track.center.x[1], track.center.y[1]]
    )

    chrono = ChronoSim(
        step_size=ch_step_size,
        track=track,
        initLoc=initLoc,
        initRot=initRot,
        irrlicht=irrlicht,
    )

    mat = MatSim(mat_step_size)

    ch_time = mat_time = 0
    while True:
        # Update controllers
        steering = steering_controller.Advance(ch_step_size, chrono)
        throttle, braking = throttle_controller.Advance(ch_step_size, chrono)

        # print('steering :: {}'.format(steering))
        # print('Throttle :: {}'.format(throttle))
        # print('Braking :: {}'.format(braking))

        chrono.driver.SetTargetSteering(steering)
        chrono.driver.SetTargetThrottle(throttle)
        chrono.driver.SetTargetBraking(braking)

        if chrono.Advance(ch_step_size) == -1:
            break

        if matplotlib and ch_time >= mat_time:
            if mat.plot(track, chrono) == -1:
                print("Quit message received.")
                break
            mat_time += mat_step_size

        ch_time += ch_step_size

    if irrlicht:
        chrono.Close()
    if matplotlib:
        mat.close()
    print("Exited")
    pass


if __name__ == "__main__":
    main()
