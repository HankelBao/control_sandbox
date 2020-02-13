from control_utilities.chrono import ChronoSim, GetInitPose
from control_utilities.track import RandomTrack
from control_utilities.matplotlib import MatSim

from mpc_controller import MPCController

import random
import sys

def main():
    if len(sys.argv) == 2:
        seed = int(sys.argv[1])
    else:
        seed = random.randint(0,100)

    # Render preferences
    matplotlib = 1
    irrlicht = 1

    # Chrono Simulation step size
    ch_step_size = 1e-2
    # Matplotlib Simulation step size
    mat_step_size = 1e-2

    # ------------
    # Create track
    # ------------
    reversed = random.randint(0,1)
    track = RandomTrack()
    track.generateTrack(seed, reversed)

    # --------------------
    # Create controller(s)
    # --------------------
    mpc_controller = MPCController()

    initLoc, initRot = GetInitPose(
        [track.center.x[0], track.center.y[0]], [track.center.x[1], track.center.y[1]], reversed=reversed
    )

    chrono = ChronoSim(
        step_size=ch_step_size,
        track=track,
        initLoc=initLoc,
        initRot=initRot,
        irrlicht=irrlicht,
    )

    mpc_controller.UpdateState(chrono)

    mat = MatSim(mat_step_size)

    ch_time = mat_time = 0
    while True:
        # Update controllers
        throttle, steering, braking = mpc_controller.Advance(ch_step_size, chrono)

        chrono.driver.SetTargetSteering(steering)
        chrono.driver.SetTargetThrottle(throttle)
        chrono.driver.SetTargetBraking(braking)

        if chrono.Advance(ch_step_size) == -1:
            chrono.Close()
            break

        if matplotlib and ch_time >= mat_time:
            if mat.plot(track, chrono) == -1:
                print("Quit message received.")
                mat.close()
                break
            mat_time += mat_step_size

        ch_time += ch_step_size

    print("Exited")
    pass


if __name__ == "__main__":
    main()
