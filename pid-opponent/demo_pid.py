from control_utilities.chrono import ChronoSim
from control_utilities.chrono_utilities import calcPose
from control_utilities.track import RandomTrack, Track
from control_utilities.matplotlib import MatSim
from control_utilities.opponent import generateRandomOpponent

from pid_controller import PIDSteeringController, PIDThrottleController
from opponent_pid_controller import OpponentPIDSteeringController, OpponentPIDThrottleController

import random
import sys

def main():
    global first, time
    if len(sys.argv) == 2:
        seed = int(sys.argv[1])
    else:
        seed = random.randint(0,100)

    # Render preferences
    matplotlib = 0
    irrlicht = 1

    # Chrono Simulation step size
    ch_step_size = 1e-2
    # Matplotlib Simulation step size
    mat_step_size = 1e-2

    # ------------
    # Create track
    # ------------
    reversed = random.randint(0,1)
    track = RandomTrack(width=20)
    track.generateTrack(seed=seed, reversed=reversed)
    print('Using seed :: {}'.format(seed))

    # ----------------
    # Create opponent
    # ----------------
    opponents = []
    n = 2
    for i in range(n):
        opponent_track = Track(track.center.waypoints, width=track.width/2)
        opponent_track.generateTrack()
        if i % 3 == 0:
            opponent_path = opponent_track.left
        elif i % 3 == 1:
            opponent_path = opponent_track.right
        else:
            opponent_path = opponent_track.center

        opponent_lat_controller = OpponentPIDSteeringController(opponent_path)
        opponent_lat_controller.SetGains(Kp=0.4, Ki=0, Kd=0.25)
        opponent_lat_controller.SetLookAheadDistance(dist=5)

        opponent_long_controller = OpponentPIDThrottleController()
        opponent_long_controller.SetGains(Kp=0.4, Ki=0, Kd=0)
        opponent_long_controller.SetTargetSpeed(speed=5.0)
        opponent = generateRandomOpponent(opponent_path, s_min=opponent_path.s[100 + i * 100], s_max=opponent_path.s[100 + i * 100], lat_controller=opponent_lat_controller, long_controller=opponent_long_controller)
        opponents.append(opponent)

    # --------------------
    # Create controller(s)
    # --------------------
    lat_controller = PIDSteeringController(track.center)
    lat_controller.SetGains(Kp=0.4, Ki=0, Kd=0.25)
    lat_controller.SetLookAheadDistance(dist=5)
    # lat_controller.initTracker(track.center)

    long_controller = PIDThrottleController()
    long_controller.SetGains(Kp=0.4, Ki=0, Kd=0)
    long_controller.SetTargetSpeed(speed=10.0)

    initLoc, initRot = calcPose([track.center.x[0],track.center.y[0]], [track.center.x[1],track.center.y[1]])

    chrono = ChronoSim(
        step_size=ch_step_size,
        track=track,
        initLoc=initLoc,
        initRot=initRot,
        irrlicht=irrlicht,
        opponents=opponents,
        draw_barriers=True,
        vis_balls=True
    )

    mat = MatSim(mat_step_size)

    ch_time = mat_time = 0
    while True:
        # Update controllers
        steering = lat_controller.Advance(ch_step_size, chrono)
        throttle, braking = long_controller.Advance(ch_step_size, chrono)

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

        if ch_time > 50:
            break
    print("Exited")
    pass

if __name__ == "__main__":
    main()
