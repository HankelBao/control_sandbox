from control_utilities.chrono import ChronoSim, GetInitPose
from control_utilities.track import RandomTrack
from control_utilities.matplotlib import MatSim
from control_utilities.track import RandomTrack, Track
from control_utilities.path import Path

from pid_controller import PIDSteeringController, PIDThrottleController

import random
import sys

def main():
    global first, time
    if len(sys.argv) == 2:
        seed = int(sys.argv[1])
    else:
        seed = random.randint(0,100)

    # Render preferences
    matplotlib = 1
    irrlicht = 0

    # Chrono Simulation step size
    ch_step_size = 1e-2
    # Matplotlib Simulation step size
    mat_step_size = 1e-2

    # ------------
    # Create track
    # ------------
    reversed = random.randint(0,1)
    track = RandomTrack()
    track.generateTrack(seed=seed, reversed=reversed)
    print('Using seed :: {}'.format(seed))

    # print("FIRST 5 POINTS OF INITIAL PATH:")
    # for i in range(5):
    #     print(track.center.getPoint(i))

    # --------------------- ----
    # Generate centerline array
    # -------------------------
    center = [] # array of centerline points

    for i in range(track.num_points):
        leftPt = track.left.getPoint(i)
        rightPt = track.right.getPoint(i)

        centerX = (leftPt.x + rightPt.x) / 2
        centerY = (leftPt.y + rightPt.y) / 2

        center.append([centerX, centerY])

    # print("FIRST 5 POINTS OF CENTER PATH:")
    # for i in range(5):
    #     print(track.center.getPoint(i))

    # ------------------------------------
    # Create new track based on centerline
    # ------------------------------------
    track = Track(center)
    track.generateTrack()

    # --------------------
    # Create controller(s)
    # --------------------
    steering_controller = PIDSteeringController(track.center)


    steering_controller.SetGains(Kp=0.4, Ki=0, Kd=0.3)
    steering_controller.SetLookAheadDistance(dist=0.5)

    # steering_controller.initTracker(track.center)

    throttle_controller = PIDThrottleController(track.center)
    throttle_controller.SetGains(Kp=0.3, Ki=0, Kd=0.5)
    throttle_controller.SetTargetSpeed(speed=9.0)#speed set to 9.0 with no obstacle avoidance

    initLoc, initRot = GetInitPose([track.center.x[0],track.center.y[0]], [track.center.x[1],track.center.y[1]], reversed=reversed)

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

        if chrono.vehicle.GetVehicleSpeed() < 8: #7 with no obstacle avoidance
            steering_controller.SetGains(Kp=0.2, Ki=0, Kd=0.6)
        elif chrono.vehicle.GetVehicleSpeed() < 9: #8 with no obstacle avoidance
            steering_controller.SetGains(Kp=0.3, Ki=0, Kd=0.45)
        else:
            steering_controller.SetGains(Kp=0.4, Ki=0, Kd=0.3)

        throttle, braking = throttle_controller.Advance(ch_step_size, chrono)

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
