from control_utilities.chrono import ChronoSim, GetInitPose
from control_utilities.track import RandomTrack
from control_utilities.matplotlib import MatSim
from control_utilities.obstacle import RandomObstacleGenerator

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
    track = RandomTrack()
    track.generateTrack(seed=seed, reversed=reversed)
    print('Using seed :: {}'.format(seed))

    # ----------------
    # Create obstacles
    # ----------------
    # Change n to add more obstacles
    obstacles = RandomObstacleGenerator.generateObstacles(track.center, i_min=100, i_max=250, n=1, seed=seed*random.randint(0,90), reversed=reversed)
    
    print(obstacles) # Is a python dictionary
    # To access: obstacles[<path_index>] = position_vector


    # --------------------
    # Create controller(s)
    # --------------------
    steering_controller = PIDSteeringController(track.center, obstacles)
    steering_controller.SetGains(Kp=0.4, Ki=0, Kd=0.3)
    steering_controller.SetLookAheadDistance(lookDist=5)
    steering_controller.SetObstacleDistance(obsDist=75)
    # steering_controller.initTracker(track.center)

    throttle_controller = PIDThrottleController(track.center)
    throttle_controller.SetGains(Kp=0.4, Ki=0, Kd=0.45)
    throttle_controller.SetLookAheadDistance(dist=5)
    throttle_controller.SetTargetSpeed(speed=6.0)

    initLoc, initRot = GetInitPose([track.center.x[0],track.center.y[0]], [track.center.x[1],track.center.y[1]], reversed=reversed)

    chrono = ChronoSim(
        step_size=ch_step_size,
        track=track,
        initLoc=initLoc,
        initRot=initRot,
        irrlicht=irrlicht,
        obstacles=obstacles,
        vis_balls=True
    )


    mat = MatSim(mat_step_size)


    ch_time = mat_time = 0
    while True:
        # Update controllers

        steering, obstacleInRange, tempObstacleAvoidancePath = steering_controller.Advance(ch_step_size, chrono)
        throttle, braking = throttle_controller.Advance(ch_step_size, chrono, obstacleInRange, tempObstacleAvoidancePath)
        state = chrono.GetState()

        if chrono.vehicle.GetVehicleSpeed() < 7:
            steering_controller.SetGains(Kp=0.2, Ki=0, Kd=0.6)
        elif chrono.vehicle.GetVehicleSpeed() < 8:
            steering_controller.SetGains(Kp=0.3, Ki=0, Kd=0.45)
        else:
            steering_controller.SetGains(Kp=0.4, Ki=0, Kd=0.3)


        chrono.driver.SetTargetSteering(steering)
        chrono.driver.SetTargetThrottle(throttle)
        chrono.driver.SetTargetBraking(braking)

        if chrono.Advance(ch_step_size) == -1:
            chrono.Close()
            break


        if matplotlib and ch_time >= mat_time:
            if mat.plot(track, chrono, obstacles) == -1:
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
