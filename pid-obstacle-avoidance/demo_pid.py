from control_utilities.chrono_wrapper import ChronoWrapper
from control_utilities.chrono_vehicle import ChronoVehicle
from control_utilities.chrono_terrain import ChronoTerrain
from control_utilities.chrono_utilities import calcPose, createChronoSystem, setDataDirectory
from control_utilities.obstacle import RandomObstacleGenerator
from control_utilities.track import RandomTrack
from control_utilities.matplotlib import MatSim

from pid_controller import PIDController, PIDLateralController, PIDLongitudinalController

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

    track = RandomTrack(x_max=200, y_max=200, width=20)
    track.generateTrack(seed=seed, reversed=reversed)
    print('Using seed :: {}'.format(seed))

    # ----------------
    # Create obstacles
    # ----------------
    # Change n to add more obstacles

    obstacles = RandomObstacleGenerator.generateObstacles(track.center, i_min=50, i_max=60, n=1, seed=seed*random.randint(0,90), reversed=reversed, movement_rate=5, movement_distance=1)

    print(obstacles) # Is a python dictionary
    # To access: obstacles[<path_index>] = position_vector


    # --------------------
    # Create controller(s)
    # --------------------
    lat_controller = PIDLateralController(track, obstacles)
    lat_controller.SetGains(Kp=0.4, Ki=0, Kd=0.3)
    lat_controller.SetLookAheadDistance(lookDist=5)
    lat_controller.SetObstacleDistance(obsDist=50)

    long_controller = PIDLongitudinalController(track.center)
    long_controller.SetGains(Kp=0.4, Ki=0, Kd=0.45)
    long_controller.SetLookAheadDistance(dist=5)
    long_controller.SetTargetSpeed(speed=6.0)

    # PID controller (wraps both lateral and longitudinal controllers)
    controller = PIDController(lat_controller, long_controller)

    # ------------------------
    # Create chrono components
    # ------------------------
    setDataDirectory()

    # Create chrono system
    system = createChronoSystem()

    # Calculate initial position and initial rotation of vehicle
    initLoc, initRot = calcPose(track.center.points[0], track.center.points[1])
    # Create chrono vehicle
    vehicle = ChronoVehicle(ch_step_size, system, controller, irrlicht=irrlicht, vehicle_type='json', initLoc=initLoc, initRot=initRot, vis_balls=True)

    # Create chrono terrain
    terrain = ChronoTerrain(ch_step_size, system, irrlicht=irrlicht, terrain_type='concrete')
    vehicle.SetTerrain(terrain)

    # Create chrono wrapper
    chrono_wrapper = ChronoWrapper(ch_step_size, system, track, vehicle, terrain, irrlicht=irrlicht, obstacles=obstacles, draw_barriers=True)

    # mat = MatSim(mat_step_size)


    ch_time = mat_time = 0
    while True:
        # Update controller
        controller.Advance(ch_step_size, vehicle)

        if vehicle.vehicle.GetVehicleSpeed() < 7:
            lat_controller.SetGains(Kp=0.2, Ki=0, Kd=0.6)
        elif vehicle.vehicle.GetVehicleSpeed() < 8:
            lat_controller.SetGains(Kp=0.3, Ki=0, Kd=0.45)
        else:
            lat_controller.SetGains(Kp=0.4, Ki=0, Kd=0.3)

        if chrono_wrapper.Advance(ch_step_size) == -1:
            chrono_wrapper.Close()
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
