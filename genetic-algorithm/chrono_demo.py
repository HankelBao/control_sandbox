from control_utilities.chrono_wrapper import ChronoWrapper
from control_utilities.chrono_vehicle import ChronoVehicle
from control_utilities.chrono_terrain import ChronoTerrain
from control_utilities.chrono_utilities import calcPose, createChronoSystem, setDataDirectory
from control_utilities.track import RandomTrack, Track
from control_utilities.path import Path
from control_utilities.segmentation import Segmentations
from control_utilities.matplotlib_wrapper import MatplotlibWrapper

from ga import GAConfig, GAPathGenerator, TrackPath, ga_search
from pid_controller import PIDController, PIDLateralController, PIDLongitudinalController

import random
import sys
import time
import numpy as np

def main():
    if len(sys.argv) == 2:
        seed = int(sys.argv[1])
    else:
        seed = random.randint(0,100)

    # Render preferences
    matplotlib = 0
    irrlicht = 1

    import matplotlib.pyplot as plt
    plt.figure()

    # Chrono simulation step size
    ch_step_size = 1e-2
    # Matplotlib visualization step size
    mat_step_size = 1e-2

    # ------------
    # Create track
    # ------------
    createRandomTrack = True
    track = None

    if createRandomTrack:
        reversed = random.randint(0,1)
        track = RandomTrack(x_max=50, y_max=50)
        track.generateTrack(seed=seed, reversed=reversed)
    else:
        print('Using seed :: {}'.format(seed))
        points = [
            [49.8, 132.9],
            [60.3, 129.3],
            [75.6, 129.0],
            [87.9, 131.7],
            [96.9, 129.6],
            [111.0, 120.0],
            [115.2, 110.7],
            [120.6, 96.9],
            [127.8, 88.5],
            [135.9, 77.4],
            [135.9, 65.1],
            [133.2, 51.3],
            [128.4, 43.2],
            [119.7, 36.3],
            [105.0, 35.7],
            [90.0, 36.3],
            [82.5, 46.2],
            [82.5, 63.6],
            [83.4, 82.2],
            [77.1, 93.9],
            [61.2, 88.5],
            [55.5, 73.5],
            [57.9, 54.6],
            [66.6, 45.0],
            [75.9, 36.3],
            [79.2, 25.5],
            [78.0, 13.2],
            [65.1, 6.0],
            [50.7, 6.0],
            [36.6, 11.7],
            [29.1, 21.3],
            [24.0, 36.9],
            [24.0, 56.1],
            [29.1, 70.8],
            [24.9, 77.7],
            [13.5, 77.7],
            [6.3, 81.6],
            [5.7, 92.7],
            [6.3, 107.7],
            [8.7, 118.2],
            [15.3, 122.7],
            [24.3, 125.4],
            [31.2, 126.0],
            [40.8, 129.6],
            [49.8, 132.9],
        ]
        track = Track(points)
        track.generateTrack()

    segmentation = Segmentations(track)
    segmentation.create_segmentations()

    stable_path = ga_search(segmentation)
    stable_path.generate_final_path()

    plt.clf()
    stable_path.generate_final_path()
    path = stable_path.final_path
    path.update_vmax()
    path.update_profile()
    #path.plot_speed_profile()
    # plt.show()
    # path.v_max = v
    # plt.pause(1)

    # --------------------
    # Create controller(s)
    # --------------------

    # Lateral controller (steering)
    lat_controller = PIDLateralController(path)
    lat_controller.SetGains(Kp=1.9, Ki=0.000, Kd=0.40)
    lat_controller.SetLookAheadDistance(dist=5)

    # Longitudinal controller (throttle and braking)
    long_controller = PIDLongitudinalController(path)
    long_controller.SetGains(Kp=0.4, Ki=0, Kd=0)
    long_controller.SetLookAheadDistance(dist=path.ps[0]*2)

    # PID controller (wraps both lateral and longitudinal controllers)
    controller = PIDController(lat_controller, long_controller)

    # ------------------------
    # Create chrono components
    # ------------------------
    setDataDirectory()

    # Create chrono system
    system = createChronoSystem()

    # Calculate initial position and initial rotation of vehicle
    initLoc, initRot = calcPose(path.points[0], path.points[1])
    # Create chrono vehicle
    vehicle = ChronoVehicle(ch_step_size, system, controller, irrlicht=irrlicht, vehicle_type='json', initLoc=initLoc, initRot=initRot, vis_balls=True)

    # Create chrono terrain
    terrain = ChronoTerrain(ch_step_size, system, irrlicht=irrlicht, terrain_type='concrete')
    vehicle.SetTerrain(terrain)

    # Create chrono wrapper
    chrono_wrapper = ChronoWrapper(ch_step_size, system, track, vehicle, terrain, irrlicht=irrlicht, draw_barriers=True)

    if matplotlib:
        matplotlib_wrapper = MatplotlibWrapper(mat_step_size, vehicle, render_step_size=1.0/20)
        path.plot(color='-b', show=False)
        matplotlib_wrapper.plotTrack(track)

    ch_time = mat_time = 0
    updates = total_time = 0.0
    while True:
        # if vehicle.vehicle.GetVehicleSpeed() < 7:
        #     lat_controller.SetGains(Kp=0.2, Ki=0, Kd=0.6)
        # elif vehicle.vehicle.GetVehicleSpeed() < 8:
        #     lat_controller.SetGains(Kp=0.3, Ki=0, Kd=0.45)
        # else:
        #     lat_controller.SetGains(Kp=0.4, Ki=0, Kd=0.3)

        if chrono_wrapper.Advance(ch_step_size) == -1:
            chrono_wrapper.Close()
            break

        # Update controller
        controller.Advance(ch_step_size, vehicle)

        if matplotlib and ch_time >= mat_time:
            if not matplotlib_wrapper.Advance(mat_step_size, save=False):
                print("Quit message received.")
                matplotlib_wrapper.close()
                break
            mat_time += mat_step_size

        ch_time += ch_step_size

        if ch_time > 300:
            break
        updates += 1.0
    # print('Update Summary:\n\tChrono Time :: {0:0.2f}\n\tTime per Update :: {1:0.5f}'.format(ch_time, total_time / updates))
    print("Exited")
    pass

if __name__ == "__main__":
    main()
