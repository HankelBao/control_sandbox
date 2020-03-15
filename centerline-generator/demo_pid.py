from control_utilities.chrono_wrapper import ChronoWrapper
from control_utilities.chrono_vehicle import ChronoVehicle
from control_utilities.chrono_terrain import ChronoTerrain
from control_utilities.chrono_utilities import calcPose, createChronoSystem, setDataDirectory
from control_utilities.track import RandomTrack, Track
from control_utilities.path import Path
from control_utilities.matplotlib_wrapper import MatplotlibWrapper

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

    # Lateral controller (steering)
    lat_controller = PIDLateralController(track.center)
    lat_controller.SetGains(Kp=0.4, Ki=0, Kd=0.25)
    lat_controller.SetLookAheadDistance(dist=5)

    # Longitudinal controller (throttle and braking)
    long_controller = PIDLongitudinalController(track.center)
    long_controller.SetGains(Kp=0.4, Ki=0, Kd=0.45)
    long_controller.SetLookAheadDistance(dist=5)
    long_controller.SetTargetSpeed(speed=10.0)

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
    chrono_wrapper = ChronoWrapper(ch_step_size, system, track, vehicle, terrain, irrlicht=irrlicht, draw_barriers=True)

    if matplotlib:
        matplotlib_wrapper = MatplotlibWrapper(mat_step_size, vehicle, render_step_size=1.0/20)
        matplotlib_wrapper.plotTrack(track)

    ch_time = mat_time = 0
    while True:
        # Update controllers
        if vehicle.vehicle.GetVehicleSpeed() < 7:
            lat_controller.SetGains(Kp=0.2, Ki=0, Kd=0.6)
        elif vehicle.vehicle.GetVehicleSpeed() < 8:
            lat_controller.SetGains(Kp=0.3, Ki=0, Kd=0.45)
        else:
            lat_controller.SetGains(Kp=0.4, Ki=0, Kd=0.3)

        # Update controller
        controller.Advance(ch_step_size, vehicle)

        if chrono_wrapper.Advance(ch_step_size) == -1:
            chrono_wrapper.Close()
            break

        if matplotlib and ch_time >= mat_time:
            if not matplotlib_wrapper.Advance(mat_step_size):
                print("Quit message received.")
                matplotlib_wrapper.close()
                break
            mat_time += mat_step_size

        ch_time += ch_step_size

        if ch_time > 50:
            break
    print("Exited")
    pass

if __name__ == "__main__":
    main()
