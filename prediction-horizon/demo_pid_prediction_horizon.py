from control_utilities.chrono_wrapper import ChronoWrapper
from control_utilities.chrono_vehicle import ChronoVehicle
from control_utilities.chrono_terrain import ChronoTerrain
from control_utilities.chrono_utilities import calcPose, createChronoSystem, setDataDirectory
from control_utilities.track import RandomTrack
from control_utilities.matplotlib import MatSim

from pid_controller import PIDController, PIDLateralController, PIDLongitudinalController
from pid_prediction_horizon_controller import PIDPredictionHorizon

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

    # --------------------
    # Create controller(s)
    # --------------------
    prediction_horizon =  PIDPredictionHorizon(track)

    # Lateral controller (steering)
    lat_controller = PIDLateralController(track.center)
    lat_controller.SetGains(Kp=0.4, Ki=0, Kd=0.25)
    lat_controller.SetLookAheadDistance(dist=5)

    # Longitudinal controller (throttle and braking)
    long_controller = PIDLongitudinalController()
    long_controller.SetGains(Kp=0.4, Ki=0, Kd=0)
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

    # mat = MatSim(mat_step_size)

    ch_time = mat_time = 0
    while True:
        if chrono_wrapper.Advance(ch_step_size) == -1:
            chrono_wrapper.Close()
            break

        # Update controller
        controller.Advance(ch_step_size, vehicle)
        prediction_horizon.Advance(ch_step_size, vehicle)

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
