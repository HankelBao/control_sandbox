from control_utilities.chrono_wrapper import ChronoWrapper
from control_utilities.chrono_vehicle import ChronoVehicle
from control_utilities.chrono_terrain import ChronoTerrain
from control_utilities.chrono_utilities import calcPose, createChronoSystem, setDataDirectory, calcRandomPose
from control_utilities.track import RandomTrack, Track
from control_utilities.matplotlib_wrapper import MatplotlibWrapper
from control_utilities.opponent import Opponent

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
    reversed = 1 # random.randint(0,1)
    track = RandomTrack(width=20)
    track.generateTrack(seed=seed, reversed=reversed)
    print('Using seed :: {}'.format(seed))

    # --------------------
    # Create controller(s)
    # --------------------
    lat_controller = PIDLateralController(track.center)
    lat_controller.SetGains(Kp=0.4, Ki=0, Kd=0.25)
    lat_controller.SetLookAheadDistance(dist=5)

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

    # ------------------
    # Create opponent(s)
    # ------------------
    opponents = []
    n = 1
    for i in range(n):
        opponent_track = Track(track.center.waypoints, width=track.width/2)
        opponent_track.generateTrack()
        if i % 3 == 0:
            opponent_path = opponent_track.left
        elif i % 3 == 1:
            opponent_path = opponent_track.right
        else:
            opponent_path = opponent_track.center

        opponent_lat_controller = PIDLateralController(opponent_path)
        opponent_lat_controller.SetGains(Kp=0.4, Ki=0, Kd=0.25)
        opponent_lat_controller.SetLookAheadDistance(dist=5)

        opponent_long_controller = PIDLongitudinalController()
        opponent_long_controller.SetGains(Kp=0.4, Ki=0, Kd=0)
        opponent_long_controller.SetTargetSpeed(speed=5.0)

        opponent_controller = PIDController(opponent_lat_controller, opponent_long_controller)

        opponent_initLoc, opponent_initRot = calcRandomPose(opponent_path, s_min=opponent_path.s[100 + i * 100], s_max=opponent_path.s[100 + i * 100])
        opponent_vehicle = ChronoVehicle(ch_step_size, system, opponent_controller, irrlicht=irrlicht, vehicle_type='json', initLoc=opponent_initLoc, initRot=opponent_initRot)
        opponent_vehicle.SetTerrain(terrain)
        opponent = Opponent(opponent_vehicle, opponent_controller)

        opponents.append(opponent)

    controller.SetOpponents(opponents)

    # Create chrono wrapper
    chrono_wrapper = ChronoWrapper(ch_step_size, system, track, vehicle, terrain, irrlicht=irrlicht, opponents=opponents, draw_barriers=True)

    if matplotlib:
        matplotlib_wrapper = MatplotlibWrapper(mat_step_size, vehicle, opponents=opponents)
        matplotlib_wrapper.plotTrack(track)

    ch_time = mat_time = 0
    while True:
        if chrono_wrapper.Advance(ch_step_size) == -1:
            chrono_wrapper.Close()
            break

        controller.Advance(ch_step_size, vehicle, perception_distance=30)
        for opponent in opponents:
            opponent.Update(ch_time, ch_step_size)

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
