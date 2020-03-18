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
    matplotlib = 1
    irrlicht = 0

    import matplotlib.pyplot as plt
    plt.figure()

    # Chrono simulation step size
    ch_step_size = 1e-2
    # Matplotlib visualization step size
    mat_step_size = 1e-2

    # ------------
    # Create track
    # ------------
    createRandomTrack = False
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

    solution_t = []
    for _ in range(100):
        stable_path = ga_search(segmentation)
        solution_t.append(np.sum(stable_path.t))
        print(np.array(solution_t))

    print("Mean: "+str(np.mean(solution_t)))
    print("Var: "+str(np.var(solution_t)))
    print("Min: "+str(np.min(solution_t)))
    print("Max: "+str(np.max(solution_t)))

if __name__ == "__main__":
    main()
