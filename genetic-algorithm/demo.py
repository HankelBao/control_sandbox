from control_utilities.track import RandomTrack, Track, Path
from track_path import TrackPath, GAPathGenerator, GAConfig
from matplotlib import pyplot as plt
import numpy as np
import heapq
import os

def main():
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
    num_points = 90

    track = Track(points, num_points=num_points)
    track.generateTrack()

    plt.ion()

    compare_track = Track(points)
    compare_track.generateTrack()

    config = GAConfig(track)
    generation = 0
    stable_path = None
    stable_generation = 0
    save = False

    while (config.upgradable()):
        print("Upgraded")
        print(config.state)
        config.upgrade()
        generator = GAPathGenerator(track, config)

        while generator.stablized_generation < config.stablized_generation:
            generator.ga_advance()

            generator.plot_best_path()
            compare_track.plot(centerline=False, show=False)

            if stable_path:
                stable_path.plot_path("g-")

            plt.show()

            generation += 1
            if save:
                plt.savefig("fig"+str(generation)+".png")

            plt.pause(0.01)
            plt.clf()

        stable_path = generator.best_path
        config.initial_a = stable_path.a

        stable_path.plot_path("g-")
        compare_track.plot(centerline=False, show=False)
        plt.show()
        if save:
            plt.savefig("fig"+str(generation)+"-stable.png")


    # center = []
    # for i in range(len(largest.x)):
    #     center.append([largest.x[i], largest.y[i]])

    # optimized_path = Path(center)
    # compare_track.plot(centerline=False, show=False)
    # optimized_path.plot(color="g-")
    # plt.pause(10000)


if __name__ == "__main__":
    main()