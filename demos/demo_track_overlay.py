import sys

from control_utilities.track import RandomTrack, Track
from control_utilities.path import Path

if __name__ == "__main__":
    if len(sys.argv) == 2:
        seed = int(sys.argv[1])
    else:
        seed = 1.0

    rand_track = RandomTrack()
    rand_track.plot(seed)

