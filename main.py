from vehicle import GenericCar
from controller import Controller
from track import Track
from simulator import Simulator

import random

def main():
    track = Track()
    controller = Controller(track)
    controller.SetGains(Kp=5, Ki=0.0, Kd=0.0)
    veh = GenericCar(track.getInitLoc()[0] + 1, track.getInitLoc()[1], track.getInitRot())
    sim = Simulator(track)

    animate = True

    time = 0.0
    step = 1e-1
    while True:
        controller.Advance(veh.state, step)
        throttle, steer, braking = controller.GetInputs()

        veh.Update(throttle, steer, braking, step)

        if animate:
            sim.plot(veh)

if __name__ == "__main__":
    main()
