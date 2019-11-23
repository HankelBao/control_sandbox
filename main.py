from vehicle import GenericCar
from controller import SteeringController, SpeedController
from track import Track
from simulator import Simulator

import random

def main():
    track = Track()
    steering_controller = SteeringController(track)
    speed_controller = SpeedController(track)
    steering_controller.SetGains(Kp=5, Ki=0.0, Kd=0.0)
    speed_controller.SetGains(Kp=5, Ki=0.0, Kd=0.0)
    speed_controller.SetTargetSpeed(1)
    veh = GenericCar(track.getInitLoc()[0] + 1, track.getInitLoc()[1], track.getInitRot())
    sim = Simulator(track)

    animate = True

    time = 0.0
    step = 1e-1
    while True:
        steering_controller.Advance(veh.state, step)
        speed_controller.Advance(veh.state, step)

        throttle = speed_controller.throttle
        braking = speed_controller.braking
        steer = steering_controller.steer
        print('Velocity :: {}'.format(veh.state.v))
        print('Throttle :: {}'.format(throttle))
        print('Braking :: {}'.format(braking))

        veh.Update(throttle, steer, braking, step)

        if animate:
            sim.plot(veh)

if __name__ == "__main__":
    main()
