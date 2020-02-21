# Control Sandbox

The main goal of this repo is to provide a robust and lightweight simulation environment to test controls algorithms.

This is being developed by [Wisconsin Autonomous](https://wisconsinautonomous.org/), a student organization at the University of Wisconsin - Madison.

## Installation Guides

I have written detailed installation guides for this repository. Please follow the guide that is relevant to the operating system you're using.

Windows: [Setup Guide](https://github.com/WisconsinAutonomous/control_sandbox/blob/master/WindowsSetup.md)
Unix (Mac and Linux): [Setup Guide](https://github.com/WisconsinAutonomous/control_sandbox/blob/master/UnixSetup.md)

## Demos

These demos are to demonstrate the use case of the control_utilities package.

#### demo_path.py

Demonstrates basic usage for the Path and RandomPathGenerator objects.

#### demo_track.py

Prerequisites: demo_path.py

Demonstrates basic usage for the Track and RandomTrack objects.

## Algorithms

#### pid

Leverages the ProjectChrono software and control_utilities. Uses a very simple PID steering controller and throttle controller to pilot a vehicle around a random course. When run, it can be visualized using irrlicht and/or matplotlib. Matplotlib shows an entire track, however, just the centerline is used.

#### mpc

In progress...

#### pid-control
General purpose PID controller that can be imported as a module and used as an object. Protection for common PID control issues including integral windup and derivative kick. Run as file for demo.

*see [pid-control](https://github.com/WisconsinAutonomous/control_sandbox/tree/master/pid-control)*

<img src="https://github.com/WisconsinAutonomous/control_sandbox/blob/master/pid-control/pid-demo.png" alt="art of flight jump" width="80%">

#### mpc-nus

MPC controller and demo simulator for various obstacle representations.

*see [mpc-nus](https://github.com/WisconsinAutonomous/controls-algorithms/tree/master/mpc-nus)*

<img src="https://github.com/WisconsinAutonomous/control_sandbox/blob/master/mpc-nus/pics/sample-accel.png" alt="mpc_accel.py sample output" width="40%"> <img src="https://github.com/WisconsinAutonomous/controls-algorithms/blob/master/mpc-nus/pics/sample-floorplan.png" alt="mpc_floorplan.py sample output" width="40%">

Stay up to date with our technical info by following our [blog](https://www.wisconsinautonomous.org/blog).

Follow us on [Facebook](https://www.facebook.com/wisconsinautonomous/), [Instagram](https://www.instagram.com/wisconsinautonomous/), and [LinkedIn](https://www.linkedin.com/company/wisconsin-autonomous/about/)!

<img src="https://github.com/WisconsinAutonomous/wa-resources/blob/master/Images/WA.png" alt="Wisconsin Autonomous Logo" height="100px">  <img src="https://github.com/WisconsinAutonomous/wa-resources/blob/master/Images/UWCrest.png" alt="University of Wisconsin - Madison Crest" height="100px" align="right">
