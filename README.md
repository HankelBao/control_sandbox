# Control Sandbox

The main goal of this repo is to provide a robust and lightweight simulation environment to test controls algorithms.

This is being developed by [Wisconsin Autonomous](https://wisconsinautonomous.org/), a student organization at the University of Wisconsin - Madison.

## Clone repo

First, clone the repository on your local machine. Please ensure you pull the submodule, as well. If you are using a terminal (all of this information is only relevant if you're working on the command line), run this command:
```
git clone --recursive https://github.com/WisconsinAutonomous/control_sandbox.git && cd control_sandbox
```

Note: If the repo was cloned without submodules pulled, run this command:
```
git submodule update --init --recursive
```

## Installation of PyChrono

This simulator requires the [ProjectChrono](http://www.projectchrono.org/) physics simulation engine. Primarily, it uses the python wrapper version, known as [PyChrono](http://www.projectchrono.org/pychrono/).

### For Windows Users

For Windows users, it is recommended to install PyChrono through Anaconda. To install Anaconda, please refer to this [link](https://docs.anaconda.com/anaconda/install/windows/).

Once installed, activate an environment you would like to install PyChrono to. It is recommended to make a new environment and install everything here. [A good resource for help with that.](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html).

Create a conda environment with python3.7
```
conda create -n env python=3.7 && conda activate env
```

Now, in your conda environment, run the following command:
```
conda install -c projectchrono/label/develop pychrono
```

Note: For some, an error occurred saying the package could not be found. If this happens to you, you may need to add the channel `conda-forge` to the install path for conda. To do this, run the following command:
```
conda config --add channels conda-forge
```

### For Mac Users

For Mac, the anaconda installation of PyChrono does not work. You get a segmentation fault error when importing the module. As a result, ProjectChrono must be installed from source. Below are detailed instructions and commands needed to correctly install this software.

Prerequisites: homebrew

Recommendation: When creating a python env, it is recommended to have an Environment directory somewhere on your system. In this tutorial, please cd into that Environment directory.

1. Install xcode and the required packages using homebrew
```
xcode-select --install && homebrew install python swig irrlicht eigen
```
2. Clone ProjectChrono locally
```
git clone https://github.com/projectchrono/chrono.git
```
3. Create an Environment folder
```
mkdir PyEnvs && cd PyEnvs
```
4. Create a python environment and activate it
```
python3.7 -m venv pychrono && source pychrono/bin/activate
```
5. Go to the chrono directory that was created when you cloned it locally

6. Create a build directory
```
mkdir build && cd build
```
7. Use cmake and make to build the project
```
cmake \
  -DCMAKE_BUILD_TYPE:STRING=Release \
  -DCMAKE_C_COMPILER=$(which clang) \
  -DCMAKE_CXX_COMPILER=$(which clang++) \
  -DENABLE_MODULE_POSTPROCESS:BOOL=ON \
  -DENABLE_MODULE_VEHICLE:BOOL=ON \
  -DENABLE_MODULE_IRRLICHT:BOOL=ON \
  -DENABLE_MODULE_PYTHON:BOOL=ON \
  .. \
  && make -j12
```

Note: substitute 12 at the end of the command with however many cores you would like to use to build chrono. I typically use all my available cores, i.e. 12.

Note #2: If have multiple versions of python installed on your system, you must specify the following flags:
```
  -DPYTHON_EXECUTABLE:FILEPATH=/usr/local/bin/python3 \
  -DPYTHON_INCLUDE_DIR:FILEPATH=/usr/local/Frameworks/Python.framework/Versions/3.7/include/python3.7m \
  -DPYTHON_LIBRARY:FILEPATH=/usr/local/Frameworks/Python.framework/Versions/3.7/lib/libpython3.7.dylib
```
These are the flags that were successfully used on a typical mac setup.

#### Verify installation of PyChrono

Verify successful installation with this command:
```
python -c 'import pychrono'
```
If this command runs without error, you're good to go!

## Installation of simulator

To use the simulator, it is recommended to install it as a local python module. You must enter the control_utilities directory and run a simple command:
```
cd control_utilities && easy_install .
```

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
