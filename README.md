# Control Sandbox

The main goal of this repo is to provide a robust and lightweight simulation environment to test controls algorithms.

This is being developed by [Wisconsin Autonomous](https://wisconsinautonomous.org/), a student organization at the University of Wisconsin - Madison.

## Clone repo

First, clone the repository on your local machine. Please ensure you pull the submodules, as well. If you are using a terminal (all of this information is only relevant if you're working on the command line), run this command:
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

To test your installation, jump down to [here](#verify-installation-of-pychrono).

### For Mac Users

For Mac, the anaconda installation of PyChrono does not work. You get a segmentation fault error when importing the module. As a result, ProjectChrono must be installed from source. Below are detailed instructions and commands needed to correctly install this software.

Prerequisites: [homebrew](https://brew.sh/) and xcode command line tools

To install xcode, you can run this command `xcode-select --install`

Recommendation: When creating a python env, it is recommended to have an Environment directory somewhere on your system.

1. **Install the required packages using homebrew**
```
homebrew install cmake python swig irrlicht eigen
```
2. **Clone ProjectChrono locally**
```
git clone https://github.com/projectchrono/chrono.git
```
3. **Create an Environment folder**
*NOTE: Steps 3 and 4 are optional.* Everything can be done outside of a python environment, it just makes it a bit cleaner. For those of you interested, a python environment allows you to install python packages specifically in a self contained place. When you exit the env, all of your packages remain, but the rest of your system is unchanged by the installs you made in the env.
```
mkdir PyEnvs && cd PyEnvs
```
4. **Create a python environment and activate it**
```
python3.7 -m venv pychrono && source pychrono/bin/activate && cd ..
```
5. **Go to the chrono directory that was created when you cloned it locally**
```
cd chrono
```
6. **Create a build directory**
```
mkdir build && cd build
```
7. **Use cmake and make to build the project**
```
cmake \
  -DCMAKE_BUILD_TYPE:STRING=Release \
  -DCMAKE_C_COMPILER=$(which clang) \
  -DCMAKE_CXX_COMPILER=$(which clang++) \
  -DENABLE_MODULE_POSTPROCESS:BOOL=ON \
  -DENABLE_MODULE_VEHICLE:BOOL=ON \
  -DENABLE_MODULE_IRRLICHT:BOOL=ON \
  -DENABLE_MODULE_PYTHON:BOOL=ON \
  -DPYTHON_EXECUTABLE:FILEPATH=/usr/local/bin/python3 \
  -DPYTHON_INCLUDE_DIR:FILEPATH=/usr/local/Frameworks/Python.framework/Versions/3.7/include/python3.7m \
  -DPYTHON_LIBRARY:FILEPATH=/usr/local/Frameworks/Python.framework/Versions/3.7/lib/libpython3.7.dylib \
  .. \
  && make -j12
```

Note: substitute 12 at the end of the command with however many cores you would like to use to build chrono. I typically use all my available cores, i.e. 12.

Note #2: For almost all Mac users, you will already have a Python2.7 installed on your system by Apple. As a result, you will need specify the use of Python3 in the last 3 flags of the last command. If you get an error regarding the location of python, please try removing the flags that have to do with python (other than `-DENABLE_MODULE_PYTHON:BOOL`). The command will now look like the following:
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
These were the flags that were successfully used on a typical mac setup and were found to be successful.

8. **Set PYTHONPATH to point to the pychrono files**

Within the same build directory, you must add the pychrono files to the PYTHONPATH. Your PYTHONPATH is basically used when you run a python command to search for files you import. Run the following command to set it. _You must be inside your build directory for this specific command to work_.

_Recommended approach._ This will create the correct PYTHONPATH each time you log onto your terminal. Otherwise, you will have to run a command _everytime_.
```
echo "export PYTHONPATH=$PYTHONPATH:$(pwd)/bin" >> ~/.zshrc && source ~/.zshrc
```
Note: this assumes you are using zsh. Run `echo $0`. If it says `-bash`, replace `~/.zshrc` with `~/.bashrc`.

If you do not want to add it to your `.zshrc` (_not recommended_), just run the following command.
```
export PYTHONPATH=$PYTHONPATH:$(pwd)/bin
```
Note: If you are not in your build directory anymore, please replace `$(pwd)/bin` with the path to chrono/build/bin.

9. **Return to the control_sandbox folder**
Now run the following command to return to the control_sandbox directory.
```
cd ../..
```

To test your installation of PyChrono, jump down to [here](#verify-installation-of-pychrono).

### For Linux Users

Because MacOS is based on a unix system, it is the same installation process. However, the anaconda installation, similar to windows, does work on Linux. Therefore, I recommend choosing either solution and replacing certain commands with your distributions equivalent (i.e. replace `homebrew` with your package manager).

Due to the large number of distributions of linux, it is best to just follow either the mac or windows installation, as they are almost exactly the same. _Also, you're a linux user... you can figure it out_ :wink:.

### Verify installation of PyChrono

Verify successful installation with this command:
```
python -c 'import pychrono'
```
If this command runs without error, you're good to go!

If you get an error, that's no fun. Please fill out an [issue](https://github.com/WisconsinAutonomous/control_sandbox/issues/new).

## Installation of the controls simulator

First, lets ensure you are in the right place. Run the following command which will print out your current working directory. If on windows, run `cd` and on a Unix system (Mac or Linux), run `pwd`.
If the final portion of the output says `/control_sandbox`, then you're good to go.

As seen in the control_utilities folder, there are a few utility files created that help describe the path and generate a simulation. This removes the direct need for all users to interact with the simulation engine directly.

In order to link to these files, there are two solutions. The first is recommended, but both work.

#### **Unix** _Recommanded_: Add the files to your PYTHONPATH
*This is the recommended approach for installing the simulator.* The environment variable PYTHONPATH is used when you run a python command to find files not in the default folder. As a result, if you add the path of the simulator to that environmental variable, it will work without installing!!

To have it added to your PYTHONPATH, run the following command.
_Recommended_ (will run everytime you open your terminal without you explicitly running the command.)
```
echo "export PYTHONPATH=$PYTHONPATH:$(pwd)/control_utilities" >> ~/.zshrc && source ~/.zshrc
```
Note: this assumes you are using zsh. Run `echo $0`. If it says `-bash`, replace `~/.zshrc` with `~/.bashrc`.

If you do not want to add it to your `.zshrc` (_not recommended_), just run the following command.
```
export PYTHONPATH=$PYTHONPATH:$(pwd)/control_utilities
```

#### **Windows** Add the files to your PYTHONPATH
_[Link as reference](https://helpdeskgeek.com/how-to/create-custom-environment-variables-in-windows/)_
1. First, find the current directory
Run `cd` in your command prompt. Copy the output. **_Should end in `\control_sandbox`_**.
2. Open the System Properties dialog, click on Advanced and then Environment Variables
3. Under User variables, click New... and create a variable as described below

Variable name: `PYTHONPATH`
Variable value: `<paste output from 1>\control_utilities\control_utilities`
Ex. Variable value: `C:\Users\user\control_sandbox\control_utilities\control_utilities`

#### Install it to your system _Not recommended_
To use the simulator, it is recommended to install it as a local python module. You must enter the control_utilities directory and run a simple command:
```
cd control_utilities && easy_install .
```
Note: If you get an error, run instead `cd control_utilities && python setup.py install --user --prefix=`.

### **Unix** Link the chrono data directory to the project.
_Only relevant for users who are **not** using anaconda_.
In order to see the simulation in 3D using irrlicht (a 3D visualizer written in C++), control_sandbox must have access to chrono's data directory. Similar to previously run steps, you must use an environmental variable. Run one of the following commands to successfully link to the data directory. _The first is the recommended solution._

To have it added to your CHRONO_DATA_DIR, run the following command.
_Recommended_ (will run everytime you open your terminal without you explicitly running the command.)
```
echo "export CHRONO_DATA_DIR=$(pwd)/chrono/data/" >> ~/.zshrc && source ~/.zshrc
```
Note: this assumes you are using zsh. Run `echo $0`. If it says `-bash`, replace `~/.zshrc` with `~/.bashrc`.

If you do not want to add it to your `.zshrc` (_not recommended_), just run the following command.
```
export CHRONO_DATA_DIR=$(pwd)/chrono/data/
```

### **Windows** Link the chrono data directory to the project.
_[Link as reference](https://helpdeskgeek.com/how-to/create-custom-environment-variables-in-windows/)_
1. First, find the current directory
Run `cd` in your command prompt. Copy the output. **_Should end in `\control_sandbox`_**.
2. Open the System Properties dialog, click on Advanced and then Environment Variables
3. Under User variables, click New... and create a variable as described below

Variable name: `CHRONO_DATA_DIR`
Variable value: `<paste output from 1>\chrono\data\`
Ex. Variable value: `C:\Users\user\control_sandbox\chrono\data\`

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
