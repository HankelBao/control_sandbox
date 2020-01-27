# Matplotlib Simulator

The main goal of this simulator is to provide a robust and lightweight simulation environment to test controls algorithms.

This is being developed by [Wisconsin Autonomous](https://wisconsinautonomous.org/), a student organization at the University of Wisconsin - Madison.

## Installation
To use the matplotlib-simulator, it is recommended to install it as a local python module. This takes a simple command:
```
python3 setup.py install --user
```
Note: PyChrono must be installed with python3
Note #2: To correctly install on Mac, I had to add the additional `--prefix=` command (literally nothing after the "=").

### Mac Installation of PyChrono
For Mac, the anaconda installation of PyChrono does not work. You get a segmentation fault error when importing the module. As a result, ProjectChrono must be installed from source. Below are detailed instructions and commands needed to correctly install this software.

1. Clone ProjectChrono locally
```
git clone https://github.com/projectchrono/chrono.git && cd chrono
```
2. Create a build directory
```
mkdir build && cd build
```
3. Use cmake and make to build the project
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
  -DPYTHON_INCLUDE_PATH:FILEPATH=/usr/local/Frameworks/Python.framework/Versions/3.7/include/python3.7m \
  -DPYTHON_LIBRARY:FILEPATH=/usr/local/Frameworks/Python.framework/Versions/3.7/lib/libpython3.7.dylib \
  .. \
  && make -j12
```
Note: substitute 12 at the end of the command with however many cores you would like to use to build chrono. I typically use all my available cores, i.e. 12.
Note #2: Make sure the last two flags point to a correct version of python on your local machine. Must be the same version of python. Could be streamlined if a python environment is used.

Stay up to date with our technical info by following our [blog](https://www.wisconsinautonomous.org/blog).

Follow us on [Facebook](https://www.facebook.com/wisconsinautonomous/), [Instagram](https://www.instagram.com/wisconsinautonomous/), and [LinkedIn](https://www.linkedin.com/company/wisconsin-autonomous/about/)!

<img src="https://github.com/WisconsinAutonomous/wa-resources/blob/master/Images/WA.png" alt="Wisconsin Autonomous Logo" height="100px">  <img src="https://github.com/WisconsinAutonomous/wa-resources/blob/master/Images/UWCrest.png" alt="University of Wisconsin - Madison Crest" height="100px" align="right">
