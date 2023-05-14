The Open Motion Planning Library (OMPL)
=======================================

Linux / macOS [![Build Status](https://travis-ci.org/ompl/ompl.svg?branch=main)](https://travis-ci.org/ompl/ompl)
Windows [![Build status](https://ci.appveyor.com/api/projects/status/valuv9sabye1y35n/branch/main?svg=true)](https://ci.appveyor.com/project/mamoll/ompl/branch/main)

Visit the [OMPL installation page](https://ompl.kavrakilab.org/core/installation.html) for
detailed installation instructions.

OMPL has the following required dependencies:

* [Boost](https://www.boost.org) (version 1.58 or higher)
* [CMake](https://www.cmake.org) (version 3.5 or higher)
* [Eigen](http://eigen.tuxfamily.org) (version 3.3 or higher)

The following dependencies are optional:

* [ODE](http://ode.org) (needed to compile support for planning using the Open Dynamics Engine)
* [Py++](https://github.com/ompl/ompl/blob/main/doc/markdown/installPyPlusPlus.md) (needed to generate Python bindings)
* [Doxygen](http://www.doxygen.org) (needed to create a local copy of the documentation at
  https://ompl.kavrakilab.org/core)

Once dependencies are installed, you can build OMPL on Linux, macOS,
and MS Windows. Go to the top-level directory of OMPL and type the
following commands:

    mkdir -p build/Release
    cd build/Release
    cmake ../..
    # next step is optional
    make -j 4 update_bindings # if you want Python bindings
    make -j 4 # replace "4" with the number of cores on your machine

# Contact-aware planning

## Implementation

The implementation mianly includes the following

1. Create a new type of cost function using simplified_elastic_band_model (`src/ompl/base/objectives/src/DeformedPathOptimizationObjective.cpp`)

2. Modify BiTRRT algorithm in `src/ompl/geometric/planners/rrt/BiTRRT.hdistanceFunction`

    (a) Add the input of the position of the elastic band

    (b) Modify the `distanceFunction`

    (c) Modify the condition that two tree merges 

    (d) use the cost `DeformedPathOptimizationObjective`

## Usage

1. You should first change the path defined in the `ompl/CMakeLists.txt`:
set the `CONTACT_DETECTION_PATH` to the path of the contact_detection package.

2. To use moveit with local installed ompl library, you need to remove the default-installed ompl version inside ros, see https://github.com/ros-planning/moveit_ros/issues/623  and https://ompl.kavrakilab.org/buildSystem.html 



    cd build/Release
    cmake ../.. -DPYTHON_EXEC=/usr/bin/python${PYTHONV}
    make
    sudo make install

Note that, after the modification of the contact_detection package, you should build and install ompl first, then build your ros workspace. 

