# Whole Body Control of Krang Simulation

## Setup

* Install [DART] (https://dartsim.github.io/install_dart_on_ubuntu.html)
* Install [Qt](https://www.qt.io/download-open-source/)
* Install [config4cpp](http://www.config4star.org/)
* Clone repo [09-URDF](https://github.gatech.edu/WholeBodyControlAttempt1/09-URDF)
* Clone this repository

## Use
Before compiling the code, specify the path to your local copy of repo 09-URDF in the "urdfpath" entry of the configuration file located at examples/3dofddp/controlParams.cfg. In the same file, there are many other configuration parameters that the user can play with. For now, their meaning will need to be understood by looking at their role in the source code itself.

To compile:

    rm -r build
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make

To run:

    cd examples/3dofddp 
    ./3dofddp
