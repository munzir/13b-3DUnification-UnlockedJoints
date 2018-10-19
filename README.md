# Whole Body Control of Krang Simulation

## Setup

* Install [DART] (https://dartsim.github.io/install_dart_on_ubuntu.html)
* Install [Qt](https://www.qt.io/download-open-source/)
* Install [config4cpp](http://www.config4star.org/)
* Clone repo [09-URDF](https://github.gatech.edu/WholeBodyControlAttempt1/09-URDF)
* Clone this repository

## Use
Before compiling the code, the local absolute path to this repo and to repo 09-URDF needs to be specified at various locations in the source files: examples/3dofddp/3dof.cpp (7 different places) and examples/3dofddp/Controller.cpp (1 place). You can find these places by searching for "13b-3DUnification" and "09-URDF" in these sources files. 

To compile:

    rm -r build
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make

To run:

    cd examples/3dofddp 
    ./3dofddp
