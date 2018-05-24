# Kidnapped Vehicle Project

Vehicle state estimation by Particle Filter.

Simulator is [here](https://github.com/udacity/self-driving-car-sim/releases/).

## How to compile & run

    # Prepare env
    docker run -it -p 4567:4567 -v `pwd`:/work udacity/controls_kit:latest
    git clone https://github.com/zuqqhi2/udacity_carnd_term2.git

    # Compile & Run
    cd udacity_carnd_term2
    cd kidnapped_vehicle
    ./clean.sh
    ./build.sh
    ./run.sh

    # Unit Test by https://github.com/catchorg/Catch2
    cd build
    ctest

    # Lint
    cpplint `find ./src -name *.cpp` 2>&1 | cat > cpplint.xml


## Image

[success]: ./images/success.png "Result"
![Result][success]
