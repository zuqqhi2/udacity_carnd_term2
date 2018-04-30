# Extended Kalman Filter Project

Simulator is [here](https://github.com/udacity/self-driving-car-sim/releases/).

## How to compile & run

    # Prepare env
    docker run -it -p 4567:4567 -v `pwd`:/work udacity/controls_kit:latest
    git clone https://github.com/zuqqhi2/udacity_carnd_term2.git

    # Compile
    cd extended_kalman_filter
    mkdir build
    cd build
    cmake ..
    make

    # Run
    ./ExtendedKF

    # Unit Test by https://github.com/catchorg/Catch2
    ctest

    # Lint
    cpplint `find ./src -name *.cpp` 2>&1 | cat > cpplint.xml


## Accuracy of test data

|Variable|RMSE Target|Dataset1 Result|Extended Kalman Filter Result|
|:------:|:---------:|:-------------:|:---------------------------:|
|px      |< 0.09     |0.0624         |0.0973                       |
|py      |< 0.10     |0.0865         |0.0854                       |
|vx      |< 0.40     |0.2942         |0.4512                       |
|vy      |< 0.30     |0.2659         |0.4396                       |

[dataset1]: ./images/dataset1-result.png "Accuracy result of dataset1"
[dataset2]: ./images/dataset2-result.png "Accuracy result of dataset2"

![Accuracy result of dataset1][dataset1]

![Accuracy result of dataset2][dataset2]
