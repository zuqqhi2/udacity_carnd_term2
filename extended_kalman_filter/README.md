# Extended Kalman Filter Project

Simulator is [here](https://github.com/udacity/self-driving-car-sim/releases/).

## How to compile & run

    # Prepare env
    docker run -it -p 4567:4567 udacity/controls_kit:latest
    git clone https://github.com/zuqqhi2/udacity_carnd_term2.git

    # Compile
    cd extended_kalman_filter
    mkdir build
    cd build
    cmake ..
    make

    # Run
    ./ExtendedKF

## Accuracy of test data

|Variable|RMSE Target|Dataset1 Result|
|:------:|:---------:|:-------------:|
|px      |< 0.11     |0.0973         |
|py      |< 0.11     |0.0854         |
|vx      |< 0.52     |0.4512         |
|vy      |< 0.52     |0.4396         |

[dataset1]: ./images/dataset1-result.png "Accuracy result of dataset1"
[dataset2]: ./images/dataset2-result.png "Accuracy result of dataset2"

![Accuracy result of dataset1][dataset1]

![Accuracy result of dataset2][dataset2]
