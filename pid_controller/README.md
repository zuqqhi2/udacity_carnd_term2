# PID Controller Project

Controling Vehicle steer and throttle by PID.

Simulator is [here](https://github.com/udacity/self-driving-car-sim/releases/).

## Result
[resultss]: ./images/result_ss.png "Result"
[pcontrol]: ./images/p_control.gif "P Control"
[pdcontrol]: ./images/pd_control.gif "PD Control"
[pidcontrol]: ./images/pid_control.gif "PID Control"

Final result youtube movie link

[![PID Control Result(Youtube)][resultss]](https://youtu.be/wUM6mPUW7Yc)

### Final Hyperparameters of PID for Steering

I choose following values for hyperparameters.
To find better values, I used **Twiddle**. `find_params_by_twiddle.py` is the utility script to find better values by Twiddle.

|Parameter|Value      |
|:-------:|:---------:|
|Kp       |0.2        |
|Ki       |0.0041     |
|Kd       |2.9469     |

### Final Hyperparameters of PID for Throttle

I choose following values for hyperparameters.
To find better value, I used same way of steering.

|Parameter|Value      |
|:-------:|:---------:|
|Kp       |0.2        |
|Ki       |0.004      |
|Kd       |3.0        |

### Effect of PID Components

Following movie is P control result.
P component can let vehicle keep on center but oscillating. So, speed up causes strong oscillating.
![P Control][pcontrol]

Following movie is PD control result.
D component solves oscillating problem. But, if I always add 0.3 bias to steer, PD control let vehicle keep on center + 0.3 line.
![PD Control][pdcontrol]

Following movie is PID control result.
I component causes oscillating for first some seconds but it solves bias problem(I keep adding 0.3 to steer in following movie as well).
![PID Control][pidcontrol]

Note: I turned off throttle control above movies.

## How to compile & run

    # Prepare env
    docker run -it -p 4567:4567 -v `pwd`:/work udacity/controls_kit:latest
    git clone https://github.com/zuqqhi2/udacity_carnd_term2.git

    # Compile & Run
    cd udacity_carnd_term2
    cd pid_controller
    mkdir build
    cd build
    cmake ..
    make

    # Unit Test by https://github.com/catchorg/Catch2
    # after compile
    cd build
    ctest

    # Find best hyperparameters by Twiddle
    # This requires manual simulator reset sometimes and takes time
    # after compile and run simulator
    apt-get update && apt install -y python3-pip python3-dev
    python3 find_params_by_twiddle.py # for steer
    python3 find_params_by_twiddle.py --throttle # for throttle
