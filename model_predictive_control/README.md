# MPC Control Project

Controling Vehicle steer and throttle by MPC.

Simulator is [here](https://github.com/udacity/self-driving-car-sim/releases/).

## Result
[resultgif]: ./images/result.gif "Result"

Result youtube movie link

[![MPC Control Result(Youtube)][resultgif]](https://youtu.be/8mGQTRtXdIE)

## Description
[model-state]: ./images/model_state.gif "State Vector"
[act-steer]: ./images/act_steer.gif "Steering Angle"
[act-throttle]: ./images/act_throttle.gif "Throttle"
[model-x]: ./images/model_x.gif "Equation X"
[model-y]: ./images/model_y.gif "Equation Y"
[model-psi]: ./images/model_psi.gif "Equation Psi"
[model-v]: ./images/model_v.gif "Equation V"
[model-cte]: ./images/model_cte.gif "Equation CTE"
[model-epsi]: ./images/model_epsi.gif "Equation EPsi"
[Lf]: ./images/lf.gif "Lf"
[func-fx]: ./images/func_fx.gif "Function f"
[func-dfx]: ./images/func_dfx.gif "Fuction df"

### Model

I used following vecotr for state.

![State Vector][model-state]

For actuator, I used followings.

![act-steer][act-steer]

![act-throttle][act-throttle]

And then followings are update equations.

![Equation X][model-x]

![Equation Y][model-y]

![Equation Psi][model-psi]

![Equation V][model-v]

![Equation CTE][model-cte]

![Equation EPsi][model-epsi]

![Lf][lf] is the distance between the front of the vehicle and its center of gravity.

I used 3rd degree polynomial for fitting curve between ground truth and vehicle trajectory.

![Function fx][func-fx], ![Function dfx][func-dfx]



### Timestep Length and Elapsed Duration (N & dt)

[hn-hdt]: ./images/highn-highdt-result.gif "High N, High dt"
[ln-ldt]: ./images/lown-lowdt-result.gif "Low N, Low dt"
[hn-mdt]: ./images/highn-middt-result.gif "High N, Middle dt"
[mn-ldt]: ./images/midn-lowdt-result.gif "Middle N, Low dt"
[mn-hdt]: ./images/midn-highdt-result.gif "Middle N, High dt"

If N times dt equals 10, it means the controller uses data between current state and 10 sec future to find best actuator values.

Both of too short term and too long term cause of bad accuracy. If the controller uses short term data the controller cannot treat sharp curve. If the controller uses long term data fitting will be complex and sensor value may be not accurate in real world.

I tried many patterns but I'll show 1 good pattern and 5 extreme examples. And I chosen 1st one(N=15, dt=0.1) because it works.

|N   |dt    |Result |
|:--:|:----:|:-----:|
|15  |0.1   |![best][resultgif]|
|50  |1.0   |![High N, High dt][hn-hdt]|
|5   |0.01  |![Low N, Low dt][ln-ldt]|
|30  |0.1   |![High N, Middle dt][hn-mdt]|
|15  |0.01  |![Middle N, Low dt][mn-ldt]|
|15  |0.5   |![Middle N, High dt][mn-hdt]|


### Polynomial Fitting and MPC Preprocessing

[waypoints]: ./images/waypoints_transform.gif "Waypoints Transformation"

Given waypoints are on map coordinate not vehicle coordinate. So, I used following equation to get vehicle coordinate waypoints.

![Waypoints Transformation][waypoints]

wx means waypoints x axis and x means vehicle x position.

### Model Predictive Control with Latency

The simulator has 100 milliseconds delay for actuator. To adopt it, I used 100 milliseconds future state as current state and passed it to solver.

Following part in `main.cpp` estimates 100 milliseconds future state.

```cpp
//=====================================
// Estimate 100msec future state
// Line fitting with ground truth with 3rd order
auto coeffs = polyfit(way_x, way_y, 3);
// Get current steering and throttle to fit delay
double cur_delta = j[1]["steering_angle"];
double cur_a = j[1]["throttle"];
// Estimate future state by kinematic model
double new_x = v * actuator_delay_sec;
double new_y = 0;
double new_psi = -v * cur_delta * actuator_delay_sec / mpc.Lf;
double new_v = v + cur_a * actuator_delay_sec;
double new_cte = coeffs[0];
double new_epsi = -atan(coeffs[1]) -
                  (v * atan(coeffs[1]) * actuator_delay_sec / mpc.Lf);
```


## How to compile & run

    # Prepare env
    docker run -it -p 4567:4567 -v `pwd`:/work udacity/controls_kit:latest
    git clone https://github.com/zuqqhi2/udacity_carnd_term2.git

    # Compile & Run
    cd udacity_carnd_term2
    cd model_predictive_control
    mkdir build
    cd build
    cmake ..
    make
