# Term 2 Project 5 - Controls MPC [![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

![demo](demo.gif)

* Yellow line: reference line
* Green line: predicted line that is guiding the vehicle

## Overview

In this project, I implement Model Predictive Control to drive the car around the track in simulator. To mimic real driving conditions, there's a 100 millisecond latency between actuations commands on top of the connection latency.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./mpc

## Other Important Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points
### Compiling
#### Your code should compile.

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./mpc`

### Implementation
#### The Model

To use Model Predictive Control, We need define our controlling model including the state, actuators and update equations.

We present the vehicle using 4 variables:
* position - `x, y`
* orientation - `psi`
* velocity - `v`

We also capture two types of errors for each state:
* Distance of vehicle from trajectory - `cte`
* Distance of vehicle orientation and trajectory orientation - `epsi`

Finally, we define two types of actuators to capture how the states evolves over time:
* steering angle - delta
* acceleration with negative values as braking - a

So, the final model looks like following:

```
State: [x, y, psi, v]
Control Inputs: [delta, a]
```

Update equations:

```
x = x + v * cos(psi) * dt
y = y + v * sin(psi) * dt
psi = psi - v * delta / Lf * dt
v = v + a * dt

where Lf is the length of vehicle from front to CoG that has a similar radius.
```

This model is called Global Kinematic Model.

#### Timestep Length and Elapsed Duration (N & dt)

Here we define two control variables for the predictive control using model above:
* Number of future states - `N`
* Elapsed duration between two states - `dt`

The larger of `N`, the more future states to predict, controller will able to model more complex future trajectory and act accordingly. However, more future states will result in longer computational time which effectively increase the latency. The smaller of `dt`, the higher density of future states which gives better accuracy. However, it will shorten the total length of future prediction, so controller may not have enough information to act proactively, such as braking before sharp turn.

I found that `N=10` and `dt=0.1` with 1s of future prediction is good enough for the controller to capture turns ahead while having a relatively low computational time for each prediction.

#### Polynomial Fitting and MPC Preprocessing

As the simulator provides sensor data, it feeds back global waypoints to the controller. To make our calculation more effective, I first transform global waypoints to vehicle's coordinate system using following equations:

```
dx = waypoint_x - vehicle_x;
dy = waypoint_y - vehicle_y;
transform_x = cos(psi) * dx + sin(psi) * dy;
transform_y = -sin(psi) * dx + cos(psi) * dy;
```

Then we fit a polynomial with order of 3 to transformed waypoints. The generated coefficients is in vehicle's coordinate system and is used by MPC as reference to calculate cost.

Notice that, the first reference state represents current state. Therefore, under vehicle's coordinate system is following:

>`[0, 0, 0, v, c0, -arctan(c1)]`

#### Model Predictive Control with Latency

To mimic real driving conditions, we add a 100 millisecond latency between actuations commands on top of the connection latency.

Two common approaches to take latency into account:
1. Following Global Kinematic Model, we propagate the position of the vehicle forward until the expected time when actuations are expected to have an effect. Then we use this position as the first reference state for the MPC.
2. We directly take latency into account in the MPC optimization problem by constraining the actuators to the values of the previous iteration for the duration of the latency. In other words, current state `t ` depends on the acutators of state `t - 1 - l_ind`, where l_ind is the number of states within the duration of the latency.

```
l_ind = floor(latency / dt)

if (t > l_ind) {
  // previous delta
  delta0 = vars[delta_start + t - 1- l_ind]
  // previous a
  a0 = vars[a_start + t - 1 - l_ind]
}
```

In my implementation, I choose the 2nd apporach because it keeps our nonliner model the same while in the 1st approach, it introduct additional estimation error when we propagate the position.

### Simulation
#### The vehicle must successfully drive a lap around the track.

After tuning the cost function, the controller is able to generate a preditive line which is very close to the reference line and guides the vehicle successfully drive around the track.