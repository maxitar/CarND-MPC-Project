# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc [param_file]`. By default, if no parameter file is provided, the program searches for `../params`. However, if it cannot find this file as well, there are defaults provided in the code.

## Model

I use the kinematic bicycle model given by the following equations (in discretized form)

1. x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub>cos(psi<sub>t</sub>)dt
2. y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub>sin(psi<sub>t</sub>)dt
3. psi<sub>t+1</sub> = psi<sub>t</sub> + v<sub>t</sub>delta<sub>t</sub>)dt/L<sub>f</sub>
4. v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub>dt
5. cte<sub>t+1</sub> = f(x<sub>t</sub>) - y<sub>t</sub> + v<sub>t</sub>sin(epsi<sub>t</sub>)dt
6. epsi<sub>t+1</sub> = psi<sub>t</sub> - psides<sub>t</sub> + v<sub>t</sub>delta<sub>t</sub>)dt/L<sub>f</sub> 

In this model the state is represented by: `x` and `y` are the coordinates of our car, `psi` is its heading, `v` is the velocity, `cte` is the cross-track error, and `epsi` is the orientiation error. Our actuators are `delta` and `a` - the steering angle and acceleration respectively. `Lf` is the distance between the front wheels and the center of gravity of the car. `delta` is restricted between `[-25degrees, 25degrees]` and `a` is restricted between `[-1, 1]`. Note that in the current case, `a` that we get from the simulator is actually not acceleration but throttle, so the model is not exactly correct.

## Timestep Length and Elapsed Duration (N & dt)

I tried different values, but ultimately I chose `N=10` and `dt=0.05`. For large `N`, the solver would sometimes time-out, but another problem was that with large `N`, it was easier to minimize `cte` and `epsi` in the distance, which leaves the car with large velocities on some sharp turns. With values of about 10, I didn't observe such behavior (note though, that my target velocity is 110, so with smaller targets it's perhaps possible to use also bigger `N`s). Surprisingly, for smaller values of `dt` the car was less stable than what I currently use, even if normalized in simulation time, e.g. `N=20` and `dt = 0.025`.

## Polynomial Fitting and MPC Preprocessing

Before fitting a polynomial for the reference trajectory, all the waypoints sent from the simulator are transformed from map coordinates to car coordinates, i.e. the car is at `x=0, y=0` and has `psi=0` angle. To accomplish this, first the car coordinates are subtracted from the map coordinates and then the resulting vectors are rotated in the negative `psi` direction. In code, this is implemented in `main.cpp` in function `map2car` on lines `77--91`.

After the transformation, a third order polynomial is fitted on the waypoints using the function `polyfit` in `main.cpp`.

## Model Predictive Control with Latency

The idea here is to use the equations in the model section unrolled for `N` timesteps as constraints for an optimization problem. Using the `Ipopt` optimization library this can be set up by specifying the cost function that we wish to minimize, the constraints of the problem, and lower and upper bounds for the variables and constraints of the problem. In code, this is most of the `MPC.cpp` file. The constraints and cost function are defined in class `FG_eval`. 

The cost function that I minimize is given by the following code snippet:
```
    for (int t = 0; t < N; ++t) {
      fg[0] += p_.w_cte*CppAD::pow(vars[cte_start + t], 2);
      fg[0] += p_.w_epsi*CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += p_.w_delta_v*CppAD::pow(vars[v_start + t]-p_.v_target, 2);
    }

    for (int t = 0; t < N-1; ++t) {
      fg[0] += p_.w_steer*CppAD::pow(vars[delta_start +t], 2);
      fg[0] += p_.w_acc*CppAD::pow(vars[a_start +t], 2);
    }
    for (int t = 0; t < N-2; ++t) {
      fg[0] += p_.w_steer_delta*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += p_.w_acc_delta*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```
In the first for-loop are the squares of the cross-track error, the orientation error and the difference of the velocity with respect to the target velocity. Using these costs, we try to minimize the distance to our reference trajectory and speed. 

In the second loop we further add the steering angle squared and the acceleration squared. With these costs we try to avoid too fast acceleration and sharp turning angles.

In the final loop we add to the cost the squares of the differences of sequential steering angles and accelerations. In this way, we can reduce the wiggling around the reference trajectory and constant switching between full throttle and full reverse.

Taking the above in consideration, I ended up with the following weights for each of the terms

```
v_target       110. // target velocity (mph)
w_cte          1.0  // cost weight for cte^2
w_epsi         1.0  // cost weight for epsi^2
w_delta_v      1e-4 // cost weight for (v - v_target)^2
w_steer        100. // cost weight for delta^2
w_acc          0.   // cost weight for a^2
w_steer_delta  500  // cost weight for (delta_t - delta_{t-1})^2
w_acc_delta    1e-2 // cost weight for (a_t - a_{t-1})^2
```

My target velocity is 110mph. In experiments using manual controls in the simulator, the maximum speed I could attain was about 105mph, so this should be enough. 
`w_steer` and `w_steer_delta` were chosen such that I get as little wiggling of the car as possible.
`w_delta_v` was chosen so much smaller than `w_cte` and `w_epsi`, so that reducing the errors is much preferable to attaining the maximum speed. This allows for slowdowns on sharp turns. `w_acc_delta=0.01` avoids too strong deceleration on these turns. In my experiments `w_acc=0` since this term only reduced my maximum speed, without bringing any additional stability to the race car (and in some cases it was even the opposite).

To account for the latency of `100ms`, I take the initial position of the car `x=0, y=0, psi=0` and solve the equations from the model section with `dt=0.1`. The steering angle and throttle are read from the data sent from the simulator. This is on lines `144--150` in `main.cpp`.

An interesting point is that in my experiments the most stable controls were not given by the first elements of the control vectors of the solution, but by the second. This might be due to some additional latency associated with my setup, so I left it as the controllable parameter `control_idx`. Actually, when using `dt=0.1` for the latency compensation, taking the first controls leads to an unstable car. However, surprisingly the simulation was more stable when using `dt=0.07` for the initial latency compensation and I was able to also use the first controls. It is still more stable using the second controls though.

The program can read its parameters from external files, so that the user does not have to recompile on each parameter change. Several sample parameter files are given in the main folder of the project (`params` is a copy of `params_control_1`). `params_control_0` and `params_control_1` showcase the two scenarios from the previous paragraph, i.e. `dt_latency=0.07, control_idx = 0` and `dt_latency=0.1, control_idx = 1`, respectively. Note that as already mentioned, it is possible that there is some additional latency on my system, such that on other machines the above results do not occur.

For reference I am running the simulator on a Windows 10 system and the MPC program on Bash on Windows. My CPU is Intel i7 4770k.
