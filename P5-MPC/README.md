# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

**Reflection of Model-Predictive-Control (MPC)**

The goal of this project is to implement Model Predictive Control to drive the car around the track. Need to calculate errors of cte and Orientation Error epsi. Additionally, consider a 100 millisecond latency between actuations commands on top of the connection latency into your implementation.


* Implement MPC and visualize both your reference path and the MPC trajectory path during simulations 
* choose state, inputs, dynamics, constraints for IPOPT to optimize control errors
* manually tune parameters to let your car run safely and as fast as possible



[//]: # (Image References)

[image1]: ./images/vehicle_model.PNG
[image2]: ./images/errors.PNG
[image3]: ./images/MPC_state.PNG
[image4]: ./images/IPOPT_Solver.PNG
[image5]: ./images/Solver_output_actuators.PNG
[image6]: ./images/cte_epsi.PNG
[image7]: ./images/acturator_constraints.PNG
[image8]: ./images/forces.PNG
[image9]: ./images/racing_angle.PNG
[image10]: ./images/tire_model.PNG
[image11]: ./images/smooth_delta.PNG
[image12]: ./images/smooth_delta_difference.PNG
[image13]: ./images/route_timestep.PNG
[image14]: ./images/Ipopt_vars.PNG
[image15]: ./images/Cost.PNG

**MPC model implementation**

1. state discussion

![][image1]

From Kinematic models which are simplifications of dynamic models that ignore tire forces, gravity, and mass, we can define the motion part of vehicles can be decided by px, py, psi and v.

![][image2]
![][image6]


However to keep my car always fit into trajactory of predicted path, I also need to consider cte and orientation error epsi.


2. actuators

![][image8]
![][image9]


The motion of vechicle can be simplied as tire model, which may receive various forces, but can be summurized as controls on speed to accelerate or brake, the a, and controls on angles of tire to slip for turning curve, the delta.

![][image10]
![][image7]

However above tire model has limitations, if vehicle/tire is acting outside this linear range then our assumption/prediction tire model will lack of accuracy. Thus we need to define our control inputs [delta, a] to within [-25rad, 25rad] and [-1, 1].

The goal of Model Predictive Control is to optimize the control inputs: [δ,a], which stand for steer_value and throttle_value, meaning the actuators on angle and speed. 


3. update equations

![][image3]
![][image4]
![][image5]


with input predited state at t+1, an optimizer will tune these inputs, until a low cost vector of control inputs is found. 

The cost can be composed of 3 parts:

1)previously the two errors in our state vector: cte and eψ. Ideally, both of these errors would be 0 - there would be no difference from the actual vehicle position and heading to the desired position and heading.


2)how far away our car runs but cannot maintain at reference speed, accelerations and brakes are accumulated into cost.


3)how smoothly car runs to change angle, delta and delta change frequencies are accumulated

![][image11]
![][image12]


**polynomial fit to waypoints**

auto coeffs = polyfit(ptsx, ptsy, 1);
From simulator messages we can get ptsx, ptsy vectors which are waypoints of the route, so we can:

1.Use polyfit to fit a 3rd order polynomial to the given x and y coordinates representing waypoints.
2.Use polyeval to evaluate y values of given x coordinates, which is the cte error value.

double cte = polyeval(coeffs, x) - y;

Recall orientation error is calculated as follows eψ=ψ−ψdes, where ψdes is can be calculated as arctan(f′(x)).

if coeffs has only 1 order, hence the solution double epsi = psi - atan(coeffs[1]);
else if we use 3 order, i.e. f(x) = coeffs[0] + coeffs[1] * x + coeffs[2] * x * x + coeffs[3] * x * x * x,
then epsi = psi(t==t0) - atan( 3 * coeffs[3] * x0 * x0 + 2 * coeffs[2] * x0 + coeffs[1] ), refer to x0=x(t==t0).



**implement 100ms latency**

In real world if our model only calculate the error with respect to the present state, but the actuation will be performed when the vehicle is in a future (and likely different) state. This can sometimes lead to instability. For example the time elapsed between when you command a steering angle to when that angle is actually achieved. This could easily be modeled by a simple dynamic system and incorporated into the vehicle model. One approach would be running a simulation using the vehicle model starting from the current state for the duration of the latency. The resulting state from the simulation is the new initial state for MPC.
Thus, MPC can deal with latency much more effectively.

Let's take t_latency = 0.1, and recall the equations for the model:

x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v_[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt


we can get 

double latency_x = v * t_latency ;
double latency_y = 0 ;
double latency_psi = v /Lf * delta * t_latency ;
double latency_v = v + a * t_latency ;
double latency_cte = cte + v * sin(epsi) * t_latency ;
double latency_epsi = epsi + v * delta * t_latency / Lf ;

initial state = [ latency_x, latency_y, latency_psi, latency_v, latency_cte, latency_epsi ]


**Non-linear optimizer Ipopt**

Ipopt is the tool we'll be using to optimize the control inputs 
[δ1,a​1,δ2,a2,...,δN−1,aN−1]. It's able to find locally optimal values (non-linear problem!) while keeping the constraints set directly to the actuators and the constraints defined by the vehicle model. Ipopt requires we give it the jacobians and hessians directly - it does not compute them for us. Hence, we need to either manually compute them or have a library do this for us. Luckily, there is a library called CppAD which does exactly this.

input Ipopt Solver with the initial state [x,y,psi,v,cte,epsi] of N timesteps plus actuators[delta,a] of N-1 timesteps long will all be stored into vars vector.

![][image14]

and Ipopt will use CPPAD library and above predition equations to fill up all complete N timesteps states and N-1 actuators and make derivative calculations to get minimum Cost.

We only use the generated contron signals [delta,a] at t=0 current time and feedback to car, instead of feedback all N timesteps controls from t=0 ~ t=N-1.


**tuning N and dt**

The length of this output vector is determined by N as Number of Timesteps:

[δ1,a​1,δ2,a2,...,δN−1,aN−1]

Thus N determines the number of variables optimized by the MPC. This is also the major driver of computational cost.

MPC attempts to approximate a continuous reference trajectory by means of discrete paths between actuations. Larger values of dt result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. This is sometimes called "discretization error".

A good approach to setting N, dt, and T is to first determine a reasonable range for T and then tune dt and N appropriately.


test with N=3, car runs out of right edge of track immediately; => if N too small, means no enough prediction for car to forsee how to adjust its controls to keep on track;

test with N=25, car starts to oscilate and swing distance longer and longer and finally run out of track; => if N too large,  it brings unnecessary long-term optimization calculations for Ipopt but actually car only needs predictions for current timestep;

so N=10 works well. 

test with dt= 0.05, car starts to oscilate more and more drastically, same as the situation when N is too large => too many unnecessary calculations;

test with dt= 0.2, N=10,  at speed 40, the car runs well with latency = 0.1s:

[test1](https://youtu.be/Ff11WI4nHdc)



**Steps to manually tune P5 MPC vehicle for high speed**

1, Now let's raise our reference speed to higher, and accordingly we may need to tune the ratio of each part in cost function:

![][image15]



run with:    :

[test1]()



2, :

run with:   :

[test2]()



3, 



run with:   :

[test3]()



---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
