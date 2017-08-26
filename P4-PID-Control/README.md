## CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

**Reflection of PID**

The goal of this project is to build a PID controller and tune the PID hyperparameters by applying the general processing flow as described in the previous lessons.

* Implement PID updateError and TotalError functions with Kp, Kd, Ki
* manually tune Kp, Kd, Ki parameters to let vehicle in simulator to automatically run on track
* (option) use twiddle algorithm to automatically calculate and adjust Kp, Kd, Ki

[//]: # (Image References)

[image1]: ./images/PID.PNG "PID Control"
[image2]: ./images/no-bias-controllers.PNG "no bias"
[image3]: ./images/biased-controllers.PNG "with biars"

![][image1]
![][image2]
![][image3]

P controller only has Kp, which ajust steer of vehicle in propotion to the Cross Track Error, too large Kp leads to overshooting and oscilations; 

PD controller has Kp and Kd,  in addition to Kp, Kd is the derivative of current CTE and the value of last timestep, Kd is used to reduce oscilations;

PID controller has Kp, Kd and Ki, the Ki is the intergral of all CTEs accumulated, with Ki the average of the whole duration total square error will be reduced to 0 finally 

**Steps to manually tune P4 vehicle in simulator**

1, first try to find a Kp which keeps you car on the track atleast for 2-3 seconds. The car might oscillate but that is fine.

I found Kp = -0.1, Kd = 0, Ki = 0:

[test1](https://youtu.be/WNbz_QOxAZ4)



2, to tune down the oscillation gradually try with increased Kd values, increasing Kd reduces oscillations, whereas increasing Kp increases the magnitude of turning. You need to find a sweet spot where the car turns sufficiently but does not oscillate

I found Kp = -0.1, Kd = -10, Ki = 0:

[test2](https://youtu.be/Lmprfa0rurI)



3, Once you find Kp and Kd values which work well, try very small values of Ki. Don’t be surprised if Ki is many orders of magnitude lower than Kp or Kd.

I found Kp = -0.1, Kd = -10, Ki = -0.001:

[test3](https://youtu.be/38PP9rMj53Y)

with added Ki,  the average of total square error in test2 has been reduced from 0.8  to 0.3 in 2 loops of track.



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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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
