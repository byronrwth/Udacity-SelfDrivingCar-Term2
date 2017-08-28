# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Code Compilation and Dependencies
In addition to the standard requirements, I have added a GNUplot plotting script
to track the various errors and control values (as seen in video linked below).
GNUplot is called via a system call with a plot script that is provided here. This
feature will work if GNUplot is installed in the system, but since this block is within
a try-catch block, the code should work even if the end user does not have this program
installed.

```
  try {
    /* *************************************
     * Open the plotting utility
     * **requires GNUPLOT and STDLIB.H**
     * *************************************
     */
    system("gnuplot postprocess.plt > /dev/null 2>&1 &");
  } catch (...) {
    std::cout << "**Error** requires GNUPLOT and STDLIB.H (for running system commands)";
  }
```

*Note*: Since this plotting utility is run in the background, rarely it may end up as a zombie
process that doesn't die with the main pid program termination. In such an instance, the user
will have to manually kill the gnuplot process

## Final Parameters and Result

In this project, I ended up using a PID controller for throttle and another one
for steering control. The final set of parameters that worked are as follows:

### Throttle PID

* Kp = 0.11
* Ki = 0
* Kd = 0.6
* t_ref = 0.45 (Reference throttle value)

Please see below section on tuning throttle parameters

### Steering PID

* Kp = 0.058
* Ki = 0.003
* Kd = 0.6

*Note*: The above set of parameters worked on my machine with screen resolution of 1024x768
and graphics quality set to *Fastest*. It did not work as smoothly when I changed the
graphics quality to *Fantastic*. These parameters may also depend on the hardware of the end
user (which seems to affect latency).

### Final Result

<a href="http://www.youtube.com/watch?feature=player_embedded&v=wyUB3-l_bt8"
target="_blank"><img src="http://img.youtube.com/vi/wyUB3-l_bt8/0.jpg"
alt="Track 1 Performance" width="480" height="360" border="10" /></a>

## Reflection: Tuning PID Parameters

Overall I used a variation of [Ziegler-Nichols](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method) to
tune the PID parameters. Described below is the overall process workflow that I ended up using.

### Tuning Throttle Control

I worked on the throttle PID to get the car to slow-down/brake as the absolute value of CTE
  increased. I set the Ki parameter to 0 as there was no bias/drift involved here and played with
  Kp and Kd parameters to get a good speed profile around the track.

For the final result, I ended up using a reference throttle value of 0.45 (max speed = 45 mph) as this
resulted in a lower maximum CTE around the lap. Below plots show a variation in various errors and control
parameters as the reference throttle was changed from 0.45 to 0.50 (max speed = 50 mph). At the higher
speed the max CTE was higher and also the car was wavering with a higher amplitude. In future, I will
work on tuning the PID parameters further so that I can achieve a higher reference speed with greater
stability.

**Variations of Throttle Reference Value (45 mph vs. 50 mph)**
<center><img src="./build/throttle-variation-45-50.png" alt="Throttle Control" style="width: 400px;"/></center>

### Tuning Steering Control

I used Ziegler-Nichol's method to obtain a working set of Steering PID parameters until
    I was able to get around the track. This involved setting all the parameters to 0. Then
    increasing only the Kp parameter until I am able to get it completing (although oscillatory)
    around the lap. Then I increased the Kd parameter until I was able to reduce the oscillations
    and the max CTE. The Ki parameter was chosen based on the accumulating I-error as seen
    in the charts.

Below chart shows the variation of Kd parameter (around the final set of parameters) and the effect
it has on the various errors and control parameters. As shown in the graphs, although Kd = 0.8 resulted
in the lowest max CTE, Kd = 0.6 provided a better (or smoother) performance on average and also
resulted in a higher average speed. 

**Variations of Kd parameter**
<center><img src="./build/compare_d.png" alt="Kd parameter" style="width: 400px;"/></center>

With slight variations of Kp parameter around 0.058 this value is a local minumum (optimum) in
terms of resulting in the lowest max CTE around the lap.

**Variations of Kp parameter**
<center><img src="./build/compare_p.png" alt="Kp parameter" style="width: 400px;"/></center>

The final set of parameters were adjusted by manual "twiddling" around the converged set of
parameters from an initial Ziegler-Nichols application.
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
