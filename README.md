# Introduction
this project is the model predictive controller project from the udacity SDC course. the goal of the project is to control the vehicle driving itself along the track in the simulator with a model predictive control method.

## Model
### State
the state variables contains px, py, psi, v and the errors cte and epsi.


- px: the current position in x-axis of map coordinate
- py: the current position in y-axis of map coordinate
- psi: the heading angle of the vehicle 
- v: the current velocity of the vehicle
- cte: the cross track error between the desired position and the actual position at px = 0
- epsi: the heading angle error between the desired heading angle with the actual heading angle at px = 0

### Actuator
there are two actuators which we can use to control the vehicle in the project.

- delta: this is the steering value which can be used to control the steering angle of the vehicle. the delta is restricted to [-25 degree, 25 degree], in order to make the control easily, the value has been mapped to [-1, 1].
- a: this is the longitudinal control command the vehicle, positive means accelerate and negative means decelerate.
### Update Equation
the kinematic model is used here to predict the future state t+1 based on the current state t.

		//x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt      
		// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt      
		// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt      
		// v_[t+1] = v[t] + a[t] * dt      
		// cte[t+1] = cte[t] + v[t] * sin(epsi[t]) * dt      
		// epsi[t+1] = epsi[t] + v[t] * delta[t] / Lf * dt


## Timestep Length and Duration (N & dt)
the timestep length N, we can say it is the steps which predicted for the future, if N is too small, then the predict horizon is too short, which makes the control erratic; although that N is bigger, then it can predict more, but it will also increase the calculation a bit. with the experiment, I set the N as 10.

for the dt, I set it to 0.1s, same as the latency, which makes it easy during handling the latency.


## Polynomial fit and MPC pre-processing


		  for (int i =0; i < ptsx.size(); i++ )
		  {
			double dx = ptsx[i] - px;            
		    double dy = ptsy[i] - py;

			waypoints_x[i] = dx * cos(-psi) - dy * sin(-psi);
			waypoints_y[i] = dx * sin(-psi) + dy * cos(-psi);
		  }

after the waypoints transfer from global map coordinate to ego vehicle coordinate, I use a 3 order polynomial to fit points.

## Model Predictive control with latency
the 100ms latency time has been considered during the model predictive control. it has been considered during the constraint setup. the steering command and the longitudinal command a0 has been delayed with one cycle if the predictive step t > 1, since the dt is defined as 0.1s, which time for one cycle is equal to the latency time.

		AD<double> a0 = vars[a_start + t - 1];  
		if (t > 1) {   // consider for actuator latency
        	a0 = vars[a_start + t - 2];
        	delta0 = vars[delta_start + t - 2];
        }


additionally for the cost function calculation, besides all the cost which has been used in the quiz of the lesson, I additionally add one combination of the speed and steering value, to make sure that speed and steering value will not been very at the same time, since it will lead erratic control if both of the values are too big.
below is how the cost function has been calculated:

	for (unsigned int t = 0; t < N; t++) {      
		fg[0] += 200*CppAD::pow(vars[cte_start + t], 2);      
		fg[0] += 200*CppAD::pow(vars[epsi_start + t], 2);      
		fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);    
		}   
	
	// Minimize the use of actuators.    
	for (unsigned int t = 0; t < N - 1; t++) {      
		fg[0] += 100*CppAD::pow(vars[delta_start + t], 2);      
		fg[0] += 5*CppAD::pow(vars[a_start + t], 2);   

		// put the multiplied value of speed and steering into cost value consideration
		fg[0] += 200*CppAD::pow(vars[delta_start + t]*vars[v_start + t], 2);
		}    
	// Minimize the value gap between sequential actuations.    
	for (unsigned int t = 0; t < N - 2; t++) {      
		fg[0] += 200*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);      
		fg[0] += 10*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);    
		}

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
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
# MPC
