# Kidnapped Vehicle

## Project Introduction

Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project, a 2 dimensional particle filter in C++ is implemented for localization of the robot.

## Running the Code

Term 2 Simulator can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

## Code Outline

* `main.cpp`: contains communication to the simulator, setting up the particle filter, and calling predict/update methods.
* `particle.cpp`: defines class ´Particle´ which has defines a Particle, initializes and moves it, and updates observations of the particle based on the map, and finally calculates the `weight` for that particle.
* `particle_filter.cpp`: generates and initializes the particles, call the predict (::move) and update (::update_observations) methods, and finally performs resmapling with replacement for the particles.

## Inputs
Map is read from `data/map_data.txt` and the sensed data (x, y, theta), velocity, and yaw rate are provided by the simulator.


## Output: 

Values provided by the c++ program to the simulator include the best particle values used for calculating the error evaluation.

