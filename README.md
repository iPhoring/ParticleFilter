# Particle Filter for vehicle localization
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

# Objective: Localization 
Our self driving car moved to an alien world with limited and noisy data. Car will not be able to use any of its past beleives thus itneeds to reestablize all beelives using a particle filter. The alien world has a map, a noisy GPS with erronous sensor and control data. This project will implement a two dimensional particle filter in C++ to localize the car for autonomous driving. The particle filter will have access to the map, noisy initial localization information from GPS and noisy observation and control data.

## Running the Code
1. Clone the github repo
1. Run ./clean.sh to remove old object and linker files
2. Run ./build.sh to make a fresh build
3. Run ./run.sh to execute the particle filter 



INPUT: values provided by the simulator to the c++ program
// sense noisy position data from the simulator
["sense_x"]
["sense_y"]
["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state
["previous_velocity"]
["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values
["sense_observations_x"]
["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator
// best particle values used for calculating the error evaluation
["best_particle_x"]
["best_particle_y"]
["best_particle_theta"]


# Implementing the Particle Filter
The directory structure of this repository is as follows:
```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.
> * Map data provided by 3D Mapping Solutions GmbH.
