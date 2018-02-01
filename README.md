# Extended Kalman Filter

Author : Manoj Kumar Subramanian

------

## Overview

This repository is as part of my Submission to the Project 1: Extended Kalman Filter Project for the Udacity Self Driving Car Nano Degree Program Term 2.

In this project,  an extended kalman filter is realized in C++ to estimate the state of a moving object of interest with noisy lidar and radar measurements. This project involves the Term 2 Simulator.

------

## Project Goals

The goals of this project are the following:

- The code must compile without any errors with cmake and make
- The project requires obtaining Root Mean Squared Error (RMSE) values for the estimated state variables to be lower than that the tolerance outlined in the project rubric [.11, .11, 0.52, 0.52].

------

## Rubric Points

### Compiling without any errors

I have used Docker for Windows using the DockerToolbox setup and pulled the Udacity's Carnd control kit docker container which constituted the necessary tools required (including cmake, make, gcc & git) for the project.

**<u>Basic Build Instructions</u>**

1. Clone this repo.

2. Make a build directory: `mkdir build && cd build`

3. Compile: `cmake .. && make` 

   - On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`

4. Run it: `./ExtendedKF` 

   The program should wait listening to port 4567.

**<u>Running the simulator</u>**

Before running the simulator, configure the port forwarding to the port 4567 since the simulator and the c++ program talk using the uWebSocketIO in port 4567.


INPUT: values provided by the simulator to the ExtendedKF c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the ExtendedKF c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

In the simulator, choose the Project 1: Extended Kalman filter, select the respective dataset and press Start button. The simulator sends the Input to the ExtendedKF c++ program and gets the estimated and rmse values from the program that is visualized in the simulator.

---

### Accuracy of the Estimation

Below are the screenshots of the program executed for the Datasets 1 and 2.

It shall be noted that the RSME values are below the project requirements of [.11, .11, 0.52, 0.52].

**Dataset1:**

![Dataset1](Docs/Dataset1.png)



**Dataset2:**

![Dataset1](Docs/Dataset2.png)

------

### Correctness of Algorithm

**1. General Process flow:**

Since the startup code was already available from the Udacity's repository, the portions, where the TODO items were mentioned, were updated.

The functions for Predict, Update (KF) and UpdateEKF are implemented in the kalman_filter.cpp with the steps defined in the lectures. The tools.cpp file updated with the calculations for Jacobian and RSME.

In the initialization step of FusionEKF.cpp, the noise factors were added.

**2. First Measurements:**

The item for the first initialization measurements to update the necessary state variables and the co-variance matrix were added. As per the dataset available, the RADAR co-ordinates function to switch between polar to cartesian is added.

**3. Predict and Update:**

For the next measurements, updating the time interval, Transition matrix (F), Q matrix and calling the predict function were implemented. Since the F used for both LIDAR and RADAR kept same, predict function is used for both types. For the update, the methods Update and UpdateEKF created in kalman_filter.cpp were used.

**4. Separating LIDAR and RADAR:**

For the updating, since the LIDAR measurements were linear, the Kalman filters Update method was used to get the estimated state variables.

For the RADAR, the Kalman filters UpdateEKF method used to use the calculated Jacobian matrix instead of the H matrix. Also, the switching between cartesian and polar added for RADAR during the comparison of the outputs.

**General considerations:**

As mentioned in the project tips, code sections to avoid divide by zero and maintaining the calculated phi angle within the -pi to +pi range were added.





