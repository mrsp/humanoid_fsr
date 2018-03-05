# README
A ROS package that compute force/torque states and the Center of Pressure on each leg based on FSR.

## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.


## Prerequisites
* Ubuntu 14.04 and later
* ROS indigo and later
* Eigen 3.2.0 and later


## Installing
* git clone https://github.com/mrsp/humanoid_fsr.git
* catkin_make


## Run
* Adjust the corresponding topics in config/estimation_params.yaml
* roslaunch humanoid_fsr humanoid_fsr.launch
