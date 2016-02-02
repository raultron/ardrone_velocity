# ardrone_velocity
## Introduction
Ardrone Velocity is a ROS Package for PID Velocity control of the AR.DRone 2.0. It is designed to work together with the ardrone_autonomy package.

Based on a velocity reference (given by the user) and a velocity measurement (obtained from ardrone_autonomy), this package implements a PID controller to control the velocity of the AR.Drone 2.0. Since this controller is dependant on Wifi communication with the AR.Drone it will have problems related to the delays in communication.

## Dependencies
This package requires an AR.Drone 2.0 and the ROS package ardrone_autonomy.

## Installation 
### Compile from source
This ROS package depends on these standard ROS packages: geometry_msgs, roscpp, rospy, nav_msgs and std_msgs.

Clone this repository into the src folder of an existing or new catkin compatible workspace, and then build it using catkin. For example, to compile in ROS Indigo:

```
cd ~/catkin_ws/src
git clone https://github.com/raultron/ardrone_velocity.git
cd ~/catkin_ws
rosdep install --from-paths src -i
catkin_make
```

## Usage
