# ardrone_velocity
## Introduction
Ardrone Velocity is a ROS Package for PID Velocity control of the AR.DRone 2.0. It is designed to work together with the [ardrone_autonomy](https://github.com/AutonomyLab/ardrone_autonomy) package.

Based on a velocity reference (given by the user) and a velocity measurement (obtained from ardrone_autonomy), this package implements a PID velocity controller of the Quadcopter. Since this controller depends on Wifi communication with the AR.Drone to obtain the velocity measurement, it won't be perfect due to delays in communication.

## Dependencies
Requires an AR.Drone 2.0 Quadcopter and the ROS package ardrone_autonomy.

ROS Dependencies:
* ardrone_autonomy
* dynamic_reconfigure
* geometry_msgs
* roscpp
* rospy
* nav_msgs
* std_msgs.

## Installation
### Compile from source

- Install the ardrone_autonomy package following the instructions of their [repository](https://github.com/AutonomyLab/ardrone_autonomy).

- Clone ardrone_velocity repository into the src folder of an existing or new catkin compatible workspace, and then build using catkin. To compile in ROS Indigo:

```
cd ~/catkin_ws/src
git clone https://github.com/raultron/ardrone_velocity.git
cd ~/catkin_ws
rosdep install --from-paths src -i
catkin_make
```

## Usage

Launch the ardrone_autonomy driver (follow the instructions from the [ardrone_autonomy](https://github.com/AutonomyLab/ardrone_autonomy) documentation. This command works well with our package:

```
rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=True _navdata_demo:=0 _looprate:=400
```

Then run the ardrone_velocity main PID controller node:

```
rosrun ardone_velocity pid_control
```

## Units
All units and frames are [ROS REP 103]{http://www.ros.org/reps/rep-0103.html} compatible. Input velocity reference for the controller must be in m/s.

## Parameters
* ``cmd_vel_ref_topic`` : The controller subscribes to this topic, expects a velocity reference published as a "Twist message" - Default: /cmd_vel_ref
* ``odometry_topic`` : The controller subscribes to this topic, expects a "Odometry Message" with the odometry information of the platform - Default: /ardrone/odometry
* ``cmd_vel_out_topic`` : The controller publish in this topic a "Twist message" with the controlled velocity to be sent to the platform - Default: /cmd_vel

## Dynamic reconfiguration of PID Parameters
It is possible to reconfigure dynamically the following parameters:

* ``Kp_x``: Proportional Coefficient in X direction - Default = 0.6
* ``Kp_y``: Proportional Coefficient in Y direction - Default = 0.6
* ``Ki_x``: Integral Coefficient in X direction - Default = 0.05
* ``Ki_y``: Integral Coefficient in Y direction - Default = 0.05
* ``Kd_x``: Derivative Coefficient in X direction - Default = 0.1
* ``Kd_y``: Derivative Coefficient in Y direction - Default = 0.1

Use the following command for convenient reconfiguration using a graphical user interface:
```
rosrun rqt_reconfigure rqt_reconfigure
```
