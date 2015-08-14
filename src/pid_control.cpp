#include "ardrone_velocity/controlnode.hpp"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"


int main(int argc,char* argv[])
{
    ros::init(argc, argv, "pid_control"); // Name of the node
    ControlNode Node;

    //int32_t looprate = 1000; //hz
    //ros::Rate loop_rate(looprate);

    //ros::spin();
    while(Node.nh.ok()){
        ros::spinOnce();
        //loop_rate.sleep();
        }
}
