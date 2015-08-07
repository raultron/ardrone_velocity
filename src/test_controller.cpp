#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/Navdata.h"

double height  = 2;
double velx    = 0;
double vely    = 0;
double angz    = 0;
bool buttonTakeoff   = false;
bool buttonLand   = false;
bool ardroneFlying  = false;
bool buttonEnable = false;
bool last_buttonEnable = false;
std_msgs::Empty empty;
ros::Subscriber cmd_vel_sub, navdata_sub;
ros::Publisher  twist_pub, takeoff_pub, land_pub;
ros::Timer timer1;
ros::Timer timer2;
ros::NodeHandle* n_p;
ardrone_autonomy::Navdata last_navdata;

enum class State{
    ground_init,
    takingoff,
    hovering_init,
    flying,
    hovering_end,
    landing,
    ground_end
};

enum class NavdataState : std::int32_t {
    unknown,
    init,
    landed,
    flying,
    hovering,
    test,
    takingoff,
    gotofixpoint,
    landing,
    looping
};

State quad_state = State::ground_init;
NavdataState navdata_state = NavdataState::unknown;


void set_hover(void){
    geometry_msgs::Twist cmd_vel_out;
    cmd_vel_out.linear.x = 0;
    cmd_vel_out.linear.y = 0;
    cmd_vel_out.linear.z = 0;
    cmd_vel_out.angular.x = 0;
    cmd_vel_out.angular.y = 0;
    cmd_vel_out.angular.z = 0;
    twist_pub.publish(cmd_vel_out);
}

void land_or_takeoff(void){
    if (ardroneFlying){
        ardroneFlying = false;
    }
    else{
        ardroneFlying = true;
    }
}

void takeoff(void){
    takeoff_pub.publish(empty);
}

void land(void){
    land_pub.publish(empty);
}

void joyCallback(const sensor_msgs::Joy& in)
{
    geometry_msgs::Twist out_twist;
    velx   = double(in.axes[1]);
    vely   = double(in.axes[0]);
    height = double(in.axes[2])*3;
    angz   = double(in.axes[3]);
    buttonTakeoff = bool(in.buttons[2]);
    buttonLand = bool(in.buttons[1]);
    buttonEnable = bool(in.buttons[0]);
    if (buttonTakeoff){
        takeoff();
    }
    if (buttonLand){
        land();
    }

    out_twist.linear.x = velx;
    out_twist.linear.y = vely;
    out_twist.linear.z = height;
    out_twist.angular.z = angz;

    if (buttonEnable){
        //To Disable auto hover
        out_twist.angular.x = 1.0;
        out_twist.angular.y = 1.0;
    }
    else{
        out_twist.angular.x = 0.0;
        out_twist.angular.y = 0.0;
    }

    twist_pub.publish(out_twist);
}



void stopFlying(const ros::TimerEvent&){
    ROS_INFO("Flying end, sending command to hover");
    quad_state = State::hovering_end;
    set_hover();
}

void startFlying(const ros::TimerEvent&){
    ROS_INFO("Flying Begin (10 seconds)");
    quad_state = State::flying;
    timer2 = n_p->createTimer(ros::Duration(10), stopFlying, true);}


void navdataCallback(const ardrone_autonomy::Navdata navdata){
    last_navdata = navdata;
}

int main(int argc,char* argv[])
{
    ros::init(argc, argv, "test_controller"); // Name of the node
    ros::NodeHandle n;

    n_p =&n;
    cmd_vel_sub = n.subscribe("/joy",1,joyCallback);
    navdata_sub = n.subscribe("/ardrone/navdata",1,navdataCallback);
    twist_pub   = n.advertise<geometry_msgs::Twist>("/cmd_vel_pid", 1);
    takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    land_pub    = n.advertise<std_msgs::Empty>("/ardrone/land", 1);

    ros::Rate rate(5); // 10 hz
    geometry_msgs::Twist out_twist;
    out_twist.linear.x = 0.0;
    out_twist.linear.y = 0.0;
    out_twist.linear.z = 0.0;
    out_twist.angular.z = 0.0;
    out_twist.angular.x = 0.0;
    out_twist.angular.y = 0.0;



    while(n.ok()){
        ros::spinOnce();
        if(quad_state == State::ground_init){
            ROS_INFO("Init Test.");
            ROS_INFO("Sending command to takeoff.");

            quad_state = State::takingoff;
        }
        else if(quad_state == State::takingoff){
            //checkHover();
            if (last_navdata.state == (int32_t)NavdataState::landed){
                ROS_INFO("FORCING command to takeoff.");
                ROS_INFO("last_navdata.state %d",last_navdata.state);
                takeoff();
            }
            else if(last_navdata.state == (int32_t)NavdataState::init){
                ROS_INFO("FORCING command to takeoff.");
                ROS_INFO("last_navdata.state %d",last_navdata.state);
                takeoff();
            }
            else if(last_navdata.state == (int32_t)NavdataState::hovering){
                quad_state = State::hovering_init;
                ROS_INFO("Take off complete, waitin 0.5 seconds.");
                timer1 = n.createTimer(ros::Duration(0.5), startFlying, true);
            }
        }
        else if(quad_state == State::hovering_init){

        }
        else if(quad_state == State::flying){
            //send_command();
            //
            //
            twist_pub.publish(out_twist);
            out_twist.linear.x = 0.2;
            out_twist.linear.y = 0.0;
            out_twist.linear.z = 0.0;
            out_twist.angular.z = 0.0;
            out_twist.angular.x = 1.0;
            out_twist.angular.y = 1.0;

        }
        else if(quad_state == State::hovering_end){
            ROS_INFO("last_navdata.state %d",last_navdata.state);
            if(last_navdata.state == (int32_t)NavdataState::hovering){

                ROS_INFO("Hovering, now sending command to land");
                land();
                quad_state = State::landing;
            }
        }
        else if(quad_state == State::landing){
            //Check for landing
            if (last_navdata.state == (int32_t)NavdataState::gotofixpoint){
                // OK do nothing
            }
            else if (last_navdata.state == (int32_t)NavdataState::landed){
                ROS_INFO("Quadcopter on the ground");
                quad_state = State::ground_end;

            }
            else {
                land();
                ROS_INFO("FORCING LAND");
            }
        }
        else if(quad_state == State::ground_end){
            //Check for landing
            ROS_INFO("Test Succesfull");
            ros::shutdown();
        }

        rate.sleep();
        }
}
