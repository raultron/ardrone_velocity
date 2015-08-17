#include "ardrone_velocity/controlnode.hpp"
#include <random>

ControlNode::ControlNode()
{
    m_cmd_vel_sub = nh.subscribe("/cmd_vel_pid",1,&ControlNode::cmd_velCallback,this);
    m_quad_vel_sub = nh.subscribe("/ardrone/odometry",1,&ControlNode::m_quad_velCallback,this,ros::TransportHints().tcpNoDelay());
    m_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    m_debug_pub = nh.advertise<std_msgs::Float64>("/pid/debug",1);
    m_i_term_x = 0.0;
    m_i_term_y = 0.0;
}

void ControlNode::cmd_velCallback(const geometry_msgs::Twist& cmd_vel_in){ 
    m_current_command = cmd_vel_in; // Units in m/s
    m_cmd_valid = true;
}

void ControlNode::m_quad_velCallback(const nav_msgs::Odometry& odo_msg){
    std_msgs::Float64 debug_msg;
    m_odo_msg = odo_msg;
    m_filtered_vel_x = m_filter_vel_x.filter(m_odo_msg.twist.twist.linear.x);
    m_filtered_vel_y = m_filter_vel_y.filter(m_odo_msg.twist.twist.linear.y);
    debug_msg.data = m_filtered_vel_x;
    m_debug_pub.publish(debug_msg);

    velocity_control();
}

void ControlNode::velocity_control(void){
    double p_term_x,d_term_x;
    double p_term_y,d_term_y;

    double Kp_x,Ki_x,Kd_x;
    double Kp_y,Ki_y,Kd_y;

    double error_x, acc_error_x;
    double error_y, acc_error_y;

    geometry_msgs::Twist cmd_vel_out;

    //We limit the maximum reference speed of the quadcopter
    //! TODO: Change this into ROS parameters
    double max_vel = 0.6;
    m_current_command.linear.x = std::min(max_vel,m_current_command.linear.x);
    m_current_command.linear.y = std::min(max_vel,m_current_command.linear.y);

    m_current_command.linear.x = std::max(-max_vel,m_current_command.linear.x);
    m_current_command.linear.y = std::max(-max_vel,m_current_command.linear.y);

    // We are only going to change linear.x and linear.y of this command
    // The rest of the values are the same
    cmd_vel_out = m_current_command;

    // In case that we receive a special command to hover
    if (cmd_vel_out.angular.x == 0 && cmd_vel_out.angular.y ==0 && cmd_vel_out.linear.x == 0 && cmd_vel_out.linear.y == 0 && cmd_vel_out.linear.z ==0){
        set_hover();
        //reset iterm
        m_i_term_x = 0.0;
        m_i_term_y = 0.0;
        return;
    }

    // otherwise we dont want to hover.
    cmd_vel_out.angular.x = 1;
    cmd_vel_out.angular.y = 1;

    //Control starts here
    //!TODO: Separate filter for input of the derivative term.
    //!TODO: Consider measurement and controled variables delays.

    //Coefficients for the PID controller
    // modify this values for proper tuning in your case
    Kp_x = 0.6;
    Kp_y = 0.6;

    Kd_x = 0.05;
    Kd_y = 0.05;

    Ki_x = 0.1;
    Ki_y = 0.1;

    //We calculate the velocity error
    error_x = m_current_command.linear.x - m_odo_msg.twist.twist.linear.x;
    error_y = m_current_command.linear.y - m_odo_msg.twist.twist.linear.y;

    //The proportional term is directly the error
    p_term_x = error_x;
    p_term_y = error_y;
    //p_term_x = 0;
    //p_term_y = 0;

    //For derivative and integral we need the current time and timestep (dt)
    t = ros::Time::now();
    ros::Duration dt = t - old_t;
    old_t = t;

    // Derivative term (based on veloctiy change instead of error change)
    // Note that we put the negative part here
    //d_term_x = -(m_odo_msg.twist.twist.linear.x - m_last_vel_x)/dt.toSec();
    //d_term_y = -(m_odo_msg.twist.twist.linear.y - m_last_vel_y)/dt.toSec();
    d_term_x = -(m_filtered_vel_x - m_last_vel_x)/0.004;
    d_term_y = -(m_filtered_vel_y - m_last_vel_y)/0.004;

    std_msgs::Float64 debug_msg;
    debug_msg.data = d_term_x*Kp_x*Kd_x;
    //m_debug_pub.publish(debug_msg);

    m_last_vel_x = m_filtered_vel_x;
    m_last_vel_y = m_filtered_vel_y;

    //! You can use this version as well but it will have some discontinuities
    //! when the reference changes
    //d_term_x = (error_x - m_last_error_x)/dt.toSec();
    //d_term_y = (error_y - m_last_error_y)/dt.toSec();
    //m_last_error_x = error_x;
    //m_last_error_y = error_y;


    //! Taken from tum_autonomy package.
    //! This calculates and limits the integral term
    // m_i_term is a member of the class
    i_term_increase(m_i_term_x,error_x*dt.toSec(), 1.2);
    i_term_increase(m_i_term_y,error_y*dt.toSec(), 1.2);

    //m_i_term_x = m_i_term_x + error_x*dt.toSec();
    //m_i_term_y = m_i_term_y + error_y;//*dt.toSec();

    // Control command (PID)
    cmd_vel_out.linear.x = Kp_x*(p_term_x + Ki_x*m_i_term_x + Kd_x*d_term_x);
    cmd_vel_out.linear.y = Kp_y*(p_term_y + Ki_y*m_i_term_y  + Kd_x*d_term_y);

    //Limit control command
    cmd_vel_out.linear.x = std::min(cmd_vel_out.linear.x,1.0);
    cmd_vel_out.linear.y = std::min(cmd_vel_out.linear.y,1.0);

    cmd_vel_out.linear.x = std::max(cmd_vel_out.linear.x,-1.0);
    cmd_vel_out.linear.y = std::max(cmd_vel_out.linear.y,-1.0);

    //Debugging information
    ROS_INFO("d_Time  : %f", dt.toSec());
    ROS_INFO("VelRef: %f", m_current_command.linear.x);
    ROS_INFO("Vel   : %f", m_odo_msg.twist.twist.linear.x);
    ROS_INFO("Error : %f", error_x);
    ROS_INFO("Cmd   : %f", cmd_vel_out.linear.x);
    ROS_INFO("pterm | iterm | dterm   : %f | %f | %f", Kp_x*p_term_x, Kp_x*Ki_x*m_i_term_x, Kp_x*Kd_x*d_term_x);
    ROS_INFO("------------------------------------------------------");

    //We publish the command
    m_cmd_vel_pub.publish(cmd_vel_out);
}


void ControlNode::set_hover(void){
    geometry_msgs::Twist cmd_vel_out;
    cmd_vel_out.linear.x = 0;
    cmd_vel_out.linear.y = 0;
    cmd_vel_out.linear.z = 0;
    cmd_vel_out.angular.x = 0;
    cmd_vel_out.angular.y = 0;
    cmd_vel_out.angular.z = 0;

    m_cmd_vel_pub.publish(cmd_vel_out);

}

void ControlNode::i_term_increase(double& i_term, double new_err, double cap)
{
    if(new_err < 0 && i_term > 0)
        i_term = std::max(0.0, i_term + 2.5 * new_err);
    else if(new_err > 0 && i_term < 0)
        i_term = std::min(0.0, i_term + 2.5 * new_err);
    else
        i_term += new_err;

    if(i_term > cap) i_term =  cap;
    if(i_term < -cap) i_term =  -cap;
}


