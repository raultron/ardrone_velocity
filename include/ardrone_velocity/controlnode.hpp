#ifndef CONTROLNODE_HPP
#define CONTROLNODE_HPP

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "ardrone_velocity/filtervelocity.hpp"

#include <dynamic_reconfigure/server.h>
#include <ardrone_velocity/dynamic_param_configConfig.h>

class ControlNode {
 public:
  ControlNode();

  // ROS message callbacks
  void cmd_velCallback(const geometry_msgs::Twist& cmd_vel_in);
  void m_quad_velCallback(const nav_msgs::Odometry& odo_msg);
  void dynamic_reconfigure_callback(
      ardrone_velocity::dynamic_param_configConfig& config, uint32_t level);

  ros::NodeHandle nh;

  // PID Controller
  void velocity_control(void);

  // Taken from tum_ardrone
  void i_term_increase(double& i_term, double new_err, double cap);

  void set_hover(void);

  bool m_cmd_valid = false;

  // Filter for quadcopter velocities
  FilterVelocity m_filter_vel_x;
  FilterVelocity m_filter_vel_y;
  // double filter_velocity(double new_value_x,double new_value_y);

 private:
  ros::Subscriber m_cmd_vel_sub;
  ros::Subscriber m_quad_vel_sub;
  ros::Publisher m_cmd_vel_pub;
  ros::Publisher m_debug_pub;  //! For debugging variables in rqt_plot
  // dynamic reconfigure server
  dynamic_reconfigure::Server<ardrone_velocity::dynamic_param_configConfig>
      m_server;
  geometry_msgs::Twist m_current_command;
  nav_msgs::Odometry m_odo_msg;
  ros::Time t;
  ros::Time old_t;
  double m_filtered_vel_x;
  double m_filtered_vel_y;
  double m_last_error_x;
  double m_last_error_y;
  double m_last_vel_x;
  double m_last_vel_y;
  double m_i_term_x;
  double m_i_term_y;
  // PID Coefficients
  double m_Kp_x, m_Ki_x, m_Kd_x;
  double m_Kp_y, m_Ki_y, m_Kd_y;
};

#endif  // CONTROLNODE_HPP
