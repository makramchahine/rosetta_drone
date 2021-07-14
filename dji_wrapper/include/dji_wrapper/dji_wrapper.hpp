#pragma once

#include <vector>

#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <std_msgs/Bool.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include <Eigen/Dense>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

#include <sstream>

#include <memory>

// ros
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>


#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace flightlib;

class DJI_class {
 public:
  DJI_class(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  DJI_class()
      : DJI_class(ros::NodeHandle(), ros::NodeHandle("~")) {}

  void set_value(int x); //based off input from the publisher/user, will send to correct function within masterplan class
  void starter(const std_msgs::String &msg);
  void go_to_pos(const geometry_msgs::Point::ConstPtr &msg); //idk exactly how i want to execute a go to position function yet with user input
  void land(const std_msgs::String &msg); 
  void off(const std_msgs::String &msg); 
  void camera_pos(const geometry_msgs::Quaternion &msg); 
  void take_pic(const std_msgs::String &msg);

  // callbacks
  void mainLoopCallback(const ros::TimerEvent& event);
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Timer measure_tracking_timer_;

  ros::Subscriber start_sub_;
  ros::Subscriber pos_sub_;
  ros::Subscriber land_sub_;
  ros::Subscriber off_sub_;
  ros::Subscriber camera_pos_sub_;
  ros::Subscriber take_pic_sub_;

  bool executing_trajectory_;

  // Performance metrics variables
  double sum_position_error_squared_;
  double max_position_error_;
  double sum_thrust_direction_error_squared_;
  double max_thrust_direction_error_;
  char input; //will be relevant to go to position function


};