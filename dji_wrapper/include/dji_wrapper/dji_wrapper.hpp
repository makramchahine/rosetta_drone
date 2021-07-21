#pragma once

#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>

#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/SetGoHomeAltitude.h>
#include <dji_osdk_ros/GetGoHomeAltitude.h>
#include <dji_osdk_ros/SetCurrentAircraftLocAsHomePoint.h>
#include <dji_osdk_ros/SetAvoidEnable.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>
#include <dji_osdk_ros/EmergencyBrake.h>
#include <dji_osdk_ros/GetAvoidEnable.h>

#include<dji_osdk_ros/SetJoystickMode.h>
#include<dji_osdk_ros/JoystickAction.h>


#include <dji_osdk_ros/dji_vehicle_node.h>
#include <tf/tf.h>
#include <sensor_msgs/Joy.h>
#include <dji_telemetry.hpp>

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

#include <dji_osdk_ros/GimbalAction.h>

#include <dji_osdk_ros/CameraStartShootSinglePhoto.h>
#include <dji_osdk_ros/CameraStartShootIntervalPhoto.h>
#include <dji_osdk_ros/CameraStopShootPhoto.h>
#include <dji_osdk_ros/CameraRecordVideoAction.h>

#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

#include "dji_version.hpp"


using namespace dji_osdk_ros;

class DJIWrapper {
 public:
  DJIWrapper(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  DJIWrapper()
      : DJIWrapper(ros::NodeHandle(), ros::NodeHandle("~")) {}

  void starter(const std_msgs::String &msg);
  void takeoff(const std_msgs::String &msg);
  void go_to_pos(const geometry_msgs::Point::ConstPtr &msg); //idk exactly how i want to execute a go to position function yet with user input
  void land(const std_msgs::String &msg); 
  void off(const std_msgs::String &msg); 
  void camera_pos(const geometry_msgs::Quaternion::ConstPtr &msg); 
  void take_pic(const std_msgs::String &msg);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  FlightTaskControl control_task;

  ros::Timer measure_tracking_timer_;
  ros::ServiceClient task_control_client;
  ros::ServiceClient flight_control_client;

  ros::Subscriber start_sub_;
  ros::Subscriber takeoff_sub_;
  ros::Subscriber pos_sub_;
  ros::Subscriber land_sub_;
  ros::Subscriber off_sub_;
  ros::Subscriber camera_pos_sub_;
  ros::Subscriber take_pic_sub_;

  // bool executing_trajectory_;

  // // Performance metrics variables
  // double sum_position_error_squared_;
  // double max_position_error_;
  // double sum_thrust_direction_error_squared_;
  // double max_thrust_direction_error_;
  // char input; //will be relevant to go to position function

  //for gimbal control
  ros::ServiceClient gimbal_control_client;
  ros::ServiceClient camera_start_shoot_single_photo_client;
  ros::ServiceClient camera_stop_shoot_photo_client;
};