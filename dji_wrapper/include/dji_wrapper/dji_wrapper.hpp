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


#include<dji_osdk_ros/SetJoystickMode.h>
#include<dji_osdk_ros/JoystickAction.h>

#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Header.h"

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

#include "dji_version.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf/transform_datatypes.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>


using namespace dji_osdk_ros;

class DJIWrapper {
 public:
  DJIWrapper(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  DJIWrapper()
      : DJIWrapper(ros::NodeHandle(), ros::NodeHandle("~")) {}

  void starter(const std_msgs::String &msg);
  void takeoff(const std_msgs::String &msg);
  void go_to_pos(const geometry_msgs::Point::ConstPtr &msg); //idk exactly how i want to execute a go to position function yet with user input
  void set_heading(const std_msgs::Float32 &msg);
  void land(const std_msgs::String &msg); 
  void off(const std_msgs::String &msg); 
  void camera_pos(const geometry_msgs::Quaternion::ConstPtr &msg); 
  void take_pic(const std_msgs::String &msg);
  void set_home(const std_msgs::String &msg);
  void follow_path(const nav_msgs::Path &msg);
  void follow_velocity_path(const nav_msgs::Path &msg);

  void localFrameRefSubCallback(const sensor_msgs::NavSatFix::ConstPtr& localFrameRef);
  void timeSyncNmeaSubSCallback(const nmea_msgs::Sentence::ConstPtr& timeSyncNmeaMsg);
  void timeSyncGpsUtcSubCallback(const dji_osdk_ros::GPSUTC::ConstPtr& timeSyncGpsUtc);
  void timeSyncFcUtcSubCallback(const dji_osdk_ros::FCTimeInUTC::ConstPtr& timeSyncFcUtc);
  void timeSyncPpsSourceSubCallback(const std_msgs::String::ConstPtr& timeSyncPpsSource);
  bool moveByPosOffset(const JoystickCommand &offsetDesired,
                     float posThresholdInM,
                     float yawThresholdInDeg);
  double quaternionToYaw(const geometry_msgs::Quaternion &msg);
  void velocityAndYawRateCtrl(const JoystickCommand &offsetDesired, uint32_t timeMs);


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
  ros::Subscriber heading_sub_;
  ros::Subscriber land_sub_;
  ros::Subscriber off_sub_;
  ros::Subscriber camera_pos_sub_;
  ros::Subscriber take_pic_sub_;
  ros::Subscriber set_home_sub_;
  ros::Subscriber path_sub_;



  ros::Subscriber localFrameRefSub;
  ros::Subscriber timeSyncNmeaSub;
  ros::Subscriber timeSyncGpsUtcSub;
  ros::Subscriber timeSyncFcUtcSub;
  ros::Subscriber timeSyncPpsSourceSub;

  sensor_msgs::NavSatFix local_Frame_ref_;
  nmea_msgs::Sentence time_sync_nmea_msg_;
  dji_osdk_ros::GPSUTC time_sync_gps_utc_;
  std_msgs::String time_sync_pps_source_;
  dji_osdk_ros::FCTimeInUTC time_sync_fc_utc_;

  //for gimbal control
  ros::ServiceClient gimbal_control_client;
  ros::ServiceClient camera_start_shoot_single_photo_client;
  ros::ServiceClient camera_stop_shoot_photo_client;
};