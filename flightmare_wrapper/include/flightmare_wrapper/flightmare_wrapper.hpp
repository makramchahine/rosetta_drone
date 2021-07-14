#pragma once

#include <vector>

#include <autopilot/autopilot_states.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <std_msgs/Bool.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include <Eigen/Dense>

#include <autopilot/autopilot_helper.h>
#include <quadrotor_msgs/AutopilotFeedback.h>

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

// rpg quadrotor
#include <quadrotor_common/parameter_helper.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/bridges/unity_message_types.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace flightlib;

class MasterPlan {
 public:
  MasterPlan(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  MasterPlan()
      : MasterPlan(ros::NodeHandle(), ros::NodeHandle("~")) {}

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

  bool setUnity(const bool render);
  bool connectUnity(void);
  bool loadParams(void);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher arm_pub_;

  ros::Subscriber autopilot_feedback_sub_;

  ros::Timer measure_tracking_timer_;

  autopilot_helper::AutoPilotHelper autopilot_helper_;

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

  // publisher
  // camera publishers
  image_transport::Publisher rgb_pub;
  image_transport::Publisher depth_pub;
  image_transport::Publisher segmentation_pub;
  image_transport::Publisher opticalflow_pub;

  // subscriber
  ros::Subscriber sub_state_est_;

  // main loop timer
  ros::Timer timer_main_loop_;

  // unity quadrotor
  std::shared_ptr<Quadrotor> quad_ptr_;
  std::shared_ptr<RGBCamera> rgb_camera_;
  QuadState quad_state_;

  std::shared_ptr<RGBCamera> rgb_camera2;

  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr_;
  SceneID scene_id_{UnityScene::WAREHOUSE};
  bool unity_ready_{false};
  bool unity_render_{false};
  RenderMessage_t unity_output_;
  uint16_t receive_id_{0};

  // auxiliary variables
  Scalar main_loop_freq_{50.0};
  FrameID frame_id{0};
};