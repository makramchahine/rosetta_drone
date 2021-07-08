#include "mid_to_sim_include.h"

#include <vector>

#include <autopilot/autopilot_states.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <std_msgs/Bool.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include <Eigen/Dense>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("In chattercallback");
  ros::NodeHandle nh;
  ros::Publisher arm_pub_ = nh.advertise<std_msgs::Bool>("bridge/arm", 1);
  autopilot_helper::AutoPilotHelper autopilot_helper_;
  ROS_INFO("I heard: [%s]", msg->data.c_str());

  ros::WallDuration sleep_t(10);
  sleep_t.sleep();

  autopilot_helper_.sendStart();
  ROS_INFO("Start");
  sleep_t.sleep();
  
  std_msgs::Bool arm_msg;
  arm_msg.data = true;
  arm_pub_.publish(arm_msg);
  sleep_t.sleep();

  const Eigen::Vector3d position_cmd = Eigen::Vector3d(0.0, 0.0, 1.0);
  const double heading_cmd = 0.0;
  autopilot_helper_.sendPoseCommand(position_cmd, heading_cmd);

  ROS_INFO("Hover");
  sleep_t.sleep();

  autopilot_helper_.sendLand();
  ROS_INFO("Land");
  sleep_t.sleep();

  autopilot_helper_.sendOff();
  ROS_INFO("Off");

}

int main(int argc, char **argv) {
  ROS_INFO("In main");
  ros::init(argc, argv, "sim_direct");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("moveit", 1000, chatterCallback);
  ros::spin();
  return 0;


}
