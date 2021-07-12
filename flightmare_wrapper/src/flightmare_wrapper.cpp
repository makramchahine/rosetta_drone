#include "flightmare_wrapper/flightmare_wrapper.hpp"

MasterPlan::MasterPlan(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh),
    pnh_(pnh) {
  start_sub_ = nh_.subscribe("start", 1000, &MasterPlan::starter,this);
  pos_sub_ = nh_.subscribe("pos", 1000, &MasterPlan::go_to_pos,this);
  land_sub_ = nh_.subscribe("land", 1000, &MasterPlan::land,this);
  off_sub_ = nh_.subscribe("off", 1000, &MasterPlan::off,this);

  arm_pub_ = nh_.advertise<std_msgs::Bool>("bridge/arm", 1);
}


void MasterPlan::set_value(int x){
  input=x;
}

void MasterPlan::starter (const std_msgs::String &msg){ //this will start the drone and then do arm bridge
  ros::WallDuration sleep_t(10);
  sleep_t.sleep();

  autopilot_helper_.sendStart();
  ROS_INFO("Start");
  sleep_t.sleep();
  
  std_msgs::Bool arm_msg;
  arm_msg.data = true;
  arm_pub_.publish(arm_msg);
  sleep_t.sleep();
}

void MasterPlan::go_to_pos (const geometry_msgs::Point::ConstPtr &msg){ //gotta figure out how to deliver an input for z oops
  const Eigen::Vector3d position_cmd = Eigen::Vector3d(msg->x, msg->y, msg->z);
  const double heading_cmd = 0.0;
  autopilot_helper_.sendPoseCommand(position_cmd, heading_cmd);
}

void MasterPlan::land (const std_msgs::String &msg){ //land the drone
  autopilot_helper_.sendLand();
}

void MasterPlan::off(const std_msgs::String &msg){ //turn the motors off
  autopilot_helper_.sendOff();
}