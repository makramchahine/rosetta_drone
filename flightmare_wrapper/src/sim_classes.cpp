//this is a tester file, ignore

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
#include "std_msgs/String.h"

#include <sstream>

class MasterPlan {
 public:
  MasterPlan(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  MasterPlan()
      : MasterPlan(ros::NodeHandle(), ros::NodeHandle("~")) {}

  void set_value(int x); //based off input from the publisher/user, will send to correct function within masterplan class
  void starter(const std_msgs::String &msg);
  void go_to_z_pos(const std_msgs::String &msg); //idk exactly how i want to execute a go to position function yet with user input
  void land(const std_msgs::String &msg); 
  void off(const std_msgs::String &msg);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher arm_pub_;

  ros::Subscriber autopilot_feedback_sub_;

  ros::Timer measure_tracking_timer_;

  autopilot_helper::AutoPilotHelper autopilot_helper_;

  ros::Subscriber start_sub_;
  ros::Subscriber z_pos_sub_;
  ros::Subscriber land_sub_;
  ros::Subscriber off_sub_;
  bool executing_trajectory_;

  // Performance metrics variables
  double sum_position_error_squared_;
  double max_position_error_;
  double sum_thrust_direction_error_squared_;
  double max_thrust_direction_error_;
  char input; //will be relevant to go to position function
};

MasterPlan::MasterPlan(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh),
    pnh_(pnh) {
  start_sub_ = nh_.subscribe("start", 1000, &MasterPlan::starter,this);
  z_pos_sub_ = nh_.subscribe("z_pos", 1000, &MasterPlan::go_to_z_pos,this);
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

void MasterPlan::go_to_z_pos (const std_msgs::String &msg){ //gotta figure out how to deliver an input for z oops
  const Eigen::Vector3d position_cmd = Eigen::Vector3d(0.0, 0.0, 1.0);
  const double heading_cmd = 0.0;
  autopilot_helper_.sendPoseCommand(position_cmd, heading_cmd);
}

void MasterPlan::land (const std_msgs::String &msg){ //land the drone
  autopilot_helper_.sendLand();
}

void MasterPlan::off(const std_msgs::String &msg){ //turn the motors off
  autopilot_helper_.sendOff();
}

// current topic names: start, ZPos, land, off (zpos is currently non fucntioning)
int main(int argc, char **argv) {
  ROS_INFO("In main");
  ros::init(argc, argv, "sim_classes");
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  MasterPlan mp(nh,pnh);

  ros::spin();
  return 0;


}