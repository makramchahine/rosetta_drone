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

#include <sstream>

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
  bool executing_trajectory_;

  // Performance metrics variables
  double sum_position_error_squared_;
  double max_position_error_;
  double sum_thrust_direction_error_squared_;
  double max_thrust_direction_error_;
  char input; //will be relevant to go to position function
};