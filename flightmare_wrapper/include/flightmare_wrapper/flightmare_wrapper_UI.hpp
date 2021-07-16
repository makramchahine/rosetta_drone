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
#include "geometry_msgs/Quaternion.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include <math.h>

#include <sstream>

class UI {
 public:
  UI(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

  UI()
      : UI(ros::NodeHandle(), ros::NodeHandle("~")) {}

  void selection();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher arm_pub_;

  ros::Subscriber autopilot_feedback_sub_;

  ros::Timer measure_tracking_timer_;

  autopilot_helper::AutoPilotHelper autopilot_helper_;

  ros::Publisher start_pub_;
  ros::Publisher takeoff_pub_;
  ros::Publisher pos_pub_;
  ros::Publisher land_pub_;
  ros::Publisher off_pub_;
  ros::Publisher camera_pos_pub_;
  ros::Publisher take_pic_pub_;
  ros::Publisher stream_pic_pub_;

  bool executing_trajectory_;

  // Performance metrics variables
  double sum_position_error_squared_;
  double max_position_error_;
  double sum_thrust_direction_error_squared_;
  double max_thrust_direction_error_;
  char input; //will be relevant to go to position function
};