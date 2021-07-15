#include "flightmare_wrapper/flightmare_wrapper.hpp"

int main(int argc, char **argv) {
  ROS_INFO("In main");
  ros::init(argc, argv, "sim_classes");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  FlightmareWrapper mp(nh,pnh);

  ros::spin();
  return 0;


}