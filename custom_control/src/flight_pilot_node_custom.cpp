#include <ros/ros.h>

#include "custom_control/flight_pilot_custom.hpp"

int main(int argc, char** argv) {
  ROS_DEBUG("MAINEEE");
  ROS_INFO("MAINEINFO");
  ros::init(argc, argv, "flight_pilot");
  ROS_INFO("cool innit");
  flightros::FlightPilotCustom pilot(ros::NodeHandle(), ros::NodeHandle("~"));

  ROS_INFO("MADE THE THING");

  // spin the ros
  ros::spin();

  return 0;
}