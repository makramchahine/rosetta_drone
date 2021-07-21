#include "image_processing/paper_commander.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "paper_commander");

  PaperCommander paper_finder(ros::NodeHandle(), ros::NodeHandle("~"));

  ros::spin();

  return 0;
}