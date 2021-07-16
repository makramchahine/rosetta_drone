#include "image_processing/paper_finder.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "paper_finder");

  PaperFinder paper_finder(ros::NodeHandle(), ros::NodeHandle("~"));

  ros::spin();

  return 0;
}