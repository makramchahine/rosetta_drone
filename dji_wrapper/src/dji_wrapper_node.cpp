#include "dji_wrapper/dji_wrapper.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "dji_wrapper");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  DJIWrapper dji(nh,pnh);

  ros::spin();
  return 0;


}