#include "UI/flightmare_wrapper_UI.hpp"

int main(int argc, char **argv){
  ros::init(argc, argv, "UI_pub");
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  UI use(nh,pnh);

  ros::spin();
  return 0;
}
