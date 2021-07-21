#include "image_processing/paper_commander.hpp"

PaperCommander::PaperCommander(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
	: nh_(nh),
  	pnh_(pnh),
    lastAreaIndex(0),
    goalAreaPercentage(.2),
    areaPercentageEpsilon(.05) {


  area_percentage_sub_ = nh_.subscribe("/area_percentage", 1000,
      &PaperCommander::processArea, this);


  area_percentage_pub_ = nh_.advertise<std_msgs::Float32>("area_smoothed", 1000);


  

}

PaperCommander::~PaperCommander() { }

void PaperCommander::processArea(const std_msgs::Float32::ConstPtr& msg)
{

  float areaPercentage = msg->data;

  pastAreas[lastAreaIndex++ % numLastAreas] = areaPercentage;

  float avg = 0;
  float smallestArea = 100000000000000000;

  for(int j=0;j<numLastAreas;j++) {
    avg += pastAreas[j];
    smallestArea = min(smallestArea, pastAreas[j]);
  }

  avg -= smallestArea;

  avg /= (numLastAreas-1);

  std_msgs::Float32 msg_out;

  msg_out.data = avg;

  area_percentage_pub_.publish(msg_out);

  if(abs(avg - goalAreaPercentage) < areaPercentageEpsilon) {
    ROS_INFO("%f SUCCESS!", avg);
  } else if(avg > goalAreaPercentage) {
    ROS_INFO("%f FARTHER!", avg);
  } else if(avg < goalAreaPercentage) {
    ROS_INFO("%f CLOSER!!", avg);
  } else {
    ROS_INFO("WHATTTTTTTT?");
  }

  // ROS_INFO(avg);


}

bool PaperCommander::loadParams(void) {
  // load parameters
  // quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  // quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  return true;
}