#pragma once

#include <ros/ros.h>
#include "std_msgs/Float32.h"

using namespace std;


class PaperCommander {
 public:
  PaperCommander(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~PaperCommander();

  void processArea(const std_msgs::Float32::ConstPtr& msg);

  bool loadParams(void);

 private:
  // ros nodes
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // publisher
  ros::Publisher area_percentage_pub_;

  // subscriber
  ros::Subscriber area_percentage_sub_;

  const static int numLastAreas{15};

  // smoothing variables
  float pastAreas[numLastAreas];

  int lastAreaIndex;

  float goalAreaPercentage;
  float areaPercentageEpsilon;



  
  bool enable_smoothing;

  // how many past datapoints to average together for final area publishing
  int averagingFilterSize;
  // removes this many of the smallest areas from the averaging filter - helps with smoothing by removing outliers
  int numSmallestDelete;
  




  const std::string OPENCV_WINDOW{"Paper Finder"};



};