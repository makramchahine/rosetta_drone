#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Float32.h"

using namespace cv;
using namespace std;


class PaperFinder {
 public:
  PaperFinder(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~PaperFinder();

  void findArea(const sensor_msgs::ImageConstPtr& msg);

  bool loadParams(void);

 private:
  // ros nodes
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // publisher
  ros::Publisher area_pub_;

  // subscriber
  image_transport::ImageTransport it_;
  image_transport::Subscriber camera_sub_;

  // smoothing variables
  vector<float> pastAreas;

  // parameters
  bool debug{false}; //whether or not to show intermediate filters
  bool show_video{true}; //whether or not to show webcam video with highlighted paper
  bool flip_image{false};

  float threshold_value{150};


  /*
  bool enable_smoothing;

  // how many past datapoints to average together for final area publishing
  int averagingFilterSize;
  // removes this many of the smallest areas from the averaging filter - helps with smoothing by removing outliers
  int numSmallestDelete;
  */




  const std::string OPENCV_WINDOW{"Paper Finder"};



};