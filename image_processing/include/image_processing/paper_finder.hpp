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

/**
* Subscribes to a camera stream and tries to detect a white piece of paper in it and calculate its area
* It then publishes those areas. It can also display a version of the video with the detected paper outlined in red
*
* @see         PaperCommander
*/
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
  ros::Publisher area_percentage_pub_;

  // subscriber
  image_transport::ImageTransport it_;
  image_transport::Subscriber camera_sub_;

  // parameters
  bool debug{true}; //whether or not to show intermediate filters
  bool show_video{true}; //whether or not to show webcam video with highlighted paper
  bool flip_image{true};

  float threshold_value{150};

  const std::string OPENCV_WINDOW{"Paper Finder"};



};