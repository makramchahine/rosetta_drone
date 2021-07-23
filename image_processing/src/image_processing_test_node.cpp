#pragma once

//#include "custom_control/flight_pilot_custom.hpp"
#include <ros/ros.h>
//#include <opencv2/opencv.hpp>
//#include "opencv2/imgproc.hpp"
//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/highgui.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
// #include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"
// #include "opencv2/dnn.hpp"
// #include "opencv2/flann.hpp"
// #include "opencv2/ml.hpp"
// #include "opencv2/objdetect.hpp"
// #include "opencv2/photo.hpp"
// #include "opencv2/stitching.hpp"
// #include "opencv2/video.hpp"
// #include "opencv2/videoio.hpp"
// #include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// #include <math.h>
#include "std_msgs/Float32.h"

using namespace cv;
using namespace std;

RNG rng(12345);


int main(int argc, char** argv) {

  ros::init(argc, argv, "paper_detector");

  ros::NodeHandle nh;

  ros::Publisher area_pub = nh.advertise<std_msgs::Float32>("area", 1000);

  VideoCapture cap;
  // open the default camera, use something different from 0 otherwise;
  // Check VideoCapture documentation.
  if(!cap.open(0))
      return 0;

  int numLastAreas = 15;
  float lastAreas[numLastAreas];
  int lastAreaIndex = 0;

  bool debug = false;


  while(ros::ok())
  {
        Mat frame;
        cap >> frame;

        Mat dst;                 // dst must be a different Mat
        flip(frame, dst, 1);     // because you can't flip in-place (leads to segfault)

        Mat greyMat;
        cvtColor(dst, greyMat, COLOR_BGR2GRAY);

        Mat blurred;
        blur( greyMat, blurred, Size( 9, 9 ), Point(-1,-1) );

        Mat normalized;

        normalize(blurred, normalized, 255, 0, NORM_MINMAX,-1, noArray());

        if(debug)
          imshow("normalized", normalized);

        Mat thresholded;

        threshold( normalized, thresholded, 150, 255, 3 );

        if(debug)
          imshow("thresholded", thresholded);

        int thresh = 100;

        float maxArea = 0;
        int maxAreaIndex = 0;

        Mat canny_output;
        Canny( thresholded, canny_output, thresh, thresh*2 );
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
        Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
        for( size_t i = 0; i< contours.size(); i++ )
        {
          auto cnt = contours[i];

          Point2f vtx[4];
          RotatedRect box = minAreaRect(cnt);
          box.points(vtx);

          float minDist = 10000000;
          float maxDist = 0;

          for( int j = 0; j < 4; j++ ) {
            float dist = norm(vtx[j] - vtx[(j+1)%4]);
            if(dist < minDist) minDist = dist;
            if(dist > maxDist) maxDist = dist;
            if(debug)
              line(drawing, vtx[j], vtx[(j+1)%4], Scalar(255), 1, LINE_AA);
          }

          

          float area = minDist*maxDist;//contourArea(approx);

          if(area > maxArea) {
            maxArea = area;
            maxAreaIndex = (int)i;
          }

          // Scalar color = Scalar( 255, 255, 255 );
          // drawContours( drawing, contours, int(i), color, 2, LINE_8, hierarchy, 0 );

          
        }
        Scalar color = Scalar( 255, 255, 255 );
        drawContours( drawing, contours, maxAreaIndex, color, 2, LINE_8, hierarchy, 0 );

        float realMaxArea = contourArea(contours[maxAreaIndex]);

        lastAreas[lastAreaIndex++ % numLastAreas] = realMaxArea;

        float avg = 0;
        float smallestArea = 10000000000000;

        for(int j=0;j<numLastAreas;j++) {
          avg += lastAreas[j];
          smallestArea = min(smallestArea, lastAreas[j]);
        }

        avg -= smallestArea;

        avg /= (numLastAreas-1);

        std_msgs::Float32 msg;

        msg.data = avg;

        area_pub.publish(msg);

        

        if(debug)
          imshow( "Contours", drawing );

        Scalar color2 = Scalar( 0, 0, 255 );
        drawContours( dst, contours, maxAreaIndex, color2, 2, LINE_8, hierarchy, 0 );

        if( frame.empty() ) break; // end of video stream
        imshow("Highlighted paper", dst);
        if( waitKey(10) == 27 ) break; // stop capturing by pressing ESC 
  }
  // the camera will be closed automatically upon exit
  // cap.close();

  

  std::string image_path_;

  ROS_INFO("1");

  nh.param<std::string>("image_path", image_path_, "/home/joseph/flightmare_ws/src/blobs.png");

  ROS_INFO("2");

  // Read image
  Mat im = imread( image_path_, IMREAD_GRAYSCALE );

  ROS_INFO("3");

  // Set up the detector with default parameters.
  //SimpleBlobDetector detector;
  // Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create();

  ROS_INFO("4");

  // Detect blobs.
  // std::vector<KeyPoint> keypoints;

  ROS_INFO("4.5");

  // detector->detect( im, keypoints);

  ROS_INFO("5");

  // Draw detected blobs as red circles.
  // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
  // Mat im_with_keypoints;
  // drawKeypoints( im, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

  ROS_INFO("6");

  // Show blobs
  // imshow("keypoints", im_with_keypoints );
  waitKey(0);


  return 0;
}