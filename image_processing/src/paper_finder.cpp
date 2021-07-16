#include "image_processing/paper_finder.hpp"

PaperFinder::PaperFinder(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
	: nh_(nh),
  	pnh_(pnh),
  	it_(nh) {


  camera_sub_ = it_.subscribe("/camera/image_raw", 10,
      &PaperFinder::findArea, this);

  area_pub_ = nh_.advertise<std_msgs::Float32>("area", 1000);

  
  if(show_video)
  	cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
  

}

PaperFinder::~PaperFinder()
{
	
	if(show_video)
  	cv::destroyWindow(OPENCV_WINDOW);
  
}

void PaperFinder::findArea(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  Mat flipped;
  flip(cv_ptr->image, flipped, 1);

  Mat greyMat;
  if(flip_image) {
  	cvtColor(flipped, greyMat, COLOR_BGR2GRAY);
  } else {
  	cvtColor(cv_ptr->image, greyMat, COLOR_BGR2GRAY);
  }
  

  Mat blurred;
  blur( greyMat, blurred, Size( 9, 9 ), Point(-1,-1) );

  Mat normalized;
  normalize(blurred, normalized, 255, 0, NORM_MINMAX,-1, noArray());

  if(debug)
    imshow("normalized", normalized);

  Mat thresholded;

  threshold( normalized, thresholded, threshold_value, 255, 3 );

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

  std_msgs::Float32 areaAsg;

  areaAsg.data = realMaxArea;

  area_pub_.publish(areaAsg); 

  if(debug)
    imshow( "Contours", drawing );

  Scalar color2 = Scalar( 0, 0, 255 );

  if(flip_image) {
  	drawContours( flipped, contours, maxAreaIndex, color2, 2, LINE_8, hierarchy, 0 );
  } else {
  	drawContours( cv_ptr->image, contours, maxAreaIndex, color2, 2, LINE_8, hierarchy, 0 );
  }

  if(cv_ptr->image.empty())
  	ROS_INFO("FAILED");

  // imwrite("/home/joseph/flightmare_ws/frame.jpg", cv_ptr->image);

  if(show_video) {
  	
  	if(flip_image) {
  		ROS_INFO("showing frame flipped");
	  	imshow(OPENCV_WINDOW, flipped);
	  } else {
	  	ROS_INFO("showing frame normal");
	  	imshow(OPENCV_WINDOW, cv_ptr->image);
  	}
  	cv::waitKey(3);
  }

  ROS_INFO("left");




}

bool PaperFinder::loadParams(void) {
  // load parameters
  // quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  // quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  return true;
}