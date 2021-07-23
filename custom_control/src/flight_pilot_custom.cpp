#include "custom_control/flight_pilot_custom.hpp"

namespace flightros {

FlightPilotCustom::FlightPilotCustom(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::INDUSTRIAL_CUSTOM),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(50.0) {
  ROS_DEBUG("CONSTRUCTING");
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  gimbal_position = Vector<3>(0,0,-1);
  gimbal_orientation = Quaternion(1,0,0,0);

  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();

  // gimbal initialization
  gimbal_ptr_ = std::make_shared<Quadrotor>();

  const Vector<3> gimbal_size(0.000001,0.000001,0.000001);
  gimbal_ptr_->setSize(gimbal_size);

  // add mono camera
  /*
  rgb_camera_ = std::make_shared<RGBCamera>();
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(720);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  quad_ptr_->addRGBCamera(rgb_camera_);
  */

  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);

  gimbal_state_.setZero();
  gimbal_ptr_->reset(gimbal_state_);

  


  // initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("flight_pilot/state_estimate", 1,
                                 &FlightPilotCustom::poseCallback, this);

  gimbal_orientation_sub_ = nh_.subscribe("/hummingbird/camera_pos", 1,
                                 &FlightPilotCustom::gimbalOrientationCallback, this);

  timer_main_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_),
                                     &FlightPilotCustom::mainLoopCallback, this);
  timer_main_loop_.start();

  //ADDING CAMERA TO DRONE



  //camera on drone
  rgb_camera2 = std::make_shared<RGBCamera>();

  // initialize publishers
  image_transport::ImageTransport it(pnh);
  rgb_pub = it.advertise("/rgb", 1);
  depth_pub = it.advertise("/depth", 1);
  segmentation_pub = it.advertise("/segmentation", 1);
  opticalflow_pub = it.advertise("/opticalflow", 1);

  // attach camera to drone
  Vector<3> B_r_BC2(0.0, 0.0, 0);
  Matrix<3, 3> R_BC2 = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC2 << std::endl;
  rgb_camera2->setFOV(90);
  rgb_camera2->setWidth(640);
  rgb_camera2->setHeight(360);
  rgb_camera2->setRelPose(B_r_BC2, R_BC2);
  rgb_camera2->setPostProcesscing(
    std::vector<bool>{true, true, true});  // depth, segmentation, optical flow
  gimbal_ptr_->addRGBCamera(rgb_camera2);



  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();

  // connect unity
  setUnity(unity_render_);
  connectUnity();
}

FlightPilotCustom::~FlightPilotCustom() {}

void FlightPilotCustom::gimbalOrientationCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
  gimbal_orientation = Quaternion(msg->w, msg->x, msg->y, msg->z);
}

void FlightPilotCustom::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;
  //
  quad_ptr_->setState(quad_state_);
  
  Vector<3> quad_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  Matrix<3, 3> quad_rot = Quaternion(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();
  Vector<3> gimbal_position_final = quad_pos + quad_rot * (gimbal_position);
  Quaternion final_orientation(gimbal_orientation.toRotationMatrix()*quad_rot);

  gimbal_state_.x[QS::POSX] = gimbal_position_final[0];
  gimbal_state_.x[QS::POSY] = gimbal_position_final[1];
  gimbal_state_.x[QS::POSZ] = gimbal_position_final[2];
  gimbal_state_.x[QS::ATTW] = gimbal_orientation.w();
  gimbal_state_.x[QS::ATTX] = gimbal_orientation.x();
  gimbal_state_.x[QS::ATTY] = gimbal_orientation.y();
  gimbal_state_.x[QS::ATTZ] = gimbal_orientation.z();
  //
  gimbal_ptr_->setState(gimbal_state_);

  ROS_INFO("POSE CALLBACK");

  
}

void FlightPilotCustom::mainLoopCallback(const ros::TimerEvent &event) {

  ROS_INFO("LOOPING");

  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(frame_id);
    unity_bridge_ptr_->handleOutput();

    if (quad_ptr_->getCollision()) {
      // collision happened
      ROS_INFO("COLLISION");
    }

    cv::Mat img;

    ros::Time timestamp = ros::Time::now();

    rgb_camera2->getRGBImage(img);
    sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    rgb_msg->header.stamp = timestamp;
    rgb_pub.publish(rgb_msg);

    rgb_camera2->getDepthMap(img);
    sensor_msgs::ImagePtr depth_msg =
      cv_bridge::CvImage(std_msgs::Header(), "32FC1", img).toImageMsg();
    depth_msg->header.stamp = timestamp;
    depth_pub.publish(depth_msg);

    rgb_camera2->getSegmentation(img);
    sensor_msgs::ImagePtr segmentation_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    segmentation_msg->header.stamp = timestamp;
    segmentation_pub.publish(segmentation_msg);

    rgb_camera2->getOpticalFlow(img);
    sensor_msgs::ImagePtr opticflow_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    opticflow_msg->header.stamp = timestamp;
    opticalflow_pub.publish(opticflow_msg);

    frame_id += 1;
  }

}

bool FlightPilotCustom::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(gimbal_ptr_);
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightPilotCustom::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightPilotCustom::loadParams(void) {
  // load parameters
  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  return true;
}

}  // namespace flightros
