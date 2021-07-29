#include "flightmare_wrapper/flightmare_wrapper.hpp"

FlightmareWrapper::FlightmareWrapper(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::INDUSTRIAL),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(50.0) {

  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  start_sub_ = nh_.subscribe("start", 1000, &FlightmareWrapper::starter,this);
  takeoff_sub_ = nh_.subscribe("takeoff",1000, &FlightmareWrapper::takeoff, this);
  pos_sub_ = nh_.subscribe("pos", 1000, &FlightmareWrapper::go_to_pos,this);
  heading_sub_ = nh_.subscribe("heading", 1000, &FlightmareWrapper::set_heading, this);
  land_sub_ = nh_.subscribe("land", 1000, &FlightmareWrapper::land,this);
  off_sub_ = nh_.subscribe("off", 1000, &FlightmareWrapper::off,this);
  camera_pos_sub_ = nh_.subscribe("camera_pos", 1000, &FlightmareWrapper::camera_pos, this);
  take_pic_sub_ = nh_.subscribe("take_pic", 1000, &FlightmareWrapper::take_pic, this);
  set_waypoints_sub_ = nh_.subscribe("set_waypoints", 1000, &FlightmareWrapper::set_waypoints, this);



  arm_pub_ = nh_.advertise<std_msgs::Bool>("bridge/arm", 1);




  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();

  /*
  // add mono camera
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


  // initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("flight_pilot/state_estimate", 1,
                                 &FlightmareWrapper::poseCallback, this);

  timer_main_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_),
                                     &FlightmareWrapper::mainLoopCallback, this);
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
  Vector<3> B_r_BC2(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC2 = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC2 << std::endl;
  rgb_camera2->setFOV(90);
  rgb_camera2->setWidth(640);
  rgb_camera2->setHeight(360);
  rgb_camera2->setRelPose(B_r_BC2, R_BC2);
  rgb_camera2->setPostProcesscing(
    std::vector<bool>{true, true, true});  // depth, segmentation, optical flow
  quad_ptr_->addRGBCamera(rgb_camera2);


  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();

  // connect unity
  setUnity(unity_render_);
  connectUnity();
}

void FlightmareWrapper::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  //Quaternion current_orientation = quad_state_.q();

  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;
  //
  quad_ptr_->setState(quad_state_);
  
  ROS_INFO("POSE CALLBACK");

  
}

void FlightmareWrapper::mainLoopCallback(const ros::TimerEvent &event) {

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

bool FlightmareWrapper::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightmareWrapper::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightmareWrapper::loadParams(void) {
  // load parameters
  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  return true;
}


void FlightmareWrapper::starter (const std_msgs::String &msg){ //this will start the drone and then do arm bridge
  autopilot_helper_.sendStart();
  ROS_INFO("Start");
}

void FlightmareWrapper::takeoff (const std_msgs::String &msg){ //this will start the drone and then do arm bridge
  std_msgs::Bool arm_msg;
  arm_msg.data = true;
  arm_pub_.publish(arm_msg);
}

void FlightmareWrapper::go_to_pos (const geometry_msgs::Point::ConstPtr &msg){ 
  const Eigen::Vector3d position_cmd = Eigen::Vector3d(msg->x, msg->y, msg->z);
  Eigen::Quaternion current_orientation = quad_state_.q();

  geometry_msgs::Quaternion goemetry_quat;
  goemetry_quat.x=double(current_orientation.x());
  goemetry_quat.y=double(current_orientation.y());
  goemetry_quat.z=double(current_orientation.z());
  goemetry_quat.w=double(current_orientation.w());
  tf2::Quaternion quat_tf;

  tf2::convert(goemetry_quat, quat_tf);
  double roll, pitch, yaw;
  
  tf2::Matrix3x3 matrix(quat_tf);
  matrix.getRPY(roll, pitch, yaw);
  const double heading_cmd = yaw;
  autopilot_helper_.sendPoseCommand(position_cmd, heading_cmd);
}

void FlightmareWrapper::set_heading (const std_msgs::Float32 &msg){
  Eigen::Vector3d current_pos;
  Vector<3> position = quad_ptr_ -> getPosition();
  current_pos[0]=position[0];
  current_pos[1]=position[1];
  current_pos[2]=position[2];
  const double heading_cmd=msg.data/180*M_PI;
  autopilot_helper_.sendPoseCommand(current_pos, heading_cmd);

}

void FlightmareWrapper::land (const std_msgs::String &msg){ //land the drone
  autopilot_helper_.sendLand();
}

void FlightmareWrapper::off(const std_msgs::String &msg){ //turn the motors off
  autopilot_helper_.sendOff();
}

void FlightmareWrapper::camera_pos(const geometry_msgs::Quaternion &msg){
  // attach camera to drone
  //nav_msgs::Odometry::ConstPtr send;
  Vector<3> current_pos = quad_ptr_ -> getPosition();

  quad_state_.x[QS::POSX] = current_pos[0];
  quad_state_.x[QS::POSY] = current_pos[1];
  quad_state_.x[QS::POSZ] = current_pos[2];
  quad_state_.x[QS::ATTW] = msg.w;
  quad_state_.x[QS::ATTX] = msg.x;
  quad_state_.x[QS::ATTY] = msg.y;
  quad_state_.x[QS::ATTZ] = msg.z;
  //
  quad_ptr_->setState(quad_state_);

}

void FlightmareWrapper::take_pic(const std_msgs::String &msg){
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
      cv::imwrite("/home/ita/flightmare_ws/simulator_pic.jpg", img);
  }
}

void FlightmareWrapper::set_waypoints(const nav_msgs::Path &msg){
 // This is stuff from integration_test, please clean up and use what you need
//   std::vector<Eigen::Vector3d> way_points ;

   
// //this is from racing - its cleaner pls use it here 
//   std::size_t num_waypoints = way_points.size();


//   Eigen::VectorXd segment_times(num_waypoints);
//   segment_times << 10.0, 10.0, 10.0, 10.0;
//   Eigen::VectorXd minimization_weights(5);
//   minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;

//   polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings =
//     polynomial_trajectories::PolynomialTrajectorySettings(
//       way_points, minimization_weights, 7, 4);

//   polynomial_trajectories::PolynomialTrajectory trajectory =
//     polynomial_trajectories::minimum_snap_trajectories::
//       generateMinimumSnapRingTrajectory(segment_times, trajectory_settings,
//                                         20.0, 20.0, 6.0);


//   autopilot_helper_.sendTrajectory(trajectory);


}