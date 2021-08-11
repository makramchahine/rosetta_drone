#include "dji_wrapper/dji_wrapper.hpp" 

DJIWrapper::DJIWrapper(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh),
    pnh_(pnh){

  start_sub_ = nh_.subscribe("start", 1000, &DJIWrapper::starter,this);
  takeoff_sub_ = nh_.subscribe("takeoff", 1000, &DJIWrapper::takeoff, this);
  pos_sub_ = nh_.subscribe("pos", 1000, &DJIWrapper::go_to_pos,this);
  heading_sub_ = nh_.subscribe("heading", 1000, &DJIWrapper::set_heading, this);
  land_sub_ = nh_.subscribe("land", 1000, &DJIWrapper::land,this);
  off_sub_ = nh_.subscribe("off", 1000, &DJIWrapper::off,this);
  camera_pos_sub_ = nh_.subscribe("/camera_pos", 1000, &DJIWrapper::camera_pos, this);
  take_pic_sub_ = nh_.subscribe("take_pic", 1000, &DJIWrapper::take_pic, this);
  task_control_client = nh_.serviceClient<FlightTaskControl>("/flight_task_control");
  gimbal_control_client = nh_.serviceClient<GimbalAction>("/gimbal_task_control");
  set_home_sub_ = nh_.subscribe("set_home", 1000, &DJIWrapper::set_home, this);
 
  camera_start_shoot_single_photo_client = nh_.serviceClient<CameraStartShootSinglePhoto>(
      "camera_start_shoot_single_photo");
  camera_stop_shoot_photo_client = nh_.serviceClient<CameraStopShootPhoto>("camera_stop_shoot_photo");
  ROS_INFO("end of constructor");
 }
void DJIWrapper::localFrameRefSubCallback(const sensor_msgs::NavSatFix::ConstPtr& localFrameRef)
{
  local_Frame_ref_ = *localFrameRef;
}

void DJIWrapper::timeSyncNmeaSubSCallback(const nmea_msgs::Sentence::ConstPtr& timeSyncNmeaMsg)
{
  time_sync_nmea_msg_ = *timeSyncNmeaMsg;
}

void DJIWrapper::timeSyncGpsUtcSubCallback(const dji_osdk_ros::GPSUTC::ConstPtr& timeSyncGpsUtc)
{
  time_sync_gps_utc_ = *timeSyncGpsUtc;
}

void DJIWrapper::timeSyncFcUtcSubCallback(const dji_osdk_ros::FCTimeInUTC::ConstPtr& timeSyncFcUtc)
{
  time_sync_fc_utc_ = *timeSyncFcUtc;
}
void DJIWrapper::timeSyncPpsSourceSubCallback(const std_msgs::String::ConstPtr& timeSyncPpsSource)
{
  time_sync_pps_source_ = *timeSyncPpsSource;
}
void DJIWrapper::starter (const std_msgs::String &msg){ //this will start the drone and then do arm bridge
	// this will actually make the drone takeoff so only uncomment if you are ready for that

	// start:
	control_task.request.task = FlightTaskControl::Request::START_MOTOR;
	task_control_client.call(control_task);
  ROS_INFO("Start motors task succesful");
}

void DJIWrapper::takeoff (const std_msgs::String &msg){
  //takeoff:
  control_task.request.task = FlightTaskControl::Request::TASK_TAKEOFF;
  task_control_client.call(control_task);
  ROS_INFO("Takeoff task succesful");
}

void DJIWrapper::go_to_pos(const geometry_msgs::Point::ConstPtr &msg){

  // FlightTaskControl flightTaskControl;
  // flightTaskControl.request.task = FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
  // flightTaskControl.request.joystickCommand.x = msg->x;
  // flightTaskControl.request.joystickCommand.y = msg->y;
  // flightTaskControl.request.joystickCommand.z = msg->z;
  // flightTaskControl.request.joystickCommand.yaw = 0;
  // flightTaskControl.request.posThresholdInM   = .8;
  // flightTaskControl.request.yawThresholdInDeg = 1;

  // flight_control_client.call(flightTaskControl);
  ROS_INFO("Position control task succesful");
}

void DJIWrapper::set_heading(const std_msgs::Float32 &msg){
  // FlightTaskControl flightTaskControl;
  // flightTaskControl.request.task = FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
  // flightTaskControl.request.joystickCommand.x = 0;
  // flightTaskControl.request.joystickCommand.y = 0;
  // flightTaskControl.request.joystickCommand.z = 0;
  // flightTaskControl.request.joystickCommand.yaw = msg.data;
  // flightTaskControl.request.posThresholdInM   = .8;
  // flightTaskControl.request.yawThresholdInDeg = 1;
}

void DJIWrapper::land(const std_msgs::String &msg){
	// //this will actually make the drone land so only uncomment if you are ready for that
	// control_task.request.task = FlightTaskControl::Request::TASK_LAND;
 //  task_control_client.call(control_task);
	ROS_INFO("Land task succesful");
}

void DJIWrapper::off(const std_msgs::String &msg){
	//the drone WILL fall out of the sky if you call this mid flight
	// std::cout <<"Are you sure you want to turn off the motors? Insert 'y' to confirm, or 'n' to cancel: ";
	// //fill in input stuff
 //  char motors_off;
 //  std::cin >> motors_off;
 //  switch (motors_off)
 //  {
 //    case 'y':
	//     {
	// 			control_task.request.task = FlightTaskControl::Request::STOP_MOTOR;
	// 			task_control_client.call(control_task);
	// 			ROS_INFO("Drone turned off");
  //      break;
	//     }
 //    case 'n':
	//     {
	//     	break;
	//   	}
 //  }  

	
}

void DJIWrapper::camera_pos(const geometry_msgs::Quaternion::ConstPtr &msg){
	//this is for the gimbal, not main camera
  ROS_INFO("in camera pos");
	tf::Quaternion quat_tf;
	tf::quaternionMsgToTF(*msg, quat_tf);

	double roll, pitch, yaw;
	tf::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

  ROS_INFO("R%fP%fY%f", roll, pitch, yaw);

  GimbalAction gimbalAction;
  gimbalAction.request.is_reset = false;
  gimbalAction.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
  gimbalAction.request.rotationMode = 0;
  gimbalAction.request.pitch = pitch*180/M_PI;
  gimbalAction.request.roll = roll*180/M_PI;
  gimbalAction.request.yaw = yaw*180/M_PI;
  gimbalAction.request.time = 0.5;
  gimbal_control_client.call(gimbalAction);
}

void DJIWrapper::take_pic(const std_msgs::String &msg){
	//this is for the gimbal, not the main camera
	cv::Mat img;

  CameraStartShootSinglePhoto cameraStartShootSinglePhoto;
  cameraStartShootSinglePhoto.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
  camera_start_shoot_single_photo_client.call(cameraStartShootSinglePhoto);
  //cv::imwrite("/home/ita/flightmare_ws/dronepic.jpg", img);

}

void DJIWrapper::set_home(const::std_msgs::String &msg){
  localFrameRefSub     = nh_.subscribe("dji_osdk_ros/local_frame_ref", 10, &DJIWrapper::localFrameRefSubCallback,this);
  timeSyncNmeaSub      = nh_.subscribe("dji_osdk_ros/time_sync_nmea_msg", 10, &DJIWrapper::timeSyncNmeaSubSCallback,this);
  timeSyncGpsUtcSub    = nh_.subscribe("dji_osdk_ros/time_sync_gps_utc", 10, &DJIWrapper::timeSyncGpsUtcSubCallback,this);
  timeSyncFcUtcSub     = nh_.subscribe("dji_osdk_ros/time_sync_fc_time_utc", 10, &DJIWrapper::timeSyncFcUtcSubCallback,this);
  timeSyncPpsSourceSub = nh_.subscribe("dji_osdk_ros/time_sync_pps_source", 10, &DJIWrapper::timeSyncPpsSourceSubCallback,this);

  auto local_frame_ref_client = nh_.serviceClient<dji_osdk_ros::SetLocalPosRef>("set_local_pos_reference");
  dji_osdk_ros::SetLocalPosRef local_frame_ref;
  local_frame_ref_client.call(local_frame_ref);
  if (local_frame_ref.response.result)
  {
     ROS_INFO("localFrameRef:");
     ROS_INFO("local_Frame_ref_(latitude, longitude, altitude) :%f, %f, %f\n ",
     local_Frame_ref_.latitude, local_Frame_ref_.longitude, local_Frame_ref_.altitude);
  }
}