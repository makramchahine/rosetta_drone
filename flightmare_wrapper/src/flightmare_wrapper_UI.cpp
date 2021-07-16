//this is for Ita 6/9/2001
//you gotta do the cmakefile, package, and a new launch file have fun :)

#include "flightmare_wrapper/flightmare_wrapper_UI.hpp"


UI::UI(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh),
    pnh_(pnh) {
  start_pub_ = nh_.advertise<std_msgs::String>("hummingbird/start", 1000);
  takeoff_pub_ = nh_.advertise<std_msgs::String>("hummingbird/takeoff", 1000);
  pos_pub_ =  nh_.advertise<geometry_msgs::Point>("hummingbird/pos", 1000);
  land_pub_  = nh_.advertise<std_msgs::String>("hummingbird/land", 1000);
  off_pub_  = nh_.advertise<std_msgs::String>("hummingbird/off", 1000);
  camera_pos_pub_ = nh_.advertise<geometry_msgs::Quaternion>("hummingbird/camera_pos", 1000);
  take_pic_pub_ = nh_.advertise<std_msgs::String>("hummingbird/take_pic", 1000);
  stream_pic_pub_ = nh_.advertise<std_msgs::String>("hummingbird/stream_pic", 1000);
  while(ros::ok()){
    selection();
    ros::spinOnce();
  }
  
}

void UI::selection(){

  std::cout
      << "| Available commands:               |"
      << std::endl;
  std::cout
      << "| [a] Start                         |"
      << std::endl;
  std::cout
      << "| [b] Takeoff                         |"
      << std::endl;      
  std::cout
      << "| [c] Go To Position              |"
      << std::endl;
  std::cout 
  	  << "| [d] Land 			|"
      << std::endl;
  std::cout 
  	  << "| [e] Off 			|"
      << std::endl;
  std::cout 
      << "| [f] Rotate Camera       |"
      << std::endl;
  std::cout 
      << "| [g] Take Picture       |"
      << std::endl;  
    std::cout
      << "| [h] Stream Picture                 |"
      << std::endl;      
  std::cout 
      << "| [q] Kill Node         |"
      << std::endl;
  std::cout << "Please select command: ";
  char inputChar;
  std::cin >> inputChar;
  
  switch (inputChar)
  {
    case 'a':
      {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hi";
        msg.data = ss.str();
        start_pub_.publish(msg);
        ROS_INFO("Published");
        break;
      }
    case 'b':
      {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hi";
        msg.data = ss.str();
        takeoff_pub_.publish(msg);
        ROS_INFO("Published");
        break;
      }      
    case 'c':
      {
	  		geometry_msgs::Point msg;

      	std::cout << "Please insert Z position: ";
			  float z_pos;
			  std::cin >> z_pos;
			  msg.z=z_pos;

      	std::cout << "Please insert Y position: ";
			  float y_pos;
			  std::cin >> y_pos;
			  msg.y=y_pos;

      	std::cout << "Please insert X Position: ";
			  float x_pos;
			  std::cin >> x_pos;
			  msg.x=x_pos;

			  pos_pub_.publish(msg);
			  ROS_INFO("Published");
      	break;
      }
    case 'd':
      {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hi";
        msg.data = ss.str();
        land_pub_.publish(msg);
        ROS_INFO("Published");
        break;
      }
    case 'e':
      {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hi";
        msg.data = ss.str();
        off_pub_.publish(msg);
        ROS_INFO("Published");
        break;
      }
    case 'f':
      {
        geometry_msgs::Quaternion msg;

        std::cout << "Please insert Roll: ";
        float roll;
        std::cin >> roll;

        std::cout << "Please insert Pitch: ";
        float pitch;
        std::cin >> pitch;

        std::cout << "Please insert Yaw: ";
        float yaw;
        std::cin >> yaw;

        tf2::Quaternion myQuaternion;

        myQuaternion.setRPY((roll*M_PI/180),(pitch*M_PI/180),(yaw*M_PI/180));
        geometry_msgs::Quaternion quat_msg;

        myQuaternion.normalize();

        msg = tf2::toMsg(myQuaternion);

        msg.x = myQuaternion.x();
        msg.y = myQuaternion.y();
        msg.z = myQuaternion.z();
        msg.w = myQuaternion.w();

        

        camera_pos_pub_.publish(msg);
        ROS_INFO("Published");
        break;
      }       
    case 'g':
      {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hi";
        msg.data = ss.str();
        take_pic_pub_.publish(msg);
        ROS_INFO("Published");
        break;        
      }           
    case 'h':
      {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hi";
        msg.data = ss.str();
        stream_pic_pub_.publish(msg);
        ROS_INFO("Published");
        break;
      }               
    case 'q':
      {
        ros::shutdown();
      }                 
  }
 
}        