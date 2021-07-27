//this is for Ita 6/9/2001
//you gotta do the cmakefile, package, and a new launch file have fun :)

#include "flightmare_wrapper/flightmare_wrapper_UI.hpp"


UI::UI(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh),
    pnh_(pnh) {
  start_pub_ = nh_.advertise<std_msgs::String>("hummingbird/start", 1000);
  takeoff_pub_ = nh_.advertise<std_msgs::String>("hummingbird/takeoff", 1000);
  pos_pub_ =  nh_.advertise<geometry_msgs::Point>("hummingbird/pos", 1000);
  heading_pub_ = nh_.advertise<std_msgs::Float32>("hummingbird/heading",1000);
  land_pub_  = nh_.advertise<std_msgs::String>("hummingbird/land", 1000);
  off_pub_  = nh_.advertise<std_msgs::String>("hummingbird/off", 1000);
  camera_pos_pub_ = nh_.advertise<geometry_msgs::Quaternion>("hummingbird/camera_pos", 1000);
  take_pic_pub_ = nh_.advertise<std_msgs::String>("hummingbird/take_pic", 1000);
  set_waypoints_pub_ = nh_.advertise<nav_msgs::Path>("hummingbird/set_waypoints",1000);
  while(ros::ok()){
    selection();
    ros::spinOnce();
  }
  
}

void UI::selection(){

  std::cout
      << "| Available commands:               "
      << std::endl;
  std::cout
      << "| [a] Start                         "
      << std::endl;
  std::cout
      << "| [b] Takeoff                      "
      << std::endl;      
  std::cout
      << "| [c] Go To Position              "
      << std::endl;
  std::cout
      << "| [d] Set Heading              "
      << std::endl;    
  std::cout 
  	  << "| [e] Land 			"
      << std::endl;
  std::cout 
  	  << "| [f] Off 			"
      << std::endl;
  std::cout 
      << "| [g] Rotate Camera       "
      << std::endl;
  std::cout 
      << "| [h] Take Picture       "
      << std::endl;  
  std::cout 
      << "| [i] Set and go to Waypoints       "
      << std::endl;         
  std::cout 
      << "| [q] Kill Node         "
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
        std_msgs::Float32 msg;

        std::cout << "Please insert Heading: ";
        float heading;
        std::cin >> heading;
        msg.data=heading;

        heading_pub_.publish(msg);
        ROS_INFO("Published");
        break;
      }      
    case 'e':
      {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hi";
        msg.data = ss.str();
        land_pub_.publish(msg);
        ROS_INFO("Published");
        break;
      }
    case 'f':
      {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hi";
        msg.data = ss.str();
        off_pub_.publish(msg);
        ROS_INFO("Published");
        break;
      }
    case 'g':
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
    case 'h':
      {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hi";
        msg.data = ss.str();
        take_pic_pub_.publish(msg);
        ROS_INFO("Published");
        break;        
      }           
    case 'i':
      {
    //     nav_msgs::Path path;
    //     geometry_msgs::PoseStamped pose;
    //     std::cout
    //         << "| Available commands:               |"
    //         << std::endl;
    //     std::cout
    //         << "| [a] Set Waypoint                         |"
    //         << std::endl;
    //     std::cout
    //         << "| [b] Start Path                         |"
    //         << std::endl;
    //     char newInput;
    //     std::cin >> newInput;
    //     switch (inputChar)
    //     {
    //       case 'a':
    //         {
    //           std::cout << "Please insert X Position: ";
    //           float x;
    //           std::cin >> x;

    //           std::cout << "Please insert Y Position: ";
    //           float y;
    //           std::cin >> y;

    //           std::cout << "Please insert Z Position: ";
    //           float z;
    //           std::cin >> z;  

    //           pose.pose.position.x = x;
    //           pose.pose.position.y = y;
    //           pose.pose.position.z = z;
              
    //           path.poses.push_back(pose);

    //           std::cout << "Please insert second X Position: ";
    //           std::cin >> x;

    //           std::cout << "Please insert second Y Position: ";
    //           std::cin >> y;

    //           std::cout << "Please insert second Z Position: ";
    //           std::cin >> z;  

    //           pose.pose.position.x = x;
    //           pose.pose.position.y = y;
    //           pose.pose.position.z = z;
              
    //           path.poses.push_back(pose);

    //           set_waypoints_pub_.publish(path);
    //           break;
    //         }
    //       case 'b':
    //         {
    //           set_waypoints_pub_.publish(path);
    //           break;
    //         }      

    //     break;
    //   }
     }                     
    case 'q':
      {
        ros::shutdown();
      }                 
  }
 
}        