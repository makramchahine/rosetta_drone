
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv){
	ros::init(argc, argv, "temp_pub");
  ros::NodeHandle n;
 
  ros::Publisher moveit_pub = n.advertise<std_msgs::String>("hummingbird/moveit", 1000);
  std_msgs::String msg;

  std::stringstream ss;
  ss << "hello";
  msg.data = ss.str();

  while(moveit_pub.getNumSubscribers()==0 && ros::ok()){
  	ROS_INFO("waiting");
	}
	ros::WallDuration sleep_t(1);
	sleep_t.sleep();
  moveit_pub.publish(msg);
  ROS_INFO("Published");
	ros::spinOnce();
	return 0;

	/*
 char inputChar;
 std::cin >> inputChar;
 switch (inputChar)
  {
    case 'a':
      {
  
	  ros::init(argc, argv, "temp_pub");
	  ros::NodeHandle n;

	  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("moveit", 1000);

	  ros::Rate loop_rate(10);


		  while (ros::ok())
		  {

		    ROS_INFO("Publishing");
		    std_msgs::String msg;

		    std::stringstream ss;
		    ss << "a";
		    msg.data = ss.str();

		    chatter_pub.publish(msg);

		    ros::spinOnce();

		    loop_rate.sleep();
		  	}

      }
  }
  */
  }