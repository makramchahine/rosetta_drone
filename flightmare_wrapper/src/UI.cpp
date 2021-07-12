#include "flightmare_wrapper/flightmare_wrapper.hpp"

#include <sstream>

void publisher(std::string topic){
  ros::NodeHandle n;
  ros::Publisher moveit_pub = n.advertise<std_msgs::String>(topic, 1000);
  std_msgs::String msg;

  std::stringstream ss;
  ss << "hi";
  msg.data = ss.str();

  while(moveit_pub.getNumSubscribers()==0 && ros::ok()){
  	ROS_INFO("waiting");
	}
  ros::WallDuration sleep_t(1);
  sleep_t.sleep();
  moveit_pub.publish(msg);
  ROS_INFO("Published");

}
int main(int argc, char **argv){
  ros::init(argc, argv, "UI_pub");
  ros::NodeHandle n;
  std::cout
      << "| Available commands:               |"
      << std::endl;
  std::cout
      << "| [a] Start                         |"
      << std::endl;
  std::cout
      << "| [b] Go To Position              |"
      << std::endl;
  std::cout 
  	  << "| [c] Land 				|"
      << std::endl;
  std::cout 
  	  << "| [d] Off 				|"
      << std::endl;

  std::cout << "Please select command: ";
  char inputChar;
  std::cin >> inputChar;
  
  switch (inputChar)
  {
    case 'a':
      {
      	publisher("hummingbird/start");
      	break;
      }
    case 'b':
      {
      	ros::Publisher moveit_pub = n.advertise<geometry_msgs::Point>("hummingbird/pos", 1000);
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

		    while(moveit_pub.getNumSubscribers()==0 && ros::ok()){
	  	  	ROS_INFO("waiting");
				}
			  ros::WallDuration sleep_t(1);
			  sleep_t.sleep();
			  moveit_pub.publish(msg);
			  ROS_INFO("Published");
      	break;
      }
    case 'c':
      {
      	publisher("hummingbird/land");
      	break;
      }
    case 'd':
      {
      	publisher("hummingbird/off");
      	break;
      }                  
  }
  ros::spinOnce();
  return 0; 
}        