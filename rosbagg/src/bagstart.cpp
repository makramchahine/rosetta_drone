#pragma once

#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>

using namespace dji_osdk_ros;

int state =0;
ros::Publisher bagg_publisher_;

std_msgs::Int8 msg0;
std_msgs::Int8 msg1;

void bagstart(const sensor_msgs::Joy msg){
  //std::cout << state;
    if (state ==0 && msg.axes[4]==0){
      state=1;
    }
    if (state == 1 && msg.axes[4]==8000){
      state=0;
      bagg_publisher_.publish(msg1);
      int res = system("sudo /home/dji/catkin_ws/src/rosbagg/src/bagstart.sh");
    }
    bagg_publisher_.publish(msg0);
    //std::cout <<msg.axes[4];
    /***if (msg.axes[0]>0.8 && msg.axes[1]<-0.8 && msg.axes[2]<-0.8 && msg.axes[3]<-0.8){
        
    }***/
}



int main(int argc, char** argv) {
  msg0.data = 0;
  msg1.data = 1;
  ros::init(argc, argv, "bagstart_node");
  ros::NodeHandle nh;
  bagg_publisher_ = nh.advertise<std_msgs::Int8>("bag_info", 10);
  ros::Subscriber rc_sub_=nh.subscribe<sensor_msgs::Joy>("/dji_osdk_ros/rc", 10, bagstart);
  
  ros::spin();
}
