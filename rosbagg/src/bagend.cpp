#pragma once

#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>

using namespace dji_osdk_ros;

int check=0;

void bagend(const std_msgs::String s){
    if (s.data == 1){
        check=1;
    }
}

void bagcheck(const sensor_msgs::Joy s){
    if (check==1 && s.axes[4]==0){
        check =2;

    }
    if (check==2 && s.axes[4]==8000){
        int res = system("sudo bagend.sh");
        check =0;
    }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "bagend_node");
  ros::NodeHandle nh;
  ros::Subscriber bagg_sub_ = nh.subscribe<std_msgs::Int8>("/bag_info", 10, bagend);
  ros::Subscriber rc_sub_=nh.subscribe<sensor_msgs::Joy>("/dji_osdk_ros/rc", 10, bagcheck);

  ros::spin();
}