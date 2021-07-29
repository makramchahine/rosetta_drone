#Topics we are to record other than camera images
bag="/dji_osdk_ros/acceleration_ground_fused /dji_osdk_ros/angular_velocity_fused /dji_osdk_ros/attitude /dji_osdk_ros/gps_health /dji_osdk_ros/gps_position /dji_osdk_ros/height_above_takeoff /dji_osdk_ros/imu /dji_osdk_ros/local_frame_ref /dji_osdk_ros/local_position /dji_osdk_ros/rc /dji_osdk_ros/rc_connection_status /dji_osdk_ros/time_sync_fc_time_utc /dji_osdk_ros/time_sync_gps_utc /dji_osdk_ros/time_sync_nmea_msg /dji_osdk_ros/time_sync_pps_source /dji_osdk_ros/trigger_time /dji_osdk_ros/velocity /dji_osdk_ros/vo_position"

#Selecting which camera we want to record
echo "Enter choice for camera: f- FPV Camera m-Main Camera"
echo "l- stereo vga front left r- stereo vga front right"
read usr

#FPV Camera
if grep -q "f" <<< "$usr"; then
bag="${bag} /dji_osdk_ros/fpv_camera_images"
rosservice call /setup_camera_stream 0 1
fi
#Main Camera
if grep -q "m" <<< "$usr"; then
bag="${bag} /dji_osdk_ros/main_camera_images"
rosservice call /setup_camera_stream 1 1
fi
#Stereo VGA FRONT LEFT
if grep -q "l" <<< "$usr"; then
bag="${bag} /dji_osdk_ros/stereo_vga_front_left_images"
rosservice call /stereo_vga_subscription 0 1 0
fi
#STEREO VGA FRONT RIGHT
if grep -q "r" <<< "$usr"; then
bag="${bag} /dji_osdk_ros/stereo_vga_front_right_images"
rosservice call /stereo_vga_subscription 0 1 0
fi

#Functions to close Camera after execution of ROS bag ended with ctrl C
function closing_stream {
  if grep -q "f" <<< "$usr"; then
	rosservice call /setup_camera_stream 0 0
fi
if grep -q "m" <<< "$usr"; then
	rosservice call /setup_camera_stream 1 0
fi
rosservice call /stereo_vga_subscription 0 1 1
}
trap closing_stream EXIT

#Storing Topics in ROS Bag
rosbag record -O subset $bag
