#sudo mount /dev/sda1 /home/dji/flash

#Topics we are to record other than camera images
bag="/dji_osdk_ros/acceleration_ground_fused /dji_osdk_ros/angular_velocity_fused /dji_osdk_ros/attitude /dji_osdk_ros/gps_health /dji_osdk_ros/gps_position /dji_osdk_ros/height_above_takeoff /dji_osdk_ros/imu /dji_osdk_ros/local_frame_ref /dji_osdk_ros/local_position /dji_osdk_ros/rc /dji_osdk_ros/rc_connection_status /dji_osdk_ros/time_sync_fc_time_utc /dji_osdk_ros/time_sync_gps_utc /dji_osdk_ros/time_sync_nmea_msg /dji_osdk_ros/time_sync_pps_source /dji_osdk_ros/trigger_time /dji_osdk_ros/velocity /dji_osdk_ros/vo_position /dji_osdk_ros/gimbal_angle"


#Main Camera

bag="${bag} /dji_osdk_ros/main_camera_images"
rosservice call /setup_camera_stream 1 1

#Storing Topics in ROS Bag
#cd /media/dji/flash
#rosbag record --split --duration=1m -o subset $bag