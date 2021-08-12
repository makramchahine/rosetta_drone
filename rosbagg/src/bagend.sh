#!/bin/bash

#kill bagstart.sh process
source /opt/ros/kinetic/setup.bash
source /home/dji/catkin_ws/devel/setup.bash

rosnode kill /bagger
#pkill -9 -f bagstart
#sleep 10
#rosservice call /setup_camera_stream 1 0
#umount /home/dji/flash

echo "Task completed"
/opt/ros/kinetic/bin/rosservice call /gimbal_task_control "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
is_reset: false
payload_index: 0
rotationMode: 0
pitch: 0.0
roll: 0.0
yaw: 90.0
time: 0.5"

#/opt/ros/kinetic/bin/rosservice call /gimbal_task_control "header:
#  seq: 0
#  stamp: {secs: 0, nsecs: 0}
#  frame_id: ''
#is_reset: false
#payload_index: 0
#rotationMode: 0
#pitch: 0.0
#roll: 0.0
#yaw: -90.0
#time: 0.5"
/opt/ros/kinetic/bin/rosservice call /gimbal_task_control "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
is_reset: true
payload_index: 0
rotationMode: 0
pitch: 0.0
roll: 0.0
yaw: 0.0
time: 0.5"



