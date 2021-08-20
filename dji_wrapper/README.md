This wrapper connects the DJI code with the UI. The UI publishes to common topics that the dji_wrapper subscribes to. The wrapper has different functions that will either directly or indirectly publish to the topics that the DJI drone uses.
There is also an rviz visualization that displays the position of the drone and gimbal based on the dji sdk topics. Gimtibal position is read from the gimbal_angles topic and position can be read from local_positon or gps_position (toggle self.use_gps in transform_publisher_node.py to choose which) while attitude comes from the attitude topic. The visualization can be launched by running dji_visualization.launch. There is also a controller visualization that reads from the drone’s velocity topics. This visualization can be launched by running controller_visualization.launch
How to start wrapper:
roscore
rosrun dji_wrapper dji_wrapper_node
rosrun UI flightmare_wrapper_UI_node
