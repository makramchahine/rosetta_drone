The wrapper connects flightmare to the UI. It subscribes to the common topics that the UI publishes to. This wrapper also has different functions to accomplish different tasks. Every function in fightmare_wrapper has a similar function in dji_wrapper that should have the same outcome. Most of the functions use autopilot helper, some use other functions provided by flightmare instead of publishing directly to flightmare’s topics.  
How to start wrapper:  
roslaunch flightmare_wrapper unity_flightmare_wrapper.launch  
rosrun UI flightmare_wrapper_UI_node  
