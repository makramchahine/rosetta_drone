# Rosetta Drone

## Installation Guide

### ROS

Install ROS based on the guide on the ROS wiki at [http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)  

This was developed and tested with ROS Melodic on Ubuntu 18.04. It may work with other versions, but we have not tested it.

### Making a workspace

If you have not already, create a workspace

~~~~
cd ~
mkdir -p flightmare_ws/src
cd flightmare_ws
catkin config --init --mkdirs --extend /opt/ros/$ROS_DISTRO --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
~~~~

### Flightmare


For the most part, use the [standard flightmare installation instructions](https://github.com/uzh-rpg/flightmare/wiki/Install-with-ROS) except for some modifications to account for opencv versions. The flightmare github repo is [here](https://github.com/uzh-rpg/flightmare)  

This project requires opencv 3.2.0 and we have had the most success by installing it implicitly through flightmare - other methods have led to nasty version conflicts with opencv 4. This means that we use the normal installation guide except we replace the first command with:  
~~~~~
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
   build-essential \
   cmake \
   libzmqpp-dev
~~~~~

which is the same thing but with `libopencv-dev` removed and the command split into two commands (which seems to cause fewer issues in some cases).  

It is also worth noting that at one step for installing flightmare dependencies, you need to have ssh keys set up with your github to be able to use their easy install system. There is a guide on how to set that up [here](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh)

Also note that the workspace may have a different name when updating `~/.bashrc` and the default name `catkin_ws` may have to be changed at the end of their guide.  
We have also found that `catkin_make` works better than their suggested `catkin build`.  


### DJI SDK ROS

The DJI SDK with ROS repo is [here](https://github.com/dji-sdk/Onboard-SDK-ROS).  

We follow their installation guide, but with a few clarifications.  

When installing [onboard-sdk](https://github.com/dji-sdk/Onboard-SDK), the `build/` directory should be made under the `Onboard-SDK/` directory. This shouldn't be in your catkin workspace. There is also a typo on their page; there should be a space in the cmake command so it would be `cmake ..`.  

There is no need to install opencv through their link since it was already installed when installing flightmare.  

For the stereo vision function, the relevant part of the install guide is in the "Sample 1" section here: https://developer.dji.com/onboard-sdk/documentation/sample-doc/advanced-sensing-object-detection.html#sample-2-object-depth-perception-in-stereo-image  

Which is mostly just doing `git clone --recursive https://github.com/leggedrobotics/darknet_ros.git` in `src/`.

Run `catkin_make` to verify that everything up until now is installed properly.  

### Rosetta Drone

Clone this repo into `src/` and do `catkin_make`