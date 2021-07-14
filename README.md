# Rosetta Drone

## Installation Guide

### ROS

Install ROS based on the guide on the ROS wiki at [http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)  

This was developed and tested with ROS Melodic on Ubuntu 18.04. It may work with other versions, but we haven't tested it.

### Making a workspace

If you haven't already, create a workspace

~~~~
cd ~
mkdir -p flightmare_ws/src
cd flightmare_ws
catkin config --init --mkdirs --extend /opt/ros/$ROS_DISTRO --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
~~~~

### Flightmare


For the most part, use the [standard flightmare installation instructions](https://github.com/uzh-rpg/flightmare/wiki/Install-with-ROS) except for some modifications to account for opencv versions.  

This project requires opencv 3.2.0 and we have had the most success by installing it implicitly through flightmare - other methods have led to nasty version conflicts with opencv 4. This means that we use the normal installation guide except we replace the first command with:  
~~~~~
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
   build-essential \
   cmake \
   libzmqpp-dev
~~~~~

which is the same thing but with libopencv-dev removed and the command split into two commands (which seems to cause fewer issues in some cases).  

It is also worth noting that at one step for installing dependencies, you need to have ssh keys set up with your github to be able to use their setup utility. There is a guide on how to set that up [here](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh)

