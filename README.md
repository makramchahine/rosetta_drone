# Rosetta Drone
This repository contains all code run onboard the drone to complete online testinf for the paper "Robust Visual Flight Navigation with Liquid Neural Networks". For training/offline analysis code, see [this repository](https://github.com/makramchahine/drone_causality)

## Installation Guide
### Hardware
This repository was tested on a DJI Manifold 2 connected to a DJI Matrice 300 using a Zenmuse z30 gimbal camera.
### ROS

This was developed and tested with ROS Melodic on Ubuntu 18.04. It may work with other versions, but we have not tested it. 

If using Melodic, it needs to be installed with Python3 bindings to support the version of tensorflow used. Use the following directions to install Melodic with python bindings

~~~
apt install ros-melodic-desktop-full
~~~
After installing ROS, install rospkg for python3
~~~
apt install python3-pip python3-all-dev python3-rospkg
~~~
This will prompt to install python3-rospkg and to remove ROS packages (already installed). Select Yes for that prompt. This will remove ROS packages and we will have to re-install them.
~~~
apt install ros-melodic-desktop-full --fix-missing
~~~


### Making a workspace

If you have not already, create a workspace

~~~~
cd ~
mkdir -p flightmare_ws/src
cd flightmare_ws
catkin config --init --mkdirs --extend /opt/ros/$ROS_DISTRO --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
~~~~

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

### Python Environment Setup
If using conda, setup python environment with

~~~
conda env create -f environment.yml
conda activate matrice
~~~

This conda environment above is for x86-based machines, and not for the drone hardware platform. For the ARM-based Manifold, instead run

~~~
pip install -r requirements.txt
~~~

Note tensorflow-probabilities and keras-tcn are optional and only for ctrnn and tcn network types. They can't be installed just through pip on some systems, and need the following steps to install:

#### Tensorflow Probability
When you run pip3 install tensorflow-probability, the dependency dm-tree lacks a wheel and instead needs bazel to be built from scratch. To install bazel:

- Download a binary from [here](https://github.com/bazelbuild/bazel/releases) for linux-arm64, ex https://github.com/bazelbuild/bazel/releases/download/5.0.0/bazel-5.0.0-linux-arm64
- Chmod +x the binary
- Move the binary to /bin/bazel
- Note the default version installed is the latest, the latest version compatible with tf 2.3.x is [0.11.1](https://github.com/tensorflow/probability/releases/tag/v0.11.1) (ex use pip3 install tensorflow-probability==0.11.1)
#### Keras-tcn
By default, if you just run pip3 install keras-tcn, you get version 3.1.2, which lacks support for the features I trained with. Newer packages won’t install because they’re missing something called tensorflow-addons. To install tensorflow addons, follow the instructions at [this repo](https://github.com/sujeendran/tensorflow-addons), which was designed for the Jetson nano

After installing tensorflow-addons, make sure you have the latest keras-tcn version with `pip3 install --upgrade keras-tcn`


## Flying with policies
In order to run networks to control the drone, first ensure that the DJI OSDK vehicle node is launched prior to forward control signals. Afterwards, call the DJI OSDK start camera stream setup to get images from the gimbal camera to publish on the topic /main_camera_images.

~~~
roslaunch dji_osdk_ros dji_vehicle_node.launch
rosservice call /setup_camera_stream 1 1
~~~

### RNN control node
This node runs networks to control the drone directly, and is the script used for experiments in the paper. To use the set of models in the paper, use the launch file at `rnn_control/launch/network_launch/control_logging_node_fine_train.launch`.

Before calling this script, the rosparam `model_name` has to be set in order to specify which type of network should be run. The elgible model types are specified in `keras_models.py`. The types used for experiments in the paper are:
- ncp
- lstm
- cfc
- ltc
- gruode
- tcn
- wiredcfccell (this is called Sparse-CfC in paper)
- ctrnn

Ex: for ncp run
~~~
rosparam set model_name ncp
roslaunch /path/to/rnn_control/launch/network_launch/control_logging_node_fine_train.launch
~~~

Once the script is running, you can fly as normal until engaging the network by moving the mode select switch to the right. To disengage the network and retake manual control authority, flip the mode switch to the left.

Image folders and csv logs will automatically be saved to /data/dji/flash, which can be changed by editing the launch file.

### Saliency control node
This node uses the CNN heads of the networks to generates saliency maps, which are then segmented to determine contours. These contours are then fed to a PID controller that steers towards them. To use set the rosparam `checkpoint_path` to the path of the model used for saliency map generation and use the launch file `rnn_control/launch/saliency_control_node.launch`.

If you want a network that is known to produce good saliency maps, use the launch file at `rnn_control/launch/saliency_control_node_ncp.launch`
