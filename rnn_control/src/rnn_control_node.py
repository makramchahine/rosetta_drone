#!/usr/bin/env python3
import os
import sys

import PIL
import PIL.Image
import cv2
import dji_osdk_ros.srv as dji_srv
import kerasncp as kncp
import numpy as np
import rospy
import tensorflow as tf
from kerasncp.tf import LTCCell
from sensor_msgs.msg import Image
from tensorflow import keras

# import logger from other ros package. Add to system path instead of including proper python lib dependency
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(script_dir, "..", ".."))
from video_compression.scripts.logger_example import Logger


def dji_msg_from_velocity(vel_cmd):
    req = dji_srv.FlightTaskControlRequest()
    req.task = dji_srv.FlightTaskControlRequest.TASK_VELOCITY_AND_YAWRATE_CONTROL
    req.joystickCommand.x = vel_cmd[0]
    req.joystickCommand.y = vel_cmd[1]
    req.joystickCommand.z = vel_cmd[2]
    req.joystickCommand.yaw = vel_cmd[3]
    req.velocityControlTimeMs = 40
    return req


def load_model(model_name: str, checkpoint_name: str):
    # make sure checkpoint includes script dir so script can be run from any file
    checkpoint_path = os.path.join(script_dir, checkpoint_name)

    last_model = tf.keras.models.load_model(checkpoint_path)
    weights_list = last_model.get_weights()

    IMAGE_SHAPE = (144, 256, 3)
    inputs = keras.Input(shape=IMAGE_SHAPE)

    rescaling_layer = keras.layers.experimental.preprocessing.Rescaling(1. / 255)
    # normalization layer unssupported by version of tensorflow on drone. Data instead normalized in callback
    mean = [0.41718618, 0.48529191, 0.38133072]
    variance = [.057, .05, .061]
    # normalization_layer = keras.layers.experimental.preprocessing.Normalization(mean=[0.41718618, 0.48529191, 0.38133072],
    #                                                                             variance=[.057, .05, .061])
    x = rescaling_layer(inputs)
    # x = normalization_layer(x)

    x = keras.layers.Conv2D(filters=16, kernel_size=(5, 5), strides=(3, 3), activation='relu')(x)
    x = keras.layers.Conv2D(filters=32, kernel_size=(3, 3), strides=(2, 2), activation='relu')(x)
    x = keras.layers.Conv2D(filters=64, kernel_size=(3, 3), strides=(2, 2), activation='relu')(x)
    x = keras.layers.Conv2D(filters=8, kernel_size=(3, 3), strides=(2, 2), activation='relu')(x)
    # fully connected layers
    x = keras.layers.Flatten()(x)
    x = keras.layers.Dense(units=128, activation='linear')(x)
    DROPOUT = 0.0
    pre_recurrent_layer = keras.layers.Dropout(rate=DROPOUT)(x)

    if model_name == 'ncp':
        wiring = kncp.wirings.NCP(
            inter_neurons=18,  # Number of inter neurons
            command_neurons=12,  # Number of command neurons
            motor_neurons=4,  # Number of motor neurons
            sensory_fanout=6,  # How many outgoing synapses has each sensory neuron
            inter_fanout=4,  # How many outgoing synapses has each inter neuron
            recurrent_command_synapses=4,  # Now many recurrent synapses are in the
            # command neuron layer
            motor_fanin=6,  # How many incoming synapses has each motor neuron
        )
        rnn_cell = LTCCell(wiring)
        inputs_state = tf.keras.Input(shape=(rnn_cell.state_size,))

        motor_out, output_states = rnn_cell(pre_recurrent_layer, inputs_state)
        single_step_model = tf.keras.Model([inputs, inputs_state], [motor_out, output_states])

        # single_step_model.load_weights(checkpoint)
        single_step_model.set_weights(weights_list[3:])
        return single_step_model, rnn_cell
    else:
        raise ValueError(f"Illegal model name {model_name}")


class RNNControlNode:
    def __init__(self, path: str, log_data: bool):
        # base path to store all files
        self.path = path

        # state vars
        self.video_open = False
        self.close_video = True
        self.logger = Logger() if log_data else None
        self.path_appendix = None
        print(f"Logging for this run: {log_data}")

        # TODO: load model name and checkpoint from rosparam
        model_name = 'ncp'
        checkpoint_name = 'rev-0_model-ncp_seq-64_opt-adam_lr-0.000900_crop-0.000000_epoch-020_val_loss:0.2127_mse:0.1679_2021:09:20:02:24:31'
        self.single_step_model, rnn_cell = load_model(model_name, checkpoint_name)
        print('Loaded Model')

        self.hidden_state = tf.zeros((1, rnn_cell.state_size))

        self.velocity_service = rospy.ServiceProxy('/flight_task_control', dji_srv.FlightTaskControl)
        rospy.Subscriber('dji_osdk_ros/main_camera_images', Image, self.image_cb)
        rospy.init_node("rnn_control_node")
        print("Finished initialization of model and ros setup")
        rospy.spin()

    def image_cb(self, msg: Image):
        im_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        # resize image
        im = PIL.Image.fromarray(im_np)
        im_smaller = im.resize((256, 144), resample=PIL.Image.BILINEAR)
        im_expanded = np.expand_dims(im_smaller, 0)

        if not self.video_open and not self.close_video:
            rostime = msg.header.stamp  # rospy.Time.now()
            time = rostime.secs + rostime.nsecs * 1e-9

            # start generating csv
            self.logger.open_writer(os.path.join(self.path, "%.2f.csv" % time))

            # make a directory to store pngs
            self.path_appendix = '%f' % time
            os.mkdir(os.path.join(self.path, self.path_appendix))
            self.video_open = True
        elif self.video_open and self.close_video:
            print("ending recording")
            # dont need to do anything in png mode since the files have already been written
            # properly close csv writer
            self.logger.close_writer()
            self.video_open = False

        if self.video_open:
            rostime = msg.header.stamp  # rospy.Time.now()

            time = rostime.secs + rostime.nsecs * 1e-9

            cv2.imwrite(os.path.join(self.path, self.path_appendix, ('%.3f' % time) + ".png"), im_smaller)

            # add newest state for this frame to the csv
            self.logger.write_state(rostime)

        # run inference on im_expanded
        vel_cmd, self.hidden_state = self.single_step_model([im_expanded, self.hidden_state])

        # construct dji velocity command
        req = dji_msg_from_velocity(vel_cmd)

        self.velocity_service.call(req)


if __name__ == "__main__":
    path_param = rospy.get_param("~path", default="/home/dji/flash/")
    log_daa = rospy.get_param("~log_data", default=True)
    node = RNNControlNode(path_param, log_daa)
