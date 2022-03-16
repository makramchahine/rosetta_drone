#!/usr/bin/env python3
import os
import sys

import dji_osdk_ros.srv as dji_srv
import numpy as np
import rospy
from sensor_msgs.msg import Image

from control_utils import process_image_network, obtain_control_authority, send_vel_cmd, find_checkpoint_path
from logger_node import Logger

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, "..", ".."))
sys.path.append(os.path.join(SCRIPT_DIR, "..", "..", "drone_causality"))
from drone_causality.utils.model_utils import load_model_from_weights, generate_hidden_list, get_readable_name, \
    get_params_from_json
from drone_causality.keras_models import IMAGE_SHAPE

CONTROL_AUTHORITY_TIME = 3


class RNNControlNode:
    def __init__(self, params_path: str, checkpoint_path: str, log_path: str, log_suffix: str = "",
                 pitch_only: bool = False):
        rospy.init_node("rnn_control_node")

        # get model params and load model
        # make params path and checkpoint path relative
        params_path = os.path.join(SCRIPT_DIR, params_path)
        checkpoint_path = os.path.join(SCRIPT_DIR, checkpoint_path)
        self.checkpoint_path = checkpoint_path
        model_params = get_params_from_json(params_path, checkpoint_path)
        model_params.no_norm_layer = True
        model_params.single_step = True
        self.single_step_model = load_model_from_weights(model_params, checkpoint_path)
        self.hiddens = generate_hidden_list(model=self.single_step_model, return_numpy=True)
        # run dummy input through network to finish loading
        dummy_image = np.random.rand(1, *IMAGE_SHAPE)
        self.single_step_model.predict([dummy_image, *self.hiddens])
        print('Loaded Model')

        # print strs
        readable_model_name = get_readable_name(model_params)
        self.logger = Logger(log_path=log_path, log_suffix=f"_{readable_model_name}{log_suffix}")

        self.pitch_only = pitch_only

        # init ros
        self.velocity_service = rospy.ServiceProxy('/flight_task_control', dji_srv.FlightTaskControl)
        self.joystick_mode_service = rospy.ServiceProxy('set_joystick_mode', dji_srv.SetJoystickMode)
        self.joystick_action_service = rospy.ServiceProxy('joystick_action', dji_srv.JoystickAction)
        self.ca_service = rospy.ServiceProxy('obtain_release_control_authority', dji_srv.ObtainControlAuthority)

        rospy.Subscriber('dji_osdk_ros/main_camera_images', Image, self._image_cb, queue_size=1)
        print("Finished initialization of model and ros setup")
        rospy.spin()

    def _image_cb(self, msg: Image):
        im_smaller, im_network = process_image_network(msg)

        if self.logger.is_recording:  # run network on image and control drone
            # run inference on im_network
            out = self.single_step_model.predict([im_network, *self.hiddens])
            vel_cmd = out[0]  # shape: 1 x 4
            self.hiddens = out[1:]  # list num_hidden long, each el is batch x hidden_dim

            if self.pitch_only:
                vel_cmd[0, 1:] = 0

            # strip batch dim for logger, shape before: 1 x 4, after 4
            if self.logger.time_since_transition() < CONTROL_AUTHORITY_TIME:
                # only ask for control authority a fixed time after transition
                obtain_control_authority(ca_service=self.ca_service,
                                              joystick_mode_service=self.joystick_mode_service, )
            send_vel_cmd(vel_cmd=vel_cmd,
                              joystick_action_service=self.joystick_action_service)

            self.logger.log(image=im_smaller, vel_cmd=vel_cmd, rtime=msg.header.stamp.to_sec())


if __name__ == "__main__":
    log_path = rospy.get_param("log_path", default="~/flash")
    params_path_ros = rospy.get_param("params_path")
    checkpoint_path_ros = rospy.get_param("checkpoint_path", default=None)
    model_name_ros = rospy.get_param("model_name", default=None)
    log_suffix_ros = rospy.get_param("log_suffix", default="")
    pitch_only_ros = rospy.get_param("pitch_only", default=False)
    checkpoint_path_ros = find_checkpoint_path(params_path=params_path_ros, checkpoint_path=checkpoint_path_ros,
                                               model_name=model_name_ros)
    if log_suffix_ros == "":
        log_suffix_ros = os.path.splitext(os.path.basename(params_path_ros))[0]

    node = RNNControlNode(log_path=log_path, params_path=params_path_ros,
                          checkpoint_path=checkpoint_path_ros, log_suffix=log_suffix_ros, pitch_only=pitch_only_ros)
