#!/usr/bin/env python3
import os
import sys
from typing import Tuple

import dji_osdk_ros.srv as dji_srv
import numpy as np
import rospy
from numpy import ndarray
from sensor_msgs.msg import Image

from logger_node import Logger, process_image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, "..", ".."))
sys.path.append(os.path.join(SCRIPT_DIR, "..", "..", "drone_causality"))
from drone_causality.utils.model_utils import load_model_from_weights, generate_hidden_list, get_readable_name, \
    get_params_from_json

# normalization constants
MEAN = [0.41718618, 0.48529191, 0.38133072]
VARIANCE = np.array([.057, .05, .061])


def process_image_network(msg: Image) -> Tuple[ndarray, ndarray]:
    std_dev = np.sqrt(VARIANCE)

    im_smaller = process_image(msg)
    im_network = im_smaller / 255
    im_network = im_network - MEAN
    im_network = im_network / std_dev  # norm layer ordinarily divdes by sqrt of var
    # shape: (batch=1, h, w, c)
    im_network = np.expand_dims(im_network, 0)  # add batch dimension
    return im_smaller, im_network


class RNNControlNode:
    def __init__(self, params_path: str, checkpoint_path: str, log_path: str, log_suffix: str = ""):
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
        print('Loaded Model')

        # print strs
        readable_model_name = get_readable_name(model_params)
        self.logger = Logger(log_path=log_path, log_suffix=f"{readable_model_name}{log_suffix}")

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

            # strip batch dim for logger, shape before: 1 x 4, after 4
            self.send_vel_cmd(vel_cmd=vel_cmd, ca_service=self.ca_service,
                              joystick_mode_service=self.joystick_mode_service,
                              joystick_action_service=self.joystick_action_service)

            self.logger.log(image=im_smaller, vel_cmd=vel_cmd, rtime=msg.header.stamp.to_sec())

    @staticmethod
    def send_vel_cmd(vel_cmd: ndarray, ca_service: rospy.ServiceProxy, joystick_mode_service: rospy.ServiceProxy,
                     joystick_action_service: rospy.ServiceProxy):
        ca_req = dji_srv.ObtainControlAuthorityRequest()
        ca_req.enable_obtain = True
        ca_res = ca_service.call(ca_req)

        joymode_req = dji_srv.SetJoystickModeRequest()
        joymode_req.horizontal_mode = dji_srv.SetJoystickModeRequest.HORIZONTAL_VELOCITY
        joymode_req.vertical_mode = dji_srv.SetJoystickModeRequest.VERTICAL_VELOCITY
        joymode_req.yaw_mode = dji_srv.SetJoystickModeRequest.YAW_RATE
        joymode_req.horizontal_coordinate = dji_srv.SetJoystickModeRequest.HORIZONTAL_BODY
        joymode_req.stable_mode = dji_srv.SetJoystickModeRequest.STABLE_ENABLE
        res1 = joystick_mode_service.call(joymode_req)

        # construct dji velocity command
        joyact_req = dji_srv.JoystickActionRequest()
        joyact_req.joystickCommand.x = vel_cmd[0][0]
        joyact_req.joystickCommand.y = vel_cmd[0][1]
        joyact_req.joystickCommand.z = vel_cmd[0][2]
        joyact_req.joystickCommand.yaw = vel_cmd[0][3] * 180 / np.pi
        res2 = joystick_action_service.call(joyact_req)


if __name__ == "__main__":
    log_path = rospy.get_param("log_path", default="~/flash")
    params_path_ros = rospy.get_param("params_path")
    checkpoint_path_ros = rospy.get_param("checkpoint_path")
    log_suffix_ros = rospy.get_param("log_suffix", default="")
    node = RNNControlNode(log_path=log_path, params_path=params_path_ros,
                          checkpoint_path=checkpoint_path_ros, log_suffix=log_suffix_ros)
