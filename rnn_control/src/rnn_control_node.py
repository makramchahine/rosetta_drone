#!/usr/bin/env python3
import json
import os
import sys
from pathlib import Path
from typing import Union, Optional

import PIL
import PIL.Image
import cv2
import dji_osdk_ros.srv as dji_srv
import numpy as np
import rospy
from sensor_msgs.msg import Image, Joy

# import logger and deepdrone from other ros package. Add to system path instead of including proper python lib dependency
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, "..", ".."))
from video_compression.scripts.logger import Logger

sys.path.append(os.path.join(SCRIPT_DIR, "..", "..", "drone_causality"))
from drone_causality.utils.model_utils import load_model_from_weights, NCPParams, LSTMParams, CTRNNParams, \
    generate_hidden_list, TCNParams, get_readable_name


def dji_msg_from_velocity(vel_cmd):
    joyact_req = dji_srv.JoystickActionRequest()
    joyact_req.joystickCommand.x = vel_cmd[0][0]
    joyact_req.joystickCommand.y = vel_cmd[0][1]
    joyact_req.joystickCommand.z = vel_cmd[0][2]
    joyact_req.joystickCommand.yaw = vel_cmd[0][3] * 180 / np.pi
    return joyact_req


class RNNControlNode:
    def __init__(self, params_path: str, checkpoint_path: str, log_path: Optional[str] = None, log_suffix: str = ""):
        rospy.init_node("rnn_control_node")
        # base path to store all files
        self.log_path = log_path
        if log_path is not None:
            # make path if not exists
            Path(log_path).mkdir(parents=True, exist_ok=True)
            print(f"Logging data for this run")

        # state vars
        self.video_open = False
        self.close_video = True
        self.logger = Logger()
        self.path_appendix = None
        # how many seconds to wait after a change in input to start accepting more commands
        # this is here as part of the Ramin-proofing state machine
        self.delay_secs = 5
        # the last time the flight mode has changed
        self.last_change_time = rospy.get_time() - self.delay_secs
        # the most recent flight mode
        self.last_input = 8000
        # the last message printed by the state machine - used to reduce spam
        self.last_message = "starting up"

        # normalization constants to replicate tf norm layers
        self.mean = [0.41718618, 0.48529191, 0.38133072]
        self.variance = np.array([.057, .05, .061])
        self.std_dev = np.sqrt(self.variance)

        # get model params and load model
        # make params path and checkpoint path relative
        params_path = os.path.join(SCRIPT_DIR, params_path)
        checkpoint_path = os.path.join(SCRIPT_DIR, checkpoint_path)
        self.checkpoint_path = checkpoint_path

        with open(params_path, "r") as f:
            data = json.loads(f.read())
            model_params: Union[NCPParams, LSTMParams, CTRNNParams, TCNParams] = eval(
                data[os.path.basename(checkpoint_path)])

        model_params.no_norm_layer = True
        model_params.single_step = True
        self.single_step_model = load_model_from_weights(model_params, checkpoint_path)
        self.hiddens = generate_hidden_list(model=self.single_step_model, return_numpy=True)
        print('Loaded Model')

        # print strs
        self.readable_model_name = get_readable_name(model_params)
        self.log_suffix = log_suffix

        # init ros
        self.velocity_service = rospy.ServiceProxy('/flight_task_control', dji_srv.FlightTaskControl)
        self.joystick_mode_service = rospy.ServiceProxy('set_joystick_mode', dji_srv.SetJoystickMode)
        self.joystick_action_service = rospy.ServiceProxy('joystick_action', dji_srv.JoystickAction)
        self.ca_service = rospy.ServiceProxy('obtain_release_control_authority', dji_srv.ObtainControlAuthority)

        rospy.Subscriber('dji_osdk_ros/main_camera_images', Image, self.image_cb, queue_size=1)
        rospy.Subscriber('dji_osdk_ros/rc', Joy, self.joy_cb, queue_size=1)
        print("Finished initialization of model and ros setup")
        rospy.spin()

    def image_cb(self, msg: Image):
        im_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        # resize image
        im = PIL.Image.fromarray(im_np)
        # shape: (h, w, c)
        # note PIL resize is width, height, whereas channel dim is height, widdth
        im_smaller = np.array(im.resize((256, 144), resample=PIL.Image.BILINEAR))
        im_network = im_smaller / 255
        im_network = im_network - self.mean
        im_network = im_network / self.std_dev  # norm layer ordinarily divdes by sqrt of var
        # shape: (batch=1, h, w, c)
        im_network = np.expand_dims(im_network, 0)  # add batch dimension

        if not self.video_open and not self.close_video:
            # start generating csv
            if self.log_path is not None:
                rostime = msg.header.stamp  # rospy.Time.now()
                rtime = rostime.secs + rostime.nsecs * 1e-9
                self.path_appendix = f"{round(rtime, 2)}_{self.readable_model_name}{self.log_suffix}"
                self.logger.open_writer(os.path.join(self.log_path, f"{self.path_appendix}.csv"))
                Path(os.path.join(self.log_path, f"{rtime}_{self.readable_model_name}")).touch()
                # make a directory to store pngs
                image_dir = os.path.join(self.log_path, self.path_appendix)
                Path(image_dir).mkdir(parents=True, exist_ok=True)

            self.video_open = True
            print("starting recording")
        elif self.video_open and self.close_video:
            print("ending recording")
            if self.log_path is not None:
                # dont need to do anything in png mode since the files have already been written
                # properly close csv writer
                self.logger.close_writer()
            self.video_open = False

        if self.video_open:  # run network on image and control drone
            # run inference on im_network
            out = self.single_step_model.predict([im_network, *self.hiddens])
            vel_cmd = out[0]
            self.hiddens = out[1:]  # list num_hidden long, each el is batch x hidden_dim

            # need to periodically reobtain control authority and set control mode
            ca_res = self.ca_service.call(self.get_ca_req())
            res1 = self.joystick_mode_service.call(self.get_joymode_req())

            self.logger.vel_cmd = vel_cmd[0]  # strip batch dim
            # construct dji velocity command
            req = dji_msg_from_velocity(vel_cmd)
            res2 = self.joystick_action_service.call(req)

            if self.log_path is not None:
                rostime = msg.header.stamp  # rospy.Time.now()
                rtime = rostime.secs + rostime.nsecs * 1e-9
                cv2.imwrite(os.path.join(self.log_path, self.path_appendix, ('%.3f' % rtime) + ".png"), im_smaller)
                # add newest state for this frame to the csv
                self.logger.write_state(rostime)

    # T -8000 (left)
    # P 8000  (center)
    # S 0     (right)
    def joy_cb(self, msg):
        """
        Processes the joystick message to detect mode changes to decide what to do with recording

        Moving the mode switch right turns on recording, and moving it to the left turns off recording
        There is also a delay that starts counting any time the input changes
        This way, after a command is given the mode switch has to be in a steady state for some time before
        new commands can be given, making it ok to accidentally switch it around a bunch while trying to get the switch to the center

        """
        current_input = msg.axes[4]

        # whether or not we're waiting for a delay
        waiting = abs(rospy.get_time() - self.last_change_time) < self.delay_secs

        if current_input == -8000 and not self.close_video and not waiting:
            self.close_video = True
            message = "close video command sent"
        elif current_input == 0 and self.close_video and not waiting:
            self.close_video = False
            message = "start video command sent"
        elif not waiting:
            message = "accepting commands"
        else:
            message = "waiting"

        if current_input != self.last_input:
            self.last_change_time = rospy.get_time()
            message += "\nwaiting %.2f seconds before allowing next command" % self.delay_secs

        # only send changed messages to avoid spam
        if self.last_message != message:
            print(message)
            self.last_message = message

        self.last_input = current_input

    @staticmethod
    def get_joymode_req():
        """
        Convenience function that returns a rosservice request for the default joystick mode used to control the drone
        via the network
        """
        joymode_req = dji_srv.SetJoystickModeRequest()
        joymode_req.horizontal_mode = dji_srv.SetJoystickModeRequest.HORIZONTAL_VELOCITY
        joymode_req.vertical_mode = dji_srv.SetJoystickModeRequest.VERTICAL_VELOCITY
        joymode_req.yaw_mode = dji_srv.SetJoystickModeRequest.YAW_RATE
        joymode_req.horizontal_coordinate = dji_srv.SetJoystickModeRequest.HORIZONTAL_BODY
        joymode_req.stable_mode = dji_srv.SetJoystickModeRequest.STABLE_ENABLE
        return joymode_req

    @staticmethod
    def get_ca_req():
        """
        Convenience function that returns rosservice request to allow node to obtain control authority
        """
        ca_req = dji_srv.ObtainControlAuthorityRequest()
        ca_req.enable_obtain = True
        return ca_req


if __name__ == "__main__":
    log_path = rospy.get_param("log_path", default="~/flash")
    params_path_ros = rospy.get_param("params_path", default="models/online_1/params.json")
    checkpoint_path_ros = rospy.get_param("checkpoint_path")
    log_suffix_ros = rospy.get_param("log_suffix", default="")
    node = RNNControlNode(log_path=log_path, params_path=params_path_ros,
                          checkpoint_path=checkpoint_path_ros, log_suffix=log_suffix_ros)
