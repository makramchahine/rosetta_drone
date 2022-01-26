#!/usr/bin/env python3
import json
import os
import sys
from pathlib import Path
from typing import Union

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
from video_compression.scripts.logger_example import Logger
sys.path.append(os.path.join(SCRIPT_DIR, "..", "..", "deepdrone"))
from deepdrone.keras_models import load_model_from_weights, NCPParams, LSTMParams, CTRNNParams


def dji_msg_from_velocity(vel_cmd):
    joyact_req = dji_srv.JoystickActionRequest()
    joyact_req.joystickCommand.x = vel_cmd[0][0]
    joyact_req.joystickCommand.y = vel_cmd[0][1]
    joyact_req.joystickCommand.z = vel_cmd[0][2]
    joyact_req.joystickCommand.yaw = vel_cmd[0][3] * 180 / np.pi
    return joyact_req


class RNNControlNode:
    def __init__(self, path: str, log_data: bool, params_path: str, checkpoint_path: str):
        rospy.init_node("rnn_control_node")
        # base path to store all files
        # make path if not exists
        Path(path).mkdir(parents=True, exist_ok=True)
        self.path = path

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

        self.log_data = log_data
        print(f"Logging for this run: {log_data}")

        self.mean = [0.41718618, 0.48529191, 0.38133072]
        self.variance = [.057, .05, .061]

        # get model params and load model
        # make params path relative
        params_path = os.path.join(SCRIPT_DIR, params_path)
        with open(params_path, "r") as f:
            data = json.loads(f.read())
            model_params: Union[NCPParams, LSTMParams, CTRNNParams] = eval(data[os.path.basename(checkpoint_path)])

        model_params.no_norm_layer = True
        self.single_step_model = load_model_from_weights(model_params, checkpoint_path, single_step=True)
        self.hiddens = [np.zeros((1, input_shape[1])) for input_shape in self.single_step_model.input_shape[1:]]
        print('Loaded Model')

        # init ros
        self.velocity_service = rospy.ServiceProxy('/flight_task_control', dji_srv.FlightTaskControl)
        self.joystick_mode_client = rospy.ServiceProxy('set_joystick_mode', dji_srv.SetJoystickMode)
        self.joystick_action_client = rospy.ServiceProxy('joystick_action', dji_srv.JoystickAction)
        self.ca_client = rospy.ServiceProxy('obtain_release_control_authority', dji_srv.ObtainControlAuthority)

        rospy.Subscriber('dji_osdk_ros/main_camera_images', Image, self.image_cb)
        rospy.Subscriber('dji_osdk_ros/rc', Joy, self.joy_cb)
        print("Finished initialization of model and ros setup")
        rospy.spin()

    def image_cb(self, msg: Image):
        im_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        # resize image
        im = PIL.Image.fromarray(im_np)
        im_smaller = np.array(im.resize((256, 144), resample=PIL.Image.BILINEAR))
        im_network = im_smaller / 255
        im_network = im_network - self.mean
        im_network = im_network / self.variance
        im_network = np.expand_dims(im_network, 0)
        im_expanded = np.expand_dims(im_network, 0)

        if not self.video_open and not self.close_video:
            # start generating csv
            if self.log_data:
                rostime = msg.header.stamp  # rospy.Time.now()
                rtime = rostime.secs + rostime.nsecs * 1e-9
                self.logger.open_writer(os.path.join(self.path, "%.2f.csv" % rtime))

                # make a directory to store pngs
                self.path_appendix = '%f' % rtime
                image_dir = os.path.join(self.path, self.path_appendix)
                Path(image_dir).mkdir(parents=True, exist_ok=True)

            self.video_open = True
        elif self.video_open and self.close_video:
            print("ending recording")
            if self.log_data:
                # dont need to do anything in png mode since the files have already been written
                # properly close csv writer
                self.logger.close_writer()
            self.video_open = False

        if self.video_open:
            if self.log_data:
                rostime = msg.header.stamp  # rospy.Time.now()
                rtime = rostime.secs + rostime.nsecs * 1e-9
                cv2.imwrite(os.path.join(self.path, self.path_appendix, ('%.3f' % rtime) + ".png"), im_smaller)
                # add newest state for this frame to the csv
                self.logger.write_state(rostime)

            # run inference on im_expanded
            out = self.single_step_model.predict([im_expanded, *self.hiddens])
            vel_cmd = out[0]
            self.hiddens = out[1:]  # list num_hidden long, each el is batch x hidden_dim
            print(vel_cmd.numpy())

            ca_req = dji_srv.ObtainControlAuthorityRequest()
            ca_req.enable_obtain = True
            ca_res = self.ca_client.call(ca_req)
            # print('\n\nca_res: ', ca_res, '\n\n')

            joymode_req = dji_srv.SetJoystickModeRequest()
            joymode_req.horizontal_mode = dji_srv.SetJoystickModeRequest.HORIZONTAL_VELOCITY
            joymode_req.vertical_mode = dji_srv.SetJoystickModeRequest.VERTICAL_VELOCITY
            joymode_req.yaw_mode = dji_srv.SetJoystickModeRequest.YAW_RATE
            joymode_req.horizontal_coordinate = dji_srv.SetJoystickModeRequest.HORIZONTAL_BODY
            joymode_req.stable_mode = dji_srv.SetJoystickModeRequest.STABLE_ENABLE
            res1 = self.joystick_mode_client.call(joymode_req)
            # print('joymode response: ', res1)

            # construct dji velocity command
            req = dji_msg_from_velocity(vel_cmd)
            res2 = self.joystick_action_client.call(req)
            # print('Joyact response: ', res2)
            # t0 = time.time()
            # while (time.time() - t0 < 1000):
            # t0 = time.time()
            # while (time.time() - t0 < 1000):
            #    res2 = self.joystick_action_client.call(joyact_req)
            #    print('Joyact response: ', res2)

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

        message = ""

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


if __name__ == "__main__":
    path_ros = rospy.get_param("path", default="~/flash")
    log_data_ros = rospy.get_param("log_data", default=False)
    params_path_ros = rospy.get_param("params_path", default="models/online_1/params.json")
    checkpoint_path_ros = rospy.get_param("checkpoint_path")
    node = RNNControlNode(path=path_ros, log_data=log_data_ros, params_path=params_path_ros, checkpoint_path=checkpoint_path_ros)
