#!/usr/bin/env python3
import os
import sys
import time

import PIL
import PIL.Image
import cv2
import dji_osdk_ros.srv as dji_srv
import kerasncp as kncp
import numpy as np
import rospy
import tensorflow as tf
from kerasncp.tf import LTCCell
from sensor_msgs.msg import Image, Joy
from tensorflow import keras

# import logger from other ros package. Add to system path instead of including proper python lib dependency
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(script_dir, "..", ".."))
from video_compression.scripts.logger_example import Logger



def dji_msg_from_velocity(vel_cmd):
    req = dji_srv.FlightTaskControlRequest()
    req.task = dji_srv.FlightTaskControlRequest.TASK_VELOCITY_AND_YAWRATE_CONTROL
    req.joystickCommand.x = vel_cmd[0][0]
    req.joystickCommand.y = vel_cmd[0][1]
    req.joystickCommand.z = vel_cmd[0][2]
    req.joystickCommand.yaw = vel_cmd[0][3]
    req.velocityControlTimeMs = 40
    return req


class RNNControlNode:
    def __init__(self, path: str, log_data: bool):
        rospy.init_node("rnn_control_node")
        # base path to store all files
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

        # TODO: load model name and checkpoint, mean, var from rosparam
        self.mean = [0.41718618, 0.48529191, 0.38133072]
        self.variance = [.057, .05, .061]
        model_name = 'ncp'
        checkpoint_name = 'rev-0_model-ncp_seq-64_opt-adam_lr-0.000900_crop-0.000000_epoch-020_val_loss:0.2127_mse:0.1679_2021:09:20:02:24:31'
        #checkpoint_name = 'rev-0_model-ncp_seq-64_opt-adam_lr-0.000800_crop-0.000000_epoch-049_val_loss:0.2528_mse:0.2151_2021:11:10:19:28:04.hdf5'

        # init ros
        self.velocity_service = rospy.ServiceProxy('/flight_task_control', dji_srv.FlightTaskControl)
        self.joystick_mode_client = rospy.ServiceProxy('set_joystick_mode', dji_srv.SetJoystickMode)
        self.joystick_action_client = rospy.ServiceProxy('joystick_action', dji_srv.JoystickAction)
        self.ca_client = rospy.ServiceProxy('obtain_release_control_authority', dji_srv.ObtainControlAuthority)

        ca_req = dji_srv.ObtainControlAuthorityRequest()
        ca_req.enable_obtain = True
        ca_res = self.ca_client.call(ca_req)
        print('\n\nca_res: ', ca_res, '\n\n')


        print("Finished initialization of model and ros setup")

        joymode_req = dji_srv.SetJoystickModeRequest()
        joymode_req.horizontal_mode = dji_srv.SetJoystickModeRequest.HORIZONTAL_VELOCITY
        joymode_req.vertical_mode = dji_srv.SetJoystickModeRequest.VERTICAL_VELOCITY
        joymode_req.yaw_mode = dji_srv.SetJoystickModeRequest.YAW_RATE
        joymode_req.horizontal_coordinate = dji_srv.SetJoystickModeRequest.HORIZONTAL_BODY
        joymode_req.stable_mode = dji_srv.SetJoystickModeRequest.STABLE_ENABLE
        res1 = self.joystick_mode_client.call(joymode_req) 
        print('joymode response: ', res1)


        joyact_req = dji_srv.JoystickActionRequest()
        joyact_req.joystickCommand.x = 1
        joyact_req.joystickCommand.y = 0
        joyact_req.joystickCommand.z = 0
        joyact_req.joystickCommand.yaw = .2

        t0 = time.time()
        while (time.time() - t0 < 2000):
            res2 = self.joystick_action_client.call(joyact_req)
            print('Joyact response: ', res2)

        rospy.spin()


if __name__ == "__main__":
    path_param = rospy.get_param("~path", default="/home/dji/flash/")
    log_data = rospy.get_param("~log_data", default=False)
    node = RNNControlNode(path_param, log_data)
