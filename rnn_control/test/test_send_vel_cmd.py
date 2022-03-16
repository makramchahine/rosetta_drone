#!/usr/bin/env python3
import os
import sys

import dji_osdk_ros.srv as dji_srv
import numpy as np
import rospy
from rospy.timer import TimerEvent

# import logger from other ros package. Add to system path instead of including proper python lib dependency
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, "..", ".."))
from rnn_control.src.rnn_control_node import RNNControlNode


class TestControlNode:
    def __init__(self):
        rospy.init_node("test_control_node")
        self.velocity_service = rospy.ServiceProxy('/flight_task_control', dji_srv.FlightTaskControl)
        self.joystick_mode_service = rospy.ServiceProxy('set_joystick_mode', dji_srv.SetJoystickMode)
        self.joystick_action_service = rospy.ServiceProxy('joystick_action', dji_srv.JoystickAction)
        self.ca_service = rospy.ServiceProxy('obtain_release_control_authority', dji_srv.ObtainControlAuthority)
        rospy.Timer(rospy.Duration(2), self.send_move_command)
        rospy.spin()

    def send_move_command(self, event: TimerEvent):
        vel_cmd = np.array([[0, 0, 0, 1]])
        RNNControlNode.send_vel_cmd(vel_cmd=vel_cmd, ca_service=self.ca_service,
                                    joystick_mode_service=self.joystick_mode_service,
                                    joystick_action_service=self.joystick_action_service)


if __name__ == "__main__":
    TestControlNode()
