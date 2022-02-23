import os
import sys
from enum import Enum
from pathlib import Path
from typing import Optional

import PIL
import PIL.Image
import cv2
import numpy as np
import rospy
from numpy import ndarray
from sensor_msgs.msg import Image, Joy

from rnn_control.src.csv_writer import CSVWriter

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, "..", "..", "drone_causality"))
from drone_causality.keras_models import IMAGE_SHAPE


class LoggerState(Enum):
    STOPPED = 0
    RECORDING = 1


class SwitchState(Enum):
    LEFT = 0
    CENTER = 1
    RIGHT = 2


DEBOUNCE_PERIOD = 2


def process_image(msg: Image) -> ndarray:
    """
    Converts rospy sensor message into numpy array without using cv_bridge (since this has dep issues in python3)
    :param msg: rospy message
    :return: numpy array of shape height x width x channels, where height=256 and width=144 as defined in
    keras_models.py
    """
    im_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    # resize image
    im = PIL.Image.fromarray(im_np)
    # shape: (h, w, c)
    height = IMAGE_SHAPE[0]
    width = IMAGE_SHAPE[1]
    # note PIL resize is width, height, whereas image ndarray shape is height, width
    im_smaller = np.array(im.resize((width, height), resample=PIL.Image.BILINEAR))
    return im_smaller


class Logger:
    """
    Utility class that contains logic to update logging state from flight control mode switch and manage file creation/
    deletion. Intended to be an instance variable of any node that should log using the mode switch. An example
    instantiation is below in the class LoggerNode.

    To use, call log with the image and velocity command (optionally) that should be saved as a csv and image

    Not an independent node to avoid preprocessing the image twice, and not a mix-in to allow flexibility in the
    callbacks for image and joystick
    """

    def __init__(self, log_path: Optional[str] = None, log_suffix: str = ""):
        self._csv_writer = CSVWriter()
        self._log_path = log_path
        self.log_suffix = log_suffix
        if log_path is not None:
            # make path if not exists
            Path(log_path).mkdir(parents=True, exist_ok=True)
            print(f"Logging data for this run")
        self._current_logname = None
        self._last_transition_time = 0

        self.state = LoggerState.STOPPED
        rospy.Subscriber('dji_osdk_ros/rc', Joy, self._update_state, queue_size=1)

    def log(self, image: ndarray, vel_cmd: Optional[ndarray] = None, rtime: Optional[float] = None) -> None:
        """
        If the logger's current state is RECORDING, this function saves image to disk and writes a csv entry into the
        log. If the loggers state is STOPPED, nothing happens

        :param image: image to be saved. Should have shape height x width x channels and have dtype uint8
        :param vel_cmd: control input to drone of shape 1x4. If none, a velocity of 0 is written
        :param rtime: Timestamp to log in csv. If not provided, will be replaced with rostime now
        :return: nothing
        """
        if self.state == LoggerState.RECORDING:
            if rtime is None:
                rtime = rospy.Time.now().to_sec()

            # csv writer doesn't take vel cmd over topic to guarantee that it is set before entry is saved
            if vel_cmd is not None:
                self._csv_writer.vel_cmd = vel_cmd[0]

            cv2.imwrite(os.path.join(self._log_path, self._current_logname, ('%.3f' % rtime) + ".png"), image)

    def _update_state(self, msg: Joy) -> None:
        current_input = msg.axes[4]
        if current_input == -8000:
            switch_state = SwitchState.LEFT
        elif current_input == 8000:
            switch_state = SwitchState.CENTER
        elif current_input == 0:
            switch_state = SwitchState.RIGHT
        else:
            raise ValueError(f"Got illegal switch state {current_input}")

        time: float = msg.header.stamp.to_sec()
        if time - self._last_transition_time < DEBOUNCE_PERIOD:
            # don't allow transitions faster than DEBOUNCE_PERIOD
            return

        if self.state == LoggerState.STOPPED:
            if switch_state == SwitchState.RIGHT:
                # start recording
                self._current_logname = f"{round(time, 2)}{self.log_suffix}"
                self._csv_writer.open_writer(os.path.join(self._log_path, f"{self._current_logname}.csv"))
                # make a directory to store pngs
                image_dir = os.path.join(self._log_path, self._current_logname)
                Path(image_dir).mkdir(parents=True, exist_ok=True)
                print("Started recording")
        elif self.state == LoggerState.RECORDING:
            if switch_state == SwitchState.LEFT:
                self._csv_writer.close_writer()
                print("Stopped recording")
        else:
            raise ValueError(f"Got illegal state machine state {self.state}")

    @property
    def is_recording(self):
        return self.state == LoggerState.RECORDING


class LoggerNode:
    """
    Minimal example of a node that uses Logger to log video images and drone topics to csv according to joystick
    input
    """

    def __init__(self):
        rospy.init_node("logger_node")
        self.logger = Logger()
        rospy.Subscriber('dji_osdk_ros/main_camera_images', Image, self.image_cb, queue_size=1)
        rospy.spin()

    def image_cb(self, msg: Image):
        image = process_image(msg)
        self.logger.log(image=image, rtime=msg.header.stamp.to_sec())
