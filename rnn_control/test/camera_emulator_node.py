#!/usr/bin/env python3
import os
import sys
from typing import Optional, List

import PIL.Image
import dji_osdk_ros.srv as dji_srv
import numpy as np
import rospy
from dji_osdk_ros.srv import SetJoystickModeResponse, FlightTaskControlResponse, ObtainControlAuthorityResponse, \
    JoystickActionResponse
from rospy.timer import TimerEvent
from sensor_msgs.msg import Image, Joy

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, "..", "src"))
from logger_node import SwitchState


# TODO: make this a generator
def load_next_image(im_list: List[str], last_idx: Optional[int] = None):
    cur_idx = last_idx if last_idx is not None else -1
    img = None
    done = False
    while img is None:
        cur_idx += 1
        try:
            img = PIL.Image.open(im_list[cur_idx])
        except IndexError:
            done = True
            break

    if img is not None:
        img = img.resize((1280, 720), PIL.Image.BILINEAR)

    return img, cur_idx, done


def get_time_from_filename(name: str) -> float:
    extension_stripped = name.split("/")[-1].split(".")[:-1]
    return float(".".join(extension_stripped))


def get_avg_rate(im_list: List[str]):
    # assume im_list sorted
    _, first_i, _ = load_next_image(im_list)
    reversed_list = im_list[::-1]
    _, last_i, _ = load_next_image(reversed_list)

    # get last part of filename, and strip extension to get time in seconds
    first_time = get_time_from_filename(im_list[first_i])
    last_time = get_time_from_filename(reversed_list[last_i])
    return len(im_list) / (last_time - first_time)


def init_dummy_services():
    def echo_callback(req):
        rospy.loginfo(f"Received joystick message {req}")
        return JoystickActionResponse()

    rospy.Service('/flight_task_control', dji_srv.FlightTaskControl, lambda x: FlightTaskControlResponse())
    rospy.Service('set_joystick_mode', dji_srv.SetJoystickMode, lambda x: SetJoystickModeResponse())
    rospy.Service('joystick_action', dji_srv.JoystickAction, echo_callback)
    rospy.Service('obtain_release_control_authority', dji_srv.ObtainControlAuthority,
                  lambda x: ObtainControlAuthorityResponse())


def send_start_recording():
    def send_right(event: TimerEvent):
        msg = Joy(axes=[0.0, 0.0, 0.0, 0.0, float(SwitchState.RIGHT.value), 0])
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)

    pub = rospy.Publisher("dji_osdk_ros/rc", Joy, queue_size=1)
    rospy.Timer(rospy.Duration(5), send_right)


def publish_camera_messages(image_directory: str, use_data_rate: bool = False, loop_video: bool = False,
                            flip_channels: bool = False):
    pub = rospy.Publisher("dji_osdk_ros/main_camera_images", Image, queue_size=2)
    rospy.init_node("camera_emulator")

    # setup ros dummies
    init_dummy_services()
    send_start_recording()

    # init node state
    contents = os.listdir(image_directory)
    contents = [os.path.join(image_directory, c) for c in contents if 'png' in c]
    contents.sort()
    last_idx = None
    # bridge = CvBridge()
    if use_data_rate:
        rate = rospy.Rate(get_avg_rate(im_list=contents))
    else:
        rate = rospy.Rate(30)

    rospy.loginfo("Starting publishing of fake camera images")
    while not rospy.is_shutdown():
        # img, last_idx, done = load_next_image(bridge, contents, last_idx)
        img, last_idx, done = load_next_image(contents, last_idx)
        if not done:
            # print('Published index %d' % last_idx)
            img_numpy = np.array(img, dtype=np.uint8)
            if flip_channels:
                img_numpy = img_numpy[:, :, ::-1]
            img_data = img_numpy.ravel().tobytes()
            img_msg = Image()
            img_msg.data = img_data
            img_msg.width = 1280
            img_msg.height = 720
            pub.publish(img_msg)
        else:
            if loop_video:
                last_idx = None
                done = False
                continue
            else:
                break
        rate.sleep()

    print("Finished publishing fake camera images")


if __name__ == "__main__":
    publish_camera_messages(
        image_directory=rospy.get_param("image_directory"),
        use_data_rate=rospy.get_param("use_data_rate", default=False),
        loop_video=rospy.get_param("loop_video", default=False),
        flip_channels=rospy.get_param("flip_channels", default=False),
    )
