# Created by Patrick Kao at 3/16/22
import json
import os
import sys
from typing import Tuple, Optional

import PIL
import PIL.Image
import dji_osdk_ros.srv as dji_srv
import numpy as np
import rospy
from numpy import ndarray
from sensor_msgs.msg import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, "..", ".."))
sys.path.append(os.path.join(SCRIPT_DIR, "..", "..", "drone_causality"))
from drone_causality.keras_models import IMAGE_SHAPE

# normalization constants
MEAN = [0.41718618, 0.48529191, 0.38133072]
VARIANCE = np.array([.057, .05, .061])


def obtain_control_authority(ca_service: rospy.ServiceProxy, joystick_mode_service: rospy.ServiceProxy, ):
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

def release_control_authority(ca_service: rospy.ServiceProxy, joystick_mode_service: rospy.ServiceProxy, ):
    ca_req = dji_srv.ObtainControlAuthorityRequest()
    ca_req.enable_obtain = False
    ca_res = ca_service.call(ca_req)

    joymode_req = dji_srv.SetJoystickModeRequest()
    joymode_req.horizontal_mode = dji_srv.SetJoystickModeRequest.HORIZONTAL_VELOCITY
    joymode_req.vertical_mode = dji_srv.SetJoystickModeRequest.VERTICAL_VELOCITY
    joymode_req.yaw_mode = dji_srv.SetJoystickModeRequest.YAW_RATE
    joymode_req.horizontal_coordinate = dji_srv.SetJoystickModeRequest.HORIZONTAL_BODY
    joymode_req.stable_mode = dji_srv.SetJoystickModeRequest.STABLE_ENABLE
    res1 = joystick_mode_service.call(joymode_req)

def send_vel_cmd(vel_cmd: ndarray, joystick_action_service: rospy.ServiceProxy):
    """
    Convenience script that invokes ros services to send vel_cmd to the drone
    :param vel_cmd: shape 1x4, with elements forward x left x up x counterclockwise that represents control signal
    :param ca_service: service proxy for obtain_release_control_authority
    :param joystick_mode_service: service proxy for set_joystick_mode
    :param joystick_action_service: service proxy for joystick_action
    :return: N/A
    """
    # construct dji velocity command
    # Note drone takes coordinates in forward x right x up x clockwise but we are given
    # forward x left x up x counterclockwise because this is what we use in training. Reverse yaw and roll
    joyact_req = dji_srv.JoystickActionRequest()
    joyact_req.joystickCommand.x = vel_cmd[0][0]
    joyact_req.joystickCommand.y = -vel_cmd[0][1]
    joyact_req.joystickCommand.z = vel_cmd[0][2]
    joyact_req.joystickCommand.yaw = -vel_cmd[0][3] * 180 / np.pi  # convert to degrees per sec from rad/sec
    res2 = joystick_action_service.call(joyact_req)


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


def process_image_network(msg: Image) -> Tuple[ndarray, ndarray]:
    """
    Converts ros Image sensor message to numpy array, and performs rescaling and normalization that would ordinarily
    be performed by tensorflow preprocessing layers but can't be used with this version of tf
    :param msg: Ros Image sensor message
    :return: Tuple of the numpy version of the processed image and also the rescaled and normalized version to be fed
    into the network
    """
    std_dev = np.sqrt(VARIANCE)

    im_smaller = process_image(msg)
    im_network = im_smaller / 255
    im_network = im_network - MEAN
    im_network = im_network / std_dev  # norm layer ordinarily divdes by sqrt of var
    # shape: (batch=1, h, w, c)
    im_network = np.expand_dims(im_network, 0)  # add batch dimension
    return im_smaller, im_network


def generate_dummy_image() -> ndarray:
    return np.random.rand(1, *IMAGE_SHAPE)

def saliency_center(img_out_saliency):
    # convert the grayscale image to binary image

    ret, thresh = cv2.threshold(img_out_saliency, 0, 255, 0)

    # calculate moments of binary image
    M = cv2.moments(thresh)

    # calculate x,y coordinate of center
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        # set values as what you need in the situation
        cX, cY = 0, 0

    return [cX, cY]

def find_checkpoint_path(params_path: str, checkpoint_path: Optional[str], model_name: Optional[str]) -> str:
    """
    Convenience function that if checkpoint_path is not passed, looks up model_name in the keys of the json at
    params_path to prevent the user from having to type the whole modeo name into the roslaunch
    :param params_path: path to params json relative to this script dir
    :param checkpoint_path: path to model checkpoint. If this is passed, function does not look for model_name
    :param model_name: name of model to look up
    :return: path relative to this dir of most likely model name
    """
    if checkpoint_path is not None:
        if model_name is not None:
            print(f"Checkpoint path explicitly passed. Not using model name {model_name}")
        return checkpoint_path
    else:
        assert model_name is not None, "Passed neither model name nor checkpoint path. Need at least 1"
        with open(os.path.join(SCRIPT_DIR, params_path), "r") as f:
            params_data = json.load(f)
        models = params_data.keys()
        for model in models:
            # case 1: not ctrnn, just look for model name
            # case 2: ctrnn type, look for whole string
            # case 3: not named by trainign script, allow custom name of whole model.hdf5
            if f"-{model_name}_" in model or f"ctrnn_{model_name}_" in model or model_name == f"{model_name}.hdf5":
                print(f"Using checkpoint path {model}")
                return os.path.join(os.path.dirname(params_path), "headless", model)
        raise ValueError(f"Could not find model {model_name} in {params_path}")