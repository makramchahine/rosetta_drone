#!/usr/bin/env python3
import os
import sys
from typing import Optional, Tuple

import cv2
import dji_osdk_ros.srv as dji_srv
import numpy as np
import rospy
from numpy import ndarray
from sensor_msgs.msg import Image
from simple_pid import PID

from logger_node import Logger
from rnn_control_node import RNNControlNode, process_image_network

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, "../../live_saliency", ".."))
sys.path.append(os.path.join(SCRIPT_DIR, "../../live_saliency", "..", "drone_causality"))
from drone_causality.utils.model_utils import load_model_from_weights, \
    generate_hidden_list, get_params_from_json
from drone_causality.visual_backprop import get_conv_head, visualbackprop_activations, convert_to_color_frame

MIN_AREA = 50
NOT_FOUND_TURN_RATE = 5
TARGET_AREA = 1000


def poly_area(contour) -> float:
    """
    Area of polygon using shoelace formula

    :param contour: ndarray of shape num_points x 1 x 2
    :return: area of polygon
    """
    x, y = np.squeeze(contour, axis=1).T
    return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


class SaliencyControlNode:
    def __init__(self, params_path: str, checkpoint_path: str, log_path: str, log_suffix: str = "",
                 display_contour: bool = False):
        rospy.init_node("live_saliency_node")
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

        self.conv_head = get_conv_head(checkpoint_path, model_params)
        print(f"Finished model loading")

        self.logger = Logger(log_path=log_path, log_suffix=f"_saliency{log_suffix}")
        self.roll_pid = PID()
        self.throttle_pid = PID()
        self.pitch_pid = PID()
        self.yaw_pid = PID()

        self.display_contour = display_contour

        self.velocity_service = rospy.ServiceProxy('/flight_task_control', dji_srv.FlightTaskControl)
        self.joystick_mode_service = rospy.ServiceProxy('set_joystick_mode', dji_srv.SetJoystickMode)
        self.joystick_action_service = rospy.ServiceProxy('joystick_action', dji_srv.JoystickAction)
        self.ca_service = rospy.ServiceProxy('obtain_release_control_authority', dji_srv.ObtainControlAuthority)

        rospy.Subscriber('dji_osdk_ros/main_camera_images', Image, self.image_cb, queue_size=1)
        rospy.spin()

    def image_cb(self, msg: Image):
        im_smaller, im_network = process_image_network(msg)
        obj = self.best_object(im_network=im_network, im_smaller=im_smaller)
        vel_cmd = np.array([[0, 0, 0, 0]])
        if obj is not None:
            centroid, area = obj
            img_center = np.array([el // 2 for el in im_smaller.shape[:2]])
            # command to send is (forward [pitch], right [roll], up [throttle], clockwise [yaw])
            roll_error = centroid[1] - img_center[1]
            vel_cmd[0, 1] = self.roll_pid(roll_error)
            throttle_error = -(centroid[0] - img_center[0])
            vel_cmd[0, 2] = self.throttle_pid(throttle_error)
            pitch_error = -(area - TARGET_AREA)
            vel_cmd[0, 0] = self.pitch_pid(pitch_error)
            # for now, set yaw as 0 always
        else:
            # spin drone to look for object
            vel_cmd[0, 3] = NOT_FOUND_TURN_RATE

        RNNControlNode.send_vel_cmd(vel_cmd=vel_cmd, ca_service=self.ca_service,
                                    joystick_mode_service=self.joystick_mode_service,
                                    joystick_action_service=self.joystick_action_service)
        self.logger.log(image=im_smaller, vel_cmd=vel_cmd, rtime=msg.header.stamp.to_sec())

    def best_object(self, im_network: ndarray, im_smaller: Optional[ndarray] = None) -> Optional[Tuple[ndarray, float]]:
        """

        :param im_network: shape height x width x channels. Normalized color image taken from drone for network consumption
        :return: centroid (ndarray of 2 els (x, y) where x is vertical and y is horizontal) and area, float representing
        area of polygon
        """
        # get saliency map
        activations = self.conv_head.predict(im_network)
        saliency = convert_to_color_frame(
            visualbackprop_activations(activations, self.conv_head))  # use for normalize, not color convert

        # find contours in saliency map
        gray = cv2.cvtColor(saliency, cv2.COLOR_BGR2GRAY)  # shape: h x w
        blurred = cv2.blur(gray, (10, 10))
        ret, thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # calculate contour stats
        num_contour_points = []
        average_contour_values = []
        contour_centroids = []
        contour_areas = []
        for i, contour in enumerate(contours):
            area = poly_area(contours[i])
            if area > MIN_AREA:
                mask = cv2.drawContours(np.squeeze(np.zeros_like(im_network, dtype=np.uint8), axis=0), contours, i, (255, 255, 255),
                                        thickness=cv2.FILLED)
                point_count = np.sum(mask) / 255

                num_contour_points.append(point_count)
                average_contour_values.append(np.sum(mask * im_network) / point_count)
                contour_centroids.append(np.mean(np.squeeze(contour, axis=1), axis=0))
                contour_areas.append(area)

        # return best contour
        if len(average_contour_values) == 0:
            # no contours found
            return None
        else:
            brightest = np.argmax(average_contour_values)
            centroid = contour_centroids[brightest]
            area = contour_areas[brightest]

            if self.display_contour:
                # show on display for debugging
                contour_im = im_smaller.copy().astype(np.uint8)
                contour_im = cv2.drawContours(contour_im, contours, brightest, (0, 255, 0), thickness=1)
                cv2.imshow("Contours", contour_im)
                cv2.waitKey(1)

            return centroid, area


if __name__ == "__main__":
    log_path = rospy.get_param("log_path", default="~/flash")
    params_path_ros = rospy.get_param("params_path", default="models/all_types_train/params.json")
    checkpoint_path_ros = rospy.get_param("checkpoint_path",
                                          default="models/all_types_train/headless/rev-0_model-ctrnn_ctt-bidirect_cn-1.000000_bba-silu_bb-dr-0.100000_fb-1.600000_bbu-128_bbl-1_wd-0.000001_seq-64_opt-adam_lr-0.000236_crop-0.000000_epoch-099_val-loss:0.2360_mse:0.0297_2022:02:05:02:17:14.hdf5")
    log_suffix_ros = rospy.get_param("log_suffix", default="")
    display_contour_ros = rospy.get_param("display_contour", default=False)
    node = SaliencyControlNode(log_path=log_path, params_path=params_path_ros,
                               checkpoint_path=checkpoint_path_ros, log_suffix=log_suffix_ros,
                               display_contour=display_contour_ros)
