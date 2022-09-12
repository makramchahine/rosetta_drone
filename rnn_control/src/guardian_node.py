#!/usr/bin/env python3
import os
import sys
import threading
from typing import Optional, Tuple

# noinspection PyUnresolvedReferences
import cv2  # need to import cv2 before tensorflow on drone or else crashes
import dji_osdk_ros.srv as dji_srv
import numpy as np
import rospy
from sensor_msgs.msg import Image

from control_utils import process_image_network, obtain_control_authority, release_control_authority, send_vel_cmd, \
    find_checkpoint_path, \
    generate_dummy_image, saliency_center
from logger_node import Logger

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, "..", ".."))
sys.path.append(os.path.join(SCRIPT_DIR, "..", "..", "drone_causality"))
from drone_causality.utils.model_utils import load_model_from_weights, generate_hidden_list, get_readable_name, \
    get_params_from_json
from drone_causality.keras_models import IMAGE_SHAPE
from drone_causality.analysis.visual_backprop import get_conv_head, compute_visualbackprop
from drone_causality.analysis.vis_utils import convert_to_color_frame

MIN_AREA = 50  # to mask candidate contours that have area less than this
UP_VEL = 0.1 # up speed
TARGET_AREA = 1200  # based on chair size in snowy run
CONTROL_AUTHORITY_TIME = 3

def poly_area(contour) -> float:
    """
    Area of polygon using shoelace formula

    :param contour: ndarray of shape num_points x 1 x 2
    :return: area of polygon
    """
    x, y = np.squeeze(contour, axis=1).T
    return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

class RNNControlNode:
    def __init__(self, params_path: str, checkpoint_path: str, params_path_bis: str, checkpoint_path_bis: str,
                 log_path: str, log_suffix: str = "",
                 pitch_only: bool = False, yaw_multiplier: float = 1.0):
        rospy.init_node("rnn_control_node")

        self.guard = False
        self.threshold = 9999999999
        self.t0 = -9999999
        self.condi = False
        self.guardian_time = 4
        self.detected = False
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
        # run dummy input through network to finish loading
        dummy_image = generate_dummy_image()
        o1 = self.single_step_model.predict([dummy_image, *self.hiddens])

        # get BIS model params and load model
        # make params path and checkpoint path relative
        params_path_bis = os.path.join(SCRIPT_DIR, params_path_bis)
        checkpoint_path_bis = os.path.join(SCRIPT_DIR, checkpoint_path_bis)
        self.checkpoint_path_bis = checkpoint_path_bis
        model_params_bis = get_params_from_json(params_path_bis, checkpoint_path_bis)
        model_params_bis.no_norm_layer = True
        model_params_bis.single_step = True
        self.single_step_model_bis = load_model_from_weights(model_params_bis, checkpoint_path_bis)
        self.hiddens_bis = generate_hidden_list(model=self.single_step_model_bis, return_numpy=True)
        self.conv_head_bis = get_conv_head(checkpoint_path_bis, model_params_bis)
        # run dummy input through network to finish loading
        o2 = self.single_step_model_bis.predict([dummy_image, *self.hiddens_bis])
        print('Loaded both models')

        # print strs
        readable_model_name = get_readable_name(model_params)
        self.logger = Logger(log_path=log_path, log_suffix=f"_{readable_model_name}{log_suffix}")

        self.pitch_only = pitch_only
        if pitch_only:
            print("Only sending pitch commands")

        self.yaw_multiplier = yaw_multiplier
        if yaw_multiplier != 1.0:
            print(f"Using yaw multiplier of {yaw_multiplier}")

        # init ros
        self.velocity_service = rospy.ServiceProxy('/flight_task_control', dji_srv.FlightTaskControl)
        self.joystick_mode_service = rospy.ServiceProxy('set_joystick_mode', dji_srv.SetJoystickMode)
        self.joystick_action_service = rospy.ServiceProxy('joystick_action', dji_srv.JoystickAction)
        self.ca_service = rospy.ServiceProxy('obtain_release_control_authority', dji_srv.ObtainControlAuthority)

        rospy.Subscriber('dji_osdk_ros/main_camera_images', Image, self._image_cb, queue_size=1, buff_size=2 ** 22)

        # run image processing in separate thread
        # state var updated by img_cb and read while sending control commands. Assume read/writes to this var are atomic
        self.image_msg: Optional[Image] = None
        thread = threading.Thread(target=self._send_control_commands, name=None, args=())
        thread.daemon = True  # allow entire process to be ctrl-c ed to stop
        thread.start()
        print("Finished initialization of model and ros setup")
        rospy.spin()

    def _send_control_commands(self):
        """
        Main logic loop that handles processing incoming images, running inference, and sending a control output
        """
        while True:
            if self.image_msg is None:
                # wait for 1st msg
                continue

            latest_msg = self.image_msg  # get master copy of msg for this iteration
            im_smaller, im_network = process_image_network(latest_msg)

            if self.logger.is_recording:  # run network on image and control drone
                # Compute the guardian control
                # run inference on im_network
                out = self.single_step_model.predict([im_network, *self.hiddens])

                vel_cmd = out[0]  # shape: 1 x 4
                self.hiddens = out[1:]  # list num_hidden long, each el is batch x hidden_dim

                # mutate vel_cmd according to options
                if self.pitch_only:
                    vel_cmd[0, 1:] = 0

                vel_cmd[0, 3] = vel_cmd[0, 3] * self.yaw_multiplier

                if not self.detected:
                    if not self.guard:
                        # GET 2 SALIENCY MAPS
                        saliency = compute_visualbackprop(im_network, self.conv_head)
                        saliency_bis = compute_visualbackprop(im_network, self.conv_head_bis)
                        saliency = saliency.numpy()
                        saliency_bis = saliency_bis.numpy()
                        ret, saliency = cv2.threshold(saliency, 50, 255, cv2.THRESH_BINARY)
                        ret_bis, saliency_bis = cv2.threshold(saliency_bis, 50, 255, cv2.THRESH_BINARY)

                        obj = self.best_object(thresh=saliency, im_smaller=im_smaller)
                        if obj is not None:
                            centroid, area = obj
                            print(centroid)
                            # Decide whether target has been attended to by looking at saliencey area
                            # if empty direction list, we are attending to the ultimate target
                            if area > TARGET_AREA:
                                self.detected = True

                        # sal = convert_to_color_frame(saliency)
                        # sal = cv2.resize(sal, (256 * 3, 144 * 3))
                        # sal_bis = convert_to_color_frame(saliency_bis)
                        # sal_bis = cv2.resize(sal_bis, (256 * 3, 144 * 3))

                        # cv2.imshow("GUARDIAN", sal)
                        # cv2.imshow("HUUGHMAN", sal_bis)

                        # COMPARE SALIENCY CENTERS
                        c = np.array(saliency_center(saliency))
                        c_bis = np.array(saliency_center(saliency_bis))

                        d = np.linalg.norm(c - c_bis)
                        self.condi = d > self.threshold
                    else:
                        self.condi = False

                    if self.condi:
                        if not self.guard:
                            # strip batch dim for logger, shape before: 1 x 4, after 4
                            obtain_control_authority(ca_service=self.ca_service,
                                                     joystick_mode_service=self.joystick_mode_service, )
                            self.guard = True
                            self.t0 = rospy.Time.now().to_sec()

                        send_vel_cmd(vel_cmd=vel_cmd,
                                     joystick_action_service=self.joystick_action_service)

                    else:
                        t = rospy.Time.now().to_sec()
                        if (t - self.t0) > self.guardian_time:
                            if self.guard:
                                release_control_authority(ca_service=self.ca_service,
                                                          joystick_mode_service=self.joystick_mode_service, )
                                self.guard = False

                            vel_cmd[0, 0] = 0.0
                            vel_cmd[0, 1] = 0.0
                            vel_cmd[0, 2] = 0.0
                            vel_cmd[0, 3] = 0.0
                        else:
                            send_vel_cmd(vel_cmd=vel_cmd,
                                         joystick_action_service=self.joystick_action_service)

                else:
                    obtain_control_authority(ca_service=self.ca_service,
                                             joystick_mode_service=self.joystick_mode_service, )
                    vel_cmd[0, 0] = 0.0
                    vel_cmd[0, 1] = 0.0
                    vel_cmd[0, 2] = UP_VEL
                    vel_cmd[0, 3] = 0.0
                    send_vel_cmd(vel_cmd=vel_cmd,
                             joystick_action_service=self.joystick_action_service)



                print(self.guard, f"Distance between saliency centers: {d}", vel_cmd)
                self.logger.log(image=im_smaller, vel_cmd=vel_cmd, rtime=latest_msg.header.stamp.to_sec())
                # cv2.imshow("GUARDIAN", sal)
                # cv2.imshow("HUUGHMAN", sal_bis)

                # cv2.waitKey(1)

    def _image_cb(self, msg: Image):
        """
        Instead of having the processing code in image_cb, have it in another thread or else large delay is introduced
        into image processing because of having to wait for the delay in image_cb in the queue and in the publishing
        :param msg: ros img message to write
        :return:
        """
        self.image_msg = msg

    def best_object(self, thresh: np.ndarray, im_smaller: Optional[np.ndarray] = None) -> Optional[Tuple[np.ndarray, float]]:
        """
        Func that finds the contour in the blurred saliency map that has the highest average pixel value and returns
        its centroid and area

        :param thresh: shape height x width x channels. Normalized color image taken from drone for network
        consumption
        :param im_smaller: If self.display_contour is True, this image will be used for visualization. This image does
        not need to be provided if the debug display is not enabled and has no effect on control output
        :return: centroid (ndarray of 2 els (x, y) where x is vertical and y is horizontal) and area, float representing
        area of polygon
        """
        # # use for normalize, not color convert
        # saliency = convert_to_color_frame(compute_visualbackprop(img=im_network, activation_model=self.conv_head))
        #
        # # find contours in saliency map
        # saliency_gray = cv2.cvtColor(saliency, cv2.COLOR_BGR2GRAY)  # shape: h x w
        # blurred = cv2.blur(saliency_gray, (10, 10))
        # ret, thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # calculate contour stats
        num_contour_points = []
        average_contour_values = []
        contour_centroids = []
        contour_areas = []
        for i, contour in enumerate(contours):
            area = poly_area(contours[i])
            if area > MIN_AREA:
                # calculate average pixel value by multiplying against contour mask
                mask = cv2.drawContours(np.zeros_like(saliency_gray, dtype=np.uint8), contours, i,
                                        (1, 1, 1),
                                        thickness=cv2.FILLED)
                point_count = np.sum(mask)

                num_contour_points.append(point_count)
                total_brightness = np.sum(mask * saliency_gray)
                average_contour_values.append(total_brightness / point_count)
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

            return centroid, area



if __name__ == "__main__":
    log_path_ros = rospy.get_param("log_path", default="~/flash")
    params_path_ros = rospy.get_param("params_path")
    params_path_rosbis = rospy.get_param("params_path_bis")
    checkpoint_path_ros = None
    checkpoint_path_rosbis = None
    model_name_ros = rospy.get_param("model_name", default=None)
    model_name_rosbis = rospy.get_param("model_name_bis", default=None)
    log_suffix_ros = rospy.get_param("log_suffix", default="")
    pitch_only_ros = rospy.get_param("pitch_only", default=False)
    yaw_multiplier_ros = rospy.get_param("yaw_multiplier", default=1.0)
    checkpoint_path_ros = find_checkpoint_path(params_path=params_path_ros, checkpoint_path=checkpoint_path_ros,
                                               model_name=model_name_ros)
    checkpoint_path_rosbis = find_checkpoint_path(params_path=params_path_rosbis,
                                                  checkpoint_path=checkpoint_path_rosbis,
                                                  model_name=model_name_rosbis)
    if log_suffix_ros == "":
        log_suffix_ros = os.path.splitext(os.path.basename(params_path_ros))[0]

    node = RNNControlNode(log_path=log_path_ros, params_path=params_path_ros,
                          checkpoint_path=checkpoint_path_ros,
                          params_path_bis=params_path_rosbis,
                          checkpoint_path_bis=checkpoint_path_rosbis,
                          log_suffix=log_suffix_ros,
                          pitch_only=pitch_only_ros,
                          yaw_multiplier=yaw_multiplier_ros)