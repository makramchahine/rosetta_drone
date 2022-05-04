#!/usr/bin/env python3
import os
import sys

import PIL
import PIL.Image
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

from rnn_control.src.logger_node import Logger, LoggerState
from rnn_control.src.rnn_control_node import RNNControlNode, process_image_network

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, "../../live_saliency", ".."))
sys.path.append(os.path.join(SCRIPT_DIR, "../../live_saliency", "..", "drone_causality"))
from drone_causality.utils.model_utils import load_model_from_weights, \
    generate_hidden_list, get_params_from_json
from drone_causality.analysis.visual_backprop import get_conv_head, visualbackprop_activations, convert_to_color_frame


class LiveSaliencyNode:
    def __init__(self, params_path: str, checkpoint_path: str, log_path : str):
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

        self.logger = Logger(log_path=log_path, log_suffix="_saliency")
        self.logger.state = LoggerState.RECORDING

        rospy.Subscriber('dji_osdk_ros/main_camera_images', Image, self.image_cb, queue_size=1)
        rospy.spin()

    def image_cb(self, msg: Image):
        im_smaller, im_network = process_image_network(msg)

        activations = self.conv_head.predict(im_network)
        saliency = visualbackprop_activations(self.conv_head, activations)

        out = self.single_step_model.predict([im_network, *self.hiddens])
        vel_cmd = out[0]
        self.hiddens = out[1:]  # list num_hidden long, each el is batch x hidden_dim

        # vel command is v_x, v_y, v_z, w
        print(f"Velocity command: {vel_cmd}")
        saliency = convert_to_color_frame(saliency)
        saliency = cv2.resize(saliency, (256 * 3, 144 * 3))

        cv2.imshow("Frame", saliency)
        self.logger.log(image=im_smaller, vel_cmd=vel_cmd, rtime=msg.header.stamp.to_sec())
        cv2.waitKey(1)


if __name__ == "__main__":
    base_dir = os.path.join(SCRIPT_DIR, "../../live_saliency", "..", "rnn_control", "src")
    checkpoint_path = "models/all_types_train/headless/rev-0_model-ncp_seq-64_opt-adam_lr-0.000291_crop-0.000000_epoch-099_val_loss:0.2465_mse:0.0590_2022:02:09:12:22:53.hdf5"
    params_path = "models/all_types_train/params.json"
    checkpoint_path = os.path.join(base_dir, checkpoint_path)
    params_path = os.path.join(base_dir, params_path)
    LiveSaliencyNode(checkpoint_path, params_path, "/home/dji/flash")
