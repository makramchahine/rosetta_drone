#!/usr/bin/env python3
import json
import os
import sys
import time
from typing import Union

import PIL
import PIL.Image
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, "..", "..", "drone_causality"))
from drone_causality.utils.model_utils import NCPParams, LSTMParams, CTRNNParams, TCNParams
from drone_causality.visual_backprop import get_conv_head, visualbackprop_activations, convert_to_color_frame




class LiveSaliencyNode:
    def __init__(self, checkpoint_path: str, params_path: str):
        rospy.init_node("rnn_control_node")
        rospy.Subscriber('dji_osdk_ros/main_camera_images', Image, self.image_cb, queue_size=1)

        with open(params_path, "r") as f:
            data = json.loads(f.read())
            model_params: Union[NCPParams, LSTMParams, CTRNNParams, TCNParams] = eval(
                data[os.path.basename(checkpoint_path)])

        model_params.no_norm_layer = True
        model_params.single_step = True

        self.mean = [0.41718618, 0.48529191, 0.38133072]
        variance = np.array([.057, .05, .061])
        self.std_dev = np.sqrt(variance)
        rtime = time.time()
        self.folder_name = f"{round(rtime, 2)}"

        self.conv_head = get_conv_head(checkpoint_path, model_params)
        print(f"Finished model loading")
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

        activations = self.conv_head.predict(im_network)

        rostime = msg.header.stamp  # rospy.Time.now()
        rtime = rostime.secs + rostime.nsecs * 1e-9
        cv2.imwrite(os.path.join("~/dji/flash", self.folder_name, ('%.3f' % rtime) + ".png"), im_smaller)

        saliency = visualbackprop_activations(self.conv_head, activations)
        cv2.imshow("Frame", convert_to_color_frame(saliency))
        cv2.waitKey(1)


if __name__ == "__main__":
    base_dir = os.path.join(SCRIPT_DIR, "..", "..", "rnn_control", "src")
    checkpoint_path = "models/all_types_train/headless/rev-0_model-ncp_seq-64_opt-adam_lr-0.000291_crop-0.000000_epoch-099_val_loss:0.2465_mse:0.0590_2022:02:09:12:22:53.hdf5"
    params_path = "models/all_types_train/params.json"
    checkpoint_path = os.path.join(base_dir, checkpoint_path)
    params_path = os.path.join(base_dir, params_path)
    LiveSaliencyNode(checkpoint_path, params_path)
