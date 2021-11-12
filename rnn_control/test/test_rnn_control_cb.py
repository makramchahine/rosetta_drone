import os
import unittest

import PIL
import numpy as np
from sensor_msgs.msg import Image

from rnn_control.src.rnn_control_node import image_cb


class TestRNNCallback(unittest.TestCase):
    def get_script_dir(self):
        return os.path.dirname(os.path.abspath(__file__))

    def test_simple_data(self):
        script_dir = self.get_script_dir()
        img = PIL.Image.open(os.path.join(script_dir, "test.png"))

        img = img.resize((1280, 720), PIL.Image.BILINEAR)
        img_data = np.array(img, dtype=np.uint8).ravel().tobytes()
        img_msg = Image()
        img_msg.data = img_data
        img_msg.width = 1280
        img_msg.height = 720
        image_cb(img_msg)
