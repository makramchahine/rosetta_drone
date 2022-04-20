# Created by Patrick Kao at 3/15/22
import json
import os
import time
import unittest
from pathlib import Path
from threading import Timer

import rospy

from rnn_control_node import RNNControlNode

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

def stop_callback():
    rospy.signal_shutdown("Just stopping publishing...")

def test_all_models():
    # for now, assume camera emulator node running
    jsons = Path(os.path.join(SCRIPT_DIR, "..", "src", "models")).rglob("*.json")
    for model_json in jsons:
        abs_path = str(model_json.resolve())
        with open(abs_path, "r") as f:
            data = json.load(f)

        for model, params in data.items():
            model_dir = os.path.join(*model_json.parts[model_json.parts.index("models"):-1])
            checkpoint_path = os.path.join(model_dir, "headless", model)
            print(f"Testing {checkpoint_path}")
            stop_timer = Timer(5.0, stop_callback) # stop node after 5 seconds
            stop_timer.start()
            RNNControlNode(params_path=abs_path[abs_path.index("models"):],
                           checkpoint_path=checkpoint_path, log_path="~/log")

if __name__ == "__main__":
    test_all_models()