#!/usr/bin/env python3
import os
from typing import Optional, List

import PIL.Image
import numpy as np
# import cv2
import rospy
# from cv_bridge import CvBridge
from sensor_msgs.msg import Image


# def load_next_image(bridge: CvBridge, im_list: List[str], last_idx: Optional[int]):
#    """
#    Retrieves and transforms the next image in the video sequence
#    :param bridge: CvBridge object for getting ROS messages
#    :param im_list: List of all files in the parent directory
#    :param last_idx index of last file returned using this function in imlist
#    :return: Returns a rospy sensor_msgs corresponding to the next image
#    """
#    cur_idx = last_idx if last_idx is not None else -1
#    img = None
#    done = False
#    # load image from file
#    while img is None:
#        cur_idx += 1
#        try:
#            # will return none if path is not valid index
#            img = cv2.imread(im_list[cur_idx])
#        except IndexError:
#            done = True
#            break
#
#    if img is not None:
#        # upscale image to match original DJI resolution
#        img = cv2.resize(img, dsize=(640*2, 360*2))
#        # convert image to ros msg
#        img = bridge.cv2_to_imgmsg(img, "rgb8")
#    return img, cur_idx, done

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
    return len(im_list)/ (last_time - first_time)


def publish_camera_messages():
    pub = rospy.Publisher("dji_osdk_ros/main_camera_images", Image, queue_size=2)
    rospy.init_node("camera_emulator")

    # setup node state
    default_directory = "/home/dji/data/1628628264.261048"
    directory = rospy.get_param("~image_directory", default=default_directory)
    contents = os.listdir(directory)
    contents = [os.path.join(directory, c) for c in contents if 'png' in c]
    contents.sort()
    last_idx = None
    # bridge = CvBridge()
    use_data_rate = rospy.get_param("~use_data_rate", default=False)
    if use_data_rate:
        rate = rospy.Rate(get_avg_rate(im_list=contents))
    else:
        rate = rospy.Rate(30)

    rospy.loginfo("Starting publishing of fake camera images")
    while not rospy.is_shutdown():
        # img, last_idx, done = load_next_image(bridge, contents, last_idx)
        img, last_idx, done = load_next_image(contents, last_idx)
        if not done:
            print('Published index %d' % last_idx)
            img_data = np.array(img, dtype=np.uint8).ravel().tobytes()
            img_msg = Image()
            img_msg.data = img_data
            img_msg.width = 1280
            img_msg.height = 720
            pub.publish(img_msg)
        else:
            break
        rate.sleep()

    print("Finished publishing fake camera images")


if __name__ == "__main__":
    publish_camera_messages()
