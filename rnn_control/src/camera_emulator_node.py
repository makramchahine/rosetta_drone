import os
from typing import Optional, List

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def load_next_image(bridge: CvBridge, im_list: List[str], last_idx: Optional[int]):
    """
    Retrieves and transforms the next image in the video sequence
    :param bridge: CvBridge object for getting ROS messages
    :param im_list: List of all files in the parent directory
    :param last_idx index of last file returned using this function in imlist
    :return: Returns a rospy sensor_msgs corresponding to the next image
    """
    cur_idx = last_idx if last_idx is not None else -1
    img = None
    done = False
    # load image from file
    while img is None:
        cur_idx += 1
        try:
            # will return none if path is not valid index
            img = cv2.imread(im_list[cur_idx])
        except IndexError:
            done = True
            break

    if img is not None:
        # upscale image to match original DJI resolution
        img = cv2.resize(img, dsize=(640*2, 360*2))
        # convert image to ros msg
        img = bridge.cv2_to_imgmsg(img, "rgb8")
    return img, cur_idx, done


def publish_camera_messages():
    pub = rospy.Publisher("dji_osdk_ros/main_camera_images", Image, queue_size=2)
    rospy.init_node("camera_emulator")
    # TODO: Infer rate from
    rate = rospy.Rate(30)

    # setup node state
    directory = "/media/dolphonie/Data/Files/UROP/devens_data/1635515333.207994"
    contents = os.listdir(directory)
    contents = [os.path.join(directory, c) for c in contents]
    contents.sort()
    last_idx = None
    bridge = CvBridge()

    rospy.loginfo("Starting publishing of fake camera images")
    while not rospy.is_shutdown():
        img, last_idx, done = load_next_image(bridge, contents, last_idx)
        if not done:
            pub.publish(img)
        else:
            break
        rate.sleep()

    print("Finished publishing fake camera images")

if __name__ == "__main__":
    publish_camera_messages()