import numpy as np
import rospy
from sensor_msgs.msg import Image, Joy

import ffio


def image_cb(msg):
    global video_start_time
    global video_times
    if not video_open and not close_video:
        rostime = rospy.Time.now()
        time = rostime.secs + rostime.nsecs * 1e-9
        video_start_time = time
        video_times = []
        writer = ffio.FFWriter('%.2f.mp4' % time, crf=crf, preset=compression_preset)
        video_open = True
    elif video_open and close_video:
        writer.close()
        with open('frame_times_%.2f.txt', 'w') as fo:
            for t in video_times:
                fo.write('%f\n' % t)
        video_open = False
        return
    if video_open:
        rostime = rospy.Time.now()
        im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        writer.write(im)
        time = rostime.secs + rostime.nsecs * 1e-9
        times.append(time)

# T -8000 (left)
# P 8000  (center)
# S 0     (right)
def joy_cb(msg):
    if msg.axes[4] == -8000:
        close_video = True
    elif msg.axes[4] == 0:
        close_video = False

crf = 10
compression_preset = 'fast'

video_start_time = None
video_times = []
# crf=0 is lossless
# crf=51 is maximum compression
times = []
video_open = False
close_video = True
rospy.Subscriber('dji_osdk_ros/main_camera_images', Image, image_cb)
rospy.Subscriber('dji_osdk_ros/rc', Joy, joy_cb)
