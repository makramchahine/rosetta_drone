#!/usr/bin/env python2

import time as TimerTime
import numpy as np
import rospy
from sensor_msgs.msg import Image, Joy

from dji_osdk_ros.srv import SetupCameraStream, GimbalAction

import ffio

import cv2

import subprocess

import std_msgs.msg

import os

from logger_example import Logger

class VideoCompressionNode():
    """Puts gimbal camera data into either a compressed mp4 or a folder of individual frames
    The mp4 format also creates a .txt file containing a list of timestamps, each corresponding to a timestamp for an individual frame
    if a frame is dropped however, these times will be off since ffpmeg doesn't say when frames are dropped so it can't be correcte for easily

    In image mode, each frame is written as a png with the timestamp as its name in a folder named after the timestamp of the start of the video

    Also creates a Logger object within the same node that keeps track of important topics and writes them to a csv full of timestamps.

    """

    def __init__(self): 
        rospy.init_node('video_compression_node')

        #base path to store all files
        self.path = "/home/dji/flash/"

        #False: store images as pngs in a folder
        #True:  store images in mp4 
        self.image_mode = False

        #this will be set to the folder containing pngs when image_mode is False
        self.path_appendix = ""

        #how many seconds to wait after a change in input to start accepting more commands
        #this is here as part of the Ramin-proofing state machine
        self.delay_secs = 5

        #the last time the flight mode has changed
        self.last_change_time = rospy.get_time() - self.delay_secs

        #the most recent flight mode
        self.last_input = 8000

        #the last message printed by the state machine - used to reduce spam
        self.last_message = "starting up"

        #compression settings
        # crf=0 is lossless
        # crf=51 is maximum compression
        self.crf = 10
        self.compression_preset = 'ultrafast'

        #for image_mode=True mp4 recording of times
        self.video_start_time = None
        self.video_times = []
        
        #state flags for deciding how to change the video
        self.video_open = False
        self.close_video = True

        #self.width = 1280
        #self.height = 720

        #output mp4 size
        self.width = 640
        self.height = 360

        rospy.Subscriber('dji_osdk_ros/main_camera_images', Image, self.image_cb)
        rospy.Subscriber('dji_osdk_ros/rc', Joy, self.joy_cb)

        self.logger = Logger()
        #start camera stream
        rospy.wait_for_service('/setup_camera_stream')
        try:
            video_starter = rospy.ServiceProxy('/setup_camera_stream', SetupCameraStream)
            resp1 = video_starter(1, 1)
            print("camera start result:",resp1)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


        print("spinning")

        rospy.spin()

    def image_cb(self, msg):
        h = std_msgs.msg.Header()
        h.seq = 0
        h.stamp = rospy.Time.now()
        h.frame_id = ''
        if not self.video_open and not self.close_video:

            #gimbal nod
            rospy.wait_for_service('/gimbal_task_control')
            gimbalctrl = rospy.ServiceProxy('/gimbal_task_control', GimbalAction)	
 

            resp2 = gimbalctrl(h, False, 0,  0,  90.0,  0.0, 0.0, 0)


            #resp2 = gimbalctrl(h, False, 0,  0,  -90.0,  0.0, 0.0, 0)
            resp2 = gimbalctrl(h, True, 0,  0, 0.0,  0.0, 0.0, 0)

            print("starting recording")


            #start rosbag
            subprocess.Popen(["sh","/home/dji/catkin_ws/src/rosbagg/src/bagstart.sh"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            rostime = msg.header.stamp#rospy.Time.now()
            time = rostime.secs + rostime.nsecs * 1e-9
            self.video_start_time = time

            #start generating csv
            self.logger.open_writer(os.path.join(self.path, "%.2f.csv" % time))

            if not self.image_mode:
                #start up mp4 writer
                self.video_times = []
                self.writer = ffio.FFWriter(self.path+'%.2f.mp4' % time, self.height, self.width, crf=self.crf, preset=self.compression_preset, verbose=True)
            else:
                #make a directory to store pngs
                self.path_appendix = '%f' % time
                os.mkdir(os.path.join(self.path, self.path_appendix))
            self.video_open = True
        elif self.video_open and self.close_video:
            print("ending recording")

            if not self.image_mode:
                self.writer.close()
                print("ffpmeg closed")
                with open(self.path+'frame_times_%.2f.txt' % self.video_start_time, 'w') as fo:
                    for t in self.video_times:
                        fo.write('%f,%d,%d\n' % (t.secs+t.nsecs*1e-9, t.secs, t.nsecs))
                print("frame times written")

            #dont need to do anything in png mode since the files have already been written
            
            #properly close csv writer
            self.logger.close_writer()

            #stop rosbag
            subprocess.Popen(["sh","/home/dji/catkin_ws/src/rosbagg/src/bagend.sh"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            self.video_open = False
            return

        if self.video_open:
            rostime = msg.header.stamp#rospy.Time.now()

            t0 = TimerTime.time()
            im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            im = im[::2, ::2, :]
                       
            time = rostime.secs + rostime.nsecs * 1e-9
 
            if self.image_mode:
                cv2.imwrite(os.path.join(self.path, self.path_appendix,  str(time) + ".png"), im)
            else:
                print('\nrehsape took %f seconds' % (TimerTime.time() - t0))
                t0 = TimerTime.time()
                self.writer.write(im)
                print('writing took %f seconds\n' % (TimerTime.time() - t0))
                self.video_times.append(rostime)

            #add newest state for this frame to the csv
            self.logger.write_state(rostime)

    # T -8000 (left)
    # P 8000  (center)
    # S 0     (right)
    def joy_cb(self, msg):
        """
        Processes the joystick message to detect mode changes to decide what to do with recording

        Moving the mode switch right turns on recording, and moving it to the left turns off recording
        There is also a delay that starts counting any time the input changes
        This way, after a command is given the mode switch has to be in a steady state for some time before
        new commands can be given, making it ok to accidentally switch it around a bunch while trying to get the switch to the center

        """
        current_input = msg.axes[4]

        #whether or not we're waiting for a delay
        waiting = abs(rospy.get_time()-self.last_change_time) < self.delay_secs	

        message = ""

        if current_input == -8000 and not self.close_video and not waiting:
            self.close_video = True
            message = "close video command sent"
        elif current_input == 0 and self.close_video and not waiting:
            self.close_video = False
            message = "start video command sent"
        elif not waiting:
            message = "accepting commands"
        else:
            message = "waiting"

        if current_input != self.last_input:
            self.last_change_time = rospy.get_time() 
            message += "\nwaiting %.2f seconds before allowing next command" % self.delay_secs

        # only send changed messages to avoid spam
        if self.last_message != message:
           print(message)
           self.last_message = message

        self.last_input = current_input



if __name__ == "__main__":
    VideoCompressionNode()
