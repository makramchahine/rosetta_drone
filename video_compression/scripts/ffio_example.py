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

class VideoCompressionNode():

    def __init__(self): 
        rospy.init_node('video_compression_node')

        self.path = "/home/dji/flash/"

        self.image_mode = False

        self.path_appendix = ""

        self.delay_secs = 5
        self.last_change_time = rospy.get_time() - self.delay_secs
        self.last_input = 8000

        self.last_message = "starting up"

        self.crf = 10
        self.compression_preset = 'ultrafast'

        self.video_start_time = None
        self.video_times = []
        # crf=0 is lossless
        # crf=51 is maximum compression
        self.video_open = False
        self.close_video = True

        #self.width = 1280
        #self.height = 720

        self.width = 640
        self.height = 360

        self.frameid = 0

        rospy.Subscriber('dji_osdk_ros/main_camera_images', Image, self.image_cb)
        rospy.Subscriber('dji_osdk_ros/rc', Joy, self.joy_cb)

        #start camera stream
        rospy.wait_for_service('/setup_camera_stream')
        try:
            video_starter = rospy.ServiceProxy('/setup_camera_stream', SetupCameraStream)
	    resp1 = video_starter(1, 1)
	    #print(h)
	    #resp2 = gimbalctrl(h, False, 0,  0,  45.0,  0.0, 0.0, 0.5)
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

            rospy.wait_for_service('/gimbal_task_control')
            gimbalctrl = rospy.ServiceProxy('/gimbal_task_control', GimbalAction)	
 

            resp2 = gimbalctrl(h, False, 0,  0,  90.0,  0.0, 0.0, 0)


            #resp2 = gimbalctrl(h, False, 0,  0,  -90.0,  0.0, 0.0, 0)
            resp2 = gimbalctrl(h, True, 0,  0, 0.0,  0.0, 0.0, 0)

            print("starting recording")
            
            subprocess.Popen(["sh","/home/dji/catkin_ws/src/rosbagg/src/bagstart.sh"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            rostime = msg.header.stamp#rospy.Time.now()
            time = rostime.secs + rostime.nsecs * 1e-9
            self.video_start_time = time


            if not self.image_mode:
                self.video_times = []
                self.writer = ffio.FFWriter(self.path+'%.2f.mp4' % time, self.height, self.width, crf=self.crf, preset=self.compression_preset, verbose=True)
            else:
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
            
            subprocess.Popen(["sh","/home/dji/catkin_ws/src/rosbagg/src/bagend.sh"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            self.video_open = False
            return

        if self.video_open:

            

            #print("image callback",msg.height, msg.width)
            rostime = msg.header.stamp#rospy.Time.now()
            t0 = TimerTime.time()
            im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            im = im[::2, ::2, :]
                       
            time = rostime.secs + rostime.nsecs * 1e-9
 
            if self.image_mode:
                cv2.imwrite(os.path.join(self.path, self.path_appendix,  str(time) + ".png"), im)
            else:
                print('\nrehsape took %f seconds' % (TimerTime.time() - t0))
                #cv2.imshow("hiya", im)
                #cv2.waitKey(0)
                #cv2.imwrite("%04d.jpg" % self.frameid, im)
                #self.frameid += 1
                t0 = TimerTime.time()
                self.writer.write(im)
                print('writing took %f seconds\n' % (TimerTime.time() - t0))
                self.video_times.append(rostime)

    # T -8000 (left)
    # P 8000  (center)
    # S 0     (right)
    def joy_cb(self, msg):
        current_input = msg.axes[4]

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

        if self.last_message != message:
           print(message)
           self.last_message = message

        self.last_input = current_input



if __name__ == "__main__":
    VideoCompressionNode()
