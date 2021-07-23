#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('image_processing')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def trackbar_callback(self, val):
    self.threshold_type = cv2.getTrackbarPos(self.trackbar_type, self.window_name)
    self.threshold_value = cv2.getTrackbarPos(self.trackbar_value, self.window_name)
    self.gamma = cv2.getTrackbarPos("gamma", self.window_name)

  def camera_stream(self):

    self.max_value = 255
    self.max_type = 4
    self.max_binary_value = 255
    self.trackbar_type = 'Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted'
    self.trackbar_value = 'Value'
    self.window_name = 'Threshold Demo'
    self.gamma = .4

    self.threshold_value = 0
    self.threshold_type = 0

    cv2.namedWindow(self.window_name)

    cv2.createTrackbar(self.trackbar_type, self.window_name , 3, self.max_type, self.trackbar_callback)
    # Create Trackbar to choose Threshold value
    cv2.createTrackbar(self.trackbar_value, self.window_name , 0, self.max_value, self.trackbar_callback)
    cv2.createTrackbar("gamma", self.window_name , 0, 1, self.trackbar_callback)

    # define a video capture object
    vid = cv2.VideoCapture(0)
      
    while(True):
          
      # Capture the video frame
      # by frame
      ret, frame = vid.read()

      flipped = cv2.flip(frame, 1)

      gray = cv2.cvtColor(flipped, cv2.COLOR_BGR2GRAY)

      lookUpTable = np.empty((1,256), np.uint8)
      for i in range(256):
          lookUpTable[0,i] = np.clip(pow(i / 255.0, self.gamma) * 255.0, 0, 255)
      res = cv2.LUT(gray, lookUpTable)

      _, dst = cv2.threshold(res, self.threshold_value, self.max_binary_value, self.threshold_type )
    
      # Display the resulting frame
      cv2.imshow('frame', dst)
        
      # the 'q' button is set as the
      # quitting button you may use any
      # desired button of your choice
      if cv2.waitKey(1) & 0xFF == ord('q'):
        break
      
    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  ic.camera_stream()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)