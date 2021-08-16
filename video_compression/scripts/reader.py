#!/usr/bin/env python2

import ffio

import cv2

import time

# this reads out some frames from the mp4 to show it's possible

reader = ffio.FFReader('out.mp4')

reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();
reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();


ret, image = reader.read()
cv2.imshow("hi", image)

cv2.waitKey(0)
