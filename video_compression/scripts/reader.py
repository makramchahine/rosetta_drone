#!/usr/bin/env python2

import ffio

import cv2

import time


reader = ffio.FFReader('out.mp4')

reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();
reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();reader.read();


ret, image = reader.read()
cv2.imshow("hi", image)

cv2.waitKey(0)
