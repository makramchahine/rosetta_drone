#!/usr/bin/env python
import cv2
import numpy as np
cap = cv2.VideoCapture(0)

if (cap.isOpened() == False): 
  print("Unable to read camera feed")

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

out = cv2.VideoWriter('output.mp4',cv2.VideoWriter_fourcc('M','P','E','G'), 25, (frame_width,frame_height))
count =0
txt_colr=(255,0,0)
rect_colr=(255,255,255)
fnt_size=0.5
fnt_thick=1

while(True):
  ret, frame = cap.read()

  if ret == True: 
    cnt = str(count)
    (w,h),_=cv2.getTextSize(cnt,cv2.FONT_HERSHEY_SIMPLEX,fnt_size,fnt_thick)
    frame = cv2.rectangle(frame,(10,20-h),(w+10,20),rect_colr,-1)
    frame=cv2.putText(frame,cnt,(10,20),cv2.FONT_HERSHEY_SIMPLEX,fnt_size,txt_colr,fnt_thick,cv2.LINE_AA)
    count =count+1
    out.write(frame)  
    cv2.imshow('frame',frame)


    if cv2.waitKey(1) & 0xFF == ord('q'):
      break

  else:
    break  


cap.release()
out.release()

cv2.destroyAllWindows()