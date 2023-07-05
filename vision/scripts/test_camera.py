#!/usr/bin/env python3
import cv2
import time
import os

time.sleep(1)
# define a video capture object
vid = cv2.VideoCapture(0)
directory = r'/home/aadit/catkin_ws/src/vision/scripts'

os.chdir(directory)

for i in range(19):
    ret, frame=vid.read()
    filename="img"+str(i)+".jpg"  
    cv2.imwrite(filename, frame)
    time.sleep(2.5)
    
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()