#!/usr/bin/env python3
import rospy
import cv2
import time
import os
from std_msgs.msg import Bool

def spin():
    global count
    while not rospy.is_shutdown() or count<19:
        if flag==1:
            ret, frame=vid.read()
            filename="img"+str(count)+".jpg"
            cv2.imwrite(filename, frame)
            time.sleep(2) #either 2 or 2.5
            count+=1
        elif flag==0:
            count=0
        rate.sleep()
    
def callback(msg):
    global flag
    flag=msg.data

if __name__ == '__main__':
    rospy.init_node("panorama")
    rate = rospy.Rate(10) #10Hz
    rospy.Subscriber('check', Bool, callback)
    # define a video capture object
    vid = cv2.VideoCapture(0)  #the camera has to be defined according to this number. Attaching the pantilt camera first may help
    directory = r'/home/nvidia/caesar2020/src/vision/scripts' #this address need to be changed according to xavier

    os.chdir(directory) #to go to the above mentioned directory
    flag=count=0
    spin()
    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv2.destroyAllWindows()