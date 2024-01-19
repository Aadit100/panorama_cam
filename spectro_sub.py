#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge,CvBridgeError
import os
import cv2

bridge=CvBridge()
image=None
directory=r'/home'   #put the address of your host computer
i=0

os.chdir(directory)

def callback(msg):
    global image
    image=bridge.imgmsg_to_cv2(msg)
            
def talker():
    global image, i
    try:
        rospy.Subscriber('/webcam',Image,callback)
    except Exception(e):
        print(e)
    rospy.init_node('image_sub',anonymous=False)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
    	if image!=None
            filename="specto_img"+str(i)+".jpg"
            cv2.imwrite(filename,image)
            cv2.imshow(image)
            i+=1
        
        rate.sleep()
            
if __name__='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
