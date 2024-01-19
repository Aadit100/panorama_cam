#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge,CvBridgeError
import cv2

spectro=0
cap=cv2.VideoCapture(0)
print(cap.isOpened())
bridge=CvBridge()

def callback(msg):
    if msg.data:
        spectro=1
    else:
        spectro=0
        
def close():
    cap.release()
            
def talker():
    global spectro
    pub=rospy.Publisher('/webcam',Image,queue_size=1)
    try:
        rospy.Subscriber('/topic_name',Bool,callback) #change this according to the topic name
    except Exception(e):
        print(e)
    rospy.on_shutdown(close)
    rospy.init_node('image_pub',anonymous=False)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        if spectro:
            ret,frame=cap.read()
            if not ret:
                break
            
            msg=bridge.cv2_to_imgmsg(frame,"bgr8")
            pub.publish(msg)
        rate.sleep()  
if __name__='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
