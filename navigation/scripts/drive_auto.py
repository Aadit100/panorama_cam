#!/usr/bin/env python3
import pyrealsense2 as rs
import rospy
from std_msgs.msg import String
from navigation.msg import gps_data
import math
from time import *
import cv2
import numpy as np
import imutils
from navigation.msg import imu_angle
from traversal.msg import WheelRpm
from traversal.srv import *
from sensor_msgs.msg import Joy
from sensors.msg import Imu
from std_msgs.msg import Float32

x_angle = 0
y_angle = 0
z_angle = 0
rotate_incr = 0
turn = False
arrow_direction = "Right"
distance_gps = 0 #distance between current position and initial position, given by GPS
distance = 0 #distance between current position and arrow given by realsense
wheelrpm_pub = rospy.Publisher('IMUGPS_Pub', WheelRpm, queue_size=10)
    
def Navigate():
    rospy.init_node('imu_angles_listener', anonymous=True)
    while not rospy.is_shutdown():
        rospy.Subscriber('imu_angles', imu_angle, angles_callback)
        rospy.Subscriber('gps_data', gps_data, gps_callback)
        rospy.Subscriber('depth', Float32, realsense_callback)
        move_straight()
        turn()

def realsense_callback():
    #callback from realsense (arrow detection distance)
    pass

def move_straight():
    
    global x_angle, y_angle, z_angle, distance, wheelrpm_pub, turn
    if(turn == True):
        return
    min_dist = 2
    vel = -25
    omega = 0
    msg = WheelRpm()
    msg.vel = 127-vel
    msg.omega = 127-omega
    msg.hb=False
    if(distance >= min_dist):
        wheelrpm_pub.publish(msg)
        turn = False

    else:
        msg.vel = 127
        msg.omega = 127
        wheelrpm_pub.publish(msg)
        rospy.sleep(10)
        turn = True

def turn():
    global x_angle, y_angle, z_angle, distance, wheelrpm_pub, turn, rotate_incr

    if(turn == False):
        return
    

    msg = WheelRpm()
    msg.vel = 127
    msg.hb=False
    error = 0
    if(arrow_direction == "Right"):
        if(z_angle < (90-error)):
            msg.omega = 152
            wheelrpm_pub.publish()
            turn = True
        else:
            msg.vel = 127
            msg.osmega = 127
            z_angle = 0
            msg.hb = False
            turn = False
            rospy.sleep(2)
            rotate_incr = rotate_incr -1 #to readjust z_angle
    else:
        if(z_angle < (90-error)):
            msg.omega = 102
            wheelrpm_pub.publish()
            turn = True
        else:
            msg.vel = 127
            msg.omega = 127
            msg.hb = False
            z_angle = 0
            turn = False
            rospy.sleep(2)
            rotate_incr = rotate_incr +1 # to readjust z_angle 

def angles_callback(msg):
    global x_angle, y_angle, z_angle
    x_angle, y_angle = msg.Pitch, msg.Roll
    z_angle = msg.Yaw + rotate_incr * 90
    rospy.loginfo("Received angles - X: {}, Y: {}, Z: {}".format(x_angle, y_angle, z_angle))

def gps_callback(msg):
    global distance_gps
    distance_gps = msg.data


if __name__ == '__main__':
    Navigate()

