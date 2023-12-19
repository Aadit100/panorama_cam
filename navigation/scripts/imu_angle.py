#!/usr/bin/env python3

import rospy
from navigation.msg import imu_angle
from time import *

STOP_THRESHOLD = 45.0  # Set the threshold angle for stopping

def angles_callback(msg):
    x_angle, y_angle, z_angle = msg.Pitch, msg.Roll, msg.Yaw
    rospy.loginfo("Received angles - X: {}, Y: {}, Z: {}".format(x_angle, y_angle, z_angle))

    if abs(z_angle) >= STOP_THRESHOLD:
        rospy.loginfo("Stop!")
        # You can add code here to perform any desired actions when the stop condition is met
        # For example, you might want to send a stop command to the Arduino or take some other action.

def listener():
    rospy.init_node('imu_angles_listener', anonymous=True)
    rospy.Subscriber('imu_angles', imu_angle, angles_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
