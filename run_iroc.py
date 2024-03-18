#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import Joy  #sensor_msgs mainly involve IMU and Joy

class Node:
    def __init__(self):
        
        self.outbuff = [0] * 5  #length should be 6 not 4  #we will see this later
        
        self.pub = rospy.Publisher('stm_write',Int32MultiArray, queue_size=10) #this is where the stm_write topic is published to 
        rospy.init_node('arm_drive')  #name of the node
        rospy.Subscriber('joy', Joy, self.joyCallback) #subscribes to the joy topic to take the inputs from the joystick controlling the arm

    def joyCallback (self, msg):
        outbuff = [0, 0, 0, 0, 0] #outbuff has total 5 values
                
        outbuff[0] = - msg.axes[0]  #shuffle these around according to convenience
        outbuff[1] = msg.axes[1]
        outbuff[2] = msg.axes[3]  #only four of the axes are in use for the joystick control for base rotation, elbow, shoulder, gripper
        outbuff[3] = (msg.buttons[1] - msg.buttons[3])*255 #for pitch and roll
        outbuff[4] = msg.axes[2]
        
        self.outbuff = outbuff #setting the outbuff variable of class as outbuff to store the previous outbuff maybe
        print (self.outbuff)

    def run (self):
        rate = rospy.Rate (50)
        while not rospy.is_shutdown():
            rate.sleep()
            msg = self.createMsg (self.outbuff)  #for creating the message #returns the msg which is then published
            self.pub.publish (msg)
    
    def createMsg (self, buff):
        # Inititalize the ROS Msg type
        msg = Int32MultiArray() #of multiarray type for joystick message
        
        # Creates a Shallow Copy of outbuff
        msg.data = buff[:]  #will give whole buffer list
        
        msg.layout = MultiArrayLayout()
        msg.layout.data_offset = 0  #still haven't figured out what this offset actually does and don't need to either
        
        msg.layout.dim = [MultiArrayDimension() ]
        msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)  #we have the buff in msg.data
        msg.layout.dim[0].label = 'write'
        
        return msg

if __name__ == '__main__':
    node = Node()  #object of node class and the constructor is run
    node.run() 

