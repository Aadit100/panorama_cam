#!/usr/bin/env python3

import rospy
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs  #sensor_msgs mainly involve IMU and Joy

class Node:
    def __init__(self):
        
        self.outbuff = [0] * 4  #length should be 6 not 4
        
        self.pub = rospy.Publisher('stm_write', std_msgs.Int32MultiArray, queue_size=10) #this is where the stm_write topic is published to 
        rospy.init_node('arm_drive')  #name of the node
        rospy.Subscriber('joy', sensor_msgs.Joy, self.joyCallback) #subscribes to the joy topic to take the inputs from the joystick controlling the arm
    
    def joyCallback_0 (self, msg):  #don't think this is used maybe for different joystick configurations
        self.outbuff = [ int (msg.axes[i] * 0xFF) for i in range(4) ]
        self.outbuff += [ (msg.buttons[i] - msg.buttons[i+2]) * 0xFF for i in range(4, 6) ] 
        print (self.outbuff)

    def joyCallback (self, msg):
        outbuff = [0, 0, 0, 0, 0, 0] #outbuff has total 6 values
        
        #outbuff = [ int (msg.axes[i] * 0xFF) for i in range(4) ]
        #outbuff += [ (msg.buttons[i] - msg.buttons[i+2]) * 0xFF for i in range(4, 6) ]
        
        axes = [ int (msg.axes[i] * 0xFF) for i in range(5) ]
        buttons = [ (msg.buttons[1] - msg.buttons[3])*255]
        buttons.append((msg.buttons[0] - msg.buttons[2])*255)  #there are 4 buttons for controlling pitch and roll and all and difference controls directions
        
        outbuff[0] = - axes[0]
        outbuff[1] = axes[1]
        outbuff[2] = axes[3]  #only four of the axes are in use for the joystick control for base rotation, elbow, shoulder, gripper
        outbuff[3] = buttons[0] #for pitch and roll
        outbuff[4] = buttons[1]
        outbuff[5] = axes[2]
        
        self.outbuff = outbuff #setting the outbuff variable of class as outbuff to store the previous outbuff maybe
        print (self.outbuff)

    def run (self):
        rate = rospy.Rate (50)
        while not rospy.is_shutdown():
            rate.sleep()
            msg = self.createMsg (self.outbuff)  #for creating the message
            self.pub.publish (msg)
    
    def createMsg (self, buff):
        # Inititalize the ROS Msg type
        msg = std_msgs.Int32MultiArray() #of multiarray type for joystick message
        
        # Creates a Shallow Copy of outbuff
        msg.data = buff[:] 
        
        msg.layout = std_msgs.MultiArrayLayout()
        msg.layout.data_offset = 0  #still haven't figured out what this offset actually does
        
        msg.layout.dim = [ std_msgs.MultiArrayDimension() ]
        msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)  #we have the buff in msg.data
        msg.layout.dim[0].label = 'write'
        
        return msg

node = Node()  #object of node class and the constructor is run
node.run() 

