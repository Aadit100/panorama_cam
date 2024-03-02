#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from traversal.msg import WheelVel #this message needs to be changed
from traversal.srv import *
from sensor_msgs.msg import Joy


class drive():

    def __init__(self):
        input_service = rospy.Service('active_input', user_input, self.user_input)

        #output for esp32
        self.pub_motor=rospy.Publisher("motion",WheelVel,queue_size=10)
        self.pub_speed_mode=rospy.Publisher("mode",Int8,queue_size=10)  #for changing speed of motor (mode in which the rover is moving)
        self.pub_control_mode=rospy.Publisher("state",Bool,queue_size=10) #for telling if it is autonomously controlled or joystick controlled

        rospy.Subscriber("/joy",Joy,self.joyCallback)

        self.vel = 0
        self.omega = 0
        self.orientation = 0
        self.d = 0 #tells us which mode the rover is running in as in from mode 1 to mode 5
        self.s_arr = [25,35,50,75,110]
        self.active_input = 0 #for joystick control
        self.control = ['joystick','autonomous']

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if(self.active_input == 0):
                self.main()   #will go into the self.main fucntion only if the rover is in joystick control mode
            
            ctrl=Bool()
            ctrl.data=self.active_input

            self.pub_control_mode.publish(ctrl)  #for telling whether the rover is in autonomous or joystick controlled mode at all times and allows autonomous code to run

            rate.sleep()

    def main(self):
        rpm=WheelVel()
        
        rpm.vel=127+self.vel  #127 is our rest state
        rpm.omega=127+self.omega   #omega will only be used in the straight wheels orientation not any other since then it will be dragging the wheels
        rpm.orientation=self.orientation

        print(rpm)
        print('Mode: %d \nControl: %s'%(self.d+1, self.control[self.active_input]))
        if(self.orientation == 0):
            print('Move: Straight \n------------')
        elif(self.orientation == 1):
            print('Move: On Spot Rotation \n------------')
        elif(self.orientation == 2):
            print('Move: Left \n------------')
        elif(self.orientation == 3):
            print('Move: Right \n------------')
        else:
            print('Move: Invalid \n------------')

        self.pub_motor.publish(rpm)

    def joyCallback(self,msg):
        self.pub_speed_mode.publish(self.d)
        if(msg.buttons[7] == 1):
            self.active_input=not self.active_input
        
        if(self.active_input == 0):

            if(abs(msg.axes[3])>0.05):
                self.vel=int(msg.axes[3]*self.s_arr[self.d])
            else:
                self.vel=0

            if(abs(msg.axes[4])>0.05) and (self.orientation==0):  #check this axes in joystick
                self.omega=int(msg.axes[4]*self.s_arr[self.d])
            else:
                self.omega=0

            if(msg.buttons[4] == 1):
                self.orientation=0   #this is for going straight so wheels in straight orientation when orange button is pressed
            elif(msg.buttons[0] == 1):
                self.orientation=1  #this is for turning on the spot where the wheels orient themselves into  a circle like order
            elif(msg.buttons[3] == 1):
                self.orientation=2   #this orients the wheels in +90 degree angle i.e. towards the left
            elif(msg.buttons[1] == 1):
                self.orientation=3   #this will orient the wheels in -90 degree i.e. facing rightwards
            else:
                pass   #after deciding the orientation the rest will be handled by esp32 s3

            if(msg.buttons[9] == 1):
                if self.d<4:
                    self.d=self.d+1
            elif(msg.buttons[8] == 1):
                if self.d>0:
                    self.d=self.d-1

        else:
            pass

    def user_input(self, user_input):
        if(user_input.active_input == 0):
            self.active_input=user_input.active_input
            return user_inputResponse("joystick control active")

        elif(user_input.active_input == 1):
            self.active_input = user_input.active_input
            return user_inputResponse("autonomous control active")

        else:
            return user_inputResponse("invalid input")

if __name__=='__main__':
    rospy.init_node("drive")
    run=drive()
    run.spin()