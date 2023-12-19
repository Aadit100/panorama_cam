#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from traversal.msg import WheelRpm
from traversal.srv import *
from sensor_msgs.msg import Joy
import numpy
import math
import pyrealsense2 as rs
import rosbag
from navigation.msg import gps_data
import cv2
import imutils
from navigation.msg import imu_angle

class drive():
	def __init__(self):
        #Service to decide control mode - autonomous or joystick    
		input_service = rospy.Service('active_input', user_input, self.user_input)
		
        #output for arduino
		self.pub_motor = rospy.Publisher("motion",WheelRpm,queue_size=10) 
		self.pub_speed_mode= rospy.Publisher("mode",Int8,queue_size=10)  
		
		#rospy.Subscriber("drive_inp",WheelRpm,self.driveCallback)   #autonomous input
		rospy.Subscriber("/joy",Joy,self.joyCallback)               #joystick input
		rospy.Subscriber('imu_angles', imu_angle, self.angles_callback)  #enc feed is message which will provide vel and angle of motor
        rospy.Subscriber('gps_data', gps_data, self.gps_callback)
		
		self.straight = 0
		self.zero_turn = 0
		self.vel = 0
		self.omega = 0
		self.d = 0
		self.brake = False
		self.rotate = False
		self.s_arr = [25,35,50,75,110]#[40,100,150,200,800][5, 10, 20, 50,90]
		self.bearing_tolerance = rospy.get_param('/rot_server/bearing_tolerance',2)
		self.active_input = 0
		self.divider = rospy.get_param('/rot_server/divider', 30)
		self.control = ['joystick', 'autonomous']
		self.curr_bear=0
		rospy.on_shutdown(self.close)
		self.pipeline = rs.pipeline() #responsible for initialising realsense
        config = rs.config()
        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
	    self.pipeline.start(config)
        self.template_r=cv2.imread('Template.png',0)  #need to store the template images of arrows
        self.template_l=cv2.imread('Template_l.png',0)
        self.template_r=cv2.resize(self.template_r,(60,40),cv2.INTER_AREA)
        self.template_l=cv2.resize(self.template_l,(60,40),cv2.INTER_AREA)
        self.h,self.w=self.template_r.shape
        self.z_angle = self.x_angle = self.y_angle =0
        self.turn = False
        self.min_dist=1.7
        self.dist_thresh=0.3
        self.kp=25
        self.kp_rot=0.55
        self.distance=None
        self.direction=None
        self.current_latitude=None
        self.current_longitude=None
        self.ret=False
        self.initial_yaw=0
        self.bag=rosbag.Bag('imu_data.bag','w')
	
	def spin(self):
		while not rospy.is_shutdown():
			self.main()
			rate.sleep()
			
    def main(self):	
        rpm = WheelRpm()
        rpm.hb = self.brake
        
        rpm.vel = 127- self.vel
        rpm.omega = 127 + self.omega

        print(rpm) 
        print('Mode : %d \nControl: %s \n--------------'%(self.d+1, self.control[self.active_input]))

        self.pub_motor.publish(rpm)
        #self.pub_rotate(False)
			
	def user_input(self, user_input):
		if(user_input.active_input == 0):
			self.active_input = user_input.active_input
			return user_inputResponse("joystick control active")
			print ("control handed over to joystick\n")
		elif(user_input.active_input == 1):
			self.active_input = user_input.active_input
			return user_inputResponse("autonomous control active")
			print ("autonomous control active\n")
		else:
			return user_inputResponse("invalid input")
		
    def joyCallback(self,msg):
		self.pub_speed_mode.publish(self.d)
		if(msg.buttons[0] == 1):
			self.active_input = not self.active_input

		if (self.active_input == 0):

			if(abs(msg.axes[1])>0.05 or abs(msg.axes[3])>0.05):
				self.vel = int(msg.axes[1]*self.s_arr[self.d])
				self.omega = int(msg.axes[3]*self.s_arr[self.d])
				
			else:
				self.vel = 0
				self.omega = 0
			if(msg.buttons[8]==1):
				self.brake = True
			else:
				self.brake = False

			if(msg.buttons[6]==1):
				if self.d < 4:
					self.d = self.d + 1
			
			elif(msg.buttons[7]==1):
				if self.d > 0:
					self.d = self.d - 1

		else:
			pass
	
    def angles_callback(self,msg):
        self.z_angle = msg.Yaw
        self.x_angle = msg.Roll
        self.y_angle = msg.Pitch
		
    def gps_callback(self,msg):
        self.current_latitude=msg.latitude
        self.current_longitude=msg.longitude


if __name__ == '__main__':
	rospy.init_node("drive")
	run = drive()
	rate = rospy.Rate(10) #10Hz
	wheelrpm_pub = rospy.Publisher('motion', WheelRpm, queue_size=10)
	run.spin()