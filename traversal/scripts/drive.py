#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from traversal.msg import WheelRpm
from traversal.srv import *
from sensor_msgs.msg import Joy
from sensors.msg import Imu
import numpy
import math

class drive():

	def __init__(self):

		rospy.init_node("drive")

		#Service to decide control mode - autonomous or joystick    
		input_service = rospy.Service('active_input', user_input, self.user_input)
		
		#Service which rotates the rover to a specified bearing
		#rotate_service = rospy.Service('rotator',rotate,self.rotator)
		rospy.Subscriber("rotate",Int8,self.rotator)
		self.pub_rotate=rospy.Publisher('is_rotate', Bool, queue_size=10)
		
		#Service written as extra to move rover straight for specified distance.
		#Use once wheel odometry is achieved
		move_service= rospy.Service('move_straight',move,self.mover)
		
		#output for arduino
		self.pub_motor = rospy.Publisher("motion",WheelRpm,queue_size=10) 
		self.pub_speed_mode= rospy.Publisher("mode",Int8,queue_size=10)  
		
		rospy.Subscriber("drive_inp",WheelRpm,self.driveCallback)   #autonomous input
		rospy.Subscriber("imu", Imu, self.imuCallback)              #bearing input
		rospy.Subscriber("/joy",Joy,self.joyCallback)               #joystick input

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

	def spin(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.main()
			rate.sleep()

	def main(self):
		if True: # not self.rotate:	
			rpm = WheelRpm()
			rpm.hb = self.brake
			
			rpm.vel = 127- self.vel
			rpm.omega = 127 + self.omega

			print(rpm) 
			print('Mode : %d \nControl: %s \n--------------'%(self.d+1, self.control[self.active_input]))

			self.pub_motor.publish(rpm)
			#self.pub_rotate(False)
		else:
			print('rotation service active')

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

	def rotator(self, msg):

		if self.active_input ==  0:
			#return rotateResponse("Rotate_cant be done")
			print("Rotate can't be done")
		else:
		    #self.final_bear = request.angle
			self.final_bear = msg.data
			self.initial_bear = self.curr_bear
			self.remainAngle = self.final_bear - self.curr_bear
			self.omega = self.omegaManager(self.remainAngle)
			self.vel = 0
			self.pub_rotate.publish(True)
			
			while (abs(self.remainAngle) > self.bearing_tolerance and self.active_input == 1):

			#Rpm.vel = 127
				self.rotate = True
				self.remainAngle = self.final_bear - self.curr_bear
			#self.omega = self.omegaManager(self.remainAngle)
						
				if self.remainAngle>180 :
						self.remainAngle = self.remainAngle - 360
				elif self.remainAngle<-180 :
						self.remainAngle = self.remainAngle + 360
			
				omega_tmp = self.omegaManager(self.remainAngle)
	
				if self.remainAngle<0:
						self.omega = int(omega_tmp)
				else:
						self.omega = - int(omega_tmp)
				#print(self.remainAngle)
                    
			self.rotate = False
			rate = rospy.Rate(2)
			self.vel = 0
			self.omega = 0
			rate.sleep()

		#return rotateResponse("Rotate_finished")
		print("Rotate finished")
		self.pub_rotate.publish(False)
				

	def omegaManager(self,angle):
		self.curr_bear-=self.omega*0.000005
		precOmega = 8 + abs(angle)/self.divider        # varies the omega depending 
		return precOmega                        # on angle to be turned
	
	def mover(self,distance):
		pass

	def velManager(self,distance):
		pass

	def driveCallback(self,msg):
		if (self.active_input == 1):
			self.vel = msg.vel
			self.omega = msg.omega
			self.hb = False
		else: 
			pass

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

	def imuCallback(self,msg):
		self.curr_bear=msg.yaw

if __name__ == '__main__':
	run = drive()
	run.spin()	
