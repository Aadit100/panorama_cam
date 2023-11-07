#!/usr/bin/env python3
#add service to reload params
import rospy
from navigation.srv import *
from navigation.msg import *
from sensors.msg import *
from traversal.msg import *
from traversal.srv import *
from termcolor import colored
import numpy as np        #don't know why numpy is imported is not used
import sys, signal,thread


class Planner():
	
	def __init__ (self):
		rospy.init_node("Planner")
		self.load_vars() #variables
		self.load_params() #param variables

		#subscribers
		try:
			rospy.Subscriber("imu",Imu, self.imuCallback)
			rospy.Subscriber("goal",Goal,self.goalCallback) #it consists of the required yaw angle and the distance we need to move
			#rospy.Subscriber("scan",LaserScan,self.rplCallback)
		except Exception e:
			print (e)

		#publishers
		self.pub_drive=rospy.Publisher("drive_inp",WheelRpm,queue_size=10)
		self.pub_planner_state=rospy.Publisher("planner_state",Planner_state,queue_size=2)

		#service server
		self.state_ser=rospy.Service('Planner_state_ctrl',plan_state,self.state_ctrl) #state service

	def spin(self):
		rate = rospy.Rate(25)		
		while not rospy.is_shutdown():
			self.main() #main func
			rate.sleep()

	def main(self):
		if(self.state=="run"):   #main part in which the calculations are done
			if(self.distance_to_dest > self.dist_tolerance): 
				self.obs_scanner_active = False		#its false in both if or else statements so it doesn't really make sense
				self.pub_planner_state.publish(0) #i guess this is 1 only when the rover has reached the required destination

				if self.iter == 0 :
					result = self.rotator()  #before anything is done rotate the rover according to what the destination
					print (result)
					self.iter = self.iter+1

				if abs(self.bearing_dest-self.bearing_curr)>3*self.bearing_tolerance:  #we are comparing the error to the tolerance
					result = self.rotator() #after one rotation again yaw angle is checked after maybe translation is done once there might be some error
					print (result)
				elif self.distance_to_dest < 3*self.dist_tolerance:
					self.omega = 0  #if the distance is not satisfied but the orientation is fine then we just need to move forward and no omega is required
					self.vel = self.forward_vel_cal(1.5) #distance is between 1 and 3 times distance tolerance #1.5 is something
					self.drive_pub()
				else:				
					error=self.bearing_dest-self.bearing_curr   #once again correcting the error if any after motion
					print (error) 
					if error>180 :
						error = error - 360
					elif error<-180 :
						error = error + 360
					
					self.omega = self.output(error)  #PID is applied  #basically they want to ensure that rover is in correct orientation before it reaches the required distance
					self.vel = self.forward_vel_cal(1.5)
					self.drive_pub()

			else :
				self.pub_planner_state.publish(1)   #if distance and orientation is satisfied then reached destination
				self.obs_scanner_active = False
				rospy.loginfo("Desination Reached!")

		elif(self.state=="pause"):  #nothing is done in this case which is wierd since what if the rover is moving. Or maybe that doesn't happen
			pass
			
		elif(self.state=="stop"):   #this is to explicitly stop the rover from moving any further
			self.vel = 0
			self.omega = 0 
			self.drive_pub()
			self.pub_planner_state.publish(0)
			pass

	def state_ctrl(self,srv_msg):    #this is for controlling the state the rover is in
		if (srv_msg.pause==1 and srv_msg.contin==0 ) :  #condition for pausing
			self.state = "pause"  #this changes the codes inherent state variable
		elif (srv_msg.contin==1 and srv_msg.pause==0):
			self.state = "run"
		elif (srv_msg.rst==1): #condition for stopping analogous to restarting
			self.state = "stop"
		else:
			rospy.loginfo("Error in changing planner state")
		#print(srv_msg.contin)
		return plan_stateResponse(self.state)

	def rotator(self):		
		while (abs(self.bearing_dest - self.bearing_curr) > self.bearing_tolerance):
			remainAngle = self.bearing_dest - self.bearing_curr  #error basically
						
			if remainAngle>180 :
				remainAngle = remainAngle - 360  #better to move in opposite direction so -360 since shorter distance
			elif remainAngle<-180 :
				remainAngle = remainAngle + 360   #similar logic to above
			
			omega_tmp = 17 + abs(remainAngle)/8  #temporary omega
	
			if remainAngle<0:
				self.omega = int(omega_tmp)  #don't know why this seems to be opposite maybe some reverse configuration in imu angle and omega of rover
			else:
				self.omega = - int(omega_tmp)
			self.vel = 0      #maybe they just want to turn first and only then move forward
			self.drive_pub()  

		rate = rospy.Rate(2)
		self.vel = 0
		self.omega = 0
		self.drive_pub() #after the rotating is done stop the rover
		rate.sleep()
		return "Rotate_finished - error=%f"%(self.bearing_dest-self.bearing_curr)

	def output(self,error):
		tim = rospy.get_time()  #gets current time
		dt = tim-self.time_prev  #compares and gets the time difference between previous time and this time of getting data

		if(self.time_prev==0): #if first time taking reading then since no signal is given before this so this is actually 0 time and from here on out the signals will start to be sent.
			dt=0

		self.tsys+=dt  #don't know what this is used for

		output_p = error*self.kp   #proportional error as always simple
		if(dt!=0):    #if it is not the first actuation signal
			output_d=((error-self.error_prev)/dt)*self.kd   #derivative of error wrt time is found and the derivative actuation is provided
		else:
			output_d=0  #if first actuation then no dervative actuation
		self.error_int=self.error_int+error*dt  #integration is done similar to riemannian sum
		output_i=self.error_int*self.ki  #integral output
		output = -(output_p+output_d+output_i)  #now obviously output will be -kx

		self.time_prev=tim   #previous time is updated to current time
		self.error_prev=error  #previous error is updated

		if output>0 :
			output=min(127,output)  #since it could be bad to give more than 127 in magnitude
		else:
			output=max(-127,output)

		return output

	
	def load_params(self):    #as the name suggests this is for loading all the parameters
		self.dist_tolerance     = float(rospy.get_param('/planner/dist_tolerance', 1.5))        #in metres ; default 5 metre
		self.bearing_tolerance  = float(rospy.get_param('/planner/bearing_tolerance', 4))    #in degrees ; default 10 degrees
		self.forward_max        = 45#float(rospy.get_param('/planner/forward_max', 40))          #in terms of pwm now
		self.forward_min        = 30#float(rospy.get_param('/planner/forward_min', 25))          #in terms of pwm now          #in terms of pwm value
		self.forward_mult       = float(rospy.get_param('/planner/forward_mult', 1.0))
		self.divider            = float(rospy.get_param('/planner/divider', 2))
		self.kp=1.7
		self.ki=0.6
		self.kd=0

	def load_vars(self):
		self.state  = "run"  # states are 'run','pause','stop'  #initial state is run
		self.bearing_dest = 150   #desired yaw angle   #this is the yaw angle we wish to reach and is not the error I guess
		self.bearing_curr = 0			#current bearing of the rover  bearing is basically yaw angle of the rover
		self.distance_to_dest = 20   #distance you want to move initially   #this variable defines the error only between current and required destination
		self.vel = 0  #wheel rpm message
		self.omega = 0
		self.iter = 0

		self.error_prev=0
		self.error_int=0
		self.time_prev=0
		self.tsys=0  #don't know what this is used for
		
	def imuCallback(self,msg):
		self.bearing_curr = msg.yaw   #this yaw we get from imu

	def goalCallback(self,msg):# each time i am getting a new goal i have to reset the distance calculator node
		self.distance_to_dest = float(msg.distance)  #new desired distance and yaw angle is put . They are getting from gps coordinates i think
		self.bearing_dest = float(msg.bearing)

	def distCallback(self,msg): #getting the position of the bot from the pos calculator #apparently not used
		self.dist = msg.dist

	'''def rplCallback(self,msg): #getting the position of the bot from the pos calculator
		self.lidar = np.array(msg.ranges)'''

	def drive_pub(self): #used to send the drive node the info, the value of theta taken is 0 to 359 if any other value is given the service won't be called.
		rpm = WheelRpm()
		rpm.vel=self.vel
		rpm.omega=self.omega
		self.pub_drive.publish(rpm)    #this is the input to drive motors

	def forward_vel_cal(self,vel_mult):
		vel = self.forward_min + (abs(self.forward_max-self.forward_min)*vel_mult)
		return min(vel,self.forward_max)

def signal_handler(signal, frame):  #For catching keyboard interrupt Ctrl+C
	print ("\nProgram exiting.....")
	sys.exit(0)


if __name__ == '__main__':
	run = Planner()
	signal.signal(signal.SIGINT, signal_handler)  #this makes keyboard interrupts such as Ctrl C and delete as keys that stop the program
	run.spin()
