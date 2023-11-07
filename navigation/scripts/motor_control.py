#Have to connect to motor driver in arduino code and take input from rostopic and give commands to motor driver accordingly   #done
#need to adjust the angles in the arduino code to stay between -pi and pi or 0 and 2pi  #done
#need to adjust the values of the pwm/control commands that are being passed since can't exceed 255
#need to adjust the conditions and values of kp,kd,ki according to position or velocity control
#need to see the velocity problem of -7200 it seems

#!/usr/bin/env python3
import rospy
from navigation.msg import *
from std_msgs.msg import Int16

class Plan():
    def __init__(self):
        rospy.init_node("Plan")
        self.load_vars() #variables
        self.load_params() #param variables

        #Subscriber
        try:
            rospy.Subscriber("feedback",enc_feed,self.feedCallback)
        except Exception(e):
            print(e)

        #Publisher
        self.pub_control=rospy.Publisher("control",Int16,queue_size=10)
        self.pub_planner_state=rospy.Publisher("planner_state",Planner_state,queue_size=2)

    def spin(self):
        rate=rospy.Rate(25)
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()

    def main(self):
        if(self.state=="pos"):
            if(abs(self.angle_dest-self.angle)>self.angle_tolerance):
                self.pub_planner_state.publish(0)
                remainAngle=self.angle_dest-self.angle
                
                if remainAngle>180:
                    remainAngle=remainAngle-360
                elif remainAngle<-180:
                    remainAngle=remainAngle+360
                
                self.control=self.output(remainAngle)
                self.control_pub()
        
            else:
                self.pub_planner_state.publish(1)
                rospy.loginfo("Destination Reached!")
        
        elif(self.state=="vel"):
            if(abs(self.vel_target-self.vel)>self.vel_tolerance):
                self.pub_planner_state.publish(0)
                error=self.vel_target_self.vel
                self.control=self.output(error)
                self.control_pub()
            else:
                self.pub_planner_state.publish(1)
                rospy.loginfo("Velocity Achieved!")

        elif(self.state=="stop"):
            self.control=0
            self.control_pub()
            self.pub_planner_state.publish(0)

    
    def output(self,error):
        tim=rospy.get_time() #gets current time
        dt=tim-self.time_prev #compares and gets the time difference b/w previous and current time

        if(self.time_prev==0):
            dt=0

        if(dt!=0):
            output_d=((error-self.error_prev)/dt)*self.kd
            output_p=error*self.kp
        else:
            output_d=0
            output_p=0
        self.error_int += error*dt
        output_i=self.error_int*self.ki
        output=(output_p+output_d+output_i)  #see whether it is +ve or -ve

        self.time_prev=tim
        self.error_prev=error

        if output>0:
            output=min(127,output)
        else:
            output=max(-127,output)

        return output

    def control_pub(self):
        con=Int16()
        con.data=int(self.control)
        self.pub_control.publish(con)

    def load_params(self):
        self.angle_tolerance=float(1)
        self.vel_tolerance=float(4)
        self.kp=1
        self.ki=0
        self.kd=0.5

    def load_vars(self):
        self.state="pos" #states are 'pos','vel','stop'
        self.angle_dest=-170
        self.angle=0
        self.vel_target=30
        self.vel=0
        self.control=0

        self.error_prev=0
        self.error_int=0
        self.time_prev=0

    def feedCallback(self,msg):
        self.vel=msg.vel
        self.angle=msg.angle
    

if __name__=='__main__':
    run=Plan()
    run.spin()