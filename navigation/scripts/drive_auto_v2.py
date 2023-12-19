#!/usr/bin/env python3
import pyrealsense2 as rs
import rospy
from navigation.msg import gps_data
import math
from time import *
import cv2
import numpy as np
import imutils
from navigation.msg import imu_angle
from traversal.msg import WheelRpm
from traversal.srv import *

class auto():

    def __init__(self):
        #self.cap=cv2.VideoCapture(0)
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
        self.z_angle = 0
        self.rotate_incr = 0
        self.turn = False
        self.min_dist=1.5
        self.distance=None
        self.direction=None
        self.current_latitude=None
        self.current_longitude=None
        self.ret=False

        #Subscriber
        try:
            rospy.Subscriber('imu_angles', imu_angle, self.angles_callback)  #enc feed is message which will provide vel and angle of motor
            rospy.Subscriber('gps_data', gps_data, self.gps_callback)
        except Exception(e):
            print(e)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        align=rs.align(rs.stream.color)
        frames=align.process(frames)
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None, None
        return True, depth_image, color_image, depth_frame
    
    def arrowdetect(self,image,depth):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found=None
        found_l=None
        for scale in np.linspace(0.06, 1.0, 70)[::-1]:
            resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
            r = gray.shape[1] / float(resized.shape[1])
            if resized.shape[0] < self.h or resized.shape[1] < self.w:
                break
            result = cv2.matchTemplate(resized, self.template_r, cv2.TM_CCOEFF_NORMED)
            minVal,maxVal,minLoc,maxLoc = cv2.minMaxLoc(result)
            result = cv2.matchTemplate(resized, self.template_l, cv2.TM_CCOEFF_NORMED)
            minVal_l,maxVal_l,minLoc_l,maxLoc_l = cv2.minMaxLoc(result)
            if found is None or maxVal > found[0]:
                found = (maxVal, maxLoc, r)
            if found_l is None or maxVal_l > found_l[0]:
                found_l = (maxVal_l, maxLoc_l, r)
        (maxVal, maxLoc, r) = found
        (maxVal_l, maxLoc_l, r) = found_l

        if maxVal_l>maxVal: 
            maxVal=maxVal_l
            maxLoc=maxLoc_l
            direction='left'
        else:
            direction='right'
        
        if maxVal>0.70:
            (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
            (endX, endY) = (int((maxLoc[0]+self.w)*r), int((maxLoc[1]+self.h)*r))
            cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
            point=(int((startX+endX)/2),int((startY+endY)/2))
            camera_info=depth.profile.as_video_stream_profile().intrinsics
            distance=depth.get_distance(point[0],point[1])
            centre=rs.rs2_deproject_pixel_to_point(camera_info,point,distance)
            distance=centre[2]
            return True, direction,point[0],distance
        return False,None,None,None
    
    def main(self):
        if(not self.turn):
            ret, depth_frame, color_frame, depth = self.get_frame()  #returns bool value, depth frame, color frame and depth of the object   #from realsense
            self.ret,self.direction,pix,self.distance=self.arrowdetect(color_frame,depth) #distance refers to the distance between the rover and the arrow that is to be detected
            if(self.ret):   #self.ret is true only if arrow has been detected
                print("arrow detected at distance: "+str(self.distance))
            self.move_straight()
        else:
            if(self.direction=="left"):
                self.rotate(+1)
            else:
                self.rotate(-1)


    def spin(self):
        while not rospy.is_shutdown():
            self.main()
            rate.sleep()

    def angles_callback(self,msg):
        self.z_angle = msg.Yaw + self.rotate_incr * 90

    def gps_callback(self,msg):
        self.current_latitude=msg.latitude
        self.current_longitude=msg.longitude
        
    def write_coordinates(self):
        file_object=open("coordinates.txt","a")
        file_object.write("latitude: "+str(self.current_latitude)+"   longitude: "+str(self.current_longitude))
        file_object.close()

    def move_straight(self):
        msg = WheelRpm()
        msg.hb=False
        msg.omega = 127

        if(self.ret and self.distance<=self.min_dist):
            msg.vel = 127
            wheelrpm_pub.publish(msg)
            rospy.sleep(10)   #I guess this is to stop for some time but this is what may be causing problems   # 10s   # Competition rules say 10s
            self.turn = True
            self.write_coordinates()
            #rospy.Subscriber('gps_data', gps_data, self.gps_callback) #I don't know if placing it here does anything
        else:
            vel = +25
            msg.vel = 127-vel  #have to check which command is right direction
            wheelrpm_pub.publish(msg)
            self.turn = False

    def rotate(self,dir):
        msg = WheelRpm()
        msg.hb=False  
        msg.vel = 127
        error = 0

        if(-(90-error)<self.z_angle<(90-error)):               
            msg.omega = 127+(-35*dir) #make sure this is correct either -35*dir or +35*dir
            wheelrpm_pub.publish(msg)
        else:
            self.rotate_incr += dir  #to readjust z_angle
            msg.omega=127
            wheelrpm_pub.publish(msg)
            self.z_angle=0
            self.turn=False
            rospy.sleep(2) 
    
    def close(self):
        msg_stop=WheelRpm()
        msg_stop.hb=False
        msg_stop.vel=127
        msg_stop.omega=127
        wheelrpm_pub.publish(msg_stop)


if __name__ == '__main__':
    rospy.init_node("arrowdetect")
    rate = rospy.Rate(10) #10Hz
    wheelrpm_pub = rospy.Publisher('motion', WheelRpm, queue_size=10)
    run=auto()
    run.spin()