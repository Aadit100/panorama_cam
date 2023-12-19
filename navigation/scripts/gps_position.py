#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from navigation.msg import gps_data
import math
from time import *

# Global variables to store the previous GPS coordinates
previous_latitude = 0
previous_longitude = 0
current_latitude = 0
current_longitude = 0
iter=0
start_time=time()

def gps_callback(data):
    global previous_latitude, previous_longitude,current_latitude,current_longitude

    current_latitude = data.latitude
    current_longitude = data.longitude

def haversine(lat1, lon1, lat2, lon2):
    # Radius of the Earth in kilometers
    R = 6371.0

    # Convert latitude and longitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Haversine formula
    dlat = abs(lat2_rad - lat1_rad)
    dlon = abs(lon2_rad - lon1_rad)
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

def gps_listener():
    global current_latitude,current_longitude,previous_latitude,previous_longitude,iter,start_time
    pub = rospy.Publisher('gps_pub', Float32, queue_size=10)
    while not rospy.is_shutdown():
        rospy.Subscriber("gps_coordinates",gps_data , gps_callback)
        if (previous_latitude != 0) and (previous_longitude != 0):
            # Calculate the distance using the Haversine formula
            distance = haversine(current_latitude, current_longitude, previous_latitude, previous_longitude)
            rospy.loginfo("Distance to Previous Location: {:.4f} km".format(distance))
            msg_pub = Float32()
            msg_pub.data = distance
            pub.publish(msg_pub)


        # Update the previous location
        if(iter==0):
            previous_latitude = current_latitude
            previous_longitude = current_longitude
        if(time()-start_time>5):
            iter=iter + 1

if __name__ == '__main__':
    try:
        rospy.init_node('gps_distance_calculator', anonymous=True)
        gps_listener()
    except rospy.ROSInterruptException:
        pass
