#!/usr/bin/env python3
import rospy
import rosbag
from navigation.msg import imu_angle
z_angle=x_angle=y_angle=0
bag=rosbag.Bag('imu_data.bag','w')


def angles_callback(msg):
    global z_angle,x_angle,y_angle
    z_angle=msg.Yaw
    x_angle=msg.Roll
    y_angle=msg.Pitch

def write_to_bagfile():
    global bag,z_angle,x_angle,y_angle
    imu_dat=imu_angle()
    imu_dat.Yaw=z_angle
    imu_dat.Roll=x_angle
    imu_dat.Pitch=y_angle
    bag.write('/imu_angles',imu_dat)

def main():
    while not rospy.is_shutdown():
        write_to_bagfile()
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("arrowdetect")
    rospy.Subscriber('/imu_angles', imu_angle, angles_callback)
    rate = rospy.Rate(10) #10Hz
    main()
    bag.close()