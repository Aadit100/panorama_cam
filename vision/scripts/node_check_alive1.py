#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8

if __name__=='__main__':
    try:
        count=0
        rospy.init_node('check',anonymous=True)
        opt=int(input("Enter option\nRotate backward:2\nRotate forward:3\n:"))
        pub=rospy.Publisher('check_node',Int8,queue_size=10)
        ch=Int8()
        rate=rospy.Rate(10)
        while (not rospy.is_shutdown() and count<10):
            ch.data=opt
            pub.publish(ch)
            count+=1
            rate.sleep()
        ch.data=0
        pub.publish(ch)
    except rospy.ROSInterruptException:
        pass