#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String

if __name__=='__main__':
    try:
        rospy.init_node('check',anonymous=True)
        pub=rospy.Publisher('check_node',Bool,queue_size=10)
        ch=Bool()
        rate=rospy.Rate(10)
        while (not rospy.is_shutdown()):
            ch.data=True
            pub.publish(ch)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
