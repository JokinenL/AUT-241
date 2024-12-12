#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

rospy.init_node('subscriber_node', anonymous=True)

def my_subscriber_cb(info):
    print(str(info.data))

my_subscriber = rospy.Subscriber('/exercises_RMCP/counter', Int32, my_subscriber_cb)

rospy.spin()


