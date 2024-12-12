#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('joint_states_subscriber', anonymous=True)

def my_subscriber_cb(info):
    print(info)

my_subscriber = rospy.Subscriber('/joint_states', JointState, my_subscriber_cb)

rospy.spin()


