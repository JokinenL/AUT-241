#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import String

rospy.init_node('string_publisher_node', anonymous=True)
publish_msg = rospy.get_param('~msg', "")

my_publisher = rospy.Publisher('/exercises_RMCP/message', String, queue_size=1)
msg = String()
msg.data = publish_msg
time.sleep(0.5)
my_publisher.publish(msg)



