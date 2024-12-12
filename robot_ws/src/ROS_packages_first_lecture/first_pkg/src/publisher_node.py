#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Int32

rospy.init_node('publisher_node', anonymous=True)
start_count = rospy.get_param('~start', "")

my_publisher = rospy.Publisher('/exercises_RMCP/counter', Int32, queue_size=1)
count_msg = Int32()
count_msg.data = int(start_count)

while not rospy.is_shutdown():
	time.sleep(1)
	my_publisher.publish(count_msg)
	count_msg.data += 1


