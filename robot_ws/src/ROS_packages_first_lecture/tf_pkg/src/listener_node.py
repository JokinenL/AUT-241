#!/usr/bin/env python3
import rospy
import time
import tf
import PyKDL 

rospy.init_node('listener_node', anonymous=True)

listener = tf.TransformListener() 

while not rospy.is_shutdown():
	time.sleep(0.5)
	try:
		(trans, rot) = listener.lookupTransform('/link_1', '/link_3', rospy.Time(0))
		print('trans: ' + str(trans))
		print('rot: ' + str(rot))
	except:
		continue
