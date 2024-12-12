#!/usr/bin/env python3
import rospy
import time
import tf
import PyKDL 

rospy.init_node('broadcaster_node', anonymous=True)

br = tf.TransformBroadcaster()
frame_br = PyKDL.Frame()
frame_br.p = PyKDL.Vector(2, 0, 0) 
increment = 0.05

while not rospy.is_shutdown():
	time.sleep(0.5)

	if frame_br.p.z() >= 1:
		increment = -0.05
	if frame_br.p.z() <= 0:
		increment = 0.05
	new_z = frame_br.p.z() + increment
	frame_br.p = PyKDL.Vector(2, 0, new_z)

	br.sendTransform((frame_br.p.x(), frame_br.p.y(), frame_br.p.z()), 
        		frame_br.M.GetQuaternion(), 
			rospy.get_rostime(), 
			"my_new_tf", 
			"/link_2")


