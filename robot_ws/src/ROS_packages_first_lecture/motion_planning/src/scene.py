#! /usr/bin/env python
import sys
import rospy
import copy
import PyKDL 
import time
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import math

#ROS init
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('scene_node', anonymous=True)
scene = moveit_commander.PlanningSceneInterface()


#Functions for poses conversion
def frame_to_pose(frame):
	pose_result = Pose()
	pose_result.position.x = frame.p[0] 
	pose_result.position.y = frame.p[1] 
	pose_result.position.z = frame.p[2] 
	ang = frame.M.GetQuaternion() 
	pose_result.orientation.x = ang[0] 
	pose_result.orientation.y = ang[1] 
	pose_result.orientation.z = ang[2] 
	pose_result.orientation.w = ang[3]
	return pose_result


#Object pose
object_frame = PyKDL.Frame()
object_frame.p = PyKDL.Vector(-0.5, 0.55, 0.3)
object_pose = frame_to_pose(object_frame)


#Wait until a modification in the objects in the scene is updated
def wait_update_object(object_name, object_is_known=False, object_is_attached=False):
	global scene
	start = rospy.get_time()
	seconds = rospy.get_time()
	while (seconds - start < 4) and not rospy.is_shutdown():
		# Test if the object is in attached objects
		attached_objects = scene.get_attached_objects([object_name])
		is_attached = len(attached_objects.keys()) > 0
		# Test if the object is in the scene.
		# Note that attaching the object will remove it from known_objects
		is_known = object_name in scene.get_known_object_names()
		# Test if we are in the expected state
		if (object_is_attached == is_attached) and (object_is_known == is_known):
			return True
		# Sleep so that we give other threads time on the processor
		rospy.sleep(0.1)
		seconds = rospy.get_time()
	return False


#Remove and detach the object if it is in the scene.
print("Initializing")

rospy.sleep(0.5)
attached_objects = scene.get_attached_objects()
for object_name in attached_objects:
	try:
		scene.remove_attached_object(eef_link, name=object_name)
		wait_update_object(object_name, object_is_attached=False, object_is_known=True)
		rospy.sleep(0.5)
	except:
		print("Error dettaching objects")

rospy.sleep(0.5)
scene_objects = scene.get_known_object_names()
for object_name in scene_objects:	
	scene.remove_world_object(object_name)
	wait_update_object(object_name, object_is_attached=False, object_is_known=False)
	rospy.sleep(0.5)
#print(scene.get_known_object_names())


#Add the object to the scene
print("Adding box")
try:
	box_pose = geometry_msgs.msg.PoseStamped()
	box_pose.header.frame_id = "base_link"
	box_pose.pose = copy.deepcopy(object_pose)
	box_name = "box"
	box_size = (0.05, 0.05, 0.1)
	box_pose.pose.position.z += box_size[2]/2
	scene.add_box(box_name, box_pose, box_size)
	wait_update_object(box_name, object_is_attached=False, object_is_known=True)
	rospy.sleep(0.5)
	#print(scene.get_known_object_names())

	table_pose = geometry_msgs.msg.PoseStamped()
	table_pose.header.frame_id = "base_link"
	table_pose.pose = copy.deepcopy(object_pose)
	table_size = (0.2, 0.2, (object_pose.position.z - 0.01))
	table_pose.pose.position.z = 0
	table_pose.pose.position.z += table_size[2]/2
	table_name = "table"
	scene.add_box(table_name, table_pose, table_size)
	wait_update_object(table_name, object_is_attached=False, object_is_known=True)
	rospy.sleep(0.5)

	#print(scene.get_known_object_names())
except:
	print("Error adding the object to the scene")


# Keep this process running until Enter is pressed
print ("Press Enter to quit...")
try:
	sys.stdin.readline()
	print("Removing objects")
	rospy.sleep(0.5)
	attached_objects = scene.get_attached_objects()
	for object_name in attached_objects:
		try:
			scene.remove_attached_object(eef_link, name=object_name)
			wait_update_object(object_name, object_is_attached=False, object_is_known=True)
			rospy.sleep(0.5)
		except:
			print("Error dettaching objects")

	rospy.sleep(0.5)
	scene_objects = scene.get_known_object_names()
	for object_name in scene_objects:	
		scene.remove_world_object(object_name)
		wait_update_object(object_name, object_is_attached=False, object_is_known=False)
		rospy.sleep(0.5)
#print(scene.get_known_object_names())
except KeyboardInterrupt:
	print("End of test")
