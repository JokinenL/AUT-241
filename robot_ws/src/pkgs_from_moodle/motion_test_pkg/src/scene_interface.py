#! /usr/bin/env python3
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
from std_srvs.srv import Trigger, TriggerResponse
from motion_test_pkg.srv import *

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('scene_node', anonymous=True)
scene = moveit_commander.PlanningSceneInterface()

#MODIFY DEPENDING ON THE NAME OF THE LINKS OF YOUR ROBOT
eef_links = ["first_gripper_base_link", "second_gripper_base_link"]
touch_links = [["first_gripper_finger_left, first_gripper_finger_right"],["second_gripper_finger_left, second_gripper_finger_right"]]

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
    print(object_name)
    try:
        for eef_link in eef_links:
            scene.remove_attached_object(eef_link, name=object_name)
            wait_update_object(object_name, object_is_attached=False, object_is_known=True)
            rospy.sleep(0.5)
    except:
        print("Error dettaching objects")

rospy.sleep(0.5)
scene_objects = scene.get_known_object_names()
for object_name in scene_objects:
	print(object_name)	
	scene.remove_world_object(object_name)
	wait_update_object(object_name, object_is_attached=False, object_is_known=False)
	rospy.sleep(0.5)
#print(scene.get_known_object_names())


#Add the object to the scene
print("Adding box")

try:
    scene_objects = scene.get_known_object_names()
    print(scene_objects)
    if not("table" in scene_objects):
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "world"
        table_frame = PyKDL.Frame()
        table_frame.p = PyKDL.Vector(0,0,0)
        table_pose.pose = copy.deepcopy(frame_to_pose(table_frame))
        table_size = (4, 3, 0.699)
        table_pose.pose.position.x = 0
        table_pose.pose.position.y = 0
        table_pose.pose.position.z = 0
        table_pose.pose.position.z += table_size[2]/2
        table_name = "table"
        scene.add_box(table_name, table_pose, table_size)
        wait_update_object(table_name, object_is_attached=False, object_is_known=True)
        rospy.sleep(0.5)
except:
    print("Error adding the object to the scene")


def spawn_box_cb(req): 
    """
    Service to grasp the dummy piece
    """
    global grasped
    resp = BoxSpawnerResponse()

    try:
        scene_objects = scene.get_known_object_names()
        attached_objects = scene.get_attached_objects()
        if req.name in scene_objects:
            if req.name in attached_objects:
                for eef_link in eef_links:
                    scene.remove_attached_object(eef_link, name=req.name)
                    wait_update_object(req.name, object_is_attached=False, object_is_known=True)
                    rospy.sleep(0.5)
            scene.remove_world_object(req.name)
            wait_update_object(req.name, object_is_attached=False, object_is_known=False)
            rospy.sleep(0.5)

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_frame = PyKDL.Frame()
        box_frame.p = PyKDL.Vector(req.x,req.y,0.7)
        box_pose.pose = copy.deepcopy(frame_to_pose(box_frame))
        if req.base:
            box_size = (0.1, 0.1, 0.05)
        else:
            box_size = (0.05, 0.05, 0.05)
        box_pose.pose.position.z += box_size[2]/2
        box_name = req.name
        scene.add_box(box_name, box_pose, box_size)
        wait_update_object(box_name, object_is_attached=False, object_is_known=True)
        rospy.sleep(0.5)
        resp.success = True

    except:
        resp.success = False
        resp.message = "Error. Grasp was not successful"

    return resp
	
rospy.Service('/scene_spawner/spawn_box', BoxSpawner, spawn_box_cb)


def attach_box_cb(req): 
    """
    Service to grasp the dummy piece
    """
    global grasped
    resp = BoxAttachResponse()

    try:
        if req.robot_name == 'robot1':
            index = 0
        else:
            index = 1
        if req.attach:
            scene.attach_box(eef_links[index], req.box_name, touch_links=touch_links[index])
            wait_update_object(req.box_name, object_is_attached=True, object_is_known=False)
            resp.success = True
        else:
            scene.remove_attached_object(eef_links[index], name=req.box_name)
            wait_update_object(req.box_name, object_is_attached=False, object_is_known=True)
        rospy.sleep(0.5)

    except:
        resp.success = False
        resp.message = "Error. Attach/dettach was not successful"

    return resp
	
rospy.Service('/scene_spawner/attach_release_box', BoxAttach, attach_box_cb)


rospy.spin()