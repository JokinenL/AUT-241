#! /usr/bin/env python
import sys
import rospy
import copy
import PyKDL 
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped


rospy.init_node('grasp_ex2_node', anonymous=True)

#Initializes moveit_commander
moveit_commander.roscpp_initialize(sys.argv) 
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm = moveit_commander.MoveGroupCommander("manipulator") #Move_groups defined in the SRDF file
gripper = moveit_commander.MoveGroupCommander("gripper")

arm.clear_pose_targets()
gripper.clear_pose_targets()

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
		# Test if the object is in the scene. Note that attaching the object will remove it from known_objects
		is_known = object_name in scene.get_known_object_names()
		# Test if we are in the expected state
		if (object_is_attached == is_attached) and (object_is_known == is_known):
			return True
		# Sleep so that we give other threads time on the processor
		rospy.sleep(0.1)
		seconds = rospy.get_time()
	return False


#End Effector goal frame
object_frame = PyKDL.Frame()
object_frame.p = PyKDL.Vector(-0.5, 0.55, 0.3)
gripper_length = 0.24

#Transform End Effector frame to the goal_frame for the last joint of the arm
goal_frame = copy.deepcopy(object_frame)
goal_frame.p[2] += gripper_length #Convert from gripper to wrist frame
goal_frame.M.DoRotX(3.14) #Gripper pointing down

#Defines the PoseStamped that is the goal
goal_pose = frame_to_pose(goal_frame)
pose_stamped_target_wrist = geometry_msgs.msg.PoseStamped()
pose_stamped_target_wrist.header.frame_id = "base_link"
pose_stamped_target_wrist.header.stamp = rospy.get_rostime()
pose_stamped_target_wrist.pose = goal_pose
#Defines the offset pose
pose_stamped_target_offset_wrist = copy.deepcopy(pose_stamped_target_wrist)
pose_stamped_target_offset_wrist.pose.position.z += 0.1


print("Moving to target pose")
arm.set_pose_target(pose_stamped_target_offset_wrist)
arm.go(wait=True)
time.sleep(0.5)

print("Open gripper")
gripper.set_named_target('gripper_open')
gripper.go(wait=True)
time.sleep(0.5)

#(plan, fraction) = arm.compute_cartesian_path([pose_stamped_target_offset_wrist.pose, pose_stamped_target_wrist.pose], 0.01, 0.0)
#arm.execute(plan, wait=True)
arm.set_pose_target(pose_stamped_target_wrist)
arm.go(wait=True)

scene.attach_box("base_link_gripper", "box", touch_links=["finger_left, finger_right"])
wait_update_object('box', object_is_attached=True, object_is_known=False) #Wait until the object is attached
rospy.sleep(0.5)

print("Grasp")
gripper.set_named_target('gripper_grasp')
gripper.go(wait=True)
time.sleep(0.5)

#(plan, fraction) = arm.compute_cartesian_path([pose_stamped_target_wrist.pose, pose_stamped_target_offset_wrist.pose], 0.01, 0.0)
#arm.execute(plan, wait=True)
arm.set_pose_target(pose_stamped_target_offset_wrist)
arm.go(wait=True)

print("Moving home")
arm.set_named_target('home')
arm.go(wait=True)
time.sleep(0.5)

print('End')
