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

rospy.init_node('moveit_ex1_node', anonymous=True)

#Initializes moveit_commander
moveit_commander.roscpp_initialize(sys.argv) 
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm = moveit_commander.MoveGroupCommander("manipulator") #Move_groups defined in the SRDF file
#gripper = moveit_commander.MoveGroupCommander("gripper")

arm.clear_pose_targets()
#gripper.clear_pose_targets()

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


#End Effector goal frame
EEF_frame = PyKDL.Frame()
EEF_frame.p = PyKDL.Vector(-0.5, 0.55, 0.3)
gripper_length = 0.24

#Transform End Effector frame to the goal_frame for the last joint of the arm
goal_frame = copy.deepcopy(EEF_frame)
goal_frame.p[2] += gripper_length #Convert from gripper to wrist frame
goal_frame.M.DoRotX(3.14) #Gripper pointing down

#Defines the PoseStamped that is the goal
goal_pose = frame_to_pose(goal_frame)
pose_stamped_target_wrist = geometry_msgs.msg.PoseStamped()
pose_stamped_target_wrist.header.frame_id = "base_link"
pose_stamped_target_wrist.header.stamp = rospy.get_rostime()
pose_stamped_target_wrist.pose = goal_pose

free_traj = True

if free_traj:
	print("Moving to target pose with free trajectory")
	arm.set_pose_target(pose_stamped_target_wrist)
	arm.go(wait=True)
	rospy.sleep(0.5)

else:
	print("Moving to target pose with cartesian straight path")
	current_pose = arm.get_current_pose().pose
	(plan, fraction) = arm.compute_cartesian_path([current_pose, pose_stamped_target_wrist.pose], 0.01, 0.0)
	arm.execute(plan, wait=True)
	rospy.sleep(0.5)

print("Moving home")
arm.set_named_target('home')
arm.go(wait=True)
time.sleep(0.5)

print("End")
