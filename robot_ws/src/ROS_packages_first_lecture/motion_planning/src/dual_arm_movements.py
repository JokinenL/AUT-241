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

rospy.init_node('dual_arm_node', anonymous=True)

#Initializes moveit_commander
moveit_commander.roscpp_initialize(sys.argv) 
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_left = moveit_commander.MoveGroupCommander("arm_left")
arm_right = moveit_commander.MoveGroupCommander("arm_right")
arms = moveit_commander.MoveGroupCommander("arms")
torso = moveit_commander.MoveGroupCommander("torso")

arm_left.clear_pose_targets()
arm_right.clear_pose_targets()
arms.clear_pose_targets()
torso.clear_pose_targets()

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

def pose_to_pose_stamped(pose_target):
	pose_stamped_target = geometry_msgs.msg.PoseStamped()
	pose_stamped_target.header.frame_id = "base_link"
	pose_stamped_target.header.stamp = rospy.get_rostime()
	pose_stamped_target.pose = pose_target
	return pose_stamped_target

#Defining targets
pose_left_1 = Pose()
pose_left_1.position.x = -0.437763041979
pose_left_1.position.y = -0.274793374765
pose_left_1.position.z = 1.45995517614
pose_left_1.orientation.x = 0.0587091632808
pose_left_1.orientation.y = -0.830650268086
pose_left_1.orientation.z = -0.430332836679
pose_left_1.orientation.w = 0.348406394819

pose_right_1 = Pose()
pose_right_1.position.x = -0.437763041979
pose_right_1.position.y = 0.4
pose_right_1.position.z = 1.45995517614
pose_right_1.orientation.x = 0.0587091632808
pose_right_1.orientation.y = -0.830650268086
pose_right_1.orientation.z = -0.430332836679
pose_right_1.orientation.w = 0.348406394819

joints_left = [-0.292291522026062, 0.21216659247875214, 0.2918360233306885, -2.1203866004943848, 1.5354539155960083, 1.359760046005249, 1.201738715171814]
joints_right = [-0.6208702325820923, 0.8482232689857483, 0.3499906063079834, -1.9124031066894531, -1.278990626335144, -1.3122066259384155, 1.6812130212783813]

#Movements
print("Moving torso to predefined configuration")
torso.set_named_target("torso_platform_zero")
torso.go(wait=True)
time.sleep(2)

print("Moving arms to predefined configuration")
arms.set_named_target("arms_combs_2")
arms.go(wait=True)
time.sleep(2)

print("Moving arms to joint values")
arms.set_joint_value_target(joints_left + joints_right)
arms.go(wait=True)
time.sleep(2)

print("Moving to target pose")
arms.set_pose_target(pose_left_1, "arm_left_link_7_t")
arms.set_pose_target(pose_right_1, "arm_right_link_7_t")
arms.go(wait=True)
time.sleep(2)

print("Moving torso and arms to restart the position for the next simulation")
torso.set_named_target("torso_combs")
torso.go(wait=True)
time.sleep(2)
arms.set_named_target("arms_combs_2")
arms.go(wait=True)
time.sleep(2)

print("End")
