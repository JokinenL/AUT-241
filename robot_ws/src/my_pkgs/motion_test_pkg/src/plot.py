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
from math import pi
from std_srvs.srv import Trigger, TriggerResponse
from motion_test_pkg.srv import BoxAttach, BoxAttachResponse, BoxSpawnerResponse, BoxSpawner, RandomPoseGenerator, RandomPoseGeneratorResponse
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import numpy as np

TABLE_LEVEL = 0.7
TOP_BOX_HEIGHT = 0.05
GRASPING_HEIGHT = TABLE_LEVEL + TOP_BOX_HEIGHT/2

POSE_GENERATOR_SERVICE_NAME = "/random_pose_generator/generate_pose"



def pose_generator_client(base):
    rospy.wait_for_service(POSE_GENERATOR_SERVICE_NAME)
    resp = RandomPoseGeneratorResponse()
    resp.success = False
    try:
        generate_pose = rospy.ServiceProxy(POSE_GENERATOR_SERVICE_NAME, RandomPoseGenerator)
        resp = generate_pose(base=base)
    except:
        rospy.logwarn("Service call failed")

    return resp


def my_print(text):
    rospy.logwarn(f"[testialusta.py] {text}")

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('plotting_node', anonymous=True)
    pose_resp = pose_generator_client(base=True)
    print(f"x: {pose_resp.x}, y: {pose_resp.y}")


    

    time.sleep(0.5)
    

if __name__ == "__main__":
    main()