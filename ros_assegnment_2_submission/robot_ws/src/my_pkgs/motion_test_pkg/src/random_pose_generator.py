#! /usr/bin/env python3
"""
This is the code for random pose generator node offering a service for generating random (x,y)-coordinates. The
coordinate points are generated in such a way that a box spawned in these locations will not collide with other
objects in the scene.
"""

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
from motion_test_pkg.srv import RandomPoseGenerator, RandomPoseGeneratorResponse
from math import sqrt
from random import uniform as rng

SERVICE_NAME = "/random_pose_generator/generate_pose"
ARM_1_FRAME = "one_sia20d_base_link"
ARM_2_FRAME = "two_sia20d_base_link"
WORLD_FRAME = "world"
MIN_DISTANCE = 0.4
MAX_DISTANCE = 0.9
MIN_X = -1.5
MAX_X = 1.5
MIN_Y = -0.8
MAX_Y = 0.8


class RandomPoseGeneratorNode:
    """
    The class responsible for the coordinate generation.
    """
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pose_generator_node', anonymous=True, log_level=rospy.WARN)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.tf_listener = tf.TransformListener()
        self.points_to_avoid = []
        self.robot1_origin = None
        self.robot2_origin = None
        rospy.Service(SERVICE_NAME, RandomPoseGenerator, self.generate_pose_cb)
        rospy.sleep(0.5)

    def spin(self):
        rospy.spin()

    def check_robot_origins(self):
        """
        Function for checking and saving the origins of the robot arms in the scene.
        """
        trans1, rot1 = self.tf_listener.lookupTransform(WORLD_FRAME, ARM_1_FRAME, rospy.Time(0))
        trans2, rot2 = self.tf_listener.lookupTransform(WORLD_FRAME, ARM_2_FRAME, rospy.Time(0))
        self.robot1_origin = (trans1[0], trans1[1])
        self.robot2_origin = (trans2[0], trans2[1])

    def loginfo(self, text):
        """
        Logger funtion with slightly modified formatting to make the readability of the log messages in the terminal better.
        Importance level: info.
        """
        rospy.loginfo(f"[random_pose_generator] {text}\n")

    def get_distance(self, point_a, point_b):
        """
        Function for computing eucleidian distance between two 2D-points.

        INPUT PARAMETERS
        :description point_a: The first 2D-point of interest in format (x, y).
        :type point_a: Tuple (of floats).

        :description point_b: The second 2D-point of interest in format (x, y).
        :type point_b: Tuple (of floats).

        RETURN VALUES
        :description dist: Distance of the two 2D-points along.
        :type dist: Float.
        """
        xa, ya = point_a
        xb, yb = point_b
        dist = sqrt((xa-xb)**2 + (ya-yb)**2)
        return dist

    def generate_pose_cb(self, req):
        """
        Callback function for service request. Generates random two dimensional point that is at least MIN_DISTANCE away
        from origins of any of the objects already in the scene and at most MAX_DISTANCE away from the closet robot arm
        in the scene.

        INPUT PARAMETERS
        :description req: Request message for the service. Does not contain any information relevant to the coordinate
                          generation, works only as a signal to execute the service.
        :type req: motion_test_pkg/msg/RandomPoseGeneratorRequest.

        RETURN VALUES
        :description resp: Response message including the generated coordinate point.
        :type resp: motion_test_pkg/msg/RandomPoseGeneratorResponse.
        """

        if self.robot1_origin is None or self.robot2_origin is None:
            self.check_robot_origins()
            self.points_to_avoid.append(self.robot1_origin)
            self.points_to_avoid.append(self.robot2_origin)
            
        objects = self.scene.get_objects()
        for object_name in objects:
            if object_name == "table":
                continue
            x_center = objects[object_name].pose.position.x
            y_center = objects[object_name].pose.position.y
            new_point_to_avoid = (x_center, y_center)
            if not new_point_to_avoid in self.points_to_avoid:
                self.points_to_avoid.append(new_point_to_avoid)
        
        while True:
            x = rng(MIN_X, MAX_X)
            y = rng(MIN_Y, MAX_Y)
            point_is_legal = True

            dist_1 = self.get_distance((x,y), self.robot1_origin)
            dist_2 = self.get_distance((x,y), self.robot2_origin)
            if (dist_1 > MAX_DISTANCE and dist_2 > MAX_DISTANCE):
                continue
            for p in self.points_to_avoid:
                dist_p = self.get_distance((x,y), p)
                if (dist_p < MIN_DISTANCE):

                    point_is_legal = False
                    break
            if point_is_legal:
                break

        resp = RandomPoseGeneratorResponse()
        resp.x = x
        resp.y = y
        resp.success = True
        self.loginfo(f"Generated random point ({x}, {y}).")

        return resp

def main():
    try:
        rpg = RandomPoseGeneratorNode()
        rpg.spin()
    except rospy.ROSInterruptException:
        pass
    

if __name__ == "__main__":
    main()