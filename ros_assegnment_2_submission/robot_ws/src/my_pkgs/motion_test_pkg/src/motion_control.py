#! /usr/bin/env python3
"""
This is the code for motion control node responsible for executing the process where two boxes, base box and
top box are first spawned into the scene and then the top box is moved over the base box by performing 
pick-and-place task. The boxes are spawned by calling /scene_spawner/spawn_box service that is implemented
in scene interface node and their coordinates are received from random pose generator node that offers
/random_pose_generator/generate_pose service. The whole process of spawning the boxes and performing the
pick-and-place task is implemented as ROS action; everytime the action goal is sent
MotionController.pick_and_place_action function is executed. The actual pick-and-place task can be performed
by single robot arm if it is the closest one to both the base and the top box or as co-operation between both
robot arms if one is closer to the top box and other is closer to the base box when they are first spawned.
"""
import sys
import rospy
import copy
import PyKDL 
import time
import tf
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import Constraints, OrientationConstraint
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from math import pi
from std_srvs.srv import Trigger, TriggerResponse
from motion_test_pkg.srv import BoxAttach, BoxAttachResponse, BoxSpawnerResponse, BoxSpawner, RandomPoseGenerator, RandomPoseGeneratorResponse
from scipy.spatial.transform import Rotation
from math import sqrt
import actionlib
from motion_test_pkg.msg import ProcessAction, ProcessFeedback, ProcessResult, ProcessGoal


AVOID_COLLISION_OFFSET = 0.001
TABLE_LEVEL = 0.7
BOX_HEIGHT = 0.05
PICKUP_HEIGHT = TABLE_LEVEL + BOX_HEIGHT/2
PLACE_HEIGHT = PICKUP_HEIGHT + BOX_HEIGHT + AVOID_COLLISION_OFFSET
WRIST_TO_FINGERTIPS_OFFSET = 0.185
PRE_GRASP_OFFSET = 0.1
BOX_SPAWNER_SERVICE_NAME = "/scene_spawner/spawn_box"
POSE_GENERATOR_SERVICE_NAME = "/random_pose_generator/generate_pose"
BOX_ATTACH_SERVICE_NAME = "/scene_spawner/attach_release_box"
DROP_OFF_LOCATION_0 = [0, -0.5, PICKUP_HEIGHT]
DROP_OFF_LOCATION_1 = [0, 0.5, PICKUP_HEIGHT]
TOP_BOX_SIDE_LENGTH = 0.05
BASE_BOX_SIDE_LENGTH = 0.1
SAFETY_DISTANCE = 1/sqrt(2)*(BASE_BOX_SIDE_LENGTH + TOP_BOX_SIDE_LENGTH)
ARM_1_FRAME = "one_sia20d_base_link"
ARM_2_FRAME = "two_sia20d_base_link"
WORLD_FRAME = "world"
NOT_STARTED = 0
FIRST_TIME_PICKING_UP = 1
SWITCHING = 2
FINAL_PLACING = 3


class Box():
    """
    Helper class for representing box object including all the box information that the MotionController object
    needs.
    """
    def __init__(self ,name, base, origin=(0,0)):
        self.name = name
        self.base = base
        self.origin = origin

class MotionController:
    """
    Main class considering this node. Responsible for the execution of the whole process described in the first
    docstring.
    """
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('motion_control_node', anonymous=True, log_level=rospy.WARN)
        self.robot = moveit_commander.RobotCommander()
        arm1 = moveit_commander.MoveGroupCommander("arm1")
        arm2 = moveit_commander.MoveGroupCommander("arm2")
        self.arm1_location = None
        self.arm2_location = None
        self.placing_arm_id = None
        self.picking_arm_id = None
        gripper1 = moveit_commander.MoveGroupCommander("gripper1")
        gripper2 = moveit_commander.MoveGroupCommander("gripper2")
        self.arms = [arm1, arm2]
        self.grippers = [gripper1, gripper2]
        self.arm_to_use = None
        self.gripper_to_use = None
        self.path_constraints = None
        self.scene = moveit_commander.PlanningSceneInterface()

        base_box = Box(name="base_box", base=True)
        top_box = Box(name="top_box", base=False)
        self.boxes = {"base_box": base_box, "top_box": top_box}
        self.tf_listener = tf.TransformListener()

        self.action_server = actionlib.SimpleActionServer("process_action", ProcessAction, self.pick_and_place_action, False)
        self.action_server.start()
        self.action_feedback = ProcessFeedback()
        self.action_result = ProcessResult()
        self.process_state = NOT_STARTED

        rospy.sleep(1.0)


    def check_robot_locations(self):
        """
        Function for checking (and setting) the locations of the two robot arms in the scene.
        """
        trans1, rot1 = self.tf_listener.lookupTransform(WORLD_FRAME, ARM_1_FRAME, rospy.Time(0))
        trans2, rot2 = self.tf_listener.lookupTransform(WORLD_FRAME, ARM_2_FRAME, rospy.Time(0))

        self.arm1_location = trans1
        self.arm2_location = trans2

    def set_arm_and_gripper_to_use(self, arm_id):
        """
        Function for setting the correct arm and gripper as "active" so they can be called from any
        function by using the arm_to_use class variable.

        INPUT PARAMETERS
        :description arm_id: Integer presenting which arm/gripper to use (needs to be 0 or 1).
        :type arm_id: Int.
        """
        self.arm_to_use = self.arms[arm_id]
        self.gripper_to_use = self.grippers[arm_id]

    def get_2d_distance(self, point_a, point_b):
        """
        Function for computing two dimensional distance between two points. Only x and y coordinates are
        considered. Thus, this function should be used only in cases where the z-component of the distance is
        ignored.

        INPUT PARAMETERS
        :description point_a: The first point of interest in format [x, y, z].
        :type point_a: List (of floats).

        :description point_b: The second point of interest in format [x, y, z].
        :type point_b: List (of floats).

        RETURN VALUES
        :description dist: 2D distance of the two points along xy-plane (z coordinate does not affect this).
        :type dist: Float.
        """

        xa, ya, _ = point_a
        xb, yb, _ = point_b
        dist = sqrt((xa-xb)**2 + (ya-yb)**2)
        return dist
        

    def return_action_result(self, msg):
        """
        Function for returning the action result to the client node.

        RETURN VALUES
        :description msg: Text to send with the action result.
        :type msg: String
        """
        self.action_result.message = msg
        self.action_server.set_succeeded(self.action_result)
    
    def loginfo(self, text):
        """
        Logger funtion with slightly modified formatting to make the readability of the log messages in the terminal better.
        Importance level: info.
        """
        rospy.loginfo(f"[motion_control_node] {text}\n")

    def logwarn(self, text):
        """
        Logger funtion with slightly modified formatting to make the readability of the log messages in the terminal better.
        Importance level: warning.
        """
        rospy.logwarn(f"[motion_control_node] {text}\n")

    def logerr(self, text):
        """
        Logger funtion with slightly modified formatting to make the readability of the log messages in the terminal better.
        Importance level: error.
        """
        rospy.logerr(f"[motion_control_node] {text}\n")



    def pick_and_place_action(self, goal):
        """
        Function that is responsible for managing the whole pick-and-place process and box spawning before it.
        This is called everytime an action goal is sent to a topic /process_action/goal.

        INPUT PARAMETERS
        :description goal: Signal to start the action, contents of the goal message don't matter.
        :type goal: motion_test_pkg/ProcessActionGoal.
        """

        self.action_result.success = True
        try:
            self.spawn_boxes()
            # Picking the top box
            target_object = self.boxes["top_box"]
            x_start, y_start = target_object.origin
            z_start = PICKUP_HEIGHT
            starting_point = [x_start, y_start, z_start]
            self.check_robot_locations()
            dist1_start = self.get_2d_distance(starting_point, self.arm1_location)
            dist2_start = self.get_2d_distance(starting_point, self.arm2_location)
            if dist1_start <= dist2_start:
                picking_arm_id = 0
            else:
                picking_arm_id = 1
            self.set_arm_and_gripper_to_use(picking_arm_id)
            self.process_state = FIRST_TIME_PICKING_UP
            self.pick_object(starting_point)

            # Considering the target location
            x_destination, y_destination = self.boxes["base_box"].origin
            z_destination = PLACE_HEIGHT
            destination = [x_destination, y_destination, z_destination]
            dist1_destination = self.get_2d_distance(destination, self.arm1_location)
            dist2_destination = self.get_2d_distance(destination, self.arm2_location)
            # Concluding which arm to use for placing the top box to its final location
            if dist1_destination < dist2_destination:
                placing_arm_id = 0
            elif dist1_destination > dist2_destination:
                placing_arm_id = 1
            else:
                placing_arm_id = picking_arm_id
            # If the arm to use for placing the 
            # top_box is different than the arm used for picking it, we need
            # to make a switch with the box at the drop off location
            if placing_arm_id != picking_arm_id:
                self.process_state = SWITCHING
                # It is unlikely but possible, that the base box is spawned so close to the default drop off location
                # that the top box would collide with it during the drop off. If this the case and the swith needs to
                # be done, the secondary drop off location is used.
                if self.get_2d_distance(DROP_OFF_LOCATION_0, destination) <= SAFETY_DISTANCE:
                    self.loginfo("Place target too close to the DROP_OFF_LOCATION_0. Using DROP_OFF_LOCATION_1 instead.")
                    drop_off_location = DROP_OFF_LOCATION_1
                else:
                    drop_off_location = DROP_OFF_LOCATION_0
                self.place_object(drop_off_location)
                self.set_arm_and_gripper_to_use(placing_arm_id)
                self.pick_object(drop_off_location)
            else:
                self.set_arm_and_gripper_to_use(placing_arm_id)
            self.process_state = FINAL_PLACING
            self.place_object(destination, final_target=True)
            self.return_action_result("Process finished.")
        # If there appear any exeptions during any of the subprocesses they will be cached and logged here
        except Exception as e:
            self.logerr(f"Interrupting the process due to following error:\n{e}")
            self.action_result.success = False
            self.return_action_result("Process interrupted due to unexpected errors.")
        

    def spin(self):
        rospy.spin()

    
    def spawn_boxes(self):
        """
        Function for spawning the boxes at the start of the process. Uses the box_spawner_client to actually
        spawn the boxes.
        """
        for box_name in self.boxes:
            box = self.boxes[box_name]
            pose_resp = self.pose_generator_client()
            self.loginfo(f"{box_name} coordinates: ({pose_resp.x}, {pose_resp.y})")
            spawner_resp = self.box_spawner_client(x=pose_resp.x, y=pose_resp.y, name=box.name, base=box.base)
            if not spawner_resp.success:
                self.publish_action_feedback(f"Spawning {box.name} failed.")
                break
            self.publish_action_feedback(f"Spawned {box.name}.")
            box.origin = (pose_resp.x, pose_resp.y)
            rospy.sleep(0.5)

    def box_spawner_client(self, x, y, name, base):
        """
        This function calls the spawn_box service offered by scene spawner node.

        INPUT PARAMETERS
        :description x: The x coordinate of the box to be spawned.
        :type x: Float.

        :description y: The y coordinate of the box to be spawned.
        :type y: Float.

        :description name: The name of the box to be spawned.
        :type name: String.

        :description base: Boolean telling if the box to be spawned is the base box.
        :type base: Bool.

        RETURN VALUES
        :description resp: Response message from the service server.
        :type resp: motion_test_pkg/srv/BoxSpawnerResponse.
        """
        rospy.wait_for_service(BOX_SPAWNER_SERVICE_NAME)
        resp = BoxSpawnerResponse()
        resp.success = False
        try:
            spawn_box = rospy.ServiceProxy(BOX_SPAWNER_SERVICE_NAME, BoxSpawner)
            resp = spawn_box(x=x, y=y, name=name, base=base)
        except:
            self.logwarn("Service call failed for box_spawner_client.")

        return resp
    
    def box_attach_client(self, box_name="top_box", attach=True):
        """
        This function calls the attach_box service offered by scene spawner node.

        INPUT PARAMETERS
        :description box_name: The name of the box to be attached; by default "top_box" since the base box
                               is never grasped.
        :type box_name: String.

        :description attach: Boolean telling if the box should be attached (True) or deattached (False).
        :type attach: Bool.

        RETURN VALUES
        :description resp: Response message from the service server.
        :type resp: motion_test_pkg/srv/BoxAttachResponse.
        """
        resp = BoxAttachResponse()
        resp.success = False
        rospy.wait_for_service(BOX_ATTACH_SERVICE_NAME)
        try:
            attach_box = rospy.ServiceProxy(BOX_ATTACH_SERVICE_NAME, BoxAttach)
            resp = attach_box(robot_name=self.arm_to_use.get_name(),
                              box_name=box_name,
                              attach=attach)
        except:
            self.logwarn("Service call failed for box_attach_client.")

        return resp

    def pose_generator_client(self):
        """
        Funtion for requesting the generate_pose service offered by random pose generator node.

        RETURN VALUES
        :description resp: Response message from the service server; includes the requested x and y coordinates.
        :type resp: motion_test_pkg/srv/RandomPoseGeneratorResponse.
        """
        rospy.wait_for_service(POSE_GENERATOR_SERVICE_NAME)
        resp = RandomPoseGeneratorResponse()
        resp.success = False
        try:
            generate_pose = rospy.ServiceProxy(POSE_GENERATOR_SERVICE_NAME, RandomPoseGenerator)
            resp = generate_pose()
        except:
            self.logwarn("Service call failed for pose_generator_client")
        return resp

    def publish_action_feedback(self, msg):
        """
        Function for publishing action feedback to the client node.

        INPUT PARAMETERS
        :description msg: Feedback text to publish.
        :type msg: String.
        """
        self.action_feedback.message = msg
        self.action_server.publish_feedback(self.action_feedback)

    def get_wrist_target_pose(self, target_coords):
        """
        Function for transforming the target coordinates in format [x, y, z] to target pose
        of the wrist of the robot arm.

        INPUT PARAMETERS
        :description target_coords: Target coordinates in format [x, y, z].
        :type target_coords: List (of floats).

        RETURN VALUES
        :description target_pose_wrist: Target pose for the wrist link of the robot arm corresponding to the target
                                        coordinates.
        :type target_pose_wrist: geometry_msgs/msg/PoseStamped.

        """
        target_x, target_y, target_z = target_coords
        target_pose_wrist = PoseStamped()
        target_pose_wrist.header.frame_id = WORLD_FRAME
        target_pose_wrist.header.stamp = rospy.get_rostime()
        target_pose_wrist.pose.position.x = target_x
        target_pose_wrist.pose.position.y = target_y
        target_pose_wrist.pose.position.z = target_z + WRIST_TO_FINGERTIPS_OFFSET

        # The z-axis of the wrist should be pointing down,
        # thus the rotation around x-axis
        target_orientation_euler = [pi, 0, 0]

        # The euler angles must be transformed to quaternions
        rot = Rotation.from_euler('xyz', target_orientation_euler, degrees=False)
        target_orientation_quat = rot.as_quat()
        target_pose_wrist.pose.orientation.x = target_orientation_quat[0]
        target_pose_wrist.pose.orientation.y = target_orientation_quat[1]
        target_pose_wrist.pose.orientation.z = target_orientation_quat[2]
        target_pose_wrist.pose.orientation.w = target_orientation_quat[3]

        return target_pose_wrist


    def go_home(self, execution_msg=None):
        """
        Function for moving both of the robot arms into their home positions on opening the grippers.

        INPUT PARAMETERS
        :description execution_msg: (Optional) text that is published as action feedback during the movement.
        :type execution_msg: String.
        """
        if execution_msg:
            self.publish_action_feedback(execution_msg)
        for arm in self.arms:
            arm.set_named_target("home")
            arm.go(wait=True)
        for gripper in self.grippers:
            gripper.set_named_target("open")
            gripper.go(wait=True)

    def go_to_target(self, target, execution_msg=None):
        """
        Function for going to a target given as an input argument. Takes into a consideration that
        not every target is reachable (in case of an unreachable target an exeption is raised).

        INPUT PARAMETERS
        :description target: The target pose for the wrist of the arm in use.
        :type target: geometry_msgs/msg/PoseStamped.

        INPUT PARAMETERS
        :description execution_msg: (Optional) text that is published as action feedback during the movement.
        :type execution_msg: String.
        """
        self.arm_to_use.set_pose_target(target)
        if self.path_constraints is not None:
            self.arm_to_use.set_path_constraints(self.path_constraints)
        plan_result = self.arm_to_use.plan()
        if plan_result[0]:
            if execution_msg:
                self.publish_action_feedback(execution_msg)
            self.arm_to_use.execute(plan_result[1])
            return
        else:
            self.publish_action_feedback("Unable to compute a movement plan to target position.")
            raise Exception("Unable to compute a movement plan to target position.")


    def open_gripper(self):
        """
        Function for opening the gripper.
        """
        self.gripper_to_use.set_named_target("open")
        self.gripper_to_use.go(wait=True)

    def grasp_with_gripper(self):
        """
        Function for grasping with the gripper.
        """
        self.gripper_to_use.set_named_target("grasp")
        self.gripper_to_use.go(wait=True)

    def close_gripper(self):
        """
        Function for closing the gripper.
        """
        self.gripper_to_use.set_named_target("close")
        self.gripper_to_use.go(wait=True)

    def pick_object(self, target_coords):
        """
        Function for executing a single pick-up task with predefined target.

        INPUT PARAMETERS
        :description target_coords: Target coordinates in format [x, y, z].
        :type target_coords: List (of floats).
        """
        if self.process_state == FIRST_TIME_PICKING_UP:
            execution_msg = "Picking up the box from its original location."

        elif self.process_state == SWITCHING:
            execution_msg = "Picking up the box from the drop-off location."
        else:
            self.logwarn("Function pick_object called during unexpected state of the process.")
            execution_msg = None


        target_coords_pre_grasp = target_coords.copy()
        target_coords_pre_grasp[2] += PRE_GRASP_OFFSET
        pre_grasp_target = self.get_wrist_target_pose(target_coords_pre_grasp)
        grasp_target = self.get_wrist_target_pose(target_coords)
        self.go_to_target(pre_grasp_target, execution_msg=execution_msg)
        self.open_gripper()
        ## Uncomment the line below to use path constraints that force the robot arm to keep the grasped object in
        ## vertical position during the transfer. Performance of any of the MoveIt's default planners/solvers does
        ## not seem to be good enough for this feature to function as desired (often the path with constrains is not
        ## found at all or at least finding it takes really long). If you uncomment this, also remember to uncomment
        ## the line responsible for clearing the constraints in place_object funtion.
        # self.set_orientation_constraints()
        self.go_to_target(grasp_target, execution_msg=execution_msg)
        self.grasp_with_gripper()
        self.box_attach_client(attach=True)
        self.go_to_target(pre_grasp_target, execution_msg="Grasped the box.")

    def set_orientation_constraints(self):
        """
        Function for formatting and setting the orientational constraints for the end effector of the robot arm
        transferring an object. After this function is called, every movement with the robot arm is trying to be executed
        in a way that the end effector rotates only around z axis (keeping the possible object attached to it vertical).
        To clear the constaintsset by this function, clear_constraints function needs to be called afterwards.

        Note: The usage of this function is not recommended  with any of the MoveIt's default planners/solvers since
        they seem to be lacking performance for constrained path planning/solving.
        """
        path_constraints = Constraints()
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "world"
        orientation_constraint.header.stamp = rospy.get_rostime()
        orientation_constraint.link_name = self.arm_to_use.get_end_effector_link()

        target_orientation_euler = [pi, 0, 0]
        rot = Rotation.from_euler('xyz', target_orientation_euler, degrees=False)
        target_orientation_quat = rot.as_quat()
        orientation_constraint.orientation.x = target_orientation_quat[0]
        orientation_constraint.orientation.y = target_orientation_quat[1]
        orientation_constraint.orientation.z = target_orientation_quat[2]
        orientation_constraint.orientation.w = target_orientation_quat[3]

        orientation_constraint.absolute_x_axis_tolerance = pi/10
        orientation_constraint.absolute_y_axis_tolerance = pi/10
        orientation_constraint.absolute_z_axis_tolerance = pi

        orientation_constraint.weight = 1.0

        path_constraints.orientation_constraints.append(orientation_constraint)
        self.path_constraints = path_constraints

    def clear_constraints(self):
        """
        Function for clearing the path contraints set with function set_orientation_constraints.
        """
        for arm in self.arms:
            arm.clear_path_constraints()
        self.path_constraints = None

    def place_object(self, target_coords, final_target=False):
        """
        Function for performing a single place task with predefined target.

        INPUT PARAMETERS
        :description target_coords: Target coordinates in format [x, y, z].
        :type target_coords: List (of floats).

        INPUT PARAMETERS
        :description final_target: Boolean telling if the object is placed at its final target with this call.
        :type final_target: Boo.
        """
        if self.process_state == SWITCHING:
            execution_msg = "Placing the box to the drop-off location."

        elif self.process_state == FINAL_PLACING:
            execution_msg = "Placing the box to its final location."
        else:
            self.logwarn("Function place_object called during unexpected state of the process.")
            execution_msg = None

        target_coords_pre_place = target_coords.copy()
        target_coords_pre_place[2] += PRE_GRASP_OFFSET
        pre_place_target = self.get_wrist_target_pose(target_coords_pre_place)
        place_target = self.get_wrist_target_pose(target_coords)
        self.go_to_target(pre_place_target, execution_msg=execution_msg)
        self.go_to_target(place_target, execution_msg=execution_msg)
        self.open_gripper()
        self.box_attach_client(attach=False)
        self.go_to_target(pre_place_target, execution_msg="Released the box.")
        ## Uncomment the line below if you also uncommented the line calling set_orientation_constraints function
        ## from pick_object function. Then the constrains that are set during the pick-up task will be cleared when they
        ## are no longer relevant since the object is already being released. As mentioned in the corresponding comment
        ## in pick_object function, using of path constraints is not recommended with any of the MoveIt's default
        ## planners/solvers since they seem to be lacking performance for constrained path planning/solving.
        # self.clear_constraints()
        self.go_home(execution_msg="Going home")
        if final_target:
            self.publish_action_feedback("Process finished.")

        
def main():
    
    try:
        motion_controller = MotionController()
        motion_controller.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()