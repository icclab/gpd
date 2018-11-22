import rospy
import numpy as np
import copy
import tf
import datetime
import gc
from tools import *
from pprint import pprint
from pyquaternion import Quaternion
from gpd.msg import GraspConfigList
from moveit_python import *
from moveit_msgs.msg import Grasp, PlaceLocation
from geometry_msgs.msg import PoseStamped, Vector3, Pose
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from gpd_controller_2cameras import GpdGrasps
from robot_controller import RobotPreparation
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from tf.transformations import *

import geometry_msgs.msg #for pose2 simple
import math
from tools import *
from pprint import pprint
from pyquaternion import Quaternion
from gpd.msg import GraspConfigList
from moveit_python import *
from moveit_msgs.msg import Grasp, PlaceLocation
from geometry_msgs.msg import PoseStamped, Vector3, Pose
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from tf.msg import tfMessage
from niryo_one_python_api.niryo_one_api import *
import time


class GpdPickPlace(object):
    grasps = []
    mark_pose = False
    #grasp_offset = -0.15
    grasp_offset = -0.1


    def __init__(self, mark_pose=False):
        self.grasp_subscriber = rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList,
                                                 self.grasp_callback)
        if mark_pose:
            self.mark_pose = True
            self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        self.p = PickPlaceInterface(group="arm", ee_group="tool", verbose=True)
        self.tf = tf.TransformListener()

    def grasp_callback(self, msg):
        self.grasps = msg.grasps
        self.grasp_subscriber.unregister()
       # frame_id = msg.header.frame_id
        pevent("Received new grasps")

    def show_grasp_pose(self, publisher, grasp_pose):
        # pinfo("Marker orientation:")
        # pprint(grasp_pose.orientation)
        marker = Marker(
            type=Marker.ARROW,
            id=0,
            lifetime=rospy.Duration(30),
            pose=grasp_pose,
            scale=Vector3(0.03, 0.02, 0.02),
            header=Header(frame_id='camera_depth_optical_frame'),
            color=ColorRGBA(1.0, 1.0, 0.0, 0.8))
        publisher.publish(marker)

    def get_gpd_grasps(self):
        pevent("Waiting for grasps to arrive")
        while len(self.grasps) == 0:
            rospy.sleep(0.01)
        return self.grasps

    def generate_grasp_msgs(self, grasps):
        formatted_grasps = []
        for i in range(0, len(grasps)): #dimitris, take out self. !
            g = Grasp()
            g.id = "dupa"
            gp = PoseStamped()
            gp.header.frame_id = "camera_depth_optical_frame"

            org_q = self.trans_matrix_to_quaternion(grasps[i])
            #   rot_q = Quaternion(0.7071, 0.7071, 0, 0)  # 90* around X axis (W, X, Y, Z)
            #  quat = rot_q * org_q

            quat = org_q


            # Move grasp back for given offset
            gp.pose.position.x = grasps[i].surface.x + self.grasp_offset * grasps[i].approach.x
            gp.pose.position.y = grasps[i].surface.y + self.grasp_offset * grasps[i].approach.y
            gp.pose.position.z = grasps[i].surface.z + self.grasp_offset * grasps[i].approach.z
        
            gp.pose.orientation.x = float(quat.elements[1])
            gp.pose.orientation.y = float(quat.elements[2])
            gp.pose.orientation.z = float(quat.elements[3])
            gp.pose.orientation.w = float(quat.elements[0])

            g.grasp_pose = gp

            g.pre_grasp_approach.direction.header.frame_id = "hand_link"
            g.pre_grasp_approach.direction.vector.x = 0.0
            g.pre_grasp_approach.direction.vector.y = 0.0
            g.pre_grasp_approach.direction.vector.z = 1.0
            g.pre_grasp_approach.min_distance = 0.05
            g.pre_grasp_approach.desired_distance = 0.1

         #   g.pre_grasp_posture.joint_names = ["gripper_right_finger_joint", "gripper_left_finger_joint"]
            g.pre_grasp_posture.joint_names = ["joint_6"]
            g.pre_grasp_posture.header.frame_id = "hand_link"
            pos = JointTrajectoryPoint()
            pos.positions.append(0)
          #  pos.positions.append(0.1337)
            g.pre_grasp_posture.points.append(pos)

          #  g.grasp_posture.joint_names = ["gripper_right_finger_joint", "gripper_left_finger_joint"]
          #  g.grasp_posture.joint_names = ["joint_6"]
          #  pos = JointTrajectoryPoint()
          #  pos.positions.append(0.0)
          #  pos.positions.append(0.0)
          #  pos.accelerations.append(0.0)
          #  pos.accelerations.append(0.0)
          #  g.grasp_posture.points.append(pos)
          #  g.grasp_posture.header.frame_id = "hand_link"

            g.allowed_touch_objects = ["<octomap>", "obj"]
            g.max_contact_force = 0.0
            #g.grasp_quality = grasps[0].score.data  perche 0 e non i????
            g.grasp_quality = grasps[i].score.data

            formatted_grasps.append(g)
        return formatted_grasps

    def trans_matrix_to_quaternion(self, grasp):
        r = np.array([[grasp.approach.x, grasp.binormal.x, grasp.axis.x],
                      [grasp.approach.y, grasp.binormal.y, grasp.axis.y],
                      [grasp.approach.z, grasp.binormal.z, grasp.axis.z]])
        return Quaternion(matrix=r)

    def pick(self, grasps_list, verbose=False):
        failed_grasps = 0
        pevent("Pick sequence started")

        # Add object mesh to planning scene
        self.add_object_mesh()

        for single_grasp in grasps_list:
            if self.mark_pose:
                self.show_grasp_pose(self.marker_publisher, single_grasp.grasp_pose.pose)
                rospy.sleep(1)

            if verbose:
                pevent("Executing grasp: ")
                pprint(single_grasp.grasp_pose.pose)

            pick_result = self.p.pickup("obj", [single_grasp, ], planning_time=9001, support_name="<octomap>",
                                        allow_gripper_support_collision=True)

            pevent("Planner returned: " + get_moveit_error_code(pick_result.error_code.val))

            if pick_result.error_code.val == 1:
                pevent("Grasp successful!")
                return single_grasp
            else:
                failed_grasps += 1
                if failed_grasps == 5:
                    pevent("All grasps failed. Aborting")
                    exit(1)

    def place2(self, place_pose):
        pevent("Place sequence started")
        group_name = "arm"

        group = moveit_commander.MoveGroupCommander(group_name)


        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.24
        pose_goal.position.y = 0.22
        pose_goal.position.z = 0.31
        #pose_goal.orientation.w = place_pose.grasp_pose.pose.orientation.w
        #pose_goal.position.x = place_pose.grasp_pose.pose.orientation.x
        #pose_goal.position.y = place_pose.grasp_pose.pose.orientation.y
        #pose_goal.position.z = place_pose.grasp_pose.pose.orientation.z
        group.set_pose_target(pose_goal)

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        # group.go(joint_goal, wait=True)

        plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()

        group.clear_pose_targets()




    def place(self, place_pose):
        pevent("Place sequence started")


        places = self.generate_place_poses(place_pose)

        place_result = self.p.place_with_retry("obj", places, support_name="<octomap>", planning_time=9001,
                                  goal_is_eef=True)

               # pevent("Planner returned: " + get_moveit_error_code(place_result.error_code.val))

    def generate_place_poses(self, initial_place_pose):
        places = list()

        l = PlaceLocation()
        l.id = "dupadupa"
        l.place_pose.header.frame_id = "camera_depth_optical_frame"


        q = Quaternion(initial_place_pose.grasp_pose.pose.orientation.w,
                        initial_place_pose.grasp_pose.pose.orientation.x,
                        initial_place_pose.grasp_pose.pose.orientation.y,
                        initial_place_pose.grasp_pose.pose.orientation.z)

# Load successful grasp pose
        l.place_pose.pose.position = initial_place_pose.grasp_pose.pose.position
        l.place_pose.pose.orientation.w = q.elements[0]
        l.place_pose.pose.orientation.x = q.elements[1]
        l.place_pose.pose.orientation.y = q.elements[2]
        l.place_pose.pose.orientation.z = q.elements[3]

# Move 20cm to the right
        l.place_pose.pose.position.y += 0.2

# Fill rest of the msg with some data
        l.post_place_posture = initial_place_pose.grasp_posture
        l.post_place_retreat = initial_place_pose.post_grasp_retreat
        l.pre_place_approach = initial_place_pose.pre_grasp_approach

        places.append(copy.deepcopy(l))

# Rotate place pose to generate more possible configurations for the planner
        m = 16  # Number of possible place poses
        for i in range(0, m - 1):
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * math.pi / m)
            places.append(copy.deepcopy(l))

        return places

    def add_object_mesh(self):
        planning = PlanningSceneInterface("camera_depth_optical_frame")

        obj_pose = Pose()
        obj_pose.position.x = 0
        obj_pose.position.y = 0
        obj_pose.position.z = 0
        obj_pose.orientation.x = 0
        obj_pose.orientation.y = 0
        obj_pose.orientation.z = 0
        obj_pose.orientation.w = 1
        planning.addMesh("obj", obj_pose, "object.stl")
    #    rospy.sleep(3.14)
    #    pprint(planning.getKnownCollisionObjects())

    def get_know_successful_grasp(self):
        g = Grasp()
        g.id = "successful_predefined_grasp"
        gp = PoseStamped()
        gp.header.frame_id = "camera_depth_optical_frame"

        gp.pose.position.x = 0.183518647951
        gp.pose.position.y = -0.23707952283
        gp.pose.position.z = 0.493978534979

        gp.pose.orientation.w = -0.604815599864
        gp.pose.orientation.x = -0.132654186819
        gp.pose.orientation.y = 0.698958888788
        gp.pose.orientation.z = -0.357851126398

        g.grasp_pose = gp

        return g
if __name__ == "__main__":
    start_time = datetime.datetime.now()
    rospy.init_node("gpd_pick_and_place")

    print("--- Start Physical Arm ---")

   # n = NiryoOne()

 #   print("Calibration started !")
    # Calibrate robot first
#try:
 #   n.calibrate_auto()
#except NiryoOneException as e:
#    print e

    print("Make sure calibration is already performed on arm !")
    time.sleep(1)
    # Test learning mode
    #   n.activate_learning_mode(False)

    # Test gripper 3
 #   n.change_tool(TOOL_GRIPPER_3_ID)

    # testing to add a box at the eef to simulate a gripper
 #   robot = moveit_commander.RobotCommander()
 #   scene = moveit_commander.PlanningSceneInterface()
 #   group_name = "arm"
 #   group = moveit_commander.MoveGroupCommander(group_name)
    # We can get the name of the reference frame for this robot:
#    planning_frame = group.get_planning_frame()
#    print("============ Reference frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
#    eef_link = group.get_end_effector_link()
#    print("============ End effector: %s" % eef_link)

    # We can get a list of all the groups in the robot:
#    group_names = robot.get_group_names()
#    print("============ Robot Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
#    print("============ Printing robot state")
#    print(robot.get_current_state())
#    print("")
    num_objects = 2
    for i in range (0, num_objects):

        # Subscribe for grasps
        pnp = GpdPickPlace(mark_pose=True)


         # Get the pointcloud from camera, filter it, extract indices and publish it to gpd CNN
        gpd_prep = GpdGrasps(max_messages=10)
        gpd_prep.filter_cloud()
        gpd_prep.publish_indexed_cloud()


        # Wait for grasps from gpd, wrap them into Grasp msg format and start picking
        selected_grasps = pnp.get_gpd_grasps()
        formatted_grasps = pnp.generate_grasp_msgs(selected_grasps)
        #n.open_gripper(TOOL_GRIPPER_3_ID, 200)
        print("Gripper 3 opened")
        successful_grasp = pnp.pick(formatted_grasps, verbose=True)
        #n.close_gripper(TOOL_GRIPPER_3_ID, 200)
        print("Gripper 3 closed")


        # Place object with successful grasp pose as the starting point
        pnp.place2(successful_grasp)
      #  n.open_gripper(TOOL_GRIPPER_3_ID, 200)
        print("Gripper 3 opened")
      #  n.close_gripper(TOOL_GRIPPER_3_ID, 200)
        print("Gripper 3 closed")
   # pnp.place(successful_grasp)
  #  fix_grasp =  pnp.get_know_successful_grasp()
   # pnp.place(fix_grasp)
    pinfo("Demo runtime: " + str(datetime.datetime.now() - start_time))


