import rospy
import numpy as np
import copy
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
from filter_scene_and_select_grasp import RobotPreparation, GpdGrasps
from moveit_python.geometry import rotate_pose_msg_by_euler_angles


class GpdPickPlace(object):
    grasps = []
    mark_pose = False
    grasp_offset = - 0.12

    def __init__(self, mark_pose=False):
        self.grasp_subscriber = rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList,
                                                 self.grasp_callback)
        if mark_pose:
            self.mark_pose = True
            self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        self.p = PickPlaceInterface(group="arm_torso", ee_group="gripper", verbose=True)

    def grasp_callback(self, msg):
        self.grasps = msg.grasps
        self.grasp_subscriber.unregister()
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
            header=Header(frame_id='xtion_rgb_optical_frame'),
            color=ColorRGBA(1.0, 1.0, 0.0, 0.8))
        publisher.publish(marker)

    def get_gpd_grasps(self):
        pevent("Waiting for grasps to arrive")
        while len(self.grasps) == 0:
            rospy.sleep(0.01)
        return self.grasps

    def generate_grasp_msgs(self, grasps):
        formatted_grasps = []
        for i in range(0, 5):
            g = Grasp()
            g.id = "dupa"
            gp = PoseStamped()
            gp.header.frame_id = "xtion_rgb_optical_frame"

            # Move grasp back for given offset
            gp.pose.position.x = grasps[i].surface.x + self.grasp_offset * grasps[i].approach.x
            gp.pose.position.y = grasps[i].surface.y + self.grasp_offset * grasps[i].approach.y
            gp.pose.position.z = grasps[i].surface.z + self.grasp_offset * grasps[i].approach.z

            quat = self.trans_matrix_to_quaternion(grasps[i])

            gp.pose.orientation.x = float(quat.elements[1])
            gp.pose.orientation.y = float(quat.elements[2])
            gp.pose.orientation.z = float(quat.elements[3])
            gp.pose.orientation.w = - float(quat.elements[0])  # ??

            g.grasp_pose = gp

            g.pre_grasp_approach.direction.header.frame_id = "gripper_link"
            g.pre_grasp_approach.direction.vector.x = 1.0
            g.pre_grasp_approach.direction.vector.y = 0.0
            g.pre_grasp_approach.direction.vector.z = 0.0
            g.pre_grasp_approach.min_distance = 0.03
            g.pre_grasp_approach.desired_distance = 0.20

            # g.pre_grasp_posture.joint_names = ["gripper_right_finger_joint", "gripper_left_finger_joint"]
            # pos = JointTrajectoryPoint()
            # pos.positions.append(0.044)
            # pos.positions.append(0.044)
            # g.pre_grasp_posture.points.append(pos)

            g.grasp_posture.joint_names = ["gripper_right_finger_joint", "gripper_left_finger_joint"]
            pos = JointTrajectoryPoint()
            pos.positions.append(0.0)
            pos.positions.append(0.0)
            pos.accelerations.append(0.0)
            pos.accelerations.append(0.0)
            # pos.effort.append(0.0)
            # pos.effort.append(0.0)
            g.grasp_posture.points.append(pos)

            g.allowed_touch_objects = ["<octomap>"]
            g.max_contact_force = 0.0
            g.grasp_quality = grasps[0].score.data

            formatted_grasps.append(g)
        return formatted_grasps

    def trans_matrix_to_quaternion(self, grasp):
        r = np.array([[grasp.approach.x, grasp.approach.y, grasp.approach.z],
                      [grasp.binormal.x, grasp.binormal.y, grasp.binormal.z],
                      [grasp.axis.x, grasp.axis.y, grasp.axis.z]])
        return Quaternion(matrix=r)

    def pick(self, grasps_list, verbose=False):
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

    def place(self, place_pose):
        pevent("Place sequence started")

        places = self.generate_place_poses(place_pose)

        place_result = self.p.place_with_retry("obj", places, support_name="<octomap>",# planner_id="gripper",
                                               planning_time=9001, goal_is_eef=True)

        pevent("Planner returned: " + get_moveit_error_code(place_result.error_code.val))

    def generate_place_poses(self, initial_place_pose):
        places = list()

        l = PlaceLocation()
        l.id = "dupadupa"
        l.place_pose.header.frame_id = "xtion_rgb_optical_frame"

        # Whats happening here?
        # Explanation: during grasp msg generation the grasp pose is moved back of the grasp_offset. Thats because
        # moveit performs grasp until the gripper origin (wrist_roll_link) and gdp returns actual grasp pose (where the
        # fingers should be closed)... Now to generate bunch of different orientations for planner by rotating original
        # pose I need to go back to the original position. Basically I want just to rotate around place pose,
        # not around the robot wrist

        # Find original grasp pose comparing its orientation initial_place_pose
        pose_id = 1337
        for i in range(0, 5):
            q = self.trans_matrix_to_quaternion(self.grasps[i])
            if (initial_place_pose.grasp_pose.pose.orientation.x == float(q.elements[1]) and
                    initial_place_pose.grasp_pose.pose.orientation.y == float(q.elements[2]) and
                    initial_place_pose.grasp_pose.pose.orientation.z == float(q.elements[3]) and
                    initial_place_pose.grasp_pose.pose.orientation.w == - float(q.elements[0])):  # minus
                pose_id = i

        # Load original grasp pose position...
        l.place_pose.pose.position.x = self.grasps[pose_id].surface.x
        l.place_pose.pose.position.y = self.grasps[pose_id].surface.y
        l.place_pose.pose.position.z = self.grasps[pose_id].surface.z

        # ... and orientation
        q = self.trans_matrix_to_quaternion(self.grasps[pose_id])
        l.place_pose.pose.orientation.x = float(q.elements[1])
        l.place_pose.pose.orientation.y = float(q.elements[2])
        l.place_pose.pose.orientation.z = float(q.elements[3])
        l.place_pose.pose.orientation.w = - float(q.elements[0])  # don't forget the minus sign

        # Move 50cm to the right
        l.place_pose.pose.position.x += 0.2

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
        planning = PlanningSceneInterface("xtion_rgb_optical_frame")

        obj_pose = Pose()
        obj_pose.position.x = 0
        obj_pose.position.y = 0
        obj_pose.position.z = 0
        obj_pose.orientation.x = 0
        obj_pose.orientation.y = 0
        obj_pose.orientation.z = 0
        obj_pose.orientation.w = 1

        planning.addMesh("obj", obj_pose, "object.stl", wait=True)


if __name__ == "__main__":
    rospy.init_node("gpd_pick_and_place")

    # Tilt the head down to see the table
    robot = RobotPreparation()
    robot.look_down()
    # robot.lift_torso()
    robot.unfold_arm()

    # Subscribe for grasps
    pnp = GpdPickPlace(mark_pose=True)

    # Get the pointcloud from camera, filter it, extract indices and publish it to gpd CNN
    gpd_prep = GpdGrasps(max_messages=8)
    gpd_prep.filter_cloud()
    gpd_prep.publish_indexed_cloud()

    # Wait for grasps from gpd, wrap them into Grasp msg format and start picking
    selected_grasps = pnp.get_gpd_grasps()
    formatted_grasps = pnp.generate_grasp_msgs(selected_grasps)
    successful_grasp = pnp.pick(formatted_grasps, verbose=True)

    # Place object with successful grasp pose as the starting point
    pnp.place(successful_grasp)
