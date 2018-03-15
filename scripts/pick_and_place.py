import rospy
import numpy as np
from tools import *
from pprint import pprint
from pyquaternion import Quaternion
from gpd.msg import GraspConfigList
from moveit_python import *
from moveit_msgs.msg import Grasp
from geometry_msgs.msg import PoseStamped, Vector3
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from filter_scene_and_select_grasp import PointHeadClient, GpdGrasps


class GpdPickPlace(object):
    grasps = []
    mark_pose = False

    def __init__(self, mark_pose=False):
        rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, self.grasp_callback)
        if mark_pose:
            self.mark_pose = True
            self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        self.p = PickPlaceInterface("arm", "gripper", verbose=True)

    def grasp_callback(self, msg):
        self.grasps = msg.grasps
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
            header=Header(frame_id='head_camera_rgb_optical_frame'),
            color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
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
            gp.header.frame_id = "head_camera_rgb_optical_frame"
            gp.pose.position.x = grasps[i].surface.x
            gp.pose.position.y = grasps[i].surface.y
            gp.pose.position.z = grasps[i].surface.z

            quat = self.trans_matrix_to_quaternion(grasps, i)

            gp.pose.orientation.x = float(quat.elements[1])
            gp.pose.orientation.y = float(quat.elements[2])
            gp.pose.orientation.z = float(quat.elements[3])
            gp.pose.orientation.w = - float(quat.elements[0])  # ??

            g.grasp_pose = gp

            g.pre_grasp_approach.direction.header.frame_id = "gripper_link"
            g.pre_grasp_approach.direction.vector.x = 1.0
            g.pre_grasp_approach.direction.vector.y = 0.0
            g.pre_grasp_approach.direction.vector.z = 0.0
            g.pre_grasp_approach.min_distance = 0.07
            g.pre_grasp_approach.desired_distance = 0.20

            g.pre_grasp_posture.joint_names = ["l_gripper_finger_joint", "r_gripper_finger_joint"]
            pos = JointTrajectoryPoint()
            pos.positions.append(0.055)
            pos.positions.append(0.055)
            g.pre_grasp_posture.points.append(pos)

            g.grasp_posture.joint_names = ["l_gripper_finger_joint", "r_gripper_finger_joint"]
            pos = JointTrajectoryPoint()
            pos.positions.append(0.0)
            pos.positions.append(0.0)
            pos.effort.append(50.0)
            pos.effort.append(50.0)
            g.grasp_posture.points.append(pos)

            g.allowed_touch_objects = ["<octomap>"]
            g.max_contact_force = 0.0
            g.grasp_quality = grasps[0].score.data

            formatted_grasps.append(g)
        return formatted_grasps

    @staticmethod
    def trans_matrix_to_quaternion(grasps, i):
        r = np.array([[grasps[i].approach.x, grasps[i].approach.y, grasps[i].approach.z],
                      [grasps[i].binormal.x, grasps[i].binormal.y, grasps[i].binormal.z],
                      [grasps[i].axis.x, grasps[i].axis.y, grasps[i].axis.z]])
        return Quaternion(matrix=r)

    def pick(self, grasps_list, verbose=False):
        pevent("Pick sequence started")

        for single_grasp in grasps_list:
            if self.mark_pose:
                self.show_grasp_pose(self.marker_publisher, single_grasp.grasp_pose.pose)

            if verbose:
                pevent("Executing grasp: ")
                pprint(single_grasp.grasp_pose.pose)

            rospy.sleep(1)
            pick_result = self.p.pickup("obj", [single_grasp, ], planning_time=9001, support_name="<octomap>",
                                        allow_gripper_support_collision=True)

            pevent("Planner returned: " + get_moveit_error_code(pick_result.error_code.val))

            if pick_result.error_code.val == 1 \
                    or pick_result.error_code.val == -7:
                pevent("Done")
                break


if __name__ == "__main__":
    rospy.init_node("gpd_pick_and_place")

    # Tilt the head down to see the table
    head = PointHeadClient()
    pevent("Moving head")
    head.look_at(2.0, 0.0, 0.0, "base_link")

    # Get the pointcloud from camera, filter it, extract indices and publish it to gpd CNN
    gpd_prep = GpdGrasps()
    gpd_prep.filter_cloud()
    gpd_prep.publish_cloud_indexed()

    pnp = GpdPickPlace(mark_pose=False)
    selected_grasps = pnp.get_gpd_grasps()
    formatted_grasps = pnp.generate_grasp_msgs(selected_grasps)
    pnp.pick(formatted_grasps, verbose=True)

    # pnp.pick(pnp.generate_grasp_msgs(pnp.get_gpd_grasps()), verbose=True)

# rospy.spin()

# TODO
# l = PlaceLocation()
# fill l
# p.place("obj", [l, ], goal_is_eef=True, support_name="supp")
