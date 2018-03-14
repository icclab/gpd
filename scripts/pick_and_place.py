import rospy
import numpy as np
from pprint import pprint
from pyquaternion import Quaternion
from gpd.msg import GraspConfigList
from moveit_python import *
from moveit_msgs.msg import Grasp, PlaceLocation, MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA

grasps = []
formatted_grasps = []

def print_color(text):
    print str('\033[33m' + '\033[1m' + text + '\033[0m')


def grasp_callback(msg):
    global grasps
    grasps = msg.grasps
    print_color("Received new grasps")


def show_grasp_pose(publisher, grasp_pose):
    print_color("Arrow orientation:")
    pprint(grasp_pose.orientation)
    marker = Marker(
        type=Marker.ARROW,
        id=0,
        lifetime=rospy.Duration(30),
        pose=grasp_pose,
        scale=Vector3(0.03, 0.03, 0.03),
        header=Header(frame_id='head_camera_rgb_optical_frame'),
        color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
    publisher.publish(marker)


rospy.init_node("gpd_pick_and_place")
print_color("Waiting for grasps to arrive")
rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, grasp_callback)
marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)

# wait for grasps
# while len(grasps) == 0:
#	rospy.sleep(0.01)

rospy.sleep(1)

p = PickPlaceInterface("arm", "gripper", verbose=True)

for i in range(0, 5):
    g = Grasp()
    g.id = "dupa"
    gp = PoseStamped()
    gp.header.frame_id = "head_camera_rgb_optical_frame"
    gp.pose.position.x = grasps[i].surface.x
    gp.pose.position.y = grasps[i].surface.y
    gp.pose.position.z = grasps[i].surface.z

    r = np.array([[grasps[i].approach.x, grasps[i].approach.y, grasps[i].approach.z],
                  [grasps[i].binormal.x, grasps[i].binormal.y, grasps[i].binormal.z],
                  [grasps[i].axis.x, grasps[i].axis.y, grasps[i].axis.z]])

    quat = Quaternion(matrix=r)

    gp.pose.orientation.x = float(quat.elements[1])
    gp.pose.orientation.y = float(quat.elements[2])
    gp.pose.orientation.z = float(quat.elements[3])
    gp.pose.orientation.w = - float(quat.elements[0])

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

rospy.sleep(1)
print_color("Pick sequence started")

for single_grasp in formatted_grasps:
    show_grasp_pose(marker_publisher, single_grasp.grasp_pose.pose)

    print_color("Executing grasp: ")
    pprint(single_grasp.grasp_pose.pose)

    rospy.sleep(1)
    pick_result = p.pickup("obj", [single_grasp, ], planning_time=9001, support_name="<octomap>",
                           allow_gripper_support_collision=True)

    if pick_result.error_code.val != -1:
        print_color("Done")
        break
    print_color("Planer failed")

# rospy.spin()

# TODO
# l = PlaceLocation()
# fill l
# p.place("obj", [l, ], goal_is_eef=True, support_name="supp")
