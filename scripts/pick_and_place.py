import rospy
from gpd.msg import GraspConfigList
from moveit_python import *
from moveit_msgs.msg import Grasp, PlaceLocation
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA

grasps = []
formatted_grasps = []


def print_color(text):
	print str('\033[33m' + '\033[1m'  + text + '\033[0m')

def grasp_callback(msg):
	global grasps
	grasps = msg.grasps
	print_color("Received new grasps")

def show_grasp_pose(publisher, pos, quat):
	marker = Marker(
                type=Marker.ARROW,
                id=0,
                lifetime=rospy.Duration(30),
                pose=Pose(Point(pos.x, pos.y, pos.z), Quaternion(quat[0], quat[1], quat[2], quat[3])),
                scale=Vector3(0.03, 0.03, 0.03),
                header=Header(frame_id='d_camera_rgb_optical_frame'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
	publisher.publish(marker)


rospy.init_node("gpd_pick_and_place")
print_color("Waiting for grasps to arrive")
rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, grasp_callback)
marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)


#wait for grasps
#while len(grasps) == 0:
#	rospy.sleep(0.01)

rospy.sleep(1)

p = PickPlaceInterface("arm", "gripper", verbose=True)
g = Grasp()

for i in range(0, 5):
	g.id = "dupa"
	gp = PoseStamped()
	gp.header.frame_id = "head_camera_rgb_optical_frame"
	gp.pose.position.x = grasps[i].surface.x
	gp.pose.position.y = grasps[i].surface.y
	gp.pose.position.z = grasps[i].surface.z

	quat = quaternion_from_euler(grasps[i].approach.x, grasps[i].approach.y, grasps[i].approach.z)

	gp.pose.orientation.x = float(quat[0]) 
	gp.pose.orientation.y = float(quat[1])
	gp.pose.orientation.z = float(quat[2])
	gp.pose.orientation.w = float(quat[3])

	g.grasp_pose = gp

	g.pre_grasp_approach.direction.header.frame_id = "wrist_roll_link"
	g.pre_grasp_approach.direction.vector.x = 1.0
	g.pre_grasp_approach.direction.vector.y = 0.0
	g.pre_grasp_approach.direction.vector.z = 0.0
	g.pre_grasp_approach.min_distance = 0.01
	g.pre_grasp_approach.desired_distance = 0.1

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

#print_color("Goal pose:")
#import pprint
#pprint.pprint(g.grasp_pose.pose)

show_grasp_pose(marker_publisher, grasps[0].surface, quat)
rospy.sleep(1)
print_color("Pick sequence started")
pick_result = p.pickup("obj", formatted_grasps, planning_time=9001)
print_color("Done")
#rospy.spin()

#TODO
#l = PlaceLocation()
#fill l
#p.place("obj", [l, ], goal_is_eef=True, support_name="supp")
