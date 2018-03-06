import rospy
from gpd.msg import GraspConfigList
from moveit_python import *
from moveit_msgs.msg import Grasp, PlaceLocation
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Quaternion

grasps = []

def grasp_callback(msg):
	global grasps
	grasps = msg.grasps
	print "Received new grasps" 


rospy.init_node("gpd_pick_and_place")
print "Waiting for grasps to arrive"
rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, grasp_callback)

#wait for grasps
#while len(grasps) == 0:
#	rospy.sleep(0.01)

rospy.sleep(1)
p = PickPlaceInterface("arm", "gripper", verbose=True)
g = Grasp()

#g.id = "test"
gp = PoseStamped()
gp.header.frame_id = "head_camera_rgb_optical_frame"
gp.pose.position.x = grasps[0].surface.x
gp.pose.position.y = grasps[0].surface.y
gp.pose.position.z = grasps[0].surface.z

quat = quaternion_from_euler(grasps[0].approach.x, grasps[0].approach.y, grasps[0].approach.z)

gp.pose.orientation.x = 0.00455083281158 #float(quat[0]) 
gp.pose.orientation.y = 0.00615461540259 #float(quat[1])
gp.pose.orientation.z = 0.720337410234 #float(quat[2])
gp.pose.orientation.w = 0.693213973258 #float(quat[3])

g.grasp_pose = gp

g.pre_grasp_approach.direction.header.frame_id = "wrist_roll_link"
g.pre_grasp_approach.direction.vector.x = 1.0
g.pre_grasp_approach.direction.vector.y = 0.0
g.pre_grasp_approach.direction.vector.z = 0.0
g.pre_grasp_approach.min_distance = 0.01
g.pre_grasp_approach.desired_distance = 0.1

#g.pre_grasp_posture.header.frame_id = "head_camera_rgb_optical_frame"
g.pre_grasp_posture.joint_names = ["l_gripper_finger_joint", "r_gripper_finger_joint"]
pos = JointTrajectoryPoint()
pos.positions.append(0.055)
pos.positions.append(0.055)
g.pre_grasp_posture.points.append(pos)

#g.grasp_posture.header.frame_id = "head_camera_rgb_optical_frame"
g.grasp_posture.joint_names = ["l_gripper_finger_joint", "r_gripper_finger_joint"]
pos = JointTrajectoryPoint()
pos.positions.append(0.0)
pos.positions.append(0.0)
pos.effort.append(50.0)
pos.effort.append(50.0)
g.grasp_posture.points.append(pos)

g.allowed_touch_objects = []
g.max_contact_force = 0.0
g.grasp_quality = grasps[0].score.data

import pprint
pprint.pprint(g)

p.pickup("obj", [g, ], planning_time=9001)

rospy.spin()

#TODO
#l = PlaceLocation()
#fill l
#p.place("obj", [l, ], goal_is_eef=True, support_name="supp")
