import rospy
import ipdb
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
from gpd_controller_summit import GpdGrasps
from robot_controller import RobotPreparation
from moveit_python.geometry import rotate_pose_msg_by_euler_angles, translate_pose_msg
from tf.transformations import *

import geometry_msgs.msg #for pose2 simple
import math
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from tf.msg import tfMessage
#from niryo_one_python_api.niryo_one_api import *
import time

from send_gripper import gripper_client
from send_gripper import gripper_client_2

from tf import TransformListener
import copy
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
import actionlib

#import roslib
#roslib.load_manifest('joint_states_listener')
#from joint_states_listener.srv import ReturnJointStates


#def call_return_joint_states(joint_names):
 #   rospy.wait_for_service("return_joint_states")
 #   try:
  #      s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
   #     resp = s(joint_names)
   # except rospy.ServiceException, e:
   #     print "error when calling return_joint_states: %s"%e
   #     sys.exit(1)
   # for (ind, joint_name) in enumerate(joint_names):
   #     if(not resp.found[ind]):
   #         print "joint %s not found!"%joint_name
   # return (resp.position, resp.velocity, resp.effort)

    # pretty-print list to string
#def pplist(list):
 #   return ' '.join(['%2.3f' % x for x in list])

JOINT_NAMES = ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_joint',
               'arm_wrist_1_joint', 'arm_wrist_2_joint', 'arm_wrist_3_joint']
#Q1 = [-0.472325325012, 0.06132878988980,-0.85,0.7074,-0.707,0]
#Q2 = [-0.472325325012, 0.06132878988980,-0.85,-0.707, 0.707,0]
#Q3 = [-0.472325325012, 0.06132878988980,-0.85,-1.141,-1.141,0]
#Q4 = [0.472325325012, 0.06132878988980,-0.85,0.7074,-0.707,0]
#Q5 = [0.472325325012, 0.06132878988980,-0.85,-0.707, 0.707,0]
#Q6 = [0.472325325012, 0.06132878988980,-0.85,-1.14,-1.14,0]

client = None

class GpdPickPlace(object):
    grasps = []
    mark_pose = False
    #grasp_offset = -0.15
    grasp_offset = -0.1


    def __init__(self, mark_pose=False):
        self.grasp_subscriber = rospy.Subscriber("/summit_xl/detect_grasps/clustered_grasps", GraspConfigList,
                                                 self.grasp_callback)

        if mark_pose:
            self.mark_pose = True
            self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)

       # ipdb.set_trace()
        self.p = PickPlaceInterface(group="manipulator", ee_group="endeffector", verbose=True, ns="/summit_xl/")
        self.tf = tf.TransformListener()
    def grasp_callback(self, msg):
        self.grasps = msg.grasps
        self.grasp_subscriber.unregister()
       # frame_id = msg.header.frame_id
        pevent("Received new grasps")

    def show_grasp_pose(self, publisher, grasp_pose):
        #ipdb.set_trace()
        marker_x = Marker(
            type=Marker.ARROW,
            id=0,
            lifetime=rospy.Duration(60),
            pose=grasp_pose,
            scale=Vector3(0.04, 0.02, 0.02),
            header=Header(frame_id='arm_camera_depth_optical_frame'),
            color=ColorRGBA(1.0, 0.0, 0.0, 0.8))
        publisher.publish(marker_x)

    def get_gpd_grasps(self):
        pevent("Waiting for grasps to arrive")
        while len(self.grasps) == 0:
            rospy.sleep(0.01)
        return self.grasps

    def tf_listen(self):
        if self.tf.frameExists("arm_camera_depth_optical_frame") and self.tf.frameExists("gripper_base_link"):
            t = self.tf.getLatestCommonTime("arm_camera_depth_optical_frame", "gripper_base_link")
            position, quaternion = self.tf.lookupTransform("arm_camera_depth_optical_frame", "gripper_base_link", t)
            print position, quaternion
            #p1 = geometry_msgs.msg.PoseStamped()
            #p1.header.frame_id = "gripper_base_link"
            #p1.pose.orientation.w = 1.0  # Neutral orientation
            #p_in_base = self.tf_listener_.transformPose("arm_camera_depth_optical_frame", p1)
            #print "Position "
            #print p_in_base
            return quaternion


    def generate_grasp_msgs(self, grasps):
        formatted_grasps = []
        for i in range(0, len(grasps)): #dimitris, take out self. !
            g = Grasp()
            g.id = "dupa"
            gp = PoseStamped()
            gp.header.frame_id = "arm_camera_depth_optical_frame"
            org_q = self.trans_matrix_to_quaternion(grasps[i])
            rot_z_q = Quaternion(-0.7071, 0, 0, 0.7071) #270* around Z axis (W, X, Y, Z)

            #code for dynamic tf listening and transformation
           # self.tf_listener_ = TransformListener()
           # dyn_rot = self.tf_listen()
            #dyn_rot_pyquaternion = copy.deepcopy(dyn_rot)
            #dyn_rot_pyquaternion[0] = copy.deepcopy(dyn_rot[3])
            #dyn_rot_pyquaternion[1] = copy.deepcopy(dyn_rot[0])
            #dyn_rot_pyquaternion[2] = copy.deepcopy(dyn_rot[1])
            #dyn_rot_pyquaternion[3] = copy.deepcopy(dyn_rot[2])
          #  quat1 = org_q * dyn_rot_pyquaternion
            #quat =  quat1 * rot_z_q

            quat = org_q #* rot_z_q

            # Move grasp back for given offset (and rotation of 270 degrees around z)

            gp.pose.position.x = grasps[i].surface.x + self.grasp_offset * grasps[i].approach.x
            gp.pose.position.y = grasps[i].surface.y + self.grasp_offset * grasps[i].approach.y
            gp.pose.position.z = grasps[i].surface.z + self.grasp_offset * grasps[i].approach.z

            gp.pose.orientation.x = float(quat.elements[1])
            gp.pose.orientation.y = float(quat.elements[2])
            gp.pose.orientation.z = float(quat.elements[3])
            gp.pose.orientation.w = float(quat.elements[0])
            #library used is pyquaternion http://kieranwynn.github.io/pyquaternion/

            g.grasp_pose = gp

            g.pre_grasp_approach.direction.header.frame_id = "arm_ee_link"
            g.pre_grasp_approach.direction.vector.z = 1.0
            g.pre_grasp_approach.min_distance = 0.06
            g.pre_grasp_approach.desired_distance = 0.1

         #   g.pre_grasp_posture.joint_names = ["gripper_right_finger_joint", "gripper_left_finger_joint"]
         #   g.pre_grasp_posture.joint_names = ["arm_tool0"]
       #     g.pre_grasp_posture.header.frame_id = "arm_wrist_3_link"
        #    pos = JointTrajectoryPoint()
        #    pos.positions.append(0)
         #   pos.positions.append(0.1337)
         #   g.pre_grasp_posture.points.append(pos)

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
            g.grasp_quality = grasps[0].score.data

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
        #ipdb.set_trace()
        # Add object mesh to planning scene
        self.add_object_mesh()

        for single_grasp in grasps_list:
            if self.mark_pose:
               # ipdb.set_trace()
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
                if failed_grasps == 10:
                    pevent("All grasps failed. Aborting")
                    exit(1)

    def place2(self, place_pose):
        pevent("Place sequence started")
        #group_name = "manipulator"
        # ipdb.set_trace()
        #group = moveit_commander.MoveGroupCommander(group_name, robot_description="/summit_xl/robot_description",
         #                                           ns="summit_xl")
        # group.set_planner_id("PRMkConfigDefault")

        pose_goal = geometry_msgs.msg.Pose()


        pose_goal.position.x = 0.516344249249
        pose_goal.position.y = -0.636391639709
        pose_goal.position.z =   0.603573918343
        pose_goal.orientation.w = 0.185884654522
        pose_goal.orientation.x = -0.681892871857
        pose_goal.orientation.y = 0.682668983936
        pose_goal.orientation.z = 0.185558646917
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
        l.place_pose.header.frame_id = "arm_camera_depth_optical_frame"


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
        planning = PlanningSceneInterface("arm_camera_depth_optical_frame", ns="/summit_xl/")

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
        gp.header.frame_id = "arm_camera_depth_optical_frame"

        gp.pose.position.x = 0.183518647951
        gp.pose.position.y = -0.23707952283
        gp.pose.position.z = 0.493978534979

        gp.pose.orientation.w = -0.604815599864
        gp.pose.orientation.x = -0.132654186819
        gp.pose.orientation.y = 0.698958888788
        gp.pose.orientation.z = -0.357851126398

        g.grasp_pose = gp

        return g



    def initial_octomap_building(self):
        global client

        # pose_current = [group.get_current_pose().pose.position.x, group.get_current_pose().pose.position.y, group.get_current_pose().pose.position.z, group.get_current_pose().pose.orientation.x, group.get_current_pose().pose.orientation.y, group.get_current_pose().pose.orientation.z]

        group_current_pose = group.get_current_pose().pose
        print("Initial position:")
        print(group_current_pose)
        oct_poses = list()
        Q = []
        oct_poses.append(copy.deepcopy(group_current_pose))

    # Rotate place pose to generate more possible configurations for the planner
        octomap_rotation_angles = 16  # Number of possible place poses
        octomap_translations_x = 0.05
        octomap_translations_y = 0.05
        octomap_translations_z = 0.05
        try:
         client = actionlib.SimpleActionClient('/summit_xl/arm_pos_based_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
         print "Waiting for server..."
         client.wait_for_server()
         print "Connected to server"

         g = FollowJointTrajectoryGoal()
         g.trajectory = JointTrajectory()
         g.trajectory.joint_names = JOINT_NAMES
       #  ipdb.set_trace()


         for i in range(0, octomap_rotation_angles - 1):
            Q.append([])
            Q[i].append(group_current_pose.position.x)
            Q[i].append(group_current_pose.position.y)
            Q[i].append(group_current_pose.position.z)
            Q[i].append(group_current_pose.orientation.x)
            Q[i].append(group_current_pose.orientation.y)
            Q[i].append(group_current_pose.orientation.z)
            group_current_pose = rotate_pose_msg_by_euler_angles(group_current_pose, 2 * math.pi / octomap_rotation_angles, 2 * math.pi / octomap_rotation_angles, 2 * math.pi / octomap_rotation_angles)
            group_current_pose = translate_pose_msg(group_current_pose, octomap_translations_x, octomap_translations_y, octomap_translations_z)
            oct_poses.append(copy.deepcopy(group_current_pose))
            g.trajectory.points.append(JointTrajectoryPoint(positions=Q[i], velocities=[0] * 6, time_from_start=rospy.Duration(0.0+i*3.0)))

         client.send_goal(g)
         client.wait_for_result()
         print("Octomap calibration positions:")
         print(g)
        except KeyboardInterrupt:
         rospy.signal_shutdown("KeyboardInterrupt")
         raise

    def initial_pose(self):
        pevent("Initial pose sequence started")
      #  group_name = "manipulator"
      #  ipdb.set_trace()
      #  group = moveit_commander.MoveGroupCommander(group_name, robot_description="/summit_xl/robot_description", ns="summit_xl")


        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = 1.07464909554
        pose_goal.position.y = 0.00558180361986
        pose_goal.position.z = 0.801929414272
        pose_goal.orientation.w = 0.504062771797
        pose_goal.orientation.x = -0.505350887775
        pose_goal.orientation.y = 0.478404045105
        pose_goal.orientation.z = 0.511537611485


        group.set_pose_target(pose_goal)

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        # group.go(joint_goal, wait=True)

        plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()

        group.clear_pose_targets()

if __name__ == "__main__":
    start_time = datetime.datetime.now()
    rospy.init_node("gpd_pick_and_place")

    pnp = GpdPickPlace(mark_pose=True)
 #   (position, velocity, effort) = call_return_joint_states(JOINT_NAMES)
 #   print("current position is:", pplist(position))
    #print("current velocity is:", pplist(velocity))
    #print("current effort is:", pplist(effort))

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name, robot_description="/summit_xl/robot_description",
                                                ns="/summit_xl")



# print "To build the initial octomap we make the robot move between the following poses:"
 #   print str([Q1[i] * 180. / pi for i in xrange(0, 6)])
  #  print str([Q2[i] * 180. / pi for i in xrange(0, 6)])
   # print str([Q3[i] * 180. / pi for i in xrange(0, 6)])
   # print str([Q4[i] * 180. / pi for i in xrange(0, 6)])
   # print str([Q5[i] * 180. / pi for i in xrange(0, 6)])
   # print str([Q6[i] * 180. / pi for i in xrange(0, 6)])
    print "Please make sure that your robot can move freely before proceeding!"
    inp = raw_input("Do you need to rebuild the octomap? y/n: ")[0]
    if (inp == 'y'):
        pnp.initial_octomap_building()
        print("Initial rotation for octomap building  performed")
    else:
        print("Initial rotation for octomap building NOT performed")
    # Start the ROS node
      # Set the value to the gripper
  #  print("--- Start Physical Arm ---")
 #



   # n = NiryoOne()

 #   print("Calibration started !")
    # Calibrate robot first
#try:
 #   n.calibrate_auto()
#except NiryoOneException as e:
#    print e

 #   print("Make sure calibration is already performed on arm !")


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
    num_objects = 1
    for i in range (0, num_objects):
        #ipdb.set_trace()
        # Subscribe for grasps
        print("--- Move Arm to Initial Position---")
        pnp.initial_pose()

         # Get the pointcloud from camera, filter it, extract indices and publish it to gpd CNN
        gpd_prep = GpdGrasps(max_messages=8)
        gpd_prep.filter_cloud()
        gpd_prep.publish_indexed_cloud()


        # Wait for grasps from gpd, wrap them into Grasp msg format and start picking
        selected_grasps = pnp.get_gpd_grasps()
        formatted_grasps = pnp.generate_grasp_msgs(selected_grasps)
        #n.open_gripper(TOOL_GRIPPER_3_ID, 200)
       # print("Gripper 3 opened")
       # result = gripper_client(0.2)
        result = gripper_client_2(8)
        print("Gripper opened")

        successful_grasp = pnp.pick(formatted_grasps, verbose=True)
        #n.close_gripper(TOOL_GRIPPER_3_ID, 200)
       # print("Gripper 3 closed")
       # result = gripper_client(0)
        result = gripper_client_2(-8)
        print("Gripper closed")
        #ipdb.set_trace()
        if successful_grasp is not None:

        # Place object with successful grasp pose as the starting point
            pnp.place2(successful_grasp)
      #  n.open_gripper(TOOL_GRIPPER_3_ID, 200)
    #    print("Gripper 3 opened")
     #   result = gripper_client(0.2)
            result = gripper_client_2(8)
            print("Gripper opened")
      #  n.close_gripper(TOOL_GRIPPER_3_ID, 200)
       # print("Gripper 3 closed")
   # pnp.place(successful_grasp)
  #  fix_grasp =  pnp.get_know_successful_grasp()
   # pnp.place(fix_grasp)
    pinfo("Demo runtime: " + str(datetime.datetime.now() - start_time))


