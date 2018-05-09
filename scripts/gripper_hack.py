#!/usr/bin/env python

import rospy
from tools import *
from pprint import pprint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient, SimpleActionServer

# so I'm able now to send a modified goal (with 90* rotated gripper joint) just after the original goal, but
# my message is somehow ignored. My "set succeeded" to old goal does nothing and arm performs original-wrong movement.
# Possible fix:
# TODO: cancel original goal and set new one using full action server impl instead of publishing modified goal to topic


class HackyGripperController:
    last_arm_goal = ""
    is_it_second_callback = False

    def __init__(self):
        rospy.init_node('gripper_controller')

        self.server = SimpleActionServer('~hacky_gripper_controller', FollowJointTrajectoryAction,
                                         execute_cb=self.gripper_callback, auto_start=False)
        self.server.start()

        self.play_m_as = SimpleActionClient('play_motion', PlayMotionAction)

        rospy.Subscriber("/arm_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, self.arm_callback)
        self.arm_pub = rospy.Publisher("/arm_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=5)

        while not self.play_m_as.wait_for_server(rospy.Duration(20.0)):
            perror("Could not connect to /play_motion AS")

        pevent("Hacky Gripper Controller: Initialized")

        rospy.spin()

    def arm_callback(self, msg):
        if self.is_it_second_callback:
            self.last_arm_goal = msg
            self.is_it_second_callback = False  # do it only once

            new_arm_trajectory = FollowJointTrajectoryActionGoal()
            pevent("Rotating the wrist")
            new_arm_trajectory = self.last_arm_goal
            for i, point in enumerate(
                    self.last_arm_goal.goal.trajectory.points):  # point.positions[6] contains gripper wrist joint position
                tmp = list(point.positions)
                tmp[6] = self.mod_wrist_pose(point.positions[6])
                new_arm_trajectory.goal.trajectory.points[i].positions = tmp
            pprint(new_arm_trajectory)
            self.arm_pub.publish(new_arm_trajectory)


    def gripper_callback(self, goal):
        # this can be useful:
        # https://github.com/pal-robotics/pal_gripper/blob/indigo-devel/pal_gripper_controller_configuration/scripts/home_gripper.py
        pmg = PlayMotionGoal()

        # Pick'nPlace API wants to close the gripper, so do it
        if goal.trajectory.points[1].positions[0] == 0.0 and goal.trajectory.points[0].positions[0] > 0.04:
            pevent("Closing the gripper")
            pmg.motion_name = 'close_gripper'

        # gripper is in pregrasp position
        elif goal.trajectory.points[1].positions[0] == 0.1337:
            if self.is_it_second_callback == False:  # need to wait until second callback to have updated arm goal values
                self.is_it_second_callback = True
                pinfo("First rotate wrist cb")
                self.server.set_succeeded()
                return

        # open the gripper
        elif goal.trajectory.points[0].positions[0] != 0.0:
            pevent("Opening the gripper")
            pmg.motion_name = 'open_gripper'

        pmg.skip_planning = False
        self.play_m_as.send_goal_and_wait(pmg)

        self.server.set_succeeded()
        pinfo("Hacky Gripper Controller: Done.")

    def mod_wrist_pose(self, pose):
        new_pose = pose + 1.5707963
        if new_pose > 2.07:  # max joint rotation angle
            new_pose = 2.07  # FIXME: do it properly, man...

        return  new_pose


if __name__ == '__main__':
    try:
        HackyGripperController()
    except rospy.ROSInterruptException:
        rospy.loginfo('Do widzenia')
