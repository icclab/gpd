#!/usr/bin/env python

import rospy
from tools import *
from pprint import pprint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient, SimpleActionServer


class HackyGripperController:
    def __init__(self):
        rospy.init_node('gripper_controller')

        self.server = SimpleActionServer('/gripper_controller/hacky_gripper_controller', FollowJointTrajectoryAction,
                                         execute_cb=self.gripper_callback, auto_start=False)
        self.server.start()

        self.play_m_as = SimpleActionClient('play_motion', PlayMotionAction)

        while not self.play_m_as.wait_for_server():
            perror("Could not connect to /play_motion AS")
            rospy.sleep(1)

        pevent("Hacky Gripper Controller: Initialized")

        self.pmg = PlayMotionGoal()

        rospy.spin()

    def gripper_callback(self, goal):
        # this can be useful:
        # https://github.com/pal-robotics/pal_gripper/blob/indigo-devel/pal_gripper_controller_configuration/scripts/home_gripper.py

        # Pick'nPlace API wants to close the gripper, so do it
        if goal.trajectory.points[1].positions[0] == 0.0 and goal.trajectory.points[0].positions[0] > 0.04:
            pevent("Closing the gripper")
            self.pmg.motion_name = 'close_gripper'

        # open the gripper
        elif goal.trajectory.points[0].positions[0] != 0.0:
            pevent("Opening the gripper")
            self.pmg.motion_name = 'open_gripper'

        if len(self.pmg.motion_name) > 0:
            self.pmg.skip_planning = False
            self.play_m_as.send_goal_and_wait(self.pmg)

            self.server.set_succeeded()
            pinfo("Hacky Gripper Controller: Done.")


if __name__ == '__main__':
    try:
        HackyGripperController()
    except rospy.ROSInterruptException:
        rospy.loginfo('Do widzenia')
