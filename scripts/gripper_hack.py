#!/usr/bin/env python

import rospy
from tools import *
from pprint import pprint
from control_msgs.msg import FollowJointTrajectoryAction
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient, SimpleActionServer


class HackyGripperController:
   
    def __init__(self):
        rospy.init_node('gripper_controller')

        self.server = SimpleActionServer('~hacky_gripper_controller', FollowJointTrajectoryAction,
                                         execute_cb=self.action_cb, auto_start=False)
        self.server.start()

        self.play_m_as = SimpleActionClient('play_motion', PlayMotionAction)

        while not self.play_m_as.wait_for_server(rospy.Duration(20.0)):
            perror("Could not connect to /play_motion AS")

        pevent("Hacky Gripper Controller: Initialized")

        rospy.spin()

    def action_cb(self, goal):
        # this can be usefull:
        # https://github.com/pal-robotics/pal_gripper/blob/indigo-devel/pal_gripper_controller_configuration/scripts/home_gripper.py
        pmg = PlayMotionGoal()
        if goal.trajectory.points[0].positions[0] > 0.04:
            pevent("Closing the gripper")
            pmg.motion_name = 'close_gripper'
        else:
            pevent("Opening the gripper")
            pmg.motion_name = 'open_gripper'

        pmg.skip_planning = False
        self.play_m_as.send_goal_and_wait(pmg)

        self.server.set_succeeded()
        rospy.sleep(5.0)
        pinfo("Hacky Gripper Controller: Done.")


if __name__ == '__main__':
    try:
        HackyGripperController()
    except rospy.ROSInterruptException:
        rospy.loginfo('Do widzenia')
        


