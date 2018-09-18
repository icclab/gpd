import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tools import pevent


class RobotPreparation(object):
    def __init__(self):
        # TODO: why it only works with latched topic?
        self.head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1, latch=True)
        self.torso_cmd = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1, latch=True)
        # self.play_m_as = SimpleActionClient('play_motion', PlayMotionAction)
        # if not self.play_m_as.wait_for_server(rospy.Duration(20.0)):
        #     perror("Could not connect to /play_motion AS")
        #     exit(1)

    def look_down(self):
        pevent("Moving head")
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0, -0.7]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
        pevent("Done")

    def lift_torso(self):
        pevent("Moving torso up")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.34]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        self.torso_cmd.publish(jt)
        rospy.sleep(5)
        pevent("Done")

    # def unfold_arm(self):
    #     pevent("Unfolding arm")
    #     pmg = PlayMotionGoal()
    #     pmg.motion_name = 'pregrasp'
    #     pmg.skip_planning = False
    #     self.play_m_as.send_goal_and_wait(pmg)
    #     pevent("Done.")