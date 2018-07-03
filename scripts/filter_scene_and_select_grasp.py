import rospy
import numpy as np
import pcl
from tools import *
from scipy.linalg import lstsq
from std_msgs.msg import Header, Int64
from geometry_msgs.msg import Point
from plane_segm import filtering
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from gpd.msg import CloudIndexed
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Point the head using controller
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


class GpdGrasps(object):
    raw_cloud = []
    filtered_cloud = pcl.PointCloud()
    message_counter = 0

    def __init__(self, max_messages=8):
        self.max_messages = max_messages
        pevent("Waiting for pointcloud")
        rospy.sleep(3)
        rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.cloud_callback)

    def cloud_callback(self, msg):
        if self.message_counter < self.max_messages:
            self.message_counter += 1
            for p in point_cloud2.read_points(msg, skip_nans=True):
                self.raw_cloud.append([p[0], p[1], p[2]])

    def filter_cloud(self):
        # Wait for point cloud to arrive.
        while self.message_counter < self.max_messages:
            rospy.sleep(0.01)

        # Do filtering until filtering returns some points or user stops the script
        while self.filtered_cloud.size == 0:
            self.filtered_cloud = filtering(self.raw_cloud)

            self.raw_cloud = None

            if self.filtered_cloud.size == 0:
                perror("PointCloud after filtering is empty. Are you sure that object is visible for the robot? \nTrying again")

    def extract_indices(self):
        # Extract the nonplanar indices. Uses a least squares fit AX = b. Plane equation: z = ax + by + c.

        X = self.filtered_cloud.to_array()
        A = np.c_[X[:, 0], X[:, 1], np.ones(X.shape[0])]
        C, _, _, _ = lstsq(A, X[:, 2])
        a, b, c, d = C[0], C[1], -1., C[2]  # coefficients of the form: a*x + b*y + c*z + d = 0.
        dist = ((a * X[:, 0] + b * X[:, 1] + d) - X[:, 2]) ** 2
        # err = dist.sum()
        idx = np.where(dist > 0.001)

        return X, idx

    def generate_cloud_indexed_msg(self):
        np_cloud, idx = self.extract_indices()

        msg = CloudIndexed()
        header = Header()
        header.frame_id = "xtion_rgb_optical_frame"
        header.stamp = rospy.Time.now()
        msg.cloud_sources.cloud = point_cloud2.create_cloud_xyz32(header, np_cloud.tolist())
        msg.cloud_sources.view_points.append(Point(0, 0, 0))

        for i in xrange(np_cloud.shape[0]):
            msg.cloud_sources.camera_source.append(Int64(0))
        for i in idx[0]:
            msg.indices.append(Int64(i))

        return msg

    def publish_indexed_cloud(self):
        msg = self.generate_cloud_indexed_msg()

        pub = rospy.Publisher('cloud_indexed', CloudIndexed, queue_size=1, latch=True)
        pub.publish(msg)
        rospy.sleep(3.14)
        pevent('Published cloud with ' + str(len(msg.indices)) + ' indices')
        pub.unregister()
