import sys
import subprocess
import rospy
import pcl
import actionlib
from plane_segm import filterCloud
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from gpd.msg import CloudIndexed
from control_msgs.msg import PointHeadAction, PointHeadGoal


# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()


i = 0
rawCloud = []  # global variable to store the point cloud


def cloudCallback(msg):
    global rawCloud
    global i
    # if len(rawCloud) == 0:
    if i < 8:
        i += 1
        for p in point_cloud2.read_points(msg, skip_nans=True):
            rawCloud.append([p[0], p[1], p[2]])


rospy.init_node('select_grasp')

head = PointHeadClient()
print "Moving head"
head.look_at(2, 0.0, 0.0, "base_link")

# Subscribe to the ROS topic that contains the grasps.
cloud_sub = rospy.Subscriber("/head_camera/depth_downsample/points", PointCloud2, cloudCallback)

# Wait for point cloud to arrive.
while i < 8:
    rospy.sleep(0.01)

cloud = pcl.PointCloud()
cloud = filterCloud(rawCloud)

p = subprocess.Popen(
    ['rosrun', 'pcl_ros', 'pcd_to_pointcloud', 'obstacles.pcd', '_frame_id:=head_camera_rgb_optical_frame'])

# Extract the nonplanar indices. Uses a least squares fit AX = b. Plane equation: z = ax + by + c.
import numpy as np
from scipy.linalg import lstsq

cloud2 = cloud.to_array()
X = cloud2
A = np.c_[X[:, 0], X[:, 1], np.ones(X.shape[0])]
C, _, _, _ = lstsq(A, X[:, 2])
a, b, c, d = C[0], C[1], -1., C[2]  # coefficients of the form: a*x + b*y + c*z + d = 0.
dist = ((a * X[:, 0] + b * X[:, 1] + d) - X[:, 2]) ** 2
err = dist.sum()
idx = np.where(dist > 0.001)

from std_msgs.msg import Header, Int64
from geometry_msgs.msg import Point

pub = rospy.Publisher('cloud_indexed', CloudIndexed, queue_size=1, latch=True)
msg = CloudIndexed()
header = Header()
header.frame_id = "head_camera_rgb_optical_frame"
header.stamp = rospy.Time.now()
msg.cloud_sources.cloud = point_cloud2.create_cloud_xyz32(header, cloud2.tolist())
msg.cloud_sources.view_points.append(Point(0, 0, 0))

for i in xrange(cloud2.shape[0]):
    msg.cloud_sources.camera_source.append(Int64(0))
for i in idx[0]:
    msg.indices.append(Int64(i))
rospy.sleep(3.14)
pub.publish(msg)
print 'Published cloud with', len(msg.indices), 'indices'
print 'Searching for grasps...'

# Select a grasp for the robot to execute.
from gpd.msg import GraspConfigList

grasps = []  # global variable to store grasps


def callback(msg):
    global grasps
    grasps = msg.grasps


# Subscribe to the ROS topic that contains the grasps.
grasps_sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback)

# Wait for grasps to arrive.
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    if len(grasps) > 0:
        rospy.loginfo('Received %d grasps.', len(grasps))
        break

grasp = grasps[0]  # grasps are sorted in descending order by score
print 'Selected grasp with score:', grasp.score
