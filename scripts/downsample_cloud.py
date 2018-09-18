import rospy
import numpy as np
import pcl
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


class DownsampleCloud():
    input_cloud = []
    received_frames = 0
    max_frames = 8

    def __init__(self):
        rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.cloud_callback)
        self.pub = rospy.Publisher("/xtion/depth_registered/points_downsampled", PointCloud2, queue_size=10)

    def cloud_callback(self, msg):
        if self.received_frames < self.max_frames:
            for p in point_cloud2.read_points(msg, skip_nans=True):
                self.input_cloud.append([p[0], p[1], p[2]])
            self.received_frames += 1

    def downsample_and_publish(self):
        while self.received_frames < self.max_frames:  # wait until cloud_callback processes 8 messages
            pass

        downsampled_cloud = pcl.PointCloud()
        downsampled_cloud.from_array(np.asarray(self.input_cloud, dtype=np.float32))

        self.input_cloud = []  # clean input cloud
        self.received_frames = 0  # allow to copy next 8 msgs chunk

        pass_fill = downsampled_cloud.make_passthrough_filter()
        pass_fill.set_filter_field_name("z")
        pass_fill.set_filter_limits(0, 1)
        downsampled_cloud = pass_fill.filter()

        sor = downsampled_cloud.make_voxel_grid_filter()
        sor.set_leaf_size(0.005, 0.005, 0.005)
        downsampled_cloud = sor.filter()

        msg_header = Header()
        msg_header.frame_id = "xtion_rgb_optical_frame"
        self.pub.publish(point_cloud2.create_cloud_xyz32(msg_header, downsampled_cloud.to_list()))


if __name__ == '__main__':
    rospy.init_node("cloud_downsampling")
    a = DownsampleCloud()

    try:
        while True:
            a.downsample_and_publish()
    except (KeyboardInterrupt, SystemExit):
        print "Done"
