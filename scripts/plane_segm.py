# -*- coding: utf-8 -*-
import pcl
import numpy as np
import subprocess

from tools import *


def filter_cloud(rawCloud):
    # cloud = pcl.load(str(arg1))
    cloud = pcl.PointCloud()
    obstaclesCloud = pcl.PointCloud()
    cloud.from_array(np.asarray(rawCloud, dtype=np.float32))
    pevent("Loaded PointCloud with: " + str(cloud.size) + " points.")

    passFill = cloud.make_passthrough_filter()
    passFill.set_filter_field_name("z")
    passFill.set_filter_limits(0, 2)
    cloud = passFill.filter()
    pinfo("PointCloud after max range filtering has: " + str(cloud.size) + " points.")

    statFill = cloud.make_statistical_outlier_filter()
    statFill.set_mean_k(50)
    statFill.set_std_dev_mul_thresh(1.0)
    cloud = statFill.filter()
    pinfo("PointCloud after outliers filtering has: " + str(cloud.size) + " points.")

    seg = cloud.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_normal_distance_weight(0.1)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_max_iterations(1000)
    seg.set_distance_threshold(0.01)
    indices, model = seg.segment()
    obstaclesCloud = cloud.extract(indices, negative=False)
    cloud = cloud.extract(indices, negative=True)
    pinfo("PointCloud after plane filtering has: " + str(cloud.size) + " points.")

    pcl.save(cloud, "objects.pcd")
    pcl.save(obstaclesCloud, "obstacles.pcd")
    # Publish cloud with extracted obstacles to create octomap
    subprocess.Popen(
        ['rosrun', 'pcl_ros', 'pcd_to_pointcloud', 'obstacles.pcd', '_frame_id:=head_camera_rgb_optical_frame'])

    return cloud
