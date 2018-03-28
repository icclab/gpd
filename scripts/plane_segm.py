# -*- coding: utf-8 -*-
import pcl
import numpy as np
import subprocess
import vtk
from tools import *


def filter_cloud(rawCloud):
    # cloud = pcl.load(str(arg1))
    cloud = pcl.PointCloud()
    obstacles_cloud = pcl.PointCloud()
    cloud.from_array(np.asarray(rawCloud, dtype=np.float32))
    pevent("Loaded PointCloud with: " + str(cloud.size) + " points.")

    pass_fill = cloud.make_passthrough_filter()
    pass_fill.set_filter_field_name("z")
    pass_fill.set_filter_limits(0, 2)
    cloud = pass_fill.filter()
    pinfo("PointCloud after max range filtering has: " + str(cloud.size) + " points.")

    stat_fill = cloud.make_statistical_outlier_filter()
    stat_fill.set_mean_k(50)
    stat_fill.set_std_dev_mul_thresh(1.0)
    cloud = stat_fill.filter()
    pinfo("PointCloud after outliers filtering has: " + str(cloud.size) + " points.")

    seg = cloud.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_normal_distance_weight(0.1)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_max_iterations(1000)
    seg.set_distance_threshold(0.01)
    indices, model = seg.segment()
    obstacles_cloud = cloud.extract(indices, negative=False)
    cloud = cloud.extract(indices, negative=True)
    pinfo("PointCloud after plane filtering has: " + str(cloud.size) + " points.")

    pcl.save(cloud, "objects.pcd")
    pcl.save(obstacles_cloud, "obstacles.pcd")

    # Make a mesh from object pointcloud and save it to stl file to load if from moveit side
    create_mesh_and_save(cloud)

    # Publish cloud with extracted obstacles to create octomap
    subprocess.Popen(
        ['rosrun', 'pcl_ros', 'pcd_to_pointcloud', 'obstacles.pcd', 'cloud_pcd', '_frame_id:=head_camera_rgb_optical_frame'])
    subprocess.Popen(
        ['rosrun', 'pcl_ros', 'pcd_to_pointcloud', 'objects.pcd', 'cloud_pcd2', '_frame_id:=head_camera_rgb_optical_frame'])

    return cloud


def create_mesh_and_save(cloud):
    filename = "object.stl"

    vtk_points = vtk.vtkPoints()
    np_cloud = np.asarray(cloud)

    for i in range(0, np_cloud.shape[0]):
        vtk_points.InsertPoint(i, np_cloud[i][0], np_cloud[i][1], np_cloud[i][2])

    profile = vtk.vtkPolyData()
    profile.SetPoints(vtk_points)

    delny = vtk.vtkDelaunay2D()
    delny.SetInputData(profile)
    delny.SetTolerance(0.001)

    stlWriter = vtk.vtkSTLWriter()
    stlWriter.SetFileName(filename)
    stlWriter.SetFileName(filename)
    stlWriter.SetInputConnection(delny.GetOutputPort())
    stlWriter.Write()
