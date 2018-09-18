Branch currently contains WIP demo for Tiago robot.
For Fetch demo version please use the **fetch-demo** branch.
The **paper-devel** branch contains demo used for the paper.

---
# How to start Tiago pick&place demo

* run `python gripper_hack.py` from gpd/scripts
* `roslaunch gpd tiago_playground.launch` (runs basic version of the demo, check another tiago_playground*.launch files)
* run `python pick_and_place.py` from gpd/scripts

# Using downsampled pointcloud:
* additionally run `downsample_cloud.py` script which aggregates 8 frames from `/xtion/depth_registered/points` topic, downsamples them and then publishes data on the `/xtion/depth_registered/points_downsampled` rostopic
* modify the pointcloud subscriber in the **22** LOC of `gpd_controller.py` to use the downsampled pointcloud topic
* `pick_and_place.py` - in **247** LOC set the `max_messages` argument to 1
* *TODO: Add option to pass the information about used version of the pointcloud to the script as an argument instead of modifying the code*

# Tricks:
* package tiago_gazebo starts **joystick_relay** node by default, which consumes a lot of CPU resources and is not used in the demo.
You can comment out **twist_mux** in `/tiago_gazebo/launch/simulation_tiago_bringup.launch`
* `perf.sh` script can be used for logging the CPU utilisation of multiple processes at once
* *cloud grasping* paper contains some useful information about the demo structure

# Grasp Pose Detection (GPD)

* **Author:** Andreas ten Pas (atp@ccs.neu.edu)
* **Version:** 1.0.0
* **Author's website:** [http://www.ccs.neu.edu/home/atp/](http://www.ccs.neu.edu/home/atp/)
* **License:** BSD


## 1) Overview

This package detects 6-DOF grasp poses for a 2-finger grasp (e.g. a parallel jaw gripper) in 3D point clouds.

<!-- <img src="readme/examples.png" alt="" style="width: 400px;"/> -->

Grasp pose detection consists of three steps: sampling a large number of grasp candidates, classifying these candidates 
as viable grasps or not, and clustering viable grasps which are geometrically similar.

The reference for this package is: [High precision grasp pose detection in dense clutter](http://arxiv.org/abs/1603.01564).

### UR5 Video

<a href="http://www.youtube.com/watch?feature=player_embedded&v=kfe5bNt35ZI
" target="_blank"><img src="http://img.youtube.com/vi/y7z-Yn1PQNI/0.jpg" 
alt="UR5 demo" width="640" height="480" border="0" /></a>


## 2) Requirements

1. [PCL 1.7 or later](http://pointclouds.org/)
2. [Eigen 3.0 or later](https://eigen.tuxfamily.org)
3. [Caffe](http://caffe.berkeleyvision.org/)
4. <a href="http://wiki.ros.org/indigo" style="color:blue">ROS Indigo</a> <span style="color:blue">and Ubuntu 
14.04</span> *or* <a href="http://wiki.ros.org/kinetic" style="color:orange">ROS Kinetic</a> 
<span style="color:orange">and Ubuntu 16.04</span>


## 3) Prerequisites

The following instructions work for **Ubuntu 14.04** or **Ubuntu 16.04**. Similar instructions should work for other 
Linux distributions that support ROS.

 1. Install Caffe [(Instructions)](http://caffe.berkeleyvision.org/installation.html). Follow the 
[CMake Build instructions](http://caffe.berkeleyvision.org/installation.html#cmake-build). **Notice for Ubuntu 14.04:** 
Due to a conflict between the Boost version required by Caffe (1.55) and the one installed as a dependency with the 
Debian package for ROS Indigo (1.54), you need to checkout an older version of Caffe that worked with Boost 1.54. So, 
when you clone Caffe, please use this command.
   
    ```
    git clone https://github.com/BVLC/caffe.git && cd caffe
    git checkout 923e7e8b6337f610115ae28859408bc392d13136
    ```

2. Install ROS. In Ubuntu 14.04, install ROS Indigo [(Instructions)](http://wiki.ros.org/indigo/Installation/Ubuntu). 
In Ubuntu 16.04, install ROS Kinetic [(Instructions)](http://wiki.ros.org/kinetic/Installation/Ubuntu).


3. Clone the [grasp_pose_generator](https://github.com/atenpas/gpg) repository into some folder:

   ```
   cd <location_of_your_workspace>
   git clone https://github.com/atenpas/gpg.git
   ```

4. Build and install the *grasp_pose_generator*: 

   ```
   cd gpg
   mkdir build && cd build
   cmake ..
   make
   sudo make install
   ```


## 4) Compiling GPD

1. Clone this repository.
   
   ```
   cd <location_of_your_workspace/src>
   git clone https://github.com/atenpas/gpd.git
   ```

2. Make sure that you linked your local caffe build in CMakeLists.txt
   Something like:

   ```
   SET(CAFFE_DIR "/home/fitter/Documents/caffe/build")
   ```

3. Build your catkin workspace.

   ```
   cd <location_of_your_workspace>
   catkin_make
   ```


## 5) Generate Grasps for a Point Cloud File

Launch the grasp pose detection on an example point cloud:
   
   ```
   roslaunch gpd tutorial0.launch
   ```
Within the GUI that appears, press r to center the view, and q to quit the GUI and load the next visualization.
The output should look similar to the screenshot shown below.

![rviz screenshot](readme/file.png "Grasps visualized in PCL")


## 6) Tutorials

1. [Detect Grasps With an RGBD camera](tutorials/tutorial_1_grasps_camera.md)
2. [Detect Grasps on a Specific Object](tutorials/tutorial_2_grasp_select.md)


## 7) Parameters

Brief explanations of parameters are given in *launch/classify_candidates_file_15_channels.launch* for using PCD files. 
For use on a robot, see *launch/ur5_15_channels.launch*. The two parameters that you typically want to play with to 
improve on then number of grasps found are *workspace* and *num_samples*. The first defines the volume of space in which 
to search for grasps as a cuboid of dimensions [minX, maxX, minY, maxY, minZ, maxZ], centered at the origin. The second 
is the number of samples that are drawn from the point cloud to detect grasps. You should set the workspace as small as 
possible and the number of samples as large as possible. 


## 8) Views

![rviz screenshot](readme/views.png "Single View and Two Views")

You can use this package with a single or with two depth sensors. The package comes with weight files for Caffe 
for both options. You can find these files in *gpd/caffe/15channels*. For a single sensor, use 
*single_view_15_channels.caffemodel* and for two depth sensors, use *two_views_15_channels_[angle]*. The *[angle]* is 
the angle between the two sensor views, as illustrated in the picture below. In the two-views setting, you want to 
register the two point clouds together before sending them to GPD.

![rviz screenshot](readme/view_angle.png "Angle Between Sensor Views")

To switch between one and two sensor views, change the parameter *trained_file* in the launch file 
*launch/caffe/ur5_15channels.launch*.


## 9) Input Channels for Neural Network

The package comes with weight files for two different input representations for the neural network that is used to 
decide if a grasp is viable or not: 3 or 15 channels. The default is 15 channels. However, you can use the 3 channels 
to achieve better runtime for a loss in grasp quality. For more details, please see the reference below.


## 10) Citation

If you like this package and use it in your own work, please cite our paper(s):

[1] Andreas ten Pas, Marcus Gualtieri, Kate Saenko, and Robert Platt. [**Grasp Pose Detection in Point 
Clouds**](http://arxiv.org/abs/1706.09911). The International Journal of Robotics Research, Vol 36, Issue 13-14, 
pp. 1455 - 1473. October 2017.

[2] Marcus Gualtieri, Andreas ten Pas, Kate Saenko, and Robert Platt. [**High precision grasp pose detection in dense 
clutter**](http://arxiv.org/abs/1603.01564). IROS 2016. 598-605.


## 11) Troubleshooting

* GCC 4.8: The package [might not compile](https://github.com/atenpas/gpd/issues/14#issuecomment-324789077) with 
GCC 4.8. This is due to [a bug](https://gcc.gnu.org/bugzilla/show_bug.cgi?id=58251) in GCC. **Solution:** Upgrade to 
GCC 4.9. 

* During `catkin_make`, you get this error: *[...]/caffe/include/caffe/util/cudnn.hpp:8:34: fatal error: caffe/proto/caffe.pb.h: No such file or directory*. **Solution ([source](https://github.com/muupan/dqn-in-the-caffe/issues/3)):**
    ```
    # In the directory you installed Caffe to
    protoc src/caffe/proto/caffe.proto --cpp_out=.
    mkdir include/caffe/proto
    mv src/caffe/proto/caffe.pb.h include/caffe/proto
    ```
