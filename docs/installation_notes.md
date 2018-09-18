sudo apt-get install software-properties-common
sudo apt install libpcl-dev
sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
sudo apt-get install --no-install-recommends libboost-all-dev
sudo apt-get install libgflags-dev
sudo apt-get install libgoogle-glog-dev
sudo apt-get install liblmdb-dev
sudo pip install pyquaternion
sudo apt-get install ros-kinetic-moveit-python

#Reqs

https://github.com/strawlab/python-pcl

#Pyassimp
sudo dpkg --remove --force-depends python-pyassimp
sudo pip install pyassimp==3.3



Makefile (caffe): LIBRARIES += glog gflags protobuf boost_system boost_filesystem m hdf5_hl hdf5 h
df5_serial hdf5_serial_hl
sudo apt-get install libhdf5-dev libhdf5-serial-dev 

# fix hdf5
https://github.com/NVIDIA/DIGITS/issues/156

sudo apt-get install libblas-dev libatlas-dev libatlas-base-dev

# fix caffe paths
--->> Build with cmake (not just make)

https://github.com/atenpas/gpd/issues/18
export CAFFE_DIR=/home/tof/workspace/catkin_ws_kinetic/src/caffe/build

# use multiple catkin workspaces
source /home/tof/workspace/catkin_ws_kinetic/devel/setup.bash
source /home/tof/workspace/tiago_public_ws/devel/setup.bash --extend
