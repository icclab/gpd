# Please build this image with --squash to prevent leaving SSH keys in history
FROM osrf/ros:kinetic-desktop-full-xenial


# Basic Utilities
RUN apt-get -y update && apt-get install -y screen sudo ssh


# Use bash instead of sh
RUN rm /bin/sh && ln -s /bin/bash /bin/sh


# Latest X11 / mesa GL (probably needed for Gazebo)
#RUN apt-get install -y\
#  xserver-xorg-dev-lts-wily\
#  libegl1-mesa-dev-lts-wily\
#  libgl1-mesa-dev-lts-wily\
#  libgbm-dev-lts-wily\
#  mesa-common-dev-lts-wily\
#  libgles2-mesa-lts-wily\
#  libwayland-egl1-mesa-lts-wily #\
#  libopenvg1-mesa


# The rest of ROS-desktop
RUN apt-get install -y ros-kinetic-desktop-full


# Additional development tools
RUN apt-get install -y \
	x11-apps \
	python-pip \
	build-essential


# Moveit and arm related stuff
RUN apt-get install -y \
	ros-kinetic-ros-control \
	ros-kinetic-ros-controllers \
	ros-kinetic-gazebo-ros-pkgs \
	ros-kinetic-gazebo-ros-control \
	ros-kinetic-moveit-visual-tools \
	ros-kinetic-moveit \
	ros-kinetic-controller-manager


# Install and build Eigen
RUN cd /opt && \
	wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz -O eigen3.tgz && \
	tar zxvf eigen3.tgz && cd eigen-eigen-5a0156e40feb && \
	mkdir build && cd build && \
	cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=RELEASE .. && \
	make install


# Caffe prequisities
RUN apt-get install -y --no-install-recommends \
        cmake \
        git \
        wget \
        libatlas-base-dev \
        libboost-all-dev \
        libgflags-dev \
        libgoogle-glog-dev \
        libhdf5-serial-dev \
        libleveldb-dev \
        liblmdb-dev \
        libopencv-dev \
        libprotobuf-dev \
        libsnappy-dev \
        protobuf-compiler \
        python-dev \
        python-numpy \
        python-setuptools \
	python-scipy


# Setup and build Caffe (CPU only mode)
ENV CAFFE_ROOT=/opt/caffe
WORKDIR $CAFFE_ROOT
ENV CLONE_TAG=1.0

RUN git clone -b ${CLONE_TAG} --depth 1 https://github.com/BVLC/caffe.git . && \
    cd python && for req in $(cat requirements.txt) pydot; do pip install $req; done && cd .. && \
    mkdir build && cd build && \
    cmake -DCPU_ONLY=1 .. && \
    make -j4

RUN protoc src/caffe/proto/caffe.proto --cpp_out=. && \
	mkdir include/caffe/proto && \
	mv src/caffe/proto/caffe.pb.h include/caffe/proto

ENV PYCAFFE_ROOT $CAFFE_ROOT/python
ENV PYTHONPATH $PYCAFFE_ROOT:$PYTHONPATH
ENV PATH $CAFFE_ROOT/build/tools:$PYCAFFE_ROOT:$PATH
RUN echo "$CAFFE_ROOT/build/lib" >> /etc/ld.so.conf.d/caffe.conf && ldconfig


# PCL
RUN apt-get install -y libpcl-dev

# Python PCL
RUN pip install cython==0.25.2 numpy
RUN git clone --branch master --depth 1 https://github.com/strawlab/python-pcl.git && \
	cd python-pcl && \
	python setup.py build_ext -i && \
	python setup.py install


# Build and install GPG
RUN cd /opt && \
	git clone --depth 1 https://github.com/atenpas/gpg.git && \
	cd gpg && \
	mkdir build && cd build && \
	cmake .. && \
	make && \
	sudo make install


# Create catkin_ws and initialize it
WORKDIR /
RUN source /opt/ros/kinetic/setup.bash && \
	mkdir -p catkin_ws/src && \
	cd catkin_ws && \
	catkin_make && \
	source /catkin_ws/devel/setup.bash


# GPD + add and remove private key to clone it (BUILD THIS DOCKER IMAGE WITH --squash TO REMOVE IT FROM HISTORY)
ARG SSH_PRIVATE_KEY
RUN mkdir /root/.ssh/ && \
	echo "${SSH_PRIVATE_KEY}" > /root/.ssh/id_rsa && \
	chmod 600 /root/.ssh/id_rsa
RUN touch /root/.ssh/known_hosts && \
	chmod 700 /root/.ssh && \
	ssh-keyscan srv-lab-t-401.zhaw.ch >> /root/.ssh/known_hosts
RUN cd catkin_ws/src && \
	git clone git@srv-lab-t-401.zhaw.ch:cloudrobotics/gpd.git && \
	cd .. && source /opt/ros/kinetic/setup.bash && \
	catkin_make
RUN rm -dr /root/.ssh/
