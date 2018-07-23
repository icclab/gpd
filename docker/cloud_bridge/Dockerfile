FROM ros:kinetic

ARG REPO_USER
ARG REPO_PASS

# install python dependencies
RUN apt-get update && apt-get install -y python-pip libsnappy* git ros-kinetic-actionlib-msgs

# install bridge dependencies
RUN pip install amqp==2.2.0 defer kombu==4.0.2 eventlet futures sortedcontainers python-snappy --upgrade python pip

# add user and make sure her stuff is writable whichever userid is given at runtime
# ref: http://blog.dscpl.com.au/2015/12/random-user-ids-when-running-docker.html
RUN adduser --disabled-password --gid 0 --gecos "ROS user" rosuser
RUN mkdir -p /home/rosuser/.ros/log
RUN chown rosuser:root /home/rosuser && chmod -R 0775 /home/rosuser
WORKDIR /home/rosuser
ENV HOME /home/rosuser

# prevent log writing permission errors
RUN chmod -R g+rwx /home/rosuser/.ros/log  
RUN chgrp -R root /home/rosuser/.ros/log 
RUN chown rosuser:root /home/rosuser/.ros/log && chmod -R 0775 /home/rosuser/.ros/log

USER rosuser

# Initialize rosdep & create a catkin workspace
RUN rosdep update \
	&& mkdir -p catkin_ws/src && cd catkin_ws/src \
	&& /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_init_workspace"

# clone the bridge and the bride demo node
RUN git clone https://$REPO_USER:$REPO_PASS@bitbucket.org/rapyutians/cloud_bridge.git catkin_ws/src/cloud_bridge
RUN cd catkin_ws/src/cloud_bridge; git fetch --tags; git checkout tags/4.1.17;

# copy cacert
#COPY testca catkin_ws/src/cloud_bridge/rr_cloud_bridge/config/testca

# build the bridge
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd catkin_ws/; catkin_make'
# RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd catkin_ws/; catkin_make install'

# source setup bash
RUN echo "source /home/rosuser/catkin_ws/devel/setup.bash" >> /home/rosuser/.bashrc

# run the bridge
CMD ["/bin/bash", "-c", "source catkin_ws/devel/setup.bash && roslaunch rr_cloud_bridge rr_cloud_bridge.launch"]
